#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use stm32f103_aircraft::hardware::{
    key::{self, disabled_flight_control, enbaled_flight_control, is_flight_control_enbaled},
    led::Led,
    mpu6050::{self, AttitudeAngle, Mpu6050},
    nrf24l01::{self},
    pid::MotorPidController,
    tb6612fng::{self, Tb6612fng},
    timer, usart,
};

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use defmt::println;
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    prelude::{_stm32_hal_gpio_GpioExt, _stm32_hal_rcc_RccExt},
    timer::{SysDelay, SysTimerExt},
};

// 定义应用程序资源和任务
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true,dispatchers = [EXTI2,EXTI3])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        delay: SysDelay,
        key: key::Key,
        led: Led,
        usart: usart::Usart,
        tb6612fng: Tb6612fng,
        pid_controller: MotorPidController,
    }

    #[local]
    struct Local {
        // mpu6050_s: Sender<'static, Mpu6050Data, MPU6050_CAPACITY>,
        // nrf24l01_s: Sender<'static, NRF24L01Cmd, NRF24L01_CAPACITY>,
        // nrf24l01_rx: RxTY,
        mpu6050: Mpu6050,
    }

    // 初始化函数
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // 获取外设实例
        let mut afio = ctx.device.AFIO.constrain();
        let mut flash = ctx.device.FLASH.constrain();
        let rcc = ctx.device.RCC.constrain();
        let i2c2 = ctx.device.I2C2;
        let mut exti = ctx.device.EXTI;
        let spi1 = ctx.device.SPI1;
        let tim2 = ctx.device.TIM2;
        let tim3 = ctx.device.TIM3;
        let usart1 = ctx.device.USART1;

        let syst = ctx.core.SYST;

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();

        // 初始化时钟
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        // 具有自定义精度的阻塞延迟
        let mut delay = syst.delay(&clocks);

        delay.delay_ms(1000_u16);
        println!("init start ...");

        // 禁用 jtag 端口进行复用
        let (_pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // 初始化按键 KEY
        let key = key::Key::new(gpiob.pb0, &mut gpiob.crl, &mut exti, &mut afio);
        // 初始化 LED 灯
        let led = Led::new(gpiob.pb1, &mut gpiob.crl);
        // 初始化 USART1 串口
        let usart = usart::Usart::new(
            gpioa.pa9,
            gpioa.pa10,
            &mut gpioa.crh,
            usart1,
            &mut afio.mapr,
            &clocks,
        );
        // 初始化定时器
        timer::init_timer(tim3, &clocks);

        // 初始化 MPU6050 传感器
        let mpu6050 = mpu6050::Mpu6050::new(
            i2c2,
            gpiob.pb10,
            gpiob.pb11,
            &mut gpiob.crh,
            &mut delay,
            clocks,
        );
        // 初始化 TB6612FNG 电机驱动
        let tb6612fng = tb6612fng::Tb6612fng::new(tb6612fng::Config {
            pa0: gpioa.pa0,
            pa1: gpioa.pa1,
            pa2: gpioa.pa2,
            pa3: gpioa.pa3,
            pa4: gpioa.pa4,
            pa5: gpioa.pa5,
            pa6: gpioa.pa6,
            pa7: gpioa.pa7,
            crl: &mut gpioa.crl,
            tim2,
            mapr: &mut afio.mapr,
            clocks: &clocks,
        });
        // 初始化电机 PID 姿态计算模块
        let pid_controller = MotorPidController::new();

        // 初始化 NRF24L01 2.4 GHz 无线通信
        let nrf24l01 = nrf24l01::Nrf24L01::new(nrf24l01::Config {
            spi_sck: pb3,
            spi_miso: pb4,
            spi_mosi: gpiob.pb5,
            nrf24_ce: gpiob.pb6,
            nrf24_csn: gpiob.pb7,
            crl: &mut gpiob.crl,
            spi1,
            mapr: &mut afio.mapr,
            clocks,
        });
        let _nrf24l01_rx = nrf24l01.nrf24.rx().unwrap();

        println!("init end ...");
        (
            Shared {
                delay,
                key,
                led,
                usart,
                tb6612fng,
                pid_controller,
            },
            Local { mpu6050 },
        )
    }

    /// 飞控开关, 按键中断
    /// 开启/关闭飞控
    /// 点灯/熄灯
    #[task(binds = EXTI9_5, local = [], shared=[key,led])]
    fn key_click_irq(ctx: key_click_irq::Context) {
        println!("key_click_irq");
        let key = ctx.shared.key;
        let led = ctx.shared.led;
        (key, led).lock(|key, led| {
            key.clear_interrupt_pending_bit();

            // 如果是开启状态, 则关闭飞控及熄灯
            if is_flight_control_enbaled() {
                // 熄灯
                led.off();
                // 关闭飞控
                disabled_flight_control();
                return;
            }

            // 点灯
            led.off();
            // 开启飞控
            enbaled_flight_control();
        });
    }

    /// TIM3 定时中断
    /// 50ms过期中断
    #[task(binds = TIM3, local = [mpu6050], shared=[usart,tb6612fng])]
    fn timer_irq(ctx: timer_irq::Context) {
        // 获取姿态角
        let angle = ctx.local.mpu6050.get_acc_angles();

        // USART1 串口指令
        usart_cmd::spawn(angle).unwrap();
    }

    /// USART1 串口指令
    #[task(priority = 1, shared=[usart,tb6612fng,pid_controller])]
    async fn usart_cmd(ctx: usart_cmd::Context, angle: AttitudeAngle) {
        let tb6612fng = ctx.shared.tb6612fng;
        let usart = ctx.shared.usart;
        let pid_controller = ctx.shared.pid_controller;
        (usart, tb6612fng, pid_controller).lock(|usart, tb6612fng, pid_controller| {
            // 接收 USART1 串口数据
            let usart_resp = usart.recv_string();
            let signal_value = match usart_resp {
                Ok(v) => v,
                Err(_err) => return,
            };
            if signal_value.is_empty() {
                return;
            }
            // set 1=50 or get 1
            // let cmd_value: &[&str] = signal_value.split(' ').into_iter().collect();
            // let cmd = cmd_value;

            let signal_value = signal_value.parse::<u16>().unwrap();
            println!("signal_value: {:?}", signal_value);

            // PID 算法
            let pid_signal_value = pid_controller.compute(angle);
            println!("pitch: {:?} {:?}", angle.pitch, pid_signal_value.pitch);
            println!("roll: {:?} {:?}", angle.roll, pid_signal_value.roll);
            println!("yaw: {:?} {:?}", angle.yaw, pid_signal_value.yaw);

            tb6612fng.flip_throttle(signal_value);
        });
    }

    /// 任务处理
    #[idle(local = [], shared = [delay,usart])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            ctx.shared.usart.lock(|usart| {
                usart.send_byte(1).unwrap();
            });
            // 等待开启飞控
            if !is_flight_control_enbaled() {
                ctx.shared.delay.lock(|delay| {
                    delay.delay_ms(1000_u16);
                });
                continue;
            };
            // nrf24l01_sender::spawn(ctx.local.nrf24l01_s.clone()).unwrap();
            println!("=======================");
            ctx.shared.delay.lock(|delay| {
                delay.delay_ms(1000_u16);
            });
        }
    }
}
