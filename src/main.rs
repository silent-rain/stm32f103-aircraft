#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use stm32f103_uav::hardware::{
    key::{self, disabled_flight_control, enbaled_flight_control, is_flight_control_enbaled},
    led,
    mpu6050::{self, Mpu6050Data, Mpu6050TY},
    nrf24l01::{self, NRF24L01Cmd, RxTY},
    tb6612fng, usart,
};

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use defmt::println;
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{self, ExtiPin, PA11, PA4},
    prelude::{_stm32_hal_gpio_GpioExt, _stm32_hal_rcc_RccExt},
    timer::{SysDelay, SysTimerExt},
};

// 消息通道容量
const MPU6050_CAPACITY: usize = 1;
const NRF24L01_CAPACITY: usize = 1;

// 定义应用程序资源和任务
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true,dispatchers = [EXTI2,EXTI3])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        delay: SysDelay,
        key: PA11<gpio::Input<gpio::PullUp>>,
        led: PA4<gpio::Output<gpio::PushPull>>,
        usart: usart::Usart,
        mpu6050: Mpu6050TY,
    }

    #[local]
    struct Local {
        mpu6050_s: Sender<'static, Mpu6050Data, MPU6050_CAPACITY>,
        nrf24l01_s: Sender<'static, NRF24L01Cmd, NRF24L01_CAPACITY>,
        nrf24l01_rx: RxTY,
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
        let usart1 = ctx.device.USART1;

        let syst = ctx.core.SYST;
        let mut nvic = ctx.core.NVIC;

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();

        // 初始化时钟
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        // 具有自定义精度的阻塞延迟
        let mut delay = syst.delay(&clocks);

        delay.delay_ms(1000_u16);
        println!("init start ...");

        // 禁用 jtag 端口进行复用
        let (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // 初始化按键 KEY
        let key = key::init_key(gpioa.pa11, &mut gpioa.crh, &mut exti, &mut nvic, &mut afio);
        // 初始化 LED 灯
        let led = led::init_led(gpioa.pa4, &mut gpioa.crl);
        // 初始化 USART1 串口
        let usart = usart::Usart::new(
            gpioa.pa9,
            gpioa.pa10,
            &mut gpioa.crh,
            usart1,
            &mut afio.mapr,
            &clocks,
            &mut nvic,
        );

        // 初始化 MPU6050 传感器
        let mpu6050 = mpu6050::init(
            i2c2,
            gpiob.pb10,
            gpiob.pb11,
            &mut gpiob.crh,
            &mut delay,
            clocks,
        );
        // 初始化 NRF24L01 2.4 GHz 无线通信
        let nrf24l01 = nrf24l01::init(nrf24l01::Config {
            pa5: gpioa.pa5,
            pa6: gpioa.pa6,
            pa7: gpioa.pa7,
            gpioa_crl: &mut gpioa.crl,
            pa8: gpioa.pa8,
            pa12: gpioa.pa12,
            gpioa_crh: &mut gpioa.crh,
            spi1,
            mapr: &mut afio.mapr,
            clocks,
        });
        let nrf24l01_rx = nrf24l01.rx().unwrap();
        // 初始化 TB6612FNG 电机驱动
        let _tb6612fng = tb6612fng::Tb6612fng::new(tb6612fng::Config {
            tim2,
            mapr: &mut afio.mapr,
            clocks: &clocks,
            pa0: gpioa.pa0,
            pa1: gpioa.pa1,
            pa2: gpioa.pa2,
            pa3: gpioa.pa3,
            gpioa_crl: &mut gpioa.crl,
        });
        // tb6612fng.set_duty(channel, speed);

        // MPU6050 传感器传递
        let (mpu6050_s, mpu6050_r) = make_channel!(Mpu6050Data, MPU6050_CAPACITY);
        mpu6050_receiver::spawn(mpu6050_r).unwrap();

        // NRF24L01 数据传递
        let (nrf24l01_s, nrf24l01_r) = make_channel!(NRF24L01Cmd, NRF24L01_CAPACITY);
        nrf24l01_receiver::spawn(nrf24l01_r).unwrap();

        println!("init end ...");
        (
            Shared {
                delay,
                key,
                led,
                usart,
                mpu6050,
            },
            Local {
                mpu6050_s: mpu6050_s.clone(),
                nrf24l01_s: nrf24l01_s.clone(),
                nrf24l01_rx,
            },
        )
    }

    /// 发送 MPU6050 传感器数据
    #[task(priority = 2, shared=[mpu6050,delay])]
    async fn mpu6050_sender(
        mut ctx: mpu6050_sender::Context,
        mut sender: Sender<'static, Mpu6050Data, MPU6050_CAPACITY>,
    ) {
        let data = ctx.shared.mpu6050.lock(|mpu6050| {
            // 读取温度传感器的标度数据，单位为摄氏度
            let temperature = mpu6050.get_temp().unwrap();

            // 获取加速度数据，单位为g
            let accel = mpu6050.get_acc().unwrap();

            // 获取角速度数据，单位为弧度每秒
            let gyro = mpu6050.get_gyro().unwrap();

            // 读取设备的姿态角，单位为度
            let angles = mpu6050.get_acc_angles().unwrap();
            // 俯仰角
            let pitch = angles[0];
            // 横滚角
            let roll = angles[1];

            Mpu6050Data {
                temperature,
                accel_x: accel.x,
                accel_y: accel.y,
                accel_z: accel.z,
                gyro_x: gyro.x,
                gyro_y: gyro.y,
                gyro_z: gyro.z,
                pitch,
                roll,
            }
        });
        sender.send(data).await.unwrap();
    }

    /// 接收 MPU6050 通道数据
    #[task(priority = 2)]
    async fn mpu6050_receiver(
        _c: mpu6050_receiver::Context,
        mut receiver: Receiver<'static, Mpu6050Data, MPU6050_CAPACITY>,
    ) {
        while let Ok(val) = receiver.recv().await {
            println!(
                "Temperature: {}°C \nAccel: ({}, {}, {}) \nGyro: ({}, {}, {}) \nPitch: {:?}, Roll: {:?}",
                val.temperature,
                val.accel_x,
                val.accel_y,
                val.accel_z,
                val.gyro_x,
                val.gyro_y,
                val.gyro_z,
                val.pitch,
                val.roll,
            );
        }
    }

    /// 将 NRF24L01 无线通信数据转发内部通道
    #[task(priority = 2, local=[nrf24l01_rx], shared=[delay])]
    async fn nrf24l01_sender(
        mut ctx: nrf24l01_sender::Context,
        mut sender: Sender<'static, NRF24L01Cmd, NRF24L01_CAPACITY>,
    ) {
        println!("wait nrf24l01 signal...");
        let rx = ctx.local.nrf24l01_rx;
        // 检查是否有数据可读
        if rx.can_read().unwrap().is_none() {
            ctx.shared.delay.lock(|delay| {
                delay.delay_ms(1000_u16);
            });
            return;
        }
        // 读取数据到缓冲区
        let payload = rx.read().unwrap();
        let data = nrf24l01::payload_string(payload);
        let data_str = data.as_str();
        println!("NRF24L01: len: {} data: {:#?}", data.len(), data_str);

        // todo: 待完善具体接收指令，可考虑cmd使用枚举 cmd&data
        sender.send(NRF24L01Cmd { cmd: 1 }).await.unwrap();
    }

    /// 接收 NRF24L01 通道数据
    #[task(priority = 2)]
    async fn nrf24l01_receiver(
        _c: nrf24l01_receiver::Context,
        mut receiver: Receiver<'static, NRF24L01Cmd, NRF24L01_CAPACITY>,
    ) {
        while let Ok(val) = receiver.recv().await {
            println!("nrf24l01_receiver: {:?}", val.cmd);
        }
    }

    /// 飞控开关, 按键中断
    /// 开启/关闭飞控
    /// 点灯/熄灯
    #[task(binds = EXTI15_10, local = [], shared=[key,led])]
    fn key_click_irq(ctx: key_click_irq::Context) {
        let key = ctx.shared.key;
        let led = ctx.shared.led;
        (key, led).lock(|key, led| {
            key.clear_interrupt_pending_bit();

            // 如果是开启状态, 则关闭飞控及熄灯
            if is_flight_control_enbaled() {
                // 熄灯
                led.set_high();
                // 关闭飞控
                disabled_flight_control();
                return;
            }

            // 点灯
            led.set_low();
            // 开启飞控
            enbaled_flight_control();
        });
    }

    /// USART1 串口中断
    /// 接收数据中断
    /// todo: 待完善指令
    #[task(binds = USART1, local = [], shared=[usart])]
    fn usart1_rev_irq(mut ctx: usart1_rev_irq::Context) {
        ctx.shared.usart.lock(|usart| {
            let cmd = usart.recv_cmd().unwrap();
            println!("cmd: {:?}", cmd);
        })
    }

    /// 任务处理
    #[idle(local = [mpu6050_s,nrf24l01_s], shared = [delay])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            // 等待开启飞控
            if !is_flight_control_enbaled() {
                ctx.shared.delay.lock(|delay| {
                    delay.delay_ms(1000_u16);
                });
                continue;
            };
            mpu6050_sender::spawn(ctx.local.mpu6050_s.clone()).unwrap();
            nrf24l01_sender::spawn(ctx.local.nrf24l01_s.clone()).unwrap();
            println!("=======================");
            ctx.shared.delay.lock(|delay| {
                delay.delay_ms(1000_u16);
            });
        }
    }
}
