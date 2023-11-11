#![no_std]
#![no_main]

use stm32f103_uav::{
    oled,
    sensor::mpu6050::{self, Mpu6050},
};

use defmt::println;
use defmt_rtt as _;
use panic_probe as _;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{self, Alternate, Edge, ExtiPin, OpenDrain, OutputSpeed, Pin},
    i2c::BlockingI2c,
    pac::{Interrupt, I2C2, NVIC},
    prelude::{_stm32_hal_gpio_GpioExt, _stm32_hal_rcc_RccExt},
    timer::{SysDelay, SysTimerExt},
};

/// Mpu6050 对象别名
type Mpu6050TY = Mpu6050<
    BlockingI2c<
        I2C2,
        (
            Pin<'B', 10, Alternate<OpenDrain>>,
            Pin<'B', 11, Alternate<OpenDrain>>,
        ),
    >,
>;

// 定义应用程序资源和任务
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        delay: SysDelay,
    }

    #[local]
    struct Local {
        button: gpio::PB1<gpio::Input<gpio::PullUp>>,
        led: gpio::PA0<gpio::Output<gpio::PushPull>>,
        mpu6050: Mpu6050TY,
    }

    // 初始化函数
    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // 获取外设实例
        let mut afio = ctx.device.AFIO.constrain();
        let mut flash = ctx.device.FLASH.constrain();
        let rcc = ctx.device.RCC.constrain();
        let syst = ctx.core.SYST;
        let mut nvic = ctx.core.NVIC;
        let i2c2 = ctx.device.I2C2;

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();

        // 初始化时钟
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        // 具有自定义精度的阻塞延迟
        let mut delay = syst.delay(&clocks);

        delay.delay_ms(1000_u16);
        println!("init start ...");

        // 初始化 OLED
        let (mut scl, mut sda) = oled::simple::init_oled(gpiob.pb8, gpiob.pb9, &mut gpiob.crh);
        let mut oled = oled::OLED::new(&mut scl, &mut sda);

        // 初始化 MPU6050 引脚
        let mpu_scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let mpu_sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        // 初始化 MPU6050 传感器
        let mut mpu6050 = mpu6050::init(i2c2, (mpu_scl, mpu_sda), &mut delay, clocks);

        // 读取温度传感器的标度数据，单位为摄氏度
        let temp = mpu6050.get_temp().unwrap();
        println!("Temperature: {}°C", temp);

        // 获取加速度数据，单位为g
        let acc = mpu6050.get_acc().unwrap();
        println!("Accel: ({}, {}, {})", acc.x, acc.y, acc.z);

        // 获取角速度数据，单位为弧度每秒
        let gyro = mpu6050.get_gyro().unwrap();
        println!("Gyro: ({}, {}, {})", gyro.x, gyro.y, gyro.z);

        // 读取设备的姿态角，单位为度
        let angles = mpu6050.get_acc_angles().unwrap();
        let pitch = angles[0]; // 俯仰角
        let roll = angles[1]; // 横滚角
        println!("Pitch: {:?}, Roll: {:?}", pitch, roll);

        // LED
        let mut led = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);
        led.set_speed(&mut gpioa.crl, gpio::IOPinSpeed::Mhz50);

        // KEY
        let mut button = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);
        // 配置 AFIO 外部中断引脚选择
        button.make_interrupt_source(&mut afio);
        // 从该引脚启用外部中断
        button.enable_interrupt(&mut ctx.device.EXTI);
        // 下升沿生成中断
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        // 使能中断
        unsafe {
            NVIC::unmask(Interrupt::EXTI1);
            nvic.set_priority(Interrupt::EXTI1, 1);
        }

        oled.show_string(1, 1, "hallo");
        println!("init end ...");
        (
            Shared { delay },
            Local {
                button,
                led,
                mpu6050,
            },
        )
    }

    /// 按钮触发 LED 灯
    /// 中断处理函数
    #[task(binds = EXTI1,priority = 1, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }

    /// 任务处理
    #[idle(local = [mpu6050], shared = [delay])]
    fn idle(ctx: idle::Context) -> ! {
        let mpu6050 = ctx.local.mpu6050;
        let mut delay = ctx.shared.delay;
        loop {
            // 读取温度传感器的标度数据，单位为摄氏度
            let temp = mpu6050.get_temp().unwrap();
            println!("Temperature: {}°C", temp);

            // 获取加速度数据，单位为g
            let acc = mpu6050.get_acc().unwrap();
            println!("Accel: ({}, {}, {})", acc.x, acc.y, acc.z);

            // 获取角速度数据，单位为弧度每秒
            let gyro = mpu6050.get_gyro().unwrap();
            println!("Gyro: ({}, {}, {})", gyro.x, gyro.y, gyro.z);

            // 读取设备的姿态角，单位为度
            let angles = mpu6050.get_acc_angles().unwrap();
            let pitch = angles[0]; // 俯仰角
            let roll = angles[1]; // 横滚角
            println!("Pitch: {:?}, Roll: {:?}", pitch, roll);

            delay.lock(|f| {
                f.delay_ms(5000_u32);
            })
        }
    }
}
