#![no_std]
#![no_main]

pub mod delay;
pub mod sensor;

use crate::sensor::mpu6050;

use defmt::println;
use defmt_rtt as _;
use panic_probe as _;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{self, Edge, ExtiPin, OutputSpeed},
    pac::{Interrupt, NVIC},
    prelude::{_stm32_hal_gpio_GpioExt, _stm32_hal_rcc_RccExt},
    timer::{SysDelay, SysTimerExt},
};

// 定义应用程序资源和任务
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        delay: SysDelay,
        button: gpio::PB1<gpio::Input<gpio::PullUp>>,
        led: gpio::PA0<gpio::Output<gpio::PushPull>>,
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

        // 初始化 MPU6050 引脚
        let mpu_scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let mpu_sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        // 初始化 MPU6050 传感器
        let mut mpu = mpu6050::init(i2c2, (mpu_scl, mpu_sda), &mut delay, clocks);

        // 读取温度传感器的标度数据，单位为摄氏度
        let temp = mpu.get_temp().unwrap();
        println!("Temperature: {}°C", temp);

        // 获取加速度数据，单位为g
        let acc = mpu.get_acc().unwrap();
        println!("Accel: ({}, {}, {})", acc.x, acc.y, acc.z);

        // 获取角速度数据，单位为弧度每秒
        let gyro = mpu.get_gyro().unwrap();
        println!("Gyro: ({}, {}, {})", gyro.x, gyro.y, gyro.z);

        // 读取设备的姿态角，单位为度
        let angles = mpu.get_acc_angles().unwrap();
        let pitch = angles[0]; // 俯仰角
        let roll = angles[1]; // 横滚角
        println!("Pitch: {:?}, Roll: {:?}", pitch, roll);

        // 获取当前加速度范围
        let accel_range = mpu.get_accel_range().unwrap();
        println!("accel_range: {:#?}", accel_range as u8);

        // 获取当前陀螺仪范围
        let gyro_range = mpu.get_gyro_range().unwrap();
        println!("gyro_range: {:#?}", gyro_range as u8);

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

        println!("init end ...");
        // 初始化静态资源以稍后通过RTIC使用它们
        (Shared {}, Local { button, led, delay })
    }

    // 中断处理函数
    #[task(binds = EXTI1,priority = 1, local = [delay, button, led])]
    fn button_click(ctx: button_click::Context) {
        println!("button click ...");
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }

    // Optional.
    //
    // https://rtic.rs/dev/book/en/by-example/app_idle.html
    // > 当没有声明空闲功能时，运行时设置 SLEEPONEXIT 位，然后在运行 init 后将微控制器发送到睡眠状态。
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
