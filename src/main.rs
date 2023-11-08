#![no_std]
#![no_main]

pub mod delay;

use defmt::println;
use defmt_rtt as _;
use panic_probe as _;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use mpu6050::Mpu6050;
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{self, Edge, ExtiPin, OutputSpeed},
    i2c::{self, BlockingI2c},
    pac::{Interrupt, NVIC},
    prelude::{_fugit_RateExtU32, _stm32_hal_gpio_GpioExt, _stm32_hal_rcc_RccExt},
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

        // MPU6050 初始化
        let mpu_scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let mpu_sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
        let pins = (mpu_scl, mpu_sda);
        // 创建i2c实例
        let i2c = BlockingI2c::i2c2(
            i2c2,
            pins,
            i2c::Mode::standard(10.kHz()),
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        // 创建mpu6050实例，使用默认的从机地址和灵敏度
        let mut mpu = Mpu6050::new(i2c);

        // 初始化mpu6050
        mpu.init(&mut delay).unwrap();

        // 获取加速度数据，单位为g
        let acc = mpu.get_acc().unwrap();
        println!("Accel: ({}, {}, {})", acc.x, acc.y, acc.z);

        // 获取角速度数据，单位为弧度每秒
        let gyro = mpu.get_gyro().unwrap();
        println!("Gyro: ({}, {}, {})", gyro.x, gyro.y, gyro.z);

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
