#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use stm32f103_uav::hardware::{
    key, led,
    mpu6050::{self, Mpu6050Data, Mpu6050TY},
};

use cortex_m::{asm::wfi, prelude::_embedded_hal_blocking_delay_DelayMs};
use defmt::println;
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};
use stm32f1xx_hal::timer::SysDelay;
use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{self, ExtiPin},
    prelude::{_stm32_hal_gpio_GpioExt, _stm32_hal_rcc_RccExt},
    timer::SysTimerExt,
};

// MPU6050 消息通道
const MPU6050_CAPACITY: usize = 1;

// 定义应用程序资源和任务
#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true,dispatchers = [USART1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        delay: SysDelay,
        key: gpio::PB1<gpio::Input<gpio::PullUp>>,
        led: gpio::PA0<gpio::Output<gpio::PushPull>>,
        mpu6050: Mpu6050TY,
    }

    #[local]
    struct Local {}

    // 初始化函数
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // 获取外设实例
        let mut afio = ctx.device.AFIO.constrain();
        let mut flash = ctx.device.FLASH.constrain();
        let rcc = ctx.device.RCC.constrain();
        let i2c2 = ctx.device.I2C2;
        let mut exti = ctx.device.EXTI;

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

        // 初始化 OLED 显示屏
        // let oled = init_oled(gpiob.pb8, gpiob.pb9, &mut gpiob.crh);
        // 初始化按键 KEY
        let key = key::init_key(gpiob.pb1, &mut gpiob.crl, &mut exti, &mut nvic, &mut afio);
        // 初始化 LED 灯
        let led = led::init_led(gpioa.pa0, &mut gpioa.crl);
        // 初始化 MPU6050 传感器
        let mpu6050 = mpu6050::init(
            gpiob.pb10,
            gpiob.pb11,
            &mut gpiob.crh,
            i2c2,
            &mut delay,
            clocks,
        );

        // MPU6050 传感器传递
        let (mpu6050_s, mpu6050_r) = make_channel!(Mpu6050Data, MPU6050_CAPACITY);
        mpu6050_receiver::spawn(mpu6050_r).unwrap();
        mpu6050_sender::spawn(mpu6050_s.clone()).unwrap();

        // oled.show_string(1, 1, "hallo");
        println!("init end ...");
        (
            Shared {
                delay,
                key,
                led,
                mpu6050,
            },
            Local {},
        )
    }

    // /// 初始化 OLED 显示屏
    // fn init_oled(
    //     pb8: PB8,
    //     pb9: PB9,
    //     crh: &mut gpio::Cr<'B', true>,
    // ) -> oled::OLED<'_, Pin<'B', 8, Output<OpenDrain>>, Pin<'B', 9, Output<OpenDrain>>> {
    //     let (mut scl, mut sda) = oled::simple::init_oled(pb8, pb9, crh);
    //     let oled = oled::OLED::new(&mut scl, &mut sda);
    //     oled
    // }

    /// 发送 MPU6050 传感器数据
    #[task(priority = 1, shared=[mpu6050,delay])]
    async fn mpu6050_sender(
        mut ctx: mpu6050_sender::Context,
        mut sender: Sender<'static, Mpu6050Data, MPU6050_CAPACITY>,
    ) {
        loop {
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

            ctx.shared.delay.lock(|delay| {
                delay.delay_ms(1000_u16);
            });
        }
    }

    /// 接收 MPU6050 传感器数据
    #[task(priority = 1)]
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

    /// 按钮中断事件触发 LED 灯
    #[task(binds = EXTI1,priority = 1, local = [],shared=[key,led])]
    fn key_click(ctx: key_click::Context) {
        let key = ctx.shared.key;
        let led = ctx.shared.led;
        (key, led).lock(|key, led| {
            key.clear_interrupt_pending_bit();
            led.toggle();
        });
    }

    /// 任务处理
    #[idle(local = [], shared = [])]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            wfi();
        }
    }
}
