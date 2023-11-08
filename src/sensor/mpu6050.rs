//! # MPU6050 传感器
//! 是一款集成了三轴陀螺仪和三轴加速度计的六轴运动传感器，
//! 它可以测量设备的姿态、角速度、加速度等信息，
//! 用于实现运动控制、姿态估计、手势识别等功能。
//! 它也可以通过 I2C 接口与微控制器连接，通过寄存器操作和数据读取。
//!
//! 这里主要用于初始化 MPU6050 传感器

use mpu6050::Mpu6050;
use stm32f1xx_hal::{
    i2c::{self, BlockingI2c, DutyCycle, Pins},
    pac::I2C2,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    timer::SysDelay,
};

/// 初始化 MPU6050 传感器
pub fn init<PINS>(
    i2c2: I2C2,
    pins: PINS,
    delay: &mut SysDelay,
    clocks: Clocks,
) -> Mpu6050<BlockingI2c<I2C2, PINS>>
where
    PINS: Pins<I2C2>,
{
    // 创建i2c实例
    let i2c = BlockingI2c::i2c2(
        i2c2,
        pins,
        i2c::Mode::fast(10.kHz(), DutyCycle::Ratio2to1),
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    // 创建mpu6050实例，使用默认的从机地址和灵敏度
    let mut mpu = Mpu6050::new(i2c);

    // 初始化 mpu6050
    mpu.init(delay).expect("初始化 mpu6050 失败");

    mpu
}

#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use super::*;

    // use defmt_rtt as _; // global logger
    // adjust HAL import
    // use stm32f1xx_hal as _; // memory layout
    // use panic_probe as _;

    use defmt::assert;
    use defmt::println;
    use stm32f1xx_hal::gpio::GpioExt;
    use stm32f1xx_hal::pac;

    #[test]
    fn test_init() {
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        let i2c2 = dp.I2C2;
        let mut gpiob = dp.GPIOB.split();

        // 初始化时钟
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        // 具有自定义精度的阻塞延迟
        let mut delay = syst.delay(&clocks);

        // 初始化 MPU6050 引脚
        let mpu_scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let mpu_sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
        let pins = (mpu_scl, mpu_sda);

        // 初始化 MPU6050 传感器
        let mut mpu = init(i2c2, pins, &mut delay, clocks);

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

        // 初始化 MPU6050
        assert!(true)
    }
}
