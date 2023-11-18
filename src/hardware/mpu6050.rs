//! # MPU6050 传感器
//!
//! 是一款集成了三轴陀螺仪和三轴加速度计的六轴运动传感器，
//! 它可以测量设备的姿态、角速度、加速度等信息，
//! 用于实现运动控制、姿态估计、手势识别等功能。
//! 它也可以通过 I2C 接口与微控制器连接，通过寄存器操作和数据读取。
//!
//! max_duty 是 PWM 信号的最大占空比，它表示 PWM 信号的高电平时间占总时间的最大比例，单位是 u16（无符号 16 位整数）。
//!
//! max_duty / 2 的意思是将 PWM 信号的占空比设置为最大占空比的一半，也就是 PWM 信号的高电平时间占总时间的一半。
//! 这样做的目的是为了使飞行器的各个电机的转速保持在一个中等水平，从而使飞行器的飞行状态保持在一个平衡状态。
//!

use mpu6050::device::{AccelRange, GyroRange, ACCEL_HPF, CLKSEL};
// pub use mpu6050::Mpu6050;
use stm32f1xx_hal::{
    gpio::{self, Alternate, OpenDrain, PB10, PB11},
    i2c::{self, BlockingI2c, DutyCycle},
    pac::I2C2,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    timer::SysDelay,
};

// 数据采样的时间间隔，假设为10ms
const DT: f32 = 0.01;

/// Mpu6050 对象别名
pub type Mpu6050TY =
    mpu6050::Mpu6050<BlockingI2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>>;

/// 姿态角
#[derive(Debug)]
pub struct AttitudeAngle {
    /// 俯仰角
    pub pitch: f32,
    /// 横滚角
    pub roll: f32,
    /// 偏航角
    pub yaw: f32,
}

/// Mpu6050
pub struct Mpu6050 {
    pub mpu: Mpu6050TY,
}

impl Mpu6050 {
    /// 初始化 MPU6050 传感器
    pub fn new(
        i2c2: I2C2,
        pb10: PB10,
        pb11: PB11,
        crh: &mut gpio::Cr<'B', true>,
        delay: &mut SysDelay,
        clocks: Clocks,
    ) -> Mpu6050 {
        // 初始化 MPU6050 引脚
        // scl（时钟线）：用于同步数据传输，控制数据的传输速度和顺序。
        // 在 MPU6050 中，scl 信号用于同步数据位的发送和接收。
        // sda（数据线）：用于传输数据到 MPU6050。
        // 在 MPU6050 中，sda 信号用于传输每个寄存器的数据，
        // 包括加速度计、陀螺仪等传感器的数据。
        let scl = pb10.into_alternate_open_drain(crh);
        let sda = pb11.into_alternate_open_drain(crh);

        // 创建i2c实例
        let i2c = BlockingI2c::i2c2(
            i2c2,
            (scl, sda),
            i2c::Mode::fast(200.kHz(), DutyCycle::Ratio2to1),
            clocks,
            1000, // 发启动信号的超时时间，单位是微妙
            10,   // 启动信号的重试次数
            1000, // 地址信号的超时时间，单位是微秒
            1000, // 数据信号的超时时间，单位是微妙
        );

        // 创建mpu6050实例，使用默认的从机地址和灵敏度
        let mut mpu = mpu6050::Mpu6050::new(i2c);

        // 初始化 mpu6050
        mpu.init(delay).expect("初始化 mpu6050 失败");

        let mut mpu_obj = Mpu6050 { mpu };

        // 初始化配置
        mpu_obj.init_config();

        mpu_obj
    }

    /// 初始化配置
    fn init_config(&mut self) {
        // 设置时钟源为陀螺仪 X 轴
        // 使用陀螺仪的时钟作为采样时钟，从而提高采样频率。
        self.mpu.set_clock_source(CLKSEL::GXAXIS).unwrap();

        // 设置加速度计高通滤波器
        // 设置为 Hpf0，即禁用高通滤波器
        // 加速度计高通滤波器可以用于滤除低频噪声或重力加速度的影响，从而提取出更高频率的运动信号。通过调整高通滤波器的截止频率，可以控制滤波器的效果。
        self.mpu.set_accel_hpf(ACCEL_HPF::_2P5).unwrap();

        // 设置陀螺仪灵敏度
        // 设置为 ±2000°/s
        self.mpu.set_gyro_range(GyroRange::D2000).unwrap();

        // 设置加速度计灵敏度
        // 设置为 ±16g
        self.mpu.set_accel_range(AccelRange::G16).unwrap();
    }
}

impl Mpu6050 {
    /// 获取姿态角
    pub fn attitude_angle(&mut self) -> AttitudeAngle {
        // 获取角速度数据，单位为弧度每秒
        let gyro = self.mpu.get_gyro().unwrap();
        // 读取设备的姿态角，单位为度
        let angles = self.mpu.get_acc_angles().unwrap();
        // 俯仰角
        let pitch = angles[0];
        // 横滚角
        let roll = angles[1];
        // 偏航角
        let yaw = gyro.z * DT;
        AttitudeAngle { pitch, roll, yaw }
    }
}

#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use super::*;

    use defmt::assert;
    use defmt::println;
    use stm32f1xx_hal::flash::FlashExt;
    use stm32f1xx_hal::gpio::GpioExt;
    use stm32f1xx_hal::pac;
    use stm32f1xx_hal::prelude::_stm32_hal_rcc_RccExt;
    use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_SysCounterExt;

    #[test]
    fn test_init() {
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        let i2c2 = dp.I2C2;
        let rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let syst = cp.SYST;

        let mut gpiob = dp.GPIOB.split();

        // 初始化时钟
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        // 具有自定义精度的阻塞延迟
        let mut delay = syst.delay(&clocks);

        // 初始化 MPU6050 传感器
        let mut mpu = Mpu6050::new(
            i2c2,
            gpiob.pb10,
            gpiob.pb11,
            &mut gpiob.crh,
            &mut delay,
            clocks,
        );

        // 读取温度传感器的标度数据，单位为摄氏度
        let temp = mpu.mpu.get_temp().unwrap();
        println!("Temperature: {}°C", temp);

        // 获取加速度数据，单位为g
        let acc = mpu.mpu.get_acc().unwrap();
        println!("Accel: ({}, {}, {})", acc.x, acc.y, acc.z);

        // 获取角速度数据，单位为弧度每秒
        let gyro = mpu.mpu.get_gyro().unwrap();
        println!("Gyro: ({}, {}, {})", gyro.x, gyro.y, gyro.z);

        // 读取设备的姿态角，单位为度
        let angles = mpu.mpu.get_acc_angles().unwrap();
        let pitch = angles[0]; // 俯仰角
        let roll = angles[1]; // 横滚角
        println!("Pitch: {:?}, Roll: {:?}", pitch, roll);

        // 获取当前加速度范围
        let accel_range = mpu.mpu.get_accel_range().unwrap();
        println!("accel_range: {:#?}", accel_range as u8);

        // 获取当前陀螺仪范围
        let gyro_range = mpu.mpu.get_gyro_range().unwrap();
        println!("gyro_range: {:#?}", gyro_range as u8);

        // 初始化 MPU6050
        assert!(true)
    }
}
