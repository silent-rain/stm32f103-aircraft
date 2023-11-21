//! # PID 控制器
//!
//! PID 算法的原理是根据误差的比例、积分和微分，来计算控制量，
//! 从而使系统的实际输出接近期望输出。
//!
//! PID 算法的优点是易于实现，参数调节方便，适应性强，缺点是可能存在稳态误差，
//! 对于非线性系统和时变系统的控制效果不理想，容易受到干扰和噪声的影响。
//!
//! 根据输入的值（也就是飞行器的实际姿态），计算输出的值（也就是飞行器的控制量），
//! 从而使飞行器达到期望的姿态。
//!
//! 根据 MPU6050 的数据，计算 pitch 和 roll 的误差，设计 PID 控制器，输出 PWM 信号，
//! 控制四个 LED 的亮度，以反映飞行器的姿态。

use crate::config::{PID_KD, PID_KI, PID_KP};

use super::mpu6050::AttitudeAngle;

// 数据采样的时间间隔，假设为10ms
const DELTA_T: f32 = 0.01;

/// PID 控制器的结构体
pub struct PidController {
    /// 比例系数
    pub kp: f32,
    /// 积分系数
    pub ki: f32,
    /// 微分系数
    pub kd: f32,
    /// 积分
    pub integral: f32,
    /// 微分
    pub derivative: f32,
    /// 上一次的误差
    pub last_error: f32,
}

// 实现 PID 控制器的方法
impl PidController {
    /// 创建一个新的 PID 控制器实例
    ///
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            derivative: 0.0,
            last_error: 0.0,
        }
    }

    /// 计算 PID 控制器的输出
    ///
    /// setpoint： 飞行器达到的期望姿态的值，单位是 °（度）
    /// input: 飞行器当前的倾角
    pub fn compute(&mut self, setpoint: f32, input: f32) -> f32 {
        // 计算误差
        let error = setpoint - input;
        // 计算比例项
        let pout = self.kp * error;

        // 计算积分项
        self.integral += error * DELTA_T;
        let iout = self.ki * self.integral;

        // 计算微分项
        let derivative = (error - self.last_error) / DELTA_T;
        let dout = self.kd * derivative;

        // 更新上一次误差
        self.last_error = error;

        // 计算总输出
        pout + iout + dout
    }
}

/// 电机 PID 控制器
pub struct MotorPidController {
    pitch_controller: PidController,
    yaw_controller: PidController,
    roll_controller: PidController,
}

/// 电机 PID 算法输出的转速
pub struct MotorPidOutput {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

impl MotorPidController {
    /// 创建 PID 算法控制器对象
    pub fn new() -> Self {
        // 创建一个用于控制 pitch 的 PID 控制器的实例
        let pitch_controller = PidController::new(PID_KP, PID_KI, PID_KD);

        // 创建一个用于控制 yaw 的 PID 控制器的实例
        let yaw_controller = PidController::new(PID_KP, PID_KI, PID_KD);

        // 创建一个用于控制 roll 的 PID 控制器的实例
        let roll_controller = PidController::new(PID_KP, PID_KI, PID_KD);

        MotorPidController {
            pitch_controller,
            yaw_controller,
            roll_controller,
        }
    }

    /// 计算姿态角的误差输出
    /// todo: 待完善
    pub fn compute(&mut self, angle: AttitudeAngle) -> MotorPidOutput {
        // 计算 pitch 的误差和输出
        let pitch = self.pitch_controller.compute(1.0, angle.pitch);

        // 计算 yaw 的误差和输出
        let yaw = self.yaw_controller.compute(1.0, angle.yaw);

        // 计算 roll 的误差和输出
        let roll = self.roll_controller.compute(1.0, angle.roll);
        MotorPidOutput { pitch, yaw, roll }
    }
}

impl Default for MotorPidController {
    fn default() -> Self {
        Self::new()
    }
}
