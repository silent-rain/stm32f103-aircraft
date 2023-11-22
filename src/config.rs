//! 全局配置

/// 飞控开关
pub static mut FLIGHT_CONTROL_ENBALED: bool = false;

/// USART1 波特率
pub const USART1_BAUDRATE: u32 = 115200;

/// RF24L01 发送协议地址
pub const NRF24L01_TX_ADDR: &[u8] = b"fnord";
/// RF24L01 接收协议地址
pub const NRF24L01_RX_ADDR: &[u8] = b"fnord";
/// RF24L01 接收通道
pub const NRF24L01_RX_ADDR_P0: usize = 0x00;
pub const NRF24L01_RX_ADDR_P1: usize = 0x01;
pub const NRF24L01_RX_ADDR_P2: usize = 0x02;
pub const NRF24L01_RX_ADDR_P3: usize = 0x03;
pub const NRF24L01_RX_ADDR_P4: usize = 0x04;
pub const NRF24L01_RX_ADDR_P5: usize = 0x05;

/// PID 算法
/// 比例系数
pub const PID_KP: f32 = 0.1;
/// 积分系数
pub const PID_KI: f32 = 0.01;
/// 微分系数
pub const PID_KD: f32 = 0.001;

/// 上下油门摇杆最大值值
///
/// 油门值是指遥控器发送给飞控板的一个表示油门量的模拟信号，
/// 一般是一个 `0~3.3V` 的电压值，或者是一个 `0~5000` 的数字量，
/// 取决于遥控器的ADC（模数转换器）的分辨率。
pub const REMOTE_CONTROL_THROTTLE_MAX: u16 = 5000;

/// 左右偏航摇杆最大值值
pub const REMOTE_CONTROL_YAW_MAX: u16 = 90;
/// 左右横滚摇杆最大值值
pub const REMOTE_CONTROL_ROLL_MAX: u16 = 90;
/// 上下俯仰摇杆最大值值
pub const REMOTE_CONTROL_PITCH_MAX: u16 = 90;
