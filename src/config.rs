//! 全局配置

/// 飞控开关
pub static mut FLIGHT_CONTROL_ENBALED: bool = false;

/// USART1 波特率
pub const USART1_BAUDRATE: u32 = 115200;

/// RF24L01 发送协议地址
pub const NRF24L01_TX_ADDR: &[u8] = b"fnord";
/// RF24L01 接收协议地址
pub const NRF24L01_RX_ADDR: &[u8] = b"fnord";
pub const NRF24L01_RX_ADDR_P0: usize = 0x0A;
pub const NRF24L01_RX_ADDR_P1: usize = 0x0B;
pub const NRF24L01_RX_ADDR_P2: usize = 0x0C;
pub const NRF24L01_RX_ADDR_P3: usize = 0x0D;
pub const NRF24L01_RX_ADDR_P4: usize = 0x0E;
pub const NRF24L01_RX_ADDR_P5: usize = 0x0F;
