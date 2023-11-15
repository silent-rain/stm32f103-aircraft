#![no_std]
#![no_main]

pub mod config;
pub mod hardware;

use defmt_rtt as _;
// global logger
use panic_probe as _;
// adjust HAL import
// memory layout
use stm32f1xx_hal as _;

// 引脚校验
// 备注*的引脚尽量不替换
// PB3、PB4、PA15 为复用引脚
#[allow(unused)]
enum _Pin {
    // 按键 KEY
    PB8,
    // LED 灯
    PB9,

    // USART 串口
    PA9,  // *
    PA10, // *

    // NRF24L01 无线通信
    PB3, // *
    PB4, // *
    PB5, // *
    PB6,
    PB7,

    // MPU6050 传感器
    PB10, // *
    PB11, // *

    // TB6612FNG 电机驱动1
    PA0, // *
    PA1, // *
    PA4,
    PA5,
    PA6,
    PA7,
    // TB6612FNG 电机驱动2
    PA2, // *
    PA3, // *
    PA8,
    PA11,
    PA12,
    PA15,

    // 待分配的引脚
    PB0,
    PB1,
    PB12,
    PB13,
    PB14,
    PB15,

    // 不可使用引脚, 需要重置才可使用
    PB2,
    PA13,
    PA14,
}
