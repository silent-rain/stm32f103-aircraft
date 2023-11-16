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
    PB0,
    // LED 灯
    PB1,

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
    // TB6612FNG 电机驱动2
    PA2, // *
    PA3, // *
    PA6,
    PA7,

    // 待分配的引脚
    PB8,
    PB9,
    PB12,
    PB13,
    PB14,
    PB15,
    PA8,
    PA11,
    PA12,
    PA15,

    // 不可使用引脚, 需要重置才可使用
    PB2,
    PA13,
    PA14,
}
