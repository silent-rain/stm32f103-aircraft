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
#[allow(unused)]
enum _Pin {
    // OLED 显示屏
    PB8, // *
    PB9, // *

    // LED 灯
    PA4,

    // 按键 KEY
    PA11,

    // NRF24L01 无线通信
    PA5, // *
    PA6, // *
    PA7, // *
    PA8,
    PA12,

    // MPU6050 传感器
    PB10, // *
    PB11, // *

    // TB6612FNG 电机驱动
    PA0, // *
    PA1, // *
    PA2, // *
    PA3, // *
    PB0,
    PB1,
    PB3, // 复用引脚
    PB4, // 复用引脚
    PB12,
    PB13,
    PB14,
    PB15,

    // USART 串口
    PA9,  // *
    PA10, // *

    // 待分配的引脚
    PB5,
    PB6,
    PB7,
    PA15, // 复用引脚

    // 不可使用引脚, 需要重置才可使用
    PB2,
    PA13,
    PA14,
}
