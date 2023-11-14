#![no_std]
#![no_main]

use defmt_rtt as _;
// global logger
use panic_probe as _;
// adjust HAL import
// memory layout
use stm32f1xx_hal as _;

pub mod hardware;
pub mod config;

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
    PA9,

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
    PB12,
    PB13,
    PB14,
    PB15,
    PA10,
    PA12,

    // 待分配的引脚

    // 不可使用引脚, 需要重置才可使用
    PB2,
    PA13,
    PA14,
    PA15,
    PB3,
    PB4,

    // 不可使用引脚, 被OLED遮挡
    PB5,
    PB6,
    PB7,
}
