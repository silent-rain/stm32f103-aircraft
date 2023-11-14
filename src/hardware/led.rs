//! LED 灯

use stm32f1xx_hal::gpio::{self, Output, OutputSpeed, PA4};

/// 初始化 LED 灯
/// 低电平点灯
pub fn init_led(pa4: PA4, crl: &mut gpio::Cr<'A', false>) -> PA4<Output> {
    let mut led = pa4.into_push_pull_output(crl);
    led.set_speed(crl, gpio::IOPinSpeed::Mhz50);

    // 默认熄灭
    led.set_high();
    led
}
