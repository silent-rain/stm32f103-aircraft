//! LED 灯

use stm32f1xx_hal::gpio::{self, Output, OutputSpeed, Pin, PA0};

/// 初始化 LED 灯
pub fn init_led(pa0: PA0, crl: &mut gpio::Cr<'A', false>) -> Pin<'A', 0, Output> {
    let mut led = pa0.into_push_pull_output(crl);
    led.set_speed(crl, gpio::IOPinSpeed::Mhz50);
    led
}
