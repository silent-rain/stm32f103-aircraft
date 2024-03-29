//! LED 灯

use stm32f1xx_hal::gpio::{Cr, IOPinSpeed, Output, OutputSpeed, PB1};

/// LED 灯
pub struct Led {
    led: PB1<Output>,
}

impl Led {
    /// 初始化 LED 灯
    pub fn new(pin: PB1, crl: &mut Cr<'B', false>) -> Self {
        let mut pin_led = pin.into_push_pull_output(crl);
        pin_led.set_speed(crl, IOPinSpeed::Mhz50);

        let mut led = Led { led: pin_led };

        // 默认熄灭
        led.off();

        led
    }

    /// 关闭 LED 灯
    /// 高电平熄灭
    pub fn off(&mut self) {
        self.led.set_high();
    }

    /// 开启 LED 灯
    /// 低电平点灯
    pub fn on(&mut self) {
        self.led.set_low();
    }

    /// 切换 LED 灯关闭/开启状态
    pub fn toggle(&mut self) {
        self.led.set_low();
        self.led.toggle()
    }
}
