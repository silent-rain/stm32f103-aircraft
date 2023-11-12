//! 简单的 OLED 实例

use stm32f1xx_hal::gpio::{self, OutputSpeed};

/// 初始化 OLED 显示屏引脚
/// pin: pb8、pb9
/// ```rust
/// use oled;
/// let (scl, sda) = oled::simple::init_oled(gpiob.pb8, gpiob.pb9, &mut gpiob.crh);
/// let mut oled = oled::OLED::new(scl, sda);
/// oled.show_string(1, 1, "hallo");
/// ```
pub fn init_oled(
    pb8: gpio::Pin<'B', 8>,
    pb9: gpio::Pin<'B', 9>,
    crh: &mut gpio::Cr<'B', true>,
) -> (
    gpio::PB8<gpio::Output<gpio::OpenDrain>>,
    gpio::PB9<gpio::Output<gpio::OpenDrain>>,
) {
    // 将引脚配置为作为开漏输出模式
    let mut scl = pb8.into_open_drain_output(crh);
    let mut sda = pb9.into_open_drain_output(crh);
    scl.set_speed(crh, gpio::IOPinSpeed::Mhz50);
    sda.set_speed(crh, gpio::IOPinSpeed::Mhz50);

    (scl, sda)
}
