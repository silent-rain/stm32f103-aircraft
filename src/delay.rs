//! SysTick 延时

use cortex_m::peripheral::{syst::SystClkSource, SYST};

/// 微秒级延时
/// xus 延时时长，范围：0~233015
pub fn delay_us(xus: u32) {
    let systick: *mut SYST = (SYST::PTR as *mut u32).cast();
    let syst = unsafe { &mut *systick };

    // 内部时钟源，时钟频率为72MHz
    syst.set_clock_source(SystClkSource::Core);

    // 设置 SysTick 的重载值;
    // 当 SysTick 的计数值达到这个值时，它会自动清零并设置 "已经回绕" 标志
    syst.set_reload((72_000_000 / 1000 / 1000) * xus);

    // 清除 SysTick 的当前计数值
    syst.clear_current();
    // 启用 SysTick 的计数器
    syst.enable_counter();

    // 关闭 SysTick 的计数器
    syst.disable_counter();
}

/// 毫秒级延时
/// xms 延时时长，范围：0~4294967295
pub fn delay_ms(xms: u32) {
    for _ in 0..xms {
        delay_us(1000);
    }
}

/// 秒级延时
/// xs 延时时长，范围：0~4294967295
pub fn delay_s(xs: u32) {
    for _ in 0..xs {
        delay_ms(1000);
    }
}
