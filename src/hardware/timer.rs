//! 定时器

use stm32f1xx_hal::{
    pac::TIM3,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    timer::{CounterHz, Event, TimerExt},
};

/// 初始化定时器
/// TIM3 定时中断
pub fn init_timer(tim3: TIM3, clocks: &Clocks) {
    // 配置系统定时器，每秒触发一次更新，并启用中断
    let mut timer: CounterHz<TIM3> = tim3.counter_hz(clocks);

    // 设置一个 50ms 后过期的计时器
    // 定时 50ms 即定时频率为 20Hz；
    // hz = 1hz/1s/50ms
    timer.start(1.Hz()).unwrap();

    // 设置一个 50ms 后过期的计时器
    // 使用用户定义的预分频器和自动重新加载寄存器以倒计时模式重新启动计时器
    // timer.start_raw(7199, 4999 / 10);

    // 当计时器到期时产生中断
    timer.listen(Event::Update);
}
