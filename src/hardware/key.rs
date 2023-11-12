//! 按键
use stm32f1xx_hal::{
    afio::{self},
    gpio::{self, Edge, ExtiPin, Input, Pin, PullUp, PB1},
    pac::{Interrupt, EXTI, NVIC},
};

/// 初始化按键 KEY
pub fn init_key(
    pb1: PB1,
    crl: &mut gpio::Cr<'B', false>,
    exti: &mut EXTI,
    nvic: &mut NVIC,
    afio: &mut afio::Parts,
) -> Pin<'B', 1, Input<PullUp>> {
    // KEY
    let mut key = pb1.into_pull_up_input(crl);
    // 配置 AFIO 外部中断引脚选择
    key.make_interrupt_source(afio);
    // 从该引脚启用外部中断
    key.enable_interrupt(exti);
    // 下升沿生成中断
    key.trigger_on_edge(exti, Edge::RisingFalling);

    // 使能中断
    unsafe {
        NVIC::unmask(Interrupt::EXTI1);
        nvic.set_priority(Interrupt::EXTI1, 1);
    }

    key
}
