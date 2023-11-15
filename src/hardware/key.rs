//! 按键
use stm32f1xx_hal::{
    afio::{self},
    gpio::{self, Edge, ExtiPin, Input, PullUp, PB8},
    pac::{Interrupt, EXTI, NVIC},
};

use crate::config::FLIGHT_CONTROL_ENBALED;

/// 按键
pub struct Key {
    key: PB8<Input<PullUp>>,
}

impl Key {
    /// 初始化按键 KEY
    pub fn new(
        pin: PB8,
        crh: &mut gpio::Cr<'B', true>,
        exti: &mut EXTI,
        nvic: &mut NVIC,
        afio: &mut afio::Parts,
    ) -> Self {
        // KEY
        let mut key = pin.into_pull_up_input(crh);
        // 配置 AFIO 外部中断引脚选择
        key.make_interrupt_source(afio);
        // 从该引脚启用外部中断
        key.enable_interrupt(exti);
        // 下升沿生成中断
        key.trigger_on_edge(exti, Edge::Falling);

        // 使能中断
        unsafe {
            NVIC::unmask(Interrupt::EXTI15_10);
            nvic.set_priority(Interrupt::EXTI15_10, 1);
        }

        Key { key }
    }

    /// 清空中断标识
    pub fn clear_interrupt_pending_bit(&mut self) {
        self.key.clear_interrupt_pending_bit();
    }
}

/// 飞控开关是否开启
pub fn is_flight_control_enbaled() -> bool {
    unsafe { FLIGHT_CONTROL_ENBALED }
}

/// 开启飞控
pub fn enbaled_flight_control() {
    unsafe {
        FLIGHT_CONTROL_ENBALED = true;
    }
}

/// 关闭飞控
pub fn disabled_flight_control() {
    unsafe {
        FLIGHT_CONTROL_ENBALED = false;
    }
}
