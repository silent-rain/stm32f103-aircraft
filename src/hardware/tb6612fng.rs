//! TB6612FNG 电机驱动
//! 用于驱动直流电机

use stm32f1xx_hal::{
    afio::MAPR,
    gpio::{self, Alternate, IOPinSpeed, OutputSpeed, PA0, PA1, PA2, PA3, PB0, PB1, PB12, PB13},
    pac::TIM2,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    timer::{Ch, Channel, PwmExt, PwmHz, Tim2NoRemap},
};

/// 配置
pub struct Config<'a> {
    pub pa0: PA0,
    pub pa1: PA1,
    pub pa2: PA2,
    pub pa3: PA3,
    pub gpioa_crl: &'a mut gpio::Cr<'A', false>,
    pub tim2: TIM2,
    pub mapr: &'a mut MAPR,
    pub clocks: &'a Clocks,
}

// pwm 类型别名
type PwmTy = PwmHz<
    TIM2,
    Tim2NoRemap,
    (Ch<0>, Ch<1>, Ch<2>, Ch<3>),
    (
        PA0<Alternate>,
        PA1<Alternate>,
        PA2<Alternate>,
        PA3<Alternate>,
    ),
>;

pub struct Tb6612fng {
    pwm: PwmTy,
    max_duty: u16,
}

impl<'a> Tb6612fng {
    /// 创建对象
    pub fn new(config: Config) -> Self {
        // 控制电机速度
        let pwma0 = config.pa0.into_alternate_push_pull(config.gpioa_crl);
        let pwma1 = config.pa1.into_alternate_push_pull(config.gpioa_crl);
        let pwma2 = config.pa2.into_alternate_push_pull(config.gpioa_crl);
        let pwma3 = config.pa3.into_alternate_push_pull(config.gpioa_crl);

        let mut pwm = config.tim2.pwm_hz::<Tim2NoRemap, _, _>(
            (pwma0, pwma1, pwma2, pwma3),
            config.mapr,
            10.kHz(),
            config.clocks,
        );

        // Enable clock on each of the channels
        // https://docs.rs/stm32f1xx-hal/0.10.0/stm32f1xx_hal/timer/index.html
        pwm.enable(Channel::C1);
        pwm.enable(Channel::C2);
        pwm.enable(Channel::C3);
        pwm.enable(Channel::C4);

        // 设置频率
        pwm.set_period(10.kHz());
        // 获取最大占空比
        let max_duty = pwm.get_max_duty();

        Tb6612fng { pwm, max_duty }
    }

    /// 设置占空比
    /// channel: 设置通道, C1-C4
    /// speed: 设置速度, 1-10
    pub fn set_duty(&mut self, channel: Channel, speed: u8) {
        let speed = self.max_duty as f32 / 10.0 * speed as f32;
        self.pwm.set_duty(channel, speed as u16);
    }

    /// 初始化电机的运动方向
    /// todo: 待定, 如果默认方向可用，则不用设置
    #[allow(unused)]
    fn init_direction(
        pb0: PB0,
        pb1: PB1,
        pb12: PB12,
        pb13: PB13,
        gpiob_crl: &'a mut gpio::Cr<'B', false>,
        gpiob_crh: &'a mut gpio::Cr<'B', true>,
    ) {
        // 控制直流电机的运动方向,
        // 当AIN1设为高电平（或1）而AIN2设为低电平（或0）时，电机将正转
        let mut ain1 = pb0.into_push_pull_output(gpiob_crl);
        let mut ain2 = pb1.into_push_pull_output(gpiob_crl);
        ain1.set_speed(gpiob_crl, IOPinSpeed::Mhz50);
        ain2.set_speed(gpiob_crl, IOPinSpeed::Mhz50);

        // 设置正转方向
        ain1.set_high();
        ain2.set_low();

        let mut ao1 = pb12.into_push_pull_output(gpiob_crh);
        let mut ao2 = pb13.into_push_pull_output(gpiob_crh);
        ao1.set_speed(gpiob_crh, IOPinSpeed::Mhz50);
        ao2.set_speed(gpiob_crh, IOPinSpeed::Mhz50);

        // 设置正转方向
        ao1.set_high();
        ao2.set_low();
    }
}
