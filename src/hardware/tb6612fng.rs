//! TB6612FNG 电机驱动
//! 用于驱动直流电机；
//! 左上角为电机1号, 1-4号电机顺时针方向进行排序；
//! 1、3号电机为顺时针方向旋转，2、4号电机为逆时针方向旋转；

use stm32f1xx_hal::{
    afio::MAPR,
    gpio::{self, Alternate, IOPinSpeed, OutputSpeed, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
    pac::TIM2,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    timer::{Ch, Channel, PwmExt, PwmHz, Tim2NoRemap},
};

use crate::config::{
    REMOTE_CONTROL_PITCH_MAX, REMOTE_CONTROL_ROLL_MAX, REMOTE_CONTROL_THROTTLE_MAX,
    REMOTE_CONTROL_YAW_MAX,
};

/// 配置
pub struct Config<'a> {
    pub pa0: PA0,
    pub pa1: PA1,
    pub pa2: PA2,
    pub pa3: PA3,
    pub pa4: PA4,
    pub pa5: PA5,
    pub pa6: PA6,
    pub pa7: PA7,
    pub crl: &'a mut gpio::Cr<'A', false>,
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
        let pwma0 = config.pa0.into_alternate_push_pull(config.crl);
        let pwma1 = config.pa1.into_alternate_push_pull(config.crl);
        let pwma2 = config.pa2.into_alternate_push_pull(config.crl);
        let pwma3 = config.pa3.into_alternate_push_pull(config.crl);

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

        let mut tb6612fng = Tb6612fng { pwm, max_duty };

        // 初始化电机的运动方向
        tb6612fng.init_motor_direction(config.pa4, config.pa5, config.pa6, config.pa7, config.crl);
        tb6612fng
    }

    /// 设置占空比
    /// channel: 设置通道, C1-C4
    /// duty: 占空比
    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        self.pwm.set_duty(channel, duty);
    }

    /// 获取占空比
    pub fn get_duty(&mut self, channel: Channel) -> u16 {
        self.pwm.get_duty(channel)
    }

    /// 直流电机1
    /// duty: 占空比
    pub fn set_motor1(&mut self, duty: u16) {
        self.set_duty(Channel::C1, duty);
    }
    /// 直流电机2
    /// duty: 占空比
    pub fn set_motor2(&mut self, duty: u16) {
        self.set_duty(Channel::C2, duty);
    }
    /// 直流电机3
    /// duty: 占空比
    pub fn set_motor3(&mut self, duty: u16) {
        self.set_duty(Channel::C3, duty);
    }
    /// 直流电机4
    /// duty: 占空比
    pub fn set_motor4(&mut self, duty: u16) {
        self.set_duty(Channel::C4, duty);
    }
    /// 直流电机1-4
    /// duty: 占空比
    pub fn set_all_motor(&mut self, duty: u16) {
        self.set_duty(Channel::C1, duty);
        self.set_duty(Channel::C2, duty);
        self.set_duty(Channel::C3, duty);
        self.set_duty(Channel::C4, duty);
    }

    /// 初始化电机的运动方向
    /// 由于引脚紧张, 因此仅使用一个引脚(AIN1/BIN1)控制方向，即单向
    /// 可以调整电机接线进行正反转
    #[allow(unused)]
    pub fn init_motor_direction(
        &mut self,
        pa4: PA4,
        pa5: PA5,
        pa6: PA6,
        pa7: PA7,
        crl: &'a mut gpio::Cr<'A', false>,
    ) {
        // 控制直流电机的运动方向,
        // 当AIN1设为高电平（或1）而AIN2设为低电平（或0）时，电机将正转
        let mut motor1_ain1 = pa4.into_push_pull_output(crl);
        motor1_ain1.set_speed(crl, IOPinSpeed::Mhz50);
        motor1_ain1.set_high();

        let mut motor2_bin1 = pa5.into_push_pull_output(crl);
        motor2_bin1.set_speed(crl, IOPinSpeed::Mhz50);
        motor2_bin1.set_high();

        let mut motor3_ain1 = pa6.into_push_pull_output(crl);
        motor3_ain1.set_speed(crl, IOPinSpeed::Mhz50);
        motor3_ain1.set_high();

        let mut motor4_bin1 = pa7.into_push_pull_output(crl);
        motor4_bin1.set_speed(crl, IOPinSpeed::Mhz50);
        motor4_bin1.set_high();
    }
}

/// 电机翻转方向枚举
pub enum Orientation {
    /// 左右偏航
    Yaw,
    /// 上下俯仰
    Pitch,
    /// 左右横滚
    Roll,
    /// 上下油门
    Throttle,
}

/// 电机翻转方向控制
impl Tb6612fng {
    /// 控制电机翻转-左右偏航
    pub fn flip_yaw(&mut self, signal_value: u16) {
        let divide = REMOTE_CONTROL_YAW_MAX / 2;
        // 不偏航
        if signal_value == divide {
            return;
        }

        // 右偏航
        if signal_value > divide {
            let duty = signal_value / (signal_value - divide) * self.max_duty;
            self.set_motor1(self.max_duty / 2 + duty); // +
            self.set_motor2(self.max_duty / 2 - duty); // -
            self.set_motor3(self.max_duty / 2 + duty); // +
            self.set_motor4(self.max_duty / 2 - duty); // -
            return;
        }

        // 左偏航
        let duty = signal_value / divide * self.max_duty;
        self.set_motor1(self.max_duty / 2 - duty); // -
        self.set_motor2(self.max_duty / 2 + duty); // +
        self.set_motor3(self.max_duty / 2 - duty); // -
        self.set_motor4(self.max_duty / 2 + duty); // +
    }

    /// 控制电机翻转-上下俯仰
    pub fn flip_pitch(&mut self, signal_value: u16) {
        let divide = REMOTE_CONTROL_PITCH_MAX / 2;
        // 不俯仰
        if signal_value == divide {
            return;
        }

        // 下仰
        if signal_value > divide {
            let duty = signal_value / (signal_value - divide) * self.max_duty;
            self.set_motor1(self.max_duty / 2 + duty); // +
            self.set_motor2(self.max_duty / 2 + duty); // +
            self.set_motor3(self.max_duty / 2 - duty); // -
            self.set_motor4(self.max_duty / 2 - duty); // -
            return;
        }

        // 上俯
        let duty = signal_value / divide * self.max_duty;
        self.set_motor1(self.max_duty / 2 - duty); // -
        self.set_motor2(self.max_duty / 2 - duty); // -
        self.set_motor3(self.max_duty / 2 + duty); // +
        self.set_motor4(self.max_duty / 2 + duty); // +
    }

    /// 控制电机翻转-左右横滚
    pub fn flip_roll(&mut self, signal_value: u16) {
        let divide = REMOTE_CONTROL_ROLL_MAX / 2;
        // 不横滚
        if signal_value == divide {
            return;
        }

        // 右横滚
        if signal_value > divide {
            let duty = signal_value / (signal_value - divide) * self.max_duty;
            self.set_motor1(self.max_duty / 2 + duty); // +
            self.set_motor2(self.max_duty / 2 - duty); // -
            self.set_motor3(self.max_duty / 2 - duty); // -
            self.set_motor4(self.max_duty / 2 + duty); // +
            return;
        }

        // 左横滚
        let duty = signal_value / divide * self.max_duty;
        self.set_motor1(self.max_duty / 2 - duty); // -
        self.set_motor2(self.max_duty / 2 + duty); // +
        self.set_motor3(self.max_duty / 2 + duty); // +
        self.set_motor4(self.max_duty / 2 - duty); // -
    }

    /// 控制电机翻转-上下油门
    pub fn flip_throttle(&mut self, signal_value: u16) {
        let divide = REMOTE_CONTROL_THROTTLE_MAX / 2;
        // 不加油门
        if signal_value == divide {
            return;
        }

        // 上拉油门
        if signal_value > divide {
            let duty = signal_value / (signal_value - divide) * self.max_duty;
            self.set_all_motor(duty); // +
            return;
        }

        // 下拉油门
        let duty = signal_value / divide * self.max_duty;
        self.set_all_motor(duty); // -
    }

    /// 控制电机翻转
    pub fn flip(&mut self, orientation: Orientation, _signal_value: u16) {
        // 使用match语句，匹配参数的值
        match orientation {
            Orientation::Yaw => {}
            Orientation::Pitch => {}
            Orientation::Roll => {}
            Orientation::Throttle => {}
        }
    }
}
