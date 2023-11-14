//! USART1 通用串口
#![allow(unused)]
mod utils;

use core::convert::Infallible;

use cortex_m::peripheral::NVIC;
use heapless::String;
use stm32f1xx_hal::{
    afio::MAPR,
    gpio::{self, PA10, PA9},
    pac::{Interrupt, USART1},
    rcc::Clocks,
    serial::{self, Rx, Serial, StopBits, Tx},
    time::U32Ext,
};

pub use utils::*;

/// USART 串口
pub struct Usart {
    tx: Tx<USART1>,
    rx: Rx<USART1>,
}

impl Usart {
    /// 初始化 USART1 串口
    /// 固定引脚: PA9、PA10
    pub fn new(
        pa9: PA9,
        pa10: PA10,
        crh: &mut gpio::Cr<'A', true>,
        usart1: USART1,
        mapr: &mut MAPR,
        clocks: &Clocks,
        nvic: &mut NVIC,
    ) -> Self {
        let tx = pa9.into_alternate_push_pull(crh);
        let rx = pa10;

        // 设置usart设备。取得USART寄存器和tx/rx引脚的所有权。其余寄存器用于启用和配置设备。
        let (tx, rx) = Serial::new(
            usart1,
            (tx, rx),
            mapr,
            serial::Config::default()
                .baudrate(9600.bps())
                .wordlength_8bits()
                .stopbits(StopBits::STOP1)
                .parity_none(),
            clocks,
        )
        .split();

        // 使能中断
        unsafe {
            NVIC::unmask(Interrupt::USART1);
            nvic.set_priority(Interrupt::USART1, 1);
        }

        Usart { tx, rx }
    }

    /// 发送指令
    pub fn send_cmd(&mut self, cmd: u8) -> Result<(), Infallible> {
        send_byte(&mut self.tx, cmd)
    }

    /// 发送字节
    pub fn send_byte(&mut self, data: u8) -> Result<(), Infallible> {
        send_byte(&mut self.tx, data)
    }

    /// 发送字符串
    pub fn send_string(&mut self, data: &str) -> Result<(), Infallible> {
        for d in data.as_bytes() {
            if *d == b'\0' {
                send_byte(&mut self.tx, b'\n')?;
                break;
            }
            send_byte(&mut self.tx, *d)?;
        }
        Ok(())
    }

    /// 打印字符串到串口
    pub fn print_string(&mut self, data: &str) -> Result<(), Infallible> {
        for d in data.as_bytes() {
            if *d == b'\0' {
                break;
            }
            send_byte(&mut self.tx, *d)?;
        }

        Ok(())
    }

    /// 接收指令
    pub fn recv_cmd(&mut self) -> Result<u8, serial::Error> {
        recv_byte(&mut self.rx)
    }

    /// 接收字符串
    /// 最大长度: 32
    pub fn recv_string(&mut self) -> Result<String<32>, serial::Error> {
        recv_string(&mut self.rx)
    }
}
