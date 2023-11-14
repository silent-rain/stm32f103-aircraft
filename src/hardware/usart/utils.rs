//! 串行接口常用工具集

use core::{convert::Infallible, u32};

use heapless::String;
use nb::block;
use stm32f1xx_hal::serial::{self, Instance, Rx, Tx};

/// 发送字节
pub fn send_byte<USART>(tx: &mut Tx<USART>, word: u8) -> Result<(), Infallible>
where
    USART: Instance,
{
    block!(tx.write(word))
}

/// 发送字节数组
/// 结束发送标识符: '\0'
pub fn send_bytes<USART>(tx: &mut Tx<USART>, words: &[u8]) -> Result<(), Infallible>
where
    USART: Instance,
{
    for word in words {
        if *word == b'\0' {
            break;
        }
        send_byte(tx, *word)?;
    }
    Ok(())
}

/// 发送字符串
/// 结束发送标识符: '\0'
pub fn send_string<USART>(tx: &mut Tx<USART>, words: &str) -> Result<(), Infallible>
where
    USART: Instance,
{
    for word in words.as_bytes() {
        if *word == b'\0' {
            break;
        }
        send_byte(tx, *word)?;
    }

    Ok(())
}

/// 发送数字
pub fn send_number<USART>(tx: &mut Tx<USART>, number: u32) -> Result<(), Infallible>
where
    USART: Instance,
{
    let mut length = 0;
    loop {
        length += 1;
        let rounding = number / (10_u32.pow(length));
        if rounding == 0 {
            break;
        }
    }

    for i in 0..length {
        let v = number / 10_u32.pow(length - i - 1) % 10 + '0' as u32;
        send_byte(tx, v as u8)?;
    }

    Ok(())
}

/// 接收字节
pub fn recv_byte<USART>(rx: &mut Rx<USART>) -> Result<u8, serial::Error>
where
    USART: Instance,
{
    block!(rx.read())
}

/// 接收字节数组
/// 最大长度: 4096
/// 结束接收标识符: '\n'
pub fn recv_bytes<'a, USART>(
    rx: &mut Rx<USART>,
    buffer: &'a mut [u8],
) -> Result<&'a [u8], serial::Error>
where
    USART: Instance,
{
    let mut widx: usize = 0;
    loop {
        let w = block!(rx.read())?;
        if w == b'\n' {
            break;
        }
        if widx < buffer.len() {
            buffer[widx] = w;
            widx += 1;
        }
    }

    Ok(buffer)
}

/// 接收字符串
/// 最大长度: 32
pub fn recv_string<USART>(rx: &mut Rx<USART>) -> Result<String<32>, serial::Error>
where
    USART: Instance,
{
    let mut s: String<32> = String::new();

    loop {
        let w = block!(rx.read())?;
        if w == b'\n' {
            break;
        }
        let _ = s.push(w as char);
    }
    Ok(s)
}
