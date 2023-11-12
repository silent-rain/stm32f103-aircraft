//! NRF24L01 SPI - 2.4 GHz 无线通信

use core::convert::Infallible;

use heapless::String;

use embedded_nrf24l01::{Configuration, CrcMode, DataRate, Payload, RxMode, StandbyMode, NRF24L01};
use stm32f1xx_hal::{
    afio::MAPR,
    gpio::{self, Alternate, Input, OpenDrain, Output, Pin, PullUp, PA3, PA4, PA5, PA6, PA7},
    pac::SPI1,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    spi::{self, Spi, Spi1NoRemap},
};

/// NRF24L01 对象别名
pub type NRF24L01TY = StandbyMode<
    NRF24L01<
        Infallible,
        Pin<'A', 3, Output<OpenDrain>>,
        Pin<'A', 4, Output<OpenDrain>>,
        Spi<
            SPI1,
            Spi1NoRemap,
            (
                Pin<'A', 5, Alternate>,
                Pin<'A', 6, Input<PullUp>>,
                Pin<'A', 7, Alternate>,
            ),
            u8,
        >,
    >,
>;

/// RxTY 对象别名
pub type RxTY = RxMode<
    NRF24L01<
        Infallible,
        Pin<'A', 3, Output<OpenDrain>>,
        Pin<'A', 4, Output<OpenDrain>>,
        Spi<
            SPI1,
            Spi1NoRemap,
            (
                Pin<'A', 5, Alternate>,
                Pin<'A', 6, Input<PullUp>>,
                Pin<'A', 7, Alternate>,
            ),
            u8,
        >,
    >,
>;

/// NRF24L01 传输指令
#[derive(Debug)]
pub struct NRF24L01Cmd {
    pub cmd: i32,
}

/// 配置参数
pub struct Config<'a> {
    pub pa5: PA5,
    pub pa6: PA6,
    pub pa7: PA7,
    pub gpioa_crl: &'a mut gpio::Cr<'A', false>,
    pub pa3: PA3,
    pub pa4: PA4,
    pub spi1: SPI1,
    pub mapr: &'a mut MAPR,
    pub clocks: Clocks,
}

/// 初始化 NRF24L01 SPI 2.4 GHz 无线通信
pub fn init(config: Config) -> NRF24L01TY {
    // 创建一个SPI实例
    let spi = {
        let sck = config.pa5.into_alternate_push_pull(config.gpioa_crl);
        let miso = config.pa6.into_pull_up_input(config.gpioa_crl);
        let mosi = config.pa7.into_alternate_push_pull(config.gpioa_crl);

        let mode = spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        };

        Spi::spi1(
            config.spi1,
            (sck, miso, mosi),
            config.mapr,
            mode,
            1.MHz(),
            config.clocks,
        )
    };

    let ce = config.pa3.into_open_drain_output(config.gpioa_crl);
    let csn = config.pa4.into_open_drain_output(config.gpioa_crl);

    let mut nrf24 = NRF24L01::new(ce, csn, spi).unwrap();

    // 配置设备
    // 配置自动重新传输
    nrf24.set_auto_retransmit(0, 0).unwrap();
    // 设置 nRF24 无线模块的通信速率为 2 Mbps，并设置数据包的重传次数为 3
    nrf24.set_rf(&DataRate::R2Mbps, 3).unwrap();
    // 配置要启用的RX管道
    nrf24
        .set_pipes_rx_enable(&[true, false, false, false, false, false])
        .unwrap();
    // 为所有RX管道配置自动确认
    nrf24.set_auto_ack(&[false; 6]).unwrap();
    // 设置CRC模式
    nrf24.set_crc(CrcMode::Disabled).unwrap();

    // 设置TX管道的地址
    nrf24.set_tx_addr(&b"fnord"[..]).unwrap();

    nrf24
}

/// 接收数据转换为字符串
/// 最大长度: 32
pub fn payload_string(payload: Payload) -> String<32> {
    let mut s: String<32> = String::new();
    let data = payload.as_ref();
    for byte in data {
        s.push(*byte as char).unwrap();
    }
    s
}
