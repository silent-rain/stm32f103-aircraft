//! NRF24L01 SPI - 2.4 GHz 无线通信

use core::convert::Infallible;

use crate::config::{NRF24L01_RX_ADDR, NRF24L01_RX_ADDR_P0, NRF24L01_TX_ADDR};

use heapless::String;

use embedded_nrf24l01::{Configuration, CrcMode, DataRate, Payload, RxMode, StandbyMode, NRF24L01};
use stm32f1xx_hal::{
    afio::MAPR,
    gpio::{self, Alternate, Input, OpenDrain, Output, PullUp, PB3, PB4, PB5, PB6, PB7},
    pac::SPI1,
    prelude::_fugit_RateExtU32,
    rcc::Clocks,
    spi::{self, Spi, Spi1Remap},
};

type Device = NRF24L01<
    Infallible,
    PB6<Output<OpenDrain>>,
    PB7<Output<OpenDrain>>,
    Spi<SPI1, Spi1Remap, (PB3<Alternate>, PB4<Input<PullUp>>, PB5<Alternate>), u8>,
>;

pub type RxTY = RxMode<Device>;

/// NRF24L01 传输指令
#[derive(Debug)]
pub struct NRF24L01Cmd {
    pub cmd: i32,
}

/// 配置参数
pub struct Config<'a> {
    pub spi_sck: PB3,
    pub spi_miso: PB4,
    pub spi_mosi: PB5,
    pub nrf24_ce: PB6,
    pub nrf24_csn: PB7,
    pub crl: &'a mut gpio::Cr<'B', false>,
    pub spi1: SPI1,
    pub mapr: &'a mut MAPR,
    pub clocks: Clocks,
}

/// NRF24L01 2.4G 无线通信
pub struct Nrf24L01 {
    pub nrf24: StandbyMode<Device>,
}

impl Nrf24L01 {
    /// 初始化 NRF24L01 SPI 2.4 GHz 无线通信
    pub fn new(config: Config) -> Self {
        // 创建一个SPI实例
        let spi = {
            let sck = config.spi_sck.into_alternate_push_pull(config.crl);
            let miso = config.spi_miso.into_pull_up_input(config.crl);
            let mosi = config.spi_mosi.into_alternate_push_pull(config.crl);

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

        let ce = config.nrf24_ce.into_open_drain_output(config.crl);
        let csn = config.nrf24_csn.into_open_drain_output(config.crl);

        let nrf24 = NRF24L01::new(ce, csn, spi).unwrap();

        let mut nrf24l01 = Nrf24L01 { nrf24 };

        // 配置设备
        nrf24l01.init_config();

        nrf24l01
    }

    /// 配置设备
    fn init_config(&mut self) {
        // 配置自动重新传输
        self.nrf24.set_auto_retransmit(0, 0).unwrap();
        // 设置 nRF24 无线模块的通信速率为 2 Mbps，并设置数据包的重传次数为 3
        self.nrf24.set_rf(&DataRate::R2Mbps, 3).unwrap();
        // 配置要启用的RX管道
        self.nrf24
            .set_pipes_rx_enable(&[true, false, false, false, false, false])
            .unwrap();
        // 为所有RX管道配置自动确认
        self.nrf24.set_auto_ack(&[false; 6]).unwrap();
        // 设置CRC模式
        self.nrf24.set_crc(CrcMode::Disabled).unwrap();

        // 设置TX管道的地址
        self.nrf24.set_tx_addr(NRF24L01_TX_ADDR).unwrap();
        self.nrf24
            .set_rx_addr(NRF24L01_RX_ADDR_P0, NRF24L01_RX_ADDR)
            .unwrap();
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
}
