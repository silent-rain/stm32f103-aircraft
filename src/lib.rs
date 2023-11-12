#![no_std]
#![no_main]

use defmt_rtt as _;
// global logger
use panic_probe as _;
// adjust HAL import
// memory layout
use stm32f1xx_hal as _;

pub mod hardware;
