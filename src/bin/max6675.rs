//! SSD1306 OLED

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]
#![feature(generic_const_exprs)]

use core::fmt::Write;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_rp::interrupt;
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9};
use embedded_graphics::mono_font::iso_8859_2::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::{Alignment, TextStyleBuilder};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use epd::display::DisplaySize400x300;
use epd::drivers::{PervasiveDisplays, SSD1619A, UC8176};
use epd::interface::{DisplayInterface, EPDInterfaceNoCS};
use epd::{EPDInterface, FastUpdateEPD, EPD};
use heapless::String;

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut delay = Delay;

    let irq = interrupt::take!(ADC_IRQ_FIFO);
    let mut adc = Adc::new(p.ADC, irq, adc::Config::default());

    let miso = p.PIN_16;
    let clk = p.PIN_18;
    let mut csn = Output::new(p.PIN_19, Level::High);

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let mut spi = Spi::new_blocking_rxonly(p.SPI0, clk, miso, config);

    let mut buf = [0u8; 2];

    loop {
        // refresh
        csn.set_low();
        Timer::after(Duration::from_millis(10)).await; //must
        spi.blocking_read(&mut buf);
        csn.set_high();

        let val = ((buf[0] as u16) << 8) | buf[1] as u16;

        if val & 0x04 != 0 {
            println!("no thermocouple attached!");
        } else {
            let temp = (val >> 3) as f32 * 0.25;
            println!("temp: {}'C", temp);
        }

        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
        info!("led toggle");
    }
}
