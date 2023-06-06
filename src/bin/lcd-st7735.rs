//! 1.4inch, 128x128, 13pin 0.3mm FPC.
//! BGR color, no invert

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Config};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::TextStyleBuilder;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use embedded_graphics_core::pixelcolor::Rgb565;
use mipidsi::RefreshOrder;

/*
1-IOVCC(3V3) 2-VCC(3V3) 3,4-GND 5-NC 6-SCL  7-RST
8-CS 9-SDA 10-DC 11-GND 12-BL+ 13-BL-
 */

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut delay = Delay;

    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let dc = p.PIN_20;
    let rst = p.PIN_16;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    config.phase = spi::Phase::CaptureOnFirstTransition;
    config.polarity = spi::Polarity::IdleLow;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::High);
    let dc = Output::new(dc, Level::High);
    let rst = Output::new(rst, Level::High);

    let di = SPIInterface::new(spi, dc, cs);

    let mut display = mipidsi::Builder::st7735s(di)
        .with_display_size(128, 128)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Normal)
        .init(&mut delay, Some(rst))
        .unwrap();
    info!("init ok");

    display.clear(Rgb565::WHITE).unwrap();

    loop {
        display.clear(Rgb565::RED).unwrap();
        Timer::after(Duration::from_millis(1000)).await;

        display.clear(Rgb565::GREEN).unwrap();
        Timer::after(Duration::from_millis(1000)).await;

        display.clear(Rgb565::BLUE).unwrap();
        Timer::after(Duration::from_millis(1000)).await;
        led.toggle();
        info!("led toggle");
    }
}
