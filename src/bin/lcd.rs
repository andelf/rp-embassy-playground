//! SSD1306 OLED

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
use embassy_time::{Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use rp::st7049a::{self, Color3};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let miso = p.PIN_16; // not used
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let dc = p.PIN_20;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 2_000_000;
    let spi = Spi::new_blocking(p.SPI0, clk, mosi, miso, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc, Level::Low);

    let mut di = SPIInterface::new(spi, dc, cs);

    let mut disp = st7049a::Display::new(di);

    disp.clear();
    disp.flush();

    // init
    // 退出休眠模式
    //    Text::
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(Color3::Pink)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    // disp.bounding_box()
    //    .into_styled(border_stroke)
    //    .draw(&mut disp)
    //   .unwrap();

    /*
    for y in 0..16 {
        for x in 0..64 {
            disp.draw_pixel(x, y, Color3::Blue);
            Timer::after(Duration::from_millis(100)).await;
            disp.flush();
        }
    }
    */

    info!("=> {:?}", disp.buf);

    disp.clear();

    for (y, c) in (0..16).zip(
        [
            Color3::White,
            Color3::Yellow,
            Color3::Pink,
            Color3::Red,
            Color3::Cyan,
            Color3::Green,
            Color3::Blue,
            Color3::Black,
            Color3::White,
            Color3::Yellow,
            Color3::Pink,
            Color3::Red,
            Color3::Cyan,
            Color3::Green,
            Color3::Blue,
            Color3::Black,
        ]
        .iter(),
    ) {
        for x in 0..64 {
            disp.draw_pixel(x, y, *c);
        }
        disp.flush();
        Timer::after(Duration::from_millis(100)).await;
    }

    loop {
        Timer::after(Duration::from_millis(1000)).await;
        led.toggle();
        info!("led toggle");
    }
}
