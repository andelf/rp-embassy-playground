//! FSC LCD, ST7049A driver, 32x16 dots, 4 colors.
//!
//! The screen has 64x32 pixels, but each dot is composed of 4 pixels.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Duration, Timer};
// use embedded_hal_1::i2c::I2c;
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
    config.frequency = 20_000_000;
    let spi = Spi::new_blocking(p.SPI0, clk, mosi, miso, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc, Level::Low);

    let mut di = SPIInterface::new(spi, dc, cs);

    let mut disp = st7049a::Display::new(di);
    disp.init();

    disp.clear(Color3::White);
    disp.flush();

    //  disp.set_bg_light(Color3::Yellow);

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

    //info!("=> {:?}", disp.buf);

    disp.clear(Color3::White);

    let colors = [
        Color3::Yellow,
        Color3::Pink,
        Color3::Red,
        Color3::Cyan,
        Color3::Green,
        Color3::White,
        Color3::Blue,
        Color3::Black,
        Color3::Cyan,
        Color3::Green,
        // Color3::White,
        Color3::Blue,
        Color3::Black,
    ];
    let mut cit = colors.iter().cycle();

    //let mut i = 0;
    let mut off = 64;
    loop {
        disp.clear(Color3::White);

        let s = "Hello World!";
        for i in 0..s.len() {
            let style = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(*cit.next().unwrap())
                .build();
            Text::new(&s[i..i + 1], Point::new(off + (i * 6) as i32, 11), style)
                .draw(&mut disp)
                .unwrap();
            // i += 1;
        }
        disp.flush();
        Timer::after(Duration::from_millis(50)).await;

        off -= 1;
        if off <= -75 {
            off = 64;
        }
    }

    loop {
        Timer::after(Duration::from_millis(1000)).await;
        led.toggle();
        info!("led toggle");
    }
}
