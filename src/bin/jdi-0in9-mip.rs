//! Memory LCD - LPM009M360A

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Alignment;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};
use fixed::traits::ToFixed;
use rp::lpm009m360a::{Rgb111, LPM009M360A};

// NOTE: actually it's 6 bit command, the next optional part is 10 bit, so 2 bytes command is possible
const CMD_NO_UPDATE: u8 = 0x00;
const CMD_BLINKING_BLACK: u8 = 0x10;
const CMD_BLINKING_INVERSION: u8 = 0x14;
const CMD_BLINKING_WHITE: u8 = 0x18;
const CMD_ALL_CLEAR: u8 = 0x20;
const CMD_VCOM: u8 = 0x40;
const CMD_UPDATE: u8 = 0x90;

/* eight color
#define COLOR_BLACK             0x00
#define COLOR_BLUE              0x02
#define COLOR_GREEN             0x04
#define COLOR_CYAN              0x06
#define COLOR_RED               0x08
#define COLOR_MAGENTA           0x0a
#define COLOR_YELLOW            0x0c
#define COLOR_WHITE             0x0e


 */

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mosi = p.PIN_19; // SI
    let clk = p.PIN_18; // SCLK
    let cs = p.PIN_17; // SCS
    let disp = p.PIN_20;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 10_000_000;
    let mut spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let mut cs = Output::new(cs, Level::Low);
    let mut disp: Output<_> = Output::new(disp, Level::Low);

    let mut display = LPM009M360A::new();

    disp.set_high(); //disp on

    cs.set_high();
    spi.blocking_write(&[CMD_ALL_CLEAR, 0x00]);
    cs.set_low();

    info!("cleared");

    info!("gogogo");

    // refresh?
    for i in 0..144 {
        cs.set_high();
        spi.blocking_write(&[CMD_UPDATE, i]);
        spi.blocking_write(&[0x88; 72 / 2]); // R
        spi.blocking_write(&[0x00, 0x00]);
        cs.set_low();
        Timer::after(Duration::from_millis(5)).await;
    }

    let mut offs = 20;

    let colors =
        [
            Rgb111::WHITE,
            Rgb111::RED,
            Rgb111::MAGENTA,
            Rgb111::YELLOW,
            Rgb111::GREEN,
            Rgb111::BLUE,
            Rgb111::CYAN,
            Rgb111::BLACK,
        ];
    let mut color_index = 0;
    loop {
        //display.clear(colors[(offs + 3) % colors.len()]);
        display.clear(Rgb111::BLACK);

        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(colors[color_index])
            .background_color(colors[(color_index + 2) % colors.len()])
            .build();

        Text::with_alignment("Hello\nWorld", Point::new(30, offs as i32), style, Alignment::Center)
            .draw(&mut display)
            .unwrap();

        for (i, row) in display.iter_rows().enumerate() {
            cs.set_high();
            spi.blocking_write(&[CMD_UPDATE, i as u8]);
            spi.blocking_write(row); // blue
            spi.blocking_write(&[0x00, 0x00]);
            cs.set_low();
        }
        //Timer::after(Duration::from_millis(1000)).await;
        offs += 1;

        if offs > 160 {
            color_index += 1;
            offs = 0;
        }
        if color_index >= colors.len() {
            color_index = 0;
        }
        led.toggle();
    }

    let mut rev = false;
    loop {
        rev = !rev;
        Timer::after(Duration::from_millis(1000)).await;
        info!("blink");
    }
}
