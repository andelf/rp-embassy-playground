//! Memory LCD - LPM013M126A

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
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};

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

    // let miso = p.PIN_16; // not used
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let cs = p.PIN_17;
    let disp = p.PIN_20;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let mut spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let mut cs = Output::new(cs, Level::Low);
    let mut disp: Output<_> = Output::new(disp, Level::Low);

    disp.set_high(); //disp on

    // clear screen

    cs.set_high();
    Timer::after(Duration::from_millis(1)).await;

    spi.blocking_write(&[CMD_ALL_CLEAR, 0x00]);
    cs.set_low();

    // refresh?
    for i in 0..176 {
        cs.set_high();
        Timer::after(Duration::from_micros(1)).await; // cs setting time. 1us?

        spi.blocking_write(&[CMD_UPDATE]);
        spi.blocking_write(&[i]);
        spi.blocking_write(&[0xcc; 176 / 2]);
        spi.blocking_write(&[0x00, 0x00]);
        cs.set_low();
        Timer::after(Duration::from_millis(5)).await;
    }

    for i in (0..176).rev() {
        cs.set_high();
        Timer::after(Duration::from_micros(1)).await; // cs setting time. 1us?

        spi.blocking_write(&[CMD_UPDATE]);
        spi.blocking_write(&[i]);
        spi.blocking_write(&[0x66; 176 / 2]);
        spi.blocking_write(&[0x00, 0x00]);
        cs.set_low();
        Timer::after(Duration::from_millis(5)).await;
    }

    let mut b = 1;
    let mut rev = false;
    loop {
        for i in 0..176 {
            let line = if rev { 176 - i - 1 } else { i };

            // scan by interlacing
            let mut line: u16 = line * 2;
            if line >= 176 {
                line = line - 176 + 1;
            }

            b = (((line / 16 + i) % 8) << 1) as u8;

            cs.set_high();
            Timer::after(Duration::from_micros(1)).await; // cs setting time. 1us?

            spi.blocking_write(&[CMD_UPDATE, line as u8]);
            spi.blocking_write(&[(b << 4) + b; 176 / 2]);
            spi.blocking_write(&[0x00, 0x00]);
            cs.set_low();
        }

        b += 2;

        if b >= 0xf {
            b = 0;
        }
        led.toggle();
        rev = !rev;
        Timer::after(Duration::from_millis(1000)).await;
    }
}
