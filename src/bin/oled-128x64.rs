//! SSD1306 128x64 OLED

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c::{self, Config};
use embassy_time::{Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use embedded_hal::blocking::i2c::Write as _;

use display_interface::DisplayError;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};
use ssd1306::{
    command::Command,
    mode::{TerminalDisplaySize, TerminalMode},
    prelude::*,
    I2CDisplayInterface, Ssd1306,
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    info!("set up i2c ");
    let i2c = i2c::I2c::new_blocking(p.I2C1, scl, sda, Config::default());
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().unwrap();

    core::write!(display, "Hello, {}", "world");
    for c in 97..123 {
        let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
    }
    for c in 65..91 {
        let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
    }

    let mut buf = heapless::String::<128>::new();
    let _ = core::write!(buf, "Hello, {}", "world");
    loop {
        info!("i2c write");

        Timer::after(Duration::from_millis(1000)).await;

        display.clear();
        display.write_str(&buf);
    }
}
