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

pub enum Color3 {
    White = 0,
    Yellow = 1,
    Pink = 2,
    Red = 3,
    Cyan = 4,
    Green = 5,
    Blue = 6,
    Black = 7,
}

const WIDTH: u32 = 64;
const HEIGHT: u32 = 16;
const PIXELS: usize = 320 * 4 / 2; //1个字节包含2个像素的数据

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

    // init
    // 退出休眠模式
    di.send_commands(DataFormat::U8(&[0x11]));
    di.send_commands(DataFormat::U8(&[0xD2]));
    di.send_data(DataFormat::U8(&[0x00]));

    di.send_commands(DataFormat::U8(&[0xc0]));
    di.send_data(DataFormat::U8(&[140, 0]));

    di.send_commands(DataFormat::U8(&[0xB0]));
    di.send_data(DataFormat::U8(&[0x03]));

    di.send_commands(DataFormat::U8(&[0xB2]));
    di.send_data(DataFormat::U8(&[0x1a]));

    di.send_commands(DataFormat::U8(&[0xB5]));
    di.send_data(DataFormat::U8(&[0x4, 1, 1, 1]));

    di.send_commands(DataFormat::U8(&[0xB6])); // led waveform
                                               //di.send_data(DataFormat::U8(&[20, 20, 20, 200, 200, 200]));
    di.send_data(DataFormat::U8(&[20, 20, 20, 255, 255, 255]));

    di.send_commands(DataFormat::U8(&[0xB7]));
    di.send_data(DataFormat::U8(&[0x40]));

    di.send_commands(DataFormat::U8(&[0x29]));

    let mut buf: [u8; PIXELS] = [0x00; PIXELS];

    //let mut seed = 0;

    for i in 0..50 {
        buf[i] = 0x07;
    }

    loop {
        //  for i in 0..PIXELS {
        //      buf[i] = (seed + seed * seed + i + i % 3 + i % 7 + i * i) as u8;
        //  }
        //  seed += 1;

        // update
        di.send_commands(DataFormat::U8(&[0x2A])); // column addr
        di.send_data(DataFormat::U8(&[0, 159]));

        di.send_commands(DataFormat::U8(&[0x2B])); // raw addr
        di.send_data(DataFormat::U8(&[0, 3]));

        di.send_commands(DataFormat::U8(&[0x2C])); // write data
        di.send_data(DataFormat::U8(&buf));

        Timer::after(Duration::from_millis(100)).await;
        led.toggle();
        info!("flush!");
    }

    loop {
        Timer::after(Duration::from_millis(1000)).await;
        led.toggle();
        info!("led toggle");
    }
}
