//! SSD1305 OLED 128x33 display example

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

#[derive(Debug, Copy, Clone)]
pub struct DisplaySize128x33;
impl DisplaySize for DisplaySize128x33 {
    const WIDTH: u8 = 128;
    const HEIGHT: u8 = 33;
    type Buffer = [u8; Self::WIDTH as usize * Self::HEIGHT as usize / 8];

    fn configure(&self, iface: &mut impl WriteOnlyDataCommand) -> Result<(), DisplayError> {
        Command::ComPinConfig(true, false).send(iface)
    }

    const DRIVER_COLS: u8 = 128;

    const DRIVER_ROWS: u8 = 33;

    const OFFSETX: u8 = 0;

    const OFFSETY: u8 = 31;
}

impl TerminalDisplaySize for DisplaySize128x33 {
    const CHAR_NUM: u8 = 64;
}

mod ssd1305 {
    pub const SET_CONTRAST: u8 = 0x81;
    pub const SET_ENTIRE_ON: u8 = 0xA4;
    pub const SET_NORM_INV: u8 = 0xA6;
    pub const SET_DISP: u8 = 0xAE;
    pub const SET_MEM_ADDR: u8 = 0x20;
    pub const SET_COL_ADDR: u8 = 0x21;
    pub const SET_PAGE_ADDR: u8 = 0x22;
    pub const SET_DISP_START_LINE: u8 = 0x40;
    pub const SET_LUT: u8 = 0x91;
    pub const SET_SEG_REMAP: u8 = 0xA0;
    pub const SET_MUX_RATIO: u8 = 0xA8;
    pub const SET_MASTER_CONFIG: u8 = 0xAD;
    pub const SET_COM_OUT_DIR: u8 = 0xC0;
    pub const SET_COMSCAN_DEC: u8 = 0xC8;
    pub const SET_DISP_OFFSET: u8 = 0xD3;
    pub const SET_COM_PIN_CFG: u8 = 0xDA;
    pub const SET_DISP_CLK_DIV: u8 = 0xD5;
    pub const SET_AREA_COLOR: u8 = 0xD8;
    pub const SET_PRECHARGE: u8 = 0xD9;
    pub const SET_VCOM_DESEL: u8 = 0xDB;
    pub const SET_CHARGE_PUMP: u8 = 0x8D;
}

fn write_cmd<I>(iface: &mut I, cmd: &[u8]) -> Result<(), DisplayError>
where
    I: embedded_hal::blocking::i2c::Write,
{
    for b in cmd {
        iface.write(0x3c, &[0x80, *b]);
    }
    Ok(())
}

fn write_data<I>(iface: &mut I, cmd: &[u8]) -> Result<(), DisplayError>
where
    I: embedded_hal::blocking::i2c::Write,
{
    for b in cmd {
        iface.write(0x3c, &[0x00, *b]);
    }
    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    info!("set up i2c ");
    let mut i2c = i2c::I2c::new_blocking(p.I2C1, scl, sda, Config::default());

    const ADDR: u8 = 0x3c;
    write_cmd(&mut i2c, &[ssd1305::SET_DISP | 0x00]); // display off

    Timer::after(Duration::from_millis(1000)).await;

    // write_cmd(&mut i2c, &[ssd1305::SET_DISP | 0x01]); // display on
    /*
    write_cmd(&mut i2c, &[ssd1305::SET_DISP_CLK_DIV, 0x00]); // 0x80

    write_cmd(&mut i2c, &[ssd1305::SET_SEG_REMAP | 0x00]);

    write_cmd(&mut i2c, &[ssd1305::SET_MUX_RATIO, 0x20]);

    write_cmd(&mut i2c, &[ssd1305::SET_DISP_OFFSET, 0x1F]);

    write_cmd(&mut i2c, &[ssd1305::SET_MASTER_CONFIG, 0x8e]);

    write_cmd(&mut i2c, &[ssd1305::SET_AREA_COLOR, 0x05]);

    write_cmd(&mut i2c, &[ssd1305::SET_MEM_ADDR, 0x00]);

    write_cmd(&mut i2c, &[ssd1305::SET_DISP_START_LINE | 0x00, ]); //?

    write_cmd(&mut i2c, &[ssd1305::SET_COMSCAN_DEC]);

    write_cmd(&mut i2c, &[ssd1305::SET_COM_PIN_CFG, 0x12]);

    write_cmd(&mut i2c, &[ssd1305::SET_LUT, 0x3f, 0x3f, 0x3f, 0x3f]);

    write_cmd(&mut i2c, &[ssd1305::SET_CONTRAST, 0xff]);

    write_cmd(&mut i2c, &[ssd1305::SET_PRECHARGE, 0xd2]);

    write_cmd(&mut i2c, &[ssd1305::SET_VCOM_DESEL, 0x34]);

    write_cmd(&mut i2c, &[ssd1305::SET_NORM_INV]);

    write_cmd(&mut i2c, &[ssd1305::SET_ENTIRE_ON]);

    write_cmd(&mut i2c, &[ssd1305::SET_CHARGE_PUMP, 0x10]); // or 0x14
    */

    #[rustfmt::skip]
    write_cmd(
        &mut i2c,
        &[
            0xAE, 0xD5, 0x00,
            0xA8, 0x20,
            0xD3, 0x1F, // vcom shift, 0~63
            0x40, 0xAD, 0x8E, 0xD8, 0x05,
            0xA0,
            0xC8,//com output scan direction,  up down
            0xDA, 0b00_00_0010,
            0x91, 0x3F, 0x3F, 0x3F, 0x3F, 0x81, 0x6F, 0xD9, 0xD2, 0xDB, 0x1E, 0xA4,
            0xA6,
        ],
    );

    // invert display
    write_cmd(&mut i2c, &[ssd1305::SET_NORM_INV | 0x01]);

    write_cmd(&mut i2c, &[ssd1305::SET_DISP | 0x01]); // display on

    // show. offset
    //write_cmd(&mut i2c, &[ssd1305::SET_COL_ADDR, 0, 127]);
    //write_cmd(&mut i2c, &[ssd1305::SET_PAGE_ADDR, 3, 7]); // self.height // 8

    // write_framebuf

    //write_data(&mut i2c, &[0xaa; 128 * 32 / 8]);

    for i in 0..5 {
        write_cmd(&mut i2c, &[0xb0 + i, 0x00]); //设置低列起始地址
        write_cmd(&mut i2c, &[0x10]); //设置高列起始地址
        write_data(&mut i2c, &[0xaa; 128]);
    }

    /*
    let mut display =
        Ssd1306::new(interface, DisplaySize128x33, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().unwrap();

    core::write!(display, "Hello, {}", "world");
    for c in 97..123 {
        let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
    }
    for c in 65..91 {
        let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
    }
    */

    loop {
        info!("i2c write");
        // i2c.write(0x3c, &[0x00, 0x00]).unwrap();
        Timer::after(Duration::from_millis(400)).await;
    }
}
