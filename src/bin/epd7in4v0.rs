//! SES imagotag G1 7.4 BWR NFC x2 EDG1-0740-A
//!
//! See-also: https://github.com/rei-vilo/PDLS_EXT3_Basic_Global/blob/main/src/Screen_EPD_EXT3.cpp
//! The code seems to be based on reverse engineering of the some firmware.
//!
//! https://www.e-shelf-labels.com/hardware/electronic-shelf-labels/g1-electronic-labels/g1-7-4-bwr.html

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]
#![feature(generic_const_exprs)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_rp::interrupt;
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{self, Blocking, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::framebuffer::Framebuffer;
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
use embedded_graphics_core::pixelcolor::raw::LittleEndian;
use embedded_graphics_core::pixelcolor::Gray4;
use embedded_hal::delay::DelayNs;
use heapless::String;
use rp::font::{FONT_MUZAI_PIXEL, FONT_SEG7_30X48, FONT_WQY16};

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

struct EPD7in5v2<'a> {
    spi: Spi<'a, SPI0, Blocking>,
    dc: Output<'a, AnyPin>,
    busy: Input<'a, AnyPin>,
}

impl EPD7in5v2<'_> {
    fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        self.spi.blocking_write(&[cmd]);
    }

    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.blocking_write(data);
    }

    fn send_command_data(&mut self, cmd: u8, data: &[u8]) {
        self.send_command(cmd);
        self.send_data(data);
    }

    pub fn busy_wait(&mut self) {
        loop {
            // negative logic
            if self.busy.is_high() {
                info!("busy out");
                break;
            }
        }
    }

    pub fn init(&mut self) {}

    pub fn clear(&mut self) {
        const NBUF: usize = 800 * 480 / 8;
        self.send_command(0x10);
        for i in 0..NBUF {
            self.send_data(&[0x00]);
        }

        self.send_command(0x11); // red
        for i in 0..NBUF {
            self.send_data(&[0x00]);
        }
    }

    /// 3.2 Send image to the EPD
    pub fn update_frame(&mut self, bw: &[u8], red: &[u8]) {
        // DUW
        self.send_command_data(0x13, &[0x00, 0x3b, 0x00, 0x00, 0x1f, 0x03]);
        // DRFW
        self.send_command_data(0x90, &[0x00, 0x3b, 0x00, 0xc9]);
        // RAM_RW
        self.send_command_data(0x12, &[0x3b, 0x00, 0x14]);

        self.send_command(0x10);
        self.send_data(bw);

        // RAM_RW
        self.send_command_data(0x12, &[0x3b, 0x00, 0x14]);

        // red
        self.send_command(0x11);
        self.send_data(red);
    }

    pub fn update_bw_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        self.send_command(0x10);
        self.send_data(buf);
    }

    pub fn update_red_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        self.send_command(0x11);
        self.send_data(buf);
    }

    pub fn refresh(&mut self) {
        // Initial COG
        self.send_command_data(0x05, &[0x7d]);
        Delay.delay_ms(200);
        self.send_command_data(0x05, &[0x00]);
        Delay.delay_ms(10);
        self.send_command_data(0xc2, &[0x3f]);
        Delay.delay_ms(1);
        self.send_command_data(0xd8, &[0x00]); // MS_SYNC
        self.send_command_data(0xd6, &[0x00]); // BVSS

        self.send_command_data(0xa7, &[0x10]);
        Delay.delay_ms(100);
        self.send_command_data(0xa7, &[0x00]);
        Delay.delay_ms(100);

        // OSC mtp_0x12
        self.send_command_data(0x03, &[0x00, 0x01]);

        self.send_command_data(0x44, &[0x00]);
        self.send_command_data(0x45, &[0x80]);
        self.send_command_data(0xa7, &[0x10]);
        Delay.delay_ms(100);
        self.send_command_data(0xa7, &[0x00]);
        Delay.delay_ms(100);

        self.send_command_data(0x44, &[0x06]);
        self.send_command_data(0x45, &[0x82]); // u_temperature
        self.send_command_data(0xa7, &[0x10]);
        Delay.delay_ms(100);
        self.send_command_data(0xa7, &[0x00]);
        Delay.delay_ms(100);

        self.send_command_data(0x60, &[0x25]); // TCON

        // STV_DIR mtp_0x1c
        self.send_command_data(0x61, &[0x00]); // STV_DIR

        // DCTL mtp_0x10
        self.send_command_data(0x01, &[0x00]); // DCTL
                                               // VCOM mtp_0x11
        self.send_command_data(0x02, &[0x00]); // VCOM

        // DC-DC soft-start
        // There are 32-bytes data for describing the sequence of soft-start.
        let mut buf0 = [0x50, 0x01, 0x0a, 0x01];
        let mut buf1 = [0x1f, 0x9f, 0x7f, 0xff];
        self.send_command_data(0x51, &buf0[0..2]);
        for v in 1..=4 {
            self.send_command_data(0x09, &buf1[0..1]);
            buf0[1] = v;
            self.send_command_data(0x51, &buf0[0..2]);
            self.send_command_data(0x09, &buf1[1..2]);
            Delay.delay_ms(2);
        }
        for v in 1..=10 {
            self.send_command_data(0x09, &buf1[0..1]);
            buf0[3] = v;
            self.send_command_data(0x51, &buf0[2..4]);
            self.send_command_data(0x09, &buf1[1..2]);
            Delay.delay_ms(2);
        }
        for v in 3..=10 {
            self.send_command_data(0x09, &buf1[2..3]);
            //buf0[1] = v;
            self.send_command_data(0x51, &buf0[2..4]);
            self.send_command_data(0x09, &buf1[3..4]);
            Delay.delay_ms(2);
        }
        for v in (2..=9).rev() {
            self.send_command_data(0x09, &buf1[2..3]);
            // buf0[1] = v;
            self.send_command_data(0x51, &buf0[2..4]);
            self.send_command_data(0x09, &buf1[3..4]);
            Delay.delay_ms(2);
        }
        self.send_command_data(0x09, &[0xff]);
        Delay.delay_ms(10);

        // display refresh start
        self.busy_wait();

        self.send_command_data(0x15, &[0x3c]); // display refresh
        Delay.delay_ms(5);
        self.busy_wait();

        self.power_off();
    }

    pub fn power_off(&mut self) {
        // Turn off DC-DC
        self.busy_wait();
        self.send_command_data(0x09, &[0x7F]);
        self.send_command_data(0x05, &[0x7D]);
        self.send_command_data(0x09, &[0x00]);
        Delay.delay_ms(200);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("init ok");

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut delay = Delay;

    let busy = p.PIN_16; // not used
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let dc = p.PIN_20;
    let rst = p.PIN_21;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc.degrade(), Level::Low);
    let mut rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy.degrade(), Pull::None);

    let mut epd = EPD7in5v2 { spi, dc, busy };

    // init
    rst.set_low();
    delay.delay_ms(200_u32); // at least 10ms
    rst.set_high();
    delay.delay_ms(200_u32); // at least 10ms

    epd.init();

    info!("epd init ok");
    Timer::after(Duration::from_millis(1000)).await;
    epd.clear();

    let (w, h, bw) = text_image::monochrome_image!("OIG.VC5NTXw.gif", channel = 0,);

    //epd.update_bw_frame(raw);

    let (w, h, red) = text_image::monochrome_image!("OIG.VC5NTXw.gif", channel = 2,);
    //
    //  epd.update_red_frame(raw);

    epd.update_frame(bw, red);

    info!("??");

    epd.refresh();

    loop {
        info!("tick");
        Timer::after(Duration::from_millis(1000)).await;
    }

    let mut fb = Framebuffer::<
        BinaryColor,
        _,
        LittleEndian,
        480,
        800,
        { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(480, 800) },
    >::new();
    fb.clear(BinaryColor::Off);

    Text::with_alignment(
        "2023-11-27",
        Point::new(50, 50),                                    //skip 1 line
        MonoTextStyle::new(&FONT_SEG7_30X48, BinaryColor::On), // black
        Alignment::Left,
    )
    .draw(&mut fb)
    .unwrap();

    epd.update_bw_frame(fb.data());
    /*
    let (w, h, raw) = text_image::monochrome_image!("remap_2.gif", channel = 2,);
    epd.update_red_frame(raw);
    */

    epd.refresh();
    info!("Init ok");

    loop {
        Timer::after(Duration::from_millis(100)).await;
        led.toggle();
        info!("led toggle");
    }
}

fn display_seconds(nsecs: i32) -> heapless::String<16> {
    // seconds to hh:mm:ss
    let mut buf = heapless::String::<16>::new();
    let mut n = nsecs;
    let h = n / 3600;
    n = n % 3600;
    let m = n / 60;
    n = n % 60;
    let s = n;
    core::write!(&mut buf, "{:02}:{:02}:{:02}", h, m, s).unwrap();
    buf
}
