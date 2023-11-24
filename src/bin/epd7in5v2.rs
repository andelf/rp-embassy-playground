//! SSD1306 OLED

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
use embedded_hal::blocking::delay::DelayMs;
use epd::display::{DisplaySize400x300, DisplaySize800x480, FrameBuffer};
use epd::drivers::{PervasiveDisplays, SSD1619A, UC8176};
use epd::interface::{DisplayInterface, EPDInterfaceNoCS};
use epd::{EPDInterface, FastUpdateEPD, EPD};
use heapless::String;
use rp::font::FONT_MUZAI_PIXEL;

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

    fn busy_wait(&mut self) {
        loop {
            self.send_command(0x71);
            if self.busy.is_high() {
                // negative logic
                break;
            }
        }
    }

    fn init(&mut self) {
        // Power setting
        self.send_command_data(0x01, &[0x07, 0x07, 0x3f, 0x3f]);

        // power on
        self.send_command(0x04);
        self.busy_wait();

        // panel setting
        // KW-3f   KWR-2F BWROTP 0f BWOTP 1f
        self.send_command_data(0x00, &[0x0F]);

        // tres
        // 800x480
        self.send_command_data(0x61, &[0x03, 0x20, 0x01, 0xE0]);

        // NO need to dual SPI

        // VCOM and data interval setting
        self.send_command_data(0x50, &[0x11, 0x07]);

        // Tcon setting
        self.send_command_data(0x60, &[0x22]);
    }

    fn clear(&mut self) {
        self.send_command(0x10);
        for i in 0..48000 {
            self.send_data(&[0xff]);
        }
        self.send_command(0x13);
        for i in 0..48000 {
            self.send_data(&[0x00]);
        }
    }

    fn display_frame(&mut self, buf: &[u8]) {
        self.send_command(0x10);
        self.send_data(buf);

        //self.send_command(0x13);
        //for _ in 0..20000 {
        //    self.send_data(&[0x00]);
        //}
        //self.send_data(buf);
    }

    fn refresh(&mut self) {
        let mut delay = Delay;
        self.send_command(0x12);
        delay.delay_ms(100_u32); //must
        self.busy_wait();
    }

    fn sleep(&mut self) {
        self.send_command(0x02); //power off
        self.busy_wait();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

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
    config.frequency = 20_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc.degrade(), Level::Low);
    let mut rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy.degrade(), Pull::None);

    let mut epd = EPD7in5v2 { spi, dc, busy };

    let mut fb: FrameBuffer<DisplaySize800x480> = epd::display::FrameBuffer::new();

    fb.fill(BinaryColor::Off);

    for i in 0..40 {
        Text::with_alignment(
            "观自在菩萨 行深般若波罗蜜多时",
            Point::new(50 + i * 20, i * 13 + 10), //skip 1 line
            MonoTextStyle::new(&FONT_MUZAI_PIXEL, BinaryColor::Off),
            Alignment::Left,
        )
        .draw(&mut fb)
        .unwrap();
    }

    // init
    rst.set_low();
    delay.delay_ms(100_u32); // at least 10ms
    rst.set_high();
    delay.delay_ms(100_u32); // at least 10ms

    epd.init();

    epd.clear();

    epd.display_frame(fb.as_bytes());

    epd.refresh();

    epd.sleep();

    loop {
        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
        info!("led toggle");
    }
}
