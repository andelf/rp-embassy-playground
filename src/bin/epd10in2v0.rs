//! 局部刷新, BW

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
use embedded_hal::delay::DelayNs;
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
use heapless::String;
use rp::font::{FONT_MUZAI_PIXEL, FONT_SEG7_30X48, FONT_WQY16};

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

struct EPD10in2<'a> {
    spi: Spi<'a, SPI0, Blocking>,
    dc: Output<'a, AnyPin>,
    busy: Input<'a, AnyPin>,
}

impl EPD10in2<'_> {
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

    // negative logic
    pub fn busy_wait(&mut self) {
        loop {
            if self.busy.is_low() {
                info!("busy out");
                break;
            }
        }
    }

    pub fn init(&mut self) {
        self.send_command(0x12); // Soft reset
        Delay.delay_ms(20_u32);

        self.send_command_data(0x46, &[0xF7]);
        self.busy_wait();

        self.send_command_data(0x47, &[0xF7]);
        self.busy_wait();

        // setting date number. 高度
        // Driver output control
        // 0x27F = 639
        self.send_command_data(0x01, &[0x7F, 0x02, 0x00]);

        // set gate voltage
        self.send_command_data(0x03, &[0x00]);
        // set source voltage
        self.send_command_data(0x04, &[0x41, 0xA8, 0x32]); // POR

        // set data entry sequence
        self.send_command_data(0x11, &[0x03]);

        // set border
        self.send_command_data(0x3C, &[0x03]);

        // set booster strength
        self.send_command_data(0x0C, &[0xAE, 0xC7, 0xC3, 0xC0, 0xC0]);

        // set internal sensor on
        self.send_command_data(0x18, &[0x80]); // use internal sensor

        // set vcom value
        self.send_command_data(0x2C, &[0x44]);

        //        self.send_command_data(0x37, &[0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x4F, 0xFF, 0xFF, 0xFF, 0xFF]); //???
        self.send_command_data(0x37, &[0x00; 10]); //???

        // setting X direction start/end position of RAM
        // 640 -> 639 => 0x27F
        // 960 -> 959 => 0x3BF
        self.send_command_data(0x44, &[0x00, 0x00, 0xBF, 0x03]); // 长

        // setting Y direction start/end position of RAM
        self.send_command_data(0x45, &[0x00, 0x00, 0x7F, 0x02]);

        // Load Waveform
        // 0x91, Load LUT with Mode 1
        self.send_command_data(0x22, &[0x91]);
        self.send_command(0x20);
        self.busy_wait();

        // Display Update Control 2
        self.send_command_data(0x22, &[0xCF]);
    }

    // Clear is required to set initial state of the panel.
    // Or else the panel will show random noise.
    pub fn clear(&mut self) {
        // set X/Y ram counter
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);

        const NBUF: usize = 960 * 640 / 8;
        self.send_command(0x24);
        for i in 0..NBUF {
            self.send_data(&[0xff]);
        }
        self.send_command(0x26);
        for i in 0..NBUF {
            self.send_data(&[0x00]);
        }
    }

    pub fn update_bw_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);

        self.send_command(0x24);
        self.send_data(buf);
    }

    pub fn update_red_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);

        self.send_command(0x26);
        self.send_data(buf);
    }

    pub fn refresh(&mut self) {
        let mut delay = Delay;
        self.send_command(0x20); // Master activation
        delay.delay_ms(100_u32); //must
        self.busy_wait();
    }

    pub fn sleep(&mut self) {
        self.send_command(0x02);
        self.busy_wait();
    }

    pub fn set_partial_refresh(&mut self, rect: Rectangle) {
        let x0 = (rect.top_left.x as u16) & 0b1111111000;
        let x1 = rect.bottom_right().unwrap().x as u16;
        let y0 = rect.top_left.y as u16 | 0b111;
        let y1 = rect.bottom_right().unwrap().y as u16;
        self.send_command_data(0x44, &[(x0 & 0xff) as u8, (x0 >> 8) as u8, (x1 & 0xff) as u8, (x1 >> 8) as u8]);
        self.send_command_data(0x45, &[(y0 & 0xff) as u8, (y0 >> 8) as u8, (y1 & 0xff) as u8, (y1 >> 8) as u8]);
    }

    pub fn unset_partial_refresh(&mut self) {
        const PARTIAL_OUT: u8 = 0x92;
        self.send_command(PARTIAL_OUT);
    }

    pub fn power_off(&mut self) {
        // Deep sleep
        self.send_command_data(0x10, &[0b11]);
        //  self.busy_wait();
    }

    // LUTs

    pub fn configure_default(&mut self) {
        #[rustfmt::skip]
        const LUT: &[u8] = &[
            // VS
            0x2A, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //1
            0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //2
            0x2A, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //3
            0x05, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //4
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //5
            // TP
            0x00, 0x02, 0x03, 0x0A, 0x00,
            0x02, 0x06, 0x0A, 0x05, 0x00, //6
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //7
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //8
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //9
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //10
            // FR
            0x22, 0x22, 0x22, 0x22, 0x22,
        ];

        self.send_command_data(0x32, LUT);
    }

    pub fn configure_init_clear(&mut self) {
        #[rustfmt::skip]
        const LUT_1GRAY_GC: &[u8] = &[
            // VS
            0x2A, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //1
            0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //2
            0x2A, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //3
            0x05, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //4
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //5
            // TP
            0x00, 0x02, 0x03, 0x0A, 0x00,
            0x02, 0x06, 0x0A, 0x05, 0x00, //6
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //7
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //8
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //9
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, //10
            // FR
            0x22, 0x22, 0x22, 0x22, 0x22,
        ];

        self.send_command_data(0x32, LUT_1GRAY_GC);
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
    config.frequency = 28_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    info!("spi created");

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc.degrade(), Level::Low);
    let mut rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy.degrade(), Pull::None);

    let mut epd = EPD10in2 { spi, dc, busy };

    // init
    rst.set_low();
    delay.delay_ms(100_u32); // at least 10ms
    rst.set_high();
    delay.delay_ms(100_u32); // at least 10ms

    epd.init();
    //epd.configure_init_clear();

    epd.clear();

    epd.refresh();

    let mut fb = Framebuffer::<
        BinaryColor,
        _,
        LittleEndian,
        960,
        640,
        { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(960, 640) },
    >::new();

    Timer::after(Duration::from_secs(1)).await;
    //epd.update_bw_frame(&[0x55; 960 * 640 / 8]);
    //epd.update_red_frame(&[0x55; 960 * 640 / 8]);

    let (w, h, raw) = text_image::monochrome_image!("remap_2.gif", channel = 1,);
    epd.update_bw_frame(raw);
    let (w, h, raw) = text_image::monochrome_image!("remap_2.gif", channel = 2,);
    epd.update_red_frame(raw);

    epd.refresh();

    loop {
        led.toggle();
        Timer::after_millis(1000).await;

        info!("tick");
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
