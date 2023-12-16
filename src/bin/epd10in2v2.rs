//! 10in2 BWR
//!
//! ping pong buf
//!
//! Display Mode 2

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
            // info!("busy");
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
        self.send_command_data(0x18, &[0x80]);

        // set vcom value
        self.send_command_data(0x2C, &[0x44]);

        // setting X direction start/end position of RAM
        // 640 -> 639 => 0x27F
        // 960 -> 959 => 0x3BF
        self.send_command_data(0x44, &[0x00, 0x00, 0xBF, 0x03]); // 长
                                                                 // setting Y direction start/end position of RAM
        self.send_command_data(0x45, &[0x00, 0x00, 0x7F, 0x02]);
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);

        // self.send_command_data(0x37, &[0x00; 10]); // ALL Mode 1

        // Display Option
        #[rustfmt::skip]
        self.send_command_data(0x37, &[
            0x00,
            0xFF, //B
            0xFF, //C
            0xFF, //D
            0xFF, //E
            // 0x0F, RAM ping-pong disable
            // 0x4F, RAM ping-pong enable
            0x4F, //F, RAM ping-pong enable. only in Display Mode 2, or 0x4F
            0xFF, //G
            0xFF, //H
            0xFF, //I
            0xFF, //J
        ]); // MODE 2

        // Load Waveform
        // 0x91, Load LUT with Mode 1
        /*
        self.send_command_data(0x22, &[0x91]);
        self.send_command(0x20);
        self.busy_wait();

        // Display Update Control 2
        self.send_command_data(0x22, &[0xCF]);
        */

        self.send_command_data(0x22, &[0x99]); // Load LUT with Mode 2
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
            self.send_data(&[0xff]); // W
        }

        // reset X/Y ram counter
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);
        self.send_command(0x26);
        for i in 0..NBUF {
            self.send_data(&[0x00]); // Red off
        }
    }

    fn clear_as_bw_mode(&mut self) {
        // set X/Y ram counter
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);

        const NBUF: usize = 960 * 640 / 8;
        self.send_command(0x24);
        for i in 0..NBUF {
            self.send_data(&[0xFF]); // W
        }

        // reset X/Y ram counter
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);
        self.send_command(0x26);
        for i in 0..NBUF {
            self.send_data(&[0xFF]); // Red off
        }
    }

    pub fn update_bw_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        //  self.send_command_data(0x4E, &[0x00, 0x00]);
        //    self.send_command_data(0x4F, &[0x00, 0x00]);

        self.send_command(0x24);
        self.send_data(buf);
    }

    pub fn update_red_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        //        self.send_command_data(0x4E, &[0x00, 0x00]);
        //      self.send_command_data(0x4F, &[0x00, 0x00]);

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
        // clear old buf
        self.clear_as_bw_mode();
        let x0 = (rect.top_left.x as u16);
        let x1 = rect.bottom_right().unwrap().x as u16;
        let y0 = rect.top_left.y as u16;
        let y1 = rect.bottom_right().unwrap().y as u16;

        self.send_command_data(0x44, &[(x0 & 0xff) as u8, (x0 >> 8) as u8, (x1 & 0xff) as u8, (x1 >> 8) as u8]);
        self.send_command_data(0x45, &[(y0 & 0xff) as u8, (y0 >> 8) as u8, (y1 & 0xff) as u8, (y1 >> 8) as u8]);

        // set X/Y ram counter
        self.send_command_data(0x4E, &[(x0 & 0xff) as u8, (x0 >> 8) as u8]);
        self.send_command_data(0x4F, &[(y0 & 0xff) as u8, (y0 >> 8) as u8]);
        //self.send_command_data(0x4E, &[0x00, 0x00]);
        //self.send_command_data(0x4F, &[0x00, 0x00]);
    }

    pub fn unset_partial_refresh(&mut self) {
        self.send_command_data(0x44, &[0x00, 0x00, 0xBF, 0x03]);
        self.send_command_data(0x45, &[0x00, 0x00, 0x7F, 0x02]);
        self.send_command_data(0x4E, &[0x00, 0x00]);
        self.send_command_data(0x4F, &[0x00, 0x00]);
    }

    pub fn power_off(&mut self) {
        // Deep sleep
        self.send_command_data(0x10, &[0b11]);
        //  self.busy_wait();
    }

    // LUTs
    /// Partial Update & Quick Update
    pub fn configure_partial_update(&mut self) {
        #[rustfmt::skip]
        const LUT: &[u8] = &[
            0b00_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT0, B2B
            0b10_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT1, B2W
            0b01_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT2, W2B
            0b00_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT3. W2W
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//5
            // TP[xA, xB, xC, xD], RP
            0x18,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,//7
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,//9
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            // FR
            0x22,0x22,0x22,0x22,0x22
        ];
        self.send_command_data(0x32, LUT);
    }

    /// Level 0 to 15
    fn configure_gray_update_level(&mut self, level: u8) {
        #[rustfmt::skip]
        let lut: &[u8] = &[
            0b01_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT0, B2B
            0b00_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT1, B2W
            0b01_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT2, W2B
            0b00_00_00_00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//LUT3. W2W
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//5
            // TP[xA, xB, xC, xD], RP
            level,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,//7
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,//9
            0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,
            // FR
            0x22,0x22,0x22,0x22,0x22
        ];
        self.send_command_data(0x32, lut);
    }

    pub fn refresh_gray4_image(&mut self, buf: &[u8]) {
        for level in (0..4).rev() {
            // level: (8, 4, 2, 1)
            self.configure_gray_update_level(1 << level);
            self.send_command(0x24);

            for chunk in buf.chunks(4) {
                let mut n = 0;
                for b in chunk {
                    if b & (0x10 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (1 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                }
                // 0xFF is white, 0x00 is black
                self.send_data(&[n]);
            }

            self.refresh();
        }
    }

    pub fn refresh_gray2_image(&mut self, buf: &[u8]) {
        for level in [1, 0] {
            // level: 9, 5
            self.configure_gray_update_level(1 << (level + 2) + 1);
            self.send_command(0x24);

            for chunk in buf.chunks(2) {
                let mut n = 0;
                for b in chunk {
                    if b & (0b01_00_00_00 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (0b00_01_00_00 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (0b00_00_01_00 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (0b00_00_00_01 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                }
                // 0xFF is white, 0x00 is black
                self.send_data(&[n]);
            }

            self.refresh();
        }
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

    info!("init ok");

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
    epd.clear();
    epd.refresh();
    info!("clear ok!!!");

    let (w, h, raw) =
        text_image::text_image!(
            text = "烟水茫茫，\n千里斜阳暮。\n山无数。\n乱红如雨",
            font = "./徐静蕾手写体.ttf",
            font_size = 48.0,
            line_spacing = 0,
            inverse,
            Gray4,
        );

    info!("w: {}, h: {}", w, h);

    epd.set_partial_refresh(Rectangle::new(Point::new(536, 160), Size::new(w, h)));

    epd.refresh_gray4_image(raw);

    loop {}

    epd.configure_partial_update();

    let mut fb = Framebuffer::<
        BinaryColor,
        _,
        LittleEndian,
        960,
        640,
        { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(960, 640) },
    >::new();
    fb.clear(BinaryColor::On);

    for h in 0..10 {
        Text::with_alignment(
            "般若波罗蜜多心经 般若波罗蜜多心经",
            Point::new(100, 100 + h * 20),
            MonoTextStyle::new(&FONT_WQY16, BinaryColor::Off), // black
            Alignment::Left,
        )
        .draw(&mut fb)
        .unwrap();
        epd.update_bw_frame(fb.data());
        epd.refresh();
        // Timer::after_millis(1000).await;

        info!("1 frame");
    }
    // loop {}

    let (w, h, raw) = text_image::text_image!(
        text = "一月 感恩节 十八 30\n且将新火试新茶。\n诗酒趁年华。\nabcdefghijk\n1234567890",
        font = "./徐静蕾手写体.ttf",
        font_size = 32.0,
        line_spacing = -2,
        inverse,
    );
    epd.set_partial_refresh(Rectangle::new(Point::new(400, 200), Size::new(w, h)));

    //    epd.configure_partial_update();
    //  epd.display_frame(raw);
    // epd.refresh();
    // epd.refresh_gray4_image(raw);
    epd.update_bw_frame(raw);

    epd.refresh();
    epd.update_bw_frame(raw);

    epd.refresh();

    // .....

    {
        // counter
        let w = 200;
        let h = 24;
        let mut buf = heapless::String::<16>::new();
        let mut n = 0;
        let mut fb = Framebuffer::<
            BinaryColor,
            _,
            LittleEndian,
            200,
            24,
            { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(200, 24) },
        >::new();
        fb.clear(BinaryColor::On);

        epd.set_partial_refresh(Rectangle::new(Point::new(600, 400), Size::new(w, h)));

        loop {
            buf.clear();
            fb.clear(BinaryColor::On);

            core::write!(&mut buf, "Check {}", n).unwrap();

            Text::with_alignment(
                &buf,
                Point::new(2, 21),
                MonoTextStyle::new(&FONT_10X20, BinaryColor::Off), // black
                Alignment::Left,
            )
            .draw(&mut fb)
            .unwrap();

            epd.update_bw_frame(fb.data());
            epd.refresh();
            //Timer::after_millis(2000).await;
            n += 1;
            // info!("tick11");
        }
    }

    loop {
        led.toggle();
        Timer::after_millis(1000).await;

        info!("tick");
    }

    loop {
        led.toggle();
        Timer::after_millis(1000).await;

        info!("tick");
    }

    //  init BWR
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

    Timer::after_millis(1000).await;

    let (w, h, raw) = text_image::text_image!(
        text = "一月 感恩节 十八 30\n且将新火试新茶。\n诗酒趁年华。\nabcdefghijk\n1234567890",
        font = "./徐静蕾手写体.ttf",
        font_size = 32.0,
        line_spacing = -2,
        inverse,
    );
    epd.set_partial_refresh(Rectangle::new(Point::new(400, 24), Size::new(w, h)));
    //    epd.configure_partial_update();
    //  epd.display_frame(raw);
    // epd.refresh();
    // epd.refresh_gray4_image(raw);
    epd.update_bw_frame(raw);

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
