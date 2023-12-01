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
use embassy_rp::spi::{self, Spi};
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
use rp::epd7in5bv2::EPD7in5;
use rp::font::{FONT_MUZAI_PIXEL, FONT_SEG7_30X48, FONT_WQY16};

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

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

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc.degrade(), Level::Low);
    let mut rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy.degrade(), Pull::None);

    //  let mut epd = EPD7in5v2 { spi, dc, busy };

    let spi_dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs);
    let mut epd = EPD7in5::new(spi_dev, dc, busy);

    // init
    rst.set_low();
    delay.delay_ms(100_u32); // at least 10ms
    rst.set_high();
    delay.delay_ms(100_u32); // at least 10ms

    epd.init(rp::epd7in5bv2::Config::mode_bw());
    epd.configure_init_update();

    epd.clear();
    epd.refresh(&mut delay);
    info!("Init ok");
    epd.configure_partial_update();

    let (w, h, raw) = text_image::text_image!(
        text = "一月 感恩节 十八 30\n且将新火试新茶。\n诗酒趁年华。\nabcdefghijk\n1234567890",
        font = "./msyh.ttf",
        font_size = 32.0,
        line_spacing = 0,
        inverse,
        Gray2,
    );

    epd.set_partial_refresh(Rectangle::new(Point::new(400, 24), Size::new(w, h)));
    //    epd.configure_partial_update();
    //  epd.display_frame(raw);
    // epd.refresh();
    // epd.refresh_gray4_image(raw);
    epd.refresh_gray2_image(raw, &mut delay);

    let (w, h, raw) = text_image::text_image!(
        text = "一月 感恩节 十八 30\n且将新火试新茶。\n诗酒趁年华。\nabcdefghijk\n1234567890",
        font = "./msyh.ttf",
        font_size = 32.0,
        line_spacing = 0,
        inverse,
        Gray4,
    );

    epd.set_partial_refresh(Rectangle::new(Point::new(400, 200), Size::new(w, h)));
    //epd.configure_partial_update();
    //epd.display_frame(&[0b01010101; 224 * 160 / 4]);
    //epd.refresh();
    //    epd.refresh_gray2_image(&[0b01010101; 224 * 160 / 4]);
    // epd.refresh_gray4_image(raw);
    epd.refresh_gray4_image(raw, &mut delay);

    loop {
        led.toggle();
        Timer::after(Duration::from_millis(1000)).await;
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
