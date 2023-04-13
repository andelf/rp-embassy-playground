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
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_rp::interrupt;
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9};
use embedded_graphics::mono_font::iso_8859_2::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::{Alignment, LineHeight, TextStyleBuilder};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use epd::display::{DisplaySize104x201, DisplaySize122x250, DisplaySize400x300};
use epd::drivers::{PervasiveDisplays, IL3895, SSD1608, SSD1619A, UC8176};
use epd::interface::{DisplayInterface, EPDInterfaceNoCS};
use epd::{EPDInterface, FastUpdateEPD, EPD};
use heapless::String;
use rp::font::{FONT_MUZAI_PIXEL, FONT_SEG7_30X48};

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut delay = Delay;

    let irq = interrupt::take!(ADC_IRQ_FIFO);
    let mut adc = Adc::new(p.ADC, irq, adc::Config::default());

    let busy = p.PIN_16; // not used
    let csn = p.PIN_17;
    let clk = p.PIN_18;
    let mosi = p.PIN_19;
    let dc = p.PIN_20;
    let rst = p.PIN_21;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc, Level::Low);
    let rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy, Pull::None);

    let di = EPDInterfaceNoCS::new(spi, dc, rst, busy);

    // let mut display: FastUpdateEPD<_, DisplaySize400x300, SSD1619A> = FastUpdateEPD::new(di);
    // let mut display: EPD<_, DisplaySize400x300, UC8176> = EPD::new(di);
    //let mut display: EPD<_, DisplaySize122x250, IL3895> = EPD::new(di);
    let mut display: FastUpdateEPD<_, DisplaySize122x250, IL3895> = FastUpdateEPD::new(di);
    // let mut display: EPD<_, DisplaySize122x250, SSD1608> = EPD::new(di);
    //let mut display: EPD<_, DisplaySize104x201, IL3895> = EPD::new(di);

    display.set_rotation(270);
    display.init(&mut delay);
    display.display_frame();

    defmt::info!("disp ok");

    let mut buf = String::<128>::new();
    let mut c = 0;

    let digit_style = MonoTextStyleBuilder::new()
        .font(&FONT_SEG7_30X48)
        .text_color(BLACK)
        .background_color(WHITE) // clear bg when drawing
        .build();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BLACK)
        .background_color(WHITE) // clear bg when drawing
        .build();

    let boot_time = Instant::now();

    loop {
        buf.clear();
        let elapsed = boot_time.elapsed().as_secs();
        rp::format_time(elapsed as _, &mut buf);
        c += 1;

        Text::with_text_style(
            &buf,
            Point::new(2, 40), //skip 1 line
            digit_style,
            TextStyleBuilder::new().line_height(LineHeight::Pixels(12)).build(),
        )
        .draw(&mut display)
        .unwrap();

        buf.clear();
        let raw_temp = adc.read_temperature().await;
        let temp = rp::convert_to_celsius(raw_temp);
        core::write!(buf, "Temp: {:.2}'C", temp);

        Text::with_alignment("Uptime:", Point::new(1, 16), text_style, Alignment::Left)
            .draw(&mut display)
            .unwrap();

        Text::with_text_style(
            &buf,
            Point::new(80, 100),
            text_style,
            TextStyleBuilder::new().line_height(LineHeight::Pixels(12)).build(),
        )
        .draw(&mut display)
        .unwrap();

        info!("begin update");
        if c % 30 == 1 {
            display.display_frame_full_update();
        } else {
            display.display_frame();
        }
        info!("update done");
        Timer::after(Duration::from_millis(10_000)).await;
    }

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(Duration::from_millis(4000)).await;

        info!("led off!");
        led.set_low();
        Timer::after(Duration::from_millis(4000)).await;
    }
}
