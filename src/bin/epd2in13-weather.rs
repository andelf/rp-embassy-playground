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
use embedded_drivers::bh1750::BH1750;
use embedded_drivers::bme280::BME280;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9, FONT_7X13, FONT_7X13_BOLD, FONT_7X13_ITALIC};
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

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    let mut i2c = i2c::I2c::new_blocking(p.I2C1, scl, sda, Config::default());
    info!("set up i2c ");

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

    let di = EPDInterface::new(spi, cs, dc, rst, busy);

    let mut display: FastUpdateEPD<_, DisplaySize122x250, IL3895> = FastUpdateEPD::new(di);

    display.set_rotation(270);
    display.init(&mut delay);
    display.display_frame();

    defmt::info!("disp ok");

    let mut buf = String::<128>::new();

    let style = MonoTextStyleBuilder::new()
        //.font(&FONT_SEG7_30X48)
        .font(&FONT_10X20)
        .text_color(BLACK)
        .background_color(WHITE) // clear bg when drawing
        .build();

    let mut bme280 = BME280::new(i2c, 0x76);
    bme280.init(&mut delay);
    info!("BME280 init OK");

    let start_time = Instant::now();

    /*let mut bh1750 = BH1750::new(i2c, 0x23);
    bh1750.init();
    Timer::after(Duration::from_millis(100)).await;
    let lux = bh1750.read().unwrap();
    println!("lux: {}", lux);*/

    let mut c = 0;

    loop {
        display.clear(BinaryColor::On);

        Line::new(Point::new(0, 14), Point::new(250, 14))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 1))
            .draw(&mut display)
            .unwrap();
        Line::new(Point::new(0, 122 - 14), Point::new(250, 122 - 14))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 1))
            .draw(&mut display)
            .unwrap();

        let measurements = bme280.measure(&mut delay).unwrap();

        buf.clear();

        core::write!(
            buf,
            "Temp: {:.2} 'C\nHum:  {:.2} %RH\nPres: {:.2} hPa\n",
            measurements.temperature,
            measurements.humidity,
            measurements.pressure / 100.0
        )
        .unwrap();

        let raw_temp = adc.read_temperature().await;
        let temp = rp::convert_to_celsius(raw_temp);
        core::write!(buf, "\n\nMCU Temp: {:.2} 'C", temp).unwrap();

        Text::new(
            &buf,
            Point::new(120, 30),
            //MonoTextStyle::new(&FONT_7X13, BinaryColor::Off).,
            MonoTextStyleBuilder::new()
                .font(&FONT_7X13)
                .text_color(BLACK)
                .background_color(WHITE) // clear bg when drawing
                .build(),
        )
        .draw(&mut display)
        .unwrap();

        // heading line
        Text::with_alignment(
            "Fri Apr 14 2:09 AM",
            Point::new(3, 11), //skip 1 line
            MonoTextStyle::new(&FONT_7X13_ITALIC, BinaryColor::Off),
            Alignment::Left,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            "UP ",
            Point::new(170, 11), //skip 1 line
            MonoTextStyle::new(&FONT_7X13_BOLD, BinaryColor::Off),
            Alignment::Left,
        )
        .draw(&mut display)
        .unwrap();

        buf.clear();
        //      core::write!(buf, "UP: ");
        let elapsed = start_time.elapsed().as_secs();
        rp::format_time(elapsed as _, &mut buf);
        Text::with_alignment(
            &buf,
            Point::new(170 + 7 * 3, 11), //skip 1 line
            MonoTextStyle::new(&FONT_7X13, BinaryColor::Off),
            Alignment::Left,
        )
        .draw(&mut display)
        .unwrap();


        // footer line
        Text::with_alignment(
            "观自在菩萨 行深般若波罗蜜多时",
            Point::new(3, 120), //skip 1 line
            MonoTextStyle::new(&FONT_MUZAI_PIXEL, BinaryColor::Off),
            Alignment::Left,
        )
        .draw(&mut display)
        .unwrap();



        c += 1;
        if c % 20 == 1 {
            display.display_frame_full_update();
        } else {
            //            display.display_frame_full_update();
            display.display_frame();
        }
        Timer::after(Duration::from_millis(10_000)).await;
        led.toggle();
    }
}
