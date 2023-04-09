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
use embassy_time::{Delay, Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9};
use embedded_graphics::mono_font::iso_8859_13::FONT_7X13;
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
use epd::display::DisplaySize400x300;
use epd::drivers::{PervasiveDisplays, SSD1619A, UC8176};
use epd::interface::{DisplayInterface, EPDInterfaceNoCS};
use epd::{EPDInterface, FastUpdateEPD, EPD};
use heapless::String;

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
    let dc = Output::new(dc, Level::Low);
    let rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy, Pull::None);

    let di = EPDInterfaceNoCS::new(spi, dc, rst, busy);

    // let mut display: FastUpdateEPD<_, DisplaySize400x300, SSD1619A> = FastUpdateEPD::new(di);
    // let mut display: EPD<_, DisplaySize400x300, UC8176> = EPD::new(di);

    let mut display: FastUpdateEPD<_, DisplaySize400x300, PervasiveDisplays> = FastUpdateEPD::new(di);
    display.set_rotation(180);
    display.init(&mut delay);
    display.display_frame();

    let mut buf = String::<256>::new();

    let mut loop_cnt = 0;

    let mut values = [0.0f32; 100];
    let mut cursor = 0;

    let mut p26 = p.PIN_26; // A0

    loop {
        loop_cnt += 1;
        buf.clear();
        //let temp = adc.read_temperature().await;
        // let val = rp::convert_to_celsius(temp);
        //core::write!(buf, "Temp: {:.3} degrees", val);

        let val = rp::convert_to_voltage(adc.read(&mut p26).await);
        core::write!(buf, "UV: {:.4} V", val);

        values[cursor] = val;
        cursor += 1;
        if cursor >= values.len() {
            cursor = 0;
        }

        let style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BLACK).build();
        Text::with_alignment(
            &buf,
            Point::new(
                200 + ((loop_cnt * loop_cnt + loop_cnt * 3) % 200 - 100),
                70 + ((loop_cnt * loop_cnt * loop_cnt + loop_cnt * 5) % 100 - 50),
            ),
            style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // draw wave graph from values

        let max = values.iter().fold(0.0f32, |acc, &v| acc.max(v));
        let min = values.iter().fold(100.0f32, |acc, &v| acc.min(v));

        let range = max - min;
        if range > 0.05 {
            let mut last = None;
            // only draw if range is big enough
            for (idx, v) in values[cursor..].iter().chain(values[..cursor].iter()).enumerate() {
                let y = 240 - ((v - min) / range * 100.0) as i32;
                let x = 4 * idx as i32;
                let p = Point::new(x, y);
                // Pixel(p, BLACK).draw(&mut display).unwrap();
                /*Circle::new(p, 5)
                .into_styled(PrimitiveStyle::with_fill(BLACK))
                .draw(&mut display)
                .unwrap();*/
                match last {
                    Some(last) => Line::new(last, p)
                        .into_styled(PrimitiveStyle::with_stroke(BLACK, 1))
                        .draw(&mut display)
                        .unwrap(),
                    None => {}
                }
                last = Some(p);
            }

            buf.clear();
            core::write!(buf, "{:.3} V", min);
            let style = MonoTextStyleBuilder::new().font(&FONT_7X13).text_color(BLACK).build();
            Text::with_alignment(
                &buf,
                Point::new(5 + loop_cnt * 3 % 200, 240 + 9),
                style,
                Alignment::Left,
            )
            .draw(&mut display)
            .unwrap();
            buf.clear();
            core::write!(buf, "{:.3} V", max);
            Text::with_alignment(
                &buf,
                Point::new(5 + loop_cnt * 3 % 200, 140 - 9),
                style,
                Alignment::Left,
            )
            .draw(&mut display)
            .unwrap();
        }

        // refresh
        if loop_cnt % 5 == 1 {
            info!("full update");
            display.display_frame_full_update();
        } else {
            display.display_frame();
        }
        display.clear(WHITE);

        Timer::after(Duration::from_millis(5000)).await;
        led.toggle();
        info!("led toggle");
    }
}
