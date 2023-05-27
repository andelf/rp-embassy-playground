//! Memory LCD - LPM013M126A drive

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;
use core::future;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Rectangle, Sector, PrimitiveStyleBuilder, Arc};
use embedded_graphics::text::Alignment;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};
use rp::lpm013m126a::{Rgb222, LPM013M126A};

/*
19 XRST

22 VST
21 VCK

14 VCOM - VCOM is driven by gpio
18 FRP
17 XFRP

20 ENB
16 HST
6 HCK

7 R1
8 R2
9 G1
10 G2
11 B1
12 B2
 */

// drive VCOM at 60Hz
// FRP is the same signal as VCOM, and XFRP is the inverse signal of VCOM and FRP.
#[embassy_executor::task(pool_size = 1)]
async fn vcom_drive(vcom: AnyPin, frp: AnyPin, xfrp: AnyPin) {
    let mut vcom = Output::new(vcom, Level::High);
    let mut frp = Output::new(frp, Level::High);
    let mut xfrp = Output::new(xfrp, Level::Low);

    // 60Hz = 16.666ms
    info!("vcom drive started");
    loop {
        vcom.set_high();
        frp.set_high();
        xfrp.set_low();
        Timer::after(Duration::from_millis(8)).await;
        vcom.set_low();
        frp.set_low();
        xfrp.set_high();
        Timer::after(Duration::from_millis(8)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("started");

    let raw1 = include_bytes!("../../240x240.raw");
    // let raw2 = include_bytes!("../../240x240.2.raw");
    // let raw2 = include_bytes!("../../color.raw");
    let raw2 = include_bytes!("../../test.raw");
    //let raw2 = include_bytes!("../../hot.raw");

    // pins
    let mut hst = Output::new(p.PIN_16, Level::Low);
    let mut hck = Output::new(p.PIN_6, Level::Low);

    // Write enable signal for the pixel memory
    let mut enb = Output::new(p.PIN_20, Level::Low);

    let mut vst = Output::new(p.PIN_22, Level::High);
    let mut xrst = Output::new(p.PIN_19, Level::Low);
    let mut vck = Output::new(p.PIN_21, Level::High);

    // Common electrode driving signal
    //let mut vcom = Output::new(p.PIN_14, Level::High);
    //let mut frp = Output::new(p.PIN_18, Level::Low);
    //let mut xfrp = Output::new(p.PIN_17, Level::Low);
    // vcom drive

    let mut r1 = Output::new(p.PIN_7, Level::Low);
    let mut r2 = Output::new(p.PIN_8, Level::Low);
    let mut g1 = Output::new(p.PIN_9, Level::Low);
    let mut g2 = Output::new(p.PIN_10, Level::Low);
    let mut b1 = Output::new(p.PIN_11, Level::Low);
    let mut b2 = Output::new(p.PIN_12, Level::Low);

    info!("io init ok");

    let mut delay = Delay;

    // initial state
    let mut display = LPM013M126A::new();

    LPM013M126A::init(&mut vst, &mut vck, &mut hst, &mut hck, &mut enb);

    spawner
        .spawn(vcom_drive(p.PIN_14.degrade(), p.PIN_18.degrade(), p.PIN_17.degrade()))
        .unwrap();

    LPM013M126A::reset(&mut xrst, &mut delay);

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb222::CYAN)
        .build();
    Text::with_alignment("Hello World", Point::new(120, 120), style, Alignment::Center)
        .draw(&mut display)
        .unwrap();

    let style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb222::RED)
        .stroke_width(20)
        .fill_color(Rgb222::GREEN)
        .build();
    for i in 0..360 {
        Arc::new(Point::new(1, 1), 240, 0.0.deg(), (i as f32).deg())
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        display.flush(
            &mut vst, &mut vck, &mut hst, &mut hck, &mut enb, &mut r1, &mut r2, &mut g1, &mut g2, &mut b1, &mut b2,
            &mut delay,
        );
        Timer::after(Duration::from_millis(50)).await;
        info!("toggle frame {}", i);
        led.toggle();
    }

    loop {
        //xrst.set_low(); // active display no update
        Timer::after(Duration::from_millis(500)).await;
        info!("toggle frame");
        led.toggle();
    }

    loop {}
}
