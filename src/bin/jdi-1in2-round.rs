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
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::image::{Image, ImageRaw};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::raw::BigEndian;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Arc, PrimitiveStyleBuilder, Rectangle, Sector};
use embedded_graphics::text::Alignment;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};
use fixed::traits::ToFixed;
use rp::lpm013m126a::{Rgb222, LPM013M126A};

/*
19 XRST

22 VST
21 VCK

// PWM VCOM control
14 to VCOM and FRP
15 XFRP

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut conf = embassy_rp::config::Config::default();
    conf.clocks = embassy_rp::clocks::ClockConfig::rosc();
    let p = embassy_rp::init(conf);

    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("started");

    // pins
    let mut hst = Output::new(p.PIN_16, Level::Low);
    let mut hck = Output::new(p.PIN_6, Level::Low);

    let mut xrst = Output::new(p.PIN_19, Level::Low);
    // Write enable signal for the pixel memory
    let mut enb = Output::new(p.PIN_20, Level::Low);

    let mut vst = Output::new(p.PIN_22, Level::High);
    let mut vck = Output::new(p.PIN_21, Level::High);

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

    // VCOM(FRP/XFRP) driver via PWM
    // 7 Hz to 125 Mhz
    // requires 60Hz vcom
    let mut pwm_conf: pwm::Config = Default::default();
    pwm_conf.compare_a = 32768;
    pwm_conf.compare_b = 32768;
    pwm_conf.invert_b = true;
    pwm_conf.divider = 9i32.to_fixed();
    let _pwm = Pwm::new_output_ab(p.PWM_CH7, p.PIN_14, p.PIN_15, pwm_conf.clone());

    LPM013M126A::reset(&mut xrst, &mut delay);
    LPM013M126A::init(&mut vst, &mut vck, &mut hst, &mut hck, &mut enb);

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb222::CYAN)
        .build();
    Text::with_alignment("Hello World", Point::new(120, 120), style, Alignment::Center)
        .draw(&mut display)
        .unwrap();

    let _color = [
        Rgb222::WHITE,
        Rgb222::BLACK,
        Rgb222::RED,
        Rgb222::GREEN,
        Rgb222::BLUE,
        Rgb222::CYAN,
        Rgb222::MAGENTA,
        Rgb222::YELLOW,
    ];

    let raw_image: ImageRaw<Rgb222, BigEndian> = ImageRaw::new(include_bytes!("../../240x240.raw"), 240);
    Image::new(&raw_image, Point::new(1, 1)).draw(&mut display).unwrap();

    for i in 1..=64 {
        let c = Rgb222::from_raw(i);
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(c)
            .stroke_width(80)
            .fill_color(c)
            .build();
        Arc::new(
            Point::new(1, 1),
            240,
            (i as f32 * 6.0).deg(),
            6.0.deg(), //            ((i + 1) as f32 * 6.0).deg(),
        )
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

        //
        display.flush(
            &mut vst, &mut vck, &mut hst, &mut hck, &mut enb, &mut r1, &mut r2, &mut g1, &mut g2, &mut b1, &mut b2,
            &mut delay,
        );
        //       Timer::after(Duration::MIN).await;
        Timer::after(Duration::from_millis(0)).await;
        //future::ready(()).await;
        info!("toggle frame {}", i);
        led.toggle();
    }

    loop {
        Timer::after(Duration::from_millis(500)).await;
        info!("sleep");
        led.toggle();
    }
}
