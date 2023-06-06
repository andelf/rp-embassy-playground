//! 1.4inch, 128x128, 13pin 0.3mm FPC.
//! BGR color, no invert

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;
use micromath::F32Ext;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Config};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9, FONT_7X14};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::TextStyleBuilder;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use embedded_graphics_core::pixelcolor::Rgb565;

/*
1-IOVCC(3V3) 2-VCC(3V3) 3,4-GND 5-NC 6-SCL  7-RST
8-CS 9-SDA 10-DC 11-GND 12-BL+ 13-BL-
 */

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut delay = Delay;

    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let dc = p.PIN_20;
    let rst = p.PIN_16;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 62_500_000;
    //    config.phase = spi::Phase::CaptureOnFirstTransition;
    //    config.polarity = spi::Polarity::IdleLow;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::High);
    let dc = Output::new(dc, Level::High);
    let rst = Output::new(rst, Level::High);

    let di = SPIInterface::new(spi, dc, cs);

    let mut display = mipidsi::Builder::st7735s(di)
        .with_display_size(128, 128)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Normal)
        .init(&mut delay, Some(rst))
        .unwrap();
    display.set_orientation(mipidsi::Orientation::PortraitInverted(false));
    info!("init ok");

    display.clear(Rgb565::WHITE).unwrap();

    const POINTS: usize = 8;

    // a 3D diamond shape
    let orig_points: [[f32; 3]; POINTS] = [
        [1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [1.0, 1.0, -1.0],
        [1.0, -1.0, -1.0],
        [-1.0, -1.0, -1.0],
        [-1.0, 1.0, -1.0],
    ];

    //  Perspective Projection:
    // distance to project plane
    let d = 1.0;

    let z_offset = -3.0; // offset on z axis, leave object

    let cube_size = 80.0;
    let screen_offset_x = 128.0 / 2.0;
    let screen_offset_y = 128.0 / 2.0;
    /*
    x' = (d * x) / z
    y' = (d * y) / z
     */
    let mut rotate_angle = 0.0_f32;

    let mut rotated_points = [[0.0; 3]; POINTS];
    let mut projected_points = [[0.0; 2]; POINTS];
    let mut points = [[0_i32; 2]; POINTS];

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X14)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    loop {
        for (i, [x, y, z]) in orig_points.iter().copied().enumerate() {
            // rotate along y axis
            rotated_points[i][0] = x * rotate_angle.cos() - z * rotate_angle.sin();
            rotated_points[i][1] = y;
            rotated_points[i][2] = x * rotate_angle.sin() + z * rotate_angle.cos();

            let rx = rotated_points[i][0];
            let ry = rotated_points[i][1];
            let rz = rotated_points[i][2] + z_offset;

            projected_points[i][0] = (d * rx) / rz;
            projected_points[i][1] = (d * ry) / rz;
        }

        for (i, [x, y]) in projected_points.iter().enumerate() {
            let x: f32 = (x * cube_size) + screen_offset_x;
            let y: f32 = (y * cube_size) + screen_offset_y;

            // manually round
            points[i][0] = (x + 0.5) as i32;
            points[i][1] = (y + 0.5) as i32;
        }

        display.clear(Rgb565::BLACK);

        Text::new("@andelf", Point::new(5, 20), text_style)
            .draw(&mut display)
            .unwrap();

        for ([idx, idy], color) in [
            ([0, 1], Rgb565::YELLOW),
            ([1, 2], Rgb565::CSS_DARK_GREEN),
            ([2, 3], Rgb565::BLUE),
            ([3, 0], Rgb565::CSS_TEAL),
            ([4, 5], Rgb565::CSS_AQUA),
            ([5, 6], Rgb565::CSS_BISQUE),
            ([6, 7], Rgb565::MAGENTA),
            ([7, 4], Rgb565::CSS_CORNFLOWER_BLUE),
            ([0, 4], Rgb565::CSS_SLATE_GRAY),
            ([1, 5], Rgb565::CSS_INDIAN_RED),
            ([2, 6], Rgb565::CYAN),
            ([3, 7], Rgb565::CSS_SIENNA),
        ]
        .iter()
        {
            Line::new(
                Point::new(points[*idx][0], points[*idx][1]),
                Point::new(points[*idy][0], points[*idy][1]),
            )
            .into_styled(PrimitiveStyle::with_stroke(*color, 1))
            .draw(&mut display)
            .unwrap();
        }

        // Timer::after(Duration::from_millis(10)).await;

        rotate_angle += 0.05;

        // 3.1415 / 2.0
        if rotate_angle > 2.0 * 3.1415 {
            rotate_angle = 0.0;
        } else if rotate_angle < 0.0 {
            rotate_angle = 2.0 * 3.1415;
        }
        Timer::after(Duration::from_millis(20)).await;
        led.toggle();
    }

    loop {
        display.clear(Rgb565::RED).unwrap();
        Timer::after(Duration::from_millis(1000)).await;

        display.clear(Rgb565::GREEN).unwrap();
        Timer::after(Duration::from_millis(1000)).await;

        display.clear(Rgb565::BLUE).unwrap();
        Timer::after(Duration::from_millis(1000)).await;
        led.toggle();
        info!("led toggle");
    }
}
