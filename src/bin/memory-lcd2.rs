//! Memory LCD - LPM013M126A

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![allow(unused_must_use)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c;
use embassy_rp::pac::common::R;
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_7X14};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Circle;
use embedded_graphics::text::Alignment;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};
use embedded_graphics_core::primitives::Rectangle;
use embedded_hal_1::spi::SpiBusWrite;
use fixed::traits::ToFixed;
use it7259::PointReport;
use memory_lcd_spi::displays::LPM013M126A;
use memory_lcd_spi::pixelcolor::Rgb111;
use memory_lcd_spi::MemoryLCD;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut delay = Delay;

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut config = i2c::Config::default();
    config.frequency = 100_000;
    let i2c = i2c::I2c::new_blocking(p.I2C1, p.PIN_15, p.PIN_14, config);

    let mut tp_rst_n = Output::new(p.PIN_13, Level::High);
    let mut tp_int_n = Input::new(p.PIN_12, Pull::None);

    let mut it7259 = it7259::IT7259::new(i2c, 0x46);
    let _ = it7259.hard_reset(&mut tp_rst_n, &mut delay);

    unwrap!(it7259.set_interrupt(Some(it7259::InterruptTrigger::Low)));
    let (x, y, scale) = it7259.get_resolution().unwrap();
    info!("touch resolution: {}x{} scale: {}", x, y, scale);

    it7259.set_idle_gesture(Some(it7259::IdleGesture::DoubleTap)).unwrap();
    it7259.enter_idle();

    let mosi = p.PIN_19; // SI
    let clk = p.PIN_18; // SCLK
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let mut spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);
    let mut disp = Output::new(p.PIN_20, Level::Low); // DISP
    let mut cs = Output::new(p.PIN_17, Level::Low);

    disp.set_high();

    let mut display: MemoryLCD<LPM013M126A<Rgb111>, _, _> = MemoryLCD::new(spi, cs);
    display.clear(Rgb111::BLACK);
    display.update(&mut delay);

    info!("all initialized");

    // A 3D cube has 8 points, each point has 3 values (x, y, z)
    let orig_points: [[f32; 3]; 8] = [
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
    let mut d = 1.0;

    let z_offset = -3.0; // offset on z axis, leave object

    let cube_size = 80.0;
    let screen_offset_x = 176.0 / 2.0;
    let screen_offset_y = 176.0 / 2.0;
    let mut rotate_angle = 0.0_f32;

    let mut rotated_points = [[0.0; 3]; 8];
    let mut projected_points = [[0.0; 2]; 8];
    let mut points = [[0_i32; 2]; 8];

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X14)
        .text_color(Rgb111::WHITE)
        .background_color(Rgb111::BLACK)
        .build();

    loop {
        for (i, [x, y, z]) in orig_points.into_iter().enumerate() {
            rotated_points[i][0] = x * rotate_angle.cos() - z * rotate_angle.sin();
            rotated_points[i][1] = y;
            rotated_points[i][2] = x * rotate_angle.sin() + z * rotate_angle.cos();

            /*
            rotated_points[i][0] = rotated_points[i][0];
            rotated_points[i][1] =
                rotated_points[i][1] * rotate_angle.cos() - rotated_points[i][2] * rotate_angle.sin();
            rotated_points[i][2] =
                rotated_points[i][1] * rotate_angle.sin() + rotated_points[i][2] * rotate_angle.cos();
                */

            /*
            x' = cos(𝜙) * x + sin(𝜙) * sin(𝜃) * y - sin(𝜙) * cos(𝜃) * z
            y' = cos(𝜃) * y + sin(𝜃) * z
            z' = -sin(𝜙) * x + cos(𝜙) * sin(𝜃) * y + cos(𝜙) * cos(𝜃) * z

            rotated_points[i][0] = rotate_angle.cos() * x + rotate_angle.sin() * rotate_angle2.sin() * y
                - rotate_angle.sin() * rotate_angle2.cos() * z;
            rotated_points[i][1] = rotate_angle2.cos() * y + rotate_angle2.sin() * z;
            rotated_points[i][2] = -rotate_angle.sin() * x
                + rotate_angle.cos() * rotate_angle2.sin() * y
                + rotate_angle.cos() * rotate_angle2.cos() * z;
                            */

            /*
            x' = (d * x) / z
            y' = (d * y) / z
             */

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
        display.clear(Rgb111::BLACK);
        for ([idx, idy], color) in [
            // use different colors for each connected line
            ([0, 1], Rgb111::RED),
            ([1, 2], Rgb111::GREEN),
            ([2, 3], Rgb111::BLUE),
            ([3, 0], Rgb111::MAGENTA),
            ([4, 5], Rgb111::YELLOW),
            ([5, 6], Rgb111::CYAN),
            ([6, 7], Rgb111::MAGENTA),
            ([7, 4], Rgb111::BLUE),
            ([0, 4], Rgb111::GREEN),
            ([1, 5], Rgb111::RED),
            ([2, 6], Rgb111::CYAN),
            ([3, 7], Rgb111::BLUE),
        ]
        .iter()
        {
            Line::new(
                Point::new(points[*idx][0], points[*idx][1]),
                Point::new(points[*idy][0], points[*idy][1]),
            )
            .into_styled(PrimitiveStyle::with_stroke(*color, 1))
            .draw(&mut *display)
            .unwrap();
        }

        Text::new("@andelf", Point::new(2, 10), text_style)
            .draw(&mut *display)
            .unwrap();

        display.update(&mut delay);
        // Timer::after(Duration::from_millis(10)).await;
        rotate_angle += 0.08;

        // 3.1415 / 2.0
        if rotate_angle > 2.0 * 3.1415 {
            rotate_angle = 0.0;
        }
    }
}
