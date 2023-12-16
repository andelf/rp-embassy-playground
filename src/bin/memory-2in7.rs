#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![allow(unused_must_use)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::{self, Spi};
use embassy_rp::{i2c, pwm};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_7X14;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::text::Text;
use fixed::traits::ToFixed;
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::prelude::*;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    Drawable,
};
use memory_lcd_spi::displays::{LS013B7DH03, LS027B7DH01};
use memory_lcd_spi::pixelcolor::Rgb111;
use memory_lcd_spi::MemoryLCD;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut delay = Delay;

    let mut led = Output::new(p.PIN_25.degrade(), Level::Low);

    let cs = Output::new(p.PIN_17, Level::Low);
    let clk = p.PIN_18; // SCLK
    let mosi = p.PIN_19; // SI
    let mut config = spi::Config::default();
    config.frequency = 2_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    let mut disp = Output::new(p.PIN_20, Level::Low); // DISP
    disp.set_high();

    let mut pwm_conf: pwm::Config = Default::default();
    pwm_conf.compare_b = 32768;
    pwm_conf.divider = 2i32.to_fixed();
    let _pwm = Pwm::new_output_b(p.PWM_CH2, p.PIN_21, pwm_conf.clone());

    // let mut display: MemoryLCD<LS013B7DH03, _, _> = MemoryLCD::new(spi, cs);

    let mut display: MemoryLCD<LS027B7DH01, _, _> = MemoryLCD::new(spi, cs);

    //display.set_rotation(memory_lcd_spi::framebuffer::Rotation::Deg90);
    display.clear(BinaryColor::On);
    display.update(&mut delay);

    if false {
        display.clear(BinaryColor::On);
        display.update(&mut delay);
        Timer::after(Duration::from_millis(1000)).await;
        display.clear(BinaryColor::Off);
        display.update(&mut delay);
        Timer::after(Duration::from_millis(1000)).await;
    }

    info!("all initialized");

    const POINTS: usize = 8;
    // A 3D cube has 8 points, each point has 3 values (x, y, z)
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

    // Perspective Projection:
    // distance to project plane
    let mut d = 1.4;

    let z_offset = -3.0; // offset on z axis, leave object

    let cube_size = 100.0;
    let screen_offset_x = 400.0 / 2.0;
    let screen_offset_y = 240.0 / 2.0;
    let mut rotate_angle = 0.0_f32;

    let mut rotated_points = [[0.0; 3]; POINTS];
    let mut projected_points = [[0.0; 2]; POINTS];
    let mut points = [[0_i32; 2]; POINTS];

    let mut inc = true;

    let text_style = MonoTextStyleBuilder::new().font(&FONT_7X14).text_color(BinaryColor::Off).build();

    loop {
        for (i, [x, y, z]) in orig_points.iter().copied().enumerate() {
            // rotate around y axis
            rotated_points[i][0] = x * rotate_angle.cos() + z * rotate_angle.sin();
            rotated_points[i][1] = y;
            rotated_points[i][2] = -x * rotate_angle.sin() + z * rotate_angle.cos();

            let rx = rotated_points[i][0];
            let ry = rotated_points[i][1];
            let rz = rotated_points[i][2] + z_offset; // offset on z axis, leave object

            /*
            x' = (d * x) / z
            y' = (d * y) / z
             */
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

        display.clear(BinaryColor::On);
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
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 1))
            .draw(&mut *display)
            .unwrap();
        }

        Text::new("@andelf", Point::new(2, 10), text_style).draw(&mut *display).unwrap();

        display.update(&mut delay);
        // Timer::after(Duration::from_micros(1)).await;
        rotate_angle += 0.02;

        if rotate_angle > 2.0 * 3.1415 {
            rotate_angle = 0.0;
        }
        if inc {
            d += 0.01;
            if d > 2.0 {
                inc = false;
            }
        } else {
            d -= 0.01;
            if d < 0.5 {
                inc = true;
            }
        }
        led.toggle();
    }
}
