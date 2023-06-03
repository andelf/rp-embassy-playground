//! Memory LCD - LPM013M126A

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![allow(unused_must_use)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c;
use embassy_rp::spi::{self, Spi};
use embassy_time::Delay;
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::prelude::*;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    Drawable,
};
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
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);
    let cs = Output::new(p.PIN_17, Level::Low);

    let mut disp = Output::new(p.PIN_20, Level::Low); // DISP

    disp.set_high();

    let mut display: MemoryLCD<LPM013M126A<BinaryColor>, _, _> = MemoryLCD::new(spi, cs);
    display.clear(BinaryColor::Off);
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

    // Perspective Projection:
    // distance to project plane
    let mut d = 1.4;

    let z_offset = -3.0; // offset on z axis, leave object

    let cube_size = 80.0;
    let screen_offset_x = 176.0 / 2.0;
    let screen_offset_y = 176.0 / 2.0;
    let mut rotate_angle = 0.0_f32;

    let mut rotated_points = [[0.0; 3]; 8];
    let mut projected_points = [[0.0; 2]; 8];
    let mut points = [[0_i32; 2]; 8];

    let mut inc = true;

    loop {
        for (i, [x, y, z]) in orig_points.iter().copied().enumerate() {
            /*
            x' = (d * x) / z
            y' = (d * y) / z
             */
            rotated_points[i][0] = x * rotate_angle.cos() - z * rotate_angle.sin();
            rotated_points[i][1] = y;
            rotated_points[i][2] = x * rotate_angle.sin() + z * rotate_angle.cos();

            let rx = rotated_points[i][0];
            let ry = rotated_points[i][1];
            let rz = rotated_points[i][2] + z_offset; // offset on z axis, leave object

            projected_points[i][0] = (d * rx) / rz;
            projected_points[i][1] = (d * ry) / rz;
        }

        display.clear(BinaryColor::On);

        for (i, [x, y]) in projected_points.iter().enumerate() {
            let x: f32 = (x * cube_size) + screen_offset_x;
            let y: f32 = (y * cube_size) + screen_offset_y;

            // manually round
            points[i][0] = (x + 0.5) as i32;
            points[i][1] = (y + 0.5) as i32;
        }

        for [idx, idy] in [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
        ]
        .iter()
        {
            Line::new(
                Point::new(points[*idx][0], points[*idx][1]),
                Point::new(points[*idy][0], points[*idy][1]),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 2))
            .draw(&mut *display)
            .unwrap();
        }

        display.update(&mut delay);
        // Timer::after(Duration::from_millis(10)).await;
        rotate_angle += 0.02;

        if rotate_angle > 3.1415 / 2.0 {
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
