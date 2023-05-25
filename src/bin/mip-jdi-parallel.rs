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
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
    Drawable,
};
use embedded_hal::prelude::{_embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_delay_DelayUs};

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

/// pixel format: u8
/// 0b00_rrggbb
pub struct LPM012M134B {
    pub fb: [u8; 240 * 240],
}

impl LPM012M134B {
    pub fn new() -> Self {
        Self { fb: [0u8; 240 * 240] }
    }
}

impl Dimensions for LPM012M134B {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(1, 1), Size::new(240, 240))
    }
}

impl DrawTarget for LPM012M134B {
    type Color = Rgb565;

    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if let Ok((x @ 1..=240, y @ 1..=240)) = coord.try_into() {
                let pos = (y * 240 + x) as usize;
                let raw_color = ((color.r() >> 3) << 4) | ((color.g() >> 4) << 2) | (color.b() >> 3);
                self.fb[pos] = raw_color;
            }
        }
        Ok(())
    }
}

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
    //let raw2 = include_bytes!("../../240x240.2.raw");
    //let raw2 = include_bytes!("../../color.raw");
    //let raw2 = include_bytes!("../../test.raw");
    let raw2 = include_bytes!("../../hot.raw");

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

    // initial state
    vck.set_low();
    vst.set_low();
    hst.set_low();
    hck.set_low();

    spawner
        .spawn(vcom_drive(p.PIN_14.degrade(), p.PIN_18.degrade(), p.PIN_17.degrade()))
        .unwrap();

    Timer::after(Duration::from_micros(1)).await; // us\
    let mut x = 0;
    let mut y = 0;

    let mut inv = true;

    xrst.set_high(); // deassert reset
    Timer::after(Duration::from_micros(22)).await;

    loop {
        inv = !inv;

        // xrst.set_high(); // deassert reset
        // Timer::after(Duration::from_micros(22)).await; // XRST set-up time

        vck.set_low();
        vst.set_high();
        Timer::after(Duration::from_micros(20)).await; // tsVST, VST setup time, 41us, must

        for i in 1..=488 {
            vck.toggle(); // rising edge or falling edge
            if i == 1 {
                vst.set_low();
                Timer::after(Duration::from_micros(41)).await; // thVST, VST hold time, 41us, must
            }

            if i >= 2 && i < 482 {
                // 240 lines
                // Timer::after(Duration::from_micros(1)).await; // tdHST, delay before HST

                hck.toggle();
                hst.set_high();
                //Timer::after(Duration::from_micros(1)).await; // tsHST, HST setup time
                Timer::after(Duration::from_micros(1)).await; // thHST, must
                hck.toggle(); // trigger hst

                for j in 1..=124 {
                    if j == 1 {
                        hst.set_low();
                        Timer::after(Duration::from_micros(1)).await; // thHST, must
                    }

                    if j == 1 {
                        enb.set_high();
                    } else if j == 10 {
                        enb.set_low();
                    }

                    if j >= 1 && j <= 120 {
                        y = (i - 2) / 2;
                        x = j - 1;

                        let pos = (x * 2) + 240 * y;
                        let raw = if inv { raw1 } else { raw2 };

                        if i % 2 == 0 {
                            // LPB
                            let pixel = raw[pos];
                            if pixel & 0b10_00_00 != 0 {
                                r1.set_high();
                            } else {
                                r1.set_low();
                            }

                            if pixel & 0b00_10_00 != 0 {
                                g1.set_high();
                            } else {
                                g1.set_low();
                            }

                            if pixel & 0b00_00_10 != 0 {
                                b1.set_high();
                            } else {
                                b1.set_low();
                            }
                            let pixel = raw[pos + 1];
                            if pixel & 0b10_00_00 != 0 {
                                r2.set_high();
                            } else {
                                r2.set_low();
                            }

                            if pixel & 0b00_10_00 != 0 {
                                g2.set_high();
                            } else {
                                g2.set_low();
                            }

                            if pixel & 0b00_00_10 != 0 {
                                b2.set_high();
                            } else {
                                b2.set_low();
                            }
                        } else {
                            // SPB
                            let pixel = raw[pos];
                            if pixel & 0b01_00_00 != 0 {
                                r1.set_high();
                            } else {
                                r1.set_low();
                            }

                            if pixel & 0b00_01_00 != 0 {
                                g1.set_high();
                            } else {
                                g1.set_low();
                            }

                            if pixel & 0b00_00_01 != 0 {
                                b1.set_high();
                            } else {
                                b1.set_low();
                            }
                            let pixel = raw[pos + 1];
                            if pixel & 0b01_00_00 != 0 {
                                r2.set_high();
                            } else {
                                r2.set_low();
                            }

                            if pixel & 0b00_01_00 != 0 {
                                g2.set_high();
                            } else {
                                g2.set_low();
                            }

                            if pixel & 0b00_00_01 != 0 {
                                b2.set_high();
                            } else {
                                b2.set_low();
                            }
                        }
                    }

                    // cortex_m::asm::delay(150);
                    hck.toggle();
                    //Timer::after(Duration::from_hz(1_000_000)).await; // us
                    // Timer::after(Duration::from_micros(5)).await; // us
                    //Timer::after(Duration::from_ticks(1)).await;
                    //Delay.delay_us(1_u32);
                    future::ready(()).await;
                }
            } else {
                // 82
                Timer::after(Duration::from_micros(1)).await; // 1us non-update
            }
        }
        hck.set_low();
        hst.set_low();
        vck.set_low();

        //xrst.set_low(); // active display no update
        Timer::after(Duration::from_millis(500)).await;
        info!("toggle frame");
        led.toggle();
    }
}
