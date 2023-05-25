//! Memory LCD - LPM013M126A

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::prelude::*;
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
pin22 VST
21 VCK
20 ENB
19 XRST
18 FRP
17 XFRP

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

    // update mode
    info!("tick?");
    Timer::after(Duration::from_micros(10)).await; // us
    info!("tick!");

    // reset
    xrst.set_low();
    Timer::after(Duration::from_millis(1000)).await;
    xrst.set_high(); //active

    info!("reset ok");

    spawner
        .spawn(vcom_drive(p.PIN_14.degrade(), p.PIN_18.degrade(), p.PIN_17.degrade()))
        .unwrap();
    info!("vcom drive started");

    let mut cnt = 0;

    vck.set_low();
    vst.set_high();

    hst.set_low();

    Timer::after(Duration::from_micros(1)).await; // us\

    let mut inv = false;
    // cycle 488
    loop {
        inv = !inv;

        xrst.set_high();

        vst.set_high();
        Timer::after(Duration::from_micros(1)).await; // us\

        for i in 1..=488 {
            vck.toggle(); // rising edge or falling edge

            if i == 1 {
                vst.set_low();
            }

            /*  if i == 485 {
                xrst.set_low();
            }
            if i == 488 {
                xrst.set_high();
            }*/

            if inv {
                if i < 120 {
                    g1.set_high();
                    g2.set_high();
                    b1.set_high();
                    b2.set_high();
                } else if i < 240 {
                    g1.set_low();
                    g2.set_low();
                    b1.set_low();
                    b2.set_low();
                } else if i < 320 {
                    g1.set_low();
                    g2.set_low();
                    b1.set_high();
                    b2.set_high();
                } else {
                    g1.set_high();
                    g2.set_high();
                    b1.set_low();
                    b2.set_low();
                }
            } else {
                if i < 120 {
                    g1.set_high();
                    g2.set_high();
                    b1.set_low();
                    b2.set_low();
                } else if i < 240 {
                    g1.set_low();
                    g2.set_low();
                    b1.set_high();
                    b2.set_high();
                } else if i < 320 {
                    g1.set_low();
                    g2.set_low();
                    b1.set_low();
                    b2.set_low();
                } else {
                    g1.set_high();
                    g2.set_high();
                    b1.set_high();
                    b2.set_high();
                }
            }

            if i >= 3 && i <= 482 {
                // 240 lines
                hst.set_high();
                Timer::after(Duration::from_micros(1)).await; // us

                for j in 1..=122 {
                    hck.toggle();

                    if j == 1 {
                        hst.set_low();
                    }

                    if j < 30 {
                        r1.set_high();
                        r2.set_high();
                    } else if j < 60 {
                        r1.set_low();
                        r2.set_low();
                    } else if j < 120 {
                        r1.set_low();
                        r2.set_high();
                    }

                    // 125Mhz for 10us
                    //Timer::after(Duration::from_hz(1_000_000)).await; // us
                    Timer::after(Duration::from_micros(1)).await; // us
                }

                enb.set_high();
                Timer::after(Duration::from_micros(41)).await; // us
                enb.set_low();
            } else {
                Timer::after(Duration::from_micros(10)).await;
            }
        }
        //      xrst.set_low(); // active display no update

        Timer::after(Duration::from_millis(500)).await;
        info!("toggle frame");
    }

    /*
        cnt += 1;
        vck.set_low();
        vst.set_high();
        Timer::after(Duration::from_micros(40)).await; // us
        vst.set_low();

        for i in 1..=488 {
            vck.toggle();

            Timer::after(Duration::from_micros(10)).await; // us

            // vck.set_low();

            hst.set_high();
            Timer::after(Duration::from_micros(1)).await; // us

            for j in 1..=122 {
                if j < 30 {
                    r2.set_high();
                    g2.set_low();
                    b2.set_low();
                } else if j < 60 {
                    r2.set_high();
                    g2.set_high();
                    b2.set_high();
                } else if j < 90 {
                    r2.set_low();
                    g2.set_low();
                    b2.set_high();
                } else {
                    r2.set_low();
                    g2.set_high();
                    b2.set_low();
                }

                if j == 1 {
                    enb.set_low();
                    hst.set_low();
                } else if j >= 121 {
                    enb.set_high();
                }

                Timer::after(Duration::from_micros(1)).await;
                hck.toggle();
                // us
                //  if j == 1 {
                //     hst.set_low();
                // }
                // hck.set_low();
                //Timer::after(Duration::from_micros(1)).await; // us
            }
        }
        info!("toggle");
        Timer::after(Duration::from_millis(100)).await;
    }
    */

    let mut rev = false;
    loop {
        info!("toggle");
        led.toggle();
        rev = !rev;
        Timer::after(Duration::from_millis(1000)).await;
    }
}

/*
    // hard reset
    reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    reset.set_high();
    Timer::after(Duration::from_millis(100)).await;

    // on to MM
    dcx.set_low(); // command
    spi.blocking_write(&[0x01]); // soft reset

    Timer::after(Duration::from_millis(20)).await; // > 10ms

    dcx.set_low(); // command
    spi.blocking_write(&[0xff]);
    dcx.set_high();
    spi.blocking_write(&[0x20]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xfb]);
    dcx.set_high();
    spi.blocking_write(&[0x01]);

    dcx.set_low(); // command
    spi.blocking_write(&[0x6d]);
    dcx.set_high();
    spi.blocking_write(&[0x74]);

    // select CMD1
    dcx.set_low(); // command
    spi.blocking_write(&[0xff]);
    dcx.set_high();
    spi.blocking_write(&[0x10]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xfb]);
    dcx.set_high();
    spi.blocking_write(&[0x01]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xb3]);
    dcx.set_high();
    spi.blocking_write(&[0x15]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xbb]);
    dcx.set_high();
    spi.blocking_write(&[0x10]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xf3]);
    dcx.set_high();
    spi.blocking_write(&[0x02]);

    dcx.set_low(); // command
    spi.blocking_write(&[0x3a]);
    dcx.set_high();
    spi.blocking_write(&[0x05]); // color format, 16bit 0x05

    dcx.set_low(); // command
    spi.blocking_write(&[0x11]); // sleep out
    dcx.set_high();

    Timer::after(Duration::from_millis(20)).await; // > 10ms

    dcx.set_low(); // command
    spi.blocking_write(&[0x2c]);
    spi.blocking_write(&[0xaa; 300 * 2]);
    dcx.set_high();

    for i in 0..300 {
        dcx.set_low(); // command
        spi.blocking_write(&[0x3c]);
        spi.blocking_write(&[0xaa; 300 * 2]);
        dcx.set_high();
    }

    Timer::after(Duration::from_millis(140)).await; // > 120ms

    cs.set_low();
    dcx.set_low(); // command
    spi.blocking_write(&[0x29]); // display on
    dcx.set_high();

    dcx.set_low(); // command
    spi.blocking_write(&[0xbb]);
    dcx.set_high();
    spi.blocking_write(&[0x00]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xb3]);
    dcx.set_high();
    spi.blocking_write(&[0x35]);

    dcx.set_low(); // command
    spi.blocking_write(&[0xff]);
    dcx.set_high();
    spi.blocking_write(&[0x20]);

    dcx.set_low(); // command
    spi.blocking_write(&[0x0b]);
    dcx.set_high();
    spi.blocking_write(&[0x00]);

    // in memory mode

    let mut b = 1;
    let mut rev = false;
    loop {
        info!("toggle");
        led.toggle();
        rev = !rev;
        Timer::after(Duration::from_millis(1000)).await;
    }
}

*/
