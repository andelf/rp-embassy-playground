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
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayUs;

/* eight color
#define COLOR_BLACK             0x00
#define COLOR_BLUE              0x02
#define COLOR_GREEN             0x04
#define COLOR_CYAN              0x06
#define COLOR_RED               0x08
#define COLOR_MAGENTA           0x0a
#define COLOR_YELLOW            0x0c
#define COLOR_WHITE             0x0e


 */

fn send_command(scl: &mut Output<AnyPin>, sdi: &mut Output<AnyPin>, csx: &mut Output<AnyPin>, cmd: u8) {
    csx.set_low();

    sdi.set_low();
    scl.set_low();
    Delay.delay_us(1u32);
    scl.set_high();
    Delay.delay_us(1u32);
    soft_spi_write(scl, sdi, csx, cmd);

    csx.set_high();
}

fn send_command_data(scl: &mut Output<AnyPin>, sdi: &mut Output<AnyPin>, csx: &mut Output<AnyPin>, cmd: u8, data: u8) {
    csx.set_low();

    sdi.set_low();
    scl.set_low();
    Delay.delay_us(1u32);
    scl.set_high();
    Delay.delay_us(1u32);
    soft_spi_write(scl, sdi, csx, cmd);
    sdi.set_high();
    scl.set_low();
    Delay.delay_us(1u32);
    scl.set_high();
    Delay.delay_us(1u32);
    soft_spi_write(scl, sdi, csx, data);

    csx.set_high();
}

fn send_command_with_mem(scl: &mut Output<AnyPin>, sdi: &mut Output<AnyPin>, csx: &mut Output<AnyPin>, cmd: u8, data: &[u8]) {
    csx.set_low();

    sdi.set_low();
    scl.set_low();
    Delay.delay_us(1u32);
    scl.set_high();
    Delay.delay_us(1u32);
    soft_spi_write(scl, sdi, csx, cmd);
    sdi.set_high();
    scl.set_low();
    Delay.delay_us(1u32);
    scl.set_high();
    Delay.delay_us(1u32);
    for d in data {
        soft_spi_write(scl, sdi, csx, *d);
    }

    csx.set_high();
}

fn soft_spi_write(scl: &mut Output<AnyPin>, sdi: &mut Output<AnyPin>, csx: &mut Output<AnyPin>, data: u8) {
    let mut data = data;
    for _ in 0..8 {
        if data & 0x80 != 0 {
            sdi.set_high();
        } else {
            sdi.set_low();
        }
        data <<= 1;
        scl.set_low();
        Delay.delay_us(1u32);
        scl.set_high();
        Delay.delay_us(1u32);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mosi = p.PIN_19; // SDI
    let clk = p.PIN_18;
    let cs = p.PIN_17;
    // let disp = p.PIN_20;

    //let dcx = p.PIN_20;
    let resx = p.PIN_16;

    // use 9bit spi

    // create SPI
    //let mut spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // let mut di = display_interface_spi::SPIInterface::new(spi, dcx, cs);

    // Configure CS
    let mut csx = Output::new(cs.degrade(), Level::High);
    // let mut dcx = Output::new(dcx, Level::Low);
    let mut resx = Output::new(resx.degrade(), Level::Low);
    let mut scl = Output::new(clk.degrade(), Level::High);
    let mut sdi = Output::new(mosi.degrade(), Level::Low);

    Timer::after(Duration::from_millis(20)).await;

    // power on
    csx.set_high();
    scl.set_high();
    sdi.set_low();

    // hard reset
    resx.set_low();
    Timer::after(Duration::from_millis(10)).await;
    resx.set_high();
    Timer::after(Duration::from_millis(100)).await;

    // N-2: ON Sequence to MM by SPI
    send_command(&mut scl, &mut sdi, &mut csx, 0x01); // soft reset
    Timer::after(Duration::from_millis(20)).await; // > 10ms

    send_command_data(&mut scl, &mut sdi, &mut csx, 0xff, 0x20);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xfb, 0x01);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0x6d, 0x74);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xff, 0x10);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xfb, 0x01);

    send_command_data(&mut scl, &mut sdi, &mut csx, 0xb3, 0x15);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xbb, 0x10);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xf3, 0x02);
    // 18bit:06h 16bit:05h 3bit_Type1:01h 3bit_Type2:09h
    send_command_data(&mut scl, &mut sdi, &mut csx, 0x3a, 0x01);
    send_command(&mut scl, &mut sdi, &mut csx, 0x11); // sleep out

    Timer::after(Duration::from_millis(20)).await; // > 10ms
    send_command_with_mem(&mut scl, &mut sdi, &mut csx, 0x2c, &[0b00_111_111; 320 * 300 / 2]);

    Timer::after(Duration::from_millis(140)).await; // > 10ms

    send_command(&mut scl, &mut sdi, &mut csx, 0x29); // display on
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xbb, 0x00);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xb3, 0x35);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0xff, 0x20);
    send_command_data(&mut scl, &mut sdi, &mut csx, 0x0b, 0x00);

    // in Memory mode

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
