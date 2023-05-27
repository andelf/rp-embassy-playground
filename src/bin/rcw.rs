//! RCW sensor

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]
#![feature(generic_const_exprs)]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::interrupt;
use embassy_time::{Delay, Duration, Instant, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut delay = Delay;

    let mut trig = Output::new(p.PIN_16, Level::Low);
    let mut echo = Input::new(p.PIN_19, Pull::None);

    //let miso = p.PIN_16;

    loop {
        // refresh
        trig.set_high();
        delay.delay_us(15_u32);
        trig.set_low();

        echo.wait_for_rising_edge().await;
        let start = Instant::now();
        echo.wait_for_falling_edge().await;
        let elapsed = start.elapsed().as_micros();

        // time * 340m/s / 2
        let distance = elapsed as f32 * 340.0 / 10_000.0 / 2.0;
        info!("distance: {} cm", distance);

        Timer::after(Duration::from_millis(500)).await;
        led.toggle();
        info!("led toggle");
    }
}
