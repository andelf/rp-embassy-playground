#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{gpio, uart};
use embassy_time::{Duration, Timer};
use embedded_drivers::pmsx003::Measurements;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut config = uart::Config::default();
    config.baudrate = 9600;
    let mut uart = uart::Uart::new_blocking(p.UART0, p.PIN_16, p.PIN_17, config);
    //uart.blocking_write("Hello World!\r\n".as_bytes()).unwrap();

    let mut buf = [0u8; 32];

    loop {

        uart.blocking_read(&mut buf);

        if let Some(measurements) = Measurements::from_bytes(&buf) {
            println!("{:#?}", measurements);
        }

        info!("led on!");
        led.set_high();
        Timer::after(Duration::from_millis(400)).await;

        info!("led off!");
        led.set_low();
        Timer::after(Duration::from_millis(400)).await;
    }
}
