#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
use embassy_rp::gpio::Pull;
use embassy_rp::{bind_interrupts, interrupt};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut adc = Adc::new(p.ADC, Irqs, Config::default());

    //let mut p26 = p.PIN_26; // A0
    //    let mut p27 = p.PIN_27;
    //  let mut p28 = p.PIN_28;

    let mut p26 = Channel::new_pin(p.PIN_28, Pull::None);
    loop {
        let level = adc.read(&mut p26).await.unwrap();
        info!("Pin 26 ADC: {} {}", level, convert_to_voltage(level));

        //        let val = level + 15;
        info!("?? {}", (level+26) >> 7);
        //    let level = adc.read(&mut p27).await;
        //  info!("Pin 27 ADC: {}", level);
        //let level = adc.read(&mut p28).await;
        //info!("Pin 28 ADC: {}", level);
        //  let temp = adc.read_temperature().await;
        //info!("Temp: {} degrees", convert_to_celsius(temp));
        Timer::after(Duration::from_secs(1)).await;
    }
}

fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721 as f32
}

fn convert_to_voltage(raw_value: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    (raw_value as f32 * 3.3 / 4096.0) as f32
}
