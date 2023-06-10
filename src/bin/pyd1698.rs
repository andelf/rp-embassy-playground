#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Flex, Level, Output, Pin};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_1::delay::DelayUs;
use {defmt_rtt as _, panic_probe as _};

pub struct PYD1698<'d> {
    dl: Flex<'d, AnyPin>,
    serin: Output<'d, AnyPin>,
}

impl<'a> PYD1698<'a> {
    pub fn read_adc_voltage<D: DelayUs>(&mut self, delay: &mut D) -> u16 {
        let raw = self.read_all(delay);

        u16::try_from(raw >> 25).unwrap() & 0x3fff
    }

    pub fn read_adc_voltage_bpf<D: DelayUs>(&mut self, delay: &mut D) -> i16 {
        let raw = self.read_adc_voltage(delay);

        // convert from 14 bit 2's complement to i16
        let mut val = raw as i16;
        if val & 0x2000 != 0 {
            val -= 0x4000;
        }
        val
    }

    // 40
    pub fn read_all<D: DelayUs>(&mut self, delay: &mut D) -> u64 {
        self.dl.set_high();
        self.dl.set_as_output();

        delay.delay_us(110);

        let mut out = 0u64;

        for i in 0..40 {
            self.dl.set_low();
            self.dl.set_as_output();
            delay.delay_us(1);

            self.dl.set_high();
            delay.delay_us(1);

            self.dl.set_as_input();
            delay.delay_us(7);

            if self.dl.is_high() {
                out = (out << 1) | 1;
            } else {
                out = out << 1;
            }
        }

        self.dl.set_low();
        self.dl.set_as_output();

        delay.delay_us(200);
        self.dl.set_as_input();

        //  let adc_voltage = ((out >> 25) & 0x3fff) as u16;
        //adc_voltage
        out
    }

    // write 25 bits of data, using serin
    pub fn write_reg<D: DelayUs>(&mut self, regval: u32, delay: &mut D) {
        self.serin.set_low();

        for i in 0..25 {
            let next_bit = (regval >> (24 - i)) & 1;

            self.serin.set_low();
            self.serin.set_high();

            if next_bit == 1 {
                self.serin.set_high();
            } else {
                self.serin.set_low();
            }

            delay.delay_us(100)
        }

        self.serin.set_low();
        delay.delay_us(600);
    }

    pub async fn wait_for_motion(&mut self) {
        self.dl.set_low();
        self.dl.set_as_input();
        self.dl.wait_for_high().await
    }
}

pub struct Config {
    // [24:17]
    pub sensitivity: u8,
    // [16:13]
    pub blind_time: u8,
    // [12:11]
    pub pulse_counter: u8,
    // [10:9]
    pub window_time: u8,
    // [8:7]
    /// 0 = Forced Read Out
    /// 1 = Interrupt Read Out Mode
    /// 2 = Wake Up Operation Mode
    pub operation_mode: u8,
    // [6:5]
    /// 0 = PIR(BPF)
    /// 1 = PIR(LPF)
    /// 3 =  Temperature Sensor
    pub filter_source: u8,
    // [4:0], fix to 16
    // _reserved: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sensitivity: 50,
            blind_time: 0,
            pulse_counter: 0,
            window_time: 0,
            operation_mode: 0,
            filter_source: 1,
        }
    }
}

impl Config {
    pub fn to_u25(&self) -> u32 {
        let mut out = 0u32;

        out |= (self.sensitivity as u32) << 17;
        out |= (self.blind_time as u32) << 13;
        out |= (self.pulse_counter as u32) << 11;
        out |= (self.window_time as u32) << 9;
        out |= (self.operation_mode as u32) << 7;
        out |= (self.filter_source as u32) << 5;
        out |= 16;

        out
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut delay = Delay;
    let mut led = Output::new(p.PIN_25, Level::Low);

    let serial_in = p.PIN_14;
    let direct_link = p.PIN_15;

    let mut serin = Output::new(serial_in.degrade(), Level::Low);
    let mut dl = Flex::new(direct_link.degrade());

    let mut sensor = PYD1698 { dl, serin };

    let mut conf = Config::default();
    conf.sensitivity = 80;
    conf.filter_source = 1;
    conf.operation_mode = 2; // wake up when motion.

    sensor.write_reg(conf.to_u25(), &mut delay);

    loop {
        sensor.wait_for_motion().await;

        let adc_voltage = sensor.read_adc_voltage(&mut delay);

        info!("motion detected: voltage: {}", adc_voltage);

        led.toggle();
    }
}
