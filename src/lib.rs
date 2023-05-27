#![no_std]

use core::fmt::Write;
use embassy_rp::gpio::{AnyPin, Flex};
use embedded_drivers::ds1302::InOutPin;
use heapless::String;

pub mod ch463;
pub mod font;
pub mod ssd1327;
pub mod st7049a;
pub mod thermistor;
pub mod ui;

pub fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721 as f32
}
pub fn convert_to_voltage(raw_value: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    (raw_value as f32 * 3.3 / 4096.0) as f32
}

/// seconds to hh:mm:ss
pub fn format_time(secs: i32, buf: &mut String<128>) {
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;

    core::write!(buf, "{:02}:{:02}:{:02}", h, m, s).unwrap();
}

pub struct InOut<'a>(Flex<'a, AnyPin>);

impl<'a> InOut<'a> {
    pub fn new(pin: Flex<'a, AnyPin>) -> Self {
        Self(pin)
    }
}

impl<'a> InOutPin for InOut<'a> {
    type Input = Flex<'a, AnyPin>;
    type Output = Flex<'a, AnyPin>;
    fn set_output(&mut self) {
        self.0.set_as_output();
    }
    fn set_input(&mut self) {
        self.0.set_as_input();
    }
    fn as_input(&self) -> &Self::Input {
        &self.0
    }
    fn as_output(&mut self) -> &mut Self::Output {
        &mut self.0
    }
}
