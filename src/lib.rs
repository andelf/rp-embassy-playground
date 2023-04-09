#![no_std]

use core::fmt::Write;
use heapless::String;

pub mod font;
pub mod st7049a;

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

    buf.clear();
    core::write!(buf, "{:02}:{:02}:{:02}", h, m, s).unwrap();
}
