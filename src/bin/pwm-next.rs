#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    i2c,
    pwm::{self, Pwm},
    spi::{self, Spi},
};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    framebuffer::Framebuffer,
    mono_font::{ascii::FONT_7X13, iso_8859_3::FONT_10X20, MonoTextStyle},
    primitives::{Line, Primitive, PrimitiveStyle},
    text::{Alignment, Text},
};
use embedded_graphics_core::{
    pixelcolor::{
        raw::{BigEndian, LittleEndian},
        Gray4, Gray8, Rgb565,
    },
    prelude::*,
};
use fixed::traits::ToFixed;
use rp::font::FONT_MUZAI_PIXEL;
use {defmt_rtt as _, panic_probe as _};

const WIDTH: usize = 66;
const HEIGHT: usize = 96;

pub struct Framebuffer565 {
    data: [u16; 66 * 96],
}

impl Framebuffer565 {
    pub fn new() -> Self {
        Self { data: [0; 66 * 96] }
    }
}

impl OriginDimensions for Framebuffer565 {
    fn size(&self) -> Size {
        Size::new(66 * 3, 96)
    }
}

impl DrawTarget for Framebuffer565 {
    type Color = Gray4;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            let x = coord.x as usize + 2;
            let y = coord.y as usize;

            if y >= 96 {
                continue;
            }
            if x >= 66 * 3 {
                continue;
            }

            let color = color.luma() as u16;

            let index = y * 66 + x / 3;

            if x % 3 == 0 {
                // write to r channel
                // write to g channel
                // let color = (color << 1) | (color & 0b1);
                self.data[index] = (self.data[index] & 0b11111_111111_00000) | color;
            } else if x % 3 == 1 {
                self.data[index] = (self.data[index] & 0b11111_000000_11111) | (color << 5);
                // write to b channel
                // first
                //let color = (color << 1) | (color & 0b1);
            } else {
                self.data[index] = (self.data[index] & 0b00000_111111_11111) | (color << 11);
                // let color = (color << 2) | (color & 0b1) | ((color & 0b1) << 1);
            }
        }
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut delay = Delay;
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut pwm_conf: pwm::Config = Default::default();
    pwm_conf.compare_a = 1;
    pwm_conf.top = 1;
    //    pwm_conf.divider = 198i32.to_fixed();
    pwm_conf.divider = 240i32.to_fixed();
    let _pwm = Pwm::new_output_a(p.PWM_CH2, p.PIN_20, pwm_conf.clone());

    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let rst = p.PIN_16;
    let dc = p.PIN_21;

    let mut config = spi::Config::default();
    config.frequency = 1_000_000;
    //    config.phase = spi::Phase::CaptureOnFirstTransition;
    //    config.polarity = spi::Polarity::IdleLow;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::High);
    let dc = Output::new(dc, Level::High);
    let mut rst = Output::new(rst, Level::High);

    let mut di = SPIInterface::new(spi, dc, cs);

    rst.set_low();
    Timer::after(Duration::from_millis(5)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(200)).await;

    // Unlock Basic Command
    di.send_commands(DataFormat::U8(&[0xFD]));
    di.send_data(DataFormat::U8(&[0x12]));

    di.send_commands(DataFormat::U8(&[0xAE])); // display off

    di.send_commands(DataFormat::U8(&[0xB3]));
    di.send_data(DataFormat::U8(&[0xB0]));

    di.send_commands(DataFormat::U8(&[0xCA]));
    di.send_data(DataFormat::U8(&[0x3F])); // set multiplex ratio

    //    di.send_commands(DataFormat::U8(&[0xA0]));
    //  di.send_data(DataFormat::U8(&[0x76, 0x00]));

    di.send_commands(DataFormat::U8(&[0xC1])); // must be the same for gray scale
    di.send_data(DataFormat::U8(&[0x7f, 0x7f, 0x7f]));

    di.send_commands(DataFormat::U8(&[0xC7])); // master contrast
    di.send_data(DataFormat::U8(&[0x0F]));

    di.send_commands(DataFormat::U8(&[0xB1])); // Phase 1 and 2 period adjustment
    di.send_data(DataFormat::U8(&[0b0100_0010]));

    di.send_commands(DataFormat::U8(&[0xB6])); // CMD_SetSecondPrechargePeriod
    di.send_data(DataFormat::U8(&[0b1000])); // N DCLKS?

    // # LUT
    // di.send_commands(DataFormat::U8(&[0xB9])); // Reset to default Look Up Table:

    // CMD_MLUTGrayscale
    // special linear LUT, ignore last bit
    di.send_commands(DataFormat::U8(&[0xB8])); // 63 bytes
    di.send_data(DataFormat::U8(&[
        0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x15, 0x17, 0x19, 0x1B,
        0x1D, 0x1F, 0x21, 0x23, 0x25, 0x27, 0x2A, 0x2D, 0x30, 0x33, 0x36, 0x39, 0x3C, 0x3F, 0x42, 0x45, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C,
        0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78, 0x7D, 0x82, 0x87, 0x8C, 0x91, 0x96, 0x9B, 0xA0, 0xA5, 0xAA, 0xAF, 0xB4,
    ]));

    // set_precharge_voltage
    di.send_commands(DataFormat::U8(&[0xBB]));
    di.send_data(DataFormat::U8(&[0x1E]));

    // vcomh
    di.send_commands(DataFormat::U8(&[0xBE]));
    di.send_data(DataFormat::U8(&[0x05]));

    // set_column_address
    di.send_commands(DataFormat::U8(&[0x15]));
    di.send_data(DataFormat::U8(&[0x00, 0x7F]));

    // set_row_address
    di.send_commands(DataFormat::U8(&[0x75]));
    di.send_data(DataFormat::U8(&[0x00, 0x7F]));

    di.send_commands(DataFormat::U8(&[0xA1])); // display start line
    di.send_data(DataFormat::U8(&[]));
    di.send_commands(DataFormat::U8(&[0xA2])); // display offset
    di.send_data(DataFormat::U8(&[100]));

    // 0xA4 => Entire Display Off, All Pixels Turn Off
    // 0xA5 => Entire Display On, All Pixels Turn On at GS Level 15
    // 0xA6 => Normal Display
    // 0xA7 => Inverse Display
    di.send_commands(DataFormat::U8(&[0xA6]));

    di.send_commands(DataFormat::U8(&[0xCA])); // mux ratio
    di.send_data(DataFormat::U8(&[100])); //  4 to 127

    // Set Re-map / Color Depth (Display RAM to Panel)
    // 0bxx_0_xxxxx  Disable COM Split Odd Even
    // 0bxx_x_xx1xx
    // 0b01_1_10110
    di.send_commands(DataFormat::U8(&[0xA0]));
    // 0b01_1_00000
    di.send_data(DataFormat::U8(&[0b01_1_10010, 0x00]));

    di.send_commands(DataFormat::U8(&[0xAF]));

    let mut p = 0;

    let mut fb = Framebuffer565::new();

    loop {
        fb.clear(Gray4::BLACK);

        Line::new(Point::new(0, 0), Point::new(195, 95))
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 1))
            .draw(&mut fb)
            .unwrap();

        let character_style = MonoTextStyle::new(&FONT_10X20, Gray4::WHITE);
        Text::with_alignment("Hello,\nWorld!", Point::new(100, 32), character_style, Alignment::Center).draw(&mut fb);

        let character_style = MonoTextStyle::new(&FONT_10X20, Gray4::new(10));
        Text::with_alignment("\nWorld!", Point::new(100, 32), character_style, Alignment::Center).draw(&mut fb);

        let character_style = MonoTextStyle::new(&FONT_10X20, Gray4::new(1));
        Text::with_alignment("!@#$%^&\n^*=-())", Point::new(150, 32), character_style, Alignment::Center).draw(&mut fb);

        // line start with 32
        // col start with 92
        //
        Line::new(Point::new(0, 1), Point::new(196, 1))
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 1))
            .draw(&mut fb)
            .unwrap();
        Line::new(Point::new(0, 95), Point::new(195, 95))
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 1))
            .draw(&mut fb)
            .unwrap();

        Line::new(Point::new(0, 0), Point::new(0, 127))
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 1))
            .draw(&mut fb)
            .unwrap();

        Line::new(Point::new(195, 0), Point::new(195, 127))
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 1))
            .draw(&mut fb)
            .unwrap();

        // Pixel(Point::new(p / 96, p % 96), Gray8::WHITE).draw(&mut fb);

        // set_column_address
        di.send_commands(DataFormat::U8(&[0x15]));
        di.send_data(DataFormat::U8(&[30, 30 + 65]));
        // set_row_address
        di.send_commands(DataFormat::U8(&[0x75]));
        di.send_data(DataFormat::U8(&[100 - 96, 100])); // same as MUX ratio

        // write_ram
        di.send_commands(DataFormat::U8(&[0x5C]));
        //di.send_data(DataFormat::U8(&[0xff; 200 * 96 - 1]));
        //di.send_data(DataFormat::U16(&fb.data[..]));
        // 1, 3 , 2
        // 0b00010_000000_00000 ??
        //

        for &b in &fb.data {
            di.send_data(DataFormat::U8(&[(b >> 8) as u8, b as u8]));
         }
        p += 1;

        led.toggle();
        info!("toggle, {}, {}", p / 200, p % 96);
        Timer::after(Duration::from_millis(10)).await;
    }
}
