//! ST7049A, 4 x 320 Dot Palette LCD Controller/Driver
//! FSC (Field Sequential Color) LCD Controller/Driver

use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_graphics::{
    pixelcolor::raw::RawU4,
    prelude::{Dimensions, DrawTarget, PixelColor, Point, Size},
    primitives::Rectangle,
    Pixel,
};
use embedded_hal::{blocking::spi, digital::v2::OutputPin};

const WIDTH: u32 = 64;
const HEIGHT: u32 = 16;
const PIXELS: usize = 320 * 4 / 2;
/// Color for FSC LCD
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Color3 {
    White = 0,
    Yellow = 1,
    Pink = 2,
    Red = 3,
    Cyan = 4,
    Green = 5,
    Blue = 6,
    Black = 7,
}

impl PixelColor for Color3 {
    type Raw = RawU4;
}

pub struct Display<DI> {
    di: DI,
    pub buf: [u8; PIXELS],
}

impl<DI: WriteOnlyDataCommand> Display<DI> {
    pub fn new(di: DI) -> Self {
        Self {
            di,
            buf: [0; PIXELS],
        }
    }

    pub fn init(&mut self) -> Result<(), DisplayError> {
        // self.di.send_commands(DataFormat::U8(&[0x01]))?; // SW reset

        self.di.send_commands(DataFormat::U8(&[0x11]))?;
        self.di.send_commands(DataFormat::U8(&[0xD2]))?;
        self.di.send_data(DataFormat::U8(&[0x00]))?;

        self.di.send_commands(DataFormat::U8(&[0xc0]))?; // Vop set
        self.di.send_data(DataFormat::U8(&[140, 0]))?;
        self.di.send_commands(DataFormat::U8(&[0xc3]))?; // bias selection
        self.di.send_data(DataFormat::U8(&[0]))?; //default, or 1

        self.di.send_commands(DataFormat::U8(&[0xB0]))?; // Duty set
        self.di.send_data(DataFormat::U8(&[0b11]))?;

        self.di.send_commands(DataFormat::U8(&[0xB2]))?; // Frame frequency
        self.di.send_data(DataFormat::U8(&[0x1a]))?; // 200Hz, as fast as possible

        self.di.send_commands(DataFormat::U8(&[0xB5]))?;
        self.di.send_data(DataFormat::U8(&[0b0100, 1, 1, 1]))?; // 0b1100, 1, 1, 1

        self.di.send_commands(DataFormat::U8(&[0xB6]))?; // led waveform

        //self.di.send_data(DataFormat::U8(&[20, 20, 20, 100, 100, 100]))?;
        self.di.send_data(DataFormat::U8(&[20, 20, 20, 200, 200, 200]))?;
        //self.di.send_data(DataFormat::U8(&[20, 20, 20, 50, 50, 50]))?;
        // 1:0.75:0.35
        //self.di
        //    .send_data(DataFormat::U8(&[0x50, 0x50, 0x50, 150, 150, 150]))?;

        self.di.send_commands(DataFormat::U8(&[0xB7]))?;
        self.di.send_data(DataFormat::U8(&[0x40]))?;

        self.di.send_commands(DataFormat::U8(&[0x29]))?; // display on

        self.di.send_commands(DataFormat::U8(&[0xD4]))?; // RGB LED control
        self.di.send_data(DataFormat::U8(&[0b0_111]))?;

        Ok(())
    }

    pub fn set_bg_light(&mut self, color: Option<Color3>) -> Result<(), DisplayError> {
        match color {
            Some(color) => {
                self.di.send_commands(DataFormat::U8(&[0xD4]))?; // RGB LED control
                self.di
                    .send_data(DataFormat::U8(&[0b1_000 | (color as u8)]))?;
            }
            None => {
                self.di.send_commands(DataFormat::U8(&[0xD4]))?; // RGB LED control
                self.di.send_data(DataFormat::U8(&[0b1_000]))?;
            }
        }
        Ok(())
    }

    pub fn clear(&mut self) {
        self.buf.fill(0);
    }

    pub fn flush(&mut self) -> Result<(), DisplayError> {
        self.di.send_commands(DataFormat::U8(&[0x2A]))?; // column addr
        self.di.send_data(DataFormat::U8(&[0, 159]))?;

        self.di.send_commands(DataFormat::U8(&[0x2B]))?; // raw addr
        self.di.send_data(DataFormat::U8(&[0, 3]))?;

        self.di.send_commands(DataFormat::U8(&[0x2C]))?; // write data
        self.di.send_data(DataFormat::U8(&self.buf))?;
        Ok(())
    }
}
impl<DI> Display<DI> {
    pub fn draw_pixel(&mut self, x: u32, mut y: u32, color: Color3) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }

        let idx = if y < HEIGHT / 2 {
            if x < WIDTH / 2 {
                match y {
                    3 | 4 => 112 + x,
                    2 | 5 => 272 + x,
                    1 | 6 => 432 + x,
                    0 | 7 => 592 + x,
                    _ => unreachable!(),
                }
            } else if x >= WIDTH / 2 {
                match y {
                    3 | 4 => 16 + x - WIDTH / 2,
                    2 | 5 => 176 + x - WIDTH / 2,
                    1 | 6 => 336 + x - WIDTH / 2,
                    0 | 7 => 496 + x - WIDTH / 2,
                    _ => unreachable!(),
                }
            } else {
                unreachable!()
            }
        } else {
            match y {
                11 | 12 => 48 + WIDTH - x - 1,
                10 | 13 => 208 + WIDTH - x - 1,
                9 | 14 => 368 + WIDTH - x - 1,
                8 | 15 => 528 + WIDTH - x - 1,
                _ => unreachable!(),
            }
        };
        if y >= HEIGHT / 2 {
            y -= HEIGHT / 2;
        }

        if x < WIDTH / 2 {
            if y < 4 {
                self.buf[idx as usize] = (color as u8) | (self.buf[idx as usize] & 0xf0);
            } else {
                self.buf[idx as usize] = ((color as u8) << 4) | (self.buf[idx as usize] & 0x0f);
            }
        } else {
            if y < 4 {
                self.buf[idx as usize] = ((color as u8) << 4) | (self.buf[idx as usize] & 0x0f);
            } else {
                self.buf[idx as usize] = (color as u8) | (self.buf[idx as usize] & 0xf0);
            }
        }
    }
}

impl<DI> Dimensions for Display<DI> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(0, 0), Size::new(64, 16))
    }
}

impl<DI> DrawTarget for Display<DI> {
    type Color = Color3;

    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let Pixel(point, color) = pixel;
            self.draw_pixel(point.x as u32, point.y as u32, color);
        }
        Ok(())
    }
}
