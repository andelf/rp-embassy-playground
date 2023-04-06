//! 4 x 320 Dot Palette LCD Controller/Driver
//! FSC (Field Sequential Color) LCD Controller/Driver

use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use display_interface_spi::SPIInterface;
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
        self.di.send_commands(DataFormat::U8(&[0x11]))?;
        self.di.send_commands(DataFormat::U8(&[0xD2]))?;
        self.di.send_data(DataFormat::U8(&[0x00]))?;

        self.di.send_commands(DataFormat::U8(&[0xc0]))?;
        self.di.send_data(DataFormat::U8(&[140, 0]))?;

        self.di.send_commands(DataFormat::U8(&[0xB0]))?;
        self.di.send_data(DataFormat::U8(&[0x03]))?;

        self.di.send_commands(DataFormat::U8(&[0xB2]))?;
        self.di.send_data(DataFormat::U8(&[0x1a]))?;

        self.di.send_commands(DataFormat::U8(&[0xB5]))?;
        self.di.send_data(DataFormat::U8(&[0x4, 1, 1, 1]))?;

        self.di.send_commands(DataFormat::U8(&[0xB6]))?; // led waveform

        //self.di.send_data(DataFormat::U8(&[20, 20, 20, 200, 200, 200]))?;
        self.di
            .send_data(DataFormat::U8(&[20, 20, 20, 255, 255, 255]))?;

        self.di.send_commands(DataFormat::U8(&[0xB7]))?;
        self.di.send_data(DataFormat::U8(&[0x40]))?;

        self.di.send_commands(DataFormat::U8(&[0x29]))?;

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
                if y == 3 || y == 4 {
                    112 + x
                } else if y == 2 || y == 5 {
                    272 + x
                } else if y == 1 || y == 6 {
                    432 + x
                } else if y == 0 || y == 7 {
                    592 + x
                } else {
                    unreachable!()
                }
            } else if x >= WIDTH / 2 {
                if y == 3 || y == 4 {
                    16 + x - WIDTH / 2
                } else if y == 2 || y == 5 {
                    176 + x - WIDTH / 2
                } else if y == 1 || y == 6 {
                    336 + x - WIDTH / 2
                } else if y == 0 || y == 7 {
                    496 + x - WIDTH / 2
                } else {
                    unreachable!()
                }
            } else {
                unreachable!()
            }
        } else {
            if y == 11 || y == 12 {
                48 + WIDTH - x - 1
            } else if y == 10 || y == 13 {
                208 + WIDTH - x - 1
            } else if y == 9 || y == 14 {
                368 + WIDTH - x - 1
            } else if y == 8 || y == 15 {
                528 + WIDTH - x - 1
            } else {
                unreachable!()
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
