//! ST7049A, 4 x 320 Dot Palette LCD Controller/Driver
//! FSC (Field Sequential Color) LCD Controller/Driver

use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_graphics::{
    pixelcolor::raw::RawU4,
    prelude::{Dimensions, DrawTarget, PixelColor, Point, Size},
    primitives::Rectangle,
    Pixel,
};
use embedded_graphics_core::{
    pixelcolor::{raw::RawU24, Rgb888},
    prelude::{OriginDimensions, RawData, RgbColor},
};

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

impl From<Color3> for RawU4 {
    fn from(c: Color3) -> Self {
        RawU4::new(c as u8)
    }
}

impl From<RawU4> for Color3 {
    fn from(c: RawU4) -> Self {
        match c.into_inner() {
            n => unsafe { core::mem::transmute(n) },
        }
    }
}

impl PixelColor for Color3 {
    type Raw = RawU4;
}

impl RgbColor for Color3 {
    fn r(&self) -> u8 {
        (self == &Self::Red || self == &Self::Yellow || self == &Self::Pink || self == &Self::White) as u8
    }

    fn g(&self) -> u8 {
        (self == &Self::Green || self == &Self::Yellow || self == &Self::Cyan || self == &Self::White) as u8
    }

    fn b(&self) -> u8 {
        (self == &Self::Blue || self == &Self::Cyan || self == &Self::Pink || self == &Self::White) as u8
    }

    const MAX_R: u8 = 1;

    const MAX_G: u8 = 1;

    const MAX_B: u8 = 1;

    const BLACK: Self = Self::Black;

    const RED: Self = Self::Red;

    const GREEN: Self = Self::Green;

    const BLUE: Self = Self::Blue;

    const YELLOW: Self = Self::Yellow;

    const MAGENTA: Self = Self::Pink;

    const CYAN: Self = Self::Cyan;

    const WHITE: Self = Self::White;
}

impl From<Rgb888> for Color3 {
    fn from(c: Rgb888) -> Self {
        let raw3bit = (((c.r() >= 128) as u8) << 2) | (((c.g() >= 128) as u8) << 1) | ((c.b() >= 128) as u8);
        match raw3bit {
            0 => Self::Black,
            1 => Self::Blue,
            2 => Self::Green,
            3 => Self::Cyan,
            4 => Self::Red,
            5 => Self::Pink,
            6 => Self::Yellow,
            7 => Self::White,
            _ => unreachable!(),
        }
    }
}

pub struct Display<DI> {
    di: DI,
    pub buf: [u8; PIXELS],
}

impl<DI: WriteOnlyDataCommand> Display<DI> {
    pub fn new(di: DI) -> Self {
        Self { di, buf: [0; PIXELS] }
    }

    fn send_cmd(&mut self, cmd: u8, data: &[u8]) -> Result<(), DisplayError> {
        self.di.send_commands(DataFormat::U8(&[cmd]))?;
        if !data.is_empty() {
            self.di.send_data(DataFormat::U8(data))?
        }
        Ok(())
    }

    pub fn init(&mut self) -> Result<(), DisplayError> {
        // self.di.send_commands(DataFormat::U8(&[0x01]))?; // SW reset

        self.di.send_commands(DataFormat::U8(&[0x11]))?; // sleep out

        self.di.send_commands(DataFormat::U8(&[0xD2]))?; // power setting, all on
        self.di.send_data(DataFormat::U8(&[0x00]))?;

        self.di.send_commands(DataFormat::U8(&[0xc0]))?; // Vop set, #important
        self.di.send_data(DataFormat::U8(&[200, 0]))?;

        self.di.send_commands(DataFormat::U8(&[0xc3]))?; // bias selection
        self.di.send_data(DataFormat::U8(&[1]))?; //default=0(1/2), or 1=1/3

        self.di.send_commands(DataFormat::U8(&[0xB0]))?; // Duty set
        self.di.send_data(DataFormat::U8(&[0b11]))?; // 4 duty

        self.send_cmd(0xB1, &[0b10])?; // LED high active

        self.di.send_commands(DataFormat::U8(&[0xB2]))?; // Frame frequency
        self.di.send_data(DataFormat::U8(&[0x1f]))?; // 200Hz, as fast as possible

        self.di.send_commands(DataFormat::U8(&[0xB5]))?; // driver mode
                                                         // more scan = 2 fields
        self.di.send_data(DataFormat::U8(&[0b1100, 1, 1, 1]))?; // 0b1100, 1, 1, 1

        // waveform
        self.send_cmd(0xB6, &[20, 20, 20, 170, 200, 200])?;
        //      self.di.send_commands(DataFormat::U8(&[0xB6]))?; // led waveform
        // self.di.send_data(DataFormat::U8(&[20, 20, 20, 100, 100, 100]))?;
        //    self.di.send_data(DataFormat::U8(&[20, 20, 20, 200, 200, 200]))?;
        //self.di.send_data(DataFormat::U8(&[20, 20, 20, 50, 50, 50]))?;
        // 1:0.75:0.35
        //self.di
        //    .send_data(DataFormat::U8(&[0x50, 0x50, 0x50, 150, 150, 150]))?;

        self.di.send_commands(DataFormat::U8(&[0xB7]))?; // LCD scan set, scan direction
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
                self.di.send_data(DataFormat::U8(&[0b1_000 | (color as u8)]))?;
            }
            None => {
                self.di.send_commands(DataFormat::U8(&[0xD4]))?; // RGB LED control
                self.di.send_data(DataFormat::U8(&[0b1_000]))?;
            }
        }
        Ok(())
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
        } else if y < 4 {
            self.buf[idx as usize] = ((color as u8) << 4) | (self.buf[idx as usize] & 0x0f);
        } else {
            self.buf[idx as usize] = (color as u8) | (self.buf[idx as usize] & 0xf0);
        }
    }
}

impl<DI> OriginDimensions for Display<DI> {
    fn size(&self) -> Size {
        Size::new(64, 16)
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
