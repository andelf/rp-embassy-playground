//! LPM009M360A display driver
//!
//! 72x144

use embedded_graphics::{
    pixelcolor::raw::RawU4,
    prelude::{DrawTarget, OriginDimensions, PixelColor, Point, RawData, RgbColor, Size},
    Pixel,
};

const CMD_NO_UPDATE: u8 = 0x00;
const CMD_BLINKING_BLACK: u8 = 0x10;
const CMD_BLINKING_INVERSION: u8 = 0x14;
const CMD_BLINKING_WHITE: u8 = 0x18;
const CMD_ALL_CLEAR: u8 = 0x20;
const CMD_VCOM: u8 = 0x40;
const CMD_UPDATE: u8 = 0x90;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
pub struct Rgb111(RawU4);

impl PixelColor for Rgb111 {
    type Raw = RawU4;
}

impl Rgb111 {
    pub fn new(red: u8, green: u8, blue: u8) -> Self {
        Self(RawU4::new((red << 3) | (green << 2) | (blue << 1)))
    }

    pub fn from_raw(raw: u8) -> Self {
        Self(RawU4::new((raw & 0b111) << 1))
    }
}

impl RgbColor for Rgb111 {
    fn r(&self) -> u8 {
        (self.0.into_inner() >> 4) & 0b11
    }

    fn g(&self) -> u8 {
        (self.0.into_inner() >> 2) & 0b11
    }

    fn b(&self) -> u8 {
        self.0.into_inner() & 0b11
    }

    const MAX_R: u8 = 0b1;

    const MAX_G: u8 = 0b1;

    const MAX_B: u8 = 0b1;

    const BLACK: Self = Self(RawU4::new(0));

    const RED: Self = Self(RawU4::new(0b1000));

    const GREEN: Self = Self(RawU4::new(0b0100));

    const BLUE: Self = Self(RawU4::new(0b0010));

    const YELLOW: Self = Self(RawU4::new(0b1100));

    const MAGENTA: Self = Self(RawU4::new(0b1010));

    const CYAN: Self = Self(RawU4::new(0b0110));

    const WHITE: Self = Self(RawU4::new(0b1110));
}

pub struct LPM009M360A {
    buffer: [u8; 72 * 144 / 2],
}

impl LPM009M360A {
    pub fn new() -> Self {
        Self {
            buffer: [0; 72 * 144 / 2],
        }
    }

    //pub fn clear(&mut self) {
    //    self.buffer = [0; 72 * 144 / 2];
    //}

    pub fn iter_rows(&self) -> RowIterator {
        RowIterator::new(&self.buffer, 0)
    }
}

pub struct RowIterator<'a> {
    buffer: &'a [u8],
    row: usize,
}

impl<'a> RowIterator<'a> {
    pub fn new(buffer: &'a [u8], row: usize) -> Self {
        Self { buffer, row }
    }
}

impl<'a> Iterator for RowIterator<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<Self::Item> {
        if self.row >= 144 {
            return None;
        }

        let row = self.row;
        self.row += 1;

        Some(&self.buffer[row * 72 / 2..(row + 1) * 72 / 2])
    }
}

impl OriginDimensions for LPM009M360A {
    fn size(&self) -> Size {
        Size::new(72, 144)
    }
}

impl DrawTarget for LPM009M360A {
    type Color = Rgb111;

    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(Point { x, y }, color) in pixels.into_iter() {
            if !(0..72).contains(&x) || !(0..144).contains(&y) {
                continue;
            }

            let x = x as usize;
            let y = y as usize;
            let index = (y * 72 + x) / 2;
            let value = self.buffer[index];
            if x % 2 == 1 {
                self.buffer[index] = (value & 0b11110000) | color.0.into_inner();
            } else {
                self.buffer[index] = (value & 0b00001111) | (color.0.into_inner() << 4);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let color = color.0.into_inner();
        let raw = color << 4 | color;
        self.buffer.fill(raw);
        Ok(())
    }
}
