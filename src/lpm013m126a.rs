//! LPM013M126A
//!
//! 240x240 round display
//! RGB222

use embedded_hal::{
    blocking::delay::DelayUs,
    digital::v2::{OutputPin, ToggleableOutputPin},
};

use embedded_graphics::{
    pixelcolor::raw::RawU8,
    prelude::{Dimensions, DrawTarget, PixelColor, Point, RawData, RgbColor, Size},
    primitives::Rectangle,
    Pixel,
};

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
pub struct Rgb222(RawU8);

impl PixelColor for Rgb222 {
    type Raw = RawU8;
}

impl From<RawU8> for Rgb222 {
    fn from(raw: RawU8) -> Self {
        Self(raw)
    }
}

impl RgbColor for Rgb222 {
    fn r(&self) -> u8 {
        (self.0.into_inner() >> 4) as u8 & 0b11
    }

    fn g(&self) -> u8 {
        (self.0.into_inner() >> 2) as u8 & 0b11
    }

    fn b(&self) -> u8 {
        self.0.into_inner() as u8 & 0b11
    }

    const MAX_R: u8 = 0b11;

    const MAX_G: u8 = 0b11;

    const MAX_B: u8 = 0b11;

    const BLACK: Self = Self(RawU8::new(0));

    const RED: Self = Self(RawU8::new(0b110000));

    const GREEN: Self = Self(RawU8::new(0b001100));

    const BLUE: Self = Self(RawU8::new(0b000011));

    const YELLOW: Self = Self(RawU8::new(0b111100));

    const MAGENTA: Self = Self(RawU8::new(0b110011));

    const CYAN: Self = Self(RawU8::new(0b001111));

    const WHITE: Self = Self(RawU8::new(0b111111));
}

impl Rgb222 {
    pub fn new(red: u8, green: u8, blue: u8) -> Self {
        Self(RawU8::new((red << 4) | (green << 2) | blue))
    }

    pub fn from_raw(raw: u8) -> Self {
        Self(RawU8::new(raw))
    }
}

/// The display
pub struct LPM013M126A {
    pub fb: [u8; 240 * 240],
}

impl LPM013M126A {
    pub fn new() -> Self {
        Self { fb: [0u8; 240 * 240] }
    }
}

impl Dimensions for LPM013M126A {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(1, 1), Size::new(240, 240))
    }
}

impl DrawTarget for LPM013M126A {
    type Color = Rgb222;

    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if let Ok((x @ 1..=240, y @ 1..=240)) = coord.try_into() {
                let x = x - 1;
                let y = y - 1;
                if (x - 120) * (x - 120) + (y - 120) * (y - 120) > 120 * 120 {
                    continue;
                }
                let pos = (y * 240 + x) as usize;
                let raw_color = color.0.into_inner();
                self.fb[pos] = raw_color;
            }
        }
        Ok(())
    }
}

impl LPM013M126A {
    pub fn reset<E>(rst: &mut dyn OutputPin<Error = E>, delay: &mut dyn DelayUs<u16>) -> Result<(), E> {
        rst.set_high()?;
        rst.set_low()?;
        delay.delay_us(500);
        rst.set_high()?;
        delay.delay_us(23_000);
        Ok(())
    }

    pub fn init<E>(
        vst: &mut dyn OutputPin<Error = E>,
        vck: &mut dyn OutputPin<Error = E>,
        hst: &mut dyn OutputPin<Error = E>,
        hck: &mut dyn OutputPin<Error = E>,
        enb: &mut dyn OutputPin<Error = E>,
    ) -> Result<(), E> {
        vck.set_low()?;
        vst.set_low()?;
        hst.set_low()?;
        hck.set_low()?;
        enb.set_low()?;
        Ok(())
    }

    pub fn flush<E>(
        &self,
        vst: &mut dyn OutputPin<Error = E>,
        vck: &mut dyn ToggleableOutputPin<Error = E>,
        hst: &mut dyn OutputPin<Error = E>,
        hck: &mut dyn ToggleableOutputPin<Error = E>,
        enb: &mut dyn OutputPin<Error = E>,
        r1: &mut dyn OutputPin<Error = E>,
        r2: &mut dyn OutputPin<Error = E>,
        g1: &mut dyn OutputPin<Error = E>,
        g2: &mut dyn OutputPin<Error = E>,
        b1: &mut dyn OutputPin<Error = E>,
        b2: &mut dyn OutputPin<Error = E>,
        delay: &mut dyn DelayUs<u16>,
    ) -> Result<(), E> {
        vst.set_high()?;
        delay.delay_us(20_000);

        for i in 1..=488 {
            vck.toggle()?;

            if i == 1 {
                vst.set_low()?;
                delay.delay_us(21_000);
            }

            if i >= 2 && i <= 481 {
                hck.toggle()?;
                delay.delay_us(1_000);

                for j in 1..=123 {
                    if j == 1 {
                        hst.set_high()?;
                    } else if j == 2 {
                        hst.set_low()?;
                    }

                    if j == 2 {
                        enb.set_high()?;
                    } else if j == 20 {
                        enb.set_low()?;
                    }

                    if j >= 1 && j <= 120 {
                        let y = (i - 2) / 2;
                        let x = j - 1;

                        let pos = (x * 2) + 240 * y;
                        let raw = self.fb;
                        if i % 2 == 0 {
                            // LPB
                            let pixel = raw[pos];
                            if pixel & 0b10_00_00 != 0 {
                                r1.set_high()?;
                            } else {
                                r1.set_low()?;
                            }
                            if pixel & 0b00_10_00 != 0 {
                                g1.set_high()?;
                            } else {
                                g1.set_low()?;
                            }
                            if pixel & 0b00_00_10 != 0 {
                                b1.set_high()?;
                            } else {
                                b1.set_low()?;
                            }
                            let pixel = raw[pos + 1];
                            if pixel & 0b10_00_00 != 0 {
                                r2.set_high()?;
                            } else {
                                r2.set_low()?;
                            }
                            if pixel & 0b00_10_00 != 0 {
                                g2.set_high()?;
                            } else {
                                g2.set_low()?;
                            }
                            if pixel & 0b00_00_10 != 0 {
                                b2.set_high()?;
                            } else {
                                b2.set_low()?;
                            }
                        } else {
                            // SPB
                            let pixel = raw[pos];
                            if pixel & 0b01_00_00 != 0 {
                                r1.set_high()?;
                            } else {
                                r1.set_low()?;
                            }
                            if pixel & 0b00_01_00 != 0 {
                                g1.set_high()?;
                            } else {
                                g1.set_low()?;
                            }
                            if pixel & 0b00_00_01 != 0 {
                                b1.set_high()?;
                            } else {
                                b1.set_low()?;
                            }
                            let pixel = raw[pos + 1];
                            if pixel & 0b01_00_00 != 0 {
                                r2.set_high()?;
                            } else {
                                r2.set_low()?;
                            }
                            if pixel & 0b00_01_00 != 0 {
                                g2.set_high()?;
                            } else {
                                g2.set_low()?;
                            }
                            if pixel & 0b00_00_01 != 0 {
                                b2.set_high()?;
                            } else {
                                b2.set_low()?;
                            }
                        }
                    }

                    delay.delay_us(1);

                    hck.toggle()?;
                }
            } else {
                delay.delay_us(1_000);
            }
        }

        Ok(())
    }
}
