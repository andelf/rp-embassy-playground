//! 128 x 128, 16 Gray Scale Dot Matrix
//! OLED/PLED Segment/Common Driver with Controller

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embedded_graphics::{
    pixelcolor::{raw::RawU4, Gray4},
    prelude::{Dimensions, DrawTarget, GrayColor, Point, Size},
    primitives::Rectangle,
    Pixel,
};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;

const WIDTH: u32 = 128;
const HEIGHT: u32 = 128;

const PIXELS: usize = 128 * 128 / 2; // 4 bit per pixel

pub struct Display<DI> {
    di: DI,
    pub buffer: [u8; PIXELS],
}

impl<DI: WriteOnlyDataCommand> Display<DI> {
    /// Creates the SSD1327 Display.
    ///
    /// Make sure to reset and initialize the display before use!
    pub fn new(di: DI) -> Self {
        Self {
            di,
            buffer: [0; PIXELS],
        }
    }

    /// Resets the display.
    pub fn reset<RST, DELAY>(&mut self, rst: &mut RST, delay: &mut DELAY) -> Result<(), DisplayError>
    where
        RST: OutputPin,
        DELAY: DelayUs<u32>,
    {
        rst.set_high().map_err(|_| DisplayError::BusWriteError)?;
        delay.delay_us(100_000);

        rst.set_low().map_err(|_| DisplayError::BusWriteError)?;
        delay.delay_us(100_000);

        rst.set_high().map_err(|_| DisplayError::BusWriteError)?;
        delay.delay_us(100_000);

        Ok(())
    }

    /// Initializes the display.
    pub fn init(&mut self) -> Result<(), DisplayError> {
        self.send_command(Command::DisplayOff)?;
        self.send_command(Command::ColumnAddress { start: 0, end: 127 })?;
        self.send_command(Command::RowAddress { start: 0, end: 127 })?;
        self.send_command(Command::Contrast(0x80))?;
        self.send_command(Command::SetRemap(0x51))?;
        self.send_command(Command::StartLine(0x00))?;
        self.send_command(Command::Offset(0x00))?;
        self.send_command(Command::DisplayModeNormal)?;
        self.send_command(Command::MuxRatio(0x7f))?;
        self.send_command(Command::PhaseLength(0xf1))?;
        self.send_command(Command::FrontClockDivider(0x00))?;
        self.send_command(Command::FunctionSelectionA(0x01))?;
        self.send_command(Command::SecondPreChargePeriod(0x0f))?;
        self.send_command(Command::ComVoltageLevel(0x0f))?;
        self.send_command(Command::PreChargeVoltage(0x08))?;
        self.send_command(Command::FunctionSelectionB(0x62))?;
        self.send_command(Command::CommandLock(0x12))?;
        self.send_command(Command::DisplayOn)?;

        Ok(())
    }

    /// Allows to send custom commands to the display.
    pub fn send_command(&mut self, command: Command) -> Result<(), DisplayError> {
        command.send(&mut self.di)
    }

    /// Flushes the display, and makes the output visible on the screen.
    pub fn flush(&mut self) -> Result<(), DisplayError> {
        self.di.send_data(DataFormat::U8(&self.buffer))
    }
}

impl<DI> Dimensions for Display<DI> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(0, 0), Size::new(128, 128))
    }
}

impl<DI> DrawTarget for Display<DI> {
    type Color = Gray4;
    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let Pixel(point, color) = pixel;
            if point.x >= WIDTH as _ || point.y >= HEIGHT as _ || point.x < 0 || point.y < 0 {
                continue;
            }

            // defmt::info!("pt {:?} {}", point.x, point.y);
            let idx = (point.x / 2 + point.y * 64) as usize;
            if point.x % 2 == 0 {
                self.buffer[idx] = (color.luma() << 4) | (self.buffer[idx] & 0x0f);
            } else {
                self.buffer[idx] = (color.luma() & 0x0f) | (self.buffer[idx] & 0xf0);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let b = color.luma() << 4 | color.luma();
        self.buffer.fill(b);
        Ok(())
    }
}

/// Holds commands which can be sent to the display.
pub enum Command {
    /// Turn display off (0xAE)
    DisplayOff,
    /// Turn display on (0xAF)
    DisplayOn,
    /// Set up column start and end address (0x15)
    ColumnAddress {
        /// The start column address
        start: u8,
        /// The end column address
        end: u8,
    },
    /// Set up row start and end address (0x75)
    RowAddress {
        /// The start row address
        start: u8,
        /// The end row address
        end: u8,
    },
    /// Contrast Control (0x81)
    Contrast(u8),
    /// Re-map setting in Graphic Display Data RAM  (0xA0)
    SetRemap(u8),
    /// Display Start Line (0xA1)
    StartLine(u8),
    /// Display Offset (0xA2)
    Offset(u8),
    /// Normal Display Mode (0xA4)
    DisplayModeNormal,
    /// Multiplex Ratio (0xA8)
    MuxRatio(u8),
    /// Phase Length (0xB1)
    PhaseLength(u8),
    /// Front Clock Divider / Oscillator Frequency (0xB3)
    FrontClockDivider(u8),
    /// Function Selection A (0xAB)
    FunctionSelectionA(u8),
    /// Second Pre-Charge Period (0xB6)
    SecondPreChargePeriod(u8),
    /// COM deselect voltage level (0xBE)
    ComVoltageLevel(u8),
    /// Pre-Charge Voltage (0xBC)
    PreChargeVoltage(u8),
    /// Function Selection B (0xD5)
    FunctionSelectionB(u8),
    /// Function Selection B (0xD5)
    CommandLock(u8),
}

impl Command {
    pub(crate) fn send<DI>(self, display: &mut DI) -> Result<(), DisplayError>
    where
        DI: WriteOnlyDataCommand,
    {
        let (data, len) = match self {
            Self::DisplayOn => ([0xAF, 0, 0], 1),
            Self::DisplayOff => ([0xAE, 0, 0], 1),
            Self::ColumnAddress { start, end } => ([0x15, start, end], 3),
            Self::RowAddress { start, end } => ([0x75, start, end], 3),
            Self::Contrast(value) => ([0x81, value, 0], 2),
            Self::SetRemap(value) => ([0xA0, value, 0], 2),
            Self::StartLine(value) => ([0xA1, value, 0], 2),
            Self::Offset(value) => ([0xA2, value, 0], 2),
            Self::DisplayModeNormal => ([0xA4, 0, 0], 1),
            Self::MuxRatio(value) => ([0xA8, value, 0], 2),
            Self::PhaseLength(value) => ([0xB1, value, 0], 2),
            Self::FrontClockDivider(value) => ([0xB3, value, 0], 2),
            Self::FunctionSelectionA(value) => ([0xAB, value, 0], 2),
            Self::SecondPreChargePeriod(value) => ([0xB6, value, 0], 2),
            Self::ComVoltageLevel(value) => ([0xBE, value, 0], 2),
            Self::PreChargeVoltage(value) => ([0xBC, value, 0], 2),
            Self::FunctionSelectionB(value) => ([0xD5, value, 0], 2),
            Self::CommandLock(value) => ([0xFD, value, 0], 2),
        };
        display.send_commands(DataFormat::U8(&data[0..len]))
    }
}
