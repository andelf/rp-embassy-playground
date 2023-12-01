//! EPD7IN5B v2
//!
//! - BLOZI 7.5in BWR ESL
//! - UC8179 driver
//! - 老五数码之家 7.5寸三色墨水屏电子标签 9.9成新无老化

use embedded_graphics_core::pixelcolor::BinaryColor;

use embedded_graphics_core::primitives::Rectangle;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

/// EPD config
pub struct Config {
    /// KW-3f KWR-2F BWROTP-0f BWOTP-1f
    /// Pannel mode
    mode: u8,
    pub n2ocp: bool,
    /// 800
    pub nsource: u16,
    /// max 600, for 7.5in ESL, use 480
    pub ngate: u16,
}

impl Config {
    pub fn mode_bw() -> Self {
        Self {
            mode: 0x3F,
            n2ocp: true,
            nsource: 800,
            ngate: 480,
        }
    }

    pub fn mode_bwr() -> Self {
        Self {
            mode: 0x2F,
            n2ocp: false,
            nsource: 800,
            ngate: 480,
        }
    }

    pub fn mode_bwr_otp() -> Self {
        Self {
            mode: 0x0F,
            n2ocp: false,
            nsource: 800,
            ngate: 480,
        }
    }

    pub fn mode_bw_otp() -> Self {
        Self {
            mode: 0x1F,
            n2ocp: false,
            nsource: 800,
            ngate: 480,
        }
    }
}

/// UC8179 driver
pub struct EPD7in5<SPI, DC, BUSY> {
    spi: SPI,
    dc: DC,
    busy: BUSY,
}

impl<SPI, DC, BUSY> EPD7in5<SPI, DC, BUSY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    BUSY: InputPin,
{
    pub fn new(spi: SPI, dc: DC, busy: BUSY) -> Self {
        Self { spi, dc, busy }
    }

    fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        self.spi.write(&[cmd]);
    }

    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.write(data);
    }

    fn send_command_data(&mut self, cmd: u8, data: &[u8]) {
        self.send_command(cmd);
        self.send_data(data);
    }

    // negative logic
    pub fn busy_wait(&mut self) {
        loop {
            self.send_command(0x71);
            if self.busy.is_high().unwrap() {
                break;
            }
        }
    }

    pub fn power_off(&mut self) {
        self.send_command(0x02);
        self.busy_wait();
    }

    pub fn sleep(&mut self) {
        self.power_off();
        self.send_command_data(0x07, &[0xA5]);
    }

    pub fn init(&mut self, config: Config) {
        // Power setting
        // VDHR=3.0V
        // VDH=15.0V
        // VDL=-15.0V
        self.send_command_data(0x01, &[0x07, 0x17, 0x3f, 0x3f, 0x03]);

        // power on
        self.send_command(0x04);
        self.busy_wait();

        // panel setting
        // KW-3f   KWR-2F BWROTP 0f BWOTP 1f
        // self.send_command_data(0x00, &[0x0F]);
        self.send_command_data(0x00, &[config.mode]);

        // tres
        // 800x480: 0x03, 0x20, 0x01, 0xE0
        self.send_command_data(
            0x61,
            &[
                (config.nsource >> 8) as u8,
                (config.nsource & 0xFF) as u8,
                (config.ngate >> 8) as u8,
                (config.ngate & 0xFF) as u8,
            ],
        );

        // VCOM and data interval setting(CDI)
        // Border LUT selection(BDV) = 0b01 -> LUTWK
        // 0b00 -> LUTKK
        // enable N2OCP
        // if KW mode, and n2ocp is enabled
        if config.mode & 0x10 != 0 && config.n2ocp {
            self.send_command_data(0x50, &[0x01 | 0b1000, 0x07]);
        } else {
            self.send_command_data(0x50, &[0x31, 0x07]);
        }

        if config.mode & 0x20 != 0 {
            self.configure_init_update(); // safe default LUT table
        }
    }

    /// LUT for INIT
    pub fn configure_init_update(&mut self) {
        #[rustfmt::skip]
            const LUT_VCOM: [u8; 60] = [
                0b00,    0x0f, 0x0f, 0x0f, 0x0f,    0x01,
                0x00,    0x1A, 0x00, 0x10, 0x20,    0x01,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
            ];

        #[rustfmt::skip]
            const LUT_W2W: [u8; 42] = [
                0b10_01_10_01, 0x0f, 0x0f, 0x0f, 0x0f, 0x01,
                0b10_00_01_10, 0x1A, 0x00, 0x10, 0x20, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];

        #[rustfmt::skip]
            const LUT_K2W: [u8; 60] = [
                0b10_01_10_01, 0x0f, 0x0f, 0x0f, 0x0f, 0x01,
                0b10_00_01_10, 0x1A, 0x00, 0x10, 0x20, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        #[rustfmt::skip]
            const LUT_W2K: [u8; 60] = [
                0b10_01_10_01, 0x0f, 0x0f, 0x0f, 0x0f, 0x01,
                0b10_00_01_10, 0x1A, 0x00, 0x10, 0x20, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        #[rustfmt::skip]
            const LUT_K2K: [u8; 60] = [
                0b10_01_10_01, 0x0f, 0x0f, 0x0f, 0x0f, 0x01,
                0b10_00_01_10, 0x1A, 0x00, 0x10, 0x20, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        /* #[rustfmt::skip]
        const LUT_BD: [u8; 42] = [
            0b10_01_10_01, 0x0f, 0x0f, 0x0f, 0x0f, 0x01,
            0b10_00_01_10, 0x1A, 0x00, 0x10, 0x20, 0x01,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ]; */

        self.send_command_data(0x20, &LUT_VCOM);
        self.send_command_data(0x21, &LUT_W2W);
        self.send_command_data(0x22, &LUT_K2W);
        self.send_command_data(0x23, &LUT_W2K);
        self.send_command_data(0x24, &LUT_K2K);
        // self.send_command_data(0x25, &LUT_BD); // can be the same as LUT_WW
    }

    /// LUT for A2, Animation, 2 levels
    pub fn configure_partial_update(&mut self) {
        // 00b: VCOM_DC
        // 01b: VDH+VCOM_DC (VCOMH) 10b: VDL+VCOM_DC (VCOML) 11b: Floating
        #[rustfmt::skip]
            const LUT_VCOM: [u8; 60] = [
                0x00,    0x0f, 0x00, 0x00, 0x00,    0x01,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
            ];

        #[rustfmt::skip]
            const LUT_W2W: [u8; 42] = [
                0x00, 0x0f, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];

        #[rustfmt::skip]
            const LUT_K2W: [u8; 60] = [
                0b10_00_00_00, 0x12, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        #[rustfmt::skip]
            const LUT_W2K: [u8; 60] = [
                0b01_00_00_00, 0x12, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        #[rustfmt::skip]
            const LUT_K2K: [u8; 60] = [
                0x00, 0x0f, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        /*         #[rustfmt::skip]
        const LUT_BD: [u8; 42] = [
            0x00,    0x0f, 0x00, 0x00, 0x00,    0x01,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ]; */

        self.send_command_data(0x20, &LUT_VCOM);
        self.send_command_data(0x21, &LUT_W2W);
        self.send_command_data(0x22, &LUT_K2W);
        self.send_command_data(0x23, &LUT_W2K);
        self.send_command_data(0x24, &LUT_K2K);
        // self.send_command_data(0x25, &LUT_BD); // same as LUT_WW
    }

    /// LUT for Gray scale, init, no drive yet
    fn configure_gray_update_init(&mut self) {
        #[rustfmt::skip]
            const LUT_VCOM: [u8; 60] = [
                0x00,    0x09, 0x00, 0x00, 0x00,    0x01, // max = 9 level
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
                0x00,    0x00, 0x00, 0x00, 0x00,    0x00,
            ];

        #[rustfmt::skip]
            const LUT_W2W: [u8; 42] = [
                0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];

        #[rustfmt::skip]
            const LUT_K2W: [u8; 60] = [
                0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        #[rustfmt::skip]
            const LUT_W2K: [u8; 60] = [
                0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        #[rustfmt::skip]
            const LUT_K2K: [u8; 60] = [
                0x00, 0x01, 0x00, 0x00, 0x00, 0x01, // this is dangerous, and will damage the panel
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];

        self.send_command_data(0x20, &LUT_VCOM);
        self.send_command_data(0x21, &LUT_W2W);
        self.send_command_data(0x22, &LUT_K2W);
        self.send_command_data(0x23, &LUT_W2K);
        self.send_command_data(0x24, &LUT_K2K);
        // self.send_command_data(0x25, &LUT_BD); // same as LUT_WW
    }

    /// LUT for increase gray scale, drive W2K and K2K
    fn configure_gray_update_level(&mut self, level: u8) {
        #[rustfmt::skip]
            let lut_any2k: [u8; 60] = [
                0b01_00_00_00, level, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        // LUT_W2K
        self.send_command_data(0x23, &lut_any2k);
        // LUT_K2K
        self.send_command_data(0x24, &lut_any2k);
    }

    /// LUT for decrease gray scale, drive W2K and K2K
    fn configure_gray_clear_level(&mut self, level: u8) {
        #[rustfmt::skip]
            let lut_any2w: [u8; 60] = [
                0b10_00_00_00, level, 0x00, 0x00, 0x00, 0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ];
        // W2W and K2W are the same
        self.send_command_data(0x21, &lut_any2w);
        self.send_command_data(0x22, &lut_any2w);
    }

    pub fn refresh_gray4_image(&mut self, buf: &[u8], delay: &mut dyn DelayNs) {
        self.configure_gray_update_init();
        for level in (0..4).rev() {
            // level: (8, 4, 2, 1) + 1
            self.configure_gray_update_level(1 << level + 1);
            self.send_command(0x13);

            for chunk in buf.chunks(4) {
                let mut n = 0;
                for b in chunk {
                    if b & (0x10 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (1 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                }
                // 0xFF is white, 0x00 is black
                self.send_data(&[n]);
            }

            self.refresh(delay);
        }
    }

    pub fn refresh_gray2_image(&mut self, buf: &[u8], delay: &mut dyn DelayNs) {
        self.configure_gray_update_init();
        for level in [1, 0] {
            // level: 9, 5
            self.configure_gray_update_level(1 << (level + 2) + 1);
            self.send_command(0x13);

            for chunk in buf.chunks(2) {
                let mut n = 0;
                for b in chunk {
                    if b & (0b01_00_00_00 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (0b00_01_00_00 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (0b00_00_01_00 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                    if b & (0b00_00_00_01 << level) != 0 {
                        n = (n << 1) | 1;
                    } else {
                        n = n << 1;
                    }
                }
                // 0xFF is white, 0x00 is black
                self.send_data(&[n]);
            }

            self.refresh(delay);
        }
    }

    pub fn clear_gray4_image(&mut self, buf: &[u8]) {}

    pub fn clear_gray2_image(&mut self, buf: &[u8]) {}

    /// Clear is required to set initial state of the panel.
    /// Or else the panel will show random noise.
    pub fn clear(&mut self) {
        self.send_command(0x10);
        for i in 0..48000 {
            self.send_data(&[0xff]);
        }
        self.send_command(0x13);
        for i in 0..48000 {
            self.send_data(&[0xff]);
        }
    }

    /// Do display refresh
    pub fn refresh(&mut self, delay: &mut dyn DelayNs) {
        self.send_command(0x12);
        delay.delay_ms(100_u32); //must
        self.busy_wait();
    }

    pub fn update_frame(&mut self, buf: &[u8]) {
        self.send_command(0x13); // NEW buf by default
        self.send_data(buf);
    }

    pub fn update_bw_frame(&mut self, buf: &[u8]) {
        self.send_command(0x10); // OLD buf, aka. BW buf
        self.send_data(buf);
    }

    pub fn update_red_frame(&mut self, buf: &[u8]) {
        self.send_command(0x13); // NEW buf, aka. RED buf
        self.send_data(buf);
    }

    pub fn set_partial_refresh(&mut self, rect: Rectangle) {
        const PARTIAL_WINDOW: u8 = 0x90;
        const PARTIAL_IN: u8 = 0x91;
        const PARTIAL_OUT: u8 = 0x92;

        self.send_command(PARTIAL_IN);
        self.send_command(PARTIAL_WINDOW);
        let x0 = (rect.top_left.x as u16) & 0b1111111000;
        let x1 = rect.bottom_right().unwrap().x as u16;
        let y0 = rect.top_left.y as u16 | 0b111;
        let y1 = rect.bottom_right().unwrap().y as u16;
        self.send_data(&x0.to_be_bytes());
        self.send_data(&x1.to_be_bytes());
        self.send_data(&y0.to_be_bytes());
        self.send_data(&y1.to_be_bytes());
        self.send_data(&[0x01]); // PT_SCAN=Gates scan both inside and outside of the partial window
    }

    pub fn unset_partial_refresh(&mut self) {
        const PARTIAL_OUT: u8 = 0x92;
        self.send_command(PARTIAL_OUT);
    }
}
