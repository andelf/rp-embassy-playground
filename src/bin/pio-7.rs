//! SPI 9-bit in PIO
//!
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![allow(unused_must_use)]

use core::fmt::Write;
use defmt::*;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{FifoJoin, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::Peripheral;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::text::Text;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::{DrawTarget, Point, RgbColor, WebColors};
use embedded_graphics_core::Drawable;
use fixed::traits::ToFixed;
// use fixed::types::U56F8;

use fixed_macro::types::U56F8;
use micromath::F32Ext;

use {defmt_rtt as _, panic_probe as _};

pub struct Spi9Bit<'l> {
    sm: StateMachine<'l, PIO0, 0>,
}

impl<'l> Spi9Bit<'l> {
    pub fn new(
        pio: impl Peripheral<P = PIO0> + 'l,
        clk: impl PioPin,
        mosi: impl PioPin,
        cs: impl PioPin,
    ) -> Spi9Bit<'l> {
        let Pio {
            mut common, mut sm0, ..
        } = Pio::new(pio);

        let prg = pio_proc::pio_asm!(
            r#"
            .side_set 2
            .wrap_target

            bitloop:
                out pins, 1        side 0x0
                jmp !osre bitloop  side 0x1     ; Fall-through if TXF empties
                nop                side 0x0 [1] ; CSn back porch

            public entry_point:                 ; Must set X,Y to n-2 before starting!
                pull ifempty       side 0x2 [1] ; Block with CSn high (minimum 2 cycles)

            .wrap                               ; Note ifempty to avoid time-of-check race

            "#,
        );

        let clk = common.make_pio_pin(clk);
        let mosi = common.make_pio_pin(mosi);
        let cs = common.make_pio_pin(cs);

        sm0.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&clk, &mosi, &cs]);

        let relocated = RelocatedProgram::new(&prg.program);
        let mut cfg = embassy_rp::pio::Config::default();

        // cs:  side set 0b10
        // clk: side set 0b01
        // fist side_set, lower bit in side_set
        cfg.use_program(&common.load_program(&relocated), &[&clk, &cs]);

        cfg.clock_divider = 1u8.into(); // run at full speed
        cfg.set_out_pins(&[&mosi]);
        //  cfg.set_set_pins(&[&mosi]);
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            direction: ShiftDirection::Left,
            threshold: 9, // 9-bit mode
        };
        cfg.fifo_join = FifoJoin::TxOnly;
        sm0.set_config(&cfg);

        sm0.set_enable(true);

        Self { sm: sm0 }
    }

    pub fn write_data(&mut self, val: u8) {
        while self.sm.tx().full() {}
        self.sm.tx().push(0x80000000 | ((val as u32) << 23));
    }

    pub fn write_command(&mut self, val: u8) {
        while self.sm.tx().full() {}
        self.sm.tx().push((val as u32) << 23);
    }
}

impl<'l> WriteOnlyDataCommand for Spi9Bit<'l> {
    fn send_commands(&mut self, cmd: DataFormat<'_>) -> Result<(), DisplayError> {
        match cmd {
            DataFormat::U8(cmds) => {
                for &c in cmds {
                    self.write_command(c);
                }
            }
            _ => {
                defmt::todo!();
            }
        }
        Ok(())
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        match buf {
            DataFormat::U8(buf) => {
                for &byte in buf {
                    self.write_data(byte);
                }
            }
            DataFormat::U16BEIter(it) => {
                for raw in it {
                    self.write_data((raw >> 8) as u8);
                    self.write_data((raw & 0xff) as u8);
                }
            }
            _ => {
                defmt::todo!();
            }
        }

        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = embassy_rp::config::Config::default();
    let p = embassy_rp::init(config);

    let mut delay = Delay;
    let mut led = Output::new(p.PIN_25, Level::Low);

    // side set pin 必须连续
    let mosi = p.PIN_19; // SI
    let clk = p.PIN_17; // SCLK
    let cs = p.PIN_18; // CS

    let rst = p.PIN_20;

    let ws2812 = p.PIN_23;

    //    let mut pio0 = Pio::new(p.PIO0);

    let mut di = Spi9Bit::new(p.PIO0, clk, mosi, cs);

    let rst = Output::new(rst, Level::High);

    let mut display = mipidsi::Builder::st7789(di)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(&mut delay, Some(rst))
        .unwrap();
    info!("init ok");

    display.clear(Rgb565::CYAN).unwrap();

    let mut buf = heapless::String::<255>::new();

    let start = Instant::now();
    let mut frames = 0;

    let char_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::CSS_DARK_GOLDENROD)
        .background_color(Rgb565::WHITE)
        .build();

    loop {
        led.toggle();
        buf.clear();

        /* if frames % 2 == 1 {
            display.clear(Rgb565::YELLOW).unwrap()
        } else {
            display.clear(Rgb565::CYAN).unwrap()
        }
        */

        let fps = frames as f32 / start.elapsed().as_millis() as f32 * 1000.0;

        core::write!(&mut buf, "count: {}\ntotal fps: {:.1}", frames, fps).unwrap();
        Text::new(&buf, Point::new(20, 100), char_style)
            .draw(&mut display)
            .unwrap();
        led.toggle();
        frames += 1;

        //di.write_data(0xaa);
        //di.write_data(0xaa);
        //Timer::after(Duration::from_millis(1)).await;

        // display.clear(Rgb565::RED).unwrap();
    }
}
