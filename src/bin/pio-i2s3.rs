//! PIO I2S driver for INMP441
//!
//! Data format 24-bit two's complement, skip first clock cycle, MSB first.
//!
//! Clock phase = 0: data is captured on the leading edge of each SCK pulse, and
//! transitions on the trailing edge, or some time before the first leading edge.
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]
#![feature(generic_const_exprs)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::*;
use embassy_rp::peripherals::{DMA_CH1, PIO0, SPI0};
use embassy_rp::pio::{FifoJoin, Pio, ShiftConfig, ShiftDirection};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peripheral, PeripheralRef};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::primitives::{Line, Primitive, PrimitiveStyle};
use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_graphics_core::prelude::*;
use fixed::traits::ToFixed;

use fixed_macro::types::U56F8;
use memory_lcd_spi::displays::LPM013M126A;
use memory_lcd_spi::MemoryLCD;
use micromath::F32Ext;

use {defmt_rtt as _, panic_probe as _};

// multicore
use embassy_executor::Executor;
use embassy_executor::_export::StaticCell;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// stack should be big enough to hold the local variables of the task
static mut CORE1_STACK: Stack<40960> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, [f32; 256], 2> = Channel::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let config = embassy_rp::config::Config::default();
    let p = embassy_rp::init(config);

    let mut delay = Delay;
    //let mut led = Output::new(p.PIN_25, Level::Low);

    info!("init screen");
    let cs = Output::new(p.PIN_17.degrade(), Level::Low); // SCS
    let mosi = p.PIN_19; // SI
    let clk = p.PIN_18; // SCLK
    let mut config = spi::Config::default();
    config.frequency = 9_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    let mut disp = Output::new(p.PIN_20, Level::Low); // DISP
    disp.set_high();

    let mut display: MemoryLCD<LPM013M126A<BinaryColor>, _, _> = MemoryLCD::new(spi, cs);
    //display.set_rotation(memory_lcd_spi::framebuffer::Rotation::Deg90);
    display.clear(BinaryColor::Off);
    display.update(&mut delay);

    info!("spawn core 1");
    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| unwrap!(spawner.spawn(core1_task(display))));
    });

    info!("init pio");
    let mut pio0 = Pio::new(p.PIO0);

    let ws = p.PIN_1;
    let sck = p.PIN_0;
    let sd = p.PIN_2;

    // F_SCK min 0.5Mhz, max 3.2MHz

    // single pixel
    let prg = pio_proc::pio_asm!(
        r#"

        // LRCLK, BCLK
        .side_set 2

         //set pindirs,1              side 0b00

        .wrap_target

//      public entry_point:
            set y, 30              side 0b11 [1]
        bitloop1:
            nop                     side 0b00 [1]
            in pins, 1              side 0b01   // rising edge, read data
            jmp y--, bitloop1       side 0b01
            set y, 30               side 0b00 [1]
            in pins, 1              side 0b01 [1]

        bitloop0:
            nop                    side 0b10 [1]
            jmp y--, bitloop0      side 0b11 [1]
            //push noblock           side 0b10 [1]
            nop           side 0b10 [1]


        .wrap
        "#,
    );

    let mut cfg = embassy_rp::pio::Config::default();

    // pin config
    let ws_pin = pio0.common.make_pio_pin(ws);
    let sck_pin = pio0.common.make_pio_pin(sck);
    let sd_pin = pio0.common.make_pio_pin(sd);

    pio0.sm0.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&ws_pin, &sck_pin]);
    pio0.sm0.set_pin_dirs(embassy_rp::pio::Direction::In, &[&sd_pin]);

    //cfg.set_out_pins(&[&led_pin]);
    //cfg.set_set_pins(&[&led_pin]);

    cfg.set_in_pins(&[&sd_pin]);

    let relocated = RelocatedProgram::new(&prg.program);

    // side set pins
    cfg.use_program(&pio0.common.load_program(&relocated), &[&sck_pin, &ws_pin]);

    // no div, full speed
    // 25_u16.into(); // 200ns
    //cfg.clock_divider = 200_u16.into(); // 200ns
    // let audio_freq: u32 = 8_000 * 64; // = 512_000
    // 44100 * 64 = 2_822_400
    // 16000 * 64 = 1_024_000
    // 125_000_000 / 2

    // 2_048_000 -> 8k
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(2_048_000)).to_fixed();
    //cfg.clock_divider = 65535_u16.into();

    cfg.shift_out = ShiftConfig {
        auto_fill: false, // use pull block
        direction: ShiftDirection::Left,
        threshold: 32,
    };
    //
    cfg.shift_in = ShiftConfig {
        auto_fill: true, // push
        direction: ShiftDirection::Left,
        threshold: 32,
    };
    // ????
    cfg.fifo_join = FifoJoin::RxOnly;

    pio0.sm0.set_config(&cfg);
    pio0.sm0.set_enable(true);

    let dma_in_ref = p.DMA_CH1.into_ref();

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| unwrap!(spawner.spawn(core0_task(pio0, dma_in_ref))));
}

#[embassy_executor::task]
async fn core0_task(mut pio0: Pio<'static, PIO0>, mut dma_in: PeripheralRef<'static, DMA_CH1>) {
    info!("core 0 task started: pio dma pull");
    let mut din = [0u32; 256];
    let mut samples = [0.0; 256];
    loop {
        pio0.sm0.rx().dma_pull(dma_in.reborrow(), &mut din).await;
        for (i, &d) in din.iter().enumerate() {
            let d = (d >> 7) & 0xff_ff_ff;

            if d & 0x80_00_00 != 0 {
                samples[i] = -((0x100_00_00 - d) as f32);
            } else {
                samples[i] = d as f32;
            }
        }
        //info!("core 0 recv {:?}", samples);
        //CHANNEL.try_send(samples);
        CHANNEL.send(samples).await;
        //       info!("core 0 pull");
    }
}

#[embassy_executor::task]
async fn core1_task(mut display: MemoryLCD<LPM013M126A<BinaryColor>, Spi<'static, SPI0, spi::Blocking>, Output<'static, AnyPin>>) {
    info!("core 1 task started: display");
    let mut delay = Delay;

    loop {
        display.clear(BinaryColor::Off);
        let mut samples = CHANNEL.recv().await;

        //info!("core 1 recv {}", samples.len());
        let spectrum = microfft::real::rfft_256(&mut samples);
        //spectrum[0].im = 0.0;

        let mut last = Point::new(10, 20);
        for (i, a) in spectrum.iter().map(|c| c.norm_sqr().sqrt()).enumerate() {
            //
            let a = a as u32 >> 15;
            //let a = a.log2() * 8.0 - 120.0;
            let a = a as i32;
            let a = a.max(1);
            let a = a.min(148);

            // bar
            // Line::new(Point::new(10, 20 + (i as i32)), Point::new(10 + a, 20 + i as i32))
            //    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            //   .draw(&mut *display);

            // dot
            // Pixel(Point::new(10 + a, 20 + i as i32), BinaryColor::On).draw(&mut *display);

            // line graph
            let pt = Point::new(10 + a, 20 + i as i32);
            Line::new(last, pt)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut *display);
            last = pt;
        }

        display.update(&mut delay).unwrap();
    }
}
