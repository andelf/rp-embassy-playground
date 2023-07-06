//! WS2812 PIO dirver
//! -strip with 14 leds
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![allow(unused_must_use)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::pio::{FifoJoin, Pio, ShiftConfig, ShiftDirection};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::Peripheral;
use embassy_time::{Delay, Duration, Instant, Timer};
use fixed::traits::ToFixed;
// use fixed::types::U56F8;

use fixed_macro::types::U56F8;
use micromath::F32Ext;

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = embassy_rp::config::Config::default();
    let p = embassy_rp::init(config);

    let mut delay = Delay;
    //let mut led = Output::new(p.PIN_25, Level::Low);

    info!("init pio");
    let mut pio0 = Pio::new(p.PIO0);

    let ws = p.PIN_1;
    let sck = p.PIN_0;
    let sd = p.PIN_2;

    let ws2812 = p.PIN_23;

    // F_SCK min 0.5Mhz, max 3.2MHz

    // single pixel
    let prg = pio_proc::pio_asm!(
        r#"

        // LRCLK, BCLK
        .side_set 2
        .wrap_target

       // set pindirs,1              side 0b00

        bitloop1:
            in pins, 1              side 0b00 // rising edge, read data
            jmp y--, bitloop1       side 0b01
            in pins, 1              side 0b10
            set y, 30               side 0b11

        bitloop0:
            nop                    side 0b10
            jmp y--, bitloop0      side 0b11
            // nop                    side 0b01
            push noblock           side 0b00

        public entry_point:
            set y, 30              side 0b01

        .wrap
        "#,
    );

    let mut cfg = embassy_rp::pio::Config::default();

    // pin config
    let led_pin = pio0.common.make_pio_pin(p.PIN_25);

    let ws_pin = pio0.common.make_pio_pin(ws);
    let sck_pin = pio0.common.make_pio_pin(sck);
    let sd_pin = pio0.common.make_pio_pin(sd);

    let neopixel_pin = pio0.common.make_pio_pin(ws2812);

    pio0.sm0.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&ws_pin, &sck_pin]);
    pio0.sm0.set_pin_dirs(embassy_rp::pio::Direction::In, &[&sd_pin]);

    //cfg.set_out_pins(&[&led_pin]);
    //cfg.set_set_pins(&[&led_pin]);

    cfg.set_in_pins(&[&sd_pin]);

    let relocated = RelocatedProgram::new(&prg.program);

    // side set pins
    cfg.use_program(&pio0.common.load_program(&relocated), &[&sck_pin, &ws_pin]);

    // no div, full speed
    //cfg.clock_divider = 9_u16.into();
    // target freq = 800hz

    // 25_u16.into(); // 200ns
    //cfg.clock_divider = 200_u16.into(); // 200ns
    // let audio_freq: u32 = 8_000 * 64; // = 512_000
    // 44100 * 64 = 2_822_400
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(512_000)).to_fixed();

    cfg.shift_out = ShiftConfig {
        auto_fill: false, // use pull block
        direction: ShiftDirection::Left,
        threshold: 32,
    };
    //
    cfg.shift_in = ShiftConfig {
        auto_fill: false, // push
        direction: ShiftDirection::Left,
        threshold: 32,
    };
    // ????
    cfg.fifo_join = FifoJoin::RxOnly;

    pio0.sm0.set_config(&cfg);
    pio0.sm0.set_enable(true);
    info!("begin rx");
    let mut dma_in_ref = p.DMA_CH1.into_ref();
    let mut din = [0u32; 128];
    let mut samples = [0.0; 128];
    let mut amplitudes = [0u32; 128];
    loop {
        pio0.sm0.rx().dma_pull(dma_in_ref.reborrow(), &mut din).await;
        for (i, &d) in din.iter().enumerate() {
            //            if d & (1 << 31) != 0 {
            //              info!("=> {:?}", din);
            //            defmt::panic!("ff {}", d);
            //      }

            samples[i] = (d >> 16) as f32;
        }
        //info!("=> {:?}", din);

        let spectrum = microfft::real::rfft_128(&mut samples);
        spectrum[0].im = 0.0;

        // the spectrum has a spike at index `signal_freq`
        for (i, a) in spectrum.iter().map(|c| c.l1_norm() as u32).enumerate() {
            amplitudes[i] = a;
        }
        info!("amplitude {:?}", &amplitudes[..128]);
    }

    loop {
        Timer::after(Duration::from_millis(50)).await;
        info!("ok");
    }

    loop {
        let val = pio0.sm0.rx().wait_pull().await;

        info!("val {:?}", val);
        Timer::after(Duration::from_millis(100)).await;
    }

    // 125_00 0_000 / 50000 = 2500hz
    // 2500 / 100 = 2.5 Hz
    //   2^27 = 134217728
    // 125000000 / 134217728 = 0.929 Hz, 1.07 seconds
    // pio0.sm0.tx().push(0xff0000);
}
