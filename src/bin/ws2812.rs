//! WS2812 PIO dirver

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

    let mut pio0 = Pio::new(p.PIO0);

    let mosi = p.PIN_19; // SI
    let clk = p.PIN_18; // SCLK
    let cs = p.PIN_20; // CS
    let rst = p.PIN_17;

    let ws2812 = p.PIN_23;

    /*
    // prepare the PIO program
    let side_set = pio::SideSet::new(false, 1, false);
    let mut a: pio::Assembler<32> = pio::Assembler::new_with_side_set(side_set);

    const T1: u8 = 2; // start bit
    const T2: u8 = 5; // data bit
    const T3: u8 = 3; // stop bit
    const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut do_zero = a.label();
    a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
    a.bind(&mut wrap_target);
    // Do stop bit
    a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
    // Do start bit
    a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
    // Do data bit = 1
    a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
    a.bind(&mut do_zero);
    // Do data bit = 0
    a.nop_with_delay_and_side_set(T2 - 1, 0);
    a.bind(&mut wrap_source);

    let prg = a.assemble_with_wrap(wrap_source, wrap_target);

                .wrap_target
                pull block
                out null, 8
    */

    let prg = pio_proc::pio_asm!(
        r#"
        .side_set 1

        .wrap_target


        out y, 8            side 0

        irq 0               side 0

        bitloop:
            out x 1          side 0 [6]; low for 7 cycles, stalls for 1

            jmp !x do_zero   side 1 [3]; if x is 0, jump to do_zero

        do_one:
            jmp bitloop      side 1 [4]

        do_zero:

            nop              side 0 [4]


        .wrap


        "#,
    );

    let mut cfg = embassy_rp::pio::Config::default();

    // pin config
    let led_pin = pio0.common.make_pio_pin(p.PIN_25);
    let mosi_pin = pio0.common.make_pio_pin(mosi);
    let clk_pin = pio0.common.make_pio_pin(clk);
    let cs_pin = pio0.common.make_pio_pin(cs);

    let neopixel_pin = pio0.common.make_pio_pin(ws2812);

    pio0.sm0.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&neopixel_pin]);

    cfg.set_out_pins(&[&led_pin]);
    cfg.set_set_pins(&[&led_pin]);

    let relocated = RelocatedProgram::new(&prg.program);
    //cfg.use_program(&pio0.common.load_program(&relocated), &[&clk_pin, &cs_pin]); // side set pins
    cfg.use_program(&pio0.common.load_program(&relocated), &[&neopixel_pin]);

    // no div, full speed
    //cfg.clock_divider = 9_u16.into();
    // target freq = 800hz
    cfg.clock_divider = 9_u16.into();

    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        direction: ShiftDirection::Left,
        threshold: 32,
    };
    cfg.fifo_join = FifoJoin::TxOnly;

    pio0.sm0.set_config(&cfg);
    pio0.sm0.set_enable(true);

    // 125_00 0_000 / 50000 = 2500hz
    // 2500 / 100 = 2.5 Hz
    //   2^27 = 134217728
    // 125000000 / 134217728 = 0.929 Hz, 1.07 seconds
    // pio0.sm0.tx().push(0xff0000);

    /** Wiring
     *
     */

    //    let mut config = spi::Config::default();
    //  config.frequency = 2_000_000;
    ///let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);
    //  let most = Output::new(mosi.degrade(), Level::Low);
    // let clk = Output::new(clk.degrade(), Level::Low);
    // let cs = Output::new(cs.degrade(), Level::Low);
    // let mut di = Soft9BitSpi::new(most, clk, cs);
    // let rst = Output::new(rst, Level::High);

    //  let mut di = SPI9bit::new(spi, cs);
    for i in 0..0xffffff {
        pio0.sm0.tx().push(i);
    }

    let mut d = 1;
    loop {
        for i in 1..=14 {
            while pio0.sm0.tx().full() {}
            pio0.sm0.tx().push(d * i);
        }
        pio0.irq0.wait().await;

        d += 1;

        Timer::after(Duration::from_millis(10)).await;
    }
}
