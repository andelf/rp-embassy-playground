#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{cmp::Ordering, sync::atomic::AtomicU32};

use defmt::*;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_futures::select::{select4, Either4, Select4};
use embassy_rp::{
    gpio::{self, AnyPin, Input, Pin, Pull},
    spi::{self, Spi},
};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_5X8, MonoTextStyleBuilder},
    pixelcolor::Gray4,
    prelude::*,
    text::{Alignment, Text},
    Drawable,
};
use gpio::{Level, Output};
use rp::ssd1327;
use {defmt_rtt as _, panic_probe as _};

static mut pos: [i32; 2] = [0; 2];

#[embassy_executor::task(pool_size = 1)]
async fn jogball_task(up: AnyPin, right: AnyPin, down: AnyPin, left: AnyPin) {
    let mut up = Input::new(up, Pull::None);
    let mut right = Input::new(right, Pull::None);
    let mut down = Input::new(down, Pull::None);
    let mut left = Input::new(left, Pull::None);

    loop {
        match select4(
            up.wait_for_any_edge(),
            right.wait_for_any_edge(),
            down.wait_for_any_edge(),
            left.wait_for_any_edge(),
        )
        .await
        {
            Either4::First(_) => unsafe { pos[1] += 1 },
            Either4::Second(_) => unsafe { pos[0] += 1 },
            Either4::Third(_) => unsafe { pos[1] -= 1 },
            Either4::Fourth(_) => unsafe { pos[0] -= 1 },
        }
    }
}

const UP: usize = 0;
const RIGHT: usize = 1;
const DOWN: usize = 2;
const LEFT: usize = 3;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut btn = Input::new(p.PIN_5, gpio::Pull::None);
    let mut bkled = Output::new(p.PIN_4, Level::Low); // active low

    info!("io setting");

    spawner
        .spawn(jogball_task(
            p.PIN_6.degrade(),
            p.PIN_7.degrade(),
            p.PIN_8.degrade(),
            p.PIN_9.degrade(),
        ))
        .unwrap();

    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let dc = p.PIN_20;
    let rst = p.PIN_21;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc, Level::Low);
    let rst = Output::new(rst, Level::High);

    let di = SPIInterface::new(spi, dc, cs);

    let mut display = ssd1327::Display::new(di);
    display.init();

    info!("init ok");

    let mut state = [0; 2];

    for i in 1..16 {
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_5X8)
            .text_color(Gray4::new(i))
            .build();
        Text::with_alignment("Hello World", Point::new(5, (i as i32) * 8), style, Alignment::Left)
            .draw(&mut display)
            .unwrap();
    }

    display.flush();

    info!("flush ok");

    loop {
        let n = unsafe { pos };
        if n != state {
            state = n;
            info!("position {:?}", n);

            // display.clear(Gray4::new(1));
            let style = MonoTextStyleBuilder::new()
                .font(&FONT_5X8)
                .text_color(Gray4::new(8))
                .background_color(Gray4::BLACK)
                .build();
            if btn.is_high() {
                Text::with_alignment(".", Point::new(n[0], n[1]), style, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();
            } else {
                Text::with_alignment("+", Point::new(n[0], n[1]), style, Alignment::Left)
                    .draw(&mut display)
                    .unwrap();
            }
            display.flush();
        }

        //btn.wait_for_any_edge().await;
        //btn.wait_for_any_edge().await;
        Timer::after(Duration::from_millis(1)).await;
        //bkled.toggle();

        //        println!("toggle!");
    }
}
