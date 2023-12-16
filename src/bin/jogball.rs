#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{cmp::Ordering, sync::atomic::AtomicU32};

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select4, Either4, Select4};
use embassy_rp::gpio::{self, AnyPin, Input, Pin, Pull};
use embassy_time::{Duration, Timer};
use gpio::{Level, Output};
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
    let mut bkled = Output::new(p.PIN_4, Level::High);

    info!("io setting");

    spawner
        .spawn(jogball_task(p.PIN_8.degrade(), p.PIN_7.degrade(), p.PIN_6.degrade(), p.PIN_9.degrade()))
        .unwrap();

    let mut state = [0; 2];

    loop {
        let n = unsafe { pos };
        if n != state {
            state = n;
            info!("position {:?}", n);
        }

        //btn.wait_for_any_edge().await;
        //btn.wait_for_any_edge().await;
        Timer::after(Duration::from_millis(0)).await;
        //bkled.toggle();

        //        println!("toggle!");
    }
}
