#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem::{size_of, size_of_val};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{AnyPin, Level, Output, Pin},
    i2c::{self, Config},
};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    framebuffer::Framebuffer,
    primitives::{Line, Primitive, PrimitiveStyle},
};
use embedded_graphics_core::{
    pixelcolor::{raw::LittleEndian, BinaryColor},
    prelude::Point,
    Drawable,
};
use embedded_hal_1::delay::DelayUs;
use {defmt_rtt as _, panic_probe as _};

pub struct NT7538<'d> {
    db0: Output<'d, AnyPin>,
    db1: Output<'d, AnyPin>,
    db2: Output<'d, AnyPin>,
    db3: Output<'d, AnyPin>,
    db4: Output<'d, AnyPin>,
    db5: Output<'d, AnyPin>,
    db6: Output<'d, AnyPin>,
    db7: Output<'d, AnyPin>,
    // RD signal is active low
    rd: Output<'d, AnyPin>,
    // WR signal is active low
    wr: Output<'d, AnyPin>,
    rs: Output<'d, AnyPin>, // ?
    // RES signal is active low
    res: Output<'d, AnyPin>,
    cs1: Output<'d, AnyPin>,
}

impl<'d> NT7538<'d> {
    pub fn new(
        db0: Output<'d, AnyPin>,
        db1: Output<'d, AnyPin>,
        db2: Output<'d, AnyPin>,
        db3: Output<'d, AnyPin>,
        db4: Output<'d, AnyPin>,
        db5: Output<'d, AnyPin>,
        db6: Output<'d, AnyPin>,
        db7: Output<'d, AnyPin>,
        rd: Output<'d, AnyPin>,
        wr: Output<'d, AnyPin>,
        rs: Output<'d, AnyPin>,
        res: Output<'d, AnyPin>,
        cs1: Output<'d, AnyPin>,
    ) -> Self {
        Self {
            db0,
            db1,
            db2,
            db3,
            db4,
            db5,
            db6,
            db7,
            rd,
            wr,
            rs,
            res,
            cs1,
        }
    }

    fn write_cmd(&mut self, data: u8) {
        // self.res.set_high();
        //self.cs1.set_low();
        self.rs.set_level(Level::Low);
        Delay.delay_us(1_u32);

        self.cs1.set_low();
        //  self.rd.set_level(Level::High);

        self.wr.set_level(Level::High);

        self.db0.set_level(Level::from(data & 0b0000_0001 != 0));
        self.db1.set_level(Level::from(data & 0b0000_0010 != 0));
        self.db2.set_level(Level::from(data & 0b0000_0100 != 0));
        self.db3.set_level(Level::from(data & 0b0000_1000 != 0));
        self.db4.set_level(Level::from(data & 0b0001_0000 != 0));
        self.db5.set_level(Level::from(data & 0b0010_0000 != 0));
        self.db6.set_level(Level::from(data & 0b0100_0000 != 0));
        self.db7.set_level(Level::from(data & 0b1000_0000 != 0));

        Delay.delay_us(10_u32);

        self.wr.set_level(Level::Low);
        Delay.delay_us(10_u32);

        self.cs1.set_high();
    }

    fn write_all_data(&mut self, data: &[u8]) {
        //self.wr.set_level(Level::High);
        self.rs.set_level(Level::High);
        Delay.delay_us(1_u32);

        self.cs1.set_low();

        //        self.rd.set_level(Level::High);

        self.wr.set_level(Level::High);

        self.db0.set_level(Level::from(data[0] & 0b0000_0001 != 0));
        self.db1.set_level(Level::from(data[0] & 0b0000_0010 != 0));
        self.db2.set_level(Level::from(data[0] & 0b0000_0100 != 0));
        self.db3.set_level(Level::from(data[0] & 0b0000_1000 != 0));
        self.db4.set_level(Level::from(data[0] & 0b0001_0000 != 0));
        self.db5.set_level(Level::from(data[0] & 0b0010_0000 != 0));
        self.db6.set_level(Level::from(data[0] & 0b0100_0000 != 0));
        self.db7.set_level(Level::from(data[0] & 0b1000_0000 != 0));

        Delay.delay_us(10_u32);

        self.wr.set_level(Level::Low);

        Delay.delay_us(10_u32);

        self.wr.set_level(Level::High);

        self.db0.set_level(Level::from(data[1] & 0b0000_0001 != 0));
        self.db1.set_level(Level::from(data[1] & 0b0000_0010 != 0));
        self.db2.set_level(Level::from(data[1] & 0b0000_0100 != 0));
        self.db3.set_level(Level::from(data[1] & 0b0000_1000 != 0));
        self.db4.set_level(Level::from(data[1] & 0b0001_0000 != 0));
        self.db5.set_level(Level::from(data[1] & 0b0010_0000 != 0));
        self.db6.set_level(Level::from(data[1] & 0b0100_0000 != 0));
        self.db7.set_level(Level::from(data[1] & 0b1000_0000 != 0));

        Delay.delay_us(10_u32);

        self.wr.set_level(Level::Low);

        Delay.delay_us(10_u32);

        self.cs1.set_high();
    }

    fn write_data(&mut self, data: u8) {
        //self.wr.set_level(Level::High);
        self.rs.set_level(Level::High);
        Delay.delay_us(1_u32);

        self.cs1.set_low();

        //        self.rd.set_level(Level::High);
        Delay.delay_us(10_u32);

        self.wr.set_level(Level::Low);

        self.db0.set_level(Level::from(data & 0b0000_0001 != 0));
        self.db1.set_level(Level::from(data & 0b0000_0010 != 0));
        self.db2.set_level(Level::from(data & 0b0000_0100 != 0));
        self.db3.set_level(Level::from(data & 0b0000_1000 != 0));
        self.db4.set_level(Level::from(data & 0b0001_0000 != 0));
        self.db5.set_level(Level::from(data & 0b0010_0000 != 0));
        self.db6.set_level(Level::from(data & 0b0100_0000 != 0));
        self.db7.set_level(Level::from(data & 0b1000_0000 != 0));

        Delay.delay_us(10_u32);

        self.wr.set_level(Level::High);

        Delay.delay_us(10_u32);

        self.cs1.set_high();
    }

    pub fn turn_on_display(&mut self) {
        self.write_cmd(0xAF);
    }

    pub fn turn_off_display(&mut self) {
        self.write_cmd(0xAE);
    }

    pub fn set_start_line(&mut self, line: u8) {
        self.write_cmd(0x40 | (line & 0x3F));
    }

    pub fn set_page(&mut self, page: u8) {
        self.write_cmd(0xB0 | (page & 0x07));
    }

    pub fn set_column(&mut self, column: u8) {
        self.write_cmd(0x10 | (column >> 4));
        self.write_cmd(column & 0x0F);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut delay = Delay;
    let mut led = Output::new(p.PIN_25, Level::Low);

    let db0 = p.PIN_0;
    let db1 = p.PIN_1;
    let db2 = p.PIN_2;
    let db3 = p.PIN_3;
    let db4 = p.PIN_4;
    let db5 = p.PIN_5;
    let db6 = p.PIN_6;
    let db7 = p.PIN_7;

    let rd = p.PIN_14;
    let wr = p.PIN_13;
    let rs = p.PIN_12;
    let res = p.PIN_11;
    let cs1 = p.PIN_10;

    let mut nt7538 = NT7538::new(
        Output::new(db0.degrade(), Level::Low),
        Output::new(db1.degrade(), Level::Low),
        Output::new(db2.degrade(), Level::Low),
        Output::new(db3.degrade(), Level::Low),
        Output::new(db4.degrade(), Level::Low),
        Output::new(db5.degrade(), Level::Low),
        Output::new(db6.degrade(), Level::Low),
        Output::new(db7.degrade(), Level::Low),
        Output::new(rd.degrade(), Level::High),
        Output::new(wr.degrade(), Level::High),
        Output::new(rs.degrade(), Level::High),
        Output::new(res.degrade(), Level::Low),
        Output::new(cs1.degrade(), Level::High),
    );

    //    nt7538.cs1.set_high();
    nt7538.res.set_low();
    Timer::after(Duration::from_millis(20)).await;
    nt7538.res.set_high();
    Timer::after(Duration::from_millis(20)).await;

    nt7538.write_cmd(0xE2); // System Reset

    Timer::after(Duration::from_millis(1)).await;

    nt7538.write_cmd(0xAE); // Display OFF

    nt7538.write_cmd(0xA0); // ADC normal
    nt7538.write_cmd(0xA7); // non-inverted display
    nt7538.write_cmd(0xA4); // normal display mode
    nt7538.write_cmd(0x82); // norma;l display

    nt7538.write_cmd(0xA2); // Sets LCD driving voltage bias ratio = 1/9 bias

    nt7538.write_cmd(0xE4); // Oscillation Frequency Select

    nt7538.write_cmd(0xC4); // Output Status Select Register, scan direction of the COM output pad

    // nt7538.write_cmd(0xE6); // DC/DC
    // nt7538.write_cmd(0b1111); // DC/DC

    // booster must
    // nt7538.write_cmd(0x2C); // Power Controller:Booster ON
    // Timer::after(Duration::from_millis(1)).await;
    nt7538.write_cmd(0b00101_110); // Power Controller:Voltage Regulator ON
    Timer::after(Duration::from_millis(10)).await;
    //nt7538.write_cmd(0x2F); // Power Controller:Voltage Follower ON
    //Timer::after(Duration::from_millis(1)).await;

    // V0 Voltage Regulator Internal Resistor Ratio Set
    nt7538.write_cmd(0x24);

    // contrast
    nt7538.write_cmd(0x81); // The Electronic Volume
    nt7538.write_cmd(0xff);

    //nt7538.write_cmd(0xAC);
    //nt7538.write_cmd(0b01);

    nt7538.set_start_line(0);

    nt7538.write_cmd(0xAF); // Display ON

    //  nt7538.turn_on_display();

    let mut fb = Framebuffer::<
        BinaryColor,
        _,
        LittleEndian,
        64,
        130,
        { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(64, 130) },
    >::new();

    info!("size {}", size_of_val(&fb));

    Line::new(Point::new(0, 0), Point::new(64, 129))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut fb)
        .unwrap();

    let mut i = 0x00;
    loop {
        for p in 0..9 {
            nt7538.set_page(p);
            nt7538.set_column(0);
            for c in 0..130 {
                nt7538.write_data(i); // fb.data()[8 * c as usize + p as usize]);
            }
        }

        i = !i;
        led.toggle();
        Timer::after(Duration::from_millis(1000)).await;
        info!("toggle {}", i);
    }
}
