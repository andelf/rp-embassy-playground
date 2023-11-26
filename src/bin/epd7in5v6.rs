//! 局部刷新, BW

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_must_use)]
#![feature(generic_const_exprs)]

use core::fmt::Write;

use defmt::*;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_rp::interrupt;
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{self, Blocking, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9};
use embedded_graphics::mono_font::iso_8859_2::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::{Alignment, TextStyleBuilder};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use embedded_graphics_core::pixelcolor::raw::LittleEndian;
use embedded_hal::blocking::delay::DelayMs;
use epd::display::{DisplaySize400x300, DisplaySize800x480, FrameBuffer};
use epd::drivers::{PervasiveDisplays, SSD1619A, UC8176};
use epd::interface::{DisplayInterface, EPDInterfaceNoCS};
use epd::{EPDInterface, FastUpdateEPD, EPD};
use heapless::String;
use rp::font::{FONT_MUZAI_PIXEL, FONT_SEG7_30X48, FONT_WQY16};

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

struct EPD7in5v2<'a> {
    spi: Spi<'a, SPI0, Blocking>,
    dc: Output<'a, AnyPin>,
    busy: Input<'a, AnyPin>,
}

impl EPD7in5v2<'_> {
    fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        self.spi.blocking_write(&[cmd]);
    }

    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.blocking_write(data);
    }

    fn send_command_data(&mut self, cmd: u8, data: &[u8]) {
        self.send_command(cmd);
        self.send_data(data);
    }

    fn busy_wait(&mut self) {
        loop {
            self.send_command(0x71);
            // negative logic
            if self.busy.is_high() {
                break;
            }
        }
    }

    fn init(&mut self) {
        // Power setting
        // VDHR=3.0V
        // VDH=15.0V
        // VDL=-15.0V
        self.send_command_data(0x01, &[0x07, 0x17, 0x3f, 0x3f, 0b000011]);

        // power on
        self.send_command(0x04);
        self.busy_wait();

        // panel setting
        // KW-3f   KWR-2F BWROTP 0f BWOTP 1f
        // self.send_command_data(0x00, &[0x0F]);
        self.send_command_data(0x00, &[0x3F]);

        // tres
        // 800x480
        self.send_command_data(0x61, &[0x03, 0x20, 0x01, 0xE0]);

        // NO need to dual SPI

        // VCOM and data interval setting(CDI)
        // Border LUT selection(BDV) = 0b01 -> LUTWK
        // 0b00 -> LUTKK
        // enable N2OCP
        self.send_command_data(0x50, &[0x01 | 0b1000, 0x07]);

        // Tcon setting
        // self.send_command_data(0x60, &[0x22]);
    }

    // LUT.

    /*
    - VCOM LUT, LUTC, 0x20
    - W2W LUT, LUTWW, 0x21. KWR 模式不适用
    - K2W LUT,  LUTKW, LUTR, 0x22.
    - W2K LUT,  LUTWK, LUTW, 0x23.
    - K2K LUT, LUTKK, LUTK, 0x24
    - BORDER LUT, LUTBD, 0x25
     */
    /// Do INIT, clear
    fn configure_init_update(&mut self) {
        #[rustfmt::skip]
        const LUT_VCOM: [u8; 60] = [
            0b00_00_00_00, 0x0f, 0x0f, 0x0f, 0x0f, 0x01,
            0b00_00_00_00, 0x1A, 0x00, 0x10, 0x20, 0x01,
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

    fn configure_partial_update(&mut self) {
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
            0x00,    0x0f, 0x00, 0x00, 0x00,    0x01,
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
            0x00,    0x0f, 0x00, 0x00, 0x00,    0x01,
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

    // Clear is required to set initial state of the panel.
    // Or else the panel will show random noise.
    fn clear(&mut self) {
        self.send_command(0x10);
        for i in 0..48000 {
            self.send_data(&[0xff]);
        }
        self.send_command(0x13);
        for i in 0..48000 {
            self.send_data(&[0xff]);
        }
    }

    fn display_frame(&mut self, buf: &[u8]) {
        // write to NEW buf
        self.send_command(0x13);
        self.send_data(buf);
    }

    fn refresh(&mut self) {
        let mut delay = Delay;
        self.send_command(0x12);
        delay.delay_ms(100_u32); //must
        self.busy_wait();
    }

    fn sleep(&mut self) {
        self.send_command(0x02);
        self.busy_wait();
    }

    fn set_partial_refresh(&mut self, rect: Rectangle) {
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

    fn unset_partial_refresh(&mut self) {
        const PARTIAL_OUT: u8 = 0x92;
        self.send_command(PARTIAL_OUT);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut delay = Delay;

    let busy = p.PIN_16; // not used
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let csn = p.PIN_17;
    let dc = p.PIN_20;
    let rst = p.PIN_21;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 20_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc.degrade(), Level::Low);
    let mut rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy.degrade(), Pull::None);

    let mut epd = EPD7in5v2 { spi, dc, busy };

    // init
    rst.set_low();
    delay.delay_ms(100_u32); // at least 10ms
    rst.set_high();
    delay.delay_ms(100_u32); // at least 10ms

    epd.init();
    epd.configure_init_update();

    epd.clear();
    epd.refresh();
    info!("Init ok");
    epd.configure_partial_update();

    info!("clear ok");

    // init OK

    /*
    Rectangle::new(Point::new(0, 10), Size::new(200, 400))
        .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(&mut fb)
        .unwrap();

    epd.display_frame_black(fb.as_bytes());

    fb.fill(BinaryColor::On); // red clean

    Rectangle::new(Point::new(300, 10), Size::new(200, 400))
        .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(&mut fb)
        .unwrap();

    epd.display_frame_red(fb.as_bytes());

    epd.refresh();

    info!("write ok");

    info!("led");
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        led.toggle();
    }
    */

    let mut fb = Framebuffer::<
        BinaryColor,
        _,
        LittleEndian,
        800,
        480,
        { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(800, 480) },
    >::new();

    fb.clear(BinaryColor::On);

    Text::with_alignment(
        "2023-11-27 00:08:00",
        Point::new(100, 380),                                   //skip 1 line
        MonoTextStyle::new(&FONT_SEG7_30X48, BinaryColor::Off), // black
        Alignment::Left,
    )
    .draw(&mut fb)
    .unwrap();

    Text::with_alignment(
        "快速刷新 你好世界",
        Point::new(200, 280),                              //skip 1 line
        MonoTextStyle::new(&FONT_WQY16, BinaryColor::Off), // black
        Alignment::Left,
    )
    .draw(&mut fb)
    .unwrap();

    epd.display_frame(fb.data());
    epd.refresh();

    Timer::after(Duration::from_millis(2000)).await;

    info!("write ok");
    Text::with_alignment(
        "局部刷新一二三四五六七八九十〇",
        Point::new(100, 100),
        MonoTextStyle::new(&FONT_WQY16, BinaryColor::Off), // black
        Alignment::Left,
    )
    .draw(&mut fb)
    .unwrap();
    epd.display_frame(fb.data());

    epd.refresh();

    Timer::after(Duration::from_millis(2000)).await;


    epd.set_partial_refresh(Rectangle::new(Point::new(400, 240), Size::new(200, 80)));
    let mut fbx = Framebuffer::<
        BinaryColor,
        _,
        LittleEndian,
        200,
        80,
        { embedded_graphics::framebuffer::buffer_size::<BinaryColor>(200, 80) },
    >::new();

    fbx.clear(BinaryColor::On);

    Text::with_alignment(
        "般若波罗蜜多心经",
        Point::new(10, 20),
        MonoTextStyle::new(&FONT_WQY16, BinaryColor::Off), // black
        Alignment::Left,
    )
    .draw(&mut fbx)
    .unwrap();
    epd.display_frame(fbx.data());
    epd.refresh();

    info!("write ok");
    Timer::after(Duration::from_millis(1000)).await;

    let article = &[
        "观自在菩萨",
        "行深般若波罗蜜多时",
        "照见五蕴皆空",
        "度一切苦厄",
        "舍利子",
        "色不异空",
        "空不异色",
        "色即是空",
        "空即是色",
        "受想行识",
        "亦复如是",
        "舍利子",
        "是诸法空相",
        "不生不灭",
        "不垢不净",
        "不增不减",
        "是故空中无色",
        "无受想行识",
        "无眼耳鼻舌身意",
        "无色声香味触法",
        "无眼界",
        "乃至无意识界",
        "无无明",
        "亦无无明尽",
        "乃至无老死",
        "亦无老死尽",
        "无苦集灭道",
        "无智亦无得",
        "以无所得故",
        "菩提萨埵",
        "依般若波罗蜜多故",
        "心无罣碍",
        "无罣碍故",
        "无有恐怖",
        "远离颠倒梦想",
        "究竟涅槃",
        "三世诸佛",
        "依般若波罗蜜多故",
        "得阿耨多罗三藐三菩提",
        "故知般若波罗蜜多",
        "是大神咒",
        "是大明咒",
        "是无上咒",
        "是无等等咒",
        "能除一切苦",
        "真实不虚",
        "故说般若波罗蜜多咒",
        "即说咒曰",
        "揭谛 揭谛 波罗揭谛",
        "波罗僧揭谛",
        "菩提萨婆诃",
    ];
    loop {
        for i in article {
            fbx.clear(BinaryColor::On);
            Text::with_alignment(
                i,
                Point::new(10, 20),
                MonoTextStyle::new(&FONT_WQY16, BinaryColor::Off), // black
                Alignment::Left,
            )
            .draw(&mut fbx)
            .unwrap();
            epd.display_frame(fbx.data());
            epd.refresh();
            // Timer::after(Duration::from_millis(50 * (i.len() as u64))).await;
        }
    }

    loop {
        Timer::after(Duration::from_millis(100)).await;
        led.toggle();
        info!("led toggle");
    }

    // epd.sleep();

    let mut buf = heapless::String::<128>::new();
    let mut n = 0;

    // y must be div of 8

    info!("write okxxx");

    Timer::after(Duration::from_millis(1000)).await;

    let mut s = heapless::String::<128>::new();

    let mut n = 0;
    loop {
        Timer::after(Duration::from_millis(100)).await;
        led.toggle();
        info!("led toggle");
    }
}
