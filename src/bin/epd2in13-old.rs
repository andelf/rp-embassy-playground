//! SSD1306 OLED

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
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config};
use embassy_rp::interrupt;
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Timer};
// use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use display_interface_spi::SPIInterface;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X9};
use embedded_graphics::mono_font::iso_8859_2::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::{Alignment, LineHeight, TextStyleBuilder};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
    Drawable,
};
use epd::display::{DisplaySize104x201, DisplaySize122x250, DisplaySize400x300};
use epd::drivers::{PervasiveDisplays, IL3895, SSD1608, SSD1619A, UC8176};
use epd::interface::{DisplayInterface, EPDInterfaceNoCS};
use epd::{EPDInterface, FastUpdateEPD, EPD};
use heapless::String;
use rp::font::FONT_MUZAI_PIXEL;

const BLACK: BinaryColor = BinaryColor::Off;
const WHITE: BinaryColor = BinaryColor::On;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut delay = Delay;

    // let irq = interrupt::take!(ADC_IRQ_FIFO);
    // let mut adc = Adc::new(p.ADC, irq, adc::Config::default());

    let busy = p.PIN_16; // not used
    let csn = p.PIN_17;
    let clk = p.PIN_18;
    let mosi = p.PIN_19;
    let dc = p.PIN_20;
    let rst = p.PIN_21;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let spi = Spi::new_blocking_txonly(p.SPI0, clk, mosi, config);

    // Configure CS
    let cs = Output::new(csn, Level::Low);
    let dc = Output::new(dc, Level::Low);
    let rst = Output::new(rst, Level::Low);
    let busy = Input::new(busy, Pull::None);

    let di = EPDInterfaceNoCS::new(spi, dc, rst, busy);

    // let mut display: FastUpdateEPD<_, DisplaySize122x250, SSD1619A> = FastUpdateEPD::new(di);
    let mut display: EPD<_, DisplaySize400x300, UC8176> = EPD::new(di);
    // let mut display: EPD<_, DisplaySize122x250, IL3895> = EPD::new(di);
    //let mut display: FastUpdateEPD<_, DisplaySize122x250, IL3895> = FastUpdateEPD::new(di);
    // let mut display: EPD<_, DisplaySize122x250, SSD1608> = EPD::new(di);
    //    let mut display: EPD<_, DisplaySize104x201, IL3895> = EPD::new(di);

    display.set_rotation(90);
    display.init(&mut delay);
    display.display_frame();

    defmt::info!("disp ok");

    let mut buf = String::<256>::new();

    let mut loop_cnt = 0;

    let mut values = [0.0f32; 100];
    let mut cursor = 0;

    let text = r#"
观自在菩萨，行深般若波罗蜜多时，照见五蕴皆空，度一切苦厄。舍利
子，色不异空，空不异色，色即是空，空即是色，受想行识，亦复如是。
舍利子，是诸法空相，不生不灭，不垢不净，不增不减。是故空中无色，
无受想行识，无眼耳鼻舌身意，无色声香味触法，无眼界，乃至无意识
界，无无明，亦无无明尽，乃至无老死，亦无老死尽。无苦集灭道，无
智亦无得，以无所得故。菩提萨埵，依般若波罗蜜多故，心无挂碍，无
挂碍故，无有恐怖，远离颠倒梦想，究竟涅槃。三世诸佛，依般若波罗
蜜多故，得阿耨多罗三藐三菩提。故知般若波罗蜜多是大神咒，是大明
咒，是无上咒，是无等等咒，能除一切苦，真实不虚。故说般若波罗蜜
多咒，即说咒曰：揭谛揭谛，波罗揭谛，波罗僧揭谛，菩提娑婆诃。"#;

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_MUZAI_PIXEL)
        .text_color(BLACK)
        .background_color(WHITE)
        .build();
    let _style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BLACK).build();
    Text::with_text_style(
        text,
        Point::new(0, -1), //skip 1 line
        style,
        TextStyleBuilder::new().line_height(LineHeight::Pixels(12)).build(),
    )
    .draw(&mut display)
    .unwrap();

    display.display_frame();

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(Duration::from_millis(4000)).await;

        info!("led off!");
        led.set_low();
        Timer::after(Duration::from_millis(4000)).await;
    }
}
