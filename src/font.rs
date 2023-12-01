use embedded_graphics::{
    image::ImageRaw,
    mono_font::{self, MonoFont},
    prelude::Size,
};

const CHARS: &str = " 。一三上不世中为乃了五亦以佛你依倒僧净减凡切利卧即厄受可味咒在垢埵增声复多大天好如娑婆子安实尽己平度异得心怕怖恐想意慰所挂提揭故无时明是晴智曰最有梦槃槽死法波涅深灭照瓣生界皆相真眼知碌碍神离究空竟等罗老耨耳能自至舌舍般色若苦菩萨蕴藐虚蜜行见观触诃识说诸谛贵身还远道阿除集雾霾颠香鼻！，：";

fn cn_pixel_mapping(c: char) -> usize {
    CHARS.chars().enumerate().find(|(_, c2)| *c2 == c).map(|(i, _)| i + 1).unwrap_or(0)
}

pub const FONT_MUZAI_PIXEL: MonoFont = MonoFont {
    image: ImageRaw::new(include_bytes!("../MuzaiPixel.raw"), 1152),
    glyph_mapping: &cn_pixel_mapping,
    character_size: Size::new(8, 12),
    character_spacing: 0, // in pixels
    baseline: 11,
    underline: mono_font::DecorationDimensions::new(12 + 2, 1),
    strikethrough: mono_font::DecorationDimensions::new(12 / 2, 1),
};

fn wqy_pixel_mapping(c: char) -> usize {
    const WQY_CHARS: &str = " “”℃、。〇一七三上下不世东中乃九二五亦京以佛你依倒僧八六净凌减分切利刷北十午南即厄受右味咒四在垢埵增声复多大好如婆子实尽局展州左年广度异得心快怖恐想意所提揭故新无日时明是晚晨智曰月有梦槃死法波海涅深灭照生界皆相真眼知碍示神离秒究空竟等米经罗罣老耨耳能自至舌舍般色若苦菩萨蕴藐虚蜜行西见观触诃识诗说诸谛身远速道部阿除集零颠香鼻！，：；？";
    WQY_CHARS.chars().enumerate().find(|(_, c2)| *c2 == c).map(|(i, _)| i).unwrap_or(0)
}
pub const FONT_WQY16: MonoFont = MonoFont {
    image: ImageRaw::new(include_bytes!("../wqy-16.raw"), 2688),
    glyph_mapping: &wqy_pixel_mapping,
    character_size: Size::new(16, 16),
    character_spacing: 0, // in pixels
    baseline: 16,
    underline: mono_font::DecorationDimensions::new(16 + 2, 1),
    strikethrough: mono_font::DecorationDimensions::new(16 / 2, 1),
};

fn digits_mapping(c: char) -> usize {
    "-.0123456789:ABCDEF"
        .chars()
        .enumerate()
        .find(|(_, c2)| *c2 == c)
        .map(|(i, _)| i + 1)
        .unwrap_or(0)
}

pub const FONT_SEG7_30X48: MonoFont = MonoFont {
    image: ImageRaw::new(include_bytes!("../seg7-digits-30x48.raw"), 800),
    glyph_mapping: &digits_mapping,
    character_size: Size::new(30, 48),
    character_spacing: 1, // in pixels
    baseline: 11,
    underline: mono_font::DecorationDimensions::new(12 + 2, 1),
    strikethrough: mono_font::DecorationDimensions::new(12 / 2, 1),
};

pub const FONT_SEG7_16X24: MonoFont = MonoFont {
    image: ImageRaw::new(include_bytes!("../Dseg7-16x24.raw"), 336),
    glyph_mapping: &digits_mapping,
    character_size: Size::new(16, 24),
    character_spacing: 0, // in pixels
    baseline: 22,
    underline: mono_font::DecorationDimensions::new(12 + 2, 1),
    strikethrough: mono_font::DecorationDimensions::new(12 / 2, 1),
};
