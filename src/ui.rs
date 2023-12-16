use embedded_graphics::{
    prelude::{DrawTarget, PixelColor, Point},
    primitives::{Line, Primitive, PrimitiveStyle, Rectangle},
    Drawable,
};

pub struct LineChart<C>
where
    C: PixelColor,
{
    color: C,
    bbox: Rectangle,
    values: [f32; 100],
    cursor: usize,
}

impl<C> LineChart<C>
where
    C: PixelColor,
{
    pub fn new(color: C, bbox: Rectangle) -> Self {
        Self {
            color,
            bbox,
            values: [0.0f32; 100],
            cursor: 0,
        }
    }

    pub fn push(&mut self, val: f32) {
        self.values[self.cursor] = val;
        self.cursor += 1;
        if self.cursor >= self.values.len() {
            self.cursor = 0;
        }
    }
}

impl<C> Drawable for LineChart<C>
where
    C: PixelColor,
{
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        let max = self.values.iter().fold(0.0f32, |acc, &v| acc.max(v));
        let min = self.values.iter().fold(100.0f32, |acc, &v| acc.min(v));

        let range = max - min;
        if range > 0.05 {
            let mut last = None;
            // only draw if range is big enough
            for (idx, v) in self.values.iter().cycle().skip(self.cursor).take(self.values.len()).enumerate() {
                let y = self.bbox.bottom_right().unwrap().y - ((v - min) / range * 100.0) as i32;
                let x = self.bbox.top_left.x + 4 * idx as i32;
                let p = Point::new(x, y);
                // Pixel(p, BLACK).draw(&mut display).unwrap();
                /*Circle::new(p, 5)
                .into_styled(PrimitiveStyle::with_fill(BLACK))
                .draw(&mut display)
                .unwrap();*/
                match last {
                    Some(last) => {
                        let _ = Line::new(last, p)
                            .into_styled(PrimitiveStyle::with_stroke(self.color, 1))
                            .draw(target);
                    }
                    None => {}
                }
                last = Some(p);
            }
        }
        Ok(())
    }
}
