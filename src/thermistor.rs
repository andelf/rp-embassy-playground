/// 100k ParCan thermistor (104GT-2)
/// ATC Semitec 104GT-2 (Used in ParCan)
pub const TEMPTABLE_5: [[u16; 2]; 32] = [
    [1, 713],
    [17, 300], // top rating 300C
    [20, 290],
    [23, 280],
    [27, 270],
    [31, 260],
    [37, 250],
    [43, 240],
    [51, 230],
    [61, 220],
    [73, 210],
    [87, 200],
    [106, 190],
    [128, 180],
    [155, 170],
    [189, 160],
    [230, 150],
    [278, 140],
    [336, 130],
    [402, 120],
    [476, 110],
    [554, 100],
    [635, 90],
    [713, 80],
    [784, 70],
    [846, 60],
    [897, 50],
    [937, 40],
    [966, 30],
    [986, 20],
    [1000, 10],
    [1010, 0],
];

pub fn analog_to_temp(raw: u16) -> f32 {
    // use bisect to find the temperature
    let mut min = 0;
    let mut max = TEMPTABLE_5.len() - 1;
    let mut mid = 0;
    while max - min > 1 {
        mid = (max + min) / 2;
        if TEMPTABLE_5[mid][0] > raw {
            max = mid;
        } else {
            min = mid;
        }
    }
    let min_temp = TEMPTABLE_5[min][1] as f32;
    let max_temp = TEMPTABLE_5[max][1] as f32;
    let min_raw = TEMPTABLE_5[min][0] as f32;
    let max_raw = TEMPTABLE_5[max][0] as f32;
    
    min_temp + (max_temp - min_temp) * (raw as f32 - min_raw) / (max_raw - min_raw)
}
