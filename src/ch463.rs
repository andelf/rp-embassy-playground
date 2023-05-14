//! Blackberry Q10 Keyboard via CH463

const BS: u8 = 0x08; // backspace
const CR: u8 = 0x0d; // Carriage Return
const SP: u8 = 0x20; // Space

const SYM: u8 = 129;
const RSHIFT: u8 = 130;
const LSHIFT: u8 = 131;
const ALT: u8 = 132;
const MIC: u8 = 133;
const SPK: u8 = 134;
const NUL: u8 = 0;

const SCAN_TABLE: [[u8; 5]; 7] = [
    [b'Q', b'E', b'R', b'U', b'O'],
    [b'W', b'S', b'G', b'H', b'L'],
    [SYM, b'D', b'T', b'Y', b'I'],
    [b'A', b'P', RSHIFT, CR, BS],
    [ALT, b'X', b'V', b'B', b'$'],
    [SP, b'Z', b'C', b'N', b'M'],
    [MIC, LSHIFT, b'F', b'J', b'K'],
];

const SCAN_TABLE_ALT: [[u8; 5]; 7] = [
    [b'#', b'2', b'3', b'_', b'+'],
    [b'1', b'4', b'/', b':', b'"'],
    [NUL, b'5', b'(', b')', b'-'],
    [b'*', b'@', NUL, NUL, NUL],
    [NUL, b'8', b'?', b'!', SPK],
    [NUL, b'7', b'9', b',', b'.'],
    [b'0', NUL, b'6', b';', b'\''],
];

// NOTE: multiple keys is not supported
pub fn parse_keys(buf: &[u8]) -> char {
    let mut key_row: usize = 0;
    let mut key_col: usize = 0;
    let mut out: u8 = 0;
    let mut alt = false;
    let mut shift = false;
    for &raw in buf {
        if raw == 0 {
            break;
        }
        let row = (raw & 0x0f) as usize;
        let col = (raw >> 4) as usize;
        let key = SCAN_TABLE[row - 1][col - 1];
        //        let key = if key == NUL {
        //          SCAN_TABLE_ALT[row as usize][col as usize]
        //    } else {
        //      key
        // };
        if key == ALT {
            alt = true;
        } else if key == LSHIFT || key == RSHIFT {
            shift = true;
        } else {
            out = key;
            key_row = row;
            key_col = col;
        }
    }
    if alt && key_row != 0 && key_col != 0 {
        out = SCAN_TABLE_ALT[key_row - 1][key_col - 1];
    }
    if !shift {
        out = out.to_ascii_lowercase();
    }
    if out == SPK {
        '🔈'
    } else if out == MIC {
        '🎤'
    } else {
        out as char
    }
}
