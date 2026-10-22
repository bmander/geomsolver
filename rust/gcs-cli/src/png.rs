//! **A PNG of RGB pixels**, written by hand: the CLI has no dependency to write one with, and the
//! format needs very little.  One `IDAT` holding a zlib stream of one deflate block of fixed
//! Huffman codes, its matches found greedily through a hash of the next three bytes.  A rendered
//! part is mostly runs of one colour, which that takes to a few per cent of its size; nothing
//! here tries harder than that.

/// The PNG of `rgb`, three bytes a pixel, row by row from the top: `width` by `height`.
pub fn encode(width: usize, height: usize, rgb: &[u8]) -> Vec<u8> {
    assert_eq!(rgb.len(), width * height * 3, "a PNG's pixels are its width by its height");
    // each scanline led by its filter, 0: the bytes as they are
    let mut raw = Vec::with_capacity(height * (3 * width + 1));
    for row in rgb.chunks_exact(3 * width) {
        raw.push(0);
        raw.extend_from_slice(row);
    }
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(height as u32).to_be_bytes());
    // eight bits a sample, truecolour, deflate, adaptive filtering, not interlaced
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &zlib(&raw));
    chunk(&mut out, b"IEND", &[]);
    out
}

/// A chunk: its length, its type, its data and the CRC of the type and the data.
fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let from = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[from..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

/// The CRC-32 PNG checks each chunk by (ISO 3309, the reflected 0xEDB88320), a byte at a time.
const CRC: [u32; 256] = {
    let mut table = [0; 256];
    let mut n = 0;
    while n < 256 {
        let mut c = n as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 == 1 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        table[n] = c;
        n += 1;
    }
    table
};

fn crc32(bytes: &[u8]) -> u32 {
    !bytes.iter().fold(!0u32, |c, &b| CRC[((c ^ b as u32) & 0xff) as usize] ^ (c >> 8))
}

/// `bytes` as a zlib stream: a header, the deflate block, and the Adler-32 of what it holds.
fn zlib(bytes: &[u8]) -> Vec<u8> {
    // deflate, a 32K window; no dictionary, and a check that makes the pair a multiple of 31
    let mut w = Bits { out: vec![0x78, 0x01], acc: 0, n: 0 };
    deflate(&mut w, bytes);
    let mut out = w.finish();
    let (mut a, mut b) = (1u32, 0u32);
    for &x in bytes {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    out.extend_from_slice(&((b << 16) | a).to_be_bytes());
    out
}

/// Bits into bytes, the least significant first (RFC 1951 §3.1.1).
struct Bits {
    out: Vec<u8>,
    acc: u64,
    n: u32,
}

impl Bits {
    /// `len` bits of `v`, its least significant first: a header field or extra bits.
    fn put(&mut self, v: u32, len: u32) {
        self.acc |= (v as u64) << self.n;
        self.n += len;
        while self.n >= 8 {
            self.out.push(self.acc as u8);
            self.acc >>= 8;
            self.n -= 8;
        }
    }

    /// A Huffman code of `len` bits, which deflate packs its most significant bit first.
    fn code(&mut self, code: u32, len: u32) {
        self.put(code.reverse_bits() >> (32 - len), len);
    }

    /// A literal byte or a length's symbol, in the fixed code (RFC 1951 §3.2.6).
    fn symbol(&mut self, s: u32) {
        match s {
            0..=143 => self.code(0x30 + s, 8),
            144..=255 => self.code(0x190 + s - 144, 9),
            256..=279 => self.code(s - 256, 7),
            _ => self.code(0xc0 + s - 280, 8),
        }
    }

    fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            self.out.push(self.acc as u8);
        }
        self.out
    }
}

/// The least length each length symbol (257…285) stands for, and how many extra bits follow it.
const LENGTH: [(u32, u32); 29] = [
    (3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (10, 0), (11, 1), (13, 1), (15, 1),
    (17, 1), (19, 2), (23, 2), (27, 2), (31, 2), (35, 3), (43, 3), (51, 3), (59, 3), (67, 4),
    (83, 4), (99, 4), (115, 4), (131, 5), (163, 5), (195, 5), (227, 5), (258, 0),
];

/// The least distance each distance code (0…29) stands for, and its extra bits.
const DISTANCE: [(u32, u32); 30] = [
    (1, 0), (2, 0), (3, 0), (4, 0), (5, 1), (7, 1), (9, 2), (13, 2), (17, 3), (25, 3), (33, 4),
    (49, 4), (65, 5), (97, 5), (129, 6), (193, 6), (257, 7), (385, 7), (513, 8), (769, 8),
    (1025, 9), (1537, 9), (2049, 10), (3073, 10), (4097, 11), (6145, 11), (8193, 12),
    (12289, 12), (16385, 13), (24577, 13),
];

/// How far back a match may reach, and how long one may be.
const WINDOW: usize = 32768;
const LONGEST: usize = 258;

/// One final block of fixed codes: each place either the longest match its three bytes' hash
/// last saw, or a literal.
fn deflate(w: &mut Bits, bytes: &[u8]) {
    w.put(1, 1);
    w.put(1, 2);
    const HASH: usize = 1 << 15;
    let hash = |i: usize| {
        let v = (bytes[i] as u32) << 16 | (bytes[i + 1] as u32) << 8 | bytes[i + 2] as u32;
        (v.wrapping_mul(0x9e37_79b1) >> 17) as usize & (HASH - 1)
    };
    let mut last = vec![usize::MAX; HASH];
    let mut i = 0;
    while i < bytes.len() {
        let mut len = 0;
        if i + 3 <= bytes.len() {
            let h = hash(i);
            let j = last[h];
            last[h] = i;
            if j != usize::MAX && i - j <= WINDOW {
                let most = LONGEST.min(bytes.len() - i);
                while len < most && bytes[j + len] == bytes[i + len] {
                    len += 1;
                }
                if len >= 3 {
                    let s = LENGTH.iter().rposition(|&(base, _)| base as usize <= len).unwrap();
                    w.symbol(257 + s as u32);
                    w.put((len - LENGTH[s].0 as usize) as u32, LENGTH[s].1);
                    let d = i - j;
                    let k = DISTANCE.iter().rposition(|&(base, _)| base as usize <= d).unwrap();
                    w.code(k as u32, 5);
                    w.put((d - DISTANCE[k].0 as usize) as u32, DISTANCE[k].1);
                    // what the match covered is seen too, so the next run finds it
                    for p in i + 1..(i + len).min(bytes.len().saturating_sub(2)) {
                        last[hash(p)] = p;
                    }
                    i += len;
                    continue;
                }
            }
        }
        w.symbol(bytes[i] as u32);
        i += 1;
    }
    w.symbol(256);
}
