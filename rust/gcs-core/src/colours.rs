//! **A colour for each part** (#676, #677): what the workspace shades an object in and what the
//! STEP export writes it in, one rule for both.
//!
//! A part is what a closed instance's solid is one evaluation of (`model::Binding`, #566), so
//! every placement of one part is one colour, as the instances of one part are in a CAD
//! assembly; an object no closed instance made is a part of its own.  Its colour is the
//! drawing's where the drawing beside the model names one (`fill`, `drawing::fills`), and
//! otherwise the palette's: indexed by a digest of what the part is — its definition and its
//! solid's path in the body, not the numbers it was given (`program::part_of`) — so a part keeps
//! its colour across a reload and a numeric edit, and moved on to the next colour no earlier
//! part has taken where one has, so that parts differ while there are colours to go round.
//!
//! Presentation chosen here and nowhere else: the page asks for the colours and the export
//! writes them, so the two never disagree.

use crate::model::Sketch;
use std::collections::BTreeMap;

/// The palette: muted, so a part reads on the workspace's light ground and its edges' dark ink
/// reads on it, and far enough apart that neighbours are told apart.  sRGB.
pub const PALETTE: [[u8; 3]; 12] = [
    [0x7a, 0x9c, 0xc6], // steel blue
    [0xd4, 0xb4, 0x83], // sand
    [0x8f, 0xb0, 0x8a], // sage
    [0xc9, 0x8b, 0x6b], // terracotta
    [0xa5, 0x97, 0xc4], // lavender
    [0x6f, 0xa8, 0xa3], // teal
    [0xe0, 0xa8, 0x7a], // apricot
    [0x9a, 0x9f, 0xab], // slate
    [0xc9, 0x8f, 0xa3], // rose
    [0xa8, 0xa8, 0x6a], // olive
    [0x8c, 0xbf, 0xdc], // sky
    [0xb7, 0xc9, 0x8a], // lime
];

/// An object and its colour, and whether a drawing gave it (else the palette did).
#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    pub solid: usize,
    pub colour: [u8; 3],
    pub drawn: bool,
}

/// Which part solid `i` is: its binding's key where a closed instance made it (one evaluation
/// placed), else its name.
pub fn part(sk: &Sketch, i: usize) -> String {
    match &sk.solids[i].binding {
        Some(b) => b.key.clone(),
        None => format!("\u{1d}{}", sk.solids[i].name),
    }
}

/// What the palette is indexed by: the part less its numbers (`program::part_of`).
fn steady(sk: &Sketch, i: usize) -> String {
    match &sk.solids[i].binding {
        Some(b) => crate::program::part_of(&b.key),
        None => crate::vocab::public_path(&sk.solids[i].name),
    }
}

/// FNV-1a, 64 bits: a digest that is the same on every platform and in every build.
fn digest(s: &str) -> u64 {
    s.bytes().fold(0xcbf2_9ce4_8422_2325, |h, b| (h ^ b as u64).wrapping_mul(0x0100_0000_01b3))
}

/// **Every object's palette colour**, by solid: each part in the order its first object comes,
/// at its digest's colour, or the next one round no earlier part has taken.  Every object, the
/// hidden ones too, so hiding one moves no other's colour.
pub fn palette(sk: &Sketch) -> BTreeMap<usize, [u8; 3]> {
    let mut taken = [false; PALETTE.len()];
    let mut of: BTreeMap<String, [u8; 3]> = BTreeMap::new();
    let mut out = BTreeMap::new();
    for i in crate::overview::objects(sk) {
        let key = part(sk, i);
        let colour = *of.entry(key).or_insert_with(|| {
            let n = PALETTE.len();
            if taken.iter().all(|&t| t) { taken = [false; PALETTE.len()]; }
            let first = (digest(&steady(sk, i)) % n as u64) as usize;
            let k = (0..n).map(|j| (first + j) % n).find(|&k| !taken[k]).unwrap_or(first);
            taken[k] = true;
            PALETTE[k]
        });
        out.insert(i, colour);
    }
    out
}

/// **Every object's colour**: the drawing's where `drawn` (`drawing::fills`) has one, else the
/// palette's — in the objects' order.
pub fn parts(sk: &Sketch, drawn: &BTreeMap<usize, [u8; 3]>) -> Vec<Part> {
    palette(sk).into_iter().map(|(solid, colour)| match drawn.get(&solid) {
        Some(&c) => Part { solid, colour: c, drawn: true },
        None => Part { solid, colour, drawn: false },
    }).collect()
}

/// `#rrggbb`.
pub fn hex(c: [u8; 3]) -> String {
    format!("#{:02x}{:02x}{:02x}", c[0], c[1], c[2])
}

/// `#rgb` or `#rrggbb`, as a drawing writes a colour; `None` for anything else (an alpha
/// included: a part is opaque, and STEP's colour has none).
pub fn parse(s: &str) -> Option<[u8; 3]> {
    let h = s.strip_prefix('#')?;
    if !h.bytes().all(|b| b.is_ascii_hexdigit()) { return None; }
    let byte = |t: &str| u8::from_str_radix(t, 16).ok();
    match h.len() {
        3 => Some([0, 1, 2].map(|k| byte(&h[k..k + 1]).unwrap() * 17)),
        6 => Some([0, 1, 2].map(|k| byte(&h[2 * k..2 * k + 2]).unwrap())),
        _ => None,
    }
}

/// A colour as STEP's `COLOUR_RGB` takes it: each component in 0..=1.
pub fn unit(c: [u8; 3]) -> [f64; 3] {
    c.map(|x| x as f64 / 255.0)
}
