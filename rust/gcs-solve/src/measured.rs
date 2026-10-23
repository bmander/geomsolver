//! **A mass or a volume the drawing states, read** (#510, `model::Measured`): `mass(10kg) arm`,
//! `claim volume(<= 42 * 1cm^3) web` — what a beam, a member's line or a solid is made of, read
//! on the solved drawing as its driving row reads it, with no support, no load and no solve: a
//! frame's members exactly (`beams::measure`), a prism its face through its depth
//! (`prisms::measure`), any other solid over its own tetrahedra (`bodies::measure`).  Beside each
//! value, the error estimated in it, the change from the same reading twice as coarse, as every
//! output of a scenario has (`beams::Read::estimated`).

use crate::beams::ROUNDING;
use crate::interior::Reads;
use crate::model::{Applied, Measured, Quantity, Sketch};

/// **`m` read on the drawing as it stands**, `coarser` times a reading's size, or why it has
/// none: a member of no length, a material of no density, a thing that is no beam, member or
/// solid.
pub fn read_at(sk: &Sketch, m: &Measured, coarser: f64) -> Result<f64, String> {
    let reads = Reads::of(m.quantity);
    if !crate::beams::members(sk, &m.of).is_empty() {
        return crate::beams::measure(sk, &m.of, reads);
    }
    match m.of.solid() {
        Some(solid) => crate::bodies::measure(sk, solid, reads, coarser),
        None => Err(format!("a {} is of a beam, a line a beam is drawn on, or a solid, and {}",
                            m.quantity.word(), what(sk, &m.of))),
    }
}

/// **`m` read, and the error estimated beside it**: the change from the reading twice as coarse
/// — nothing where the two agree to rounding (a frame's, a straight prism's), none where the
/// coarser has no value.
pub fn read(sk: &Sketch, m: &Measured) -> Result<(f64, Option<f64>), String> {
    let v = read_at(sk, m, 1.0)?;
    let error = read_at(sk, m, 2.0).ok().map(|c| {
        let e = (v - c).abs();
        if e <= ROUNDING * v.abs().max(c.abs()) { 0.0 } else { e }
    });
    Ok((v, error))
}

/// Whether value `v` keeps `m`: within 1e-9 of what it states for `==`, on its side for `<=` and
/// `>=` (`beams::keeps`'s rule).
pub fn keeps(m: &Measured, v: f64) -> bool {
    crate::beams::compares(m.compare, m.value, v)
}

/// **A measure as a reader reads it**, six figures in its unit: a mass in kilograms, a volume in
/// the drawing's length cubed (`42000mm³` — never `42000mm^3`, which a document reads as the cube
/// of 42000mm).
pub fn value_text(sk: &Sketch, q: Quantity, v: f64) -> String {
    figures_text(sk, q, v, 6)
}

/// The error estimated beside a measure, to two figures: `± 0.0021kg`.
pub fn error_text(sk: &Sketch, q: Quantity, e: f64) -> String {
    format!("± {}", figures_text(sk, q, e, 2))
}

fn figures_text(sk: &Sketch, q: Quantity, v: f64, figures: usize) -> String {
    let fig = crate::json::fmt_g(v, figures);
    match q {
        Quantity::Mass => format!("{fig}kg"),
        Quantity::Volume => format!("{fig}{}³", sk.units.name()),
    }
}

/// What a measure is of, said where it names nothing it reads.
fn what(sk: &Sketch, of: &Applied) -> String {
    match of {
        Applied::Ent(e) => format!("this is {}", e.kind.a()),
        Applied::Face(_) => "this is a face".to_string(),
        Applied::Beam(b) => format!("`{}` is a beam", sk.beams[*b].name),
    }
}
