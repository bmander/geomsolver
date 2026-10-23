//! **What a body is made of, stated of the drawing** (#510): `mass(10kg) arm`,
//! `volume(42 * 1cm^3) web` — a measure of a beam, a member's line or a solid, which answers to no
//! load and so is no scenario's.  Stated, it is a row of the drawing (`CKind::Mass`,
//! `CKind::Volume`) the solve holds; claimed (`claim mass(<= 1kg) plank`), it is judged on the
//! solved drawing.  Pure data, set by elaboration and carried by the Solvent source alone.

use super::scenario::Applied;
use crate::expr::Cmp;
use crate::units::Dim;

/// **Which measure**: a mass, its density over its volume, or the volume itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    Mass,
    Volume,
}

impl Quantity {
    pub fn word(self) -> &'static str {
        match self {
            Quantity::Mass => "mass",
            Quantity::Volume => "volume",
        }
    }

    /// The word's quantity: `mass`, `volume`, or `None` for any other word.
    pub fn parse(w: &str) -> Option<Quantity> {
        [Quantity::Mass, Quantity::Volume].into_iter().find(|q| q.word() == w)
    }

    /// What its number is: a mass in kilograms, a volume in the drawing's length cubed.
    pub fn dim(self) -> Dim {
        match self {
            Quantity::Mass => Dim::MASS,
            Quantity::Volume => Dim::LENGTH.mul(Dim::LENGTH).mul(Dim::LENGTH),
        }
    }
}

/// **A mass or a volume the drawing states**: of what (a beam, a member's line, a solid), how it
/// compares with the number (`==` stated or claimed, `<=` and `>=` claimed alone), and whether it
/// is a claim, judged after the solve, rather than a row the solve holds.
#[derive(Clone, Debug, PartialEq)]
pub struct Measured {
    pub quantity: Quantity,
    pub of: Applied,
    pub compare: Cmp,
    pub value: f64,
    pub claim: bool,
}
