//! **What a body is made of, stated of the drawing** (#510): `mass(10kg) arm`,
//! `volume(42 * 1cm^3) web` — a measure of a beam, a member's line or a solid, which answers to no
//! load and so is no scenario's.  Stated, it is a row of the drawing (`CKind::Mass`,
//! `CKind::Volume`) the solve holds; claimed (`claim mass(<= 1kg) plank`), it is judged on the
//! solved drawing.  **Where it is made, too** (#953): a face's `area(A) F`, its second moment
//! about a line `moment(I, about: l) F`, and the point a body's volume or a face's area is
//! centred on, `g centroid B`, or a body's mass, `g centerofmass B`.  Pure data, set by
//! elaboration and carried by the Solvent source alone.

use super::scenario::Applied;
use crate::expr::Cmp;
use crate::units::Dim;

/// **Which measure**: a mass, its density over its volume, or the volume itself; a face's area or
/// its second moment about a line (#953); the centroid of a body's volume or a face's area, or a
/// body's centre of mass (#953), each a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    Mass,
    Volume,
    Area,
    Moment,
    Centroid,
    CenterOfMass,
}

impl Quantity {
    /// Every measure, in the order the words are listed.
    pub const ALL: [Quantity; 6] = [Quantity::Mass, Quantity::Volume, Quantity::Area,
                                     Quantity::Moment, Quantity::Centroid, Quantity::CenterOfMass];

    pub fn word(self) -> &'static str {
        match self {
            Quantity::Mass => "mass",
            Quantity::Volume => "volume",
            Quantity::Area => "area",
            Quantity::Moment => "moment",
            Quantity::Centroid => "centroid",
            Quantity::CenterOfMass => "centerofmass",
        }
    }

    /// The word with its article, as a sentence says it: `a mass`, `an area`.
    pub fn a(self) -> String {
        super::article(self.word())
    }

    /// The word's quantity, or `None` for any other word.
    pub fn parse(w: &str) -> Option<Quantity> {
        Quantity::ALL.into_iter().find(|q| q.word() == w)
    }

    /// Whether it is a point a declared point is put at (`g centroid B`), written between them,
    /// rather than a number written in its parentheses (`area(A) F`).
    pub fn point(self) -> bool {
        matches!(self, Quantity::Centroid | Quantity::CenterOfMass)
    }

    /// Whether it weighs by density: a mass and a centre of mass.
    pub fn weighed(self) -> bool {
        matches!(self, Quantity::Mass | Quantity::CenterOfMass)
    }

    /// Whether it is a face's (an area, a second moment of one) rather than a body's.
    pub fn of_face(self) -> bool {
        matches!(self, Quantity::Area | Quantity::Moment)
    }

    /// What its number is: a mass in kilograms, a volume in the drawing's length cubed, an area
    /// in its square and a second moment in its fourth power; a point's, how far it stands from
    /// where the measure puts it, a length.
    pub fn dim(self) -> Dim {
        let l = Dim::LENGTH;
        match self {
            Quantity::Mass => Dim::MASS,
            Quantity::Volume => l.mul(l).mul(l),
            Quantity::Area => l.mul(l),
            Quantity::Moment => l.mul(l).mul(l).mul(l),
            Quantity::Centroid | Quantity::CenterOfMass => l,
        }
    }
}

/// **A measure the drawing states**: of what (a beam, a member's line, a solid, a face of one),
/// how it compares with the number (`==` stated or claimed, `<=` and `>=` claimed alone), and
/// whether it is a claim, judged after the solve, rather than a row the solve holds.  A point's
/// (`g centroid B`) names the point (`at`) and, stated, is three rows, one an ordinate of the
/// world (`k`); its number is how far the point stands from where the measure puts it, nothing.
/// A second moment names the line it is about (`about`).  A stated number may be affine in one
/// unknown the drawing declares (#953, `free`: its name and coefficient, `value` the constant), so
/// the row determines it — `mass(M) boat` sizes nothing and says what the boat weighs.
#[derive(Clone, Debug, PartialEq)]
pub struct Measured {
    pub quantity: Quantity,
    pub of: Applied,
    pub compare: Cmp,
    pub value: f64,
    pub claim: bool,
    pub at: Option<usize>,
    pub about: Option<super::EntRef>,
    pub k: usize,
    pub free: Option<(String, f64)>,
    /// The bodies a point's measure is of where it names an instance (#953): every solid under it
    /// that is no operand of another, each once — `of` the first.  Empty: `of` alone.
    pub bodies: Vec<usize>,
}

impl Measured {
    /// The solids it is of: its bodies, or the solid `of` is or a face of.
    pub fn solids(&self) -> Vec<usize> {
        match self.bodies.is_empty() {
            true => self.of.solid().into_iter().collect(),
            false => self.bodies.clone(),
        }
    }

    /// What it states the measure is where the drawing stands: its number, and where it reads an
    /// unknown that unknown's part — `None` where the unknown is bound to no column.
    pub fn stated(&self, sk: &super::Sketch) -> Option<f64> {
        match &self.free {
            None => Some(self.value),
            Some((name, m)) => {
                let q = *sk.free_vars.get(name)?;
                Some(self.value + m * sk.params[q as usize].value)
            }
        }
    }
}
