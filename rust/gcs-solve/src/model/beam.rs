//! **A beam** (#510 S0): lines of the drawing made members of a frame, by a section and a
//! material.
//!
//! `arm := beam(span, A: b * h, Iy: h * b^3 / 12, Iz: b * h^3 / 12, J: 0.229 * h * b^3) of steel`.
//! Each line is an Euler–Bernoulli member, rigid at its ends, and the drawing is the mesh:
//! members join where their ends meet, and nothing is meshed.  A section is held in the
//! **document's units** (an area in its length squared), as every length of the drawing is; the
//! material it is made of is held in SI (`Material`), and an analysis reads it back into the
//! document's units (`Units::from_si`).  An analysis of a scenario reads beams (`frame.rs`);
//! nothing in the solve does.

use crate::units::Dim;
use std::collections::BTreeMap;

/// A number a beam's section states.  The section's axes are the beam's own: x along it, z the
/// normal of the plane it is drawn in, y square to both in that plane — so `Iz` is its stiffness
/// bending in its plane and `Iy` across it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SectionProp {
    /// Its area, a length squared.
    A,
    /// Its second moment of area about its y: bending across its plane.
    Iy,
    /// Its second moment of area about its z: bending in its plane.
    Iz,
    /// Its torsion constant, a length to the fourth.
    J,
    /// Its elastic section modulus about its y, `Iy` over the farthest fibre from it: what a
    /// factor of safety reads.
    Sy,
    /// Its elastic section modulus about its z.
    Sz,
}

impl SectionProp {
    pub const ALL: [SectionProp; 6] = [
        SectionProp::A,
        SectionProp::Iy,
        SectionProp::Iz,
        SectionProp::J,
        SectionProp::Sy,
        SectionProp::Sz,
    ];

    /// The four every beam states: what its stiffness is made of.
    pub const STIFFNESS: [SectionProp; 4] =
        [SectionProp::A, SectionProp::Iy, SectionProp::Iz, SectionProp::J];

    /// The label a `beam(…)` gives it.
    pub fn name(self) -> &'static str {
        match self {
            SectionProp::A => "A",
            SectionProp::Iy => "Iy",
            SectionProp::Iz => "Iz",
            SectionProp::J => "J",
            SectionProp::Sy => "Sy",
            SectionProp::Sz => "Sz",
        }
    }

    /// The number `s` names, or the complaint that it names none.
    pub fn named(s: &str) -> Result<SectionProp, String> {
        SectionProp::ALL.into_iter().find(|p| p.name() == s).ok_or_else(|| {
            let ns: Vec<String> =
                SectionProp::ALL.iter().map(|p| format!("`{}`", p.name())).collect();
            format!("`{s}` is no number of a beam's section: {}", ns.join(", "))
        })
    }

    /// Its dimension: a power of length.
    pub fn dim(self) -> Dim {
        let l2 = Dim::LENGTH.mul(Dim::LENGTH);
        match self {
            SectionProp::A => l2,
            SectionProp::Iy | SectionProp::Iz | SectionProp::J => l2.mul(l2),
            SectionProp::Sy | SectionProp::Sz => l2.mul(Dim::LENGTH),
        }
    }
}

/// **A beam's section**: the numbers stated of it, in the document's units, each positive.  The
/// four of `SectionProp::STIFFNESS` are always stated.
#[derive(Clone, Debug, PartialEq)]
pub struct Section {
    pub props: BTreeMap<SectionProp, f64>,
}

impl Section {
    /// A number stated of it.
    pub fn get(&self, p: SectionProp) -> Option<f64> {
        self.props.get(&p).copied()
    }

    /// Its stiffness's four numbers, in `interior::SECTION`'s order: `[A, Iy, Iz, J]`.
    pub fn stiffness(&self) -> [f64; 4] {
        SectionProp::STIFFNESS.map(|p| self.get(p).unwrap_or(f64::NAN))
    }
}

/// **A beam**: the lines it is drawn on (by index), its section, and the material it is made of
/// (by name, a key of `Sketch::materials`) — `None` where it names none, or one that was refused.
#[derive(Clone, Debug, PartialEq)]
pub struct Beam {
    /// What the document calls it, absolute: `arm`, or `p.arm` inside an instance.
    pub name: String,
    pub lines: Vec<usize>,
    pub section: Section,
    pub material: Option<String>,
}
