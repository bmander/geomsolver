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
/// four of `SectionProp::STIFFNESS` are always stated.  A number read over unknowns of the drawing
/// (#510 S1: `Iz: b * h^3 / 12`, `h` declared) is in `over` as well, and `props` holds what it
/// came to at the unknowns' seeds; what it is is read at the drawing (`Section::read`).
#[derive(Clone, Debug, PartialEq)]
pub struct Section {
    pub props: BTreeMap<SectionProp, f64>,
    pub over: BTreeMap<SectionProp, Over>,
}

/// **A section's number read over unknowns** (#510 S1): its text as the flattener wrote it, every
/// number of its scope written in, and the declared unknowns it reads, by absolute name.  A
/// requirement driving the frame finds them; the text is read as a tape over them (`tape.rs`), so
/// it may be any function of them (`b * h^3 / 12`), not only one affine in one.
#[derive(Clone, Debug, PartialEq)]
pub struct Over {
    pub text: String,
    pub unknowns: Vec<String>,
}

impl Over {
    /// Its tape over `vars` (which hold every unknown it reads), in the document's units.
    pub fn tape(&self, vars: &[String], units: crate::units::Units)
        -> Result<crate::tape::Tape, String>
    {
        let p = crate::expr::parse_in(&self.text, units)?;
        crate::tape::Tape::compile(&p.body, vars)
    }
}

impl Section {
    /// A section of numbers it states, none read over an unknown.
    pub fn stated(props: BTreeMap<SectionProp, f64>) -> Section {
        Section { props, over: BTreeMap::new() }
    }

    /// A number stated of it, as it came to at elaboration (at the unknowns' seeds).
    pub fn get(&self, p: SectionProp) -> Option<f64> {
        self.props.get(&p).copied()
    }

    /// **A number of it at the drawing**: as stated, or worked out at the values the unknowns it
    /// reads stand at — one no row binds yet at its seed (a driving row's frame is read before
    /// the expression pass binds what it reads), `Err` where it has none.
    pub fn read(&self, p: SectionProp, sk: &super::Sketch) -> Option<Result<f64, String>> {
        let Some(over) = self.over.get(&p) else { return self.get(p).map(Ok) };
        Some((|| {
            let mut x = Vec::with_capacity(over.unknowns.len());
            for name in &over.unknowns {
                let bound = sk.free_vars.get(name).map(|&q| &sk.params[q as usize])
                    .filter(|q| !q.fixed).map(|q| q.value);
                let seed = sk.declared.get(name).and_then(|d| d.seed);
                x.push(bound.or(seed).ok_or_else(|| {
                    format!("`{name}` is an unknown with no seed: `hint(…)` it")
                })?);
            }
            let tape = over.tape(&over.unknowns, sk.units)?;
            let v = tape.eval(&x, &mut crate::tape::Scratch::new()).v;
            match v > 0.0 && v.is_finite() {
                true => Ok(v),
                false => Err(format!("`{}` is {v} where the drawing stands, and a section's \
                                      numbers are positive", p.name())),
            }
        })())
    }

    /// Its stiffness's four numbers at the drawing, in `interior::SECTION`'s order: `[A, Iy, Iz,
    /// J]`, or why one cannot be read.
    pub fn stiffness(&self, sk: &super::Sketch) -> Result<[f64; 4], String> {
        let mut out = [0.0; 4];
        for (o, p) in out.iter_mut().zip(SectionProp::STIFFNESS) {
            *o = self.read(p, sk).unwrap_or(Ok(f64::NAN))?;
        }
        Ok(out)
    }

    /// Every unknown it reads, each once, in the order first read.
    pub fn unknowns(&self) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for n in self.over.values().flat_map(|o| &o.unknowns) {
            if !out.contains(n) {
                out.push(n.clone());
            }
        }
        out
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
