//! **A load scenario** (#510 P4): the supports a body is held by, the loads it carries, and the
//! requirements judged under them — `scenario lift { fixed arm.base … }`, elaborated.
//!
//! Pure data, and nothing here is solved: what a requirement's output *is* (a deflection, a
//! natural frequency) is an analysis the scenario only names.  A drawing states as many
//! scenarios as it has load cases, and each is read over the one solved drawing — a scenario adds
//! no geometry and no row of its own, so the drawing's solve is the same whichever is asked about.
//! Every number is in the units the arithmetic is done in (`units.rs`): a length in the
//! document's unit, a force in kg·(document length)/s², a pressure in kg/((document length)·s²).

use super::*;
use crate::expr::Cmp;
use crate::units::Dim;

/// What a support or a load is applied to, as the existing selectors name it: an entity (a point,
/// a line, a solid's whole body), or the faces a path past a solid names (`arm.base`, sets.md's
/// faces as sets).  Whether a face path names faces is the analysis's to judge, where it reads the
/// solid's boundary after the solve, as a region reading a face does.
///
/// Or a beam (#510 S0), by its index in `Sketch::beams`: what its members are, for a body load
/// or an output of the frame they are part of.
#[derive(Clone, Debug, PartialEq)]
pub enum Applied {
    Ent(EntRef),
    Face(FaceRef),
    Beam(usize),
}

impl Applied {
    /// The entity it is of: itself, or the solid a face is a face of — `None` for a beam, which
    /// is no entity.
    pub fn entity(&self) -> Option<EntRef> {
        match self {
            Applied::Ent(e) => Some(*e),
            Applied::Face(f) => Some(EntRef::solid(f.solid as usize)),
            Applied::Beam(_) => None,
        }
    }

    /// The solid it is of, where it is of one.
    pub fn solid(&self) -> Option<usize> {
        self.entity().filter(|e| e.kind == EntKind::Solid).map(|e| e.i())
    }
}

/// How a support holds what it is applied to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hold {
    /// `fixed X`: neither moves nor turns.
    Fixed,
    /// `pinned X`: does not move, and is free to turn.
    Pinned,
}

impl Hold {
    const ALL: [Hold; 2] = [Hold::Fixed, Hold::Pinned];

    /// The word a scenario writes it with.
    pub fn word(self) -> &'static str {
        match self {
            Hold::Fixed => "fixed",
            Hold::Pinned => "pinned",
        }
    }

    pub fn parse(w: &str) -> Option<Hold> {
        Hold::ALL.into_iter().find(|h| h.word() == w)
    }
}

/// `fixed arm.base`, `pinned a along: up` — a support.  `along` restricts what is held to the
/// motion along one direction (a roller is `pinned b along: up`); `None` holds every direction.
#[derive(Clone, Debug, PartialEq)]
pub struct Support {
    pub hold: Hold,
    pub at: Applied,
    pub along: Option<EntRef>,
}

/// What a load is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadKind {
    /// `load X (fx, fy, fz)` or `load X f along: d`: a force on what it is applied to.
    Force,
    /// `pressure F p`: a pressure on a face, pushing into the material.
    Pressure,
    /// `accelerate B (ax, ay, az)`: a body load, the body's density times the acceleration.
    Acceleration,
}

impl LoadKind {
    const ALL: [LoadKind; 3] = [LoadKind::Force, LoadKind::Pressure, LoadKind::Acceleration];

    /// The word a scenario writes it with.
    pub fn word(self) -> &'static str {
        match self {
            LoadKind::Force => "load",
            LoadKind::Pressure => "pressure",
            LoadKind::Acceleration => "accelerate",
        }
    }

    pub fn parse(w: &str) -> Option<LoadKind> {
        LoadKind::ALL.into_iter().find(|k| k.word() == w)
    }

    /// What its number is: a force, a pressure or an acceleration.
    pub fn dim(self) -> Dim {
        let accel = Dim::LENGTH.div(Dim::TIME).div(Dim::TIME);
        match self {
            LoadKind::Force => accel.mul(Dim::MASS),
            LoadKind::Pressure => accel.mul(Dim::MASS).div(Dim::LENGTH).div(Dim::LENGTH),
            LoadKind::Acceleration => accel,
        }
    }

    /// Whether it is a vector (a force, an acceleration) rather than a magnitude (a pressure).
    pub fn directed(self) -> bool {
        self != LoadKind::Pressure
    }
}

/// A load's number: a vector in the world's axes, a magnitude along a direction the drawing
/// names, or the one number a pressure is.
#[derive(Clone, Debug, PartialEq)]
pub enum LoadValue {
    Vector([f64; 3]),
    Along { magnitude: f64, along: EntRef },
    Scalar(f64),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Load {
    pub kind: LoadKind,
    pub at: Applied,
    pub value: LoadValue,
}

/// **What an analysis reports**, the words a requirement is written over.  Each is a number of
/// one dimension, read of what it names under the scenario's supports and loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Output {
    /// `deflection(p)`, `deflection(p, along: d)`: how far a point (or a face, at its worst)
    /// moves — a length.
    Deflection,
    /// `safety(b)`: the body's least factor of safety against its material's failure — a plain
    /// number.
    Safety,
    /// `mode(b, n)`: the body's `n`th natural frequency, counted from 1.
    Mode,
    /// `buckling(b)`: the factor on the scenario's loads at which the body first buckles.
    Buckling,
    /// `compliance(b)`: the work the loads do on the body, an energy.
    Compliance,
    /// `mass(b)`: the body's mass, its density over its volume.
    Mass,
}

impl Output {
    pub const ALL: [Output; 6] = [
        Output::Deflection,
        Output::Safety,
        Output::Mode,
        Output::Buckling,
        Output::Compliance,
        Output::Mass,
    ];

    pub fn word(self) -> &'static str {
        match self {
            Output::Deflection => "deflection",
            Output::Safety => "safety",
            Output::Mode => "mode",
            Output::Buckling => "buckling",
            Output::Compliance => "compliance",
            Output::Mass => "mass",
        }
    }

    pub fn parse(w: &str) -> Option<Output> {
        Output::ALL.into_iter().find(|o| o.word() == w)
    }

    /// What its number is.
    pub fn dim(self) -> Dim {
        match self {
            Output::Deflection => Dim::LENGTH,
            Output::Safety | Output::Buckling => Dim::SCALAR,
            Output::Mode => Dim::SCALAR.div(Dim::TIME),
            Output::Compliance => {
                Dim::LENGTH.mul(Dim::LENGTH).mul(Dim::MASS).div(Dim::TIME).div(Dim::TIME)
            }
            Output::Mass => Dim::MASS,
        }
    }

    /// Whether it takes a count after what it is of: `mode(b, n)`.
    pub fn counted(self) -> bool {
        self == Output::Mode
    }

    /// Whether it may be read along a direction: `deflection(p, along: d)`.
    pub fn directed(self) -> bool {
        self == Output::Deflection
    }
}

/// **Which of the three statements a requirement is** (#510): what the solve does with it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Grade {
    /// `claim …`: judged after the solve, never solved for.
    Claim,
    /// `… == v`: a square row whose value is the output, driving a dimension.
    Driving,
    /// `… <= v`, `… >= v`: a bound an optimiser keeps.
    Bound,
}

/// `deflection(arm.tip) <= 0.5mm` — a requirement: an output of what it names, compared with a
/// number the drawing states.
#[derive(Clone, Debug, PartialEq)]
pub struct Requirement {
    pub output: Output,
    pub of: Applied,
    /// `deflection(p, along: d)`'s direction.
    pub along: Option<EntRef>,
    /// `mode(b, n)`'s `n`, counted from 1.
    pub count: Option<u32>,
    /// `<=`, `>=` or `==`, the three a requirement is written with.
    pub compare: Cmp,
    pub value: f64,
    pub claim: bool,
}

impl Requirement {
    pub fn grade(&self) -> Grade {
        match (self.claim, self.compare) {
            (true, _) => Grade::Claim,
            (false, Cmp::Eq) => Grade::Driving,
            (false, _) => Grade::Bound,
        }
    }
}

/// **A scenario** — its supports and loads (an extended scenario's first, then its own) and the
/// requirements it states, which are its own alone.
#[derive(Clone, Debug, PartialEq)]
pub struct Scenario {
    pub name: String,
    /// The scenario it extends, by its index among the drawing's.
    pub extends: Option<usize>,
    pub supports: Vec<Support>,
    pub loads: Vec<Load>,
    pub requirements: Vec<Requirement>,
}
