//! **An equation of integrals the drawing states** (§6.25, #953): `integral(density(p) over p in
//! boat) == M`, `claim integral(1 over p in web) <= 42cm^3` — each side arithmetic over integrals,
//! numbers and the unknowns the drawing declares, each integral a body over a point `p` of a set
//! (a solid, a face of one, a beam or a line one is drawn on, an instance's bodies) by the set's
//! own measure.  Stated, it is a row of the drawing (`CKind::Integral`) the solve holds; claimed,
//! it is judged on the solved drawing.  The words a document writes it in — `mass`, `volume`,
//! `area`, `moment`, `centroid`, `centerofmass` — are `std.measures`'s.  Pure data, set by
//! elaboration and carried by the Solvent source alone.

use super::scenario::Applied;
use super::EntRef;
use crate::expr::{Ast, Cmp};
use crate::units::Dim;

/// **What an integral is over**: a beam, a line a beam is drawn on, a solid or a face of one —
/// or every body under an instance, each once.
#[derive(Clone, Debug, PartialEq)]
pub enum Domain {
    Of(Applied),
    Bodies(Vec<usize>),
}

impl Domain {
    /// The solids it is over: its bodies, or the solid it is or is a face of.
    pub fn solids(&self) -> Vec<usize> {
        match self {
            Domain::Of(a) => a.solid().into_iter().collect(),
            Domain::Bodies(b) => b.clone(),
        }
    }

    /// The face it is over, where it is one.
    pub fn face(&self) -> Option<&super::FaceRef> {
        match self {
            Domain::Of(Applied::Face(f)) => Some(f),
            _ => None,
        }
    }
}

/// **What an integrand reads besides its point**, as numbers of the drawing the solve may move:
/// a point (three), a direction made unit (three: a line's, an axis's, a plane's normal), or a
/// line whole (six: its first point, then its direction).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reading {
    Point(usize),
    Direction(EntRef),
    Line(EntRef),
}

impl Reading {
    /// How many of the integrand's variables it is.
    pub fn width(self) -> usize {
        match self {
            Reading::Line(_) => 6,
            _ => 3,
        }
    }
}

/// The integrand's variables before its readings': the point's three numbers and the density
/// there.
pub const POINT_VARS: usize = 4;

/// **The name of the integrand's variable `k`**: `#x`, `#y`, `#z`, `#rho`, then each reading's
/// numbers `#r0`, `#r1`, … — names no document can spell.
pub fn integrand_var(k: usize) -> String {
    match k {
        0 => "#x".into(),
        1 => "#y".into(),
        2 => "#z".into(),
        3 => "#rho".into(),
        k => format!("#r{}", k - POINT_VARS),
    }
}

/// **The name of a side's variable for integral `j`**: `#0`, `#1`, … — before the unknowns, each
/// by its own name.
pub fn integral_var(j: usize) -> String {
    format!("#{j}")
}

/// **One integral**: over what (`over`), its body (`integrand`, over `integrand_var`s), what the
/// body reads (`reads`, in the order its variables run), and what it comes to: the body's
/// dimension times its set's measure's.
#[derive(Clone, Debug, PartialEq)]
pub struct Integral {
    pub over: Domain,
    pub integrand: Ast,
    pub reads: Vec<Reading>,
    pub dim: Dim,
}

impl Integral {
    /// The integrand's variables, its point's and its readings'.
    pub fn vars(&self) -> Vec<String> {
        let n = POINT_VARS + self.reads.iter().map(|r| r.width()).sum::<usize>();
        (0..n).map(integrand_var).collect()
    }

    /// Whether its body reads its point or anything of the drawing: an integral that does not
    /// is a number times its set's volume or mass, read as a whole over any set (a beam's, a
    /// prism's).
    pub fn local(&self) -> bool {
        let names = self.integrand.deps();
        !self.reads.is_empty() || ["#x", "#y", "#z"].iter().any(|v| names.contains(*v))
    }
}

impl Integral {
    /// **Whether its body is a polynomial of degree two or less in its point** — a volume, a
    /// first moment, a second — read then through the ten moments of where it is taken, exactly.
    pub fn quadratic(&self) -> bool {
        degree(&self.integrand).is_some_and(|d| d <= 2)
    }
}

/// An expression's degree in the integral's point (`#x`, `#y`, `#z`): `None` where it is no
/// polynomial in it — a root, a quotient by it, a function of it, a power it varies.
fn degree(a: &Ast) -> Option<u32> {
    let point = |v: &str| ["#x", "#y", "#z"].contains(&v);
    match a {
        Ast::Num(..) => Some(0),
        Ast::Var(v) => Some(u32::from(point(v))),
        Ast::Neg(x) => degree(x),
        Ast::Bin(op, x, y) => {
            let (dx, dy) = (degree(x)?, degree(y)?);
            match op {
                crate::expr::Op::Add | crate::expr::Op::Sub => Some(dx.max(dy)),
                crate::expr::Op::Mul => Some(dx + dy),
                crate::expr::Op::Div => (dy == 0).then_some(dx),
                crate::expr::Op::Pow => match &**y {
                    Ast::Num(k, _) if *k >= 0.0 && k.fract() == 0.0 && *k <= 8.0 => {
                        Some(dx * *k as u32)
                    }
                    _ => (dx == 0 && dy == 0).then_some(0),
                },
            }
        }
        Ast::Call(_, args) => args.iter().all(|x| degree(x) == Some(0)).then_some(0),
        Ast::Measure(..) | Ast::If(..) => None,
    }
}

/// **An equation the drawing states** (§6.25): its integrals, its two sides over them (`lhs`,
/// `rhs`, each over `integral_var`s and then the `unknowns` by name), how they compare (`==`
/// stated or claimed; `<=` and `>=` claimed alone), whether it is claimed, what its sides are,
/// and what a report calls it — the statement as written, a word's use where a word wrote it.
#[derive(Clone, Debug, PartialEq)]
pub struct Measured {
    pub integrals: Vec<Integral>,
    pub lhs: Ast,
    pub rhs: Ast,
    pub unknowns: Vec<String>,
    pub compare: Cmp,
    pub claim: bool,
    pub dim: Dim,
    pub head: String,
}

impl Measured {
    /// The sides' variables: each integral's, then each unknown's.
    pub fn vars(&self) -> Vec<String> {
        (0..self.integrals.len()).map(integral_var).chain(self.unknowns.iter().cloned()).collect()
    }

    /// Every solid any of its integrals is over, each once, in the order first named.
    pub fn solids(&self) -> Vec<usize> {
        let mut out: Vec<usize> = Vec::new();
        for s in self.integrals.iter().flat_map(|i| i.over.solids()) {
            if !out.contains(&s) {
                out.push(s);
            }
        }
        out
    }
}
