//! **Measurements of the solved drawing, read after the solve** (`expr::Measure`).
//!
//! A motion's `ratio:`, `phase:` and `advance:` are read when the motion is read — by a
//! placement, a sweep, an envelope, a mesh — and never by the solve, so they may be written
//! over what the solve decides: `ratio: length(gen_g) / length(pinion_radius)`.  The text is
//! compiled once at elaboration, every measured name resolved to the entity it denotes, and
//! worked out again from the geometry each time the motion is read.  Nothing is stored that a
//! solve could leave stale, and a cache keyed on what a motion reads (`solid::reads`) reads the
//! number this gives, so it moves exactly when the measured geometry does.
//!
//! Every other context is read before the solve (a `param` feeds constraints, a seed is where
//! the solve begins, an extent is settled at elaboration), and a measurement there is refused:
//! `expr::measure_refusal`, E107.

use crate::expr::{self, Aff, Ast, Measure};
use crate::model::{EntKind, EntRef, Sketch};
use crate::units::{Dim, Units};
use std::collections::BTreeMap;

/// An expression over measurements of the drawing, compiled against it.
#[derive(Clone, Debug)]
pub struct Measured {
    /// As the sheet reads it: every measured name absolute (`length(pair.gen_g)`).
    pub text: String,
    body: Ast,
    /// What each measured name denotes.
    pub ents: BTreeMap<String, EntRef>,
    /// What a length reads as: a `Length` where the document names a unit, a bare number where
    /// it does not (the seed reading's rule).
    length: Dim,
}

/// Which of a motion's numbers an expression stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionSlot {
    Ratio,
    Phase,
    Advance,
}

impl MotionSlot {
    pub fn label(self) -> &'static str {
        match self {
            MotionSlot::Ratio => "ratio",
            MotionSlot::Phase => "phase",
            MotionSlot::Advance => "advance",
        }
    }

    /// The dimension the slot takes.
    pub fn dim(self) -> Dim {
        match self {
            MotionSlot::Ratio => Dim::SCALAR,
            MotionSlot::Phase => Dim::ANGLE,
            MotionSlot::Advance => Dim::LENGTH,
        }
    }
}

/// One of a motion's numbers, written as a measurement: worked out whenever the motion is read.
#[derive(Clone, Debug)]
pub struct MotionMeasure {
    pub slot: MotionSlot,
    pub value: Measured,
}

/// The kinds each measurement takes, as the words a refusal says.
fn takes(m: Measure) -> &'static str {
    match m {
        Measure::Length => "a line or an arc",
        Measure::Radius => "a circle, an arc, a sphere or a cylinder",
        Measure::Distance => "two points, or a point and a line",
        Measure::Angle => "two lines",
    }
}

fn fits(m: Measure, kinds: &[EntKind]) -> bool {
    use EntKind::*;
    match (m, kinds) {
        (Measure::Length, [Line | Arc]) => true,
        (Measure::Radius, [Circle | Arc | Sphere | Cylinder]) => true,
        (Measure::Distance, [Point, Point | Line] | [Line, Point]) => true,
        (Measure::Angle, [Line, Line]) => true,
        _ => false,
    }
}

impl Measured {
    /// Compile `text` (as the flattener left it: measured names absolute) against the drawing,
    /// `resolve` saying what each name denotes, and check it comes to `want`.
    pub fn compile(
        text: &str,
        units: Units,
        want: Dim,
        what: &str,
        resolve: impl Fn(&str) -> Result<EntRef, String>,
    ) -> Result<Measured, String> {
        let p = expr::parse_in(text, units)?;
        if p.name.is_some() {
            return Err(format!("`{what}` measures the drawing and names nothing"));
        }
        let mut ents = BTreeMap::new();
        for (m, args) in p.body.measures() {
            let mut kinds = Vec::with_capacity(args.len());
            for a in &args {
                let e = match ents.get(a) {
                    Some(&e) => e,
                    None => resolve(a)?,
                };
                ents.insert(a.clone(), e);
                kinds.push(e.kind);
            }
            if !fits(m, &kinds) {
                return Err(format!(
                    "`{}` measures {}, and was given {}",
                    m.text(&args),
                    takes(m),
                    kinds.iter().map(|k| format!("a {}", k.as_str())).collect::<Vec<_>>().join(" and ")
                ));
            }
        }
        let length = if units.name().is_some() { Dim::LENGTH } else { Dim::SCALAR };
        let out = Measured { text: text.trim().to_string(), body: p.body, ents, length };
        // the dimension is a fact about the text, not about where the geometry stands: checked
        // once here over stand-in numbers of the right kinds, so a wrong one is refused at
        // elaboration and never discovered by the first read
        let a = expr::eval_measured(&out.body, &BTreeMap::new(), &|m, _| {
            Ok(Aff::of_dim(1.0, out.dim_of(m)))
        })?;
        if a.free.is_some() {
            return Err(format!("`{what}` reads `{}`, which nothing gives a number", a.free.unwrap()));
        }
        a.dim.require(want, what)?;
        Ok(out)
    }

    fn dim_of(&self, m: Measure) -> Dim {
        match m {
            Measure::Angle => Dim::ANGLE,
            Measure::Length | Measure::Radius | Measure::Distance => self.length,
        }
    }

    /// What it comes to on the drawing as it now stands, in the language's units (degrees for
    /// an angle, the document's for a length).
    pub fn value(&self, sk: &Sketch) -> Result<f64, String> {
        let a = expr::eval_measured(&self.body, &BTreeMap::new(), &|m, args| {
            let es: Vec<EntRef> = args
                .iter()
                .map(|a| self.ents.get(a).copied().ok_or_else(|| format!("`{a}` names nothing")))
                .collect::<Result<_, _>>()?;
            Ok(Aff::of_dim(measure(sk, m, &es)?, self.dim_of(m)))
        })?;
        match a.number() {
            Some(v) if v.is_finite() => Ok(v),
            Some(v) => Err(format!("`{}` comes to {v}", self.text)),
            None => Err(format!("`{}` is not a number", self.text)),
        }
    }

    /// The same expression over entities renumbered by `f`; `None` where one of them is gone,
    /// so what reads it goes with it (a copy that leaves a measured line behind).
    pub fn remap(&self, f: impl Fn(EntRef) -> Option<EntRef>) -> Option<Measured> {
        let ents = self.ents.iter().map(|(n, &e)| f(e).map(|e| (n.clone(), e))).collect::<Option<_>>()?;
        Some(Measured { ents, ..self.clone() })
    }
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    crate::space::norm(std::array::from_fn(|k| b[k] - a[k]))
}

/// One measurement, in space: a line's length is between its ends' lifts, a distance to a
/// line is to the line produced, and an angle is between two lines' directions `p1 → p2`.
pub fn measure(sk: &Sketch, m: Measure, es: &[EntRef]) -> Result<f64, String> {
    let get = |e: EntRef| -> Result<(), String> {
        (e.i() < sk.count(e.kind)).then_some(()).ok_or_else(|| format!("no such {}", e.kind.as_str()))
    };
    for &e in es {
        get(e)?;
    }
    let ends = |l: usize| {
        let l = &sk.lines[l];
        (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
    };
    let param = |p: u32| sk.params[p as usize].value;
    let v = match (m, es) {
        (Measure::Length, [e]) if e.kind == EntKind::Line => {
            let (a, b) = ends(e.i());
            dist(a, b)
        }
        (Measure::Length, [e]) if e.kind == EntKind::Arc => {
            let (a0, a1) = sk.arc_angles(e.i());
            param(sk.arcs[e.i()].radius).abs() * (a1 - a0)
        }
        (Measure::Radius, [e]) => param(match e.kind {
            EntKind::Circle => sk.circles[e.i()].radius,
            EntKind::Arc => sk.arcs[e.i()].radius,
            EntKind::Sphere => sk.spheres[e.i()].radius,
            EntKind::Cylinder => sk.cylinders[e.i()].param,
            k => return Err(format!("a {} has no radius", k.as_str())),
        })
        .abs(),
        (Measure::Distance, [a, b]) if a.kind == EntKind::Point && b.kind == EntKind::Point => {
            dist(sk.world_point(a.i()), sk.world_point(b.i()))
        }
        (Measure::Distance, [p, l] | [l, p]) if p.kind == EntKind::Point && l.kind == EntKind::Line => {
            let x = sk.world_point(p.i());
            let (a, b) = ends(l.i());
            let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
            let n = crate::space::norm(d);
            if n == 0.0 {
                return Err("a distance to a line of no length".into());
            }
            let r: [f64; 3] = std::array::from_fn(|k| x[k] - a[k]);
            crate::space::norm(crate::space::cross(r, d)) / n
        }
        (Measure::Angle, [l1, l2]) if l1.kind == EntKind::Line && l2.kind == EntKind::Line => {
            let dir = |l: usize| {
                let (a, b) = ends(l);
                let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
                d
            };
            let (u, w) = (dir(l1.i()), dir(l2.i()));
            let (nu, nw) = (crate::space::norm(u), crate::space::norm(w));
            if nu == 0.0 || nw == 0.0 {
                return Err("an angle to a line of no length".into());
            }
            // atan2 of the cross and the dot: accurate at 0° and 180°, where acos is not
            let c = crate::space::norm(crate::space::cross(u, w));
            c.atan2(crate::space::dot(u, w)).to_degrees()
        }
        _ => return Err(format!("`{}` cannot measure these", m.name())),
    };
    if !v.is_finite() {
        return Err(format!("`{}` came to {v}", m.name()));
    }
    Ok(v)
}
