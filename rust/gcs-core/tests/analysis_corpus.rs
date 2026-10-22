//! **The verification corpus** (#510 P8): the analytic cases every analysis of #510 is held to,
//! each a Solvent document in `rust/examples/analysis/` — a cantilever, Euler's column, Lamé's
//! thick cylinder, Kirsch's plate with a hole, a simply supported plate's modes, and NAFEMS LE1
//! and FV32.  Each states its answer as the claims of a scenario, a bracket about the reference.
//! The gate here holds each document to what it says it is: it solves to no freedom, its body is
//! the size its numbers say, and every bracket it claims is the closed form of its own drawing —
//! read off the solved solid, its material and its scenario, not off its text — at the tolerance
//! the case gives.  The closed forms are in turn held to their equations (`closed`), and an
//! external solver judges every claim (`ccx`, CalculiX, the oracle `make test-ccx` runs).
use gcs_core::expr::Cmp;
use gcs_core::model::{Applied, LoadKind, LoadValue, Material, Output, Property, Requirement};
use gcs_core::program::Elaborated;
use gcs_core::units::Dim;
use std::f64::consts::PI;

use crate::common::{at, ent};

mod ccx;
mod closed;

/// How a case is read: as the solid it draws, or in the plane of the face it sweeps, in plane
/// stress, its depth the thickness.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reading {
    Solid,
    Plane,
}

impl Reading {
    /// How the README's table says it.
    fn word(self) -> &'static str {
        match self {
            Reading::Solid => "solid",
            Reading::Plane => "plane stress",
        }
    }
}

/// A case: its document, the solid it is about and how that is read, the answers its claims
/// are held to, and how finely the oracle meshes it — the largest element in millimetres, and
/// gmsh's elements a full turn of a curve.
pub struct Case {
    pub name: &'static str,
    pub src: &'static str,
    pub solid: &'static str,
    pub reading: Reading,
    pub answers: fn(&Case, &Elaborated) -> Vec<Answer>,
    pub mesh: (f64, u32),
}

macro_rules! case {
    ($name:ident, $solid:literal, $reading:ident, $mesh:expr) => {
        Case {
            name: stringify!($name),
            src: include_str!(concat!("../../examples/analysis/", stringify!($name), ".sv")),
            solid: $solid,
            reading: Reading::$reading,
            answers: $name,
            mesh: $mesh,
        }
    };
}

pub const CASES: [Case; 7] = [
    case!(cantilever, "beam", Solid, (6.0, 0)),
    case!(column, "strut", Solid, (4.0, 0)),
    case!(lame, "tube", Solid, (3.0, 72)),
    case!(kirsch, "plate", Plane, (8.0, 180)),
    case!(plate_modes, "plate", Solid, (20.0, 0)),
    case!(nafems_le1, "membrane", Plane, (40.0, 120)),
    case!(nafems_fv32, "membrane", Plane, (150.0, 0)),
];

/// The case named `name`.
fn case(name: &str) -> &'static Case {
    CASES.iter().find(|c| c.name == name).expect("a case")
}

/// The case's document, read as a host reads it, solved, with no freedom left.
pub fn read(case: &Case) -> Elaborated {
    let mut e = crate::common::text::solved(case.src);
    assert_eq!(crate::common::dof(&mut e.sketch), 0, "{}", case.name);
    e
}

/// The case's solid built exactly, its boundary checked.
pub fn exact(case: &Case, e: &Elaborated) -> std::rc::Rc<gcs_core::solid::Exact> {
    let x = e.sketch.exact_solid(ent(e, case.solid).i())
        .unwrap_or_else(|m| panic!("{}: {m}", case.name));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("{}: {}", case.name, k.msg));
    x
}

/// The solid's extent along each world axis, in the document's lengths.  Every case is drawn in
/// the front, x along and z up, and swept across y.
fn extent(x: &gcs_core::solid::Exact) -> [f64; 3] {
    let (lo, hi) = x.brep.bounds();
    std::array::from_fn(|k| (hi[k] - lo[k]) / x.mm)
}

/// The material the case's solid is made of.
pub fn material<'a>(case: &Case, e: &'a Elaborated) -> &'a Material {
    let name = e.sketch.solids[ent(e, case.solid).i()].material.as_ref().expect("made of one");
    &e.sketch.materials[name]
}

/// A property of the case's material, in the document's own units.
fn property(case: &Case, e: &Elaborated, p: Property) -> f64 {
    let si = material(case, e).get(p).unwrap_or_else(|| panic!("{}: no {p:?}", case.name));
    si / e.sketch.units.to_si(1.0, p.dim())
}

/// A number given in SI, in the document's own units.
fn from_si(e: &Elaborated, v: f64, dim: Dim) -> f64 {
    v / e.sketch.units.to_si(1.0, dim)
}

/// The scenario named `name`.
fn scenario<'a>(e: &'a Elaborated, name: &str) -> &'a gcs_core::model::Scenario {
    &e.scenarios.iter().find(|s| s.scenario.name == name).expect("stated").scenario
}

/// The one load of a scenario, of a kind, as a magnitude: a force along a direction, or a
/// pressure.
fn load(e: &Elaborated, scen: &str, kind: LoadKind) -> f64 {
    let ls: Vec<_> = scenario(e, scen).loads.iter().filter(|l| l.kind == kind).collect();
    let [l] = &ls[..] else { panic!("`{scen}` has {} {kind:?} loads", ls.len()) };
    match l.value {
        LoadValue::Along { magnitude, .. } | LoadValue::Scalar(magnitude) => magnitude,
        LoadValue::Vector(v) => gcs_core::space::norm(v),
    }
}

/// A bracket a scenario claims about one output: its least and greatest, from the claims
/// written `>=`, `<=` or `==`.
#[derive(Clone, Debug, PartialEq)]
pub struct Bracket {
    pub scenario: String,
    pub output: Output,
    pub count: Option<u32>,
    pub of: Applied,
    pub along: Option<gcs_core::model::EntRef>,
    pub lo: Option<f64>,
    pub hi: Option<f64>,
}

impl Bracket {
    /// What it is about: its scenario, its output and its count.
    pub fn key(&self) -> (&str, Output, Option<u32>) {
        (&self.scenario, self.output, self.count)
    }

    /// Whether `v` is inside, at the precision the document writes its numbers to.
    pub fn holds(&self, v: f64) -> bool {
        let slack = |b: f64| 1e-9 * b.abs();
        self.lo.is_none_or(|lo| v >= lo - slack(lo)) && self.hi.is_none_or(|hi| v <= hi + slack(hi))
    }
}

/// Every bracket the case's scenarios claim, in the order first written.  The corpus states its
/// answers as claims alone: an answer is judged, never solved for.
pub fn brackets(e: &Elaborated) -> Vec<Bracket> {
    let mut out: Vec<Bracket> = Vec::new();
    for s in e.scenarios.iter().map(|s| &s.scenario) {
        for r in &s.requirements {
            let Requirement { output, of, along, count, compare, value, claim } = r.clone();
            assert!(claim, "`{}`: the corpus claims its answers, never drives by them", s.name);
            let same = |b: &Bracket| {
                (&b.scenario, b.output, b.count, &b.of, b.along)
                    == (&s.name, output, count, &of, along)
            };
            let i = match out.iter().position(same) {
                Some(i) => i,
                None => {
                    let scenario = s.name.clone();
                    out.push(Bracket { scenario, output, count, of, along, lo: None, hi: None });
                    out.len() - 1
                }
            };
            let b = &mut out[i];
            match compare {
                Cmp::Ge => b.lo = Some(value),
                Cmp::Le => b.hi = Some(value),
                Cmp::Eq => (b.lo, b.hi) = (Some(value), Some(value)),
                other => panic!("`{}`: {other:?}", s.name),
            }
        }
    }
    out
}

/// What a case's bracket is about: its reference in the document's own units, the bracket as
/// factors on it, and how near the document's written numbers must come to them — the last
/// place a number is written to, or the rounding of an expression worked out.
#[derive(Clone, Debug)]
pub struct Answer {
    pub scenario: &'static str,
    pub output: Output,
    pub count: Option<u32>,
    pub reference: f64,
    pub within: [f64; 2],
    pub figures: f64,
}

impl Answer {
    pub fn key(&self) -> (&str, Output, Option<u32>) {
        (self.scenario, self.output, self.count)
    }
}

/// An answer worked out from the document's own expression.
const EXACT: f64 = 1e-12;

/// The bracket `(1 ± tol)` about a reference.
fn about(tol: f64) -> [f64; 2] {
    [1.0 - tol, 1.0 + tol]
}

fn answer(scenario: &'static str, output: Output, count: Option<u32>, reference: f64,
          within: [f64; 2], figures: f64) -> Answer {
    Answer { scenario, output, count, reference, within, figures }
}

/// **Each case's answers, from its solved drawing** (`Case::answers`).  The geometry is read off
/// the solid and the points it draws, the material off the solid's `of`, and the loads off the
/// scenario.
fn cantilever(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let x = exact(case, e);
    let [l, b, h] = extent(&x);
    let (young, rho) = (property(case, e, Property::E), property(case, e, Property::Rho));
    let p = load(e, "tip_load", LoadKind::Force);
    let tip = closed::cantilever_tip(p, l, young, b * h.powi(3) / 12.0);
    let mode = closed::cantilever_mode(young, h * b.powi(3) / 12.0, rho, b * h, l);
    // bending alone within 1%, the solid's shear inside; the document's 1.8751 is the root to
    // five figures
    vec![
        answer("tip_load", Output::Deflection, None, tip, about(0.01), EXACT),
        answer("free", Output::Mode, Some(1), mode, about(0.01), 1e-5),
        answer("free", Output::Mass, None, rho * volume(&x), [1.0, 1.0], 1e-9),
    ]
}

fn column(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let [b, depth, l] = extent(&exact(case, e));
    assert_eq!(b, depth, "a square strut");
    let p = load(e, "press", LoadKind::Pressure);
    let euler = closed::euler_cantilever(property(case, e, Property::E), b.powi(4) / 12.0, l);
    vec![answer("press", Output::Buckling, None, euler / (p * b * b), about(0.01), EXACT)]
}

fn lame(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let x = exact(case, e);
    let [dx, depth, dz] = extent(&x);
    let p = load(e, "inflate", LoadKind::Pressure);
    let t = closed::Lame { a: at(e, "m")[0], c: at(e, "n")[0], p };
    assert!((dx - t.c).abs() < 1e-9 && (dz - t.c).abs() < 1e-9, "a quarter of the tube");
    let quarter = PI / 4.0 * (t.c * t.c - t.a * t.a) * depth;
    assert!((volume(&x) - quarter).abs() < 1e-9 * quarter, "the quarter's volume");
    let (young, nu) = (property(case, e, Property::E), property(case, e, Property::Nu));
    let safety = property(case, e, Property::Yield) / t.mises(t.a);
    vec![
        answer("inflate", Output::Deflection, None, t.moves(t.a, young, nu), about(0.01), EXACT),
        answer("inflate", Output::Safety, None, safety, about(0.03), EXACT),
    ]
}

fn kirsch(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let (r, w) = (at(e, "m")[0], 2.0 * extent(&exact(case, e))[2]);
    let s = -load(e, "pull", LoadKind::Pressure);
    assert!(s > 0.0, "pulled");
    let safety = property(case, e, Property::Yield) / (closed::heywood(2.0 * r, w) * s);
    vec![answer("pull", Output::Safety, None, safety, about(0.03), EXACT)]
}

fn plate_modes(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let [a, h, side] = extent(&exact(case, e));
    assert!((a - side).abs() < 1e-9, "square");
    let (young, nu) = (property(case, e, Property::E), property(case, e, Property::Nu));
    let modes = closed::plate_modes(a, h, young, nu, property(case, e, Property::Rho), 6);
    // the document writes the roots to five figures
    let mode = |n: u32| modes[n as usize - 1].0;
    (1..=6).map(|n| answer("ring", Output::Mode, Some(n), mode(n), about(0.01), 5e-5)).collect()
}

fn nafems_le1(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let x = exact(case, e);
    let length = |v: f64| from_si(e, v, Dim::LENGTH);
    let [dx, thickness, dz] = extent(&x);
    let [(a, b), (c, d)] = closed::LE1_ELLIPSES;
    let near = |got: f64, want: f64| (got - length(want)).abs() < 1e-9 * got;
    assert!(near(dx, c) && near(dz, d) && near(thickness, closed::LE1_THICKNESS));
    let area = PI / 4.0 * (length(c) * length(d) - length(a) * length(b));
    assert!((volume(&x) - area * thickness).abs() < 1e-9 * area * thickness, "two exact ellipses");
    let pull = from_si(e, closed::LE1_PULL, Dim::PRESSURE);
    assert!((load(e, "le1", LoadKind::Pressure) + pull).abs() < 1e-9 * pull, "pulled outward");
    let target = from_si(e, closed::LE1_SYY_D, Dim::PRESSURE);
    let safety = property(case, e, Property::Yield) / target;
    vec![answer("le1", Output::Safety, None, safety, about(0.03), EXACT)]
}

fn nafems_fv32(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let x = exact(case, e);
    let length = |v: f64| from_si(e, v, Dim::LENGTH);
    let [dx, thickness, dz] = extent(&x);
    let (l, root, tip) = closed::FV32_TAPER;
    assert!((dx - length(l)).abs() < 1e-9 * dx && (dz - length(root)).abs() < 1e-9 * dz);
    let taper = length(l) * (length(root) + length(tip)) / 2.0 * thickness;
    assert!((volume(&x) - taper).abs() < 1e-6 * taper, "an even taper");
    let mode = |i: usize| answer("fv32", Output::Mode, Some(i as u32 + 1), closed::FV32[i],
                                 about(0.01), EXACT);
    (0..closed::FV32.len()).map(mode).collect()
}

/// The solid's volume, in the document's lengths.
fn volume(x: &gcs_core::solid::Exact) -> f64 {
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// The bracket an answer is about.
pub fn bracket<'a>(bs: &'a [Bracket], a: &Answer) -> &'a Bracket {
    let found: Vec<_> = bs.iter().filter(|b| b.key() == a.key()).collect();
    let [b] = &found[..] else { panic!("{} brackets of {a:?}", found.len()) };
    b
}

/// **Every claim is its own drawing's answer.**  For each case, each bracket its scenarios claim
/// is the answer's factors on the reference worked out from the solved drawing, and no claim is
/// left over.
#[test]
fn each_case_claims_its_own_answer() {
    for case in &CASES {
        let e = read(case);
        let bs = brackets(&e);
        let want = (case.answers)(case, &e);
        assert_eq!(bs.len(), want.len(), "{}: {bs:?}", case.name);
        for a in &want {
            let b = bracket(&bs, a);
            let near = |got: Option<f64>, side: f64| {
                let w = side * a.reference;
                got.is_some_and(|g| (g - w).abs() <= a.figures * w.abs())
            };
            assert!(near(b.lo, a.within[0]) && near(b.hi, a.within[1]),
                    "{}: {:?} claims [{:?}, {:?}] about {} by {:?}", case.name, a.output, b.lo,
                    b.hi, a.reference, a.within);
            assert!(b.holds(a.reference), "{}: {a:?}", case.name);
        }
    }
}

/// The cantilever's bracket holds Euler–Bernoulli's answer, a frame's, and Timoshenko's, the
/// solid's.
#[test]
fn the_cantilevers_bracket_holds_both_beams() {
    let case = case("cantilever");
    let e = read(case);
    let [l, b, h] = extent(&exact(case, &e));
    let (young, nu) = (property(case, &e, Property::E), property(case, &e, Property::Nu));
    let p = load(&e, "tip_load", LoadKind::Force);
    let bending = closed::cantilever_tip(p, l, young, b * h.powi(3) / 12.0);
    let shear = closed::cantilever_shear(p, l, young, nu, b * h);
    let bs = brackets(&e);
    let tip = bs.iter().find(|x| x.output == Output::Deflection).unwrap();
    assert!(tip.holds(bending) && tip.holds(bending + shear), "{tip:?}: {bending} + {shear}");
    assert!(shear / bending > 1e-3 && shear / bending < 3e-3, "{}", shear / bending);
}

/// Every document in the directory is a case, and the README's table reads each as the case
/// table does.
#[test]
fn the_corpus_is_its_files() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/analysis");
    let mut files: Vec<String> = std::fs::read_dir(&dir).unwrap().flatten()
        .filter_map(|f| f.file_name().into_string().ok()?.strip_suffix(".sv").map(String::from))
        .collect();
    files.sort();
    let mut listed: Vec<String> = CASES.iter().map(|c| c.name.to_string()).collect();
    listed.sort();
    assert_eq!(files, listed, "every document in rust/examples/analysis is a case");
    let readme = include_str!("../../examples/analysis/README.md");
    for case in &CASES {
        let row = readme.lines().find(|l| l.starts_with(&format!("| `{}.sv` |", case.name)));
        let row = row.unwrap_or_else(|| panic!("{} is in the README's table", case.name));
        assert!(row.contains(&format!("| {} |", case.reading.word())), "{row}");
    }
}
