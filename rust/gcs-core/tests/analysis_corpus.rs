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
use gcs_core::model::{Applied, LoadKind, LoadValue, Output, Property, Requirement, Sketch};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::units::Dim;
use std::f64::consts::PI;

mod ccx;
mod closed;

/// How a case is read: as the solid it draws, or in the plane of the face it sweeps, in plane
/// stress, its depth the thickness.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reading {
    Solid,
    Plane,
}

/// A case: its document, how it is read, and the stage of #510 that is first to answer it.
pub struct Case {
    pub name: &'static str,
    pub src: &'static str,
    pub solid: &'static str,
    pub reading: Reading,
    pub stage: &'static str,
}

macro_rules! case {
    ($name:literal, $solid:literal, $reading:ident, $stage:literal) => {
        Case {
            name: $name,
            src: include_str!(concat!("../../examples/analysis/", $name, ".sv")),
            solid: $solid,
            reading: Reading::$reading,
            stage: $stage,
        }
    };
}

pub const CASES: [Case; 7] = [
    case!("cantilever", "beam", Solid, "S3"),
    case!("column", "strut", Solid, "S3"),
    case!("lame", "tube", Solid, "S3"),
    case!("kirsch", "plate", Plane, "S2"),
    case!("plate_modes", "plate", Solid, "S3"),
    case!("nafems_le1", "membrane", Plane, "S2"),
    case!("nafems_fv32", "membrane", Plane, "S2"),
];

/// The case's document, read as a host reads it, solved, with no freedom left.
pub fn read(case: &Case) -> Elaborated {
    let (p, errs, linked) = gcs_core::library::parse_linked(case.src);
    assert!(errs.is_empty() && linked.is_empty(), "{}: {errs:?} {linked:?}", case.name);
    let mut e = elaborate(&p);
    let said: Vec<_> = e.errors().map(|d| d.message.clone()).collect();
    assert!(e.ok() && said.is_empty(), "{}: {said:?}", case.name);
    let solved = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(solved.success, "{} does not solve", case.name);
    let opts = gcs_core::diagnose::DiagnoseOptions::default();
    assert_eq!(gcs_core::diagnose::diagnose(&mut e.sketch, opts).dof, 0, "{}", case.name);
    e
}

/// The index of the solid `name`.
pub fn solid(sk: &Sketch, name: &str) -> usize {
    sk.solids.iter().position(|s| s.name == name).unwrap_or_else(|| panic!("no solid `{name}`"))
}

/// The case's solid built exactly, its boundary checked.
pub fn exact(case: &Case, sk: &Sketch) -> std::rc::Rc<gcs_core::solid::Exact> {
    let x = sk.exact_solid(solid(sk, case.solid)).unwrap_or_else(|m| panic!("{}: {m}", case.name));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("{}: {}", case.name, k.msg));
    x
}

/// The solid's extent along each world axis, in the document's lengths.  Every case is drawn in
/// the front, x along and z up, and swept across y.
fn extent(x: &gcs_core::solid::Exact) -> [f64; 3] {
    let (lo, hi) = x.brep.bounds();
    std::array::from_fn(|k| (hi[k] - lo[k]) / x.mm)
}

/// A property of the material the case's solid is made of, in the document's own units.
fn property(case: &Case, sk: &Sketch, p: Property) -> f64 {
    let name = sk.solids[solid(sk, case.solid)].material.as_ref().expect("made of a material");
    let si = sk.materials[name].get(p).unwrap_or_else(|| panic!("`{name}` states no {p:?}"));
    si / sk.units.to_si(1.0, p.dim())
}

/// A point the case draws, where it solved, in the document's lengths.
fn at(e: &Elaborated, name: &str) -> [f64; 3] {
    e.sketch.world_point(crate::common::ent(e, name).i())
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
        LoadValue::Vector(v) => (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt(),
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

/// **Each case's answers, from its solved drawing.**  The geometry is read off the solid and the
/// points it draws, the material off the solid's `of`, and the loads off the scenario.
pub fn answers(case: &Case, e: &Elaborated) -> Vec<Answer> {
    let sk = &e.sketch;
    let x = exact(case, sk);
    let [dx, dy, dz] = extent(&x);
    let (young, nu) = (property(case, sk, Property::E), property(case, sk, Property::Nu));
    let mass = |rho: f64| rho * gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3);
    match case.name {
        "cantilever" => {
            let (l, b, h) = (dx, dy, dz);
            let rho = property(case, sk, Property::Rho);
            let p = load(e, "tip_load", LoadKind::Force);
            let tip = closed::cantilever_tip(p, l, young, b * h.powi(3) / 12.0);
            let mode = closed::cantilever_mode(young, h * b.powi(3) / 12.0, rho, b * h, l);
            // bending alone within 1%, the solid's shear inside; the document's 1.8751 is the
            // root to five figures
            vec![
                answer("tip_load", Output::Deflection, None, tip, about(0.01), EXACT),
                answer("free", Output::Mode, Some(1), mode, about(0.01), 1e-5),
                answer("free", Output::Mass, None, mass(rho), [1.0, 1.0], 1e-9),
            ]
        }
        "column" => {
            let (b, l) = (dx, dz);
            assert_eq!(dx, dy, "a square strut");
            let p = load(e, "press", LoadKind::Pressure);
            let euler = closed::euler_cantilever(young, b.powi(4) / 12.0, l);
            vec![answer("press", Output::Buckling, None, euler / (p * b * b), about(0.01), EXACT)]
        }
        "lame" => {
            let p = load(e, "inflate", LoadKind::Pressure);
            let t = closed::Lame { a: at(e, "m")[0], c: at(e, "n")[0], p };
            assert!((dx - t.c).abs() < 1e-9 && (dz - t.c).abs() < 1e-9, "a quarter of the tube");
            let volume = PI / 4.0 * (t.c * t.c - t.a * t.a) * dy;
            assert!((mass(1.0) - volume).abs() < 1e-9 * volume, "the quarter's volume");
            let yield_ = property(case, sk, Property::Yield);
            vec![
                answer("inflate", Output::Deflection, None, t.moves(t.a, young, nu), about(0.01),
                       EXACT),
                answer("inflate", Output::Safety, None, yield_ / t.mises(t.a), about(0.03), EXACT),
            ]
        }
        "kirsch" => {
            let (r, w) = (at(e, "m")[0], 2.0 * dz);
            let s = -load(e, "pull", LoadKind::Pressure);
            assert!(s > 0.0, "pulled");
            let kt = closed::heywood(2.0 * r, w);
            let yield_ = property(case, sk, Property::Yield);
            vec![answer("pull", Output::Safety, None, yield_ / (kt * s), about(0.03), EXACT)]
        }
        "plate_modes" => {
            assert!((dx - dz).abs() < 1e-9, "square");
            let rho = property(case, sk, Property::Rho);
            let modes = closed::plate_modes(dx, dy, young, nu, rho, 6);
            // the document writes the roots to five figures
            let mode = |n: u32| modes[n as usize - 1].0;
            (1..=6).map(|n| answer("ring", Output::Mode, Some(n), mode(n), about(0.01), 5e-5))
                .collect()
        }
        "nafems_le1" => {
            let target = closed::LE1_SYY_D / sk.units.to_si(1.0, Dim::PRESSURE);
            let yield_ = property(case, sk, Property::Yield);
            assert!((dx - 3250.0).abs() < 1e-9 && (dz - 2750.0).abs() < 1e-9);
            assert!((dy - 100.0).abs() < 1e-9);
            let area = PI / 4.0 * (3250.0 * 2750.0 - 2000.0 * 1000.0);
            assert!((mass(1.0) - area * dy).abs() < 1e-9 * area * dy, "between two exact ellipses");
            let pull = 10e6 / sk.units.to_si(1.0, Dim::PRESSURE);
            let pressure = load(e, "le1", LoadKind::Pressure);
            assert!((pressure + pull).abs() < 1e-9 * pull, "pulled at 10 MPa");
            vec![answer("le1", Output::Safety, None, yield_ / target, about(0.03), EXACT)]
        }
        "nafems_fv32" => {
            assert!((dx - 10000.0).abs() < 1e-9 && (dz - 5000.0).abs() < 1e-9);
            let volume = 30e6 * dy;
            assert!((mass(1.0) - volume).abs() < 1e-6 * volume, "tapering 5 m to 1 m over 10 m");
            closed::FV32.iter().enumerate()
                .map(|(i, &f)| {
                    answer("fv32", Output::Mode, Some(i as u32 + 1), f, about(0.01), EXACT)
                })
                .collect()
        }
        other => panic!("no answers for `{other}`"),
    }
}

/// The bracket an answer is about.
pub fn bracket<'a>(bs: &'a [Bracket], a: &Answer) -> &'a Bracket {
    let found: Vec<_> = bs.iter()
        .filter(|b| (b.scenario.as_str(), b.output, b.count) == (a.scenario, a.output, a.count))
        .collect();
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
        let want = answers(case, &e);
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
    let case = &CASES[0];
    let e = read(case);
    let sk = &e.sketch;
    let [l, b, h] = extent(&exact(case, sk));
    let (young, nu) = (property(case, sk, Property::E), property(case, sk, Property::Nu));
    let p = load(&e, "tip_load", LoadKind::Force);
    let bending = closed::cantilever_tip(p, l, young, b * h.powi(3) / 12.0);
    let shear = closed::cantilever_shear(p, l, young, nu, b * h);
    let bs = brackets(&e);
    let tip = bs.iter().find(|x| x.output == Output::Deflection).unwrap();
    assert!(tip.holds(bending) && tip.holds(bending + shear), "{tip:?}: {bending} + {shear}");
    assert!(shear / bending > 1e-3 && shear / bending < 3e-3, "{}", shear / bending);
}

/// Each case is read as the table says, and names its stage.
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
    for case in &CASES {
        assert!(["S0", "S2", "S3"].contains(&case.stage));
        let readme = include_str!("../../examples/analysis/README.md");
        assert!(readme.contains(&format!("`{}.sv`", case.name)), "{} is in the README", case.name);
    }
}
