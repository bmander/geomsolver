//! **The documents #459 measures boundaries on, and the counts it reads off them**: the
//! prototype (`solve::solve_through`, #454) against the flat solve, on cases each written once in
//! Solvent and solved both ways.  The timings are the bench's (`bench boundaries`), since the
//! core keeps no clock; what is here does not depend on the machine.
//!
//! * **Repeated and distinct parts** (`belts`): N belt drives, the direction paper's (§6.1), each
//!   with its two runs as boundaries.  Identical copies stand apart and read nothing of each other;
//!   distinct ones differ in the large pulley, and may share one unknown centre distance that the
//!   first drive's crossing settles, so every step of the parent moves every child.
//! * **A closed loop** (`dyads`): a chain of two-root points, each over the last and a ground
//!   pivot, closed by one parent row over the chain's end: a kinematic loop through `k`
//!   boundaries, which the flat solve sees as one coupled block.
//! * **Branch and singular inputs** (`belts` swept, `meet`): the centres swept down until one rim
//!   holds the other, the radii swept equal so the runs go parallel, and a line swept onto a circle
//!   it cuts, where the boundary's two roots meet (`x² = u` at 0).
//!
//! Generators of benchmark documents, not cases: they stay off the case library and the app's
//! menu, as `fixtures.rs`'s do.

use crate::linalg::absmax;
use crate::model::Sketch;
use crate::program::Elaborated;
use crate::solve::{solve, Boundaries, Boundary, Failure, SolveOpts, SolveResult};
use crate::system::System;
use std::fmt::Write;

/// A document to solve both ways, and the instances it is solved through.
pub struct Document {
    pub src: String,
    pub boundaries: Vec<String>,
}

/// One belt drive: its centre distance and its two pulleys' radii, mm.
#[derive(Clone, Copy, Debug)]
pub struct Drive {
    pub centres: f64,
    pub r_small: f64,
    pub r_large: f64,
}

impl Drive {
    /// The paper's drive, its centres 120 apart.
    pub const PAPER: Drive = Drive { centres: 120.0, r_small: 20.0, r_large: 45.0 };
}

/// What a drive is made of: `boundary_solve.rs`'s belt, the pulley's solid left out (the solve
/// reads none) and the crossing seeded from its line, so a copy anywhere starts where it should.
const DRIVE_PARTS: &str = "\
component Pulley(view: plane, centre: point, r: Length, bore: Length) {
  in view {
    rim := circle(center: centre) hint(r: r)
    radius(r) rim
    hole := circle(center: centre) hint(r: bore)
    radius(bore) hole
  }
}

component Run(view: plane, a: circle, b: circle, up: Scalar) {
  in view {
    s := point hint((a.center.x, a.center.y + up * a.r))
    e := point hint((b.center.x, b.center.y + up * b.r))
    span := line(s, e)
    s coincident a
    e coincident b
    span tangent(at: p1) a
    span tangent(at: p2) b
  }
}

component Crossing(view: plane, a: line, b: line, l: line) {
  in view {
    x := point hint((l.p1.x - 60mm, l.p1.y))
    x coincident a
    x coincident b
    claim x coincident l
  }
}
";

/// How far apart the copies stand, mm: no rim of one reaches another.
pub const PITCH: f64 = 400.0;

/// `drives`, each standing `PITCH` along from the last with its second shaft seeded off its place,
/// so the parent has the shaft to move and the runs follow.  `shared`: every drive's centre
/// distance is one unknown, `c`, which the first drive's crossing settles where its own centres
/// would put it — one parent row read through the first drive's runs, and one parent unknown read
/// by every drive's.
pub fn belts(drives: &[Drive], shared: bool) -> Document {
    let mut src = String::from("unit mm\nuse std (horizontal)\n\n");
    src.push_str(DRIVE_PARTS);
    if shared {
        src.push_str("\nparam c: Length hint(100mm)\n");
    }
    let mut boundaries = Vec::new();
    for (k, d) in drives.iter().enumerate() {
        let x = PITCH * k as f64;
        let centres = if shared { "c".to_string() } else { format!("{}mm", d.centres) };
        let _ = write!(src, "
in std.front {{
  o1_{k} := point
  fix(({x}, 0)) o1_{k}
  o2_{k} := point hint(({}, 10))
  o1_{k} horizontal o2_{k}
  o1_{k} distance({centres}) o2_{k}
  construction centres_{k} := line(o1_{k}, o2_{k})
}}
small_{k} := Pulley(std.front, o1_{k}, r: {}mm, bore: 4mm)
large_{k} := Pulley(std.front, o2_{k}, r: {}mm, bore: 4mm)
upper_{k} := Run(std.front, small_{k}.rim, large_{k}.rim, up: 1)
lower_{k} := Run(std.front, small_{k}.rim, large_{k}.rim, up: -1)
meet_{k} := Crossing(std.front, upper_{k}.span, lower_{k}.span, centres_{k})
", x + 100.0, d.r_small, d.r_large);
        boundaries.push(format!("upper_{k}"));
        boundaries.push(format!("lower_{k}"));
    }
    if shared {
        // the runs, produced, meet at x = −r₁c/(r₂ − r₁)
        let d = drives[0];
        let at = -d.r_small * d.centres / (d.r_large - d.r_small);
        let _ = write!(src, "\nin std.front {{\n  h := point\n  fix(x == {at}mm) h\n}}\n\
                             h coincident meet_0.x\n");
    }
    Document { src, boundaries }
}

/// The links' length, mm, of `dyads`: the crank's and each two-root point's to the joint before
/// it and to its pivot.
const LINK: f64 = 5.0;

/// The crank's angle the loop of `dyads` closes at, and the one it is seeded at, degrees.
pub const DYAD_CLOSED: f64 = 60.0;
pub const DYAD_SEED: f64 = 55.0;

/// The pivots of `dyads(k)`, the crank's at the origin: chosen so that with the crank at
/// `DYAD_CLOSED` the joints run `LINK` apart along a wavering line, each held from its pivot at a
/// wavering angle.  Neither is straight, so the closure's length is no extremum of the crank's
/// angle and every joint stays within a link's reach of the last as the crank turns about it.
fn dyad_pivots(k: usize) -> Vec<(f64, f64)> {
    let (rad, at) = (f64::to_radians, |a: f64| (LINK * a.cos(), LINK * a.sin()));
    let mut j = at(rad(DYAD_CLOSED));
    let mut g = vec![(0.0, 0.0)];
    for i in 1..=k {
        let bend = rad(25.0 * (0.9 * i as f64).cos());
        let step = at(bend);
        j = (j.0 + step.0, j.1 + step.1);
        let hold = at(rad(60.0 + 15.0 * (i as f64).sin()) + bend);
        g.push((j.0 - hold.0, j.1 - hold.1));
    }
    g
}

/// Where the joints of `dyads(k)` stand with the crank at `theta` degrees: the crank's end, then
/// each two-root point on the left of the way from the last joint to its pivot.  `None` where a
/// joint is out of a link's reach.
pub fn dyad_joints(k: usize, theta: f64) -> Option<Vec<(f64, f64)>> {
    let g = dyad_pivots(k);
    let t = theta.to_radians();
    let mut j = vec![(LINK * t.cos(), LINK * t.sin())];
    for (i, &b) in g.iter().enumerate().skip(1) {
        let a = j[i - 1];
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let d = dx.hypot(dy);
        if d >= 2.0 * LINK {
            return None;
        }
        let h = (LINK * LINK - d * d / 4.0).sqrt();
        j.push((a.0 + dx / 2.0 - h * dy / d, a.1 + dy / 2.0 + h * dx / d));
    }
    Some(j)
}

/// A closed loop through `k` boundaries (`k` at least 2): a crank about a held pivot, then `k`
/// two-root points, each `LINK` from the joint before it and from its own held pivot, and the
/// loop closed back onto the crank by the last joint's distance from it, measured where the crank
/// stands at `DYAD_CLOSED`.  The crank is the parent's (two unknowns, its length and the closure),
/// each two-root point a boundary reading the one before it.  Flat, the closure couples the whole
/// chain into one block: no joint is placed before the crank is.
///
/// Every joint is seeded where the crank at `DYAD_SEED` puts it, in coordinates: a seed relative
/// to the joint before it is read where that joint was seeded, and down a long chain the drift
/// picks the other root (at 32 links, both solves fail from it).  Seeded alike, the two solves
/// are measured on solving, not on seeding.
pub fn dyads(k: usize) -> Document {
    assert!(k >= 2, "one link from the crank is as long wherever it turns");
    let g = dyad_pivots(k);
    let closed = dyad_joints(k, DYAD_CLOSED).expect("the loop closes");
    let seed = dyad_joints(k, DYAD_SEED).expect("the seed is in reach");
    let reach = (closed[k].0 - closed[0].0).hypot(closed[k].1 - closed[0].1);
    let mut src = format!("\
unit mm
use std

component Dyad(view: plane, a: point, b: point, sx: Length, sy: Length) {{
  in view {{
    x := point hint((sx, sy))
    x distance({LINK}mm) a
    x distance({LINK}mm) b
  }}
}}

in std.front {{
  j0 := point hint(({}, {}))
  j0 distance({LINK}mm) g0
", seed[0].0, seed[0].1);
    for (i, p) in g.iter().enumerate() {
        let _ = writeln!(src, "  g{i} := point\n  fix(({}, {})) g{i}", p.0, p.1);
    }
    src.push_str("}\n");
    let mut boundaries = Vec::new();
    for i in 1..=k {
        let from = if i == 1 { "j0".to_string() } else { format!("d{}.x", i - 1) };
        let (sx, sy) = seed[i];
        let _ = writeln!(src, "d{i} := Dyad(std.front, {from}, g{i}, sx: {sx}mm, sy: {sy}mm)");
        boundaries.push(format!("d{i}"));
    }
    let _ = writeln!(src, "d{k}.x distance({reach}mm) j0");
    Document { src, boundaries }
}

/// A line cut by a circle, the cut a boundary (`x² = u` near 0): `x` on the line `l` and on the
/// circle of radius 10 about the origin, `l` level and held at its ends' abscissae, and the parent
/// placing `l`'s height by where the cut lands: `x` at `reach` from `(0, −30)`.  The cut is at
/// height `(reach² − 1000) / 60`, and touches the circle, its two roots one, at `reach` 40.
pub fn meet(reach: f64) -> Document {
    let src = format!("\
unit mm
use std (horizontal)

component Meet(view: plane, l: line, c: circle) {{
  in view {{
    x := point hint((c.center.x + 0.6 * c.r, c.center.y + 0.8 * c.r))
    x coincident l
    x coincident c
  }}
}}

in std.front {{
  o := point
  fix((0, 0)) o
  c := circle(center: o) hint(r: 10)
  radius(10mm) c
  a := point hint((-20, 5))
  b := point hint((20, 5))
  fix(x == -20mm) a
  fix(x == 20mm) b
  a horizontal b
  l := line(a, b)
  q := point
  fix((0, -30)) q
}}
one := Meet(std.front, l, c)
one.x distance({reach}mm) q
");
    Document { src, boundaries: vec!["one".to_string()] }
}

/// `src` parsed and elaborated, or what stopped it.
pub fn read(src: &str) -> Result<Elaborated, String> {
    let (prog, errs, linked) = crate::library::parse_linked(src);
    if !errs.is_empty() || !linked.is_empty() {
        return Err(format!("does not parse: {errs:?} {linked:?}"));
    }
    let e = crate::program::elaborate(&prog);
    if !e.ok() {
        let why: Vec<String> = e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message))
            .collect();
        return Err(format!("does not elaborate: {why:?}"));
    }
    Ok(e)
}

/// The instances named, as boundaries of `e`'s drawing.
pub fn bounds(e: &Elaborated, names: &[String]) -> Result<Vec<Boundary>, String> {
    names.iter().map(|n| super::instance(e, &e.sketch, n).ok_or(format!("no instance `{n}`")))
        .collect()
}

/// A flat solve and the size of what it factors.
#[derive(Clone, Debug)]
pub struct Flat {
    pub result: SolveResult,
    /// The whole system's free columns and residual rows: what the whole-system DogLeg factors
    /// (dense to `system::DENSE_MAX` columns, the normal equations above).
    pub unknowns: usize,
    pub rows: usize,
    /// The block-triangular decomposition's blocks, and the most columns one has.
    pub blocks: usize,
    pub largest_block: usize,
}

/// `sk` solved flat under `opts`, as a document is.
pub fn flat(sk: &mut Sketch, opts: SolveOpts) -> Flat {
    let mut sys = System::new(sk);
    let order = sys.block_order();
    let (unknowns, rows) = (sys.n_free, sys.n_res);
    let largest_block = order.blocks.iter().map(|b| b.cols.len()).max().unwrap_or(0);
    let blocks = order.blocks.len();
    let result = sys.solve(sk, opts);
    Flat { result, unknowns, rows, blocks, largest_block }
}

/// A boundary's derivative at `sk`'s pose, against its own evaluations.
#[derive(Clone, Copy, Debug)]
pub struct Derivative {
    /// The largest entry of `dy/dx`, parameter units: how fast the boundary's outputs run from its
    /// inputs, which grows without bound towards a singular `F_y`.
    pub largest: f64,
    /// The worst disagreement with a central difference of two more evaluations, each input
    /// stepped by `1e-6` of its size, relative to the entry's size or 1 (`boundary_solve.rs`'s
    /// measure, DD1).
    pub error: f64,
}

/// `b`'s derivative at `sk`'s pose, every unknown against every input; an evaluation that fails
/// says why.
pub fn derivative(sk: &Sketch, b: &Boundary) -> Result<Derivative, Failure> {
    let mut bs = Boundaries::new(sk, std::slice::from_ref(b)).expect("a boundary");
    let inputs = bs.inputs(&b.name).unwrap();
    let x0: Vec<f64> = inputs.iter().map(|&p| sk.params[p as usize].value).collect();
    let at = bs.evaluate(&b.name, &x0)?;
    let mut error: f64 = 0.0;
    for j in 0..x0.len() {
        let h = 1e-6 * x0[j].abs().max(1.0);
        let mut x = x0.clone();
        x[j] = x0[j] + h;
        let up = bs.evaluate(&b.name, &x)?;
        x[j] = x0[j] - h;
        let down = bs.evaluate(&b.name, &x)?;
        for i in 0..at.y.len() {
            let fd = (up.y[i] - down.y[i]) / (2.0 * h);
            let d = at.dydx.at(i, j);
            error = error.max((fd - d).abs() / d.abs().max(1.0));
        }
    }
    Ok(Derivative { largest: absmax(&at.dydx.data), error })
}

/// `to` started where `from` ended: every parameter neither holds, by index.  For two
/// elaborations of one text that differ only in a number, whose parameters line up.
pub fn carry(from: &Sketch, to: &mut Sketch) {
    assert_eq!(from.params.len(), to.params.len(), "not one document's parameters");
    for (t, f) in to.params.iter_mut().zip(&from.params) {
        if !t.fixed && !f.fixed {
            t.value = f.value;
        }
    }
}

/// The flat solve's answer, solved: an `Err` where it does not settle.
pub fn solved(src: &str) -> Result<(Elaborated, Sketch), String> {
    let e = read(src)?;
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    if !(r.success && r.settled()) {
        return Err(format!("the flat solve does not settle: {}", r.message));
    }
    Ok((e, sk))
}
