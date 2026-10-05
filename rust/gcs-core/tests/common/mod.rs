//! What the curve tests share: a document built or the test fails saying why, the involute's
//! closed form, and the finite-difference check of a compiled system's Jacobian — what the
//! spatial tests share: a document read or refused, and the arithmetic of lines in space — and
//! what the block-solve tests and their measurements share: a solve under one `BlockMode`, a pose
//! to the bit, the corpus they are measured over, the rough starts made from a solution, and how
//! far one pose is from another.
#![allow(dead_code)]

use gcs_core::model::{EntRef, Sketch};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::rng::Rng;
use gcs_core::solve::{BlockMode, SolveOpts};
use gcs_core::space::{cross, norm, sub};
use gcs_core::system::System;

pub fn build(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>());
    e
}

/// Where the involute of the circle at `(cx, cy)` of base radius `rb` is at roll `u_deg`,
/// worked out here rather than asked of the core — so a test and the thing it tests do not
/// share an implementation.
pub fn involute_at(cx: f64, cy: f64, rb: f64, u_deg: f64) -> (f64, f64) {
    let r = u_deg.to_radians();
    (cx + rb * (r.cos() + r * r.sin()), cy + rb * (r.sin() - r * r.cos()))
}

/// The radius of curvature of the plane curve `c` at `t`, by central differences of its points
/// with step `h`: `|C'|³ / |C' × C''|` — a reference worked out here, never asked of the core.
pub fn radius_by_differences(c: impl Fn(f64) -> (f64, f64), t: f64, h: f64) -> f64 {
    let (p, q, r) = (c(t - h), c(t), c(t + h));
    let (dx, dy) = ((r.0 - p.0) / (2.0 * h), (r.1 - p.1) / (2.0 * h));
    let (ddx, ddy) = ((r.0 - 2.0 * q.0 + p.0) / (h * h), (r.1 - 2.0 * q.1 + p.1) / (h * h));
    (dx * dx + dy * dy).powf(1.5) / (dx * ddy - dy * ddx).abs()
}

/// **The Jacobian the kernels write is the system's own derivative**: every column of the
/// assembled Jacobian against a central difference of the assembled residuals, so a tape's
/// gradient, a kernel's column order and `params_on`'s are all checked at once.
pub fn fd_jacobian(sk: &Sketch, tol: f64) {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    let dense = sys.jacobian_dense(&z);
    for j in 0..z.len() {
        let h = 1e-6 * z[j].abs().max(1.0);
        let (mut lo, mut hi) = (z.clone(), z.clone());
        lo[j] -= h;
        hi[j] += h;
        let (a, b) = (sys.residuals(&lo), sys.residuals(&hi));
        for i in 0..sys.n_res {
            let fd = (b[i] - a[i]) / (2.0 * h);
            let got = dense.at(i, j);
            assert!(
                (got - fd).abs() <= tol * fd.abs().max(1.0),
                "d r{i} / d z{j}: kernel {got}, finite difference {fd}",
            );
        }
    }
}

/* -- the spatial tests' --------------------------------------------------------------------- */

/// `src` parsed and elaborated with no error, or the test fails with every diagnostic.
pub fn read(src: &str) -> Elaborated {
    let (prog, errs, linked) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty() && linked.is_empty(), "does not parse: {errs:?} {linked:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    );
    e
}

/// `src` is refused with `code` and a message holding `needle`, shown at `at`.
pub fn refused(src: &str, code: &str, needle: &str, at: &str) {
    let (prog, errs, linked) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty() && linked.is_empty(), "{errs:?} {linked:?}");
    let e = elaborate(&prog);
    let saw: Vec<String> =
        e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
    let d = e.diags.iter().find(|d| d.code.as_str() == code && d.message.contains(needle))
        .unwrap_or_else(|| panic!("expected {code} `{needle}`\n{src}\n{saw:#?}"));
    assert_eq!(d.span.slice(prog.text()), at, "{saw:?}");
}

/// The entity the document calls `n`.
pub fn ent(e: &Elaborated, n: &str) -> EntRef {
    e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"))
}

/// A line's two ends where they stand in space.
pub fn ends(sk: &Sketch, l: EntRef) -> ([f64; 3], [f64; 3]) {
    let l = &sk.lines[l.i()];
    (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
}

/// The unit vector along `a`.
pub fn unit(a: [f64; 3]) -> [f64; 3] {
    let l = norm(a);
    [a[0] / l, a[1] / l, a[2] / l]
}

/// `p`'s distance from the line through `a` along `d`.
pub fn off_line(p: [f64; 3], a: [f64; 3], d: [f64; 3]) -> f64 {
    norm(cross(sub(p, a), d)) / norm(d)
}

/* -- the block-solve tests' ----------------------------------------------------------------- */

/// The default solve, as solventc runs it, with the block pass under `blocks`.
pub fn with_blocks(blocks: BlockMode) -> SolveOpts {
    SolveOpts { blocks, ..SolveOpts::default() }
}

/// Every parameter's value, bit for bit: what "the same pose" means when a path must not move it.
pub fn bits(sk: &Sketch) -> Vec<u64> {
    sk.get_x().into_iter().map(f64::to_bits).collect()
}

/// How far `sk`'s parameters are from `reference`'s, the worst of them over the reference's
/// extent.
pub fn apart(sk: &Sketch, reference: &Sketch) -> f64 {
    sk.get_x().iter().zip(reference.get_x()).map(|(x, y)| (x - y).abs()).fold(0., f64::max)
        / reference.extent()
}

/// How far `sk`'s points (in space, so that a view turned over with its drawing is the same
/// pose) and radii are from `reference`'s, over the reference's extent.
pub fn off(sk: &Sketch, reference: &Sketch) -> f64 {
    let e = reference.extent();
    let points = (0..reference.points.len()).map(|i| {
        let (a, b) = (sk.world_point(i), reference.world_point(i));
        (0..3).map(|k| (a[k] - b[k]).abs()).fold(0., f64::max)
    });
    let radii = reference.circles.iter().map(|c| c.radius)
        .chain(reference.arcs.iter().map(|a| a.radius))
        .map(|i| (sk.params[i as usize].value - reference.params[i as usize].value).abs());
    points.chain(radii).fold(0., f64::max) / e
}

/// A document the measurements read, by the name they print it under.
pub struct Doc {
    pub name: String,
    pub sketch: Sketch,
}

/// Every example document that elaborates, as solventc reads it (`fixtures::examples`), and the
/// spiral-bevel designs.
pub fn corpus() -> Vec<Doc> {
    let mut docs = Vec::new();
    for (name, e) in fixtures::examples() {
        match e {
            Some(e) if e.ok() => docs.push(Doc { name, sketch: e.sketch }),
            _ => println!("skip {name}: does not elaborate"),
        }
    }
    for (label, configuration) in fixtures::gear::designs() {
        let sketch = fixtures::gear::unsolved(&configuration).sketch;
        docs.push(Doc { name: format!("spiral_bevel@{label}"), sketch });
    }
    docs
}

/// The kinds of rough start `rough_starts` makes, by the index it tags each with.
pub const ROUGH: [&str; 3] = ["halved", "doubled", "jittered"];

/// The rough starts the measurements solve from, made from a solution: halved and doubled about
/// its centroid (`scaled`), and jittered by up to a thousandth of the extent with three seeds
/// (`jittered`) — each tagged with its kind's index in `ROUGH` and labelled.
pub fn rough_starts(solution: &Sketch) -> Vec<(usize, String, Sketch)> {
    let mut starts =
        vec![(0, "x0.5".into(), scaled(solution, 0.5)), (1, "x2".into(), scaled(solution, 2.))];
    for seed in 1..=3 {
        starts.push((2, format!("jitter{seed}"), jittered(solution, 0.001, seed)));
    }
    starts
}

/// Scale every free point coordinate about the free points' centroid by `f`, and every free
/// radius by `f`.
pub fn scaled(sk: &Sketch, f: f64) -> Sketch {
    let mut out = sk.clone();
    let free: Vec<(u32, u32)> = sk.points.iter().map(|p| (p.x, p.y))
        .filter(|&(x, y)| !sk.params[x as usize].fixed || !sk.params[y as usize].fixed).collect();
    if free.is_empty() { return out; }
    let n = free.len() as f64;
    let cx = free.iter().map(|&(x, _)| sk.params[x as usize].value).sum::<f64>() / n;
    let cy = free.iter().map(|&(_, y)| sk.params[y as usize].value).sum::<f64>() / n;
    for &(x, y) in &free {
        for (i, c) in [(x, cx), (y, cy)] {
            let p = &mut out.params[i as usize];
            if !p.fixed { p.value = c + f * (p.value - c); }
        }
    }
    for r in sk.circles.iter().map(|c| c.radius).chain(sk.arcs.iter().map(|a| a.radius)) {
        let p = &mut out.params[r as usize];
        if !p.fixed { p.value *= f; }
    }
    out
}

/// Jitter every free point coordinate by up to `amount` of the extent, and every free radius by
/// up to `amount` of itself.
pub fn jittered(sk: &Sketch, amount: f64, seed: u32) -> Sketch {
    let mut out = sk.clone();
    let mut rng = Rng::new(seed);
    let e = sk.extent();
    for p in &sk.points {
        for i in [p.x, p.y] {
            let q = &mut out.params[i as usize];
            if !q.fixed { q.value += rng.uniform(-amount, amount) * e; }
        }
    }
    for r in sk.circles.iter().map(|c| c.radius).chain(sk.arcs.iter().map(|a| a.radius)) {
        let q = &mut out.params[r as usize];
        if !q.fixed { q.value *= 1. + rng.uniform(-amount, amount); }
    }
    out
}

/// How many blocks `sk`'s equations have in their block-triangular order.
pub fn blocks_of(sk: &Sketch) -> usize {
    System::new(sk).block_order().blocks.len()
}

/// Whether `sk` has no freedom left: no column its block order leaves under-determined.
pub fn is_determined(sk: &Sketch) -> bool {
    System::new(sk).block_order().under_cols.is_empty()
}

/// Stand plane `pi` where `b` says: its rays turned to `b`'s directions and its place moved —
/// for a test that holds a sketch and turns a plane by hand, as a solve would.
pub fn set_basis(sk: &mut Sketch, pi: usize, b: gcs_core::plane::Basis) {
    let (u, v) = (sk.planes[pi].u as usize, sk.planes[pi].v as usize);
    for (r, d) in [(u, b.u), (v, b.v)] {
        for k in 0..3 {
            let q = sk.rays[r].d[k] as usize;
            sk.params[q].value = d[k];
        }
    }
    sk.set_plane_origin(pi, b.o);
}

/// Move the whole of space rigidly: every ray turned by `turn`, and every plane's place and
/// every point in space turned and then shifted by `shift` — what a part looks like picked up
/// and set down elsewhere.
pub fn move_space(sk: &mut Sketch, turn: impl Fn([f64; 3]) -> [f64; 3], shift: [f64; 3]) {
    let moved = |x: [f64; 3]| {
        let t = turn(x);
        [t[0] + shift[0], t[1] + shift[1], t[2] + shift[2]]
    };
    for r in 0..sk.rays.len() {
        let d = sk.rays[r].d.map(|k| sk.params[k as usize].value);
        for (k, x) in turn(d).into_iter().enumerate() {
            let q = sk.rays[r].d[k] as usize;
            sk.params[q].value = x;
        }
    }
    for p in 0..sk.planes.len() {
        let o = sk.basis(p).o;
        sk.set_plane_origin(p, moved(o));
    }
    for i in 0..sk.points.len() {
        let Some(z) = sk.points[i].z else { continue };
        let ps = [sk.points[i].x, sk.points[i].y, z];
        let at = moved(ps.map(|k| sk.params[k as usize].value));
        for k in 0..3 {
            sk.params[ps[k] as usize].value = at[k];
        }
    }
}

/// `syntax::parse`, linked against the library — what a host does, so `use std` reads.
pub fn parse(src: &str) -> (gcs_core::syntax::Program, Vec<gcs_core::syntax::SynErr>) {
    let (p, errs, _) = gcs_core::library::parse_linked(src);
    (p, errs)
}

/// `syntax::parse_legacy`, linked against the library.
pub fn parse_legacy(src: &str) -> (gcs_core::syntax::Program, Vec<gcs_core::syntax::SynErr>) {
    let (mut p, errs) = gcs_core::syntax::parse_legacy(src);
    let _ = gcs_core::modules::link(&mut p, &mut gcs_core::library::resolve);
    (p, errs)
}

/// The points `use std` adds to a document: the four standard planes' origins and `std.origin`,
/// numbered after the document's own.
pub const STD_POINTS: usize = 5;

/// A 2D document drawn in the front plane: `use std`, and every statement but a `unit`, a `use`,
/// a `style` and a component's definition inside one `in std.front { … }` — what a document
/// written before points stood in space says now (`docs/planes-plan.md`).  One that already says
/// `use std` is taken as written.
pub fn front(src: &str) -> String {
    if src.lines().any(|l| l.trim() == "use std") {
        return src.to_string();   // written for the front already
    }
    let (mut head, mut body, mut defs) = (String::new(), String::new(), String::new());
    let mut depth = 0i32;
    for line in src.lines() {
        let t = line.trim_start();
        let opens = |s: &str| s.matches('{').count() as i32 - s.matches('}').count() as i32;
        if depth > 0 {
            defs.push_str(line);
            defs.push('\n');
            depth += opens(line);
        } else if t.starts_with("component ") {
            defs.push_str(line);
            defs.push('\n');
            depth = opens(line);
        } else if t.starts_with("unit ") || t.starts_with("use ") || t.starts_with("style ") {
            head.push_str(line);
            head.push('\n');
        } else {
            body.push_str(line);
            body.push('\n');
        }
    }
    let std = if head.lines().any(|l| l.trim() == "use std") { "" } else { "use std\n" };
    format!("{head}{std}{defs}in std.front {{\n{body}}}\n")
}
