//! Jansen's linkage (`jansen.sv`): the leg assembles on the pose the machine is built to, and
//! the toe's traced stride is the leg itself posed round the crank.
//!
//! The reference is worked out here and not asked of the core: each joint is where two rods of
//! stated length meet, so it is one of the two intersections of two circles, and which one is
//! what the document's `ccw`/`cw` lines state.  The document never writes an intersection down.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::examples;
use gcs_core::solve::{solve, SolveOpts};

// the holy numbers, as the document names them
const A: f64 = 38.0;
const B: f64 = 41.5;
const C: f64 = 39.3;
const D: f64 = 40.1;
const E: f64 = 55.8;
const F: f64 = 39.4;
const G: f64 = 36.7;
const H: f64 = 65.7;
const I: f64 = 49.0;
const J: f64 = 50.0;
const K: f64 = 61.9;
const L: f64 = 7.8;
const M: f64 = 15.0;

type P = (f64, f64);

/// The intersection of the circle of radius `rp` about `p` with that of `rq` about `q`, on the
/// left of p→q for `ccw` and the right otherwise — the reading `ccw(p, q, x)` takes.
fn meet(p: P, rp: f64, q: P, rq: f64, ccw: bool) -> P {
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    let d = dx.hypot(dy);
    let x = (rp * rp - rq * rq + d * d) / (2.0 * d);
    let y = (rp * rp - x * x).max(0.0).sqrt() * if ccw { 1.0 } else { -1.0 };
    let (ux, uy) = (dx / d, dy / d);
    (p.0 + x * ux - y * uy, p.1 + x * uy + y * ux)
}

/// The leg at a crank pin at page bearing `deg`, axle at the origin.
fn leg(deg: f64) -> [(&'static str, P); 8] {
    let axle = (0.0, 0.0);
    let pivot = (-A, -L);
    let r = deg.to_radians();
    let pin = (M * r.cos(), M * r.sin());
    let top = meet(pin, J, pivot, B, false);
    let knee = meet(pin, K, pivot, C, true);
    let back = meet(pivot, D, top, E, true);
    let heel = meet(back, F, knee, G, false);
    let toe = meet(heel, H, knee, I, false);
    [("axle", axle), ("pivot", pivot), ("pin", pin), ("top", top), ("back", back),
     ("knee", knee), ("heel", heel), ("toe", toe)]
}

fn close(a: P, b: P, tol: f64) -> bool {
    (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol
}

/// The drawing solves onto the built pose, the crank is its one freedom, and the pose is the
/// one every `ccw`/`cw` line states.
#[test]
fn the_leg_assembles_on_the_stated_pose() {
    let (prog, errs) = gcs_core::syntax::parse(examples::JANSEN);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under));

    let pin = e.map.ent_named("leg.pin").unwrap();
    let (px, py) = e.sketch.point_xy(pin.i());
    let want = leg(py.atan2(px).to_degrees());
    for (name, at) in want {
        let full = match name {
            "axle" | "pivot" => name.to_string(),
            _ => format!("leg.{name}"),
        };
        let p = e.map.ent_named(&full).unwrap();
        let got = e.sketch.point_xy(p.i());
        assert!(close(got, at, 1e-6), "{name}: drawn {got:?}, built {at:?}");
    }
}

/// **The traced stride is the leg.**  `path` is a locus over a scratch copy of the linkage, and
/// at every crank angle it puts the toe where the drawing's own rods would.
#[test]
fn the_stride_is_the_toe_round_the_crank() {
    let (prog, _) = gcs_core::syntax::parse(examples::JANSEN);
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok());
    assert_eq!(e.sketch.curves.len(), 1);
    // `u` is measured from the pivot-to-axle line, so the pin's page bearing is u + that
    let datum = L.atan2(A).to_degrees();
    for k in 0..24 {
        let u = 15.0 * k as f64;
        let got = e.sketch.curve_point(0, u);
        let want = leg(u + datum)[7].1;
        assert!(close(got, want, 1e-6), "at u = {u}: trace {got:?}, leg {want:?}");
    }
}
