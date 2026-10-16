//! **The paths series** (`rust/examples/paths/`, issue #363): each document solved to DOF 0 and
//! each solid held to the closed form its comments give — a coil wound on a helix drawn in space,
//! a ribbon in Frenet's frame beside one in the frame that does not twist, a pipe bent in two
//! planes with a bar riding it to the end cap's frame, a line routed through space with no plane
//! written, an S-bend whose right turn is an arc walked from its end, a cable run through corners
//! in space between two connectors, and a flange square to an inclined stub.
use gcs_core::brep::geom::V;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;
use gcs_core::traced::{reading, Need};
use std::f64::consts::PI;

use crate::common::parsed::determined;
use crate::space_paths::{integral, through3};

fn ent(e: &Elaborated, name: &str) -> gcs_core::model::EntRef {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"))
}

fn at(e: &Elaborated, name: &str) -> V {
    e.sketch.world_point(ent(e, name).i())
}

/// The exact B-rep of the solid `name`, its boundary checked.
fn exact(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let x = e.sketch.exact_solid(ent(e, name).i()).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    x
}

/// The solid `name`'s volume within `rel` of `want` (its B-rep in millimetres, one a unit here).
fn volume(e: &Elaborated, name: &str, want: f64, rel: f64) {
    let got = gcs_core::brep::props::volume(&exact(e, name).brep);
    assert!((got - want).abs() <= rel * want, "`{name}`: {got} against {want}");
}

/// The corners of the solid `name` where they stand in the drawing: its B-rep is written in
/// millimetres about `origin`.
fn corners(e: &Elaborated, name: &str) -> Vec<V> {
    let x = exact(e, name);
    x.brep.vertices.iter().map(|v| std::array::from_fn(|k| x.origin[k] + v.p[k] / x.mm)).collect()
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn unit(a: V) -> V {
    let n = dot(a, a).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

fn close(p: V, q: V, tol: f64) -> bool {
    sub(p, q).iter().all(|d| d.abs() <= tol)
}

/// The four of `corners` nearest `at`: a cap there.
fn cap(corners: &[V], at: V) -> Vec<V> {
    let mut ps = corners.to_vec();
    ps.sort_by(|p, q| dot(sub(*p, at), sub(*p, at)).total_cmp(&dot(sub(*q, at), sub(*q, at))));
    ps.truncate(4);
    ps
}

/// Each of `want` within 1e-9 of one of `got`.
fn has_corners(got: &[V], want: &[V], what: &str) {
    for w in want {
        let d = got.iter().map(|p| dot(sub(*p, *w), sub(*p, *w)).sqrt()).fold(f64::MAX, f64::min);
        assert!(d <= 1e-9, "{what}: {w:?} is {d} from the nearest of {got:?}");
    }
}

/// The helix of radius `r` rising `pitch` a turn over `turns` about `z` from `x`, at `s` of its
/// run: its point, unit tangent, principal normal (toward the axis) and binormal.
fn helix(r: f64, pitch: f64, turns: f64, s: f64) -> [V; 4] {
    let th = 2.0 * PI * turns * s;
    let p = [r * th.cos(), r * th.sin(), pitch * turns * s];
    let t = unit([-2.0 * PI * r * th.sin(), 2.0 * PI * r * th.cos(), pitch]);
    let n = [-th.cos(), -th.sin(), 0.0];
    [p, t, n, cross(t, n)]
}

/// **`coil.sv`**: a 1.5 mm square wire along three turns of a helix of radius 10 and pitch 5
/// drawn in space is its area along the helix's length.
#[test]
fn the_coil_is_its_wire_along_the_helix() {
    let e = determined(include_str!("../../examples/paths/coil.sv"));
    volume(&e, "coil", 2.25 * 3.0 * (20.0 * PI).hypot(5.0), 1e-9);
}

/// **`frenet.sv`**: both ribbons are their area along the helix; `facing`'s end caps stand in
/// Frenet's frame, its width along the principal normal; `minimal`'s width turns against it by
/// the torsion along the length, −2π·pitch/√((2π·r)² + pitch²) a turn; and the centre of
/// curvature a third of the way up is (r² + b²)/r in from the curve, b = pitch/2π.
#[test]
fn the_ribbon_faces_its_axis_in_frenets_frame() {
    let e = determined(include_str!("../../examples/paths/frenet.sv"));
    let (r, pitch, turns, w, h) = (12.0, 6.0, 3.0, 4.0, 0.5);
    let length = turns * (2.0 * PI * r).hypot(pitch);
    volume(&e, "facing", w * h * length, 1e-9);
    volume(&e, "minimal", w * h * length, 1e-9);
    let facing = corners(&e, "facing");
    for s in [0.0, 1.0] {
        let [p, _, n, b] = helix(r, pitch, turns, s);
        let want: Vec<V> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter()
            .map(|&(i, j)| std::array::from_fn(|k| p[k] + i * w / 2.0 * n[k] + j * h / 2.0 * b[k]))
            .collect();
        has_corners(&cap(&facing, p), &want, &format!("`facing`'s cap at {s}"));
    }
    // the width's bearing against the principal normal, about the tangent, at each end
    let minimal = corners(&e, "minimal");
    let bearing = |s: f64| {
        let [p, _, n, b] = helix(r, pitch, turns, s);
        let corners = cap(&minimal, p);
        let wide = corners.iter().flat_map(|a| corners.iter().map(move |c| sub(*c, *a)))
            .find(|d| (dot(*d, *d).sqrt() - w).abs() < 1e-6).expect("a side `w` long");
        dot(wide, b).atan2(dot(wide, n))
    };
    let per_turn = -2.0 * PI * pitch / (2.0 * PI * r).hypot(pitch);
    let got = (bearing(1.0) - bearing(0.0)).rem_euclid(PI);
    let want = (turns * per_turn).rem_euclid(PI);
    assert!((got - want).abs() <= 1e-9, "the minimal frame turned {got} against {want}");
    let b = pitch / (2.0 * PI);
    let rho = (r * r + b * b) / r;
    let [p, _, n, _] = helix(r, pitch, turns, 0.3);
    let centre: V = std::array::from_fn(|k| p[k] + rho * n[k]);
    assert!(close(at(&e, "here.m"), p, 1e-9), "{:?}", at(&e, "here.m"));
    assert!(close(at(&e, "here.c"), centre, 1e-9), "{:?} against {centre:?}", at(&e, "here.c"));
}

/// **`bent_pipe.sv`**: the round tube is Pappus's; the bar standing ½ off the path sweeps its
/// area along its centroid's run; and its end cap stands in the frame the closed form says, the
/// tangent +z, `u` −x and `v` −y.
#[test]
fn the_pipe_bent_in_two_planes_carries_its_frame() {
    let e = determined(include_str!("../../examples/paths/bent_pipe.sv"));
    let length = 60.0 + 20.0 * PI;
    volume(&e, "pipe", 16.0 * PI * length, 1e-9);
    volume(&e, "bar", 6.0 * (length - PI / 4.0), 1e-9);
    let g = at(&e, "g");
    assert!(close(g, [20.0, 60.0, 40.0], 1e-9), "{g:?}");
    let want: Vec<V> = [(-1.0, -1.0), (2.0, -1.0), (2.0, 1.0), (-1.0, 1.0)].iter()
        .map(|&(du, dv)| [g[0] - du, g[1] - dv, g[2]]).collect();
    has_corners(&cap(&corners(&e, "bar"), g), &want, "the bar's end cap");
}

/// **`routed_line.sv`**: three bends over points in space, each drawn in the plane through its
/// centre and its ends — square to z, x and y — and the tube along the run is Pappus's.
#[test]
fn the_line_routed_through_space_mints_its_bends_planes() {
    let e = determined(include_str!("../../examples/paths/routed_line.sv"));
    volume(&e, "tube", 16.0 * PI * (80.0 + 30.0 * PI), 1e-9);
    assert!(close(at(&e, "i"), [-20.0, 60.0, 60.0], 1e-9), "{:?}", at(&e, "i"));
    let sk = &e.sketch;
    let planes = [("k1", [0.0, 0.0, 1.0]), ("k3", [1.0, 0.0, 0.0]), ("k5", [0.0, 1.0, 0.0])];
    for (name, normal) in planes {
        let k = ent(&e, name);
        let n = sk.basis(sk.plane_of(sk.round_center(k)).expect("drawn in a plane")).normal();
        assert!((dot(unit(n), normal).abs() - 1.0).abs() <= 1e-9, "`{name}`'s plane: {n:?}");
    }
}

/// **`s_bend.sv`**: the right turn, written from `f` to `e`, is walked from `e` — the chain one
/// path, its tube Pappus's, its end where the closed form puts it — and the names still read the
/// points as written, both claims theorems.
#[test]
fn the_s_bend_walks_its_right_turn_from_its_end() {
    let mut e = determined(include_str!("../../examples/paths/s_bend.sv"));
    volume(&e, "tube", 16.0 * PI * (60.0 + 20.0 * PI), 1e-9);
    assert!(close(at(&e, "g"), [80.0, 60.0, 0.0], 1e-9), "{:?}", at(&e, "g"));
    assert_eq!(e.sketch.walked_back.len(), 1);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty(), "{d:?}");
    assert_eq!(d.claims_theorem.len(), 2);
}

/// **`cable.sv`**: the cable's control points are the cubic through the corners solved apart,
/// its ends along the connectors' axes; it stands at corner k where its parameter is k; and each
/// tube is its area along its curve's length.
#[test]
fn the_cable_runs_through_its_corners_out_of_its_connectors() {
    let e = determined(include_str!("../../examples/paths/cable.sv"));
    let corners: [V; 4] = [[0.0, 0.0, 0.0], [30.0, 15.0, 10.0], [60.0, 0.0, 25.0],
                           [80.0, -20.0, 50.0]];
    let sk = &e.sketch;
    let cable = ent(&e, "cable");
    let (&spline, _) = sk.spline_traced.iter().find(|(_, &t)| t as usize == cable.i())
        .expect("the cable is a curve traced in space");
    let got: Vec<V> = sk.splines[spline as usize].ctrl.iter()
        .map(|&p| sk.world_point(p as usize)).collect();
    let want = through3(&corners, Some([0.0, 10.0, 0.0]), Some([0.0, 0.0, 10.0]));
    assert_eq!(got.len(), want.len());
    for (g, w) in got.iter().zip(&want) {
        assert!(close(*g, *w, 1e-9), "{got:?} against {want:?}");
    }
    for (k, p) in corners.iter().enumerate() {
        let r = reading(sk, cable.i(), [k as f64, 0.0], Need::Point).unwrap();
        assert!(close(r.position, *p, 1e-9), "corner {k}: {:?}", r.position);
    }
    for (name, tube, hi) in [("cable", "cable_tube", 3.0), ("hose", "hose_tube", 1.0)] {
        let i = ent(&e, name).i();
        let length = integral(0.0, hi, |u| {
            let d = reading(sk, i, [u, 0.0], Need::Tangent).unwrap().d[0];
            dot(d, d).sqrt()
        });
        volume(&e, tube, 4.0 * PI * length, 1e-9);
    }
}

/// **`flange.sv`**: at each lean the stub is π·r²·length and the flange π·rf²·thick, every corner
/// of the flange's B-rep standing on its rim, `rf` from the axis, and from the stub's end back
/// `thick` down it.
#[test]
fn the_flange_stands_square_to_its_stub_at_any_lean() {
    let text = include_str!("../../examples/paths/flange.sv");
    for lean in ["30deg", "70deg", "90deg"] {
        let e = determined(&text.replace("lean := 30deg", &format!("lean := {lean}")));
        volume(&e, "stub", PI * 36.0 * 40.0, 1e-9);
        volume(&e, "flange", PI * 225.0 * 4.0, 1e-9);
        let (base, top) = (at(&e, "base"), at(&e, "top"));
        let axis = unit(sub(top, base));
        let flange = corners(&e, "flange");
        assert!(!flange.is_empty());
        for p in flange {
            let along = dot(sub(p, top), axis);
            let off = sub(sub(p, top), axis.map(|x| x * along));
            assert!((-4.0 - 1e-9..=1e-9).contains(&along), "{lean}: {p:?} is {along} along");
            assert!((dot(off, off).sqrt() - 15.0).abs() <= 1e-9, "{lean}: {p:?} is off the rim");
        }
    }
}
