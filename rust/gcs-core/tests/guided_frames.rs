//! **Frames along a path** (#398 C): a jointed frame whose joint is held along a path (`std.Along`,
//! `std.AlongFrenet`, each the frame `f` it makes: `g coincident f.axis`) stands where the path's parameter puts it, its pose
//! read off the path after the solve — a motion of its own kind (`MotionDef::Guided`), the
//! rotation-minimising frame integrated by double reflection from its home or the path's own
//! frame read off its derivatives.  What is drawn in it is drawn at the home, and a face drawn
//! there is swept as a declared parameter runs, `sweep(sec over s)`, standing at every station
//! where the frame does.
//!
//! Gated by stations along a spline against today's (stage 5's unbuilt gate): the tube and the
//! ribbon swept in such frames against `std.On`'s and `std.Frenet`'s families corner for corner,
//! along a spline in a plane and one in space; by the frame along a traced helix against the
//! helix's rotation-minimising frame in closed form, its rate against a difference of its pose;
//! by `MotionBounds` holding every sampled pose of every stretch and the speed bound every
//! sampled speed; and by the refusals.
use gcs_core::brep::geom::V;
use gcs_core::interval::Interval as I;
use gcs_core::motion::Family;
use gcs_core::program::Elaborated;
use gcs_core::space::{cross, norm, sub};
use std::f64::consts::PI;

use crate::common::parsed::determined;

/// A rectangle drawn in the plane `n` about its origin, and today's stations on a path: a
/// rectangle in `std.On`'s plane and in `std.Frenet`'s; the planar spline and the cubic in space
/// the frames run along; and the traced helix.
const PATHS: &str = "unit mm
use std (coords)

component Rect(n: plane, w: Length, h: Length) {
  in n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: -w / 2, dv: -h / 2) n
  b coords(du: w / 2, dv: -h / 2) n
  c coords(du: w / 2, dv: h / 2) n
  d coords(du: -w / 2, dv: h / 2) n
}

component Rider(g: path, up: axis, w: Length, h: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  sec := Rect(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Ribbon(g: path, w: Length, h: Length, s: Scalar) {
  st := std.Frenet(g, s: s)
  sec := Rect(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

in std.top {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((60mm, 0mm)) k0
  fix((60mm, 20mm)) k1
  fix((80mm, 30mm)) k2
  fix((100mm, 30mm)) k3
  curvy := spline(k0, k1, k2, k3)
}
ka := point hint((0, 0, 0))
kb := point hint((10, 20, 5))
kc := point hint((30, 20, 15))
kd := point hint((40, 0, 30))
fix((0mm, 0mm, 0mm)) ka
fix((10mm, 20mm, 5mm)) kb
fix((30mm, 20mm, 15mm)) kc
fix((40mm, 0mm, 30mm)) kd
cubic := spline(ka, kb, kc, kd)
param s: Scalar over (0, 1)
";

/// A frame `n` along `guide` by `word` (its arguments `args`), a `w` × `h` rectangle drawn in it,
/// and the rectangle swept as `s` runs: `along`.
fn along(word: &str, guide: &str, args: &str) -> String {
    format!("al := std.{word}({guide}{args})
sec := Rect(al.f, w: 2mm, h: 1mm)
f := face(sec.a, sec.b, sec.c, sec.d, -> close)
along := sweep(f over s)
")
}

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// The exact B-rep of the solid `name`, its boundary checked.
fn brep(e: &Elaborated, name: &str) -> gcs_core::brep::topo::Brep {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    x.brep.clone()
}

/// The solids `a` and `b` one solid: as many corners, each of `a`'s within 1e-9 of one of `b`'s
/// (their caps' corners, where the stations stand at the ends), and their volumes within a part
/// in 10⁹.
fn same_solid(e: &Elaborated, a: &str, b: &str, what: &str) {
    let (ba, bb) = (brep(e, a), brep(e, b));
    assert_eq!(ba.vertices.len(), bb.vertices.len(), "{what}");
    for p in &ba.vertices {
        let d = bb.vertices.iter().map(|q| norm(sub(p.p, q.p))).fold(f64::INFINITY, f64::min);
        assert!(d < 1e-9, "{what}: {:?} stands {d} from every corner of the other", p.p);
    }
    let (va, vb) = (gcs_core::brep::props::volume(&ba), gcs_core::brep::props::volume(&bb));
    assert!((va - vb).abs() <= 1e-9 * vb, "{what}: {va} against {vb}");
}

/// The motion of the frame named `name`.
fn motion(e: &Elaborated, name: &str) -> Family {
    let i = e.sketch.motions.iter().position(|m| m.name == name)
        .unwrap_or_else(|| panic!("no motion `{name}`"));
    Family::read(&e.sketch, i).unwrap()
}

/// **A tube in a frame along a spline is today's station's**: a rectangle drawn in `std.Along`'s
/// frame at its home and swept as `s` runs stands, corner for corner, where the same rectangle in
/// `std.On`'s plane does — along a spline in a plane, `up` its normal (where the frame is the one
/// holding `v` along `up`, wherever its home), and along a cubic in space, homed where `On`'s
/// family is, the middle of its interval.  Their volumes agree.
#[test]
fn a_tube_in_a_frame_along_a_spline_is_todays_stations() {
    for (guide, home) in [("curvy", "0"), ("cubic", "0.5")] {
        let today = format!("today := sweep(Rider({guide}, std.z, w: 2mm, h: 1mm).f over s)\n");
        let src = format!("{PATHS}{}{today}", along("Along", guide, &format!(", up: std.z, home: {home}")));
        same_solid(&determined(&src), "along", "today", guide);
    }
}

/// **A ribbon in the frame of a spline in space is `std.Frenet`'s**: the path's own frame, read off
/// its derivatives at every station, wherever it is homed — corner for corner and by volume.
#[test]
fn a_ribbon_in_a_frenet_frame_along_a_spline_is_todays_stations() {
    let src = format!("{PATHS}{}today := sweep(Ribbon(cubic, w: 2mm, h: 1mm).f over s)\n",
                      along("AlongFrenet", "cubic", ", home: 0.25"));
    same_solid(&determined(&src), "along", "today", "the ribbon");
}

/// **The frame along a traced helix is the helix's rotation-minimising frame**: from its home at
/// `u = 0.3`, the tangent carried to the tangent, the principal normal to the normal turned back
/// about it by the torsion over the arc run (`θ = −τ·(s − s_h)`), the home's point to the
/// helix's — each to 1e-9 — and its rate against a difference of its pose.
#[test]
fn the_frame_along_a_traced_helix_is_its_closed_form() {
    let src = "unit mm
use std

o := point hint((0, 0, 0))
fix((0, 0, 0)) o
X1 := point hint((10, 0, 0))
fix((10, 0, 0)) X1
Y1 := point hint((0, 10, 0))
fix((0, 10, 0)) Y1
Z1 := point hint((0, 0, 10))
fix((0, 0, 10)) Z1
xl := line(o, X1)
yl := line(o, Y1)
zl := line(o, Z1)
component Helix(o: point, x: line, y: line, z: line, r: Length, pitch: Length, turns: Scalar,
                u: Scalar) {
  p := point hint((r * cos(360deg * turns * u), r * sin(360deg * turns * u), pitch * turns * u))
  o distance(r * cos(360deg * turns * u), along: x) p
  o distance(r * sin(360deg * turns * u), along: y) p
  o distance(pitch * turns * u, along: z) p
}
guide := trace(Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 2).p over u in (0, 1))
al := std.Along(guide, up: std.z, home: 0.3)
";
    let e = determined(src);
    let f = motion(&e, "al.f");
    let (r, b): (f64, f64) = (10.0, 5.0 / (2.0 * PI));
    let c = r.hypot(b);
    let tau = b / (c * c);
    let phi = |u: f64| 4.0 * PI * u;
    let at = |u: f64| [r * phi(u).cos(), r * phi(u).sin(), b * phi(u)];
    let tangent = |u: f64| [-r * phi(u).sin() / c, r * phi(u).cos() / c, b / c];
    let normal = |u: f64| [-phi(u).cos(), -phi(u).sin(), 0.0];
    let binormal = |u: f64| cross(tangent(u), normal(u));
    let home = 0.3;
    for k in 0..=20 {
        let u = k as f64 / 20.0;
        let m = f.at(u).unwrap();
        let close = |got: V, want: V, what: &str| {
            assert!(norm(sub(got, want)) < 1e-9, "{what} at {u}: {got:?} against {want:?}");
        };
        close(m.point(at(home)), at(u), "the home's point");
        close(m.vector(tangent(home)), tangent(u), "the tangent");
        let theta = -tau * c * (phi(u) - phi(home));
        let want: V = std::array::from_fn(|i| theta.cos() * normal(u)[i]
            + theta.sin() * binormal(u)[i]);
        close(m.vector(normal(home)), want, "the normal");
        // its rate a unit of the parameter, against a difference of its pose
        let x = [3.0, -2.0, 7.0];
        let h = 1e-6;
        let (lo, hi) = ((u - h).max(0.0), (u + h).min(1.0));
        let fd: V = std::array::from_fn(|i| (f.at(hi).unwrap().point(x)[i]
            - f.at(lo).unwrap().point(x)[i]) / (hi - lo));
        let v = m.velocity(x);
        assert!(norm(sub(v, fd)) < 1e-5 * (1.0 + norm(v)), "the rate at {u}: {v:?} {fd:?}");
    }
    // a traced curve carries no bound on its derivatives: its stretch's bounds fail closed
    assert!(f.bounds(I::new(0.0, 0.1).unwrap()).is_err());
}

/// **`MotionBounds` holds every sampled pose of every stretch, and the speed bound every sampled
/// speed**: the frame along the cubic in space, over sixteen stretches of its parameter, a box of
/// points carried forward and back.
#[test]
fn a_frame_along_a_spline_is_bounded_over_every_stretch() {
    let e = determined(&format!("{PATHS}{}", along("Along", "cubic", ", up: std.z, home: 0.5")));
    let f = motion(&e, "al.f");
    let pts: [V; 3] = [[0.0, 0.0, 0.0], [12.0, -3.0, 4.0], [40.0, 10.0, 25.0]];
    for k in 0..16 {
        let (a, b) = (k as f64 / 16.0, (k + 1) as f64 / 16.0);
        let bounds = f.bounds(I::new(a, b).unwrap()).unwrap();
        for p in pts {
            let ip = [I::point(p[0]).unwrap(), I::point(p[1]).unwrap(), I::point(p[2]).unwrap()];
            let carried = bounds.point(ip).unwrap();
            let back = bounds.inverse_point(ip).unwrap();
            let speed = f.inverse_point_speed_bound(p, I::new(a, b).unwrap()).unwrap();
            for j in 0..=20 {
                let u = a + (b - a) * j as f64 / 20.0;
                let m = f.at(u).unwrap();
                let q = m.point(p);
                assert!((0..3).all(|i| carried[i].contains(q[i])), "{q:?} outside {carried:?}");
                let w = m.inverse().point(p);
                assert!((0..3).all(|i| back[i].contains(w[i])), "{w:?} outside {back:?}");
                let v = norm(m.inverse().velocity(p));
                assert!(v <= speed, "speed {v} past its bound {speed} over [{a}, {b}]");
            }
        }
    }
    // the path's own frame carries no bound on its torsion: it fails closed
    let e = determined(&format!("{PATHS}{}", along("AlongFrenet", "cubic", "")));
    assert!(motion(&e, "al.f").bounds(I::new(0.0, 0.1).unwrap()).is_err());
}

/// **What a frame along a path cannot mean is said**: a ratio other than 0 or 1, an advance, a
/// frame not drawn on its path, and a relation the solve holds reading its motion.
#[test]
fn a_frame_along_a_path_refuses_what_it_cannot_mean() {
    let said = |below: &str| crate::common::lenient::read(&format!("{PATHS}{below}")).1;
    let frame = "n := plane(u: hint(dir: std.x), v: hint(dir: std.z))\n";
    let s = said(&format!("{frame}cubic coincident n.axis\nfix(ratio == 0.5, advance == 0mm) n\n\
        m := point\nm coincident(t == 0) cubic\nn.origin coincident m\n"));
    let torsion = |m: &String| m.starts_with("E080") && m.contains("turns 0.5 of its path's");
    assert!(s.iter().any(torsion), "{s:?}");
    let s = said(&format!("{frame}cubic coincident n.axis\nfix(ratio == 0, advance == 2mm) n\n"));
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("no advance")), "{s:?}");
    // where the frame stands is the solve's: said once it has solved
    let mut e = crate::common::lenient::solved(&format!("{PATHS}{frame}cubic coincident n.axis\n\
        fix(ratio == 0, advance == 0mm) n\nfix(origin == (5, 5, 5)) n\n"));
    let after = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    assert!(after.iter().any(|d| d.message.contains("not drawn on its path")), "{after:?}");
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let s = said(&format!("{}X := point\nX coincident sec.a at 30deg\n",
                          along("Along", "cubic", ", up: std.z")));
    assert!(s.iter().any(|m| m.contains("rides along a path")), "{s:?}");
}
