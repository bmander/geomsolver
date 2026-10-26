//! **A dimension that reads the body's own points** (#378): in a body traced in space a dimension
//! may read a coordinate of a point the body declares (`p.x`).  It is lowered to a number the body
//! places, held to its expression by a row of its own (`traced::RowKind::Number`) and read by the
//! free twin where a tape over the formals would be — so it is an ordinary unknown of the body:
//! solved with the rest at a place, integrated by its differentiated row over one formal, placed
//! at each node over two, and in every derivative the integrators read (the rates' system, the
//! variational equations, the Taylor orders).  `rate_examples.rs` holds the fronts it was asked
//! for, Maxwell's fish-eye; here are its pieces, each against a closed form and its contacts'
//! Jacobians against differences.
use crate::common::{build, parse, solved};
use gcs_core::model::Sketch;
use gcs_core::program::elaborate;
use gcs_core::traced::Need;

/// Four points fixed at the origin and a unit along each axis, and the lines along the axes, for a
/// component written over them.
const AXES: &str = "\
c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1
xa := line(c, x1)
ya := line(c, y1)
za := line(c, z1)
";

/// Every column of the sketch free and moved off the solution.
fn perturbed(sk: &mut Sketch) {
    let mut rng = gcs_core::rng::Rng::new(7);
    for p in sk.params.iter_mut() {
        p.fixed = false;
        p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
    }
}

/// Off the solution, each contact's Jacobian (a rate of one too) against central differences.
fn jacobians_hold(sk: &mut Sketch) {
    perturbed(sk);
    for c in sk.constraints.iter()
        .filter(|c| c.kind.traced_contact().is_some() || c.rate_source().is_some()) {
        gcs_core::fdcheck::check_constraint(sk, c, 1e-6, 1e-6)
            .unwrap_or_else(|why| panic!("{:?}: {why}", c.kind));
    }
}

/// **A speed that reads its own point, over one formal** (`rates.rs`): a point running along a
/// line from the centre at `x / L` a unit of `s`, `x` how far along the line it is, is
/// `x₀ e^{s/L}`.  The number is a coordinate of the curve's state, integrated through its row
/// differentiated, one more row in the rates' system; a point held 30 along the line is found
/// where the closed form is, `L ln 3`, and its contact's Jacobian — through the flow's
/// sensitivities to the centre and the line the speed reads — is its residual's.  (The line is
/// off the axes: a space `parallel` held along one loses rank in the one-formal integrator,
/// rates or no — D4's finding 1.)
#[test]
fn a_speed_that_reads_its_point_grows_exponentially() {
    let src = format!("unit mm
use std

component Grow(c: point, x1: point, y1: point, z1: point, d1: point, L: Length, s: Length) {{
  xa := line(c, x1)
  ya := line(c, y1)
  za := line(c, z1)
  dl := line(c, d1)
  p := point hint((8, 4, 3))
  rate(p over s) parallel dl
  rate(p over s) distance(((p.x - c.x) * 6 + (p.y - c.y) * 3 + (p.z - c.z) * 2) / 7 / L,
                          along: dl)
  initially {{
    c distance(60mm / 7, along: xa) p
    c distance(30mm / 7, along: ya) p
    c distance(20mm / 7, along: za) p
  }}
}}

{AXES}
d1 := point hint((6, 3, 2))
fix((6, 3, 2)) d1
dl := line(c, d1)
g := trace(Grow(c, x1, y1, z1, d1, L: 20mm).p over s in (0mm, 30mm))
h := point hint((26, 13, 9))
h coincident g
c distance(30mm, along: dl) h
");
    let e = build(&src);
    let mut sk = solved(&e);
    let g = e.map.ent_named("g").unwrap().i();
    for k in 0..=30 {
        let s = k as f64;
        let r = gcs_core::traced::reading(&sk, g, [s, 0.0], Need::Tangent).unwrap();
        let x = 10.0 * (s / 20.0).exp();
        let along = [6.0 / 7.0, 3.0 / 7.0, 2.0 / 7.0];
        assert!((0..3).all(|i| (r.position[i] - x * along[i]).abs() < 1e-10 * x
                               && (r.d[0][i] - x / 20.0 * along[i]).abs() < 1e-10 * x),
                "at {s}: {:?} {:?} against {x}", r.position, r.d[0]);
    }
    assert_eq!(gcs_core::report::traced_text(&sk, g),
               "g: a curve traced in space, integrated from its home at 0 across (0, 30)");
    let contact = sk.constraints.iter()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnTraced).unwrap().clone();
    let place = contact.local_values(&sk)[3];
    assert!((place - 20.0 * 3f64.ln()).abs() < 1e-9, "found at {place}");
    jacobians_hold(&mut sk);
}

/// **A body with no rate that reads its own point**: solved at each place with the rest, its
/// derivatives the implicit function's through the number's row over jets.  A circle of 10 about
/// the z axis lifted by `x·y/20`, the saddle curve `z = 2.5 sin 2u`: its point and tangent, a line
/// along its tangent where `x` is 5 (u = 60°), and both contacts' Jacobians.
#[test]
fn a_body_with_no_rate_reads_its_point() {
    let src = format!("unit mm
use std

component Saddle(c: point, x1: point, y1: point, z1: point, u: Angle) {{
  xa := line(c, x1)
  ya := line(c, y1)
  za := line(c, z1)
  p := point hint((10 * cos(u), 10 * sin(u), 0))
  c distance(10mm * cos(u), along: xa) p
  c distance(10mm * sin(u), along: ya) p
  c distance((p.x - c.x) * (p.y - c.y) / 20mm, along: za) p
}}

{AXES}
w := trace(Saddle(c, x1, y1, z1).p over u in (0deg, 360deg))
h := point hint((5, 8, 2))
h coincident w
c distance(5mm, along: xa) h
h2 := point hint((0, 13, 1))
tl := line(h, h2)
w tangent(at: h) tl
h distance(10mm) h2
");
    let e = build(&src);
    let mut sk = solved(&e);
    let w = e.map.ent_named("w").unwrap().i();
    let deg = std::f64::consts::PI / 180.0;
    let closed = |u: f64| {
        let t = u * deg;
        ([10.0 * t.cos(), 10.0 * t.sin(), 2.5 * (2.0 * t).sin()],
         [-10.0 * t.sin() * deg, 10.0 * t.cos() * deg, 5.0 * (2.0 * t).cos() * deg])
    };
    for k in 0..=24 {
        let u = 15.0 * k as f64;
        let r = gcs_core::traced::reading(&sk, w, [u, 0.0], Need::Tangent).unwrap();
        let (x, t) = closed(u);
        assert!((0..3).all(|i| (r.position[i] - x[i]).abs() < 1e-10
                                && (r.d[0][i] - t[i]).abs() < 1e-12),
                "at {u}: {:?} {:?} against {x:?} {t:?}", r.position, r.d[0]);
    }
    let at = |n: &str| sk.world_point(e.map.ent_named(n).unwrap().i());
    let (h, h2) = (at("h"), at("h2"));
    let (x, t) = closed(60.0);
    assert!((0..3).all(|i| (h[i] - x[i]).abs() < 1e-9), "{h:?} against {x:?}");
    let l = (0..3).map(|i| t[i] * t[i]).sum::<f64>().sqrt();
    let cos = (0..3).map(|i| (h2[i] - h[i]) / 10.0 * t[i] / l).sum::<f64>();
    assert!((cos.abs() - 1.0).abs() < 1e-9, "the tangent at {cos}");
    jacobians_hold(&mut sk);
}

/// `fisheye_fronts.sv`'s body over a short stretch: a point held on the x axis 10 inside the lens
/// from the source is on the front at the optical length the closed form gives it.
const SHORT: &str = "\
h := point hint((-40, 0, 1))
h coincident wave
c distance(-40mm, along: xa) h
c level(za) h
";

/// The fish-eye's front over a short stretch and a narrow arc, `xa` and `za` drawn.
fn short_front() -> String {
    include_str!("../../examples/rates/fisheye_fronts.sv")
        .replace("s in (2mm, 160mm), u in (-120deg, 120deg)",
                 "s in (2mm, 20mm), u in (-30deg, 30deg)")
        + "xa := line(c, x1)\nza := line(c, z1)\n" + SHORT
}

/// **A speed that reads its own point, over two formals** (`lines.rs`): the number is placed at
/// every node with the points the rates do not move, and the variational equations carry it.  A
/// point held on the fish-eye's front where the ray along the axis is 40 from the centre is found
/// at `s = R acos(2R·40 / (R² + 40²))`, on the ray (u = 0); its contact's Jacobian in its point,
/// its place, the centre the index reads and an axis — each a march of its own — against
/// central differences.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 18 s: a front marched")]
fn a_front_holds_a_point_through_its_speed() {
    let e = build(&short_front());
    let mut sk = solved(&e);
    let contact = sk.constraints.iter().rev()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnTraced).unwrap().clone();
    let place: Vec<f64> = contact.local_values(&sk)[3..5].to_vec();
    let sigma = 50.0 * (4000.0f64 / 4100.0).acos();
    assert!((place[0] - sigma).abs() < 1e-7 && place[1].abs() < 1e-6, "{place:?} against {sigma}");
    perturbed(&mut sk);
    let v = contact.local_values(&sk);
    let got = contact.jacobian(&sk, &v);
    // the lead, the place, then the centre's x (which the index reads) and the last axis's end
    let lead = v.len() - 2 - 12;
    for k in (0..lead + 2).chain([lead + 2, lead + 11]) {
        let h = 1e-4 * v[k].abs().max(1.0);
        let (mut lo, mut hi) = (v.clone(), v.clone());
        lo[k] -= h;
        hi[k] += h;
        let (a, b) = (contact.residual(&sk, &lo), contact.residual(&sk, &hi));
        for r in 0..contact.n_residuals() {
            let fd = (b[r] - a[r]) / (2.0 * h);
            let j = got[r * v.len() + k];
            assert!((fd - j).abs() < 1e-6 * (1.0 + j.abs()),
                    "row {r}, column {k}: {j} against {fd}");
        }
    }
}

/// The errors a document elaborates to.
fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    if !errs.is_empty() {
        return errs.into_iter().map(|e| e.message).collect();
    }
    elaborate(&prog).errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

/// **What a marched body refuses of a number that reads its points**: one in `initially`, which
/// is the Cauchy data the points start from, and one in a row that places only the points the
/// rates move, which over two formals is no row of any set (it would be placed at each step and
/// differentiated along the march at once).
#[test]
fn what_a_marched_body_refuses_of_its_numbers() {
    let doc = include_str!("../../examples/rates/fisheye_fronts.sv");
    let cases = [
        ("    c distance(0mm, along: ya) p\n", "    c distance((p.x - c.x) * 0, along: ya) p\n",
         "E103 `initially { … }` is a marched body's Cauchy data"),
        ("  c distance(0mm, along: ya) k ", "  c distance((p.x - c.x) / 1000, along: ya) p\n  \
          c distance(0mm, along: ya) k ",
         "E103 this relation's dimension reads the points the rates move, and it places only \
          them"),
    ];
    for (old, new, want) in cases {
        assert!(doc.contains(old), "{old}");
        let errs = errors(&doc.replace(old, new));
        assert!(errs.iter().any(|e| e.starts_with(want)), "{want}: {errs:?}");
    }
}
