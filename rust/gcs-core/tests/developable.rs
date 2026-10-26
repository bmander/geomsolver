//! **Developable panels** (#645): `std.Developable`, the ruled set one plane touches along each
//! ruling, and `std.Development`, its layer laid flat in a plane — the six documents of
//! `rust/examples/developable/` each held to the closed form its comments give, and the rules of
//! `docs/contract/developables.md` each as the property it states: the pattern an isometry of the
//! panel, the panel developable, the exact development and the body's own rows agreeing, the
//! branch, the fans, the closure, the refusals, the band as a solid, and the four relations the
//! words are written in.
use std::f64::consts::PI;

use gcs_core::developable::{Developable, Development, Laid, Piece};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::{CurveBody, Sketch};
use gcs_core::path::Path;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::parsed::{build, determined};

const FRUSTUM: &str = include_str!("../../examples/developable/frustum.sv");
const OFFSET: &str = include_str!("../../examples/developable/offset_adapter.sv");
const TILTED: &str = include_str!("../../examples/developable/tilted_adapter.sv");
const CUT: &str = include_str!("../../examples/developable/cut_cylinder.sv");
const SQUARE: &str = include_str!("../../examples/developable/square_to_round.sv");
const DORY: &str = include_str!("../../examples/developable/dory_panel.sv");
const OFFSET_TRANSITION: &str = include_str!("../../examples/developable/offset_transition.sv");
const HULL: &str = include_str!("../../examples/developable/dory.sv");

/// The first six, and the offset transition: one laid panel a document.
const ALL: [(&str, &str); 7] = [("frustum", FRUSTUM), ("offset", OFFSET), ("tilted", TILTED),
                                ("cut", CUT), ("square", SQUARE), ("dory", DORY),
                                ("offset transition", OFFSET_TRANSITION)];

/// The curve the document calls `name`.
fn curve(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// The development `flat` lays, as the drawing stands.
fn development(e: &Elaborated) -> Development {
    let built = e.sketch.development_of(curve(e, "flat.a")).expect("a laid rail");
    built.as_ref().clone().unwrap_or_else(|r| panic!("not laid: {r}"))
}

/// The panel `flat` lays.
fn panel(e: &Elaborated) -> Developable {
    development(e).panel().clone()
}

/// `n + 1` places evenly over a panel's running number.
fn places(p: &Developable, n: usize) -> Vec<f64> {
    let (lo, hi) = p.span();
    (0..=n).map(|k| lo + (hi - lo) * k as f64 / n as f64).collect()
}

fn at(d: &Development, s: f64) -> Laid {
    d.at(s).unwrap_or_else(|r| panic!("no image at {s}: {r}"))
}

fn apart(p: [f64; 2], q: [f64; 2]) -> f64 {
    (p[0] - q[0]).hypot(p[1] - q[1])
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

/// The rails a laid rail's panel is built between, as paths read off the drawing.
fn rails(sk: &Sketch, i: usize) -> [Path; 2] {
    let CurveBody::Developed(dev) = &sk.curve_defs[sk.curves[i].def as usize].body else {
        panic!("no laid rail");
    };
    [dev.rails[0].path(sk).unwrap(), dev.rails[1].path(sk).unwrap()]
}

/// The pattern's signed area: `a`'s image forward, `b`'s back, by the shoelace over `n` places.
fn signed_area(d: &Development, n: usize) -> f64 {
    let ss = places(d.panel(), n);
    let mut poly: Vec<[f64; 2]> = ss.iter().map(|&s| at(d, s).f).collect();
    poly.extend(ss.iter().rev().map(|&s| at(d, s).g));
    let mut a = 0.0;
    for k in 0..poly.len() {
        let (p, q) = (poly[k], poly[(k + 1) % poly.len()]);
        a += p[0] * q[1] - q[0] * p[1];
    }
    0.5 * a
}

/* -- the closed forms ------------------------------------------------------------------------ */

/// **`frustum.sv`**: the panel laid flat is the annular sector of radii `50ℓ/20` and `30ℓ/20`
/// about one centre, through `2π·20/ℓ` — and its layer `d` out, radii `5d` more, through the same
/// angle (the drawn flat is the layer 1 out).
#[test]
fn a_frustum_lays_flat_on_its_annular_sector() {
    let e = determined(FRUSTUM);
    let l = 20f64.hypot(100.0);
    let (r1, r2, sweep) = (50.0 * l / 20.0, 30.0 * l / 20.0, 2.0 * PI * 20.0 / l);
    let p = panel(&e);
    assert!(p.closed() && p.pieces().iter().all(|q| q.0 == Piece::Smooth));
    for d in [0.0, 1.0] {
        let dev = p.develop(d).unwrap();
        // the home ruling from (0, 0) along +x: the apex's image beyond its `g`, on that line
        let centre = [r1 + 5.0 * d, 0.0];
        for s in places(&p, 48) {
            let x = at(&dev, s);
            assert!(rel(apart(x.f, centre), r1 + 5.0 * d) < 1e-9, "{d} {s}: {:?}", x.f);
            assert!(rel(apart(x.g, centre), r2 + 5.0 * d) < 1e-9, "{d} {s}: {:?}", x.g);
        }
        let (lo, hi) = p.span();
        let (a, b) = (at(&dev, lo), at(&dev, hi));
        let (u, v) = (sub2(a.g, a.f), sub2(b.g, b.f));
        let turned = (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).abs();
        assert!((turned - sweep).abs() < 1e-9, "{d}: {turned} against {sweep}");
    }
}

fn sub2(p: [f64; 2], q: [f64; 2]) -> [f64; 2] {
    [p[0] - q[0], p[1] - q[1]]
}

/// **`offset_adapter.sv`**: an oblique cone — every ruling pairs equal angles on the two rims, and
/// passes through the apex `(75, 0, 250)`.
#[test]
fn an_offset_adapter_pairs_equal_angles_through_its_apex() {
    let e = determined(OFFSET);
    let p = panel(&e);
    for s in places(&p, 40) {
        let r = p.ruling(s).unwrap();
        let (ta, tb) = (r.m[1].atan2(r.m[0]), r.n[1].atan2(r.n[0] - 30.0));
        let gap = (ta - tb).sin().abs() + (1.0 - (ta - tb).cos());
        assert!(gap < 1e-9, "{s}: {ta} against {tb}");
        // the apex five halves of the way along the ruling from `m`
        let apex = [r.m[0] + 2.5 * (r.n[0] - r.m[0]), r.m[1] + 2.5 * (r.n[1] - r.m[1]),
                    r.m[2] + 2.5 * (r.n[2] - r.m[2])];
        assert!(norm(sub(apex, [75.0, 0.0, 250.0])) < 1e-8, "{s}: {apex:?}");
    }
}

/// **`tilted_adapter.sv`**: the home ruling from the seam at −40° meets the tilted rim at the outer
/// root, `(50.035, −20.281, 90.658)`, which an independent root-find of `det(a′, b′, b − a)`
/// agrees with — never the crossing root round the far side.
#[test]
fn a_tilted_adapter_takes_the_outer_root() {
    let e = determined(TILTED);
    let p = panel(&e);
    let home = p.ruling(0.0).unwrap();
    assert!(norm(sub(home.n, [50.035, -20.281, 90.658])) < 2e-3, "{:?}", home.n);
    // the condition, read independently on the tilted rim at the root found
    let [a, b] = rails(&e.sketch, curve(&e, "flat.a"));
    let ja = a.jet(home.u, false).unwrap();
    let jb = b.jet(home.w, false).unwrap();
    let r = sub(home.n, home.m);
    let f = dot(ja[1], cross(jb[1], r)) / (norm(ja[1]) * norm(jb[1]) * norm(r));
    assert!(f.abs() < 1e-9, "{f}");
    // and the branch: `b` runs through the plane the way `a` does, at every ruling
    for s in places(&p, 60) {
        let q = p.ruling(s).unwrap();
        let ja = a.jet(q.u, false).unwrap();
        let jb = b.jet(q.w, false).unwrap();
        let r = sub(q.n, q.m);
        assert!(dot(cross(ja[1], r), cross(jb[1], r)) > 0.0, "{s}");
    }
}

/// **`cut_cylinder.sv`**: the panel between the base circle and the mitre's ellipse is the
/// cylinder, laid flat as the mitre template — its bottom edge the circle unrolled straight, and
/// each generator square to it and `h + 40 tan β · cos(x/40)` long, `x` along the edge.
#[test]
fn a_cut_cylinder_unrolls_to_its_cosine() {
    let e = determined(CUT);
    let p = panel(&e);
    let dev = p.develop(0.0).unwrap();
    let base = at(&dev, 0.0);
    let up = sub2(base.g, base.f);
    let up = [up[0] / up[0].hypot(up[1]), up[1] / up[0].hypot(up[1])];
    let along = [up[1], -up[0]];
    let tb = 20f64.to_radians().tan();
    for s in places(&p, 64) {
        let x = at(&dev, s);
        let off = sub2(x.f, base.f);
        // on the bottom edge
        assert!((off[0] * up[0] + off[1] * up[1]).abs() < 1e-8, "{s}: {:?}", x.f);
        let t = off[0] * along[0] + off[1] * along[1];
        let gen = sub2(x.g, x.f);
        let want = 80.0 + 40.0 * tb * (t / 40.0).cos();
        assert!((gen[0].hypot(gen[1]) - want).abs() < 1e-7, "{s}: {:?} against {want}", gen);
        assert!((gen[0] * along[0] + gen[1] * along[1]).abs() < 1e-7, "{s}: not square");
    }
    assert!(rel(dev.length(0), 2.0 * PI * 40.0) < 1e-9);
    assert!(rel(dev.area(), 2.0 * PI * 40.0 * 80.0) < 1e-8, "{}", dev.area());
}

/// **`square_to_round.sv`**: four planar triangles and four fans about the corners — each
/// triangle's rulings in one plane, each fan's foot standing at its corner — and the pattern's
/// edges 640 and 100π long.
#[test]
fn a_square_to_round_is_triangles_and_corner_fans() {
    let e = determined(SQUARE);
    let p = panel(&e);
    let pieces = p.pieces();
    assert_eq!(pieces.iter().filter(|q| q.0 == Piece::FanA).count(), 4, "{pieces:?}");
    let corners = [[100.0, 60.0, 0.0], [-100.0, 60.0, 0.0], [-100.0, -60.0, 0.0], [100.0, -60.0, 0.0]];
    for &(kind, lo, hi) in &pieces {
        let ss: Vec<f64> = (0..=12).map(|k| lo + (hi - lo) * k as f64 / 12.0).collect();
        let first = p.ruling(ss[0]).unwrap();
        for &s in &ss {
            let r = p.ruling(s).unwrap();
            match kind {
                Piece::FanA => {
                    assert!(corners.iter().any(|c| norm(sub(r.m, *c)) < 1e-9), "{s}: {:?}", r.m);
                    assert!(norm(sub(r.m, first.m)) < 1e-9);
                }
                // along a straight side the panel is one plane, its rulings meeting one point
                _ => {
                    assert!(norm(sub(r.normal, first.normal)) < 1e-9, "{s}");
                    assert!(norm(sub(r.n, first.n)) < 1e-9, "{s}: {:?}", r.n);
                }
            }
        }
    }
    let dev = development(&e);
    assert!(rel(dev.length(0), 640.0) < 1e-9, "{}", dev.length(0));
    assert!(rel(dev.length(1), 100.0 * PI) < 1e-9, "{}", dev.length(1));
    // the triangles' areas from their closed forms are the whole less the fans'
    let triangles = 2.0 * 200.0 * 10f64.hypot(150.0) / 2.0 + 2.0 * 120.0 * 50f64.hypot(150.0) / 2.0;
    assert!(p.area() > triangles && p.area() < triangles + 0.6 * p.area());
}

/// **`dory_panel.sv`**: the sheer runs past the chine at both ends, so the chine's end points fan
/// — the stem's cone and the transom's start at the rails' ends — and the pattern lies on its
/// sheet of plywood, placed there by relations.
#[test]
fn a_dory_panel_fans_at_its_ends_and_lies_on_its_sheet() {
    let e = determined(DORY);
    let p = panel(&e);
    let pieces = p.pieces();
    let (first, last) = (pieces[0], pieces[pieces.len() - 1]);
    assert_eq!((first.0, last.0), (Piece::FanA, Piece::FanA), "{pieces:?}");
    for (piece, end) in [(first, [0.0, 0.0, 120.0]), (last, [1960.0, -320.0, 40.0])] {
        for k in 0..=8 {
            let r = p.ruling(piece.1 + (piece.2 - piece.1) * k as f64 / 8.0).unwrap();
            assert!(norm(sub(r.m, end)) < 1e-9, "{:?}", r.m);
        }
    }
    // every point of the pattern on the 2440 × 1220 board
    for name in ["flat.a", "flat.b"] {
        let i = curve(&e, name);
        for k in 0..=200 {
            let (x, y) = e.sketch.curve_point(i, k as f64 / 200.0);
            assert!((0.0..=2440.0).contains(&x) && (0.0..=1220.0).contains(&y), "{name} {k}: {x} {y}");
        }
    }
}

/// The development `flat` lays (its laid `a` named `{flat}.a`).
fn development_of(e: &Elaborated, flat: &str) -> Development {
    let built = e.sketch.development_of(curve(e, &format!("{flat}.a"))).expect("a laid rail");
    built.as_ref().clone().unwrap_or_else(|r| panic!("{flat} not laid: {r}"))
}

/// Every point of `flat`'s laid rails, sampled, inside the box `[x0, x1] × [y0, y1]` of its plane.
fn lies_within(e: &Elaborated, flat: &str, [x0, x1, y0, y1]: [f64; 4]) {
    for rail in ["a", "b"] {
        let i = curve(e, &format!("{flat}.{rail}"));
        for k in 0..=200 {
            let (x, y) = e.sketch.curve_point(i, k as f64 / 200.0);
            assert!(x >= x0 && x <= x1 && y >= y0 && y <= y1, "{flat}.{rail} {k}: {x} {y}");
        }
    }
}

/// **`offset_transition.sv`**: the round off the square's centre, so four different triangles —
/// each planar, its rulings meeting at the round's point its side's plane touches, `(40, −50,
/// 220)`, `(−40, 30, 220)`, `(40, 110, 220)`, `(120, 30, 220)` — and four different corner cones;
/// the pattern's edges 960 and 160π long, laid on its blank.
#[test]
fn an_offset_transition_has_four_triangles_of_its_own() {
    let e = determined(OFFSET_TRANSITION);
    let dev = development_of(&e, "flat");
    let p = dev.panel().clone();
    let pieces = p.pieces();
    assert_eq!(pieces.iter().filter(|q| q.0 == Piece::FanA).count(), 4, "{pieces:?}");
    let mut apexes: Vec<[f64; 3]> = Vec::new();
    for &(_, lo, hi) in pieces.iter().filter(|q| q.0 == Piece::Smooth) {
        let first = p.ruling(lo).unwrap();
        for k in 0..=10 {
            let r = p.ruling(lo + (hi - lo) * k as f64 / 10.0).unwrap();
            assert!(norm(sub(r.n, first.n)) < 1e-6, "{lo}: {:?} {:?}", r.n, first.n);
        }
        if !apexes.iter().any(|q| norm(sub(*q, first.n)) < 1e-6) {
            apexes.push(first.n);
        }
    }
    for want in [[40.0, -50.0, 220.0], [-40.0, 30.0, 220.0], [40.0, 110.0, 220.0],
                 [120.0, 30.0, 220.0]] {
        assert!(apexes.iter().any(|q| norm(sub(*q, want)) < 1e-6), "{want:?} not in {apexes:?}");
    }
    let flat = p.develop(0.0).unwrap();
    assert!(rel(flat.length(0), 960.0) < 1e-9 && rel(flat.length(1), 160.0 * PI) < 1e-9);
    lies_within(&e, "flat", [0.0, 800.0, 0.0, 800.0]);
}

/// **`dory.sv`**: a whole hull of three panels, each solved with its rails — the bottom's rulings
/// straight across, each foot the mirror image of the other; the two sides mirror images, their
/// patterns' rails and areas one; every pattern on its sheet of plywood.
#[test]
fn a_dory_is_three_panels_cut_from_two_sheets() {
    let e = determined(HULL);
    let names: Vec<&str> = e.sketch.panels.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names.len(), 3, "{names:?}");
    let bottom = development_of(&e, "bottom_flat");
    let p = bottom.panel();
    assert!(p.pieces().iter().all(|q| q.0 == Piece::Smooth));
    for s in places(p, 30) {
        let r = p.ruling(s).unwrap();
        assert!((r.m[0] - r.n[0]).abs() < 1e-7 && (r.m[1] + r.n[1]).abs() < 1e-7
                && (r.m[2] - r.n[2]).abs() < 1e-7, "{s}: {:?} {:?}", r.m, r.n);
    }
    assert!(rel(bottom.length(0), bottom.length(1)) < 1e-9);
    let (port, starboard) = (development_of(&e, "port_flat"), development_of(&e, "starboard_flat"));
    // port runs chine to sheer, starboard sheer to chine: each one's `a` the other's `b`
    assert!(rel(port.length(0), starboard.length(1)) < 1e-9);
    assert!(rel(port.length(1), starboard.length(0)) < 1e-9);
    assert!(rel(port.area(), starboard.area()) < 1e-9);
    // and each laid exactly: its rails' lengths the panel's
    for d in [&port, &starboard, &bottom] {
        let flat = d.panel().develop(0.0).unwrap();
        for which in 0..2 {
            assert!(rel(flat.length(which), d.panel().rail_length(which)) < 1e-9);
        }
    }
    for side in ["port_flat", "starboard_flat"] {
        lies_within(&e, side, [0.0, 2440.0, 0.0, 1220.0]);
    }
    lies_within(&e, "bottom_flat", [0.0, 2440.0, 1400.0, 2620.0]);
}

/* -- the rules, on every example ------------------------------------------------------------- */

/// **An isometry**: laid flat, each ruling keeps its length, each rail its length (relative 1e-9),
/// and the pattern's area is the panel's, by an independent shoelace over the images.
#[test]
fn every_pattern_is_an_isometry_of_its_panel() {
    for (name, src) in ALL {
        let e = determined(src);
        let p = panel(&e);
        let dev = p.develop(0.0).unwrap();
        for s in places(&p, 60) {
            let (r, x) = (p.ruling(s).unwrap(), at(&dev, s));
            assert!(rel(apart(x.f, x.g), norm(sub(r.n, r.m))) < 1e-9, "{name} {s}");
        }
        for which in 0..2 {
            assert!(rel(dev.length(which), p.rail_length(which)) < 1e-9,
                    "{name} {which}: {} against {}", dev.length(which), p.rail_length(which));
        }
        assert!(rel(dev.area(), p.area()) < 1e-8, "{name}: {} against {}", dev.area(), p.area());
        assert!(rel(signed_area(&dev, 4000).abs(), p.area()) < 1e-4, "{name}");
    }
}

/// **Developable**: the tangent plane is one along each ruling — the normal read at `m` from `a`'s
/// tangent, at `n` from `b`'s, agree to 1e-12 where the rails are read exactly.
#[test]
fn the_tangent_plane_is_one_along_each_ruling() {
    for (name, src) in [("frustum", FRUSTUM), ("offset", OFFSET), ("tilted", TILTED), ("dory", DORY)] {
        let e = determined(src);
        let p = panel(&e);
        let [a, b] = rails(&e.sketch, curve(&e, "flat.a"));
        for s in places(&p, 50) {
            let r = p.ruling(s).unwrap();
            if r.piece != Piece::Smooth {
                continue;
            }
            let g = sub(r.n, r.m);
            let na = cross(a.jet(r.u, false).unwrap()[1], g);
            let nb = cross(b.jet(r.w, false).unwrap()[1], g);
            let (na, nb) = (scale(na, 1.0 / norm(na)), scale(nb, 1.0 / norm(nb)));
            assert!(norm(sub(na, nb)) < 1e-12 * 1e3, "{name} {s}: {:?} {:?}", na, nb);
            assert!(norm(sub(na, r.normal)) < 1e-9, "{name} {s}");
        }
    }
}

fn scale(v: [f64; 3], k: f64) -> [f64; 3] {
    [v[0] * k, v[1] * k, v[2] * k]
}

/// **Exact agrees with generic**: the closed-form development and the march of `Development`'s
/// own rows (Newton for the rates at every evaluation) lay the four smooth panels alike.
#[test]
fn the_exact_development_agrees_with_the_bodys_rows() {
    for (name, src) in [("frustum", FRUSTUM), ("offset", OFFSET), ("tilted", TILTED), ("cut", CUT)] {
        let e = determined(src);
        let p = panel(&e);
        for d in [0.0, 1.0] {
            let (x, y) = (p.develop(d).unwrap(), p.develop_generic(d).unwrap());
            for s in places(&p, 40) {
                let (u, v) = (at(&x, s), at(&y, s));
                assert!(apart(u.f, v.f) < 1e-7 && apart(u.g, v.g) < 1e-7,
                        "{name} {d} {s}: {:?} {:?} against {:?} {:?}", u.f, u.g, v.f, v.g);
            }
        }
    }
}

/// **The two marches are deterministic**: built twice, the rulings and the images agree to the bit.
#[test]
fn a_panel_and_its_pattern_are_built_the_same_twice() {
    let read = || {
        let e = determined(SQUARE);
        let p = panel(&e);
        let dev = development(&e);
        places(&p, 30).into_iter().flat_map(|s| {
            let (r, x) = (p.ruling(s).unwrap(), at(&dev, s));
            [r.m, r.n].into_iter().flatten().chain(x.f.into_iter().chain(x.g)).collect::<Vec<_>>()
        }).collect::<Vec<f64>>()
    };
    let (a, b) = (read(), read());
    assert!(a.iter().zip(&b).all(|(x, y)| x.to_bits() == y.to_bits()));
}

/// **The panel moves with its drawing**: the frustum's lower rim at 60, the sector's radius is
/// `60ℓ/30` for the new slant.
#[test]
fn a_panel_follows_its_rims() {
    let src = FRUSTUM.replace("r1 := 50mm", "r1 := 60mm");
    assert_ne!(src, FRUSTUM);
    let e = determined(&src);
    let p = panel(&e);
    let dev = p.develop(0.0).unwrap();
    let l = 30f64.hypot(100.0);
    let r1 = 60.0 * l / 30.0;
    for s in places(&p, 12) {
        assert!(rel(apart(at(&dev, s).f, [r1, 0.0]), r1) < 1e-9);
    }
}

/* -- recognition ----------------------------------------------------------------------------- */

/// **What the body says, not its name**: a copy of `Developable`'s body written by hand is
/// recognised, and its band is the same solid as `std`'s.
#[test]
fn a_hand_written_panel_is_recognised() {
    let own = FRUSTUM.replace("panel := std.Developable(lower, upper)", "\
component Panel(a: path, b: path) := { p |
  m := point
  n := point
  T := plane
  m coincident a
  n coincident b
  T tangent(at: m) a
  T tangent(at: n, sense: same) b
  p coincident(t in (0, 1)) line(m, n)
}
panel := Panel(lower, upper)");
    assert_ne!(own, FRUSTUM, "the frustum's panel is `std.Developable(lower, upper)`");
    let (a, b) = (determined(FRUSTUM), determined(&own));
    let volume = |e: &Elaborated| {
        let i = e.map.ent_named("reducer").unwrap().i();
        e.sketch.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Report).unwrap().volume()
    };
    assert!(rel(volume(&b), volume(&a)) < 1e-12, "{} {}", volume(&b), volume(&a));
    assert_eq!(development(&b).length(0).to_bits(), development(&a).length(0).to_bits());
}

/* -- the band -------------------------------------------------------------------------------- */

/// **The sheet's solid**: the frustum's band 2 thick has the thin wall's closed form
/// `πℓ((r₁ + r₂)t + t² cos α)`, its STL a closed shell, its glTF a binary glTF; STEP is refused,
/// once, in the core's words.
#[test]
fn a_band_is_a_solid() {
    let e = determined(FRUSTUM);
    let i = e.map.ent_named("reducer").unwrap().i();
    let s = e.sketch.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Report).unwrap();
    let l = 20f64.hypot(100.0);
    let want = PI * l * (80.0 * 2.0 + 4.0 * 100.0 / l);
    assert!(rel(s.volume(), want) < 1e-4, "{} against {want}", s.volume());
    let (vertices, tris) =
        gcs_core::solid::agreement::stl_triangles(&s.stl().unwrap(), 1.0).unwrap();
    // the STL's corners welded where they are one point, to its single precision
    let mut welded = std::collections::BTreeMap::new();
    let id: Vec<usize> = vertices.iter().map(|v| {
        let key = v.map(|x| (x / 1e-4).round() as i64);
        let n = welded.len();
        *welded.entry(key).or_insert(n)
    }).collect();
    let mut edges = std::collections::BTreeMap::new();
    for t in &tris {
        for k in 0..3 {
            let (a, b) = (id[t[k] as usize], id[t[(k + 1) % 3] as usize]);
            *edges.entry((a.min(b), a.max(b))).or_insert(0usize) += 1;
        }
    }
    assert!(!tris.is_empty() && edges.values().all(|&n| n == 2), "not closed");
    let glb = gcs_core::gltf::glb(&e.sketch, &[i], 0.1);
    assert_eq!(&glb[..4], b"glTF");
    let step = gcs_core::solid::cad::recipe(&e.sketch, i).unwrap_err();
    assert!(step.contains("not yet written to STEP"), "{step}");
}

/// **Bands close**: two closed rails march once round and come back to the home ruling; a rail
/// wound twice round to the other's once does not, and is refused.
#[test]
fn a_band_comes_round_or_is_refused() {
    let e = determined(SQUARE);
    assert!(panel(&e).closed());
    let a = Path::Circle { c: [0.0; 3], u: [1.0, 0.0, 0.0], v: [0.0, 1.0, 0.0], r: 50.0 };
    let corner = |k: usize| {
        let t = PI / 4.0 + PI / 2.0 * (k % 4) as f64;
        [30.0 * t.cos(), 30.0 * t.sin(), 100.0]
    };
    let links: Vec<Path> = (0..8).map(|k| {
        let (p, q) = (corner(k), corner(k + 1));
        Path::Line { a: p, d: sub(q, p) }
    }).collect();
    let twice = Path::Chain { links, closed: true };
    let refusal = Developable::build(a, twice, [50.0, 0.0, 0.0]).unwrap_err();
    assert!(refusal.why.contains("do not come round"), "{refusal}");
}

/* -- the flat as a figure -------------------------------------------------------------------- */

/// **Three freedoms**: unplaced, the flat is a figure the diagnosis leaves three freedoms; two
/// relations place it (the documents are DOF 0); a relation reads `flat.a` as any curve of its
/// plane.
#[test]
fn the_flat_is_a_figure_placed_by_relations() {
    let unplaced = CUT.replace("flat.f coincident corner", "")
        .replace("line(flat.f, flat.g) parallel edge", "");
    let mut e = crate::common::parsed::solved(&unplaced);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 3);
    let read = format!("{CUT}\nin blank {{\n  z := point\n  z coincident(t == 0.5) flat.a\n}}\n");
    let e = determined(&read);
    let z = e.sketch.point_xy(e.map.ent_named("z").unwrap().i());
    let (x, y) = e.sketch.curve_point(curve(&e, "flat.a"), 0.5);
    assert!((z.0 - x).hypot(z.1 - y) < 1e-6, "{z:?} against {x} {y}");
}

/// **Which face is up**: the outside, by the rails' order — given the other way round, the pattern
/// is the same one turned over.
#[test]
fn the_rails_the_other_way_round_turn_the_pattern_over() {
    let e = determined(FRUSTUM);
    let p = panel(&e);
    let [a, b] = rails(&e.sketch, curve(&e, "flat.a"));
    let home = p.ruling(0.0).unwrap();
    let swapped = Developable::build(b, a, home.n).unwrap();
    let (x, y) = (p.develop(0.0).unwrap(), swapped.develop(0.0).unwrap());
    let (s1, s2) = (signed_area(&x, 400), signed_area(&y, 400));
    // `a` forward and `b` back of one, `b` forward and `a` back of the other: a mirror image
    // traverses the same loop the other way, which turns the shoelace's sign twice
    assert!(rel(s2.abs(), s1.abs()) < 1e-6 && s1 * s2 > 0.0, "{s1} {s2}");
}

/* -- refusals, said once at the use ---------------------------------------------------------- */

/// What the document's solids and patterns say after the solve: the program's diagnostics.
fn after(src: &str) -> Vec<String> {
    let (p, errs, _) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = elaborate(&p);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    solve(&mut e.sketch, SolveOpts::default());
    gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &p).into_iter()
        .map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect()
}

/// Two rims, the upper an arc of it, or `rim` and something else.
fn pair(b: &str) -> String {
    format!("unit mm
use std
in std.top {{
  c := point
  fix((0mm, 0mm)) c
  rim := circle(center: c)
  radius(50mm) rim
}}
up := plane
fix(origin == (0mm, 0mm, 100mm)) up
fix(dir == (1, 0, 0)) up.u
fix(dir == (0, 1, 0)) up.v
in up {{
  d := point
  e := point
  g := point
  fix((0mm, 0mm)) d
  fix((30mm, 0mm)) e
  fix((0mm, 30mm)) g
  bow := arc(center: d, from: e, to: g)
  ring := circle(center: d)
  radius(30mm) ring
}}
panel := std.Developable(rim, {b})
skin := {{ q | q distance(<= 1mm, side: outside) panel }}
band := solid(skin)
")
}

#[test]
fn one_rail_closed_and_one_open_is_refused_once() {
    let said = after(&pair("bow"));
    let refused: Vec<&String> = said.iter().filter(|m| m.contains("one rail is closed")).collect();
    assert_eq!(refused.len(), 1, "{said:?}");
    assert!(refused[0].starts_with("E080"));
}

#[test]
fn a_rail_that_is_no_path_is_refused() {
    let (p, _, _) = gcs_core::library::parse_linked(&pair("c"));
    let e = elaborate(&p);
    assert!(e.errors().any(|d| d.code.as_str() == "E040" && d.message.contains("no path a panel \
        runs along")), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

#[test]
fn no_root_of_the_same_sense_is_refused() {
    let src = "unit mm
use std
a0 := point hint((0, 0, 0))
a1 := point hint((100, 0, 0))
b0 := point hint((150, 0, 0))
b1 := point hint((250, 0, 20))
fix((0mm, 0mm, 0mm)) a0
fix((100mm, 0mm, 0mm)) a1
fix((150mm, 0mm, 0mm)) b0
fix((250mm, 0mm, 20mm)) b1
la := line(a0, a1)
lb := line(b0, b1)
panel := std.Developable(la, lb)
skin := { q | q distance(<= 1mm, side: outside) panel }
band := solid(skin)
";
    let said = after(src);
    assert!(said.iter().any(|m| m.starts_with("E080") && m.contains("`sense: same`")), "{said:?}");
}

/// Rulings that fold: the dory's sheer given an S in its plane, so the planes touching both rails
/// would have it run back where its tangent turns back — refused where `a` is, at each use, never
/// guessed past.
#[test]
fn folding_rulings_are_refused() {
    let folding = DORY
        .replace("fix((1550mm, -530mm, 467.5mm)) s3", "fix((1550mm, -400mm, 500mm)) s3")
        .replace("fix((2025mm, -440mm, 490mm)) s4", "fix((2025mm, -560mm, 460mm)) s4");
    assert_ne!(folding, DORY);
    let said = after(&folding);
    let folds: Vec<&String> = said.iter().filter(|m| m.contains("the rulings fold")).collect();
    // the band's, and the flat's
    assert_eq!(folds.len(), 2, "{said:?}");
    assert!(folds.iter().all(|m| m.starts_with("E080")), "{said:?}");
}

/// A ruling along a rail's tangent: the plane through `a`'s line touches the arc only where the
/// line runs on into it — refused at the home.
#[test]
fn a_ruling_tangent_to_a_rail_is_refused() {
    let a = Path::Line { a: [0.0; 3], d: [100.0, 0.0, 0.0] };
    let b = Path::Arc { c: [200.0, 0.0, 50.0], u: [0.0, 0.0, -1.0], v: [0.0, 1.0, 0.0], r: 50.0,
                        a0: -1.0, sweep: 2.0 };
    let refusal = Developable::build(a, b, [50.0, 0.0, 0.0]).unwrap_err();
    assert!(refusal.why.contains("tangent to a rail"), "{refusal}");
}

/* -- the report ------------------------------------------------------------------------------ */

/// `--where flat` and `--where panel`: the laid rails' lengths, the pattern's area and box, the
/// panel's rulings and whether it closes.
#[test]
fn the_report_reads_the_panel_and_its_pattern() {
    let e = determined(SQUARE);
    let rows: std::collections::BTreeMap<String, f64> =
        gcs_core::report::positions_where(&e.sketch, &e.map, &|n| n == "flat" || n == "panel")
            .into_iter().collect();
    assert!(rel(rows["flat.a.length"], 640.0) < 1e-9);
    assert!(rel(rows["flat.b.length"], 100.0 * PI) < 1e-9);
    assert!(rel(rows["flat.area"], rows["panel.area"]) < 1e-8);
    assert_eq!(rows["panel.closed"], 1.0);
    assert_eq!(rows["panel.fans"], 4.0);
    assert!(rows["panel.rulings"] > 8.0 && rows["flat.bounds.x1"] > rows["flat.bounds.x0"]);
}

/* -- the four relations ---------------------------------------------------------------------- */

/// **A plane touching a path**: at a point of a circle it holds the circle's tangent and the
/// point; at a point of a spline in space, the spline's tangent and the point.
#[test]
fn a_plane_touches_a_circle_and_a_curve_in_space() {
    let e = determined_but_plane("unit mm
use std
in std.top {
  c := point
  fix((0mm, 0mm)) c
  rim := circle(center: c)
  radius(50mm) rim
  m := point hint((-30, -40))
  m coincident rim
  m distance(30mm, along: std.x) c
}
T := plane
T tangent(at: m) rim
");
    let (sk, t) = (&e.sketch, e.map.ent_named("T").unwrap().i());
    let b = sk.basis(t);
    let m = sk.world_point(e.map.ent_named("m").unwrap().i());
    let o = b.o;
    assert!(dot(b.normal(), sub(m, o)).abs() < 1e-9);
    // the circle's tangent at m, square to its radius in the plane z = 0
    assert!(dot(b.normal(), [-m[1], m[0], 0.0]).abs() < 1e-9 * 50.0);
    let e = determined_but_plane("unit mm
use std
c0 := point hint((0, 0, 0))
c1 := point hint((100, 50, 20))
c2 := point hint((200, 0, 60))
c3 := point hint((300, -40, 30))
fix((0mm, 0mm, 0mm)) c0
fix((100mm, 50mm, 20mm)) c1
fix((200mm, 0mm, 60mm)) c2
fix((300mm, -40mm, 30mm)) c3
k := spline(c0, c1, c2, c3)
m := point hint((150, 20, 40))
m coincident k
c0 distance(150mm, along: std.x) m
T := plane
T tangent(at: m) k
");
    let (sk, t) = (&e.sketch, e.map.ent_named("T").unwrap().i());
    let b = sk.basis(t);
    let m = sk.world_point(e.map.ent_named("m").unwrap().i());
    let o = b.o;
    assert!(dot(b.normal(), sub(m, o)).abs() < 1e-9);
    // the spline's tangent there, by a difference of two points beside `m` on it
    let k = gcs_core::path::Path::read(sk, e.map.ent_named("k").unwrap()).unwrap();
    let u = k.nearest(m).unwrap();
    let t1 = k.jet(u, false).unwrap()[1];
    assert!(dot(b.normal(), t1).abs() < 1e-9 * norm(t1));
}

/// Solved, every freedom left the plane's: its origin in itself and its turn about the touching
/// line, which the relation leaves.
fn determined_but_plane(src: &str) -> Elaborated {
    let mut e = crate::common::parsed::solved(src);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.dof >= 1, "{}", d.dof);
    e
}

/// **A contact over an interval**: `p coincident(t in (0, 1)) line(a, b)` holds `p` on the line
/// and bounds where — a start past `a` is steered onto the segment, and a distance only past `b`
/// breaks the bound.
#[test]
fn a_contact_over_an_interval_holds_the_segment() {
    let doc = |hint: &str, d: &str| format!("unit mm
use std
a := point hint((0, 0, 100))
b := point hint((100, 0, 100))
fix((0mm, 0mm, 100mm)) a
fix((100mm, 0mm, 100mm)) b
p := point hint({hint})
p coincident(t in (0, 1)) line(a, b)
p distance({d}) a
");
    let e = determined(&doc("(-30, 0, 100)", "20mm"));
    let p = e.sketch.world_point(e.map.ent_named("p").unwrap().i());
    assert!(norm(sub(p, [20.0, 0.0, 100.0])) < 1e-9, "{p:?}");
    let mut e = build(&doc("(30, 0, 100)", "140mm"));
    solve(&mut e.sketch, SolveOpts::default());
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.bounds_violated.len(), 1);
}

/// The rate rows a clothoid's companion is written in: `rate(r over s) equal rate(p over s)` and
/// `rate(r over s) angle(rate(p over s), tl) tl`, beside or for `parallel`.
fn paced(direction: &str) -> String {
    format!("unit mm
use std
A := 50mm
component Clothoid(start: point, heading: line, A: Length, s: Length) {{
  p := point
  q := point hint((10, 0))
  k := point hint((0, 10))
  r := point hint((-5, 0))
  tl := line(p, q)
  nl := line(p, k)
  length(10mm) tl
  length(10mm) nl
  tl angle(90deg) nl
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)
  rate(q over s) distance(10mm * s / A^2, along: nl)
  rate(r over s) equal rate(p over s)
  {direction}
  initially {{
    p coincident start
    heading angle(0deg) tl
    r distance(5mm, along: heading) p
    r level(nl) p
  }}
}}
in std.front {{
  o := point
  x := point
  fix((0mm, 0mm)) o
  fix((-40mm, 0mm)) x
  straight := line(x, o)
  cl := Clothoid(o, straight, A: A, s: 0mm)
}}
transition := trace(cl.p over s in (0mm, 80mm))
paced := trace(cl.r over s in (0mm, 80mm))
in std.front {{
  z := point
  z coincident(t == 40) paced
  y := point
  y coincident(t == 40) transition
}}
")
}

/// **Rates against rates**: a point whose rate is as fast as the clothoid's and goes its way keeps
/// the step it started at — 5 behind along the heading, everywhere.
#[test]
fn a_rate_equal_to_a_rate_keeps_pace() {
    for direction in ["rate(r over s) parallel tl", "rate(r over s) angle(rate(p over s), tl) tl"] {
        let e = determined(&paced(direction));
        let pt = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
        let (z, y) = (pt("z"), pt("y"));
        assert!((y.0 - z.0 - 5.0).abs() < 1e-6 && (y.1 - z.1).abs() < 1e-6, "{direction}: {z:?} {y:?}");
    }
}

/// A rate compared with one along another formal is refused, naming the formal the body runs
/// over.
#[test]
fn a_rate_against_a_rate_of_another_formal_is_refused() {
    let src = paced("rate(r over s) parallel tl")
        .replace("rate(r over s) equal rate(p over s)", "rate(r over s) equal rate(p over u)");
    let (p, _, _) = gcs_core::library::parse_linked(&src);
    let e = elaborate(&p);
    assert!(e.errors().any(|d| d.message.contains("`u` is no formal here")),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/* -- the path reader ------------------------------------------------------------------------- */

/// **One reader of a path's jets**: a circle's, a chain's and an ellipse's first and second
/// derivatives against central differences, and a chain's kinks one-sided.
#[test]
fn the_path_reader_differentiates_what_it_reads() {
    let e = determined(CUT);
    let [base, mitre] = rails(&e.sketch, curve(&e, "flat.a"));
    let line = |a: [f64; 3], b: [f64; 3]| Path::Line { a, d: sub(b, a) };
    let chain = Path::Chain { links: vec![line([0.0; 3], [10.0, 0.0, 0.0]),
                                          line([10.0, 0.0, 0.0], [10.0, 5.0, 2.0])],
                              closed: false };
    for (name, p, at) in [("circle", &base, 0.7), ("ellipse", &mitre, 1.3), ("chain", &chain, 0.4)] {
        let (lo, hi) = p.domain();
        let u = lo + (hi - lo) * at;
        let h = 1e-6 * (hi - lo);
        let (j, jp, jm) = (p.jet(u, false).unwrap(), p.jet(u + h, false).unwrap(),
                           p.jet(u - h, false).unwrap());
        for k in 0..3 {
            let d1 = (jp[0][k] - jm[0][k]) / (2.0 * h);
            let d2 = (jp[1][k] - jm[1][k]) / (2.0 * h);
            assert!((j[1][k] - d1).abs() < 1e-5 * (1.0 + d1.abs()), "{name} {k}: {} {d1}", j[1][k]);
            assert!((j[2][k] - d2).abs() < 1e-4 * (1.0 + d2.abs()), "{name} {k}: {} {d2}", j[2][k]);
        }
    }
    assert_eq!(chain.kinks(), vec![1.0]);
    let before = chain.jet_side(1.0, false, gcs_core::path::Side::Before).unwrap()[1];
    let after = chain.jet_side(1.0, false, gcs_core::path::Side::After).unwrap()[1];
    assert!(norm(sub(before, [10.0, 0.0, 0.0])) < 1e-12 && norm(sub(after, [0.0, 5.0, 2.0])) < 1e-12);
}
