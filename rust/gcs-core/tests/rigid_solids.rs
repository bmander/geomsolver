//! **A solid that has only moved is the solid it was** (#412): each moving part of the V-twin is
//! drawn in its own frame and is rigid in it, so stood at another crank angle it is the same
//! evaluation under a new placement — never built again — and that placement stands it exactly
//! where a fresh evaluation would.
use std::rc::Rc;

use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solid::{ApproximationPolicy, EvaluatedSolid, LocalPoint};
use gcs_core::{examples, follow, overview, solve};

/// The V-twin as written, and solved.
fn vtwin() -> (Elaborated, Sketch) {
    let e = crate::common::read(examples::source("vtwin").unwrap());
    let mut sk = e.sketch.clone();
    let r = solve::solve(&mut sk, Default::default());
    assert!(r.success, "{}", r.message);
    (e, sk)
}

/// The index of what the V-twin calls `name`.
fn named(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("{name}")).i()
}

/// Each object's evaluation, as the box asks for it.
fn evaluated(sk: &Sketch) -> Vec<(usize, Rc<EvaluatedSolid>)> {
    overview::objects(sk).into_iter().map(|i| {
        let s = sk.evaluated_solid(i, ApproximationPolicy::Mesh)
            .unwrap_or_else(|e| panic!("{}: {e}", sk.solid_name(i)));
        (i, s)
    }).collect()
}

/// Solid `i` evaluated afresh where `sk` stands, nothing remembered.
fn fresh(sk: &Sketch, i: usize) -> Rc<EvaluatedSolid> {
    let copy = sk.clone();
    copy.caches.clear();
    copy.evaluated_solid(i, ApproximationPolicy::Mesh).unwrap()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|k| a[k] - b[k])
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| a[k] * b[k]).sum()
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// How far `p` is from the triangle `t` (Ericson's closest point, by regions).
fn to_triangle(p: [f64; 3], t: [[f64; 3]; 3]) -> f64 {
    let [a, b, c] = t;
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let at = |s: f64, u: f64| -> f64 {
        norm(sub(p, std::array::from_fn(|k| a[k] + s * ab[k] + u * ac[k])))
    };
    if d1 <= 0.0 && d2 <= 0.0 { return norm(ap); }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 { return norm(bp); }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 { return at(d1 / (d1 - d3), 0.0); }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 { return norm(cp); }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 { return at(0.0, d2 / (d2 - d6)); }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return norm(sub(p, std::array::from_fn(|k| b[k] + w * (c[k] - b[k]))));
    }
    let den = 1.0 / (va + vb + vc);
    at(vb * den, vc * den)
}

/// The moved solid is the fresh one: the same volume, faces, bounds and round features, and its
/// boundary on the fresh one's, each within the two meshes' sag.
fn agrees(name: &str, moved: &EvaluatedSolid, fresh: &EvaluatedSolid) {
    let sag = moved.sagitta() + fresh.sagitta();
    let (vm, vf) = (moved.volume(), fresh.volume());
    assert!((vm - vf).abs() <= 1e-6 * vf.abs(), "{name}: volume {vm} != {vf}");
    assert_eq!(moved.surviving_faces(), fresh.surviving_faces(), "{name}");
    let (bm, bf) = (moved.world_bounds(), fresh.world_bounds());
    for k in 0..3 {
        let off = (bm.lo[k] - bf.lo[k]).abs().max((bm.hi[k] - bf.hi[k]).abs());
        assert!(off <= sag + 1e-9, "{name}: bounds {bm:?} != {bf:?}");
    }
    let (rm, rf) = (moved.round_features(), fresh.round_features());
    assert_eq!(rm.len(), rf.len(), "{name}");
    for (a, b) in rm.iter().zip(rf) {
        let (ca, cb) = (moved.to_world(a.center).0, fresh.to_world(b.center).0);
        assert!(norm(sub(ca, cb)) < 1e-9, "{name}: round feature at {ca:?} != {cb:?}");
        let (na, nb) = (moved.vector_to_world(a.normal), fresh.vector_to_world(b.normal));
        assert!(dot(na, nb) > 1.0 - 1e-12, "{name}: round feature along {na:?} != {nb:?}");
        assert_eq!((a.of.as_str(), a.radius), (b.of.as_str(), b.radius), "{name}");
    }
    let tris: Vec<[[f64; 3]; 3]> = fresh.world_boundary().iter()
        .flat_map(|p| (1..p.pts.len() - 1).map(move |k| [p.pts[0], p.pts[k], p.pts[k + 1]]))
        .collect();
    let pts: Vec<[f64; 3]> = moved.world_boundary().iter().flat_map(|p| p.pts.clone()).collect();
    let step = (pts.len() / 150).max(1);
    for &p in pts.iter().step_by(step) {
        let d = tris.iter().map(|&t| to_triangle(p, t)).fold(f64::INFINITY, f64::min);
        assert!(d <= sag + 1e-9, "{name}: a moved boundary point {p:?} is {d} off the fresh one");
    }
}

/// **The gate** (#412): stood at successive crank angles, the disc's, the cylinders' and the
/// pistons' evaluations are the ones built at home, moved — and each moved one is a fresh
/// evaluation where it now stands.  The plate stands still and is the very solid it was.
#[test]
fn the_vtwin_moves_its_parts_rather_than_building_them_again() {
    let (_, home) = vtwin();
    let first = evaluated(&home);
    let name = |i: usize| home.solid_name(i);
    let moving = |i: usize| ["disc", "cyl", "pis"].iter().any(|p| name(i).contains(p));
    assert!(first.iter().filter(|(i, _)| moving(*i)).count() >= 5,
            "{:?}", first.iter().map(|(i, _)| name(*i)).collect::<Vec<_>>());
    assert!(first.iter().any(|(i, _)| name(*i).starts_with("plate")));
    let mut sk = home.clone();
    for t in [40.0, 115.0, 200.0, 290.0] {
        follow::stand(&mut sk, "t", t).expect("stood");
        for (i, was) in &first {
            let now = sk.evaluated_solid(*i, ApproximationPolicy::Mesh).unwrap();
            assert!(now.same_shape(was), "`{}` at {t}: built again", name(*i));
            if moving(*i) {
                assert!(!Rc::ptr_eq(&now, was), "`{}` at {t}: did not move", name(*i));
                agrees(&format!("`{}` at {t}", name(*i)), &now, &fresh(&sk, *i));
            } else {
                assert!(Rc::ptr_eq(&now, was), "`{}` at {t}: the plate stands still", name(*i));
            }
            // asked again where it stands, it is the one answer
            assert!(Rc::ptr_eq(&now, &sk.evaluated_solid(*i, ApproximationPolicy::Mesh).unwrap()));
        }
    }
    // and home again is where it was built
    follow::home(&mut sk);
    for (i, was) in &first {
        assert!(Rc::ptr_eq(&sk.evaluated_solid(*i, ApproximationPolicy::Mesh).unwrap(), was));
    }
}

/// A moved solid's world outputs apply its turn: its mesh in the world is its own mesh placed by
/// its matrix, and an STL of it is where a fresh one's is.
#[test]
fn a_moved_solid_hands_out_the_world_through_its_placement() {
    let (e, home) = vtwin();
    let disc = named(&e, "crank.disc.body");
    let was = home.evaluated_solid(disc, ApproximationPolicy::Mesh).unwrap();
    let mut sk = home.clone();
    follow::stand(&mut sk, "t", 90.0).unwrap();
    let now = sk.evaluated_solid(disc, ApproximationPolicy::Mesh).unwrap();
    assert!(now.same_shape(&was) && !Rc::ptr_eq(&now, &was));
    let r = now.rotation();
    assert!((r[0][0] - 1.0).abs() > 0.5, "a quarter turn: {r:?}");
    let m = now.matrix();
    let world = now.world_mesh();
    let own = now.mesh();
    for (p, w) in own.positions.chunks_exact(3).zip(world.positions.chunks_exact(3)) {
        let placed: [f64; 3] =
            std::array::from_fn(|k| m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k]);
        assert!(norm(sub(placed, [w[0], w[1], w[2]])) < 1e-9);
        let back = now.to_local(now.to_world(LocalPoint([p[0], p[1], p[2]])));
        assert!(norm(sub(back.0, [p[0], p[1], p[2]])) < 1e-9);
    }
    let stl = |s: &EvaluatedSolid| {
        let b = s.stl().unwrap();
        let mut lo = [f64::INFINITY; 3];
        for t in b[84..].chunks_exact(50) {
            for v in 0..3 {
                for k in 0..3 {
                    let at = 12 + 12 * v + 4 * k;
                    lo[k] = lo[k].min(f32::from_le_bytes(t[at..at + 4].try_into().unwrap()) as f64);
                }
            }
        }
        lo
    };
    let (a, b) = (stl(&now), stl(&fresh(&sk, disc)));
    assert!(norm(sub(a, b)) <= 2.0 * now.sagitta() + 1e-4, "{a:?} != {b:?}");
}

/// A solid whose shape changes is built again: the disc's rim made larger is a new evaluation,
/// where the same rim turned with the crank was not.
#[test]
fn a_solid_whose_shape_changes_is_built_again() {
    let (e, mut sk) = vtwin();
    let disc = named(&e, "crank.disc.body");
    let was = sk.evaluated_solid(disc, ApproximationPolicy::Mesh).unwrap();
    let rim = named(&e, "crank.disc.rim");
    sk.params[sk.circles[rim].radius as usize].value += 1.0;
    let now = sk.evaluated_solid(disc, ApproximationPolicy::Mesh).unwrap();
    assert!(!now.same_shape(&was), "a larger rim is a new shape");
    assert!(now.volume() > was.volume());
}

/// **A stand solves only what the parameter reaches** (#498): stood round its crank, the V-twin's
/// plate and the pistons' own numbers, drawn in their frames, are where they stood at home to the
/// bit — not settled again to the solve's precision — so the shape keys the gate reads compare
/// them exactly.
#[test]
fn a_stand_leaves_what_the_crank_does_not_reach_to_the_bit() {
    let (_, home) = vtwin();
    let still = |i: &usize| {
        ["plate.", "bankL.pis.", "bankR.pis."].iter().any(|p| home.params[*i].name.starts_with(p))
    };
    let held: Vec<usize> = (0..home.params.len()).filter(still).collect();
    assert!(held.len() > 400, "{}", held.len());
    let mut sk = home.clone();
    for t in [40.0, 115.0, 200.0, 290.0] {
        follow::stand(&mut sk, "t", t).expect("stood");
        for &i in &held {
            let (was, now) = (home.params[i].value, sk.params[i].value);
            let name = &home.params[i].name;
            assert!(was.to_bits() == now.to_bits(), "`{name}` at {t}: {was} != {now}");
        }
        // and what the crank does reach has moved
        assert!(home.get_x().iter().zip(sk.get_x()).any(|(a, b)| (a - b).abs() > 1.0), "at {t}");
    }
}

/// A part with a dimension that reads the parameter is built again where it is stood: the disc
/// whose radius is `r` is a new shape at each instant, and the peg beside it, which reads nothing
/// that runs, is the very evaluation it was (#498).
#[test]
fn a_solid_whose_dimension_reads_the_parameter_is_built_again_where_stood() {
    let e = crate::common::read("unit mm\nuse std\nparam r: Length over (10mm, 20mm)\n\
        in std.front {\n  o := point\n  fix((0, 0)) o\n  \
        rim := circle(center: o) hint(r: 10)\n  radius(r) rim\n  \
        p := point\n  fix((40, 0)) p\n  side := circle(center: p) hint(r: 5)\n  \
        radius(5mm) side\n}\ndisc := solid(face(rim), depth: 5mm)\n\
        peg := solid(face(side), depth: 5mm)\n");
    let mut sk = e.sketch.clone();
    assert!(solve::solve(&mut sk, Default::default()).success);
    let (disc, peg) = (named(&e, "disc"), named(&e, "peg"));
    let at = |sk: &Sketch, i: usize| sk.evaluated_solid(i, ApproximationPolicy::Mesh).unwrap();
    let (mut was, pegged) = (at(&sk, disc), at(&sk, peg));
    for r in [10.5, 15.0, 20.0] {
        follow::stand(&mut sk, "r", r).expect("stood");
        let now = at(&sk, disc);
        assert!(!now.same_shape(&was), "the disc at {r} is a new shape");
        let area = std::f64::consts::PI * r * r * 5.0;
        assert!((now.volume() - area).abs() < 0.02 * area, "{} at {r}", now.volume());
        assert!(Rc::ptr_eq(&at(&sk, peg), &pegged), "the peg at {r} stands still");
        was = now;
    }
}
