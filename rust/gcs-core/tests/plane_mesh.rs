//! **A region meshed for analysis** (#510 S2, `delaunay::plane`): quadratic triangles over a
//! region of the plane bounded by curves, by Delaunay refinement — every angle over 20.7°, the
//! boundary's nodes on its curves, the elements curved as the sides are and covering the region's
//! area, the sides' segments in order with the material on their left, the mesh the same run to
//! run.  The rules are `docs/contract/analysis.md`'s.
use gcs_core::delaunay::plane::{mesh, Criteria, Mesh, Region, Side};
use gcs_core::interior::Tri6;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// A straight side from `a` to `b`.
fn line<'a>(a: [f64; 2], b: [f64; 2]) -> Side<'a> {
    Side { at: Box::new(move |s| [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])]),
           straight: true, cuts: Vec::new() }
}

/// An arc about `c` of radius `r` from bearing `from`, turning `sweep`.
fn arc<'a>(c: [f64; 2], r: f64, from: f64, sweep: f64) -> Side<'a> {
    Side { at: Box::new(move |s| {
        let a = from + s * sweep;
        [c[0] + r * a.cos(), c[1] + r * a.sin()]
    }), straight: false, cuts: Vec::new() }
}

fn criteria(size: f64) -> Criteria {
    Criteria { ratio: 2f64.sqrt(), size, turn: PI / 24.0, most: 200_000 }
}

/// Kirsch's quarter: a plate `w` by `h` with a quarter hole of radius `r` at its corner.
fn quarter<'a>(r: f64, w: f64, h: f64) -> Region<'a> {
    Region { loops: vec![vec![
        line([r, 0.0], [w, 0.0]), line([w, 0.0], [w, h]), line([w, h], [0.0, h]),
        line([0.0, h], [0.0, r]), arc([0.0, 0.0], r, FRAC_PI_2, -FRAC_PI_2),
    ]], points: Vec::new() }
}

/// A square with a round hole: the hole a closed side alone in its loop, run either way.
fn holed<'a>(a: f64, r: f64, sense: f64) -> Region<'a> {
    Region { loops: vec![
        vec![line([0.0, 0.0], [a, 0.0]), line([a, 0.0], [a, a]), line([a, a], [0.0, a]),
             line([0.0, a], [0.0, 0.0])],
        vec![arc([a / 2.0, a / 2.0], r, 0.0, sense * TAU)],
    ], points: Vec::new() }
}

/// The least angle of a triangle, in degrees.
fn least_angle(p: [[f64; 2]; 3]) -> f64 {
    (0..3).map(|k| {
        let (a, b, c) = (p[k], p[(k + 1) % 3], p[(k + 2) % 3]);
        let (u, v) = ([b[0] - a[0], b[1] - a[1]], [c[0] - a[0], c[1] - a[1]]);
        (u[0] * v[1] - u[1] * v[0]).abs().atan2(u[0] * v[0] + u[1] * v[1]).to_degrees()
    }).fold(180.0, f64::min)
}

/// What a mesh is held to: every corner triangle counter-clockwise and of a least angle over
/// 20.7° (√2's bound), but where two sides of the region meet at less; every inner side of an
/// element on two elements and every boundary side on one, the boundary exactly the sides'
/// segments, each with an element on its left; the elements' areas summing to `area`; and every
/// element valid, its `det J` positive.  A parabola through three points of a circle 7.5° apart
/// stands off it by a few parts in a hundred million of the radius, so the area is held to 1e-7.
fn holds(m: &Mesh, area: f64, corner: f64) {
    let el = |n: [usize; 6]| Tri6 { n, e: 1.0, nu: 0.3, rho: 1.0, depth: 1.0, strain: false };
    let mut sides: std::collections::BTreeMap<(usize, usize), Vec<usize>> = Default::default();
    let mut total = 0.0;
    for (t, n) in m.tris.iter().enumerate() {
        let p = n.map(|i| m.points[i]);
        let corners = [p[0], p[1], p[2]];
        let o = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]);
        assert!(o > 0.0, "element {t} is turned over");
        let least = least_angle(corners);
        assert!(least >= 20.7 || least >= corner - 1e-9, "element {t}: least angle {least}°");
        let e = el(*n);
        assert!(e.stiffness(&p).is_ok(), "element {t} is not valid");
        total += e.area(&p);
        for k in 0..3 {
            let (a, b) = (n[k], n[(k + 1) % 3]);
            sides.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    assert!((total - area).abs() <= 1e-7 * area, "the elements cover {total}, the region {area}");
    let mut boundary: Vec<(usize, usize)> =
        sides.iter().filter(|(_, ts)| ts.len() == 1).map(|(k, _)| *k).collect();
    assert!(sides.values().all(|ts| ts.len() <= 2));
    let mut segs: Vec<(usize, usize)> = Vec::new();
    for list in &m.sides {
        for s in list {
            // the material on its left: an element runs it first to last
            assert!(m.tris.iter().any(|n| (0..3).any(|k| n[k] == s[0] && n[(k + 1) % 3] == s[2]
                                                       && n[3 + k] == s[1])),
                    "segment {s:?} has no element on its left");
            segs.push((s[0].min(s[2]), s[0].max(s[2])));
        }
        for w in list.windows(2) {
            assert_eq!(w[0][2], w[1][0], "a side's segments run end to end");
        }
    }
    boundary.sort();
    segs.sort();
    assert_eq!(boundary, segs);
}

/// **Kirsch's quarter meshes fair and graded** (#510 S2): every angle over 20.7°, the elements
/// covering the plate's area less the quarter hole's, every node of the hole on it, the elements
/// at the hole no longer than a few of its pieces and those far off ten times as long, and the
/// mesh the same run to run.
#[test]
fn kirsch_quarter_meshes_fair_and_graded() {
    let (r, w, h) = (10.0, 400.0, 200.0);
    let size = 20.0;
    let m = mesh(&quarter(r, w, h), &criteria(size)).unwrap();
    holds(&m, w * h - PI * r * r / 4.0, 90.0);
    for (i, on) in m.on.iter().enumerate() {
        if let Some((4, _)) = on {
            let p = m.points[i];
            assert!((p[0].hypot(p[1]) - r).abs() <= 1e-12 * r);
        }
    }
    let span = |n: &[usize; 6]| {
        let p = n.map(|i| m.points[i]);
        (0..3).map(|k| (p[k][0] - p[(k + 1) % 3][0]).hypot(p[k][1] - p[(k + 1) % 3][1]))
            .fold(0.0, f64::max)
    };
    let at_hole = |n: &&[usize; 6]| n.iter().any(|&i| matches!(m.on[i], Some((4, _))));
    let near: f64 = m.tris.iter().filter(at_hole).map(span).fold(0.0, f64::max);
    let far: f64 = m.tris.iter().filter(|n| m.points[n[0]][0] > 0.75 * w).map(span)
        .fold(0.0, f64::max);
    let piece = r * PI / 24.0;
    assert!(near < 3.0 * piece && far > 10.0 * near, "near {near}, far {far}");
    assert_eq!(m, mesh(&quarter(r, w, h), &criteria(size)).unwrap());
}

/// **A hole is a loop of its own, whichever way it is drawn** (#510 S2): a square with a round
/// hole, the circle one closed side run counter-clockwise or clockwise, meshes the square less
/// the disc either way, its segments with the material on their left.
#[test]
fn a_hole_is_meshed_out_whichever_way_it_runs() {
    let (a, r) = (100.0, 20.0);
    for sense in [1.0, -1.0] {
        let m = mesh(&holed(a, r, sense), &criteria(15.0)).unwrap();
        holds(&m, a * a - PI * r * r, 90.0);
        assert!(m.sides[4].len() >= 48, "the circle in pieces of 7.5° at most");
    }
}

/// **What a region that cannot be meshed says** (#510 S2): a loop that does not close, and a
/// corner too sharp for refinement to end within the points it may take.
#[test]
fn a_region_that_cannot_be_meshed_says_why() {
    let open = Region { loops: vec![vec![line([0.0, 0.0], [1.0, 0.0]), line([1.0, 0.0], [1.0, 1.0]),
                                         line([1.0, 1.1], [0.0, 0.0])]],
                         points: Vec::new() };
    assert!(mesh(&open, &criteria(0.5)).unwrap_err().contains("does not close"));
    let few = Criteria { most: 40, ..criteria(1.0) };
    assert!(mesh(&quarter(10.0, 400.0, 200.0), &few).unwrap_err().contains("more than 40"));
}

/// **A point named is a node** (#510 S2): a parameter a side must be cut at, and a point inside
/// the region, are each a node of the mesh where they are, the side's node knowing its
/// parameter; and the mesh holds as it did.
#[test]
fn a_point_named_is_a_node() {
    let (r, w, h) = (10.0, 400.0, 200.0);
    let mut region = quarter(r, w, h);
    region.loops[0][0].cuts = vec![0.3];
    region.loops[0][4].cuts = vec![0.5];
    region.points = vec![[123.0, 45.0]];
    let m = mesh(&region, &criteria(20.0)).unwrap();
    holds(&m, w * h - PI * r * r / 4.0, 90.0);
    let at = |p: [f64; 2]| m.points.iter().position(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 1e-12);
    let cut = at([r + 0.3 * (w - r), 0.0]).expect("the side's cut is a node");
    assert_eq!(m.on[cut], Some((0, 0.3)));
    let a = FRAC_PI_2 - 0.5 * FRAC_PI_2;
    assert!(at([r * a.cos(), r * a.sin()]).is_some(), "the arc's cut is a node");
    assert!(at([123.0, 45.0]).is_some_and(|i| m.on[i].is_none()), "the inner point is a node");
}
