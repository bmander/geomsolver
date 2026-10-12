//! **X1 of #183: rounds and chamfers of a coaxial region, exactly in the meridian.**  A prototype,
//! kept in this file on purpose (the language step that would call it, L5, has not been decided):
//! a region of lines and arcs — a meridian, as `solid::region::meridian` folds one — rounded or
//! chamfered where a step of one group of faces meets a step of another, with nothing but the
//! planar kernel's own steps.  `A round(r) B` is the ball of radius `r` rolled into that corner:
//! its centre where the two faces' carriers, each offset `r` toward the ball, cross (the opening's
//! `−r` then `+r`, read at one corner), the corner replaced by the arc between the two feet.  The
//! ball lies inside the material at a convex corner and outside it at a concave one, read off the
//! turn of the boundary, never said.  `A chamfer(d) B` is the segment between a point on each face,
//! each leg measured one of two ways (`Leg`; #183 X0's finding 7).
//!
//! Turned about the axis the arcs are tori and spheres and the segments cones and planes: the
//! kernel already revolves such a profile exactly.  So the gates are volumes by Green's theorem
//! (`turned`), held to 1e-9 against the closed forms the Rust fillet stack (#79, #83) checks its
//! fixtures by, and the hypoid's drawn rounding and end chamfer against the prototype's.
use gcs_core::brep::corners::{chamfer, round, Carrier, Leg};
use gcs_core::brep::planar::{boolean, Op, Region, Seg, Step};
use gcs_core::model::{RegionShape, SolidDef};
use gcs_core::program::Elaborated;
use gcs_core::space::{cross as cross3, dot as dot3, norm as norm3, sub as sub3};
use gcs_core::syntax::Cmp;
use std::f64::consts::{PI, TAU};

type P = [f64; 2];

fn sub(a: P, b: P) -> P { [a[0] - b[0], a[1] - b[1]] }
fn add(a: P, b: P) -> P { [a[0] + b[0], a[1] + b[1]] }
fn scale(a: P, k: f64) -> P { [a[0] * k, a[1] * k] }
fn dot(a: P, b: P) -> f64 { a[0] * b[0] + a[1] * b[1] }
#[allow(dead_code)]
fn cross(a: P, b: P) -> f64 { a[0] * b[1] - a[1] * b[0] }
fn norm(a: P) -> f64 { a[0].hypot(a[1]) }
fn dist(a: P, b: P) -> f64 { norm(sub(a, b)) }
fn unit(a: P) -> P { scale(a, 1.0 / norm(a)) }
#[allow(dead_code)]
fn left(a: P) -> P { [-a[1], a[0]] }

// -- exact measures ----------------------------------------------------------------------------

/// `∮ ρ²/2 dz` along a step, `(ρ, z)` its coordinates, in closed form.
fn flux(s: &Seg) -> f64 {
    match *s {
        Seg::Line { a: p, b: q } => {
            (q[1] - p[1]) / 2.0 * (p[0] * p[0] + p[0] * q[0] + q[0] * q[0]) / 3.0
        }
        Seg::Arc { c, r, a0, sweep } => {
            let prim = |phi: f64| {
                let (sn, cs) = phi.sin_cos();
                (c[0] * c[0] * r * sn + c[0] * r * r * (phi + sn * cs)
                    + r * r * r * (sn - sn.powi(3) / 3.0)) / 2.0
            };
            prim(a0 + sweep) - prim(a0)
        }
    }
}

/// The volume a meridian sweeps turned once about `ρ = 0` (Green's theorem, every step exact).
fn turned(region: &Region) -> f64 {
    2.0 * PI * region.loops.iter().flatten().map(|s| flux(&s.seg)).sum::<f64>()
}

fn close(got: f64, want: f64, what: &str) {
    let off = got - want;
    assert!(off.abs() <= 1e-9 * want.abs(), "{what}: {got} != {want} (off by {off:e})");
}

// -- meridians of the stack's turned fixtures ---------------------------------------------------

/// A polygon's meridian, its sides tagged in turn.
fn polygon(pts: &[P], tags: &[u32]) -> Region {
    let k = pts.len();
    Region::new(vec![(0..k).map(|i| Step {
        seg: Seg::Line { a: pts[i], b: pts[(i + 1) % k] },
        chart: None,
        tag: tags[i],
    }).collect()])
}

/// A disc of radius `r` from `z0` to `z1` about the axis: bottom, side, top, axis tagged
/// `tag`, `tag + 1`, `tag + 2`, `tag + 3`.
fn slab(r: f64, z0: f64, z1: f64, tag: u32) -> Region {
    polygon(&[[0.0, z0], [r, z0], [r, z1], [0.0, z1]], &[tag, tag + 1, tag + 2, tag + 3])
}

/// #79's frustum: 15 across its foot at 10, 5 across its top at 30; foot, slant, top, axis
/// tagged 11 to 14.
fn frustum() -> Region {
    polygon(&[[0.0, 10.0], [15.0, 10.0], [5.0, 30.0], [0.0, 30.0]], &[11, 12, 13, 14])
}

fn faces(tag: u32, n: u32) -> Vec<u32> { (tag..tag + n).collect() }

fn union(a: &Region, b: &Region) -> Region { boolean(a, b, Op::Union, 1e-9).unwrap() }

/// The stack's `root_ring`: the ring a ball of radius `r` fills where a cylinder of radius `big`
/// stands on a plane — the square at the corner less the quarter disc, turned (Pappus).
fn root_ring(big: f64, r: f64) -> f64 {
    let moment = r * r * (big + r / 2.0) - PI * r * r / 4.0 * (big + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * moment
}

/// The stack's `turned_wedge`: the volume the ball of radius `r` in the meridian corner at `corner`
/// between unit directions `d` leaves turned, by its section in closed form.
fn turned_wedge(corner: P, d: [P; 2], r: f64) -> f64 {
    let half = dot(d[0], d[1]).acos() / 2.0;
    let s = r / half.tan();
    let t = d.map(|u| add(corner, scale(u, s)));
    let c = add(corner, scale(unit(add(d[0], d[1])), r / half.sin()));
    let angle = |p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
    let (a0, mut a1) = (angle(t[0]), angle(t[1]));
    while a1 - a0 > PI { a1 -= TAU; }
    while a0 - a1 > PI { a1 += TAU; }
    let f = flux(&Seg::Line { a: corner, b: t[0] }) + flux(&Seg::Arc { c, r, a0, sweep: a1 - a0 })
        + flux(&Seg::Line { a: t[1], b: corner });
    2.0 * PI * f.abs()
}

/// #79's pad: a boss of radius 8 standing 20 on a plate 10 thick, its root filled to 3, a bore of
/// radius 4 drilled through after.  The plate is a 60 × 40 prism; in the meridian it is a disc
/// wide enough that the ring stands on its top, which is all the ring reads.
#[test]
fn a_boss_root_is_its_ring() {
    let (big, r, small) = (8.0, 3.0, 4.0);
    let plate = slab(20.0, -10.0, 0.0, 1);
    let boss = slab(big, 0.0, 20.0, 11);
    let pad = union(&plate, &boss);
    let rounded = round(&pad, &faces(11, 4), &faces(1, 4), r, 100).unwrap();
    close(turned(&rounded) - turned(&pad), root_ring(big, r), "the root ring");
    let bore = slab(small, -10.0, 20.0, 21);
    let drilled = boolean(&rounded, &bore, Op::Cut, 1e-9).unwrap();
    let want = PI * 20.0 * 20.0 * 10.0 + PI * big * big * 20.0 + root_ring(big, r)
        - PI * small * small * 30.0;
    close(turned(&drilled), want, "the drilled pad");
}

/// #79's turned fixture (`x0/turned.sv`): a frustum standing on a disc, its foot filled to 3 and
/// its top rim rounded to 1 — concave and convex by the same word.
#[test]
fn a_frustum_root_and_rim_are_tori() {
    let plate = slab(40.0, 0.0, 10.0, 1);
    let body = union(&plate, &frustum());
    let root = round(&body, &faces(11, 4), &faces(1, 4), 3.0, 100).unwrap();
    let rim = round(&root, &[13], &[12], 1.0, 101).unwrap();
    let slant = unit([-10.0, 20.0]);
    let want_root = turned_wedge([15.0, 10.0], [[1.0, 0.0], slant], 3.0);
    let want_rim = turned_wedge([5.0, 30.0], [[-1.0, 0.0], scale(slant, -1.0)], 1.0);
    close(turned(&root) - turned(&body), want_root, "the root");
    close(turned(&root) - turned(&rim), want_rim, "the rim");
    let frustum_v = PI * 20.0 / 3.0 * (15.0 * 15.0 + 15.0 * 5.0 + 5.0 * 5.0);
    close(turned(&rim), PI * 40.0 * 40.0 * 10.0 + frustum_v + want_root - want_rim, "the body");
    // the order of two rounds on different corners is no matter
    let rim_first = round(&body, &[13], &[12], 1.0, 101).unwrap();
    let other = round(&rim_first, &faces(11, 4), &faces(1, 4), 3.0, 100).unwrap();
    close(turned(&other), turned(&rim), "the rounds the other way about");
}

/// The stack's closed form of #83's knob's neck: a rod of radius `rod` about the axis entering a
/// ball of radius `big` about `s`, filled to `r`.  Its section is the corner, its foot on the rod,
/// the arc of the ball rolled, its foot on the ball and the ball's arc back, turned.
pub(crate) fn knob_neck(s: P, big: f64, rod: f64, r: f64) -> f64 {
    let corner = [rod, s[1] - (big * big - rod * rod).sqrt()];
    let centre = [rod + r, s[1] - ((big + r).powi(2) - (rod + r).powi(2)).sqrt()];
    let (on_rod, on_ball) = ([rod, centre[1]], add(s, scale(unit(sub(centre, s)), big)));
    let at = |c: P, p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
    let arc = |c: P, r: f64, from: P, to: P| {
        Seg::Arc { c, r, a0: at(c, from), sweep: at(c, to) - at(c, from) }
    };
    2.0 * PI * (flux(&Seg::Line { a: corner, b: on_rod })
        + flux(&arc(centre, r, on_rod, on_ball)) + flux(&arc(s, big, on_ball, corner))).abs()
}

/// #83's knob (`x0/knob.sv`): a rod standing on a disc and entering a ball, its root filled to 2
/// (plane against cylinder) and its neck to 1.5 (cylinder against sphere, a section with a curved
/// side).
#[test]
fn a_knobs_root_and_neck_turn_their_sections() {
    let (s, big, rod, r): (P, f64, f64, f64) = ([0.0, 44.0], 8.0, 4.0, 1.5);
    let disc = slab(30.0, 0.0, 8.0, 1);
    let shaft = slab(rod, 8.0, 40.0, 11);
    let ball = Region::new(vec![vec![
        Step { seg: Seg::Arc { c: s, r: big, a0: -PI / 2.0, sweep: PI }, chart: None, tag: 21 },
        Step { seg: Seg::Line { a: [0.0, s[1] + big], b: [0.0, s[1] - big] }, chart: None,
               tag: 22 },
    ]]);
    let knob = union(&union(&disc, &shaft), &ball);
    let root = round(&knob, &faces(11, 4), &faces(1, 4), 2.0, 100).unwrap();
    let neck = round(&root, &faces(11, 4), &[21, 22], r, 101).unwrap();
    let corner = [rod, s[1] - (big * big - rod * rod).sqrt()];
    let neck_v = knob_neck(s, big, rod, r);
    let root_v = root_ring(rod, 2.0);
    close(turned(&root) - turned(&knob), root_v, "the root");
    close(turned(&neck) - turned(&root), neck_v, "the neck");
    let cap = { let h = corner[1] - (s[1] - big); PI * h * h * (3.0 * big - h) / 3.0 };
    let want = PI * 30.0 * 30.0 * 8.0 + PI * rod * rod * (corner[1] - 8.0)
        + 4.0 / 3.0 * PI * big.powi(3) - cap + root_v + neck_v;
    close(turned(&neck), want, "the knob");
}

/// A chamfer of the frustum's rim is a cone band: the triangle it takes, turned (Pappus).  Both
/// legs measured from the edge, then from the other face: at the rim's corner (a turn of 117°)
/// the two differ by `1 / sin`.
#[test]
fn a_rim_chamfer_takes_its_triangle() {
    let frustum = frustum();
    let (corner, slant, top) = ([5.0, 30.0], unit([10.0, -20.0]), [-1.0, 0.0]);
    // a triangle's area times the path of its centroid
    let pappus = |a: P, b: P, c: P| {
        2.0 * PI * (cross(sub(b, a), sub(c, a)) / 2.0).abs() * (a[0] + b[0] + c[0]) / 3.0
    };
    let d = 1.0;
    let by_edge = chamfer(&frustum, &[12], &[13], [Leg::FromEdge(d); 2], 100).unwrap();
    let want = pappus(corner, add(corner, scale(slant, d)), add(corner, scale(top, d)));
    close(turned(&frustum) - turned(&by_edge), want, "legs from the edge");
    let sin = cross(slant, top).abs();
    let by_other = chamfer(&frustum, &[12], &[13], [Leg::FromOther(d); 2], 100).unwrap();
    let want = pappus(corner, add(corner, scale(slant, d / sin)), add(corner, scale(top, d / sin)));
    close(turned(&frustum) - turned(&by_other), want, "legs from the other face");
    // and the two legs told apart: the slant's 2, the top's 1
    let two = chamfer(&frustum, &[12], &[13], [Leg::FromEdge(2.0), Leg::FromEdge(1.0)], 100)
        .unwrap();
    let want = pappus(corner, add(corner, scale(slant, 2.0)), add(corner, scale(top, 1.0)));
    close(turned(&frustum) - turned(&two), want, "two legs");
}

/// A size of zero is the identity, decided by value; a ball the faces cannot hold is refused,
/// naming the faces — under the definition it would consume one (#183 X0, finding 9).
#[test]
fn zero_is_the_identity_and_a_ball_too_big_is_refused() {
    let frustum = frustum();
    let same = round(&frustum, &[12], &[13], 0.0, 100).unwrap();
    assert_eq!(turned(&same), turned(&frustum));
    let why = round(&frustum, &[12], &[13], 20.0, 100).unwrap_err();
    assert!(why.contains("12") && why.contains("13"), "{why}");
    // a disc's side is 10 tall: rounds of 6 at both its ends meet within it
    let why = round(&slab(40.0, 0.0, 10.0, 1), &[2], &[1, 3], 6.0, 100).unwrap_err();
    assert!(why.contains("meet within it"), "{why}");
}

// -- the hypoid's drawn rounding and end chamfer -------------------------------------------------

fn entity(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// #145's crown tip rounding (`crown/rounding.sv`'s preview): a section's two tip corners rounded
/// by `TipRounding`'s drawn tangent arcs.  The sharp section, from the drawn lines' carriers,
/// rounded by the prototype: the same centres and feet.
#[test]
fn the_crown_tip_rounding_is_the_balls() {
    let e = fixtures::read(include_str!("../../examples/spiral_bevel/crown/rounding.sv"));
    let sk = &e.sketch;
    // std.front is the world's (x, z)
    let at = |name: &str| {
        let p = sk.world_point(entity(&e, name));
        assert!(p[1].abs() < 1e-12);
        [p[0], p[2]]
    };
    let carrier = |a: &str, b: &str| Carrier::Line { p: at(a), d: unit(sub(at(b), at(a))) };
    let meet = |a: Carrier, b: Carrier| a.meet(b)[0];
    let (outer, tip, inner) = (carrier("bo", "oj"), carrier("ot", "it"), carrier("ij", "bi"));
    let sharp = polygon(&[at("bi"), at("bo"), meet(outer, tip), meet(tip, inner)], &[1, 2, 3, 4]);
    let r = 0.3 * 2.0;
    let rounded = round(&round(&sharp, &[2], &[3], r, 10).unwrap(), &[3], &[4], r, 11).unwrap();
    let arc = |tag: u32| rounded.loops[0].iter().find(|s| s.tag == tag).unwrap().seg;
    let mut worst: f64 = 0.0;
    let arcs = [(10, "outer_round", ["oj", "ot"]), (11, "inner_round", ["it", "ij"])];
    for (tag, name, feet) in arcs {
        let Seg::Arc { c, r: got, .. } = arc(tag) else { panic!("a round is an arc") };
        let drawn = &sk.arcs[entity(&e, name)];
        let centre = sk.world_point(drawn.center as usize);
        worst = worst.max(dist(c, [centre[0], centre[2]])).max((got - r).abs());
        let s = arc(tag);
        let ends = [s.start(), s.end()];
        for f in feet {
            worst = worst.max(ends.iter().map(|p| dist(*p, at(f))).fold(f64::INFINITY, f64::min));
        }
    }
    assert!(worst < 1e-9, "the prototype's rounds leave the drawn ones by {worst:e}");
}

/// #145's end relief (`blank/member.sv`'s preview, `blank/ends.sv`'s `EndChamfer`): the blank's
/// meridian, folded by the core from its region's terms, chamfered at the tip's toe and heel
/// corners by the prototype, against the drawn chamfers' ends and the volume the body rule builds
/// (the region less the two drawn rings, by the kernel).  `EndChamfer` measures each leg from the
/// other face: its end on the tip `size` inside the sphere, its end on the sphere `size` below the
/// tip cone.  Measured from the edge instead, both legs move by `1 / sin` of the corner's angle.
#[test]
fn the_end_relief_is_a_chamfer_from_the_other_face() {
    let e = fixtures::gear::read(include_str!("../../examples/spiral_bevel/blank/member.sv"),
        &fixtures::gear::project());
    let sk = &e.sketch;
    let solid = entity(&e, "member.material");
    let m = gcs_core::solid::region::meridian(sk, solid).unwrap();
    // every term is named for the region (`m.names`), not for the set it came from, so a face is
    // told by its shape and side: the tip the cone it is inside, the heel the ball it is inside,
    // the toe the ball it is outside
    let SolidDef::Region { terms, .. } = &sk.solids[solid].def else { panic!("a region") };
    let tag = |ball: bool, cmp: Cmp| terms.iter().position(|t| {
        matches!(t.shape, RegionShape::Ball { .. }) == ball && t.cmp == cmp
            && matches!(t.shape, RegionShape::Ball { .. } | RegionShape::Cone { .. })
    }).unwrap() as u32 + 1;
    let (tip, toe, heel) = (tag(false, Cmp::Le), tag(true, Cmp::Ge), tag(true, Cmp::Le));
    let to_meridian = |p: [f64; 3]| {
        let d = sub3(p, m.origin);
        [norm3(cross3(d, m.axis)), dot3(d, m.axis)]
    };
    let size = 0.2;
    let legs = [Leg::FromOther(size); 2];
    let ends = |legs: [Leg; 2]| {
        let toe_end = chamfer(&m.region, &[tip], &[toe], legs, 100).unwrap();
        chamfer(&toe_end, &[tip], &[heel], legs, 101).unwrap()
    };
    let relieved = ends(legs);
    // the chamfers' ends, the drawn ones'
    let copied = |end: &str, part: &str| e.map.names.values().flatten()
        .find(|n| n.starts_with("member.") && n.ends_with(&format!("{end}_end.{part}")))
        .unwrap_or_else(|| panic!("no `{part}` of the {end} end")).clone();
    let mut worst: f64 = 0.0;
    for (end, t) in [("toe", 100), ("heel", 101)] {
        let flat = relieved.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
        for part in ["along_tip", "down_end"] {
            let drawn = to_meridian(sk.world_point(entity(&e, &copied(end, part))));
            worst = worst.max(dist(flat.start(), drawn).min(dist(flat.end(), drawn)));
        }
    }
    assert!(worst < 1e-9, "the prototype's chamfers leave the drawn ones by {worst:e}");
    // the volumes: the sharp meridian turned against the region's exact solid, and the chamfered
    // one against the body the rings are cut from
    let volume = |name: &str| {
        let x = sk.exact_solid(entity(&e, name)).unwrap_or_else(|why| panic!("{name}: {why}"));
        gcs_core::brep::props::volume(&x.brep)
    };
    close(turned(&m.region), volume("member.material"), "the sharp blank");
    let body = volume("body");
    close(turned(&relieved), body, "the relieved blank");
    // legs measured from the edge instead
    let by_edge = ends([Leg::FromEdge(size); 2]);
    let (taken, short) = (turned(&m.region) - body, turned(&by_edge) - turned(&relieved));
    let (of_chamfers, of_blank) = (short / taken, short / body);
    eprintln!("end relief: the chamfers take {taken:.6} mm³; legs from the edge take {short:.3e} \
               mm³ less ({of_chamfers:.3e} of the chamfers', {of_blank:.3e} of the blank)");
    assert!(short > 0.0 && short / taken < 1e-2, "{short} of {taken}");
}

/// X3 of #183: the end relief as a chamfer said in Solvent's own terms, the points of the blank
/// nearer the tip and the sphere than `size` in all (`distance(p, tip) + distance(p, toe) <
/// size`).  Its edge passes through the drawn chamfer's two ends (legs from the other face, X1's
/// finding 3), and between them bows off the straight chamfer by the sphere's curvature: by how
/// much, against the chamfer's own material.
#[test]
fn the_end_relief_as_a_sum_of_distances_bows_on_the_sphere() {
    let e = fixtures::gear::read(include_str!("../../examples/spiral_bevel/blank/member.sv"),
        &fixtures::gear::project());
    let sk = &e.sketch;
    let solid = entity(&e, "member.material");
    let m = gcs_core::solid::region::meridian(sk, solid).unwrap();
    let SolidDef::Region { terms, .. } = &sk.solids[solid].def else { panic!("a region") };
    let tag = |ball: bool, cmp: Cmp| terms.iter().position(|t| {
        matches!(t.shape, RegionShape::Ball { .. }) == ball && t.cmp == cmp
            && matches!(t.shape, RegionShape::Ball { .. } | RegionShape::Cone { .. })
    }).unwrap() as u32 + 1;
    let (tip, toe, heel) = (tag(false, Cmp::Le), tag(true, Cmp::Ge), tag(true, Cmp::Le));
    let size = 0.2;
    let step = |t: u32| m.region.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
    let Seg::Line { a: ta, b: tb } = step(tip) else { panic!("the tip is a line") };
    // signed distances, positive in the material (on a step's left)
    let to_tip = |q: P| cross(sub(tb, ta), sub(q, ta)) / dist(ta, tb);
    let mut worst_end: f64 = 0.0;
    for (end, t) in [(toe, 100), (heel, 101)] {
        let Seg::Arc { c, r, sweep, .. } = step(end) else { panic!("an end is an arc") };
        let to_sphere = |q: P| sweep.signum() * (r - dist(q, c));
        let one = chamfer(&m.region, &[tip], &[end], [Leg::FromOther(size); 2], t).unwrap();
        let taken = turned(&m.region) - turned(&one);
        let flat = one.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
        let (a, b) = (flat.start(), flat.end());
        // the corner: where the tip meets the sphere, beyond the chord
        let off = |q: &P| to_sphere(*q).abs();
        let corner = [ta, tb].into_iter().min_by(|p, q| off(p).total_cmp(&off(q))).unwrap();
        let mut n = unit(left(sub(b, a)));
        if dot(n, sub(corner, a)) < 0.0 {
            n = scale(n, -1.0);
        }
        // where the sum reaches `size` along the chord's normal, toward the corner
        let edge_at = |u: f64| {
            let q0 = add(a, scale(sub(b, a), u));
            let f = |s: f64| { let q = add(q0, scale(n, s)); to_tip(q) + to_sphere(q) - size };
            let (mut lo, mut hi) = (-size / 2.0, size / 2.0);
            for _ in 0..100 {
                let mid = 0.5 * (lo + hi);
                if (f(mid) > 0.0) == (f(lo) > 0.0) { lo = mid } else { hi = mid }
            }
            (q0, 0.5 * (lo + hi))
        };
        worst_end = worst_end.max(edge_at(0.0).1.abs()).max(edge_at(1.0).1.abs());
        // the material between the chord and the sum's edge, turned: a midpoint sum
        let k = 2000;
        let (mut bow, mut widest): (f64, f64) = (0.0, 0.0);
        for i in 0..k {
            let (q0, s) = edge_at((i as f64 + 0.5) / k as f64);
            widest = widest.max(s.abs());
            bow += 2.0 * PI * s * (q0[0] + 0.5 * s * n[0]) * dist(a, b) / k as f64;
        }
        eprintln!("end relief ({}): the chamfer takes {taken:.6} mm³; the sum's edge stands off \
                   its chord {widest:.2e} mm at most, {:+.2e} of the chamfer's material",
            if end == toe { "toe" } else { "heel" }, -bow / taken);
        assert!(widest > 0.0 && widest < 1e-3 * size && bow.abs() < 1e-3 * taken);
    }
    assert!(worst_end < 1e-12, "the sum's edge leaves the legs' ends by {worst_end:e}");
}

/// **The hypoid's end relief stated in its blank's own region** (#183 step 6): `std.Chamfered` at
/// the tip's toe and heel corners, each leg 0.2 from the other face, is today's `EndChamfer` body —
/// the blank less the two drawn rings — wherever a point is farther from either boundary than the
/// sum of distances bows off the straight chamfer on the sphere (7e-5 mm, X3).  Read by the field:
/// a chamfer beside a sphere is not lowered (the sum bows there, #208).
#[test]
fn the_end_relief_stated_in_the_region_is_end_chamfers() {
    let text = include_str!("../../examples/spiral_bevel/blank/member.sv");
    let close = text.rfind('}').unwrap();
    let stated = "  toe_side := { q | q outside member.toe }\n\
                  relieved := { p | p inside member.tip; p inside member.heel; p outside member.toe\n\
                    p outside(8mm) member.pitch\n\
                    p inside std.Chamfered(member.tip, toe_side, la: 0.2mm, lb: 0.2mm)\n\
                    p inside std.Chamfered(member.tip, member.heel, la: 0.2mm, lb: 0.2mm) }\n\
                  stated := solid(relieved)\n";
    let text = format!("{}{stated}{}", &text[..close], &text[close..]);
    let e = fixtures::gear::read(&text, &fixtures::gear::project());
    let sk = &e.sketch;
    let m = gcs_core::solid::region::meridian(sk, entity(&e, "member.material")).unwrap();
    let ours = gcs_core::solid::MaterialField::read(sk, entity(&e, "stated"), 1e-10).unwrap();
    let theirs = gcs_core::solid::MaterialField::read(sk, entity(&e, "body"), 1e-10).unwrap();
    // the tip's corners with the spheres, in the meridian, and points about each
    let SolidDef::Region { terms, .. } = &sk.solids[entity(&e, "member.material")].def else { panic!() };
    let tip = terms.iter().position(|t| matches!(t.shape, RegionShape::Cone { .. }) && t.cmp == Cmp::Le)
        .unwrap() as u32 + 1;
    let corners: Vec<P> = m.region.loops.iter().flat_map(|l| {
        (0..l.len()).filter(move |&i| (l[i].tag == tip) != (l[(i + 1) % l.len()].tag == tip))
            .map(move |i| l[i].end())
    }).collect();
    assert_eq!(corners.len(), 2, "the tip meets each sphere once");
    let lift = |q: P| -> [f64; 3] {
        std::array::from_fn(|k| m.origin[k] + q[0] * m.seam[k] + q[1] * m.axis[k])
    };
    let (mut checked, mut seen_cut) = (0, 0);
    for c in corners {
        for i in 0..=40 {
            for j in 0..=40 {
                // off the corner's own ordinate: a ray level with a vertex is the parity rule's
                // edge (its arc's end and its line's start a unit in the last place apart)
                let q = [c[0] + (i as f64 - 20.0) * 0.02 + 1.37e-3, c[1] + (j as f64 - 20.0) * 0.02 + 2.91e-3];
                let p = lift(q);
                let (a, b) = (ours.side(p), theirs.side(p));
                if a.abs() < 2e-4 || b.abs() < 2e-4 {
                    continue;
                }
                assert_eq!(a < 0.0, b < 0.0, "at {q:?} (corner {c:?}): {a} against {b}");
                checked += 1;
                // inside the sharp blank and cut by the relief
                if b > 0.0 && m.region.contains(q) {
                    seen_cut += 1;
                }
            }
        }
    }
    assert!(checked > 2000 && seen_cut > 10, "{checked} points, {seen_cut} in the reliefs");
}


/// **The crown's tip rounding by the word** (#183 step 6): the section drawn sharp from the same
/// carriers, swept a little either side of its view, its tip's two edges rounded by `round` — the
/// field's boundary passes through `TipRounding`'s drawn arcs, and a point a twentieth of the
/// radius either side of an arc is on that side.
#[test]
fn the_crown_tip_rounding_is_the_words() {
    let text = include_str!("../../examples/spiral_bevel/crown/rounding.sv").replacen("use std\n", "use std (round)\n", 1);
    let close = text.rfind('}').unwrap();
    let more = "  in std.front {\n\
                    c1 := point hint((6.5, 2.5))\n    c2 := point hint((-0.5, 2.5))\n\
                    c1 coincident outer\n    c1 coincident tip\n    c2 coincident tip\n    c2 coincident inner\n\
                    (sb := line(bi, bo)) -> (so := line(bo, c1)) -> (st := line(c1, c2)) -> (si := line(c2, bi)) -> close\n\
                    sharp := face(sb, so, st, si)\n  }\n\
                  tooth := solid(sharp, from: -1mm, to: 1mm)\n\
                  tooth.st round(r: 0.6mm) tooth.so\n\
                  tooth.st round(r: 0.6mm) tooth.si\n";
    let text = format!("{}{more}{}", &text[..close], &text[close..]);
    let e = fixtures::read(&text);
    let sk = &e.sketch;
    let field = gcs_core::solid::MaterialField::read(sk, entity(&e, "tooth"), 1e-10).unwrap();
    let r = 0.6;
    // what the sharp corners had, a little in from each toward its arc's centre, is taken away
    for (corner, name) in [("c1", "outer_round"), ("c2", "inner_round")] {
        let (p, c) = (sk.world_point(entity(&e, corner)), sk.world_point(sk.arcs[entity(&e, name)].center as usize));
        let q: [f64; 3] = std::array::from_fn(|k| p[k] + 0.05 * (c[k] - p[k]));
        assert!(field.side(q) > 0.0, "{corner}: {}", field.side(q));
    }
    for name in ["outer_round", "inner_round"] {
        let arc = &sk.arcs[entity(&e, name)];
        let c = sk.world_point(arc.center as usize);
        let (s, t) = (sk.world_point(arc.start as usize), sk.world_point(arc.end as usize));
        // the arc's middle, in the view (y = 0), and its outward normal there
        let mid: [f64; 3] = std::array::from_fn(|k| (s[k] + t[k]) / 2.0 - c[k]);
        let n = 1.0 / norm3(mid);
        let out: [f64; 3] = mid.map(|x| x * n);
        let on: [f64; 3] = std::array::from_fn(|k| c[k] + r * out[k]);
        let off = |d: f64| field.side(std::array::from_fn(|k| on[k] + d * out[k]));
        assert!(off(0.0).abs() < 1e-7, "{name}: the field at the drawn arc reads {}", off(0.0));
        assert!(off(-r / 20.0) < 0.0 && off(r / 20.0) > 0.0, "{name}");
    }
}
