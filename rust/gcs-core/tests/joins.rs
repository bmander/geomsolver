//! **Some points of sets** ([0.212], docs/contract/sets.md): `some q in A, r in B: p in X` in a
//! set's body, the points some point of each set puts the set's point in — of a segment between
//! them, the join of the sets (of convex sets their hull, built exactly by the kernel's hull), of
//! a ball about one, the set grown.  Held to closed forms: a polytope's volume and area, Steiner's
//! for one grown, Pappus's for a capsule.
use gcs_core::brep::geom::Surface;
use gcs_core::brep::props::volume;
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solid::region::route;
use std::f64::consts::PI;

use crate::common::{ent, refused, text::solved};

fn close(what: &str, a: f64, b: f64) { assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{what}: {a} against {b}"); }

fn area(b: &gcs_core::brep::topo::Brep) -> f64 { crate::brep::total_area(b) }

/// `name`, built exactly: its boundary, by the route that builds it.
fn exact(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let s = ent(e, name).i();
    let r = route(&e.sketch, s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    assert!(r.refusal().is_none(), "`{name}`: {:?}", r.refusal());
    e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"))
}

/// The regular tetrahedron on (3, 3, 3), (3, −3, −3), (−3, 3, −3), (−3, −3, 3), as the join of two
/// of its opposite edges; and grown by 1, brep-bench's offset/tetrahedron-grown.
const TETRAHEDRON: &str = "unit mm\nuse std\nuse std.sets\nuse std.rounds\n\
    a := point\nb := point\nc := point\nd := point\n\
    fix((3, 3, 3)) a\nfix((3, -3, -3)) b\nfix((-3, 3, -3)) c\nfix((-3, -3, 3)) d\n\
    tetrahedron := solid({ p | some q in std.sets.Segment(a, b), r in std.sets.Segment(c, d): \
    p in std.sets.Segment(q, r) })\n\
    grown := std.rounds.Grown(tetrahedron, d: 1mm)\n";

/// **A tetrahedron is the join of two opposite edges**: some point of each, and the segment
/// between them — the hull of its four corners, read point by point as its planes and built
/// exactly as the kernel's hull; grown by 1, Steiner's, as the bench gives it.
#[test]
fn a_tetrahedron_is_the_join_of_two_of_its_edges() {
    let e = solved(TETRAHEDRON);
    let x = exact(&e, "tetrahedron");
    assert_eq!(x.brep.faces.len(), 4);
    assert!(x.brep.faces.iter().all(|f| matches!(f.surface, Surface::Plane(_))));
    close("volume", volume(&x.brep), 72.0);
    close("area", area(&x.brep), 72.0 * 3f64.sqrt());
    // its field: its inradius below nothing at its middle, nothing on a face and at a corner
    let f = MaterialField::read(&e.sketch, ent(&e, "tetrahedron").i(), 1e-10).unwrap();
    close("middle", f.side([0.0; 3]), -3f64.sqrt());
    assert!(f.side([3.0, 3.0, 3.0]).abs() < 1e-12 && f.side([0.0, 0.0, 3.0]).abs() < 1e-12);
    assert!(f.side([3.1, 3.1, 3.1]) > 0.0 && f.side([0.0, 0.0, -2.9]) < 0.0);
    let g = exact(&e, "grown.body");
    close("grown", volume(&g.brep), 249.53323018738044);
}

/// A square's corners and an octagon's 30 above it: brep-bench's loft/square-to-octagon.
const LOFT: &str = "unit mm\nuse std\nuse std.sets\n\
    s0 := point\ns1 := point\ns2 := point\ns3 := point\n\
    fix((20, -20, 0)) s0\nfix((20, 20, 0)) s1\nfix((-20, 20, 0)) s2\nfix((-20, -20, 0)) s3\n\
    o0 := point\no1 := point\no2 := point\no3 := point\no4 := point\no5 := point\no6 := point\no7 := point\n\
    fix((6, -14, 30)) o0\nfix((14, -6, 30)) o1\nfix((14, 6, 30)) o2\nfix((6, 14, 30)) o3\n\
    fix((-6, 14, 30)) o4\nfix((-14, 6, 30)) o5\nfix((-14, -6, 30)) o6\nfix((-6, -14, 30)) o7\n\
    square := { x | some u in std.sets.Segment(s0, s1), v in std.sets.Segment(s2, s3): x in std.sets.Segment(u, v) }\n\
    near := { x | some u in std.sets.Segment(o0, o1), v in std.sets.Segment(o2, o3): x in std.sets.Segment(u, v) }\n\
    far := { x | some u in std.sets.Segment(o4, o5), v in std.sets.Segment(o6, o7): x in std.sets.Segment(u, v) }\n\
    octagon := { x | some u in near, v in far: x in std.sets.Segment(u, v) }\n\
    loft := solid({ p | some q in square, r in octagon: p in std.sets.Segment(q, r) })\n";

/// **Joins nest**: a square the join of two of its sides, an octagon the join of two joins, and
/// the transition between them the join of both — the hull of all twelve corners, its faces the
/// floor, the top, four trapezoids and four triangles, to the bench's closed forms.
#[test]
fn a_square_to_octagon_transition_is_a_join_of_joins() {
    let e = solved(LOFT);
    let x = exact(&e, "loft");
    assert_eq!(x.brep.faces.len(), 10);
    close("volume", volume(&x.brep), 33760.0);
    close("area", area(&x.brep), 2256.0 + 624.0 * 26f64.sqrt() + 160.0 * 22f64.sqrt());
}

/// **A pyramid is the join of its apex and its base**, a face drawn on the top plane: the pyramid
/// on the rectangle ±10 × ±6 with its apex 12 above, a third of its base times its height.
#[test]
fn a_pyramid_is_the_join_of_its_apex_and_its_base() {
    let src = "unit mm\nuse std\nuse std.sets\nin std.top {\n\
        b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
        fix((-10, -6)) b0\nfix((10, -6)) b1\nfix((10, 6)) b2\nfix((-10, 6)) b3\n\
        (s := line(b0, b1)) -> (e := line(b1, b2)) -> (n := line(b2, b3)) -> (w := line(b3, b0)) -> close\n\
        base := face(s, e, n, w)\n}\n\
        apex := point\nfix((0, 0, 12)) apex\n\
        pyramid := solid({ p | some q in base: p in std.sets.Segment(apex, q) })\n";
    let e = solved(src);
    let x = exact(&e, "pyramid");
    assert_eq!(x.brep.faces.len(), 5);
    close("volume", volume(&x.brep), 20.0 * 12.0 * 12.0 / 3.0);
}

/// **A ball about some point of a set is the set grown**: about some point of a segment, a capsule
/// — read as the distance from the segment, its field the distance less the radius.
#[test]
fn a_ball_about_some_point_of_a_set_is_the_set_grown() {
    let src = "unit mm\nuse std\nuse std.sets\n\
        a := point\nb := point\nfix((0, 0, 0)) a\nfix((10, 0, 0)) b\n\
        capsule := solid({ p | some q in std.sets.Segment(a, b): p distance(<= 2mm) q })\n";
    let e = solved(src);
    let f = MaterialField::read(&e.sketch, ent(&e, "capsule").i(), 1e-10).unwrap();
    close("on the axis", f.side([5.0, 0.0, 0.0]), -2.0);
    close("beside it", f.side([5.0, 3.0, 0.0]), 1.0);
    close("past an end", f.side([13.0, 0.0, 0.0]), 1.0);
    let _ = PI;
}

/// **What is not read is refused by name**: a relation that is neither a segment between the
/// points nor a ball about one, a `some` whose set's point is a drawn one, a join of a polygon
/// turning both ways, and `not some` of points.
#[test]
fn what_some_points_of_sets_cannot_read_is_refused() {
    let base = "unit mm\nuse std\nuse std.sets\na := point\nb := point\nfix((0, 0, 0)) a\nfix((10, 0, 0)) b\n";
    refused(&format!("{base}far := solid({{ p | some q in std.sets.Segment(a, b): p distance(>= 2mm) q }})\n"),
        "E040", "is not read yet", "far := solid({ p | some q in std.sets.Segment(a, b): p distance(>= 2mm) q })");
    // said once: the solid it was refused in is not refused again for want of a term
    let (prog, _, _) = gcs_core::library::parse_linked(&format!(
        "{base}far := solid({{ p | some q in std.sets.Segment(a, b): p distance(>= 2mm) q }})\n"));
    let e = gcs_core::program::elaborate(&prog);
    assert_eq!(e.errors().count(), 1, "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    refused(&format!("{base}x := point\nsome q in std.sets.Segment(a, b): x in std.sets.Segment(a, q)\n"),
        "E040", "read where a solid reads its set's point", "in");
    let (_, errs, _) = gcs_core::library::parse_linked(&format!(
        "{base}s := solid({{ p | not some q in std.sets.Segment(a, b): p in std.sets.Segment(a, q) }})\n"));
    assert!(errs.iter().any(|e| e.message.contains("`not some` reads the complement")), "{errs:?}");
    // an L's corners turn both ways: the hull of them is not its join with a point
    let src = "unit mm\nuse std\nuse std.sets\nin std.top {\n\
        l0 := point\nl1 := point\nl2 := point\nl3 := point\nl4 := point\nl5 := point\n\
        fix((0, 0)) l0\nfix((20, 0)) l1\nfix((20, 10)) l2\nfix((10, 10)) l3\nfix((10, 20)) l4\nfix((0, 20)) l5\n\
        (k0 := line(l0, l1)) -> (k1 := line(l1, l2)) -> (k2 := line(l2, l3)) -> (k3 := line(l3, l4)) -> \
        (k4 := line(l4, l5)) -> (k5 := line(l5, l0)) -> close\n\
        ell := face(k0, k1, k2, k3, k4, k5)\n}\n\
        apex := point\nfix((5, 5, 12)) apex\n\
        spire := solid({ p | some q in ell: p in std.sets.Segment(apex, q) })\n";
    let e = solved(src);
    let s = ent(&e, "spire").i();
    let why = MaterialField::read(&e.sketch, s, 1e-10).err().unwrap_or_default();
    assert!(why.contains("convex polygon"), "{why}");
}

/// What reading a join point by point costs (step 1 of [0.212]'s experiment,
/// docs/some-points-of-sets.md): the tetrahedron as a join and as the half-spaces behind its
/// faces' planes, each read where a field is first made and then at a million points.
#[test]
#[ignore = "measurement: a join's field read against its half-spaces' (docs/some-points-of-sets.md)"]
fn a_joins_field_is_measured() {
    let planes = "unit mm\nuse std\n\
        a := point\nb := point\nc := point\nd := point\n\
        fix((3, 3, 3)) a\nfix((3, -3, -3)) b\nfix((-3, 3, -3)) c\nfix((-3, -3, 3)) d\n\
        ab := line(a, b)\nac := line(a, c)\nad := line(a, d)\nbc := line(b, c)\nbd := line(b, d)\n\
        fa := plane(u: bd, v: bc)\nfb := plane(u: ac, v: ad)\nfc := plane(u: ad, v: ab)\nfd := plane(u: ab, v: ac)\n\
        tetrahedron := solid({ p |\n  p distance(<= 0mm, along: n) fa\n  p distance(<= 0mm, along: n) fb\n\
          p distance(<= 0mm, along: n) fc\n  p distance(<= 0mm, along: n) fd\n})\n";
    for (name, src) in [("a join", TETRAHEDRON), ("half-spaces", planes)] {
        let e = solved(src);
        let s = ent(&e, "tetrahedron").i();
        let started = std::time::Instant::now();
        let f = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
        let made = started.elapsed();
        let n = 1_000_000;
        let started = std::time::Instant::now();
        let mut inside = 0;
        for i in 0..n {
            let t = i as f64 / n as f64;
            let p = [8.0 * (t * 7.0).fract() - 4.0, 8.0 * (t * 13.0).fract() - 4.0, 8.0 * t - 4.0];
            inside += usize::from(f.side(p) < 0.0);
        }
        let read = started.elapsed();
        let started = std::time::Instant::now();
        let x = e.sketch.exact_solid(s).unwrap();
        eprintln!("{name}: made in {made:?}; {:.3} µs a point ({inside} of {n} inside); built in {:?}, {} faces",
            read.as_secs_f64() * 1e6 / n as f64, started.elapsed(), x.brep.faces.len());
    }
}

/// What the language says with the join, beyond the cases it was made for: a reusable word, an
/// octahedron (a square on its corners joined to its axis), a prismatoid (that square joined to a
/// smaller one turned a quarter from it, 12 above), an oblique prism (two triangles, each a
/// segment joined to a point), written across lines.
const WORDS: &str = "unit mm\nuse std\nuse std.sets\n\
    component Join(A: set, B: set) := { p | some q in A, r in B: p in std.sets.Segment(q, r) }\n\
    s0 := point\ns1 := point\ns2 := point\ns3 := point\ntop := point\nbottom := point\n\
    fix((10, 0, 0)) s0\nfix((0, 10, 0)) s1\nfix((-10, 0, 0)) s2\nfix((0, -10, 0)) s3\n\
    fix((0, 0, 10)) top\nfix((0, 0, -10)) bottom\n\
    s01 := std.sets.Segment(s0, s1)\ns23 := std.sets.Segment(s2, s3)\naxis := std.sets.Segment(top, bottom)\n\
    sides := Join(s01, s23)\n\
    octahedron := solid(Join(sides, axis))\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((4, 4, 12)) b0\nfix((-4, 4, 12)) b1\nfix((-4, -4, 12)) b2\nfix((4, -4, 12)) b3\n\
    roof := { x | some u in std.sets.Segment(b0, b1),\n  v in std.sets.Segment(b2, b3):\n  x in std.sets.Segment(u, v) }\n\
    prismatoid := solid({ p | some q in Join(s01, s23), r in roof:\n  p in std.sets.Segment(q, r) })\n\
    t0 := point\nt1 := point\nt2 := point\nu0 := point\nu1 := point\nu2 := point\n\
    fix((0, 0, 30)) t0\nfix((10, 0, 30)) t1\nfix((0, 8, 30)) t2\nfix((4, 3, 40)) u0\nfix((14, 3, 40)) u1\nfix((4, 11, 40)) u2\n\
    low := { x | some u in std.sets.Segment(t0, t1): x in std.sets.Segment(u, t2) }\n\
    high := { x | some u in std.sets.Segment(u0, u1): x in std.sets.Segment(u, u2) }\n\
    prism := solid(Join(low, high))\n";

/// **The join is a word, and remixes**: written once as a component, it makes an octahedron, a
/// prismatoid (one operand the word, the other a set across lines; `h (A₁ + A₂ + 4 A_m) / 6`, its
/// middle section the two squares' Minkowski mean, of area `(200 + 2 · 160 + 64) / 4`) and an
/// oblique prism — each to its closed form, built exactly.
#[test]
fn the_join_is_a_word_that_makes_other_polytopes() {
    let e = solved(WORDS);
    for (name, faces, v) in [("octahedron", 8, 4000.0 / 3.0), ("prismatoid", 10, 12.0 / 6.0 * (200.0 + 64.0 + 4.0 * 146.0)),
                             ("prism", 5, 40.0 * 10.0)] {
        let x = exact(&e, name);
        assert_eq!(x.brep.faces.len(), faces, "{name}");
        close(name, volume(&x.brep), v);
    }
}

/// A box 20 × 12 × 6 on the top plane, a peak 8 over its top, and an L.
const SOLIDS: &str = "unit mm\nuse std\nuse std.sets\nuse std.solids\nin std.top {\n\
    k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
    fix((0, 0)) k0\nfix((20, 0)) k1\nfix((20, 12)) k2\nfix((0, 12)) k3\n\
    (f0 := line(k0, k1)) -> (f1 := line(k1, k2)) -> (f2 := line(k2, k3)) -> (f3 := line(k3, k0)) -> close\n\
    plate := face(f0, f1, f2, f3)\n\
    l0 := point\nl1 := point\nl2 := point\nl3 := point\nl4 := point\nl5 := point\n\
    fix((40, 0)) l0\nfix((60, 0)) l1\nfix((60, 10)) l2\nfix((50, 10)) l3\nfix((50, 20)) l4\nfix((40, 20)) l5\n\
    (g0 := line(l0, l1)) -> (g1 := line(l1, l2)) -> (g2 := line(l2, l3)) -> (g3 := line(l3, l4)) -> \
    (g4 := line(l4, l5)) -> (g5 := line(l5, l0)) -> close\n\
    ell := face(g0, g1, g2, g3, g4, g5)\n}\n\
    construction box := solid(std.solids.Prism(plate, from: 0mm, to: 6mm))\n\
    construction step := solid(std.solids.Prism(ell, from: 0mm, to: 6mm))\n\
    peak := point\nfix((10, 6, 14)) peak\nspire := point\nfix((45, 5, 14)) spire\n\
    house := solid({ p | some q in box: p in std.sets.Segment(peak, q) })\n\
    rounded := solid({ p | some q in box: p distance(<= 2mm) q })\n\
    tower := solid({ p | some q in step: p in std.sets.Segment(spire, q) })\n";

/// **A solid is a set a join reads**: a box joined to a point over it is a house, walls and roof
/// in one, the hull of the box's corners and the peak; a ball about some point of the box is the
/// box grown (`Grown`'s set, Steiner's); an L is refused by name, its hull another set.
#[test]
fn a_join_of_a_convex_solid_is_its_corners_hull() {
    let e = solved(SOLIDS);
    let x = exact(&e, "house");
    assert_eq!(x.brep.faces.len(), 9);
    close("house", volume(&x.brep), 20.0 * 12.0 * 6.0 + 20.0 * 12.0 * 8.0 / 3.0);
    let (a, b, c, r) = (20.0, 12.0, 6.0, 2.0);
    let steiner = a * b * c + 2.0 * (a * b + b * c + c * a) * r + PI * r * r * (a + b + c) + 4.0 / 3.0 * PI * r.powi(3);
    close("rounded", volume(&exact(&e, "rounded").brep), steiner);
    let why = MaterialField::read(&e.sketch, ent(&e, "tower").i(), 1e-10).err().unwrap_or_default();
    assert!(why.contains("every edge convex"), "{why}");
}

/// **Joins combine as solids do**: two tetrahedra, one turned through the other, united — the
/// stella octangula, twice 72 less the octahedron they share — and the same union said with `or`
/// in one set's body, read by the field within the box of both; and the bench's grown tetrahedron
/// as its description writes it, a ball about some point of the join.
#[test]
fn joins_combine_as_solids_do() {
    let src = "unit mm\nuse std\nuse std.sets\nuse std.solids (union)\n\
        component Join(A: set, B: set) := { p | some q in A, r in B: p in std.sets.Segment(q, r) }\n\
        a := point\nb := point\nc := point\nd := point\n\
        fix((3, 3, 3)) a\nfix((3, -3, -3)) b\nfix((-3, 3, -3)) c\nfix((-3, -3, 3)) d\n\
        a2 := point\nb2 := point\nc2 := point\nd2 := point\n\
        fix((-3, -3, -3)) a2\nfix((-3, 3, 3)) b2\nfix((3, -3, 3)) c2\nfix((3, 3, -3)) d2\n\
        ab := std.sets.Segment(a, b)\ncd := std.sets.Segment(c, d)\n\
        ab2 := std.sets.Segment(a2, b2)\ncd2 := std.sets.Segment(c2, d2)\n\
        t1 := Join(ab, cd)\nt2 := Join(ab2, cd2)\n\
        one := solid(t1)\nother := solid(t2)\nother union one\n\
        star := solid({ p | p in t1 or p in t2 })\n\
        construction tet := solid(Join(ab, cd))\n\
        grown := solid({ p | some q in tet: p distance(<= 1mm) q })\n";
    let e = solved(src);
    let united = e.sketch.exact_solid(ent(&e, "one").i()).unwrap();
    close("united", volume(&united.brep), 2.0 * 72.0 - 36.0);
    let f = MaterialField::read(&e.sketch, ent(&e, "star").i(), 1e-10).unwrap();
    assert!(f.side([2.0, 2.0, 2.0]) < 0.0 && f.side([-2.0, -2.0, -2.0]) < 0.0 && f.side([0.0, 0.0, 4.0]) > 0.0);
    close("grown", volume(&exact(&e, "grown").brep), 249.53323018738044);
}
