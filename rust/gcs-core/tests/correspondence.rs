//! **Rails and correspondence** (#243 L8): a loft between sections drawn apart is said in the
//! library, over three general features of the language, each with its use outside lofts:
//!
//! * **a block walks a face** — `repeat e in f`, `cycle e in f` — its edges in order, a face of
//!   corners by the straight run from each to the next;
//! * **walks run in step** — `cycle (ea, eb) in (a, b)` — a link of each in every copy, so two
//!   faces' edges pair by their places in the walks;
//! * **a face of a block's copies** — `face(side)` — as many edges as the block made, the count
//!   never written.
//!
//! Over them, `std.Ruled(a, b, s:)` joins two faces corner to corner by straight rails and
//! `std.Apex(a, tip, s:)` a face to a point, each standing its section where the plane through
//! three rails at `s` cuts every rail; `std.Similar` and `std.Affine` carry a profile drawn once
//! onto two or three rails, each corner at its own place in the frame they make, read off the
//! drawing (a station's number reading the geometry it was given, any expression of it).
//! Correspondence is the walks' written order: a connector is where a walk starts, and faces of
//! different corner counts are made alike by naming a point on an edge — a statement, never
//! inferred.  Gated off the exact B-rep to a part in 10⁹: a ruled loft between parallel planes is
//! its prismatoid, a loft to a point a pyramid, and a profile on rails ∫A ds with A the profile's
//! area times the square of its scale, or its frame's area ratio.
use gcs_core::program::Elaborated;

use crate::common::parsed::determined;
use crate::common::refused;

const TWO_FACES: &str = include_str!("../../../docs/lofts/l0/two_faces.sv");

/// A square inlet in `std.front` and the planes and corners of the faces it is lofted to, 40 mm
/// on, square to `y`: `OUTLET` a diamond, `SQUARE` a square, `TRIANGLE` a triangle.
const FRONT: &str = "unit mm
use std

end_plane := plane
fix(origin == (0mm, 40mm, 0mm)) end_plane
fix(dir == (1, 0, 0)) end_plane.u
fix(dir == (0, 0, 1)) end_plane.v
in std.front {
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix((-10mm, -10mm)) a0
  fix((10mm, -10mm)) a1
  fix((10mm, 10mm)) a2
  fix((-10mm, 10mm)) a3
}
inlet := face(a0, a1, a2, a3, -> close)
";

const OUTLET: &str = "in end_plane {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0mm, -12mm)) b0
  fix((12mm, 0mm)) b1
  fix((0mm, 12mm)) b2
  fix((-12mm, 0mm)) b3
}
";

/// A triangle in the end plane, its top edge `t1`–`t2`'s middle `m` named so it can stand in a
/// walk of four.
const TRIANGLE: &str = "in end_plane {
  t0 := point
  t1 := point
  t2 := point
  m := point
  fix((-10mm, -10mm)) t0
  fix((10mm, -10mm)) t1
  fix((0mm, 10mm)) t2
  half := line(t2, t0)
  m coincident(t == 0.5) half
}
";

/// A blade 50 mm long between two rails, its section a 20 × 3 rectangle drawn once in
/// `std.front` and carried onto the rails by `std.Similar` from the middles of its short sides:
/// the rails' places in the station are where the profile's `p0` and `p1` go.  `RAILS_IN_PLAN`
/// close in from 20 apart to 8 in plan; `RAILS_TURNING` the same, one rising 3 and the other
/// falling 3, so the section turns as it narrows.
const BLADE: &str = "unit mm
use std

component Blade(g: line, up: axis, port: line, starboard: line, prof: face, from: point,
                to: point, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  l := point in st.n
  r := point in st.n
  l coincident port
  r coincident starboard
  sec := std.Similar(prof, from, to, st.n, l, r)
}

in std.top {
  o := point
  fix((0mm, 0mm)) o
  tip := point
  fix((0mm, 50mm)) tip
  spine := line(o, tip)
}
in std.front {
  p0 := point
  p1 := point
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((-10mm, 100mm)) p0
  fix((10mm, 100mm)) p1
  fix((-10mm, 98.5mm)) k0
  fix((10mm, 98.5mm)) k1
  fix((10mm, 101.5mm)) k2
  fix((-10mm, 101.5mm)) k3
}
profile := face(k0, k1, k2, k3, -> close)
l0 := point
l1 := point
r0 := point
r1 := point
port_rail := line(l0, l1)
starboard_rail := line(r0, r1)
blade := solid(Blade(spine, std.z, port_rail, starboard_rail, profile, p0, p1).sec.f
               over s in (0, 1))
";

const RAILS_IN_PLAN: &str = "fix((-10mm, 0mm, 0mm)) l0
fix((-4mm, 50mm, 0mm)) l1
fix((10mm, 0mm, 0mm)) r0
fix((4mm, 50mm, 0mm)) r1
";

const RAILS_TURNING: &str = "fix((-10mm, 0mm, 0mm)) l0
fix((-4mm, 50mm, 3mm)) l1
fix((10mm, 0mm, 0mm)) r0
fix((4mm, 50mm, -3mm)) r1
";

/// A square 6 mm across, a named chain, carried by `std.Affine` onto three rails in space from
/// the frame `o`, `x`, `y` it is drawn in: the rails run 40 mm up from the triangle (0, 0), (20,
/// 0), (0, 20) to (5, 5), (15, 0), (0, 25), and the station is the plane through them.
const AFFINE: &str = "unit mm
use std

component Tri(ra: path, rb: path, rc: path, prof: face, from: point, to: point, third: point,
              s: Scalar) {
  st := std.Across(ra, rb, rc, s: s)
  pa := point in st.n
  pb := point in st.n
  pc := point in st.n
  pa coincident ra
  pb coincident rb
  pc coincident rc
  sec := std.Affine(prof, from, to, third, st.n, pa, pb, pc)
}

a0 := point
b0 := point
c0 := point
a1 := point
b1 := point
c1 := point
fix((0mm, 0mm, 0mm)) a0
fix((20mm, 0mm, 0mm)) b0
fix((0mm, 20mm, 0mm)) c0
fix((5mm, 5mm, 40mm)) a1
fix((15mm, 0mm, 40mm)) b1
fix((0mm, 25mm, 40mm)) c1
ra := line(a0, a1)
rb := line(b0, b1)
rc := line(c0, c1)
in std.front {
  o := point
  x := point
  y := point
  fix((100mm, 0mm)) o
  fix((110mm, 0mm)) x
  fix((100mm, 10mm)) y
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((102mm, 2mm)) k0
  fix((108mm, 2mm)) k1
  fix((108mm, 8mm)) k2
  fix((102mm, 8mm)) k3
  sq := line(k0, k1) -> line(k1, k2) -> line(k2, k3) -> line(k3, k0) -> close
}
prof := face(sq)
body := solid(Tri(ra, rb, rc, prof, o, x, y).sec.f over s in (0, 1))
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// The volume of the solid `name` off its exact B-rep, its boundary checked.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// The names of the faces of the exact solid `name`.
fn faces(e: &Elaborated, name: &str) -> std::collections::BTreeSet<String> {
    let x = e.sketch.exact_solid(index(e, name)).unwrap();
    x.brep.faces.iter().map(|f| f.name.clone()).collect()
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// The area of the polygon `p`, by the shoelace.
fn area(p: &[(f64, f64)]) -> f64 {
    let n = p.len();
    (0..n).map(|i| p[i].0 * p[(i + 1) % n].1 - p[(i + 1) % n].0 * p[i].1).sum::<f64>().abs() / 2.0
}

/// The ruled loft between two polygons in parallel planes `h` apart, corner `k` to corner `k`:
/// its section's corners run linearly, so its area is quadratic in the running number and the
/// prismatoid is exact.
fn prismatoid(h: f64, a: &[(f64, f64)], b: &[(f64, f64)]) -> f64 {
    let mid: Vec<(f64, f64)> =
        a.iter().zip(b).map(|(p, q)| ((p.0 + q.0) / 2.0, (p.1 + q.1) / 2.0)).collect();
    h / 6.0 * (area(a) + 4.0 * area(&mid) + area(b))
}

const SQUARE: [(f64, f64); 4] = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)];
const DIAMOND: [(f64, f64); 4] = [(0.0, -12.0), (12.0, 0.0), (0.0, 12.0), (-12.0, 0.0)];

/// **Two faces drawn apart, lofted by the word** (`docs/lofts/l0/two_faces.sv`): the square and
/// the diamond joined corner to corner in written order, CAD's "loft these two sketches", is the
/// prismatoid 40/6 · 1856 — what the retired `solid(a, b, along: g)` measured.  Its walls are the
/// station's edges, `side[k]` from corner `k`, between the caps `start` and `end`.
#[test]
fn two_faces_drawn_apart_are_lofted_corner_to_corner() {
    let e = determined(TWO_FACES);
    near(volume(&e, "body"), 40.0 / 6.0 * 1856.0, 1e-9, "two faces");
    near(prismatoid(40.0, &SQUARE, &DIAMOND), 40.0 / 6.0 * 1856.0, 1e-12, "the closed form");
    let want: std::collections::BTreeSet<String> =
        ["start", "end", "side[0]", "side[1]", "side[2]", "side[3]"]
            .iter().map(|w| format!("body.{w}")).collect();
    assert_eq!(faces(&e, "body"), want);
}

/// **A connector is where a walk starts**: the diamond walked from its second corner pairs the
/// square's first corner with it, a quarter turn on — the section twists through the run and its
/// mid section is smaller, but the corners still run linearly, so the prismatoid holds.
#[test]
fn a_connector_is_where_a_walk_starts() {
    let src = format!("{FRONT}{OUTLET}outlet := face(b1, b2, b3, b0, -> close)
body := solid(std.Ruled(inlet, outlet).f over s in (0, 1))
");
    let e = determined(&src);
    let turned = [DIAMOND[1], DIAMOND[2], DIAMOND[3], DIAMOND[0]];
    let want = prismatoid(40.0, &SQUARE, &turned);
    near(want, 40.0 / 6.0 * (400.0 + 4.0 * 52.0 + 288.0), 1e-12, "the closed form");
    near(volume(&e, "body"), want, 1e-9, "a quarter turn on");
}

/// **Faces of different corner counts are made alike by a statement**: a triangle walked with
/// the middle of its top edge named as a corner has four, and lofts to the square at the
/// prismatoid; walked as three it is refused, saying how to make the walks alike.
#[test]
fn an_edge_split_in_the_walk_pairs_faces_of_different_counts() {
    let src = format!("{FRONT}{TRIANGLE}tri := face(t0, t1, t2, m, -> close)
body := solid(std.Ruled(inlet, tri).f over s in (0, 1))
");
    let e = determined(&src);
    let tri = [(-10.0, -10.0), (10.0, -10.0), (0.0, 10.0), (-5.0, 0.0)];
    near(volume(&e, "body"), prismatoid(40.0, &SQUARE, &tri), 1e-9, "triangle split");
    let src = format!("{FRONT}{TRIANGLE}tri := face(t0, t1, t2, -> close)
body := solid(std.Ruled(inlet, tri).f over s in (0, 1))
");
    refused(&src, "E080", "`inlet` walks 4 links and `tri` 3", "use std");
}

/// **A loft to a point** (`std.Apex`): the square's corners joined to a tip 30 mm off its plane
/// and off its centre is a pyramid, a third of base by height whatever it leans, with no cap at
/// the tip.
#[test]
fn a_loft_to_a_point_is_a_pyramid() {
    let src = format!("{FRONT}tip := point
fix((3mm, 30mm, 2mm)) tip
body := solid(std.Apex(inlet, tip).f over s in (0, 1))
");
    let e = determined(&src);
    near(volume(&e, "body"), 400.0 * 30.0 / 3.0, 1e-9, "pyramid");
    let f = faces(&e, "body");
    assert!(f.contains("body.start") && !f.contains("body.end"), "{f:?}");
}

/// **Two rails hold a section by a similarity** (`std.Similar`): a 20 × 3 profile carried onto
/// rails closing from 20 apart to 8 is scaled by their spread, `k = (20 − 12s)/20`, thickness and
/// all — ∫60k² · 50 ds = 1560 — and on rails that rise and fall as they close it turns as well,
/// its spread `√((20 − 12s)² + (6s)²)` — 1650.
#[test]
fn two_rails_hold_a_section_by_a_similarity() {
    let e = determined(&format!("{BLADE}{RAILS_IN_PLAN}"));
    near(volume(&e, "blade"), 3000.0 * (1.0 - 0.6 + 0.12), 1e-9, "rails in plan");
    let e = determined(&format!("{BLADE}{RAILS_TURNING}"));
    let spread2 = 400.0 - 240.0 + 48.0 + 12.0; // ∫(20 − 12s)² + 36s² ds
    near(volume(&e, "blade"), 60.0 * 50.0 / 400.0 * spread2, 1e-9, "rails turning");
}

/// **Three rails hold it by an affine map** (`std.Affine`): a 6 mm square drawn in the frame `o`,
/// `x`, `y` (its area 36 to the frame's 50) is carried onto three rails whose triangle runs from
/// area 200 to 87.5 between parallel planes, so its area is 36/50 of the triangle's, quadratic in
/// `s`: the prismatoid over the triangle's 200, 146.875 and 87.5.
#[test]
fn three_rails_hold_a_section_by_an_affine_map() {
    let e = determined(AFFINE);
    let k = 36.0 / 50.0;
    near(volume(&e, "body"), 40.0 / 6.0 * k * (200.0 + 4.0 * 146.875 + 87.5), 1e-9, "affine");
}

/// **A block walks a face**, outside any loft: a hole at the middle of every edge of a face of
/// corners, sized by its index — the runs a face of corners makes are the walk's edges.
#[test]
fn a_block_walks_a_face() {
    let e = determined("unit mm
use std
in std.front {
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) a
  fix((60mm, 0mm)) b
  fix((0mm, 40mm)) c
  plate := face(a, b, c, -> close)
  repeat e in plate as i {
    m := point
    m midpoint e
    hole := circle(center: m)
    radius(2mm + i * 1mm) hole
  }
}
");
    let points: Vec<(f64, f64)> =
        (0..e.sketch.points.len()).map(|i| e.sketch.point_xy(i)).collect();
    for (x, y) in [(30.0, 0.0), (30.0, 20.0), (0.0, 20.0)] {
        assert!(points.iter().any(|p| (p.0 - x).abs() < 1e-9 && (p.1 - y).abs() < 1e-9),
                "no middle at ({x}, {y})");
    }
    let mut radii: Vec<f64> =
        e.sketch.circles.iter().map(|c| e.sketch.params[c.radius as usize].value).collect();
    radii.sort_by(f64::total_cmp);
    assert!(radii.iter().zip([2.0, 3.0, 4.0]).all(|(r, w)| (r - w).abs() < 1e-9), "{radii:?}");
}

/// **A face of a block's copies**: `face(e)` is the face of `e[0]`, `e[1]`, … as the block made
/// them, so a regular hexagon drawn by a `cycle` is a face without its count written twice —
/// swept 10 mm, (3√3/2)·10² · 10.
#[test]
fn a_face_of_a_blocks_copies() {
    let e = determined("unit mm
use std (coords)
in std.front {
  c := point
  fix((0mm, 0mm)) c
  cycle 6 as i {
    p := point
    p coords(du: 10mm * cos(60deg * i), dv: 10mm * sin(60deg * i)) std.front
    e := line(p, next.p)
  }
  hex := face(e)
}
prism := solid(hex, depth: 10mm)
");
    near(volume(&e, "prism"), 1.5 * 3f64.sqrt() * 100.0 * 10.0, 1e-9, "hexagonal prism");
}

/// **What a walk refuses**, in words: a face with holes (one loop is walked), corners mixed with
/// edges, two walks of different lengths in step, and a `cycle` over an open chain in step.
#[test]
fn walks_are_refused_in_words() {
    let holed = "unit mm
use std
in std.front {
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) a
  fix((60mm, 0mm)) b
  fix((0mm, 40mm)) c
  d := point
  fix((10mm, 10mm)) d
  k := circle(center: d)
  radius(2mm) k
  plate := face(a, b, c, holes: k, -> close)
  repeat e in plate {
    m := point
    m midpoint e
  }
}
";
    refused(holed, "E080", "has holes, and a block walks one loop", "plate");
    let mixed = "unit mm
use std
in std.front {
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) a
  fix((60mm, 0mm)) b
  fix((0mm, 40mm)) c
  ab := line(a, b)
  plate := face(ab, c, -> close)
  repeat e in plate {
    m := point
    m midpoint e
  }
}
";
    refused(mixed, "E080", "both corners and edges", "plate");
    let open = "unit mm
use std
in std.front {
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) a
  fix((60mm, 0mm)) b
  fix((0mm, 40mm)) c
  one := line(a, b) -> line(b, c)
  two := line(c, b) -> line(b, a)
  cycle (x, y) in (one, two) {
    m := point
    m midpoint x
  }
}
";
    refused(open, "E103", "is an open chain, and a `cycle` closes", "one");
}

/// **Walks in step are written alike**: as many names as walks, bracketed both sides.
#[test]
fn walks_in_step_are_written_alike() {
    let said = |src: &str| {
        let (_, errs) = crate::common::parse(src);
        errs.iter().map(|e| e.message.clone()).collect::<Vec<_>>().join("\n")
    };
    let head = "unit mm\nuse std\n";
    let s = said(&format!("{head}cycle (x, y) in (a, b, c) {{\n  m := point\n}}\n"));
    assert!(s.contains("2 names for 3 walks"), "{s}");
    let s = said(&format!("{head}cycle (x, y) in a {{\n  m := point\n}}\n"));
    assert!(s.contains("bracketed alike"), "{s}");
}
