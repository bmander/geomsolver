//! **A point carried by a map** (#357): `q mapped(from -> a, to -> b, third -> c) p` holds `q`
//! where the affine map taking the three source points to the three targets carries `p`, and
//! two pairs the similarity.  Each side is read in its own plane, so a point drawn in one view is
//! carried into another.  Held here to closed forms in a plain sketch: a triangle's centroid goes
//! to the image triangle's centroid, a similarity turns and scales, the mirror reflects, and a
//! source frame standing in a line is refused where it is stated (E061, naming its points).
use crate::common::parsed::{determined, solved};
use crate::common::refused;
use gcs_core::program::Elaborated;

/// A triangle drawn in the top view and its image in the front view, nothing between them but
/// what the map states.
const TRIANGLES: &str = "\
unit mm
use std
in std.top {
  s0 := point
  s1 := point
  s2 := point
  fix((0, 0)) s0
  fix((30, 0)) s1
  fix((6, 18)) s2
  g := point
  fix((12, 6)) g
}
in std.front {
  t0 := point
  t1 := point
  t2 := point
  fix((100, 10)) t0
  fix((140, 25)) t1
  fix((90, 60)) t2
  q := point
}
";

fn page(e: &Elaborated, n: &str) -> [f64; 2] {
    let p = e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`")).i();
    let pp = e.sketch.point_params(p);
    [e.sketch.params[pp[0] as usize].value, e.sketch.params[pp[1] as usize].value]
}

fn near(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9
}

/// **A triangle's centroid maps to the image triangle's centroid**: `g` is the centroid of the
/// source triangle, so `q` is the centroid of the targets, whatever the map stretches.
#[test]
fn a_centroid_maps_to_the_centroid() {
    let src = format!("{TRIANGLES}q mapped(s0 -> t0, s1 -> t1, s2 -> t2) g\n");
    let e = determined(&src);
    let c = [(100.0 + 140.0 + 90.0) / 3.0, (10.0 + 25.0 + 60.0) / 3.0];
    assert!(near(page(&e, "q"), c), "{:?} against {c:?}", page(&e, "q"));
}

/// **Two pairs are the similarity**: the source's `s0 → s1` taken to `t0 → t1`, turned and
/// scaled, a point a quarter turn off it the same quarter turn off the image; and `mirror: true`
/// the quarter turn the other way.
#[test]
fn two_pairs_turn_and_scale_and_the_mirror_reflects() {
    // `g` at (12, 6) stands 0.4 along `s0 → s1` (30 long) and 0.2 a quarter turn left of it
    let e = solved(&format!("{TRIANGLES}q mapped(s0 -> t0, s1 -> t1) g\n"));
    let (i, j) = (0.4, 0.2);
    let (ex, ey) = (40.0, 15.0);
    let want = [100.0 + i * ex - j * ey, 10.0 + i * ey + j * ex];
    assert!(near(page(&e, "q"), want), "{:?} against {want:?}", page(&e, "q"));
    let e = solved(&format!("{TRIANGLES}q mapped(s0 -> t0, s1 -> t1, mirror: true) g\n"));
    let want = [100.0 + i * ex + j * ey, 10.0 + i * ey - j * ex];
    assert!(near(page(&e, "q"), want), "{:?} against {want:?}", page(&e, "q"));
}

/// **The map's pairs are written back as they were**: the printer pairs each source with its
/// target.
#[test]
fn a_map_prints_as_written() {
    let src = format!("{TRIANGLES}q mapped(s0 -> t0, s1 -> t1, mirror: true) g\n");
    let (mut prog, errs, _) = gcs_core::library::parse_linked(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let text = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert!(text.contains("q mapped(s0 -> t0, s1 -> t1, mirror: true) g"), "{text}");
    // and a face carried whole, as the definition it is
    let src = format!("{RECT}rev := edged mapped(o -> a, x -> b, mirror: true) in std.top\n");
    let (mut prog, errs, _) = gcs_core::library::parse_linked(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let text = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    let said = "rev := edged mapped(o -> a, x -> b, mirror: true) in std.top";
    assert!(text.contains(said), "{text}");
    let (mut again, errs, _) = gcs_core::library::parse_linked(&text);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(gcs_core::syntax::render_flat(&mut again).unwrap(), text);
}

/// **A source frame standing in a line is refused where it is stated**, naming its three points:
/// it has no place for a point off the line, and its rows would divide by zero.
#[test]
fn a_collinear_frame_is_refused() {
    let src = TRIANGLES.replace("fix((6, 18)) s2", "fix((60, 0)) s2")
        + "q mapped(s0 -> t0, s1 -> t1, s2 -> t2) g\n";
    let at = "q mapped(s0 -> t0, s1 -> t1, s2 -> t2) g";
    refused(&src, "E061", "s0, s1 and s2 stand in a line", at);
}

/// And the parentheses hold pairs: a lone reference, a fourth pair, `mirror:` with three.
#[test]
fn a_map_takes_two_or_three_pairs() {
    let three = |tail: &str| format!("{TRIANGLES}q mapped(s0 -> t0, s1 -> t1, s2 -> t2{tail}) g\n");
    refused(&three(", mirror: true"), "E040", "`mirror:` reverses the similarity", "mirror");
    let one = format!("{TRIANGLES}q mapped(s0 -> t0) g\n");
    refused(&one, "E040", "two or three pairs", "mapped(s0 -> t0)");
}

/// A 10 × 6 rectangle in the front view, its frame `o`, `x`: of corners (`rect`) and of named
/// edges walked counter-clockwise (`edged`), and the two targets `a`, `b` in the top view, 20
/// apart, so the similarity doubles every length.
const RECT: &str = "\
unit mm
use std
in std.front {
  o := point
  x := point
  fix((0, 0)) o
  fix((10, 0)) x
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0, 0)) k0
  fix((10, 0)) k1
  fix((10, 6)) k2
  fix((0, 6)) k3
  ab := line(k0, k1)
  bc := line(k1, k2)
  cd := line(k2, k3)
  da := line(k3, k0)
}
rect := face(k0, k1, k2, k3, -> close)
edged := face(ab, bc, cd, da)
in std.top {
  a := point
  b := point
  fix((100, 0)) a
  fix((100, 20)) b
  c := point
  fix((120, 0)) c
}
";

/// A "D" in the front view, a flat 10 long and the half disc over it, and a disc of radius 3.
const DEE: &str = "\
in std.front {
  m := point
  fix((5, 0)) m
  dome := arc(center: m, start: k1, end: k0)
  disc := circle(center: m)
  radius(3mm) disc
}
dee := face(ab, dome)
round := face(disc)
";

/// **A face carried whole** (M2): the rectangle of corners mapped by the similarity taking `o`
/// to `a` and `x` to `b` is twice the size, turned a quarter, and its walls are the source's own
/// — the closing runs a face of corners names `close0`, … — and the same face of named edges
/// keeps their names.
#[test]
fn a_face_is_carried_whole_its_walls_named_as_the_sources() {
    let names = |e: &Elaborated| faces(e, "body");
    let e = determined(&format!("{RECT}sec := rect mapped(o -> a, x -> b) in std.top\n\
                                 body := solid(sec, depth: 5mm)\n"));
    assert!((volume(&e, "body") - 20.0 * 12.0 * 5.0).abs() < 1e-9, "{}", volume(&e, "body"));
    let want = ["near", "far", "close0", "close1", "close2", "close3"];
    assert_eq!(names(&e), want.iter().map(|w| format!("body.{w}")).collect());
    // the corners keep their names, under the image's
    assert!(near(page(&e, "sec.k2"), [100.0 - 12.0, 20.0]), "{:?}", page(&e, "sec.k2"));
    let e = determined(&format!("{RECT}sec := edged mapped(o -> a, x -> b) in std.top\n\
                                 body := solid(sec, depth: 5mm)\n"));
    assert!((volume(&e, "body") - 1200.0).abs() < 1e-9, "{}", volume(&e, "body"));
    let want = ["near", "far", "ab", "bc", "cd", "da"];
    assert_eq!(names(&e), want.iter().map(|w| format!("body.{w}")).collect());
    // a corner named by its path is its image's by the same path
    let e = determined(&format!("{RECT}tri := face(ab.p1, ab.p2, bc.p2, -> close)\n\
                                 sec := tri mapped(o -> a, x -> b) in std.top\n\
                                 body := solid(sec, depth: 5mm)\n"));
    assert!((volume(&e, "body") - 600.0).abs() < 1e-9, "{}", volume(&e, "body"));
    assert!(near(page(&e, "sec.bc.p2"), [88.0, 20.0]), "{:?}", page(&e, "sec.bc.p2"));
}

/// **Arcs and circles under a similarity** keep their kind, the radius scaled by `|b − a| / |x −
/// o|`: the D's half disc of radius 5 is one of 10, the disc of 3 one of 6, each solid its
/// closed form; and under three pairs an arc would be an ellipse, refused naming it.
#[test]
fn an_arc_under_a_similarity_is_an_arc_its_radius_scaled() {
    let src = format!("{RECT}{DEE}sec := dee mapped(o -> a, x -> b) in std.top\n\
                       disc2 := round mapped(o -> a, x -> b) in std.top\n\
                       body := solid(sec, depth: 5mm)\nrod := solid(disc2, depth: 5mm)\n");
    let e = determined(&src);
    let radius = |name: &str| {
        let r = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
        let p = match r.kind {
            gcs_core::model::EntKind::Arc => e.sketch.arcs[r.i()].radius,
            _ => e.sketch.circles[r.i()].radius,
        };
        e.sketch.params[p as usize].value
    };
    assert!((radius("sec.dome") - 10.0).abs() < 1e-9, "{}", radius("sec.dome"));
    assert!((radius("disc2.disc") - 6.0).abs() < 1e-9, "{}", radius("disc2.disc"));
    let pi = std::f64::consts::PI;
    assert!((volume(&e, "body") - pi * 100.0 / 2.0 * 5.0).abs() < 1e-6, "{}", volume(&e, "body"));
    assert!((volume(&e, "rod") - pi * 36.0 * 5.0).abs() < 1e-6, "{}", volume(&e, "rod"));
    let sheared = format!("{RECT}{DEE}sec := dee mapped(o -> a, x -> b, k3 -> c) in std.top\n");
    refused(&sheared, "E080", "`dome` is an arc, and under a map of three pairs a circle is an \
        ellipse", "mapped");
    let circle = format!("{RECT}{DEE}in std.top {{\n  k := circle\n}}\n\
                          k mapped(o -> a, x -> b, k3 -> c) disc\n");
    refused(&circle, "E040", "a circle is an ellipse", "mapped(o -> a, x -> b, k3 -> c)");
}

/// The corners of a face's loop as it is walked, each where an edge meets the next, on its page.
fn corners(e: &Elaborated, face: &str) -> Vec<[f64; 2]> {
    let f = &e.sketch.faces[index(e, face)];
    let ends = |r: gcs_core::model::EntRef| match r.kind {
        gcs_core::model::EntKind::Line => {
            let l = &e.sketch.lines[r.i()];
            [l.p1, l.p2]
        }
        _ => {
            let a = &e.sketch.arcs[r.i()];
            [a.start, a.end]
        }
    };
    let n = f.edges.len();
    (0..n).map(|k| {
        let (here, next) = (ends(f.edges[k]), ends(f.edges[(k + 1) % n]));
        let p = *here.iter().find(|p| next.contains(p)).expect("an edge meets the next");
        let pp = e.sketch.point_params(p as usize);
        [e.sketch.params[pp[0] as usize].value, e.sketch.params[pp[1] as usize].value]
    }).collect()
}

/// Twice the signed area of a loop: positive counter-clockwise.
fn turn(c: &[[f64; 2]]) -> f64 {
    (0..c.len()).map(|k| {
        let (p, q) = (c[k], c[(k + 1) % c.len()]);
        p[0] * q[1] - q[0] * p[1]
    }).sum()
}

/// **The mirror** (M3): `mirror: true` reflects the face, and the image is walked the other way
/// round, so it winds counter-clockwise as its source does — the flip a mirrored frame needed
/// (#309, #330), settled once, in the map — and the solid swept from it is the source's twin.
#[test]
fn a_mirrored_face_winds_as_its_source() {
    let e = determined(&format!("{RECT}sec := edged mapped(o -> a, x -> b) in std.top\n\
                                 rev := edged mapped(o -> a, x -> b, mirror: true) in std.top\n\
                                 body := solid(rev, depth: 5mm)\n"));
    assert!(turn(&corners(&e, "edged")) > 0.0);
    assert!(turn(&corners(&e, "sec")) > 0.0);
    assert!(turn(&corners(&e, "rev")) > 0.0, "{:?}", corners(&e, "rev"));
    // reflected: the corner a quarter turn left of `o → x` is a quarter turn right of `a → b`
    let has = |face: &str, at: [f64; 2]| corners(&e, face).iter().any(|&c| near(c, at));
    assert!(has("sec", [88.0, 20.0]) && has("rev", [112.0, 20.0]), "{:?}", corners(&e, "rev"));
    assert!((volume(&e, "body") - 1200.0).abs() < 1e-9, "{}", volume(&e, "body"));
}

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

