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
