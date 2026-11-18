//! **A cut list** (#940, `solventc --parts`): each part, how many, and the box it is cut from, its
//! sides longest first — square to the part's own faces however it stands in the assembly.

use gcs_core::stock::{cut_list, oriented};

/// A board drawn in whatever frame it is given, and three of them: two level, one pitched 25°.
const BOARDS: &str = "\
unit in
use std
use std.solids
component Board(f: group, len: Length) {
  in f.front {
    a := point
    b := point hint((len, 0))
    c := point hint((len, 3.5))
    d := point hint((0, 3.5))
    fix((0, 0)) a
    fix((len, 0)) b
    fix((len, 3.5)) c
    fix((0, 3.5)) d
    ab := line(a, b)
    bc := line(b, c)
    cd := line(c, d)
    da := line(d, a)
  }
  body := solid(std.solids.Prism(face(ab, bc, cd, da), from: -1.5in, to: 0in))
}
in std.front {
  o := point
  fix((0, 20)) o
  t := point
  fix((10, 24.6631)) t
  pitched := std.Turned(o, t)
}
level := { front: std.front }
tilt := { front: pitched.axes }
one := Board(level, len: 96in)
two := Board(level, len: 96in)
rafter := Board(tilt, len: 72in)
";

/// **Each board once, counted, at its true size**: the two level ones one row, the pitched one
/// its board's 72 × 3.5 × 1.5 and not the box round it in the world, and the text says each.
#[test]
fn a_cut_list_counts_each_part_at_its_own_size() {
    let (p, errs) = crate::common::parse(BOARDS);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = gcs_core::program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let parts = cut_list(&e.sketch).unwrap();
    let near = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);
    assert_eq!(parts.len(), 2, "{parts:?}");
    assert!(near(parts[0].size, [96.0, 3.5, 1.5]) && parts[0].objects == ["one.body", "two.body"],
            "{parts:?}");
    assert!(near(parts[1].size, [72.0, 3.5, 1.5]) && parts[1].objects == ["rafter.body"],
            "{parts:?}");
    assert_eq!(gcs_core::stock::text(&e.sketch, &parts), [
        "cut list: 2 parts, 3 objects (length × width × thickness, in)",
        "  2 × body: 96 × 3.5 × 1.5 — one.body, two.body",
        "  1 × rafter.body: 72 × 3.5 × 1.5 — rafter.body",
    ]);
}

/// **The smallest box square to the faces**: a box turned about two axes is boxed at its own
/// sides, and a body with no flat face (points alone) square to the world's axes.
#[test]
fn a_part_is_boxed_square_to_its_own_faces() {
    let (c, s) = (0.6f64.cos(), 0.6f64.sin());
    // a 10 × 4 × 2 box turned 0.6 rad about z, then tipped 0.3 rad about x
    let (c2, s2) = (0.3f64.cos(), 0.3f64.sin());
    let turn = |p: [f64; 3]| {
        let q = [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]];
        [q[0], c2 * q[1] - s2 * q[2], s2 * q[1] + c2 * q[2]]
    };
    let mut points = Vec::new();
    for i in 0..8 {
        let p = [if i & 1 == 0 { 0.0 } else { 10.0 }, if i & 2 == 0 { 0.0 } else { 4.0 },
                 if i & 4 == 0 { 0.0 } else { 2.0 }];
        points.push(turn(p));
    }
    let faces = [([1.0, 0.0, 0.0], 8.0), ([0.0, 1.0, 0.0], 20.0), ([0.0, 0.0, 1.0], 40.0),
                 ([-1.0, 0.0, 0.0], 8.0)].map(|(n, a)| (turn(n), a));
    let size = oriented(&points, faces.into_iter());
    assert!((size[0] - 10.0).abs() < 1e-9 && (size[1] - 4.0).abs() < 1e-9
            && (size[2] - 2.0).abs() < 1e-9, "{size:?}");
    let size = oriented(&points, std::iter::empty());
    assert!(size[0] > 10.0, "square to the world, the box round it: {size:?}");
}
