//! **A point on a curve drawn in a plane is in that plane** (#249, #243 L½): a spline or a traced
//! curve has no contact in space, so a point standing in space said `coincident` to one drawn in
//! `P` is drawn in `P` — membership, as `p coincident P` is (§6.21's one rule, `tests/membership`)
//! — and the statement is the 2D contact.  A line said `tangent` to one has its free ends drawn
//! there.  A point a statement keeps in space is refused with what keeps it.
use gcs_core::constraints::CKind;
use gcs_core::space::{cross, norm, sub};

use crate::common::{ent, involute_at, kinds, plane_of, read, refused, solved};

/// A cubic Bézier guide drawn in `view`, its control points held: at `t = ½` it passes (12.5,
/// 22.5) in the plane's coordinates, heading along (60, 40).
fn guide(view: &str) -> String {
    format!(
        "unit mm
use std
in {view} {{
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) k0
  fix((0mm, 20mm)) k1
  fix((20mm, 30mm)) k2
  fix((40mm, 30mm)) k3
  g := spline(k0, k1, k2, k3)
}}
"
    )
}

/// An involute of a circle of radius 10 about the origin of `view`, traced.
fn involute(view: &str) -> String {
    format!(
        "unit mm
use std
component Inv(c: circle, u: Angle) {{
  p := point(x: c.center.x + c.r * (cos(u) + u / 1rad * sin(u)), y: c.center.y + c.r * (sin(u) - u / 1rad * cos(u)))
}}
in {view} {{
  o := point
  fix((0mm, 0mm)) o
  b := circle(center: o)
  fix(r == 10mm) b
}}
e := Inv(b).p over u in (0deg, 90deg)
"
    )
}

/// A plane's coordinates `(a, b)` in space: `std.top` is (x, y), `std.side` (y, z) — said here,
/// not read off the plane's basis, so the test and what it tests share no arithmetic.
fn lifted(view: &str, (a, b): (f64, f64)) -> [f64; 3] {
    match view {
        "std.top" => [a, b, 0.0],
        "std.side" => [0.0, a, b],
        _ => unreachable!(),
    }
}

fn near(p: [f64; 3], q: [f64; 3]) -> bool {
    norm(sub(p, q)) < 1e-7
}

/// The issue's case: `m`, declared in space, on a spline drawn in a plane is drawn in that plane
/// and solved onto the curve there — the contact the plane's own, no row in space.
#[test]
fn a_point_in_space_on_a_spline_is_drawn_in_its_plane() {
    for view in ["std.top", "std.side"] {
        let e = read(&format!("{}m := point hint((12mm, 22mm))\nm coincident(t == 0.5) g\n",
            guide(view)));
        let plane = plane_of(&e, "k0");
        assert!(plane.is_some());
        assert_eq!(plane_of(&e, "m"), plane, "{view}");
        assert_eq!(kinds(&e), vec![CKind::PointOnSpline], "{view}");
        let sk = solved(&e);
        let m = sk.world_point(ent(&e, "m").i());
        assert!(near(m, lifted(view, (12.5, 22.5))), "{view}: {m:?}");
    }
}

/// The same of a traced curve: `m` on the involute, 12 from the circle's centre, is where the
/// involute's roll makes `√(1 + u²) = 1.2`, in the plane the circle is drawn in.  And a relation
/// over `m` and a point of that plane reads as the plane's own.
#[test]
fn a_point_in_space_on_a_traced_curve_is_drawn_in_its_plane() {
    let u = (1.2f64 * 1.2 - 1.0).sqrt().to_degrees();
    for view in ["std.top", "std.side"] {
        let e = read(&format!(
            "{}m := point\nm coincident e hint(t: 40deg)\nm distance(12mm) o\n",
            involute(view)
        ));
        let plane = plane_of(&e, "o");
        assert_eq!(plane_of(&e, "m"), plane, "{view}");
        assert_eq!(kinds(&e), vec![CKind::PointOnCurve, CKind::Distance], "{view}");
        let sk = solved(&e);
        let m = sk.world_point(ent(&e, "m").i());
        assert!(near(m, lifted(view, involute_at(0.0, 0.0, 10.0, u))), "{view}: {m:?}");
    }
}

/// A line said tangent to a spline drawn in a plane lies in it: its free ends are drawn there,
/// so the tangency is the plane's own.  `OnSpline`'s station (`docs/lofts/l0/stations`) is this.
#[test]
fn a_line_in_space_tangent_to_a_spline_is_drawn_in_its_plane() {
    let e = read(&format!(
        "{}m := point\nm coincident(t == 0.5) g\nq := point\ntl := line(m, q)\n\
         g tangent(t == 0.5) tl\nm distance(10mm) q\n",
        guide("std.top")
    ));
    let plane = plane_of(&e, "k0");
    for p in ["m", "q"] {
        assert_eq!(plane_of(&e, p), plane, "{p}");
    }
    let sk = solved(&e);
    let (m, q) = (sk.world_point(ent(&e, "m").i()), sk.world_point(ent(&e, "q").i()));
    assert!(near(m, [12.5, 22.5, 0.0]), "{m:?}");
    assert!(q[2].abs() < 1e-9);
    assert!(norm(cross(sub(q, m), [0.6, 0.4, 0.0])) < 1e-7, "{q:?}");
    // a line with both ends in space
    let e = read(&format!(
        "{}l := line(hint((5mm, 5mm)), hint((30mm, 25mm)))\ng tangent l\n", guide("std.top")
    ));
    for p in ["l.p1", "l.p2"] {
        assert_eq!(plane_of(&e, p), plane, "{p}");
    }
    assert!(!kinds(&e).iter().any(|k| k.spatial()), "{:?}", kinds(&e));
    // a word the operands do not take draws nothing: `l tangent g` is refused as written, and
    // `l` stands where it stood
    let src = format!("{}l := line(hint((5mm, 5mm)), hint((30mm, 25mm)))\nl tangent g\n",
        guide("std.top"));
    refused(&src, "E040", "does not relate a line to a spline", "tangent");
    let (prog, _, _) = gcs_core::library::parse_linked(&src);
    let e = gcs_core::program::elaborate(&prog);
    assert_eq!(plane_of(&e, "l.p1"), None);
}

/// A point a statement keeps in space is on no curve drawn in a plane: there is no contact in
/// space, and the refusal says what keeps it there.
#[test]
fn a_point_kept_in_space_is_refused_on_a_curve_with_the_cause() {
    let src = format!("{}m := point hint((12mm, 22mm, 5mm))\nm coincident g\n", guide("std.top"));
    refused(&src, "E062", "its seed has a `z`", "m coincident g");
    let src = format!("{}m := point\nfix((1mm, 2mm, 3mm)) m\nm coincident e\n",
        involute("std.top"));
    refused(&src, "E062", "a `fix` holds it there", "m coincident e");
    // said once: the relation is not read again to say it has no meaning in space
    let (prog, _, _) = gcs_core::library::parse_linked(&src);
    let e = gcs_core::program::elaborate(&prog);
    assert_eq!(e.errors().count(), 1);
}
