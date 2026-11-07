//! **A face swept up a screw over an interval stated in place** (#846): `some t in (a, b): p in
//! sec at(under: m) t`, `m` a screw, a slide or a turn at a constant rate, is the face's profile
//! carried by the motion from where the interval begins to where it ends (`solid::carried`) —
//! built exactly by the kernel, held to its field, and written inside a component as anywhere,
//! since it names no running parameter.  Each is held here to its closed form, to the face family
//! the same sweep is over a declared parameter, and to its own field point by point.
use gcs_core::brep::props;
use gcs_core::program::Elaborated;
use gcs_core::solid::region::{general, route, Route};
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// The issue's tooth space: a 2 mm square 10 mm off a vertical spindle, drawn in the top view and
/// swept a quarter turn up a screw of 50 mm lead — written in a component, which no face family
/// can stand in.
const SPACE: &str = "\
unit mm
use std
use std.shapes
use std.place (vertical, coords)
use std.motion (screws)

component Space() {
  in std.front {
    construction spindle := vertical line(std.origin, hint((0, 10)))
    length(10mm) spindle
  }
  in std.top {
    a := point hint((10, 0))
    a coords(du: 10mm, dv: 0mm) std.top
    sq := std.shapes.CenteredRectangle(a, w: 2mm, h: 2mm)
  }
  section := face(sq.loop)
  screw := frame
  screw screws(LEAD) spindle
  space := solid({ p | some t in (FROM, TO): p in section at(under: screw) t })
}
x := Space()
";

/// `SPACE` with its lead and interval.
fn space(lead: &str, from: &str, to: &str) -> Elaborated {
    solved(&SPACE.replace("LEAD", lead).replace("FROM", from).replace("TO", to))
}

/// Solid `name` built exactly as a carried face (`Route::Carried`), never the field's surface.
fn exact(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let i = ent(e, name).i();
    assert!(matches!(route(&e.sketch, i), Ok(Route::Carried(Ok(_)))), "{name}: {:?}",
            route(&e.sketch, i).ok().and_then(|r| r.refusal().map(str::to_string)));
    e.sketch.exact_solid(i).unwrap_or_else(|m| panic!("`{name}`: {m}"))
}

/// Its faces' own names as the kernel built them, sorted.
fn face_names(x: &gcs_core::solid::Exact) -> Vec<String> {
    let mut v: Vec<String> = x.named.iter().map(|(_, f)| f.clone()).collect();
    v.sort();
    v
}

fn close(got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol * want.abs(), "{got} against {want}");
}

/// The issue's form 1: the sweep in a component builds, exactly, as the face family over a
/// declared parameter does — the square's area times its rise, a quarter of the lead.
#[test]
fn a_screw_in_a_component_is_built_exactly() {
    let e = space("50mm", "0deg", "90deg");
    let x = exact(&e, "x.space");
    close(props::volume(&x.brep), 4.0 * 12.5, 1e-9);
    assert_eq!(x.brep.faces.len(), 6);
    // named as the paths name them: its section's sides, and its ends
    let names = face_names(&x);
    assert!(names.contains(&"start".to_string()) && names.contains(&"end".to_string()), "{names:?}");
    let mut paths: Vec<String> = gcs_core::solid::faces::face_paths(&e.sketch, ent(&e, "x.space").i())
        .iter().map(|p| p.face.clone()).collect();
    paths.sort();
    paths.dedup();
    let mut built = names.clone();
    built.dedup();
    assert_eq!(paths, built);
    // its field: in where the closed form says, out elsewhere
    let field = general(&e.sketch, ent(&e, "x.space").i()).unwrap();
    for (t, inside) in [(0.3, true), (0.9, true), (-0.1, false), (1.1, false)] {
        let (s, c) = (t * PI / 2.0).sin_cos();
        for (r, within) in [(10.0, true), (10.8, true), (11.3, false), (8.6, false)] {
            let p = [r * c, r * s, 12.5 * t];
            let want = inside && within;
            assert_eq!(field.value(p) < 0.0, want, "t {t}, r {r}: {}", field.value(p));
        }
    }
}

/// The same quarter turn swept as a face family over a declared parameter (the issue's form 3):
/// one solid, whichever is written.
#[test]
fn the_screw_in_place_is_the_face_family() {
    let family = solved("\
unit mm
use std
use std.shapes
use std.place (vertical, coords)
use std.motion (screws)

component Section(s: Scalar) {
  in std.front {
    construction spindle := vertical line(std.origin, hint((0, 10)))
    length(10mm) spindle
  }
  screw := frame
  screw screws(50mm) spindle
  fix(value == 90deg * s) screw
  in screw {
    in std.top {
      a := point hint((10, 0))
      a coords(du: 10mm, dv: 0mm) std.top
      sq := std.shapes.CenteredRectangle(a, w: 2mm, h: 2mm)
    }
    section := face(sq.loop)
  }
}
param s: Scalar over (0, 1)
space := solid({ p | some s: p in Section(s: s).section at s })
");
    let f = family.sketch.exact_solid(ent(&family, "space").i()).unwrap();
    let e = space("50mm", "0deg", "90deg");
    let x = exact(&e, "x.space");
    close(props::volume(&x.brep), props::volume(&f.brep), 1e-9);
    assert_eq!(x.brep.faces.len(), f.brep.faces.len());
}

/// An interval that does not begin at nothing starts where the screw has carried the face to; a
/// left-handed screw winds the other way; and cut from a barrel, the groove takes what of it the
/// barrel holds.
#[test]
fn a_screw_from_anywhere_either_hand() {
    let e = space("50mm", "-60deg", "90deg");
    close(props::volume(&exact(&e, "x.space").brep), 4.0 * 50.0 * 150.0 / 360.0, 1e-9);
    let e = space("-50mm", "-90deg", "0deg");
    close(props::volume(&exact(&e, "x.space").brep), 4.0 * 12.5, 1e-9);
    // a left hand turned back a quarter turn stands a quarter of its lead above the top plane,
    // turned to -y
    let field = general(&e.sketch, ent(&e, "x.space").i()).unwrap();
    assert!(field.value([0.0, -10.0, 12.5 - 1e-3]) < 0.0);
    assert!(field.value([0.0, 10.0, 12.5 - 1e-3]) > 0.0);
    assert!(field.value([0.0, -10.0, -12.5 + 1e-3]) > 0.0);

    let src = SPACE.replace("LEAD", "50mm").replace("FROM", "-60deg").replace("TO", "90deg")
        .replace("use std.motion (screws)", "use std.motion (screws)\nuse std.solids (cut)")
        .replace("  section := face(sq.loop)", "\
  section := face(sq.loop)
  in std.top {
    o := point
    rim := circle(center: o) hint(r: 14)
  }
  fix((0, 0)) o
  radius(14mm) rim
  body := solid(std.solids.Prism(face(rim), from: -5mm, to: 10mm))
  space cut body");
    let e = solved(&src);
    let b = e.sketch.exact_solid(ent(&e, "x.body").i()).unwrap().brep.clone();
    // the groove runs from 50·(-60/360) below the top plane to 12.5 above it: 15 of it in the
    // barrel
    close(props::volume(&b), PI * 196.0 * 15.0 - 4.0 * 15.0, 1e-9);
}

/// A thread drawn as `lofts/thread.sv` draws it: its square in a plane holding the axis, carried
/// three turns up a screw of pitch 4 — Pappus's area times its centroid's circle a turn, as the
/// face family reads it.
#[test]
fn a_thread_drawn_through_its_axis() {
    let src = "\
unit mm
use std
use std.place (on_u)
use std.shapes
use std.motion (screws)

component Thread() {
  screw := frame
  screw screws(4mm) std.top.normal
  in std.front {
    c := point
    c on_u(d: 10mm) std.front
  }
  sq := std.shapes.CenteredRectangle(c, w: SIZE, h: SIZE) in std.front
  sec := face(sq.loop)
  thread := solid({ p | some t in (0deg, 1080deg): p in sec at(under: screw) t })
}
x := Thread()
";
    let e = solved(&src.replace("SIZE", "2mm"));
    close(props::volume(&exact(&e, "x.thread").brep), 4.0 * 2.0 * PI * 10.0 * 3.0, 1e-7);
    // its field, a turn and a half up: in on the square there, out between two flights
    let field = general(&e.sketch, ent(&e, "x.thread").i()).unwrap();
    assert!(field.value([-10.0, 0.0, 6.0]) < 0.0);
    assert!(field.value([-10.0, 0.0, 4.0]) > 0.0);
    assert!(field.value([-10.0, 0.0, 6.0 + 4.0]) < 0.0);
    assert!(field.value([10.0, 0.0, 13.0]) > 0.0);
    // a square as tall as the pitch, carried more than a turn, would meet itself: refused
    let e = solved(&src.replace("SIZE", "4mm"));
    let i = ent(&e, "x.thread").i();
    let why = e.sketch.exact_solid(i).err().expect("refused");
    assert!(why.contains("meets itself"), "{why}");
}

/// A slide carries a face along a line crossing its plane: the face's area times how far the
/// slide carries it across the plane.  A turn short of a whole one, from anywhere: Pappus's.
#[test]
fn a_slide_and_a_turn_in_place() {
    let e = solved("\
unit mm
use std
use std.shapes
use std.place (coords)
use std.motion (slides, turns)

component Moved() {
  in std.front {
    o := point
    fix((0, 0)) o
    k := point hint((3, 4))
    fix((3, 4)) k
    construction way := line(o, k)
    construction up := vertical line(o, hint((0, 10)))
    length(10mm) up
    b := point hint((10, 0))
    b coords(du: 10mm, dv: 0mm) std.front
    bq := std.shapes.CenteredRectangle(b, w: 2mm, h: 2mm)
  }
  in std.top {
    a := point hint((10, 0))
    a coords(du: 10mm, dv: 0mm) std.top
    sq := std.shapes.CenteredRectangle(a, w: 2mm, h: 2mm)
  }
  slide := frame
  slide slides(10mm) way
  turn := frame
  turn turns(1) up
  flat := face(sq.loop)
  upright := face(bq.loop)
  slid := solid({ p | some t in (0deg, 180deg): p in flat at(under: slide) t })
  turned := solid({ p | some t in (30deg, 90deg): p in upright at(under: turn) t })
}
x := Moved()
");
    // carried 5 along (3, 4)/5: 4 across the top plane
    close(props::volume(&exact(&e, "x.slid").brep), 4.0 * 4.0, 1e-9);
    close(props::volume(&exact(&e, "x.turned").brep), 4.0 * 10.0 * PI / 3.0, 1e-9);
}

/// What the sweep would meet itself in, or has no volume, is refused with its reason: a section
/// leaning on a screw's axis, a slide along the face's own plane.
#[test]
fn a_sweep_that_is_no_body_is_refused() {
    let refused = |src: &str, name: &str, says: &str| {
        let e = solved(src);
        let why = e.sketch.exact_solid(ent(&e, name).i()).err().expect("refused");
        assert!(why.contains(says), "{why}");
    };
    refused(&SPACE.replace("LEAD", "50mm").replace("FROM", "0deg").replace("TO", "90deg")
        .replace("vertical line(std.origin, hint((0, 10)))", "line(std.origin, hint((10, 10)))")
        .replace("length(10mm) spindle", "fix((10, 10)) spindle.p2"),
        "x.space", "square to the screw's axis or holds it");
    refused("\
unit mm
use std
use std.shapes
use std.place (horizontal, coords)
use std.motion (slides)
in std.front {
  construction way := horizontal line(std.origin, hint((10, 0)))
  length(10mm) way
}
in std.top {
  a := point hint((10, 0))
  a coords(du: 10mm, dv: 0mm) std.top
  sq := std.shapes.CenteredRectangle(a, w: 2mm, h: 2mm)
}
slide := frame
slide slides(10mm) way
flat := face(sq.loop)
slid := solid({ p | some t in (0deg, 180deg): p in flat at(under: slide) t })
", "slid", "sweeps no volume");
}
