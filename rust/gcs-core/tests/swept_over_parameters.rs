//! **A face swept as a declared parameter runs** (#398 B): a face family's running number is a
//! declared parameter, `param s: Scalar over (0, 1)` and `sweep(Station(…).f over s)`; a face of
//! the drawing itself is swept as the parameter runs, `sweep(sec over s)`; and what a frame with a
//! value draws in its block stands where the value puts it — so the epic's helix (#391) is written
//! as it shows it, a section drawn in the screw's block and swept over `s`.
//!
//! Gated by the helix against its closed form to `EXACT` (every corner of the section where the
//! screw carries it, at stations of the march along `s`), against Pappus off its exact B-rep, and
//! against the thread `std.stations.Helix`'s station sweeps; and by the refusals.
use gcs_core::program::Elaborated;
use gcs_core::space::{norm, sub};
use std::f64::consts::PI;

use crate::common::parsed::determined;

/// How near the solve puts a corner to the screw's closed form: the solve's own tolerance.
const EXACT: f64 = 1e-10;

/// **The epic's helix** (#391), as written there — with the joint's value in the `fix` stage 3
/// settled on (`fix(value == …) screw`, its ratio stated), since `screw.angle == …` is no
/// relation: a frame turning about `std.top`'s normal and rising `pitch` a turn, its value
/// `360deg · turns · s`, and a 2 mm square drawn in its block on the front plane, centred 10 from
/// the axis.
const EPIC: &str = "unit mm
use std
use std.solids
use std.place (on_u)
use std.shapes
use std.stations
use std.motion (screws)

param s: Scalar over (0, 1)
r := 10mm
size := 2mm
pitch := 4mm
turns := 3
screw := frame
screw screws(pitch) std.top.normal
fix(value == 360deg * turns * s) screw
in screw {
  in std.front {
    c := point
    c on_u(d: r) std.front
  }
  sq := std.shapes.CenteredRectangle(c, w: size, h: size) in std.front
  sec := face(sq.loop)
}
thread := solid({ p | some s: p in sec at s })
";

/// The same thread as `lofts/thread.sv` sweeps it: the square drawn in `std.stations.Helix`'s axial
/// plane, a face family over the instance written in place.
const STATION: &str = "unit mm
use std
use std.solids
use std.place (on_u)
use std.shapes
use std.stations
use std.motion (screws)

component Thread(base: plane, r: Length, size: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.stations.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.axial {
    c := point
    c on_u(d: r) st.axial
  }
  sq := std.shapes.CenteredRectangle(c, w: size, h: size) in st.axial
  f := face(sq.loop)
}
param s: Scalar over (0, 1)
thread := solid({ p | some s: p in Thread(std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3).f at s })
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// The exact B-rep of the solid `name`, its boundary checked.
fn brep(e: &Elaborated, name: &str) -> gcs_core::brep::topo::Brep {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    x.brep.clone()
}

/// The words `src` is refused with, as the library links it.
fn said(src: &str) -> Vec<String> {
    crate::common::lenient::read(src).1
}

/// `p` where the screw carries it at `s`: turned `360° · 3 · s` about `std.z` and risen
/// `4 · 3 · s`.
fn screwed(p: [f64; 3], s: f64) -> [f64; 3] {
    let (sn, cs) = (3.0 * s * 2.0 * PI).sin_cos();
    [p[0] * cs - p[1] * sn, p[0] * sn + p[1] * cs, p[2] + 12.0 * s]
}

/// **The epic's helix stands where its value puts it**: drawn where `s` stands, its start, the
/// square on the front plane 10 from the axis; followed along `s`, each corner where the screw
/// carries it there, to `EXACT`; swept, Pappus's 4 · 2π·10 · 3 = 240π mm³, every wall the surface
/// its first rim sweeps, and corner for corner the thread `std.stations.Helix`'s station sweeps.
#[test]
fn the_epics_helix_stands_where_its_value_puts_it() {
    let e = determined(EPIC);
    let corners = ["sq.a", "sq.b", "sq.c", "sq.d"];
    let at = |e: &Elaborated, n: &str| e.sketch.world_point(index(e, n));
    let rest: Vec<[f64; 3]> = corners.iter().map(|n| at(&e, n)).collect();
    // at rest: the front plane, centred 10 out along `x`
    for (n, p) in corners.iter().zip(&rest) {
        assert!(p[1].abs() < EXACT && (p[0] - 10.0).abs() <= 1.0 + EXACT
                && p[2].abs() <= 1.0 + EXACT, "{n} at {p:?}");
    }
    let f = gcs_core::follow::follow(&e.sketch, "s").expect("the drawing is followed along `s`");
    assert!(f.singular.is_none() && (f.reached() - 1.0).abs() < 1e-12);
    for s in [0.1, 0.37, 0.5, 0.83, 1.0] {
        let mut sk = e.sketch.clone();
        f.put(&mut sk, s).unwrap();
        for (n, p) in corners.iter().zip(&rest) {
            let (got, want) = (sk.world_point(index(&e, n)), screwed(*p, s));
            assert!(norm(sub(got, want)) < EXACT, "{n} at s = {s}: {got:?} against {want:?}");
        }
    }
    let b = brep(&e, "thread");
    let v = gcs_core::brep::props::volume(&b);
    assert!((v - 240.0 * PI).abs() <= 1e-9 * 240.0 * PI, "{v} against 240π");
    let walls: Vec<&str> = b.faces.iter()
        .filter(|f| !f.name.ends_with(".start") && !f.name.ends_with(".end"))
        .map(|f| f.surface.kind()).collect();
    assert!(!walls.is_empty() && walls.iter().all(|&k| k == "blend"), "{walls:?}");
    let station = brep(&determined(STATION), "thread");
    assert_eq!(b.vertices.len(), station.vertices.len());
    for v in &b.vertices {
        let found = station.vertices.iter().any(|w| norm(sub(v.p, w.p)) < 1e-9);
        assert!(found, "{:?} is no corner of the station's thread", v.p);
    }
}

/// **The relations a valued frame's block states hold the drawing at rest**, carried whole: the
/// square stays 2 mm a side and 10 from the axis at every station, the plane it is drawn in turned
/// with it — and what the block declares rides on nothing more, so the screw's value reads no
/// riding and is no refusal.
#[test]
fn a_valued_frames_block_is_its_drawing_at_rest_carried() {
    let e = determined(EPIC);
    let f = gcs_core::follow::follow(&e.sketch, "s").unwrap();
    for s in [0.25, 0.6] {
        let mut sk = e.sketch.clone();
        f.put(&mut sk, s).unwrap();
        let p = |n: &str| sk.world_point(index(&e, n));
        assert!((norm(sub(p("sq.a"), p("sq.b"))) - 2.0).abs() < EXACT);
        let c = p("c");
        assert!((c[0].hypot(c[1]) - 10.0).abs() < EXACT, "{c:?}");
        assert!((c[2] - 12.0 * s).abs() < EXACT, "{c:?}");
    }
    let c = e.sketch.points[index(&e, "c")].clone();
    assert!(c.plane.is_some());
    assert!(e.sketch.carriers(gcs_core::model::EntRef::point(index(&e, "c")))
        .is_ok_and(|path| path.is_empty()));
}

/// **What a valued frame's block cannot mean is said**: a declaration drawn in the frame and in no
/// plane of it (a value carries a plane).  A frame declared in the block is no refusal (#823): it
/// rides on the valued frame as the rest of the block does, drawn where the value puts it, and so
/// rides on nothing (`tests/epicyclic.rs`).
#[test]
fn a_valued_frames_block_refuses_what_it_cannot_mean() {
    let bare = EPIC.replace("in screw {\n  in std.front {\n    c := point",
                            "in screw {\n  q := point\n  in std.front {\n    c := point");
    let s = said(&bare);
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("in no plane of it")), "{s:?}");
    let framed = EPIC.replace("in screw {\n", "in screw {\n  g := frame\n");
    let s = said(&framed);
    assert!(s.is_empty(), "{s:?}");
    let e = determined(&framed);
    assert!(e.sketch.frames[index(&e, "g")].rides.is_empty());
}

/// **A face family reads its parameter as declared**: a formal of another kind than the parameter
/// is refused; a solid swept over a parameter that is no angle is refused, since the roll is an
/// angle — and the face of the drawing is what is swept over one.
#[test]
fn a_face_family_reads_its_parameter_as_declared() {
    let length = STATION.replace("param s: Scalar over (0, 1)", "param s: Length over (0mm, 1mm)");
    let s = said(&length);
    assert!(s.iter().any(|m| m.starts_with("E103") && m.contains("a parameter of its kind")),
            "{s:?}");
    let solid = EPIC.replace("thread := solid({ p | some s: p in sec at s })",
                             "bar := solid(std.solids.Prism(sec, from: 0mm, to: 1mm))\nthread := solid({ p | some s: p in bar at s })");
    let s = said(&solid);
    assert!(s.iter().any(|m| m.starts_with("E103") && m.contains("is no angle")), "{s:?}");
}

/// **A seated part elsewhere in the drawing leaves the station held** (#817): a triangle in the
/// top carried 36° up a screw about a line through `p_c` and 100 a turn, its stations the whole
/// drawing — and a disc seated on a point raised from `p_c`, which the station's part reaches
/// through `p_c`.  The disc's frame comes into the part square, as it was drawn, so the station
/// has no shear of it to turn, and the family sweeps the triangle's 2 mm² through the screw's
/// 10 mm.
#[test]
fn a_part_seated_off_the_screws_line_leaves_the_station_held() {
    const SEATED: &str = "unit mm
use std
use std.place (coords, raised, seats)
use std.motion (screws)
use std.solids

in std.top {
  p_c := point hint((5, 5))
  p_c coords(du: 5mm, dv: 5mm) std.top
}
p_up := point
p_up raised(d: 10mm, along: std.z) p_c
construction ax := line(p_c, p_up)
param s: Scalar over (0, 1)
screw := frame
screw screws(100mm) ax
fix(value == 36deg * s) screw
in screw {
  in std.top {
    a := point hint((10, 0))
    a coords(du: 10mm, dv: 0mm) std.top
    b := point hint((12, 0))
    b coords(du: 12mm, dv: 0mm) std.top
    c := point hint((12, 2))
    c coords(du: 12mm, dv: 2mm) std.top
  }
  sec := face(a, b, c, -> close)
}
body := solid({ p | some s: p in sec at s })

disc_at := point
disc_at raised(d: 50mm, along: std.z) p_c
g := Disc()
g seats(turn: 0deg, up: std.z, east: std.x) disc_at

component Disc() {
  in std.top {
    rim := circle(center: std.top.origin) hint(r: 3)
    radius(3mm) rim
  }
  body := solid(std.solids.Prism(face(rim), from: 0mm, to: 2mm))
}
";
    let e = determined(SEATED);
    let v = gcs_core::brep::props::volume(&brep(&e, "body"));
    assert!((v - 20.0).abs() <= 1e-6 * 20.0, "{v} against 20");
}
