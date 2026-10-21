//! **Face references as resolved operands** (#188): a face's name is the route through a body's
//! operands to the primitive it is a piece of and the face's own name in it, given by the one
//! walk the exact boundary is named by and matched by one rule; the members `ends`, `surface`
//! and `body` are read once by the flattener, and a name the document declares under one of
//! those words is read as itself.
use gcs_core::model::FaceRef;
use gcs_core::program::Elaborated;
use gcs_core::solid::faces::{face_paths, BodyFaces, FacePath};
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{ent, text::solved};
use crate::region_general::field_volume;

/// A path names a face by its segments from the first, never by a suffix.
#[test]
fn a_path_names_a_face_from_its_route() {
    let f = FacePath::new("turned.boss", "near");
    assert_eq!(f.text(), "turned.boss.near");
    for yes in [&[][..], &["turned"], &["turned", "boss"], &["turned", "boss", "near"]] {
        assert!(f.matches(yes), "{yes:?}");
    }
    for no in [&["boss"][..], &["near"], &["boss", "near"], &["turned", "near"],
               &["turned", "boss", "near", "x"]] {
        assert!(!f.matches(no), "{no:?}");
    }
    // the body's own sweep has no route: its cap is named alone
    assert!(FacePath::new("", "near").matches(&["near"]));
    // a copy's edge names its face with the copy's index: by it, or without it as every copy
    let hole = FacePath::new("drill", "hole[1]");
    for yes in [&["drill", "hole[1]"][..], &["drill", "hole"], &["drill"]] {
        assert!(hole.matches(yes), "{yes:?}");
    }
    for no in [&["drill", "hole[2]"][..], &["drill", "hole[]"], &["drill", "hol"], &["drill[1]"]] {
        assert!(!hole.matches(no), "{no:?}");
    }
}

/// A plate 60 × 20 × 10 with three holes cut through it by a `repeat`, each drill a copy.
const DRILLED: &str = "unit mm\nuse std (chamfer)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((60, 0)) b1\nfix((60, 20)) b2\nfix((0, 20)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n}\n\
    plate := solid(outline, depth: 10mm)\n\
    repeat 3 as i {\n\
      c := point in std.front\n\
      fix((15 + 15 * i, 10)) c\n\
      hole := radius(3mm) circle(center: c) in std.front\n\
      drill := solid(face(hole), through: plate)\n\
      drill cut plate\n\
    }\n";

type Areas = std::collections::BTreeMap<String, f64>;

/// **One copy of a pattern, by the name its face is reported under**: the side a copy's drill
/// leaves is `plate.drill.hole[1]`, and a face path written so names that one hole — the index
/// is the face's name, not a copy of a declaration — where `plate.drill.hole` names them all.
#[test]
fn a_face_a_copy_named_is_named_by_its_index() {
    let read = |chamfered: &str| {
        let e = solved(&format!("{DRILLED}{chamfered}"));
        gcs_core::report::positions(&e.sketch, &e.map).into_iter()
            .filter(|(k, _)| k.starts_with("plate.drill.hole") || k == "plate.volume")
            .collect::<Areas>()
    };
    let plain = read("");
    let hole = |k: usize| format!("plate.drill.hole[{k}].area");
    for k in 0..3 {
        assert!(plain.contains_key(&hole(k)), "{plain:?}");
    }
    let one = read("plate.near chamfer(d: 1mm) plate.drill.hole[1]\n");
    // as an index a block's binder reads
    let bound = read("repeat 1 as i {\nplate.near chamfer(d: 1mm) plate.drill.hole[i + 1]\n}\n");
    let all = read("plate.near chamfer(d: 1mm) plate.drill.hole\n");
    let shorter = |m: &Areas, k: usize| m[&hole(k)] < plain[&hole(k)] - 1.0;
    for k in 0..3 {
        assert_eq!(shorter(&one, k), k == 1, "hole {k}: {one:?}");
        assert_eq!(shorter(&bound, k), k == 1, "hole {k}: {bound:?}");
        assert!(shorter(&all, k), "hole {k}: {all:?}");
        assert!((one[&hole(k)] - plain[&hole(k)]).abs() < 1e-9 || k == 1);
    }
    // each hole's chamfer takes as much as another's
    let taken = |m: &Areas| plain["plate.volume"] - m["plate.volume"];
    assert!(taken(&one) > 0.0);
    let (a, b) = (taken(&all), taken(&one));
    assert!((a - 3.0 * b).abs() < 1e-6 * a, "{a} against three of {b}");
    // an index naming no copy's face names nothing, as any face path that names nothing
    assert!(taken(&read("plate.near chamfer(d: 1mm) plate.drill.hole[7]\n")).abs() < 1e-9);
}

/// A block 40 × 24 × 12 on the front plane, its near face at y = 0, a copy of it turned a half
/// turn about the z axis, and a body standing on that copy.
const PLACED: &str = "unit mm\nuse std (round)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n\
    o := point\nz := point\nfix((0, 0)) o\nfix((0, 10)) z\nax := line(o, z)\n}\n\
    block := solid(outline, depth: 12mm)\n\
    turn := frame\nstd.Turn(turn, about: ax, ratio: 1)\n\
    moved := block at(under: turn) 180deg\n\
    pad := solid(moved)\n";

/// A placed operand's faces are listed under its route, by the names its body's exact boundary
/// gives them, so each reads the faces it names.
#[test]
fn a_placed_operands_faces_are_named_as_its_body_names_them() {
    let e = solved(PLACED);
    let pad = ent(&e, "pad").i();
    let paths = face_paths(&e.sketch, pad);
    let texts: Vec<String> = paths.iter().map(FacePath::text).collect();
    for want in ["moved.near", "moved.far", "moved.south", "moved.east", "moved.north", "moved.west"] {
        assert!(texts.iter().any(|t| t == want), "{want} among {texts:?}");
    }
    let body = BodyFaces::read(&e.sketch, pad).unwrap();
    let names = |path: &[&str]| body.faces(&FaceRef {
        solid: pad as u32,
        path: path.iter().map(|s| s.to_string()).collect(),
        surface: false,
        smooth: false,
    });
    for p in &paths {
        let segs: Vec<&str> = p.segments().collect();
        assert_eq!(names(&segs).len(), 1, "{} names one face", p.text());
    }
    assert_eq!(names(&["moved"]).len(), 6);
    assert_eq!(names(&[]).len(), 6);
    // a suffix names nothing: `near` is no face of `pad`'s own
    assert!(names(&["near"]).is_empty());
}

/// Rounding the rim of a placed operand's near face: the corner of the rim is taken away, the
/// rest of the block kept — the copy turned a half turn, so it stands at x < 0, behind its near
/// face at y = 0.
#[test]
fn a_placed_operands_faces_are_rounded() {
    let before = solved(PLACED);
    let after = solved(&format!("{PLACED}pad.moved.near round(r: 2mm) pad.moved\n"));
    let made = after.sketch.solids.len() - before.sketch.solids.len();
    assert!(made > 0, "the round makes its solids");
    let side = |e: &Elaborated, p: [f64; 3]| {
        MaterialField::read(&e.sketch, ent(e, "pad").i(), 1e-10).unwrap().side(p)
    };
    // beside the rim's edge with the south face, within the ball's reach of both
    let corner = [-20.0, -0.1, 0.1];
    assert!(side(&before, corner) < 0.0, "the corner is the block's before");
    assert!(side(&after, corner) > 0.0, "and rounded away after");
    // the far cap's rim and the block's middle stay
    for kept in [[-20.0, -11.9, 0.1], [-20.0, -6.0, 12.0]] {
        assert!(side(&after, kept) < 0.0, "{kept:?} stays");
    }
}

/// A component whose members are sets called `ends` and `surface`: each read as the set it is,
/// not as where an edge ends or a set's surface.
const MEMBERS: &str = "unit mm\n\
    component Pair(c: point) {\n\
      ends := { p | p distance(<= 2mm) c }\n\
      surface := { p | p distance(<= 3mm) c }\n\
    }\n\
    c := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
    h := Pair(c)\n";

#[test]
fn a_member_called_ends_or_surface_is_itself() {
    let e = solved(&format!("{MEMBERS}within := {{ p | p inside h.ends }}\n\
                             near := {{ p | distance(p, h.surface) <= 1mm }}\n\
                             a := solid(within)\nb := solid(near)\n"));
    let ball = |r: f64| 4.0 / 3.0 * PI * r.powi(3);
    for (name, want) in [("a", ball(2.0)), ("b", ball(4.0))] {
        let field = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
        let got = field_volume(&field, [-5.0; 3], [5.0; 3], 1.0 / 16.0);
        assert!((got - want).abs() < 2e-3 * want, "{name}: {got} against {want}");
    }
}
