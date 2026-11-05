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
    // a copy the document wrote steps with its index: named by it, or without it as every copy
    let drill = FacePath::new("drill[1]", "hole");
    for yes in [&["drill[1]", "hole"][..], &["drill", "hole"], &["drill"], &["drill[1]"]] {
        assert!(drill.matches(yes), "{yes:?}");
    }
    for no in [&["drill[2]"][..], &["drill[]"], &["dril"], &["drill[1]", "hole[1]"]] {
        assert!(!drill.matches(no), "{no:?}");
    }
}

/// A plate 60 × 20 × 10 with three holes cut through it by a `repeat`, each drill a copy.
const DRILLED: &str = "unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (chamfer)\nuse std.motion (turns)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((60, 0)) b1\nfix((60, 20)) b2\nfix((0, 20)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n}\n\
    plate := solid(std.solids.Prism(outline, from: -10mm, to: 0))\n\
    repeat 3 as i {\n\
      c := point in std.front\n\
      fix((15 + 15 * i, 10)) c\n\
      hole := radius(3mm) circle(center: c) in std.front\n\
      drill := solid(std.solids.Through(face(hole)))\n\
      drill cut plate\n\
    }\n";

type Areas = std::collections::BTreeMap<String, f64>;

/// **One copy of a pattern** ([0.157]): a solid the document writes in a `repeat`'s body is a
/// step of its own in a body's faces' names, with its index (`plate.drill[1].hole`, the hole the
/// second drill cut), so a face path names one copy's faces — `plate.drill[1]`, as a reference
/// to the copy is written — where `plate.drill` names them all.  A word's solids are not: the
/// chamfer's faces are `taken.flat` however many it makes.
#[test]
fn a_copy_written_in_a_block_is_named_by_its_index() {
    let read = |chamfered: &str| {
        let e = solved(&format!("{DRILLED}{chamfered}"));
        gcs_core::report::positions(&e.sketch, &e.map).into_iter()
            .filter(|(k, _)| k.starts_with("plate.drill") || k.starts_with("plate.taken")
                || k == "plate.volume")
            .collect::<Areas>()
    };
    let plain = read("");
    let hole = |k: usize| format!("plate.drill[{k}].hole.area");
    for k in 0..3 {
        assert!(plain.contains_key(&hole(k)), "{plain:?}");
    }
    let one = read("plate.near chamfer(d: 1mm) plate.drill[1]\n");
    assert!(one.contains_key("plate.taken.flat.area"), "{one:?}");
    // its face by name, and the copy as a block's binder reads it
    let face = read("plate.near chamfer(d: 1mm) plate.drill[1].hole\n");
    let bound = read("repeat 1 as i {\nplate.near chamfer(d: 1mm) plate.drill[i + 1]\n}\n");
    let all = read("plate.near chamfer(d: 1mm) plate.drill.hole\n");
    let shorter = |m: &Areas, k: usize| m[&hole(k)] < plain[&hole(k)] - 1.0;
    for k in 0..3 {
        for (m, what) in [(&one, "one"), (&face, "face"), (&bound, "bound")] {
            assert_eq!(shorter(m, k), k == 1, "{what}, hole {k}: {m:?}");
        }
        assert!(shorter(&all, k), "hole {k}: {all:?}");
        assert!((one[&hole(k)] - plain[&hole(k)]).abs() < 1e-9 || k == 1);
    }
    // each hole's chamfer takes as much as another's
    let taken = |m: &Areas| plain["plate.volume"] - m["plate.volume"];
    assert!(taken(&one) > 0.0);
    let (a, b) = (taken(&all), taken(&one));
    assert!((a - 3.0 * b).abs() < 1e-6 * a, "{a} against three of {b}");
    // a copy there is not names nothing, as any face path that names nothing
    assert!(taken(&read("plate.near chamfer(d: 1mm) plate.drill[7]\n")).abs() < 1e-9);
}

/// A block 40 × 24 × 12 on the front plane, its near face at y = 0, a copy of it turned a half
/// turn about the z axis, and a body standing on that copy.
const PLACED: &str = "unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round)\nuse std.motion (turns)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n\
    o := point\nz := point\nfix((0, 0)) o\nfix((0, 10)) z\nax := line(o, z)\n}\n\
    block := solid(std.solids.Prism(outline, from: -12mm, to: 0))\n\
    turn := frame\nturn turns(1) ax\n\
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
    let e = solved(&format!("{MEMBERS}within := {{ p | p in h.ends }}\n\
                             near := {{ p | distance(p, boundary(h.side)) <= 1mm }}\n\
                             a := solid(within)\nb := solid(near)\n"));
    let ball = |r: f64| 4.0 / 3.0 * PI * r.powi(3);
    for (name, want) in [("a", ball(2.0)), ("b", ball(4.0))] {
        let field = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
        let got = field_volume(&field, [-5.0; 3], [5.0; 3], 1.0 / 16.0);
        assert!((got - want).abs() < 2e-3 * want, "{name}: {got} against {want}");
    }
}

/// A bar `w × 10 × 10` from `x0`, its foot on the front plane's `y0`: one solid, `body`.
const BARS: &str = "unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round)\n\
    component Bar(x0: Length, w: Length, y0: Length, h: Length) {\n\
      in std.front {\n\
        a := point hint((x0, y0))\nb := point hint((x0 + w, y0))\n\
        c := point hint((x0 + w, y0 + h))\ne := point hint((x0, y0 + h))\n\
        fix((x0, y0)) a\nfix((x0 + w, y0)) b\nfix((x0 + w, y0 + h)) c\nfix((x0, y0 + h)) e\n\
        (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, e)) -> \
        (da := line(e, a)) -> close\n\
      }\n\
      body := solid(std.solids.Prism(face(ab, bc, cd, da), from: 0mm, to: 10mm))\n\
    }\n\
    spine := Bar(x0: 0mm, w: 60mm, y0: 0mm, h: 10mm)\n\
    part := solid(spine.body)\n";

/// What a document's `part` came to: its volume and its faces' areas, by name.
fn part(src: &str) -> Areas {
    let e = solved(src);
    gcs_core::report::positions(&e.sketch, &e.map).into_iter()
        .filter(|(k, _)| k.starts_with("part.") && (k.ends_with(".area") || k == "part.volume"))
        .collect()
}

/// The fill a ball of radius `r` leaves in a square concave edge `l` long.
fn fill(r: f64, l: f64) -> f64 {
    r * r * (1.0 - PI / 4.0) * l
}

/// **An operand by its whole path** ([0.203], #760): the `body` of two instances unioned into one
/// part are two steps, `spine.body` and `hub.body`, so their faces stand apart — through 0.202
/// both were `part.body` — and a round between them is written by the instances' names.
#[test]
fn the_bodies_of_two_instances_are_named_apart() {
    let doc = format!("{BARS}hub := Bar(x0: 20mm, w: 10mm, y0: 10mm, h: 20mm)\n\
                       hub.body union part\n");
    let plain = part(&doc);
    for k in ["part.spine.body.ab.area", "part.hub.body.cd.area", "part.hub.body.near.area"] {
        assert!(plain.contains_key(k), "{k}: {plain:?}");
    }
    assert!(!plain.keys().any(|k| k.starts_with("part.body.")), "{plain:?}");
    // the hub's foot is the spine's top: none of it is left
    assert!(!plain.contains_key("part.hub.body.ab.area"), "{plain:?}");
    // the hub's two sides meet the spine's top in two concave edges, 10 long
    let want = plain["part.volume"] + 2.0 * fill(2.0, 10.0);
    for (a, b) in [("part.spine", "part.hub"), ("part.spine.body", "part.hub.body"),
                   ("part.spine.body.cd", "part.hub")] {
        let got = part(&format!("{doc}{a} round(r: 2mm) {b}\n"))["part.volume"];
        assert!((got - want).abs() < 1e-6 * want, "{a} round {b}: {got} against {want}");
    }
}

/// Two pads, each the `pad` of an instance a `repeat` makes, drawn in a plane or a frame of its
/// copy's own (`frame`): a frame's instance is closed, its two pads one binding built once and
/// placed (#566), whose faces a round reads by the same names.
fn pads(frame: bool) -> String {
    // a frame's front is its own: the pad is drawn in it, its plane's v the world's y
    let (kind, v) = if frame { ("frame", "(0, 1, 0)") } else { ("plane", "(0, 0, 1)") };
    let (open, close) = if frame { ("in std.front {\n", "}\n") } else { ("", "") };
    format!("component Pad() {{\n{open}\
          a := point hint((0, 0))\nb := point hint((6, 0))\n\
          c := point hint((6, 8))\ne := point hint((0, 8))\n\
          fix((0, 0)) a\nfix((6, 0)) b\nfix((6, 8)) c\nfix((0, 8)) e\n\
          (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, e)) -> \
          (da := line(e, a)) -> close\n\
          {close}pad := solid(std.solids.Prism(face(ab, bc, cd, da), from: 0mm, to: 10mm))\n\
        }}\n\
        at := [10mm, 40mm]\n\
        repeat x in at {{\n\
          rail := {kind}\n\
          fix(origin == (x, 0mm, 10mm)) rail\n\
          fix(dir == (1, 0, 0)) rail.u\n\
          fix(dir == {v}) rail.v\n\
          joint := Pad() in rail\n\
          joint.pad union part\n\
        }}\n")
}

/// **A copy on the way is an index** ([0.203], #760): the `pad` of an instance a `repeat`'s copy
/// holds is `joint[k].pad` through the part, so `part.joint` names every copy's faces and
/// `part.joint[1]` one's — through 0.202 each was `part.pad`, the copies alike — and a path that
/// names no face says so (W116), with the steps the part's faces are named by.  So in a frame,
/// where the copies are one binding placed: through 0.202 a round read their faces by their
/// keys (`#41.0.joint.pad.cd`), which no path names, and rounded nothing.
#[test]
fn an_instance_a_copy_holds_is_named_by_its_index() {
    for frame in [false, true] {
        a_copy_holds(&format!("{BARS}{}", pads(frame)));
    }
}

fn a_copy_holds(doc: &str) {
    let plain = part(doc);
    for k in 0..2 {
        assert!(plain.contains_key(&format!("part.joint[{k}].pad.cd.area")), "{plain:?}");
    }
    assert!(!plain.keys().any(|k| k.starts_with("part.pad.")), "{plain:?}");
    // each pad's two sides meet the spine's top in concave edges 10 long
    let one = plain["part.volume"] + 2.0 * fill(2.0, 10.0);
    let both = one + 2.0 * fill(2.0, 10.0);
    for (b, want) in [("part.joint", both), ("part.joint.pad", both), ("part.joint[1]", one),
                      ("part.joint[0].pad", one)] {
        let got = part(&format!("{doc}part.spine round(r: 2mm) {b}\n"))["part.volume"];
        assert!((got - want).abs() < 1e-6 * want, "{b}: {got} against {want}");
    }
    // the name through 0.202: nothing rounded, and said where the round is
    let wrong = format!("{doc}part.spine round(r: 2mm) part.pad\n");
    let said = w116(&wrong);
    let [(line, message)] = said.as_slice() else { panic!("{said:?}") };
    assert_eq!(*line, wrong.lines().count() as u32, "{message}");
    for step in ["`part.pad`", "`joint[0]`", "`joint[1]`", "`spine`"] {
        assert!(message.contains(step), "{step}: {message}");
    }
    assert_eq!(part(&wrong)["part.volume"], plain["part.volume"]);
    // past a beginning that names faces, the steps that go on from it
    let said = w116(&format!("{doc}part.spine round(r: 2mm) part.joint.pud\n"));
    let past = "past `part.joint` they are named by `pad`";
    assert!(said.len() == 1 && said[0].1.contains(past), "{said:?}");
    // a path that names faces says nothing
    assert_eq!(w116(&format!("{doc}part.spine round(r: 2mm) part.joint\n")), []);
}

/// Each W116 `src` says: the line it is said at, and its words.
fn w116(src: &str) -> Vec<(u32, String)> {
    solved(src).diags.iter().filter(|d| d.code.as_str() == "W116")
        .map(|d| (gcs_core::syntax::line_col(src, d.span.lo).0, d.message.clone())).collect()
}
