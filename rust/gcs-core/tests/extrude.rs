//! **Extrude** (#162 F2): a click inside a region writes the face, the solid and — with a body
//! selected — the body rule, as one splice; the solid is then sized by re-sweeping it in place
//! (`set_sweep`), previewed before the source is written (`set_prism`), by an arrow the eye sees
//! along its face's normal (`extrude_handle`, `extent_at`).

use gcs_core::edit::{self, Kind, SolidSweep};
use gcs_core::overview::workspace::{extent_at, extrude_handle, Projection};
use gcs_core::program::{self, Elaborated};
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::syntax::BodyWord;
use gcs_core::{library, solve};
use std::f64::consts::PI;

/// On the front: a 60 × 40 rectangle `ab…da` from the origin, swept 5 behind as `block`; a
/// circle `hole` of radius 5 inside it at (30, 20); and a 10 × 10 square `pq…sp` at (80, 0),
/// clear of it.
const DRAWN: &str = "unit mm\nuse std\nin std.front {\n\
    a := point hint((0, 0))\nb := point hint((60, 0))\nc := point hint((60, 40))\n\
    d := point hint((0, 40))\nab := line(a, b)\nbc := line(b, c)\ncd := line(c, d)\n\
    da := line(d, a)\no := point hint((30, 20))\nhole := circle(center: o) hint(r: 5)\n\
    p := point hint((80, 0))\nq := point hint((90, 0))\nr := point hint((90, 10))\n\
    s := point hint((80, 10))\npq := line(p, q)\nqr := line(q, r)\nrs := line(r, s)\n\
    sp := line(s, p)\n\
    fix((0, 0)) a\nfix((60, 0)) b\nfix((60, 40)) c\nfix((0, 40)) d\nfix((30, 20)) o\nfix(r == 5) hole\n\
    fix((80, 0)) p\nfix((90, 0)) q\nfix((90, 10)) r\nfix((80, 10)) s\n}\n\
    block := solid(face(ab, bc, cd, da), depth: 5)\n";

fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}\n{src}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    e
}

fn names(n: &[&str]) -> Vec<String> {
    n.iter().map(|s| s.to_string()).collect()
}

fn volume(e: &Elaborated, name: &str) -> f64 {
    e.sketch.evaluated_solid(e.map.ent_named(name).unwrap().i(), Report).unwrap().volume()
}

/// The depth a fresh extrusion is written at, read back off its statement.
fn depth_written(text: &str, solid: &str) -> f64 {
    let line = text.lines().find(|l| l.starts_with(&format!("{solid} := solid("))).unwrap();
    line.split("depth: ").nth(1).unwrap().trim_end_matches(')').parse().unwrap()
}

fn near(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

#[test]
fn a_click_writes_the_face_and_the_solid_as_one_splice() {
    let e = build(DRAWN);
    let out = edit::extrude(&e, &e.sketch, &names(&["pq", "qr", "rs", "sp"]), &[], None, false);
    assert_eq!((out.refused.as_deref(), out.kind), (None, Kind::Structural));
    assert_eq!(out.names, vec!["b0", "f0"]);
    let d = depth_written(&out.text, "b0");
    assert!(d > 0.0, "{}", out.text);
    assert!(out.text.ends_with(&format!("f0 := face(pq, qr, rs, sp)\nb0 := solid(f0, depth: {})\n",
        gcs_core::syntax::num(d))), "{}", out.text);
    assert!(near(volume(&build(&out.text), "b0"), 100.0 * d, 1e-9));
}

#[test]
fn a_region_with_a_hole_is_extruded_holed() {
    let e = build(DRAWN);
    let out = edit::extrude(&e, &e.sketch, &names(&["ab", "bc", "cd", "da"]), &[names(&["hole"])],
        None, false);
    assert!(out.text.contains("f0 := face(ab, bc, cd, da, holes: hole)\n"), "{}", out.text);
    let d = depth_written(&out.text, "b0");
    assert!(near(volume(&build(&out.text), "b0"), d * (2400.0 - 25.0 * PI), 2e-3));
}

#[test]
fn with_a_body_selected_it_joins_cuts_or_cuts_through() {
    let e = build(DRAWN);
    let block = 5.0 * 2400.0;
    // joined: the square beside it comes to the body
    let out = edit::extrude(&e, &e.sketch, &names(&["pq", "qr", "rs", "sp"]), &[],
        Some((BodyWord::Union, "block")), false);
    assert!(out.text.ends_with("b0 union block\n"), "{}", out.text);
    let d = depth_written(&out.text, "b0");
    assert!(near(volume(&build(&out.text), "block"), block + 100.0 * d, 1e-9));
    // cut: the circle's prism takes what it overlaps
    let out = edit::extrude(&e, &e.sketch, &names(&["hole"]), &[], Some((BodyWord::Cut, "block")), false);
    assert!(out.text.ends_with("b0 cut block\n"), "{}", out.text);
    let d = depth_written(&out.text, "b0");
    assert!(near(volume(&build(&out.text), "block"), block - d.min(5.0) * 25.0 * PI, 2e-3));
    // through all: no depth, through the body, and cut
    let out = edit::extrude(&e, &e.sketch, &names(&["hole"]), &[], Some((BodyWord::Cut, "block")), true);
    assert!(out.text.ends_with("b0 := solid(f0, through: block)\nb0 cut block\n"), "{}", out.text);
    assert!(near(volume(&build(&out.text), "block"), 5.0 * (2400.0 - 25.0 * PI), 2e-3));
    // through all with nothing to go through is refused
    let out = edit::extrude(&e, &e.sketch, &names(&["hole"]), &[], None, true);
    assert!(out.refused.is_some_and(|m| m.contains("select it first")));
}

#[test]
fn a_region_the_source_cannot_name_or_a_missing_body_is_refused() {
    let e = build(DRAWN);
    let out = edit::extrude(&e, &e.sketch, &names(&["ab", "nothing"]), &[], None, false);
    assert!(out.refused.is_some(), "{}", out.text);
    let out = edit::extrude(&e, &e.sketch, &names(&["pq", "qr", "rs", "sp"]), &[],
        Some((BodyWord::Union, "nowhere")), false);
    assert!(out.refused.is_some(), "{}", out.text);
    assert_eq!(out.text, e.program.text());
}

#[test]
fn a_solid_is_resized_in_place_and_the_file_is_otherwise_untouched() {
    let e = build(DRAWN);
    let block = e.map.ent_named("block").unwrap().i();
    let rest = |t: &str| t.lines().filter(|l| !l.starts_with("block :=")).collect::<Vec<_>>().join("\n");
    let cases = [
        (SolidSweep { from: Some("0".into()), to: Some("7".into()), ..Default::default() },
            "block := solid(face(ab, bc, cd, da), from: 0, to: 7)", 7.0),
        (SolidSweep { from: Some("-3".into()), to: Some("3".into()), ..Default::default() },
            "block := solid(face(ab, bc, cd, da), from: -3, to: 3)", 6.0),
        (SolidSweep { depth: Some("4".into()), ..Default::default() },
            "block := solid(face(ab, bc, cd, da), depth: 4)", 4.0),
    ];
    for (how, line, thick) in cases {
        let out = edit::set_sweep(&e, &e.program, block, &how);
        assert_eq!(out.refused, None, "{line}");
        assert!(out.text.contains(&format!("{line}\n")), "{}", out.text);
        assert_eq!(rest(&out.text), rest(e.program.text()));
        assert!(near(volume(&build(&out.text), "block"), 2400.0 * thick, 1e-9), "{line}");
    }
    // a sweep the parser refuses is refused in its words
    let bad = SolidSweep { depth: Some("4".into()), about: Some("ab".into()), ..Default::default() };
    assert!(edit::set_sweep(&e, &e.program, block, &bad).refused.is_some_and(|m| m.contains("not both")));
}

#[test]
fn a_prism_previews_a_new_extent_before_the_source_says_it() {
    let mut e = build(DRAWN);
    let block = e.map.ent_named("block").unwrap().i();
    assert!(e.sketch.set_prism(block, 0.0, 9.0));
    assert!(near(volume(&e, "block"), 2400.0 * 9.0, 1e-9));
    // a body is no prism
    let joined = build(&format!("{DRAWN}stock := solid(face(pq, qr, rs, sp), depth: 2)\nstock union block\n"));
    let body = joined.map.ent_named("block").unwrap().i();
    let mut sk = joined.sketch.clone();
    assert!(matches!(sk.solids[body].def, gcs_core::model::SolidDef::Body { .. }));
    assert!(!sk.set_prism(body, 0.0, 1.0));
}

/// The eye's picture-plane coordinates of a point in space (`overview::eye`'s convention).
fn seen(x: [f64; 3], az: f64, el: f64) -> (f64, f64) {
    let right = [-az.sin(), az.cos(), 0.0];
    let up = [-az.cos() * el.sin(), -az.sin() * el.sin(), el.cos()];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    (dot(right, x), dot(up, x))
}

#[test]
fn the_arrow_runs_from_the_face_to_its_far_end_and_reads_a_drag_back() {
    let e = build(DRAWN);
    let block = e.map.ent_named("block").unwrap().i();
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(&e.sketch, az, el);
    let h = extrude_handle(&e.sketch, &proj, block).expect("seen from three quarters");
    // the front's normal is −y, toward its viewer; `depth: 5` is behind, at +y
    let close = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1) < 1e-9;
    assert!(close(h.base, seen([30.0, 0.0, 20.0], az, el)), "{h:?}");
    assert!(close(h.tip, seen([30.0, 5.0, 20.0], az, el)), "{h:?}");
    assert_eq!((h.from, h.to), (-5.0, 0.0));
    for t in [7.0, -3.0] {
        let at = seen([30.0, -t, 20.0], az, el);
        let got = extent_at(&e.sketch, &proj, block, at).unwrap();
        assert!((got - t).abs() < 1e-9, "{got} {t}");
    }
    assert!(h.readable);
    // from the front the normal is the line of sight: no arrow to drag, though the extents stand
    let front = Projection::new(&e.sketch, -std::f64::consts::FRAC_PI_2, 0.0);
    let h = extrude_handle(&e.sketch, &front, block).unwrap();
    assert!(!h.readable && (h.from, h.to) == (-5.0, 0.0));
    assert_eq!(extent_at(&e.sketch, &front, block, (30.0, 20.0)), None);
}
