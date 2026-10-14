//! **Revolve** (#162 F3): `about:` takes an axis lying in the face's plane as well as a line drawn
//! there (`std.z` on `std.front`); a click inside a region and the line or axis to turn it about
//! writes the face, the solid and — with a body selected — the body rule, as one splice
//! (`edit::revolve`); the turn is then sized in signed degrees, rewritten in place
//! (`edit::set_turn`), previewed before the source is written (`Sketch::set_turn`), by a handle
//! the eye sees on the arc the profile's middle turns through (`turn_handle`, `turn_at`).

use gcs_core::edit::{self, Kind};
use gcs_core::overview::workspace::{turn_at, turn_handle, Projection};
use gcs_core::program;
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::syntax::BodyWord;
use gcs_core::library;
use std::f64::consts::PI;
use crate::common::gestures::{build, names, near, seen, volume};

/// On the front (x, z): a 4 × 4 square `ab…da` with corners (10, 2) and (14, 6); a line `ax`
/// along z at x = 20; and on the top (x, y) a line `ty` — held, every one.
const DRAWN: &str = "unit mm\nuse std\nin std.front {\n\
    a := point hint((10, 2))\nb := point hint((14, 2))\nc := point hint((14, 6))\n\
    d := point hint((10, 6))\nab := line(a, b)\nbc := line(b, c)\ncd := line(c, d)\n\
    da := line(d, a)\nq0 := point hint((20, 0))\nq1 := point hint((20, 10))\nax := line(q0, q1)\n\
    fix((10, 2)) a\nfix((14, 2)) b\nfix((14, 6)) c\nfix((10, 6)) d\n\
    fix((20, 0)) q0\nfix((20, 10)) q1\n}\n\
    in std.top {\nt0 := point hint((0, 0))\nt1 := point hint((0, 10))\nty := line(t0, t1)\n\
    fix((0, 0)) t0\nfix((0, 10)) t1\n}\n";

/// The square's area and the distance of its middle from each axis it is turned about here.
const AREA: f64 = 16.0;

/// What elaborating `src` refuses, by code and message.
fn refusals(src: &str) -> Vec<(String, String)> {
    let (p, errors, _) = library::parse_linked(src);
    assert!(errors.is_empty(), "{errors:?}");
    program::elaborate(&p).errors().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect()
}

fn square() -> Vec<String> {
    names(&["ab", "bc", "cd", "da"])
}

/// Pappus: the square turned `turn` radians about an axis its middle is `r` from.
fn pappus(r: f64, turn: f64) -> f64 {
    AREA * r * turn
}

#[test]
fn a_face_turns_about_an_axis_in_its_own_plane() {
    // `std.z` and `std.x` are the front's own axes; a held axis standing in it is read there
    let cases = [
        ("about: std.z", 12.0, 2.0 * PI),
        ("about: std.x", 4.0, 2.0 * PI),
        ("about: std.z, sweep: 90deg", 12.0, PI / 2.0),
        ("about: std.z, sweep: 90deg, sense: cw", 12.0, PI / 2.0),
        ("about: up", 8.0, 2.0 * PI),
    ];
    for (how, r, turn) in cases {
        let src = format!("{DRAWN}up := axis\nfix(dir == (0, 0, 1), origin == (20, 0, 0)) up\n\
            ring := solid(face(ab, bc, cd, da), {how})\n");
        let e = build(&src);
        // faceted about the axis: the volume is good to the report's sagitta
        assert!(near(volume(&e, "ring"), pappus(r, turn), 2e-3), "{how}: {}", volume(&e, "ring"));
    }
    // the axis drawn as a line is what it always was
    assert!(near(volume(&build(&format!("{DRAWN}ring := solid(face(ab, bc, cd, da), about: ax)\n")),
        "ring"), pappus(8.0, 2.0 * PI), 2e-3));
}

#[test]
fn an_axis_off_the_faces_plane_is_refused() {
    // `std.y` runs square to the front, and a held axis may stand beside it
    for (axis, extra) in [("std.y", ""),
        ("off", "off := axis\nfix(dir == (0, 0, 1), origin == (20, 5, 0)) off\n")] {
        let src = format!("{DRAWN}{extra}ring := solid(face(ab, bc, cd, da), about: {axis})\n");
        let got = refusals(&src);
        assert!(got.iter().any(|(c, m)| c == "E081" && m.contains("does not lie in the face's own plane")),
            "{axis}: {got:?}");
    }
    // a line drawn in another view is refused as it was, and a point is no axis at all
    let got = refusals(&format!("{DRAWN}ring := solid(face(ab, bc, cd, da), about: ty)\n"));
    assert!(got.iter().any(|(c, m)| c == "E081" && m.contains("not in the face's own plane")), "{got:?}");
    let got = refusals(&format!("{DRAWN}ring := solid(face(ab, bc, cd, da), about: a)\n"));
    assert!(got.iter().any(|(c, m)| c == "E081" && m.contains("a line or an axis")), "{got:?}");
}

#[test]
fn an_axis_the_solve_places_is_read_where_it_is_evaluated() {
    // through a point of the front, parallel to z: in the plane once solved
    let placed = build(&format!("{DRAWN}up := axis hint(dir: (0.3, 0.2, 1))\nup parallel std.z\n\
        q0 coincident up\nring := solid(face(ab, bc, cd, da), about: up)\n"));
    assert!(near(volume(&placed, "ring"), pappus(8.0, 2.0 * PI), 2e-3));
    // free to turn anywhere, it is refused only where it is evaluated off the plane
    let free = build(&format!("{DRAWN}up := axis hint(dir: (0, 1, 0))\n\
        ring := solid(face(ab, bc, cd, da), about: up)\n"));
    let why = free.sketch.evaluated_solid(free.map.ent_named("ring").unwrap().i(), Report).unwrap_err();
    assert!(why.contains("does not lie in the profile's plane"), "{why}");
}

#[test]
fn a_click_and_an_axis_write_the_face_and_the_solid_as_one_splice() {
    let e = build(DRAWN);
    let out = edit::revolve(&e.program, &square(), &[], "std.z", None);
    assert_eq!((out.refused.as_deref(), out.kind), (None, Kind::Structural));
    assert_eq!(out.names, vec!["b0", "f0"]);
    assert!(out.text.ends_with("f0 := face(ab, bc, cd, da)\nb0 := solid(f0, about: std.z)\n"),
        "{}", out.text);
    assert!(near(volume(&build(&out.text), "b0"), pappus(12.0, 2.0 * PI), 2e-3));
    // about a drawn line too
    let out = edit::revolve(&e.program, &square(), &[], "ax", None);
    assert!(out.text.ends_with("b0 := solid(f0, about: ax)\n"), "{}", out.text);
}

#[test]
fn with_a_body_selected_it_joins_or_cuts() {
    let src = format!("{DRAWN}in std.front {{\ne := point hint((0, 0))\nf := point hint((30, 0))\n\
        g := point hint((30, 4))\nh := point hint((0, 4))\nef := line(e, f)\nfg := line(f, g)\n\
        gh := line(g, h)\nhe := line(h, e)\nfix((0, 0)) e\nfix((30, 0)) f\nfix((30, 4)) g\n\
        fix((0, 4)) h\n}}\ndisc := solid(face(ef, fg, gh, he), about: std.z)\n");
    let e = build(&src);
    let disc = PI * 900.0 * 4.0;
    // the square's ring stands on the disc (z 2..6 over 0..4): joined, it adds what is above
    let out = edit::revolve(&e.program, &square(), &[], "std.z", Some((BodyWord::Union, "disc")));
    assert!(out.text.ends_with("b0 union disc\n"), "{}", out.text);
    let above = PI * (196.0 - 100.0) * 2.0;
    assert!(near(volume(&build(&out.text), "disc"), disc + above, 2e-3));
    let out = edit::revolve(&e.program, &square(), &[], "std.z", Some((BodyWord::Cut, "disc")));
    assert!(out.text.ends_with("b0 cut disc\n"), "{}", out.text);
    assert!(near(volume(&build(&out.text), "disc"), disc - above, 2e-3));
}

#[test]
fn an_axis_off_the_plane_or_a_missing_one_refuses_the_gesture() {
    let e = build(DRAWN);
    for axis in ["std.y", "ty", "nowhere"] {
        let out = edit::revolve(&e.program, &square(), &[], axis, None);
        assert!(out.refused.is_some(), "{axis}: {}", out.text);
        assert_eq!(out.text, e.program.text());
    }
}

#[test]
fn a_turn_is_resized_in_place_keeping_its_axis() {
    let ring_in =
        |how: &str| build(&format!("{DRAWN}ring := solid(face(ab, bc, cd, da), {how})\n"));
    // a number is signed, a whole turn is written as none; an expression keeps the way it turns
    let cases = [
        ("about: std.z", "90", "about: std.z, sweep: 90", PI / 2.0),
        ("about: std.z", "-90", "about: std.z, sweep: 90, sense: cw", PI / 2.0),
        ("about: std.z", "-360", "about: std.z, sense: cw", 2.0 * PI),
        ("about: std.z, sense: cw", "400", "about: std.z", 2.0 * PI),
        ("about: std.z, sense: cw", "45deg * 2", "about: std.z, sweep: 45deg * 2, sense: cw",
            PI / 2.0),
    ];
    for (was, angle, said, turn) in cases {
        let e = ring_in(was);
        let ring = e.map.ent_named("ring").unwrap().i();
        let out = edit::set_turn(&e, &e.program, ring, angle);
        assert_eq!(out.refused, None, "{said}");
        assert!(out.text.contains(&format!("ring := solid(face(ab, bc, cd, da), {said})\n")),
            "{}", out.text);
        assert!(near(volume(&build(&out.text), "ring"), pappus(12.0, turn), 2e-3), "{said}");
    }
    // a prism turns about nothing, and no turn is no sweep
    let block = build(&format!("{DRAWN}block := solid(face(ab, bc, cd, da), depth: 2)\n"));
    let i = block.map.ent_named("block").unwrap().i();
    assert!(edit::set_turn(&block, &block.program, i, "90").refused.is_some());
    let e = ring_in("about: std.z");
    let ring = e.map.ent_named("ring").unwrap().i();
    assert!(edit::set_turn(&e, &e.program, ring, "0").refused.is_some());
}

#[test]
fn a_revolution_previews_a_new_turn_before_the_source_says_it() {
    let mut e = build(&format!("{DRAWN}ring := solid(face(ab, bc, cd, da), about: std.z)\n\
        block := solid(face(ab, bc, cd, da), depth: 2)\n"));
    let ring = e.map.ent_named("ring").unwrap().i();
    let epoch = e.sketch.shape_epoch;
    assert!(e.sketch.set_turn(ring, -60.0));
    assert!(near(volume(&e, "ring"), pappus(12.0, PI / 3.0), 2e-3));
    assert_ne!(e.sketch.shape_epoch, epoch, "a binding caching a mesh is told");
    assert!(e.sketch.set_turn(ring, 360.0));
    assert!(near(volume(&e, "ring"), pappus(12.0, 2.0 * PI), 2e-3));
    // a prism is no revolution, and no turn is nothing
    let block = e.map.ent_named("block").unwrap().i();
    assert!(!e.sketch.set_turn(block, 60.0));
    assert!(!e.sketch.set_turn(ring, 0.0));
}

#[test]
fn the_handle_runs_round_the_arc_and_reads_a_drag_back() {
    let mut e = build(&format!("{DRAWN}ring := solid(face(ab, bc, cd, da), about: std.z, \
        sweep: 90deg)\n"));
    let ring = e.map.ent_named("ring").unwrap().i();
    let (az, el) = (0.6, 0.5);
    let proj = Projection::new(&e.sketch, az, el);
    let close = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1) < 1e-9;
    // the square's middle (12, 0, 4) turns right-handed about z, toward +y
    let h = turn_handle(&e.sketch, &proj, ring).expect("seen from three quarters");
    assert!(close(h.base, seen([12.0, 0.0, 4.0], az, el)), "{h:?}");
    assert!(close(h.tip.unwrap(), seen([0.0, 12.0, 4.0], az, el)), "{h:?}");
    assert!(close(*h.arc.first().unwrap(), h.base) && close(*h.arc.last().unwrap(), h.tip.unwrap()));
    assert!((h.turn - 90.0).abs() < 1e-9);
    // read back: the whole turns round nearest where the drag was, so carried back past the
    // start it turns the other way, and past a whole turn it stops
    let at = |p: [f64; 3], near: f64| turn_at(&e.sketch, &proj, ring, seen(p, az, el), near).unwrap();
    assert!((at([0.0, 12.0, 4.0], 80.0) - 90.0).abs() < 1e-9);
    assert!((at([0.0, -12.0, 4.0], -80.0) + 90.0).abs() < 1e-9);
    assert!((at([0.0, -12.0, 4.0], 250.0) - 270.0).abs() < 1e-9);
    assert!((at([12.0, 0.0, 4.0], 700.0) - 360.0).abs() < 1e-9);
    // `sense: cw` turns it the other way, and the handle says so
    assert!(e.sketch.set_turn(ring, -90.0));
    let h = turn_handle(&e.sketch, &proj, ring).unwrap();
    assert!(close(h.tip.unwrap(), seen([0.0, -12.0, 4.0], az, el)) && (h.turn + 90.0).abs() < 1e-9);
    // square on to the front the turn is seen edge on: nothing to drag
    let front = Projection::new(&e.sketch, -std::f64::consts::FRAC_PI_2, 0.0);
    assert!(turn_handle(&e.sketch, &front, ring).unwrap().tip.is_none());
    assert_eq!(turn_at(&e.sketch, &front, ring, (12.0, 4.0), 90.0), None);
}
