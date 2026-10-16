//! Measurements of the solved drawing (`length(l)`, `radius(c)`, `distance(a, b)`,
//! `angle(l1, l2)`), refused (E107) wherever the number is needed before the solve — and a
//! frame's joint is (#383): its ratio and advance are numbers relations hold, so a turn's ratio
//! is stated from the document's own numbers (`ratio: wheel_pitch / pinion_pitch`), where an edit
//! of the number re-times it, and a measured one is refused.  The motion that read its numbers
//! after the solve, `motion(about: l, ratio: length(a) / length(b), phase: …)`, is retired, and
//! refused naming the words that replace it.
use gcs_core::{io,model::MotionDef,motion,program,
    solid::{ApproximationPolicy::Report,WorldPoint},solve,syntax};

/// A stock beside an upright axis, and three lines: `big` `big_len` (30) long, `small`
/// `small_len` (10), and `slope` 30° off `big`.
const DRAWING: &str = "\
unit mm
use std
big_len := 30mm
small_len := 10mm
in std.front {
o := point
z := point
fix((0, 0)) o
fix((0, 1)) z
ax := line(o,z)
a := point
b := point
c := point
d := point
fix((1, 0)) a
fix((3, 0)) b
fix((3, 2)) c
fix((1, 2)) d
ab := line(a,b)
bc := line(b,c)
cd := line(c,d)
da := line(d,a)
profile := face(ab,bc,cd,da)
stock := solid(profile, from: 0mm, to: 2mm)
r0 := point
r1 := point hint((30, -4))
fix((0, -4)) r0
big := horizontal line(r0,r1)
r0 distance(big_len) r1
s0 := point
s1 := point hint((10, -6))
fix((0, -6)) s0
small := horizontal line(s0,s1)
s0 distance(small_len) s1
k0 := point
k1 := point
fix((0, -8)) k0
fix((0.8660254037844387, -7.5)) k1
slope := line(k0,k1)
wheel := circle(center: o) hint(r: 12)
radius(12mm) wheel
}
";

/// `DRAWING` with `small_len` stated `v`: the document edited, as `examples::document`'s
/// `with_params` gives a named number the caller's value.
fn small_at(src: &str,v: f64) -> String {
    assert!(src.contains("small_len := 10mm\n"));
    src.replace("small_len := 10mm\n",&format!("small_len := {v}mm\n"))
}

fn build(src: &str) -> program::Elaborated {
    let (mut p,errors) = crate::common::parse(src);
    assert!(errors.is_empty(),"{errors:?}");
    assert!(gcs_core::modules::link(&mut p,&mut gcs_core::library::resolve).is_empty());
    program::elaborate(&p)
}

fn read(src: &str) -> program::Elaborated {
    let mut e = build(src);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}

fn index(e: &program::Elaborated,name: &str) -> usize { e.map.ent_named(name).unwrap().i() }

/// The motion node of the frame `name`: its joint's.
fn joint(e: &program::Elaborated,name: &str) -> usize {
    e.sketch.frames[index(e,name)].joint.as_ref().and_then(|j| j.motion)
        .unwrap_or_else(|| panic!("`{name}` has no joint")) as usize
}

fn family(e: &program::Elaborated,name: &str) -> motion::Family {
    motion::Family::read(&e.sketch,joint(e,name)).unwrap()
}

fn same_pose(a: &motion::Family,b: &motion::Family,t: f64) {
    let (p,q) = (a.at(t).unwrap(),b.at(t).unwrap());
    for x in [[1.,0.,0.],[0.,1.,0.],[3.,-2.,5.]] {
        let (u,v) = (p.point(x),q.point(x));
        for k in 0..3 { assert!((u[k]-v[k]).abs() < 1e-9,"at {t}: {u:?} != {v:?}"); }
    }
}

fn diag(src: &str) -> Vec<(String,String)> {
    let e = build(src);
    e.diags.iter().map(|d| (d.code.as_str().to_string(),d.message.clone())).collect()
}

fn refused(src: &str,code: &str,words: &str) {
    let d = diag(src);
    assert!(d.iter().any(|(c,m)| c == code && m.contains(words)),"{d:?}");
}

/// **A ratio stated from the document's numbers turns as the number it comes to, and follows an
/// edit**: `big_len / small_len` is 3, as `ratio: 3` is, and with `small_len` 15 it is 2.  A turn
/// that stood at a phase is read at the roll shifted by it: a phase of 30° at ratio 3 is the roll
/// 10° on.  Measured off the drawing, the same ratio is refused: a joint's numbers are needed
/// before the solve.
#[test]
fn a_stated_ratio_turns_as_the_number_it_states_and_follows_an_edit() {
    let turns = "stated := frame\nstd.Turn(stated, about: ax, ratio: big_len / small_len)\n\
        plain := frame\nstd.Turn(plain, about: ax, ratio: 3)\n\
        edited := frame\nstd.Turn(edited, about: ax, ratio: 2)\n";
    let src = format!("{DRAWING}{turns}");
    let e = read(&src);
    let (ratio,phase,advance) = e.sketch.motions[joint(&e,"stated")].rotation(&e.sketch).unwrap();
    assert!((ratio-3.).abs() < 1e-12 && phase == 0. && advance == 0.,"{ratio} {phase} {advance}");
    for t in [-1.3,0.,0.4,2.] { same_pose(&family(&e,"stated"),&family(&e,"plain"),t); }
    // a longer `small` re-times it: 30 / 15
    let e = read(&small_at(&src,15.));
    let (ratio,_,_) = e.sketch.motions[joint(&e,"stated")].rotation(&e.sketch).unwrap();
    assert!((ratio-2.).abs() < 1e-12,"{ratio}");
    for t in [-1.3,0.,0.4,2.] { same_pose(&family(&e,"stated"),&family(&e,"edited"),t); }
    // the roll a phase stood at: 30° at ratio 3 is 10° of roll, the turn of `3 · 0.4 + 30°` that
    // the ratio-2 turn makes at half of it
    let phase = 30f64.to_radians();
    let at = family(&e,"plain").at(0.4+phase/3.).unwrap();
    let want = family(&e,"edited").at((3.*0.4+phase)/2.).unwrap();
    for x in [[1.,0.,0.],[3.,-2.,5.]] {
        let (u,v) = (at.point(x),want.point(x));
        for k in 0..3 { assert!((u[k]-v[k]).abs() < 1e-9,"{u:?} != {v:?}"); }
    }
    refused(&format!("{DRAWING}measured := frame\n\
        std.Turn(measured, about: ax, ratio: length(big) / length(small))\n"),
        "E107","`length(big)` measures the solved drawing");
}

/// **A joint's numbers are stated, never measured**: a ratio over a radius and a distance, a
/// screw's advance a distance and a slide's half a length are each refused, a measurement
/// being read before the solve (E107); the same numbers stated turn and slide as they say.
#[test]
fn a_joints_ratio_and_advance_are_stated_and_a_measured_one_is_refused() {
    for tail in [
        "screw := frame\nstd.Turn(screw, about: ax, ratio: radius(wheel) / distance(o, k1))\n",
        "screw := frame\nstd.Turn(screw, about: ax, ratio: 1, advance: distance(k0, big))\n",
        "slide := frame\nstd.Slide(slide, along: ax, advance: length(small) / 2)\n",
    ] {
        refused(&format!("{DRAWING}{tail}"),"E107",gcs_core::expr::MEASURE_MARK);
    }
    // o is the origin and k1 (sin 60°, −7.5); k0 stands 4 below the line `big` runs along
    let expect = 12./0.8660254037844387f64.hypot(7.5);
    let e = read(&format!("{DRAWING}screw := frame\n\
        std.Turn(screw, about: ax, ratio: {expect}, advance: 4mm)\n\
        slide := frame\nstd.Slide(slide, along: ax, advance: small_len / 2)\n"));
    let (ratio,_,advance) = e.sketch.motions[joint(&e,"screw")].rotation(&e.sketch).unwrap();
    assert!((ratio-expect).abs() < 1e-12 && (advance-4.).abs() < 1e-12,"{ratio} {advance}");
    let slide = &e.sketch.motions[joint(&e,"slide")];
    assert!(matches!(slide.def,MotionDef::Translation {..}),"{:?}",slide.def);
    assert!((slide.advance(&e.sketch).unwrap()-5.).abs() < 1e-12);
    // a turn of the roll slides it the advance along the axis, up the page
    let up = family(&e,"slide").at(std::f64::consts::TAU).unwrap().point([0.;3]);
    let page_up = e.sketch.world_point(index(&e,"z"));
    assert!((up.iter().zip(page_up).map(|(u,v)| (u-5.*v)*(u-5.*v)).sum::<f64>()).sqrt() < 1e-9,
        "{up:?}");
}

/// **A placed solid moves when the number its frame turns by is edited**: 30° of roll at ratio
/// 3 is 90°, and with `small_len` 15, 60° at ratio 2 — each time the copy holds the stock's
/// point turned so, and its volume the stock's.
#[test]
fn a_placed_solid_moves_when_the_number_its_frame_turns_by_does() {
    let src = format!("{DRAWING}turn := frame\n\
        std.Turn(turn, about: ax, ratio: big_len / small_len)\n\
        moved := solid(stock, under: turn, at: 30deg)\n");
    let inside = WorldPoint([2.,-1.,1.]);
    let mut seen = Vec::new();
    for small in [10.,15.] {
        let e = read(&small_at(&src,small));
        let stock = e.sketch.evaluated_solid(index(&e,"stock"),Report).unwrap();
        assert!(stock.contains_world(inside));
        let moved = e.sketch.evaluated_solid(index(&e,"moved"),Report).unwrap();
        let there = WorldPoint(family(&e,"turn").at(30f64.to_radians()).unwrap().point(inside.0));
        assert!(moved.contains_world(there),"{small}");
        assert!((moved.volume()-8.).abs() < 1e-9);
        seen.push(there.0);
    }
    assert!((0..3).any(|k| (seen[0][k]-seen[1][k]).abs() > 0.5),"the edit moved nothing: {seen:?}");
}

/// A copy of a placed solid brings its frame's motion, at the ratio it states, and deleting the
/// line the frame turns about, or the frame, takes the motion and the placement with it, in the
/// sketch and in the source.
#[test]
fn a_placed_solids_motion_is_copied_with_it_and_goes_with_its_axis() {
    let e = read(&format!("{DRAWING}turn := frame\n\
        std.Turn(turn, about: ax, ratio: big_len / small_len)\n\
        moved := solid(stock, under: turn, at: 30deg)\n"));
    let copied = io::copy(&e.sketch,&[e.map.ent_named("moved").unwrap()]);
    assert_eq!(copied.motions.len(),1);
    assert_eq!(copied.motions[0].rotation(&copied).unwrap().0,3.);
    let i = copied.solids.iter().position(|s| s.name == "moved").unwrap();
    let original = e.sketch.evaluated_solid(index(&e,"moved"),Report).unwrap();
    assert!((copied.evaluated_solid(i,Report).unwrap().volume()-original.volume()).abs() < 1e-9);
    for name in ["ax","turn"] {
        let what = e.map.ent_named(name).unwrap();
        let gone = io::without(&e.sketch,&[what],&[]);
        assert!(gone.motions.is_empty() && !gone.solids.iter().any(|s| s.name == "moved"),"{name}");
        let edit = gcs_core::edit::remove(&e,&e.program,&e.sketch,&[what],&[]);
        assert!(edit.refused.is_none(),"{name}: {:?}",edit.refused);
        assert!(!edit.text.contains("moved :=") && !edit.text.contains("std.Turn"),"{name}: {}",
            edit.text);
    }
}

/// The flat print keeps a joint's number as it is written: the ratio over the document's two
/// numbers, read back to the same turn.
#[test]
fn the_flat_print_keeps_a_joints_number_as_written() {
    // the flat form prints no component instance: the joint written out as `std.Turn` writes it
    let src = format!("{DRAWING}turn := frame\nax coincident turn.axis\n\
        fix(ratio == big_len / small_len, advance == 0mm) turn\n");
    let e = read(&src);
    let mut program = e.program.clone();
    let flat = syntax::render_flat(&mut program).unwrap().to_string();
    assert!(flat.contains("ratio == big_len / small_len"),"{flat}");
    let again = read(&flat);
    let (a,b) = (e.sketch.motions[joint(&e,"turn")].rotation(&e.sketch).unwrap(),
        again.sketch.motions[joint(&again,"turn")].rotation(&again.sketch).unwrap());
    assert!((a.0-3.).abs() < 1e-12 && (a.0-b.0).abs() < 1e-12 && (a.1-b.1).abs() < 1e-12);
}

/// A ratio inside a component is stated from the instance's own numbers, each instance turning
/// at its own; measured off the instance's lines it is refused, naming the measurement as
/// written.
#[test]
fn a_ratio_inside_a_component_is_stated_from_the_instances_numbers() {
    let pair = |ratio: &str| format!("\
unit mm
use std
component Pair(ax: line, big: Length, small: Length) {{
  r0 := point
  r1 := point hint((30, -4))
  fix((0, -4)) r0
  wheel := horizontal line(r0,r1)
  r0 distance(big) r1
  s0 := point
  s1 := point hint((10, -6))
  fix((0, -6)) s0
  pinion := horizontal line(s0,s1)
  s0 distance(small) s1
  turn := frame
  std.Turn(turn, about: ax, ratio: {ratio})
}}
in std.front {{
o := point
z := point
fix((0, 0)) o
fix((0, 1)) z
ax := line(o,z)
one := Pair(ax, big: 30mm, small: 10mm)
two := Pair(ax, big: 12mm, small: 3mm)
}}
");
    let e = read(&pair("-big / small"));
    for (name,ratio) in [("one.turn",-3.),("two.turn",-4.)] {
        let m = &e.sketch.motions[joint(&e,name)];
        assert!((m.rotation(&e.sketch).unwrap().0-ratio).abs() < 1e-12,"{name}");
    }
    refused(&pair("-length(wheel) / distance(pinion.p1, pinion.p2)"),"E107","`length(wheel)`");
}

#[test]
fn a_measurement_read_before_the_solve_is_refused_with_the_stratum() {
    let mark = gcs_core::expr::MEASURE_MARK;
    // a param feeds constraints, so it is needed before the solve
    refused(&format!("{DRAWING}k := length(big)\n"),"E107",mark);
    // a seed is where the solve begins
    refused(&format!("{DRAWING}p := point hint((length(big), 0))\n"),"E107",mark);
    // a constraint's number is what the solve is solving for
    refused(&format!("{DRAWING}q := point hint((5, 5))\nq distance(length(big)) o\n"),"E107",mark);
    // a solid's extent is settled at elaboration
    refused(&format!("{DRAWING}tall := solid(profile, depth: length(small))\n"),"E107",mark);
    // an attitude and a placement angle are both read before anything measures
    refused(&format!("{DRAWING}turn := frame\nstd.Turn(turn, about: ax, ratio: 1)\n\
        moved := solid(stock, under: turn, at: angle(big, slope))\n"),"E107",mark);
    // and a joint's numbers are held by relations, so they are too
    refused(&format!("{DRAWING}turn := frame\nstd.Turn(turn, about: ax, ratio: 1, \
        advance: length(small))\n"),"E107",mark);
    // one refusal, not one per path that carried it
    let d = diag(&format!("{DRAWING}k := length(big)\n"));
    assert_eq!(d.iter().filter(|(_,m)| m.contains(mark)).count(),1,"{d:?}");
}

/// The stratum is said first: a measurement in a joint is refused as one, whatever it measures —
/// a length that is no plain number, a point's length, a line's radius, a name the drawing does
/// not have — and a measurement's arguments are names, not numbers.
#[test]
fn a_measurement_in_a_joint_is_refused_by_its_stratum_and_its_arguments_are_names() {
    let mark = gcs_core::expr::MEASURE_MARK;
    for ratio in ["length(big)","length(o) / length(small)","radius(big) / 1mm",
                  "length(nothing) / length(small)"] {
        refused(&format!("{DRAWING}turn := frame\nstd.Turn(turn, about: ax, ratio: {ratio})\n"),
            "E107",mark);
    }
    let d = diag(&format!("{DRAWING}turn := frame\nstd.Turn(turn, about: ax, ratio: length(3))\n"));
    assert!(d.iter().any(|(_,m)| m.contains("its arguments are names")),"{d:?}");
}

/// **The retired motion is refused naming what replaced it**, as the document is read:
/// `motion(about: …)` names `std.Turn` and `std.Spin`, `motion(along: …)` `std.Slide`, `phase:`
/// the roll it shifts, and a motion relative to another the reading's `seen_from:`.
#[test]
fn the_retired_motion_is_refused_naming_its_replacement() {
    // a syntax error: the parser says it
    let refused = |src: &str,words: &str| {
        let (_,errors) = crate::common::parse(src);
        assert!(errors.iter().any(|e| e.message.contains(words)),"{words}: {errors:?}");
    };
    refused(&format!("{DRAWING}turn := motion(about: ax, ratio: length(big) / length(small))\n"),
        "std.Turn(f, about: l, ratio: r)");
    refused(&format!("{DRAWING}turn := motion(about: o, ratio: 1)\n"),"std.Spin(f, about: o");
    refused(&format!("{DRAWING}slide := motion(along: ax, advance: 3mm)\n"),
        "std.Slide(f, along: l, advance: a)");
    refused(&format!("{DRAWING}turn := motion(about: ax, ratio: 3, phase: angle(big, slope))\n"),
        "roll 0 is as drawn");
    refused(&format!("{DRAWING}a1 := frame\nstd.Turn(a1, about: ax, ratio: 1)\n\
        b1 := frame\nstd.Turn(b1, about: ax, ratio: 2)\nrel := motion(a1, relative_to: b1)\n"),
        "`under: a, seen_from: b`");
}

#[test]
fn the_spiral_bevel_rolls_by_tooth_count_are_its_trig_ratios() {
    // the bevel pair, whose rolls are the crown's tooth count over each member's (`hypoid.sv`'s
    // step 5, #283), each written a second time with its ratio in closed form: a pitch cone's
    // distance over its pitch radius is one over the sine of its pitch angle
    let anchor = "  crown_pitch := -360deg / crown_teeth\n";
    let text = fixtures::gear::bevel(&fixtures::gear::source());
    assert!(text.contains(anchor));
    let e = fixtures::gear::read(&text.replace(anchor,&format!("{anchor}\
  trig_pinion_roll := frame\n\
  std.Turn(trig_pinion_roll, about: pinion_cone.ax, ratio: 1 / sin(atan2(24, 48)))\n\
  trig_gear_roll := frame\n\
  std.Turn(trig_gear_roll, about: gear_cone.ax, ratio: -1 / sin(atan2(48, 24)))\n")));
    let find = |name: &str| joint(&e,&format!("pair.{name}"));
    for member in ["pinion_roll","gear_roll"] {
        let (counted,trig) = (find(member),find(&format!("trig_{member}")));
        let MotionDef::Rotation {ratio,..} = e.sketch.motions[trig].def else { panic!() };
        let read = e.sketch.motions[counted].rotation(&e.sketch).unwrap().0;
        assert!((read-ratio).abs() < 1e-9*ratio.abs(),"{member}: {read} against {ratio}");
        let (a,b) = (motion::Family::read(&e.sketch,trig).unwrap(),
            motion::Family::read(&e.sketch,counted).unwrap());
        for t in [-0.6,-0.1,0.,0.25,0.6] { same_pose(&a,&b,t); }
    }
}

#[test]
fn the_pin_wheel_example_times_its_roll_by_its_pitch_radii() {
    let src = include_str!("../../examples/lantern_generation.sv");
    for (wheel,ratio) in [("16mm",-2.),("20mm",-2.5)] {
        let e = read(&src.replace("wheel_pitch := 16mm",&format!("wheel_pitch := {wheel}")));
        let read = e.sketch.motions[joint(&e,"pinion_turn")].rotation(&e.sketch).unwrap().0;
        assert!((read-ratio).abs() < 1e-9,"{wheel}: {read}");
        assert!(e.sketch.solids.iter().any(|s| s.name == "space"));
    }
}
