//! Measurements of the solved drawing (`length(l)`, `radius(c)`, `distance(a, b)`,
//! `angle(l1, l2)`), read where a statement is read after the solve — a motion's `ratio:`,
//! `phase:` and `advance:` — and refused (E107) wherever the number is needed before it.
use gcs_core::{constraints::{CKind,SpecKind},io,model::MotionDef,motion,program,
    solid::{ApproximationPolicy::Report,WorldPoint},solve,syntax};

/// A stock beside an upright axis, and three lines to measure: `big` 30 long, `small` 10, and
/// `slope` 30° off `big`.
const DRAWING: &str = "\
unit mm
use std
in std.front {
o := point
z := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
axis := line(o,z)
a := point
b := point
c := point
d := point
fix(x == 1, y == 0) a
fix(x == 3, y == 0) b
fix(x == 3, y == 2) c
fix(x == 1, y == 2) d
ab := line(a,b)
bc := line(b,c)
cd := line(c,d)
da := line(d,a)
profile := face(ab,bc,cd,da)
stock := solid(profile, from: 0mm, to: 2mm)
r0 := point
r1 := point hint(x: 30,y: -4)
fix(x == 0, y == -4) r0
big := horizontal line(r0,r1)
r0 distance(30mm) r1
s0 := point
s1 := point hint(x: 10,y: -6)
fix(x == 0, y == -6) s0
small := horizontal line(s0,s1)
s0 distance(10mm) s1
k0 := point
k1 := point
fix(x == 0, y == -8) k0
fix(x == 0.8660254037844387, y == -7.5) k1
slope := line(k0,k1)
wheel := circle(center: o) hint(r: 12)
radius(12mm) wheel
}
";

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

fn same_pose(a: &motion::Family,b: &motion::Family,t: f64) {
    let (p,q) = (a.at(t).unwrap(),b.at(t).unwrap());
    for x in [[1.,0.,0.],[0.,1.,0.],[3.,-2.,5.]] {
        let (u,v) = (p.point(x),q.point(x));
        for k in 0..3 { assert!((u[k]-v[k]).abs() < 1e-9,"at {t}: {u:?} != {v:?}"); }
    }
}

/// The distance dimension on `s0`–`s1`, set to `v` and the drawing solved again.
fn resize_small(e: &mut program::Elaborated,v: f64) {
    let small = e.map.ent_named("small").unwrap();
    let line = &e.sketch.lines[small.i()];
    let (p1,p2) = (line.p1,line.p2);
    let ends = |c: &gcs_core::constraints::Constraint| c.args.iter().filter_map(|a| match a {
        gcs_core::constraints::Arg::Ent(r) => Some(r.i() as u32), _ => None }).collect::<Vec<_>>();
    let c = e.sketch.constraints.iter()
        .find(|c| c.kind == CKind::Distance && ends(c) == vec![p1,p2]).unwrap();
    let (id,slot) = (c.id,c.spec().iter().find(|s| s.1 == SpecKind::Length).unwrap().0);
    assert!(e.sketch.set_constraint_num(id,slot,v));
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
}

#[test]
fn a_measured_ratio_and_phase_turn_as_the_numbers_they_measure_and_follow_an_edit() {
    let mut e = read(&format!("{DRAWING}\
        measured := motion(about: axis, ratio: length(big) / length(small), phase: angle(big, slope))\n\
        plain := motion(about: axis, ratio: 3, phase: 30deg)\n\
        edited := motion(about: axis, ratio: 2, phase: 30deg)\n"));
    let m = index(&e,"measured");
    // what is stored is never read: a measured number is worked out when the motion is
    let MotionDef::Rotation {ratio,..} = e.sketch.motions[m].def else { panic!() };
    assert!(ratio.is_nan());
    let (ratio,phase,advance) = e.sketch.motions[m].rotation(&e.sketch).unwrap();
    assert!((ratio-3.).abs() < 1e-9 && (phase-30f64.to_radians()).abs() < 1e-9 && advance == 0.,
        "{ratio} {phase} {advance}");
    let measured = motion::Family::read(&e.sketch,m).unwrap();
    let plain = motion::Family::read(&e.sketch,index(&e,"plain")).unwrap();
    for t in [-1.3,0.,0.4,2.] { same_pose(&measured,&plain,t); }
    // a longer `small` halves nothing that was stored: the next read measures 30 / 15
    resize_small(&mut e,15.);
    let (ratio,_,_) = e.sketch.motions[m].rotation(&e.sketch).unwrap();
    assert!((ratio-2.).abs() < 1e-9,"{ratio}");
    let measured = motion::Family::read(&e.sketch,m).unwrap();
    let edited = motion::Family::read(&e.sketch,index(&e,"edited")).unwrap();
    for t in [-1.3,0.,0.4,2.] { same_pose(&measured,&edited,t); }
}

#[test]
fn radius_distance_and_advance_measure_in_space() {
    let e = read(&format!("{DRAWING}\
        screw := motion(about: axis, ratio: radius(wheel) / distance(o, k1), advance: distance(k0, big))\n\
        slide := motion(along: axis, advance: length(small) / 2)\n\
        slide_plain := motion(along: axis, advance: 5mm)\n"));
    let screw = motion::Family::read(&e.sketch,index(&e,"screw")).unwrap();
    let slide = motion::Family::read(&e.sketch,index(&e,"slide")).unwrap();
    let slide_plain = motion::Family::read(&e.sketch,index(&e,"slide_plain")).unwrap();
    // o is the origin and k1 (sin 60°, −7.5); k0 stands 4 below the line `big` runs along
    let expect = 12./0.8660254037844387f64.hypot(7.5);
    let (ratio,_,advance) = e.sketch.motions[index(&e,"screw")].rotation(&e.sketch).unwrap();
    assert!((ratio-expect).abs() < 1e-9 && (advance-4.).abs() < 1e-9,"{ratio} {advance}");
    let plain_text = format!("{DRAWING}p := motion(about: axis, ratio: {expect}, advance: 4mm)\n");
    let reference = read(&plain_text);
    let reference = motion::Family::read(&reference.sketch,index(&reference,"p")).unwrap();
    for t in [-1.,0.,0.3,1.7] {
        same_pose(&screw,&reference,t);
        same_pose(&slide,&slide_plain,t);
    }
}

#[test]
fn a_placed_solid_moves_when_the_geometry_its_motion_measures_does() {
    let mut e = read(&format!("{DRAWING}\
        turn := motion(about: axis, ratio: length(big) / length(small))\n\
        moved := solid(stock, under: turn, at: 30deg)\n"));
    let moved = index(&e,"moved");
    let stock = e.sketch.evaluated_solid(index(&e,"stock"),Report).unwrap();
    let inside = WorldPoint([2.,-1.,1.]);
    assert!(stock.contains_world(inside));
    let at = 30f64.to_radians();
    let where_now = |e: &program::Elaborated| {
        let f = motion::Family::read(&e.sketch,index(e,"turn")).unwrap();
        WorldPoint(f.at(at).unwrap().point(inside.0))
    };
    let before = e.sketch.evaluated_solid(moved,Report).unwrap();
    let key = gcs_core::solid::reads(&e.sketch,moved,0.);
    assert!(before.contains_world(where_now(&e)));
    // 90° at ratio 3; the edit makes it 60° at ratio 2, and the cache must see it
    resize_small(&mut e,15.);
    assert_ne!(key,gcs_core::solid::reads(&e.sketch,moved,0.));
    let after = e.sketch.evaluated_solid(moved,Report).unwrap();
    assert!(!std::rc::Rc::ptr_eq(&before,&after));
    assert!(after.contains_world(where_now(&e)));
    assert!((after.volume()-8.).abs() < 1e-9);
}

#[test]
fn a_measured_motion_is_copied_with_what_it_measures_and_goes_with_it() {
    let e = read(&format!("{DRAWING}\
        turn := motion(about: axis, ratio: length(big) / length(small))\n\
        moved := solid(stock, under: turn, at: 30deg)\n"));
    // a copy of the placed solid brings the motion, and the motion what it measures
    let copied = io::copy(&e.sketch,&[e.map.ent_named("moved").unwrap()]);
    assert_eq!(copied.motions.len(),1);
    assert_eq!(copied.motions[0].rotation(&copied).unwrap().0,3.);
    let i = copied.solids.iter().position(|s| s.name == "moved").unwrap();
    let original = e.sketch.evaluated_solid(index(&e,"moved"),Report).unwrap();
    assert!((copied.evaluated_solid(i,Report).unwrap().volume()-original.volume()).abs() < 1e-9);
    // deleting a measured line takes the motion and the placement with it
    let gone = io::without(&e.sketch,&[e.map.ent_named("small").unwrap()],&[]);
    assert!(gone.motions.is_empty() && !gone.solids.iter().any(|s| s.name == "moved"));
    // and so does deleting it from the source
    let edit = gcs_core::edit::remove(&e,&e.program,&e.sketch,&[e.map.ent_named("small").unwrap()],&[]);
    assert!(edit.refused.is_none(),"{:?}",edit.refused);
    assert!(!edit.text.contains("turn := motion"),"{}",edit.text);
}

#[test]
fn the_source_and_the_flat_print_keep_the_measurement_as_written() {
    let src = format!("{DRAWING}turn := motion(about: axis, ratio: length(big) / length(small), \
        phase: angle(big, slope))\n");
    let e = read(&src);
    let mut program = e.program.clone();
    let flat = syntax::render_flat(&mut program).unwrap().to_string();
    assert!(flat.contains("ratio: length(big) / length(small), phase: angle(big, slope)"),"{flat}");
    let again = read(&flat);
    let (a,b) = (e.sketch.motions[0].rotation(&e.sketch).unwrap(),
        again.sketch.motions[0].rotation(&again.sketch).unwrap());
    assert!((a.0-b.0).abs() < 1e-12 && (a.1-b.1).abs() < 1e-12);
}

#[test]
fn a_measurement_inside_a_component_names_the_instance_geometry() {
    let e = read("\
unit mm
use std
component Pair(axis: line, big: Length, small: Length) {
  r0 := point
  r1 := point hint(x: 30,y: -4)
  fix(x == 0, y == -4) r0
  wheel := horizontal line(r0,r1)
  r0 distance(big) r1
  s0 := point
  s1 := point hint(x: 10,y: -6)
  fix(x == 0, y == -6) s0
  pinion := horizontal line(s0,s1)
  s0 distance(small) s1
  turn := motion(about: axis, ratio: -length(wheel) / distance(pinion.p1, pinion.p2))
}
in std.front {
o := point
z := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
axis := line(o,z)
one := Pair(axis, big: 30mm, small: 10mm)
two := Pair(axis, big: 12mm, small: 3mm)
}
");
    // each instance measures its own lines, named under it
    for (name,ratio) in [("one.turn",-3.),("two.turn",-4.)] {
        let m = &e.sketch.motions[index(&e,name)];
        assert!((m.rotation(&e.sketch).unwrap().0-ratio).abs() < 1e-9,"{name}");
        assert!(m.measured[0].value.text.contains(&format!("{}.wheel",&name[..3])),"{}",m.measured[0].value.text);
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

#[test]
fn a_measurement_read_before_the_solve_is_refused_with_the_stratum() {
    let mark = gcs_core::expr::MEASURE_MARK;
    // a param feeds constraints, so it is needed before the solve
    refused(&format!("{DRAWING}k := length(big)\n"),"E107",mark);
    // a seed is where the solve begins
    refused(&format!("{DRAWING}p := point hint(x: length(big), y: 0)\n"),"E107",mark);
    // a constraint's number is what the solve is solving for
    refused(&format!("{DRAWING}q := point hint(x: 5,y: 5)\nq distance(length(big)) o\n"),"E107",mark);
    // a solid's extent is settled at elaboration
    refused(&format!("{DRAWING}tall := solid(profile, depth: length(small))\n"),"E107",mark);
    // an attitude and a placement angle are both read before anything measures
    refused(&format!("{DRAWING}turn := motion(about: axis)\nmoved := solid(stock, under: turn, at: angle(big, slope))\n"),
        "E107",mark);
    // one refusal, not one per path that carried it
    let d = diag(&format!("{DRAWING}k := length(big)\n"));
    assert_eq!(d.iter().filter(|(_,m)| m.contains(mark)).count(),1,"{d:?}");
}

#[test]
fn a_measurement_must_come_to_its_slots_dimension_over_geometry_it_can_measure() {
    // a ratio is a plain number: a length over a length is one, a length is not
    refused(&format!("{DRAWING}turn := motion(about: axis, ratio: length(big))\n"),"E080","ratio");
    // a phase is an angle (a bare ratio would read as degrees, as a bare number does)
    refused(&format!("{DRAWING}turn := motion(about: axis, phase: length(big))\n"),"E080","phase");
    // a length is of a line or an arc
    refused(&format!("{DRAWING}turn := motion(about: axis, ratio: length(o) / length(small))\n"),
        "E080","measures a line or an arc");
    refused(&format!("{DRAWING}turn := motion(about: axis, ratio: radius(big) / 1mm)\n"),"E080","radius");
    // a name the drawing does not have
    refused(&format!("{DRAWING}turn := motion(about: axis, ratio: length(nothing) / length(small))\n"),
        "E101","nothing");
    // and the arguments are names, not numbers
    let d = diag(&format!("{DRAWING}turn := motion(about: axis, ratio: length(3))\n"));
    assert!(d.iter().any(|(_,m)| m.contains("its arguments are names")),"{d:?}");
}

#[test]
fn the_spiral_bevel_rolls_measured_off_the_pitch_geometry_are_its_trig_ratios() {
    // the bevel pair, whose rolls the layout measures off its pitch triangles
    // (`generation.sv`), each written a second time with its ratio in closed form: a pitch
    // cone's distance over its pitch radius is one over the sine of its pitch angle
    let anchor = "  gear_generation := motion(crown_roll, relative_to: gear_roll)\n";
    let e = fixtures::gear::read_configured_with(&mut |name,text| {
        let text = fixtures::gear::bevel(name,text);
        if name != "generation" { return text; }
        assert!(text.contains(anchor));
        text.replace(anchor,&format!("{anchor}\
  trig_pinion_roll := motion(about: pinion.axis, ratio: 1 / sin(atan2(24, 48)))\n\
  trig_gear_roll := motion(about: gear.axis, ratio: -1 / sin(atan2(48, 24)))\n"))
    });
    let find = |suffix: &str| e.sketch.motions.iter().position(|m| m.name.ends_with(suffix))
        .unwrap_or_else(|| panic!("no motion `{suffix}`"));
    for member in ["pinion_roll","gear_roll"] {
        let (measured,trig) = (find(&format!(".{member}")),find(&format!(".trig_{member}")));
        let MotionDef::Rotation {ratio,..} = e.sketch.motions[trig].def else { panic!() };
        let read = e.sketch.motions[measured].rotation(&e.sketch).unwrap().0;
        assert!((read-ratio).abs() < 1e-9*ratio.abs(),"{member}: {read} against {ratio}");
        let (a,b) = (motion::Family::read(&e.sketch,trig).unwrap(),
            motion::Family::read(&e.sketch,measured).unwrap());
        for t in [-0.6,-0.1,0.,0.25,0.6] { same_pose(&a,&b,t); }
    }
}

#[test]
fn the_pin_wheel_example_times_its_roll_by_its_drawn_radii() {
    let src = include_str!("../../examples/lantern_generation.sv");
    for (wheel,ratio) in [("16mm",-2.),("20mm",-2.5)] {
        let e = read(&src.replace("wheel_pitch := 16mm",&format!("wheel_pitch := {wheel}")));
        let read = e.sketch.motions[index(&e,"pinion_turn")].rotation(&e.sketch).unwrap().0;
        assert!((read-ratio).abs() < 1e-9,"{wheel}: {read}");
        assert!(e.sketch.solids.iter().any(|s| s.name == "space"));
    }
}
