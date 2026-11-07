//! **A sweep distributes over union** (#684 C, #586, `solid::distributed`): a body cutting the
//! sweep of a union cuts each member's sweep, under the union's motion over its roll, each admitted
//! by its own class and refused by its own rows naming the member. What each cut removes is computed
//! here again from the tools and the motion alone and held to the exact solid this kernel builds,
//! whose STL is held to the field of the union's sweep; and the union cuts what its members cut
//! written one after another.
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Class,Error,Options};
use std::f64::consts::PI;

use crate::solid_sweeps_examples::text;
use crate::turns_and_slides::{exported,measure,near};

fn admit(e: &Elaborated,body: &str) -> Result<admission::Admission,Error> {
    admission::admit_body(&e.sketch,fixtures::solid(e,body),&Options::default())
}

/// The series' wheel of two beads (`solid_sweeps/union.sv`), its text: the class's simplest case,
/// held there to its class and its volume (`solid_sweeps_examples.rs`).
fn beads() -> String { text("union.sv") }

/// Whether the point (x, y) square to the slide is in the shadow along z of a torus about the line
/// x = 11.5, z = −9 along y: its ring `ring` about that line in the plane y = `at`, its tube `tube`.
/// The ring's points stand at every distance `ring` from the line, so a point no farther than that
/// from it is under the ring wherever the tube's width covers it, and one farther is under the
/// ring's nearest point.
fn shadowed(x: f64,y: f64,ring: f64,tube: f64,at: f64) -> bool {
    let d = (x-11.5).abs();
    if d <= ring { (y-at).abs() <= tube } else { (d-ring).hypot(y-at) <= tube }
}

/// The groove's section: what of the rod's disc either bead's shadow covers, in rows across the
/// rod from where the nearer tube's shadow begins, 2 mm short of its centre at 5.5.
pub(crate) fn beads_section() -> f64 {
    let hit = |x: f64,y: f64| shadowed(x,y,6.,2.,0.) || shadowed(x,y,6.,1.5,1.5);
    measure([3.5,5.],&|x| { let h = (25.-x*x).max(0.).sqrt(); [-h,h] },&hit,&|a,b| b-a)
}

/// The beads' document with its groove written as each bead's sweep cut in turn: the long form the
/// union stands for.
fn beads_one_after_another() -> String {
    let text = beads();
    let at = text.find("construction groove :=").unwrap();
    format!("{}construction groove := solid({{ p | some t in (0deg, 360deg): p in bead at(under: feed) t }})\n\
        construction groove2 := solid({{ p | some t in (0deg, 360deg): p in bead2 at(under: feed) t }})\ngroove cut rod\ngroove2 cut rod\n",&text[..at])
}

/// The union's groove is its members' grooves cut one after another, to the kernel's rounding: the
/// two builds are one construction.
#[test]
fn the_union_cuts_what_its_members_cut_in_turn() {
    let (union,long) = (fixtures::read(&beads()),fixtures::read(&beads_one_after_another()));
    near(exported(&union,"rod"),exported(&long,"rod"),1e-9);
}

/// The beads' groove placed a second time half a turn round the rod: each bead's sweep is minted
/// once and placed twice, and the second groove takes as much again from the far side.
#[test]
fn a_placed_unions_sweep_places_each_members() {
    let text = beads().replace("(slides)","(slides, turns)")+"rounding := frame\nrounding turns(1) ax\nconstruction far := groove at(under: rounding) 180deg\nfar cut rod\n";
    let e = fixtures::read(&text);
    let a = admit(&e,"rod").unwrap();
    assert_eq!(a.sweeps().len(),2,"one sweep a bead");
    assert!(a.sweeps().iter().all(|s| s.placements.len() == 2),"each bead's sweep placed where the union's is");
    near(exported(&e,"rod"),PI*25.*20.-2.*beads_section()*20.,2e-6);
}

/// `text` with its groove swept from `tool` where it was swept from `was`, and `before` written
/// ahead of it.
fn regroove(text: &str,was: &str,before: &str,tool: &str) -> String {
    let at = text.find("construction groove := solid(").expect("the groove");
    let end = at+text[at..].find('\n').unwrap();
    let line = &text[at..end];
    let swept = format!("p in {was} at(");
    assert!(line.contains(&swept),"{line}");
    format!("{}{before}{}{}",&text[..at],line.replace(&swept,&format!("p in {tool} at(")),&text[end..])
}

/// A union placed whole is distributed as its members each placed so: the wheel turned about its
/// own axis is the same wheel, and cuts the same groove.
#[test]
fn a_union_placed_whole_is_its_members_placed() {
    let text = regroove(&beads().replace("(slides)","(slides, turns)"),"wheel","spin := frame\nspin turns(1) wheel_axis\n\
        construction turned := wheel at(under: spin) 30deg\n","turned");
    let e = fixtures::read(&text);
    assert_eq!(admit(&e,"rod").unwrap().sweeps().len(),2);
    near(exported(&e,"rod"),PI*25.*20.-beads_section()*20.,2e-6);
}

/// Two balls on one arm swung a quarter turn, half sunk in a block: each ball's sweep is its tube
/// (the canal), so the body is static material once its union is distributed, and the grooves are
/// the two half tubes with half a ball at each end (Pappus).
#[test]
fn two_balls_swung_are_two_tubes() {
    let text = regroove(&text("ball.sv"),"cutter","\
in std.front {
  private centre2 := point
  centre2 coords(du: 6mm, dv: 0mm) std.front
  private bottom2 := point
  private top2 := point
  bottom2 coords(du: 6mm, dv: -2mm) std.front
  top2 coords(du: 6mm, dv: 2mm) std.front
  private diameter2 := line(bottom2, top2)
  private meridian2 := arc(center: centre2, start: bottom2, end: top2) hint(r: 2)
  radius(2mm) meridian2
}
construction small := solid(std.solids.Revolution(face(meridian2, diameter2), about: diameter2))
construction pair := solid(cutter)
small union pair
","pair");
    let e = fixtures::read(&text);
    let a = admit(&e,"block").unwrap();
    assert!(a.sweeps().is_empty(),"each member's sweep is a ball's tube, static material");
    let half = |r: f64,swing: f64| (PI*r*r*swing*PI/2.+4./3.*PI*r*r*r)/2.;
    near(exported(&e,"block"),24.*24.*8.-half(3.,12.)-half(2.,6.),1e-9);
}

/// A tool's member is asked its own class's rows, and admitted by the closed class where they
/// refuse it (#721 F2c5, a torus slid square to its axis): the second bead grown to reach the rod
/// with the rings where its tube lies flat in contact all round (the constant-twist class's S1). Its
/// other member stays the constant-twist class's: each member its own sweep, the body is built by
/// that class's road and the closed member cut from what it leaves on one arrangement. The groove is
/// what of the rod's disc either bead's shadow covers, the second's ring now 7 from the wheel's
/// axis, through the rod's 20.
#[test]
fn a_member_its_class_refuses_is_the_closed_classs() {
    let text = beads().replace("tc2 coords(du: reach, dv: step) bench","tc2 coords(du: reach - 1mm, dv: step) bench");
    let e = fixtures::read(&text);
    let a = admit(&e,"rod").unwrap();
    assert!(a.sweeps().iter().any(|s| matches!(s.class,Class::Closed(_))),"a member the closed class's");
    assert!(a.sweeps().iter().any(|s| !matches!(s.class,Class::Closed(_))),"a member another class's");
    let hit = |x: f64,y: f64| shadowed(x,y,6.,2.,0.) || shadowed(x,y,7.,1.5,1.5);
    let section = measure([3.,5.],&|x| { let h = (25.-x*x).max(0.).sqrt(); [-h,h] },&hit,&|a,b| b-a);
    near(exported(&e,"rod"),PI*25.*20.-section*20.,2e-6);
}

/// A union within an intersection is not distributed, but read whole by its B-rep — two tori's faces
/// and the concave crease where they meet — it is the constant twist's: both faces' points meet that
/// corner, and the sheet is creased there (#721 F1d; before, T2 refused the corner and the closed
/// class built it, #721 F2c5).
#[test]
fn a_union_within_an_intersection_is_the_constant_twists() {
    let text = regroove(&beads(),"wheel","\
in bench {
  private k0 := point
  private k1 := point
  private k2 := point
  private k3 := point
  k0 coords(du: 0mm, dv: -5mm) bench
  k1 coords(du: 20mm, dv: -5mm) bench
  k2 coords(du: 20mm, dv: 5mm) bench
  k3 coords(du: 0mm, dv: 5mm) bench
  private kb := line(k0, k1)
  private kw := line(k1, k2)
  private kt := line(k2, k3)
  private ka := line(k3, k0)
}
construction slab := solid(std.solids.Prism(face(kb, kw, kt, ka), from: -20mm, to: 20mm))
construction trimmed := solid(wheel)
slab common trimmed
","trimmed");
    let e = fixtures::read(&text);
    let a = admit(&e,"rod").unwrap();
    let Class::ConstantTwist(found) = &a.sweeps()[0].class else { panic!("the constant twist's") };
    assert!(found.concave(),"creased at the concave corner");
    // the slab holds the wheel whole: the groove is the beads'
    near(exported(&e,"rod"),PI*25.*20.-beads_section()*20.,2e-6);
}

/// Two tori on one axis, overlapping, rolled about a skew axis through a post (`admission.rs`'s
/// roll), written as one tool and as each torus's sweep cut in turn: the second torus above the
/// first, its tube crossing the first's, so the union's profile has a crease no one tool is
/// admitted with (T2's).
fn rolled_tori(long: bool) -> String {
    let tools = format!("{}\
in std.front {{
private tc2 := point
fix((4, 2.4)) tc2
private ring2 := circle(center: tc2) hint(r: 0.4)
radius(0.4mm) ring2
}}
construction bead := solid(std.solids.Revolution(face(ring2), about: taxis))
construction lump := solid(tool)
bead union lump
",fixtures::tools::torus(2.));
    let reading = "under: spin, seen_from: observer";
    let sweeps = if long { format!("construction removal := solid({{ p | some t in (-75deg, 75deg): p in tool at({reading}) t }})\n\
        construction removal2 := solid({{ p | some t in (-75deg, 75deg): p in bead at({reading}) t }})\nremoval2 cut part\n") }
        else { format!("construction removal := solid({{ p | some t in (-75deg, 75deg): p in lump at({reading}) t }})\n") };
    format!("{tools}{}{sweeps}{}",fixtures::motions::cradle_roll(0.25,fixtures::motions::Observer::Skew),fixtures::tools::skew_post(4.))
}

/// Each torus's sweep is admitted to the generating class by its own rows.
#[test]
fn a_union_of_tori_rolled_is_two_generating_sweeps() {
    let a = admit(&fixtures::read(&rolled_tori(false)),"part").unwrap();
    assert_eq!(a.sweeps().len(),2,"one sweep a torus");
    assert!(a.sweeps().iter().all(|s| matches!(s.class,Class::Generating) && s.name == "removal" && s.contacts > 100));
}

/// The rolled union built exactly, its STL held to the field of the union's sweep: what the long
/// form cuts, its cut face named by the member that made it, by its route in the union
/// (`removal.tool.ring`, as `faces::face_paths` lists it), where the long form's is the sweep's own.
#[test]
fn the_rolled_union_cuts_what_its_members_cut_in_turn() {
    let (union,long) = (fixtures::read(&rolled_tori(false)),fixtures::read(&rolled_tori(true)));
    near(exported(&union,"part"),exported(&long,"part"),1e-7);
    let say = gcs_core::brep::sweep::Say {stage:&|_| {},mark:&|_| {}};
    let built = gcs_core::brep::export::exact(&union.sketch,fixtures::solid(&union,"part"),None,None,&say).unwrap();
    let names: Vec<(&str,&str)> = built.solid.faces.iter().map(|f| gcs_core::brep::recipe::named_parts(&f.name)).collect();
    assert!(names.contains(&("removal","tool.ring")),"{names:?}");
    let paths = gcs_core::solid::faces::face_paths(&union.sketch,fixtures::solid(&union,"part"));
    assert!(paths.iter().any(|p| p.text() == "removal.tool.ring" ),"{:?}",paths.iter().map(|p| p.text()).collect::<Vec<_>>());
}

/// The page's stepped build of `body` (`brep::export::Builder`), each stage counted, and its volume
/// against the export's.
fn stepped_as_exported(text: &str,body: &str) {
    let e = fixtures::read(text);
    let i = fixtures::solid(&e,body);
    let mut b = gcs_core::brep::export::Builder::new(&e.sketch,i).unwrap_or_else(|r| panic!("{}",r.message));
    while !b.step().unwrap_or_else(|r| panic!("{}",r.message)) {}
    let (done,total) = b.stages();
    assert_eq!(done,total,"`{body}`'s stages counted");
    let say = gcs_core::brep::sweep::Say {stage:&|_| {},mark:&|_| {}};
    let x = gcs_core::brep::export::exact(&e.sketch,i,None,None,&say).unwrap_or_else(|r| panic!("{}",r.message));
    near(b.display().unwrap().faces.volume,gcs_core::brep::props::volume(&x.solid),1e-3);
}

/// The page builds a union's cut a stage at a time from the same copy the export does: the beads'
/// rod, its sweeps sheets of the copy; the two balls' block, static there; and each as the swept
/// operand of a body built from it.
#[test]
fn the_page_builds_a_unions_cut_by_stages() {
    let balls = regroove(&text("ball.sv"),"cutter","\
in std.front {
  private centre2 := point
  centre2 coords(du: 6mm, dv: 0mm) std.front
  private bottom2 := point
  private top2 := point
  bottom2 coords(du: 6mm, dv: -2mm) std.front
  top2 coords(du: 6mm, dv: 2mm) std.front
  private diameter2 := line(bottom2, top2)
  private meridian2 := arc(center: centre2, start: bottom2, end: top2) hint(r: 2)
  radius(2mm) meridian2
}
construction small := solid(std.solids.Revolution(face(meridian2, diameter2), about: diameter2))
construction pair := solid(cutter)
small union pair
","pair");
    for (text,body) in [(beads(),"rod"),(balls,"block")] {
        stepped_as_exported(&text,body);
        stepped_as_exported(&format!("{text}made := solid({body})\n"),"made");
    }
}

/// **A tool with a concave corner is the constant twist's** (#721 F1d): `union.sv`'s two beads drawn
/// as one tool, the revolution of the outline of their two discs — the first disc's arc below where
/// they cross, the second's above — so the tool meets itself at a concave corner where the arcs do,
/// which no union distributes. Read from its B-rep, the characteristic crosses that corner where
/// both faces' points meet it (`Junction::Edge` with no fan), and the sheet is creased there
/// (`Node::crease`, `Characteristic::creases`): asked no T2 (a concave corner is the walk's to meet),
/// it is admitted by the constant twist, and the rod loses what either bead's shadow covers.
#[test]
fn a_tool_creased_at_a_concave_corner_cuts_as_its_members() {
    // where the discs' circles cross, `±√20 / 3` either side of the beads' axis line and `4 / 3` up
    let (u,v) = (20f64.sqrt()/3.,4./3.);
    let text = beads().replace("construction bead := solid(std.solids.Revolution(face(rim), about: wheel_axis))\n\
        construction bead2 := solid(std.solids.Revolution(face(rim2), about: wheel_axis))\nconstruction wheel := solid(bead)\nbead2 union wheel",
        "construction wheel := solid(std.solids.Revolution(face(below, above), about: wheel_axis))")
        .replace("  radius(tube2) rim2\n}",&format!("  radius(tube2) rim2\n  private lo := point\n  lo coords(du: reach - {u}mm, dv: {v}mm) bench\n  \
            private hi := point\n  hi coords(du: reach + {u}mm, dv: {v}mm) bench\n  \
            private below := arc(center: tc, start: lo, end: hi) hint(r: 2)\n  radius(tube) below\n  \
            private above := arc(center: tc2, start: hi, end: lo) hint(r: 1.5)\n  radius(tube2) above\n}}"));
    assert!(text.contains("face(below, above)") && text.contains("private above"));
    let e = fixtures::read(&text);
    let a = admit(&e,"rod").unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    let Class::ConstantTwist(found) = &s.class else { panic!("the constant twist's: {:?}",std::mem::discriminant(&s.class)) };
    assert!(found.screw.is_slide() && !found.creases().is_empty(),"slid, creased at the concave corner");
    near(exported(&e,"rod"),PI*25.*20.-beads_section()*20.,2e-6);
}
