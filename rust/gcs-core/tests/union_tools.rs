//! **A sweep distributes over union** (#684 C, #586, `solid::distributed`): a body cutting the
//! sweep of a union cuts each member's sweep, under the union's motion over its roll, each admitted
//! by its own class and refused by its own rows naming the member. What each cut removes is computed
//! here again from the tools and the motion alone and held to the exact solid this kernel builds,
//! whose STL is held to the field of the union's sweep; and the union cuts what its members cut
//! written one after another.
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Class,Condition,Error,Options};
use std::f64::consts::PI;

use crate::solid_sweeps_examples::text;
use crate::turns_and_slides::{exported,measure,near,refused};

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
    format!("{}construction groove := sweep(bead over t in (0deg, 360deg), under: feed)\n\
        construction groove2 := sweep(bead2 over t in (0deg, 360deg), under: feed)\ngroove cut rod\ngroove2 cut rod\n",&text[..at])
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

/// A union placed whole is distributed as its members each placed so: the wheel turned about its
/// own axis is the same wheel, and cuts the same groove.
#[test]
fn a_union_placed_whole_is_its_members_placed() {
    let text = beads().replace("(slides)","(slides, turns)").replace("construction groove := sweep(wheel ","spin := frame\nspin turns(1) wheel_axis\n\
        construction turned := wheel at(under: spin) 30deg\nconstruction groove := sweep(turned ");
    let e = fixtures::read(&text);
    assert_eq!(admit(&e,"rod").unwrap().sweeps().len(),2);
    near(exported(&e,"rod"),PI*25.*20.-beads_section()*20.,2e-6);
}

/// Two balls on one arm swung a quarter turn, half sunk in a block: each ball's sweep is its tube
/// (the canal), so the body is static material once its union is distributed, and the grooves are
/// the two half tubes with half a ball at each end (Pappus).
#[test]
fn two_balls_swung_are_two_tubes() {
    let text = text("ball.sv").replace("construction groove := sweep(cutter ","\
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
construction small := solid(face(meridian2, diameter2), about: diameter2)
construction pair := solid(cutter)
small union pair
construction groove := sweep(pair ");
    let e = fixtures::read(&text);
    let a = admit(&e,"block").unwrap();
    assert!(a.sweeps().is_empty(),"each member's sweep is a ball's tube, static material");
    let half = |r: f64,swing: f64| (PI*r*r*swing*PI/2.+4./3.*PI*r*r*r)/2.;
    near(exported(&e,"block"),24.*24.*8.-half(3.,12.)-half(2.,6.),1e-9);
}

/// A tool's member is asked its own class's rows, and refused by them naming it: the second bead
/// grown to reach the rod with the rings where its tube lies flat, in contact all round (S1).
#[test]
fn a_member_is_refused_by_its_own_row_naming_it() {
    let text = beads().replace("tc2 coords(du: reach, dv: step) bench","tc2 coords(du: reach - 1mm, dv: step) bench");
    let r = refused(admit(&fixtures::read(&text),"rod"));
    assert_eq!((r.condition,r.sweep.as_str()),(Condition::Regular,"groove"));
    assert!(r.message.starts_with("its tool's member `bead2`: "),"{}",r.message);
}

/// A union within an intersection is not distributed: refused at T1, saying so.
#[test]
fn a_union_within_an_intersection_is_refused() {
    let text = beads().replace("construction groove := sweep(wheel ","\
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
construction slab := solid(face(kb, kw, kt, ka), from: -20mm, to: 20mm)
construction trimmed := solid(wheel)
slab common trimmed
construction groove := sweep(trimmed ");
    let r = refused(admit(&fixtures::read(&text),"rod"));
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("within an intersection"),"{}",r.message);
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
construction bead := solid(face(ring2), about: taxis)
construction lump := solid(tool)
bead union lump
",fixtures::tools::torus(2.));
    let reading = "over t in (-75deg, 75deg), under: spin, seen_from: observer";
    let sweeps = if long { format!("construction removal := sweep(tool {reading})\nconstruction removal2 := sweep(bead {reading})\n\
        removal2 cut part\n") } else { format!("construction removal := sweep(lump {reading})\n") };
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
