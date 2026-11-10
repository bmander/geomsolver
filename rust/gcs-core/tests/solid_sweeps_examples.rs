//! **The solid sweeps series** (`rust/examples/solid_sweeps/`, #684, #721): one short document for
//! each class of continuous sweep this kernel builds exactly. Each is read as the class its README
//! names — admitted to it, or for a ball read as its tube — built within 0.01 mm, its STL held to
//! the material field with no disagreement, and its volume held to a figure made apart from the
//! code under test: a closed form, or arithmetic that reads the tool and the motion and nothing
//! the construction does.
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Class,Options};
use gcs_core::solid::export::Tolerance;
use std::f64::consts::{PI,TAU};

use crate::round_lowering::lens;
use crate::turns_and_slides::{exported,exported_at,exported_within,measure,near,reaches,turned_away,turned_torus,Lean};

/// The text of the example `name` of the series.
pub(crate) fn text(name: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/solid_sweeps").join(name);
    std::fs::read_to_string(path).unwrap()
}

/// The example `name` of the series, elaborated and solved.
fn example(name: &str) -> Elaborated { fixtures::read(&text(name)) }

/// The class `body`'s one sweep is admitted to.
pub(crate) fn class(e: &Elaborated,body: &str) -> Class {
    let a = admission::admit_body(&e.sketch,fixtures::solid(e,body),&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    s.class.clone()
}

/// `ball.sv`: the ball's sweep is its tube, so the block's sweep is static material and no class
/// is asked; the groove is half the tube about a quarter turn of radius 12 with half a ball at each
/// end (Pappus).
#[test]
fn the_ball_leaves_half_its_tube() {
    let e = example("ball.sv");
    assert!(gcs_core::solid::canal::is_canal(&e.sketch,fixtures::solid(&e,"groove")));
    let groove = (PI*9.*12.*PI/2.+4./3.*PI*27.)/2.;
    near(exported(&e,"block"),24.*24.*8.-groove,1e-9);
}

/// `path.sv`: a ball along a spline (#684 D) is its tube too, fitted to the path; the groove is half
/// of it, the spline's length read off its control points by Gauss–Legendre quadrature
/// (`ball_paths::bezier_length`).
#[test]
fn the_ball_along_a_spline_leaves_half_its_tube() {
    let e = example("path.sv");
    let groove = fixtures::solid(&e,"groove");
    assert!(gcs_core::solid::canal::built(&e.sketch,groove).unwrap().unwrap().fitted(),"its tube fitted to its path");
    let length = crate::ball_paths::bezier_length([[0.,0.],[10.,0.],[20.,20.],[30.,20.]]);
    near(crate::ball_paths::held(&e,"block").0,42.*32.*8.-crate::canal::tube(3.,length)/2.,1e-9);
}

/// `slide.sv`: a slide, its groove the lens the rod's disc and the tube's share, carried the rod's
/// 20 mm.
#[test]
fn the_slid_wheel_leaves_its_tubes_lens() {
    let e = example("slide.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide(),"read as a slide");
    near(exported(&e,"rod"),PI*25.*20.-lens(5.,2.,5.5)*20.,1e-6);
}

/// `turn.sv`: a turn, its groove the leaning torus's shadow in a half-plane through the axis,
/// turned through the block's 60 degrees (Pappus, the shadow read from the torus's centre circle).
#[test]
fn the_turned_cutter_leaves_its_shadow_turned() {
    let e = example("turn.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn(),"read as a turn");
    near(exported(&e,"block"),PI/3.*(144.-64.)-turned_away(&turned_torus(Lean::Across(25.))),2e-5);
}

/// `rocked.sv`: `turn.sv`'s cutter rocked out and back by a law (#684 L): a sweep is its path's, so
/// the drive is read as its turn over the law's reach, from `from` to `to`, and the groove is
/// `turn.sv`'s, held to the same figure.
#[test]
fn the_rocked_cutter_leaves_the_turns_groove() {
    let e = example("rocked.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn(),"read as a turn");
    near(exported(&e,"block"),PI/3.*(144.-64.)-turned_away(&turned_torus(Lean::Across(25.))),2e-5);
}

/// `screw.sv`: a screw, carried past both ends of the rod, so the groove's section square to the
/// axis is one region at every height, turned. A point of it is cut where its helix back through
/// the roll passes within the tube's radius of the wheel's centre circle (radius 6 about
/// (11.5, 0, 0), square to the wheel's axis, which leans from y away from z by the helix's angle at
/// 5.5 from the rod's axis), read over the turns that bring it to the wheel's heights a degree
/// apart (`reaches`), in polar rows about the rod's axis. The groove takes that area from each of the rod's 20 mm.
#[test]
fn the_screwed_wheel_leaves_its_section_at_every_height() {
    let e = example("screw.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a screw");
    let lead = 30.;
    let (sh,ch) = (TAU*5.5/lead).atan().sin_cos();
    // the distance of the point p carried back along the screw by `t` from the wheel's centre circle
    let back = |p: [f64;3],t: f64| -> f64 {
        let (s,c) = t.sin_cos();
        let w = [c*p[0]+s*p[1]-11.5,-s*p[0]+c*p[1],p[2]-lead*t/TAU];
        let axial = w[1]*ch-w[2]*sh;
        let radial = [w[0],w[1]-axial*ch,w[2]+axial*sh];
        ((radial[0]*radial[0]+radial[1]*radial[1]+radial[2]*radial[2]).sqrt()-6.).hypot(axial)
    };
    // the turns at which a point at height 10 stands within the wheel's heights, 8 either side of
    // its middle
    let turns = [2.*TAU/lead,18.*TAU/lead];
    let steps = ((turns[1]-turns[0]).to_degrees().ceil()) as usize;
    let hit = |a: f64,r: f64| { let p = [r*a.cos(),r*a.sin(),10.]; reaches(&|t| back(p,t),turns,steps,2.) };
    let section = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    near(exported(&e,"rod"),PI*25.*20.-section*20.,1e-5);
}

/// `vthread.sv`: a screw whose wheel has a sharp rim (#684 E), so the characteristic runs from one
/// flank round the rim's fan to the other; the groove's section is one region at every height,
/// turned, as `screw.sv`'s is. A point of it is cut where its helix back through the roll passes
/// into the wheel: within the V of its rim, whose tip runs 10 from the wheel's axis (through
/// (14, 0, 0), leaning from y away from z by the helix's angle at 4.5 from the rod's axis) and
/// whose flanks lie 30° either side of the wheel's plane — what the wheel's bore would add never
/// comes within 9 of the rod's points. Read as `screw.sv`'s is, by the signed distance from the
/// nearer flank, over the turns that bring the point to the wheel's heights.
#[test]
fn the_v_wheel_leaves_its_section_round_the_rim() {
    let e = example("vthread.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a screw");
    assert_eq!(found.fans(),["the corner of `vwheel.upper` and `vwheel.lower`"]);
    let lead = 30.;
    let (sh,ch) = (TAU*4.5/lead).atan().sin_cos();
    let (sf,cf) = (PI/6.).sin_cos();
    // how far the point p carried back along the screw by `t` lies outside the V
    let back = |p: [f64;3],t: f64| -> f64 {
        let (s,c) = t.sin_cos();
        let w = [c*p[0]+s*p[1]-14.,-s*p[0]+c*p[1],p[2]-lead*t/TAU];
        let axial = w[1]*ch-w[2]*sh;
        let radial = [w[0],w[1]-axial*ch,w[2]+axial*sh];
        let rho = (radial[0]*radial[0]+radial[1]*radial[1]+radial[2]*radial[2]).sqrt();
        axial.abs()*cf+(rho-10.)*sf
    };
    // the turns at which a point at height 10 stands within the wheel's heights, 12 either side
    // of its middle
    let turns = [-2.*TAU/lead,22.*TAU/lead];
    let steps = ((turns[1]-turns[0]).to_degrees().ceil()) as usize;
    let hit = |a: f64,r: f64| { let p = [r*a.cos(),r*a.sin(),10.]; reaches(&|t| back(p,t),turns,steps,0.) };
    let section = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    assert!(section > 0.5,"the groove's section is {section}");
    near(exported(&e,"rod"),PI*25.*20.-section*20.,1e-5);
}

/// `whole_turn.sv`: a turn carried round a whole turn (#684 N), its wheel's rim sharp, so the
/// characteristic runs from one flank round the rim's fan to the other, and its section is revolved
/// all the way round, closing on itself. The groove is the wheel's shadow in a half-plane through
/// the rod's axis, turned once round: a circle about the axis is cut where it passes into the V of
/// the rim, whose tip runs 10 from the wheel's axis (through (14, 0, 10), leaning from z toward y
/// by 20°) and whose flanks lie 30° either side of the wheel's plane — read by the signed distance
/// from the nearer flank round the circle; what the wheel's bore would add never comes within 6 of
/// the rod's axis. Pappus over that shadow.
#[test]
fn the_wheel_turned_once_leaves_its_shadow_all_round() {
    let e = example("whole_turn.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn() && found.whole,"read as a whole turn");
    assert_eq!(found.fans(),["the corner of `vwheel.upper` and `vwheel.lower`"]);
    let (sl,cl) = (20f64).to_radians().sin_cos();
    let (sf,cf) = (PI/6.).sin_cos();
    // how far q lies outside the V
    let outside = |q: [f64;3]| -> f64 {
        let w = [q[0]-14.,q[1],q[2]-10.];
        let along = w[1]*sl+w[2]*cl;
        let rho = (w[0]*w[0]+(w[1]-along*sl).powi(2)+(w[2]-along*cl).powi(2)).sqrt();
        along.abs()*cf+(rho-10.)*sf
    };
    let hit = |z: f64,r: f64| reaches(&|a| outside([r*a.cos(),r*a.sin(),z]),[-PI,PI],90,0.);
    let shadow = measure([6.,14.],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    assert!(shadow > 0.5,"the groove's shadow is {shadow}");
    near(exported(&e,"rod"),PI*25.*20.-TAU*shadow,1e-5);
}

/// `rack.sv`: the generating class, its tooth space the arithmetic `rack_cut` reads it by (an arc
/// of each circle about the blank's centre at each roll, joined over the roll), through the
/// blank's 6 mm.
#[test]
fn the_rack_leaves_one_tooth_space() {
    let e = example("rack.sv");
    assert!(matches!(class(&e,"gear"),Class::Generating),"the generating class");
    let removed = PI*22.*22.*6.-exported(&e,"gear");
    let want = 6.*crate::rack_cut::space_area(&crate::rack_cut::TOOTH,400,4000);
    assert!((removed-want).abs() < 0.03,"removed {removed:.5} mm³, the space being {want:.5}");
}

/// `ring_gear.sv` (#818): `rack.sv`'s gear, its space cut at 20 teeth, cut from a ring as a body
/// built of sweeps: the gear's space in the generating class, the ring composed by the kernel's
/// Boolean. The ring keeps its annulus outside the gear's rim and, inside it, 20 teeth each its
/// 6 mm thickness times the area the rack tooth removes from the annulus between the ring's bore
/// and the rim, by `rack_cut`'s arithmetic. Built once: the page's stepped builder is held to the
/// same build (`object_builder::builds_as_exported`).
#[test]
fn the_ring_gear_keeps_the_gears_spaces_as_its_teeth() {
    let e = example("ring_gear.sv");
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"gear"),&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    assert!(a.sweeps().iter().all(|s| matches!(s.class,Class::Generating)),"the generating class");
    let exact = exported_within(&e,"ring",Some(Tolerance::new(0.01).unwrap()));
    let tooth = 6.*crate::rack_cut::rack_space(&crate::rack_cut::TOOTH,20.,[19.,22.],[-PI/3.,PI/3.],400,4000);
    let want = PI*(30.*30.-22.*22.)*6.+20.*tooth;
    let kept = gcs_core::brep::props::volume(&exact.solid);
    // the arithmetic integrates by midpoints over 400 radii; the two agree within 2e-3
    assert!((kept-want).abs() < 0.01,"kept {kept:.5} mm³, its teeth being {want:.5}");
    crate::object_builder::builds_as_exported("ring_gear.sv",&e.sketch,fixtures::solid(&e,"ring"),&exact);
}

/// `bar_end.sv`: the generating class with a prism's cap in the blank (#684 F), the bar sectioned
/// across the disc's height; the space each height's slice of the bar rolled on the circle of 16,
/// by `rack_cut`'s arithmetic (`prism_caps::bar_end_space`), through the disc's 6 mm.
#[test]
fn the_bars_end_leaves_its_space() {
    let e = example("bar_end.sv");
    assert!(matches!(class(&e,"part"),Class::Capped(_)),"the generating class, its cap sectioned across");
    near(exported(&e,"part"),PI*22.*22.*6.-crate::prism_caps::bar_end_space(),1e-6);
}

/// `next_tooth.sv`: a placed prism's cap in the blank (#684 AC), `bar_end.sv`'s bar placed a quarter
/// turn of the roll along the rack and rolled a quarter turn earlier: it cuts the bar's space
/// turned a quarter turn round the disc, so the same volume, and the field, which reads the placed
/// tool apart from its sections, agrees with where the cut is.
#[test]
fn the_next_tooth_leaves_the_bars_space_turned() {
    let e = example("next_tooth.sv");
    let Class::Capped(capped) = class(&e,"part") else { panic!("the generating class, its cap sectioned across") };
    assert!(capped.across[2].abs() > 0.99,"sectioned across the disc's height, as the bar is: {:?}",capped.across);
    near(exported(&e,"part"),PI*22.*22.*6.-crate::prism_caps::bar_end_space(),1e-6);
}

/// `helical.sv`: the generating class with a drafted prism (#684 G), the rack's tooth leaned 15°
/// across the blank and drafted 2°, so the motion keeps no plane of it and no planar class asks
/// it. Its space is each section square to the blank's axis cut by the tooth's section there, a
/// quadrilateral read from the tooth's four drafted planes and swept by `rack.sv`'s arithmetic
/// (`drafted_tools::space_volume`), through the blank's 6 mm.
#[test]
fn the_drafted_helical_tooth_leaves_its_space() {
    let e = example("helical.sv");
    let space = fixtures::solid(&e,"space");
    assert!(!gcs_core::solid::planar_subtraction::asks(&e.sketch,space) && !gcs_core::solid::planar_class::asks(&e.sketch,space));
    assert!(matches!(class(&e,"gear"),Class::Generating),"the generating class");
    let removed = PI*22.*22.*6.-exported(&e,"gear");
    let want = crate::drafted_tools::space_volume(15.,2.,[2.,8.]);
    assert!((removed-want).abs() < 2e-4*want,"removed {removed:.5} mm³, the space being {want:.5}");
}

/// `parted.sv`: the generating class with a prism drafted both ways from its plane (#684 P), the
/// blank's middle moved to the helical tooth's plane and the tooth drafted 2° either side of it, each side
/// folding there in a convex crease. Its space is each section square to the blank's axis cut by
/// the tooth's section there, its eight half-planes meeting in it, swept by `rack.sv`'s arithmetic
/// (`drafted_tools::parted_volume`), through the blank's 6 mm: more than the tooth drafted one way
/// from the front cuts, less than the undrafted tooth.
#[test]
fn the_tooth_drafted_both_ways_leaves_its_space() {
    let e = example("parted.sv");
    let space = fixtures::solid(&e,"space");
    assert!(!gcs_core::solid::planar_subtraction::asks(&e.sketch,space) && !gcs_core::solid::planar_class::asks(&e.sketch,space));
    assert!(matches!(class(&e,"gear"),Class::Generating),"the generating class");
    let removed = PI*22.*22.*6.-exported(&e,"gear");
    let want = crate::drafted_tools::parted_volume(15.,2.,0.,[-3.,3.]);
    eprintln!("removed {removed:.5} mm³, the space being {want:.5}");
    assert!(want > crate::drafted_tools::space_volume(15.,2.,[2.,8.])+5. && want < crate::drafted_tools::space_volume(15.,0.,[2.,8.]));
    assert!((removed-want).abs() < 2e-4*want,"removed {removed:.5} mm³, the space being {want:.5}");
}

/// `conjugate.sv`: the planar subtraction class, its wheel the disc's area kept outside the plus
/// at every pose of the period (`planar_subtraction`'s own arithmetic), through the 6 mm slab.
#[test]
fn the_plus_leaves_its_conjugate_wheel() {
    let e = example("conjugate.sv");
    assert!(matches!(class(&e,"wheel"),Class::Subtraction(_)),"the planar subtraction class");
    near(exported(&e,"wheel"),6.*crate::planar_subtraction::kept_area(90),2e-4);
}

/// `union.sv`: a union swept by a slide is swept as its set (#721 F1d), the beads one solid read by
/// its B-rep, its characteristic ended at the concave corner where they meet; admitted to the
/// constant-twist class as the union's sweep; the groove what of the rod's disc either bead's shadow
/// covers, in rows across the rod (`union_tools::beads_section`), through the rod's 20 mm.
#[test]
fn the_beads_leave_what_either_shadow_covers() {
    let e = example("union.sv");
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"rod"),&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("the union swept whole") };
    let Class::ConstantTwist(found) = &s.class else { panic!("the union's sweep is a slide's") };
    assert!(s.name == "groove" && found.screw.is_slide(),"read as the union's slide");
    near(exported(&e,"rod"),PI*25.*20.-crate::union_tools::beads_section()*20.,2e-6);
}

/// `square_thread.sv`: a prism under a screw (#684 M), a bar standing across the rod, its tip at
/// radius 4. The characteristic runs along the tip's middle and out along the two corners where it
/// meets the flanks, round their fans. A point of the bar at radius r turns `acos(4 / r)` at most
/// from the tip's plane, so the groove at radius r is the bar's width and the lead's share of
/// twice that turn, `2 + 6 acos(4 / r) / π`, along the axis; over the rod's 20 mm, the rod at
/// radius r loses that width over each 6 mm lead, all round. In closed form, with
/// `∫ r acos(a / r) dr = r² acos(a / r) / 2 − a √(r² − a²) / 2`.
#[test]
fn the_square_tool_leaves_a_groove_widening_outward() {
    let e = example("square_thread.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a screw");
    assert_eq!(found.fans(),["the corner of `bar.nose` and `bar.lower`","the corner of `bar.nose` and `bar.upper`"]);
    let (r0,r,w,lead,l) = (4_f64,5_f64,2.,6.,20.);
    let turned = r*r/2.*(r0/r).acos()-r0/2.*(r*r-r0*r0).sqrt();
    let groove = TAU*l/lead*(w*(r*r-r0*r0)/2.+lead/PI*turned);
    near(exported(&e,"rod"),PI*r*r*l-groove,1e-5);
}

/// `flat_end.sv` (#684 AE): the bar's flat end faces down toward the axis, so the turn carries its
/// contact line round into the floor, its inner side's into the wall, and the rim between them
/// joins the two round its fan. The block loses what the bar's shadow covers, read slice by slice
/// from the bar's half-spaces (`prism_twists::Bar::removed`, the same bar as `flat_end(20, 5)`),
/// within 2e-5 at 0.001 mm (at 0.01 the fitted sheet's bias over the floor is 9e-5 of the part).
#[test]
fn the_flat_end_cuts_a_floor() {
    let e = example("flat_end.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a turn");
    assert_eq!(found.fans(),["the rim where `bar.inner_side` meets `bar.far`"]);
    near(exported_at(&e,"block",0.001),PI/3.*80.-crate::prism_twists::flat_end(20.,5.).removed(),2e-5);
}

/// `short_screw.sv` (#721 F4): the bar passes the blank in 280°, though its section
/// needs 323° of delays. Built over its actual poses at 0.01 mm, to an independent volume
/// from the bar's half-spaces, with zero STL/field disagreement and the same stepped build.
#[test]
fn the_short_screw_carries_its_characteristic_over_the_stated_roll() {
    let e = example("short_screw.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    crate::prism_twists::section_exceeds_roll(&found,280_f64.to_radians());
    // The native path still asks the temporal-contact chart; a screw is refused by name.
    use gcs_core::brep::swept::generating::{Form,Uncharted};
    use gcs_core::brep::trajectory::Trajectory;
    assert!(matches!(Form::of(&Trajectory::Twist(found.screw)),Err(Uncharted::Motion)));
    let bar = crate::prism_twists::Bar {lead:3.,..crate::prism_twists::flat_end(40.,5.)};
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),PI/3.*80.-bar.lost(),2e-5);
    crate::object_builder::builds_as_exported("short_screw.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// `square_end.sv` (#721 G1): `flat_end.sv`'s bar, its section turned 30° so its corner stands
/// nearest the axis: the corner's point of contact runs out along it to the bar's far end, a vertex,
/// and the walk goes round the vertex onto the end's rim, the sheet creased along the vertex's
/// circle. The block loses what the bar's shadow covers, read slice by slice from the bar's
/// half-spaces (`prism_twists::Bar::removed`, the same bar as `flat_end(20, 30)`), within 2e-5 at
/// 0.01 mm, its STL agreeing with the field, the page's stepped builder held to the same build.
#[test]
fn the_square_ended_bar_goes_round_its_vertex() {
    let e = example("square_end.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a turn");
    assert_eq!(found.vertices(),["the corner of `bar.upper` and `bar.inner_side`"]);
    assert_eq!(found.fans(),["the corner of `bar.upper` and `bar.inner_side`","the rim where `bar.inner_side` meets `bar.far`"]);
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),PI/3.*80.-crate::prism_twists::flat_end(20.,30.).removed(),2e-5);
    crate::object_builder::builds_as_exported("square_end.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// `capped_thread.sv` (#721 G1): `square_thread.sv`'s bar standing only 2.5 either side of its
/// plane, its ends in the rod: the corners where its tip meets its flanks carry the contact out to
/// the ends, vertices, and the walk goes round each onto an end's rim, the sheet creased along both
/// vertices' helices. The rod loses each circle's angle the bar covers over the rolls reaching its
/// height, the bar's half-angle at radius `ρ` the lesser of its tip's `acos(4 / ρ)` and its end's
/// `asin(2.5 / ρ)` (`closed_class::screwed_partway_away`, screwed past the rod's top), within 2e-5
/// at 0.01 mm, the fitted sheets within a micrometre of the contacts; its STL agrees with the
/// field, and the page's stepped builder is held to the same build. (The closed class built it
/// before, in 25 s.)
#[test]
fn the_capped_thread_goes_round_its_bars_vertices() {
    let e = example("capped_thread.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a screw");
    assert_eq!(found.vertices(),["the rim where `bar.lower` meets `bar.near`","the corner of `bar.nose` and `bar.upper`"]);
    let exact = exported_within(&e,"rod",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),PI*25.*20.-crate::closed_class::screwed_partway_away(21.,2.5),2e-5);
    crate::object_builder::builds_as_exported("capped_thread.sv",&e.sketch,fixtures::solid(&e,"rod"),&exact);
}

/// `creased.sv` (#684 AG): the bar's inner side's halves each touch along a line crossing its
/// rulings, and the crease where they meet joins the two round its fan. The block loses what the
/// bar's shadow covers, read slice by slice from its ten half-planes (`prism_twists::Bar::removed`,
/// the same bar as `parted(20, 2)`), within 2e-5 at 0.001 mm.
#[test]
fn the_creased_bar_crosses_its_crease() {
    let e = example("creased.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a turn");
    assert_eq!(found.fans(),["the crease across `bar.inner_side`"]);
    near(exported_at(&e,"block",0.001),PI/3.*80.-crate::prism_twists::parted(20.,2.).removed(),2e-5);
}

/// `facing.sv` (#684 AF): the bar's inner side faces the axis squarely, so each of its rulings'
/// contact is the same all along it, and it touches along the one ruling in the plane through the
/// axis, all along it; the turn carries that line round into the wall, a cone `reach + z tan(tilt)`
/// from the axis at height z, and the block loses, within its 60°, `π/6 ∫ (12² − r²) dz` over its
/// heights, in closed form.
#[test]
fn the_facing_side_cuts_a_cone() {
    let e = example("facing.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert_eq!(found.screw.kind(),"a turn");
    assert_eq!(found.lined(),["`bar.inner_side`"]);
    assert!(found.fans().is_empty(),"{:?}",found.fans());
    let t = 20f64.to_radians().tan();
    near(exported(&e,"block"),PI/3.*80.-PI/6.*(88.-2./3.*t*t),1e-5);
}

/// `slot.sv` (#684 O): the end mill's flat bottom, which the slide keeps in its own plane, touches
/// all over and sweeps the slot's floor, the strip its diameter square to the slide carries; its
/// side's two lines square to the slide are the walls, each meeting the floor in a sharp edge where
/// the walk runs onto and off the bottom's line. Read from its B-rep (#721 F1c), the mill is a
/// cylinder standing along the normal of the planes the slide keeps, so the planar subtraction class
/// reads it first, the region its section sweeps carried through the slab its bottom bounds. The
/// block loses `4 × 3` along its 20, in closed form.
#[test]
fn the_end_mill_cuts_a_slot() {
    let e = example("slot.sv");
    assert!(matches!(class(&e,"block"),Class::Subtraction(_)),"the planar subtraction class");
    near(exported(&e,"block"),2000.-4.*3.*20.,1e-6);
}

/// `vgroove.sv` (#684 V): the V wheel slid square to its axis, each flank touching along its
/// meridian square to the slide, the two meeting the rim at one point where every normal of the
/// rim's fan is in contact: the walk steps from the one flank straight onto the other and the
/// sheet is creased along that point's path, the groove's root. The walls are planes, and the
/// block loses the V's section, `depth² tan(flank)`, along its 20, in closed form.
#[test]
fn the_v_wheel_slid_square_cuts_a_v_groove() {
    let e = example("vgroove.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide(),"read as a slide");
    assert_eq!(found.sharp(),["the corner of `vwheel.upper` and `vwheel.lower`"]);
    assert_eq!(found.creases().len(),1);
    assert!(found.fans().is_empty(),"{:?}",found.fans());
    near(exported(&e,"block"),2000.-4.*(PI/6.).tan()*20.,1e-6);
}

/// `punch.sv` (#684 W): a drum pushed edge first wholly through a plate, its characteristic closed
/// on itself: along its side's line one side of the push, across a flat end, which the push keeps in
/// its own plane, back along the other side's line and across the other end, a rectangle carried up
/// as a tube, creased at its four corners and begun at one. The plate loses the drum's shadow along
/// the push, `drum × 2 radius`, through its thickness, in closed form.
#[test]
fn the_drum_punches_its_shadow_through_a_plate() {
    let e = example("punch.sv");
    // (read from its B-rep, #721 F1c, the drum stands along the normal of the planes the push
    // keeps: the planar subtraction class reads its shadow first)
    assert!(matches!(class(&e,"plate"),Class::Subtraction(_)),"the planar subtraction class");
    near(exported(&e,"plate"),20.*10.*4.-8.*4.*4.,1e-6);
}

/// `drill.sv` (#684 X): a drill pushed down along its own axis wholly through a plate. Every ring of
/// it meets the slide at one angle all round, so it touches nowhere or all round, and its ring of
/// greatest radius sweeps the cylinder of that radius; its shadow along the slide is its whole disc,
/// so the sweep within the plate is that cylinder's, written exactly: the plate loses `π 2² 4`,
/// whatever the drill's point.
#[test]
fn the_drill_bores_its_radius_through_a_plate() {
    let e = example("drill.sv");
    let Class::ConstantTwist(found) = class(&e,"plate") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide() && found.closed,"a slide, its characteristic closed on itself");
    assert_eq!(found.rounds(),["the ring of the tool's greatest radius"]);
    assert!(found.creases().is_empty() && found.fans().is_empty() && found.flats().is_empty());
    let (_,_,radius) = found.cylinder().expect("its sheet the cylinder its ring sweeps");
    assert_eq!(radius,2.);
    near(exported(&e,"plate"),20.*10.*4.-PI*4.*4.,1e-12);
}

/// `spoke.sv` (#684 X): a pin lying along a radius of the turn, its axis meeting the block's at the
/// centre, swung across the curved block. Its rings touch at their two points in the plane of the
/// two axes, its side along its top and bottom lines, the slot's roof and floor; on its end's rim
/// the ring whose normals pass through the centre touches all round and sweeps a zone of the sphere
/// about it, the slot's end wall, the walk turning onto it from the fan beside the side, where the
/// fan stands at the top line's end, half round, and back down the fan onto the bottom line. The
/// branch beyond it, across the end face, lies within the zone. The slot is the band |z| ≤ 0.5 from
/// the block's inner wall at 8 out to the sphere of radius √(11² + 0.5²), turned through 60°:
/// `π/6 (57.25 − 1/12)` in closed form, within 1e-5 (2.9e-6, alike at 0.001 mm: the fit's).
#[test]
fn the_spoke_swings_a_slot_ended_by_a_zone_of_a_sphere() {
    let e = example("spoke.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn() && !found.closed,"a turn, its characteristic open");
    assert_eq!(found.rounds(),["a ring of the corner of `pin.tip` and `pin.side` in contact all round"]);
    assert_eq!(found.sharp(),["the corner of `pin.tip` and `pin.side`"]);
    near(exported(&e,"block"),PI/3.*80.-PI/6.*(57.25-1./12.),1e-5);
}

/// The greatest of `f` over `[a, b]`: read at 200 points, the best of them refined by Brent's search
/// between its neighbours (`f` unimodal there).
fn greatest(f: &dyn Fn(f64) -> f64,a: f64,b: f64) -> f64 {
    let n = 200;
    let x = |k: usize| a+(b-a)*k as f64/n as f64;
    let k = (0..=n).map(|k| (k,f(x(k)))).max_by(|p,q| p.1.total_cmp(&q.1)).unwrap().0;
    let (lo,hi) = (x(k.saturating_sub(1)),x((k+1).min(n)));
    -gcs_core::roots::brent(&|t| -f(t),lo,hi,1e-15,200,|_,_| false).value
}

/// What a cone to a point of half angle `half` takes from a rod of radius `rod` and length `length`
/// screwed about the rod's axis `lead` a turn, its apex `apex` from the rod's axis and its own
/// axis square to the rod's through it, passing wholly along the rod (`chased_point.sv`). Read in
/// helical coordinates, worked from the cone alone: a point at radius `r` is removed where its helix
/// meets the cone. On the cylinder of radius `r` a point of the cone at angle `ψ` from the plane of
/// the two axes stands `a = r cos ψ − apex` along the cone's axis and `r sin ψ` across, so the cone
/// holds heights within `w(ψ) = √((a tan half)² − (r sin ψ)²)` of the plane square to the rod
/// through the cone's axis; a helix `z − h ψ` (`h` the lead over a turn) meets that slice over a
/// spread of `2 max (w + h ψ)`, which is `2 max (w + h ψ) / h` of angle round the rod, at most a whole
/// turn. The rod loses that angle at each radius, over its length.
fn chased(rod: f64,length: f64,apex: f64,half: f64,lead: f64) -> f64 {
    let h = lead/TAU;
    let spread = |r: f64| {
        let w = |psi: f64| { let a = r*psi.cos()-apex; let w2 = (a*half.tan()).powi(2)-(r*psi.sin()).powi(2);
            if a >= 0. && w2 >= 0. { w2.sqrt()+h*psi } else { f64::NEG_INFINITY } };
        // (none where the slice holds no point the scan reads, at the apex's own radius)
        (2.*greatest(&w,0.,(apex/r).acos())/h).clamp(0.,TAU)
    };
    // (r = apex + s², the spread rising as a root from the apex's radius)
    length*crate::swept_slid::gauss(&|s: f64| { let r = apex+s*s; spread(r)*r*2.*s },0.,(rod-apex).sqrt(),60)
}

/// What a cone to a point of half angle `half` turned about z takes from a ring sector of `sector`
/// radians standing from `z = -top` to `top` (`engraved_arc.sv`), passing wholly across it: its apex
/// at `tip`, its axis `axis`, and `across` square to it. Each circle about z that meets the cone
/// loses the sector's angle, so the sector loses `sector/2 ∫ (ρ_max² − ρ_min²) dz` over the heights
/// the cone reaches within it, each height's slice of the cone convex and clear of the axis, so its
/// nearest and farthest radii on its rim: each ruling, `tip + t (axis + tan half (cos θ across + sin
/// θ (axis × across)))`, meets the height `z` once.
fn engraved(tip: [f64;3],axis: [f64;3],across: [f64;3],half: f64,top: f64,sector: f64) -> f64 {
    let other = gcs_core::space::cross(axis,across);
    let rim = |z: f64,theta: f64| {
        let d: [f64;3] = std::array::from_fn(|k| axis[k]+half.tan()*(theta.cos()*across[k]+theta.sin()*other[k]));
        let t = (z-tip[2])/d[2];
        (tip[0]+t*d[0]).hypot(tip[1]+t*d[1])
    };
    let width = |z: f64| greatest(&|th| rim(z,th).powi(2),0.,TAU)+greatest(&|th| -rim(z,th).powi(2),0.,TAU);
    sector/2.*crate::swept_slid::gauss(&width,tip[2],top,8)
}

/// `chased_point.sv` (#721 G5): a V-bit lying across a rod, its 60° point a millimetre into it,
/// screwed along it 6 mm a turn as a thread is chased with a pointed tool. The walk runs in along
/// one root of the cone's rings, turns at its apex, where every normal of the cone round it is the
/// bit's, and runs back out along the other: one apex, the sheet creased along its helix. The rod
/// loses `chased` (`π 5² 20` less, worked in helical coordinates from the cone alone), built within
/// 0.01 mm, its STL agreeing with the field, the page's stepped builder held to the same build.
#[test]
fn the_pointed_tool_chases_a_thread_its_root_the_apexs_helix() {
    let e = example("chased_point.sv");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert!(!found.screw.is_slide() && !found.screw.is_turn(),"read as a screw");
    assert_eq!(found.apexes(),["`vbit.cone`"]);
    assert!(found.fans().is_empty() && found.flats().is_empty() && !found.concave());
    let exact = exported_within(&e,"rod",Some(Tolerance::new(0.01).unwrap()));
    let removed = chased(5.,20.,4.,PI/6.,6.);
    assert!(removed > 50. && removed < 60.,"the figure reads a groove: {removed}");
    near(gcs_core::brep::props::volume(&exact.solid),PI*25.*20.-removed,1e-6);
    crate::object_builder::builds_as_exported("chased_point.sv",&e.sketch,fixtures::solid(&e,"rod"),&exact);
}

/// `engraved_arc.sv` (#721 G5): a V-bit leaning 25° across a turn, its 60° point half a millimetre
/// into a curved block's top, turned wholly across it: the walk turns at the apex, and the wall is
/// two surfaces of revolution meeting along the circle the apex turns on. The block loses its 60°
/// of the bit's shadow turned (`engraved`, worked from the cone alone).
#[test]
fn the_pointed_tool_engraves_an_arc_its_root_the_apexs_circle() {
    let e = example("engraved_arc.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn(),"read as a turn");
    assert_eq!(found.apexes(),["`bit.cone`"]);
    let lean = 25_f64.to_radians();
    let removed = engraved([10.,0.,0.5],[0.,lean.sin(),lean.cos()],[1.,0.,0.],PI/6.,1.,PI/3.);
    near(exported(&e,"block"),PI/3.*(144.-64.)-removed,1e-6);
}

/// `raked_vee.sv` (#721 G5): a V-bit leaning 20° along its slide, its 60° point a millimetre into a
/// block, slid wholly along it: the walk runs up one ruling square to the slide, turns at the apex
/// and runs down the other, and the walls are two planes, each holding the slide and touching the
/// cone, so leaning `asin(sin 30° / cos 20°)` from upright. The block loses that V's section along
/// its 20, exactly.
#[test]
fn the_raked_pointed_tool_cuts_a_vee_its_root_the_apexs_path() {
    let e = example("raked_vee.sv");
    let Class::ConstantTwist(found) = class(&e,"block") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide(),"read as a slide");
    assert_eq!(found.apexes(),["`bit.cone`"]);
    let flank = (0.5/20_f64.to_radians().cos()).asin();
    near(exported(&e,"block"),2000.-20.*flank.tan(),1e-9);
}

/// `plunge.sv` (the closed class, #721 X1): a bar of square section pushed aslant 8 into the plate,
/// stopping inside it. The constant-twist class refuses it (its silhouette turns at a vertex onto a
/// cap, and it ends in the blank); the closed class reads its six planes and twelve lines and cuts
/// it from the plate on one arrangement. The plate loses each `y` section's rectangle swept along
/// the push and clipped to the plate (`closed_class::plunged_away`), to 1e-9.
#[test]
fn the_bar_plunged_aslant_stops_inside_the_plate() {
    let e = example("plunge.sv");
    let Class::Closed(found) = class(&e,"plate") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_slide(),"a slide");
    assert_eq!((found.tool.faces.len(),found.tool.edges.len()),(6,12));
    near(exported(&e,"plate"),800.-crate::closed_class::plunged_away(8.),1e-9);
}

/// `stopped_punch.sv` (the closed class, X1 with a curved tool): `punch.sv`'s drum pushed aslant 8
/// into the plate, stopping inside it. The constant-twist class refuses it (as it refuses the drum
/// pushed aslant wholly through, and it ends in the blank); the closed class reads its cylinder, its
/// two flat ends and its rims under the slide and cuts it from the plate on one arrangement, the
/// side's rulings swept into their tangent planes and the rims into oblique cylinders. The plate
/// loses each `y` section's rectangle swept along the push and clipped to it
/// (`closed_class::drum_plunged_away`), to 1e-9.
#[test]
fn the_drum_plunged_aslant_stops_inside_the_plate() {
    let e = example("stopped_punch.sv");
    let Class::Closed(found) = class(&e,"plate") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_slide(),"a slide");
    near(exported(&e,"plate"),800.-crate::closed_class::drum_plunged_away(8.),1e-9);
}

/// `stopped_drum.sv` (the closed class, X1 with a curved tool under a turn): a drum about an axis
/// along the turn's, turned from −140° to 30°, halfway into the sector. The constant-twist class
/// refuses it (it ends in the blank); the closed class cuts it from the ring on one arrangement, its
/// side's generators turned into cylinders about the axis. The ring loses each circle's covered
/// angle within the sector (`closed_class::drum_turned_partway_away`), to 1e-9.
#[test]
fn the_drum_turned_partway_stops_inside_the_ring() {
    let e = example("stopped_drum.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_turn(),"a turn");
    near(exported(&e,"block"),PI/3.*80.-crate::closed_class::drum_turned_partway_away(),1e-9);
}

/// `helical_groove.sv` (the closed class, a round tool screwed: helical milling): the drum of
/// `stopped_drum.sv` screwed half a millimetre a turn from −140° to 140°, wholly across the sector.
/// The constant-twist class refuses it (its flat ends keep no plane, S2); the closed class cuts it
/// from the ring on one arrangement, its side's generators screwed into cylinders about the axis and
/// each rim's disc into the ramp it sweeps. The ring loses `π/3 (20 + k π)`, `k = 0.5/2π`, to 1e-8:
/// the sector's planes through the axis cut the rims' ramps in traced closed-form points.
/// Built once, within 0.01 mm: the page's stepped builder is held to the same build
/// (`object_builder::builds_as_exported`, within the export's gross bar), which a test of its own
/// once built again.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: a drum's screwed rims through a sector")]
fn the_drum_screwed_mills_a_helical_groove() {
    let e = example("helical_groove.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert_eq!(found.kind(),"a screw");
    let k = 0.5/std::f64::consts::TAU;
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),PI/3.*80.-PI/3.*(20.+k*PI),1e-8);
    crate::object_builder::builds_as_exported("helical_groove.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// `stopped_bullnose.sv` (the closed class, a torus under a turn): `stopped_drum.sv`'s drum with its
/// bottom corner rounded to 0.3, turned from −140° to 30°, halfway into the sector. Its round, a
/// torus about an axis along the turn's, touches round its meridians in the plane through both
/// axes, turned into tori about the turn's, and round its bottom ring, kept in its plane. Each level
/// of the mill is a disc of radius `R(z)`, 10 out, so each circle about the axis loses the angle
/// the disc spans from the sector's edge to 30° past: `π d/3 ∫R dz + π/2 ∫R² dz` of the ring, to 1e-9.
/// Built once, within 0.01 mm: the page's stepped builder is held to the same build
/// (`object_builder::builds_as_exported`, within the export's gross bar), which a test of its own
/// once built again.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 100 s: a bull-nose's round turned on one arrangement")]
fn the_bull_nose_turned_partway_stops_inside_the_ring() {
    let e = example("stopped_bullnose.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_turn(),"a turn");
    let (d,big,c,h) = (10.,1.,0.3,1.);
    let rz = (big-c)*c+PI*c*c/4.+big*(h-c);
    let r2z = (big-c)*(big-c)*c+(big-c)*PI*c*c/2.+2.*c*c*c/3.+big*big*(h-c);
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),PI/3.*80.-(PI*d/3.*rz+PI/2.*r2z),1e-9);
    crate::object_builder::builds_as_exported("stopped_bullnose.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// `leaning_mill.sv` (the closed class, #780 L2: sharp circles leaning to a turn's axis): a
/// flat-end mill, a drum of radius 1 and 3 long with its ends square to its axis, leaning 25° from
/// upright 5 out from the turn's axis and 1.2 beside the plane through it, swung 100° wholly inside
/// a ring of radii 3 and 8 and height 5. Its side's contacts turn into surfaces of revolution held
/// exactly, its rims into the rings their circles sweep. Each slice of it is convex and clear of the
/// axis, so the ring loses the mill's own `3π` and `100/360` of the volume it sweeps turned whole,
/// its slices' radial spans worked in closed form (`swept_turned::flat_drum_whole`), to 1e-9.
/// Built once, within 0.01 mm: the page's stepped builder is held to the same build
/// (`object_builder::builds_as_exported`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: a leaning flat-end mill turned 100° on one arrangement")]
fn the_leaning_mill_turned_partway_bores_its_channel() {
    let e = example("leaning_mill.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_turn(),"a turn");
    let beta = 25f64.to_radians();
    let swept = 3.*PI+100./360.*crate::swept_turned::flat_drum_whole([beta.sin(),0.,beta.cos()]);
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),275.*PI-swept,1e-9);
    crate::object_builder::builds_as_exported("leaning_mill.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// What `ball_end_mill.sv` stated `lead` a turn leaves of its ring sector (`π/3 · 160`): in helical
/// coordinates `(ρ, ψ, ζ = z − kψ)`, `k` the lead over `2π`, a point of the sector is removed where
/// the helix through it meets the mill, which the swing carries wholly across the sector: `ζ`
/// within `[−M(ρ), 3 + k a]` (the ball's reach below, `swept_closed::helix_reach`, and the side's
/// above), which within the sector's heights is every point from `kψ − M` up to its top. So each
/// circle from 9 to 11 loses `2 + M` over the sector's angle (`kψ` averaging out across it), the
/// ring `π/3 (40 + ∫ ρ M dρ)`.
fn ball_end_mill_leaves(lead: f64) -> f64 {
    let k = lead/TAU;
    PI/3.*160.-PI/3.*(40.+crate::swept_closed::across_the_ring(10.,1.,&|rho| crate::swept_closed::helix_reach(rho,10.,1.,k).0))
}

/// `ball_end_mill.sv` (the closed class, a ball under a screw, #780 C1, #791): a ball-end mill,
/// its ball of radius 1 about a point 10 out under a side 3 tall, screwed 3 mm a turn from 60°
/// short of the sector's middle to 60° past it, wholly across. Its ball sweeps the pipe about its
/// centre's helix, which touches the cylinder its side screws into along the helix of the ball's
/// outer equator: at the pipe's start that helix, the cylinder's foot's rim and the pipe's meeting
/// with the foot's plane touch at one point, where the foot's pieces once did not close (the
/// meeting read leaving it 80 radians off, by probes of its parameter crowded at its start). The
/// ring loses `π/3 (40 + ∫ ρ M dρ)` (`ball_end_mill_leaves`), to 1e-8. Built once, within
/// 0.01 mm: its STL agrees with the field, and the page's stepped builder is held to the same build.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: a ball-end mill screwed on one arrangement")]
fn the_ball_end_mill_screwed_mills_its_helical_groove() {
    let e = example("ball_end_mill.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert_eq!(found.kind(),"a screw");
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),ball_end_mill_leaves(3.),1e-8);
    crate::object_builder::builds_as_exported("ball_end_mill.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// **`ball_end_mill.sv` at other leads** (#791): the point where the pipe's meeting with the foot's
/// plane, the foot's rim and the touch helix touch is there at every lead, so the build is held at
/// 2 and 4 mm a turn as well as the document's 3 — a fix that only moved one lead's rounding off
/// the edge would not hold the others. Each to `π/3 (40 + ∫ ρ M dρ)`, to 1e-8.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: the ball-end mill at two more leads")]
fn the_ball_end_mill_screwed_at_other_leads_mills_its_groove() {
    let text = text("ball_end_mill.sv");
    for lead in [2.,4.] {
        // its `lead` stated over the document's, as `examples::document`'s `with_params` does
        let (stated,_) = gcs_core::edit::rebind(&text,&[("lead".into(),format!("{lead}mm"))]).unwrap();
        near(exported(&fixtures::read(&stated),"block"),ball_end_mill_leaves(lead),1e-8);
    }
}

/// `leaning_ring.sv` (the closed class, a torus slid at a lean, #805): a ring of middle radius 3 and
/// tube 1, its axis leaning 35° from upright, pushed 8 straight down into a block until wholly in it.
/// The constant-twist class refuses it (it ends in the blank); the closed class cuts it on one
/// arrangement, its two contacts — smooth across the branch locus where each level's two points of
/// them meet — swept into the cylinders over their outlines, the hole's wall and its core's. The
/// block loses the centre's depth times the annulus between the outlines and half the ring
/// (`swept_slid::plunged_away`), to 1e-9, its STL agreeing with the field.
#[test]
fn the_ring_plunged_at_a_lean_bores_round_its_core() {
    let e = example("leaning_ring.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_slide(),"a slide");
    near(exported(&e,"block"),16.*16.*8.-crate::swept_slid::plunged_away(3.,1.,35f64.to_radians(),4.),1e-9);
}

/// `leaning_bullnose.sv` (the closed class, a torus's band slid at a lean, #810): a bull-nose mill of
/// radius 2, its corner rounded 0.5 and 3 tall, its axis leaning 30° from upright, pushed 8 straight
/// down into a block until wholly in it. The constant-twist class refuses it (it ends in the blank);
/// the closed class cuts it on one arrangement, its round's contact swept into the sheet over its
/// outline's arc, which the planes the side's rulings sweep touch along the seam points' paths,
/// given. The block loses the flux of `(0, 0, z)` out of the mill's faces turned down where it
/// stops (`swept_slid::Mill::plunged_away`), to 1e-9, its STL agreeing with the field.
#[test]
fn the_bull_nose_plunged_at_a_lean_stops_inside_the_block() {
    let e = example("leaning_bullnose.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_slide(),"a slide");
    let mill = crate::swept_slid::Mill {big:2.,c:0.5,top:0.,h:3.};
    near(exported(&e,"block"),16.*16.*8.-mill.plunged_away(30f64.to_radians(),[0.,0.,-4.]),1e-9);
}

/// `spline_mill.sv` (the closed class under any rigid motion, #780 L3): a bull-nose mill leaning
/// 30° toward where it goes, riding a frame along `path.sv`'s spline with its round sunk 1 mm in a
/// block. No constant twist carries it, and the generating class reads no frame along a path; the
/// closed class reads each instant's contact round the mill, fits the sheet it sweeps, and cuts the
/// block on one arrangement. The block loses what the mill holds below its top where it starts and
/// `∫ ∫ (n · v)⁺ dA dt` over its faces below the top (`swept_patches::advanced`), to a part in a
/// hundred million of what is left, its STL agreeing with the field. Built once, within 0.01 mm: the
/// page's stepped builder is held to the same build (`object_builder::builds_as_exported`).
#[test]
fn the_bull_nose_along_a_spline_cuts_what_its_faces_advance_through() {
    let e = example("spline_mill.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert_eq!(found.kind(),"any rigid motion");
    let path = found.path.clone();
    let revolved = gcs_core::brep::swept::patches::Revolved::read(&found.tool).unwrap();
    let groove = crate::swept_patches::advanced(&revolved,&found.tool,&path,found.roll,true);
    let exact = exported_within(&e,"block",Some(Tolerance::new(0.01).unwrap()));
    near(gcs_core::brep::props::volume(&exact.solid),42.*32.*8.-groove,1e-8);
    crate::object_builder::builds_as_exported("spline_mill.sv",&e.sketch,fixtures::solid(&e,"block"),&exact);
}

/// `stopped_turn.sv` (the closed class, X1): an upright bar turned from −140° to 30°, halfway into
/// the sector. The constant-twist class refuses it (it ends in the blank); the closed class cuts it
/// from the ring on one arrangement, its end pose's faces among the candidates. The ring loses each
/// circle's covered angle within the sector (`closed_class::turned_partway_away`), to 1e-9.
#[test]
fn the_bar_turned_partway_stops_inside_the_ring() {
    let e = example("stopped_turn.sv");
    let Class::Closed(found) = class(&e,"block") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_turn(),"a turn");
    near(exported(&e,"block"),PI/3.*80.-crate::closed_class::turned_partway_away(),1e-9);
}

/// `stopped_screw.sv` (the closed class, X1 under a screw): `square_thread.sv`'s bar screwed only
/// halfway up the rod. The constant-twist class refuses it (it ends in the blank); the closed class
/// cuts it on one arrangement, its level edges' helicoidal sheets met in closed form. The rod loses
/// each circle's covered angle (`closed_class::screwed_partway_away`), to 1e-9.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 40 s: a bar's screwed sheets cut from a rod on one arrangement")]
fn the_bar_screwed_partway_stops_inside_the_rod() {
    let e = example("stopped_screw.sv");
    let Class::Closed(found) = class(&e,"rod") else { panic!("the closed class") };
    assert_eq!(found.kind(),"a screw");
    near(exported(&e,"rod"),PI*25.*20.-crate::closed_class::screwed_partway_away(10.,8.),1e-9);
}

/// Every document in the folder is held by a test here, and the README names each.
#[test]
fn every_example_is_held_and_named() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/solid_sweeps");
    let readme = std::fs::read_to_string(dir.join("README.md")).unwrap();
    let held = include_str!("solid_sweeps_examples.rs");
    let mut names: Vec<String> = std::fs::read_dir(&dir).unwrap().flatten().map(|f| f.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".sv")).collect();
    names.sort();
    assert!(!names.is_empty());
    for name in names {
        assert!(readme.contains(&format!("`{name}`")),"the README names `{name}`");
        assert!(held.contains(&format!("example(\"{name}\")")),"a test reads `{name}`");
    }
}


