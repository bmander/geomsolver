//! **The solid sweeps series** (`rust/examples/solid_sweeps/`, #684): one short document for each
//! class of continuous sweep this kernel builds exactly. Each is read as the class its README
//! names — admitted to it, or for a ball read as its tube — built within 0.01 mm, its STL held to
//! the material field with no disagreement, and its volume held to a figure made apart from the
//! code under test: a closed form, or arithmetic that reads the tool and the motion and nothing
//! the construction does.
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Class,Options};
use std::f64::consts::{PI,TAU};

use crate::round_lowering::lens;
use crate::turns_and_slides::{exported,measure,near,reaches,turned_away,turned_torus,Lean};

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

/// `union.sv`: a union's sweep is its members' (#684 C), each bead's a slide's, admitted to the
/// constant-twist class as the union's sweep; the groove what of the rod's disc either bead's shadow
/// covers, in rows across the rod (`union_tools::beads_section`), through the rod's 20 mm.
#[test]
fn the_beads_leave_what_either_shadow_covers() {
    let e = example("union.sv");
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"rod"),&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    assert_eq!(a.sweeps().len(),2,"one sweep a bead");
    for s in a.sweeps() {
        let Class::ConstantTwist(found) = &s.class else { panic!("a bead's sweep is a slide's") };
        assert!(s.name == "groove" && found.screw.is_slide(),"read as the union's slide");
    }
    near(exported(&e,"rod"),PI*25.*20.-crate::union_tools::beads_section()*20.,2e-6);
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

