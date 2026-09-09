//! Bodies with continuous swept cuts through the CLI's generic construction
//! (`native::sweep_boundary`): sections of the native cutter, exact contacts,
//! arrangement of the declared blank and classification by the declared field.
//! Nothing here chooses a profile walk; the checks are recorded volumes, the
//! field on both sides of every sheet node, a closed-form fixture and refusals.
use super::*;
use gcs_core::{program,syntax,solve,interval::Interval};
use std::f64::consts::PI;

/// Link the gear project with the member component also publishing its blank
/// and a single unindexed space, for checks on one tooth without the others.
pub(super) fn read_gears_with(base: &Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let (mut p,errors) = syntax::parse(&source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| {
        let text = std::fs::read_to_string(base.join(format!("{name}.sv"))).ok()
            .or_else(|| gcs_core::library::resolve(name))?;
        let text = if name == "matched_pair" {
            text.replace("  solid body(design.heel)\n","  solid body(design.heel)\n  construction solid blank(design.heel)\n  outside_tip cut blank\n  design.toe cut blank\n  design.back cut blank\n  construction solid single(design.heel)\n  outside_tip cut single\n  design.toe cut single\n  design.back cut single\n  removal cut single\n")
        } else { text };
        Some(rewrite(name,text))
    });
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}"); e
}
pub(super) fn read_gears(base: &Path) -> program::Elaborated { read_gears_with(base,&mut |_,text| text) }

fn sub(a: [f64;3],b: [f64;3]) -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) }
fn dot(a: [f64;3],b: [f64;3]) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: [f64;3],b: [f64;3]) -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }

/// Kernel against field on both sides of every interior sheet node: wherever
/// the field is decisive, the united solid must agree.
fn sides_agree(cad: &Cad,part: c_int,blank: c_int,sheet: &native::sweep_boundary::Sheet,
    material: &mut gcs_core::solid::MaterialEvaluator) -> (usize,usize) {
    let inside = cad.0.solid_contains(blank,&sheet.points,1e-6).unwrap();
    let mut queries = Vec::new();
    for (i,(p,state)) in sheet.points.iter().zip(&inside).enumerate() {
        if *state != 1 || i % 11 != 0 { continue; }
        let (r,c) = (i/sheet.columns,i%sheet.columns);
        if r == 0 || c == 0 || r+1 == sheet.rows || c+1 == sheet.columns { continue; }
        let n = cross(sub(sheet.points[i+1],sheet.points[i-1]),sub(sheet.points[i+sheet.columns],sheet.points[i-sheet.columns]));
        let len = dot(n,n).sqrt(); if len == 0. { continue; }
        for sign in [-1.,1.] { queries.push(std::array::from_fn(|k| p[k]+sign*0.1*n[k]/len)); }
    }
    let states = cad.0.solid_contains(part,&queries,1e-6).unwrap();
    let (mut agree,mut disagree) = (0,0);
    for (p,state) in queries.iter().zip(&states) {
        let bounds = material.bounds(p.map(|x| Interval::point(x).unwrap()),
            gcs_core::interval::minimum::Options {value_tolerance:0.025,max_evaluations:20000}).unwrap();
        let [lo,hi] = bounds.value.bounds();
        if hi < 0. || lo > 0. { if (hi < 0.) == (*state == 1) { agree += 1; } else { disagree += 1; } }
    }
    (agree,disagree)
}

/// One member's single tooth space through the generic construction must
/// reproduce its recorded volume and agree with the declared field.
fn single_space(member: &str,expected: f64) {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
    let blank_id = e.map.ent_named(&format!("pair.{member}.blank")).unwrap().i();
    let single_id = e.map.ent_named(&format!("pair.{member}.single")).unwrap().i();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();
    let started = std::time::Instant::now();
    let (face,sheet,error) = native::sweep_boundary::swept_sheet(&cad.0,&e.sketch,removal,blank).unwrap();
    eprintln!("{member}: sheet {}x{} in {:?}, withheld error {error:e} mm",sheet.rows,sheet.columns,started.elapsed());
    assert!(error < 0.02,"withheld contact error {error}");
    let partition = cad.0.split_solid(blank,&[face]).unwrap();
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,single_id,1e-10).unwrap().evaluator(4096);
    let (kept,removed) = native::sweep_boundary::classify(&cad.0,partition,&mut material).unwrap();
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    assert!((cell_total-blank_volume).abs() < 1e-5*blank_volume);
    assert_eq!(removed.len(),1,"one tooth space");
    let space = removed[0].volume;
    eprintln!("{member}: tooth space {space:.6} mm^3 against recorded {expected:.6} ({:.2e} relative)",(space-expected).abs()/expected);
    assert!((space-expected).abs() < 1e-3*expected,"tooth space volume {space} vs recorded {expected}");
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    let (agree,disagree) = sides_agree(&cad,part,blank,&sheet,&mut material);
    eprintln!("{member}: {agree} side checks agree, {disagree} disagree");
    assert!(agree > 100 && disagree == 0);
}

#[test]
fn generic_sheet_reproduces_the_pinion_tooth_space() { single_space("pinion",120.708817); }

#[test]
fn generic_sheet_reproduces_the_gear_tooth_space() { single_space("gear",117.137321); }

/// A sphere swept about an axis parallel to its own is the degenerate case for
/// this construction: every cutter point's contact condition is constant, so no
/// station family parameterizes the envelope. It is refused, not mis-built; the
/// closed-form torus check of the arrangement itself lives in `cells.rs`.
const BEAD: &str = "
private point bc
bc distance(3.8mm, along: u) std.front
bc distance(0mm, along: v) std.front
private point bb hint(x: 3.8, y: -1)
private point bt hint(x: 3.8, y: 1)
private line bd(bb, bt)
bc midpoint bd
bd parallel spindle
distance(2mm) bd
private arc bm(center: bc, start: bb, end: bt)
radius(1mm) bm
construction solid bead(face(bm, bd), about: bd)
solid part(bead)
removal.body cut part
";

#[test]
fn a_cutter_with_a_motion_independent_contact_condition_is_refused() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let source = format!("{}{BEAD}",include_str!("../../../examples/solid_generating_sweep.sv"));
    let e = read(&source,&base);
    let part_id = e.map.ent_named("part").unwrap().i();
    let cad = Cad::new();
    let error = native::sweep_boundary::construct_solid(&cad.0,&e.sketch,part_id).unwrap_err();
    eprintln!("{error}");
    assert!(error.contains("does not depend on the motion"));
    let _ = PI;
}

#[test]
fn a_roll_that_leaves_the_cutter_in_the_blank_is_refused() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    // The gear's cutter still overlaps its blank at 35 degrees of roll.
    let e = read_gears_with(&base,&mut |name,text| if name == "matched_pair" { text.replace("roll_limit: 45deg","roll_limit: 35deg") } else { text });
    let removal = e.map.ent_named("pair.gear.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.gear.blank").unwrap().i();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let error = native::sweep_boundary::swept_sheet(&cad.0,&e.sketch,removal,blank).unwrap_err();
    eprintln!("{error}");
    assert!(error.contains("leaves the cutter inside the blank") && error.contains("35.0 degrees"));
}

#[test]
fn the_static_recipe_lists_swept_cuts_with_their_poses() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let body = e.map.ent_named("pair.pinion.body").unwrap().i();
    let removal = e.map.ent_named("pair.pinion.removal").unwrap().i();
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch,body).unwrap();
    assert_eq!(recipe.sweeps.len(),24);
    assert!(recipe.sweeps.iter().all(|s| s.swept == removal));
    // The static remainder is the blank: no sweep node, and the body's cuts are
    // only the boundary solids.
    let nodes = recipe.recipe.get("nodes").unwrap().arr();
    assert!(nodes.iter().all(|n| n.get("kind").unwrap().as_str() != "swept"));
    let cuts = nodes.iter().find(|n| n.get("id").unwrap().as_i64() as usize == body).unwrap().get("cut").unwrap().arr().len();
    assert_eq!(cuts,3);
    assert!(gcs_core::solid::cad::recipe(&e.sketch,body).unwrap_err().contains("continuous motion sweeps"));
}

fn whole_member(member: &str,expected: f64) {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let body = e.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
    let cad = Cad::new();
    let started = std::time::Instant::now();
    let part = native::sweep_boundary::construct_solid(&cad.0,&e.sketch,body).unwrap();
    let volume = cad.0.volume(part).unwrap();
    eprintln!("{member}: {volume:.6} mm^3 against recorded {expected:.6} in {:?}",started.elapsed());
    assert!((volume-expected).abs() < 1e-3*expected);
    let out = std::env::temp_dir().join(format!("solvent-{member}-{}",std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let (step,stl) = (out.join(format!("{member}.step")),out.join(format!("{member}.stl")));
    cad.0.step(part,step.to_str().unwrap()).unwrap();
    cad.0.stl(part,stl.to_str().unwrap()).unwrap();
    gcs_core::mesh::stl_shells(&std::fs::read(&stl).unwrap()).unwrap();
    eprintln!("exported {} and {}",step.display(),stl.display());
}

#[test]
#[ignore = "about four minutes: builds and exports the whole pinion to a temporary directory"]
fn whole_pinion_through_the_generic_construction() { whole_member("pinion",9141.979353); }

#[test]
#[ignore = "about eleven minutes: builds and exports the whole gear to a temporary directory"]
fn whole_gear_through_the_generic_construction() { whole_member("gear",20284.109756); }
