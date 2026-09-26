//! Bodies with continuous swept cuts through the CLI's generic construction
//! (`native::sweep_boundary`): sections of the native cutter, exact contacts,
//! arrangement of the declared blank and classification by the declared field.
//! Nothing here chooses a profile walk; the checks are recorded volumes, the
//! field on both sides of every sheet node, a closed-form fixture and refusals.
use super::*;
use gcs_core::{program,interval::Interval};
use std::f64::consts::PI;

/// Link the gear project with the member component also publishing its blank
/// and a single unindexed space, for checks on one tooth without the others.
/// The recorded volumes are the bevel pair's, so the axis offset reads as zero.
pub(super) fn read_gears_with(base: &Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    fixtures::gear::read_with(&source,base,&mut |name,text| {
        let text = if name == "matched_pair" { fixtures::gear::publish_blank(&text,"  construction solid single(design.heel)\n  \
            design.tip bound single\n  design.toe cut single\n  design.back cut single\n  removal cut single\n") } else { text };
        rewrite(name,text)
    })
}
pub(super) fn read_gears(base: &Path) -> program::Elaborated { read_gears_with(base,&mut |_,text| text) }

/// The design an inspection reads: the bevel pair at `offset` degrees, with the pressure shift
/// and the crown's spiral angle from `SOLVENT_INSPECT_SHIFT` and `_SPIRAL` (degrees) when set.
fn design_knobs(offset: f64) -> Vec<(&'static str,f64)> {
    let knob = |name: &str| std::env::var(name).ok().map(|v| v.parse::<f64>().unwrap());
    [("offset_angle",Some(offset)),("pressure_shift",knob("SOLVENT_INSPECT_SHIFT")),("spiral_angle",knob("SOLVENT_INSPECT_SPIRAL"))]
        .into_iter().filter_map(|(p,v)| v.map(|v| (p,v))).collect()
}

use gcs_core::space::{sub,dot,cross,norm};

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
    let (face,sheet,error) = native::sweep_boundary::swept_sheet(&cad.0,&e.sketch,removal,blank,&gcs_core::solid::SpatialField::read(&e.sketch,blank_id,1e-10).unwrap()).unwrap();
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

/// Phase 1 of docs/generating-sweeps-plan.md: one pinion space through the
/// same construction at the offset angle `SOLVENT_INSPECT_OFFSET` (degrees),
/// reporting rather than asserting what it builds and how the field judges
/// both sides of its sheet. No recorded volume exists for a hypoid.
#[test]
#[ignore]
fn the_native_space_at_an_offset_against_its_field() {
    let offset: f64 = std::env::var("SOLVENT_INSPECT_OFFSET").ok().and_then(|v| v.parse().ok()).unwrap_or(15.);
    // `SOLVENT_INSPECT_SHIFT` and `SOLVENT_INSPECT_SPIRAL` (degrees) set the
    // pressure shift and the crown's spiral angle.
    let knobs = design_knobs(offset);
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears_with(&base,&mut |name,text| fixtures::gear::configure(name,text,&knobs));
    // `SOLVENT_INSPECT_MEMBER`: pinion by default.
    let member = std::env::var("SOLVENT_INSPECT_MEMBER").unwrap_or("pinion".into());
    let id = |n: &str| e.map.ent_named(&format!("pair.{member}.{n}")).unwrap().i();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,id("blank")).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();
    let started = std::time::Instant::now();
    let built = native::sweep_boundary::swept_sheet(&cad.0,&e.sketch,id("removal"),blank,&gcs_core::solid::SpatialField::read(&e.sketch,id("blank"),1e-10).unwrap());
    let (face,sheet,error) = match built {
        Ok(b) => b,
        Err(refusal) => { eprintln!("offset {offset}: sheet refused: {refusal}"); return; }
    };
    eprintln!("offset {offset}: sheet {}x{} in {:?}, withheld error {error:e} mm",sheet.rows,sheet.columns,started.elapsed());
    let partition = cad.0.split_solid(blank,&[face]).unwrap();
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,id("single"),1e-10).unwrap().evaluator(4096);
    let (kept,removed) = match native::sweep_boundary::classify(&cad.0,partition,&mut material) {
        Ok(c) => c,
        Err(refusal) => { eprintln!("offset {offset}: classification refused: {refusal}"); return; }
    };
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    eprintln!("offset {offset}: blank {blank_volume:.4}, cells {cell_total:.4}, {} kept, {} removed {:?}",
        kept.len(),removed.len(),removed.iter().map(|c| c.volume).collect::<Vec<_>>());
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    let (agree,disagree) = sides_agree(&cad,part,blank,&sheet,&mut material);
    eprintln!("offset {offset}: {agree} side checks agree, {disagree} disagree ({:?})",started.elapsed());
}

#[test]
#[ignore = "the traced sheet (robustness step 3) fails this gear space's fit contract near the cutter's \
    crease, the class B pleat of docs/generating-sweeps-robustness.md; the harness's gear rows are the \
    native path's acceptance record, and field meshing (docs/field-meshing.md) is its replacement"]
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
    // Admission is the gate that names it (M2); the construction behind it refuses too, for
    // whichever reason its sampling meets first.
    use gcs_core::solid::admission;
    match admission::admit_body(&e.sketch,part_id,&admission::Options::default()) {
        Err(admission::Error::Refused(r)) => { eprintln!("{r}"); assert_eq!(r.condition,admission::Condition::Stationary); }
        other => panic!("admitted or unreadable: {other:?}"),
    }
    // Nor can the construction be reached without an admission.
    let cad = Cad::new();
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch,part_id).unwrap();
    assert!(native::sweep_boundary::construct_solid(&cad.0,&e.sketch,part_id,&recipe,None).is_err());
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
    let error = native::sweep_boundary::swept_sheet(&cad.0,&e.sketch,removal,blank,&gcs_core::solid::SpatialField::read(&e.sketch,blank_id,1e-10).unwrap()).unwrap_err();
    eprintln!("{error}");
    assert!(error.message.contains("leaves the cutter inside the blank") && error.message.contains("35.0 degrees"));
    assert_eq!(error.stage,gcs_core::solid::export::Stage::Clearance);
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
    // The static remainder is the blank: no sweep node, and the body's cuts and
    // bounds are only the boundary solids (toe and back cut, the tip cone bounds).
    let nodes = recipe.recipe.get("nodes").unwrap().arr();
    assert!(nodes.iter().all(|n| n.get("kind").unwrap().as_str() != "swept"));
    let blank = nodes.iter().find(|n| n.get("id").unwrap().as_i64() as usize == body).unwrap();
    assert_eq!(blank.get("cut").unwrap().arr().len(),2);
    assert_eq!(blank.get("bound").unwrap().arr().len(),1);
    assert!(gcs_core::solid::cad::recipe(&e.sketch,body).unwrap_err().contains("continuous motion sweeps"));
}

fn whole_member(member: &str,expected: f64) {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let body = e.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
    let cad = Cad::new();
    let started = std::time::Instant::now();
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch,body).unwrap();
    let admitted = gcs_core::solid::admission::admit_body(&e.sketch,body,&Default::default()).unwrap();
    let part = native::sweep_boundary::construct_solid(&cad.0,&e.sketch,body,&recipe,Some(&admitted)).unwrap();
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

#[test]
#[ignore = "measurement: cost of one material probe against the whole gear body field"]
fn measure_gear_probe_cost() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let body_id = e.map.ent_named("pair.gear.body").unwrap().i();
    let single_id = e.map.ent_named("pair.gear.single").unwrap().i();
    let points = [("tooth space",[53.78966032067442,-2.8070521623123743,0.09038416472908987]),
        ("material",[-11.929242531843038,-39.35479647066296,-36.23791869602705])];
    for (label,id) in [("single",single_id),("body",body_id)] {
        let mut material = gcs_core::solid::MaterialField::read(&e.sketch,id,1e-10).unwrap().evaluator(4096);
        for (what,p) in points {
            for distance in [0.05,0.0125] {
                let started = std::time::Instant::now();
                let probe = material.probe(p.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
                    gcs_core::interval::minimum::Options {value_tolerance:distance/4.,max_evaluations:40000}).unwrap();
                let evaluations: usize = probe.center.sweeps.iter().map(|q| q.minimum.evaluations).sum();
                let sides: usize = probe.sides.iter().flatten().map(|b| b.sweeps.iter().map(|q| q.minimum.evaluations).sum::<usize>()).sum();
                let mut statuses = std::collections::BTreeMap::new();
                for q in &probe.center.sweeps { *statuses.entry(format!("{:?}",q.minimum.status)).or_insert(0) += 1; }
                eprintln!("{label} field, {what} point, distance {distance}: {:?} in {:?}; {} sweep queries, {evaluations} centre + {sides} side evaluations, {statuses:?}, cached poses {}",
                    probe.state,started.elapsed(),probe.center.sweeps.len(),material.cached_poses());
            }
        }
    }
}

/// Where a sheet fails its withheld contacts, for the design `SOLVENT_INSPECT_OFFSET`,
/// `_SHIFT`, `_SPIRAL` and `_MEMBER`: each withheld contact farther than 0.05 mm from the
/// fitted sheet with its column and whether it lies in the blank, and each column's
/// largest step between consecutive contacts against its median.
#[test]
#[ignore]
fn where_a_sheet_misses_its_contacts() {
    let offset: f64 = std::env::var("SOLVENT_INSPECT_OFFSET").ok().and_then(|v| v.parse().ok()).unwrap_or(25.);
    let knobs = design_knobs(offset);
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears_with(&base,&mut |name,text| fixtures::gear::configure(name,text,&knobs));
    let member = std::env::var("SOLVENT_INSPECT_MEMBER").unwrap_or("gear".into());
    let id = |n: &str| e.map.ent_named(&format!("pair.{member}.{n}")).unwrap().i();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,id("blank")).unwrap()).unwrap();
    let field = gcs_core::solid::SpatialField::read(&e.sketch,id("blank"),1e-10).unwrap();
    let (_face,sheet,error) = native::sweep_boundary::swept_sheet(&cad.0,&e.sketch,id("removal"),blank,&field).unwrap();
    eprintln!("sheet {}x{}, withheld error {error:.4}",sheet.rows,sheet.columns);
    // Each grid cell's own normal (from its diagonals) against the contact normals at its
    // corners: a sheared or folded grid shows as cells where they disagree.
    let at = |r: usize,c: usize| r*sheet.columns+c;
    let mut bad = Vec::new();
    for r in 0..sheet.rows-1 { for c in 0..sheet.columns-1 {
        let n = cross(sub(sheet.points[at(r+1,c+1)],sheet.points[at(r,c)]),sub(sheet.points[at(r,c+1)],sheet.points[at(r+1,c)]));
        if norm(n) == 0. { bad.push((180.,r,c)); continue; }
        let worst = [at(r,c),at(r+1,c),at(r,c+1),at(r+1,c+1)].iter().map(|&k| {
            let m = sheet.normals[k];
            ((n[0]*m[0]+n[1]*m[1]+n[2]*m[2])/norm(n)).abs().min(1.).acos().to_degrees()
        }).fold(0.,f64::max);
        if worst > 30. { bad.push((worst,r,c)); }
    }}
    eprintln!("{} of {} cells whose own normal is over 30 degrees from a corner's contact normal",bad.len(),(sheet.rows-1)*(sheet.columns-1));
    let mut rows_hit: Vec<usize> = bad.iter().map(|b| b.1).collect(); rows_hit.sort(); rows_hit.dedup();
    eprintln!("  rows involved: {rows_hit:?}");
    for (w,r,c) in bad.iter().take(12) {
        let p = sheet.points[at(*r,*c)];
        eprintln!("  cell row {r} column {c}: {w:.1} degrees, in blank {}, at {:?}",field.value(p) < 0.,p.map(|x| (x*1e3).round()/1e3));
    }
}
