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
        let text = if name == "members" { fixtures::gear::publish_blank(&text,"  construction solid single(design.heel)\n  \
            design.tip bound single\n  design.toe cut single\n  design.back cut single\n  removal cut single\n") } else { text };
        rewrite(name,text)
    })
}
pub(super) fn read_gears(base: &Path) -> program::Elaborated { read_gears_with(base,&mut |_,text| text) }

/// The design an inspection reads: the bevel pair at `offset` millimetres between the shafts,
/// with the pressure shift and the crown's spiral angle from `SOLVENT_INSPECT_SHIFT` and
/// `_SPIRAL` (degrees) when set.
fn design_knobs(offset: f64) -> Vec<(&'static str,f64)> {
    let knob = |name: &str| std::env::var(name).ok().map(|v| v.parse::<f64>().unwrap());
    [("offset",Some(offset)),("pressure_shift",knob("SOLVENT_INSPECT_SHIFT")),("spiral_angle",knob("SOLVENT_INSPECT_SPIRAL"))]
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
    let base = fixtures::gear::project();
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
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 170 s: a native pinion tooth space built and measured")]
fn generic_sheet_reproduces_the_pinion_tooth_space() { single_space("pinion",120.708817); }

/// Phase 1 of docs/generating-sweeps-plan.md: one pinion space through the
/// same construction at the offset `SOLVENT_INSPECT_OFFSET` (millimetres),
/// reporting rather than asserting what it builds and how the field judges
/// both sides of its sheet. No recorded volume exists for a hypoid.
#[test]
#[ignore]
fn the_native_space_at_an_offset_against_its_field() {
    let offset: f64 = std::env::var("SOLVENT_INSPECT_OFFSET").ok().and_then(|v| v.parse().ok()).unwrap_or(15.);
    // `SOLVENT_INSPECT_SHIFT` and `SOLVENT_INSPECT_SPIRAL` (degrees) set the
    // pressure shift and the crown's spiral angle.
    let knobs = design_knobs(offset);
    let base = fixtures::gear::project();
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
    let base = fixtures::gear::project();
    // The gear's cutter still overlaps its blank at 35 degrees of roll.
    let e = read_gears_with(&base,&mut |name,text| fixtures::gear::roll(name,text,"gear",35.));
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
    let base = fixtures::gear::project();
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
    let base = fixtures::gear::project();
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
    let base = fixtures::gear::project();
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
    let base = fixtures::gear::project();
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

/// Phase 2 of docs/native-hypoid-plan.md: where the configured gear's sheet leaves its withheld
/// contacts. The cutter faces each node's source point lies on (by the motion's inverse at its
/// contact time; two at a corner's fan), a few columns row by row; the withheld contacts in or near
/// the blank against the fitted sheet, worst by normal turn, tallied by their cell's first node's
/// faces; each column's chord-length row parameters against the average the interpolation uses;
/// the fit under each parametrization; and any time leap between rows.
#[test]
#[ignore = "diagnostic, about 30 s: the configured gear's withheld contacts by cutter face"]
fn where_the_configured_gear_sheet_turns() {
    let e = fixtures::gear::read_configured_with(&mut |name,text| if name == "members" {
        fixtures::gear::publish_blank(&text,"") } else { text });
    // `SOLVENT_INSPECT_MEMBER`: the gear by default.
    let member = std::env::var("SOLVENT_INSPECT_MEMBER").unwrap_or("gear".into());
    let id = |n: &str| e.map.ent_named(&format!("pair.{member}.{n}")).unwrap().i();
    let sk = &e.sketch;
    let cad = Cad::new();
    let field = gcs_core::solid::SpatialField::read(sk,id("blank"),1e-10).unwrap();
    let scale = gcs_core::solid::cad::millimetres(sk).unwrap();
    let inside = |points: &[[f64;3]]| Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect());
    let removal = id("removal");
    // `SOLVENT_INSPECT_ROWS=length`: rows placed by length; by walk length otherwise.
    use native::sweep_boundary::Rows;
    let placement = if std::env::var("SOLVENT_INSPECT_ROWS").is_ok_and(|v| v == "length") { Rows::Length } else { Rows::Walk };
    let (face,sheet,_) = native::sweep_boundary::swept_sheets(&cad.0,sk,removal,&inside,&[placement],
        &mut |sheet| Ok((cad.0.fit_sheet(&sheet.points,sheet.rows,sheet.columns).unwrap(),sheet,0.))).unwrap();
    let gcs_core::model::SolidDef::Swept {source,motion,..} = &sk.solids[removal].def else { panic!() };
    let family = gcs_core::motion::Family::read(sk,*motion as usize).unwrap();
    let cutter = cad.0.construct(&gcs_core::solid::cad::recipe(sk,*source as usize).unwrap()).unwrap();
    let faces = cad.0.faces(cutter).unwrap();
    // The cutter point a contact came from, and the faces it lies on.
    let source_of = |p: [f64;3],t: f64| -> [f64;3] {
        scaled_point(family.at(t).unwrap().inverse().point(p.map(|x| x/scale)),scale)
    };
    let on = |q: [f64;3]| -> Vec<(usize,f64)> {
        let mut d: Vec<(usize,f64)> = faces.iter().enumerate().map(|(k,&f)| (k,cad.0.face_normal(f,q).map(|x| x.1).unwrap_or(f64::INFINITY))).collect();
        d.sort_by(|a,b| a.1.total_cmp(&b.1));
        d.into_iter().filter(|x| x.1 < 1e-4).collect()
    };
    for (k,&f) in faces.iter().enumerate() {
        let p = cad.0.face_point(f,0.5,0.5,1e-7).ok().flatten();
        eprintln!("cutter face {k}: kind {} mid {:?}",cad.0.face_kind(f).unwrap(),p.map(|p| p.position.map(|x| (x*1e3).round()/1e3)));
    }
    let (rows,columns) = (sheet.rows,sheet.columns);
    eprintln!("sheet {rows}x{columns}, {} withheld",sheet.withheld.len());
    // The middle column's rows: which face each node's source lies on, its time, whether in the blank.
    for c in [columns/4,columns/2,3*columns/4,columns-2,columns-1] {
        eprintln!("column {c}:");
        for r in 0..rows {
            let k = r*columns+c;
            let q = source_of(sheet.points[k],sheet.times[k]);
            let step = if r > 0 { gcs_core::space::distance(sheet.points[k],sheet.points[k-columns]) } else { 0. };
            eprintln!("  row {r:3}: faces {:?} time {:8.4} in {} step {step:.4} p {:?}",on(q).iter().map(|x| x.0).collect::<Vec<_>>(),
                sheet.times[k],field.value(sheet.points[k].map(|x| x/scale)) < 0.,sheet.points[k].map(|x| (x*1e3).round()/1e3));
        }
    }
    let feet = cad.0.surface_feet(face,&sheet.withheld).unwrap();
    let mut rows_out = Vec::new();
    for (i,(p,n)) in sheet.withheld.iter().zip(&sheet.withheld_normals).enumerate() {
        let near = field.value(p.map(|x| x/scale));
        if near >= 0.5/scale { continue; }
        let Some((m,gap)) = feet[i] else { rows_out.push((180.,f64::INFINITY,i,near)); continue };
        let angle = dot(m,*n).abs().min(1.).acos().to_degrees();
        rows_out.push((angle,gap,i,near));
    }
    rows_out.sort_by(|a,b| b.0.total_cmp(&a.0));
    let mut tally: std::collections::BTreeMap<Vec<usize>,(usize,usize,f64,f64)> = Default::default();
    // Withheld contacts are the mid stations' curves at mid rows, rows-1 of them per column.
    let place = |i: usize| (i%(rows-1),i/(rows-1));
    for &(angle,gap,i,_) in &rows_out {
        // No time is kept with a withheld contact: it is attributed to its cell's first node.
        let (r,c) = place(i);
        let k = r*columns+c;
        let q = source_of(sheet.points[k],sheet.times[k]);
        let key: Vec<usize> = on(q).iter().map(|x| x.0).collect();
        let e = tally.entry(key).or_insert((0,0,0.,0.));
        e.0 += 1; if angle > 20. { e.1 += 1; } e.2 = e.2.max(angle); e.3 = e.3.max(gap);
    }
    for &(angle,gap,i,near) in rows_out.iter().take(25) {
        let (r,c) = place(i);
        let (k,t) = (r*columns+c,sheet.times[r*columns+c]);
        let q = source_of(sheet.points[k],t);
        eprintln!("withheld row {r} column {c}: {angle:.2} degrees, gap {gap:.4} mm, blank value {near:.3}, node time {t:.4}, node faces {:?}, at {:?}",
            on(q),sheet.withheld[i].map(|x| (x*1e3).round()/1e3));
    }
    for (k,(n,bad,angle,gap)) in &tally { eprintln!("faces {k:?}: {n} withheld, {bad} over 20 degrees, worst {angle:.2} degrees, gap {gap:.4}"); }
    // Each column's own chord-length row parameters against their average over the columns, which is
    // what the interpolation uses: how far (in rows) a column's node sits from where the fit puts it.
    let own: Vec<Vec<f64>> = (0..columns).map(|c| {
        let mut l = vec![0.];
        for r in 1..rows { l.push(l[r-1]+gcs_core::space::distance(sheet.points[r*columns+c],sheet.points[(r-1)*columns+c])); }
        let total = l[rows-1]; l.iter().map(|x| x/total).collect()
    }).collect();
    let mean: Vec<f64> = (0..rows).map(|r| own.iter().map(|o| o[r]).sum::<f64>()/columns as f64).collect();
    let mut worst = (0.,0,0);
    for c in 0..columns { for r in 1..rows-1 {
        let d = (own[c][r]-mean[r]).abs()/(mean[r+1]-mean[r-1]).abs()*2.;
        if d > worst.0 { worst = (d,r,c); }
    } }
    eprintln!("chord-length parameters: a column's node is at most {:.2} rows from the average parameter (row {} column {})",worst.0,worst.1,worst.2);
    for c in [0,columns/4,columns/2,3*columns/4,columns-1] {
        let steps: Vec<f64> = (1..rows).map(|r| own[c][r]-own[c][r-1]).collect();
        let (lo,hi) = steps.iter().fold((f64::INFINITY,0_f64),|(l,h),s| (l.min(*s),h.max(*s)));
        let jump = (1..steps.len()).map(|k| steps[k].max(steps[k-1])/steps[k].min(steps[k-1])).fold(0.,f64::max);
        eprintln!("  column {c}: row steps {:.4} to {:.4} of its length, adjacent steps differ by up to {jump:.1}x",lo,hi);
    }
    for parametrization in [0,1,2] {
        let f = cad.0.fit_sheet_with(&sheet.points,rows,columns,parametrization).unwrap();
        let feet = cad.0.surface_feet(f,&sheet.withheld).unwrap();
        let (mut gap,mut turn) = (0_f64,0_f64);
        for (i,(p,n)) in sheet.withheld.iter().zip(&sheet.withheld_normals).enumerate() {
            if field.value(p.map(|x| x/scale)) >= 0.5/scale { continue; }
            let Some((m,g)) = feet[i] else { gap = f64::INFINITY; continue };
            gap = gap.max(g); turn = turn.max(dot(m,*n).abs().min(1.).acos().to_degrees());
        }
        eprintln!("parametrization {parametrization}: withheld within {gap:.4} mm, normals within {turn:.2} degrees");
    }
    // Where a column's time leaps between rows: in the margin, the sheet's rows now stop before one.
    let leaps: Vec<(usize,usize,f64)> = (0..columns).flat_map(|c| (1..rows).map(move |r| (r,c)))
        .map(|(r,c)| (r,c,(sheet.times[r*columns+c]-sheet.times[(r-1)*columns+c]).abs())).filter(|x| x.2 >= 1.).collect();
    eprintln!("time leaps of a radian or more between rows: {leaps:?}");
    // Where the fitted face folds: its own normal turning by more than 60 degrees between
    // neighbouring evaluations of a grid four times finer than the sheet's.
    let (nu,nv) = (4*(rows-1)+1,4*(columns-1)+1);
    let at: Vec<Option<native::kernel::FacePoint>> = (0..nu*nv).map(|k| cad.0.face_point(face,(k/nv) as f64/(nu-1) as f64,
        (k%nv) as f64/(nv-1) as f64,1e-7).unwrap()).collect();
    let (mut folds,mut in_blank) = (Vec::new(),0_f64);
    for i in 0..nu { for j in 0..nv {
        let Some(a) = &at[i*nv+j] else { continue };
        for (di,dj) in [(1,0),(0,1)] {
            if i+di >= nu || j+dj >= nv { continue }
            let Some(b) = &at[(i+di)*nv+j+dj] else { continue };
            let turn = dot(a.normal,b.normal).clamp(-1.,1.).acos().to_degrees();
            if field.value(a.position.map(|x| x/scale)) < 0. { in_blank = in_blank.max(turn); }
            if turn > 60. { folds.push((turn,i as f64/4.,j as f64/4.,a.position,field.value(a.position.map(|x| x/scale)))); }
        }
    } }
    eprintln!("{} neighbouring evaluations of the fitted face turn over 60 degrees, {} of them in or within 0.5 mm of the blank",
        folds.len(),folds.iter().filter(|f| f.4 < 0.5/scale).count());
    eprintln!("in the blank, neighbouring evaluations turn by at most {in_blank:.2} degrees");
    for (turn,r,c,p,v) in folds.iter().filter(|f| f.4 < 0.5/scale).take(30) {
        eprintln!("  near row {r:.2} column {c:.2}: {turn:.1} degrees at {:?}, blank value {v:.3}",p.map(|x| (x*1e3).round()/1e3));
    }
}

fn scaled_point(p: [f64;3],s: f64) -> [f64;3] { p.map(|x| x*s) }
