//! Both spiral bevel members by arrangement and classification: one tooth-space
//! sheet per member from the declared cutter's contact charts, indexed by the
//! declared motion, splitting the declared blank, with every cell judged by the
//! declared material field. The profile walk each sheet follows is chosen here by
//! hand from the cutter's patch order; making the CLI derive it is the next step.
//! Ignored tests hold the whole-member builds and the explorations that chose
//! the bands, u ranges and roll limits.
use super::*;
use gcs_core::{program,syntax,solve,interval::Interval};

/// Link the gear project with the member component also publishing its blank,
/// so the stock can be constructed natively without the generating cuts.
pub(super) fn read_gears(base: &Path) -> program::Elaborated {
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let (mut p,errors) = syntax::parse(&source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| {
        let text = std::fs::read_to_string(base.join(format!("{name}.sv"))).ok()
            .or_else(|| gcs_core::library::resolve(name))?;
        if name != "matched_pair" { return Some(text); }
        // `blank` is the stock before any generating cut; `single` subtracts one
        // unindexed removal, the material a one-tooth partition must be judged by.
        Some(text.replace("  solid body(design.heel)\n","  solid body(design.heel)\n  construction solid blank(design.heel)\n  outside_tip cut blank\n  design.toe cut blank\n  design.back cut blank\n  construction solid single(design.heel)\n  outside_tip cut single\n  design.toe cut single\n  design.back cut single\n  removal cut single\n"))
    });
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}"); e
}

#[test]
#[ignore = "exploration: prints source chart root counts and endpoint reach"]
fn explore_pinion_source_chart_and_endpoint_reach() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    for member in ["pinion","gear"] {
        let removal = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let blank = e.map.ent_named(&format!("pair.{member}.blank")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
        let [roll0,roll1] = sweep.domain();
        eprintln!("== {member}: roll [{roll0},{roll1}], {} patches",sweep.patches().len());
        let cad = Cad::new();
        let stock = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank).unwrap()).unwrap();
        let caps = match cad.0.sweep_caps(&e.sketch,removal) {
            Ok(caps) => caps,
            Err(error) => { eprintln!("  endpoint construction failed: {error}"); continue; }
        };
        for cap in &caps.endpoints {
            eprintln!("  endpoint {:.3} rad: cutter/blank common volume {:.6} mm^3 (blank {:.3})",
                cap.parameter,cad.0.common_volume(cap.solid,stock).unwrap(),cad.0.volume(stock).unwrap());
        }
        for (i,patch) in sweep.patches().iter().enumerate() {
            let [_,[v0,v1]] = patch.domain();
            let (nu,nv) = (16,32);
            let mut histogram = [0;4];
            let (mut tmin,mut tmax) = (f64::INFINITY,f64::NEG_INFINITY);
            let mut errors = 0;
            for a in 0..=nu { for b in 0..=nv {
                let u = 0.02+0.96*a as f64/nu as f64;
                let v = v0+(v1-v0)*b as f64/nv as f64;
                match sweep.at_source(i,u,v,1e-9) {
                    Ok(roots) => {
                        histogram[roots.len().min(3)] += 1;
                        for r in roots { tmin = tmin.min(r.root.time); tmax = tmax.max(r.root.time); }
                    }
                    Err(_) => errors += 1,
                }
            } }
            eprintln!("  patch {i} {} v[{v0},{v1}] periodic {}: roots 0/1/2/3+ = {:?}, errors {errors}, time [{tmin:.3},{tmax:.3}]",
                patch.name,patch.is_periodic(),histogram);
            // Shape of the root region in seam-centered v, per u row and interval.
            for a in [0,nu/2,nu] {
                let u = 0.02+0.96*a as f64/nu as f64;
                let mut line = format!("    u={u:.2}:");
                for factor in [1.,1.7,2.5] {
                    let interval = [factor*roll0,factor*roll1];
                    let (mut lo,mut hi,mut count,mut multiple) = (f64::INFINITY,f64::NEG_INFINITY,0,0);
                    for b in 0..=100 {
                        let v = -0.5+b as f64/100.;
                        let roots = sweep.at_source_over(i,u,v.rem_euclid(1.),interval,1e-9).unwrap();
                        if roots.len() > 1 { multiple += 1; }
                        if !roots.is_empty() { lo = lo.min(v); hi = hi.max(v); count += 1; }
                    }
                    line += &format!(" x{factor}: v[{lo:.2},{hi:.2}] {count} nodes {multiple} multi;");
                }
                eprintln!("{line}");
            }
        }
    }
}

/// One tooth-space sheet: the five active patches of the crown section joined
/// along the profile, over a v band through the revolution seam, with contact
/// times taken over `interval`. Rows follow the profile, columns the band.
struct Sheet { points: Vec<[f64;3]>,rows: usize,columns: usize,nodes: Vec<(usize,f64,f64,f64)> }

fn tooth_sheet(sweep: &SweepContacts,patches: &[(usize,usize,[f64;2])],band: [f64;2],columns: usize,
    interval: [f64;2]) -> Sheet {
    let rows = patches.iter().map(|p| p.1).sum::<usize>()+1;
    let mut points = Vec::with_capacity(rows*(columns+1));
    let mut nodes = Vec::new();
    let mut row = 0;
    let centre = columns/2;
    for (k,&(patch,steps,[u0,u1])) in patches.iter().enumerate() {
        let last = k+1 == patches.len();
        for a in 0..=steps {
            if a == steps && !last { continue; }
            let u = u0+(u1-u0)*a as f64/steps as f64;
            // Contact times along one profile station: the root nearest the
            // declared interval's centre at the band centre, then by continuity
            // outward, so a second root a half turn away is never mixed in.
            let mut line: Vec<Option<(f64,[f64;3])>> = vec![None;columns+1];
            for (order,b) in (centre..=columns).chain((0..centre).rev()).enumerate() {
                let v = (band[0]+(band[1]-band[0])*b as f64/columns as f64).rem_euclid(1.);
                let roots = sweep.at_source_over(patch,u,v,interval,1e-9).unwrap();
                assert!(!roots.is_empty(),"patch {patch} u={u} v={v}: no contact time in {interval:?}");
                let previous = if order == 0 { 0. } else if b > centre { line[b-1].unwrap().0 } else { line[b+1].unwrap().0 };
                let root = roots.iter().min_by(|x,y| (x.root.time-previous).abs().total_cmp(&(y.root.time-previous).abs())).unwrap();
                line[b] = Some((root.root.time,root.contact.position));
            }
            for b in 0..=columns {
                let v = (band[0]+(band[1]-band[0])*b as f64/columns as f64).rem_euclid(1.);
                let (time,position) = line[b].unwrap();
                points.push(position);
                nodes.push((patch,u,v,time));
            }
            row += 1;
        }
    }
    assert_eq!(row,rows);
    Sheet {points,rows,columns:columns+1,nodes}
}

#[test]
fn pinion_tooth_space_from_one_sheet_and_material_cells() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named("pair.pinion.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.pinion.blank").unwrap().i();
    let single_id = e.map.ent_named("pair.pinion.single").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let [roll0,roll1] = sweep.domain();
    let names: Vec<_> = sweep.patches().iter().map(|p| p.name.rsplit('.').next().unwrap().to_string()).collect();
    assert_eq!(names,["base","outer","outer_round","tip","inner_round","inner"]);
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();

    let started = std::time::Instant::now();
    let wide = [-std::f64::consts::PI,std::f64::consts::PI];
    // Rows are allocated by the envelope's arc length along the band centre, so
    // the interpolation sees nearly even spacing across the patch joins.
    let ranges = [(1,[0.2,1.]),(2,[0.,1.]),(3,[0.,1.]),(4,[0.,1.]),(5,[0.,0.8])];
    let band: [f64;2] = [-0.21,-0.09];
    let lengths: Vec<f64> = ranges.iter().map(|&(patch,[u0,u1])| {
        let mut length = 0.; let mut last: Option<[f64;3]> = None;
        for a in 0..=16 {
            let mut roots = sweep.at_source_over(patch,u0+(u1-u0)*a as f64/16.,((band[0]+band[1])/2.).rem_euclid(1.),wide,1e-9).unwrap();
            roots.sort_by(|x,y| x.root.time.abs().total_cmp(&y.root.time.abs()));
            let p = roots[0].contact.position;
            if let Some(q) = last { length += distance(p,q); }
            last = Some(p);
        }
        length
    }).collect();
    let total: f64 = lengths.iter().sum();
    let plan: Vec<_> = ranges.iter().zip(&lengths).map(|(&(patch,range),&length)|
        (patch,((length/total*88.).round() as usize).max(4),range)).collect();
    eprintln!("arc lengths {lengths:?} mm; rows per patch {:?}",plan.iter().map(|p| p.1).collect::<Vec<_>>());
    let sheet = tooth_sheet(&sweep,&plan,band,48,wide);
    let mut far = 0_f64; let mut jumps = 0_f64;
    for (i,n) in sheet.nodes.iter().enumerate() {
        far = far.max(n.3.abs());
        if i % sheet.columns != 0 {
            let step = (n.3-sheet.nodes[i-1].3).abs();
            if step > 0.5 { eprintln!("  time jump {step:.3} at patch {} u={:.3} v={:.3}",n.0,n.1,n.2); }
            jumps = jumps.max(step);
        }
    }
    eprintln!("sheet {}x{} in {:?}; largest |time| {far:.3} rad, largest time step between columns {jumps:.3}",
        sheet.rows,sheet.columns,started.elapsed());
    // The sheet must leave the blank on every side, or the split cannot separate.
    let mut boundary = Vec::new();
    for r in 0..sheet.rows { for c in 0..sheet.columns {
        if r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns { boundary.push(sheet.points[r*sheet.columns+c]); }
    } }
    let states = cad.0.solid_contains(blank,&boundary,1e-6).unwrap();
    assert!(states.iter().all(|s| *s == 0),"{} sheet boundary nodes inside the blank",states.iter().filter(|s| **s != 0).count());
    // Contacts within the declared roll must all lie beyond the blank at its ends.
    let declared: Vec<_> = sheet.nodes.iter().zip(&sheet.points).filter(|(n,_)| n.3 >= roll0 && n.3 <= roll1).collect();
    let states = cad.0.solid_contains(blank,&declared.iter().map(|(_,p)| **p).collect::<Vec<_>>(),1e-6).unwrap();
    let inside: Vec<_> = declared.iter().zip(&states).filter(|(_,s)| **s == 1).map(|((n,_),_)| **n).collect();
    let (mut vlo,mut vhi,mut tlo,mut thi) = (f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,f64::NEG_INFINITY);
    let mut u_range = std::collections::BTreeMap::new();
    for (patch,u,v,t) in &inside {
        let v = if *v > 0.5 { v-1. } else { *v };
        vlo = vlo.min(v); vhi = vhi.max(v); tlo = tlo.min(*t); thi = thi.max(*t);
        let entry = u_range.entry(*patch).or_insert((f64::INFINITY,f64::NEG_INFINITY));
        entry.0 = entry.0.min(*u); entry.1 = entry.1.max(*u);
    }
    eprintln!("{} of {} declared-roll nodes lie inside the blank: v in [{vlo:.3},{vhi:.3}], time in [{tlo:.3},{thi:.3}], u per patch {u_range:?}",
        inside.len(),declared.len());
    let face = cad.fit_with(&sheet.points,sheet.rows,sheet.columns,1).unwrap();
    // Withheld fit check: exact contacts between grid nodes, projected onto the
    // fitted face; the incidence distance is the fit error there.
    let mut worst = 0_f64; let mut checked = 0;
    for r in 0..sheet.rows-1 { for c in (3..sheet.columns-1).step_by(9) {
        let (patch,u0,v0,_) = sheet.nodes[r*sheet.columns+c];
        let (q,u1,v1,_) = sheet.nodes[(r+1)*sheet.columns+c+1];
        if q != patch { continue; }
        let v = (v0+v1)/2.;
        let mut exact = sweep.at_source_over(patch,(u0+u1)/2.,v,wide,1e-9).unwrap();
        exact.sort_by(|a,b| a.root.time.abs().total_cmp(&b.root.time.abs()));
        let p = exact[0].contact.position;
        let gap = cad.0.face_parameters(face,p,0.5).unwrap().map(|(_,gap)| gap).unwrap_or(0.5);
        worst = worst.max(gap); checked += 1;
    } }
    eprintln!("withheld fit error {worst:e} mm over {checked} interior points");

    let started = std::time::Instant::now();
    let partition = cad.0.split_solid(blank,&[face]).unwrap();
    eprintln!("split in {:?}; tolerances blank {:?}, sheet {:?}, partition {:?}",started.elapsed(),
        cad.0.tolerances(blank).unwrap(),cad.0.tolerances(face).unwrap(),cad.0.tolerances(partition).unwrap());
    for cell in cad.0.solids(partition).unwrap() { eprintln!("  cell tolerances {:?}",cad.0.tolerances(cell).unwrap()); }
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,single_id,1e-10).unwrap().evaluator(4096);
    let started = std::time::Instant::now();
    let (kept,removed) = super::cells::classify(&cad,partition,&mut material).unwrap();
    eprintln!("classified in {:?}: {} material cells, {} removed cells",started.elapsed(),kept.len(),removed.len());
    for cell in kept.iter().chain(&removed) {
        eprintln!("  cell volume {:.6} margin {:.4} at {:?}",cell.volume,cell.margin,cell.point);
    }
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    assert!((cell_total-blank_volume).abs() < 1e-6*blank_volume);
    assert_eq!(removed.len(),1,"one tooth space");
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    eprintln!("single-space part volume {:.6} of blank {:.6}; tolerances {:?}; {} faces",
        cad.0.volume(part).unwrap(),blank_volume,cad.0.tolerances(part).unwrap(),cad.0.faces(part).unwrap().len());

    // Independent check: the declared material field against the kernel's
    // classification at points near the generated surface and elsewhere.
    let mut queries = Vec::new();
    let (mut bounds_lo,mut bounds_hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for p in &sheet.points { for k in 0..3 { bounds_lo[k] = bounds_lo[k].min(p[k]); bounds_hi[k] = bounds_hi[k].max(p[k]); } }
    let n = 4;
    for i in 0..n { for j in 0..n { for k in 0..n {
        let f = [(i as f64+0.37)/n as f64,(j as f64+0.61)/n as f64,(k as f64+0.23)/n as f64];
        queries.push(std::array::from_fn(|a| bounds_lo[a]+(bounds_hi[a]-bounds_lo[a])*f[a]));
    } } }
    let states = cad.0.solid_contains(part,&queries,1e-6).unwrap();
    let started = std::time::Instant::now();
    let (mut agree,mut disagree,mut unresolved) = (0,0,0);
    for (p,state) in queries.iter().zip(&states) {
        let bounds = material.bounds(p.map(|x| Interval::point(x).unwrap()),
            gcs_core::interval::minimum::Options {value_tolerance:1e-4,max_evaluations:20000}).unwrap();
        let [lo,hi] = bounds.value.bounds();
        if hi < -0.02 || lo > 0.02 {
            if (hi < 0.) == (*state == 1) { agree += 1; } else { disagree += 1; eprintln!("  disagree at {p:?}: field [{lo},{hi}], kernel {state}"); }
        } else { unresolved += 1; }
    }
    eprintln!("field/kernel agreement in {:?}: {agree} agree, {disagree} disagree, {unresolved} within 0.02 mm of a boundary or unresolved",started.elapsed());
    assert_eq!(disagree,0);
}

#[test]
#[ignore = "about three minutes: builds and exports the whole pinion to a temporary directory"]
fn whole_pinion_from_indexed_sheets_and_material_cells() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named("pair.pinion.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.pinion.blank").unwrap().i();
    let body_id = e.map.ent_named("pair.pinion.body").unwrap().i();
    let teeth = 24;
    let indexing = gcs_core::motion::Family::read(&e.sketch,e.map.ent_named("pair.reference.pinion_index").unwrap().i()).unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();

    let started = std::time::Instant::now();
    let wide = [-std::f64::consts::PI,std::f64::consts::PI];
    let plan = [(1,34,[0.2,1.]),(2,8,[0.,1.]),(3,4,[0.,1.]),(4,7,[0.,1.]),(5,36,[0.,0.8])];
    let sheet = tooth_sheet(&sweep,&plan,[-0.21,-0.09],48,wide);
    let face = cad.fit_with(&sheet.points,sheet.rows,sheet.columns,1).unwrap();
    let tools: Vec<_> = (0..teeth).map(|i| {
        let pose = indexing.at(i as f64*std::f64::consts::TAU/teeth as f64).unwrap();
        cad.0.place(face,pose,scale).unwrap()
    }).collect();
    eprintln!("{} indexed sheets in {:?}",tools.len(),started.elapsed());
    let started = std::time::Instant::now();
    let partition = cad.0.split_solid(blank,&tools).unwrap();
    eprintln!("split in {:?}; partition tolerances {:?}",started.elapsed(),cad.0.tolerances(partition).unwrap());
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,body_id,1e-10).unwrap().evaluator(4096);
    let started = std::time::Instant::now();
    let (kept,removed) = super::cells::classify(&cad,partition,&mut material).unwrap();
    eprintln!("classified in {:?}: {} material cells, {} removed cells",started.elapsed(),kept.len(),removed.len());
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    assert!((cell_total-blank_volume).abs() < 1e-6*blank_volume);
    assert_eq!(removed.len(),teeth,"one removed cell per tooth space");
    let spaces: Vec<f64> = removed.iter().map(|c| c.volume).collect();
    let (lo,hi) = spaces.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),v| (lo.min(*v),hi.max(*v)));
    eprintln!("tooth space volumes in [{lo:.6},{hi:.6}] mm^3");
    assert!(hi-lo < 1e-6*hi,"indexed spaces are congruent");
    let started = std::time::Instant::now();
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    let volume = cad.0.volume(part).unwrap();
    eprintln!("pinion in {:?}: volume {volume:.6} mm^3 of blank {blank_volume:.6}; tolerances {:?}; {} faces",
        started.elapsed(),cad.0.tolerances(part).unwrap(),cad.0.faces(part).unwrap().len());
    let out = std::env::temp_dir().join(format!("solvent-pinion-{}",std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let step = out.join("pinion.step"); let stl = out.join("pinion.stl");
    let started = std::time::Instant::now();
    cad.0.step(part,step.to_str().unwrap()).unwrap();
    cad.0.stl(part,stl.to_str().unwrap()).unwrap();
    let bytes = std::fs::read(&stl).unwrap();
    gcs_core::mesh::stl_shells(&bytes).unwrap();
    eprintln!("exported {} and {} ({} bytes STL) in {:?}",step.display(),stl.display(),bytes.len(),started.elapsed());
}

#[test]
#[ignore = "exploration: gear blank construction variants"]
fn explore_gear_blank_construction() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let cad = Cad::new();
    let solid = |name: &str| {
        let id = e.map.ent_named(name).unwrap().i();
        cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,id).unwrap()).unwrap()
    };
    let (heel,toe,tip,back) = (solid("pair.reference.heel.carrier"),solid("pair.reference.toe.carrier"),
        solid("pair.reference.gear_tip_boundary.carrier"),solid("pair.reference.gear_back_boundary.carrier"));
    // Arrangement instead of a Boolean: the cone's faces split the sphere.
    for (label,tools) in [("tip faces",cad.0.faces(tip).unwrap()),("toe faces",cad.0.faces(toe).unwrap()),("back faces",cad.0.faces(back).unwrap())] {
        match cad.0.split_solid(heel,&tools) {
            Ok(partition) => {
                let cells = cad.0.cells(partition).unwrap();
                eprintln!("heel split by {label}: {} cells, volumes {:?}, tolerances {:?}",cells.len(),
                    cells.iter().map(|c| (c.volume*1e3).round()/1e3).collect::<Vec<_>>(),cad.0.tolerances(partition).unwrap());
            }
            Err(error) => eprintln!("heel split by {label} failed: {error}"),
        }
    }
    for (label,tools,fuzzy) in [("tip solid",vec![tip],0.),("tip faces fuzzy 1e-5",cad.0.faces(tip).unwrap(),1e-5),("tip faces fuzzy 1e-3",cad.0.faces(tip).unwrap(),1e-3)] {
        match cad.0.split_solid_fuzzy(heel,&tools,fuzzy) {
            Ok(partition) => {
                let cells = cad.0.cells(partition).unwrap();
                eprintln!("heel split by {label}: {} cells, volumes {:?}, tolerances {:?}",cells.len(),
                    cells.iter().map(|c| (c.volume*1e3).round()/1e3).collect::<Vec<_>>(),cad.0.tolerances(partition).unwrap());
            }
            Err(error) => eprintln!("heel split by {label} failed: {error}"),
        }
    }
    match cad.0.common(heel,tip) {
        Ok(inside) => {
            eprintln!("heel ∩ tip: volume {:.3}",cad.0.volume(inside).unwrap());
            match cad.0.cut(inside,toe).and_then(|s| cad.0.cut(s,back)) {
                Ok(blank) => eprintln!("gear blank by intersection: volume {:.3}, tolerances {:?}",cad.0.volume(blank).unwrap(),cad.0.tolerances(blank).unwrap()),
                Err(error) => eprintln!("gear blank by intersection failed: {error}"),
            }
        }
        Err(error) => eprintln!("heel ∩ tip failed: {error}"),
    }
    for name in ["pair.reference.heel.carrier","pair.reference.toe.carrier",
        "pair.reference.gear_tip_boundary.carrier","pair.reference.gear_back_boundary.carrier",
        "pair.reference.pinion_tip_boundary.carrier","pair.gear.outside_tip","pair.gear.blank"] {
        let id = e.map.ent_named(name).unwrap().i();
        match gcs_core::solid::cad::recipe(&e.sketch,id).and_then(|r| cad.0.construct(&r)) {
            Ok(solid) => eprintln!("{name}: volume {:.3}, tolerances {:?}",cad.0.volume(solid).unwrap(),cad.0.tolerances(solid).unwrap()),
            Err(error) => eprintln!("{name}: {error}"),
        }
    }
}

#[test]
#[ignore = "exploration: cutter/blank overlap against roll angle"]
fn explore_roll_needed_to_clear_each_blank() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let cad = Cad::new();
    let scale = e.sketch.units.length.unwrap().1;
    for member in ["pinion","gear"] {
        let removal = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let blank = e.map.ent_named(&format!("pair.{member}.blank")).unwrap().i();
        let stock = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank).unwrap()).unwrap();
        let caps = cad.0.sweep_caps(&e.sketch,removal).unwrap();
        let gcs_core::model::SolidDef::Swept {motion,..} = &e.sketch.solids[removal].def else { panic!() };
        let motion = gcs_core::motion::Family::read(&e.sketch,*motion as usize).unwrap();
        let mut line = format!("{member}:");
        for degrees in [-60,-50,-45,-40,-35,-30,0,30,35,40,45,50,60] {
            let pose = motion.at(degrees as f64*std::f64::consts::PI/180.).unwrap();
            let placed = cad.0.place(caps.source,pose,scale).unwrap();
            line += &format!(" {degrees}deg:{:.3}",cad.0.common_volume(placed,stock).unwrap());
        }
        eprintln!("{line}");
    }
}

/// Where a sheet's declared-roll contacts enter the blank, in seam-centred v and
/// per-patch u, so a band and u ranges can be chosen with margin.
fn report_reach(cad: &Cad,blank: c_int,sheet: &Sheet,roll: [f64;2],label: &str) {
    let declared: Vec<_> = sheet.nodes.iter().zip(&sheet.points).filter(|(n,_)| n.3 >= roll[0] && n.3 <= roll[1]).collect();
    let states = cad.0.solid_contains(blank,&declared.iter().map(|(_,p)| **p).collect::<Vec<_>>(),1e-6).unwrap();
    let inside: Vec<_> = declared.iter().zip(&states).filter(|(_,s)| **s == 1).map(|((n,_),_)| **n).collect();
    let (mut vlo,mut vhi,mut tlo,mut thi) = (f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,f64::NEG_INFINITY);
    let mut u_range = std::collections::BTreeMap::new();
    for (patch,u,v,t) in &inside {
        let v = if *v > 0.5 { v-1. } else { *v };
        vlo = vlo.min(v); vhi = vhi.max(v); tlo = tlo.min(*t); thi = thi.max(*t);
        let entry = u_range.entry(*patch).or_insert((f64::INFINITY,f64::NEG_INFINITY));
        entry.0 = entry.0.min(*u); entry.1 = entry.1.max(*u);
    }
    eprintln!("{label}: {} of {} declared-roll nodes inside the blank: v in [{vlo:.3},{vhi:.3}], time in [{tlo:.3},{thi:.3}], u per patch {u_range:?}",
        inside.len(),declared.len());
}

#[test]
#[ignore = "exploration: per-patch reach of the gear cutter into its blank"]
fn explore_gear_sheet_reach() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named("pair.gear.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.gear.blank").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let roll = sweep.domain();
    let names: Vec<_> = sweep.patches().iter().map(|p| p.name.rsplit('.').next().unwrap().to_string()).collect();
    eprintln!("gear patches {names:?}, roll {roll:?}");
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let caps = cad.0.sweep_caps(&e.sketch,removal).unwrap();
    for cap in &caps.endpoints {
        eprintln!("endpoint {:.3}: common volume {:.6}",cap.parameter,cad.0.common_volume(cap.solid,blank).unwrap());
    }
    let wide = [-std::f64::consts::PI,std::f64::consts::PI];
    let band = [-0.45,0.5];
    for (patch,name) in [(2,"A.upper"),(3,"A.inner_round"),(4,"A.inner"),(6,"B.outer"),(7,"B.outer_round"),(8,"B.upper")] {
        let sheet = tooth_sheet(&sweep,&[(patch,24,[0.,1.])],band,64,wide);
        report_reach(&cad,blank,&sheet,roll,name);
    }
}

/// Invert a revolved patch's static chart at a source point: (u,v) with
/// `patch.at(u,v).position == target`, by Gauss-Newton from a guess. u is
/// clamped to the annulus; the caller checks the residual.
fn invert(patch: &gcs_core::solid::RevolvedSurface,target: [f64;3],guess: [f64;2]) -> [f64;2] {
    let mut x = guess;
    for _ in 0..30 {
        let p = patch.at(x[0],x[1].rem_euclid(1.)).unwrap();
        let r: [f64;3] = std::array::from_fn(|k| p.position[k]-target[k]);
        if r.iter().map(|v| v*v).sum::<f64>().sqrt() < 1e-12 { break; }
        let (du,dv) = (p.du,p.dv);
        let (a,b,c) = (dot(du,du),dot(du,dv),dot(dv,dv));
        let (ru,rv) = (dot(du,r),dot(dv,r));
        let det = a*c-b*b;
        x[0] -= (c*ru-b*rv)/det; x[1] -= (a*rv-b*ru)/det;
        x[0] = x[0].clamp(0.,1.);
    }
    x
}
fn dot(a: [f64;3],b: [f64;3]) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }

/// A sheet over given rows (patch, u) and an explicit revolution angle per
/// column, contact times by continuity outward from the centre column.
fn sheet_with_columns(sweep: &SweepContacts,rows: &[(usize,f64)],columns: &[f64],interval: [f64;2]) -> Sheet {
    let centre = columns.len()/2;
    let mut points = Vec::new(); let mut nodes = Vec::new();
    for &(patch,u) in rows {
        let mut line: Vec<Option<(f64,[f64;3])>> = vec![None;columns.len()];
        for (order,c) in (centre..columns.len()).chain((0..centre).rev()).enumerate() {
            let v = columns[c].rem_euclid(1.);
            let roots = sweep.at_source_over(patch,u,v,interval,1e-9).unwrap();
            assert!(!roots.is_empty(),"patch {patch} u={u} v={v}: no contact time");
            let previous = if order == 0 { 0. } else if c > centre { line[c-1].unwrap().0 } else { line[c+1].unwrap().0 };
            let root = roots.iter().min_by(|x,y| (x.root.time-previous).abs().total_cmp(&(y.root.time-previous).abs())).unwrap();
            line[c] = Some((root.root.time,root.contact.position));
        }
        for (c,entry) in line.iter().enumerate() {
            let (time,position) = entry.unwrap();
            points.push(position); nodes.push((patch,u,columns[c].rem_euclid(1.),time));
        }
    }
    Sheet {points,rows:rows.len(),columns:columns.len(),nodes}
}

/// Kernel against field on both sides of every sheet node inside the blank:
/// the cutter side must be removed and the other side material wherever the
/// field is decisive. This catches a leaked or misplaced sheet directly.
fn check_sides(cad: &Cad,part: c_int,blank: c_int,sweep: &SweepContacts,sheet: &Sheet,interval: [f64;2],
    material: &mut gcs_core::solid::MaterialEvaluator,offset: f64) -> (usize,usize,usize) {
    let inside = cad.0.solid_contains(blank,&sheet.points,1e-6).unwrap();
    let mut queries = Vec::new();
    for (i,(node,state)) in sheet.nodes.iter().zip(&inside).enumerate() {
        if *state != 1 || i % 7 != 0 { continue; }
        let mut roots = sweep.at_source_over(node.0,node.1,node.2,interval,1e-9).unwrap();
        roots.sort_by(|a,b| (a.root.time-node.3).abs().total_cmp(&(b.root.time-node.3).abs()));
        let contact = roots[0].contact;
        for sign in [-1.,1.] { queries.push(std::array::from_fn(|k| contact.position[k]+sign*offset*contact.normal[k])); }
    }
    let states = cad.0.solid_contains(part,&queries,1e-6).unwrap();
    let (mut agree,mut disagree,mut undecided) = (0,0,0);
    for (p,state) in queries.iter().zip(&states) {
        let bounds = material.bounds(p.map(|x| Interval::point(x).unwrap()),
            gcs_core::interval::minimum::Options {value_tolerance:offset/4.,max_evaluations:20000}).unwrap();
        let [lo,hi] = bounds.value.bounds();
        if hi < 0. || lo > 0. {
            if (hi < 0.) == (*state == 1) { agree += 1; } else { disagree += 1;
                if disagree <= 5 { eprintln!("  disagree at {p:?}: field [{lo:.4},{hi:.4}], kernel {state}"); } }
        } else { undecided += 1; }
    }
    eprintln!("sides of {} inside nodes: {agree} agree, {disagree} disagree, {undecided} undecided by the field",queries.len()/2);
    (agree,disagree,undecided)
}

#[test]
#[ignore = "records the failed two-sheet construction: tangential sheets leave an unorientable fragment"]
fn gear_tooth_space_from_two_aligned_sheets() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named("pair.gear.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.gear.blank").unwrap().i();
    let single_id = e.map.ent_named("pair.gear.single").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let roll = sweep.domain();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();
    let wide = [-std::f64::consts::PI,std::f64::consts::PI];
    // Columns: the neighbour ring's angle, and the outer ring's angle of the
    // neighbour's junction point, so both sheets' rows stay radially aligned.
    let n = 64;
    let band_b: [f64;2] = [0.09,0.22];
    let v_b: Vec<f64> = (0..=n).map(|c| band_b[0]+(band_b[1]-band_b[0])*c as f64/n as f64).collect();
    let v_a: Vec<f64> = v_b.iter().map(|&vb| {
        let jb = sweep.patches()[7].at(1.,vb).unwrap().position;
        invert(&sweep.patches()[2],jb,[0.98,vb])[1]
    }).collect();
    let mut rows_a = Vec::new();
    for i in 0..=8 { rows_a.push((2,0.94+0.06*i as f64/8.)); }
    for i in 1..=12 { rows_a.push((3,i as f64/12.)); }
    for i in 1..=36 { rows_a.push((4,0.8*i as f64/36.)); }
    let mut rows_b = Vec::new();
    for i in 0..=36 { rows_b.push((6,0.2+0.8*i as f64/36.)); }
    for i in 1..=12 { rows_b.push((7,i as f64/12.)); }
    let started = std::time::Instant::now();
    let a = sheet_with_columns(&sweep,&rows_a,&v_a,wide);
    let b = sheet_with_columns(&sweep,&rows_b,&v_b,wide);
    eprintln!("sheets {}x{} and {}x{} in {:?}",a.rows,a.columns,b.rows,b.columns,started.elapsed());
    report_reach(&cad,blank,&a,roll,"A");
    report_reach(&cad,blank,&b,roll,"B");
    let faces = [cad.fit_with(&a.points,a.rows,a.columns,1).unwrap(),cad.fit_with(&b.points,b.rows,b.columns,1).unwrap()];
    let mut worst = 0_f64; let mut off = 0;
    for c in 0..b.columns {
        let p = b.points[(b.rows-1)*b.columns+c];
        match cad.0.face_parameters(faces[0],p,0.05).unwrap() { Some((_,gap)) => worst = worst.max(gap), None => off += 1 }
    }
    eprintln!("B junction on A: worst gap {worst:e} mm, {off} of {} columns not within 0.05 mm",b.columns);
    for (label,tools,fuzzy) in [("A",vec![faces[0]],1e-5),("B",vec![faces[1]],1e-5),("A+B 1e-5",faces.to_vec(),1e-5),("A+B 1e-4",faces.to_vec(),1e-4),("A+B 1e-3",faces.to_vec(),1e-3)] {
        let started = std::time::Instant::now();
        match cad.0.split_solid_fuzzy(blank,&tools,fuzzy) {
            Ok(partition) => eprintln!("split by {label} in {:?}: {} cells, tolerances {:?}",started.elapsed(),
                cad.0.solids(partition).unwrap().len(),cad.0.tolerances(partition).unwrap()),
            Err(error) => eprintln!("split by {label} failed: {error}"),
        }
    }
    let started = std::time::Instant::now();
    let partition = cad.0.split_solid_fuzzy(blank,&faces,1e-3).unwrap();
    eprintln!("split in {:?}; partition tolerances {:?}",started.elapsed(),cad.0.tolerances(partition).unwrap());
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,single_id,1e-10).unwrap().evaluator(4096);
    let started = std::time::Instant::now();
    let (kept,removed) = super::cells::classify(&cad,partition,&mut material).unwrap();
    eprintln!("classified in {:?}: {} material cells, {} removed cells",started.elapsed(),kept.len(),removed.len());
    for cell in kept.iter().chain(&removed) {
        eprintln!("  cell volume {:.6} margin {:.4} at {:?}",cell.volume,cell.margin,cell.point);
    }
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    assert!((cell_total-blank_volume).abs() < 1e-6*blank_volume);
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    eprintln!("single-space gear volume {:.6} of blank {blank_volume:.6}; tolerances {:?}; {} faces",
        cad.0.volume(part).unwrap(),cad.0.tolerances(part).unwrap(),cad.0.faces(part).unwrap().len());
    let (_,da,_) = check_sides(&cad,part,blank,&sweep,&a,wide,&mut material,0.1);
    let (_,db,_) = check_sides(&cad,part,blank,&sweep,&b,wide,&mut material,0.1);
    assert_eq!(da+db,0);
    assert_eq!(removed.len(),1,"one tooth space");
}

/// Newton with a finite-difference Jacobian on a small square system.
fn newton<const N: usize>(mut x: [f64;N],f: &dyn Fn([f64;N])->[f64;N]) -> Option<[f64;N]> {
    for _ in 0..40 {
        let r = f(x);
        let norm = r.iter().map(|v| v*v).sum::<f64>().sqrt();
        if norm < 1e-11 { return Some(x); }
        let mut j = [[0.;N];N];
        for k in 0..N {
            let h = 1e-7;
            let mut xh = x; xh[k] += h;
            let rh = f(xh);
            for i in 0..N { j[i][k] = (rh[i]-r[i])/h; }
        }
        // Gaussian elimination.
        let mut a = j; let mut b = r;
        for k in 0..N {
            let pivot = (k..N).max_by(|&p,&q| a[p][k].abs().total_cmp(&a[q][k].abs())).unwrap();
            a.swap(k,pivot); b.swap(k,pivot);
            if a[k][k].abs() < 1e-14 { return None; }
            for i in k+1..N {
                let m = a[i][k]/a[k][k];
                for c in k..N { a[i][c] -= m*a[k][c]; }
                b[i] -= m*b[k];
            }
        }
        let mut d = [0.;N];
        for i in (0..N).rev() {
            let mut sum = b[i];
            for c in i+1..N { sum -= a[i][c]*d[c]; }
            d[i] = sum/a[i][i];
        }
        for k in 0..N { x[k] -= d[k]; }
    }
    None
}
/// A revolved patch evaluated with linear extrapolation in u past its arc, so a
/// root solver near an arc end keeps a derivative; the caller checks the result.
fn at_extended(patch: &gcs_core::solid::RevolvedSurface,u: f64,v: f64) -> [f64;3] {
    let v = v.rem_euclid(1.);
    if (0. ..=1.).contains(&u) { return patch.at(u,v).unwrap().position; }
    let edge = u.clamp(0.,1.);
    let p = patch.at(edge,v).unwrap();
    std::array::from_fn(|k| p.position[k]+(u-edge)*p.du[k])
}
fn normalize3(a: [f64;3]) -> [f64;3] { let n = dot(a,a).sqrt(); a.map(|v| v/n) }
fn cross3(a: [f64;3],b: [f64;3]) -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }

/// The gear tooth-space sheet as one profile walk per column: the neighbour's
/// flank and round, the space bottom, the outer crown's round and flank.
/// Where the two rounds are apart the bottom is the shared tip plane's
/// envelope between their junctions; where they cross it is the sweep of their
/// sharp intersection edge, the fan of normals between the two rounds. Both
/// are tangent-continuous with the rounds, and they meet continuously where
/// the bottom width vanishes. Contact times follow continuity down the profile.
fn gear_space_sheet(sweep: &SweepContacts,band_b: [f64;2],columns: usize,interval: [f64;2]) -> Sheet {
    let (tip,a_round,a_flank,b_flank,b_round) = (2,3,4,6,7);
    let patches = sweep.patches();
    let (nf,nr,nb) = (36,12,12);
    let rows = (nf+1)+nr+(nb-1)+(nr+1)+nf;
    let mut grid: Vec<Vec<([f64;3],(usize,f64,f64,f64))>> = Vec::new();
    let mut fan_sign: Option<[f64;2]> = None;
    let mut widths = Vec::new();
    for c in 0..=columns {
        let vb = (band_b[0]+(band_b[1]-band_b[0])*c as f64/columns as f64).rem_euclid(1.);
        // The neighbour's junction in the outer crown's tip chart, extended
        // linearly past the annulus edge: t is the radial distance from the
        // outer junction toward the far edge, negative inside it.
        let jb = patches[b_round].at(1.,vb).unwrap().position;
        let ray = |v: f64| { let p1 = patches[tip].at(1.,v.rem_euclid(1.)).unwrap().position; let p0 = patches[tip].at(0.,v.rem_euclid(1.)).unwrap().position; (p1,p0) };
        let located = newton([0.01,vb],&|x: [f64;2]| {
            let (p1,p0) = ray(x[1]);
            let q: [f64;3] = std::array::from_fn(|k| p1[k]+x[0]*(p0[k]-p1[k]));
            // Two equations: distance along the ray and across it, in the plane.
            let (d,n) = (dot([q[0]-jb[0],q[1]-jb[1],q[2]-jb[2]],normalize3([p0[0]-p1[0],p0[1]-p1[1],p0[2]-p1[2]])),
                { let across = cross3(normalize3([p0[0]-p1[0],p0[1]-p1[1],p0[2]-p1[2]]),normalize3(cross3(patches[tip].at(0.5,x[1].rem_euclid(1.)).unwrap().du,patches[tip].at(0.5,x[1].rem_euclid(1.)).unwrap().dv)));
                  dot([q[0]-jb[0],q[1]-jb[1],q[2]-jb[2]],across) });
            [d,n]
        }).expect("tip chart location");
        let (t_b,va) = (located[0],located[1].rem_euclid(1.));
        widths.push(t_b);
        let mut column: Vec<(usize,f64,f64,Option<([f64;3],[f64;3])>)> = Vec::new(); // (patch,u,v,point-normal)
        for i in 0..=nf { column.push((b_flank,0.2+0.8*i as f64/nf as f64,vb,None)); }
        if t_b >= 0. {
            for i in 1..=nr { column.push((b_round,i as f64/nr as f64,vb,None)); }
            for k in 1..nb { let t = t_b*(1.-k as f64/nb as f64); column.push((tip,1.-t,va,None)); }
            for i in 0..=nr { column.push((a_round,i as f64/nr as f64,va,None)); }
        } else {
            // The rounds cross: find the crossing on the neighbour's meridian.
            let x = newton([0.99,0.01,va],&|x: [f64;3]| {
                let p = at_extended(&patches[b_round],x[0],vb);
                let q = at_extended(&patches[a_round],x[1],x[2]);
                std::array::from_fn(|k| p[k]-q[k])
            }).expect("round crossing");
            let (ub,ua,vx) = (x[0],x[1],x[2].rem_euclid(1.));
            assert!((0. ..=1.).contains(&ub) && (0. ..=1.).contains(&ua),"column {c}: rounds cross outside their arcs at u_b={ub}, u_a={ua}");
            let pb = patches[b_round].at(ub,vb).unwrap(); let pa = patches[a_round].at(ua,vx).unwrap();
            let reference = normalize3(cross3(patches[tip].at(1.,va).unwrap().du,patches[tip].at(1.,va).unwrap().dv));
            let (mut nb_,mut na_) = (normalize3(cross3(pb.du,pb.dv)),normalize3(cross3(pa.du,pa.dv)));
            let sign = *fan_sign.get_or_insert([dot(nb_,reference).signum(),dot(na_,reference).signum()]);
            nb_ = nb_.map(|v| v*sign[0]); na_ = na_.map(|v| v*sign[1]);
            for i in 1..=nr { column.push((b_round,ub*i as f64/nr as f64,vb,None)); }
            for k in 1..nb {
                let f = k as f64/nb as f64;
                let n = normalize3(std::array::from_fn(|j| (1.-f)*nb_[j]+f*na_[j]));
                column.push((usize::MAX,0.,0.,Some((pb.position,n))));
            }
            for i in 0..=nr { column.push((a_round,ua+(1.-ua)*i as f64/nr as f64,vx,None)); }
        }
        for i in 1..=nf { column.push((a_flank,0.8*i as f64/nf as f64,column.last().unwrap().2,None)); }
        assert_eq!(column.len(),rows);
        let mut previous = 0.;
        let mut result = Vec::new();
        for (patch,u,v,pn) in column {
            let roots = match pn {
                None => sweep.at_source_over(patch,u,v,interval,1e-9).unwrap(),
                Some((p,n)) => sweep.at_point_normal_over(p,n,interval,1e-9).unwrap(),
            };
            assert!(!roots.is_empty(),"column {c}: no contact time on patch {patch} u={u} v={v}");
            let root = roots.iter().min_by(|x,y| (x.root.time-previous).abs().total_cmp(&(y.root.time-previous).abs())).unwrap();
            previous = root.root.time;
            result.push((root.contact.position,(patch,u,v,root.root.time)));
        }
        grid.push(result);
    }
    eprintln!("bottom width t per column (negative = rounds crossed): first {:.4}, min {:.4}, last {:.4}, crossing columns {}",
        widths[0],widths.iter().cloned().fold(f64::INFINITY,f64::min),widths[widths.len()-1],widths.iter().filter(|w| **w < 0.).count());
    let mut points = Vec::with_capacity(rows*(columns+1)); let mut nodes = Vec::new();
    for r in 0..rows { for column in &grid { points.push(column[r].0); nodes.push(column[r].1); } }
    Sheet {points,rows,columns:columns+1,nodes}
}

#[test]
fn gear_tooth_space_from_one_sheet_with_strip_or_fan_bottom() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named("pair.gear.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.gear.blank").unwrap().i();
    let single_id = e.map.ent_named("pair.gear.single").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let roll = sweep.domain();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();
    let wide = [-std::f64::consts::PI,std::f64::consts::PI];
    let started = std::time::Instant::now();
    let sheet = gear_space_sheet(&sweep,[0.09,0.22],64,wide);
    let mut jumps = 0_f64; let mut row_step = 0_f64;
    for (i,n) in sheet.nodes.iter().enumerate() {
        if i % sheet.columns != 0 { jumps = jumps.max((n.3-sheet.nodes[i-1].3).abs()); }
        if i >= sheet.columns { row_step = row_step.max(distance(sheet.points[i],sheet.points[i-sheet.columns])); }
    }
    eprintln!("sheet {}x{} in {:?}; largest time step along a row {jumps:.3}, largest step between rows {row_step:.3} mm",
        sheet.rows,sheet.columns,started.elapsed());
    report_reach(&cad,blank,&sheet,roll,"gear sheet");
    let mut boundary = Vec::new();
    for r in 0..sheet.rows { for c in 0..sheet.columns {
        if r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns { boundary.push(sheet.points[r*sheet.columns+c]); }
    } }
    let inside = cad.0.solid_contains(blank,&boundary,1e-6).unwrap().iter().filter(|s| **s == 1).count();
    assert_eq!(inside,0,"sheet boundary nodes inside the blank");
    let face = cad.fit_with(&sheet.points,sheet.rows,sheet.columns,1).unwrap();
    let started = std::time::Instant::now();
    let partition = cad.0.split_solid(blank,&[face]).unwrap();
    eprintln!("split in {:?}; partition tolerances {:?}",started.elapsed(),cad.0.tolerances(partition).unwrap());
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,single_id,1e-10).unwrap().evaluator(4096);
    let started = std::time::Instant::now();
    let (kept,removed) = super::cells::classify(&cad,partition,&mut material).unwrap();
    eprintln!("classified in {:?}: {} material cells, {} removed cells",started.elapsed(),kept.len(),removed.len());
    for cell in kept.iter().chain(&removed) {
        eprintln!("  cell volume {:.6} margin {:.4} at {:?}",cell.volume,cell.margin,cell.point);
    }
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    assert!((cell_total-blank_volume).abs() < 1e-6*blank_volume);
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    eprintln!("single-space gear volume {:.6} of blank {blank_volume:.6}; tolerances {:?}; {} faces",
        cad.0.volume(part).unwrap(),cad.0.tolerances(part).unwrap(),cad.0.faces(part).unwrap().len());
    let (_,disagree,_) = check_sides(&cad,part,blank,&sweep,&sheet,wide,&mut material,0.1);
    assert_eq!(disagree,0);
    assert_eq!(removed.len(),1,"one tooth space");
}

#[test]
#[ignore = "about fifteen minutes: builds and exports the whole gear to a temporary directory"]
fn whole_gear_from_indexed_sheets_and_material_cells() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let removal = e.map.ent_named("pair.gear.removal").unwrap().i();
    let blank_id = e.map.ent_named("pair.gear.blank").unwrap().i();
    let body_id = e.map.ent_named("pair.gear.body").unwrap().i();
    let teeth = 48;
    let indexing = gcs_core::motion::Family::read(&e.sketch,e.map.ent_named("pair.reference.gear_index").unwrap().i()).unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    let sweep = SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    let cad = Cad::new();
    let blank = cad.0.construct(&gcs_core::solid::cad::recipe(&e.sketch,blank_id).unwrap()).unwrap();
    let blank_volume = cad.0.volume(blank).unwrap();
    let started = std::time::Instant::now();
    let wide = [-std::f64::consts::PI,std::f64::consts::PI];
    let sheet = gear_space_sheet(&sweep,[0.09,0.22],64,wide);
    let face = cad.fit_with(&sheet.points,sheet.rows,sheet.columns,1).unwrap();
    let tools: Vec<_> = (0..teeth).map(|i| {
        let pose = indexing.at(i as f64*std::f64::consts::TAU/teeth as f64).unwrap();
        cad.0.place(face,pose,scale).unwrap()
    }).collect();
    eprintln!("{} indexed sheets in {:?}",tools.len(),started.elapsed());
    let started = std::time::Instant::now();
    let partition = cad.0.split_solid(blank,&tools).unwrap();
    eprintln!("split in {:?}; partition tolerances {:?}",started.elapsed(),cad.0.tolerances(partition).unwrap());
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,body_id,1e-10).unwrap().evaluator(4096);
    let started = std::time::Instant::now();
    let (kept,removed) = super::cells::classify(&cad,partition,&mut material).unwrap();
    eprintln!("classified in {:?}: {} material cells, {} removed cells",started.elapsed(),kept.len(),removed.len());
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    eprintln!("cells sum to {cell_total:.6} against blank {blank_volume:.6} ({:.2e} relative)",(cell_total-blank_volume).abs()/blank_volume);
    assert!((cell_total-blank_volume).abs() < 1e-5*blank_volume,"quadrature of the cells disagrees with the blank");
    assert_eq!(removed.len(),teeth,"one removed cell per tooth space");
    let spaces: Vec<f64> = removed.iter().map(|c| c.volume).collect();
    let (lo,hi) = spaces.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),v| (lo.min(*v),hi.max(*v)));
    eprintln!("tooth space volumes in [{lo:.6},{hi:.6}] mm^3");
    assert!(hi-lo < 1e-5*hi,"indexed spaces are congruent");
    let started = std::time::Instant::now();
    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    let volume = cad.0.volume(part).unwrap();
    eprintln!("gear in {:?}: volume {volume:.6} mm^3 of blank {blank_volume:.6}; tolerances {:?}; {} faces",
        started.elapsed(),cad.0.tolerances(part).unwrap(),cad.0.faces(part).unwrap().len());
    let out = std::env::temp_dir().join(format!("solvent-gear-{}",std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let step = out.join("gear.step"); let stl = out.join("gear.stl");
    let started = std::time::Instant::now();
    cad.0.step(part,step.to_str().unwrap()).unwrap();
    cad.0.stl(part,stl.to_str().unwrap()).unwrap();
    let bytes = std::fs::read(&stl).unwrap();
    gcs_core::mesh::stl_shells(&bytes).unwrap();
    eprintln!("exported {} and {} ({} bytes STL) in {:?}",step.display(),stl.display(),bytes.len(),started.elapsed());
}

#[test]
#[ignore = "exploration: a probe point against the single and full gear fields"]
fn explore_unresolved_gear_probe() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read_gears(&base);
    let body_id = e.map.ent_named("pair.gear.body").unwrap().i();
    let single_id = e.map.ent_named("pair.gear.single").unwrap().i();
    let p = [36.77593489977285,-34.439787711552185,-8.323568418324937];
    let indexing = gcs_core::motion::Family::read(&e.sketch,e.map.ent_named("pair.reference.gear_index").unwrap().i()).unwrap();
    let mut single = gcs_core::solid::MaterialField::read(&e.sketch,single_id,1e-10).unwrap().evaluator(4096);
    for k in 0..48 {
        let pose = indexing.at(k as f64*std::f64::consts::TAU/48.).unwrap();
        let q = pose.inverse().point(p);
        let bounds = single.bounds(q.map(|x| Interval::point(x).unwrap()),
            gcs_core::interval::minimum::Options {value_tolerance:0.01,max_evaluations:40000}).unwrap();
        let [lo,hi] = bounds.value.bounds();
        if lo > -0.5 { eprintln!("index {k}: single field at the rotated point [{lo:.4},{hi:.4}]"); }
    }
    for (label,id) in [("single",single_id),("body",body_id)] {
        let mut material = gcs_core::solid::MaterialField::read(&e.sketch,id,1e-10).unwrap().evaluator(4096);
        let started = std::time::Instant::now();
        let probe = material.probe(p.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],0.05,
            gcs_core::interval::minimum::Options {value_tolerance:0.0125,max_evaluations:40000}).unwrap();
        eprintln!("{label}: {:?} in {:?}; centre value {:?}",probe.state,started.elapsed(),probe.center.value.bounds());
        let mut statuses = std::collections::BTreeMap::new();
        for q in &probe.center.sweeps { *statuses.entry(format!("{:?}",q.minimum.status)).or_insert(0) += 1; }
        eprintln!("  {} sweep queries: {statuses:?}",probe.center.sweeps.len());
        for q in probe.center.sweeps.iter().filter(|q| q.minimum.value.bounds()[0] < 0.5) {
            eprintln!("  near sweep: value {:?} status {:?} evaluations {} domain {:?}",q.minimum.value.bounds(),q.minimum.status,q.minimum.evaluations,q.domain.bounds());
        }
    }
}
