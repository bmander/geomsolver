//! Derisking experiment: build a swept-solid boundary by arrangement and
//! classification. Candidate sheets (envelope charts and endpoint caps) split the
//! stock; every cell is classified by the declared material field; material cells
//! are united. The kernel arranges, the source decides, and no visibility or trim
//! logic is written here.
use super::*;
use gcs_core::{interval::{Interval,minimum::Options},motion::Family,
    solid::{cad,MaterialField,ProbeState}};
use std::f64::consts::PI;

const STOCK: &str = "
private point r0 hint(x: 0, y: -0.8)
private point r1 hint(x: 3.5, y: -0.8)
private point r2 hint(x: 3.5, y: 0.8)
private point r3 hint(x: 0, y: 0.8)
ground r0
ground r1
ground r2
ground r3
private line e0(r0, r1)
private line e1(r1, r2)
private line e2(r2, r3)
private line e3(r3, r0)
construction solid stock(face(e0, e1, e2, e3), about: e3)
solid part(stock)
removal.body cut part
";

/// Classify every cell of a partition against the material field. A cell is
/// judged at several interior points with measured boundary distances; every
/// point needs a ball certificate within that distance and all must agree, so
/// an unresolved point or a cell that a leaking sheet failed to separate
/// refuses the build instead of guessing.
pub(super) fn classify(cad: &Cad,partition: c_int,material: &mut gcs_core::solid::MaterialEvaluator)
    -> Result<(Vec<native::cells::Cell>,Vec<native::cells::Cell>),String> {
    let (mut kept,mut removed) = (Vec::new(),Vec::new());
    for cell in cad.0.cells(partition)? {
        let samples = cad.0.samples(cell.solid,4,12)?;
        if samples.is_empty() { return Err(format!("cell of volume {} has no interior sample",cell.volume)); }
        let mut verdict = None;
        for (point,boundary) in samples {
            // A ball certificate only needs the field resolved to the probe
            // distance, which stays inside the measured boundary distance.
            let distance = (boundary*0.5).min(0.05);
            if distance <= 1e-4 { continue; }
            let probe = material.probe(point.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
                Options {value_tolerance:distance/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
            let inside = match probe.state {
                ProbeState::InteriorBall => true,
                ProbeState::ExteriorBall => false,
                state => return Err(format!("cell at {point:?} (boundary distance {boundary}, volume {}) is {state:?}",cell.volume)),
            };
            match verdict {
                None => verdict = Some(inside),
                Some(previous) if previous != inside => return Err(format!(
                    "cell of volume {} reads both material and removed: a sheet did not separate it",cell.volume)),
                _ => {}
            }
        }
        match verdict {
            Some(true) => kept.push(cell),
            Some(false) => removed.push(cell),
            None => return Err(format!("cell of volume {} has no sample clear of its boundary",cell.volume)),
        }
    }
    Ok((kept,removed))
}

#[test]
fn sphere_sweep_solid_from_sheets_caps_and_material_cells() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let source = format!("{}{STOCK}",include_str!("../../../examples/solid_generating_sweep.sv"));
    let e = read(&source,&base);
    let sweep_id = e.map.ent_named("removal.body").unwrap().i();
    let stock_id = e.map.ent_named("stock").unwrap().i();
    let part_id = e.map.ent_named("part").unwrap().i();
    let sweep = SweepContacts::read(&e.sketch,sweep_id,1e-10).unwrap();
    let motion = Family::read(&e.sketch,e.map.ent_named("generating").unwrap().i()).unwrap();
    let [roll0,roll1] = sweep.domain();
    assert_eq!(sweep.patches().len(),1);
    // Orientation facts the independent membership formula below relies on.
    let pose = motion.at(roll1).unwrap();
    assert!(distance(pose.point([0.;3]),[0.;3]) < 1e-12);
    assert!(distance(pose.vector([0.,0.,1.]),[0.,0.,1.]) < 1e-12,"the spindle is the world z axis");
    assert!(distance(sweep.patches()[0].at(0.5,0.).unwrap().position,[4.,0.,0.]) < 1e-9);
    let center = pose.point([3.,0.,0.]);
    assert!((center[1].atan2(center[0]).abs()-PI/3.).abs() < 1e-12);

    // Envelope sheets: one per contact branch over (meridian, roll). The source
    // poles are regular points of the envelope, so their rows come from the
    // moved source point rather than the degenerate parameterization.
    let cad = Cad::new();
    let (nu,nr) = (32,32);
    let mut sheets = Vec::new();
    for branch in 0..2 {
        let mut points = Vec::with_capacity((nu+1)*(nr+1));
        let mut worst_step = 0_f64;
        for i in 0..=nu {
            let u = i as f64/nu as f64;
            for j in 0..=nr {
                let roll = roll0+(roll1-roll0)*j as f64/nr as f64;
                let p = if i == 0 || i == nu {
                    let source = sweep.patches()[0].at(u,if branch == 0 { 0. } else { 0.5 }).unwrap().position;
                    motion.at(roll).unwrap().point(source)
                } else {
                    sweep.at(0,u,roll,1e-10).unwrap().into_iter().find(|c| c.branch == branch)
                        .unwrap_or_else(|| panic!("branch {branch} absent at u={u} roll={roll}")).contact.position
                };
                if j > 0 { worst_step = worst_step.max(distance(p,points[points.len()-1])); }
                points.push(p);
            }
        }
        assert!(worst_step < 0.3,"sheet {branch} rows are continuous: {worst_step}");
        // Every sheet point lies on the torus the sweep is known to generate.
        for p in &points {
            let tube = (p[0].hypot(p[1])-3.).hypot(p[2]);
            assert!((tube-1.).abs() < 1e-9,"sheet point off the torus by {}",(tube-1.).abs());
        }
        sheets.push(cad.fit(&points,nu+1,nr+1).unwrap());
    }
    // Endpoint caps are the declared cutter placed at both roll limits.
    let caps = cad.0.sweep_caps(&e.sketch,sweep_id).unwrap();
    let mut tools = sheets.clone();
    for cap in &caps.endpoints { tools.extend(cap.faces.iter().copied()); }
    eprintln!("tools: {} sheets, {} cap faces",sheets.len(),tools.len()-sheets.len());

    let stock = cad.0.construct(&cad::recipe(&e.sketch,stock_id).unwrap()).unwrap();
    let stock_volume = cad.0.volume(stock).unwrap();
    assert!((stock_volume-PI*3.5*3.5*1.6).abs() < 1e-9);
    let started = std::time::Instant::now();
    let partition = cad.0.split_solid(stock,&tools).unwrap();
    let split_time = started.elapsed();
    let mut material = MaterialField::read(&e.sketch,part_id,1e-10).unwrap().evaluator(4096);
    let (kept,removed) = classify(&cad,partition,&mut material).unwrap();
    eprintln!("split in {split_time:?}: {} material cells, {} removed cells",kept.len(),removed.len());
    for cell in kept.iter().chain(&removed) {
        eprintln!("  cell volume {:.6} margin {:.4} at {:?}",cell.volume,cell.margin,cell.point);
    }
    let cell_total: f64 = kept.iter().chain(&removed).map(|c| c.volume).sum();
    assert!((cell_total-stock_volume).abs() < 1e-6*stock_volume,"cells partition the stock");
    assert!(!kept.is_empty() && !removed.is_empty());

    let part = cad.0.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>()).unwrap();
    let volume = cad.0.volume(part).unwrap();
    let kept_volume: f64 = kept.iter().map(|c| c.volume).sum();
    assert!((volume-kept_volume).abs() < 1e-6*stock_volume);
    let faces = cad.0.faces(part).unwrap().len();
    eprintln!("part volume {volume:.9} mm^3 ({} of stock), {faces} faces; tolerances {:?} (stock {:?}, partition {:?})",
        volume/stock_volume,cad.0.tolerances(part).unwrap(),cad.0.tolerances(stock).unwrap(),cad.0.tolerances(partition).unwrap());

    // Independent check: closed-form membership of the stock minus the swept
    // balls, against the kernel's own classification of the fused solid.
    let removed_by_formula = |p: [f64;3]| -> f64 {
        // Signed distance to the union of the tube over the roll span and the
        // endpoint balls, negative inside.
        let phi = p[1].atan2(p[0]);
        let mut d = f64::INFINITY;
        if phi.abs() <= PI/3. { d = (p[0].hypot(p[1])-3.).hypot(p[2])-1.; }
        for sign in [-1.,1.] {
            let c = [3.*(PI/3.).cos(),3.*sign*(PI/3.).sin(),0.];
            d = d.min(distance(p,c)-1.);
        }
        d
    };
    let stock_by_formula = |p: [f64;3]| -> f64 { (p[0].hypot(p[1])-3.5).max(p[2].abs()-0.8) };
    // Volume by fine quadrature of the closed form, then membership of a coarser
    // point set against the kernel's classification of the fused solid.
    let (nx,nz) = (360,90);
    let mut inside_count = 0_u64;
    for i in 0..nx { for j in 0..nx { for k in 0..nz {
        let p = [-3.6+7.2*(i as f64+0.5)/nx as f64,-3.6+7.2*(j as f64+0.5)/nx as f64,-0.9+1.8*(k as f64+0.5)/nz as f64];
        if stock_by_formula(p) < 0. && removed_by_formula(p) > 0. { inside_count += 1; }
    } } }
    let estimate = inside_count as f64/(nx*nx*nz) as f64*7.2*7.2*1.8;
    eprintln!("closed-form volume estimate {estimate:.4} against kernel {volume:.4}");
    assert!((volume-estimate).abs() < 0.005*volume,"volume {volume} vs quadrature {estimate}");
    let (mut queries,mut expected,mut skipped) = (Vec::new(),Vec::new(),0);
    let n = 16;
    for i in 0..n { for j in 0..n { for k in 0..n {
        let p = [-3.6+7.2*(i as f64+0.37)/n as f64,-3.6+7.2*(j as f64+0.61)/n as f64,-0.9+1.8*(k as f64+0.23)/n as f64];
        if stock_by_formula(p).abs() < 0.02 || removed_by_formula(p).abs() < 0.02 { skipped += 1; continue; }
        queries.push(p); expected.push(stock_by_formula(p) < 0. && removed_by_formula(p) > 0.);
    } } }
    let started = std::time::Instant::now();
    let states = cad.0.solid_contains(part,&queries,1e-7).unwrap();
    let mut agree_inside = 0;
    for ((p,inside),state) in queries.iter().zip(&expected).zip(&states) {
        assert_eq!(*state == 1,*inside,"kernel and formula disagree at {p:?}");
        if *inside { agree_inside += 1; }
    }
    eprintln!("{} points agree with the closed form ({agree_inside} inside, {skipped} skipped near a boundary) in {:?}",
        queries.len(),started.elapsed());
    assert!(queries.len() > 3_000);

    let out = std::env::temp_dir().join(format!("solvent-cells-{}",std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let step = out.join("sphere-sweep.step"); let stl = out.join("sphere-sweep.stl");
    cad.0.step(part,step.to_str().unwrap()).unwrap();
    cad.0.stl(part,stl.to_str().unwrap()).unwrap();
    let bytes = std::fs::read(&stl).unwrap();
    gcs_core::mesh::stl_shells(&bytes).unwrap();
    eprintln!("exported {} and {} ({} bytes STL)",step.display(),stl.display(),bytes.len());
}
