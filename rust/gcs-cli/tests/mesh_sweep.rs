//! Bodies with swept cuts through mesh arrangement and material classification.
#[path="../src/cad/manifold.rs"]
#[allow(dead_code)]
mod manifold;
mod support;

#[test]
fn manifold_round_trips_a_cube_and_subtracts_a_box() {
    let cube = |lo: [f64;3],hi: [f64;3]| {
        let v: Vec<[f64;3]> = (0..8).map(|i| [if i&1 == 0 { lo[0] } else { hi[0] },if i&2 == 0 { lo[1] } else { hi[1] },if i&4 == 0 { lo[2] } else { hi[2] }]).collect();
        // Outward-wound faces of the box.
        let t = vec![[0,2,1],[1,2,3],[4,5,6],[5,7,6],[0,1,4],[1,5,4],[2,6,3],[3,6,7],[0,4,2],[2,4,6],[1,3,5],[3,7,5]];
        manifold::Solid::from_triangles(&v,&t).unwrap()
    };
    let a = cube([0.;3],[2.;3]);
    assert!((a.volume()-8.).abs() < 1e-12);
    let b = cube([1.,1.,-1.],[3.,3.,3.]);
    let d = a.difference(&b).unwrap();
    assert!((d.volume()-6.).abs() < 1e-12);
    let (inside,outside) = a.split(&b).unwrap();
    assert!((inside.volume()-2.).abs() < 1e-12 && (outside.volume()-6.).abs() < 1e-12);
    let (v,t) = d.triangles().unwrap();
    let back = manifold::Solid::from_triangles(&v,&t).unwrap();
    assert!((back.volume()-6.).abs() < 1e-12);
    let moved = a.placed(&[1.,0.,0.,10., 0.,1.,0.,0., 0.,0.,1.,0.]).unwrap();
    assert!(moved.intersection(&a).unwrap().is_empty());
    assert_eq!(a.components().unwrap().len(),1);
}

#[path="../src/cad/mesh_sweep.rs"]
#[allow(dead_code)]
mod mesh_sweep;

#[test]
fn the_pinion_blank_meshes_as_a_closed_manifold() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let body = e.map.ent_named("pair.pinion.body").unwrap().i();
    let started = std::time::Instant::now();
    let blank = gcs_core::solid::static_solid(&e.sketch,body,24).unwrap();
    eprintln!("static solid: {} primitives in {:?}, unit {}",blank.csg.prims.len(),started.elapsed(),blank.unit);
    let started = std::time::Instant::now();
    let mesh = mesh_sweep::solid_of(&blank).unwrap();
    eprintln!("manifold blank: {:.6} mm^3, {} triangles in {:?}",mesh.volume(),mesh.triangle_count(),started.elapsed());
    assert!((mesh.volume()-12038.99).abs() < 0.002*12038.99);
    assert!(blank.contains([54.,0.,0.]) || !blank.contains([0.;3]));
}

#[test]
fn a_slab_of_a_flat_grid_is_a_closed_box() {
    let (rows,columns) = (4,6);
    let mut points = Vec::new(); let mut normals = Vec::new();
    for r in 0..rows { for c in 0..columns { points.push([c as f64,r as f64,0.]); normals.push([0.,0.,1.]); } }
    let grid = mesh_sweep::SheetGrid {points,normals,times:(0..columns).map(|c| c as f64).collect(),rows,columns,closed_rows:false};
    let slab = mesh_sweep::slab(&grid,0.01).unwrap();
    assert!((slab.volume()-5.*3.*0.01).abs() < 1e-12,"{}",slab.volume());
}

#[test]
fn interior_points_of_the_blank_are_inside_it() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let body = e.map.ent_named("pair.pinion.body").unwrap().i();
    let blank = gcs_core::solid::static_solid(&e.sketch,body,24).unwrap();
    let mesh = mesh_sweep::solid_of(&blank).unwrap();
    for depth in [0.003,0.03,0.3] {
        let p = mesh_sweep::interior_point(&mesh,depth).unwrap();
        eprintln!("depth {depth}: {p:?} inside {}",blank.contains(p));
        assert!(blank.contains(p));
    }
    let (v,t) = mesh.triangles().unwrap();
    // Signed volume from the returned winding must be positive.
    let mut six = 0.;
    for tri in &t { let [a,b,c] = tri.map(|i| v[i as usize]); six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]); }
    eprintln!("signed volume {}",six/6.);
    assert!(six > 0.);
}

#[cfg(feature="occt")]
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;

/// Both members through the mesh arrangement, against the volumes the kernel
/// path recorded. The mesh path tessellates the blank and chords the sheet, so
/// the agreement is to a tenth of a percent, not to the kernel's tolerance.
#[cfg(feature="occt")]
fn whole_member(member: &str,expected: f64) {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let volume = member_volume(&e,member);
    assert!((volume-expected).abs() < 1e-3*expected,"{member}: {volume} against {expected}");
}

/// One member through the mesh arrangement, its signed volume from the STL's own winding.
#[cfg(feature="occt")]
fn member_volume(e: &gcs_core::program::Elaborated,member: &str) -> f64 {
    let body = e.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
    let session = native::Session::new().unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    let sheets = |swept: usize,inside: &dyn Fn(&[[f64;3]]) -> Result<Vec<bool>,String>| {
        let sheet = native::sweep_boundary::swept_sheet_grid(&session,&e.sketch,swept,inside)?;
        Ok(vec![mesh_sweep::SheetGrid {points:sheet.points.iter().map(|p| p.map(|v| v/scale)).collect(),
            normals:sheet.normals.clone(),times:(0..sheet.columns).map(|c| c as f64).collect(),rows:sheet.rows,columns:sheet.columns,closed_rows:false}])
    };
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,body,&sheets).unwrap();
    let bytes = mesh_sweep::stl(&vertices,&triangles,member).unwrap();
    let mut six = 0.;
    for t in &triangles { let [a,b,c] = t.map(|i| vertices[i as usize]); six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]); }
    let volume = six/6.;
    eprintln!("{member}: {volume:.6} mm^3, {} triangles, {} bytes in {:?}",triangles.len(),bytes.len(),started.elapsed());
    volume
}

/// The pair as configured slides the pinion around the crown by the offset
/// angle: its axis turns about the crown normal at the mean point and no longer
/// meets the gear's, by about the mean cone distance times the sine of the
/// angle. The pinion is still one crown tooth swept through its blank at every
/// index; its blank is a body of revolution about its own axis, so every
/// placement of the sheet cuts.
#[cfg(feature="occt")]
#[test]
#[ignore]
fn the_hypoid_pinion_exports_with_every_placement_cutting() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let hypoid = support::read_configured_with(&source,&base,&mut |name,text| support::hypoid6(name,text));
    let axis = |name: &str| {
        let line = &hypoid.sketch.lines[hypoid.map.ent_named(name).unwrap().i()];
        let (a,b) = (hypoid.sketch.world_point(line.p1 as usize),hypoid.sketch.world_point(line.p2 as usize));
        let d: [f64;3] = std::array::from_fn(|k| b[k]-a[k]); let l = d.iter().map(|x| x*x).sum::<f64>().sqrt();
        (a,d.map(|x| x/l))
    };
    let ((a1,d1),(a2,d2)) = (axis("pair.reference.pinion_axis"),axis("pair.reference.gear_axis"));
    let n = [d1[1]*d2[2]-d1[2]*d2[1],d1[2]*d2[0]-d1[0]*d2[2],d1[0]*d2[1]-d1[1]*d2[0]];
    let nl = n.iter().map(|x| x*x).sum::<f64>().sqrt();
    let offset = (0..3).map(|k| (a1[k]-a2[k])*n[k]).sum::<f64>().abs()/nl;
    let mean_distance = 2.*(24f64).hypot(48.)/2.;
    let expected = mean_distance*(6f64).to_radians().sin();
    eprintln!("axis offset {offset:.3} mm against mean_distance * sin(6deg) = {expected:.3}");
    assert!((offset-expected).abs() < 0.1*expected,"the configured pair is a hypoid: axis offset {offset} mm");
    let volume = member_volume(&hypoid,"pinion");
    let bevel = member_volume(&support::read(&source,&base),"pinion");
    eprintln!("hypoid pinion {volume:.3} mm^3, bevel pinion {bevel:.3} mm^3");
    assert!((volume-bevel).abs() > 10.,"the offset changes the pinion: {volume} vs {bevel}");
}

/// Whole members are minutes each; `cargo test -- --ignored` runs them.
#[cfg(feature="occt")]
#[test]
#[ignore]
fn the_pinion_exports_through_the_mesh_path() { whole_member("pinion",9142.079); }

#[cfg(feature="occt")]
#[test]
#[ignore]
fn the_gear_exports_through_the_mesh_path() { whole_member("gear",20284.173); }

/// One tooth space of a member: the pair with `repeat teeth` read as one
/// placement, `configured` keeping the offset angle or reading it as zero.
/// The volumes are this construction's own, recorded once it agreed with the
/// sectioned kernel construction to a tenth of a cubic millimetre; a lost or
/// leaking space is a hundred times that.
#[cfg(feature="occt")]
fn single_space(member: &str,configured: bool,expected: f64) {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let mut one = |name: &str,text: String| if name == "matched_pair" { text.replace("repeat teeth as i {","repeat 1 as i {") } else { text };
    let mut six = |name: &str,text: String| one(name,support::hypoid6(name,text));
    let e = if configured { support::read_configured_with(&source,&base,&mut six) } else { support::read_with(&source,&base,&mut one) };
    let volume = member_volume(&e,member);
    assert!((volume-expected).abs() < 1.,"{member}: {volume} against {expected}");
}

#[cfg(feature="occt")]
#[test]
fn one_hypoid_pinion_space_through_the_tracer() { single_space("pinion",true,13028.897); }

/// One pinion space at the offset angle `SOLVENT_INSPECT_OFFSET` (degrees),
/// its boundary checked against the material field: a point a tenth of a
/// millimetre inside the mesh must not be exterior by the field, nor one
/// outside interior. No closed form gives a hypoid's volume, and this is the
/// check that the sheets enclose what the field says is cut.
#[cfg(feature="occt")]
#[test]
#[ignore]
fn a_hypoid_pinion_space_agrees_with_its_field() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let offset: f64 = std::env::var("SOLVENT_INSPECT_OFFSET").ok().and_then(|v| v.parse().ok()).unwrap_or(20.);
    // `SOLVENT_INSPECT_NOCUT=1`: the blank alone, to see what its own
    // faceting contributes to the disagreements
    let nocut = std::env::var("SOLVENT_INSPECT_NOCUT").is_ok();
    let mut one = |name: &str,text: String| match name {
        "matched_pair" => { let t = text.replace("repeat teeth as i {","repeat 1 as i {"); if nocut { t.replace("    indexed cut body\n","") } else { t } },
        "configuration" => support::design(name,text,offset,0.,35.),
        _ => text,
    };
    let e = support::read_configured_with(&source,&base,&mut one);
    let body = e.map.ent_named("pair.pinion.body").unwrap().i();
    let scale = e.sketch.units.length.unwrap().1;
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,body,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    mesh_sweep::stl(&vertices,&triangles,"pinion").unwrap();
    let mut material = gcs_core::solid::MaterialField::read(&e.sketch,body,1e-10).unwrap().evaluator(4096);
    let (mut checked,mut unresolved) = (0,0);
    let mut disagreements: Vec<([f64;3],bool,[f64;2])> = Vec::new();
    let step = (triangles.len()/400).max(1);
    for t in triangles.iter().step_by(step) {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let n = [(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),(b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])];
        let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        if len < 1e-6 { continue; }
        let centroid: [f64;3] = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        for (side,inside) in [(-0.1,true),(0.1,false)] {
            let p: [f64;3] = std::array::from_fn(|k| centroid[k]+side*n[k]/len);
            let bounds = material.bounds(p.map(|x| gcs_core::interval::Interval::point(x/scale).unwrap()),
                gcs_core::interval::minimum::Options {value_tolerance:0.02/scale,max_evaluations:20000}).unwrap();
            let [lo,hi] = bounds.value.bounds();
            checked += 1;
            if lo <= 0. && hi >= 0. { unresolved += 1; continue; }
            if (inside && lo > 0.) || (!inside && hi < 0.) { disagreements.push((p,inside,[lo,hi])); }
        }
    }
    eprintln!("offset {offset}: {checked} points checked, {unresolved} unresolved, {} disagreements",disagreements.len());
    let d3 = |a: [f64;3],b: [f64;3]| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
    for (p,inside,b) in disagreements.iter().take(20) {
        eprintln!("  {p:?} is {} the mesh, field {b:?}",if *inside { "inside" } else { "outside" });
        if std::env::var("SOLVENT_INSPECT_TRIANGLES").is_err() { continue; }
        // the nearest triangle by centroid, its vertices and the field at each
        let (_,t) = triangles.iter().map(|t| { let [a,b,c] = t.map(|i| vertices[i as usize]); (d3(std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.),*p),t) })
            .fold((f64::INFINITY,&triangles[0]),|m,x| if x.0 < m.0 { x } else { m });
        for &i in t {
            let v = vertices[i as usize];
            let bounds = material.bounds(v.map(|x| gcs_core::interval::Interval::point(x/scale).unwrap()),
                gcs_core::interval::minimum::Options {value_tolerance:0.005/scale,max_evaluations:40000}).unwrap();
            eprintln!("    vertex {i} {:?}: field {:?}",v.map(|x| (x*1e4).round()/1e4),bounds.value.bounds().map(|x| (x*scale*1e5).round()/1e5));
        }
    }
    assert!(disagreements.is_empty());
}

/// The weld and shell check rerun on a mesh dumped by `SOLVENT_DUMP_MESH`:
/// `SOLVENT_MESH=<path>`.
#[test]
#[ignore]
fn weld_a_dumped_mesh() {
    let path = std::env::var("SOLVENT_MESH").unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let u64_at = |o: usize| u64::from_le_bytes(bytes[o..o+8].try_into().unwrap()) as usize;
    let (nv,nt) = (u64_at(0),u64_at(8));
    let vertices: Vec<[f64;3]> = (0..nv).map(|i| std::array::from_fn(|k| f64::from_le_bytes(bytes[16+(3*i+k)*8..16+(3*i+k)*8+8].try_into().unwrap()))).collect();
    let base = 16+nv*24;
    let triangles: Vec<[u32;3]> = (0..nt).map(|i| std::array::from_fn(|k| u32::from_le_bytes(bytes[base+(3*i+k)*4..base+(3*i+k)*4+4].try_into().unwrap()))).collect();
    eprintln!("{nv} vertices, {nt} triangles");
    match mesh_sweep::stl(&vertices,&triangles,"dumped") {
        Ok(bytes) => eprintln!("shell ok, {} bytes",bytes.len()),
        Err(e) => panic!("{e}"),
    }
}

/// The tracer's untrimmed pieces at one parameter, for inspection:
/// `SOLVENT_INSPECT_OFFSET=<degrees> SOLVENT_INSPECT_T=<parameter>`, with
/// every pair of pieces that lie on one another named.
#[cfg(feature="occt")]
#[test]
#[ignore]
fn inspect_pieces_at_a_parameter() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let offset: f64 = std::env::var("SOLVENT_INSPECT_OFFSET").ok().and_then(|v| v.parse().ok()).unwrap_or(6.);
    let t: f64 = std::env::var("SOLVENT_INSPECT_T").ok().and_then(|v| v.parse().ok()).unwrap_or(0.);
    let member = std::env::var("SOLVENT_INSPECT_MEMBER").unwrap_or("pinion".into());
    let mut one = |name: &str,text: String| match name {
        "matched_pair" => text.replace("repeat teeth as i {","repeat 1 as i {"),
        "configuration" => support::design(name,text,offset,0.,35.),
        _ => text,
    };
    let e = support::read_configured_with(&source,&base,&mut one);
    let removal = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
    let sweep = gcs_core::solid::SweepContacts::read(&e.sketch,removal,1e-10).unwrap();
    // `SOLVENT_INSPECT_NEAR=x,y,z`: over the roll, which pieces pass within
    // half a millimetre of a world point, and when
    if let Ok(near) = std::env::var("SOLVENT_INSPECT_NEAR") {
        let target: Vec<f64> = near.split(',').map(|v| v.trim().parse().unwrap()).collect();
        let target = [target[0],target[1],target[2]];
        let gcs_core::model::SolidDef::Swept {motion,from,to,..} = &e.sketch.solids[removal].def else { panic!("not a sweep") };
        let family = gcs_core::motion::Family::read(&e.sketch,*motion as usize).unwrap();
        let d3 = |a: [f64;3],b: [f64;3]| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
        let radius: f64 = std::env::var("SOLVENT_INSPECT_RADIUS").ok().and_then(|v| v.parse().ok()).unwrap_or(0.5);
        let window: Option<(f64,f64)> = std::env::var("SOLVENT_INSPECT_WINDOW").ok().map(|w| { let v: Vec<f64> = w.split(',').map(|x| x.trim().parse().unwrap()).collect(); (v[0],v[1]) });
        let (t0,t1) = window.unwrap_or((from.value,to.value));
        let body = e.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
        let scale = e.sketch.units.length.unwrap().1;
        let mut material = gcs_core::solid::MaterialField::read(&e.sketch,body,1e-10).unwrap().evaluator(4096);
        let mut field_at = |p: [f64;3]| -> [f64;2] {
            material.bounds(p.map(|x| gcs_core::interval::Interval::point(x/scale).unwrap()),
                gcs_core::interval::minimum::Options {value_tolerance:0.005/scale,max_evaluations:40000}).unwrap().value.bounds().map(|x| x*scale)
        };
        let steps: usize = std::env::var("SOLVENT_INSPECT_STEPS").ok().and_then(|v| v.parse().ok()).unwrap_or(400);
        for k in 0..=steps {
            let t = t0+(t1-t0)*k as f64/steps as f64;
            let pose = family.at(t).unwrap();
            for (source,c) in sweep.pieces_at(t,1e-9).unwrap() {
                let (d,i) = c.points.iter().enumerate().map(|(i,p)| (d3(pose.point(*p),target),i)).fold((f64::INFINITY,0),|m,x| if x.0 < m.0 { x } else { m });
                if d < radius {
                    let world = pose.point(c.points[i]);
                    eprintln!("t {t:.5}: {source} within {d:.4} at point {i} of {} (world {:?}), field there {:?}",c.points.len(),world.map(|x| (x*1e4).round()/1e4),field_at(world).map(|x| (x*1e5).round()/1e5));
                    if std::env::var("SOLVENT_INSPECT_WHOLE").is_ok() {
                        for (j,p) in c.points.iter().enumerate() {
                            let w = pose.point(*p);
                            if d3(w,target) > 3. { continue; }
                            eprintln!("    point {j}: tool {:?} world {:?} normal {:?} field {:?}",p.map(|x| (x*1e4).round()/1e4),w.map(|x| (x*1e4).round()/1e4),c.normals[j].map(|x| (x*1e3).round()/1e3),field_at(w).map(|x| (x*1e5).round()/1e5));
                        }
                    }
                }
            }
        }
        return;
    }
    let pieces = sweep.pieces_at(t,1e-9).unwrap();
    let dist = |a: [f64;3],b: [f64;3]| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
    let length = |c: &gcs_core::solid::Characteristic| c.points.windows(2).map(|w| dist(w[0],w[1])).sum::<f64>();
    // what the tool's own field hides: a point off its boundary
    let gcs_core::model::SolidDef::Swept {source,..} = &e.sketch.solids[removal].def else { panic!("not a sweep") };
    let field = gcs_core::solid::SpatialField::read(&e.sketch,*source as usize,1e-10).unwrap();
    let scale = pieces.iter().flat_map(|(_,c)| c.points.iter()).map(|p| dist(*p,[0.;3])).fold(1_f64,f64::max);
    for (i,(source,c)) in pieces.iter().enumerate() {
        let hidden: Vec<(usize,f64)> = c.points.iter().enumerate().map(|(k,p)| (k,field.value(*p))).filter(|(_,v)| v.abs() > scale*1e-9).collect();
        eprintln!("piece {i}: {source}: {} points, length {:.4}, closed {}, from {:?} to {:?}, {} hidden{}",c.points.len(),length(c),c.closed,
            c.points[0].map(|x| (x*1e4).round()/1e4),c.points[c.points.len()-1].map(|x| (x*1e4).round()/1e4),hidden.len(),
            hidden.first().map(|(k,v)| format!(" (first at {k}, field {v:.3e})")).unwrap_or_default());
    }
    // every piece end's gap to the nearest end of any other piece
    let ends: Vec<(usize,[f64;3])> = pieces.iter().enumerate().flat_map(|(i,(_,c))| [(i,c.points[0]),(i,c.points[c.points.len()-1])]).collect();
    for (i,p) in &ends {
        let (gap,j) = ends.iter().filter(|(j,_)| j != i).map(|(j,q)| (dist(*p,*q),*j)).fold((f64::INFINITY,0),|m,x| if x.0 < m.0 { x } else { m });
        if gap > 1e-9 { eprintln!("piece {i} ({}) end {:?} is {gap:.3e} from its nearest, piece {j} ({})",pieces[*i].0,p.map(|x| (x*1e4).round()/1e4),pieces[j].0); }
    }
    // pieces lying on one another: every sample of one within 1e-4 of the other
    let onto = |a: &gcs_core::solid::Characteristic,b: &gcs_core::solid::Characteristic| -> f64 {
        a.points.iter().step_by((a.points.len()/16).max(1)).map(|p| b.points.windows(2).map(|w| {
            let ab = [w[1][0]-w[0][0],w[1][1]-w[0][1],w[1][2]-w[0][2]]; let ap = [p[0]-w[0][0],p[1]-w[0][1],p[2]-w[0][2]];
            let l2 = ab[0]*ab[0]+ab[1]*ab[1]+ab[2]*ab[2];
            let f = if l2 > 0. { ((ab[0]*ap[0]+ab[1]*ap[1]+ab[2]*ap[2])/l2).clamp(0.,1.) } else { 0. };
            dist(*p,[w[0][0]+f*ab[0],w[0][1]+f*ab[1],w[0][2]+f*ab[2]])
        }).fold(f64::INFINITY,f64::min)).fold(0_f64,f64::max)
    };
    for i in 0..pieces.len() { for j in 0..pieces.len() {
        if i == j || pieces[i].1.points.len() < 2 || pieces[j].1.points.len() < 2 { continue; }
        let d = onto(&pieces[i].1,&pieces[j].1);
        if d < 1e-3 { eprintln!("piece {i} ({}) lies on piece {j} ({}) within {d:.2e}",pieces[i].0,pieces[j].0); }
    } }
}

#[cfg(feature="occt")]
#[test]
fn one_bevel_pinion_space_through_the_tracer() { single_space("pinion",false,11906.847); }

#[cfg(feature="occt")]
#[test]
fn one_gear_space_through_the_tracer() { single_space("gear",true,25775.875); }

const SPHERE_TOOL: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private point bottom hint(x: 3, y: -1)
private point top hint(x: 3, y: 1)
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid tool(face(meridian, diameter), about: diameter)
";
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
removal cut part
";

/// Volume and membership of a constructed body against a closed form over a
/// box, with points within `skin` of the closed form's boundary skipped.
fn check_closed_form(vertices: &[[f64;3]],triangles: &[[u32;3]],inside: &dyn Fn([f64;3]) -> f64,lo: [f64;3],hi: [f64;3],skin: f64) {
    check_form(vertices,triangles,inside,lo,hi,skin,true)
}

/// Membership alone, for a form that is expensive to ask (a material field).
fn check_membership(vertices: &[[f64;3]],triangles: &[[u32;3]],inside: &dyn Fn([f64;3]) -> f64,lo: [f64;3],hi: [f64;3],skin: f64) {
    check_form(vertices,triangles,inside,lo,hi,skin,false)
}

fn check_form(vertices: &[[f64;3]],triangles: &[[u32;3]],inside: &dyn Fn([f64;3]) -> f64,lo: [f64;3],hi: [f64;3],skin: f64,quadrature: bool) {
    let mut six = 0.;
    for t in triangles { let [a,b,c] = t.map(|i| vertices[i as usize]); six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]); }
    let volume = six/6.;
    if quadrature {
        let n = 120;
        let mut count = 0_u64;
        for i in 0..n { for j in 0..n { for k in 0..n {
            let p: [f64;3] = std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*([i,j,k][a] as f64+0.5)/n as f64);
            if inside(p) < 0. { count += 1; }
        } } }
        let estimate = count as f64/(n*n*n) as f64*(hi[0]-lo[0])*(hi[1]-lo[1])*(hi[2]-lo[2]);
        eprintln!("volume {volume:.5} against closed-form quadrature {estimate:.5}");
        assert!((volume-estimate).abs() < 0.01*estimate,"volume {volume} vs {estimate}");
    } else { eprintln!("volume {volume:.5}"); }
    let solid = manifold::Solid::from_triangles(vertices,triangles).unwrap();
    assert!((solid.volume()-volume).abs() < 1e-9);
    // Membership by ray parity against the mesh, at withheld points.
    let mut checked = 0;
    let mut disagreements: Vec<([f64;3],f64)> = Vec::new();
    let m = 12;
    for i in 0..m { for j in 0..m { for k in 0..m {
        let p: [f64;3] = std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*([i,j,k][a] as f64+0.37)/m as f64);
        let f = inside(p);
        if f.abs() < skin { continue; }
        let mut crossings = 0;
        let d = [0.3127,0.7231,0.6129];
        for t in triangles {
            let [a,b,c] = t.map(|i| vertices[i as usize]);
            let (e1,e2) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
            let h = [d[1]*e2[2]-d[2]*e2[1],d[2]*e2[0]-d[0]*e2[2],d[0]*e2[1]-d[1]*e2[0]];
            let det = e1[0]*h[0]+e1[1]*h[1]+e1[2]*h[2];
            if det.abs() < 1e-12 { continue; }
            let s = [p[0]-a[0],p[1]-a[1],p[2]-a[2]];
            let u = (s[0]*h[0]+s[1]*h[1]+s[2]*h[2])/det; if !(0. ..=1.).contains(&u) { continue; }
            let q = [s[1]*e1[2]-s[2]*e1[1],s[2]*e1[0]-s[0]*e1[2],s[0]*e1[1]-s[1]*e1[0]];
            let v = (d[0]*q[0]+d[1]*q[1]+d[2]*q[2])/det; if v < 0. || u+v > 1. { continue; }
            let t = (e2[0]*q[0]+e2[1]*q[1]+e2[2]*q[2])/det; if t > 0. { crossings += 1; }
        }
        if (crossings%2 == 1) != (f < 0.) { disagreements.push((p,f)); }
        checked += 1;
    } } }
    eprintln!("{checked} withheld points checked against the form, {} disagree",disagreements.len());
    for (p,f) in &disagreements { eprintln!("  disagree at {p:?}: form {f}"); }
    assert!(disagreements.is_empty(),"{} of {checked} withheld points disagree with the form",disagreements.len());
    assert!(checked > 500);
}

#[test]
fn a_sphere_swept_past_a_bead_carves_a_torus_bite() {
    let source = format!("{SPHERE_TOOL}motion turn(about: spindle)\nconstruction solid removal(tool, under: turn, from: -60deg, to: 60deg)\n{BEAD}");
    let e = support::read(&source,std::path::Path::new("."));
    let part = e.map.ent_named("part").unwrap().i();
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,part,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    eprintln!("torus bite in {:?}, {} triangles",started.elapsed(),triangles.len());
    // Inside the bead (radius 1 about (3.8,0,0)) and outside the tube (ring
    // radius 3 about z, tube radius 1), which the bead's angular span keeps
    // within the swept sector.
    let inside = |p: [f64;3]| ((p[0]-3.8).hypot(p[1]).hypot(p[2])-1.).max(1.-(p[0].hypot(p[1])-3.).hypot(p[2]));
    check_closed_form(&vertices,&triangles,&inside,[2.8,-1.,-1.],[4.8,1.,1.],0.03);
}

#[test]
fn a_cylinder_plunging_through_a_bead_bores_it() {
    let source = format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 4, y: -1)
private point c2 hint(x: 4, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid tool(face(bottom, wall, top, axis), about: axis)
motion plunge(along: axis, advance: 6mm)
construction solid removal(tool, under: plunge, from: 0deg, to: 360deg)
{BEAD}");
    let e = support::read(&source,std::path::Path::new("."));
    let part = e.map.ent_named("part").unwrap().i();
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,part,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    eprintln!("bored bead in {:?}, {} triangles",started.elapsed(),triangles.len());
    // The tool starts at z in [-1,1] and travels 6 along the axis (world z),
    // so the bead, within |z| < 1, is bored by a cylinder of radius 1 about x=3.
    let inside = |p: [f64;3]| ((p[0]-3.8).hypot(p[1]).hypot(p[2])-1.).max(1.-(p[0]-3.).hypot(p[1]));
    check_closed_form(&vertices,&triangles,&inside,[2.8,-1.,-1.],[4.8,1.,1.],0.03);
}

const BOX_TOOL: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point b0 hint(x: 2, y: -1)
private point b1 hint(x: 4, y: -1)
private point b2 hint(x: 4, y: 1)
private point b3 hint(x: 2, y: 1)
ground b0
ground b1
ground b2
ground b3
private line e0(b0, b1)
private line e1(b1, b2)
private line e2(b2, b3)
private line e3(b3, b0)
construction solid tool(face(e0, e1, e2, e3), from: 0mm, to: 3mm)
";

/// A prism tool: a box translated along x through the bead. Every side of the
/// box is stationary under the translation, so the candidates are the segments
/// round its ends carried along; the bead loses the half the box passes
/// through. The page is x across and z up with depth along -y, so the box is
/// x in [2,4], z in [-1,1], y in [-3,0] and travels 10 along x.
#[test]
fn a_box_translated_through_a_bead_removes_a_half_space_of_it() {
    let source = format!("{BOX_TOOL}private point r0 hint(x: 0, y: 0)
private point r1 hint(x: 10, y: 0)
ground r0
ground r1
construction centerline line rail(r0, r1)
motion feed(along: rail, advance: 10mm)
construction solid removal(tool, under: feed, from: 0deg, to: 360deg)
{BEAD}");
    let e = support::read(&source,std::path::Path::new("."));
    let part = e.map.ent_named("part").unwrap().i();
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,part,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    eprintln!("box through bead in {:?}, {} triangles",started.elapsed(),triangles.len());
    let inside = |p: [f64;3]| ((p[0]-3.8).hypot(p[1]).hypot(p[2])-1.).max(-p[1]);
    check_closed_form(&vertices,&triangles,&inside,[2.8,-1.,-1.],[4.8,1.,1.],0.03);
}

/// A prism tool under a rotation: a triangular prism turning about an axis
/// parallel to its extrusion and off to one side of it, through the bead. There is
/// no closed form to hand, so the mesh is held to the declared material field
/// itself at withheld points: its planar sides' contact lines and its edges'
/// fans must bound exactly what the field says is removed.
#[test]
fn a_triangular_prism_turning_about_an_offset_axis_agrees_with_its_field() {
    let source = format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point t0 hint(x: 3, y: -0.8)
private point t1 hint(x: 4.5, y: 0)
private point t2 hint(x: 3, y: 0.8)
ground t0
ground t1
ground t2
private line e0(t0, t1)
private line e1(t1, t2)
private line e2(t2, t0)
construction solid tool(face(e0, e1, e2), from: -1.5mm, to: 1.5mm)
private point a0 hint(x: 2.5, y: 0)
a0 distance(2.5mm, along: u) std.front
a0 distance(0mm, along: v) std.front
private point a1 hint(x: 2.5, y: 5)
a1 distance(2.5mm, along: u) std.front
a1 distance(5mm, along: v) std.front
construction centerline line pivot(a0, a1)
motion turn(about: pivot)
construction solid removal(tool, under: turn, from: -50deg, to: 50deg)
{BEAD}");
    let e = support::read(&source,std::path::Path::new("."));
    let part = e.map.ent_named("part").unwrap().i();
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,part,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    eprintln!("turned prism through bead in {:?}, {} triangles",started.elapsed(),triangles.len());
    let material = std::cell::RefCell::new(gcs_core::solid::MaterialField::read(&e.sketch,part,1e-10).unwrap().evaluator(4096));
    let inside = |p: [f64;3]| {
        let bounds = material.borrow_mut().bounds(p.map(|x| gcs_core::interval::Interval::point(x).unwrap()),
            gcs_core::interval::minimum::Options {value_tolerance:0.005,max_evaluations:20000}).unwrap();
        let [lo,hi] = bounds.value.bounds();
        if hi < 0. { hi } else if lo > 0. { lo } else { 0. }
    };
    check_membership(&vertices,&triangles,&inside,[2.8,-1.,-1.],[4.8,1.,1.],0.03);
}
