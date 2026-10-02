use gcs_core::{io,model::{MotionDef,SolidDef},program,solid::{self,MaterialField},solve,syntax,
    interval::{Interval as I,minimum::{Options,Status}}};

mod contacts;

const SOURCE: &str = "unit mm
center := point
a := point
b := point
o := point
z := point
fix(x == 3, y == 0) center
fix(x == 3, y == -1) a
fix(x == 3, y == 1) b
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
rim := arc(center: center,start: a,end: b)
radius(1) rim
diameter := line(a,b)
axis := line(o,z)
tool := solid(face(rim,diameter),about: diameter)
generating := motion(about: axis)
";
fn read(source: &str) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source);
    assert!(errors.is_empty(),"{errors:?}");
    assert!(gcs_core::modules::link(&mut p,&mut gcs_core::library::resolve).is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}
fn material(sk: &gcs_core::model::Sketch,i: usize,p: [f64;3]) -> [f64;2] {
    let mut query = MaterialField::read(sk,i,1e-10).unwrap().evaluator(256);
    let answer = query.bounds(p.map(|v| I::point(v).unwrap()),
        Options {value_tolerance:1e-4,max_evaluations:50000}).unwrap();
    assert!(answer.sweeps.iter().all(|s| s.minimum.status == Status::Converged));
    answer.value.bounds()
}

#[test]
fn continuous_sweep_source_has_the_full_interval_and_endpoint_caps() {
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let SolidDef::Swept {from,to,..} = &e.sketch.solids[i].def else { panic!() };
    assert!((from.value+std::f64::consts::PI/3.).abs() < 1e-12);
    assert!((to.value-std::f64::consts::PI/3.).abs() < 1e-12);
    for p in [[3_f64,0.,0.],[-3.,0.,0.],[0.,3.,0.],[4.,2.,1.],[0.;3]] {
        let angle = p[1].atan2(p[0]).clamp(from.value,to.value);
        let expected = (p[0]-3.*angle.cos()).hypot(p[1]-3.*angle.sin()).hypot(p[2])-1.;
        let got = material(&e.sketch,i,p);
        assert!(got[0] <= expected+1e-12 && got[1] >= expected-1e-12,"{p:?}: {got:?}, {expected}");
    }
    assert!(solid::cad::recipe(&e.sketch,i).unwrap_err().contains("continuous motion sweeps"));
}

#[test]
fn a_continuous_sweep_is_meshed_from_its_field() {
    // A unit sphere swept along a circle of radius 3 through ±60°: a tube of length 2π and the
    // two hemispherical caps, 2π² + 4π/3.
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let solid = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap();
    let exact = 2.*std::f64::consts::PI.powi(2)+4./3.*std::f64::consts::PI;
    assert!((solid.volume()-exact).abs() < 0.03*exact,"volume {} against {exact}",solid.volume());
    assert!(solid.contains_world(solid::WorldPoint([3.,0.,0.])));
    assert!(solid.contains_world(solid::WorldPoint([1.5,2.598,0.])));
    assert!(!solid.contains_world(solid::WorldPoint([-3.,0.,0.])));
    // Every pixel length a view asks with is the one field mesh.
    let view = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::from_unit(0.01)).unwrap();
    assert!(std::rc::Rc::ptr_eq(&solid,&view));
    assert!(!solid.mesh().positions.is_empty());
}

#[test]
fn sweep_print_copy_paste_and_delete_preserve_transitive_dependencies() {
    let e = read(&format!("swept := solid(tool,under: generating,from: -60deg,to: 60deg)\n{SOURCE}"));
    let mut p = e.program.clone();
    let printed = syntax::render_flat(&mut p).unwrap();
    assert!(printed.contains("from: -60deg, to: 60deg"),"{printed}");
    let restored = read(printed);
    let i = restored.map.ent_named("swept").unwrap().i();
    assert!(material(&restored.sketch,i,[3.,0.,0.])[1] < -0.99);
    let copied = io::copy(&e.sketch,&[e.map.ent_named("swept").unwrap()]);
    assert_eq!(copied.solids.len(),2); assert_eq!(copied.motions.len(),1);
    let i = copied.solids.iter().position(|s| s.name == "swept").unwrap();
    assert!(material(&copied,i,[3.,0.,0.])[1] < -0.99);
    let mut pasted = gcs_core::model::Sketch::new();
    io::paste(&mut pasted,&copied,10.,20.);
    let i = pasted.solids.iter().position(|s| s.name == "swept").unwrap();
    assert!(material(&pasted,i,[13.,0.,20.])[1] < -0.99);
    let removed = io::without(&e.sketch,&[e.map.ent_named("axis").unwrap()],&[]);
    assert!(removed.motions.is_empty());
    assert_eq!(removed.solids.len(),1);
    assert_eq!(removed.solids[0].name,"tool");
}

#[test]
fn repeated_component_cuts_index_one_continuous_sweep() {
    let e = read(&format!("{SOURCE}
component IndexedCuts(tool: solid,target: solid,indexing: motion,count: Int) {{
  repeat count as i {{
    placed := solid(tool,under: indexing,at: i*360deg/count)
    placed cut target
  }}
}}
band := circle(center: center)
radius(2) band
stock := solid(face(band),about: axis)
body := solid(stock)
swept := solid(tool,under: generating,from: -10deg,to: 10deg)
cuts := IndexedCuts(swept,body,generating,count: 3)
"));
    let i = e.map.ent_named("body").unwrap().i();
    for k in 0..6 {
        let angle = k as f64*std::f64::consts::PI/3.;
        let p = [3.*angle.cos(),3.*angle.sin(),0.];
        let got = material(&e.sketch,i,p);
        assert!(if k % 2 == 0 { got[0] > 0.9 } else { got[1] < -0.1 },"{k}: {got:?}");
    }
    // The field mesh agrees with the field at the same points.
    let solid = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap();
    for k in 0..6 {
        let angle = k as f64*std::f64::consts::PI/3.;
        let inside = solid.contains_world(solid::WorldPoint([3.*angle.cos(),3.*angle.sin(),0.]));
        assert_eq!(inside,k % 2 == 1,"{k}");
    }
}

#[test]
fn sweep_fingerprints_read_the_motion_family_and_source_geometry() {
    let mut e = read(&format!("{SOURCE}swept := solid(tool,under: generating,from: 0deg,to: 360deg)\n"));
    let i = e.map.ent_named("swept").unwrap().i();
    let before = solid::reads(&e.sketch,i,solid::REPORT_UNIT);
    let m = e.map.ent_named("generating").unwrap().i();
    let MotionDef::Rotation {ratio,..} = &mut e.sketch.motions[m].def else { panic!() };
    *ratio = 2.; // Endpoint poses coincide; the whole family must still invalidate the cache.
    let changed = solid::reads(&e.sketch,i,solid::REPORT_UNIT);
    assert_ne!(before,changed);
    let radius = e.sketch.arcs[0].radius as usize;
    e.sketch.params[radius].value = 0.5;
    assert_ne!(changed,solid::reads(&e.sketch,i,solid::REPORT_UNIT));
}

#[test]
fn invalid_intervals_cycles_and_nested_sweeps_are_explicit() {
    for bad in ["bad := solid(tool,under: generating,from: 1deg)",
        "bad := solid(tool,under: generating,to: 1deg)",
        "bad := solid(tool,under: generating,from: 0deg,to: 0deg)",
        "bad := solid(tool,under: generating,from: 1deg,to: -1deg)",
        "bad := solid(tool,under: generating,from: 1mm,to: 2mm)",
        "bad := solid(tool,under: generating,from: 0deg,to: 1deg,at: 0deg)",
        "bad := solid(tool,under: generating,from: 0deg,to: 1deg,depth: 1mm)",
        "bad := solid(tool,under: axis,from: 0deg,to: 1deg)",
        "bad := solid(bad,under: generating,from: 0deg,to: 1deg)"] {
        let (p,errors) = syntax::parse(&format!("{SOURCE}{bad}\n"));
        let e = program::elaborate(&p);
        assert!(!errors.is_empty() || !e.ok() ||
            e.sketch.solids.iter().enumerate().any(|(i,_)| solid::validate(&e.sketch,i).is_err()),"{bad}");
    }
    let e = read(&format!("{SOURCE}inner_sweep := solid(tool,under: generating,from: -10deg,to: 10deg)\n\
        outer_sweep := solid(inner_sweep,under: generating,from: -10deg,to: 10deg)\n"));
    let i = e.map.ent_named("outer_sweep").unwrap().i();
    assert!(MaterialField::read(&e.sketch,i,1e-10).unwrap_err().contains("nested continuous sweeps"));
}

/// A finer mesh is the same surface in more, smaller facets; a fineness the mesher does not
/// offer is refused with the range it does.
#[test]
fn a_swept_surface_meshes_finer_when_asked() {
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let mesh = |f: f64| solid::FieldMesher::with_fineness(&e.sketch,i,f).unwrap().finish().unwrap();
    let (normal,fine) = (mesh(0.5),mesh(1.0));
    // facets half the size: about four times as many over the same surface
    let ratio = fine.triangles.len() as f64/normal.triangles.len() as f64;
    assert!((2.5..6.0).contains(&ratio),"{} then {} triangles",normal.triangles.len(),fine.triangles.len());
    let area = |s: &solid::FieldSurface| s.triangles.iter().map(|t| {
        let [a,b,c] = t.map(|k| s.vertices[k as usize]);
        0.5*gcs_core::space::norm(gcs_core::space::cross(gcs_core::space::sub(b,a),gcs_core::space::sub(c,a)))
    }).sum::<f64>();
    let (a1,a2) = (area(&normal),area(&fine));
    assert!((a1-a2).abs() < 0.02*a2,"areas {a1} and {a2}");
    for bad in [0.0,-1.0,f64::NAN,100.0] {
        let error = solid::FieldMesher::with_fineness(&e.sketch,i,bad).err().unwrap();
        assert!(error.contains("mesh fineness") && error.contains("0.25 to 4"),"{error}");
    }
}

#[test]
fn a_swept_surface_is_refined_in_steps_and_supplied_to_the_drawing() {
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let mut mesher = solid::FieldMesher::new(&e.sketch,i).unwrap();
    let mut seen = Vec::new();
    loop {
        let done = mesher.step(100).unwrap();
        let s = mesher.snapshot();
        seen.push((s.triangles.len(),s.provisional));
        // what a host shows is the core's words, and they begin with the phase
        let p = mesher.progress();
        assert!(p.doing().starts_with(p.phase),"{} / {}",p.doing(),p.phase);
        if done { break; }
    }
    // A preview first, refined over many steps, and a final surface last.
    assert!(seen.len() > 5,"{seen:?}");
    assert!(seen.iter().rev().skip(1).all(|s| s.1) && !seen.last().unwrap().1,"{seen:?}");
    let first = seen.iter().find(|s| s.0 > 0).unwrap().0;
    assert!(first < seen.last().unwrap().0,"{seen:?}");
    let surface = mesher.snapshot();

    // A page meshing elsewhere: the solid is refused until its surface arrives.
    e.sketch.field_meshing.set(gcs_core::solid::FieldMeshing::Deferred);
    assert!(e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap_err().contains("being meshed"));
    e.sketch.supply_field(i,solid::FieldSurface {provisional:true,..surface.clone()}).unwrap();
    assert!(e.sketch.field_provisional(i).unwrap());
    let preview = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap();
    assert!(preview.provisional() && preview.stl().is_err());
    // nor does a scene take a surface still being refined
    assert!(gcs_core::gltf::checked_glb(&e.sketch,&[i],solid::ApproximationPolicy::Mesh).unwrap_err().contains("still being refined"));
    // asked for as a preview, it is written as it stands, every triangle in the file
    let bytes = preview.preview_stl().unwrap();
    assert!(bytes.len() > 84 && (bytes.len()-84)%50 == 0 && u32::from_le_bytes(bytes[80..84].try_into().unwrap()) > 0);
    e.sketch.supply_field(i,surface).unwrap();
    assert!(!e.sketch.field_provisional(i).unwrap());
    let done = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap();
    let exact = 2.*std::f64::consts::PI.powi(2)+4./3.*std::f64::consts::PI;
    assert!(!done.provisional() && (done.volume()-exact).abs() < 0.03*exact,"{}",done.volume());
    assert!(done.stl().is_ok());
}

/// What a page's worker is to mesh is the core's answer: the swept objects, each keyed by the
/// drawing it is a surface of — the same key for the same drawing elaborated twice, another once
/// the drawing moves — and a supplied surface the core cannot use is refused with the reason.
#[test]
fn field_jobs_name_the_swept_objects_and_supplied_surfaces_are_checked() {
    let source = include_str!("../../examples/solid_generating_sweep.sv");
    let e = read(source);
    let jobs = e.sketch.field_jobs();
    let i = e.map.ent_named("removal.body").unwrap().i();
    assert_eq!(jobs.iter().map(|j| (j.solid,j.name.as_str())).collect::<Vec<_>>(),vec![(i,"removal.body")]);
    // the construction tool it is swept from is no object, and has no sweep: no job
    assert_eq!(read(source).sketch.field_jobs(),jobs,"the same drawing elaborated again");
    let moved = read(&source.replace("finish: 60deg","finish: 50deg"));
    assert_ne!(moved.sketch.field_jobs()[0].key,jobs[0].key,"another drawing is another surface");

    let square = solid::FieldSurface {vertices: vec![[0.;3],[1.,0.,0.],[0.,1.,0.]],triangles: vec![[0,1,2]],provisional: true,exact: None};
    let refused = |k: usize,s: solid::FieldSurface| e.sketch.supply_field(k,s).unwrap_err();
    assert!(refused(e.sketch.solids.len(),square.clone()).contains("names no solid"));
    let plain = (0..e.sketch.solids.len()).find(|&k| !e.sketch.is_swept(k)).unwrap();
    assert!(refused(plain,square.clone()).contains("has no sweep"));
    assert!(refused(i,solid::FieldSurface {triangles: vec![[0,1,3]],..square.clone()}).contains("past the 3 given"));
    assert!(e.sketch.field_provisional(e.sketch.solids.len()).is_err());
    assert!(!e.sketch.field_provisional(plain).unwrap());
    e.sketch.supply_field(i,square).unwrap();
    assert!(e.sketch.field_provisional(i).unwrap());
}

#[test]
fn a_reading_gives_a_sweeps_value_gradient_and_contact_time() {
    // A unit sphere swept ±60° round a circle of radius 3: outside it the field is the distance to
    // the nearest point of the arc less one, rising straight away from that point.
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let field = MaterialField::read(&e.sketch,i,1e-10).unwrap();
    let limit = std::f64::consts::PI/3.;
    for p in [[4.5_f64,0.3,0.2],[1.,2.5,-0.7],[2.,-2.8,1.1],[3.,0.,1.5],[-0.5,3.6,0.4]] {
        let angle = p[1].atan2(p[0]).clamp(-limit,limit);
        let c = [3.*angle.cos(),3.*angle.sin(),0.];
        let d: [f64;3] = std::array::from_fn(|k| p[k]-c[k]);
        let len = (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt();
        // By default the minimum is read to a thousandth of itself; asked exactly, to 1e-8.
        let loose = field.reading(p);
        assert!((loose.value-(len-1.)).abs() <= 1e-3*(len-1.).abs()+1e-8,"{p:?}: {} against {}",loose.value,len-1.);
        let exact = gcs_core::solid::Query {relative:0.,..gcs_core::solid::Query::at(p)};
        let r = field.query(p,&mut exact.clone());
        assert!((r.value-(len-1.)).abs() < 1e-8,"{p:?}: {} against {}",r.value,len-1.);
        for k in 0..3 { assert!((r.gradient[k]-d[k]/len).abs() < 1e-5,"{p:?}: {:?} against {:?}",r.gradient,d.map(|x| x/len)); }
        assert!((r.time().unwrap()-angle).abs() < 1e-5,"{p:?}: time {:?} against {angle}",r.time());
        assert!(!r.ambiguous,"{p:?}");
    }
    // Opposite the gap both end caps are equally near: two contact times, a crease.
    assert!(field.reading([-3.,0.,0.]).ambiguous);
}

#[test]
fn a_reading_bounds_what_cannot_decide_and_is_exact_where_it_can() {
    // Branch and bound: the groove's part is the block less the swept ball, and its reading asks
    // the sweep only for a bound where the block decides. Whatever it skipped, the part reads
    // exactly what the two read alone make it, max(block, −groove), and its sign is `side`'s.
    let e = read(include_str!("../../examples/swept_groove.sv"));
    let field = |n: &str| MaterialField::read(&e.sketch,e.map.ent_named(n).unwrap().i(),1e-10).unwrap();
    let (part,block,groove) = (field("part"),field("block"),field("groove"));
    let exact = |p: [f64;3]| gcs_core::solid::Query {relative:0.,..gcs_core::solid::Query::at(p)};
    for i in 0..9 { for j in 0..9 { for k in 0..5 {
        let p = [-22.+5.5*i as f64+0.3,-22.+5.5*j as f64+0.1,-14.+4.*k as f64+0.2];
        let whole = part.query(p,&mut exact(p)).value;
        let alone = block.query(p,&mut exact(p)).value.max(-groove.query(p,&mut exact(p)).value);
        assert!((whole-alone).abs() < 1e-8,"{p:?}: the part reads {whole}, its operands {alone}");
        if alone.abs() > 1e-6 { assert_eq!(whole < 0.,part.side(p) < 0.,"{p:?}: {whole}"); }
    } } }
}

#[test]
fn a_warm_reading_is_the_cold_one_whatever_its_hint() {
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let field = MaterialField::read(&e.sketch,i,1e-10).unwrap();
    let exact = |p: [f64;3]| gcs_core::solid::Query {relative:0.,..gcs_core::solid::Query::at(p)};
    let warm = |p: [f64;3],hints: Vec<Option<f64>>| gcs_core::solid::Query {source:gcs_core::solid::Source::Warm {hints,local:false},..exact(p)};
    // Along a segment through the swept body, each reading warm from the last.
    let mut q = warm([0.;3],Vec::new());
    for k in 0..=20 {
        let s = k as f64/20.;
        let p = [4.6-3.*s,-1.+2.8*s,0.3];
        let hints = match std::mem::replace(&mut q.source,gcs_core::solid::Source::Exact) {
            gcs_core::solid::Source::Warm {hints,..} => hints,_ => unreachable!(),
        };
        q = warm(p,hints);
        let (warm,cold) = (field.query(p,&mut q),field.query(p,&mut exact(p)));
        assert!((warm.value-cold.value).abs() < 1e-9,"{p:?}: warm {} cold {}",warm.value,cold.value);
        assert!((0..3).all(|j| (warm.gradient[j]-cold.gradient[j]).abs() < 1e-5),"{p:?}");
        assert!((warm.time().unwrap()-cold.time().unwrap()).abs() < 1e-4,"{p:?}: {:?} {:?}",warm.time(),cold.time());
    }
    // A hint at a local minimum that is not the least: near the +60° cap, warm from the -60° one.
    let limit = std::f64::consts::PI/3.;
    let p = [1.2,2.9,0.4];
    let warm = field.query(p,&mut warm(p,vec![Some(-limit)]));
    let cold = field.query(p,&mut exact(p));
    assert!((warm.value-cold.value).abs() < 1e-9 && (warm.time().unwrap()-limit).abs() < 1e-4,"{warm:?} {cold:?}");
}
