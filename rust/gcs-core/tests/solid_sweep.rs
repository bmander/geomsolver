use gcs_core::{io,model::{MotionDef,SolidDef},program,solid::{self,MaterialField},solve,syntax,
    interval::{Interval as I,minimum::{Options,Status}}};

mod contacts;

const SOURCE: &str = "unit mm
point center hint(x: 3,y: 0)
point a hint(x: 3,y: -1)
point b hint(x: 3,y: 1)
point o hint(x: 0,y: 0)
point z hint(x: 0,y: 1)
ground center
ground a
ground b
ground o
ground z
arc rim(center: center,start: a,end: b)
radius(1) rim
line diameter(a,b)
line axis(o,z)
solid tool(face(rim,diameter),about: diameter)
motion generating(about: axis)
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
    let e = read(&format!("solid swept(tool,under: generating,from: -60deg,to: 60deg)\n{SOURCE}"));
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
    solid placed(tool,under: indexing,at: i*360deg/count)
    placed cut target
  }}
}}
circle band(center: center)
radius(2) band
solid stock(face(band),about: axis)
solid body(stock)
solid swept(tool,under: generating,from: -10deg,to: 10deg)
cuts: IndexedCuts(swept,body,generating,count: 3)
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
    let mut e = read(&format!("{SOURCE}solid swept(tool,under: generating,from: 0deg,to: 360deg)\n"));
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
    for bad in ["solid bad(tool,under: generating,from: 1deg)",
        "solid bad(tool,under: generating,to: 1deg)",
        "solid bad(tool,under: generating,from: 0deg,to: 0deg)",
        "solid bad(tool,under: generating,from: 1deg,to: -1deg)",
        "solid bad(tool,under: generating,from: 1mm,to: 2mm)",
        "solid bad(tool,under: generating,from: 0deg,to: 1deg,at: 0deg)",
        "solid bad(tool,under: generating,from: 0deg,to: 1deg,depth: 1mm)",
        "solid bad(tool,under: axis,from: 0deg,to: 1deg)",
        "solid bad(bad,under: generating,from: 0deg,to: 1deg)"] {
        let (p,errors) = syntax::parse(&format!("{SOURCE}{bad}\n"));
        let e = program::elaborate(&p);
        assert!(!errors.is_empty() || !e.ok() ||
            e.sketch.solids.iter().enumerate().any(|(i,_)| solid::validate(&e.sketch,i).is_err()),"{bad}");
    }
    let e = read(&format!("{SOURCE}solid inner_sweep(tool,under: generating,from: -10deg,to: 10deg)\n\
        solid outer_sweep(inner_sweep,under: generating,from: -10deg,to: 10deg)\n"));
    let i = e.map.ent_named("outer_sweep").unwrap().i();
    assert!(MaterialField::read(&e.sketch,i,1e-10).unwrap_err().contains("nested continuous sweeps"));
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
        if done { break; }
    }
    // A preview first, refined over many steps, and a final surface last.
    assert!(seen.len() > 5,"{seen:?}");
    assert!(seen.iter().rev().skip(1).all(|s| s.1) && !seen.last().unwrap().1,"{seen:?}");
    let first = seen.iter().find(|s| s.0 > 0).unwrap().0;
    assert!(first < seen.last().unwrap().0,"{seen:?}");
    let surface = mesher.snapshot();

    // A page meshing elsewhere: the solid is refused until its surface arrives.
    e.sketch.defer_fields.set(true);
    assert!(e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap_err().contains("being meshed"));
    e.sketch.supply_field(i,solid::FieldSurface {provisional:true,..surface.clone()});
    let preview = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap();
    assert!(preview.provisional() && preview.stl().is_err());
    // nor does a scene take a surface still being refined
    assert!(gcs_core::gltf::checked_glb(&e.sketch,&[i],solid::ApproximationPolicy::Mesh).unwrap_err().contains("still being refined"));
    // asked for as a preview, it is written as it stands, every triangle in the file
    let bytes = preview.preview_stl().unwrap();
    assert!(bytes.len() > 84 && (bytes.len()-84)%50 == 0 && u32::from_le_bytes(bytes[80..84].try_into().unwrap()) > 0);
    e.sketch.supply_field(i,surface);
    let done = e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).unwrap();
    let exact = 2.*std::f64::consts::PI.powi(2)+4./3.*std::f64::consts::PI;
    assert!(!done.provisional() && (done.volume()-exact).abs() < 0.03*exact,"{}",done.volume());
    assert!(done.stl().is_ok());
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
        let exact = gcs_core::solid::ReadingOptions {relative:0.,..gcs_core::solid::ReadingOptions::at(p)};
        let r = field.reading_with(p,&exact,&mut 0);
        assert!((r.value-(len-1.)).abs() < 1e-8,"{p:?}: {} against {}",r.value,len-1.);
        for k in 0..3 { assert!((r.gradient[k]-d[k]/len).abs() < 1e-5,"{p:?}: {:?} against {:?}",r.gradient,d.map(|x| x/len)); }
        assert!((r.time.unwrap()-angle).abs() < 1e-5,"{p:?}: time {:?} against {angle}",r.time);
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
    let exact = |p: [f64;3]| gcs_core::solid::ReadingOptions {relative:0.,..gcs_core::solid::ReadingOptions::at(p)};
    for i in 0..9 { for j in 0..9 { for k in 0..5 {
        let p = [-22.+5.5*i as f64+0.3,-22.+5.5*j as f64+0.1,-14.+4.*k as f64+0.2];
        let whole = part.reading_with(p,&exact(p),&mut 0).value;
        let alone = block.reading_with(p,&exact(p),&mut 0).value.max(-groove.reading_with(p,&exact(p),&mut 0).value);
        assert!((whole-alone).abs() < 1e-8,"{p:?}: the part reads {whole}, its operands {alone}");
        if alone.abs() > 1e-6 { assert_eq!(whole < 0.,part.side(p) < 0.,"{p:?}: {whole}"); }
    } } }
}

#[test]
fn a_warm_reading_is_the_cold_one_whatever_its_hint() {
    let e = read(include_str!("../../examples/solid_generating_sweep.sv"));
    let i = e.map.ent_named("removal.body").unwrap().i();
    let field = MaterialField::read(&e.sketch,i,1e-10).unwrap();
    let exact = |p: [f64;3]| gcs_core::solid::ReadingOptions {relative:0.,..gcs_core::solid::ReadingOptions::at(p)};
    // Along a segment through the swept body, each reading warm from the last.
    let mut hints = Vec::new();
    for k in 0..=20 {
        let s = k as f64/20.;
        let p = [4.6-3.*s,-1.+2.8*s,0.3];
        let (warm,cold) = (field.reading_warm(p,&exact(p),&mut 0,&mut hints),field.reading_with(p,&exact(p),&mut 0));
        assert!((warm.value-cold.value).abs() < 1e-9,"{p:?}: warm {} cold {}",warm.value,cold.value);
        assert!((0..3).all(|j| (warm.gradient[j]-cold.gradient[j]).abs() < 1e-5),"{p:?}");
        assert!((warm.time.unwrap()-cold.time.unwrap()).abs() < 1e-4,"{p:?}: {:?} {:?}",warm.time,cold.time);
    }
    // A hint at a local minimum that is not the least: near the +60° cap, warm from the -60° one.
    let limit = std::f64::consts::PI/3.;
    let p = [1.2,2.9,0.4];
    let mut hints = vec![Some(-limit)];
    let warm = field.reading_warm(p,&exact(p),&mut 0,&mut hints);
    let cold = field.reading_with(p,&exact(p),&mut 0);
    assert!((warm.value-cold.value).abs() < 1e-9 && (warm.time.unwrap()-limit).abs() < 1e-4,"{warm:?} {cold:?}");
}
