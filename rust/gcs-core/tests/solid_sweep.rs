use gcs_core::{io,model::{MotionDef,SolidDef},program,solid::{self,MaterialField},solve,syntax,
    interval::{Interval as I,minimum::{Options,Status}}};

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
    assert!(e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh)
        .unwrap_err().contains("continuous motion sweeps"));
    assert!(solid::cad::recipe(&e.sketch,i).unwrap_err().contains("continuous motion sweeps"));
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
    assert!(e.sketch.evaluated_solid(i,solid::ApproximationPolicy::Mesh).is_err());
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
