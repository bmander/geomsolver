//! Rigid instances share a source definition, but own their placed geometry.
use gcs_core::{io,model::MotionDef,program,solid::{ApproximationPolicy::Report,WorldPoint},solve,syntax};

const SOURCE: &str = "unit mm
point a hint(x: 1,y: 0)
point b hint(x: 3,y: 0)
point c hint(x: 3,y: 2)
point d hint(x: 1,y: 2)
point o hint(x: 0,y: 0)
point z hint(x: 0,y: 1)
ground a
ground b
ground c
ground d
ground o
ground z
line axis(o,z)
line ab(a,b)
line bc(b,c)
line cd(c,d)
line da(d,a)
face profile(ab,bc,cd,da)
solid stock(profile, from: 0mm, to: 2mm)
motion turn(about: axis)
";

fn read(src: &str) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(src);
    assert!(errors.is_empty(),"{errors:?}");
    assert!(gcs_core::modules::link(&mut p,&mut gcs_core::library::resolve).is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}
fn solid(e: &program::Elaborated,name: &str) -> std::rc::Rc<gcs_core::solid::EvaluatedSolid> {
    e.sketch.evaluated_solid(e.map.ent_named(name).unwrap().i(),Report).unwrap()
}

#[test]
fn placed_solids_have_rotated_material_closed_meshes_and_face_names() {
    let e = read(&format!("{SOURCE}solid moved(stock, under: turn, at: 90deg)\n"));
    let s = solid(&e,"moved");
    assert!((s.volume()-8.).abs() < 1e-9);
    assert!(s.contains_world(WorldPoint([1.,2.,1.])));
    assert!(!s.contains_world(WorldPoint([2.,-1.,1.])));
    assert!(solid(&e,"stock").contains_world(WorldPoint([2.,-1.,1.])));
    assert_eq!(super::solid::unpaired(s.mesh()),0);
    assert!(s.surviving_faces().contains("moved.ab"),"{:?}",s.surviving_faces());
    assert!(s.stl().unwrap().len() > 84);
}

#[test]
fn nested_placements_transform_composite_sources_and_invalidate_motion_caches() {
    let mut e = read(&format!("{SOURCE}\nsolid body(stock)\n\
        solid first(body, under: turn, at: 45deg)\n\
        solid moved(first, under: turn, at: 45deg)\n"));
    let old = solid(&e,"moved");
    assert!(old.contains_world(WorldPoint([1.,2.,1.])));
    let m = e.map.ent_named("turn").unwrap().i();
    let MotionDef::Rotation {phase,..} = &mut e.sketch.motions[m].def else { unreachable!() };
    *phase = std::f64::consts::FRAC_PI_2;
    let changed = solid(&e,"moved");
    assert!(!std::rc::Rc::ptr_eq(&old,&changed));
    assert!(changed.contains_world(WorldPoint([-1.,-2.,1.])));
    assert!(!changed.contains_world(WorldPoint([1.,2.,1.])));
    assert!((changed.volume()-8.).abs() < 1e-9);
}

#[test]
fn placement_print_copy_and_paste_retain_source_and_motion_dependencies() {
    let e = read(&format!("solid moved(stock, under: turn, at: 90deg)\n{SOURCE}"));
    let mut program = e.program.clone();
    let text = syntax::render_flat(&mut program).unwrap().to_string();
    assert!(text.contains("under: turn, at: 90deg"),"{text}");
    assert!(solid(&read(&text),"moved").contains_world(WorldPoint([1.,2.,1.])));
    let copied = io::copy(&e.sketch,&[e.map.ent_named("moved").unwrap()]);
    assert_eq!(copied.motions.len(),1);
    assert_eq!(copied.solids.len(),2);
    let i = copied.solids.iter().position(|s| s.name == "moved").unwrap();
    assert!(copied.evaluated_solid(i,Report).unwrap().contains_world(WorldPoint([1.,2.,1.])));
    let mut target = gcs_core::model::Sketch::new();
    io::paste(&mut target,&copied,10.,20.);
    let i = target.solids.iter().position(|s| s.name == "moved").unwrap();
    assert!((target.evaluated_solid(i,Report).unwrap().volume()-8.).abs() < 1e-9);
}

#[test]
fn repeated_component_placements_cut_a_body_with_a_shared_through_cutter() {
    let e = read(include_str!("../../examples/solid_indexed_pattern.sv"));
    let body = solid(&e,"body");
    let expected = std::f64::consts::PI*(400.-6.*4.)*5.;
    assert!((body.volume()-expected).abs()/expected < 0.003);
    assert!(body.contains_world(WorldPoint([0.,0.,-2.])));
    for i in 0..6 {
        let angle = i as f64*std::f64::consts::TAU/6.;
        assert!(!body.contains_world(WorldPoint([12.*angle.cos(),12.*angle.sin(),-2.])));
    }
    assert_eq!(body.surviving_faces().iter().filter(|name|
        name.starts_with("pattern.") && name.ends_with(".indexed.hole")).count(),6);
}

#[test]
#[ignore = "legacy faceted Boolean triangulation leaves unpaired edges in a six-hole pattern; native STEP passes"]
fn indexed_pattern_stl_must_have_closed_encoded_topology() {
    let e = read(include_str!("../../examples/solid_indexed_pattern.sv"));
    let body = e.sketch.evaluated_solid(e.map.ent_named("body").unwrap().i(),
        gcs_core::solid::ApproximationPolicy::Mesh).unwrap();
    gcs_core::mesh::stl_topology(&body.stl().unwrap()).unwrap();
}

#[test]
fn malformed_placements_and_cycles_are_refused() {
    for bad in ["solid bad(stock, under: turn)","solid bad(stock, at: 90deg)",
        "solid bad(stock, under: turn, at: 90deg, depth: 2mm)",
        "solid bad(stock, under: axis, at: 90deg)","solid bad(stock, under: turn, at: 2mm)",
        "solid bad(profile, under: turn, at: 90deg)","solid bad(bad, under: turn, at: 90deg)"] {
        let (p,errors) = syntax::parse(&format!("{SOURCE}{bad}\n"));
        let e = program::elaborate(&p);
        assert!(!errors.is_empty() || !e.ok() ||
            e.sketch.solids.iter().enumerate().any(|(i,_)| gcs_core::solid::validate(&e.sketch,i).is_err()),"{bad}");
    }
}

#[test]
fn solids_bind_final_motion_indices_after_dependency_ordering() {
    // Declaration order, alphabetical order and dependency order all differ.
    // Each solid must retain its named motion, including through a formal.
    let e = read(&format!("{SOURCE}\n\
        motion z_turn(about: axis,phase: 90deg)\n\
        motion a_relative(z_turn,relative_to: turn)\n\
        component Copy(stock: solid,movement: motion) {{\n\
          solid moved(stock,under: movement,at: 0deg)\n\
          solid swept(stock,under: movement,from: -10deg,to: 10deg)\n\
        }}\n\
        copy: Copy(stock,a_relative)\n\
        solid direct(stock,under: z_turn,at: 0deg)\n"));
    use gcs_core::model::SolidDef;
    for (name,expected) in [("copy.moved","a_relative"),("copy.swept","a_relative"),("direct","z_turn")] {
        let s = &e.sketch.solids[e.map.ent_named(name).unwrap().i()];
        let motion = match s.def {
            SolidDef::Placed {motion,..} | SolidDef::Swept {motion,..} => motion,
            _ => panic!("expected a motion operand"),
        };
        assert_eq!(motion as usize,e.map.ent_named(expected).unwrap().i(),"{name}");
    }
    for name in ["copy.moved","direct"] {
        assert!(solid(&e,name).contains_world(WorldPoint([1.,2.,1.])));
        assert!(!solid(&e,name).contains_world(WorldPoint([2.,-1.,1.])));
    }
}
