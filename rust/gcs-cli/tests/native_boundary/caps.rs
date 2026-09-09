//! Endpoint faces keep native trims and orientation before global selection.
use super::*;
use gcs_core::{interval::minimum::Options,model::SolidDef,solid::{MaterialField,ProbeState}};

#[test]
fn bounded_face_samples_respect_holes_and_outward_orientation() {
    let e = support::read(&format!("{BOX}circle c(center: std.origin)\nradius(0.5mm) c\n\
        solid drill(face(c),from: -3mm,to: 3mm)\ndrill cut body\n\
        line axis(std.origin,std.up.toward)\nmotion spin(about: axis)\n\
        solid removal(body,under: spin,from: -20deg,to: 30deg)\n"),Path::new("."));
    let cad = native::Session::new().unwrap();
    let caps = cad.sweep_caps(&e.sketch,e.map.ent_named("removal").unwrap().i()).unwrap();
    let faces = cad.faces(caps.source).unwrap();
    assert_eq!(faces.len(),7);
    for cap in &caps.endpoints { assert_eq!(cap.faces.len(),faces.len()); }
    let mut holes = 0;
    let mut inward_cylinder = 0;
    for face in faces {
        if cad.face_point(face,0.5,0.5,1e-9).unwrap().is_none() { holes += 1; }
        for (u,v) in [(0.13,0.27),(0.63,0.81)] {
            let p = cad.face_point(face,u,v,1e-9).unwrap().unwrap();
            assert!(!p.on_trim);
            assert!((norm(p.normal)-1.).abs() < 1e-12);
            if p.normal[1].abs() < 1e-8 && p.position[0].hypot(p.position[2]) < 0.6 {
                assert!((dot(p.normal,[p.position[0],0.,p.position[2]])+0.5).abs() < 1e-9);
                inward_cylinder += 1;
            } else {
                assert!(dot(p.normal,[p.position[0],p.position[1]-1.,p.position[2]]) > 0.9);
            }
        }
        assert!(cad.face_point(face,-0.1,0.5,1e-9).is_err());
        assert!(cad.face_point(face,0.5,0.5,0.).is_err());
        assert!(cad.face_point(face,f64::NAN,0.5,1e-9).is_err());
    }
    assert_eq!((holes,inward_cylinder),(2,2));
    assert!(cad.face_point(-1,0.5,0.5,1e-9).is_err());
    assert!(cad.sweep_caps(&e.sketch,e.map.ent_named("body").unwrap().i()).is_err());
}

#[test]
fn finite_sphere_caps_match_independent_whole_arc_material() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    for unit in ["mm","in"] {
        let source = include_str!("../../../examples/solid_generating_sweep.sv").replace("unit mm",&format!("unit {unit}"));
        let e = support::read(&source,&base);
        let scale = e.sketch.units.length.unwrap().1;
        let id = e.map.ent_named("removal.body").unwrap().i();
        let cad = native::Session::new().unwrap();
        let start = std::time::Instant::now();
        let caps = cad.sweep_caps(&e.sketch,id).unwrap();
        let mut material = MaterialField::read(&e.sketch,id,1e-10/scale).unwrap().evaluator(1024);
        // Independently minimize distance to the circular center path in mm.
        let signed = |p: [f64;3]| {
            let t = p[1].atan2(p[0]).clamp(-std::f64::consts::PI/3.,std::f64::consts::PI/3.);
            norm([p[0]-3.*t.cos(),p[1]-3.*t.sin(),p[2]])-1.
        };
        let mut states = [0;2];
        for (i,cap) in caps.endpoints.iter().enumerate() {
            assert_eq!(cap.faces.len(),1);
            assert!((cap.parameter-[-1.,1.][i]*std::f64::consts::PI/3.).abs() < 1e-14);
            let center = [3.*cap.parameter.cos(),3.*cap.parameter.sin(),0.];
            for u in [0.13,0.37,0.63,0.87] { for v in [0.23,0.71] {
                let point = cad.face_point(cap.faces[0],u,v,1e-9).unwrap().unwrap();
                let radial = std::array::from_fn(|k| point.position[k]-center[k]);
                assert!((norm(radial)-1.).abs() < 1e-10);
                assert!(dot(radial,point.normal) > 1.-1e-10);
                let result = material.probe(point.position.map(|x| Interval::point(x/scale).unwrap()),
                    point.normal,0.01/scale,Options {value_tolerance:1e-4/scale,max_evaluations:20000}).unwrap();
                if signed(point.position) < -0.01 {
                    assert_eq!(result.state,ProbeState::InteriorBall); states[0] += 1;
                } else {
                    assert!(signed(std::array::from_fn(|k| point.position[k]-0.01*point.normal[k])) < 0.);
                    assert!(signed(std::array::from_fn(|k| point.position[k]+0.01*point.normal[k])) > 0.);
                    assert_eq!(result.state,ProbeState::OutwardBracket); states[1] += 1;
                }
                for b in std::iter::once(&result.center).chain(result.sides.iter().flatten()) {
                    assert_eq!(b.sweeps.len(),1);
                    assert_eq!(b.sweeps[0].domain.bounds(),caps.endpoints.each_ref().map(|c| c.parameter));
                }
            } }
        }
        assert!(states.iter().all(|n| *n > 0));
        eprintln!("{unit} sphere caps: {} covered, {} exposed samples in {:?}",states[0],states[1],start.elapsed());
    }
}

#[test]
fn both_gear_members_retain_trimmed_source_faces_at_the_declared_endpoints() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = native::Session::new().unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let SolidDef::Swept {source,from,to,..} = &e.sketch.solids[id].def else { panic!() };
        let field = SpatialField::read(&e.sketch,*source as usize,1e-10/scale).unwrap();
        let mut material = MaterialField::read(&e.sketch,id,1e-10/scale).unwrap().evaluator(2048);
        let caps = cad.sweep_caps(&e.sketch,id).unwrap();
        assert_eq!(caps.endpoints.each_ref().map(|c| c.parameter),[from.value,to.value]);
        let source_faces = cad.faces(caps.source).unwrap();
        let mut checked = 0;
        let mut states = [0;3];
        for cap in &caps.endpoints {
            assert_eq!(cap.faces.len(),source_faces.len());
            assert_eq!(boundary(&cad,cap.solid).len(),boundary(&cad,caps.source).len());
            for &face in &cap.faces {
                let mut points = Vec::new();
                for u in [0.17,0.5,0.83] { for v in [0.19,0.53,0.81] {
                    if let Some(p) = cad.face_point(face,u,v,1e-9).unwrap() {
                        if !p.on_trim { points.push(p); }
                    }
                } }
                for p in &points {
                    let inverse = cap.pose.inverse();
                    let source = inverse.point(p.position.map(|x| x/scale));
                    let normal = inverse.vector(p.normal);
                    let value = |offset| field.bounds(std::array::from_fn(|k|
                        Interval::point(source[k]+offset*normal[k]).unwrap())).unwrap();
                    let [lo,hi] = value(0.).bounds();
                    assert!(lo.abs().max(hi.abs()) < 1e-6/scale,"{member}: source mismatch {lo}, {hi}");
                    assert!(value(-1e-4/scale).bounds()[1] < 0.,"{member}: inward normal");
                    assert!(value(1e-4/scale).bounds()[0] > 0.,"{member}: outward normal");
                    checked += 1;
                }
                let Some(p) = points.first() else { continue; };
                let proof = material.probe(p.position.map(|x| Interval::point(x/scale).unwrap()),p.normal,
                    0.01/scale,Options {value_tolerance:1e-4/scale,max_evaluations:20000}).unwrap();
                match proof.state {
                    ProbeState::InteriorBall => states[0] += 1,
                    ProbeState::OutwardBracket => states[1] += 1,
                    ProbeState::Unresolved => states[2] += 1,
                    state => panic!("{member}: unexpected endpoint state {state:?}"),
                }
                for b in std::iter::once(&proof.center).chain(proof.sides.iter().flatten()) {
                    assert_eq!(b.sweeps.len(),1);
                    assert_eq!(b.sweeps[0].domain.bounds(),[from.value,to.value]);
                }
            }
        }
        assert!(checked > 30);
        assert!(states[0]+states[1] > 0);
        eprintln!("{member} caps: {checked} native/source checks; covered/exposed/unresolved {states:?}");
    }
}
