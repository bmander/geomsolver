//! Connect native sharp-edge candidates to the full declared motion interval.
use super::*;
use gcs_core::{interval::minimum::Options,model::SolidDef,solid::{MaterialField,ProbeState}};

#[test]
fn native_candidates_are_checked_against_the_complete_declared_sweep() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = native::Session::new().unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let SolidDef::Swept {source,motion,from,to} = &e.sketch.solids[id].def else { panic!() };
        let native = cad.construct(&cad::recipe(&e.sketch,*source as usize).unwrap()).unwrap();
        let motion = Family::read(&e.sketch,*motion as usize).unwrap();
        let source = SpatialField::read(&e.sketch,*source as usize,1e-10/scale).unwrap();
        let mut material = MaterialField::read(&e.sketch,id,1e-10/scale).unwrap().evaluator(2048);
        let distance = 0.01/scale;
        let options = Options {value_tolerance:distance/8.,max_evaluations:20000};
        let mut states = Vec::new();
        let mut evaluations = 0;
        for row in boundary(&cad,native) {
            if row[3]&2 != 0 { continue; }
            let s = sample(&cad,&row,0.37).unwrap();
            let edge = EdgePoint {position:v(&s,0).map(|x| x/scale),..edge(&s)};
            for time in [-0.3,0.,0.3] {
                let candidate = match envelope::edge_contact(edge,motion.at(time).unwrap(),1e-6,1e-10/scale) {
                    Ok(Some(c)) => c,
                    Ok(None) | Err(envelope::Error::Degenerate) => continue,
                    Err(error) => panic!("{member}: {error:?}"),
                };
                let result = material.probe(candidate.position.map(|x| Interval::point(x).unwrap()),
                    candidate.normal,distance,options).unwrap();
                assert!(matches!(result.state,ProbeState::OutwardBracket | ProbeState::InteriorBall),
                    "{member}, edge {}, roll {time}: {:?}, center {:?}, sides {:?}",row[0],result.state,
                    result.center.value.bounds(),result.sides.as_ref().map(|s| s.each_ref().map(|q| q.value.bounds())));
                for bounds in std::iter::once(&result.center).chain(result.sides.iter().flatten()) {
                    assert_eq!(bounds.sweeps.len(),1);
                    for query in &bounds.sweeps {
                        assert_eq!(query.domain.bounds(),[from.value,to.value]);
                        evaluations += query.minimum.evaluations;
                    }
                }
                if result.state == ProbeState::InteriorBall {
                    // Recheck the returned covering pose with the separate static
                    // source path, without minimization or the sweep pose cache.
                    let witness = result.center.sweeps[0].minimum.witness;
                    assert!((witness-time).abs() > 1e-3);
                    let p = motion.at(witness).unwrap().inverse().point(candidate.position);
                    let value = source.bounds(p.map(|x| Interval::point(x).unwrap())).unwrap();
                    assert!(value.bounds()[1] < -distance);
                }
                states.push(result.state);
            }
        }
        let outward = states.iter().filter(|s| **s == ProbeState::OutwardBracket).count();
        let interior = states.iter().filter(|s| **s == ProbeState::InteriorBall).count();
        assert_eq!((outward,interior),if member == "pinion" { (6,0) } else { (8,4) });
        eprintln!("{member}: {outward} outward brackets, {interior} covered candidates, {evaluations} interval evaluations");
    }
}
