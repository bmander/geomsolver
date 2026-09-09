//! Finished source bodies retain the workbench's blank, indexing and cutter sides.
use super::*;

#[test]
fn declarative_matched_pair_agrees_with_independent_member_material() {
    for (teeth,module) in [([24,48],2.),([28,49],1.5)] {
        let pair = Pair::read(teeth,module);
        let e = read_model(include_str!("../../../../../../examples/spiral_bevel/gears.sv"),teeth,module);
        let options = Options {value_tolerance:module*1e-4,max_evaluations:100000};
        let band = I::new(-module*1e-5,module*1e-5).unwrap();
        for member in 0..2 {
            let name = ["pinion","gear"][member];
            let id = e.map.ent_named(&format!("pair.{name}.body")).unwrap().i();
            let field = MaterialField::read(&e.sketch,id,module*1e-10)
                .unwrap_or_else(|error| panic!("{teeth:?} {name}: {error}"));
            let mut source = field.evaluator(100000);
            assert!(source.support_bounds().unwrap().is_some());
            let mut reference = Member::read(&pair,member);
            let frame = pair.local_frame(member).inverse();
            let mut points = vec![([0.;3],false)];
            for side in 0..2 {
                let edge = if member == side { "outer" } else { "inner" };
                let flank = pair.patch(member,side,edge);
                let tip = pair.tip(member,&flank,pair.rm,pair.analytic(member,&flank,0.5,pair.rm).parameters);
                let join = pair.seam(&flank,false).endpoint_parameters()[0];
                let c = pair.analytic(member,&flank,(tip.parameters[0]+join)/2.,pair.rm);
                let s = pair.model.map.ent_named(&flank.name).unwrap().i();
                let region = RevolvedRegion::read(&pair.model.sketch,
                    pair.model.sketch.surfaces[s].solid as usize,module*1e-10).unwrap();
                let displacement = module*0.005;
                let plus = std::array::from_fn(|k| c.contact.position[k]+displacement*c.contact.normal[k]);
                let sign = region.classify(pair.motion_families[member].at(c.parameters[2]).unwrap()
                    .inverse().point(frame.point(plus)),0.).unwrap().signed_distance.signum();
                for offset in [-displacement,displacement] {
                    let local = std::array::from_fn(|k| c.contact.position[k]+sign*offset*c.contact.normal[k]);
                    points.push((frame.point(local),offset > 0.));
                    // A distant index exercises the full repeat, not just tooth zero.
                    let angle = TAU*(teeth[member]/2) as f64/teeth[member] as f64;
                    let rotated = [local[0]*angle.cos()-local[1]*angle.sin(),
                        local[0]*angle.sin()+local[1]*angle.cos(),local[2]];
                    points.push((frame.point(rotated),offset > 0.));
                }
            }
            let normal_module = module*35_f64.to_radians().cos();
            let theta = pair.delta[member]+(-3.*normal_module/pair.rm).asin();
            for (fraction,inside) in [(0.89,false),(1.,true),(1.11,false)] {
                let rho = fraction*pair.rm;
                points.push((frame.point([rho*theta.sin(),0.,rho*theta.cos()]),inside));
            }
            for (p,inside) in points {
                let b = point(p);
                let expected = reference.field.bounds_outside(b,band,options).unwrap().value.bounds();
                let actual = source.bounds_outside(b,band,options).unwrap().value.bounds();
                for (label,value) in [("reference",expected),("source",actual)] {
                    assert!(if inside { value[1] < 0. } else { value[0] > 0. },
                        "{teeth:?} {name}, {p:?}, inside={inside}: {label} {value:?}");
                }
            }
        }
    }
}

#[test]
fn source_contact_curves_reproduce_independent_crown_characteristics() {
    use gcs_core::solid::SweepContacts;
    let teeth = [24,48]; let module = 2.;
    let pair = Pair::read(teeth,module);
    let e = read_model(include_str!("../../../../../../examples/spiral_bevel/gears.sv"),teeth,module);
    let mut checked = 0;
    for member in 0..2 {
        let name = ["pinion","gear"][member];
        let id = e.map.ent_named(&format!("pair.{name}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,module*1e-10).unwrap();
        assert_eq!(sweep.patches().len(),[6,9][member]);
        for side in 0..2 {
            let edge = if member == side { "outer" } else { "inner" };
            for edge in [edge.to_string(),format!("{edge}_round")] {
                let patch = pair.patch(member,side,&edge);
                for u in [0.1,0.3,0.5,0.7,0.9] { for rho in [0.9*pair.rm,pair.rm,1.1*pair.rm] {
                    let expected = pair.analytic(member,&patch,u,rho);
                    let mut roll = expected.parameters[2];
                    let mut local = expected.contact.position;
                    if member == 1 && side == 1 {
                        // R_body(theta) M(t+delta) = M(t) R_crown(delta).
                        // The second active face belongs to the indexed neighbor.
                        roll += TAU/pair.teeth[0].hypot(pair.teeth[1]);
                        let theta = TAU/pair.teeth[1];
                        local = [local[0]*theta.cos()-local[1]*theta.sin(),
                            local[0]*theta.sin()+local[1]*theta.cos(),local[2]];
                    }
                    let world = pair.local_frame(member).inverse().point(local);
                    let mut distance = f64::INFINITY;
                    let mut temporal_match = false;
                    for i in 0..sweep.patches().len() {
                        match sweep.at(i,u,roll,module*1e-10) {
                            Ok(roots) => for root in roots {
                                let p = root.contact.position;
                                let gap = (p[0]-world[0]).hypot(p[1]-world[1]).hypot(p[2]-world[2]);
                                distance = distance.min(gap);
                                if gap < module*1e-8 {
                                    let times = sweep.at_source(i,u,root.v,module*1e-10).unwrap();
                                    temporal_match |= times.iter().any(|r| {
                                        let p = r.contact.position;
                                        (r.root.time-roll).abs() < 1e-8
                                            && (p[0]-world[0]).hypot(p[1]-world[1]).hypot(p[2]-world[2]) < module*1e-8
                                    });
                                }
                            },
                            Err(gcs_core::envelope::Error::Degenerate) => {},
                            Err(error) => panic!("{name}: {error:?}"),
                        }
                    }
                    assert!(distance < module*1e-8,"{name} {edge} u={u}, rho={rho}: {distance}");
                    assert!(temporal_match,"{name} {edge} u={u}, rho={rho}: temporal chart");
                    checked += 1;
                } }
            }
        }
    }
    assert_eq!(checked,120);
}
