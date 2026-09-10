//! Close a gear space against the active surface of its indexed neighbor.
use super::*;
use gcs_core::{motion::Family,solid::SpatialField};

pub(super) struct GearSpace {
    pub(super) field: SpatialField,
    pub(super) definition: String,
}

impl GearSpace {
    pub(super) fn read(pair: &Pair) -> Self {
        let read = |rack,side| {
            let (profile,origin,axis,definition) = functional_profile_with_side(pair,rack,Some(side));
            (RevolvedField::new(profile,origin,axis).unwrap(),
                format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"profiles\":[{definition}]}}"))
        };
        let (outer,a) = read("gear_outer",ProfileSide::Inner);
        let (inner,b) = read("gear_inner",ProfileSide::Outer);
        // With M(t)=B(t)^-1 C(t), delta=-2pi/Nc and theta=2pi/zg,
        // R_body(theta) M(t+delta) = M(t) R_crown(delta).
        // The second side thus belongs to the adjacent material tooth. Its
        // inactive opposite wall is not part of this tooth-space boundary.
        let delta = -TAU/pair.teeth[0].hypot(pair.teeth[1]);
        let mut sk = pair.model.sketch.clone();
        let id = pair.model.map.ent_named("pair.crown_roll").unwrap().i();
        let gcs_core::model::MotionDef::Rotation {axis,ratio,phase,..} = &mut sk.motions[id].def else { panic!() };
        *ratio = 0.; *phase = delta;
        let axis = &sk.lines[*axis as usize];
        let origin = sk.world_point(axis.p1 as usize);
        let end = sk.world_point(axis.p2 as usize);
        let axis: [f64;3] = std::array::from_fn(|k| end[k]-origin[k]);
        let turn = format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"ratio\":0,\"phase\":{delta}}}");
        let neighbor = Family::read(&sk,id).unwrap();
        let field = SpatialField::from(outer).intersection(SpatialField::from(inner)
            .transformed(&neighbor,0.).unwrap()).unwrap();
        Self {field,
            definition:format!("{{\"intersection\":[{a},{{\"under\":{turn},\"field\":{b}}}]}}")}
    }

}

#[test]
fn neighbor_closed_gear_space_matches_selected_boundary_and_interior() {
    let pair = Pair::read([24,48],2.);
    let field = GearSpace::read(&pair);
    let family = &pair.motion_families[1];
    let teeth = pair.teeth[1] as usize;
    let frame = pair.local_frame(1).inverse();
    let mut sweep = sweep_evaluator(&pair,1,field.field.clone());
    let mut rows = vec![];
    let tolerance = pair.module*1e-4;
    let displacement = pair.module*0.005;
    let started = std::time::Instant::now();
    for side in 0..2 {
        let edge = if side == 0 { "inner" } else { "outer" };
        let flank = pair.patch(1,side,edge);
        let round = pair.patch(1,side,&format!("{edge}_round"));
        let surface_id = pair.model.map.ent_named(&flank.name).unwrap().i();
        let crown = pair.model.sketch.surfaces[surface_id].solid as usize;
        let region = RevolvedRegion::read(&pair.model.sketch,crown,pair.module*1e-10).unwrap();
        for fraction in [0.9,1.,1.1] {
            let rho = fraction*pair.rm;
            let tip = pair.tip(1,&flank,rho,pair.analytic(1,&flank,0.5,rho).parameters);
            let join = pair.seam(&flank,false).endpoint_parameters()[0];
            for (surface,us) in [(&flank,[tip.parameters[0],(tip.parameters[0]+join)/2.,join]),
                (&round,[0.,0.5,1.])] {
                for (station,u) in us.into_iter().enumerate() {
                    let p = pair.analytic(1,surface,u,rho);
                    let plus = std::array::from_fn(|k| p.contact.position[k]+displacement*p.contact.normal[k]);
                    let sign = region.classify(family.at(p.parameters[2]).unwrap().inverse().point(frame.point(plus)),0.)
                        .unwrap().signed_distance.signum();
                    for offset in [-displacement,0.,displacement] {
                        let material = std::array::from_fn(|k| p.contact.position[k]+sign*offset*p.contact.normal[k]);
                        let own = if side == 0 { 0 } else { teeth-1 };
                        // Test every indexed copy at the middle working-flank
                        // station across toe/mean/heel. All other flank/fillet
                        // witnesses test the gap adjacent to their active side.
                        let indices = if offset > 0. && surface.name == flank.name && station == 1 {
                            (0..teeth).collect::<Vec<_>>()
                        } else { vec![own] };
                        for index in indices {
                            let position = frame.point(rotate(2,-TAU*index as f64/pair.teeth[1],0.).point(material));
                            let domain = pair.domain(surface)[2];
                            assert_eq!(sweep.domain().bounds(),domain);
                            let found = sweep.bounds(position.map(|v| I::point(v).unwrap()),
                                Options {value_tolerance:tolerance,max_evaluations:100000}).unwrap();
                            assert_eq!(found.status,Status::Converged,"{} face {fraction} u {u} index {index}: {found:?}",surface.name);
                            let [lower,upper] = found.value.bounds();
                            if offset < 0. { assert!(upper < 0.,"lost intended space: {} face {fraction} u {u}: {found:?}",surface.name); }
                            else if offset > 0. { assert!(lower > 0.,"overcut: {} face {fraction} u {u} index {index}: {found:?}",surface.name); }
                            else { assert!(lower >= -tolerance && upper <= tolerance,"lost boundary: {} face {fraction} u {u}: {found:?}",surface.name); }
                            rows.push(format!("{{\"surface\":\"{}\",\"face_fraction\":{fraction},\"u\":{u},\"index\":{index},\"position_mm\":{position:?},\"offset_mm\":{offset},\"minimum\":[{lower},{upper}],\"roll_domain\":{domain:?},\"roll_witness\":{},\"evaluations\":{}}}",surface.name,found.witness,found.evaluations));
                        }
                    }
                }
            }
        }
    }
    // The independently specified root cone must remain exposed across the
    // space, and points between that floor and the addendum must be removed.
    // These interior witnesses catch a closure that preserves the two sides
    // but leaves an unwanted ridge between them; they are not spatial coverage.
    for fraction in [0.9,1.,1.1] {
        let rho = fraction*pair.rm;
        let normal_module = pair.module*35_f64.to_radians().cos();
        let root_theta = pair.delta[1]-(1.25*normal_module/rho).asin();
        let tip_theta = pair.delta[1]+(normal_module/rho).asin();
        let outer = pair.patch(1,0,"inner_round");
        let inner = pair.patch(1,1,"outer_round");
        let a = pair.analytic(1,&outer,0.,rho).contact.position;
        let b = rotate(2,TAU/pair.teeth[1],0.).point(pair.analytic(1,&inner,1.,rho).contact.position);
        let first = a[1].atan2(a[0]);
        let span = (b[1].atan2(b[0])-first).rem_euclid(TAU);
        assert!(span > 0. && span < TAU/pair.teeth[1]);
        for u in [0.1,0.5,0.9] {
            let phi = first+u*span;
            for distance in [-displacement,0.,displacement,
                0.5*(tip_theta-root_theta)*rho,0.9*(tip_theta-root_theta)*rho] {
                let theta = root_theta+distance/rho;
                let position = frame.point([rho*theta.sin()*phi.cos(),rho*theta.sin()*phi.sin(),rho*theta.cos()]);
                let domain = pair.domain(&outer)[2];
                assert_eq!(sweep.domain().bounds(),domain);
                let found = sweep.bounds(position.map(|v| I::point(v).unwrap()),
                    Options {value_tolerance:tolerance,max_evaluations:100000}).unwrap();
                assert_eq!(found.status,Status::Converged,"floor face {fraction} u {u} distance {distance}: {found:?}");
                let [lower,upper] = found.value.bounds();
                if distance > 0. { assert!(upper < 0.,"unwanted gap material: face {fraction} u {u} distance {distance}: {found:?}"); }
                else if distance < 0. { assert!(lower > 0.,"root overcut: face {fraction} u {u}: {found:?}"); }
                else { assert!(lower >= -tolerance && upper <= tolerance,"lost root floor: face {fraction} u {u}: {found:?}"); }
                let offset = -distance;
                rows.push(format!("{{\"surface\":\"pair.gear_gap_floor\",\"face_fraction\":{fraction},\"u\":{u},\"index\":0,\"position_mm\":{position:?},\"offset_mm\":{offset},\"minimum\":[{lower},{upper}],\"roll_domain\":{domain:?},\"roll_witness\":{},\"evaluations\":{}}}",found.witness,found.evaluations));
            }
        }
    }
    assert_eq!(rows.len(),153+6*(teeth-1));
    eprintln!("neighbor-closed gear space: {} whole-roll queries, {} interval poses, {:.3}s",rows.len(),sweep.cached_poses(),started.elapsed().as_secs_f64());
    if let Some(path) = std::env::var_os("SOLVENT_CLOSED_SPACE_OUTPUT") {
        let definition = &field.definition;
        let motion_definition = functional_motion_definition(&pair,1);
        std::fs::write(path,format!("{{\"schema\":3,\"cases\":[{{\"schema\":3,\"member\":\"gear\",\"units\":{{\"length\":\"mm\",\"angle\":\"rad\"}},\"status\":\"neighbor-closed tooth-space experiment\",\"source_error\":\"not certified\",\"definition\":{definition},\"motion_definition\":{motion_definition},\"value_tolerance_mm\":{tolerance},\"points\":[{}]}}]}}\n",rows.join(",\n"))).unwrap();
    }
}
