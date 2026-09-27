//! Volume-first experiments. A sampled counterexample can disprove an envelope
//! boundary; absence of a sampled counterexample is not a continuous-sweep proof.
use super::*;
use gcs_core::{solid::{RevolvedRegion,PlanarField,RevolvedField,SpatialField,SweptField,SweepEvaluator},interval::{Interval as I,minimum::{self,Options,Status}}};

mod closure;
mod source;
mod member;

#[derive(Clone,Copy)]
enum ProfileSide { Inner, Outer }

/// Explicit support construction of this rounded generating section. This is not
/// a general conversion of arbitrary Solvent profiles. The emitted coefficients
/// specify the functional set independently of the numerical envelope workbench.
fn functional_profile(pair: &Pair,rack: &str) -> (PlanarField,[f64;3],[f64;3],String) {
    functional_profile_with_side(pair,rack,None)
}

fn functional_profile_with_side(pair: &Pair,rack: &str,active: Option<ProfileSide>)
    -> (PlanarField,[f64;3],[f64;3],String) {
    let active = active.map(|side| match side { ProfileSide::Inner => "inner", ProfileSide::Outer => "outer" });
    let sk = &pair.model.sketch;
    let axis_name = if rack == "pinion" { "pair.crown_front_axis" } else { "pair.crown_back_axis" };
    let axis = &sk.lines[pair.model.map.ent_named(axis_name).unwrap().i()];
    let origin = sk.world_point(axis.p1 as usize);
    let end = sk.world_point(axis.p2 as usize);
    let axis: [f64;3] = std::array::from_fn(|k| end[k]-origin[k]);
    let length = axis[0].hypot(axis[1]).hypot(axis[2]);
    let unit = axis.map(|v| v/length);
    let coordinate = |id: u32| {
        let p = sk.world_point(id as usize);
        let q: [f64;3] = std::array::from_fn(|k| p[k]-origin[k]);
        let z = (0..3).map(|k| q[k]*unit[k]).sum::<f64>();
        let r: [f64;3] = std::array::from_fn(|k| q[k]-z*unit[k]);
        [r[0].hypot(r[1]).hypot(r[2]),z]
    };
    let mut planes = vec![];
    let mut fields = vec![];
    for name in ["base","outer","tip","inner"] {
        if active.is_some_and(|side| (name == "outer" || name == "inner") && name != side) { continue; }
        let line = &sk.lines[pair.model.map.ent_named(&format!("pair.{rack}.{name}")).unwrap().i()];
        let a = coordinate(line.p1); let b = coordinate(line.p2);
        // This profile is traversed counterclockwise in (radius,height).
        let normal = [b[1]-a[1],a[0]-b[0]];
        fields.push(PlanarField::half_plane(a,normal).unwrap());
        planes.push(format!("{{\"through\":{a:?},\"normal\":{normal:?}}}"));
    }
    let mut corners = vec![];
    for name in ["outer_round","inner_round"] {
        if active.is_some_and(|side| !name.starts_with(side)) { continue; }
        let arc = &sk.arcs[pair.model.map.ent_named(&format!("pair.{rack}.{name}")).unwrap().i()];
        let center = coordinate(arc.center);
        let radius = sk.params[arc.radius as usize].value;
        let [a,b] = [arc.start,arc.end].map(|id| {
            let p = coordinate(id); [p[0]-center[0],p[1]-center[1]]
        });
        assert!(a[0]*b[1]-a[1]*b[0] > 0.,"this construction requires minor CCW corners");
        let n0 = [-a[1],a[0]]; let n1 = [b[1],-b[0]];
        let cap = PlanarField::disk(center,radius).unwrap()
            .union(PlanarField::half_plane(center,n0).unwrap()).unwrap()
            .union(PlanarField::half_plane(center,n1).unwrap()).unwrap();
        fields.push(cap);
        corners.push(format!("{{\"center\":{center:?},\"radius\":{radius},\"sector_normals\":[{n0:?},{n1:?}]}}"));
    }
    let profile = fields.into_iter().reduce(|a,b| a.intersection(b).unwrap()).unwrap();
    let definition = format!("{{\"name\":\"{rack}\",\"half_planes\":[{}],\"rounded_corners\":[{}]}}",planes.join(","),corners.join(","));
    (profile,origin,axis,definition)
}

fn functional_generator(pair: &Pair,member: usize) -> (RevolvedField,String) {
    let racks = if member == 0 { vec!["pinion"] } else { vec!["gear_outer","gear_inner"] };
    let mut profiles = racks.into_iter().map(|name| functional_profile(pair,name));
    let (mut profile,origin,axis,first) = profiles.next().unwrap();
    let mut definitions = vec![first];
    for (other,o,a,definition) in profiles {
        // The source declares the same axis for these sections. Union before
        // revolution and sweep preserves both material volumes exactly.
        assert_eq!(origin,o); assert_eq!(axis,a);
        profile = profile.union(other).unwrap();
        definitions.push(definition);
    }
    let definition = format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"profiles\":[{}]}}",definitions.join(","));
    (RevolvedField::new(profile,origin,axis).unwrap(),definition)
}

#[test]
fn explicit_functional_generator_matches_source_material_at_sampled_points() {
    let pair = Pair::read([24,48],2.);
    let mut count = 0;
    for rack in ["pinion","gear_outer","gear_inner"] {
        let crown = pair.model.map.ent_named(&format!("pair.{rack}_crown")).unwrap();
        let source = RevolvedRegion::read(&pair.model.sketch,crown.i(),pair.module*1e-10).unwrap();
        let (profile,origin,axis,_) = functional_profile(&pair,rack);
        let field = RevolvedField::new(profile,origin,axis).unwrap();
        let [cx,cy,_] = pair.offset;
        let r = 0.8*pair.rm;
        for ir in -30..=30 { for iz in -30..=30 {
            for angle in [0_f64,0.7,2.8] {
                let radius = r+ir as f64*pair.module/10.;
                let p = [cx+radius*angle.cos(),cy+radius*angle.sin(),iz as f64*pair.module/10.];
                let a = source.classify(p,0.).unwrap().signed_distance;
                let b = field.bounds(p.map(|v| I::point(v).unwrap())).unwrap().bounds();
                if a > pair.module*1e-8 { assert!(b[0] > 0.,"{rack} {p:?}: {a}, {b:?}"); }
                if a < -pair.module*1e-8 { assert!(b[1] < 0.,"{rack} {p:?}: {a}, {b:?}"); }
                count += 1;
            }
        } }
    }
    assert_eq!(count,33489);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 25 s: whole-roll bounds of both generators")]
fn functional_pair_generators_have_whole_roll_bounds_without_error_assumptions() {
    let pair = Pair::read([24,48],2.);
    let cases: Vec<_> = (0..2).map(|member| sweep_probe(&pair,member)).collect();
    if let Some(path) = std::env::var_os("SOLVENT_SWEEP_PROBE_OUTPUT") {
        std::fs::write(path,format!("{{\"schema\":2,\"cases\":[{}]}}\n",cases.join(",\n"))).unwrap();
    }
}

fn sweep_probe(pair: &Pair,member: usize) -> String {
    let member_name = ["pinion","gear"][member];
    let (field,definition) = functional_generator(pair,member);
    let motion_definition = functional_motion_definition(pair,member);
    let family = &pair.motion_families[member];
    let frame = pair.local_frame(member).inverse();
    // Interval bounds apply to this explicit functional generator. Its coefficients
    // are binary64 data extracted from the solved section; correspondence to the
    // intended exact source constraints remains a separate source-error question.
    let value_tolerance = pair.module*1e-4;
    let displacement = pair.module*0.005;
    let mut rows = vec![];
    let mut sweep = sweep_evaluator(pair,member,SpatialField::from(field));
    let mut max_evaluations = 0;
    let started = std::time::Instant::now();
    for side in 0..2 {
        let name = if member == side { "outer" } else { "inner" };
        let flank = pair.patch(member,side,name);
        let round = pair.patch(member,side,&format!("{name}_round"));
        let surface_id = pair.model.map.ent_named(&flank.name).unwrap().i();
        let crown = pair.model.sketch.surfaces[surface_id].solid as usize;
        let region = RevolvedRegion::read(&pair.model.sketch,crown,pair.module*1e-10).unwrap();
        for fraction in [0.9,1.,1.1] {
            let rho = fraction*pair.rm;
            let tip = pair.tip(member,&flank,rho,pair.analytic(member,&flank,0.5,rho).parameters);
            let join = pair.seam(&flank,false).endpoint_parameters()[0];
            for (surface,us) in [(&flank,[tip.parameters[0],(tip.parameters[0]+join)/2.,join]),
                (&round,[0.,0.5,1.])] {
                for u in us {
                    let p = pair.analytic(member,surface,u,rho);
                    let value = |x,t| region.classify(family.at(t).unwrap().inverse().point(x),0.)
                        .unwrap().signed_distance;
                    let position = frame.point(p.contact.position);
                    assert!(value(position,p.parameters[2]).abs() < pair.module*1e-8);
                    let normal = frame.vector(p.contact.normal);
                    let plus = std::array::from_fn(|k| position[k]+displacement*normal[k]);
                    let sign = value(plus,p.parameters[2]).signum();
                    assert!(value(plus,p.parameters[2]).abs() > displacement*0.9,
                        "{} u={u} face={fraction}: offset value {}, contact {:?}, normal {normal:?}",
                        surface.name,value(plus,p.parameters[2]),p.parameters);
                    for offset in [-displacement,0.,displacement] {
                        let position = std::array::from_fn(|k| position[k]+sign*offset*normal[k]);
                        let domain = pair.domain(surface)[2];
                        assert_eq!(sweep.domain().bounds(),domain);
                        let found = sweep.bounds(position.map(|v| I::point(v).unwrap()),
                            Options {value_tolerance,max_evaluations:100000}).unwrap();
                        assert_eq!(found.status,Status::Converged,"{} at {u}: {found:?}",surface.name);
                        let [lower,upper] = found.value.bounds();
                        if offset < 0. { assert!(upper < 0.,"{found:?}"); }
                        else if offset > 0. { assert!(lower > 0.,"{found:?}"); }
                        else { assert!(lower >= -value_tolerance && upper <= value_tolerance,"{found:?}"); }
                        max_evaluations = max_evaluations.max(found.evaluations);
                        rows.push(format!("{{\"surface\":\"{}\",\"face_fraction\":{fraction},\"u\":{u},\"position_mm\":{position:?},\"offset_mm\":{offset},\"minimum\":[{lower},{upper}],\"roll_domain\":{domain:?},\"roll_witness\":{},\"evaluations\":{}}}",
                            surface.name,found.witness,found.evaluations));
                    }
                }
            }
        }
    }
    assert_eq!(rows.len(),108);
    let seconds = started.elapsed().as_secs_f64();
    eprintln!("{member_name} functional whole-roll bounds: {} points, max {max_evaluations} evaluations, {} interval poses, {seconds:.3}s",rows.len(),sweep.cached_poses());
    format!("{{\"schema\":2,\"member\":\"{member_name}\",\"units\":{{\"length\":\"mm\",\"angle\":\"rad\"}},\"status\":\"interval bounds for explicit functional generator\",\"source_error\":\"not certified\",\"definition\":{definition},\"motion_definition\":{motion_definition},\"motion\":\"outward interval pose and speed bounds\",\"interval_poses\":{},\"value_tolerance_mm\":{value_tolerance},\"seconds\":{seconds},\"points\":[{}]}}",sweep.cached_poses(),rows.join(",\n"))
}

fn sweep_evaluator(pair: &Pair,member: usize,source: SpatialField) -> SweepEvaluator {
    let patch = pair.patch(member,0,if member == 0 { "outer" } else { "inner" });
    let domain = pair.domain(&patch)[2];
    SweptField::new(source,pair.motion_families[member].clone(),I::new(domain[0],domain[1]).unwrap())
        .evaluator(1_000_000)
}

#[test]
fn indexed_full_crown_union_exposes_neighboring_gear_overcut() {
    let pair = Pair::read([24,48],2.);
    let mut count = 0;
    let mut covered = vec![];
    let mut cases = vec![];
    for member in 0..2 {
        let (field,definition) = functional_generator(&pair,member);
        let mut rows = vec![];
        let family = &pair.motion_families[member];
        let frame = pair.local_frame(member).inverse();
        let mut sweep = sweep_evaluator(&pair,member,SpatialField::from(field));
        for side in 0..2 {
            let edge = if member == side { "outer" } else { "inner" };
            let flank = pair.patch(member,side,edge);
            let tip = pair.tip(member,&flank,pair.rm,pair.analytic(member,&flank,0.5,pair.rm).parameters);
            let join = pair.seam(&flank,false).endpoint_parameters()[0];
            let contact = pair.analytic(member,&flank,(tip.parameters[0]+join)/2.,pair.rm);
            let surface_id = pair.model.map.ent_named(&flank.name).unwrap().i();
            let crown = pair.model.sketch.surfaces[surface_id].solid as usize;
            let region = RevolvedRegion::read(&pair.model.sketch,crown,pair.module*1e-10).unwrap();
            let displacement = pair.module*0.005;
            let plus = std::array::from_fn(|k| contact.contact.position[k]+displacement*contact.contact.normal[k]);
            let sign = region.classify(family.at(contact.parameters[2]).unwrap().inverse().point(frame.point(plus)),0.)
                .unwrap().signed_distance.signum();
            let material = std::array::from_fn(|k| contact.contact.position[k]+sign*displacement*contact.contact.normal[k]);
            for index in 0..pair.teeth[member] as usize {
                // Query an inverse-indexed point against the original sweep.
                // Each resulting binary64 point is exact query data; transfer
                // from ideal tooth indexing remains a source-error question.
                let p = frame.point(rotate(2,-TAU*index as f64/pair.teeth[member],0.).point(material));
                assert_eq!(sweep.domain().bounds(),pair.domain(&flank)[2]);
                let found = sweep.bounds(p.map(|v| I::point(v).unwrap()),
                    Options {value_tolerance:pair.module*1e-4,max_evaluations:100000}).unwrap();
                assert_eq!(found.status,Status::Converged,"member {member}, side {side}, index {index}: {found:?}");
                let [lower,upper] = found.value.bounds();
                if lower <= 0. {
                    // This construction is deliberately retained as a rejected
                    // candidate: its inactive outer wall cuts a neighboring tooth.
                    assert!(upper < -0.018,"expected a definite overcut: {found:?}");
                    covered.push((member,side,index));
                }
                let domain = pair.domain(&flank)[2];
                let u = (tip.parameters[0]+join)/2.;
                rows.push(format!("{{\"surface\":\"{}\",\"face_fraction\":1,\"u\":{u},\"index\":{index},\"position_mm\":{p:?},\"offset_mm\":{displacement},\"minimum\":[{lower},{upper}],\"roll_domain\":{domain:?},\"roll_witness\":{},\"evaluations\":{}}}",flank.name,found.witness,found.evaluations));
                count += 1;
            }
        }
        let member_name = ["pinion","gear"][member];
        let motion_definition = functional_motion_definition(&pair,member);
        let value_tolerance = pair.module*1e-4;
        cases.push(format!("{{\"schema\":2,\"member\":\"{member_name}\",\"units\":{{\"length\":\"mm\",\"angle\":\"rad\"}},\"status\":\"rejected indexed full-crown candidate\",\"source_error\":\"not certified\",\"definition\":{definition},\"motion_definition\":{motion_definition},\"value_tolerance_mm\":{value_tolerance},\"points\":[{}]}}",rows.join(",\n")));
    }
    assert_eq!(count,144);
    assert_eq!(covered,vec![(1,1,47)]);
    if let Some(path) = std::env::var_os("SOLVENT_INDEXED_SWEEP_OUTPUT") {
        std::fs::write(path,format!("{{\"schema\":2,\"cases\":[{}]}}\n",cases.join(",\n"))).unwrap();
    }
}

fn functional_motion_definition(pair: &Pair,member: usize) -> String {
    let member_name = ["pinion","gear"][member];
    let rotation = |name: &str| {
        let id = pair.model.map.ent_named(name).unwrap().i();
        let gcs_core::model::MotionDef::Rotation {axis,ratio,phase,..} = pair.model.sketch.motions[id].def else { panic!() };
        let axis = &pair.model.sketch.lines[axis as usize];
        let origin = pair.model.sketch.world_point(axis.p1 as usize);
        let end = pair.model.sketch.world_point(axis.p2 as usize);
        let axis: [f64;3] = std::array::from_fn(|k| end[k]-origin[k]);
        format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"ratio\":{ratio},\"phase\":{phase}}}")
    };
    format!("{{\"source\":{},\"observer\":{}}}",
        rotation("pair.crown_roll"),rotation(&format!("pair.{member_name}_roll")))
}
