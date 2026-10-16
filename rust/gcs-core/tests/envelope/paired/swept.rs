//! Volume-first experiments. A sampled counterexample can disprove an envelope
//! boundary; absence of a sampled counterexample is not a continuous-sweep proof.
use super::*;
use gcs_core::{solid::{RevolvedRegion,PlanarField,RevolvedField,SpatialField,SweptField,SweepEvaluator},interval::{Interval as I,minimum::{self,Options,Status}}};

mod closure;
mod source;
mod member;

#[derive(Clone,Copy)]
enum ProfileSide { Inner, Outer }

/// A crown's meridian about the cutter's axis (`cut_ax`, up from C) in (radius, height), as
/// pieces of a functional profile: a half-plane per line (through a point, its normal outward) and
/// a rounded corner per minor arc (its centre and radius and the two radii to its ends,
/// counterclockwise), each by its face's name.  A step on the axis bounds nothing a revolution
/// does not, and is left out.
struct Pieces { origin: [f64;3], axis: [f64;3], planes: Vec<(String,[f64;2],[f64;2])>,
    corners: Vec<(String,[f64;2],f64,[f64;2],[f64;2])> }

impl Pieces {
    fn read(pair: &Pair,rack: &str) -> Self {
        use gcs_core::brep::planar::Seg;
        let sk = &pair.model.sketch;
        let ax = &sk.lines[pair.model.map.ent_named("pair.mill.ax").unwrap().i()];
        let origin = sk.world_point(ax.p1 as usize);
        let end = sk.world_point(ax.p2 as usize);
        let axis: [f64;3] = std::array::from_fn(|k| end[k]-origin[k]);
        let length = axis[0].hypot(axis[1]).hypot(axis[2]);
        let unit = axis.map(|v| v/length);
        let coordinate = |p: [f64;3]| {
            let q: [f64;3] = std::array::from_fn(|k| p[k]-origin[k]);
            let z = (0..3).map(|k| q[k]*unit[k]).sum::<f64>();
            let r: [f64;3] = std::array::from_fn(|k| q[k]-z*unit[k]);
            [r[0].hypot(r[1]).hypot(r[2]),z]
        };
        let crown = pair.model.map.ent_named(Pair::crown(rack)).unwrap().i();
        let m = gcs_core::solid::region::meridian(sk,crown).unwrap();
        let basis = gcs_core::plane::Basis {u: m.seam,v: m.axis,o: m.origin};
        let at = |q: [f64;2]| coordinate(basis.lift(q[0],q[1]));
        let steps: Vec<(String,Seg)> = m.region.loops.iter().flatten()
            .map(|s| (m.name(s.tag).rsplit('.').next().unwrap().to_string(),s.seg)).collect();
        // which way the meridian is walked in (radius,height): outward is to the right of a
        // counterclockwise walk
        let turn = steps.iter().map(|(_,s)| { let (a,b) = (at(s.start()),at(s.end())); a[0]*b[1]-a[1]*b[0] })
            .sum::<f64>().signum();
        let (mut planes,mut corners) = (vec![],vec![]);
        for (name,seg) in &steps {
            match *seg {
                Seg::Line {a,b} => {
                    let (a,b) = (at(a),at(b));
                    if a[0].abs() < 1e-9*length && b[0].abs() < 1e-9*length { continue }
                    planes.push((name.clone(),a,[turn*(b[1]-a[1]),turn*(a[0]-b[0])]));
                }
                Seg::Arc {c,r,..} => {
                    let center = at(c);
                    let [a,b] = [seg.start(),seg.end()].map(|q| { let p = at(q); [p[0]-center[0],p[1]-center[1]] });
                    // a minor corner, walked counterclockwise in (radius,height)
                    let [a,b] = if a[0]*b[1]-a[1]*b[0] > 0. { [a,b] } else { [b,a] };
                    assert!(a[0]*b[1]-a[1]*b[0] > 0.,"this construction requires minor corners");
                    corners.push((name.clone(),center,r,a,b));
                }
                Seg::Conic {..} => panic!("a crown has no conic"),
            }
        }
        Pieces {origin,axis,planes,corners}
    }

    /// Keep only what `keep` says of each piece's face name.
    fn only(mut self,keep: impl Fn(&str) -> bool) -> Self {
        self.planes.retain(|p| keep(&p.0));
        self.corners.retain(|c| keep(&c.0));
        self
    }

    /// The profile: every half-plane within every rounded corner, and its coefficients.
    fn field(&self,name: &str) -> (PlanarField,[f64;3],[f64;3],String) {
        let mut fields = vec![];
        let mut planes = vec![];
        for (_,through,normal) in &self.planes {
            fields.push(PlanarField::half_plane(*through,*normal).unwrap());
            planes.push(format!("{{\"through\":{through:?},\"normal\":{normal:?}}}"));
        }
        let mut corners = vec![];
        for (_,center,radius,a,b) in &self.corners {
            let n0 = [-a[1],a[0]]; let n1 = [b[1],-b[0]];
            let cap = PlanarField::disk(*center,*radius).unwrap()
                .union(PlanarField::half_plane(*center,n0).unwrap()).unwrap()
                .union(PlanarField::half_plane(*center,n1).unwrap()).unwrap();
            fields.push(cap);
            corners.push(format!("{{\"center\":{center:?},\"radius\":{radius},\"sector_normals\":[{n0:?},{n1:?}]}}"));
        }
        let profile = fields.into_iter().reduce(|a,b| a.intersection(b).unwrap()).unwrap();
        let definition = format!("{{\"name\":\"{name}\",\"half_planes\":[{}],\"rounded_corners\":[{}]}}",planes.join(","),corners.join(","));
        (profile,self.origin,self.axis,definition)
    }
}

/// Explicit support construction of this rounded generating section: the crown's region, its
/// meridian read as half-planes and rounded corners. This is not a general conversion of
/// arbitrary Solvent regions. The emitted coefficients specify the functional set independently
/// of the numerical envelope workbench.
fn functional_profile(pair: &Pair,rack: &str) -> (PlanarField,[f64;3],[f64;3],String) {
    functional_profile_with_side(pair,rack,None)
}

fn functional_profile_with_side(pair: &Pair,rack: &str,active: Option<ProfileSide>)
    -> (PlanarField,[f64;3],[f64;3],String) {
    let active = active.map(|side| match side { ProfileSide::Inner => "inner", ProfileSide::Outer => "outer" });
    Pieces::read(pair,rack).only(|name| match active {
        Some(side) => !(name.starts_with("inner") || name.starts_with("outer")) || name.starts_with(side),
        None => true,
    }).field(rack)
}

/// A mate tooth of the gear's crown whole, as the drawn sections had it (the rejected full-crown
/// candidate's): the space cutter's side of it (`rack`, its cap left out), and its other flank
/// the tooth's opposite one carried a tooth's width along the pitch line — through the pitch point
/// reflected over the shared one, as the drawn mate's was — rounded into the mate's tip by the
/// same radius.  At no backlash, as the pair is read.
fn whole_mate(pair: &Pair,rack: &str) -> (PlanarField,[f64;3],[f64;3],String) {
    let side = Pieces::read(pair,rack).only(|name| name != "cap");
    let tooth = Pieces::read(pair,"pinion");
    let sk = &pair.model.sketch;
    let point = |name: &str| sk.world_point(pair.model.map.ent_named(name).unwrap().i());
    let unit = side.axis.map(|v| v/side.axis[0].hypot(side.axis[1]).hypot(side.axis[2]));
    let radius = |p: [f64;3]| {
        let q: [f64;3] = std::array::from_fn(|k| p[k]-side.origin[k]);
        let z = (0..3).map(|k| q[k]*unit[k]).sum::<f64>();
        (0..3).map(|k| (q[k]-z*unit[k]).powi(2)).sum::<f64>().sqrt()
    };
    let (ri,ro) = (radius(point("pair.mill.inner_pitch")),radius(point("pair.mill.outer_pitch")));
    // the outer mate's other flank is the tooth's inner one moved out by two pitch gaps; the
    // inner mate's, its outer one moved in
    let (from,shift) = if rack == "gear_outer" { ("inner",2.*(ro-ri)) } else { ("outer",2.*(ri-ro)) };
    let (_,through,tooth_normal) = tooth.planes.iter().find(|p| p.0 == from).unwrap().clone();
    let through = [through[0]+shift,through[1]];
    // outward from the mate: away from its own flank, which the tooth's opposite normal faces
    let normal = tooth_normal.map(|x| -x);
    let (_,tip_through,tip_normal) = side.planes.iter().find(|p| p.0 == "tip").unwrap().clone();
    let rho = side.corners[0].2;
    // the round's centre, `rho` inside both the flank and the tip; its ends, its feet on them
    let n = |v: [f64;2]| { let l = v[0].hypot(v[1]); [v[0]/l,v[1]/l] };
    let (nf,nt) = (n(normal),n(tip_normal));
    let (cf,ct) = (nf[0]*through[0]+nf[1]*through[1]-rho,nt[0]*tip_through[0]+nt[1]*tip_through[1]-rho);
    let det = nf[0]*nt[1]-nf[1]*nt[0];
    let center = [(cf*nt[1]-ct*nf[1])/det,(nf[0]*ct-nt[0]*cf)/det];
    let [a,b] = [nf,nt].map(|v| [rho*v[0],rho*v[1]]);
    let [a,b] = if a[0]*b[1]-a[1]*b[0] > 0. { [a,b] } else { [b,a] };
    let mut whole = side;
    whole.planes.push(("other".into(),through,normal));
    whole.corners.push(("other_round".into(),center,rho,a,b));
    whole.field(rack)
}

fn functional_generator(pair: &Pair,member: usize) -> (RevolvedField,String) {
    let mut profiles = if member == 0 { vec![functional_profile(pair,"pinion")] }
        else { vec![whole_mate(pair,"gear_outer"),whole_mate(pair,"gear_inner")] }.into_iter();
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
        let crown = pair.model.map.ent_named(Pair::crown(rack)).unwrap();
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
        let region = RevolvedRegion::read(&pair.model.sketch,pair.crown_of(&flank),pair.module*1e-10).unwrap();
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
            let region = RevolvedRegion::read(&pair.model.sketch,pair.crown_of(&flank),pair.module*1e-10).unwrap();
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
        // roll 0 is as drawn: a frame's turn has no phase
        let gcs_core::model::MotionDef::Rotation {axis,ratio,..} = pair.model.sketch.motions[id].def else { panic!() };
        let axis = &pair.model.sketch.lines[axis as usize];
        let origin = pair.model.sketch.world_point(axis.p1 as usize);
        let end = pair.model.sketch.world_point(axis.p2 as usize);
        let axis: [f64;3] = std::array::from_fn(|k| end[k]-origin[k]);
        format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"ratio\":{ratio},\"phase\":0}}")
    };
    format!("{{\"source\":{},\"observer\":{}}}",
        rotation("pair.crown_roll"),rotation(&format!("pair.{member_name}_roll")))
}
