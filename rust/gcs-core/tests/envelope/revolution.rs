//! The generating geometry comes from a solved Solvent component, including its fillets.
use super::*;
use gcs_core::{diagnose,library,model::EntRef,program,solid::RevolvedSurface,solve};

fn read() -> program::Elaborated {
    let src = include_str!("../../../examples/spiral_bevel/reference.sv");
    let (p,errors,link) = library::parse_linked(src);
    assert!(errors.is_empty() && link.is_empty(), "{errors:?} {link:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}",e.diags);
    // Disturb every free point; the dimensions and tangencies must restore the geometry.
    for i in 0..e.sketch.points.len() {
        for p in e.sketch.point_params(i) {
            if !e.sketch.params[p as usize].fixed {
                e.sketch.params[p as usize].value += 0.001*(i as f64).sin();
            }
        }
    }
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    assert_eq!(diagnose::diagnose(&mut e.sketch,Default::default()).dof,0);
    e
}

fn patch(e: &program::Elaborated, name: &str) -> RevolvedSurface {
    RevolvedSurface::named(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap()
}

#[test]
fn declarative_revolution_has_exact_conical_and_toroidal_patches() {
    let e = read();
    let cone = patch(&e,"outer");
    let torus = patch(&e,"outer_round");
    let a = 20f64.to_radians();
    let join_height = 2.-0.6*(1.-a.sin());
    let reference_radius = 0.8*24f64.hypot(48.);
    let major = reference_radius+1.3-join_height*a.tan()-0.6*a.cos();
    for u in [0.1,0.3,0.5,0.7,0.9] {
        for v in [0.13,0.37,0.61,0.89] {
            let p = cone.at(u,v).unwrap().position;
            assert!((p[0].hypot(p[1])+p[2]*a.tan()-reference_radius-1.3).abs() < 1e-7);
            let p = torus.at(u,v).unwrap().position;
            assert!(((p[0].hypot(p[1])-major).powi(2)+(p[2]-1.4).powi(2)-0.36).abs()
                < 1e-7);
            for s in [&cone,&torus] {
                let f = s.at(u,v).unwrap();
                for (j,want) in [f.du,f.dv].into_iter().enumerate() {
                    let mut lo = [u,v]; let mut hi = [u,v];
                    lo[j] -= 1e-6; hi[j] += 1e-6;
                    let lo = s.at(lo[0],lo[1]).unwrap().position;
                    let hi = s.at(hi[0],hi[1]).unwrap().position;
                    near(want,std::array::from_fn(|i| (hi[i]-lo[i])/2e-6),1e-6);
                }
            }
        }
    }
    // At the tangent join the positions and oriented normals agree for every turn.
    for v in [0.,0.2,0.7,1.] {
        let a = envelope::contact(cone.at(1.,v).unwrap(),Motion::identity()).unwrap();
        let b = envelope::contact(torus.at(0.,v).unwrap(),Motion::identity()).unwrap();
        near(a.position,b.position,1e-7);
        near(a.normal,b.normal,1e-7);
    }
    assert_eq!(torus.at(-0.1,0.).unwrap_err(),Error::OutsideDomain);
    assert!(RevolvedSurface::read(&e.sketch,e.map.ent_named("crown").unwrap().i(),
        e.map.ent_named("axis").unwrap()).is_err());
}

#[test]
fn model_radius_changes_are_read_by_new_exact_surface_snapshots() {
    let mut e = read();
    let before = patch(&e,"outer_round").at(0.5,0.2).unwrap().position;
    let arc: EntRef = e.map.ent_named("rack.outer_round").unwrap();
    let radius = e.sketch.round_radius(arc);
    e.sketch.params[radius].value *= 1.2;
    let after = patch(&e,"outer_round").at(0.5,0.2).unwrap().position;
    assert!((0..3).any(|i| (after[i]-before[i]).abs() > 1e-3));
}

#[test]
fn exact_revolution_respects_partial_turn_sense_and_world_plane_not_page_placement() {
    use gcs_core::{model::{Sense,SolidDef},plane::Basis};
    let mut e = read();
    let si = e.map.ent_named("crown").unwrap().i();
    let pi = e.map.ent_named("std.front").unwrap().i();
    let SolidDef::Revolve { face,sweep,sense,.. } = &mut e.sketch.solids[si].def else {
        panic!("expected a revolution");
    };
    sweep.value = std::f64::consts::FRAC_PI_2;
    *sense = Sense::Cw;
    e.sketch.faces[*face as usize].support = gcs_core::model::FaceSupport::Plane(Some(pi as u32));
    e.sketch.planes[pi].basis = Basis { u: [0.,0.,1.],v: [1.,0.,0.],o: [10.,20.,30.] };
    let before = patch(&e,"outer_round");
    let a = before.at(0.4,0.).unwrap().position;
    near(before.at(0.4,1.).unwrap().position,[a[0],20.+a[2]-30.,30.],1e-7);

    // Move and rotate the entire drawing on the page while leaving the solid's world
    // plane untouched. Page pose must cancel before the profile is revolved in space.
    let (s,c) = 0.37f64.sin_cos();
    for i in 0..e.sketch.points.len() {
        let (x,y) = e.sketch.point_xy(i);
        let [px,py] = e.sketch.point_params(i);
        e.sketch.params[px as usize].value = c*x-s*y+7.;
        e.sketch.params[py as usize].value = s*x+c*y-4.;
    }
    for p in &e.sketch.planes {
        let pc = e.sketch.params[p.frame.c as usize].value;
        let ps = e.sketch.params[p.frame.s as usize].value;
        e.sketch.params[p.frame.c as usize].value = c*pc-s*ps;
        e.sketch.params[p.frame.s as usize].value = s*pc+c*ps;
    }
    let after = patch(&e,"outer_round");
    for (u,v) in [(0.,0.),(0.2,0.7),(0.9,0.4),(1.,1.)] {
        let a = before.at(u,v).unwrap(); let b = after.at(u,v).unwrap();
        near(a.position,b.position,1e-7);
        near(a.du,b.du,1e-7);
        near(a.dv,b.dv,1e-7);
    }
}

#[test]
fn root_envelope_uses_the_declared_round_and_meets_flank_and_root_cone_tangentially() {
    let e = read();
    let flank = patch(&e,"outer");
    let round = patch(&e,"outer_round");
    let tip = patch(&e,"tip");
    let delta = 24f64.atan2(48.);
    let rm = 24f64.hypot(48.);
    let theta = (35f64-90.).to_radians();
    let reference_radius = 0.8*rm+1.3;
    let offset = [rm-reference_radius*theta.cos(),-reference_radius*theta.sin(),0.];
    let motion = |t| Motion::translation(offset,[0.;3]).unwrap()
        .then(rotate(2,t,1.))
        .then(rotate(1,delta-std::f64::consts::FRAC_PI_2,0.))
        .then(rotate(2,-t/delta.sin(),-1./delta.sin()));
    let sample = |surface: &RevolvedSurface, u: f64, rho: f64,
                  seed: [f64;3]| -> envelope::Intersection {
        envelope::intersect_parameters(|u,v| surface.at(u,v).unwrap(),motion,
            |p,c| [p[0]-u,c.position.iter().map(|x| x*x).sum::<f64>().sqrt()-rho],
            seed,IntersectionOptions {
                bounds: [[0.,1.],[0.5,1.],[-0.6,0.6]],parameter_scale: [1.,1.,1.],
                residual_tolerance: [1e-9,1e-11,1e-9],max_iterations: 100,
            }).unwrap()
    };
    for face in [0.9,1.,1.1] {
        let rho = rm*face;
        let a = sample(&flank,1.,rho,[1.,theta/std::f64::consts::TAU+1.,0.]);
        let b = sample(&round,0.,rho,[0.,a.parameters[1],a.parameters[2]]);
        near(a.contact.position,b.contact.position,1e-7);
        near(a.contact.normal,b.contact.normal,1e-7);
        let mut seed = b.parameters;
        let mut previous = b.contact.position;
        let mut min_segment = f64::INFINITY;
        for i in 1..=40 {
            let u = i as f64/40.;
            seed[0] = u;
            let p = sample(&round,u,rho,seed);
            seed = p.parameters;
            let segment = (0..3).map(|k| (p.contact.position[k]-previous[k]).powi(2))
                .sum::<f64>().sqrt();
            min_segment = min_segment.min(segment);
            previous = p.contact.position;
            assert!(p.contact.normal_velocity.abs() < 1e-9);
        }
        // The generated circular transition reaches the root cone without a cusp.
        assert!(min_segment > 0.001, "min segment {min_segment}");
        let end = sample(&round,1.,rho,seed);
        let root = sample(&tip,0.,rho,[0.,seed[1],seed[2]]);
        near(end.contact.position,root.contact.position,1e-7);
        near(end.contact.normal,root.contact.normal,1e-7);
        let p = end.contact.position;
        let depth = p[0].hypot(p[1])*delta.cos()-p[2]*delta.sin();
        assert!((depth+2.).abs() < 1e-7, "root depth {depth}");
    }
}
