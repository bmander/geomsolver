//! Independent envelope checks: elementary geometry, differential motion, and a published
//! spiral bevel gear fixture. The gear settings live in tests, not in the generic engine.
use gcs_core::envelope::{self, Error, IntersectionOptions, Motion, SurfacePoint};

mod nasa;
mod crown;
mod revolution;
mod paired;

fn near(a: [f64;3], b: [f64;3], tol: f64) {
    for k in 0..3 { assert!((a[k]-b[k]).abs() < tol, "{a:?} != {b:?} ({tol})"); }
}

fn rotate(axis: usize, angle: f64, rate: f64) -> Motion {
    let mut a = [0.;3]; a[axis] = 1.;
    Motion::rotation(a, angle, rate).unwrap()
}

#[test]
fn compound_motion_velocity_includes_both_frames_and_translation() {
    let motion = |t: f64| {
        Motion::translation([t*t, 2.*t, 3.], [2.*t,2.,0.]).unwrap()
            .then(rotate(0, t*0.7, 0.7))
            .then(rotate(2, -t*1.9, -1.9))
            .then(Motion::translation([0.,t.sin(),0.], [0.,t.cos(),0.]).unwrap())
    };
    for t in [-1.2, 0., 0.4, 2.7] {
        let p = [2.,-1.,4.];
        let a = motion(t-1e-6).point(p);
        let b = motion(t+1e-6).point(p);
        near(motion(t).velocity(p), std::array::from_fn(|i| (b[i]-a[i])/2e-6), 1e-8);
        let v = motion(t).vector(p);
        assert!((v.iter().map(|x| x*x).sum::<f64>()-21.).abs() < 1e-12);
        for identity in [motion(t).then(motion(t).inverse()),
            motion(t).inverse().then(motion(t))] {
            near(identity.point(p), p, 1e-12);
            near(identity.velocity(p), [0.;3], 1e-12);
        }
    }
}

// A sphere translated along z envelopes a cylinder of the sphere's radius.
fn sphere(u: f64, v: f64, r: f64) -> SurfacePoint {
    SurfacePoint {
        position: [r*v.cos()*u.cos(), r*v.cos()*u.sin(), r*v.sin()],
        du: [-r*v.cos()*u.sin(), r*v.cos()*u.cos(), 0.],
        dv: [-r*v.sin()*u.cos(), -r*v.sin()*u.sin(), r*v.cos()],
    }
}

fn opts(scale: f64, time_scale: f64) -> IntersectionOptions {
    IntersectionOptions {
        bounds: [[-1.,1.],[-1.,1.],[-10.*time_scale,10.*time_scale]],
        parameter_scale: [1.,1.,time_scale],
        residual_tolerance: [1e-10*scale/time_scale,1e-10*scale,1e-10*scale],
        max_iterations: 80,
    }
}

#[test]
fn translating_sphere_envelopes_cylinder_independent_of_size_and_motion_units() {
    for s in [1e-6,1.,25.4,1e6] {
        for ts in [0.001,1.,1000.] {
            let result = envelope::intersect(|u,v| sphere(u,v,2.*s),
                |t| Motion::translation([0.,0.,t*s/ts], [0.,0.,s/ts]).unwrap(),
                |c| [c.position[1]-s, c.position[2]-3.*s],
                [0.3,0.2,2.8*ts], opts(s,ts)).unwrap();
            near(result.contact.position, [3f64.sqrt()*s,s,3.*s], 1e-9*s);
            near(result.parameters, [std::f64::consts::PI/6.,0.,3.*ts], 1e-9*ts.max(1.));
        }
    }
}

#[test]
fn intersections_refuse_singular_zero_residual_and_invalid_inputs() {
    let section = |c: &envelope::Contact| [c.position[1], c.position[2]];
    let solve = |seed, options| envelope::intersect(|u,v| sphere(u,v,2.),
        |_| Motion::identity(), section, seed, options);
    assert_eq!(solve([0.;3], opts(1.,1.)).unwrap_err(), Error::SingularIntersection);
    assert_eq!(solve([2.,0.,0.], opts(1.,1.)).unwrap_err(), Error::OutsideDomain);
    assert_eq!(solve([f64::NAN,0.,0.], opts(1.,1.)).unwrap_err(), Error::NonFinite);
    let mut options = opts(1.,1.);
    options.residual_tolerance[0] = 0.;
    assert_eq!(solve([0.;3], options).unwrap_err(), Error::InvalidOptions);
    assert_eq!(envelope::contact(SurfacePoint { position: [0.;3], du: [1.,0.,0.],
        dv: [2.,0.,0.] }, Motion::identity()).unwrap_err(), Error::Degenerate);
}

#[test]
fn impossible_section_and_exhausted_iteration_budget_are_errors() {
    let motion = |t| Motion::translation([0.,0.,t], [0.,0.,1.]).unwrap();
    let surface = |u,v| sphere(u,v,2.);
    assert!(envelope::intersect(surface, motion, |c| [c.position[1]-4.,c.position[2]],
        [0.3,0.2,0.], opts(1.,1.)).is_err());
    let mut options = opts(1.,1.); options.max_iterations = 1;
    assert_eq!(envelope::intersect(surface, motion, |c| [c.position[1]-1.,c.position[2]],
        [0.3,0.2,0.], options).unwrap_err(), Error::NotConverged);
}

#[test]
fn narrow_search_bounds_do_not_make_a_regular_intersection_singular() {
    let mut options = opts(1.,1.);
    options.bounds = [[0.,1e-7],[-1e-7,0.],[0.,1e-7]];
    // Exact root at the corner: every derivative must be evaluated from inside its domain.
    let p = envelope::intersect(|u,v| sphere(u,v,2.),
        |t| Motion::translation([0.,0.,t], [0.,0.,1.]).unwrap(),
        |c| [c.position[1],c.position[2]], [0.;3], options).unwrap();
    near(p.contact.position,[2.,0.,0.],1e-12);
}

#[test]
fn fixed_parameters_are_exact_and_remaining_equations_still_have_to_hold() {
    for scale in [0.001,1.,1000.] {
        let mut options = opts(scale,1.);
        options.bounds[0] = [0.,0.];
        let surface = |u,v| sphere(u,v,2.*scale);
        let motion = |t| Motion::translation([0.,0.,t*scale],[0.,0.,scale]).unwrap();
        let section = |p: [f64;3],c: &envelope::Contact| [p[0],c.position[2]-3.*scale];
        let c = envelope::intersect_parameters(surface,motion,section,[0.,0.2,2.8],options).unwrap();
        assert_eq!(c.parameters[0],0.);
        near(c.contact.position,[2.*scale,0.,3.*scale],1e-8*scale);
        assert_eq!(envelope::intersect_parameters(surface,motion,section,[1e-12,0.2,2.8],options)
            .unwrap_err(),Error::OutsideDomain);
        // Redundant equations are checked, not silently discarded with the held parameter.
        assert_eq!(envelope::intersect_parameters(surface,motion,
            |p,c| [p[0]+scale,c.position[2]-3.*scale],[0.,0.2,2.8],options)
            .unwrap_err(),Error::NotConverged);
        options.bounds = [[0.,0.],[0.,0.],[3.,3.]];
        let c = envelope::intersect_parameters(surface,motion,section,[0.,0.,3.],options).unwrap();
        assert_eq!(c.iterations,0);
        options.bounds[2] = [2.,2.];
        assert_eq!(envelope::intersect_parameters(surface,motion,section,[0.,0.,2.],options)
            .unwrap_err(),Error::NotConverged);
    }
}
