//! Parametric, common-crown conjugacy experiment. This is a mathematical fixture for
//! the future Solvent component, not a new gear-specific engine primitive.
//!
//! Crown generation: Chang, Huston, Coy, NASA TM-101449 (1989), pp. 8–11.
//! https://ntrs.nasa.gov/api/citations/19890007877/downloads/19890007877.pdf
//! The shared-crown construction and velocity proof are derived explicitly in
//! docs/spiral-bevel-design.md. There is no root/tooth-tip solid closure in this fixture.
use super::*;
use std::f64::consts::{FRAC_PI_2, TAU};

fn dot(a: [f64;3], b: [f64;3]) -> f64 { (0..3).map(|i| a[i]*b[i]).sum() }
fn cross(a: [f64;3], b: [f64;3]) -> [f64;3] {
    [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
}

struct Pair {
    teeth: [f64;2],
    module: f64,
    spiral: f64,
    pressure: f64,
}

impl Pair {
    fn pitch(&self) -> [f64;2] {
        let d = self.teeth[0].atan2(self.teeth[1]);
        [d,FRAC_PI_2-d]
    }
    fn mean_cone_distance(&self) -> f64 {
        self.module*self.teeth[0].hypot(self.teeth[1])/2.
    }
    fn pose(&self, member: usize, t: f64) -> Motion {
        let d = self.pitch()[member];
        if member == 0 {
            rotate(2,t/d.sin(),1./d.sin()).then(rotate(1,FRAC_PI_2-d,0.))
        } else {
            rotate(2,-t/d.sin(),-1./d.sin()).then(rotate(1,FRAC_PI_2+d,0.))
        }
    }
    fn relative(&self, member: usize, t: f64) -> Motion {
        rotate(2,t,1.).then(self.pose(member,t).inverse())
    }
    fn crown(&self, height: f64, angle: f64, flank: f64) -> SurfacePoint {
        let rm = self.mean_cone_distance();
        let rc = rm*0.8;
        let theta = self.spiral-FRAC_PI_2;
        let c = [rm-rc*theta.cos(),-rc*theta.sin()];
        let slope = flank*self.pressure.tan();
        let r = rc+height*slope;
        // At mean pitch, the circular tooth trace is at the specified spiral angle.
        // Angularly offset the two sides by half a crown tooth's angular thickness.
        let phase = flank*TAU/(4.*self.teeth[0].hypot(self.teeth[1]));
        let p = SurfacePoint {
            position: [c[0]+r*angle.cos(),c[1]+r*angle.sin(),height],
            du: [slope*angle.cos(),slope*angle.sin(),1.],
            dv: [-r*angle.sin(),r*angle.cos(),0.],
        };
        let rotation = rotate(2,phase,0.);
        SurfacePoint { position: rotation.point(p.position),
            du: rotation.vector(p.du), dv: rotation.vector(p.dv) }
    }

    // Intersect the crown cone with a sphere analytically, then solve the characteristic
    // equation A sin(t) + B cos(t) = 0. This gives an independent branch reference for
    // tracing through a cusp, where the original radius/axial section becomes singular.
    fn characteristic(&self, rho: f64, height: f64, flank: f64) -> [f64;3] {
        let rm = self.mean_cone_distance();
        let rc = rm*0.8;
        let theta0 = self.spiral-FRAC_PI_2;
        let c = [rm-rc*theta0.cos(),-rc*theta0.sin()];
        let cn = c[0].hypot(c[1]);
        let radius = rc+height*flank*self.pressure.tan();
        let cos_angle = (rho*rho-height*height-cn*cn-radius*radius)/(2.*cn*radius);
        assert!(cos_angle.abs() < 1.);
        let theta = c[1].atan2(c[0])-cos_angle.acos();
        let s = self.crown(height,theta,flank);
        let n = cross(s.du,s.dv);
        let a = n[2]*s.position[0]-height*n[0];
        let b = n[2]*s.position[1]-height*n[1];
        let mut t = (-b).atan2(a);
        if t > FRAC_PI_2 { t -= std::f64::consts::PI; }
        if t < -FRAC_PI_2 { t += std::f64::consts::PI; }
        [height,theta,t]
    }
}

#[test]
fn common_crown_flanks_have_conjugate_contact_at_configurable_ratios_and_sizes() {
    for teeth in [[12.,36.],[16.,32.],[20.,20.],[17.,43.]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair { teeth,module,spiral: 35f64.to_radians(),
                pressure: 20f64.to_radians() };
            let rm = pair.mean_cone_distance();
            let pitch = pair.pitch();
            // Equality of the crown tooth pitches is necessary for indexing both wheels.
            assert!((teeth[0]/pitch[0].sin()-teeth[1]/pitch[1].sin()).abs() < 1e-12);
            for flank in [-1.,1.] {
                let surface = |u,v| pair.crown(u,v,flank);
                for face in [0.85,0.925,1.,1.075,1.15] {
                    let mut seed = [0.,pair.spiral-FRAC_PI_2,0.];
                    // A local band, not the eventual root-to-tip domain. A separate test
                    // below preserves a deeper section this experimental flank cannot solve.
                    for depth in [0.,-0.15*module,0.,0.15*module] {
                        let p = envelope::intersect(surface, |t| pair.relative(0,t),
                            |c| [c.position[2]-rm*face*pitch[0].cos()+depth*pitch[0].sin(),
                                c.position[0].hypot(c.position[1])-rm*face*pitch[0].sin()
                                    -depth*pitch[0].cos()],
                            seed,IntersectionOptions {
                                bounds: [[-module,module],[seed[1]-0.8,seed[1]+0.8],[-0.8,0.8]],
                                parameter_scale: [module,1.,1.],
                                residual_tolerance: [1e-10*module;3], max_iterations: 80,
                            }).unwrap_or_else(|e| panic!(
                            "{e:?}: teeth={teeth:?}, module={module}, flank={flank}, \
                             face={face}, depth={depth}, seed={seed:?}"));
                        seed = p.parameters;
                        let [u,v,t] = p.parameters;
                        let c2 = envelope::contact(surface(u,v),pair.relative(1,t)).unwrap();
                        assert!(c2.normal_velocity.abs() < 1e-9*module);
                        let b1 = pair.pose(0,t);
                        let b2 = pair.pose(1,t);
                        let world = b1.point(p.contact.position);
                        near(world,b2.point(c2.position),1e-10*module);
                        near(b1.vector(p.contact.normal),b2.vector(c2.normal),1e-10);
                        // Independent check from the actual two shaft angular velocities,
                        // without reading either generated surface's stored velocity.
                        let omega = [1./pitch[0].tan()+1./pitch[1].tan(),0.,0.];
                        assert!(dot(b1.vector(p.contact.normal),cross(omega,world)).abs()
                            < 1e-9*module);
                    }
                }
            }
        }
    }
}

#[test]
fn unsolved_deeper_crown_section_is_not_returned_as_a_tooth_surface() {
    let pair = Pair { teeth: [12.,36.],module: 0.2,spiral: 35f64.to_radians(),
        pressure: 20f64.to_radians() };
    let d = pair.pitch()[0];
    let rm = pair.mean_cone_distance();
    let depth = -0.45*pair.module;
    let result = envelope::intersect(|u,v| pair.crown(u,v,-1.), |t| pair.relative(0,t),
        |c| [c.position[2]-rm*0.85*d.cos()+depth*d.sin(),
            c.position[0].hypot(c.position[1])-rm*0.85*d.sin()-depth*d.cos()],
        // A converged preceding section at depth -0.3 module.
        [0.0800177809069566,-1.1751580385203968,0.0684906160128146],
        IntersectionOptions { bounds: [[-0.2,0.2],[-2.,0.],[-0.8,0.8]],
            parameter_scale: [0.2,1.,1.],residual_tolerance: [2e-11;3],max_iterations: 80 });
    assert_eq!(result.unwrap_err(),Error::NotConverged);
}

#[test]
fn crown_root_limit_is_a_cusp_of_the_envelope_not_a_search_bound() {
    let pair = Pair { teeth: [12.,36.],module: 0.2,spiral: 35f64.to_radians(),
        pressure: 20f64.to_radians() };
    let rho = (0.85*pair.mean_cone_distance()).hypot(0.45*pair.module);
    let position = |h| {
        let [u,v,t] = pair.characteristic(rho,h,-1.);
        let c = envelope::contact(pair.crown(u,v,-1.),pair.relative(0,t)).unwrap();
        assert!(c.normal_velocity.abs() < 1e-12);
        c.position
    };
    let polar = |h| { let p = position(h); p[0].hypot(p[1]).atan2(p[2]) };
    // Ternary refinement of the minimum in the independently sampled local branch.
    let (mut lo,mut hi) = (0.,pair.module);
    for _ in 0..70 {
        let a = (2.*lo+hi)/3.; let b = (lo+2.*hi)/3.;
        if polar(a) < polar(b) { hi = b; } else { lo = a; }
    }
    let h = (lo+hi)/2.;
    let target = pair.pitch()[0]-(0.45*pair.module/(0.85*pair.mean_cone_distance())).atan();
    assert!(polar(h) > target);
    assert!(polar(h) < polar(0.) && polar(h) < polar(pair.module));
    let a = position(h-1e-5*pair.module);
    let b = position(h+1e-5*pair.module);
    let speed = (0..3).map(|i| ((b[i]-a[i])/(2e-5*pair.module)).powi(2)).sum::<f64>().sqrt();
    assert!(speed < 1e-5, "speed={speed}");
    eprintln!("crown cusp h={h}, polar={}deg, requested={}deg, speed={speed}",
        polar(h).to_degrees(),target.to_degrees());

    // Fixing the generating height and sphere, instead of axial/radial coordinates,
    // lets the generic intersection solver trace both sides of this cusp.
    for height in [0.,0.08,0.12,h,0.16,0.2] {
        let expected = pair.characteristic(rho,height,-1.);
        let p = envelope::intersect_parameters(|u,v| pair.crown(u,v,-1.),
            |t| pair.relative(0,t),
            |q,c| [q[0]-height,dot(c.position,c.position).sqrt()-rho],
            [height,expected[1]+0.01,expected[2]+0.01],IntersectionOptions {
                bounds: [[-0.2,0.21],[-2.,0.],[-0.8,0.8]],
                parameter_scale: [0.2,1.,1.],residual_tolerance: [2e-11;3],
                max_iterations: 80,
            }).unwrap();
        near(p.parameters,expected,1e-9);
    }
}
