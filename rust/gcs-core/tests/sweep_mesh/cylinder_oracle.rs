//! Independent geometry of (x-3)^2+y^2 <= 1, |z| <= 1, turned about x.
//! No model, field, contact tracer or construction queries. Binary64 diagnostics,
//! with an explicit boundary guard; the residual is NOT a distance certificate.
use std::f64::consts::{PI,TAU,FRAC_PI_2,FRAC_PI_6};
use super::harness::V3;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Side { Material, Exterior, Boundary }
#[derive(Clone,Copy)]
pub struct Cylinder { pub half_roll: f64 }
impl Cylinder {
    pub fn fixture() -> Self { Self {half_roll:FRAC_PI_6} }
    pub fn radius(self,x: f64,phi: f64) -> f64 {
        assert!(x.is_finite() && phi.is_finite() && self.half_roll.is_finite() && self.half_roll >= 0.);
        if !(2. ..=4.).contains(&x) { return 0.; }
        let a = ((x-2.)*(4.-x)).max(0.).sqrt();
        let corner = a.hypot(1.);
        if self.half_roll >= FRAC_PI_2 { return corner; } // rectangle repeats every pi
        let angle = phi.rem_euclid(PI);
        if a == 0. {
            // Union of rotated line segments at x=2,4: two filled sectors.
            // Their interiors are planar end faces of the 3D swept solid.
            // Guard angular reduction at the sector edges, where radius jumps.
            return if (angle-FRAC_PI_2).abs() <= self.half_roll+1e-13 { 1. } else { 0. };
        }
        let beta = 1_f64.atan2(a);
        // A corner can attain the circumradius iff its angle is in the roll window.
        for c in [beta,PI-beta] {
            let d = (angle-c+FRAC_PI_2).rem_euclid(PI)-FRAC_PI_2;
            if d.abs() <= self.half_roll { return corner; }
        }
        let ray = |q: f64| {
            let (s,c) = q.sin_cos();
            let y = if c == 0. { f64::INFINITY } else { a/c.abs() };
            let z = if s == 0. { f64::INFINITY } else { 1./s.abs() };
            y.min(z)
        };
        ray(angle-self.half_roll).max(ray(angle+self.half_roll))
    }
    pub fn point(self,x: f64,phi: f64) -> V3 {
        let r = self.radius(x,phi); [x,r*phi.cos(),r*phi.sin()]
    }
    pub fn side(self,p: V3) -> Side {
        assert!(p.iter().all(|v| v.is_finite()));
        let guard = 2e-12; // roundoff guard only; never a spatial error bound
        if p[0] < 2.-guard || p[0] > 4.+guard { return Side::Exterior; }
        let x = p[0].clamp(2.,4.);
        let r = p[1].hypot(p[2]);
        let gap = self.radius(x,p[2].atan2(p[1]))-r;
        if gap < -guard { Side::Exterior }
        else if p[0] <= 2.+guard || p[0] >= 4.-guard || gap <= guard { Side::Boundary }
        else { Side::Material }
    }
    /// Midpoint quadrature, x=3+sin(u) removes the square-root endpoint derivative.
    /// Successive refinements give a convergence estimate, not a rigorous enclosure.
    pub fn volume(self,n: usize) -> f64 {
        assert!(n > 0);
        let mut sum = 0.;
        let du = PI/n as f64; let dp = FRAC_PI_2/(4*n) as f64;
        for i in 0..n {
            let u = -FRAC_PI_2+(i as f64+0.5)*du;
            let x = 3.+u.sin();
            for j in 0..4*n {
                let r = self.radius(x,(j as f64+0.5)*dp);
                sum += 2.*r*r*u.cos()*du*dp; // four quadrants and polar area 1/2
            }
        }
        sum
    }
}

#[test]
fn continuous_rectangle_oracle_limits_and_degeneracies() {
    let zero = Cylinder {half_roll:0.};
    for x in [2.01_f64,2.3,3.,3.8,3.99] { for i in 0..137 {
        let phi = TAU*(i as f64+0.31)/137.;
        let a = ((x-2.)*(4.-x)).sqrt();
        let expected = (a/phi.cos().abs()).min(1./phi.sin().abs());
        assert!((zero.radius(x,phi)-expected).abs() < 2e-14);
    } }
    let full = Cylinder {half_roll:PI};
    for x in [2.,2.1,3.,3.8,4.] { for phi in [0.,1.,PI,TAU] {
        assert!((full.radius(x,phi)-(2.-(x-3.).powi(2)).sqrt()).abs() < 1e-14);
    } }
    assert!((full.volume(512)-10.*PI/3.).abs() < 2e-5);
    assert!((zero.volume(512)-2.*PI).abs() < 1e-4);
    let c = Cylinder::fixture();
    for x in [2.,2.1,2.9,3.,4.] { for i in 0..129 {
        let phi = TAU*i as f64/129.; let r = c.radius(x,phi);
        assert!((r-c.radius(6.-x,-phi)).abs() < 2e-14);
        assert!((r-c.radius(x,phi+7.*TAU)).abs() < 2e-13);
    } }
    for x in [2.,4.] {
        assert_eq!(c.side([x,0.,0.]),Side::Boundary);
        assert_eq!(c.side([x,0.,0.8]),Side::Boundary);
        assert_eq!(c.side([x,0.8,0.]),Side::Exterior);
        assert_eq!(c.radius(x,FRAC_PI_2),1.);
        assert_eq!(c.radius(x,0.),0.);
    }
    assert_eq!(c.side([3.,0.,0.]),Side::Material);
    assert_eq!(c.side([1.999,0.,0.]),Side::Exterior);
    assert_eq!(c.side([4.001,0.,0.]),Side::Exterior);
    assert_eq!(zero.radius(2.,FRAC_PI_2),1.);
    assert_eq!(zero.radius(2.,0.),0.);
}

#[test]
fn corner_candidates_cover_intermediate_poses_and_volume_converges() {
    let c = Cylinder::fixture();
    // Separate direct inverse rotation membership checks the finite maximizer.
    // The dense poses are only a lower-bound check, never the oracle definition.
    for x in [2.03,2.3,3.,3.9] { for i in 0..61 {
        let phi = TAU*(i as f64+0.17)/61.; let r = c.radius(x,phi);
        let mut sampled = 0_f64;
        for j in 0..=2000 {
            let q = phi+c.half_roll-2.*c.half_roll*j as f64/2000.;
            let a = ((x-2.)*(4.-x)).sqrt();
            sampled = sampled.max((a/q.cos().abs()).min(1./q.sin().abs()));
        }
        assert!(sampled <= r+2e-14 && r-sampled < 0.002);
        assert_eq!(c.side([x,0.999*r*phi.cos(),0.999*r*phi.sin()]),Side::Material);
        assert_eq!(c.side([x,1.001*r*phi.cos(),1.001*r*phi.sin()]),Side::Exterior);
    } }
    let v: Vec<_> = [64,128,256,512].map(|n| c.volume(n)).into();
    eprintln!("independent cylinder volume refinements: {v:?}");
    assert!((v[3]-v[2]).abs() < 5e-5);
    assert!((v[3]-v[2]).abs() < (v[1]-v[0]).abs());
}
