use super::*;
use gcs_core::seam::{SurfaceSeam,SurfaceSeamOptions};

fn source() -> String { format!("{MODEL}{}",boundary::SPHERE) }
/// The cylinder `first_surface` meeting `second`.
fn read(e: &program::Elaborated,second: &str) -> SurfaceSeam {
    SurfaceSeam::new("section_curve".into(),surface(e,"first_surface"),projector(e,second))
}
fn options(s: &SurfaceSeam) -> SurfaceSeamOptions {
    SurfaceSeamOptions {bounds:s.domain(),parameter_scale:[1.;2],
        residual_tolerance:[1e-10;2],min_transversality:1e-6,max_iterations:100}
}
fn near(a: f64,b: f64) { assert!((a-b).abs() < 1e-8,"{a} != {b}"); }
fn dot(a: [f64;3],b: [f64;3]) -> f64 { (0..3).map(|i| a[i]*b[i]).sum() }

#[test]
fn a_surface_seam_solves_a_circle_without_generating_motion() {
    let e = solved(&source());
    let s = read(&e,"cut.wall");
    for v in [0.,0.05,0.125,0.2,0.25] {
        let result = s.intersect(|p,_| p[1]-v,[0.4,0.1],options(&s)).unwrap();
        let p = result.point;
        // Independent cylinder/sphere section: r=3, R=sqrt(9.25), hence z=0.5.
        near(p.position[0].hypot(p.position[1]),3.);
        near(p.position[2],0.5);
        near(p.parameters[0][0],0.5);
        near(p.parameters[0][1],v);
        near(p.transversality,0.5/9.25f64.sqrt());
        near(dot(p.tangent,p.tangent),1.);
        for n in p.normals { near(dot(n,n),1.); near(dot(n,p.tangent),0.); }
        let second = surface(&e,"cut.wall").at(p.parameters[1][0],p.parameters[1][1]).unwrap();
        for i in 0..3 { near(p.position[i],second.position[i]); }
        assert!(p.incidence_error <= 1e-10);
    }
}

#[test]
fn surface_seams_preserve_finite_membership_and_snapshot_domains() {
    let mut e = solved(&source());
    let original = read(&e,"cut.wall");
    let wall = index(&e,"cut.wall");
    e.sketch.surfaces[wall].span = Some([180f64.to_radians(),270f64.to_radians()]);
    let limited = read(&e,"cut.wall");
    assert_ne!(original.domains(),limited.domains());
    assert!(original.evaluate([0.5,0.1],1e-10,1e-6).is_ok());
    assert_eq!(limited.evaluate([0.5,0.1],1e-10,1e-6).unwrap_err(),Error::OutsideDomain);
    assert_eq!(limited.intersect(|p,_| p[1]-0.1,[0.4,0.1],options(&limited))
        .unwrap_err(),Error::OutsideDomain);
    for p in [[0.6,0.1],[0.5,0.3]] {
        assert_eq!(original.evaluate(p,1e-10,1e-6).unwrap_err(),Error::OutsideDomain);
    }
}

#[test]
fn surface_seams_refuse_tangency_and_nonisolated_sections_at_exact_seeds() {
    let e = solved(&source());
    let s = read(&e,"cut.wall");
    assert_eq!(s.intersect(|_,_| 0.,[0.5,0.1],options(&s)).unwrap_err(),Error::SingularIntersection);
    // Fixing v structurally removes the free direction, so the same incidence is isolated.
    let mut opts = options(&s);
    opts.bounds[1] = [0.1,0.1];
    assert!(s.intersect(|_,_| 0.,[0.4,0.1],opts).is_ok());
    opts.bounds[0] = [0.5,0.5];
    assert_eq!(s.intersect(|_,_| 0.,[0.5,0.1],opts).unwrap().iterations,0);
    opts.bounds[0] = [0.4,0.4];
    assert_eq!(s.intersect(|_,_| 0.,[0.4,0.1],opts).unwrap_err(),Error::NotConverged);
    let e = solved(&source().replace("sqrt(9.25)","3"));
    assert_eq!(read(&e,"cut.wall").evaluate([0.,0.1],1e-10,1e-6).unwrap_err(),Error::SingularIntersection);
    let e = solved(&format!("{}duplicate := surface(body,low,from: 0deg,to: 90deg)\n",source()));
    assert_eq!(read(&e,"duplicate").evaluate([0.5,0.1],1e-10,1e-6).unwrap_err(),Error::SingularIntersection);
}

#[test]
fn surface_seam_controls_fail_before_callbacks() {
    let e = solved(&source());
    let s = read(&e,"cut.wall");
    let mut cases = vec![];
    for bad in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(s.evaluate([0.5,0.1],bad,1e-6).unwrap_err(),Error::InvalidOptions);
        let mut o = options(&s); o.min_transversality = bad; cases.push(o);
        let mut o = options(&s); o.parameter_scale[0] = bad; cases.push(o);
        let mut o = options(&s); o.residual_tolerance[1] = bad; cases.push(o);
    }
    let mut o = options(&s); o.min_transversality = 1.; cases.push(o);
    let mut o = options(&s); o.max_iterations = 0; cases.push(o);
    let mut o = options(&s); o.max_iterations = u32::MAX; cases.push(o);
    let mut o = options(&s); o.bounds[0] = [0.8,0.2]; cases.push(o);
    for o in cases {
        assert_eq!(s.intersect(|_,_| panic!("invalid controls"),[0.5,0.1],o)
            .unwrap_err(),Error::InvalidOptions);
    }
    assert_eq!(s.intersect(|_,_| 0.,[f64::NAN,0.1],options(&s)).unwrap_err(),Error::NonFinite);
    assert_eq!(s.intersect(|_,_| f64::NAN,[0.5,0.1],options(&s)).unwrap_err(),Error::NonFinite);
}
