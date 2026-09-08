use super::*;
use gcs_core::{seam::{self,BoundarySeam,SeamKind,SurfaceSeam,SurfaceSeamOptions},
    solid::RevolvedSurface};

fn source() -> String {
    format!("{MODEL}{}\nseam section_curve(first_surface,cut.wall)\n",boundary::SPHERE)
}
fn read(e: &program::Elaborated) -> SurfaceSeam {
    SurfaceSeam::named(&e.sketch,e.map.ent_named("section_curve").unwrap().i()).unwrap()
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
    let s = read(&e);
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
        let wall = e.map.ent_named("cut.wall").unwrap();
        let second = RevolvedSurface::named(&e.sketch,wall.i()).unwrap()
            .at(p.parameters[1][0],p.parameters[1][1]).unwrap();
        for i in 0..3 { near(p.position[i],second.position[i]); }
        assert!(p.incidence_error <= 1e-10);
    }
    for (name,kind) in [("shared",SeamKind::Generating),("boundary_edge",SeamKind::Boundary),
        ("section_curve",SeamKind::Surfaces)] {
        assert_eq!(seam::kind(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap(),kind);
    }
    let i = e.map.ent_named("section_curve").unwrap().i();
    assert!(EnvelopeSeam::named(&e.sketch,i,tolerance()).is_err());
    assert!(BoundarySeam::named(&e.sketch,i,1e-10).is_err());
    assert!(SurfaceSeam::named(&e.sketch,0).is_err());
}

#[test]
fn surface_seams_preserve_finite_membership_and_snapshot_domains() {
    let mut e = solved(&source());
    let original = read(&e);
    let wall = e.map.ent_named("cut.wall").unwrap().i();
    e.sketch.surfaces[wall].span = Some([180f64.to_radians(),270f64.to_radians()]);
    let limited = read(&e);
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
    let s = read(&e);
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
    assert_eq!(read(&e).evaluate([0.,0.1],1e-10,1e-6).unwrap_err(),Error::SingularIntersection);
    let e = solved(&source().replace("seam section_curve(first_surface,cut.wall)",
        "surface duplicate(body,low,from: 0deg,to: 90deg)\nseam section_curve(first_surface,duplicate)"));
    assert_eq!(read(&e).evaluate([0.5,0.1],1e-10,1e-6).unwrap_err(),Error::SingularIntersection);
}

#[test]
fn surface_seam_controls_fail_before_callbacks_and_unsupported_vertices_fail_at_build() {
    let e = solved(&source());
    let s = read(&e);
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
    for tail in ["vertex unsupported(section_curve,boundary_edge)","seam same(first_surface,first_surface)"] {
        let e = build(&format!("{}{tail}\n",source()));
        assert!(!e.ok(),"{tail}");
    }
}

#[test]
fn surface_seams_copy_dependencies_and_obey_component_privacy() {
    let e = solved(&source());
    let entity = e.map.ent_named("section_curve").unwrap();
    let copied = io::copy(&e.sketch,&[entity]);
    assert_eq!(copied.surfaces.len(),2);
    assert!(copied.envelopes.is_empty() && copied.motions.is_empty());
    assert!(SurfaceSeam::named(&copied,0).unwrap().evaluate([0.5,0.1],1e-10,1e-6).is_ok());
    let mut dst = e.sketch.clone();
    io::paste(&mut dst,&copied,0.,0.);
    assert!(SurfaceSeam::named(&dst,dst.seams.len()-1).unwrap().evaluate([0.5,0.1],1e-10,1e-6).is_ok());
    assert!(!io::without(&e.sketch,&[e.map.ent_named("cut.wall").unwrap()],&[])
        .seams.iter().any(|s| s.name == "section_curve"));
    let src = format!("component Pair(a: surface,b: surface) {{ private seam hidden(a,b) }}\n\
        part: Pair(first_surface,cut.wall)\n{}",source());
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"part.hidden").is_none());
    let e = build(&format!("{src}\nseam leak(part.hidden.first,cut.wall)\n"));
    assert!(e.errors().any(|d| d.message.contains("private member")));
}
