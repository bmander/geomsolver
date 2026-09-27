use gcs_core::{diagnose,io,model::{EntKind,EntRef},program,solid::RevolvedSurface,solve,syntax};

const MODEL: &str = "unit mm
point o hint(x: 0, y: 0)
point q hint(x: 0, y: 1)
point c hint(x: 3, y: 0)
ground o
ground q
ground c
line axis(o,q)
circle meridian(center: c)
radius(1mm) meridian
solid ring(face(meridian), about: axis)
surface wall(ring, meridian)
";

fn build(src: &str) -> program::Elaborated {
    let (p,errors) = syntax::parse(src);
    assert!(errors.is_empty(),"{errors:?}");
    program::elaborate(&p)
}

fn solved(src: &str) -> program::Elaborated {
    let mut e = build(src);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    assert_eq!(diagnose::diagnose(&mut e.sketch,Default::default()).dof,0);
    e
}

#[test]
fn generating_profile_bounds_enclose_entire_circular_intervals_and_snapshot_derivatives() {
    use gcs_core::interval::{Error,Interval as I};
    let mut e = solved(MODEL);
    let profile = RevolvedSurface::named(&e.sketch,0).unwrap();
    for bounds in [[0.,1.],[0.1,0.2],[0.4,0.6],[0.8,1.]] {
        let interval = I::new(bounds[0],bounds[1]).unwrap();
        let [p,d,dd] = profile.generating_profile_jet_bounds(interval).unwrap();
        assert_eq!(profile.generating_profile_bounds(interval).unwrap(),(p,d));
        for i in 0..=100 {
            let u = bounds[0]+(bounds[1]-bounds[0])*i as f64/100.;
            let a = u*std::f64::consts::TAU;
            let expected = [3.+a.cos(),0.,a.sin()];
            let tangent = [-a.sin()*std::f64::consts::TAU,0.,a.cos()*std::f64::consts::TAU];
            let curvature = [-a.cos()*std::f64::consts::TAU.powi(2),0.,-a.sin()*std::f64::consts::TAU.powi(2)];
            for k in 0..3 {
                assert!(p[k].contains(expected[k])); assert!(d[k].contains(tangent[k]));
                assert!(dd[k].contains(curvature[k]));
            }
        }
    }
    assert_eq!(profile.generating_profile_bounds(I::new(-0.1,0.).unwrap()).unwrap_err(),Error::OutsideDomain);
    assert_eq!(profile.generating_profile_bounds(I::new(1.,1.1).unwrap()).unwrap_err(),Error::OutsideDomain);
    let before = profile.generating_profile_bounds(I::ZERO).unwrap();
    let circle = e.map.ent_named("meridian").unwrap();
    let radius = e.sketch.round_radius(circle);
    e.sketch.params[radius].value = 2.;
    let after = RevolvedSurface::named(&e.sketch,0).unwrap().generating_profile_bounds(I::ZERO).unwrap();
    assert!(before.0[0].contains(4.) && after.0[0].contains(5.));
    assert_eq!(profile.generating_profile_bounds(I::ZERO).unwrap(),before);
    // Requesting curvature must not change the existing first-order query's
    // domain of successful finite enclosures.
    e.sketch.params[radius].value = 6e306;
    let huge = RevolvedSurface::named(&e.sketch,0).unwrap();
    assert!(huge.generating_profile_bounds(I::ZERO).is_ok());
    assert_eq!(huge.generating_profile_jet_bounds(I::ZERO).unwrap_err(),Error::Overflow);
}

#[test]
fn named_surface_tracks_solved_geometry_and_has_no_solver_parameters() {
    let mut e = solved(MODEL);
    let surface = e.map.ent_named("wall").unwrap();
    assert_eq!(surface.kind,EntKind::Surface);
    assert!(e.sketch.entity_params(surface).is_empty());
    assert_eq!(e.sketch.children(surface),
        [e.map.ent_named("ring").unwrap(),e.map.ent_named("meridian").unwrap()]);
    let before = RevolvedSurface::named(&e.sketch,surface.i()).unwrap();
    let p = before.at(0.,0.25).unwrap();
    assert!(p.position[0].abs() < 1e-10 && (p.position[1]-4.).abs() < 1e-10);
    let circle = e.map.ent_named("meridian").unwrap();
    let radius = e.sketch.round_radius(circle);
    e.sketch.params[radius].value = 2.;
    let after = RevolvedSurface::named(&e.sketch,surface.i()).unwrap().at(0.,0.25).unwrap();
    assert!((after.position[1]-5.).abs() < 1e-10);
    assert!((before.at(0.,0.25).unwrap().position[1]-4.).abs() < 1e-10);
    assert!(RevolvedSurface::named(&e.sketch,999).is_err());
}

#[test]
fn surface_references_survive_flat_printing_copy_paste_and_dependency_deletion() {
    let e = solved(MODEL);
    let mut p = e.program.clone();
    let text = syntax::render_flat(&mut p).unwrap().to_string();
    let again = solved(&text);
    assert_eq!(again.sketch.surfaces.len(),1);
    let surface = e.map.ent_named("wall").unwrap();
    let clip = io::copy(&e.sketch,&[surface]);
    assert_eq!(clip.surfaces.len(),1);
    assert_eq!(clip.solids.len(),1);
    let original = RevolvedSurface::named(&clip,0).unwrap().at(0.3,0.4).unwrap().position;
    let mut dst = e.sketch.clone();
    io::paste(&mut dst,&clip,0.,0.);
    assert_eq!(dst.surfaces.len(),2);
    assert_eq!(dst.surfaces[1].solid,1);
    let pasted = RevolvedSurface::named(&dst,1).unwrap().at(0.3,0.4).unwrap().position;
    for i in 0..3 { assert!((original[i]-pasted[i]).abs() < 1e-10); }
    assert!(io::without(&e.sketch,&[e.map.ent_named("meridian").unwrap()],&[]).surfaces.is_empty());
    let without = io::without(&e.sketch,&[surface],&[]);
    assert!(without.surfaces.is_empty());
    assert_eq!(without.solids.len(),1);
    assert!(io::copy(&e.sketch,&[EntRef::point(0)]).surfaces.is_empty());
}

#[test]
fn surfaces_pass_through_components_and_keep_private_names_private() {
    let inner = MODEL.replace("unit mm\n","").replace("surface wall", "private surface wall");
    let src = format!("unit mm\ncomponent Part() {{\n{inner}}}\np: Part()\n");
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"p.wall").is_none());
    let e = build(&format!("{src}surface forbidden(p.wall.solid,p.wall.edge)\n"));
    assert!(e.errors().any(|d| d.message.contains("private member")),"{:?}",e.diags);
    // A surface formal aliases the source, including declared child paths before it is built.
    let forwarded = format!("component Copy(s: surface) {{ surface out(s.solid,s.edge) }}\n\
        copy: Copy(wall)\n{MODEL}");
    let e = solved(&forwarded);
    assert_eq!(e.sketch.surfaces.len(),2);
    let a = e.map.ent_named("copy.out").unwrap();
    let b = e.map.ent_named("wall").unwrap();
    assert_eq!(e.sketch.children(a),e.sketch.children(b));
}

#[test]
fn angular_surface_spans_keep_source_parameters_and_finite_incidence() {
    let src = MODEL.replace("surface wall(ring, meridian)",
        "param begin = 180deg\nparam end = begin + 180deg\n\
         surface wall(ring, meridian, from: begin, to: end)");
    let e = solved(&src);
    let s = RevolvedSurface::named(&e.sketch,0).unwrap();
    assert_eq!(s.domain(),[[0.,1.],[0.5,1.]]);
    let raw = RevolvedSurface::read(&e.sketch,0,e.map.ent_named("meridian").unwrap()).unwrap();
    for v in [0.5,0.7,1.] {
        let p = s.at(0.31,v).unwrap();
        assert_eq!(p.position,raw.at(0.31,v).unwrap().position);
        let projected = s.projector().unwrap().project(p.position).unwrap();
        assert!(projected.incidence_error < 1e-10);
    }
    assert!(s.at(0.31,0.25).is_err());
    let outside = s.projector().unwrap().project(raw.at(0.31,0.25).unwrap().position).unwrap();
    assert!(outside.incidence_error > 1.);
    assert!(outside.parameters[1] < 0.5 || outside.parameters[1] > 1.);
    let mut p = e.program.clone();
    // Flat source parameters, like ordinary expressions, retain their declarations.
    let text = syntax::render_flat(&mut p).unwrap().to_string();
    let again = solved(&text);
    assert_eq!(RevolvedSurface::named(&again.sketch,0).unwrap().domain(),s.domain());
    let clip = io::copy(&e.sketch,&[e.map.ent_named("wall").unwrap()]);
    assert_eq!(RevolvedSurface::named(&clip,0).unwrap().domain(),s.domain());
    let mut pasted = e.sketch.clone();
    io::paste(&mut pasted,&clip,0.,0.);
    assert_eq!(RevolvedSurface::named(&pasted,1).unwrap().domain(),s.domain());
}

#[test]
fn surface_spans_follow_sweep_sense_and_component_angles() {
    let source = MODEL.replace("about: axis)","about: axis, sweep: 90deg, sense: cw)");
    let source = format!("component Portion(s: solid,e: circle,start: Angle,end: Angle) {{\n\
        surface part(s,e,from: start,to: end)\n}}\n\
        portion: Portion(ring,meridian,start: 30deg,end: 60deg)\n{source}");
    let e = solved(&source);
    let part = e.map.ent_named("portion.part").unwrap();
    let s = RevolvedSurface::named(&e.sketch,part.i()).unwrap();
    assert!((s.domain()[1][0]-1./3.).abs() < 1e-15);
    assert!((s.domain()[1][1]-2./3.).abs() < 1e-15);
    let p = s.at(0.,0.5).unwrap();
    assert!((p.position[0]-4.*std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
    assert!((p.position[1]+4.*std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
    assert!(s.projector().unwrap().project(p.position).unwrap().incidence_error < 1e-10);
}

#[test]
fn surface_spans_refuse_missing_reversed_and_out_of_sweep_angles() {
    for bounds in ["from: 30deg", "from: 30deg, from: 40deg, to: 60deg"] {
        let (_,errors) = syntax::parse(&MODEL.replace("surface wall(ring, meridian)",
            &format!("surface wall(ring, meridian, {bounds})")));
        assert!(!errors.is_empty());
    }
    for (bounds,want) in [("from: 30deg, to: 20deg","increasing"),
        ("from: -1deg, to: 90deg","within"),("from: 0deg, to: 361deg","within"),
        ("from: 0mm, to: 90deg","Angle")] {
        let e = build(&MODEL.replace("surface wall(ring, meridian)",
            &format!("surface wall(ring, meridian, {bounds})")));
        assert!(e.errors().any(|d| d.message.contains(want)),"{:?}",e.diags);
    }
}

#[test]
fn a_surface_refuses_wrong_solids_edges_and_planar_constraints() {
    for (tail,want) in [
        ("surface bad(meridian,meridian)","first argument is a solid"),
        ("surface bad(ring,axis)","not a boundary"),
        ("solid prism(face(meridian),depth: 2mm)\nsurface bad(prism,meridian)","unmodified revolution"),
        ("surface bad(missing,meridian)","no such entity"),
        ("ground wall","pins a point"),
    ] {
        let e = build(&format!("{MODEL}{tail}\n"));
        assert!(!e.ok() && e.errors().any(|d| d.message.contains(want)),"{:?}",e.diags);
    }
}

#[test]
fn analytic_projection_preserves_orientation_and_distinguishes_finite_patches() {
    let e = solved(MODEL);
    let surface = RevolvedSurface::named(&e.sketch,0).unwrap();
    let projector = surface.projector().unwrap();
    for u in [0.,0.13,0.37,0.65,1.] {
        for v in [0.,0.2,0.6,1.] {
            let s = surface.at(u,v).unwrap();
            let c = gcs_core::envelope::contact(s,gcs_core::envelope::Motion::identity()).unwrap();
            let on = projector.project(s.position).unwrap();
            assert!(on.incidence_error < 1e-10);
            for k in 0..3 { assert!((on.normal[k]-c.normal[k]).abs() < 1e-10); }
            for offset in [-0.01,0.01] {
                let p = std::array::from_fn(|k| s.position[k]+offset*c.normal[k]);
                let q = projector.project(p).unwrap();
                assert!((q.signed_residual-offset).abs() < 1e-10);
                assert!((q.incidence_error-offset.abs()).abs() < 1e-10);
            }
        }
    }
    let partial = solved(&MODEL.replace("about: axis)","about: axis,sweep: 90deg)"));
    let patch = RevolvedSurface::named(&partial.sketch,0).unwrap().projector().unwrap();
    let off = patch.project([0.,-4.,0.]).unwrap();
    assert!(off.signed_residual.abs() < 1e-10,"the point lies on the continued torus");
    assert!(off.incidence_error > 4.,"it is not on the declared quarter-turn patch");
    assert!(off.parameters[1] < 0. || off.parameters[1] > 1.);
    assert!(projector.project([f64::NAN,0.,0.]).is_err());
}

#[test]
fn straight_meridian_sphere_sections_return_all_finite_roots() {
    let e = solved("unit mm
point a hint(x: 1,y: -2)
point b hint(x: 1,y: 2)
point c hint(x: 0,y: 2)
point d hint(x: 0,y: -2)
ground a
ground b
ground c
ground d
line side(a,b)
line top(b,c)
line axis(d,c)
line bottom(d,a)
face profile(side,top,axis,bottom)
solid drum(profile,about: axis)
surface wall(drum,side)
");
    let s = RevolvedSurface::named(&e.sketch,0).unwrap();
    let [p,d,dd] = s.generating_profile_jet_bounds(gcs_core::interval::Interval::new(0.,1.).unwrap()).unwrap();
    assert!(p[0].contains(1.) && p[2].contains(-2.) && p[2].contains(2.));
    assert!(d[2].contains(4.));
    assert_eq!(dd,[gcs_core::interval::Interval::ZERO;3]);
    let points = s.line_on_sphere([0.;3],2.,0.25).unwrap();
    assert_eq!(points.len(),2);
    for (i,p) in points.iter().enumerate() {
        assert!((p.position[0].hypot(p.position[1])-1.).abs() < 1e-10);
        assert!((p.position[2]-(if i == 0 { -1. } else { 1. })*3f64.sqrt()).abs() < 1e-10);
    }
    assert_eq!(s.line_on_sphere([0.;3],1.,0.25).unwrap().len(),1);
    assert!(s.line_on_sphere([0.;3],0.5,0.25).unwrap().is_empty());
    assert!(s.line_on_sphere([0.;3],4.,0.25).unwrap().is_empty());
    assert!(s.line_on_sphere([0.;3],-1.,0.).is_err());
    assert!(s.line_on_sphere([0.;3],1.,-1.).is_err());
}
