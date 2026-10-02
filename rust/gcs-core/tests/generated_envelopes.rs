use gcs_core::{envelope::{Error,GeneratedEnvelope,IntersectionOptions},io,
    model::EntKind,program,solve,syntax};

const MODEL: &str = "unit mm
o := point hint(x: 0,y: 0)
q := point hint(x: 0,y: 1)
x := point hint(x: 1,y: 0)
c := point hint(x: 3,y: 0)
ground o
ground q
ground x
ground c
axis := line(o,q)
spin_axis := line(o,x)
meridian := circle(center: c)
radius(1mm) meridian
ring := solid(face(meridian),about: axis)
wall := surface(ring,meridian)
roll := motion(about: spin_axis)
generated := envelope(wall,under: roll,from: -20deg,to: 20deg)
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
    e
}
fn near(a: [f64;3],b: [f64;3]) {
    for i in 0..3 { assert!((a[i]-b[i]).abs() < 1e-9,"{a:?} != {b:?}"); }
}

#[test]
fn named_envelope_solves_a_characteristic_and_reports_off_locus_trials() {
    let e = solved(MODEL);
    let generated = GeneratedEnvelope::named(&e.sketch,0).unwrap();
    let trial = generated.evaluate([0.125,0.1,0.1]).unwrap();
    assert!(trial.normal_velocity.abs() > 1.);
    let p = generated.intersect(|p,_| [p[0]-0.125,p[2]-0.1],[0.125,0.45,0.1],
        IntersectionOptions {bounds:[[0.125,0.125],[0.25,0.75],[0.1,0.1]],
            parameter_scale:[1.;3],residual_tolerance:[1e-11;3],max_iterations:80}).unwrap();
    let h = 0.5f64.sqrt();
    near(p.contact.position,[-3.-h,-h*0.1f64.sin(),h*0.1f64.cos()]);
    near(p.contact.normal,[h,h*0.1f64.sin(),-h*0.1f64.cos()]);
    assert!(p.contact.normal_velocity.abs() < 1e-11);
    assert_eq!(generated.evaluate([0.1,0.2,1.]).unwrap_err(),Error::OutsideDomain);
    assert_eq!(generated.evaluate([-0.1,0.2,0.]).unwrap_err(),Error::OutsideDomain);
    assert!(GeneratedEnvelope::named(&e.sketch,99).is_err());
    let bad_bounds = IntersectionOptions {bounds:[[0.,1.],[0.,1.],[-1.,1.]],
        parameter_scale:[1.;3],residual_tolerance:[1e-10;3],max_iterations:80};
    assert_eq!(generated.intersect(|_,_| [0.,0.],[0.,0.,0.],bad_bounds).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn retained_contacts_require_the_locus_and_keep_their_snapshot_domain() {
    let mut e = solved(MODEL);
    let generated = GeneratedEnvelope::named(&e.sketch,0).unwrap();
    let domain = generated.domain();
    let trial = [0.125,0.1,0.1];
    assert!(generated.evaluate(trial).unwrap().normal_velocity.abs() > 1.);
    assert_eq!(generated.at(trial,1e-10).unwrap_err(),Error::OutsideDomain);
    let on = [0.125,0.,0.1];
    assert!(generated.at(on,1e-10).is_ok());
    for tolerance in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(generated.at(on,tolerance).unwrap_err(),Error::InvalidOptions);
    }
    e.sketch.envelopes[0].roll = [0.2,0.3];
    assert_eq!(generated.domain(),domain);
    assert!(generated.at(on,1e-10).is_ok());
    let changed = GeneratedEnvelope::named(&e.sketch,0).unwrap();
    assert_eq!(changed.at(on,1e-10).unwrap_err(),Error::OutsideDomain);
    e.sketch.envelopes[0].roll = [f64::NAN,0.3];
    assert!(GeneratedEnvelope::named(&e.sketch,0).is_err());
}

#[test]
fn envelope_searches_respect_the_declared_source_span() {
    let e = solved(&MODEL.replace("wall := surface(ring,meridian)",
        "wall := surface(ring,meridian,from: 180deg,to: 360deg)"));
    let generated = GeneratedEnvelope::named(&e.sketch,0).unwrap();
    assert_eq!(generated.domain()[1],[0.5,1.]);
    assert_eq!(generated.evaluate([0.125,0.,0.1]).unwrap_err(),Error::OutsideDomain);
    let options = IntersectionOptions {bounds:[[0.125,0.125],[0.5,1.],[0.1,0.1]],
        parameter_scale:[1.;3],residual_tolerance:[1e-10;3],max_iterations:100};
    let result = generated.intersect(|p,_| [p[0]-0.125,p[2]-0.1],
        [0.125,0.6,0.1],options).unwrap();
    assert!((result.parameters[1]-0.5).abs() < 1e-9);
    let mut bad = options;
    bad.bounds[1] = [0.,1.];
    assert_eq!(generated.intersect(|_,_| [0.,0.],[0.125,0.6,0.1],bad).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn envelope_declarations_round_trip_and_follow_dependency_copy_delete() {
    let e = solved(MODEL);
    let mut p = e.program.clone();
    let text = syntax::render_flat(&mut p).unwrap().to_string();
    let again = solved(&text);
    assert_eq!(again.sketch.envelopes.len(),1);
    let entity = e.map.ent_named("generated").unwrap();
    assert_eq!(entity.kind,EntKind::Envelope);
    assert!(e.sketch.entity_params(entity).is_empty());
    assert_eq!(e.sketch.children(entity),[e.map.ent_named("wall").unwrap(),e.map.ent_named("roll").unwrap()]);
    let clip = io::copy(&e.sketch,&[entity]);
    assert_eq!(clip.envelopes.len(),1);
    assert_eq!(clip.surfaces.len(),1);
    assert_eq!(clip.motions.len(),1);
    let mut dst = e.sketch.clone();
    io::paste(&mut dst,&clip,0.,0.);
    assert_eq!(dst.envelopes.len(),2);
    assert_eq!(dst.envelopes[1].surface,1);
    assert_eq!(dst.envelopes[1].motion,1);
    near(GeneratedEnvelope::named(&dst,1).unwrap().evaluate([0.125,0.,0.1]).unwrap().position,
        GeneratedEnvelope::named(&e.sketch,0).unwrap().evaluate([0.125,0.,0.1]).unwrap().position);
    for name in ["wall","roll","meridian","spin_axis"] {
        assert!(io::without(&e.sketch,&[e.map.ent_named(name).unwrap()],&[]).envelopes.is_empty());
    }
    assert!(io::copy(&e.sketch,&[e.map.ent_named("o").unwrap()]).envelopes.is_empty());
}

#[test]
fn envelope_formals_alias_forward_source_dependencies_and_private_members() {
    let src = format!("component Copy(e: envelope,limit: Angle) {{\n\
        private out := envelope(e.surface,under: e.motion,from: -limit,to: limit)\n}}\n\
        copy := Copy(generated,limit: 10deg)\n{MODEL}");
    let e = solved(&src);
    assert_eq!(e.sketch.envelopes.len(),2);
    assert!(e.map.entity_path(&e.sketch,"copy.out").is_none());
    let bad = build(&format!("{src}bad := envelope(copy.out.surface,under: roll,from: -10deg,to: 10deg)\n"));
    assert!(bad.errors().any(|d| d.message.contains("private member")),"{:?}",bad.diags);
}

#[test]
fn envelopes_reject_invalid_dependencies_and_domains() {
    for (tail,want) in [
        ("bad := envelope(axis,roll,from: -1deg,to: 1deg)","valid surface"),
        ("bad := envelope(wall,axis,from: -1deg,to: 1deg)","valid motion"),
        ("bad := envelope(wall,roll,from: 1mm,to: 2mm)","envelope bound"),
        ("bad := envelope(wall,roll,from: 1deg,to: 1deg)","increasing"),
        ("bad := envelope(wall,roll,from: 1deg,to: -1deg)","increasing"),
        ("ground generated","pins a point"),
    ] {
        let e = build(&format!("{MODEL}{tail}\n"));
        assert!(!e.ok() && e.errors().any(|d| d.message.contains(want)),"{:?}",e.diags);
    }
    for args in ["wall,roll", "wall,roll,from: 0deg", "wall,roll,from: 0deg,to: 1deg,to: 2deg"] {
        assert!(!syntax::parse(&format!("bad := envelope({args})\n")).1.is_empty());
    }
}

#[test]
fn envelope_roots_on_domain_edges_are_reached_without_accepting_false_boundary_roots() {
    let e = solved(MODEL);
    let generated = GeneratedEnvelope::named(&e.sketch,0).unwrap();
    let options = IntersectionOptions {bounds:[[0.125,0.125],[0.,0.25],[0.1,0.1]],
        parameter_scale:[1.;3],residual_tolerance:[1e-11;3],max_iterations:80};
    let p = generated.intersect(|_,_| [0.,0.],[0.125,0.05,0.1],options).unwrap();
    assert!(p.parameters[1].abs() < 1e-12);
    assert!(p.residuals[0].abs() < 1e-11);
    let excluded = IntersectionOptions {bounds:[[0.125,0.125],[0.0001,0.25],[0.1,0.1]],..options};
    assert_eq!(generated.intersect(|_,_| [0.,0.],[0.125,0.05,0.1],excluded).unwrap_err(),
        Error::NotConverged);
    // A surface family with no motion has an identically zero equation; it does not
    // define an isolated characteristic, even though its initial residual is zero.
    let e = solved(&MODEL.replace("roll := motion(about: spin_axis)","roll := motion(about: spin_axis,ratio: 0)"));
    let stationary = GeneratedEnvelope::named(&e.sketch,0).unwrap();
    assert_eq!(stationary.intersect(|_,_| [0.,0.],[0.125,0.05,0.1],options).unwrap_err(),
        Error::SingularIntersection);
}
