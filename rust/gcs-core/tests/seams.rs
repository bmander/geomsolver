use gcs_core::{diagnose,envelope::Error,io,model::EntKind,program,
    seam::{EnvelopeSeam,SeamIntersectionOptions,SeamTolerance},solve,syntax};

mod boundary;
mod surfaces;
mod vertices;

const MODEL: &str = "unit mm
point o hint(x: 0,y: 0)
point q hint(x: 0,y: 2)
point x hint(x: 1,y: 0)
point a hint(x: 2,y: 0)
point b hint(x: 3,y: 0)
point m hint(x: 3,y: 1)
point c hint(x: 3,y: 2)
point d hint(x: 2,y: 2)
ground o
ground q
ground x
ground a
ground b
ground m
ground c
ground d
line axis(o,q)
line spin_axis(o,x)
line bottom(a,b)
line low(b,m)
line high(m,c)
line top(c,d)
line inner(d,a)
face profile(bottom,low,high,top,inner)
solid body(profile,about: axis)
surface first_surface(body,low,from: 0deg,to: 90deg)
surface second_surface(body,high,from: 0deg,to: 90deg)
motion roll(about: spin_axis)
envelope first_envelope(first_surface,under: roll,from: -20deg,to: 20deg)
envelope second_envelope(second_surface,under: roll,from: -20deg,to: 20deg)
seam shared(first_envelope,second_envelope)
";

fn tolerance() -> SeamTolerance { SeamTolerance {position:1e-9,normal:1e-9,axis:1e-10} }
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
fn options(seam: &EnvelopeSeam) -> SeamIntersectionOptions {
    let [_,v,roll] = seam.domain();
    SeamIntersectionOptions {bounds:[v,roll],parameter_scale:[1.;2],
        normal_tolerance:1e-10,section_tolerance:1e-10,max_iterations:100}
}

#[test]
fn a_shared_characteristic_has_one_source_vertex_and_checks_both_envelopes() {
    let e = solved(MODEL);
    let seam = EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap();
    assert_eq!(seam.endpoint_parameters(),[1.,0.]);
    assert_eq!(seam.domain()[0],[1.,1.]);
    let c = seam.evaluate([1.,0.,0.1],1e-10,1e-10).unwrap();
    let expected = [3.,-0.1f64.sin(),0.1f64.cos()];
    for i in 0..3 { assert!((c.position[i]-expected[i]).abs() < 1e-10); }
    assert_eq!(seam.evaluate([1.,0.1,0.1],1e-10,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(seam.evaluate([0.,0.,0.1],1e-10,1e-10).unwrap_err(),Error::OutsideDomain);
    let p = seam.intersect(|c| c.position[1]+0.1f64.sin(),[0.05,0.],options(&seam),1e-9).unwrap();
    for i in 0..3 { assert!((p.contact.position[i]-expected[i]).abs() < 1e-9); }
    assert!((p.parameters[2]-0.1).abs() < 1e-9);
    let entity = e.map.ent_named("shared").unwrap();
    assert_eq!(entity.kind,EntKind::Seam);
    assert!(e.sketch.entity_params(entity).is_empty());
    assert_eq!(e.sketch.children(entity),
        [e.map.ent_named("first_envelope").unwrap(),e.map.ent_named("second_envelope").unwrap()]);
}

#[test]
fn seam_snapshots_roundtrip_and_copy_their_complete_dependencies() {
    let mut e = solved(MODEL);
    let mut p = e.program.clone();
    let again = solved(syntax::render_flat(&mut p).unwrap());
    assert_eq!(again.sketch.seams.len(),1);
    let entity = e.map.ent_named("shared").unwrap();
    let clip = io::copy(&e.sketch,&[entity]);
    assert_eq!(clip.seams.len(),1);
    assert_eq!(clip.envelopes.len(),2);
    assert_eq!(clip.solids.len(),1);
    let original = EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap();
    let mut dst = e.sketch.clone();
    io::paste(&mut dst,&clip,0.,0.);
    let copied = EnvelopeSeam::named(&dst,1,tolerance()).unwrap();
    assert_eq!(copied.evaluate([1.,0.,0.1],1e-9,1e-9).unwrap().position,
        original.evaluate([1.,0.,0.1],1e-9,1e-9).unwrap().position);
    assert!(io::without(&e.sketch,&[e.map.ent_named("m").unwrap()],&[]).seams.is_empty());
    let removed = io::without(&e.sketch,&[entity],&[]);
    assert!(removed.seams.is_empty());
    assert_eq!(removed.envelopes.len(),2);
    let m = e.map.ent_named("m").unwrap();
    let y = e.sketch.points[m.i()].y as usize;
    e.sketch.params[y].value = 1.25;
    let changed = EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap();
    assert!((changed.evaluate([1.,0.,0.],1e-9,1e-9).unwrap().position[2]-1.25).abs() < 1e-10);
    assert!((original.evaluate([1.,0.,0.],1e-9,1e-9).unwrap().position[2]-1.).abs() < 1e-10);
}

#[test]
fn a_seam_refuses_coincident_lookalikes_nontangent_junctions_and_incompatible_sources() {
    let mut e = solved(MODEL);
    let duplicate = e.sketch.point(3.,1.,true,"duplicate");
    let high = e.map.ent_named("high").unwrap();
    e.sketch.lines[high.i()].p1 = duplicate as u32;
    assert!(EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap_err().contains("shared source vertex"));
    let e = solved(&MODEL.replace("first_surface(body,low,","first_surface(body,high,")
        .replace("second_surface(body,high,","second_surface(body,top,"));
    assert!(EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap_err().contains("tangent"));
    let e = build(&MODEL.replace("seam shared(first_envelope,second_envelope)",
        "seam shared(first_envelope,first_envelope)"));
    assert!(e.errors().any(|d| d.message.contains("exactly one shared")));
    let e = build(&MODEL.replace("seam shared(first_envelope,second_envelope)",
        "seam shared(first_surface,second_surface)"));
    assert!(e.ok(),"{:?}",e.diags);
    assert!(EnvelopeSeam::named(&e.sketch,0,tolerance()).is_err());
    let e = build(&MODEL.replace("motion roll(about: spin_axis)",
        "motion roll(about: spin_axis)\nmotion other(about: axis)")
        .replace("second_surface,under: roll","second_surface,under: other"));
    assert!(e.errors().any(|d| d.message.contains("same source revolution and motion")));
    let e = solved(&MODEL.replace("second_surface(body,high,from: 0deg,to: 90deg)",
        "second_surface(body,high,from: 180deg,to: 270deg)"));
    assert!(EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap_err().contains("overlapping"));
}

#[test]
fn seam_formals_can_reach_forward_operands_and_private_seams_stay_private() {
    let src = format!("component Copy(s: seam) {{ seam out(s.first,s.second) }}\n\
        copy: Copy(shared)\n{MODEL}");
    let e = solved(&src);
    let copy = e.map.ent_named("copy.out").unwrap();
    assert_eq!(e.sketch.children(copy),e.sketch.children(e.map.ent_named("shared").unwrap()));
    let src = format!("{MODEL}component Part(a: envelope,b: envelope) {{\n\
        private construction seam join(a,b)\n}}\npart: Part(first_envelope,second_envelope)\n");
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"part.join").is_none());
    let e = build(&format!("{src}seam leak(part.join.first,second_envelope)\n"));
    assert!(e.errors().any(|d| d.message.contains("private member")));
}

#[test]
fn a_retained_seam_must_satisfy_material_trims_on_both_faces() {
    let source = format!("{MODEL}
component Sphere(o: point,size: Length) {{
  private point bottom hint(x: 0,y: -size)
  private point top hint(x: 0,y: size)
  ground bottom
  ground top
  private arc rim(center: o,start: bottom,end: top)
  radius(size) rim
  private line diameter(top,bottom)
  solid ball(face(rim,diameter),about: diameter)
}}
large: Sphere(o,size: 4mm)
small: Sphere(o,size: 3.1mm)
patch first_patch(first_envelope,inside: large.ball)
patch second_patch(second_envelope,inside: small.ball)
seam clipped(first_patch,second_patch)
");
    for (source,retained) in [(source.clone(),false),(source.replace("size: 3.1mm","size: 3.5mm"),true)] {
        let e = solved(&source);
        let seam = EnvelopeSeam::named(&e.sketch,1,tolerance()).unwrap();
        let evaluated = seam.evaluate([1.,0.,0.1],1e-9,1e-9);
        let intersected = seam.intersect(|c| c.position[1]+0.1f64.sin(),
            [0.05,0.],options(&seam),1e-9);
        assert_eq!(evaluated.is_ok(),retained,"{evaluated:?}");
        assert_eq!(intersected.is_ok(),retained,"{intersected:?}");
        if !retained { assert_eq!(intersected.unwrap_err(),Error::OutsideDomain); }
    }
}

#[test]
fn an_unresolved_arc_radius_cannot_hide_a_gap_at_a_shared_vertex() {
    let source = MODEL.replace("point a hint(x: 2,y: 0)","point a hint(x: 1,y: 0)")
        .replace("point c hint(x: 3,y: 2)","point c hint(x: 2,y: 2)")
        .replace("point d hint(x: 2,y: 2)","point d hint(x: 1,y: 2)")
        .replace("line high(m,c)","point arc_center hint(x: 2,y: 1)\nground arc_center\n\
            arc high(center: arc_center,start: m,end: c)\nradius(1mm) high");
    let mut e = solved(&source);
    assert!(EnvelopeSeam::named(&e.sketch,0,tolerance()).is_ok());
    let arc = e.map.ent_named("high").unwrap();
    let radius = e.sketch.arcs[arc.i()].radius as usize;
    e.sketch.params[radius].value += 1e-4;
    let error = EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap_err();
    assert!(error.contains("position error"),"{error}");
}

#[test]
fn a_stationary_family_does_not_produce_a_falsely_unique_seam_intersection() {
    let e = solved(&MODEL.replace("motion roll(about: spin_axis)",
        "motion roll(about: spin_axis,ratio: 0)"));
    let seam = EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap();
    let result = seam.intersect(|c| c.position[1]-1.,[0.05,0.],options(&seam),1e-9);
    assert_eq!(result.unwrap_err(),Error::SingularIntersection);
}

#[test]
fn seam_evaluation_rejects_invalid_numerical_controls() {
    let e = solved(MODEL);
    assert!(EnvelopeSeam::named(&e.sketch,99,tolerance()).is_err());
    let mut bad = tolerance(); bad.normal = 1.;
    assert!(EnvelopeSeam::named(&e.sketch,0,bad).is_err());
    bad = tolerance(); bad.axis = f64::NAN;
    assert!(EnvelopeSeam::named(&e.sketch,0,bad).is_err());
    let seam = EnvelopeSeam::named(&e.sketch,0,tolerance()).unwrap();
    assert!(seam.evaluate([1.,0.,0.],-1.,1e-9).is_err());
    assert!(seam.evaluate([1.,0.,0.],1e-9,f64::NAN).is_err());
    assert!(seam.evaluate([1.,0.,f64::NAN],1e-9,1e-9).is_err());
    let mut bad = options(&seam); bad.bounds[0] = [0.,1.];
    assert_eq!(seam.intersect(|_| 0.,[0.,0.],bad,1e-9).unwrap_err(),Error::OutsideDomain);
    for tolerance in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(seam.intersect(|_| panic!("invalid controls must precede callbacks"),
            [0.,0.],options(&seam),tolerance).unwrap_err(),Error::InvalidOptions);
    }
}
