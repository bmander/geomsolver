use super::*;
use gcs_core::edge::{EdgeTolerance,SpatialEdge};

const EDGES: &str = "
end_offset := Sphere(shifted,size: sqrt(10.25-cos(0.2rad))*1mm) in std.front
end_join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.2rad))*1mm) in std.front
end_edge := seam(first_envelope,end_offset.wall)
end_join_edge := seam(second_envelope,end_join_cut.wall)
finish := vertex(radial,end_edge)
finish_join := vertex(shared,end_join_edge)
bounded := edge(radial,from: corner,to: finish,along: axis)
joined := edge(shared,from: junction,to: finish_join,along: axis)
";

fn source() -> String { format!("{MODEL}{CORNERS}{EDGES}") }
fn controls() -> EdgeTolerance { EdgeTolerance {junction:tolerance(),point:checked()} }
fn witnesses(u: f64) -> [[f64;3];2] { [[u,0.,0.1],[u,0.,0.2]] }
fn edge(e: &program::Elaborated,index: usize,u: f64) -> SpatialEdge {
    SpatialEdge::named(&e.sketch,index,witnesses(u),controls()).unwrap()
}

#[test]
fn finite_edges_solve_axial_sections_and_preserve_declared_endpoints() {
    let e = solved(&source());
    for (index,u) in [(0,0.5),(1,1.)] {
        let curve = edge(&e,index,u);
        for fraction in [0.,0.2,0.5,0.8,1.] {
            let point = curve.sample(fraction,100).unwrap();
            let t = ((1.-fraction)*0.1f64.cos()+fraction*0.2f64.cos()).acos();
            near(point.parameters,[u,0.,t]);
            near(point.position,[3.,-u*t.sin(),u*t.cos()]);
        }
        let endpoints = curve.endpoints();
        for i in 0..2 {
            assert_eq!(curve.sample(i as f64,100).unwrap().position,endpoints[i].position);
        }
        assert_eq!(curve.sample(-0.001,100).unwrap_err(),Error::OutsideDomain);
        assert_eq!(curve.sample(1.001,100).unwrap_err(),Error::OutsideDomain);
        assert_eq!(curve.sample(f64::NAN,100).unwrap_err(),Error::NonFinite);
        assert_eq!(curve.sample(0.5,0).unwrap_err(),Error::InvalidOptions);
        let entity = e.map.ent_named(if index == 0 { "bounded" } else { "joined" }).unwrap();
        assert_eq!(entity.kind,EntKind::Edge);
        assert!(e.sketch.entity_params(entity).is_empty());
        assert_eq!(e.sketch.children(entity).len(),4);
    }
}

#[test]
fn reversing_an_edge_or_its_axis_preserves_the_same_locus() {
    let original = solved(&source());
    let reversed = solved(&source().replace("from: corner,to: finish","from: finish,to: corner")
        .replace("from: junction,to: finish_join","from: finish_join,to: junction"));
    // Reversing the revolution axis also reverses its source chart, so use a
    // separate along line to test direction reversal without changing the surface.
    let along = solved(&source().replace("along: axis","along: reverse")
        .replace("axis := line(o,q)","axis := line(o,q)\nreverse := line(q,o)"));
    for (index,u) in [(0,0.5),(1,1.)] {
        let forward = edge(&original,index,u);
        let backward = SpatialEdge::named(&reversed.sketch,index,
            [witnesses(u)[1],witnesses(u)[0]],controls()).unwrap();
        let along = edge(&along,index,u);
        for t in [0.,0.25,0.5,0.75,1.] {
            near(forward.sample(t,100).unwrap().position,backward.sample(1.-t,100).unwrap().position);
            near(forward.sample(t,100).unwrap().position,along.sample(t,100).unwrap().position);
        }
    }
}

#[test]
fn a_junction_endpoint_is_mapped_into_the_boundary_seams_second_face_chart() {
    let src = format!("{}\nheight := Sphere(o,size: sqrt(9+1.05*1.05)*1mm) in std.front\n\
        second_join_edge := seam(second_envelope,join_cut.wall)\n\
        upper_edge := seam(second_envelope,height.wall)\nupper := vertex(second_join_edge,upper_edge)\n\
        second_face := edge(second_join_edge,from: junction,to: upper,along: axis)\n",
        source().replace("\njoin_edge := seam(second_envelope,","\njoin_edge := seam(first_envelope,"));
    let e = solved(&src);
    let h = 1.05;
    let angle = ((h*h-1.+2.*0.1f64.cos())/(2.*h)).acos();
    let v = SpatialEdge::named(&e.sketch,2,[[1.,0.,0.1],[0.05,0.,angle]],controls()).unwrap();
    assert_eq!(v.endpoints()[0].parameters[0],0.);
    for f in [0.,0.5,1.] {
        let p = v.sample(f,100).unwrap();
        let h = 1.+p.parameters[0];
        near(p.position,[3.,-h*p.parameters[2].sin(),h*p.parameters[2].cos()]);
    }
    let bad = format!("{src}\nlookalike := Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm) in std.front\n\
        other := seam(second_envelope,lookalike.wall)\nbad := edge(other,junction,upper,axis)\n");
    assert!(build(&bad).errors().any(|d| d.message.contains("declared faces and boundary")));
}

#[test]
fn finite_edges_reject_lookalike_endpoints_wrong_types_and_degenerate_slices() {
    for tail in [
        "bad := edge(radial,from: corner,to: corner,along: axis)",
        "bad := edge(radial,from: corner,to: finish_join,along: axis)",
        "bad := edge(radial,from: o,to: finish,along: axis)",
        "bad := edge(radial,from: corner,to: finish,along: o)",
        "bad := edge(first_envelope,from: corner,to: finish,along: axis)",
        "bad := edge(radial,from: corner,to: finish)",
        "fix(x == 0, y == 0) bounded",
    ] {
        let (p,errors) = syntax::parse(&format!("{}{tail}\n",source()));
        assert!(!errors.is_empty() || !program::elaborate(&p).ok(),"accepted {tail}");
    }
    let e = solved(&source().replace("along: axis","along: spin_axis"));
    assert!(SpatialEdge::named(&e.sketch,0,witnesses(0.5),controls()).unwrap_err()
        .contains("distinct positions along"));
    let mut e = solved(&source());
    let q = e.map.ent_named("q").unwrap();
    let y = e.sketch.points[q.i()].y as usize;
    e.sketch.params[y].value = 0.;
    assert!(SpatialEdge::named(&e.sketch,0,witnesses(0.5),controls()).unwrap_err().contains("along line"));
    let e = solved(&source());
    assert!(SpatialEdge::named(&e.sketch,0,witnesses(0.6),controls()).is_err());
    assert!(SpatialEdge::named(&e.sketch,99,witnesses(0.5),controls()).is_err());
    for bad in [0.,-1.,f64::NAN,f64::INFINITY] {
        let mut t = controls(); t.point.incidence = bad;
        assert!(SpatialEdge::named(&e.sketch,0,witnesses(0.5),t).is_err());
    }
}

#[test]
fn edge_formals_forward_references_privacy_and_dependency_copy_are_ordinary() {
    let src = format!("component Copy(e: edge) {{\n\
        construction out := edge(e.seam,from: e.from,to: e.to,along: e.along)\n}}\n\
        copy := Copy(bounded)\n{}",source());
    let mut e = solved(&src);
    let out = e.map.ent_named("copy.out").unwrap();
    let original = SpatialEdge::named(&e.sketch,out.i(),witnesses(0.5),controls()).unwrap();
    let clip = io::copy(&e.sketch,&[out]);
    assert_eq!(clip.edges.len(),1);
    assert_eq!(clip.vertices.len(),2);
    assert_eq!(clip.seams.len(),3);
    assert!(clip.roles_of(gcs_core::model::EntRef::new(EntKind::Edge,0)).construction);
    near(SpatialEdge::named(&clip,0,witnesses(0.5),controls()).unwrap().sample(0.5,100).unwrap().position,
        original.sample(0.5,100).unwrap().position);
    let mut pasted = e.sketch.clone();
    io::paste(&mut pasted,&clip,0.,0.);
    near(SpatialEdge::named(&pasted,pasted.edges.len()-1,witnesses(0.5),controls()).unwrap()
        .sample(0.5,100).unwrap().position,original.sample(0.5,100).unwrap().position);
    let removed = io::without(&e.sketch,&[e.map.ent_named("finish").unwrap()],&[]);
    assert_eq!(removed.edges.len(),1); // The independent generating-junction edge survives.
    let src = format!("{}\ncomponent Private(s: seam,a: vertex,b: vertex,l: line) {{\n\
        private hidden := edge(s,from: a,to: b,along: l)\n}}\n\
        part := Private(radial,corner,finish,axis)\n",source());
    let p = solved(&src);
    assert!(p.map.entity_path(&p.sketch,"part.hidden").is_none());
    let p = build(&format!("{src}leak := edge(part.hidden.seam,corner,finish,axis)\n"));
    assert!(p.errors().any(|d| d.message.contains("private member")));
    let surface = e.map.ent_named("end_offset.wall").unwrap().i();
    e.sketch.surfaces[surface].span = Some([90f64.to_radians(),180f64.to_radians()]);
    assert!(SpatialEdge::named(&e.sketch,out.i(),witnesses(0.5),controls()).is_err());
    assert!(original.sample(0.5,100).is_ok());
}

#[test]
fn a_tangent_endpoint_reuses_the_checked_corner_instead_of_a_singular_slice_equation() {
    let src = source().replace("sqrt(10.25-cos(0.1rad))","sqrt(9.25)");
    let e = solved(&src);
    let v = SpatialEdge::named(&e.sketch,0,[[0.5,0.,0.],[0.5,0.,0.2]],controls()).unwrap();
    // z = 0.5*cos(t) has zero derivative at this valid endpoint. Re-solving it
    // through the slice equation would replace the corner's defining constraints.
    near(v.sample(0.,100).unwrap().position,[3.,0.,0.5]);
    assert_eq!(v.sample(0.,100).unwrap().position,v.endpoints()[0].position);
    assert!(v.sample(0.5,100).is_ok());
    assert_eq!(v.sample(0.,0).unwrap_err(),Error::InvalidOptions);
}
