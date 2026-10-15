use super::*;
use gcs_core::{edge::{EdgePoint,EdgeTolerance,Source,SpatialEdge},seam::BoundarySeam};

const EDGES: &str = "
end_offset := Sphere(shifted,size: sqrt(10.25-cos(0.2rad))*1mm) in std.front
end_join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.2rad))*1mm) in std.front
";

/// Which seam an edge is a stretch of: the first envelope meeting `globe.wall`, or the two
/// envelopes' shared characteristic.
#[derive(Clone,Copy,PartialEq)]
enum Along { Radial,Shared }

/// Every sphere a corner is cut by: its index is the corner's identity.
const CUTS: [&str;5] = ["offset.wall","end_offset.wall","join_cut.wall","end_join_cut.wall",
    "height.wall"];

fn source() -> String { format!("{MODEL}{CORNERS}{EDGES}") }
fn controls() -> EdgeTolerance { EdgeTolerance {junction:tolerance(),point:checked()} }
fn witnesses(u: f64) -> [[f64;3];2] { [[u,0.,0.1],[u,0.,0.2]] }
fn direction(e: &program::Elaborated,line: &str) -> [f64;3] {
    let l = &e.sketch.lines[index(e,line)];
    let [a,b] = [l.p1,l.p2].map(|p| e.sketch.world_point(p as usize));
    std::array::from_fn(|k| b[k]-a[k])
}
fn seam(e: &program::Elaborated,along: Along) -> Source {
    match along {
        Along::Radial => Source::Boundary(BoundarySeam::new("radial".into(),
            &face(e,"first_envelope",&[]),projector(e,"globe.wall")).unwrap()),
        Along::Shared => Source::Junction(shared(e)),
    }
}
/// Where the seam meets the sphere `cut`, at `p` in the seam's chart.
fn corner_at(e: &program::Elaborated,along: Along,cut: &str,p: [f64;3]) -> Result<EdgePoint,String> {
    let position = match along {
        Along::Radial => corner(e,&[],["globe.wall",cut]).position(p,checked()),
        Along::Shared => junction_with(e,cut,tolerance()).position(p,checked()),
    };
    Ok(EdgePoint {position:position.map_err(|e| format!("invalid edge endpoint: {e:?}"))?,parameters:p})
}
/// The stretch of a seam from where it meets `cuts[0]` to where it meets `cuts[1]`.
fn edge_with(e: &program::Elaborated,along: Along,cuts: [&str;2],witnesses: [[f64;3];2],
    direction: [f64;3],t: EdgeTolerance) -> Result<SpatialEdge,String> {
    let ids = cuts.map(|c| CUTS.iter().position(|&k| k == c).unwrap());
    let endpoints = [corner_at(e,along,cuts[0],witnesses[0])?,corner_at(e,along,cuts[1],witnesses[1])?];
    SpatialEdge::new("edge".into(),seam(e,along),ids,endpoints,direction,t)
}
/// `bounded` (radial, `offset` to `end_offset`) or `joined` (shared, `join_cut` to `end_join_cut`).
fn edge(e: &program::Elaborated,along: Along,u: f64) -> SpatialEdge {
    let cuts = if along == Along::Radial { ["offset.wall","end_offset.wall"] }
        else { ["join_cut.wall","end_join_cut.wall"] };
    edge_with(e,along,cuts,witnesses(u),direction(e,"ax"),controls()).unwrap()
}

#[test]
fn finite_edges_solve_axial_sections_and_preserve_their_corners() {
    let e = solved(&source());
    for (along,u) in [(Along::Radial,0.5),(Along::Shared,1.)] {
        let curve = edge(&e,along,u);
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
    }
}

#[test]
fn reversing_an_edge_or_its_direction_preserves_the_same_locus() {
    let e = solved(&source());
    for (along,u,cuts) in [(Along::Radial,0.5,["offset.wall","end_offset.wall"]),
        (Along::Shared,1.,["join_cut.wall","end_join_cut.wall"])] {
        let forward = edge(&e,along,u);
        let [a,b] = witnesses(u);
        let backward = edge_with(&e,along,[cuts[1],cuts[0]],[b,a],direction(&e,"ax"),controls()).unwrap();
        let reversed = edge_with(&e,along,cuts,[a,b],direction(&e,"ax").map(|x| -x),controls()).unwrap();
        for t in [0.,0.25,0.5,0.75,1.] {
            near(forward.sample(t,100).unwrap().position,backward.sample(1.-t,100).unwrap().position);
            near(forward.sample(t,100).unwrap().position,reversed.sample(t,100).unwrap().position);
        }
    }
}

#[test]
fn a_junction_corner_is_read_in_the_boundary_seams_second_face_chart() {
    let src = format!("{}\nheight := Sphere(o,size: sqrt(9+1.05*1.05)*1mm) in std.front\n",source());
    let e = solved(&src);
    let h = 1.05;
    let angle = ((h*h-1.+2.*0.1f64.cos())/(2.*h)).acos();
    // The second envelope meeting `join_cut`, from where the shared characteristic meets it on
    // the first face to the corner the second face makes with `height`.
    let second = face(&e,"second_envelope",&[]);
    let along = Source::Boundary(BoundarySeam::new("second_join".into(),&second,projector(&e,"join_cut.wall")).unwrap());
    let joined = JunctionVertex::new("junction".into(),shared(&e),projector(&e,"join_cut.wall"),0).unwrap()
        .position([1.,0.,0.1],checked()).unwrap();
    let upper = [0.05,0.,angle];
    let upper = EdgePoint {position:BoundaryVertex::new("upper".into(),&second,
        [projector(&e,"join_cut.wall"),projector(&e,"height.wall")]).unwrap().position(upper,checked()).unwrap(),
        parameters:upper};
    // the junction's shared vertex is the second face's u = 0
    let start = EdgePoint {position:joined,parameters:[0.,0.,0.1]};
    let v = SpatialEdge::new("second_face".into(),along.clone(),[2,4],[start,upper],direction(&e,"ax"),controls()).unwrap();
    assert_eq!(v.endpoints()[0].parameters[0],0.);
    for f in [0.,0.5,1.] {
        let p = v.sample(f,100).unwrap();
        let h = 1.+p.parameters[0];
        near(p.position,[3.,-h*p.parameters[2].sin(),h*p.parameters[2].cos()]);
    }
    // the same corner left in the first face's chart is another point of the second face
    let unconverted = EdgePoint {parameters:[1.,0.,0.1],..start};
    assert!(SpatialEdge::new("bad".into(),along,[2,4],[unconverted,upper],direction(&e,"ax"),controls()).is_err());
}

#[test]
fn finite_edges_reject_one_corner_twice_and_degenerate_slices() {
    let e = solved(&source());
    let cuts = ["offset.wall","end_offset.wall"];
    let at = witnesses(0.5)[0];
    let error = edge_with(&e,Along::Radial,[cuts[0],cuts[0]],[at,at],direction(&e,"ax"),controls())
        .unwrap_err();
    assert!(error.contains("two distinct corners"),"{error}");
    let error = edge_with(&e,Along::Radial,cuts,witnesses(0.5),direction(&e,"spin_axis"),controls()).unwrap_err();
    assert!(error.contains("distinct positions along"),"{error}");
    let error = edge_with(&e,Along::Radial,cuts,witnesses(0.5),[0.;3],controls()).unwrap_err();
    assert!(error.contains("along line"),"{error}");
    // a witness off the corner is refused where the corner is read
    assert!(edge_with(&e,Along::Radial,cuts,witnesses(0.6),direction(&e,"ax"),controls()).is_err());
    let (source,endpoints) = (seam(&e,Along::Radial),[corner_at(&e,Along::Radial,cuts[0],witnesses(0.5)[0]).unwrap(),
        corner_at(&e,Along::Radial,cuts[1],witnesses(0.5)[1]).unwrap()]);
    // the seam's own check of an endpoint: one moved off it disagrees with the corner
    let moved = EdgePoint {position:[3.,0.,0.],..endpoints[0]};
    let error = SpatialEdge::new("bad".into(),source.clone(),[0,1],[moved,endpoints[1]],direction(&e,"ax"),controls())
        .unwrap_err();
    assert!(error.contains("disagree in position"),"{error}");
    for bad in [0.,-1.,f64::NAN,f64::INFINITY] {
        let mut t = controls(); t.point.incidence = bad;
        assert!(SpatialEdge::new("bad".into(),source.clone(),[0,1],endpoints,direction(&e,"ax"),t).is_err());
    }
}

#[test]
fn an_edge_is_a_snapshot_of_its_parts() {
    let mut e = solved(&source());
    let original = edge(&e,Along::Radial,0.5);
    let surface = index(&e,"end_offset.wall");
    e.sketch.surfaces[surface].span = Some([90f64.to_radians(),180f64.to_radians()]);
    assert!(edge_with(&e,Along::Radial,["offset.wall","end_offset.wall"],witnesses(0.5),
        direction(&e,"ax"),controls()).is_err());
    assert!(original.sample(0.5,100).is_ok());
    near(original.clone().sample(0.5,100).unwrap().position,original.sample(0.5,100).unwrap().position);
}

#[test]
fn a_tangent_endpoint_reuses_the_checked_corner_instead_of_a_singular_slice_equation() {
    let src = source().replace("sqrt(10.25-cos(0.1rad))","sqrt(9.25)");
    let e = solved(&src);
    let v = edge_with(&e,Along::Radial,["offset.wall","end_offset.wall"],[[0.5,0.,0.],[0.5,0.,0.2]],
        direction(&e,"ax"),controls()).unwrap();
    // z = 0.5*cos(t) has zero derivative at this valid endpoint. Re-solving it
    // through the slice equation would replace the corner's defining constraints.
    near(v.sample(0.,100).unwrap().position,[3.,0.,0.5]);
    assert_eq!(v.sample(0.,100).unwrap().position,v.endpoints()[0].position);
    assert!(v.sample(0.5,100).is_ok());
    assert_eq!(v.sample(0.,0).unwrap_err(),Error::InvalidOptions);
}
