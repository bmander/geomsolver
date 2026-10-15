use super::*;
use gcs_core::{edge::{EdgePoint,EdgeTolerance,Source,SpatialEdge},seam::BoundarySeam,
    spatial_face::{SpatialFaceBoundary,Support,Use},topology::Direction};

/// The spheres the two envelopes' faces are bounded by: below and above the shared
/// characteristic, and either end of it.
const BANDS: &str = "
in std.front {
  lower := Sphere(o,size: sqrt(9+0.98*0.98)*1mm)
  upper := Sphere(o,size: sqrt(9+1.02*1.02)*1mm)
  far_cut := Sphere(shifted,size: sqrt(11-2*cos(0.2rad))*1mm)
}
";

fn source() -> String { format!("{MODEL}{CORNERS}{BANDS}") }
fn controls() -> EdgeTolerance { EdgeTolerance {junction:tolerance(),point:checked()} }

/// A corner of the faces: where a band sphere meets an end sphere, on the first face (`b`), the
/// shared characteristic (`m`) or the second face (`t`), at the near (`l`) or far (`r`) end.
const CORNER_NAMES: [&str;6] = ["bl","br","ml","mr","tl","tr"];

/// A corner's witness in its own chart: the first face's, or the second's for `t`.
fn witness(name: &str) -> [f64;3] {
    let h: f64 = match &name[..1] { "b" => 0.98,"m" => 1.,_ => 1.02 };
    let t: f64 = if name.ends_with('l') { 0.1 } else { 0.2 };
    [if name.starts_with('t') { h-1. } else { h },0.,((h*h-1.+2.*t.cos())/(2.*h)).acos()]
}

/// Every part the faces are made of, built off one solved drawing.
struct Parts<'a> {
    e: &'a program::Elaborated,
    faces: [TrimmedPatch;2],
}

impl Parts<'_> {
    fn new(e: &program::Elaborated) -> Parts<'_> {
        Parts {e,faces:[face(e,"first_envelope",&[]),face(e,"second_envelope",&[])]}
    }

    fn end(name: &str) -> &'static str { if name.ends_with('l') { "join_cut.wall" } else { "far_cut.wall" } }

    /// Where the corner `name` is, at its witness.
    fn position(&self,name: &str) -> Result<[f64;3],String> {
        let p = witness(name);
        let position = match &name[..1] {
            "m" => JunctionVertex::new(name.into(),shared_with(self.e,tolerance())?,projector(self.e,Self::end(name)),0)?
                .position(p,checked()),
            band => {
                let (face,wall) = if band == "b" { (&self.faces[0],"lower.wall") } else { (&self.faces[1],"upper.wall") };
                BoundaryVertex::new(name.into(),face,[projector(self.e,wall),projector(self.e,Self::end(name))])?
                    .position(p,checked())
            }
        };
        position.map_err(|e| format!("invalid corner `{name}`: {e:?}"))
    }

    /// The corner `name` in the chart of a seam on `face` (0 or 1): a junction corner read on
    /// the second face takes that face's `u` at the shared vertex, 0.
    fn point(&self,name: &str,face: usize) -> Result<EdgePoint,String> {
        let mut parameters = witness(name);
        if name.starts_with('m') && face == 1 { parameters[0] = 0.; }
        Ok(EdgePoint {position:self.position(name)?,parameters})
    }

    /// The edge `name`, its seam and its two corners.
    fn edge(&self,name: &str) -> Result<SpatialEdge,String> {
        let (face,wall,ends) = match name {
            "bottom_edge" | "other_bottom" => (Some(0),"lower.wall",["bl","br"]),
            "top_edge" => (Some(1),"upper.wall",["tl","tr"]),
            "left_low" => (Some(0),"join_cut.wall",["bl","ml"]),
            "right_low" => (Some(0),"far_cut.wall",["br","mr"]),
            "left_high" => (Some(1),"join_cut.wall",["ml","tl"]),
            "right_high" => (Some(1),"far_cut.wall",["mr","tr"]),
            _ => (None,"",["ml","mr"]),
        };
        let source = match face {
            Some(f) => Source::Boundary(BoundarySeam::new(name.into(),&self.faces[f],projector(self.e,wall))?),
            None => Source::Junction(shared_with(self.e,tolerance())?),
        };
        let f = face.unwrap_or(0);
        let ids = ends.map(|c| CORNER_NAMES.iter().position(|&k| k == c).unwrap());
        let l = &self.e.sketch.lines[index(self.e,"ax")];
        let [a,b] = [l.p1,l.p2].map(|p| self.e.sketch.world_point(p as usize));
        SpatialEdge::new(name.into(),source,ids,[self.point(ends[0],f)?,self.point(ends[1],f)?],
            std::array::from_fn(|k| b[k]-a[k]),controls())
    }

    /// The face on `support` bounded by `edges` walked in `directions`: an edge's identity is its
    /// place in `edges`, and the shared characteristic read on the second face takes its `u`.
    fn boundary(&self,name: &str,support: usize,edges: &[(&str,Direction)]) -> Result<SpatialFaceBoundary,String> {
        let uses = edges.iter().enumerate().map(|(id,&(edge,direction))| Ok(Use {edge:self.edge(edge)?,id,direction,
            endpoint_u:(edge == "middle_edge" && support == 1).then_some(0.)}))
            .collect::<Result<Vec<_>,String>>()?;
        SpatialFaceBoundary::new(name.into(),Support::Envelope(self.faces[support].clone()),uses,controls())
    }
}

use Direction::{Forward as F,Reverse as R};
const LOWER: [(&str,Direction);4] = [("left_low",F),("middle_edge",F),("right_low",R),("bottom_edge",R)];
const UPPER: [(&str,Direction);4] = [("left_high",F),("top_edge",F),("right_high",R),("middle_edge",R)];

#[test]
fn spatial_faces_share_an_edge_and_convert_their_support_charts() {
    let e = solved(&source());
    let parts = Parts::new(&e);
    let lower = parts.boundary("lower_face",0,&LOWER).unwrap();
    let upper = parts.boundary("upper_face",1,&UPPER).unwrap();
    for f in [0.,0.25,0.5,0.75,1.] {
        let a = lower.sample_boundary(1,f,100).unwrap();
        let b = upper.sample_boundary(3,1.-f,100).unwrap();
        assert_eq!(a.position,b.position);
        assert_eq!(a.parameters[0],1.);
        assert_eq!(b.parameters[0],0.);
    }
    for (face,edges) in [(&lower,LOWER),(&upper,UPPER)] {
        for (edge,&(name,_)) in edges.iter().enumerate() {
            let a = face.sample_boundary(edge,0.,100).unwrap();
            let b = face.sample_boundary(edge,1.,100).unwrap();
            for f in [0.,0.25,0.5,0.75,1.] {
                let p = face.sample_boundary(edge,f,100).unwrap();
                let z = (1.-f)*a.position[2]+f*b.position[2];
                let (radius2,center) = match name {
                    "bottom_edge" => (0.98f64.powi(2),0.),
                    "middle_edge" => (1.,0.),
                    "top_edge" => (1.02f64.powi(2),0.),
                    "left_low" | "left_high" => (2.-2.*0.1f64.cos(),1.),
                    "right_low" | "right_high" => (2.-2.*0.2f64.cos(),1.),
                    _ => unreachable!(),
                };
                near(p.position,[3.,-(radius2-(z-center).powi(2)).sqrt(),z]);
                assert!(p.incidence_error <= controls().point.incidence);
            }
        }
    }
    let reversed = parts.boundary("lower_face",0,&[("bottom_edge",F),("right_low",F),("middle_edge",R),("left_low",R)])
        .unwrap();
    for i in 0..4 {
        near(lower.sample_boundary(i,0.25,100).unwrap().position,
            reversed.sample_boundary(3-i,0.75,100).unwrap().position);
    }
}

#[test]
fn a_spatial_face_is_a_snapshot_of_its_parts() {
    let mut e = solved(&source());
    let original = Parts::new(&e).boundary("upper_face",1,&UPPER).unwrap();
    let surface = index(&e,"second_surface");
    e.sketch.surfaces[surface].span = Some([0.2,0.4]);
    assert!(Parts::new(&e).boundary("upper_face",1,&UPPER).is_err());
    assert!(original.sample_boundary(1,0.5,100).is_ok());
    near(original.clone().sample_boundary(1,0.5,100).unwrap().position,
        original.sample_boundary(1,0.5,100).unwrap().position);
}

#[test]
fn spatial_faces_reject_the_wrong_support_and_open_loops() {
    let e = solved(&source());
    let parts = Parts::new(&e);
    // the lower loop does not lie on the second envelope
    let error = parts.boundary("bad",1,&LOWER).unwrap_err();
    assert!(error.contains("invalid face boundary endpoint"),"{error}");
    for loop_ in [
        &[("left_low",F),("middle_edge",F),("right_high",R),("bottom_edge",R)][..],
        &[("left_low",F),("middle_edge",F),("right_low",F),("bottom_edge",R)],
        &[("left_low",F),("middle_edge",F),("right_low",R)],
        &[("left_low",F)],
    ] {
        assert!(parts.boundary("bad",0,loop_).is_err(),"accepted {loop_:?}");
    }
    let face = parts.boundary("lower_face",0,&LOWER).unwrap();
    assert_eq!(face.sample_boundary(99,0.5,100).unwrap_err(),Error::InvalidOptions);
    assert_eq!(face.sample_boundary(2,-f64::from_bits(1),100).unwrap_err(),Error::OutsideDomain);
    assert_eq!(face.sample_boundary(2,f64::NAN,100).unwrap_err(),Error::NonFinite);
    assert_eq!(face.sample_boundary(2,0.,0).unwrap_err(),Error::InvalidOptions);
}

#[test]
fn surface_supported_boundaries_check_incidence_without_asserting_a_disk_interior() {
    // Two distinct edges trace the same arc in opposite directions. Their loop
    // has valid incidence but no disk interior: this reader must not be
    // confused with the still-separate geometric face/solid validity check.
    let e = solved(&source());
    let parts = Parts::new(&e);
    let support = surface(&e,"lower.wall");
    let uses = [("bottom_edge",F),("other_bottom",R)].into_iter().enumerate()
        .map(|(id,(edge,direction))| Use {edge:parts.edge(edge).unwrap(),id,direction,endpoint_u:None}).collect();
    let face = SpatialFaceBoundary::new("loop_only".into(),Support::Surface(support.clone()),uses,controls()).unwrap();
    for fraction in [0.,0.25,0.5,0.75,1.] {
        let a = face.sample_boundary(0,fraction,100).unwrap();
        let b = face.sample_boundary(1,1.-fraction,100).unwrap();
        assert_eq!(a.position,b.position);
        assert_eq!(a.parameters[2],0.);
        near(support.at(a.parameters[0],a.parameters[1]).unwrap().position,a.position);
        assert!(a.incidence_error <= controls().point.incidence);
    }
}
