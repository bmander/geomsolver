//! Native source coordinates retained before candidate polylines are chained.
//! Identities are local to one immutable SweepContacts snapshot. A chained
//! vertex may have several observations with different positions: retaining
//! them does not certify coincidence or authorize a shared boundary.
use crate::solid::SweepContacts;
use std::sync::Arc;
type V3 = [f64;3];

#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Source { Face(usize), Edge(usize), Crease(usize) }

#[derive(Clone,Copy,Debug,PartialEq)]
pub struct SourcePoint {
    pub source: Source,
    /// Face (u,v), or edge/crease (t,0), in the original native chart.
    pub parameters: [f64;2],
}

/// Which continuous candidate chart generated a traced observation. None
/// marks a fold endpoint or pole where the regular chart must terminate.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Observation {
    pub point: SourcePoint,
    pub chart: Option<Chart>,
    pub event: Option<Event>,
}

/// Events found by the existing numerical tracer, not isolated trim vertices.
/// In particular, a stopped continuation or eligibility search must not be
/// promoted to a topological boundary without resolving the cause.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Event {
    Fold,
    Pole,
    ContinuationLimit,
    NormalVelocityBand { face: usize },
    EligibilityLimit,
}

#[derive(Clone,Debug)]
pub struct FacePoint {
    pub face: usize,
    pub parameters: [f64;2],
    /// Unposed position evaluated on this particular consumer.
    pub position: V3,
}
#[derive(Clone,Debug)]
pub struct Evaluation {
    /// Posed position on the first consumer. All consumers remain inspectable.
    pub position: V3,
    pub faces: Vec<FacePoint>,
}
#[derive(Clone,Debug,PartialEq)]
pub enum Error { MissingSource(Source), InvalidParameter, OutsideDomain, UnresolvedCrease,
    Surface(crate::envelope::Error), Motion(String), MissingBranch, NotContact, MissingVertex, MissingObservation, VertexCount }

impl SweepContacts {
    /// Evaluate original source geometry, including both native charts of an
    /// edge. No closest-point recovery, polyline interpolation, or visibility
    /// inference occurs here. Roll may be outside the sweep for tracer overrun.
    pub fn source_at(&self,point: SourcePoint,roll: f64) -> Result<Evaluation,Error> {
        let [u,v] = point.parameters;
        if !u.is_finite() || !v.is_finite() || !roll.is_finite() { return Err(Error::InvalidParameter); }
        let face_point = |face: usize,[u,v]: [f64;2]| -> Result<FacePoint,Error> {
            let f = self.faces().get(face).ok_or(Error::MissingSource(Source::Face(face)))?;
            let d = f.domain();
            if !(d[0][0]..=d[0][1]).contains(&u) || !(d[1][0]..=d[1][1]).contains(&v) { return Err(Error::OutsideDomain); }
            let p = f.at(u,v).map_err(Error::Surface)?;
            Ok(FacePoint {face,parameters:[u,v],position:p.position})
        };
        let faces = match point.source {
            Source::Face(face) => vec![face_point(face,[u,v])?],
            Source::Edge(edge) => {
                if !(0. ..=1.).contains(&u) || v != 0. { return Err(Error::OutsideDomain); }
                let e = self.edges().get(edge).ok_or(Error::MissingSource(point.source))?;
                let mut points = Vec::new();
                for k in 0..2 {
                    let (a,b) = e.charts[k].at(u,&self.faces()[e.faces[k]]);
                    points.push(face_point(e.faces[k],[a,b])?);
                }
                points
            }
            Source::Crease(crease) => {
                if !(0. ..=1.).contains(&u) || v != 0. { return Err(Error::OutsideDomain); }
                let c = self.creases().get(crease).ok_or(Error::MissingSource(point.source))?;
                let (a,b) = c.at(u,&self.faces()[c.faces[0]],&self.faces()[c.faces[1]]).ok_or(Error::UnresolvedCrease)?;
                vec![face_point(c.faces[0],[a.0,a.1])?,face_point(c.faces[1],[b.0,b.1])?]
            }
        };
        let pose = self.motion().at(roll).map_err(Error::Motion)?;
        Ok(Evaluation {position:pose.point(faces[0].position),faces})
    }
}

/// An evaluable candidate support, before global visibility/trim arrangement.
/// Native coordinates are (u,v) on an endpoint face, (u,roll) on a station
/// branch, (v,roll) on a stationary station, and (t,roll) on an edge/crease.
/// A station branch number is the equation's local root index, not a certified
/// continuation through a fold, periodic seam, or other singular event.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Chart {
    Endpoint { face: usize, end: super::super::swept_boundary::End },
    Station { face: usize, branch: usize },
    Stationary { face: usize, u: f64 },
    Edge(usize),
    Crease(usize),
}
impl Chart {
    pub fn at(self,sweep: &SweepContacts,parameters: [f64;2],tolerance: f64) -> Result<(SourcePoint,f64,Evaluation),Error> {
        if !(tolerance > 0. && tolerance.is_finite()) || !parameters.iter().all(|p| p.is_finite()) { return Err(Error::InvalidParameter); }
        let [a,b] = parameters;
        let domain = sweep.domain();
        let (point,roll,contact) = match self {
            Self::Endpoint {face,end} => (SourcePoint {source:Source::Face(face),parameters},
                domain[usize::from(end == super::super::swept_boundary::End::To)],false),
            Self::Station {face,branch} => {
                if !(domain[0]..=domain[1]).contains(&b) { return Err(Error::OutsideDomain); }
                let f = sweep.faces().get(face).ok_or(Error::MissingSource(Source::Face(face)))?;
                let d = f.domain();
                if !(d[0][0]..=d[0][1]).contains(&a) { return Err(Error::OutsideDomain); }
                let pose = sweep.motion().at(b).map_err(Error::Motion)?;
                let roots = f.station(a,pose).map_err(Error::Surface)?.roots(tolerance).map_err(Error::Surface)?;
                let v = roots.into_iter().find(|r| r.0 == branch).ok_or(Error::MissingBranch)?.1;
                (SourcePoint {source:Source::Face(face),parameters:[a,v]},b,true)
            }
            Self::Stationary {face,u} => (SourcePoint {source:Source::Face(face),parameters:[u,a]},b,true),
            Self::Edge(edge) => (SourcePoint {source:Source::Edge(edge),parameters:[a,0.]},b,false),
            Self::Crease(crease) => (SourcePoint {source:Source::Crease(crease),parameters:[a,0.]},b,false),
        };
        if !(domain[0]..=domain[1]).contains(&roll) { return Err(Error::OutsideDomain); }
        let evaluation = sweep.source_at(point,roll)?;
        if contact {
            let f = &evaluation.faces[0];
            let face = &sweep.faces()[f.face];
            if !face.contains(f.parameters[0],f.parameters[1]) { return Err(Error::OutsideDomain); }
            let s = face.at(f.parameters[0],f.parameters[1]).map_err(Error::Surface)?;
            let pose = sweep.motion().at(roll).map_err(Error::Motion)?;
            let c = crate::envelope::contact(s,pose).map_err(Error::Surface)?;
            if c.normal_velocity.abs() > tolerance { return Err(Error::NotContact); }
        }
        Ok((point,roll,evaluation))
    }
}

/// The tracer's immutable observations. These do not follow edits to the public
/// patch mesh; compare their evaluations to the mesh before using them. Legacy
/// cap cutting and grazing-region union cannot supply native coordinates yet.
#[derive(Clone)]
pub struct Provenance {
    snapshot: Arc<SweepContacts>,
    samples: Vec<(f64,Vec<Observation>)>,
}
impl std::fmt::Debug for Provenance {
    fn fmt(&self,f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Provenance").field("samples",&self.samples).finish_non_exhaustive()
    }
}
impl Provenance {
    pub(super) fn new(snapshot: Arc<SweepContacts>,samples: Vec<(f64,Vec<Observation>)>) -> Self { Self {snapshot,samples} }
    pub fn snapshot(&self) -> &SweepContacts { &self.snapshot }
    pub fn samples(&self) -> &[(f64,Vec<Observation>)] { &self.samples }
    /// Residual to every original consumer, including observations displaced
    /// by legacy chaining. This is a diagnostic, not a spatial error bound.
    pub fn deviations(&self,positions: &[V3]) -> Result<Vec<f64>,Error> {
        if positions.len() != self.samples.len() { return Err(Error::VertexCount); }
        if !positions.iter().flatten().all(|x| x.is_finite()) { return Err(Error::InvalidParameter); }
        positions.iter().enumerate().map(|(i,&p)| {
            Ok(self.at(i)?.iter().map(|e| crate::space::distance(p,e.position)).fold(0.,f64::max))
        }).collect()
    }
    pub fn at(&self,vertex: usize) -> Result<Vec<Evaluation>,Error> {
        let (roll,points) = self.samples.get(vertex).ok_or(Error::MissingVertex)?;
        if points.is_empty() { return Err(Error::MissingObservation); }
        points.iter().map(|p| self.snapshot.source_at(p.point,*roll)).collect()
    }
}
