//! Spatial corners at intersections of named seams. A declaration establishes
//! shared identity; coincidence of separately declared vertices does not merge them.
use crate::{envelope::{check_tolerance,Error,Intersection,IntersectionOptions},
    model::{EntKind,EntRef,Sketch},patch::EnvelopePatch,
    seam::{BoundarySeamTolerance,EnvelopeSeam,SeamIntersectionOptions,SeamTolerance},
    solid::SurfaceProjector};

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum VertexKind { Boundaries, Junction }

enum Definition {
    Boundaries {face: EntRef,boundaries: [usize;2]},
    Junction {seam: usize,boundary: usize,face: usize},
}

fn definition(sk: &Sketch,ids: [u32;2]) -> Result<Definition,String> {
    if ids[0] == ids[1] { return Err("a vertex needs two distinct seams".into()); }
    let read = |i: u32| sk.seams.get(i as usize).ok_or("no such seam");
    let [a,b] = [read(ids[0])?,read(ids[1])?];
    use crate::seam::SeamKind::{Boundary,Generating,Surfaces};
    let kinds = [crate::seam::kind(sk,ids[0] as usize)?,crate::seam::kind(sk,ids[1] as usize)?];
    match kinds {
        [Surfaces,_] | [_,Surfaces] =>
            Err("vertices on surface/surface seams need a surface-chart corner definition".into()),
        [Boundary,Boundary] => {
            if a.first != b.first { return Err("boundary seams must share the same named generating face".into()); }
            if a.second == b.second { return Err("a vertex needs different boundary surfaces".into()); }
            Ok(Definition::Boundaries {face:a.first,boundaries:[a.second.i(),b.second.i()]})
        }
        [Generating,Generating] => Err("a vertex currently needs at least one finite boundary seam".into()),
        [Boundary,Generating] | [Generating,Boundary] => {
            let (join,boundary,id) = if kinds[0] == Boundary { (b,a,ids[1]) } else { (a,b,ids[0]) };
            if boundary.first != join.first && boundary.first != join.second {
                return Err("the boundary seam must use a face of the generating junction".into());
            }
            Ok(Definition::Junction {seam:id as usize,boundary:boundary.second.i(),
                face:usize::from(boundary.first != join.first)})
        }
    }
}

pub(crate) fn validate(sk: &Sketch,ids: [u32;2]) -> Result<(),String> {
    definition(sk,ids).map(|_| ())
}

/// Structural chart correspondence. At a generating junction, a boundary on
/// either incident face also meets that same boundary on the other face. This
/// follows exact face and boundary identities, never coordinate proximity.
pub(crate) fn seam_chart(sk: &Sketch,index: usize,seam: usize) -> Result<Option<f64>,String> {
    let v = sk.vertices.get(index).ok_or("no such vertex")?;
    let target = sk.seams.get(seam).ok_or("no such seam")?;
    crate::seam::validate(sk,[target.first,target.second])?;
    match definition(sk,[v.first,v.second])? {
        Definition::Boundaries {..} if [v.first as usize,v.second as usize].contains(&seam) => Ok(None),
        Definition::Junction {seam:join,boundary,..} => {
            if seam == join { return Ok(None); }
            let join = &sk.seams[join];
            if target.second == EntRef::new(EntKind::Surface,boundary) {
                if target.first == join.first { return Ok(None); }
                if target.first == join.second {
                    return Ok(Some(crate::seam::junction(sk,[join.first,join.second])?[1]));
                }
            }
            Err("the edge endpoint is not incident through its declared faces and boundary".into())
        }
        _ => Err("an edge endpoint must explicitly name its seam".into()),
    }
}

/// Re-express a checked corner in an incident seam chart. A junction's
/// second face generally has a different endpoint u; copying all three numbers
/// would silently address another point on that face.
pub(crate) fn on_seam(sk: &Sketch,index: usize,seam: usize,p: [f64;3],
    junction_tolerance: SeamTolerance,point_tolerance: BoundarySeamTolerance)
    -> Result<([f64;3],[f64;3]),String> {
    let vertex = sk.vertices.get(index).ok_or("no such vertex")?;
    let endpoint_u = seam_chart(sk,index,seam)?;
    let def = definition(sk,[vertex.first,vertex.second])?;
    let (parameters,position) = match def {
        Definition::Boundaries {..} => (p,BoundaryVertex::named(sk,index,junction_tolerance.axis)?
            .position(p,point_tolerance)),
        Definition::Junction {..} => {
            let vertex = JunctionVertex::named(sk,index,junction_tolerance)?;
            let mut parameters = p;
            if let Some(u) = endpoint_u { parameters[0] = u; }
            (parameters,vertex.position(p,point_tolerance))
        }
    };
    Ok((parameters,position.map_err(|e| format!("invalid edge endpoint: {e:?}"))?))
}

pub fn kind(sk: &Sketch,index: usize) -> Result<VertexKind,String> {
    let v = sk.vertices.get(index).ok_or("no such vertex")?;
    Ok(match definition(sk,[v.first,v.second])? {
        Definition::Boundaries {..} => VertexKind::Boundaries,
        Definition::Junction {..} => VertexKind::Junction,
    })
}

/// One checked local root. Parameters remain in the canonical generating chart;
/// residual meanings follow the selected corner's solve method. A vertex has no
/// unique face normal or generating velocity, so neither is exposed as its own.
#[derive(Clone,Copy,Debug)]
pub struct SolvedVertex {
    pub position: [f64;3],
    pub parameters: [f64;3],
    pub residuals: [f64;3],
    pub iterations: u32,
}

impl From<Intersection> for SolvedVertex {
    fn from(p: Intersection) -> Self {
        Self {position:p.contact.position,parameters:p.parameters,residuals:p.residuals,iterations:p.iterations}
    }
}

fn incidence(boundary: &SurfaceProjector,p: [f64;3],tolerance: f64) -> Result<(),Error> {
    check_tolerance(tolerance)?;
    if boundary.project(p)?.incidence_error > tolerance { Err(Error::OutsideDomain) } else { Ok(()) }
}

/// A generated face meeting two finite surfaces. It owns one copy of that face
/// with its material conditions, not two boundary-seam copies of the same face.
#[derive(Clone,Debug)]
pub struct BoundaryVertex {
    pub name: String,
    face: EnvelopePatch,
    boundaries: [SurfaceProjector;2],
}

impl BoundaryVertex {
    pub fn named(sk: &Sketch,index: usize,axis_tolerance: f64) -> Result<Self,String> {
        check_tolerance(axis_tolerance).map_err(|_| "invalid clipping-axis tolerance")?;
        let v = sk.vertices.get(index).ok_or("no such vertex")?;
        let Definition::Boundaries {face,boundaries} = definition(sk,[v.first,v.second])? else {
            return Err("this vertex uses a generating junction".into());
        };
        let [a,b] = boundaries.map(|i| SurfaceProjector::named(sk,i));
        Ok(Self {name:v.name.clone(),face:EnvelopePatch::read(sk,face,axis_tolerance)?,boundaries:[a?,b?]})
    }

    pub fn domain(&self) -> [[f64;2];3] { self.face.envelope().domain() }

    pub fn position(&self,p: [f64;3],tolerance: BoundarySeamTolerance) -> Result<[f64;3],Error> {
        check_tolerance(tolerance.incidence)?;
        let c = self.face.at(p,tolerance.normal_velocity,tolerance.trim)?;
        for b in &self.boundaries { incidence(b,c.position,tolerance.incidence)?; }
        Ok(c.position)
    }

    /// Residuals and tolerances: envelope, first boundary, second boundary.
    /// Bounds and seeds restrict a local search; they do not prove global uniqueness.
    pub fn solve(&self,seed: [f64;3],options: IntersectionOptions,trim_tolerance: f64)
        -> Result<SolvedVertex,Error> {
        check_tolerance(trim_tolerance)?;
        let mut result = self.face.envelope().intersect_boundaries(
            [&self.boundaries[0],&self.boundaries[1]],seed,options)?;
        result.contact = self.face.at(result.parameters,options.residual_tolerance[0],trim_tolerance)?;
        Ok(result.into())
    }
}

/// A tangent generating junction meeting a finite surface. The boundary seam's
/// generating face is already one of the junction's faces and is not copied again.
#[derive(Clone,Debug)]
pub struct JunctionVertex {
    pub name: String,
    junction: EnvelopeSeam,
    boundary: SurfaceProjector,
    boundary_face: usize,
}

impl JunctionVertex {
    pub fn named(sk: &Sketch,index: usize,tolerance: SeamTolerance) -> Result<Self,String> {
        let v = sk.vertices.get(index).ok_or("no such vertex")?;
        let Definition::Junction {seam,boundary,face} = definition(sk,[v.first,v.second])? else {
            return Err("this vertex uses two boundary seams".into());
        };
        Ok(Self {name:v.name.clone(),junction:EnvelopeSeam::named(sk,seam,tolerance)?,
            boundary:SurfaceProjector::named(sk,boundary)?,boundary_face:face})
    }

    /// Original chart of the junction's first face, with its u fixed structurally.
    pub fn domain(&self) -> [[f64;2];3] { self.junction.domain() }

    pub fn position(&self,p: [f64;3],tolerance: BoundarySeamTolerance) -> Result<[f64;3],Error> {
        check_tolerance(tolerance.incidence)?;
        let c = self.junction.contacts(p,tolerance.normal_velocity,tolerance.trim)?;
        incidence(&self.boundary,c[self.boundary_face].position,tolerance.incidence)?;
        Ok(c[0].position)
    }

    /// Only [v,roll] are unknown. Residuals and tolerances are both generating
    /// equations followed by boundary incidence, in the junction's operand order.
    pub fn solve(&self,seed: [f64;2],options: SeamIntersectionOptions,trim_tolerance: f64)
        -> Result<SolvedVertex,Error> {
        check_tolerance(trim_tolerance)?;
        let result = self.junction.intersect(|c| self.boundary.project(c.position)
            .map(|p| p.signed_residual).unwrap_or(f64::NAN),seed,options,trim_tolerance)?;
        self.position(result.parameters,BoundarySeamTolerance {
            normal_velocity:options.normal_tolerance,incidence:options.section_tolerance,
            trim:trim_tolerance,
        })?;
        Ok(result.into())
    }
}
