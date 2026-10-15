//! Ordered finite boundaries on an exact analytic support, made of their parts. This binds
//! topology to geometry; it does not yet certify a disk interior or closed solid.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::{edge::{EdgeTolerance,SpatialEdge},envelope::Error,patch::EnvelopePatch,
    solid::{RevolvedSurface,SurfaceProjector},topology::{Direction,EdgeUse}};

#[derive(Clone,Debug)]
enum Source {
    Envelope(EnvelopePatch),
    Surface {projector:SurfaceProjector,domain:[[f64;2];2]},
}

/// What a face made of its parts lies on (#212): a generating face, or an analytic surface.
#[derive(Clone,Debug)]
pub enum Support { Envelope(crate::patch::TrimmedPatch),Surface(RevolvedSurface) }

/// One use of an edge in a face made of its parts: the edge and its identity (a caller's
/// numbering, shared by the faces it bounds), which way the loop walks it, and where its seam's
/// chart is the other face's of a junction, this face's `u` there.
#[derive(Clone,Debug)]
pub struct Use { pub edge: SpatialEdge,pub id: usize,pub direction: Direction,pub endpoint_u: Option<f64> }

/// One boundary point, retaining the shared edge position and the support chart.
/// `incidence_error` is the distance to the support evaluated in that chart.
#[derive(Clone,Copy,Debug)]
pub struct BoundaryPoint {
    pub position: [f64;3],
    /// [u,v,roll] for an envelope; [u,v,0] for a revolved surface.
    pub parameters: [f64;3],
    pub incidence_error: f64,
}

/// An immutable analytic support and its ordered finite edge snapshots. Building
/// checks the loop and endpoint witnesses. Each subsequent sample checks incidence
/// on this particular face, including conversion to a junction's second chart.
/// Loop simplicity in space, interior selection, normal orientation and global
/// curve regularity remain separate from these local boundary checks.
#[derive(Clone,Debug)]
pub struct SpatialFaceBoundary {
    pub name: String,
    source: Source,
    uses: Vec<EdgeUse>,
    edges: Vec<SpatialEdge>,
    endpoint_u: Vec<Option<f64>>,
    tolerance: EdgeTolerance,
}

impl SpatialFaceBoundary {
    /// A face on `support` bounded by `uses`, made of its parts (#212): the loop closes on the
    /// edges' own corner identities, in order, and every use's ends are checked on the support.
    pub fn new(name: String,support: Support,uses: Vec<Use>,tolerance: EdgeTolerance) -> Result<Self,String> {
        if uses.len() < 2 { return Err("a spatial face needs a closed loop of at least two edges".into()); }
        let ends: Vec<[usize;2]> = uses.iter().map(|u| {
            let v = u.edge.vertices();
            if u.direction == Direction::Forward { v } else { [v[1],v[0]] }
        }).collect();
        if (0..ends.len()).any(|i| ends[i][1] != ends[(i+1)%ends.len()][0]) {
            return Err("spatial face edges must form one ordered closed loop of shared vertex identities".into());
        }
        let source = match support {
            Support::Surface(surface) => Source::Surface {projector:surface.projector()?,domain:surface.domain()},
            Support::Envelope(patch) => Source::Envelope(patch.envelope_patch()?.clone()),
        };
        let edge_uses = uses.iter().map(|u| EdgeUse {edge:u.id,direction:u.direction}).collect();
        let endpoint_u = uses.iter().map(|u| u.endpoint_u).collect();
        let edges = uses.into_iter().map(|u| u.edge).collect();
        Self::checked(Self {name,source,uses:edge_uses,edges,endpoint_u,tolerance})
    }

    fn checked(result: Self) -> Result<Self,String> {
        for edge in 0..result.uses.len() {
            for fraction in [0.,1.] {
                result.sample_boundary(edge,fraction,1)
                    .map_err(|e| format!("invalid face boundary endpoint: {e:?}"))?;
            }
        }
        Ok(result)
    }

    pub fn edge_uses(&self) -> &[EdgeUse] { &self.uses }

    /// `edge` is an index in the ordered loop; fraction follows the directed use.
    /// Positions are shared edge positions, with a separate finite incidence check
    /// on this support. Adjacent faces never manufacture their own boundary curve.
    pub fn sample_boundary(&self,edge: usize,fraction: f64,max_iterations: u32)
        -> Result<BoundaryPoint,Error> {
        let u = self.uses.get(edge).ok_or(Error::InvalidOptions)?;
        // Check before reversing, so rounding of 1-fraction cannot hide a tiny
        // out-of-domain negative fraction or turn it into an accepted endpoint.
        if !fraction.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&fraction) { return Err(Error::OutsideDomain); }
        let fraction = if u.direction == Direction::Forward { fraction } else { 1.-fraction };
        let p = self.edges[edge].sample(fraction,max_iterations)?;
        let t = self.tolerance.point;
        let (parameters,position) = match &self.source {
            Source::Envelope(source) => {
                let mut parameters = p.parameters;
                if let Some(u) = self.endpoint_u[edge] { parameters[0] = u; }
                let c = source.at(parameters,t.normal_velocity,t.trim)?;
                (parameters,c.position)
            }
            Source::Surface {projector,domain} => {
                let b = projector.project(p.position)?;
                let [u,v] = std::array::from_fn(|i| b.parameters[i].clamp(domain[i][0],domain[i][1]));
                ([u,v,0.],b.point)
            }
        };
        let d: [f64;3] = std::array::from_fn(|i| position[i]-p.position[i]);
        let incidence_error = d[0].dhypot(d[1]).dhypot(d[2]);
        if !incidence_error.is_finite() { return Err(Error::NonFinite); }
        if incidence_error > t.incidence { return Err(Error::OutsideDomain); }
        Ok(BoundaryPoint {position:p.position,parameters,incidence_error})
    }
}
