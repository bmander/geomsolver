//! Ordered finite boundaries on an exact named analytic support. This binds
//! topology to geometry; it does not yet certify a disk interior or closed solid.
use crate::{edge::{EdgeTolerance,SpatialEdge},envelope::Error,
    model::{EntKind,EntRef,FaceE,Sketch},patch::EnvelopePatch,
    solid::{RevolvedSurface,SurfaceProjector},topology::{Direction,EdgeUse}};
use std::collections::BTreeSet;

/// Validate declared identity and orient the ordered loop. No coordinate welding
/// or sampled proximity can establish incidence with a different named support.
pub(crate) fn boundary(sk: &Sketch,f: &FaceE) -> Result<Vec<EdgeUse>,String> {
    let support = f.on().ok_or("a spatial face needs a named support")?;
    if !matches!(support.kind,EntKind::Surface | EntKind::Envelope | EntKind::Patch)
        || support.i() >= sk.count(support.kind) {
        return Err("a spatial face's support must be a surface, envelope or material patch".into());
    }
    if !f.holes.is_empty() { return Err("spatial faces currently take one boundary loop".into()); }
    if f.edges.len() < 2 { return Err("a spatial face needs a closed loop of at least two edges".into()); }
    let mut seen = BTreeSet::new();
    let mut ends = Vec::with_capacity(f.edges.len());
    for &edge in &f.edges {
        if edge.kind != EntKind::Edge || !seen.insert(edge) {
            return Err("a spatial face traverses distinct named spatial edges".into());
        }
        let e = sk.edges.get(edge.i()).ok_or("no such face boundary edge")?;
        crate::edge::validate(sk,e)?;
        let s = &sk.seams[e.seam as usize];
        if support != s.first && support != s.second {
            return Err(format!("edge `{}` does not belong to the face's exact named support",e.name));
        }
        ends.push([e.start,e.end]);
    }
    // For a two-edge loop both orientations may close; the first edge's declared
    // direction chooses one. Longer simple loops get their orientation from order.
    for direction in [Direction::Forward,Direction::Reverse] {
        let [start,mut end] = if direction == Direction::Forward { ends[0] } else { [ends[0][1],ends[0][0]] };
        let mut uses = vec![EdgeUse {edge:f.edges[0].i(),direction}];
        let mut visited = BTreeSet::from([start]);
        for (i,&[a,b]) in ends.iter().enumerate().skip(1) {
            if !visited.insert(end) { break; }
            let direction = if a == end { end = b; Direction::Forward }
                else if b == end { end = a; Direction::Reverse } else { break; };
            uses.push(EdgeUse {edge:f.edges[i].i(),direction});
        }
        if uses.len() == ends.len() && end == start { return Ok(uses); }
    }
    Err("spatial face edges must form one ordered closed loop of shared vertex identities".into())
}

#[derive(Clone,Debug)]
enum Source {
    Envelope(EnvelopePatch),
    Surface {projector:SurfaceProjector,domain:[[f64;2];2]},
}

/// One boundary point, retaining the shared edge position and the support chart.
/// `incidence_error` is the distance to the support evaluated in that chart.
#[derive(Clone,Copy,Debug)]
pub struct BoundaryPoint {
    pub position: [f64;3],
    /// [u,v,roll] for an envelope; [u,v,0] for a revolved surface.
    pub parameters: [f64;3],
    pub incidence_error: f64,
}

/// An immutable analytic support and its ordered finite edge snapshots. Reading
/// checks the loop and endpoint witnesses. Each subsequent sample checks incidence
/// on this particular face, including conversion to a junction's second chart.
/// Loop simplicity in space, interior selection, normal orientation and global
/// curve regularity remain separate from these local boundary checks.
#[derive(Clone,Debug)]
pub struct SpatialFaceBoundary {
    pub name: String,
    support: EntRef,
    source: Source,
    uses: Vec<EdgeUse>,
    edges: Vec<SpatialEdge>,
    endpoint_u: Vec<Option<f64>>,
    tolerance: EdgeTolerance,
}

impl SpatialFaceBoundary {
    /// The declaration's oriented edge identities, without reading solved geometry.
    pub fn declared_boundary(sk: &Sketch,index: usize) -> Result<Vec<EdgeUse>,String> {
        boundary(sk,sk.faces.get(index).ok_or("no such face")?)
    }

    /// Vertex witnesses are indexed by the sketch's spatial vertex IDs and use
    /// their canonical charts. Only vertices required by this face are read.
    pub fn named(sk: &Sketch,index: usize,vertex_parameters: &[[f64;3]],tolerance: EdgeTolerance)
        -> Result<Self,String> {
        let face = sk.faces.get(index).ok_or("no such face")?;
        let uses = boundary(sk,face)?;
        let support = face.on().unwrap();
        let source = if support.kind == EntKind::Surface {
            let surface = RevolvedSurface::named(sk,support.i())?;
            Source::Surface {projector:surface.projector()?,domain:surface.domain()}
        } else { Source::Envelope(EnvelopePatch::read(sk,support,tolerance.junction.axis)?) };
        let mut edges = Vec::with_capacity(uses.len());
        let mut endpoint_u = Vec::with_capacity(uses.len());
        for u in &uses {
            let e = &sk.edges[u.edge];
            let parameters = [e.start,e.end].map(|i| vertex_parameters.get(i as usize)
                .copied().ok_or("missing face boundary vertex witness"));
            let [a,b] = parameters;
            edges.push(SpatialEdge::named(sk,u.edge,[a?,b?],tolerance)?);
            let seam = &sk.seams[e.seam as usize];
            endpoint_u.push(if seam.second == support && support.kind != EntKind::Surface {
                Some(crate::seam::junction(sk,[seam.first,seam.second])?[1])
            } else { None });
        }
        let result = Self {name:face.name.clone(),support,source,uses,edges,endpoint_u,tolerance};
        for edge in 0..result.uses.len() {
            for fraction in [0.,1.] {
                result.sample_boundary(edge,fraction,1)
                    .map_err(|e| format!("invalid face boundary endpoint: {e:?}"))?;
            }
        }
        Ok(result)
    }

    pub fn support(&self) -> EntRef { self.support }
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
        let incidence_error = d[0].hypot(d[1]).hypot(d[2]);
        if !incidence_error.is_finite() { return Err(Error::NonFinite); }
        if incidence_error > t.incidence { return Err(Error::OutsideDomain); }
        Ok(BoundaryPoint {position:p.position,parameters,incidence_error})
    }
}
