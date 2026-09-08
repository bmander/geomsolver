//! Named curves shared by generating faces or analytic surfaces.
//! A seam is declared once for subsequent oriented uses in face topology.
mod boundary;
pub use boundary::{BoundarySeam,BoundarySeamTolerance};
mod surfaces;
pub use surfaces::{SurfaceSeam,SurfaceSeamPoint,SurfaceSeamIntersection,SurfaceSeamOptions};
use crate::{envelope::{self,Contact,Error,Intersection,IntersectionOptions,Motion},
    model::{edge_ends,EntKind,EntRef,Sketch},patch::EnvelopePatch};

fn distance(a: [f64;3],b: [f64;3]) -> f64 {
    (a[0]-b[0]).hypot(a[1]-b[1]).hypot(a[2]-b[2])
}

fn source_envelope(sk: &Sketch,mut e: EntRef) -> Result<usize,String> {
    if e.kind == EntKind::Patch {
        e = sk.patches.get(e.i()).ok_or("no such patch")?.source;
    }
    if e.kind != EntKind::Envelope || e.i() >= sk.envelopes.len() {
        return Err("a seam needs envelopes or patches of envelopes".into());
    }
    Ok(e.i())
}

pub(crate) fn validate(sk: &Sketch,faces: [EntRef;2]) -> Result<(),String> {
    if faces.iter().all(|f| f.kind == EntKind::Surface) {
        if faces[0] == faces[1] { return Err("a surface seam needs two distinct named surfaces".into()); }
        for f in faces { sk.surfaces.get(f.i()).ok_or("no such seam surface")?; }
        return Ok(());
    }
    if faces[1].kind == EntKind::Surface {
        source_envelope(sk,faces[0])?;
        sk.surfaces.get(faces[1].i()).ok_or("no such boundary surface")?;
        Ok(())
    } else { junction(sk,faces).map(|_| ()) }
}

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum SeamKind { Generating, Boundary, Surfaces }

/// Classify validated operands, so downstream readers cannot conflate a pair of
/// surfaces with the envelope/boundary case just because operand two is a surface.
pub fn kind(sk: &Sketch,index: usize) -> Result<SeamKind,String> {
    let s = sk.seams.get(index).ok_or("no such seam")?;
    validate(sk,[s.first,s.second])?;
    Ok(if s.first.kind == EntKind::Surface { SeamKind::Surfaces }
        else if s.second.kind == EntKind::Surface { SeamKind::Boundary }
        else { SeamKind::Generating })
}

/// Structural incidence is a shared source vertex, not a nearest-point weld.
pub(crate) fn junction(sk: &Sketch,faces: [EntRef;2]) -> Result<[f64;2],String> {
    let indices = [source_envelope(sk,faces[0])?,source_envelope(sk,faces[1])?];
    let envelopes = indices.map(|i| &sk.envelopes[i]);
    let surfaces = envelopes.map(|e| sk.surfaces.get(e.surface as usize).ok_or("no such source surface"));
    let [a,b] = surfaces;
    let [a,b] = [a?,b?];
    if a.solid != b.solid || envelopes[0].motion != envelopes[1].motion {
        return Err("a generating seam needs the same source revolution and motion".into());
    }
    let ends = [a,b].map(|s| edge_ends(sk,s.edge).ok_or("a generating seam needs source edges with endpoints"));
    let [a,b] = ends; let [a,b] = [a?,b?];
    let mut shared = vec![];
    for (i,p) in [a.0,a.1].into_iter().enumerate() {
        for (j,q) in [b.0,b.1].into_iter().enumerate() {
            if p == q { shared.push([i as f64,j as f64]); }
        }
    }
    let [parameters] = shared.as_slice() else {
        return Err("a generating seam needs exactly one shared source vertex".into());
    };
    Ok(*parameters)
}

/// Explicit geometric tolerances for a solved seam snapshot. `position` and `axis`
/// are model lengths; `normal` is the Euclidean difference between unit normals,
/// allowing either orientation of the common tangent plane.
#[derive(Clone,Copy,Debug)]
pub struct SeamTolerance {
    pub position: f64,
    pub normal: f64,
    pub axis: f64,
}

/// Search controls for the two unknowns [v,roll]. The shared endpoint's u is
/// structural and cannot be freed by a caller. Both envelope equations use the
/// same normal-velocity tolerance; the section equation has its own units.
#[derive(Clone,Copy,Debug)]
pub struct SeamIntersectionOptions {
    pub bounds: [[f64;2];2],
    pub parameter_scale: [f64;2],
    pub normal_tolerance: f64,
    pub section_tolerance: f64,
    pub max_iterations: u32,
}

#[derive(Clone,Debug)]
pub struct EnvelopeSeam {
    pub name: String,
    faces: [EnvelopePatch;2],
    ends: [f64;2],
    domain: [[f64;2];3],
    tolerance: SeamTolerance,
}

impl EnvelopeSeam {
    pub fn named(sk: &Sketch,index: usize,tolerance: SeamTolerance) -> Result<Self,String> {
        if [tolerance.position,tolerance.normal,tolerance.axis].iter()
            .any(|x| !x.is_finite() || *x < 0.) || tolerance.normal >= 1. {
            return Err("a seam needs finite nonnegative tolerances and normal tolerance below one".into());
        }
        let seam = sk.seams.get(index).ok_or("no such seam")?;
        let faces = [seam.first,seam.second];
        let ends = junction(sk,faces)?;
        let [a,b] = faces.map(|e| EnvelopePatch::read(sk,e,tolerance.axis));
        let faces = [a?,b?];
        let domains = faces.each_ref().map(|f| f.envelope().domain());
        let domain = [[ends[0],ends[0]],
            [domains[0][1][0].max(domains[1][1][0]),domains[0][1][1].min(domains[1][1][1])],
            [domains[0][2][0].max(domains[1][2][0]),domains[0][2][1].min(domains[1][2][1])]];
        if domain[1..].iter().any(|b| b[0] >= b[1]) {
            return Err("the generating faces need overlapping angular domains".into());
        }
        // Both endpoint frames undergo the same revolution and then the same rigid
        // generating motion. Their coincidence and tangent-plane angle are therefore
        // invariant in v and roll; one source-frame check establishes that relationship.
        let v = (domain[1][0]+domain[1][1])/2.;
        let contacts = [0,1].map(|i| faces[i].envelope().generating_surface().at(ends[i],v)
            .and_then(|s| envelope::contact(s,Motion::identity())));
        let [a,b] = contacts;
        let [a,b] = [a.map_err(|e| format!("invalid seam junction: {e:?}"))?,
            b.map_err(|e| format!("invalid seam junction: {e:?}"))?];
        Self::check_pair(a,b,tolerance).map_err(|_| format!(
            "`{}`: the source junction must be coincident and tangent; position error {:e} (limit {:e}), normal-plane error {:e} (limit {:e})",
            seam.name,distance(a.position,b.position),tolerance.position,
            distance(a.normal,b.normal).min(distance(a.normal,b.normal.map(|x| -x))),tolerance.normal))?;
        Ok(Self {name:seam.name.clone(),faces,ends,domain,tolerance})
    }

    fn check_pair(a: Contact,b: Contact,t: SeamTolerance) -> Result<(),Error> {
        if distance(a.position,b.position) > t.position
            || distance(a.normal,b.normal).min(distance(a.normal,b.normal.map(|x| -x))) > t.normal {
            return Err(Error::Degenerate);
        }
        Ok(())
    }

    /// The first envelope's original chart; its u coordinate is the shared endpoint.
    /// A local solve has only v and roll unknown. The implicit locus can still be
    /// singular; section intersections check rank. Orientation/face loops are separate.
    pub fn domain(&self) -> [[f64;2];3] { self.domain }

    /// The shared vertex's original u coordinate on each generating edge, in operand order.
    pub fn endpoint_parameters(&self) -> [f64;2] { self.ends }

    pub fn evaluate(&self,p: [f64;3],normal_tolerance: f64,trim_tolerance: f64)
        -> Result<Contact,Error> {
        Ok(self.contacts(p,normal_tolerance,trim_tolerance)?[0])
    }

    /// Retain both checked contacts when a downstream boundary names one specific
    /// face. Agreement within seam tolerance does not transfer finer incidence.
    pub(crate) fn contacts(&self,p: [f64;3],normal_tolerance: f64,trim_tolerance: f64)
        -> Result<[Contact;2],Error> {
        envelope::check_tolerance(normal_tolerance)?;
        envelope::check_tolerance(trim_tolerance)?;
        if !p.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        if (0..3).any(|i| p[i] < self.domain[i][0] || p[i] > self.domain[i][1]) {
            return Err(Error::OutsideDomain);
        }
        let read = |i: usize| self.faces[i].at([self.ends[i],p[1],p[2]],
            normal_tolerance,trim_tolerance);
        let a = read(0)?; let b = read(1)?;
        Self::check_pair(a,b,self.tolerance)?;
        Ok([a,b])
    }

    /// Intersect the shared characteristic with one spatial section equation.
    /// Trial points need not satisfy material trims; the retained solution must satisfy both faces.
    /// The returned parameters are [first-source u,v,roll]; its residuals are the
    /// two envelope equations followed by the section equation.
    pub fn intersect(&self,section: impl Fn(&Contact) -> f64,seed: [f64;2],
        options: SeamIntersectionOptions,trim_tolerance: f64) -> Result<Intersection,Error> {
        envelope::check_tolerance(trim_tolerance)?;
        if (0..2).any(|i| options.bounds[i][0] < self.domain[i+1][0]
            || options.bounds[i][1] > self.domain[i+1][1]) { return Err(Error::OutsideDomain); }
        let first = self.faces[0].envelope();
        let second = self.faces[1].envelope();
        let mut result = envelope::intersect_evaluated(|p| first.evaluate(p),|p,c| [
            second.evaluate([self.ends[1],p[1],p[2]])
                .map(|c| c.normal_velocity).unwrap_or(f64::NAN),section(c)],
            [self.ends[0],seed[0],seed[1]],IntersectionOptions {
                bounds:[self.domain[0],options.bounds[0],options.bounds[1]],
                parameter_scale:[1.,options.parameter_scale[0],options.parameter_scale[1]],
                residual_tolerance:[options.normal_tolerance,options.normal_tolerance,
                    options.section_tolerance],max_iterations:options.max_iterations,
            })?;
        result.contact = self.evaluate(result.parameters,options.normal_tolerance,trim_tolerance)?;
        Ok(result)
    }
}
