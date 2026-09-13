//! Bounded spatial validation of an existing candidate, without changing its mesh.
//! Triangle subdivision proves forward distance; a complete support partition
//! proves reverse coverage. Neither uses a residual as an upper distance bound.
use super::{BoundaryBracket,FieldJudge,JudgeError,KeptMesh};
use crate::interval::{Error,Interval as I};
use crate::solid::MaterialField;

type Box3 = [I;3];

#[derive(Clone,Copy,Debug)]
pub struct AuditOptions {
    pub tolerance: f64,
    /// Total visits: reserve half for reverse coverage; unused forward visits transfer.
    pub max_cells: usize,
    /// Binary subdivision depth in each direction, at most 48.
    pub max_depth: u8,
    /// Roll evaluations per swept operand in a spatial-box query. An uncertain
    /// enclosure triggers spatial refinement; it is never treated as a strict sign.
    pub box_budget: usize,
}

#[derive(Clone,Debug,PartialEq)]
pub enum AuditError {
    InvalidOptions,
    NoFiniteSupport,
    Arithmetic(Error),
    Judge(JudgeError),
    Budget {visited: usize},
    Triangle {triangle: usize},
    Coverage {bounds: Box3},
}
impl From<Error> for AuditError { fn from(e: Error) -> Self { Self::Arithmetic(e) } }
impl From<JudgeError> for AuditError { fn from(e: JudgeError) -> Self { Self::Judge(e) } }

/// A dyadic subtriangle is covered by this box and its strict boundary bracket.
#[derive(Clone,Debug)]
pub struct SurfaceWitness {
    pub triangle: usize,
    /// Each step encodes 2*bisected_edge + child (0 or 1).
    pub path: Vec<u8>,
    pub bounds: Box3,
    /// Encloses an exact barycentric point of the original triangle.
    pub on_mesh: Box3,
    pub bracket: BoundaryBracket,
    /// Bound over the subtriangle identified by `path`, not every point of
    /// its larger axis-aligned `bounds`. Convexity transfers corner bounds.
    pub distance: f64,
}

/// Terminal cells cover the entire construction-derived support. An uncertain
/// cell must have a distance witness to an actual point on the candidate mesh.
#[derive(Clone,Debug)]
pub struct CoverageWitness {
    pub bounds: Box3,
    /// None when a mesh-distance witness made a field query unnecessary.
    pub value: Option<I>,
    pub mesh_witness: Option<MeshWitness>,
    pub distance: f64,
}

#[derive(Clone,Debug)]
pub struct SpatialAudit {
    domain: Box3,
    surface: Vec<SurfaceWitness>,
    coverage: Vec<CoverageWitness>,
    distance: f64,
    visited: [usize;2],
}
impl SpatialAudit {
    pub fn domain(&self) -> Box3 { self.domain }
    pub fn surface(&self) -> &[SurfaceWitness] { &self.surface }
    pub fn coverage(&self) -> &[CoverageWitness] { &self.coverage }
    pub fn distance_bound(&self) -> f64 { self.distance }
    /// Forward subtriangles and reverse support cells visited.
    pub fn visited(&self) -> [usize;2] { self.visited }
}

fn point(p: [f64;3]) -> Result<Box3,Error> {
    Ok([I::point(p[0])?,I::point(p[1])?,I::point(p[2])?])
}
fn middle(b: Box3) -> [f64;3] { b.map(|v| { let [a,b] = v.bounds(); a*0.5+b*0.5 }) }
fn hull(points: &[Box3]) -> Box3 {
    std::array::from_fn(|k| I::new(points.iter().map(|p| p[k].bounds()[0]).fold(f64::INFINITY,f64::min),
        points.iter().map(|p| p[k].bounds()[1]).fold(f64::NEG_INFINITY,f64::max)).unwrap())
}
fn average(a: Box3,b: Box3) -> Result<Box3,Error> {
    let mut out = [I::ZERO;3];
    for k in 0..3 { out[k] = a[k].mul(I::point(0.5)?)?.add(b[k].mul(I::point(0.5)?)?)?; }
    Ok(out)
}
/// Bounds every distance between the two boxes, including rounding.
fn distance(a: Box3,b: Box3) -> Result<f64,Error> {
    let mut sum = I::ZERO;
    for k in 0..3 { sum = sum.add(a[k].sub(b[k])?.square()?)?; }
    Ok(I::new(sum.bounds()[0].max(0.),sum.bounds()[1])?.sqrt()?.bounds()[1])
}

/// An exact convex combination of one original mesh triangle, enclosed outward.
#[derive(Clone,Debug)]
pub struct MeshWitness {
    pub triangle: usize,
    /// Point = lerp(lerp(a,b,parameters[0]),c,parameters[1]).
    pub parameters: [f64;2],
    pub point: Box3,
}

/// A strict enclosure over a tolerance neighborhood proves a contradiction.
/// All other issues below describe unfinished proof work, not incorrect geometry.
#[derive(Clone,Debug)]
pub enum AuditIssue {
    Unresolved(AuditError),
    /// The region's exact barycentre has no boundary within tolerance. This
    /// disproves the patch bound; it need not exclude every point of the region.
    OffSurface { neighborhood: Box3, value: I },
}
#[derive(Clone,Debug)]
pub struct SurfaceObligation {
    pub triangle: usize,
    /// Each step encodes 2*bisected_edge + child (0 or 1).
    pub path: Vec<u8>,
    pub bounds: Box3,
    pub issue: AuditIssue,
}
#[derive(Clone,Debug)]
pub struct CoverageObligation {
    pub bounds: Box3,
    pub depth: u8,
    pub issue: AuditError,
}

/// A bounded report survives refusal. Witnesses plus outstanding regions partition
/// the original triangles and support. Setup failures explicitly mark unattempted
/// work; a partial distance maximum is deliberately not an acceptance bound.
#[derive(Clone,Debug,Default)]
pub struct AuditReport {
    domain: Option<Box3>,
    surface: Vec<SurfaceWitness>,
    coverage: Vec<CoverageWitness>,
    surface_unfinished: Vec<SurfaceObligation>,
    coverage_unfinished: Vec<CoverageObligation>,
    not_attempted: Option<AuditError>,
    coverage_not_attempted: Option<AuditError>,
    visited: [usize;2],
}
impl AuditReport {
    pub fn domain(&self) -> Option<Box3> { self.domain }
    pub fn surface(&self) -> &[SurfaceWitness] { &self.surface }
    pub fn coverage(&self) -> &[CoverageWitness] { &self.coverage }
    pub fn surface_unfinished(&self) -> &[SurfaceObligation] { &self.surface_unfinished }
    pub fn coverage_unfinished(&self) -> &[CoverageObligation] { &self.coverage_unfinished }
    /// Invalid options prevent both spatial checks.
    pub fn not_attempted(&self) -> Option<&AuditError> { self.not_attempted.as_ref() }
    /// Missing finite support prevents reverse coverage only.
    pub fn coverage_not_attempted(&self) -> Option<&AuditError> { self.coverage_not_attempted.as_ref() }
    pub fn visited(&self) -> [usize;2] { self.visited }
    pub fn is_complete(&self) -> bool {
        self.not_attempted.is_none() && self.coverage_not_attempted.is_none()
            && self.domain.is_some() && !self.surface.is_empty() && !self.coverage.is_empty()
            && self.surface_unfinished.is_empty() && self.coverage_unfinished.is_empty()
    }
    pub(super) fn into_result(self) -> Result<SpatialAudit,AuditError> {
        if let Some(e) = self.not_attempted.or(self.coverage_not_attempted) { return Err(e); }
        if let Some(w) = self.surface_unfinished.first() {
            return Err(match &w.issue { AuditIssue::Unresolved(e) => e.clone(),
                AuditIssue::OffSurface {..} => AuditError::Triangle {triangle:w.triangle} });
        }
        if let Some(w) = self.coverage_unfinished.first() { return Err(w.issue.clone()); }
        let domain = self.domain.ok_or(AuditError::NoFiniteSupport)?;
        if self.surface.is_empty() || self.coverage.is_empty() { return Err(AuditError::InvalidOptions); }
        let distance = self.surface.iter().map(|w| w.distance).chain(self.coverage.iter().map(|w| w.distance)).fold(0_f64,f64::max);
        Ok(SpatialAudit {domain,surface:self.surface,coverage:self.coverage,distance,visited:self.visited})
    }
}

fn barycentre(corners: [Box3;3]) -> Result<Box3,Error> {
    let mut p = [I::ZERO;3];
    for k in 0..3 { p[k] = corners[0][k].add(corners[1][k])?.add(corners[2][k])?.div(I::point(3.)?)?; }
    Ok(p)
}
fn expand(p: Box3,tol: f64) -> Result<Box3,Error> {
    let mut out = p;
    for k in 0..3 { out[k] = p[k].add(I::new(-tol,tol)?)?; }
    Ok(out)
}

pub(super) fn audit(field: &MaterialField,judge: &mut FieldJudge,mesh: &KeptMesh,options: AuditOptions)
    -> Result<SpatialAudit,AuditError> { run(field,judge,mesh,options,true).into_result() }

pub(super) fn report(field: &MaterialField,judge: &mut FieldJudge,mesh: &KeptMesh,options: AuditOptions)
    -> AuditReport { run(field,judge,mesh,options,false) }

// Acceptance can stop at its first unmet obligation. Only diagnostic mode
// exposes a report, so it always finishes accounting for both partitions.
fn run(field: &MaterialField,judge: &mut FieldJudge,mesh: &KeptMesh,options: AuditOptions,stop_on_issue: bool)
    -> AuditReport {
    let mut out = AuditReport::default();
    let tol = options.tolerance;
    if !tol.is_finite() || tol*0.25 <= 0. || options.max_cells == 0 || options.max_depth > 48 || options.box_budget < 4 {
        out.not_attempted = Some(AuditError::InvalidOptions); return out;
    }
    match field.support_bounds() {
        Ok(Some(b)) => out.domain = Some(b),
        Ok(None) => out.coverage_not_attempted = Some(AuditError::NoFiniteSupport),
        Err(e) => out.coverage_not_attempted = Some(e.into()),
    }
    if stop_on_issue && out.coverage_not_attempted.is_some() { return out; }
    let forward_limit = options.max_cells/2+options.max_cells%2;
    for (triangle,t) in mesh.triangles.iter().enumerate() {
        let [a,b,c] = t.map(|i| mesh.vertices[i as usize]);
        let normal = super::triangle_normal(a,b,c).filter(|n| n.iter().all(|v| v.is_finite()));
        // Caller has checked finite vertices and index ranges.
        let mut pending = vec![([point(a).unwrap(),point(b).unwrap(),point(c).unwrap()],Vec::new())];
        while let Some((corners,path)) = pending.pop() {
            let bounds = hull(&corners);
            let attempt = (|| -> Result<(),AuditError> {
                if out.visited[0] >= forward_limit { return Err(AuditError::Budget {visited:out.visited[0]}); }
                out.visited[0] += 1;
                let n = normal.ok_or(AuditError::Triangle {triangle})?;
                let on_mesh = barycentre(corners)?;
                let mut radius = 0_f64;
                for corner in corners { radius = radius.max(distance(corner,on_mesh)?); }
                if radius <= tol*0.75 {
                    let p = middle(on_mesh);
                    let mut d = tol*0.5;
                    for _ in 0..8 {
                        let inside = std::array::from_fn(|k| p[k]-d*n[k]);
                        let outside = std::array::from_fn(|k| p[k]+d*n[k]);
                        if let Some(bracket) = judge.bracket(inside,outside)? {
                            // Convexity bounds the entire triangle by its three corners;
                            // using their bounding box instead needlessly enlarges this bound.
                            let mut error = 0_f64;
                            for corner in corners { error = error.max(distance(corner,point(inside)?)?).max(distance(corner,point(outside)?)?); }
                            if error <= tol {
                                out.surface.push(SurfaceWitness {triangle,path:path.clone(),bounds,on_mesh,bracket,distance:error});
                                return Ok(());
                            }
                        }
                        d *= 0.5;
                    }
                    // This stronger query can prove an actual distance violation.
                    // A failed search on its own remains unresolved.
                    let neighborhood = expand(on_mesh,tol)?;
                    let value = judge.enclose(neighborhood,options.box_budget)?;
                    if !value.contains(0.) {
                        out.surface_unfinished.push(SurfaceObligation {triangle,path:path.clone(),bounds,
                            issue:AuditIssue::OffSurface {neighborhood,value}});
                        return Ok(());
                    }
                }
                if path.len() >= options.max_depth as usize { return Err(AuditError::Triangle {triangle}); }
                let edge = (0..3).max_by(|&a,&b| {
                    let length = |i| crate::space::distance(middle(corners[i]),middle(corners[(i+1)%3]));
                    length(a).total_cmp(&length(b))
                }).unwrap();
                let (a,b,c) = (corners[edge],corners[(edge+1)%3],corners[(edge+2)%3]);
                let ab = average(a,b)?;
                for (child,corners) in [[a,ab,c],[ab,b,c]].into_iter().enumerate().rev() {
                    let mut next = path.clone(); next.push((2*edge+child) as u8); pending.push((corners,next));
                }
                Ok(())
            })();
            if let Err(e) = attempt {
                out.surface_unfinished.push(SurfaceObligation {triangle,path,bounds,issue:AuditIssue::Unresolved(e)});
            }
            if stop_on_issue && !out.surface_unfinished.is_empty() { return out; }
        }
    }
    let Some(domain) = out.domain else { return out; };
    // Reverse witnesses refer directly to the input mesh, independently of forward
    // success. A bad/open surface can therefore still receive a coverage report.
    let index = index::MeshIndex::new(mesh);
    let mut pending = vec![(domain,0_u8)];
    while let Some((bounds,depth)) = pending.pop() {
        let attempt = (|| -> Result<(),AuditError> {
            if out.visited[0]+out.visited[1] >= options.max_cells {
                return Err(AuditError::Budget {visited:out.visited.iter().sum()});
            }
            out.visited[1] += 1;
            if let Some((w,d)) = index.witness(bounds,tol,mesh)? {
                out.coverage.push(CoverageWitness {bounds,value:None,mesh_witness:Some(w),distance:d});
                return Ok(());
            }
            // Wide spatial boxes often straddle material regardless of roll
            // refinement. Reserve the full roll budget for small undecided cells.
            let wide = bounds.iter().any(|v| v.bounds()[1]-v.bounds()[0] > tol);
            let budget = if wide { options.box_budget.min(64) } else { options.box_budget };
            let value = judge.enclose(bounds,budget)?;
            if !value.contains(0.) {
                out.coverage.push(CoverageWitness {bounds,value:Some(value),mesh_witness:None,distance:0.});
                return Ok(());
            }
            if depth >= options.max_depth { return Err(AuditError::Coverage {bounds}); }
            let center = middle(bounds);
            let axis = (0..3).max_by(|&a,&b| {
                let width = |k: usize| { let [lo,hi] = bounds[k].bounds(); hi-lo };
                width(a).total_cmp(&width(b))
            }).unwrap();
            let [lo,hi] = bounds[axis].bounds(); let mid = center[axis];
            if mid <= lo || mid >= hi { return Err(AuditError::Coverage {bounds}); }
            let (mut left,mut right) = (bounds,bounds);
            left[axis] = I::new(lo,mid)?; right[axis] = I::new(mid,hi)?;
            pending.push((right,depth+1)); pending.push((left,depth+1));
            Ok(())
        })();
        if let Err(e) = attempt { out.coverage_unfinished.push(CoverageObligation {bounds,depth,issue:e}); }
        if stop_on_issue && !out.coverage_unfinished.is_empty() { return out; }
    }
    out
}

mod index;
