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
    /// Shared budget for triangle subdivisions and support cells.
    pub max_cells: usize,
    pub max_depth: u8,
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
    pub path: Vec<u8>,
    pub bounds: Box3,
    /// Encloses an exact barycentric point of the original triangle.
    pub on_mesh: Box3,
    pub bracket: BoundaryBracket,
    pub distance: f64,
}

/// Terminal cells cover the entire construction-derived support. An uncertain
/// cell must have a distance witness to an actual point on the candidate mesh.
#[derive(Clone,Debug)]
pub struct CoverageWitness {
    pub bounds: Box3,
    pub value: I,
    pub surface_witness: Option<usize>,
    pub distance: f64,
}

#[derive(Clone,Debug)]
pub struct SpatialAudit {
    domain: Box3,
    surface: Vec<SurfaceWitness>,
    coverage: Vec<CoverageWitness>,
    distance: f64,
}
impl SpatialAudit {
    pub fn domain(&self) -> Box3 { self.domain }
    pub fn surface(&self) -> &[SurfaceWitness] { &self.surface }
    pub fn coverage(&self) -> &[CoverageWitness] { &self.coverage }
    pub fn distance_bound(&self) -> f64 { self.distance }
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

pub(super) fn audit(field: &MaterialField,judge: &mut FieldJudge,mesh: &KeptMesh,options: AuditOptions)
    -> Result<SpatialAudit,AuditError> {
    let tol = options.tolerance;
    if !tol.is_finite() || tol*0.25 <= 0. || options.max_cells == 0 || options.max_depth > 24 {
        return Err(AuditError::InvalidOptions);
    }
    let domain = field.support_bounds()?.ok_or(AuditError::NoFiniteSupport)?;
    let mut surface = Vec::new();
    let mut visited = 0;
    let mut spend = || -> Result<(),AuditError> {
        if visited >= options.max_cells { return Err(AuditError::Budget {visited}); }
        visited += 1; Ok(())
    };
    for (triangle,t) in mesh.triangles.iter().enumerate() {
        let [a,b,c] = t.map(|i| mesh.vertices[i as usize]);
        let Some(n) = super::triangle_normal(a,b,c) else { return Err(AuditError::Triangle {triangle}); };
        let mut pending = vec![([point(a)?,point(b)?,point(c)?],Vec::new())];
        while let Some((corners,path)) = pending.pop() {
            spend()?;
            let bounds = hull(&corners);
            if distance(bounds,bounds)? <= tol*0.5 {
                let mut on_mesh = [I::ZERO;3];
                for k in 0..3 { on_mesh[k] = corners[0][k].add(corners[1][k])?.add(corners[2][k])?.div(I::point(3.)?)?; }
                let p = middle(on_mesh);
                let mut d = tol*0.75;
                let mut found = None;
                // The finite ladder may decline genuinely thin material. Its failure
                // says unresolved, never that the region is safe to discard.
                for _ in 0..8 {
                    let inside = std::array::from_fn(|k| p[k]-d*n[k]);
                    let outside = std::array::from_fn(|k| p[k]+d*n[k]);
                    if let Some(bracket) = judge.bracket(inside,outside)? {
                        let error = distance(bounds,point(inside)?)?.max(distance(bounds,point(outside)?)?);
                        if error <= tol { found = Some((bracket,error)); break; }
                    }
                    d *= 0.5;
                }
                if let Some((bracket,error)) = found {
                    surface.push(SurfaceWitness {triangle,path,bounds,on_mesh,bracket,distance:error});
                    continue;
                }
                // No local witness: retain the unresolved region at the depth/budget
                // limit instead of certifying it from other subtriangles' centroids.
            }
            if path.len() >= options.max_depth as usize { return Err(AuditError::Triangle {triangle}); }
            let [a,b,c] = corners;
            let (ab,bc,ca) = (average(a,b)?,average(b,c)?,average(c,a)?);
            for (child,corners) in [[a,ab,ca],[ab,b,bc],[ca,bc,c],[ab,bc,ca]].into_iter().enumerate().rev() {
                let mut next = path.clone(); next.push(child as u8); pending.push((corners,next));
            }
        }
    }
    let mut index = crate::space::Grid::new(tol);
    for (i,w) in surface.iter().enumerate() {
        let id = u32::try_from(i).map_err(|_| AuditError::Budget {visited:usize::MAX})?;
        index.insert(middle(w.on_mesh),id);
    }
    let mut coverage = Vec::new();
    let mut pending = vec![(domain,0_u8)];
    while let Some((bounds,depth)) = pending.pop() {
        spend()?;
        let value = judge.enclose(bounds)?;
        if !value.contains(0.) {
            coverage.push(CoverageWitness {bounds,value,surface_witness:None,distance:0.});
            continue;
        }
        let center = middle(bounds);
        let mut candidates = Vec::new();
        index.around(center,|i| candidates.push(i as usize));
        let mut best = None;
        for i in candidates {
            let d = distance(bounds,surface[i].on_mesh)?;
            if d <= tol && best.is_none_or(|(_,old)| d < old) { best = Some((i,d)); }
        }
        if let Some((i,d)) = best {
            coverage.push(CoverageWitness {bounds,value,surface_witness:Some(i),distance:d});
            continue;
        }
        if depth >= options.max_depth { return Err(AuditError::Coverage {bounds}); }
        let axis = (0..3).max_by(|&a,&b| {
            let width = |k: usize| { let [lo,hi] = bounds[k].bounds(); hi-lo };
            width(a).total_cmp(&width(b))
        }).unwrap();
        let [lo,hi] = bounds[axis].bounds(); let mid = center[axis];
        if mid <= lo || mid >= hi { return Err(AuditError::Coverage {bounds}); }
        let (mut left,mut right) = (bounds,bounds);
        left[axis] = I::new(lo,mid)?; right[axis] = I::new(mid,hi)?;
        pending.push((right,depth+1)); pending.push((left,depth+1));
    }
    let error = surface.iter().map(|w| w.distance).chain(coverage.iter().map(|w| w.distance)).fold(0_f64,f64::max);
    Ok(SpatialAudit {domain,surface,coverage,distance:error})
}
