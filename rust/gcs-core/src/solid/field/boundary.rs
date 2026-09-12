//! Spatially bounded extraction from a material field, without an assumed SDF.
use super::{norm,point,Error,I,V,MaterialEvaluator,SweepError};
use crate::{interval::minimum::Options,topology::ClosedShell};
use std::collections::BTreeMap;
mod witness_index;
use witness_index::WitnessIndex;
mod components;
pub use components::BoundaryComponent;

type GridPoint = [u32;3];

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum BoundaryStage { Partition, Triangulation, Certification }

#[derive(Clone,Copy,Debug)]
pub struct BoundaryOptions {
    /// Maximum two-sided boundary distance in model length units.
    pub spatial_tolerance: f64,
    /// Octree depth cap, at most 24. All surface cells use one finest grid.
    pub max_depth: u8,
    pub max_cells: usize,
    /// Per-sweep refinement controls. Cell-center queries may request a larger
    /// field width proportional to cell size. Corner queries refine toward this
    /// tolerance only if their sign is unresolved; signs always require bounds.
    pub sweep: Options,
    /// The box to extract within, or `None` to derive it from the material's own
    /// support as before. A given box is taken as it stands — padded by the
    /// spatial tolerance like a derived one, and sized by the same depth rule —
    /// so a caller may extract one neighbourhood rather than the whole field.
    /// Nothing outside it is looked at, and no claim is made about what lies
    /// there: the result is the boundary *within* the box, and its shell is
    /// closed only if the box holds the whole of a component.
    pub domain: Option<V>,
}

#[derive(Clone,Debug,PartialEq)]
pub enum BoundaryError {
    InvalidOptions,
    NoFiniteSupport,
    Arithmetic(Error),
    Field(SweepError),
    ResolutionLimit,
    CellBudget {visited:usize},
    AmbiguousPoint {point:[f64;3],value:I},
    UnresolvedCell {cell:V},
    NoBoundary,
    Topology {error:crate::topology::Error,components:Vec<BoundaryComponent>},
}
impl From<Error> for BoundaryError { fn from(e: Error) -> Self { Self::Arithmetic(e) } }
impl From<SweepError> for BoundaryError { fn from(e: SweepError) -> Self { Self::Field(e) } }

/// A terminal cell in the complete dyadic partition of the derived search box.
/// `value` includes the center enclosure plus the one-Lipschitz radius bound.
#[derive(Clone,Debug)]
pub struct BoundaryCell {
    pub start: GridPoint,
    pub step: u32,
    pub bounds: V,
    pub center: [f64;3],
    pub center_value: I,
    pub radius: f64,
    pub value: I,
    /// For a cell not proved uniform, this mesh vertex bounds the distance
    /// from every point of the cell to the output mesh.
    pub mesh_vertex: Option<usize>,
}

#[derive(Clone,Debug)]
pub struct BoundaryPoint {pub point:[f64;3],pub value:I}

/// Both signs lie in the cell containing the associated triangles. Continuity
/// gives an actual material boundary in that cell, even for a non-distance field.
#[derive(Clone,Debug)]
pub struct BoundaryCrossing {
    pub cell: usize,
    pub triangles: std::ops::Range<usize>,
    pub inside: BoundaryPoint,
    pub outside: BoundaryPoint,
}

/// A checked single closed shell with a conservative two-sided distance bound
/// to this snapshot's material boundary. This does not prove isotopy, absence of
/// features below that distance, geometric self-intersection freedom, source-solve accuracy or STL accuracy
/// after coordinate quantization. Geometry and its certificate cannot be mutated.
#[derive(Clone,Debug)]
pub struct FieldBoundary {
    vertices: Vec<[f64;3]>,
    triangles: Vec<[usize;3]>,
    shell: ClosedShell,
    domain: V,
    divisions: u32,
    cells: Vec<BoundaryCell>,
    crossings: Vec<BoundaryCrossing>,
    spatial_error: f64,
}
impl FieldBoundary {
    pub fn vertices(&self) -> &[[f64;3]] { &self.vertices }
    pub fn triangles(&self) -> &[[usize;3]] { &self.triangles }
    pub fn shell(&self) -> &ClosedShell { &self.shell }
    pub fn domain(&self) -> V { self.domain }
    pub fn divisions(&self) -> u32 { self.divisions }
    pub fn cells(&self) -> &[BoundaryCell] { &self.cells }
    pub fn crossings(&self) -> &[BoundaryCrossing] { &self.crossings }
    pub fn spatial_error_bound(&self) -> f64 { self.spatial_error }
}

fn distance_to_box(p: [f64;3],b: V) -> Result<f64,Error> {
    let mut farthest = [I::ZERO;3];
    for k in 0..3 {
        let d = b[k].sub(I::point(p[k])?)?.bounds();
        farthest[k] = I::point(d[0].abs().max(d[1].abs()))?;
    }
    Ok(norm(farthest)?.bounds()[1])
}
fn diameter(b: V) -> Result<f64,Error> {
    let mut width = [I::ZERO;3];
    for k in 0..3 { let [lo,hi] = b[k].bounds(); width[k] = I::point(hi)?.sub(I::point(lo)?)?; }
    Ok(norm(width)?.bounds()[1])
}

struct Grid {domain:V,n:u32}
impl Grid {
    fn point(&self,index: GridPoint) -> [f64;3] {
        std::array::from_fn(|k| {
            let [lo,hi] = self.domain[k].bounds();
            let t = f64::from(index[k])/f64::from(self.n);
            (lo*(1.-t)+hi*t).clamp(lo,hi)
        })
    }
    fn bounds(&self,start: GridPoint,step: u32) -> Result<V,BoundaryError> {
        let lo = self.point(start); let hi = self.point(start.map(|i| i+step));
        let mut b = [I::ZERO;3];
        for k in 0..3 {
            if lo[k] >= hi[k] { return Err(BoundaryError::ResolutionLimit); }
            b[k] = I::new(lo[k],hi[k])?;
        }
        Ok(b)
    }
}

struct Mesh {
    vertices: Vec<[f64;3]>,
    triangles: Vec<[usize;3]>,
    edges: BTreeMap<[GridPoint;2],usize>,
}
impl Mesh {
    fn edge(&mut self,a: GridPoint,b: GridPoint,grid: &Grid) -> usize {
        let key = if a < b { [a,b] } else { [b,a] };
        *self.edges.entry(key).or_insert_with(|| {
            let a = grid.point(key[0]); let b = grid.point(key[1]);
            let i = self.vertices.len();
            self.vertices.push(std::array::from_fn(|k| (a[k]*0.5+b[k]*0.5).clamp(a[k].min(b[k]),a[k].max(b[k]))));
            i
        })
    }
    fn triangle(&mut self,mut t: [usize;3],inside: [f64;3],outside: [f64;3]) -> Result<(),BoundaryError> {
        let [a,b,c] = t.map(|i| self.vertices[i]);
        let mut u = [I::ZERO;3]; let mut v = u; let mut toward = u;
        for k in 0..3 {
            u[k] = I::point(b[k])?.sub(I::point(a[k])?)?;
            v[k] = I::point(c[k])?.sub(I::point(a[k])?)?;
            toward[k] = I::point(outside[k])?.sub(I::point(inside[k])?)?;
        }
        let mut dot = I::ZERO;
        for k in 0..3 {
            let j = (k+1)%3; let l = (k+2)%3;
            dot = dot.add(u[j].mul(v[l])?.sub(u[l].mul(v[j])?)?.mul(toward[k])?)?;
        }
        if dot.bounds()[1] < 0. { t.swap(1,2); }
        else if dot.bounds()[0] <= 0. { return Err(BoundaryError::ResolutionLimit); }
        self.triangles.push(t);
        Ok(())
    }
}

impl MaterialEvaluator {
    pub fn boundary(&mut self,options: BoundaryOptions) -> Result<FieldBoundary,BoundaryError> {
        self.boundary_with_observer(options,|_,_| {})
    }

    /// Observe completed cell counts in each stage. The callback supplies no
    /// geometry, field values, pruning decisions or acceptance criteria.
    pub fn boundary_with_observer(&mut self,options: BoundaryOptions,mut observe: impl FnMut(BoundaryStage,usize))
        -> Result<FieldBoundary,BoundaryError> {
        let tol = options.spatial_tolerance;
        if !tol.is_finite() || tol <= 0. || options.max_depth > 24 || options.max_cells == 0
            || !options.sweep.value_tolerance.is_finite() || options.sweep.value_tolerance <= 0.
            || options.sweep.max_evaluations < 4 { return Err(BoundaryError::InvalidOptions); }
        // the caller's box where one is given, else the material's own support as before
        let mut domain = match options.domain {
            Some(domain) => domain,
            None => self.support_bounds()?.ok_or(BoundaryError::NoFiniteSupport)?,
        };
        for p in &mut domain { *p = p.add(I::new(-tol,tol)?)?; }
        let target = I::point(tol)?.div(I::point(4.)?)?.bounds()[0];
        let span = diameter(domain)?;
        let mut depth = 0_u8;
        while (span/f64::from(1_u32<<depth)).next_up() > target {
            if depth >= options.max_depth { return Err(BoundaryError::ResolutionLimit); }
            depth += 1;
        }
        let grid = Grid {domain,n:1<<depth};
        let mut cells = vec![];
        let mut pending = vec![([0;3],grid.n)];
        let mut visited = 0;
        while let Some((start,step)) = pending.pop() {
            if visited >= options.max_cells { return Err(BoundaryError::CellBudget {visited}); }
            observe(BoundaryStage::Partition,visited);
            visited += 1;
            let bounds = grid.bounds(start,step)?;
            let center = if step > 1 { grid.point(start.map(|i| i+step/2)) }
                else { bounds.map(|v| { let [a,b] = v.bounds(); a*0.5+b*0.5 }) };
            let radius = distance_to_box(center,bounds)?;
            let center_value = self.bounds_outside(point(center)?,I::new(-radius,radius)?,Options {
                value_tolerance:options.sweep.value_tolerance.max(radius),..options.sweep})?.value;
            let value = center_value.add(I::new(-radius,radius)?)?;
            if !value.contains(0.) || step == 1 {
                cells.push(BoundaryCell {start,step,bounds,center,center_value,radius,value,mesh_vertex:None});
            } else {
                let half = step/2;
                for corner in (0..8).rev() {
                    pending.push((std::array::from_fn(|k| start[k]+if corner&(1<<k) != 0 {half} else {0}),half));
                }
            }
        }
        observe(BoundaryStage::Partition,visited);

        let mut mesh = Mesh {vertices:vec![],triangles:vec![],edges:BTreeMap::new()};
        let mut points = BTreeMap::<GridPoint,BoundaryPoint>::new();
        let mut representatives = BTreeMap::new();
        let mut crossings = vec![];
        let mut error = 0_f64;
        let mut processed = 0;
        for (cell_id,cell) in cells.iter().enumerate().filter(|(_,c)| c.value.contains(0.)) {
            observe(BoundaryStage::Triangulation,processed); processed += 1;
            let indices: [GridPoint;8] = std::array::from_fn(|c|
                std::array::from_fn(|k| cell.start[k]+u32::from(c&(1<<k) != 0)));
            for index in indices {
                if let std::collections::btree_map::Entry::Vacant(entry) = points.entry(index) {
                    let p = grid.point(index);
                    let mut tolerance = target.max(options.sweep.value_tolerance);
                    let value = loop {
                        let value = self.bounds_outside(point(p)?,I::ZERO,Options {value_tolerance:tolerance,..options.sweep})?.value;
                        if !value.contains(0.) { break value; }
                        if tolerance <= options.sweep.value_tolerance {
                            return Err(BoundaryError::AmbiguousPoint {point:p,value});
                        }
                        tolerance = (tolerance*0.125).max(options.sweep.value_tolerance);
                    };
                    entry.insert(BoundaryPoint {point:p,value});
                }
            }
            let negative: [bool;8] = indices.map(|i| points[&i].value.bounds()[1] < 0.);
            if negative.iter().all(|v| *v == negative[0]) { continue; }
            let inside = points[&indices[negative.iter().position(|v| *v).unwrap()]].clone();
            let outside = points[&indices[negative.iter().position(|v| !*v).unwrap()]].clone();
            let first = mesh.triangles.len();
            // Six conforming tetrahedra along the cube's 000--111 diagonal.
            for [a,b,_] in [[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]] {
                let tet = [0,1<<a,(1<<a)|(1<<b),7];
                let ins: Vec<_> = tet.into_iter().filter(|i| negative[*i]).collect();
                let outs: Vec<_> = tet.into_iter().filter(|i| !negative[*i]).collect();
                if ins.is_empty() || outs.is_empty() { continue; }
                let pin = points[&indices[ins[0]]].point; let pout = points[&indices[outs[0]]].point;
                if ins.len() == 2 {
                    let [a,b,c,d] = [
                        mesh.edge(indices[ins[0]],indices[outs[0]],&grid),
                        mesh.edge(indices[ins[1]],indices[outs[0]],&grid),
                        mesh.edge(indices[ins[1]],indices[outs[1]],&grid),
                        mesh.edge(indices[ins[0]],indices[outs[1]],&grid)];
                    mesh.triangle([a,b,c],pin,pout)?; mesh.triangle([a,c,d],pin,pout)?;
                } else {
                    let (single,other) = if ins.len() == 1 { (ins[0],outs) } else { (outs[0],ins) };
                    let t = std::array::from_fn(|k| mesh.edge(indices[single],indices[other[k]],&grid));
                    mesh.triangle(t,pin,pout)?;
                }
            }
            error = error.max(diameter(cell.bounds)?);
            representatives.insert(cell.start,mesh.triangles[first][0]);
            crossings.push(BoundaryCrossing {cell:cell_id,triangles:first..mesh.triangles.len(),inside,outside});
        }
        observe(BoundaryStage::Triangulation,processed);
        if mesh.triangles.is_empty() { return Err(BoundaryError::NoBoundary); }
        let mut witness_ids: Vec<_> = representatives.values().copied().collect();
        witness_ids.sort_unstable(); witness_ids.dedup();
        let witness_index = WitnessIndex::new(&mut witness_ids,&mesh.vertices).unwrap();
        // All possible true boundary points lie in the uncertain leaf cells.
        // Give each such cell an actual mesh point within the requested distance.
        // A small hidden component away from existing triangles cannot disappear.
        let mut certified = 0;
        for cell in cells.iter_mut().filter(|c| c.value.contains(0.)) {
            observe(BoundaryStage::Certification,certified);
            let mut best = None;
            for dx in -1_i64..=1 { for dy in -1_i64..=1 { for dz in -1_i64..=1 {
                let q: [i64;3] = std::array::from_fn(|k| i64::from(cell.start[k])+[dx,dy,dz][k]);
                if q.iter().any(|v| *v < 0 || *v >= i64::from(grid.n)) { continue; }
                if let Some(&vertex) = representatives.get(&q.map(|v| v as u32)) {
                    let distance = distance_to_box(mesh.vertices[vertex],cell.bounds)?;
                    if best.is_none_or(|(_,d)| distance < d) { best = Some((vertex,distance)); }
                }
            } } }
            // A residual is not distance: a conservative possible-boundary cell
            // need not touch a meshed cell. The neighborhood is only a fast
            // search. Before refusing, search all representatives with interval
            // distance pruning; none of the possible-boundary cells is omitted.
            if best.is_none_or(|(_,d)| d > tol) {
                witness_index.search(cell.bounds,&mesh.vertices,tol,&mut best)?;
            }
            let Some((vertex,distance)) = best.filter(|(_,d)| *d <= tol) else {
                return Err(BoundaryError::UnresolvedCell {cell:cell.bounds});
            };
            cell.mesh_vertex = Some(vertex); error = error.max(distance);
            certified += 1;
        }
        observe(BoundaryStage::Certification,certified);
        if error > tol { return Err(BoundaryError::ResolutionLimit); }
        let shell = ClosedShell::from_triangles(mesh.vertices.len(),&mesh.triangles).map_err(|error|
            BoundaryError::Topology {error,components:components::components(&mesh.vertices,&mesh.triangles)})?;
        Ok(FieldBoundary {vertices:mesh.vertices,triangles:mesh.triangles,shell,domain,divisions:grid.n,
            cells,crossings,spatial_error:error})
    }
}
