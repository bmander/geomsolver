//! The volume a refinement fills, as an analysis reads it (#510 P6): the cells inside as linear
//! tetrahedra over the surface's vertices, and as quadratic ones (TET10, `tetra::Tet10`) whose
//! edge nodes on the boundary are placed on it.
//!
//! - **The cells** are the tetrahedra whose orthocentre the domain holds. Their boundary is the
//!   restricted facets, so the volume's boundary is the surface mesh, to the vertex.
//! - **An edge node** is its edge's middle, except on the boundary: an edge on a feature curve —
//!   two protecting balls consecutive on it — has its node at the curve's arc-length middle
//!   between them, and any other edge of a boundary facet has it where the domain's boundary
//!   crosses the line through its middle along the facets' mean normal, within a quarter of the
//!   edge's length (the crossing found as a facet's surface centre is). An edge whose line meets
//!   no boundary there stays straight, and is counted.
//! - **No element is inverted.** A quadratic element whose Jacobian's least Bernstein
//!   coefficient is under `LEAST_JACOBIAN` of its straight element's (`Tet10::jacobian_ratio`)
//!   has its boundary edges straightened again, and its neighbours are judged again, until every
//!   element passes; a straight element is its linear tetrahedron, positive.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use super::cells::CellReport;
use super::protect::Curves;
use crate::delaunay::tetra::{self,Tet10,EDGES};
use std::collections::BTreeMap;

/// The least `Tet10::jacobian_ratio` a curved element keeps: a lower bound on its Jacobian
/// everywhere, as a fraction of its straight element's.
pub const LEAST_JACOBIAN: f64 = 0.2;

/// A volume mesh: linear tetrahedra over `vertices`, and the quadratic elements over `nodes`.
#[derive(Clone,Debug,Default)]
pub struct Volume {
    /// The corner nodes, the surface's vertices among them.
    pub vertices: Vec<P>,
    /// Whether each vertex is a protecting ball's centre: a point of a feature curve.
    pub protected: Vec<bool>,
    /// Every cell, positively oriented (`tetra::volume6 > 0`).
    pub tets: Vec<[u32;4]>,
    /// The boundary, triangles over `vertices` facing out: the cells' faces on no other cell.
    pub boundary: Vec<[u32;3]>,
    /// The quadratic elements' nodes: `vertices` first, then one an edge.
    pub nodes: Vec<P>,
    /// One TET10 a cell, in `tets`' order: its corners, then its edge nodes in `tetra::EDGES`'
    /// order.
    pub elements: Vec<[u32;10]>,
    pub report: VolumeReport,
}

/// What the volume is worth and what made it.
#[derive(Clone,Debug,Default)]
pub struct VolumeReport {
    pub surface: Report,
    pub cells: CellReport,
    /// The worst circumradius over shortest edge, and the least dihedral angle in degrees, over
    /// every cell.
    pub worst_radius_edge: f64,
    pub least_dihedral: f64,
    /// Cells left breaking the criteria (where refinement could not reach them: see `CellReport`).
    pub bad_cells: usize,
    /// Boundary edges whose node is on the surface, on a feature curve, and left straight.
    pub curved: usize,
    pub on_curves: usize,
    pub straight: usize,
    /// Edge nodes put back at their edge's middle so that no element is inverted.
    pub straightened: usize,
    /// The least `Tet10::jacobian_ratio` over every element: 1 when every one is straight.
    pub least_jacobian: f64,
}

impl Volume {
    /// The cell `k`'s corners.
    pub fn tet(&self,k: usize) -> [P;4] { self.tets[k].map(|v| self.vertices[v as usize]) }
    /// The quadratic element `k`.
    pub fn element(&self,k: usize) -> Tet10 { Tet10 {nodes:self.elements[k].map(|v| self.nodes[v as usize])} }
    /// The volume of the linear cells.
    pub fn linear_volume(&self) -> f64 { (0..self.tets.len()).map(|k| tetra::volume6(self.tet(k))).sum::<f64>()/6. }
    /// The volume of the quadratic elements, exactly.
    pub fn quadratic_volume(&self) -> f64 { (0..self.elements.len()).map(|k| self.element(k).volume()).sum() }
}

/// Where each feature curve's protecting balls stand, in arc-length order: the curve's
/// `(arc length, ball)`s.
fn stations(r: &Refiner,curves: &Curves) -> Vec<Vec<(f64,usize)>> {
    let mut out = vec![Vec::new();curves.lengths.len()];
    for (b,places) in r.owners.iter().enumerate() {
        for place in places { out[place.curve].push((place.s,b)); }
    }
    for list in &mut out { list.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1))); }
    out
}

/// The point midway along a feature curve between balls `a` and `b`, when both stand on one
/// curve next to each other.
fn arc_middle(curves: &Curves,stations: &[Vec<(f64,usize)>],a: usize,b: usize) -> Option<P> {
    for (k,list) in stations.iter().enumerate() {
        let (Some(i),Some(j)) = (list.iter().position(|x| x.1 == a),list.iter().position(|x| x.1 == b)) else { continue };
        let n = list.len();
        let (lo,hi) = (i.min(j),i.max(j));
        let (sa,sb) = (list[lo].0,list[hi].0);
        if hi == lo+1 { return Some(curves.at(k,0.5*(sa+sb))); }
        if curves.closed[k] && lo == 0 && hi == n-1 {
            let length = curves.lengths[k];
            let s = 0.5*(sb+sa+length);
            return Some(curves.at(k,if s >= length { s-length } else { s }));
        }
    }
    None
}

/// The volume the refiner holds, its edge nodes placed against `curves` (the refinement's).
pub(super) fn extract(r: &mut Refiner,curves: &[Vec<P>]) -> Result<Volume,String> {
    let cells = r.cells.clone().ok_or("no criteria for the cells")?;
    let all: Vec<u32> = r.tri.tets().collect();
    let mut inside = Vec::new();
    for t in all {
        if r.tri.touches_enclosure(t) {
            if r.tet_side(t) < 0 { return Err("a cell inside the domain reaches the enclosing tetrahedron".into()); }
            continue;
        }
        if r.tet_side(t) < 0 { inside.push(t); }
    }
    if inside.is_empty() { return Err("the refinement found no cell inside the domain".into()); }
    let is_inside = |t: u32,inside: &[u32]| inside.binary_search(&t).is_ok();
    // Vertices renumbered in the order the cells meet them (looked up, never iterated).
    let mut index: HashMap<u32,u32> = HashMap::new();
    let mut vertices: Vec<P> = Vec::new();
    let mut original: Vec<u32> = Vec::new();
    let mut tets = Vec::with_capacity(inside.len());
    let mut boundary = Vec::new();
    for &t in &inside {
        let tet = r.tri.tet(t);
        tets.push(tet.v.map(|v| *index.entry(v).or_insert_with(|| {
            vertices.push(r.tri.points()[v as usize].p);
            original.push(v);
            (vertices.len()-1) as u32
        })));
        for i in 0..4 {
            if tet.n[i] != NONE && is_inside(tet.n[i],&inside) { continue; }
            // Outward: away from the cell's own opposite corner.
            let mut f = facet_vertices(tet.v,i);
            let p = |v: u32| r.tri.points()[v as usize].p;
            if orient(p(f[0]),p(f[1]),p(f[2]),p(tet.v[i])) > 0 { f.swap(1,2); }
            boundary.push(f);
        }
    }
    let boundary: Vec<[u32;3]> = boundary.iter().map(|f| f.map(|v| index[&v])).collect();
    let protected: Vec<bool> = original.iter().map(|v| r.ball_of.contains_key(v)).collect();
    let mut report = VolumeReport {surface:r.report.clone(),cells:r.cell_report.clone(),least_dihedral:180.,
        least_jacobian:1.,..VolumeReport::default()};
    for &q in &tets {
        let q = q.map(|v| vertices[v as usize]);
        if !(tetra::volume6(q) > 0.) { return Err("a cell is flat in floating point".into()); }
        let (rho,least) = (tetra::radius_edge(q),tetra::least_dihedral(q));
        report.worst_radius_edge = report.worst_radius_edge.max(rho);
        report.least_dihedral = report.least_dihedral.min(least);
        let size = tetra::circumsphere(q).map_or(f64::INFINITY,|c| c.1);
        if rho > cells.radius_edge || size > cells.size || least < cells.dihedral { report.bad_cells += 1; }
    }
    // Every edge once, in order, and the boundary's outward normal summed along it.
    let mut edges: BTreeMap<(u32,u32),u32> = BTreeMap::new();
    for q in &tets { for &(i,j) in &EDGES { edges.insert((q[i].min(q[j]),q[i].max(q[j])),0); } }
    let mut normals: BTreeMap<(u32,u32),P> = BTreeMap::new();
    for f in &boundary {
        let [a,b,c] = f.map(|v| vertices[v as usize]);
        let n = crate::space::cross(sub(b,a),sub(c,a));
        for j in 0..3 {
            let (x,y) = (f[j],f[(j+1)%3]);
            let e = normals.entry((x.min(y),x.max(y))).or_insert([0.;3]);
            for k in 0..3 { e[k] += n[k]; }
        }
    }
    let curves = Curves::new(curves);
    let stations = stations(r,&curves);
    let mut nodes = vertices.clone();
    let mut middles = Vec::with_capacity(edges.len());
    let mut curved = Vec::with_capacity(edges.len());
    for (&(a,b),slot) in edges.iter_mut() {
        let (pa,pb) = (vertices[a as usize],vertices[b as usize]);
        let middle: P = std::array::from_fn(|k| 0.5*(pa[k]+pb[k]));
        *slot = nodes.len() as u32;
        middles.push(middle);
        let Some(&n) = normals.get(&(a,b)) else { nodes.push(middle); curved.push(false); continue };
        let balls = (r.ball_of.get(&original[a as usize]).copied(),r.ball_of.get(&original[b as usize]).copied());
        let on_curve = match balls { (Some(x),Some(y)) => arc_middle(&curves,&stations,x,y), _ => None };
        let node = match on_curve {
            Some(p) => { report.on_curves += 1; Some(p) }
            None => {
                let found = surface_point(r,middle,n,dist2(pa,pb).sqrt());
                if found.is_some() { report.curved += 1; } else { report.straight += 1; }
                found
            }
        };
        nodes.push(node.unwrap_or(middle));
        curved.push(node.is_some_and(|p| p != middle));
    }
    let first = vertices.len();
    let elements: Vec<[u32;10]> = tets.iter().map(|q| {
        let mut e = [0u32;10];
        e[..4].copy_from_slice(q);
        for (k,&(i,j)) in EDGES.iter().enumerate() { e[4+k] = edges[&(q[i].min(q[j]),q[i].max(q[j]))]; }
        e
    }).collect();
    // Straighten what an inverted element needs, until none is.
    loop {
        let mut changed = false;
        for e in &elements {
            if !e[4..].iter().any(|&m| curved[m as usize-first]) { continue; }
            let element = Tet10 {nodes:e.map(|v| nodes[v as usize])};
            if element.jacobian_ratio() >= LEAST_JACOBIAN { continue; }
            for &m in &e[4..] {
                let m = m as usize;
                if curved[m-first] { curved[m-first] = false; nodes[m] = middles[m-first]; report.straightened += 1; changed = true; }
            }
        }
        if !changed { break; }
    }
    for e in &elements {
        if e[4..].iter().any(|&m| curved[m as usize-first]) {
            report.least_jacobian = report.least_jacobian.min(Tet10 {nodes:e.map(|v| nodes[v as usize])}.jacobian_ratio());
        }
    }
    Ok(Volume {vertices,protected,tets,boundary,nodes,elements,report})
}

/// Where the domain's boundary crosses the line through `middle` along `normal`, within a
/// quarter of `length` of it, toward the side `middle` is not on: none where it does not.
fn surface_point(r: &mut Refiner,middle: P,normal: P,length: f64) -> Option<P> {
    let l = dot(normal,normal).sqrt();
    if !(l > 0.) || !(length > 0.) { return None; }
    let side = r.side(middle);
    // Inside, the boundary is outward; outside, inward.
    let reach = 0.25*length*side as f64/l;
    let end: P = std::array::from_fn(|k| middle[k]-reach*normal[k]);
    if r.side(end) == side { return None; }
    Some(r.crossing(middle,side,end))
}
