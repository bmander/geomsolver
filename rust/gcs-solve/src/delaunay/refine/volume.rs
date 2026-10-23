//! The volume a refinement fills, as an analysis reads it (#510 P6): the cells inside as linear
//! tetrahedra over the surface's vertices, and as quadratic ones (TET10, `tetra::Tet10`) whose
//! edge nodes on the boundary are placed on it.
//!
//! - **The cells** are the tetrahedra whose orthocentre the domain holds. Their boundary is the
//!   restricted facets, so the volume's boundary is the surface mesh, to the vertex.
//! - **An edge node** is its edge's middle, except on the boundary: an edge on a feature curve —
//!   two protecting balls consecutive on it — has its node at the curve's arc-length middle
//!   between them (`protect`'s halfway points), and any other edge of a boundary facet has it where the domain's boundary
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
use crate::delaunay::tetra::{self,Tet10,EDGES};
use crate::space::{add,cross,norm,scale};
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
    /// Slivers lying on the boundary taken out of the volume (`peel`).
    pub peeled: usize,
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

/// **Slivers lying on the boundary, peeled** (#510): a cell two or three of whose faces are on the
/// boundary has every corner on the surface; where it is a sliver (its least dihedral angle under
/// the cells' bound) and thinner than the surface's `facet_distance` — its volume over its largest
/// face, three times — it is taken out, and its other faces are the boundary instead: the surface
/// moves by no more than its own criterion allows it to stand off, and the volume loses an element
/// that is nearly flat, whose stiffness is nearly singular.  Refinement cannot reach such a cell
/// when its corners are protecting balls' centres or its orthocentre lies in one.  A peel that would
/// leave an edge of the boundary on other than two faces, or a corner on no cell, is not made.
/// Cells in triangulation order, faces as `Refiner::extract` orients them, until none is peeled.
fn peel(r: &mut Refiner,inside: Vec<u32>,facets: &[([u32;3],u32,usize)],cells: &Cells) -> (Vec<u32>,Vec<[u32;3]>,usize) {
    let key = |f: [u32;3]| { let mut k = f; k.sort_unstable(); k };
    let mut cell: std::collections::BTreeSet<u32> = inside.iter().copied().collect();
    let mut faces: BTreeMap<[u32;3],[u32;3]> = facets.iter().map(|(f,_,_)| (key(*f),*f)).collect();
    let mut edges: BTreeMap<(u32,u32),usize> = BTreeMap::new();
    let edge = |a: u32,b: u32| (a.min(b),a.max(b));
    for f in faces.values() { for k in 0..3 { *edges.entry(edge(f[k],f[(k+1)%3])).or_default() += 1; } }
    let mut uses: BTreeMap<u32,usize> = BTreeMap::new();
    for &t in &cell { for v in r.tri.tet(t).v { *uses.entry(v).or_default() += 1; } }
    let thin = r.criteria.facet_distance;
    let mut peeled = 0;
    loop {
        let mut changed = false;
        let order: Vec<u32> = cell.iter().copied().collect();
        for t in order {
            let tet = r.tri.tet(t);
            let q = tet.v.map(|v| r.tri.points()[v as usize].p);
            let on: Vec<usize> = (0..4).filter(|&i| faces.contains_key(&key(facet_vertices(tet.v,i)))).collect();
            if on.len() < 2 || on.len() > 3 || tetra::least_dihedral(q) >= cells.dihedral { continue; }
            let area = (0..4).map(|i| { let f = facet_vertices(tet.v,i).map(|v| r.tri.points()[v as usize].p);
                norm(cross(sub(f[1],f[0]),sub(f[2],f[0]))) }).fold(0.,f64::max);
            if !(tetra::volume6(q).abs() <= 2.*thin*area) || tet.v.iter().any(|v| uses[v] == 1) { continue; }
            // the faces it would leave on the boundary, each oriented out of its neighbour
            let mut out = Vec::new();
            for i in (0..4).filter(|i| !on.contains(i)) {
                let Some((n,j)) = r.tri.mirror(t,i) else { break };
                if !cell.contains(&n) { break; }
                let nt = r.tri.tet(n);
                let mut f = facet_vertices(nt.v,j);
                let p = |v: u32| r.tri.points()[v as usize].p;
                if orient(p(f[0]),p(f[1]),p(f[2]),p(nt.v[j])) > 0 { f.swap(1,2); }
                out.push(f);
            }
            if out.len() + on.len() != 4 { continue; }
            // every edge of the boundary still on two faces: only its own six edges change
            let sides = |f: [u32;3]| [0,1,2].map(|k| edge(f[k],f[(k+1)%3]));
            let after: Vec<((u32,u32),i64)> = EDGES.iter().map(|&(a,b)| {
                let e = edge(tet.v[a],tet.v[b]);
                let gone = on.iter().filter(|&&i| sides(facet_vertices(tet.v,i)).contains(&e)).count();
                let made = out.iter().filter(|f| sides(**f).contains(&e)).count();
                (e,edges.get(&e).copied().unwrap_or(0) as i64-gone as i64+made as i64)
            }).collect();
            if after.iter().any(|&(_,c)| c != 0 && c != 2) { continue; }
            for &i in &on { faces.remove(&key(facet_vertices(tet.v,i))); }
            for f in out { faces.insert(key(f),f); }
            for (e,c) in after {
                if c == 0 { edges.remove(&e); } else { edges.insert(e,c as usize); }
            }
            for v in tet.v { *uses.get_mut(&v).unwrap() -= 1; }
            cell.remove(&t);
            peeled += 1;
            changed = true;
        }
        if !changed { break; }
    }
    let kept: Vec<u32> = inside.into_iter().filter(|t| cell.contains(t)).collect();
    // the restricted facets in their order, then the faces a peel left, in order
    let mut boundary: Vec<[u32;3]> = Vec::with_capacity(faces.len());
    let mut left = faces;
    for (f,_,_) in facets { if left.remove(&key(*f)).is_some() { boundary.push(*f); } }
    boundary.extend(left.into_values());
    (kept,boundary,peeled)
}

/// The volume the refiner holds, its boundary the restricted `facets` (`Refiner::extract`).
pub(super) fn extract(r: &mut Refiner,facets: &[([u32;3],u32,usize)]) -> Result<Volume,String> {
    let cells = r.filling.as_ref().ok_or("no criteria for the cells")?.cells.clone();
    let all: Vec<u32> = r.tri.tets().collect();
    let mut inside = Vec::new();
    for t in all {
        if !r.inside(t) {
            if r.tri.touches_enclosure(t) && r.tet_side(t) < 0 { return Err("a cell inside the domain reaches the enclosing tetrahedron".into()); }
            continue;
        }
        inside.push(t);
    }
    if inside.is_empty() { return Err("the refinement found no cell inside the domain".into()); }
    let (inside,facets,peeled) = peel(r,inside,facets,&cells);
    // Vertices renumbered in the order the cells meet them (looked up, never iterated), each
    // with the protecting ball it is, if any.
    let mut index: HashMap<u32,u32> = HashMap::new();
    let mut vertices: Vec<P> = Vec::new();
    let mut ball: Vec<Option<usize>> = Vec::new();
    let tets: Vec<[u32;4]> = inside.iter().map(|&t| r.tri.tet(t).v.map(|v| *index.entry(v).or_insert_with(|| {
        vertices.push(r.tri.points()[v as usize].p);
        ball.push(r.ball_of.get(&v).copied());
        (vertices.len()-1) as u32
    }))).collect();
    let boundary: Vec<[u32;3]> = facets.iter().map(|f| f.map(|v| index[&v])).collect();
    let protected: Vec<bool> = ball.iter().map(Option::is_some).collect();
    let mut report = VolumeReport {surface:r.report.clone(),cells:r.filling.as_ref().map(|f| f.report.clone()).unwrap_or_default(),
        least_dihedral:180.,least_jacobian:1.,peeled,..VolumeReport::default()};
    for &q in &tets {
        let q = q.map(|v| vertices[v as usize]);
        if !(tetra::volume6(q) > 0.) { return Err("a cell is flat in floating point".into()); }
        report.worst_radius_edge = report.worst_radius_edge.max(tetra::radius_edge(q));
        report.least_dihedral = report.least_dihedral.min(tetra::least_dihedral(q));
        if cells.badness(q).0 > 1. { report.bad_cells += 1; }
    }
    // Every edge once, in order, and the boundary's outward normal summed along it.
    let mut edges: BTreeMap<(u32,u32),u32> = BTreeMap::new();
    for q in &tets { for &(i,j) in &EDGES { edges.insert((q[i].min(q[j]),q[i].max(q[j])),0); } }
    let mut normals: BTreeMap<(u32,u32),P> = BTreeMap::new();
    for f in &boundary {
        let [a,b,c] = f.map(|v| vertices[v as usize]);
        let n = cross(sub(b,a),sub(c,a));
        for j in 0..3 {
            let (x,y) = (f[j],f[(j+1)%3]);
            let e = normals.entry((x.min(y),x.max(y))).or_insert([0.;3]);
            for k in 0..3 { e[k] += n[k]; }
        }
    }
    let mut nodes = vertices.clone();
    for (&(a,b),slot) in edges.iter_mut() {
        let (pa,pb) = (vertices[a as usize],vertices[b as usize]);
        let middle = scale(add(pa,pb),0.5);
        *slot = nodes.len() as u32;
        let Some(&n) = normals.get(&(a,b)) else { nodes.push(middle); continue };
        let on_curve = match (ball[a as usize],ball[b as usize]) {
            (Some(x),Some(y)) => r.halfway[x].iter().find(|h| h.0 == y).map(|h| h.1),
            _ => None,
        };
        let node = match on_curve {
            Some(p) => { report.on_curves += 1; Some(p) }
            None => {
                let found = surface_point(r,middle,n,dist2(pa,pb).sqrt());
                if found.is_some() { report.curved += 1; } else { report.straight += 1; }
                found
            }
        };
        nodes.push(node.unwrap_or(middle));
    }
    let elements: Vec<[u32;10]> = tets.iter().map(|q| {
        let mut e = [0u32;10];
        e[..4].copy_from_slice(q);
        for (k,&(i,j)) in EDGES.iter().enumerate() { e[4+k] = edges[&(q[i].min(q[j]),q[i].max(q[j]))]; }
        e
    }).collect();
    // Straighten what an inverted element needs, until none is: the last pass, changing nothing,
    // has every curved element's final ratio.
    loop {
        let mut changed = false;
        let mut least = 1f64;
        for e in &elements {
            let straight = Tet10::straight(std::array::from_fn(|k| nodes[e[k] as usize]));
            if (4..10).all(|m| nodes[e[m] as usize] == straight.nodes[m]) { continue; }
            let ratio = Tet10 {nodes:e.map(|v| nodes[v as usize])}.jacobian_ratio();
            least = least.min(ratio);
            if ratio >= LEAST_JACOBIAN { continue; }
            for m in 4..10 {
                let node = &mut nodes[e[m] as usize];
                if *node != straight.nodes[m] { *node = straight.nodes[m]; report.straightened += 1; changed = true; }
            }
        }
        if !changed { report.least_jacobian = least; break; }
    }
    Ok(Volume {vertices,protected,tets,boundary,nodes,elements,report})
}

/// Where the domain's boundary crosses the line through `middle` along `normal`, within a
/// quarter of `length` of it, toward the side `middle` is not on: none where it does not.
fn surface_point(r: &mut Refiner,middle: P,normal: P,length: f64) -> Option<P> {
    let l = norm(normal);
    if !(l > 0.) || !(length > 0.) { return None; }
    let side = r.side(middle);
    // Inside, the boundary is outward; outside, inward.
    let reach = 0.25*length*side as f64/l;
    let end: P = std::array::from_fn(|k| middle[k]-reach*normal[k]);
    if r.side(end) == side { return None; }
    Some(r.crossing(middle,side,end))
}
