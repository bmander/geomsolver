//! What the labels leave of the seed sheets: the triangles all of whose
//! vertices the field kept or moved, wound outward, and the boundary loops
//! of that mesh, which the closure audit reads. (Bisecting mixed edges to
//! rim points and building creases comes with milestone 4.)
use super::certify::triangle_normal;
use super::project::{Label,Labelled};
use crate::solid::SweepPatch;

type V3 = [f64;3];

/// An indexed mesh with, per triangle, the sheet it came from.
#[derive(Clone,Debug,Default)]
pub struct KeptMesh {
    pub vertices: Vec<V3>,
    pub triangles: Vec<[u32;3]>,
    pub sheet: Vec<u32>,
}

/// The kept triangles of every sheet, each wound so that its normal agrees
/// with the direction its vertices were judged along (outward). Vertices are
/// the judged positions; sheets keep separate vertices here (welding is the
/// stitch's).
pub fn kept_triangles(sheets: &[SweepPatch],labelled: &[Labelled]) -> KeptMesh {
    let mut out = KeptMesh::default();
    for (s,(sheet,l)) in sheets.iter().zip(labelled).enumerate() {
        let base = out.vertices.len() as u32;
        out.vertices.extend_from_slice(&l.points);
        for t in &sheet.triangles {
            if !t.iter().all(|&v| matches!(l.labels[v as usize],Label::Kept | Label::Moved)) { continue; }
            let [a,b,c] = t.map(|v| l.points[v as usize]);
            let Some(n) = triangle_normal(a,b,c) else { continue };
            let outward: V3 = std::array::from_fn(|k| t.iter().map(|&v| l.directions[v as usize][k]).sum());
            let agree = n[0]*outward[0]+n[1]*outward[1]+n[2]*outward[2] >= 0.;
            out.triangles.push(if agree { [base+t[0],base+t[1],base+t[2]] } else { [base+t[0],base+t[2],base+t[1]] });
            out.sheet.push(s as u32);
        }
    }
    out
}

/// The boundary of an indexed mesh: its directed edges used by exactly one
/// triangle, chained into loops. An edge used more than twice is returned
/// as a loop of two, so the audit sees it.
pub fn boundary_loops(triangles: &[[u32;3]]) -> Vec<Vec<u32>> {
    let mut uses: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
    for t in triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_default() += 1; } }
    // a boundary edge keeps its triangle's direction
    let mut next: std::collections::BTreeMap<u32,Vec<u32>> = Default::default();
    for t in triangles { for k in 0..3 {
        let (a,b) = (t[k],t[(k+1)%3]);
        if uses[&(a.min(b),a.max(b))] == 1 { next.entry(a).or_default().push(b); }
    } }
    let mut loops = Vec::new();
    while let Some((&start,_)) = next.iter().next() {
        let mut walk = vec![start];
        let mut at = start;
        loop {
            let Some(list) = next.get_mut(&at) else { break };
            let Some(to) = list.pop() else { next.remove(&at); break };
            if list.is_empty() { next.remove(&at); }
            if to == start { break; }
            walk.push(to); at = to;
        }
        loops.push(walk);
    }
    loops
}
