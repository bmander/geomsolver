//! What a stage's mesh should be able to say about itself. The construction's
//! characteristic failure is a defect made at one stage and reported many
//! stages later: two vertices minted a fraction of a snap apart while a cap is
//! cut are not noticed until the zip cannot pair the loop that walks round the
//! whisker between them, with nine stages in between. So every stage's output
//! is measured here, and the first stage whose mesh is not clean names where
//! the defect was made rather than where it surfaced.
//!
//! What is always wrong: an edge used more than twice, an edge two triangles
//! walk the same way (one of them faces backwards), and a triangle the
//! construction reads as degenerate. What is *not* wrong is an edge used once:
//! a sheet is open until the shell closes, so boundary edges are counted and
//! never flagged.
//!
//! The nearest pair is measured but never counted a defect, and only within
//! the band that matters. Two vertices at exactly one point are *by design*
//! before the weld: every seed contributes its own copy of a shared point, and
//! `weld` is what makes them one. So a pair nearer than the coincidence is
//! about to be welded and says nothing. The band worth seeing is between the
//! coincidence and the snap: too far apart for the weld to join, too near for
//! the short-edge collapse to care, too near to be a feature of the boundary —
//! and so a whisker for a boundary loop to walk round. Even there it may be a
//! genuine thin feature, which the certificate already declines to claim, so
//! this measures and does not refuse.
use super::adjacency::Edges;
use super::trim::KeptMesh;
use crate::space::{Grid,degenerate,distance};

type V3 = [f64;3];

/// The most edges at fault worth naming: enough to point a window at, not enough to bury the line.
const FAULTS: usize = 4;

/// The measurements of one stage's mesh.
#[derive(Clone,Debug,Default,PartialEq)]
pub struct Hygiene {
    pub vertices: usize,
    /// Vertices some triangle uses. The rest are the labelled points a seed
    /// contributed and the clip never kept, and say nothing about the mesh.
    pub used: usize,
    pub triangles: usize,
    /// The two nearest vertices in use within the snap, and how far apart.
    pub closest: Option<(f64,u32,u32)>,
    /// Undirected edges one triangle walks: the mesh's boundary, and no fault.
    pub boundary_edges: usize,
    /// Edges more than two triangles walk.
    pub crowded_edges: usize,
    /// Edges two triangles walk the same way round.
    pub same_way: usize,
    pub degenerate: usize,
    /// Triangles repeating another's three vertices. Always a fault: the second covers nothing
    /// the first does not, and it is what `dedupe` exists to drop.
    pub duplicates: usize,
    /// The first few edges at fault: the two vertices, where the edge's middle stands, and how
    /// many triangles walk it each way round. A count alone says a stage went wrong; this says
    /// where, which is what a window wants to be pointed at.
    pub faults: Vec<((u32,u32),V3,(usize,usize))>,
}

impl Hygiene {
    /// Nothing here is a defect of the mesh itself. An open boundary is not one,
    /// and neither is a near pair: a case that certifies may carry one, so it is
    /// reported whenever a line is printed but never the reason to print.
    pub fn clean(&self) -> bool {
        self.crowded_edges == 0 && self.same_way == 0 && self.degenerate == 0 && self.duplicates == 0
    }

    /// One line, naming only what is amiss beyond the counts.
    pub fn report(&self,snap: f64) -> String {
        let mut out = format!("{} vertices ({} used), {} triangles, {} boundary edges",
            self.vertices,self.used,self.triangles,self.boundary_edges);
        if let Some((d,a,b)) = self.closest { out += &format!("; v{a} and v{b} are {d:.9} apart, inside the snap {snap}"); }
        if self.crowded_edges > 0 { out += &format!("; {} edges used more than twice",self.crowded_edges); }
        if self.same_way > 0 { out += &format!("; {} edges walked twice the same way",self.same_way); }
        if self.degenerate > 0 { out += &format!("; {} degenerate triangles",self.degenerate); }
        if self.duplicates > 0 { out += &format!("; {} triangles repeat another's corners",self.duplicates); }
        for ((a,b),at,(f,r)) in &self.faults {
            out += &format!("\n      at fault: v{a}-v{b}, middle {at:?}, walked {f} one way and {r} the other");
        }
        out
    }
}

/// Measure a mesh against the checks above. `coincidence` is what the weld
/// joins, `snap` the construction's vertex tolerance; the nearest pair reported
/// is the nearest standing strictly between them.
///
/// It is found through a grid at the snap, each vertex asked of the cells about
/// it and filed after, so every pair within the snap is seen once and the walk
/// is linear in the vertices. A pairwise scan would be quadratic, which on the
/// dumbbell's thirty-five thousand triangles would cost more than the
/// construction it is measuring.
pub fn hygiene(mesh: &KeptMesh,coincidence: f64,snap: f64) -> Hygiene {
    let mut out = Hygiene {vertices:mesh.vertices.len(),triangles:mesh.triangles.len(),..Default::default()};
    let mut used = vec![false;mesh.vertices.len()];
    for t in &mesh.triangles { for &v in t { if let Some(u) = used.get_mut(v as usize) { *u = true; } } }
    out.used = used.iter().filter(|u| **u).count();
    // the nearest two in use within the snap
    let mut grid = Grid::new(snap.max(f64::MIN_POSITIVE));
    for (v,p) in mesh.vertices.iter().enumerate() {
        if !used[v] { continue; }
        let mut best: Option<(f64,u32)> = None;
        grid.around(*p,|i| {
            let d = distance(mesh.vertices[i as usize],*p);
            if d > coincidence && d <= snap && best.is_none_or(|(b,j)| d < b || (d == b && i < j)) { best = Some((d,i)); }
        });
        if let Some((d,i)) = best {
            if out.closest.is_none_or(|(b,_,_)| d < b) { out.closest = Some((d,i,v as u32)); }
        }
        grid.insert(*p,v as u32);
    }
    for ((a,b),(f,r)) in Edges::new(&mesh.triangles).counts() {
        if f+r == 1 { out.boundary_edges += 1; }
        let crowded = f+r > 2;
        let folded = f >= 2 || r >= 2;
        if crowded { out.crowded_edges += 1; }
        if folded { out.same_way += 1; }
        if (crowded || folded) && out.faults.len() < FAULTS {
            let (p,q) = (mesh.vertices[a as usize],mesh.vertices[b as usize]);
            out.faults.push(((a,b),std::array::from_fn(|k| 0.5*(p[k]+q[k])),(f,r)));
        }
    }
    // triangles repeating another's three corners, by the same walk `dedupe` drops them with
    let mut keys: Vec<[u32;3]> = mesh.triangles.iter().map(|t| { let mut k = *t; k.sort(); k }).collect();
    keys.sort_unstable();
    out.duplicates = keys.windows(2).filter(|w| w[0] == w[1]).count();
    out.degenerate = mesh.triangles.iter().filter(|t| {
        let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
        degenerate(a,b,c)
    }).count();
    out
}
