//! What stands in one ball of a stage's mesh. The construction passes a mesh
//! through a dozen stages, and chasing a defect means asking the same small
//! question of each of them: what is near this point, and whose is it? Asked
//! at every stage boundary the answer shows a defect forming rather than
//! leaving it to be inferred backwards from the stage that refused over it.
//!
//! A window is keyed by **position**, which is the only handle that survives
//! the pipeline: a vertex is renumbered between stages as triangles are
//! dropped, welded and split, so the same point of the surface is a different
//! index at `Clipped` than at `Split`, and an index carried from one stage's
//! report to another's names something else entirely.
//!
//! The vertices are scanned rather than indexed. One ball is one query, and
//! building a grid over every vertex to answer it costs more than the walk it
//! saves; the shared index earns its keep where a query is asked per vertex,
//! as the weld and the hygiene's nearest pair ask it.
use super::adjacency::Edges;
use super::trim::KeptMesh;
use crate::space::distance;

type V3 = [f64;3];

/// A vertex standing in the window.
#[derive(Clone,Debug,PartialEq)]
pub struct Near {
    pub vertex: u32,
    pub at: V3,
    pub distance: f64,
    /// The sheets whose triangles use it, ascending. Empty where no triangle
    /// uses it: a point a seed contributed that the clip never kept, which is
    /// in the array but not in the mesh.
    pub sheets: Vec<u32>,
    /// Whether an edge one triangle alone walks ends here.
    pub boundary: bool,
}

/// The mesh within `radius` of `at`.
#[derive(Clone,Debug,PartialEq)]
pub struct Window {
    pub at: V3,
    pub radius: f64,
    /// The vertices within the radius, nearest first, ties by index.
    pub vertices: Vec<Near>,
    /// Every pair of them, nearest first: the two vertices, how far apart, and
    /// how many triangles use the edge between them (none, where no edge
    /// joins them at all — which is what a whisker looks like).
    pub pairs: Vec<(u32,u32,f64,usize)>,
    /// Triangles with a corner within the radius: the triangle, its sheet, its corners.
    pub triangles: Vec<(u32,u32,[u32;3])>,
}

/// The most pairs worth printing: beyond this the table is longer than it is useful.
const PAIRS: usize = 12;

impl Window {
    /// One line of heading and one per thing, or a single line when the ball is empty.
    pub fn report(&self) -> String {
        if self.vertices.is_empty() { return format!("window at {:?} radius {}: empty",self.at,self.radius); }
        let mut out = format!("window at {:?} radius {}: {} vertices, {} triangles",
            self.at,self.radius,self.vertices.len(),self.triangles.len());
        for n in &self.vertices {
            out += &format!("\n      v{} at {:?}, {:.9} away, sheets {:?}{}",
                n.vertex,n.at,n.distance,n.sheets,if n.boundary { ", on the boundary" } else { "" });
        }
        for (a,b,d,uses) in self.pairs.iter().take(PAIRS) {
            out += &format!("\n      v{a} to v{b}: {d:.9} apart, {}",
                if *uses == 0 { "joined by no edge".to_string() } else { format!("an edge of {uses} triangles") });
        }
        if self.pairs.len() > PAIRS { out += &format!("\n      ({} further pairs)",self.pairs.len()-PAIRS); }
        for (i,s,t) in &self.triangles { out += &format!("\n      t{i} of sheet {s}: {t:?}"); }
        out
    }
}

/// What stands within `radius` of `at`.
pub fn window(mesh: &KeptMesh,at: V3,radius: f64) -> Window {
    let mut found: Vec<(f64,u32)> = mesh.vertices.iter().enumerate()
        .filter_map(|(v,p)| { let d = distance(*p,at); (d <= radius).then_some((d,v as u32)) }).collect();
    found.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let inside: std::collections::BTreeSet<u32> = found.iter().map(|(_,v)| *v).collect();
    let edges = Edges::new(&mesh.triangles);
    let mut ends: std::collections::BTreeSet<u32> = Default::default();
    for (a,b) in edges.boundary() { ends.insert(a); ends.insert(b); }
    // the triangles on the window, and from them the sheets each of its vertices belongs to
    let mut sheets: std::collections::BTreeMap<u32,std::collections::BTreeSet<u32>> = Default::default();
    let mut triangles = Vec::new();
    for (i,t) in mesh.triangles.iter().enumerate() {
        if !t.iter().any(|v| inside.contains(v)) { continue; }
        let s = mesh.sheet.get(i).copied().unwrap_or(u32::MAX);
        triangles.push((i as u32,s,*t));
        for &v in t { if inside.contains(&v) { sheets.entry(v).or_default().insert(s); } }
    }
    let mut pairs: Vec<(u32,u32,f64,usize)> = Vec::new();
    for i in 0..found.len() { for j in i+1..found.len() {
        let (a,b) = (found[i].1,found[j].1);
        let d = distance(mesh.vertices[a as usize],mesh.vertices[b as usize]);
        pairs.push((a.min(b),a.max(b),d,edges.uses(a,b)));
    } }
    pairs.sort_by(|x,y| x.2.total_cmp(&y.2).then(x.0.cmp(&y.0)).then(x.1.cmp(&y.1)));
    let vertices = found.into_iter().map(|(d,v)| Near {
        vertex:v,at:mesh.vertices[v as usize],distance:d,
        sheets:sheets.get(&v).map_or(Vec::new(),|s| s.iter().copied().collect()),
        boundary:ends.contains(&v),
    }).collect();
    Window {at,radius,vertices,pairs,triangles}
}
