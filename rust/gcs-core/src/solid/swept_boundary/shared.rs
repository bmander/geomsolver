//! Experimental tessellation of already arranged parametric domains. The caller
//! supplies trim/event identities and globally visible domains; this module does
//! not infer them from nearby mesh vertices. Output is a candidate, never an
//! accepted solid. Refinement tests are sampled approximation diagnostics.
use super::KeptMesh;
use crate::space::{norm,sub};
use std::collections::BTreeMap;
type V3 = [f64;3];
type UV = [f64;2];
const CORNERS: [UV;4] = [[0.,0.],[0.,1.],[1.,1.],[1.,0.]];

/// Outward domain order is (0,0), (0,1), (1,1), (1,0). A collapsed
/// boundary has equal endpoint identities, declared by the chart adapter.
pub struct Patch<'a> {
    pub id: u32,
    pub source: u32,
    pub corners: [u32;4],
    pub edges: [u32;4],
    /// Minimum dyadic levels in each parameter, for anisotropic chart sampling.
    pub minimum_levels: [u8;2],
    pub evaluate: Box<dyn Fn(UV) -> V3 + 'a>,
}
#[derive(Clone,Copy)]
pub struct Options {
    pub sagitta: f64,
    /// Agreement of evaluations at a declared common parameter, not a weld radius.
    pub agreement: f64,
    pub max_level: u8,
    pub max_triangles: usize,
}
#[derive(Debug,PartialEq)]
pub enum Error {
    InvalidInput,
    InconsistentCorner(u32),
    InconsistentEdge(u32),
    EdgeIncidence(u32),
    NonFinite(u32),
    Degenerate(u32),
    Budget,
    Topology(crate::topology::Error),
}
#[derive(Debug)]
pub struct EdgeUse { pub patch: u32, pub parameters: Vec<UV> }
#[derive(Debug)]
pub struct Edge {
    pub id: u32,
    pub vertices: Vec<u32>,
    pub uses: Vec<EdgeUse>,
}
#[derive(Debug)]
pub struct Face { pub patch: u32, pub parameters: [UV;3] }
#[derive(Debug)]
pub struct Tessellation {
    pub mesh: KeptMesh,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    pub level: u8,
    pub divisions: BTreeMap<u32,[usize;2]>,
    pub sampled_error: f64,
}
fn uv(side: usize,t: f64) -> UV {
    let (a,b) = (CORNERS[side],CORNERS[(side+1)%4]);
    std::array::from_fn(|k| a[k]+t*(b[k]-a[k]))
}
fn at(p: &Patch<'_>,uv: UV) -> Result<V3,Error> {
    let x = (p.evaluate)(uv);
    if x.iter().all(|x| x.is_finite()) { Ok(x) } else { Err(Error::NonFinite(p.id)) }
}

/// Build independently, checking declared incidence before emitting triangles.
/// The current bounded experiment propagates the finest dyadic level to all
/// domains. It deliberately trades extra interior samples for simple conformity.
/// No partial mesh is returned on failure; the caller's candidate is untouched.
pub fn tessellate(patches: &[Patch<'_>],options: Options) -> Result<Tessellation,Error> {
    if patches.is_empty() || !(options.sagitta > 0. && options.sagitta.is_finite())
        || !(options.agreement > 0. && options.agreement.is_finite()) || options.max_level > 12 {
        return Err(Error::InvalidInput);
    }
    let mut ordered: Vec<_> = patches.iter().collect(); ordered.sort_by_key(|p| p.id);
    if ordered.windows(2).any(|w| w[0].id == w[1].id) { return Err(Error::InvalidInput); }
    let mut corners = BTreeMap::<u32,V3>::new();
    let mut uses = BTreeMap::<u32,Vec<(&Patch<'_>,usize)>>::new();
    for &p in &ordered { for k in 0..4 {
        let x = at(p,CORNERS[k])?;
        if let Some(&old) = corners.get(&p.corners[k]) {
            if norm(sub(x,old)) > options.agreement { return Err(Error::InconsistentCorner(p.corners[k])); }
        } else { corners.insert(p.corners[k],x); }
        uses.entry(p.edges[k]).or_default().push((p,k));
    } }
    for (&id,u) in &uses {
        let (p,k) = u[0]; let ends = [p.corners[k],p.corners[(k+1)%4]];
        if ends[0] != ends[1] && u.len() != 2 { return Err(Error::EdgeIncidence(id)); }
        for &(q,j) in u.iter().skip(1) {
            let other = [q.corners[j],q.corners[(j+1)%4]];
            if other != [ends[1],ends[0]] { return Err(Error::InconsistentEdge(id)); }
        }
    }
    let mut levels: BTreeMap<_,_> = ordered.iter().map(|p| (p.id,p.minimum_levels)).collect();
    if levels.values().flatten().any(|&n| n > 12 || n as u16+options.max_level as u16 > 12) { return Err(Error::InvalidInput); }
    // Opposite sides use one structured parameter sequence. Propagate every
    // incident consumer's refinement request through that equality constraint.
    loop {
        let mut changed = false;
        for consumers in uses.values() {
            let max = consumers.iter().map(|(p,k)| levels[&p.id][1-k%2]).max().unwrap();
            for &(p,k) in consumers {
                let l = &mut levels.get_mut(&p.id).unwrap()[1-k%2];
                if *l < max { *l = max; changed = true; }
            }
        }
        if !changed { break; }
    }
    for level in 0..=options.max_level {
        let divisions: BTreeMap<_,_> = levels.iter().map(|(&id,l)| (id,l.map(|k| 1usize << (k+level)))).collect();
        let total = divisions.values().map(|[u,v]| 2*u*v).sum::<usize>();
        if total > options.max_triangles { return Err(Error::Budget); }
        let mut mesh = KeptMesh::default();
        let mut corner_ids = BTreeMap::new();
        for (&id,&p) in &corners { corner_ids.insert(id,mesh.vertices.len() as u32); mesh.vertices.push(p); }
        let mut edges = Vec::new(); let mut boundary = BTreeMap::new();
        for (&id,consumers) in &uses {
            let (first,k) = consumers[0]; let a = first.corners[k]; let b = first.corners[(k+1)%4];
            let n = divisions[&first.id][1-k%2];
            let mut vertices = Vec::new(); let mut edge_uses = Vec::new();
            for j in 0..=n {
                let t = j as f64/n as f64; let point = at(first,uv(k,t))?;
                let vertex = if a == b {
                    if norm(sub(point,corners[&a])) > options.agreement { return Err(Error::InconsistentEdge(id)); }
                    corner_ids[&a]
                } else if j == 0 { corner_ids[&a] } else if j == n { corner_ids[&b] }
                else { let v = mesh.vertices.len() as u32; mesh.vertices.push(point); v };
                vertices.push(vertex);
            }
            for &(p,side) in consumers {
                let reverse = p.corners[side] != a;
                let mut parameters = Vec::new();
                for j in 0..=n {
                    let t = j as f64/n as f64; let param = uv(side,if reverse {1.-t} else {t});
                    if norm(sub(at(p,param)?,mesh.vertices[vertices[j] as usize])) > options.agreement {
                        return Err(Error::InconsistentEdge(id));
                    }
                    parameters.push(param);
                    let [nu,nv] = divisions[&p.id];
                    let i = (param[0]*nu as f64).round() as usize;
                    let k = (param[1]*nv as f64).round() as usize;
                    boundary.insert((p.id,i,k),vertices[j]);
                }
                edge_uses.push(EdgeUse {patch:p.id,parameters});
            }
            edges.push(Edge {id,vertices,uses:edge_uses});
        }
        let mut faces = Vec::new(); let mut error = 0_f64;
        for &p in &ordered {
            let [nu,nv] = divisions[&p.id];
            let mut grid = vec![0u32;(nu+1)*(nv+1)];
            for i in 0..=nu { for j in 0..=nv {
                let v = if let Some(&v) = boundary.get(&(p.id,i,j)) { v }
                    else { let v = mesh.vertices.len() as u32; mesh.vertices.push(at(p,[i as f64/nu as f64,j as f64/nv as f64])?); v };
                grid[i*(nv+1)+j] = v;
            } }
            for i in 0..nu { for j in 0..nv {
                for indices in [[[i,j],[i,j+1],[i+1,j+1]],[[i,j],[i+1,j+1],[i+1,j]]] {
                    let t = indices.map(|[i,j]| grid[i*(nv+1)+j]);
                    let params = indices.map(|[i,j]| [i as f64/nu as f64,j as f64/nv as f64]);
                    // Test the chord even on a collapsed triangle: a declared
                    // singular edge must not hide a curved interior sliver.
                    for weights in [[0.5,0.5,0.],[0.,0.5,0.5],[0.5,0.,0.5],[1./3.;3]] {
                        let param = std::array::from_fn(|k| (0..3).map(|j| weights[j]*params[j][k]).sum());
                        let chord = std::array::from_fn(|k| (0..3).map(|j| weights[j]*mesh.vertices[t[j] as usize][k]).sum());
                        error = error.max(norm(sub(at(p,param)?,chord)));
                    }
                    if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] { continue; }
                    if super::triangle_normal(mesh.vertices[t[0] as usize],mesh.vertices[t[1] as usize],mesh.vertices[t[2] as usize]).is_none() {
                        return Err(Error::Degenerate(p.id));
                    }
                    mesh.triangles.push(t); mesh.sheet.push(p.source); faces.push(Face {patch:p.id,parameters:params});
                }
            } }
        }
        if error <= options.sagitta {
            let tris: Vec<_> = mesh.triangles.iter().map(|t| t.map(|v| v as usize)).collect();
            crate::topology::ClosedShell::from_triangles(mesh.vertices.len(),&tris).map_err(Error::Topology)?;
            return Ok(Tessellation {mesh,edges,faces,level,divisions,sampled_error:error});
        }
    }
    Err(Error::Budget)
}
