//! Triangle search proposes convex combinations; outward arithmetic certifies them.
use super::{Box3,Error,I,KeptMesh,MeshWitness,distance,middle,point};

struct Node { bounds: [[f64;3];2], end: usize, first: usize, count: usize }
pub(super) struct MeshIndex { nodes: Vec<Node>, items: Vec<(usize,[[f64;3];2])> }
impl MeshIndex {
    pub(super) fn new(mesh: &KeptMesh) -> Self {
        let items = mesh.triangles.iter().enumerate().map(|(i,t)| {
            let p = t.map(|v| mesh.vertices[v as usize]);
            (i,[std::array::from_fn(|k| p.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min)),
                std::array::from_fn(|k| p.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max))])
        }).collect();
        let mut out = Self {nodes:Vec::new(),items};
        if !out.items.is_empty() { out.build(0,out.items.len()); }
        out
    }
    fn build(&mut self,first: usize,count: usize) {
        let bounds = [std::array::from_fn(|k| self.items[first..first+count].iter().map(|(_,b)| b[0][k]).fold(f64::INFINITY,f64::min)),
            std::array::from_fn(|k| self.items[first..first+count].iter().map(|(_,b)| b[1][k]).fold(f64::NEG_INFINITY,f64::max))];
        let slot = self.nodes.len();
        self.nodes.push(Node {bounds,end:slot+1,first,count});
        if count > 8 {
            let axis = (0..3).max_by(|&a,&b| (bounds[1][a]-bounds[0][a]).total_cmp(&(bounds[1][b]-bounds[0][b]))).unwrap();
            let mid = count/2;
            self.items[first..first+count].select_nth_unstable_by(mid,|(i,a),(j,b)|
                (a[0][axis]*0.5+a[1][axis]*0.5).total_cmp(&(b[0][axis]*0.5+b[1][axis]*0.5)).then(i.cmp(j)));
            self.nodes[slot].count = 0;
            self.build(first,mid); self.build(first+mid,count-mid);
            self.nodes[slot].end = self.nodes.len();
        }
    }
    pub(super) fn witness(&self,cell: Box3,tol: f64,mesh: &KeptMesh) -> Result<Option<(MeshWitness,f64)>,Error> {
        let p = middle(cell);
        // Search heuristics can miss a witness and cause refinement/refusal. Only
        // the final interval bound can accept a cell.
        if cell.iter().any(|v| v.bounds()[1]-v.bounds()[0] > 2.*tol) { return Ok(None); }
        let overlaps = |b: [[f64;3];2]| (0..3).all(|k| b[0][k] <= p[k]+tol && b[1][k] >= p[k]-tol);
        let mut node = 0;
        while node < self.nodes.len() {
            let n = &self.nodes[node];
            if !overlaps(n.bounds) { node = n.end; continue; }
            for &(triangle,bounds) in &self.items[n.first..n.first+n.count] {
                if !overlaps(bounds) { continue; }
                let [a,b,c] = mesh.triangles[triangle].map(|v| mesh.vertices[v as usize]);
                let q = crate::space::closest_on_triangle(p,a,b,c).0;
                let (ab,ac,aq) = (crate::space::sub(b,a),crate::space::sub(c,a),crate::space::sub(q,a));
                let dot = crate::space::dot;
                let (aa,bb,cc,dd,ee) = (dot(ab,ab),dot(ab,ac),dot(ac,ac),dot(aq,ab),dot(aq,ac));
                let det = aa*cc-bb*bb;
                let (u,v) = if det > 0. && det.is_finite() {
                    let v = ((aa*ee-bb*dd)/det).clamp(0.,1.);
                    let u = if v < 1. { ((cc*dd-bb*ee)/det/(1.-v)).clamp(0.,1.) } else {0.};
                    if u.is_finite() && v.is_finite() { (u,v) } else {(0.,0.)}
                } else {(0.,0.)};
                let lerp = |a: Box3,b: Box3,t: f64| -> Result<Box3,Error> {
                    let t = I::point(t)?; let s = I::point(1.)?.sub(t)?;
                    let mut out = [I::ZERO;3];
                    for k in 0..3 { out[k] = a[k].mul(s)?.add(b[k].mul(t)?)?; }
                    Ok(out)
                };
                let point = lerp(lerp(point(a)?,point(b)?,u)?,point(c)?,v)?;
                let d = distance(cell,point)?;
                if d <= tol { return Ok(Some((MeshWitness {triangle,parameters:[u,v],point},d))); }
            }
            node += 1;
        }
        Ok(None)
    }
}
