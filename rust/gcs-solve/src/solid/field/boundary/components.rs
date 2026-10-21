//! Diagnostic connected pieces of a refused candidate, never a solid certificate.
use super::{I,V};
use std::collections::BTreeMap;

#[derive(Clone,Debug,PartialEq)]
pub struct BoundaryComponent {
    pub vertices: usize,
    pub triangles: usize,
    pub bounds: V,
    /// Approximate, for locating/refining a rejected candidate only.
    pub signed_volume: f64,
}

pub(super) fn components(vertices: &[[f64;3]],triangles: &[[usize;3]]) -> Vec<BoundaryComponent> {
    fn root(parent: &mut [usize],mut i: usize) -> usize {
        while parent[i] != i { parent[i] = parent[parent[i]]; i = parent[i]; }
        i
    }
    let mut parent: Vec<_> = (0..vertices.len()).collect();
    for t in triangles {
        for &i in &t[1..] { let a = root(&mut parent,t[0]); let b = root(&mut parent,i); parent[b] = a; }
    }
    let roots: Vec<_> = (0..vertices.len()).map(|i| root(&mut parent,i)).collect();
    let mut groups = BTreeMap::new();
    for (i,&p) in vertices.iter().enumerate() {
        let c = groups.entry(roots[i]).or_insert_with(|| BoundaryComponent {
            vertices:0,triangles:0,bounds:p.map(|v| I::point(v).unwrap()),signed_volume:0.,
        });
        c.vertices += 1;
        for k in 0..3 { let [a,b] = c.bounds[k].bounds(); c.bounds[k] = I::new(a.min(p[k]),b.max(p[k])).unwrap(); }
    }
    for t in triangles {
        let c = groups.get_mut(&roots[t[0]]).unwrap(); c.triangles += 1;
        let origin = c.bounds.map(|b| b.bounds()[0]);
        let [a,b,d] = t.map(|i| std::array::from_fn::<_,3,_>(|k| vertices[i][k]-origin[k]));
        c.signed_volume += (a[0]*(b[1]*d[2]-b[2]*d[1])
            +a[1]*(b[2]*d[0]-b[0]*d[2])+a[2]*(b[0]*d[1]-b[1]*d[0]))/6.;
    }
    let mut result: Vec<_> = groups.into_values().collect();
    result.sort_by(|a,b| b.triangles.cmp(&a.triangles));
    result
}
