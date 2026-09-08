//! Shared candidate geometry and strict-sign boundary witnesses.
use super::*;

pub(super) type V = [f64;3];
pub(super) fn add(a:V,b:V) -> V { std::array::from_fn(|k| a[k]+b[k]) }
pub(super) fn sub(a:V,b:V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
pub(super) fn mul(a:V,s:f64) -> V { a.map(|x| x*s) }
pub(super) fn dot(a:V,b:V) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }
pub(super) fn cross(a:V,b:V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
pub(super) fn length(a:V) -> f64 { a[0].hypot(a[1]).hypot(a[2]) }
pub(super) fn unit(a:V) -> V { let n = length(a); assert!(n > 1e-12); mul(a,1./n) }
pub(super) fn tangent(n:V) -> V {
    let k = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
    let mut axis = [0.;3]; axis[k] = 1.; unit(cross(n,axis))
}

pub(super) fn boundary_radius(center:V,inside:V,outside:V) -> f64 {
    let mut radius = 0_f64;
    for endpoint in [inside,outside] {
        let squared = (0..3).fold(I::ZERO,|s,k| s.add(I::point(endpoint[k]).unwrap()
            .sub(I::point(center[k]).unwrap()).unwrap().square().unwrap()).unwrap());
        radius = radius.max(I::new(0.,squared.bounds()[1]).unwrap().sqrt().unwrap().bounds()[1]);
    }
    radius
}

#[derive(Clone,Debug,PartialEq)]
pub(super) struct Vertex {
    pub(super) p:V,
    pub(super) n:V,
    pub(super) size:f64,
    pub(super) inside:V,
    pub(super) outside:V,
    pub(super) radius:f64,
    pub(super) branches:Vec<V>,
}

impl Vertex {
    pub(super) fn normals(&self) -> &[V] {
        if self.branches.is_empty() { std::slice::from_ref(&self.n) } else { &self.branches }
    }

    pub(super) fn near_edge_interior(&self,a:&Vertex,b:&Vertex) -> bool {
        // These witness radii bound proximity to material boundaries. A gap
        // smaller than their combined uncertainty cannot justify spanning a
        // retained sample without using its vertex in the edge connectivity.
        let radius = self.radius+a.radius.max(b.radius);
        if (0..3).any(|k| self.p[k] < a.p[k].min(b.p[k])-radius ||
            self.p[k] > a.p[k].max(b.p[k])+radius) { return false; }
        let edge = sub(b.p,a.p); let squared = dot(edge,edge);
        if squared <= 0. { return false; }
        let t = dot(sub(self.p,a.p),edge)/squared;
        t > 0. && t < 1. && length(sub(self.p,add(a.p,mul(edge,t)))) <= radius &&
            length(sub(self.p,a.p)) > radius && length(sub(self.p,b.p)) > radius
    }
}
