//! **Whether a point is in a swept solid** (#721 X0, step 1): `x ∈ sweep(S, M)` exactly when the
//! point carried back, `M(t)⁻¹ x`, lies in the tool `S` for some roll `t` — the definition, read by
//! the kernel without the drawing. The tool is read off its B-rep, meshed within a bar
//! (`Solid`): its signed distance at a point is the distance to the nearest triangle, negative
//! where the triangles' generalised winding number says the point is inside (Jacobson, Kavan &
//! Sorkine-Hornung 2013), so a mesh that only nearly closes still classifies. Over the roll the
//! test is a branch and bound (`Swept::member`): at a roll the carried point stands at a signed
//! distance `d`; it moves no faster than the trajectory's speed bound `L`
//! (`Trajectory::inverse_point_speed_bound`), so over a stretch of half-width `h` about that roll
//! it stays outside where `d − sag > L h`, and it is inside where `d < −sag`. A point the bound
//! cannot settle before the stretches shrink to `FINEST` of the roll stands within the mesh's sag
//! of the swept boundary, and is said so (`Unsettled`) rather than guessed.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::mesh::mesh;
use crate::topo::Brep;
use crate::trajectory::Trajectory;
use crate::util::bvh::{Bounds,Bvh};
use crate::util::interval::Interval;
use crate::util::space::{add,cross,dot,norm,scale,sub};

type V = [f64;3];

/// The least stretch of the roll the bound halves to, as a part of the roll.
const FINEST: f64 = 1e-9;
/// The most stretches one point's test reads.
const MOST_READS: usize = 1 << 16;

/// A tool read for membership: its boundary meshed, scaled to the trajectory's units, a box
/// about each triangle, and how far the mesh may stand off the exact boundary (its sag, plus the
/// bar's rounding).
pub struct Solid { pts: Vec<V>,tris: Vec<[u32;3]>,tree: Bvh<3>,pub sag: f64 }

impl Solid {
    /// The B-rep `b` (its units) meshed within `bar` and read in units `1/scale` of its own (a
    /// millimetre B-rep read in a model's units of `scale` millimetres).
    pub fn new(b: &Brep,bar: f64,scale: f64) -> Result<Solid,String> {
        let m = mesh(b,bar,0.1)?;
        let pts: Vec<V> = m.pts.iter().map(|p| p.map(|x| x/scale)).collect();
        if m.tris.is_empty() { return Err("the tool meshes to no triangle".into()) }
        let tree = Bvh::new(m.tris.iter().map(|t| {
            let [a,b,c] = t.map(|i| pts[i as usize]);
            Bounds {lo:std::array::from_fn(|k| a[k].min(b[k]).min(c[k])),hi:std::array::from_fn(|k| a[k].max(b[k]).max(c[k]))}
        }));
        let sag = (m.sag.max(bar))/scale;
        Ok(Solid {pts,tris:m.tris,tree,sag})
    }

    /// The distance from `p` to the meshed boundary.
    pub fn distance(&self,p: V) -> f64 {
        let best = std::cell::Cell::new(f64::INFINITY);
        self.tree.query_nearest(|b| box_gap(b,p),|| best.get(),|i| {
            let [a,b,c] = self.tris[i].map(|k| self.pts[k as usize]);
            let d = norm(sub(p,closest(p,a,b,c)));
            if d < best.get() { best.set(d) }
        });
        best.get()
    }

    /// The generalised winding number at `p`: near 1 inside, near 0 outside.
    pub fn winding(&self,p: V) -> f64 {
        let mut w = 0.;
        for t in &self.tris {
            let [a,b,c] = t.map(|k| sub(self.pts[k as usize],p));
            let (la,lb,lc) = (norm(a),norm(b),norm(c));
            let det = dot(a,cross(b,c));
            let div = la*lb*lc+dot(a,b)*lc+dot(b,c)*la+dot(c,a)*lb;
            w += 2.*det.datan2(div);
        }
        w/(4.*std::f64::consts::PI)
    }

    /// The signed distance at `p`: negative inside.
    pub fn signed_distance(&self,p: V) -> f64 {
        let d = self.distance(p);
        if self.winding(p) > 0.5 { -d } else { d }
    }
}

/// How far a box stands from `p`.
fn box_gap(b: Bounds<3>,p: V) -> f64 {
    let d: V = std::array::from_fn(|k| (b.lo[k]-p[k]).max(p[k]-b.hi[k]).max(0.));
    norm(d)
}

/// The point of triangle `abc` nearest `p` (Ericson, Real-Time Collision Detection, 5.1.5).
fn closest(p: V,a: V,b: V,c: V) -> V {
    let (ab,ac,ap) = (sub(b,a),sub(c,a),sub(p,a));
    let (d1,d2) = (dot(ab,ap),dot(ac,ap));
    if d1 <= 0. && d2 <= 0. { return a }
    let bp = sub(p,b);
    let (d3,d4) = (dot(ab,bp),dot(ac,bp));
    if d3 >= 0. && d4 <= d3 { return b }
    let vc = d1*d4-d3*d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. { return add(a,scale(ab,d1/(d1-d3))) }
    let cp = sub(p,c);
    let (d5,d6) = (dot(ab,cp),dot(ac,cp));
    if d6 >= 0. && d5 <= d6 { return c }
    let vb = d5*d2-d1*d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. { return add(a,scale(ac,d2/(d2-d6))) }
    let va = d3*d6-d5*d4;
    if va <= 0. && d4-d3 >= 0. && d5-d6 >= 0. { return add(b,scale(sub(c,b),(d4-d3)/((d4-d3)+(d5-d6)))) }
    let denom = 1./(va+vb+vc);
    add(a,add(scale(ab,vb*denom),scale(ac,vc*denom)))
}

/// Why a point's membership is not settled: it stands within the mesh's sag of the swept boundary
/// at roll `at`, or its trajectory could not be read or bounded.
#[derive(Clone,Debug,PartialEq)]
pub enum Unsettled { Near { at: f64 },Unbounded,Unread(String) }

/// A tool carried along a trajectory over a roll: what membership is read of.
pub struct Swept<'a> { pub tool: &'a Solid,pub path: &'a Trajectory,pub roll: [f64;2] }

impl Swept<'_> {
    /// **Whether `x` is in the swept solid**, or why that is not settled (`Unsettled`).
    pub fn member(&self,x: V) -> Result<bool,Unsettled> {
        let [a,b] = self.roll;
        let domain = Interval::new(a.min(b),a.max(b)).map_err(|_| Unsettled::Unbounded)?;
        let speed = self.path.inverse_point_speed_bound(x,domain).map_err(|_| Unsettled::Unbounded)?;
        let sag = self.tool.sag;
        let finest = FINEST*(b-a).abs().max(1e-300);
        let mut stack = vec![(a.min(b),a.max(b))];
        let mut near: Option<f64> = None;
        let mut reads = 0;
        while let Some((lo,hi)) = stack.pop() {
            reads += 1;
            if reads > MOST_READS { return Err(near.map_or(Unsettled::Unbounded,|at| Unsettled::Near {at})) }
            let m = 0.5*(lo+hi);
            let pose = self.path.pose(m).map_err(|e| Unsettled::Unread(e.msg))?;
            let d = self.tool.signed_distance(pose.inverse().point(x));
            if d < -sag { return Ok(true) }
            let h = 0.5*(hi-lo);
            if d-sag > speed*h { continue }
            if h <= finest { near.get_or_insert(m); continue }
            stack.push((m,hi));
            stack.push((lo,m));
        }
        match near { Some(at) => Err(Unsettled::Near {at}),None => Ok(false) }
    }
}
