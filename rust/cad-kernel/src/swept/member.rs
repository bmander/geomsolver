//! **Whether a point is in a swept solid** (#721 X0, step 1): `x ∈ sweep(S, M)` exactly when the
//! point carried back, `M(t)⁻¹ x`, lies in the tool `S` for some roll `t` — the definition, read by
//! the kernel without the drawing. The tool is read off its B-rep, meshed within a bar
//! (`Solid`): its signed distance at a point is the distance to the nearest triangle, negative
//! where the triangles' generalised winding number says the point is inside (Jacobson, Kavan &
//! Sorkine-Hornung 2013), so a mesh that only nearly closes still classifies. Over the roll the
//! test is a branch and bound (`Swept::member`): at a roll the carried point stands at a signed
//! distance `d`; it moves no faster than the trajectory's speed bound `L`
//! (`Trajectory::inverse_point_speed_bound`), so over a stretch of half-width `h` about that roll
//! it stays outside where `d − sag > L h`, and it is inside where `d < −sag`. The stretches are read
//! best first, the one whose bound reaches deepest inside first (#810). A point the bound cannot
//! settle before the stretches shrink to `FINEST` of the roll stands within the mesh's sag of the
//! swept boundary, and is said so (`Unsettled`) rather than guessed.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::mesh::mesh;
use crate::topo::Brep;
use crate::trajectory::Trajectory;
use crate::util::bvh::{Bounds,Bvh};
use crate::util::interval::Interval;
use crate::util::space::{closest_on_triangle,cross,distance,dot,norm,sub};

type V = [f64;3];

/// The least stretch of the roll the bound halves to, as a part of the roll.
const FINEST: f64 = 1e-9;
/// The most stretches one point's test reads.
const MOST_READS: usize = 1 << 16;
/// The most stretches `Swept::beyond` reads before it says `None`, the cautious answer: a ball near
/// the boundary whose proof would take more is left near, and its pairs met.
const BEYOND_READS: usize = 256;

/// A tool read for membership: its boundary meshed, scaled to the trajectory's units, a box
/// about each triangle, and how far the mesh may stand off the exact boundary (its measured sag,
/// and rounding's).
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
        // the sag measured on the mesh (none where every face is flat), and rounding's at its size
        let size = pts.iter().fold(0_f64,|r,p| r.max(norm(*p)));
        let sag = m.sag/scale+1e-12*(1.+size);
        Ok(Solid {pts,tris:m.tris,tree,sag})
    }

    /// How many triangles the boundary is meshed in.
    pub fn triangles(&self) -> usize { self.tris.len() }

    /// The distance from `p` to the meshed boundary.
    pub fn distance(&self,p: V) -> f64 {
        let best = std::cell::Cell::new(f64::INFINITY);
        self.tree.query_nearest(|b| b.gap(Bounds {lo:p,hi:p}),|| best.get(),|i| {
            let [a,b,c] = self.tris[i].map(|k| self.pts[k as usize]);
            let d = distance(p,closest_on_triangle(p,a,b,c).0);
            if d < best.get() { best.set(d) }
        });
        best.get()
    }

    /// **The distance from `p` to the meshed boundary, and on which side `p` stands** (E1's zones):
    /// inside or not read off the nearest triangle's normal where the nearest point lies within that
    /// triangle — on a closed mesh turned outward, exactly the side — and off the winding number
    /// where it lies on a triangle's edge or corner, where the triangles there disagree.
    pub fn signed_near(&self,p: V) -> (f64,bool) {
        let best = std::cell::Cell::new((f64::INFINITY,usize::MAX,crate::util::space::Region::Vertex));
        self.tree.query_nearest(|b| b.gap(Bounds {lo:p,hi:p}),|| best.get().0,|i| {
            let [a,b,c] = self.tris[i].map(|k| self.pts[k as usize]);
            let (q,region) = closest_on_triangle(p,a,b,c);
            let d = distance(p,q);
            if d < best.get().0 { best.set((d,i,region)) }
        });
        let (d,i,region) = best.get();
        if i == usize::MAX { return (d,false) }
        if region == crate::util::space::Region::Face {
            let [a,b,c] = self.tris[i].map(|k| self.pts[k as usize]);
            let n = cross(sub(b,a),sub(c,a));
            let side = dot(sub(p,a),n);
            if side != 0. { return (d,side < 0.) }
        }
        (d,self.winding(p) > 0.5)
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

/// Why a point's membership is not settled: it stands within the mesh's sag of the swept boundary
/// at roll `at`, or its trajectory could not be read or bounded, or the build asking stopped.
#[derive(Clone,Debug,PartialEq)]
pub enum Unsettled { Near { at: f64 },Unbounded,Unread(String),
    /// The build reading it went past its budget (`util::meter`): nothing more is read.
    Stopped }

/// A stretch of the roll membership is still to read (`Swept::member`): the least signed distance
/// its parent's reading allows over it, the order it was pushed in, and its ends. The heap takes
/// the least key first, and of equal keys the first pushed.
struct Stretch { key: f64,order: u64,lo: f64,hi: f64 }
impl PartialEq for Stretch { fn eq(&self,o: &Stretch) -> bool { self.cmp(o) == std::cmp::Ordering::Equal } }
impl Eq for Stretch {}
impl PartialOrd for Stretch { fn partial_cmp(&self,o: &Stretch) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for Stretch {
    fn cmp(&self,o: &Stretch) -> std::cmp::Ordering { o.key.total_cmp(&self.key).then(o.order.cmp(&self.order)) }
}

/// A tool carried along a trajectory over a roll: what membership is read of.
pub struct Swept<'a> { pub tool: &'a Solid,pub path: &'a Trajectory,pub roll: [f64;2] }

impl Swept<'_> {
    /// **Whether `x` is in the swept solid**, or why that is not settled (`Unsettled`). A path of
    /// pieces (K1) is read piece by piece over the stretch of the roll each holds, the sweep of a
    /// path its pieces' sweeps united: a member of any is one, and it is settled out only where every
    /// piece settles it so.
    pub fn member(&self,x: V) -> Result<bool,Unsettled> {
        if let Trajectory::Pieces(pieces) = self.path {
            let [a,b] = [self.roll[0].min(self.roll[1]),self.roll[0].max(self.roll[1])];
            let mut unsettled = None;
            for piece in pieces.pieces() {
                let (lo,hi) = (piece.over[0].max(a),piece.over[1].min(b));
                if !(hi > lo) { continue }
                match (Swept {tool:self.tool,path:&piece.path,roll:[lo,hi]}).member(x) {
                    Ok(true) => return Ok(true),
                    Ok(false) => {}
                    Err(e) => { unsettled.get_or_insert(e); }
                }
            }
            return unsettled.map_or(Ok(false),Err)
        }
        let [a,b] = self.roll;
        let domain = Interval::new(a.min(b),a.max(b)).map_err(|_| Unsettled::Unbounded)?;
        let speed = self.path.inverse_point_speed_bound(x,domain).map_err(|_| Unsettled::Unbounded)?;
        let sag = self.tool.sag;
        let finest = FINEST*(b-a).abs().max(1e-300);
        // **best first** (#810): each stretch keyed by the least signed distance its parent's
        // reading allows over it, `d − L h`, the deepest first, ties in the order pushed — so a pose
        // where the point is inside is read before a stretch where it stands within the sag of the
        // boundary is halved down to `FINEST` (taken depth first, the left of the two, such a
        // stretch ran past `MOST_READS` before the right, where the point lay deep inside, was read)
        let mut frontier = std::collections::BinaryHeap::new();
        let mut pushed = 0u64;
        frontier.push(Stretch {key:f64::NEG_INFINITY,order:0,lo:a.min(b),hi:a.max(b)});
        let mut near: Option<f64> = None;
        let mut reads = 0;
        while let Some(Stretch {lo,hi,..}) = frontier.pop() {
            reads += 1;
            // each stretch read counted against the build's budget (#777 V3)
            if !crate::util::meter::charge(crate::util::meter::Stage::Membership,1,|| (x,"membership".into())) { return Err(Unsettled::Stopped) }
            if reads > MOST_READS { return Err(near.map_or(Unsettled::Unbounded,|at| Unsettled::Near {at})) }
            let m = 0.5*(lo+hi);
            let pose = self.path.pose(m).map_err(|e| Unsettled::Unread(e.msg))?;
            // the side, by the winding number, only where the distance can settle either way
            let back = pose.inverse().point(x);
            let mut d = self.tool.distance(back);
            if d > sag && self.tool.winding(back) > 0.5 { d = -d }
            if d < -sag { return Ok(true) }
            let h = 0.5*(hi-lo);
            if d-sag > speed*h { continue }
            if h <= finest { near.get_or_insert(m); continue }
            let key = d-speed*h;
            for (lo,hi) in [(lo,m),(m,hi)] { pushed += 1; frontier.push(Stretch {key,order:pushed,lo,hi}) }
        }
        match near { Some(at) => Err(Unsettled::Near {at}),None => Ok(false) }
    }

    /// **Whether `x` stands more than `r` from the swept boundary, and on which side** (E1, #880;
    /// docs/swept-solid-design.md §12, P3): `Some(true)` where `m(x) < −r`, `m(x)` the least signed
    /// distance from the tool of `x` carried back over the roll — so every point within `r` of `x`
    /// is a member, `m` being 1-Lipschitz (a signed distance is, and a rigid motion keeps
    /// distances); `Some(false)` where `m(x) > r`, so none is; `None` where neither is proved. Read
    /// as `member` reads (the mesh's sag, the trajectory's speed bound `L` over each stretch), a
    /// stretch halved no further than `L h ≤ r/4` and no more than `BEYOND_READS` read, past which
    /// it proves nothing: `None` is the cautious answer, never a wrong one.
    pub fn beyond(&self,x: V,r: f64) -> Result<Option<bool>,Unsettled> {
        if let Trajectory::Pieces(pieces) = self.path {
            let [a,b] = [self.roll[0].min(self.roll[1]),self.roll[0].max(self.roll[1])];
            let mut out = Some(false);
            for piece in pieces.pieces() {
                let (lo,hi) = (piece.over[0].max(a),piece.over[1].min(b));
                if !(hi > lo) { continue }
                match (Swept {tool:self.tool,path:&piece.path,roll:[lo,hi]}).beyond(x,r)? {
                    Some(true) => return Ok(Some(true)),
                    Some(false) => {}
                    None => out = None,
                }
            }
            return Ok(out)
        }
        let [a,b] = self.roll;
        let domain = Interval::new(a.min(b),a.max(b)).map_err(|_| Unsettled::Unbounded)?;
        let speed = self.path.inverse_point_speed_bound(x,domain).map_err(|_| Unsettled::Unbounded)?;
        let sag = self.tool.sag;
        let mut frontier = std::collections::BinaryHeap::new();
        let mut pushed = 0u64;
        frontier.push(Stretch {key:f64::NEG_INFINITY,order:0,lo:a.min(b),hi:a.max(b)});
        // whether some pose stands within `r` (and the sag) of the tool, so `m(x) > r` is not proved
        let mut near = false;
        let mut reads = 0;
        while let Some(Stretch {lo,hi,key,..}) = frontier.pop() {
            // the deepest any stretch left may reach is short of proving `x` inside: nothing more
            if key >= -r-sag && near { return Ok(None) }
            reads += 1;
            if !crate::util::meter::charge(crate::util::meter::Stage::Membership,1,|| (x,"membership".into())) { return Err(Unsettled::Stopped) }
            if reads > BEYOND_READS { return Ok(None) }
            let m = 0.5*(lo+hi);
            let pose = self.path.pose(m).map_err(|e| Unsettled::Unread(e.msg))?;
            let back = pose.inverse().point(x);
            let (mut d,inside) = self.tool.signed_near(back);
            if d > r+sag && inside { d = -d }
            if d < -r-sag { return Ok(Some(true)) }
            if d <= r+sag { near = true }
            let h = 0.5*(hi-lo);
            if d-sag-speed*h > r { continue }
            if speed*h <= 0.25*r { near = true; continue }
            let key = d-sag-speed*h;
            for (lo,hi) in [(lo,m),(m,hi)] { pushed += 1; frontier.push(Stretch {key,order:pushed,lo,hi}) }
        }
        Ok(if near { None } else { Some(false) })
    }
}
