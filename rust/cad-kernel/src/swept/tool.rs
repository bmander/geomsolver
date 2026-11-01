//! **The tool a solid is swept as**, the kernel's (#721 F2b): its faces — faces of revolution
//! (`surface`), a prism's sides, drafted or not, and its caps — the edges where two faces of one
//! profile loop meet, the creases where a drafted prism's halves fold at its parting plane, and its
//! material, which says which points are of the tool. The solver reads one off a drawing
//! (the solver's `SweepContacts::read`); nothing here knows how it was drawn.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::contact::SurfacePoint;
use crate::trajectory::{Error,Motion};
use crate::util::space::{add,lerp,scale,sub};
use super::surface::RevolvedSurface;
use std::sync::Arc;

type V = [f64;3];

/// The tool's material: below nought within it, above without, nought on its boundary — what
/// tells a point of a face's chart on the tool from one off it (past a trim, or hidden by another
/// operand), and a face's outward side.
pub trait Material: Send + Sync + std::fmt::Debug {
    fn value(&self,p: V) -> f64;
}

/// A tool as a sweep reads it: its faces, the edges where two faces of a profile loop meet, the
/// creases where a drafted prism's sides fold at its parting plane (#684 P: the side below it at
/// its `v = 1`, then the side above at its `v = 0`, one point at each `u`), and its material. Every
/// face is kept, those a Boolean of the tool hides too: the material is what tells them apart.
#[derive(Clone,Debug)]
pub struct Tool { pub faces: Vec<ToolSurface>,pub edges: Vec<PatchEdge>,pub creases: Vec<[usize;2]>,pub material: Arc<dyn Material> }

impl Tool {
    pub fn patches(&self) -> &[ToolSurface] { &self.faces }
    pub fn edges(&self) -> &[PatchEdge] { &self.edges }
    pub fn creases(&self) -> &[[usize;2]] { &self.creases }
    pub fn material(&self) -> &dyn Material { &*self.material }
    /// Where `edge`'s two faces meet, and a point just inside each face beside it, on a
    /// revolution's charts' start (`v = 0`) and halfway along a prism's sides, clear of its caps:
    /// the probe a corner is told by — the chord between the two points in the tool's material at a
    /// convex corner, out of it at a concave one. (At a cap the chord lies on the tool's boundary,
    /// where the material reads nought either way.)
    pub fn corner(&self,edge: &PatchEdge) -> Option<(V,[V;2])> {
        let ([a,b],[ua,ub]) = (edge.patches,edge.ends);
        let (pa,pb) = (self.faces.get(a)?,self.faces.get(b)?);
        let h = 1e-4;
        let inward = |u: f64| if u < 0.5 { u+h } else { u-h };
        let v = if pa.periodic() { 0. } else { 0.5 };
        Some((pa.at(ua,v).ok()?.position,[pa.at(inward(ua),v).ok()?.position,pb.at(inward(ub),v).ok()?.position]))
    }
}

/// One face of a sweep's tool: a revolution of a profile edge (`u` along the edge, `v` round the
/// turn, periodic), a side of a prism, drafted or not (`u` along the edge, `v` along the
/// extrusion, from one cap to the other), or a prism's cap (`u` and `v` across the box its profile
/// lies in).
#[derive(Clone,Debug)]
pub enum ToolSurface { Revolved(RevolvedSurface),Extruded(ExtrudedSurface),Cap(CapSurface) }

impl ToolSurface {
    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        match self { ToolSurface::Revolved(s) => s.at(u,v),ToolSurface::Extruded(s) => s.at(u,v),ToolSurface::Cap(s) => s.at(u,v) }
    }
    pub fn name(&self) -> &str {
        match self { ToolSurface::Revolved(s) => &s.name,ToolSurface::Extruded(s) => &s.name,ToolSurface::Cap(s) => &s.name }
    }
    /// The source parameters' domain.
    pub fn domain(&self) -> [[f64;2];2] {
        match self { ToolSurface::Revolved(s) => s.domain(),ToolSurface::Extruded(_) | ToolSurface::Cap(_) => [[0.,1.],[0.,1.]] }
    }
    /// Whether `v` runs round a turn and comes back (a full revolution) rather than from one cap
    /// to another.
    pub fn periodic(&self) -> bool { matches!(self,ToolSurface::Revolved(_)) }
    /// The revolution, where this face is one — what a revolution's closed forms are asked of.
    pub fn revolved(&self) -> Option<&RevolvedSurface> {
        match self { ToolSurface::Revolved(s) => Some(s),_ => None }
    }
    /// Which of its prism's caps this face is (0 its `start`, at `from`, where its sides' `v` is 0;
    /// 1 its `end`), none where it is no cap: admission asks a cap only where it reaches the blank
    /// (#684 F).
    pub fn cap(&self) -> Option<usize> { match self { ToolSurface::Cap(s) => Some(s.end as usize),_ => None } }
}

/// A prism's cap: the plane of its profile at its `from` (`start`) or its `to` (`end`), read over
/// the box the profile lies in, `u` and `v` along the box's sides, so `du × dv` is the cap's outward
/// normal. A point of the box outside the profile is no point of the tool, and a sample there is
/// passed over as the material field says (`admission`'s `on_tool`), as a revolution's off a trim.
#[derive(Clone,Debug)]
pub struct CapSurface { pub name: String,end: bool,corner: V,du: V,dv: V }

impl CapSurface {
    /// A cap at its prism's `end` (else its `start`), the box from `corner` along `du` and `dv`,
    /// whose `du × dv` is its outward normal.
    pub fn new(name: String,end: bool,corner: V,du: V,dv: V) -> CapSurface { CapSurface {name,end,corner,du,dv} }
}

impl CapSurface {
    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
        Ok(SurfacePoint {position:add(self.corner,add(scale(self.du,u),scale(self.dv,v))),du:self.du,dv:self.dv})
    }

    pub fn placed(&self,pose: Motion) -> Self {
        CapSurface {name:self.name.clone(),end:self.end,corner:pose.point(self.corner),du:pose.vector(self.du),dv:pose.vector(self.dv)}
    }
}

/// A prism's side: a line, arc or circle of its profile carried along the profile's normal from
/// the prism's `from` to its `to`.  `at(u, v)` is the edge at `u` (its own direction, `p1` to `p2`
/// or round the arc) moved `v` of the way along. A drafted prism's side (#684 G) leans: `at(u, v)`
/// is the point `v` of the way from the edge at `u` to its edge in the section at `to` at `u` — a
/// plane where the edge is a line, a cone where it is an arc. Running both ways from its parting
/// plane (#684 P), it is two such sides, one each side of the plane, which meet there in a crease
/// (`Tool::creases`).
#[derive(Clone,Debug)]
pub struct ExtrudedSurface { pub name: String,edge: Edge,side: Side }

impl ExtrudedSurface {
    /// The side `edge` runs as it is carried, as `side` says.
    pub fn new(name: String,edge: Edge,side: Side) -> ExtrudedSurface { ExtrudedSurface {name,edge,side} }
}

/// A side's edge: a line from `start` along `delta`, or an arc about `center` (`Edge::round`).
#[derive(Clone,Copy,Debug)]
pub enum Edge { Line { start: V,delta: V },Round { center: V,a: V,b: V,sweep: f64 } }

/// How a side runs from its edge at `from` to `to`: along the extrusion, or to its edge there.
#[derive(Clone,Copy,Debug)]
pub enum Side { Straight(V),Leaning(Edge) }

impl Edge {
    /// The arc about `center` of radius `r` from angle `start` through `sweep` (negative clockwise),
    /// in the plane whose axes `vector` lifts a pair of coordinates along.
    pub fn round(center: V,r: f64,start: f64,sweep: f64,vector: impl Fn(f64,f64) -> V) -> Edge {
        let (sn,cs) = start.dsin_cos();
        let turn = sweep.signum();
        Edge::Round {center,a:vector(r*cs,r*sn),b:vector(-turn*r*sn,turn*r*cs),sweep:sweep.abs()}
    }

    /// The edge's point at fraction `u` of it, and its derivative in `u`.
    fn at(&self,u: f64) -> (V,V) {
        match *self {
            Edge::Line {start,delta} => (add(start,scale(delta,u)),delta),
            Edge::Round {center,a,b,sweep} => {
                let (sn,cs) = (u*sweep).dsin_cos();
                (add(center,add(scale(a,cs),scale(b,sn))),scale(add(scale(a,-sn),scale(b,cs)),sweep))
            }
        }
    }

    pub fn placed(&self,pose: Motion) -> Edge {
        match *self {
            Edge::Line {start,delta} => Edge::Line {start:pose.point(start),delta:pose.vector(delta)},
            Edge::Round {center,a,b,sweep} => Edge::Round {center:pose.point(center),a:pose.vector(a),b:pose.vector(b),sweep},
        }
    }
}

impl ExtrudedSurface {
    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
        let (p,d) = self.edge.at(u);
        Ok(match self.side {
            Side::Straight(depth) => SurfacePoint {position:add(p,scale(depth,v)),du:d,dv:depth},
            Side::Leaning(far) => {
                let (q,e) = far.at(u);
                SurfacePoint {position:lerp(p,q,v),du:lerp(d,e,v),dv:sub(q,p)}
            }
        })
    }

    /// Whether its edge is a line at both its ends (a plane side), so its ends at the caps, and a
    /// drafted half's at its parting plane, are lines too: what the constant twist walks across
    /// (#684 AE, AG).
    pub fn straight(&self) -> bool {
        matches!((self.edge,self.side),(Edge::Line {..},Side::Straight(_) | Side::Leaning(Edge::Line {..})))
    }

    pub fn placed(&self,pose: Motion) -> Self {
        let side = match self.side {
            Side::Straight(depth) => Side::Straight(pose.vector(depth)),
            Side::Leaning(far) => Side::Leaning(far.placed(pose)),
        };
        ExtrudedSurface {name:self.name.clone(),edge:self.edge.placed(pose),side}
    }
}

/// The edge two adjacent patches of a profile loop share: the end `ends[i]` (0 or 1, in `u`) of
/// patch `patches[i]`, swept — a circle round a revolution, a line along a prism.
#[derive(Clone,Copy,Debug)]
pub struct PatchEdge {
    pub patches: [usize;2],
    pub ends: [f64;2],
}

