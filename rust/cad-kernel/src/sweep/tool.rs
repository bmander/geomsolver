//! The contacts of a continuous sweep's tool, read from an ordinary swept-solid definition.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::surface::RevolvedSurface;
use crate::sweep::{envelope::{self,Contact,Error,Motion,SurfacePoint},motion::Family};
use crate::util::space::{add,dot,lerp,norm,scale,sub};
use std::f64::consts::TAU;
use std::sync::Arc;

/// **A tool's material**: its signed distance at a point, negative inside — what tells a tool
/// point on its own boundary from one a Boolean hid, and a convex corner from a concave one.
/// The caller's (a field it built from its tool).
pub trait Material: std::fmt::Debug+Send+Sync {
    /// The signed distance at `p`, negative inside.
    fn value(&self,p: [f64;3]) -> f64;
}

/// **Where a caller's tools are read from**: its document, as it now stands. What
/// `SweepContacts::read`, `ExtrudedSurface::read` and `drafted`, and `CapSurface::read` ask.
pub trait Source {
    /// How the document names an edge of a profile.
    type Edge;
    /// Continuous sweep `solid`'s contacts.
    fn contacts(&self,solid: usize,axis_tolerance: f64) -> Result<SweepContacts,String>;
    /// Edge `edge` of prism `solid`'s profile carried along its extrusion.
    fn extruded(&self,solid: usize,edge: Self::Edge) -> Result<ExtrudedSurface,String>;
    /// Drafted prism `solid`'s sides, half by half and loop by loop.
    fn drafted(&self,solid: usize) -> Result<Vec<Vec<Vec<ExtrudedSurface>>>,String>;
    /// Prism `solid`'s caps, its `start` then its `end`.
    fn caps(&self,solid: usize) -> Result<[CapSurface;2],String>;
}

type V = [f64;3];

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
    /// Prism `solid`'s caps in `source`, its `start` then its `end` (`Source::caps`).
    pub fn read<S: Source+?Sized>(source: &S,solid: usize) -> Result<[Self;2],String> { source.caps(solid) }

    /// The cap of a prism named `name` (`end` its `end`, at `to`; else its `start`): the plane
    /// through `corner` spanned by `du` and `dv`, read over the parallelogram they make, `du × dv`
    /// its outward normal.
    pub fn new(name: String,end: bool,corner: V,du: V,dv: V) -> Self { CapSurface {name,end,corner,du,dv} }

    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
        Ok(SurfacePoint {position:add(self.corner,add(scale(self.du,u),scale(self.dv,v))),du:self.du,dv:self.dv})
    }

    /// The cap carried by the fixed placement `pose`.
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
/// (`SweepContacts::creases`).
#[derive(Clone,Debug)]
pub struct ExtrudedSurface { pub name: String,edge: ProfileEdge,side: Side }

/// An edge of a prism's profile in space: a line from `start` along `delta`, or an arc about
/// `center` from `a` towards `b` (each a radius) through `sweep` radians.
#[derive(Clone,Copy,Debug)]
pub enum ProfileEdge { Line { start: V,delta: V },Round { center: V,a: V,b: V,sweep: f64 } }

/// How a side runs from its edge at `from` to `to`: along the extrusion, or to its edge there.
#[derive(Clone,Copy,Debug)]
pub enum Side { Straight(V),Leaning(ProfileEdge) }

impl ProfileEdge {
    /// The arc about `center` of radius `r` from angle `start` through `sweep` (negative clockwise),
    /// in the plane whose axes `vector` lifts a pair of coordinates along.
    pub fn round(center: V,r: f64,start: f64,sweep: f64,vector: impl Fn(f64,f64) -> V) -> ProfileEdge {
        let (sn,cs) = start.dsin_cos();
        let turn = sweep.signum();
        ProfileEdge::Round {center,a:vector(r*cs,r*sn),b:vector(-turn*r*sn,turn*r*cs),sweep:sweep.abs()}
    }

    /// The edge's point at fraction `u` of it, and its derivative in `u`.
    fn at(&self,u: f64) -> (V,V) {
        match *self {
            ProfileEdge::Line {start,delta} => (add(start,scale(delta,u)),delta),
            ProfileEdge::Round {center,a,b,sweep} => {
                let (sn,cs) = (u*sweep).dsin_cos();
                (add(center,add(scale(a,cs),scale(b,sn))),scale(add(scale(a,-sn),scale(b,cs)),sweep))
            }
        }
    }

    /// The edge carried by the fixed placement `pose`.
    pub fn placed(&self,pose: Motion) -> ProfileEdge {
        match *self {
            ProfileEdge::Line {start,delta} => ProfileEdge::Line {start:pose.point(start),delta:pose.vector(delta)},
            ProfileEdge::Round {center,a,b,sweep} => ProfileEdge::Round {center:pose.point(center),a:pose.vector(a),b:pose.vector(b),sweep},
        }
    }
}

impl ExtrudedSurface {
    /// The side named `name`: the edge `edge` carried to `side`.
    pub fn new(name: String,edge: ProfileEdge,side: Side) -> Self { ExtrudedSurface {name,edge,side} }

    /// Edge `edge` of prism `solid`'s profile in `source` (`Source::extruded`).
    pub fn read<S: Source+?Sized>(source: &S,solid: usize,edge: S::Edge) -> Result<Self,String> {
        source.extruded(solid,edge)
    }

    /// The sides of drafted prism `solid` in `source`, half by half and loop by loop
    /// (`Source::drafted`).
    pub fn drafted<S: Source+?Sized>(source: &S,solid: usize) -> Result<Vec<Vec<Vec<Self>>>,String> {
        source.drafted(solid)
    }

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

    /// The side carried by the fixed placement `pose`.
    pub fn placed(&self,pose: Motion) -> Self {
        let side = match self.side {
            Side::Straight(depth) => Side::Straight(pose.vector(depth)),
            Side::Leaning(far) => Side::Leaning(far.placed(pose)),
        };
        ExtrudedSurface {name:self.name.clone(),edge:self.edge.placed(pose),side}
    }
}

/// The revolved faces of a continuous sweep's tool and the edges between its faces, with the
/// motion that carries them. Every source face is retained, including faces hidden by source
/// Booleans; the source material is what rejects hidden portions.
#[derive(Clone,Debug)]
pub struct SweepContacts {
    pub(super) source: Arc<dyn Material>,
    pub(super) patches: Vec<ToolSurface>,
    /// Where adjacent patches of a profile loop meet.
    pub(super) edges: Vec<PatchEdge>,
    /// Where a drafted prism's sides fold at its parting plane (#684 P): the side below it at its
    /// `v = 1`, then the side above at its `v = 0`, one point at each `u`.
    pub(super) creases: Vec<[usize;2]>,
    pub(super) motion: Family,
    /// The roll it is read over: the sweep's, or a whole turn's once round (`whole`).
    pub(super) roll: [f64;2],
    /// Whether the motion is a turn the sweep's roll carries round a whole turn or more (#684 N).
    pub(super) whole: bool,
}

/// Why a point carrying a normal has no contact times: its normal is no direction, its contact
/// equation is degenerate (in contact at every time, or only grazing, which the root finder
/// cannot tell apart), or something failed that says nothing about the point.
#[derive(Clone,Debug,PartialEq)]
pub enum PointContactError { DegenerateNormal, Degenerate, Failed(String) }

impl PointContactError {
    fn of(e: Error) -> Self {
        if e == Error::Degenerate { PointContactError::Degenerate } else { PointContactError::Failed(format!("{e:?}")) }
    }
}

impl std::fmt::Display for PointContactError {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PointContactError::DegenerateNormal => f.write_str("degenerate normal"),
            PointContactError::Degenerate => f.write_str("Degenerate"),
            PointContactError::Failed(m) => f.write_str(m),
        }
    }
}

/// The edge two adjacent patches of a profile loop share: the end `ends[i]` (0 or 1, in `u`) of
/// patch `patches[i]`, swept — a circle round a revolution, a line along a prism.
#[derive(Clone,Copy,Debug)]
pub struct PatchEdge {
    pub patches: [usize;2],
    pub ends: [f64;2],
}

impl PatchEdge {
    /// The edges adjacent `patches` of each loop of a profile share (each loop in its order, a face
    /// on the axis that turns to nothing `None`): where two meet at a meridian end, the circle that
    /// end sweeps.
    pub fn of_loops(patches: &[ToolSurface],loops: &[Vec<Option<usize>>]) -> Result<Vec<PatchEdge>,String> {
        let mut edges: Vec<PatchEdge> = Vec::new();
        for profile in loops {
            let n = profile.len();
            for k in 0..n {
                let (Some(a),Some(b)) = (profile[k],profile[(k+1)%n]) else { continue; };
                let ends = |p: &ToolSurface| -> Result<[[f64;3];2],String> {
                    Ok([p.at(0.,0.).map_err(|e| format!("{e:?}"))?.position,p.at(1.,0.).map_err(|e| format!("{e:?}"))?.position])
                };
                let (ea,eb) = (ends(&patches[a])?,ends(&patches[b])?);
                let scale = ea.iter().chain(&eb).map(|p| p[0].dhypot(p[1]).dhypot(p[2])).fold(1_f64,f64::max);
                let close = |p: [f64;3],q: [f64;3]| (0..3).map(|k| (p[k]-q[k]).dpowi(2)).sum::<f64>().sqrt() < 1e-7*scale;
                let Some((ua,ub)) = [(1.,0.),(1.,1.),(0.,0.),(0.,1.)].into_iter()
                    .find(|&(ua,ub)| close(ea[ua as usize],eb[ub as usize])) else { continue; };
                edges.push(PatchEdge {patches:[a,b],ends:[ua,ub]});
            }
        }
        Ok(edges)
    }
}

#[derive(Clone,Copy,Debug)]
pub struct TimedContact {
    pub root: crate::sweep::motion::ContactTime,
    pub contact: Contact,
}

impl SweepContacts {
    /// The contacts of the tool's faces `patches`, the edges where adjacent ones meet (`edges`,
    /// `PatchEdge::of_loops`) and a drafted prism's `creases`, its material `source`, under
    /// `motion` over `roll`; `whole` where `roll` is a whole turn of a longer one read once round.
    pub fn new(source: Arc<dyn Material>,patches: Vec<ToolSurface>,edges: Vec<PatchEdge>,creases: Vec<[usize;2]>,
        motion: Family,roll: [f64;2],whole: bool) -> Self {
        Self {source,patches,edges,creases,motion,roll,whole}
    }

    /// Read continuous sweep `solid`'s contacts off `source` (`Source::contacts`).
    pub fn read<S: Source+?Sized>(source: &S,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        source.contacts(solid,axis_tolerance)
    }
    pub fn edges(&self) -> &[PatchEdge] { &self.edges }
    /// Where `edge`'s two faces meet, and a point just inside each face beside it, on their charts'
    /// start (`v = 0`): the probe a corner is told by — the chord between the two points in the
    /// tool's material at a convex corner, out of it at a concave one.
    pub fn corner(&self,edge: &PatchEdge) -> Option<(V,[V;2])> {
        let ([a,b],[ua,ub]) = (edge.patches,edge.ends);
        let (pa,pb) = (self.patches.get(a)?,self.patches.get(b)?);
        let h = 1e-4;
        let inward = |u: f64| if u < 0.5 { u+h } else { u-h };
        Some((pa.at(ua,0.).ok()?.position,[pa.at(inward(ua),0.).ok()?.position,pb.at(inward(ub),0.).ok()?.position]))
    }
    pub fn patches(&self) -> &[ToolSurface] { &self.patches }
    /// Where a drafted prism's sides fold at its parting plane (#684 P): each pair the side below
    /// it, ending there at `v = 1`, and the side above, starting there at `v = 0`.
    pub fn creases(&self) -> &[[usize;2]] { &self.creases }
    pub fn motion(&self) -> &Family { &self.motion }
    pub fn source_material(&self) -> &dyn Material { &*self.source }
    /// The tool's material, shared.
    pub fn material(&self) -> &Arc<dyn Material> { &self.source }
    /// The roll the contacts are read over: the sweep's, or a whole turn's once round from its start.
    pub fn domain(&self) -> [f64;2] { self.roll }
    /// Whether the sweep is a whole turn or more, read once round (`domain`): it has no ends, the
    /// tool at either lying within what it sweeps (#684 N).
    pub fn whole(&self) -> bool { self.whole }
    /// Contact times of a point carrying a given unit normal, over a motion
    /// interval. This is the sharp-edge sweep condition: a convex source edge
    /// carries every normal between its incident faces', and each such normal
    /// contacts where it is perpendicular to the point's velocity. The caller
    /// supplies the normal; nothing here checks that the edge actually carries it.
    pub fn at_point_normal_over(&self,position: [f64;3],normal: [f64;3],interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,PointContactError> {
        let n = envelope::normalized(normal).ok_or(PointContactError::DegenerateNormal)?;
        let seed = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let du = envelope::normalized(crate::plane::cross(seed,n)).ok_or(PointContactError::DegenerateNormal)?;
        let dv = crate::plane::cross(n,du);
        let surface = envelope::SurfacePoint {position,du,dv};
        let roots = self.motion.normal_velocity(surface).map_err(PointContactError::Failed)?
            .roots(interval,tolerance,4096).map_err(PointContactError::of)?;
        roots.into_iter().map(|root| {
            let contact = envelope::contact(surface,self.motion.at(root.time).map_err(PointContactError::Failed)?)
                .map_err(PointContactError::of)?;
            if contact.normal_velocity.abs() > tolerance {
                return Err(PointContactError::Failed("point-normal contact failed the normal-velocity equation".into()));
            }
            Ok(TimedContact {root,contact})
        }).collect()
    }

    /// The same chart over a caller-supplied motion interval. Contacts outside the
    /// declared roll lie on the envelope of a longer motion, not on this sweep's
    /// boundary; a caller may use them as an extended candidate sheet only where a
    /// separate material classification decides what is exposed.
    pub fn at_source_over(&self,patch: usize,u: f64,v: f64,interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,String> {
        let surface = self.patches.get(patch).ok_or("no such sweep patch")?.at(u,v)
            .map_err(|e| format!("{e:?}"))?;
        let roots = self.motion.normal_velocity(surface)?.roots(interval,tolerance,4096)
            .map_err(|e| format!("{e:?}"))?;
        roots.into_iter().map(|root| {
            let contact = envelope::contact(surface,self.motion.at(root.time)?)
                .map_err(|e| format!("{e:?}"))?;
            if contact.normal_velocity.abs() > tolerance {
                return Err("temporal contact failed the original normal-velocity equation".into());
            }
            Ok(TimedContact {root,contact})
        }).collect()
    }
}

/// One primitive of a tool as a reader made its faces: a full revolution's, loop by loop in its
/// profile's order, a line on its axis (which turns to no face) `None`; a prism's sides loop by
/// loop, and its caps (its `start`, then its `end`); or a drafted prism's sides, half by half and
/// loop by loop — the faces a sweep's contacts are read over, before any placement.
#[derive(Clone,Debug)]
pub enum Faces {
    Revolved(Vec<Vec<Option<RevolvedSurface>>>),
    Prism { sides: Vec<Vec<ExtrudedSurface>>,caps: [CapSurface;2] },
    Drafted(Vec<Vec<Vec<ExtrudedSurface>>>),
}

/// **A sweep's tool**, as plain data: a profile of lines and arcs (`build::Profile`) turned once
/// about a line in its plane, through `origin` along `axis`; a profile swept along its normal from
/// `from` to `to` (ordinates along it from the profile's plane), its sides leaning `draft` radians
/// into its material away from that plane where there is one (`build::drafted`'s); a union or an
/// intersection of tools; a tool carried by a fixed placement (`placed`); or faces a caller's own
/// reader made (`Read`), the same faces met twice under one placement read once. `name` names its
/// faces (`name.edge`, by the profile's edge names, `edge{k}` where it has none), as a refusal or
/// a cut face says them.
#[derive(Clone,Debug)]
pub enum Tool {
    Revolved { name: String,profile: crate::build::Profile,origin: V,axis: V },
    Prism { name: String,profile: crate::build::Profile,from: f64,to: f64,draft: Option<f64> },
    Union(Vec<Tool>),
    Intersection(Vec<Tool>),
    Placed(Motion,Box<Tool>),
    Read(Arc<Faces>),
}

/// What a tool's walk gathers: its faces placed, each profile loop's faces in order (a line on an
/// axis `None`), and a drafted prism's creases.
#[derive(Default)]
struct Gathered { patches: Vec<ToolSurface>,loops: Vec<Vec<Option<usize>>>,creases: Vec<[usize;2]> }

impl Tool {
    /// `tool` carried by the rigid placement `m`.
    pub fn placed(m: &crate::geom::Rigid,tool: Tool) -> Tool {
        Tool::Placed(Motion::of_parts(m.r,[[0.;3];3],m.t,[0.;3]),Box::new(tool))
    }

    /// A primitive's faces, before any placement: a revolution's, a prism's, or what a reader made.
    pub fn faces(&self) -> Result<Arc<Faces>,String> {
        use crate::build::{walks,offset};
        use crate::geom::{Curve,Frame};
        // each step of a walk as a revolution's generating edge or a prism's profile edge
        let profile_edge = |s: &crate::build::Seg| -> Result<ProfileEdge,String> {
            match &s.curve {
                Curve::Line {..} => Ok(ProfileEdge::Line {start:s.start(),delta:sub(s.end(),s.start())}),
                Curve::Circle(f,r) => {
                    let (t0,t1) = (s.t[0],s.t[1]);
                    let turn = (t1-t0).signum();
                    let (sn,cs) = t0.dsin_cos();
                    let a = add(scale(f.x,r*cs),scale(f.y,r*sn));
                    let b = scale(add(scale(f.x,-r*sn),scale(f.y,r*cs)),turn);
                    Ok(ProfileEdge::Round {center:f.o,a,b,sweep:(t1-t0).abs()})
                }
                _ => Err("a tool's profile is lines and arcs: a spline's sweep is not read".into()),
            }
        };
        let named = |name: &str,p: &crate::build::Profile,l: usize,k: usize| {
            let edge = p.name(l,k);
            let edge = if !edge.is_empty() { edge } else if l == 0 { format!("edge{k}") } else { format!("loop{l}.edge{k}") };
            if name.is_empty() { edge } else { format!("{name}.{edge}") }
        };
        match self {
            Tool::Read(faces) => Ok(faces.clone()),
            Tool::Revolved {name,profile,origin,axis} => {
                let a = crate::util::space::unit(*axis);
                let normal = crate::util::space::unit(profile.normal);
                if !(norm(a) > 0.) || dot(a,normal).abs() > 1e-9 { return Err(format!("`{name}`: a revolution's axis lies in its profile's plane")) }
                let plane = Frame::about(profile.origin,normal);
                let coords = |q: V| { let l = plane.local(q); [l[0],l[1]] };
                let off_axis = |q: V| { let d = sub(q,*origin); norm(sub(d,scale(a,dot(d,a)))) };
                let loops = walks(profile,&coords)?.iter().enumerate().map(|(l,walk)| walk.iter().map(|s| {
                    if matches!(s.curve,Curve::Line {..}) && off_axis(s.start()) <= super::AXIS_TOLERANCE
                        && off_axis(s.end()) <= super::AXIS_TOLERANCE { return Ok(None) }
                    let meridian = match profile_edge(s)? {
                        ProfileEdge::Line {start,delta} => super::surface::Meridian::Line {start,delta},
                        ProfileEdge::Round {center,a,b,sweep} => super::surface::Meridian::Round {center,a,b,sweep},
                    };
                    Ok(Some(RevolvedSurface::new(named(name,profile,l,s.source),meridian,*origin,a,TAU)))
                }).collect::<Result<Vec<_>,String>>()).collect::<Result<Vec<_>,_>>()?;
                Ok(Arc::new(Faces::Revolved(loops)))
            }
            Tool::Prism {name,profile,from,to,draft} => {
                let n = crate::util::space::unit(profile.normal);
                let plane = Frame::about(profile.origin,n);
                let coords = |q: V| { let l = plane.local(q); [l[0],l[1]] };
                let depth = scale(n,to-from);
                if !(norm(depth) > 0.) { return Err(format!("`{name}`: a prism of no depth")) }
                let lifted = |e: ProfileEdge,h: f64| e.placed(Motion::of_parts([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],[[0.;3];3],
                    scale(n,h),[0.;3]));
                let walked = walks(profile,&coords)?;
                if let Some(draft) = draft.filter(|d| *d != 0.) {
                    // each half from one ordinate to the other, its sides between the profile moved
                    // `|h| tan draft` into its material at each (`build::offset`), side for side
                    let (lo,hi) = (from.min(*to),from.max(*to));
                    let halves = if lo < 0. && hi > 0. { vec![[lo,0.],[0.,hi]] } else { vec![[lo,hi]] };
                    let section = |h: f64| -> Result<Vec<Vec<crate::build::Seg>>,String> {
                        walks(&offset(profile,-h.abs()*draft.dtan())?,&coords)
                    };
                    let halves = halves.into_iter().map(|[h0,h1]| {
                        let (near,far) = (section(h0)?,section(h1)?);
                        if near.len() != walked.len() || far.len() != walked.len()
                            || (0..walked.len()).any(|l| near[l].len() != walked[l].len() || far[l].len() != walked[l].len()) {
                            return Err(format!("`{name}`: its draft takes a side of its profile to nothing"))
                        }
                        (0..walked.len()).map(|l| (0..walked[l].len()).map(|k| Ok(ExtrudedSurface::new(named(name,profile,l,walked[l][k].source),
                            lifted(profile_edge(&near[l][k])?,h0),Side::Leaning(lifted(profile_edge(&far[l][k])?,h1))))).collect()).collect()
                    }).collect::<Result<Vec<_>,String>>()?;
                    return Ok(Arc::new(Faces::Drafted(halves)))
                }
                let sides = walked.iter().enumerate().map(|(l,walk)| walk.iter().map(|s| Ok(ExtrudedSurface::new(named(name,profile,l,s.source),
                    lifted(profile_edge(s)?,*from),Side::Straight(depth)))).collect()).collect::<Result<Vec<_>,String>>()?;
                // the caps over the box the profile lies in, a little past its points, `u × v` the
                // profile's normal: outward at the farther cap along it, and at the nearer `v` runs back
                let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
                for s in walked.iter().flatten() { for j in 0..=64 {
                    let q = coords(s.curve.point(s.t[0]+(s.t[1]-s.t[0])*j as f64/64.));
                    lo = [lo[0].min(q[0]),lo[1].min(q[1])]; hi = [hi[0].max(q[0]),hi[1].max(q[1])];
                } }
                let pad = 0.02*(hi[0]-lo[0]).max(hi[1]-lo[1]);
                let (lo,hi) = (lo.map(|x| x-pad),hi.map(|x| x+pad));
                let (u,v) = (scale(plane.x,hi[0]-lo[0]),scale(plane.y,hi[1]-lo[1]));
                let at = |h: f64| plane.at([lo[0],lo[1],h]);
                let cap = |h: f64,other: f64,end: bool| {
                    let farther = h >= other;
                    let edge = if end { "end" } else { "start" };
                    CapSurface::new(if name.is_empty() { edge.to_string() } else { format!("{name}.{edge}") },end,
                        if farther { at(h) } else { add(at(h),v) },u,if farther { v } else { scale(v,-1.) })
                };
                Ok(Arc::new(Faces::Prism {sides,caps:[cap(*from,*to,false),cap(*to,*from,true)]}))
            }
            Tool::Union(_) | Tool::Intersection(_) | Tool::Placed(..) => Err("a Boolean or a placement is no primitive".into()),
        }
    }

    /// Every primitive's faces, placed as the tool's placements put them, depth first in the
    /// order written: a reader's faces met again under the same placement are not read twice.
    fn gather(&self,pose: Motion,seen: &mut Vec<(*const Faces,[[u64;3];4])>,out: &mut Gathered) -> Result<(),String> {
        match self {
            Tool::Union(tools) | Tool::Intersection(tools) => tools.iter().try_for_each(|t| t.gather(pose,seen,out)),
            Tool::Placed(m,tool) => tool.gather(m.then(pose),seen,out),
            _ => {
                let faces = self.faces()?;
                if let Tool::Read(_) = self {
                    let columns: [[u64;3];4] = std::array::from_fn(|i| {
                        let p = if i == 3 { pose.point([0.;3]) } else { let mut p = [0.;3]; p[i] = 1.; pose.vector(p) };
                        p.map(f64::to_bits)
                    });
                    let key = (Arc::as_ptr(&faces),columns);
                    if seen.contains(&key) { return Ok(()) }
                    seen.push(key);
                }
                match &*faces {
                    Faces::Revolved(loops) => for lp in loops {
                        out.loops.push(lp.iter().map(|s| s.as_ref().map(|s| {
                            out.patches.push(ToolSurface::Revolved(s.placed(pose)));
                            out.patches.len()-1
                        })).collect());
                    },
                    // a prism's sides, one an edge of its profile, then its caps, square to the
                    // extrusion, which admission asks only where one reaches the blank (#684 F)
                    Faces::Prism {sides,caps} => {
                        for lp in sides {
                            out.loops.push(lp.iter().map(|s| {
                                out.patches.push(ToolSurface::Extruded(s.placed(pose)));
                                Some(out.patches.len()-1)
                            }).collect());
                        }
                        out.patches.extend(caps.iter().map(|c| ToolSurface::Cap(c.placed(pose))));
                    }
                    // a drafted prism's sides, each a plane or a cone between its two sections
                    // (#684 G), and where it runs both ways from its parting plane each side's
                    // halves' crease there (#684 P); its caps are admission's to keep out of the blank
                    Faces::Drafted(halves) => {
                        let start = out.patches.len();
                        for sides in halves.iter().flatten() {
                            out.loops.push(sides.iter().map(|side| {
                                out.patches.push(ToolSurface::Extruded(side.placed(pose)));
                                Some(out.patches.len()-1)
                            }).collect());
                        }
                        // the halves side for side alike, the second's sides the first's as many on
                        if halves.len() == 2 {
                            let n = (out.patches.len()-start)/2;
                            out.creases.extend((start..start+n).map(|a| [a,a+n]));
                        }
                    }
                }
                Ok(())
            }
        }
    }
}

impl SweepContacts {
    /// The contacts of `tool` (`material` its material, `Tool::material` where the tool is plain
    /// data) under `motion` over `roll`: every primitive's faces placed, the edges where adjacent
    /// faces of a profile loop meet, a drafted prism's creases — and a turn carried round a whole
    /// turn or more read once round (#684 N), as it sweeps what one whole turn does.
    pub fn of_tool(tool: &Tool,material: Arc<dyn Material>,motion: Family,roll: [f64;2]) -> Result<Self,String> {
        let mut gathered = Gathered::default();
        tool.gather(Motion::identity(),&mut Vec::new(),&mut gathered)?;
        let Gathered {patches,loops,creases} = gathered;
        let edges = PatchEdge::of_loops(&patches,&loops)?;
        let once = motion.twist().and_then(|screw| super::characteristic::once_round(&screw,roll));
        Ok(SweepContacts::new(material,patches,edges,creases,motion,once.unwrap_or(roll),once.is_some()))
    }
}
