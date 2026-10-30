//! The contacts of a continuous sweep's tool, read from an ordinary swept-solid definition.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,RevolvedSurface};
use crate::{envelope::{self,Contact,Error,Motion,SurfacePoint},model::{EntRef,Sketch,SolidDef,EntKind},motion::Family};
use crate::space::{add,lerp,scale,sub};
use std::{collections::BTreeSet,f64::consts::TAU};

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
    /// Prism `solid`'s caps, its `start` then its `end`.
    pub fn read(sk: &Sketch,solid: usize) -> Result<[Self;2],String> {
        let s = sk.solids.get(solid).ok_or("no such solid")?;
        let SolidDef::Prism {face,from,to,draft:None} = &s.def else { return Err("a cap is read of a prism with square sides".into()) };
        let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT).ok_or("invalid prism profile")?;
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for &(x,y) in &p.pts { lo = [lo[0].min(x),lo[1].min(y)]; hi = [hi[0].max(x),hi[1].max(y)]; }
        if !(hi[0] > lo[0] && hi[1] > lo[1]) { return Err("a prism's profile of no area".into()) }
        // a little past the profile's points, which its arcs bulge beyond
        let pad = 0.02*(hi[0]-lo[0]).max(hi[1]-lo[1]);
        let (lo,hi) = (lo.map(|x| x-pad),hi.map(|x| x+pad));
        let normal = p.basis.normal();
        let (u,v) = (scale(p.basis.u,hi[0]-lo[0]),scale(p.basis.v,hi[1]-lo[1]));
        let at = |h: f64| add(p.basis.lift(lo[0],lo[1]),scale(normal,h));
        // `u × v` is the profile's normal: outward at the farther cap along it, and at the nearer
        // `v` runs back
        let cap = |h: f64,other: f64,end: bool| {
            let farther = h >= other;
            CapSurface {name:format!("{}.{}",s.name,if end { "end" } else { "start" }),end,
                corner:if farther { at(h) } else { add(at(h),v) },du:u,dv:if farther { v } else { scale(v,-1.) }}
        };
        Ok([cap(from.value,to.value,false),cap(to.value,from.value,true)])
    }

    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
        Ok(SurfacePoint {position:add(self.corner,add(scale(self.du,u),scale(self.dv,v))),du:self.du,dv:self.dv})
    }

    fn placed(&self,pose: Motion) -> Self {
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
pub struct ExtrudedSurface { pub name: String,edge: Edge,side: Side }

#[derive(Clone,Copy,Debug)]
enum Edge { Line { start: V,delta: V },Round { center: V,a: V,b: V,sweep: f64 } }

/// How a side runs from its edge at `from` to `to`: along the extrusion, or to its edge there.
#[derive(Clone,Copy,Debug)]
enum Side { Straight(V),Leaning(Edge) }

impl Edge {
    /// The arc about `center` of radius `r` from angle `start` through `sweep` (negative clockwise),
    /// in the plane whose axes `vector` lifts a pair of coordinates along.
    fn round(center: V,r: f64,start: f64,sweep: f64,vector: impl Fn(f64,f64) -> V) -> Edge {
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

    fn placed(&self,pose: Motion) -> Edge {
        match *self {
            Edge::Line {start,delta} => Edge::Line {start:pose.point(start),delta:pose.vector(delta)},
            Edge::Round {center,a,b,sweep} => Edge::Round {center:pose.point(center),a:pose.vector(a),b:pose.vector(b),sweep},
        }
    }
}

impl ExtrudedSurface {
    /// Edge `edge` of prism `solid`'s profile, its face's lifted as the prism's recipe lifts it.
    pub fn read(sk: &Sketch,solid: usize,edge: EntRef) -> Result<Self,String> {
        let solid = sk.solids.get(solid).ok_or("no such solid")?;
        let SolidDef::Prism {face,from,to,draft:None} = &solid.def else { return Err("an extruded patch requires a prism with square sides".into()) };
        let profile = sk.faces.get(*face as usize).ok_or("no such profile")?;
        let name = profile.boundaries().flat_map(|(e,n)| e.iter().zip(n))
            .find_map(|(e,n)| (*e == edge).then_some(n))
            .ok_or("the requested edge is not a boundary of this prism's profile")?;
        let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT).ok_or("invalid prism profile")?;
        let lift = |q: (f64,f64)| p.basis.lift(q.0,q.1);
        let vector = |x: f64,y: f64| add(scale(p.basis.u,x),scale(p.basis.v,y));
        let normal = p.basis.normal();
        let base = scale(normal,from.value);
        let edge = match edge.kind {
            EntKind::Line => {
                let line = sk.lines.get(edge.i()).ok_or("no such profile line")?;
                let start = lift(sk.point_xy(line.p1 as usize));
                Edge::Line {start:add(start,base),delta:sub(lift(sk.point_xy(line.p2 as usize)),start)}
            }
            EntKind::Arc | EntKind::Circle => {
                let (center,radius,start,sweep) = if edge.kind == EntKind::Arc {
                    let a = sk.arcs.get(edge.i()).ok_or("no such profile arc")?;
                    let (start,end) = sk.arc_angles(edge.i());
                    (a.center,a.radius,start,end-start)
                } else {
                    let c = sk.circles.get(edge.i()).ok_or("no such profile circle")?;
                    (c.center,c.radius,0.,TAU)
                };
                let r = sk.params[radius as usize].value;
                if !r.is_finite() || r <= 0. || !sweep.is_finite() || sweep <= 0. {
                    return Err("a round profile edge needs positive finite radius and sweep".into());
                }
                Edge::round(add(lift(sk.point_xy(center as usize)),base),r,start,sweep,vector)
            }
            _ => return Err("an extruded patch requires a line, arc or circle".into()),
        };
        let depth = scale(normal,to.value-from.value);
        if !(depth[0].dhypot(depth[1]).dhypot(depth[2]) > 0.) { return Err("a prism of no depth".into()) }
        Ok(ExtrudedSurface {name:format!("{}.{}",solid.name,name),edge,side:Side::Straight(depth)})
    }

    /// The sides of drafted prism `solid` (#684 G), half by half and loop by loop, each loop walked
    /// with its material on its left: each side the ruled surface between its edge in the section at
    /// one end of its half and in the section at the other, each section the profile moved `|h| tan
    /// draft` into its material (`field::Sections`, the section `build::drafted` builds). That is the
    /// side exactly where every point at a fixed fraction of its edge moves along a line as the draft
    /// deepens: where each corner joins two lines or goes on tangent (`Sections::linear`). A prism on
    /// one side of its parting plane is one half, from `from` to `to`; one running both ways from it
    /// (#684 P) two, from `from` to the plane and from the plane to `to`, side `k` of loop `l` of the
    /// first meeting the second's at the profile itself, a convex crease across the side whose fan
    /// turns its normal from the one half's to the other's. Refused otherwise, saying why.
    pub fn drafted(sk: &Sketch,solid: usize) -> Result<Vec<Vec<Vec<Self>>>,String> {
        use super::surface::Edge as Drawn;
        let solid = sk.solids.get(solid).ok_or("no such solid")?;
        let SolidDef::Prism {face,from,to,draft:Some(draft)} = &solid.def else {
            return Err("a drafted prism's sides require a drafted prism".into())
        };
        let (from,to) = (from.value,to.value);
        let parted = from.min(to) < 0. && from.max(to) > 0.;
        if parted && draft.value < 0. {
            return Err("it runs both ways from its profile's plane, drafted outward, so its sides meet there in a concave crease, \
                which trims the two halves' envelopes against each other".into())
        }
        let profile = sk.faces.get(*face as usize).ok_or("no such profile")?;
        let named: Vec<(&EntRef,&String)> = profile.boundaries().flat_map(|(e,n)| e.iter().zip(n)).collect();
        if let Some((_,n)) = named.iter().find(|(e,_)| !matches!(e.kind,EntKind::Line | EntKind::Arc | EntKind::Circle)) {
            return Err(format!("its side `{n}` is no line, arc or circle"))
        }
        // each drawn edge, a line, arc or circle, read as one loop edge in the profile's order
        let (basis,loops) = super::field::prism_loops(sk,*face as usize)?;
        let drawn: Vec<(&Drawn,&String)> = loops.iter().flatten().zip(named.iter().map(|(_,n)| *n)).collect();
        let sections = super::field::Sections::new(&loops,draft.value,[from,to]).map_err(|e| e.replace("the field reads ",""))?;
        if !sections.linear() {
            return Err("an arc meets a side at an angle, and a draft moves that corner along the curve its cone and the side \
                meet in, not along a line".into())
        }
        let normal = basis.normal();
        let lift = |q: [f64;2],h: f64| add(basis.lift(q[0],q[1]),scale(normal,h));
        let vector = |x: f64,y: f64| add(scale(basis.u,x),scale(basis.v,y));
        let edge = |e: &Drawn,h: f64| match *e {
            Drawn::Line {a,b,..} => Edge::Line {start:lift(a,h),delta:vector(b[0]-a[0],b[1]-a[1])},
            Drawn::Arc {center,radius,start,sweep,..} => Edge::round(lift(center,h),radius,start,sweep,vector),
            Drawn::Conic {..} => unreachable!("refused when read"),
        };
        // each side named by the drawn edge it walks: of its kind, through its ends, an arc about its centre
        let same = |p: [f64;2],q: [f64;2]| (p[0]-q[0]).dhypot(p[1]-q[1]) <= 1e-9*(1.+p[0].abs().max(p[1].abs()));
        let name = |e: &Drawn| drawn.iter().find(|(d,_)| {
            let ([a,b],[p,q]) = (e.endpoints(),d.endpoints());
            ((same(a,p) && same(b,q)) || (same(a,q) && same(b,p))) && match (e,d) {
                (Drawn::Line {..},Drawn::Line {..}) => true,
                (Drawn::Arc {center:c,..},Drawn::Arc {center:o,..}) => same(*c,*o),
                _ => false,
            }
        }).map(|(_,n)| n.as_str());
        // each half from one height to the other
        let halves = if parted { vec![[from,0.],[0.,to]] } else { vec![[from,to]] };
        halves.into_iter().map(|[h0,h1]| {
            let (near,far) = (sections.section_at(h0)?,sections.section_at(h1)?);
            sections.loops().iter().enumerate().map(|(l,lp)| lp.iter().enumerate().map(|(k,e)| {
                let name = name(e).ok_or("a drafted side is no drawn edge of its profile")?;
                Ok(ExtrudedSurface {name:format!("{}.{}",solid.name,name),edge:edge(&near[l][k],h0),
                    side:Side::Leaning(edge(&far[l][k],h1))})
            }).collect()).collect()
        }).collect()
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

    fn placed(&self,pose: Motion) -> Self {
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
    pub(super) source: SpatialField,
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

#[derive(Clone,Copy,Debug)]
pub struct TimedContact {
    pub root: crate::motion::ContactTime,
    pub contact: Contact,
}

impl SweepContacts {
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        super::validate(sk,solid)?;
        let SolidDef::Swept {source,motion,..} = &sk.solids[solid].def else {
            return Err("contact construction requires a continuous swept solid".into());
        };
        let source_field = SpatialField::read(sk,*source as usize,axis_tolerance)?;
        let mut pending = vec![(*source as usize,Motion::identity())];
        let mut seen = BTreeSet::new();
        let mut patches = Vec::new();
        let mut loops = Vec::new();
        let mut edges: Vec<PatchEdge> = Vec::new();
        let mut creases: Vec<[usize;2]> = Vec::new();
        while let Some((id,pose)) = pending.pop() {
            let columns: [[u64;3];4] = std::array::from_fn(|i| {
                let p = if i == 3 { pose.point([0.;3]) } else {
                    let mut p = [0.;3]; p[i] = 1.; pose.vector(p)
                }; p.map(f64::to_bits)
            });
            if !seen.insert((id,columns)) { continue; }
            match &sk.solids[id].def {
                SolidDef::Revolve {face,..} => {
                    for (edges,_) in sk.faces[*face as usize].boundaries() {
                        let mut order = Vec::with_capacity(edges.len());
                        for &edge in edges {
                            let surface = RevolvedSurface::read(sk,id,edge)?.placed(pose);
                            let surface = ToolSurface::Revolved(surface);
                            if edge.kind == EntKind::Line {
                                let radius = |u| surface.at(u,0.).map(|p|
                                    p.dv[0].dhypot(p.dv[1]).dhypot(p.dv[2])/TAU);
                                // A diameter on the axis disappears in a full revolution.
                                if radius(0.).map_err(|e| format!("{e:?}"))? <= axis_tolerance
                                    && radius(1.).map_err(|e| format!("{e:?}"))? <= axis_tolerance { order.push(None); continue; }
                            }
                            order.push(Some(patches.len()));
                            patches.push(surface);
                        }
                        loops.push(order);
                    }
                }
                // a region's faces, each step of its meridian turned (§6.21): its profile is the
                // meridian, read as a revolution's
                SolidDef::Region {..} => {
                    let m = super::region::meridian(sk,id)?;
                    for lp in RevolvedSurface::meridian_loops(&m,&sk.solids[id].name) {
                        loops.push(lp.into_iter().map(|s| s.map(|s| {
                            patches.push(ToolSurface::Revolved(s.placed(pose)));
                            patches.len()-1
                        })).collect());
                    }
                }
                // a drafted prism's sides, each a plane or a cone between its two sections (#684 G),
                // and where it runs both ways from its parting plane each side's halves' crease there
                // (#684 P); its caps are admission's to keep out of the blank (`admission::ends`)
                SolidDef::Prism {draft:Some(_),..} => {
                    let (halves,start) = (ExtrudedSurface::drafted(sk,id)?,patches.len());
                    for sides in halves.iter().flatten() {
                        loops.push(sides.iter().map(|side| {
                            patches.push(ToolSurface::Extruded(side.placed(pose)));
                            Some(patches.len()-1)
                        }).collect());
                    }
                    // the halves side for side alike, the second's sides the first's as many on
                    if halves.len() == 2 {
                        let n = (patches.len()-start)/2;
                        creases.extend((start..start+n).map(|a| [a,a+n]));
                    }
                }
                // a prism's sides, one an edge of its profile, then its caps, square to the
                // extrusion, which admission asks only where one reaches the blank (#684 F)
                SolidDef::Prism {face,..} => {
                    for (edges,_) in sk.faces[*face as usize].boundaries() {
                        let mut order = Vec::with_capacity(edges.len());
                        for &edge in edges {
                            order.push(Some(patches.len()));
                            patches.push(ToolSurface::Extruded(ExtrudedSurface::read(sk,id,edge)?.placed(pose)));
                        }
                        loops.push(order);
                    }
                    patches.extend(CapSurface::read(sk,id)?.map(|c| ToolSurface::Cap(c.placed(pose))));
                }
                SolidDef::Placed {source,motion,at} => pending.push((*source as usize,
                    Family::read(sk,*motion as usize)?.at(at.value)?.then(pose))),
                SolidDef::Body {..} => pending.extend(sk.solids[id].operands().into_iter()
                    .rev().map(|id| (id as usize,pose))),
                _ => return Err("contact construction requires static revolutions, prisms and Boolean operands".into()),
            }
        }
        // Adjacent patches of a loop that share a meridian end share the circle that end sweeps.
        for profile in &loops {
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
        // a whole turn or more sweeps what one whole turn does, a solid of revolution (#684 N): every
        // reader of its contacts reads it once round
        let (motion,roll) = (Family::read(sk,*motion as usize)?,super::roll::interval(sk,solid)?);
        let once = motion.twist().and_then(|screw| super::constant_twist::once_round(&screw,roll));
        Ok(Self {source:source_field,patches,edges,creases,motion,roll:once.unwrap_or(roll),whole:once.is_some()})
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
    pub fn source_material(&self) -> &SpatialField { &self.source }
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
