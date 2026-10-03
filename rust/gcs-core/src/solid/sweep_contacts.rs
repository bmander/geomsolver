//! The contacts of a continuous sweep's tool, read from an ordinary swept-solid definition.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,RevolvedSurface};
use crate::{envelope::{self,Contact,Error,Motion,SurfacePoint},model::{EntRef,Sketch,SolidDef,EntKind},motion::Family};
use crate::space::{add,scale,sub};
use std::{collections::BTreeSet,f64::consts::TAU};

type V = [f64;3];

/// One face of a sweep's tool: a revolution of a profile edge (`u` along the edge, `v` round the
/// turn, periodic), or a side of a prism (`u` along the edge, `v` along the extrusion, from one
/// cap to the other).
#[derive(Clone,Debug)]
pub enum ToolSurface { Revolved(RevolvedSurface),Extruded(ExtrudedSurface) }

impl ToolSurface {
    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        match self { ToolSurface::Revolved(s) => s.at(u,v),ToolSurface::Extruded(s) => s.at(u,v) }
    }
    pub fn name(&self) -> &str {
        match self { ToolSurface::Revolved(s) => &s.name,ToolSurface::Extruded(s) => &s.name }
    }
    /// The source parameters' domain.
    pub fn domain(&self) -> [[f64;2];2] {
        match self { ToolSurface::Revolved(s) => s.domain(),ToolSurface::Extruded(_) => [[0.,1.],[0.,1.]] }
    }
    /// Whether `v` runs round a turn and comes back (a full revolution) rather than from one cap
    /// to another.
    pub fn periodic(&self) -> bool { matches!(self,ToolSurface::Revolved(_)) }
    /// The revolution, where this face is one — what a revolution's closed forms are asked of.
    pub fn revolved(&self) -> Option<&RevolvedSurface> {
        match self { ToolSurface::Revolved(s) => Some(s),ToolSurface::Extruded(_) => None }
    }
}

/// A prism's side: a line, arc or circle of its profile carried along the profile's normal from
/// the prism's `from` to its `to`.  `at(u, v)` is the edge at `u` (its own direction, `p1` to `p2`
/// or round the arc) moved `v` of the way along.
#[derive(Clone,Debug)]
pub struct ExtrudedSurface { pub name: String,edge: Edge,depth: V }

#[derive(Clone,Copy,Debug)]
enum Edge { Line { start: V,delta: V },Round { center: V,a: V,b: V,sweep: f64 } }

impl ExtrudedSurface {
    /// Edge `edge` of prism `solid`'s profile, its face's lifted as the prism's recipe lifts it.
    pub fn read(sk: &Sketch,solid: usize,edge: EntRef) -> Result<Self,String> {
        let solid = sk.solids.get(solid).ok_or("no such solid")?;
        let SolidDef::Prism {face,from,to} = &solid.def else { return Err("an extruded patch requires a prism".into()) };
        let profile = sk.faces.get(*face as usize).ok_or("no such profile")?;
        let name = profile.edges.iter().zip(&profile.edge_names)
            .chain(profile.holes.iter().flat_map(|h| h.edges.iter().zip(&h.edge_names)))
            .find_map(|(e,n)| (*e == edge).then_some(n))
            .ok_or("the requested edge is not a boundary of this prism's profile")?;
        let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT).ok_or("invalid prism profile")?;
        let lift = |q: (f64,f64)| { let q = crate::plane::in_view(p.pose.0,p.pose.1,p.pose.2,q); p.basis.lift(q.0,q.1) };
        let vector = |q: (f64,f64)| {
            let r = (p.pose.0*q.0+p.pose.1*q.1,-p.pose.1*q.0+p.pose.0*q.1);
            add(scale(p.basis.u,r.0),scale(p.basis.v,r.1))
        };
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
                let (sn,cs) = start.dsin_cos();
                Edge::Round {center:add(lift(sk.point_xy(center as usize)),base),a:vector((r*cs,r*sn)),
                    b:vector((-r*sn,r*cs)),sweep}
            }
            _ => return Err("an extruded patch requires a line, arc or circle".into()),
        };
        let depth = scale(normal,to.value-from.value);
        if !(depth[0].dhypot(depth[1]).dhypot(depth[2]) > 0.) { return Err("a prism of no depth".into()) }
        Ok(ExtrudedSurface {name:format!("{}.{}",solid.name,name),edge,depth})
    }

    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
        let (p,d) = match self.edge {
            Edge::Line {start,delta} => (add(start,scale(delta,u)),delta),
            Edge::Round {center,a,b,sweep} => {
                let (sn,cs) = (u*sweep).dsin_cos();
                (add(center,add(scale(a,cs),scale(b,sn))),scale(add(scale(a,-sn),scale(b,cs)),sweep))
            }
        };
        Ok(SurfacePoint {position:add(p,scale(self.depth,v)),du:d,dv:self.depth})
    }

    fn placed(&self,pose: Motion) -> Self {
        let edge = match self.edge {
            Edge::Line {start,delta} => Edge::Line {start:pose.point(start),delta:pose.vector(delta)},
            Edge::Round {center,a,b,sweep} => Edge::Round {center:pose.point(center),a:pose.vector(a),b:pose.vector(b),sweep},
        };
        ExtrudedSurface {name:self.name.clone(),edge,depth:pose.vector(self.depth)}
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
    pub(super) motion: Family,
    pub(super) roll: [f64;2],
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
        let SolidDef::Swept {source,motion,from,to} = &sk.solids[solid].def else {
            return Err("contact construction requires a continuous swept solid".into());
        };
        let source_field = SpatialField::read(sk,*source as usize,axis_tolerance)?;
        let mut pending = vec![(*source as usize,Motion::identity())];
        let mut seen = BTreeSet::new();
        let mut patches = Vec::new();
        let mut loops = Vec::new();
        let mut edges: Vec<PatchEdge> = Vec::new();
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
                // a prism's sides, one an edge of its profile; its caps, square to the extrusion,
                // are admission's to keep out of the blank (`admission::check`)
                SolidDef::Prism {face,..} => {
                    for (edges,_) in sk.faces[*face as usize].boundaries() {
                        let mut order = Vec::with_capacity(edges.len());
                        for &edge in edges {
                            order.push(Some(patches.len()));
                            patches.push(ToolSurface::Extruded(ExtrudedSurface::read(sk,id,edge)?.placed(pose)));
                        }
                        loops.push(order);
                    }
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
        Ok(Self {source:source_field,patches,edges,motion:Family::read(sk,*motion as usize)?,
            roll:[from.value,to.value]})
    }
    pub fn edges(&self) -> &[PatchEdge] { &self.edges }
    pub fn patches(&self) -> &[ToolSurface] { &self.patches }
    pub fn motion(&self) -> &Family { &self.motion }
    pub fn source_material(&self) -> &SpatialField { &self.source }
    pub fn domain(&self) -> [f64;2] { self.roll }
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
