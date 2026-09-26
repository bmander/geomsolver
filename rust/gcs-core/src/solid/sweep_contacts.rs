//! The contacts of a continuous sweep's tool, read from an ordinary swept-solid definition.
use super::{SpatialField,RevolvedSurface};
use crate::{envelope::{self,Contact,Error,Motion},model::{Sketch,SolidDef,EntKind},motion::Family};
use std::{collections::BTreeSet,f64::consts::TAU};

/// The revolved faces of a continuous sweep's tool and the edges between its faces, with the
/// motion that carries them. Every source face is retained, including faces hidden by source
/// Booleans; the source material is what rejects hidden portions.
#[derive(Clone,Debug)]
pub struct SweepContacts {
    pub(super) source: SpatialField,
    pub(super) patches: Vec<RevolvedSurface>,
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

/// The circle two adjacent patches of a revolved profile loop share: the meridian end `ends[i]`
/// (0 or 1) of patch `patches[i]`, swept.
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
                            if edge.kind == EntKind::Line {
                                let radius = |u| surface.at(u,0.).map(|p|
                                    p.dv[0].hypot(p.dv[1]).hypot(p.dv[2])/TAU);
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
                SolidDef::Placed {source,motion,at} => pending.push((*source as usize,
                    Family::read(sk,*motion as usize)?.at(at.value)?.then(pose))),
                SolidDef::Body {..} => pending.extend(sk.solids[id].operands().into_iter()
                    .rev().map(|id| (id as usize,pose))),
                _ => return Err("contact construction requires static revolutions and Boolean operands".into()),
            }
        }
        // Adjacent patches of a loop that share a meridian end share the circle that end sweeps.
        for profile in &loops {
            let n = profile.len();
            for k in 0..n {
                let (Some(a),Some(b)) = (profile[k],profile[(k+1)%n]) else { continue; };
                let ends = |p: &RevolvedSurface| -> Result<[[f64;3];2],String> {
                    Ok([p.at(0.,0.).map_err(|e| format!("{e:?}"))?.position,p.at(1.,0.).map_err(|e| format!("{e:?}"))?.position])
                };
                let (ea,eb) = (ends(&patches[a])?,ends(&patches[b])?);
                let scale = ea.iter().chain(&eb).map(|p| p[0].hypot(p[1]).hypot(p[2])).fold(1_f64,f64::max);
                let close = |p: [f64;3],q: [f64;3]| (0..3).map(|k| (p[k]-q[k]).powi(2)).sum::<f64>().sqrt() < 1e-7*scale;
                let Some((ua,ub)) = [(1.,0.),(1.,1.),(0.,0.),(0.,1.)].into_iter()
                    .find(|&(ua,ub)| close(ea[ua as usize],eb[ub as usize])) else { continue; };
                edges.push(PatchEdge {patches:[a,b],ends:[ua,ub]});
            }
        }
        Ok(Self {source:source_field,patches,edges,motion:Family::read(sk,*motion as usize)?,
            roll:[from.value,to.value]})
    }
    pub fn edges(&self) -> &[PatchEdge] { &self.edges }
    pub fn patches(&self) -> &[RevolvedSurface] { &self.patches }
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
