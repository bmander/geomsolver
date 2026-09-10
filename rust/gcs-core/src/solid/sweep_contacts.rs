//! Candidate envelope faces read from an ordinary swept-solid definition.
use super::{SpatialField,RevolvedSurface,RevolvedContact,ToolFace,ToolEdge,EdgeChart};
use crate::{envelope::{self,Contact,Error,Motion},model::{Sketch,SolidDef,EntKind},motion::Family};
use std::{collections::BTreeSet,f64::consts::TAU};
mod cover;
mod curves;
pub use curves::{ContactCurve,ContactCurves,ContactCurvePoint};
mod transitions;
pub use transitions::ContactTransition;
mod paths;
pub use paths::{ContactPath,ContactPathSegment};
pub use cover::{ContactCover,ContactCoverOptions,ContactCell,ContactEvidence,ContactLimit,ContactCoverError,ContactChart,ContactParameter};

/// Smooth-face contact candidates of a continuous sweep. This retains every
/// source face, including faces hidden by source Booleans. Source material can
/// reject hidden portions; sharp-edge sweeps, endpoint caps, singular events and
/// global trimming must still be constructed before claiming a closed B-rep.
#[derive(Clone,Debug)]
pub struct SweepContacts {
    pub(super) source: SpatialField,
    pub(super) patches: Vec<RevolvedSurface>,
    /// Patch indices of each revolved profile loop in loop order, so adjacent
    /// patches and the profile vertices between them are known. Axis diameters
    /// contribute no patch and leave a gap in the loop.
    pub(super) loops: Vec<Vec<Option<usize>>>,
    /// The tool's boundary by surface family, and the edges between faces,
    /// for the candidate tracer. Revolved faces are the patches above.
    pub(super) faces: Vec<ToolFace>,
    pub(super) edges: Vec<ToolEdge>,
    pub(super) motion: Family,
    pub(super) roll: [f64;2],
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
        let mut faces: Vec<ToolFace> = Vec::new();
        let mut edges: Vec<ToolEdge> = Vec::new();
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
                SolidDef::Prism {..} => {
                    let (prism_faces,prism_edges) = super::tool_faces::prism_faces(sk,id)?;
                    let first = faces.len();
                    faces.extend(prism_faces.iter().map(|f| f.placed(pose)));
                    edges.extend(prism_edges.into_iter().map(|e| ToolEdge {faces:e.faces.map(|i| i+first),charts:e.charts}));
                }
                SolidDef::Placed {source,motion,at} => pending.push((*source as usize,
                    Family::read(sk,*motion as usize)?.at(at.value)?.then(pose))),
                SolidDef::Body {..} => pending.extend(sk.solids[id].operands().into_iter()
                    .rev().map(|id| (id as usize,pose))),
                _ => return Err("contact construction requires static revolutions, prisms and Boolean operands".into()),
            }
        }
        // Every revolved patch is a face; adjacent patches of a loop that share a
        // meridian end share the circle that end sweeps, an edge charted at that
        // end on both.
        let first_revolved = faces.len();
        faces.extend(patches.iter().cloned().map(ToolFace::Revolved));
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
                edges.push(ToolEdge {faces:[first_revolved+a,first_revolved+b],charts:[EdgeChart::FixedU(ua),EdgeChart::FixedU(ub)]});
            }
        }
        Ok(Self {source:source_field,patches,loops,faces,edges,motion:Family::read(sk,*motion as usize)?,
            roll:[from.value,to.value]})
    }
    pub fn faces(&self) -> &[ToolFace] { &self.faces }
    pub fn edges(&self) -> &[ToolEdge] { &self.edges }
    pub fn patches(&self) -> &[RevolvedSurface] { &self.patches }
    pub fn loops(&self) -> &[Vec<Option<usize>>] { &self.loops }
    pub fn motion(&self) -> &Family { &self.motion }
    pub fn source_material(&self) -> &SpatialField { &self.source }
    pub fn domain(&self) -> [f64;2] { self.roll }
    pub fn at(&self,patch: usize,u: f64,roll: f64,tolerance: f64)
        -> Result<Vec<RevolvedContact>,Error> {
        if !roll.is_finite() { return Err(Error::NonFinite); }
        if roll < self.roll[0] || roll > self.roll[1] { return Err(Error::OutsideDomain); }
        let surface = self.patches.get(patch).ok_or(Error::OutsideDomain)?;
        surface.contacts(u,self.motion.at(roll).map_err(|_| Error::NonFinite)?,tolerance)
    }

    /// Fix revolution angle and solve for profile position. Meridian and ring
    /// charts have separate algebraic branch labels; neither alone covers events.
    pub fn at_angle(&self,patch: usize,v: f64,roll: f64,tolerance: f64)
        -> Result<Vec<super::surface::MeridianContact>,Error> {
        if !roll.is_finite() { return Err(Error::NonFinite); }
        if roll < self.roll[0] || roll > self.roll[1] { return Err(Error::OutsideDomain); }
        let surface = self.patches.get(patch).ok_or(Error::OutsideDomain)?;
        surface.meridian_contacts(v,self.motion.at(roll).map_err(|_| Error::NonFinite)?,tolerance)
    }

    /// Alternate chart: hold both source parameters and solve for motion times.
    /// This may cross a fold in the (u,time) chart without a surface singularity.
    /// Only single rotations and two relative rotations support the analytic
    /// temporal reduction. Neither chart alone guarantees complete coverage.
    pub fn at_source(&self,patch: usize,u: f64,v: f64,tolerance: f64)
        -> Result<Vec<TimedContact>,String> {
        self.at_source_over(patch,u,v,self.roll,tolerance)
    }

    /// Contact times of a point carrying a given unit normal, over a motion
    /// interval. This is the sharp-edge sweep condition: a convex source edge
    /// carries every normal between its incident faces', and each such normal
    /// contacts where it is perpendicular to the point's velocity. The caller
    /// supplies the normal; nothing here checks that the edge actually carries it.
    pub fn at_point_normal_over(&self,position: [f64;3],normal: [f64;3],interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,String> {
        let n = envelope::normalized(normal).ok_or("degenerate normal")?;
        let seed = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let du = envelope::normalized(crate::plane::cross(seed,n)).ok_or("degenerate normal")?;
        let dv = crate::plane::cross(n,du);
        let surface = envelope::SurfacePoint {position,du,dv};
        let roots = self.motion.normal_velocity(surface)?.roots(interval,tolerance,4096)
            .map_err(|e| format!("{e:?}"))?;
        roots.into_iter().map(|root| {
            let contact = envelope::contact(surface,self.motion.at(root.time)?)
                .map_err(|e| format!("{e:?}"))?;
            if contact.normal_velocity.abs() > tolerance {
                return Err("point-normal contact failed the normal-velocity equation".into());
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
