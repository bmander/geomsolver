//! Candidate envelope faces read from an ordinary swept-solid definition.
use super::{SpatialField,RevolvedSurface,RevolvedContact};
use crate::{envelope::{self,Contact,Error,Motion},model::{Sketch,SolidDef,EntKind},motion::Family};
use std::{collections::BTreeSet,f64::consts::TAU};
mod cover;
mod curves;
pub use curves::{ContactCurve,ContactCurves,ContactCurvePoint};
pub use cover::{ContactCover,ContactCoverOptions,ContactCell,ContactEvidence,ContactLimit,ContactCoverError,ContactChart,ContactParameter};

/// Smooth-face contact candidates of a continuous sweep. This retains every
/// source face, including faces hidden by source Booleans. Source material can
/// reject hidden portions; sharp-edge sweeps, endpoint caps, singular events and
/// global trimming must still be constructed before claiming a closed B-rep.
#[derive(Clone,Debug)]
pub struct SweepContacts {
    source: SpatialField,
    patches: Vec<RevolvedSurface>,
    motion: Family,
    roll: [f64;2],
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
        while let Some((id,pose)) = pending.pop() {
            let columns: [[u64;3];4] = std::array::from_fn(|i| {
                let p = if i == 3 { pose.point([0.;3]) } else {
                    let mut p = [0.;3]; p[i] = 1.; pose.vector(p)
                }; p.map(f64::to_bits)
            });
            if !seen.insert((id,columns)) { continue; }
            match &sk.solids[id].def {
                SolidDef::Revolve {face,..} => {
                    for (edges,_) in sk.faces[*face as usize].boundaries() { for &edge in edges {
                        let surface = RevolvedSurface::read(sk,id,edge)?.placed(pose);
                        if edge.kind == EntKind::Line {
                            let radius = |u| surface.at(u,0.).map(|p|
                                p.dv[0].hypot(p.dv[1]).hypot(p.dv[2])/TAU);
                            // A diameter on the axis disappears in a full revolution.
                            if radius(0.).map_err(|e| format!("{e:?}"))? <= axis_tolerance
                                && radius(1.).map_err(|e| format!("{e:?}"))? <= axis_tolerance { continue; }
                        }
                        patches.push(surface);
                    } }
                }
                SolidDef::Placed {source,motion,at} => pending.push((*source as usize,
                    Family::read(sk,*motion as usize)?.at(at.value)?.then(pose))),
                SolidDef::Body {..} => pending.extend(sk.solids[id].operands().into_iter()
                    .rev().map(|id| (id as usize,pose))),
                _ => return Err("contact construction requires static revolutions and Boolean operands".into()),
            }
        }
        Ok(Self {source:source_field,patches,motion:Family::read(sk,*motion as usize)?,
            roll:[from.value,to.value]})
    }
    pub fn patches(&self) -> &[RevolvedSurface] { &self.patches }
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
        let surface = self.patches.get(patch).ok_or("no such sweep patch")?.at(u,v)
            .map_err(|e| format!("{e:?}"))?;
        let roots = self.motion.normal_velocity(surface)?.roots(self.roll,tolerance,4096)
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
