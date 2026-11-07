//! The planar generating class (#65), rows P1–P5: a sweep under a motion that turns a plane in
//! itself, of a tool that is a stock cut by one pocket — prisms standing square to that plane
//! through the blank. Over a whole period what such a sweep leaves of the blank is the blank within
//! the pocket's inner envelope (`envelope::planar`) carried along the prisms: a Wankel rotor is its
//! blank less the housing turned about it. The rows hold that reading:
//!
//! - P1: the motion turns the plane in itself about axes square to it (the envelope's plane row);
//! - P2: the tool is a stock cut by one pocket, both prisms on the pocket's plane spanning the
//!   blank's height along its normal, the pocket's profile one closed curve;
//! - P3: the roll is a whole period, so the envelope is closed and nothing is cut at its ends;
//! - P4: the pocket's inner envelope is one regular loop: no fold, no crossing, no gap;
//! - P5: the stock's outer wall never reaches the blank — every point of the blank, at every pose
//!   sampled, lies within the stock — so the pocket alone decides what is kept.
//!
//! Sampled where the other classes are: P5 on a grid of the blank's section and of the roll, P4
//! on the envelope's own samples (`envelope::planar`). The kernel reads the pocket from the tool's
//! B-rep, its curve carried there exactly where it is a point's path under turns (#721 F1d:
//! `brep::swept::pocket::Pocket::read`), asks the rows of each placement and chooses the class
//! (`brep::swept::dispatch`, #721 F1c); what is here is whether the drawing writes the tool as a
//! stock cut by one pocket (`asks`), for which the class's refusal stands, and the drawing's own
//! reading of the pocket, what the accuracy meter measures a build against (`pocket`, `prism`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::Condition;
use crate::model::{EntKind,Sketch,SolidDef};
use crate::space::{cross,dot,norm};

/// What the class finds of one sweep, its rows asked of a placement, and the tool as it reads it
/// are the kernel's (`brep::swept::pocket`, #721 F1c).
pub use crate::brep::swept::pocket::{Failure,Found,Pocket};

type V = [f64;3];


/// Whether `swept`'s rows are this class's: its tool is a prism cut by a pocket. A prism alone (a
/// rack) cuts with its sides, as the generating class's rows ask of it.
pub fn asks(sk: &Sketch,swept: usize) -> bool {
    let SolidDef::Swept {source,..} = &sk.solids[swept].def else { return false };
    matches!(&sk.solids[*source as usize].def,
        SolidDef::Body {stock,through,..} if !through.is_empty() && matches!(sk.solids[*stock as usize].def,SolidDef::Prism {..}))
}

/// The stretch along unit `n` a prism standing square to it spans in space.
pub(crate) fn prism(sk: &Sketch,id: usize,n: V) -> Result<[f64;2],Failure> {
    let solid = &sk.solids[id];
    let SolidDef::Prism {face,from,to,draft:None} = &solid.def else {
        return Err((Condition::Prisms,format!("`{}` is no prism with square sides",solid.name),None))
    };
    let basis = match sk.faces[*face as usize].plane {
        Some(p) => sk.basis(p as usize),
        None => crate::plane::Basis::page(),
    };
    let m = basis.normal();
    let m = m.map(|x| x/norm(m));
    if norm(cross(m,n)) > 1e-9 {
        return Err((Condition::Prisms,format!("`{}` stands on another plane than the pocket's",solid.name),None))
    }
    let (o,s) = (dot(basis.o,n),dot(m,n));
    let (a,b) = (o+s*from.value,o+s*to.value);
    Ok([a.min(b),a.max(b)])
}

/// A planar sweep's tool as the class reads it: a stock cut by one pocket, the pocket's profile one
/// closed curve, under a motion over a roll — the shape `admit` and the accuracy meter both read.
#[derive(Clone,Copy,Debug)]
pub(crate) struct Drawn { pub curve: usize,pub motion: usize,pub roll: [f64;2] }

/// `swept`'s tool read as a pocketed stock, else refused at P2 by name.
pub(crate) fn pocket(sk: &Sketch,swept: usize) -> Result<Drawn,Failure> {
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else {
        return Err((Condition::Prisms,"not a sweep".into(),None))
    };
    let tool = &sk.solids[*source as usize];
    let pocketed = |why: &str| (Condition::Prisms,format!("`{}` {why}: a planar sweep's tool is a stock cut by one pocket",tool.name),None);
    let (_,pocket) = match &tool.def {
        SolidDef::Body {stock,on,through,bound} if on.is_empty() && bound.is_empty() && through.len() == 1 => (*stock as usize,through[0] as usize),
        _ => return Err(pocketed("is more than a stock and a pocket")),
    };
    let SolidDef::Prism {face,..} = &sk.solids[pocket].def else { return Err(pocketed("is cut by no prism")) };
    let loops: Vec<_> = sk.faces[*face as usize].boundaries().collect();
    let curve = match loops.as_slice() {
        [(edges,_)] if edges.len() == 1 && edges[0].kind == EntKind::Curve => edges[0].i(),
        _ => return Err((Condition::Prisms,format!("`{}`'s profile is not one closed curve",sk.solids[pocket].name),None)),
    };
    let roll = super::roll::interval(sk,swept).map_err(|m| (Condition::Prisms,m,None))?;
    Ok(Drawn {curve,motion:*motion as usize,roll})
}
