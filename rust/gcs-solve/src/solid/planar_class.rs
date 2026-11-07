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
//! on the envelope's own samples (`envelope::planar`). The kernel asks the rows of each placement
//! and chooses the class (`brep::swept::dispatch`, #721 F1c); what is here is the drawing's reading
//! of the pocket (`read`), which the kernel is handed until it reads the pocket from the tool's
//! B-rep (#721 F1d).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::{Condition,Options};
use crate::envelope::Motion;
use crate::envelope::planar::{InnerEnvelope,ReadEnvelope};
use crate::model::{EntKind,Sketch,SolidDef};
use crate::space::{cross,dot,norm};

/// What the class finds of one sweep, its rows asked of a placement, and the tool as it reads it
/// are the kernel's (`brep::swept::pocket`, #721 F1c).
pub use crate::brep::swept::pocket::{Failure,Found,Pocket};

type V = [f64;3];

/// The stock's material as the drawing's field reads it, what the kernel's P5 reads the blank
/// against while the pocket is read off the drawing.
impl crate::brep::swept::tool::Material for super::SpatialField {
    fn value(&self,p: V) -> f64 { super::SpatialField::value(self,p) }
}

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
pub(crate) struct Drawn { pub stock: usize,pub pocket: usize,pub curve: usize,pub motion: usize,pub roll: [f64;2] }

/// `swept`'s tool read as a pocketed stock, else refused at P2 by name.
pub(crate) fn pocket(sk: &Sketch,swept: usize) -> Result<Drawn,Failure> {
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else {
        return Err((Condition::Prisms,"not a sweep".into(),None))
    };
    let tool = &sk.solids[*source as usize];
    let pocketed = |why: &str| (Condition::Prisms,format!("`{}` {why}: a planar sweep's tool is a stock cut by one pocket",tool.name),None);
    let (stock,pocket) = match &tool.def {
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
    Ok(Drawn {stock,pocket,curve,motion:*motion as usize,roll})
}

/// **Sweep `swept`'s tool read off the drawing as the class reads it** (#721 F1c), refusing at
/// P1–P4 by row: the pocket's inner envelope over the roll (P1, P3, P4), its plane, both prisms'
/// spans along its normal, the stock's field and the inverse poses of a grid of the roll that P5
/// reads the blank against — the kernel's `Pocket`, whose rows it asks of each placement
/// (`brep::swept::pocket`). Read off the drawing until the kernel reads it from the tool's B-rep,
/// which fits the pocket's curve where the drawing has it exactly (#721 F1d).
pub fn read(sk: &Sketch,swept: usize,options: &Options) -> Result<Pocket,Failure> {
    let drawn = pocket(sk,swept)?;
    let view = sk.curve_view(drawn.curve);
    let basis = view.map_or_else(crate::plane::Basis::page,|v| sk.basis(v));
    let n = basis.normal();
    let n = n.map(|x| x/norm(n));
    let spans = [prism(sk,drawn.stock,n)?,prism(sk,drawn.pocket,n)?];
    let envelope = InnerEnvelope::read(sk,drawn.curve,drawn.motion,drawn.roll)
        .map_err(|r| (r.row,r.message,r.witness.map(|p| sk.world_in(view,(p[0],p[1])))))?;
    let family = crate::motion::Family::read(sk,drawn.motion).map_err(|e| (Condition::Plane,e,None))?;
    let stock = super::SpatialField::read(sk,drawn.stock,options.axis_tolerance).map_err(|e| (Condition::Prisms,e,None))?;
    let [from,to] = drawn.roll;
    let rolls = (((to-from)/std::f64::consts::TAU*360.).ceil() as usize).max(36);
    let backs = (0..=rolls).map(|k| family.at(from+(to-from)*k as f64/rolls as f64).map(Motion::inverse))
        .collect::<Result<_,_>>().map_err(|e| (Condition::Plane,e,None))?;
    Ok(Pocket {envelope,u:basis.u,n,spans,names:[sk.solids[drawn.stock].name.clone(),sk.solids[drawn.pocket].name.clone()],
        stock:std::sync::Arc::new(stock),backs})
}
