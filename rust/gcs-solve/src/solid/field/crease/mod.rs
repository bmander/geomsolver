//! The creases of a material field, found from the field itself: where two operands meet on the
//! material's boundary. The protected Delaunay refinement (`delaunay::refine`) keeps a sharp edge
//! only where it is told the edge's curve, and an implicit domain has no curves to be told; these
//! are them.
//!
//! A crease is where the operand deciding the field changes — the block's face on one side, the
//! groove's sweep on the other — or where one sweep's contact time jumps, two contacts of one
//! tool. `MaterialField::operand` reads either operand alone, with its gradient, so a point is
//! pinned onto both zero sets by minimum-norm Newton on the two values and traced by
//! predictor-corrector along the cross product of their gradients (the marching of surface
//! intersection in CAD). A crease ends where a third operand takes the boundary over (a corner) or
//! where it leaves the bounding ball; it closes where it comes back to its start.
//!
//! Starting points come from a mesh already made without features: an edge whose ends are decided
//! by different operands crosses a crease. A crease loop smaller than that mesh's facets can be
//! missed, which is marching's known limit (its remedy, loop detection, is not attempted).
use super::{MaterialField,Query,Reading,Symmetry};
pub use super::OperandId;
use crate::space::{distance,segment_distance};
mod graph;
mod trace;
pub use graph::features;
use trace::{pin,trace};
use graph::split;

type P = [f64;3];

/// A crease starts only where its two surfaces meet at least this far from tangent, in radians:
/// two degrees. It may run on to where they are nearly tangent (`pin` refuses only exact tangency).
const LEAST_DIHEDRAL: f64 = 2.*std::f64::consts::PI/180.;

/// What creases are found in: a field read whole and one operand at a time. A material field is
/// one (the only one outside the tests); a reading's operands are the source's own
/// (`OperandId`), handed back to be read alone.
pub trait CreaseSource {
    /// The whole field's reading at a point, its sweeps' minima found to `accuracy`.
    fn read(&self,p: P,accuracy: f64) -> Reading;
    /// One operand's reading alone at a point (`MaterialField::operand`), to `accuracy`; `None`
    /// when the source has no such operand.
    fn read_operand(&self,p: P,op: OperandId,accuracy: f64) -> Option<Reading>;
    /// Rigid maps the source reads the same under, checked at `samples` (`MaterialField::symmetries`).
    fn symmetries(&self,_samples: &[P]) -> Vec<Symmetry> { Vec::new() }
}

/// A point query to `accuracy` at `p`: exact, every sweep's minimum to that and no coarser.
fn query_at(p: P,accuracy: f64) -> Query { Query {accuracy,relative:0.,..Query::at(p)} }

impl CreaseSource for MaterialField {
    fn read(&self,p: P,accuracy: f64) -> Reading { self.query(p,&mut query_at(p,accuracy)) }
    fn read_operand(&self,p: P,op: OperandId,accuracy: f64) -> Option<Reading> { self.operand(p,op,&query_at(p,accuracy)) }
    fn symmetries(&self,samples: &[P]) -> Vec<Symmetry> { MaterialField::symmetries(self,samples) }
}

/// How finely creases are found and followed.
#[derive(Clone,Copy,Debug)]
pub struct CreaseOptions {
    /// The step along a crease.
    pub step: f64,
    /// A point is on both zero sets, and on the material's boundary, to this.
    pub tolerance: f64,
    /// Contact times of one sweep this far apart are two operands.
    pub time_gap: f64,
    /// The bounding ball a crease is followed within.
    pub centre: P,
    pub radius: f64,
    /// Points a crease may have before it is given up (a crease that never closes nor ends).
    pub max_points: usize,
}

/// A traced crease: its points in order, whether it closes on itself, the two operands it was
/// started between (a hand-off may change them along it), and why it stops at each end.
#[derive(Clone,Debug)]
pub struct Crease { pub points: Vec<P>,pub closed: bool,pub operands: [OperandId;2],pub ends: [End;2] }

/// A crease's points are read to a hundredth of the tolerance they are pinned to.
fn accuracy(tolerance: f64) -> f64 { 0.01*tolerance }

/// Why a traced crease stops where it does.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum End {
    /// It came back to its start.
    Closed,
    /// A third operand meets both at a point: a corner of the material.
    Corner(OperandId),
    /// A third operand takes the boundary over, and the three pin no point there.
    Unpinned(OperandId),
    /// It left the bounding ball.
    Ball,
    /// The corrector failed at every step size: the two surfaces turn tangent, or Newton fails.
    Lost,
    /// Hand-offs across tangent operands went round without progress.
    Handoffs,
    /// The point budget ran out.
    Budget,
    /// It walked onto crease `n` of those already traced, at a point of it.
    Met(usize),
}

/// About how many of a first pass's vertices a symmetry is checked at (`MaterialField::symmetries`).
const SYMMETRY_VERTICES: usize = 64;

/// Starting points for creases on a mesh made without them: the middle of every edge whose ends
/// are decided by different operands, with those two operands.
pub fn seeds<S: CreaseSource + ?Sized>(field: &S,vertices: &[P],triangles: &[[u32;3]],options: &CreaseOptions)
    -> Vec<(P,[OperandId;2])> {
    // a vertex whose reading names no operand (its sweep's pose could not be read) seeds nothing
    let operands: Vec<Option<OperandId>> = vertices.iter().map(|&v| field.read(v,accuracy(options.tolerance)).operand).collect();
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for t in triangles {
        for j in 0..3 {
            let (u,v) = (t[j].min(t[(j+1)%3]),t[j].max(t[(j+1)%3]));
            if !seen.insert((u,v)) { continue; }
            let (Some(a),Some(b)) = (operands[u as usize],operands[v as usize]) else { continue };
            if a.differs(b,options.time_gap) {
                let (pu,pv) = (vertices[u as usize],vertices[v as usize]);
                out.push((std::array::from_fn(|k| 0.5*(pu[k]+pv[k])),[a,b]));
            }
        }
    }
    out
}

/// The creases through a set of seeds, each traced once: a seed that pins onto a crease already
/// traced starts nothing. A seed may lie some way off the crease it finds — a mesh made without
/// features cuts a sharp edge on a chamfer — so it is the pinned point that is compared.
pub fn creases<S: CreaseSource + ?Sized>(field: &S,seeds: &[(P,[OperandId;2])],options: &CreaseOptions) -> Vec<Crease> {
    creases_under(field,seeds,options,&[])
}

/// `creases` of a field alike under rigid maps (`MaterialField::symmetries`, a gear's teeth):
/// each crease traced once and carried by every map to its images, whose seeds then find them.
pub(super) fn creases_under<S: CreaseSource + ?Sized>(field: &S,seeds: &[(P,[OperandId;2])],options: &CreaseOptions,maps: &[Symmetry])
    -> Vec<Crease> {
    let mut out: Vec<Crease> = Vec::new();
    let near = |out: &[Crease],p: P,within: f64| out.iter().any(|c| c.points.len() == 1 && distance(p,c.points[0]) < within
        || c.points.windows(2).any(|w| segment_distance(p,w[0],w[1]) < within));
    for &(p,ops) in seeds {
        // a seed on a crease already traced needs no pinning
        if near(&out,p,0.5*options.step) { continue; }
        let Some((q,..)) = pin(field,p,&mut ops.clone(),options.tolerance,options.step) else { continue };
        if near(&out,q,0.25*options.step) { continue; }
        let Some(c) = trace(field,q,ops,options,&out) else { continue };
        if c.points.len() < 2 { continue; }
        // A trace that ended on another crease's middle splits that crease there, so the two share
        // a corner rather than one ending beside the other.
        for (e,at) in [(c.ends[0],c.points[0]),(c.ends[1],*c.points.last().unwrap())] {
            if let End::Met(k) = e { split(&mut out,k,at); }
        }
        // (an image keeps the original's operands and ends, a `Met` naming the crease it met)
        let images: Vec<Crease> = maps.iter().map(|m| Crease {points:c.points.iter().map(|&x| m.apply(x)).collect(),..c.clone()}).collect();
        out.push(c);
        // An image lying on a crease already there (a crease its own image: a ring about the axis)
        // is not added again; T-junctions an image makes are `junctions`' to resolve.
        for image in images {
            let n = image.points.len();
            let there = [n/4,n/2,3*n/4].iter().all(|&k| near(&out,image.points[k],0.25*options.step));
            if !there { out.push(image); }
        }
    }
    out
}

