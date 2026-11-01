//! Exact generating patches of existing declarative solids. These retain the solved
//! profile's lines and circles; no facets enter their position or derivative evaluation.
//! Boundary trimming and outward orientation remain separate questions.
//!
//! A face of revolution itself, its rings and their contact, and its support equation are the
//! kernel's (`brep::swept::surface`, #721 F2b); what is read here is the drawing: which edge of
//! which profile, turned about which axis, and named what (`ReadRevolved`).
#[allow(unused_imports)]
use crate::fmath::Det;
mod region;
pub use region::{RegionLocation,RegionSample,RevolvedRegion};
pub(crate) use region::Edge;
pub use crate::brep::swept::surface::{Meridian,RevolvedContact,RevolvedSurface,SurfaceProjection,SurfaceProjector};

use crate::envelope::Error;
use crate::model::{EntKind, EntRef, Sense, Sketch, SolidDef};
use crate::plane::{self, Basis};

use crate::space::{sub, scale};

type V = [f64;3];

/// A face of revolution read off the drawing: an edge of a revolution's profile, a named surface,
/// or a region solid's meridian, turned.
pub trait ReadRevolved: Sized {
    /// Read a named model surface against the sketch's current solved state.
    fn named(sk: &Sketch, surface: usize) -> Result<Self, String>;
    /// Read a line, arc or circle of a revolution's profile, including hole boundaries.
    /// This checks the requested patch, not closure or validity of the entire solid.
    fn read(sk: &Sketch, solid: usize, edge: EntRef) -> Result<Self, String>;
    /// A region solid's faces (§6.21): each step of its meridian turned once about its axis,
    /// named by the term it came from — but a line on the axis, which turns to no face.
    fn of_meridian(m: &super::region::Meridian,solid: &str) -> Vec<Self>;
    /// `of_meridian` loop by loop, in the meridian's order, a line on the axis `None`: the
    /// profile a region tool's sweep is read over, as a revolution's faces' loops are — and the
    /// faces a check reads off a region cutter, each beside its neighbours (#214).
    fn meridian_loops(m: &super::region::Meridian,solid: &str) -> Vec<Vec<Option<Self>>>;
}

impl ReadRevolved for RevolvedSurface {
    fn named(sk: &Sketch, surface: usize) -> Result<Self, String> {
        let s = sk.surfaces.get(surface).ok_or("no such surface")?;
        let mut patch = Self::read(sk,s.solid as usize,s.edge)?;
        patch.name = s.name.clone();
        match s.span { Some(span) => patch.spanned(span),None => Ok(patch) }
    }

    fn read(sk: &Sketch, solid: usize, edge: EntRef) -> Result<Self, String> {
        let solid = sk.solids.get(solid).ok_or("no such solid")?;
        let SolidDef::Revolve { face, axis, sweep, sense } = &solid.def else {
            return Err("an exact revolved patch requires a revolution".into());
        };
        if !sweep.value.is_finite() || sweep.value <= 0. {
            return Err("a revolution needs a finite positive sweep".into());
        }
        let (a2, b2) = sk.revolve_axis(*face, *axis)
            .ok_or("the revolution axis does not lie in the profile's plane")?;
        let face = sk.faces.get(*face as usize).ok_or("no such profile")?;
        let name = face.edges.iter().zip(&face.edge_names)
            .chain(face.holes.iter().flat_map(|h| h.edges.iter().zip(&h.edge_names)))
            .find_map(|(e,n)| (*e == edge).then_some(n))
            .ok_or("the requested edge is not a boundary of this revolution's profile")?;
        let basis = if let Some(p) = face.plane {
            sk.frames.get(p as usize).ok_or("no profile plane")?;
            sk.basis(p as usize)
        } else { Basis::page() };
        let lift = |q: (f64,f64)| basis.lift(q.0,q.1);
        let origin = lift(a2);
        let delta = sub(lift(b2),origin);
        let len = delta[0].dhypot(delta[1]).dhypot(delta[2]);
        if !len.is_finite() || len == 0. { return Err("degenerate revolution axis".into()); }
        let meridian = match edge.kind {
            EntKind::Line => {
                let line = sk.lines.get(edge.i()).ok_or("no such profile line")?;
                let start = lift(sk.point_xy(line.p1 as usize));
                Meridian::Line { start,delta: sub(lift(sk.point_xy(line.p2 as usize)),start) }
            }
            EntKind::Arc | EntKind::Circle => {
                let (center,radius,start,sweep) = if edge.kind == EntKind::Arc {
                    let a = sk.arcs.get(edge.i()).ok_or("no such profile arc")?;
                    let (start,end) = sk.arc_angles(edge.i());
                    (a.center,a.radius,start,end-start)
                } else {
                    let c = sk.circles.get(edge.i()).ok_or("no such profile circle")?;
                    (c.center,c.radius,0.,std::f64::consts::TAU)
                };
                let r = sk.params[radius as usize].value;
                if !r.is_finite() || r <= 0. || !sweep.is_finite() || sweep <= 0. {
                    return Err("a round profile edge needs positive finite radius and sweep".into());
                }
                Meridian::round(lift(sk.point_xy(center as usize)),basis.u,basis.v,r,start,sweep)
            }
            _ => return Err("exact revolved patches currently require lines, arcs or circles".into()),
        };
        let sweep = sweep.value.min(std::f64::consts::TAU)
            * if *sense == Sense::Cw { -1. } else { 1. };
        let patch = Self::new(format!("{}.{}",solid.name,name),meridian,origin,scale(delta,1./len),sweep);
        patch.at(0.,0.).map_err(|e| format!("invalid revolved surface: {e:?}"))?;
        Ok(patch)
    }

    fn of_meridian(m: &super::region::Meridian,solid: &str) -> Vec<Self> {
        Self::meridian_loops(m,solid).into_iter().flatten().flatten().collect()
    }

    fn meridian_loops(m: &super::region::Meridian,solid: &str)
        -> Vec<Vec<Option<Self>>> {
        use crate::brep::planar::Seg;
        let basis = Basis { u: m.seam,v: m.axis,o: m.origin };
        let at = |q: [f64;2]| basis.lift(q[0],q[1]);
        let on_axis = |q: [f64;2]| q[0].abs() <= crate::solid::cad::AXIS_TOLERANCE;
        m.region.loops.iter().map(|lp| lp.iter().map(|step| {
            let meridian = match step.seg {
                Seg::Line {a,b} if on_axis(a) && on_axis(b) => return None,
                Seg::Line {a,b} => Meridian::Line {start: at(a),delta: sub(at(b),at(a))},
                // a round edge runs counter-clockwise from its start
                Seg::Arc {c,r,a0,sweep} => Meridian::round(at(c),m.seam,m.axis,r,
                    if sweep < 0. { a0+sweep } else { a0 },sweep.abs()),
                Seg::Conic {a,m: c,b,w} => Meridian::Conic {a: at(a),m: at(c),b: at(b),w},
            };
            let name = format!("{solid}.{}",m.name(step.tag));
            Some(Self::new(name,meridian,m.origin,m.axis,std::f64::consts::TAU))
        }).collect()).collect()
    }

}

/// A named surface's support equation, read off the drawing.
pub trait ReadProjector: Sized {
    fn named(sk: &Sketch, index: usize) -> Result<Self,String>;
}

impl ReadProjector for SurfaceProjector {
    fn named(sk: &Sketch, index: usize) -> Result<Self,String> {
        RevolvedSurface::named(sk,index)?.projector()
    }

}
