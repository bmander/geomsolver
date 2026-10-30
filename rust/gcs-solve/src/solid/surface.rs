//! Exact generating patches of existing declarative solids, read off the sketch. The patches,
//! their contacts, projections and material are the kernel's (`cad_kernel::sweep::surface`);
//! here is what reads a revolution's profile edge, a named surface and a revolution's material
//! off the model, the kernel's `Source` for the sketch.
#[allow(unused_imports)]
use crate::fmath::Det;
pub use cad_kernel::sweep::surface::*;

use crate::model::{EntKind, EntRef, Sense, Sketch, SolidDef};
use crate::plane::Basis;
use crate::space::{sub, scale};
use std::f64::consts::TAU;

impl Source for Sketch {
    type Edge = EntRef;
    fn revolved(&self,solid: usize,edge: EntRef) -> Result<RevolvedSurface,String> { read(self,solid,edge) }
    fn surface(&self,surface: usize) -> Result<RevolvedSurface,String> { named(self,surface) }
    fn revolved_region(&self,solid: usize,axis_tolerance: f64) -> Result<RevolvedRegion,String> {
        region(self,solid,axis_tolerance)
    }
}

/// Read a named model surface against the sketch's current solved state.
fn named(sk: &Sketch, surface: usize) -> Result<RevolvedSurface, String> {
    let s = sk.surfaces.get(surface).ok_or("no such surface")?;
    let mut patch = read(sk,s.solid as usize,s.edge)?;
    patch.name = s.name.clone();
    match s.span { Some(span) => patch.spanned(span),None => Ok(patch) }
}

/// Read a line, arc or circle of a revolution's profile, including hole boundaries.
/// This checks the requested patch, not closure or validity of the entire solid.
fn read(sk: &Sketch, solid: usize, edge: EntRef) -> Result<RevolvedSurface, String> {
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
    let patch = RevolvedSurface::new(format!("{}.{}",solid.name,name),meridian,origin,
        scale(delta,1./len),sweep);
    patch.at(0.,0.).map_err(|e| format!("invalid revolved surface: {e:?}"))?;
    Ok(patch)
}

/// `axis_tolerance` is an absolute length used to recognize solved axis edges.
/// Their endpoints within this tolerance are put on the axis. Other edges retain
/// their solved geometry. Read again after modifying or resolving the sketch.
fn region(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<RevolvedRegion,String> {
    if !axis_tolerance.is_finite() || axis_tolerance < 0. {
        return Err("an axis tolerance must be finite and nonnegative".into());
    }
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    // a region is a revolution of its meridian (§6.21)
    if let SolidDef::Region {..} = s.def {
        return Ok(RevolvedRegion::of_meridian(&*crate::solid::region::meridian(sk,solid)?,axis_tolerance));
    }
    let SolidDef::Revolve {face,ref sweep,..} = s.def else {
        return Err("analytic material membership requires an unmodified revolution".into());
    };
    if sweep.value < TAU || !sweep.value.is_finite() {
        return Err("analytic material membership requires a full revolution".into());
    }
    super::validate(sk,solid)?;
    let face = &sk.faces[face as usize];
    let patches = face.boundaries().map(|(edges,_)| edges.iter()
        .map(|&e| read(sk,solid,e).map(|p| (p,e)))
        .collect::<Result<Vec<_>,_>>())
        .collect::<Result<Vec<_>,_>>()?;
    // an arc's own ends, which its neighbours share (a full circle has none)
    let patches = patches.into_iter().map(|l| l.into_iter().map(|(p,e)| {
        let ends = (e.kind == EntKind::Arc).then(|| {
            let arc = &sk.arcs[e.i()];
            [arc.start,arc.end].map(|i| sk.world_point(i as usize))
        });
        (p,ends)
    }).collect()).collect();
    let basis = face.plane.map(|p| sk.basis(p as usize)).unwrap_or(Basis::page());
    let normal = crate::plane::cross(basis.u,basis.v);
    RevolvedRegion::of_profile(patches,normal,axis_tolerance)
}
