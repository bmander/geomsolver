//! Material membership of full revolutions, evaluated on analytic profile curves.
//! This is separate from the faceted CSG kernel: callers must choose which geometry
//! they mean. It does not silently substitute an analytic surface for a mesh face.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::space::add;
use std::f64::consts::TAU;

type P = [f64;2];
fn distance(a: P,b: P) -> f64 { (a[0]-b[0]).dhypot(a[1]-b[1]) }

/// One analytic profile edge in planar coordinates is the kernel's (`brep::plane2::Edge`).
pub(crate) use crate::brep::plane2::Edge;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum RegionLocation { Inside, Boundary, Outside }

#[derive(Clone,Copy,Debug)]
pub struct RegionSample {
    pub location: RegionLocation,
    /// Negative in material, positive outside. Analytic finite meridian distances,
    /// in model length units; evaluated in floating point, not an interval proof.
    pub signed_distance: f64,
}

/// A solved snapshot of an unmodified full revolution with line/circular profiles.
/// Profile topology uses the existing solid validator; point membership and distance
/// use analytic curves and have no tessellation setting. Hole interiors are voids.
#[derive(Clone,Debug)]
pub struct RevolvedRegion {
    pub(crate) origin: V,
    pub(crate) axis: V,
    pub(crate) loops: Vec<Vec<Edge>>,
    /// The axis tolerance the snapshot was read with: an endpoint put on the axis
    /// moved by up to this, so its neighbours join it within the same distance.
    pub(crate) axis_tolerance: f64,
}

impl RevolvedRegion {
    /// `axis_tolerance` is an absolute length used to recognize solved axis edges.
    /// Their endpoints within this tolerance are put on the axis. Other edges retain
    /// their solved geometry. Read again after modifying or resolving the sketch.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        if !axis_tolerance.is_finite() || axis_tolerance < 0. {
            return Err("an axis tolerance must be finite and nonnegative".into());
        }
        let s = sk.solids.get(solid).ok_or("no such solid")?;
        // a region is a revolution of its meridian (§6.21)
        if let SolidDef::Region {..} = s.def {
            return Ok(Self::of_meridian(&*crate::solid::region::meridian(sk,solid)?,axis_tolerance));
        }
        let SolidDef::Revolve {face,ref sweep,..} = s.def else {
            return Err("analytic material membership requires an unmodified revolution".into());
        };
        if sweep.value < TAU || !sweep.value.is_finite() {
            return Err("analytic material membership requires a full revolution".into());
        }
        super::super::validate(sk,solid)?;
        let face = &sk.faces[face as usize];
        let patches = face.boundaries().map(|(edges,_)| edges.iter()
            .map(|&e| RevolvedSurface::read(sk,solid,e).map(|p| (p,e)))
            .collect::<Result<Vec<_>,_>>())
            .collect::<Result<Vec<_>,_>>()?;
        let first = &patches.first().and_then(|p| p.first()).ok_or("empty profile")?.0;
        let origin = first.origin(); let axis = first.axis_direction();
        let basis = face.plane.map(|p| sk.basis(p as usize)).unwrap_or(Basis::page());
        let normal = plane::cross(basis.u,basis.v);
        let radial = plane::cross(axis,normal);
        let coords = |p| [plane::dot(p,radial),plane::dot(p,axis)];
        let mut loops = patches.iter().map(|patches| patches.iter().map(|(p,e)| {
            Ok(match *p.meridian() {
                Meridian::Line {start,delta} => {
                    let a = coords(sub(start,origin));
                    let b = coords(sub(add(start,delta),origin));
                    if distance(a,b) == 0. { return Err("degenerate profile line".into()); }
                    Edge::Line {a,b,axis:false}
                }
                Meridian::Round {center,a,b,sweep} => {
                    let a = coords(a); let b = coords(b);
                    let ends = if e.kind == EntKind::Arc {
                        let arc = &sk.arcs[e.i()];
                        [arc.start,arc.end].map(|i| coords(sub(sk.world_point(i as usize),origin)))
                    } else { [[0.;2];2] }; // Full circles have no ray seam.
                    Edge::Arc {center:coords(sub(center,origin)),radius:a[0].dhypot(a[1]),
                        start:a[1].datan2(a[0]),sweep:sweep*(a[0]*b[1]-a[1]*b[0]).signum(),ends}
                }
                Meridian::Conic {a,m,b,w} => Edge::Conic {a:coords(sub(a,origin)),
                    m:coords(sub(m,origin)),b:coords(sub(b,origin)),w},
            })
        }).collect::<Result<Vec<_>,String>>()).collect::<Result<Vec<_>,_>>()?;
        let bounds = loops.iter().flatten().map(Edge::radial_bounds)
            .fold([f64::INFINITY,f64::NEG_INFINITY],|b,e| [b[0].min(e[0]),b[1].max(e[1])]);
        if bounds[0] < -axis_tolerance && bounds[1] > axis_tolerance {
            return Err("the revolution axis crosses the analytic profile".into());
        }
        if bounds[1].max(-bounds[0]) <= axis_tolerance {
            return Err("the profile has no material away from its axis".into());
        }
        for edge in loops.iter_mut().flatten() {
            if -bounds[0] > bounds[1] { edge.reflect(); }
            if let Edge::Line {a,b,axis} = edge {
                if a[0].abs() <= axis_tolerance && b[0].abs() <= axis_tolerance {
                    a[0] = 0.; b[0] = 0.; *axis = true;
                }
            }
        }
        Ok(Self {origin,axis,loops,axis_tolerance})
    }

    /// A region solid's meridian as analytic membership: its loops' lines and arcs in (radius,
    /// height), the loop enclosing most first, its sides on the axis marked as such.
    pub(crate) fn of_meridian(m: &crate::solid::region::Meridian,axis_tolerance: f64) -> Self {
        use crate::brep::planar::Seg;
        let loops = m.region.outer_first().into_iter().map(|l| l.iter().map(|step| match step.seg {
            Seg::Line {a,b} => Edge::Line {a,b,axis:a[0].abs() <= axis_tolerance && b[0].abs() <= axis_tolerance},
            Seg::Arc {c,r,a0,sweep} => Edge::Arc {center:c,radius:r,start:a0,sweep,ends:[step.start(),step.end()]},
            Seg::Conic {a,m,b,w} => Edge::Conic {a,m,b,w},
        }).collect()).collect();
        Self {origin:m.origin,axis:m.axis,loops,axis_tolerance}
    }

    /// Classify using an explicit boundary band in model length units. Axis edges
    /// that disappear on revolution are not boundaries (e.g. a sphere's diameter).
    pub fn classify(&self,point: V,tolerance: f64) -> Result<RegionSample,Error> {
        if !point.iter().all(|v| v.is_finite()) || !tolerance.is_finite() {
            return Err(Error::NonFinite);
        }
        if tolerance < 0. { return Err(Error::OutsideDomain); }
        let q = sub(point,self.origin);
        let z = plane::dot(q,self.axis);
        let v = sub(q,scale(self.axis,z));
        let p = [v[0].dhypot(v[1]).dhypot(v[2]),z];
        let distance = self.loops.iter().flatten().map(|e| e.distance(p))
            .fold(f64::INFINITY,f64::min);
        if !distance.is_finite() { return Err(Error::NonFinite); }
        let inside = |edges: &[Edge]| edges.iter().map(|e| e.crossings(p)).sum::<usize>()%2 == 1;
        let material = inside(&self.loops[0]) && !self.loops[1..].iter().any(|l| inside(l));
        let location = if distance <= tolerance { RegionLocation::Boundary }
            else if material { RegionLocation::Inside } else { RegionLocation::Outside };
        Ok(RegionSample {location,signed_distance:if material { -distance } else { distance }})
    }
}
