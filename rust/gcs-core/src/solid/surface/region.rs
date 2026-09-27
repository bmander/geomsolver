//! Material membership of full revolutions, evaluated on analytic profile curves.
//! This is separate from the faceted CSG kernel: callers must choose which geometry
//! they mean. It does not silently substitute an analytic surface for a mesh face.
use super::*;
use std::f64::consts::{FRAC_PI_2,PI,TAU};

type P = [f64;2];
fn distance(a: P,b: P) -> f64 { (a[0]-b[0]).hypot(a[1]-b[1]) }

/// One analytic profile edge in planar coordinates: a revolved meridian's
/// (radius, height), or a prism section's plane view coordinates. An arc's
/// `ends` are the exact source vertices, `ends[0]` at `start`; a full circle
/// has `sweep` of one turn and no seam.
#[derive(Clone,Debug)]
pub(crate) enum Edge {
    Line {a:P,b:P,axis:bool},
    Arc {center:P,radius:f64,start:f64,sweep:f64,ends:[P;2]},
}

impl Edge {
    pub(crate) fn at(&self,t: f64) -> P {
        match *self {
            Self::Line {a,b,..} => std::array::from_fn(|i| a[i]+t*(b[i]-a[i])),
            Self::Arc {center,radius,start,sweep,..} => {
                let (s,c) = (start+t*sweep).sin_cos();
                [center[0]+radius*c,center[1]+radius*s]
            }
        }
    }

    fn radial_bounds(&self) -> [f64;2] {
        let mut lo = self.at(0.)[0].min(self.at(1.)[0]);
        let mut hi = self.at(0.)[0].max(self.at(1.)[0]);
        if let Self::Arc {center,radius,start,sweep,..} = *self {
            for angle in [0.,PI] {
                if ((angle-start)*sweep.signum()).rem_euclid(TAU) <= sweep.abs() {
                    let r = center[0]+radius*angle.cos();
                    lo = lo.min(r); hi = hi.max(r);
                }
            }
        }
        [lo,hi]
    }

    fn reflect(&mut self) {
        match self {
            Self::Line {a,b,..} => { a[0] = -a[0]; b[0] = -b[0]; }
            Self::Arc {center,start,sweep,ends,..} => {
                center[0] = -center[0]; *start = PI-*start; *sweep = -*sweep;
                for end in ends { end[0] = -end[0]; }
            }
        }
    }

    // The closest point on an analytic finite meridian. Rotation is unrestricted,
    // so this is also the distance to its revolved surface in cylindrical space.
    pub(crate) fn distance(&self,p: P) -> f64 {
        match *self {
            Self::Line {a,b,axis} => {
                // An axis edge disappears in a full revolution; it is not a wall.
                if axis { return f64::INFINITY; }
                let d = [b[0]-a[0],b[1]-a[1]];
                let len = d[0].hypot(d[1]);
                let t = (((p[0]-a[0])*(d[0]/len)+(p[1]-a[1])*(d[1]/len))/len)
                    .clamp(0.,1.);
                distance(p,self.at(t))
            }
            Self::Arc {center,radius,start,sweep,..} => {
                let q = [p[0]-center[0],p[1]-center[1]];
                let angle = q[1].atan2(q[0]);
                if ((angle-start)*sweep.signum()).rem_euclid(TAU) <= sweep.abs() {
                    (q[0].hypot(q[1])-radius).abs()
                } else { distance(p,self.at(0.)).min(distance(p,self.at(1.))) }
            }
        }
    }

    // Half-open horizontal ray crossings. Split arcs at their vertical extrema
    // analytically, so tangent rays and shared endpoints do not get counted twice.
    pub(crate) fn crossings(&self,p: P) -> usize {
        match *self {
            Self::Line {a,b,..} => usize::from((a[1] > p[1]) != (b[1] > p[1])
                && p[0] < a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1])),
            Self::Arc {center,radius,start,sweep,ends} => {
                if sweep.abs() == TAU {
                    let y = (p[1]-center[1])/radius;
                    if y.abs() >= 1. { return 0; }
                    let x = radius*(1.-y*y).sqrt();
                    return usize::from(p[0] < center[0]-x)+usize::from(p[0] < center[0]+x);
                }
                let lo = start.min(start+sweep);
                let hi = start.max(start+sweep);
                let mut angles = vec![lo];
                let first = ((lo-FRAC_PI_2)/PI).floor() as i32+1;
                let last = ((hi-FRAC_PI_2)/PI).ceil() as i32;
                for k in first..last { angles.push(FRAC_PI_2+f64::from(k)*PI); }
                angles.push(hi);
                let zs = if sweep > 0. { [ends[0][1],ends[1][1]] }
                    else { [ends[1][1],ends[0][1]] };
                angles.windows(2).enumerate().filter(|(i,a)| {
                    // Shared source vertices have exactly the same ordinate on both
                    // edges. Reconstructing one with sin(pi) can open a spurious ray gap.
                    let za = if *i == 0 { zs[0] } else { center[1]+radius*a[0].sin() };
                    let zb = if *i+2 == angles.len() { zs[1] }
                        else { center[1]+radius*a[1].sin() };
                    if (za > p[1]) == (zb > p[1]) { return false; }
                    let y = (p[1]-center[1])/radius;
                    let x = center[0]+radius*(1.-y*y).max(0.).sqrt()
                        * ((a[0]+a[1])/2.).cos().signum();
                    p[0] < x
                }).count()
            }
        }
    }
}

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
        let origin = first.origin; let axis = first.axis;
        let basis = face.plane()?.map(|p| sk.basis(p as usize)).unwrap_or(Basis::page());
        let normal = plane::cross(basis.u,basis.v);
        let radial = plane::cross(axis,normal);
        let coords = |p| [plane::dot(p,radial),plane::dot(p,axis)];
        let mut loops = patches.iter().map(|patches| patches.iter().map(|(p,e)| {
            Ok(match p.meridian {
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
                    Edge::Arc {center:coords(sub(center,origin)),radius:a[0].hypot(a[1]),
                        start:a[1].atan2(a[0]),sweep:sweep*(a[0]*b[1]-a[1]*b[0]).signum(),ends}
                }
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
        let p = [v[0].hypot(v[1]).hypot(v[2]),z];
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
