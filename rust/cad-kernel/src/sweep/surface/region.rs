//! Material membership of full revolutions, evaluated on analytic profile curves.
//! This is separate from the faceted CSG kernel: callers must choose which geometry
//! they mean. It does not silently substitute an analytic surface for a mesh face.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::*;
use std::f64::consts::{FRAC_PI_2,PI,TAU};

type P = [f64;2];
fn distance(a: P,b: P) -> f64 { (a[0]-b[0]).dhypot(a[1]-b[1]) }

/// One analytic profile edge in planar coordinates: a revolved meridian's
/// (radius, height), or a prism section's plane view coordinates. An arc's
/// `ends` are the exact source vertices, `ends[0]` at `start`; a full circle
/// has `sweep` of one turn and no seam. A conic's arc is the rational quadratic
/// Bézier over `a`, `m`, `b`, its middle weighed `w` (`brep::planar::Seg::Conic`).
#[derive(Clone,Debug)]
pub enum Edge {
    Line {a:P,b:P,axis:bool},
    Arc {center:P,radius:f64,start:f64,sweep:f64,ends:[P;2]},
    Conic {a:P,m:P,b:P,w:f64},
}

impl Edge {
    pub fn at(&self,t: f64) -> P {
        match *self {
            Self::Line {a,b,..} => std::array::from_fn(|i| a[i]+t*(b[i]-a[i])),
            Self::Arc {center,radius,start,sweep,..} => {
                let (s,c) = (start+t*sweep).dsin_cos();
                [center[0]+radius*c,center[1]+radius*s]
            }
            Self::Conic {..} => self.conic().point(t),
        }
    }

    /// A conic's arc as the planar kernel's step, which evaluates it.
    pub fn conic(&self) -> crate::planar::Seg {
        let Self::Conic {a,m,b,w} = *self else { unreachable!("a conic's step of a conic") };
        crate::planar::Seg::Conic {a,m,b,w}
    }

    fn radial_bounds(&self) -> [f64;2] {
        // a conic's arc lies within its control triangle
        if let Self::Conic {a,m,b,..} = *self {
            return [a[0].min(m[0]).min(b[0]),a[0].max(m[0]).max(b[0])];
        }
        let mut lo = self.at(0.)[0].min(self.at(1.)[0]);
        let mut hi = self.at(0.)[0].max(self.at(1.)[0]);
        if let Self::Arc {center,radius,start,sweep,..} = *self {
            for angle in [0.,PI] {
                if ((angle-start)*sweep.signum()).rem_euclid(TAU) <= sweep.abs() {
                    let r = center[0]+radius*angle.dcos();
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
            Self::Conic {a,m,b,..} => { a[0] = -a[0]; m[0] = -m[0]; b[0] = -b[0]; }
        }
    }

    // The closest point on an analytic finite meridian. Rotation is unrestricted,
    // so this is also the distance to its revolved surface in cylindrical space.
    pub fn distance(&self,p: P) -> f64 {
        match *self {
            Self::Line {a,b,axis} => {
                // An axis edge disappears in a full revolution; it is not a wall.
                if axis { return f64::INFINITY; }
                let d = [b[0]-a[0],b[1]-a[1]];
                let len = d[0].dhypot(d[1]);
                let t = (((p[0]-a[0])*(d[0]/len)+(p[1]-a[1])*(d[1]/len))/len)
                    .clamp(0.,1.);
                distance(p,self.at(t))
            }
            Self::Arc {center,radius,start,sweep,..} => {
                let q = [p[0]-center[0],p[1]-center[1]];
                let angle = q[1].datan2(q[0]);
                if ((angle-start)*sweep.signum()).rem_euclid(TAU) <= sweep.abs() {
                    (q[0].dhypot(q[1])-radius).abs()
                } else { distance(p,self.at(0.)).min(distance(p,self.at(1.))) }
            }
            Self::Conic {..} => {
                let seg = self.conic();
                distance(p,seg.point(seg.nearest(p))).min(distance(p,self.at(0.))).min(distance(p,self.at(1.)))
            }
        }
    }

    // Half-open horizontal ray crossings. Split arcs at their vertical extrema
    // analytically, so tangent rays and shared endpoints do not get counted twice.
    pub fn crossings(&self,p: P) -> usize {
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
                // the sweep's ends and the turning heights between, read as asked: a field asks of
                // every edge at every point it reads
                let first = ((lo-FRAC_PI_2)/PI).floor() as i32+1;
                let last = ((hi-FRAC_PI_2)/PI).ceil() as i32;
                let n = (last-first).max(0) as usize+2;
                let angle = |j: usize| if j == 0 { lo } else if j == n-1 { hi } else { FRAC_PI_2+f64::from(first+j as i32-1)*PI };
                let zs = if sweep > 0. { [ends[0][1],ends[1][1]] }
                    else { [ends[1][1],ends[0][1]] };
                (0..n-1).filter(|&i| {
                    let a = [angle(i),angle(i+1)];
                    // Shared source vertices have exactly the same ordinate on both
                    // edges. Reconstructing one with sin(pi) can open a spurious ray gap.
                    let za = if i == 0 { zs[0] } else { center[1]+radius*a[0].dsin() };
                    let zb = if i+2 == n { zs[1] }
                        else { center[1]+radius*a[1].dsin() };
                    if (za > p[1]) == (zb > p[1]) { return false; }
                    let y = (p[1]-center[1])/radius;
                    let x = center[0]+radius*(1.-y*y).max(0.).sqrt()
                        * ((a[0]+a[1])/2.).dcos().signum();
                    p[0] < x
                }).count()
            }
            Self::Conic {a,m,b,w} => {
                // in powers of t, `w(y1 − y0)(1 − t)² + (y2 − y0)t(1 − t) + w(y2 − y1)t²` is half
                // the numerator of y′ (its cubic terms cancel) and `(y0 − h)(1 − t)² +
                // 2w(y1 − h)t(1 − t) + (y2 − h)t²` that of y − h: split where the height turns,
                // each stretch crossed at most once, its ends' heights read as a line's are (its
                // vertices exactly as written)
                let powers = |e0: f64,e1: f64,e2: f64| (e0,e1-2.*e0,e0-e1+e2);
                let (c0,c1,c2) = powers(w*(m[1]-a[1]),b[1]-a[1],w*(b[1]-m[1]));
                let mut cuts = vec![0.];
                cuts.extend(crate::util::roots::quadratic(c2,c1,c0).into_iter().filter(|&t| t > 0. && t < 1.));
                cuts.push(1.);
                let (h0,h1,h2) = powers(a[1]-p[1],2.*w*(m[1]-p[1]),b[1]-p[1]);
                let seg = self.conic();
                cuts.windows(2).filter(|t| {
                    if (seg.point(t[0])[1] > p[1]) == (seg.point(t[1])[1] > p[1]) { return false; }
                    // where the stretch meets the ray's height: the root within it (to roundoff)
                    let off = |x: f64| (x.clamp(t[0],t[1])-x).abs();
                    let at = crate::util::roots::quadratic(h2,h1,h0).into_iter()
                        .min_by(|x,y| off(*x).total_cmp(&off(*y)))
                        .map_or(0.5*(t[0]+t[1]),|x| x.clamp(t[0],t[1]));
                    p[0] < seg.point(at)[0]
                }).count()
            }
        }
    }
}

fn turn(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn gap(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }

impl Edge {
    // Connectivity comes from the source vertices. Reconstructing a constrained
    // arc endpoint with radius and trigonometry adds source-solve and rounding
    // residuals; those must not turn one shared vertex into two separate ones.
    pub fn endpoints(&self) -> [P;2] {
        match *self {
            Self::Line {a,b,..} => [a,b],
            Self::Arc {sweep,ends,..} if sweep.abs() < TAU => ends,
            Self::Arc {..} => [self.at(0.),self.at(1.)],
            Self::Conic {a,b,..} => [a,b],
        }
    }
    pub fn reversed(mut self) -> Self {
        match &mut self {
            Self::Line {a,b,..} => std::mem::swap(a,b),
            Self::Arc {start,sweep,ends,..} => {
                *start += *sweep; *sweep = -*sweep; ends.swap(0,1);
            }
            Self::Conic {a,b,..} => std::mem::swap(a,b),
        }
        self
    }
    pub fn tangent(&self,t: f64) -> P {
        match *self {
            Self::Line {a,b,..} => gap(b,a),
            Self::Arc {start,sweep,..} => {
                let (s,c) = (start+t*sweep).dsin_cos();
                [-s*sweep.signum(),c*sweep.signum()]
            }
            Self::Conic {..} => self.conic().tangent(t),
        }
    }
    pub fn area_twice(&self) -> f64 {
        match *self {
            Self::Line {a,b,..} => turn(a,b),
            Self::Arc {center,radius,sweep,..} =>
                turn(center,gap(self.at(1.),self.at(0.)))+radius*radius*sweep,
            // ∫ x dy − y dx by Gauss–Legendre: a sign for the winding
            Self::Conic {..} => crate::util::quadrature::GAUSS8.iter().map(|&(x,w)| {
                let t = 0.5+0.5*x;
                w*turn(self.at(t),self.tangent(t))
            }).sum::<f64>()/2.,
        }
    }
    // The farthest boundary point from planar zero; material lies within the
    // convex hull of its boundary, hence inside this disk.
    pub fn reach(&self) -> f64 {
        match *self {
            Self::Line {a,b,..} => a[0].dhypot(a[1]).max(b[0].dhypot(b[1])),
            Self::Arc {center,radius,..} => center[0].dhypot(center[1])+radius,
            // within its control triangle
            Self::Conic {a,m,b,..} => a[0].dhypot(a[1]).max(m[0].dhypot(m[1])).max(b[0].dhypot(b[1])),
        }
    }
    // A box holding the edge: its ends, an arc's whole circle, a conic's control triangle.
    pub fn boxed(&self) -> [P;2] {
        match *self {
            Self::Line {a,b,..} => [[a[0].min(b[0]),a[1].min(b[1])],[a[0].max(b[0]),a[1].max(b[1])]],
            Self::Arc {center,radius,..} => [[center[0]-radius,center[1]-radius],[center[0]+radius,center[1]+radius]],
            Self::Conic {a,m,b,..} => [[a[0].min(m[0]).min(b[0]),a[1].min(m[1]).min(b[1])],
                [a[0].max(m[0]).max(b[0]),a[1].max(m[1]).max(b[1])]],
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
    /// A point of the axis.
    pub fn origin(&self) -> V { self.origin }
    /// The axis's unit direction.
    pub fn axis(&self) -> V { self.axis }
    /// The profile's loops in (radius, height), the loop enclosing most first.
    pub fn loops(&self) -> &[Vec<Edge>] { &self.loops }
    /// The axis tolerance the snapshot was read with.
    pub fn axis_tolerance(&self) -> f64 { self.axis_tolerance }
}

impl RevolvedRegion {
    /// `axis_tolerance` is an absolute length used to recognize solved axis edges.
    /// Their endpoints within this tolerance are put on the axis. Other edges retain
    /// their solved geometry. Read again after modifying or resolving the sketch.
    pub fn read<S: Source+?Sized>(source: &S,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        source.revolved_region(solid,axis_tolerance)
    }

    /// A full revolution's material from its profile's faces, loop by loop, the outer first:
    /// each face with the ends of the arc it was turned from, where it is an arc short of a
    /// circle (world points, its start then its end), and `normal` the profile plane's.
    pub fn of_profile(patches: Vec<Vec<(RevolvedSurface,Option<[V;2]>)>>,normal: V,axis_tolerance: f64)
        -> Result<Self,String> {
        let first = &patches.first().and_then(|p| p.first()).ok_or("empty profile")?.0;
        let origin = first.origin; let axis = first.axis;
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
                    let ends = match e {
                        Some(ends) => ends.map(|p| coords(sub(p,origin))),
                        None => [[0.;2];2], // Full circles have no ray seam.
                    };
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
    pub fn of_meridian(m: &RegionMeridian,axis_tolerance: f64) -> Self {
        use crate::planar::Seg;
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
