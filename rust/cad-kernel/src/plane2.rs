//! The plane's own arithmetic, written once for the kernel's two-dimensional readers: the vector
//! helpers, and where lines and circles meet — the planar Booleans' steps (`planar`), a
//! meridian's corners (`corners`), two meridians crossing in a half-plane (`ssi`), and the
//! cross-section of two parallel cylinders or of a plane along a cylinder (`ssi`, `query`).
//!
//! **A touch is judged by the gap** (#218): two curves touch where they stand within the tolerance
//! of one another, never by the half chord, which is the gap's square root and sees a gap of
//! roundoff as two meetings `√(2rε)` apart.  `Touch::Gap` is that rule, every classifying reader's
//! (`planar`, `ssi`, `query`); `Touch::Exact` is the arithmetic's own roots, for `corners`, which
//! builds a round's centre rather than classifying a meeting.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use std::f64::consts::{FRAC_PI_2,PI,TAU};

pub type P = [f64;2];

pub fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
pub fn add(a: P,b: P) -> P { [a[0]+b[0],a[1]+b[1]] }
pub fn scale(a: P,k: f64) -> P { [a[0]*k,a[1]*k] }
pub fn dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1] }
pub fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
/// The length, by `hypot`.
pub fn norm(a: P) -> f64 { a[0].dhypot(a[1]) }
pub fn dist(a: P,b: P) -> f64 { norm(sub(a,b)) }
/// The unit vector along `a`, as `a × (1 / |a|)` (NaN for a zero vector).
pub fn unit(a: P) -> P { scale(a,1./norm(a)) }
/// `a` turned a quarter counter-clockwise.
pub fn left(a: P) -> P { [-a[1],a[0]] }

/// How a meeting tells a touch from a crossing and from a miss.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Touch {
    /// No tolerance: the roots the arithmetic gives (a tangency two equal points, a near miss
    /// none). Lines are parallel below `1e-15` of their unit directions' cross product.
    /// `corners`' offsets, where a round's centre is.
    Exact,
    /// A length: two curves touch where they stand within `tol` of one another; lines (unit
    /// directions) are parallel within `1e-14`.
    Gap(f64),
}

impl Touch {
    fn tol(self) -> f64 {
        match self { Touch::Exact => 0.,Touch::Gap(t) => t }
    }
}

/// A line or a circle of the plane: what meets.  A line's direction is a unit one.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Carrier {
    Line { p: P,d: P },
    Circle { c: P,r: f64 },
}

/// Where two carriers meet; none where they are one carrier (lines within the rule's tolerance of
/// one another, circles alike in centre and radius).  A line meets a circle in the order the
/// arithmetic finds them along the line; circles cross to the left of the line between their
/// centres first.
pub fn meet(a: Carrier,b: Carrier,touch: Touch) -> Option<Vec<P>> {
    match (a,b) {
        (Carrier::Line {p,d},Carrier::Line {p: q,d: e}) => line_line(p,d,q,e,touch),
        (Carrier::Line {p,d},Carrier::Circle {c,r}) | (Carrier::Circle {c,r},Carrier::Line {p,d}) =>
            Some(line_circle(p,d,c,r,touch)),
        (Carrier::Circle {c,r},Carrier::Circle {c: k,r: s}) => circle_circle(c,r,k,s,touch),
    }
}

/// Where the line through `p` along `d` crosses the line through `q` along `e`: the point along
/// the first; none where they are one line.
pub fn line_line(p: P,d: P,q: P,e: P,touch: Touch) -> Option<Vec<P>> {
    let den = cross(d,e);
    let parallel = match touch {
        Touch::Exact => den.abs() < 1e-15,
        Touch::Gap(_) => den.abs() <= 1e-14,
    };
    let w = sub(q,p);
    if parallel { return if cross(w,d).abs() <= touch.tol() { None } else { Some(vec![]) } }
    let s = cross(w,e)/den;
    Some(vec![[p[0]+d[0]*s,p[1]+d[1]*s]])
}

/// Where the line through `p` along `d` meets the circle about `c` of radius `r`, in order along
/// the line.
pub fn line_circle(p: P,d: P,c: P,r: f64,touch: Touch) -> Vec<P> {
    let f = sub(p,c);
    let (b,k) = (dot(f,d),dot(f,f)-r*r);
    let disc = b*b-k;
    if let Touch::Gap(tol) = touch {
        // a line within `tol` of tangent touches once
        let foot = [p[0]-b*d[0],p[1]-b*d[1]];
        if (dist(foot,c)-r).abs() <= tol { return vec![foot] }
    }
    if disc < 0. { return vec![] }
    let w = disc.sqrt();
    [-b-w,-b+w].iter().map(|s| [p[0]+d[0]*s,p[1]+d[1]*s]).collect()
}

/// Where the circle about `c` of radius `r` meets the one about `k` of radius `s`; none where
/// they are one circle.
pub fn circle_circle(c: P,r: f64,k: P,s: f64,touch: Touch) -> Option<Vec<P>> {
    let d = dist(c,k);
    let (a,h) = match circles(d,r,s,touch) {
        Circles::Same => return None,
        Circles::Miss => return Some(vec![]),
        Circles::Touch(a) => (a,None),
        Circles::Cross(a,h) => (a,Some(h)),
    };
    let u = if touch == Touch::Exact { unit(sub(k,c)) } else { [(k[0]-c[0])/d,(k[1]-c[1])/d] };
    let m = [c[0]+a*u[0],c[1]+a*u[1]];
    Some(match h {
        None => vec![m],
        Some(h) => vec![[m[0]-h*u[1],m[1]+h*u[0]],[m[0]+h*u[1],m[1]-h*u[0]]],
    })
}

/// How two circles meet, their centres `d` apart.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Circles {
    /// One circle.
    Same,
    Miss,
    /// At the point `a` along from the first centre toward the second.
    Touch(f64),
    /// At the two points `a` along from the first centre toward the second and `h` either side.
    Cross(f64,f64),
}

/// How the circles of radii `r` and `s`, their centres `d` apart, meet.
pub fn circles(d: f64,r: f64,s: f64,touch: Touch) -> Circles {
    let tol = match touch {
        Touch::Exact => {
            if d == 0. { return Circles::Miss }
            let a = (d*d+r*r-s*s)/(2.*d);
            let h2 = r*r-a*a;
            return if h2 < 0. { Circles::Miss } else { Circles::Cross(a,h2.sqrt()) }
        }
        Touch::Gap(tol) => tol,
    };
    if d <= tol && (r-s).abs() <= tol { return Circles::Same }
    if d <= tol || d > r+s+tol || d < (r-s).abs()-tol { return Circles::Miss }
    let a = ((r*r-s*s+d*d)/(2.*d)).clamp(-r,r);
    let h = (r*r-a*a).max(0.).sqrt();
    let touches = h <= tol || (r+s-d).abs() <= tol || (d-(r-s).abs()).abs() <= tol;
    if touches { Circles::Touch(a) } else { Circles::Cross(a,h) }
}

/// How a line `h` from a circle's centre meets it.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Chord {
    Miss,
    /// At its foot.
    Touch,
    /// At the two points `half` either side of its foot.
    Cross(f64),
}

/// How the line `h` (signed) from the centre of the circle of radius `r` meets it.
pub fn chord(h: f64,r: f64,touch: Touch) -> Chord {
    match touch {
        Touch::Exact => {
            let half2 = r*r-h*h;
            if half2 < 0. { Chord::Miss } else { Chord::Cross(half2.sqrt()) }
        }
        Touch::Gap(tol) => {
            if h.abs() > r+tol { return Chord::Miss }
            let half = (r*r-h*h).max(0.).sqrt();
            if half <= tol || (r-h.abs()).abs() <= tol { Chord::Touch } else { Chord::Cross(half) }
        }
    }
}

/// The distance between two points of the plane, by `hypot`.
fn edge_distance(a: [f64;2],b: [f64;2]) -> f64 { (a[0]-b[0]).dhypot(a[1]-b[1]) }

/// One analytic profile edge in planar coordinates: a revolved meridian's
/// (radius, height), or a prism section's plane view coordinates. An arc's
/// `ends` are the exact source vertices, `ends[0]` at `start`; a full circle
/// has `sweep` of one turn and no seam. A conic's arc is the rational quadratic
/// Bézier over `a`, `m`, `b`, its middle weighed `w` (`brep::planar::Seg::Conic`).
#[derive(Clone,Debug)]
pub enum Edge {
    Line {a:[f64;2],b:[f64;2],axis:bool},
    Arc {center:[f64;2],radius:f64,start:f64,sweep:f64,ends:[[f64;2];2]},
    Conic {a:[f64;2],m:[f64;2],b:[f64;2],w:f64},
}

impl Edge {
    pub fn at(&self,t: f64) -> [f64;2] {
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

    pub fn radial_bounds(&self) -> [f64;2] {
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

    pub fn reflect(&mut self) {
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
    pub fn distance(&self,p: [f64;2]) -> f64 {
        match *self {
            Self::Line {a,b,axis} => {
                // An axis edge disappears in a full revolution; it is not a wall.
                if axis { return f64::INFINITY; }
                let d = [b[0]-a[0],b[1]-a[1]];
                let len = d[0].dhypot(d[1]);
                let t = (((p[0]-a[0])*(d[0]/len)+(p[1]-a[1])*(d[1]/len))/len)
                    .clamp(0.,1.);
                edge_distance(p,self.at(t))
            }
            Self::Arc {center,radius,start,sweep,..} => {
                let q = [p[0]-center[0],p[1]-center[1]];
                let angle = q[1].datan2(q[0]);
                if ((angle-start)*sweep.signum()).rem_euclid(TAU) <= sweep.abs() {
                    (q[0].dhypot(q[1])-radius).abs()
                } else { edge_distance(p,self.at(0.)).min(edge_distance(p,self.at(1.))) }
            }
            Self::Conic {..} => {
                let seg = self.conic();
                edge_distance(p,seg.point(seg.nearest(p))).min(edge_distance(p,self.at(0.))).min(edge_distance(p,self.at(1.)))
            }
        }
    }

    // Half-open horizontal ray crossings. Split arcs at their vertical extrema
    // analytically, so tangent rays and shared endpoints do not get counted twice.
    pub fn crossings(&self,p: [f64;2]) -> usize {
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

fn edge_cross(a: [f64;2],b: [f64;2]) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn edge_delta(a: [f64;2],b: [f64;2]) -> [f64;2] { [a[0]-b[0],a[1]-b[1]] }

impl Edge {
    // Connectivity comes from the source vertices. Reconstructing a constrained
    // arc endpoint with radius and trigonometry adds source-solve and rounding
    // residuals; those must not turn one shared vertex into two separate ones.
    pub fn endpoints(&self) -> [[f64;2];2] {
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
    pub fn tangent(&self,t: f64) -> [f64;2] {
        match *self {
            Self::Line {a,b,..} => edge_delta(b,a),
            Self::Arc {start,sweep,..} => {
                let (s,c) = (start+t*sweep).dsin_cos();
                [-s*sweep.signum(),c*sweep.signum()]
            }
            Self::Conic {..} => self.conic().tangent(t),
        }
    }
    pub fn area_twice(&self) -> f64 {
        match *self {
            Self::Line {a,b,..} => edge_cross(a,b),
            Self::Arc {center,radius,sweep,..} =>
                edge_cross(center,edge_delta(self.at(1.),self.at(0.)))+radius*radius*sweep,
            // ∫ x dy − y dx by Gauss–Legendre: a sign for the winding
            Self::Conic {..} => crate::util::quadrature::GAUSS8.iter().map(|&(x,w)| {
                let t = 0.5+0.5*x;
                w*edge_cross(self.at(t),self.tangent(t))
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
    pub fn boxed(&self) -> [[f64;2];2] {
        match *self {
            Self::Line {a,b,..} => [[a[0].min(b[0]),a[1].min(b[1])],[a[0].max(b[0]),a[1].max(b[1])]],
            Self::Arc {center,radius,..} => [[center[0]-radius,center[1]-radius],[center[0]+radius,center[1]+radius]],
            Self::Conic {a,m,b,..} => [[a[0].min(m[0]).min(b[0]),a[1].min(m[1]).min(b[1])],
                [a[0].max(m[0]).max(b[0]),a[1].max(m[1]).max(b[1])]],
        }
    }
}
