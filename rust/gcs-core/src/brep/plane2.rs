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
use crate::fmath::Det;

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
