//! Where two surfaces meet, in closed form where it has one: two planes in a line; surfaces of
//! revolution about one axis (a plane square to it, a cylinder, a cone, a sphere centred on it, a
//! torus) in circles, found where their meridians cross in one half-plane; a plane and a cylinder
//! in lines or an ellipse; a plane and a sphere in a circle; parallel cylinders in lines. The
//! rest is left to be traced (`Ssi::Traced`).
use super::geom::{Curve,Frame,Surface,V};
use crate::space::{add,cross,dot,norm,scale,sub};

fn unit(a: V) -> V { scale(a,1./norm(a)) }

/// What two surfaces share.
#[derive(Clone,Debug,PartialEq)]
pub enum Ssi {
    /// Whole curves (a line unbounded, a circle or an ellipse closed), possibly none.
    Curves(Vec<Curve>),
    /// The two are one surface.
    Same,
    /// No closed form here: the curve must be traced.
    Traced,
}

/// A meridian: a surface of revolution's section in the half-plane `ρ ≥ 0` of its axis, in
/// `(ρ, z)` with `z` measured along the shared axis from a shared origin.
#[derive(Clone,Copy,Debug)]
enum Meridian {
    /// `z = h`: a plane square to the axis.
    Level(f64),
    /// The ray from `p` along the unit `d` (`d[1] > 0` or a vertical line `ρ = r`), both ways.
    Line { p: [f64;2],d: [f64;2] },
    /// A circle about `c` of radius `r`.
    Circle { c: [f64;2],r: f64 },
}

/// A surface's meridian about the axis through `o` along the unit `z`, where it is a surface of
/// revolution about that axis (to `tol`).
fn meridian(s: &Surface,o: V,z: V,tol: f64) -> Option<Meridian> {
    let f = s.frame();
    let height = |p: V| dot(sub(p,o),z);
    let on_axis = |p: V| norm(cross(sub(p,o),z)) <= tol;
    let aligned = norm(cross(f.z,z)) <= 1e-12;
    let sign = dot(f.z,z).signum();
    match *s {
        Surface::Plane(_) => aligned.then(|| Meridian::Level(height(f.o))),
        Surface::Sphere(_,r) => on_axis(f.o).then(|| Meridian::Circle {c:[0.,height(f.o)],r}),
        _ if !(aligned && on_axis(f.o)) => None,
        Surface::Cylinder(_,r) => Some(Meridian::Line {p:[r,height(f.o)],d:[0.,1.]}),
        Surface::Cone(_,r,a) => {
            // (r + v sin α, v cos α) in the cone's own (ρ, z); its z may run against the axis
            Some(Meridian::Line {p:[r,height(f.o)],d:[a.sin(),a.cos()*sign]})
        }
        Surface::Torus(_,big,r) => Some(Meridian::Circle {c:[big,height(f.o)],r}),
    }
}

/// Where two meridians cross in `ρ ≥ −tol`, as `(ρ, z)`; `None` where they coincide.
fn cross2(a: Meridian,b: Meridian,tol: f64) -> Option<Vec<[f64;2]>> {
    use Meridian::*;
    let line_line = |p: [f64;2],d: [f64;2],q: [f64;2],e: [f64;2]| -> Option<Vec<[f64;2]>> {
        let det = d[0]*e[1]-d[1]*e[0];
        let w = [q[0]-p[0],q[1]-p[1]];
        if det.abs() <= 1e-14 {
            // parallel: the same line, or none
            return if (w[0]*d[1]-w[1]*d[0]).abs() <= tol { None } else { Some(vec![]) }
        }
        let s = (w[0]*e[1]-w[1]*e[0])/det;
        Some(vec![[p[0]+s*d[0],p[1]+s*d[1]]])
    };
    let line_circle = |p: [f64;2],d: [f64;2],c: [f64;2],r: f64| -> Vec<[f64;2]> {
        let w = [p[0]-c[0],p[1]-c[1]];
        let bb = w[0]*d[0]+w[1]*d[1];
        let cc = w[0]*w[0]+w[1]*w[1]-r*r;
        let disc = bb*bb-cc;
        // a line within `tol` of tangent touches once
        let foot = [p[0]-bb*d[0],p[1]-bb*d[1]];
        let gap = ((foot[0]-c[0]).hypot(foot[1]-c[1])-r).abs();
        if gap <= tol { return vec![foot] }
        if disc < 0. { return vec![] }
        let root = disc.sqrt();
        [-bb-root,-bb+root].iter().map(|s| [p[0]+s*d[0],p[1]+s*d[1]]).collect()
    };
    let pts = match (a,b) {
        (Level(h),Level(k)) => return if (h-k).abs() <= tol { None } else { Some(vec![]) },
        (Level(h),Line {p,d}) | (Line {p,d},Level(h)) => line_line([0.,h],[1.,0.],p,d)?,
        (Level(h),Circle {c,r}) | (Circle {c,r},Level(h)) => line_circle([0.,h],[1.,0.],c,r),
        (Line {p,d},Line {p: q,d: e}) => line_line(p,d,q,e)?,
        (Line {p,d},Circle {c,r}) | (Circle {c,r},Line {p,d}) => line_circle(p,d,c,r),
        (Circle {c,r},Circle {c: k,r: s}) => {
            let d = (k[0]-c[0]).hypot(k[1]-c[1]);
            if d <= tol && (r-s).abs() <= tol { return None }
            if d <= tol || d > r+s+tol || d < (r-s).abs()-tol { vec![] }
            else {
                let a = ((r*r-s*s+d*d)/(2.*d)).clamp(-r,r);
                let h = (r*r-a*a).max(0.).sqrt();
                let u = [(k[0]-c[0])/d,(k[1]-c[1])/d];
                let m = [c[0]+a*u[0],c[1]+a*u[1]];
                if h <= tol { vec![m] } else { vec![[m[0]-h*u[1],m[1]+h*u[0]],[m[0]+h*u[1],m[1]-h*u[0]]] }
            }
        }
    };
    Some(pts.into_iter().filter(|q| q[0] >= -tol).collect())
}

/// Whether two surfaces are the same surface, to `tol`.
pub fn same(a: &Surface,b: &Surface,tol: f64) -> bool {
    let (f,g) = (a.frame(),b.frame());
    let parallel = norm(cross(f.z,g.z)) <= 1e-12;
    let on_axis = norm(cross(sub(g.o,f.o),f.z)) <= tol;
    match (*a,*b) {
        (Surface::Plane(_),Surface::Plane(_)) => parallel && dot(sub(g.o,f.o),f.z).abs() <= tol,
        (Surface::Cylinder(_,r),Surface::Cylinder(_,s)) => parallel && on_axis && (r-s).abs() <= tol,
        (Surface::Sphere(_,r),Surface::Sphere(_,s)) => norm(sub(g.o,f.o)) <= tol && (r-s).abs() <= tol,
        (Surface::Torus(_,big,r),Surface::Torus(_,bog,s)) =>
            parallel && norm(sub(g.o,f.o)) <= tol && (big-bog).abs() <= tol && (r-s).abs() <= tol,
        (Surface::Cone(_,r,a1),Surface::Cone(_,s,a2)) => {
            if !(parallel && on_axis) { return false }
            // the same apex, half-angle and opening (a cone opens along z where its α is positive)
            let apex = |f: &Frame,r: f64,a: f64| sub(f.o,scale(f.z,r*a.cos()/a.sin()));
            let opens = |f: &Frame,a: f64| scale(f.z,a.signum());
            norm(sub(apex(f,r,a1),apex(g,s,a2))) <= tol && (a1.abs()-a2.abs()).abs() <= 1e-12
                && dot(opens(f,a1),opens(g,a2)) > 0.
        }
        _ => false,
    }
}

/// Where two surfaces meet (`tol` a length).
pub fn intersect(a: &Surface,b: &Surface,tol: f64) -> Ssi {
    if same(a,b,tol) { return Ssi::Same }
    let (f,g) = (*a.frame(),*b.frame());
    // two planes
    if let (Surface::Plane(_),Surface::Plane(_)) = (a,b) {
        let d = cross(f.z,g.z);
        if norm(d) <= 1e-12 { return Ssi::Curves(vec![]) }
        let d = unit(d);
        // the point of both nearest f's origin: solve n1·x = h1, n2·x = h2, d·x = d·o
        let (h1,h2) = (dot(f.z,f.o),dot(g.z,g.o));
        let m = [f.z,g.z,d];
        let rhs = [h1,h2,dot(d,f.o)];
        let p = solve3(m,rhs);
        return Ssi::Curves(vec![Curve::Line {p,d}])
    }
    // one axis for both: a surface of revolution's, or a plane's normal through the other's
    for (s,t) in [(a,b),(b,a)] {
        let Some((o,z)) = axis_of(s,t) else { continue };
        let (Some(ma),Some(mb)) = (meridian(s,o,z,tol),meridian(t,o,z,tol)) else { continue };
        let Some(points) = cross2(ma,mb,tol) else { return Ssi::Same };
        let mut out = Vec::new();
        for [rho,h] in points {
            // a circle of no radius is a point on the axis: a touch, not a curve
            if rho <= tol { continue }
            out.push(Curve::Circle(Frame::about(add(o,scale(z,h)),z),rho));
        }
        return Ssi::Curves(out)
    }
    match (*a,*b) {
        (Surface::Plane(_),Surface::Cylinder(..)) | (Surface::Cylinder(..),Surface::Plane(_)) => {
            let (pl,cy) = if matches!(a,Surface::Plane(_)) { (f,*b) } else { (g,*a) };
            let Surface::Cylinder(c,r) = cy else { unreachable!() };
            let (n,h) = (pl.z,dot(pl.z,pl.o));
            let cos = dot(n,c.z);
            if cos.abs() <= 1e-12 {
                // the plane along the axis: lines at the chord's ends
                let dist = h-dot(n,c.o);
                if dist.abs() > r+tol { return Ssi::Curves(vec![]) }
                let foot = add(c.o,scale(n,dist));
                let w = unit(cross(c.z,n));
                let half = (r*r-dist*dist).max(0.).sqrt();
                if half <= tol { return Ssi::Curves(vec![Curve::Line {p:foot,d:c.z}]) }
                return Ssi::Curves(vec![Curve::Line {p:add(foot,scale(w,half)),d:c.z},Curve::Line {p:sub(foot,scale(w,half)),d:c.z}])
            }
            // oblique: an ellipse about where the axis pierces the plane, its minor axis the
            // radius square to both
            let centre = add(c.o,scale(c.z,(h-dot(n,c.o))/cos));
            let minor = cross(c.z,n);
            if norm(minor) <= 1e-12 { return Ssi::Curves(vec![Curve::Circle(Frame::new(centre,c.z,c.x),r)]) }
            let minor = unit(minor);
            let major = cross(minor,n);
            Ssi::Curves(vec![Curve::Ellipse(Frame::new(centre,cross(major,minor),major),r/cos.abs(),r)])
        }
        (Surface::Plane(_),Surface::Sphere(..)) | (Surface::Sphere(..),Surface::Plane(_)) => {
            let (pl,sp) = if matches!(a,Surface::Plane(_)) { (f,*b) } else { (g,*a) };
            let Surface::Sphere(c,r) = sp else { unreachable!() };
            let dist = dot(pl.z,sub(c.o,pl.o));
            if dist.abs() > r+tol { return Ssi::Curves(vec![]) }
            let rho = (r*r-dist*dist).max(0.).sqrt();
            if rho <= tol { return Ssi::Curves(vec![]) }
            Ssi::Curves(vec![Curve::Circle(Frame::new(sub(c.o,scale(pl.z,dist)),pl.z,pl.x),rho)])
        }
        (Surface::Cylinder(_,r),Surface::Cylinder(_,s)) if norm(cross(f.z,g.z)) <= 1e-12 => {
            // parallel axes: the sections' circles meet in the lines' feet
            let w = sub(g.o,f.o);
            let w = sub(w,scale(f.z,dot(w,f.z)));
            let d = norm(w);
            if d > r+s+tol || d < (r-s).abs()-tol || d <= tol { return Ssi::Curves(vec![]) }
            let u = unit(w);
            let v = cross(f.z,u);
            let a = ((r*r-s*s+d*d)/(2.*d)).clamp(-r,r);
            let h = (r*r-a*a).max(0.).sqrt();
            let m = add(f.o,scale(u,a));
            if h <= tol { return Ssi::Curves(vec![Curve::Line {p:m,d:f.z}]) }
            Ssi::Curves(vec![Curve::Line {p:add(m,scale(v,h)),d:f.z},Curve::Line {p:sub(m,scale(v,h)),d:f.z}])
        }
        _ => Ssi::Traced,
    }
}

/// An axis both surfaces may turn about: a surface of revolution's own (a sphere lends the other's),
/// or a plane's normal through the other's centre.
fn axis_of(s: &Surface,t: &Surface) -> Option<(V,V)> {
    match (s,t) {
        (Surface::Plane(_),_) | (Surface::Sphere(..),_) => None,
        _ => { let f = s.frame(); Some((f.o,f.z)) }
    }.or_else(|| match (s,t) {
        (Surface::Plane(p),Surface::Sphere(c,_)) => Some((c.o,p.z)),
        (Surface::Sphere(c,_),Surface::Sphere(d,_)) => {
            let w = sub(d.o,c.o);
            (norm(w) > 0.).then(|| (c.o,unit(w)))
        }
        (Surface::Sphere(c,_),Surface::Plane(p)) => Some((c.o,p.z)),
        _ => None,
    })
}

/// `m x = rhs` for a 3×3 system by Cramer's rule.
fn solve3(m: [V;3],rhs: V) -> V {
    let det = dot(m[0],cross(m[1],m[2]));
    let col = |k: usize| -> f64 {
        let mut a = m;
        for i in 0..3 { a[i][k] = rhs[i]; }
        dot(a[0],cross(a[1],a[2]))
    };
    [col(0)/det,col(1)/det,col(2)/det]
}
