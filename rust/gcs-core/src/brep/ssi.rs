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
#[derive(Clone,Debug)]
enum Meridian {
    /// `z = h`: a plane square to the axis.
    Level(f64),
    /// The ray from `p` along the unit `d` (`d[1] > 0` or a vertical line `ρ = r`), both ways.
    Line { p: [f64;2],d: [f64;2] },
    /// A circle about `c` of radius `r`.
    Circle { c: [f64;2],r: f64 },
    /// A B-spline turned about the axis, and the point and direction that axis runs through.
    Spline(std::sync::Arc<super::nurbs::BSpline>,V,V),
}

impl Meridian {
    /// A signed distance in `(ρ, z)` whose zero set is this meridian (not a spline's).
    fn implicit(&self,[r,z]: [f64;2]) -> f64 {
        match *self {
            Meridian::Level(h) => z-h,
            Meridian::Line {p,d} => (r-p[0])*d[1]-(z-p[1])*d[0],
            Meridian::Circle {c,r: rad} => (r-c[0]).hypot(z-c[1])-rad,
            Meridian::Spline(..) => unreachable!("a spline meridian has no closed-form distance"),
        }
    }
}

/// Where a spline meridian crosses or touches another (not a spline), in `(ρ, z)`: every sign
/// change of the other's distance along it, refined, and every sampled minimum of its size that
/// comes within `tol`.
fn spline_cross(s: &super::nurbs::BSpline,o: V,z: V,other: &Meridian,tol: f64) -> Vec<[f64;2]> {
    let rz = |t: f64| { let q = sub(s.point(t),o); let h = dot(q,z); [norm(sub(q,scale(z,h))),h] };
    let g = |t: f64| other.implicit(rz(t));
    let [a,b] = s.domain();
    let mut cuts = vec![a];
    cuts.extend(s.breaks([a,b]));
    cuts.push(b);
    let mut ts: Vec<f64> = Vec::new();
    for w in cuts.windows(2) { for j in 0..32 { ts.push(w[0]+(w[1]-w[0])*j as f64/32.); } }
    ts.push(b);
    let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
    let mut out: Vec<[f64;2]> = Vec::new();
    let push = |p: [f64;2],out: &mut Vec<[f64;2]>| if !out.iter().any(|q| (q[0]-p[0]).hypot(q[1]-p[1]) <= tol) { out.push(p) };
    for i in 0..ts.len()-1 {
        let (x,y) = (gs[i],gs[i+1]);
        if x.abs() <= tol { push(rz(ts[i]),&mut out); }
        if x.abs() > tol && y.abs() > tol && (x < 0.) != (y < 0.) {
            let t = crate::roots::bracketed_root(|t| Some(g(t)),ts[i],x,ts[i+1],y,1e-15*(b-a));
            push(rz(t),&mut out);
        } else if i+2 < ts.len() && y.abs() < x.abs() && y.abs() < gs[i+2].abs() && (x < 0.) == (y < 0.) && y.abs() > tol {
            let (t,v) = crate::roots::brent(&|t| g(t).abs(),ts[i],ts[i+2],1e-15*(b-a),200,|_,_| false);
            if v <= tol { push(rz(t),&mut out); }
        }
    }
    if gs[ts.len()-1].abs() <= tol { push(rz(b),&mut out); }
    out
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
        Surface::Revolution(_,ref c) => match &**c { Curve::BSpline(b) => Some(Meridian::Spline(b.clone(),o,z)),_ => None },
        Surface::Extrusion(..) | Surface::Blend(..) => None,
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
        (Spline(..),_) | (_,Spline(..)) => unreachable!("a spline meridian is crossed by `spline_cross`"),
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

/// Whether two swept curves are one curve once `seen` takes out the motion that sweeps them: the
/// same degree and knots and poles within `tol`, or else equal outright.
fn same_curve(c: &Curve,d: &Curve,tol: f64,seen: &dyn Fn(V) -> V) -> bool {
    match (c,d) {
        (Curve::BSpline(a),Curve::BSpline(b)) => a.degree == b.degree && a.knots == b.knots
            && a.poles.len() == b.poles.len() && a.poles.iter().zip(&b.poles).all(|(&p,&q)| norm(sub(seen(p),seen(q))) <= tol),
        _ => c == d,
    }
}

/// Whether two surfaces are the same surface, to `tol`.
pub fn same(a: &Surface,b: &Surface,tol: f64) -> bool {
    let (f,g) = (a.frame(),b.frame());
    let parallel = norm(cross(f.z,g.z)) <= 1e-12;
    let on_axis = norm(cross(sub(g.o,f.o),f.z)) <= tol;
    match (a.clone(),b.clone()) {
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
        // a curve swept along one direction, or turned about one axis, as the other is (a face
        // of the same profile, copied or cut): the same curve and the same line
        // (moved along the sweep, or turned about the axis, a B-spline's poles are where they were
        // with that motion taken out: along the direction, or to their meridian (ρ, z))
        (Surface::Extrusion(_,c),Surface::Extrusion(_,d)) => parallel
            && same_curve(&c,&d,tol,&|p| sub(p,scale(f.z,dot(p,f.z)))),
        (Surface::Revolution(_,c),Surface::Revolution(_,d)) => parallel && on_axis
            && same_curve(&c,&d,tol,&|p| { let q = sub(p,f.o); let h = dot(q,f.z); [norm(sub(q,scale(f.z,h))),h,0.] }),
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
        let points = match (&ma,&mb) {
            // two spline meridians have no closed-form crossing: traced
            (Meridian::Spline(..),Meridian::Spline(..)) => continue,
            (Meridian::Spline(sp,o,z),m) | (m,Meridian::Spline(sp,o,z)) => spline_cross(sp,*o,*z,m,tol),
            _ => {
                let Some(points) = cross2(ma,mb,tol) else { return Ssi::Same };
                points
            }
        };
        let mut out = Vec::new();
        for [rho,h] in points {
            // a circle of no radius is a point on the axis: a touch, not a curve
            if rho <= tol { continue }
            out.push(Curve::Circle(Frame::about(add(o,scale(z,h)),z),rho));
        }
        return Ssi::Curves(out)
    }
    match (a.clone(),b.clone()) {
        (Surface::Plane(_),Surface::Cylinder(..)) | (Surface::Cylinder(..),Surface::Plane(_)) => {
            let (pl,cy) = if matches!(a,Surface::Plane(_)) { (f,b.clone()) } else { (g,a.clone()) };
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
            let (pl,sp) = if matches!(a,Surface::Plane(_)) { (f,b.clone()) } else { (g,a.clone()) };
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

/// The curves two surfaces meet in, traced through `seeds` (points on both) within the box
/// `[lo, hi]`: from each seed not already on a traced curve, a march along `∇a × ∇b` both ways
/// — each step predicted along the tangent and pulled onto both surfaces, shortened while the
/// tangent turns more than a few degrees — until it leaves the box or comes back to where it
/// began. A seed where the two are tangent (their normals parallel) is refused.
pub fn trace(a: &Surface,b: &Surface,seeds: &[V],lo: V,hi: V,tol: f64) -> Result<Vec<Curve>,String> {
    use super::geom::Traced;
    // the sine of the angle the two meet at, below which a trace is ill-conditioned (a point
    // within `tol` of both surfaces may be `tol / sin θ` off their meeting, and where they touch
    // their meeting has a singular point): refused, not traced
    const SHALLOW: f64 = 0.0175; // one degree
    let tangent = |p: V| -> Option<V> {
        let t = cross(a.gradient(p),b.gradient(p));
        let n = norm(t);
        (n > SHALLOW).then(|| scale(t,1./n))
    };
    let shallow = |p: V| format!("a {} and a {} meet at {:.2}° at [{:.4}, {:.4}, {:.4}], nearly touching: their meeting is \
        ill-conditioned there, not built yet",a.kind(),b.kind(),norm(cross(a.gradient(p),b.gradient(p))).asin().to_degrees(),p[0],p[1],p[2]);
    let probe = Traced {a:a.clone(),b:b.clone(),pts:vec![],closed:false};
    let diag = norm(sub(hi,lo)).max(tol);
    let h0 = (a.feature().min(b.feature())/12.).min(diag/24.).max(diag*1e-4);
    let inside = |p: V,pad: f64| (0..3).all(|k| p[k] >= lo[k]-pad && p[k] <= hi[k]+pad);
    let mut curves: Vec<Curve> = Vec::new();
    for &seed in seeds {
        let p0 = probe.project(seed);
        if curves.iter().any(|c| crate::space::distance(c.point(c.inverse(p0)),p0) <= 8.*tol) { continue }
        let Some(_) = tangent(p0) else { return Err(shallow(p0)) };
        let mut halves: Vec<Vec<V>> = Vec::new();
        let mut closed = false;
        for sense in [1.,-1.] {
            let mut pts = vec![p0];
            let mut p = p0;
            let mut t = scale(tangent(p0).unwrap(),sense);
            let mut h = h0;
            for _ in 0..100_000 {
                let q = probe.project(add(p,scale(t,h)));
                let Some(tq) = tangent(q) else { return Err(shallow(q)) };
                let tq = if dot(tq,t) < 0. { scale(tq,-1.) } else { tq };
                if dot(tq,t) < 0.996 && h > h0*1e-4 { h /= 2.; continue }
                // back where it began: this step's chord passes the start, a closed curve
                if pts.len() > 3 {
                    let d = sub(q,p);
                    let s = dot(sub(p0,p),d)/dot(d,d);
                    if (0. ..=1.).contains(&s) && crate::space::distance(p0,crate::space::lerp(p,q,s)) < 0.25*h {
                        closed = true;
                        pts.push(p0);
                        break
                    }
                }
                pts.push(q);
                if !inside(q,h) { break }
                p = q;
                t = tq;
                if dot(tq,t) > 0.9995 { h = (h*1.5).min(h0); }
            }
            halves.push(pts);
            if closed { break }
        }
        let pts = if closed { halves.swap_remove(0) } else {
            let mut back = halves.pop().unwrap();
            back.reverse();
            back.pop();
            back.extend(halves.pop().unwrap());
            back
        };
        if pts.len() < 2 { continue }
        curves.push(Curve::Traced(std::sync::Arc::new(Traced {a:a.clone(),b:b.clone(),pts,closed})));
    }
    Ok(curves)
}
