//! Where two surfaces meet, in closed form where it has one: two planes in a line; surfaces of
//! revolution about one axis (a plane square to it, a cylinder, a cone, a sphere centred on it, a
//! torus) in circles, found where their meridians cross in one half-plane; a plane and a cylinder
//! in lines or an ellipse; a plane and a sphere in a circle; parallel cylinders in lines. The
//! rest is left to be traced (`Ssi::Traced`) from seeds, and between analytic surfaces a curve no
//! seed reaches is searched for (`unseen`).
#[allow(unused_imports)]
use crate::fmath::Det;
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
    /// `z := h`: a plane square to the axis.
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
            Meridian::Circle {c,r: rad} => (r-c[0]).dhypot(z-c[1])-rad,
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
    let push = |p: [f64;2],out: &mut Vec<[f64;2]>| if !out.iter().any(|q| (q[0]-p[0]).dhypot(q[1]-p[1]) <= tol) { out.push(p) };
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
            Some(Meridian::Line {p:[r,height(f.o)],d:[a.dsin(),a.dcos()*sign]})
        }
        Surface::Torus(_,big,r) => Some(Meridian::Circle {c:[big,height(f.o)],r}),
        Surface::Revolution(_,ref c) => match &**c { Curve::BSpline(b) => Some(Meridian::Spline(b.clone(),o,z)),_ => None },
        Surface::Extrusion(..) | Surface::Blend(..) | Surface::BSpline(..) => None,
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
        let gap = ((foot[0]-c[0]).dhypot(foot[1]-c[1])-r).abs();
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
            let d = (k[0]-c[0]).dhypot(k[1]-c[1]);
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
            let apex = |f: &Frame,r: f64,a: f64| sub(f.o,scale(f.z,r*a.dcos()/a.dsin()));
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
        ill-conditioned there, not built yet",a.kind(),b.kind(),norm(cross(a.gradient(p),b.gradient(p))).dasin().to_degrees(),p[0],p[1],p[2]);
    let probe = Traced {a:a.clone(),b:b.clone(),pts:vec![],closed:false};
    let diag = norm(sub(hi,lo)).max(tol);
    let h0 = (a.feature().min(b.feature())/12.).min(diag/24.).max(diag*1e-4);
    let inside = |p: V,pad: f64| (0..3).all(|k| p[k] >= lo[k]-pad && p[k] <= hi[k]+pad);
    let mut curves: Vec<Curve> = Vec::new();
    // the curves traced so far, as traced (exact: each point a projection onto both surfaces), so a
    // seed on one is known by its distance to the curve and not to a polyline's chords
    let mut traced_so_far: Vec<Curve> = Vec::new();
    for &seed in seeds {
        let p0 = probe.project(seed);
        if traced_so_far.iter().any(|c| crate::space::distance(c.point(c.inverse(p0)),p0) <= 8.*tol) { continue }
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
                // a projection that does not reach both surfaces: a step too long (shortened), or past a
                // sheet's domain, where its foot is held at the edge (still so at the least step): the
                // end of this direction. Past the edge a sheet's signed distance runs on along its
                // tangent extension, so it is the distance to the patch itself that says so — read by
                // the signed one alone, a trace went back and forth at an edge a hundred thousand times
                let off = |s: &Surface| s.off_patch(q).unwrap_or(0.);
                if a.implicit(q).abs().max(b.implicit(q).abs()).max(off(a)).max(off(b)) > 16.*tol {
                    // a step too long for the projection to land: shorter, as a turn too sharp is
                    if h > h0*1e-4 { h /= 2.; continue }
                    if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
                        eprintln!("brep:   a step left the surfaces after {} points: {:.1e} and {:.1e} off, h {h:.1e}",pts.len(),a.implicit(q),b.implicit(q));
                    }
                    break
                }
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
        if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() { eprintln!("brep:   a curve of {} points, closed {closed}, from {:?} to {:?}",pts.len(),pts[0],pts[pts.len()-1]); }
        let traced = Traced {a:a.clone(),b:b.clone(),pts,closed};
        traced_so_far.push(Curve::Traced(std::sync::Arc::new(traced)));
    }
    // each fitted through the seeds on it, exactly: they are where the faces' boundaries cross, the
    // vertices its edges will end at (a fit within `tol` of the traced points may pass farther from
    // a vertex between them, where a swept surface's curve runs on into its tangent)
    let projected: Vec<V> = seeds.iter().map(|&s| probe.project(s)).collect();
    for c in traced_so_far {
        let Curve::Traced(t) = &c else { unreachable!() };
        let through: Vec<(f64,V)> = projected.iter().filter_map(|&p| {
            let x = c.inverse(p);
            if crate::space::distance(c.point(x),p) > 8.*tol { return None }
            // where along the trace, closely: a seed placed a little past its neighbour in the
            // trace's order doubles the fit back on itself there
            let (mut a,mut b) = ((x-1.).max(0.),(x+1.).min((t.pts.len()-1) as f64));
            let d = |x: f64| crate::space::distance(t.at(x),p);
            for _ in 0..40 {
                let (m1,m2) = (a+(b-a)*0.382,a+(b-a)*0.618);
                if d(m1) < d(m2) { b = m2 } else { a = m1 }
            }
            Some(((a+b)/2.,p))
        }).collect();
        curves.push(fitted(t,tol/4.,&through).unwrap_or(c));
    }
    Ok(curves)
}

/// Whether the search for unseen meetings (`unseen`) can answer for a surface, as either side:
/// one with closed-form bounds on its derivatives and a signed distance for its implicit.
pub fn searchable(s: &Surface) -> bool { derivative_bounds(s,[0.,0.]).is_some() }

/// Bounds over the parameter strip `v` of an analytic surface's derivatives: the largest sizes of
/// `S_u`, `S_v`, `S_uu`, `S_uv`, `S_vv`. None for a surface swept from a curve, a loft or a sheet.
fn derivative_bounds(s: &Surface,[v0,v1]: [f64;2]) -> Option<[f64;5]> {
    match *s {
        Surface::Plane(_) => Some([1.,1.,0.,0.,0.]),
        Surface::Cylinder(_,r) => Some([r,1.,r,0.,0.]),
        Surface::Cone(_,r,a) => {
            let sa = a.dsin();
            let q = (r+v0*sa).abs().max((r+v1*sa).abs());
            Some([q,1.,q,sa.abs(),0.])
        }
        Surface::Sphere(_,r) => Some([r;5]),
        Surface::Torus(_,big,r) => Some([big.abs()+r,r,big.abs()+r,r,r]),
        Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) => None,
    }
}

/// A bound on the size of an analytic surface's `implicit`'s Hessian over the ball about `p` of
/// radius `r` — the curvature of its level sets, the inverse of the distance to where they focus —
/// or none where the ball reaches that focus (an axis, a centre, a torus's core circle).
fn curvature_bound(s: &Surface,p: V,r: f64) -> Option<f64> {
    let [x,y,z] = s.frame().local(p);
    let rho = x.dhypot(y);
    let over = |d: f64| (d > 0.).then(|| 1./d);
    match *s {
        Surface::Plane(_) => Some(0.),
        Surface::Sphere(..) => over(rho.dhypot(z)-r),
        // a cone's implicit is cos α times ρ less a linear part: ρ's curvature at most
        Surface::Cylinder(..) | Surface::Cone(..) => over(rho-r),
        // the distance to the core circle in the meridian half-plane, which turns with ρ
        Surface::Torus(_,big,_) => Some(over((rho-big).dhypot(z)-r)?+over(rho-r)?),
        Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) => None,
    }
}

/// Points along a traced curve, or a fitted one, no farther apart than `spacing` (a traced curve's
/// chords: they pass within a sagitta of it far smaller than the spacing).
fn samples_along(c: &Curve,spacing: f64) -> Vec<V> {
    // even in the parameter, twice as many as its sampled speed asks
    let even = |t0: f64,t1: f64,length: f64| -> Vec<V> {
        let n = (2.*length/spacing).ceil().clamp(1.,1e6) as usize;
        (0..=n).map(|j| c.point(t0+(t1-t0)*j as f64/n as f64)).collect()
    };
    match c {
        Curve::Traced(t) => t.pts.windows(2).flat_map(|w| {
            let n = (crate::space::distance(w[0],w[1])/spacing).ceil().clamp(1.,1e6) as usize;
            (0..n).map(move |j| crate::space::lerp(w[0],w[1],j as f64/n as f64))
        }).chain(t.pts.last().copied()).collect(),
        Curve::BSpline(b) => { let [t0,t1] = b.domain(); even(t0,t1,c.speed()*(t1-t0)) }
        _ => match c.period() { Some(p) => even(0.,p,c.speed()*p),None => Vec::new() },
    }
}

/// A point where `b` meets a face of `a` that no curve of `known` passes near: what an edge's
/// crossing cannot seed — a closed curve lying inside both faces. `domain` is a box of `a`'s
/// parameters holding the face, and the search keeps to the faces' shared box `[lo, hi]`.
///
/// The box is halved, the longer side in space first, and each cell answered three ways: clear of
/// `b` (shown, not sampled: `b`'s implicit is a signed distance, so it changes by no more than the
/// distance moved — the cell's reach from its middle — and, where the cell keeps clear of where
/// `b`'s level sets focus, by no more than its first-order change and a bound on its second);
/// beside a known curve, once no wider than `leaf`; or holding a point pulled onto both surfaces
/// from its middle, new and returned. A cell answering none of these is halved again, down to a
/// few tolerances — below that its meeting with `b` is not resolved, and that is refused, never
/// guessed. Two curves nearer than `leaf` are not told apart. `Ok(None)`: every part of the face is
/// clear of `b` or beside a known curve. Both surfaces must be `searchable`.
pub fn unseen(a: &Surface,domain: [[f64;2];2],b: &Surface,known: &[Curve],lo: V,hi: V,tol: f64) -> Result<Option<V>,String> {
    // a cap on the cells asked, far past any face this kernel builds, that a pathology ends at
    const CELLS: usize = 4_000_000;
    let pad = 4.*tol;
    if (0..3).any(|k| !(hi[k]+pad >= lo[k]-pad)) { return Ok(None) }
    let diag = norm(sub(hi,lo));
    let leaf = (a.feature().min(b.feature()).min(diag)/64.).max(diag*1e-4).max(16.*tol);
    let floor = (leaf*1e-6).max(4.*tol);
    // the known curves' points, filed by place, read within a cell's reach of its middle
    let spacing = leaf/2.;
    let mut grid = crate::space::Grid::new(2.*leaf+spacing);
    let mut along: Vec<V> = Vec::new();
    for c in known { for p in samples_along(c,spacing) { grid.insert(p,along.len() as u32); along.push(p); } }
    let near_known = |p: V,within: f64| {
        let mut hit = false;
        grid.around(p,|i| if !hit && crate::space::distance(along[i as usize],p) <= within { hit = true });
        hit
    };
    let probe = super::geom::Traced {a:a.clone(),b:b.clone(),pts:vec![],closed:false};
    let mut cells = vec![domain];
    let mut asked = 0;
    while let Some([[u0,u1],[v0,v1]]) = cells.pop() {
        asked += 1;
        if asked > CELLS {
            return Err(format!("a {} and a {}: the search for where they meet asked {CELLS} cells without settling, not built yet",
                a.kind(),b.kind()))
        }
        let Some([mu,mv,muu,muv,mvv]) = derivative_bounds(a,[v0,v1]) else {
            return Err(format!("a {} cannot be searched for where it meets a {}",a.kind(),b.kind()))
        };
        let (hu,hv) = ((u1-u0)/2.,(v1-v0)/2.);
        let mid = [(u0+u1)/2.,(v0+v1)/2.];
        let (p,su,sv) = a.d1(mid);
        // every point of the cell is within `reach` of its middle
        let reach = mu*hu+mv*hv;
        if (0..3).any(|k| p[k]+reach < lo[k]-pad || p[k]-reach > hi[k]+pad) { continue }
        let g = b.implicit(p);
        if g.abs() > reach { continue }
        if let Some(k) = curvature_bound(b,p,reach) {
            let n = b.gradient(p);
            let slope = dot(n,su).abs()*hu+dot(n,sv).abs()*hv;
            let bend = k*reach*reach+muu*hu*hu+2.*muv*hu*hv+mvv*hv*hv;
            if g.abs()-slope-bend/2. > 0. { continue }
        }
        if reach <= leaf {
            if near_known(p,2.*reach+spacing) { continue }
            let q = probe.project(p);
            if a.implicit(q).abs().max(b.implicit(q).abs()) <= 16.*tol {
                // a meeting beyond the shared box is on neither face; one on a known curve says
                // nothing of this cell, which is halved
                if (0..3).any(|k| q[k] < lo[k]-pad || q[k] > hi[k]+pad) { continue }
                if !near_known(q,spacing) { return Ok(Some(q)) }
            }
            if reach <= floor {
                return Err(format!("a {} and a {} come within {:.1e} of each other at [{:.4}, {:.4}, {:.4}] and no meeting was \
                    found there: whether they touch is not resolved, not built yet",a.kind(),b.kind(),g.abs(),p[0],p[1],p[2]))
            }
        }
        if mu*hu >= mv*hv { cells.push([[mid[0],u1],[v0,v1]]); cells.push([[u0,mid[0]],[v0,v1]]); }
        else { cells.push([[u0,u1],[mid[1],v1]]); cells.push([[u0,u1],[v0,mid[1]]]); }
    }
    if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
        eprintln!("brep:   a {} over {domain:?} is clear of a {} but for {} known curve(s), in {asked} cells (leaf {leaf:.1e})",
            a.kind(),b.kind(),known.len());
    }
    Ok(None)
}

/// An open traced curve as a cubic B-spline through its points (on both surfaces already) by chord
/// length — its own parameter has a kink at every point, which its derivative, a difference of two
/// projections, reads through the surfaces' noise — each gap's middle projected and the curve
/// refined where it passes further than `tol` from it; `None` where no refinement comes within that.
/// Its derivative is then its own exactly: what an integral along it and a face's curve in its
/// parameters need. Each refinement projects one point a gap, never the whole curve again.
fn fitted(t: &super::geom::Traced,tol: f64,through: &[(f64,V)]) -> Option<Curve> {
    if t.closed || t.pts.len() < 2 { return None }
    // Interpolated from every eighth traced point to begin with, a gap between two interpolated
    // points split at its middle where the fit misses the traced curve there by more than `tol`:
    // a traced point in it, or the traced curve's middle of a step, farther from the fit (the fit's
    // own points projected onto both surfaces read a sheet's numerical foot, whose noise is near
    // `tol`, and never settle; for that noise a gap is split to an eighth of a step and no
    // further).  A curve with a knot at every traced
    // point is as many knots as the trace took steps, and a mesh puts several points in every span.
    // Neighbouring gaps are kept within twice each other's steps: an interpolating cubic through
    // points crowded beside sparse ones loops away from the curve between them, and passes near
    // every point it was checked at while doing so.
    const STRIDE: usize = 8;
    let last = t.pts.len()-1;
    // (a point it must pass through takes the place of a regular one within half a stride of it,
    // which beside it would leave a gap the grading spreads)
    let must: Vec<f64> = through.iter().map(|w| w.0).filter(|&x| x > 0. && x < last as f64).collect();
    // every eighth step's length apart, by length along the trace and not by count: a trace takes
    // small steps where it starts from a seed, and points crowded there beside a stride of full
    // steps make the interpolation hook back at the curve's end
    let mut steps: Vec<f64> = t.pts.windows(2).map(|w| crate::space::distance(w[0],w[1])).collect();
    let lengths = steps.clone();
    steps.sort_by(f64::total_cmp);
    let stride = STRIDE as f64*steps[steps.len()/2];
    let mut regular = vec![0.];
    let mut since = 0.;
    for (i,l) in lengths.iter().enumerate().take(last.saturating_sub(1)) {
        since += l;
        if since >= stride { regular.push((i+1) as f64); since = 0.; }
    }
    // the last point, the regular one before it given up where it would leave a gap under half a stride
    if regular.len() > 1 && lengths[*regular.last().unwrap() as usize..].iter().sum::<f64>() < stride/2. { regular.pop(); }
    regular.push(last as f64);
    // how far along the trace a (fractional) index is
    let mut along = vec![0.];
    for l in &lengths { along.push(along.last().unwrap()+l); }
    let length_at = |x: f64| { let i = (x.floor() as usize).min(last.saturating_sub(1)); along[i]+(x-i as f64)*lengths.get(i).copied().unwrap_or(0.) };
    let mut at: Vec<f64> = regular.into_iter()
        .filter(|&x| must.iter().all(|&m| if x == 0. || x == last as f64 { (m-x).abs() >= 0.25 } else {
            (m-x).abs() >= STRIDE as f64/2. && (length_at(m)-length_at(x)).abs() >= stride/2. }))
        .chain(must.iter().copied()).collect();
    at.sort_by(f64::total_cmp);
    at.dedup_by(|x,y| (*x-*y).abs() < 1e-9);
    let between = std::cell::RefCell::new(through.iter().map(|&(x,p)| (x.to_bits(),p)).collect::<std::collections::BTreeMap<u64,V>>());
    let point = |x: f64| if x.fract() == 0. { t.pts[x as usize] } else {
        *between.borrow_mut().entry(x.to_bits()).or_insert_with(|| t.at(x))
    };
    // a traced point where one stands near the middle, else the middle itself: a traced point
    // beside an end (a vertex the fit passes through, between two traced points) would crowd it
    let middle = |a: f64,b: f64| { let m = ((a+b)/2.).floor(); if b-a >= 2. && (m-a).min(b-m) >= (b-a)/4. { m } else { (a+b)/2. } };
    let debug = std::env::var_os("SOLVENT_BREP_DEBUG").is_some();
    for round in 0..24 {
        let pts: Vec<V> = at.iter().map(|&x| point(x)).collect();
        let mut s = vec![0.];
        for w in pts.windows(2) { s.push(s.last().unwrap()+crate::space::distance(w[0],w[1])); }
        if !(*s.last().unwrap() > 0.) || s.windows(2).any(|w| !(w[1] > w[0])) { return None }
        let spline = super::nurbs::interpolate(&pts,&s,3)?;
        let fit = Curve::BSpline(std::sync::Arc::new(spline));
        let mut split = vec![false;at.len()-1];
        for (k,w) in at.windows(2).enumerate() {
            let (a,b) = (w[0],w[1]);
            // the traced points inside, and the traced curve's middle of every step between them
            let inner: Vec<f64> = if b-a >= 1. {
                (a as usize..=b as usize).flat_map(|i| [i as f64,i as f64+0.5]).filter(|&x| x > a+0.125 && x < b-0.125).collect()
            } else { vec![(a+b)/2.] };
            let off = |x: f64| {
                let m = point(x);
                let guess = s[k]+(s[k+1]-s[k])*(x-a)/(b-a);
                crate::space::distance(super::geom::Curve::point(&fit,fit_near(&fit,m,guess)),m) > tol
            };
            // a gap an eighth of a step wide misses by the traced curve's own noise, and is left
            split[k] = b-a > 0.125 && inner.into_iter().any(off);
        }
        let missed = split.iter().filter(|&&x| x).count();
        if debug { eprintln!("brep: fit of a {} × {} trace, round {round}: {} of {} points, {missed} gaps missed",t.a.kind(),t.b.kind(),
            at.len(),t.pts.len()); }
        if missed == 0 { return Some(fit) }
        // graded: a gap more than twice a neighbour's length (as that neighbour will be) is split too
        loop {
            let width = |k: usize,split: &[bool]| { let w = s[k+1]-s[k]; if split[k] { w/2. } else { w } };
            let mut more = false;
            for k in 0..split.len() {
                // (a gap an eighth of a step wide is the traced curve's own noise, and never split)
                if split[k] || at[k+1]-at[k] <= 0.125 { continue }
                let w = width(k,&split);
                if (k > 0 && w > 2.*width(k-1,&split)) || (k+1 < split.len() && w > 2.*width(k+1,&split)) { split[k] = true; more = true; }
            }
            if !more { break }
        }
        let mut next = vec![at[0]];
        for (k,w) in at.windows(2).enumerate() {
            if split[k] { next.push(middle(w[0],w[1])); }
            next.push(w[1]);
        }
        at = next;
    }
    None
}

/// The parameter of the point of `c` nearest `p`, by Newton from `t`.
fn fit_near(c: &Curve,p: V,mut t: f64) -> f64 {
    for _ in 0..12 {
        let (x,d,dd) = c.d2(t);
        let e = sub(x,p);
        let f = dot(e,d);
        let df = dot(d,d)+dot(e,dd);
        if !(df.abs() > 0.) { break }
        let step = f/df;
        t -= step;
        if step.abs() <= 1e-15*(1.+t.abs()) { break }
    }
    t
}
