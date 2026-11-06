//! Where two surfaces meet, in closed form where it has one: two planes in a line; surfaces of
//! revolution about one axis (a plane square to it, a cylinder, a cone, a sphere centred on it, a
//! torus, a spline turned) in circles, found where their meridians cross in one half-plane; a plane and a cylinder
//! in lines or an ellipse; a plane and a sphere in a circle; parallel cylinders in lines; a
//! spline extruded and a plane or a cylinder running its way in lines, and a cone about a line its
//! way in the spline carried onto the cone (`extruded`). The rest is left to be traced
//! (`Ssi::Traced`) from seeds, and between analytic surfaces a curve no seed reaches is searched
//! for (`unseen`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{Carry,Curve,Frame,Rigid,Surface,V};
use super::plane2;
use crate::util::space::{add,cross,distance,dot,norm,scale,segment_distance,sub,unit};

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
pub(super) enum Meridian {
    /// `z := h`: a plane square to the axis.
    Level(f64),
    /// The ray from `p` along the unit `d` (`d[1] > 0` or a vertical line `ρ = r`), both ways.
    Line { p: [f64;2],d: [f64;2] },
    /// A circle about `c` of radius `r`.
    Circle { c: [f64;2],r: f64 },
    /// A curve turned about the axis — a B-spline, or a turned contact's meridian (#780 C3) — over
    /// its domain, and the point and direction that axis runs through.
    Curve(Curve,[f64;2],V,V),
}

impl Meridian {
    /// A signed distance in `(ρ, z)` whose zero set is this meridian (not a spline's).
    fn implicit(&self,[r,z]: [f64;2]) -> f64 {
        match *self {
            Meridian::Level(h) => z-h,
            Meridian::Line {p,d} => (r-p[0])*d[1]-(z-p[1])*d[0],
            Meridian::Circle {c,r: rad} => (r-c[0]).dhypot(z-c[1])-rad,
            Meridian::Curve(..) => unreachable!("a curve meridian has no closed-form distance"),
        }
    }
    /// The gradient of `implicit` in `(ρ, z)`.
    fn gradient(&self,[r,z]: [f64;2]) -> [f64;2] {
        match *self {
            Meridian::Level(_) => [0.,1.],
            Meridian::Line {d,..} => [d[1],-d[0]],
            Meridian::Circle {c,..} => {
                let (x,y) = (r-c[0],z-c[1]);
                let l = x.dhypot(y);
                if l > 0. { [x/l,y/l] } else { [1.,0.] }
            }
            Meridian::Curve(..) => unreachable!("a curve meridian has no closed-form distance"),
        }
    }
}

/// A spline meridian's point at `t` in `(ρ, z)` about the axis through `o` along the unit `z`.
fn rz_at(s: &Curve,o: V,z: V,t: f64) -> [f64;2] {
    let q = sub(s.point(t),o);
    let h = dot(q,z);
    [norm(sub(q,scale(z,h))),h]
}

/// A spline meridian's point at `t` in `(ρ, z)` (`rz_at`, read off its first derivative's
/// evaluation) and how fast `ρ` and `z` run there.
fn rz_jet(s: &Curve,o: V,z: V,t: f64) -> ([f64;2],[f64;2]) {
    let (p,d) = s.d1(t);
    let q = sub(p,o);
    let h = dot(q,z);
    let across = sub(q,scale(z,h));
    let r = norm(across);
    ([r,h],[if r > 0. { dot(across,d)/r } else { 0. },dot(d,z)])
}

/// Where a spline meridian crosses or touches another (not a spline), in `(ρ, z)`: the other's
/// distance along it, its roots and touches found by `spline_roots`.
fn spline_cross(s: &Curve,dom: [f64;2],o: V,z: V,other: &Meridian,tol: f64) -> Vec<[f64;2]> {
    let rz = |t: f64| rz_at(s,o,z,t);
    let at = |t: f64| { let [r,h] = rz(t); [r,h,0.] };
    // the distance's rate along the spline: the gradient in (ρ, z) against how fast ρ and z run
    let rate = |t: f64| { let (q,dq) = rz_jet(s,o,z,t); let g = other.gradient(q); g[0]*dq[0]+g[1]*dq[1] };
    spline_roots(s,dom,&|t| other.implicit(rz(t)),&rate,&at,tol).into_iter().map(rz).collect()
}

/// **Where two spline meridians cross or touch, in `(ρ, z)`** (#735 A2's coaxial surfaces of
/// revolution, both meridians splines: two hyperboloids a swept tool's edges turn into): the first
/// read against the second's signed distance, its point's offset from the second's nearest along
/// the second's normal there (`spline_roots`, as `spline_cross` reads a line or a circle), each
/// root held to lie within `tol` of the second. `None` where the two run together over a stretch
/// (no finite set of circles): left to be traced.
fn spline_spline((a,da): (&Curve,[f64;2]),(b,db): (&Curve,[f64;2]),o: V,z: V,tol: f64) -> Option<Vec<[f64;2]>> {
    let [b0,b1] = db;
    let seeds: Vec<(f64,[f64;2])> = spline_samples(b,db).into_iter().map(|u| (u,rz_at(b,o,z,u))).collect();
    // the second's nearest point to `p` (from its nearest sample, by Newton on (B − p)·B', held to
    // its domain, to a step its rounding allows), and its unit normal there
    let nearest = |p: [f64;2]| -> ([f64;2],[f64;2]) {
        let d2 = |q: [f64;2]| (q[0]-p[0])*(q[0]-p[0])+(q[1]-p[1])*(q[1]-p[1]);
        let mut u = seeds.iter().min_by(|x,y| d2(x.1).total_cmp(&d2(y.1))).map_or(b0,|s| s.0);
        for _ in 0..30 {
            let (q,dq) = rz_jet(b,o,z,u);
            let h = dq[0]*dq[0]+dq[1]*dq[1];
            if !(h > 0.) { break }
            let next = (u-((q[0]-p[0])*dq[0]+(q[1]-p[1])*dq[1])/h).clamp(b0,b1);
            if (next-u).abs() <= 1e-12*(b1-b0) { u = next; break }
            u = next;
        }
        let (q,dq) = rz_jet(b,o,z,u);
        let l = dq[0].dhypot(dq[1]).max(1e-300);
        (q,[dq[1]/l,-dq[0]/l])
    };
    let g = |t: f64| { let p = rz_at(a,o,z,t); let (q,n) = nearest(p); (p[0]-q[0])*n[0]+(p[1]-q[1])*n[1] };
    let dg = |t: f64| { let (p,dp) = rz_jet(a,o,z,t); let (_,n) = nearest(p); n[0]*dp[0]+n[1]*dp[1] };
    let ts = spline_samples(a,da);
    let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
    // running together over a stretch: three samples in a row on the second
    if gs.windows(3).any(|w| w.iter().all(|x| x.abs() <= tol)) { return None }
    let at = |t: f64| { let [r,h] = rz_at(a,o,z,t); [r,h,0.] };
    Some(roots_among(da,&ts,&gs,&g,&dg,&at,tol).into_iter().map(|t| rz_at(a,o,z,t))
        .filter(|&p| { let (q,_) = nearest(p); (p[0]-q[0]).dhypot(p[1]-q[1]) <= tol }).collect())
}

/// The parameters at which `g` along the spline `s`, its rate `dg`, comes to zero within `tol`
/// (`util::roots::sampled_roots`: a crossing, or a touch where `dg` turns): one each where `at`
/// puts them within `tol` of each other.
fn spline_roots(s: &Curve,dom: [f64;2],g: &dyn Fn(f64) -> f64,dg: &dyn Fn(f64) -> f64,at: &dyn Fn(f64) -> V,tol: f64) -> Vec<f64> {
    let ts = spline_samples(s,dom);
    let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
    roots_among(dom,&ts,&gs,g,dg,at,tol)
}

/// `spline_roots` from `g` read already at the spline's samples `ts` (`gs`).
fn roots_among([a,b]: [f64;2],ts: &[f64],gs: &[f64],g: &dyn Fn(f64) -> f64,dg: &dyn Fn(f64) -> f64,
    at: &dyn Fn(f64) -> V,tol: f64) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    for r in crate::util::roots::sampled_roots(g,dg,ts,gs,tol,1e-15*(b-a)) {
        if !out.iter().any(|&u| distance(at(u),at(r.at)) <= tol) { out.push(r.at) }
    }
    out
}

/// Where a spline's crossings are looked for: 32 steps a knot span, both ends included.
fn spline_samples(s: &Curve,[a,b]: [f64;2]) -> Vec<f64> {
    let mut cuts = vec![a];
    cuts.extend(s.breaks([a,b]));
    cuts.push(b);
    let mut ts: Vec<f64> = Vec::new();
    for w in cuts.windows(2) { for j in 0..32 { ts.push(w[0]+(w[1]-w[0])*j as f64/32.); } }
    ts.push(b);
    ts
}

/// **A spline extruded along `f.z` and a surface the same all along that way** (#644: a tooth
/// space's prism cut from a turned blank): a plane along it or a cylinder about a line along it
/// meets the spline's face in the lines along it through where the spline crosses that surface,
/// exactly; a cone about a line along it, in the spline carried along it onto the cone (`lifted`).
/// Traced instead — None — for any other surface, a cone's curve that will not be written so, and
/// wherever the two may touch, which the trace and the Boolean read (a tool's side in the pocket
/// it cut, along the pocket's wall): a meeting at under `SHALLOW`, or the distance along the
/// spline turning between two samples within `tol` of zero or past it (two crossings the samples
/// alone would read as none). (Traced, a straight meeting took a step as short as the
/// spline's sharpest turn anywhere allows, thousands of them, and was fitted through a thousand.)
fn extruded(f: &Frame,c: &Curve,s: &Surface,tol: f64) -> Option<Vec<Curve>> {
    let Curve::BSpline(sp) = c else { return None };
    let g = s.frame();
    let along = norm(cross(f.z,g.z)) <= 1e-12;
    // the sine of the angle the two meet at, at the spline's `t` carried to `p`
    let steep = |t: f64,p: V| norm(cross(unit(cross(sp.d1(t).1,f.z)),unit(s.gradient(p)))) >= SHALLOW;
    match *s {
        Surface::Plane(_) if dot(f.z,g.z).abs() <= 1e-12 => {}
        Surface::Cylinder(..) if along => {}
        Surface::Cone(_,r,a) if along => return lifted(sp,g,r,a,tol,&steep).map(|q| vec![q]),
        _ => return None,
    }
    // (a plane's and a cylinder's signed distance is the same all along the extrusion), and where
    // it turns between two samples
    let gap = |t: f64| s.implicit(sp.point(t));
    let rate = |t: f64| { let (p,d) = sp.d1(t); dot(s.gradient(p),d) };
    let [t0,t1] = sp.domain();
    for w in spline_samples(c,sp.domain()).windows(2) {
        let (r0,r1) = (rate(w[0]),rate(w[1]));
        if (r0 < 0.) == (r1 < 0.) { continue }
        let x = crate::util::roots::bracketed_root(|t| Some(rate(t)),w[0],r0,w[1],r1,1e-15*(t1-t0));
        let g = gap(x);
        if g.abs() <= tol || g*gap(w[0]) <= 0. && g*gap(w[1]) <= 0. { return None }
    }
    let roots = spline_roots(c,sp.domain(),&gap,&rate,&|t| sp.point(t),tol);
    roots.iter().all(|&t| steep(t,sp.point(t)))
        .then(|| roots.into_iter().map(|t| Curve::Line {p:sp.point(t),d:f.z}).collect())
}

/// A cubic spline carried along the axis of the cone `g`, `r`, `α` onto it: each of its points to
/// the cone's height at its radius, read in `ρ ≥ 0` as a meridian's crossings are. Written as a
/// quintic over the spline's own parameter on its own knots, each twice more — which hold the
/// spline itself exactly, so only the height is fitted, and as the sixth power of a span (a cubic
/// took five times the poles) — a span halved wherever the carried curve strays from it farther
/// than a quarter of `tol` at a quarter point (as a traced curve is fitted, `fitted`). None for a
/// spline not polynomial cubic, a cone nearly a cylinder, a spline coming near the axis, the two
/// meeting anywhere at an angle not `steep`, or past 4096 spans.
fn lifted(sp: &super::nurbs::BSpline,g: &Frame,r: f64,a: f64,tol: f64,steep: &dyn Fn(f64,V) -> bool)
    -> Option<Curve> {
    let (sa,ca) = a.dsin_cos();
    if sp.degree != 3 || sp.weights.is_some() || !(sa.abs() > 1e-9) { return None }
    let radius = |t: f64| { let w = sub(sp.point(t),g.o); norm(sub(w,scale(g.z,dot(w,g.z)))) };
    let q = |t: f64| {
        let (x,rho) = (sp.point(t),radius(t));
        add(x,scale(g.z,(rho-r)*ca/sa-dot(sub(x,g.o),g.z)))
    };
    // the spline's knots each twice more, so a quintic on them holds the cubic as it joins
    let mut knots: Vec<f64> = super::nurbs::distinct(&sp.knots).into_iter()
        .flat_map(|(k,n)| std::iter::repeat_n(k,n+2)).collect();
    for _ in 0..24 {
        let fit = super::nurbs::interpolate_on(&q,5,knots.clone())?;
        let breaks = super::nurbs::distinct(&knots);
        let spans: Vec<[f64;2]> = breaks.windows(2).map(|w| [w[0].0,w[1].0]).collect();
        let mut halves = Vec::new();
        for &[t0,t1] in &spans {
            let quarters = [0.25,0.5,0.75].map(|s| t0+(t1-t0)*s);
            // near the axis the carried curve turns as sharply as the spline is near it
            if quarters.iter().any(|&t| radius(t) <= 1e-3*sp.hull_length() || !steep(t,q(t))) {
                return None
            }
            if quarters.iter().any(|&t| distance(fit.point(t),q(t)) > tol/4.) { halves.push((t0+t1)/2.); }
        }
        if halves.is_empty() { return Some(Curve::BSpline(std::sync::Arc::new(fit))) }
        if spans.len()+halves.len() > 4096 { return None }
        knots.extend(halves);
        knots.sort_by(f64::total_cmp);
    }
    None
}

/// A surface's meridian about the axis through `o` along the unit `z`, where it is a surface of
/// revolution about that axis (to `tol`).
pub(super) fn meridian(s: &Surface,o: V,z: V,tol: f64) -> Option<Meridian> {
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
        Surface::Revolution(_,ref c) => match &**c {
            Curve::BSpline(b) => Some(Meridian::Curve((**c).clone(),b.domain(),o,z)),
            Curve::Turned(t) if !t.posed => Some(Meridian::Curve((**c).clone(),t.span,o,z)),
            _ => None,
        },
        Surface::Affine(..) | Surface::Extrusion(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => None,
    }
}

/// Where two meridians cross in `ρ ≥ −tol`, as `(ρ, z)`; `None` where they coincide.
fn cross2(a: Meridian,b: Meridian,tol: f64) -> Option<Vec<[f64;2]>> {
    use Meridian::*;
    let carrier = |m: Meridian| match m {
        Level(h) => plane2::Carrier::Line {p:[0.,h],d:[1.,0.]},
        Line {p,d} => plane2::Carrier::Line {p,d},
        Circle {c,r} => plane2::Carrier::Circle {c,r},
        Curve(..) => unreachable!("a curve meridian is crossed by `spline_cross`"),
    };
    // a level is the line the crossing is read along
    let (a,b) = if matches!(b,Level(_)) && !matches!(a,Level(_)) { (b,a) } else { (a,b) };
    let pts = plane2::meet(carrier(a),carrier(b),plane2::Touch::Gap(tol));
    pts.map(|pts| pts.into_iter().filter(|q| q[0] >= -tol).collect())
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

/// Whether two surfaces are the same surface, to `tol`. An affine image is read in its preimage,
/// the other surface carried back there and held to the tolerance shrunk by the map's greatest
/// stretch; two images neither of which is plain there are the same quadric or not known to be
/// one (`affine::Quadric::same`).
pub fn same(a: &Surface,b: &Surface,tol: f64) -> bool {
    if let Some(met) = pulled_back(a,b,tol,false) { return matches!(met,Ssi::Same) }
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
            let apex = |f: &Frame,r: f64,a: f64| cone_apex(f,r,a);
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
        (Surface::Blend(_,x),Surface::Blend(_,y)) => one_screwed_circle(&x,&y,tol),
        (Surface::Plane(p),Surface::BSpline(_,n)) | (Surface::BSpline(_,n),Surface::Plane(p)) => net_in_plane(&n,&p,tol),
        _ => false,
    }
}

/// **A sheet lying in a plane is that plane** (#780 C4): a B-spline surface, rational or not, lies
/// in the hull of its poles (its weights positive), so where every pole stands within `tol` of the
/// plane every point of it does — a fitted sheet a slide sweeps a flat face's strip into, which a
/// closed sweep's candidate holding that face's plane meets as a coincidence (A2), not as a touch
/// to trace. A sheet lying in the plane has its poles in it too (the distance from the plane is a
/// spline whose coefficients are the poles' distances), so nothing that lies in it is missed but
/// by its fit's rounding.
fn net_in_plane(n: &super::nurbs::Net,p: &Frame,tol: f64) -> bool {
    n.poles.iter().flatten().all(|&q| p.local(q)[2].abs() <= tol)
}

/// The apex of the cone of radius `r` at its frame's origin and half-angle `half` (not finite for
/// a cylinder's half-angle of nothing).
fn cone_apex(f: &Frame,r: f64,half: f64) -> V { let (sa,ca) = half.dsin_cos(); sub(f.o,scale(f.z,r*ca/sa)) }

/// **Two cones with one apex meet in lines through it** (#797), exactly: the generators of the
/// first, `d(u) = sin α (cos u x + sin u y) + cos α z` from the apex, that the second holds,
/// `(d·z′)² = cos² β` — `A + R cos(u − φ) = ±cos β`, two generators a sign or none. Traced, the
/// meeting of two spherical sectors sharing their apex crowded its steps toward the apex, where
/// both surfaces' curvature grows without bound, and its fit took 36 000 knots a curve. None where
/// the apices part by more than `tol`, the axes are one (the meridians' reading), or the cones
/// touch along a generator (a double root, left to the trace's reading of a touch).
fn cones_at_one_apex((c,r,half): (&Frame,f64,f64),(k,s,kalf): (&Frame,f64,f64),tol: f64) -> Option<Vec<Curve>> {
    let apex = |f: &Frame,r: f64,a: f64| (a.dsin().abs() > 1e-12).then(|| cone_apex(f,r,a));
    let (p,q) = (apex(c,r,half)?,apex(k,s,kalf)?);
    if distance(p,q) > tol || norm(cross(c.z,k.z)) <= 1e-12 { return None }
    let (sa,ca) = half.dsin_cos();
    let cb = kalf.dcos();
    if !(cb.abs() > 1e-12) { return None }
    let (a0,ax,ay) = (ca*dot(c.z,k.z),sa*dot(c.x,k.z),sa*dot(c.y,k.z));
    let big_r = ax.dhypot(ay);
    if !(big_r > 1e-12) { return None }
    let phi = ay.datan2(ax);
    let o = scale(add(p,q),0.5);
    let mut out = Vec::new();
    for target in [cb,-cb] {
        let x = (target-a0)/big_r;
        if (x.abs()-1.).abs() <= 1e-9 { return None }
        if x.abs() > 1. { continue }
        let w = x.dacos();
        for u in [phi-w,phi+w] {
            let (su,cu) = u.dsin_cos();
            out.push(Curve::Line {p:o,d:c.dir([sa*cu,sa*su,ca])});
        }
    }
    Some(out)
}

/// Where two surfaces meet (`tol` a length).
pub fn intersect(a: &Surface,b: &Surface,tol: f64) -> Ssi {
    if let Some(met) = pulled_back(a,b,tol,true) { return met }
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
            // two spline meridians: where one crosses the other, unless they run together
            // (where they run together, read the other way round they would too: traced)
            (Meridian::Curve(sa,da,o,z),Meridian::Curve(sb,db,..)) => match spline_spline((sa,*da),(sb,*db),*o,*z,tol) {
                Some(points) => points,
                None => break,
            },
            (Meridian::Curve(sp,d,o,z),m) | (m,Meridian::Curve(sp,d,o,z)) => spline_cross(sp,*d,*o,*z,m,tol),
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
        (Surface::Cone(c,r,half),Surface::Cone(k,s,kalf)) if let Some(lines) = cones_at_one_apex((&c,r,half),(&k,s,kalf),tol) =>
            Ssi::Curves(lines),
        (s @ (Surface::Cylinder(..) | Surface::Cone(..)),Surface::Revolution(_,c)) | (Surface::Revolution(_,c),s @ (Surface::Cylinder(..) | Surface::Cone(..)))
            if let Some(curves) = own_contact(&s,&c,tol) => Ssi::Curves(curves),
        (Surface::Plane(pl),Surface::Revolution(_,c)) | (Surface::Revolution(_,c),Surface::Plane(pl))
            if let Some(curves) = plane_holding_rim(&pl,&c,tol).or_else(|| plane_contact_section(&pl,&c)) => Ssi::Curves(curves),
        (Surface::Cylinder(cy,r),Surface::Revolution(_,c)) | (Surface::Revolution(_,c),Surface::Cylinder(cy,r))
            if let Some(curves) = side_holding_rim(&cy,r,&c,tol) => Ssi::Curves(curves),
        (Surface::Plane(_),Surface::Cylinder(..)) | (Surface::Cylinder(..),Surface::Plane(_)) => {
            let (pl,cy) = if matches!(a,Surface::Plane(_)) { (f,b.clone()) } else { (g,a.clone()) };
            let Surface::Cylinder(c,r) = cy else { unreachable!() };
            Ssi::Curves(plane_cylinder(pl.z,dot(pl.z,pl.o),&c,r,tol))
        }
        (Surface::Sphere(sp,r),Surface::Torus(t,big,small)) | (Surface::Torus(t,big,small),Surface::Sphere(sp,r))
            if let Some(circle) = ball_in_tube(&sp,r,&t,big,small,tol) => Ssi::Curves(vec![circle]),
        (Surface::Cylinder(cy,r),Surface::Torus(t,big,small)) | (Surface::Torus(t,big,small),Surface::Cylinder(cy,r))
            if let Some(circle) = tube_along_torus(&cy,r,&t,big,small,tol) => Ssi::Curves(vec![circle]),
        (Surface::Torus(t,big,small),Surface::Torus(u,bog,smell)) if let Some(curves) = tori_touching(a,b,(&t,big,small),(&u,bog,smell),tol) =>
            Ssi::Curves(curves),
        (Surface::Plane(_),Surface::Sphere(..)) | (Surface::Sphere(..),Surface::Plane(_)) => {
            let (pl,sp) = if matches!(a,Surface::Plane(_)) { (f,b.clone()) } else { (g,a.clone()) };
            let Surface::Sphere(c,r) = sp else { unreachable!() };
            let dist = dot(pl.z,sub(c.o,pl.o));
            // touching at a point, by the gap (the circle's radius is its square root)
            let plane2::Chord::Cross(rho) = plane2::chord(dist,r,plane2::Touch::Gap(tol)) else {
                return Ssi::Curves(vec![])
            };
            Ssi::Curves(vec![Curve::Circle(Frame::new(sub(c.o,scale(pl.z,dist)),pl.z,pl.x),rho)])
        }
        (Surface::Plane(_),Surface::Cone(..)) | (Surface::Cone(..),Surface::Plane(_)) => {
            // a plane through the axis: the two lines through the apex, its meridian either side
            let (pl,co) = if matches!(a,Surface::Plane(_)) { (f,b.clone()) } else { (g,a.clone()) };
            let Surface::Cone(c,r,half) = co else { unreachable!() };
            let (sa,ca) = half.dsin_cos();
            // the generator through the cone's point at the radial direction `w`
            let line = |w: V| unit(add(scale(w,sa),scale(c.z,ca)));
            // a plane through the apex leaning to the axis by the cone's own half-angle touches it
            // along one generator, its meeting that line (twice over): the tangent line, where a
            // swept face at an end pose meets the cone its contact line turns into (#721 X6), or
            // where a cone's ruling slid sweeps its tangent plane. Judged by the gap, as a plane
            // touching a cylinder is; a cone narrowing along its axis (its half-angle below
            // nothing) alike
            if sa.abs() > 1e-12 && ca > 1e-12 {
                let apex = cone_apex(&c,r,half);
                let lean = dot(pl.z,c.z).abs()-sa.abs();
                let radial = sub(pl.z,scale(c.z,dot(pl.z,c.z)));
                if dot(pl.z,sub(apex,pl.o)).abs() <= tol && lean.abs() <= 1e-9 && norm(radial) > 1e-12 {
                    let w = unit(radial);
                    // of the two generators in the plane's normal's meridian, the one the plane holds
                    let w = if dot(pl.z,line(w)).abs() <= dot(pl.z,line(scale(w,-1.))).abs() { w } else { scale(w,-1.) };
                    return Ssi::Curves(vec![Curve::Line {p:add(c.o,scale(w,r)),d:line(w)}])
                }
            }
            if let Some(e) = plane_cone_ellipse(&pl,&c,r,half,tol) { return Ssi::Curves(e) }
            if dot(pl.z,c.z).abs() > 1e-12 || dot(pl.z,sub(c.o,pl.o)).abs() > tol { return Ssi::Traced }
            let w = unit(cross(pl.z,c.z));
            Ssi::Curves([w,scale(w,-1.)].into_iter()
                .map(|w| Curve::Line {p:add(c.o,scale(w,r)),d:line(w)}).collect())
        }
        (Surface::Plane(_),Surface::Extrusion(..)) | (Surface::Extrusion(..),Surface::Plane(_))
            if norm(cross(f.z,g.z)) <= 1e-12 => {
            // a plane square to the sweep: the swept curve itself, moved along the sweep onto it
            let (pl,ex) = if matches!(a,Surface::Plane(_)) { (f,b) } else { (g,a) };
            let Surface::Extrusion(e,c) = ex else { unreachable!() };
            let lift = |p: V| add(p,scale(e.z,dot(pl.z,sub(pl.o,p))/dot(pl.z,e.z)));
            match &**c {
                // carried along the sweep, a translation: its weights and knots kept, so a rational
                // curve (an ellipse's rim) stays the curve the plane cuts
                Curve::BSpline(sp) => Ssi::Curves(vec![Curve::BSpline(std::sync::Arc::new(super::nurbs::BSpline {
                    degree:sp.degree,knots:sp.knots.clone(),weights:sp.weights.clone(),poles:sp.poles.iter().map(|&p| lift(p)).collect()}))]),
                Curve::Line {p,d} => Ssi::Curves(vec![Curve::Line {p:lift(*p),d:*d}]),
                Curve::Circle(fr,r) => Ssi::Curves(vec![Curve::Circle(Frame::new(lift(fr.o),fr.z,fr.x),*r)]),
                _ => Ssi::Traced,
            }
        }
        (Surface::Extrusion(e,c),ref s) | (ref s,Surface::Extrusion(e,c))
            if let Some(curves) = extruded(&e,&c,s,tol) => Ssi::Curves(curves),
        (Surface::Plane(pl),Surface::BSpline(_,net)) | (Surface::BSpline(_,net),Surface::Plane(pl)) => {
            // a sheet one linear span across, its second row its first moved by one step `d` (a
            // curve swept along a line, as a generating sweep's sheet is), and a plane the step
            // crosses: the first row's curve carried along the step onto the plane, exactly (the
            // carrying is affine, so its poles carried are the curve's) — where every pole lands
            // within the sheet's span, nothing where none does, and traced where they part
            if net.dv != 1 || net.poles[0].len() != 2 || net.weights.is_some() { return Ssi::Traced }
            let d = sub(net.poles[0][1],net.poles[0][0]);
            let along = dot(pl.z,d);
            if net.poles.iter().any(|r| norm(sub(sub(r[1],r[0]),d)) > tol) || !(along.abs() > 1e-9*norm(d)) { return Ssi::Traced }
            let steps: Vec<f64> = net.poles.iter().map(|r| dot(pl.z,sub(pl.o,r[0]))/along).collect();
            let slack = tol/norm(d);
            if steps.iter().all(|&s| s < -slack) || steps.iter().all(|&s| s > 1.+slack) { return Ssi::Curves(vec![]) }
            if steps.iter().any(|&s| !(-slack..=1.+slack).contains(&s)) { return Ssi::Traced }
            match super::nurbs::BSpline::new(net.du,net.uknots.clone(),net.poles.iter().zip(&steps).map(|(r,&s)| add(r[0],scale(d,s))).collect()) {
                Ok(c) => Ssi::Curves(vec![Curve::BSpline(std::sync::Arc::new(c))]),
                Err(_) => Ssi::Traced,
            }
        }
        (Surface::Cylinder(_,r),Surface::Cylinder(_,s)) if norm(cross(f.z,g.z)) <= 1e-12 => {
            // parallel axes: the sections' circles meet in the lines' feet
            let w = sub(g.o,f.o);
            let w = sub(w,scale(f.z,dot(w,f.z)));
            // touching (a fillet's face on the face it rolls against): one line, judged by how far
            // apart the surfaces stand (`Touch::Gap`)
            let (a,h) = match plane2::circles(norm(w),r,s,plane2::Touch::Gap(tol)) {
                plane2::Circles::Same | plane2::Circles::Miss => return Ssi::Curves(vec![]),
                plane2::Circles::Touch(a) => (a,None),
                plane2::Circles::Cross(a,h) => (a,Some(h)),
            };
            let u = unit(w);
            let v = cross(f.z,u);
            let m = add(f.o,scale(u,a));
            let Some(h) = h else { return Ssi::Curves(vec![Curve::Line {p:m,d:f.z}]) };
            Ssi::Curves(vec![Curve::Line {p:add(m,scale(v,h)),d:f.z},Curve::Line {p:sub(m,scale(v,h)),d:f.z}])
        }
        (Surface::Cylinder(..) | Surface::Cone(..),Surface::Cylinder(..) | Surface::Cone(..)) if norm(cross(f.z,g.z)) <= 1e-12 => {
            // parallel axes, a cone among them: touching along a line where their sections' circles
            // touch at every height — their radii running at one rate, or opposite rates, beside a
            // fixed distance between their axes (a frustum at an end pose, and the cone its generator
            // turns into about an axis along its own, #721 X6); traced otherwise
            match parallel_touch(a,b,tol) { Some(line) => Ssi::Curves(vec![line]),None => Ssi::Traced }
        }
        (Surface::Cylinder(_,r),Surface::Cylinder(_,s)) if (r-s).abs() <= tol => {
            // one radius, axes crossing at O: a point as far from both axes has
            // ((p−O)·a₁)² = ((p−O)·a₂)², so the meeting is the first cylinder cut by the two planes
            // through O square to a₁ − a₂ and a₁ + a₂: two ellipses, crossing where both touch
            let axis = |c: &Frame| Curve::Line {p:c.o,d:c.z};
            let all = [f64::NEG_INFINITY,f64::INFINITY];
            let Some(&[(_,o)]) = super::query::curve_curve(&axis(&f),all,&axis(&g),all,tol).as_deref() else { return Ssi::Traced };
            let mut out = Vec::new();
            for m in [sub(f.z,g.z),add(f.z,g.z)] {
                let m = unit(m);
                out.extend(plane_cylinder(m,dot(m,o),&f,r,tol));
            }
            Ssi::Curves(out)
        }
        (Surface::Plane(pl),Surface::Revolution(rf,c)) | (Surface::Revolution(rf,c),Surface::Plane(pl)) =>
            match plane_quadric(&pl,&rf,&c,tol) { Some(curves) => Ssi::Curves(curves),None => Ssi::Traced },
        (Surface::Blend(..),_) | (_,Surface::Blend(..)) => match screwed_meeting(a,b,tol) {
            Some(curves) => Ssi::Curves(curves),
            None => Ssi::Traced,
        },
        _ => Ssi::Traced,
    }
}

/// **A leaning round face at a pose of a turn, against the surface its own contact turns into**
/// (#780 C3): a cylinder or a cone that is the contact's face (`turned::Contact`) turned by some
/// `θ` about the turn's axis — the same axis line turned, radius and half-angle — touches the
/// surface of revolution of its contact's meridian along that contact turned by `θ`, exactly
/// (`Turned::posed`): the curve where, at that pose, the face's normal is square to the turn's
/// velocity, which is where its slice at each height reaches as far from the axis as the meridian
/// does (or as near). Held only where the face stands on one side of the surface wherever both
/// reach — read over a grid of the face's surface across the meridian's heights, each point's
/// radius against the meridian's at its height — so the touch is all they share; else none, and the
/// two are traced. (Traced, the touch along a curve came apart in two where nothing parted it, and
/// a face holding it had a hole in no piece.)
fn own_contact(s: &Surface,c: &Curve,tol: f64) -> Option<Vec<Curve>> {
    use crate::turned::{Jet,Turned};
    let Curve::Turned(t) = c else { return None };
    if t.posed { return None }
    let (g,Some(k)) = (t.frame,t.contact()) else { return None };
    let (f,r,tan) = match *s {
        Surface::Cylinder(f,r) => (f,r.abs(),0.),
        Surface::Cone(f,r,half) => { let (sa,ca) = half.dsin_cos(); (f,r,sa/ca) }
        _ => return None,
    };
    // the face's axis in the turn's frame, its radius at its origin, and its rate along the axis
    let (o,mut a,mut tan) = (g.local(f.o),g.dir_local(unit(f.z)),tan);
    if (a[2] < 0.) != (k.a[2] < 0.) { a = a.map(|x| -x); tan = -tan; }
    let tan_k = k.sa/k.ca;
    if (a[2]-k.a[2]).abs() > 1e-9 || (tan-tan_k).abs() > 1e-9 { return None }
    // the pose: the turn about the axis carrying the contact's axis onto the face's
    let pose = Rigid::turn([0.;3],[0.,0.,1.],a[1].datan2(a[0])-k.a[1].datan2(k.a[0]));
    let (o_k,a_k) = (pose.point(k.o),pose.vector(k.a));
    if norm(sub(a,a_k)) > 1e-9 || norm(cross(sub(o,o_k),a)) > tol { return None }
    // the radius at the contact's origin, read along the face
    let along = dot(sub(o_k,o),a);
    if (r+along*tan-k.r).abs() > tol { return None }
    // the meridian's heights, and its radius at a height (it climbs: its root along it)
    let [v0,v1] = t.span;
    let z_of = |v: f64| k.meridian(Jet::at(v))[1].v;
    let (za,zb) = (z_of(v0),z_of(v1));
    let (zlo,zhi) = (za.min(zb),za.max(zb));
    let rho_at = |z: f64| -> f64 {
        let v = crate::util::roots::bracketed_root(|v| Some(z_of(v)-z),v0,za-z,v1,zb-z,1e-13*(v1-v0).abs());
        k.meridian(Jet::at(v))[0].v
    };
    // the face's surface over the heights the meridian spans: its rings along the axis by their
    // centres' heights, a radius further each way
    let reach = k.r.abs()+(v1-v0).abs()*k.sa.abs()+1.;
    let (h0,h1) = ((zlo-reach-o[2])/a[2],(zhi+reach-o[2])/a[2]);
    let (h0,h1) = (h0.min(h1),h0.max(h1));
    let (x,y) = crate::util::space::across(a);
    let size = norm(o)+reach;
    let mut sides = (false,false);
    for i in 0..=48 {
        let h = h0+(h1-h0)*i as f64/48.;
        let radius = r+h*tan;
        if radius < 0. { continue }
        for j in 0..48 {
            let (su,cu) = (j as f64*std::f64::consts::TAU/48.).dsin_cos();
            let p = add(add(o,scale(a,h)),add(scale(x,radius*cu),scale(y,radius*su)));
            if p[2] <= zlo || p[2] >= zhi { continue }
            let gap = p[0].dhypot(p[1])-rho_at(p[2]);
            if gap > 1e-7*size { sides.0 = true } else if gap < -1e-7*size { sides.1 = true }
        }
    }
    if sides.0 && sides.1 { return None }
    // posed: the contact itself, read in the turn's frame turned by the pose
    let frame = Frame {o:g.o,x:g.dir(pose.vector([1.,0.,0.])),y:g.dir(pose.vector([0.,1.,0.])),z:g.z};
    Some(vec![Curve::Turned(std::sync::Arc::new(Turned {frame,posed:true,..**t}))])
}

/// **A plane against the surface of revolution of a round face's contact** (#780 L2), exactly:
/// its cut, read round the angle `τ` where `cos(ψ − ψₙ) = C(v) = −cos τ` (`turned::Section`). None
/// where `C` is not proved to run one way over the meridian's span, or the plane is all but level
/// (read as meridians are): traced.
fn plane_contact_section(pl: &Frame,c: &Curve) -> Option<Vec<Curve>> {
    let Curve::Turned(t) = c else { return None };
    let (Some(of),false) = (t.contact(),t.posed) else { return None };
    let g = t.frame;
    let n = g.dir_local(unit(pl.z));
    let (k,spans) = crate::turned::Section::new(of,n,dot(n,g.local(pl.o)),t.span)?;
    Some(spans.into_iter().map(|span| Curve::Turned(std::sync::Arc::new(crate::turned::Turned {frame:g,
        source:crate::turned::Source::Section(k),span,posed:true}))).collect())
}

/// **A cylinder holding a sharp circle's ring at some pose of the turn** (#780 L2: a flat-ended
/// tool's side at an end pose, its rim on the ring that rim turns into), against that ring,
/// exactly: the circle itself, and the loop where the side crosses the ring off it
/// (`turned::Crossing`), which crosses the circle where the side's contact reaches the rim. None
/// where the cylinder holds no turned copy of the circle (its axis the circle's normal turned about
/// the turn's axis, through its centre, of its radius), or the loop is not one `Crossing` reads.
fn side_holding_rim(cy: &Frame,radius: f64,c: &Curve,tol: f64) -> Option<Vec<Curve>> {
    let Curve::Turned(t) = c else { return None };
    let crate::turned::Source::Rim(rim) = t.source else { return None };
    if t.posed { return None }
    let g = t.frame;
    let looped = crate::turned::Crossing::held(&rim,g.dir_local(unit(cy.z)),g.local(cy.o),radius,tol)?;
    Some(vec![Curve::Circle(Frame::about(g.at(looped.c),g.dir(looped.a)),rim.r),
        Curve::Turned(std::sync::Arc::new(crate::turned::Turned {frame:g,source:crate::turned::Source::Crossing(looped),
            span:[0.,std::f64::consts::TAU],posed:true}))])
}

/// **A plane holding a sharp circle's ring at some pose of the turn** (#780 L2: a flat end at an end
/// pose, its rim on the ring that rim turns into), against that ring, exactly: two circles of the
/// circle's radius. Measured from the turn's origin, at `c + q` of the plane (`q` in the posed
/// circle's own frame `e₁, e₂` about its centre `c`), `(|x|², z) = m + L q + (D, 0)`, `L = [[2 c·e₁,
/// 2 c·e₂], [e₁_z, e₂_z]]` and `D = |q|² − r²` the circle's power, while the ring is `m + L q` over
/// `|q| = r`: so the plane's point is on the ring where `|q + D k|² = r²`, `k = L⁻¹ (1, 0)` —
/// `D (1 + 2 q·k + D |k|²) = 0`, the circle itself (`D = 0`) and `|q + k/|k|²|² = r²`, a second
/// circle of the same radius. None where the plane holds no turned copy of the circle (its normal
/// the circle's turned about the axis, its centre on it), or `L` cannot be inverted.
fn plane_holding_rim(pl: &Frame,c: &Curve,tol: f64) -> Option<Vec<Curve>> {
    let Curve::Turned(t) = c else { return None };
    let crate::turned::Source::Rim(rim) = t.source else { return None };
    if t.posed { return None }
    let g = t.frame;
    let (o,e1,e2) = rim.posed(g.dir_local(unit(pl.z))).into_iter().find(|&(o,..)| dot(sub(g.at(o),pl.o),unit(pl.z)).abs() <= tol)?;
    // k = L⁻¹ (1, 0), the first column of the inverse
    let inv = crate::turned::rim_inverse(o,e1,e2,1e-12*(1.+norm(o)))?;
    let k = [inv[0][0],inv[1][0]];
    let kk = k[0]*k[0]+k[1]*k[1];
    let other = sub(o,add(scale(e1,k[0]/kk),scale(e2,k[1]/kk)));
    let n = g.dir(cross(e1,e2));
    Some([o,other].iter().map(|&q| Curve::Circle(Frame::about(g.at(q),n),rim.r)).collect())
}

/// **A plane cutting every generator of a cone, in an ellipse** (#780 C3: a leaning frustum cut by
/// a level plane, at an end pose of a turn), exactly. With the apex `A`, the axis `k` and the
/// half-angle `α`, a point `A + w` is on the cone where `(w·k)² = cos²α |w|²`; on the plane `A + d n
/// + u e + v (n × e)`, `e` the axis's shadow on it (`k = (n·k) n + κ e`), that is `(c² − κ²) (u −
/// u₀)² + c² v² = d² c² s² / (c² − κ²)`, `u₀ = d (n·k) κ / (c² − κ²)`, `c, s` the half-angle's cosine
/// and sine — an ellipse about `A + d n + u₀ e` of radii `|d| c s / (c² − κ²)` along `e` and
/// `|d| s / √(c² − κ²)` across it, wherever the plane leans to the axis less than the generators do
/// (`κ < c`) — and nothing where that ellipse is on the nappe the surface is not (the cone's points
/// stand from the apex along its axis the way its half-angle opens). None for a plane through the
/// apex, along the axis's square (a circle, read as a meridian's), or cutting a generator nowhere
/// or once (a hyperbola, a parabola).
fn plane_cone_ellipse(pl: &Frame,c: &Frame,r: f64,half: f64,tol: f64) -> Option<Vec<Curve>> {
    let (sa,ca) = half.dsin_cos();
    if !(sa.abs() > 1e-9) { return None }
    let k = unit(c.z);
    let apex = sub(c.o,scale(k,r*ca/sa));
    let n = unit(pl.z);
    let nk = dot(n,k);
    let shadow = sub(k,scale(n,nk));
    let kappa = norm(shadow);
    let cc = ca*ca;
    let a2 = cc-kappa*kappa;
    // (square to the axis: a circle, read elsewhere; leaning as far as a generator or past it: an
    // open conic)
    if kappa <= 1e-9 || !(a2 > 1e-9) { return None }
    let d = dot(sub(pl.o,apex),n);
    if d.abs() <= tol { return None }
    let e = scale(shadow,1./kappa);
    let u0 = d*nk*kappa/a2;
    let centre = add(apex,add(scale(n,d),scale(e,u0)));
    if dot(sub(centre,apex),k)*sa*ca < 0. { return Some(vec![]) }
    let (big,small) = (d.abs()*ca.abs()*sa.abs()/a2,d.abs()*sa.abs()/a2.sqrt());
    Some(vec![Curve::Ellipse(Frame::new(centre,n,e),big,small)])
}

/// **A ball within a torus's tube touching it round a meridian**: centred on the tube's middle
/// circle and of the tube's radius, the ball is one of those the torus is the envelope of, within
/// its solid and touching it only round the tube's circle through its centre (a ball at an end pose
/// of a turn, and the torus its great circle turns into, #721 X6). None for any other ball.
fn ball_in_tube(sp: &Frame,r: f64,t: &Frame,big: f64,small: f64,tol: f64) -> Option<Curve> {
    let z = unit(t.z);
    let rel = sub(sp.o,t.o);
    let w = sub(rel,scale(z,dot(rel,z)));
    let rho = norm(w);
    if dot(rel,z).abs() > tol || (rho-big.abs()).abs() > tol || (r.abs()-small.abs()).abs() > tol || rho <= tol { return None }
    let w = scale(w,1./rho);
    Some(Curve::Circle(Frame::new(sp.o,cross(z,w),w),r.abs()))
}

/// **A cylinder touching a torus round a meridian**: of the tube's radius, its axis through a point
/// of the tube's middle circle along that circle's tangent there, the cylinder is the ball there
/// carried along the tangent, and touches the torus round the meridian through that point (a torus
/// at an end pose of a slide square to its axis, and the cylinder its meridian sweeps, #721 X6). The
/// meridian, or none for any other cylinder.
fn tube_along_torus(cy: &Frame,r: f64,t: &Frame,big: f64,small: f64,tol: f64) -> Option<Curve> {
    let (z,a) = (unit(t.z),unit(cy.z));
    if (r.abs()-small.abs()).abs() > tol || dot(a,z).abs() > 1e-9 { return None }
    // where the cylinder's axis passes nearest the torus's centre
    let foot = add(cy.o,scale(a,dot(sub(t.o,cy.o),a)));
    let rel = sub(foot,t.o);
    if dot(rel,z).abs() > tol || (norm(rel)-big.abs()).abs() > tol || dot(unit(rel),a).abs() > 1e-9 { return None }
    Some(Curve::Circle(Frame::new(foot,a,unit(rel)),r.abs()))
}

/// **Two tori touching round a meridian** (#721 F2c5: a torus at an end pose of a turn about an axis
/// along its own, and the torus a meridian of it in the plane through both axes turns into): their
/// axes along one another, their middle circles at one height and touching, their tubes of one
/// radius. They touch round the meridian through the point the middle circles touch at, and cross
/// beside it: in each level `h` across the tube each is two circles about its own axis, of radii
/// `R ± √(r² − h²)`, and of the four pairs, two touch at that meridian's points and one crosses where
/// `A cos θ + B sin θ = C(h)` (`branches`, the first's circle's angle `θ`), its two roots joining
/// where they turn. The meridian and the crossing's branches, or none for any other two tori.
fn tori_touching(a: &Surface,b: &Surface,(t,big,small): (&Frame,f64,f64),(u,bog,smell): (&Frame,f64,f64),tol: f64) -> Option<Vec<Curve>> {
    let z = unit(t.z);
    let (r,big,bog) = (small.abs(),big.abs(),bog.abs());
    if norm(cross(z,u.z)) > 1e-12 || (r-smell.abs()).abs() > tol || dot(sub(u.o,t.o),z).abs() > tol { return None }
    let d = sub(u.o,t.o);
    let d = sub(d,scale(z,dot(d,z)));
    let gap = norm(d);
    if gap <= tol || ((gap-(big+bog)).abs() > tol && (gap-(big-bog).abs()).abs() > tol) { return None }
    // where the middle circles touch, on the line of their centres
    let w = scale(d,1./gap);
    let m = [add(t.o,scale(w,big)),sub(t.o,scale(w,big))].into_iter()
        .find(|&m| (norm(sub(sub(m,u.o),scale(z,dot(sub(m,u.o),z))))-bog).abs() <= 2.*tol)?;
    let mut out = vec![Curve::Circle(Frame::new(m,cross(z,w),w),r)];
    let (x,y) = (t.x,cross(z,t.x));
    let level = |v: f64| { let h = r*(2.*v-1.); (h,(r*r-h*h).max(0.).sqrt()) };
    for (s1,s2) in [(1.,1.),(1.,-1.),(-1.,1.),(-1.,-1.)] {
        let (r1,r2) = (big+s1*r,bog+s2*r);
        if ((r1-r2).abs()-gap).abs() <= tol || (r1+r2-gap).abs() <= tol { continue }
        let coefficients = |v: f64| {
            let (_,q) = level(v);
            let (r1,r2) = (big+s1*q,bog+s2*q);
            (-2.*r1*dot(d,x),-2.*r1*dot(d,y),r2*r2-gap*gap-r1*r1)
        };
        let point = |th: f64,v: f64| {
            let (h,q) = level(v);
            let (sn,cs) = th.dsin_cos();
            add(add(t.o,scale(z,h)),scale(add(scale(x,cs),scale(y,sn)),big+s1*q))
        };
        out.extend(circle_branches(&coefficients,&[],&point,&|pts,closed| Curve::Traced(std::sync::Arc::new(
            super::geom::Traced {a:a.clone(),b:b.clone(),pts,closed}))));
    }
    Some(out)
}

/// The radius of a cylinder or a cone at height `h` along its own axis from its frame's origin.
fn radius_at(s: &Surface,h: f64) -> Option<f64> {
    match *s {
        Surface::Cylinder(_,r) => Some(r.abs()),
        Surface::Cone(_,r,half) => { let (sa,ca) = half.dsin_cos(); (ca > 1e-12).then(|| r+h*sa/ca) }
        _ => None,
    }
}

/// **The line two cylinders or cones with parallel axes touch along**, where they do: at two
/// heights the circles they cut from a plane square to the axes touch, within `tol` (the distance
/// between their centres the sum or the difference of their radii), and the line through the two
/// points they touch at lies on both. None where they do not touch so at both heights, or a radius
/// there is none (past an apex).
fn parallel_touch(a: &Surface,b: &Surface,tol: f64) -> Option<Curve> {
    let (f,g) = (*a.frame(),*b.frame());
    let z = unit(f.z);
    let sense = dot(g.z,z).signum();
    let scale_of = radius_at(a,0.)?.max(radius_at(b,dot(sub(f.o,g.o),z)*sense)?).max(1.);
    let touch = |h: f64| -> Option<V> {
        let c1 = add(f.o,scale(z,h));
        let hg = dot(sub(c1,g.o),z)*sense;
        let c2 = add(g.o,scale(g.z,hg));
        let (r1,r2) = (radius_at(a,h)?,radius_at(b,hg)?);
        if !(r1 > tol && r2 > tol) { return None }
        let w = sub(sub(c2,c1),scale(z,dot(sub(c2,c1),z)));
        let d = norm(w);
        if d <= tol { return None }
        let w = scale(w,1./d);
        // externally (the far circle beside it), or the one within the other
        if (d-(r1+r2)).abs() <= tol { Some(add(c1,scale(w,r1))) }
        else if (d-(r1-r2).abs()).abs() <= tol { Some(add(c1,scale(w,if r1 >= r2 { r1 } else { -r1 }))) }
        else { None }
    };
    let (h0,h1) = [(0.,scale_of),(0.,-scale_of)].into_iter().find(|&(h0,h1)| touch(h0).is_some() && touch(h1).is_some())?;
    let (p,q) = (touch(h0)?,touch(h1)?);
    Some(Curve::Line {p,d:unit(sub(q,p))})
}

/// Whether two lines' sheets are screwed by one screw: their axes along one another and through
/// one another within `tol`, and the same pitch.
fn one_screw(l: &super::geom::ScrewedLine,m: &super::geom::ScrewedLine,tol: f64) -> bool {
    let off = sub(m.centre,l.centre);
    norm(cross(l.axis,m.axis)) <= 1e-12 && norm(sub(off,scale(l.axis,dot(off,l.axis)))) <= tol
        && (l.pitch()-m.pitch()).abs() <= 1e-12*l.pitch().abs().max(m.pitch().abs())
}

/// **The helix two sheets of one screw meet in, as `u` on the first** (`on`): its line meets the
/// second's carried to its height, both square to the axis — `Some(None)` where the two lines run
/// parallel there and apart, so the sheets meet nowhere. `None` where they are not two such sheets,
/// or are one.
pub(crate) fn helix_between(on: &super::geom::Blend,by: &super::geom::Blend,tol: f64) -> Option<Option<f64>> {
    let (l,m) = (by.screwed_line()?,on.screwed_line()?);
    if !one_screw(&l,&m,tol) || !l.level() || !m.level() { return None }
    // `by`'s line screwed to `on`'s height, and where the two meet in that level
    let turn = dot(sub(m.p0,l.p0),l.axis)/l.pitch();
    let screwed = super::geom::Carry::Screw {center:l.centre,axis:l.axis,angle:turn,advance:l.pitch()*turn};
    let (p,d) = (screwed.at(l.p0,1.),screwed.dir(l.run,1.));
    let cr = cross(d,m.run);
    if norm(cr) <= 1e-12*norm(d)*norm(m.run) {
        // parallel: apart, or one line (one sheet)
        let gap = sub(m.p0,p);
        return (norm(sub(gap,scale(d,dot(gap,d)/dot(d,d)))) > tol).then_some(None)
    }
    // p0 + t run on the line through p along d: (p0 − p + t run) × d = 0
    Some(Some(dot(cross(sub(m.p0,p),d),cr)/dot(cr,cr)))
}

/// **Two sheets lines screw into by one screw meet in helices** (#777 M5: a leaning edge's patch and
/// its neighbour's sheet, which share the helix of their common corner): a point of `on`'s line,
/// `p(u) = p₀ + u Δ`, lies on `by`'s sheet where the screw carries some point of `by`'s line onto it,
/// `p(u) = screw_θ(q(w))` — three equations in `(u, w, θ)`, `θ` over the turns the two sheets' spans
/// differ by, isolated with a bound (`util::isolate::solve`, #777 M1): `q` is a line, so the second
/// derivatives are `|∂²/∂w∂θ| ≤ |Δ_by|` and `|∂²/∂θ²| ≤` its reach from the axis. Each root's
/// meeting is the helix of `p(u)`, `on`'s rail there (`Curve::Iso`), exact where a trace would fit it
/// — and a fit's end, leaning a few degrees off, once ordered the curves leaving a corner wrongly.
/// `None` for sheets of different screws, or where the search gives up.
fn screwed_lines_meeting(on: &std::sync::Arc<super::geom::Blend>,by: &super::geom::Blend,tol: f64) -> Option<Vec<Curve>> {
    let (m,l) = (on.screwed_line()?,by.screwed_line()?);
    if !one_screw(&l,&m,tol) { return None }
    let (c,k) = (l.centre,l.axis);
    // the turn `θ` that carries `by`'s line onto a point of `on`'s: `by`'s span less `on`'s
    let (a0,a1) = (m.angle.min(0.),m.angle.max(0.));
    let (b0,b1) = (l.angle.min(0.),l.angle.max(0.));
    let span = [b0-a1,b1-a0];
    // the screw a radian of turn at a time, `θ` its parameter
    let screw = Carry::Screw {center:c,axis:k,angle:1.,advance:l.pitch()};
    let f = |x: [f64;3]| -> ([f64;3],[[f64;3];3]) {
        let (u,w,t) = (x[0],x[1],x[2]);
        let p = add(m.p0,scale(m.run,u));
        let sq = screw.at(add(l.p0,scale(l.run,w)),t);
        let (rq,dt) = (screw.dir(l.run,t),screw.rate(sq));
        (std::array::from_fn(|i| p[i]-sq[i]),std::array::from_fn(|i| [m.run[i],-rq[i],-dt[i]]))
    };
    let reach = norm(sub(l.p0,c))+norm(l.run);
    let h = 2.*norm(l.run)+reach;
    let second = |_: &[[f64;2];3]| [h;3];
    let size = norm(m.run).max(norm(l.run)).max(reach);
    let width = tol*1e-2/size.max(1e-300);
    let near = |x: [f64;3]| { let (v,_) = f(x); norm(v) <= tol };
    let found = crate::util::isolate::solve::<3>(&f,&second,[[0.,1.],[0.,1.],span],width,&near).ok()?;
    let mut us: Vec<f64> = found.into_iter().map(|r| r.at[0].clamp(0.,1.)).collect();
    us.sort_by(f64::total_cmp);
    us.dedup_by(|y,x| (*y-*x).abs()*norm(m.run) <= tol);
    Some(us.into_iter().map(|u| Curve::Iso(on.clone(),u)).collect())
}

/// **A sheet a line screws into meets a sheet a level circle screws into by the same screw in
/// helices** (#780 C2: a cone's contact ruling under a screw, screwed into a patch, against its own
/// rim's sheet, which share the helix of the ruling's end on the rim): the screw keeps each sheet,
/// so the line's point `p(u)` is on the circle's where the screw carries it to the circle's height
/// onto the circle — `g(u) = |w(u) − R_{s(u)} c₀|² − r²`, `w(u)` the point's offset from the axis,
/// `c₀` the circle's centre's, `s(u)` the turn its climb to the circle's height takes, linear in
/// `u` — and where the helix of `p(u)` over the line's sheet reaches heights the circle's sheet
/// holds. Isolated with a bound on its derivatives (`util::isolate`), each root's meeting the
/// line's sheet's rail there (`Curve::Iso`). `None` where the two are not such sheets of one screw,
/// or where the search gives up.
fn screwed_line_circle(on: &std::sync::Arc<super::geom::Blend>,by: &super::geom::Blend,tol: f64) -> Option<Vec<Curve>> {
    let (l,c) = (on.screwed_line()?,by.level_circle()?);
    let k = c.axis;
    let perp = |x: V| sub(x,scale(k,dot(x,k)));
    if l.angle == 0. || c.angle == 0. { return None }
    let pitch = c.rise/c.angle;
    if norm(cross(l.axis,k)) > 1e-12 || norm(perp(sub(l.centre,c.centre))) > tol
        || (l.pitch()-pitch).abs() > 1e-12*pitch.abs().max(1e-300) { return None }
    let (w0,m) = (perp(sub(l.p0,c.centre)),perp(l.run));
    let c0 = perp(sub(c.frame.o,c.centre));
    // the line's point's height over the circle's
    let z = |u: f64| dot(sub(add(l.p0,scale(l.run,u)),c.frame.o),k);
    // the turn that takes the circle's sheet up to the line's point's height, and its rate
    let (s0,s1) = (z(0.)/pitch,dot(l.run,k)/pitch);
    let turned = |u: f64| { let (sn,cs) = (s0+s1*u).dsin_cos(); add(scale(c0,cs),scale(cross(k,c0),sn)) };
    let g = |u: f64| { let e = sub(add(w0,scale(m,u)),turned(u)); dot(e,e)-c.r*c.r };
    let dg = |u: f64| { let q = turned(u); 2.*dot(sub(add(w0,scale(m,u)),q),sub(m,scale(cross(k,q),s1))) };
    let (big,lean) = (norm(w0)+norm(m)+norm(c0),norm(m)+s1.abs()*norm(c0));
    let bound = |_: f64,_: f64| [2.*big*lean,2.*lean*lean+2.*s1*s1*big*norm(c0)];
    let size = big.max(c.r).max(1e-300);
    let roots = crate::util::isolate::roots(&g,&dg,&bound,[0.,1.],tol*1e-3/size,&|u| g(u).abs() <= 2.*c.r*tol).ok()?;
    // the circle's sheet's `v` at the line's point at `u` carried `t` along the line's sheet
    let v_at = |u: f64,t: f64| (z(u)+l.rise*dot(l.axis,k)*t)/c.rise;
    let slack = 1e-9;
    Some(roots.into_iter().map(|r| r.at).filter(|&u| {
        let (a,b) = (v_at(u,0.),v_at(u,1.));
        a.min(b) <= 1.+slack && a.max(b) >= -slack
    }).map(|u| Curve::Iso(on.clone(),u)).collect())
}

/// **Where a sheet a line screws into meets a surface it shares the screw's axis with**, in closed
/// form (`Blend::screwed_line`): the screw keeps the cylinder about its axis and the other sheet of
/// one screw, so they meet where the line does, carried — helices, each the sheet's rail at the
/// line's point there (`Curve::Iso`). A coaxial cylinder of radius `r` meets the line where
/// `|ρ(u)| = r`, two points, or one where the line passes the axis at `r` within `tol` (the sheet
/// tangent to the cylinder along that point's helix, judged by the gap); another sheet of the same
/// pitch, at the point where its level line meets this one's line carried to its height
/// (`helix_between`). A plane square to the axis meets a level line's sheet in one ruling, the line
/// carried to its height, where that lies on the sheet; any other plane in its cut
/// (`Curve::SheetCut`) and the rulings it holds. Anything else: `None`, traced.
fn screwed_meeting(a: &Surface,b: &Surface,tol: f64) -> Option<Vec<Curve>> {
    let (blend,other) = match (a,b) { (Surface::Blend(_,x),o) | (o,Surface::Blend(_,x)) => (x,o),_ => return None };
    let sheet = if matches!(a,Surface::Blend(..)) { a } else { b };
    if let Some(curves) = screwed_circle_meeting(sheet,blend,other,tol) { return Some(curves) }
    if let Surface::Sphere(sp,r) = other { if let Some(circle) = pipe_end(blend,sp.o,r.abs(),tol) {
        return Some(vec![circle]) } }
    if let Surface::Cylinder(cy,r) = other { if let Some(curves) = screwed_circle_round(blend,cy,r.abs(),tol) { return Some(curves) } }
    if let Surface::Plane(pl) = other {
        if let Some(curves) = screwed_circle_level(sheet,blend,pl,tol).or_else(|| pipe_plane(sheet,blend,pl,tol)) { return Some(curves) }
    }
    if let Surface::Blend(_,ob) = other {
        if let Some(curves) = pipes_meeting(blend,ob,tol).or_else(|| screwed_line_circle(blend,ob,tol)).or_else(|| screwed_line_circle(ob,blend,tol)) {
            return Some(curves)
        }
    }
    let line = blend.screwed_line()?;
    let (o,z,a0,run) = (line.centre,line.axis,line.p0,line.run);
    let perp = |x: V| sub(x,scale(z,dot(x,z)));
    let iso = |u: f64| Curve::Iso(blend.clone(),u);
    let ruling = |v: f64| Curve::Line {p:blend.carry.at(a0,v),d:unit(blend.carry.dir(run,v))};
    match other {
        Surface::Cylinder(c,r) if norm(cross(c.z,z)) <= 1e-12 && norm(perp(sub(c.o,o))) <= tol => {
            let (w0,w1) = (perp(sub(a0,o)),perp(run));
            let qa = dot(w1,w1);
            if qa <= 1e-24*dot(run,run) { return None }
            // the line's point nearest the axis, and how far it passes it
            let u = -dot(w0,w1)/qa;
            let foot = norm(add(w0,scale(w1,u)));
            if (foot-r).abs() <= tol { return Some(vec![iso(u)]) }
            if foot > *r { return Some(vec![]) }
            let h = ((r-foot)*(r+foot)).sqrt()/qa.sqrt();
            Some(vec![iso(u-h),iso(u+h)])
        }
        Surface::Plane(f) if norm(cross(f.z,z)) <= 1e-12 && line.level() => {
            // the ruling at the plane's height: `v` where the line climbs to it
            let v = line.v_at(f.o);
            let slack = tol/line.rise.abs();
            if !(-slack..=1.+slack).contains(&v) { return Some(vec![]) }
            Some(vec![ruling(v)])
        }
        Surface::Plane(f) => {
            // the rulings running parallel to the plane: `β(v) = n·R(θ v) Δ` none, `P cos θ + Q sin θ
            // + R₀ = 0`; held where the ruling stands within `tol` of the plane
            let (n,h,angle) = (f.z,dot(f.z,f.o),line.angle);
            let (p,q,r0) = (dot(n,perp(run)),dot(n,cross(z,run)),dot(n,z)*dot(z,run));
            let amplitude = p.hypot(q);
            let mut held = Vec::new();
            if amplitude > 1e-12*norm(run) && r0.abs() <= amplitude {
                let (phi,w) = (q.atan2(p),(-r0/amplitude).clamp(-1.,1.).acos());
                for base in [phi+w,phi-w] {
                    // every turn of the root the sheet's stretch holds, a margin past it
                    let first = ((angle.min(0.)-1e-2*angle.abs()-base)/std::f64::consts::TAU).floor() as i64;
                    let last = ((angle.max(0.)+1e-2*angle.abs()-base)/std::f64::consts::TAU).ceil() as i64;
                    for m in first..=last {
                        let v = (base+std::f64::consts::TAU*m as f64)/angle;
                        if !(-1e-2..=1.+1e-2).contains(&v) { continue }
                        // the ruling's reach to the plane, `α`
                        if (dot(n,blend.carry.at(a0,v))-h).abs() <= tol && !held.iter().any(|&x: &f64| (x-v).abs() <= 1e-12) { held.push(v) }
                    }
                }
            }
            held.sort_by(f64::total_cmp);
            let mut out: Vec<Curve> = held.iter().map(|&v| ruling(v)).collect();
            out.push(Curve::SheetCut(std::sync::Arc::new(super::geom::SheetCut::new(blend.clone(),n,h,held)?)));
            Some(out)
        }
        Surface::Cone(..) => screwed_line_cone(sheet,blend,&line,other,tol),
        Surface::Blend(_,ob) => match helix_between(ob,blend,tol) {
            Some(h) => Some(h.map(|u| Curve::Iso(ob.clone(),u)).into_iter().collect()),
            None => screwed_lines_meeting(ob,blend,tol),
        },
        _ => None,
    }
}

/// **A sheet a line screws into meets a cone in closed form** (#780 C2: a cone's contact ruling
/// under a screw, screwed into a patch, against the cone at an end pose, which holds that ruling
/// and touches the patch all along it). The sheet's ruling at `v`, `S(u, v) = P(v) + u D(v)`, meets
/// the cone `|w|² = (r + h tan α)²` where `A(v) u² + B(v) u + C(v) = 0`: two roots `u±(v)` over the
/// `v` where the discriminant holds (`branches`), run off where a root leaves the sheet's reach. A
/// ruling the cone holds (`A = B = C = 0`, where `C` crosses nought or, touching, turns:
/// `roots::sampled_roots`) is returned whole, and the cut runs into it at its limit there, the roots
/// of the coefficients' first differences across it where the sheet crosses the cone along it, of
/// their second where it touches (read a hair off it, where the coefficients' rounding is still
/// below what they have grown to). Its points are written in closed form and handed over as traced
/// meetings (`Traced`), so no near touch is traced.
fn screwed_line_cone(sheet: &Surface,blend: &std::sync::Arc<super::geom::Blend>,line: &super::geom::ScrewedLine,other: &Surface,tol: f64)
    -> Option<Vec<Curve>> {
    use super::geom::Traced;
    let Surface::Cone(f,r0,al) = *other else { return None };
    let (t,z) = (al.dtan(),unit(f.z));
    let (p0,run) = (line.p0,line.run);
    let coefficients = |v: f64| -> [f64;3] {
        let (p,d) = (blend.carry.at(p0,v),blend.carry.dir(run,v));
        let a0 = sub(p,f.o);
        let (h0,h1) = (dot(a0,z),dot(d,z));
        let (w0,w1) = (sub(a0,scale(z,h0)),sub(d,scale(z,h1)));
        let e = r0+t*h0;
        [dot(w1,w1)-t*t*h1*h1,2.*(dot(w0,w1)-t*h1*e),dot(w0,w0)-e*e]
    };
    let size = norm(sub(p0,f.o))+norm(run)+r0.abs();
    let hair = 1e-4;
    // the rulings the cone holds: the roots of `C` (the ruling's foot on the cone), crossings or
    // touches, and the sheet's own ends, where all three coefficients vanish
    let held: Vec<f64> = {
        const N: usize = 2048;
        let c_of = |v: f64| coefficients(v)[2];
        let ts: Vec<f64> = (0..=N).map(|i| i as f64/N as f64).collect();
        let cs: Vec<f64> = ts.iter().map(|&v| c_of(v)).collect();
        // (a root read within rounding of the squared lengths `C` is: one touching grows as the
        // square of the way from it, and a looser reading would take a stretch of rulings for it)
        let found = crate::util::roots::sampled_roots(c_of,|v| (c_of(v+1e-6)-c_of(v-1e-6))/2e-6,&ts,&cs,1e-9*size*size,1e-12);
        let mut held: Vec<f64> = Vec::new();
        for v in [0.,1.].into_iter().chain(found.into_iter().map(|r| r.at)) {
            if coefficients(v).iter().all(|x| x.abs() <= 2.*size*tol) && !held.iter().any(|&w| (w-v).abs() <= hair) { held.push(v) }
        }
        held
    };
    // the two roots, larger first (the larger by `quadratic`'s rule, without cancellation; one at
    // infinity where the ruling runs along the cone's generators)
    let quadratic = |[a,b,c]: [f64;3]| -> Option<[f64;2]> {
        let disc = b*b-4.*a*c;
        if !(disc >= 0.) { return None }
        let q = -0.5*(b+disc.sqrt().copysign(b));
        if q == 0. { return (a != 0.).then(|| [-b/(2.*a);2]) }
        let (x,y) = (q/a,c/q);
        Some(if x >= y { [x,y] } else { [y,x] })
    };
    // at a held ruling, approached from `side`, the roots of the first differences across it, or
    // where they stand within rounding of none (the sheet touching the cone along it), the second
    let limit = |w: f64,side: f64| -> Option<[f64;2]> {
        let (q1,q0) = (coefficients(w+hair),coefficients(w-hair));
        let first: [f64;3] = std::array::from_fn(|k| side*(q1[k]-q0[k]));
        let second: [f64;3] = std::array::from_fn(|k| q1[k]+q0[k]);
        let big = |q: &[f64;3]| q.iter().fold(0f64,|m,x| m.max(x.abs()));
        quadratic(if big(&first) > big(&second) { first } else { second })
    };
    let reach = |u: f64| (-1.0..=2.).contains(&u);
    let point = |u: f64,v: f64| reach(u).then(|| add(blend.carry.at(p0,v),scale(blend.carry.dir(run,v),u)));
    // (no chord longer than a step of the cone's circle: a long sheet's even samples, a turn of a line
    // screwed across the box, once stood chords a twentieth of its radius apart)
    let step = std::f64::consts::PI/1024.*r0.abs().max(1e-3*size);
    let apart = |x: f64,y: f64,v: f64,w: f64| match (point(x,v),point(y,w)) { (Some(p),Some(q)) => distance(p,q) > step,_ => false };
    let mut out: Vec<Curve> = held.iter().map(|&v| Curve::Line {p:blend.carry.at(p0,v),d:unit(blend.carry.dir(run,v))}).collect();
    let roots = |v: f64| quadratic(coefficients(v));
    out.extend(branches(&Branches {roots:&roots,real:&|v| roots(v).is_some(),point:&point,apart:&apart,coincide:&held,limit:&limit,hair},
        &|pts,closed| Curve::Traced(std::sync::Arc::new(Traced {a:sheet.clone(),b:other.clone(),pts,closed}))));
    Some(out)
}

/// **A level circle screwed**, read off a blend (`Blend::level_circle`): the screw's centre, unit
/// axis, turn and rise over the blend's `v`, and the circle (its frame, square to the axis, and
/// radius).
fn screwed_circle(blend: &super::geom::Blend) -> Option<(V,V,f64,f64,Frame,f64)> {
    blend.level_circle().map(|c| (c.centre,c.axis,c.angle,c.rise,c.frame,c.r))
}

/// **Two circles screwed by one screw, as one surface**: the second's circle, screwed to the
/// first's height, is the first's — a sheet a rim screws into, carried on along the screw, holds the
/// other rim of a tool as tall as a whole number of leads (#721 F2c5), and a ball's pipe entered
/// stretch by stretch is one surface over its stretches (#780 L1). One surface only where the two
/// sheets share some of the screw's turn (`screwed_overlap`); apart, they are two stretches of one
/// sheet.
fn one_screwed_circle(a: &super::geom::Blend,b: &super::geom::Blend,tol: f64) -> bool {
    screwed_overlap(a,b,tol).is_some_and(|o| o.shared > 0.)
}

/// How much of the screw's turn two sheets of one circle under one screw share (`screwed_overlap`),
/// in the turns of the first's screw, a radian at a time, from the first's circle.
struct Overlap {
    /// What they share: nought where they abut, within the turn `tol` makes at the circle's reach
    /// from the axis; below it where they stand apart.
    shared: f64,
    /// The turns the first covers, `[0, A₁]` either way round.
    mine: [f64;2],
    /// The turns the second covers, `[−T, A₂ − T]` either way round, `T` the turn that lifts its
    /// circle onto the first's.
    theirs: [f64;2],
}

/// **Two sheets of one circle under one screw** — a level circle's (`Blend::level_circle`) or a
/// ball's pipe's (`Blend::pipe`) — read in the turns of the first's screw (`Overlap`): the screws'
/// axes along one line either way (a screw about `−k` turning the other way), the same pitch and
/// radius, and the second's circle carried by the turn `T` that lifts its centre to the first's
/// height the first's circle. `None` where they are not.
fn screwed_overlap(a: &super::geom::Blend,b: &super::geom::Blend,tol: f64) -> Option<Overlap> {
    use super::geom::Carry;
    // each one's screw a radian at a time (its centre, unit axis and pitch), its turn about that
    // axis, and its circle's frame and radius
    let read = |x: &super::geom::Blend| -> Option<(V,V,f64,f64,Frame,f64)> {
        if let Some(c) = x.level_circle() { return (c.angle != 0.).then(|| (c.centre,c.axis,c.rise/c.angle,c.angle,c.frame,c.r)) }
        let p = x.pipe()?;
        let (Carry::Screw {center,axis:k,advance:pitch,..},Carry::Screw {axis,angle,..}) = (p.radian()?,x.carry) else { return None };
        Some((center,k,pitch,angle*dot(axis,k).signum(),p.frame,p.r))
    };
    let ((c1,k1,pitch1,a1,f1,r1),(c2,k2,pitch2,a2,f2,r2)) = (read(a)?,read(b)?);
    let perp = |x: V| sub(x,scale(k1,dot(x,k1)));
    if norm(cross(k1,k2)) > 1e-12 || norm(perp(sub(c2,c1))) > tol || (pitch1-pitch2).abs() > 1e-12*pitch1.abs()
        || (r1-r2).abs() > tol || pitch1 == 0. { return None }
    // the turn that lifts the second's circle to the first's: the same circle there
    let turn = dot(sub(f1.o,f2.o),k1)/pitch1;
    let screw = Carry::Screw {center:c1,axis:k1,angle:turn,advance:pitch1*turn};
    if norm(sub(screw.at(f2.o,1.),f1.o)) > tol || norm(cross(screw.dir(f2.z,1.),f1.z)) > 1e-9 { return None }
    let a2 = a2*dot(k1,k2).signum();
    let (mine,theirs) = ([a1.min(0.),a1.max(0.)],[a2.min(0.)-turn,a2.max(0.)-turn]);
    let slack = tol/(norm(perp(sub(f1.o,c1)))+r1).hypot(pitch1);
    let shared = mine[1].min(theirs[1])-mine[0].max(theirs[0]);
    Some(Overlap {shared:if shared.abs() <= slack { 0. } else { shared },mine,theirs})
}

/// **The sheet any circle screws into meets a cylinder about the screw's axis in helices** (#721
/// F2c5: a ball's great circle square to its centre's velocity, leaning wherever the screw climbs):
/// the screw keeps each point's distance from its axis, so the meeting is the helices of the
/// circle's points at the cylinder's radius, the roots of `|ρ(θ)|² − R²` round the circle — a
/// trigonometric polynomial of degree two, four roots at most — isolated with a bound on its
/// derivatives (`util::isolate`), a touch where it only reaches nought (the circle tangent to the
/// cylinder) placed where its derivative changes sign. `None` for any other sheet or cylinder.
pub(crate) fn screwed_circle_round(blend: &std::sync::Arc<super::geom::Blend>,cy: &Frame,big: f64,tol: f64) -> Option<Vec<Curve>> {
    let super::geom::Carry::Screw {center,axis,..} = blend.carry else { return None };
    let k = unit(axis);
    if norm(cross(unit(cy.z),k)) > 1e-12 { return None }
    let perp = |x: V| sub(x,scale(k,dot(x,k)));
    if norm(perp(sub(cy.o,center))) > tol { return None }
    let (Curve::Circle(f,r),Curve::Circle(..)) = (&blend.a,&blend.b) else { return None };
    let same = |q: V,p: V| norm(sub(q,p)) <= 1e-9*(1.+norm(p));
    let span = blend.ta[1]-blend.ta[0];
    if !(0..3).all(|i| same(blend.b.point(blend.tb[0]+(blend.tb[1]-blend.tb[0])*i as f64/2.),blend.a.point(blend.ta[0]+span*i as f64/2.))) { return None }
    let r = r.abs();
    let g = |th: f64| { let (sn,cs) = th.dsin_cos(); let d = perp(sub(add(f.o,add(scale(f.x,r*cs),scale(f.y,r*sn))),center)); dot(d,d)-big*big };
    let u_of = |th: f64| blend.u_of_angle(th);
    // `g' = 2 d · P'⊥`, `g'' = 2 (|P'⊥|² + d · P''⊥)`, the circle's point `P` at `r` from its
    // centre and `d` its offset from the axis, within `D = |(o − c)⊥| + r`
    let dg = |th: f64| {
        let (sn,cs) = th.dsin_cos();
        let d = perp(sub(add(f.o,add(scale(f.x,r*cs),scale(f.y,r*sn))),center));
        2.*dot(d,perp(add(scale(f.x,-r*sn),scale(f.y,r*cs))))
    };
    let far = norm(perp(sub(f.o,center)))+r;
    let bound = |_: f64,_: f64| [2.*far*r,2.*(r*r+far*r)];
    let tau = std::f64::consts::TAU;
    // the circle within the tolerance of the cylinder at `th`, and the angle the tolerance spans
    let on = |th: f64| g(th).abs() <= 2.*big*tol;
    let step = tol/r.max(1e-300);
    let found: Vec<f64> = crate::util::isolate::roots(&g,&dg,&bound,[0.,tau],tol*1e-3/r.max(1e-300),&on).ok()?
        .into_iter().map(|x| x.at).collect();
    // a run of roots the circle keeps within the tolerance of the cylinder between is one helix
    // (#780 R2's rule, `query::settled`): a double root rounding parts into several a hair apart —
    // a ball's great circle tangent to the cylinder its outer equator screws round, four roots
    // within 5e-8 rad (#780 C1) — is the one helix the sheet touches the cylinder along, and read
    // four times it left the faces it bounds unclosed. A touch is placed where `g'` changes sign
    // across the run, a simple root of it settled to rounding (the run's middle only stands within
    // the square root of rounding of a double root: 2.6e-7 off, where the helix, the cylinder's rim
    // and the pipe's meeting with the rim's plane all touch at its start); a lone crossing where it
    // was found
    let pad = step.min(1e-3);
    let mut roots: Vec<f64> = found.chunk_by(|&x,&y| (y-x)*r <= tol || (1..4).all(|j| on(x+(y-x)*j as f64/4.))).map(|run| {
        let (first,last) = (run[0],run[run.len()-1]);
        let (a,b) = (first-pad,last+pad);
        let (da,db) = (dg(a),dg(b));
        let crossing = run.len() == 1 && g(a)*g(b) < 0.;
        if !crossing && da*db < 0. { crate::util::roots::bracketed_root(|t| Some(dg(t)),a,da,b,db,4e-16*b.abs().max(1.)) }
        else { 0.5*(first+last) }
    }).collect();
    // the circle closes: a root at its end is the one at its start
    if roots.len() > 1 && (roots[roots.len()-1]-tau-roots[0]).abs()*r <= tol { roots.pop(); }
    Some(roots.into_iter().map(|th| Curve::Iso(blend.clone(),u_of(th))).collect())
}

/// **The sheet any circle screws into meets a plane square to the screw's axis**, in closed form
/// (#721 F2c5: a ball's pipe against a level plane through it): the screw lifts every point alike, so
/// the circle's point at `u` reaches the plane at the `v` its height gives, `v(u) = (h − z(u))/rise`,
/// and the meeting is `S(u, v(u))` over the runs of `u` where that `v` is on the sheet — written in
/// closed form at points dense where the run ends and wherever the point turns, and handed over as a
/// traced meeting. `None` for any other sheet or plane.
fn screwed_circle_level(sheet: &Surface,blend: &std::sync::Arc<super::geom::Blend>,pl: &Frame,tol: f64) -> Option<Vec<Curve>> {
    let super::geom::Carry::Screw {center,axis,advance,..} = blend.carry else { return None };
    let k = unit(axis);
    if norm(cross(unit(pl.z),k)) > 1e-12 || blend.level_circle().is_some() { return None }
    let (Curve::Circle(..),Curve::Circle(..)) = (&blend.a,&blend.b) else { return None };
    let same = |q: V,p: V| norm(sub(q,p)) <= 1e-9*(1.+norm(p));
    if !(0..3).all(|i| same(blend.d1([i as f64/2.,1.]).0,blend.carry.at(blend.d1([i as f64/2.,0.]).0,1.))) { return None }
    let rise = advance*dot(axis,k);
    if rise == 0. { return None }
    let h = dot(sub(pl.o,center),k);
    let v_of = |u: f64| (h-dot(sub(blend.d1([u,0.]).0,center),k))/rise;
    let slack = tol/rise.abs();
    let on = |u: f64| (-slack..=1.+slack).contains(&v_of(u));
    let point = |u: f64| blend.d1([u,v_of(u).clamp(0.,1.)]).0;
    const N: usize = 2048;
    let us: Vec<f64> = (0..=N).map(|i| i as f64/N as f64).collect();
    let ok: Vec<bool> = us.iter().map(|&u| on(u)).collect();
    // a closed circle whose every point reaches the plane: one loop
    if ok.iter().all(|&o| o) {
        let mut pts: Vec<V> = us.iter().map(|&u| point(u)).collect();
        pts[N] = pts[0];
        return Some(vec![Curve::Traced(std::sync::Arc::new(super::geom::Traced {a:sheet.clone(),b:Surface::Plane(*pl),pts,closed:true}))])
    }
    // runs of `u` (round the circle, from where it leaves the sheet), each end refined
    let start = (0..N).find(|&i| !ok[i])?;
    let mut out = Vec::new();
    let mut i = 0;
    while i < N {
        let j = (start+i)%N;
        if !ok[j] { i += 1; continue }
        let first = i;
        while i < N && ok[(start+i)%N] { i += 1 }
        // a run's end where its `v` reaches the sheet's edge, not the tolerance past it: there the
        // meeting ends at the vertex where the edge's circle crosses the plane, on both (#791: the
        // tolerance past, clamped onto that circle, it stood a millionth off the plane, where it, a
        // cylinder's rim and the helix the sheet touches that cylinder along all touch); a run
        // wholly within the tolerance past the edge (a graze) is read to the tolerance
        let refine = |inside: f64,outside: f64| {
            let (lo,hi) = if (0.0..=1.).contains(&v_of(inside)) { (0.,1.) } else { (-slack,1.+slack) };
            crate::util::roots::bisect(inside,outside,|a,b| 0.5*(a+b),|a,b| { let m = 0.5*(a+b); m != a && m != b },
                |m| (lo..=hi).contains(&v_of(m))).0
        };
        let at = |n: usize| { let x = (start+n) as f64/N as f64; x };
        let (u0,u1) = (refine(at(first),at(first)-1./N as f64),refine(at(i-1),at(i)));
        let mut run: Vec<f64> = vec![u0];
        run.extend((1..24).map(|q| u0+(at(first)-u0)*0.5f64.powi(24-q)).filter(|&u| u > u0 && u < at(first)));
        run.extend((first..i).map(at).filter(|&u| u > u0 && u < u1));
        run.extend((1..24).rev().map(|q| u1-(u1-at(i-1))*0.5f64.powi(24-q)).filter(|&u| u > at(i-1) && u < u1));
        run.push(u1);
        run.sort_by(f64::total_cmp);
        run.dedup();
        let pts: Vec<V> = run.iter().map(|&u| point(u.rem_euclid(1.))).collect();
        if pts.len() >= 2 && norm(sub(pts[0],pts[pts.len()-1])) > tol || pts.len() > 2 {
            out.push(Curve::Traced(std::sync::Arc::new(super::geom::Traced {a:sheet.clone(),b:Surface::Plane(*pl),pts,closed:false})));
        }
    }
    Some(out)
}

/// **Where two stretches of one ball's pipe meet** (#780 L1, X5: a ball screwed through more than a
/// turn at a lead short of its diameter, its pipe entered stretch by stretch, each meeting itself
/// nowhere): where they abut, at the circle they share; and wherever the pipe comes back onto
/// itself between them, along the helices of the start circle's points the screw carries another
/// of its points onto, a turn and more on (`Pipe::returns`), each `on`'s rail there
/// (`Curve::Iso`) — the turns between the two stretches' held clear of the half turn about nought,
/// where only the pipe's own points stand (the circle clear of the axis, `Pipe::spread`). `None`
/// for any other two sheets, two stretches sharing some of the screw's turn (one surface), or where
/// the search gave up.
fn pipes_meeting(on: &std::sync::Arc<super::geom::Blend>,by: &super::geom::Blend,tol: f64) -> Option<Vec<Curve>> {
    let p = on.pipe()?;
    by.pipe()?;
    let Overlap {shared,mine,theirs} = screwed_overlap(on,by,tol)?;
    if shared > 0. { return None }
    p.spread()?;
    let mut out = Vec::new();
    if shared == 0. {
        // abutting: the circle where one ends and the other starts
        let screw = p.radian()?;
        let at = if (mine[1]-theirs[0]).abs() <= (mine[0]-theirs[1]).abs() { mine[1] } else { mine[0] };
        out.push(Curve::Circle(Frame::new(screw.at(p.frame.o,at),screw.dir(p.frame.z,at),screw.dir(p.frame.x,at)),p.r));
    }
    // `t = φ₂ − φ₁`, `φ₁` on `on`, `φ₂` on `by`, outside the half turn either side of nought
    let [lo,hi] = [theirs[0]-mine[1],theirs[1]-mine[0]];
    let (half,tau) = (std::f64::consts::PI,std::f64::consts::TAU);
    let mut feet: Vec<f64> = Vec::new();
    for range in [[lo,hi.min(-half)],[lo.max(half),hi]] {
        if !(range[1] > range[0]) { continue }
        // (one foot may meet the helices of several points: its rail once)
        for [th1,..] in p.returns(range,tol)? {
            if !feet.iter().any(|&f| { let d = (f-th1).rem_euclid(tau); d.min(tau-d)*p.r <= tol }) { feet.push(th1) }
        }
    }
    out.extend(feet.into_iter().map(|th| Curve::Iso(on.clone(),on.u_of_angle(th))));
    Some(out)
}

/// **A ball's pipe meets a plane holding one of its circles** (#780 L1, X5: the plane where one
/// stretch of a pipe entered stretch by stretch ends and the next starts), in closed form: that
/// circle, whole, and wherever else the plane cuts the pipe, its line in the plane of each carried
/// circle meeting the circle, `r n·x_v cos θ + r n·y_v sin θ = h − n·c_v` (`circle_branches`, as a
/// level circle's sheet meets any plane), run into the held circle at its limit. The circle's plane
/// turns with the screw at a fixed lean to its axis, so it is the plane's at most once a turn: where
/// its normal, turned on, is the plane's (either way) and its centre on the plane. `None` for any
/// other sheet, or a plane holding none of the pipe's circles.
fn pipe_plane(sheet: &Surface,blend: &std::sync::Arc<super::geom::Blend>,pl: &Frame,tol: f64) -> Option<Vec<Curve>> {
    use super::geom::Carry;
    let pipe = blend.pipe()?;
    let Carry::Screw {axis,angle,..} = blend.carry else { return None };
    let k = unit(axis);
    let f = pipe.frame;
    let n = unit(pl.z);
    let perp = |x: V| sub(x,scale(k,dot(x,k)));
    // the turns at which the circle's normal is the plane's, either way (both pass for a plane
    // square to the axis), and its centre on it
    let mut coincide: Vec<f64> = Vec::new();
    for s in [1.,-1.] {
        let m = scale(n,s);
        if (dot(m,k)-dot(f.z,k)).abs() > 1e-9 { continue }
        let (zp,mp) = (perp(f.z),perp(m));
        if norm(zp) <= 1e-12 || norm(mp) <= 1e-12 { continue }
        // (`k` parts matched and both unit, the turn about `k` carrying one's square part onto the
        // other's carries one onto the other)
        let phi = dot(k,cross(zp,mp)).datan2(dot(zp,mp));
        let reach = (angle.abs()/std::f64::consts::TAU).ceil() as i64+1;
        for turns in -reach..=reach {
            // (the carry turns `angle` a unit of `v` about its axis, read as unit)
            let v = (phi+std::f64::consts::TAU*turns as f64)/angle;
            if !(-1e-12..=1.+1e-12).contains(&v) { continue }
            let v = v.clamp(0.,1.);
            if (dot(n,blend.carry.at(f.o,v))-dot(n,pl.o)).abs() <= tol && !coincide.iter().any(|&w| (w-v).abs() <= 1e-12) { coincide.push(v) }
        }
    }
    if coincide.is_empty() { return None }
    let mut out: Vec<Curve> = coincide.iter().map(|&v| {
        let c = &blend.carry;
        Curve::Circle(Frame::new(c.at(f.o,v),c.dir(f.z,v),c.dir(f.x,v)),pipe.r)
    }).collect();
    out.extend(plane_branches(sheet,blend,&f,pipe.r,pl,&coincide));
    Some(out)
}

/// **The plane `pl` cut from a circle's sheet under a screw, level by level**: the circle of radius
/// `r` in `f` (its plane `f`'s `x` and `y`) carried to each `v` meets the plane's line in its plane
/// where `r n·x_v cos θ + r n·y_v sin θ = h − n·c_v` (`circle_branches`), run into the `coincide`
/// heights, where the circle lies in the plane, at their limit. A level circle's sheet against any
/// plane, a ball's pipe against one holding one of its circles.
fn plane_branches(sheet: &Surface,blend: &super::geom::Blend,f: &Frame,r: f64,pl: &Frame,coincide: &[f64]) -> Vec<Curve> {
    let (n,carry) = (unit(pl.z),&blend.carry);
    let h = dot(n,pl.o);
    let circle_at = |v: f64| (carry.at(f.o,v),carry.dir(f.x,v),carry.dir(f.y,v));
    let coefficients = |v: f64| { let (c,x,y) = circle_at(v); (r*dot(n,x),r*dot(n,y),h-dot(n,c)) };
    let point = |th: f64,v: f64| { let (c,x,y) = circle_at(v); let (sn,cs) = th.dsin_cos(); add(c,add(scale(x,r*cs),scale(y,r*sn))) };
    circle_branches(&coefficients,coincide,&point,&|pts,closed| Curve::Traced(std::sync::Arc::new(super::geom::Traced {a:sheet.clone(),
        b:Surface::Plane(*pl),pts,closed})))
}

/// **A ball touching the pipe its great circle screws into, round that circle** (#721 F2c5: a ball
/// under a screw touches round its great circle square to its centre's velocity, which the screw
/// carries into the helical pipe about its centre's path; the ball at either end pose touches the
/// pipe round the circle there). The blend of a circle and itself carried by a screw, and a ball of
/// its radius about the circle's centre carried to the blend's start or end: that circle, or none.
fn pipe_end(blend: &super::geom::Blend,centre: V,r: f64,tol: f64) -> Option<Curve> {
    let super::geom::Carry::Screw {..} = blend.carry else { return None };
    let (Curve::Circle(f,rad),Curve::Circle(..)) = (&blend.a,&blend.b) else { return None };
    if (rad.abs()-r).abs() > tol { return None }
    let same = |q: V,p: V| norm(sub(q,p)) <= 1e-9*(1.+norm(p));
    let span = blend.ta[1]-blend.ta[0];
    if !(0..3).all(|i| same(blend.b.point(blend.tb[0]+(blend.tb[1]-blend.tb[0])*i as f64/2.),blend.a.point(blend.ta[0]+span*i as f64/2.))) { return None }
    [0.,1.].into_iter().find(|&v| norm(sub(blend.carry.at(f.o,v),centre)) <= tol).map(|v|
        Curve::Circle(Frame::new(blend.carry.at(f.o,v),blend.carry.dir(f.z,v),blend.carry.dir(f.x,v)),r))
}

/// **Where the sheet a level circle screws into meets a surface the screw's axis runs along**, in
/// closed form (a round tool's rim under a screw, #721 F2c5): the blend of a circle square to the
/// screw's axis and itself carried by the screw, `S(u, v) = carry_v(c + r (cos θ x + sin θ y))`. At
/// each `v` the circle is level, so it meets
/// - a plane square to the axis only at the `v` that lifts it there, whole: that circle;
/// - a cylinder about the screw's axis where its points stand at the cylinder's radius, the same at
///   every `v` (the screw keeps the distance): `A cos θ + B sin θ = C`, helices (`Curve::Iso`), one
///   where the circle touches it;
/// - a cylinder about another line along the axis (the tool's own side at an end pose), or a cone
///   about any line along it (a frustum's side, its radius `R(v)` the cone's at the height the
///   circle has climbed to, #780 C2), where the two circles of that level meet,
///   `A(v) cos θ + B(v) sin θ = C(v)`, `A, B = 2 r d·x_v, 2 r d·y_v`,
///   `C = R² − |d|² − r²`, `d` the offset of their centres: two branches `θ±(v)` over the `v` where
///   `|C| ≤ √(A² + B²)`, joining where they turn (the circles touching) and ending at the `v` where
///   the circle is the cylinder's own section (the whole circle on both), at its points along the
///   centre's way there. Its points are written in closed form and handed over as a traced meeting
///   (`Traced`, its chords pulled onto both), so no near touch is traced.
/// `None` for any other blend or surface: traced.
fn screwed_circle_meeting(sheet: &Surface,blend: &std::sync::Arc<super::geom::Blend>,other: &Surface,tol: f64) -> Option<Vec<Curve>> {
    use super::geom::{Carry,Traced};
    let (center,k,angle,rise,f,r) = screwed_circle(blend)?;
    let perp = |x: V| sub(x,scale(k,dot(x,k)));
    let circle_at = |v: f64| (blend.carry.at(f.o,v),blend.carry.dir(f.x,v),blend.carry.dir(f.y,v));
    let point = |th: f64,v: f64| { let (c,x,y) = circle_at(v); let (sn,cs) = th.dsin_cos(); add(c,add(scale(x,r*cs),scale(y,r*sn))) };
    // the blend's `u` of the circle's angle `θ`
    let u_of = |th: f64| blend.u_of_angle(th);
    match *other {
        Surface::Blend(_,ref other_blend) => {
            // another circle the same screw carries: where this circle's points lie on it, the other's
            // circle screwed to this one's height, each such point's helix
            // (two stretches of one circle's sheet standing apart along the screw meet nowhere)
            if screwed_overlap(blend,other_blend,tol).is_some_and(|o| o.shared <= 0.) { return Some(vec![]) }
            let (c2,k2,a2,r2,f2,rad2) = screwed_circle(other_blend)?;
            let off = sub(c2,center);
            if norm(cross(k,k2)) > 1e-12 || norm(sub(off,scale(k,dot(off,k)))) > tol || rise == 0. || r2 == 0.
                || (rise/angle-r2/a2).abs() > 1e-12*(rise/angle).abs() { return None }
            let pitch = rise/angle;
            let turn = dot(sub(f.o,f2.o),k)/pitch;
            let screw = Carry::Screw {center,axis:k,angle:turn,advance:pitch*turn};
            let o2 = screw.at(f2.o,1.);
            let d = perp(sub(f.o,o2));
            // |d + r (cos θ x + sin θ y)| = rad2: `2 r d·x cos θ + 2 r d·y sin θ = rad2² − |d|² − r²`
            let (a,b,c) = (2.*r*dot(d,f.x),2.*r*dot(d,f.y),rad2*rad2-dot(d,d)-r*r);
            let rho = a.hypot(b);
            if rho <= tol*r { return None }
            let ratio = c/rho;
            let phi = b.datan2(a);
            if (ratio.abs()-1.).abs()*r <= tol { return Some(vec![Curve::Iso(blend.clone(),u_of(phi+if ratio < 0. { std::f64::consts::PI } else { 0. }))]) }
            if ratio.abs() > 1. { return Some(vec![]) }
            let w = ratio.dacos();
            Some([phi+w,phi-w].iter().map(|&th| Curve::Iso(blend.clone(),u_of(th))).collect())
        }
        Surface::Plane(p) if norm(cross(p.z,k)) <= 1e-12 => {
            if rise == 0. { return Some(vec![]) }
            let v = dot(sub(p.o,f.o),k)/rise;
            let slack = tol/rise.abs();
            if !(-slack..=1.+slack).contains(&v) { return Some(vec![]) }
            let (c,x,_) = circle_at(v.clamp(0.,1.));
            let c = add(c,scale(k,dot(sub(p.o,c),k)));
            Some(vec![Curve::Circle(Frame::new(c,f.z,x),r)])
        }
        Surface::Cylinder(cy,_) | Surface::Cone(cy,..) if norm(cross(cy.z,k)) <= 1e-12 => {
            // the surface's radius at the height of `c` along its axis: a cylinder's the same at
            // every height, a cone's `r + z tan α` (#780 C2: a cone's own rim, screwed)
            let (r0,slope,cone) = match *other {
                Surface::Cone(_,r0,a) => (r0,a.dtan(),true),
                Surface::Cylinder(_,big) => (big.abs(),0.,false),
                _ => return None,
            };
            let radius = |c: V| r0+dot(sub(c,cy.o),cy.z)*slope;
            // the circle's own frame and its offset from the surface's axis at `v`, and the
            // coefficients there
            let coefficients = |v: f64| {
                let (c,x,y) = circle_at(v);
                let (d,big) = (perp(sub(c,cy.o)),radius(c));
                (2.*r*dot(d,x),2.*r*dot(d,y),big*big-dot(d,d)-r*r)
            };
            if !cone && norm(perp(sub(cy.o,center))) <= tol {
                // coaxial: the same at every `v`, helices
                let (a,b,c) = coefficients(0.);
                let rho = a.hypot(b);
                if rho <= 1e-300 { return Some(vec![]) }
                let ratio = c/rho;
                let phi = b.datan2(a);
                if (ratio.abs()-1.).abs()*r <= tol { return Some(vec![Curve::Iso(blend.clone(),u_of(phi+if ratio < 0. { std::f64::consts::PI } else { 0. }))]) }
                if ratio.abs() > 1. { return Some(vec![]) }
                let w = ratio.dacos();
                return Some([phi+w,phi-w].iter().map(|&th| Curve::Iso(blend.clone(),u_of(th))).collect())
            }
            // another line along the axis: where the circle is the surface's own section, whole
            let mut out = Vec::new();
            let (q,p0) = (perp(sub(cy.o,center)),perp(sub(f.o,center)));
            let mut coincide: Vec<f64> = Vec::new();
            if (norm(q)-norm(p0)).abs() <= tol && norm(p0) > tol {
                let turn = dot(k,cross(p0,q)).datan2(dot(p0,q));
                for n in -2..=2 {
                    let v = (turn+std::f64::consts::TAU*n as f64)/angle;
                    if (-1e-12..=1.+1e-12).contains(&v) && (radius(circle_at(v.clamp(0.,1.)).0)-r).abs() <= tol {
                        let v = v.clamp(0.,1.);
                        coincide.push(v);
                        let (c,x,_) = circle_at(v);
                        out.push(Curve::Circle(Frame::new(c,f.z,x),r));
                    }
                }
            }
            out.extend(circle_branches(&coefficients,&coincide,&point,&|pts,closed| Curve::Traced(
                std::sync::Arc::new(Traced {a:sheet.clone(),b:other.clone(),pts,closed}))));
            Some(out)
        }
        // any other plane: its line in each level meets the level circle
        Surface::Plane(p) => Some(plane_branches(sheet,blend,&f,r,&p,&[])),
        _ => None,
    }
}

/// **The branches of a level circle's meeting under a screw**, `A(v) cos θ + B(v) sin θ = C(v)` over
/// the sheet's `v` (`coefficients`), through the circle's exact points `point(θ, v)` at
/// `θ = atan2(B, A) ± acos(C/ρ)` (`branches`): each between two where either root turns by more than
/// `π/1024`, and each end at one of the `coincide` heights, where the circle is wholly the other
/// surface's, at its limit there: the roots of the coefficients' rates, read across it (`(A, B, C)`
/// grow from nothing there, so a one-sided reading leans by the hair) — on a cylinder the circle's
/// points square to the centre's way, where `C` grows as the square of the way; on a cone, whose
/// radius runs with the height, turned off them to where its normal is square to the screw's
/// velocity (#780 C2: the feet of its contact rulings, where the sheet touches it).
fn circle_branches(coefficients: &dyn Fn(f64) -> (f64,f64,f64),coincide: &[f64],point: &dyn Fn(f64,f64) -> V,
    traced: &dyn Fn(Vec<V>,bool) -> Curve) -> Vec<Curve> {
    const STEP: f64 = std::f64::consts::PI/1024.;
    let hair = 1e-6;
    let angles = |(a,b,c): (f64,f64,f64)| { let (phi,w) = (b.datan2(a),(c/a.hypot(b).max(1e-300)).clamp(-1.,1.).dacos()); [phi+w,phi-w] };
    let roots = |v: f64| { let (a,b,c) = coefficients(v); (a.hypot(b) > 0.).then(|| angles((a,b,c))) };
    let real = |v: f64| { let (a,b,c) = coefficients(v); let rho = a.hypot(b); rho > 0. && c.abs() <= rho };
    let limit = |w: f64,side: f64| {
        let ((a1,b1,c1),(a0,b0,c0)) = (coefficients(w+hair),coefficients(w-hair));
        Some(angles((side*(a1-a0),side*(b1-b0),side*(c1-c0))))
    };
    let apart = |x: f64,y: f64,_: f64,_: f64| { let d = (y-x).rem_euclid(std::f64::consts::TAU); d.min(std::f64::consts::TAU-d) > STEP };
    branches(&Branches {roots:&roots,real:&real,point:&|th,v| Some(point(th,v)),apart:&apart,coincide,limit:&limit,hair},traced)
}

/// What a meeting read as two roots along a sheet's `v` is (`branches`): `roots(v)`, the two (each
/// branch at its own index), read where `real(v)` says they are (and clamped a rounding past);
/// `point(x, v)`, the meeting's point at root `x`, or
/// none past the sheet's reach; `apart(x, y, v, w)`, whether one branch's points at two samples
/// stand a step apart; the `coincide` heights, where the meeting is a whole curve of both (a circle
/// one surface's section, a ruling the other holds), kept clear of by `hair`; and `limit(w, side)`,
/// the roots a branch ends at there, approached from `side`.
struct Branches<'a> {
    roots: &'a dyn Fn(f64) -> Option<[f64;2]>,
    real: &'a dyn Fn(f64) -> bool,
    point: &'a dyn Fn(f64,f64) -> Option<V>,
    apart: &'a dyn Fn(f64,f64,f64,f64) -> bool,
    coincide: &'a [f64],
    limit: &'a dyn Fn(f64,f64) -> Option<[f64;2]>,
    hair: f64,
}

/// **A meeting's branches over a sheet's `v`** (a level circle's under a screw, a screwed line's
/// against a cone), as traced curves (`traced`) through their exact points: runs of `v` over which
/// the two roots stand, each end a turn where they join (refined by bisection), the sheet's edge,
/// or a coincidence — kept clear of it by a hair, the samples drawn in ever closer to it, and ended
/// at its limit; points added between two where either branch's stand a step apart; a branch cut
/// where its point leaves the sheet's reach.
fn branches(m: &Branches,traced: &dyn Fn(Vec<V>,bool) -> Curve) -> Vec<Curve> {
    const N: usize = 2048;
    // how many halvings deep the points are drawn in toward a turn or an end where the roots meet,
    // where they part as a square root: a chord there short enough that its sag, quadratic in it,
    // stands far below the roots' parting, linear in it
    const DEEP: usize = 32;
    let Branches {roots,real:defined,point,apart,coincide,limit,hair} = *m;
    let mut out = Vec::new();
    let near_coincidence = |v: f64| coincide.iter().any(|&w| (w-v).abs() <= hair);
    let vs: Vec<f64> = (0..=N).map(|i| i as f64/N as f64).collect();
    let ok: Vec<bool> = vs.iter().map(|&v| !near_coincidence(v) && defined(v)).collect();
    let refine = |inside: f64,outside: f64| -> f64 {
        let (mut a,mut b) = (inside,outside);
        for _ in 0..60 { let m = 0.5*(a+b); if defined(m) { a = m } else { b = m } }
        a
    };
    // a branch's end at a coincidence: its limit there
    let end_at = |v: f64,k: usize| -> Option<V> {
        let w = *coincide.iter().find(|&&w| (w-v).abs() <= 2.*hair)?;
        let side = if v < w { -1. } else { 1. };
        point(limit(w,side)?[k],w)
    };
    let mut i = 0;
    while i <= N {
        if !ok[i] { i += 1; continue }
        let start = i;
        while i < N && ok[i+1] { i += 1 }
        let end = i;
        i += 1;
        let lo_v = if start > 0 && !near_coincidence(vs[start-1]) { Some(refine(vs[start],vs[start-1])) } else { None };
        let hi_v = if end < N && !near_coincidence(vs[end+1]) { Some(refine(vs[end],vs[end+1])) } else { None };
        let coincident = |i: usize| coincide.iter().copied().find(|&w| (w-vs[i]).abs() <= hair+1./N as f64);
        let lo_w = if start > 0 && near_coincidence(vs[start-1]) { coincident(start-1) } else { None };
        let hi_w = if end < N && near_coincidence(vs[end+1]) { coincident(end+1) } else { None };
        let v0 = lo_v.or(lo_w.map(|w| w+hair)).unwrap_or(vs[start]);
        let v1 = hi_v.or(hi_w.map(|w| w-hair)).unwrap_or(vs[end]);
        // the stretch's samples, ends included
        let mut stretch: Vec<f64> = vec![v0];
        if let Some(w) = lo_w { stretch.extend((1..24).map(|j| w+(vs[start]-w)*0.5f64.powi(24-j)).filter(|&v| v > v0 && v < vs[start])) }
        stretch.extend(vs[start..=end].iter().copied().filter(|&v| v > v0 && v < v1));
        if let Some(w) = hi_w { stretch.extend((1..24).rev().map(|j| w-(w-vs[end])*0.5f64.powi(24-j)).filter(|&v| v > vs[end] && v < v1)) }
        // and beside a turn, where the roots part as the square root of the way from it
        if let Some(t) = lo_v { stretch.extend((1..DEEP).map(|j| t+(vs[start]-t)*0.5f64.powi((DEEP-j) as i32)).filter(|&v| v > v0 && v < vs[start])) }
        if let Some(t) = hi_v { stretch.extend((1..DEEP).rev().map(|j| t-(t-vs[end])*0.5f64.powi((DEEP-j) as i32)).filter(|&v| v > vs[end] && v < v1)) }
        // and toward the sheet's own edge, where the roots may meet as at a turn (two tori's crossing
        // circles touching at the top of their tubes)
        if lo_v.is_none() && lo_w.is_none() && start < end { stretch.extend((1..DEEP).map(|j| v0+(vs[start+1]-v0)*0.5f64.powi((DEEP-j) as i32)).filter(|&v| v > v0 && v < vs[start+1])) }
        if hi_v.is_none() && hi_w.is_none() && start < end { stretch.extend((1..DEEP).rev().map(|j| v1-(v1-vs[end-1])*0.5f64.powi((DEEP-j) as i32)).filter(|&v| v > vs[end-1] && v < v1)) }
        stretch.sort_by(f64::total_cmp);
        stretch.dedup();
        stretch.push(v1);
        // and between two where either branch's points stand a step apart (a chord a traced curve's
        // integrals read as one stretch)
        let turned = |a: f64,b: f64| match (roots(a),roots(b)) { (Some(x),Some(y)) => (0..2).any(|k| apart(x[k],y[k],a,b)),_ => false };
        let mut fine: Vec<f64> = vec![stretch[0]];
        for w in stretch.windows(2) {
            let mut todo = vec![(w[0],w[1],0)];
            while let Some((a,b,depth)) = todo.pop() {
                if depth < 16 && turned(a,b) { let m = 0.5*(a+b); todo.push((m,b,depth+1)); todo.push((a,m,depth+1)); }
                else { fine.push(b) }
            }
        }
        let stretch = fine;
        // each branch's points, `None` where it stands past the sheet's reach
        let branch_pts = |k: usize| -> Vec<Option<V>> {
            let mut pts: Vec<Option<V>> = stretch.iter().filter_map(|&v| roots(v).map(|r| point(r[k],v))).collect();
            if let Some(p) = end_at(v0,k) { pts.insert(0,Some(p)) }
            if let Some(p) = end_at(v1,k) { pts.push(Some(p)) }
            pts
        };
        let (plus,minus) = (branch_pts(0),branch_pts(1));
        // turning at an end, the two are one curve through the turn; at both, one loop
        let curves: Vec<Vec<Option<V>>> = match (lo_v.is_some(),hi_v.is_some()) {
            (true,false) => { let mut pts: Vec<Option<V>> = minus.iter().rev().copied().collect(); pts.extend(plus.iter().skip(1)); vec![pts] }
            (_,true) => { let mut pts = plus; pts.extend(minus.iter().rev().skip(1)); vec![pts] }
            (false,false) => vec![plus,minus],
        };
        let closed = lo_v.is_some() && hi_v.is_some();
        for pts in curves {
            let whole = pts.iter().all(Option::is_some);
            for run in pts.split(Option::is_none) {
                let run: Vec<V> = run.iter().flatten().copied().collect();
                if run.len() >= 2 { out.push(traced(run,closed && whole)) }
            }
        }
    }
    out
}

/// A central conic's arc over `span` of its parameter as one rational quadratic B-spline of `n`
/// equal stretches: each its ends on the curve (`at`) and its middle pole the curve's point at its
/// middle pushed out from the `centre` by `1 / w(h)`, weighed `w(h)` (`h` the stretch's half; an
/// ellipse's `cos h`, a hyperbola's `cosh h`), exact.
fn conic_arc(centre: V,[a,b]: [f64;2],n: usize,at: &dyn Fn(f64) -> V,w: &dyn Fn(f64) -> f64) -> Option<Curve> {
    let (mut poles,mut weights,mut knots) = (vec![at(a)],vec![1.],vec![0.;3]);
    for k in 0..n {
        let (s0,s1) = (a+(b-a)*k as f64/n as f64,a+(b-a)*(k+1) as f64/n as f64);
        let wk = w(0.5*(s1-s0));
        poles.extend([add(centre,scale(sub(at(0.5*(s0+s1)),centre),1./wk)),at(s1)]);
        weights.extend([wk,1.]);
        knots.extend(vec![(k+1) as f64;if k+1 == n { 3 } else { 2 }]);
    }
    Some(Curve::BSpline(std::sync::Arc::new(super::nurbs::BSpline::rational(2,knots,poles,weights).ok()?)))
}

/// **A plane and a quadric of revolution, met in closed form**: a surface of revolution whose
/// meridian is a conic symmetric about the axis, `ρ² + β z² + γ z + δ = 0` in its frame (the
/// hyperboloid a line turns into, `build`'s and the swept solid's, #721 X6), is a quadric, which a
/// plane meets in a conic: in the plane's coordinates `Q(u, v) = 0`, quadratic, read about its
/// centre on its principal axes as `λ₁ s² + λ₂ t² + F = 0`. An ellipse where `λ₁ λ₂ > 0` (its arcs
/// within the meridian's heights, where it runs past them); where
/// `λ₁ λ₂ < 0` a hyperbola, or its two asymptotes where it stands within `tol` of them (a plane
/// holding a ruling, as a face holding the edge the hyperboloid was turned from does), each branch
/// held to the heights the meridian spans; arcs written exactly as rational quadratics (a stretch
/// `[τa, τb]` of `(a cosh τ, b sinh τ)` its ends and `(a cosh τm, b sinh τm) / cosh δ` weighed
/// `cosh δ`, `δ` its half). `None` — traced — where the meridian is no such conic, or the plane
/// meets it in a parabola or two parallel lines.
fn plane_quadric(pl: &Frame,rf: &Frame,c: &Curve,tol: f64) -> Option<Vec<Curve>> {
    let Curve::BSpline(m) = c else { return None };
    let cosh = |x: f64| { let e = x.dexp(); 0.5*(e+1./e) };
    let sinh = |x: f64| { let e = x.dexp(); 0.5*(e-1./e) };
    let [t0,t1] = m.domain();
    let local = |t: f64| { let l = rf.local(m.point(t)); [l[0],l[2]] };
    // β, γ, δ through three of the meridian's points, held at more
    let three: Vec<[f64;2]> = [0.1,0.5,0.9].iter().map(|&f| local(t0+(t1-t0)*f)).collect();
    let [beta,gamma,delta] = solve3([0,1,2].map(|i| [three[i][1]*three[i][1],three[i][1],1.]),[0,1,2].map(|i| -three[i][0]*three[i][0]));
    if !(beta.is_finite() && gamma.is_finite() && delta.is_finite()) { return None }
    let samples: Vec<[f64;2]> = (0..=16).map(|k| local(t0+(t1-t0)*k as f64/16.)).collect();
    let size = samples.iter().fold(1_f64,|r,p| r.max(p[0].abs()).max(p[1].abs()));
    // how far the quadric misses the meridian's own points, in its own units: what any value of it
    // is known to
    let mut misfit = 0_f64;
    for &[r,z] in &samples {
        let g = r*r+beta*z*z+gamma*z+delta;
        let grad = (4.*r*r+(2.*beta*z+gamma).powi(2)).sqrt();
        if !(g.abs() <= 1e-9*size*grad.max(1e-300)) { return None }
        misfit = misfit.max(g.abs());
    }
    let (zlo,zhi) = samples.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),p| (a.min(p[1]),b.max(p[1])));
    let (zlo,zhi) = (zlo-tol,zhi+tol);
    // the plane in the quadric's frame, and Q's coefficients over it
    let (x0,a,b) = (rf.local(pl.o),rf.dir_local(pl.x),rf.dir_local(pl.y));
    let mm = |v: V| [v[0],v[1],beta*v[2]];
    let g: V = add(mm(x0),[0.,0.,0.5*gamma]);
    let (cuu,cuv,cvv) = (dot(a,mm(a)),dot(a,mm(b)),dot(b,mm(b)));
    let (cu,cv,c0) = (dot(a,g),dot(b,g),dot(x0,mm(x0))+gamma*x0[2]+delta);
    let det = cuu*cvv-cuv*cuv;
    let theta = 0.5*(2.*cuv).datan2(cuu-cvv);
    let (sn,cs) = theta.dsin_cos();
    let mut l1 = cuu*cs*cs+2.*cuv*sn*cs+cvv*sn*sn;
    let mut l2 = cuu*sn*sn-2.*cuv*sn*cs+cvv*cs*cs;
    let scale_of = l1.abs().max(l2.abs());
    if !(l1.abs().min(l2.abs()) > 1e-12*scale_of) || !(det.abs() > 0.) { return None }
    // the centre and the constant there
    let w = [-(cvv*cu-cuv*cv)/det,-(cuu*cv-cuv*cu)/det];
    let f = c0+cu*w[0]+cv*w[1];
    let centre = pl.at([w[0],w[1],0.]);
    let (mut e1,mut e2) = (add(scale(pl.x,cs),scale(pl.y,sn)),add(scale(pl.x,-sn),scale(pl.y,cs)));
    if l1*l2 > 0. {
        // an ellipse, a point or nothing
        if f*l1 >= 0. { return Some(vec![]) }
        let (ea,eb) = ((-f/l1).sqrt(),(-f/l2).sqrt());
        let ellipse = Frame::new(centre,pl.z,e1);
        // its height `h₀ + P cos φ + Q sin φ`: whole where that stays within the meridian's, else
        // its arcs within them, each a rational quadratic a quarter turn at most
        let h0 = rf.local(centre)[2];
        let (p,q) = (ea*dot(rf.z,ellipse.x),eb*dot(rf.z,ellipse.y));
        let amp = p.dhypot(q);
        if h0-amp >= zlo && h0+amp <= zhi { return Some(vec![Curve::Ellipse(ellipse,ea,eb)]) }
        if h0+amp < zlo || h0-amp > zhi { return Some(vec![]) }
        let phase = q.datan2(p);
        let mut cuts: Vec<f64> = Vec::new();
        for zb in [zlo,zhi] {
            let c = (zb-h0)/amp;
            if c.abs() < 1. { let d = c.dacos(); cuts.extend([phase-d,phase+d]) }
        }
        let tau = std::f64::consts::TAU;
        let mut cuts: Vec<f64> = cuts.into_iter().map(|x| x.rem_euclid(tau)).collect();
        cuts.sort_by(f64::total_cmp);
        let at = |phi: f64| ellipse.at([ea*phi.dcos(),eb*phi.dsin(),0.]);
        let mut out = Vec::new();
        for i in 0..cuts.len() {
            let (a0,a1) = (cuts[i],if i+1 < cuts.len() { cuts[i+1] } else { cuts[0]+tau });
            let mid = 0.5*(a0+a1);
            let hm = h0+p*mid.dcos()+q*mid.dsin();
            if !(a1-a0 > 1e-12) || hm < zlo || hm > zhi { continue }
            let n = ((a1-a0)/std::f64::consts::FRAC_PI_2).ceil().max(1.) as usize;
            out.push(conic_arc(centre,[a0,a1],n,&at,&|h: f64| h.dcos())?);
        }
        return Some(out)
    }
    // the hyperbola's branches open along `e1`: `−F/λ₁ > 0`
    if -f/l1 < 0. { std::mem::swap(&mut l1,&mut l2); std::mem::swap(&mut e1,&mut e2) }
    // a plane through the cone of the quadric's asymptotes — tangent to a hyperboloid, holding its
    // rulings — is so where its constant at the centre is nothing but rounding: the quadric's own
    // misfit, or the cancellation of the terms it sums (a plane holding a ruling exactly leaves
    // `F` some 1e-14, whose square root, the hyperbola's semi-axis, is some 1e-7: no measure of
    // the geometry, and no ground to call the branches apart at a tolerance finer than that, #735)
    let rounding = (4.*misfit).max(1e3*f64::EPSILON*(c0.abs()+(cu*w[0]).abs()+(cv*w[1]).abs()));
    if f.abs() <= rounding || (f.abs()/l1.abs().min(l2.abs())).sqrt() <= tol {
        let r = (-l1/l2).sqrt();
        return Some([1.,-1.].map(|k| Curve::Line {p:centre,d:unit(add(e1,scale(e2,k*r)))}).to_vec())
    }
    let (ha,hb) = ((-f/l1).sqrt(),(f/l2).sqrt());
    let height = |p: V| rf.local(p)[2];
    let h0 = height(centre);
    let mut out = Vec::new();
    for sigma in [1.,-1.] {
        let at = |tau: f64| add(centre,add(scale(e1,sigma*ha*cosh(tau)),scale(e2,hb*sinh(tau))));
        // the height along the branch, `h₀ + P cosh τ + Q sinh τ`; where it is each bound, as roots
        // in `e^τ` of a quadratic
        let (p,q) = (sigma*ha*dot(rf.z,e1),hb*dot(rf.z,e2));
        let mut taus: Vec<f64> = Vec::new();
        for zb in [zlo,zhi] {
            let roots = crate::util::roots::quadratic(0.5*(p+q),h0-zb,0.5*(p-q));
            taus.extend(roots.into_iter().filter(|x| *x > 0. && x.is_finite()).map(|x| x.dln()));
        }
        taus.sort_by(f64::total_cmp);
        let inside = |tau: f64| { let h = height(at(tau)); h >= zlo && h <= zhi };
        // a branch running on within the heights for ever is no stretch: traced
        if inside(taus.first().map_or(0.,|t| t-1.)) || inside(taus.last().map_or(0.,|t| t+1.)) { return None }
        for wnd in taus.windows(2) {
            let (ta,tb) = (wnd[0],wnd[1]);
            if !(tb-ta > 1e-12) || !inside(0.5*(ta+tb)) { continue }
            // stretches short enough that each weight stays modest
            let n = ((tb-ta)/2.).ceil().max(1.) as usize;
            out.push(conic_arc(centre,[ta,tb],n,&at,&cosh)?);
        }
    }
    Some(out)
}

/// A plane `n·x = h` and a cylinder: lines where the plane runs along the axis (one where it
/// touches), else an ellipse about where the axis pierces it, its minor axis the radius square to
/// both.
fn plane_cylinder(n: V,h: f64,c: &Frame,r: f64,tol: f64) -> Vec<Curve> {
    let cos = dot(n,c.z);
    if cos.abs() <= 1e-12 {
        let dist = h-dot(n,c.o);
        let foot = add(c.o,scale(n,dist));
        let w = unit(cross(c.z,n));
        // (touching: by the gap, as for two cylinders)
        return match plane2::chord(dist,r,plane2::Touch::Gap(tol)) {
            plane2::Chord::Miss => vec![],
            plane2::Chord::Touch => vec![Curve::Line {p:foot,d:c.z}],
            plane2::Chord::Cross(half) =>
                vec![Curve::Line {p:add(foot,scale(w,half)),d:c.z},Curve::Line {p:sub(foot,scale(w,half)),d:c.z}],
        }
    }
    let centre = add(c.o,scale(c.z,(h-dot(n,c.o))/cos));
    let minor = cross(c.z,n);
    if norm(minor) <= 1e-12 { return vec![Curve::Circle(Frame::new(centre,c.z,c.x),r)] }
    let minor = unit(minor);
    let major = cross(minor,n);
    vec![Curve::Ellipse(Frame::new(centre,cross(major,minor),major),r/cos.abs(),r)]
}

/// **An affine image meets a surface where its preimage meets the surface carried back** (an
/// affine map keeps incidence): a plane's meeting with a sphere's image the image of a circle, an
/// ellipse; any surface's with an image of the same map the image of the two preimages' meeting.
/// The preimages' closed forms are found at the tolerance shrunk by the map's greatest stretch, so
/// a gap they read as a touch is no wider in space, and each curve is carried back
/// (`Curve::mapped`). Where the other, carried back, is still an image (of another map), the two
/// are one where their quadrics are (`affine::Quadric::same`) and traced otherwise. `None` where
/// neither is an image; `Same` or `Traced` alone where only whether they are one is asked (`meet`
/// false, `same`).
fn pulled_back(a: &Surface,b: &Surface,tol: f64,meet: bool) -> Option<Ssi> {
    let (m,other) = match (a,b) { (Surface::Affine(m),t) | (t,Surface::Affine(m)) => (m,t),_ => return None };
    let back = other.mapped(&m.back).0;
    if matches!(back,Surface::Affine(_)) {
        let one = matches!(other,Surface::Affine(n) if matches!((m.quadric,n.quadric),(Some(p),Some(q)) if p.same(&q,tol)));
        return Some(if one { Ssi::Same } else { Ssi::Traced })
    }
    let tol = tol/m.stretch[1];
    if !meet { return Some(if same(&m.of,&back,tol) { Ssi::Same } else { Ssi::Traced }) }
    Some(match intersect(&m.of,&back,tol) {
        Ssi::Curves(cs) => Ssi::Curves(cs.iter().map(|c| c.mapped(&m.map).0).collect()),
        met => met,
    })
}

/// An axis both surfaces may turn about: a surface of revolution's own (a sphere lends the other's),
/// or a plane's normal through the other's centre.
fn axis_of(s: &Surface,t: &Surface) -> Option<(V,V)> {
    match (s,t) {
        (Surface::Plane(_),_) | (Surface::Sphere(..),_) | (Surface::Affine(..),_) => None,
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
pub fn solve3(m: [V;3],rhs: V) -> V {
    let det = dot(m[0],cross(m[1],m[2]));
    let col = |k: usize| -> f64 {
        let mut a = m;
        for i in 0..3 { a[i][k] = rhs[i]; }
        dot(a[0],cross(a[1],a[2]))
    };
    [col(0)/det,col(1)/det,col(2)/det]
}

/// How far `p` stands off two surfaces at once: the larger of their signed distances' sizes.
fn off_both(a: &Surface,b: &Surface,p: V) -> f64 { a.implicit(p).abs().max(b.implicit(p).abs()) }

/// The sine of the angle two surfaces meet at, below which a trace is ill-conditioned (a point
/// within `tol` of both may be `tol / sin θ` off their meeting, and where they touch their meeting
/// has a singular point): refused, not traced (one degree).
pub(crate) const SHALLOW: f64 = 0.0175;

/// **Where two surfaces touch at a point**: their meeting there, to second order. Over the plane
/// they share at `p`, `b`'s signed distance along `a` grows as `½ tᵀ Q t`, `Q` the difference of
/// their second fundamental forms (`a`'s turned by the sign of their normals' agreement): a
/// definite `Q` touches at `p` alone, an indefinite one meets in two branches crossing there along
/// its null directions. A touch: those directions (each either way; none for a definite `Q`), and
/// the radius within which the two stand nearer parallel than a trace can follow (`SHALLOW`).
/// `None` where `Q` is too near singular to say (the two touch along a curve, or closer than
/// second order).
#[derive(Clone,Debug)]
pub(crate) struct Touch { pub at: V,pub branches: Vec<V>,pub radius: f64 }

/// The step two surfaces' derivatives are differenced over, near their finer feature.
fn difference_step(a: &Surface,b: &Surface) -> f64 { 1e-4*a.feature().min(b.feature()).min(1e6).max(1e-9) }

pub(crate) fn touch(a: &Surface,b: &Surface,p: V) -> Option<Touch> {
    let (na,nb) = (unit(a.gradient(p)),unit(b.gradient(p)));
    let agree = if dot(na,nb) < 0. { -1. } else { 1. };
    let f = Frame::about(p,na);
    let (e1,e2) = (f.x,f.y);
    // each surface's second derivatives along the shared plane, by its exact gradient either side
    let h = difference_step(a,b);
    let hessian = |s: &Surface,t: V| scale(sub(s.gradient(add(p,scale(t,h))),s.gradient(sub(p,scale(t,h)))),0.5/h);
    let (ha1,ha2,hb1,hb2) = (hessian(a,e1),hessian(a,e2),hessian(b,e1),hessian(b,e2));
    let q11 = dot(e1,hb1)-agree*dot(e1,ha1);
    let q22 = dot(e2,hb2)-agree*dot(e2,ha2);
    let q12 = 0.5*(dot(e1,hb2)-agree*dot(e1,ha2)+dot(e2,hb1)-agree*dot(e2,ha1));
    let (mean,dev) = (0.5*(q11+q22),(0.25*(q11-q22)*(q11-q22)+q12*q12).sqrt());
    let (big,least) = (mean.abs()+dev,(mean.abs()-dev).abs());
    if !(least > 1e-6*big && big > 0.) { return None }
    let radius = 2.*SHALLOW/least;
    // Q(ψ) = mean + B cos 2ψ + C sin 2ψ (B, C its deviation's parts), zero where they cancel
    let branches = if q11*q22-q12*q12 > 0. { Vec::new() } else {
        let phase = (2.*q12).datan2(q11-q22);
        let off = (-mean/dev).clamp(-1.,1.).dacos();
        [phase+off,phase-off].into_iter().flat_map(|w| {
            let (sn,cs) = (0.5*w).dsin_cos();
            let t = add(scale(e1,cs),scale(e2,sn));
            [t,scale(t,-1.)]
        }).collect()
    };
    Some(Touch {at:p,branches,radius})
}

/// **The point near `p` where `a` and `b` touch**: on `a`, where `b`'s signed distance along it is
/// stationary — their normals parallel there — by Newton over `a`'s tangent plane, its
/// derivatives by differences; `None` where that does not settle, or settles off `b` (the two pass
/// apart or cross there, and do not touch). Settled is a step a thousandth of `tol`, or one within
/// a tenth of it that no longer shrinks: the differences' rounding moves the point by as much (two
/// vaults' crown at a `tol` of 1e-9 cycled through steps of 1.3e-11).
pub(crate) fn touch_near(a: &Surface,b: &Surface,p: V,tol: f64) -> Option<V> {
    let on_a = |q: V| a.point(a.inverse(q));
    let h = difference_step(a,b);
    let mut x = on_a(p);
    let mut last = f64::INFINITY;
    for _ in 0..32 {
        let f = Frame::about(x,a.gradient(x));
        let g = |s: f64,t: f64| b.implicit(on_a(add(x,add(scale(f.x,s),scale(f.y,t)))));
        let (g0,[sp,sm,tp,tm]) = (g(0.,0.),[g(h,0.),g(-h,0.),g(0.,h),g(0.,-h)]);
        let (gs,gt) = ((sp-sm)/(2.*h),(tp-tm)/(2.*h));
        let (gss,gtt) = ((sp-2.*g0+sm)/(h*h),(tp-2.*g0+tm)/(h*h));
        let gst = (g(h,h)-g(h,-h)-g(-h,h)+g(-h,-h))/(4.*h*h);
        let det = gss*gtt-gst*gst;
        if !(det.abs() > 0.) { return None }
        let (ds,dt) = ((-gs*gtt+gt*gst)/det,(-gt*gss+gs*gst)/det);
        x = on_a(add(x,add(scale(f.x,ds),scale(f.y,dt))));
        let step = ds.hypot(dt);
        if step <= 1e-3*tol || step <= 0.1*tol && step >= 0.5*last { return (b.implicit(x).abs() <= 8.*tol).then_some(x) }
        last = step;
    }
    None
}

/// **A trace refused for two surfaces nearly touching where it marched** (`trace_beside`'s refusal
/// below `SHALLOW`): the surfaces' kinds, the angle they meet at there in degrees, and the point.
/// What a caller reads the meeting's ridges and level curves for (#778 T4, `meeting::marched`),
/// known by its type and never by its words (#780 R3); said as before where it stands.
#[derive(Clone,Debug,PartialEq)]
pub struct Shallow { pub a: &'static str,pub b: &'static str,pub degrees: f64,pub at: V }

impl Shallow {
    fn of(a: &Surface,b: &Surface,p: V) -> Shallow {
        Shallow {a:a.kind(),b:b.kind(),degrees:norm(cross(a.gradient(p),b.gradient(p))).dasin().to_degrees(),at:p}
    }
}

impl std::fmt::Display for Shallow {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let p = self.at;
        write!(f,"a {} and a {} meet at {:.2}° at [{:.4}, {:.4}, {:.4}], nearly touching: their meeting is ill-conditioned \
            there, not built yet",self.a,self.b,self.degrees,p[0],p[1],p[2])
    }
}

impl From<Shallow> for String { fn from(s: Shallow) -> String { s.to_string() } }

/// The curves two surfaces meet in, traced through `seeds` (points on both) within the box
/// `[lo, hi]`: from each seed not already on a traced curve, a march along `∇a × ∇b` both ways
/// — each step predicted along the tangent and pulled onto both surfaces, shortened while the
/// tangent turns more than a few degrees — until it leaves the box or comes back to where it
/// began. A seed where the two are tangent (their normals parallel) is refused (`Shallow`), but
/// for one `beside` a meeting already known — about a point where the two touch and meet in nothing
/// on the faces asked of, or along an edge they touch along (`boolean`): there a seed traces
/// nothing and a march ends.
pub fn trace(a: &Surface,b: &Surface,seeds: &[V],lo: V,hi: V,tol: f64) -> Result<Vec<Curve>,Shallow> {
    trace_beside(a,b,seeds,&mut |_| false,lo,hi,tol)
}

/// `trace`, ending where it comes `beside` a known meeting.
pub(crate) fn trace_beside(a: &Surface,b: &Surface,seeds: &[V],beside: &mut dyn FnMut(V) -> bool,lo: V,hi: V,tol: f64)
    -> Result<Vec<Curve>,Shallow> {
    use super::geom::Traced;
    let tangent = |p: V| -> Option<V> {
        let t = cross(a.gradient(p),b.gradient(p));
        let n = norm(t);
        (n > SHALLOW).then(|| scale(t,1./n))
    };
    let shallow = |p: V| Shallow::of(a,b,p);
    let probe = Traced {a:a.clone(),b:b.clone(),pts:vec![],closed:false};
    let diag = norm(sub(hi,lo)).max(tol);
    let h0 = (a.feature_within(lo,hi).min(b.feature_within(lo,hi))/12.).min(diag/24.).max(diag*1e-4);
    let inside = |p: V,pad: f64| (0..3).all(|k| p[k] >= lo[k]-pad && p[k] <= hi[k]+pad);
    let mut curves: Vec<Curve> = Vec::new();
    // the seeds projected onto both, once; those landing on both are where a patch's edge crosses
    // the other
    let projected: Vec<V> = seeds.iter().map(|&s| probe.project(s)).collect();
    let edge_seeds: Vec<V> = projected.iter().copied()
        .filter(|&q| off_both(a,b,q) <= 16.*tol).collect();
    // the curves traced so far, as traced (exact: each point a projection onto both surfaces), so a
    // seed on one is known by its distance to the curve and not to a polyline's chords
    let mut traced_so_far: Vec<Curve> = Vec::new();
    // **a meeting through a point where the two touch** (#218's 1, #58): two rounds tangent to
    // one face meet in a curve that ends where both touch it — a point where the two surfaces
    // touch and their meeting has two branches crossing (`touch`). A march arriving there ends
    // there; a seed standing there is traced from once the rest are, along each half of each
    // branch no curve has arrived along
    // (found walking over either: a fitted sheet's foot is read numerically, an analytic one's not)
    let touch_at = |q: V| -> Option<Touch> {
        let at = touch_near(a,b,q,tol).or_else(|| touch_near(b,a,q,tol))?;
        let t = touch(a,b,at)?;
        (distance(at,q) <= 2.*t.radius.max(h0)).then_some(t)
    };
    let mut touches: Vec<Touch> = Vec::new();
    // the seeds, then each touch's half-branches: a start, and the sense its march leaves in (none:
    // both ways from a seed)
    let mut starts: Vec<(V,Option<V>)> = projected.iter().map(|&p| (p,None)).collect();
    let mut k = 0;
    loop {
        // every seed traced: each touch's half-branches no curve arrived along, once
        if k == projected.len() { queue_branches(&mut starts,&touches,&traced_so_far,tol) }
        let Some(&(p0,leaving)) = starts.get(k) else { break };
        k += 1;
        if leaving.is_none() && traced_so_far.iter().any(|c| crate::util::space::distance(c.point(c.inverse(p0)),p0) <= 8.*tol) { continue }
        if leaving.is_none() && tangent(p0).is_none() {
            if beside(p0) { continue }
            match touch_at(p0) {
                // where the two touch at that point alone (their meeting there no curve, `touch`'s
                // `Q` definite): nothing to trace from it
                Some(t) if t.branches.is_empty() => continue,
                Some(t) => {
                    if !touches.iter().any(|u| distance(u.at,t.at) <= 8.*tol) { touches.push(t); }
                    continue
                }
                // no touch at a point: the two touch along a curve through it, read from the
                // meeting's critical set by whoever asked (`meeting::ridges`, #778)
                None => return Err(shallow(p0)),
            }
        }
        let mut halves: Vec<Vec<V>> = Vec::new();
        let mut closed = false;
        for sense in [1.,-1.] {
            // from a touch, one way only: out along its half-branch
            if leaving.is_some() && sense < 0. { halves.push(vec![p0]); continue }
            let mut pts = vec![p0];
            let mut p = p0;
            // (a seed where the two touch has no tangent to march along: refused as nearly touching)
            let mut t = match leaving { Some(d) => d,None => match tangent(p0) { Some(t) => scale(t,sense),None => return Err(shallow(p0)) } };
            let mut h = h0;
            // a march from a touch steps out along its branch before it reads the meeting's tangent
            if leaving.is_some() {
                let mut r = h0;
                let first = loop {
                    let q = probe.project(add(p0,scale(t,r)));
                    if tangent(q).is_some() || r > 64.*h0 { break q }
                    r *= 2.;
                };
                let Some(tq) = tangent(first) else { return Err(shallow(first)) };
                pts.push(first);
                p = first;
                t = if dot(tq,t) < 0. { scale(tq,-1.) } else { tq };
            }
            // the way `∇a × ∇b` faces the march: it turns about only through a point where the two
            // are parallel, a touch a step may cross without landing where they nearly are
            let mut facing = dot(cross(a.gradient(p),b.gradient(p)),t) > 0.;
            for _ in 0..100_000 {
                // each step counted against the build's budget (#777 V3): a stopped build ends here
                if !crate::util::meter::charge(crate::util::meter::Stage::Trace,1,|| (p,format!("{} × {}",a.kind(),b.kind()))) { break }
                let q = probe.project(add(p,scale(t,h)));
                // a projection that does not reach both surfaces: a step too long (shortened), or past a
                // sheet's domain, where its foot is held at the edge (still so at the least step): the
                // end of this direction. Past the edge a sheet's signed distance runs on along its
                // tangent extension, so it is the distance to the patch itself that says so — read by
                // the signed one alone, a trace went back and forth at an edge a hundred thousand times
                let lands = |q: V| {
                    let off = |s: &Surface| s.off_patch(q).unwrap_or(0.);
                    off_both(a,b,q).max(off(a)).max(off(b)) <= 16.*tol
                };
                if !lands(q) {
                    // a step too long for the projection to land: shorter, as a turn too sharp is
                    if h > h0*1e-4 { h /= 2.; continue }
                    if super::debug() {
                        // (asked of the surfaces' searches with their warm starts kept as they were)
                        let off = crate::util::warm::kept(|| (a.implicit(q),b.implicit(q)));
                        eprintln!("brep:   a step left the surfaces after {} points: {:.1e} and {:.1e} off, h {h:.1e}",pts.len(),off.0,off.1);
                    }
                    // **a trace leaving a patch ends on its edge**: a march standing on an edge
                    // crossing already (seeded there) ends there; else the last step's length is
                    // halved between where it lands on the patch and where it does not, so the end is
                    // the edge's crossing (a vertex the curve's edge ends at), not up to a step short
                    // of it — and where an edge crossing it was seeded from lies within that step
                    // ahead, that vertex, which a sheet's numerical foot finds only to its slack
                    if edge_seeds.iter().any(|&s| distance(s,p) <= tol) { break }
                    let along = |m: f64| probe.project(add(p,scale(t,m)));
                    let (inside_h,_) = crate::util::roots::bisect(0.,h,|x,y| 0.5*(x+y),|x,y| y-x > tol,|m| lands(along(m)));
                    let last = (inside_h > 0.).then(|| along(inside_h));
                    let end = last.unwrap_or(p);
                    let crossing = edge_seeds.iter().copied()
                        .filter(|&s| dot(sub(s,p),t) > 0. && distance(s,end) <= (h+16.*tol).min(distance(s,p)+tol))
                        .min_by(|x,y| distance(*x,end).total_cmp(&distance(*y,end)));
                    if let Some(q) = crossing.or(last) {
                        drop_crowding(&mut pts,q,0.25*h0,tol);
                        if distance(q,*pts.last().unwrap()) > tol { pts.push(q) }
                    }
                    break
                }
                let Some(tq) = tangent(q) else {
                    // into a known meeting: what runs on from there is known, or off the faces
                    if beside(q) { pts.push(q); break }
                    // into a point where the two touch, along one of its branches: ending there
                    let Some(touched) = touch_at(q).filter(|t| !t.branches.is_empty()) else { return Err(shallow(q)) };
                    if !touched.branches.iter().any(|&d| dot(d,t) > 0.9) { return Err(shallow(q)) }
                    end_at_touch(&mut pts,&mut touches,touched,h0,tol);
                    break
                };
                let turned = (dot(tq,t) > 0.) != facing;
                let tq = if dot(tq,t) < 0. { scale(tq,-1.) } else { tq };
                if dot(tq,t) < 0.996 && h > h0*1e-4 { h /= 2.; continue }
                // **a step across a touch** (two vaults' groins crossing at their crown, where the
                // two are tangent): `∇a × ∇b` turned about between its ends, so it vanished between
                // them, where it shrinks to nothing; a touch there along this march's branch ends the
                // march, as one arrived at does, so the meeting's curves end at the touch, a vertex
                // its four half-branches share, and do not run through it as one curve
                if turned {
                    let sine = |x: V| norm(cross(a.gradient(x),b.gradient(x)));
                    let (sp,sq) = (sine(p),sine(q));
                    let across = add(p,scale(sub(q,p),sp/(sp+sq)));
                    let crossed = touch_at(across)
                        .filter(|u| distance(u.at,across) <= h && u.branches.iter().any(|&d| dot(d,t) > 0.9));
                    if let Some(touched) = crossed { end_at_touch(&mut pts,&mut touches,touched,h0,tol); break }
                    facing = !facing;
                }
                // back where it began: this step's chord passes the start, a closed curve
                if pts.len() > 3 {
                    let d = sub(q,p);
                    let s = dot(sub(p0,p),d)/dot(d,d);
                    if (0. ..=1.).contains(&s) && crate::util::space::distance(p0,crate::util::space::lerp(p,q,s)) < 0.25*h {
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
        if pts.len() >= 2 {
            if super::debug() { eprintln!("brep:   a curve of {} points, closed {closed}, from {:?} to {:?}",pts.len(),pts[0],pts[pts.len()-1]); }
            let traced = Traced {a:a.clone(),b:b.clone(),pts,closed};
            traced_so_far.push(Curve::Traced(std::sync::Arc::new(traced)));
        }
    }
    // each fitted through the seeds on it, exactly: they are where the faces' boundaries cross, the
    // vertices its edges will end at (a fit within `tol` of the traced points may pass farther from
    // a vertex between them, where a swept surface's curve runs on into its tangent). **One seed a
    // place**: a seed within `tol` of one taken is that vertex again (a crossing seeded once more
    // by another face's edge, 4e-14 off it): placed along the trace only to the search's width below
    // (1e-8 of a step), two such were placed apart and interpolated a hair apart in space, and the
    // cubic magnified their rounding into a swing out along the curve and back, on both surfaces
    // all the way (a ring groove's floor ran 0.17 mm past its corner)
    for c in distinct(traced_so_far,tol) {
        let Curve::Traced(t) = &c else { unreachable!() };
        let mut through: Vec<(f64,V)> = Vec::new();
        for &p in &projected {
            if through.iter().any(|&(_,q)| distance(q,p) <= tol) { continue }
            let x = c.inverse(p);
            if distance(c.point(x),p) > 8.*tol { continue }
            // where along the trace, closely: a seed placed a little past its neighbour in the
            // trace's order doubles the fit back on itself there
            let (mut a,mut b) = ((x-1.).max(0.),(x+1.).min((t.pts.len()-1) as f64));
            let d = |x: f64| distance(t.at(x),p);
            for _ in 0..40 {
                let (m1,m2) = (a+(b-a)*0.382,a+(b-a)*0.618);
                if d(m1) < d(m2) { b = m2 } else { a = m1 }
            }
            through.push(((a+b)/2.,p));
        }
        curves.push(fitted(t,tol/4.,&through).unwrap_or(c));
    }
    Ok(curves)
}

/// **One curve a meeting**: of two traces, one whose points all lie within `8 tol` of the other
/// is that meeting traced again — a march that left a seed on its meeting unrecognised (its chord
/// cutting a turn the seed stands on), traced once more from it — and is dropped: the other where
/// it reaches farther, else whichever took the shorter longest step, its chords cutting no turn the
/// other's do. Read on the traces, before either is fitted, so the one kept is fitted through every
/// seed on the meeting; two edges along one meeting split no cell (`vthread.sv`'s fan: the split
/// took the copy whose fit ran 3.4e-6 off it).
fn distinct(traced: Vec<Curve>,tol: f64) -> Vec<Curve> {
    // up to 33 of its points, evenly by index and its ends among them (on both surfaces already),
    // and its longest step
    let read = |t: &super::geom::Traced| {
        let (n,m) = (t.pts.len()-1,(t.pts.len()-1).min(32));
        let pts: Vec<V> = (0..=m).map(|k| t.pts[k*n/m]).collect();
        (pts,t.pts.windows(2).map(|w| distance(w[0],w[1])).fold(0.,f64::max))
    };
    // (its ends within two marches' slack: each lands within `16 tol` past a patch's edge, which
    // its edge's vertex trims, so one reaching that much farther reaches no farther)
    let along = |pts: &[V],c: &Curve| pts.iter().enumerate().all(|(k,&p)| {
        distance(c.point(c.inverse(p)),p) <= if k == 0 || k+1 == pts.len() { 32. } else { 8. }*tol
    });
    let mut kept: Vec<(Curve,Vec<V>,f64)> = Vec::new();
    'next: for c in traced {
        let Curve::Traced(t) = &c else { unreachable!() };
        let (pc,hc) = read(t);
        for k in kept.iter_mut() {
            let c_on_k = along(&pc,&k.0);
            if c_on_k && k.2 <= hc { continue 'next }
            if along(&k.1,&c) { *k = (c,pc,hc); continue 'next }
            if c_on_k { continue 'next }
        }
        kept.push((c,pc,hc));
    }
    kept.into_iter().map(|(c,..)| c).collect()
}

/// A march ended at a touch (`touched`): its points crowding the touch dropped, the touch its last,
/// and the touch known, once, for the branches no curve arrives along to be traced from.
fn end_at_touch(pts: &mut Vec<V>,touches: &mut Vec<Touch>,touched: Touch,h0: f64,tol: f64) {
    drop_crowding(pts,touched.at,0.25*h0,tol);
    pts.push(touched.at);
    if !touches.iter().any(|u| distance(u.at,touched.at) <= 8.*tol) { touches.push(touched); }
}

/// **The steps crowding a march's end dropped**, `end` about to be pushed: a fit through points a
/// step's ten-thousandth apart turns sharply between them, so the last points within `reach` of
/// `end` go, but only while the chord from the point kept before them to `end` passes within
/// `tol` of each: a point that carries the curve's turn stays, however near the end. By distance
/// alone a turn sharper than `reach` was cut to a chord across it (a sharp rim's fan: a meeting
/// 0.09 mm long, `reach` 0.05), and its fit and the edge made from it ran off the surfaces there.
fn drop_crowding(pts: &mut Vec<V>,end: V,reach: f64,tol: f64) {
    let mut keep = pts.len();
    for k in (1..pts.len()).rev() {
        if distance(pts[k],end) >= reach { break }
        if !pts[k..].iter().all(|&m| segment_distance(m,pts[k-1],end) <= tol) { break }
        keep = k;
    }
    pts.truncate(keep);
}

/// Once every seed is traced: from each touch where two surfaces' meeting has branches crossing,
/// each half-branch no traced curve arrives along, queued as a start leaving that way.
fn queue_branches(starts: &mut Vec<(V,Option<V>)>,touches: &[Touch],traced: &[Curve],tol: f64) {
    for t in touches {
        for &d in &t.branches {
            // a curve arrives along it where a point of the curve stands out that way from the touch
            let probe = add(t.at,scale(d,t.radius.max(64.*tol)));
            let arrived = traced.iter().any(|c| {
                let q = c.point(c.inverse(probe));
                distance(q,t.at) <= 2.*t.radius.max(64.*tol)+8.*tol && dot(sub(q,t.at),d) > 0.7*distance(q,t.at)
            });
            if !arrived { starts.push((t.at,Some(d))); }
        }
    }
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
        // each derivative the preimage's mapped, no longer than its greatest stretch makes it
        Surface::Affine(ref m) => derivative_bounds(&m.of,[v0,v1]).map(|b| b.map(|x| x*m.stretch[1])),
        Surface::Torus(_,big,r) => Some([big.abs()+r,r,big.abs()+r,r,r]),
        Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => None,
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
        // the level sets of a convex quadric's distance (an ellipsoid's, an elliptic cylinder's) are
        // its parallels, which turn no faster than `1 / (ρ + d)` at a distance `d` off it, `ρ` its
        // least radius of curvature (inside, a ball of that radius rolls freely, so its medial axis
        // is no nearer); any other image's is not bounded here
        Surface::Affine(ref m) => if m.quadric.is_some() { over(s.feature()+s.implicit(p)-r) } else { None },
        // a cone's implicit is cos α times ρ less a linear part: ρ's curvature at most
        Surface::Cylinder(..) | Surface::Cone(..) => over(rho-r),
        // the distance to the core circle in the meridian half-plane, which turns with ρ
        Surface::Torus(_,big,_) => Some(over((rho-big).dhypot(z)-r)?+over(rho-r)?),
        Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => None,
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
            let n = (crate::util::space::distance(w[0],w[1])/spacing).ceil().clamp(1.,1e6) as usize;
            (0..n).map(move |j| crate::util::space::lerp(w[0],w[1],j as f64/n as f64))
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
    unseen_beside(a,domain,b,known,&[],&mut |_| false,lo,hi,tol)
}

/// `unseen`, known also the curves `stretches` over their stretches (edges the two touch along),
/// and a cell whose meeting (its middle pulled onto both) is `beside` a known meeting (a point
/// where the two touch alone, its branches traced up to it) answered as beside a known curve.
#[allow(clippy::too_many_arguments)]
pub(crate) fn unseen_beside(a: &Surface,domain: [[f64;2];2],b: &Surface,known: &[Curve],stretches: &[(&Curve,[f64;2])],
    beside: &mut dyn FnMut(V) -> bool,lo: V,hi: V,tol: f64) -> Result<Option<V>,String> {
    // a cap on the cells asked, far past any face this kernel builds, that a pathology ends at
    const CELLS: usize = 4_000_000;
    let pad = 4.*tol;
    if (0..3).any(|k| !(hi[k]+pad >= lo[k]-pad)) { return Ok(None) }
    let diag = norm(sub(hi,lo));
    let leaf = (a.feature_within(lo,hi).min(b.feature_within(lo,hi)).min(diag)/64.).max(diag*1e-4).max(16.*tol);
    let floor = (leaf*1e-6).max(4.*tol);
    // the known curves' points, filed by place, read within a cell's reach of its middle
    let spacing = leaf/2.;
    let mut grid = crate::util::space::Grid::new(2.*leaf+spacing);
    let mut along: Vec<V> = Vec::new();
    for c in known { for p in samples_along(c,spacing) { grid.insert(p,along.len() as u32); along.push(p); } }
    for &(c,[t0,t1]) in stretches {
        let n = (2.*c.speed()*(t1-t0).abs()/spacing).ceil().clamp(1.,1e6) as usize;
        for j in 0..=n { let p = c.point(t0+(t1-t0)*j as f64/n as f64); grid.insert(p,along.len() as u32); along.push(p); }
    }
    let near_known = |p: V,within: f64| {
        let mut hit = false;
        grid.around(p,|i| if !hit && crate::util::space::distance(along[i as usize],p) <= within { hit = true });
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
            if beside(q) { continue }
            if off_both(a,b,q) <= 16.*tol {
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
    if super::debug() {
        eprintln!("brep:   a {} over {domain:?} is clear of a {} but for {} known curve(s), in {asked} cells (leaf {leaf:.1e})",
            a.kind(),b.kind(),known.len());
    }
    Ok(None)
}

/// An open traced curve as a cubic B-spline through its points (on both surfaces already) by chord
/// length — its own parameter has a kink at every point, which its derivative, a difference of two
/// projections, reads through the surfaces' noise — each gap's middle projected and the curve
/// refined where it passes further than `tol` from it, and then where it passes further than
/// `4 tol` (the trace's own) from either surface between its points; `None` where no refinement
/// comes within both. Its derivative is then its own exactly: what an integral along it and a
/// face's curve in its parameters need. Each refinement projects one point a gap, never the whole
/// curve again.
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
    let lengths: Vec<f64> = t.pts.windows(2).map(|w| crate::util::space::distance(w[0],w[1])).collect();
    let mut sorted = lengths.clone();
    let median = *sorted.select_nth_unstable_by(lengths.len()/2,f64::total_cmp).1;
    let stride = STRIDE as f64*median;
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
    let debug = super::debug();
    // of the fits the trace's check accepted, the one with the fewest gaps off the surfaces, and
    // their count
    let (mut best,mut fewest) = (None,usize::MAX);
    for round in 0..24 {
        let pts: Vec<V> = at.iter().map(|&x| point(x)).collect();
        let mut s = vec![0.];
        for w in pts.windows(2) { s.push(s.last().unwrap()+crate::util::space::distance(w[0],w[1])); }
        if !(*s.last().unwrap() > 0.) || s.windows(2).any(|w| !(w[1] > w[0])) { break }
        let Some(spline) = super::nurbs::interpolate(&pts,&s,3) else { break };
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
                crate::util::space::distance(super::geom::Curve::point(&fit,fit_near(&fit,m,guess)),m) > tol
            };
            // a gap an eighth of a step wide misses by the traced curve's own noise, and is left
            split[k] = b-a > 0.125 && inner.into_iter().any(off);
        }
        let near = !split.contains(&true);
        // and once near every point of the trace, on both surfaces between its points, to the
        // trace's own `4 tol`: what an edge made from it is held to. A fit near every point it
        // was read at may still stray between them (one through a trace that cut a sharp rim's
        // turn ran 3.4e-6 off its fan), and a gap off the surfaces is split as one that misses
        // (its width no bar: the trace's noise is a quarter of this). Only while that brings
        // fewer off: a gap narrower than the trace's noise lets the cubic wiggle between points
        // crowded closer each split (a chamfer's foot on a sheet, 1.4e-6 off in a gap a
        // twentieth of a step wide, doubled its strays every round), and there the fit with the
        // fewest is the answer — as near both as interpolation through the trace comes
        if near {
            let on = |u: f64| off_both(&t.a,&t.b,super::geom::Curve::point(&fit,u)) <= 4.*tol;
            for k in 0..split.len() {
                split[k] = !(1..4).all(|j| on(s[k]+(s[k+1]-s[k])*j as f64/4.));
            }
        }
        let missed = split.iter().filter(|&&x| x).count();
        if debug {
            eprintln!("brep: fit of a {} × {} trace, round {round}: {} of {} points, {missed} gaps \
                missed{}",t.a.kind(),t.b.kind(),at.len(),t.pts.len(),if near { " off the surfaces" } else { "" });
        }
        if missed == 0 { return Some(fit) }
        if near {
            if missed >= fewest { break }
            (best,fewest) = (Some(fit),missed);
        }
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
    best
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
