//! The questions a Boolean asks of geometry and of a boundary: where a curve meets a surface,
//! whether a point of a face's surface is inside the face, and whether a point of space is inside
//! a solid.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{Curve,Surface,Uv,V};
use super::topo::{unwrap,Brep,EdgeCurve,Face};
use crate::space::{add,distance,norm,scale};

/// Where a point stands against a face or a solid.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Place { In,Out,On }

/// Where a curve meets a surface over a stretch of its parameter.
#[derive(Clone,Debug,PartialEq)]
pub enum Meets {
    /// The parameters where it crosses (`false`) or touches (`true`) the surface.
    At(Vec<(f64,bool)>),
    /// The whole stretch lies on the surface.
    Along,
}

/// Where two curves cross, to `tol`, in closed form for lines and circles: each crossing's
/// parameter on `a` (over `ta`) and its point, where it lies within `b`'s stretch `tb` too. Two
/// curves running along one another cross nowhere (their shared ends are vertices already), and
/// any other kind of curve gives `None`: the caller has no closed form to read. A Boolean asks it
/// of an edge lying in the other solid's surface, against that face's own edges.
pub fn curve_curve(a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],tol: f64) -> Option<Vec<(f64,V)>> {
    use crate::space::{cross,dot,sub};
    let within = |c: &Curve,t: f64,span: [f64;2]| -> Option<f64> {
        let t = match c.period() { Some(p) => super::geom::around(t,span[0],p),None => t };
        let slack = tol/c.speed().max(1e-300);
        // a crossing just before the start of a closed curve's stretch is at its start
        let t = match c.period() { Some(p) if t > span[1]+slack && t-p >= span[0]-slack => t-p,_ => t };
        (t >= span[0]-slack && t <= span[1]+slack).then_some(t.clamp(span[0],span[1]))
    };
    // points of a line through `p` along the unit `d` at distance `r` from `o`, in the plane
    // through `o` square to `z` the line lies in
    let line_circle = |p: V,d: V,o: V,z: V,r: f64| -> Vec<V> {
        let along = dot(d,z);
        if along.abs() > 1e-12 {
            let q = add(p,scale(d,dot(sub(o,p),z)/along));
            return if (distance(q,o)-r).abs() <= tol { vec![q] } else { Vec::new() };
        }
        if dot(sub(p,o),z).abs() > tol { return Vec::new(); }
        let foot = add(p,scale(d,dot(sub(o,p),d)));
        let h = distance(foot,o);
        if h > r+tol { return Vec::new(); }
        // a tangent line meets the circle once: two roots `half` either side of the foot stand off
        // the circle by `half² / 2r`, so where that is within the tolerance they are the one touch
        // (taken apart, the square root of a rounding puts each a long way from it)
        let half2 = (r*r-h*h).max(0.);
        if half2 <= 2.*r*tol { vec![foot] } else { let half = half2.sqrt(); vec![add(foot,scale(d,-half)),add(foot,scale(d,half))] }
    };
    let on = |c: &Curve,q: V| match *c {
        Curve::Line {p,d} => norm(cross(sub(q,p),d)) <= tol,
        Curve::Circle(f,r) => dot(sub(q,f.o),f.z).abs() <= tol && (distance(q,f.o)-r).abs() <= tol,
        _ => false,
    };
    let points: Vec<V> = match (a,b) {
        (&Curve::Line {p:p1,d:d1},&Curve::Line {p:p2,d:d2}) => {
            let n = cross(d1,d2);
            if norm(n) <= 1e-12 { return Some(Vec::new()); }
            // the nearest points of the two lines
            let w = sub(p1,p2);
            let (b_,d_,e_) = (dot(d1,d2),dot(d1,w),dot(d2,w));
            let den = 1.-b_*b_;
            let (s,t) = ((b_*e_-d_)/den,(e_-b_*d_)/den);
            let (q1,q2) = (add(p1,scale(d1,s)),add(p2,scale(d2,t)));
            if distance(q1,q2) <= tol { vec![q1] } else { Vec::new() }
        }
        (&Curve::Line {p,d},&Curve::Circle(f,r)) | (&Curve::Circle(f,r),&Curve::Line {p,d}) => line_circle(p,d,f.o,f.z,r),
        (&Curve::Circle(f1,r1),&Curve::Circle(f2,r2)) => {
            let n = cross(f1.z,f2.z);
            if norm(n) <= 1e-12 {
                if dot(sub(f2.o,f1.o),f1.z).abs() > tol { return Some(Vec::new()); }
                // coplanar: the radical line, then the first circle along it
                let between = sub(f2.o,f1.o);
                let l = norm(between);
                if l <= tol { return Some(Vec::new()); }
                let u = scale(between,1./l);
                let x = (l*l+r1*r1-r2*r2)/(2.*l);
                let foot = add(f1.o,scale(u,x));
                line_circle(foot,cross(f1.z,u),f1.o,f1.z,r1)
            } else {
                // the line the two planes share, then the first circle along it
                let d = scale(n,1./norm(n));
                let (h1,h2) = (dot(f1.o,f1.z),dot(f2.o,f2.z));
                let c = dot(f1.z,f2.z);
                let den = 1.-c*c;
                let p = add(scale(f1.z,(h1-h2*c)/den),scale(f2.z,(h2-h1*c)/den));
                line_circle(p,d,f1.o,f1.z,r1)
            }
        }
        _ => return None,
    };
    Some(points.into_iter().filter(|&q| on(a,q) && on(b,q)).filter_map(|q| {
        let t = within(a,a.inverse(q),ta)?;
        within(b,b.inverse(q),tb)?;
        Some((t,q))
    }).collect())
}

/// Where two curves lying in `surface` cross, to `tol`: `curve_curve`'s closed forms where they
/// have one, else — one of them a line, a circle or an ellipse — the other's roots on a plane
/// carrying it (a conic's own plane; a line's plane through it square to the surface there), kept
/// where they lie on it within its stretch. Each crossing's parameter on `a` and its point; `None`
/// where neither is a line or a conic, or the other runs along the carrier.
pub fn crossings_in(surface: &Surface,a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],tol: f64) -> Option<Vec<(f64,V)>> {
    use crate::space::cross;
    if let Some(found) = curve_curve(a,ta,b,tb,tol) { return Some(found) }
    // the roots of `other` on the plane carrying `line` (a line or a circle), on it
    let carried = |carrier: &Curve,span: [f64;2],other: &Curve,over: [f64;2]| -> Option<Vec<(f64,V)>> {
        let plane = match *carrier {
            Curve::Line {p,d} => {
                let mid = add(p,scale(d,0.5*(span[0]+span[1])));
                let n = crate::space::normalised(cross(d,surface.gradient(mid)))?;
                super::geom::Frame::about(p,n)
            }
            Curve::Circle(f,_) | Curve::Ellipse(f,..) => super::geom::Frame::about(f.o,f.z),
            _ => return None,
        };
        let Meets::At(roots) = curve_surface(other,over,&Surface::Plane(plane),tol) else { return None };
        let on = |q: V| -> Option<f64> {
            let mut t = carrier.inverse(q);
            if let Some(period) = carrier.period() { t = super::geom::around(t,span[0],period); }
            let slack = tol/carrier.speed().max(1e-300);
            (t >= span[0]-slack && t <= span[1]+slack && distance(carrier.point(t),q) <= 8.*tol).then_some(t)
        };
        Some(roots.into_iter().filter_map(|(t,_)| { let q = other.point(t); on(q).map(|_| (t,q)) }).collect())
    };
    match (a,b) {
        (_,Curve::Line {..} | Curve::Circle(..) | Curve::Ellipse(..)) => carried(b,tb,a,ta),
        (Curve::Line {..} | Curve::Circle(..) | Curve::Ellipse(..),_) => {
            // the roots are `b`'s: read back onto `a`
            let found = carried(a,ta,b,tb)?;
            Some(found.into_iter().map(|(_,q)| {
                let mut t = a.inverse(q);
                if let Some(period) = a.period() { t = super::geom::around(t,ta[0],period); }
                (t,q)
            }).collect())
        }
        _ => None,
    }
}

/// Where `curve` over `[t0, t1]` meets `surface`, to `tol`: every sign change of the surface's
/// signed distance along it refined to a root, every sampled minimum of its size that comes within
/// `tol` refined to a touching point, and an end on the surface a root there.
pub fn curve_surface(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Meets {
    if let (&Curve::Line {p,d},Surface::Cylinder(c,r)) = (curve,surface) { return line_cylinder(p,d,[t0,t1],c,*r,tol) }
    let f = |t: f64| surface.implicit(curve.point(t));
    let length = (t1-t0)*curve.speed();
    let n = ((length/surface.feature()*16.).ceil() as usize)
        .max(if curve.period().is_some() { ((t1-t0)/(std::f64::consts::PI/32.)).ceil() as usize } else { 1 })
        .clamp(16,1<<14);
    let ts: Vec<f64> = (0..=n).map(|i| t0+(t1-t0)*i as f64/n as f64).collect();
    let fs: Vec<f64> = ts.iter().map(|&t| f(t)).collect();
    if fs.iter().all(|v| v.abs() <= tol) { return Meets::Along }
    let mut roots: Vec<(f64,bool)> = Vec::new();
    let push = |t: f64,touch: bool,roots: &mut Vec<(f64,bool)>| {
        if let Some(last) = roots.last() { if (t-last.0).abs()*curve.speed() <= tol { return } }
        roots.push((t,touch));
    };
    if fs[0].abs() <= tol { push(t0,false,&mut roots); }
    for i in 0..n {
        let (a,b) = (fs[i],fs[i+1]);
        if a.abs() > tol && b.abs() > tol && (a < 0.) != (b < 0.) {
            let t = crate::roots::bracketed_root(|t| Some(f(t)),ts[i],a,ts[i+1],b,1e-15*(t1-t0).abs().max(1.));
            push(t,false,&mut roots);
        } else if i+1 < n && b.abs() > tol && b.abs() < a.abs() && b.abs() < fs[i+2].abs() && (a < 0.) == (b < 0.)
            && (b < 0.) == (fs[i+2] < 0.) {
            // a sampled minimum of |f| on one side: does it come down to the surface?
            let (t,v) = crate::roots::brent(&|t| f(t).abs(),ts[i],ts[i+2],1e-15*(t1-t0).abs().max(1.),200,|_,_| false);
            if v <= tol { push(t,true,&mut roots); }
        } else if b.abs() <= tol && i+1 < n {
            // a sample on the surface: a crossing there, or a touch
            let (before,after) = (a,fs[i+2]);
            let touch = before.abs() > tol && after.abs() > tol && (before < 0.) == (after < 0.);
            push(ts[i+1],touch,&mut roots);
        }
    }
    if fs[n].abs() <= tol { push(t1,false,&mut roots); }
    // a swept curve's implicit runs on past its ends along their tangents (and steps across the
    // normal there): a root off the surface itself is that extension's, not the surface's
    if matches!(surface,Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..)) {
        roots.retain(|&(t,_)| {
            let p = curve.point(t);
            distance(p,surface.point(surface.inverse(p))) <= tol.max(1e-9*(1.+norm(p)))
        });
    }
    Meets::At(roots)
}

/// A line through `p` along the unit `d`, over `[t0, t1]`, against the cylinder of radius `r` about
/// `c`'s axis, in closed form: across the axis it is a chord of the circle, and where that chord's
/// two roots stand off the circle by no more than the tolerance (`half² ≤ 2r·tol`) they are one
/// touch at the foot — sampled, a tangent line's double root is found only to the square root of
/// the tolerance. A line along the axis lies in the surface or misses it.
fn line_cylinder(p: V,d: V,[t0,t1]: [f64;2],c: &super::geom::Frame,r: f64,tol: f64) -> Meets {
    use crate::space::{dot,sub};
    let across = |v: V| sub(v,scale(c.z,dot(v,c.z)));
    let (w,e) = (across(sub(p,c.o)),across(d));
    let ee = dot(e,e);
    if ee <= 1e-24 {
        return if (norm(w)-r).abs() <= tol { Meets::Along } else { Meets::At(Vec::new()) }
    }
    let foot = -dot(w,e)/ee;
    let h = norm(add(w,scale(e,foot)));
    if h > r+tol { return Meets::At(Vec::new()) }
    let half2 = r*r-h*h;
    let roots: Vec<(f64,bool)> = if half2 <= 2.*r*tol { vec![(foot,true)] } else {
        let half = (half2/ee).sqrt();
        vec![(foot-half,false),(foot+half,false)]
    };
    let slack = tol;
    Meets::At(roots.into_iter().filter(|&(t,_)| t >= t0-slack && t <= t1+slack).map(|(t,k)| (t.clamp(t0,t1),k)).collect())
}

/// A face's box in space, from its edges and a grid across its parameters, grown by what the
/// samples may miss between them (a quarter of the longest step bounds the bulge of an arc no more
/// than a semicircle between two samples).
pub(crate) fn face_box(b: &Brep,fi: usize) -> ([f64;3],[f64;3]) {
    let f = &b.faces[fi];
    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    let mut step: f64 = 0.;
    let mut grow = |p: V,last: &mut Option<V>| {
        for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); }
        if let Some(q) = *last { step = step.max(distance(p,q)); }
        *last = Some(p);
    };
    let (mut ulo,mut uhi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for l in &f.loops { for c in l {
        let e = &b.edges[c.edge as usize];
        let mut last = None;
        for j in 0..=16 {
            let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/16.;
            grow(e.point(t,&b.vertices),&mut last);
            let uv = c.pcurve.at(t,e,&f.surface,&b.vertices);
            for k in 0..2 { ulo[k] = ulo[k].min(uv[k]); uhi[k] = uhi[k].max(uv[k]); }
        }
    } }
    for i in 0..=8 {
        let (mut row,mut column) = (None,None);
        for j in 0..=8 {
            grow(f.surface.point([ulo[0]+(uhi[0]-ulo[0])*i as f64/8.,ulo[1]+(uhi[1]-ulo[1])*j as f64/8.]),&mut row);
            grow(f.surface.point([ulo[0]+(uhi[0]-ulo[0])*j as f64/8.,ulo[1]+(uhi[1]-ulo[1])*i as f64/8.]),&mut column);
        }
    }
    let pad = step/4.;
    (lo.map(|x| x-pad),hi.map(|x| x+pad))
}

/// `curve_surface` over only the stretches of `[t0, t1]` where the curve comes within `grow` of the
/// box `bx` (a face's box in space): a crossing of a face is in its box, and a B-spline's signed
/// distance, sampled along a ray four times the solid's size, is a foot found by Newton at every
/// sample.  A line is clipped to the box exactly; any other curve by samples, each stretch grown
/// by a sample to either side.  A stretch lying along the surface is asked of the whole again.
pub fn curve_surface_within(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64,bx: ([f64;3],[f64;3]),grow: f64) -> Meets {
    let (lo,hi) = (bx.0.map(|x| x-grow-tol),bx.1.map(|x| x+grow+tol));
    let mut stretches: Vec<[f64;2]> = Vec::new();
    match curve {
        Curve::Line {p,d} => {
            let (mut a,mut z) = (t0,t1);
            for k in 0..3 {
                if d[k].abs() < 1e-300 {
                    if p[k] < lo[k] || p[k] > hi[k] { return Meets::At(Vec::new()) }
                } else {
                    let (u,v) = ((lo[k]-p[k])/d[k],(hi[k]-p[k])/d[k]);
                    a = a.max(u.min(v));
                    z = z.min(u.max(v));
                }
            }
            if a < z { stretches.push([a,z]); }
        }
        _ => {
            const N: usize = 64;
            let ts: Vec<f64> = (0..=N).map(|i| t0+(t1-t0)*i as f64/N as f64).collect();
            // a point within half a step's length of the box may have the curve inside it beside it
            let reach = curve.speed()*(t1-t0)/N as f64;
            let near: Vec<bool> = ts.iter().map(|&t| { let q = curve.point(t); (0..3).all(|k| q[k] >= lo[k]-reach && q[k] <= hi[k]+reach) }).collect();
            let mut i = 0;
            while i <= N {
                if !near[i] { i += 1; continue }
                let start = i.saturating_sub(1);
                while i <= N && near[i] { i += 1; }
                stretches.push([ts[start],ts[i.min(N)]]);
            }
        }
    }
    if stretches.len() == 1 && stretches[0] == [t0,t1] { return curve_surface(curve,[t0,t1],surface,tol) }
    let mut roots = Vec::new();
    for s in stretches {
        match curve_surface(curve,s,surface,tol) {
            Meets::Along => return curve_surface(curve,[t0,t1],surface,tol),
            Meets::At(r) => roots.extend(r),
        }
    }
    Meets::At(roots)
}

/// A boundary with its faces' loops drawn in their parameters, for classifying points against it —
/// borrowing the boundary, or owning it where it is kept (`Located::owned`).
#[derive(Clone,Debug)]
pub struct Located<'a> {
    pub b: std::borrow::Cow<'a,Brep>,
    pub tol: f64,
    /// How far each loop's polygon may stray from its edges (a length): points farther than this
    /// from every edge are classified by the polygon's winding, nearer ones by the side of the
    /// nearest edge.
    coarse: f64,
    /// Each face's loops as closed polygons in its parameters.
    polys: Vec<Vec<Vec<Uv>>>,
    /// Each face's parameter box.
    boxes: Vec<[Uv;2]>,
    /// Each face's box in space (`face_box`).
    space: Vec<([f64;3],[f64;3])>,
    /// Each face's seams (`Face::seams`), which bound nothing.
    seams: Vec<std::collections::BTreeSet<u32>>,
}

impl<'a> Located<'a> {
    /// A located boundary that keeps its own copy.
    pub fn owned(b: Brep,tol: f64) -> Located<'static> {
        let (tol,coarse,polys,boxes,space,seams) = { let l = Located::new(&b,tol); (l.tol,l.coarse,l.polys,l.boxes,l.space,l.seams) };
        Located {b:std::borrow::Cow::Owned(b),tol,coarse,polys,boxes,space,seams}
    }
    pub fn new(b: &'a Brep,tol: f64) -> Located<'a> {
        let coarse = (1e-4*b.size()).max(tol*8.);
        let mut polys = Vec::new();
        let mut boxes = Vec::new();
        for f in &b.faces {
            let loops: Vec<Vec<Uv>> = f.loops.iter().map(|l| {
                let mut pts = Vec::new();
                for c in l {
                    let e = &b.edges[c.edge as usize];
                    let at = |t: f64| c.pcurve.at(t,e,&f.surface,&b.vertices);
                    let mut run = vec![(e.t[0],at(e.t[0]))];
                    let mut stack = vec![(e.t[0],e.t[1],0)];
                    // subdivide until each chord's middle maps within a share of `coarse` of the edge
                    while let Some((a,z,depth)) = stack.pop() {
                        let (ua,uz) = (at(a),at(z));
                        let m = (a+z)/2.;
                        let chord = [(ua[0]+uz[0])/2.,(ua[1]+uz[1])/2.];
                        let off = distance(f.surface.point(chord),e.point(m,&b.vertices));
                        if depth < 2 || (off > coarse/8. && depth < 20) { stack.push((m,z,depth+1)); stack.push((a,m,depth+1)); }
                        else { run.push((z,uz)); }
                    }
                    if c.reversed { run.reverse(); }
                    pts.extend(run[..run.len()-1].iter().map(|p| p.1));
                }
                pts
            }).collect();
            let mut bx = [[f64::INFINITY;2],[f64::NEG_INFINITY;2]];
            for l in &loops { for p in l { for k in 0..2 { bx[0][k] = bx[0][k].min(p[k]); bx[1][k] = bx[1][k].max(p[k]); } } }
            polys.push(loops);
            boxes.push(bx);
        }
        let space = (0..b.faces.len()).map(|fi| face_box(b,fi)).collect();
        let seams = b.faces.iter().map(Face::seams).collect();
        Located {b:std::borrow::Cow::Borrowed(b),tol,coarse,polys,boxes,space,seams}
    }

    /// The parameters of a point of face `fi`'s surface, on the branch of its periods its loops
    /// are drawn on.
    pub fn uv(&self,fi: usize,p: V) -> Uv {
        let f = &self.b.faces[fi];
        let bx = self.boxes[fi];
        let mid = [(bx[0][0]+bx[1][0])/2.,(bx[0][1]+bx[1][1])/2.];
        unwrap(f.surface.inverse_near(p,mid),mid,f.surface.periods())
    }

    /// The point of an edge nearest `p`: its parameter and distance.
    fn nearest(&self,e: u32,p: V) -> (f64,f64) {
        let e = &self.b.edges[e as usize];
        let EdgeCurve::Curve(c) = &e.curve else { return (e.t[0],distance(p,self.b.vertices[e.v[0] as usize].p)) };
        let mut t = c.inverse(p);
        if let Some(period) = c.period() {
            // the representative nearest the edge's stretch
            let mid = (e.t[0]+e.t[1])/2.;
            t += ((mid-t)/period).round()*period;
        }
        let mut best = (t.clamp(e.t[0],e.t[1]),0.);
        best.1 = distance(p,c.point(best.0));
        for end in e.t { let d = distance(p,c.point(end)); if d < best.1 { best = (end,d); } }
        best
    }

    /// Where a point of face `fi`'s surface stands against the face: `On` within `tol` of its
    /// boundary. A seam (an edge the face uses twice, once each side) is no boundary: a point on
    /// one, away from every other edge, is in the face.
    pub fn face_place(&self,fi: usize,p: V) -> Place {
        let f: &Face = &self.b.faces[fi];
        let mut closest: Option<(f64,&super::topo::Coedge,f64)> = None;
        let mut boundary = f64::INFINITY;
        for l in &f.loops { for c in l {
            let (t,d) = self.nearest(c.edge,p);
            if !self.seams[fi].contains(&c.edge) { boundary = boundary.min(d); }
            if closest.is_none_or(|(best,_,_)| d < best) { closest = Some((d,c,t)); }
        } }
        let Some((d,c,t)) = closest else { return Place::Out };
        if boundary <= self.tol { return Place::On }
        if d <= self.tol { return Place::In }
        let uv = self.uv(fi,p);
        let e = &self.b.edges[c.edge as usize];
        if d <= self.coarse && t > e.t[0] && t < e.t[1] {
            // near one edge's interior: which side of it, in the face's parameters — the point
            // on the branch beside each use of the edge (a seam is used twice, once each side of
            // the face), inside if beside any use it is on the face's side
            let hint = f.loops.iter().flatten().find(|u| u.edge == c.edge).map_or(uv,|u| u.pcurve.at(t,e,&f.surface,&self.b.vertices));
            let raw = f.surface.inverse_near(p,hint);
            let inside = f.loops.iter().flatten().filter(|u| u.edge == c.edge).any(|u| {
                let q = u.pcurve.at(t,e,&f.surface,&self.b.vertices);
                let mut dq = u.pcurve.derivative(t,e,&f.surface,&self.b.vertices);
                if u.reversed { dq = [-dq[0],-dq[1]]; }
                let near = unwrap(raw,q,f.surface.periods());
                let left = dq[0]*(near[1]-q[1])-dq[1]*(near[0]-q[0]) > 0.;
                left != f.reversed
            });
            return if inside { Place::In } else { Place::Out }
        }
        let mut winding = 0i32;
        for l in &self.polys[fi] {
            let n = l.len();
            for i in 0..n {
                let (a,b) = (l[i],l[(i+1)%n]);
                if a[1] <= uv[1] {
                    if b[1] > uv[1] && (b[0]-a[0])*(uv[1]-a[1])-(uv[0]-a[0])*(b[1]-a[1]) > 0. { winding += 1; }
                } else if b[1] <= uv[1] && (b[0]-a[0])*(uv[1]-a[1])-(uv[0]-a[0])*(b[1]-a[1]) < 0. { winding -= 1; }
            }
        }
        // a reversed face's loops turn the other way round
        let inside = if f.reversed { winding < 0 } else { winding > 0 };
        if inside { Place::In } else { Place::Out }
    }

    /// Where a point of space stands against the solid: `On` within `tol` of a face, otherwise by
    /// the parity of the faces a ray from it crosses (a ray grazing an edge or touching a face is
    /// abandoned for another direction).
    pub fn solid_place(&self,p: V) -> Place {
        for (fi,f) in self.b.faces.iter().enumerate() {
            if f.surface.implicit(p).abs() <= self.tol && self.face_place(fi,p) != Place::Out { return Place::On }
        }
        let reach = 4.*self.b.size()+distance(p,self.b.vertices.first().map_or(p,|v| v.p));
        const DIRECTIONS: [V;5] = [[0.5773,0.6124,0.5402],[-0.3172,0.8123,-0.4893],[0.7071,-0.3162,0.6325],
            [-0.6412,-0.5123,0.5712],[0.1234,-0.4321,-0.8934]];
        'dirs: for d in DIRECTIONS {
            let d = scale(d,1./norm(d));
            let ray = Curve::Line {p,d};
            let mut crossings = 0;
            for (fi,f) in self.b.faces.iter().enumerate() {
                match curve_surface_within(&ray,[0.,reach],&f.surface,self.tol*1e-3,self.space[fi],self.tol) {
                    Meets::Along => continue 'dirs,
                    Meets::At(roots) => for (t,touch) in roots {
                        if t <= self.tol { continue }
                        let q = add(p,scale(d,t));
                        match self.face_place(fi,q) {
                            Place::Out => {}
                            Place::On => continue 'dirs,
                            Place::In => { if touch { continue 'dirs } crossings += 1; }
                        }
                    },
                }
            }
            return if crossings%2 == 1 { Place::In } else { Place::Out }
        }
        Place::On
    }
}

/// Up to `capacity` points inside the solid `b`, each with a lower bound on its distance from the
/// boundary, deepest first and no two nearer than half the deepest's depth.  Candidates are a
/// jittered grid over the box and points stepped in from the boundary along its normals (for a cell
/// too thin for the grid: a sliver a split leaves), judged against a mesh of the boundary whose sag
/// was measured: a point farther from the mesh than that sag is on the side of the boundary the mesh
/// says it is (its winding number), and is the distance less the sag from the boundary at least.  So
/// nothing returned is a guess.
pub fn interior(b: &Brep,capacity: usize) -> Result<Vec<(V,f64)>,String> {
    let (lo,hi) = b.bounds();
    let size = distance(lo,hi);
    let m = super::mesh::mesh(b,size*5e-4,0.2)?;
    let tris: Vec<[V;3]> = m.tris.iter().map(|t| t.map(|i| m.pts[i as usize])).collect();
    const N: usize = 12;
    const STEPPED: usize = 400;
    let mut rng = crate::rng::Rng::new(0x1a7e);
    let mut candidates: Vec<V> = (0..N*N*N).map(|k| {
        let ijk = [k % N,(k/N) % N,k/(N*N)];
        std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*(ijk[a] as f64+rng.uniform(0.2,0.8))/N as f64)
    }).collect();
    let stride = tris.len().div_ceil(STEPPED).max(1);
    for t in tris.iter().step_by(stride) {
        let (u,v) = ([t[1][0]-t[0][0],t[1][1]-t[0][1],t[1][2]-t[0][2]],[t[2][0]-t[0][0],t[2][1]-t[0][1],t[2][2]-t[0][2]]);
        let n = crate::space::cross(u,v);
        let l = norm(n);
        if l == 0. { continue }
        let centre = [(t[0][0]+t[1][0]+t[2][0])/3.,(t[0][1]+t[1][1]+t[2][1])/3.,(t[0][2]+t[1][2]+t[2][2])/3.];
        for k in 1..=6 { candidates.push(add(centre,scale(n,-size/4f64.dpowi(k)/l))); }
    }
    let judged = crate::par::map(&candidates,|&p| {
        let (mut w,mut d2) = (0.,f64::INFINITY);
        for t in &tris {
            let [a,b,c] = t.map(|q| [q[0]-p[0],q[1]-p[1],q[2]-p[2]]);
            let (la,lb,lc) = (norm(a),norm(b),norm(c));
            let det = a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]);
            let dot = |x: V,y: V| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
            w += 2.*det.datan2(la*lb*lc+dot(a,b)*lc+dot(b,c)*la+dot(c,a)*lb);
            d2 = d2.min(triangle_distance2(p,t));
        }
        (w/(4.*std::f64::consts::PI),d2.sqrt()-m.sag)
    });
    let mut inside: Vec<(V,f64)> = candidates.iter().zip(judged).filter(|(_,(w,d))| *d > 0. && *w > 0.5).map(|(&p,(_,d))| (p,d)).collect();
    inside.sort_by(|a,b| b.1.total_cmp(&a.1));
    let mut chosen: Vec<(V,f64)> = Vec::new();
    for (p,d) in inside {
        if chosen.len() == capacity { break }
        if chosen.first().is_some_and(|f| chosen.iter().any(|c| distance(c.0,p) < f.1/2.)) { continue }
        chosen.push((p,d));
    }
    Ok(chosen)
}

/// The square of the distance from `p` to the triangle `t`.
fn triangle_distance2(p: V,t: &[V;3]) -> f64 {
    let sub = |a: V,b: V| [a[0]-b[0],a[1]-b[1],a[2]-b[2]];
    let dot = |a: V,b: V| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    let (a,b,c) = (t[0],t[1],t[2]);
    let (ab,ac,ap) = (sub(b,a),sub(c,a),sub(p,a));
    let (d1,d2) = (dot(ab,ap),dot(ac,ap));
    let at = |q: V| { let d = sub(p,q); dot(d,d) };
    if d1 <= 0. && d2 <= 0. { return at(a) }
    let bp = sub(p,b);
    let (d3,d4) = (dot(ab,bp),dot(ac,bp));
    if d3 >= 0. && d4 <= d3 { return at(b) }
    let vc = d1*d4-d3*d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. { let v = d1/(d1-d3); return at(add(a,scale(ab,v))) }
    let cp = sub(p,c);
    let (d5,d6) = (dot(ab,cp),dot(ac,cp));
    if d6 >= 0. && d5 <= d6 { return at(c) }
    let vb = d5*d2-d1*d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. { let w = d2/(d2-d6); return at(add(a,scale(ac,w))) }
    let va = d3*d6-d5*d4;
    if va <= 0. && d4-d3 >= 0. && d5-d6 >= 0. { let w = (d4-d3)/((d4-d3)+(d5-d6)); return at(add(b,scale(sub(c,b),w))) }
    let denom = 1./(va+vb+vc);
    let (v,w) = (vb*denom,vc*denom);
    at(add(a,add(scale(ab,v),scale(ac,w))))
}
