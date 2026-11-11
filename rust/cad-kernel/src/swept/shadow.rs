//! **A whole turn of any tool, by its shadow** (#880 Q1): under a turn about an axis, every point
//! of space moves round a circle about it, and over a whole turn the sweep holds each such circle
//! whole or not at all. So the sweep is the solid of revolution of the tool's **shadow** in a
//! half-plane through the axis — the set of `(ρ, z)` whose circle meets the tool — and is built in
//! that half-plane, with no surface met in space:
//!
//! 1. **Candidates** (`Meridian`): every curve of the half-plane the shadow's boundary may run along,
//!    each the image of something on the tool, exact. A face's contact (its points whose normal is
//!    square to the turn's velocity) — a plane's, a line (a hyperbola's arc, `ρ² = t² + k² (z −
//!    h)²`); a cylinder's or a cone's about an axis along the turn's, in a plane with it, or leaning
//!    to it (`turned::Contact`, read in closed form); a ball's great circle in the plane through the
//!    axis and its centre (a circle, and its mirror where the ball reaches across the axis); a face
//!    kept in itself (a plane square to the axis, a cylinder, a cone, a ball or a torus about it) its
//!    own meridian. A sharp edge's image — a line's hyperbola, a circle's (`turned::Rim`, an ellipse
//!    in `(ρ² + z², z)`), a level circle's horizontal stretch. And the axis itself, where the tool
//!    reaches across it. A face's contact is read over the face alone: a line within a plane cut at
//!    its meetings with the face's edges, a round face's over the stretches of its surface's
//!    parameter its points lie in the face over.
//! 2. **Arranged**: each candidate split where another crosses it (found on its samples, then held
//!    to both exact curves by Newton) and where another one's end stands on it (a contact running
//!    into the sharp edge it ends on, the two tangent there, given by construction: the contact's end
//!    is the edge's point). Collinear straight pieces are united first.
//! 3. **Kept by membership**: a piece of the arrangement is on the shadow's boundary exactly where
//!    the shadow lies on one side of it and not the other, read a hair to each side at its middle by
//!    the sweep's own membership (`member::Swept`, the turn over a whole turn); it is kept oriented
//!    with the shadow on its left. Pieces with the shadow on both sides or neither — a contact buried
//!    in the shadow, a stretch of the axis outside it, a candidate's own fold — fall away.
//! 4. **Linked and revolved**: the kept pieces are joined end to end into loops (where several meet
//!    at a point, the one turning most to the right), the counter-clockwise ones bounding regions and
//!    the clockwise ones their holes, and each region is turned once (`build::revolve`), its edges
//!    the candidates' exact curves: a line, a circle, a rational quadratic (`closed::meridian_arc`),
//!    a turned contact or rim (`geom::Curve::Turned`).
//!
//! What it returns is judged as every closed build is (`closed::judge`: its point location against
//! membership, its sense), and refused with the witness where it disagrees (`Unbuilt::Wrong`). A
//! face no plane, cylinder, cone, ball or torus about the axis, a torus not about it, a curve no line
//! or circle, a round contact that is no curve over its face (its two roots meeting, or reaching the
//! axis) — is refused by what it is (`Unbuilt::Curved`). It takes no blank (#880 P1).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{Profile,ProfileEdge};
use crate::geom::{Curve,Frame,Surface};
use crate::query::{Located,Place};
use crate::topo::{Brep,EdgeCurve};
use crate::trajectory::{Screw,Trajectory};
use crate::turned::{Contact,Iv,Jet,Rim,Turned};
use crate::util::space::{cross,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
use super::member::{Solid,Swept};
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

type V = [f64;3];
type P = [f64;2];

/// How many points each curved candidate is sampled at, for its crossings with the others and its
/// stretches within its face.
const SAMPLES: usize = 160;

/// **A candidate of the shadow's boundary**, in the half-plane `(ρ, z)` about the turn's axis.
#[derive(Clone,Debug)]
enum Meridian {
    /// The straight stretch from `a` to `b` (its parameter from 0 to 1): a face kept in itself (a
    /// plane square to the axis, a cylinder or a cone about it), a level circle's stretch, a line's
    /// image where it runs straight, the axis.
    Segment { a: P,b: P },
    /// The image of the line `p + s e` over `span` of `s` (in the turn's frame, `e` unit): a
    /// hyperbola's arc, its throat (the line's point nearest the axis) at neither end inside.
    Line { p: V,e: V,span: [f64;2] },
    /// The circle `c + r (cos θ, sin θ)` over `span` of `θ`: a ball's great circle, a torus's tube.
    Circle { c: P,r: f64,span: [f64;2] },
    /// A cylinder's or a cone's contact, or a sharp circle, turned (`turned::Turned`, its frame the
    /// turn's).
    Turned(Turned),
}

impl Meridian {
    fn span(&self) -> [f64;2] {
        match self {
            Meridian::Segment {..} => [0.,1.],
            Meridian::Line {span,..} | Meridian::Circle {span,..} => *span,
            Meridian::Turned(t) => t.span,
        }
    }
    /// Its point and its rate at `t`.
    fn at(&self,t: f64) -> (P,P) {
        match *self {
            Meridian::Segment {a,b} => ([a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])],[b[0]-a[0],b[1]-a[1]]),
            Meridian::Line {p,e,..} => {
                let x = [p[0]+t*e[0],p[1]+t*e[1],p[2]+t*e[2]];
                let r = x[0].dhypot(x[1]);
                let dr = if r > 0. { (x[0]*e[0]+x[1]*e[1])/r } else { e[0].dhypot(e[1]) };
                ([r,x[2]],[dr,e[2]])
            }
            Meridian::Circle {c,r,..} => {
                let (s,k) = t.dsin_cos();
                ([c[0]+r*k,c[1]+r*s],[-r*s,r*k])
            }
            Meridian::Turned(ref tu) => {
                let [r,z] = tu.source.meridian(Jet::<f64>::at(t));
                ([r.v,z.v],[r.d,z.d])
            }
        }
    }
    fn point(&self,t: f64) -> P { self.at(t).0 }
    /// Its stretch over `[t0, t1]`, from `a` to `b` (its ends as the arrangement placed them), as an
    /// edge of a profile in the half-plane `g`'s `x` and `z` span.
    fn edge(&self,[t0,t1]: [f64;2],a: P,b: P,g: &Frame) -> Result<ProfileEdge,String> {
        let at = |r: f64,z: f64| g.at([r,0.,z]);
        let line = ProfileEdge::Line {a:at(a[0],a[1]),b:at(b[0],b[1])};
        Ok(match *self {
            Meridian::Segment {..} => line,
            Meridian::Line {p,e,..} => {
                let (ez,eh) = (e[2],e[0].dhypot(e[1]));
                if ez.abs() <= 1e-12 || eh <= 1e-12 { return Ok(line) }
                // the hyperbola `ρ² = throat² + k² (z − height)²`
                let s = -(p[0]*e[0]+p[1]*e[1])/(eh*eh);
                let throat = (p[0]+s*e[0]).dhypot(p[1]+s*e[1]);
                super::closed::meridian_arc(&at,(throat,p[2]+s*ez,eh/ez.abs()),a,b)?
            }
            Meridian::Circle {c,r,..} => {
                let frame = Frame {o:at(c[0],c[1]),x:g.x,y:g.z,z:cross(g.x,g.z)};
                ProfileEdge::Arc {frame,r,span:Some([t0.min(t1),t0.max(t1)])}
            }
            Meridian::Turned(tu) => ProfileEdge::Exact(Curve::Turned(Arc::new(Turned {span:[t0.min(t1),t0.max(t1)],..tu}))),
        })
    }
    /// The parameter of its point nearest `q`, near `t` (Gauss–Newton held to its span).
    fn nearest(&self,q: P,mut t: f64) -> f64 {
        let [lo,hi] = self.span();
        for _ in 0..40 {
            let (x,d) = self.at(t);
            let dd = d[0]*d[0]+d[1]*d[1];
            if !(dd > 0.) { break }
            let next = (t-((x[0]-q[0])*d[0]+(x[1]-q[1])*d[1])/dd).clamp(lo,hi);
            if (next-t).abs() <= 1e-15*(1.+t.abs()) { t = next; break }
            t = next;
        }
        t
    }
    /// Its samples, `n + 1` of them evenly over its span (a segment's two ends).
    fn samples(&self,n: usize) -> Vec<(f64,P)> {
        let [lo,hi] = self.span();
        let n = if matches!(self,Meridian::Segment {..}) { 1 } else { n };
        (0..=n).map(|k| { let t = lo+(hi-lo)*k as f64/n as f64; (t,self.point(t)) }).collect()
    }
}

/// `a` to `b`, the stretches of `[a, b]` where `inside` holds, each end found by bisection between
/// samples that differ (`cuts`, where known, are where it may change: each stretch between two is
/// read at its middle alone).
fn stretches(inside: &dyn Fn(f64) -> bool,[a,b]: [f64;2],cuts: Option<Vec<f64>>) -> Vec<[f64;2]> {
    let mut out: Vec<[f64;2]> = Vec::new();
    let mut push = |s: [f64;2]| match out.last_mut() {
        Some(last) if last[1] == s[0] => last[1] = s[1],
        _ => if s[1] > s[0] { out.push(s) },
    };
    if let Some(mut cuts) = cuts {
        cuts.retain(|t| *t > a && *t < b);
        cuts.extend([a,b]);
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|x,y| (*x-*y).abs() <= 1e-13*(1.+x.abs()));
        for w in cuts.windows(2) { if inside(0.5*(w[0]+w[1])) { push([w[0],w[1]]) } }
        return out
    }
    let ts: Vec<f64> = (0..=SAMPLES).map(|k| a+(b-a)*k as f64/SAMPLES as f64).collect();
    let ins: Vec<bool> = ts.iter().map(|&t| inside(t)).collect();
    let edge = |mut lo: f64,mut hi: f64,lo_in: bool| {
        for _ in 0..60 { let m = 0.5*(lo+hi); if inside(m) == lo_in { lo = m } else { hi = m } }
        0.5*(lo+hi)
    };
    let mut start = ins[0].then_some(a);
    for k in 1..ts.len() {
        if ins[k] == ins[k-1] { continue }
        let at = edge(ts[k-1],ts[k],ins[k-1]);
        if ins[k] { start = Some(at) } else if let Some(s) = start.take() { push([s,at]) }
    }
    if let Some(s) = start { push([s,b]) }
    out
}

/// The parameters along the line `x0 + s d` where it meets the edges of the tool's face `k` (all
/// in a plane): each line edge's crossing, each circle's two; none where an edge is another curve
/// (`None`: the stretches are then read on samples).
fn cuts_on_line(tool: &Brep,k: usize,x0: V,d: V) -> Option<Vec<f64>> {
    let mut out = Vec::new();
    for c in tool.faces[k].loops.iter().flatten() {
        match &tool.edges[c.edge as usize].curve {
            EdgeCurve::Curve(Curve::Line {p,d:e}) => {
                // x0 + s d = p + t e, in the plane: least squares
                let w = sub(*p,x0);
                let (dd,de,ee) = (dot(d,d),dot(d,*e),dot(*e,*e));
                let det = dd*ee-de*de;
                if det.abs() <= 1e-14*dd*ee { continue }
                out.push((dot(w,d)*ee-dot(w,*e)*de)/det);
            }
            EdgeCurve::Curve(Curve::Circle(f,r)) => {
                let w = sub(x0,f.o);
                let (qa,qb,qc) = (dot(d,d),2.*dot(w,d),dot(w,w)-r*r);
                out.extend(crate::util::roots::quadratic(qa,qb,qc));
            }
            EdgeCurve::Degenerate => {}
            _ => return None,
        }
    }
    Some(out)
}

fn sub3(a: &V,b: &V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }

/// The tool's points `x` whose image is read, each in the turn's frame `g`.
struct Reader<'a> { tool: &'a Brep,g: Frame,located: Located<'a>,eps: f64 }

impl Reader<'_> {
    fn image(&self,x: V) -> P { let l = self.g.local(x); [l[0].dhypot(l[1]),l[2]] }
    fn in_face(&self,k: usize,x: V) -> bool { self.located.face_place(k,x) != Place::Out }
    /// The line from `a` to `b` (in space) as candidates: split at its point nearest the axis where
    /// that falls inside it, and straight where it runs along the axis, level, or through the axis.
    fn line(&self,a: V,b: V,out: &mut Vec<Meridian>) {
        let (p,q) = (self.g.local(a),self.g.local(b));
        let len = norm(sub3(&q,&p));
        if !(len > self.eps) { return }
        let e = scale(sub3(&q,&p),1./len);
        let eh = e[0].dhypot(e[1]);
        let seg = |s0: f64,s1: f64,out: &mut Vec<Meridian>| {
            let m = Meridian::Line {p,e,span:[s0,s1]};
            out.push(Meridian::Segment {a:m.point(s0),b:m.point(s1)});
        };
        if eh <= 1e-12 { return seg(0.,len,out) }
        let s = -(p[0]*e[0]+p[1]*e[1])/(eh*eh);
        let throat = (p[0]+s*e[0]).dhypot(p[1]+s*e[1]);
        let straight = e[2].abs() <= 1e-12 || throat <= self.eps;
        let mut cuts = vec![0.];
        if s > self.eps && s < len-self.eps { cuts.push(s) }
        cuts.push(len);
        for w in cuts.windows(2) {
            if straight { seg(w[0],w[1],out) } else { out.push(Meridian::Line {p,e,span:[w[0],w[1]]}) }
        }
    }
    /// **The stretches of `[a, b]` over which the curve `x` (its point and its rate) lies in the face
    /// `k`**, each end within the span placed on the face's edge it crosses there: found by
    /// bisection on the face's point location, then held to the edge by Gauss–Newton on `x(t) =
    /// E(s)` (point location reads a point a tolerance off an edge as on it, so a bisected end
    /// stands up to that far past the face).
    fn within(&self,k: usize,x: &dyn Fn(f64) -> (V,V),[a,b]: [f64;2]) -> Vec<[f64;2]> {
        let mut out = stretches(&|t| self.in_face(k,x(t).0),[a,b],None);
        for s in out.iter_mut() {
            for end in 0..2 {
                if s[end] == a || s[end] == b { continue }
                if let Some(t) = self.onto_edge(k,x,s[end],[a,b]) { s[end] = t }
            }
        }
        out.retain(|s| s[1] > s[0]);
        out
    }
    /// The parameter near `t` at which the curve `x` crosses an edge of face `k` (the edge nearest
    /// `x(t)`), held to `span`; none where Gauss–Newton does not close on it.
    fn onto_edge(&self,k: usize,x: &dyn Fn(f64) -> (V,V),t: f64,[lo,hi]: [f64;2]) -> Option<f64> {
        let p = x(t).0;
        let mut best: Option<(f64,&Curve,f64)> = None;
        for c in self.tool.faces[k].loops.iter().flatten() {
            let e = &self.tool.edges[c.edge as usize];
            let EdgeCurve::Curve(curve) = &e.curve else { continue };
            for i in 0..=64 {
                let s = e.t[0]+(e.t[1]-e.t[0])*i as f64/64.;
                let d = norm(sub(curve.point(s),p));
                if best.is_none_or(|b| d < b.0) { best = Some((d,curve,s)) }
            }
        }
        let (_,curve,mut s) = best?;
        let mut t = t;
        for _ in 0..60 {
            let ((xp,xd),(ep,ed,_)) = (x(t),curve.d2(s));
            let f = sub(xp,ep);
            if norm(f) <= 1e-13*(1.+norm(xp)) { return Some(t) }
            // [xd, −ed] (dt, ds) = −f, in least squares
            let (a11,a12,a22) = (dot(xd,xd),-dot(xd,ed),dot(ed,ed));
            let (b1,b2) = (-dot(xd,f),dot(ed,f));
            let det = a11*a22-a12*a12;
            if !(det.abs() > 0.) { return None }
            t = (t+(b1*a22-a12*b2)/det).clamp(lo,hi);
            s += (a11*b2-a12*b1)/det;
        }
        let ((xp,_),(ep,..)) = (x(t),curve.d2(s));
        (norm(sub(xp,ep)) <= self.eps).then_some(t)
    }
    /// The 3D line along the surface of face `k` from `a` to `b`, within the face.
    fn ruling(&self,k: usize,a: V,b: V,out: &mut Vec<Meridian>) {
        let d = sub3(&b,&a);
        for [s0,s1] in self.within(k,&|s| ([a[0]+s*d[0],a[1]+s*d[1],a[2]+s*d[2]],d),[0.,1.]) {
            self.line([a[0]+s0*d[0],a[1]+s0*d[1],a[2]+s0*d[2]],[a[0]+s1*d[0],a[1]+s1*d[1],a[2]+s1*d[2]],out)
        }
    }
    /// The distinct values the face `k`'s edges take of its surface's parameter `which` (each
    /// edge's at its middle: the meridian parameter of a face bounded by circles about its axis).
    fn bounds_of(&self,k: usize,which: usize) -> Vec<f64> {
        let face = &self.tool.faces[k];
        let seams = face.seams();
        let mut out: Vec<f64> = face.loops.iter().flatten().filter(|c| !seams.contains(&c.edge)).map(|c| {
            let e = &self.tool.edges[c.edge as usize];
            face.surface.inverse(e.point(0.5*(e.t[0]+e.t[1]),&self.tool.vertices))[which]
        }).collect();
        out.sort_by(f64::total_cmp);
        out.dedup_by(|x,y| (*x-*y).abs() <= 1e-9);
        out
    }
    /// A point of face `k` (its first edge's start), what a stretch of a face kept in itself is read
    /// at round its axis.
    fn any_point(&self,k: usize) -> Option<V> {
        let c = self.tool.faces[k].loops.iter().flatten().next()?;
        Some(self.tool.vertices[self.tool.ends(c)[0] as usize].p)
    }
}

/// **Every candidate of the shadow's boundary**, read off the tool's B-rep (the module's step 1).
fn candidates(r: &Reader) -> Result<Vec<Meridian>,Unbuilt> {
    let (tool,g,eps) = (r.tool,r.g,r.eps);
    let z_axis = g.z;
    let mut out: Vec<Meridian> = Vec::new();
    // each sharp edge's image
    let uses = tool.uses();
    for (ei,e) in tool.edges.iter().enumerate() {
        let EdgeCurve::Curve(curve) = &e.curve else { continue };
        let faces: Vec<usize> = uses[ei].iter().map(|u| u.0).collect();
        // a seam (one face on both sides) bounds nothing
        if faces.len() == 2 && faces[0] == faces[1] { continue }
        let mid = e.point(0.5*(e.t[0]+e.t[1]),&tool.vertices);
        // a smooth edge (one normal across it) bounds nothing its faces' contacts do not
        if faces.len() == 2 {
            let n: Vec<Option<V>> = faces.iter().map(|&f| tool.faces[f].outward(mid)).collect();
            if let (Some(a),Some(b)) = (n[0],n[1]) { if norm(cross(a,b)) <= 1e-9 && dot(a,b) > 0. { continue } }
        }
        match curve {
            Curve::Line {..} => r.line(e.point(e.t[0],&tool.vertices),e.point(e.t[1],&tool.vertices),&mut out),
            Curve::Circle(f,rad) => {
                let rad = rad.abs();
                let (o,x,y) = (g.local(f.o),g.dir_local(unit(f.x)),g.dir_local(unit(f.y)));
                let [w0,w1] = e.t;
                let n = cross(x,y);
                if n[0].dhypot(n[1]) <= 1e-12 {
                    // a level circle: the stretches of its height between its nearest and farthest
                    // points from the axis; a circle about the axis, a point
                    if o[0].dhypot(o[1]) <= eps { continue }
                    let w = (o[0]*y[0]+o[1]*y[1]).datan2(o[0]*x[0]+o[1]*x[1]);
                    let mut cuts = vec![w0,w1];
                    for k in -3..=3 { let c = w+k as f64*PI; if c > w0 && c < w1 { cuts.push(c) } }
                    cuts.sort_by(f64::total_cmp);
                    let at = |w: f64| { let (s,c) = w.dsin_cos(); r.image(f.at([rad*c,rad*s,0.])) };
                    for c in cuts.windows(2) { out.push(Meridian::Segment {a:at(c[0]),b:at(c[1])}) }
                    continue
                }
                let rim = Rim {o,x,y,r:rad};
                if rim.simple(eps) { out.push(Meridian::Turned(Turned::rim(g,rim,[w0,w1]))); continue }
                // a circle reaching the axis turns into no ring
                let clear = crate::turned::pieces([0.,TAU],64).all(|[a,b]| {
                    let p = rim.point(Jet::at(Iv::new(a,b)));
                    (p[0].v*p[0].v+p[1].v*p[1].v).bounds().is_some_and(|[lo,_]| lo > eps*eps)
                });
                if !clear { return Err(Unbuilt::Curved {at:f.o}) }
                // its normal in the plane through the axis and its centre: its two halves turn into
                // one surface, each half's meridian the other's, folded where its height is least
                // and greatest
                let w = (y[2]).datan2(x[2]);
                let mut cuts = vec![w0,w1];
                for k in -3..=3 { let c = w+k as f64*PI; if c > w0 && c < w1 { cuts.push(c) } }
                cuts.sort_by(f64::total_cmp);
                for c in cuts.windows(2) { out.push(Meridian::Turned(Turned::rim(g,rim,[c[0],c[1]]))) }
            }
            // any other curve lying level (a leaning round face's rim cut level: an ellipse, a
            // conic's spline) turns within its plane: its height's stretches between its nearest
            // and farthest points from the axis
            curve => match level(curve,e.t,g,eps) {
                Some(cuts) => {
                    let at = |t: f64| r.image(curve.point(t));
                    for c in cuts.windows(2) { out.push(Meridian::Segment {a:at(c[0]),b:at(c[1])}) }
                }
                None => return Err(Unbuilt::Curved {at:mid}),
            },
        }
    }
    // each face's contact, or the face itself where the turn keeps it in its surface
    for (k,face) in tool.faces.iter().enumerate() {
        let Some(at) = r.any_point(k) else { continue };
        match face.surface {
            Surface::Plane(f) => {
                let n = unit(f.z);
                let m = cross(n,z_axis);
                if norm(m) <= 1e-12 {
                    // level: its own height, from the axis (where it holds the axis) out as far as
                    // its edges reach (their own stretches, exact, reach the rest)
                    let z0 = g.local(f.o)[2];
                    if r.in_face(k,g.at([0.,0.,z0])) {
                        let far = face.loops.iter().flatten().flat_map(|c| {
                            let e = &tool.edges[c.edge as usize];
                            (0..=32).map(move |i| e.point(e.t[0]+(e.t[1]-e.t[0])*i as f64/32.,&tool.vertices))
                        }).map(|x| r.image(x)[0]).fold(0.,f64::max);
                        out.push(Meridian::Segment {a:[0.,z0],b:[far,z0]});
                    }
                    continue
                }
                // its contact: the line of its points in the plane through the axis square to `n × Z`
                let d = unit(cross(n,m));
                let x0 = { let c = g.o; let w = dot(sub(f.o,c),n); [c[0]+w*n[0],c[1]+w*n[1],c[2]+w*n[2]] };
                let along = |s: f64| [x0[0]+s*d[0],x0[1]+s*d[1],x0[2]+s*d[2]];
                let reach = { let (lo,hi) = tool.bounds(); 2.*(norm(sub(hi,lo))+norm(sub(x0,lo))) };
                let cuts = cuts_on_line(tool,k,x0,d);
                let spans = match cuts { Some(_) => stretches(&|s| r.in_face(k,along(s)),[-reach,reach],cuts),None => r.within(k,&|s| (along(s),d),[-reach,reach]) };
                for [s0,s1] in spans {
                    r.line(along(s0),along(s1),&mut out)
                }
            }
            Surface::Cylinder(f,_) | Surface::Cone(f,..) => {
                let (rad,sa,ca) = match face.surface {
                    Surface::Cone(_,rad,half) => { let (s,c) = half.dsin_cos(); (rad,s,c) }
                    Surface::Cylinder(_,rad) => (rad.abs(),0.,1.),
                    _ => unreachable!(),
                };
                let a = unit(f.z);
                let (lo,hi) = rulings(tool,k);
                let gen = |u: f64| (face.surface.point([u,lo]),face.surface.point([u,hi]));
                let np = cross(a,z_axis);
                if norm(np) <= 1e-9 {
                    let ol = g.local(f.o);
                    if ol[0].dhypot(ol[1]) <= eps {
                        // about the axis: kept in itself, its meridian
                        let (p,q) = gen(0.);
                        out.push(Meridian::Segment {a:r.image(p),b:r.image(q)});
                    } else {
                        // along it: its generators in the plane through both axes
                        let w = unit(sub(f.o,g.at([0.,0.,ol[2]])));
                        for w in [w,scale(w,-1.)] {
                            let (p,q) = gen(dot(w,f.y).datan2(dot(w,f.x)));
                            r.ruling(k,p,q,&mut out)
                        }
                    }
                    continue
                }
                if dot(unit(np),sub(f.o,g.o)).abs() <= eps {
                    // its axis in a plane with the turn's: its generators in that plane — where it
                    // crosses the turn's axis within the face, a ring there touches all round
                    if let Some(v) = crossing(f.o,a,g.o,z_axis) { if v >= lo*ca-eps && v <= hi*ca+eps { return Err(Unbuilt::Curved {at:f.o}) } }
                    let w = unit(cross(np,a));
                    for w in [w,scale(w,-1.)] {
                        let (p,q) = gen(dot(w,f.y).datan2(dot(w,f.x)));
                        r.ruling(k,p,q,&mut out)
                    }
                    continue
                }
                // leaning: its two contacts, read in closed form, over the stretches within the face
                // read a little past the face's span each way, so its ends are where it crosses the
                // face's edges (`within`), not where its rulings' span was sampled to reach
                let span = { let past = 0.02*(hi-lo); [lo-past,hi+past] };
                for side in [1.,-1.] {
                    let contact = Contact {o:g.local(f.o),a:g.dir_local(a),r:rad,sa,ca,side};
                    let curve = |v: f64| g.at(contact.point(Jet::<f64>::at(v)).0.map(|j| j.v));
                    let held = crate::turned::pieces(span,64).all(|[x,y]| {
                        let (p,disc) = contact.point(Jet::at(Iv::new(x,y)));
                        disc.v.bounds().is_some_and(|[l,_]| l > 0.) && (p[0].v*p[0].v+p[1].v*p[1].v).bounds().is_some_and(|[l,_]| l > 0.)
                    });
                    if !held { return Err(Unbuilt::Curved {at:curve(0.5*(lo+hi))}) }
                    let rated = |v: f64| { let p = contact.point(Jet::<f64>::at(v)).0; (g.at(p.map(|j| j.v)),g.dir(p.map(|j| j.d))) };
                    for s in r.within(k,&rated,span) {
                        out.push(Meridian::Turned(Turned::meridian(g,contact,s)))
                    }
                }
            }
            Surface::Sphere(f,rad) => {
                let rad = rad.abs();
                let m = g.local(f.o);
                let (rc,zc) = (m[0].dhypot(m[1]),m[2]);
                if rc <= eps {
                    // about the axis: kept in itself, its meridian over the latitudes it spans
                    let sign = dot(unit(f.z),z_axis);
                    if sign.abs() < 1.-1e-9 && !r.bounds_of(k,1).is_empty() { return Err(Unbuilt::Curved {at:f.o}) }
                    let mut vs = r.bounds_of(k,1);
                    vs.extend([-PI/2.,PI/2.]);
                    vs.sort_by(f64::total_cmp);
                    vs.dedup_by(|x,y| (*x-*y).abs() <= 1e-9);
                    let u = face.surface.inverse(at)[0];
                    for w in vs.windows(2) {
                        if !r.in_face(k,face.surface.point([u,0.5*(w[0]+w[1])])) { continue }
                        let span = if sign > 0. { [w[0],w[1]] } else { [-w[1],-w[0]] };
                        out.push(Meridian::Circle {c:[0.,zc],r:rad,span});
                    }
                    continue
                }
                // its great circle in the plane through the axis and its centre, within the face; a
                // part reaching across the axis is mirrored
                let h = [m[0]/rc,m[1]/rc,0.];
                let point = |t: f64| { let (s,c) = t.dsin_cos(); g.at([m[0]+rad*c*h[0],m[1]+rad*c*h[1],zc+rad*s]) };
                let flip = if rc < rad { let t = (-rc/rad).dacos(); vec![t,-t] } else { vec![] };
                let rated = |t: f64| { let (s,c) = t.dsin_cos(); (point(t),g.dir([-rad*s*h[0],-rad*s*h[1],rad*c])) };
                for [t0,t1] in r.within(k,&rated,[-PI,PI]) {
                    let mut cuts = vec![t0,t1];
                    cuts.extend(flip.iter().copied().filter(|&t| t > t0 && t < t1));
                    cuts.sort_by(f64::total_cmp);
                    for w in cuts.windows(2) {
                        let mid = 0.5*(w[0]+w[1]);
                        if rc+rad*mid.dcos() >= 0. { out.push(Meridian::Circle {c:[rc,zc],r:rad,span:[w[0],w[1]]}) }
                        else { out.push(Meridian::Circle {c:[-rc,zc],r:rad,span:[PI-w[1],PI-w[0]]}) }
                    }
                }
            }
            Surface::Torus(f,big,small) => {
                let m = g.local(f.o);
                let sign = dot(unit(f.z),z_axis);
                if m[0].dhypot(m[1]) > eps || sign.abs() < 1.-1e-9 || small.abs() >= big.abs() { return Err(Unbuilt::Curved {at}) }
                // about the axis: kept in itself, its tube's circle over the stretches it spans
                let mut vs = r.bounds_of(k,1);
                if vs.is_empty() { vs = vec![-PI,PI] }
                let u = face.surface.inverse(at)[0];
                let n = vs.len();
                for i in 0..n {
                    let (v0,mut v1) = (vs[i],if n == 1 { vs[0]+TAU } else { vs[(i+1)%n] });
                    if v1 <= v0 { v1 += TAU }
                    if !r.in_face(k,face.surface.point([u,0.5*(v0+v1)])) { continue }
                    let span = if sign > 0. { [v0,v1] } else { [-v1,-v0] };
                    out.push(Meridian::Circle {c:[big.abs(),m[2]],r:small.abs(),span});
                }
            }
            _ => return Err(Unbuilt::Curved {at}),
        }
    }
    // the axis, over the heights the tool spans
    let (lo,hi) = tool.bounds();
    let zs: Vec<f64> = (0..8).map(|c| g.local([[lo[0],hi[0]][c&1],[lo[1],hi[1]][(c>>1)&1],[lo[2],hi[2]][c>>2]])[2]).collect();
    let (zl,zh) = (zs.iter().copied().fold(f64::INFINITY,f64::min),zs.iter().copied().fold(f64::NEG_INFINITY,f64::max));
    out.push(Meridian::Segment {a:[0.,zh],b:[0.,zl]});
    Ok(united(out,eps))
}

/// The parameters over `[t0, t1]` at which `curve`, lying at one height about `g`'s axis, is
/// nearest the axis and farthest from it (its radius's rate nought: found on 256 samples, each held
/// by Newton), with its ends; none where it does not lie level.
fn level(curve: &Curve,[t0,t1]: [f64;2],g: Frame,eps: f64) -> Option<Vec<f64>> {
    let height = |t: f64| g.local(curve.point(t))[2];
    let z0 = height(t0);
    if (0..=32).any(|k| (height(t0+(t1-t0)*k as f64/32.)-z0).abs() > eps) { return None }
    // ρ²'s rate and its own rate at `t`, in the turn's frame
    let rates = |t: f64| -> (f64,f64) {
        let (x,d,dd) = curve.d2(t);
        let (x,d,dd) = (g.local(x),g.dir_local(d),g.dir_local(dd));
        (2.*(x[0]*d[0]+x[1]*d[1]),2.*(d[0]*d[0]+d[1]*d[1]+x[0]*dd[0]+x[1]*dd[1]))
    };
    let n = 256;
    let ts: Vec<f64> = (0..=n).map(|k| t0+(t1-t0)*k as f64/n as f64).collect();
    let rs: Vec<f64> = ts.iter().map(|&t| rates(t).0).collect();
    let mut out = vec![t0];
    for k in 0..n {
        if rs[k] == 0. || rs[k].signum() == rs[k+1].signum() { continue }
        let (mut lo,mut hi) = (ts[k],ts[k+1]);
        let mut t = 0.5*(lo+hi);
        for _ in 0..80 {
            let (f,df) = rates(t);
            if f == 0. { break }
            if f.signum() == rs[k].signum() { lo = t } else { hi = t }
            let next = if df != 0. { t-f/df } else { 0.5*(lo+hi) };
            let next = if next > lo && next < hi { next } else { 0.5*(lo+hi) };
            if (next-t).abs() <= 1e-15*(1.+t.abs()) { t = next; break }
            t = next;
        }
        out.push(t);
    }
    out.push(t1);
    Some(out)
}

/// Where the line through `o` along the unit `a` crosses the line through `c` along the unit `z`
/// (the two in one plane), as its distance along `a` from `o`; none where the two run parallel.
fn crossing(o: V,a: V,c: V,z: V) -> Option<f64> {
    let (b,w) = (dot(a,z),sub(o,c));
    let den = 1.-b*b;
    (den > 1e-12).then(|| (b*dot(w,z)-dot(w,a))/den)
}

/// The span of a cylinder's or a cone's face along its rulings (its surface's `v`), read off its
/// boundary edges.
fn rulings(tool: &Brep,k: usize) -> (f64,f64) {
    let face = &tool.faces[k];
    face.loops.iter().flatten().flat_map(|c| {
        let e = &tool.edges[c.edge as usize];
        (0..=8).map(move |i| e.point(e.t[0]+(e.t[1]-e.t[0])*i as f64/8.,&tool.vertices))
    }).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),p| { let v = face.surface.inverse(p)[1]; (lo.min(v),hi.max(v)) })
}

/// The candidates with their straight pieces along one line united: each such line's stretches
/// merged where they overlap, so no two candidates lie along one another.
fn united(cands: Vec<Meridian>,eps: f64) -> Vec<Meridian> {
    let (segs,mut out): (Vec<Meridian>,Vec<Meridian>) = cands.into_iter().partition(|m| matches!(m,Meridian::Segment {..}));
    let mut lines: Vec<(P,P,Vec<[f64;2]>)> = Vec::new();
    for s in segs {
        let Meridian::Segment {a,b} = s else { unreachable!() };
        let len = (b[0]-a[0]).dhypot(b[1]-a[1]);
        if !(len > eps) { continue }
        let d = [(b[0]-a[0])/len,(b[1]-a[1])/len];
        let found = lines.iter().position(|(o,e,_)| {
            (e[0]*d[1]-e[1]*d[0]).abs() <= 1e-9 && ((a[0]-o[0])*e[1]-(a[1]-o[1])*e[0]).abs() <= eps
        });
        let i = found.unwrap_or_else(|| { lines.push((a,d,Vec::new())); lines.len()-1 });
        let (o,e,spans) = &mut lines[i];
        let along = |p: P| (p[0]-o[0])*e[0]+(p[1]-o[1])*e[1];
        let (s0,s1) = (along(a),along(b));
        spans.push([s0.min(s1),s0.max(s1)]);
    }
    for (o,e,mut spans) in lines {
        spans.sort_by(|x,y| x[0].total_cmp(&y[0]));
        let mut merged: Vec<[f64;2]> = Vec::new();
        for s in spans {
            match merged.last_mut() { Some(m) if s[0] <= m[1]+eps => m[1] = m[1].max(s[1]),_ => merged.push(s) }
        }
        for [s0,s1] in merged {
            out.push(Meridian::Segment {a:[o[0]+s0*e[0],o[1]+s0*e[1]],b:[o[0]+s1*e[0],o[1]+s1*e[1]]});
        }
    }
    out
}

/// Where the candidates `a` and `b` cross, as their parameters: their samples' polylines crossing,
/// each held to both exact curves by Newton.
fn crossings(a: &Meridian,b: &Meridian,sa: &[(f64,P)],sb: &[(f64,P)],near: f64) -> Vec<[f64;2]> {
    let boxed = |s: &[(f64,P)]| s.iter().fold([[f64::INFINITY;2],[f64::NEG_INFINITY;2]],|[lo,hi],(_,p)| [[lo[0].min(p[0]),lo[1].min(p[1])],[hi[0].max(p[0]),hi[1].max(p[1])]]);
    let ([la,ha],[lb,hb]) = (boxed(sa),boxed(sb));
    if la[0] > hb[0]+near || lb[0] > ha[0]+near || la[1] > hb[1]+near || lb[1] > ha[1]+near { return vec![] }
    let mut out: Vec<[f64;2]> = Vec::new();
    for i in 0..sa.len()-1 {
        let (p0,p1) = (sa[i].1,sa[i+1].1);
        for j in 0..sb.len()-1 {
            let (q0,q1) = (sb[j].1,sb[j+1].1);
            let (d,e) = ([p1[0]-p0[0],p1[1]-p0[1]],[q1[0]-q0[0],q1[1]-q0[1]]);
            let den = d[0]*e[1]-d[1]*e[0];
            if den.abs() <= 1e-300 { continue }
            let w = [q0[0]-p0[0],q0[1]-p0[1]];
            let (s,t) = ((w[0]*e[1]-w[1]*e[0])/den,(w[0]*d[1]-w[1]*d[0])/den);
            let slack = 1e-9;
            if s < -slack || s > 1.+slack || t < -slack || t > 1.+slack { continue }
            let mut x = [sa[i].0+s*(sa[i+1].0-sa[i].0),sb[j].0+t*(sb[j+1].0-sb[j].0)];
            let ([a0,a1],[b0,b1]) = (a.span(),b.span());
            let mut ok = false;
            for _ in 0..50 {
                let ((pa,da),(pb,db)) = (a.at(x[0]),b.at(x[1]));
                let f = [pa[0]-pb[0],pa[1]-pb[1]];
                if f[0].dhypot(f[1]) <= 1e-13*(1.+pa[0].abs()+pa[1].abs()) { ok = true; break }
                let det = -da[0]*db[1]+da[1]*db[0];
                if det.abs() <= 1e-300 { break }
                let dx = [(-f[0]*(-db[1])+f[1]*(-db[0]))/det,(-da[0]*f[1]+da[1]*f[0])/det];
                x = [(x[0]+dx[0]).clamp(a0,a1),(x[1]+dx[1]).clamp(b0,b1)];
            }
            if !ok {
                let (pa,pb) = (a.point(x[0]),b.point(x[1]));
                ok = (pa[0]-pb[0]).dhypot(pa[1]-pb[1]) <= near;
            }
            if ok { out.push(x) }
        }
    }
    out
}

/// A piece of the arrangement kept on the shadow's boundary: the candidate, its stretch (walked
/// from `t[0]` to `t[1]`, the shadow on its left) and its ends' places among the vertices.
#[derive(Clone,Debug)]
struct Kept { m: usize,t: [f64;2],ends: [usize;2] }

/// **The closed solid `tool` sweeps turned a whole turn about the unit `axis` through `c`**, by its
/// shadow (the module's), membership read on the tool meshed within `bar`, built to `tol`.
pub fn turned(tool: &Brep,c: V,axis: V,bar: f64,tol: f64) -> Result<Brep,Unbuilt> {
    let g = Frame::about(c,axis);
    let size = { let (lo,hi) = tool.bounds(); norm(sub(hi,lo)) };
    let eps = 1e-9*(1.+size);
    let reader = Reader {tool,g,located:Located::new(tool,tol),eps};
    let cands = candidates(&reader)?;
    let solid = Solid::new(tool,bar,1.).map_err(Unbuilt::Failed)?;
    let path = Trajectory::Twist(Screw {origin:c,axis:unit(axis),ratio:1.,advance:0.});
    let swept = Swept {tool:&solid,path:&path,roll:[0.,TAU]};
    let near = 1e-7*(1.+size);
    // (2) the arrangement: each candidate's split parameters
    let samples: Vec<Vec<(f64,P)>> = cands.iter().map(|m| m.samples(SAMPLES)).collect();
    let mut splits: Vec<Vec<f64>> = cands.iter().map(|m| m.span().to_vec()).collect();
    for i in 0..cands.len() {
        for j in i+1..cands.len() {
            for [s,t] in crossings(&cands[i],&cands[j],&samples[i],&samples[j],near) { splits[i].push(s); splits[j].push(t) }
        }
    }
    // each candidate's ends standing on another
    let ends: Vec<P> = cands.iter().flat_map(|m| { let [a,b] = m.span(); [m.point(a),m.point(b)] }).collect();
    for (i,m) in cands.iter().enumerate() {
        for &q in &ends {
            let best = samples[i].iter().map(|&(t,p)| ((p[0]-q[0]).dhypot(p[1]-q[1]),t)).min_by(|x,y| x.0.total_cmp(&y.0)).unwrap().1;
            let t = m.nearest(q,best);
            let p = m.point(t);
            if (p[0]-q[0]).dhypot(p[1]-q[1]) <= near { splits[i].push(t) }
        }
    }
    // (3) each piece kept where the shadow lies on one side of it alone: read a hair to each side,
    // the hair under a third of the way to the nearest other candidate (so it stands in the cells
    // beside the piece, however thin), on the tool meshed finer where the mesh's sag leaves a
    // reading unsettled
    let finer = std::cell::RefCell::new(Vec::<Solid>::new());
    let shadow = |p: P| -> Result<bool,Unbuilt> {
        let x = g.at([p[0],0.,p[1]]);
        let mut last = match swept.member(x) { Ok(b) => return Ok(b),Err(e) => e };
        for k in 0..2 {
            if finer.borrow().len() <= k {
                let s = Solid::new(tool,bar/8f64.powi(k as i32+1),1.).map_err(Unbuilt::Failed)?;
                finer.borrow_mut().push(s);
            }
            let fine = finer.borrow();
            match (Swept {tool:&fine[k],path:&path,roll:[0.,TAU]}).member(x) { Ok(b) => return Ok(b),Err(e) => last = e }
        }
        Err(Unbuilt::Failed(format!("the shadow at ({}, {}) is not settled: {last:?}",p[0],p[1])))
    };
    // the distance from `p` to the candidates but the piece `[t0, t1]` of candidate `i` it is on
    let clearance = |i: usize,[t0,t1]: [f64;2],p: P| -> f64 {
        let (lo,hi) = (t0.min(t1),t0.max(t1));
        let mut best = f64::INFINITY;
        for (j,s) in samples.iter().enumerate() {
            for w in s.windows(2) {
                if j == i && w[1].0 > lo && w[0].0 < hi { continue }
                let (a,b) = (w[0].1,w[1].1);
                let d = [b[0]-a[0],b[1]-a[1]];
                let dd = d[0]*d[0]+d[1]*d[1];
                let f = if dd > 0. { (((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1])/dd).clamp(0.,1.) } else { 0. };
                best = best.min((p[0]-a[0]-f*d[0]).dhypot(p[1]-a[1]-f*d[1]));
            }
        }
        best
    };
    let mut vertices: Vec<P> = Vec::new();
    let mut vertex = |p: P| -> usize {
        match vertices.iter().position(|v| (v[0]-p[0]).dhypot(v[1]-p[1]) <= near) {
            Some(i) => i,
            None => { vertices.push(p); vertices.len()-1 }
        }
    };
    let mut kept: Vec<Kept> = Vec::new();
    for (i,m) in cands.iter().enumerate() {
        let mut ts = std::mem::take(&mut splits[i]);
        ts.sort_by(f64::total_cmp);
        ts.dedup_by(|x,y| { let (p,q) = (m.point(*x),m.point(*y)); (p[0]-q[0]).dhypot(p[1]-q[1]) <= near });
        let axis_piece = matches!(m,Meridian::Segment {a,b} if a[0] == 0. && b[0] == 0.);
        for w in ts.windows(2) {
            let (pa,pb) = (m.point(w[0]),m.point(w[1]));
            if (pa[0]-pb[0]).dhypot(pa[1]-pb[1]) <= near { continue }
            // read at its middle, and where that is not settled at a third and two thirds along
            let mut sides = None;
            for f in [0.5,1./3.,2./3.] {
                let (p,d) = m.at(w[0]+f*(w[1]-w[0]));
                let l = d[0].dhypot(d[1]);
                if !(l > 0.) { continue }
                let n = [-d[1]/l,d[0]/l];
                let hair = (0.3*clearance(i,[w[0],w[1]],p)).min((8.*solid.sag).max(1e-6*(1.+size))).max(1e-9*(1.+size));
                let left = shadow([p[0]+hair*n[0],p[1]+hair*n[1]]);
                let right = if axis_piece { Ok(false) } else { shadow([p[0]-hair*n[0],p[1]-hair*n[1]]) };
                if let (Ok(l),Ok(r)) = (&left,&right) { sides = Some((*l,*r)); break }
                if f == 2./3. { left?; right?; }
            }
            let Some((left,right)) = sides else { continue };
            if left == right { continue }
            let (a,b) = (vertex(pa),vertex(pb));
            kept.push(if left { Kept {m:i,t:[w[0],w[1]],ends:[a,b]} } else { Kept {m:i,t:[w[1],w[0]],ends:[b,a]} });
        }
    }
    // two candidates along one curve (a tool's mirror halves) keep one piece
    let mut unique: Vec<Kept> = Vec::new();
    for k in kept {
        let mid = cands[k.m].point(0.5*(k.t[0]+k.t[1]));
        let twin = unique.iter().any(|u| u.ends == k.ends && {
            let q = cands[u.m].point(0.5*(u.t[0]+u.t[1]));
            (q[0]-mid[0]).dhypot(q[1]-mid[1]) <= 1e3*near
        });
        if !twin { unique.push(k) }
    }
    let kept = unique;
    if kept.is_empty() { return Err(Unbuilt::Failed("the tool's shadow has no boundary".into())) }
    // (4) linked into loops, the shadow on their left
    let tangent = |k: &Kept,at_end: bool| -> P {
        let (_,d) = cands[k.m].at(if at_end { k.t[1] } else { k.t[0] });
        let s = if k.t[1] >= k.t[0] { 1. } else { -1. };
        let l = d[0].dhypot(d[1]).max(1e-300);
        [s*d[0]/l,s*d[1]/l]
    };
    let mut used = vec![false;kept.len()];
    let mut loops: Vec<Vec<usize>> = Vec::new();
    while let Some(first) = used.iter().position(|u| !u) {
        let mut walk = vec![first];
        used[first] = true;
        loop {
            let last = &kept[*walk.last().unwrap()];
            if last.ends[1] == kept[first].ends[0] { break }
            let din = tangent(last,true);
            let next = (0..kept.len()).filter(|&j| !used[j] && kept[j].ends[0] == last.ends[1])
                .min_by(|&i,&j| {
                    let turn = |k: usize| { let d = tangent(&kept[k],false); (din[0]*d[1]-din[1]*d[0]).datan2(din[0]*d[0]+din[1]*d[1]) };
                    turn(i).total_cmp(&turn(j))
                });
            let Some(next) = next else {
                let at = vertices[last.ends[1]];
                return Err(Unbuilt::Failed(format!("the tool's shadow's boundary does not close at ({}, {})",at[0],at[1])))
            };
            used[next] = true;
            walk.push(next);
        }
        loops.push(walk);
    }
    // each loop's polyline and its signed area: counter-clockwise a region, clockwise a hole
    let (cs,ks) = (&cands,&kept);
    let polyline = |l: &[usize]| -> Vec<P> {
        l.iter().flat_map(|&k| { let p = &ks[k]; (0..16).map(move |i| cs[p.m].point(p.t[0]+(p.t[1]-p.t[0])*i as f64/16.)) }).collect()
    };
    let area = |poly: &[P]| -> f64 { (0..poly.len()).map(|i| { let (a,b) = (poly[i],poly[(i+1)%poly.len()]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()/2. };
    let polys: Vec<Vec<P>> = loops.iter().map(|l| polyline(l)).collect();
    let areas: Vec<f64> = polys.iter().map(|p| area(p)).collect();
    let outers: Vec<usize> = (0..loops.len()).filter(|&i| areas[i] > 0.).collect();
    let mut holes: Vec<Vec<usize>> = vec![Vec::new();loops.len()];
    for h in (0..loops.len()).filter(|&i| areas[i] < 0.) {
        let probe = polys[h][0];
        let inside = outers.iter().copied().filter(|&o| super::region::parity(&polys[o],probe)).min_by(|&a,&b| areas[a].total_cmp(&areas[b]));
        let Some(o) = inside else { return Err(Unbuilt::Failed("a hole in the tool's shadow lies in no region of it".into())) };
        holes[o].push(h);
    }
    let mut out = Brep::default();
    for &o in &outers {
        let edges = |l: &[usize]| -> Result<Vec<ProfileEdge>,Unbuilt> {
            l.iter().map(|&k| { let p = &kept[k]; cands[p.m].edge(p.t,vertices[p.ends[0]],vertices[p.ends[1]],&g).map_err(Unbuilt::Failed) }).collect()
        };
        let mut ls = vec![edges(&loops[o])?];
        for &h in &holes[o] { ls.push(edges(&loops[h])?) }
        let profile = Profile {origin:g.o,normal:g.y,names:Vec::new(),loops:ls};
        let solid = crate::build::revolve(&profile,g.o,g.z,TAU).map_err(Unbuilt::Failed)?;
        out.join(&solid);
    }
    out.check(tol).map_err(|e| Unbuilt::Failed(e.msg))?;
    // judged by the definition before it is returned
    let (lo,hi) = out.bounds();
    let corners: Vec<V> = (0..8).map(|c| [[lo[0],hi[0]][c&1],[lo[1],hi[1]][(c>>1)&1],[lo[2],hi[2]][c>>2]]).collect();
    let (domain,..) = super::closed::cube_holding(&corners,tol)?;
    super::closed::judge(&out,&swept,&domain,None,&[],tol)?;
    Ok(out)
}

