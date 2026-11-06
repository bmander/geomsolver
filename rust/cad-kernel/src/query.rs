//! The questions a Boolean asks of geometry and of a boundary: where a curve meets a surface,
//! whether a point of a face's surface is inside the face, and whether a point of space is inside
//! a solid.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{Curve,Surface,Uv,V};
use super::topo::{unwrap,Brep,EdgeCurve,Face};
use super::plane2::{chord,Chord,Touch};
use crate::util::isolate::GaveUp;
use crate::util::interval::{Iv,Num};
use crate::enclose::{Enclosed,I3};
use crate::util::space::{add,distance,dot,norm,scale};

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
    /// No reading was proved (#814): the search gave up, and says where — never a guess in its
    /// place. A ray abandons its direction; a Boolean is refused.
    Refused(Refusal),
}

/// **Why a reading of a curve against a surface was refused, with its witness** (#814 P3): the
/// curve's and the surface's kinds, the stretch read, the box the search could not settle (each
/// unknown's stretch: the curve's parameter first) where it names one, and why.
#[derive(Clone,Debug,PartialEq)]
pub struct Refusal { pub curve: &'static str,pub surface: &'static str,pub stretch: [f64;2],pub at: Vec<[f64;2]>,pub why: &'static str }

impl std::fmt::Display for Refusal {
    fn fmt(&self,f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f,"where a {} over [{}, {}] meets a {} was not proved: its search {}",self.curve,self.stretch[0],self.stretch[1],self.surface,self.why)?;
        if !self.at.is_empty() {
            let b: Vec<String> = self.at.iter().map(|s| format!("[{}, {}]",s[0],s[1])).collect();
            write!(f," (the box {})",b.join(" × "))?;
        }
        Ok(())
    }
}

impl Refusal {
    /// The refusal of `curve` over `stretch` against `surface` for the search's `gave` up.
    fn of(curve: &Curve,stretch: [f64;2],surface: &Surface,gave: GaveUp) -> Refusal {
        let (why,at) = match gave {
            GaveUp::Most => ("read more boxes than any search may",Vec::new()),
            GaveUp::Stopped => ("was stopped by its build's budget",Vec::new()),
            GaveUp::Unsettled {at} => ("could not settle a box",at),
        };
        Refusal {curve:curve.kind(),surface:surface.kind(),stretch,at,why}
    }
    /// A reading with nothing to search over.
    fn unbounded(curve: &Curve,stretch: [f64;2],surface: &Surface) -> Refusal {
        Refusal {curve:curve.kind(),surface:surface.kind(),stretch,at:Vec::new(),why:"had no bounded stretch of either to search"}
    }
}

/// Where two curves cross, to `tol`, in closed form for lines and circles: each crossing's
/// parameter on `a` (over `ta`) and its point, where it lies within `b`'s stretch `tb` too. Two
/// curves running along one another cross nowhere (their shared ends are vertices already), and
/// any other kind of curve gives `None`: the caller has no closed form to read. A Boolean asks it
/// of an edge lying in the other solid's surface, against that face's own edges.
pub fn curve_curve(a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],tol: f64) -> Option<Vec<(f64,V)>> {
    use crate::util::space::{cross,dot,sub};
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
        match chord(distance(foot,o),r,Touch::Gap(tol)) {
            Chord::Miss => Vec::new(),
            Chord::Touch => vec![foot],
            Chord::Cross(half) => vec![add(foot,scale(d,-half)),add(foot,scale(d,half))],
        }
    };
    let on = |c: &Curve,q: V| match *c {
        Curve::Line {p,d} => norm(cross(sub(q,p),d)) <= tol,
        Curve::Circle(f,r) => dot(sub(q,f.o),f.z).abs() <= tol && (distance(q,f.o)-r).abs() <= tol,
        Curve::Ellipse(..) | Curve::Turned(..) => distance(c.point(c.inverse(q)),q) <= tol,
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
        // a circle and an ellipse in one plane (#780 C3: a leaning drum at an end pose cut by a
        // level plane, its ellipse touching the circle the plane cuts from the surface its contact
        // turns into): the ellipse's roots on the circle's cylinder, certified — its trigonometric
        // quartic bounded by its amplitudes (`conic_algebraic`) — so a touch is one root, the vertex
        // where the two are tangent, whatever the rounding
        (&Curve::Circle(f,r),ellipse @ Curve::Ellipse(g,..)) | (ellipse @ Curve::Ellipse(g,..),&Curve::Circle(f,r))
            if norm(cross(f.z,g.z)) <= 1e-12 && dot(sub(g.o,f.o),f.z).abs() <= tol => {
            match curve_surface(ellipse,[0.,std::f64::consts::TAU],&Surface::Cylinder(f,r.abs()),tol) {
                Meets::At(roots) => roots.into_iter().map(|(t,_)| ellipse.point(t)).collect(),
                // the ellipse a circle of the same radius about the same centre: running along one
                // another, they cross nowhere
                Meets::Along => return Some(Vec::new()),
                Meets::Refused(_) => return None,
            }
        }
        // a circle and a plane's cut of a contact's surface turned, in the circle's plane (#780 L2: a
        // flat end at an end pose, its rim the circle its plane cuts from the rim's ring, and its cut
        // of the side's contact turned, tangent to it where the side's contact reaches the rim): the
        // cut's roots on the circle's cylinder, settled as every reading is, so a touch is one root
        (&Curve::Circle(f,r),turned @ Curve::Turned(t)) | (turned @ Curve::Turned(t),&Curve::Circle(f,r))
            if t.plane().is_some_and(|(n,d)| norm(cross(n,f.z)) <= 1e-9 && (dot(n,f.o)-d).abs() <= tol) => {
            match curve_surface(turned,t.span,&Surface::Cylinder(f,r.abs()),tol) {
                Meets::At(roots) => roots.into_iter().map(|(u,_)| turned.point(u)).collect(),
                Meets::Along => return Some(Vec::new()),
                Meets::Refused(_) => return None,
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

/// **Where two curves on sheets lines screw into cross**, in closed form (`ssi`'s meetings of such
/// sheets): two curves on one sheet, each a rule `u(v)` over the sheet's `v` — a plane's cut its
/// `−α/β` (`SheetCut`), a helix (`Iso`) its own `u` — cross where their rules agree; two planes'
/// cuts of two sheets of one screw, on one plane, where the plane meets the helix the two sheets
/// share, the first's rule against that helix's `u`. Each a root of the rules' difference over the
/// stretch both reach (the first's alone for the second), bracketed at samples and settled
/// (`bracketed_root`); a sign change across a rule running off to infinity is no root, and is told
/// by the two curves' points standing apart there. `None` where the two are none of these.
fn screwed_crossings(a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],tol: f64) -> Option<Vec<(f64,V)>> {
    use super::geom::SheetCut;
    // the cut whose rule is read, the other curve, and their stretches
    let (cut,other,cut_is_a,cut_t,other_t) = match (a,b) {
        (Curve::SheetCut(x),o) => (x,o,true,ta,tb),
        (o,Curve::SheetCut(x)) => (x,o,false,tb,ta),
        _ => return None,
    };
    // the other's rule — a cut's on the same sheet, or a helix's `u` on this one — and whether the
    // two parameters are one (both the sheet's `v`)
    let (second,one): (Result<&SheetCut,f64>,bool) = match other {
        Curve::SheetCut(y) if y.blend == cut.blend => (Ok(&**y),true),
        Curve::Iso(y,u) if *y == cut.blend => (Err(*u),true),
        Curve::Iso(_,_) => {
            // a helix of another sheet lying on this one: its `u` here, the same at both its ends
            let ends = other_t.map(|t| cut.rule_at(other.point(t)));
            if (ends[0]-ends[1]).abs()*cut.run() > tol { return None }
            (Err(0.5*(ends[0]+ends[1])),false)
        }
        Curve::SheetCut(y) => {
            // one plane, two sheets of one screw: the helix they share, as `u` on the first
            if crate::util::space::norm(crate::util::space::cross(cut.n,y.n)) > 1e-12 || (cut.h-dot(cut.n,y.n)*y.h).abs() > tol { return None }
            match super::ssi::helix_between(&cut.blend,&y.blend,tol)? {
                Some(u) => (Err(u),false),
                // the two sheets meet nowhere, so neither do their cuts
                None => return Some(Vec::new()),
            }
        }
        _ => return None,
    };
    let g = |v: f64| cut.u(v).0-match second { Ok(c) => c.u(v).0,Err(u) => u };
    let [lo,hi] = if one { [cut_t[0].max(other_t[0]),cut_t[1].min(other_t[1])] } else { cut_t };
    if !(hi > lo) { return Some(Vec::new()) }
    const N: usize = 64;
    let vs: Vec<f64> = (0..=N).map(|k| lo+(hi-lo)*k as f64/N as f64).collect();
    let gs: Vec<f64> = vs.iter().map(|&v| g(v)).collect();
    let mut roots: Vec<f64> = Vec::new();
    for k in 0..N {
        if gs[k] == 0. { roots.push(vs[k]); continue }
        if gs[k]*gs[k+1] < 0. { roots.push(crate::util::roots::bracketed_root(|v| Some(g(v)),vs[k],gs[k],vs[k+1],gs[k+1],1e-15*(1.+hi.abs()))) }
    }
    if gs[N] == 0. { roots.push(hi) }
    let mut out: Vec<(f64,V)> = Vec::new();
    for v in roots {
        // on the other within its stretch, and not a rule's pole
        let p = cut.point(v);
        let t = if one { v } else { other.inverse(p) };
        let slack = 1e-12*(1.+t.abs());
        if t < other_t[0]-slack || t > other_t[1]+slack || distance(other.point(t),p) > 8.*tol { continue }
        if out.iter().any(|x| distance(x.1,p) <= tol) { continue }
        out.push((if cut_is_a { v } else { t },p));
    }
    Some(out)
}

/// Where two curves lying in `surface` cross, to `tol`: `curve_curve`'s closed forms where they
/// have one, else — one of them a line, a circle or an ellipse — the other's roots on a plane
/// carrying it (a conic's own plane; a line's plane through it square to the surface there), kept
/// where they lie on it within its stretch. Each crossing's parameter on `a` and its point; `None`
/// where neither is a line or a conic, or the other runs along the carrier; the refusal where a
/// reading of it was not proved (#814).
pub fn crossings_in(surface: &Surface,a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],tol: f64) -> Result<Option<Vec<(f64,V)>>,Refusal> {
    use crate::util::space::cross;
    if let Some(found) = curve_curve(a,ta,b,tb,tol) { return Ok(Some(found)) }
    if let Some(found) = screwed_crossings(a,ta,b,tb,tol) { return Ok(Some(found)) }
    // the roots of `other` on the plane carrying `line` (a line or a circle), on it
    let carried = |carrier: &Curve,span: [f64;2],other: &Curve,over: [f64;2]| -> Result<Option<Vec<(f64,V)>>,Refusal> {
        let plane = match *carrier {
            Curve::Line {p,d} => {
                let mid = add(p,scale(d,0.5*(span[0]+span[1])));
                let Some(n) = crate::util::space::normalised(cross(d,surface.gradient(mid))) else { return Ok(None) };
                super::geom::Frame::about(p,n)
            }
            Curve::Circle(f,_) | Curve::Ellipse(f,..) => super::geom::Frame::about(f.o,f.z),
            _ => return Ok(None),
        };
        let roots = match curve_surface(other,over,&Surface::Plane(plane),tol) {
            Meets::At(roots) => roots,
            Meets::Along => return Ok(None),
            Meets::Refused(r) => return Err(r),
        };
        let on = |q: V| -> Option<f64> {
            let mut t = carrier.inverse(q);
            if let Some(period) = carrier.period() { t = super::geom::around(t,span[0],period); }
            let slack = tol/carrier.speed().max(1e-300);
            (t >= span[0]-slack && t <= span[1]+slack && distance(carrier.point(t),q) <= 8.*tol).then_some(t)
        };
        let found: Vec<(f64,V)> = roots.into_iter().filter_map(|(t,_)| { let q = other.point(t); on(q).map(|_| (t,q)) }).collect();
        Ok(Some(tangential(carrier,other,over,found,tol)))
    };
    let carries = |c: &Curve| matches!(c,Curve::Line {..} | Curve::Circle(..) | Curve::Ellipse(..));
    match (a,b) {
        (_,b) if carries(b) => carried(b,tb,a,ta),
        (a,_) if carries(a) => {
            // the roots are `b`'s: read back onto `a`
            let Some(found) = carried(a,ta,b,tb)? else { return Ok(None) };
            Ok(Some(found.into_iter().map(|(_,q)| {
                let mut t = a.inverse(q);
                if let Some(period) = a.period() { t = super::geom::around(t,ta[0],period); }
                (t,q)
            }).collect()))
        }
        _ => Ok(None),
    }
}

/// **A crossing where `other` arrives tangent to `carrier` is one crossing**: there `other` stays
/// within the tolerance of the carrier over a stretch the square root of the tolerance times its
/// curvature long, and its roots on the carrier's plane are several points of that one stretch (a
/// fillet's trace on a wall arriving where the fillet lies tangent to a face the wall meets). Roots
/// joined by a stretch of `other` that never leaves the carrier are one, taken at `other`'s end
/// where one of them is there (the vertex that end already is), else nearest the carrier.
fn tangential(carrier: &Curve,other: &Curve,over: [f64;2],mut found: Vec<(f64,V)>,tol: f64) -> Vec<(f64,V)> {
    if found.len() < 2 { return found }
    found.sort_by(|x,y| x.0.total_cmp(&y.0));
    let off = |q: V| distance(carrier.point(carrier.inverse(q)),q);
    let slack = tol/other.speed().max(1e-300);
    let at_end = |t: f64| (t-over[0]).abs() <= slack || (t-over[1]).abs() <= slack;
    let mut out: Vec<(f64,V)> = Vec::new();
    let mut cluster = vec![found[0]];
    let flush = |cluster: &mut Vec<(f64,V)>,out: &mut Vec<(f64,V)>| {
        let pick = cluster.iter().copied().find(|&(t,_)| at_end(t))
            .unwrap_or_else(|| cluster.iter().copied().min_by(|x,y| off(x.1).total_cmp(&off(y.1))).unwrap());
        out.push(pick);
        cluster.clear();
    };
    for &(t,q) in &found[1..] {
        let last = cluster.last().unwrap().0;
        let hugging = (1..8).all(|j| off(other.point(last+(t-last)*j as f64/8.)) <= 8.*tol);
        if !hugging { flush(&mut cluster,&mut out); }
        cluster.push((t,q));
    }
    flush(&mut cluster,&mut out);
    out
}

/// **Where `curve` over `[t0, t1]` meets `surface`, to `tol`**: proved, in closed form where the
/// pair has one measured cheaper than the general reading, else by the general reading every pair
/// has (`General`, #814), and whatever either returns settled by one rule (#780 R2, `settled`) — a
/// curve lying in the surface is `Along`, a root past either end by more than the tolerance is no
/// meeting and one within it is at the end, an end on the surface is a crossing there, and a run of
/// roots the curve keeps within the tolerance of the surface between is one meeting, a crossing
/// where the surface's side changes across it and a touch where it does not. A search that cannot
/// prove its roots is `Refused`, with its witness, never sampled in its place.
/// So an edge a Boolean meets with a surface and a ray a point is located by
/// (`Located::solid_place`) are read alike, by the same readings.
pub fn curve_surface(curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Meets { meets(&Fast,curve,t,surface,tol) }

/// **`curve_surface` by the reader named** (#814's boundary for readings): `Fast`, the closed forms
/// where a pair has one and the general reading for every other (what `curve_surface` reads by),
/// or `General` alone — what each closed form is held to (`curve_surface_searched`).
pub fn curve_surface_by(reader: &dyn Reader,curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Meets { meets(reader,curve,t,surface,tol) }

/// `curve_surface`: `read`, settled.
fn meets(reader: &dyn Reader,curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Meets {
    match read(reader,curve,t,surface,tol) {
        Ok(None) => Meets::Along,
        Ok(Some(roots)) => Meets::At(settled(curve,t,surface,tol,roots)),
        Err(r) => Meets::Refused(r),
    }
}

/// The roots a reader finds of `curve` over `[t0, t1]` against `surface`, not yet settled (what
/// `within` reads stretch by stretch and settles once), `None` where the curve lies along it, or
/// the refusal of a search that proved nothing.
fn read(reader: &dyn Reader,curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Result<Option<Vec<(f64,bool)>>,Refusal> {
    // an affine image's by its preimage's, the curve carried back (`Curve::mapped`) over the
    // parameters its image runs, the tolerance shrunk by the map's greatest stretch
    if let Surface::Affine(m) = surface {
        let (back,[al,be]) = curve.mapped(&m.back);
        return read(reader,&back,[al*t0+be,al*t1+be],&m.of,tol/m.stretch[1])
            .map(|r| r.map(|at| at.into_iter().map(|(t,k)| ((t-be)/al,k)).collect()))
    }
    // each search counted against the build's budget (#777 V3), and in its profile as proved or
    // measured (V4); a build past it reads no more (#780 R3)
    if !crate::util::meter::tick(crate::util::meter::Stage::Ray) { return Ok(Some(Vec::new())) }
    // a curve lying in the surface is along it, whoever reads it
    if along(curve,[t0,t1],surface,tol) { return Ok(None) }
    match reader.read(curve,[t0,t1],surface,tol) {
        Ok((m,proved)) => {
            crate::util::meter::crossing(curve.kind(),surface.kind(),proved);
            Ok(match m { Meets::At(roots) => Some(roots),Meets::Along => None,Meets::Refused(r) => return Err(r) })
        }
        // a build past its budget reads nothing more (#780 R3): the build is refused by its budget
        Err(Uncertified::Stopped) => Ok(Some(Vec::new())),
        Err(Uncertified::Refused(r)) => Err(r),
        Err(Uncertified::Gave(g)) => Err(Refusal::of(curve,[t0,t1],surface,g)),
        Err(Uncertified::NoForm) => Err(Refusal::unbounded(curve,[t0,t1],surface)),
    }
}

/// **How a curve and a surface are read** (#814's boundary for readings): a closed form for the
/// pair, or the general reading every pair has through its kinds' enclosures, or a reading a test
/// supplies — one swapped for another without its callers knowing. Each answers its roots (not yet
/// settled) and whether they are proved (a trace's are measured), or why it has none.
pub trait Reader {
    fn read(&self,curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Result<(Meets,bool),Uncertified>;
}

/// **The closed forms where a pair has one, the general reading for every other** (what
/// `curve_surface` reads by): each closed form kept only where it is measured cheaper than the
/// general reading, and held to it (`curve_surface_searched`, the differential tests).
pub struct Fast;

/// **The general reading alone** (#814 P2): the Krawczyk operator over the kinds' enclosures
/// (`general`).
pub struct General;

impl Reader for Fast {
    fn read(&self,curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Result<(Meets,bool),Uncertified> {
        match closed(curve,t,surface,tol) {
            Ok(m) => Ok((m,true)),
            Err(Uncertified::NoForm) => General.read(curve,t,surface,tol),
            Err(e) => Err(e),
        }
    }
}

impl Reader for General {
    fn read(&self,curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Result<(Meets,bool),Uncertified> { general(curve,t,surface,tol) }
}

/// **Whether `curve` over `[t0, t1]` lies in `surface` to `tol`** (#780 R2: asked once, of every
/// pair, before any reader): read at its ends, middle and quarters (`lies_along`, which a curve
/// clear of the surface leaves at its first point), and where those all lie in it, at the steps
/// the sampled search takes (`samples`) — a ruling of a hyperboloid, a helix a sheet carries, an
/// edge along a stretch of a face's surface. An unbounded stretch lies along nothing: a reader
/// says so where it can (a line along a cylinder).
fn along(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> bool {
    if !(t0.is_finite() && t1.is_finite()) { return false }
    if !lies_along(curve,[t0,t1],surface,tol) { return false }
    let n = samples(curve,[t0,t1],surface);
    (0..=n).all(|i| surface.implicit(curve.point(t0+(t1-t0)*i as f64/n as f64)).abs() <= tol)
}

/// **What any reader returns, settled by one rule** (#780 R2), so a closed form, a bounded
/// isolation and the sampled search read an edge at the tolerance of tangent, or ending on the
/// surface, alike:
/// - a root past either end of `[t0, t1]` by more than `tol` along the curve is no meeting of the
///   stretch, and one within it is at that end;
/// - a run of roots the curve stays within `tol` of the surface between — each next within `tol`
///   of the last along the curve, or the curve within `tol` of the surface at the quarters
///   between them — is one meeting: a crossing where the surface's sign changes across the run,
///   read where the curve has left the tolerance on either side, and a touch, at its middle, where
///   it does not. So a root found twice (at a knot two spans share, from two stretches) is one
///   crossing; a double root rounding parts into two simple ones a hair apart, a shallow pair the
///   samples bracket, and the noise of a polynomial's rounding where the curve runs within the
///   tolerance of the surface for a while (a line along the top of a torus's tube, its distance
///   the fourth power of the way from its point) are one touch. Read as crossings they cut a face
///   where an edge only grazes the surface, and left a vertex on another face's boundary.
/// - an end on the surface (within `tol`, and on a bounded surface on the surface itself, not its
///   extension) is a meeting there, a crossing — the end's vertex, as the sampled search reads it —
///   and takes the run the curve keeps within `tol` of the surface beside it (a touch rounding
///   places a little way in, the helix a ball's pipe starts on).
fn settled(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64,roots: Vec<(f64,bool)>) -> Vec<(f64,bool)> {
    let gap = |a: f64,b: f64| distance(curve.point(a),curve.point(b));
    let side = |t: f64| surface.implicit(curve.point(t));
    let mut out: Vec<(f64,bool)> = roots.into_iter().filter(|r| r.0.is_finite()).filter_map(|(t,k)| {
        if t >= t0 && t <= t1 { return Some((t,k)) }
        let end = if t < t0 { t0 } else { t1 };
        (gap(t,end) <= tol).then_some((end,k))
    }).collect();
    out.sort_by(|x,y| x.0.total_cmp(&y.0));
    // whether the curve keeps within the tolerance of the surface from root `a` to root `b`
    let joined = |a: f64,b: f64| gap(a,b) <= tol || (1..4).all(|k| side(a+(b-a)*k as f64/4.).abs() <= tol);
    // the surface's side where the curve has left the tolerance, from `t` the way `way` goes:
    // stepped twice the tolerance off and doubled, no farther than the stretch's end
    let clear = |t: f64,way: f64| -> f64 {
        let end = if way < 0. { t0 } else { t1 };
        let mut step = 2.*tol/norm(curve.d1(t).1).max(1e-300);
        loop {
            let x = if end.is_finite() { (t+way*step).clamp(t0.min(t1),t0.max(t1)) } else { t+way*step };
            let v = side(x);
            if v.abs() > tol || x == end || !(step < 1e300) { return v }
            step *= 2.;
        }
    };
    let mut runs: Vec<(f64,bool)> = Vec::with_capacity(out.len());
    let mut i = 0;
    while i < out.len() {
        let mut j = i;
        while j+1 < out.len() && joined(out[j].0,out[j+1].0) { j += 1 }
        if j == i { runs.push(out[i]) } else {
            let (a,b) = (out[i].0,out[j].0);
            let touch = out[i..=j].iter().all(|r| r.1) || clear(a,-1.)*clear(b,1.) > 0.;
            runs.push((if touch { 0.5*(a+b) } else { a },touch));
        }
        i = j+1;
    }
    for end in [t0,t1] {
        if !end.is_finite() || !(side(end).abs() <= tol) || !on_bounded(surface,curve.point(end),tol) { continue }
        runs.retain(|r| !joined(r.0,end));
        runs.push((end,false));
    }
    runs.sort_by(|x,y| x.0.total_cmp(&y.0));
    runs
}

/// **Why a reader has no answer** (#780 R3, #814), told apart so each is answered its own way: no
/// closed form for the pair (`NoForm`, and a closed form's search past its most boxes), which the
/// general reading then reads; a build stopped past its budget (`Stopped`), which reads nothing
/// more; or a search that proved nothing (`Gave`, its box, and `Refused`, with its witness), which
/// is the answer.
#[derive(Clone,Debug)]
pub enum Uncertified { NoForm,Stopped,Gave(GaveUp),Refused(Refusal) }

impl From<GaveUp> for Uncertified {
    fn from(g: GaveUp) -> Uncertified { match g { GaveUp::Most => Uncertified::NoForm,GaveUp::Stopped => Uncertified::Stopped,g => Uncertified::Gave(g) } }
}

/// A certified reading of a curve against a surface, or why there is none.
type Certified = Result<Meets,Uncertified>;

/// **Where `curve` meets `surface` in closed form** (#777 V1, M1, #814 P4): a fast path for a pair
/// with one — exact, or its roots isolated from an enclosure of a function whose zeros they are
/// (`util::isolate`), each kept where it is measured cheaper than the general reading and held to
/// it. `NoForm` for any other pair: the general reading's (`Fast`).
fn closed(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Certified {
    match (curve,surface) {
        (&Curve::Line {p,d},Surface::Plane(f)) => line_poly(&[dot(f.z,crate::util::space::sub(p,f.o)),dot(f.z,d)],p,d,[t0,t1],surface,tol,&|_| true),
        (&Curve::Line {p,d},Surface::Cylinder(c,r)) => Ok(line_cylinder(p,d,c,*r,tol)),
        (&Curve::Line {p,d},Surface::Cone(f,r,a)) => line_cone(p,d,[t0,t1],f,*r,*a,surface,tol),
        (&Curve::Line {p,d},Surface::Sphere(c,r)) => Ok(line_sphere(p,d,c.o,*r,tol)),
        (&Curve::Line {p,d},Surface::Torus(f,big,r)) => line_torus(p,d,[t0,t1],f,big.abs(),r.abs(),surface,tol),
        (&Curve::Line {p,d},Surface::Revolution(f,c)) => line_revolution(p,d,f,c,surface,tol),
        (&Curve::Line {p,d},Surface::Extrusion(_,c)) if let Curve::Slid(h) = &**c => line_slid(p,d,h,surface,tol),
        // a level circle's sheet in closed form, measured faster than the general reading and held
        // to it (`swept_closed::every_surface_the_closed_class_emits_locates_points_exactly`), and a
        // line along a screw's axis against the sheet a circle screws into, along the helices the
        // sheet meets that line's cylinder along (`line_along_screw`); any other sheet by the
        // general reading
        (&Curve::Line {p,d},Surface::Blend(_,b)) => match b.level_circle().map(|c| line_screwed_circle(p,d,[t0,t1],&c,tol)) {
            None | Some(Err(Uncertified::NoForm)) => line_along_screw(p,d,[t0,t1],b,tol),
            Some(read) => read,
        },
        (Curve::Circle(f,big),Surface::Blend(_,b)) => circle_round_screwed(f,big.abs(),b,[t0,t1],tol),
        (Curve::Iso(b,u),Surface::Sphere(c,r)) => helix_sphere(b,*u,c.o,r.abs(),[t0,t1],tol),
        // a circle's or an ellipse's on any surface with an algebraic form here, an edge of a
        // drum, a fillet's rim or a section's arc (#780 R2)
        (Curve::Circle(..) | Curve::Ellipse(..),_) if algebraic(surface,tol).is_some() => conic_algebraic(curve,[t0,t1],surface,tol),
        (Curve::BSpline(sp),Surface::Plane(f)) => spline_plane(sp,[t0,t1],f,surface,tol),
        _ => Err(Uncertified::NoForm),
    }
}

/// **A surface's algebraic form**, where it has one here: its degree, the polynomial whose zero set
/// it is (in its frame's coordinates — a plane's height, `ρ² − r²` a cylinder's, `|q|² − r²` a
/// ball's, `ρ² − (r + z tan a)²` a cone's, `(|q|² + R² − r²)² − 4R²ρ²` a torus's), and which of its
/// points are the surface's (a cone's one sheet). `None` for a surface with none.
fn algebraic(surface: &Surface,tol: f64) -> Option<(usize,Box<dyn Fn(V) -> f64 + '_>,Box<dyn Fn(V) -> bool + '_>)> {
    let all: Box<dyn Fn(V) -> bool> = Box::new(|_| true);
    Some(match *surface {
        Surface::Plane(ref f) => (1,Box::new(move |p| f.local(p)[2]),all),
        Surface::Cylinder(ref f,r) => (2,Box::new(move |p| { let q = f.local(p); q[0]*q[0]+q[1]*q[1]-r*r }),all),
        Surface::Sphere(ref f,r) => (2,Box::new(move |p| { let q = f.local(p); q[0]*q[0]+q[1]*q[1]+q[2]*q[2]-r*r }),all),
        Surface::Cone(ref f,r,a) => {
            let (sa,ca) = a.dsin_cos();
            if !(ca.abs() > 1e-9) { return None }
            let k = sa/ca;
            (2,Box::new(move |p| { let q = f.local(p); let w = r+k*q[2]; q[0]*q[0]+q[1]*q[1]-w*w }),
                Box::new(move |p| r+k*f.local(p)[2] >= -tol))
        }
        Surface::Torus(ref f,big,r) => {
            let (big,r) = (big.abs(),r.abs());
            (4,Box::new(move |p| { let q = f.local(p); let (flat,s) = (q[0]*q[0]+q[1]*q[1],q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+big*big-r*r);
                s*s-4.*big*big*flat }),all)
        }
        _ => return None,
    })
}

/// **A circle or an ellipse (`o + a cos t + b sin t`) against a surface with an algebraic form of
/// degree `n`** (`algebraic`; #780 R2): the form along it is a trigonometric polynomial of degree
/// `n`, `c₀ + Σ (a_k cos kt + b_k sin kt)`, its coefficients read exactly from `2n + 1` points
/// evenly round it, written once over `Num` (`Trig`) so its rate is enclosed over a stretch by the
/// same formula read in intervals, and its roots isolated by it (`util::isolate`, #814), a touch
/// accepted where the surface's distance is within `tol`. Each kept where the surface's points are.
fn conic_algebraic(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Certified {
    use std::f64::consts::TAU;
    let (n,form,keep) = algebraic(surface,tol).ok_or(Uncertified::NoForm)?;
    let at = |t: f64| curve.point(t);
    let m = 2*n+1;
    let ys: Vec<f64> = (0..m).map(|j| form(at(TAU*j as f64/m as f64))).collect();
    let c0 = ys.iter().sum::<f64>()/m as f64;
    let terms: Vec<(f64,f64)> = (1..=n).map(|k| {
        let (mut x,mut y) = (0.,0.);
        for (j,v) in ys.iter().enumerate() { let (s,c) = (TAU*(k*j) as f64/m as f64).dsin_cos(); x += v*c; y += v*s; }
        (2.*x/m as f64,2.*y/m as f64)
    }).collect();
    let g = crate::util::isolate::Written(Trig {c0,terms:&terms});
    let found = crate::util::isolate::roots(&g,[t0,t1],tol*1e-3/curve.speed().max(1e-300),&|t| surface.implicit(at(t)).abs() <= tol)?;
    Ok(Meets::At(found.into_iter().filter(|r| keep(at(r.at))).map(|r| (r.at,r.touch)).collect()))
}

/// **A trigonometric polynomial** `c₀ + Σ (a_k cos kt + b_k sin kt)` (`conic_algebraic`), written
/// once over `Num`.
struct Trig<'a> { c0: f64,terms: &'a [(f64,f64)] }

impl crate::util::isolate::Formula for Trig<'_> {
    fn jet<T: Num>(&self,t: crate::util::interval::Jet<T>) -> crate::util::interval::Jet<T> {
        self.terms.iter().enumerate().fold(crate::util::interval::Jet::of(self.c0),|acc,(i,&(x,y))| {
            let (s,c) = t.times((i+1) as f64).sin_cos();
            acc+c.times(x)+s.times(y)
        })
    }
}

/// **A B-spline curve against a plane** (#780 R2): its points' heights over the plane, weighed, are
/// a scalar B-spline (`BSpline::heights`), whose value and rate are enclosed over a stretch by de
/// Boor's algorithm in intervals (`enclose::scalar`), so its roots are isolated span by span
/// (`util::isolate`; a root at a knot both spans share `settled` reads once), a touch accepted
/// where the plane's distance is within `tol`.
fn spline_plane(sp: &super::nurbs::BSpline,[t0,t1]: [f64;2],f: &super::geom::Frame,surface: &Surface,tol: f64) -> Certified {
    let h = sp.heights(f.z,f.o);
    let [lo,hi] = sp.domain();
    let (a,b) = (t0.max(lo),t1.min(hi));
    let mut cuts = vec![a];
    cuts.extend(sp.breaks([a,b]));
    cuts.push(b);
    let c = h.coefficients();
    let at = |t: f64| h.at(t);
    let value = |t: f64| crate::enclose::scalar(sp.degree,&sp.knots,c,[t,t]).0;
    let slope = |a: f64,b: f64| crate::enclose::scalar(sp.degree,&sp.knots,c,[a,b]).1;
    let g = crate::util::isolate::Parts {at:&at,value:&value,slope:&slope};
    let reach = sp.hull_length().max(1e-300)/(hi-lo).max(1e-300);
    let mut out = Vec::new();
    for w in cuts.windows(2) {
        let found = crate::util::isolate::roots(&g,[w[0],w[1]],tol*1e-3/reach,&|t| surface.implicit(sp.point(t)).abs() <= tol)?;
        out.extend(found.into_iter().map(|r| (r.at,r.touch)));
    }
    Ok(Meets::At(out))
}

/// Whether `curve` over `[t0, t1]` lies in `surface` to `tol`, read at its ends, middle and
/// quarters: a ruling of a hyperboloid, a helix a sheet carries.
fn lies_along(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> bool {
    (0..=4).all(|k| surface.implicit(curve.point(t0+(t1-t0)*k as f64/4.)).abs() <= tol)
}

/// Whether `p`, within the tolerance of a bounded surface's implicit, is on the surface itself
/// and not on its extension past its ends (a swept curve's runs on along its end tangents).
fn on_bounded(surface: &Surface,p: V,tol: f64) -> bool {
    !surface.bounded() || distance(p,surface.point(surface.inverse(p))) <= tol.max(1e-9*(1.+norm(p)))
}

/// **A line against a surface whose distance along it is a polynomial**, `c` its coefficients in
/// the line's parameter (lowest first) — a plane's linear, a cone's quadratic, a torus's quartic —
/// its roots isolated with the polynomial's own enclosures (`util::isolate::Poly`), a touch
/// accepted where the surface's distance there is within `tol`, and each kept where `keep` takes
/// its point (a cone's one sheet), over the stretch reached a tolerance past either end (what is
/// past an end, and roots a hair apart, `settled` reads).
fn line_poly(c: &[f64],p: V,d: V,[t0,t1]: [f64;2],surface: &Surface,tol: f64,keep: &dyn Fn(V) -> bool) -> Certified {
    let at = |t: f64| add(p,scale(d,t));
    let speed = norm(d).max(1e-300);
    let slack = tol/speed;
    let (lo,hi) = (t0-slack,t1+slack);
    let found = crate::util::isolate::roots(&crate::util::isolate::Poly(c),[lo,hi],tol*1e-3/speed,&|t| surface.implicit(at(t)).abs() <= tol)?;
    Ok(Meets::At(found.into_iter().filter(|r| keep(at(r.at))).map(|r| (r.at,r.touch)).collect()))
}

/// A line against a cone (`Surface::Cone`, its frame `f`, radius `r` at the origin, half-angle
/// `a`): `x² + y² − (r + z tan a)²` along it, a quadratic, on the one sheet the cone's parameters
/// reach (`r + z tan a ≥ 0`).
fn line_cone(p: V,d: V,t: [f64;2],f: &super::geom::Frame,r: f64,a: f64,surface: &Surface,tol: f64) -> Certified {
    let (sa,ca) = a.dsin_cos();
    if !(ca.abs() > 1e-9) { return Err(Uncertified::NoForm) }
    let k = sa/ca;
    let (q,e) = (f.local(p),f.dir_local(d));
    // (r + z k)² with z = q₂ + t e₂, and x² + y²
    let w = [r+k*q[2],k*e[2]];
    let c = [q[0]*q[0]+q[1]*q[1]-w[0]*w[0],2.*(q[0]*e[0]+q[1]*e[1])-2.*w[0]*w[1],e[0]*e[0]+e[1]*e[1]-w[1]*w[1]];
    line_poly(&c,p,d,t,surface,tol,&|x| r+k*f.local(x)[2] >= -tol)
}

/// A line against a torus about `f`'s axis, its tube's middle radius `big` and the tube's `r`:
/// `(|w|² + R² − r²)² − 4R² (x² + y²)` along it, a quartic.
fn line_torus(p: V,d: V,t: [f64;2],f: &super::geom::Frame,big: f64,r: f64,surface: &Surface,tol: f64) -> Certified {
    use crate::util::isolate::{poly_mul,poly_sum};
    let (q,e) = (f.local(p),f.dir_local(d));
    let sq = |i: usize| [q[i]*q[i],2.*q[i]*e[i],e[i]*e[i]];
    let (x,y,z) = (sq(0),sq(1),sq(2));
    let flat = poly_sum(&[(1.,&x),(1.,&y)]);
    let s = poly_sum(&[(1.,&flat),(1.,&z),(big*big-r*r,&[1.])]);
    let c = poly_sum(&[(1.,&poly_mul(&s,&s)),(-4.*big*big,&flat)]);
    line_poly(&c,p,d,t,surface,tol,&|_| true)
}

/// **A line against a surface of revolution whose meridian is a conic** (a rational B-spline of
/// degree two at most, `Surface::Revolution` about `f`'s axis: a hyperboloid a leaning edge turns
/// into, its meridian `closed::meridian_arc`'s), exactly: over each knot span the meridian's
/// homogeneous radius, height and weight `X, Z, W` are quadratics in the span's own parameter `σ`,
/// and the line's radius squared is `a t² + b t + c` at the height `z(t)`, linear in `t`. Where the
/// line climbs (`z' = D ≠ 0`) its parameter at the meridian's height is `t = N / (W D)`, `N = Z −
/// z₀ W`, and it meets the surface where the quartic `a N² + b D N W + c D² W² − D² X²` is nought,
/// its roots isolated (`util::isolate`); a level line meets the circles at the roots of `N`, each
/// a chord. `NoForm` for a meridian of higher degree. (A line lying in it, a hyperboloid's ruling,
/// `meets` has read as along it already.)
fn line_revolution(p: V,d: V,f: &super::geom::Frame,c: &Curve,surface: &Surface,tol: f64) -> Certified {
    use crate::util::isolate::{poly_mul,poly_sum,poly};
    if let Curve::Turned(m) = c { return line_turned(p,d,f,m,surface,tol) }
    let Curve::BSpline(sp) = c else { return Err(Uncertified::NoForm) };
    if sp.degree > 2 { return Err(Uncertified::NoForm) }
    let (q,e) = (f.local(p),f.dir_local(d));
    let speed = norm(d).max(1e-300);
    let (a,b,cc) = (e[0]*e[0]+e[1]*e[1],2.*(q[0]*e[0]+q[1]*e[1]),q[0]*q[0]+q[1]*q[1]);
    let dz = e[2];
    let [k0,k1] = sp.domain();
    let mut knots = vec![k0];
    knots.extend(sp.breaks([k0,k1]));
    knots.push(k1);
    // a line all but level is read neither way well: the general reading's
    if dz.abs() > 1e-12*speed && dz.abs() < 1e-7*speed { return Err(Uncertified::NoForm) }
    let width = 1e-12;
    // a level line's direction and foot, square to the axis
    let (flat,from) = (f.dir([e[0],e[1],0.]),f.dir([q[0],q[1],0.]));
    let mut out: Vec<(f64,bool)> = Vec::new();
    for w in knots.windows(2) {
        // the span's homogeneous local radius, height and weight, each a quadratic in σ
        let h = |s: f64| { let (x,wt) = sp.homogeneous(w[0]+(w[1]-w[0])*s); let l = f.local(scale(x,1./wt)); [l[0]*wt,l[2]*wt,wt] };
        let (h0,hm,h1) = (h(0.),h(0.5),h(1.));
        let quad = |i: usize| [h0[i],-3.*h0[i]+4.*hm[i]-h1[i],2.*h0[i]-4.*hm[i]+2.*h1[i]];
        let (xx,zz,ww) = (quad(0),quad(1),quad(2));
        let n = poly_sum(&[(1.,&zz),(-q[2],&ww)]);
        if dz.abs() > 1e-12*speed {
            let pp = poly_sum(&[(a,&poly_mul(&n,&n)),(b*dz,&poly_mul(&n,&ww)),(cc*dz*dz,&poly_mul(&ww,&ww)),(-dz*dz,&poly_mul(&xx,&xx))]);
            let t_of = |s: f64| poly(&n,s)/(poly(&ww,s)*dz);
            let at = |s: f64| add(p,scale(d,t_of(s)));
            for r in crate::util::isolate::roots(&crate::util::isolate::Poly(&pp),[0.,1.],width,&|s| surface.implicit(at(s)).abs() <= tol)? {
                out.push((t_of(r.at),r.touch));
            }
        } else {
            // a level line: the circles at its height, each met as a chord
            for r in crate::util::isolate::roots(&crate::util::isolate::Poly(&n),[0.,1.],width,&|_| false)? {
                let rho = poly(&xx,r.at)/poly(&ww,r.at);
                if let Meets::At(found) = line_chord(from,flat,rho.abs(),tol) { out.extend(found) }
            }
        }
    }
    Ok(Meets::At(out))
}

/// **A line against a surface of revolution whose meridian is a turned contact** (#780 C3,
/// `turned::Turned`: no conic, but a closed form in the face's slant `v` with its derivatives),
/// as `line_revolution` reads a conic's: the line's radius squared is `a t² + b t + c` at the
/// height `z(t) = z₀ + D t`, so where it climbs it meets the surface where
/// `G(v) = a N² + b D N + c D² − D² ρ(v)²`, `N = z(v) − z₀`, is nought, its parameter `N / D`
/// there; a level line meets the circles at the roots of `N`, each a chord. The roots are isolated
/// (`util::isolate::roots`) from `G` and `G'` enclosed over each stretch by the meridian's jets
/// read in outward-rounded intervals (`turned::Iv`): any meridian with bounded derivatives, not
/// only a polynomial one. `NoForm` where the surface's axis is not the meridian's.
fn line_turned(p: V,d: V,f: &super::geom::Frame,m: &crate::turned::Turned,surface: &Surface,tol: f64) -> Certified {
    use crate::turned::Jet;
    let g = &m.frame;
    if m.posed { return Err(Uncertified::NoForm) }
    if norm(crate::util::space::cross(g.z,f.z)) > 1e-12 || norm(crate::util::space::cross(crate::util::space::sub(f.o,g.o),g.z)) > tol {
        return Err(Uncertified::NoForm)
    }
    let (q,e) = (g.local(p),g.dir_local(d));
    let speed = norm(d).max(1e-300);
    let (a,b,cc) = (e[0]*e[0]+e[1]*e[1],2.*(q[0]*e[0]+q[1]*e[1]),q[0]*q[0]+q[1]*q[1]);
    let dz = e[2];
    // a line all but level is read neither way well: the general reading's
    if dz.abs() > 1e-12*speed && dz.abs() < 1e-7*speed { return Err(Uncertified::NoForm) }
    let [s0,s1] = m.span;
    let (s0,s1) = (s0.min(s1),s0.max(s1));
    let width = 1e-12*(s1-s0).max(1e-300);
    let mut out: Vec<(f64,bool)> = Vec::new();
    if dz.abs() > 1e-12*speed {
        // G of the meridian's jet, its radius read squared, in floats or in intervals
        fn big<T: crate::turned::Num>([r2,z]: [Jet<T>;2],(a,b,cc,dz,z0): (f64,f64,f64,f64,f64)) -> Jet<T> {
            let n = z-Jet::of(z0);
            (n*n).times(a)+n.times(b*dz)+Jet::of(cc*dz*dz)-r2.times(dz*dz)
        }
        let k = (a,b,cc,dz,q[2]);
        let jet = |s: f64| big(m.source.meridian2(Jet::at(s)),k);
        let t_of = |s: f64| (m.source.meridian2(Jet::at(s))[1].v-q[2])/dz;
        let at = |s: f64| add(p,scale(d,t_of(s)));
        let (jv,value) = (|s: f64| { let j = jet(s); (j.v,j.d) },|s: f64| big(m.source.meridian2(Jet::at(Iv::point(s))),k).v);
        let slope = |x: f64,y: f64| big(m.source.meridian2(Jet::at(Iv::new(x,y))),k).d;
        let g = crate::util::isolate::Parts {at:&jv,value:&value,slope:&slope};
        for r in crate::util::isolate::roots(&g,[s0,s1],width,&|s| surface.implicit(at(s)).abs() <= tol)? {
            out.push((t_of(r.at),r.touch));
        }
    } else {
        // a level line: the circles at its height, each met as a chord
        let (flat,from) = (g.dir([e[0],e[1],0.]),g.dir([q[0],q[1],0.]));
        let jet = |s: f64| m.source.meridian2(Jet::at(s))[1]-Jet::of(q[2]);
        let (jv,value) = (|s: f64| { let j = jet(s); (j.v,j.d) },|s: f64| m.source.meridian2(Jet::at(Iv::point(s)))[1].v-Iv::point(q[2]));
        let slope = |x: f64,y: f64| m.source.meridian2(Jet::at(Iv::new(x,y)))[1].d;
        let g = crate::util::isolate::Parts {at:&jv,value:&value,slope:&slope};
        for r in crate::util::isolate::roots(&g,[s0,s1],width,&|_| false)? {
            let rho = m.source.meridian(Jet::at(r.at))[0].v;
            if let Meets::At(found) = line_chord(from,flat,rho.abs(),tol) { out.extend(found) }
        }
    }
    Ok(Meets::At(out))
}

/// **A line against the cylinder a slid torus's contact sweeps** (#805, `slid::Slid` read as its
/// apparent contour, extruded along the slide): it meets the cylinder where the contour crosses the
/// plane holding the line and the slide (`Exact::crossings`, isolated with the contour's jets
/// bounded in outward intervals), its parameter there the line's at that point seen along the
/// slide. A line along the slide is a ruling or misses: `NoForm` where it stands within the
/// tolerance of the contour (`meets` has read a ruling as along it already), none where it stands
/// clear. `NoForm` for the contact itself, read as no contour.
fn line_slid(p: V,d: V,h: &crate::slid::Slid,surface: &Surface,tol: f64) -> Certified {
    use crate::turned::Exact;
    use crate::util::space::{cross,sub};
    if !h.contour { return Err(Uncertified::NoForm) }
    let w = h.slide();
    let flat = |v: V| sub(v,scale(w,dot(v,w)));
    let e = flat(d);
    let ee = dot(e,e);
    if ee <= 1e-24*dot(d,d) {
        let foot = h.d2(h.inverse(p)).0;
        return if norm(flat(sub(foot,p))) > tol { Ok(Meets::At(Vec::new())) } else { Err(Uncertified::NoForm) }
    }
    // the line's parameter at the contour's point `x`, seen along the slide
    let t_of = |s: f64| dot(flat(sub(h.d2(s).0,p)),e)/ee;
    let found = h.crossings(p,cross(w,scale(e,1./ee.sqrt())),&|s| surface.implicit(add(p,scale(d,t_of(s)))).abs() <= tol)?;
    Ok(Meets::At(found.into_iter().map(|r| (t_of(r.at),r.touch)).collect()))
}

/// **`curve_surface` by the general reading alone** (`General`), whatever closed form the pair has,
/// settled as every reading is: what each closed form is held to and measured against (#777 M3's
/// fast paths, #814 P4).
pub fn curve_surface_searched(curve: &Curve,t: [f64;2],surface: &Surface,tol: f64) -> Meets { curve_surface_by(&General,curve,t,surface,tol) }

/// **The general reading** (#814 P2): where any curve meets any surface, its roots proved by the
/// Krawczyk operator over the two kinds' enclosures (`enclose`) — no bound written by hand for the
/// pair. A surface with an algebraic form (`algebraic`: a plane, a quadric, a torus) is read along
/// the curve, one unknown: `g(t) = F(C(t))`, its rate `∇F(C(t)) · C'(t)` enclosed over a stretch from
/// the curve's enclosure (`Implicit`), a touch accepted where the surface's distance is within
/// `tol`, each root kept where the surface's points are (a cone's one sheet). Any other is read as
/// the curve's point and the surface's equal, `C(t) = S(u, v)`, three unknowns over the curve's
/// stretch and the parameters the surface reaches (`Meeting`), a crossing a root the operator
/// certifies and a touch a root no box can part from its neighbours, its point on both to `tol`.
/// A stretch with no end is the curve's within the surface's own box where it is a line and the
/// surface is bounded, and refused otherwise; a search that proves nothing is refused, with its box.
fn general(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Result<(Meets,bool),Uncertified> {
    let refuse = |g: GaveUp| Uncertified::Refused(Refusal::of(curve,[t0,t1],surface,g));
    let Some([a,b]) = bounded_stretch(curve,[t0,t1],surface,tol) else {
        return Err(Uncertified::Refused(Refusal::unbounded(curve,[t0,t1],surface)))
    };
    if !(b > a) { return Ok((Meets::At(Vec::new()),true)) }
    let whole = curve.enclose([[a,b]]);
    let proved = whole.proved;
    if let Some((_,_,keep)) = algebraic(surface,tol) {
        let g = Implicit {curve,surface};
        let speed = norm(whole.d[0].map(|x| x.mag()));
        let speed = if speed.is_finite() { speed } else { curve.speed() };
        let found = crate::util::isolate::roots(&g,[a,b],tol*1e-3/speed.max(1e-300),&|t| surface.implicit(curve.point(t)).abs() <= tol)
            .map_err(refuse)?;
        return Ok((Meets::At(found.into_iter().filter(|r| keep(curve.point(r.at))).map(|r| (r.at,r.touch)).collect()),proved))
    }
    let Some([pu,pv]) = reach_of(surface,&whole) else { return Err(Uncertified::Refused(Refusal::unbounded(curve,[t0,t1],surface))) };
    let sys = Meeting {curve,surface,range:[[a,b],pu,pv]};
    // the resolution a touch is read to: the tolerance over the farthest any unknown carries a
    // point across the box (read at its middle and corners)
    let reach = (0..9).map(|k| {
        let x = if k == 8 { [0.5;3] } else { [(k&1) as f64,((k>>1)&1) as f64,((k>>2)&1) as f64] };
        let (_,j) = crate::util::isolate::System::at(&sys,x);
        (0..3).map(|c| norm([j[0][c],j[1][c],j[2][c]])).fold(0.,f64::max)
    }).fold(0.,f64::max);
    let width = tol*1e-2/reach.max(1e-300);
    let near = |x: [f64;3]| { let (fx,_) = crate::util::isolate::System::at(&sys,x); norm(fx) <= tol };
    let found = crate::util::isolate::solve::<3>(&sys,[[0.,1.];3],width,&near).map_err(refuse)?;
    Ok((Meets::At(found.into_iter().map(|r| (sys.natural(0,r.at[0].clamp(0.,1.)),r.touch)).collect()),proved))
}

/// A surface's algebraic form and its gradient at `x` (`algebraic`'s, in its frame's coordinates,
/// the gradient carried back to space), written once over `Num`: read in floats, or in intervals
/// over a box. `None` for a surface with none.
fn form<T: Num>(surface: &Surface,x: [T;3]) -> Option<(T,[T;3])> {
    use crate::util::fmath::Det;
    let f = surface.frame();
    let c = |v: f64| T::of(v);
    let d: [T;3] = std::array::from_fn(|i| x[i]-c(f.o[i]));
    let along = |e: V| d[0]*c(e[0])+d[1]*c(e[1])+d[2]*c(e[2]);
    let q = [along(f.x),along(f.y),along(f.z)];
    let two = c(2.);
    let (value,g): (T,[T;3]) = match *surface {
        Surface::Plane(_) => (q[2],[c(0.),c(0.),c(1.)]),
        Surface::Cylinder(_,r) => (q[0]*q[0]+q[1]*q[1]-c(r*r),[two*q[0],two*q[1],c(0.)]),
        Surface::Sphere(_,r) => (q[0]*q[0]+q[1]*q[1]+q[2]*q[2]-c(r*r),[two*q[0],two*q[1],two*q[2]]),
        Surface::Cone(_,r,a) => {
            let (sa,ca) = a.dsin_cos();
            if !(ca.abs() > 1e-9) { return None }
            let k = sa/ca;
            let w = c(r)+q[2]*c(k);
            (q[0]*q[0]+q[1]*q[1]-w*w,[two*q[0],two*q[1],-(two*w*c(k))])
        }
        Surface::Torus(_,big,r) => {
            let (big,r) = (big.abs(),r.abs());
            let flat = q[0]*q[0]+q[1]*q[1];
            let s = flat+q[2]*q[2]+c(big*big-r*r);
            let four = c(4.);
            let k = four*s-c(8.*big*big);
            (s*s-four*c(big*big)*flat,[k*q[0],k*q[1],four*s*q[2]])
        }
        _ => return None,
    };
    Some((value,std::array::from_fn(|i| g[0]*c(f.x[i])+g[1]*c(f.y[i])+g[2]*c(f.z[i]))))
}

/// **A surface's algebraic form along a curve** (`general`'s one unknown): `F(C(t))` and its rate,
/// enclosed from the curve's enclosure (`enclose`) and the form read in intervals (`form`).
struct Implicit<'a> { curve: &'a Curve,surface: &'a Surface }

impl crate::util::isolate::Function for Implicit<'_> {
    fn at(&self,t: f64) -> (f64,f64) {
        let (x,d) = self.curve.d1(t);
        form(self.surface,x).map_or((f64::NAN,f64::NAN),|(f,g)| (f,dot(g,d)))
    }
    fn value(&self,t: f64) -> Iv { form(self.surface,self.curve.enclose([[t,t]]).at).map_or(Iv::ANY,|(f,_)| f) }
    fn slope(&self,a: f64,b: f64) -> Iv {
        let e = self.curve.enclose([[a,b]]);
        form(self.surface,e.at).map_or(Iv::ANY,|(_,g)| crate::enclose::dot3(g,e.d[0]))
    }
}

/// **A curve's point and a surface's equal** (`general`'s three unknowns): `C(t) − S(u, v)` over
/// the stretches `range` (the curve's, then the surface's), each unknown read from nought to one
/// across its own, the Jacobian `[C', −S_u, −S_v]` scaled by them, enclosed from the two kinds'
/// enclosures (`enclose`).
struct Meeting<'a> { curve: &'a Curve,surface: &'a Surface,range: [[f64;2];3] }

impl Meeting<'_> {
    /// Unknown `k`'s own parameter at `x` across its stretch.
    fn natural(&self,k: usize,x: f64) -> f64 { let [a,b] = self.range[k]; a+x*(b-a) }
    /// The parameters a box of the unknowns reaches, outward.
    fn natural_box(&self,bx: &[[f64;2];3]) -> [[f64;2];3] {
        std::array::from_fn(|k| {
            let [a,b] = self.range[k];
            let x = Iv::point(a)+Iv::new(bx[k][0],bx[k][1])*(Iv::point(b)-Iv::point(a));
            [x.lo().max(a.min(b)),x.hi().min(a.max(b))]
        })
    }
    /// Each unknown's stretch's length, enclosed.
    fn widths(&self) -> [Iv;3] { std::array::from_fn(|k| Iv::point(self.range[k][1])-Iv::point(self.range[k][0])) }
}

impl crate::util::isolate::System<3> for Meeting<'_> {
    fn at(&self,x: [f64;3]) -> ([f64;3],[[f64;3];3]) {
        let (t,u,v) = (self.natural(0,x[0]),self.natural(1,x[1]),self.natural(2,x[2]));
        let (q,dq) = self.curve.d1(t);
        let (s,su,sv) = self.surface.d1([u,v]);
        let w: [f64;3] = std::array::from_fn(|k| self.range[k][1]-self.range[k][0]);
        (std::array::from_fn(|i| q[i]-s[i]),std::array::from_fn(|i| [dq[i]*w[0],-su[i]*w[1],-sv[i]*w[2]]))
    }
    fn value(&self,x: [f64;3]) -> [Iv;3] {
        let n = self.natural_box(&x.map(|y| [y,y]));
        let (c,s) = (self.curve.enclose([n[0]]),self.surface.enclose([n[1],n[2]]));
        crate::enclose::sub3(c.at,s.at)
    }
    fn jacobian(&self,bx: &[[f64;2];3]) -> [[Iv;3];3] {
        let n = self.natural_box(bx);
        let (c,s) = (self.curve.enclose([n[0]]),self.surface.enclose([n[1],n[2]]));
        let w = self.widths();
        std::array::from_fn(|i| [c.d[0][i]*w[0],-(s.d[0][i]*w[1]),-(s.d[1][i]*w[2])])
    }
}

/// **The stretch a reading searches** (`general`): `[t0, t1]` where it is bounded; a line's with no
/// end, the stretch of it within the surface's own box (its enclosure over all its parameters,
/// grown by the tolerance), where the surface is bounded. `None` where neither.
fn bounded_stretch(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Option<[f64;2]> {
    if t0.is_finite() && t1.is_finite() { return Some([t0,t1]) }
    let &Curve::Line {p,d} = curve else { return None };
    let dom = domain(surface)?;
    let e = surface.enclose(dom);
    let (mut a,mut z) = (t0,t1);
    for k in 0..3 {
        let (lo,hi) = (e.at[k].lo()-tol,e.at[k].hi()+tol);
        if !(lo.is_finite() && hi.is_finite()) { return None }
        if d[k].abs() < 1e-300 { if p[k] < lo || p[k] > hi { return Some([0.,0.]) } continue }
        let (u,v) = ((lo-p[k])/d[k],(hi-p[k])/d[k]);
        a = a.max(u.min(v));
        z = z.min(u.max(v));
    }
    Some(if a < z { [a,z] } else { [0.,0.] })
}

/// **The parameters a surface runs over**, where they are bounded: a ball's and a torus's angles, a
/// surface of revolution's turn and its meridian's stretch, a blend's square, a B-spline's domain, a
/// family's run and interval, an affine image's preimage's. `None` for an unbounded surface (a
/// plane, a cylinder, a cone, an extrusion).
pub fn domain(surface: &Surface) -> Option<[[f64;2];2]> {
    use std::f64::consts::{FRAC_PI_2,TAU};
    match surface {
        Surface::Sphere(..) => Some([[0.,TAU],[-FRAC_PI_2,FRAC_PI_2]]),
        Surface::Torus(..) => Some([[0.,TAU],[0.,TAU]]),
        Surface::Revolution(_,c) => Some([[0.,TAU],curve_domain(c)?]),
        Surface::Blend(..) => Some([[0.,1.],[0.,1.]]),
        Surface::BSpline(_,n) => Some(n.domain()),
        Surface::Family(_,w) => Some([[0.,1.],w.span()]),
        Surface::Affine(m) => domain(&m.of),
        Surface::Plane(_) | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Extrusion(..) => None,
    }
}

/// **The stretch of its parameter a curve runs over**, where it is bounded: a conic's turn, a
/// B-spline's domain, a trace's points, a rail's and a rim's, a sheet's cut's, an image's
/// preimage's carried over, an exact curve's span. `None` for a line.
pub fn curve_domain(c: &Curve) -> Option<[f64;2]> {
    use std::f64::consts::TAU;
    Some(match c {
        Curve::Line {..} => return None,
        Curve::Circle(..) | Curve::Ellipse(..) => [0.,TAU],
        Curve::BSpline(b) => b.domain(),
        Curve::Traced(c) => [0.,(c.pts.len()-1) as f64],
        Curve::Iso(..) | Curve::SheetCut(..) | Curve::Rim(..) => [0.,1.],
        Curve::Rail(w,_) => w.span(),
        Curve::Affine(m) => { let [a,b] = curve_domain(&m.of)?; let (x,y) = (m.at[0]*a+m.at[1],m.at[0]*b+m.at[1]); [x.min(y),x.max(y)] }
        Curve::Turned(m) => { let [a,b] = m.span; [a.min(b),a.max(b)] }
        Curve::Slid(m) => { use crate::turned::Exact; let [a,b] = m.span(); [a.min(b),a.max(b)] }
    })
}

/// **The parameters of a surface a curve can meet it at** (`general`'s three unknowns): its own,
/// where they are bounded (`domain`); an extrusion's, its profile's stretch (a line's, the reach of
/// the curve's box along it) and the heights along its way the curve's box reaches past the
/// profile's. `None` where neither is bounded.
fn reach_of(surface: &Surface,curve: &crate::enclose::Patch<1>) -> Option<[[f64;2];2]> {
    if let Some(d) = domain(surface) { return Some(d) }
    let Surface::Extrusion(f,c) = surface else { return None };
    let along = |x: I3,e: V| crate::enclose::dot3(x,crate::enclose::c3(e));
    let u = match curve_domain(c) {
        Some(u) => u,
        None => {
            let &Curve::Line {p,d} = &**c else { return None };
            let r = along(crate::enclose::sub3(curve.at,crate::enclose::c3(p)),d)*(Iv::point(1.)/Iv::point(crate::util::space::dot(d,d)));
            [r.lo(),r.hi()]
        }
    };
    let profile = c.enclose([u]);
    let v = along(curve.at,f.z)-along(profile.at,f.z);
    let v = [v.lo(),v.hi()];
    (u.iter().chain(&v).all(|x| x.is_finite()) && u[0] <= u[1] && v[0] <= v[1]).then_some([u,v])
}

/// How many steps `along` reads a curve at over `[t0, t1]` against `surface`: a fraction of the
/// surface's feature apart, and a closed curve's at least 64 a turn.
fn samples(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface) -> usize {
    let length = (t1-t0)*curve.speed();
    let (lo,hi) = curve.bounds([t0,t1]);
    ((length/surface.feature_within(lo,hi)*16.).ceil() as usize)
        .max(if curve.period().is_some() { ((t1-t0)/(std::f64::consts::PI/32.)).ceil() as usize } else { 1 })
        .clamp(16,1<<14)
}

/// A line through `p` along the unit `d` against the cylinder of radius `r` about
/// `c`'s axis, in closed form: across the axis it is a chord of the circle (`line_chord`). A line
/// along the axis lies in the surface or misses it.
fn line_cylinder(p: V,d: V,c: &super::geom::Frame,r: f64,tol: f64) -> Meets {
    use crate::util::space::sub;
    let across = |v: V| sub(v,scale(c.z,dot(v,c.z)));
    let (w,e) = (across(sub(p,c.o)),across(d));
    if dot(e,e) <= 1e-24 {
        return if (norm(w)-r).abs() <= tol { Meets::Along } else { Meets::At(Vec::new()) }
    }
    line_chord(w,e,r,tol)
}

/// A line through `p` along `d` against the sphere of radius `r` about `o`, in
/// closed form: a chord of the great circle in the plane through it and the centre (`line_chord`).
fn line_sphere(p: V,d: V,o: V,r: f64,tol: f64) -> Meets { line_chord(crate::util::space::sub(p,o),d,r,tol) }

/// The line `w + t e` (`w` from a centre) against the circle of radius `r` about that centre in the
/// plane through them: a chord's two roots, or one touch exactly at the foot where it is tangent
/// (`plane2::chord`), whatever stretch is asked (what lies past it `settled` reads). Exact, where a
/// sampled search places a touch only to its distance's rate's rounding, and over one evaluation,
/// not thousands.
fn line_chord(w: V,e: V,r: f64,tol: f64) -> Meets {
    let ee = dot(e,e);
    if ee <= 1e-24 { return Meets::At(Vec::new()) }
    let foot = -dot(w,e)/ee;
    let roots = match chord(norm(add(w,scale(e,foot))),r,Touch::Gap(tol)) {
        Chord::Miss => Vec::new(),
        Chord::Touch => vec![(foot,true)],
        Chord::Cross(half) => { let half = half/ee.sqrt(); vec![(foot-half,false),(foot+half,false)] }
    };
    Meets::At(roots)
}

/// **A line through `p` along the unit `d`, over `[t0, t1]`, against the sheet a level circle screws
/// into** (`geom::LevelCircle`, a round rim under a screw, #721 F2c5), its roots isolated, not
/// sampled. The circle level with the line's point `q(t)` is at the `v` its height gives, linear
/// in `t`; screwed back from there, `q(t)` lies in the circle's plane, at `A(t)` from its centre,
/// and the line meets the sheet where `G(t) = |A(t)|² − r² = 0`. Across the axis `A(t) = R(−θ v(t))
/// (w + e t) − c⊥`, a line turned at the constant rate the rise gives: written once over `Num`
/// (`Screwed`), so `G` and `G'` are enclosed over a stretch by the same formula in intervals and
/// its roots isolated by them (`util::isolate`, #814). Square to the axis the line is level and
/// meets the one circle there as a chord. A thin sheet's two crossings a hair apart are both
/// found: no step need land between them.
fn line_screwed_circle(p: V,d: V,[t0,t1]: [f64;2],c: &super::geom::LevelCircle,tol: f64) -> Certified {
    use crate::util::space::sub;
    let k = c.axis;
    let perp = |x: V| sub(x,scale(k,dot(x,k)));
    let (v0,v1) = (c.v_at(p),dot(d,k)/c.rise);
    // the stretch of the line level with the sheet, `v` in `[0, 1]`
    if v1.abs()*(t1-t0).abs() <= 1e-15 {
        if !(0.0..=1.0).contains(&v0) { return Ok(Meets::At(Vec::new())) }
        let o = c.back(c.frame.o,-v0);
        return Ok(line_chord(sub(p,o),perp(d),c.r,tol))
    }
    // (the sheet's ends held a tolerance wide: a line ending on an end circle meets it there)
    let slack = tol/(c.rise.abs()*v1.abs().max(1e-300)*norm(d).max(1e-300)).max(1e-300)*v1.abs();
    let (ta,tb) = ((0.-v0-slack)/v1,(1.-v0+slack)/v1);
    let (lo,hi) = (ta.min(tb).max(t0),ta.max(tb).min(t1));
    if lo > hi { return Ok(Meets::At(Vec::new())) }
    let g = crate::util::isolate::Written(Screwed {
        w:perp(sub(p,c.centre)),e:perp(d),cp:perp(sub(c.frame.o,c.centre)),k,turn:[-c.angle*v0,-c.angle*v1],r:c.r,
    });
    let gv = |t: f64| { use crate::util::isolate::Function; g.at(t).0 };
    let step = tol*1e-3/norm(d).max(1e-300);
    let mut roots: Vec<(f64,bool)> = crate::util::isolate::roots(&g,[lo,hi],step,&|m| gv(m).abs() <= 2.*c.r*tol)?
        .into_iter().map(|r| (r.at,r.touch)).collect();
    // where the line crosses the level of the sheet's end circle on it, a root there (the line's
    // own ends `settled` reads)
    for end in [lo,hi].into_iter().filter(|&e| e != t0 && e != t1) {
        let near = roots.iter().any(|&(t,_)| (t-end).abs()*norm(d) <= tol);
        if !near && gv(end).abs() <= 2.*c.r*tol { roots.push((end,false)) }
    }
    Ok(Meets::At(roots))
}

/// `|R(θ(t)) (w + e t) − c|² − r²`, `θ(t) = turn₀ + turn₁ t` about the unit `k`, `w`, `e` and `c`
/// square to it (`line_screwed_circle`), written once over `Num`.
struct Screwed { w: V,e: V,cp: V,k: V,turn: [f64;2],r: f64 }

impl crate::util::isolate::Formula for Screwed {
    fn jet<T: Num>(&self,t: crate::util::interval::Jet<T>) -> crate::util::interval::Jet<T> {
        use crate::util::interval::Jet;
        use crate::util::space::cross;
        let (s,c) = (t.times(self.turn[1])+Jet::of(self.turn[0])).sin_cos();
        // `R(θ) x = x cos θ + (k × x) sin θ` for `x` square to `k`, `x = w + e t`
        let (kw,ke) = (cross(self.k,self.w),cross(self.k,self.e));
        let a: [Jet<T>;3] = std::array::from_fn(|i| {
            let x = Jet::of(self.w[i])+t.times(self.e[i]);
            let kx = Jet::of(kw[i])+t.times(ke[i]);
            x*c+kx*s-Jet::of(self.cp[i])
        });
        a[0]*a[0]+a[1]*a[1]+a[2]*a[2]-Jet::of(self.r*self.r)
    }
}

/// **A circle about a screw's axis against the sheet a circle screws into**, exactly (#721 F2c5: a
/// cylinder's rim about the axis against a ball's pipe): the screw keeps each point's distance from
/// its axis, so the sheet meets the circle's cylinder in the helices of its circle's points at that
/// radius (`ssi`'s `screwed_circle_round`), and the circle where each helix reaches its height
/// within the sheet. Where the helix is one the sheet touches the cylinder along, the circle,
/// lying in the cylinder, touches the sheet there. `NoForm` for a circle not about the screw's axis,
/// or a sheet no circle screws into.
fn circle_round_screwed(f: &super::geom::Frame,big: f64,b: &std::sync::Arc<super::geom::Blend>,[t0,t1]: [f64;2],tol: f64) -> Certified {
    use crate::util::space::{cross,sub};
    let super::geom::Carry::Screw {center,axis,angle,advance} = b.carry else { return Err(Uncertified::NoForm) };
    let k = crate::util::space::normalised(axis).ok_or(Uncertified::NoForm)?;
    if norm(cross(f.z,k)) > 1e-12 { return Err(Uncertified::NoForm) }
    if norm(sub(sub(f.o,center),scale(k,dot(sub(f.o,center),k)))) > tol || angle == 0. { return Err(Uncertified::NoForm) }
    let rise = advance*dot(axis,k);
    if rise == 0. { return Err(Uncertified::NoForm) }
    let h = dot(sub(f.o,center),k);
    let mut roots = Vec::new();
    for u in screw_helices(b,big,tol)? {
        let v = (h-dot(sub(b.d1([u,0.]).0,center),k))/rise;
        let slack = tol/rise.abs();
        if !(-slack..=1.+slack).contains(&v) { continue }
        let (p,touch) = on_screw_helix(b,u,v);
        // the circle's parameter at the point, within its stretch
        let q = sub(p,f.o);
        let mut t = dot(q,f.y).datan2(dot(q,f.x));
        while t < t0-1e-12 { t += std::f64::consts::TAU }
        while t > t1+1e-12 { t -= std::f64::consts::TAU }
        if t < t0-1e-12 || t > t1+1e-12 { continue }
        roots.push((t.clamp(t0,t1),touch));
    }
    Ok(Meets::At(roots))
}

/// **Where the sheet a circle screws into meets the cylinder of radius `big` about the screw's
/// axis**: the `u` of each helix `ssi::screwed_circle_round` reads there (a touch read once, placed
/// where the circle's distance from the axis turns), what a circle about the axis
/// (`circle_round_screwed`) and a line along it (`line_along_screw`) are crossed by. `NoForm` for a
/// sheet no circle screws into.
fn screw_helices(b: &std::sync::Arc<super::geom::Blend>,big: f64,tol: f64) -> Result<Vec<f64>,Uncertified> {
    let super::geom::Carry::Screw {center,axis,..} = b.carry else { return Err(Uncertified::NoForm) };
    let k = crate::util::space::normalised(axis).ok_or(Uncertified::NoForm)?;
    let helices = super::ssi::screwed_circle_round(b,&super::geom::Frame::about(center,k),big,tol).ok_or(Uncertified::NoForm)?;
    Ok(helices.into_iter().filter_map(|c| if let Curve::Iso(_,u) = c { Some(u) } else { None }).collect())
}

/// A screwed sheet's point on the helix `u` at `v` (clamped to the sheet), and whether the sheet
/// touches the cylinder about the axis there: on a helix it touches that cylinder along, its normal
/// is the cylinder's own.
fn on_screw_helix(b: &super::geom::Blend,u: f64,v: f64) -> (V,bool) {
    use crate::util::space::{cross,normalised,sub};
    let super::geom::Carry::Screw {center,axis,..} = b.carry else { unreachable!("a screwed sheet") };
    let (p,su,sv) = b.d1([u,v.clamp(0.,1.)]);
    let off = sub(p,center);
    let radial = normalised(sub(off,scale(axis,dot(off,axis)/dot(axis,axis)))).unwrap_or([0.;3]);
    (p,norm(cross(normalised(cross(su,sv)).unwrap_or([0.;3]),radial)) <= 1e-6)
}

/// **A line along a screw's axis against the sheet a circle screws into** (#791), in closed form:
/// the screw keeps a point's distance from its axis, so the line meets the sheet only on the
/// cylinder about the axis through it, along the helices `ssi::screwed_circle_round` reads there
/// (the circle's points at the line's radius, carried), and each helix crosses the line where it
/// has turned round to the line's angle, once a turn: `v = (2πn − φ)/θ`, `φ` its start's angle from
/// the line's and `θ` the screw's turn. A touch helix is read once and placed where the circle's
/// distance from the axis turns, so a line tangent to the sheet there — the seam of the cylinder a
/// ball's pipe touches along that helix — meets it at the helix's own point whatever the rounding,
/// the vertex the cylinder's meeting with the sheet passes through. Searched (`curve_sheet`), that
/// double root was found at one lead and missed at another, and the helix, crossing the seam
/// uncut, split nothing (the ball-end mill at 4 mm a turn,
/// `swept_closed::the_drawn_ball_end_mills_pipe_beside_its_screwed_side_is_arranged_at_every_lead`).
/// A touch where the sheet's normal there is the cylinder's. `NoForm` for any other line or sheet.
fn line_along_screw(p: V,d: V,[t0,t1]: [f64;2],b: &std::sync::Arc<super::geom::Blend>,tol: f64) -> Certified {
    use crate::util::space::{cross,normalised,sub};
    let super::geom::Carry::Screw {center,axis,angle,..} = b.carry else { return Err(Uncertified::NoForm) };
    let k = normalised(axis).ok_or(Uncertified::NoForm)?;
    let dd = norm(d);
    if !(dd > 0.) || norm(cross(d,k)) > 1e-12*dd || angle == 0. { return Err(Uncertified::NoForm) }
    let off = sub(sub(p,center),scale(k,dot(sub(p,center),k)));
    let big = norm(off);
    if big <= tol { return Err(Uncertified::NoForm) }
    // each helix's start's angle about the axis, from the line's
    let around = super::geom::Frame::new(center,k,off);
    // the turns `θ v` over the sheet, the tolerance's worth past each end
    let slack = tol/(angle.abs()*big);
    let (lo,hi) = { let (a,z) = (angle*-slack,angle*(1.+slack)); (a.min(z),a.max(z)) };
    let tau = std::f64::consts::TAU;
    let mut roots = Vec::new();
    for u in screw_helices(b,big,tol)? {
        let at = around.local(b.d1([u,0.]).0);
        let phi = at[1].datan2(at[0]);
        for n in ((lo+phi)/tau).ceil() as i64..=((hi+phi)/tau).floor() as i64 {
            let (q,touch) = on_screw_helix(b,u,(tau*n as f64-phi)/angle);
            let t = dot(sub(q,p),d)/(dd*dd);
            if t >= t0-tol/dd && t <= t1+tol/dd { roots.push((t,touch)) }
        }
    }
    Ok(Meets::At(roots))
}

/// **A helix against a ball**, its roots isolated (#721 F2c5: the helix a ball's pipe touches a
/// cylinder along, which starts on the ball at its start pose, touching it there): the iso-line of
/// a sheet a curve screws into, where the curve's two ends are one, is its point at `u` carried by
/// the screw, `p(v)`, and it meets the ball where `G(v) = |p(v) − c|² − r² = 0`, written once over
/// `Num` (`Helical`) and isolated by its enclosures (`util::isolate`, #814): a stretch dropped where
/// `G` cannot reach nought across it, read as one root where it is monotone, halved otherwise, a
/// touch where it only reaches nought.
fn helix_sphere(b: &super::geom::Blend,u: f64,c: V,r: f64,[t0,t1]: [f64;2],tol: f64) -> Certified {
    use crate::util::space::sub;
    let super::geom::Carry::Screw {center,axis,angle,advance} = b.carry else { return Err(Uncertified::NoForm) };
    let p0 = b.d1([u,0.]).0;
    let same = |q: V,p: V| distance(q,p) <= 1e-9*(1.+norm(p));
    if !same(b.d1([u,1.]).0,b.carry.at(p0,1.)) || !same(b.d1([u,0.5]).0,b.carry.at(p0,0.5)) { return Err(Uncertified::NoForm) }
    let k = crate::util::space::normalised(axis).ok_or(Uncertified::NoForm)?;
    let rho = norm(sub(sub(p0,center),scale(k,dot(sub(p0,center),k))));
    let (a,h) = (angle.abs(),(advance*norm(axis)).abs());
    let g = crate::util::isolate::Written(Helical {carry:b.carry,p0,c,r});
    let gv = |v: f64| { use crate::util::isolate::Function; g.at(v).0 };
    let step = tol*1e-3/(a*rho+h).max(1e-300);
    // (an end on the ball, the touch beside it there taken in, `settled` reads)
    let roots = crate::util::isolate::roots(&g,[t0,t1],step,&|m| gv(m).abs() <= 2.*r*tol)?;
    Ok(Meets::At(roots.into_iter().map(|x| (x.at,x.touch)).collect()))
}

/// `|p(v) − c|² − r²`, `p(v)` the point `p₀` carried to `v` (`helix_sphere`), written once over
/// `Num`.
struct Helical { carry: super::geom::Carry,p0: V,c: V,r: f64 }

impl crate::util::isolate::Formula for Helical {
    fn jet<T: Num>(&self,v: crate::util::interval::Jet<T>) -> crate::util::interval::Jet<T> {
        use crate::util::interval::Jet;
        let p = self.carry.jet(self.p0,v);
        let d: [Jet<T>;3] = std::array::from_fn(|i| p[i]-Jet::of(self.c[i]));
        d[0]*d[0]+d[1]*d[1]+d[2]*d[2]-Jet::of(self.r*self.r)
    }
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

/// How far box `(lo, hi)` is from `p`: 0 inside it.
fn gap((lo,hi): ([f64;3],[f64;3]),p: V) -> f64 {
    norm(std::array::from_fn(|k| (lo[k]-p[k]).max(p[k]-hi[k]).max(0.)))
}

/// A box about edge `e`: its curve's enclosure over its stretch (`Curve::bounds`), a degenerate
/// edge its pole, grown by the edge's tolerance and a hair of `size`, the solid's (`Brep::size`,
/// which walks every edge and face, so read once for all of them: #589).
pub(crate) fn edge_box(b: &Brep,e: usize,size: f64) -> ([f64;3],[f64;3]) {
    let edge = &b.edges[e];
    let (lo,hi) = match &edge.curve {
        EdgeCurve::Curve(c) => c.bounds(edge.t),
        EdgeCurve::Degenerate => { let p = b.vertices[edge.v[0] as usize].p; (p,p) }
    };
    let pad = edge.tol+1e-9*size.max(1.);
    (lo.map(|x| x-pad),hi.map(|x| x+pad))
}

/// `curve_surface` over only the stretches of `[t0, t1]` where the curve comes within `grow` of the
/// box `bx` (a face's box in space): a crossing of a face is in its box.  A line is clipped to the
/// box exactly; any other curve by its enclosure (`enclose`) over each of 64 pieces of the stretch,
/// a piece kept where its enclosure reaches the box (#814: never by samples, which a curve can
/// leave the box between).  A stretch lying along the surface is asked of the whole again; a
/// stretch refused refuses the whole.
pub fn curve_surface_within(curve: &Curve,t: [f64;2],surface: &Surface,tol: f64,bx: ([f64;3],[f64;3]),grow: f64) -> Meets {
    within(curve,t,surface,tol,bx,grow)
}

/// `curve_surface_within`.
fn within(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64,bx: ([f64;3],[f64;3]),grow: f64) -> Meets {
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
        _ if !(t0.is_finite() && t1.is_finite()) => stretches.push([t0,t1]),
        _ => {
            const N: usize = 64;
            let ts: Vec<f64> = (0..=N).map(|i| t0+(t1-t0)*i as f64/N as f64).collect();
            let near: Vec<bool> = ts.windows(2).map(|w| {
                let e = curve.enclose([[w[0],w[1]]]).at;
                (0..3).all(|k| !(e[k].hi() < lo[k] || e[k].lo() > hi[k]))
            }).collect();
            let mut i = 0;
            while i < N {
                if !near[i] { i += 1; continue }
                let start = i;
                while i < N && near[i] { i += 1; }
                stretches.push([ts[start],ts[i]]);
            }
        }
    }
    if stretches.len() == 1 && stretches[0] == [t0,t1] { return meets(&Fast,curve,[t0,t1],surface,tol) }
    let mut roots = Vec::new();
    for s in stretches {
        match read(&Fast,curve,s,surface,tol) {
            Ok(None) => return meets(&Fast,curve,[t0,t1],surface,tol),
            Ok(Some(r)) => roots.extend(r),
            Err(r) => return Meets::Refused(r),
        }
    }
    // the stretches' roots settled once, over the whole (a root two overlapping stretches both
    // found is one, and a stretch's end cut at the box is no end of the curve)
    Meets::At(settled(curve,[t0,t1],surface,tol,roots))
}

/// A face's loops drawn in its parameters as closed polygons: each edge's run along a loop cut in
/// chords until every chord's middle, carried onto the surface, lies within a bar of the edge in
/// space (at least four chords an edge, at most 2²⁰). **The one drawing of a boundary a point is
/// read against** (`Located`, and `boolean`'s pieces and holes): it carries `sag`, how far its
/// chords' middles stray from the edges at most, so that whatever reads its winding knows how
/// near the edges it may not be trusted (sampled at the middles, as `ssi`'s traces are: the
/// chords are short enough that a middle's is a chord's).
#[derive(Clone,Debug)]
pub struct Outline { loops: Vec<Vec<Uv>>,sag: f64,bins: std::sync::OnceLock<Bins> }

/// An outline's chords sorted into bands of `v` (`Outline::winding`): each band the chords whose
/// `v` reaches into it, by loop and index.
#[derive(Clone,Debug,Default)]
struct Bins { lo: f64,hi: f64,step: f64,bands: Vec<Vec<(u32,u32)>> }

impl Bins {
    /// About four chords a band, at most `MAX_BANDS` bands.
    fn of(loops: &[Vec<Uv>]) -> Bins {
        const MAX_BANDS: usize = 1024;
        let (mut lo,mut hi) = (f64::INFINITY,f64::NEG_INFINITY);
        for p in loops.iter().flatten() { lo = lo.min(p[1]); hi = hi.max(p[1]); }
        let chords: usize = loops.iter().map(Vec::len).sum();
        let n = (chords/4).clamp(1,MAX_BANDS);
        if !(hi > lo) { return Bins {lo,hi,step:1.,bands:vec![Vec::new();1]}; }
        let mut bins = Bins {lo,hi,step:(hi-lo)/n as f64,bands:vec![Vec::new();n]};
        for (k,l) in loops.iter().enumerate() {
            for i in 0..l.len() {
                let (a,b) = (l[i][1],l[(i+1)%l.len()][1]);
                for band in bins.band(a.min(b))..=bins.band(a.max(b)) { bins.bands[band].push((k as u32,i as u32)); }
            }
        }
        bins
    }
    /// The band `v` falls in.
    fn band(&self,v: f64) -> usize { (((v-self.lo)/self.step).floor().max(0.) as usize).min(self.bands.len()-1) }
}

impl Outline {
    /// The loops of `surface`'s face drawn to `bar`: each loop its runs in turn, each run its
    /// parameters and its point in space at a fraction of the way along it, from where the last
    /// run ended to where the next begins.
    pub fn of<F: Fn(f64) -> (Uv,V)>(surface: &Surface,bar: f64,loops: impl IntoIterator<Item=impl IntoIterator<Item=F>>) -> Outline {
        let mut sag = 0f64;
        let mut stack = Vec::new();
        let loops = loops.into_iter().map(|l| {
            let mut pts = Vec::new();
            for run in l {
                // each chord its ends' fractions and parameters, so a point is read once: the
                // middle a chord is cut at is the end of both halves
                let (u0,u1) = (run(0.).0,run(1.).0);
                pts.push(u0);
                stack.push((0.,u0,1.,u1,0));
                // bisected the first way round first, so the chords come in order
                let mut deepest = 0;
                while let Some((a,ua,z,uz,depth)) = stack.pop() {
                    let m = (a+z)/2.;
                    let (um,pm) = run(m);
                    let off = distance(surface.point([(ua[0]+uz[0])/2.,(ua[1]+uz[1])/2.]),pm);
                    // each chord cut counted (#777 V3): a stopped build draws no finer
                    let going = crate::util::meter::charge(crate::util::meter::Stage::Outline,1,|| (pm,surface.kind().to_string()));
                    if depth < 2 || (off > bar && depth < 20 && going) { stack.push((m,um,z,uz,depth+1)); stack.push((a,ua,m,um,depth+1)); }
                    else { sag = sag.max(off); deepest = deepest.max(depth); if z < 1. { pts.push(uz); } }
                }
                crate::util::meter::outline_depth(deepest);
            }
            pts
        }).collect();
        // the bands are made when a winding is first asked: an outline asked once (a Boolean's
        // piece) is cheaper scanned whole
        Outline {loops,sag,bins:std::sync::OnceLock::new()}
    }

    /// The loops, each a closed polygon (its last point joined to its first).
    pub fn loops(&self) -> &[Vec<Uv>] { &self.loops }

    /// How far a chord's middle strays from its edge in space, at most.
    pub fn sag(&self) -> f64 { self.sag }

    /// The loops' signed area in the parameters, counter-clockwise positive.
    pub fn area(&self) -> f64 {
        self.loops.iter().map(|l| (0..l.len()).map(|i| { let (a,b) = (l[i],l[(i+1)%l.len()]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()).sum::<f64>()/2.
    }

    /// How many times the loops wind about `p`, counter-clockwise in the parameters.
    /// Only the chords whose `v` reaches `p`'s band are read: no other crosses its level line.
    pub fn winding(&self,p: Uv) -> i32 {
        let bins = self.bins.get_or_init(|| Bins::of(&self.loops));
        if !(p[1] >= bins.lo && p[1] <= bins.hi) { return 0; }
        let mut w = 0;
        for &(k,i) in &bins.bands[bins.band(p[1])] {
            let l = &self.loops[k as usize];
            let (a,b) = (l[i as usize],l[(i as usize+1)%l.len()]);
            let cross = (b[0]-a[0])*(p[1]-a[1])-(p[0]-a[0])*(b[1]-a[1]);
            if a[1] <= p[1] { if b[1] > p[1] && cross > 0. { w += 1; } }
            else if b[1] <= p[1] && cross < 0. { w -= 1; }
        }
        w
    }

    /// The points `at` (a fraction, ½ the middle) of the way along the spans level lines across
    /// the loops have inside them, widest first: points to read inside the drawing, never yet the
    /// face's (`sag`).
    pub fn spans(&self,at: f64) -> Vec<Uv> {
        let (mut vlo,mut vhi) = (f64::INFINITY,f64::NEG_INFINITY);
        for p in self.loops.iter().flatten() { vlo = vlo.min(p[1]); vhi = vhi.max(p[1]); }
        let mut found: Vec<(f64,Uv)> = Vec::new();
        let mut xs: Vec<f64> = Vec::new();
        for k in 1..16 {
            let v = vlo+(vhi-vlo)*(k as f64+0.37)/16.;
            xs.clear();
            for p in &self.loops {
                for i in 0..p.len() {
                    let (a,b) = (p[i],p[(i+1)%p.len()]);
                    if (a[1] <= v) != (b[1] <= v) { xs.push(a[0]+(v-a[1])/(b[1]-a[1])*(b[0]-a[0])); }
                }
            }
            xs.sort_by(|a,b| a.total_cmp(b));
            for w in xs.chunks(2) {
                if w.len() == 2 && w[1] > w[0] { found.push((w[1]-w[0],[w[0]+at*(w[1]-w[0]),v])); }
            }
        }
        found.sort_by(|a,b| b.0.total_cmp(&a.0));
        found.into_iter().map(|f| f.1).collect()
    }
}

/// Whether `uv` lies on the material's side of a use of an edge through `q`, going `dq` in face
/// `f`'s parameters: its left, mirrored where the face is reversed, read on `q`'s branch of the
/// surface's periods (`Located::face_place` and `boolean`'s pieces, one rule).
pub(crate) fn beside(f: &Face,q: Uv,dq: Uv,uv: Uv) -> bool {
    let w = unwrap(uv,q,f.surface.periods());
    (dq[0]*(w[1]-q[1])-dq[1]*(w[0]-q[0]) > 0.) != f.reversed
}

/// The point of an edge's stretch `t` of `curve` nearest `p`: its parameter and distance (a
/// closed curve read on the representative nearest the stretch; a degenerate edge is its `pole`).
pub(crate) fn nearest_on(curve: &EdgeCurve,t: [f64;2],pole: V,p: V) -> (f64,f64) {
    let EdgeCurve::Curve(curve) = curve else { return (t[0],distance(p,pole)) };
    let mut s = curve.inverse(p);
    if let Some(period) = curve.period() {
        let mid = (t[0]+t[1])/2.;
        s += ((mid-s)/period).round()*period;
    }
    let mut best = (s.clamp(t[0],t[1]),0.);
    best.1 = distance(p,curve.point(best.0));
    for end in t { let d = distance(p,curve.point(end)); if d < best.1 { best = (end,d); } }
    best
}

/// A boundary with its faces' loops drawn in their parameters, for classifying points against it —
/// borrowing the boundary, or owning it where it is kept (`Located::owned`).
#[derive(Clone,Debug)]
pub struct Located<'a> {
    pub b: std::borrow::Cow<'a,Brep>,
    pub tol: f64,
    /// How near its edges a point is read by the side of the nearest edge (a length), farther
    /// ones by the winding of its loops' outline, drawn to an eighth of it (and never nearer than
    /// twice the outline's `sag`).
    coarse: f64,
    /// Each face's loops drawn in its parameters, to an eighth of `coarse`, and its parameter box:
    /// drawn the first time the face is asked about, so a Boolean of a large solid with a small one
    /// draws the faces near the small one, not every face (#589).
    drawn: Vec<std::sync::OnceLock<(Outline,[Uv;2])>>,
    /// Each face's box in space (`face_box`).
    space: Vec<([f64;3],[f64;3])>,
    /// Each face's seams (`Face::seams`), which bound nothing.
    seams: Vec<std::collections::BTreeSet<u32>>,
    /// Each edge's box in space (`edge_box`).
    edge_space: Vec<([f64;3],[f64;3])>,
    /// The solid's size (`Brep::size`), read once.
    size: f64,
}

impl<'a> Located<'a> {
    /// A located boundary that keeps its own copy.
    pub fn owned(b: Brep,tol: f64) -> Located<'static> {
        let (tol,coarse,drawn,space,seams,edge_space,size) = { let l = Located::new(&b,tol);
            (l.tol,l.coarse,l.drawn,l.space,l.seams,l.edge_space,l.size) };
        Located {b:std::borrow::Cow::Owned(b),tol,coarse,drawn,space,seams,edge_space,size}
    }
    pub fn new(b: &'a Brep,tol: f64) -> Located<'a> { Located::boxed(b,tol,&|_| None) }

    /// `new`, each face's box in space taken from `known` where it has one (`face_box` where not):
    /// a face a Boolean carried whole has the box it had (#589).
    pub(crate) fn boxed(b: &'a Brep,tol: f64,known: &dyn Fn(usize) -> Option<([f64;3],[f64;3])>) -> Located<'a> {
        let size = b.size();
        let coarse = (1e-4*size).max(tol*8.);
        let drawn = (0..b.faces.len()).map(|_| std::sync::OnceLock::new()).collect();
        let space = (0..b.faces.len()).map(|fi| known(fi).unwrap_or_else(|| face_box(b,fi))).collect();
        let seams = b.faces.iter().map(Face::seams).collect();
        let edge_space = (0..b.edges.len()).map(|e| edge_box(b,e,size)).collect();
        Located {b:std::borrow::Cow::Borrowed(b),tol,coarse,drawn,space,seams,edge_space,size}
    }

    /// Face `fi`'s outline and its parameter box, drawn once.
    fn drawn(&self,fi: usize) -> &(Outline,[Uv;2]) {
        self.drawn[fi].get_or_init(|| {
            let (b,f) = (&*self.b,&self.b.faces[fi]);
            let outline = Outline::of(&f.surface,self.coarse/8.,f.loops.iter().map(|l| l.iter().map(|c| {
                let e = &b.edges[c.edge as usize];
                move |s: f64| {
                    let t = if c.reversed { e.t[1]-(e.t[1]-e.t[0])*s } else { e.t[0]+(e.t[1]-e.t[0])*s };
                    (c.pcurve.at(t,e,&f.surface,&b.vertices),e.point(t,&b.vertices))
                }
            })));
            let mut bx = [[f64::INFINITY;2],[f64::NEG_INFINITY;2]];
            for p in outline.loops().iter().flatten() { for k in 0..2 { bx[0][k] = bx[0][k].min(p[k]); bx[1][k] = bx[1][k].max(p[k]); } }
            (outline,bx)
        })
    }

    /// The parameters of a point of face `fi`'s surface, on the branch of its periods its loops
    /// are drawn on.
    pub fn uv(&self,fi: usize,p: V) -> Uv {
        let f = &self.b.faces[fi];
        let bx = self.drawn(fi).1;
        let mid = [(bx[0][0]+bx[1][0])/2.,(bx[0][1]+bx[1][1])/2.];
        unwrap(f.surface.inverse_near(p,mid),mid,f.surface.periods())
    }

    /// Face `fi`'s box in space (`face_box`).
    pub fn face_box(&self,fi: usize) -> ([f64;3],[f64;3]) { self.space[fi] }

    /// How far edge `e`'s box is from `p`: no point of the edge is nearer.
    pub fn edge_gap(&self,e: u32,p: V) -> f64 { gap(self.edge_space[e as usize],p) }

    /// How far face `fi`'s box is from `p`: no point of the face or its edges is nearer.
    pub fn face_gap(&self,fi: usize,p: V) -> f64 { gap(self.space[fi],p) }

    /// The point of an edge nearest `p`: its parameter and distance.
    pub fn nearest(&self,e: u32,p: V) -> (f64,f64) {
        let e = &self.b.edges[e as usize];
        nearest_on(&e.curve,e.t,self.b.vertices[e.v[0] as usize].p,p)
    }

    /// **Whether face `fi` holds `p`, a point of a surface the same as the face's** (`ssi::same`):
    /// in the face (`face_place`), which holds a point only on the face's own patch where its
    /// surface has a rim. A sheet lying in a plane is that plane over its patch alone (#780 C4):
    /// past the patch a point's foot is clamped to its edge, which the face's loop may run along,
    /// and read there the point would be in the face.
    pub fn face_holds(&self,fi: usize,p: V) -> bool { self.face_place(fi,p) == Place::In }

    /// Where a point of face `fi`'s surface stands against the face: `On` within `tol` of its
    /// boundary. A seam (an edge the face uses twice, once each side) is no boundary: a point on
    /// one, away from every other edge, is in the face.
    pub fn face_place(&self,fi: usize,p: V) -> Place {
        let f: &Face = &self.b.faces[fi];
        // where every edge's box is farther than the near-edge reading reaches, the winding
        // decides, as it would with every edge measured
        let near = self.coarse.max(2.*self.drawn(fi).0.sag());
        // a point past a patch's rim, on its surface's extension, is on no point of the face (#780
        // L1: a pipe entered stretch by stretch runs on smoothly into the next stretch, whose points
        // the inverse clamps onto their shared circle, read there as in the face — a ray crossing
        // the pipe beside the circle counted on both): its foot off it by more than the tolerance,
        // read from the parameters in hand and held to the surface's own inverse before refused
        let past = |uv: Uv| f.surface.bounded() && distance(f.surface.point(uv),p) > self.tol.max(1e-9*(1.+norm(p)))
            && !on_bounded(&f.surface,p,self.tol);
        if f.loops.iter().flatten().all(|c| self.edge_gap(c.edge,p) > near) {
            let uv = self.uv(fi,p);
            if past(uv) { return Place::Out }
            let winding = self.drawn(fi).0.winding(uv);
            let inside = if f.reversed { winding < 0 } else { winding > 0 };
            return if inside { Place::In } else { Place::Out }
        }
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
        // (past the rim, however near its edge)
        if past(uv) { return Place::Out }
        let e = &self.b.edges[c.edge as usize];
        if d <= self.coarse.max(2.*self.drawn(fi).0.sag()) && t > e.t[0] && t < e.t[1] {
            // near one edge's interior: which side of it, in the face's parameters — the point
            // on the branch beside each use of the edge (a seam is used twice, once each side of
            // the face), inside if beside any use it is on the face's side
            let hint = f.loops.iter().flatten().find(|u| u.edge == c.edge).map_or(uv,|u| u.pcurve.at(t,e,&f.surface,&self.b.vertices));
            let raw = f.surface.inverse_near(p,hint);
            let inside = f.loops.iter().flatten().filter(|u| u.edge == c.edge).any(|u| {
                let (q,mut dq) = u.pcurve.at_and_derivative(t,e,&f.surface,&self.b.vertices);
                if u.reversed { dq = [-dq[0],-dq[1]]; }
                beside(f,q,dq,raw)
            });
            return if inside { Place::In } else { Place::Out }
        }
        let winding = self.drawn(fi).0.winding(uv);
        // a reversed face's loops turn the other way round
        let inside = if f.reversed { winding < 0 } else { winding > 0 };
        if inside { Place::In } else { Place::Out }
    }

    /// Where a point of space stands against the solid: `On` within `tol` of a face, otherwise by
    /// the parity of the faces a ray from it crosses (a ray grazing an edge or touching a face, or
    /// one whose reading against a face was refused, is abandoned for another direction).
    pub fn solid_place(&self,p: V) -> Place {
        for (fi,f) in self.b.faces.iter().enumerate() {
            if f.surface.implicit(p).abs() <= self.tol && self.face_place(fi,p) != Place::Out { return Place::On }
        }
        let reach = 4.*self.size+distance(p,self.b.vertices.first().map_or(p,|v| v.p));
        const DIRECTIONS: [V;5] = [[0.5773,0.6124,0.5402],[-0.3172,0.8123,-0.4893],[0.7071,-0.3162,0.6325],
            [-0.6412,-0.5123,0.5712],[0.1234,-0.4321,-0.8934]];
        'dirs: for d in DIRECTIONS {
            let d = scale(d,1./norm(d));
            let ray = Curve::Line {p,d};
            let mut crossings = 0;
            for (fi,f) in self.b.faces.iter().enumerate() {
                match within(&ray,[0.,reach],&f.surface,self.tol*1e-3,self.space[fi],self.tol) {
                    // lying along a face, or a reading not proved (#814 P3): another direction
                    Meets::Along | Meets::Refused(_) => continue 'dirs,
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
    let mut rng = crate::util::rng::Rng::new(0x1a7e);
    let mut candidates: Vec<V> = (0..N*N*N).map(|k| {
        let ijk = [k % N,(k/N) % N,k/(N*N)];
        std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*(ijk[a] as f64+rng.uniform(0.2,0.8))/N as f64)
    }).collect();
    let stride = tris.len().div_ceil(STEPPED).max(1);
    for t in tris.iter().step_by(stride) {
        let (u,v) = ([t[1][0]-t[0][0],t[1][1]-t[0][1],t[1][2]-t[0][2]],[t[2][0]-t[0][0],t[2][1]-t[0][1],t[2][2]-t[0][2]]);
        let n = crate::util::space::cross(u,v);
        let l = norm(n);
        if l == 0. { continue }
        let centre = [(t[0][0]+t[1][0]+t[2][0])/3.,(t[0][1]+t[1][1]+t[2][1])/3.,(t[0][2]+t[1][2]+t[2][2])/3.];
        for k in 1..=6 { candidates.push(add(centre,scale(n,-size/4f64.dpowi(k)/l))); }
    }
    // each candidate's distance from the mesh, the least over its triangles, sought through their
    // boxes (each grown by a hair, so a box passed by for being farther than the nearest found holds
    // no triangle that would read nearer)
    let grow = 1e-9*(1.+size);
    let boxes = crate::util::bvh::Bvh::new(tris.iter().map(|t| crate::util::bvh::Bounds {
        lo:std::array::from_fn(|a| t[0][a].min(t[1][a]).min(t[2][a])-grow),
        hi:std::array::from_fn(|a| t[0][a].max(t[1][a]).max(t[2][a])+grow),
    }));
    let depth = crate::util::par::map(&candidates,|&p| {
        let d2 = std::cell::Cell::new(f64::INFINITY);
        boxes.query_nearest(|b| b.gap(crate::util::bvh::Bounds {lo:p,hi:p}),|| d2.get().sqrt(),
            |k| d2.set(d2.get().min(triangle_distance2(p,&tris[k]))));
        d2.get().sqrt()-m.sag
    });
    // its winding number, which only a point the choice reaches is asked
    let winding = |p: V| {
        let mut w = 0.;
        for t in &tris {
            let [a,b,c] = t.map(|q| [q[0]-p[0],q[1]-p[1],q[2]-p[2]]);
            let (la,lb,lc) = (norm(a),norm(b),norm(c));
            let det = a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]);
            let dot = |x: V,y: V| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
            w += 2.*det.datan2(la*lb*lc+dot(a,b)*lc+dot(b,c)*la+dot(c,a)*lb);
        }
        w/(4.*std::f64::consts::PI)
    };
    // clear of the mesh, the deepest first (the earlier of two as deep): each taken where it is
    // inside and no nearer a point taken than half the first's depth. Their windings are read a
    // batch at a time across the cores, ahead of the choice
    let mut order: Vec<usize> = (0..candidates.len()).filter(|&i| depth[i] > 0.).collect();
    order.sort_by(|&i,&j| depth[j].total_cmp(&depth[i]));
    let open = |chosen: &[(V,f64)],i: usize|
        !chosen.first().is_some_and(|f| chosen.iter().any(|c| distance(c.0,candidates[i]) < f.1/2.));
    let mut chosen: Vec<(V,f64)> = Vec::new();
    for batch in order.chunks(4*crate::util::par::threads()) {
        if chosen.len() == capacity { break }
        let asked: Vec<usize> = batch.iter().copied().filter(|&i| open(&chosen,i)).collect();
        let inside = crate::util::par::map(&asked,|&i| winding(candidates[i]) > 0.5);
        for (&i,inside) in asked.iter().zip(inside) {
            if chosen.len() == capacity { break }
            if !inside || !open(&chosen,i) { continue }
            chosen.push((candidates[i],depth[i]));
        }
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
