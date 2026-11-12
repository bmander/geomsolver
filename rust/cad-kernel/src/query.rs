//! The questions a Boolean asks of geometry and of a boundary: where a curve meets a surface,
//! whether a point of a face's surface is inside the face, and whether a point of space is inside
//! a solid.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{Curve,Surface,Uv,V};
use super::nurbs::blossom;
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
    let within = |c: &Curve,t: f64,span: [f64;2]| within_stretch(c,t,span,tol);
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

/// Curve `c`'s parameter `t` inside its stretch `span` to within `tol` along it, turned into the
/// stretch where the curve is closed and clamped to it; `None` outside.
fn within_stretch(c: &Curve,t: f64,span: [f64;2],tol: f64) -> Option<f64> {
    let t = match c.period() { Some(p) => super::geom::around(t,span[0],p),None => t };
    let slack = tol/c.speed().max(1e-300);
    // a crossing just before the start of a closed curve's stretch is at its start
    let t = match c.period() { Some(p) if t > span[1]+slack && t-p >= span[0]-slack => t-p,_ => t };
    (t >= span[0]-slack && t <= span[1]+slack).then_some(t.clamp(span[0],span[1]))
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
    // a ruling the cut's plane holds (laid beside the cut, or an edge lying along it): the cut crosses
    // it at that ruling's `v`, where the sheet touches the plane (#885) — given by the cut's own
    // closed form, not sought along a curve that runs off to infinity at its other parallel
    // rulings; a line that is no held ruling is left to its carrier's roots
    if let Curve::Line {p,d} = *other {
        let at = cut.held_along(p,d,tol)?;
        let q = cut.point(at);
        let slack = 1e-12*(1.+at.abs());
        if !(at >= cut_t[0]-slack && at <= cut_t[1]+slack) { return Some(Vec::new()) }
        let t = within_stretch(other,other.inverse(q),other_t,tol);
        return Some(t.map(|t| (if cut_is_a { at } else { t },q)).into_iter().collect())
    }
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
    // a net's: a curve whose start, middle or end stands within `tol` of none of its spans' boxes
    // lies along no stretch of it (`near_net`)
    if ![t0,0.5*(t0+t1),t1].iter().all(|&t| near_net(surface,curve.point(t),tol)) { return false }
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
///   read where the curve has left the tolerance on either side, and a touch where it does not —
///   where the surface's distance turns along the run, read from its rate at the run's ends, or at
///   its middle where it does not turn. So a root found twice (at a knot two spans share, from two stretches) is one
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
    // where the curve leaves the tolerance of the surface from `t` the way `way` goes, within the
    // stretch: stepped twice the tolerance along it and doubled, then halved back to it
    let left = |t: f64,way: f64| -> f64 {
        let end = if way < 0. { t0.min(t1) } else { t0.max(t1) };
        if !end.is_finite() { return t }
        let mut step = 2.*tol/norm(curve.d1(t).1).max(1e-300);
        let mut inside = t;
        for _ in 0..200 {
            let x = (inside+way*step).clamp(t0.min(t1),t0.max(t1));
            if side(x).abs() <= tol { inside = x; if x == end { return x } step *= 2. } else { step *= 0.5 }
            if step < 1e-15*(1.+t.abs()) { break }
        }
        inside
    };
    let mut runs: Vec<(f64,bool)> = Vec::with_capacity(out.len());
    let mut i = 0;
    while i < out.len() {
        let mut j = i;
        while j+1 < out.len() && joined(out[j].0,out[j+1].0) { j += 1 }
        if j == i { runs.push(out[i]) } else {
            let (a,b) = (out[i].0,out[j].0);
            let touch = out[i..=j].iter().all(|r| r.1) || clear(a,-1.)*clear(b,1.) > 0.;
            let rate = |t: f64| { let (p,d) = curve.d1(t); dot(surface.gradient(p),d) };
            if !touch { runs.push((a,false)) } else {
                // one touch, where the surface's distance turns along the run as far as the curve
                // keeps within the tolerance (a run proved whole is proved short of where it
                // leaves): its rate is read at the run's ends, where the curve leaves the
                // tolerance and the rate is past rounding's, and the turn bracketed between them;
                // at the run's middle where it does not turn (#814: a run proved whole, placed at its
                // middle, stood 0.6 of a trace's parameter from where its neighbours met the surface;
                // read as two touches where rounding's flips of the rate inside it were counted, a
                // last bit's difference between two machines built a turned ring three times its volume)
                let (a,b) = (left(a,-1.),left(b,1.));
                let (ra,rb) = (rate(a),rate(b));
                runs.push((if (ra < 0.) != (rb < 0.) { crate::util::roots::bracketed_root(|t| Some(rate(t)),a,ra,b,rb,1e-15*(1.+a.abs().max(b.abs()))) } else { 0.5*(a+b) },true));
            }
        }
        i = j+1;
    }
    for end in [t0,t1] {
        if !end.is_finite() || !near_net(surface,curve.point(end),tol) || !(side(end).abs() <= tol) || !on_bounded(surface,curve.point(end),tol) { continue }
        runs.retain(|r| !joined(r.0,end));
        runs.push((end,false));
    }
    runs.sort_by(|x,y| x.0.total_cmp(&y.0));
    runs
}

/// **Whether `p` may stand within `tol` of `surface`**, read off a net's spans' boxes alone
/// (`net_index`): a point within `tol` of none is off the net, its distance not read — which a
/// fitted sheet reads by searching for its foot, from afar at every span it may be by (each end of a
/// ray placing a point, at every tooth's sheet it passed). Any other surface: `true`.
fn near_net(surface: &Surface,p: V,tol: f64) -> bool {
    let Surface::BSpline(_,net) = surface else { return true };
    if net.weights.is_some() { return true }
    let mut near = false;
    net_index(net).tree.query_box(crate::util::bvh::Bounds {lo:p.map(|x| x-tol),hi:p.map(|x| x+tol)},|_| near = true);
    near
}

/// **Why a reader has no answer** (#780 R3, #814), told apart so each is answered its own way: no
/// closed form for the pair (`NoForm`, and a closed form's search past its most boxes), which the
/// general reading then reads; a build stopped past its budget (`Stopped`), which reads nothing
/// more; or a search that proved nothing (`Gave`, with its box), which is the answer — `read` makes
/// it the refusal, with its witness. The general reading's `NoForm` is a pair with nothing bounded
/// to search over, refused too.
#[derive(Clone,Debug)]
pub enum Uncertified { NoForm,Stopped,Gave(GaveUp) }

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
        // sheet meets that line's cylinder along (`line_along_screw`); a straight edge's sheet
        // where the line crosses it cleanly (`line_screwed_line`); any other sheet, and any other
        // reading of those, by the general reading
        (&Curve::Line {p,d},Surface::Blend(_,b)) => match b.level_circle().map(|c| line_screwed_circle(p,d,[t0,t1],&c,tol)) {
            None | Some(Err(Uncertified::NoForm)) => match line_along_screw(p,d,[t0,t1],b,tol) {
                Err(Uncertified::NoForm) => line_screwed_line(p,d,[t0,t1],b,tol),
                read => read,
            },
            Some(read) => read,
        },
        (Curve::Circle(f,big),Surface::Blend(_,b)) => circle_round_screwed(f,big.abs(),b,[t0,t1],tol),
        (Curve::Iso(b,u),Surface::Sphere(c,r)) => helix_sphere(b,*u,c.o,r.abs(),[t0,t1],tol),
        // a circle's or an ellipse's on any surface with an algebraic form here, an edge of a
        // drum, a fillet's rim or a section's arc (#780 R2)
        (Curve::Circle(..) | Curve::Ellipse(..),_) if degree(surface).is_some() => conic_algebraic(curve,[t0,t1],surface,tol),
        // and against a surface turned from a conic, whose algebraic form is read span by span
        (Curve::Circle(..) | Curve::Ellipse(..),Surface::Revolution(f,c)) => conic_revolution(curve,[t0,t1],f,c,surface,tol),
        (Curve::BSpline(sp),Surface::Plane(f)) => spline_plane(sp,[t0,t1],f,surface,tol),
        _ => Err(Uncertified::NoForm),
    }
}

/// **A surface's algebraic form's degree**, where it has one here (`form`: a plane's height, `ρ² −
/// r²` a cylinder's, `|q|² − r²` a ball's, `ρ² − (r + z tan a)²` a cone's, `(|q|² + R² − r²)² −
/// 4R²ρ²` a torus's). `None` for a surface with none.
fn degree(surface: &Surface) -> Option<usize> {
    form::<f64>(surface,surface.frame().o)?;
    Some(match surface { Surface::Plane(_) => 1,Surface::Torus(..) => 4,_ => 2 })
}

/// Whether `p`, a zero of a surface's algebraic form, is a point of the surface: on a cone, of its
/// one sheet (`r + z tan a ≥ 0`); on any other, any.
fn kept(surface: &Surface,tol: f64,p: V) -> bool {
    use crate::util::fmath::Det;
    match *surface { Surface::Cone(ref f,r,a) => { let (sa,ca) = a.dsin_cos(); r+sa/ca*f.local(p)[2] >= -tol },_ => true }
}

/// **A circle or an ellipse (`o + a cos t + b sin t`) against a surface with an algebraic form of
/// degree `n`** (`algebraic`; #780 R2): the form along it is a trigonometric polynomial of degree
/// `n`, `c₀ + Σ (a_k cos kt + b_k sin kt)`, its coefficients read exactly from `2n + 1` points
/// evenly round it, written once over `Num` (`Trig`) so its rate is enclosed over a stretch by the
/// same formula read in intervals, and its roots isolated by it (`util::isolate`, #814), a touch
/// accepted where the surface's distance is within `tol`. Each kept where the surface's points are.
///
/// **A run beside a touch is proved by the polynomial's Taylor model** (`Taylored`, #904 A4), the
/// form over the curve's box (`Implicit`) where that proves it first. Beside a touch the polynomial
/// is within its own rounding of nought over a stretch its value's enclosure cannot sign, and the
/// form read in intervals over the curve's box — a box of the world's axes, read into the surface's
/// frame — is wider than the band the run lies in until the stretch is a hair wide. A circle lying
/// in the plane tangent to a torus's tube was halved to that hair across the whole stretch, and at
/// one attitude of the pair in a dozen read past any search's boxes and was refused: a gear
/// member's tool that built in its own frame was refused in a frame turned to it. The Taylor model
/// is the polynomial's own, the same at every attitude, and proves the run as soon as the stretch
/// lies in the band.
fn conic_algebraic(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Certified {
    let n = degree(surface).ok_or(Uncertified::NoForm)?;
    let at = |t: f64| curve.point(t);
    let (c0,terms) = trig_along(curve,n,&|p| form(surface,p).map_or(f64::NAN,|x| x.0));
    let trig = Trig {c0,terms:&terms};
    let gradient = |x: I3| form(surface,x).map(|(_,g)| g);
    let by = Either(Implicit::new(curve,surface,tol),Taylored::new(trig,curve,&gradient,tol));
    let g = Banded {f:crate::util::isolate::Written(trig),by};
    let found = crate::util::isolate::roots(&g,[t0,t1],tol*1e-3/curve.speed().max(1e-300),&|t| surface.implicit(at(t)).abs() <= tol)?;
    Ok(Meets::At(found.into_iter().filter(|r| kept(surface,tol,at(r.at))).map(|r| (r.at,r.touch)).collect()))
}

/// **A closed form's function, its runs read by another** (`by`): where a curve lies within the
/// tolerance of an algebraic surface for a while, no stretch of the closed form's polynomial can be
/// parted or excluded either, and the run is proved by the general reading's form (`Implicit`) or
/// by the polynomial's Taylor model (`Taylored`).
struct Banded<F,B> { f: F,by: B }

/// **What proves a run** (`Banded`'s `by`): whether a curve is within the tolerance of a surface
/// over all of `[a, b]`.
trait Run { fn within(&self,a: f64,b: f64) -> bool; }

/// A run proved by either of two (`conic_algebraic`'s), the first asked first.
struct Either<A,B>(A,B);

impl<A: Run,B: Run> Run for Either<A,B> {
    fn within(&self,a: f64,b: f64) -> bool { self.0.within(a,b) || self.1.within(a,b) }
}

impl<F: crate::util::isolate::Function,B: Run> crate::util::isolate::Function for Banded<F,B> {
    fn at(&self,t: f64) -> (f64,f64) { self.f.at(t) }
    fn value(&self,t: f64) -> Iv { self.f.value(t) }
    fn slope(&self,a: f64,b: f64) -> Iv { self.f.slope(a,b) }
    fn within(&self,a: f64,b: f64) -> bool { self.by.within(a,b) }
}

/// **A form of degree `n` along a circle or an ellipse** (`o + a cos t + b sin t`) as the
/// trigonometric polynomial it is, `c₀ + Σ (a_k cos kt + b_k sin kt)`, `k` to `n`: its
/// coefficients read exactly from `2n + 1` points evenly round it.
fn trig_along(curve: &Curve,n: usize,form: &dyn Fn(V) -> f64) -> (f64,Vec<(f64,f64)>) {
    use std::f64::consts::TAU;
    let m = 2*n+1;
    let ys: Vec<f64> = (0..m).map(|j| form(curve.point(TAU*j as f64/m as f64))).collect();
    let c0 = ys.iter().sum::<f64>()/m as f64;
    let terms = (1..=n).map(|k| {
        let (mut x,mut y) = (0.,0.);
        for (j,v) in ys.iter().enumerate() { let (s,c) = (TAU*(k*j) as f64/m as f64).dsin_cos(); x += v*c; y += v*s; }
        (2.*x/m as f64,2.*y/m as f64)
    }).collect();
    (c0,terms)
}

/// **A trigonometric polynomial** `c₀ + Σ (a_k cos kt + b_k sin kt)` (`conic_algebraic`,
/// `conic_revolution`), written once over `Num`.
#[derive(Clone,Copy)]
struct Trig<'a> { c0: f64,terms: &'a [(f64,f64)] }

impl crate::util::isolate::Formula for Trig<'_> {
    fn jet<T: Num>(&self,t: crate::util::interval::Jet<T>) -> crate::util::interval::Jet<T> {
        self.terms.iter().enumerate().fold(crate::util::interval::Jet::of(self.c0),|acc,(i,&(x,y))| {
            let (s,c) = t.times((i+1) as f64).sin_cos();
            acc+c.times(x)+s.times(y)
        })
    }
}

/// How many terms a Taylor model of a `Trig` keeps before its remainder.
const TAYLOR: usize = 8;

/// **A circle or an ellipse against a surface of revolution whose meridian is a conic** (a
/// rational B-spline of degree two at most, `Surface::Revolution` about `f`'s axis, its meridian in
/// the frame's `xz` half plane): the curve's counterpart of `line_revolution`. Over each knot span
/// the meridian is a conic arc `F(r, z) = 0` in the radius `r` and height `z` — its quadratic
/// Bézier's, `4 w₁² Λ₀ Λ₂ − w₀ w₂ Λ₁²` in the barycentric coordinates `Λᵢ` of its poles, or the line
/// through its ends where they lie on one — and the surface turned from it is where `F(ρ, z) F(−ρ, z)
/// = P² − ρ² O²` is nought, `P` the part of `F` even in `r` and `r O` the odd. A meridian symmetric
/// about the axis (a paraboloid's, `O` nought) is `P = 0` alone, so a contact on the axis keeps
/// its own order. Along the curve the form is a trigonometric polynomial of degree four (two for
/// `P`), its roots isolated (`util::isolate`), and a run the curve keeps within the tolerance
/// proved by its Taylor model (`Taylored`) — the ball's meridian in a paraboloid cup that fits its
/// bottom, within the fourth power of the way from the pole it touches at, which neither the
/// polynomial's interval enclosures nor the general reading's two unknowns can part or exclude.
/// Each root kept where it lies on the bounded surface (a root of the conic past the span, or of
/// its mirror, is off it). `NoForm` for a meridian of higher degree or out of the frame's plane,
/// or a weight not positive.
fn conic_revolution(curve: &Curve,[t0,t1]: [f64;2],f: &super::geom::Frame,c: &Curve,surface: &Surface,
                    tol: f64) -> Certified {
    let Curve::BSpline(sp) = c else { return Err(Uncertified::NoForm) };
    if sp.degree > 2 { return Err(Uncertified::NoForm) }
    let at = |t: f64| curve.point(t);
    let near = |t: f64| surface.implicit(at(t)).abs() <= tol;
    let mut out: Vec<(f64,bool)> = Vec::new();
    for w in spans(sp).windows(2) {
        let form = Revolved::of(f,|s: f64| sp.homogeneous(w[0]+(w[1]-w[0])*s)).ok_or(Uncertified::NoForm)?;
        let (c0,terms) = trig_along(curve,if form.even { 2 } else { 4 },&|p| form.at::<f64>(p).0);
        let trig = Trig {c0,terms:&terms};
        let gradient = |x: I3| Some(form.at(x).1);
        let g = Banded {f:crate::util::isolate::Written(trig),by:Taylored::new(trig,curve,&gradient,tol)};
        for r in crate::util::isolate::roots(&g,[t0,t1],tol*1e-3/curve.speed().max(1e-300),&near)? {
            if on_bounded(surface,at(r.at),2.*tol) { out.push((r.at,r.touch)) }
        }
    }
    out.sort_by(|x,y| x.0.total_cmp(&y.0));
    Ok(Meets::At(out))
}

/// A B-spline's knot spans: its domain's ends and the breaks between.
fn spans(sp: &super::nurbs::BSpline) -> Vec<f64> {
    let [k0,k1] = sp.domain();
    let mut knots = vec![k0];
    knots.extend(sp.breaks([k0,k1]));
    knots.push(k1);
    knots
}

/// **The algebraic form of a surface turned from a conic arc** (`conic_revolution`): in `frame`'s
/// coordinates `(x, y, z)`, `ρ² = x² + y²`, it is `P² − ρ² O²` with `P = A ρ² + C z² + E z + G`
/// and `O = B z + D` — or `P` alone where the arc is symmetric about the axis (`even`) — from the
/// arc's `F(r, z) = A r² + B r z + C z² + D r + E z + G`, `f` its `[A, B, C, D, E, G]`.
struct Revolved { frame: super::geom::Frame,even: bool,f: [f64;6] }

impl Revolved {
    /// The form of the surface turned from the conic arc `h` (homogeneous, `σ` over `[0, 1]`, in
    /// space) about `frame`'s axis; `None` where the arc is no quadratic arc in the frame's `xz`
    /// plane with positive weights.
    fn of(frame: &super::geom::Frame,h: impl Fn(f64) -> (V,f64)) -> Option<Revolved> {
        use super::plane2::{cross,sub};
        // each point homogeneous in the frame's `(r, z)`: `[r w, z w, w]`, its `y` nought
        let read = |s: f64| { let (x,w) = h(s); let l = frame.local(scale(x,1./w)); ([l[0]*w,l[2]*w,w],l) };
        let ((h0,l0),(hm,lm),(h2,l2)) = (read(0.),read(0.5),read(1.));
        let reach = [l0,lm,l2].iter().flatten().fold(0.,|m: f64,x| m.max(x.abs())).max(1e-300);
        if [l0,lm,l2].iter().any(|l| l[1].abs() > 1e-9*reach) { return None }
        // the middle pole from the point halfway, `(P₀ + 2P₁ + P₂)/4`
        let h1: [f64;3] = std::array::from_fn(|i| 2.*hm[i]-0.5*(h0[i]+h2[i]));
        let [w0,w1,w2] = [h0[2],h1[2],h2[2]];
        if !(w0 > 0. && w1 > 0. && w2 > 0.) { return None }
        let [p0,p1,p2] = [h0,h1,h2].map(|p| [p[0]/p[2],p[1]/p[2]]);
        // a linear form `[r, z, 1]` vanishing on the line through `u` and `v` (`(u − q) × (v − q)`)
        let through = |u: [f64;2],v: [f64;2]| { let w = sub(v,u); [-w[1],w[0],cross(u,v)] };
        // the product of two linear forms, as `[A, B, C, D, E, G]`
        let times = |x: [f64;3],y: [f64;3]| [x[0]*y[0],x[0]*y[1]+x[1]*y[0],x[1]*y[1],
            x[0]*y[2]+x[2]*y[0],x[1]*y[2]+x[2]*y[1],x[2]*y[2]];
        let f: [f64;6] = if cross(sub(p1,p0),sub(p2,p0)).abs() <= 1e-12*reach*reach {
            // the poles on one line: the arc is a stretch of it
            let l = through(p0,p2);
            [0.,0.,0.,l[0],l[1],l[2]]
        } else {
            let (x,y) = (times(through(p1,p2),through(p0,p1)),times(through(p2,p0),through(p2,p0)));
            std::array::from_fn(|i| 4.*w1*w1*x[i]-w0*w2*y[i])
        };
        // each coefficient weighed by the power of length it carries, the whole scaled to one
        let weighed = [f[0]*reach*reach,f[1]*reach*reach,f[2]*reach*reach,f[3]*reach,f[4]*reach,f[5]];
        let whole = weighed.iter().map(|x| x.abs()).sum::<f64>();
        if !(whole > 0.) || !whole.is_finite() { return None }
        let even = (weighed[1].abs()+weighed[3].abs())/whole <= 1e-9;
        Some(Revolved {frame:*frame,even,f:f.map(|x| x/whole)})
    }
    /// The form and its gradient at `x`, written once over `Num`.
    fn at<T: Num>(&self,x: [T;3]) -> (T,[T;3]) {
        let k = |v: f64| T::of(v);
        let [a,b,c,d,e,g] = self.f;
        let q = local(&self.frame,x);
        let two = k(2.);
        let rho2 = q[0]*q[0]+q[1]*q[1];
        let p = k(a)*rho2+k(c)*q[2]*q[2]+k(e)*q[2]+k(g);
        let dp = [two*k(a)*q[0],two*k(a)*q[1],two*k(c)*q[2]+k(e)];
        let (v,grad) = if self.even { (p,dp) } else {
            let odd = k(b)*q[2]+k(d);
            let o2 = odd*odd;
            (p*p-rho2*o2,[two*p*dp[0]-two*q[0]*o2,two*p*dp[1]-two*q[1]*o2,two*p*dp[2]-two*rho2*odd*k(b)])
        };
        (v,global(&self.frame,grad))
    }
}

/// **A run along a conic proved by its polynomial's Taylor model** (`conic_revolution`,
/// `conic_algebraic`): the `Trig` the form is along the curve, over a stretch, enclosed about the
/// stretch's middle — `TAYLOR` terms read there, each term's rotated a quarter per order, and the
/// remainder bounded by `Σ k^TAYLOR (|a_k| + |b_k|)` (`rest`), which no derivative of that order
/// exceeds anywhere — held to the form's gradient over the curve's enclosure there (`gradient`,
/// `None` where it has none; `banded`). Where the form has a root of high order (the fourth power of
/// the way from it, a ball's meridian in the paraboloid that fits its bottom) its terms cancel, and
/// the formula read in intervals encloses each term's spread and not the sum's: this encloses the
/// sum.
struct Taylored<'a> { trig: Trig<'a>,rest: f64,curve: &'a Curve,gradient: &'a dyn Fn(I3) -> Option<I3>,tol: f64 }

impl<'a> Taylored<'a> {
    fn new(trig: Trig<'a>,curve: &'a Curve,gradient: &'a dyn Fn(I3) -> Option<I3>,tol: f64) -> Taylored<'a> {
        let rest = trig.terms.iter().enumerate().fold(Iv::point(0.),|acc,(i,&(x,y))| {
            acc+Iv::point(((i+1) as f64).powi(TAYLOR as i32))*(Iv::point(x.abs())+Iv::point(y.abs()))
        }).hi();
        Taylored {trig,rest,curve,gradient,tol}
    }
    /// The polynomial's value over `[a, b]`, enclosed by its Taylor model.
    fn value(&self,a: f64,b: f64) -> Iv {
        let m = 0.5*(a+b);
        let h = Iv::new(a,b)-Iv::point(m);
        // each order's coefficient at the middle, `g⁽ʲ⁾(m) / j!`, the terms' sines read once
        let mut orders = [Iv::point(0.);TAYLOR];
        orders[0] = Iv::point(self.trig.c0);
        for (i,&(x,y)) in self.trig.terms.iter().enumerate() {
            let k = (i+1) as f64;
            let (s,c) = Num::sin_cos(Iv::point(m).times(k));
            let (x,y) = (Iv::point(x),Iv::point(y));
            let turns = [x*c+y*s,y*c-x*s];
            let mut scale = Iv::point(1.);
            for (j,o) in orders.iter_mut().enumerate() {
                if j > 0 { scale = scale*Iv::point(k)/Iv::point(j as f64); }
                let term = if j%4 < 2 { turns[j%2] } else { -turns[j%2] };
                *o = *o+term*scale;
            }
        }
        let (mut sum,mut power) = (Iv::point(0.),Iv::point(1.));
        for (j,o) in orders.iter().enumerate() {
            if j > 0 { power = power*h; }
            sum = sum+*o*power;
        }
        let fact: f64 = (1..=TAYLOR).map(|j| j as f64).product();
        let rest = (Iv::point(self.rest)*Iv::point((power*h).mag())/Iv::point(fact)).hi();
        sum+Iv::new(-rest,rest)
    }
}

impl Run for Taylored<'_> {
    fn within(&self,a: f64,b: f64) -> bool {
        let Some(g) = (self.gradient)(self.curve.enclose([[a,b]]).at) else { return false };
        banded(self.value(a,b),g,self.tol)
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
    let value = |t: f64| crate::enclose::scalar(sp.degree,&sp.knots,c,[t,t],false).0;
    let slope = |a: f64,b: f64| crate::enclose::scalar(sp.degree,&sp.knots,c,[a,b],true).1;
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
    let line = Curve::Line {p,d};
    let g = Banded {f:crate::util::isolate::Poly(c),by:Implicit::new(&line,surface,tol)};
    let found = crate::util::isolate::roots(&g,[lo,hi],tol*1e-3/speed,&|t| surface.implicit(at(t)).abs() <= tol)?;
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
    let knots = spans(sp);
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
    // (`Most` given up here is the answer, never the general reading's to try again: not `From`)
    let refuse = Uncertified::Gave;
    let Some([a,b]) = bounded_stretch(curve,[t0,t1],surface,tol) else { return Err(Uncertified::NoForm) };
    if !(b > a) { return Ok((Meets::At(Vec::new()),true)) }
    let whole = curve.enclose([[a,b]]);
    let proved = whole.proved;
    if degree(surface).is_some() {
        let g = Implicit::new(curve,surface,tol);
        let speed = norm(whole.d[0].map(|x| x.mag()));
        let speed = if speed.is_finite() { speed } else { curve.speed() };
        let found = crate::util::isolate::roots(&g,[a,b],tol*1e-3/speed.max(1e-300),&|t| surface.implicit(curve.point(t)).abs() <= tol)
            .map_err(refuse)?;
        return Ok((Meets::At(found.into_iter().filter(|r| kept(surface,tol,curve.point(r.at))).map(|r| (r.at,r.touch)).collect()),proved))
    }
    // a surface swept by a curve, against that curve: two unknowns
    if let Surface::Extrusion(f,c) | Surface::Revolution(f,c) = surface {
        let turned = matches!(surface,Surface::Revolution(..));
        // a line along an extrusion's way stays one point of its profile's plane: on the surface
        // all along (`along` has read it) or nowhere
        if let (false,&Curve::Line {d,..}) = (turned,curve) { if norm(crate::util::space::cross(d,f.z)) <= 1e-12*norm(d) { return Ok((Meets::At(Vec::new()),true)) } }
        let Some([pu,pv]) = reach_of(surface,&whole) else { return Err(Uncertified::NoForm) };
        let sys = Profiled {curve,profile:c,frame:*f,turned,range:[[a,b],if turned { pv } else { pu }]};
        let reach = (0..5).map(|k| {
            let x = if k == 4 { [0.5;2] } else { [(k&1) as f64,((k>>1)&1) as f64] };
            let (_,j) = crate::util::isolate::System::at(&sys,x);
            (0..2).map(|c| j[0][c].abs()+j[1][c].abs()).fold(0.,f64::max)
        }).fold(0.,f64::max);
        let scale = if turned { 1.+distance(curve.point(a),f.o).max(distance(curve.point(b),f.o)) } else { 1. };
        let width = tol*1e-2*scale/reach.max(1e-300);
        let found = crate::util::isolate::solve::<2>(&sys,[[0.,1.];2],width,&|x| sys.near(x,tol)).map_err(refuse)?;
        return Ok((Meets::At(found.into_iter().map(|r| (sys.natural(0,r.at[0].clamp(0.,1.)),r.touch)).collect()),proved && c.enclose([sys.range[1]]).proved))
    }
    let Some([pu,pv]) = reach_of(surface,&whole) else { return Err(Uncertified::NoForm) };
    // the meeting over one box of the surface's parameters, isolated and certified
    let over = |pu: [f64;2],pv: [f64;2],[a,b]: [f64;2]| -> Result<Vec<(f64,bool)>,Uncertified> {
        let sys = Meeting {curve,surface,range:[[a,b],pu,pv],tol};
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
        Ok(found.into_iter().map(|r| (sys.natural(0,r.at[0].clamp(0.,1.)),r.touch)).collect())
    };
    // **a net read span by span**: its knot spans whose control points' box the curve can reach
    // (a span of a B-spline net lies in the hull of its poles), each isolated over its own
    // parameters — a fitted sheet of thousands of spans read whole encloses loosely and halves
    // deep, where the few a ray passes are each read in a handful of steps
    if let (Surface::BSpline(_,net),&Curve::Line {p,d}) = (surface,curve) {
        if net.weights.is_none() {
            // each span the line meets read whole, the line's stretch narrowed to the span's box,
            // so a crossing is read in the span holding it and never on the face of a box of the
            // search's own making. The box is grown by its own size (a flat span's is as thin as
            // it, and the line's stretch through it a hair beside the span's parameters: the
            // search's unknowns scaled a billion apart, it certifies nothing)
            let index = net_index(net);
            let mut boxes = Vec::new();
            index.tree.query(|bx| line_meets_box(p,d,[a,b],bx.lo.map(|x| x-tol),bx.hi.map(|x| x+tol)),|i| boxes.push(i));
            boxes.sort();
            let mut found = Vec::new();
            for i in boxes {
                let bx = index.boxes[i];
                let grow = (0..3).map(|k| bx.hi[k]-bx.lo[k]).fold(tol,f64::max);
                let Some(t) = line_in_box(p,d,[a,b],(bx.lo.map(|x| x-grow),bx.hi.map(|x| x+grow))) else { continue };
                // the span's parameters narrowed to where the line can reach it (`bezier_reach`)
                let Some([su,sv]) = bezier_reach(net,index.spans[i],p,d,tol) else { continue };
                found.extend(over(su,sv,t)?);
            }
            // a crossing on the knot line two spans share is found by each (each box is read grown
            // past its sides): the two are one
            found.sort_by(|x,y| x.0.total_cmp(&y.0));
            let same = tol/norm(d).max(1e-300);
            let mut merged: Vec<(f64,bool)> = Vec::with_capacity(found.len());
            for (t,touch) in found {
                match merged.last_mut() {
                    Some(last) if t-last.0 <= same => last.1 &= touch,
                    _ => merged.push((t,touch)),
                }
            }
            return Ok((Meets::At(merged),proved))
        }
    }
    // **any other curve read once over the whole of the surface's parameters**, and against a net
    // where that reads past any search's boxes, **over the spans the curve can reach** (`reached`):
    // a circle along a quintic tube's rim, read over the whole net, may. The whole first: a curve
    // along a sheet across many spans is one run proved over the whole, which a box cutting the
    // run proves less readily, or not at all
    match (over(pu,pv,[a,b]),surface) {
        (Err(Uncertified::Gave(GaveUp::Most)),Surface::BSpline(_,net)) if net.weights.is_none() =>
            Ok((Meets::At(reached(curve,[a,b],net,tol,&over)?),proved)),
        (found,_) => Ok((Meets::At(found?),proved)),
    }
}

/// How many times `reached` halves a curve's stretch at most.
const CURVE_DEEPEST: usize = 6;

/// **Where a curve meets a net, read over the spans it can reach**: its stretch `[a, b]` halved
/// while its enclosure reaches a span's box (within `tol`, `net_index`) and is wider than the
/// narrowest it reaches, to `CURVE_DEEPEST` halvings, a piece reaching none dropped (the net lies in
/// its spans' boxes, so it meets nothing there); and the net read once by `over` over the hull of
/// the spans the pieces reach, the curve over their stretch.
fn reached(curve: &Curve,[a,b]: [f64;2],net: &std::sync::Arc<crate::nurbs::Net>,tol: f64,
           over: &dyn Fn([f64;2],[f64;2],[f64;2]) -> Result<Vec<(f64,bool)>,Uncertified>) -> Result<Vec<(f64,bool)>,Uncertified> {
    use crate::util::bvh::Bounds;
    let index = net_index(net);
    let widest = |lo: V,hi: V| (0..3).map(|k| hi[k]-lo[k]).fold(0.,f64::max);
    let near = |lo: V,hi: V| Bounds {lo:lo.map(|x| x-tol),hi:hi.map(|x| x+tol)};
    let (mut t,mut spans) = ([f64::INFINITY,f64::NEG_INFINITY],[[f64::INFINITY,f64::NEG_INFINITY];2]);
    let mut stack = vec![([a,b],0)];
    while let Some((s,depth)) = stack.pop() {
        let e = curve.enclose([s]).at;
        let (lo,hi): (V,V) = (e.map(|x| x.lo()),e.map(|x| x.hi()));
        if !lo.iter().chain(&hi).all(|x| x.is_finite()) { return Err(Uncertified::Gave(GaveUp::Most)) }
        let mut reached: Vec<usize> = Vec::new();
        index.tree.query_box(near(lo,hi),|i| reached.push(i));
        if reached.is_empty() { continue }
        let narrowest = reached.iter().map(|&i| widest(index.boxes[i].lo,index.boxes[i].hi)).fold(f64::INFINITY,f64::min);
        if widest(lo,hi) > narrowest.max(tol) && depth < CURVE_DEEPEST {
            let m = 0.5*(s[0]+s[1]);
            stack.extend([([m,s[1]],depth+1),([s[0],m],depth+1)]);
            continue
        }
        t = [t[0].min(s[0]),t[1].max(s[1])];
        for i in reached { for k in 0..2 { spans[k] = [spans[k][0].min(index.spans[i][k][0]),spans[k][1].max(index.spans[i][k][1])] } }
    }
    if !(t[0] <= t[1]) { return Ok(Vec::new()) }
    over(spans[0],spans[1],t)
}

/// **A net's knot spans indexed**: each span's parameters, boxed by its Bézier patch's control
/// points (`blossom`: the hull its surface lies in, close about it, where the net's own poles reach
/// over five spans each way), in a bounding-volume hierarchy a ray is read through (`general`), a
/// point is placed near (`Located::solid_place`) and a foot is sought in (`geom::net_inverse`).
pub(crate) struct NetIndex { pub spans: Vec<[[f64;2];2]>,pub boxes: Vec<crate::util::bvh::Bounds<3>>,pub tree: crate::util::bvh::Bvh<3> }

/// Whether the line through `p` along `d` over `[a, b]` meets the box `lo`–`hi`.
fn line_meets_box(p: V,d: V,ab: [f64;2],lo: V,hi: V) -> bool { line_in_box(p,d,ab,(lo,hi)).is_some() }

/// How many nets' indexes are kept at once (`net_index`): past it the store is emptied.
const INDEXED: usize = 256;

/// **The index of `net`'s spans**, built once for the net and kept, keyed by the net it was built
/// for (held, so no other net takes its place).
pub(crate) fn net_index(net: &std::sync::Arc<crate::nurbs::Net>) -> std::sync::Arc<NetIndex> {
    type Store = std::collections::BTreeMap<usize,(std::sync::Arc<crate::nurbs::Net>,std::sync::Arc<NetIndex>)>;
    static STORE: std::sync::Mutex<Store> = std::sync::Mutex::new(std::collections::BTreeMap::new());
    let key = std::sync::Arc::as_ptr(net) as usize;
    if let Some((_,i)) = STORE.lock().unwrap().get(&key) { return i.clone() }
    let index = std::sync::Arc::new(indexed(net));
    let mut store = STORE.lock().unwrap();
    if store.len() >= INDEXED { store.clear() }
    store.insert(key,(net.clone(),index.clone()));
    index
}

/// `net`'s spans, each boxed by its Bézier points, indexed (`NetIndex`): each row's poles carried to
/// their Bézier points along each span of v once, then each span's along u.
fn indexed(net: &crate::nurbs::Net) -> NetIndex {
    let (du,dv) = (net.du,net.dv);
    let (nu,nv) = (net.poles.len(),net.poles[0].len());
    let spans = |knots: &[f64],d: usize,n: usize| -> Vec<([f64;2],usize)> {
        (d..n).filter(|&k| knots[k+1] > knots[k]).map(|k| ([knots[k],knots[k+1]],k)).collect()
    };
    let (us,vs) = (spans(&net.uknots,du,nu),spans(&net.vknots,dv,nv));
    // rows[i][s]: row i's Bézier points over v's span s
    let rows: Vec<Vec<Vec<V>>> = (0..nu).map(|i| vs.iter().map(|&(sv,l)| {
        (0..=dv).map(|j| blossom(&net.vknots,&net.poles[i][l-dv..=l],l,dv,sv,j)).collect()
    }).collect()).collect();
    let mut out = Vec::with_capacity(us.len()*vs.len());
    let mut bounds = Vec::with_capacity(us.len()*vs.len());
    for &(su,k) in &us {
        for (s,&(sv,_)) in vs.iter().enumerate() {
            let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
            for j in 0..=dv {
                let col: Vec<V> = (k-du..=k).map(|i| rows[i][s][j]).collect();
                for i in 0..=du {
                    let q = blossom(&net.uknots,&col,k,du,su,i);
                    for c in 0..3 { lo[c] = lo[c].min(q[c]); hi[c] = hi[c].max(q[c]) }
                }
            }
            out.push([su,sv]);
            bounds.push(crate::util::bvh::Bounds {lo,hi});
        }
    }
    NetIndex {spans:out,tree:crate::util::bvh::Bvh::new(bounds.iter().copied()),boxes:bounds}
}

/// How finely a span's Bézier patch is halved toward a line through it, as a part of the span each
/// way (`bezier_reach`), before what is left of it is isolated and certified.
const BEZIER_FINEST: f64 = 1./64.;

/// **Where in a span's parameters a line can meet it**: the span of `net` over `[su, sv]` as its
/// Bézier patch (each control point a blossom of the net at the span's ends, `blossom`), halved —
/// the longer way in its parameters, by de Casteljau — while its control points stand both sides of
/// each of two planes holding the line through `p` along `d` (within `tol`); a patch whose points do
/// not cannot meet it (the patch lies in their hull). Halving stops at `BEZIER_FINEST` of the span,
/// and what is left is given as one box: the hull of the pieces left, grown by a piece each way
/// within the span, so no crossing stands on a face of the halving's making. `None`: none is left.
fn bezier_reach(net: &crate::nurbs::Net,[su,sv]: [[f64;2];2],p: V,d: V,tol: f64) -> Option<[[f64;2];2]> {
    let (du,dv) = (net.du,net.dv);
    let span = |knots: &[f64],deg: usize,n: usize,t: f64| (deg..n).rfind(|&k| knots[k] <= t && knots[k+1] > t).unwrap_or(deg);
    let (nu,nv) = (net.poles.len(),net.poles[0].len());
    let (k,l) = (span(&net.uknots,du,nu,su[0]),span(&net.vknots,dv,nv,sv[0]));
    // the patch: each row of the span's poles to its Bézier points along v, then each column along u
    let rows: Vec<Vec<V>> = (k-du..=k).map(|i| (0..=dv).map(|j| blossom(&net.vknots,&net.poles[i][l-dv..=l],l,dv,sv,j)).collect()).collect();
    let patch: Vec<Vec<V>> = (0..=du).map(|i| (0..=dv).map(|j| {
        let col: Vec<V> = rows.iter().map(|r| r[j]).collect();
        blossom(&net.uknots,&col,k,du,su,i)
    }).collect()).collect();
    let d = scale(d,1./norm(d).max(1e-300));
    let (n1,n2) = crate::util::space::across(d);
    let straddles = |pts: &[Vec<V>]| {
        let (mut a1,mut b1,mut a2,mut b2) = (f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,f64::NEG_INFINITY);
        for q in pts.iter().flatten() {
            let r = crate::util::space::sub(*q,p);
            let (x,y) = (dot(r,n1),dot(r,n2));
            a1 = a1.min(x); b1 = b1.max(x); a2 = a2.min(y); b2 = b2.max(y);
        }
        a1 <= tol && b1 >= -tol && a2 <= tol && b2 >= -tol
    };
    let mut out: Option<[[f64;2];2]> = None;
    let mut stack = vec![(patch,su,sv)];
    while let Some((pts,bu,bv)) = stack.pop() {
        if !straddles(&pts) { continue }
        let (wu,wv) = ((bu[1]-bu[0])/(su[1]-su[0]),(bv[1]-bv[0])/(sv[1]-sv[0]));
        if wu <= BEZIER_FINEST && wv <= BEZIER_FINEST {
            let (gu,gv) = (bu[1]-bu[0],bv[1]-bv[0]);
            let grown = [[(bu[0]-gu).max(su[0]),(bu[1]+gu).min(su[1])],[(bv[0]-gv).max(sv[0]),(bv[1]+gv).min(sv[1])]];
            out = Some(match out {
                None => grown,
                Some([[u0,u1],[v0,v1]]) => [[u0.min(grown[0][0]),u1.max(grown[0][1])],[v0.min(grown[1][0]),v1.max(grown[1][1])]],
            });
            continue
        }
        if wu >= wv {
            // halve along u: each column of control points split by de Casteljau
            let (mut lo,mut hi) = (vec![vec![[0.;3];dv+1];du+1],vec![vec![[0.;3];dv+1];du+1]);
            for j in 0..=dv {
                let (a,b) = casteljau(&pts.iter().map(|r| r[j]).collect::<Vec<_>>());
                for i in 0..=du { lo[i][j] = a[i]; hi[i][j] = b[i] }
            }
            let m = 0.5*(bu[0]+bu[1]);
            stack.push((lo,[bu[0],m],bv));
            stack.push((hi,[m,bu[1]],bv));
        } else {
            let (mut lo,mut hi) = (Vec::with_capacity(du+1),Vec::with_capacity(du+1));
            for r in &pts { let (a,b) = casteljau(r); lo.push(a); hi.push(b) }
            let m = 0.5*(bv[0]+bv[1]);
            stack.push((lo,bu,[bv[0],m]));
            stack.push((hi,bu,[m,bv[1]]));
        }
    }
    out
}

/// A Bézier curve's control points split at its middle: the two halves' (de Casteljau).
fn casteljau(pts: &[V]) -> (Vec<V>,Vec<V>) {
    let n = pts.len();
    let mut work = pts.to_vec();
    let (mut lo,mut hi) = (vec![work[0]],vec![work[n-1]]);
    for r in 1..n {
        for m in 0..n-r { work[m] = std::array::from_fn(|c| 0.5*(work[m][c]+work[m+1][c])) }
        lo.push(work[0]);
        hi.push(work[n-1-r]);
    }
    hi.reverse();
    (lo,hi)
}


/// A surface's algebraic form and its gradient at `x` (`algebraic`'s, in its frame's coordinates,
/// the gradient carried back to space), written once over `Num`: read in floats, or in intervals
/// over a box. `None` for a surface with none.
fn form<T: Num>(surface: &Surface,x: [T;3]) -> Option<(T,[T;3])> {
    let f = surface.frame();
    let c = |v: f64| T::of(v);
    let q = local(&f,x);
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
    Some((value,global(&f,g)))
}

/// A point's coordinates in frame `f`, written over `Num`.
fn local<T: Num>(f: &super::geom::Frame,x: [T;3]) -> [T;3] {
    let c = |v: f64| T::of(v);
    let d: [T;3] = std::array::from_fn(|i| x[i]-c(f.o[i]));
    let along = |e: V| d[0]*c(e[0])+d[1]*c(e[1])+d[2]*c(e[2]);
    [along(f.x),along(f.y),along(f.z)]
}

/// A gradient read in frame `f` carried back to space, written over `Num`.
fn global<T: Num>(f: &super::geom::Frame,g: [T;3]) -> [T;3] {
    let c = |v: f64| T::of(v);
    std::array::from_fn(|i| g[0]*c(f.x[i])+g[1]*c(f.y[i])+g[2]*c(f.z[i]))
}

/// **A surface's algebraic form along a curve** (`general`'s one unknown): `F(C(t))` and its rate,
/// enclosed from the curve's enclosure (`enclose`) and the form read in intervals (`form`).
struct Implicit<'a> { curve: &'a Curve,surface: &'a Surface,tol: f64,held: std::cell::Cell<Option<([u64;2],crate::enclose::Patch<1>)>> }

impl<'a> Implicit<'a> {
    fn new(curve: &'a Curve,surface: &'a Surface,tol: f64) -> Implicit<'a> { Implicit {curve,surface,tol,held:std::cell::Cell::new(None)} }
    /// The curve's enclosure over `[a, b]`, the last one held (a stretch's run test and its slope
    /// read the same one).
    fn over(&self,a: f64,b: f64) -> crate::enclose::Patch<1> {
        let key = [a.to_bits(),b.to_bits()];
        if let Some((k,e)) = self.held.get() { if k == key { return e } }
        let e = self.curve.enclose([[a,b]]);
        self.held.set(Some((key,e)));
        e
    }
}

impl Run for Implicit<'_> {
    fn within(&self,a: f64,b: f64) -> bool { crate::util::isolate::Function::within(self,a,b) }
}

impl crate::util::isolate::Function for Implicit<'_> {
    fn at(&self,t: f64) -> (f64,f64) {
        let (x,d) = self.curve.d1(t);
        form(self.surface,x).map_or((f64::NAN,f64::NAN),|(f,g)| (f,dot(g,d)))
    }
    fn value(&self,t: f64) -> Iv { form(self.surface,self.curve.enclose([[t,t]]).at).map_or(Iv::ANY,|(f,_)| f) }
    fn slope(&self,a: f64,b: f64) -> Iv {
        let e = self.over(a,b);
        form(self.surface,e.at).map_or(Iv::ANY,|(_,g)| crate::enclose::dot3(g,e.d[0]))
    }
    /// Within the tolerance all along: the form over the curve's enclosure within half the
    /// tolerance times the least its gradient's length can be there (to first order, the surface's
    /// distance).
    fn within(&self,a: f64,b: f64) -> bool {
        let e = self.over(a,b);
        let Some((f,g)) = form(self.surface,e.at) else { return false };
        banded(f,g,self.tol)
    }
}

/// **Whether a form enclosed as `f` over a stretch, its gradient as `g`, is within `tol` of nought
/// there**: within half the tolerance times the least the gradient's length can be (to first order,
/// the surface's distance) — the one rule a run along an algebraic surface is proved by
/// (`Implicit`, `Taylored`).
fn banded(f: Iv,g: [Iv;3],tol: f64) -> bool {
    let least = |x: Iv| if x.has_zero() { 0. } else { x.lo().abs().min(x.hi().abs()) };
    let grad = g.iter().map(|&x| least(x)*least(x)).sum::<f64>().sqrt();
    f.mag() <= 0.5*tol*grad*(1.-1e-9)
}

/// **A curve's point and a surface's equal** (`general`'s three unknowns): `C(t) − S(u, v)` over
/// the stretches `range` (the curve's, then the surface's), each unknown read from nought to one
/// across its own, the Jacobian `[C', −S_u, −S_v]` scaled by them, enclosed from the two kinds'
/// enclosures (`enclose`).
struct Meeting<'a> { curve: &'a Curve,surface: &'a Surface,range: [[f64;2];3],tol: f64 }

/// Unknown `k`'s own parameter at `x` across its stretch of `range` (a search reads each unknown
/// from nought to one: `Meeting`'s, `Profiled`'s).
fn natural<const N: usize>(range: &[[f64;2];N],k: usize,x: f64) -> f64 { let [a,b] = range[k]; a+x*(b-a) }

/// The parameters a box of the unknowns reaches across `range`, outward.
fn natural_box<const N: usize>(range: &[[f64;2];N],bx: &[[f64;2];N]) -> [[f64;2];N] {
    std::array::from_fn(|k| {
        let [a,b] = range[k];
        let x = Iv::point(a)+Iv::new(bx[k][0],bx[k][1])*(Iv::point(b)-Iv::point(a));
        [x.lo().max(a.min(b)),x.hi().min(a.max(b))]
    })
}

/// Each unknown's stretch's length across `range`, enclosed.
fn widths<const N: usize>(range: &[[f64;2];N]) -> [Iv;N] { std::array::from_fn(|k| Iv::point(range[k][1])-Iv::point(range[k][0])) }

impl Meeting<'_> {
    fn natural(&self,k: usize,x: f64) -> f64 { natural(&self.range,k,x) }
    /// The feet of `p` on the surface from `uv`, by Gauss–Newton held to the surface's stretches,
    /// and how far it stands off there.
    fn foot(&self,p: V,mut uv: [f64;2]) -> ([f64;2],f64) {
        use crate::util::space::sub;
        let r = [self.range[1],self.range[2]];
        for _ in 0..12 {
            let (s,su,sv) = self.surface.d1(uv);
            let e = sub(p,s);
            let (a,b,c) = (dot(su,su),dot(su,sv),dot(sv,sv));
            let (g0,g1) = (dot(e,su),dot(e,sv));
            let det = a*c-b*b;
            if !(det.abs() > 1e-300) { break }
            let step = [(c*g0-b*g1)/det,(a*g1-b*g0)/det];
            let next = [(uv[0]+step[0]).clamp(r[0][0].min(r[0][1]),r[0][0].max(r[0][1])),(uv[1]+step[1]).clamp(r[1][0].min(r[1][1]),r[1][0].max(r[1][1]))];
            let moved = (next[0]-uv[0]).abs()+(next[1]-uv[1]).abs();
            uv = next;
            if moved <= 1e-15*(1.+uv[0].abs()+uv[1].abs()) { break }
        }
        (uv,distance(self.surface.point(uv),p))
    }
    /// Whether the curve over `[ta, tb]` keeps within the tolerance of the surface along the
    /// straight path of its parameters from `fa` to `fb`, proved in intervals.
    fn piece(&self,[ta,tb]: [f64;2],fa: [f64;2],fb: [f64;2]) -> bool {
        use crate::enclose::{add3,scale3,sub3};
        let (lo,hi) = (ta.min(tb),ta.max(tb));
        let span = Iv::point(tb)-Iv::point(ta);
        let rate = [Iv::point(fb[0])-Iv::point(fa[0]),Iv::point(fb[1])-Iv::point(fa[1])].map(|d| d/span);
        let at = |t: Iv| [Iv::point(fa[0])+rate[0]*(t-Iv::point(ta)),Iv::point(fa[1])+rate[1]*(t-Iv::point(ta))];
        let mid = 0.5*(lo+hi);
        let pm = at(Iv::point(mid));
        let (Some(um),Some(vm)) = (pm[0].bounds(),pm[1].bounds()) else { return false };
        let value = sub3(self.curve.enclose([[mid,mid]]).at,self.surface.enclose([um,vm]).at);
        let all = at(Iv::new(lo,hi));
        let (Some(ua),Some(va)) = (all[0].bounds(),all[1].bounds()) else { return false };
        let (c,s) = (self.curve.enclose([[lo,hi]]),self.surface.enclose([ua,va]));
        let slope = sub3(c.d[0],add3(scale3(s.d[0],rate[0]),scale3(s.d[1],rate[1])));
        let half = Iv::new(lo,hi)-Iv::point(mid);
        let g: [Iv;3] = std::array::from_fn(|i| value[i]+slope[i]*half);
        let bound = self.tol/3f64.sqrt();
        g.iter().all(|x| x.mag() <= bound)
    }
    /// `run`, the stretch in the search's own unknowns.
    fn along_run(&self,x: [f64;3]) -> Option<[f64;2]> {
        let [t0,t1] = self.range[0];
        let (lo,hi) = (t0.min(t1),t0.max(t1));
        let t = self.natural(0,x[0]);
        let f0 = [self.natural(1,x[1]),self.natural(2,x[2])];
        // (as fine as the search reads: the tolerance's hundredth over the curve's speed)
        let resolution = (1e-2*self.tol/norm(self.curve.d1(t).1).max(1e-300)).max(1e-15*(hi-lo).max(1e-300));
        let reach = |way: f64| -> f64 {
            let (mut at,mut foot) = (t,f0);
            let mut step = 1e-6*(hi-lo);
            let mut pieces = 0;
            loop {
                let next = (at+way*step).clamp(lo,hi);
                if next == at || pieces > 4096 { return at }
                let (fb,off) = self.foot(self.curve.point(next),foot);
                if off <= self.tol && self.piece([at,next],foot,fb) {
                    at = next;
                    foot = fb;
                    pieces += 1;
                    step *= 2.;
                } else {
                    step *= 0.5;
                    if step < resolution { return at }
                }
            }
        };
        let (a,b) = (reach(-1.),reach(1.));
        if !(b > a) { return None }
        let x = |s: f64| (s-t0)/(t1-t0);
        let (xa,xb) = (x(a),x(b));
        Some([xa.min(xb),xa.max(xb)])
    }
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
        let n = natural_box(&self.range,&x.map(|y| [y,y]));
        let (c,s) = (self.curve.enclose([n[0]]),self.surface.enclose([n[1],n[2]]));
        crate::enclose::sub3(c.at,s.at)
    }
    fn over(&self,bx: &[[f64;2];3]) -> ([Iv;3],[[Iv;3];3]) {
        let n = natural_box(&self.range,bx);
        let (c,s) = (self.curve.enclose([n[0]]),self.surface.enclose([n[1],n[2]]));
        let w = widths(&self.range);
        (crate::enclose::sub3(c.at,s.at),std::array::from_fn(|i| [c.d[0][i]*w[0],-(s.d[0][i]*w[1]),-(s.d[1][i]*w[2])]))
    }
    /// **A run of the curve along the surface through `x`** (#814: what no box can part): from `x`,
    /// each way, the curve read piece by piece against the surface along a straight path of its
    /// parameters between the feet of the piece's ends — each found by Gauss–Newton — and `C(t) −
    /// S(path(t))` proved within the tolerance over the piece in intervals: its value at the
    /// piece's middle and its rate `C' − S_u u' − S_v v'` over the piece (the two tangents cancel
    /// along the run, so a piece may be long). Pieces doubled while each is proved, halved where one
    /// is not, the run ending where a piece no wider than the search's resolution is not, or the
    /// stretch ends. `None` where no piece is proved.
    fn run(&self,x: [f64;3]) -> Option<[f64;2]> { self.along_run(x) }
    /// The surface's normal at the box's middle, and two tangents: what a curve all but tangent to
    /// it is told off it along.
    fn frame(&self,x: [f64;3]) -> Option<[[f64;3];3]> {
        use crate::util::space::{cross,normalised};
        let (_,su,sv) = self.surface.d1([self.natural(1,x[1]),self.natural(2,x[2])]);
        let n = normalised(cross(su,sv))?;
        let e1 = normalised(su)?;
        Some([n,e1,cross(n,e1)])
    }
}

/// **A curve against a surface swept by a curve** (`general`'s two unknowns): an extrusion's
/// profile `P(s)` along its way — the curve's point and the profile's equal seen along the way,
/// `[q₀ − p₀, q₁ − p₁]` in the frame — or a revolution's meridian `M(s)` about its axis — the curve's
/// point at the meridian's height and radius, `[q₀² + q₁² − (m₀² + m₁²), q₂ − m₂]`, squared so no
/// root is read and the axis is no singularity — each unknown read from nought to one across its
/// stretch (`range`: the curve's, then the profile's), enclosed from the two curves' enclosures.
struct Profiled<'a> { curve: &'a Curve,profile: &'a Curve,frame: super::geom::Frame,turned: bool,range: [[f64;2];2] }

impl Profiled<'_> {
    fn natural(&self,k: usize,x: f64) -> f64 { natural(&self.range,k,x) }
    /// `G` and its Jacobian from the two curves' points and rates in the frame, `w` the stretches'
    /// lengths: written once over `Num`.
    fn of<T: Num>(&self,q: [T;3],dq: [T;3],m: [T;3],dm: [T;3],w: [T;2]) -> ([T;2],[[T;2];2]) {
        if self.turned {
            let two = T::of(2.);
            ([q[0]*q[0]+q[1]*q[1]-(m[0]*m[0]+m[1]*m[1]),q[2]-m[2]],
             [[two*(q[0]*dq[0]+q[1]*dq[1])*w[0],-(two*(m[0]*dm[0]+m[1]*dm[1])*w[1])],[dq[2]*w[0],-(dm[2]*w[1])]])
        } else {
            ([q[0]-m[0],q[1]-m[1]],[[dq[0]*w[0],-(dm[0]*w[1])],[dq[1]*w[0],-(dm[1]*w[1])]])
        }
    }
    /// Whether the curve's point at `x` is within `tol` of the surface's there.
    fn near(&self,x: [f64;2],tol: f64) -> bool {
        let f = &self.frame;
        let (q,m) = (f.local(self.curve.point(self.natural(0,x[0]))),f.local(self.profile.point(self.natural(1,x[1]))));
        use crate::util::fmath::Det;
        if self.turned { (q[0].dhypot(q[1])-m[0].dhypot(m[1])).abs().max((q[2]-m[2]).abs()) <= tol }
        else { (q[0]-m[0]).dhypot(q[1]-m[1]) <= tol }
    }
}

impl crate::util::isolate::System<2> for Profiled<'_> {
    fn at(&self,x: [f64;2]) -> ([f64;2],[[f64;2];2]) {
        let f = &self.frame;
        let ((q,dq),(m,dm)) = (self.curve.d1(self.natural(0,x[0])),self.profile.d1(self.natural(1,x[1])));
        let w = [self.range[0][1]-self.range[0][0],self.range[1][1]-self.range[1][0]];
        self.of(f.local(q),f.dir_local(dq),f.local(m),f.dir_local(dm),w)
    }
    fn value(&self,x: [f64;2]) -> [Iv;2] {
        let n = natural_box(&self.range,&x.map(|y| [y,y]));
        let (c,p) = (self.curve.enclose([n[0]]),self.profile.enclose([n[1]]));
        let f = &self.frame;
        let one = Iv::point(1.);
        self.of(crate::enclose::frame_local(f,c.at),crate::enclose::frame_dir_local(f,c.d[0]),
            crate::enclose::frame_local(f,p.at),crate::enclose::frame_dir_local(f,p.d[0]),[one,one]).0
    }
    fn over(&self,bx: &[[f64;2];2]) -> ([Iv;2],[[Iv;2];2]) {
        let n = natural_box(&self.range,bx);
        let (c,p) = (self.curve.enclose([n[0]]),self.profile.enclose([n[1]]));
        let f = &self.frame;
        let w = widths(&self.range);
        self.of(crate::enclose::frame_local(f,c.at),crate::enclose::frame_dir_local(f,c.d[0]),
            crate::enclose::frame_local(f,p.at),crate::enclose::frame_dir_local(f,p.d[0]),w)
    }
}

/// **The stretch a reading searches** (`general`): `[t0, t1]` where it is bounded; a line's with no
/// end, the stretch of it within the surface's own box (its enclosure over all its parameters,
/// grown by the tolerance), where the surface is bounded. `None` where neither.
fn bounded_stretch(curve: &Curve,[t0,t1]: [f64;2],surface: &Surface,tol: f64) -> Option<[f64;2]> {
    if t0.is_finite() && t1.is_finite() { return Some([t0,t1]) }
    let &Curve::Line {p,d} = curve else { return None };
    let e = surface.enclose(domain(surface)?);
    let (lo,hi): (V,V) = (e.at.map(|x| x.lo()-tol),e.at.map(|x| x.hi()+tol));
    if !lo.iter().chain(&hi).all(|x| x.is_finite()) { return None }
    Some(line_in_box(p,d,[t0,t1],(lo,hi)).unwrap_or([0.,0.]))
}

/// The stretch of the line `p + t d` over `[t0, t1]` within the box `(lo, hi)`, slab by slab; `None`
/// where it misses it.
fn line_in_box(p: V,d: V,[t0,t1]: [f64;2],(lo,hi): (V,V)) -> Option<[f64;2]> {
    let (mut a,mut z) = (t0,t1);
    for k in 0..3 {
        if d[k].abs() < 1e-300 { if p[k] < lo[k] || p[k] > hi[k] { return None } continue }
        let (u,v) = ((lo[k]-p[k])/d[k],(hi[k]-p[k])/d[k]);
        a = a.max(u.min(v));
        z = z.min(u.max(v));
    }
    (a < z).then_some([a,z])
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
        Curve::Trochoid(m) => { let [a,b] = m.span; [a.min(b),a.max(b)] }
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

/// **A line against the sheet a straight edge is carried into** (a polyhedral tool's edge screwed,
/// `Blend` between two lines), its clean crossings in closed form: the sheet is ruled, `S(u, v) =
/// Q(v) + u W(v)` — `Q` the carried start of the section's edge at `v` and `W` its carried run, the
/// carry rigid — so the line `p + t d` meets the ruling at `v` exactly where the three are
/// coplanar, `F(v) = (Q(v) − p) · (d × W(v)) = 0`, written once over `Num` (`Ruled`) and isolated
/// by its enclosures (`util::isolate`). At a root `t` and `u` are read off `Q − p = t d − u W`.
/// `F'(v) = det[d, W, S_v]` there, so a double root is a line tangent to the sheet. Only the
/// clean reading is answered here — every root simple, the line across its ruling, each meeting
/// clear inside the sheet's square and the line's stretch or clear outside it — and anything else
/// is the general reading's (`NoForm`): a touch, and a meeting at an end of the sheet or the line,
/// are read by its rules alone. The general reading isolated the three unknowns at once, a ray
/// of a point's location against a screwed sheet the slowest read of a sweep's build.
fn line_screwed_line(p: V,d: V,[t0,t1]: [f64;2],b: &super::geom::Blend,tol: f64) -> Certified {
    use crate::util::space::{cross,sub};
    let (&Curve::Line {d:da,..},&Curve::Line {d:db,..}) = (&b.a,&b.b) else { return Err(Uncertified::NoForm) };
    if !(t0.is_finite() && t1.is_finite() && t1 > t0) { return Err(Uncertified::NoForm) }
    // the section's edge at `v` before it is carried, `M(v) + u N(v)`, each affine in `v`
    let (a0,b0) = (b.a.point(b.ta[0]),b.b.point(b.tb[0]));
    let (al,be) = (scale(da,b.ta[1]-b.ta[0]),scale(db,b.tb[1]-b.tb[0]));
    let f = Ruled::new(b.carry,[a0,sub(b0,a0)],[al,sub(be,al)],p,d).ok_or(Uncertified::NoForm)?;
    // the sheet's speed along `v` and `u`, the most at its corners and middle: a root's resolution
    let [speed,reach] = [[0.,0.],[1.,0.],[0.,1.],[1.,1.],[0.5,0.5]].iter().map(|&uv| b.d1(uv))
        .fold([0f64;2],|[s,r],(_,su,sv)| [s.max(norm(sv)),r.max(norm(su))]);
    let dd = norm(d);
    if !(speed > 0. && reach > 0. && dd > 0.) { return Err(Uncertified::NoForm) }
    // how near an end of the sheet or the line a meeting may stand and still be read here: far
    // wider than the general reading's tolerance, so that whatever it would read differently
    // there is its own to read
    let slack = 1e-6+1e3*tol/speed.min(reach);
    let g = crate::util::isolate::Written(f);
    let gv = |v: f64| { use crate::util::isolate::Function; g.at(v).0 };
    let width = tol*1e-3/speed;
    let ts = slack*(t1-t0)+1e3*tol/dd;
    let clear_in = |x: f64,lo: f64,hi: f64,s: f64| x > lo+s && x < hi-s;
    let clear_out = |x: f64,lo: f64,hi: f64,s: f64| x < lo-s || x > hi+s;
    let found = crate::util::isolate::roots(&g,[-slack,1.+slack],width,&|v| gv(v).abs() <= tol*dd*reach)
        .map_err(|e| if matches!(e,GaveUp::Stopped) { Uncertified::Stopped } else { Uncertified::NoForm })?;
    let mut roots = Vec::new();
    for r in found {
        if r.touch { return Err(Uncertified::NoForm) }
        let v = r.at;
        // the ruling there: its start `Q` and its run `W`, the sheet at `u = 0`
        let (q,w,_) = b.d1([0.,v]);
        let n = cross(d,w);
        let nn = dot(n,n);
        // the line along the ruling there: the general reading's
        if !(nn > 1e-12*dd*dd*dot(w,w)) { return Err(Uncertified::NoForm) }
        let qp = sub(q,p);
        let (t,u) = (dot(cross(qp,w),n)/nn,dot(cross(qp,d),n)/nn);
        if clear_out(v,0.,1.,slack) || clear_out(u,0.,1.,slack) || clear_out(t,t0,t1,ts) { continue }
        if !(clear_in(v,0.,1.,slack) && clear_in(u,0.,1.,slack) && clear_in(t,t0,t1,ts)) { return Err(Uncertified::NoForm) }
        roots.push((t,false));
    }
    Ok(Meets::At(roots))
}

/// `(Q(v) − p) · (d × W(v))` (`line_screwed_line`), `Q` the point `M₀ + v M₁` carried to `v` by the
/// screw (centre `c`, unit axis `k`, turn `θ`, rise `h`) and `W` the direction `N₀ + v N₁` turned
/// to it. Turned back by `R_v` the screw's turn at `v` (the triple product unchanged), it is
/// `R_vᵀd · (N × A) + N · R_vᵀ((c − p) × d)`, `A = M − c + h v k`: with `R_vᵀx · y = (k·x)(k·y) +
/// cos θv (x·y − (k·x)(k·y)) − sin θv ((k × x)·y)`, **`α(v) + β(v) cos θv − γ(v) sin θv`**, each of
/// the three a quadratic in `v` — one sine and cosine, written once over `Num`, where reading `Q`
/// and `W` carried read five, and their product enclosed wide.
struct Ruled { trig: [[f64;3];3],theta: f64 }

impl Ruled {
    fn new(carry: super::geom::Carry,m: [V;2],n: [V;2],p: V,d: V) -> Option<Ruled> {
        use crate::util::space::{cross,sub};
        let super::geom::Carry::Screw {center,axis:k,angle,advance} = carry else { return None };
        if (norm(k)-1.).abs() > 1e-12 { return None }
        // `N × A` a quadratic, `N` affine: their coefficients by power of `v`
        let (a0,a1) = (sub(m[0],center),add(m[1],scale(k,advance)));
        let y1 = [cross(n[0],a0),add(cross(n[0],a1),cross(n[1],a0)),cross(n[1],a1)];
        let y2 = [n[0],n[1],[0.;3]];
        let (x1,x2) = (d,cross(sub(center,p),d));
        let trig = std::array::from_fn(|j| {
            let (kx1,kx2,ky1,ky2) = (dot(k,x1),dot(k,x2),dot(k,y1[j]),dot(k,y2[j]));
            [kx1*ky1+kx2*ky2,dot(x1,y1[j])-kx1*ky1+dot(x2,y2[j])-kx2*ky2,dot(cross(k,x1),y1[j])+dot(cross(k,x2),y2[j])]
        });
        Some(Ruled {trig,theta:angle})
    }
}

impl crate::util::isolate::Formula for Ruled {
    fn jet<T: Num>(&self,v: crate::util::interval::Jet<T>) -> crate::util::interval::Jet<T> {
        use crate::util::interval::Jet;
        let c = &self.trig;
        let poly = |i: usize| Jet::of(c[0][i])+v*(Jet::of(c[1][i])+v.times(c[2][i]));
        let (s,co) = v.times(self.theta).sin_cos();
        poly(0)+co*poly(1)-s*poly(2)
    }
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
        &Curve::Line {p,d} => stretches.extend(line_in_box(p,d,[t0,t1],(lo,hi))),
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
    /// The box its faces' boxes fill: past it no face stands.
    whole: ([f64;3],[f64;3]),
}

impl<'a> Located<'a> {
    /// A located boundary that keeps its own copy.
    pub fn owned(b: Brep,tol: f64) -> Located<'static> {
        let (tol,coarse,drawn,space,seams,edge_space,size,whole) = { let l = Located::new(&b,tol);
            (l.tol,l.coarse,l.drawn,l.space,l.seams,l.edge_space,l.size,l.whole) };
        Located {b:std::borrow::Cow::Owned(b),tol,coarse,drawn,space,seams,edge_space,size,whole}
    }
    pub fn new(b: &'a Brep,tol: f64) -> Located<'a> { Located::boxed(b,tol,&|_| None) }

    /// `new`, each face's box in space taken from `known` where it has one (`face_box` where not):
    /// a face a Boolean carried whole has the box it had (#589).
    pub(crate) fn boxed(b: &'a Brep,tol: f64,known: &dyn Fn(usize) -> Option<([f64;3],[f64;3])>) -> Located<'a> {
        let size = b.size();
        let coarse = (1e-4*size).max(tol*8.);
        let drawn = (0..b.faces.len()).map(|_| std::sync::OnceLock::new()).collect();
        let space: Vec<([f64;3],[f64;3])> = (0..b.faces.len()).map(|fi| known(fi).unwrap_or_else(|| face_box(b,fi))).collect();
        let mut whole = super::geom::EMPTY;
        for &bx in &space { super::geom::widen(&mut whole,bx) }
        let seams = b.faces.iter().map(Face::seams).collect();
        let edge_space = (0..b.edges.len()).map(|e| edge_box(b,e,size)).collect();
        Located {b:std::borrow::Cow::Borrowed(b),tol,coarse,drawn,space,seams,edge_space,size,whole}
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

    /// **Face `fi`'s box in its surface's parameters**: its loops' outline's, on the branch of its
    /// periods they are drawn on (`uv`).
    pub fn parameter_box(&self,fi: usize) -> [Uv;2] { self.drawn(fi).1 }

    /// **Whether the parameters `uv` lie in face `fi`**, read off its loops' outline in its
    /// parameters by their winding, on the branch they are drawn on (`uv`); no point is read in space.
    pub fn holds_parameters(&self,fi: usize,uv: Uv) -> bool {
        let winding = self.drawn(fi).0.winding(uv);
        if self.b.faces[fi].reversed { winding < 0 } else { winding > 0 }
    }

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
        // (whether a degenerate edge, a pole or an apex, is within the near-edge reading's reach)
        let mut pole = false;
        for l in &f.loops { for c in l {
            let (t,d) = self.nearest(c.edge,p);
            if !self.seams[fi].contains(&c.edge) { boundary = boundary.min(d); }
            if closest.is_none_or(|(best,_,_)| d < best) { closest = Some((d,c,t)); }
            pole |= d <= near && matches!(self.b.edges[c.edge as usize].curve,EdgeCurve::Degenerate);
        } }
        let Some((d,c,t)) = closest else { return Place::Out };
        if boundary <= self.tol { return Place::On }
        if d <= self.tol { return Place::In }
        let uv = self.uv(fi,p);
        // (past the rim, however near its edge)
        if past(uv) { return Place::Out }
        let e = &self.b.edges[c.edge as usize];
        // near a pole or an apex, a degenerate edge, every edge of the face meets in space, so the
        // nearest there need be no nearest in the face's parameters: a point of a cone's other nappe
        // just past its apex was read beside its seam and in the face (#721 G5). There the
        // parameters decide, the loops drawn in them
        if !pole && d <= near && t > e.t[0] && t < e.t[1] {
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
            // (a face whose box stands farther than the tolerance holds no point on it: its
            // surface's distance not asked, which a fitted sheet reads by searching its poles)
            let (lo,hi) = self.space[fi];
            if (0..3).any(|k| p[k] < lo[k]-self.tol || p[k] > hi[k]+self.tol) { continue }
            // a net's: where no span's box (its Bézier points', `net_index`) stands within it
            if !near_net(&f.surface,p,self.tol) { continue }
            if f.surface.implicit(p).abs() <= self.tol && self.face_place(fi,p) != Place::Out { return Place::On }
        }
        // past the box the faces fill no face stands: a point off it is outside, and a ray from a
        // point within it need reach only out of it. The ways out tried shortest first: from a hub
        // inside a gear's ring, a ray along the axis leaves by its own faces, where one across it
        // read every tooth's sheet
        if gap(self.whole,p) > self.tol { return Place::Out }
        let (lo,hi) = (self.whole.0.map(|x| x-self.tol),self.whole.1.map(|x| x+self.tol));
        const DIRECTIONS: [V;5] = [[0.5773,0.6124,0.5402],[-0.3172,0.8123,-0.4893],[0.7071,-0.3162,0.6325],
            [-0.6412,-0.5123,0.5712],[0.1234,-0.4321,-0.8934]];
        let mut ways = DIRECTIONS.map(|d| { let d = scale(d,1./norm(d)); (line_in_box(p,d,[0.,f64::INFINITY],(lo,hi)).map_or(0.,|s| s[1]),d) });
        ways.sort_by(|a,b| a.0.total_cmp(&b.0));
        'dirs: for (exit,d) in ways {
            let reach = exit+self.tol+1e-9*self.size;
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
/// nothing returned is a guess.  A cell thinner than its first mesh's sag everywhere has no point
/// that mesh can vouch for (the tip relief a hypoid's flank keeps where its mill leaves it a hundredth
/// of a millimetre thick, #904 A1/A2): it is read again on a mesh ten times finer (`SAGS`).
pub fn interior(b: &Brep,capacity: usize) -> Result<Vec<(V,f64)>,String> {
    let mut chosen = Vec::new();
    for sag in SAGS {
        chosen = interior_at(b,capacity,sag)?;
        if !chosen.is_empty() { break }
    }
    Ok(chosen)
}

/// The sags `interior` meshes a cell at in turn, as fractions of its box's diagonal: the first
/// where any sample clears it is the one read.
const SAGS: [f64;2] = [5e-4,5e-5];

/// `interior` on a mesh of `b` whose sag is `sag` of its box's diagonal.
fn interior_at(b: &Brep,capacity: usize,sag: f64) -> Result<Vec<(V,f64)>,String> {
    let (lo,hi) = b.bounds();
    let size = distance(lo,hi);
    let m = super::mesh::mesh(b,size*sag,0.2)?;
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
    // inside and no nearer a point taken than half the first's depth
    let mut order: Vec<usize> = (0..candidates.len()).filter(|&i| depth[i] > 0.).collect();
    order.sort_by(|&i,&j| depth[j].total_cmp(&depth[i]));
    let open = |chosen: &[(V,f64)],i: usize|
        !chosen.first().is_some_and(|f| chosen.iter().any(|c| distance(c.0,candidates[i]) < f.1/2.));
    // Whether each candidate is inside, as far as it is known. The winding number of a closed mesh
    // is constant off it, so one read where it is whole (to a millionth: a mesh open somewhere reads
    // fractions, and shares none) settles every candidate in the ball about it no triangle enters:
    // the deepest candidates of a thin cell are mostly outside it, its box's grid far from it, and
    // were read one by one, two thousand windings of every triangle for four points. The rest are
    // read a batch at a time across the cores, ahead of the choice; what is chosen is what reading
    // every one would choose (one at a time on one thread, so each read settles what it can before
    // the next is asked)
    let mut inside: Vec<Option<bool>> = vec![None;candidates.len()];
    let threads = crate::util::par::threads();
    let batch = if threads == 1 { 1 } else { 4*threads };
    let mut chosen: Vec<(V,f64)> = Vec::new();
    for (k,&i) in order.iter().enumerate() {
        if chosen.len() == capacity { break }
        if !open(&chosen,i) { continue }
        if inside[i].is_none() {
            let asked: Vec<usize> = order[k..].iter().copied().filter(|&j| inside[j].is_none() && open(&chosen,j)).take(batch).collect();
            let read = crate::util::par::map(&asked,|&j| winding(candidates[j]));
            for (&j,w) in asked.iter().zip(read) {
                inside[j] = Some(w > 0.5);
                if (w-w.round()).abs() > 1e-6 { continue }
                // (the ball's radius the candidate's distance from the mesh, a hair inside it; only
                // the candidates still ahead are ever asked)
                let r = (depth[j]+m.sag)*(1.-1e-9);
                for &q in &order[k..] {
                    let x = crate::util::space::sub(candidates[q],candidates[j]);
                    if inside[q].is_none() && dot(x,x) < r*r { inside[q] = Some(w > 0.5) }
                }
            }
        }
        if inside[i] == Some(true) { chosen.push((candidates[i],depth[i])) }
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
