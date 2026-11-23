//! **The tube road's faces sewn** (#880 T1f): each chart's kept region as faces, their loops made
//! of the traced curves with both faces' parameters, sewn by construction (`sew::sewn`).
//!
//! **The trims.** Each sheet's depth zero set (`seams`) is read as runs by what covers it — another
//! instant inside the roll (a double curve of the sheets, each traced twice, once on each of its
//! preimages: the run whose own instant is the later kept, its partner read off its cover), or an
//! end pose (a double curve of the sheet and the posed tool, traced on the sheet alone: on the end
//! pose it is the image). Every end of a run is an event found exactly, and each is one vertex of
//! everything meeting there:
//! * a **rim crossing** — a run leaving the sheet's rim, where it meets its end pose's contact
//!   circle: solved on the rim, one curve read in two charts, as the root along it of the far
//!   cover's depth (`crossing`), and both charts given that point; its partner on the covering
//!   instant is where the partner preimage's cover passes to the end pose (a **switch**);
//! * a **swallowtail point** — the double curve's two preimages joined to it inside its disc along
//!   the swallowtail's normal form (`rings::joined_by_normal_form`, the ring road's rule);
//! * a **corner** — where a cuspidal edge reaches the roll's end (`corner`, the root of `J` along
//!   the rim): there the sheet's double curve with the end pose meets the pose's contact circle,
//!   every far cover within the rounding, and the run is joined to it straight in the sheet's
//!   parameters, each point of the join held within the tolerance of the end pose.
//!
//! **The faces.** Every chart is cut across its angle at two angles clear of every trim (the
//! windows), and a sheet from each swallowtail point straight to the rim the swallowtail opens away
//! from (so no face meets itself along its double curve); the rims are split at every vertex on
//! them. Each window's edges are walked as a planar graph, and each part kept where its inner point
//! bounds the sweep (`Tube::side` above nought, and no other instant holding it). A sheet's face
//! lies on a net fitted over its window as a map (`nurbs::fit_net`), an end pose's on its torus.
use super::{Chart,Depth,Disc,Tube,edges,seams,swallowtails,swallowtails_on};
use super::cells::Axis;
use std::collections::BTreeMap;
use crate::geom::{Curve,Frame,Surface};
use crate::nurbs::BSpline;
use crate::swept::closed::Unbuilt;
use crate::swept::depth::{Cover,Reading};
use crate::topo::{Brep,Coedge,Edge,EdgeCurve,Face,Pcurve};
use crate::util::fmath::{Det,HALF_READ};
use crate::util::space::{distance,dot,sub};
use crate::zero_set::Field;
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

type V = [f64;3];
type P = [f64;2];

/// How finely the road fits its curves and nets, as a part of the tolerance (structural: what
/// `upright` and the ring road fit to, so the sewing's gaps are the tolerance's).
const FIT: f64 = 0.1;
/// The degree of every curve and net fitted.
pub const DEGREE: usize = 5;
/// The most points a fitted curve is read at (structural, as `upright`'s).
const MOST_FIT: usize = 4096;
/// Newton's steps a settling takes before it is read as not converging (structural, as
/// `zero_set`'s corrector's).
const CORRECTIONS: usize = 32;

fn debug() -> bool { std::env::var_os("SOLVENT_TUBE_DEBUG").is_some() }

fn failed(why: String) -> Unbuilt { Unbuilt::Failed(format!("the tube road: {why}")) }

impl Tube<'_> {
    /// **The end pose `k`'s parameters `(θ, ψ)` of a point on its torus**, in closed form (`ψ`
    /// round the tube from the outward way about the axis, as `Chart::End` reads it).
    pub fn on_end(&self,k: usize,x: V) -> Option<P> {
        let pose = self.path.pose(self.roll[k]).ok()?;
        let l = self.ring.frame.local(pose.inverse().point(x));
        let rho = l[0].dhypot(l[1]);
        Some([l[1].datan2(l[0]).rem_euclid(TAU),(l[2]-self.height).datan2(rho-self.radius)])
    }

    /// The sheet's parameters of its point `x`, from `uv` by Gauss–Newton on `X(θ, t) = x` (the
    /// sheet regular there), while the steps shrink.
    fn settled(&self,side: f64,x: V,mut uv: P) -> Option<P> {
        let mut prev = f64::INFINITY;
        for _ in 0..CORRECTIONS {
            let p = self.point(side,uv[0],uv[1])?;
            let r = sub(x,p.x);
            let (a,b,c) = (dot(p.x_th,p.x_th),dot(p.x_th,p.x_t),dot(p.x_t,p.x_t));
            let (g0,g1) = (dot(p.x_th,r),dot(p.x_t,r));
            let det = a*c-b*b;
            if !(det > 0.) { return None }
            let d = [(c*g0-b*g1)/det,(a*g1-b*g0)/det];
            let step = d[0].dhypot(d[1]);
            if !(step < prev) { break }
            prev = step;
            uv = [uv[0]+d[0],uv[1]+d[1]];
        }
        Some(uv)
    }

    /// **What covers the point `x` read at `c`, as a chart's parameters**: an end pose's, where the
    /// covering instant is an end of the roll (to its rounding); else the sheet whose point at the
    /// covering instant and the core circle's angle of the tool's point is the nearer, settled.
    fn partner(&self,x: V,c: &Cover) -> Option<(Chart,P)> {
        let [a,b] = self.roll;
        for (k,e) in [a,b].into_iter().enumerate() {
            if (c.at-e).abs() <= HALF_READ*(b-a) { return Some((Chart::End(k),self.on_end(k,x)?)) }
        }
        let l = self.ring.frame.local(c.point);
        let th = l[1].datan2(l[0]).rem_euclid(TAU);
        if let Some(uv) = self.in_collar(x,c) { return Some((Chart::Collar,uv)) }
        let side = [1.,-1.].into_iter().min_by(|&s,&r| {
            let d = |s: f64| self.point(s,th,c.at).map_or(f64::INFINITY,|p| distance(p.x,x));
            d(s).total_cmp(&d(r))
        })?;
        Some((Chart::Sheet(side),self.settled(side,x,[th,c.at])?))
    }

    /// **The collar's parameters of a point `x` its cover `c` names inside the pinch's hole**: `ψ`
    /// round the tube at the tool's point in closed form, `ρ` from `(A, B)` at the place read
    /// square to it, settled by Gauss–Newton on the collar's point (`settled_on`); none where the
    /// cover's place lies outside the hole.
    fn in_collar(&self,x: V,c: &Cover) -> Option<P> {
        let p = self.pinch.as_ref()?;
        let l = self.ring.frame.local(c.point);
        let th = l[1].datan2(l[0]).rem_euclid(TAU);
        if !(p.side([th,c.at]) < 0.) { return None }
        let psi = (l[2]-self.height).datan2(l[0].dhypot(l[1])-self.radius).rem_euclid(TAU);
        let [a,b] = p.collar.ab(th,c.at)?;
        let (sp,cp) = psi.dsin_cos();
        self.settled_on(Chart::Collar,x,[psi,-a*sp+b*cp])
    }

    /// A chart's parameters of its point `x`, from `uv` by Gauss–Newton on its point (`settled`'s,
    /// on any chart), while the steps shrink.
    fn settled_on(&self,chart: Chart,x: V,mut uv: P) -> Option<P> {
        let mut prev = f64::INFINITY;
        for _ in 0..CORRECTIONS {
            let p = self.at(chart,uv)?;
            let r = sub(x,p.x);
            let (a,b,c) = (dot(p.x_th,p.x_th),dot(p.x_th,p.x_t),dot(p.x_t,p.x_t));
            let (g0,g1) = (dot(p.x_th,r),dot(p.x_t,r));
            let det = a*c-b*b;
            if !(det > 0.) { return None }
            let d = [(c*g0-b*g1)/det,(a*g1-b*g0)/det];
            let step = d[0].dhypot(d[1]);
            if !(step < prev) { break }
            prev = step;
            uv = [uv[0]+d[0],uv[1]+d[1]];
        }
        Some(uv)
    }

    /// **Where a chart's point `q` stands on the pinch's rim**: the rim's sign and the collar's
    /// angle there — on the collar at `ρ = ±ρ₀`, on a sheet where `|(A, B)| = ρ₀` to the rounding of
    /// `ρ₀` (where the tracer's exit settled it on the hole's side).
    fn on_hole(&self,chart: Chart,q: P) -> Option<(f64,f64)> {
        let p = self.pinch.as_ref()?;
        match chart {
            Chart::Collar => (q[1].abs() >= p.rho*(1.-HALF_READ)).then(|| (q[1].signum(),q[0])),
            Chart::Sheet(side) => if p.side(q).abs() <= HALF_READ*p.rho { p.psi(side,q).map(|psi| (side,psi)) } else { None },
            Chart::End(_) => None,
        }
    }

    /// **Where a cuspidal edge reaches the roll's end `k`**: the root of `J` along the rim from its
    /// traced end `th`, by Newton's method while the steps shrink.
    pub fn corner(&self,side: f64,k: usize,mut th: f64) -> Option<P> {
        let t = self.roll[k];
        let mut prev = f64::INFINITY;
        for _ in 0..CORRECTIONS {
            let (j,g) = (self.jacobian(side,th,t)?,self.jacobian_gradient(side,th,t)?);
            if !(g[0].abs() > 0.) { return None }
            let step = j/g[0];
            if !(step.abs() < prev) { break }
            prev = step.abs();
            th -= step;
        }
        Some([th.rem_euclid(TAU),t])
    }
}

/// **A run's crossing of its sheet's rim at the roll's end `k`**, solved on the rim — one curve read
/// in two charts, well-conditioned along it where neither chart's field is across it (the end pose
/// touches the sheet along its rim) — as the root in `θ` of the far cover's depth at the rim's point
/// continued from `c` (`Depth::along`), its rate the envelope theorem's; by Newton's method while
/// the steps shrink. The crossing's angle and its cover.
fn crossing(field: &Depth,k: usize,mut th: f64,mut c: Cover) -> Option<(f64,Cover)> {
    let t = field.tube.roll[k];
    let mut prev = f64::INFINITY;
    for _ in 0..CORRECTIONS {
        let v = field.along([th,t],&c)?;
        let g = v.grad?[0];
        if !(g.abs() > 0.) { return None }
        let step = v.f/g;
        c = v.at;
        if !(step.abs() < prev) { break }
        prev = step.abs();
        th -= step;
    }
    Some((th,c))
}

/// **A run's crossing of the pinch's rim**, solved on the rim — one curve read in three charts (the
/// collar's `ρ = ±ρ₀`, the sheet's `place(ψ, ±ρ₀)`) — as the root in `ψ` of the far cover's depth
/// on the collar at `(ψ, sign ρ₀)` continued from `c` (`Depth::along`), by Newton's method while the
/// steps shrink. The crossing's angle and its cover.
fn hole_crossing(tube: &Tube,sign: f64,mut psi: f64,mut c: Cover,tol: f64) -> Option<(f64,Cover)> {
    let rho = sign*tube.pinch.as_ref()?.rho;
    let field = Depth {tube,chart:Chart::Collar,within:tol,holes:&[]};
    let mut prev = f64::INFINITY;
    for _ in 0..CORRECTIONS {
        let v = field.along([psi,rho],&c)?;
        let g = v.grad?[0];
        if !(g.abs() > 0.) { return None }
        let step = v.f/g;
        c = v.at;
        if !(step.abs() < prev) { break }
        prev = step.abs();
        psi -= step;
    }
    Some((psi,c))
}

/// A vertex: its point, and its parameters on each chart it lies on (a chart twice where it is
/// a double curve's point of one sheet).
#[derive(Clone,Debug)]
struct Vertex { x: V,on: Vec<(Chart,P)> }

/// **A trim**: a run of a sheet's depth zero set, its points on its own chart and on its partner's
/// (each angle unwrapped along it), the vertices at its ends, and how it is read between them.
#[derive(Clone,Debug)]
struct Trim { own: Chart,other: Chart,uv: Vec<P>,partner: Vec<P>,ends: [usize;2],shape: Shape }

/// How a trim is read between its points: traced (each point settled on the double curve), the
/// swallowtail's normal form from a pair on a disc's rim to its point (`rings::normal_form`, its
/// offsets `d` and kernel's way, from the point's parameters on each chart `base`, over the part
/// `span` of it a seam left), or straight in its sheet's parameters to a corner on the end pose
/// `k`.
#[derive(Clone,Debug)]
enum Shape { Traced,Normal { base: [P;2],d: [P;2],way: f64,span: [f64;2] },Straight { k: usize } }

impl Trim {
    /// The trim's point at `s` of `[0, 1]`: its two charts' parameters, settled where it is traced.
    fn at(&self,tube: &Tube,s: f64) -> Option<[P;2]> {
        let n = self.uv.len();
        if s <= 0. { return Some([self.uv[0],self.partner[0]]) }
        if s >= 1. { return Some([self.uv[n-1],self.partner[n-1]]) }
        match self.shape {
            Shape::Traced => {
                // (its place along its own polyline by length in the chart's parameters)
                let mut ls = vec![0.];
                for w in self.uv.windows(2) { ls.push(ls[ls.len()-1]+crate::plane2::dist(w[0],w[1])) }
                let l = s*ls[n-1];
                let j = (0..n-1).find(|&j| ls[j+1] >= l).unwrap_or(n-2);
                let f = if ls[j+1] > ls[j] { (l-ls[j])/(ls[j+1]-ls[j]) } else { 0. };
                let lerp = |a: P,b: P| [a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f];
                let (u,p) = (lerp(self.uv[j],self.uv[j+1]),lerp(self.partner[j],self.partner[j+1]));
                let dir = [self.uv[j+1][0]-self.uv[j][0],self.uv[j+1][1]-self.uv[j][1]];
                settle(tube,(self.own,self.other),[u,p],dir)
            }
            Shape::Normal {base,d,way,span} => {
                let o = crate::swept::rings::normal_form(d,way)(1.-(span[0]+(span[1]-span[0])*s));
                Some([[base[0][0]+o[0][0],base[0][1]+o[0][1]],[base[1][0]+o[1][0],base[1][1]+o[1][1]]])
            }
            Shape::Straight {k} => {
                let (q,c) = (self.uv[0],self.uv[n-1]);
                let u = [q[0]+(c[0]-q[0])*s,q[1]+(c[1]-q[1])*s];
                let e = tube.on_end(k,tube.at(self.own,u)?.x)?;
                let near = self.partner[0];
                Some([u,[unwrap(e[0],near[0]),unwrap(e[1],near[1])]])
            }
        }
    }
}

/// What an edge of the faces is, for its curve in space: a run traced, a sheet's rim at the roll's
/// end `k` between two angles, a cut at an angle across a sheet (between two instants), across an
/// end pose's band (between two angles round its tube, `ψ` unwrapped) or across the collar
/// (between its two rims, or two of its radii), a seam along a chart at its second parameter
/// between two of its first (`Along`), or the pinch's rim on the sheet `sign` between two of the
/// collar's angles.
#[derive(Clone,Debug)]
enum Kind { Trim(usize),Rim { side: f64,k: usize,th: [f64;2] },Across { chart: Chart,th: f64,v: [f64;2] },Along { chart: Chart,v: f64,th: [f64;2] },
    Hole { sign: f64,psi: [f64;2] } }

/// An edge of the faces: what it is, its vertices, and its uses, each a chart and its polyline
/// there (in the chart's own parameters, an end pose's `ψ` unwrapped along it).
#[derive(Clone,Debug)]
struct Arc2 { kind: Kind,v: [usize;2],uses: Vec<(Chart,Vec<P>)> }

/// **A chart's windows' cuts**: two angles splitting the largest gap in angle the trims' uses on
/// it (and the swallowtail points on it, `points`) leave free, at its thirds — each chart its own,
/// so a trim on one chart round every angle with its partner round the rest on another is cut on
/// each (the bench's tumbling torus: its sheet `−`'s double curves with the end pose run near its
/// rim over 2.5 of a turn's angle, their partners on the end pose over the rest). Refused where no
/// gap is wider than a hundredth of a turn (structural: a trim round every angle is a road not
/// built).
fn windows(trims: &[Trim],points: &[(Chart,P)],chart: Chart) -> Result<[f64;2],Unbuilt> {
    let mut ths: Vec<f64> = trims.iter().flat_map(|t| [(t.own,&t.uv),(t.other,&t.partner)])
        .filter(|(c,_)| *c == chart).flat_map(|(_,l)| l.iter().map(|p| p[0].rem_euclid(TAU))).collect();
    ths.extend(points.iter().filter(|(c,_)| *c == chart).map(|(_,p)| p[0].rem_euclid(TAU)));
    if ths.is_empty() { return Ok([0.5*PI,1.5*PI]) }
    ths.sort_by(f64::total_cmp);
    let n = ths.len();
    let (gap,at) = (0..n).map(|i| { let next = if i+1 < n { ths[i+1] } else { ths[0]+TAU }; (next-ths[i],ths[i]) })
        .max_by(|p,q| p.0.total_cmp(&q.0)).expect("some angles");
    if gap < 0.01*TAU { return Err(failed(format!("the trims leave no angle of {chart:?} free for the windows' cuts"))) }
    // (in order within the turn: the windows run from the first to the second and on round)
    let (x,y) = ((at+gap/3.).rem_euclid(TAU),(at+2.*gap/3.).rem_euclid(TAU));
    Ok([x.min(y),x.max(y)])
}

/// The charts, in the order their faces are read.
const CHARTS: [Chart;4] = [Chart::Sheet(1.),Chart::Sheet(-1.),Chart::End(0),Chart::End(1)];

/// A chart's windows' cuts, of each chart's.
fn cuts_of(cuts: &[(Chart,[f64;2])],chart: Chart) -> [f64;2] { cuts.iter().find(|(c,_)| *c == chart).expect("every chart's cuts").1 }

/// The chart's end-pose band: where the sheet `+`'s rim stands on the end pose `k` at `θ`, and
/// which way round the tube the band runs from it (`side` above nought there).
fn band(tube: &Tube,k: usize,th: f64) -> Option<(f64,f64,f64)> {
    let plus = tube.on_end(k,tube.point(1.,th,tube.roll[k])?.x)?[1];
    let minus = tube.on_end(k,tube.point(-1.,th,tube.roll[k])?.x)?[1];
    // (the way whose middle, between the two rims, is kept)
    let up = (minus-plus).rem_euclid(TAU);
    let s = tube.side(Chart::End(k),[th,plus+0.5*up])?;
    Some(if s > 0. { (plus,1.,up) } else { (plus,-1.,TAU-up) })
}

/// **The closed solid**, the tube road's (the module's method), sewn to `tol`.
pub fn solid(tube: &Tube,tol: f64) -> Result<Brep,Unbuilt> {
    let feature = 0.25*tube.ring.thickness;
    let [a,b] = tube.roll;
    let on_rim = |t: f64| [a,b].iter().position(|&e| (t-e).abs() <= HALF_READ*(b-a));
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut trims: Vec<Trim> = Vec::new();
    // (each swallowtail point, and the cuts from it: its sheet, its point, its vertex)
    let mut tails: Vec<(f64,P,usize)> = Vec::new();
    // (where there is a pinch the collar first: its runs' ends on the hole's rim seed the sheets')
    let charts = if tube.pinch.is_some() { vec![Chart::Collar,Chart::Sheet(1.),Chart::Sheet(-1.)] } else { vec![Chart::Sheet(1.),Chart::Sheet(-1.)] };
    let mut on_hole: Vec<(f64,f64)> = Vec::new();
    // (each chart's cuspidal edges, which its faces' cells stop at)
    let mut all_folds: Vec<(Chart,Vec<Vec<P>>)> = Vec::new();
    for chart in charts {
        let side = if let Chart::Sheet(s) = chart { s } else { 0. };
        let (folds,_) = edges(tube,chart,tol,feature)?;
        let folds: Vec<Vec<P>> = folds.into_iter().map(|p| p.uv).collect();
        if let Some(path) = std::env::var_os("SOLVENT_TUBE_DUMP") {
            use std::io::Write;
            if let Ok(mut f) = std::fs::OpenOptions::new().append(true).create(true).open(path) {
                for l in &folds { let _ = writeln!(f,"fold {chart:?} {}",l.iter().map(|p| format!("{},{}",p[0],p[1])).collect::<Vec<_>>().join(" ")); }
                for l in tube.hole(chart)? { let _ = writeln!(f,"hole {chart:?} {}",l.iter().map(|p| format!("{},{}",p[0],p[1])).collect::<Vec<_>>().join(" ")); }
                let _ = writeln!(f,"roll {a} {b}");
            }
        }
        all_folds.push((chart,folds.clone()));
        let points: Vec<P> = folds.iter().flat_map(|f| if side != 0. { swallowtails(tube,side,f) } else { swallowtails_on(tube,chart,f) }).collect();
        if chart == Chart::Collar && !points.is_empty() { return Err(failed(format!("the collar holds swallowtail points {points:?}"))) }
        // the corners: each fold's end on a rim, settled there
        let mut corners: Vec<(usize,P)> = Vec::new();
        for f in &folds {
            for q in [f[0],f[f.len()-1]] {
                if side == 0. { continue }
                let Some(k) = on_rim(q[1]) else { continue };
                let c = tube.corner(side,k,q[0]).ok_or_else(|| failed(format!("a corner of the sheet {side} at {q:?} not settled")))?;
                if corners.iter().any(|(_,d)| (d[0]-c[0]).abs() <= HALF_READ) { continue }
                let x = tube.point(side,c[0],c[1]).ok_or_else(|| failed("no sheet at a corner".into()))?.x;
                let e = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a corner".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,c),(Chart::End(k),e)]});
                corners.push((vertices.len()-1,c));
            }
        }
        let seeds: Vec<P> = on_hole.iter().filter(|h| side == 0. || h.0 == side).filter_map(|&(sign,psi)| match chart {
            Chart::Collar => tube.pinch.as_ref().map(|p| [psi,sign*p.rho]),_ => tube.pinch.as_ref().and_then(|p| p.rim(sign,psi)) }).collect();
        if debug() { eprintln!("tube: {chart:?} seeded on the pinch's rim at {seeds:?}") }
        let (pieces,_,discs) = seams(tube,chart,&folds,&points,&seeds,tol,feature)?;
        let field = Depth {tube,chart,within:tol,holes:&discs};
        let runs = merged(tube,chart,&field,pieces,tol);
        // each piece cut into runs by what covers it, and the runs kept; each switch of what covers
        // it resolved once for both runs beside it (`switched`: its vertex, or none where it is the
        // partner run's rim crossing, read once every run's ends are — a switch between a sheet's
        // cover and the collar's is the partner's crossing of the pinch's rim, `rim_vertex`)
        // (an end pose's, a sheet's, or — its place inside the pinch's hole — the collar's)
        let in_hole = |c: &Cover| tube.pinch.as_ref().is_some_and(|p| {
            let l = tube.ring.frame.local(c.point);
            p.side([l[1].datan2(l[0]).rem_euclid(TAU),c.at]) < 0.
        });
        let class = |c: &Cover| on_rim(c.at).map_or(if in_hole(c) { 3 } else { 2 },|k| k);
        let mut kept: Vec<(Vec<P>,Vec<Cover>,[Option<Option<usize>>;2])> = Vec::new();
        for (uv,at) in runs {
            let n = uv.len();
            let mut from = 0;
            let mut before = None;
            for i in 1..=n {
                if i < n && class(&at[i]) == class(&at[from]) { continue }
                let after = if i < n { Some(if class(&at[i-1]) == 3 || class(&at[i]) == 3 { None }
                    else { switched(tube,chart,(uv[i-1],&at[i-1]),(uv[i],&at[i]),&mut vertices,tol)? }) } else { None };
                let (u,c) = (uv[from..i].to_vec(),at[from..i].to_vec());
                let m = c.len()/2;
                // (a double curve of the sheets traced on both preimages: the later one's kept)
                let keep = class(&c[m]) < 2 || tube.own(chart,u[m]) > c[m].at;
                // (each end on the pinch's rim a vertex, kept or not: a kept trim's partner
                // switches chart there; and a seed of the charts read after)
                for (q,cv) in [(u[0],&c[0]),(u[u.len()-1],&c[c.len()-1])] {
                    let Some((sign,psi)) = tube.on_hole(chart,q) else { continue };
                    let j = rim_vertex(tube,&mut vertices,sign,psi,cv.clone(),tol)?;
                    if let Some((_,[psi,_])) = vertices[j].on.iter().find(|(ch,_)| *ch == Chart::Collar) { on_hole.push((sign,*psi)) }
                }
                if keep && !u.is_empty() { kept.push((u,c,[before,after])) }
                (before,from) = (after,i);
            }
        }
        // each run's partner, and its ends resolved
        for (uv,at,switch) in kept {
            let mut partner = Vec::with_capacity(uv.len());
            let mut other = None;
            for (q,c) in uv.iter().zip(&at) {
                let x = tube.at(chart,*q).ok_or_else(|| failed("no sheet at a trim's point".into()))?.x;
                let (ch,p) = tube.partner(x,c).ok_or_else(|| failed(format!("no partner of the trim's point {q:?}")))?;
                if other.is_some_and(|o| o != ch) { return Err(failed(format!("a trim's partner changes chart at {q:?}"))) }
                other = Some(ch);
                partner.push(p);
            }
            let other = other.ok_or_else(|| failed("an empty trim".into()))?;
            // (a run's end on the pinch's rim read twice — the tracer's seed there and its exit settled
            // there, one crossing: a trim does not run along the rim — the end's alone kept, so the way
            // the run leaves it is its own, not a rounding's)
            let (mut uv,mut partner) = (uv,partner);
            for e in 0..2 {
                while uv.len() > 2 {
                    let (i,j) = if e == 0 { (0,1) } else { (uv.len()-1,uv.len()-2) };
                    if tube.on_hole(chart,uv[i]).is_none() || tube.on_hole(chart,uv[j]).is_none() { break }
                    uv.remove(j);
                    partner.remove(j);
                }
            }
            // (each angle unwrapped along the run, and an end pose's ψ)
            for j in 1..uv.len() {
                uv[j][0] = unwrap(uv[j][0],uv[j-1][0]);
                partner[j][0] = unwrap(partner[j][0],partner[j-1][0]);
                if let Chart::End(_) = other { partner[j][1] = unwrap(partner[j][1],partner[j-1][1]) }
            }
            trims.push(Trim {own:chart,other,uv,partner,ends:[usize::MAX;2],shape:Shape::Traced});
            let ti = trims.len()-1;
            for e in 0..2 {
                let t = &trims[ti];
                let i = if e == 0 { 0 } else { t.uv.len()-1 };
                // (the run's last cover, a switch at its start having put a point before it)
                let (q,c) = (t.uv[i],at[if e == 0 { 0 } else { at.len()-1 }].clone());
                match switch[e] {
                    Some(Some(v)) => { ended(&mut trims[ti],e,v,&vertices)?; continue }
                    Some(None) => continue,
                    None => {}
                }
                // a crossing of the pinch's rim
                if let Some((sign,psi)) = tube.on_hole(chart,q) {
                    let j = rim_vertex(tube,&mut vertices,sign,psi,c,tol)?;
                    let v = &vertices[j];
                    let own = v.on.iter().find(|(ch,_)| *ch == chart).map(|(_,p)| *p).expect("on its chart");
                    let other = trims[ti].other;
                    let p = v.on[2..].iter().find(|(ch,_)| *ch == other).map(|(_,p)| *p)
                        .ok_or_else(|| failed(format!("a crossing of the pinch's rim at {q:?} whose partner is not on {other:?}")))?;
                    let t = &mut trims[ti];
                    (t.uv[i],t.partner[i],t.ends[e]) = (own,p,j);
                    continue
                }
                // a rim crossing
                if side == 0. { return Err(failed(format!("a trim of the collar ends at {q:?}, at no event"))) }
                // (a run against the end pose `k` reaching its sheet's rim at that end: the corner. Beside
                // the rim the end pose stands from the sheet's point by half its depth's second rate
                // in the instant times the square of the instant's way to the end, nought only where
                // that rate is, on the cuspidal edge: the run reaches the rim at the corner alone, and
                // about it the sheet and the end pose touch, so each far cover there stands within the
                // rounding and the run's points stand on their double curve to it alone — whether the
                // trace stops at the cuspidal edge short of the rim or runs on to the rim beside the
                // corner is a rounding's. Its tail is the straight join to the corner (`cornered`)
                // wherever that join holds the end pose, taken back point by point from the rim.)
                if let (Some(k),Chart::End(o)) = (on_rim(q[1]),trims[ti].other) {
                    if o == k {
                        let apart = |c: P| crate::plane2::dist([unwrap(c[0],q[0]),c[1]],q);
                        let Some(&(v,cq)) = corners.iter().min_by(|x,y| apart(x.1).total_cmp(&apart(y.1))) else {
                            return Err(failed(format!("a trim of the sheet {side} reaches its end pose's rim at {q:?}, at no corner")))
                        };
                        let c = [unwrap(cq[0],q[0]),cq[1]];
                        let n = trims[ti].uv.len();
                        // (the `m`-th of its points from the rim's; the join from the farthest it holds
                        // from, the point before the rim's at least)
                        let point = |m: usize| if e == 0 { m } else { n-1-m };
                        let mut cut = 1;
                        while cut+2 < n && join_holds(tube,side,k,trims[ti].uv[point(cut+1)],c,tol).is_ok() { cut += 1 }
                        let t = &mut trims[ti];
                        if e == 0 { t.uv.drain(..cut); t.partner.drain(..cut); } else { t.uv.truncate(n-cut); t.partner.truncate(n-cut) }
                        let (j,join) = cornered(tube,side,k,cq,v,&trims[ti],e,&mut vertices,tol)?;
                        trims[ti].ends[e] = j;
                        trims.push(join);
                        continue
                    }
                }
                if let Some(k) = on_rim(q[1]) {
                    let (th,c) = crossing(&field,k,q[0],c).ok_or_else(|| failed(format!("a rim crossing from {q:?} not settled")))?;
                    let own = [th.rem_euclid(TAU),tube.roll[k]];
                    let x = tube.at(chart,own).ok_or_else(|| failed("no sheet at a rim crossing".into()))?.x;
                    let (ch,p) = tube.partner(x,&c).ok_or_else(|| failed("no partner of a rim crossing".into()))?;
                    let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a rim crossing".into()))?;
                    vertices.push(Vertex {x,on:vec![(chart,own),(Chart::End(k),end),(ch,p)]});
                    let t = &mut trims[ti];
                    t.uv[i] = own;
                    t.partner[i] = p;
                    t.ends[e] = vertices.len()-1;
                    continue
                }
                // a swallowtail point's disc
                // (ending on the rim as `seams` reads it, the march's last step short of or past
                // it: within the disc's radius of it)
                if let Some(d) = discs.iter().find(|d| d.side([unwrap(q[0],d.at[0]),q[1]]).abs() <= d.radius) {
                    if !tails.iter().any(|(s,p,_)| *s == side && *p == d.at) {
                        let x = tube.point(side,d.at[0],d.at[1]).ok_or_else(|| failed("no sheet at a swallowtail point".into()))?.x;
                        vertices.push(Vertex {x,on:vec![(chart,d.at)]});
                        tails.push((side,d.at,vertices.len()-1));
                    }
                    let tail = tails.iter().find(|(s,p,_)| *s == side && *p == d.at).expect("the tail").2;
                    let (j,join) = joined(tube,side,d,&trims[ti],e,tail,&mut vertices,tol)?;
                    trims[ti].ends[e] = j;
                    trims.push(join);
                    continue
                }
                // a corner, joined straight
                let apart = |c: P| crate::plane2::dist([unwrap(c[0],q[0]),c[1]],q);
                let Some(&(v,cq)) = corners.iter().min_by(|x,y| apart(x.1).total_cmp(&apart(y.1))) else {
                    return Err(failed(format!("a trim of the sheet {side} ends at {q:?}, at no event")))
                };
                let Chart::End(k) = trims[ti].other else { return Err(failed(format!("a trim ends at {q:?} beside a corner, its partner no end pose"))) };
                let (j,join) = cornered(tube,side,k,cq,v,&trims[ti],e,&mut vertices,tol)?;
                trims[ti].ends[e] = j;
                trims.push(join);
            }
        }
    }
    met(tube,&mut trims,&vertices,tol,feature)?;
    // the switches: each the partner of a rim crossing on the switching trim's chart
    if debug() {
        for (i,t) in trims.iter().enumerate() { let n = t.uv.len(); eprintln!("tube: run {i} on {:?} against {:?}, ends {:?}, {:?} .. {:?}, partner {:?} .. {:?}; next {:?}, before last {:?}",t.own,t.other,t.ends,t.uv[0],t.uv[n-1],t.partner[0],t.partner[n-1],t.uv[1.min(n-1)],t.uv[n.saturating_sub(2)]) }
        for (i,v) in vertices.iter().enumerate() { eprintln!("tube: vertex {i} {:?}",v.on) }
    }
    for ti in 0..trims.len() {
        for e in 0..2 {
            if trims[ti].ends[e] != usize::MAX { continue }
            let t = &trims[ti];
            let q = t.uv[if e == 0 { 0 } else { t.uv.len()-1 }];
            let apart = |p: P| crate::plane2::dist([unwrap(p[0],q[0]),p[1]],q);
            let near = |v: &Vertex| v.on.iter().filter(|(c,_)| *c == t.own).map(|(_,p)| apart(*p)).fold(f64::INFINITY,f64::min);
            let (v,_) = vertices.iter().enumerate().filter(|(_,v)| v.on.len() >= 3)
                .map(|(j,v)| (j,near(v))).min_by(|x,y| x.1.total_cmp(&y.1))
                .ok_or_else(|| failed(format!("a trim's cover switches at {q:?}, at no rim crossing")))?;
            ended(&mut trims[ti],e,v,&vertices)?;
        }
    }
    // (each trim's angles unwrapped along it once its ends are in place, and an end pose's ψ)
    for t in &mut trims {
        for j in 1..t.uv.len() {
            t.uv[j][0] = unwrap(t.uv[j][0],t.uv[j-1][0]);
            t.partner[j][0] = unwrap(t.partner[j][0],t.partner[j-1][0]);
            if let Chart::End(_) = t.own { t.uv[j][1] = unwrap(t.uv[j][1],t.uv[j-1][1]) }
            if let Chart::End(_) = t.other { t.partner[j][1] = unwrap(t.partner[j][1],t.partner[j-1][1]) }
        }
    }
    build(tube,&mut vertices,&mut trims,&tails,&all_folds,tol)
}

/// **The pieces of a chart's seams joined across the cuts `round` traces in**: two ends on one
/// cut are one point where they stand at one place to the rounding or — each the root along the cut
/// its own part's trace read — within the tolerance of one another in space (an end pose's `ψ`, a
/// whole turn too, read within its turn) — or, on one cut, where the cut between them stands within
/// the tolerance of the zero set, the far cover continued from one end (`Depth::along`) within it
/// at their middle and quarters: a crossing shallow to the cut is read on each side to the
/// tolerance in space, and its two roots along the cut stand farther apart than that (the forward
/// member's sheet `+` crossing `θ = 0` at `t` 1.50984 and 1.50986, 1.2e-5 apart, its two runs left
/// apart and the one's loose end sent to a corner).
fn merged(tube: &Tube,chart: Chart,field: &Depth,pieces: Vec<crate::zero_set::Piece<Cover>>,tol: f64) -> Vec<(Vec<P>,Vec<Cover>)> {
    let [a,b] = tube.roll;
    let runs: Vec<(Vec<P>,Vec<Cover>)> = pieces.into_iter().map(|p| (p.uv,p.at)).collect();
    let near = |p: P,q: P| match (tube.at(chart,p),tube.at(chart,q)) { (Some(x),Some(y)) => distance(x.x,y.x) <= tol,_ => false };
    let turn = |x: f64| (x+PI).rem_euclid(TAU)-PI;
    let (dv,rv) = match chart { Chart::End(_) => (turn as fn(f64) -> f64,HALF_READ),Chart::Sheet(_) => ((|x| x) as fn(f64) -> f64,HALF_READ*(b-a)),
        Chart::Collar => ((|x| x) as fn(f64) -> f64,HALF_READ*tube.pinch.as_ref().map_or(0.,|p| 2.*p.rho)) };
    // (the segment between two points within the tolerance of the zero set, the far cover read on
    // from the one's at its middle and quarters: an angle, and an end pose's `ψ`, met within its turn)
    let band = |p: P,c: &Cover,q: P| [0.25,0.5,0.75].iter().all(|&f| {
        let r = [p[0]+turn(q[0]-p[0])*f,p[1]+dv(q[1]-p[1])*f];
        crate::zero_set::Field::along(field,r,c).is_some_and(|v| v.f.abs() <= tol)
    });
    // a run in the band of one kept before it — its two ends and its middle each joined to that
    // run's nearest point (in the parameters, within the feature in space) through the band — is
    // that run read again, and dropped: where the far cover meets the chart all but tangentially
    // the depth stands within the tolerance across a strip, and each sign change of it along an
    // inner edge there seeds a run of its own (the forward member's sheet `−`: one double curve
    // traced seven times, 2e-5 apart along `t`)
    let feature = 0.25*tube.ring.thickness;
    let apart = |p: P,q: P| turn(p[0]-q[0]).dhypot(dv(p[1]-q[1]));
    let again = |a: &(Vec<P>,Vec<Cover>),b: &(Vec<P>,Vec<Cover>)| [0,a.0.len()/2,a.0.len()-1].iter().all(|&i| {
        let p = a.0[i];
        let q = b.0.iter().copied().min_by(|x,y| apart(p,*x).total_cmp(&apart(p,*y))).expect("a run of points");
        let close = match (tube.at(chart,p),tube.at(chart,q)) { (Some(x),Some(y)) => distance(x.x,y.x) <= feature,_ => false };
        close && band(p,&a.1[i],q)
    });
    let mut kept: Vec<(Vec<P>,Vec<Cover>)> = Vec::new();
    for r in runs { if !kept.iter().any(|k| again(&r,k)) { kept.push(r) } }
    let mut runs = kept;
    let same = |p: P,c: &Cover,q: P| turn(p[0]-q[0]) == 0. && dv(p[1]-q[1]) == 0.
        || turn(p[0]-q[0]).abs() <= HALF_READ && (dv(p[1]-q[1]).abs() <= rv || near(p,q) || band(p,c,q));
    'merge: loop {
        for i in 0..runs.len() { for j in 0..runs.len() {
            if i == j { continue }
            let (pi,pj) = (runs[i].0[runs[i].0.len()-1],runs[j].0[0]);
            let ci = runs[i].1[runs[i].1.len()-1].clone();
            if same(pi,&ci,pj) {
                let (mut u,mut c) = runs.remove(j);
                let i = if j < i { i-1 } else { i };
                u.remove(0); c.remove(0);
                runs[i].0.extend(u); runs[i].1.extend(c);
                continue 'merge
            }
            let pj = runs[j].0[runs[j].0.len()-1];
            if i < j && same(pi,&ci,pj) {
                let (mut u,mut c) = runs.remove(j);
                u.reverse(); c.reverse();
                u.remove(0); c.remove(0);
                runs[i].0.extend(u); runs[i].1.extend(c);
                continue 'merge
            }
        } }
        break
    }
    runs
}

/// **The two end poses' meeting** (the old road's `depth::ends_met`): the start pose's seams
/// (`seams` on `Chart::End(0)`, its contact circles its inner edges) where the end pose covers it,
/// each run a trim of the start pose against the end pose, its partner the end pose's place of each
/// point (`Tube::on_end`, closed form). Each end of a run is an event found already, one vertex
/// standing on both poses: where it meets the start pose's contact circle — the sheet's rim at the
/// start — the sheet's rim crossing covered by the end pose; where the start pose's cover passes
/// to the end pose from the instants before it, the sheet's rim crossing at the end covered by the
/// start pose. Each is the vertex on both poses nearest the run's end, held within the trace's
/// longest step (`feature`) of it in space, its parameters put at that end (in place of the
/// trace's own reading of it, on the contact circle; before it, at a switch).
fn met(tube: &Tube,trims: &mut Vec<Trim>,vertices: &[Vertex],tol: f64,feature: f64) -> Result<(),Unbuilt> {
    let chart = Chart::End(0);
    let [a,b] = tube.roll;
    let (circles,_) = edges(tube,chart,tol,feature)?;
    let circles: Vec<Vec<P>> = circles.into_iter().map(|p| p.uv).collect();
    let (pieces,_,discs) = seams(tube,chart,&circles,&[],&[],tol,feature)?;
    let ended = |c: &Cover| (c.at-b).abs() <= HALF_READ*(b-a);
    let turn = |x: f64| (x+PI).rem_euclid(TAU)-PI;
    for (uv,at) in merged(tube,chart,&Depth {tube,chart,within:tol,holes:&discs},pieces,tol) {
        let n = uv.len();
        let mut from = 0;
        for i in 1..=n {
            if i < n && ended(&at[i]) == ended(&at[from]) { continue }
            if ended(&at[from]) {
                let mut u = uv[from..i].to_vec();
                let mut partner = Vec::with_capacity(u.len());
                for q in &u {
                    let x = tube.at(chart,*q).ok_or_else(|| failed("no start pose at its meeting with the end pose".into()))?.x;
                    partner.push(tube.on_end(1,x).ok_or_else(|| failed("no end pose at its meeting with the start pose".into()))?);
                }
                let mut ends = [usize::MAX;2];
                for e in 0..2 {
                    let j = if e == 0 { 0 } else { u.len()-1 };
                    let q = u[j];
                    let x = tube.at(chart,q).ok_or_else(|| failed("no start pose at a meeting's end".into()))?.x;
                    let apart = |p: P| turn(p[0]-q[0]).dhypot(turn(p[1]-q[1]));
                    let (v,on) = vertices.iter().enumerate()
                        .filter_map(|(v,w)| {
                            let p0 = w.on.iter().find(|(c,_)| *c == Chart::End(0))?.1;
                            let p1 = w.on.iter().find(|(c,_)| *c == Chart::End(1))?.1;
                            Some((v,[p0,p1]))
                        })
                        .min_by(|x,y| apart(x.1[0]).total_cmp(&apart(y.1[0])))
                        .ok_or_else(|| failed(format!("the end poses' meeting ends at {q:?}, at no vertex on both")))?;
                    if distance(vertices[v].x,x) > feature {
                        return Err(failed(format!("the end poses' meeting ends at {q:?}, {:e} from the nearest vertex on both",distance(vertices[v].x,x))))
                    }
                    // (on the contact circle the trace's end is the event read again: replaced; at a
                    // switch the event lies past the run's last point: put after it)
                    let switch = if e == 0 { from > 0 } else { i < n };
                    let (p0,p1) = (on[0],on[1]);
                    match (switch,e) {
                        (false,0) => { u[0] = p0; partner[0] = p1 }
                        (false,_) => { let m = u.len()-1; u[m] = p0; partner[m] = p1 }
                        (true,0) => { u.insert(0,p0); partner.insert(0,p1) }
                        (true,_) => { u.push(p0); partner.push(p1) }
                    }
                    ends[e] = v;
                }
                trims.push(Trim {own:chart,other:Chart::End(1),uv:u,partner,ends,shape:Shape::Traced});
            }
            from = i;
        }
    }
    Ok(())
}

/// **A switch of what covers a run**, between its points `a` and `b` on the sheet `chart`, one
/// covered by an instant inside the roll (by the sheet `B`'s point there), the other by the end
/// pose `k`: the triple point where `B`'s preimage of the double curve reaches its rim at `k` and
/// the cover passes to the end pose there — a rim crossing on `B`. Where the run covered by `B` is
/// the one kept (its own instant the later), `B`'s run is not, and the crossing is solved here, on
/// `B`'s rim (`crossing`, from `B`'s point covered at the run's own instant), the vertex returned;
/// else `B`'s run is kept and its own rim crossing is this switch (none returned: read once every
/// run's ends are). A cover passing from one end pose to the other is the end poses' meeting, not
/// yet read: refused by name.
fn switched(tube: &Tube,chart: Chart,a: (P,&Cover),b: (P,&Cover),vertices: &mut Vec<Vertex>,tol: f64) -> Result<Option<usize>,Unbuilt> {
    let [lo,hi] = tube.roll;
    let on_rim = |t: f64| [lo,hi].iter().position(|&e| (t-e).abs() <= HALF_READ*(hi-lo));
    let ((q,c),k) = match (on_rim(a.1.at),on_rim(b.1.at)) {
        (None,Some(k)) => (a,k),
        (Some(k),None) => (b,k),
        _ => return Err(failed(format!("a run's cover passes from one end pose to the other at {:?}: the end poses' meeting, not yet read",a.0))),
    };
    if !(q[1] > c.at) { return Ok(None) }
    let x = tube.at(chart,q).ok_or_else(|| failed("no sheet at a switch".into()))?.x;
    let (other,p) = tube.partner(x,c).ok_or_else(|| failed(format!("no partner of the switch's point {q:?}")))?;
    if let Chart::End(_) = other { return Err(failed(format!("a switch at {q:?} whose cover is no sheet"))) }
    // (`B`'s point covered at the run's own instant, the cover `crossing` continues)
    let y = tube.at(other,p).ok_or_else(|| failed("no sheet at a switch's partner".into()))?.x;
    let pose = tube.path.pose(q[1]).map_err(|_| failed("no pose at a switch".into()))?;
    let mine = Cover {depth:0.,at:q[1],point:pose.inverse().point(y),piece:0};
    // (no disc of `B`'s cut about the rim: a swallowtail point is inside its chart)
    let field = Depth {tube,chart:other,within:tol,holes:&[]};
    let (th,cover) = crossing(&field,k,p[0],mine).ok_or_else(|| failed(format!("the switch at {q:?}: its rim crossing not settled")))?;
    let own = [th.rem_euclid(TAU),tube.roll[k]];
    let y = tube.at(other,own).ok_or_else(|| failed("no sheet at a switch's rim crossing".into()))?.x;
    let (ch,pa) = tube.partner(y,&cover).ok_or_else(|| failed("no partner of a switch's rim crossing".into()))?;
    if ch != chart { return Err(failed(format!("the switch at {q:?}: its rim crossing's partner on {ch:?}"))) }
    let end = tube.on_end(k,y).ok_or_else(|| failed("no end pose at a switch's rim crossing".into()))?;
    vertices.push(Vertex {x:y,on:vec![(other,own),(Chart::End(k),end),(ch,pa)]});
    Ok(Some(vertices.len()-1))
}

/// **A trim's end `e` at its switch's vertex `v`**: the switch's own parameters on the trim's chart
/// (of the vertex's places there, the nearest the run's end) and its partner the vertex's place on
/// the trim's partner (another of them where that is the same chart), put at that end.
fn ended(t: &mut Trim,e: usize,v: usize,vertices: &[Vertex]) -> Result<(),Unbuilt> {
    let q = t.uv[if e == 0 { 0 } else { t.uv.len()-1 }];
    let apart = |p: P| crate::plane2::dist([unwrap(p[0],q[0]),p[1]],q);
    let on = &vertices[v].on;
    let (j,own) = on.iter().enumerate().filter(|(_,(c,_))| *c == t.own).map(|(j,(_,p))| (j,*p))
        .min_by(|x,y| apart(x.1).total_cmp(&apart(y.1))).ok_or_else(|| failed(format!("a switch at {q:?} not on the trim's chart")))?;
    let other = on.iter().enumerate().find(|(i,(c,_))| *i != j && *c == t.other).map(|(_,(_,p))| *p)
        .ok_or_else(|| failed("a switch whose crossing is not on the trim's partner".into()))?;
    if e == 0 { t.uv.insert(0,own); t.partner.insert(0,other) } else { t.uv.push(own); t.partner.push(other) }
    t.ends[e] = v;
    Ok(())
}

/// **The vertex where a run crosses the pinch's rim** (`hole_crossing`, from the collar's angle
/// `psi` and the run's cover `c` there): on the collar, on the sheet `sign` and on its partner —
/// one crossing read once, a vertex already at its point (within the tolerance) taken.
fn rim_vertex(tube: &Tube,vertices: &mut Vec<Vertex>,sign: f64,psi: f64,c: Cover,tol: f64) -> Result<usize,Unbuilt> {
    let p = tube.pinch.as_ref().ok_or_else(|| failed("a rim crossing with no pinch".into()))?;
    let (psi,c) = hole_crossing(tube,sign,psi,c,tol).ok_or_else(|| failed(format!("a crossing of the pinch's rim from ψ {psi} not settled")))?;
    let (psi,rho) = (psi.rem_euclid(TAU),sign*p.rho);
    let x = tube.collar_at(psi,rho).ok_or_else(|| failed("no collar at a crossing of its rim".into()))?.x;
    if let Some(j) = vertices.iter().position(|v| v.on.iter().any(|(ch,q)| *ch == Chart::Collar && q[1] == rho) && distance(v.x,x) <= tol) { return Ok(j) }
    let place = p.rim(sign,psi).ok_or_else(|| failed("the pinch's rim not read at a crossing".into()))?;
    let (ch,pp) = tube.partner(x,&c).ok_or_else(|| failed(format!("no partner of a crossing of the pinch's rim at ψ {psi}")))?;
    vertices.push(Vertex {x,on:vec![(Chart::Collar,[psi,rho]),(Chart::Sheet(sign),place),(ch,pp)]});
    Ok(vertices.len()-1)
}

/// **A trim joined to its swallowtail point inside its disc** (`rings::joined_by_normal_form`, the
/// ring road's rule): from its end `e` on the disc's rim, with its partner there, to the point (the
/// vertex `tail`) — a trim of its own along the normal form, from a vertex at that end, returned.
fn joined(tube: &Tube,side: f64,d: &Disc,t: &Trim,e: usize,tail: usize,vertices: &mut Vec<Vertex>,tol: f64) -> Result<(usize,Trim),Unbuilt> {
    let i = if e == 0 { 0 } else { t.uv.len()-1 };
    let (q,r) = (t.uv[i],t.partner[i]);
    let at = d.at;
    let off = |q: P| [(q[0]-at[0]+PI).rem_euclid(TAU)-PI,q[1]-at[1]];
    let pair = [off(q),off(r)];
    let k = tube.kernel(side,at[0],at[1]).ok_or_else(|| failed("no kernel at a swallowtail point".into()))?;
    let x = |q: P| tube.point(side,q[0],q[1]).map(|p| p.x);
    let xq = x(q).ok_or_else(|| failed("no sheet at a disc's rim".into()))?;
    let span = distance(xq,x(at).ok_or_else(|| failed("no sheet at a swallowtail point".into()))?);
    let outer = |_: usize,o: P,_: f64| -> Option<V> {
        let q = [at[0]+o[0],at[1]+o[1]];
        (tube.jacobian(side,q[0],q[1])? > 0.).then(|| x(q))?
    };
    crate::swept::rings::joined_by_normal_form(pair,k[1]/k[0],span,(8.*tol*tube.r).sqrt(),&outer,tol)
        .ok_or_else(|| failed(format!("a double curve not joined to its swallowtail point {at:?}")))?;
    vertices.push(Vertex {x:xq,on:vec![(t.own,q),(t.other,r)]});
    let junction = vertices.len()-1;
    // (the point's angle unwrapped as the run's, on each chart)
    let base = [[q[0]-pair[0][0],at[1]],[r[0]-pair[1][0],at[1]]];
    let mut join = Trim {own:t.own,other:t.other,uv:vec![],partner:vec![],ends:[junction,tail],shape:Shape::Normal {base,d:pair,way:k[1]/k[0],span:[0.,1.]}};
    join.uv = vec![q,base[0]];
    join.partner = vec![r,base[1]];
    let line: Vec<[P;2]> = (0..=16).map(|j| join.at(tube,j as f64/16.)).collect::<Option<_>>().ok_or_else(|| failed("a join not read".into()))?;
    join.uv = line.iter().map(|p| p[0]).collect();
    join.partner = line.iter().map(|p| p[1]).collect();
    Ok((junction,join))
}

/// **A trim joined to a corner** of its sheet at the roll's end `k` (its vertex `corner`, at `c`),
/// from its end `e` straight in the sheet's parameters — about the corner every far cover stands
/// within the rounding, the run all but tangent to the rim, so the trace cannot reach it — a trim of
/// its own from a vertex at that end, returned: as many chords as the double curve's longest step
/// (`√(8 tol r)`) takes, each point and chord's middle held within `tol` of the end pose and on the
/// side of the cuspidal edge the sheet bounds; its partner each point's on the end pose.
#[allow(clippy::too_many_arguments)]
fn cornered(tube: &Tube,side: f64,k: usize,c: P,corner: usize,t: &Trim,e: usize,vertices: &mut Vec<Vertex>,tol: f64) -> Result<(usize,Trim),Unbuilt> {
    let i = if e == 0 { 0 } else { t.uv.len()-1 };
    let (q,r) = (t.uv[i],t.partner[i]);
    let c = [q[0]+(c[0]-q[0]+PI).rem_euclid(TAU)-PI,c[1]];
    let x = |p: P| tube.point(side,p[0],p[1]).map(|p| p.x);
    let xq = x(q).ok_or_else(|| failed("no sheet at a trim's end".into()))?;
    let n = join_holds(tube,side,k,q,c,tol)?;
    vertices.push(Vertex {x:xq,on:vec![(t.own,q),(t.other,r)]});
    let junction = vertices.len()-1;
    let mut join = Trim {own:t.own,other:t.other,uv:vec![q,c],partner:vec![r,r],ends:[junction,corner],shape:Shape::Straight {k}};
    let line: Vec<[P;2]> = (0..=2*n).map(|j| join.at(tube,j as f64/(2*n) as f64)).collect::<Option<_>>().ok_or_else(|| failed("a join not read".into()))?;
    join.uv = line.iter().map(|p| p[0]).collect();
    join.partner = line.iter().map(|p| p[1]).collect();
    // (its last point the corner's own on the end pose, the vertex's)
    let n = join.partner.len();
    let e = vertices[corner].on.iter().find(|(ch,_)| *ch == Chart::End(k)).map(|(_,p)| *p).expect("a corner on its end pose");
    join.partner[n-1] = [unwrap(e[0],join.partner[n-2][0]),unwrap(e[1],join.partner[n-2][1])];
    Ok((junction,join))
}

/// **Whether the straight join from `q` to the corner `c`** of the sheet `side` at the roll's end
/// `k` holds (`cornered`'s): its chords' count, each point and chord's middle within `tol` of the
/// end pose and, short of the corner, on the side of the cuspidal edge the sheet bounds.
fn join_holds(tube: &Tube,side: f64,k: usize,q: P,c: P,tol: f64) -> Result<usize,Unbuilt> {
    let x = |p: P| tube.point(side,p[0],p[1]).map(|p| p.x);
    let span = distance(x(q).ok_or_else(|| failed("no sheet at a trim's end".into()))?,x(c).ok_or_else(|| failed("no sheet at a corner".into()))?);
    let n = (span/(8.*tol*tube.r).sqrt()).ceil().max(1.) as usize;
    for j in 1..=2*n {
        let f = j as f64/(2*n) as f64;
        let p = [q[0]+(c[0]-q[0])*f,q[1]+(c[1]-q[1])*f];
        let y = x(p).ok_or_else(|| failed("no sheet on a corner's join".into()))?;
        let (depth,_) = tube.depth_at(y,tube.roll[k]).ok_or_else(|| failed("no depth on a corner's join".into()))?;
        if depth.abs() > tol || (j < 2*n && !(tube.jacobian(side,p[0],p[1]).is_some_and(|j| j > 0.))) {
            return Err(failed(format!("a trim not joined to its corner {c:?}: {depth:e} off the end pose at {p:?}")))
        }
    }
    Ok(n)
}

/// **A curve through `f` over `[0, 1]`** with its uses' parameters on the same parameter:
/// interpolated at evenly spaced parameters doubled until it holds `f` within `fit` at their
/// quarters, and each use's parameters read through its chart within `fit` there too.
fn fitted(f: &dyn Fn(f64) -> (V,Vec<P>),at: &dyn Fn(usize,P) -> Option<V>,bars: &[f64]) -> Result<(BSpline,Vec<BSpline>),Unbuilt> {
    let fit = bars[0];
    let mut n = 8;
    loop {
        let ts: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let reads: Vec<(V,Vec<P>)> = ts.iter().map(|&t| f(t)).collect();
        let pts: Vec<V> = reads.iter().map(|r| r.0).collect();
        if pts.iter().flatten().any(|x| !x.is_finite()) { return Err(failed("a curve could not be read".into())) }
        let curve = crate::nurbs::interpolate(&pts,&ts,DEGREE).ok_or_else(|| failed("a curve could not be interpolated".into()))?;
        let uses = reads[0].1.len();
        let pcs: Vec<BSpline> = (0..uses).map(|u| crate::nurbs::interpolate(&reads.iter().map(|r| [r.1[u][0],r.1[u][1],0.]).collect::<Vec<_>>(),&ts,DEGREE))
            .collect::<Option<_>>().ok_or_else(|| failed("a pcurve could not be interpolated".into()))?;
        let mut worst = 0f64;
        for i in 0..n { for q in [0.25,0.5,0.75] {
            let t = (i as f64+q)/n as f64;
            let y = curve.point(t);
            worst = worst.max(distance(y,f(t).0));
            for (u,pc) in pcs.iter().enumerate() {
                let p = pc.point(t);
                // (each use against its own bar, read as the fit's share of it)
                worst = worst.max(at(u,[p[0],p[1]]).map_or(f64::INFINITY,|z| distance(z,y))*fit/bars[u]);
            }
        } }
        if worst <= fit { return Ok((curve,pcs)) }
        if n >= MOST_FIT {
            if debug() {
                for i in 0..n { for q in [0.25,0.5,0.75] {
                    let t = (i as f64+q)/n as f64;
                    let y = curve.point(t);
                    let e0 = distance(y,f(t).0);
                    let es: Vec<f64> = pcs.iter().enumerate().map(|(u,pc)| { let p = pc.point(t); at(u,[p[0],p[1]]).map_or(f64::INFINITY,|z| distance(z,y))*fit/bars[u] }).collect();
                    if e0 > fit || es.iter().any(|&e| e > fit) { eprintln!("tube: misfit at {t}: {e0:e} {es:?} {:?}",f(t).1) }
                } }
            }
            return Err(failed(format!("a curve fits within {fit:e} only to {worst:e}")))
        }
        n *= 2;
    }
}

/// **A double curve's point**: its two charts' parameters, from `guess`, settled by Newton's method
/// on `X₁(u₁) = X₂(u₂)` with `u₁` held on the line square to `dir` through the guess, while the
/// steps shrink. None where it does not settle.
fn settle(tube: &Tube,(c1,c2): (Chart,Chart),guess: [P;2],dir: P) -> Option<[P;2]> {
    let [mut u1,mut u2] = guess;
    let value = dir[0]*u1[0]+dir[1]*u1[1];
    let mut prev = f64::INFINITY;
    for _ in 0..CORRECTIONS {
        let (x1,x2) = (tube.at(c1,u1)?,tube.at(c2,u2)?);
        let r = sub(x1.x,x2.x);
        let f = [r[0],r[1],r[2],dir[0]*u1[0]+dir[1]*u1[1]-value];
        let m = [[x1.x_th[0],x1.x_t[0],-x2.x_th[0],-x2.x_t[0]],[x1.x_th[1],x1.x_t[1],-x2.x_th[1],-x2.x_t[1]],
            [x1.x_th[2],x1.x_t[2],-x2.x_th[2],-x2.x_t[2]],[dir[0],dir[1],0.,0.]];
        let d = solve4(m,f)?;
        let step = d.iter().map(|x| x*x).sum::<f64>().sqrt();
        if !(step < prev) { break }
        prev = step;
        u1 = [u1[0]-d[0],u1[1]-d[1]];
        u2 = [u2[0]-d[2],u2[1]-d[3]];
    }
    Some([u1,u2])
}

/// `m x = f` by Gaussian elimination with partial pivoting; none where `m` is singular.
fn solve4(mut m: [[f64;4];4],mut f: [f64;4]) -> Option<[f64;4]> {
    for c in 0..4 {
        let p = (c..4).max_by(|&i,&j| m[i][c].abs().total_cmp(&m[j][c].abs()))?;
        if !(m[p][c].abs() > 0.) { return None }
        m.swap(c,p); f.swap(c,p);
        for r in c+1..4 {
            let g = m[r][c]/m[c][c];
            for k in c..4 { m[r][k] -= g*m[c][k] }
            f[r] -= g*f[c];
        }
    }
    let mut x = [0.;4];
    for c in (0..4).rev() { x[c] = (f[c]-(c+1..4).map(|k| m[c][k]*x[k]).sum::<f64>())/m[c][c] }
    Some(x)
}

/// `x` moved by whole periods to stand within half a period of `near`.
fn unwrap(x: f64,near: f64) -> f64 { x-TAU*((x-near)/TAU).round() }

/// The faces, their edges and vertices as the trims and the windows give them, sewn.
fn build(tube: &Tube,vertices: &mut Vec<Vertex>,trims: &mut Vec<Trim>,tails: &[(f64,P,usize)],folds: &[(Chart,Vec<Vec<P>>)],tol: f64) -> Result<Brep,Unbuilt> {
    let [a,b] = tube.roll;
    let mut points: Vec<(Chart,P)> = tails.iter().map(|t| (Chart::Sheet(t.0),t.1)).collect();
    // (the sheets' windows clear of the pinch's hole, which lies whole in one of them)
    for side in [1.,-1.] { points.extend(tube.hole(Chart::Sheet(side))?.into_iter().flatten().map(|p| (Chart::Sheet(side),p))) }
    let mut cuts: Vec<(Chart,[f64;2])> = CHARTS.iter().map(|&c| windows(trims,&points,c).map(|w| (c,w))).collect::<Result<_,_>>()?;
    if tube.pinch.is_some() { cuts.push((Chart::Collar,windows(trims,&points,Chart::Collar)?)) }
    let collar_cuts = if tube.pinch.is_some() { cuts_of(&cuts,Chart::Collar) } else { [0.;2] };
    // the cuts from each swallowtail point toward the rim it opens away from (the way whose point a
    // disc's span beside it is kept), each to the first trim it meets or else that rim
    let mut across: Vec<Arc2> = Vec::new();
    for &(side,p,v) in tails {
        let chart = Chart::Sheet(side);
        let step = 0.02*(b-a);
        let kept = |t: f64| match tube.reading(side,p[0],t,tol) { Some(Reading::Alone) => true,Some(Reading::Far(c)) => c.depth >= 0.,_ => false };
        let k = if kept(p[1]-step) { 0 } else if kept(p[1]+step) { 1 } else {
            return Err(failed(format!("neither way from the swallowtail point {p:?} is kept")))
        };
        let (foot,e) = match met_by_cut(tube,chart,p,tube.roll[k],v,trims,vertices)? {
            Some(f) => f,
            None => {
                let e = tube.roll[k];
                let x = tube.point(side,p[0],e).ok_or_else(|| failed("no sheet at a cut's foot".into()))?.x;
                let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a cut's foot".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,[p[0],e]),(Chart::End(k),end)]});
                (vertices.len()-1,e)
            }
        };
        let line: Vec<P> = (0..=32).map(|j| [p[0],p[1]+(e-p[1])*j as f64/32.]).collect();
        across.push(Arc2 {kind:Kind::Across {chart,th:p[0],v:[p[1],e]},v:[v,foot],uses:vec![(chart,line.clone()),(chart,line)]});
    }
    let mut arcs: Vec<Arc2> = Vec::new();
    for (i,t) in trims.iter().enumerate() {
        arcs.push(Arc2 {kind:Kind::Trim(i),v:t.ends,uses:vec![(t.own,t.uv.clone()),(t.other,t.partner.clone())]});
    }
    arcs.extend(across);
    // the rims, split at every vertex on them and at the cuts of the sheet's and the end pose's
    for side in [1.,-1.] {
        for k in 0..2 {
            let e = tube.roll[k];
            let chart = Chart::Sheet(side);
            let mut splits: Vec<(f64,usize)> = vertices.iter().enumerate()
                .flat_map(|(j,v)| v.on.iter().filter(|(c,p)| *c == chart && p[1] == e).map(move |(_,p)| (p[0].rem_euclid(TAU),j))).collect();
            // (an angle both charts cut at, one vertex)
            let mut at: Vec<f64> = cuts_of(&cuts,chart).into_iter().chain(cuts_of(&cuts,Chart::End(k))).collect();
            at.sort_by(f64::total_cmp);
            at.dedup();
            for c in at {
                let x = tube.point(side,c,e).ok_or_else(|| failed("no sheet at a cut".into()))?.x;
                let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a cut".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,[c,e]),(Chart::End(k),end)]});
                splits.push((c,vertices.len()-1));
            }
            splits.sort_by(|p,q| p.0.total_cmp(&q.0));
            let n = splits.len();
            for i in 0..n {
                let (t0,v0) = splits[i];
                let (t1,v1) = if i+1 < n { splits[i+1] } else { (splits[0].0+TAU,splits[0].1) };
                arcs.push(Arc2 {kind:Kind::Rim {side,k,th:[t0,t1]},v:[v0,v1],uses:rim_uses(tube,side,k,[t0,t1])?});
            }
        }
    }
    // the pinch's rim, each sheet's, split at every vertex on it and at the collar's cuts
    if let Some(p) = &tube.pinch {
        for sign in [1.,-1.] {
            let rho = sign*p.rho;
            let mut splits: Vec<(f64,usize)> = vertices.iter().enumerate()
                .flat_map(|(j,v)| v.on.iter().filter(|(c,q)| *c == Chart::Collar && q[1] == rho).map(move |(_,q)| (q[0].rem_euclid(TAU),j))).collect();
            for &c in &collar_cuts {
                let x = tube.collar_at(c,rho).ok_or_else(|| failed("no collar at a cut".into()))?.x;
                let place = p.rim(sign,c).ok_or_else(|| failed("the pinch's rim not read at a cut".into()))?;
                vertices.push(Vertex {x,on:vec![(Chart::Collar,[c,rho]),(Chart::Sheet(sign),place)]});
                splits.push((c,vertices.len()-1));
            }
            splits.sort_by(|p,q| p.0.total_cmp(&q.0));
            let n = splits.len();
            for i in 0..n {
                let (t0,v0) = splits[i];
                let (t1,v1) = if i+1 < n { splits[i+1] } else { (splits[0].0+TAU,splits[0].1) };
                arcs.push(Arc2 {kind:Kind::Hole {sign,psi:[t0,t1]},v:[v0,v1],uses:hole_uses(tube,sign,[t0,t1])?});
            }
        }
        for &c in &collar_cuts {
            let at = |rho: f64| vertices.iter().position(|v| v.on.iter().any(|(ch,q)| *ch == Chart::Collar && q[0] == c && q[1] == rho));
            let (Some(v0),Some(v1)) = (at(-p.rho),at(p.rho)) else { return Err(failed("a collar's cut with no vertex on its rim".into())) };
            let line: Vec<P> = (0..=32).map(|j| [c,-p.rho+2.*p.rho*j as f64/32.]).collect();
            arcs.push(Arc2 {kind:Kind::Across {chart:Chart::Collar,th:c,v:[-p.rho,p.rho]},v:[v0,v1],uses:vec![(Chart::Collar,line.clone()),(Chart::Collar,line)]});
        }
    }
    // the cuts across each chart at the windows' angles
    let at_rim = |vs: &Vec<Vertex>,chart: Chart,q: P| vs.iter().position(|v| v.on.iter().any(|(c,p)| *c == chart && p[0] == q[0] && p[1] == q[1]));
    for side in [1.,-1.] {
        let chart = Chart::Sheet(side);
        for c in cuts_of(&cuts,chart) {
            let (v0,v1) = (at_rim(vertices,chart,[c,a]).expect("a rim's cut"),at_rim(vertices,chart,[c,b]).expect("a rim's cut"));
            let line: Vec<P> = (0..=32).map(|j| [c,a+(b-a)*j as f64/32.]).collect();
            arcs.push(Arc2 {kind:Kind::Across {chart,th:c,v:[a,b]},v:[v0,v1],uses:vec![(chart,line.clone()),(chart,line)]});
        }
    }
    for k in 0..2 {
        let chart = Chart::End(k);
        for c in cuts_of(&cuts,chart) {
            let (plus,way,width) = band(tube,k,c).ok_or_else(|| failed("no band on an end pose".into()))?;
            let (v0,v1) = (at_rim(vertices,Chart::Sheet(1.),[c,tube.roll[k]]).expect("a rim's cut"),at_rim(vertices,Chart::Sheet(-1.),[c,tube.roll[k]]).expect("a rim's cut"));
            let line: Vec<P> = (0..=32).map(|j| [c,plus+way*width*j as f64/32.]).collect();
            arcs.push(Arc2 {kind:Kind::Across {chart,th:c,v:[plus,plus+way*width]},v:[v0,v1],uses:vec![(chart,line.clone()),(chart,line)]});
        }
    }
    faces(tube,vertices,&mut arcs,trims,&cuts,folds,tol)
}

/// **Where the cut from the swallowtail point `p` (its vertex `tail`) toward the roll's end `e`
/// first meets a trim on its chart**, if it does before the rim: the trims' polylines on the chart
/// (a run's own, or its partner's where that is the chart) crossed by the cut's angle strictly
/// between `p` and `e`, the nearest `p`; the point settled on the double curve with its angle held
/// (`settle`), a vertex of both its charts, and the trim split there into two (a traced one alone:
/// a join is refused by name). The vertex and the instant the cut ends at. A cut crossing a trim
/// with no vertex there is no planar graph: on the full turn's sheet `−` the cut from its second
/// swallowtail point ran down to the start and crossed a double curve with the end pose there, and
/// one face used a trim on both its sides (#880 T1f).
fn met_by_cut(tube: &Tube,chart: Chart,p: P,e: f64,tail: usize,trims: &mut Vec<Trim>,vertices: &mut Vec<Vertex>)
    -> Result<Option<(usize,f64)>,Unbuilt> {
    // (the nearest crossing: its trim, whether on the partner's polyline, its segment, the point)
    let mut best: Option<(usize,bool,usize,f64,f64)> = None;
    for (i,t) in trims.iter().enumerate() {
        if t.ends.contains(&tail) { continue }
        for (mine,line,ch) in [(false,&t.uv,t.own),(true,&t.partner,t.other)] {
            if ch != chart { continue }
            for j in 0..line.len()-1 {
                let (u,w) = (line[j],line[j+1]);
                let c = unwrap(p[0],u[0]);
                if u[0] == w[0] || (u[0]-c)*(w[0]-c) > 0. { continue }
                let f = (c-u[0])/(w[0]-u[0]);
                let t = u[1]+(w[1]-u[1])*f;
                if !((t-p[1])*(e-t) > 0.) { continue }
                if best.is_none_or(|q| (t-p[1]).abs() < (q.4-p[1]).abs()) { best = Some((i,mine,j,f,t)) }
            }
        }
    }
    let Some((i,mine,j,f,_)) = best else { return Ok(None) };
    let t = &trims[i];
    if !matches!(t.shape,Shape::Traced) { return Err(failed(format!("the cut from the swallowtail point {p:?} meets a join"))) }
    let lerp = |l: &Vec<P>| [l[j][0]+(l[j+1][0]-l[j][0])*f,l[j][1]+(l[j+1][1]-l[j][1])*f];
    let (u,w) = (lerp(&t.uv),lerp(&t.partner));
    // (the angle held on the cut's own chart: its place on the polyline crossed)
    let [u,w] = if mine {
        let [w,u] = settle(tube,(t.other,t.own),[w,u],[1.,0.]).ok_or_else(|| failed(format!("the cut from {p:?} not settled on a trim")))?; [u,w]
    } else { settle(tube,(t.own,t.other),[u,w],[1.,0.]).ok_or_else(|| failed(format!("the cut from {p:?} not settled on a trim")))? };
    let x = tube.at(t.own,u).ok_or_else(|| failed("no chart at a cut's foot on a trim".into()))?.x;
    vertices.push(Vertex {x,on:vec![(t.own,u),(t.other,w)]});
    let v = vertices.len()-1;
    let mut first = t.clone();
    let mut second = t.clone();
    first.uv.truncate(j+1); first.uv.push(u);
    first.partner.truncate(j+1); first.partner.push(w);
    first.ends[1] = v;
    second.uv = std::iter::once(u).chain(t.uv[j+1..].iter().copied()).collect();
    second.partner = std::iter::once(w).chain(t.partner[j+1..].iter().copied()).collect();
    second.ends[0] = v;
    trims[i] = first;
    trims.push(second);
    let foot = if mine { w } else { u };
    Ok(Some((v,foot[1])))
}

/// A use's polyline placed in a window from `w0` (its angle within a turn above `w0`, its middle's
/// so, or at the angle `target`, a cut's at the window's side; an end pose's `ψ` read as its offset round the tube from the band's start, the way the band
/// runs).
fn placed(tube: &Tube,chart: Chart,line: &[P],w0: f64,target: Option<f64>) -> Option<Vec<P>> {
    let mid = line[line.len()/2][0];
    let shift = match target { Some(th) => th-line[0][0],None => w0+(mid-w0).rem_euclid(TAU)-mid };
    line.iter().map(|q| {
        let th = q[0]+shift;
        match chart {
            Chart::Sheet(_) | Chart::Collar => Some([th,q[1]]),
            Chart::End(k) => {
                let (plus,way,_) = band(tube,k,q[0])?;
                let o = ((q[1]-plus)*way).rem_euclid(TAU);
                // (on the band's start itself, nought, not a turn)
                Some([th,if o > TAU-1e-9 { 0. } else { o }])
            }
        }
    }).collect()
}

/// **A window's planar graph**: its edges (an arc, its use, its polyline placed in the window), the
/// loops walked from them, each loop's polygon and area (counter-clockwise a part on its left), and
/// each part's holes.
struct Graph { lines: Vec<(usize,usize,Vec<P>)>,loops: Vec<(Vec<(usize,bool)>,Vec<(usize,usize)>)>,polys: Vec<(Vec<P>,f64)>,holes: Vec<Vec<usize>> }

/// The window `(w0, w1)`, the `w`-th of the chart's (`cuts` its sides), read as a planar graph.
fn graph(tube: &Tube,chart: Chart,cuts: [f64;2],w: usize,arcs: &[Arc2],vertices: &[Vertex]) -> Result<Graph,Unbuilt> {
    let (w0,w1) = if w == 0 { (cuts[0],cuts[1]) } else { (cuts[1],cuts[0]+TAU) };
    let mut lines: Vec<(usize,usize,Vec<P>)> = Vec::new();
    for (i,arc) in arcs.iter().enumerate() {
        for (u,(c,line)) in arc.uses.iter().enumerate() {
            if *c != chart { continue }
            // (a cut across a chart is one edge of the graph, both its sides used: its first use
            // in the window it starts, at the window's side, its second in the one it ends;
            // a cut from a swallowtail point, or a seam, its first use alone)
            let target = match arc.kind {
                Kind::Across {th,..} if cuts.contains(&th) => {
                    if u == 0 && th == cuts[w] { Some(w0) } else if u == 1 && th == cuts[1-w] { Some(w1) } else { continue }
                }
                Kind::Across {..} | Kind::Along {..} if u == 1 && arc.uses[0].0 == chart => continue,
                _ => None,
            };
            let p = placed(tube,chart,line,w0,target).ok_or_else(|| failed("a line not placed".into()))?;
            let mid = p[p.len()/2][0];
            if target.is_none() && !(mid > w0 && mid < w1) { continue }
            lines.push((i,u,p));
        }
    }
    let loops = walk(&lines,arcs,vertices)?;
    let polys: Vec<(Vec<P>,f64)> = loops.iter().map(|(l,_)| {
        let poly: Vec<P> = l.iter().flat_map(|&(e,fwd)| {
            let mut p = lines[e].2.clone();
            if !fwd { p.reverse() }
            p.pop();
            p
        }).collect();
        let area: f64 = (0..poly.len()).map(|j| { let (p,q) = (poly[j],poly[(j+1)%poly.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
        (poly,area)
    }).collect();
    // (a loop turning clockwise is the outside of its part of the graph: the window's own, or —
    // where it meets no node of a part turning counter-clockwise and stands inside it — an island
    // in that part, a hole of the least such)
    let mut holes: Vec<Vec<usize>> = vec![Vec::new();loops.len()];
    for (j,(pj,aj)) in polys.iter().enumerate() {
        if *aj > 0. { continue }
        let host = (0..loops.len()).filter(|&i| polys[i].1 > 0. && !loops[i].1.iter().any(|n| loops[j].1.contains(n))
            && inside(pj[0],&polys[i].0)).min_by(|&x,&y| polys[x].1.total_cmp(&polys[y].1));
        if let Some(i) = host { holes[i].push(j) }
    }
    Ok(Graph {lines,loops,polys,holes})
}

impl Graph {
    /// The part `i`'s rings (its polygon, then its holes').
    fn rings(&self,i: usize) -> Vec<Vec<P>> {
        std::iter::once(self.polys[i].0.clone()).chain(self.holes[i].iter().map(|&j| self.polys[j].0.clone())).collect()
    }

    /// The part `i`'s rings with each point's edge: the line, its segment from that point and whether
    /// the line is walked forward.
    fn origins(&self,i: usize) -> Vec<Vec<(usize,usize,bool)>> {
        std::iter::once(i).chain(self.holes[i].iter().copied()).map(|k| {
            self.loops[k].0.iter().flat_map(|&(e,fwd)| {
                let n = self.lines[e].2.len();
                (0..n-1).map(move |j| (e,if fwd { j } else { n-2-j },fwd))
            }).collect()
        }).collect()
    }
}

/// **A kept part of a window, as the first walk reads it**: its chart and window, its rings, and its
/// cells (none on an end pose, which is its torus).
struct Kept { chart: Chart,w: usize,rings: Vec<Vec<P>>,cover: Option<super::cells::Cover>,step: f64 }

/// The parts of a chart's window kept: where the part's inner point bounds the sweep.
fn kept(tube: &Tube,chart: Chart,g: &Graph,i: usize,tol: f64) -> Result<bool,Unbuilt> {
    let (poly,area) = &g.polys[i];
    if *area <= 0. { return Ok(false) }
    let rings: Vec<&[P]> = std::iter::once(&poly[..]).chain(g.holes[i].iter().map(|&j| &g.polys[j].0[..])).collect();
    let inner = interior(&rings).ok_or_else(|| failed(format!("a part of {chart:?} with no inner point")))?;
    let q = match chart { Chart::Sheet(_) | Chart::Collar => inner,Chart::End(k) => {
        let (plus,way,_) = band(tube,k,inner[0]).ok_or_else(|| failed("no band".into()))?; [inner[0],plus+way*inner[1]] } };
    let field = Depth {tube,chart,within:tol,holes:&[]};
    Ok(match field.reading(q) { Some((_,Reading::Alone)) => true,Some((_,Reading::Far(c))) => c.depth >= 0.,_ => false })
}

/// **The faces of every chart**: each window's planar graph walked and each part kept where its inner
/// point bounds the sweep; each kept part of a sheet or of the collar covered by its cells
/// (`cells::Ask::cover`), the splits' lines inside it added as seams (`seamed`); the graphs walked
/// again, each part now one cell's, and sewn.
#[allow(clippy::too_many_arguments)]
fn faces(tube: &Tube,vertices: &mut Vec<Vertex>,arcs: &mut Vec<Arc2>,trims: &mut Vec<Trim>,cuts: &[(Chart,[f64;2])],folds: &[(Chart,Vec<Vec<P>>)],tol: f64)
    -> Result<Brep,Unbuilt> {
    let fit = FIT*tol;
    let mut charts = CHARTS.to_vec();
    if tube.pinch.is_some() { charts.push(Chart::Collar) }
    // the first walk: the kept parts and their cells
    let mut parts: Vec<Kept> = Vec::new();
    let mut seams: Vec<Seam> = Vec::new();
    for &chart in &charts {
        let cs = cuts_of(cuts,chart);
        for w in 0..2 {
            let g = graph(tube,chart,cs,w,arcs,vertices)?;
            for i in 0..g.loops.len() {
                if !kept(tube,chart,&g,i,tol)? { continue }
                let rings = g.rings(i);
                let (cover,step) = match chart {
                    Chart::End(_) => (None,0.),
                    _ => {
                        let window = if w == 0 { (cs[0],cs[1]) } else { (cs[1],cs[0]+TAU) };
                        let none = Vec::new();
                        let f = folds.iter().find(|(c,_)| *c == chart).map_or(&none,|f| &f.1);
                        let stops = super::cells::Stops::new(tube,chart,f,window)?;
                        if let Some(path) = std::env::var_os("SOLVENT_TUBE_DUMP") {
                            use std::io::Write;
                            if let Ok(mut f) = std::fs::OpenOptions::new().append(true).create(true).open(path) {
                                for r in &rings { let _ = writeln!(f,"face {chart:?} {w} {}",r.iter().map(|q| format!("{},{}",q[0],q[1])).collect::<Vec<_>>().join(" ")); }
                                for (l,_) in &stops.lines { let _ = writeln!(f,"stop {chart:?} {}",l.iter().map(|q| format!("{},{}",q[0],q[1])).collect::<Vec<_>>().join(" ")); }
                            }
                        }
                        let ask = super::cells::Ask {tube,chart,rings:rings.clone(),stops:&stops,tol};
                        let cover = ask.cover()?;
                        if debug() { eprintln!("tube: {chart:?} window {w}: a part in {} cells, {} splits",cover.leaves.len(),cover.splits.len()) }
                        let mid = rings[0][0];
                        let rate = tube.rates_at(chart,mid).map_or(1.,|m| m[0].max(m[1]));
                        (Some(cover),tol/rate)
                    }
                };
                if let Some(cover) = &cover { seams_of(tube,&g,i,chart,cover,arcs,&mut seams)? }
                parts.push(Kept {chart,w,rings,cover,step});
            }
        }
    }
    seamed(tube,vertices,arcs,trims,&seams)?;
    if debug() { for (i,a) in arcs.iter().enumerate() { if let Kind::Across {..} | Kind::Along {..} = a.kind { eprintln!("tube: arc {i} {:?} {:?}",a.kind,a.v) } } }
    // each arc's curve in space and its uses' pcurves, in the uses' own parameters
    let mut curves: Vec<(Arc<Curve>,Vec<BSpline>)> = Vec::new();
    for arc in arcs.iter() {
        let at = |u: usize,p: P| tube.at(arc.uses[u].0,p).map(|q| q.x);
        let (c,pcs) = match arc.kind {
            Kind::Trim(i) => {
                let t = &trims[i];
                let f = |s: f64| match t.at(tube,s) {
                    Some([u,p]) => (tube.at(t.own,u).map_or([f64::NAN;3],|q| q.x),vec![u,p]),
                    None => ([f64::NAN;3],vec![[f64::NAN;2];2]),
                };
                // (a join's partner held to the tolerance alone: the normal form's leaves stand
                // within it of one another, and a corner's join within it of the end pose)
                let bars = [fit,if let Shape::Traced = t.shape { fit } else { tol }];
                if debug() { eprintln!("tube: trim {i} on {:?} against {:?}, {:?}, from {:?} to {:?}",t.own,t.other,t.shape,t.uv[0],t.uv[t.uv.len()-1]) }
                fitted(&f,&at,&bars)?
            }
            Kind::Rim {side,k,th} => {
                let f = |s: f64| { let t = th[0]+(th[1]-th[0])*s; let x = tube.point(side,t,tube.roll[k]).map_or([f64::NAN;3],|p| p.x);
                    let e = tube.on_end(k,x).map_or(f64::NAN,|q| q[1]); (x,vec![[t,tube.roll[k]],[t,e]]) };
                let g = |s: f64| { let (x,mut u) = f(s); let e0 = f(0.).1[1][1]; u[1][1] = unwrap(u[1][1],e0); (x,u) };
                fitted(&g,&at,&[fit,fit])?
            }
            Kind::Across {chart,th,v} => {
                let f = |s: f64| { let p = [th,v[0]+(v[1]-v[0])*s]; (tube.at(chart,p).map_or([f64::NAN;3],|q| q.x),vec![p,p]) };
                fitted(&f,&at,&[fit,fit])?
            }
            Kind::Along {chart,v,th} => {
                let f = |s: f64| { let p = [th[0]+(th[1]-th[0])*s,v]; (tube.at(chart,p).map_or([f64::NAN;3],|q| q.x),vec![p,p]) };
                fitted(&f,&at,&[fit,fit])?
            }
            Kind::Hole {sign,psi} => {
                let pinch = tube.pinch.as_ref().ok_or_else(|| failed("a pinch's rim with no pinch".into()))?;
                let (rho,near) = (sign*pinch.rho,arc.uses[1].1[0][0]);
                let f = |s: f64| {
                    let a = psi[0]+(psi[1]-psi[0])*s;
                    let x = tube.collar_at(a,rho).map_or([f64::NAN;3],|q| q.x);
                    let q = pinch.rim(sign,a).unwrap_or([f64::NAN;2]);
                    (x,vec![[a,rho],[unwrap(q[0],near),q[1]]])
                };
                fitted(&f,&at,&[fit,fit])?
            }
        };
        if debug() { eprintln!("tube: arc {} {:?} {:?} mid {:?} ends {:?} {:?}",curves.len(),arc.kind,arc.v,c.point(0.5),c.point(0.),c.point(1.)) }
        curves.push((Arc::new(Curve::BSpline(Arc::new(c))),pcs));
    }
    // the second walk: each kept part's piece in the cell its inner point lies in
    let mut pieces: Vec<(Chart,(f64,f64),Vec<(usize,usize,Vec<P>)>,Vec<Vec<(usize,bool)>>,usize,Option<usize>)> = Vec::new();
    for &chart in &charts {
        let cs = cuts_of(cuts,chart);
        for w in 0..2 {
            let (w0,w1) = if w == 0 { (cs[0],cs[1]) } else { (cs[1],cs[0]+TAU) };
            let g = graph(tube,chart,cs,w,arcs,vertices)?;
            for i in 0..g.loops.len() {
                let (poly,area) = &g.polys[i];
                if *area <= 0. { continue }
                let rings: Vec<&[P]> = std::iter::once(&poly[..]).chain(g.holes[i].iter().map(|&j| &g.polys[j].0[..])).collect();
                let inner = interior(&rings).ok_or_else(|| failed(format!("a part of {chart:?} with no inner point")))?;
                // (the part of the first walk it lies in, kept, and its cell)
                let Some(k) = parts.iter().position(|p| p.chart == chart && p.w == w && within(inner,&p.rings)) else { continue };
                let l = &g.loops[i].0;
                let ls: Vec<Vec<(usize,bool)>> = std::iter::once(l.clone()).chain(g.holes[i].iter().map(|&j| g.loops[j].0.clone())).collect();
                if debug() { eprintln!("tube: kept {chart:?} window {w}: {:?}",l.iter().map(|&(e,_)| g.lines[e].0).collect::<Vec<_>>()) }
                let c = match &parts[k].cover {
                    None => None,
                    Some(cover) => Some(cover.leaves.iter().position(|(r,_)| inner[0] >= r[0][0] && inner[0] <= r[0][1] && inner[1] >= r[1][0] && inner[1] <= r[1][1])
                        .ok_or_else(|| failed(format!("a piece of {chart:?} at {inner:?} in no cell")))?),
                };
                if debug() { if let (Some(c),Some(cover)) = (c,&parts[k].cover) {
                    let bb = poly.iter().fold([[f64::INFINITY,f64::NEG_INFINITY];2],|b,p| [[b[0][0].min(p[0]),b[0][1].max(p[0])],[b[1][0].min(p[1]),b[1][1].max(p[1])]]);
                    eprintln!("tube: a piece of {chart:?} over {bb:?}, inner {inner:?}, in the leaf {:?}",cover.leaves[c].0)
                } }
                pieces.push((chart,(w0,w1),g.lines.clone(),ls,k,c));
            }
        }
    }
    // each cell's extent the hull of its pieces' edges as they are drawn: its run (and a box's sides)
    // the least and most of their pcurves' poles in the chart's parameters, as placed — each edge's
    // pcurve lies in its poles' hull, so every edge of the cell's faces is read inside the cell, and
    // where an edge is a seam its poles are the seam's own line, the cell's side at its split the
    // seam's very number (a run read from the first walk's polygons stops a chord's sag short of
    // where the edges settled)
    let mut hulls: BTreeMap<(usize,usize),[[f64;2];2]> = BTreeMap::new();
    for (_,_,lines,ls,k,c) in &pieces {
        let Some(c) = *c else { continue };
        let h = hulls.entry((*k,c)).or_insert([[f64::INFINITY,f64::NEG_INFINITY];2]);
        for l in ls { for &(e,_) in l {
            let (i,u,ref placed) = lines[e];
            let shift = placed[0][0]-arcs[i].uses[u].1[0][0];
            for p in &curves[i].1[u].poles { for (j,x) in [p[0]+shift,p[1]].into_iter().enumerate() { h[j] = [h[j][0].min(x),h[j][1].max(x)] } }
        } }
    }
    for (&(k,c),h) in &hulls {
        let Some(cover) = parts[k].cover.as_mut() else { continue };
        let cell = &mut cover.leaves[c].1;
        let (a,b) = (cell.axis.along(),cell.axis.across());
        cell.run = h[a];
        if let super::cells::Bound::At(y) = &mut cell.lo { *y = h[b][0] }
        if let super::cells::Bound::At(y) = &mut cell.hi { *y = h[b][1] }
    }
    // each piece's face, its net fitted once per cell and shared by every piece in it
    let mut out = Assembly::default();
    let mut used = vec![0usize;arcs.len()];
    let mut nets: BTreeMap<(usize,usize),Arc<crate::nurbs::Net>> = BTreeMap::new();
    for (chart,(w0,w1),lines,ls,k,c) in &pieces {
        let (chart,k) = (*chart,*k);
        let ls: Vec<&[(usize,bool)]> = ls.iter().map(|l| &l[..]).collect();
        {
            {
                let on = match (&parts[k].cover,*c) {
                    (None,_) | (_,None) => On::Torus,
                    (Some(cover),Some(c)) => {
                        let cell = &cover.leaves[c].1;
                        let net = match nets.get(&(k,c)) {
                            Some(n) => n.clone(),
                            None => {
                                let (net,err) = cell.net(tube,parts[k].step,fit)
                                    .ok_or_else(|| failed(format!("the cell {cell:?} of {chart:?} not fitted within {fit:e}")))?;
                                if debug() { eprintln!("tube: a cell of {chart:?} ({:?}, {:?}): {}×{} poles, fitted to {err:e}, read at 97² to {:e}",cell.axis,cell.run,
                                    net.poles.len(),net.poles[0].len(),super::cells::worst(tube,cell,&net,97,parts[k].step)) }
                                let net = Arc::new(net);
                                nets.insert((k,c),net.clone());
                                net
                            }
                        };
                        On::Cell(cell,net,parts[k].step)
                    }
                };
                for l in &ls { for &(e,_) in l.iter() { used[lines[e].0] += 1 } }
                face(tube,chart,(*w0,*w1),lines,&ls,arcs,&curves,vertices,on,&mut out,tol)?;
            }
        }
    }
    if debug() {
        for (i,n) in used.iter().enumerate() { if *n != 0 && *n != 2 { eprintln!("tube: arc {i} {:?} {:?} used {n} times",arcs[i].kind,arcs[i].v) } }
        for i in 0..arcs.len() { for j in i+1..arcs.len() {
            if used[i] == 0 || used[j] == 0 { continue }
            let (a,b) = (&curves[i].0,&curves[j].0);
            let same = |x: V,y: V| distance(x,y) <= tol;
            let (a0,a1,b0,b1) = (a.point(0.),a.point(1.),b.point(0.),b.point(1.));
            if (same(a0,b0) && same(a1,b1) || same(a0,b1) && same(a1,b0)) && same(a.point(0.5),b.point(0.5)) {
                eprintln!("tube: arcs {i} {:?} {:?} and {j} {:?} {:?} coincide, used {} and {}",arcs[i].kind,arcs[i].v,arcs[j].kind,arcs[j].v,used[i],used[j])
            }
        } }
    }
    crate::sew::assembled(out.brep,tol).map_err(|e| failed(format!("the faces assembled: {e}")))
}

/// Whether `p` lies inside the rings (the outer first, then its holes).
fn within(p: P,rings: &[Vec<P>]) -> bool { inside(p,&rings[0]) && !rings[1..].iter().any(|h| inside(p,h)) }

/// **The solid as its faces are made**: its B-rep, and the vertex and edge there of each of the
/// road's vertices and arcs met so far.
#[derive(Default)]
struct Assembly { brep: Brep,vertices: BTreeMap<usize,u32>,edges: BTreeMap<usize,u32> }

/// What a face's surface is: an end pose's torus, or a cell's net (its step for the cell's sides).
enum On<'a> { Torus,Cell(&'a super::cells::Cell,Arc<crate::nurbs::Net>,f64) }

/// How a seam ends: on an edge of its part (the arc, which of its uses, the place along that use's
/// polyline — its segment and the fraction along it — and the window's shift of that use's first
/// parameter), or on an earlier seam of its part (a junction).
#[derive(Clone,Copy,Debug)]
enum Ends { On { arc: usize,used: usize,s: f64,shift: f64 },Junction }

/// **A seam**: a split's line inside a kept part, between two of its ends — its chart, its way, the
/// line's value and its extent across (each placed in the window), and its ends.
#[derive(Clone,Debug)]
struct Seam { chart: Chart,axis: Axis,x: f64,ys: [f64;2],ends: [Ends;2] }

/// **The seams of the part `i` of a window's graph**: each split of its cells' (`cover.splits`) read
/// against the part's rings, the line's runs inside the part each a seam, its ends where it crosses
/// an edge of the part (each crossing read half-open, so a vertex is crossed once) or else on an
/// earlier split's line.
fn seams_of(tube: &Tube,g: &Graph,i: usize,chart: Chart,cover: &super::cells::Cover,arcs: &[Arc2],out: &mut Vec<Seam>) -> Result<(),Unbuilt> {
    let _ = tube;
    let rings = g.rings(i);
    let origins = g.origins(i);
    for &(axis,x,[y0,y1]) in &cover.splits {
        let (a,c) = (axis.along(),axis.across());
        let mut cs: Vec<(f64,Ends)> = Vec::new();
        for (r,ring) in rings.iter().enumerate() {
            let n = ring.len();
            for k in 0..n {
                let (p,q) = (ring[k],ring[(k+1)%n]);
                if (p[a] <= x) == (q[a] <= x) { continue }
                let f = (x-p[a])/(q[a]-p[a]);
                let y = p[c]+(q[c]-p[c])*f;
                if !(y >= y0 && y <= y1) { continue }
                let (e,seg,fwd) = origins[r][k];
                let (arc,used,ref line) = g.lines[e];
                let s = seg as f64+if fwd { f } else { 1.-f };
                cs.push((y,Ends::On {arc,used,s,shift:line[0][0]-arcs[arc].uses[used].1[0][0]}));
            }
        }
        cs.sort_by(|p,q| p.0.total_cmp(&q.0));
        let mut ys: Vec<(f64,Ends)> = Vec::new();
        if cs.first().is_none_or(|f| f.0 > y0) { ys.push((y0,Ends::Junction)) }
        ys.extend(cs);
        if ys.last().is_none_or(|l| l.0 < y1) { ys.push((y1,Ends::Junction)) }
        for w in ys.windows(2) {
            let m = 0.5*(w[0].0+w[1].0);
            if !(w[1].0 > w[0].0) || !within(axis.point(x,m),&rings) { continue }
            if debug() { eprintln!("tube: a seam of {chart:?} {axis:?} at {x} over {:?}, ends {:?}",[w[0].0,w[1].0],[w[0].1,w[1].1]) }
            out.push(Seam {chart,axis,x,ys:[w[0].0,w[1].0],ends:[w[0].1,w[1].1]});
        }
    }
    Ok(())
}

/// The polylines of a sheet's rim at the roll's end `k` over the angles `th`: on the sheet, and on
/// the end pose (its `ψ` beside the angle).
fn rim_uses(tube: &Tube,side: f64,k: usize,[t0,t1]: [f64;2]) -> Result<Vec<(Chart,Vec<P>)>,Unbuilt> {
    let e = tube.roll[k];
    let m = (((t1-t0).abs()/(TAU/256.)).ceil() as usize).max(4);
    let ths: Vec<f64> = (0..=m).map(|j| t0+(t1-t0)*j as f64/m as f64).collect();
    let own: Vec<P> = ths.iter().map(|&t| [t,e]).collect();
    let end: Vec<P> = ths.iter().map(|&t| tube.point(side,t,e).and_then(|p| tube.on_end(k,p.x)).map(|q| [t,q[1]]))
        .collect::<Option<_>>().ok_or_else(|| failed("a rim not read on its end pose".into()))?;
    Ok(vec![(Chart::Sheet(side),own),(Chart::End(k),end)])
}

/// The polylines of the pinch's rim on the sheet `sign` over the collar's angles `psi`: on the
/// collar, and on the sheet (its angle unwrapped along it).
fn hole_uses(tube: &Tube,sign: f64,[t0,t1]: [f64;2]) -> Result<Vec<(Chart,Vec<P>)>,Unbuilt> {
    let p = tube.pinch.as_ref().ok_or_else(|| failed("a pinch's rim with no pinch".into()))?;
    let rho = sign*p.rho;
    let m = (((t1-t0).abs()/(TAU/256.)).ceil() as usize).max(4);
    let psis: Vec<f64> = (0..=m).map(|j| t0+(t1-t0)*j as f64/m as f64).collect();
    let own: Vec<P> = psis.iter().map(|&a| [a,rho]).collect();
    let mut sheet: Vec<P> = psis.iter().map(|&a| p.rim(sign,a)).collect::<Option<_>>().ok_or_else(|| failed("the pinch's rim not read".into()))?;
    for j in 1..sheet.len() { sheet[j][0] = unwrap(sheet[j][0],sheet[j-1][0]) }
    Ok(vec![(Chart::Collar,own),(Chart::Sheet(sign),sheet)])
}

/// The polyline of a cut at the first parameter `th` over `v`, or of a seam at the second `v` over
/// `th` (both uses one line).
fn line_across(th: f64,[v0,v1]: [f64;2]) -> Vec<P> { (0..=32).map(|j| [th,v0+(v1-v0)*j as f64/32.]).collect() }
fn line_along(v: f64,[t0,t1]: [f64;2]) -> Vec<P> { (0..=32).map(|j| [t0+(t1-t0)*j as f64/32.,v]).collect() }

/// **The arc `i` split where a seam along `axis` meets its use `used` at `q`** (that use's chart's
/// parameters, `s` along its polyline): the point settled on the arc exactly — a trim's on its double
/// curve with the seam's parameter held (`settle`), a rim's, a cut's and a seam's where the seam's
/// line meets them, the pinch's rim's by its collar's angle halved to the rounding — a vertex of
/// every chart it lies on, and the arc its two parts (the first keeping its place).
#[allow(clippy::too_many_arguments)]
fn split_arc(tube: &Tube,vertices: &mut Vec<Vertex>,arcs: &mut Vec<Arc2>,trims: &mut Vec<Trim>,i: usize,used: usize,s: f64,q: P,axis: Axis)
    -> Result<usize,Unbuilt> {
    let held = match axis { Axis::Columns => [1.,0.],Axis::Rows => [0.,1.] };
    match arcs[i].kind.clone() {
        Kind::Trim(ti) => {
            let t = trims[ti].clone();
            let n = t.uv.len();
            if !matches!(t.shape,Shape::Traced) {
                // (a join, read evenly along its own parameter: its point meeting the seam's line
                // halved to the rounding within the segment of its polyline as it now stands that
                // the line crosses nearest `q` — a join split before is read again, so a place
                // along the polyline it was found on names another point of it — and each part its
                // own share of it, read again)
                let (a,c) = (axis.along(),axis.across());
                let g = |x: f64| t.at(tube,x).map(|p| p[used][a]-q[a]);
                let line = if used == 0 { &t.uv } else { &t.partner };
                let _ = s;
                let j = (0..n-1).filter(|&j| (line[j][a] <= q[a]) != (line[j+1][a] <= q[a]))
                    .min_by(|&i,&j| {
                        let y = |j: usize| { let (p,r) = (line[j],line[j+1]); (p[c]+(r[c]-p[c])*(q[a]-p[a])/(r[a]-p[a])-q[c]).abs() };
                        y(i).total_cmp(&y(j))
                    }).ok_or_else(|| failed(format!("a seam's line at {q:?} does not cross its join")))?;
                let (mut lo,mut hi) = (j as f64/(n-1) as f64,(j+1) as f64/(n-1) as f64);
                let mut gl = g(lo).ok_or_else(|| failed("a join not read at a seam".into()))?;
                loop {
                    let m = 0.5*(lo+hi);
                    if !(m > lo && m < hi) { break }
                    let gm = g(m).ok_or_else(|| failed("a join not read at a seam".into()))?;
                    if (gm > 0.) == (gl > 0.) { lo = m; gl = gm } else { hi = m }
                }
                let cut = 0.5*(lo+hi);
                let [u,w] = t.at(tube,cut).ok_or_else(|| failed("a join not read at a seam".into()))?;
                let x = tube.at(t.own,u).ok_or_else(|| failed("no chart at a seam's end on a join".into()))?.x;
                vertices.push(Vertex {x,on:vec![(t.own,u),(t.other,w)]});
                let v = vertices.len()-1;
                let part = |[f0,f1]: [f64;2],ends: [usize;2]| -> Result<Trim,Unbuilt> {
                    let shape = match t.shape { Shape::Normal {base,d,way,span} => {
                        let at = |f: f64| span[0]+(span[1]-span[0])*f;
                        Shape::Normal {base,d,way,span:[at(f0),at(f1)]} } ,ref other => other.clone() };
                    let line: Vec<[P;2]> = (0..=16).map(|k| t.at(tube,f0+(f1-f0)*k as f64/16.)).collect::<Option<_>>()
                        .ok_or_else(|| failed("a join not read at a seam".into()))?;
                    Ok(Trim {own:t.own,other:t.other,uv:line.iter().map(|p| p[0]).collect(),partner:line.iter().map(|p| p[1]).collect(),ends,shape})
                };
                let (first,second) = (part([0.,cut],[t.ends[0],v])?,part([cut,1.],[v,t.ends[1]])?);
                arcs[i] = Arc2 {kind:Kind::Trim(ti),v:first.ends,uses:vec![(first.own,first.uv.clone()),(first.other,first.partner.clone())]};
                trims[ti] = first;
                arcs.push(Arc2 {kind:Kind::Trim(trims.len()),v:second.ends,uses:vec![(second.own,second.uv.clone()),(second.other,second.partner.clone())]});
                trims.push(second);
                return Ok(v)
            }
            let j = (s.floor() as usize).min(n-2);
            let f = s-j as f64;
            let lerp = |l: &Vec<P>| [l[j][0]+(l[j+1][0]-l[j][0])*f,l[j][1]+(l[j+1][1]-l[j][1])*f];
            let [u,w] = if used == 0 { settle(tube,(t.own,t.other),[q,lerp(&t.partner)],held) }
                else { settle(tube,(t.other,t.own),[q,lerp(&t.uv)],held).map(|[w,u]| [u,w]) }
                .ok_or_else(|| failed(format!("a seam's end not settled on a trim at {q:?}")))?;
            let x = tube.at(t.own,u).ok_or_else(|| failed("no chart at a seam's end on a trim".into()))?.x;
            vertices.push(Vertex {x,on:vec![(t.own,u),(t.other,w)]});
            let v = vertices.len()-1;
            let mut first = t.clone();
            let mut second = t.clone();
            first.uv.truncate(j+1); first.uv.push(u);
            first.partner.truncate(j+1); first.partner.push(w);
            first.ends[1] = v;
            second.uv = std::iter::once(u).chain(t.uv[j+1..].iter().copied()).collect();
            second.partner = std::iter::once(w).chain(t.partner[j+1..].iter().copied()).collect();
            second.ends[0] = v;
            arcs[i] = Arc2 {kind:Kind::Trim(ti),v:first.ends,uses:vec![(first.own,first.uv.clone()),(first.other,first.partner.clone())]};
            trims[ti] = first;
            trims.push(second);
            let tj = trims.len()-1;
            let t2 = &trims[tj];
            arcs.push(Arc2 {kind:Kind::Trim(tj),v:t2.ends,uses:vec![(t2.own,t2.uv.clone()),(t2.other,t2.partner.clone())]});
            Ok(v)
        }
        Kind::Rim {side,k,th} => {
            if axis != Axis::Columns { return Err(failed(format!("a seam along a rim at {q:?}"))) }
            let e = tube.roll[k];
            let c = q[0];
            let x = tube.point(side,c,e).ok_or_else(|| failed("no sheet at a seam's end on a rim".into()))?.x;
            let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a seam's end on a rim".into()))?;
            vertices.push(Vertex {x,on:vec![(Chart::Sheet(side),[c,e]),(Chart::End(k),end)]});
            let v = vertices.len()-1;
            let [v0,v1] = arcs[i].v;
            arcs[i] = Arc2 {kind:Kind::Rim {side,k,th:[th[0],c]},v:[v0,v],uses:rim_uses(tube,side,k,[th[0],c])?};
            arcs.push(Arc2 {kind:Kind::Rim {side,k,th:[c,th[1]]},v:[v,v1],uses:rim_uses(tube,side,k,[c,th[1]])?});
            Ok(v)
        }
        Kind::Across {chart,th,v:[a,b]} => {
            if axis != Axis::Rows { return Err(failed(format!("a seam along a cut at {q:?}"))) }
            let y = q[1];
            let x = tube.at(chart,[th,y]).ok_or_else(|| failed("no chart at a seam's end on a cut".into()))?.x;
            vertices.push(Vertex {x,on:vec![(chart,[th,y])]});
            let v = vertices.len()-1;
            let [v0,v1] = arcs[i].v;
            arcs[i] = Arc2 {kind:Kind::Across {chart,th,v:[a,y]},v:[v0,v],uses:vec![(chart,line_across(th,[a,y]));2]};
            arcs.push(Arc2 {kind:Kind::Across {chart,th,v:[y,b]},v:[v,v1],uses:vec![(chart,line_across(th,[y,b]));2]});
            Ok(v)
        }
        Kind::Along {chart,v:at,th:[a,b]} => {
            if axis != Axis::Columns { return Err(failed(format!("a seam along a seam at {q:?}"))) }
            let c = q[0];
            let x = tube.at(chart,[c,at]).ok_or_else(|| failed("no chart at a seam's end on a seam".into()))?.x;
            vertices.push(Vertex {x,on:vec![(chart,[c,at])]});
            let v = vertices.len()-1;
            let [v0,v1] = arcs[i].v;
            arcs[i] = Arc2 {kind:Kind::Along {chart,v:at,th:[a,c]},v:[v0,v],uses:vec![(chart,line_along(at,[a,c]));2]};
            arcs.push(Arc2 {kind:Kind::Along {chart,v:at,th:[c,b]},v:[v,v1],uses:vec![(chart,line_along(at,[c,b]));2]});
            Ok(v)
        }
        Kind::Hole {sign,psi} => {
            let p = tube.pinch.as_ref().ok_or_else(|| failed("a pinch's rim with no pinch".into()))?;
            let rho = sign*p.rho;
            let line = &arcs[i].uses[used].1;
            let m = line.len()-1;
            let j = (s.floor() as usize).min(m-1);
            let at = |j: usize| psi[0]+(psi[1]-psi[0])*j as f64/m as f64;
            let c = if used == 0 {
                if axis != Axis::Columns { return Err(failed(format!("a seam along the pinch's rim at {q:?}"))) }
                q[0]
            } else {
                // (the collar's angle whose place on the sheet meets the seam's line, halved)
                let a = axis.along();
                let g = |x: f64| p.rim(sign,x).map(|r| if a == 0 { unwrap(r[0],q[0]) } else { r[1] }-q[a]);
                let (mut lo,mut hi) = (at(j),at(j+1));
                let (Some(mut gl),Some(_)) = (g(lo),g(hi)) else { return Err(failed("the pinch's rim not read at a seam".into())) };
                loop {
                    let mid = 0.5*(lo+hi);
                    if !(mid > lo.min(hi) && mid < lo.max(hi)) { break }
                    let gm = g(mid).ok_or_else(|| failed("the pinch's rim not read at a seam".into()))?;
                    if (gm > 0.) == (gl > 0.) { lo = mid; gl = gm } else { hi = mid }
                }
                0.5*(lo+hi)
            };
            let x = tube.collar_at(c,rho).ok_or_else(|| failed("no collar at a seam's end on its rim".into()))?.x;
            let place = p.rim(sign,c).ok_or_else(|| failed("the pinch's rim not read at a seam".into()))?;
            vertices.push(Vertex {x,on:vec![(Chart::Collar,[c,rho]),(Chart::Sheet(sign),place)]});
            let v = vertices.len()-1;
            let [v0,v1] = arcs[i].v;
            arcs[i] = Arc2 {kind:Kind::Hole {sign,psi:[psi[0],c]},v:[v0,v],uses:hole_uses(tube,sign,[psi[0],c])?};
            arcs.push(Arc2 {kind:Kind::Hole {sign,psi:[c,psi[1]]},v:[v,v1],uses:hole_uses(tube,sign,[c,psi[1]])?});
            Ok(v)
        }
    }
}

/// **The seams added to the faces' graph**: every arc a seam ends on split there (each arc's ends
/// split from its last along it, so those before stand in its first part), a vertex at each seam's
/// junction with an earlier one, and each seam an edge between its ends and the junctions on it —
/// a cut across its first parameter (`Kind::Across`) or along it (`Kind::Along`), its parameters
/// the chart's own (an angle within its turn).
fn seamed(tube: &Tube,vertices: &mut Vec<Vertex>,arcs: &mut Vec<Arc2>,trims: &mut Vec<Trim>,seams: &[Seam]) -> Result<(),Unbuilt> {
    // (each end on an edge: its arc, its place along it, the seam and which end)
    let mut on: Vec<(usize,f64,usize,usize)> = Vec::new();
    for (k,sm) in seams.iter().enumerate() {
        for e in 0..2 { if let Ends::On {arc,s,..} = sm.ends[e] { on.push((arc,s,k,e)) } }
    }
    on.sort_by(|p,q| p.0.cmp(&q.0).then(q.1.total_cmp(&p.1)));
    let mut ends: Vec<[f64;2]> = seams.iter().map(|s| s.ys).collect();
    let mut at: BTreeMap<(usize,usize),usize> = BTreeMap::new();
    for &(arc,s,k,e) in &on {
        let sm = &seams[k];
        let Ends::On {used,shift,..} = sm.ends[e] else { unreachable!() };
        let p = sm.axis.point(sm.x,sm.ys[e]);
        let q = [p[0]-shift,p[1]];
        let v = split_arc(tube,vertices,arcs,trims,arc,used,s,q,sm.axis)?;
        at.insert((k,e),v);
        // (the seam's end at the vertex's own place across its line, settled there)
        let c = sm.axis.across();
        let near = vertices[v].on.iter().filter(|(ch,_)| *ch == sm.chart).map(|(_,p)| *p)
            .min_by(|a,b| (unwrap(a[0],q[0])-q[0]).hypot(a[1]-q[1]).total_cmp(&(unwrap(b[0],q[0])-q[0]).hypot(b[1]-q[1])));
        if let Some(p) = near { ends[k][e] = if c == 0 { unwrap(p[0],q[0])+shift } else { p[1] } }
    }
    // (the chart's own parameters of a placed point: its first within its turn)
    let raw = |p: P| [p[0].rem_euclid(TAU),p[1]];
    // the junctions: a vertex at each, its placed point kept to find the seams through it
    let mut junctions: Vec<(Chart,P,usize)> = Vec::new();
    for (k,sm) in seams.iter().enumerate() {
        for e in 0..2 {
            if !matches!(sm.ends[e],Ends::Junction) { continue }
            let p = sm.axis.point(sm.x,sm.ys[e]);
            let v = match junctions.iter().find(|j| j.0 == sm.chart && j.1 == p) {
                Some(j) => j.2,
                None => {
                    let x = tube.at(sm.chart,raw(p)).ok_or_else(|| failed(format!("no chart at a seam's junction {p:?}")))?.x;
                    vertices.push(Vertex {x,on:vec![(sm.chart,raw(p))]});
                    junctions.push((sm.chart,p,vertices.len()-1));
                    vertices.len()-1
                }
            };
            at.insert((k,e),v);
        }
    }
    // each seam an edge between its ends and the junctions on it, in order along it
    for (k,sm) in seams.iter().enumerate() {
        let (a,c) = (sm.axis.along(),sm.axis.across());
        let mut stops: Vec<(f64,usize)> = vec![(ends[k][0],at[&(k,0)]),(ends[k][1],at[&(k,1)])];
        for j in &junctions {
            if j.0 == sm.chart && j.1[a] == sm.x && j.1[c] > sm.ys[0] && j.1[c] < sm.ys[1] { stops.push((j.1[c],j.2)) }
        }
        stops.sort_by(|p,q| p.0.total_cmp(&q.0));
        for w in stops.windows(2) {
            let (kind,line) = match sm.axis {
                Axis::Columns => { let th = sm.x.rem_euclid(TAU); (Kind::Across {chart:sm.chart,th,v:[w[0].0,w[1].0]},line_across(th,[w[0].0,w[1].0])) }
                Axis::Rows => {
                    let shift = w[0].0-w[0].0.rem_euclid(TAU);
                    let th = [w[0].0-shift,w[1].0-shift];
                    (Kind::Along {chart:sm.chart,v:sm.x,th},line_along(sm.x,th))
                }
            };
            arcs.push(Arc2 {kind,v:[w[0].1,w[1].1],uses:vec![(sm.chart,line.clone()),(sm.chart,line)]});
        }
    }
    Ok(())
}

/// A point inside a polygon with holes (`rings`, the outer first; even and odd crossings): the
/// middle of the widest run across it at the middle of its height.
fn interior(rings: &[&[P]]) -> Option<P> {
    let (lo,hi) = rings[0].iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),p| (l.min(p[1]),h.max(p[1])));
    for f in [0.5,0.3,0.7,0.2,0.8] {
        let y = lo+(hi-lo)*f;
        let mut xs: Vec<f64> = rings.iter().flat_map(|poly| (0..poly.len()).filter_map(move |j| {
            let (a,b) = (poly[j],poly[(j+1)%poly.len()]);
            ((a[1] <= y) != (b[1] <= y)).then(|| a[0]+(y-a[1])/(b[1]-a[1])*(b[0]-a[0]))
        })).collect();
        xs.sort_by(f64::total_cmp);
        if let Some(w) = xs.chunks(2).filter(|w| w.len() == 2).max_by(|a,b| (a[1]-a[0]).total_cmp(&(b[1]-b[0]))) {
            if w[1] > w[0] { return Some([0.5*(w[0]+w[1]),y]) }
        }
    }
    None
}

/// Whether `p` lies inside the polygon `poly` (even and odd crossings of the way out along `u`).
fn inside(p: P,poly: &[P]) -> bool {
    (0..poly.len()).filter(|&j| {
        let (a,b) = (poly[j],poly[(j+1)%poly.len()]);
        (a[1] <= p[1]) != (b[1] <= p[1]) && a[0]+(p[1]-a[1])/(b[1]-a[1])*(b[0]-a[0]) > p[0]
    }).count()%2 == 1
}

/// **A window's planar graph walked into loops**, each part on the left of its edges: at each
/// vertex the edges leaving it in order of their way out, and from an edge arriving the next one
/// clockwise from its own way back. Each loop its edges (a line of `lines`, and whether forward)
/// and the nodes it passes.
#[allow(clippy::type_complexity)]
fn walk(lines: &[(usize,usize,Vec<P>)],arcs: &[Arc2],vertices: &[Vertex]) -> Result<Vec<(Vec<(usize,bool)>,Vec<(usize,usize)>)>,Unbuilt> {
    use std::collections::BTreeMap;
    // (a node: a vertex at one of its places on the chart — a double curve's point has two on one
    // sheet — the place nearest the line's end, its angle and ψ read within a turn)
    let node = |e: usize,end: usize| -> (usize,usize) {
        let (i,u,_) = lines[e];
        let (chart,raw) = &arcs[i].uses[u];
        let v = arcs[i].v[end];
        let q = raw[if end == 0 { 0 } else { raw.len()-1 }];
        let apart = |p: P| {
            let d0 = (p[0]-q[0]+PI).rem_euclid(TAU)-PI;
            let d1 = if let Chart::End(_) = chart { (p[1]-q[1]+PI).rem_euclid(TAU)-PI } else { p[1]-q[1] };
            d0.dhypot(d1)
        };
        let k = vertices[v].on.iter().enumerate().filter(|(_,(c,_))| c == chart)
            .min_by(|x,y| apart(x.1.1).total_cmp(&apart(y.1.1))).map_or(0,|(k,_)| k);
        (v,k)
    };
    // the half-edges leaving each node, with their angle
    let mut out: BTreeMap<(usize,usize),Vec<(f64,usize,bool)>> = BTreeMap::new();
    for (e,(_,_,p)) in lines.iter().enumerate() {
        let (v0,v1) = (node(e,0),node(e,1));
        let ang = |q: P,r: P| (r[1]-q[1]).datan2(r[0]-q[0]);
        out.entry(v0).or_default().push((ang(p[0],p[1]),e,true));
        let n = p.len();
        out.entry(v1).or_default().push((ang(p[n-1],p[n-2]),e,false));
    }
    for l in out.values_mut() { l.sort_by(|x,y| x.0.total_cmp(&y.0)) }
    let mut used = vec![[false;2];lines.len()];
    let mut loops = Vec::new();
    for e0 in 0..lines.len() {
        for fwd0 in [true,false] {
            if used[e0][fwd0 as usize] { continue }
            let mut l = Vec::new();
            let mut nodes = Vec::new();
            let (mut e,mut fwd) = (e0,fwd0);
            loop {
                if used[e][fwd as usize] { break }
                used[e][fwd as usize] = true;
                l.push((e,fwd));
                let v = if fwd { node(e,1) } else { node(e,0) };
                nodes.push(v);
                let ring = out.get(&v).ok_or_else(|| failed("a vertex left by nothing".into()))?;
                // (the half-edge back along this one, and the next clockwise from it)
                let back = ring.iter().position(|&(_,f,w)| f == e && w == !fwd).ok_or_else(|| failed("an edge not left".into()))?;
                let next = ring[(back+ring.len()-1)%ring.len()];
                (e,fwd) = (next.1,next.2);
            }
            if l.len() < 2 { return Err(failed("a window's edge closing on itself".into())) }
            loops.push((l,nodes));
        }
    }
    Ok(loops)
}

/// **One kept part as a face**: on a sheet or the collar its cell's net, on an end pose its torus;
/// its loop's edges each the arc's curve and its use's pcurve in the surface's parameters — through
/// the cell's map, affine on a box (the pcurve's poles moved), else read through the map and fitted
/// (`pcurve_on`).
#[allow(clippy::too_many_arguments)]
fn face(tube: &Tube,chart: Chart,(w0,_): (f64,f64),lines: &[(usize,usize,Vec<P>)],loops: &[&[(usize,bool)]],arcs: &[Arc2],
    curves: &[(Arc<Curve>,Vec<BSpline>)],vertices: &[Vertex],on: On,out: &mut Assembly,tol: f64) -> Result<(),Unbuilt> {
    let _ = w0;
    let cache = std::cell::RefCell::new(BTreeMap::new());
    // the surface, and the map from the chart's parameters (placed) to the surface's: affine, or
    // through the cell
    let (surface,affine): (Surface,Option<Box<dyn Fn(P) -> P>>) = match &on {
        On::Cell(cell,net,_) => {
            let s = Surface::BSpline(Frame::WORLD,net.clone());
            if cell.boxed() {
                let (super::cells::Bound::At(lo),super::cells::Bound::At(hi)) = (&cell.lo,&cell.hi) else { unreachable!() };
                let (run,lo,hi,axis) = (cell.run,*lo,*hi,cell.axis);
                (s,Some(Box::new(move |p: P| {
                    let (x,y) = (p[axis.along()],p[axis.across()]);
                    let (a,b) = ((x-run[0])/(run[1]-run[0]),(y-lo)/(hi-lo));
                    match axis { Axis::Columns => [a,b],Axis::Rows => [b,a] }
                })))
            } else { (s,None) }
        }
        On::Torus => {
            let Chart::End(k) = chart else { return Err(failed(format!("a torus's face on {chart:?}"))) };
            let m = tube.path.at(tube.roll[k]).map_err(|_| failed("no pose at an end".into()))?;
            let f = &tube.ring.frame;
            let frame = Frame {o:m.point(f.at([0.,0.,tube.height])),x:m.vector(f.x),y:m.vector(f.y),z:m.vector(f.z)};
            (Surface::Torus(frame,tube.radius,tube.r),Some(Box::new(|p: P| p)))
        }
    };
    let mut rings: Vec<Vec<Coedge>> = Vec::new();
    // (the outer loop's first point: on an end pose a hole begins within half a turn of `ψ` of it)
    let mut start: Option<P> = None;
    for (n,l) in loops.iter().enumerate() {
    let mut uses = Vec::new();
    // (an end pose's pcurves each moved by whole turns of `ψ` to start where the last ended)
    let mut last: Option<P> = start;
    for &(e,fwd) in l.iter() {
        let (i,u,ref placed) = lines[e];
        let arc = &arcs[i];
        let (curve,pcs) = &curves[i];
        let pc = &pcs[u];
        // (the pcurve's own angle moved by the turns the line's placing moved it, and on an end
        // pose its ψ by the turns that meet the last)
        let shift_th = placed[0][0]-arc.uses[u].1[0][0];
        let begin = if fwd { pc.point(0.) } else { pc.point(1.) };
        let shift_v = match (chart,last) {
            (Chart::End(_),Some(q)) => { let s = [begin[0]+shift_th,begin[1]]; unwrap(s[1],q[1])-s[1] }
            _ => 0.,
        };
        let pcurve = match &affine {
            Some(map) => {
                let poles: Vec<V> = pc.poles.iter().map(|&p| { let q = map([p[0]+shift_th,p[1]+shift_v]); [q[0],q[1],0.] }).collect();
                BSpline {poles,..pc.clone()}
            }
            None => {
                let On::Cell(cell,net,step) = &on else { unreachable!() };
                let map = |p: P| cell.unit(tube,[p[0]+shift_th,p[1]],*step,&cache);
                pcurve_on(pc,&map,net,FIT*tol)?
            }
        };
        if debug() {
            let worst = (0..=16).map(|k| { let t = k as f64/16.; let q = pcurve.point(t); distance(surface.point([q[0],q[1]]),curve.point(t)) }).fold(0f64,f64::max);
            if worst > tol { eprintln!("tube: an edge {:?} of {chart:?} off its face by {worst:e}, the face's cell {}",arc.kind,match &on { On::Cell(c,..) => format!("{:?} {:?} {:?} {:?}",c.axis,c.run,c.lo,c.hi),On::Torus => "torus".into() }) }
        }
        if start.is_none() { let b = if fwd { pcurve.point(0.) } else { pcurve.point(1.) }; start = Some([b[0],b[1]]) }
        let end = if fwd { pcurve.point(1.) } else { pcurve.point(0.) };
        last = Some([end[0],end[1]]);
        let [v0,v1] = arc.v;
        // (the arc's one edge, its vertices the arc's, shared by every face using it)
        let edge = match out.edges.get(&i) { Some(&k) => k,None => {
            let mut v = |k: usize| *out.vertices.entry(k).or_insert_with(|| out.brep.vertex(vertices[k].x));
            let ends = [v(v0),v(v1)];
            out.brep.edges.push(Edge {curve:EdgeCurve::Curve((**curve).clone()),t:[0.,1.],v:ends,tol:0.});
            let k = (out.brep.edges.len()-1) as u32;
            out.edges.insert(i,k);
            k
        } };
        uses.push(Coedge {edge,reversed:!fwd,pcurve:Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(pcurve))))});
    }
    // (the outer loop counter-clockwise in the surface's own parameters, a hole clockwise: its
    // pcurves' polygon's area says which way round it turns — an end pose's band may run down `ψ`)
    let mut ring: Vec<P> = Vec::new();
    for c in &uses {
        let Pcurve::Curve(pc) = &c.pcurve else { unreachable!("a pcurve of its own") };
        for j in 0..8 { let s = j as f64/8.; let q = pc.point(if c.reversed { 1.-s } else { s }); ring.push([q[0],q[1]]) }
    }
    let area: f64 = (0..ring.len()).map(|j| { let (p,q) = (ring[j],ring[(j+1)%ring.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
    if (area < 0.) == (n == 0) { uses = crate::topo::reverse_loop(uses) }
    rings.push(uses);
    }
    out.brep.faces.push(Face {surface,reversed:false,loops:rings,name:format!("{chart:?}")});
    Ok(())
}

/// **An edge's pcurve on a cell's net**: its pcurve in the chart's parameters `pc` read through the
/// cell's map (`map`) at evenly spaced parameters, doubled until at their quarters the net at the
/// fitted pcurve stands within `bar` of the net at the pcurve read through the map (the pcurve's own
/// share, as a box's pcurve moved affinely adds none: the net's and the curve's fits are each held
/// to their bar).
fn pcurve_on(pc: &BSpline,map: &dyn Fn(P) -> Option<P>,net: &crate::nurbs::Net,bar: f64) -> Result<BSpline,Unbuilt> {
    let mut n = 8;
    loop {
        let ts: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        // (a point read past the cell's side, within the tolerance of it — an edge beside a
        // swallowtail point, where the cell's side is the cuspidal edge, or a corner settled a
        // rounding past the cell's run — read by the net on the side, which it clamps to)
        let on = |t: f64| { let p = pc.point(t); map([p[0],p[1]]) };
        let pts: Vec<V> = ts.iter().map(|&t| on(t).map_or([f64::NAN;3],|q| [q[0],q[1],0.])).collect();
        if pts.iter().flatten().any(|x| !x.is_finite()) { return Err(failed("a pcurve not read through its cell".into())) }
        let fitted = crate::nurbs::interpolate(&pts,&ts,DEGREE).ok_or_else(|| failed("a pcurve could not be interpolated on its cell".into()))?;
        let mut worst = 0f64;
        for i in 0..n { for q in [0.25,0.5,0.75] {
            let t = (i as f64+q)/n as f64;
            let p = fitted.point(t);
            let q = on(t).ok_or_else(|| failed("a pcurve not read through its cell".into()))?;
            worst = worst.max(distance(net.point(p[0],p[1]),net.point(q[0],q[1])));
        } }
        if worst <= bar { return Ok(fitted) }
        if n >= MOST_FIT {
            if debug() {
                for k in 0..=8 { let t = k as f64/8.; let p = pc.point(t); eprintln!("tube: pcurve at {t}: chart {:?} unit {:?} fitted {:?}",p,map([p[0],p[1]]),fitted.point(t)) }
            }
            return Err(failed(format!("a pcurve on its cell fits within {bar:e} only to {worst:e}")))
        }
        n *= 2;
    }
}
