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
use super::{Chart,Depth,Disc,Tube,edges,seams,swallowtails};
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
const DEGREE: usize = 5;
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
        let side = [1.,-1.].into_iter().min_by(|&s,&r| {
            let d = |s: f64| self.point(s,th,c.at).map_or(f64::INFINITY,|p| distance(p.x,x));
            d(s).total_cmp(&d(r))
        })?;
        Some((Chart::Sheet(side),self.settled(side,x,[th,c.at])?))
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
/// offsets `d` and kernel's way, from the point's parameters on each chart `base`), or straight in
/// its sheet's parameters to a corner on the end pose `k`.
#[derive(Clone,Debug)]
enum Shape { Traced,Normal { base: [P;2],d: [P;2],way: f64 },Straight { k: usize } }

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
            Shape::Normal {base,d,way} => {
                let o = crate::swept::rings::normal_form(d,way)(1.-s);
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
/// end `k` between two angles, or a cut at an angle across a sheet (between two instants) or
/// across an end pose's band (between two angles round its tube, `ψ` unwrapped).
#[derive(Clone,Debug)]
enum Kind { Trim(usize),Rim { side: f64,k: usize,th: [f64;2] },Across { chart: Chart,th: f64,v: [f64;2] } }

/// An edge of the faces: what it is, its vertices, and its uses, each a chart and its polyline
/// there (in the chart's own parameters, an end pose's `ψ` unwrapped along it).
#[derive(Clone,Debug)]
struct Arc2 { kind: Kind,v: [usize;2],uses: Vec<(Chart,Vec<P>)> }

/// The windows' cuts: two angles splitting the largest gap in angle every trim leaves free, at its
/// thirds. Refused where no gap is wider than a hundredth of a turn (structural: a trim round
/// every angle is a road not built).
fn windows(trims: &[Trim],points: &[P]) -> Result<[f64;2],Unbuilt> {
    let mut ths: Vec<f64> = trims.iter().flat_map(|t| t.uv.iter().chain(&t.partner).map(|p| p[0].rem_euclid(TAU))).collect();
    ths.extend(points.iter().map(|p| p[0].rem_euclid(TAU)));
    if ths.is_empty() { return Ok([0.5*PI,1.5*PI]) }
    ths.sort_by(f64::total_cmp);
    let n = ths.len();
    let (gap,at) = (0..n).map(|i| { let next = if i+1 < n { ths[i+1] } else { ths[0]+TAU }; (next-ths[i],ths[i]) })
        .max_by(|p,q| p.0.total_cmp(&q.0)).expect("some angles");
    if gap < 0.01*TAU { return Err(failed("the trims leave no angle free for the windows' cuts".into())) }
    Ok([(at+gap/3.).rem_euclid(TAU),(at+2.*gap/3.).rem_euclid(TAU)])
}

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
    for side in [1.,-1.] {
        let chart = Chart::Sheet(side);
        let (folds,_) = edges(tube,chart,tol,feature)?;
        let folds: Vec<Vec<P>> = folds.into_iter().map(|p| p.uv).collect();
        let points: Vec<P> = folds.iter().flat_map(|f| swallowtails(tube,side,f)).collect();
        // the corners: each fold's end on a rim, settled there
        let mut corners: Vec<(usize,P)> = Vec::new();
        for f in &folds {
            for q in [f[0],f[f.len()-1]] {
                let Some(k) = on_rim(q[1]) else { continue };
                let c = tube.corner(side,k,q[0]).ok_or_else(|| failed(format!("a corner of the sheet {side} at {q:?} not settled")))?;
                if corners.iter().any(|(_,d)| (d[0]-c[0]).abs() <= HALF_READ) { continue }
                let x = tube.point(side,c[0],c[1]).ok_or_else(|| failed("no sheet at a corner".into()))?.x;
                let e = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a corner".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,c),(Chart::End(k),e)]});
                corners.push((vertices.len()-1,c));
            }
        }
        let (pieces,_,discs) = seams(tube,chart,&folds,&points,&[],tol,feature)?;
        let field = Depth {tube,chart,within:tol,holes:&discs};
        // the pieces joined across the cuts `round` traces in
        let mut runs: Vec<(Vec<P>,Vec<Cover>)> = pieces.into_iter().map(|p| (p.uv,p.at)).collect();
        let same = |p: P,q: P| (p[0]-q[0]+PI).rem_euclid(TAU)-PI == 0. && p[1] == q[1]
            || ((p[0]-q[0]+PI).rem_euclid(TAU)-PI).abs() <= HALF_READ && (p[1]-q[1]).abs() <= HALF_READ*(b-a);
        'merge: loop {
            for i in 0..runs.len() { for j in 0..runs.len() {
                if i == j { continue }
                let (pi,pj) = (runs[i].0[runs[i].0.len()-1],runs[j].0[0]);
                if same(pi,pj) {
                    let (mut u,mut c) = runs.remove(j);
                    let i = if j < i { i-1 } else { i };
                    u.remove(0); c.remove(0);
                    runs[i].0.extend(u); runs[i].1.extend(c);
                    continue 'merge
                }
                let pj = runs[j].0[runs[j].0.len()-1];
                if i < j && same(pi,pj) {
                    let (mut u,mut c) = runs.remove(j);
                    u.reverse(); c.reverse();
                    u.remove(0); c.remove(0);
                    runs[i].0.extend(u); runs[i].1.extend(c);
                    continue 'merge
                }
            } }
            break
        }
        // each piece cut into runs by what covers it, and the runs kept
        let class = |c: &Cover| on_rim(c.at).map_or(2,|k| k);
        let mut kept: Vec<(Vec<P>,Vec<Cover>,[bool;2])> = Vec::new();
        for (uv,at) in runs {
            let n = uv.len();
            let mut from = 0;
            for i in 1..=n {
                if i < n && class(&at[i]) == class(&at[from]) { continue }
                let (u,c) = (uv[from..i].to_vec(),at[from..i].to_vec());
                let m = c.len()/2;
                // (a double curve of the sheets traced on both preimages: the later one's kept)
                let keep = class(&c[m]) != 2 || u[m][1] > c[m].at;
                // (whether each end is a switch of what covers it)
                if keep && !u.is_empty() { kept.push((u,c,[from > 0,i < n])) }
                from = i;
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
            // (each angle unwrapped along the run, and an end pose's ψ)
            let (mut uv,mut partner) = (uv,partner);
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
                let (q,c) = (t.uv[i],at[i].clone());
                if switch[e] { continue }
                // a rim crossing
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
    // the switches: each the partner of a rim crossing on the switching trim's chart
    for ti in 0..trims.len() {
        for e in 0..2 {
            if trims[ti].ends[e] != usize::MAX { continue }
            let t = &trims[ti];
            let i = if e == 0 { 0 } else { t.uv.len()-1 };
            let q = t.uv[i];
            let apart = |p: P| crate::plane2::dist([unwrap(p[0],q[0]),p[1]],q);
            let near = |v: &Vertex| v.on.iter().filter(|(c,_)| *c == t.own).map(|(_,p)| apart(*p)).fold(f64::INFINITY,f64::min);
            let (v,_) = vertices.iter().enumerate().filter(|(_,v)| v.on.len() >= 3)
                .map(|(j,v)| (j,near(v))).min_by(|x,y| x.1.total_cmp(&y.1))
                .ok_or_else(|| failed(format!("a trim's cover switches at {q:?}, at no rim crossing")))?;
            // (the switch's own parameters: the rim crossing's partner on this chart, its partner
            // here the crossing's point on the end pose)
            let own = vertices[v].on.iter().filter(|(c,_)| *c == t.own).map(|(_,p)| *p)
                .min_by(|x,y| apart(*x).total_cmp(&apart(*y))).expect("on the chart");
            let other = vertices[v].on.iter().find(|(c,_)| *c == t.other).map(|(_,p)| *p)
                .ok_or_else(|| failed("a switch whose crossing is not on the trim's partner".into()))?;
            let t = &mut trims[ti];
            if e == 0 { t.uv.insert(0,own); t.partner.insert(0,other) } else { t.uv.push(own); t.partner.push(other) }
            t.ends[e] = v;
        }
    }
    // (each trim's angles unwrapped along it once its ends are in place, and an end pose's ψ)
    for t in &mut trims {
        for j in 1..t.uv.len() {
            t.uv[j][0] = unwrap(t.uv[j][0],t.uv[j-1][0]);
            t.partner[j][0] = unwrap(t.partner[j][0],t.partner[j-1][0]);
            if let Chart::End(_) = t.other { t.partner[j][1] = unwrap(t.partner[j][1],t.partner[j-1][1]) }
        }
    }
    build(tube,&mut vertices,&trims,&tails,tol)
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
    let mut join = Trim {own:t.own,other:t.other,uv:vec![],partner:vec![],ends:[junction,tail],shape:Shape::Normal {base,d:pair,way:k[1]/k[0]}};
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
    let span = distance(xq,x(c).ok_or_else(|| failed("no sheet at a corner".into()))?);
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
        if n >= MOST_FIT { return Err(failed(format!("a curve fits within {fit:e} only to {worst:e}"))) }
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
fn build(tube: &Tube,vertices: &mut Vec<Vertex>,trims: &[Trim],tails: &[(f64,P,usize)],tol: f64) -> Result<Brep,Unbuilt> {
    let [a,b] = tube.roll;
    let points: Vec<P> = tails.iter().map(|t| t.1).collect();
    let cuts = windows(trims,&points)?;
    let mut arcs: Vec<Arc2> = Vec::new();
    for (i,t) in trims.iter().enumerate() {
        arcs.push(Arc2 {kind:Kind::Trim(i),v:t.ends,uses:vec![(t.own,t.uv.clone()),(t.other,t.partner.clone())]});
    }
    // the cuts from each swallowtail point to the rim it opens away from: the way whose point a
    // disc's span beside it is kept
    let mut feet: Vec<(f64,usize,f64)> = Vec::new();
    for &(side,p,v) in tails {
        let chart = Chart::Sheet(side);
        let step = 0.02*(b-a);
        let kept = |t: f64| match tube.reading(side,p[0],t,tol) { Some(Reading::Alone) => true,Some(Reading::Far(c)) => c.depth >= 0.,_ => false };
        let k = if kept(p[1]-step) { 0 } else if kept(p[1]+step) { 1 } else {
            return Err(failed(format!("neither way from the swallowtail point {p:?} is kept")))
        };
        let e = tube.roll[k];
        let x = tube.point(side,p[0],e).ok_or_else(|| failed("no sheet at a cut's foot".into()))?.x;
        let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a cut's foot".into()))?;
        vertices.push(Vertex {x,on:vec![(chart,[p[0],e]),(Chart::End(k),end)]});
        let foot = vertices.len()-1;
        let line: Vec<P> = (0..=32).map(|j| [p[0],p[1]+(e-p[1])*j as f64/32.]).collect();
        arcs.push(Arc2 {kind:Kind::Across {chart,th:p[0],v:[p[1],e]},v:[v,foot],uses:vec![(chart,line.clone()),(chart,line)]});
        feet.push((side,k,p[0]));
    }
    // the rims, split at every vertex on them and at the cuts
    for side in [1.,-1.] {
        for k in 0..2 {
            let e = tube.roll[k];
            let chart = Chart::Sheet(side);
            let mut splits: Vec<(f64,usize)> = vertices.iter().enumerate()
                .flat_map(|(j,v)| v.on.iter().filter(|(c,p)| *c == chart && p[1] == e).map(move |(_,p)| (p[0].rem_euclid(TAU),j))).collect();
            for &c in &cuts {
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
                let m = (((t1-t0)/(TAU/256.)).ceil() as usize).max(4);
                let ths: Vec<f64> = (0..=m).map(|j| t0+(t1-t0)*j as f64/m as f64).collect();
                let own: Vec<P> = ths.iter().map(|&t| [t,e]).collect();
                let end: Vec<P> = ths.iter().map(|&t| tube.point(side,t,e).and_then(|p| tube.on_end(k,p.x)).map(|q| [t,q[1]]))
                    .collect::<Option<_>>().ok_or_else(|| failed("a rim not read on its end pose".into()))?;
                arcs.push(Arc2 {kind:Kind::Rim {side,k,th:[t0,t1]},v:[v0,v1],uses:vec![(chart,own),(Chart::End(k),end)]});
            }
        }
    }
    // the cuts across each chart at the windows' angles
    let at_rim = |vs: &Vec<Vertex>,chart: Chart,q: P| vs.iter().position(|v| v.on.iter().any(|(c,p)| *c == chart && p[0] == q[0] && p[1] == q[1]));
    for &c in &cuts {
        for side in [1.,-1.] {
            let chart = Chart::Sheet(side);
            let (v0,v1) = (at_rim(vertices,chart,[c,a]).expect("a rim's cut"),at_rim(vertices,chart,[c,b]).expect("a rim's cut"));
            let line: Vec<P> = (0..=32).map(|j| [c,a+(b-a)*j as f64/32.]).collect();
            arcs.push(Arc2 {kind:Kind::Across {chart,th:c,v:[a,b]},v:[v0,v1],uses:vec![(chart,line.clone()),(chart,line)]});
        }
        for k in 0..2 {
            let chart = Chart::End(k);
            let (plus,way,width) = band(tube,k,c).ok_or_else(|| failed("no band on an end pose".into()))?;
            let (v0,v1) = (at_rim(vertices,Chart::Sheet(1.),[c,tube.roll[k]]).expect("a rim's cut"),at_rim(vertices,Chart::Sheet(-1.),[c,tube.roll[k]]).expect("a rim's cut"));
            let line: Vec<P> = (0..=32).map(|j| [c,plus+way*width*j as f64/32.]).collect();
            arcs.push(Arc2 {kind:Kind::Across {chart,th:c,v:[plus,plus+way*width]},v:[v0,v1],uses:vec![(chart,line.clone()),(chart,line)]});
        }
    }
    faces(tube,vertices,&arcs,trims,cuts,tol)
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
            Chart::Sheet(_) => Some([th,q[1]]),
            Chart::End(k) => {
                let (plus,way,_) = band(tube,k,q[0])?;
                let o = ((q[1]-plus)*way).rem_euclid(TAU);
                // (on the band's start itself, nought, not a turn)
                Some([th,if o > TAU-1e-9 { 0. } else { o }])
            }
        }
    }).collect()
}

/// **The faces of every chart**, each window's planar graph walked, each part kept where its inner
/// point bounds the sweep, sewn.
fn faces(tube: &Tube,vertices: &[Vertex],arcs: &[Arc2],trims: &[Trim],cuts: [f64;2],tol: f64) -> Result<Brep,Unbuilt> {
    let fit = FIT*tol;
    // each arc's curve in space and its uses' pcurves, in the uses' own parameters
    let mut curves: Vec<(Arc<Curve>,Vec<BSpline>)> = Vec::new();
    for arc in arcs {
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
        };
        curves.push((Arc::new(Curve::BSpline(Arc::new(c))),pcs));
    }
    let mut pieces: Vec<Brep> = Vec::new();
    for chart in [Chart::Sheet(1.),Chart::Sheet(-1.),Chart::End(0),Chart::End(1)] {
        for w in 0..2 {
            let (w0,w1) = if w == 0 { (cuts[0],cuts[1]) } else { (cuts[1],cuts[0]+TAU) };
            // the window's edges: (arc, use, its polyline placed)
            let mut lines: Vec<(usize,usize,Vec<P>)> = Vec::new();
            for (i,arc) in arcs.iter().enumerate() {
                for (u,(c,line)) in arc.uses.iter().enumerate() {
                    if *c != chart { continue }
                    // (a cut across a chart is one edge of the graph, both its sides used: its first use
                    // in the window it starts, at the window's side, its second in the one it ends;
                    // a cut from a swallowtail point its first use alone)
                    let target = match arc.kind {
                        Kind::Across {th,..} if cuts.contains(&th) => {
                            if u == 0 && th == cuts[w] { Some(w0) } else if u == 1 && th == cuts[1-w] { Some(w1) } else { continue }
                        }
                        Kind::Across {..} if u == 1 && arc.uses[0].0 == chart => continue,
                        _ => None,
                    };
                    let p = placed(tube,chart,line,w0,target).ok_or_else(|| failed("a line not placed".into()))?;
                    let mid = p[p.len()/2][0];
                    if target.is_none() && !(mid > w0 && mid < w1) { continue }
                    lines.push((i,u,p));
                }
            }
            let loops = walk(&lines,arcs,vertices)?;
            for l in loops {
                let poly: Vec<P> = l.iter().flat_map(|&(e,fwd)| {
                    let p = &lines[e].2;
                    let mut p = p.clone();
                    if !fwd { p.reverse() }
                    p.pop();
                    p
                }).collect();
                let area: f64 = (0..poly.len()).map(|j| { let (p,q) = (poly[j],poly[(j+1)%poly.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
                if debug() {
                    let named = |e: usize| match arcs[lines[e].0].kind { Kind::Trim(i) => format!("trim {i}"),Kind::Rim {..} => "rim".into(),
                        Kind::Across {th,..} => format!("across {th}") };
                    eprintln!("tube: {chart:?} window {w}: a loop {:?}, area {area:e}",l.iter().map(|&(e,f)| (arcs[lines[e].0].v,f,named(e))).collect::<Vec<_>>())
                }
                if area <= 0. { continue }
                let inner = interior(&poly).ok_or_else(|| failed(format!("a part of {chart:?} with no inner point")))?;
                let q = match chart { Chart::Sheet(_) => inner,Chart::End(k) => {
                    let (plus,way,_) = band(tube,k,inner[0]).ok_or_else(|| failed("no band".into()))?; [inner[0],plus+way*inner[1]] } };
                let field = Depth {tube,chart,within:tol,holes:&[]};
                let keep = match field.reading(q) { Some((_,Reading::Alone)) => true,Some((_,Reading::Far(c))) => c.depth >= 0.,_ => false };
                if !keep { continue }
                pieces.push(face(tube,chart,(w0,w1),&lines,&l,arcs,&curves,vertices,tol)?);
            }
        }
    }
    crate::sew::sewn(&pieces,tol).map_err(|e| failed(format!("the faces sewn: {e}")))
}

/// A point inside a polygon: the middle of the widest run across it at the middle of its height.
fn interior(poly: &[P]) -> Option<P> {
    let (lo,hi) = poly.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),p| (l.min(p[1]),h.max(p[1])));
    for f in [0.5,0.3,0.7,0.2,0.8] {
        let y = lo+(hi-lo)*f;
        let mut xs: Vec<f64> = (0..poly.len()).filter_map(|j| {
            let (a,b) = (poly[j],poly[(j+1)%poly.len()]);
            ((a[1] <= y) != (b[1] <= y)).then(|| a[0]+(y-a[1])/(b[1]-a[1])*(b[0]-a[0]))
        }).collect();
        xs.sort_by(f64::total_cmp);
        if let Some(w) = xs.chunks(2).filter(|w| w.len() == 2).max_by(|a,b| (a[1]-a[0]).total_cmp(&(b[1]-b[0]))) {
            if w[1] > w[0] { return Some([0.5*(w[0]+w[1]),y]) }
        }
    }
    None
}

/// **A window's planar graph walked into loops**, each part on the left of its edges: at each
/// vertex the edges leaving it in order of their way out, and from an edge arriving the next one
/// clockwise from its own way back. Each loop its edges (a line of `lines`, and whether forward).
fn walk(lines: &[(usize,usize,Vec<P>)],arcs: &[Arc2],vertices: &[Vertex]) -> Result<Vec<Vec<(usize,bool)>>,Unbuilt> {
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
            let (mut e,mut fwd) = (e0,fwd0);
            loop {
                if used[e][fwd as usize] { break }
                used[e][fwd as usize] = true;
                l.push((e,fwd));
                let v = if fwd { node(e,1) } else { node(e,0) };
                let ring = out.get(&v).ok_or_else(|| failed("a vertex left by nothing".into()))?;
                // (the half-edge back along this one, and the next clockwise from it)
                let back = ring.iter().position(|&(_,f,w)| f == e && w == !fwd).ok_or_else(|| failed("an edge not left".into()))?;
                let next = ring[(back+ring.len()-1)%ring.len()];
                (e,fwd) = (next.1,next.2);
            }
            if l.len() < 2 { return Err(failed("a window's edge closing on itself".into())) }
            loops.push(l);
        }
    }
    Ok(loops)
}

/// **One kept part as a face**: on a sheet a net fitted over the window as a map, on an end pose its
/// torus; its loop's edges each the arc's curve and its use's pcurve, in the surface's parameters.
#[allow(clippy::too_many_arguments)]
fn face(tube: &Tube,chart: Chart,(w0,w1): (f64,f64),lines: &[(usize,usize,Vec<P>)],l: &[(usize,bool)],arcs: &[Arc2],
    curves: &[(Arc<Curve>,Vec<BSpline>)],vertices: &[Vertex],tol: f64) -> Result<Brep,Unbuilt> {
    let [a,b] = tube.roll;
    // the surface, and the map from the chart's parameters (placed) to the surface's
    let (surface,map): (Surface,Box<dyn Fn(P) -> P>) = match chart {
        Chart::Sheet(side) => {
            let f = |u: f64,v: f64| tube.point(side,w0+(w1-w0)*u,a+(b-a)*v).map_or([f64::NAN;3],|p| p.x);
            let (net,_) = crate::nurbs::fit_net(&f,DEGREE,DEGREE,32,16,FIT*tol).ok_or_else(|| failed(format!("the sheet {side} not fitted over a window")))?;
            (Surface::BSpline(Frame::WORLD,Arc::new(net)),Box::new(move |p: P| [(p[0]-w0)/(w1-w0),(p[1]-a)/(b-a)]))
        }
        Chart::End(k) => {
            let m = tube.path.at(tube.roll[k]).map_err(|_| failed("no pose at an end".into()))?;
            let f = &tube.ring.frame;
            let frame = Frame {o:m.point(f.at([0.,0.,tube.height])),x:m.vector(f.x),y:m.vector(f.y),z:m.vector(f.z)};
            (Surface::Torus(frame,tube.radius,tube.r),Box::new(|p: P| p))
        }
    };
    let mut out = Brep::default();
    let mut uses = Vec::new();
    // (an end pose's pcurves each moved by whole turns of `ψ` to start where the last ended)
    let mut last: Option<P> = None;
    let mut first: Option<u32> = None;
    for (j,&(e,fwd)) in l.iter().enumerate() {
        let (i,u,ref placed) = lines[e];
        let arc = &arcs[i];
        let (curve,pcs) = &curves[i];
        let pc = &pcs[u];
        // (the pcurve's own angle moved by the turns the line's placing moved it, and on an end
        // pose its ψ by the turns that meet the last)
        let shift_th = placed[0][0]-arc.uses[u].1[0][0];
        let start = if fwd { pc.point(0.) } else { pc.point(1.) };
        let shift_v = match (chart,last) {
            (Chart::End(_),Some(q)) => { let s = [start[0]+shift_th,start[1]]; unwrap(s[1],q[1])-s[1] }
            _ => 0.,
        };
        let moved = |p: V| map([p[0]+shift_th,p[1]+shift_v]);
        let poles: Vec<V> = pc.poles.iter().map(|&p| { let q = moved(p); [q[0],q[1],0.] }).collect();
        let pcurve = BSpline {poles,..pc.clone()};
        let end = if fwd { pcurve.point(1.) } else { pcurve.point(0.) };
        last = Some(match chart { Chart::End(_) => [end[0],end[1]],_ => [end[0],end[1]] });
        let [v0,v1] = arc.v;
        let (vs,ve) = if fwd { (v0,v1) } else { (v1,v0) };
        let start_v = if j == 0 { let id = out.vertex(vertices[vs].x); first = Some(id); id } else { (out.vertices.len()-1) as u32 };
        let end_v = if j+1 == l.len() { first.expect("a first vertex") } else { out.vertex(vertices[ve].x) };
        let ends = if fwd { [start_v,end_v] } else { [end_v,start_v] };
        out.edges.push(Edge {curve:EdgeCurve::Curve((**curve).clone()),t:[0.,1.],v:ends,tol:0.});
        uses.push(Coedge {edge:(out.edges.len()-1) as u32,reversed:!fwd,pcurve:Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(pcurve))))});
    }
    // (a loop counter-clockwise in the surface's own parameters: its pcurves' polygon's area says
    // which way round it turns — an end pose's band may run down `ψ`)
    let mut ring: Vec<P> = Vec::new();
    for c in &uses {
        let Pcurve::Curve(pc) = &c.pcurve else { unreachable!("a pcurve of its own") };
        for j in 0..8 { let s = j as f64/8.; let q = pc.point(if c.reversed { 1.-s } else { s }); ring.push([q[0],q[1]]) }
    }
    let area: f64 = (0..ring.len()).map(|j| { let (p,q) = (ring[j],ring[(j+1)%ring.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
    if area < 0. { uses = crate::topo::reverse_loop(uses) }
    out.faces.push(Face {surface,reversed:false,loops:vec![uses],name:format!("{chart:?}")});
    Ok(out)
}

