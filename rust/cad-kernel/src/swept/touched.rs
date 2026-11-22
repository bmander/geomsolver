//! **A ring touching a meridian whole over the roll, entered by depth** (#942, the depth trim): a
//! torus carried so that the velocity at every point of a meridian lies in the meridian's own
//! plane — along a path in a plane its axis lies square to, brep-bench's ring along a spline — touches
//! that meridian whole at every instant, and its contact away from it is one parallel each side,
//! `n · v = 0` solved round the tube at each angle round the axis. The sheet each parallel sweeps
//! folds exactly at the meridian (there the parallel's tangent is the velocity), into a leaf
//! before it and a leaf behind; the meridian's touching half sweeps a sheet of its own, the strip.
//!
//! What bounds the sweep is read for depth (`swept::depth`): of each parallel's sheet one leaf is
//! kept whole and the other is covered from beyond, but where the roll starts or stops, where the
//! other leaf stands outside every instant from its rim on the end pose's tool until it crosses
//! into that tool again — so that piece of it is kept, bounded by its rim and by that crossing,
//! whose far side on the tool is the wedge the end pose keeps there. Each is entered as a sheet of
//! its own over exactly what is kept, its closed form fitted: a leaf over the angle from the
//! meridian and the roll; the strip over the fraction round the meridian's half and the roll; the
//! end piece over the fraction of the way from its rim to the crossing and the angle, its corner on
//! the meridian at the end pose one point. Every meeting of two of them, and of each with the tool
//! at an end pose, is a curve both were built through, and given: the arrangement seeks none.
//!
//! **Cut exactly at the meridian.** Beside it the leaf behind, the fold and the leaf before stand
//! within 1e-7 of one another (the cover there cubic in the angle, under 4e-9 within 0.016 of it on
//! the bench's case): the kept leaf is taken from the meridian, whose seam stands off the true edge
//! of what is kept by at most 6.9e-8 there — against the bar of 0.005 the sweep is built to, and
//! within the tolerance's reach of the strip — and the topology is the strip's by construction.
use crate::boolean::Given;
use crate::geom::{Curve,Frame,Surface};
use crate::nurbs::{BSpline,Net};
use crate::topo::Brep;
use crate::trajectory::Trajectory;
use crate::util::space::{add,distance,dot,scale};
use super::closed::Unbuilt;
use super::depth::Sampled;
use std::f64::consts::{FRAC_PI_2,PI,TAU};
use std::sync::Arc;

type V = [f64;3];

/// What a ring touching a meridian whole enters: the tool where the roll starts and stops, the
/// sheets over what is kept, and their meetings, given.
pub struct Entered { pub ends: [Brep;2],pub sheets: Vec<Brep>,pub given: Vec<Given> }

/// The torus in the tool's coordinates: its frame (`z` its axis), its radii.
#[derive(Clone,Copy)]
struct Torus { f: Frame,big: f64,small: f64 }

impl Torus {
    /// Its point and normal at `α` round the axis and `ψ` round the tube, and their rates by `α`.
    fn at(&self,a: f64,p: f64) -> [V;4] {
        let f = &self.f;
        let er = add(scale(f.x,a.cos()),scale(f.y,a.sin()));
        let ea = add(scale(f.x,-a.sin()),scale(f.y,a.cos()));
        let n = add(scale(er,p.cos()),scale(f.z,p.sin()));
        let x = add(add(f.o,scale(er,self.big)),scale(n,self.small));
        [x,n,scale(ea,self.big+self.small*p.cos()),scale(ea,p.cos())]
    }
    /// Its signed distance at `x` (below nought inside).
    fn signed(&self,x: V) -> f64 {
        let l = self.f.local(x);
        (l[0].hypot(l[1])-self.big).hypot(l[2])-self.small
    }
}

/// The contact's function at an instant: `n · v` at `(α, ψ)`, divided by `sin(α − θ)` — its rate
/// by `α` on the meridian `θ` itself, where it is nought all round.
fn contact(torus: &Torus,m: &crate::trajectory::Motion,theta: f64,a: f64,p: f64) -> f64 {
    let [x,n,xa,na] = torus.at(a,p);
    let v = m.velocity(x);
    let nw = m.vector(n);
    if a == theta {
        dot(m.vector(na),v)+dot(nw,crate::util::space::sub(m.velocity(add(x,xa)),m.velocity(x)))
    } else {
        dot(nw,v)/(a-theta).sin()
    }
}

/// `contact` and its rate by `ψ` off the meridian `θ`, for Newton's steps.
fn contact_d(torus: &Torus,m: &crate::trajectory::Motion,theta: f64,a: f64,p: f64) -> (f64,f64) {
    let [x,n,..] = torus.at(a,p);
    let f = &torus.f;
    let er = add(scale(f.x,a.cos()),scale(f.y,a.sin()));
    let np = add(scale(er,-p.sin()),scale(f.z,p.cos()));
    let v = m.velocity(x);
    let nw = m.vector(n);
    let s = (a-theta).sin();
    let dv = crate::util::space::sub(m.velocity(add(x,scale(np,torus.small))),v);
    (dot(nw,v)/s,(dot(m.vector(np),v)+dot(nw,dv))/s)
}

/// The contact's root off the meridian from `guess`: Newton's steps while each is under half the
/// last, to where a step no longer moves it; the bracket's search (`root`) where they do not settle.
fn contact_root(torus: &Torus,m: &crate::trajectory::Motion,theta: f64,a: f64,guess: f64) -> Option<f64> {
    let mut p = guess;
    let mut last = f64::INFINITY;
    loop {
        let (g,dg) = contact_d(torus,m,theta,a,p);
        let step = g/dg;
        if !(step.abs() < 0.5*last) { break }
        let q = p-step;
        if q == p { return Some(p) }
        last = step.abs();
        p = q;
    }
    if last <= 4.*f64::EPSILON*(1.+p.abs()) { return Some(p) }
    root(&|q| contact(torus,m,theta,a,q),guess)
}

/// The root of `g` nearest `guess`: a bracket widened from a 64th of a turn about it (the scan the
/// meridian's roots are first read at) until `g` changes sign, then halved to the rounding.
fn root(g: &dyn Fn(f64) -> f64,guess: f64) -> Option<f64> {
    let g0 = g(guess);
    if g0 == 0. { return Some(guess) }
    let mut w = TAU/64.;
    while w <= PI {
        for s in [guess-w,guess+w] {
            if g(s)*g0 <= 0. {
                let (mut lo,mut hi) = if s < guess { (s,guess) } else { (guess,s) };
                let glo = g(lo);
                loop {
                    let mid = 0.5*(lo+hi);
                    if mid <= lo || mid >= hi { return Some(mid) }
                    if (g(mid) <= 0.) == (glo <= 0.) { lo = mid } else { hi = mid }
                }
            }
        }
        w *= 2.;
    }
    None
}

/// The contact's two parallels on the meridian `θ` at a motion: the roots of `contact` round the
/// tube, read at 64 places and halved; `None` unless there are exactly two.
fn parallels(torus: &Torus,m: &crate::trajectory::Motion,theta: f64) -> Option<[f64;2]> {
    const LOOKS: usize = 64;
    let g = |p: f64| contact(torus,m,theta,theta,p);
    let mut out = Vec::new();
    for k in 0..LOOKS {
        let (p,q) = (TAU*k as f64/LOOKS as f64,TAU*(k+1) as f64/LOOKS as f64);
        if g(p) == 0. || g(p)*g(q) < 0. { out.push(root(&g,0.5*(p+q))?.rem_euclid(TAU)) }
    }
    (out.len() == 2).then(|| [out[0],out[1]])
}

/// The angle nearest `p` of those `q` stands for, a whole number of turns apart.
fn near(q: f64,p: f64) -> f64 { p+(q-p+PI).rem_euclid(TAU)-PI }

/// The degree each sheet is fitted in either way: quintic, so a sheet held to a tenth of the
/// tolerance needs a few spans where a cubic needed hundreds (the leaves' 112 × 227 nodes), and
/// whatever reads it span by span (area, its export) reads that many fewer.
const DEGREE: usize = 5;

/// **A closed form fitted over a rectangle** of parameters: interpolated at the nodes `us` × `vs`
/// (`DEGREE` each way, rows along `v` then their poles along `u`), read again halfway between nodes
/// and each stretch whose middle stands off the closed form by more than `fit` halved, until none
/// does. `column(v, us)` gives the points at one `v` — the instant the expensive part of a point.
/// Several closed forms sharing one `vs` (a leaf and the strip it meets, their seams one curve)
/// are fitted together, a stretch of `v` halved where any needs it.
fn fitted(columns: &[&dyn Fn(f64,&[f64]) -> Result<Vec<V>,Unbuilt>],us: &mut [Vec<f64>],vs: &mut Vec<f64>,fit: f64)
    -> Result<Vec<Net>,Unbuilt> {
    // (as K2's fits: a net refined past a thousand nodes either way is refused, not built)
    const MOST: usize = 1024;
    loop {
        let mut nets = Vec::new();
        let mut split_v = vec![false;vs.len()-1];
        let mut splits_u: Vec<Vec<bool>> = Vec::new();
        for (k,column) in columns.iter().enumerate() {
            let u = &us[k];
            let mids = |x: &[f64]| -> Vec<f64> { x.windows(2).flat_map(|w| [w[0],0.5*(w[0]+w[1])]).chain(x.last().copied()).collect() };
            let (uu,vv) = (mids(u),mids(vs));
            let grid: Vec<Vec<V>> = vv.iter().map(|&v| column(v,&uu)).collect::<Result<_,_>>()?;
            // the nodes' points, row by row along `v`
            let rows: Vec<BSpline> = (0..u.len()).map(|i| crate::nurbs::interpolate(&(0..vs.len()).map(|j| grid[2*j][2*i]).collect::<Vec<_>>(),vs,DEGREE))
                .collect::<Option<_>>().ok_or_else(|| Unbuilt::Failed("a ring's touched sheet not interpolated".into()))?;
            let nv = rows[0].poles.len();
            let cols: Vec<BSpline> = (0..nv).map(|j| crate::nurbs::interpolate(&rows.iter().map(|r| r.poles[j]).collect::<Vec<_>>(),u,DEGREE))
                .collect::<Option<_>>().ok_or_else(|| Unbuilt::Failed("a ring's touched sheet not interpolated".into()))?;
            let poles: Vec<Vec<V>> = (0..cols[0].poles.len()).map(|i| cols.iter().map(|c| c.poles[i]).collect()).collect();
            let net = Net {du:cols[0].degree,dv:rows[0].degree,uknots:cols[0].knots.clone(),vknots:rows[0].knots.clone(),poles,weights:None};
            let mut split_u = vec![false;u.len()-1];
            // (a place halfway both ways off by more than `fit` splits the way its neighbours
            // halfway along one way say is off, both where neither is)
            let err: Vec<Vec<f64>> = vv.iter().enumerate().map(|(jj,&v)| uu.iter().enumerate()
                .map(|(ii,&x)| if ii%2 == 0 && jj%2 == 0 { 0. } else { distance(net.point(x,v),grid[jj][ii]) }).collect()).collect();
            for jj in 0..vv.len() { for ii in 0..uu.len() {
                let e = err[jj][ii];
                if !(e > fit) { continue }
                match (ii%2,jj%2) {
                    (1,0) => split_u[ii/2] = true,
                    (0,1) => split_v[jj/2] = true,
                    _ => {
                        let along_u = err[jj-1][ii].max(err[jj+1][ii]) > fit;
                        let along_v = err[jj][ii-1].max(err[jj][ii+1]) > fit;
                        if along_u || !along_v { split_u[ii/2] = true }
                        if along_v || !along_u { split_v[jj/2] = true }
                    }
                }
            } }
            splits_u.push(split_u);
            nets.push(net);
        }
        if crate::debug() {
            eprintln!("touched: fitted at {} × {:?}; split {} of v, {:?} of u",vs.len(),us.iter().map(|u| u.len()).collect::<Vec<_>>(),
                split_v.iter().filter(|&&s| s).count(),splits_u.iter().map(|s| s.iter().filter(|&&x| x).count()).collect::<Vec<_>>());
        }
        if !split_v.iter().any(|&s| s) && splits_u.iter().all(|s| !s.iter().any(|&x| x)) { return Ok(nets) }
        let halved = |x: &[f64],s: &[bool]| -> Vec<f64> {
            x.windows(2).zip(s).flat_map(|(w,&s)| if s { vec![w[0],0.5*(w[0]+w[1])] } else { vec![w[0]] }).chain(x.last().copied()).collect()
        };
        *vs = halved(vs,&split_v);
        for (u,s) in us.iter_mut().zip(&splits_u) { *u = halved(u,s) }
        if vs.len() > MOST || us.iter().any(|u| u.len() > MOST) {
            return Err(Unbuilt::Failed("a ring's touched sheet not fitted within a thousand nodes".into()))
        }
    }
}

/// One face over a net's whole domain (`build::grid_sheet`, a row of poles at one point its corner).
fn sheet(net: &Net,name: &str) -> Result<Brep,Unbuilt> {
    let [[u0,u1],[v0,v1]] = net.domain();
    crate::build::grid_sheet(net,&[u0,u1],&[v0,v1],&[name.to_string()],[false,false]).map_err(Unbuilt::Failed)
}

fn bspline(b: BSpline) -> Curve { Curve::BSpline(Arc::new(b)) }

/// **A ring touching a meridian whole, entered by depth** where it is cut from `blank` and its
/// other whole-touched meridian sweeps clear of it; `None` where no meridian is touched whole at
/// two of the closed class's poses (`looks`), and refused by name — `Curved` at the place it reads
/// otherwise — where what is kept is not as the module says: the tool no torus, the contact not two
/// parallels, a leaf not kept whole or not covered, an end piece not ending where it crosses the
/// tool, or a kept sheet held from beyond by more than the tolerance anywhere it is read.
#[allow(clippy::too_many_arguments)]
pub fn entered(tool: &Brep,path: &Trajectory,[a,b]: [f64;2],looks: usize,bar: f64,fit: f64,tol: f64,blank: &Brep)
    -> Result<Option<Entered>,Unbuilt> {
    let ring = super::rings::Ring::read(tool)?;
    let clock = crate::util::clock::Instant::now();
    let lap = |what: &str| if crate::timing() { eprintln!("touched: {what} at {:?}",clock.elapsed()) };
    let touched: Vec<(f64,f64,usize)> = (0..=looks).map(|k| a+(b-a)*k as f64/looks as f64)
        .map(|t| Ok::<_,Unbuilt>((t,ring.touched_whole(&crate::trajectory::Twist::body(path.at(t)?)))))
        .filter_map(|r| match r { Ok((t,Some((theta,k)))) => Some(Ok((t,theta,k))),Ok(_) => None,Err(e) => Some(Err(e)) })
        .collect::<Result<_,_>>()?;
    if touched.len() < 2 { return Ok(None) }
    let refused = |at: V| Unbuilt::Curved {at};
    let (t0,theta,k) = touched[0];
    let first = path.pose(t0)?.point(ring.meridian_point(theta,k));
    let torus = match tool.faces.iter().map(|f| &f.surface).collect::<Vec<_>>()[..] {
        [Surface::Torus(f,big,small)] => Torus {f:*f,big:*big,small:*small},
        _ => return Err(refused(first)),
    };
    // the two meridians touched whole: the one the blank holds, and its opposite, clear of it
    let (lo,hi) = blank.bounds();
    let within = |x: V| (0..3).all(|i| x[i] >= lo[i]-tol && x[i] <= hi[i]+tol);
    let m0 = torus.f.local(ring.meridian_point(theta,k));
    let alpha = m0[1].atan2(m0[0]);
    let instants: Vec<f64> = (0..=looks).map(|k| a+(b-a)*k as f64/looks as f64).collect();
    let swept_by = |th: f64| -> Result<Vec<V>,Unbuilt> {
        let mut out = Vec::new();
        for &t in &instants { let p = path.pose(t)?; for i in 0..8 { out.push(p.point(torus.at(th,TAU*i as f64/8.)[0])) } }
        Ok(out)
    };
    let theta = {
        let (x,y) = (swept_by(alpha)?,swept_by(alpha+PI)?);
        match (x.iter().any(|&p| within(p)),y.iter().any(|&p| within(p))) {
            (true,false) => alpha,
            (false,true) => alpha+PI,
            _ => return Err(refused(first)),
        }
    };
    // each instant's motion, and its parallels on the meridian, branch by branch from the first
    let motion = |t: f64| path.at(t).map_err(Unbuilt::from);
    let start = parallels(&torus,&motion(a)?,theta).ok_or_else(|| refused(first))?;
    let on_meridian = |t: f64| -> Result<[f64;2],Unbuilt> {
        let m = motion(t)?;
        let p = parallels(&torus,&m,theta).ok_or_else(|| refused(path.pose(t).map_or(first,|p| p.point(torus.at(theta,0.)[0]))))?;
        // (each branch the root nearest its start, the two taken by different branches: a parallel
        // drifting a quarter turn from where it started is refused, not followed onto the other)
        let pick = |s: f64| { let d = |q: f64| (near(q,s)-s).abs(); if d(p[0]) <= d(p[1]) { (0,near(p[0],s)) } else { (1,near(p[1],s)) } };
        let ((i0,q0),(i1,q1)) = (pick(start[0]),pick(start[1]));
        if i0 == i1 { return Err(refused(path.pose(t).map_or(first,|pose| pose.point(torus.at(theta,q0)[0])))) }
        Ok([q0,q1])
    };
    // a leaf's points at one instant: branch `j`, on the side `side` of the meridian (+1 or −1),
    // at angles `us` from it, each root sought from the last
    let leaf = |j: usize,side: f64,t: f64,us: &[f64]| -> Result<Vec<V>,Unbuilt> {
        let m = motion(t)?;
        let pose = path.pose(t)?;
        let mut p = on_meridian(t)?[j];
        us.iter().map(|&u| {
            let al = theta+side*u;
            if u != 0. { p = contact_root(&torus,&m,theta,al,p).ok_or_else(|| refused(pose.point(torus.at(al,p)[0])))? }
            Ok(pose.point(torus.at(al,p)[0]))
        }).collect()
    };
    let depth = Sampled::new(&ring,path,[a,b],((b-a)/super::closed::grain(tool,path,[a,b],bar)?).ceil().max(1.) as usize)?;
    // a place is kept where nothing beyond its own instant holds it by more than the tolerance
    let kept = |x: V,t: f64| depth.cover(x,t).is_none_or(|c| c.depth >= -tol);
    let mid = 0.5*(a+b);
    lap("the depth read");
    // **which leaf each branch keeps**: read halfway round to the equator, halfway along the roll
    let mut sides = [0f64;2];
    for j in 0..2 {
        let k: Vec<bool> = [1.,-1.].iter().map(|&s| leaf(j,s,mid,&[0.25*PI]).map(|x| kept(x[0],mid))).collect::<Result<_,_>>()?;
        sides[j] = match (k[0],k[1]) { (true,false) => 1.,(false,true) => -1.,_ => return Err(refused(leaf(j,1.,mid,&[0.25*PI])?[0])) };
    }
    // **the strip**: the meridian's half between the parallels that nothing beyond holds
    let strip_at = |sweep: f64,t: f64,us: &[f64]| -> Result<Vec<V>,Unbuilt> {
        let pose = path.pose(t)?;
        let p = on_meridian(t)?;
        let d = (p[1]-p[0]).rem_euclid(TAU)-if sweep < 0. { TAU } else { 0. };
        Ok(us.iter().map(|&u| pose.point(torus.at(theta,p[0]+u*d)[0])).collect())
    };
    let sweep = {
        let at = |s: f64| strip_at(s,mid,&[0.5]).map(|x| kept(x[0],mid));
        match (at(1.)?,at(-1.)?) { (true,false) => 1.,(false,true) => -1.,_ => return Err(refused(strip_at(1.,mid,&[0.5])?[0])) }
    };
    // **the leaves and the strip fitted together**, their seams on the meridian one curve
    let column_leaf = |j: usize| move |t: f64,us: &[f64]| leaf(j,sides[j],t,us);
    let (c0,c1) = (column_leaf(0),column_leaf(1));
    let cs = |t: f64,us: &[f64]| strip_at(sweep,t,us);
    let mut us = vec![(0..=8).map(|i| FRAC_PI_2*i as f64/8.).collect::<Vec<_>>(),(0..=8).map(|i| FRAC_PI_2*i as f64/8.).collect(),
        (0..=8).map(|i| i as f64/8.).collect()];
    let mut vs: Vec<f64> = (0..=16).map(|i| a+(b-a)*i as f64/16.).collect();
    let nets = fitted(&[&c0,&c1,&cs],&mut us,&mut vs,fit)?;
    lap("the leaves and the strip fitted");
    // **each end piece of a leaf not kept**: from its rim at the end pose to where it crosses that
    // pose's tool again, read as a graph over the angle from the meridian
    let mut pieces: Vec<(usize,usize,Net)> = Vec::new();
    for (e,end) in [a,b].into_iter().enumerate() {
        let way = if e == 0 { 1. } else { -1. };
        let back = path.pose(end)?.inverse();
        // (scanned at the bar's grain: a crossing into the tool and out again within less is not
        // read, and the piece is then refused where it is read for depth)
        let grain = super::closed::grain(tool,path,[a,b],bar)?;
        for j in 0..2 {
            let side = -sides[j];
            // (the leaf's point near the end: its root sought from the branch's on the meridian there)
            let rim = on_meridian(end)?[j];
            let at = |u: f64,t: f64| -> Result<V,Unbuilt> {
                let m = motion(t)?;
                let al = theta+side*u;
                let p = if u == 0. { rim } else { contact_root(&torus,&m,theta,al,rim).ok_or_else(|| refused(first))? };
                Ok(path.pose(t)?.point(torus.at(al,p)[0]))
            };
            let g = |u: f64,t: f64| at(u,t).map(|x| torus.signed(back.point(x)));
            // where the leaf crosses the tool: the first place from the rim inside it, scanned at
            // the roll's grain, then the last outside before it halved toward the rim, then halved
            // (near the corner the crossing comes in linearly with the angle, `slope` read halfway
            // round, and the scan steps a sixteenth of that way there: the leaf, past the crossing,
            // comes out of the tool again within less than the grain)
            let slope = std::cell::Cell::new(f64::INFINITY);
            let crossing = |u: f64| -> Result<Option<f64>,Unbuilt> {
                if u == 0. { return Ok(Some(end)) }
                let step = grain.min(slope.get()*u/16.);
                let mut s = step;
                let inside = loop {
                    if s > b-a { return Ok(None) }
                    if g(u,end+way*s)? < 0. { break s }
                    s += step;
                };
                let mut out = inside;
                loop {
                    out *= 0.5;
                    if out == 0. { return Ok(Some(end)) }
                    if g(u,end+way*out)? > 0. { break }
                }
                let (mut o,mut i) = (out,inside);
                loop {
                    let m = 0.5*(o+i);
                    if m <= o || m >= i { break }
                    if g(u,end+way*m)? > 0. { o = m } else { i = m }
                }
                Ok(Some(end+way*o))
            };
            let Some(tq) = crossing(0.25*PI)? else { continue };
            slope.set((tq-end).abs()/(0.25*PI));
            // **the corner, where the crossing is read no more**: there the piece between its rim and
            // its crossing stands within the rounding of the tool all across, so no sign of `g`
            // places the crossing; below the angle where the piece stands within `fit` of the tool
            // all across (read at eight places on the way), any place on the way lies on both to
            // `fit`, and the crossing is taken on the quadratic through nought and the crossing at
            // that angle and twice it — each of its places held to lying on the tool within `fit`
            let thick = |u: f64| -> Result<f64,Unbuilt> {
                let Some(tc) = crossing(u)? else { return Ok(f64::INFINITY) };
                (1..8).map(|i| g(u,end+(tc-end)*i as f64/8.).map(f64::abs)).try_fold(0f64,|m,x| x.map(|x| m.max(x)))
            };
            let mut thick_u = 0.25*PI;
            if thick(thick_u)? <= fit { return Err(refused(leaf(j,side,tq,&[thick_u])?[0])) }
            // (halved toward the corner until it is thin, then halved between to an eighth of the way)
            while thick(0.5*thick_u)? > fit { thick_u *= 0.5 }
            let mut thin = 0.5*thick_u;
            while thick_u-thin > 0.125*thick_u {
                let m = 0.5*(thin+thick_u);
                if thick(m)? <= fit { thin = m } else { thick_u = m }
            }
            let corner = thick_u;
            let (t1,t2) = (crossing(corner)?.unwrap_or(end)-end,crossing(2.*corner)?.unwrap_or(end)-end);
            // τ(u) = u (c0 + c1 u) through (corner, t1) and (2 corner, t2)
            let c1 = (t2/(2.*corner)-t1/corner)/corner;
            let c0 = t1/corner-c1*corner;
            let model = move |u: f64| end+u*(c0+c1*u);
            let crossing = |u: f64| -> Result<Option<f64>,Unbuilt> {
                if u >= corner { return crossing(u) }
                let t = model(u);
                if !(g(u,t)?.abs() <= fit) { return Err(refused(at(u,t)?)) }
                Ok(Some(t))
            };
            // kept halfway to the crossing, held just past it
            let x = at(0.25*PI,0.5*(end+tq))?;
            if !kept(x,0.5*(end+tq)) { continue }
            let column = |u: f64,ss: &[f64]| -> Result<Vec<V>,Unbuilt> {
                let tc = crossing(u)?.ok_or_else(|| refused(leaf(j,side,end,&[u]).map_or(first,|x| x[0])))?;
                ss.iter().map(|&s| at(u,end+s*(tc-end))).collect()
            };
            let mut uu = vec![(0..=8).map(|i| i as f64/8.).collect::<Vec<_>>()];
            let mut vv: Vec<f64> = (0..=8).map(|i| FRAC_PI_2*i as f64/8.).collect();
            let net = fitted(&[&column],&mut uu,&mut vv,fit)?.remove(0);
            // read for depth at its own instants, inside and along its rim and its crossing
            for i in 0..=6 { for k in 0..=6 {
                let (s,u) = (0.05+0.9*i as f64/6.,FRAC_PI_2*(0.05+0.9*k as f64/6.));
                let Some(tc) = crossing(u)? else { continue };
                let t = end+s*(tc-end);
                let x = at(u,t)?;
                if within(x) && !kept(x,t) { return Err(refused(x)) }
            } }
            pieces.push((e,j,net));
        }
    }
    lap("the end pieces fitted");
    // **every kept sheet read for depth**, inside and along its rims, and the equator and the far
    // meridian clear of the blank
    let read = |net: &Net,t_of: &dyn Fn(f64,f64) -> f64| -> Result<(),Unbuilt> {
        let [[u0,u1],[v0,v1]] = net.domain();
        for i in 0..=6 { for k in 0..=6 {
            let (u,v) = (u0+(u1-u0)*(0.05+0.9*i as f64/6.),v0+(v1-v0)*(0.05+0.9*k as f64/6.));
            let x = net.point(u,v);
            if !within(x) { continue }
            if !kept(x,t_of(u,v)) { return Err(refused(x)) }
        } }
        Ok(())
    };
    for net in &nets { read(net,&|_,v| v)? }
    for net in nets.iter().take(2) {
        let [[_,u1],[v0,v1]] = net.domain();
        for k in 0..=16 { let x = net.point(u1,v0+(v1-v0)*k as f64/16.); if within(x) { return Err(refused(x)) } }
    }
    for (_,_,net) in &pieces {
        let [[u0,u1],[_,v1]] = net.domain();
        for k in 0..=16 { let x = net.point(u0+(u1-u0)*k as f64/16.,v1); if within(x) { return Err(refused(x)) } }
    }
    lap("the sheets read");
    // **the tool at either end**, its seam kept away from the rims laid on it
    let rims: [Vec<V>;2] = [0,1].map(|e| nets.iter().flat_map(|n| {
        let [[u0,u1],[v0,v1]] = n.domain();
        let v = if e == 0 { v0 } else { v1 };
        (0..=8).map(move |i| n.point(u0+(u1-u0)*i as f64/8.,v))
    }).collect());
    let ends: [Brep;2] = [0,1].map(|e| -> Result<Brep,Unbuilt> {
        let pose = path.pose([a,b][e])?;
        let moved = tool.moved(&crate::geom::Rigid {r:pose.r,t:pose.p});
        Ok(super::closed::seam_away(moved,pose.point(torus.f.o),pose.vector(torus.f.z),&rims[e]))
    }).into_iter().collect::<Result<Vec<_>,_>>()?.try_into().map_err(|_| Unbuilt::Failed("no end".into()))?;
    let surface = |x: &Brep| x.faces[0].surface.clone();
    let mut sheets = Vec::new();
    for (i,net) in nets.iter().enumerate() { sheets.push(sheet(net,["touched leaf 0","touched leaf 1","touched strip"][i])?) }
    for (_,_,net) in &pieces { sheets.push(sheet(net,"touched end piece")?) }
    // **the meetings, given**: each leaf and the strip with the tool at either end in its rim; the
    // strip with each leaf in the leaf's seam on the meridian; an end piece with its end's tool in
    // its rim and its crossing, and touching its own branch's leaf and the strip at its corner
    let mut given = Vec::new();
    for (e,end) in ends.iter().enumerate() {
        for (i,net) in nets.iter().enumerate() {
            let c = if e == 0 { net.column(0) } else { net.column(net.poles[0].len()-1) };
            given.push(Given {surfaces:[surface(end),surface(&sheets[i])],curves:vec![bspline(c)],touches:Vec::new()});
        }
    }
    for j in 0..2 {
        given.push(Given {surfaces:[surface(&sheets[2]),surface(&sheets[j])],curves:vec![bspline(nets[j].row(0))],touches:Vec::new()});
    }
    for (k,(e,j,net)) in pieces.iter().enumerate() {
        let s = surface(&sheets[3+k]);
        given.push(Given {surfaces:[surface(&ends[*e]),s.clone()],curves:vec![bspline(net.row(0)),bspline(net.row(net.poles.len()-1))],
            touches:Vec::new()});
        let corner = net.point(0.,0.);
        given.push(Given {surfaces:[surface(&sheets[*j]),s.clone()],curves:Vec::new(),touches:vec![corner]});
        given.push(Given {surfaces:[surface(&sheets[2]),s],curves:Vec::new(),touches:vec![corner]});
    }
    if crate::debug() {
        eprintln!("touched: leaves on sides {sides:?}, strip {sweep}, nets {:?}, {} end pieces {:?}",nets.iter().map(|n| (n.poles.len(),n.poles[0].len())).collect::<Vec<_>>(),
            pieces.len(),pieces.iter().map(|p| (p.0,p.1)).collect::<Vec<_>>());
    }
    Ok(Some(Entered {ends,sheets,given}))
}
