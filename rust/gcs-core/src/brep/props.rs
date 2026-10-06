//! What a B-rep measures, read off its loops: the volume by the divergence theorem, each face's
//! flux turned into a line integral round its loops in its own parameters (Green's theorem), so
//! nothing is meshed and a face of revolution is integrated as exactly as a plane.
//!
//! `V = ⅓ Σ_F ∬_F X·n dA`, and on a face `X·n dA = s g(u, v) du dv` with `g = S·(S_u × S_v)` and
//! `s = ±1` its sense. With `G(u, v) = ∫_0^u g(σ, v) dσ`, Green's theorem gives
//! `∬_D g du dv = ∮_∂D G dv` round the domain counter-clockwise — and a face's loops run
//! counter-clockwise exactly when it is not reversed, so `∮ G dv` round its loops as stored is
//! its flux, whichever its sense. On a B-spline face `G` starts at its chart's first `u` and is
//! tabled exactly (`Prefix`), and every loop is closed in the face's parameters across the gaps a
//! kernel's loops leave between uses — Green's theorem holds for a closed curve only, and unclosed
//! the gaps times `G` were 3e-6 of the pinion, and moved with the origin.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{Curve,Surface,Uv};
use super::nurbs::Net;
use super::topo::{Brep,EdgeCurve,Pcurve};
use crate::space::{cross,dot};

/// Ten-point Gauss–Legendre nodes and weights on [−1, 1].
const GAUSS: [(f64,f64);10] = [
    (-0.9739065285171717,0.0666713443086881),(-0.8650633666889845,0.1494513491505806),
    (-0.6794095682990244,0.2190863625159820),(-0.4333953941292472,0.2692667193099963),
    (-0.1488743389816312,0.2955242247147529),(0.1488743389816312,0.2955242247147529),
    (0.4333953941292472,0.2692667193099963),(0.6794095682990244,0.2190863625159820),
    (0.8650633666889845,0.1494513491505806),(0.9739065285171717,0.0666713443086881),
];

/// `∫_a^b f` by ten-point Gauss–Legendre on `pieces` equal pieces.
fn gauss(a: f64,b: f64,pieces: usize,f: &mut dyn FnMut(f64) -> f64) -> f64 {
    let h = (b-a)/pieces as f64;
    let mut sum = 0.;
    for p in 0..pieces {
        let mid = a+h*(p as f64+0.5);
        for &(x,w) in &GAUSS { sum += w*f(mid+x*h/2.); }
    }
    sum*h/2.
}

/// Pieces for an angle-like span, a sixteenth of a turn each.
fn pieces(span: f64,periodic: bool) -> usize {
    if periodic { ((span.abs()/(std::f64::consts::PI/8.)).ceil() as usize).max(1) } else { 4 }
}

/// `n`-point Gauss–Legendre nodes and weights on [−1, 1], by Newton on the Legendre polynomial.
fn rule(n: usize) -> Vec<(f64,f64)> {
    (0..n).map(|i| {
        let mut x = (std::f64::consts::PI*(i as f64+0.75)/(n as f64+0.5)).dcos();
        let mut dp = 1.;
        for _ in 0..100 {
            let (mut p0,mut p1) = (1.,x);
            for k in 2..=n { let p2 = ((2*k-1) as f64*x*p1-(k-1) as f64*p0)/k as f64; p0 = p1; p1 = p2; }
            dp = n as f64*(x*p1-p0)/(x*x-1.);
            let step = p1/dp;
            x -= step;
            if step.abs() < 1e-16 { break }
        }
        (x,2./((1.-x*x)*dp*dp))
    }).collect()
}

/// `G(u, v) = ∫_{u₀}^u g(σ, v) dσ` on a polynomial B-spline face, from the domain's start rather
/// than 0 (which changes `G` by a function of `v` alone, and `∮ F(v) dv` round a closed loop is 0).
/// `g` is a polynomial of degree `3du − 1` in `u` and `3dv − 1` in `v` on each patch, so a Gauss
/// rule of `⌈3du/2⌉` points is exact on a `u` span, and `G` at a `u` knot is exactly the polynomial
/// through `3dv` points of each `v` span: those are tabled once a `v` span is first asked for, and
/// `G` anywhere is a table read plus one span's rule.
struct Prefix<'a> {
    net: &'a Net,
    /// The distinct `u` knots across the domain, and the `v` ones.
    us: Vec<f64>,
    vs: Vec<f64>,
    ru: Vec<(f64,f64)>,
    /// Chebyshev points of the second kind on [−1, 1], `3dv` of them.
    nodes: Vec<f64>,
    /// Per `v` span, once asked for: per node, `G` at each `u` knot.
    cols: Vec<Option<Vec<Vec<f64>>>>,
}

impl<'a> Prefix<'a> {
    fn new(net: &'a Net) -> Prefix<'a> {
        let [du,dv] = net.domain();
        let knots = |k: usize,[a,b]: [f64;2]| { let mut x = vec![a]; x.extend(net.breaks(k,[a,b])); x.push(b); x };
        let (us,vs) = (knots(0,du),knots(1,dv));
        let m = 3*net.dv;
        let nodes = (0..m).map(|k| (std::f64::consts::PI*k as f64/(m-1) as f64).dcos()).collect();
        let cols = vec![None;vs.len()-1];
        Prefix {net,us,vs,ru:rule((3*net.du).div_ceil(2)+1),nodes,cols}
    }
    fn g(&self,u: f64,v: f64) -> f64 { let (x,su,sv) = self.net.d1(u,v); dot(x,cross(su,sv)) }
    fn across(&self,a: f64,b: f64,v: f64) -> f64 {
        let (mid,h) = ((a+b)/2.,(b-a)/2.);
        h*self.ru.iter().map(|&(x,w)| w*self.g(mid+x*h,v)).sum::<f64>()
    }
    /// The span of `knots` holding `t`, the end ones taking what lies past them.
    fn span(knots: &[f64],t: f64) -> usize { knots.partition_point(|&k| k <= t).clamp(1,knots.len()-1)-1 }
    fn big_g(&mut self,u: f64,v: f64) -> f64 {
        let (i,j) = (Self::span(&self.us,u),Self::span(&self.vs,v));
        let (v0,v1) = (self.vs[j],self.vs[j+1]);
        let at = |x: f64| (v0+v1)/2.+x*(v1-v0)/2.;
        if self.cols[j].is_none() {
            let col = self.nodes.iter().map(|&x| {
                let mut acc = vec![0.];
                for w in self.us.windows(2) { acc.push(acc.last().unwrap()+self.across(w[0],w[1],at(x))); }
                acc
            }).collect();
            self.cols[j] = Some(col);
        }
        // the polynomial through the nodes at `v` (barycentric, the Chebyshev weights)
        let col = self.cols[j].as_ref().unwrap();
        let x = (2.*v-v0-v1)/(v1-v0);
        let m = self.nodes.len();
        let (mut num,mut den) = (0.,0.);
        let mut exact = None;
        for (k,&xk) in self.nodes.iter().enumerate() {
            if x == xk { exact = Some(col[k][i]); break }
            let w = if k%2 == 0 { 1. } else { -1. }*if k == 0 || k == m-1 { 0.5 } else { 1. }/(x-xk);
            num += w*col[k][i];
            den += w;
        }
        exact.unwrap_or(num/den)+self.across(self.us[i],u,v)
    }
}

/// The volume the boundary encloses (negative if it is turned inside out).
pub fn volume(b: &Brep) -> f64 { fluxes(b).iter().sum::<f64>()/3. }

/// Each face's flux `∬ X·n dA` out of the solid, side by side.
pub fn fluxes(b: &Brep) -> Vec<f64> {
    crate::par::indices(b.faces.len(),|fi| terms(b,fi).iter().flatten().map(|t| t.0+t.1).sum())
}

/// A face's flux term by term: for each use of each loop, its line integral and the closing
/// segment from its end to the next use's start (what a reading of the face can be compared by).
pub fn terms(b: &Brep,fi: usize) -> Vec<Vec<(f64,f64)>> {
    {
        let f = &b.faces[fi];
        let mut out: Vec<Vec<(f64,f64)>> = Vec::new();
        let s = &f.surface;
        let [pu,pv] = s.periods();
        let g = |u: f64,v: f64| { let (x,su,sv) = s.d1([u,v]); dot(x,cross(su,sv)) };
        // a rational net's `g` is a quotient, which no Gauss rule integrates exactly: it takes the
        // stretch-by-stretch rule a swept face does
        let mut prefix = match s { Surface::BSpline(_,n) if !n.is_rational() => Some(Prefix::new(n)),_ => None };
        // The face's box in its parameters. Away from a spline's table, `g` is integrated from the
        // middle of the face's range and across the parameter it spans less: `G` is then as small
        // as the face allows, and an error δ in a use's parameters moves the flux by `G δ` — from 0
        // on a sphere ring turned 9 radians round, `G` is π R³ times the turn, and a pcurve 1e-8
        // off in v moved a gear's flux by 1e-5 of itself
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for c in f.loops.iter().flatten() {
            let e = &b.edges[c.edge as usize];
            for j in 0..=8 {
                let q = c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/8.,e,s,&b.vertices);
                for k in 0..2 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); }
            }
        }
        // the parameter `g` is integrated along (`G` then multiplies the other's derivative)
        let along = if prefix.is_some() || !(hi[1]-lo[1] < hi[0]-lo[0]) { 0 } else { 1 };
        let base = if prefix.is_some() || !lo[along].is_finite() { 0. } else { (lo[along]+hi[along])/2. };
        let periodic = [pu,pv][along].is_some();
        // ∫ from the base, a stretch at a time between the surface's breaks along it
        let mut big_g = |u: f64,v: f64| {
            if let Some(p) = prefix.as_mut() { return p.big_g(u,v) }
            let x = [u,v][along];
            let mut at_x = |w: f64| if along == 0 { g(w,v) } else { g(u,w) };
            let cuts = s.breaks(along,[base,x]);
            if cuts.is_empty() { return gauss(base,x,pieces(x-base,periodic),&mut at_x) }
            let mut at = vec![base];
            if x >= base { at.extend(cuts) } else { at.extend(cuts.into_iter().rev()) }
            at.push(x);
            at.windows(2).map(|w| gauss(w[0],w[1],2,&mut at_x)).sum::<f64>()
        };
        // ∬ g du dv = ∮ G dv with G along u, or -∮ G du with G along v
        let (other,sign) = if along == 0 { (1,1.) } else { (0,-1.) };
        for l in &f.loops {
            let mut row: Vec<(f64,f64)> = Vec::new();
            for c in l {
                let e = &b.edges[c.edge as usize];
                // a pole's degenerate edge runs along u alone: nothing in dv, but its du counts
                if matches!(e.curve,EdgeCurve::Degenerate) && (other == 1 || !matches!(c.pcurve,Pcurve::Line {..})) {
                    row.push((0.,0.)); continue
                }
                let span = e.t[1]-e.t[0];
                let periodic = pu.is_some() || pv.is_some() || matches!(&e.curve,EdgeCurve::Curve(c) if c.period().is_some());
                let mut integrand = |t: f64| {
                    let dv = c.pcurve.derivative(t,e,s,&b.vertices)[other];
                    if dv == 0. { return 0. }
                    let [u,v] = c.pcurve.at(t,e,s,&b.vertices);
                    sign*big_g(u,v)*dv
                };
                // where the edge's curve or its curve in the face's parameters may jump in a derivative
                let mut breaks = if let EdgeCurve::Curve(c) = &e.curve { c.breaks(e.t) } else { vec![] };
                if let Pcurve::Curve(p) = &c.pcurve { breaks.extend(p.breaks(e.t)); }
                breaks.sort_by(f64::total_cmp);
                breaks.dedup();
                let line = if !breaks.is_empty() {
                    // smooth between its knots or points: integrated a stretch at a time
                    let mut cuts = vec![e.t[0]];
                    cuts.extend(breaks);
                    cuts.push(e.t[1]);
                    let each = if let EdgeCurve::Curve(Curve::Traced(_)) = &e.curve { 1 } else { 2 };
                    cuts.windows(2).map(|w| gauss(w[0],w[1],each,&mut integrand)).sum()
                } else { gauss(e.t[0],e.t[1],pieces(span,periodic),&mut integrand) };
                row.push((if c.reversed { -line } else { line },0.));
            }
            // the loop closed in the face's parameters, each use's end joined straight to the next's
            // start: the kernel's measured gaps between them are small in space, but `G` across a
            // spline face's chart can be large, and Green's theorem needs a closed curve
            let ends: Vec<(Uv,Uv)> = l.iter().map(|c| {
                let e = &b.edges[c.edge as usize];
                let (a,z) = (c.pcurve.at(e.t[0],e,s,&b.vertices),c.pcurve.at(e.t[1],e,s,&b.vertices));
                if c.reversed { (z,a) } else { (a,z) }
            }).collect();
            for k in 0..ends.len() {
                let (p,q) = (ends[k].1,ends[(k+1)%ends.len()].0);
                let dv = q[other]-p[other];
                if dv == 0. { continue }
                row[k].1 = gauss(0.,1.,1,&mut |x| sign*big_g(p[0]+x*(q[0]-p[0]),p[1]+x*(q[1]-p[1]))*dv);
            }
            out.push(row);
        }
        out
    }
}
