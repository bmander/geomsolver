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
use crate::util::fmath::Det;
use super::geom::{Curve,Surface,Uv,V};
use super::nurbs::Net;
use super::topo::{Brep,EdgeCurve,Face,Pcurve};
use crate::util::space::{cross,dot};

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

/// `gauss` over `pieces`, then twice as many, until two readings agree to rounding (or `SETTLE`
/// doublings): an edge's integrand runs smooth along it only as fast as its curve in the face's
/// parameters does, and a circle on a ball passing beside a pole sweeps half a turn of `u` within a
/// few degrees of its own (a ball at an end pose of a screw, its great circle square to the helix).
fn settled(a: f64,b: f64,pieces: usize,f: &mut dyn FnMut(f64) -> f64) -> f64 {
    settled_by(pieces,SETTLE,&mut |n| { let v = gauss(a,b,n,f); Some((v,v.abs())) }).expect("a reading")
}

/// `settled` of a reading that may fail, doubled `most` times at most: `read(n)` the integral on `n`
/// pieces and the size its rounding is measured against, two readings agreeing to 1e-13 of it;
/// `None` where a reading is.
fn settled_by(pieces: usize,most: usize,read: &mut dyn FnMut(usize) -> Option<(f64,f64)>) -> Option<f64> {
    let (mut n,mut last) = (pieces,read(pieces)?.0);
    for _ in 0..most {
        n *= 2;
        let (next,size) = read(n)?;
        // (the coarser reading kept where the finer agrees with it: a face read well at its first
        // pieces is read as it always was, to the bit)
        if (next-last).abs() <= 1e-13*size.max(1e-300) { return Some(last) }
        last = next;
    }
    Some(last)
}

/// How many times `settled` doubles an edge's pieces at most.
const SETTLE: usize = 6;

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

/// The stretch of `knots` (rising) holding `t`, the end ones taking what lies past them.
fn span(knots: &[f64],t: f64) -> usize { knots.partition_point(|&k| k <= t).clamp(1,knots.len()-1)-1 }

/// `G(u, v) = ∫_{u₀}^u g(σ, v) dσ` on a polynomial B-spline face, from a `u` knot `u₀` of the face's
/// own (`base`) rather than 0 (which changes `G` by a function of `v` alone, and `∮ F(v) dv` round a
/// closed loop is 0). `g` is a polynomial of degree `3du − 1` in `u` and `3dv − 1` in `v` on each
/// patch, so a Gauss rule of `⌈3du/2⌉` points is exact on a `u` span, and `G` at a `u` knot is exactly
/// the polynomial through `3dv` points of each `v` span: those are tabled as asked for, each `v`
/// span's column filled out from the base knot only as far as a `u` asked reaches — a face of a
/// fitted sheet reaches a part of its hundreds of spans, which were integrated across whole for
/// every column — and `G` anywhere is a table read plus one span's rule.
struct Prefix<'a> {
    net: &'a Net,
    /// The distinct `u` knots across the domain, and the `v` ones.
    us: Vec<f64>,
    vs: Vec<f64>,
    ru: Vec<(f64,f64)>,
    /// Chebyshev points of the second kind on [−1, 1], `3dv` of them.
    nodes: Vec<f64>,
    /// The `u` knot `G` is integrated from.
    base: usize,
    /// Per `v` span, once asked for: per node, the net's rows there (`Row`) and `G` at each `u`
    /// knot read so far (from `base`).
    cols: Vec<Option<Vec<(Vec<Row>,Vec<Option<f64>>)>>>,
}

/// A row of a net's poles read at one `v`: the point the row's poles make there, and its rate in
/// `v` — at that `v` the sheet and `S_v` are B-spline curves in `u` with these poles.
type Row = (V,V);

impl<'a> Prefix<'a> {
    /// The table of `net`, integrated from the `u` knot nearest `from`.
    fn new(net: &'a Net,from: f64) -> Prefix<'a> {
        let [du,dv] = net.domain();
        let knots = |k: usize,[a,b]: [f64;2]| { let mut x = vec![a]; x.extend(net.breaks(k,[a,b])); x.push(b); x };
        let (us,vs) = (knots(0,du),knots(1,dv));
        let m = 3*net.dv;
        let nodes = (0..m).map(|k| (std::f64::consts::PI*k as f64/(m-1) as f64).dcos()).collect();
        let cols = vec![None;vs.len()-1];
        let base = if from.is_finite() { (0..us.len()).min_by(|&a,&b| (us[a]-from).abs().total_cmp(&(us[b]-from).abs())).unwrap_or(0) } else { 0 };
        Prefix {net,us,vs,ru:rule((3*net.du).div_ceil(2)+1),nodes,base,cols}
    }
    fn g(&self,u: f64,v: f64) -> f64 { let (x,su,sv) = self.net.d1(u,v); dot(x,cross(su,sv)) }
    /// The net's rows read at `v` (`Row`).
    fn rows(&self,v: f64) -> Vec<Row> {
        let n = self.net;
        let ([v0,v1],nv) = (n.domain()[1],n.poles[0].len());
        super::nurbs::with_basis(n.dv,&n.vknots,nv,v.clamp(v0,v1),1,|sv,bv| n.poles.iter().map(|row| {
            let (mut x,mut d) = ([0.;3],[0.;3]);
            for j in 0..=n.dv { let q = row[sv-n.dv+j]; for c in 0..3 { x[c] += bv[0][j]*q[c]; d[c] += bv[1][j]*q[c]; } }
            (x,d)
        }).collect())
    }
    /// `across` at the `v` the rows were read at, by the curves in `u` they make: `g = C·(C' × D)`.
    fn across_rows(&self,a: f64,b: f64,rows: &[Row]) -> f64 {
        let n = self.net;
        let ([u0,u1],nu) = (n.domain()[0],n.poles.len());
        let (mid,h) = ((a+b)/2.,(b-a)/2.);
        h*self.ru.iter().map(|&(x,w)| super::nurbs::with_basis(n.du,&n.uknots,nu,(mid+x*h).clamp(u0,u1),1,|su,bu| {
            let (mut c,mut cu,mut d) = ([0.;3],[0.;3],[0.;3]);
            for i in 0..=n.du {
                let (q,r) = rows[su-n.du+i];
                for k in 0..3 { c[k] += bu[0][i]*q[k]; cu[k] += bu[1][i]*q[k]; d[k] += bu[0][i]*r[k]; }
            }
            w*dot(c,cross(cu,d))
        })).sum::<f64>()
    }
    fn across(&self,a: f64,b: f64,v: f64) -> f64 {
        let (mid,h) = ((a+b)/2.,(b-a)/2.);
        h*self.ru.iter().map(|&(x,w)| w*self.g(mid+x*h,v)).sum::<f64>()
    }
    fn big_g(&mut self,u: f64,v: f64) -> f64 {
        let (i,j) = (span(&self.us,u),span(&self.vs,v));
        let (v0,v1) = (self.vs[j],self.vs[j+1]);
        let at = |x: f64| (v0+v1)/2.+x*(v1-v0)/2.;
        let base = self.base;
        let n = self.us.len();
        let mut col = self.cols[j].take().unwrap_or_else(|| self.nodes.iter().map(|&x| {
            let mut c = vec![None;n];
            c[base] = Some(0.);
            (self.rows(at(x)),c)
        }).collect());
        // each node's column filled out from the base to knot `i`, a span at a time
        for (rows,c) in col.iter_mut() {
            if c[i].is_some() { continue }
            if i > base {
                let mut at_k = (base..=i).rev().find(|&m| c[m].is_some()).unwrap();
                while at_k < i { let next = c[at_k].unwrap()+self.across_rows(self.us[at_k],self.us[at_k+1],rows); at_k += 1; c[at_k] = Some(next); }
            } else {
                let mut at_k = (i..=base).find(|&m| c[m].is_some()).unwrap();
                while at_k > i { let next = c[at_k].unwrap()-self.across_rows(self.us[at_k-1],self.us[at_k],rows); at_k -= 1; c[at_k] = Some(next); }
            }
        }
        // the polynomial through the nodes at `v` (barycentric, the Chebyshev weights)
        let x = (2.*v-v0-v1)/(v1-v0);
        let m = self.nodes.len();
        let (mut num,mut den) = (0.,0.);
        let mut exact = None;
        for (k,&xk) in self.nodes.iter().enumerate() {
            let gk = col[k].1[i].unwrap();
            if x == xk { exact = Some(gk); break }
            let w = if k%2 == 0 { 1. } else { -1. }*if k == 0 || k == m-1 { 0.5 } else { 1. }/(x-xk);
            num += w*gk;
            den += w;
        }
        self.cols[j] = Some(col);
        exact.unwrap_or(num/den)+self.across(self.us[i],u,v)
    }
}

/// How far a span of a traced curve's points may turn, chord against chord: each of its chords within
/// this angle (its cosine) of its first, so within twice it of the span's own chord.
const SPAN_COS: f64 = 0.9659258262890683;

/// **`∫ f(uv, uv') dt` along a traced edge inverted onto `surface`** (#798), `f` given the edge's
/// parameters on the face and their rate. Green's integral does not depend on how its curve is
/// parameterised, only on the curve, so it is read not by the edge's own parameter — which counts
/// the trace's points, its rate stepping at every one, so that each stretch between two was a rule
/// of its own, each node of it three projections onto both surfaces (a canal's meeting with an end
/// disc, a thousand stretches, measured in seconds) — but over spans of the points turning a little
/// (`SPAN_COS`), each by its distance `σ` along its chord `n`: the curve's point at each node is
/// where it crosses the plane `n·x = σ` (`Traced::across`), and its rate `T / (T·n)` read there,
/// both as exact as the trace, so each span is one smooth integrand. It is read by Gauss–Legendre on
/// cells of its stretches, doubled until two readings agree to rounding or each cell is one stretch:
/// the march sets its points closer where the curve turns faster, and nodes spread evenly along the
/// chord read a turn at a span's end too coarsely to settle (the tip of the sliver a plane through
/// its axis cuts from a screwed rim's ramp, 2e-5 of the span). A span the curve does not cross as
/// a graph over its chord (its
/// rate turned past 60° from it, or a crossing found off the stretch it was sought in) is read by
/// the edge's parameter, stretch by stretch, as before.
fn along_trace(tr: &super::geom::Traced,e: &super::topo::Edge,[ua,ub]: [Uv;2],s: &Surface,vertices: &[super::topo::Vertex],
    f: &mut dyn FnMut(Uv,Uv) -> f64) -> f64 {
    use crate::util::space::{norm,normalised,scale,sub};
    let [t0,t1] = e.t;
    // the edge's ends and the trace's points between (its breaks), each with its parameter
    let breaks = if let EdgeCurve::Curve(c) = &e.curve { c.breaks(e.t) } else { vec![] };
    let at: Vec<(f64,[f64;3])> = std::iter::once((t0,tr.at(t0))).chain(breaks.into_iter().map(|k| (k,tr.guess(k))))
        .chain(std::iter::once((t1,tr.at(t1)))).collect();
    // by the edge's parameter, stretch by stretch: `at[i..=j]`
    let pc = Pcurve::Inverse {a:ua,b:ub};
    let by_parameter = |i: usize,j: usize,f: &mut dyn FnMut(Uv,Uv) -> f64| -> f64 {
        let mut integrand = |t: f64| { let (uv,d) = pc.at_and_derivative(t,e,s,vertices); f(uv,d) };
        at[i..=j].windows(2).map(|w| gauss(w[0].0,w[1].0,1,&mut integrand)).sum::<f64>()
    };
    let chord = |i: usize| sub(at[i+1].1,at[i].1);
    let mut total = 0.;
    let mut i = 0;
    while i+1 < at.len() {
        // the span: chords within `SPAN_COS` of its first (one chord where that has no length, and
        // nothing to read)
        let mut j = i+1;
        if let Some(d) = normalised(chord(i)) {
            while j+1 < at.len() && { let c = chord(j); dot(c,d) >= SPAN_COS*norm(c) } { j += 1 }
        }
        let n = sub(at[j].1,at[i].1);
        let len = norm(n);
        if len > 0. { total += across_span(tr,&at[i..=j],scale(n,1./len),len,e,(ua,ub),s,f).unwrap_or_else(|| by_parameter(i,j,f)) }
        i = j;
    }
    total
}

/// `∫ f` over one span of a traced edge's points, `at` (each with its parameter), by its distance
/// along its chord `n` of length `len`; `None` where a node is not crossed as a graph over the chord.
#[allow(clippy::too_many_arguments)]
fn across_span(tr: &super::geom::Traced,at: &[(f64,[f64;3])],n: [f64;3],len: f64,e: &super::topo::Edge,(ua,ub): (Uv,Uv),
    s: &Surface,f: &mut dyn FnMut(Uv,Uv) -> f64) -> Option<f64> {
    use crate::util::space::{distance,norm,sub};
    // each point's distance along the chord, rising from 0 to `len`
    let sig: Vec<f64> = at.iter().map(|&(_,x)| dot(n,sub(x,at[0].1))).collect();
    let h0 = dot(n,at[0].1);
    // `∫ f` over the span on `cells` cells, each a run of its stretches as near alike in number as
    // they go, and the size of its rounding: the mean size of `f` at its nodes over the span's length
    let stretches = at.len()-1;
    let mut read = |cells: usize| -> Option<(f64,f64)> {
        let (mut size,mut failed) = (0.,false);
        let mut integrand = |x: f64| {
            if failed { return 0. }
            // the stretch holding `x`, and the parabola's point there to start from
            let a = span(&sig,x);
            let w = sig[a+1]-sig[a];
            let fr = if w > 0. { ((x-sig[a])/w).clamp(0.,1.) } else { 0. };
            let t = at[a].0+fr*(at[a+1].0-at[a].0);
            let q = tr.guess(t);
            let Some((p,rate)) = tr.across(q,n,h0+x).filter(|&(p,rate)| norm(rate) <= 2. && distance(p,q) <= distance(at[a].1,at[a+1].1))
                else { failed = true; return 0. };
            let uv = super::topo::inverse_at(ua,ub,t,p,e,s);
            let y = f(uv,super::topo::in_parameters(s,uv,rate));
            size += y.abs();
            y
        };
        let cells = cells.min(stretches);
        let ends: Vec<usize> = (0..=cells).map(|k| k*stretches/cells).collect();
        let v = ends.windows(2).map(|w| gauss(sig[w[0]],sig[w[1]],1,&mut integrand)).sum::<f64>();
        (!failed).then_some((v,size*len/(GAUSS.len()*cells) as f64))
    };
    // the cells doubled until two readings agree, or each is one stretch
    settled_by(1,stretches.next_power_of_two().trailing_zeros() as usize,&mut read)
}

/// The volume the boundary encloses (negative if it is turned inside out).
pub fn volume(b: &Brep) -> f64 { fluxes(b).iter().sum::<f64>()/3. }

/// Each face's flux `∬ X·n dA` out of the solid, side by side.
pub fn fluxes(b: &Brep) -> Vec<f64> {
    crate::util::par::indices(b.faces.len(),|fi| terms(b,fi).iter().flatten().map(|t| t.0+t.1).sum())
}

/// A face's flux term by term: for each use of each loop, its line integral and the closing
/// segment from its end to the next use's start (what a reading of the face can be compared by).
pub fn terms(b: &Brep,fi: usize) -> Vec<Vec<(f64,f64)>> { integrated(b,&b.faces[fi],false) }

/// A face's area, `∬ |S_u × S_v| du dv` over its domain, read off its loops as its flux is — so a
/// face on another surface over the same boundary can be held to it (`family::lower`). The face
/// need not be one of `b`'s, only its edges.
pub fn area(b: &Brep,f: &Face) -> f64 {
    integrated(b,f,true).iter().flatten().map(|t| t.0+t.1).sum::<f64>().abs()
}

/// `terms`, of the flux or (`area`) of the area.
fn integrated(b: &Brep,f: &Face,area: bool) -> Vec<Vec<(f64,f64)>> {
    {
        let mut out: Vec<Vec<(f64,f64)>> = Vec::new();
        let s = &f.surface;
        let [pu,pv] = s.periods();
        let g = |u: f64,v: f64| { let (x,su,sv) = s.d1([u,v]); if area { crate::util::space::norm(cross(su,sv)) } else { dot(x,cross(su,sv)) } };
        // a rational net's `g` is a quotient, which no Gauss rule integrates exactly: it takes the
        // stretch-by-stretch rule a swept face does
        let mut prefix: Option<Prefix> = None;
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
        // a polynomial net's `G` tabled from the face's middle `u` (`Prefix`)
        if let Surface::BSpline(_,n) = s { if !n.is_rational() && !area { prefix = Some(Prefix::new(n,(lo[0]+hi[0])/2.)) } }
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
                // `G dv` at a point of the edge on the face, given its parameters and their rate
                let mut value = |[u,v]: Uv,d: Uv| { let dv = d[other]; if dv == 0. { 0. } else { sign*big_g(u,v)*dv } };
                // a traced curve inverted onto the face: read across planes, a span of its points
                // at a time (#798)
                if let (EdgeCurve::Curve(Curve::Traced(tr)),&Pcurve::Inverse {a,b:z}) = (&e.curve,&c.pcurve) {
                    let line = along_trace(tr,e,[a,z],s,&b.vertices,&mut value);
                    row.push((if c.reversed { -line } else { line },0.));
                    continue
                }
                let span = e.t[1]-e.t[0];
                let periodic = pu.is_some() || pv.is_some() || matches!(&e.curve,EdgeCurve::Curve(c) if c.period().is_some());
                // each node's point sought from the last node's, carried along by its rate, where
                // that node is just behind it (the nodes of a reading run along the edge in order)
                let mut last: Option<(f64,Uv,Uv)> = None;
                let mut integrand = |t: f64| {
                    let seed = last.filter(|&(t0,_,_)| t >= t0 && t-t0 <= span.abs()/8.)
                        .map(|(t0,uv,d)| [uv[0]+d[0]*(t-t0),uv[1]+d[1]*(t-t0)]);
                    let (uv,d) = c.pcurve.at_and_derivative_from(t,e,s,&b.vertices,seed);
                    last = Some((t,uv,d));
                    value(uv,d)
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
                } else { settled(e.t[0],e.t[1],pieces(span,periodic),&mut integrand) };
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
