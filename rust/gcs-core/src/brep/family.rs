//! **A face family's walls, exact** (#243 L2): the surface `S(u, s)` an edge of a running section
//! sweeps — the station solved at `s` (`solid::family`), then its edge read at the fraction `u` of
//! its own run — and the solid those walls bound with the two end sections (`family`).
//!
//! The station is solved at the **nodes** of each piece of the interval (`Stations`: six
//! Gauss–Lobatto points a piece, each piece checked against a solve between them before it is
//! kept), and between them each of the edge's numbers is the degree-five polynomial through its
//! six solved values. So `S` is the edge's own form in `u` — a line, an arc by angle, a B-spline by
//! its parameter, never differenced — and a polynomial in `s` whose derivatives are its own. Each
//! number at `s` is a dot product with six weights read once for that `s` (`Stations::weights`),
//! so the station is never solved again where the surface is read.
//!
//! An edge is written as **numbers** a placement moves: its points, then its directions, then its
//! plain numbers (`Run::shape`) — a line its two ends; an arc its centre, the radius vectors to its
//! start and a quarter turn on, and its sweep (`C + cos(uΔ) R₀ + sin(uΔ) R₉₀`, which stays
//! continuous where an angle would wrap); a B-spline its poles, then the span of its parameter.
//!
//! **The solid** (`family`): each wall on its sheet, its parameters `(u, s)` over `[0, 1] × [a, b]`
//! (a whole circle's periodic in `u`); its rims the section's edges at the ends, the rails between
//! walls the sheets' `u = 0` lines; the caps `start` and `end` planes through the end sections.
//! An edge whose run has vanished at an end is a degenerate edge there (its rails meet in one
//! vertex), and a section vanished whole is an apex with no cap. Where the walls meet at an angle
//! across the interval (`creases`, #243 L7: a ruled loft's sections) the solid is cut there into
//! bands, each wall a face a band, one name over all, and each band's rims an edge of it. A wall
//! whose sheet is a plane, a
//! cylinder, a cone or a torus (`analytic`) lies on that surface instead — never by its word, by
//! what its numbers do — once its points lie on it and its area agrees with the sheet's; and where
//! the whole section moves rigidly, by one screw or one straight step (`carried`, #243 L6), every
//! other wall lies on the surface its first rim sweeps under that motion (`M(s)·C(u)`, a `Blend`
//! of the rim with itself), held alike.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{hull,include,BSpline,Blend,Box3,Carry,Curve,Frame,Rigid,Surface,Uv,V,EMPTY};
use super::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::space::{add,cross,distance,dot,norm,scale,sub,unit};
use std::sync::Arc;

/// The six Gauss–Lobatto points of degree five on `[−1, 1]`: where a piece's stations are solved.
pub const NODES: [f64;6] = [-1.,-0.7650553239294647,-0.2852315164806451,0.2852315164806451,0.7650553239294647,1.];

/// What an edge of the section is, so how its numbers make it.
#[derive(Clone,Debug,PartialEq)]
pub enum Run {
    /// `A + u (B − A)`: its two ends.
    Line,
    /// `C + cos(uΔ) R₀ + sin(uΔ) R₉₀`: its centre, its radius vectors to its start and a quarter
    /// turn on, and its sweep `Δ` (radians, signed the way it is walked).
    Round,
    /// A B-spline of fixed degree, knots and weights: its poles, then the span `[t₀, t₁]` of its
    /// parameter it runs over (either way round).
    Spline { degree: usize,knots: Vec<f64>,weights: Option<Vec<f64>> },
}

impl Run {
    /// How many of an edge's `len` numbers are points, directions and plain numbers, in that order.
    pub fn shape(&self,len: usize) -> (usize,usize,usize) {
        match self { Run::Line => (2,0,0),Run::Round => (1,2,1),Run::Spline {..} => ((len-2)/3,0,2) }
    }
    /// The kind, as a recipe spells it.
    pub fn kind(&self) -> &'static str { match self { Run::Line => "line",Run::Round => "round",Run::Spline {..} => "spline" } }
    /// A B-spline edge's curve with poles read off `n` (its own, or their rates: the curve is
    /// linear in its poles where its weights stand still).
    fn spline(&self,n: &[f64]) -> BSpline {
        let Run::Spline {degree,knots,weights} = self else { unreachable!("a spline's") };
        let poles = (0..(n.len()-2)/3).map(|i| v3(n,3*i)).collect();
        BSpline {degree:*degree,knots:knots.clone(),poles,weights:weights.clone()}
    }
    /// The edge with numbers `n` at the fraction `u` of its run: the point, `∂/∂u` and `∂²/∂u²`.
    pub fn at(&self,n: &[f64],u: f64) -> (V,V,V) {
        match self {
            Run::Line => { let (a,b) = (v3(n,0),v3(n,3)); let d = sub(b,a); (add(a,scale(d,u)),d,[0.;3]) }
            Run::Round => {
                let (c,r0,r9,sweep) = (v3(n,0),v3(n,3),v3(n,6),n[9]);
                let (sn,cs) = (u*sweep).dsin_cos();
                let radial = add(scale(r0,cs),scale(r9,sn));
                let along = add(scale(r0,-sn),scale(r9,cs));
                (add(c,radial),scale(along,sweep),scale(radial,-sweep*sweep))
            }
            Run::Spline {..} => {
                let (t0,t1) = (n[n.len()-2],n[n.len()-1]);
                let (x,d,dd) = self.spline(n).d2(t0+u*(t1-t0));
                (x,scale(d,t1-t0),scale(dd,(t1-t0)*(t1-t0)))
            }
        }
    }
}

/// The pieces of a family's interval and, per piece, where its six stations stand: the nodes, at
/// the piece's Gauss–Lobatto points but where an end is read as its limit (the stations stop short
/// of it, and the polynomial through them carries on to it).
#[derive(Clone,Debug,PartialEq)]
pub struct Stations {
    pub spans: Vec<[f64;2]>,
    pub at: Vec<[f64;6]>,
    /// Per piece, the inverse of its nodes' Vandermonde matrix in the piece's own variable
    /// `t = (s − middle) / half` (row `k` the coefficient of `tᵏ`).
    inv: Vec<[[f64;6];6]>,
}

impl Stations {
    pub fn new(spans: Vec<[f64;2]>,at: Vec<[f64;6]>) -> Result<Stations,String> {
        if spans.is_empty() || spans.len() != at.len() { return Err("a face family has no pieces".into()) }
        if spans.windows(2).any(|w| w[0][1] != w[1][0]) || spans.iter().any(|s| !(s[1] > s[0])) {
            return Err("a face family's pieces do not run on from each other".into())
        }
        let mut inv = Vec::with_capacity(spans.len());
        for (sp,nodes) in spans.iter().zip(&at) {
            let (mid,half) = ((sp[0]+sp[1])/2.,(sp[1]-sp[0])/2.);
            let t: [f64;6] = nodes.map(|s| (s-mid)/half);
            inv.push(invert(&std::array::from_fn(|i| std::array::from_fn(|k| t[i].dpowi(k as i32))))
                .ok_or("a face family's stations stand on one another")?);
        }
        Ok(Stations {spans,at,inv})
    }
    /// The interval.
    pub fn span(&self) -> [f64;2] { [self.spans[0][0],self.spans[self.spans.len()-1][1]] }
    /// The piece holding `s`, the end ones taking what lies past them.
    fn piece(&self,s: f64) -> usize { self.spans.partition_point(|sp| sp[1] < s).min(self.spans.len()-1) }
    /// The weights of the six nodes of piece `j` that read a number and its rate at `s` (the
    /// polynomial through them, and its derivative in `s`).
    fn weights(&self,j: usize,s: f64) -> [[f64;6];2] {
        let sp = self.spans[j];
        let (mid,half) = ((sp[0]+sp[1])/2.,(sp[1]-sp[0])/2.);
        let t = (s-mid)/half;
        // tᵏ and its derivative in `s`, k t^(k−1) / half
        let mut mono = [[0.;6];2];
        let mut power = 1.;
        for k in 0..6 {
            mono[0][k] = power;
            if k+1 < 6 { mono[1][k+1] = (k+1) as f64*power/half; }
            power *= t;
        }
        std::array::from_fn(|d| std::array::from_fn(|i| (0..6).map(|k| mono[d][k]*self.inv[j][k][i]).sum()))
    }
    /// The stations of the pieces `js` alone: a band of the interval (`family`'s, at a crease).
    fn band(&self,js: std::ops::Range<usize>) -> Stations {
        let (spans,at) = (self.spans[js.clone()].to_vec(),self.at[js.clone()].to_vec());
        Stations {spans,at,inv:self.inv[js].to_vec()}
    }
    /// Where the surface stops being a polynomial in `s`, strictly inside `[a, b]`: the pieces' joints.
    pub fn breaks(&self,[a,b]: [f64;2]) -> Vec<f64> {
        let (a,b) = (a.min(b),a.max(b));
        self.spans[1..].iter().map(|sp| sp[0]).filter(|&k| k > a && k < b).collect()
    }
}

/// The inverse of a 6 × 6 matrix by Gauss–Jordan with partial pivoting, or none where it is singular.
fn invert(m: &[[f64;6];6]) -> Option<[[f64;6];6]> {
    let mut a = *m;
    let mut out: [[f64;6];6] = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1. } else { 0. }));
    for c in 0..6 {
        let p = (c..6).max_by(|&x,&y| a[x][c].abs().total_cmp(&a[y][c].abs()))?;
        if !(a[p][c].abs() > 1e-14) { return None }
        a.swap(c,p);
        out.swap(c,p);
        let d = a[c][c];
        for j in 0..6 { a[c][j] /= d; out[c][j] /= d; }
        for r in 0..6 {
            if r == c { continue }
            let f = a[r][c];
            if f == 0. { continue }
            for j in 0..6 { a[r][j] -= f*a[c][j]; out[r][j] -= f*out[c][j]; }
        }
    }
    Some(out)
}

/// **A wall of a face family**: one edge of the section as the station runs — its kind, the
/// pieces of the interval, and the edge's numbers at each node (piece by piece, six a piece).
#[derive(Clone,Debug,PartialEq)]
pub struct Sheet {
    pub run: Run,pub stations: Arc<Stations>,pub nodes: Vec<Vec<f64>>,pub closed: bool,
    /// Per piece, a box the wall keeps within over it (`bounds`): where `inverse` looks first.
    boxes: Vec<Box3>,
}

thread_local! {
    /// The last foot each thread found on each of the last few walls (by their addresses), as a
    /// B-spline sheet's (`geom::net_inverse`): only ever a start, held to settling near the point.
    static LAST_FOOT: std::cell::Cell<[(usize,Uv);4]> = const { std::cell::Cell::new([(0,[0.;2]);4]) };
}

/// Three numbers of `n` from `i`.
fn v3(n: &[f64],i: usize) -> V { [n[i],n[i+1],n[i+2]] }

impl Sheet {
    pub fn new(run: Run,stations: Arc<Stations>,nodes: Vec<Vec<f64>>,closed: bool) -> Result<Sheet,String> {
        let len = nodes.first().map_or(0,Vec::len);
        let ok = match &run {
            Run::Line => len == 6,
            Run::Round => len == 10,
            Run::Spline {degree,knots,weights} => len >= 8 && (len-2)%3 == 0 && knots.len() == (len-2)/3+degree+1
                && weights.as_ref().is_none_or(|w| w.len() == (len-2)/3 && w.iter().all(|&w| w.is_finite() && w > 0.)),
        };
        if !ok || nodes.len() != 6*stations.spans.len() || nodes.iter().any(|n| n.len() != len || n.iter().any(|x| !x.is_finite())) {
            return Err(format!("a face family's {} wall has numbers of the wrong count",run.kind()))
        }
        Ok(Sheet {run,stations,nodes,closed,boxes:Vec::new()}.boxed())
    }
    /// The sheet with its pieces' boxes read off its numbers.
    fn boxed(mut self) -> Sheet {
        self.boxes = self.stations.spans.iter().map(|&sp| self.bounds(sp)).collect();
        self
    }
    /// Whole-circle walls run round once: `u` periodic with period 1.
    pub fn period(&self) -> Option<f64> { self.closed.then_some(1.) }
    /// The interval the wall runs over.
    pub fn span(&self) -> [f64;2] { self.stations.span() }
    /// The edge's numbers at `s`, and (where `rates`) how fast they run there.
    fn read(&self,s: f64,rates: bool) -> [Vec<f64>;2] {
        let j = self.stations.piece(s);
        let w = self.stations.weights(j,s);
        let nodes = &self.nodes[6*j..6*j+6];
        let len = nodes[0].len();
        std::array::from_fn(|d| if d == 1 && !rates { Vec::new() } else {
            (0..len).map(|q| (0..6).map(|i| w[d][i]*nodes[i][q]).sum()).collect()
        })
    }
    /// The edge's numbers at `s`.
    pub fn numbers(&self,s: f64) -> Vec<f64> { let [n,_] = self.read(s,false); n }
    /// The edge with numbers `n` at the fraction `u` of its run: the point, `∂/∂u` and `∂²/∂u²`.
    pub fn at(&self,n: &[f64],u: f64) -> (V,V,V) { self.run.at(n,u) }
    /// `∂S/∂s` at `u` where the numbers are `n` and run at `dn`.
    fn rate(&self,n: &[f64],dn: &[f64],u: f64) -> V {
        match self.run {
            Run::Line => add(v3(dn,0),scale(sub(v3(dn,3),v3(dn,0)),u)),
            Run::Round => {
                let (r0,r9,sweep) = (v3(n,3),v3(n,6),n[9]);
                let (sn,cs) = (u*sweep).dsin_cos();
                let moved = add(v3(dn,0),add(scale(v3(dn,3),cs),scale(v3(dn,6),sn)));
                add(moved,scale(add(scale(r0,-sn),scale(r9,cs)),u*dn[9]))
            }
            Run::Spline {..} => {
                let k = n.len();
                let t = n[k-2]+u*(n[k-1]-n[k-2]);
                let dt = dn[k-2]+u*(dn[k-1]-dn[k-2]);
                add(self.run.spline(dn).point(t),scale(self.run.spline(n).d2(t).1,dt))
            }
        }
    }
    /// The wall over the pieces `js` alone, read over `stations` (`Stations::band`'s of them).
    fn band(&self,stations: Arc<Stations>,js: std::ops::Range<usize>) -> Sheet {
        let nodes = self.nodes[6*js.start..6*js.end].to_vec();
        Sheet {run:self.run.clone(),stations,nodes,closed:self.closed,boxes:Vec::new()}.boxed()
    }
    /// `∂S/∂s` at `(u, s)` as piece `j` reads it: at a joint, from either side of it.
    fn rate_in(&self,j: usize,[u,s]: Uv) -> V {
        let w = self.stations.weights(j,s);
        let nodes = &self.nodes[6*j..6*j+6];
        let [n,dn]: [Vec<f64>;2] = std::array::from_fn(|d| {
            (0..nodes[0].len()).map(|q| (0..6).map(|i| w[d][i]*nodes[i][q]).sum()).collect()
        });
        self.rate(&n,&dn,u)
    }
    /// `S`, `S_u`, `S_s`.
    pub fn d1(&self,[u,s]: Uv) -> (V,V,V) {
        let [n,dn] = self.read(s,true);
        let (x,xu,_) = self.at(&n,u);
        (x,xu,self.rate(&n,&dn,u))
    }
    /// A rail: `S`, `S_s` and `S_ss` along `s` at `u` — the second rate by a central difference of
    /// the first, exact enough for the curvature a sampler reads (as a loft's rail's is).
    pub fn rail(&self,u: f64,s: f64) -> (V,V,V) {
        let [a,b] = self.span();
        let h = 1e-5*(b-a);
        let (x,_,d) = self.d1([u,s]);
        let (lo,hi) = ((s-h).max(a),(s+h).min(b));
        (x,d,scale(sub(self.d1([u,hi]).2,self.d1([u,lo]).2),1./(hi-lo)))
    }
    /// A rim: the edge at station `s`, `C`, `C'` and `C''` in `u`.
    pub fn rim(&self,u: f64,s: f64) -> (V,V,V) { self.at(&self.numbers(s),u) }
    /// The point of the wall nearest `p`: Gauss–Newton from the last foot this thread found on it
    /// where that settles near `p` (a mesh asks of one corner after the next); else from the
    /// nearest of a grid — `u` in sixteenths, `s` at each piece's ends and quarters — read piece by
    /// piece nearest box first, none past the nearest point found (a piece keeps within its box),
    /// `s` held to the interval and `u` too unless the wall runs round.
    pub fn inverse(&self,p: V) -> Uv {
        let key = self as *const Sheet as usize;
        let near = 1e-3*(1.+dot(p,p).sqrt());
        let last = LAST_FOOT.with(|c| c.get());
        let warm = last.iter().find(|f| f.0 == key).map(|f| self.newton(p,f.1))
            .filter(|&uv| distance(self.d1(uv).0,p) <= near);
        let uv = warm.unwrap_or_else(|| {
            let reach = |b: &Box3| { let gap: V = std::array::from_fn(|i| (b.0[i]-p[i]).max(p[i]-b.1[i]).max(0.)); norm(gap)*(1.-1e-12) };
            let mut order: Vec<(f64,usize)> = self.boxes.iter().enumerate().map(|(j,b)| (reach(b),j)).collect();
            order.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
            let mut best = (f64::INFINITY,[0.;2]);
            for (bound,j) in order {
                if bound > best.0 { break }
                let sp = self.stations.spans[j];
                for k in 0..=4 {
                    let s = sp[0]+(sp[1]-sp[0])*k as f64/4.;
                    let n = &self.numbers(s);
                    for i in 0..=16 {
                        let u = i as f64/16.;
                        let d = distance(self.at(n,u).0,p);
                        if d < best.0 { best = (d,[u,s]); }
                    }
                }
            }
            self.newton(p,best.1)
        });
        let mut next = [(key,uv);4];
        let mut k = 1;
        for f in last { if f.0 != key && k < 4 { next[k] = f; k += 1; } }
        LAST_FOOT.with(|c| c.set(next));
        uv
    }
    /// Gauss–Newton for the parameters of the wall's point nearest `p`, from `uv`.
    pub fn newton(&self,p: V,mut uv: Uv) -> Uv {
        let [a,b] = self.span();
        for _ in 0..30 {
            let (x,su,ss) = self.d1(uv);
            let e = sub(x,p);
            let (a11,a12,a22) = (dot(su,su),dot(su,ss),dot(ss,ss));
            let (b1,b2) = (dot(su,e),dot(ss,e));
            let det = a11*a22-a12*a12;
            if !(det.abs() > 0.) { break }
            let (du,ds) = ((a22*b1-a12*b2)/det,(a11*b2-a12*b1)/det);
            let mut next = [uv[0]-du,(uv[1]-ds).clamp(a,b)];
            if !self.closed { next[0] = next[0].clamp(0.,1.); }
            let moved = (next[0]-uv[0]).abs().max((next[1]-uv[1]).abs()/(b-a));
            uv = next;
            if moved < 1e-15 { break }
        }
        if self.closed { uv[0] = uv[0].rem_euclid(1.); }
        uv
    }
    /// The parameter `s` of the rail at `u` nearest `p`.
    pub fn rail_inverse(&self,u: f64,p: V) -> f64 {
        let mut best = (f64::INFINITY,0.);
        for sp in &self.stations.spans {
            for j in 0..=4 {
                let s = sp[0]+(sp[1]-sp[0])*j as f64/4.;
                let d = distance(self.d1([u,s]).0,p);
                if d < best.0 { best = (d,s); }
            }
        }
        let [a,b] = self.span();
        let mut s = best.1;
        for _ in 0..20 {
            let (x,d,dd) = self.rail(u,s);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let next = (s-dot(e,d)/h).clamp(a,b);
            if (next-s).abs() <= 1e-15*(b-a) { s = next; break }
            s = next;
        }
        s
    }
    /// The fraction `u` of the rim at `s` nearest `p`.
    pub fn rim_inverse(&self,s: f64,p: V) -> f64 {
        let n = &self.numbers(s);
        let mut best = (f64::INFINITY,0.);
        for j in 0..=64 {
            let u = j as f64/64.;
            let d = distance(self.at(n,u).0,p);
            if d < best.0 { best = (d,u); }
        }
        let mut u = best.1;
        for _ in 0..20 {
            let (x,d,dd) = self.at(n,u);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let next = if self.closed { u-dot(e,d)/h } else { (u-dot(e,d)/h).clamp(0.,1.) };
            if (next-u).abs() <= 1e-15 { u = next; break }
            u = next;
        }
        if self.closed { u.rem_euclid(1.) } else { u }
    }
    /// Each number's range over `[s0, s1]`, never smaller: per piece reaching it, the polynomial's
    /// constant term widened by every other term at the farthest `t` the piece is read at.
    fn ranges(&self,[s0,s1]: [f64;2]) -> Vec<[f64;2]> {
        let (s0,s1) = (s0.min(s1),s0.max(s1));
        let len = self.nodes[0].len();
        let mut out = vec![[f64::INFINITY,f64::NEG_INFINITY];len];
        let st = &self.stations;
        for (j,sp) in st.spans.iter().enumerate() {
            let last = j+1 == st.spans.len();
            if (sp[1] < s0 && !last) || (sp[0] > s1 && j > 0) { continue }
            let (mid,half) = ((sp[0]+sp[1])/2.,(sp[1]-sp[0])/2.);
            let reach = [((s0.max(sp[0]).min(sp[1]))-mid)/half,((s1.min(sp[1]).max(sp[0]))-mid)/half]
                .iter().fold(1f64,|m,t| m.max(t.abs()));
            for q in 0..len {
                let c: [f64;6] = std::array::from_fn(|k| (0..6).map(|i| st.inv[j][k][i]*self.nodes[6*j+i][q]).sum());
                let spread: f64 = (1..6).map(|k| c[k].abs()*reach.dpowi(k as i32)).sum();
                out[q] = [out[q][0].min(c[0]-spread),out[q][1].max(c[0]+spread)];
            }
        }
        out
    }
    /// A box about the wall over `[s0, s1]`, never smaller (`ranges`): a line's ends' boxes, an
    /// arc's centre's widened by its radius vectors' reach, a B-spline's poles' (it keeps within
    /// their hull, its weights being positive).
    pub fn bounds(&self,span: [f64;2]) -> Box3 {
        let r = self.ranges(span);
        let lo = |i: usize| -> V { [r[i][0],r[i+1][0],r[i+2][0]] };
        let hi = |i: usize| -> V { [r[i][1],r[i+1][1],r[i+2][1]] };
        let mut b = EMPTY;
        match self.run {
            Run::Line => for i in [0,3] { include(&mut b,lo(i)); include(&mut b,hi(i)); },
            Run::Round => {
                let reach: V = std::array::from_fn(|k| (3..9).step_by(3).map(|i| r[i+k][0].abs().max(r[i+k][1].abs())).sum());
                b = (sub(lo(0),reach),add(hi(0),reach));
            }
            Run::Spline {..} => for i in (0..r.len()-2).step_by(3) { include(&mut b,lo(i)); include(&mut b,hi(i)); },
        }
        b
    }
    /// A box about the rim at `s` over `[u0, u1]`, never smaller.
    pub fn rim_bounds(&self,s: f64,[u0,u1]: [f64;2]) -> Box3 {
        let n = &self.numbers(s);
        match self.run {
            Run::Line => hull([u0,u1].map(|u| self.at(n,u).0)),
            Run::Round => {
                let c = v3(n,0);
                let reach: V = std::array::from_fn(|k| n[3+k].dhypot(n[6+k]));
                (sub(c,reach),add(c,reach))
            }
            Run::Spline {..} => hull(self.run.spline(n).poles),
        }
    }
    /// Where, strictly inside `span` of parameter `k` (0 for `u`, 1 for `s`), the wall stops being
    /// smooth: a B-spline edge's knots (at the fractions of its run the first station reads them
    /// at), and the pieces' joints.
    pub fn breaks(&self,k: usize,span: [f64;2]) -> Vec<f64> {
        if k == 1 { return self.stations.breaks(span) }
        let Run::Spline {..} = self.run else { return Vec::new() };
        let n = &self.nodes[0];
        let (t0,t1) = (n[n.len()-2],n[n.len()-1]);
        let (lo,hi) = (span[0].min(span[1]),span[0].max(span[1]));
        let mut out: Vec<f64> = self.run.spline(n).breaks([t0.min(t1),t0.max(t1)]).into_iter()
            .map(|t| (t-t0)/(t1-t0)).filter(|&u| u > lo && u < hi).collect();
        out.sort_by(f64::total_cmp);
        out
    }
    /// The length over which the wall turns appreciably: an arc's least radius, a B-spline's
    /// sixteenth of its poles' spread, and the least radius of curvature of its rails at the nodes.
    pub fn feature(&self) -> f64 {
        let mut f = f64::INFINITY;
        for n in &self.nodes {
            match self.run {
                Run::Line => {}
                Run::Round => f = f.min(norm(v3(n,3))),
                Run::Spline {..} => { let (lo,hi) = hull(self.run.spline(n).poles); f = f.min(distance(lo,hi)/16.) }
            }
        }
        for (j,sp) in self.stations.spans.iter().enumerate() {
            for &s in &self.stations.at[j] {
                let s = s.clamp(sp[0],sp[1]);
                for u in [0.,0.5,1.] {
                    let (_,d,dd) = self.rail(u,s);
                    let k = norm(cross(d,dd));
                    if k > 0. { f = f.min(norm(d).dpowi(3)/k); }
                }
            }
        }
        let [a,b] = self.span();
        let size = distance(self.d1([0.,a]).0,self.d1([1.,b]).0)+distance(self.d1([0.,b]).0,self.d1([1.,a]).0);
        f.min(size.max(f64::MIN_POSITIVE))
    }
    pub fn moved(&self,m: &Rigid) -> Sheet {
        let nodes = self.nodes.iter().map(|n| {
            let (pts,dirs,_) = self.run.shape(n.len());
            let mut out = n.clone();
            for i in 0..pts { let p = m.point(v3(n,3*i)); out[3*i..3*i+3].copy_from_slice(&p); }
            for i in pts..pts+dirs { let d = m.vector(v3(n,3*i)); out[3*i..3*i+3].copy_from_slice(&d); }
            out
        }).collect();
        Sheet {nodes,..self.clone()}.boxed()
    }
}

/// A round edge's numbers as a circle where they are one — its radius vectors as long as each other
/// and square, within `rel` of the radius — its centre, its axis (right-handed from the start
/// toward the quarter turn) and its radius.
fn circle(n: &[f64],rel: f64) -> Option<(V,V,f64)> {
    let (c,r0,r9) = (v3(n,0),v3(n,3),v3(n,6));
    let r = norm(r0);
    (r > 0. && (norm(r9)-r).abs() <= rel*r && dot(r0,r9).abs() <= rel*r*r).then(|| (c,unit(cross(r0,r9)),r))
}

/// A wall of a family as the solid is built from it: its sheet and its name.
#[derive(Clone,Debug)]
pub struct Wall { pub sheet: Arc<Sheet>,pub name: String }

/// The rim of `sheet` at `s` as an edge's curve and its parameters: a line or a circle where it is
/// one (so the caps and what meets them read it in closed form), else the sheet's own rim.
fn rim_curve(sheet: &Arc<Sheet>,s: f64) -> (Curve,[f64;2]) {
    let n = &sheet.numbers(s);
    match sheet.run {
        Run::Line => {
            let (a,b) = (v3(n,0),v3(n,3));
            (Curve::Line {p:a,d:unit(sub(b,a))},[0.,distance(a,b)])
        }
        Run::Round => match circle(n,1e-12) {
            // its frame turned the way it is walked, so its angle grows along it
            Some((c,z,r)) => (Curve::Circle(Frame::new(c,scale(z,n[9].signum()),v3(n,3)),r),[0.,n[9].abs()]),
            None => (Curve::Rim(sheet.clone(),s),[0.,1.]),
        },
        Run::Spline {..} => (Curve::Rim(sheet.clone(),s),[0.,1.]),
    }
}

/// The length of the rim at `s`, by its polygon of sixty-four chords.
fn rim_length(sheet: &Sheet,s: f64) -> f64 {
    let n = &sheet.numbers(s);
    (0..64).map(|j| distance(sheet.at(n,j as f64/64.).0,sheet.at(n,(j+1) as f64/64.).0)).sum()
}

/// The normal `Σ pᵢ × pᵢ₊₁` of a closed polygon (twice its vector area), which a walk runs
/// counter-clockwise about.
fn newell(pts: &[V]) -> V {
    let mut n = [0.;3];
    for i in 0..pts.len() { n = add(n,cross(pts[i],pts[(i+1)%pts.len()])); }
    n
}

/// A loop's points at station `s`, sixteen an edge.
fn loop_points(walls: &[Wall],s: f64) -> Vec<V> {
    walls.iter().flat_map(|w| { let n = w.sheet.numbers(s); (0..16).map(move |j| w.sheet.at(&n,j as f64/16.).0) })
        .collect()
}

/// A band of a family between its creases: its interval, and each loop's walls' sheets over it.
struct Band {
    span: [f64;2],
    walls: Vec<Vec<Arc<Sheet>>>,
}

/// How far apart the two sides of a joint may turn a wall's run along `s`, as `1 − cos`, before
/// the joint is a crease (about a twentieth of a degree): well above what the pieces' polynomials
/// miss a smooth run by, far below a section a ruled loft meets at an angle.
const CREASE: f64 = 1e-6;

/// How near another crease or an end a crease may stand, as a part of the interval, and still be
/// one: a kink a law in pieces puts inside a piece is closed in on by pieces a millionth of the
/// interval long, whose joints either side of it are no sections.
const CREASE_GAP: f64 = 1e-3;

/// **Where the walls meet at an angle across the interval** (#243 L7): the joints of the pieces
/// at which some wall's run along `s` turns — a ruled loft's sections, each a link of a chain the
/// family runs in step — as indices of the piece each begins.  A joint the walls run on through,
/// tangent, is none (a pipe's, from a straight to a bend).
fn creases(loops: &[Vec<Wall>]) -> Vec<usize> {
    let Some(first) = loops.first().and_then(|l| l.first()) else { return Vec::new() };
    let st = &first.sheet.stations;
    let [a,b] = st.span();
    let gap = CREASE_GAP*(b-a);
    let turned = |j: usize| {
        let s = st.spans[j][0];
        loops.iter().flatten().any(|w| (0..=4).any(|i| {
            let uv = [i as f64/4.,s];
            let (l,r) = (w.sheet.rate_in(j-1,uv),w.sheet.rate_in(j,uv));
            let (nl,nr) = (norm(l),norm(r));
            nl > 0. && nr > 0. && 1.-dot(l,r)/(nl*nr) > CREASE
        }))
    };
    let all: Vec<usize> = (1..st.spans.len()).filter(|&j| turned(j)).collect();
    let at = |j: usize| st.spans[j][0];
    all.iter().copied().filter(|&j| {
        let s = at(j);
        s-a > gap && b-s > gap && all.iter().all(|&k| k == j || (at(k)-s).abs() > gap)
    }).collect()
}

/// **The solid a face family's walls bound** (see the module): `loops` the section's loops, the
/// outer first, each the walls of its edges in walk order; an edge's run at an end no longer than
/// `vanish` (mm) is vanished there.
pub fn family(loops: &[Vec<Wall>],vanish: f64) -> Result<Brep,String> {
    let first = &loops.first().and_then(|l| l.first()).ok_or("a face family has no walls")?.sheet;
    let [a,b] = first.span();
    if loops.iter().flatten().any(|w| w.sheet.span() != [a,b]) { return Err("a face family's walls run over different intervals".into()) }
    let mid = (a+b)/2.;
    // the section's normal at the middle, the walls' frames' axis
    let section = unit(newell(&loop_points(&loops[0],mid)));
    let mut out = Brep::default();
    // the bands between the creases, each wall's sheet over its pieces alone, and where they begin
    // and end
    let pieces = first.stations.spans.len();
    let starts: Vec<usize> = std::iter::once(0).chain(creases(loops)).collect();
    let bands: Vec<Band> = starts.iter().enumerate().map(|(i,&j0)| {
        let j1 = starts.get(i+1).copied().unwrap_or(pieces);
        if starts.len() == 1 {
            let walls = loops.iter().map(|l| l.iter().map(|w| w.sheet.clone()).collect()).collect();
            return Band {span:[a,b],walls}
        }
        let stations = Arc::new(first.stations.band(j0..j1));
        let walls = loops.iter()
            .map(|l| l.iter().map(|w| Arc::new(w.sheet.band(stations.clone(),j0..j1))).collect())
            .collect();
        Band {span:stations.span(),walls}
    }).collect();
    let cuts: Vec<f64> = bands.iter().map(|b| b.span[0]).chain([b]).collect();
    // per loop, per end, the rims' edges (none where the loop vanished whole)
    let mut rims: Vec<[Vec<(u32,bool)>;2]> = Vec::new();
    // each rail's edge, and the wall it is the start of
    let mut all_rails: Vec<(u32,Arc<Sheet>)> = Vec::new();
    for (li,walls) in loops.iter().enumerate() {
        let m = walls.len();
        let closed = m == 1 && walls[0].sheet.closed;
        if (m == 1) != closed { return Err("a face family's loop of one edge must run round".into()) }
        // each cut's corners — the ends' and each crease's — those an edge vanished between at an
        // end joined in one vertex
        let mut corner_ids: Vec<Vec<u32>> = Vec::with_capacity(cuts.len());
        let mut vanished: Vec<Vec<bool>> = Vec::with_capacity(cuts.len());
        for (e,&s) in cuts.iter().enumerate() {
            let end = e == 0 || e+1 == cuts.len();
            vanished.push(walls.iter().map(|w| end && rim_length(&w.sheet,s) <= vanish).collect());
            let mut root: Vec<usize> = (0..m).collect();
            for k in 0..m {
                if vanished[e][k] && !closed {
                    let (x,y) = (root[k],root[(k+1)%m]);
                    for r in &mut root { if *r == y { *r = x; } }
                }
            }
            let mut ids = vec![u32::MAX;m];
            for k in 0..m {
                if ids[root[k]] == u32::MAX {
                    let members: Vec<V> = (0..m).filter(|&j| root[j] == root[k]).map(|j| walls[j].sheet.rim(0.,s).0).collect();
                    let p = scale(members.iter().fold([0.;3],|acc,&q| add(acc,q)),1./members.len() as f64);
                    ids[root[k]] = out.vertex(p);
                }
                ids[k] = ids[root[k]];
            }
            corner_ids.push(ids);
        }
        // each corner's rail, a band at a time
        let rails: Vec<Vec<u32>> = bands.iter().enumerate().map(|(bi,band)| (0..m).map(|k| {
            let v = [corner_ids[bi][k],corner_ids[bi+1][k]];
            out.edge(EdgeCurve::Curve(Curve::Rail(band.walls[li][k].clone(),0.)),band.span,v)
        }).collect()).collect();
        if bands.len() == 1 {
            all_rails.extend(rails[0].iter().zip(walls).map(|(&e,w)| (e,w.sheet.clone())));
        }
        // each edge's rim where a band begins and where the last ends, vanished only at the ends
        let mut ends: Vec<Vec<(u32,bool)>> = Vec::with_capacity(cuts.len());
        for (e,&s) in cuts.iter().enumerate() {
            let band = &bands[e.min(bands.len()-1)];
            let mut rim = Vec::with_capacity(m);
            for k in 0..m {
                let v = [corner_ids[e][k],corner_ids[e][(k+1)%m]];
                let id = if vanished[e][k] {
                    out.edge(EdgeCurve::Degenerate,[0.,1.],[v[0],v[0]])
                } else {
                    let (curve,t) = rim_curve(&band.walls[li][k],s);
                    out.edge(EdgeCurve::Curve(curve),t,v)
                };
                rim.push((id,vanished[e][k]));
            }
            ends.push(rim);
        }
        for (bi,band) in bands.iter().enumerate() {
            let [lo,hi] = band.span;
            let middle = (lo+hi)/2.;
            // which way the loop runs about its own plane in this band: the section may have
            // turned round since the family's middle (a U)
            let n_loop = unit(newell(&loop_points(walls,middle)));
            for (k,w) in walls.iter().enumerate() {
                let sheet = &band.walls[li][k];
                let line = |p: Uv,q: Uv| Pcurve::Line {a:p,b:q};
                let mut l = vec![
                    Coedge {edge:ends[bi][k].0,reversed:false,pcurve:line([0.,lo],[1.,lo])},
                    Coedge {edge:rails[bi][(k+1)%m],reversed:false,pcurve:line([1.,lo],[1.,hi])},
                    Coedge {edge:ends[bi+1][k].0,reversed:true,pcurve:line([0.,hi],[1.,hi])},
                    Coedge {edge:rails[bi][k],reversed:true,pcurve:line([0.,lo],[0.,hi])},
                ];
                // outward, in the section's plane: right of the walk about its own loop's normal
                // for the outer loop, left of it for a hole
                let (_,tangent,_) = sheet.d1([0.5,middle]);
                let outward = if li == 0 { cross(tangent,n_loop) } else { cross(n_loop,tangent) };
                let surface = Surface::Family(Frame::about(w.sheet.rim(0.,a).0,section),sheet.clone());
                let reversed = dot(surface.normal_raw([0.5,middle]),outward) < 0.;
                let face = Face {surface,reversed,loops:vec![],name:w.name.clone()};
                let ccw = out.loop_area(&face,&l) > 0.;
                if ccw == reversed { l = reverse_loop(l); }
                out.faces.push(Face {loops:vec![l],..face});
            }
        }
        // the caps' rims: the first cut's and the last's
        let last = ends.pop().unwrap_or_default();
        let first = ends.swap_remove(0);
        rims.push([first,last].map(|e| if e.iter().all(|x| x.1) { Vec::new() } else { e }));
    }
    // the caps: the start's material toward where the stations go, the end's behind it
    for (e,s,name) in [(0,a,"start"),(1,b,"end")] {
        if rims[0][e].is_empty() {
            if rims.iter().any(|r| !r[e].is_empty()) { return Err(format!("a face family's section vanishes at its {name} but a hole of it does not")) }
            continue
        }
        let pts = loop_points(&loops[0],s);
        let normal = unit(newell(&pts));
        let centre = scale(pts.iter().fold([0.;3],|acc,&q| add(acc,q)),1./pts.len() as f64);
        // the way the stations go there: their corners' rates, summed
        let rate = loops[0].iter().fold([0.;3],|acc,w| add(acc,w.sheet.d1([0.,s]).2));
        let travel = if dot(normal,rate) > 0. { normal } else { scale(normal,-1.) };
        let outward = if e == 0 { scale(travel,-1.) } else { travel };
        let surface = Surface::Plane(Frame::about(centre,normal));
        let reversed = dot(normal,outward) < 0.;
        let face = Face {surface,reversed,loops:vec![],name:name.into()};
        let mut cap_loops = Vec::new();
        for (li,r) in rims.iter().enumerate() {
            if r[e].is_empty() { continue }
            let l: Vec<Coedge> = r[e].iter().filter(|x| !x.1)
                .map(|x| Coedge {edge:x.0,reversed:false,pcurve:Pcurve::Inverse {a:[0.;2],b:[0.;2]}}).collect();
            let ccw = out.loop_area(&face,&l) > 0.;
            // the outer loop turns with the face's sense, its holes against it
            cap_loops.push(if (li == 0) != (ccw != reversed) { reverse_loop(l) } else { l });
        }
        out.faces.push(Face {loops:cap_loops,..face});
    }
    // carried rigidly, the walls on what the first rim sweeps and the rails on what each corner
    // does — the whole solid exact, its every wall held to the general one — else wall by wall
    let size = loops.iter().flatten().map(|w| distance(w.sheet.d1([0.,a]).0,w.sheet.d1([1.,b]).0)).fold(0.,f64::max);
    let tol = AGREE*size.max(f64::MIN_POSITIVE);
    let rigid = if bands.len() == 1 { carried(loops,tol) } else { None };
    if let Some(carry) = rigid {
        let mut exact = out.clone();
        for (k,sheet) in all_rails {
            let e = &mut exact.edges[k as usize];
            e.curve = EdgeCurve::Curve(Curve::Iso(swept(&sheet,carry),0.));
            e.t = [0.,1.];
        }
        let walls = exact.faces.iter().filter(|f| matches!(f.surface,Surface::Family(..))).count();
        let lowered = lower(&exact,&out,Some(carry));
        if lowered.len() == walls {
            for (fi,face) in lowered { exact.faces[fi] = face; }
            return Ok(exact)
        }
    }
    for (fi,face) in lower(&out,&out,None) { out.faces[fi] = face; }
    Ok(out)
}

/// **A section carried rigidly** (#243 L6): the motion every station is the first one moved by,
/// where there is one — a screw (a turn about one axis and a step along it in proportion to the
/// turn, a turn alone included) or a step along one line. Read off the section's points at every
/// node and the ends (four an edge): each station's place against the first by three of them,
/// the turn's axis off the station turned farthest from square, its centre off the one turned
/// nearest a half turn, each station's turn unwound from the last one's; then every point of every
/// station held to the motion within `tol`, the turn and the step running one way together. Never
/// read off what wrote the section: a helix's word, a twisted duct and a coil all say it alike.
fn carried(loops: &[Vec<Wall>],tol: f64) -> Option<Carry> {
    let first = &loops.first()?.first()?.sheet;
    let [a,b] = first.span();
    let st = &first.stations;
    let mut at: Vec<f64> = st.at.iter().zip(&st.spans).flat_map(|(nodes,sp)| nodes.map(|s| s.clamp(sp[0],sp[1]))).collect();
    at.push(a);
    at.push(b);
    at.sort_by(f64::total_cmp);
    at.dedup();
    let points = |s: f64| -> Vec<V> {
        loops.iter().flatten().flat_map(|w| { let n = w.sheet.numbers(s); (0..4).map(move |j| w.sheet.at(&n,j as f64/4.).0) }).collect()
    };
    let p0 = points(a);
    // three points of the first station well apart: its first, the farthest from it, and the
    // farthest from the line through those two
    let i1 = (0..p0.len()).max_by(|&i,&j| distance(p0[i],p0[0]).total_cmp(&distance(p0[j],p0[0])))?;
    let d1 = crate::space::normalised(sub(p0[i1],p0[0]))?;
    let off = |p: V| norm(cross(d1,sub(p,p0[0])));
    let i2 = (0..p0.len()).max_by(|&i,&j| off(p0[i]).total_cmp(&off(p0[j])))?;
    if !(off(p0[i2]) > 1e3*tol) { return None }
    let frame = |p: &[V]| -> [V;3] {
        let e1 = unit(sub(p[i1],p[0]));
        let w = sub(p[i2],p[0]);
        let e2 = unit(sub(w,scale(e1,dot(w,e1))));
        [e1,e2,cross(e1,e2)]
    };
    let f0 = frame(&p0);
    // each station's turn `R` (taking the first station's frame to its own) and step `t`:
    // x ↦ R x + t
    let turn = |f: &[V;3],x: V| -> V { add(add(scale(f[0],dot(x,f0[0])),scale(f[1],dot(x,f0[1]))),scale(f[2],dot(x,f0[2]))) };
    let mut moves: Vec<([V;3],V,Vec<V>)> = Vec::with_capacity(at.len());
    for &s in &at {
        let p = points(s);
        let f = frame(&p);
        let t = sub(p[0],turn(&f,p0[0]));
        moves.push((f,t,p));
    }
    // sin θ times the axis, and cos θ, of a turn: Σ f0ᵢ × R f0ᵢ = 2 sin θ a, Σ f0ᵢ · R f0ᵢ = 1 + 2 cos θ
    let sine = |f: &[V;3]| scale((0..3).fold([0.;3],|acc,i| add(acc,cross(f0[i],f[i]))),0.5);
    let cosine = |f: &[V;3]| ((0..3).map(|i| dot(f0[i],f[i])).sum::<f64>()-1.)/2.;
    let held = |carry: &Carry,v: &[f64]| moves.iter().zip(v).all(|((_,_,p),&v)| p0.iter().zip(p).all(|(&x,&y)| distance(carry.at(x,v),y) <= tol));
    let widest = (0..moves.len()).max_by(|&i,&j| norm(sine(&moves[i].0)).total_cmp(&norm(sine(&moves[j].0))))?;
    if norm(sine(&moves[widest].0)) <= 1e-12 {
        // no turn: a step along one line
        let delta = moves.last()?.1;
        let len = norm(delta);
        if !(len > 1e3*tol) { return None }
        let v: Vec<f64> = moves.iter().map(|m| dot(m.1,delta)/(len*len)).collect();
        let carry = Carry::Line {delta};
        return (v.windows(2).all(|w| w[1] >= w[0]) && held(&carry,&v)).then_some(carry)
    }
    let axis = unit(sine(&moves[widest].0));
    // each station's turn about `axis`, signed, unwound from the last
    let w0 = crate::space::normalised(cross(axis,f0[0])).or_else(|| crate::space::normalised(cross(axis,f0[1])))?;
    let mut theta: Vec<f64> = Vec::with_capacity(moves.len());
    for (f,_,_) in &moves {
        let w = turn(f,w0);
        let raw = dot(axis,cross(w0,w)).datan2(dot(w0,w));
        let last = theta.last().copied().unwrap_or(0.);
        theta.push(raw+std::f64::consts::TAU*((last-raw)/std::f64::consts::TAU).round());
    }
    let angle = *theta.last()?;
    if !(angle.abs() > 1e-9) { return None }
    // the centre, off the station turned nearest a half turn: (I − R) c = t − (a·t) a in the
    // plane square to the axis, so c = ½ b + ½ cot(θ/2) a × b there
    let half = (0..moves.len()).max_by(|&i,&j| cosine(&moves[j].0).total_cmp(&cosine(&moves[i].0)))?;
    let t = moves[half].1;
    let bv = sub(t,scale(axis,dot(axis,t)));
    let centre = add(scale(bv,0.5),scale(cross(axis,bv),0.5/(theta[half]/2.).dtan()));
    let advance = dot(axis,moves.last()?.1);
    let v: Vec<f64> = theta.iter().map(|th| th/angle).collect();
    let carry = Carry::Screw {center:centre,axis,angle,advance};
    (v.windows(2).all(|w| w[1] >= w[0]) && held(&carry,&v)).then_some(carry)
}

/// How near a lowered surface must pass every point of the sheet it stands for, as a part of the
/// wall's size, and how near their areas must agree.
const AGREE: f64 = 1e-10;

/// **The analytic fast path**: each wall whose sheet is a plane, a cylinder, a cone or a torus put
/// on that surface — read off what its numbers do, never off what made them: a line edge that
/// stays in one plane; an arc whose plane stays square to one axis through its centre (a cylinder
/// where its radius holds, else a cone where the radius runs with the height); an arc whose plane
/// turns about one axis lying in it (a torus). Failing those, where the section moves rigidly by
/// `carry` (`carried`), the surface its first rim sweeps (`swept`). Held to the sheet before it is
/// kept: the sheet's points at every node on the surface within `AGREE` of the wall's size, and
/// the face's area on it within `AGREE` of the sheet's in `general`. A wall with a vanished edge is
/// lowered only to a plane. The walls of `b` that lower, each as its face on its new surface.
fn lower(b: &Brep,general: &Brep,carry: Option<Carry>) -> Vec<(usize,Face)> {
    let mut out = Vec::new();
    for fi in 0..b.faces.len() {
        let Surface::Family(_,sheet) = &b.faces[fi].surface else { continue };
        let sheet = sheet.clone();
        let [s0,s1] = sheet.span();
        let size = distance(sheet.d1([0.,s0]).0,sheet.d1([1.,s1]).0).max(distance(sheet.d1([1.,s0]).0,sheet.d1([0.,s1]).0));
        let tol = AGREE*size.max(f64::MIN_POSITIVE);
        let degenerate = b.faces[fi].loops[0].iter().any(|c| matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate));
        let frame = *b.faces[fi].surface.frame();
        let candidates = [analytic(&sheet,tol),carry.map(|c| Surface::Blend(frame,swept(&sheet,c)))];
        for surface in candidates.into_iter().flatten() {
            if degenerate && !matches!(surface,Surface::Plane(_)) { continue }
            if let Some(face) = lowered(b,general,fi,&sheet,surface,size,tol) { out.push((fi,face)); break }
        }
    }
    out
}

/// The wall `sheet` swept by `carry` from its rim at the family's start: `M(v)·C(u)`, the rim
/// blended with itself (`geom::Blend`); at `u = 0`, its rail.
fn swept(sheet: &Arc<Sheet>,carry: Carry) -> Arc<Blend> {
    let (curve,t) = rim_curve(sheet,sheet.span()[0]);
    Arc::new(Blend {a:curve.clone(),ta:t,b:curve,tb:t,carry,closed:sheet.closed})
}

/// Face `fi` of `b`, whose surface is `sheet`, put on `surface` — or none where the sheet's points
/// at a node stray from it by more than `tol`, or the face's area on it is not the sheet's in
/// `general` (the solid on its sheets, edge for edge) to `AGREE` (`size` the wall's).
fn lowered(b: &Brep,general: &Brep,fi: usize,sheet: &Arc<Sheet>,surface: Surface,size: f64,tol: f64) -> Option<Face> {
    let [s0,s1] = sheet.span();
    // every node's points on it
    let on = sheet.stations.at.iter().zip(&sheet.stations.spans).all(|(at,sp)| at.iter().all(|&s| {
        let s = s.clamp(sp[0],sp[1]);
        (0..=8).all(|i| { let p = sheet.d1([i as f64/8.,s]).0; distance(surface.point(surface.inverse(p)),p) <= tol })
    }));
    if !on { return None }
    // the corners' parameters on it, each walked to from the last so a periodic one unwraps
    let chart = |uv: Uv,from: Option<(Uv,Uv)>| -> Uv {
        let Some((from_fs,from_uv)) = from else { return surface.inverse(sheet.d1(uv).0) };
        let mut at = from_uv;
        for j in 1..=32 {
            let f = j as f64/32.;
            let q = [from_fs[0]+(uv[0]-from_fs[0])*f,from_fs[1]+(uv[1]-from_fs[1])*f];
            at = super::topo::unwrap(surface.inverse(sheet.d1(q).0),at,surface.periods());
        }
        at
    };
    let c00 = chart([0.,s0],None);
    let c10 = chart([1.,s0],Some(([0.,s0],c00)));
    let c01 = chart([0.,s1],Some(([0.,s0],c00)));
    let c11 = chart([1.,s1],Some(([0.,s1],c01)));
    let corner = |uv: Uv| -> Uv {
        match (uv[0] > 0.5,uv[1] > (s0+s1)/2.) { (false,false) => c00,(true,false) => c10,(false,true) => c01,(true,true) => c11 }
    };
    let mut face = b.faces[fi].clone();
    for c in face.loops.iter_mut().flatten() {
        let Pcurve::Line {a,b: z} = c.pcurve else { continue };
        let (p,q) = (corner(a),corner(z));
        c.pcurve = if matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate) { Pcurve::Line {a:p,b:q} } else { Pcurve::Inverse {a:p,b:q} };
    }
    face.surface = surface;
    // the face's sense: its outward normal as the sheet's
    let sheet_face = &general.faces[fi];
    let mid = [0.5,(s0+s1)/2.];
    let (x,_,_) = sheet.d1(mid);
    let out = scale(sheet_face.surface.normal_raw(mid),if sheet_face.reversed { -1. } else { 1. });
    face.reversed = dot(face.surface.normal_raw(face.surface.inverse(x)),out) < 0.;
    let ccw = b.loop_area(&face,&face.loops[0]) > 0.;
    if ccw == face.reversed { face.loops[0] = reverse_loop(face.loops[0].clone()); }
    let (want,got) = (super::props::area(general,sheet_face),super::props::area(b,&face));
    ((want-got).abs() <= AGREE*want.abs().max(size*size*AGREE)).then_some(face)
}

/// The analytic surface a wall's sheet is, where its numbers say it is one (see `lower`): read off
/// its first, middle and last nodes and the axis they give, and held to every node after.
fn analytic(sheet: &Sheet,tol: f64) -> Option<Surface> {
    let nodes = &sheet.nodes;
    match sheet.run {
        Run::Line => {
            // the plane of the first node's line and the farthest other end
            let (a,b) = (v3(&nodes[0],0),v3(&nodes[0],3));
            let d = sub(b,a);
            let far = nodes.iter().flat_map(|n| [v3(n,0),v3(n,3)]).max_by(|p,q| norm(cross(d,sub(*p,a))).total_cmp(&norm(cross(d,sub(*q,a)))))?;
            let normal = crate::space::normalised(cross(d,sub(far,a)))?;
            let flat = nodes.iter().all(|n| [v3(n,0),v3(n,3)].iter().all(|&p| dot(sub(p,a),normal).abs() <= tol));
            if flat { return Some(Surface::Plane(Frame::new(a,normal,d))) }
            turned_line(nodes,tol)
        }
        Run::Round => {
            let all: Vec<(V,V,f64)> = nodes.iter().map(|n| circle(n,1e-9)).collect::<Option<_>>()?;
            let (c0,z0,r0) = all[0];
            let (c1,z1,_) = all[all.len()-1];
            if norm(cross(z0,z1)) <= 1e-9 {
                // coaxial: every centre on the axis, every plane square to it
                let axis = crate::space::normalised(sub(c1,c0)).filter(|e| norm(cross(*e,z0)) <= 1e-9)?;
                let h = |c: V| dot(sub(c,c0),axis);
                if all.iter().any(|&(c,z,_)| norm(cross(z,axis)) > 1e-9 || norm(sub(sub(c,c0),scale(axis,h(c)))) > tol) { return None }
                let (r1,h1) = (all[all.len()-1].2,h(c1));
                let x = v3(&nodes[0],3);
                if (r1-r0).abs() <= tol {
                    return all.iter().all(|&(_,_,r)| (r-r0).abs() <= tol).then(|| Surface::Cylinder(Frame::new(c0,axis,x),r0))
                }
                // a cone: the radius affine in the height
                let slope = (r1-r0)/h1;
                if all.iter().any(|&(c,_,r)| (r0+slope*h(c)-r).abs() > tol) { return None }
                let half = slope.datan();
                return Some(Surface::Cone(Frame::new(c0,axis,x),r0,half))
            }
            // turned about one axis lying in every arc's plane: the line where the first and last
            // planes meet, the centres all as far from it
            let along = unit(cross(z0,z1));
            // a point on both planes: solve z0·p = z0·c0, z1·p = z1·c1 with p in the plane spanned by z0, z1
            let (d0,d1,k) = (dot(z0,c0),dot(z1,c1),dot(z0,z1));
            let det = 1.-k*k;
            if !(det.abs() > 1e-12) { return None }
            let (g0,g1) = ((d0-k*d1)/det,(d1-k*d0)/det);
            let o = add(scale(z0,g0),scale(z1,g1));
            let o = add(o,scale(along,dot(sub(c0,o),along)));
            let big = distance(c0,o);
            if !(big > r0) { return None }
            let ok = all.iter().all(|&(c,z,r)| (r-r0).abs() <= tol && dot(z,along).abs() <= 1e-9
                && dot(sub(c,o),along).abs() <= tol && (distance(c,o)-big).abs() <= tol && dot(sub(o,c),z).abs() <= tol);
            ok.then(|| Surface::Torus(Frame::new(o,along,sub(c0,o)),big,r0))
        }
        Run::Spline {..} => None,
    }
}

/// A line edge turned about one axis — its ends each keeping their distance from the axis and
/// their height along it, the axis the one the first, middle and last of one end's places circle
/// — as the cylinder or the cone that turn sweeps.
fn turned_line(nodes: &[Vec<f64>],tol: f64) -> Option<Surface> {
    let (p0,pm,p1) = (v3(&nodes[0],0),v3(&nodes[nodes.len()/2],0),v3(&nodes[nodes.len()-1],0));
    let (e1,e2) = (sub(pm,p0),sub(p1,p0));
    let z = cross(e1,e2);
    let zz = dot(z,z);
    if !(zz > 0.) { return None }
    // the centre of the circle through the three
    let centre = add(p0,scale(add(scale(cross(z,e1),dot(e2,e2)),scale(cross(e2,z),dot(e1,e1))),0.5/zz));
    let z = unit(z);
    let meridian = |p: V| { let d = sub(p,centre); let h = dot(d,z); (norm(sub(d,scale(z,h))),h) };
    let ((ra,ha),(rb,hb)) = (meridian(v3(&nodes[0],0)),meridian(v3(&nodes[0],3)));
    let keeps = nodes.iter().all(|n| {
        let ((r1,h1),(r2,h2)) = (meridian(v3(n,0)),meridian(v3(n,3)));
        (r1-ra).abs() <= tol && (h1-ha).abs() <= tol && (r2-rb).abs() <= tol && (h2-hb).abs() <= tol
    });
    if !keeps || (hb-ha).abs() <= tol { return None }
    let a = v3(&nodes[0],0);
    let x = sub(sub(a,centre),scale(z,ha));
    if (rb-ra).abs() <= tol { return Some(Surface::Cylinder(Frame::new(centre,z,x),ra)) }
    // its axis the way the line climbs, so its half-angle is under a right angle
    let (z,h0,dh) = if hb > ha { (z,ha,hb-ha) } else { (scale(z,-1.),-ha,ha-hb) };
    Some(Surface::Cone(Frame::new(add(centre,scale(z,h0)),z,x),ra,(rb-ra).datan2(dh)))
}
