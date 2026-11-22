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
use crate::util::space::{cross,dot,norm};

/// Ten-point Gauss–Legendre nodes and weights on [−1, 1].
const GAUSS: [(f64,f64);10] = [
    (-0.9739065285171717,0.0666713443086881),(-0.8650633666889845,0.1494513491505806),
    (-0.6794095682990244,0.2190863625159820),(-0.4333953941292472,0.2692667193099963),
    (-0.1488743389816312,0.2955242247147529),(0.1488743389816312,0.2955242247147529),
    (0.4333953941292472,0.2692667193099963),(0.6794095682990244,0.2190863625159820),
    (0.8650633666889845,0.1494513491505806),(0.9739065285171717,0.0666713443086881),
];

/// `∫_a^b f` by ten-point Gauss–Legendre on `pieces` equal pieces.
pub(crate) fn gauss(a: f64,b: f64,pieces: usize,f: &mut dyn FnMut(f64) -> f64) -> f64 {
    let h = (b-a)/pieces as f64;
    let mut sum = 0.;
    for p in 0..pieces {
        let mid = a+h*(p as f64+0.5);
        for &(x,w) in &GAUSS { sum += w*f(mid+x*h/2.); }
    }
    sum*h/2.
}

/// The Kronrod nodes of `GAUSS` (its 21-point extension, exact for a polynomial of degree 31):
/// the eleven it adds, with their weights, and the weights it gives `GAUSS`'s ten (in its order).
const KRONROD: [(f64,f64);11] = [
    (-0.9956571630258081,0.011694638867371874),(-0.9301574913557082,0.054755896574351996),
    (-0.7808177265864169,0.09312545458369760),(-0.5627571346686047,0.12349197626206585),
    (-0.2943928627014602,0.14277593857706008),(0.,0.1494455540029169),
    (0.2943928627014602,0.14277593857706008),(0.5627571346686047,0.12349197626206585),
    (0.7808177265864169,0.09312545458369760),(0.9301574913557082,0.054755896574351996),
    (0.9956571630258081,0.011694638867371874),
];
const KRONROD_GAUSS: [f64;10] = [
    0.032558162307964727,0.07503967481091995,0.10938715880229764,0.13470921731147333,
    0.14773910490133849,0.14773910490133849,0.13470921731147333,0.10938715880229764,
    0.07503967481091995,0.032558162307964727,
];

/// **One stretch of a use between its breaks, checked**: the ten-point rule (`gauss` on one
/// piece), kept where its 21-point Kronrod extension, which reuses its nodes, agrees with it to a
/// part in 1e-13 of `∫|f|` or within `floor`; else the extension, where its own error as QUADPACK
/// estimates it is within that; otherwise the stretch `settled`. Ten nodes read most
/// stretches of a spline to their last bits, and those read as they always were, to the bit; but a
/// stretch is not one polynomial of a degree ten read — `G` along a curve, an area's `|S_u × S_v|`,
/// a rational sheet — and unchecked, a file's edge one knot of whose curve stood a hair past its
/// start had its whole half turn about a rational net read by ten, a face's area 3.8e-5 off
/// (brep-bench's frustum screwed across a ring sector, its drum leaning across a turn 1.7e-6).
fn checked(a: f64,b: f64,floor: f64,f: &mut dyn FnMut(f64) -> f64) -> f64 {
    let (mid,h) = ((a+b)/2.,(b-a)/2.);
    let mut at = Vec::with_capacity(GAUSS.len());
    let ten = gauss(a,b,1,&mut |t| { let y = f(t); at.push(y); y });
    let (mut k,mut size) = (0.,0.);
    for (&y,&w) in at.iter().zip(&KRONROD_GAUSS) { k += w*y; size += w*y.abs(); }
    for &(x,w) in &KRONROD { let y = f(mid+x*h); k += w*y; size += w*y.abs(); }
    let (k,size) = (k*h,size*h.abs());
    let within = (1e-13*size).max(floor);
    let off = (k-ten).abs();
    // (the extension's own error by QUADPACK's estimate, `(200 |K − G|)^1.5` of the size: within
    // the budget, it is read)
    if off <= within { ten } else if size > 0. && size*(200.*off/size).dpowf(1.5).min(1.) <= within { k } else { settled(a,b,1,floor,f) }
}

/// The two-point Gauss rule over `[a, b]`: exact for a cubic.
fn gauss2(a: f64,b: f64,mut f: impl FnMut(f64) -> f64) -> f64 {
    let (m,h) = ((a+b)/2.,(b-a)/2.);
    let k = h/3f64.sqrt();
    h*(f(m-k)+f(m+k))
}

/// **A swept sheet's face across its fold, read by columns** (#880 T1b): where a carried section's
/// sheet folds at the face (`Surface::Blend`, its area factor below `ON_FOLD` at a point of a use
/// its box was read from — the fold point's path, where the sheet's two rates run together; a
/// sheet whose rates merely lean together, below `NEAR_FOLD` across its box without folding, a
/// spline bar screwed four turns, took the columns at every reading the trim made of it, eight
/// times its build), a point beside the fold stands in the parameters at the square root of its
/// distance in space, so a use's curve inverted onto the sheet there is off along the fold's way by that much.
/// Green's theorem (`integrated`) reads it through `G`, the integrand gathered from the face's
/// middle, which is no smaller there, and on the ball screwed tighter than its radius two such faces
/// read 1.65e-7 and 4e-8 off their own boundaries. Read instead as `∬ g du dv` column by column in
/// `u` — each column's crossings with the loops bisected on the uses' own curves, `g` gathered
/// between them — the error stands where `g` itself vanishes, at the fold. `None` for any other
/// face, and for a sheet running round (a period), whose columns would wrap.
fn by_columns(b: &Brep,f: &Face,what: What,lo: Uv,hi: Uv) -> Option<f64> {
    let s = &f.surface;
    if s.periods().iter().any(Option::is_some) || !(hi[0] > lo[0] && hi[1] > lo[1]) { return None }
    // each use, its curve in the parameters read at points along its edge (in the loop's order)
    const N: usize = 256;
    let uses: Vec<(&Pcurve,&super::topo::Edge,Vec<(f64,Uv)>)> = f.loops.iter().flatten().map(|c| {
        let e = &b.edges[c.edge as usize];
        let mut pts: Vec<(f64,Uv)> = (0..=N).map(|k| { let t = e.t[0]+(e.t[1]-e.t[0])*k as f64/N as f64; (t,c.pcurve.at(t,e,s,&b.vertices)) }).collect();
        if c.reversed { pts.reverse() }
        (&c.pcurve,e,pts)
    }).collect();
    // the columns' crossings: each sampled stretch of a use astride `u`, bisected on its own curve
    let mut column = |u: f64| -> f64 {
        let mut vs: Vec<f64> = Vec::new();
        for &(pc,e,ref pts) in &uses {
            for w in pts.windows(2) {
                let ((t0,p),(t1,q)) = (w[0],w[1]);
                if (p[0] < u) == (q[0] < u) { continue }
                let below = p[0] < u;
                let (x,y) = crate::util::roots::bisect(t0,t1,|x,y| 0.5*(x+y),|x,y| { let m = 0.5*(x+y); m != x && m != y },
                    |m| (pc.at(m,e,s,&b.vertices)[0] < u) == below);
                vs.push(pc.at(0.5*(x+y),e,s,&b.vertices)[1]);
            }
        }
        vs.sort_by(f64::total_cmp);
        vs.chunks(2).filter(|c| c.len() == 2).map(|c| gauss(c[0],c[1],4,&mut |v| integrand(s,[u,v],what))).sum()
    };
    // over `u`, cut where a column's crossings change: at the uses' ends and where a use turns in `u`
    // (turns a billionth of the box apart one cut: a use along a column turns at its own rounding)
    let mut cuts: Vec<f64> = Vec::new();
    for (_,_,pts) in &uses {
        cuts.extend([pts[0].1[0],pts[pts.len()-1].1[0]]);
        cuts.extend(pts.windows(3).filter(|w| (w[1].1[0]-w[0].1[0])*(w[2].1[0]-w[1].1[0]) < 0.).map(|w| w[1].1[0]));
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|x,y| (*x-*y).abs() <= 1e-9*(hi[0]-lo[0]));
    let total: f64 = cuts.windows(2).map(|w| settled(w[0],w[1],4,0.,&mut column)).sum();
    // the face's side, as Green's theorem takes it: the way its loops run round the parameters
    // (a weighed area's sign is its caller's, read off its area the same way: `face_moments`)
    Some(match what {
        What::Area => total.abs(),
        What::Weighed(_) => total,
        _ => total*f.loops.iter().map(|l| b.loop_area(f,l)).sum::<f64>().signum(),
    })
}

/// The area factor (`Surface::area_factor`) a swept sheet's face is read by columns below, at a
/// point of a use: on the fold point's path, to the rounding of the use's inversion there.
const ON_FOLD: f64 = 1e-6;

/// **What a face's measure integrates**: the flux of `X` or the area — each with its closed forms
/// and tables — or the flux of a field `F(X)`, or the area weighed by `w(X)`, read by the rule
/// alone (`moments`, `face_moments`).
#[derive(Clone,Copy)]
enum What<'a> { Flux,Area,Field(&'a (dyn Fn(V) -> V + Sync)),Weighed(&'a (dyn Fn(V) -> f64 + Sync)) }

impl What<'_> {
    /// Whether it is the area, read by the area's closed forms.
    fn area(self) -> bool { matches!(self,What::Area) }
    /// Whether it is the flux of `X` or the area, which have closed forms and tables.
    fn plain(self) -> bool { matches!(self,What::Flux | What::Area) }
}

/// The integrand of a face's measure at `uv`: the area's `|S_u × S_v|`, the flux's
/// `X · (S_u × S_v)`, a field's `F(X) · (S_u × S_v)` or a weighed area's `w(X) |S_u × S_v|`.
fn integrand(s: &Surface,uv: Uv,what: What) -> f64 {
    let (x,su,sv) = s.d1(uv);
    integrand_of(x,su,sv,what)
}

/// `integrand` of the point and the derivatives read.
fn integrand_of(x: V,su: V,sv: V,what: What) -> f64 {
    match what {
        What::Flux => dot(x,cross(su,sv)),
        What::Area => norm(cross(su,sv)),
        What::Field(f) => dot(f(x),cross(su,sv)),
        What::Weighed(w) => w(x)*norm(cross(su,sv)),
    }
}

/// `gauss` over `pieces`, then twice as many, keeping the coarser reading where they agree.
/// Otherwise refine each piece locally: a circle passing close to a ball's pole can sweep half
/// a turn of `u` in a tiny part of its span, which six doublings of the whole edge still miss.
fn settled(a: f64,b: f64,pieces: usize,floor: f64,f: &mut dyn FnMut(f64) -> f64) -> f64 {
    let coarse = gauss(a,b,pieces,f);
    let mut size = 0.;
    let fine = gauss(a,b,2*pieces,&mut |t| { let y = f(t); size += y.abs(); y });
    if (fine-coarse).abs() <= 1e-13*fine.abs().max(1e-300) { return coarse }
    // The mean magnitude of the integrand, including where its signed readings cancel. This
    // sets one error budget for the edge, not a relative budget for each vanishing contribution
    // beside a pole: the latter refines rounding noise ever more finely near the singularity.
    let within = (1e-13*size*(b-a).abs()/(2*pieces*GAUSS.len()) as f64).max(floor).max(1e-300);
    let h = (b-a)/pieces as f64;
    let mut cells: Vec<_> = (0..pieces).map(|i| {
        let (x,y) = (a+i as f64*h,a+(i+1) as f64*h);
        Reading::new(x,y,gauss(x,y,1,f),0,f)
    }).collect();
    // Spend the edge's work on the largest estimated error. Keep the best reading if refinement
    // stops improving it: approaching an exact pole can amplify the inverse's rounding, so the
    // finest reading need not be the best. Depth and work are bounded as the doublings were.
    let mut best = cells.iter().map(|c| c.lo+c.hi).sum();
    let mut error = cells.iter().map(Reading::error).sum::<f64>();
    let mut stalled = 0;
    for _ in 0..4096 {
        if error <= within || stalled == 32 { break }
        let Some(i) = (0..cells.len()).filter(|&i| cells[i].depth < 20)
            .max_by(|&i,&j| cells[i].error().total_cmp(&cells[j].error())) else { break };
        let c = cells.swap_remove(i);
        let m = (c.a+c.b)/2.;
        if m == c.a || m == c.b { cells.push(c); break }
        cells.push(Reading::new(c.a,m,c.lo,c.depth+1,f));
        cells.push(Reading::new(m,c.b,c.hi,c.depth+1,f));
        let next = cells.iter().map(Reading::error).sum::<f64>();
        if next < error {
            best = cells.iter().map(|c| c.lo+c.hi).sum();
            error = next;
            stalled = 0;
        } else { stalled += 1 }
    }
    best
}

/// One interval's rule and its two halves, reused when that interval is split.
struct Reading { a: f64,b: f64,whole: f64,lo: f64,hi: f64,depth: usize }

impl Reading {
    fn new(a: f64,b: f64,whole: f64,depth: usize,f: &mut dyn FnMut(f64) -> f64) -> Reading {
        let m = (a+b)/2.;
        Reading {a,b,whole,lo:gauss(a,m,1,f),hi:gauss(m,b,1,f),depth}
    }
    fn error(&self) -> f64 { (self.lo+self.hi-self.whole).abs() }
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
    /// Per `v` span, once asked for, its column (`Column`).
    cols: Vec<Option<Column>>,
}

/// A `v` span's column of `Prefix`'s table: per node, the net's rows there (`Row`) and `G` at each
/// `u` knot, read over the knots `lo..=hi` (about `base`), every node alike.
struct Column { rows: Vec<Vec<Row>>,g: Vec<Vec<f64>>,lo: usize,hi: usize }

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
        let cols = (1..vs.len()).map(|_| None).collect();
        let base = if from.is_finite() { (0..us.len()).min_by(|&a,&b| (us[a]-from).abs().total_cmp(&(us[b]-from).abs())).unwrap_or(0) } else { 0 };
        Prefix {net,us,vs,ru:rule((3*net.du).div_ceil(2)+1),nodes,base,cols}
    }
    fn g(&self,u: f64,v: f64) -> f64 { let (x,su,sv) = self.net.d1(u,v); dot(x,cross(su,sv)) }
    /// The net's rows read at `v` (`Row`) by the patch of the `v` span starting at `from` — at that
    /// span's far end too, which a B-spline read there takes from the next span: across a knot of
    /// multiplicity `dv` (a net joined in value only, creased) `S_v` jumps, and a column's end node
    /// read so tabled `G` from the wrong side, a family's bar 2.8e-5 off its flux.
    fn rows(&self,v: f64,from: f64) -> Vec<Row> {
        let n = self.net;
        let ([v0,v1],nv) = (n.domain()[1],n.poles[0].len());
        // the span's own knots and poles alone: every `v` is read in its one span
        let s = super::nurbs::span(n.dv,&n.vknots,nv,from);
        let knots = &n.vknots[s-n.dv..=s+n.dv+1];
        super::nurbs::with_basis(n.dv,knots,n.dv+1,v.clamp(v0,v1),1,|sv,bv| n.poles.iter().map(|row| {
            let (mut x,mut d) = ([0.;3],[0.;3]);
            for j in 0..=n.dv { let q = row[s+sv-2*n.dv+j]; for c in 0..3 { x[c] += bv[0][j]*q[c]; d[c] += bv[1][j]*q[c]; } }
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
        let (base,n) = (self.base,self.us.len());
        let mut col = self.cols[j].take().unwrap_or_else(|| Column {rows:self.nodes.iter().map(|&x| self.rows(at(x),v0)).collect(),
            g:vec![vec![0.;n];self.nodes.len()],lo:base,hi:base});
        // filled out to knot `i`, a span at a time
        while col.hi < i {
            let h = col.hi;
            for (rows,g) in col.rows.iter().zip(&mut col.g) { g[h+1] = g[h]+self.across_rows(self.us[h],self.us[h+1],rows) }
            col.hi += 1;
        }
        while col.lo > i {
            let l = col.lo;
            for (rows,g) in col.rows.iter().zip(&mut col.g) { g[l-1] = g[l]-self.across_rows(self.us[l-1],self.us[l],rows) }
            col.lo -= 1;
        }
        // the polynomial through the nodes at `v` (barycentric, the Chebyshev weights)
        let x = (2.*v-v0-v1)/(v1-v0);
        let m = self.nodes.len();
        let (mut num,mut den) = (0.,0.);
        let mut exact = None;
        for (k,&xk) in self.nodes.iter().enumerate() {
            let gk = col.g[k][i];
            if x == xk { exact = Some(gk); break }
            let w = if k%2 == 0 { 1. } else { -1. }*if k == 0 || k == m-1 { 0.5 } else { 1. }/(x-xk);
            num += w*gk;
            den += w;
        }
        self.cols[j] = Some(col);
        exact.unwrap_or(num/den)+self.across(self.us[i],u,v)
    }
}

/// **A polynomial net read along one parameter, the other held** (`along` 0: `u` runs, `v` held at
/// `fixed`): at the held value each row of poles across it makes one pole of the iso curve and of
/// the sheet's rate across it, so each point read along it is a curve's, `du + 1` poles a span,
/// where the sheet's is `(du + 1)(dv + 1)` and both bases. `G` along a face's parameter integrates
/// its spans from the face's middle at every node of every use: on brep-bench's bull-nose carried
/// along a spline (a 91 × 143 quintic) the area read some twenty million points of the sheet. The
/// rows are read as the spans asked reach them.
struct Iso<'a> {
    net: &'a Net,
    along: usize,
    /// The held parameter's span and its basis there (values and rates).
    span: usize,
    basis: [Vec<f64>;2],
    /// Each row across: the iso curve's pole and the rate's across it, once read.
    rows: Vec<Option<Row>>,
}

impl<'a> Iso<'a> {
    fn new(net: &'a Net,along: usize,fixed: f64) -> Iso<'a> {
        let [du,dv] = net.domain();
        let (nu,nv) = (net.poles.len(),net.poles[0].len());
        let (d,knots,count,[a,b]) = if along == 0 { (net.dv,&net.vknots,nv,dv) } else { (net.du,&net.uknots,nu,du) };
        let (span,basis) = super::nurbs::with_basis(d,knots,count,fixed.clamp(a,b),1,|s,bs| (s,[bs[0][..=d].to_vec(),bs[1][..=d].to_vec()]));
        Iso {net,along,span,basis,rows:vec![None;if along == 0 { nu } else { nv }]}
    }
    fn row(&mut self,i: usize) -> Row {
        if let Some(r) = self.rows[i] { return r }
        let n = self.net;
        let (d,s) = (if self.along == 0 { n.dv } else { n.du },self.span);
        let (mut x,mut r) = ([0.;3],[0.;3]);
        for j in 0..=d {
            let q = if self.along == 0 { n.poles[i][s-d+j] } else { n.poles[s-d+j][i] };
            for c in 0..3 { x[c] += self.basis[0][j]*q[c]; r[c] += self.basis[1][j]*q[c]; }
        }
        self.rows[i] = Some((x,r));
        (x,r)
    }
    /// The sheet's point and `S_u`, `S_v` at `w` along it.
    fn d1(&mut self,w: f64) -> (V,V,V) {
        let n = self.net;
        let [du,dv] = n.domain();
        let (d,knots,[a,b]) = if self.along == 0 { (n.du,&n.uknots,du) } else { (n.dv,&n.vknots,dv) };
        let count = self.rows.len();
        let (s,b0,b1) = super::nurbs::with_basis(d,knots,count,w.clamp(a,b),1,|s,bs| {
            let mut v0 = [0.;super::nurbs::MAX_DEGREE+1];
            let mut v1 = [0.;super::nurbs::MAX_DEGREE+1];
            v0[..=d].copy_from_slice(&bs[0][..=d]);
            v1[..=d].copy_from_slice(&bs[1][..=d]);
            (s,v0,v1)
        });
        let (mut x,mut dw,mut across) = ([0.;3],[0.;3],[0.;3]);
        for k in 0..=d {
            let (q,r) = self.row(s-d+k);
            for c in 0..3 { x[c] += b0[k]*q[c]; dw[c] += b1[k]*q[c]; across[c] += b0[k]*r[c]; }
        }
        if self.along == 0 { (x,dw,across) } else { (x,across,dw) }
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

/// **`G` in closed form on the analytic surfaces** (`Closed`, prepared once a face): `∫_base^x g` along the parameter `along` (0 for
/// `u`, 1 for `v`), the other held, where `g` is the area's `|S_u × S_v|` or the flux's
/// `X · (S_u × S_v)`. In the surface's frame, with `P` the world origin's place in it (`X = o + l`,
/// so `X · N = (P + l) · w` for `N` the frame's image of `w`), each `g` is a sum of powers and
/// products of the sine and cosine of the angle and of the length along it, integrated term by term:
/// on a plane `g` is constant; on a cylinder `r (P₀ cos u + P₁ sin u) + r²`; on a cone, its radius
/// there `q = r + v sin a`, `q (cos a (P₀ cos u + P₁ sin u) − P₂ sin a) + q² cos a − q v cos a sin
/// a`; on a sphere `r² cos v (cos v (P₀ cos u + P₁ sin u) + P₂ sin v) + r³ cos v`; on a torus,
/// `q = R + r cos v`, `q r (cos v (P₀ cos u + P₁ sin u) + P₂ sin v + q cos v + r sin² v)`; along an
/// extrusion's rulings, constant. `None` elsewhere, and where an area's `|·|` would change its sign
/// within the span (a sphere's past a pole, a spindle torus's, a cone's past its apex), which the
/// rule integrates instead: the same integral, read numerically.
struct Closed<'a> {
    s: &'a Surface,
    area: bool,
    along: usize,
    base: f64,
    /// The world origin's place in the surface's frame (`P`), the base's sine and cosine, and a
    /// cone's half angle's (nought elsewhere): what every reading of the face shares.
    p: V,
    sb: f64,
    cb: f64,
    cone: (f64,f64),
}

impl<'a> Closed<'a> {
    /// The face's closed form along `along` from `base`, where its surface has one.
    fn new(s: &'a Surface,area: bool,along: usize,base: f64) -> Option<Closed<'a>> {
        let cone = match *s {
            Surface::Cone(_,_,a) => a.dsin_cos(),
            Surface::Plane(_) | Surface::Cylinder(..) | Surface::Sphere(..) | Surface::Torus(..) => (0.,0.),
            Surface::Extrusion(..) if along == 1 => (0.,0.),
            _ => return None,
        };
        let f = s.frame();
        let (sb,cb) = base.dsin_cos();
        Some(Closed {s,area,along,base,p:f.dir_local(f.o),sb,cb,cone})
    }

    /// `G` at `(u, v)`; `None` where an area's `|·|` changes sign within the span.
    fn at(&self,[u,v]: Uv) -> Option<f64> {
        let (area,along,base,p,(sb,cb)) = (self.area,self.along,self.base,self.p,(self.sb,self.cb));
        let x = [u,v][along];
        let int_1 = x-base;
        // `sin x` and `cos x`, and the integrals of `cos` and `sin` from the base: `sin x − sin b`
        // and `cos b − cos x`
        let trig = || { let (sx,cx) = x.dsin_cos(); (sx,cx,sx-sb,cb-cx) };
        // the integrals of `cos²` and `cos sin` from the base: `x/2 + sin 2x / 4` and `sin² / 2`
        let squares = |sx: f64,cx: f64| (int_1/2.+(sx*cx-sb*cb)/2.,(sx*sx-sb*sb)/2.);
        // `P₀ cos u + P₁ sin u`, and its integral along `u` from the integrals of `cos` and `sin`
        let ring = || { let (su,cu) = u.dsin_cos(); p[0]*cu+p[1]*su };
        let int_ring = |int_c: f64,int_s: f64| p[0]*int_c+p[1]*int_s;
        Some(match *self.s {
            Surface::Plane(_) => if area { int_1 } else { p[2]*int_1 },
            Surface::Cylinder(_,r) => if area { r*int_1 } else if along == 0 {
                let (_,_,int_c,int_s) = trig();
                r*int_ring(int_c,int_s)+r*r*int_1
            } else { (r*ring()+r*r)*int_1 },
            Surface::Cone(_,r,_) => {
                let (sa,ca) = self.cone;
                let q = |v: f64| r+v*sa;
                if area {
                    if along == 0 { q(v).abs()*int_1 }
                    else if sa == 0. { r.abs()*int_1 }
                    else {
                        // `q |q| / (2 sin a)` rises as `|q|` does, across the apex too
                        let h = |v: f64| q(v)*q(v).abs()/(2.*sa);
                        h(x)-h(base)
                    }
                } else if along == 0 {
                    let (_,_,int_c,int_s) = trig();
                    let q = q(v);
                    q*ca*int_ring(int_c,int_s)+(-q*p[2]*sa+q*q*ca-q*v*ca*sa)*int_1
                } else {
                    // a quadratic in `v`: two Gauss points read it exactly
                    let a = ca*ring()-p[2]*sa;
                    gauss2(base,x,|v: f64| { let q = q(v); a*q+q*q*ca-q*v*ca*sa })
                }
            }
            Surface::Sphere(_,r) => {
                let (sv,cv) = v.dsin_cos();
                if area {
                    if along == 0 { r*r*cv.abs()*int_1 } else {
                        let half = std::f64::consts::FRAC_PI_2;
                        if !(-half..=half).contains(&x) || !(-half..=half).contains(&base) { return None }
                        r*r*trig().2
                    }
                } else if along == 0 {
                    let (_,_,int_c,int_s) = trig();
                    r*r*cv*cv*int_ring(int_c,int_s)+(r*r*cv*p[2]*sv+r*r*r*cv)*int_1
                } else {
                    let (sx,cx,int_c,_) = trig();
                    let (c2,cs) = squares(sx,cx);
                    r*r*(ring()*c2+p[2]*cs)+r*r*r*int_c
                }
            }
            Surface::Torus(_,big,r) => {
                let (sv,cv) = v.dsin_cos();
                let q = big+r*cv;
                if area {
                    if along == 0 { r*q.abs()*int_1 } else if big >= r { r*(big*int_1+r*trig().2) } else {
                        if !(big > -r) { return None }
                        // a spindle torus's `|R + r cos v|` changes its sign at `cos v = −R/r`: its
                        // integral the odd, piecewise `h` on a half turn and whole turns of it
                        r*(spindle(big,r,x)-spindle(big,r,base))
                    }
                } else if along == 0 {
                    let (_,_,int_c,int_s) = trig();
                    q*r*cv*int_ring(int_c,int_s)+q*r*(p[2]*sv+q*cv+r*sv*sv)*int_1
                } else {
                    let (sx,cx,int_c,int_s) = trig();
                    let (c2,cs) = squares(sx,cx);
                    let b = ring()+big;
                    r*(big*b*int_c+big*p[2]*int_s+big*r*int_1+r*b*c2+r*p[2]*cs+r*r*int_c)
                }
            }
            // along a ruling `X · (c′ × z)` and `|c′ × z|` stand still
            Surface::Extrusion(..) => {
                let (x0,su,sv) = self.s.d1([u,base]);
                let w = cross(su,sv);
                (if area { norm(w) } else { dot(x0,w) })*int_1
            }
            _ => return None,
        })
    }
}

/// **Where a use's curve in a face's parameters crosses a line its integrand kinks along**: a
/// spline sheet's knot lines (a cubic's third derivative jumps there, a fillet's net of one
/// continuity its second), and the lines where an area's `|S_u × S_v|` passes nought and turns —
/// a spindle torus's `R + r cos v = 0`, a cone's apex. A rule across a kink converges as a rule
/// across a kink does: the adaptive one refining it to its last bits (most of a commercial file's
/// measuring), a fixed one some 1e-5 off (a spindle torus's seam read across `v`). Each crossing
/// bisected between the use's points at sixteenths of its span, to a billionth of it. A pcurve
/// that is a line is settled whole, through the kinks (crossing each once, at a fraction the
/// rule's points need not meet), and cut only where it crosses a net's crease — a knot line of
/// multiplicity the degree, the net joined in value only — where `g`, and so `G`, *jump* by
/// `X · (S_u × ΔS_v)`, which no rule settles: in closed form, its parameter affine in the edge's.
fn crossings(c: &super::topo::Coedge,e: &super::topo::Edge,s: &Surface,vertices: &[super::topo::Vertex]) -> Vec<f64> {
    // the values of parameter `k` in `[a, b]` the integrand kinks at
    let kinks = |k: usize,a: f64,b: f64| -> Vec<f64> {
        match *s {
            Surface::BSpline(..) => s.breaks(k,[a,b]),
            Surface::Torus(_,big,r) if k == 1 && big.abs() < r => {
                let s0 = (-big/r).dacos();
                let tau = std::f64::consts::TAU;
                let first = ((a+s0)/tau).floor() as i64;
                let last = ((b+s0)/tau).ceil() as i64;
                (first..=last).flat_map(|n| [n as f64*tau-s0,n as f64*tau+s0]).filter(|&x| x > a && x < b).collect()
            }
            Surface::Cone(_,r,half) if k == 1 && half.dsin() != 0. => {
                let apex = -r/half.dsin();
                if apex > a && apex < b { vec![apex] } else { vec![] }
            }
            _ => vec![],
        }
    };
    if !matches!(s,Surface::BSpline(..) | Surface::Torus(..) | Surface::Cone(..)) { return vec![] }
    if let Pcurve::Line {a,b} = c.pcurve {
        // a line where it crosses a net's crease, in closed form (its parameter affine in the
        // edge's): a knot of multiplicity its degree or more, the net joined in value only there,
        // across which `G` jumps
        let Surface::BSpline(_,n) = s else { return vec![] };
        let [t0,t1] = e.t;
        let mut out = Vec::new();
        for (k,(knots,d)) in [(&n.uknots,n.du),(&n.vknots,n.dv)].into_iter().enumerate() {
            if !(a[k] != b[k]) { continue }
            let (lo,hi) = (a[k].min(b[k]),a[k].max(b[k]));
            let mut i = 0;
            while i < knots.len() {
                let j = knots[i..].iter().take_while(|&&x| x == knots[i]).count();
                let x = knots[i];
                if j >= d && x > lo && x < hi { out.push(t0+(x-a[k])/(b[k]-a[k])*(t1-t0)) }
                i += j;
            }
        }
        return out
    }
    let n = 16;
    let at = |t: f64| c.pcurve.at(t,e,s,vertices);
    let ts: Vec<f64> = (0..=n).map(|j| e.t[0]+(e.t[1]-e.t[0])*j as f64/n as f64).collect();
    let uvs: Vec<Uv> = ts.iter().map(|&t| at(t)).collect();
    let mut out = Vec::new();
    // (and on a net, where its normal turns over between two points of the use — its area factor
    // `|S_u × S_v|` through nought, a kink: a spindle torus's net crossing its axis, whose rows there
    // are no knots)
    if let Surface::BSpline(..) = s {
        let normal = |uv: Uv| { let (_,su,sv) = s.d1(uv); cross(su,sv) };
        let ns: Vec<V> = uvs.iter().map(|&q| normal(q)).collect();
        for j in 0..n {
            if !(dot(ns[j],ns[j+1]) < 0.) { continue }
            let (mut lo,mut hi) = (ts[j],ts[j+1]);
            for _ in 0..60 {
                let m = 0.5*(lo+hi);
                if dot(normal(at(m)),ns[j]) >= 0. { lo = m } else { hi = m }
                if hi-lo <= 1e-12*(e.t[1]-e.t[0]).abs() { break }
            }
            out.push(0.5*(lo+hi));
        }
    }
    for j in 0..n {
        for k in 0..2 {
            let (a,b) = (uvs[j][k],uvs[j+1][k]);
            for x in kinks(k,a.min(b),a.max(b)) {
                // bisected on the side of the line each end lies
                let (mut lo,mut hi) = (ts[j],ts[j+1]);
                let below = a < x;
                for _ in 0..60 {
                    let m = 0.5*(lo+hi);
                    if (at(m)[k] < x) == below { lo = m } else { hi = m }
                    if hi-lo <= 1e-9*(e.t[1]-e.t[0]).abs() { break }
                }
                out.push(0.5*(lo+hi));
            }
        }
    }
    out
}

/// `∫_0^x |a + r cos σ| dσ` (`|a| < r`): on `[0, π]` the integral of `a + r cos` up to its root
/// `s₀ = acos(−a/r)` and back down past it, odd about 0, and `P` a whole turn.
fn spindle(a: f64,r: f64,x: f64) -> f64 {
    let s0 = (-a/r).dacos();
    let up = |x: f64| a*x+r*x.dsin();
    let h = |x: f64| if x <= s0 { up(x) } else { 2.*up(s0)-up(x) };
    let turns = (x/std::f64::consts::TAU).round();
    let y = x-turns*std::f64::consts::TAU;
    turns*2.*h(std::f64::consts::PI)+if y < 0. { -h(-y) } else { h(y) }
}

/// The volume the boundary encloses (negative if it is turned inside out). A boundary standing
/// farther from the origin than ten times its size is measured moved to it (by the middle of its
/// vertices): `X · n` there is the volume's flux plus the far point's, which cancels round the
/// boundary, and its rounding is the far point's (a millimetre solid a kilometre out read 5e-5
/// of itself off, #993).
pub fn volume(b: &Brep) -> f64 {
    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for v in &b.vertices { for k in 0..3 { lo[k] = lo[k].min(v.p[k]); hi[k] = hi[k].max(v.p[k]); } }
    let c: V = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
    let size = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
    if b.vertices.is_empty() || !(norm(c) > 10.*size) { return fluxes(b).iter().sum::<f64>()/3. }
    let moved = b.moved(&crate::geom::Rigid {r:crate::geom::Rigid::identity().r,t:c.map(|x| -x)});
    fluxes(&moved).iter().sum::<f64>()/3.
}

/// Each face's flux `∬ X·n dA` out of the solid, side by side.
pub fn fluxes(b: &Brep) -> Vec<f64> {
    crate::util::par::indices(b.faces.len(),|fi| terms(b,fi).iter().flatten().map(|t| t.0+t.1).sum())
}

/// A face's flux term by term: for each use of each loop, its line integral and the closing
/// segment from its end to the next use's start (what a reading of the face can be compared by);
/// a swept sheet's face across its fold, read by columns (`by_columns`), one term.
pub fn terms(b: &Brep,fi: usize) -> Vec<Vec<(f64,f64)>> { integrated(b,&b.faces[fi],What::Flux) }

/// A face's area, `∬ |S_u × S_v| du dv` over its domain, read off its loops as its flux is — so a
/// face on another surface over the same boundary can be held to it (`family::lower`). The face
/// need not be one of `b`'s, only its edges.
pub fn area(b: &Brep,f: &Face) -> f64 {
    integrated(b,f,What::Area).iter().flatten().map(|t| t.0+t.1).sum::<f64>().abs()
}

/// **The ten moments of the solid `b` bounds** (#953): `∫ 1`, `∫ t₀`, `∫ t₁`, `∫ t₂`, then
/// `∫ t₀²`, `t₁²`, `t₂²`, `t₀ t₁`, `t₀ t₂`, `t₁ t₂` by volume, `t = (X − c) / h` — each by the
/// divergence theorem the flux of a field whose divergence it is, `F = (h t₀^(a+1)/(a+1) t₁^b t₂^c,
/// 0, 0)` for `t₀^a t₁^b t₂^c`, each face's read off its loops as the volume's is, so nothing is
/// meshed.  Its parameters' polynomial the rule reads exactly on a polynomial sheet, its knots
/// cut.  Only those `want` asks for are read; the rest are nought.
pub fn moments(b: &Brep,c: V,h: f64,want: [bool;10]) -> [f64;10] {
    let t = move |x: V| -> V { std::array::from_fn(|k| (x[k]-c[k])/h) };
    let fields: [&(dyn Fn(V) -> V + Sync);10] = [
        &|x| [h*t(x)[0],0.,0.],
        &|x| [h*t(x)[0].powi(2)/2.,0.,0.],
        &|x| { let t = t(x); [h*t[0]*t[1],0.,0.] },
        &|x| { let t = t(x); [h*t[0]*t[2],0.,0.] },
        &|x| [h*t(x)[0].powi(3)/3.,0.,0.],
        &|x| { let t = t(x); [h*t[0]*t[1]*t[1],0.,0.] },
        &|x| { let t = t(x); [h*t[0]*t[2]*t[2],0.,0.] },
        &|x| { let t = t(x); [h*t[0]*t[0]/2.*t[1],0.,0.] },
        &|x| { let t = t(x); [h*t[0]*t[0]/2.*t[2],0.,0.] },
        &|x| { let t = t(x); [h*t[0]*t[1]*t[2],0.,0.] },
    ];
    std::array::from_fn(|k| if !want[k] { 0. } else {
        crate::util::par::indices(b.faces.len(),|fi| {
            integrated(b,&b.faces[fi],What::Field(fields[k])).iter().flatten().map(|t| t.0+t.1).sum::<f64>()
        }).iter().sum()
    })
}

/// **The ten moments of face `f` of `b`** by area, in `moments`' order and numbers: the area
/// weighed by each monomial of `t = (X − c) / h`, read off its loops as its area is, its sign its
/// area's.  Only those `want` asks for are read; the rest are nought.
pub fn face_moments(b: &Brep,f: &Face,c: V,h: f64,want: [bool;10]) -> [f64;10] {
    let t = move |x: V| -> V { std::array::from_fn(|k| (x[k]-c[k])/h) };
    let weights: [&(dyn Fn(V) -> f64 + Sync);10] = [
        &|_| 1.,&|x| t(x)[0],&|x| t(x)[1],&|x| t(x)[2],
        &|x| t(x)[0].powi(2),&|x| t(x)[1].powi(2),&|x| t(x)[2].powi(2),
        &|x| { let t = t(x); t[0]*t[1] },&|x| { let t = t(x); t[0]*t[2] },&|x| { let t = t(x); t[1]*t[2] },
    ];
    let read = |k: usize| integrated(b,f,What::Weighed(weights[k])).iter().flatten().map(|t| t.0+t.1).sum::<f64>();
    let area = read(0);
    let sense = area.signum();
    std::array::from_fn(|k| if !want[k] { 0. } else if k == 0 { sense*area } else { sense*read(k) })
}

/// `terms`, of what `what` integrates.
fn integrated(b: &Brep,f: &Face,what: What) -> Vec<Vec<(f64,f64)>> {
    {
        let mut out: Vec<Vec<(f64,f64)>> = Vec::new();
        let s = &f.surface;
        let [pu,pv] = s.periods();
        let g = |u: f64,v: f64| integrand(s,[u,v],what);
        // The face's box in its parameters. Away from a spline's table, `g` is integrated from the
        // middle of the face's range and across the parameter it spans less: `G` is then as small
        // as the face allows, and an error δ in a use's parameters moves the flux by `G δ` — from 0
        // on a sphere ring turned 9 radians round, `G` is π R³ times the turn, and a pcurve 1e-8
        // off in v moved a gear's flux by 1e-5 of itself
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        let sheet = matches!(s,Surface::Blend(..));
        let mut folds = false;
        // (the least ratio of each derivative's length to the other's on the boundary: a sheet's
        // row or column all but collapsed there, a pole in all but name)
        let mut thin = [f64::INFINITY;2];
        for c in f.loops.iter().flatten() {
            let e = &b.edges[c.edge as usize];
            for j in 0..=8 {
                let q = c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/8.,e,s,&b.vertices);
                for k in 0..2 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); }
                folds |= sheet && s.area_factor(q) < ON_FOLD;
                if matches!(s,Surface::BSpline(..)) {
                    let (_,su,sv) = s.d1(q);
                    let (lu,lv) = (norm(su),norm(sv));
                    if lu > 0. && lv > 0. { thin[0] = thin[0].min(lu/lv); thin[1] = thin[1].min(lv/lu); }
                }
            }
        }
        // (a swept sheet's face across its fold read by columns instead)
        if folds { if let Some(v) = by_columns(b,f,what,lo,hi) { return vec![vec![(v,0.)]] } }
        // a polynomial net's `G` tabled from the face's middle `u` (`Prefix`); a rational net's `g`
        // is a quotient, which no Gauss rule integrates exactly: it takes the stretch-by-stretch
        // rule a swept face does
        // a pole a loop passes through, where the uses arriving and leaving set out along different
        // values of the parameter the surface collapses there (`Brep::close_poles` lays a
        // degenerate edge between them): `g` integrated along that parameter, so that `G` there
        // is nought and the value a use arrives along — which a file's edge standing off its face
        // leaves to within that gap, and a curve passing near the pole sweeps through in a hair of
        // its span — weighs nothing
        // — a sheet's row or column all but collapsed where the boundary reaches it too (one of
        // its derivatives a thousandth of the other's there: a fillet's net run into a corner),
        // where the parameter along it is read off its edges' feet as ill as at a pole
        let collapsed = f.loops.iter().flatten().find_map(|c| match (&b.edges[c.edge as usize].curve,&c.pcurve) {
            (EdgeCurve::Degenerate,&Pcurve::Line {a,b}) => (0..2).max_by(|&i,&j| (a[i]-b[i]).abs().total_cmp(&(a[j]-b[j]).abs())),
            _ => None,
        }).or_else(|| (0..2).filter(|&k| thin[k] < 1e-3).min_by(|&i,&j| thin[i].total_cmp(&thin[j])));
        let mut prefix = match s {
            Surface::BSpline(_,n) if !n.is_rational() && matches!(what,What::Flux) && collapsed != Some(1) => Some(Prefix::new(n,(lo[0]+hi[0])/2.)),
            _ => None,
        };
        // a sheet carried between two straight edges (a polyhedral tool's edge swept, `Blend`): its
        // point affine in `u` at every `v` — the carry is rigid, its rate affine in the point — so
        // the flux's `g` is a quadratic in `u`, which two Gauss nodes integrate exactly, read along
        // `u` whichever its face spans less (along `v` the screw's turn makes it no polynomial, and
        // the general rule read forty nodes a stretch, every face of a sweep's arrangement)
        let ruled = matches!(what,What::Flux) && matches!(s,Surface::Blend(_,bl) if matches!((&bl.a,&bl.b),(Curve::Line {..},Curve::Line {..})));
        // the parameter `g` is integrated along (`G` then multiplies the other's derivative)
        let near_pole = collapsed.is_some() || !(super::topo::poleless(s) || matches!(s,Surface::BSpline(..) | Surface::Blend(..)));
        // (on a sheet, the parameter whose knots across the face are fewer: `G` is read a stretch
        // between them at a time, each node's work the stretches it crosses)
        let fewer = match s {
            Surface::BSpline(..) if lo.iter().chain(&hi).all(|x| x.is_finite()) =>
                Some(if s.breaks(1,[lo[1],hi[1]]).len() < s.breaks(0,[lo[0],hi[0]]).len() { 1 } else { 0 }),
            _ => None,
        };
        let along = match collapsed { Some(k) if !ruled => k,_ => if ruled || prefix.is_some() { 0 } else if let Some(k) = fewer { k }
            else if !(hi[1]-lo[1] < hi[0]-lo[0]) { 0 } else { 1 } };
        let base = if prefix.is_some() || !lo[along].is_finite() { 0. } else { (lo[along]+hi[along])/2. };
        let periodic = [pu,pv][along].is_some();
        let closed = if prefix.is_some() || !what.plain() { None } else { Closed::new(s,what.area(),along,base) };
        // ∫ from the base, a stretch at a time between the surface's breaks along it
        let mut big_g = |u: f64,v: f64| {
            if let Some(p) = prefix.as_mut() { return p.big_g(u,v) }
            if let Some(g) = closed.as_ref().and_then(|c| c.at([u,v])) { return g }
            let x = [u,v][along];
            // (a polynomial net read along its iso curve there, `Iso`)
            let mut iso = match s { Surface::BSpline(_,n) if !n.is_rational() => Some(Iso::new(n,along,[v,u][along])),_ => None };
            let mut at_x = |w: f64| match iso.as_mut() {
                Some(c) => { let (x,su,sv) = c.d1(w); integrand_of(x,su,sv,what) }
                None => if along == 0 { g(w,v) } else { g(u,w) },
            };
            let cuts = s.breaks(along,[base,x]);
            if cuts.is_empty() { return if ruled { gauss2(base,x,&mut at_x) } else { gauss(base,x,pieces(x-base,periodic),&mut at_x) } }
            let mut at = vec![base];
            if x >= base { at.extend(cuts) } else { at.extend(cuts.into_iter().rev()) }
            at.push(x);
            at.windows(2).map(|w| gauss(w[0],w[1],2,&mut at_x)).sum::<f64>()
        };
        // ∬ g du dv = ∮ G dv with G along u, or -∮ G du with G along v
        let (other,sign) = if along == 0 { (1,1.) } else { (0,-1.) };
        // the face's own scale, `G` at its box's corners across its span: an edge whose integral
        // is all but nought (one along the base, or a chart's edge where `G` vanishes) is settled to
        // a part in 1e-13 of the face's, not of its own, which its rounding would never reach
        // — and to a part in 1e4 of the strip its own tolerance leaves undetermined, the face's
        // scale times the tolerance over the face's size: a file's edge standing off its face has
        // feet that kink where they meet a sheet's edge, which a rule refines to the cap without
        // reading the face more truly than its edge stands
        let (scale,size) = if lo.iter().chain(&hi).all(|x| x.is_finite()) {
            let corners = [[lo[0],lo[1]],[lo[0],hi[1]],[hi[0],lo[1]],[hi[0],hi[1]]];
            let at: Vec<V> = corners.iter().map(|&q| s.point(q)).collect();
            let size = (0..4).flat_map(|i| (0..4).map(move |j| (i,j))).map(|(i,j)| crate::util::space::distance(at[i],at[j])).fold(0.,f64::max);
            (corners.iter().map(|&[u,v]| big_g(u,v).abs()).fold(0.,f64::max)*(hi[other]-lo[other]),size)
        } else { (0.,0.) };
        let floor = |tol: f64| scale*if size > 0. { (1e-4*tol/size).max(1e-13) } else { 1e-13 };
        for l in &f.loops {
            let mut row: Vec<(f64,f64)> = Vec::new();
            for c in l {
                let e = &b.edges[c.edge as usize];
                // a pole's degenerate edge counts by its line in the face's parameters: along `u` (a
                // sphere's, a cone's) nothing in dv but its du, along `v` (a spline's collapsed row,
                // #993) its dv, `G` there the face's width up to it
                if matches!(e.curve,EdgeCurve::Degenerate) && !matches!(c.pcurve,Pcurve::Line {..}) {
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
                breaks.extend(crossings(c,e,s,&b.vertices));
                breaks.sort_by(f64::total_cmp);
                breaks.dedup();
                let line = if !breaks.is_empty() {
                    // smooth between its knots or points: integrated a stretch at a time
                    let mut cuts = vec![e.t[0]];
                    cuts.extend(breaks);
                    cuts.push(e.t[1]);
                    // (ten nodes a stretch, checked: most stretches of a spline ten read to their
                    // last bits, and a file's curve of a hundred knots was read at twice the nodes
                    // it needed; a stretch ten do not read is settled)
                    // (settled a stretch at a time on a surface with a pole, where a curve passing near
                    // it sweeps half a turn of the collapsed parameter in a hair of a stretch)
                    if near_pole { cuts.windows(2).map(|w| settled(w[0],w[1],1,floor(e.tol),&mut integrand)).sum() }
                    else { cuts.windows(2).map(|w| checked(w[0],w[1],floor(e.tol),&mut integrand)).sum() }
                } else { settled(e.t[0],e.t[1],pieces(span,periodic),floor(e.tol),&mut integrand) };
                row.push((if c.reversed { -line } else { line },0.));
            }
            // the loop closed in the face's parameters, each use's end joined straight to the next's
            // start: the kernel's measured gaps between them are small in space, but `G` across a
            // spline face's chart can be large, and Green's theorem needs a closed curve
            // — and each use's own ends closed to where its curve arrives: an inverse curve's end
            // is read where the walk put it, on the branch of a pole the next use leaves along,
            // its curve a hair in from it on the branch it arrives along (its feet swinging round
            // the pole it passes within its tolerance of); the rule above reads the curve inside,
            // so the chord between the two is the face's boundary there (`sound`'s `Drawn::leap`
            // reads the region it closes)
            let ends: Vec<[Uv;4]> = l.iter().map(|c| {
                let e = &b.edges[c.edge as usize];
                let (a,z) = (c.pcurve.at(e.t[0],e,s,&b.vertices),c.pcurve.at(e.t[1],e,s,&b.vertices));
                let inside = |t: f64,end: Uv,h: f64| {
                    if !matches!(c.pcurve,Pcurve::Inverse {..}) { return end }
                    let (q,d) = c.pcurve.at_and_derivative(t+h,e,s,&b.vertices);
                    // (the curve a hair in where it stands farther from the end than its rate
                    // carries it there: a leap; else the end itself, so a smooth end adds nothing,
                    // not even rounding)
                    let step = (d[0]*h).dhypot(d[1]*h);
                    let gap = (q[0]-end[0]).dhypot(q[1]-end[1]);
                    if q.iter().all(|x| x.is_finite()) && gap.is_finite() && gap > 10.*step.max(1e-12*(1.+end[0].abs()+end[1].abs())) { q } else { end }
                };
                let h = 1e-9*(e.t[1]-e.t[0]);
                let (ai,zi) = (inside(e.t[0],a,h),inside(e.t[1],z,-h));
                if c.reversed { [z,zi,ai,a] } else { [a,ai,zi,z] }
            }).collect();
            for k in 0..ends.len() {
                let next = &ends[(k+1)%ends.len()];
                let path = [ends[k][2],ends[k][3],next[0],next[1]];
                row[k].1 = path.windows(2).map(|w| {
                    let (p,q) = (w[0],w[1]);
                    let dv = q[other]-p[other];
                    if dv == 0. { 0. } else { gauss(0.,1.,1,&mut |x| sign*big_g(p[0]+x*(q[0]-p[0]),p[1]+x*(q[1]-p[1]))*dv) }
                }).sum();
            }
            out.push(row);
        }
        out
    }
}
