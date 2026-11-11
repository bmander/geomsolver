//! **The tumbling torus's volume and area, made apart from every kernel** (#880 E2): brep-bench's
//! `swept/tumbling-torus` (`swept_tumble.sv`), the solid a torus (centre (3, 0, 2), axis along z,
//! radii 1 and 0.5) fills as it turns by t/4 about the upright line through (2, 0, 0) and is seen
//! from a frame turning by t about the line through (0, 0.5, 0.5) along x, t from −75° to 75°.
//! Nothing here reads the kernel: only `std`, the motion and the torus as numbers.
//!
//! The torus is every point within its tube's radius `r` of its core circle `C`, so the sweep is
//! every point within `r` of the ribbon `Σ(t, θ) = M(t) C(θ)` the core circle sweeps, and a point's
//! membership is `m(p) = min over t of d(M(t)⁻¹ p, C) − r`, the distance to a circle in closed form
//! — 1-Lipschitz in `p`. A point of the sweep's boundary has its nearest point of `Σ` inside the
//! roll, where it stands `r` along `Σ`'s normal (the ribbon is regular: its tangents never part by
//! less than a sine of 0.057), or at an end of the roll, where it lies on the torus there. So the
//! boundary's candidates are the two offset sheets `Σ ± r ν` over `(t, θ)` and the torus at the two
//! end poses over `(θ, ψ)`; a candidate's point is kept where no pose holds it strictly inside —
//! its own pose a least of the distance over the roll (the sheet's area factor
//! `1 ∓ r tr W + r² det W` positive; at an end pose, the distance growing into the roll) and every
//! other least no nearer than `r`.
//!
//! The volume is `⅓ ∮ (p − o)·n dA` and the area `∮ dA` over what is kept. Each candidate is read
//! along lines of its inner parameter: the stretches kept found where the margins at a stretch's
//! ends no longer outrun a Lipschitz bound, each change bisected to 1e-13, and the stretches
//! integrated by adaptive Gauss–Kronrod (7–15); the lines then by adaptive Gauss–Kronrod in the
//! outer parameter. The kept surface closes: `∮ n dA` is nothing to the tolerance.
//!
//! At a tolerance of 1e-8 for each candidate the sweep's volume is 25.2750727653 and its area
//! 59.1796087950. cadrs (2026-11-10) agrees to 1.1e-8 and 4.1e-9; cad-kernel 0.6.3's solid is
//! 3.33e-5 low (1.32e-6) and 5.6e-4 high (9.5e-6), at any fit tolerance from 1e-5 to 1e-7 — four
//! of its dipped stretches (`swept::rings::dip`) stand wholly inside the sweep beside the end
//! poses, up to 3.6e-3 deep, and bound pockets of it the solid leaves out
//! (docs/swept-solid-design.md).
use std::f64::consts::TAU;

type V = [f64;3];
const CENTRE: V = [3.,0.,2.];
const RING: f64 = 1.;
const TUBE: f64 = 0.5;
/// The torus's own turn, about the upright line through this point, at a quarter of the roll.
const HUB: V = [2.,0.,0.];
/// The observer's turn, about the line through this point along x, at the roll.
const KAXIS: V = [0.,0.5,0.5];
fn roll() -> [f64;2] { [-75f64.to_radians(),75f64.to_radians()] }
/// Where the volume's flux is read from.
const ORIGIN: V = [2.,0.3,1.2];
/// The poses membership reads at once, before its leasts are refined.
const POSES: usize = 2400;

fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn sub(a: V,b: V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn scale(a: V,k: f64) -> V { [a[0]*k,a[1]*k,a[2]*k] }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }
type M = [[f64;3];3];
fn apply(m: M,x: V) -> V { [dot(m[0],x),dot(m[1],x),dot(m[2],x)] }
fn matmul(a: M,b: M) -> M {
    let mut c = [[0.;3];3];
    for i in 0..3 { for j in 0..3 { for k in 0..3 { c[i][j] += a[i][k]*b[k][j] } } }
    c
}

/// The pose at `t`, `x ↦ m x + k`: turned by t/4 about the hub's line, then by −t about the
/// observer's.
fn pose(t: f64) -> (M,V) {
    let (s1,c1) = (0.25*t).sin_cos();
    let (s2,c2) = (-t).sin_cos();
    let ra = [[c1,-s1,0.],[s1,c1,0.],[0.,0.,1.]];
    let rb = [[1.,0.,0.],[0.,c2,-s2],[0.,s2,c2]];
    (matmul(rb,ra),add(KAXIS,apply(rb,sub(sub(HUB,apply(ra,HUB)),KAXIS))))
}
/// The pose at `t` undone, the torus's centre taken off: `q ↦ M(t)⁻¹ q − centre`.
fn undone(t: f64) -> (M,V) {
    let (m,k) = pose(t);
    let mt = [[m[0][0],m[1][0],m[2][0]],[m[0][1],m[1][1],m[2][1]],[m[0][2],m[1][2],m[2][2]]];
    (mt,sub(scale(apply(mt,k),-1.),CENTRE))
}
/// The distance from `w` (about the torus's centre) to its core circle.
fn to_core(w: V) -> f64 { let rho = (w[0]*w[0]+w[1]*w[1]).sqrt(); ((rho-RING)*(rho-RING)+w[2]*w[2]).sqrt() }
fn distance(p: V,t: f64) -> f64 { let (m,k) = undone(t); to_core(add(apply(m,p),k)) }

/// The least of `distance(p, ·)` on `[a, b]`, its ends included (Brent's minimisation).
fn least(p: V,mut a: f64,mut b: f64) -> f64 {
    let (lo,hi) = (a,b);
    let gold = 0.381_966_011_250_105_1;
    let mut x = a+gold*(b-a);
    let (mut w,mut v) = (x,x);
    let mut fx = distance(p,x);
    let (mut fw,mut fv) = (fx,fx);
    let (mut d,mut e) = (0f64,0f64);
    for _ in 0..200 {
        let xm = 0.5*(a+b);
        let tol1 = 1e-11*x.abs()+1e-13;
        if (x-xm).abs() <= 2.*tol1-0.5*(b-a) { break }
        let mut golden = true;
        if e.abs() > tol1 {
            let r = (x-w)*(fx-fv);
            let mut q = (x-v)*(fx-fw);
            let mut pp = (x-v)*q-(x-w)*r;
            q = 2.*(q-r);
            if q > 0. { pp = -pp } else { q = -q }
            if pp.abs() < (0.5*q*e).abs() && pp > q*(a-x) && pp < q*(b-x) {
                e = d;
                d = pp/q;
                if x+d-a < 2.*tol1 || b-x-d < 2.*tol1 { d = if xm >= x { tol1 } else { -tol1 } }
                golden = false;
            }
        }
        if golden { e = if x >= xm { a-x } else { b-x }; d = gold*e }
        let u = if d.abs() >= tol1 { x+d } else { x+tol1.copysign(d) };
        let fu = distance(p,u);
        if fu <= fx {
            if u >= x { a = x } else { b = x }
            (v,fv,w,fw,x,fx) = (w,fw,x,fx,u,fu);
        } else {
            if u < x { a = u } else { b = u }
            if fu <= fw || w == x { (v,fv,w,fw) = (w,fw,u,fu) }
            else if fu <= fv || v == x || v == w { (v,fv) = (u,fu) }
        }
    }
    fx.min(distance(p,lo)).min(distance(p,hi))
}

/// The roll's poses, undone, read at once.
struct Poses { t: Vec<f64>,undone: Vec<(M,V)> }
impl Poses {
    fn new() -> Poses {
        let [a,b] = roll();
        let t: Vec<f64> = (0..POSES).map(|i| a+(b-a)*i as f64/(POSES-1) as f64).collect();
        let undone = t.iter().map(|&t| undone(t)).collect();
        Poses {t,undone}
    }
}

/// Whose pose a candidate's point is: a sheet's at a roll inside, or the torus's where it starts or
/// stops.
#[derive(Clone,Copy)]
enum Own { At(f64),Start,End }

/// **How far a candidate's point stands from being held inside by another pose**: the least
/// distance from `p` to the core circle over the roll outside its own pose's basin (the stretch of
/// the roll about its own pose over which the distance only grows away from it), less the tube's
/// radius. Negative where another pose holds the point strictly inside the sweep.
fn other(ps: &Poses,p: V,own: Own) -> f64 {
    let n = POSES;
    let f: Vec<f64> = ps.undone.iter().map(|(m,k)| to_core(add(apply(*m,p),*k))).collect();
    // the basin: the samples strictly between `lo` and `hi`
    let (lo,hi): (isize,isize) = match own {
        Own::At(t) => {
            let i = (((t-ps.t[0])/(ps.t[1]-ps.t[0])).floor() as isize).clamp(0,n as isize-2) as usize;
            let (mut prev,mut j) = (TUBE,i+1);
            while j < n && f[j] >= prev { prev = f[j]; j += 1 }
            let hi = j as isize;
            let (mut prev,mut j) = (TUBE,i as isize);
            while j >= 0 && f[j as usize] >= prev { prev = f[j as usize]; j -= 1 }
            (j,hi)
        }
        Own::Start => {
            let (mut prev,mut j) = (f[0],1);
            while j < n && f[j] >= prev { prev = f[j]; j += 1 }
            (-1,j as isize)
        }
        Own::End => {
            let (mut prev,mut j) = (f[n-1],n as isize-2);
            while j >= 0 && f[j as usize] >= prev { prev = f[j as usize]; j -= 1 }
            (j,n as isize)
        }
    };
    let mut g = f64::INFINITY;
    let outside = (0..=lo).chain(hi.max(0)..n as isize).map(|k| k as usize);
    for k in outside {
        let left = if k > 0 { f[k-1] } else { f64::INFINITY };
        let right = if k+1 < n { f[k+1] } else { f64::INFINITY };
        if f[k] <= left && f[k] <= right {
            // a sample far from the tube's radius needs no refining to say which side it stands
            let v = if f[k]-TUBE > 0.05 { f[k] } else { least(p,ps.t[k.saturating_sub(1)],ps.t[(k+1).min(n-1)]) };
            g = g.min(v);
        }
    }
    g-TUBE
}

/// A second-order jet in two variables: a value, its gradient and its Hessian `[00, 01, 11]`.
#[derive(Clone,Copy)]
struct J { v: f64,g: [f64;2],h: [f64;3] }
impl J {
    fn c(v: f64) -> J { J {v,g:[0.;2],h:[0.;3]} }
    fn var(v: f64,i: usize) -> J { let mut j = J::c(v); j.g[i] = 1.; j }
    /// `φ(self)`, given `φ`, `φ'` and `φ''` at its value.
    fn of(self,f: f64,d1: f64,d2: f64) -> J {
        let (g,h) = (self.g,self.h);
        J {v:f,g:[d1*g[0],d1*g[1]],h:[d1*h[0]+d2*g[0]*g[0],d1*h[1]+d2*g[0]*g[1],d1*h[2]+d2*g[1]*g[1]]}
    }
    fn sin(self) -> J { let (s,c) = self.v.sin_cos(); self.of(s,c,-s) }
    fn cos(self) -> J { let (s,c) = self.v.sin_cos(); self.of(c,-s,-c) }
    fn k(self,k: f64) -> J { self.of(self.v*k,k,0.) }
}
impl std::ops::Add for J {
    type Output = J;
    fn add(self,b: J) -> J {
        let a = self;
        J {v:a.v+b.v,g:[a.g[0]+b.g[0],a.g[1]+b.g[1]],h:[a.h[0]+b.h[0],a.h[1]+b.h[1],a.h[2]+b.h[2]]}
    }
}
impl std::ops::Sub for J { type Output = J; fn sub(self,b: J) -> J { self+b.k(-1.) } }
impl std::ops::Mul for J {
    type Output = J;
    fn mul(self,b: J) -> J {
        let a = self;
        J {v:a.v*b.v,g:[a.g[0]*b.v+a.v*b.g[0],a.g[1]*b.v+a.v*b.g[1]],
            h:[a.h[0]*b.v+2.*a.g[0]*b.g[0]+a.v*b.h[0],a.h[1]*b.v+a.g[0]*b.g[1]+a.g[1]*b.g[0]+a.v*b.h[1],
                a.h[2]*b.v+2.*a.g[1]*b.g[1]+a.v*b.h[2]]}
    }
}

/// The ribbon `Σ(t, θ)` and its first and second derivatives, as jets.
fn ribbon(t: f64,th: f64) -> [J;3] {
    let (t,th) = (J::var(t,0),J::var(th,1));
    let x = [J::c(CENTRE[0])+th.cos().k(RING),J::c(CENTRE[1])+th.sin().k(RING),J::c(CENTRE[2])];
    // turned by t/4 about the hub's line
    let (s,c) = (t.k(0.25).sin(),t.k(0.25).cos());
    let (dx,dy) = (x[0]-J::c(HUB[0]),x[1]-J::c(HUB[1]));
    let y = [J::c(HUB[0])+c*dx-s*dy,J::c(HUB[1])+s*dx+c*dy,x[2]];
    // then by −t about the observer's
    let (s,c) = (t.k(-1.).sin(),t.k(-1.).cos());
    let (dy,dz) = (y[1]-J::c(KAXIS[1]),y[2]-J::c(KAXIS[2]));
    [y[0],J::c(KAXIS[1])+c*dy-s*dz,J::c(KAXIS[2])+s*dy+c*dz]
}

/// A point of a candidate: where it is, its outward normal, its area element, and whether it is
/// kept with its margin (a distance) and a bound on how fast the point moves along its line.
struct Point { p: V,n: V,da: f64,kept: bool,margin: f64,speed: f64 }

/// The offset sheet `Σ + σ r ν` at `(t, θ)`.
fn sheet(ps: &Poses,t: f64,th: f64,sigma: f64,read: bool) -> Point {
    let s = ribbon(t,th);
    let at = |f: &dyn Fn(&J) -> f64| [f(&s[0]),f(&s[1]),f(&s[2])];
    let (st,sth) = (at(&|j| j.g[0]),at(&|j| j.g[1]));
    let normal = cross(st,sth);
    let nu = scale(normal,1./norm(normal));
    let (e,f,g) = (dot(st,st),dot(st,sth),dot(sth,sth));
    let (l,m,n) = (dot(at(&|j| j.h[0]),nu),dot(at(&|j| j.h[1]),nu),dot(at(&|j| j.h[2]),nu));
    let first = e*g-f*f;
    let (trace,det) = ((e*n-2.*f*m+g*l)/first,(l*n-m*m)/first);
    // the area factor of the offset at the tube's radius: its pose a least of the distance where
    // positive
    let factor = 1.-sigma*TUBE*trace+TUBE*TUBE*det;
    let p = add(at(&|j| j.v),scale(nu,sigma*TUBE));
    let speed = g.sqrt()*(1.+2.*TUBE*(trace.abs()+det.abs().sqrt()));
    let (kept,margin) = if read {
        let o = other(ps,p,Own::At(t));
        (factor > 0. && o >= 0.,if o < 0. || factor > 0. { o } else { -0.02*factor.abs() })
    } else { (true,0.) };
    Point {p,n:scale(nu,sigma),da:factor.abs()*norm(normal),kept,margin,speed}
}

/// The torus at end pose `e` (0 where the roll starts, 1 where it stops) at `(θ, ψ)`.
fn end(ps: &Poses,e: usize,th: f64,psi: f64,read: bool) -> Point {
    let t = roll()[e];
    let ((st,ct),(sp,cp)) = (th.sin_cos(),psi.sin_cos());
    let m = [cp*ct,cp*st,sp];
    let (rot,k) = pose(t);
    let p = add(apply(rot,add(add(CENTRE,[RING*ct,RING*st,0.]),scale(m,TUBE))),k);
    let (kept,margin) = if read {
        // the distance's rate into the roll (a sign, and a margin taken to change no faster than
        // ten times the point moves): the torus trails there where it grows
        let h = 1e-6;
        let rate = (distance(p,t+h)-distance(p,t-h))/(2.*h)*if e == 0 { 1. } else { -1. };
        let o = other(ps,p,if e == 0 { Own::Start } else { Own::End });
        let r = 0.1*rate;
        (rate > 0. && o >= 0.,if rate > 0. { if o >= 0. { o.min(r) } else { o } } else { r })
    } else { (true,0.) };
    Point {p,n:apply(rot,m),da:TUBE*(RING+TUBE*cp),kept,margin,speed:1.}
}

/// What a stretch of the boundary adds: the volume's flux, the area, and `∮ n dA`.
#[derive(Clone,Copy,Default)]
struct Sum { volume: f64,area: f64,normal: V }
impl Sum {
    fn plus(self,o: Sum) -> Sum { Sum {volume:self.volume+o.volume,area:self.area+o.area,normal:add(self.normal,o.normal)} }
    fn times(self,k: f64) -> Sum { Sum {volume:self.volume*k,area:self.area*k,normal:scale(self.normal,k)} }
    fn of(q: &Point) -> Sum { Sum {volume:dot(sub(q.p,ORIGIN),q.n)*q.da/3.,area:q.da,normal:scale(q.n,q.da)} }
    fn apart(self,o: Sum) -> f64 { (self.volume-o.volume).abs()+(self.area-o.area).abs()+norm(sub(self.normal,o.normal)) }
}

/// Gauss–Kronrod 7–15 on `[a, b]`: the Kronrod sum and the Gauss.
fn kronrod(f: &(dyn Fn(f64) -> Sum + Sync),a: f64,b: f64,parallel: bool) -> (Sum,Sum) {
    const X: [f64;7] = [0.991455371120812639206854697526329,0.949107912342758524526189684047851,
        0.864864423359769072789712788640926,0.741531185599394439863864773280788,
        0.586087235467691130294144845693013,0.405845151377397166906606412076961,
        0.207784955007898467600689403773245];
    const WK: [f64;8] = [0.022935322010529224963732008058970,0.063092092629978553290700663189204,
        0.104790010322250183839876322541518,0.140653259715525918745189590510238,
        0.169004726639267902826583426598550,0.190350578064785409913256402421014,
        0.204432940075298892414161999234649,0.209482141084727828012999174891714];
    const WG: [f64;4] = [0.129484966168869693270611432679082,0.279705391489276667901467771423780,
        0.381830050505118944950369775488975,0.417959183673469387755102040816327];
    let (c,h) = (0.5*(a+b),0.5*(b-a));
    let mut xs = vec![c];
    for x in X { xs.push(c-h*x); xs.push(c+h*x) }
    let ys: Vec<Sum> = if parallel {
        std::thread::scope(|s| {
            let parts: Vec<_> = xs.chunks(4).map(|xs| s.spawn(move || xs.iter().map(|&x| f(x)).collect::<Vec<_>>())).collect();
            parts.into_iter().flat_map(|p| p.join().unwrap()).collect()
        })
    } else { xs.iter().map(|&x| f(x)).collect() };
    let (mut k,mut g) = (ys[0].times(WK[7]),ys[0].times(WG[3]));
    for j in 0..7 {
        let pair = ys[1+2*j].plus(ys[2+2*j]);
        k = k.plus(pair.times(WK[j]));
        if j%2 == 1 { g = g.plus(pair.times(WG[j/2])) }
    }
    (k.times(h),g.times(h))
}
/// `∫ f` over `[a, b]`, halved where the Kronrod and Gauss sums part by more than `tol`.
fn integral(f: &(dyn Fn(f64) -> Sum + Sync),a: f64,b: f64,tol: f64,parallel: bool) -> Sum {
    let (k,g) = kronrod(f,a,b,parallel);
    if k.apart(g) <= tol || b-a < 1e-12 { return k }
    let c = 0.5*(a+b);
    integral(f,a,c,0.6*tol,parallel).plus(integral(f,c,b,0.6*tol,parallel))
}

/// The stretches of `[0, 2π]` a line keeps: from 128 points, each interval halved until the
/// margins at its ends outrun its points' speed across it (no change hides there) or, where it
/// changes, until it is 1e-13 wide.
fn kept(read: &dyn Fn(f64) -> Point) -> Vec<[f64;2]> {
    fn walk(read: &dyn Fn(f64) -> Point,a: f64,qa: (bool,f64,f64),b: f64,qb: (bool,f64,f64),cuts: &mut Vec<f64>,depth: u32) {
        if qa.0 == qb.0 {
            if qa.1.abs()+qb.1.abs() > 1.2*qa.2.max(qb.2)*(b-a) || b-a < 1e-9 || depth > 60 { return }
        } else if b-a < 1e-13 || depth > 60 {
            cuts.push(0.5*(a+b));
            return;
        }
        let m = 0.5*(a+b);
        let q = read(m);
        let qm = (q.kept,q.margin,q.speed);
        walk(read,a,qa,m,qm,cuts,depth+1);
        walk(read,m,qm,b,qb,cuts,depth+1);
    }
    let n = 128;
    let xs: Vec<f64> = (0..=n).map(|i| TAU*i as f64/n as f64).collect();
    let qs: Vec<(bool,f64,f64)> = xs.iter().map(|&x| { let q = read(x); (q.kept,q.margin,q.speed) }).collect();
    let mut cuts = Vec::new();
    for i in 0..n { walk(read,xs[i],qs[i],xs[i+1],qs[i+1],&mut cuts,0) }
    let (mut out,mut from) = (Vec::new(),if qs[0].0 { Some(0.) } else { None });
    for c in cuts {
        match from { Some(f) => { out.push([f,c]); from = None } None => from = Some(c) }
    }
    if let Some(f) = from { out.push([f,TAU]) }
    out
}

/// A candidate's flux, area and `∮ n dA`: lines of its inner parameter over `[0, 2π]`, kept where
/// membership says, integrated over its outer parameter `[a, b]` to `tol`.
fn candidate(point: &(dyn Fn(f64,f64,bool) -> Point + Sync),[a,b]: [f64;2],tol: f64) -> Sum {
    let line = |x: f64| {
        let read = |y: f64| point(x,y,true);
        let at = |y: f64| Sum::of(&point(x,y,false));
        kept(&read).into_iter().fold(Sum::default(),|s,[c,d]| s.plus(integral(&at,c,d,1e-13,false)))
    };
    let panels = 8;
    (0..panels).fold(Sum::default(),|s,i| {
        let (c,d) = (a+(b-a)*i as f64/panels as f64,a+(b-a)*(i+1) as f64/panels as f64);
        s.plus(integral(&line,c,d,tol/panels as f64,true))
    })
}

/// **The tumbling torus's volume and area, made apart** (the slow tier): the two offset sheets of
/// the ribbon and the torus at both end poses, each kept where membership says, the four read to
/// 1e-6 each (some minutes): the volume and area within 1e-7 of the sweep's, `∮ n dA` nothing, and
/// each candidate's share what it was read at 1e-8. The outer offset sheet is kept whole; the inner
/// one only past t ≈ 0.487, where its kept stretch is born.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, some minutes: the tumbling torus's volume made apart")]
fn the_tumbling_torus_measures_apart_from_any_kernel() {
    let ps = Poses::new();
    let roll = roll();
    let whole = [0.,TAU];
    let parts = [
        ("the inner sheet",candidate(&|t,th,r| sheet(&ps,t,th,1.,r),roll,1e-6),1.762531284938,4.499186100472),
        ("the outer sheet",candidate(&|t,th,r| sheet(&ps,t,th,-1.,r),roll,1e-6),18.084529790689,38.356122122775),
        ("the start",candidate(&|th,psi,r| end(&ps,0,th,psi,r),whole,1e-6),1.808432374894,8.244989519392),
        ("the end",candidate(&|th,psi,r| end(&ps,1,th,psi,r),whole,1e-6),3.619579314800,8.079311052336),
    ];
    let mut total = Sum::default();
    for (name,s,volume,area) in parts {
        assert!((s.volume-volume).abs() < 1e-7 && (s.area-area).abs() < 1e-7,
            "{name}: volume {:.12} area {:.12}, read at 1e-8 {volume} {area}",s.volume,s.area);
        total = total.plus(s);
    }
    assert!((total.volume-25.2750727653).abs() < 2.5e-6,"{:.10}",total.volume);
    assert!((total.area-59.1796087950).abs() < 6e-6,"{:.10}",total.area);
    assert!(norm(total.normal) < 1e-5,"the boundary does not close: ∮ n dA = {:?}",total.normal);
}
