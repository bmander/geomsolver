//! **Where two surfaces meet, as a topological answer** (#735 A2): not curves alone but what the
//! meeting is — its branches, each saying whether the surfaces cross along it or only touch; its
//! singular points, where its own topology changes (a point where the surfaces touch and meet in
//! nothing else near, or where branches cross); and whether the two are one surface over the
//! region. An arrangement computes each pair of surfaces' meeting once and shares it with every face
//! of those surfaces (`Shared`).
//!
//! **Complete, not just accurate.** Over two boxes of parameters `[da, db]` a meeting's branches
//! either reach a box's boundary or close inside it. Those reaching the boundary are seeded where
//! the boundary's four sides cross the other surface (each side a curve on its surface, its sign
//! changes against the other's signed distance found by bisection between samples finer than the
//! side's feature). A closed branch bounds a disc of the parameters on which the other surface's
//! signed distance `f = b ∘ a` has an interior extremum: so every critical point of `f` (a Morse
//! analysis of the meeting in `a`'s parameters, found by Newton from a grid) is a seed for the
//! branches round it, each reached by walking down `f`'s gradient to its zero. A critical point at
//! which `f` itself vanishes is where the two surfaces touch: a singular point of the meeting,
//! classified to second order (`ssi::touch`) as a touch alone or a crossing of branches. And between
//! analytic surfaces nothing seeded so is left unproved: `ssi::unseen` halves the box, each cell
//! shown clear of `b` by the signed distance's Lipschitz and second-order bounds or beside a branch
//! already traced, and any it cannot clear is traced from in turn, until none is left. Closed forms
//! (`ssi::intersect`: the quadric pairs, coaxial surfaces of revolution, a screwed line's sheets)
//! are taken where they exist; the search still runs over them for the critical points.
use super::geom::{Curve,Surface,Uv,V};
use super::ssi::{intersect,Ssi};
use crate::util::space::{cross,distance,norm,scale,sub};

/// How two surfaces stand along a branch of their meeting.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Contact {
    /// They cross along it, each passing from one side of the other to its other side.
    Crossing,
    /// They touch along it, their normals parallel all along: tangent, not crossing (a plane on a
    /// cylinder's side, two surfaces of revolution along a circle).
    Touching,
}

/// A branch of a meeting: its curve (whole, as a closed form gives it, or traced) and how the two
/// surfaces stand along it, read over the boxes asked about.
#[derive(Clone,Debug)]
pub struct Branch { pub curve: Curve,pub contact: Contact }

/// What a singular point of a meeting is.
#[derive(Clone,Debug,PartialEq)]
pub enum Singular {
    /// The two touch at the point and meet in nothing else near it (`f` has an extremum of zero there).
    Touch,
    /// Branches cross at it: a saddle of `f` at zero, or two branches of a closed form crossing,
    /// each going out along the directions given.
    Crossing(Vec<V>),
}

/// A meeting's singular point and what it is.
#[derive(Clone,Debug,PartialEq)]
pub struct Point { pub at: V,pub kind: Singular }

/// **What two surfaces share over two boxes of their parameters** (the module's account).
#[derive(Clone,Debug)]
pub struct Meeting {
    /// The two are one surface (within the tolerance), coinciding over the whole region.
    pub same: bool,
    /// Its branches.
    pub branches: Vec<Branch>,
    /// Its singular points: tangencies alone, and where branches cross.
    pub singular: Vec<Point>,
    /// Whether the branches came in closed form (else traced).
    pub closed_form: bool,
}

/// Samples along each side of a box of parameters, and the grid a box's critical points are
/// sought from: well past any feature of the surfaces this kernel builds over one face.
const SIDE: usize = 128;
const GRID: usize = 12;

/// The signed distance from `b` of `a`'s point at `uv`.
fn f(a: &Surface,b: &Surface,uv: Uv) -> f64 { b.implicit(a.point(uv)) }

/// The box of a box of parameters' points in space, sampled and grown by a sixteenth of its
/// diagonal (what a trace and a search keep to).
fn reach(s: &Surface,d: [[f64;2];2]) -> (V,V) {
    let (lo,hi) = super::geom::hull((0..=16).flat_map(|i| (0..=16).map(move |j|
        s.point([d[0][0]+(d[0][1]-d[0][0])*i as f64/16.,d[1][0]+(d[1][1]-d[1][0])*j as f64/16.]))));
    let grow = norm(sub(hi,lo))/16.;
    (lo.map(|x| x-grow),hi.map(|x| x+grow))
}

/// The two boxes' shared box in space.
fn shared(a: (V,V),b: (V,V)) -> (V,V) {
    (std::array::from_fn(|k| a.0[k].max(b.0[k])),std::array::from_fn(|k| a.1[k].min(b.1[k])))
}

/// **Where a box's boundary crosses the other surface**: along each of its four sides, the sign
/// changes of `b`'s signed distance between samples, each bracketed root found (`roots::bracketed_root`).
fn boundary_seeds(a: &Surface,d: [[f64;2];2],b: &Surface) -> Vec<V> {
    let mut out = Vec::new();
    let sides: [(Uv,Uv);4] = [([d[0][0],d[1][0]],[d[0][1],d[1][0]]),([d[0][1],d[1][0]],[d[0][1],d[1][1]]),
        ([d[0][1],d[1][1]],[d[0][0],d[1][1]]),([d[0][0],d[1][1]],[d[0][0],d[1][0]])];
    for (p,q) in sides {
        let at = |s: f64| [p[0]+(q[0]-p[0])*s,p[1]+(q[1]-p[1])*s];
        let mut last = (0.,f(a,b,at(0.)));
        for i in 1..=SIDE {
            let s = i as f64/SIDE as f64;
            let v = f(a,b,at(s));
            if v == 0. { out.push(a.point(at(s))); }
            else if (v < 0.) != (last.1 < 0.) && last.1 != 0. {
                let root = crate::util::roots::bracketed_root(|x| Some(f(a,b,at(x))),last.0,last.1,s,v,1e-12);
                out.push(a.point(at(root)));
            }
            last = (s,v);
        }
    }
    out
}

/// `f`'s gradient and Hessian in `a`'s parameters at `uv`, by central differences of step `h`.
fn jet(a: &Surface,b: &Surface,uv: Uv,h: [f64;2]) -> (f64,[f64;2],[[f64;2];2]) {
    let g = |du: f64,dv: f64| f(a,b,[uv[0]+du,uv[1]+dv]);
    let (f0,up,um,vp,vm) = (g(0.,0.),g(h[0],0.),g(-h[0],0.),g(0.,h[1]),g(0.,-h[1]));
    let grad = [(up-um)/(2.*h[0]),(vp-vm)/(2.*h[1])];
    let huu = (up-2.*f0+um)/(h[0]*h[0]);
    let hvv = (vp-2.*f0+vm)/(h[1]*h[1]);
    let huv = (g(h[0],h[1])-g(h[0],-h[1])-g(-h[0],h[1])+g(-h[0],-h[1]))/(4.*h[0]*h[1]);
    (f0,grad,[[huu,huv],[huv,hvv]])
}

/// **A critical point of `f` classified by its Hessian's rank** (#778 T1): where it is, `f`'s
/// value there, and the rank — 2 an isolated critical point, 1 a point of a critical curve (a
/// Morse–Bott set: two surfaces tangent along a curve, or a fold beside which they cross shallowly),
/// 0 where the two coincide or touch to higher order. `n` and `e` are the Hessian's non-null and
/// null eigendirections in `a`'s parameters (unit in the surface's own metric, so `e` is a way along
/// the surface), and `curvature` its eigenvalue along `n` (`∂²f/∂n²`, per length squared).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Critical { pub uv: Uv,pub value: f64,pub rank: usize,pub n: [f64;2],pub e: [f64;2],pub curvature: f64 }

/// The Hessian's eigenvalues below this part of its largest are none: its rank.
const RANK: f64 = 1e-4;
/// The Hessian's eigenvalues below this part of its largest take no step (`critical_set`'s
/// minimum-norm Newton): beside a critical curve the way along it barely bends, and a step along
/// it would run far out on its rounding.
const STEP_RANK: f64 = 1e-2;

/// `f`'s jet at `uv` in `a`'s own metric there (each parameter scaled by its speed, `D`): its value,
/// its gradient `D⁻¹ g`, its Hessian `D⁻¹ H D⁻¹`, and the speeds.
fn metric_jet(a: &Surface,b: &Surface,uv: Uv,h: [f64;2]) -> (f64,[f64;2],[[f64;2];2],[f64;2]) {
    let (v,g,m) = jet(a,b,uv,h);
    let k = speeds(a,uv);
    (v,[g[0]/k[0],g[1]/k[1]],[[m[0][0]/(k[0]*k[0]),m[0][1]/(k[0]*k[1])],[m[1][0]/(k[1]*k[0]),m[1][1]/(k[1]*k[1])]],k)
}

/// `metric_jet` without the Hessian (five readings of `f`, not nine): its value, gradient and the
/// speeds, what a march along `f = 0` or onto it takes.
fn metric_grad(a: &Surface,b: &Surface,uv: Uv,h: [f64;2]) -> (f64,[f64;2],[f64;2]) {
    let g = |du: f64,dv: f64| f(a,b,[uv[0]+du,uv[1]+dv]);
    let grad = [(g(h[0],0.)-g(-h[0],0.))/(2.*h[0]),(g(0.,h[1])-g(0.,-h[1]))/(2.*h[1])];
    let k = speeds(a,uv);
    (g(0.,0.),[grad[0]/k[0],grad[1]/k[1]],k)
}

/// `a`'s speeds along its two parameters at `uv`: the metric `D` the jets are read in.
fn speeds(a: &Surface,uv: Uv) -> [f64;2] {
    let (_,su,sv) = a.d1(uv);
    [norm(su).max(1e-300),norm(sv).max(1e-300)]
}

/// A symmetric 2 × 2 matrix's eigenvalues, the larger in size first, with their unit eigenvectors.
fn eigen(m: [[f64;2];2]) -> ([f64;2],[[f64;2];2]) {
    let (a,b,c) = (m[0][0],m[0][1],m[1][1]);
    let (mean,dev) = (0.5*(a+c),(0.25*(a-c)*(a-c)+b*b).sqrt());
    let (l1,l2) = (mean+dev,mean-dev);
    let phase = 0.5*(2.*b).atan2(a-c);
    let (s,co) = phase.sin_cos();
    let (v1,v2) = ([co,s],[-s,co]);
    if l1.abs() >= l2.abs() { ([l1,l2],[v1,v2]) } else { ([l2,l1],[v2,v1]) }
}

/// The rank of `f`'s Hessian `m` (in the metric) beside its gradient's scale: eigenvalues below
/// `RANK` of the largest, or below what the differences resolve, are none.
fn rank_of(l: [f64;2],floor: f64) -> usize {
    if l[0].abs() <= floor { 0 } else if l[1].abs() <= RANK*l[0].abs() { 1 } else { 2 }
}

/// `uv` brought into the box `d` round each parameter whose period the box spans whole (a
/// cylinder's or a torus's angle): a critical curve lying along the period's seam is inside it.
fn wrapped(a: &Surface,d: [[f64;2];2],uv: Uv) -> Uv {
    let p = a.periods();
    std::array::from_fn(|k| match p[k] {
        Some(per) if d[k][1]-d[k][0] >= per*(1.-1e-12) => super::geom::around(uv[k],d[k][0],per),
        _ => uv[k],
    })
}

/// **The critical set of `f` over the box, by rank** (#778 T1, the meeting's Morse analysis in
/// `a`'s parameters): from each point of a grid, minimum-norm Newton steps on `∇f = 0` — the
/// Hessian inverted on its non-null eigendirections alone, so a start settles onto a critical curve
/// (where it has rank 1) as well as at a point — kept where it settles inside the box, each once,
/// classified by the Hessian's rank there with its eigendirections.
pub fn critical_set(a: &Surface,d: [[f64;2];2],b: &Surface,tol: f64) -> Vec<Critical> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    let h = [w[0]*1e-5,w[1]*1e-5];
    let size = { let (lo,hi) = reach(a,d); norm(sub(hi,lo)).max(1e-300) };
    let mut found: Vec<Critical> = Vec::new();
    for i in 0..GRID { for j in 0..GRID {
        let mut uv = [d[0][0]+w[0]*(i as f64+0.5)/GRID as f64,d[1][0]+w[1]*(j as f64+0.5)/GRID as f64];
        let mut settled = false;
        for _ in 0..40 {
            let (_,g,m,k) = metric_jet(a,b,uv,h);
            let (l,vs) = eigen(m);
            let floor = 1e-9/size.max(1e-300);
            if l[0].abs() <= floor { break }
            // the step on the eigendirections well clear of null alone (a way along which `f` barely
            // bends steps nowhere: a critical curve's), held within an eighth of the box
            let mut step = [0.;2];
            for q in 0..2 {
                if l[q].abs() <= STEP_RANK*l[0].abs() { continue }
                let c = (g[0]*vs[q][0]+g[1]*vs[q][1])/l[q];
                step[0] += c*vs[q][0];
                step[1] += c*vs[q][1];
            }
            let long = step[0].hypot(step[1]);
            if long > size/8. { step = [step[0]*size/8./long,step[1]*size/8./long] }
            // back to the parameters
            let raw = [step[0]/k[0],step[1]/k[1]];
            // past the box's edge held on it (a touch along its edge, a pipe's end, settles there)
            let next = wrapped(a,d,[uv[0]-raw[0],uv[1]-raw[1]]);
            uv = [next[0].clamp(d[0][0],d[0][1]),next[1].clamp(d[1][0],d[1][1])];
            if distance(a.point(uv),a.point(next)) > size/4. { break }
            if raw[0].abs() <= 1e-9*w[0] && raw[1].abs() <= 1e-9*w[1] { settled = true; break }
        }
        if !settled { continue }
        let p = a.point(uv);
        let (value,g,m,_) = metric_jet(a,b,uv,h);
        // settled on the non-null ways alone, `f` may still slope along the null one: a fold (where
        // a plane leaning a hair crosses a vault's crown), no critical point — read by the level
        // curves, not as a ridge
        if g[0].hypot(g[1]) > 1e-7 { continue }
        let (l,vs) = eigen(m);
        let rank = rank_of(l,1e-9/size.max(1e-300));
        // a point of a critical curve met before stands for the rest of it near there; a point once
        let near = if rank == 1 { size/GRID as f64 } else { 1e3*tol.max(1e-9*size) };
        if found.iter().any(|c| distance(a.point(c.uv),p) <= near && c.rank == rank) { continue }
        found.push(Critical {uv,value,rank,n:vs[0],e:vs[1],curvature:l[0]});
    } }
    found
}

/// **The critical points of `f` over the box** (the meeting's Morse analysis in `a`'s parameters):
/// `critical_set`'s, with `f`'s value at each: zero at a singular point of the meeting.
pub fn critical_points(a: &Surface,d: [[f64;2];2],b: &Surface,tol: f64) -> Vec<(Uv,f64)> {
    critical_set(a,d,b,tol).into_iter().map(|c| (c.uv,c.value)).collect()
}

/// **A ridge: a critical curve of `f` traced** (#778 T2): the curve `∂f/∂n = 0` continued along the
/// Hessian's null way `e` from a rank-1 critical point, both ways to the box's edge or back to its
/// start. At each point, its parameters on `a`, `f`'s value there (the fold's), its way across `n`
/// (in the parameters, unit in the metric) and `f`'s curvature across, `∂²f/∂n²`.
#[derive(Clone,Debug)]
pub struct Ridge { pub uv: Vec<Uv>,pub value: Vec<f64>,pub across: Vec<[f64;2]>,pub curvature: Vec<f64> }

/// How far from nought a ridge's fold may stand and still be traced: where the crossings beside it
/// meet steeper than this the ordinary trace follows them (`ssi::SHALLOW`, a tenfold margin).
const STEEP: f64 = 10.*super::ssi::SHALLOW;

/// **The ridges of `f` over the box** (#778 T2): from each rank-1 critical point whose fold `f`
/// lies within reach of nought (crossings beside it no steeper than `STEEP`: `|f| ≤ STEEP² /
/// (2|κ|)`), the ridge traced by predicting a step along `e` and settling across it by Newton on
/// `∂f/∂n`, each ridge once. A rank-0 zero is refused, typed, naming the surfaces and the point: the
/// two coincide there, or touch to higher order (#717).
pub fn ridges(a: &Surface,d: [[f64;2];2],b: &Surface,tol: f64) -> Result<Vec<Ridge>,String> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    let h = [w[0]*1e-5,w[1]*1e-5];
    let size = { let (lo,hi) = reach(a,d); norm(sub(hi,lo)).max(1e-300) };
    let inside = |uv: Uv| uv[0] >= d[0][0] && uv[0] <= d[0][1] && uv[1] >= d[1][0] && uv[1] <= d[1][1];
    let step = (a.feature().min(b.feature()).min(size)/96.).max(size*1e-4);
    let mut out: Vec<Ridge> = Vec::new();
    let set = critical_set(a,d,b,tol);
    if super::debug() { for c in &set { eprintln!("brep: critical {:?} on a {} against a {}",c,a.kind(),b.kind()) } }
    for c in set {
        if c.rank == 0 && c.value.abs() <= 8.*tol {
            let p = a.point(c.uv);
            return Err(format!("a {} and a {} coincide or touch to higher order at [{:.4}, {:.4}, {:.4}]: refused (#717)",a.kind(),b.kind(),p[0],p[1],p[2]))
        }
        if c.rank != 1 || c.value.abs() > STEEP*STEEP/(2.*c.curvature.abs()) { continue }
        let p = a.point(c.uv);
        if out.iter().any(|r| r.uv.iter().any(|&q| distance(a.point(q),p) <= 2.*step)) { continue }
        // settled across: Newton on ∂f/∂n along n
        let settle = |uv: Uv,n: [f64;2]| -> Option<(Uv,f64,[f64;2],[f64;2],f64)> {
            let mut uv = uv;
            for _ in 0..30 {
                let (_,g,m,k) = metric_jet(a,b,uv,h);
                let (l,vs) = eigen(m);
                // the non-null way nearest the one before, so it turns smoothly
                let nn = if vs[0][0]*n[0]+vs[0][1]*n[1] >= 0. { vs[0] } else { [-vs[0][0],-vs[0][1]] };
                if !(l[0].abs() > 0.) { return None }
                let slope = g[0]*nn[0]+g[1]*nn[1];
                let t = -slope/l[0];
                uv = [uv[0]+t*nn[0]/k[0],uv[1]+t*nn[1]/k[1]];
                if t.abs() <= 1e-12*size { break }
            }
            let (v,g,m,_) = metric_jet(a,b,uv,h);
            let (l,vs) = eigen(m);
            // the critical curve ends where f no longer stays flat along it: `∂f/∂e` off nought
            let along = (g[0]*vs[1][0]+g[1]*vs[1][1]).abs();
            if l[1].abs() > 1e3*RANK*l[0].abs() && along > 1e-6 { return None }
            let nn = if vs[0][0]*n[0]+vs[0][1]*n[1] >= 0. { vs[0] } else { [-vs[0][0],-vs[0][1]] };
            Some((uv,v,nn,vs[1],l[0]))
        };
        let Some((uv0,v0,n0,e0,k0)) = settle(c.uv,c.n) else { continue };
        let mut halves: Vec<Vec<(Uv,f64,[f64;2],f64)>> = Vec::new();
        let mut closed = false;
        for sense in [1.,-1.] {
            let mut pts = vec![(uv0,v0,n0,k0)];
            let (mut uv,mut n,mut e) = (uv0,n0,[e0[0]*sense,e0[1]*sense]);
            for _ in 0..20_000 {
                if !crate::util::meter::charge(crate::util::meter::Stage::Trace,1,|| (a.point(uv),format!("{} × {} (a ridge)",a.kind(),b.kind()))) { break }
                let k = speeds(a,uv);
                let delta = [step*e[0]/k[0],step*e[1]/k[1]];
                let guess = wrapped(a,d,[uv[0]+delta[0],uv[1]+delta[1]]);
                if !inside(guess) {
                    // the last step shortened onto the box's edge, so the ridge ends on it, where the
                    // box's boundary crosses the branches it holds
                    let mut t = 1f64;
                    for q in 0..2 {
                        if delta[q] > 0. && uv[q]+delta[q] > d[q][1] { t = t.min((d[q][1]-uv[q])/delta[q]) }
                        if delta[q] < 0. && uv[q]+delta[q] < d[q][0] { t = t.min((d[q][0]-uv[q])/delta[q]) }
                    }
                    if t > 1e-9 {
                        let edge = [uv[0]+t*delta[0],uv[1]+t*delta[1]];
                        if let Some((x,v,nn,_,kk)) = settle(edge,n) {
                            let x = [x[0].clamp(d[0][0],d[0][1]),x[1].clamp(d[1][0],d[1][1])];
                            pts.push((x,v,nn,kk));
                        }
                    }
                    break
                }
                let Some((next,v,nn,ne,kk)) = settle(guess,n) else { break };
                let next = wrapped(a,d,next);
                // settled past the box's edge: held on it where only a hair past (a ridge along the
                // edge), else its end
                let held = [next[0].clamp(d[0][0],d[0][1]),next[1].clamp(d[1][0],d[1][1])];
                let past = distance(a.point(held),a.point(next));
                if past > 1e-6*size {
                    if past <= step { pts.push((held,v,nn,kk)) }
                    break
                }
                let next = held;
                if pts.len() > 3 && distance(a.point(next),a.point(uv0)) <= 0.75*step { closed = true; break }
                e = if ne[0]*e[0]+ne[1]*e[1] >= 0. { ne } else { [-ne[0],-ne[1]] };
                n = nn;
                uv = next;
                pts.push((uv,v,nn,kk));
            }
            halves.push(pts);
            if closed { break }
        }
        let mut pts: Vec<(Uv,f64,[f64;2],f64)> = halves.get(1).map_or(Vec::new(),|b| b.iter().rev().copied().collect());
        if !pts.is_empty() { pts.pop(); }
        pts.extend(halves[0].iter().copied());
        if pts.len() < 2 { continue }
        out.push(Ridge {uv:pts.iter().map(|x| x.0).collect(),value:pts.iter().map(|x| x.1).collect(),
            across:pts.iter().map(|x| x.2).collect(),curvature:pts.iter().map(|x| x.3).collect()});
    }
    Ok(out)
}

/// **The meeting traced as `f = 0` in `a`'s own parameters** (#778 T3): from each seed, a march
/// along the level curve of `f = b ∘ a` — its tangent square to `∇f` in `a`'s metric, each step
/// predicted along it and pulled back by Newton along `∇f` — to the box's edge or back to its start.
/// One scalar equation in two unknowns: no cross product of the two normals, so it holds however
/// shallow the two cross, down to `f`'s gradient at its rounding. Each curve through the points it
/// reached (`nurbs::interpolate`, cubic by chord length), a crossing. Seeds already on a curve found
/// are passed by.
pub fn level_curves(a: &Surface,d: [[f64;2];2],b: &Surface,seeds: &[V],tol: f64) -> Vec<Branch> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    let h = [w[0]*1e-6,w[1]*1e-6];
    let size = { let (lo,hi) = reach(a,d); norm(sub(hi,lo)).max(1e-300) };
    // (fine enough that the cubic through its points stands within 1e-9 of the size off the curve)
    let step = (a.feature().min(b.feature()).min(size)/192.).max(size*1e-5);
    let inside = |uv: Uv| uv[0] >= d[0][0]-1e-12*w[0] && uv[0] <= d[0][1]+1e-12*w[0] && uv[1] >= d[1][0]-1e-12*w[1] && uv[1] <= d[1][1]+1e-12*w[1];
    // pulled onto `f = 0` along its gradient in the metric
    let onto = |uv: Uv| -> Option<Uv> {
        let mut uv = uv;
        for _ in 0..40 {
            let (v,g,k) = metric_grad(a,b,uv,h);
            let gg = g[0]*g[0]+g[1]*g[1];
            if !(gg > 0.) { return None }
            let t = v/gg;
            uv = [uv[0]-t*g[0]/k[0],uv[1]-t*g[1]/k[1]];
            if v.abs() <= 1e-3*tol { break }
        }
        (f(a,b,uv).abs() <= tol).then_some(uv)
    };
    let mut found: Vec<Vec<V>> = Vec::new();
    let near_found = |found: &Vec<Vec<V>>,p: V| found.iter().any(|c| c.windows(2).any(|s| crate::util::space::segment_distance(p,s[0],s[1]) <= 0.25*step));
    for &seed in seeds {
        let uv0 = a.inverse(seed);
        let Some(uv0) = onto(uv0) else { continue };
        if !inside(uv0) || near_found(&found,a.point(uv0)) { continue }
        let mut halves: Vec<Vec<V>> = Vec::new();
        let mut closed = false;
        for sense in [1.,-1.] {
            let mut pts = vec![a.point(uv0)];
            let mut uv = uv0;
            // the step shortened where the level curve turns sharply in the parameters (a parabola's
            // vertex where a plane leans a hair to a vault's crown) and let out again after
            let mut hstep = step;
            for _ in 0..100_000 {
                if !crate::util::meter::charge(crate::util::meter::Stage::Trace,1,|| (a.point(uv),format!("{} × {} (a level curve)",a.kind(),b.kind()))) { break }
                let (_,g,k) = metric_grad(a,b,uv,h);
                let gn = g[0].hypot(g[1]);
                if !(gn > 0.) { break }
                let t = [-g[1]*sense/gn,g[0]*sense/gn];
                // the tangent a step on, read before the step is taken: more than a tenth of a radian
                // off this one, the step halved
                let turned = |hs: f64| -> bool {
                    let ahead = [uv[0]+hs*t[0]/k[0],uv[1]+hs*t[1]/k[1]];
                    let Some(q) = onto(ahead) else { return true };
                    let (_,g2,_) = metric_grad(a,b,q,h);
                    let n2 = g2[0].hypot(g2[1]);
                    !(n2 > 0.) || (g[0]*g2[0]+g[1]*g2[1])/(gn*n2) < 0.995
                };
                while hstep > 1e-7*size && turned(hstep) { hstep *= 0.5 }
                let here = hstep;
                hstep = (hstep*1.5).min(step);
                let delta = [here*t[0]/k[0],here*t[1]/k[1]];
                let mut guess = [uv[0]+delta[0],uv[1]+delta[1]];
                let mut last = false;
                if !inside(guess) {
                    let mut s = 1f64;
                    for q in 0..2 {
                        if delta[q] > 0. && guess[q] > d[q][1] { s = s.min((d[q][1]-uv[q])/delta[q]) }
                        if delta[q] < 0. && guess[q] < d[q][0] { s = s.min((d[q][0]-uv[q])/delta[q]) }
                    }
                    guess = [uv[0]+s*delta[0],uv[1]+s*delta[1]];
                    last = true;
                }
                let Some(next) = onto(guess) else { break };
                let next = [next[0].clamp(d[0][0],d[0][1]),next[1].clamp(d[1][0],d[1][1])];
                let p = a.point(next);
                if pts.len() > 3 && distance(p,a.point(uv0)) <= 0.75*step { closed = true; break }
                pts.push(p);
                uv = next;
                if last { break }
            }
            halves.push(pts);
            if closed { break }
        }
        let mut pts: Vec<V> = halves.get(1).map_or(Vec::new(),|b| b.iter().rev().copied().collect());
        if !pts.is_empty() { pts.pop(); }
        pts.extend(halves[0].iter().copied());
        if closed { pts.push(pts[0]) }
        if pts.len() >= 2 { found.push(pts) }
    }
    found.into_iter().filter_map(|pts| fitted(&pts).map(|curve| Branch {curve,contact:Contact::Crossing})).collect()
}

/// The cubic (or lower, for fewer points) through `pts` at their chord lengths: a traced branch's
/// curve. None for fewer than two points, or none apart.
fn fitted(pts: &[V]) -> Option<Curve> {
    if pts.len() < 2 { return None }
    let mut t = vec![0.];
    for k in 1..pts.len() { t.push(t[k-1]+distance(pts[k-1],pts[k])) }
    let total = t[t.len()-1];
    if !(total > 0.) { return None }
    let t: Vec<f64> = t.iter().map(|x| x/total).collect();
    let degree = if pts.len() >= 4 { 3 } else { pts.len()-1 };
    super::nurbs::interpolate(pts,&t,degree).map(|c| Curve::BSpline(std::sync::Arc::new(c)))
}

/// Whether `p` (on `s`) lies in the box `d` of its parameters, a periodic one read round its
/// period, a tolerance's reach past its edges counted in.
fn in_box(s: &Surface,d: [[f64;2];2],p: V,tol: f64) -> bool {
    let uv = s.inverse(p);
    let periods = s.periods();
    let (_,su,sv) = s.d1(uv);
    let slack = [tol/norm(su).max(1e-300),tol/norm(sv).max(1e-300)];
    (0..2).all(|k| {
        let mut x = uv[k];
        if let Some(per) = periods[k] { x = super::geom::around(x,d[k][0],per) }
        x >= d[k][0]-slack[k] && x <= d[k][1]+slack[k]
    })
}

/// The box `d` of `s`'s parameters within those its points are defined over: a B-spline net's
/// domain, which its evaluation clamps to (a face's box read a margin past it would walk a march
/// along one clamped point).
pub fn within_domain(s: &Surface,d: [[f64;2];2]) -> [[f64;2];2] {
    match s {
        Surface::BSpline(_,n) => { let dom = n.domain(); std::array::from_fn(|k| [d[k][0].max(dom[k][0]),d[k][1].min(dom[k][1])]) }
        _ => d,
    }
}

/// Whether `s`'s signed distance is in closed form (`Surface::implicit`'s exact kinds), smooth
/// enough to difference twice: what a meeting's ridges are read through (#778 T1).
pub fn exact_distance(s: &Surface) -> bool { matches!(s,Surface::Plane(_) | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..)) }

/// Whether `p` lies on one of `branches`: within `8 tol / sin θ` of it, `θ` the two surfaces'
/// angle there — a point of a crossing at that angle is placed only so well (#735 A3's
/// `on_meeting`), and a touch's to the square root of the tolerance, beside its own radius.
pub fn on_a_branch(a: &Surface,b: &Surface,branches: &[Branch],p: V,tol: f64) -> bool {
    let sine = norm(cross(unit_or(a.gradient(p)),unit_or(b.gradient(p)))).max(1e-6);
    branches.iter().any(|br| {
        // (a point where the two stand tangent within a few of a ridge's steps of a touch is that
        // touch: a ridge's step is a forty-eighth of their finer feature)
        let feature = a.feature().min(b.feature()).min(1e6);
        let reach = match br.contact {
            Contact::Crossing => 8.*tol/sine,
            Contact::Touching if sine < super::ssi::SHALLOW => (8.*tol*feature).sqrt().max(feature/12.),
            Contact::Touching => (8.*tol*feature).sqrt().max(8.*tol),
        };
        branch_distance(&br.curve,p) <= reach
    })
}

/// How far `p` stands from a branch's curve: from a polyline of 512 chords along it, a closed
/// one's too (whose foot a Newton search can settle on the wrong side of its seam), the nearer of
/// that and the curve's own foot.
fn branch_distance(c: &Curve,p: V) -> f64 {
    let [t0,t1] = match c { Curve::BSpline(s) => s.domain(),_ => return distance(c.point(c.inverse(p)),p) };
    let pts: Vec<V> = (0..=512).map(|k| c.point(t0+(t1-t0)*k as f64/512.)).collect();
    let chords = pts.windows(2).map(|w| crate::util::space::segment_distance(p,w[0],w[1])).fold(f64::INFINITY,f64::min);
    chords.min(distance(c.point(c.inverse(p)),p))
}

/// **The branches a ridge holds** (#778 T2, T3): the stretches where its fold is within `tol` of
/// nought, a touch (`Contact::Touching`); and where the fold has the sign opposite `f`'s curvature
/// across it, the two crossings beside it, at each point of the ridge the roots of `f` along `n`
/// nearest it either side, isolated with a bound (`util::isolate`, #777 M1) over the reach
/// `3 √(2|f|/|κ|)` — however shallow they cross, well conditioned, the fold solved in its two-root
/// form, not linearised. Each run of points a curve through them (`nurbs::interpolate`, cubic,
/// by chord length).
pub fn ridge_branches(a: &Surface,d: [[f64;2];2],b: &Surface,r: &Ridge,tol: f64) -> Vec<Branch> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    // (the box's edges a rounding's width wide: a station on one stays on it)
    let inside = |uv: Uv| (0..2).all(|k| uv[k] >= d[k][0]-1e-9*w[k] && uv[k] <= d[k][1]+1e-9*w[k]);
    let mut out: Vec<Branch> = Vec::new();
    // the touch: runs of the ridge where the fold is within `tol` of nought
    let mut run: Vec<V> = Vec::new();
    let mut runs: Vec<Vec<V>> = Vec::new();
    for (k,&uv) in r.uv.iter().enumerate() {
        if r.value[k].abs() <= tol { run.push(a.point(uv)) } else if !run.is_empty() { runs.push(std::mem::take(&mut run)) }
    }
    if !run.is_empty() { runs.push(run) }
    for pts in runs { if let Some(c) = fitted(&pts) { out.push(Branch {curve:c,contact:Contact::Touching}) } }
    // the crossings beside it: each side's root along n nearest the ridge
    let mut sides: [Vec<V>;2] = [Vec::new(),Vec::new()];
    let flush = |sides: &mut [Vec<V>;2],out: &mut Vec<Branch>| for s in sides.iter_mut() {
        if let Some(c) = fitted(s) { out.push(Branch {curve:c,contact:Contact::Crossing}) }
        s.clear();
    };
    for (k,&uv) in r.uv.iter().enumerate() {
        let (v,kappa,n) = (r.value[k],r.curvature[k],r.across[k]);
        if !(v*kappa < 0.) || v.abs() <= tol { flush(&mut sides,&mut out); continue }
        let reach = 3.*(2.*v.abs()/kappa.abs()).sqrt();
        let sp = speeds(a,uv);
        // (round a period the box spans whole: a crossing beside a ridge on its seam is inside)
        let at = |t: f64| wrapped(a,d,[uv[0]+t*n[0]/sp[0],uv[1]+t*n[1]/sp[1]]);
        let g = |t: f64| f(a,b,at(t));
        let dt = 1e-6*reach.max(1e-12);
        let dg = |t: f64| (g(t+dt)-g(t-dt))/(2.*dt);
        // f along n is the fold's parabola to second order: its slopes and curvature bounded
        // over the reach by its own, doubled, and the variation read across it
        let samples: Vec<f64> = (0..=16).map(|i| { let t = -reach+2.*reach*i as f64/16.; (g(t+dt)-2.*g(t)+g(t-dt))/(dt*dt) }).collect();
        let bend = samples.iter().fold(kappa.abs(),|m,&x| m.max(x.abs()))*2.;
        let d0 = dg(0.).abs();
        let bound = |x0: f64,x1: f64| [bend*x0.abs().max(x1.abs())+d0,bend];
        let roots = crate::util::isolate::roots(&g,&dg,&bound,[-reach,reach],tol*1e-3,&|t| g(t).abs() <= tol).unwrap_or_default();
        let plus = roots.iter().filter(|x| x.at > 0. && !x.touch).map(|x| x.at).fold(f64::INFINITY,f64::min);
        let minus = roots.iter().filter(|x| x.at < 0. && !x.touch).map(|x| x.at).fold(f64::NEG_INFINITY,f64::max);
        if !(plus.is_finite() && minus.is_finite()) || !inside(at(plus)) || !inside(at(minus)) { flush(&mut sides,&mut out); continue }
        sides[0].push(a.point(at(plus)));
        sides[1].push(a.point(at(minus)));
    }
    flush(&mut sides,&mut out);
    out
}

/// From a critical point of `f` that is no zero of it, the points of the meeting reached walking
/// down (or up) `|f|`'s gradient toward zero in eight ways out: a branch closing round it is met so.
fn descents(a: &Surface,d: [[f64;2];2],b: &Surface,uv: Uv,tol: f64) -> Vec<V> {
    let w = [d[0][1]-d[0][0],d[1][1]-d[1][0]];
    let h = [w[0]*1e-5,w[1]*1e-5];
    let mut out = Vec::new();
    for k in 0..8 {
        let ang = std::f64::consts::TAU*k as f64/8.;
        let mut x = [uv[0]+0.01*w[0]*ang.cos(),uv[1]+0.01*w[1]*ang.sin()];
        let sign = f(a,b,uv).signum();
        let mut last = f(a,b,x);
        for _ in 0..400 {
            if !(x[0] >= d[0][0] && x[0] <= d[0][1] && x[1] >= d[1][0] && x[1] <= d[1][1]) { break }
            let (fx,g,_) = jet(a,b,x,h);
            if fx.abs() <= tol || fx.signum() != sign {
                // bracketed between the last step and this one: bisect along the gradient line
                out.push(a.point(x));
                break
            }
            let gn = (g[0]*g[0]/(w[0]*w[0])+g[1]*g[1]/(w[1]*w[1])).sqrt().max(1e-300);
            // a step a fiftieth of the box across, scaled to the gradient, down |f|
            let s = 0.02/gn;
            x = [x[0]-sign*s*g[0]/(w[0]*w[0])*w[0]*w[0],x[1]-sign*s*g[1]/(w[1]*w[1])*w[1]*w[1]];
            let fx = f(a,b,x);
            if fx.abs() >= last.abs() && fx.signum() == sign { break }
            last = fx;
        }
    }
    out
}

/// How the two stand along a curve over the shared box: touching where `|∇a × ∇b|` stays under a
/// thousandth all along its points within it, else crossing.
fn contact(a: &Surface,b: &Surface,c: &Curve,(lo,hi): (V,V)) -> Contact {
    let span = match c.period() { Some(p) => [0.,p],None => match c {
        Curve::Line {p,d} => {
            // the stretch within the box: a line's
            let (mut t0,mut t1) = (f64::NEG_INFINITY,f64::INFINITY);
            for k in 0..3 {
                if d[k].abs() < 1e-300 { continue }
                let (x,y) = ((lo[k]-p[k])/d[k],(hi[k]-p[k])/d[k]);
                t0 = t0.max(x.min(y)); t1 = t1.min(x.max(y));
            }
            if !(t1 > t0) { return Contact::Crossing }
            [t0,t1]
        }
        Curve::BSpline(s) => s.domain(),
        Curve::Traced(t) => [0.,(t.pts.len()-1) as f64],
        _ => [0.,1.],
    } };
    let mut any = false;
    for j in 0..=64 {
        let p = c.point(span[0]+(span[1]-span[0])*j as f64/64.);
        if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) { continue }
        any = true;
        if norm(cross(unit_or(a.gradient(p)),unit_or(b.gradient(p)))) > 1e-3 { return Contact::Crossing }
    }
    if any { Contact::Touching } else { Contact::Crossing }
}

/// **Why a meeting's march answered nothing** (#780 R3): refused as shallow (`ssi::Shallow`), which
/// the meeting is read past (`marched`), or for any other reason, which stands.
#[derive(Debug)]
pub(crate) enum Unmarched { Shallow(super::ssi::Shallow),Failed(String) }

impl From<super::ssi::Shallow> for Unmarched { fn from(e: super::ssi::Shallow) -> Unmarched { Unmarched::Shallow(e) } }
impl From<String> for Unmarched { fn from(e: String) -> Unmarched { Unmarched::Failed(e) } }
impl From<Unmarched> for String {
    fn from(e: Unmarched) -> String { match e { Unmarched::Shallow(e) => e.into(),Unmarched::Failed(e) => e } }
}

/// **A meeting marched, and read past a march refused as shallow** (#778 T3, T4): the one fallback
/// for every meeting with no closed form (#780 R3) — ridges, then the trace, then level curves for
/// what it refuses. `march(known, past)` traces the meeting of `a` over `da` and `b` over `db`
/// (`sides`) through its seeds, the branches `known` already held (a march ends where it comes to
/// one); `past` once a refusal has been read past, where a seed whose march is refused again stands
/// where the branches read the meeting and is passed by (`traced_alone`).
///
/// It is marched first with the meeting's ridges where the caller read them before (`ridged`),
/// else with none. Refused as shallow, the meeting is read from its critical set over the `sides`
/// and `seeds` the caller then gives (`past_inputs`, asked only here: the seeds clear of `ridged`
/// already): its ridges, where the caller had not read them — over the parameters of the side
/// `ridge_order` puts first, the other's where those hold none — and the level curves `f = 0`
/// through the seeds no new branch holds, in either's parameters; then it is marched again with
/// them known. Refused as before where that reads no branch the march did not know. The march's
/// curves, and every branch known.
pub(crate) fn marched<'s>(ridged: Option<Vec<Branch>>,tol: f64,past_inputs: impl FnOnce() -> ([(&'s Surface,[[f64;2];2]);2],Vec<V>),
    march: &mut dyn FnMut(&[Branch],bool) -> Result<Vec<Curve>,Unmarched>) -> Result<(Vec<Curve>,Vec<Branch>),String> {
    let read = ridged.is_some();
    let mut branches = ridged.unwrap_or_default();
    let refused = match march(&branches,false) {
        Ok(t) => return Ok((t,branches)),
        Err(Unmarched::Shallow(e)) => e,
        Err(Unmarched::Failed(e)) => return Err(e),
    };
    let known = branches.len();
    let ([(a,da),(b,db)],seeds) = past_inputs();
    if !read {
        for (s,d,o) in ridge_order(a,da,b,db) {
            for r in ridges(s,d,o,tol)? { branches.extend(ridge_branches(s,d,o,&r,tol)) }
            if !branches.is_empty() { break }
        }
    }
    let free: Vec<V> = seeds.into_iter().filter(|&p| !on_a_branch(a,b,&branches[known..],p,tol)).collect();
    for (s,d,o) in [(a,da,b),(b,db,a)] {
        let level = level_curves(s,d,o,&free,tol);
        if !level.is_empty() { branches.extend(level); break }
    }
    if branches.len() == known { return Err(refused.into()) }
    if super::debug() {
        eprintln!("brep: a meeting read from its ridges and level curves: {} branches",branches.len());
        for br in &branches { if let Curve::BSpline(s) = &br.curve { let [t0,t1] = s.domain();
            eprintln!("brep:   {:?} from {:?} to {:?}",br.contact,br.curve.point(t0),br.curve.point(t1)) } }
        for s in &free { eprintln!("brep:   seed {s:?}") }
    }
    let t = march(&branches,true)?;
    Ok((t,branches))
}

/// The sides a meeting's ridges are read over, in the order they are tried: first the one whose
/// partner's distance is exact (a plane's, a quadric's, a torus's) where only one's is — a sheet's
/// is a numerical foot, too rough to difference twice — each with its box and its partner.
pub(crate) fn ridge_order<'s>(a: &'s Surface,da: [[f64;2];2],b: &'s Surface,db: [[f64;2];2]) -> [(&'s Surface,[[f64;2];2],&'s Surface);2] {
    if exact_distance(a) && !exact_distance(b) { [(b,db,a),(a,da,b)] } else { [(a,da,b),(b,db,a)] }
}

/// **Each seed traced alone, past a refusal** (`marched`): one on a curve `found` already holds
/// passed by, and one whose march is refused as shallow again — standing where the ridges and level
/// curves read the meeting (a branch arriving tangent at a touch) — passed by too.
pub(crate) fn traced_alone(seeds: &[V],found: &mut Vec<Curve>,trace: &mut dyn FnMut(V) -> Result<Vec<Curve>,super::ssi::Shallow>,
    tol: f64) {
    for &p in seeds {
        if found.iter().any(|c| distance(c.point(c.inverse(p)),p) <= 8.*tol) { continue }
        if let Ok(more) = trace(p) { found.extend(more) }
    }
}

/// **The meeting of `a` over the box `da` of its parameters and `b` over `db`**, to `tol` (the
/// module's account).
pub fn meet(a: &Surface,da: [[f64;2];2],b: &Surface,db: [[f64;2];2],tol: f64) -> Result<Meeting,String> { meet_with(a,da,b,db,tol,true) }

/// `meet`, its closed forms taken where `closed_forms`, else bypassed: the general method alone,
/// what each closed form is held to (#778 T5). An argument, not a switch, so nothing else changes.
pub fn meet_with(a: &Surface,da: [[f64;2];2],b: &Surface,db: [[f64;2];2],tol: f64,closed_forms: bool) -> Result<Meeting,String> {
    if super::ssi::same(a,b,tol) { return Ok(Meeting {same:true,branches:Vec::new(),singular:Vec::new(),closed_form:true}) }
    let (ra,rb) = (reach(a,da),reach(b,db));
    let (lo,hi) = shared(ra,rb);
    let mut singular: Vec<Point> = Vec::new();
    // the Morse analysis in each surface's parameters: the critical points of the other's distance
    let mut critical: Vec<(V,bool)> = Vec::new();
    for (s,d,o) in [(a,da,b),(b,db,a)] {
        for (uv,value) in critical_points(s,d,o,tol) {
            let p = s.point(uv);
            if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) { continue }
            critical.push((p,value.abs() <= 8.*tol));
        }
    }
    for &(p,zero) in &critical {
        if !zero || singular.iter().any(|q| distance(q.at,p) <= 1e3*tol) { continue }
        match super::ssi::touch(a,b,p) {
            Some(t) if t.branches.is_empty() => singular.push(Point {at:p,kind:Singular::Touch}),
            Some(t) => singular.push(Point {at:p,kind:Singular::Crossing(t.branches)}),
            // degenerate to second order: the two touch along a curve through it, said by its branch
            None => {}
        }
    }
    let start = crate::util::clock::Instant::now();
    let met = if closed_forms { intersect(a,b,tol) } else { Ssi::Traced };
    crate::util::meter::meeting(a.kind(),b.kind(),!matches!(met,Ssi::Traced),start.elapsed());
    // the ridges of the meeting, in `a`'s parameters (#778 T2, T3): its touches along curves, and
    // the crossings beside them however shallow, read from the critical set — the general road
    let mut from_ridges: Vec<Branch> = Vec::new();
    if matches!(met,Ssi::Traced) {
        // over the parameters of the side read first (`ridge_order`)
        let [(s,d,o),_] = ridge_order(a,da,b,db);
        for r in ridges(s,d,o,tol)? { from_ridges.extend(ridge_branches(s,d,o,&r,tol)) }
    }
    let (curves,closed_form) = match met {
        Ssi::Same => return Ok(Meeting {same:true,branches:Vec::new(),singular:Vec::new(),closed_form:true}),
        Ssi::Curves(cs) => (cs,true),
        Ssi::Traced => {
            let mut seeds = boundary_seeds(a,da,b);
            seeds.extend(boundary_seeds(b,db,a));
            for (s,d,o) in [(a,da,b),(b,db,a)] {
                for (uv,value) in critical_points(s,d,o,tol) {
                    if value.abs() > 8.*tol { seeds.extend(descents(s,d,o,uv,tol)); }
                }
            }
            // a seed where they touch traces nothing; the touch is a singular point; and a seed on a
            // branch a ridge holds is that branch
            let touch_points: Vec<V> = singular.iter().map(|q| q.at).collect();
            seeds.retain(|&p| !touch_points.iter().any(|&q| distance(p,q) <= 1e3*tol));
            if super::debug() { for &p in &seeds {
                let off = from_ridges.iter().map(|br| distance(br.curve.point(br.curve.inverse(p)),p)).fold(f64::INFINITY,f64::min);
                eprintln!("brep: seed {p:?}: {off:e} from the ridges' {} branches, sine {:e}, on {}",from_ridges.len(),
                    norm(cross(unit_or(a.gradient(p)),unit_or(b.gradient(p)))),on_a_branch(a,b,&from_ridges,p,tol));
            } }
            seeds.retain(|&p| !on_a_branch(a,b,&from_ridges,p,tol));
            // the meeting over both boxes: a seed on one's edge outside the other's box is none of it
            seeds.retain(|&p| in_box(a,da,p,tol) && in_box(b,db,p,tol));
            // marched through the seeds, and read past a march refused as shallow (`marched`)
            let ridged = from_ridges.len();
            let (traced,branches) = marched(Some(from_ridges),tol,|| ([(a,da),(b,db)],seeds.clone()),&mut |known,past| {
                let mut traced = if past {
                    // (the seeds are clear of the ridges' branches: of the level curves' too)
                    let seeds: Vec<V> = seeds.iter().copied().filter(|&p| !on_a_branch(a,b,&known[ridged..],p,tol)).collect();
                    let mut found = Vec::new();
                    traced_alone(&seeds,&mut found,&mut |p| super::ssi::trace(a,b,&[p],lo,hi,tol),tol);
                    found
                } else { super::ssi::trace(a,b,&seeds,lo,hi,tol)? };
                let curves: Vec<Curve> = known.iter().map(|br| br.curve.clone()).collect();
                // what no seed reached, proved absent cell by cell between analytic surfaces
                if super::ssi::searchable(a) && super::ssi::searchable(b) {
                    let mut more = 0;
                    // a ridge's touches are stretches the two touch along, and a cell beside a branch
                    // a ridge holds is beside a known meeting
                    let stretches: Vec<(&Curve,[f64;2])> = known.iter().filter(|br| br.contact == Contact::Touching)
                        .filter_map(|br| if let Curve::BSpline(s) = &br.curve { Some((&br.curve,s.domain())) } else { None }).collect();
                    let mut beside = |p: V| on_a_branch(a,b,known,p,tol);
                    while let Some(p) = super::ssi::unseen_beside(a,da,b,&[traced.clone(),curves.clone()].concat(),&stretches,&mut beside,lo,hi,tol)? {
                        more += 1;
                        if more > 64 { return Err(format!("a {} and a {} meet in more than 64 curves no seed reaches",a.kind(),b.kind()).into()) }
                        let found = match super::ssi::trace(a,b,&[p],lo,hi,tol) {
                            // past a refusal, one refused again stands where the branches read it
                            Err(_) if past => break,
                            found => found?,
                        };
                        if found.is_empty() {
                            return Err(format!("a {} and a {} meet at {p:?} in a curve that could not be traced",a.kind(),b.kind()).into())
                        }
                        traced.extend(found);
                    }
                }
                Ok(traced)
            })?;
            from_ridges = branches;
            (traced,false)
        }
    };
    // where two branches of a closed form cross (two cylinders of one radius whose axes cross)
    for i in 0..curves.len() { for j in i+1..curves.len() {
        let (Some(pi),Some(pj)) = (curves[i].period(),curves[j].period()) else { continue };
        for (_,q) in super::query::crossings_in(a,&curves[i],[0.,pi],&curves[j],[0.,pj],tol).unwrap_or_default() {
            if singular.iter().any(|s| distance(s.at,q) <= 1e3*tol) { continue }
            let ds = [curves[i].tangent(curves[i].inverse(q)),curves[j].tangent(curves[j].inverse(q))];
            singular.push(Point {at:q,kind:Singular::Crossing(ds.iter().flat_map(|&d| [d,scale(d,-1.)]).collect())});
        }
    } }
    let mut branches: Vec<Branch> = curves.into_iter().map(|c| { let contact = contact(a,b,&c,(lo,hi)); Branch {curve:c,contact} }).collect();
    branches.extend(from_ridges);
    Ok(Meeting {same:false,branches,singular,closed_form})
}

/// **Meetings computed once and shared** (#735 A2): each distinct surface given an index, two faces
/// on surfaces the kernel holds the same (`ssi::same`) the same one, and each pair of indices'
/// closed-form meeting (`ssi::intersect`) found the first time any two faces on them are met and
/// handed to every other, so the same meeting curve bounds every face that uses it, bit for bit.
/// A meeting with no closed form is traced, and each curve traced kept for the pair of surfaces
/// whatever faces it was traced for (`traced`), so the faces met after it are seeded off it and
/// handed it, not tracing it again.
#[derive(Clone,Debug,Default)]
pub struct Shared {
    surfaces: Vec<Surface>,
    /// each kind's surfaces among them, which alone a surface of that kind is compared with
    of_kind: std::collections::BTreeMap<&'static str,Vec<usize>>,
    meetings: std::collections::BTreeMap<(usize,usize),Ssi>,
    traced: std::collections::BTreeMap<(usize,usize),Vec<Curve>>,
}

impl Shared {
    /// The index of `s` among the surfaces seen: an earlier one it is the same as (exactly, or
    /// within `tol` where both are of the same kind), else a new one.
    pub fn index(&mut self,s: &Surface,tol: f64) -> usize {
        let mine = self.of_kind.entry(s.kind()).or_default();
        if let Some(&i) = mine.iter().find(|&&i| self.surfaces[i] == *s) { return i }
        if let Some(&i) = mine.iter().find(|&&i| super::ssi::same(&self.surfaces[i],s,tol)) { return i }
        self.surfaces.push(s.clone());
        mine.push(self.surfaces.len()-1);
        self.surfaces.len()-1
    }
    /// The meeting of surfaces `i` and `j` (indices), found once: the first surface of each index
    /// met with the other's, in the order asked (a closed form's curves run as the order says).
    pub fn meeting(&mut self,i: usize,j: usize,tol: f64) -> Ssi {
        if let Some(m) = self.meetings.get(&(i,j)) { return m.clone() }
        let start = crate::util::clock::Instant::now();
        let m = intersect(&self.surfaces[i],&self.surfaces[j],tol);
        // the profile's count of meetings found in closed form and left to a trace (#777 V4)
        crate::util::meter::meeting(self.surfaces[i].kind(),self.surfaces[j].kind(),!matches!(m,Ssi::Traced),start.elapsed());
        self.meetings.insert((i,j),m.clone());
        m
    }
    /// The curves of surfaces `i` and `j`'s traced meeting found so far, in either order.
    pub fn traced(&self,i: usize,j: usize) -> &[Curve] { self.traced.get(&(i.min(j),i.max(j))).map_or(&[],|v| &v[..]) }
    /// More curves of `i` and `j`'s traced meeting.
    pub fn add_traced(&mut self,i: usize,j: usize,curves: impl IntoIterator<Item=Curve>) {
        self.traced.entry((i.min(j),i.max(j))).or_default().extend(curves)
    }
}

/// `v` made unit, or left as it is where it has no length.
fn unit_or(v: V) -> V { crate::util::space::normalised(v).unwrap_or(v) }


/// **Where two curves cross, whatever they are** (#735 A2: where branches of meetings cross, a
/// singular point of an arrangement), over their stretches `ta` and `tb`: each curve sampled
/// finely enough that its chords stray from it by far less than they are long, every two chords
/// whose boxes come within reach of one another the start of a polish — each curve's point nearest
/// the other's, in turn, until they stop moving — kept where the two come within `reach` of one
/// another inside both stretches, each crossing once. Each crossing's parameter on `a`, on `b`, and
/// its point. What `query::crossings_in` leaves (`None`: a spline's or a trace's crossing with
/// another), found by search, never assumed absent.
pub fn crossings(a: &Curve,ta: [f64;2],b: &Curve,tb: [f64;2],reach: f64) -> Vec<(f64,f64,V)> {
    use super::geom::around;
    use crate::util::bvh::{Bounds,Bvh};
    const N: usize = 512;
    let samples = |c: &Curve,t: [f64;2]| -> Vec<(f64,V)> { (0..=N).map(|k| { let s = t[0]+(t[1]-t[0])*k as f64/N as f64; (s,c.point(s)) }).collect() };
    let (sa,sb) = (samples(a,ta),samples(b,tb));
    let longest = |s: &[(f64,V)]| s.windows(2).map(|w| distance(w[0].1,w[1].1)).fold(0.,f64::max);
    let pad = reach+longest(&sa).max(longest(&sb));
    // each chord's box, `b`'s grown by `pad` and filed in a hierarchy each of `a`'s is asked of
    let chord = |w: &[(f64,V)],grow: f64| { let (lo,hi) = super::geom::hull([w[0].1,w[1].1]);
        Bounds {lo:lo.map(|x| x-grow),hi:hi.map(|x| x+grow)} };
    let boxes = Bvh::new(sb.windows(2).map(|w| chord(w,pad)));
    let wrap = |c: &Curve,t: f64,span: [f64;2]| match c.period() { Some(p) => around(t,span[0],p),None => t };
    let mut out: Vec<(f64,f64,V)> = Vec::new();
    for wa in sa.windows(2) {
        // a build past its budget goes no further (#777 V3): the inverses below charge it, and
        // walked on, every pair of near chords would still be polished at full cost (#780)
        if crate::util::meter::stopped() { break }
        let mut near = Vec::new();
        boxes.query_box(chord(wa,0.),|j| near.push(j));
        near.sort_unstable();
        for j in near {
            // the polish, from the chords' middles: each curve's foot of the other's point
            let (mut s,mut t) = ((wa[0].0+wa[1].0)/2.,(sb[j].0+sb[j+1].0)/2.);
            let mut last = f64::INFINITY;
            for _ in 0..60 {
                t = wrap(b,b.inverse(a.point(s)),tb);
                s = wrap(a,a.inverse(b.point(t)),ta);
                let d = distance(a.point(s),b.point(t));
                if !(d < last*(1.-1e-9)) { last = d; break }
                last = d;
            }
            if !(last <= reach) || s < ta[0]-1e-12 || s > ta[1]+1e-12 || t < tb[0]-1e-12 || t > tb[1]+1e-12 { continue }
            let p = a.point(s);
            // two crossing at a shallow angle meet over a stretch `reach / sin θ` long: one crossing
            let (da,db) = (a.tangent(s),b.tangent(t));
            let sine = (norm(cross(da,db))/(norm(da)*norm(db)).max(1e-300)).max(1e-3);
            if out.iter().any(|x| distance(x.2,p) <= 2.*reach/sine) { continue }
            out.push((s,t,p));
        }
    }
    out
}

/// **The widest reach a meeting along any of `curves` has** (each a curve, its stretch, and the
/// other surface whose meeting with the one they lie on it is): `reach` read at seventeen points
/// along each, the largest — how far apart two of them may stand where they cross and still be
/// crossing, the farthest any is known.
pub fn reach_along(curves: &[(&Curve,[f64;2],&Surface)],reach: &dyn Fn(&Surface,V) -> f64) -> f64 {
    curves.iter().flat_map(|&(c,t,o)| (0..=16).map(move |k| reach(o,c.point(t[0]+(t[1]-t[0])*k as f64/16.))))
        .fold(0.,f64::max)
}
