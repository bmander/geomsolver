//! **The depth reading on brep-bench's ring carried along a spline** (#942, the depth trim): each
//! candidate sheet's points read for what holds them (`swept::depth`), printed for measurement.
//!
//! `cargo run --release -p cad-kernel --example ring_depth -- bridge|strip|ends|rows`
use cad_kernel::swept::depth::Sampled as Depth;
use cad_kernel::swept::rings::Ring;
use cad_kernel::trajectory::Trajectory;
use std::f64::consts::{FRAC_PI_2,PI};

#[derive(Debug)]
struct Runner(cad_kernel::nurbs::BSpline);
impl cad_kernel::trajectory::Path for Runner {
    fn at(&self,t: f64) -> Result<cad_kernel::trajectory::Motion,cad_kernel::KernelError> {
        let (point,d,dd) = self.0.d2(t);
        let speed2 = d[0]*d[0]+d[1]*d[1];
        let (c,s) = (d[0]/speed2.sqrt(),d[1]/speed2.sqrt());
        let turning = (d[0]*dd[1]-d[1]*dd[0])/speed2;
        let r = [[c,-s,0.],[s,c,0.],[0.,0.,1.]];
        let dr = [[-s*turning,-c*turning,0.],[c*turning,-s*turning,0.],[0.;3]];
        Ok(cad_kernel::trajectory::Motion::of_parts(r,dr,point,d))
    }
}

type V = [f64;3];
fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn sc(a: V,k: f64) -> V { [a[0]*k,a[1]*k,a[2]*k] }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }

/// The torus's point in its own coordinates at `α` round its axis (`y`; 0 along `x`, −π/2 its
/// bottom) and `ψ` round its tube (0 outward, π/2 along `y`).
fn local(alpha: f64,psi: f64) -> V {
    let er = [alpha.cos(),0.,alpha.sin()];
    add(add([0.,0.,0.5],sc(er,1.5)),add(sc(er,0.5*psi.cos()),[0.,0.5*psi.sin(),0.]))
}

/// The contact parallel's `ψ₀` at the instant `t` (the branch `j` half a turn on): `n · v = 0` on
/// the normal circle at `α = 0`, where both terms carry `cos α` alike.
fn psi0(path: &Trajectory,t: f64,j: usize) -> f64 {
    let m = path.at(t).unwrap();
    let core = [1.5,0.,0.5];
    let v = m.velocity(core);
    let (a,b) = (dot(m.vector([1.,0.,0.]),v),dot(m.vector([0.,1.,0.]),v));
    (-a).atan2(b)+j as f64*PI
}

fn main() {
    let tool = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let ring = Ring::read(&tool).unwrap();
    let curve = cad_kernel::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(Runner(curve)));
    let n: usize = std::env::var("N").ok().and_then(|s| s.parse().ok()).unwrap_or(4000);
    let depth = Depth::new(&ring,&path,[0.,1.],n).unwrap();
    let mode = std::env::args().nth(1).unwrap_or_default();
    let place = |alpha: f64,psi: f64,t: f64| path.pose(t).unwrap().point(local(alpha,psi));
    // the class of a sheet's point at its own instant: kept, folded under, or covered from afar
    let class = |x: V,t: f64| -> (char,f64,f64) {
        match depth.cover(x,t) {
            None => ('K',f64::INFINITY,f64::NAN),
            Some(c) if (c.t-t).abs() <= 2./n as f64 && c.depth < 0. => ('F',c.depth,c.t),
            Some(c) if c.depth < 0. => ('C',c.depth,c.t),
            Some(c) => ('K',c.depth,c.t),
        }
    };
    match mode.as_str() {
        // along each branch's parallel across the bottom meridian, at a few instants
        "bridge" => for t in [0.02,0.1,0.25,0.4,0.5,0.6,0.75,0.9,0.98] {
            for j in 0..2 {
                let p0 = psi0(&path,t,j);
                let mut line = format!("t {t:.2} branch {j} ψ0 {p0:+.6}:");
                for e in [-0.3,-0.1,-0.03,-0.01,-0.003,-0.001,-3e-4,-1e-4,1e-4,3e-4,0.001,0.002,0.003,0.005,0.008,0.012,0.016,0.02,0.025,0.03,0.05,0.1,0.3] {
                    let x = place(-FRAC_PI_2+e,p0,t);
                    let (k,d,ct) = class(x,t);
                    line += &format!(" {e:+}:{k}{d:+.1e}@{ct:.4}");
                }
                println!("{line}");
            }
        },
        // tube 0's back leaf near the start: where it is kept, and what holds it beyond (the
        // instant, and the tool's angle there: below θ* its back leaf, above it its front)
        "back" => for t in [0.002,0.01,0.02,0.05,0.1,0.15,0.2,0.3] {
            let p0 = psi0(&path,t,0);
            let mut line = format!("t {t:.3}:");
            for k in 0..=24 {
                let e = 1.25*k as f64/24.;
                let x = place(-FRAC_PI_2-e,p0,t);
                let (c,d,ct) = class(x,t);
                let a = depth.cover(x,t).map_or(f64::NAN,|c| c.at[2].atan2(c.at[0])+FRAC_PI_2);
                line += &format!(" {e:.2}:{c}{d:+.0e}@{ct:.3}/{a:+.2}");
            }
            println!("{line}");
        },
        // round the bottom meridian's normal circle at a few instants
        "strip" => for t in [0.02,0.25,0.5,0.75,0.98] {
            let p0 = psi0(&path,t,0);
            let mut line = format!("t {t:.2} ψ0 {p0:+.4}:");
            for i in 0..24 {
                let psi = p0+PI*i as f64/12.;
                let x = place(-FRAC_PI_2,psi,t);
                let (k,d,ct) = class(x,t);
                line += &format!(" {:+.2}:{k}{d:+.1e}@{ct:.3}",psi-p0);
            }
            println!("{line}");
        },
        // the torus where the roll ends, about its bottom meridian: rows α − θ*, columns ψ − ψ₀
        "ends" => for t in [0.,1.] {
            let p0 = psi0(&path,t,0);
            let es = [-0.3,-0.1,-0.03,-0.01,-0.003,-0.001,-1e-4,0.,1e-4,0.001,0.003,0.01,0.03,0.1,0.3];
            let ps: Vec<f64> = [-0.3,-0.03,-0.003,-3e-4,0.,3e-4,0.003,0.03,0.3,1.,1.57,2.,2.8,3.11,3.138,3.1413,PI,3.1419,3.145,3.17,3.4,4.,4.7,5.5].to_vec();
            println!("t {t}: ψ0 {p0:+.6}; columns ψ−ψ0 {ps:?}");
            for &e in &es {
                let mut line = format!("{e:+8.4}  ");
                for &q in &ps {
                    let x = place(-FRAC_PI_2+e,p0+q,t);
                    let (k,d,_) = class(x,t);
                    line += &format!("{k}{:>9} ",if d.is_finite() { format!("{d:+.1e}") } else { "".into() });
                }
                println!("{line}");
            }
        },
        // each tube's kept stretches of α − θ* below the block's top, at instants through the roll
        "rows" => {
            let m = 2000;
            for j in 0..2 { for i in 0..=20 {
                let t = i as f64/20.;
                let p0 = psi0(&path,t,j);
                let mut runs: Vec<(char,f64,f64,f64)> = Vec::new();
                for q in 0..=m {
                    let e = -1.5+3.*q as f64/m as f64;
                    let x = place(-FRAC_PI_2+e,p0,t);
                    if x[2] > 0. { continue }
                    let (k,d,_) = class(x,t);
                    match runs.last_mut() { Some(r) if r.0 == k => { r.2 = e; if d.is_finite() { r.3 = if k == 'K' { r.3.min(d) } else { r.3.min(d) } } }
                        _ => runs.push((k,e,e,if d.is_finite() { d } else { f64::INFINITY })) }
                }
                let s: Vec<String> = runs.iter().map(|r| format!("{}[{:+.3},{:+.3}]{:.0e}",r.0,r.1,r.2,r.3)).collect();
                println!("tube {j} t {t:.2}: {}",s.join(" "));
            } }
        },
        // the deepest cover of what the simplified cut keeps wrongly or drops: the tube's stretch from
        // θ* to its kept edge, and the strip's rims, through the roll
        "slivers" => {
            let (mut worst_tube,mut worst_strip,mut worst_kept): ((f64,f64),(f64,f64),(f64,f64)) = ((0.,0.),(0.,0.),(0.,0.));
            for i in 0..=400 {
                let t = i as f64/400.;
                for j in 0..2 {
                    let p0 = psi0(&path,t,j);
                    let sgn = if j == 0 { 1. } else { -1. };
                    for q in 0..=200 {
                        let e = sgn*0.03*q as f64/200.;
                        let (k,d,_) = class(place(-FRAC_PI_2+e,p0,t),t);
                        if k != 'K' && d < worst_tube.0 { worst_tube = (d,t) }
                        // and the far side of θ*, dropped: is any of it kept, and by how much
                        let (k,d,_) = class(place(-FRAC_PI_2-e,p0,t),t);
                        if k == 'K' && q > 0 && d.is_finite() && d > worst_kept.0 && t > 0. && t < 1. { worst_kept = (d,t) }
                    }
                    for q in 0..=200 {
                        let psi = p0+if j == 0 { 0.05 } else { -0.05 }*q as f64/200.;
                        let (k,d,_) = class(place(-FRAC_PI_2,psi,t),t);
                        if k != 'K' && d < worst_strip.0 { worst_strip = (d,t) }
                    }
                }
            }
            println!("tube between θ* and its kept edge: deepest {:?}; strip within 0.05 of its rims: deepest {:?}; dropped side kept by {:?}",worst_tube,worst_strip,worst_kept);
        },
        // the start pose's kept wedge beside the tube's rim, in front of θ*: its width in ψ, how far
        // it stands, and the instant covering it beyond
        "wedge" => for t in [0.,1.] {
            let p0 = psi0(&path,t,0);
            let sg = if t == 0. { 1. } else { -1. };
            for e in [0.003,0.005,0.01,0.02,0.03,0.05,0.1,0.2,0.3,0.6,1.0,1.2] {
                let e = sg*e;
                let mut kept = (f64::NAN,f64::NAN,0f64,f64::NAN);
                for q in 0..=4000 {
                    let dq = -0.06+0.0605*q as f64/4000.;
                    let x = place(-FRAC_PI_2+e,p0+dq,t);
                    let (k,d,ct) = class(x,t);
                    if k == 'K' { if kept.0.is_nan() { kept.0 = dq } kept.1 = dq; if d.is_finite() { kept.2 = kept.2.max(d) } }
                    else if !kept.0.is_nan() && kept.3.is_nan() { kept.3 = ct }
                    if k != 'K' && kept.0.is_nan() { kept.3 = ct }
                }
                let z = place(-FRAC_PI_2+e,p0,t)[2];
                println!("t {t} e {e:+.3} (z {z:+.3}): kept ψ−ψ0 in [{:+.5}, {:+.5}], stands {:.1e}, covered beyond from t {:.4}",kept.0,kept.1,kept.2,kept.3);
            }
        },
        // the whole roll's depth a hair either side of points of each kept piece, along its normal
        "sides" => {
            let least = |x: V| depth.minima(x).iter().map(|c| c.depth).fold(f64::INFINITY,f64::min);
            let normal = |alpha: f64,psi: f64,t: f64| path.pose(t).unwrap().vector(add(sc([alpha.cos(),0.,alpha.sin()],psi.cos()),[0.,psi.sin(),0.]));
            let h = 1e-6;
            let mut cases: Vec<(&str,f64,f64,f64)> = vec![("Q1",-FRAC_PI_2-0.05,psi0(&path,0.,0)+1.,0.),("W",-FRAC_PI_2+0.1,psi0(&path,0.,0)-0.005,0.),
                ("W far",-FRAC_PI_2+0.6,psi0(&path,0.,0)-0.03,0.)];
            for t in [0.003,0.3,0.7,0.997] {
                cases.push(("strip",-FRAC_PI_2,psi0(&path,t,0)+1.,t));
                cases.push(("tube0",-FRAC_PI_2+0.4,psi0(&path,t,0),t));
                cases.push(("tube1",-FRAC_PI_2-0.4,psi0(&path,t,1),t));
            }
            for (name,a,p,t) in cases {
                let (x,n) = (place(a,p,t),normal(a,p,t));
                println!("{name} at t {t}: inward {:+.3e}, on {:+.3e}, outward {:+.3e}",least(add(x,sc(n,-h))),least(x),least(add(x,sc(n,h))));
            }
        },
        // the start wedge's seam traced into its corner at a tolerance (`TOL`, the bench's by default)
        "seam" => {
            let tol: f64 = std::env::var("TOL").ok().and_then(|s| s.parse().ok()).unwrap_or(5.3e-8);
            let p0 = psi0(&path,0.,0);
            let pose = path.pose(0.).unwrap();
            let sheet = |e: f64,s: f64| (pose.point(local(-FRAC_PI_2+e,p0+s)),0.);
            let clock = std::time::Instant::now();
            let seam = depth.seam_over(&sheet,[0.3,(tol*20.).min(2.5e-4)],&|_| [0.,-0.3],tol);
            match seam {
                Ok(s) => { println!("{} points in {:?}; worst depth {:.1e}",s.len(),clock.elapsed(),s.iter().map(|p| p.cover.depth.abs()).fold(0.,f64::max));
                    for p in s.iter().step_by((s.len()/8).max(1)).chain(s.last()) { println!("  e {:+.6} ψ−ψ0 {:+.7} partner t {:.6} depth {:.1e}",p.e,p.s,p.cover.t,p.cover.depth) } }
                Err(e) => println!("refused: {e:?}"),
            }
        },
        "edges" => {
            let p0 = psi0(&path,0.,0);
            let pose = path.pose(0.).unwrap();
            let sheet = |e: f64,s: f64| (pose.point(local(-FRAC_PI_2+e,p0+s)),0.);
            for e in [0.3,0.2,0.1,0.05,0.02,0.01,0.005,0.002,0.001,3e-4,1e-4] {
                println!("e {e}: {:?}",depth.edge_on(&sheet,e,[0.,-0.3],1e-5).map(|p| (p.s,p.cover.t,p.cover.depth)));
            }
        },
        "covers" => {
            let p0 = psi0(&path,0.,0);
            let x = path.pose(0.).unwrap().point(local(-FRAC_PI_2+0.05,p0-0.005));
            let clock = std::time::Instant::now();
            for _ in 0..100 { let _ = depth.cover(x,0.); }
            println!("100 covers {:?}; minima {}",clock.elapsed(),depth.minima(x).len());
        },
        "rim" => {
            let p0 = psi0(&path,0.,0);
            let pose = path.pose(0.).unwrap();
            for e in [0.02,0.015,0.01,0.007,0.005,0.003,0.002,0.001,3e-4] {
                let x = pose.point(local(-FRAC_PI_2+e,p0));
                let x2 = pose.point(local(-FRAC_PI_2+e,p0-0.3));
                println!("e {e}: rim {:?}; far {:?}",depth.cover(x,0.),depth.cover(x2,0.).map(|c| c.depth));
            }
        },
        "time" => {
            let clock = std::time::Instant::now();
            let x = place(-FRAC_PI_2,psi0(&path,0.5,0)+4.71,0.5);
            let m = depth.minima(x);
            println!("{} minima in {:?}: {:?}",m.len(),clock.elapsed(),&m[..m.len().min(6)]);
            let clock = std::time::Instant::now();
            for _ in 0..1000 { let _ = path.pose(0.3); }
            println!("1000 poses {:?}",clock.elapsed());
        },
        _ => eprintln!("ring_depth bridge|strip|ends|rows"),
    }
}
