//! **The ring road's profile on brep-bench's tumbling tori** (#942 F4): `full-turn`, `half-turn` or `thin`, built
//! closed through `swept::closed::ring` with `SOLVENT_BREP_TIME` set, each step's time and what it
//! read printed (the instants read, their time on the cores, the roots read alone). With
//! `SOLVENT_SWEEP_DUMP` set as well, a trim refused is written there (`sweep_probe` reads it).
//!
//! `spline` is brep-bench's ring carried along a spline on the ring road (`closed::ring_cut_from`,
//! `trajectory::Along`, the bench's sibling ring cases' bar and tolerance); `bench-spline` the same
//! case exactly as the bench's runner builds it (`closed::cut_from`, its own frame).
//!
//! `cargo run --release -p cad-kernel --example ring_profile -- full-turn|half-turn|thin [roll end, degrees] | spline | bench-spline`
use cad_kernel::trajectory::{Screw,Trajectory};

/// brep-bench's tumbling torus: a torus of major radius one about `centre`, its axis `z`, turned by
/// `ratio · t` about the vertical line through `(hub, 0, 0)`, seen from a frame turning by
/// `observer · t` about the line through `(0, h cos ℓ, h sin ℓ)` along
/// `(cos α, sin α cos ℓ, sin α sin ℓ)`.
#[allow(clippy::too_many_arguments)]
fn tumbling(centre: [f64;3],hub: f64,tube: f64,ratio: f64,lean: f64,heading: f64,height: f64,observer: f64)
    -> (cad_kernel::topo::Brep,Trajectory) {
    let tool = cad_kernel::build::torus(centre,[0.,0.,1.],1.,tube).unwrap();
    let (sl,cl) = lean.to_radians().sin_cos();
    let (sa,ca) = heading.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[hub,0.,0.],axis:[0.,0.,1.],ratio,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,height*cl,height*sl],axis:[ca,sa*cl,sa*sl],ratio:observer,advance:0.});
    (tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)})
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("spline") { return along_a_spline() }
    if args.first().map(String::as_str) == Some("bench-spline") { return as_the_bench_runs_it() }
    let (tube,(tool,path),half) = match args.first().map(String::as_str) {
        Some("full-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),180.),
        Some("half-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),90.),
        Some("thin") => (0.015,tumbling([3.,0.,2.],2.,0.015,0.25,45.,0.,0.5f64.sqrt(),1.),75.),
        // (brep-bench's 80° `tumbling-torus`, as it spells it: about the line through (0, 0.5, 0.5)
        // along x, built at a bar of 0.002 and a tolerance of 1e-5)
        Some("eighty") => (0.5,tumbling([3.,0.,2.],2.,0.5,0.25,45.,0.,0.5f64.sqrt(),1.),75.),
        // (the gate's own spelling: the observer's origin (0, 0.5, 0.5) and its axis `x` exactly)
        Some("eighty-gate") => {
            let tool = cad_kernel::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
            let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
            let frame = Trajectory::Twist(Screw {origin:[0.,0.5,0.5],axis:[1.,0.,0.],ratio:1.,advance:0.});
            (0.5,(tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)}),75.)
        }
        _ => { eprintln!("ring_profile full-turn|half-turn|thin [roll end, degrees] | spline | bench-spline"); std::process::exit(2) }
    };
    let end: f64 = args.get(1).map_or(half,|e| e.parse().expect("degrees"));
    let roll = [-half.to_radians(),end.to_radians()];
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    let eighty = matches!(args.first().map(String::as_str),Some("eighty" | "eighty-gate"));
    let (bar,mut tol) = if eighty { (0.002,1e-5) } else { (tube/250.,tube/50000.) };
    // (`… size-tol`: built to the modelling tolerance of the tool's size instead)
    if args.iter().any(|a| a == "size-tol") { tol = cad_kernel::geom::tolerance(tool.size()) }
    eprintln!("bar {bar:e}, tolerance {tol:e}");
    let built = cad_kernel::swept::closed::ring(&tool,&path,roll,bar,tol);
    // the check the bench's runner makes: at the modelling tolerance of the solid's size
    if let Ok(b) = &built {
        let size_tol = cad_kernel::geom::tolerance(b.size());
        println!("checked at the size's tolerance {size_tol:e}: {:?}",b.check(size_tol));
    }
    match &built {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
    // `… agree n sag`: n points of the built solid's box classified both ways, by the solid (its mesh
    // within `sag`, by its winding number) and by the sweep's definition (`Swept::member`), and each
    // point read differently measured by the ring's own depth over the roll
    if let (Ok(b),Some(n)) = (&built,args.iter().position(|a| a == "agree").map(|i| args[i+1].parse::<usize>().unwrap())) {
        let sag: f64 = args.iter().position(|a| a == "agree").map_or(1e-4,|i| args[i+2].parse().unwrap());
        agree(b,&tool,&path,roll,n,sag,tube/50000.);
    }
}

/// The built solid against the sweep's definition at `n` points of its box (`… agree n sag`).
fn agree(built: &cad_kernel::topo::Brep,tool: &cad_kernel::topo::Brep,path: &Trajectory,roll: [f64;2],n: usize,sag: f64,tol: f64) {
    use cad_kernel::swept::member::{Solid,Swept};
    let clock = std::time::Instant::now();
    let body = Solid::new(built,sag,1.).unwrap();
    let solid = Solid::new(tool,0.002,1.).unwrap();
    let swept = Swept {sampled:false,tool:&solid,path,roll};
    let ring = cad_kernel::swept::rings::Ring::read(tool).unwrap();
    let (lo,hi) = built.bounds();
    let mut rng = cad_kernel::util::rng::Rng::new(7);
    let pts: Vec<[f64;3]> = (0..n).map(|_| std::array::from_fn(|k| rng.uniform(lo[k],hi[k]))).collect();
    // the ring's least signed distance from `x` over the roll: at 20 000 instants, settled
    let depth = |x: [f64;3]| {
        let m = 20_000;
        let ts: Vec<f64> = (0..=m).map(|k| roll[0]+(roll[1]-roll[0])*k as f64/m as f64).collect();
        let d = |t: f64| ring.signed(path.pose(t).unwrap().inverse().point(x)).0;
        let ds: Vec<f64> = ts.iter().map(|&t| d(t)).collect();
        let k = (0..=m).min_by(|&i,&j| ds[i].total_cmp(&ds[j])).unwrap();
        let (mut a,mut b) = (ts[k.saturating_sub(1)],ts[(k+1).min(m)]);
        let g = 0.5*(5f64.sqrt()-1.);
        for _ in 0..80 { let (x1,x2) = (b-g*(b-a),a+g*(b-a)); if d(x1) < d(x2) { b = x2 } else { a = x1 } }
        ds[k].min(d(0.5*(a+b)))
    };
    let read = cad_kernel::util::par::indices(n,|i| {
        let x = pts[i];
        let s = body.signed_distance(x);
        match swept.member(x) {
            Ok(m) => Some((m,s)),
            Err(_) => None,
        }
    });
    let (mut inside,mut unsettled,mut apart) = (0,0,Vec::new());
    for (i,r) in read.iter().enumerate() {
        match r {
            None => unsettled += 1,
            Some((m,s)) => { if *m { inside += 1 } if *m != (*s < 0.) { apart.push((i,*m,*s)) } }
        }
    }
    let box_volume = (0..3).map(|k| hi[k]-lo[k]).product::<f64>();
    let settled = n-unsettled;
    let p = inside as f64/settled as f64;
    println!("{n} points ({unsettled} unsettled) in {:?}: the sweep's count {:.4} ± {:.4}; {} read apart",clock.elapsed(),box_volume*p,
        box_volume*(p*(1.-p)/settled as f64).sqrt(),apart.len());
    let mut worst = 0f64;
    for &(i,m,s) in &apart {
        let d = depth(pts[i]);
        worst = worst.max(d.abs());
        println!("  {:?}: the sweep {}, the solid's distance {s:+.3e}, the ring's depth over the roll {d:+.3e}{}",pts[i],if m { "in" } else { "out" },
            if d.abs() > tol { " — deeper than the tolerance" } else { "" });
    }
    println!("the points read apart stand within {worst:.3e} of the sweep's boundary (the tolerance {tol:e}, the mesh's sag {sag:e})");
}

/// brep-bench's `ring-carried-along-a-spline`: a torus of radii 1.5 and 0.5 about `y` through
/// (0, 0, 0.5), its frame carried along a cubic in the plane `z = 0` (`trajectory::Along`, the
/// rotation-minimising frame), cut from the block (−6, −6, −8)–(36, 26, 0).
fn along_a_spline() {
    use cad_kernel::nurbs::BSpline;
    let block = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,0.]).unwrap();
    let tool = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(cad_kernel::trajectory::Along::new(curve).unwrap()));
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    match cad_kernel::swept::closed::ring_cut_from(&block,&tool,&path,[0.,1.],0.01,1e-5) {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(&b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
}

/// The frame brep-bench's runner carries the ring by: its `x` along the curve's tangent, its `z` up.
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

/// brep-bench's runner exactly: `closed::cut_from`, its own frame, a bar of 0.005, the tolerance
/// of the larger of the block and the torus.
fn as_the_bench_runs_it() {
    use cad_kernel::nurbs::BSpline;
    let stock = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,0.]).unwrap();
    let ring = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(Runner(curve)));
    let tol = cad_kernel::geom::tolerance(stock.size().max(ring.size()));
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    eprintln!("tolerance {tol:e}");
    match cad_kernel::swept::closed::cut_from(&stock,&ring,&path,[0.,1.],0.005,tol) {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(&b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
}
