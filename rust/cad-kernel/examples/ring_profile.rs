//! **The ring road's profile on brep-bench's tumbling tori** (#942 F4): `full-turn`, `half-turn` or `thin`, built
//! closed through `swept::closed::ring` with `SOLVENT_BREP_TIME` set, each step's time and what it
//! read printed (the instants read, their time on the cores, the roots read alone). With
//! `SOLVENT_SWEEP_DUMP` set as well, a trim refused is written there (`sweep_probe` reads it).
//!
//! `spline` is brep-bench's ring carried along a spline on the ring road (`closed::ring_cut_from`,
//! `trajectory::Along`, the bench's sibling ring cases' bar and tolerance); `bench-spline` the same
//! case exactly as the bench's runner builds it (`closed::cut_from`, its own frame).
//!
//! `eighty-gate` is brep-bench's 80° torus at its observer's origin (0, 0.5, 0.5) exactly, `eighty`
//! at that origin as `√½ (0, cos 45°, sin 45°)`, an ulp off; `eighty-at oy oz ay az ratio roll` at
//! any spelling of its observer's origin, its axis `(1, ay, az)`, its own ratio and its roll's end in
//! degrees; `half-at ulps|add y z` the half turn, its observer's origin moved.
//! After a build: the check at the size's tolerance and the call's, and `Brep::misfits` at the size's;
//! `nudge d|ulp` moves the tumbling tori's observer's origin along `y` before (an input a hair off
//! the bench's); `fluxes sag` reads each face's flux against its mesh's and the volume off the mesh;
//! `agree n sag` holds the solid to the sweep's definition point by point.
//!
//! `crossed-forward` and `crossed-reverse` are brep-bench's crossed tumbling tori (#880 T1f), built
//! so; `shape crossed-forward|crossed-reverse|… [looks]` reads their contact's shape over the roll
//! and each meridian touched whole for an instant (`Ring::touched_once`), with `n · v`, `s''` and the
//! depth round it; `event-map crossed-forward|crossed-reverse t* θ* dθ dt n` maps how each branch's
//! sheet stands under the sweep about such an instant.
//!
//! `cargo run --release -p cad-kernel --example ring_profile -- full-turn|half-turn|thin [roll end, degrees] | spline | bench-spline`
use cad_kernel::trajectory::{Screw,Trajectory};
#[path = "support/bench.rs"]
mod bench;

/// brep-bench's tumbling torus (`bench::tumbling`), its observer's origin moved by `… nudge d`.
#[allow(clippy::too_many_arguments)]
fn tumbling(centre: [f64;3],hub: f64,tube: f64,ratio: f64,lean: f64,heading: f64,height: f64,observer: f64)
    -> (cad_kernel::topo::Brep,Trajectory) {
    bench::tumbling_moved(centre,hub,tube,ratio,lean,heading,height,observer,|mut origin| {
        // (`… nudge d`: the observer's origin moved by `d` along `y`, or by one ulp where `d` is `ulp`)
        let args: Vec<String> = std::env::args().collect();
        if let Some(i) = args.iter().position(|a| a == "nudge") {
            origin[1] = if args[i+1] == "ulp" { f64::from_bits(origin[1].to_bits()+1) } else { origin[1]+args[i+1].parse::<f64>().unwrap() };
            eprintln!("the observer's origin moved to {origin:?}");
        }
        origin
    })
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("spline") { return along_a_spline() }
    if args.first().map(String::as_str) == Some("bench-spline") { return as_the_bench_runs_it(None) }
    if args.first().map(String::as_str) == Some("bench-spline-at") {
        let by: f64 = args[3].parse().expect("a number");
        return as_the_bench_runs_it(Some((args[1].clone(),args[2] == "ulps",by)))
    }
    if args.first().map(String::as_str) == Some("shape") { return shape(&args[1..]) }
    if args.first().map(String::as_str) == Some("event-map") { return event_map(&args[1..]) }
    if args.first().map(String::as_str) == Some("collar") { return collar(&args[1..]) }
    let (tube,(tool,path),half) = match args.first().map(String::as_str) {
        Some("full-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),180.),
        Some("half-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),90.),
        // (brep-bench's crossed pair: `tumbling-torus-crossed-forward` and `-reverse`)
        Some("crossed-forward") => (0.22,tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,1.),100.),
        Some("crossed-reverse") => (0.22,tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,-1.),100.),
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
        // (`eighty-at oy oz ay az ratio roll`: the gate's spelling with its observer's origin, axis,
        // own ratio and roll's half given)
        Some("eighty-at") => {
            let x: Vec<f64> = args[1..7].iter().map(|a| a.parse().expect("a number")).collect();
            let tool = cad_kernel::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
            let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:x[4],advance:0.});
            let axis = cad_kernel::util::space::normalised([1.,x[2],x[3]]).unwrap();
            let frame = Trajectory::Twist(Screw {origin:[0.,x[0],x[1]],axis,ratio:1.,advance:0.});
            (0.5,(tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)}),x[5])
        }
        // (`half-at ulps|add y z`: the half turn, its observer's origin moved by `y` and `z` ulps,
        // or by `y` and `z`; `full-at` the full turn so)
        Some("half-at" | "full-at") => {
            let (y,z): (f64,f64) = (args[2].parse().expect("a number"),args[3].parse().expect("a number"));
            let moved = |x: f64,by: f64| if args[1] == "ulps" { f64::from_bits((x.to_bits() as i64+by as i64) as u64) } else { x+by };
            let tumbled = bench::tumbling_moved([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.,
                |o| [o[0],moved(o[1],y),moved(o[2],z)]);
            (0.16,tumbled,if args[0] == "full-at" { 180. } else { 90. })
        }
        _ => { eprintln!("ring_profile full-turn|half-turn|thin [roll end, degrees] | spline | bench-spline"); std::process::exit(2) }
    };
    let end: f64 = if args[0] == "eighty-at" || args[0] == "half-at" || args[0] == "full-at" { half } else { args.get(1).map_or(half,|e| e.parse().expect("degrees")) };
    let roll = [-half.to_radians(),end.to_radians()];
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    let eighty = matches!(args.first().map(String::as_str),Some("eighty" | "eighty-gate" | "eighty-at"));
    let (bar,mut tol) = if eighty { (0.002,1e-5) } else { (tube/250.,tube/50000.) };
    // (`… size-tol`: built to the modelling tolerance of the tool's size instead)
    if args.iter().any(|a| a == "size-tol") { tol = cad_kernel::geom::tolerance(tool.size()) }
    eprintln!("bar {bar:e}, tolerance {tol:e}");
    let built = cad_kernel::swept::closed::ring(&tool,&path,roll,bar,tol);
    // the check the bench's runner makes: at the modelling tolerance of the solid's size
    if let Ok(b) = &built {
        let size_tol = cad_kernel::geom::tolerance(b.size());
        println!("checked at the size's tolerance {size_tol:e}: {:?}",b.check(size_tol));
        println!("checked at the call's {tol:e}: {:?}",b.check(tol));
        let misfits = b.misfits(size_tol);
        println!("{} misfits at the size's tolerance",misfits.len());
        for (what,d,held) in &misfits { println!("  {d:.3e} (held to {held:.2e}, {:.1}×): {what}",d/held) }
    }
    match &built {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
    // `… fluxes sag`: each face's flux, and the solid's volume read off its mesh within `sag` (each
    // triangle's signed tetrahedron from the origin)
    if let (Ok(b),Some(i)) = (&built,args.iter().position(|a| a == "fluxes")) {
        let sag: f64 = args[i+1].parse().unwrap();
        let m = cad_kernel::mesh::mesh(b,sag,0.1).unwrap();
        let mut by_mesh = vec![0.;b.faces.len()];
        for (t,&f) in m.tris.iter().zip(&m.of) { let [p,q,r] = t.map(|i| m.pts[i as usize]);
            by_mesh[f as usize] += cad_kernel::util::space::dot(p,cad_kernel::util::space::cross(q,r))/2. }
        for (fi,f) in cad_kernel::props::fluxes(b).iter().enumerate() {
            println!("face {fi} ({}): flux {f:+.6e}, by the mesh {:+.6e}, apart {:+.2e}",b.faces[fi].surface.kind(),by_mesh[fi],f-by_mesh[fi])
        }
        let v: f64 = m.tris.iter().map(|t| { let [p,q,r] = t.map(|i| m.pts[i as usize]);
            cad_kernel::util::space::dot(p,cad_kernel::util::space::cross(q,r))/6. }).sum();
        println!("the mesh within {sag:e}: {} triangles, volume {v}",m.tris.len());
        // (each face read apart from its mesh by more than the sag's share: its loops term by term)
        for fi in 0..b.faces.len() {
            let f = &b.faces[fi];
            let flux: f64 = cad_kernel::props::terms(b,fi).iter().flatten().map(|t| t.0+t.1).sum();
            if (flux-by_mesh[fi]).abs() <= 1e-3*by_mesh[fi].abs().max(1e-3) { continue }
            println!("face {fi}: {:?}",f.surface.kind());
            // (its parameters' box read whole by a product Gauss rule, ten nodes a span each way: a
            // face whose loop is its box has that flux)
            let uv: Vec<[f64;2]> = f.loops.iter().flatten().flat_map(|c| b.uv_ends(f,c)).collect();
            let (lo,hi) = ([0,1].map(|k| uv.iter().map(|q| q[k]).fold(f64::INFINITY,f64::min)),[0,1].map(|k| uv.iter().map(|q| q[k]).fold(f64::NEG_INFINITY,f64::max)));
            let cuts = |k: usize| { let mut x = vec![lo[k]]; x.extend(f.surface.breaks(k,[lo[k],hi[k]])); x.push(hi[k]); x };
            let (cu,cv) = (cuts(0),cuts(1));
            let (xs,ws) = ([-0.9739065285171717,-0.8650633666889845,-0.6794095682990244,-0.4333953941292472,-0.1488743389816312,
                0.1488743389816312,0.4333953941292472,0.6794095682990244,0.8650633666889845,0.9739065285171717],
                [0.0666713443086881,0.1494513491505806,0.2190863625159820,0.2692667193099963,0.2955242247147529,
                0.2955242247147529,0.2692667193099963,0.2190863625159820,0.1494513491505806,0.0666713443086881]);
            let mut sum = 0.;
            for w in cu.windows(2) { for z in cv.windows(2) { for i in 0..10 { for j in 0..10 {
                let (u,v) = (0.5*(w[0]+w[1])+0.5*(w[1]-w[0])*xs[i],0.5*(z[0]+z[1])+0.5*(z[1]-z[0])*xs[j]);
                let (x,su,sv) = f.surface.d1([u,v]);
                sum += ws[i]*ws[j]*0.25*(w[1]-w[0])*(z[1]-z[0])*cad_kernel::util::space::dot(x,cad_kernel::util::space::cross(su,sv));
            } } } }
            println!("  its box {lo:?}..{hi:?} ({} by {} spans) read whole: {sum:+.6e}",cu.len()-1,cv.len()-1);
            // (and each side's ∫ G dv, G from the box's middle u: ∫∫ from there to the side)
            let mid = 0.5*(lo[0]+hi[0]);
            for side in [lo[0],hi[0]] {
                let mut x = vec![mid];
                let (p,q) = (mid.min(side),mid.max(side));
                x.extend(f.surface.breaks(0,[p,q]));
                x.push(q);
                if side < mid { x.reverse(); x[0] = mid; *x.last_mut().unwrap() = side; }
                let split = std::env::var("SPLIT_V").ok().and_then(|v| v.parse::<f64>().ok());
                let mut part = 0.;
                let mut sum = 0.;
                for w in x.windows(2) { for z in cv.windows(2) { for i in 0..10 { for j in 0..10 {
                    let (u,v) = (0.5*(w[0]+w[1])+0.5*(w[1]-w[0])*xs[i],0.5*(z[0]+z[1])+0.5*(z[1]-z[0])*xs[j]);
                    let (x,su,sv) = f.surface.d1([u,v]);
                    let y = ws[i]*ws[j]*0.25*(w[1]-w[0])*(z[1]-z[0])*cad_kernel::util::space::dot(x,cad_kernel::util::space::cross(su,sv));
                    sum += y;
                    if split.is_some_and(|sv| v < sv) { part += y }
                } } } }
                println!("  ∫ G dv up the side u = {side}, G from u = {mid}: {sum:+.6e} (below the split {part:+.6e})");
            }
            for (li,(l,t)) in f.loops.iter().zip(cad_kernel::props::terms(b,fi)).enumerate() {
                for (c,t) in l.iter().zip(t) {
                    let e = &b.edges[c.edge as usize];
                    let pc = format!("{:?}",c.pcurve);
                    println!("  loop {li} edge {} ({}) reversed {}: {:?}, terms {:+.4e} {:+.4e}; pcurve {}",c.edge,e.curve.kind(),c.reversed,b.uv_ends(f,c),t.0,t.1,
                        &pc[..pc.len().min(60)]);
                    // (an inverse pcurve read at 40 places from the line between its ends, and carried
                    // along from the place before as `props` reads it)
                    if matches!(c.pcurve,cad_kernel::topo::Pcurve::Inverse {..}) {
                        let mut last: Option<(f64,[f64;2],[f64;2])> = None;
                        for k in 0..=40 {
                            let t = e.t[0]+(e.t[1]-e.t[0])*k as f64/40.;
                            let plain = c.pcurve.at(t,e,&f.surface,&b.vertices);
                            let seed = last.map(|(t0,uv,d)| [uv[0]+d[0]*(t-t0),uv[1]+d[1]*(t-t0)]);
                            let (uv,d) = c.pcurve.at_and_derivative_from(t,e,&f.surface,&b.vertices,seed);
                            last = Some((t,uv,d));
                            if (uv[0]-plain[0]).abs()+(uv[1]-plain[1]).abs() > 1e-6 { println!("    at {k}: from the line {plain:?}, carried {uv:?} (seed {seed:?}, rate {d:?})") }
                            let h = 1e-4*(e.t[1]-e.t[0]);
                            let (p0,p1) = (c.pcurve.at((t-h).max(e.t[0]),e,&f.surface,&b.vertices),c.pcurve.at((t+h).min(e.t[1]),e,&f.surface,&b.vertices));
                            let w = (t+h).min(e.t[1])-(t-h).max(e.t[0]);
                            let (_,su,sv) = f.surface.d1(plain);
                            let sin = cad_kernel::util::space::norm(cad_kernel::util::space::cross(su,sv))/(cad_kernel::util::space::norm(su)*cad_kernel::util::space::norm(sv));
                            if k%4 == 0 { println!("    at {k}: rate {d:?}, the feet's chord [{:.4e}, {:.4e}], sin {sin:.3e}",(p1[0]-p0[0])/w,(p1[1]-p0[1])/w) }
                        }
                    }
                }
            }
        }
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
    let swept = Swept::new(&solid,path,roll);
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
    let block = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,0.]).unwrap();
    let tool = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let along = cad_kernel::trajectory::Along::new(bench::cubic()).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(along));
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    match cad_kernel::swept::closed::ring_cut_from(&block,&tool,&path,[0.,1.],0.01,1e-5) {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(&b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
}

/// brep-bench's runner exactly: `closed::cut_from`, its own frame, a bar of 0.005, the tolerance
/// of the larger of the block and the torus.
fn as_the_bench_runs_it(moved: Option<(String,bool,f64)>) {
    use cad_kernel::nurbs::BSpline;
    // (`bench-spline-at <input> ulps|add <by>`: one input moved — a control point's coordinate
    // `p1x` … `p3y`, the ring's `major`, `minor`, its centre's `cx` `cy` `cz`, the block's `top`, the
    // roll's `t0` or `t1`)
    let m = |name: &str,x: f64| match &moved {
        Some((n,ulps,by)) if n == name => if *ulps { f64::from_bits((x.to_bits() as i64+*by as i64) as u64) } else { x+by },
        _ => x,
    };
    let stock = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,m("top",0.)]).unwrap();
    let ring = cad_kernel::build::torus([m("cx",0.),m("cy",0.),m("cz",0.5)],[0.,1.,0.],m("major",1.5),m("minor",0.5)).unwrap();
    let curve = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[m("p1x",10.),m("p1y",0.),0.],[m("p2x",20.),m("p2y",20.),0.],
        [m("p3x",30.),m("p3y",20.),0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(bench::Runner(curve)));
    let roll = [m("t0",0.),m("t1",1.)];
    let tol = cad_kernel::geom::tolerance(stock.size().max(ring.size()));
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    eprintln!("tolerance {tol:e}");
    match cad_kernel::swept::closed::cut_from(&stock,&ring,&path,roll,0.005,tol) {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(&b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
}

/// **`shape crossed-forward|crossed-reverse|near-orthogonal|thick [looks]`: the contact's shape over
/// the roll as the road reads it** (#880 T1f): at `looks` rolls each curve's pieces (`T` touching,
/// `F` a fold or bridge, `B` buried; a curve running round twice says so), and each meridian's gap
/// from being touched whole (`rings::whole_gap`: `B` where `A` vanishes, nought where the two
/// branches of the tube's contact meet); between two rolls that differ the change halved to the
/// rounding and named.
fn shape(args: &[String]) {
    use cad_kernel::swept::rings;
    let (tool,path,half) = match args.first().map(String::as_str) {
        Some("crossed-forward") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,1.); (t,p,100.) }
        Some("crossed-reverse") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,-1.); (t,p,100.) }
        Some("near-orthogonal") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.08,1.01,1.,89.,0.,1.); (t,p,180.) }
        Some("thick") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.16,1.01,1.,89.,0.,1.); (t,p,180.) }
        Some("half-turn") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.); (t,p,90.) }
        Some("full-turn") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.); (t,p,180.) }
        Some("thin") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.015,0.25,45.,0.,0.5f64.sqrt(),1.); (t,p,75.) }
        Some("eighty") => { let (t,p) = bench::tumbling([3.,0.,2.],2.,0.5,0.25,45.,0.,0.5f64.sqrt(),1.); (t,p,75.) }
        _ => { eprintln!("ring_profile shape crossed-forward|crossed-reverse|near-orthogonal|thick [looks]"); std::process::exit(2) }
    };
    let looks: usize = args.get(1).map_or(400,|x| x.parse().expect("a count"));
    let roll = [-(half as f64).to_radians(),(half as f64).to_radians()];
    let ring = rings::Ring::read(&tool).unwrap();
    let word = |t: f64| -> String {
        match rings::shape_at(&ring,&path,roll,t) {
            Ok(cs) => cs.iter().map(|c| {
                let span = c.last().map_or(0.,|p| p.1)-c.first().map_or(0.,|p| p.0);
                let turns = (span/std::f64::consts::TAU).round();
                let kinds: String = c.iter().map(|p| match p.3 { rings::Side::Touching => 'T',rings::Side::Fold => 'F',rings::Side::Buried => 'B' }).collect();
                // (each curve's kinds from its least rotation, the curves in order: the shape
                // `same_shape` compares, not where the curve's reading starts)
                let n = kinds.len();
                let kinds = (0..n.max(1)).map(|r| format!("{}{}",&kinds[r.min(n)..],&kinds[..r.min(n)])).min().unwrap_or_default();
                if turns > 1. { format!("{kinds}x{turns}") } else { kinds }
            }).collect::<Vec<_>>().tap_sorted().join("|"),
            Err(e) => format!("refused {e:?}"),
        }
    };
    let gap = |t: f64| -> f64 {
        let w = cad_kernel::trajectory::Twist::body(path.at(t).unwrap());
        rings::whole_gap(&ring,&w).iter().map(|g| g.2.abs()).fold(f64::INFINITY,f64::min)
    };
    let ts: Vec<f64> = (0..=looks).map(|k| roll[0]+(roll[1]-roll[0])*k as f64/looks as f64).collect();
    let words: Vec<String> = ts.iter().map(|&t| word(t)).collect();
    println!("at {:.6}: {} (least whole-touch gap {:.3e})",ts[0],words[0],gap(ts[0]));
    for k in 1..ts.len() {
        if words[k] == words[k-1] { continue }
        // halved to the rounding between the two rolls that differ
        let (mut x,mut y) = (ts[k-1],ts[k]);
        while y-x > 1e-13 { let m = 0.5*(x+y); if word(m) == words[k-1] { x = m } else { y = m } }
        println!("at {:.12}: {} -> {} (least whole-touch gap {:.3e} there)",0.5*(x+y),word(x),word(y),gap(0.5*(x+y)));
    }
    for (t,theta,k) in ring.touched_once(&path,roll).unwrap() { println!("touched once (Ring::touched_once): t {t:.12}, θ {theta:.9}, arc {k}") }
    // each sign change of the whole-touch gap (`B` where `A` vanishes, signed), halved to the rounding
    let signed = |t: f64,j: usize| -> (f64,f64) {
        let w = cad_kernel::trajectory::Twist::body(path.at(t).unwrap());
        let g = rings::whole_gap(&ring,&w);
        (g[j].2,g[j].1)
    };
    for j in 0..2 {
        for k in 1..ts.len() {
            let (g0,g1) = (signed(ts[k-1],j).0,signed(ts[k],j).0);
            if (g0 < 0.) == (g1 < 0.) { continue }
            // (the angle `A` vanishes at jumps half a turn where `P` passes nought: a sign change there is none)
            if (signed(ts[k-1],j).1-signed(ts[k],j).1).abs() > 0.5 && ((signed(ts[k-1],j).1-signed(ts[k],j).1).abs()-std::f64::consts::TAU).abs() > 0.5 { continue }
            let (mut x,mut y) = (ts[k-1],ts[k]);
            while y-x > 1e-14 { let m = 0.5*(x+y); if (signed(m,j).0 < 0.) == (g0 < 0.) { x = m } else { y = m } }
            let t = 0.5*(x+y);
            let (_,theta) = signed(t,j);
            let m = ring.meridian_point(theta,0);
            println!("the meridian at θ {theta:.9} touched whole at t {t:.12} ({:.3}°): its outer point {:?} on the tool, {:?} in space",
                t.to_degrees(),m,path.pose(t).unwrap().point(m));
            // where each branch arrives at the meridian either side, and the pieces about it
            for branch in [true,false] {
                for side in [-1e-7,1e-7] {
                    let r = rings::branch_at(&ring,&path,roll,t,branch,&[theta+side]).unwrap();
                    let psi = { let l = ring.frame.local(path.pose(t).unwrap().inverse().point(r[0].1)); (l[2]-2.).atan2(l[0].hypot(l[1])-1.) };
                    println!("  branch {branch} at θ*{side:+e}: ψ {:.6}, s'' {:+.4e}, at {:?}",psi.rem_euclid(std::f64::consts::TAU),r[0].2,r[0].1);
                }
            }
            for dt in [-1e-3,0.,1e-3] {
                let cs = rings::shape_at(&ring,&path,roll,t+dt).unwrap();
                for c in &cs { let held: Vec<String> = c.iter().filter(|p| { let x = theta-p.0; x.rem_euclid(std::f64::consts::TAU) < p.1-p.0 }).map(|p| format!("{:?} [{:.4}, {:.4}]",p.3,p.0,p.1)).collect(); println!("  at t*{dt:+e} the piece over θ*: {held:?}") }
            }
            // round it: `n · v` (nought to the rounding), `s''` and how deep the sweep's other instants hold it
            let scale = 1.;
            let psis: Vec<f64> = (0..24).map(|i| std::f64::consts::TAU*i as f64/24.).collect();
            for (psi,x,nv,bend) in rings::meridian_read(&ring,&path,roll,t,theta,&psis).unwrap() {
                let cover = cad_kernel::swept::depth::cover(&ring,&path,roll,x,t,0.,4.4e-6*scale);
                let said = match cover { Ok(cad_kernel::swept::depth::Reading::Far(c)) => format!("far {:+.3e} at {:.4}",c.depth,c.at),Ok(r) => format!("{r:?}"),Err(e) => format!("{e:?}") };
                println!("  ψ {psi:.3}: n·v {nv:+.1e}, s'' {bend:+.4e}, {said}");
            }
        }
    }
}

/// A list sorted, as a word for the probe's shapes.
trait Sorted { fn tap_sorted(self) -> Self; }
impl Sorted for Vec<String> { fn tap_sorted(mut self) -> Self { self.sort(); self } }

/// **`event-map crossed-forward|crossed-reverse t* θ* dθ dt n`: how the sheets stand about a
/// meridian touched whole** (#880 T1f): over `θ* ± dθ` and `t* ± dt`, `n` a side, each branch's
/// contact read as the depth reads it — `#` its far cover deeper than the tolerance (covered),
/// `n` held by the instants beside its own (`Near`), `o` on the boundary (`Alone`, or its far cover
/// within the tolerance), lowercase where `s''` is below nought.
fn event_map(args: &[String]) {
    use cad_kernel::swept::{depth,rings};
    let observer = if args[0] == "crossed-forward" { 1. } else { -1. };
    let (tool,path) = bench::tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,observer);
    let x: Vec<f64> = args[1..5].iter().map(|a| a.parse().unwrap()).collect();
    let n: usize = args[5].parse().unwrap();
    let roll = [-100f64.to_radians(),100f64.to_radians()];
    let ring = rings::Ring::read(&tool).unwrap();
    let tol = 0.22/50000.;
    for branch in [true,false] {
        println!("branch {branch}: θ across, t down from {:.6} to {:.6}",x[0]+x[3],x[0]-x[3]);
        for j in (0..=n).rev() {
            let t = x[0]-x[3]+2.*x[3]*j as f64/n as f64;
            let thetas: Vec<f64> = (0..=2*n).map(|i| x[1]-x[2]+2.*x[2]*i as f64/(2*n) as f64).collect();
            let row: String = rings::branch_at(&ring,&path,roll,t,branch,&thetas).unwrap().iter().map(|&(_,p,bend)| {
                let c = match depth::cover(&ring,&path,roll,p,t,0.,tol) {
                    Ok(depth::Reading::Near) => 'n',
                    Ok(depth::Reading::Far(c)) if c.depth < -tol => '#',
                    Ok(_) => 'o',
                    Err(_) => '?',
                };
                if bend < 0. { c } else { c.to_ascii_uppercase() }
            }).collect();
            println!("{t:+.5} {row}");
        }
    }
}

/// **`collar crossed-forward|crossed-reverse ρ n m`: the sweep's boundary on the collar about the
/// meridian touched whole** (#880 T1f, `rings::Collar`): over `ψ` round the meridian (`n` columns)
/// and `ρ` in `±ρ` (`m` rows; `ρ > 0` the branch `atan2(−A, B)`), each point read by the depth —
/// `#` far covered deeper than the tolerance, `n` held by the instants beside its own, `o` on the
/// boundary, `?` the chart unread; then where along each of `m` rows the boundary ends (the far
/// cover's depth through nought, halved to the rounding), its `ψ` and its covering instant less its
/// own: a double curve's preimage, its partner the same curve where that passes nought (a
/// swallowtail point).
fn collar(args: &[String]) {
    use cad_kernel::swept::{depth,rings};
    use std::f64::consts::TAU;
    let observer = if args[0] == "crossed-forward" { 1. } else { -1. };
    let (tool,path) = bench::tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,observer);
    let rmax: f64 = args[1].parse().unwrap();
    let (n,m): (usize,usize) = (args[2].parse().unwrap(),args[3].parse().unwrap());
    let roll = [-100f64.to_radians(),100f64.to_radians()];
    let tol = 0.22/50000.;
    let ring = rings::Ring::read(&tool).unwrap();
    let event = ring.touched_once(&path,roll).unwrap()[0];
    let collar = rings::Collar::new(&ring,&path,roll,event).expect("a regular collar");
    println!("event t* {:.12} θ* {:.9}; (A, B) by (θ, t) {:?}",event.0,event.1,collar.jacobian);
    // (the far cover's depth at `(ψ, ρ)` and its instant less the point's own; `None` unread)
    let read = |psi: f64,rho: f64| -> Option<(char,f64,f64)> {
        let (x,[_,t]) = collar.point(psi,rho)?;
        Some(match depth::cover(&ring,&path,roll,x,t,0.,tol).ok()? {
            depth::Reading::Near => ('n',-1.,0.),
            depth::Reading::Far(c) => (if c.depth < -tol { '#' } else { 'o' },c.depth,c.at-t),
            depth::Reading::Alone => ('o',1.,f64::NAN),
        })
    };
    let rows: Vec<f64> = (0..=m).map(|j| rmax-2.*rmax*j as f64/m.max(1) as f64).collect();
    for &rho in &rows {
        let line: String = (0..n).map(|i| read(TAU*i as f64/n as f64,rho).map_or('?',|r| r.0)).collect();
        println!("{rho:+.4} {line}");
    }
    for &rho in &rows {
        let mut out = format!("ρ {rho:+.4}:");
        let kept = |psi: f64| read(psi,rho).is_some_and(|r| r.1 > -tol && r.0 != 'n');
        for i in 0..n {
            let (p,q) = (TAU*i as f64/n as f64,TAU*(i+1) as f64/n as f64);
            if kept(p) == kept(q) { continue }
            let (mut x,mut y) = (p,q);
            loop { let h = 0.5*(x+y); if !(h > x && h < y) { break } if kept(h) == kept(p) { x = h } else { y = h } }
            let held = if kept(p) { y } else { x };
            let (Some((_,d,dt)),Some((pt,[th,t]))) = (read(held,rho),collar.point(held,rho)) else { out += " (unread)"; continue };
            out += &format!("  ψ {:.6}° (θ {th:.6}, t {t:.6}) cover {d:+.1e} at own{dt:+.5} {pt:.5?}",held.to_degrees());
        }
        println!("{out}");
    }
}
