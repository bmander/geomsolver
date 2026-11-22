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
    // (`… nudge d`: the observer's origin moved by `d` along `y`, or by one ulp where `d` is `ulp`)
    let args: Vec<String> = std::env::args().collect();
    let mut origin = [0.,height*cl,height*sl];
    if let Some(i) = args.iter().position(|a| a == "nudge") {
        origin[1] = if args[i+1] == "ulp" { f64::from_bits(origin[1].to_bits()+1) } else { origin[1]+args[i+1].parse::<f64>().unwrap() };
        eprintln!("the observer's origin moved to {origin:?}");
    }
    let frame = Trajectory::Twist(Screw {origin,axis:[ca,sa*cl,sa*sl],ratio:observer,advance:0.});
    (tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)})
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("spline") { return along_a_spline() }
    if args.first().map(String::as_str) == Some("bench-spline") { return as_the_bench_runs_it(None) }
    if args.first().map(String::as_str) == Some("bench-spline-at") {
        let by: f64 = args[3].parse().expect("a number");
        return as_the_bench_runs_it(Some((args[1].clone(),args[2] == "ulps",by)))
    }
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
            let (sl,cl) = 30f64.to_radians().sin_cos();
            let (sa,ca) = 20f64.to_radians().sin_cos();
            let tool = cad_kernel::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
            let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
            let frame = Trajectory::Twist(Screw {origin:[0.,moved(0.7071*cl,y),moved(0.7071*sl,z)],axis:[ca,sa*cl,sa*sl],ratio:1.,advance:0.});
            (0.16,(tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)}),if args[0] == "full-at" { 180. } else { 90. })
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
    let path = Trajectory::General(std::sync::Arc::new(Runner(curve)));
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
