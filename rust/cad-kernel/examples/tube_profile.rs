//! **The tube road's charts read on a tumbling torus** (#880 T1f, `swept::tube`): its two offset
//! sheets fitted as maps over the whole `(θ, t)` rectangle (`nurbs::fit_net`), their cuspidal edges
//! (`tube::edges`) and swallowtail points, a map of how each sheet stands under the sweep (`J`'s
//! sign against `depth::cover`'s reading: kept, covered, held by its neighbours), the kept regions
//! counted on that map, and the depth's zero set on each sheet traced (`zero_set::trace`).
//!
//! `TUBE_AT=side,θ,t,dθ,dt` reads the depth and `J` on a 9 × 9 grid about a sheet's point instead;
//! `build` builds the solid by the tube road (`faces::solid`: faces, volume, time, covers read),
//! `old` by the old ring road (`closed::ring`), `compare` reads the tube's certified cover against
//! `depth::cover` on the grid, and `bound` the Jacobian's proved sign over cells against `J` read in
//! them, and the sheets' edges' time. `TUBE_HALF` (degrees) sets the roll's half instead.
//! The tube is read pinched (`Tube::pinched`): `pinch` reads each chart's cuspidal edges,
//! swallowtail points and (with `trace`) depth's zero set, the collar's too (`SEEDS=θ,t,…` adds
//! seeds); `fit` fits each sheet over a window holding the pinch and reads it against the sheet
//! outside the hole; `COLLAR_AT=ψ0,ψ1,ρ,n` reads the collar's side and depth along `ψ`;
//! `SEED_AT=side,θ,t,θ0,θ1` traces a sheet's depth from one seed alone over `[θ0, θ1]`.
//!
//! `cargo run --release -p cad-kernel --example tube_profile -- half-turn|full-turn|thin|eighty|tumble|crossed-forward|crossed-reverse [grid θ] [grid t] [trace]`
use cad_kernel::swept::depth::Reading;
use cad_kernel::swept::rings::Ring;
use cad_kernel::swept::tube::{self,Chart,Tube};
#[path = "support/bench.rs"]
mod bench;
use std::f64::consts::TAU;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let ((tool,path),half,tube_r) = match args.first().map(String::as_str) {
        Some("full-turn") => (bench::tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),180.,0.16),
        Some("tumble") => (bench::tumbling([3.,0.,2.],2.,0.5,0.25,45.,0.,0.5f64.sqrt(),1.),75.,0.5),
        Some("eighty") => (bench::tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),80.,0.16),
        Some("thin") => (bench::tumbling([3.,0.,2.],2.,0.015,0.25,45.,0.,0.5f64.sqrt(),1.),75.,0.015),
        Some("crossed-forward") => (bench::tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,1.),100.,0.22),
        Some("crossed-reverse") => (bench::tumbling([3.,0.,2.],2.,0.22,0.7,45.,20.,0.7071,-1.),100.,0.22),
        _ => (bench::tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),90.,0.16),
    };
    let nth = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(128usize);
    let nt = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(64usize);
    let tracing = args.iter().any(|a| a == "trace");
    let half = std::env::var("TUBE_HALF").ok().and_then(|h| h.parse().ok()).unwrap_or(half);
    let tol = tube_r/std::env::var("TUBE_TOL").ok().and_then(|t| t.parse::<f64>().ok()).unwrap_or(50_000.);
    let roll = [-(half as f64).to_radians(),(half as f64).to_radians()];
    let ring = Ring::read(&tool).unwrap();
    let clock = Instant::now();
    // (`NOPINCH`: the tube read without its pinch, `Tube::new`)
    let tb = if std::env::var_os("NOPINCH").is_some() { Tube::new(&ring,&path,roll).unwrap() } else { Tube::pinched(&ring,&path,roll,tol,0.25*ring.thickness).unwrap() };
    if let Some(p) = &tb.pinch { println!("pinch at (t*, θ*) {:?}: ρ₀ {}, reach {:?}, in {:?}",p.collar.at,p.rho,p.reach,clock.elapsed()) }
    if let Some(at) = std::env::var_os("SEED_AT") {
        let v: Vec<f64> = at.to_str().unwrap().split(',').map(|x| x.parse().unwrap()).collect();
        let chart = Chart::Sheet(v[0]);
        let field = tube::Depth {tube:&tb,chart,within:tol,holes:&[]};
        let rect = [[v[3],v[4]],roll];
        let r = cad_kernel::zero_set::through(&field,&cad_kernel::zero_set::Question {rect,edges:&[],seeds:&[[v[1],v[2]]],tol,feature:0.25*ring.thickness});
        match r { Ok(t) => for p in &t.pieces { println!("{} points {:?} {:?} .. {:?}",p.uv.len(),p.ends,p.uv[0],p.uv[p.uv.len()-1]);
            for q in p.uv.iter().step_by(4) { println!("    {q:?} hole {:?} J {:?}",tb.pinch.as_ref().map(|h| h.side(*q)),tb.jacobian(v[0],q[0],q[1])) } },Err(e) => println!("{e}") }
        let s = [v[1],v[2]];
        println!("least at the seed {:?}",cad_kernel::zero_set::Field::least(&field,s).map(|v| (v.f,v.grad)));
        return
    }
    // (`CELL_AT=side,θ0,θ1,t0,t1,n`: the Jacobian's proved bound over the cell and over each of its
    // n × n sub-cells, against `J`'s least read on a 9 × 9 grid in each)
    if let Some(at) = std::env::var_os("CELL_AT") {
        let v: Vec<f64> = at.to_str().unwrap().split(',').map(|x| x.parse().unwrap()).collect();
        let (side,n) = (v[0],v[5] as usize);
        for i in 0..n { for j in 0..n {
            let a = [v[1]+(v[2]-v[1])*i as f64/n as f64,v[1]+(v[2]-v[1])*(i+1) as f64/n as f64];
            let b = [v[3]+(v[4]-v[3])*j as f64/n as f64,v[3]+(v[4]-v[3])*(j+1) as f64/n as f64];
            let b9 = tb.jacobian_bound(side,[a,b]).map(|b| (b.value.lo(),b.value.hi()));
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            for p in 0..9 { for q in 0..9 {
                if let Some(j) = tb.jacobian(side,a[0]+(a[1]-a[0])*p as f64/8.,b[0]+(b[1]-b[0])*q as f64/8.) { lo = lo.min(j); hi = hi.max(j) }
            } }
            let bad = b9.is_some_and(|(l,h)| l > hi || h < lo);
            println!("{a:.4?} × {b:.4?}: bound {b9:?}, J read {lo:.3e} .. {hi:.3e}{}",if bad { "  WRONG" } else { "" });
        } }
        return
    }
    // (`EDGE_RECT=side,θ0,θ1,t0,t1`: a sheet's cuspidal edges traced over that rectangle alone)
    if let Some(at) = std::env::var_os("EDGE_RECT") {
        let v: Vec<f64> = at.to_str().unwrap().split(',').map(|x| x.parse().unwrap()).collect();
        let field = tube::Edge {tube:&tb,chart:Chart::Sheet(v[0])};
        let q = cad_kernel::zero_set::Question {rect:[[v[1],v[2]],[v[3],v[4]]],edges:&[],seeds:&[],tol,feature:0.25*ring.thickness};
        match cad_kernel::zero_set::trace(&field,&q) {
            Ok(t) => { for p in &t.pieces { println!("{} points {:?} {:?} .. {:?}",p.uv.len(),p.ends,p.uv[0],p.uv[p.uv.len()-1]) }
                let s = &t.search; println!("search free {} monotone {} beside {} sampled {} unresolved {}",s.free,s.monotone,s.beside,s.sampled,s.unresolved.len());
                // (`AGAINST=θ,t,…`: each such point's least distance in space, on the spine, from the pieces)
                if let Ok(a) = std::env::var("AGAINST") {
                    let x: Vec<f64> = a.split(',').map(|x| x.parse().unwrap()).collect();
                    for c in x.chunks(2) {
                        let y = tb.spine(c[0],c[1]).unwrap().s;
                        let d = t.pieces.iter().flat_map(|p| p.uv.iter()).map(|q| cad_kernel::util::space::distance(tb.spine(q[0],q[1]).unwrap().s,y)).fold(f64::INFINITY,f64::min);
                        println!("({},{}) {d:.4e} from the pieces in space",c[0],c[1]);
                    }
                } }
            Err(e) => println!("{e}"),
        }
        return
    }
    if let Some(at) = std::env::var_os("LINE_AT") {
        let v: Vec<f64> = at.to_str().unwrap().split(',').map(|x| x.parse().unwrap()).collect();
        let (side,a,b,n) = (v[0],[v[1],v[2]],[v[3],v[4]],v[5] as usize);
        let field = tube::Depth {tube:&tb,chart:Chart::Sheet(side),within:tol,holes:&[]};
        for i in 0..=n {
            let f = i as f64/n as f64;
            let uv = [a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f];
            let sp = tb.spine(uv[0],uv[1]).unwrap();
            let area = cad_kernel::util::space::norm(cad_kernel::util::space::cross(sp.s_th,sp.s_t));
            println!("({:.6},{:.6}) J {:?} area {area:.3e} bound {:?} reading {:?}",uv[0],uv[1],tb.jacobian(side,uv[0],uv[1]),
                tb.jacobian_bound(side,[[uv[0]-1e-4,uv[0]+1e-4],[uv[1]-1e-4,uv[1]+1e-4]]).map(|b| (b.value.lo(),b.value.hi())),
                field.reading(uv).map(|r| match r.1 { Reading::Far(c) => format!("far {:.3e} at {:.6}",c.depth,c.at),o => format!("{o:?}") }));
        }
        return
    }
    if let Some(at) = std::env::var_os("COLLAR_AT") {
        let v: Vec<f64> = at.to_str().unwrap().split(',').map(|x| x.parse().unwrap()).collect();
        let field = tube::Depth {tube:&tb,chart:Chart::Collar,within:tol,holes:&[]};
        for i in 0..=(v[3] as usize) {
            let psi = v[0]+(v[1]-v[0])*i as f64/v[3];
            let uv = [psi,v[2]];
            println!("ψ {:.4}° side {:?} reading {:?}",psi.to_degrees(),tb.side(Chart::Collar,uv),field.reading(uv).map(|r| match r.1 { Reading::Far(c) => format!("far {:.3e} at {:.6} (own {:.6})",c.depth,c.at,tb.own(Chart::Collar,uv)),o => format!("{o:?}") }));
        }
        return
    }
    // `fit`: each sheet's net over the window holding the pinch, and how far it stands from the
    // sheet outside the hole
    if args.iter().any(|a| a == "fit") {
        let p = tb.pinch.as_ref().expect("a pinch");
        let th = p.collar.at.1;
        let (w0,w1) = (th-2.,th+1.);
        for side in [1.,-1.] {
            let clock = Instant::now();
            let net = tube::faces::sheet_net(&tb,side,(w0,w1),0.1*tol).unwrap();
            let el = clock.elapsed();
            let mut worst = 0f64;
            let n = 600;
            for i in 0..=n { for j in 0..=n/2 {
                let (u,v) = ((i as f64+0.37)/(n as f64+1.),(j as f64+0.61)/(n as f64/2.+1.));
                let (a,b) = (w0+(w1-w0)*u,roll[0]+(roll[1]-roll[0])*v);
                if p.side([a,b]) < 0. { continue }
                let x = tb.point(side,a,b).unwrap().x;
                let y = net.point(u,v);
                worst = worst.max(((x[0]-y[0]).powi(2)+(x[1]-y[1]).powi(2)+(x[2]-y[2]).powi(2)).sqrt());
            } }
            println!("side {side:+}: {}×{} poles in {el:?}; outside the hole the sheet within {worst:e} (bar {:e})",net.poles.len(),net.poles[0].len(),0.1*tol);
        }
        return
    }
    // `pinch`: each chart's cuspidal edges, swallowtail points and depth's zero set about the pinch
    if args.iter().any(|a| a == "pinch") {
        let charts = if std::env::var_os("ENDS").is_some() { vec![Chart::End(0),Chart::End(1)] } else { vec![Chart::Sheet(1.),Chart::Sheet(-1.),Chart::Collar] };
        for chart in charts {
            let clock = Instant::now();
            let (folds,search) = match tube::edges(&tb,chart,tol,0.25*ring.thickness) { Ok(e) => e,Err(e) => { println!("{chart:?}: edges refused: {e:?}"); continue } };
            let tails: Vec<[f64;2]> = folds.iter().flat_map(|f| tube::swallowtails_on(&tb,chart,&f.uv)).collect();
            println!("{chart:?}: {} edge pieces in {:?}, search {:?}; swallowtail points {tails:?}",folds.len(),clock.elapsed(),
                search.iter().map(|s| (s.free,s.monotone,s.beside,s.sampled,s.unresolved.len())).collect::<Vec<_>>());
            for p in &folds { println!("  edge {} points {:?} ({:.5},{:.5}) .. ({:.5},{:.5})",p.uv.len(),p.ends,p.uv[0][0],p.uv[0][1],p.uv[p.uv.len()-1][0],p.uv[p.uv.len()-1][1]) }
            // (`GAUGE`: each sheet edge's points with `J` and the gauge `∇J · k`, its sense carried)
            if let (Chart::Sheet(side),true) = (chart,std::env::var_os("GAUGE").is_some()) {
                for p in &folds {
                    let mut sense = [1.,0.];
                    for q in &p.uv {
                        let (Some(k),Some(g)) = (tb.kernel(side,q[0],q[1]),tb.jacobian_gradient(side,q[0],q[1])) else { println!("    {q:?} none"); continue };
                        let k = if k[0]*sense[0]+k[1]*sense[1] < 0. { [-k[0],-k[1]] } else { k };
                        sense = k;
                        println!("    ({:.6},{:.6}) J {:.3e} gauge {:+.4e} k {:?}",q[0],q[1],tb.jacobian(side,q[0],q[1]).unwrap_or(f64::NAN),g[0]*k[0]+g[1]*k[1],k);
                    }
                }
            }
            if !tracing { continue }
            let clock = Instant::now();
            let lines: Vec<Vec<[f64;2]>> = folds.iter().map(|p| p.uv.clone()).collect();
            println!("  beside the edges: {:?}",tube::beside_edges(&tb,chart,&lines,tol,0.25*ring.thickness));
            let extra: Vec<[f64;2]> = std::env::var("SEEDS").ok().map(|v| { let x: Vec<f64> = v.split(',').map(|x| x.parse().unwrap()).collect(); x.chunks(2).map(|c| [c[0],c[1]]).collect() }).unwrap_or_default();
            match tube::seams(&tb,chart,&lines,&tails,&extra,tol,0.25*ring.thickness) {
                Ok((pieces,_,discs)) => {
                    for d in &discs { println!("  disc about {:?}: radius {:e}",d.at,d.radius) }
                    println!("{chart:?}: depth zero set {} pieces in {:?}",pieces.len(),clock.elapsed());
                    for p in &pieces {
                        println!("  {} points, ends {:?}, ({:.6}, {:.6}) .. ({:.6}, {:.6}), covered at {:.6} .. {:.6}, own {:.6} .. {:.6}",p.uv.len(),p.ends,p.uv[0][0],p.uv[0][1],
                            p.uv[p.uv.len()-1][0],p.uv[p.uv.len()-1][1],p.at[0].at,p.at[p.at.len()-1].at,tb.own(chart,p.uv[0]),tb.own(chart,p.uv[p.uv.len()-1]));
                    }
                }
                Err(e) => println!("{chart:?}: depth zero set refused in {:?}: {e:?}",clock.elapsed()),
            }
        }
        return
    }
    println!("tube: radius {} height {} r {} roll {:?} tol {:e}",tb.radius,tb.height,tb.r,roll,tol);
    // `bound`: the Jacobian's proved sign over cells against `J` read inside them
    if args.iter().any(|a| a == "bound") {
        println!("rates {:?}",tb.rates);
        for side in [1.,-1.] {
            let (mut proved,mut cells,mut wrong) = (0,0,0);
            for &(nu,nv) in &[(16usize,8usize),(64,32),(256,128)] {
                for i in 0..nu { for j in 0..nv {
                    let c = [[TAU*i as f64/nu as f64,TAU*(i+1) as f64/nu as f64],[roll[0]+(roll[1]-roll[0])*j as f64/nv as f64,roll[0]+(roll[1]-roll[0])*(j+1) as f64/nv as f64]];
                    cells += 1;
                    let Some(b) = tb.jacobian_bound(side,c) else { continue };
                    proved += 1;
                    let [lo,hi] = b.value.bounds().unwrap();
                    for p in 0..=4 { for q in 0..=4 {
                        let (th,t) = (c[0][0]+(c[0][1]-c[0][0])*p as f64/4.,c[1][0]+(c[1][1]-c[1][0])*q as f64/4.);
                        let jv = tb.jacobian(side,th,t).unwrap();
                        if jv < lo || jv > hi { wrong += 1; if wrong < 5 { println!("  J {jv} outside [{lo}, {hi}] at ({th},{t})") } }
                    } }
                } }
                println!("side {side:+} {nu}×{nv}: {proved} of {cells} proved so far, {wrong} wrong");
            }
            let clock = Instant::now();
            let e = tube::edges(&tb,Chart::Sheet(side),tol,0.25*ring.thickness);
            println!("side {side:+}: edges in {:?}: {:?}",clock.elapsed(),e.as_ref().map(|(p,s)| (p.len(),s.iter().map(|s| (s.free,s.monotone,s.beside,s.sampled,s.unresolved.len())).collect::<Vec<_>>())));
        }
        return
    }
    // `compare`: the tube's certified cover against `depth::cover` on a grid over each sheet
    if args.iter().any(|a| a == "compare") {
        for side in [1.,-1.] {
            let (mut worst,mut apart,mut n) = (0f64,0usize,0usize);
            let (mut t_new,mut t_old) = (std::time::Duration::ZERO,std::time::Duration::ZERO);
            for j in 0..=nt { for i in 0..nth {
                let (th,t) = (TAU*(i as f64+0.5)/nth as f64,roll[0]+(roll[1]-roll[0])*j as f64/nt as f64);
                let Some(p) = tb.point(side,th,t) else { continue };
                let c = Instant::now();
                let new = tb.cover(p.x,t,tol);
                t_new += c.elapsed();
                let c = Instant::now();
                let old = cad_kernel::swept::depth::cover(&ring,&path,roll,p.x,t,0.,tol).ok();
                t_old += c.elapsed();
                n += 1;
                match (new,old) {
                    (Some(Reading::Far(a)),Some(Reading::Far(b))) => { if (a.depth-b.depth).abs() > tol { apart += 1; if apart <= 8 { println!("  ({th:.4},{t:.4}) new {:.3e} at {:.5}, old {:.3e} at {:.5}",a.depth,a.at,b.depth,b.at) } } worst = worst.max((a.depth-b.depth).abs()) }
                    (Some(Reading::Near),Some(Reading::Near)) | (Some(Reading::Alone),Some(Reading::Alone)) => {}
                    (a,b) => {
                        apart += 1;
                        if apart <= 8 {
                            println!("  ({th:.4},{t:.4}) new {a:?} old {b:?}");
                            // the oracle: the depth sampled densely each way from the point's own instant
                            for (lo,hi) in [(roll[0],t),(t,roll[1])] {
                                let n = 200_000;
                                let ds: Vec<(f64,f64)> = (0..=n).map(|k| { let s = lo+(hi-lo)*k as f64/n as f64; (s,tb.depth_at(p.x,s).map_or(f64::NAN,|r| r.0)) }).collect();
                                let top = ds.iter().cloned().fold((0.,f64::NEG_INFINITY),|m,x| if x.1 > m.1 { x } else { m });
                                println!("    oracle over [{lo:.5}, {hi:.5}]: greatest {:.12} at {:.6}, ends {:.12} and {:.12}",top.1,top.0,ds[0].1,ds[n].1);
                            }
                        }
                    }
                }
            } }
            let c = Instant::now();
            let dom = cad_kernel::util::interval::Interval::new(roll[0],roll[1]).unwrap();
            let mut acc = 0.;
            for _ in 0..1000 { acc += path.inverse_point_speed_bound([1.,2.,3.],dom).unwrap()+path.inverse_point_acceleration_bound([1.,2.,3.],dom).unwrap() }
            println!("bounds {:?} each ({acc})",c.elapsed()/1000);
            let c = Instant::now();
            for k in 0..1000 { acc += path.at(k as f64*1e-3).unwrap().inverse().point([1.,2.,3.])[0] }
            println!("at {:?} each ({acc})",c.elapsed()/1000);
            let c = Instant::now();
            for k in 0..1000 { acc += path.pose(k as f64*1e-3).unwrap().inverse().point([1.,2.,3.])[0] }
            println!("pose {:?} each ({acc})",c.elapsed()/1000);
            println!("side {side:+}: {n} points, {apart} apart, worst depth apart {worst:e}; new {:?} a cover, old {:?}",t_new/n as u32,t_old/n as u32);
        }
        return
    }
    // `old`: the solid by the old ring road (`closed::ring`), its volume and time
    if args.iter().any(|a| a == "old") {
        let clock = Instant::now();
        match cad_kernel::swept::closed::ring(&tool,&path,roll,tube_r/250.,tol) {
            Ok(s) => println!("the old road: {} faces, volume {:.10}, in {:?}",s.faces.len(),cad_kernel::props::volume(&s),clock.elapsed()),
            Err(e) => println!("the old road refused in {:?}: {e:?}",clock.elapsed()),
        }
        return
    }
    // `build`: the solid by the tube road, its faces, volume and time, and the certified covers it asked
    if args.iter().any(|a| a == "build") {
        let clock = Instant::now();
        let solid = tube::faces::solid(&tb,tol);
        let covers = tube::covers_read();
        match solid {
            Ok(s) => {
                let took = clock.elapsed();
                println!("built: {} faces, volume {:.10}, check {:?}, in {:?}; {covers} covers, {:?} a cover",s.faces.len(),
                    cad_kernel::props::volume(&s),s.check(tol).err(),took,took/covers.max(1) as u32);
                // the misfits at the size's tolerance (`geom::tolerance` over the box's diagonal)
                let (lo,hi) = s.bounds();
                let size = ((hi[0]-lo[0]).powi(2)+(hi[1]-lo[1]).powi(2)+(hi[2]-lo[2]).powi(2)).sqrt();
                let m = s.misfits(cad_kernel::geom::tolerance(size));
                println!("misfits at {:e}: {}",cad_kernel::geom::tolerance(size),m.len());
                for (what,d,held) in &m { println!("  {what}: {d:e} (held to {held:e})") }
                // (`AUDIT=δ`: each mesh vertex moved δ in and out along its triangles' normal, read
                // by the depth — the least over the roll, sampled and settled — the disagreements by face)
                if let Some(delta) = std::env::var("AUDIT").ok().and_then(|d| d.parse::<f64>().ok()) {
                    let m = cad_kernel::mesh::mesh(&s,1e-4,0.1).unwrap();
                    let mut nrm = vec![[0f64;3];m.pts.len()];
                    let mut face = vec![0u32;m.pts.len()];
                    for (t,&f) in m.tris.iter().zip(&m.of) {
                        let [a,b,c] = t.map(|i| m.pts[i as usize]);
                        let n = cad_kernel::util::space::cross(cad_kernel::util::space::sub(b,a),cad_kernel::util::space::sub(c,a));
                        for &i in t { for k in 0..3 { nrm[i as usize][k] += n[k] } face[i as usize] = f }
                    }
                    let [a,b] = roll;
                    let depth = |x: [f64;3]| -> f64 {
                        let d = |t: f64| tb.depth_at(x,t).map_or(f64::INFINITY,|r| r.0);
                        let n = 512;
                        let (i,_) = (0..=n).map(|i| d(a+(b-a)*i as f64/n as f64)).enumerate().min_by(|p,q| p.1.total_cmp(&q.1)).unwrap();
                        let st = (b-a)/n as f64;
                        let (mut l,mut h) = ((a+st*i as f64-st).max(a),(a+st*i as f64+st).min(b));
                        let g = 0.5*(5f64.sqrt()-1.);
                        while h-l > 1e-13 { let (x1,x2) = (h-g*(h-l),l+g*(h-l)); if d(x1) < d(x2) { h = x2 } else { l = x1 } }
                        d(0.5*(l+h))
                    };
                    let body = cad_kernel::swept::member::Solid::new(&s,1e-4,1.).unwrap();
                    let mut bad: std::collections::BTreeMap<u32,(usize,usize,[f64;3])> = std::collections::BTreeMap::new();
                    for (i,p) in m.pts.iter().enumerate().step_by(8) {
                        let l = cad_kernel::util::space::norm(nrm[i]);
                        if !(l > 0.) { continue }
                        let n = cad_kernel::util::space::scale(nrm[i],1./l);
                        for sgn in [-1.,1.] {
                            let q = cad_kernel::util::space::add(*p,cad_kernel::util::space::scale(n,sgn*delta));
                            let d = depth(q);
                            if d.abs() < 0.25*delta { continue }
                            let sd = body.signed_distance(q);
                            if (d < 0.) != (sd < 0.) {
                                let e = bad.entry(face[i]).or_insert((0,0,q));
                                if d > 0. { e.0 += 1 } else { e.1 += 1 }
                                e.2 = q;
                            }
                        }
                    }
                    println!("audit at δ {delta:e} over {} vertices:",m.pts.len());
                    for (f,(i,o,p)) in &bad { if i+o > 0 { println!("  face {f} ({}): {i} in the solid, outside by the depth; {o} out of the solid, inside by the depth; e.g. {p:?}",s.faces[*f as usize].name) } }
                }
            }
            Err(e) => println!("refused in {:?} ({covers} covers): {e:?}",clock.elapsed()),
        }
        return
    }
    if let Some(at) = std::env::var_os("TUBE_AT") {
        let v: Vec<f64> = at.to_str().unwrap().split(',').map(|x| x.parse().unwrap()).collect();
        let (side,th0,t0,dth,dt) = (v[0],v[1],v[2],v[3],v[4]);
        let field = tube::Depth {tube:&tb,chart:Chart::Sheet(side),within:tol,holes:&[]};
        {
            if let Some(v) = cad_kernel::zero_set::Field::least(&field,[th0,t0]) {
            let g = v.grad.unwrap();
            let l = g[0].hypot(g[1]);
            let tan = [-g[1]/l,g[0]/l];
            let k = tb.kernel(side,th0,t0).unwrap();
            let p = tb.point(side,th0,t0).unwrap();
            let sp = |d: [f64;2]| { let w = [0,1,2].map(|i| p.x_th[i]*d[0]+p.x_t[i]*d[1]); (w[0]*w[0]+w[1]*w[1]+w[2]*w[2]).sqrt() };
            println!("tangent {tan:?} kernel {k:?} |X·tangent| {:e} |X·kernel| {:e} |X_θ| {:e} |X_t| {:e} cover at {}",sp(tan),sp(k),sp([1.,0.]),sp([0.,1.]),v.at.at); }
        }
        for i in -4..=4 { for j in -4..=4 {
            let (th,t) = (th0+dth*i as f64/4.,t0+dt*j as f64/4.);
            let r = tb.reading(side,th,t,tol);
            let f = cad_kernel::zero_set::Field::least(&field,[th,t]);
            if let Some(p) = &tb.pinch { print!("[hole {:+.3e}] ",p.side([th,t])) }
            println!("({th:.6},{t:.6}) J {:.3e} reading {:?} least {:?}",tb.jacobian(side,th,t).unwrap(),r.map(|r| match r { cad_kernel::swept::depth::Reading::Far(c) => format!("far {:.3e} at {:.6}",c.depth,c.at),o => format!("{o:?}") }),
                f.map(|v| (v.f,v.grad)));
        } }
        return
    }
    for side in [1.,-1.] {
        // the sheet fitted as a map over the whole rectangle
        let clock = Instant::now();
        let f = |u: f64,v: f64| tb.point(side,u*TAU,roll[0]+v*(roll[1]-roll[0])).map_or([f64::NAN;3],|p| p.x);
        let fit = cad_kernel::nurbs::fit_net(&f,5,5,32,32,0.1*tol);
        match &fit {
            Some((net,err)) => println!("side {side:+}: fit {}×{} poles, error {err:e} (bar {:e}) in {:?}",net.poles.len(),net.poles[0].len(),0.1*tol,clock.elapsed()),
            None => println!("side {side:+}: no fit"),
        }
        // the cuspidal edges and their swallowtail points
        let clock = Instant::now();
        let mut folds: Vec<Vec<[f64;2]>> = Vec::new();
        let mut tails: Vec<[f64;2]> = Vec::new();
        match tube::edges(&tb,Chart::Sheet(side),tol,0.25*ring.thickness) {
            Ok((pieces,search)) => {
                folds = pieces.iter().map(|p| p.uv.clone()).collect();
                tails = pieces.iter().flat_map(|p| tube::swallowtails(&tb,side,&p.uv)).collect();
                println!("side {side:+}: {} cuspidal-edge pieces in {:?}; search {:?}",pieces.len(),clock.elapsed(),
                    search.iter().map(|s| (s.free,s.monotone,s.beside,s.sampled,s.unresolved.len())).collect::<Vec<_>>());
                for p in &pieces {
                    let st = tube::swallowtails(&tb,side,&p.uv);
                    println!("  edge {} points, ends {:?}, from ({:.6}, {:.6}) to ({:.6}, {:.6}); swallowtails {:?}",p.uv.len(),p.ends,
                        p.uv[0][0],p.uv[0][1],p.uv[p.uv.len()-1][0],p.uv[p.uv.len()-1][1],st);
                }
            }
            Err(e) => println!("side {side:+}: edges refused: {e:?}"),
        }
        // the map: J's sign against the reading
        let clock = Instant::now();
        let mut grid = vec![vec![0u8;nth];nt+1];
        let (mut counts,mut disagree) = ([0usize;5],0usize);
        for (j,row) in grid.iter_mut().enumerate() {
            let t = roll[0]+(roll[1]-roll[0])*j as f64/nt as f64;
            for (i,cell) in row.iter_mut().enumerate() {
                let th = TAU*(i as f64+0.5)/nth as f64;
                let Some(jac) = tb.jacobian(side,th,t) else { *cell = 4; counts[4] += 1; continue };
                let r = tb.reading(side,th,t,tol);
                let k = match r { Some(Reading::Near) => 0,Some(Reading::Alone) => 1,Some(Reading::Far(c)) if c.depth >= 0. => 1,Some(Reading::Far(_)) => 2,None => 4 };
                if (jac > 0.) == (k == 0) && k != 4 && j > 0 && j < nt { disagree += 1 }
                *cell = k;
                counts[k as usize] += 1;
            }
        }
        // the kept regions: components of the map's kept cells, θ periodic
        let mut label = vec![vec![usize::MAX;nth];nt+1];
        let mut regions = Vec::new();
        for j0 in 0..=nt { for i0 in 0..nth {
            if grid[j0][i0] != 1 || label[j0][i0] != usize::MAX { continue }
            let id = regions.len();
            let mut stack = vec![(j0,i0)];
            label[j0][i0] = id;
            let mut n = 0;
            while let Some((j,i)) = stack.pop() {
                n += 1;
                let mut nb = vec![(j,(i+1)%nth),(j,(i+nth-1)%nth)];
                if j > 0 { nb.push((j-1,i)) }
                if j < nt { nb.push((j+1,i)) }
                for (a,b) in nb { if grid[a][b] == 1 && label[a][b] == usize::MAX { label[a][b] = id; stack.push((a,b)) } }
            }
            regions.push(n);
        } }
        println!("side {side:+}: map {nth}×{} in {:?}: held by neighbours {}, kept {}, covered {}, unread {}; J's sign against the reading {} apart (inside rows); kept regions (cells) {:?}",
            nt+1,clock.elapsed(),counts[0],counts[1],counts[2],counts[4],disagree,regions);
        if nth <= 64 {
            for row in grid.iter().rev() { println!("  {}",row.iter().map(|&c| ['.','#','x','?','!'][c as usize]).collect::<String>()) }
        }
        if tracing {
            let clock = Instant::now();
            match tube::seams(&tb,Chart::Sheet(side),&folds,&tails,&[],tol,0.25*ring.thickness) {
                Ok((pieces,search,discs)) => {
                    for d in &discs { println!("  disc about {:?}: radius {:e}",d.at,d.radius) }
                    println!("side {side:+}: depth zero set {} pieces in {:?}; search {:?}",pieces.len(),clock.elapsed(),
                        search.iter().map(|s| (s.free,s.monotone,s.beside,s.sampled,s.unresolved.len())).collect::<Vec<_>>());
                    for p in &pieces {
                        println!("  {} points, ends {:?}, ({:.6}, {:.6}) .. ({:.6}, {:.6}), covered at {:.6} .. {:.6}",p.uv.len(),p.ends,p.uv[0][0],p.uv[0][1],
                            p.uv[p.uv.len()-1][0],p.uv[p.uv.len()-1][1],p.at[0].at,p.at[p.at.len()-1].at);
                    }
                }
                Err(e) => println!("side {side:+}: depth zero set refused in {:?}: {e:?}",clock.elapsed()),
            }
        }
    }
    // the end poses: where each stops bounding the sweep (its contact circles), and its depth's zero set
    for k in 0..2 {
        let chart = Chart::End(k);
        let clock = Instant::now();
        let circles = match tube::edges(&tb,chart,tol,0.25*ring.thickness) {
            Ok((pieces,search)) => {
                println!("end {k}: {} contact-circle pieces in {:?}; search {:?}",pieces.len(),clock.elapsed(),
                    search.iter().map(|s| (s.free,s.monotone,s.beside,s.sampled,s.unresolved.len())).collect::<Vec<_>>());
                for p in &pieces {
                    println!("  {} points, ends {:?}, ({:.6}, {:.6}) .. ({:.6}, {:.6})",p.uv.len(),p.ends,p.uv[0][0],p.uv[0][1],
                        p.uv[p.uv.len()-1][0],p.uv[p.uv.len()-1][1]);
                }
                pieces.iter().map(|p| p.uv.clone()).collect::<Vec<_>>()
            }
            Err(e) => { println!("end {k}: refused: {e:?}"); continue }
        };
        if tracing {
            let clock = Instant::now();
            match tube::seams(&tb,chart,&circles,&[],&[],tol,0.25*ring.thickness) {
                Ok((pieces,search,_)) => {
                    println!("end {k}: depth zero set {} pieces in {:?}; search {:?}",pieces.len(),clock.elapsed(),
                        search.iter().map(|s| (s.free,s.monotone,s.beside,s.sampled,s.unresolved.len())).collect::<Vec<_>>());
                    for p in &pieces {
                        println!("  {} points, ends {:?}, ({:.6}, {:.6}) .. ({:.6}, {:.6}), covered at {:.6} .. {:.6}",p.uv.len(),p.ends,p.uv[0][0],p.uv[0][1],
                            p.uv[p.uv.len()-1][0],p.uv[p.uv.len()-1][1],p.at[0].at,p.at[p.at.len()-1].at);
                    }
                }
                Err(e) => println!("end {k}: depth zero set refused in {:?}: {e:?}",clock.elapsed()),
            }
        }
    }
}
