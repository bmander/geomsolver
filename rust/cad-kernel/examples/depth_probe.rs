//! **The depth-based trim measured on brep-bench's tumbling tori** (#942): each double curve the
//! ring road traces read under `swept::depth::cover` — its points' far depth, the instant that covers
//! them against the partner's — and the far cover read about each swallowtail point.
//!
//! `cargo run --release -p cad-kernel --example depth_probe -- full-turn|half-turn|thin [curves|apex|map θ0 θ1 t0 t1 n|point x,y,z|sliver t…|ends [band] [x,y,z r]|ratio [every]]`
//! (`ends`: each double curve's points against the bridges' ends on their own rows; `ratio`: their
//! `s''` against the pair's least's depth, the level a bridge's end is put at; `sliver`: at each roll, each narrow pair's folds, how far apart, and how deep the sweep holds
//! the leaf between them — over the roll, and by the instants beside its own)
use cad_kernel::swept::closed::{RingFitted,ring_fitted};
use cad_kernel::swept::depth::{Reading,cover,far_depth,label_at,sheet};
use bench::tumbling;
#[path = "support/bench.rs"]
mod bench;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("edges") {
        // a dumped candidate's edges with an end within `r` of `x` (`edges <dump> c x,y,z r`): each
        // edge's ends, its kind, and every use — the face, and the use's ends in its parameters
        let d = cad_kernel::swept::dump::load_trim(std::path::Path::new(&args[1])).unwrap();
        let b = &d.candidates[args[2].parse::<usize>().unwrap()];
        let x: Vec<f64> = args[3].split(',').map(|v| v.parse().unwrap()).collect();
        let r: f64 = args[4].parse().unwrap();
        let uses = b.uses();
        for (ei,e) in b.edges.iter().enumerate() {
            let p = e.v.map(|v| b.vertices[v as usize].p);
            if !p.iter().any(|p| cad_kernel::util::space::distance(*p,[x[0],x[1],x[2]]) <= r) { continue }
            println!("edge {ei} vertices {:?} at {:?}",e.v,p);
            for &(fi,li,ci) in &uses[ei] {
                let f = &b.faces[fi];
                println!("  face {fi} loop {li}: {:?}",b.uv_ends(f,&f.loops[li][ci]));
            }
        }
        return
    }
    if args.first().map(String::as_str) == Some("rowgap") {
        // two dumped faces' iso-curves at the roll `t` (`rowgap <dump> c f c g t n`): at `n` places
        // along the first face's `u` span, how far its point stands from the second's iso-curve
        // (its nearest by a grid and golden sections over the second's `u` span)
        let d = cad_kernel::swept::dump::load_trim(std::path::Path::new(&args[1])).unwrap();
        let p: Vec<usize> = args[2..6].iter().map(|v| v.parse().unwrap()).collect();
        let (t,n): (f64,usize) = (args[6].parse().unwrap(),args[7].parse().unwrap());
        let span = |c: usize,f: usize| {
            let b = &d.candidates[c];
            let f = &b.faces[f];
            let uv: Vec<[f64;2]> = f.loops.iter().flatten().flat_map(|e| b.uv_ends(f,e)).collect();
            (uv.iter().map(|q| q[0]).fold(f64::INFINITY,f64::min),uv.iter().map(|q| q[0]).fold(f64::NEG_INFINITY,f64::max))
        };
        let (fa,fb) = (&d.candidates[p[0]].faces[p[1]].surface,&d.candidates[p[2]].faces[p[3]].surface);
        let ((a0,a1),(b0,b1)) = (span(p[0],p[1]),span(p[2],p[3]));
        for i in 0..=n {
            let u = a0+(a1-a0)*i as f64/n as f64;
            let x = fa.point([u,t]);
            let g = |v: f64| cad_kernel::util::space::distance(fb.point([v,t]),x);
            let k = (0..=2000).min_by(|&i,&j| g(b0+(b1-b0)*i as f64/2000.).total_cmp(&g(b0+(b1-b0)*j as f64/2000.))).unwrap();
            let (mut lo,mut hi) = (b0+(b1-b0)*(k.max(1)-1) as f64/2000.,b0+(b1-b0)*(k+1).min(2000) as f64/2000.);
            let gr = 0.5*(5f64.sqrt()-1.);
            for _ in 0..100 { let (x1,x2) = (hi-gr*(hi-lo),lo+gr*(hi-lo)); if g(x1) < g(x2) { hi = x2 } else { lo = x1 } }
            println!("u {u:.6} {x:.7?}: {:.3e} from the other's at {:.6}",g(0.5*(lo+hi)),0.5*(lo+hi));
        }
        return
    }
    if args.first().map(String::as_str) == Some("face") {
        // a dumped trim's candidate's face, its parameters `(u, t)` a tube's sheet's: the far cover
        // over its parameters' box — `.` uncovered, `f` covered less than the tolerance deep, `F`
        // more, `N` held by its neighbours, `a` alone
        let t = cad_kernel::swept::dump::load_trim(std::path::Path::new(&args[1])).unwrap();
        let (ci,fi,n): (usize,usize,usize) = (args[2].parse().unwrap(),args[3].parse().unwrap(),args[4].parse().unwrap());
        let ring = cad_kernel::swept::rings::Ring::read(&t.tool).unwrap();
        let b = &t.candidates[ci];
        let f = &b.faces[fi];
        let uv: Vec<[f64;2]> = f.loops.iter().flatten().flat_map(|c| b.uv_ends(f,c)).collect();
        let (lo,hi) = uv.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(l,h),p| ([l[0].min(p[0]),l[1].min(p[1])],[h[0].max(p[0]),h[1].max(p[1])]));
        let (lo,hi) = match args.get(5) { Some(w) => { let w: Vec<f64> = w.split(',').map(|v| v.parse().unwrap()).collect(); ([w[0],w[2]],[w[1],w[3]]) } None => (lo,hi) };
        println!("face ({ci}, {fi}) {}: u {:?} t {:?}",f.surface.kind(),[lo[0],hi[0]],[lo[1],hi[1]]);
        if let Some(x) = args.get(6) {
            // the face's parameters nearest the point `x` in the window, by a grid of a thousand a side
            let x: Vec<f64> = x.split(',').map(|v| v.parse().unwrap()).collect();
            let mut best = (f64::INFINITY,[0.;2]);
            for i in 0..=1000 { for j in 0..=1000 {
                let uv = [lo[0]+(hi[0]-lo[0])*i as f64/1000.,lo[1]+(hi[1]-lo[1])*j as f64/1000.];
                let d = cad_kernel::util::space::distance(f.surface.point(uv),[x[0],x[1],x[2]]);
                if d < best.0 { best = (d,uv) }
            } }
            println!("nearest {best:?}");
        }
        // (the point standing off its sheet by `off`: the tolerance unless given, `closed`'s tail
        // rule reading the fit's, a tenth of it)
        let off: f64 = args.get(7).map_or(t.tol,|v| v.parse().unwrap());
        let rows = cad_kernel::util::par::indices(n+1,|i| {
            let v = hi[1]-(hi[1]-lo[1])*i as f64/n as f64;
            let line: String = (0..=n).map(|j| {
                let u = lo[0]+(hi[0]-lo[0])*j as f64/n as f64;
                let x = f.surface.point([u,v]);
                match cover(&ring,&t.path,t.roll,x,v,off,t.tol) {
                    Ok(Reading::Near) => 'N',
                    Ok(Reading::Far(c)) => if c.depth < -t.tol { 'F' } else if c.depth < 0. { 'f' } else { '.' },
                    Ok(Reading::Alone) => 'a',
                    Err(_) => '!',
                }
            }).collect();
            format!("t {v:.6} {line}")
        });
        for r in rows { println!("{r}") }
        return
    }
    let (tube,(tool,path),half) = match args.first().map(String::as_str) {
        Some("full-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),180.),
        Some("half-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),90.),
        Some("thin") => (0.015,tumbling([3.,0.,2.],2.,0.015,0.25,45.,0.,0.5f64.sqrt(),1.),75.),
        _ => { eprintln!("depth_probe full-turn|half-turn|thin …"); std::process::exit(2) }
    };
    let roll = [-(half as f64).to_radians(),(half as f64).to_radians()];
    let tol = tube/50000.;
    let what = args.get(1).map_or("curves",String::as_str);
    let ring = cad_kernel::swept::rings::Ring::read(&tool).unwrap();
    match what {
        "point" => {
            let x: Vec<f64> = args[2].split(',').map(|v| v.parse().unwrap()).collect();
            let x = [x[0],x[1],x[2]];
            let ts: Vec<f64> = (0..=400_000).map(|j| roll[0]+(roll[1]-roll[0])*j as f64/400_000.).collect();
            let ds = cad_kernel::swept::depth::depths(&ring,&path,x,&ts);
            let mut order: Vec<usize> = (1..ds.len()-1).filter(|&j| ds[j] <= ds[j-1] && ds[j] <= ds[j+1]).collect();
            order.sort_by(|&i,&j| ds[i].total_cmp(&ds[j]));
            for &j in order.iter().take(4) { println!("a least of the depth {:+.3e} at {:.7}",ds[j],ts[j]) }
            for own in [roll[0],roll[1]] {
                println!("as a point of the pose at {own}: {:?}",cover(&ring,&path,roll,x,own,0.,tol));
            }
            return
        }
        "sliver" => {
            // at each roll given, each narrow pair's bridge: its angles, and the pair's sliver — the
            // angles between its folds (`s''` below nought), the folds' distance apart in space, and
            // how far the leaf between them stands from the chord joining them
            for t in args[2..].iter().map(|v| v.parse::<f64>().unwrap()) {
                let pieces = cad_kernel::swept::rings::pieces_at(&ring,&path,[t,roll[1]],t).unwrap();
                for (ci,c) in pieces.iter().enumerate() {
                    for &(from,to,l,side) in c {
                        if side != cad_kernel::swept::rings::Side::Fold { continue }
                        let n = 4000;
                        let pts: Vec<(f64,[f64;3],f64)> = (0..=n).filter_map(|k| {
                            let th = from+(to-from)*k as f64/n as f64;
                            sheet(&ring,&path,roll,th,t,l).map(|(x,b)| (th,x,b))
                        }).collect();
                        let neg: Vec<&(f64,[f64;3],f64)> = pts.iter().filter(|p| p.2 < 0.).collect();
                        let (Some(f0),Some(f1)) = (neg.first(),neg.last()) else { println!("t {t}: curve {ci} bridge [{from:.6}, {to:.6}], no fold"); continue };
                        let chord = cad_kernel::util::space::sub(f1.1,f0.1);
                        let len = cad_kernel::util::space::norm(chord);
                        let off = neg.iter().map(|p| {
                            let d = cad_kernel::util::space::sub(p.1,f0.1);
                            let along = if len > 0. { cad_kernel::util::space::dot(d,chord)/len } else { 0. };
                            (cad_kernel::util::space::dot(d,d)-along*along).max(0.).sqrt()
                        }).fold(0f64,f64::max);
                        println!("t {t}: curve {ci} bridge [{from:.6}, {to:.6}]; folds at {:.7} and {:.7} ({:.2e} apart), {:.3e} apart in space, the leaf {off:.3e} off their chord",
                            f0.0,f1.0,f1.0-f0.0,len);
                        // the sweep's depth at the leaf's points between the folds: the least over
                        // the roll of the tool's signed distance (its own instant nought), read at
                        // a twenty-thousandth of it about the instant and settled by golden sections
                        let held = |x: [f64;3]| {
                            let d = |tau: f64| cad_kernel::swept::depth::depths(&ring,&path,x,&[tau])[0];
                            let n = 20_000;
                            let ts: Vec<f64> = (0..=n).map(|k| roll[0]+(roll[1]-roll[0])*k as f64/n as f64).collect();
                            let ds = cad_kernel::swept::depth::depths(&ring,&path,x,&ts);
                            let k = (0..=n).min_by(|&i,&j| ds[i].total_cmp(&ds[j])).unwrap();
                            let (mut lo,mut hi) = (ts[k.saturating_sub(1)],ts[(k+1).min(n)]);
                            let g = 0.5*(5f64.sqrt()-1.);
                            for _ in 0..80 { let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo)); if d(x1) < d(x2) { hi = x2 } else { lo = x1 } }
                            let m = 0.5*(lo+hi);
                            (d(m).min(ds[k]),m)
                        };
                        let deepest = neg.iter().step_by((neg.len()/16).max(1)).map(|p| { let (h,at) = held(p.1); (h,p.0,at) })
                            .min_by(|a,b| a.0.total_cmp(&b.0)).unwrap();
                        println!("    between the folds held {:.3e} deep at its deepest (θ {:.6}, by the pose at {:.7})",deepest.0,deepest.1,deepest.2);
                        // and by the instants beside its own alone, within a hundredth of the roll
                        // of it: the fold's own thickness
                        let near = |x: [f64;3]| {
                            let ts: Vec<f64> = (0..=4000).map(|k| t-0.01+0.02*k as f64/4000.).collect();
                            cad_kernel::swept::depth::depths(&ring,&path,x,&ts).into_iter().fold(f64::INFINITY,f64::min)
                        };
                        let thick = neg.iter().map(|p| (near(p.1),p.0)).min_by(|a,b| a.0.total_cmp(&b.0)).unwrap();
                        println!("    held by its neighbours {:.3e} deep at its deepest (θ {:.6})",thick.0,thick.1);
                    }
                }
            }
            return
        }
        "line" => {
            // the depth along the roll at the sheet's point (θ, t, σ)
            let n: Vec<f64> = args[2..8].iter().map(|v| v.parse().unwrap()).collect();
            let (th,t,sg,lo,hi,k) = (n[0],n[1],n[2],n[3],n[4],n[5] as usize);
            let l = label_at(&ring,&path,th,t,sg).unwrap();
            let (x,_) = sheet(&ring,&path,roll,th,t,l).unwrap();
            let ts: Vec<f64> = (0..=k).map(|j| lo+(hi-lo)*j as f64/k as f64).collect();
            for (t,d) in ts.iter().zip(cad_kernel::swept::depth::depths(&ring,&path,x,&ts)) { println!("{t:.8} {d:+.4e}") }
            println!("{:?}",cover(&ring,&path,roll,x,t,0.,tol));
            return
        }
        "about" => {
            // each double curve about each swallowtail point on it: the points within `n` of it along
            // the curve, their angle and roll less the point's, and where the curve's rolls cross the
            // point's own roll (its row, if the roll is parted there)
            let n: usize = args.get(2).map_or(12,|v| v.parse().unwrap());
            let RingFitted {curves,..} = ring_fitted(&tool,&path,roll,tol).unwrap();
            for (ci,c) in curves.iter().enumerate() {
                let at = c.iter().position(|p| p.0.is_nan()).unwrap_or(c.len());
                for ap in &c[at+1..] {
                    let Some(k) = c[..at].iter().position(|p| p.0 == ap.0 && p.1 == ap.1) else { println!("curve {ci}: the point {ap:?} not on it"); continue };
                    println!("curve {ci} ({at} points): its point {k} at θ {:.7}, t {:.7}",ap.0,ap.1);
                    for j in k.saturating_sub(n)..(k+n+1).min(at) {
                        println!("  {j}: dθ {:+.3e}, dt {:+.3e}",c[j].0-ap.0,c[j].1-ap.1);
                    }
                    let crossings: Vec<usize> = (1..at).filter(|&j| (c[j-1].1-ap.1)*(c[j].1-ap.1) < 0.).collect();
                    println!("  the curve crosses the point's roll between points {crossings:?}");
                    let space = |q: (f64,f64,f64)| label_at(&ring,&path,q.0,q.1,q.2).and_then(|l| sheet(&ring,&path,roll,q.0,q.1,l)).map(|s| s.0);
                    println!("  the point in space {:?}",space(*ap));
                    for &j in &crossings {
                        for i in j.saturating_sub(3)..(j+3).min(at) { println!("  {i}: dθ {:+.4e}, dt {:+.4e}, at {:?}",c[i].0-ap.0,c[i].1-ap.1,space(c[i])) }
                    }
                    // each point of the curve between the swallowtail point and the next crossing of its
                    // row: how far it stands in space from the row's own contact (the sheet at the
                    // point's roll, its nearest place settled by golden sections over an angle's
                    // twentieth about the point's)
                    let Some(&j) = crossings.iter().min_by_key(|&&j| j.abs_diff(k)) else { continue };
                    let l = label_at(&ring,&path,ap.0,ap.1,ap.2).unwrap();
                    let row = |th: f64| sheet(&ring,&path,roll,th,ap.1,l).map_or([f64::NAN;3],|s| s.0);
                    let (lo,hi) = (k.min(j),k.max(j));
                    let step = ((hi-lo)/24).max(1);
                    for i in (lo..=hi).step_by(step) {
                        let Some(x) = space(c[i]) else { continue };
                        let f = |th: f64| cad_kernel::util::space::distance(row(th),x);
                        let (mut a0,mut b0) = (c[i].0-0.05,c[i].0+0.05);
                        let g = 0.5*(5f64.sqrt()-1.);
                        for _ in 0..100 { let (x1,x2) = (b0-g*(b0-a0),a0+g*(b0-a0)); if f(x1) < f(x2) { b0 = x2 } else { a0 = x1 } }
                        println!("  {i}: dt {:+.3e}, {:.3e} from the row's contact{}",c[i].1-ap.1,f(0.5*(a0+b0)),if f(0.5*(a0+b0)) <= tol { " (within the tolerance)" } else { "" });
                    }
                }
            }
            return
        }
        "ends" => {
            // each double curve's points against the bridges' ends on their own row: the nearest end
            // of a narrow pair's bridge round the axis, how far apart in space, and whether the point
            // stands inside the bridge (`ends [band] [x,y,z r]`: each point within `band` tolerances
            // printed, or within `r` of the place `x`)
            let band: f64 = args.get(2).map_or(8.,|v| v.parse().unwrap());
            let RingFitted {curves,..} = ring_fitted(&tool,&path,roll,tol).unwrap();
            let tau = std::f64::consts::TAU;
            for (ci,c) in curves.iter().enumerate() {
                let at = c.iter().position(|p| p.0.is_nan()).unwrap_or(c.len());
                let mut least = (f64::INFINITY,0);
                for (k,&(th,t,sg)) in c[..at].iter().enumerate() {
                    let Some(l) = label_at(&ring,&path,th,t,sg) else { continue };
                    let Some((x,_)) = sheet(&ring,&path,roll,th,t,l) else { continue };
                    let Ok(pieces) = cad_kernel::swept::rings::pieces_at(&ring,&path,[t,roll[1]],t) else { println!("  {k}: unread"); continue };
                    let mut best: Option<(f64,f64,bool,f64)> = None;
                    for pc in &pieces { for &(from,to,pl,side) in pc {
                        if side != cad_kernel::swept::rings::Side::Fold { continue }
                        let inside = { let d = (th-from).rem_euclid(tau); d <= (to-from).abs() };
                        for e in [from,to] {
                            let Some((y,_)) = sheet(&ring,&path,roll,e,t,pl) else { continue };
                            let d = cad_kernel::util::space::distance(x,y);
                            let dth = { let d = (th-e).rem_euclid(tau); d.min(tau-d) };
                            if best.is_none_or(|b| d < b.0) { best = Some((d,dth,inside,e)) }
                        }
                    } }
                    let Some((d,dth,inside,e)) = best else { continue };
                    if d < least.0 { least = (d,k) }
                    let near = args.get(3).map(|v| { let q: Vec<f64> = v.split(',').map(|z| z.parse().unwrap()).collect();
                        cad_kernel::util::space::distance(x,[q[0],q[1],q[2]]) <= args[4].parse::<f64>().unwrap() });
                    if d <= band*tol || near == Some(true) { println!("  curve {ci} point {k}: θ {th:.7} t {t:.7}: {:.2} tol from the end at {e:.7} (dθ {dth:.2e}){}",d/tol,if inside { ", inside" } else { "" }) }
                }
                println!("curve {ci}: {at} points; nearest a bridge's end {:.3e} ({:.2} tol) at point {}",least.0,least.0/tol,least.1);
            }
            return
        }
        "ratio" => {
            // each double curve's points' `s''` against their row's nearest narrow pair's least: the
            // level the bridge's end is put at is 1, the swallowtail's normal form puts the curve at 2
            // (`ratio [every]`)
            let every: usize = args.get(2).map_or(25,|v| v.parse().unwrap());
            let RingFitted {curves,..} = ring_fitted(&tool,&path,roll,tol).unwrap();
            let tau = std::f64::consts::TAU;
            for (ci,c) in curves.iter().enumerate() {
                let at = c.iter().position(|p| p.0.is_nan()).unwrap_or(c.len());
                let (mut lo,mut hi) = ((f64::INFINITY,0),(f64::NEG_INFINITY,0));
                for (k,&(th,t,sg)) in c[..at].iter().enumerate() {
                    let Some(l) = label_at(&ring,&path,th,t,sg) else { continue };
                    let Some((_,bend)) = sheet(&ring,&path,roll,th,t,l) else { continue };
                    let Ok(pieces) = cad_kernel::swept::rings::pieces_at(&ring,&path,[t,roll[1]],t) else { continue };
                    let apart = |e: f64| { let d = (th-e).rem_euclid(tau); d.min(tau-d) };
                    let Some(&(from,to,pl,_)) = pieces.iter().flatten().filter(|p| p.3 == cad_kernel::swept::rings::Side::Fold)
                        .min_by(|p,q| apart(0.5*(p.0+p.1)).total_cmp(&apart(0.5*(q.0+q.1)))) else { continue };
                    let least = (0..=400).filter_map(|j| sheet(&ring,&path,roll,from+(to-from)*j as f64/400.,t,pl).map(|s| s.1)).fold(f64::INFINITY,f64::min);
                    let rho = bend/least.abs();
                    if rho < lo.0 { lo = (rho,k) }
                    if rho > hi.0 { hi = (rho,k) }
                    if k%every == 0 { println!("  curve {ci} point {k}: θ {th:.6} t {t:.6}: s''/|least| {rho:+.4}, bridge [{from:.6}, {to:.6}]") }
                }
                println!("curve {ci}: least ratio {:.4} at {}, greatest {:.4} at {}",lo.0,lo.1,hi.0,hi.1);
            }
            return
        }
        "tail" => {
            // each double curve's preimage points: at its angle, the stretch of the roll from it to
            // the fold (`s''` nought) on the side `s''` falls, its far depth's least there and the
            // stretch's length in space
            let RingFitted {ring,curves,..} = ring_fitted(&tool,&path,roll,tol).unwrap();
            for c in &curves {
                let at = c.iter().position(|p| p.0.is_nan()).unwrap_or(c.len());
                let apexes = &c[at+1..];
                for &(th,t,sg) in c[..at].iter() {
                    let near = apexes.iter().map(|a| ((a.0-th).powi(2)+(a.1-t).powi(2)).sqrt()).fold(f64::INFINITY,f64::min);
                    let l = label_at(&ring,&path,th,t,sg).unwrap();
                    let bend = |t: f64| sheet(&ring,&path,roll,th,t,l).map_or(f64::NAN,|s| s.1);
                    let Some(x0) = sheet(&ring,&path,roll,th,t,l) else { continue };
                    let h = 1e-6;
                    let dir = if bend(t+h) < bend(t-h) { 1. } else { -1. };
                    let mut far = t;
                    let mut step = 1e-5;
                    while bend(far+dir*step) > 0. && (far-t).abs() < 0.2 { far += dir*step; step *= 1.5 }
                    let (mut lo,mut hi) = (far,far+dir*step);
                    for _ in 0..60 { let m = 0.5*(lo+hi); if bend(m) > 0. { lo = m } else { hi = m } }
                    let tf = lo;
                    let xf = sheet(&ring,&path,roll,th,tf,l).unwrap().0;
                    let width = cad_kernel::util::space::distance(x0.0,xf);
                    let mut least = (f64::INFINITY,t);
                    for j in 0..=64 {
                        let s = t+(tf-t)*j as f64/64.;
                        if let Some(cv) = far_depth(&ring,&path,roll,th,s,l,tol) { if cv.depth < least.0 { least = (cv.depth,s) } }
                    }
                    println!("θ {th:.6} t {t:.6} from the apex {near:.2e}: fold at {tf:.6}, width {width:.3e}, least far depth {:+.3e}",least.0);
                }
            }
            return
        }
        "rows" => {
            // per row of angle, the stretches of the roll where the touching sheet is covered more
            // than the tolerance deep by another instant (`F`) or held by its neighbours (`N`, the
            // far side of a fold), and the far depth's least
            let n: Vec<f64> = args[2..9].iter().map(|v| v.parse().unwrap()).collect();
            let (th0,th1,nth,t0,t1,nt,sigma) = (n[0],n[1],n[2] as usize,n[3],n[4],n[5] as usize,n[6]);
            let l = label_at(&ring,&path,0.5*(th0+th1),0.5*(t0+t1),sigma).unwrap();
            println!("label {l:?}");
            let out = cad_kernel::util::par::indices(nth+1,|i| {
                let th = th0+(th1-th0)*i as f64/nth as f64;
                let mut line = String::new();
                let mut last = ' ';
                let mut least = f64::INFINITY;
                for j in 0..=nt {
                    let t = t0+(t1-t0)*j as f64/nt as f64;
                    let c = match sheet(&ring,&path,roll,th,t,l) {
                        None => '?',
                        Some((x,_)) => match cover(&ring,&path,roll,x,t,0.,tol) {
                            Ok(Reading::Near) => 'N',
                            Ok(Reading::Far(cv)) => { least = least.min(cv.depth); if cv.depth < -tol { 'F' } else { '.' } }
                            Ok(Reading::Alone) => '.',
                            Err(_) => '!',
                        }
                    };
                    if c != last { line += &format!(" {c}@{t:.6}"); last = c }
                }
                format!("{th:.6}:{line}  least {least:+.2e}")
            });
            for o in out { println!("{o}") }
            return
        }
        "pose" => {
            // the tool at the start (`0`) or end (`1`) pose: a window of its torus's angles about the
            // point nearest `x`, each place's reading as a point of that pose (`E` covered by the other end's
            // pose itself, `o` within the tolerance of nought)
            let end: usize = args[2].parse().unwrap();
            let x: Vec<f64> = args[3].split(',').map(|v| v.parse().unwrap()).collect();
            let (half_width,n): (f64,usize) = (args[4].parse().unwrap(),args[5].parse().unwrap());
            let own = roll[end];
            let pose = path.pose(own).unwrap();
            let p = pose.inverse().point([x[0],x[1],x[2]]);
            let (c,tube_r) = ([3.,0.,2.],tube);
            let phi = (p[1]-c[1]).atan2(p[0]-c[0]);
            let rho = (p[0]-c[0]).hypot(p[1]-c[1]);
            let psi = (p[2]-c[2]).atan2(rho-1.);
            let at = |phi: f64,psi: f64| {
                let r = 1.+tube_r*psi.cos();
                pose.point([c[0]+r*phi.cos(),c[1]+r*phi.sin(),c[2]+tube_r*psi.sin()])
            };
            let x0 = at(phi,psi);
            println!("nearest {x0:?} (φ {phi:.6}, ψ {psi:.6}): {:?}",cover(&ring,&path,roll,x0,own,0.,tol));
            let rows = cad_kernel::util::par::indices(n+1,|i| {
                let ps = psi-half_width+2.*half_width*i as f64/n as f64;
                let mut least = f64::INFINITY;
                let line: String = (0..=n).map(|j| {
                    let ph = phi-half_width+2.*half_width*j as f64/n as f64;
                    match cover(&ring,&path,roll,at(ph,ps),own,0.,tol) {
                        Ok(Reading::Near) => 'N',
                        Ok(Reading::Far(cv)) => { least = least.min(cv.depth); if cv.depth < -tol { if (cv.at-roll[1-end]).abs() == 0. { 'E' } else { 'F' } } else if cv.depth < tol { 'o' } else { '.' } }
                        Ok(Reading::Alone) => 'a',
                        Err(_) => '!',
                    }
                }).collect();
                format!("ψ {ps:+.5} {line} least {least:+.2e}")
            });
            for r in rows { println!("{r}") }
            return
        }
        "meet" => {
            // the tool posed at the start's meeting with the tool posed at the end: the zero set of the
            // end pose's signed distance over the start pose's face (`zero_set::trace`), each piece's
            // ends and, along it, each point's reading as a point of either pose (`.` uncovered, `o`
            // within the tolerance, `F` covered, `N` held by its neighbours)
            use cad_kernel::zero_set::{Bound,Field,Question,Value,trace};
            let poses = [path.pose(roll[0]).unwrap(),path.pose(roll[1]).unwrap()];
            let start = tool.moved(&cad_kernel::geom::Rigid {r:poses[0].r,t:poses[0].p});
            let f = &start.faces[0];
            let uv: Vec<[f64;2]> = f.loops.iter().flatten().flat_map(|c| start.uv_ends(f,c)).collect();
            let (lo,hi) = uv.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(l,h),p| ([l[0].min(p[0]),l[1].min(p[1])],[h[0].max(p[0]),h[1].max(p[1])]));
            // (or a window of half-width `w` about the point `x` of the face)
            let (lo,hi) = match (args.get(2),args.get(3)) {
                (Some(x),Some(w)) => {
                    let x: Vec<f64> = x.split(',').map(|v| v.parse().unwrap()).collect();
                    let w: f64 = w.parse().unwrap();
                    let c = f.surface.inverse([x[0],x[1],x[2]]);
                    ([c[0]-w,c[1]-w],[c[0]+w,c[1]+w])
                }
                _ => (lo,hi),
            };
            struct G<'a> { s: &'a cad_kernel::geom::Surface,ring: &'a cad_kernel::swept::rings::Ring,back: cad_kernel::trajectory::Pose }
            impl Field for G<'_> {
                type At = ();
                fn least(&self,uv: [f64;2]) -> Option<Value<()>> {
                    let (x,su,sv) = self.s.d1(uv);
                    let h = 1e-7;
                    let g = |x: [f64;3]| self.ring.signed(self.back.inverse().point(x)).0;
                    let d = |e: [f64;3]| (g(cad_kernel::util::space::add(x,cad_kernel::util::space::scale(e,h)))-g(cad_kernel::util::space::sub(x,cad_kernel::util::space::scale(e,h))))/(2.*h);
                    Some(Value {f:g(x),grad:Some([d(su),d(sv)]),at:()})
                }
                fn along(&self,uv: [f64;2],_: &()) -> Option<Value<()>> { self.least(uv) }
                fn surface(&self,uv: [f64;2]) -> ([f64;3],[f64;3],[f64;3]) { self.s.d1(uv) }
                fn bound(&self,c: [[f64;2];2]) -> Option<Bound> {
                    let m = [0.5*(c[0][0]+c[0][1]),0.5*(c[1][0]+c[1][1])];
                    let (_,su,sv) = self.s.d1(m);
                    let n = cad_kernel::util::space::norm;
                    let reach = 0.5*(1.5*n(su)*(c[0][1]-c[0][0])+1.5*n(sv)*(c[1][1]-c[1][0]));
                    Some(Bound::lipschitz(self.least(m)?.f,1.,reach))
                }
            }
            let field = G {s:&f.surface,ring:&ring,back:poses[1]};
            let clock = std::time::Instant::now();
            let traced = trace(&field,&Question {rect:[[lo[0],hi[0]],[lo[1],hi[1]]],edges:&[],seeds:&[],tol,feature:tube}).unwrap();
            println!("{} pieces in {:?}; search {:?}",traced.pieces.len(),clock.elapsed(),traced.search);
            for (i,p) in traced.pieces.iter().enumerate() {
                let len: f64 = p.uv.windows(2).map(|w| cad_kernel::util::space::distance(f.surface.point(w[0]),f.surface.point(w[1]))).sum();
                let k = p.uv.len().div_ceil(80).max(1);
                let read = |e: usize,x: [f64;3]| match cover(&ring,&path,roll,x,roll[e],0.,tol) {
                    Ok(Reading::Near) => 'N',Ok(Reading::Far(c)) => if c.depth < -tol { 'F' } else if c.depth < tol { 'o' } else { '.' },Ok(Reading::Alone) => 'a',Err(_) => '!' };
                let line = |e: usize| -> String { p.uv.iter().step_by(k).map(|&u| read(e,f.surface.point(u))).collect() };
                println!("piece {i}: {} points, {len:.4} long, ends {:?}, from {:?} to {:?}",p.uv.len(),p.ends,f.surface.point(p.uv[0]),f.surface.point(p.uv[p.uv.len()-1]));
                println!("  start {}\n  end   {}",line(0),line(1));
            }
            return
        }
        "torus" => {
            // the tool at the start (`0`) or end (`1`) pose at its torus's angles `(φ, ψ)`: the place
            // and its reading as a point of that pose
            let end: usize = args[2].parse().unwrap();
            let (phi,psi): (f64,f64) = (args[3].parse().unwrap(),args[4].parse().unwrap());
            let pose = path.pose(roll[end]).unwrap();
            let r = 1.+tube*psi.cos();
            let x = pose.point([3.+r*phi.cos(),r*phi.sin(),2.+tube*psi.sin()]);
            println!("{x:?}: {:?}",cover(&ring,&path,roll,x,roll[end],0.,tol));
            return
        }
        "birth" => {
            // a fold pair's birth over `[t0, t1]`: the least `s''` over the angles `[θ0, θ1]` on the
            // label at the window's middle and the place `σ`, read at `n` angles and settled by
            // golden sections about the least, halved in the roll to where it passes nought
            let n: Vec<f64> = args[2..8].iter().map(|v| v.parse().unwrap()).collect();
            let (th0,th1,t0,t1,sg,k) = (n[0],n[1],n[2],n[3],n[4],n[5] as usize);
            let l = label_at(&ring,&path,0.5*(th0+th1),0.5*(t0+t1),sg).unwrap();
            let low = |t: f64| -> (f64,f64) {
                let s = |th: f64| sheet(&ring,&path,roll,th,t,l).map_or(f64::INFINITY,|s| s.1);
                let j = (0..=k).min_by(|&i,&j| s(th0+(th1-th0)*i as f64/k as f64).total_cmp(&s(th0+(th1-th0)*j as f64/k as f64))).unwrap();
                let (mut lo,mut hi) = (th0+(th1-th0)*(j.max(1)-1) as f64/k as f64,th0+(th1-th0)*(j+1).min(k) as f64/k as f64);
                let g = 0.5*(5f64.sqrt()-1.);
                for _ in 0..80 { let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo)); if s(x1) < s(x2) { hi = x2 } else { lo = x1 } }
                (0.5*(lo+hi),s(0.5*(lo+hi)))
            };
            for j in 0..=10 { let t = t0+(t1-t0)*j as f64/10.; let (th,v) = low(t); println!("t {t:.8}: least s'' {v:+.3e} at θ {th:.7}") }
            let (mut lo,mut hi) = (t0,t1);
            if (low(lo).1 < 0.) != (low(hi).1 < 0.) {
                for _ in 0..60 { let m = 0.5*(lo+hi); if (low(m).1 < 0.) == (low(lo).1 < 0.) { lo = m } else { hi = m } }
                println!("least s'' passes nought at t {:.9}, θ {:.7}",0.5*(lo+hi),low(0.5*(lo+hi)).0);
            }
            return
        }
        "find" => {
            // the sheet's (θ, t, label) nearest `x` over t ∈ [t0, t1]: a grid, then a finer one about it
            let x: Vec<f64> = args[2].split(',').map(|v| v.parse().unwrap()).collect();
            let x = [x[0],x[1],x[2]];
            let (mut t0,mut t1): (f64,f64) = (args[3].parse().unwrap(),args[4].parse().unwrap());
            let nf = ring.elements.len() as f64;
            let (mut th0,mut th1) = (0.,std::f64::consts::TAU);
            let mut best;
            for _ in 0..4 {
                let rows = cad_kernel::util::par::indices(201,|j| {
                    let t = t0+(t1-t0)*j as f64/200.;
                    let mut b = (f64::INFINITY,0.,0.,None);
                    for i in 0..=400 { let th = th0+(th1-th0)*i as f64/400.;
                        for k in 0..8 { if let Some(l) = label_at(&ring,&path,th,t,nf*k as f64/8.) { if let Some(s) = sheet(&ring,&path,roll,th,t,l) {
                            let d = cad_kernel::util::space::distance(s.0,x); if d < b.0 { b = (d,th,t,Some(l)) } } } } }
                    b
                });
                best = rows.into_iter().min_by(|a,b| a.0.total_cmp(&b.0)).unwrap();
                let (dth,dt) = ((th1-th0)/40.,(t1-t0)/20.);
                (th0,th1,t0,t1) = (best.1-dth,best.1+dth,best.2-dt,best.2+dt);
                println!("{best:?}");
            }
            return
        }
        "near" => {
            // each sheet point (θ, t) of the list (on the label at σ), its distance in space from the
            // contact at the roll `tc` over angles [θ0, θ1], and where along it
            let n: Vec<f64> = args[2..6].iter().map(|v| v.parse().unwrap()).collect();
            let (tc,th0,th1,sg) = (n[0],n[1],n[2],n[3]);
            for pair in args[6..].iter() {
                let q: Vec<f64> = pair.split(',').map(|v| v.parse().unwrap()).collect();
                let l = label_at(&ring,&path,q[0],q[1],sg).unwrap();
                let x = sheet(&ring,&path,roll,q[0],q[1],l).unwrap().0;
                let lc = label_at(&ring,&path,0.5*(th0+th1),tc,sg).unwrap();
                let mut best = (f64::INFINITY,0.);
                for i in 0..=200_000 { let a = th0+(th1-th0)*i as f64/200_000.;
                    if let Some(s) = sheet(&ring,&path,roll,a,tc,lc) { let d = cad_kernel::util::space::distance(s.0,x); if d < best.0 { best = (d,a) } } }
                println!("θ {:.7} t {:.7}: {:.3e} from the contact at θ {:.7}",q[0],q[1],best.0,best.1);
            }
            return
        }
        "seams" | "seams-through" => {
            // the cover's boundary over the box [θ0, θ1] × [t0, t1] of the sheet on the label at σ,
            // seeded at (θs, ts), its feature in space: each piece and its partner, each partner point's
            // own cover read back (a double curve's: covered at its partner's instant)
            let n: Vec<f64> = args[2..10].iter().map(|v| v.parse().unwrap()).collect();
            let (th0,th1,t0,t1,ths,ts,sg,feature) = (n[0],n[1],n[2],n[3],n[4],n[5],n[6],n[7]);
            let l = label_at(&ring,&path,ths,ts,sg).unwrap();
            let clock = std::time::Instant::now();
            let x = sheet(&ring,&path,roll,ths,ts,l).unwrap().0;
            for _ in 0..100 { let _ = cover(&ring,&path,roll,x,ts,0.,tol); }
            println!("a cover in {:?}",clock.elapsed()/100);
            let clock = std::time::Instant::now();
            let s = if what == "seams" { cad_kernel::swept::depth::seams(&ring,&path,roll,l,[[th0,th1],[t0,t1]],&[[ths,ts]],tol,feature) }
                else { cad_kernel::swept::depth::seams_through(&ring,&path,roll,l,[[th0,th1],[t0,t1]],&[[ths,ts]],tol,feature) };
            println!("in {:?}",clock.elapsed());
            let s = s.unwrap();
            if s.len() >= 2 {
                let on = |q: (f64,f64,f64)| sheet(&ring,&path,roll,q.0,q.1,l).unwrap().0;
                for &q in &s[1].sheet {
                    let x = on(q);
                    let d = s[0].sheet.windows(2).flat_map(|w| (0..=64).map(move |k| { let f = k as f64/64.; (w[0].0+(w[1].0-w[0].0)*f,w[0].1+(w[1].1-w[0].1)*f,0.) }))
                        .map(|y| cad_kernel::util::space::distance(x,on(y))).fold(f64::INFINITY,f64::min);
                    println!("  {q:?}: {d:.2e} from the first piece; far depth {:?}",far_depth(&ring,&path,roll,q.0,q.1,l,tol).map(|c| c.depth));
                }
            }
            for p in s {
                println!("piece of {} from {:?} to {:?}, ends {:?}",p.sheet.len(),p.sheet[0],p.sheet[p.sheet.len()-1],p.ends); for q in &p.sheet { println!("    {q:?} far {:?}",far_depth(&ring,&path,roll,q.0,q.1,l,tol).map(|c| (c.depth,c.at))) }
                for r in &p.partner {
                    let mut worst = 0f64;
                    for (k,&(th,t,sg)) in r.iter().enumerate() {
                        let lp = label_at(&ring,&path,th,t,sg).unwrap();
                        if let Some(c) = far_depth(&ring,&path,roll,th,t,lp,tol) { worst = worst.max((c.at-p.sheet[k.min(p.sheet.len()-1)].1).abs().min(1.)) }
                    }
                    println!("  partner of {} from {:?} to {:?}; its cover's instant off the sheet's by at most {worst:.2e}",r.len(),r[0],r[r.len()-1]);
                }
            }
            return
        }
        "chord" => {
            // `Covered` read along the chord from (θa, ta) to (θb, tb) on the label at σ: the least and
            // the branch of the cover at the first point continued, at nine points
            use cad_kernel::zero_set::Field;
            let n: Vec<f64> = args[2..7].iter().map(|v| v.parse().unwrap()).collect();
            let l = label_at(&ring,&path,n[0],n[1],n[4]).unwrap();
            let field = cad_kernel::swept::depth::Covered {ring:&ring,path:&path,roll,label:l,within:tol};
            let from = field.least([n[0],n[1]]).unwrap().at;
            for k in 0..=8 {
                let f = k as f64/8.;
                let uv = [n[0]+(n[2]-n[0])*f,n[1]+(n[3]-n[1])*f];
                let (a,b) = (field.least(uv).map(|v| (v.f,v.at.at)),field.along(uv,&from).map(|v| (v.f,v.at.at)));
                println!("{f:.3}: least {a:?}, along {b:?}");
                if k == 4 {
                    let g = field.along(uv,&from).unwrap().grad;
                    let hh = 1e-6;
                    let fd = [(field.along([uv[0]+hh,uv[1]],&from).unwrap().f-field.along([uv[0]-hh,uv[1]],&from).unwrap().f)/(2.*hh),
                        (field.along([uv[0],uv[1]+hh],&from).unwrap().f-field.along([uv[0],uv[1]-hh],&from).unwrap().f)/(2.*hh)];
                    let (_,su,sv) = field.surface(uv);
                    println!("   gradient {g:?}, by differences {fd:?}, rates {su:?} {sv:?}");
                }
            }
            return
        }
        "birth-rows" => {
            // the label whose sheet at (θ, t) passes nearest `x`, and along the roll over [t0, t1]
            // (n steps) the least `s''` over the angles [θ0, θ1] on it; the shape's cuts
            let n: Vec<f64> = args[2..9].iter().map(|v| v.parse().unwrap()).collect();
            let x: Vec<f64> = args[9].split(',').map(|v| v.parse().unwrap()).collect();
            let (th,t,th0,th1,t0,t1,k) = (n[0],n[1],n[2],n[3],n[4],n[5],n[6] as usize);
            let nf = ring.elements.len() as f64;
            let l = (0..64).filter_map(|i| label_at(&ring,&path,th,t,nf*i as f64/64.)).filter_map(|l| sheet(&ring,&path,roll,th,t,l).map(|s| (l,cad_kernel::util::space::distance(s.0,[x[0],x[1],x[2]]))))
                .min_by(|a,b| a.1.total_cmp(&b.1)).unwrap();
            println!("label {:?} at {:.3e}",l.0,l.1);
            let (scale,cuts) = cad_kernel::swept::rings::changes(&ring,&path,roll).unwrap();
            println!("scale {scale:.4e}, cuts {cuts:?}");
            for j in 0..=k {
                let t = t0+(t1-t0)*j as f64/k as f64;
                let (mut best,mut at) = (f64::INFINITY,th0);
                for i in 0..=4000 { let a = th0+(th1-th0)*i as f64/4000.; if let Some(s) = sheet(&ring,&path,roll,a,t,l.0) { if s.1 < best { best = s.1; at = a } } }
                println!("t {t:.6}: least s'' {best:+.4e} at θ {at:.6}");
            }
            return
        }
        "pieces" => {
            // each curve's pieces at the roll `t`: their angles, labels and sides
            let t: f64 = args[2].parse().unwrap();
            for (ci,c) in cad_kernel::swept::rings::pieces_at(&ring,&path,[t,roll[1]],t).unwrap().iter().enumerate() {
                for p in c { println!("curve {ci}: [{:.7}, {:.7}] {:?} {:?}",p.0,p.1,p.2,p.3) }
            }
            return
        }
        "cover" => {
            // a point `x` of a sheet at its own roll `t` read by `cover` standing off it by `off`,
            // and the depth along the roll about `t` (`… cover x,y,z t off [half-width] [n]`)
            let x: Vec<f64> = args[2].split(',').map(|v| v.parse().unwrap()).collect();
            let x = [x[0],x[1],x[2]];
            let (t,off): (f64,f64) = (args[3].parse().unwrap(),args[4].parse().unwrap());
            println!("{:?}",cover(&ring,&path,roll,x,t,off,tol));
            if let Some(w) = args.get(5).map(|v| v.parse::<f64>().unwrap()) {
                let n: usize = args.get(6).map_or(40,|v| v.parse().unwrap());
                let ts: Vec<f64> = (0..=n).map(|k| t-w+2.*w*k as f64/n as f64).collect();
                for (s,d) in ts.iter().zip(cad_kernel::swept::depth::depths(&ring,&path,x,&ts)) { println!("{s:.8} {d:+.4e}") }
            }
            return
        }
        "at" => {
            // the sheet's point at (θ, t, σ) and its reading
            let n: Vec<f64> = args[2..5].iter().map(|v| v.parse().unwrap()).collect();
            let l = label_at(&ring,&path,n[0],n[1],n[2]).unwrap();
            let (x,bend) = sheet(&ring,&path,roll,n[0],n[1],l).unwrap();
            println!("{x:?} s'' {bend:+.3e} {l:?}: {:?}",cover(&ring,&path,roll,x,n[1],0.,tol));
            return
        }
        "map" => {
            // the far depth over a window of the sheet, on the label at its middle's place
            let n: Vec<f64> = args[2..7].iter().map(|v| v.parse().unwrap()).collect();
            let sigma: f64 = args.get(7).map_or(0.,|v| v.parse().unwrap());
            let (th0,th1,t0,t1,k) = (n[0],n[1],n[2],n[3],n[4] as usize);
            let l = label_at(&ring,&path,0.5*(th0+th1),0.5*(t0+t1),sigma).unwrap();
            println!("label {l:?}");
            for i in 0..=k {
                let th = th0+(th1-th0)*i as f64/k as f64;
                let row: Vec<String> = (0..=k).map(|j| {
                    let t = t0+(t1-t0)*j as f64/k as f64;
                    let side = sheet(&ring,&path,roll,th,t,l).map_or(f64::NAN,|s| s.1);
                    if side < 0. { return "    ~    ".into() }
                    match far_depth(&ring,&path,roll,th,t,l,tol) { Some(c) => format!("{:+.1e}",c.depth),None => "    .    ".into() }
                }).collect();
                println!("{th:.5} {}",row.join(" "));
            }
            return
        }
        _ => {}
    }
    let clock = std::time::Instant::now();
    if what == "fit" {
        let RingFitted {ring,fits,curves,..} = ring_fitted(&tool,&path,roll,tol).unwrap();
        println!("{} stretches {:?}, in {:?}",fits.len(),fits.iter().map(|f| f.roll).collect::<Vec<_>>(),clock.elapsed());
        for c in &curves {
            let at = c.iter().position(|p| p.0.is_nan()).unwrap_or(c.len());
            println!("a curve of {at} points from {:?} to {:?}, swallowtail points {:?}",c[0],c[at-1],&c[at+1..]);
            // (`fit t0 t1`: its points whose roll is within the window, and how each bends)
            if let (Some(t0),Some(t1)) = (args.get(2).and_then(|v| v.parse::<f64>().ok()),args.get(3).and_then(|v| v.parse::<f64>().ok())) {
                for (k,&(th,t,sg)) in c[..at].iter().enumerate().filter(|p| p.1.1 >= t0 && p.1.1 <= t1) {
                    let s = label_at(&ring,&path,th,t,sg).and_then(|l| sheet(&ring,&path,roll,th,t,l));
                    println!("  {k}: θ {th:.7} t {t:.7} σ {sg:.5}: {:?}",s.map(|(x,b)| (x,b)));
                }
            }
        }
        return
    }
    let RingFitted {ring,fits,curves,cut,..} = ring_fitted(&tool,&path,roll,tol).unwrap();
    println!("{} stretches, {} curves, cut {cut}, in {:?}",fits.len(),curves.len(),clock.elapsed());
    for (ci,c) in curves.iter().enumerate() {
        let at = c.iter().position(|p| p.0.is_nan()).unwrap_or(c.len());
        let apexes = &c[at+1..];
        println!("curve {ci}: {} preimage points, swallowtail points {apexes:?}",at);
        let mut worst = 0f64;
        for (k,&(th,t,sg)) in c[..at].iter().enumerate() {
            let Some(l) = label_at(&ring,&path,th,t,sg) else { println!("  {k}: no label"); continue };
            let Some((x,bend)) = sheet(&ring,&path,roll,th,t,l) else { println!("  {k}: no point"); continue };
            let r = cover(&ring,&path,roll,x,t,0.,tol).unwrap();
            let near_apex = apexes.iter().map(|a| ((a.0-th).powi(2)+(a.1-t).powi(2)).sqrt()).fold(f64::INFINITY,f64::min);
            match r {
                Reading::Far(cv) => {
                    worst = worst.max(cv.depth.abs());
                    if what == "all" || k < 4 || k+4 > at || near_apex < 0.02 || cv.depth.abs() > tol {
                        println!("  {k}: θ {th:.7} t {t:.7} σ {sg:.4} s'' {bend:+.2e}; far depth {:+.3e} at {:.7} piece {} (from the apex {near_apex:.2e})",cv.depth,cv.at,cv.piece);
                    }
                }
                other => println!("  {k}: θ {th:.7} t {t:.7} σ {sg:.4} s'' {bend:+.2e}; {other:?} (from the apex {near_apex:.2e})"),
            }
        }
        println!("  greatest |far depth| on the curve {worst:.3e} (tolerance {tol:.2e})");
    }
}
