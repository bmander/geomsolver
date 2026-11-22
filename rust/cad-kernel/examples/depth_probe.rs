//! **The depth-based trim measured on brep-bench's tumbling tori** (#942): each double curve the
//! ring road traces read under `swept::depth::cover` — its points' far depth, the instant that covers
//! them against the partner's — and the far cover read about each swallowtail point.
//!
//! `cargo run --release -p cad-kernel --example depth_probe -- full-turn|half-turn|thin [curves|apex|map θ0 θ1 t0 t1 n|point x,y,z]`
use cad_kernel::swept::depth::{Reading,cover,far_depth,label_at,sheet};
use cad_kernel::trajectory::{Screw,Trajectory};

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
        let rows = cad_kernel::util::par::indices(n+1,|i| {
            let v = hi[1]-(hi[1]-lo[1])*i as f64/n as f64;
            let line: String = (0..=n).map(|j| {
                let u = lo[0]+(hi[0]-lo[0])*j as f64/n as f64;
                let x = f.surface.point([u,v]);
                match cover(&ring,&t.path,t.roll,x,v,t.tol,t.tol) {
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
        "tail" => {
            // each double curve's preimage points: at its angle, the stretch of the roll from it to
            // the fold (`s''` nought) on the side `s''` falls, its far depth's least there and the
            // stretch's length in space
            let (ring,_,_,curves,_) = cad_kernel::swept::closed::ring_fitted(&tool,&path,roll,tol).unwrap();
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
            // point nearest `x`, each place's reading as a point of that pose
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
                        Ok(Reading::Far(cv)) => { least = least.min(cv.depth); if cv.depth < -tol { 'F' } else { '.' } }
                        Ok(Reading::Alone) => 'a',
                        Err(_) => '!',
                    }
                }).collect();
                format!("ψ {ps:+.5} {line} least {least:+.2e}")
            });
            for r in rows { println!("{r}") }
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
    let (ring,_,fits,curves,cut) = cad_kernel::swept::closed::ring_fitted(&tool,&path,roll,tol).unwrap();
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
