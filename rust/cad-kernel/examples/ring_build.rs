//! **brep-bench's ring carried along a spline, built as the bench builds it** (#942, the depth
//! trim): the block cut by the torus carried by the runner's frame through `swept::closed::cut_from`,
//! a bar of 0.005 and the tolerance of its size; its volume, faces and time printed.
//!
//! `cargo run --release -p cad-kernel --example ring_build`
#[path = "support/bench.rs"]
mod bench;

fn main() {
    let block = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,0.]).unwrap();
    let tool = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = bench::cubic();
    // (the runner's own frame, or with `ALONG` set the kernel's frame along the curve, which
    // encloses its motion over a stretch)
    let path = if std::env::var_os("ALONG").is_some() {
        cad_kernel::trajectory::Trajectory::General(std::sync::Arc::new(cad_kernel::trajectory::Along::new(curve).unwrap()))
    } else { cad_kernel::trajectory::Trajectory::General(std::sync::Arc::new(bench::Runner(curve))) };
    let size = block.size().max(tool.size());
    let tol = std::env::args().nth(1).map_or(cad_kernel::geom::tolerance(size),|s| s.parse().unwrap());
    let clock = std::time::Instant::now();
    match cad_kernel::swept::closed::cut_from(&block,&tool,&path,[0.,1.],0.005,tol) {
        Ok(b) => {
            let built = clock.elapsed();
            let kinds: Vec<&str> = b.faces.iter().map(|f| f.surface.kind()).collect();
            if std::env::var_os("FACES").is_some() {
                let uses = b.uses();
                for (fi,f) in b.faces.iter().enumerate().filter(|(_,f)| f.surface.kind() == "torus") {
                    println!("face {fi} ({}): area {:.6}",f.name,cad_kernel::props::area(&b,f));
                    for co in f.loops.iter().flatten() {
                        let e = &b.edges[co.edge as usize];
                        let [p,q] = e.t.map(|t| e.point(t,&b.vertices));
                        let other: Vec<String> = uses[co.edge as usize].iter().filter(|u| u.0 != fi).map(|u| format!("{} {}",u.0,b.faces[u.0].surface.kind())).collect();
                        println!("  edge {} ({}): {:.4?} .. {:.4?}, beside {other:?}",co.edge,e.curve.kind(),p,q);
                    }
                }
            }
            let c = std::time::Instant::now();
            let v = cad_kernel::props::volume(&b);
            eprintln!("volume {v} in {:?}",c.elapsed());
            let c = std::time::Instant::now();
            let a: f64 = b.faces.iter().map(|f| cad_kernel::props::area(&b,f)).sum();
            eprintln!("area {a} in {:?}",c.elapsed());
            for f in &b.faces {
                let c = std::time::Instant::now();
                let x = cad_kernel::props::area(&b,f);
                if c.elapsed().as_millis() > 50 { eprintln!("  {} area {x} in {:?}",f.surface.kind(),c.elapsed()) }
            }
            let c = std::time::Instant::now();
            let ok = b.check(tol).is_ok();
            eprintln!("check {ok} in {:?}",c.elapsed());
            let c = std::time::Instant::now();
            let st = cad_kernel::step::write(&b,"ring",tol,cad_kernel::step::Fit::Exact).map(|s| s.len());
            eprintln!("step {st:?} in {:?}",c.elapsed());
            println!("built in {built:?}: volume {:.6}, check {:?}, faces {kinds:?}",cad_kernel::props::volume(&b),b.check(tol).err());
        }
        Err(e) => println!("refused in {:?}: {e:?}",clock.elapsed()),
    }
}
