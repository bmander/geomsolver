//! **brep-bench's ring carried along a spline, built as the bench builds it** (#942, the depth
//! trim): the block cut by the torus carried by the runner's frame through `swept::closed::cut_from`,
//! a bar of 0.005 and the tolerance of its size; its volume, faces and time printed.
//!
//! `cargo run --release -p cad-kernel --example ring_build`
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

fn main() {
    let block = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,0.]).unwrap();
    let tool = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = cad_kernel::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],
        vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    // (the runner's own frame, or with `ALONG` set the kernel's frame along the curve, which
    // encloses its motion over a stretch)
    let path = if std::env::var_os("ALONG").is_some() {
        cad_kernel::trajectory::Trajectory::General(std::sync::Arc::new(cad_kernel::trajectory::Along::new(curve).unwrap()))
    } else { cad_kernel::trajectory::Trajectory::General(std::sync::Arc::new(Runner(curve))) };
    let size = block.size().max(tool.size());
    let tol = std::env::args().nth(1).map_or(cad_kernel::geom::tolerance(size),|s| s.parse().unwrap());
    let clock = std::time::Instant::now();
    match cad_kernel::swept::closed::cut_from(&block,&tool,&path,[0.,1.],0.005,tol) {
        Ok(b) => {
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
            println!("built in {:?}: volume {:.6}, check {:?}, faces {kinds:?}",clock.elapsed(),cad_kernel::props::volume(&b),b.check(tol).err());
        }
        Err(e) => println!("refused in {:?}: {e:?}",clock.elapsed()),
    }
}
