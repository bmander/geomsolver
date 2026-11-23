//! EXPERIMENT (#880 T1f): a cuspidal sheet cut by a plane through its cusp, traced (`ssi::trace`);
//! `TOL` sets the tolerance (at 1e-5 the landing slack is over the least step).
use cad_kernel::geom::{Frame,Surface};
use std::time::Instant;
fn main() {
    // a cuspidal sheet (u², v, u³) over u in [-1, 1], v in [0, 1] (its fold row u = 0 inside the
    // net), cut by the plane z = 0.2 (y − 0.5): the meeting y = 0.5 + 5 u³ runs into the cusp
    let f = |u: f64,v: f64| { let s = 2.*u-1.; [s*s,v,s*s*s] };
    let (net,err) = cad_kernel::nurbs::fit_net(&f,3,3,8,4,1e-12).unwrap();
    println!("net {}×{} err {err:e}",net.poles.len(),net.poles[0].len());
    let sheet = Surface::BSpline(Frame::WORLD,std::sync::Arc::new(net.clone()));
    let n: [f64;3] = [0.,-0.2,1.]; let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
    let plane = Surface::Plane(Frame::about([0.,0.5,0.],[n[0]/l,n[1]/l,n[2]/l]));
    let tol: f64 = std::env::var("TOL").ok().map_or(1e-9,|t| t.parse().unwrap());
    let s = 0.3f64;
    let seed = [s*s,0.5+5.*s*s*s,s*s*s];
    let c = Instant::now();
    let r = cad_kernel::ssi::trace(&plane,&sheet,&[seed],[-1.,-1.,-2.],[2.,2.,2.],tol);
    match r {
        Ok(cs) => { for k in &cs { if let cad_kernel::geom::Curve::Traced(t) = k { println!("curve {} points {:?} .. {:?}",t.pts.len(),t.pts[0],t.pts[t.pts.len()-1]) } else { println!("{}",k.kind()) } } }
        Err(e) => println!("refused {e}"),
    }
    println!("in {:?}",c.elapsed());
}
