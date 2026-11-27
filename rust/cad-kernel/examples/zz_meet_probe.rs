use cad_kernel::geom::{Curve,Frame,Surface,V};
use cad_kernel::meeting::meet;
use cad_kernel::util::space::{distance};
use std::f64::consts::TAU;
fn main(){
  let mut rng=cad_kernel::util::rng::Rng::new(736);
  for j in 1..7 { for _ in 0..3 {
    let mut frame = || Frame::new([rng.uniform(-0.5,0.5),rng.uniform(-0.5,0.5),rng.uniform(-0.5,0.5)],
                [rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(0.2,1.)],[rng.uniform(-1.,1.),1.,rng.uniform(-1.,1.)]);
    let (fa,fb)=(frame(),frame());
    if j!=4 { continue }
    let a=Surface::Cylinder(fa,1.); let da=[[0.,TAU],[-2.,2.]];
    let b=Surface::Torus(fb,1.4,0.5); let db=[[0.,TAU],[0.,TAU]];
    let m=meet(&a,da,&b,db,1e-7).unwrap();
    let p:V=[-0.25872868105248603, 0.9334419856325503, 1.1396446609783701];
    println!("branches {}", m.branches.len());
    for br in &m.branches { let c=&br.curve; let t=c.inverse(p); let q=c.point(t);
      if let Curve::Traced(tr)=c { let poly=tr.pts.windows(2).map(|w| cad_kernel::util::space::segment_distance(p,w[0],w[1])).fold(f64::INFINITY,f64::min);
        let n=tr.pts.len()-1; let mut best=(f64::INFINITY,0.); for k in 0..=n*200 { let s=k as f64/200.; let d=distance(c.point(s),p); if d<best.0 {best=(d,s)} }
        println!(" traced n={} closed={} inverse t={t} off {:e}; polyline off {poly:e}; dense best {:e} at {}",n,tr.closed,distance(q,p),best.0,best.1);
        if poly<1e-3 { for k in 0..=12 { let t=281.+k as f64*0.25; let (x,d)=c.d1(t); let h=1e-6; let fd=cad_kernel::util::space::scale(cad_kernel::util::space::sub(c.point(t+h),c.point(t-h)),0.5/h);
           println!("   t {t}: speed {:.6e} fd {:.6e} diff {:.3e} chord {:.4e} sin {:.3e}",cad_kernel::util::space::norm(d),cad_kernel::util::space::norm(fd),cad_kernel::util::space::distance(d,fd),distance(tr.pts[(t as usize).min(n-1)],tr.pts[(t as usize+1).min(n)]),
             cad_kernel::util::space::norm(cad_kernel::util::space::cross(cad_kernel::util::space::unit(a.gradient(x)),cad_kernel::util::space::unit(b.gradient(x))))); } }
      } else { println!(" {} off {:e}",c.kind(),distance(q,p)); } }
  } }
}
