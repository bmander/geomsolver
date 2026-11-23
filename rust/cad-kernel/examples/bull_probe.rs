//! A probe of the bull nose's case (#880 T1f): the regions the mill's sections sweep, height by
//! height, and the groove's volume read from their areas.
#[path = "support/bench.rs"]
mod bench;
use bench::bull_nose::*;
use cad_kernel::swept::region::SweptRegion;

fn area(path: &cad_kernel::trajectory::Trajectory,roll: [f64;2],r: f64) -> f64 {
    let disc = cad_kernel::build::prism(&cad_kernel::build::disc([0.,0.,-1.],[0.,0.,1.],r),0.,1.).unwrap();
    let standing = cad_kernel::swept::subtraction::Prismatic::along(&disc,[0.,0.,1.],1.).unwrap();
    let setting = standing.setting(-0.5,true,path,roll).unwrap();
    let reg = SweptRegion::read(setting,[[-40.,-40.],[60.,60.]],false).unwrap();
    (0..reg.loops.len()).map(|l| {
        let q = reg.polyline(l,std::env::args().nth(2).map_or(4096,|s| s.parse().unwrap()));
        (0..q.len()).map(|i| { let (a,b) = (q[i],q[(i+1)%q.len()]); 0.5*(a[0]*b[1]-a[1]*b[0]) }).sum::<f64>()
    }).sum()
}

fn main() {
    let (_,_,mut path,roll,bar,tol) = case(CONTROL);
    if std::env::var_os("RUNNER").is_some() {
        path = bench::runner();
        let tool = cad_kernel::build::prism(&cad_kernel::build::disc([0.,0.,-1.],[0.,0.,1.],2.),0.,1.).unwrap();
        println!("{:?}",cad_kernel::swept::planar_solid::swept(&tool,&path,roll,bar,tol).map(|b| b.map(|b| cad_kernel::props::volume(&b))));
        return
    }
    let n: usize = std::env::args().nth(1).map_or(64,|s| s.parse().unwrap());
    // Gauss–Legendre, 4 points a panel, over the torus's angle
    let g = [(-0.8611363115940526,0.3478548451374538),(-0.3399810435848563,0.6521451548625461),
        (0.3399810435848563,0.6521451548625461),(0.8611363115940526,0.3478548451374538)];
    let h = std::f64::consts::FRAC_PI_2/n as f64;
    let mut nose = 0.;
    for k in 0..n {
        for &(x,w) in &g {
            let th = h*(k as f64+0.5+0.5*x);
            nose += 0.5*h*w*area(&path,roll,1.5+0.5*th.cos())*0.5*th.cos();
        }
    }
    let wall = area(&path,roll,2.);
    println!("panels {n}: wall {wall:.10} nose {nose:.10} groove {:.10} volume {:.10}",wall+nose,11200.-wall-nose);
}
