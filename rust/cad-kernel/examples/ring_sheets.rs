//! **What the ring along a spline enters, read for its open edges** (#942, the depth trim): each
//! candidate sheet's faces and each edge only one of its faces uses below the block's top.
//!
//! `cargo run --release -p cad-kernel --example ring_sheets`
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
    let tool = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = cad_kernel::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],
        vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = cad_kernel::trajectory::Trajectory::General(std::sync::Arc::new(Runner(curve)));
    let tol = std::env::args().nth(1).map_or(5.3e-8,|s| s.parse().unwrap());
    let e = cad_kernel::swept::closed::ring_entered_by(&tool,&path,[0.,1.],Some(0.005),tol).unwrap();
    for (i,s) in e.sheets.iter().enumerate() {
        println!("sheet {i}: {} faces",s.faces.len());
        for (ei,u) in s.uses().iter().enumerate() {
            if u.len() != 1 { continue }
            let ed = &s.edges[ei];
            let [p,q] = ed.t.map(|t| ed.point(t,&s.vertices));
            let m = ed.point(0.5*(ed.t[0]+ed.t[1]),&s.vertices);
            if m[2] > -1e-6 && p[2] > -1e-6 && q[2] > -1e-6 { continue }
            if m[2] > 2.6 { continue }
            println!("  face {} edge {ei} ({}): {:.4?} .. {:.4?} .. {:.4?}",u[0].0,ed.curve.kind(),p,m,q);
        }
    }
    println!("{} given",e.given.len());
}
