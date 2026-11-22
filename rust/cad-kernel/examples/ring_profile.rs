//! **The ring road's profile on brep-bench's tumbling tori** (#942 F4): `full-turn` or `thin`, built
//! closed through `swept::closed::ring` with `SOLVENT_BREP_TIME` set, each step's time and what it
//! read printed (the instants read, their time on the cores, the roots read alone). With
//! `SOLVENT_SWEEP_DUMP` set as well, a trim refused is written there (`sweep_probe` reads it).
//!
//! `spline` is brep-bench's ring carried along a spline on the ring road (`closed::ring_cut_from`,
//! `trajectory::Along`, the bench's sibling ring cases' bar and tolerance); `bench-spline` the same
//! case exactly as the bench's runner builds it (`closed::cut_from`, its own frame).
//!
//! `cargo run --release -p cad-kernel --example ring_profile -- full-turn|thin [roll end, degrees] | spline | bench-spline`
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
    let frame = Trajectory::Twist(Screw {origin:[0.,height*cl,height*sl],axis:[ca,sa*cl,sa*sl],ratio:observer,advance:0.});
    (tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)})
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("spline") { return along_a_spline() }
    if args.first().map(String::as_str) == Some("bench-spline") { return as_the_bench_runs_it() }
    let (tube,(tool,path),half) = match args.first().map(String::as_str) {
        Some("full-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),180.),
        Some("thin") => (0.015,tumbling([3.,0.,2.],2.,0.015,0.25,45.,0.,0.5f64.sqrt(),1.),75.),
        _ => { eprintln!("ring_profile full-turn|thin [roll end, degrees] | spline | bench-spline"); std::process::exit(2) }
    };
    let end: f64 = args.get(1).map_or(half,|e| e.parse().expect("degrees"));
    let roll = [-half.to_radians(),end.to_radians()];
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    let built = cad_kernel::swept::closed::ring(&tool,&path,roll,tube/250.,tube/50000.);
    match built {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(&b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
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
fn as_the_bench_runs_it() {
    use cad_kernel::nurbs::BSpline;
    let stock = cad_kernel::build::block([-6.,-6.,-8.],[36.,26.,0.]).unwrap();
    let ring = cad_kernel::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(Runner(curve)));
    let tol = cad_kernel::geom::tolerance(stock.size().max(ring.size()));
    if std::env::var_os("SOLVENT_BREP_TIME").is_none() { std::env::set_var("SOLVENT_BREP_TIME","1") }
    let clock = std::time::Instant::now();
    eprintln!("tolerance {tol:e}");
    match cad_kernel::swept::closed::cut_from(&stock,&ring,&path,[0.,1.],0.005,tol) {
        Ok(b) => println!("built: {} faces, volume {}, in {:?}",b.faces.len(),cad_kernel::props::volume(&b),clock.elapsed()),
        Err(e) => println!("refused: {e:?}, in {:?}",clock.elapsed()),
    }
}
