//! **The ring road's built cases read at the tolerance of their size** (#942): each built as its
//! gate builds it, then every place `Brep::check` would refuse at `geom::tolerance(size)` listed with
//! by how much (`Brep::misfits`) — what the bench's runner reads as validity.
//!
//! `cargo run --release -p cad-kernel --example size_check -- half-turn|full-turn|eighty|across`
use cad_kernel::trajectory::{Screw,Trajectory};

fn tumbling(tube: f64,ratio: f64,lean: f64,heading: f64,height: f64) -> (cad_kernel::topo::Brep,Trajectory) {
    let tool = cad_kernel::build::torus([3.,0.,2.],[0.,0.,1.],1.,tube).unwrap();
    let (sl,cl) = lean.to_radians().sin_cos();
    let (sa,ca) = heading.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,height*cl,height*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,advance:0.});
    (tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)})
}

fn main() {
    let case = std::env::args().nth(1).unwrap_or_default();
    let clock = std::time::Instant::now();
    let built = match case.as_str() {
        "half-turn" => {
            let (tool,path) = tumbling(0.16,0.375,30.,20.,0.7071);
            cad_kernel::swept::closed::ring(&tool,&path,[-90f64.to_radians(),90f64.to_radians()],0.16/250.,0.16/50000.)
        }
        "full-turn" => {
            let (tool,path) = tumbling(0.16,0.375,30.,20.,0.7071);
            cad_kernel::swept::closed::ring(&tool,&path,[-180f64.to_radians(),180f64.to_radians()],0.16/250.,0.16/50000.)
        }
        "eighty" => {
            let tool = cad_kernel::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
            let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
            let frame = Trajectory::Twist(Screw {origin:[0.,0.5,0.5],axis:[1.,0.,0.],ratio:1.,advance:0.});
            let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
            cad_kernel::swept::closed::ring(&tool,&path,[-75f64.to_radians(),75f64.to_radians()],0.002,1e-5)
        }
        "across" => {
            let block = cad_kernel::build::revolve(&polygon(&[[8.,0.,-1.],[12.,0.,-1.],[12.,0.,1.],[8.,0.,1.]]),[0.,0.,0.],[0.,0.,1.],60f64.to_radians()).unwrap();
            let lean = 25f64.to_radians();
            let cutter = cad_kernel::build::torus([5.,0.,0.],[0.,lean.sin(),lean.cos()],2.5,2.).unwrap();
            let path = Trajectory::Twist(Screw {origin:[0.,0.,0.],axis:[0.,0.,1.],ratio:1.,advance:0.});
            cad_kernel::swept::closed::ring_cut_from(&block,&cutter,&path,[-140f64.to_radians(),140f64.to_radians()],0.01,1e-5)
        }
        _ => { eprintln!("size_check half-turn|full-turn|eighty|across"); std::process::exit(2) }
    };
    let b = match built { Ok(b) => b,Err(e) => { println!("refused: {e:?}"); return } };
    let size_tol = cad_kernel::geom::tolerance(b.size());
    println!("built in {:?}: volume {}, {} faces; the size's tolerance {size_tol:e}",clock.elapsed(),cad_kernel::props::volume(&b),b.faces.len());
    let misfits = b.misfits(size_tol);
    println!("{} misfits",misfits.len());
    for (what,d,held) in &misfits { println!("  {d:.3e} (held to {held:.2e}, {:.1}×): {what}",d/held) }
}

fn polygon(points: &[[f64;3]]) -> cad_kernel::build::Profile {
    use cad_kernel::build::{Profile,ProfileEdge};
    let n = points.len();
    let (u,v) = ([points[1][0]-points[0][0],points[1][1]-points[0][1],points[1][2]-points[0][2]],
        [points[2][0]-points[0][0],points[2][1]-points[0][1],points[2][2]-points[0][2]]);
    let c = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
    let l = (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt();
    Profile {origin:points[0],normal:[c[0]/l,c[1]/l,c[2]/l],loops:vec![(0..n).map(|k| ProfileEdge::Line {a:points[k],b:points[(k+1)%n]}).collect()],names:Vec::new()}
}
