//! **The ring road's profile on brep-bench's tumbling tori** (#942 F4): `full-turn`, `half-turn` or `thin`, built
//! closed through `swept::closed::ring` with `SOLVENT_BREP_TIME` set, each step's time and what it
//! read printed (the instants read, their time on the cores, the roots read alone). With
//! `SOLVENT_SWEEP_DUMP` set as well, a trim refused is written there (`sweep_probe` reads it).
//!
//! `cargo run --release -p cad-kernel --example ring_profile -- full-turn|half-turn|thin [roll end, degrees]`
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
    let (tube,(tool,path),half) = match args.first().map(String::as_str) {
        Some("full-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),180.),
        Some("half-turn") => (0.16,tumbling([3.,0.,2.],2.,0.16,0.375,30.,20.,0.7071,1.),90.),
        Some("thin") => (0.015,tumbling([3.,0.,2.],2.,0.015,0.25,45.,0.,0.5f64.sqrt(),1.),75.),
        _ => { eprintln!("ring_profile full-turn|half-turn|thin [roll end, degrees]"); std::process::exit(2) }
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
