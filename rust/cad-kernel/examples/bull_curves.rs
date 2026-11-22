//! A probe of the upright road on the bull nose's case (#880 T1f): its closed solid, built alone —
//! along the looping spline without turning, or (`RUNNER` set) turned with the bench runner's frame
//! along its cubic, a motion keeping the plane that turns about its normal.
#[path = "support/bench.rs"]
mod bench;
use bench::bull_nose::*;

fn main() {
    let (_,tool,mut path,roll,bar,tol) = case(CONTROL);
    if std::env::var_os("RUNNER").is_some() { path = bench::runner() }
    let clock = std::time::Instant::now();
    match cad_kernel::swept::upright::swept(&tool,&path,roll,bar,tol) {
        Ok(Some(b)) => println!("built in {:?}: {} faces, volume {:.10}",clock.elapsed(),b.faces.len(),cad_kernel::props::volume(&b)),
        other => println!("{:?} in {:?}",other.map(|_| ()),clock.elapsed()),
    }
}
