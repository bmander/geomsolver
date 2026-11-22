//! A probe of the upright road on the bull nose's case (#880 T1f): its closed solid, built alone.
#[path = "support/bench.rs"]
mod bench;
use bench::bull_nose::*;

fn main() {
    let (_,tool,path,roll,bar,tol) = case(CONTROL);
    let clock = std::time::Instant::now();
    match cad_kernel::swept::upright::swept(&tool,&path,roll,bar,tol) {
        Ok(Some(b)) => println!("built in {:?}: {} faces, volume {:.10}",clock.elapsed(),b.faces.len(),cad_kernel::props::volume(&b)),
        other => println!("{:?} in {:?}",other.map(|_| ()),clock.elapsed()),
    }
}
