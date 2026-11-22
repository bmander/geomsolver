//! **brep-bench's bull nose carried along a looping spline, built as the bench builds it** (#880
//! T1f): the block cut by the mill carried without turning along the cubic through
//! `swept::closed::cut_from`; its volume, faces and time printed.
//!
//! `cargo run --release -p cad-kernel --example bull_nose`
#[path = "support/bench.rs"]
mod bench;
use bench::bull_nose::*;

fn main() {
    let (stock,tool,path,roll,bar,tol) = case(CONTROL);
    eprintln!("tool faces {:?}, tol {tol:e}",tool.faces.iter().map(|f| f.surface.kind()).collect::<Vec<_>>());
    let clock = std::time::Instant::now();
    match cad_kernel::swept::closed::cut_from(&stock,&tool,&path,roll,bar,tol) {
        Ok(b) => {
            let built = clock.elapsed();
            let kinds: Vec<&str> = b.faces.iter().map(|f| f.surface.kind()).collect();
            println!("built in {built:?}: volume {:.9}, check {:?}, faces {kinds:?}",
                cad_kernel::props::volume(&b),b.check(tol).err());
        }
        Err(e) => println!("refused in {:?}: {e:?}",clock.elapsed()),
    }
}
