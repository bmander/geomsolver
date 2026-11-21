//! Read each STEP file named on the command line and say what the reader made of it: a line a
//! file, tab separated — the file, `read` or `refused`, and the solids' counts and measures or
//! the refusal.
use cad_kernel::props::{area,volume};
use cad_kernel::step_read::read;

fn main() {
    for path in std::env::args().skip(1) {
        let clock = std::time::Instant::now();
        let text = match std::fs::read_to_string(&path) { Ok(t) => t,Err(e) => { println!("{path}\tunreadable\t{e}"); continue } };
        match read(&text) {
            Ok(solids) => for g in &solids {
                let b = &g.brep;
                let a: f64 = b.faces.iter().map(|f| area(b,f)).sum();
                println!("{path}\tread\t#{}\tfaces {}\tedges {}\tvertices {}\tvolume {:.9e}\tarea {:.9e}\t{:.2?}",g.entity,b.faces.len(),
                    b.edges.len(),b.vertices.len(),volume(b),a,clock.elapsed());
            },
            Err(r) => println!("{path}\trefused\t{:?}\t{r}\t{:.2?}",r.why,clock.elapsed()),
        }
    }
}
