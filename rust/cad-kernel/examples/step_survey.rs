//! Read each STEP file named on the command line and say what the reader made of it: a line a
//! file, tab separated — the file, `read` or `refused`, and the solids' counts and measures or
//! the refusal.
//!
//!     step_survey [--reference TABLE] [--faces] FILE…
//!
//! `--reference TABLE` (the STEP corpus's `reference.tsv`, rust/gcs-core/tests/step_corpus/) puts
//! beside each file its row's reference volume and area, where it comes from, the reading's
//! relative errors and the bar they are held to (`ok` within it, `OFF` past it); a file is found
//! in the table by the row whose path ends its own. `--faces` lists each solid's faces after its
//! line, a line each — index, the ADVANCED_FACE it was read from (`#0` for one the reader laid),
//! surface kind, `reversed` or `forward`, area — for comparing face by face with the oracle's
//! (`scripts/step-corpus/oracle.py --faces`, which names the same entities).
use cad_kernel::props::{area,volume};
use cad_kernel::step_read::read;

/// A row of the reference table: volume, area, where they come from, and the bars.
struct Reference { volume: f64,area: f64,from: String,bar_volume: f64,bar_area: f64 }

fn table(path: &str) -> Vec<(String,Reference)> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let head: Vec<&str> = lines.next().unwrap_or("").split('\t').collect();
    let col = |n: &str| head.iter().position(|h| *h == n).unwrap_or_else(|| panic!("{path}: no column {n}"));
    let (p,v,a,from,bv,ba) = (col("path"),col("ref_volume"),col("ref_area"),col("ref_from"),col("bar_volume"),col("bar_area"));
    lines.filter(|l| !l.is_empty()).map(|l| {
        let f: Vec<&str> = l.split('\t').collect();
        let x = |i: usize| f[i].parse::<f64>().unwrap_or(f64::NAN);
        (f[p].to_string(),Reference {volume:x(v),area:x(a),from:f[from].to_string(),bar_volume:x(bv),bar_area:x(ba)})
    }).collect()
}

fn main() {
    let mut reference = None;
    let mut faces = false;
    let mut files = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--reference" => reference = Some(table(&args.next().expect("--reference names a table"))),
            "--faces" => faces = true,
            _ => files.push(a),
        }
    }
    for path in files {
        let row = reference.as_ref().and_then(|t| t.iter().filter(|(p,_)| path == *p || path.ends_with(&format!("/{p}")))
            .max_by_key(|(p,_)| p.len()).map(|(_,r)| r));
        let clock = std::time::Instant::now();
        let text = match std::fs::read_to_string(&path) { Ok(t) => t,Err(e) => { println!("{path}\tunreadable\t{e}"); continue } };
        let beside = |v: Option<(f64,f64)>| match (row,v) {
            (None,_) => String::new(),
            (Some(r),None) => format!("\treference volume {:.9e}\tarea {:.9e}\t({})",r.volume,r.area,r.from),
            (Some(r),Some((v,a))) => {
                let (dv,da) = ((v-r.volume).abs()/r.volume.abs(),(a-r.area).abs()/r.area.abs());
                format!("\treference volume {:.9e}\tarea {:.9e}\t({})\tdV {dv:.1e}\tdA {da:.1e}\tbars {:.0e} {:.0e}\t{}",r.volume,r.area,
                    r.from,r.bar_volume,r.bar_area,if dv <= r.bar_volume && da <= r.bar_area { "ok" } else { "OFF" })
            }
        };
        match read(&text) {
            Ok(solids) => {
                // the file's measures are its solids' together, as the reference's are
                let v: f64 = solids.iter().map(|g| volume(&g.brep)).sum();
                let a: f64 = solids.iter().map(|g| g.brep.faces.iter().map(|f| area(&g.brep,f)).sum::<f64>()).sum();
                for (i,g) in solids.iter().enumerate() {
                    let b = &g.brep;
                    let ga: f64 = b.faces.iter().map(|f| area(b,f)).sum();
                    let last = i+1 == solids.len();
                    println!("{path}\tread\t#{}\tfaces {}\tedges {}\tvertices {}\tvolume {:.9e}\tarea {:.9e}\t{:.2?}{}",g.entity,b.faces.len(),
                        b.edges.len(),b.vertices.len(),volume(b),ga,clock.elapsed(),if last { beside(Some((v,a))) } else { String::new() });
                    if faces {
                        for (k,f) in b.faces.iter().enumerate() {
                            println!("  {k}\t#{}\t{}\t{}\t{:.12e}",g.faces.get(k).copied().unwrap_or(0),f.surface.kind(),
                                if f.reversed { "reversed" } else { "forward" },area(b,f));
                        }
                    }
                }
            },
            Err(r) => println!("{path}\trefused\t{:?}\t{r}\t{:.2?}{}",r.why,clock.elapsed(),beside(None)),
        }
    }
}
