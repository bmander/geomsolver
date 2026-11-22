//! Read each STEP file named on the command line without the soundness check, measure what was
//! read, then hold it sound apart: a line a file, tab separated — the file, the read's time, what
//! `sound` says and its time, and (with `--reference TABLE`, the corpus's `reference.tsv`) the
//! reading's relative errors against its row and whether they are within its bars. So a refusal of
//! `sound` is seen beside whether the solid it refuses was read right.
//!
//!     sound_survey [--reference TABLE] [--no-measure | --measure-refused] [--faces] FILE…
//!
//! `--faces` lists each solid's faces after its line as `step_survey --faces` does — index, the
//! ADVANCED_FACE it was read from, surface kind, sense, area — to join with the oracle's.
use cad_kernel::props::{area,volume};
use cad_kernel::step_read::{context,sound,topology};

struct Reference { volume: f64,area: f64,bar_volume: f64,bar_area: f64 }

fn table(path: &str) -> Vec<(String,Reference)> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let head: Vec<&str> = lines.next().unwrap_or("").split('\t').collect();
    let col = |n: &str| head.iter().position(|h| *h == n).unwrap_or_else(|| panic!("{path}: no column {n}"));
    let (p,v,a,bv,ba) = (col("path"),col("ref_volume"),col("ref_area"),col("bar_volume"),col("bar_area"));
    lines.filter(|l| !l.is_empty()).map(|l| {
        let f: Vec<&str> = l.split('\t').collect();
        let x = |i: usize| f.get(i).and_then(|s| s.parse::<f64>().ok()).unwrap_or(f64::NAN);
        (f[p].to_string(),Reference {volume:x(v),area:x(a),bar_volume:x(bv),bar_area:x(ba)})
    }).collect()
}

fn main() {
    let (mut reference,mut measure,mut faces,mut refused_only,mut files) = (None,true,false,false,Vec::new());
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--reference" => reference = Some(table(&args.next().expect("--reference names a table"))),
            "--no-measure" => measure = false,
            "--measure-refused" => refused_only = true,
            "--faces" => faces = true,
            _ => files.push(a),
        }
    }
    for path in files {
        let row = reference.as_ref().and_then(|t| t.iter().filter(|(p,_)| path == *p || path.ends_with(&format!("/{p}")))
            .max_by_key(|(p,_)| p.len()).map(|(_,r)| r));
        let Ok(text) = std::fs::read_to_string(&path) else { println!("{path}\tunreadable"); continue };
        let clock = std::time::Instant::now();
        let read = context::parse(&text).and_then(|file| {
            let found = context::solids(&file)?;
            found.into_iter().map(|f| topology::solid(&file,f)).collect::<Result<Vec<_>,_>>()
        });
        let took = clock.elapsed();
        let solids = match read { Ok(s) => s,Err(r) => { println!("{path}\tread {took:.2?}\treader refuses\t{r}"); continue } };
        let clock = std::time::Instant::now();
        let said: Vec<String> = solids.iter().filter_map(|g| sound::sound(&g.brep,&g.context).err().map(|u| u.refusal(g).to_string())).collect();
        let judged = clock.elapsed();
        let verdict = if said.is_empty() { "sound".to_string() } else { format!("UNSOUND {}",said.join(" | ")) };
        let measured = if !measure || (refused_only && said.is_empty()) { String::new() } else {
            let v: f64 = solids.iter().map(|g| volume(&g.brep)).sum();
            let a: f64 = solids.iter().map(|g| g.brep.faces.iter().map(|f| area(&g.brep,f)).sum::<f64>()).sum();
            match row {
                Some(r) => {
                    let (dv,da) = ((v-r.volume).abs()/r.volume.abs(),(a-r.area).abs()/r.area.abs());
                    format!("\tdV {dv:.1e}\tdA {da:.1e}\tbars {:.0e} {:.0e}\t{}",r.bar_volume,r.bar_area,
                        if dv <= r.bar_volume && da <= r.bar_area { "agrees" } else { "DISAGREES" })
                }
                None => format!("\tvolume {v:.9e}\tarea {a:.9e}"),
            }
        };
        let n: usize = solids.iter().map(|g| g.brep.faces.len()).sum();
        // the greatest tolerance against the solid's size, what `sound::LOOSE` bounds
        let loose = solids.iter().map(|g| {
            let b = &g.brep;
            let (lo,hi) = b.vertices.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),v| (std::array::from_fn(|k| lo[k].min(v.p[k])),std::array::from_fn(|k| hi[k].max(v.p[k]))));
            let size = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
            b.edges.iter().map(|e| e.tol).chain(b.vertices.iter().map(|v| v.tol)).fold(0.,f64::max)/size
        }).fold(0.,f64::max);
        let measured = format!("{measured}\ttol/size {loose:.1e}");
        println!("{path}\tfaces {n}\tread {took:.2?}\tsound {judged:.2?}{measured}\t{verdict}");
        if faces {
            for g in &solids {
                for (k,f) in g.brep.faces.iter().enumerate() {
                    println!("  {k}\t#{}\t{}\t{}\t{:.12e}",g.faces.get(k).copied().unwrap_or(0),f.surface.kind(),
                        if f.reversed { "reversed" } else { "forward" },area(&g.brep,f));
                }
            }
        }
    }
}
