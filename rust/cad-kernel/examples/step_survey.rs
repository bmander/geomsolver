//! Read each STEP file named on the command line and say what the reader made of it: a line a
//! file, tab separated — the file, `read` or `refused`, and the solids' counts and measures or
//! the refusal.
//!
//!     step_survey [--reference TABLE] [--faces] [--oracle CORPUS] [--past-sound] [--moved D] FILE…
//!
//! `--reference TABLE` (the STEP corpus's `reference.tsv`, rust/gcs-core/tests/step_corpus/) puts
//! beside each file its row's reference volume and area, where it comes from, the reading's
//! relative errors and the bar they are held to (`ok` within it, `OFF` past it); a file is found
//! in the table by the row whose path ends its own. `--faces` lists each solid's faces after its
//! line, a line each — index, the ADVANCED_FACE it was read from (`#0` for one the reader laid),
//! surface kind, `reversed` or `forward`, area — and, where the file lies under the corpus
//! (`--oracle CORPUS`, else `$STEP_CORPUS`) and the oracle has measured its faces
//! (scripts/step-corpus/faces.py), beside each: `Σ max(tol, u)·length` of its uses, `u` the
//! file's stated uncertainty (`slack`, mm²), OCCT's
//! area and brep-bench's measurer's, the reading's difference from them relative to its area,
//! the bar `step_corpus::face_bar` holds it to, and `ok`, `OFF` or `disputed` (the readers
//! disagree past the bar; `ok` within it of either). A face's line says why where a reference is
//! missing (`-`), and the file's last line counts the faces past their bars and gives each
//! reader's faces' sum (the gate holds no face to a reader whose sum is not the file's area).
//! `--past-sound` reads a file `step_read::sound` refuses once more without it
//! (`topology::solid`, its solids unplaced) and lists that reading's faces under a `past sound`
//! line: what the reader's own faces measure, whether or not the boundary was sound.
//!
//! Each file read also says what it determines of its totals, as the corpus gate holds them
//! (`step_corpus::determined`): `S`, the sum over its faces of `Σ max(tol, u)·length`, its reach
//! `R` from the point its volume's flux is taken about, and the relative bars `S/A` and `R·S/(3V)`.
//! `--moved D` reads each solid's volume again moved by `D` mm along each axis
//! (`props::volume` on `Brep::moved`): a boundary whose edges stand off their faces does not
//! close, and its volume moves with it by `|D|·|∮ n dA|/3`, which the slack bounds by `|D|·S/3`.
//! Where a third reader measured the faces (`<path>.quad.faces.tsv`, scripts/step-corpus/
//! teapot.py), `--faces` gives its area too, and a face is held to it as the gate holds it.
use cad_kernel::props::{area,volume};
use cad_kernel::step_read::{context,read,topology,Why};
use cad_kernel::geom::Rigid;
use cad_kernel::topo::{Brep,EdgeCurve};
use cad_kernel::util::json;
use std::collections::BTreeMap;

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

/// The bar `step_corpus::face_bar` holds a face's area to (mm²): `TOL × slack + REL × area +
/// ABS` (the constants are the gate's, rust/gcs-core/tests/step_corpus.rs says why).
fn face_bar(slack: f64,area: f64) -> f64 { 1.*slack+1e-6*area.abs()+1e-9 }

/// How far `b`'s vertices and edges reach from the point `props::volume` takes its flux about
/// (as `step_corpus::reach`: the origin, or the vertices' box's centre more than ten diagonals
/// away).
fn reach(b: &Brep) -> f64 {
    let (mut lo,mut hi) = ([f64::INFINITY; 3],[f64::NEG_INFINITY; 3]);
    for v in &b.vertices { for k in 0..3 { lo[k] = lo[k].min(v.p[k]); hi[k] = hi[k].max(v.p[k]) } }
    if b.vertices.is_empty() { return 0. }
    let c: [f64; 3] = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
    let size = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
    let o = if (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt() > 10.*size { c } else { [0.; 3] };
    let far = |p: [f64; 3]| ((p[0]-o[0]).powi(2)+(p[1]-o[1]).powi(2)+(p[2]-o[2]).powi(2)).sqrt();
    let on_edges = b.edges.iter().filter(|e| !matches!(e.curve,EdgeCurve::Degenerate))
        .flat_map(|e| (0..=8).map(move |k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/8.,&b.vertices)));
    b.vertices.iter().map(|v| far(v.p)).chain(on_edges.map(far)).fold(0.,f64::max)
}

/// An edge's length, by the chord of 32 pieces of its parameter (a degenerate edge's 0).
fn length(b: &Brep,e: usize) -> f64 {
    let edge = &b.edges[e];
    if matches!(edge.curve,EdgeCurve::Degenerate) { return 0. }
    let at = |k: usize| edge.point(edge.t[0]+(edge.t[1]-edge.t[0])*k as f64/32.,&b.vertices);
    (0..32).map(|k| { let (p,q) = (at(k),at(k+1)); ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt() }).sum()
}

/// A third reader's faces of the corpus file `rel` (`<rel>.quad.faces.tsv`), where there are any.
fn quad(root: &str,rel: &str) -> BTreeMap<u32,f64> {
    let text = std::fs::read_to_string(format!("{root}/oracle/{rel}.quad.faces.tsv")).unwrap_or_default();
    text.lines().skip(2).filter_map(|l| { let (e,a) = l.split_once('\t')?; Some((e.parse().ok()?,a.parse().ok()?)) }).collect()
}

/// The oracle's faces of the corpus file `rel`: OCCT's area by entity (or why there is none), and
/// the measurer's.
fn oracle(root: &str,rel: &str) -> (Result<BTreeMap<u32,f64>,String>,Result<BTreeMap<u32,f64>,String>) {
    let at = |x: &str| format!("{root}/oracle/{rel}{x}");
    let occt = (|| {
        let j = json::parse(&std::fs::read_to_string(at(".json")).map_err(|_| "no OCCT reading".to_string())?)?;
        if j.get("error").is_some() { return Err("OCCT failed".to_string()) }
        let named = j.get("faces_named").map(|n| n.arr().len() == 2 && n.arr()[0].as_i64() == n.arr()[1].as_i64()).unwrap_or(false);
        if !(named || j.get("faces_mapped").map(|m| m.as_bool()).unwrap_or(false)) { return Err("OCCT's faces not named".into()) }
        let mut out = BTreeMap::new();
        for line in std::fs::read_to_string(at(".faces.tsv")).map_err(|_| "no OCCT face table".to_string())?.lines().skip(1) {
            let f: Vec<&str> = line.split('\t').collect();
            let e: u32 = f.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
            *out.entry(e).or_insert(0.) += f.get(5).and_then(|s| s.parse::<f64>().ok()).unwrap_or(f64::NAN);
        }
        Ok(out)
    })();
    let mass = (|| {
        let text = std::fs::read_to_string(at(".mass.faces.tsv")).map_err(|_| "no measurer face table".to_string())?;
        let mut out = BTreeMap::new();
        for line in text.lines().skip(2) {
            let (e,a) = line.split_once('\t').unwrap_or((line,""));
            let e: u32 = e.parse().unwrap_or(0);
            if e == 0 { return Err(format!("the measurer: {a}")) }
            if let Ok(a) = a.parse::<f64>() { *out.entry(e).or_insert(0.) += a }
        }
        // its scale, as step_corpus::measurer_scale reads it: 1 or 25.4² where the median ratio
        // of the faces both readers measured is so to a part in a hundred (an inch file's areas are square
        // inches to the measurer), else 1 for a file in millimetres alone
        let scale = match &occt {
            Ok(o) if out.iter().any(|(e,m)| *m > 0. && o.get(e).map_or(false,|x| *x > 0.)) => {
                let mut r: Vec<f64> = out.iter().filter_map(|(e,m)| o.get(e).filter(|x| **x > 0. && *m > 0.).map(|x| m/x)).collect();
                r.sort_by(f64::total_cmp);
                let r = r[r.len()/2];
                [1.,25.4*25.4].into_iter().find(|k| (k*r-1.).abs() < 1e-2)
                    .ok_or_else(|| format!("the measurer reads the file at {r:.3e} of OCCT's scale"))?
            }
            _ => match json::parse(&std::fs::read_to_string(at(".json")).unwrap_or_default()).ok()
                .and_then(|j| Some(j.get("units")?.as_str() == "mm")) {
                Some(true) => 1.,
                _ => return Err("the measurer's scale unknown".to_string()),
            },
        };
        Ok(out.into_iter().map(|(e,a)| (e,scale*a)).collect())
    })();
    (occt,mass)
}

fn main() {
    let mut reference = None;
    let mut faces = false;
    let mut past_sound = false;
    let mut moved: Option<f64> = None;
    let mut corpus = std::env::var("STEP_CORPUS").ok();
    let mut files = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--reference" => reference = Some(table(&args.next().expect("--reference names a table"))),
            "--faces" => faces = true,
            "--past-sound" => past_sound = true,
            "--moved" => moved = Some(args.next().and_then(|d| d.parse().ok()).expect("--moved takes a length (mm)")),
            "--oracle" => corpus = Some(args.next().expect("--oracle names the corpus")),
            _ => files.push(a),
        }
    }
    for path in files {
        let row = reference.as_ref().and_then(|t| t.iter().filter(|(p,_)| path == *p || path.ends_with(&format!("/{p}")))
            .max_by_key(|(p,_)| p.len()).map(|(_,r)| r));
        let clock = std::time::Instant::now();
        // the oracle's faces, where the file lies under the corpus
        let refs = corpus.as_ref().filter(|_| faces).and_then(|root| {
            let (full,root) = (std::fs::canonicalize(&path).ok()?,std::fs::canonicalize(root).ok()?);
            let rel = full.strip_prefix(&root).ok()?.to_str()?.to_string();
            let (o,m) = oracle(root.to_str()?,&rel);
            Some((o,m,quad(root.to_str()?,&rel)))
        });
        let mut off_faces = 0;
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
        let got = match read(&text) {
            Err(r) if past_sound && r.why == Why::Unsound => {
                println!("{path}\trefused\t{:?}\t{r}\t{:.2?}{}",r.why,clock.elapsed(),beside(None));
                let again = context::parse(&text).and_then(|file| context::solids(&file)?.into_iter()
                    .map(|f| topology::solid(&file,f)).collect::<Result<Vec<_>,_>>());
                if again.is_ok() { println!("{path}\tpast sound") }
                again
            }
            got => got,
        };
        match got {
            Ok(solids) => {
                // the file's measures are its solids' together, as the reference's are
                let v: f64 = solids.iter().map(|g| volume(&g.brep)).sum();
                let a: f64 = solids.iter().map(|g| g.brep.faces.iter().map(|f| area(&g.brep,f)).sum::<f64>()).sum();
                // what the file determines of its totals (step_corpus::determined)
                let slack = |stated: bool| -> f64 { solids.iter().map(|g| {
                    let b = &g.brep;
                    let u = if stated { g.context.uncertainty.unwrap_or(0.) } else { 0. };
                    let lengths: Vec<f64> = (0..b.edges.len()).map(|e| length(b,e)).collect();
                    b.faces.iter().flat_map(|f| f.loops.iter().flatten()).map(|c| b.edges[c.edge as usize].tol.max(u)*lengths[c.edge as usize])
                        .sum::<f64>()
                }).sum() };
                let (s,sm) = (slack(true),slack(false));
                let r = solids.iter().map(|g| reach(&g.brep)).fold(0.,f64::max);
                println!("{path}\tdetermines\tS {s:.3e} mm²\tR {r:.4e} mm\tarea to {:.1e}\tvolume to {:.1e}\tmeasured tolerances alone: S {sm:.3e}, area to {:.1e}, volume to {:.1e}",
                    s/a.abs(),r*s/(3.*v.abs()),sm/a.abs(),r*sm/(3.*v.abs()));
                if let Some(d) = moved {
                    for k in 0..3 {
                        let t: [f64; 3] = std::array::from_fn(|j| if j == k { d } else { 0. });
                        let w: f64 = solids.iter().map(|g| volume(&g.brep.moved(&Rigid {r:Rigid::identity().r,t}))).sum();
                        println!("{path}\tmoved {d} mm along {}\tvolume {w:.12e}\tchanged {:.2e} (relative)\tthe slack bounds |D|·S/(3V) = {:.2e}, the measured tolerances' {:.2e}",
                            ["x","y","z"][k],(w-v).abs()/v.abs(),d.abs()*s/(3.*v.abs()),d.abs()*sm/(3.*v.abs()));
                    }
                }
                for (i,g) in solids.iter().enumerate() {
                    let b = &g.brep;
                    let ga: f64 = b.faces.iter().map(|f| area(b,f)).sum();
                    let last = i+1 == solids.len();
                    println!("{path}\tread\t#{}\tfaces {}\tedges {}\tvertices {}\tvolume {:.9e}\tarea {:.9e}\t{:.2?}{}",g.entity,b.faces.len(),
                        b.edges.len(),b.vertices.len(),volume(b),ga,clock.elapsed(),if last { beside(Some((v,a))) } else { String::new() });
                    if faces {
                        let lengths: Vec<f64> = (0..b.edges.len()).map(|e| length(b,e)).collect();
                        for (k,f) in b.faces.iter().enumerate() {
                            let e = g.faces.get(k).copied().unwrap_or(0);
                            let a = area(b,f);
                            let u = g.context.uncertainty.unwrap_or(0.);
                            let slack: f64 = f.loops.iter().flatten().map(|c| b.edges[c.edge as usize].tol.max(u)*lengths[c.edge as usize]).sum();
                            let mut line = format!("  {k}\t#{e}\t{}\t{}\t{a:.12e}",f.surface.kind(),if f.reversed { "reversed" } else { "forward" });
                            if let Some((occt,mass,quad)) = &refs {
                                let bar = face_bar(slack,a);
                                let o = occt.as_ref().ok().and_then(|m| m.get(&e).copied());
                                let m = mass.as_ref().ok().and_then(|m| m.get(&e).copied());
                                let q = quad.get(&e).copied();
                                let d = |r: Option<f64>| r.map_or("-".to_string(),|r| format!("{:+.1e}",(a-r)/a.abs().max(1e-300)));
                                let off = |r: f64| (a-r).abs() <= bar;
                                let verdict = match (q,o,m) {
                                    // the quadrature checks the readers: one past its bar of it is set aside
                                    (Some(q),o,m) => if off(q) && [o,m].into_iter().flatten().filter(|r| (r-q).abs() <= bar).all(off) { "ok" }
                                        else { off_faces += 1; "OFF" },
                                    (None,None,None) => "-",
                                    (None,Some(r),None) | (None,None,Some(r)) => if off(r) { "ok" } else { off_faces += 1; "OFF" },
                                    (None,Some(x),Some(y)) if (x-y).abs() <= bar => if off(x) && off(y) { "ok" } else { off_faces += 1; "OFF" },
                                    (None,Some(x),Some(y)) => if off(x) || off(y) { "disputed" } else { off_faces += 1; "OFF" },
                                };
                                line += &format!("\tslack {slack:.1e}\tOCCT {}\t{}\tmeasurer {}\t{}{}\tbar {bar:.1e}\t{verdict}",
                                    o.map_or("-".into(),|x| format!("{x:.9e}")),d(o),m.map_or("-".into(),|x| format!("{x:.9e}")),d(m),
                                    q.map_or(String::new(),|x| format!("\tquadrature {x:.12e}\t{}",d(Some(x)))));
                            }
                            println!("{line}");
                        }
                    }
                }
            },
            Err(r) => println!("{path}\trefused\t{:?}\t{r}\t{:.2?}{}",r.why,clock.elapsed(),beside(None)),
        }
        if let Some((occt,mass,_)) = &refs {
            // what each reader's faces sum to: one whose sum is not the file's area has misread a
            // face somewhere, and the gate does not hold faces to it (step_corpus::Refs::of)
            let sum = |m: &BTreeMap<u32,f64>| format!("{:.9e} over {} faces",m.values().sum::<f64>(),m.len());
            println!("{path}\tfaces past their bars: {off_faces}\tOCCT: {}\tmeasurer: {}",
                occt.as_ref().map_or_else(|e| e.clone(),sum),mass.as_ref().map_or_else(|e| e.clone(),sum));
        }
    }
}
