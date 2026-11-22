//! **Questions put to a refused trim, apart from the build** (#942 F5): a directory a refused build
//! wrote under `SOLVENT_SWEEP_DUMP` (`swept::dump::dump_trim`), read back and asked
//!
//! - `replay`: the trim run again, its open edges narrated (`SOLVENT_TRIM_DEBUG`);
//! - `boundary x,y,z dx,dy,dz [reach] [steps]`: where the sweep's boundary crosses the line through
//!   the point along the direction, and the candidates' faces it lies on;
//! - `through x,y,z [within]`: the candidates' faces and edges, and the given meetings, near the point.
//!
//! `cargo run --release -p cad-kernel --example sweep_probe -- <dir> <question> …`: built once, each
//! question a run against the files, no rebuild between them.
use cad_kernel::swept::dump::load_trim;

fn v3(s: &str) -> [f64;3] {
    let x: Vec<f64> = s.split(',').map(|v| v.trim().parse().expect("a number")).collect();
    assert!(x.len() == 3,"three numbers, by commas: {s}");
    [x[0],x[1],x[2]]
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let usage = "sweep_probe <dir> replay | boundary x,y,z dx,dy,dz [reach] [steps] | through x,y,z [within]";
    let (Some(dir),Some(question)) = (args.first(),args.get(1)) else { eprintln!("{usage}"); std::process::exit(2) };
    let t = load_trim(std::path::Path::new(dir)).unwrap_or_else(|e| { eprintln!("{e}"); std::process::exit(1) });
    println!("{} candidates, {} given meetings, roll {:?}, tolerance {:e}",t.candidates.len(),t.given.len(),t.roll,t.tol);
    match question.as_str() {
        "replay" => {
            if std::env::var_os("SOLVENT_TRIM_DEBUG").is_none() { eprintln!("(set SOLVENT_TRIM_DEBUG=1 to narrate its open edges)") }
            match t.replay() {
                Ok((b,report)) => println!("built: {} faces, volume {}; {report:?}",b.faces.len(),cad_kernel::props::volume(&b)),
                Err(why) => println!("refused: {why:?}"),
            }
        }
        "boundary" => {
            let (at,dir) = (v3(args.get(2).map_or("",String::as_str)),v3(args.get(3).map_or("",String::as_str)));
            let reach = args.get(4).map_or(0.01,|r| r.parse().expect("a reach"));
            let steps = args.get(5).map_or(400,|r| r.parse().expect("a count"));
            for c in t.boundary_along(at,dir,reach,steps).unwrap_or_else(|e| { eprintln!("{e}"); std::process::exit(1) }) {
                println!("boundary at {:?}: {:?} to {:?}",c.at,c.from,c.to);
                for n in c.on.iter().take(8) { println!("  candidate {} face {} ({} {}) off {:e}",n.candidate,n.face,n.kind,n.name,n.off) }
            }
        }
        "through" => {
            let p = v3(args.get(2).map_or("",String::as_str));
            let within = args.get(3).map_or(1e-6,|r| r.parse().expect("a distance"));
            let near = t.through(p,within);
            for n in &near.faces { println!("face: candidate {} face {} ({} {}) off {:e}",n.candidate,n.face,n.kind,n.name,n.off) }
            for (c,e,kind,d) in &near.edges { println!("edge: candidate {c} edge {e} ({kind}) {d:e} away") }
            for (g,[a,b],kind,d) in &near.given { println!("given {g} ({a} × {b}): a {kind} {d:e} away") }
        }
        "faces" => {
            // each candidate's check, and the loops of its faces with an edge within `within` of `p`:
            // each use's edge, its ends in space and in the face's parameters
            let p = v3(args.get(2).map_or("",String::as_str));
            let within = args.get(3).map_or(1e-3,|r| r.parse().expect("a distance"));
            for (k,b) in t.candidates.iter().enumerate() {
                println!("candidate {k}: {} faces, check {:?}",b.faces.len(),b.check(t.tol));
                for (i,f) in b.faces.iter().enumerate() {
                    let near = f.loops.iter().flatten().any(|c| { let e = &b.edges[c.edge as usize];
                        matches!(&e.curve,cad_kernel::topo::EdgeCurve::Curve(cv) if (0..=64).any(|j| cad_kernel::util::space::distance(cv.point(e.t[0]+(e.t[1]-e.t[0])*j as f64/64.),p) <= within)) });
                    if !near { continue }
                    println!("  face {i} ({}):",f.surface.kind());
                    for (li,l) in f.loops.iter().enumerate() { for c in l {
                        let e = &b.edges[c.edge as usize];
                        let ends = [c.pcurve.at(e.t[0],e,&f.surface,&b.vertices),c.pcurve.at(e.t[1],e,&f.surface,&b.vertices)];
                        let (a,z) = (b.vertices[e.v[0] as usize].p,b.vertices[e.v[1] as usize].p);
                        println!("    loop {li} edge {} reversed {} over {:?}: {a:.6?} to {z:.6?}, in the face {ends:.6?}",c.edge,c.reversed,e.t);
                    } }
                }
            }
        }
        _ => { eprintln!("{usage}"); std::process::exit(2) }
    }
}
