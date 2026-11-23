//! **brep-bench's bull nose carried along a looping spline, built as the bench builds it** (#880
//! T1f): the block cut by the mill carried without turning along the cubic through
//! `swept::closed::cut_from`; its volume, faces and time printed.
//!
//! `cargo run --release -p cad-kernel --example bull_nose`
#[path = "support/bench.rs"]
mod bench;
use bench::bull_nose::*;

fn main() {
    // (`bull_nose ulp|<d> <pole> <axis>`: that pole moved by an ulp or by `d`)
    let mut control = CONTROL;
    if let Some(m) = std::env::args().nth(1) {
        let pole: usize = std::env::args().nth(2).map_or(1,|s| s.parse().unwrap());
        let axis: usize = std::env::args().nth(3).map_or(0,|s| s.parse().unwrap());
        let x = control[pole][axis];
        control[pole][axis] = if m == "ulp" { f64::from_bits(x.to_bits()+1) } else { x+m.parse::<f64>().unwrap() };
        eprintln!("pole {pole} moved to {:?}",control[pole]);
    }
    let (stock,tool,path,roll,bar,tol) = case(control);
    eprintln!("tool faces {:?}, tol {tol:e}",tool.faces.iter().map(|f| f.surface.kind()).collect::<Vec<_>>());
    let clock = std::time::Instant::now();
    match cad_kernel::swept::closed::cut_from(&stock,&tool,&path,roll,bar,tol) {
        Ok(b) => {
            let built = clock.elapsed();
            let kinds: Vec<&str> = b.faces.iter().map(|f| f.surface.kind()).collect();
            println!("built in {built:?}: volume {:.9}, check {:?}, faces {kinds:?}",
                cad_kernel::props::volume(&b),b.check(tol).err());
            if std::env::var_os("RUNNER").is_some() {
                let c = std::time::Instant::now();
                let a: f64 = b.faces.iter().map(|f| cad_kernel::props::area(&b,f)).sum();
                let ta = c.elapsed();
                let c = std::time::Instant::now();
                let v = cad_kernel::props::volume(&b);
                let tv = c.elapsed();
                let c = std::time::Instant::now();
                let ok = b.check(tol).is_ok();
                let tc = c.elapsed();
                let c = std::time::Instant::now();
                let m = cad_kernel::mesh::mesh(&b,1e-3*b.size(),0.1).map(|m| m.tris.len());
                let tm = c.elapsed();
                let c = std::time::Instant::now();
                let st = cad_kernel::step::write(&b,"bull-nose",tol,cad_kernel::step::Fit::Exact).map(|s| s.len());
                let ts = c.elapsed();
                println!("build {built:?}, area {a:.6} {ta:?}, volume {v:.9} {tv:?}, check {ok} {tc:?}, mesh {m:?} {tm:?}, step {st:?} {ts:?}");
            }
            if std::env::var_os("AREAS").is_some() {
                for (fi,f) in b.faces.iter().enumerate() {
                    let c = std::time::Instant::now();
                    let a = cad_kernel::props::area(&b,f);
                    let net = match &f.surface { cad_kernel::geom::Surface::BSpline(_,n) => format!("net {}x{}",n.poles.len(),n.poles[0].len()),_ => String::new() };
                    let spans: Vec<String> = f.loops.iter().flatten().map(|c| { let e = &b.edges[c.edge as usize];
                        let k = match &e.curve { cad_kernel::topo::EdgeCurve::Curve(cad_kernel::geom::Curve::BSpline(s)) => s.poles.len(),_ => 0 };
                        let p = match &c.pcurve { cad_kernel::topo::Pcurve::Curve(c) => match &**c { cad_kernel::geom::Curve::BSpline(s) => format!("/{}",s.poles.len()),_ => "/c".into() },cad_kernel::topo::Pcurve::Inverse {..} => "/inv".into(),_ => "/l".into() };
                        format!("{k}{p}") }).collect();
                    println!("face {fi} {} {net}: area {a:.4} in {:?}; edges {spans:?}",f.surface.kind(),c.elapsed());
                }
            }
            if let Ok(fi) = std::env::var("FACE") {
                let fi: usize = fi.parse().unwrap();
                let f = &b.faces[fi];
                println!("face {fi}: {:?} reversed {}",f.surface,f.reversed);
                for l in &f.loops { for c in l {
                    let e = &b.edges[c.edge as usize];
                    let uv = |t: f64| c.pcurve.at(t,e,&f.surface,&b.vertices);
                    println!("  edge {} {} rev {}: uv {:?} .. {:?} mid {:?}; {:?}..{:?}",c.edge,e.curve.kind(),c.reversed,uv(e.t[0]),uv(e.t[1]),uv(0.5*(e.t[0]+e.t[1])),e.point(e.t[0],&b.vertices),e.point(e.t[1],&b.vertices));
                } }
            }
            if std::env::var_os("MESH").is_some() {
                let c = std::time::Instant::now();
                let sag: f64 = std::env::var("MESH").unwrap().parse().unwrap(); let m = cad_kernel::mesh::mesh(&b,sag,0.1).unwrap();
                let v: f64 = m.tris.iter().map(|t| { let [a,b,c] = t.map(|i| m.pts[i as usize]);
                    (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6. }).sum();
                eprintln!("mesh {} triangles, volume {v:.9} in {:?}",m.tris.len(),c.elapsed());
                let c = std::time::Instant::now();
                let a: f64 = b.faces.iter().map(|f| cad_kernel::props::area(&b,f)).sum();
                eprintln!("area {a:.6} in {:?}",c.elapsed());
            }
        }
        Err(e) => println!("refused in {:?}: {e:?}",clock.elapsed()),
    }
}
