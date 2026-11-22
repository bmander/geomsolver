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
