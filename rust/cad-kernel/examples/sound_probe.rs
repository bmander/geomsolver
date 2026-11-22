//! A face of a STEP file's solid laid out for reading by eye: its surface, and each use of each
//! of its loops — the edge, its curve, its pcurve's kind, its tolerance and its points in the
//! face's parameters and in space.
//!
//!     sound_probe FILE FACE [SAMPLES]
use cad_kernel::step_read::{context,topology};
use cad_kernel::topo::Pcurve;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let text = std::fs::read_to_string(&args[0]).unwrap();
    let fi: usize = args[1].parse().unwrap();
    let n: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(8);
    let file = context::parse(&text).unwrap();
    for found in context::solids(&file).unwrap() {
        let g = topology::solid(&file,found).unwrap();
        let b = &g.brep;
        let f = &b.faces[fi];
        println!("face {fi} #{} {} reversed {} periods {:?}",g.faces[fi],f.surface.kind(),f.reversed,f.surface.periods());
        if let cad_kernel::geom::Surface::BSpline(_,s) = &f.surface {
            println!("  net {}x{} degrees {} {} u [{},{}] v [{},{}]",s.poles.len(),s.poles[0].len(),s.du,s.dv,
                s.uknots[0],s.uknots[s.uknots.len()-1],s.vknots[0],s.vknots[s.vknots.len()-1]);
        }
        {
            // the face alone held sound: what its loops say (a face alone bounds nothing besides)
            let alone = cad_kernel::topo::Brep {vertices:b.vertices.clone(),edges:b.edges.clone(),faces:vec![f.clone()]};
            println!("  alone: {:?}",cad_kernel::step_read::sound::sound(&alone,&g.context).err().map(|u| u.msg));
        }
        for (li,l) in f.loops.iter().enumerate() {
            println!(" loop {li}");
            for c in l {
                let e = &b.edges[c.edge as usize];
                let kind = match &c.pcurve { Pcurve::Line {..} => "line",Pcurve::Inverse {..} => "inverse",Pcurve::Curve(_) => "curve" };
                let uses = b.faces.iter().flat_map(|f| f.loops.iter().flatten()).filter(|d| d.edge == c.edge).count();
                println!("  edge {} #{} {} v {:?} t {:?} tol {:.3e} reversed {} pcurve {kind} uses {uses} vtol {:.3e} {:.3e}",c.edge,g.edges[c.edge as usize],
                    e.curve.kind(),e.v,e.t,e.tol,c.reversed,b.vertices[e.v[0] as usize].tol,b.vertices[e.v[1] as usize].tol);
                for k in 0..=n {
                    let t = e.t[0]+(e.t[1]-e.t[0])*k as f64/n as f64;
                    let uv = c.pcurve.at(t,e,&f.surface,&b.vertices);
                    let p = e.point(t,&b.vertices);
                    let q = f.surface.point(uv);
                    let (_,su,sv) = f.surface.d1(uv);
                    let _ = (su,sv);
                    if k == 0 { println!("    speeds {:.4e} {:.4e}",cad_kernel::util::space::norm(su),cad_kernel::util::space::norm(sv)); }
                    println!("    t {t:.6} uv [{:.6}, {:.6}] p [{:.4}, {:.4}, {:.4}] off {:.2e}",uv[0],uv[1],p[0],p[1],p[2],cad_kernel::util::space::distance(p,q));
                }
            }
        }
    }
}
