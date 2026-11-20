use cad_kernel::trajectory::{Screw,Trajectory};
fn main() {
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5,0.5],axis:[1.,0.,0.],ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let tool = cad_kernel::build::block([2.4,-0.6,1.4],[3.6,0.6,2.6]).unwrap();
    let roll = [-15f64.to_radians(),15f64.to_radians()];
    let tol = cad_kernel::geom::tolerance(tool.size());
    let (candidates,given) = cad_kernel::swept::general::candidates(&tool,&path,roll,tol).unwrap();
    let size = tool.size();
    let trim = cad_kernel::swept::trim::Trim {resolution:size/32.,hair:(4.*tol).max(1e-6*size),reach:None};
    let dir = std::env::temp_dir().join("zz-dump");
    let _ = std::fs::remove_dir_all(&dir);
    cad_kernel::swept::dump::dump_trim(&dir,&tool,&path,roll,&candidates,&given,size,tol,trim,"x").unwrap();
    let t = cad_kernel::swept::dump::load_trim(&dir).unwrap();
    for (k,(a,b)) in candidates.iter().zip(&t.candidates).enumerate() {
        for (i,(fa,fb)) in a.faces.iter().zip(&b.faces).enumerate() {
            if fa.surface != fb.surface { println!("candidate {k} face {i}: surface differs"); 
                if let (cad_kernel::geom::Surface::Family(f1,s1),cad_kernel::geom::Surface::Family(f2,s2)) = (&fa.surface,&fb.surface) {
                    println!("  frame equal {}, run {} stations {} nodes {} closed {}",f1 == f2,s1.run == s2.run,s1.stations == s2.stations,s1.nodes == s2.nodes,s1.closed == s2.closed);
                    if s1.stations != s2.stations { println!("  spans equal {}, at equal {}",s1.stations.spans == s2.stations.spans,s1.stations.at == s2.stations.at) }
                }
                break }
        }
    }
    for (n,(a,b)) in given.iter().zip(&t.given).enumerate() {
        for (c,d) in a.curves.iter().zip(&b.curves) { if c != d { println!("given {n}: a {} curve differs",c.kind()); break } }
        if a.surfaces[0] != b.surfaces[0] || a.surfaces[1] != b.surfaces[1] { println!("given {n}: surfaces differ") }
    }
    println!("checked {} candidates, {} given",candidates.len(),given.len());
}
