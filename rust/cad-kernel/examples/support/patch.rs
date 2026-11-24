//! What the patch examples share (#1013, `patch_probe`, `patch_fuzz`): the block under a part of a
//! sheet, its top face on the whole sheet or on that part.
use cad_kernel::geom::{Surface,V};
use cad_kernel::nurbs::{BSpline,Net};
use cad_kernel::topo::Brep;
use std::sync::Arc;

/// The block on `z = 0` under the part `us × vs` of `patch` (a graph over the floor), its walls
/// upright, sewn: its top face on the whole of `patch`, trimmed to the part (`whole`), or on the
/// part alone (`Net::segment`, the same parameters, so the trims are the same either way).
pub fn under(patch: &Net,us: [f64;2],vs: [f64;2],whole: bool) -> Brep {
    let top = patch.segment(us,vs);
    let floored = |r: &Vec<V>| r.iter().map(|p| [p[0],p[1],0.]).collect();
    let floor = Net {poles:top.poles.iter().map(floored).collect(),..top.clone()};
    // a wall under a rim of the top: degree 1 up from its foot on the floor
    let wall = |rim: BSpline| Net {du:rim.degree,dv:1,uknots:rim.knots.clone(),vknots:vec![0.,0.,1.,1.],
        poles:rim.poles.iter().map(|p| vec![[p[0],p[1],0.],*p]).collect(),weights:None};
    let (last,lastv) = (top.poles.len()-1,top.poles[0].len()-1);
    let walls = [top.row(0),top.row(last),top.column(0),top.column(lastv)].map(wall);
    let mut pieces = vec![cad_kernel::build::sheet(top.clone()).unwrap(),cad_kernel::build::sheet(floor).unwrap()];
    pieces[0].faces[0].name = "top".into();
    pieces.extend(walls.into_iter().map(|w| cad_kernel::build::sheet(w).unwrap()));
    let mut b = cad_kernel::sew::sewn(&pieces,1e-9).unwrap();
    if whole {
        let f = b.faces.iter_mut().find(|f| f.name == "top").unwrap();
        let Surface::BSpline(frame,_) = f.surface else { unreachable!("the top on its patch") };
        f.surface = Surface::BSpline(frame,Arc::new(patch.clone()));
    }
    b
}
