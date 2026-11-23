//! **The tube road** (#880 T1f, `swept::tube`): a torus's sweep read as the offset of the surface
//! its core circle sweeps — two sheets, each one smooth map over the whole `(θ, t)` rectangle, their
//! cuspidal edges and swallowtail points, and the depth over each.

use gcs_core::brep::swept::depth::Reading;
use gcs_core::brep::swept::rings::{Ring,changes,fitted_over};
use gcs_core::brep::swept::tube::{Chart,Tube,edges,swallowtails};
use std::f64::consts::TAU;
use crate::common::bench::tumbling;

/// **The half turn's two sheets are maps over the whole rectangle** (#880 T1f): each `X±` fitted by
/// `nurbs::fit_net` (quintic, no fold check: a map's net) to a tenth of the tolerance over
/// `θ ∈ [0, 2π]` and the whole roll; the sheet `+` has no cuspidal edge, the sheet `−` one, crossing
/// `θ ≡ 0` from the roll's end back to it, with one swallowtail point on it — the ring road's
/// swallowtail point (`rings::fitted_over`'s apex) to the rounding of its own reading. And `J`'s
/// sign is the reading's: on a grid over each sheet, inside the roll, every point with `J > 0` is
/// read kept or covered by another instant, and every point with `J ≤ 0` held by its neighbours.
#[test]
fn the_half_turns_sheets_are_maps_with_one_cuspidal_edge_and_the_ring_roads_swallowtail_point() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let [a,b] = roll;
    for side in [1.,-1.] {
        let f = |u: f64,v: f64| tube.point(side,u*TAU,a+v*(b-a)).unwrap().x;
        let (_,err) = gcs_core::brep::nurbs::fit_net(&f,5,5,32,32,0.1*tol).unwrap();
        assert!(err <= 0.1*tol,"the sheet {side}: fitted {err:e} off");
        for j in 1..16 {
            let t = a+(b-a)*j as f64/16.;
            for i in 0..32 {
                let th = TAU*(i as f64+0.5)/32.;
                let jac = tube.jacobian(side,th,t).unwrap();
                let near = matches!(tube.reading(side,th,t,tol),Some(Reading::Near));
                assert_eq!(jac > 0.,!near,"the sheet {side} at ({th}, {t}): J {jac}, held by its neighbours {near}");
            }
        }
    }
    let (plus,_) = edges(&tube,Chart::Sheet(1.),tol,0.25*ring.thickness).unwrap();
    assert!(plus.is_empty(),"the sheet + folds: {plus:?}");
    let (minus,_) = edges(&tube,Chart::Sheet(-1.),tol,0.25*ring.thickness).unwrap();
    assert_eq!(minus.len(),2,"the sheet −'s cuspidal edge, cut at θ 0: {minus:?}");
    let tails: Vec<[f64;2]> = minus.iter().flat_map(|p| swallowtails(&tube,-1.,&p.uv)).collect();
    let [tail] = tails[..] else { panic!("one swallowtail point: {tails:?}") };
    // (the ring road's, read on its stretch from the fold pair's birth)
    let (scale,cuts) = changes(&ring,&path,roll).unwrap();
    let fit = fitted_over(&ring,&path,roll,[cuts[0]+1e-9*(b-a),b],scale,0.1*tol).unwrap();
    let [apex] = fit.apexes[..] else { panic!("one swallowtail point on the ring road: {:?}",fit.apexes) };
    let (th,t) = (apex.at.0.rem_euclid(TAU),apex.at.1);
    assert!((tail[0]-th).abs() < 1e-6 && (tail[1]-t).abs() < 1e-6,"the swallowtail point at {tail:?}, the ring road's at ({th}, {t})");
}

/// **The half turn's seams traced from the depth on every chart** (#880 T1f, `tube::seams`): on the
/// sheet `−`, about its swallowtail point, a disc of radius 0.01 (halved from the feature until every
/// far cover on its rim is within the tolerance) cut from the depth's domain; its double curve's two
/// preimages each run from the disc's rim — one across `θ ≡ 0` to the roll's end, one to where its
/// cover passes to the end pose — and the end pose's one seam crosses `θ ≡ 0` too, leaving its
/// contact circle where the first preimage leaves the sheet's rim — the same crossing, but each
/// chart's read on its own: 1.64e-5 apart, five tolerances, the end pose touching the sheet along
/// that rim so a crossing of it is ill-conditioned along it (the face structure finds such a vertex
/// once, on the shared rim: not yet built). Nothing on the sheet `+` nor on the start pose.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the half turn's seams on four charts, ~60 s")]
fn the_half_turns_seams_are_traced_from_the_depth_on_every_chart() {
    use gcs_core::brep::swept::tube::seams;
    use gcs_core::brep::zero_set::End;
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let feature = 0.25*ring.thickness;
    let [_,b] = roll;
    let (plus,_,_) = seams(&tube,Chart::Sheet(1.),&[],&[],&[],tol,feature).unwrap();
    assert!(plus.is_empty(),"seams on the sheet +: {}",plus.len());
    let (folds,_) = edges(&tube,Chart::Sheet(-1.),tol,feature).unwrap();
    let folds: Vec<Vec<[f64;2]>> = folds.into_iter().map(|p| p.uv).collect();
    let tails: Vec<[f64;2]> = folds.iter().flat_map(|f| swallowtails(&tube,-1.,f)).collect();
    let (minus,_,discs) = seams(&tube,Chart::Sheet(-1.),&folds,&tails,&[],tol,feature).unwrap();
    let [disc] = discs[..] else { panic!("one disc: {discs:?}") };
    assert!((disc.radius-0.01).abs() < 1e-12,"the disc's radius {}",disc.radius);
    assert_eq!(minus.len(),3,"the sheet −'s seams: {:?}",minus.iter().map(|p| (p.uv[0],p.uv[p.uv.len()-1],p.ends)).collect::<Vec<_>>());
    assert_eq!(minus.iter().filter(|p| p.ends.contains(&End::Edge(0))).count(),2,"two pieces from the disc's rim");
    for k in 0..2 {
        let (circles,_) = edges(&tube,Chart::End(k),tol,feature).unwrap();
        assert_eq!(circles.len(),8,"the end {k}'s contact circles, in quarters");
    }
    let (start,_,_) = seams(&tube,Chart::End(0),&[],&[],&[],tol,feature).unwrap();
    assert!(start.is_empty(),"seams on the start pose: {}",start.len());
    // (the end pose's seam leaves its contact circle where the sheet's leaves its rim at the roll's end)
    let circles: Vec<Vec<[f64;2]>> = edges(&tube,Chart::End(1),tol,feature).unwrap().0.into_iter().map(|p| p.uv).collect();
    let (end,_,_) = seams(&tube,Chart::End(1),&circles,&[],&[],tol,feature).unwrap();
    assert_eq!(end.len(),2,"the end pose's seam, cut at θ 0");
    let on_rim: Vec<[f64;2]> = minus.iter().flat_map(|p| [p.uv[0],p.uv[p.uv.len()-1]]).filter(|q| q[1] == b).collect();
    let [rim] = on_rim[..] else { panic!("one seam leaving the sheet's rim at the roll's end: {on_rim:?}") };
    let x = tube.at(Chart::Sheet(-1.),rim).unwrap().x;
    let near = end.iter().flat_map(|p| [p.uv[0],p.uv[p.uv.len()-1]]).map(|q| {
        let y = tube.at(Chart::End(1),q).unwrap().x;
        ((x[0]-y[0]).powi(2)+(x[1]-y[1]).powi(2)+(x[2]-y[2]).powi(2)).sqrt()
    }).fold(f64::INFINITY,f64::min);
    assert!(near <= 8.*tol,"the end pose's seam {near:e} from the sheet's at the rim");
}
