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
