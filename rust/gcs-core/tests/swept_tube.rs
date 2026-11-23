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

/// **The half turn built by the tube road** (#880 T1f, `tube::faces::solid`): its sheets' trims
/// traced, each run's ends the events they meet found exactly — the double curve of the sheet `−`
/// from its rim crossing (solved on the rim) joined to its swallowtail point along the normal form,
/// its partner switching to the end pose where the crossing's cover passes there, and that run
/// joined straight to the corner where the cuspidal edge reaches the roll's end — and the charts'
/// kept parts sewn: a closed solid passing `check` at the call's tolerance, its volume the old
/// road's within 1e-7 of itself and its mesh's within the mesh's bar, and twenty thousand seeded
/// points over its box each a member of the solid exactly where the depth (the least over the roll
/// of the distance to the posed core circle, less the tube) is below nought — read where either
/// stands clear of the boundary: the depth sampled over the roll and settled about its least
/// sample, a point outside only where every instant between the samples stands clear by the
/// point's speed over half a step.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the half turn built by the tube road, ~60 s")]
fn the_half_turn_builds_by_the_tube_road() {
    use gcs_core::brep::swept::tube::faces;
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let solid = faces::solid(&tube,tol).unwrap();
    solid.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&solid);
    eprintln!("the half turn by the tube road: {} faces, volume {v:.10}",solid.faces.len());
    assert!((v/12.2377154283-1.).abs() <= 1e-7,"the volume {v}, the old road's 12.2377154283");
    held(&solid,&tube,&path,roll,v);
}

/// A tube road's solid held to its mesh's volume and to membership by the depth: twenty thousand
/// seeded points over its box, each read where the solid or the depth stands clear of the
/// boundary (`the_half_turn_builds_by_the_tube_road`'s reading).
fn held(solid: &gcs_core::brep::topo::Brep,tube: &Tube,path: &gcs_core::brep::trajectory::Trajectory,roll: [f64;2],v: f64) {
    let bar = 1e-3;
    let m = gcs_core::brep::mesh::mesh(solid,bar,0.2).unwrap();
    let w = crate::brep::mesh_volume(&m);
    assert!((w-v).abs() <= 4.*bar*v.powf(2./3.)*6.,"the mesh's volume {w}, the solid's {v}");
    // membership by the solid and by the depth
    let body = gcs_core::brep::swept::member::Solid::new(solid,bar,1.).unwrap();
    let (lo,hi) = solid.bounds();
    let [a,b] = roll;
    let n = 256;
    let domain = gcs_core::brep::util::interval::Interval::new(a,b).unwrap();
    let depth = |x: [f64;3]| -> Option<bool> {
        let d = |t: f64| tube.depth_at(x,t).map_or(f64::INFINITY,|r| r.0);
        let samples: Vec<f64> = (0..=n).map(|i| d(a+(b-a)*i as f64/n as f64)).collect();
        let (i,&least) = samples.iter().enumerate().min_by(|p,q| p.1.total_cmp(q.1)).unwrap();
        // (settled by golden section between the samples beside the least)
        let step = (b-a)/n as f64;
        let (mut l,mut h) = ((a+step*i as f64-step).max(a),(a+step*i as f64+step).min(b));
        let g = 0.5*(5f64.sqrt()-1.);
        while h-l > 1e-12 { let (x1,x2) = (h-g*(h-l),l+g*(h-l)); if d(x1) < d(x2) { h = x2 } else { l = x1 } }
        let settled = d(0.5*(l+h)).min(least);
        let speed = path.inverse_point_speed_bound(x,domain).unwrap();
        let margin = 10.*bar;
        if settled < -margin { Some(true) } else if least-0.5*speed*step > margin { Some(false) } else { None }
    };
    let mut rng = gcs_core::rng::Rng::new(880);
    let (mut read,mut members) = (0,0);
    for _ in 0..20_000 {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k],hi[k]));
        let Some(inside) = depth(p) else { continue };
        let s = body.signed_distance(p);
        assert_eq!(inside,s < 0.,"at {p:?}: a member by the depth {inside}, the solid's distance {s}");
        read += 1;
        if inside { members += 1 }
    }
    eprintln!("{read} of 20000 points read clear of the boundary, {members} members");
    assert!(read >= 15_000 && members >= 1_000,"{read} read, {members} members");
}

/// **The crossed reverse member, its roll's end at 5°, built by the tube road** (#880 T1f): no pinch
/// and no cuspidal edge reaching a seam; the old road misclassifies a cell and refuses it (`Wrong`).
/// No volume is known apart, so it is held to its mesh and to membership by the depth: a closed
/// solid passing `check`, of 8 faces.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, membership read at twenty thousand points, ~20 s")]
fn the_crossed_reverse_member_at_five_degrees_builds_by_the_tube_road() {
    use gcs_core::brep::swept::tube::faces;
    let (tool,path,_,tol) = crate::common::bench::crossed(-1.);
    let roll = [-5f64.to_radians(),5f64.to_radians()];
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let solid = faces::solid(&tube,tol).unwrap();
    solid.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&solid);
    eprintln!("the crossed reverse member at 5° by the tube road: {} faces, volume {v:.10}",solid.faces.len());
    assert_eq!(solid.faces.len(),8);
    held(&solid,&tube,&path,roll,v);
}
