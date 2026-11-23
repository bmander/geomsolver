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

/// **The motion's rate bounds hold** (#880 T1f, `Trajectory::point_jet_bound_over`): for the half
/// turn's motion (a turn seen from a turn), at points about the tool the acceleration of
/// `M(t)⁻¹ x` read by second differences over the roll never passes its bound, and the bound is
/// within a few times what is read (no bound so loose the cover's steps stall); at the core
/// circle's points the acceleration, jerk and fourth rate of `M(t) x` read by differences never pass
/// the circle's (`Tube::rates`); a path evaluated gives no bound.
#[test]
fn the_motions_rate_bounds_hold() {
    use gcs_core::brep::util::interval::Interval;
    let (tool,path,[a,b],_) = tumbling(90.);
    let domain = Interval::new(a,b).unwrap();
    let mut rng = gcs_core::rng::Rng::new(880);
    for _ in 0..32 {
        let x: [f64;3] = std::array::from_fn(|_| rng.uniform(-5.,5.));
        let bound = path.inverse_point_acceleration_bound(x,domain).unwrap();
        let h = 1e-4;
        let q = |t: f64| path.pose(t).unwrap().inverse().point(x);
        let mut most = 0f64;
        for k in 1..400 {
            let t = a+(b-a)*k as f64/400.;
            let (l,m,r) = (q(t-h),q(t),q(t+h));
            let acc = (0..3).map(|i| ((l[i]-2.*m[i]+r[i])/(h*h)).powi(2)).sum::<f64>().sqrt();
            most = most.max(acc);
        }
        assert!(most <= bound*(1.+1e-5),"at {x:?}: an acceleration of {most} read, the bound {bound}");
        assert!(bound <= 4.*most,"at {x:?}: the bound {bound}, the most read {most}");
    }
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,[a,b]).unwrap();
    let rates = tube.rates.unwrap();
    let f = &ring.frame;
    let h = 1e-2;
    let mut most = [0f64;3];
    for i in 0..16 {
        let (s,c) = (TAU*i as f64/16.).sin_cos();
        let x = f.at([tube.radius*c,tube.radius*s,tube.height]);
        let p = |t: f64| path.pose(t).unwrap().point(x);
        for k in 2..198 {
            let t = a+(b-a)*k as f64/200.;
            let v: Vec<[f64;3]> = (-2..=2).map(|j| p(t+h*j as f64)).collect();
            let n = |w: [f64;3]| (w[0]*w[0]+w[1]*w[1]+w[2]*w[2]).sqrt();
            let d2 = n(std::array::from_fn(|i| (v[1][i]-2.*v[2][i]+v[3][i])/(h*h)));
            let d3 = n(std::array::from_fn(|i| (v[4][i]-2.*v[3][i]+2.*v[1][i]-v[0][i])/(2.*h*h*h)));
            let d4 = n(std::array::from_fn(|i| (v[4][i]-4.*v[3][i]+6.*v[2][i]-4.*v[1][i]+v[0][i])/(h*h*h*h)));
            for (m,d) in most.iter_mut().zip([d2,d3,d4]) { *m = m.max(d) }
        }
    }
    for k in 0..3 { assert!(most[k] <= rates.jet[k+1]*1.01,"the circle's rate {}: {} read, the bound {}",k+2,most[k],rates.jet[k+1]) }
    let evaluated = gcs_core::brep::trajectory::Trajectory::General(std::sync::Arc::new(Evaluated(path.clone())));
    assert!(evaluated.inverse_point_acceleration_bound([1.,2.,3.],domain).is_err());
}

/// A motion read only by evaluating it.
#[derive(Debug)]
struct Evaluated(gcs_core::brep::trajectory::Trajectory);
impl gcs_core::brep::trajectory::Path for Evaluated {
    fn at(&self,t: f64) -> Result<gcs_core::brep::trajectory::Motion,gcs_core::brep::KernelError> { self.0.at(t) }
}

/// **The tube's certified cover reads as `depth::cover` does** (#880 T1f, `Tube::cover`): on a grid
/// over both sheets of the half turn, a point held by its neighbours is held by both readings; where
/// both read a far cover its depth agrees to the tolerance; and where `depth::cover`'s march reads
/// the point alone the tube's reads it alone or a far cover not below nought — its march, finer
/// about the top of the point's own contact, finds a fall past it the sampled march stepped over.
#[test]
fn the_tubes_cover_reads_as_the_depths() {
    use gcs_core::brep::swept::depth::cover;
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let [a,b] = roll;
    let (mut far,mut alone) = (0,0);
    for side in [1.,-1.] {
        for j in 0..=12 { for i in 0..24 {
            let (th,t) = (TAU*(i as f64+0.5)/24.,a+(b-a)*j as f64/12.);
            let Some(p) = tube.point(side,th,t) else { continue };
            let new = tube.cover(p.x,t,tol).unwrap();
            let old = cover(&ring,&path,roll,p.x,t,0.,tol).unwrap();
            match (new,old) {
                (Reading::Far(n),Reading::Far(o)) => {
                    assert!((n.depth-o.depth).abs() <= tol,"({th}, {t}) on {side}: {} against {}",n.depth,o.depth);
                    far += 1;
                }
                (Reading::Near,Reading::Near) => {}
                (Reading::Alone,Reading::Alone) => alone += 1,
                (Reading::Far(n),Reading::Alone) => assert!(n.depth >= 0.,"({th}, {t}) on {side}: a cover {} the march read alone",n.depth),
                (n,o) => panic!("({th}, {t}) on {side}: {n:?} against {o:?}"),
            }
        } }
    }
    assert!(far >= 100 && alone >= 10,"{far} far covers, {alone} alone");
    // one such point, the depth sampled densely as the oracle: on the sheet `+` its own contact
    // climbs to a top 2.2120598518 at τ 1.568359, inside the roll's last stretch, then falls
    // 2.96e-6 — less than the tolerance, more than the rounding — to the end pose's 2.2120568897.
    // `depth::cover`'s march steps from below the top to the roll's end and reads no fall (`Alone`),
    // within its contract (no stretch held more than the tolerance deep is stepped over); the tube's
    // reads the far cover at the end. Both read it uncovered, by more than two units: no decision of
    // the old road on the sign of a cover reads it otherwise.
    let (th,t) = (TAU*12.5/48.,a+(b-a)*5./24.);
    let x = tube.point(1.,th,t).unwrap().x;
    let n = 200_000;
    let ds: Vec<(f64,f64)> = (0..=n).map(|k| { let s = t+(b-t)*k as f64/n as f64; (s,tube.depth_at(x,s).unwrap().0) }).collect();
    let top = ds.iter().cloned().fold((0.,f64::NEG_INFINITY),|m,p| if p.1 > m.1 { p } else { m });
    let fall = top.1-ds[n].1;
    assert!(top.0 < b && fall > 1e-12 && fall < tol,"the top {top:?}, the fall {fall:e} to the end");
    assert!(matches!(cover(&ring,&path,roll,x,t,0.,tol).unwrap(),Reading::Alone));
    let Reading::Far(c) = tube.cover(x,t,tol).unwrap() else { panic!("the tube's cover reads no far cover") };
    assert!(c.at == b && (c.depth-ds[n].1).abs() <= 1e-12,"{c:?} against the end's {}",ds[n].1);
}

/// **The charts' sides are proved over cells** (#880 T1f, `Tube::jacobian_bound` and
/// `Tube::contact_bound`, the bounds of `tube::edges`'s search): over the half turn's two sheets cut
/// into 256 × 128 cells and its end poses into 64 × 64, every enclosure a bound gives holds the side
/// (`Tube::side`) read at nine points of its cell and proves its sign, and most cells are proved —
/// on the sheets all but those about the sheet `−`'s cuspidal edge and where the spine's two rates
/// stand near parallel, on the end poses all but those about the contact circles.
#[test]
fn the_charts_sides_are_bounded_over_cells() {
    let (tool,path,roll,_) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    for (chart,(nu,nv),least) in [(Chart::Sheet(1.),(256,128),0.8),(Chart::Sheet(-1.),(256,128),0.7),
        (Chart::End(0),(64,64),0.8),(Chart::End(1),(64,64),0.8)] {
        let r = tube.rect(chart);
        let mut proved = 0;
        for i in 0..nu { for j in 0..nv {
            let at = |k: usize,f: f64| r[k][0]+(r[k][1]-r[k][0])*f;
            let c = [[at(0,i as f64/nu as f64),at(0,(i+1) as f64/nu as f64)],[at(1,j as f64/nv as f64),at(1,(j+1) as f64/nv as f64)]];
            let bound = match chart { Chart::Sheet(side) => tube.jacobian_bound(side,c),Chart::End(k) => tube.contact_bound(k,c),Chart::Collar => None };
            let Some(bound) = bound else { continue };
            proved += 1;
            let [lo,hi] = bound.value.bounds().unwrap();
            assert!(lo > 0. || hi < 0.,"{chart:?} over {c:?}: [{lo}, {hi}] proves no sign");
            for p in 0..3 { for q in 0..3 {
                let uv = [c[0][0]+(c[0][1]-c[0][0])*p as f64/2.,c[1][0]+(c[1][1]-c[1][0])*q as f64/2.];
                let side = tube.side(chart,uv).unwrap();
                assert!(side >= lo && side <= hi,"{chart:?} at {uv:?}: its side {side} outside [{lo}, {hi}]");
            } }
        } }
        assert!(proved as f64 >= least*(nu*nv) as f64,"{chart:?}: {proved} of {} cells proved",nu*nv);
    }
}

/// **A far cover continued into the point's own contact ends** (#880 T1f, `Depth::along`): on the
/// full turn's sheet `−`, beside its second swallowtail point, the double curve's two instants meet
/// (2.226 against the point's own 2.246); past there a cover continued from an instant beside the
/// point's own finds its least at the stretch's cut toward that instant, the own contact's slope —
/// no basin of another instant. Read on, each corrector's pass halved the way again and the cover
/// slid into the own contact, nought everywhere on the sheet: a phantom branch along `t` = 2.246 to
/// the cut `θ = π`, where the true depth is 1.28. Now it ends there (none read), and the point's
/// least is its true far cover, clear above nought.
#[test]
fn a_far_cover_continued_into_the_points_own_contact_ends() {
    use gcs_core::brep::swept::depth::Cover;
    use gcs_core::brep::swept::tube::Depth;
    use gcs_core::brep::zero_set::Field;
    let (tool,path,roll,tol) = tumbling(180.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let field = Depth {tube:&tube,chart:Chart::Sheet(-1.),within:tol,holes:&[]};
    let uv = [2.690779,2.245987];
    let from = Cover {depth:0.,at:2.2459,point:[0.;3],piece:0};
    let along = field.along(uv,&from);
    assert!(along.is_none(),"a cover continued into the own contact: {:?}",along.map(|v| (v.f,v.at.at)));
    let least = field.least(uv).unwrap();
    assert!(least.f > 0.9,"the far cover there {} at {}",least.f,least.at.at);
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
    // (an end on an inner edge named by that edge, the folds' first: the disc's rim is read by its side)
    let on_disc = |p: &gcs_core::brep::zero_set::Piece<_>| [(p.ends[0],p.uv[0]),(p.ends[1],p.uv[p.uv.len()-1])].iter()
        .any(|(e,q)| matches!(e,End::Edge(_)) && disc.side(*q).abs() <= 1e-6);
    assert_eq!(minus.iter().filter(|p| on_disc(p)).count(),2,"two pieces from the disc's rim: {:?}",
        minus.iter().map(|p| (p.ends,disc.side(p.uv[0]),disc.side(p.uv[p.uv.len()-1]))).collect::<Vec<_>>());
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

/// **The full turn built by the tube road** (#880 T1f): a double curve's far cover continued until
/// its instant closes on the point's own ends there (`Depth::along`, no phantom branch along the own
/// contact); a preimage seeded across a cut traced though its other preimage stands on it in space
/// (`zero_set`'s near test read in the chart); a switch whose sheet-covered side is the run kept,
/// its rim crossing solved on its partner's rim; the two end poses' meeting read as the start
/// pose's trims against the end pose (`depth::ends_met`'s two short arcs), each between two rim
/// crossings standing on both poses; the cut from a swallowtail point ended at the first trim it
/// meets, that trim split there; and an island of a window a hole of the face about it. 14 faces,
/// `check`, its volume the old road's 23.6738852531 within 1e-7 of itself (23.6738858153 read),
/// held to its mesh and to membership by the depth.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the full turn built by the tube road, ~90 s")]
fn the_full_turn_builds_by_the_tube_road() {
    use gcs_core::brep::swept::tube::faces;
    let (tool,path,roll,tol) = tumbling(180.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let solid = faces::solid(&tube,tol).unwrap();
    solid.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&solid);
    eprintln!("the full turn by the tube road: {} faces, volume {v:.10}",solid.faces.len());
    assert_eq!(solid.faces.len(),14);
    assert!((v/23.6738852531-1.).abs() <= 1e-7,"the volume {v}, the old road's 23.6738852531");
    held(&solid,&tube,&path,roll,v);
}

/// **The thin built by the tube road** (#880 T1f, brep-bench's `tumbling-torus-thin`, tube 0.015): its
/// sheet `−`'s depth crosses its rim at the roll's end shallowly, so the trace's rim root, read
/// through the certified cover to its tolerance, changed sign twice about the one crossing — a
/// way from one reading leaving at once onto the other, a two-point piece along the rim that the
/// run joined and doubled back on past its crossing (misfit 3.3e-7 against 3e-8); now no piece
/// (`zero_set`: a way's two points on the one side it started on are that side's crossing read
/// twice). 10 faces, `check`, its volume the old road's 0.6963027896 within 1e-7 of itself
/// (0.6963027945 read), held to its mesh and to membership by the depth.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the thin built by the tube road, ~300 s")]
fn the_thin_builds_by_the_tube_road() {
    use gcs_core::brep::swept::tube::faces;
    let (tool,path,roll,tol) = crate::common::bench::thin();
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let solid = faces::solid(&tube,tol).unwrap();
    solid.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&solid);
    eprintln!("the thin by the tube road: {} faces, volume {v:.10}",solid.faces.len());
    assert_eq!(solid.faces.len(),10);
    assert!((v/0.6963027896-1.).abs() <= 1e-7,"the volume {v}, the old road's 0.6963027896");
    // (a tube of 0.015 fills a part in a hundred of its box: read at enough points for a thousand
    // members)
    held_at(&solid,&tube,&path,roll,v,160_000);
}

/// **The bench's tumbling torus built by the tube road** (#880 T1f, `tumbling-torus`, tube 0.5): its
/// sheet `−`'s double curve with the end pose runs into a corner where the two stand within 1e-12
/// of one another, its zero set the very edge of where the end pose is a cover past the point's own
/// contact (no cover at all beyond: its basin merged into the own contact); the march ends there
/// (`zero_set::End::Undefined`) and the run is joined straight to the corner, every point of the
/// join within the tolerance of the end pose. Its trims on the sheet `−` run over 2.5 of a turn's
/// angle and their partners on the end pose over the rest, so each chart has its own windows. 8
/// faces, `check`, its volume the one made apart (`swept_rings`'s bench spelling, 25.2750727653)
/// within 1e-7 of itself (25.2750727786 read, 5e-10 apart), held to its mesh and to membership by
/// the depth. The old road at this spelling of the motion reads 25.2750392926, 1.3e-6 below: its
/// solid's faces carry surface covered by other instants (nine points of its mesh at 0.003 covered
/// 1e-5 to 3.4e-3 deep, on three faces), the tube road's none — gated here (`covered`). The three
/// volumes: the tube road's 25.2750727786, the one made apart 25.2750727653, the old road's
/// 25.2750392467 (25.2750392926 as main reads it).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the tumbling torus built by both roads, ~120 s")]
fn the_tumbling_torus_builds_by_the_tube_road() {
    use gcs_core::brep::swept::tube::faces;
    let (tool,path,roll,tol) = crate::common::bench::tumbling_torus();
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let solid = faces::solid(&tube,tol).unwrap();
    solid.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&solid);
    eprintln!("the tumbling torus by the tube road: {} faces, volume {v:.10}",solid.faces.len());
    assert_eq!(solid.faces.len(),8);
    assert!((v/25.2750727653-1.).abs() <= 1e-7,"the volume {v}, the one made apart 25.2750727653");
    held(&solid,&tube,&path,roll,v);
    // the boundary's points under other instants: the tube road's none; the old road's, at this
    // spelling of the motion, some, deep
    let (n,_) = covered(&solid,&tube,roll,tol);
    assert_eq!(n,0,"the tube road's boundary covered deeper than the tolerance at {n} points");
    let old = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.5/250.,tol).unwrap();
    let w = gcs_core::brep::props::volume(&old);
    let (n,worst) = covered(&old,&tube,roll,tol);
    eprintln!("the old road: volume {w:.10}, {n} points of its mesh covered deeper than the tolerance, the deepest {worst:e}");
    assert!((w/25.2750727653-1.).abs() > 1e-6,"the old road's volume {w}, within 1e-6 of the one made apart");
    assert!(n > 0 && worst < -100.*tol,"the old road's boundary: {n} points covered, the deepest {worst:e}");
}

/// **How many points of a solid's boundary lie under other instants**, and the deepest: its mesh at
/// 0.003 read by the depth (the least over the roll of the distance to the posed core circle less
/// the tube, sampled at 512 instants and settled by golden section about each sampled basin), a
/// point counted where it stands deeper than the tolerance.
fn covered(solid: &gcs_core::brep::topo::Brep,tube: &Tube,[a,b]: [f64;2],tol: f64) -> (usize,f64) {
    let m = gcs_core::brep::mesh::mesh(solid,0.003,0.2).unwrap();
    let depth = |x: [f64;3]| -> f64 {
        let d = |t: f64| tube.depth_at(x,t).map_or(f64::INFINITY,|r| r.0);
        let n = 512;
        let step = (b-a)/n as f64;
        let s: Vec<f64> = (0..=n).map(|i| d(a+step*i as f64)).collect();
        let mut best = f64::INFINITY;
        for i in 0..=n {
            if i > 0 && i < n && !(s[i] <= s[i-1] && s[i] <= s[i+1]) { continue }
            let (mut l,mut h) = ((a+step*i as f64-step).max(a),(a+step*i as f64+step).min(b));
            let g = 0.5*(5f64.sqrt()-1.);
            while h-l > 1e-13 { let (x1,x2) = (h-g*(h-l),l+g*(h-l)); if d(x1) < d(x2) { h = x2 } else { l = x1 } }
            best = best.min(d(0.5*(l+h))).min(s[i]);
        }
        best
    };
    m.pts.iter().map(|&x| depth(x)).filter(|&d| d < -tol).fold((0,0f64),|(n,w),d| (n+1,w.min(d)))
}

/// **The 80° torus built by the tube road** (#880 T1f): the sheet `−`'s cuspidal edges run the whole
/// roll with no swallowtail point, each meeting the roll's ends at corners. 8 faces, `check`, its
/// volume the old road's 10.9393141414 within 1e-7 of itself (10.9393141712 read), held to its mesh
/// and to membership by the depth.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, membership read at twenty thousand points, ~30 s")]
fn the_eighty_degree_torus_builds_by_the_tube_road() {
    use gcs_core::brep::swept::tube::faces;
    let (tool,path,roll,tol) = tumbling(80.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let solid = faces::solid(&tube,tol).unwrap();
    solid.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&solid);
    eprintln!("the 80° torus by the tube road: {} faces, volume {v:.10}",solid.faces.len());
    assert_eq!(solid.faces.len(),8);
    assert!((v/10.9393141414-1.).abs() <= 1e-7,"the volume {v}, the old road's 10.9393141414");
    held(&solid,&tube,&path,roll,v);
}

/// A tube road's solid held to its mesh's volume and to membership by the depth: twenty thousand
/// seeded points over its box, each read where the solid or the depth stands clear of the
/// boundary (`the_half_turn_builds_by_the_tube_road`'s reading).
fn held(solid: &gcs_core::brep::topo::Brep,tube: &Tube,path: &gcs_core::brep::trajectory::Trajectory,roll: [f64;2],v: f64) {
    held_at(solid,tube,path,roll,v,20_000)
}

/// `held` at `count` seeded points, three quarters of them read clear of the boundary and a
/// thousand of those members: a thin body fills little of its box, and is read at more.
fn held_at(solid: &gcs_core::brep::topo::Brep,tube: &Tube,path: &gcs_core::brep::trajectory::Trajectory,roll: [f64;2],v: f64,count: usize) {
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
    for _ in 0..count {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k],hi[k]));
        let Some(inside) = depth(p) else { continue };
        let s = body.signed_distance(p);
        assert_eq!(inside,s < 0.,"at {p:?}: a member by the depth {inside}, the solid's distance {s}");
        read += 1;
        if inside { members += 1 }
    }
    eprintln!("{read} of {count} points read clear of the boundary, {members} members");
    assert!(4*read >= 3*count && members >= 1_000,"{read} of {count} read, {members} members");
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

/// **The collar charts the pinch of each crossed member** (#880 T1f, `Tube::pinched`,
/// `Chart::Collar`): where an instant touches a meridian whole the core circle's sweep is singular
/// and the collar `(ψ, ρ)` charts both sheets about it. `ρ₀` by its rule — half the least of the
/// collar's reach (the terms' scale halved until its rim is read whole) and the `|ρ|` of the
/// swallowtail points on the collar's own cuspidal edges inside it: forward the reach 0.1742 holds
/// the two at `ρ` 0.0710 and 0.087, so `ρ₀` 0.0355; reverse the reach 0.1091 holds none, so
/// 0.0545. The collar is the sheets: at its rims each of its points is the sheet's point at its
/// place (`ρ > 0` the sheet `+`), its side the sheet's `J` in sign; and its depth's zero set is
/// what the old road's collar reading measured, trims crossing it rim to rim at `ψ` 175.5° and
/// 337.9° reverse (175.333° and 337.210° on the meridian) and at 72.83° forward (72.883°), each
/// with its covering instant past its own on the strip's side. And the sheets read up to the
/// hole's rim: their depth stops there, their runs ending on it where the collar's begin (reverse,
/// own instants 0.867 and 0.970 on both).
#[test]
fn the_collar_charts_the_crossed_members_pinch() {
    use gcs_core::brep::swept::tube::seams;
    for (observer,rho0,psis) in [(1.,0.0355,vec![72.83]),(-1.,0.0545,vec![175.5,337.9])] {
        let (tool,path,roll,tol) = crate::common::bench::crossed(observer);
        let ring = Ring::read(&tool).unwrap();
        let feature = 0.25*ring.thickness;
        let tube = Tube::pinched(&ring,&path,roll,tol,feature).unwrap();
        let pinch = tube.pinch.as_ref().expect("a pinch");
        eprintln!("observer {observer}: ρ₀ {}, at {:?}",pinch.rho,pinch.collar.at);
        assert!((pinch.rho-rho0).abs() < 1e-3*rho0.max(0.05),"ρ₀ {} against {rho0}",pinch.rho);
        // the collar is the sheets at its rims, and its side is their J in sign
        for i in 0..24 {
            let psi = TAU*i as f64/24.;
            for side in [1.,-1.] {
                let rho = side*pinch.rho;
                let p = tube.at(Chart::Collar,[psi,rho]).unwrap();
                let [th,t] = pinch.rim(side,psi).unwrap();
                let q = tube.point(side,th,t).unwrap();
                assert!(gcs_core::brep::util::space::distance(p.x,q.x) < 1e-12,"the collar off the sheet {side} at ψ {psi}");
                let (j,s) = (tube.jacobian(side,th,t).unwrap(),tube.side(Chart::Collar,[psi,rho]).unwrap());
                assert!(j.abs() < 1e-6 || (j > 0.) == (s > 0.),"the collar's side {s} against J {j} at ψ {psi}, ρ {rho}");
            }
        }
        // the collar's trims, rim to rim
        let (folds,_) = edges(&tube,Chart::Collar,tol,feature).unwrap();
        let lines: Vec<Vec<[f64;2]>> = folds.into_iter().map(|p| p.uv).collect();
        let (pieces,_,_) = seams(&tube,Chart::Collar,&lines,&[],&[],tol,feature).unwrap();
        for &want in &psis {
            let p = pieces.iter().find(|p| (p.uv[0][0].to_degrees()-want).abs() < 1.).unwrap_or_else(|| panic!("no trim at ψ {want}°"));
            let n = p.uv.len();
            assert_eq!(p.uv[0][1].abs(),pinch.rho);
            assert_eq!(p.uv[n-1][1].abs(),pinch.rho);
            assert!(p.uv[0][1]*p.uv[n-1][1] < 0.,"a trim at ψ {want}° not across the collar");
        }
    }
}

/// **The crossed reverse's sheet `−` holds a closed cuspidal edge folded beside its fold loop**
/// (#880 T1f): beside the near-pinch at `(θ, t)` (1.076, 0.461) the spine doubles back, and a lobe
/// of the sheet where `J` stands below nought (θ 1.02 to 1.44, t 0.16 to 0.44) lies within 2e-3 to
/// 5e-2 of the fold loop through `(0.535, 0.873)` in space, under the feature: the search answered
/// its cells beside that loop and never traced it, and the double curve ending on it at its two
/// swallowtail points ran to `Undefined` ends with no event there. Now both closed edges are traced
/// (`zero_set`'s beside reads the sign), and the lobe's two swallowtail points are where the
/// double curve's runs end, (1.0201, 0.3481) and (1.4383, 0.1616).
#[test]
fn the_crossed_reverse_sheets_lobe_beside_its_fold_loop_is_traced() {
    let (tool,path,roll,tol) = crate::common::bench::crossed(-1.);
    let ring = Ring::read(&tool).unwrap();
    let feature = 0.25*ring.thickness;
    let tube = Tube::pinched(&ring,&path,roll,tol,feature).unwrap();
    let (folds,_) = edges(&tube,Chart::Sheet(-1.),tol,feature).unwrap();
    let closed: Vec<_> = folds.iter().filter(|p| p.closed()).collect();
    assert_eq!(closed.len(),2,"two closed cuspidal edges: {:?}",folds.iter().map(|p| (p.ends,p.uv[0])).collect::<Vec<_>>());
    let lobe = closed.iter().find(|p| p.uv.iter().all(|q| q[1] < 0.47)).expect("the lobe below the near-pinch");
    let tails = swallowtails(&tube,-1.,&lobe.uv);
    assert_eq!(tails.len(),2,"the lobe's swallowtail points {tails:?}");
    for want in [[1.0201,0.3481],[1.4383,0.1616]] {
        assert!(tails.iter().any(|q| (q[0]-want[0]).abs() < 1e-3 && (q[1]-want[1]).abs() < 1e-3),"no swallowtail point at {want:?}: {tails:?}");
    }
}

/// **A crossed member built by the tube road, held** (#880 T1f): the bench's exact call
/// (`closed::ring` over the bench's roll, its bar the tube's 250th and its tolerance the 50 000th,
/// which the old road's refusal at the pinch sends to the tube road), `check`, its volume within
/// 1e-5 of the peer kernel cadrs's (`want`), held to its mesh and to membership by the depth, none
/// of its mesh's points covered, its boundary audited by the depth (`audited`), no misfit at the
/// size's tolerance; and built again with the observer's origin moved by one ulp
/// and by 1e-6, each to the same volume within 1e-5. The faces' count and the volume returned.
fn crossed_held(observer: f64,want: f64) -> (usize,f64) {
    let build = |moved: &dyn Fn(f64) -> f64| {
        let (tool,path,roll,tol) = crate::common::bench::crossed_moved(observer,|o| o.map(moved));
        // (through `closed::ring`, which the old road's refusal at the pinch sends to the tube road)
        let solid = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.22/250.,tol).unwrap();
        solid.check(tol).unwrap();
        (gcs_core::brep::props::volume(&solid),solid,tool,path,roll,tol)
    };
    let clock = std::time::Instant::now();
    let (v,solid,tool,path,roll,tol) = build(&|x| x);
    eprintln!("the crossed member {observer:+}: {} faces, volume {v:.10} (cadrs {want}) in {:?}",solid.faces.len(),clock.elapsed());
    assert!((v/want-1.).abs() <= 1e-5,"the volume {v}, cadrs's {want}");
    let (lo,hi) = solid.bounds();
    let size = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
    let misfits = solid.misfits(gcs_core::brep::geom::tolerance(size));
    assert!(misfits.is_empty(),"misfits: {misfits:?}");
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::pinched(&ring,&path,roll,tol,0.25*ring.thickness).unwrap();
    held(&solid,&tube,&path,roll,v);
    let (n,worst) = covered(&solid,&tube,roll,tol);
    assert_eq!(n,0,"the boundary covered deeper than the tolerance at {n} points, the deepest {worst:e}");
    for delta in [2e-5,2.*tol] { audited(&solid,&tube,roll,delta,tol) }
    for (what,moved) in [("one ulp",&(|x: f64| x.next_up()) as &dyn Fn(f64) -> f64),("1e-6",&|x: f64| x+1e-6)] {
        let (w,..) = build(moved);
        eprintln!("  the observer's origin moved by {what}: volume {w:.10}");
        assert!((w/v-1.).abs() <= 1e-5,"moved by {what}: the volume {w} against {v}");
    }
    (solid.faces.len(),v)
}

/// **A solid's boundary audited by the depth alone** (#880 T1f): each point of its mesh at 1e-3
/// stepped `delta` out along its face's normal (its mesh's way out) stands outside the sweep by the depth, stepped
/// in, inside — read where the step stays clear of every other face, the point farther than
/// `8 delta` from its face's edges. A region of a chart kept though another instant covers it,
/// or a face missing over a region kept beside it, reads apart here; a mesh's membership
/// (`member::Solid`, its sag the bar's) cannot read a step below its bar, and read the crossed
/// forward member's boundary apart at 2e-5 where it was right.
fn audited(solid: &gcs_core::brep::topo::Brep,tube: &Tube,[a,b]: [f64;2],delta: f64,tol: f64) {
    use gcs_core::brep::util::space::{add,cross,dot,norm,scale,sub};
    let m = gcs_core::brep::mesh::mesh(solid,1e-3,0.2).unwrap();
    let mut normal = vec![[0f64;3];m.pts.len()];
    let mut face = vec![u32::MAX;m.pts.len()];
    // (each face's own edges: its triangles' sides used once among them)
    let mut sides: std::collections::BTreeMap<(u32,u32,u32),usize> = std::collections::BTreeMap::new();
    for (t,&f) in m.tris.iter().zip(&m.of) {
        let [p,q,r] = t.map(|i| m.pts[i as usize]);
        let n = cross(sub(q,p),sub(r,p));
        for &i in t { for k in 0..3 { normal[i as usize][k] += n[k] } face[i as usize] = f }
        for (i,j) in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])] { *sides.entry((f,i.min(j),i.max(j))).or_insert(0) += 1 }
    }
    let edges: Vec<([f64;3],[f64;3])> = sides.iter().filter(|(_,&n)| n == 1).map(|(&(_,i,j),_)| (m.pts[i as usize],m.pts[j as usize])).collect();
    let clear = |x: [f64;3]| edges.iter().all(|&(p,q)| {
        let d = sub(q,p);
        let s = (dot(sub(x,p),d)/dot(d,d)).clamp(0.,1.);
        norm(sub(x,add(p,scale(d,s)))) > 8.*delta
    });
    let depth = |x: [f64;3]| -> f64 {
        let d = |t: f64| tube.depth_at(x,t).map_or(f64::INFINITY,|r| r.0);
        let n = 512;
        let step = (b-a)/n as f64;
        let s: Vec<f64> = (0..=n).map(|i| d(a+step*i as f64)).collect();
        let mut best = f64::INFINITY;
        for i in 0..=n {
            if i > 0 && i < n && !(s[i] <= s[i-1] && s[i] <= s[i+1]) { continue }
            let (mut l,mut h) = ((a+step*i as f64-step).max(a),(a+step*i as f64+step).min(b));
            let g = 0.5*(5f64.sqrt()-1.);
            while h-l > 1e-13 { let (x1,x2) = (h-g*(h-l),l+g*(h-l)); if d(x1) < d(x2) { h = x2 } else { l = x1 } }
            best = best.min(d(0.5*(l+h))).min(s[i]);
        }
        best
    };
    let (mut read,mut turned,mut apart) = (0,0,Vec::new());
    for (i,&x) in m.pts.iter().enumerate() {
        let l = norm(normal[i]);
        if !(l > 0.) || !clear(x) { continue }
        // (the face's own normal at the point, out of the solid: a mesh's turns sharply beside a
        // swallowtail point, where the sheet's fold bends it at the scale of the step)
        // (where the surface's inverse stands back at the point within the tolerance: a net over a
        // window holding the pinch's hole folds over it, and its inverse can land on the other leaf)
        let f = &solid.faces[face[i] as usize];
        let uv = f.surface.inverse(x);
        let Some(n) = f.surface.normal(uv) else { continue };
        if !(norm(sub(f.surface.point(uv),x)) <= tol) { continue }
        let n = scale(n,if f.reversed { -1. } else { 1. });
        if !(dot(n,normal[i]) > 0.) { turned += 1; continue }
        for sign in [1.,-1.] {
            let d = depth(add(x,scale(n,sign*delta)));
            read += 1;
            if !(sign*d > 0.) { apart.push((face[i],x,sign,d)) }
        }
    }
    eprintln!("  the boundary audited at {delta:e}: {read} steps read of {} points, {turned} turned from the mesh, {} apart",m.pts.len(),apart.len());
    assert!(read >= m.pts.len() && 100*turned <= m.pts.len() && apart.is_empty(),"at {delta:e}, {read} read, apart (face, point, way, depth): {:?}",&apart[..apart.len().min(8)]);
}

/// **The crossed forward member built by the tube road** (#880 T1f): its pinch charted by the
/// collar at `(t*, θ*)` (1.3264, 5.9712), sheet `−`'s two double curves with itself ending at
/// swallowtail points. 14 faces, volume 16.6109998153 against cadrs's 16.6109997999 — once its
/// flux was read cut at each pcurve's crossings of its graded net's knot lines (`props`; read
/// whole, 16.6116290673, the face holding the pinch's hole 6.6e-4 off). About 15 s a build.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the crossed forward built three times, ~2 min")]
fn the_crossed_forward_member_builds_by_the_tube_road() {
    let (faces,v) = crossed_held(1.,16.610999799920087);
    assert_eq!(faces,14);
    assert!((v-16.6109998153).abs() <= 1e-9,"the volume {v}, pinned 16.6109998153");
}

/// **The crossed reverse member built by the tube road** (#880 T1f): its pinch charted by the
/// collar, the closed cuspidal edge beside its fold loop traced (the lobe below the near-pinch), the
/// double curves ending at its swallowtail points joined there. 15 faces, volume 13.7898471917
/// against cadrs's 13.7898472784. About a minute a build, most of it the two graded nets over the
/// windows holding the near-pinch at `(θ, t)` (1.076, 0.461). Built again with the observer's
/// origin one ulp off, its trim against the end pose ran on to the rim beside the corner where it
/// had stopped at the cuspidal edge (a rounding's), and fitted only to 1.2e-5: such a run's end is
/// the corner, its tail the straight join.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the crossed reverse built three times, ~4 min")]
fn the_crossed_reverse_member_builds_by_the_tube_road() {
    let (faces,v) = crossed_held(-1.,13.789847278416222);
    assert_eq!(faces,15);
    assert!((v-13.7898471917).abs() <= 1e-9,"the volume {v}, pinned 13.7898471917");
}
