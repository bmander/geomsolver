//! **How deep another instant covers a point of a candidate** (#942, the depth-based trim,
//! `swept::depth`): brep-bench's tumbling tori read by the far cover — a point of a tube's sheet
//! against the instants outside its own contact, a point of the tool at an end pose against the
//! instants after (before) it.

use gcs_core::brep::swept::depth::{Reading,cover,depths,far_depth,label_at,seams,sheet};
use gcs_core::brep::swept::depth::Sampled;
use gcs_core::brep::swept::rings::{Ring,Side,pieces_at};
use std::f64::consts::{FRAC_PI_2,PI};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};

/// brep-bench's tumbling torus turned 0.375 a radian about one line and back a radian about
/// another (`half` the roll each way, in degrees): the half turn's (90) and the full turn's (180),
/// its tool, motion, roll and tolerance.
fn tumbling(half: f64) -> (Brep,Trajectory,[f64;2],f64) {
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,
        advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-half.to_radians(),half.to_radians()],0.16/50_000.)
}

/// The far cover of the half turn's sheet at `(θ, t)`, on the contact nearest the place `σ` round
/// the meridian.
fn far(ring: &Ring,path: &Trajectory,roll: [f64;2],(th,t,sg): (f64,f64,f64),tol: f64) -> Reading {
    let l = label_at(ring,path,th,t,sg).unwrap();
    let (x,bend) = sheet(ring,path,roll,th,t,l).unwrap();
    assert!(bend > 0.,"({th}, {t}): the sheet touching, s'' {bend}");
    cover(ring,path,roll,x,t,0.,tol).unwrap()
}

/// **The far cover agrees with the double curve** (#942): two pairs of the half turn's double
/// curve as `rings::double_curve_from_apex` traces it (its points to seven places, each within a
/// tenth of the tolerance of the curve) — each point covered by its partner's instant, by no more
/// than the tolerance either way: the curve where another instant's cover begins, its partner face
/// and instant named by the cover, not by a meeting of surfaces.
#[test]
fn the_far_cover_of_the_half_turns_sheet_begins_on_its_double_curve() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    for (p,partner) in [((6.2302900,1.5655403,0.4781),1.4621677),((6.4444197,1.4623041,0.4051),1.5674549)] {
        let Reading::Far(c) = far(&ring,&path,roll,p,tol) else { panic!("{p:?}: no far cover") };
        assert!(c.depth.abs() <= tol,"{p:?}: covered {:e} deep",c.depth);
        assert!((c.at-partner).abs() <= 1e-5,"{p:?}: covered at {} for its partner's {partner}",c.at);
    }
}

/// **The swallowtail's tail is covered less than the tolerance near its point** (#942): between
/// the half turn's double curve and its fold, 6.2e-3 of an angle from the swallowtail point, the
/// tail is held by another instant some 5e-9 deep, within the tolerance (3.2e-6) — the sliver the
/// ring road's trim was refused at, 8e-4 from the point, no feature at the tolerance; 7.2e-2 away
/// it is held 8e-5 deep, a feature a trim must cut.
#[test]
fn the_half_turns_tail_is_covered_less_than_the_tolerance_near_its_swallowtail_point() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let Reading::Far(near) = far(&ring,&path,roll,(6.343211,1.486769,0.4441),tol) else { panic!("no far cover near the point") };
    assert!(near.depth < 0. && near.depth > -tol,"near the swallowtail point: covered {:e} deep",near.depth);
    let Reading::Far(away) = far(&ring,&path,roll,(6.276129,1.5362,0.4459),tol) else { panic!("no far cover away from the point") };
    assert!(away.depth < -10.*tol,"away from the swallowtail point: covered {:e} deep",away.depth);
}

/// **Where the full turn's end poses touch, both are buried** (#942): the tool where the roll
/// starts and where it stops touch near (2.256, 2.808, −0.219); there the start pose is held 0.047
/// deep by an instant just before the roll's end, and the end pose by the instants just before it
/// — no trim is needed there. And the start pose's equator near (2.5965, 3.3856, −0.2104), where the
/// ring road's trim left an open edge, is held by the instants after it: no piece of it is kept.
#[test]
fn where_the_full_turns_end_poses_touch_both_are_buried() {
    let (tool,path,roll,tol) = tumbling(180.);
    let ring = Ring::read(&tool).unwrap();
    // the torus's point at the angles `φ` about its axis and `ψ` round its tube, at a pose
    let at = |t: f64,phi: f64,psi: f64| {
        let r = 1.+0.16*psi.cos();
        path.pose(t).unwrap().point([3.+r*phi.cos(),r*phi.sin(),2.+0.16*psi.sin()])
    };
    let start = at(roll[0],2.355154,-1.569185);
    match cover(&ring,&path,roll,start,roll[0],0.,tol).unwrap() {
        Reading::Far(c) => assert!(c.depth < -0.04 && c.at > 3.,"the start pose at the touch: {c:?}"),
        other => panic!("the start pose at the touch: {other:?}"),
    }
    let end = at(roll[1],-2.355278,-1.567330);
    // (the angles to six places: the two points within a few millionths of one another)
    assert!(gcs_core::brep::util::space::distance(start,end) < 1e-5,"the poses touch: {start:?} {end:?}");
    assert!(matches!(cover(&ring,&path,roll,end,roll[1],0.,tol).unwrap(),Reading::Near),"the end pose at the touch");
    let equator = at(roll[0],1.760338,-0.000259);
    assert!(matches!(cover(&ring,&path,roll,equator,roll[0],0.,tol).unwrap(),Reading::Near),"the start pose's equator");
}

/// **The cover's boundary traced from the depth is the double curve** (#942, `depth::seams`): over
/// a box of the half turn's sheet beside the swallowtail point, from the cut where its fold pair is
/// born down to where the end pose covers it, the zero set of the far cover's depth traced from one
/// point of the double curve — one piece, rim to rim, through the double curve's other point there
/// (to the tolerance), its partner's preimages each covered at the instant of its own point: both
/// faces' parameters of the seam come from the depth, with no meeting of surfaces.
#[test]
fn the_cover_s_boundary_traced_from_the_depth_is_the_half_turn_s_double_curve() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let seed = [6.4444197,1.4623041];
    let l = label_at(&ring,&path,seed[0],seed[1],0.4051).unwrap();
    // (the feature a quarter of the tube's thickness, the step `cover` marches by)
    let s = seams(&ring,&path,roll,l,[[6.40,6.447],[1.455,1.476]],&[seed],tol,0.04).unwrap();
    assert_eq!(s.len(),1,"{s:?}");
    let piece = &s[0];
    assert!(piece.sheet.len() >= 2 && piece.partner.len() == 1,"{piece:?}");
    let at = |th: f64,t: f64| sheet(&ring,&path,roll,th,t,l).unwrap().0;
    // (the double curve's other point in the box, `rings::double_curve_from_apex`'s to seven places)
    let other = at(6.4065157,1.4640931);
    let near = piece.sheet.windows(2).flat_map(|w| (0..=64).map(move |k| { let f = k as f64/64.; [w[0].0+(w[1].0-w[0].0)*f,w[0].1+(w[1].1-w[0].1)*f] }))
        .map(|q| gcs_core::brep::util::space::distance(at(q[0],q[1]),other)).fold(f64::INFINITY,f64::min);
    assert!(near <= 2.*tol,"the double curve's point {near:e} from the traced seam");
    for (k,&(th,t,sg)) in piece.partner[0].iter().enumerate() {
        let lp = label_at(&ring,&path,th,t,sg).unwrap();
        let c = far_depth(&ring,&path,roll,th,t,lp,tol).unwrap();
        assert!(c.depth.abs() <= tol,"the partner's point {k}: covered {:e} deep",c.depth);
        assert!((c.at-piece.sheet[k].1).abs() <= 1e-6,"the partner's point {k} covered at {} for its own point's {}",c.at,piece.sheet[k].1);
    }
}

/// **A fold pair born before its swallowtail point bounds nothing the trim keeps** (#942,
/// `rings::fitted_unborn`): the half turn's fold pair is born at 1.4766 and its double curve begins
/// at the swallowtail point 0.0073 of the roll later. The road parts the roll at the point and
/// enters the sheet before it as though the pair were not yet born: one touching piece across the
/// bridge, where between the folds the sheet lies folded back, trimmed only by the tail above the
/// double curve (over the angle round the axis, `Fitted::tails`). Read at eight rolls across that
/// stretch and 64 places across each bridge, the sweep's depth at each (the tool's least signed
/// distance over the roll — the instants beside its own read a hundred-thousandth of the roll apart,
/// the rest at 4096, the least settled by golden sections): every place between the folds (360) is
/// tail, held by a later instant (to 2.3e-4 deep), and every place the tail rule keeps (61) is held
/// no deeper than the tolerance (to the rounding: on the boundary). So the trim drops the whole of
/// the pair's sliver before the point, and keeps only the boundary.
#[test]
fn the_half_turns_fold_pair_bounds_nothing_the_trim_keeps_before_its_swallowtail_point() {
    use gcs_core::brep::swept::rings::{changes,double_curve_from_apex,fitted_over};
    let (tool,path,roll,tol) = tumbling(90.);
    let [a,b] = roll;
    let ring = Ring::read(&tool).unwrap();
    let (scale,cuts) = changes(&ring,&path,roll).unwrap();
    let birth = cuts[0];
    let fit = fitted_over(&ring,&path,roll,[birth+1e-9*(b-a),b],scale,0.1*tol).unwrap();
    let [apex] = fit.apexes[..] else { panic!("one swallowtail point: {:?}",fit.apexes) };
    let curve = double_curve_from_apex(&ring,&path,roll,apex,tol).unwrap();
    let line: Vec<[f64;2]> = curve.iter().take_while(|p| !p.0.is_nan()).map(|p| [p.0,p.1]).collect();
    // (above the curve at the place's angle, brought within half a turn of the curve's middle)
    let above = |th: f64,t: f64| {
        let mid = 0.5*(line[0][0]+line[line.len()-1][0]);
        let th = mid+(th-mid+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
        let i = line.partition_point(|p| p[0] < th);
        if i == 0 || i == line.len() { return false }
        let ([a0,t0],[a1,t1]) = (line[i-1],line[i]);
        t > t0+(t1-t0)*(th-a0)/(a1-a0)
    };
    let held = |x: [f64;3],t: f64| {
        let d = |tau: f64| depths(&ring,&path,x,&[tau])[0];
        let fine = 1e-5*(b-a);
        let ts: Vec<f64> = (-2000..=2000).map(|k| t+fine*k as f64).chain((0..=4096).map(|k| a+(b-a)*k as f64/4096.))
            .filter(|&s| s >= a && s <= b).collect();
        let ds = depths(&ring,&path,x,&ts);
        let k = (0..ts.len()).min_by(|&i,&j| ds[i].total_cmp(&ds[j])).unwrap();
        let step = if k <= 4000 { fine } else { (b-a)/4096. };
        let (mut lo,mut hi) = ((ts[k]-step).max(a),(ts[k]+step).min(b));
        let g = 0.5*(5f64.sqrt()-1.);
        for _ in 0..80 { let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo)); if d(x1) < d(x2) { hi = x2 } else { lo = x1 } }
        ds[k].min(d(0.5*(lo+hi)))
    };
    let (mut kept,mut dropped,mut folded) = ((0,0f64),(0,0f64),0);
    for k in 1..=8 {
        let t = birth+(apex.at.1-birth)*k as f64/8.;
        for c in pieces_at(&ring,&path,[t,b],t).unwrap() {
            for (from,to,l,side) in c {
                if side != Side::Fold { continue }
                for j in 0..=64 {
                    let th = from+(to-from)*j as f64/64.;
                    let Some((x,bend)) = sheet(&ring,&path,roll,th,t,l) else { continue };
                    let depth = held(x,t);
                    if bend < 0. { folded += 1 }
                    assert!(bend >= 0. || above(th,t),"at ({th}, {t}): the fold's far side kept");
                    if above(th,t) {
                        assert!(depth < 0.,"at ({th}, {t}): dropped as tail, held {depth:e} deep");
                        dropped = (dropped.0+1,dropped.1.min(depth));
                    } else {
                        assert!(depth >= -tol,"at ({th}, {t}): kept, held {depth:e} deep");
                        kept = (kept.0+1,kept.1.min(depth));
                    }
                }
            }
        }
    }
    assert!(kept.0 >= 32 && folded >= 8*16,"kept {kept:?}, dropped {dropped:?}, {folded} folded");
}

/// **The thin torus's double curves, joined to their swallowtail points, are where the far cover
/// begins** (#942 T1d-tori, `rings::joined`): brep-bench's `tumbling-torus-thin` (a torus of tube
/// 0.015, its tolerance a 50 000th of that). Each fold pair's double curve is traced from a row's
/// seed and joined along the swallowtail's normal form to its point, where the two leaves stand
/// within the tolerance of one another over a band and the march alone slid onto a fold's inner
/// leaf. The two curves end at their swallowtail points, `(6.159841, 0.8371692)` and `(2.839100,
/// 1.1404747)` (angles modulo a turn), every point on an outer leaf (`s''` above nought), and every
/// point's far cover within the tolerance of nought — a double point of the sweep, its other
/// instant named by the cover. (The point itself stands on the fold, `s''` nought.)
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 30 s: the thin tumbling torus's double curves traced")]
fn the_thin_tori_double_curves_end_at_their_swallowtail_points_and_the_far_cover_begins_on_them() {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.015).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],axis:[1.,0.,0.],
        ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-75f64.to_radians(),75f64.to_radians()];
    let tol = 0.015/50_000.;
    let (ring,_,_,curves,_) = gcs_core::brep::swept::closed::ring_fitted(&tool,&path,roll,tol).unwrap();
    assert_eq!(curves.len(),2,"two fold pairs, two double curves");
    let tau = std::f64::consts::TAU;
    let mut ends = Vec::new();
    for c in &curves {
        let mark = c.iter().position(|p| p.0.is_nan()).unwrap();
        let [ap] = c[mark+1..] else { panic!("one swallowtail point on each curve: {:?}",&c[mark+1..]) };
        assert!(c[..mark].iter().any(|p| p.0 == ap.0 && p.1 == ap.1),"the point {ap:?} on its curve");
        ends.push((ap.0.rem_euclid(tau),ap.1));
        for (k,&(th,t,sg)) in c[..mark].iter().enumerate() {
            // (the point itself, where the two preimages meet, stands on the fold: `s''` nought, and
            // no far cover)
            if th == ap.0 && t == ap.1 { continue }
            let l = label_at(&ring,&path,th,t,sg).unwrap();
            let (_,bend) = sheet(&ring,&path,roll,th,t,l).unwrap();
            assert!(bend > 0.,"point {k} ({th}, {t}): s'' {bend}, not an outer leaf");
            if let Some(cv) = far_depth(&ring,&path,roll,th,t,l,tol) {
                assert!(cv.depth.abs() <= tol,"point {k} ({th}, {t}): its far cover {:e}",cv.depth);
            }
        }
    }
    ends.sort_by(|a,b| a.1.total_cmp(&b.1));
    for (got,want) in ends.iter().zip([(6.159841,0.8371692),(2.839100,1.1404747)]) {
        assert!((got.0-want.0).abs() < 1e-6 && (got.1-want.1).abs() < 1e-7,"a swallowtail point at {got:?}, for {want:?}");
    }
}

/// **A far cover past an own contact that barely climbs is read from its climb's top** (#942
/// T1d-tori, `depth::cover`): beside the thin tumbling torus's second swallowtail point, a point of
/// its fitted sheet in the lens the double curve's dip cuts off below the point's row — the place
/// the trim probed that piece at, its own instant 1.1401698, standing off its sheet by the fit's
/// tenth of the tolerance. Its distance from the tool climbs from its own instant to 3.1e-8, a hair
/// past that, and falls slowly to 2.2e-6 inside the tool at an instant 0.018 of the roll on (seven
/// tolerances): it is in the tail. Read against the sample before rather than the greatest so far,
/// no two neighbours stood the rounding apart and it was read `Alone`, the boundary, and the lens
/// was kept where the tail above the row was dropped.
#[test]
fn a_far_cover_past_an_own_contact_that_barely_climbs_is_read_from_its_climbs_top() {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.015).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],axis:[1.,0.,0.],
        ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-75f64.to_radians(),75f64.to_radians()];
    let tol = 0.015/50_000.;
    let ring = Ring::read(&tool).unwrap();
    let (x,t) = ([1.972716153444042,1.791053032761949,1.3022206590889487],1.1401698);
    // (the climb's top within a few roundings of the fit's tenth of the tolerance it stands off by)
    let ts: Vec<f64> = (0..=40).map(|k| t+0.001*k as f64).collect();
    let ds = depths(&ring,&path,x,&ts);
    let top = ds.iter().take_while(|d| **d > 0.).fold(0f64,|m,&d| m.max(d));
    assert!(top > 0.1*tol && top < 0.2*tol,"the own contact climbs to {top:e}");
    let Reading::Far(c) = cover(&ring,&path,roll,x,t,0.1*tol,tol).unwrap() else { panic!("the lens's point not read covered") };
    assert!(c.depth < -5.*tol && (c.at-1.158).abs() < 2e-3,"held {:e} deep at {}",c.depth,c.at);
}

// **The ring along a spline read by sampling** (#942, the touched entry): its runner's frame
// encloses nothing, so it is read by `Sampled`.

type V = [f64;3];

/// The frame brep-bench's runner carries the ring by: its `x` along the curve's tangent, its `z` up.
#[derive(Debug)]
struct Runner(gcs_core::brep::nurbs::BSpline);
impl gcs_core::brep::trajectory::Path for Runner {
    fn at(&self,t: f64) -> Result<gcs_core::brep::trajectory::Motion,gcs_core::brep::KernelError> {
        let (point,d,dd) = self.0.d2(t);
        let speed2 = d[0]*d[0]+d[1]*d[1];
        let (c,s) = (d[0]/speed2.sqrt(),d[1]/speed2.sqrt());
        let turning = (d[0]*dd[1]-d[1]*dd[0])/speed2;
        let r = [[c,-s,0.],[s,c,0.],[0.,0.,1.]];
        let dr = [[-s*turning,-c*turning,0.],[c*turning,-s*turning,0.],[0.;3]];
        Ok(gcs_core::brep::trajectory::Motion::of_parts(r,dr,point,d))
    }
}

/// The torus of radii 1.5 and 0.5 about `y` through (0, 0, 0.5), read as a ring, and the runner's
/// frame along the bench's cubic.
fn ring_along() -> (Ring,Trajectory) {
    let tool = gcs_core::brep::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = gcs_core::brep::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],
        vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    (Ring::read(&tool).unwrap(),Trajectory::General(std::sync::Arc::new(Runner(curve))))
}

fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn sc(a: V,k: f64) -> V { [a[0]*k,a[1]*k,a[2]*k] }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }

/// The torus's point in its own coordinates at `α` round its axis (−π/2 its bottom meridian, θ*)
/// and `ψ` round its tube (0 away from the axis), and its normal there.
fn local(alpha: f64,psi: f64) -> (V,V) {
    let er = [alpha.cos(),0.,alpha.sin()];
    let n = add(sc(er,psi.cos()),[0.,psi.sin(),0.]);
    (add(add([0.,0.,0.5],sc(er,1.5)),sc(n,0.5)),n)
}

/// The contact parallel's `ψ₀` at the instant `t`, branch `j` half a turn on: `n · v = 0` on the
/// normal circle at `α = 0`.
fn psi0(path: &Trajectory,t: f64,j: usize) -> f64 {
    let m = path.at(t).unwrap();
    let v = m.velocity([1.5,0.,0.5]);
    (-dot(m.vector([1.,0.,0.]),v)).atan2(dot(m.vector([0.,1.,0.]),v))+j as f64*PI
}

/// **Each sheet's kept piece bounds the sweep, and the meridian's strip is kept whole** (#942, the
/// depth trim): a hair inside a point of each kept piece — the tool where the roll starts, behind
/// its bottom meridian and in the wedge before it beside the tube's rim, the strip swept by the
/// bottom meridian, each tube beyond its bridge — the whole roll's depth is that hair below nought,
/// a hair outside it that hair above. The strip's touching half has no basin but its own at all;
/// each tube is held from beyond on the far side of the meridian, by more than its stretch beside
/// θ* is (under 1e-7: the sliver, its fold and the far leaf there stand within that of one another).
#[test]
fn a_rings_kept_pieces_bound_its_sweep_by_depth() {
    let (ring,path) = ring_along();
    let depth = Sampled::new(&ring,&path,[0.,1.],4000).unwrap();
    let place = |a: f64,p: f64,t: f64| { let (x,n) = local(a,p); let pose = path.pose(t).unwrap(); (pose.point(x),pose.vector(n)) };
    let least = |x: V| depth.minima(x).iter().map(|c| c.depth).fold(f64::INFINITY,f64::min);
    let h = 1e-6;
    let mut cases = vec![(-FRAC_PI_2-0.05,psi0(&path,0.,0)+1.,0.),(-FRAC_PI_2+0.1,psi0(&path,0.,0)-0.005,0.)];
    for t in [0.003,0.3,0.7,0.997] {
        cases.extend([(-FRAC_PI_2,psi0(&path,t,0)+1.,t),(-FRAC_PI_2+0.4,psi0(&path,t,0),t),(-FRAC_PI_2-0.4,psi0(&path,t,1),t)]);
    }
    for (a,p,t) in cases {
        let (x,n) = place(a,p,t);
        let (inward,on,outward) = (least(add(x,sc(n,-h))),least(x),least(add(x,sc(n,h))));
        assert!((inward+h).abs() < 1e-9 && on.abs() < 1e-12 && (outward-h).abs() < 1e-9,"α {a} ψ {p} t {t}: {inward} {on} {outward}");
    }
    for t in [0.1,0.5,0.9] {
        let p0 = psi0(&path,t,0);
        for k in 1..12 {
            let (x,_) = place(-FRAC_PI_2,p0+PI*k as f64/12.,t);
            assert!(depth.cover(x,t).is_none(),"the strip at t {t}, ψ − ψ0 {}",PI*k as f64/12.);
        }
        // tube 0 kept in front of θ*, held from behind it; its stretch beside θ* within 1e-7
        let (front,_) = place(-FRAC_PI_2+0.1,p0,t);
        let (behind,_) = place(-FRAC_PI_2-0.1,p0,t);
        assert!(depth.cover(front,t).is_none_or(|c| c.depth >= 0.),"t {t}");
        assert!(depth.cover(behind,t).is_some_and(|c| c.depth < -1e-6),"t {t}");
        for e in [0.001,0.005,0.01] {
            let (x,_) = place(-FRAC_PI_2+e,p0,t);
            assert!(depth.cover(x,t).is_none_or(|c| c.depth > -1e-7),"t {t} e {e}");
        }
    }
}

/// **The wedge the ring where it starts keeps beside its tube's rim is traced to its corner, each
/// point with its partner** (#942, the depth trim): along a path turning from its start, the ring
/// there stands out past the wall the later instants sweep, in a wedge in front of its bottom
/// meridian beside the tube's rim, held from beyond where the tube of an instant just after reaches
/// over it. That seam, traced as a graph over the angle from θ* (`Depth::seam_over`), runs into the
/// corner where the meridian meets the rim — the wedge's width there falls below the tolerance —
/// and each point carries the instant holding it from beyond and the tool's point there, which lies
/// on that instant's contact parallel: both faces' parameters by construction.
#[test]
fn the_start_wedges_seam_is_traced_into_its_corner_with_its_partner() {
    let (ring,path) = ring_along();
    let depth = Sampled::new(&ring,&path,[0.,1.],4000).unwrap();
    let tol = 1e-5;
    let p0 = psi0(&path,0.,0);
    let pose = path.pose(0.).unwrap();
    let sheet = |e: f64,s: f64| (pose.point(local(-FRAC_PI_2+e,p0+s).0),0.);
    let seam = depth.seam_over(&sheet,[0.3,2.5e-4],&|_| [0.,-0.3],tol).unwrap();
    assert!(seam.len() >= 3 && seam.last().unwrap().e == 2.5e-4,"{} points",seam.len());
    for p in &seam {
        assert!(p.cover.depth >= 0. && p.cover.depth <= tol,"{p:?}");
        // its partner: an instant just after, the tool's point there on tube 0's parallel
        let at = p.cover.at;
        let rho = at[0].hypot(at[2]-0.5);
        let psi = at[1].atan2(rho-1.5);
        let d = (psi-psi0(&path,p.cover.t,0)+PI).rem_euclid(2.*PI)-PI;
        assert!(p.cover.t > 0. && p.cover.t < 0.05 && 0.5*d.abs() <= tol,"{p:?}: ψ off by {d}");
    }
    // the wedge narrowing to its corner: at the last point under the tolerance across
    let last = seam.last().unwrap();
    let rim = pose.point(local(-FRAC_PI_2+last.e,p0).0);
    assert!(gcs_core::brep::util::space::distance(rim,last.x) <= 2.*tol,"{last:?}");
}
