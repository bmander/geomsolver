//! **A whole turn of any tool, by its shadow** (#880 Q1, `swept::shadow`): the tool's shadow in a
//! half-plane through the turn's axis arranged in that half-plane, kept by membership and revolved.
use crate::swept_rings::polygon;
use gcs_core::brep::build::revolve;
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::shadow::turned;
use gcs_core::brep::topo::Brep;
use std::f64::consts::TAU;

type V = [f64;3];

/// brep-bench's `swept/v-wheel-turned-once-round-rod`'s wheel: the triangle (4, 0, 10), (6.5, 0,
/// 10) ± 2.5 tan 30°·a turned about the line through (14, 0, 10) along a = (0, sin 20°, cos 20°).
fn v_wheel() -> Brep {
    let lean = 20f64.to_radians();
    let a = [0.,lean.sin(),lean.cos()];
    let h = 2.5*30f64.to_radians().tan();
    let tri = polygon(&[[4.,0.,10.],[6.5,a[1]*h,10.+a[2]*h],[6.5,-a[1]*h,10.-a[2]*h]]);
    revolve(&tri,[14.,0.,10.],a,TAU).unwrap()
}

/// The V wheel's margin at `x`: at most nought within it (its distance `s` from its own axis
/// between the bore's 7.5 and the rim's 10, its height `h` along it within the flanks' `(10 − s) tan
/// 30°`).
fn v_wheel_margin(x: V) -> f64 {
    let lean = 20f64.to_radians();
    let a = [0.,lean.sin(),lean.cos()];
    let w = [x[0]-14.,x[1],x[2]-10.];
    let h = w[0]*a[0]+w[1]*a[1]+w[2]*a[2];
    let s = (w[0]-h*a[0]).hypot(w[1]-h*a[1]).hypot(w[2]-h*a[2]);
    (7.5-s).max(h.abs()-(10.-s)*30f64.to_radians().tan())
}

/// **The volume a tool sweeps turned a whole turn about `z`, worked apart from the build**: at each
/// height of `[z0, z1]` (Simpson's rule over `n` stretches of `θ`, `z = m − c cos θ`), the radii whose circle about `z` meets
/// the tool (`margin` at most nought somewhere round it: its least over 256 angles, each of the
/// three least then narrowed by golden sections) — found on a grid of radii out to `reach` and their
/// ends by bisection — weighed by `2πρ`: the sweep within the cylinder of radius `reach`.
fn turned_volume_apart(margin: &dyn Fn(V) -> f64,reach: f64,[z0,z1]: [f64;2],n: usize,tol: f64) -> f64 {
    let least = |r: f64,z: f64| -> f64 {
        let f = |t: f64| margin([r*t.cos(),r*t.sin(),z]);
        let m = 256;
        let vals: Vec<f64> = (0..m).map(|k| f(TAU*k as f64/m as f64)).collect();
        let mut order: Vec<usize> = (0..m).filter(|&k| vals[k] <= vals[(k+m-1)%m] && vals[k] <= vals[(k+1)%m]).collect();
        order.sort_by(|&i,&j| vals[i].total_cmp(&vals[j]));
        order.iter().take(3).map(|&k| {
            let (mut a,mut b) = (TAU*(k as f64-1.)/m as f64,TAU*(k as f64+1.)/m as f64);
            let g = 0.5*(5f64.sqrt()-1.);
            for _ in 0..48 {
                let (c,d) = (b-g*(b-a),a+g*(b-a));
                if f(c) < f(d) { b = d } else { a = c }
            }
            f(0.5*(a+b))
        }).fold(f64::INFINITY,f64::min)
    };
    let slice = |z: f64| -> f64 {
        let steps = 160;
        let rs: Vec<f64> = (0..=steps).map(|k| reach*k as f64/steps as f64).collect();
        let ins: Vec<bool> = rs.iter().map(|&r| least(r,z) <= 0.).collect();
        let edge = |mut lo: f64,mut hi: f64,lo_in: bool| {
            for _ in 0..44 { let m = 0.5*(lo+hi); if (least(m,z) <= 0.) == lo_in { lo = m } else { hi = m } }
            0.5*(lo+hi)
        };
        let mut area = 0.;
        let mut from = ins[0].then_some(0.);
        for k in 1..rs.len() {
            if ins[k] == ins[k-1] { continue }
            let at = edge(rs[k-1],rs[k],ins[k-1]);
            if ins[k] { from = Some(at) } else if let Some(f) = from.take() { area += std::f64::consts::PI*(at*at-f*f) }
        }
        // a stretch still open at `reach`: the sweep within the cylinder of that radius
        if let Some(f) = from { area += std::f64::consts::PI*(reach*reach-f*f) }
        area
    };
    // over `z = m − c cos θ`, so a slice's area running to nought as a square root at either end
    // of the tool's heights is smooth in `θ`; adaptive Simpson over `n` stretches of it, halving
    // where the slices kink
    let (m,c) = (0.5*(z0+z1),0.5*(z1-z0));
    let f = |t: f64| { let (s,co) = t.sin_cos(); c*s*slice(m-c*co) };
    let pi = std::f64::consts::PI;
    (0..n).map(|k| crate::round_lowering::simpson(&f,pi*k as f64/n as f64,pi*(k+1) as f64/n as f64,tol/n as f64)).sum::<f64>()
}

/// **A V wheel leaning across the turn, turned whole, is its shadow revolved** (#880 Q1;
/// brep-bench's `swept/v-wheel-turned-once-round-rod`, which the arrangement refused): its flanks'
/// and bore's contacts (cones and a cylinder about an axis leaning 20° across the turn) and its
/// three sharp circles laid in the half-plane, the shadow's boundary kept by membership and turned
/// once (`closed`, which asks `shadow` first). Held to the volume of its shadow worked apart from the
/// build (the wheel's analytic margin) to 1e-6, to membership, to a closed STEP file; and the rod it
/// grooves, cut by the ordinary Boolean, to the same integral within the rod's radius.
#[test]
fn a_leaning_v_wheel_turned_whole_is_its_shadow_revolved() {
    use gcs_core::brep::boolean::{Op,combined};
    use gcs_core::brep::build::{disc,prism};
    use gcs_core::brep::swept::closed::closed;
    use gcs_core::brep::trajectory::{Screw,Trajectory};
    let wheel = v_wheel();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.});
    let built = closed(&wheel,&path,[0.,TAU],0.01,1e-6).unwrap_or_else(|e| panic!("the wheel turned whole: {e:?}"));
    // by its shadow, as the module builds it alone
    let alone = turned(&wheel,[0.;3],[0.,0.,1.],0.01,1e-6).unwrap_or_else(|e| panic!("the wheel's shadow: {e:?}"));
    assert_eq!(volume(&alone),volume(&built),"the closed sweep is the shadow's");
    let (lo,hi) = wheel.bounds();
    let apart = turned_volume_apart(&v_wheel_margin,25.,[lo[2],hi[2]],32,1e-2);
    let v = volume(&built);
    assert!((v-apart).abs() <= 1e-6*apart,"the wheel turned whole: {v} against {apart} worked apart");
    crate::swept_closed::agrees_over(&built,&wheel,&path,[0.,TAU]);
    let text = gcs_core::brep::step::write(&built,"groove",1e-3).unwrap();
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(&built)).unwrap_or_else(|m| panic!("its STEP: {m}"));
    // the rod grooved: the sweep subtracted by the Boolean
    let rod = prism(&disc([0.;3],[0.,0.,1.],5.),0.,20.).unwrap();
    let cut = combined(&rod,&built,Op::Cut,1e-6).unwrap();
    let groove = turned_volume_apart(&v_wheel_margin,5.,[lo[2],hi[2]],32,1e-3);
    let left = 500.*std::f64::consts::PI-groove;
    assert!((volume(&cut)-left).abs() <= 1e-6*left,"the rod grooved: {} against {left}",volume(&cut));
}
