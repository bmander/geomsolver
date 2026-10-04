//! The inner envelope of a closed planar curve under a planar motion (`envelope::planar`, what the
//! planar generating class reads of a pocket): the Wankel rotor's flanks are the inner envelope of
//! the housing's bore seen from the rotor, against the closed form (Yamamoto).
use crate::bore::{solved,wankel};
use gcs_core::envelope::planar::{Condition,InnerEnvelope};

/// The rotor's flank in its own frame, about its centre, apex at angle 0: for s ∈ [0, 2π/3],
/// `e^{is}·(R + 2ie·sin(3s/2)·e^{iα})` with `cos α = −(3e/R)·cos(3s/2)`, `α ∈ (0, π)`; the
/// other two flanks are this one turned by 120° and 240°.
pub fn flank(r: f64,e: f64,s: f64) -> [f64;2] {
    let a = (-(3.*e/r)*(1.5*s).cos()).acos();
    let (re,im) = (r-2.*e*(1.5*s).sin()*a.sin(),2.*e*(1.5*s).sin()*a.cos());
    [re*s.cos()-im*s.sin(),re*s.sin()+im*s.cos()]
}

/// How far `g` (about the rotor's centre) is from the closed-form flank at its polar angle: the
/// flank's angle runs monotonically from 0 to 120° over s, so it is inverted by bisection.
pub fn off_flank(r: f64,e: f64,g: [f64;2]) -> f64 {
    let turn = 2.*std::f64::consts::PI/3.;
    let angle = g[1].atan2(g[0]).rem_euclid(turn);
    let (mut lo,mut hi) = (0.,turn);
    for _ in 0..200 {
        let mid = 0.5*(lo+hi);
        let p = flank(r,e,mid);
        if p[1].atan2(p[0]).rem_euclid(std::f64::consts::TAU) < angle { lo = mid } else { hi = mid }
    }
    let p = flank(r,e,0.5*(lo+hi));
    p[0].hypot(p[1])-g[0].hypot(g[1])
}

fn rotor(r: f64,e: f64) -> Result<(InnerEnvelope,gcs_core::program::Elaborated),gcs_core::envelope::planar::Refusal> {
    let el = solved(&wankel(r,e,"bore := envelope(apex, under: rotor_turn, from: 0deg, to: 1080deg)"));
    let (bore,turn) = (el.map.ent_named("bore").unwrap().i(),el.map.ent_named("housing_turn").unwrap().i());
    InnerEnvelope::read(&el.sketch,bore,turn,[0.,3.*std::f64::consts::TAU]).map(|env| (env,el))
}

#[test]
fn the_rotors_flanks_are_the_inner_envelope_of_the_bore() {
    for (r,e) in [(105.,15.),(100.,25.)] {
        let (env,_) = rotor(r,e).unwrap_or_else(|f| panic!("K = {}: {f}",r/e));
        // three flanks, from apex to apex, counter-clockwise
        assert_eq!(env.pieces.len(),3,"K = {}",r/e);
        let centre = [e,0.];
        let mut corners: Vec<[f64;2]> = env.pieces.iter().map(|p| p[0].at).collect();
        corners.sort_by(|a,b| (a[1]-centre[1]).atan2(a[0]-centre[0]).total_cmp(&(b[1]-centre[1]).atan2(b[0]-centre[0])));
        for (k,c) in corners.iter().enumerate() {
            let a = (k as f64-1.)*2.*std::f64::consts::PI/3.;
            let want = [centre[0]+r*a.cos(),centre[1]+r*a.sin()];
            assert!((c[0]-want[0]).hypot(c[1]-want[1]) < 1e-6,"K = {}: apex {c:?} against {want:?}",r/e);
        }
        let mut n = 0;
        for piece in &env.pieces {
            for c in piece {
                let g = [c.at[0]-centre[0],c.at[1]-centre[1]];
                let off = off_flank(r,e,g);
                assert!(off.abs() < 1e-6,"K = {}: {:?} is {off} off the flank",r/e,c.at);
                n += 1;
            }
            // and between contacts, solved onto the envelope
            for f in [0.1,0.37,0.5,0.81] {
                let p = env.point(env.pieces.iter().position(|q| std::ptr::eq(q,piece)).unwrap(),f).unwrap();
                assert!(off_flank(r,e,[p[0]-centre[0],p[1]-centre[1]]).abs() < 1e-6);
            }
        }
        assert!(n > 300);
    }
}

#[test]
fn below_the_limiting_k_the_envelope_is_refused_and_names_its_row() {
    // K = 2.5: the bore crosses itself and the flanks have gaps; no rotor is read
    let refused = rotor(50.,20.).map(|_| ()).unwrap_err();
    assert!(refused.row == Condition::Envelope,"{refused}");
}

#[test]
fn an_envelope_is_read_over_a_whole_period() {
    let el = solved(&wankel(105.,15.,"bore := envelope(apex, under: rotor_turn, from: 0deg, to: 1080deg)"));
    let (bore,turn) = (el.map.ent_named("bore").unwrap().i(),el.map.ent_named("housing_turn").unwrap().i());
    let refused = InnerEnvelope::read(&el.sketch,bore,turn,[0.,std::f64::consts::TAU]).unwrap_err();
    assert_eq!(refused.row,Condition::Period,"{refused}");
}

/// The rotor's area by Green's theorem over the closed-form flanks: three times one flank's
/// `½∮(x dy − y dx)` about the rotor's centre, by Simpson's rule on many steps.
pub fn rotor_area(r: f64,e: f64) -> f64 {
    let n = 20000;
    let h = 2.*std::f64::consts::PI/3./n as f64;
    let term = |s: f64| {
        let (p,q) = (flank(r,e,s-1e-6),flank(r,e,s+1e-6));
        let c = flank(r,e,s);
        0.5*(c[0]*(q[1]-p[1])-c[1]*(q[0]-p[0]))/2e-6
    };
    let simpson: f64 = (0..=n).map(|k| {
        let w = if k == 0 || k == n { 1. } else if k % 2 == 1 { 4. } else { 2. };
        w*term(k as f64*h)
    }).sum::<f64>()*h/3.;
    3.*simpson
}

/// P1: a motion that turns the curve's plane out of itself (about a line lying in it) gives no
/// envelope in the plane.
#[test]
fn a_motion_off_the_plane_gives_no_envelope() {
    let el = solved(&wankel(105.,15.,"bore := envelope(apex, under: rotor_turn, from: 0deg, to: 1080deg)\n\
        tilt := motion(about: rightward_line)\nrightward_line := line(origin, rightward)"));
    let (bore,tilt) = (el.map.ent_named("bore").unwrap().i(),el.map.ent_named("tilt").unwrap().i());
    let refused = InnerEnvelope::read(&el.sketch,bore,tilt,[0.,std::f64::consts::TAU]).unwrap_err();
    assert_eq!(refused.row,Condition::Plane,"{refused}");
}
