//! The Wankel engine (`rust/examples/wankel`, issue #65): a rotor and a housing from one planetary
//! motion and three numbers. The bore is the apex's envelope (its path), a flank the bore's
//! envelope under the housing's motion, the rotor its blank less the housing turned about it over a
//! whole period — admitted to the planar generating class, which reads the bore's inner envelope,
//! and built exactly — each held to its closed form.
use crate::planar_envelope::{off_flank,rotor_area};
use gcs_core::envelope::planar::InnerEnvelope;
use gcs_core::{brep,solid::admission};
use std::f64::consts::PI;

const R: f64 = 105.;
const E: f64 = 15.;
const W: f64 = 80.;

fn standard() -> gcs_core::program::Elaborated { fixtures::wankel::standard() }

fn say() -> brep::sweep::Say<'static> { brep::sweep::Say {stage:&|_: &str| {},mark:&|_| {}} }

#[test]
fn the_engines_bore_and_flanks_are_their_closed_forms() {
    let e = standard();
    let sk = &e.sketch;
    let bore = e.map.ent_named("bore").unwrap().i();
    for k in 0..=36 {
        let roll = k as f64*30.;
        let t = (roll/3.).to_radians();
        let (x,y) = sk.curve_point(bore,roll);
        assert!((x-E*(3.*t).cos()-R*t.cos()).abs() < 1e-9 && (y-E*(3.*t).sin()-R*t.sin()).abs() < 1e-9,"{roll}");
    }
    // the page's flank: where the bore touches the rotor as the housing turns about it, from near
    // one apex to near the next (a stationary root, the apex, is no profile and is passed by)
    let flank = e.map.ent_named("flank").unwrap().i();
    let poly = sk.curve_polyline(flank);
    assert!(poly.len() > 100);
    for p in &poly { assert!(off_flank(R,E,[p.0-E,p.1]).abs() < 1e-6,"{p:?}"); }
    let reach = poly.iter().map(|p| (p.0-poly[0].0).hypot(p.1-poly[0].1)).fold(0.,f64::max);
    assert!(reach > 0.9*R*3f64.sqrt(),"the flank runs most of the way between apexes: {reach}");
}

#[test]
fn the_housing_is_its_case_less_the_bore() {
    let e = standard();
    let housing = fixtures::solid(&e,"housing");
    let built = brep::export::exact(&e.sketch,housing,None,None,&say()).unwrap();
    built.solid.check(1e-9).unwrap();
    let v = brep::props::volume(&built.solid);
    let want = PI*((R+4.*E).powi(2)-(R*R+3.*E*E))*(W+2.);
    // the bore fitted within `FIT_MM` along its length, over the case's height
    assert!((v-want).abs() <= 2.*PI*R*1.1*(W+2.)*gcs_core::solid::cad::FIT_MM,"{v} against {want}");
}

#[test]
fn the_rotor_is_admitted_to_the_planar_class_and_built_as_its_closed_form() {
    let e = standard();
    let rotor = fixtures::solid(&e,"rotor");
    let admitted = admission::admit_body(&e.sketch,rotor,&admission::Options::default()).unwrap();
    let [sweep] = admitted.sweeps() else { panic!("one sweep") };
    let admission::Class::Planar(found) = &sweep.class else { panic!("{:?}",sweep.class) };
    assert_eq!(found.envelope.pieces.len(),3);
    let built = brep::export::exact(&e.sketch,rotor,Some(&admitted),None,&say()).unwrap();
    built.solid.check(1e-6).unwrap();
    assert_eq!(built.solid.faces.len(),5);
    let v = brep::props::volume(&built.solid);
    let want = rotor_area(R,E)*W;
    assert!((v-want).abs() < 1e-6*want,"{v} against {want}");
}

/// A sweep outside the planar class is refused by the row it fails, never built.
#[test]
fn each_planar_row_names_what_it_refuses() {
    let source = fixtures::wankel::source();
    let refused = |text: &str,numbers: &[(&str,&str)]| -> admission::Refusal {
        let e = fixtures::wankel::read_text(text,numbers);
        match admission::admit_body(&e.sketch,fixtures::solid(&e,"rotor"),&admission::Options::default()) {
            Err(admission::Error::Refused(r)) => r,
            other => panic!("admitted or unread: {}",other.map(|_| String::new()).unwrap_or_else(|e| format!("{e:?}"))),
        }
    };
    let edit = |from: &str,to: &str| { assert!(source.contains(from),"{from}"); source.replace(from,to) };
    // P2: a case that does not stand through the blank
    let r = refused(&edit("solid(face(rim), from: -1mm,","solid(face(rim), from: 10mm,"),&[]);
    assert_eq!(r.condition,admission::Condition::Prisms,"{r}");
    // P3: two turns of the shaft are no period
    let r = refused(&edit("under: housing_turn, from: 0deg, to: 1080deg)\nrotor","under: housing_turn, from: 0deg, to: 720deg)\nrotor"),&[]);
    assert_eq!(r.condition,admission::Condition::Period,"{r}");
    // P5: a case too small, its wall reaching the blank as it turns
    let r = refused(&edit("radius(R + 4 * e) rim","radius(R + e) rim"),&[]);
    assert_eq!(r.condition,admission::Condition::Wall,"{r}");
    assert!(r.witness.is_some());
    assert!(r.to_string().contains("planar generating class"),"{r}");
}

/// Below the limiting K = R/e = 3 the bore crosses itself: the housing's pocket is no face, so no
/// sweep of it is read and no rotor built, and the bore's inner envelope says why.
#[test]
fn below_the_limiting_k_no_rotor_is_read() {
    let e = fixtures::wankel::read(&[("eccentricity","45mm")]);
    let rotor = fixtures::solid(&e,"rotor");
    match admission::admit_body(&e.sketch,rotor,&admission::Options::default()) {
        Err(admission::Error::Unreadable(m)) => assert!(m.contains("`chamber`: self-intersecting"),"{m}"),
        other => panic!("{}",other.map(|_| "admitted".to_string()).unwrap_or_else(|e| format!("{e:?}"))),
    }
    assert!(brep::export::exact(&e.sketch,rotor,None,None,&say()).is_err());
    let (bore,turn) = (e.map.ent_named("bore").unwrap().i(),e.map.ent_named("housing_turn").unwrap().i());
    let refused = InnerEnvelope::read(&e.sketch,bore,turn,[0.,3.*std::f64::consts::TAU]).map(|_| ()).unwrap_err();
    assert!(refused.to_string().contains("no simple loop"),"{refused}");
}

/// The chamber displacement read off the solved parts: chamber 0 at roll θ is bounded by the bore
/// from apex 0, at the bore's own parameter θ, to apex 1 at θ + 360° (the apex is where the bore
/// is the path of), and by the flank between those apexes carried by `rotor_turn`. Its area by
/// the shoelace over both, its largest less its least over the cycle times the width, is
/// 3√3·e·R·W.
#[test]
fn the_chamber_displacement_is_three_root_three_e_r_w() {
    let e = standard();
    let sk = &e.sketch;
    let index = |n: &str| e.map.ent_named(n).unwrap().i();
    let bore = index("bore");
    let family = gcs_core::motion::Family::read(sk,index("rotor_turn")).unwrap();
    let view = sk.curve_view(bore);
    // the bore at a twentieth of a degree of the roll over a period and the turn past it that a
    // chamber reaches, and anywhere between (its parameter is the roll in degrees)
    let step = 0.05_f64.to_radians();
    let at = |t: f64| { let (x,y) = sk.curve_point(bore,t.to_degrees()); [x,y] };
    let grid: Vec<[f64;2]> = (0..=28800).map(|k| at(k as f64*step)).collect();
    // the flank from apex 0 (on the line of centres) to apex 1, in the rotor's own frame: a piece
    // of the bore's inner envelope under the housing's motion
    let env = InnerEnvelope::read(sk,bore,index("housing_turn"),[0.,3.*std::f64::consts::TAU]).unwrap();
    let apex0 = [E+R,0.];
    let piece = env.pieces.iter().position(|p| (p[0].at[0]-apex0[0]).hypot(p[0].at[1]-apex0[1]) < 1e-6).unwrap();
    let flank: Vec<[f64;3]> = (0..=4000).map(|k| env.point(piece,k as f64/4000.).unwrap())
        .map(|p| sk.world_in(view,(p[0],p[1]))).collect();
    let area = |theta: f64| -> f64 {
        let mut ring: Vec<[f64;2]> = vec![at(theta)];
        let (first,last) = ((theta/step).floor() as usize+1,((theta+std::f64::consts::TAU)/step).ceil() as usize-1);
        ring.extend(grid[first..=last].iter().copied());
        ring.push(at(theta+std::f64::consts::TAU));
        let pose = family.pose_at(theta).unwrap();
        ring.extend(flank.iter().rev().map(|&w| { let p = sk.on_view_sheet(pose.point(w),view); [p.0,p.1] }));
        let n = ring.len();
        (0..n).map(|i| ring[i][0]*ring[(i+1)%n][1]-ring[(i+1)%n][0]*ring[i][1]).sum::<f64>().abs()/2.
    };
    // the extremes on a grid of the roll, then refined by golden section
    let coarse: Vec<(f64,f64)> = (0..540).map(|k| { let t = (k as f64*2.).to_radians(); (t,area(t)) }).collect();
    let refine = |t0: f64,sign: f64| -> f64 {
        let (mut lo,mut hi) = (t0-2f64.to_radians(),t0+2f64.to_radians());
        let g = 0.618_033_988_749_894_9;
        let f = |t: f64| sign*area(t);
        let (mut x1,mut x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
        let (mut f1,mut f2) = (f(x1),f(x2));
        for _ in 0..60 {
            if f1 > f2 { hi = x2; x2 = x1; f2 = f1; x1 = hi-g*(hi-lo); f1 = f(x1); }
            else { lo = x1; x1 = x2; f1 = f2; x2 = lo+g*(hi-lo); f2 = f(x2); }
        }
        sign*f(0.5*(lo+hi))
    };
    let most = coarse.iter().max_by(|a,b| a.1.total_cmp(&b.1)).unwrap().0;
    let least = coarse.iter().min_by(|a,b| a.1.total_cmp(&b.1)).unwrap().0;
    let displacement = (refine(most,1.)-refine(least,-1.))*W;
    let want = 3.*3f64.sqrt()*E*R*W;
    assert!((displacement-want).abs() < 1e-5*want,"{displacement} against {want}");
}

/// The rotor to make, three arcs a clearance inside the flanks, clears the housing by half the
/// clearance at every pose of a whole turn: the claim over `rotor_turn` reads the arc rotor placed
/// under it at each sampled roll, solving nothing again, and reports the worst pose.
#[test]
fn the_arc_rotor_clears_the_housing_over_a_whole_turn() {
    let e = standard();
    let verdicts = gcs_core::diagnose::judge_solids(&e.sketch);
    let [v] = verdicts.as_slice() else { panic!("one claim") };
    assert_eq!(v.holds(),Some(true),"{}",v.text());
    assert_eq!(v.samples(),gcs_core::diagnose::SWEEP_STEPS+1);
    assert_eq!(v.failed_samples().len(),0);
    let sweep = v.sweep().unwrap();
    assert!(sweep.motion().is_some() && sweep.to() == 1080.);
    // half a millimetre in at the apexes and the crowns; the worst somewhere between, and more than
    // the quarter claimed
    let measured = v.measured().unwrap();
    assert!(measured > 0.25 && measured < 0.5,"{measured}");
    assert!(v.worst().is_some());
}

/// What the page previews before the exact rotor is built: the rotor's material field, the blank
/// less the housing swept over the period, reads inside the closed-form flank and outside it a
/// twentieth of a millimetre either side (the field is the sweep's, read by its roll search).
#[test]
fn the_rotors_material_field_has_the_flank_for_its_boundary() {
    use gcs_core::interval::{Interval as I,minimum::Options};
    let e = standard();
    let rotor = fixtures::solid(&e,"rotor");
    let mut query = gcs_core::solid::MaterialField::read(&e.sketch,rotor,1e-10).unwrap().evaluator(256);
    for s in [0.4,1.3] {
        let f = crate::planar_envelope::flank(R,E,s);
        let length = f[0].hypot(f[1]);
        for d in [-0.05,0.05] {
            let p = [E+f[0]*(1.+d/length),f[1]*(1.+d/length),W/2.];
            let b = query.bounds(p.map(|v| I::point(v).unwrap()),Options {value_tolerance:1e-3,max_evaluations:50000}).unwrap();
            let [lo,hi] = b.value.bounds();
            if d < 0. { assert!(hi < 0.,"{p:?}: [{lo}, {hi}]") } else { assert!(lo > 0.,"{p:?}: [{lo}, {hi}]") }
        }
    }
}
