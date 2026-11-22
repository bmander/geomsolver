//! **Two surfaces resting on one another at a point, whatever their kinds** (#994 item 5): where
//! two surfaces touch with their relative curvature definite (an elliptic touch: they stand on
//! either side of their common tangent plane about the point), that point is their whole meeting
//! within a reach the tolerance sets, `8 √(2 tol / λ)`, `λ` the least size of the difference of
//! their second fundamental forms. Fixture 122's blanket round over a clevis's top makes such
//! touches: an R20 blend tangent to the top from above, and every round along the top's rim
//! tangent to it from below — a cylinder along a straight edge, a torus along an arc, a ball at a
//! corner — each resting on the blend at a point. `ssi::touch_near` found the point by a step
//! tolerance it could not meet where `λ` is small and `Touches::alone` refused the touch where
//! the radius within which the two stand nearer parallel than a trace follows was not small
//! beside the faces; the search's point beside it was then a curve no trace could follow ("a
//! cylinder and a torus meet at …, where no edge crosses, in a curve that could not be traced",
//! or "nearly touching", or a seam's circle through the point "not proved"). Here every Boolean of
//! such a pair is the operands' own volumes, the same pairs sunk into one another still cross, and
//! a touch whose `Q` is indefinite still starts the curves through it.
use gcs_core::brep::boolean::{boolean,Op};
use gcs_core::brep::build::{block,cone,prism,sphere,torus,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use gcs_core::space::{add,cross,scale,unit};
use std::f64::consts::PI;

use crate::common::rounds::square_run;
use crate::cylinders_touching::TOL;

/// A rod of radius `r` about the line through `c` along `d`, `half` either side of `c`.
fn rod(c: V,d: V,r: f64,half: f64) -> Brep {
    let d = unit(d);
    let x = unit(cross(d,if d[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] }));
    let circle = ProfileEdge::Arc {frame:Frame::new(c,d,x),r,span:None};
    prism(&Profile {names:vec![],origin:c,normal:d,loops:vec![vec![circle]]},-half,half).unwrap()
}

/// A blend-sized body, a point `p` of its side, the normal `n` there out of it, and two tangents.
struct Big { name: &'static str,body: Brep,volume: f64,p: V,n: V,t: [V;2] }

fn bigs() -> Vec<Big> {
    let (c,s) = (1f64.cos(),1f64.sin());
    let n = unit([c,s,0.5]);
    let t1 = [-s,c,0.];
    vec![
        // a cylinder of 20 along y, resting points along its lowest line
        Big {name:"an R20 cylinder",body:rod([0.,0.,20.],[0.,1.,0.],20.,20.),volume:PI*400.*40.,p:[0.;3],n:[0.,0.,-1.],
            t:[[1.,0.,0.],[0.,1.,0.]]},
        // a torus of tube 20 about x, at its outer equator's lowest point
        Big {name:"an R20 torus",body:torus([0.,0.,60.],[1.,0.,0.],40.,20.).unwrap(),volume:2.*PI*PI*40.*400.,p:[0.;3],
            n:[0.,0.,-1.],t:[[1.,0.,0.],[0.,1.,0.]]},
        // a frustum's side, half way up, a radian round
        Big {name:"a frustum",body:cone([0.;3],[0.,0.,1.],40.,30.,20.).unwrap(),volume:PI*20./3.*(1600.+1200.+900.),
            p:[35.*c,35.*s,10.],n,t:[t1,cross(n,t1)]},
    ]
}

/// What rests on `big` at its point, sunk `sink` into it: rods of 1 across its tangent and at 60°
/// to it, a ring (a torus of tube 1 about the normal, its tube's top circle through the point,
/// as a round along an arc is) and a ball of 1, each with its volume.
fn smalls(big: &Big,sink: f64) -> Vec<(&'static str,Brep,f64)> {
    let along = |deg: f64| { let a = deg.to_radians(); add(scale(big.t[0],a.cos()),scale(big.t[1],a.sin())) };
    let at = |d: f64| add(big.p,scale(big.n,d-sink));
    vec![
        ("a rod across",rod(at(1.),along(0.),1.,3.),PI*6.),
        ("a rod at 60°",rod(at(1.),along(60.),1.,3.),PI*6.),
        ("a ring",torus(add(at(1.),scale(along(60.),8.)),big.n,8.,1.).unwrap(),2.*PI*PI*8.),
        ("a ball",sphere(at(1.),unit(add(big.t[0],scale(big.t[1],0.3))),1.).unwrap(),4./3.*PI),
    ]
}

/// The three Booleans of `a` and `b`, either way round, each checked: union, cut and common.
fn booleans(a: &Brep,b: &Brep,what: &str) -> [[f64;3];2] {
    [(a,b),(b,a)].map(|(x,y)| [Op::Union,Op::Cut,Op::Common].map(|op| {
        let out = boolean(x,y,op,TOL).unwrap_or_else(|e| panic!("{what}, {op:?}: {e}"));
        out.check(1e-8).unwrap_or_else(|e| panic!("{what}, {op:?}: {e}"));
        volume(&out)
    }))
}

#[test]
fn surfaces_resting_on_one_another_are_their_own_volumes() {
    for big in bigs() {
        for (name,small,v) in smalls(&big,0.) {
            let what = format!("{name} on {}",big.name);
            let [ab,ba] = booleans(&small,&big.body,&what);
            let bar = 1e-9*(v+big.volume);
            for ([u,c,k],[vx,_]) in [(ab,[v,big.volume]),(ba,[big.volume,v])] {
                assert!((u-(v+big.volume)).abs() <= bar,"{what}: union {u}");
                assert!((c-vx).abs() <= bar,"{what}: cut {c} against {vx}");
                assert!(k.abs() <= bar,"{what}: common {k}");
            }
        }
    }
}

#[test]
fn surfaces_sunk_into_one_another_still_cross() {
    // a tenth in: what they share is no part of either alone, and the union and the cut are the
    // two less it
    for big in bigs() {
        for (name,small,v) in smalls(&big,0.1) {
            let what = format!("{name} sunk into {}",big.name);
            let [[u,c,k],_] = booleans(&small,&big.body,&what);
            let bar = 1e-8*(v+big.volume);
            assert!(k > 1e-4 && k < v,"{what}: common {k}");
            assert!((u+k-(v+big.volume)).abs() <= bar,"{what}: union {u} and common {k}");
            assert!((c+k-v).abs() <= bar,"{what}: cut {c} and common {k}");
        }
    }
}

#[test]
fn a_rod_resting_on_a_saddle_crosses_through_its_touch() {
    // in a torus's hole, a rod along y resting on its inner equator: there the torus is a saddle,
    // `Q` indefinite, and the two meet in curves crossing at the touch, which the trace starts
    // from along each branch; the rod's ends reach into the tube. Sunk a little, the same
    for sink in [0.,0.01,0.1] {
        let ring = torus([0.;3],[0.,0.,1.],20.,5.).unwrap();
        let bar = rod([14.+sink,0.,0.],[0.,1.,0.],1.,3.);
        let what = format!("a rod on a saddle, sunk {sink}");
        let [[u,c,k],_] = booleans(&bar,&ring,&what);
        let (va,vb) = (volume(&bar),volume(&ring));
        assert!(k > 0.1,"{what}: common {k}");
        assert!((u+k-(va+vb)).abs() <= 1e-9*(va+vb),"{what}: union {u} and common {k}");
        assert!((c+k-va).abs() <= 1e-9*(va+vb),"{what}: cut {c} and common {k}");
    }
}

#[test]
fn a_ball_clear_of_a_cylinder_or_a_torus_meets_it_in_no_curve() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    let side = |b: &Brep,kind: &str| b.faces.iter().find(|f| f.surface.kind() == kind).unwrap().surface.clone();
    let ball = |c: V,r: f64| side(&sphere(c,unit([0.3,0.2,1.]),r).unwrap(),"sphere");
    let tube = side(&rod([0.;3],[0.,0.,1.],5.,10.),"cylinder");
    let ring = side(&torus([0.;3],[0.,0.,1.],20.,5.).unwrap(),"torus");
    // resting outside the tube, inside it, on the ring's top, inside its tube; and clear of each
    for (what,s,c,r,resting) in [("outside a tube",&tube,[7.,1.,2.],50f64.sqrt()-5.,true),
        ("inside a tube",&tube,[2.,1.,2.],5.-5f64.sqrt(),true),("on a ring",&ring,[20.,0.,7.],2.,true),
        ("in a ring's tube",&ring,[0.,21.,1.],5.-2f64.sqrt(),true),("clear of a tube",&tube,[9.,0.,0.],3.,false),
        ("clear of a ring",&ring,[3.,0.,3.],10.,false)] {
        for (x,y) in [(ball(c,r),s.clone()),(s.clone(),ball(c,r))] {
            assert_eq!(intersect(&x,&y,TOL),Ssi::Curves(vec![]),"a ball {what}");
        }
        // a hundredth larger, one resting crosses
        if resting { assert_eq!(intersect(&ball(c,r+1e-2),s,TOL),Ssi::Traced,"a ball across {what}") }
    }
    // centred on the tube's axis, it meets it in circles, read about the axis as before
    assert!(matches!(intersect(&ball([0.,0.,1.],6.),&tube,TOL),Ssi::Curves(c) if c.len() == 2));
}

/// A ring of radius 8 and tube 1, its axis leaning `lean` degrees from z toward x, standing `sink`
/// into the plane `z = 0` from above (`up`) or below: at no sink it rests on the plane at the one
/// point under the lowest of its core circle, `(8 cos lean, 0, 0)`, where its centre is
/// `8 sin lean + 1` from the plane.
fn ring_on_plane(lean: f64,sink: f64,up: bool) -> Brep {
    let (s,c) = lean.to_radians().sin_cos();
    let side = if up { 1. } else { -1. };
    torus([0.,0.,side*(8.*s+1.-sink)],[s,0.,side*c],8.,1.).unwrap()
}

#[test]
fn a_ring_resting_aslant_on_a_plane_meets_it_in_no_curve() {
    // the least distance of a plane and a torus in closed form (`ssi::apart`): resting, resting
    // within the tolerance either way and clear meet in no curve, where before the pair was traced;
    // sunk a hundredth they cross; and all but level, the two stand within the tolerance round
    // an arc of the ring, a touch along a curve, never no curve
    use gcs_core::brep::ssi::{intersect,Ssi};
    let side = |b: &Brep,kind: &str| b.faces.iter().find(|f| f.surface.kind() == kind).unwrap().surface.clone();
    // the plate's top, the plane z = 0
    let top = block([-12.,-12.,-2.],[12.,12.,0.]).unwrap().faces.iter().map(|f| f.surface.clone())
        .find(|s| s.kind() == "plane" && s.implicit([3.,4.,0.]).abs() <= 1e-12).unwrap();
    for lean in [15.,30.,60.] {
        for up in [true,false] {
            for sink in [0.,0.5*TOL,-0.5*TOL,-1e-3,-1.] {
                let ring = side(&ring_on_plane(lean,sink,up),"torus");
                for (x,y) in [(top.clone(),ring.clone()),(ring.clone(),top.clone())] {
                    assert_eq!(intersect(&x,&y,TOL),Ssi::Curves(vec![]),"a ring at {lean}°, {sink}, {up}");
                }
            }
            let ring = side(&ring_on_plane(lean,1e-2,up),"torus");
            assert_eq!(intersect(&top,&ring,TOL),Ssi::Traced,"a ring at {lean}° sunk in");
        }
    }
    let ring = side(&ring_on_plane(1e-6,0.,true),"torus");
    assert_ne!(intersect(&top,&ring,TOL),Ssi::Curves(vec![]),"a ring all but level on the plane");
}

#[test]
fn a_ring_resting_aslant_on_a_plate_is_its_own_volume_under_every_boolean() {
    let plate = block([-12.,-12.,-2.],[12.,12.,0.]).unwrap();
    let (vp,vr) = (24.*24.*2.,2.*PI*PI*8.);
    for lean in [15.,30.,60.] {
        let ring = ring_on_plane(lean,0.,true);
        let what = format!("a ring at {lean}° on a plate");
        let [ab,ba] = booleans(&ring,&plate,&what);
        let bar = 1e-9*(vp+vr);
        for ([u,c,k],vx) in [(ab,vr),(ba,vp)] {
            assert!((u-(vp+vr)).abs() <= bar,"{what}: union {u}");
            assert!((c-vx).abs() <= bar,"{what}: cut {c} against {vx}");
            assert!(k.abs() <= bar,"{what}: common {k}");
        }
    }
}

#[test]
fn a_ball_a_hair_off_a_tubes_axis_is_never_read_as_meeting_it_in_no_curve() {
    // a ball inside a tube of 5, its centre `e` off the axis and its radius `5 − e`: it touches the
    // tube at one point, and stands within `2 e` of it all round the circle there — within the
    // tolerance round the whole circle at `e = 2 tol`, a touch along a curve the closed form of
    // their distance does not read as a point (before: no curve, the centre being off the axis by
    // more than the tolerance); a millimetre off, the touch is a point, and the meeting none
    use gcs_core::brep::ssi::{intersect,Ssi};
    let side = |b: &Brep,kind: &str| b.faces.iter().find(|f| f.surface.kind() == kind).unwrap().surface.clone();
    let tube = side(&rod([0.;3],[0.,0.,1.],5.,10.),"cylinder");
    for (e,point) in [(2.*TOL,false),(10.*TOL,false),(1.,true)] {
        let ball = side(&sphere([e,0.,0.],[0.,0.,1.],5.-e).unwrap(),"sphere");
        for (x,y) in [(ball.clone(),tube.clone()),(tube.clone(),ball.clone())] {
            let met = intersect(&x,&y,TOL);
            assert_eq!(met == Ssi::Curves(vec![]),point,"a ball {e} off the axis: {met:?}");
        }
    }
}

/// **The document**: a hoop resting on a beam — a torus of tube `m` and radius `big` turned from
/// the front plane, its top circle crossing the beam's lowest line, a cylinder of 20 across the
/// side plane — unioned (on `main`: "a cylinder and a torus meet at …, where no edge crosses, in a
/// curve that could not be traced"), is the two volumes.
#[test]
fn a_hoop_on_a_beam_is_built() {
    for (big,m) in [(3.,0.5),(8.,1.),(2.,0.25)] {
        let e = crate::common::text::solved(&format!("\
unit mm
use std
use std.solids (union)
in std.front {{
  a0 := point
  fix((0, -2)) a0
  a1 := point
  fix((0, 2)) a1
  ax := line(a0, a1)
  c1 := point
  fix(({big}, 0)) c1
  tube := circle(center: c1) hint(r: {m})
  radius({m}) tube
}}
in std.side {{
  c2 := point
  fix((0, {lift})) c2
  bar := circle(center: c2) hint(r: 20)
  radius(20) bar
}}
hoop := solid(std.solids.Revolution(face(tube), about: ax))
beam := solid(std.solids.Prism(face(bar), from: -5mm, to: 5mm))
hoop union beam
",lift = 20.+m));
        let what = format!("a hoop of {big} and {m} on a beam");
        let x = e.sketch.exact_solid(crate::common::ent(&e,"beam").i()).unwrap_or_else(|r| panic!("{what}: {r}"));
        x.brep.check(1e-8).unwrap_or_else(|r| panic!("{what}: {r}"));
        let want = 2.*PI*PI*big*m*m+PI*400.*10.;
        let v = volume(&x.brep);
        assert!((v-want).abs() <= 1e-9*want,"{what}: {v} against {want}");
    }
}

/// **The clevis of fixture 122's class**: an eye on a band's end blended into the band's top by an
/// R20 round, the band's far side an arc bowed in under the eye, which stands out past it, and every
/// edge of the top rounded `r` — the round along the arc a torus resting on the blend where the
/// blend leaves the top — built, exported, and its boundary the field's over the whole body (#994:
/// the floor's chain goes on into the blend and the blend into the eye, and parts about the side
/// that ends the blend; the corner it leaves there no ball reaches, and the round of the band's end
/// 45 mm off read it as its own, which the construction does not build — `std.rounds`' `rolled`).
fn rim_round_on_a_blend(bow: f64) {
    for r in [1.,2.] {
        let e = crate::common::text::solved(&clevis(r,bow));
        let what = format!("the clevis bowed {bow} rounded {r}");
        let b = crate::common::rounds::exported(&e,"whole");
        let read = crate::common::rounds::field_is_the_kernels(&e,"whole",[-5.,-60.,-15.],[70.,-30.,15.],
                                                               0.2,2000);
        assert!(read > 1000,"{what}: {read} points read");
        // the round takes from the blended band, and less than the rim's whole square would
        let before = volume(&crate::common::rounds::exported(&e,"part"));
        let taken = before-volume(&b);
        assert!(taken > 0. && taken < square_run(r)*400.,"{what}: {taken}");
    }
}

#[test]
fn a_rim_round_on_a_blend_bowed_in_by_1_is_the_fields() { rim_round_on_a_blend(-1.) }

#[test]
fn a_rim_round_on_a_blend_bowed_in_by_3_is_the_fields() { rim_round_on_a_blend(-3.) }

#[test]
fn a_rim_round_on_a_blend_bowed_in_by_6_is_the_fields() { rim_round_on_a_blend(-6.) }

/// Bowed out by 3 and rounded 1, the same: field and kernel over the whole body (rounded 2 it is
/// refused, the far side's run disagreeing with the field: `docs/contract/rounds.md`).
#[test]
fn a_rim_round_on_a_blend_bowed_out_by_3_rounded_1_is_the_fields() {
    let e = crate::common::text::solved(&clevis(1.,3.));
    let read = crate::common::rounds::field_is_the_kernels(&e,"whole",[-5.,-60.,-15.],[70.,-30.,15.],
                                                           0.2,2000);
    assert!(read > 1000,"the clevis bowed out 3 rounded 1: {read} points read");
}

/// **The configuration alone** (#994): a straight band, the eye standing 2 out past its far side,
/// so the blend at the band's floor is ended by that side and the eye goes on past it — the floor's
/// chain parts about the side — and the floor's every edge rounded: field and kernel over the whole
/// body, and the round takes what the floor's rim's square would less the eye's stretch at most.
#[test]
fn a_rim_round_beside_a_blend_an_eye_stands_out_past_is_the_fields() {
    for r in [1.,2.] {
        let e = crate::common::text::solved(&clevis_to(r,0.,52.));
        let what = format!("the straight band rounded {r}");
        let b = crate::common::rounds::exported(&e,"whole");
        let read = crate::common::rounds::field_is_the_kernels(&e,"whole",[-5.,-60.,-15.],[70.,-30.,15.],
                                                               0.2,2000);
        assert!(read > 1000,"{what}: {read} points read");
        let taken = volume(&crate::common::rounds::exported(&e,"part"))-volume(&b);
        assert!(taken > 0. && taken < square_run(r)*2.*(55.75+13.7),"{what}: {taken}");
    }
}

/// The clevis: a band 55.75 long whose far side (`y = −50`) bows in by 3, an eye of 12.5 across it
/// at its end, the eye blended into the band's top by a round of 20, and the top's every edge
/// rounded by `r`.
fn clevis(r: f64,bow: f64) -> String { clevis_to(r,bow,50.) }

/// The same, the eye reaching to `y = −reach`, and the far side straight where `bow` is nothing.
fn clevis_to(r: f64,bow: f64,reach: f64) -> String {
    // the far side's arc from (0, −50) to (55.75, −50), its sagitta `bow` out of the band (into it
    // where it is less than nothing)
    let (l,h) = (55.75f64,bow.abs());
    let big = (h*h+l*l/4.)/(2.*h);
    let (y,sense) = if bow < 0. { (-50.-(big-h),"cw") } else { (-50.+(big-h),"ccw") };
    let side = if bow == 0. { "line(a, b)".to_string() }
               else { format!("arc(center: k, start: a, end: b, sense: {sense})") };
    format!("\
unit mm
use std
use std.solids (union)
use std.rounds (round)
in std.top {{
  a := point
  fix((0, -50)) a
  b := point
  fix((55.75, -50)) b
  c := point
  fix((55.75, -36.3)) c
  d := point
  fix((0, -36.3)) d
  k := point
  fix(({x}, {y:.9})) k
  e1 := {side}
  e2 := line(b, c)
  e3 := line(c, d)
  e4 := line(d, a)
  band := face(e1, e2, e3, e4)
}}
in std.front {{
  ec := point
  fix((55.75, 0)) ec
  eo := circle(center: ec) hint(r: 12.5)
  radius(12.5) eo
}}
plate := solid(std.solids.Prism(band, from: -5mm, to: 5mm))
eye := solid(std.solids.Prism(face(eo), from: 36.3mm, to: {reach}mm))
eye union plate
part := solid(plate)
blend := part.plate.eye.eo round(r: 20mm) part.plate.far
whole := solid(part)
whole.part.plate.far round(r: {r}mm) whole
",x = l/2.,y = if bow == 0. { -60. } else { y })
}
