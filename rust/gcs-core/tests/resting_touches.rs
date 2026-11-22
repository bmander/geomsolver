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
use gcs_core::brep::build::{cone,prism,sphere,torus,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use gcs_core::space::{add,cross,norm,scale};
use std::f64::consts::PI;

const TOL: f64 = 1e-9;

fn unit(a: V) -> V { scale(a,1./norm(a)) }

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

/// **The document**: a hoop resting on a beam — a torus of tube `m` and radius `big` turned from
/// the front plane, its top circle crossing the beam's lowest line, a cylinder of 20 across the
/// side plane — unioned (on `main`: "a cylinder and a torus meet at …, where no edge crosses, in a
/// curve that could not be traced"), is the two volumes; and the clevis of fixture 122's class, an
/// eye on a band's end blended into the band's top by an R20 round, the band's far side an arc, and
/// every edge of the top rounded — the round along the arc a torus resting on the blend where the
/// blend leaves the top — built, exported, and its boundary the field's.
#[test]
fn a_hoop_on_a_beam_and_a_rim_round_on_a_blend_are_built() {
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
    for (r,bow) in [(1.,-3.),(2.,-3.),(1.,-1.),(2.,-1.)] {
        let e = crate::common::text::solved(&clevis(r,bow));
        let what = format!("the clevis bowed {bow} rounded {r}");
        let b = crate::common::rounds::exported(&e,"whole");
        // (about where the arc's round rests on the blend, `x` up to 42: past it, under the eye
        // standing out beyond the bowed side, the field and the kernel part a hair's width apart,
        // a separate matter of the blend's end)
        let read = crate::common::rounds::field_is_the_kernels(&e,"whole",[25.,-60.,-15.],[42.,-30.,15.],0.2,3000);
        assert!(read > 500,"{what}: {read} points read");
        // the round takes from the blended band, and less than the rim's whole square would
        let before = volume(&crate::common::rounds::exported(&e,"part"));
        let taken = before-volume(&b);
        assert!(taken > 0. && taken < (1.-PI/4.)*r*r*400.,"{what}: {taken}");
    }
}

/// The clevis: a band 55.75 long whose far side (`y = −50`) bows in by 3, an eye of 12.5 across it
/// at its end, the eye blended into the band's top by a round of 20, and the top's every edge
/// rounded by `r`.
fn clevis(r: f64,bow: f64) -> String {
    // the far side's arc from (0, −50) to (55.75, −50), its sagitta `bow` out of the band (into it
    // where it is less than nothing)
    let (l,h) = (55.75f64,bow.abs());
    let big = (h*h+l*l/4.)/(2.*h);
    let (y,sense) = if bow < 0. { (-50.-(big-h),"cw") } else { (-50.+(big-h),"ccw") };
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
  e1 := arc(center: k, start: a, end: b, sense: {sense})
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
eye := solid(std.solids.Prism(face(eo), from: 36.3mm, to: 50mm))
eye union plate
part := solid(plate)
blend := part.plate.eye.eo round(r: 20mm) part.plate.far
whole := solid(part)
whole.part.plate.far round(r: {r}mm) whole
",x = l/2.)
}
