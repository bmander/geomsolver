//! **Two cylinders whose axes lean apart, resting on one another at a point** (#994 item 5,
//! fixture 122: a blanket round over a clevis's top refused with "a cylinder and a cylinder meet
//! at …, where no edge crosses, in a curve that could not be traced"). Two rods whose axes stand
//! the sum of their radii apart touch at one point and meet in no curve; traced, the search for a
//! meeting no edge crosses found a point beside the touch, and the trace from it gave nothing, or
//! the march was refused as nearly touching. `ssi::rods_apart` reads them in closed form where
//! they lean a degree or more apart (less, they touch along a stretch, read as before): every
//! Boolean of two such rods, at any lean, whole or ending at the touch, is the rods' own volumes
//! to rounding, and two sunk a tenth into one another still cross.
use gcs_core::brep::boolean::{boolean,Op};
use gcs_core::brep::build::{prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,Rigid,Surface,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use std::f64::consts::PI;

/// The Booleans' tolerance in the tests of surfaces meeting (`crossed_cylinders`, `resting_touches`).
pub(crate) const TOL: f64 = 1e-9;

/// A rod of radius `r` about the z axis over `z`, carried by `m`.
fn rod(r: f64,z: [f64;2],m: &Rigid) -> Brep {
    let xy: V = [0.,0.,1.];
    let circle = ProfileEdge::Arc {frame:Frame::new([0.;3],xy,[1.,0.,0.]),r,span:None};
    prism(&Profile {names:vec![],origin:[0.;3],normal:xy,loops:vec![vec![circle]]},z[0],z[1]).unwrap().moved(m)
}

/// The rod of radius `r` along x over `along`, and the rod of radius `s` over `across` along the
/// direction turned `lean` from x about z, lifted `apart` above it: they rest on one another at
/// the origin's top, `(0, 0, r)`, where `apart` is `r + s`.
fn rods(r: f64,s: f64,along: [f64;2],across: [f64;2],lean: f64,apart: f64) -> (Brep,Brep) {
    let to_x = Rigid::turn([0.;3],[0.,1.,0.],PI/2.);
    let to_y = Rigid::turn([0.;3],[1.,0.,0.],-PI/2.);
    let leaned = Rigid::turn([0.;3],[0.,0.,1.],(lean-90.).to_radians());
    let lifted = Rigid {t:[0.,0.,apart],..Rigid::identity()};
    (rod(r,along,&to_x),rod(s,across,&to_y).moved(&leaned).moved(&lifted))
}

fn side(b: &Brep) -> Surface {
    b.faces.iter().find(|f| f.surface.kind() == "cylinder").unwrap().surface.clone()
}

/// The pairs of radii, from a rod on one of twice its size to a round of a millimetre on an R20
/// blend, and the leans between their axes.
const RADII: [(f64,f64);4] = [(1.,2.),(1.,20.),(0.5,20.),(3.,3.)];
const LEANS: [f64;5] = [90.,60.,30.,10.,2.];

#[test]
fn rods_resting_on_one_another_or_apart_meet_in_no_curve() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    for (r,s) in RADII {
        for lean in LEANS {
            // resting, resting within the tolerance either way, and clear of one another
            for apart in [r+s,r+s+0.5*TOL,r+s-0.5*TOL,r+s+1e-3,r+s+5.] {
                let (a,b) = rods(r,s,[-3.,3.],[-3.,3.],lean,apart);
                for (x,y) in [(side(&a),side(&b)),(side(&b),side(&a))] {
                    assert_eq!(intersect(&x,&y,TOL),Ssi::Curves(vec![]),"{r} on {s} at {lean}°, {apart} apart");
                }
            }
            // sunk a hundredth into one another they cross in a closed curve, traced
            let (a,b) = rods(r,s,[-3.,3.],[-3.,3.],lean,r+s-1e-2);
            assert_eq!(intersect(&side(&a),&side(&b),TOL),Ssi::Traced,"{r} into {s} at {lean}°");
        }
    }
    // leaning less than a degree apart they stand within the tolerance over a stretch a thousand
    // times the touch's at right angles, and longer as they lean less: a touch along a curve, read
    // from the meeting's ridges, never as no curve
    for lean in [0.5,1e-3,3e-5] {
        let (a,b) = rods(1.,2.,[-3.,3.],[-3.,3.],lean,3.);
        assert_eq!(intersect(&side(&a),&side(&b),TOL),Ssi::Traced,"{lean}°");
    }
    // one radius, the axes crossing: the two ellipses as before
    let (a,b) = rods(1.,1.,[-3.,3.],[-3.,3.],90.,0.);
    assert!(matches!(intersect(&side(&a),&side(&b),TOL),Ssi::Curves(c) if c.len() == 2));
}

/// **Where two rods resting on one another lean too little to rest at a point** (`ssi::apart`):
/// not at a degree, `SHALLOW`, as a rule of their own had it, but where the touch is no longer
/// alone over its radius `2 SHALLOW (r + s) / sin² θ` — the rods part only linearly past
/// `(r + s) / sin θ`, slower than its second order says from `2√2 (r + s) / sin θ` on — so at
/// `sin θ = SHALLOW / √2`, 0.71°, whatever the radii: at 0.8° no curve, at 0.6° left to the trace.
#[test]
fn rods_rest_at_a_point_from_a_lean_of_shallow_over_root_two() {
    use gcs_core::brep::ssi::{intersect,Ssi};
    for (r,s) in RADII {
        for (lean,alone) in [(0.8,true),(0.6,false)] {
            let (a,b) = rods(r,s,[-3.,3.],[-3.,3.],lean,r+s);
            for (x,y) in [(side(&a),side(&b)),(side(&b),side(&a))] {
                let met = intersect(&x,&y,TOL);
                assert_eq!(met == Ssi::Curves(vec![]),alone,"{r} on {s} at {lean}°: {met:?}");
            }
        }
    }
}

#[test]
fn rods_resting_on_one_another_are_their_own_volumes_under_every_boolean() {
    for (r,s) in RADII {
        for lean in LEANS {
            // whole across one another, one ending at the touch, both ending there, and the one
            // under ending short of the other's far side
            for (along,across) in [([-3.,3.],[-3.,3.]),([-3.,3.],[0.,3.]),([0.,3.],[0.,3.]),([-2.,0.],[-2.,2.])] {
                let (a,b) = rods(r,s,along,across,lean,r+s);
                let (va,vb) = (volume(&a),volume(&b));
                let what = format!("{r} on {s} at {lean}°, {along:?} {across:?}");
                assert!((va-PI*r*r*(along[1]-along[0])).abs() <= 1e-12*va,"{what}");
                for (x,y,vx,vy) in [(&a,&b,va,vb),(&b,&a,vb,va)] {
                    for (op,want) in [(Op::Union,vx+vy),(Op::Cut,vx),(Op::Common,0.)] {
                        let out = boolean(x,y,op,TOL).unwrap_or_else(|e| panic!("{what}, {op:?}: {e}"));
                        out.check(1e-8).unwrap_or_else(|e| panic!("{what}, {op:?}: {e}"));
                        let v = volume(&out);
                        assert!((v-want).abs() <= 1e-10*(vx+vy),"{what}, {op:?}: {v} against {want}");
                    }
                }
            }
        }
    }
}

#[test]
fn rods_sunk_a_little_into_one_another_still_cross() {
    // a tenth into one another: what they share stands no higher than that over the
    // parallelogram where their widths cross, and the union and the cut are the rods less it
    for (r,s) in RADII {
        for lean in [90f64,30.] {
            let (a,b) = rods(r,s,[-3.,3.],[-3.,3.],lean,r+s-0.1);
            let what = format!("{r} into {s} at {lean}°");
            let (va,vb) = (volume(&a),volume(&b));
            let common = volume(&boolean(&a,&b,Op::Common,TOL).unwrap_or_else(|e| panic!("{what}: {e}")));
            assert!(common > 0. && common < 0.1*4.*r*s/f64::to_radians(lean).sin(),"{what}: {common}");
            let union = volume(&boolean(&a,&b,Op::Union,TOL).unwrap_or_else(|e| panic!("{what}: {e}")));
            assert!((union-(va+vb-common)).abs() <= 1e-9*(va+vb),"{what}: {union}");
            let cut = volume(&boolean(&a,&b,Op::Cut,TOL).unwrap_or_else(|e| panic!("{what}: {e}")));
            assert!((cut-(va-common)).abs() <= 1e-9*(va+vb),"{what}: {cut}");
        }
    }
}

/// The same as a document: a rail across the front plane resting on a beam across the side plane,
/// unioned — on `main` refused with the issue's words — is their two volumes, whole or with the
/// rail ending at the touch.
#[test]
fn a_rail_resting_across_a_beam_is_the_two_rods() {
    for (r,s,from) in [(1.,20.,-3.),(1.,2.,-3.),(0.5,20.,-3.),(1.,20.,0.)] {
        let e = crate::common::text::solved(&format!("\
unit mm
use std
use std.solids (union)
in std.front {{
  c1 := point
  fix((0, 0)) c1
  pin := circle(center: c1) hint(r: {r})
  radius({r}) pin
}}
in std.side {{
  c2 := point
  fix((0, {apart})) c2
  bar := circle(center: c2) hint(r: {s})
  radius({s}) bar
}}
rail := solid(std.solids.Prism(face(pin), from: {from}mm, to: 3mm))
beam := solid(std.solids.Prism(face(bar), from: -3mm, to: 3mm))
rail union beam
",apart = r+s));
        let what = format!("{r} on {s} from {from}");
        let x = e.sketch.exact_solid(crate::common::ent(&e,"beam").i()).unwrap_or_else(|m| panic!("{what}: {m}"));
        x.brep.check(1e-8).unwrap_or_else(|m| panic!("{what}: {m}"));
        let want = PI*r*r*(3.-from)+PI*s*s*6.;
        let v = volume(&x.brep);
        assert!((v-want).abs() <= 1e-9*want,"{what}: {v} against {want}");
    }
}
