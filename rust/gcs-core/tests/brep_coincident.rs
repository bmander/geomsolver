//! **Faces on one surface but for the rounding of what they are drawn through are on one surface**
//! (#961): a pad resting on a plate, two blocks lying flush side by side, two rods of one radius on
//! one axis — each drawn through rounded points, so the faces meant to coincide lean a hair apart
//! (`1e-12` to `1e-8` rad) or stand a few tolerances off — are united and cut as the coincident
//! faces they are within the arrangement's reach, whichever operand comes first and however the
//! lean is turned: one surface wherever any phase asks (`meeting::Shared::index_over`,
//! `ssi::same_over`), their volumes those of the solids drawn and their boundaries closed. A lean
//! past the reach is two surfaces still.
use gcs_core::brep::boolean::{combined_written,Op};
use gcs_core::brep::build::{cone,prism,revolve,sphere,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Frame,Rigid,Surface,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::ssi::{same,same_over};
use gcs_core::brep::topo::Brep;
use std::f64::consts::PI;

use crate::brep::{block,rod};

const XY: V = [0.,0.,1.];

fn close(a: f64,b: f64,within: f64,what: &str) { assert!((a-b).abs() <= within,"{what}: {a} against {b}"); }

/// How far a volume may move when a face of area `area` is read on another surface standing within
/// the arrangement's reach of it (`ON_CURVE` tolerances) over it: the solids' tolerance.
fn moved_by(a: &Brep,b: &Brep,area: f64) -> f64 { gcs_core::brep::tolerant::ON_CURVE*tolerance(a.size().max(b.size()))*area }

/// `a` combined with `b` by `op` as a body's Boolean is (`combined_written`), checked closed to the
/// ten tolerances its Boolean checks it to (an edge laid on a face of the other operand stands off
/// it by as much as the two faces stood apart there).
/// And every face agrees with the edges and corners it is bounded by to within the reach the
/// arrangement merged them at (`ON_CURVE` tolerances): a face two operands' coincident faces
/// became is written on the solid's surface (the lower operand's), and the other's edges and
/// corners laid on it stood no farther off it than the two surfaces stood apart there.
fn built(a: &Brep,b: &Brep,op: Op,what: &str) -> Brep {
    let tol = tolerance(a.size().max(b.size()));
    let out = combined_written(a,None,b,op,0.).unwrap_or_else(|e| panic!("{what}: {e}")).0;
    out.check(10.*tol).unwrap_or_else(|e| panic!("{what}: {e}"));
    let reach = gcs_core::brep::tolerant::ON_CURVE*tol;
    for (fi,f) in out.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() {
            let e = &out.edges[c.edge as usize];
            for k in 0..=8 {
                let p = e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/8.,&out.vertices);
                let off = gcs_core::space::distance(f.surface.point(f.surface.inverse(p)),p);
                assert!(off <= reach,"{what}: face {fi} ({}) stands {off:e} off its edge {} at {k}/8, past {reach:e}",
                    f.surface.kind(),c.edge);
            }
        }
    }
    out
}

/// The leans a face drawn through rounded points takes (`0` exactly, and a hair to a few
/// tolerances over a pad's breadth), and the axes it leans about: across the pad, along it, and
/// askew, each through a corner of the face meant to lie on the other.
const LEANS: [f64;5] = [0.,1e-12,1e-10,1e-9,5e-9];
const ABOUT: [V;3] = [[0.,1.,0.],[1.,0.,0.],[1.,1.,0.]];

/// **A pad resting on a plate, its foot leaning a hair off the plate's top, is united with it**
/// (fixture 115's strips, each pad's face drawn through points rounded to five or nine decimals):
/// the pad's foot and the plate's top are one plane, the union the plate and the pad, its volume
/// theirs, whichever is the solid and whichever the operand; and a pad sunk to the plate's top
/// from below, cut, leaves the plate less the pad.
#[test]
fn a_pad_leaning_a_hair_off_a_plate_is_united_with_it() {
    let plate = block([0.,0.,0.],[100.,100.,1.]);
    for &theta in &LEANS { for &axis in &ABOUT {
        let turn = Rigid::turn([20.,30.,1.],axis,theta);
        let pad = block([20.,30.,1.],[30.,40.,3.]).moved(&turn);
        let what = format!("a pad leaning {theta:e} about {axis:?}");
        for (a,b) in [(&plate,&pad),(&pad,&plate)] {
            let u = built(a,b,Op::Union,&what);
            close(volume(&u),100.*100.+10.*10.*2.,moved_by(a,b,100.),&what);
            assert_eq!(u.faces.len(),11,"{what}: the plate's six faces, its top holed, and the pad's five");
        }
        let sunk = block([20.,30.,-1.],[30.,40.,1.]).moved(&turn);
        let c = built(&plate,&sunk,Op::Cut,&what);
        close(volume(&c),100.*100.-10.*10.,moved_by(&plate,&sunk,100.),&what);
    } }
}

/// **A pad leaning past the reach is two planes still**: leaning `1e-3` rad off the plate's top
/// about an edge of its foot, its foot rises a hundredth of a millimetre over the pad — far past
/// the arrangement's reach — and the union keeps the foot a face of its own, a wedge open under it,
/// the volume the plate's and the pad's.
#[test]
fn a_pad_leaning_past_the_reach_keeps_its_foot() {
    let plate = block([0.,0.,0.],[100.,100.,1.]);
    // (turned so the foot rises off the plate, not into it)
    for (axis,lean) in [([0.,1.,0.],-1e-3),([1.,0.,0.],1e-3)] {
        let pad = block([20.,30.,1.],[30.,40.,3.]).moved(&Rigid::turn([20.,30.,1.],axis,lean));
        let what = format!("a pad leaning 1e-3 about {axis:?}");
        let u = built(&plate,&pad,Op::Union,&what);
        close(volume(&u),100.*100.+10.*10.*2.,1e-9,&what);
        assert_eq!(u.faces.len(),12,"{what}: the plate's six and the pad's six, its foot among them");
    }
}

/// **Two blocks lying flush side by side, partly overlapping, their tops and feet leaning a hair
/// apart, are united**: the faces meant on one plane overlap over part of each, and the union is
/// the two less what they share, its faces those of the two drawn exactly flush.
#[test]
fn flush_blocks_overlapping_in_part_are_united() {
    let a = block([0.,0.,0.],[10.,10.,1.]);
    let flush = built(&a,&block([5.,2.,0.],[15.,8.,1.]),Op::Union,"exactly flush").faces.len();
    for &theta in &LEANS { for &axis in &ABOUT {
        let b = block([5.,2.,0.],[15.,8.,1.]).moved(&Rigid::turn([10.,5.,0.5],axis,theta));
        let what = format!("a block leaning {theta:e} about {axis:?}");
        for (x,y) in [(&a,&b),(&b,&a)] {
            let u = built(x,y,Op::Union,&what);
            close(volume(&u),100.+60.-30.,moved_by(x,y,2.*30.),&what);
            assert_eq!(u.faces.len(),flush,"{what}: the faces of the two drawn exactly flush");
        }
    } }
}

/// **Faces a union made one surface are one surface to what reads its result** (#994,
/// `tolerant::one_surface`): the flush blocks' tops, and their feet, each kept on its own
/// operand's plane leaning a hair off the other's, are one surface the same way out to a ball's
/// sides and the field (`fillet::one_surface`), as the union read them.
#[test]
fn faces_a_union_made_one_surface_are_one_surface_after_it() {
    use gcs_core::brep::fillet::one_surface;
    let a = block([0.,0.,0.],[10.,10.,1.]);
    for &theta in &LEANS { for &axis in &ABOUT {
        let b = block([5.,2.,0.],[15.,8.,1.]).moved(&Rigid::turn([10.,5.,0.5],axis,theta));
        let what = format!("a block leaning {theta:e} about {axis:?}");
        for (x,y) in [(&a,&b),(&b,&a)] {
            let u = built(x,y,Op::Union,&what);
            let tol = tolerance(u.size());
            for (level,up) in [(1.,true),(0.,false)] {
                let on: Vec<usize> = (0..u.faces.len()).filter(|&f| match u.faces[f].surface {
                    Surface::Plane(fr) => (fr.z[2].abs()-1.).abs() <= 1e-6 && (fr.o[2]-level).abs() <= 1e-6
                        && ((fr.z[2] > 0.) != u.faces[f].reversed) == up,
                    _ => false,
                }).collect();
                assert!(on.len() >= 2,"{what}: {on:?}");
                for &f in &on { for &g in &on {
                    assert!(one_surface(&u,f,g,tol),"{what}: faces {f} and {g} at {level}");
                } }
            }
        }
    } }
}

/// **Two rods of one radius on one axis, one leaning a hair, are one rod where they overlap**: their
/// walls coincide over the length both cover, and the union is the rod from the lower's foot to the
/// upper's top.
#[test]
fn coaxial_rods_of_one_radius_leaning_a_hair_are_united() {
    for &r in &[2.,5.5] {
        let low = rod([0.,0.,0.],r,[0.,10.]);
        for &theta in &LEANS[..4] { for &axis in &ABOUT {
            let high = rod([0.,0.,0.],r,[5.,15.]).moved(&Rigid::turn([0.,0.,7.5],axis,theta));
            let what = format!("a rod of radius {r} leaning {theta:e} about {axis:?}");
            for (x,y) in [(&low,&high),(&high,&low)] {
                let u = built(x,y,Op::Union,&what);
                close(volume(&u),PI*r*r*15.,moved_by(x,y,2.*PI*r*5.),&what);
            }
        } }
    }
}

/// **Two frustums of one cone, one leaning a hair, are one cone where they overlap** (a
/// countersink's cone flush with a chamfer's, drawn from rounded coordinates): the frustum from
/// radius 5 to 3 over ten millimetres and the one from 4 to 2 over the next ten, from halfway up
/// the first, united are the frustum from 5 to 2 over fifteen, `π h (R² + R r + r²) / 3`.
#[test]
fn frustums_of_one_cone_leaning_a_hair_are_united() {
    let low = cone([0.;3],XY,5.,3.,10.).unwrap();
    assert!(low.faces.iter().any(|f| f.surface.kind() == "cone"),"a frustum's side is a cone");
    for &theta in &LEANS[..4] { for &axis in &ABOUT {
        let high = cone([0.,0.,5.],XY,4.,2.,10.).unwrap().moved(&Rigid::turn([0.,0.,7.5],axis,theta));
        let what = format!("a frustum leaning {theta:e} about {axis:?}");
        for (x,y) in [(&low,&high),(&high,&low)] {
            let u = built(x,y,Op::Union,&what);
            close(volume(&u),PI*15./3.*(25.+10.+4.),moved_by(x,y,2.*PI*4.5*5.),&what);
        }
    } }
}

/// The sector of the torus of middle radius `big` and tube `small` about the z axis through the
/// origin, from `from` round `turn` radians.
fn torus_sector(big: f64,small: f64,from: f64,turn: f64) -> Brep {
    let f = Frame::new([0.;3],XY,[1.,0.,0.]);
    let centre = [big,0.,0.];
    let section = ProfileEdge::Arc {frame:Frame::new(centre,f.y,f.x),r:small,span:None};
    let p = Profile {origin:centre,normal:f.y,loops:vec![vec![section]],names:Vec::new()};
    revolve(&p,[0.;3],XY,turn).unwrap().moved(&Rigid::turn([0.;3],XY,from))
}

/// **Two sectors of one torus, one leaning a hair, are one torus where they overlap**: the sectors
/// from 0 to 200° and from 160° to 360°, the second leaning about the torus's centre, united are
/// the whole ring, `2 π² R r²`, either first.
#[test]
fn sectors_of_one_torus_leaning_a_hair_are_united() {
    let (big,small) = (10.,3.);
    let one = torus_sector(big,small,0.,200f64.to_radians());
    assert!(one.faces.iter().any(|f| f.surface.kind() == "torus"),"a sector's side is a torus");
    for &theta in &LEANS[..4] { for &axis in &ABOUT {
        let other = torus_sector(big,small,160f64.to_radians(),200f64.to_radians()).moved(&Rigid::turn([0.;3],axis,theta));
        let what = format!("a sector leaning {theta:e} about {axis:?}");
        for (x,y) in [(&one,&other),(&other,&one)] {
            let u = built(x,y,Op::Union,&what);
            close(volume(&u),2.*PI*PI*big*small*small,moved_by(x,y,4.*PI*PI*big*small*40./360.),&what);
        }
    } }
}

/// **A ball and the same ball drawn a few tolerances off, seamed about another axis, are one
/// ball**: a sphere has no axis, so `same` reads two by centre and radius alone, at the reach the
/// arrangement reads every surface by; united either way they are the ball, `4 π r³ / 3`.
#[test]
fn a_ball_drawn_a_few_tolerances_off_itself_is_one_ball() {
    let r = 6.;
    let ball = sphere([0.;3],XY,r).unwrap();
    let tol = tolerance(2.*r);
    for off in [0.,tol,3.*tol] {
        let other = sphere([off,0.5*off,0.],[0.,1.,0.],r).unwrap();
        let what = format!("a ball {off:e} off");
        for (x,y) in [(&ball,&other),(&other,&ball)] {
            let u = built(x,y,Op::Union,&what);
            close(volume(&u),4./3.*PI*r*r*r,moved_by(x,y,4.*PI*r*r),&what);
        }
    }
}

/// **One surface over a region, and not past it** (`ssi::same_over`): two planes leaning `θ` apart
/// through a common line are one over a box within `tol / θ` of the line, and two elsewhere; two
/// cylinders of one radius whose axes lean apart likewise; and with no region, `same`'s reading
/// (parallel to `1e-12`) is kept.
#[test]
fn surfaces_leaning_apart_are_one_over_a_region_within_the_tolerance() {
    let tol = tolerance(100.);
    let flat = Surface::Plane(Frame::new([0.;3],XY,[1.,0.,0.]));
    let theta: f64 = 1e-9;
    let leaning = Surface::Plane(Frame::new([0.;3],[0.,theta.sin(),theta.cos()],[1.,0.,0.]));
    let near: ([f64;3],[f64;3]) = ([-10.,-10.,-1.],[10.,10.,1.]);
    let far: ([f64;3],[f64;3]) = ([-10.,400.,-1.],[10.,420.,1.]);
    assert!(!same(&flat,&leaning,tol),"read over all of space, two planes");
    assert!(same_over(&flat,&leaning,tol,near),"within tol / θ of where they cross, one");
    assert!(!same_over(&flat,&leaning,tol,far),"four hundred millimetres off, θ times that apart: two");
    let lifted = Surface::Plane(Frame::new([0.,0.,2.*tol],XY,[1.,0.,0.]));
    assert!(!same_over(&flat,&lifted,tol,near),"parallel and twice the tolerance apart: two");
    let axis = |lean: f64| Surface::Cylinder(Frame::new([0.;3],[lean.sin(),0.,lean.cos()],[0.,1.,0.]),3.);
    assert!(same_over(&axis(0.),&axis(theta),tol,([-3.,-3.,-10.],[3.,3.,10.])),"two rods' walls, one over their length");
    assert!(!same_over(&axis(0.),&axis(theta),tol,([-3.,-3.,990.],[3.,3.,1010.])),"a metre up the axis, two");
    assert!(!same_over(&axis(0.),&axis(1e-6),tol,([-3.,-3.,-10.],[3.,3.,10.])),"a lean a thousand times that: two");
    // a cone and a torus likewise, about the apex and the centre
    let cone_at = |lean: f64| Surface::Cone(Frame::new([0.;3],[lean.sin(),0.,lean.cos()],[0.,1.,0.]),5.,-0.2);
    let near_apex = ([-6.,-6.,0.],[6.,6.,10.]);
    assert!(same_over(&cone_at(0.),&cone_at(theta),tol,near_apex),"a cone leaning 1e-9, one over its frustum");
    assert!(!same_over(&cone_at(0.),&cone_at(1e-6),tol,near_apex),"leaning 1e-6, two");
    let ring = |lean: f64| Surface::Torus(Frame::new([0.;3],[lean.sin(),0.,lean.cos()],[0.,1.,0.]),10.,3.);
    let round = ([-13.,-13.,-3.],[13.,13.,3.]);
    assert!(same_over(&ring(0.),&ring(theta),tol,round),"a torus leaning 1e-9, one over its ring");
    assert!(!same_over(&ring(0.),&ring(1e-6),tol,round),"leaning 1e-6, two");
    let fat = Surface::Torus(Frame::new([0.;3],XY,[1.,0.,0.]),10.,3.+2.*tol);
    assert!(!same_over(&ring(0.),&fat,tol,round),"a tube twice the tolerance fatter: two");
}

/// A strip bent from a sheet of thickness 1 (fixture 115's feet and lip, #994): drawn in the plane
/// through `o` along `u` and `v` (`v` up the sheet's thickness), a pad from `c` to `c + 1` along
/// `u` (its end at `c + 1`), bent at `c` through `turn` round an inner radius of 3 and an outer of
/// 4 centred at `(c, 4)`, then a leg `leg` long; swept `from` `to` along `u × v`.
fn strip(o: V,u: V,v: V,c: f64,turn: f64,leg: f64,from: f64,to: f64) -> Brep {
    use gcs_core::space::cross;
    let n = cross(u,v);
    let at = |a: f64,b: f64| -> V { std::array::from_fn(|k| o[k]+u[k]*a+v[k]*b) };
    let start = 1.5*PI-turn;
    let bend = |r: f64| (c+r*start.cos(),4.+r*start.sin());
    let ((i0,i1),(o0,o1)) = (bend(3.),bend(4.));
    let (du,dv) = (start.sin(),-start.cos());
    let line = |a: (f64,f64),b: (f64,f64)| ProfileEdge::Line {a:at(a.0,a.1),b:at(b.0,b.1)};
    let arc = |r: f64| ProfileEdge::Arc {frame:Frame::new(at(c,4.),n,u),r,span:Some([start,1.5*PI])};
    let edges = vec![line((c+1.,1.),(c,1.)),arc(3.),line((i0,i1),(i0+leg*du,i1+leg*dv)),
        line((i0+leg*du,i1+leg*dv),(o0+leg*du,o1+leg*dv)),line((o0+leg*du,o1+leg*dv),(o0,o1)),arc(4.),
        line((c,0.),(c+1.,0.)),line((c+1.,0.),(c+1.,1.))];
    prism(&Profile {names:vec![],origin:at(c,0.),normal:n,loops:vec![edges]},from,to).unwrap()
}

/// The section of `strip`: its pad, its bend and its leg.
fn section(turn: f64,leg: f64) -> f64 { 1.+3.5*turn+leg }

/// **A plate, a lip along its edge and a foot at its corner united in every order are one solid**
/// (#994, fixture 115 at its true edge): the plate and two bent strips whose pads lie in the
/// plate's thickness — their tops and feet on its top and foot, overlapping it and each other — or
/// abut its edges, each strip drawn leaning a hair (up to `7e-7` rad across its pad, `2e-9` along
/// it, as a bend solved through points rounded to a few decimals leans it), within the
/// arrangement's reach of the plate over the pad. United in each of the six orders, and with the
/// two strips united first, they are one solid: closed, every edge on its faces (`built`), its
/// volume the closed form of the three less what they share, and its faces as many whatever the
/// order. Read one surface at a time, the faces on the plate's top and foot took whichever came
/// first for their surface (`Arrangement::classed`), and the edges two such faces' corners were
/// minted on stood apart by more than the pool's reach (`tolerant::at_end`) and crossed by more
/// than the tolerance (`query::crossings_in`): "a plane's pieces do not close", "a piece … touches
/// … along no face", a shell kept and dropped — in some orders, and in others the solid, its faces
/// one more or less. Each strip is turned about its own length by `leans` — the lip about its
/// pad's middle, the foot about a line `pivot` up its thickness — and the lip by `along` about
/// the plate's width, and the volume held to the closed form of the strips so turned
/// (`Strips::united`, which reads the turns across the pads) within the reach over the pads'
/// tops and feet and what the turn along the lip moves its pad by.
fn strips_on_a_plate(overlapping: bool,pivot: f64,leans: &[(f64,f64)],along: f64) {
    let s = three_strips(overlapping,pivot);
    let (ux,uy) = ([1.,0.,0.],[0.,1.,0.]);
    for &(lean_lip,lean_foot) in leans {
        let (lip_turn,foot_turn) = (Rigid::turn(s.lip_at,ux,lean_lip).then(&Rigid::turn(s.lip_at,uy,along)),Rigid::turn(s.foot_at,uy,lean_foot));
        let (lip,foot) = (s.lip.moved(&lip_turn),s.foot.moved(&foot_turn));
        let what = format!("the lip leaning {lean_lip:e}, the foot {lean_foot:e}");
        let three = [("plate",&s.plate),("lip",&lip),("foot",&foot)];
        // (the volume as far as the faces read on one surface stand apart over the pads, and the
        // same in every order to rounding)
        let exact = s.united(&lip_turn,&foot_turn);
        let within = moved_by(&s.plate,&lip,4.*s.pads)+along*s.reach*s.pads;
        let (mut faces,mut first): (Option<usize>,Option<f64>) = (None,None);
        for order in [[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]] {
            let [a,b,c] = order.map(|k| three[k]);
            for nested in [false,true] {
                let what = if nested { format!("{what}: {} ∪ ({} ∪ {})",a.0,b.0,c.0) } else { format!("{what}: {} ∪ {} ∪ {}",a.0,b.0,c.0) };
                let u = if nested { built(a.1,&built(b.1,c.1,Op::Union,&what),Op::Union,&what) }
                    else { built(&built(a.1,b.1,Op::Union,&what),c.1,Op::Union,&what) };
                close(volume(&u),exact,within,&what);
                close(volume(&u),*first.get_or_insert(volume(&u)),1e-6,&format!("{what}: the same volume in every order"));
                assert_eq!(u.faces.len(),*faces.get_or_insert(u.faces.len()),"{what}: as many faces in every order");
            }
        }
    }
}

/// The plate and the two strips of `strips_on_a_plate`, drawn exactly, and what the volume of the
/// three united is read from.
struct Strips {
    plate: Brep,lip: Brep,foot: Brep,
    /// where each strip leans about, on its pad's middle
    lip_at: V,foot_at: V,
    /// the plate's box; the lip's pad's corners (it runs along x over `lip_x`), the foot's (along y
    /// over `foot_y`); each strip's volume
    box_: (V,V),lip_pad: [V;4],lip_x: [f64;2],foot_pad: [V;4],foot_y: [f64;2],strips: f64,
    /// the plate's area, the pads' and how far a pad reaches from where it leans about
    area: f64,pads: f64,reach: f64,
}

fn three_strips(overlapping: bool,pivot: f64) -> Strips {
    let t: V = [-49.8088,-48.6,-2.67665];
    let w = |p: V| -> V { std::array::from_fn(|k| p[k]+t[k]) };
    let (x0,y0,x1,y1) = (19.6/13.,36.6/13.,1249.2/13.,1227./13.);
    let plate = block(w([x0,y0,0.]),w([x1,y1,1.]));
    let (up,ux,uy) = ([0.,0.,1.],[1.,0.,0.],[0.,1.,0.]);
    // each strip, where it leans about, the corner its pad starts at and the stretch it runs
    // over, and its volume
    let (lip,lip_at,foot,foot_at,(c,lip_x),(d,foot_y),strips) = if overlapping {
        // the lip's pad over the plate's edge along x, the foot's over its edge along y, and the
        // two over one another at the corner, every pad in the plate's thickness
        let (c,d) = (34./13.,17./13.);
        (strip(w([x0,0.,0.]),uy,up,c,0.5*PI,11.98462,0.,x1-x0),w([0.5*(x0+x1),c+0.5,0.5]),
            strip(w([0.,2.7,0.]),ux,up,d,0.25*PI,13.79264,-(31.8-2.7),0.),w([d+0.5,17.,pivot]),(c,[x0,x1]),(d,[2.7,31.8]),
            section(0.5*PI,11.98462)*(x1-x0)+section(0.25*PI,13.79264)*29.1)
    } else {
        // the lip's pad abutting the plate's edge along x and running on past its corner, the
        // foot's abutting its edge along y, its side in the plane of the lip's pad's end
        (strip(w([x0-1.,0.,0.]),uy,up,y0-1.,0.5*PI,12.,0.,x1-x0+1.),w([0.5*(x0+x1),y0-0.5,0.5]),
            strip(w([0.,y0,0.]),ux,up,x0-1.,0.25*PI,14.,-30.,0.),w([x0-0.5,y0+15.,pivot]),(y0-1.,[x0-1.,x1]),(x0-1.,[y0,y0+30.]),
            section(0.5*PI,12.)*(x1-x0+1.)+section(0.25*PI,14.)*30.)
    };
    let square = |a: f64,corner: &dyn Fn(f64,f64) -> V| [corner(a,0.),corner(a+1.,0.),corner(a+1.,1.),corner(a,1.)];
    Strips {plate,lip,foot,lip_at,foot_at,box_:(w([x0,y0,0.]),w([x1,y1,1.])),
        lip_pad:square(c,&|a,b| w([lip_x[0],a,b])),lip_x:lip_x.map(|x| x+t[0]),
        foot_pad:square(d,&|a,b| w([a,foot_y[0],b])),foot_y:foot_y.map(|y| y+t[1]),strips,
        area:(x1-x0)*(y1-y0),pads:(lip_x[1]-lip_x[0])+(foot_y[1]-foot_y[0]),reach:x1-x0}
}

impl Strips {
    /// **The volume of the plate and the strips turned by `lip` and `foot` united, in closed
    /// form**: the three's volumes less what each two share and with what the three share, each
    /// strip meeting the others by its pad alone — a section the turn carries (a unit square
    /// turned about its middle) along its length — so each share is an integral over height of the
    /// lengths the pads' sections and the plate's box hold at it, piecewise linear in height, read
    /// exactly by Simpson's rule between every height a length bends at. A turn about a strip's
    /// own length is all it is read for.
    fn united(&self,lip: &Rigid,foot: &Rigid) -> f64 {
        let (lo,hi) = self.box_;
        // each pad's section in its plane: the lip's in (y, z), the foot's in (x, z)
        let lq: Vec<[f64;2]> = self.lip_pad.iter().map(|&p| { let q = lip.point(p); [q[1],q[2]] }).collect();
        let fq: Vec<[f64;2]> = self.foot_pad.iter().map(|&p| { let q = foot.point(p); [q[0],q[2]] }).collect();
        // the stretch across a section at height z, within `within`
        let across = |q: &[[f64;2]],z: f64,within: [f64;2]| -> f64 {
            let (mut a,mut b) = (f64::INFINITY,f64::NEG_INFINITY);
            for i in 0..q.len() {
                let (p,r) = (q[i],q[(i+1)%q.len()]);
                if (p[1]-z)*(r[1]-z) > 0. || p[1] == r[1] { continue }
                let x = p[0]+(r[0]-p[0])*(z-p[1])/(r[1]-p[1]);
                a = a.min(x); b = b.max(x);
            }
            (b.min(within[1])-a.max(within[0])).max(0.)
        };
        let meet = |a: [f64;2],b: [f64;2]| [a[0].max(b[0]),a[1].min(b[1])];
        let length = |a: [f64;2]| (a[1]-a[0]).max(0.);
        let (xp,yp,zp) = ([lo[0],hi[0]],[lo[1],hi[1]],[lo[2],hi[2]]);
        // every height a length bends at: a corner, the plate's top and foot, an edge crossing a
        // side it is held within
        let mut at: Vec<f64> = lq.iter().chain(&fq).map(|p| p[1]).chain(zp).collect();
        for (q,sides) in [(&lq,[yp[0],yp[1],self.foot_y[0],self.foot_y[1]]),(&fq,[xp[0],xp[1],self.lip_x[0],self.lip_x[1]])] {
            for i in 0..4 {
                let (p,r) = (q[i],q[(i+1)%4]);
                for s in sides { if (p[0]-s)*(r[0]-s) < 0. { at.push(p[1]+(r[1]-p[1])*(s-p[0])/(r[0]-p[0])) } }
            }
        }
        at.sort_by(f64::total_cmp);
        let integral = |f: &dyn Fn(f64) -> f64,over: [f64;2]| -> f64 {
            let mut cuts: Vec<f64> = at.iter().copied().filter(|&z| z > over[0] && z < over[1]).collect();
            cuts.insert(0,over[0]); cuts.push(over[1]);
            cuts.windows(2).map(|w| (w[1]-w[0])/6.*(f(w[0])+4.*f(0.5*(w[0]+w[1]))+f(w[1]))).sum()
        };
        let pads = [lq.iter().chain(&fq).map(|p| p[1]).fold(f64::INFINITY,f64::min),
            lq.iter().chain(&fq).map(|p| p[1]).fold(f64::NEG_INFINITY,f64::max)];
        let pl = length(meet(self.lip_x,xp))*integral(&|z| across(&lq,z,yp),zp);
        let pf = length(meet(self.foot_y,yp))*integral(&|z| across(&fq,z,xp),zp);
        let lf = integral(&|z| across(&lq,z,self.foot_y)*across(&fq,z,self.lip_x),pads);
        let plf = integral(&|z| across(&lq,z,meet(self.foot_y,yp))*across(&fq,z,meet(self.lip_x,xp)),zp);
        self.area*(hi[2]-lo[2])+self.strips-pl-pf-lf+plf
    }
}

/// The leans a strip drawn through rounded points takes, within the arrangement's reach of the
/// plate over its pad: none, and up to `7e-7` rad across the pad.
const HAIRS: &[(f64,f64)] = &[(0.,0.),(1e-7,5e-7),(5e-7,-7e-7),(-7e-7,1e-7)];

#[test]
fn strips_in_a_plates_thickness_at_its_true_edge_unite_in_every_order() { strips_on_a_plate(true,0.5,HAIRS,2e-9) }

/// **Strips turned across a plate's faces at an ordinary angle unite with it in every order**
/// (#994): `strips_on_a_plate`'s pads in the plate's thickness, each turned `1e-4` to `1e-3` rad
/// about its own length — so its top and foot cross the plate's along a line, a wedge of the pad
/// standing proud of the plate on one side of it and sunk into it on the other — united in the
/// six orders and with the strips first: closed, the closed form's volume (`Strips::united`) and
/// as many faces in every order. A plate's top and the raised half of a lip's pad crossing it
/// were read as one surface over the line their boxes share alone (`Arrangement::classed`), and
/// the foot's meeting with them laid `3e-4` off the lip's other half: refused as a shell kept on
/// one side of a plane and dropped on another, in some orders.
#[test]
fn strips_crossing_a_plates_faces_in_its_thickness_unite_in_every_order() {
    strips_on_a_plate(true,0.5,&[(3e-4,-5e-4),(1e-3,-7e-4),(-1e-3,1e-3),(5e-4,2e-4),(-3e-4,-1e-3),(3e-4,-3e-4),(1e-3,-1e-3)],0.)
}

/// **Strips abutting a plate's true edges, turned across them at an ordinary angle or a few
/// hundred thousandths of a radian, unite with it in every order** (#994): `strips_on_a_plate`'s
/// pads abutting the plate's edges and one another, each turned about its own length — about the
/// middle of the sheet's thickness, so the lip's end crosses the plate's side, and the foot's side
/// the plate's other side, along lines meeting at the plate's corner, and with the foot turned
/// about a line three tenths up it, so they do not. Closed, the closed form's volume and as many
/// faces in every order. The lip's foot turned `1e-3` rad crosses the foot's side a tolerance and
/// a half under the foot's own edge: the span was dropped as outside the foot's side
/// (`Arrangement::kept`), and the lip's foot left with a slit — refused wherever lip and foot were
/// joined before the plate.
#[test]
fn strips_crossing_a_plates_true_edges_unite_in_every_order() {
    strips_on_a_plate(false,0.5,&[(1e-3,-1e-3),(3e-4,-3e-4),(1e-4,-1e-4),(1e-5,-1e-5),(-1e-3,1e-3)],0.);
    strips_on_a_plate(false,0.3,&[(1e-3,-7e-4),(3e-4,-5e-4),(5e-4,2e-4),(-3e-4,-1e-3),(5e-5,-2e-4),(3e-5,-1e-5)],0.);
}

#[test]
fn strips_abutting_a_plates_true_edges_unite_in_every_order() { strips_on_a_plate(false,0.5,HAIRS,2e-9) }

/// Fixture 115's fragment (#961): two folded strips whose pads overlap, flush, at a corner, and a
/// plate beside them, every coordinate written to `places` decimals — the pads' faces meant to
/// coincide leaning `1e-6` rad apart at five decimals (a bend's radius solved through a point fixed
/// as written), `1e-10` at nine, none at twelve. Its exact volume is the plate's, the strips'
/// sections' (a pad, a quarter or an eighth of an annulus, a leg) times their lengths, less the
/// pads' overlap.
fn strips(places: usize) -> String {
    let f = |x: f64| format!("{x:.places$}");
    let s = 0.5f64.sqrt();
    let o = 17./13.;
    let (p50,p54) = ([o-3.*s,4.-3.*s],[o-4.*s,4.-4.*s]);
    let leg = 9.75264/s;
    let p51 = [p50[0]-leg*s,p50[1]+leg*s];
    let p55 = [p51[0]-s,p51[1]-s];
    let mut src = FRAGMENT.to_string();
    // each point as the fragment writes it, then as the drawing means it
    for (written,meant) in [("(2.30769, ",30./13.),("(1.30769, ",o)] { src = src.replace(written,&format!("({}, ",f(meant))); }
    for (written,meant) in [("-0.81363",p50[0]),("1.87868",p50[1]),("-10.56627",p51[0]),("11.63132",p51[1]),
        ("-1.52074",p54[0]),("1.17157",p54[1]),("-11.27338",p55[0]),("10.92421",p55[1]),("3.61538",47./13.),("2.61538",34./13.),
        ("-0.38462",-5./13.),("-1.38462",-18./13.),("15.98462",207.8/13.),("1.50769",19.6/13.),("96.09231",1249.2/13.),
        ("2.81538",36.6/13.),("94.38462",1227./13.)] {
        assert!(src.contains(written),"{written}");
        src = src.replace(written,&f(meant));
    }
    src
}

/// **Fixture 115's strips are united with the plate at every precision they are written to**
/// (#961): at five decimals the pads' faces lean `1e-6` rad apart over a pad less than a millimetre
/// across — within the arrangement's reach of one another there — and were refused as two planes
/// coinciding along an edge, at seven and nine `1e-8` and `1e-10` apart and refused as a piece
/// touching a face along none, and written exactly they built. Each now builds closed, its volume
/// the closed form's within what rounding its coordinates moves it by (half the last place over
/// the area it moves). The fragment as the issue gives it builds too.
#[test]
fn fixture_115s_strips_unite_at_every_precision() {
    let s = 0.5f64.sqrt();
    let tab = (1.+PI*7./8.+9.75264/s)*29.1;
    let flange = (1.+PI*7./4.+(207.8/13.-4.))*(1229.6/13.);
    let plate = (1229.6/13.)*(1190.4/13.);
    let overlap = 0.8*(-48.6+47./13.+45.9);
    let exact = plate+flange+tab-overlap;
    let area = 2.*(plate+flange+tab);
    let part = |src: &str| {
        let e = crate::common::text::solved(src);
        let x = e.sketch.exact_solid(crate::common::ent(&e,"part").i()).unwrap_or_else(|m| panic!("`part`: {m}"));
        x.brep.check(1e-6).unwrap_or_else(|m| panic!("`part`: {m}"));
        volume(&x.brep)
    };
    for places in [5,6,7,9,12] {
        let v = part(&strips(places));
        close(v,exact,0.5*10f64.powi(-(places as i32))*area+1e-6,&format!("{places} places"));
    }
    let v = part(FRAGMENT);
    close(v,exact,0.5e-5*area,"the issue's fragment");
}

/// Fixture 115 at its true edge (#994) as a document: a plate drawn in `std.top`, a lip bent up
/// along its edge drawn in `std.side` and a foot bent off at its corner drawn in `std.front`, the
/// strips' pads in the plate's thickness — over its edge, and over one another at its corner —
/// every point fixed where it is meant, written to `places` decimals, far from the origin (the
/// fixture's `(-49.8088, -48.6, -2.67665)`); each bend's centre and the ends it starts from fixed,
/// the ends it reaches seeded, as a bend is drawn. `order` unites the three.
fn at_true_edge(places: usize,order: &str) -> String {
    let t = [-49.8088,-48.6,-2.67665];
    let f = |x: f64| format!("{x:.places$}");
    let (x0,y0,x1,y1) = (19.6/13.,36.6/13.,1249.2/13.,1227./13.);
    // a strip's section in its plane, `(a, b)` its coordinates offset by `at`
    let section = |name: &str,at: [f64;2],c: f64,turn: f64,leg: f64| -> String {
        let s = 1.5*PI-turn;
        let (du,dv) = (s.sin(),-s.cos());
        let pt = |p: &str,a: f64,b: f64,fixed: bool| if fixed { format!("  {name}{p} := point\n  fix(({}, {})) {name}{p}\n",f(at[0]+a),f(at[1]+b)) }
            else { format!("  {name}{p} := point hint(({}, {}))\n",f(at[0]+a),f(at[1]+b)) };
        let (i,q) = ((c+3.*s.cos(),4.+3.*s.sin()),(c+4.*s.cos(),4.+4.*s.sin()));
        let edges = [("t","line({n}e1, {n}b1)"),("ai","arc(center: {n}o, start: {n}i, end: {n}b1)"),("li","line({n}i, {n}i2)"),
            ("c","line({n}i2, {n}q2)"),("lo","line({n}q2, {n}q)"),("ao","arc(center: {n}o, start: {n}q, end: {n}b0)"),
            ("b","line({n}b0, {n}e0)"),("e","line({n}e0, {n}e1)")];
        [pt("e1",c+1.,1.,true),pt("b1",c,1.,true),pt("b0",c,0.,true),pt("e0",c+1.,0.,true),pt("o",c,4.,true),
            pt("i",i.0,i.1,false),pt("q",q.0,q.1,false),pt("i2",i.0+leg*du,i.1+leg*dv,true),pt("q2",q.0+leg*du,q.1+leg*dv,true),
            edges.iter().map(|(e,def)| format!("  {name}{e} := {}\n",def.replace("{n}",name))).collect(),
            format!("  {name} := face({})\n",edges.iter().map(|(e,_)| format!("{name}{e}")).collect::<Vec<_>>().join(", "))].concat()
    };
    let corner = |p: &str,x: f64,y: f64| format!("  {p} := point\n  fix(({}, {})) {p}\n",f(t[0]+x),f(t[1]+y));
    format!("use std\nuse std.solids (cut, union)\nin std.top {{\n{}{}{}{}  sheet := face(a1, a2, a3, a4, -> close)\n}}\n\
        in std.side {{\n{}}}\nin std.front {{\n{}}}\n\
        plate := solid(std.solids.Prism(sheet, from: {}mm, to: {}mm))\npart := solid(plate)\n\
        lip_lo := solid(std.solids.Prism(lip, from: {}mm, to: {}mm))\n\
        foot_lo := solid(std.solids.Prism(foot, from: {}mm, to: {}mm))\n{order}",
        corner("a1",x0,y0),corner("a2",x1,y0),corner("a3",x1,y1),corner("a4",x0,y1),
        section("lip",[t[1],t[2]],34./13.,0.5*PI,11.98462),section("foot",[t[0],t[2]],17./13.,0.25*PI,13.79264),
        f(t[2]),f(t[2]+1.),f(t[0]+x0),f(t[0]+x1),f(-(t[1]+31.8)),f(-(t[1]+2.7)))
}

/// **Fixture 115's lip and foot unite with the plate at its true edge, whichever is joined first**
/// (#994): the document `at_true_edge` draws, the strips each united into the part, the foot into
/// the lip first and the lip into the part, and the lip into the foot first — where joined straight
/// onto the plate the fixture's part was refused ("`foot_lo`: a plane's pieces do not close") and
/// joined through the lip it built. Each order builds closed at five and nine decimals, its volume
/// the closed form's (`strips_on_a_plate`'s: the three less what they share) within what rounding
/// the coordinates moves it by.
#[test]
fn fixture_115s_lip_and_foot_unite_with_the_plate_at_its_true_edge_in_any_order() {
    let (x0,y0,x1,y1) = (19.6/13.,36.6/13.,1249.2/13.,1227./13.);
    let (c,d) = (34./13.,17./13.);
    let strips = section(0.5*PI,11.98462)*(x1-x0)+section(0.25*PI,13.79264)*29.1;
    let (lx,fy) = (c+1.-y0,d+1.-x0);
    let exact = (x1-x0)*(y1-y0)+strips-(lx*(x1-x0)+fy*(31.8-y0)+fy*(c+1.-2.7)-fy*lx);
    let area = 2.*((x1-x0)*(y1-y0)+strips);
    for order in ["lip_lo union part\nfoot_lo union part\n","foot_lo union part\nlip_lo union part\n",
        "foot_lo union lip_lo\nlip_lo union part\n","lip_lo union foot_lo\nfoot_lo union part\n"] {
        for places in [5,9] {
            let e = crate::common::text::solved(&at_true_edge(places,order));
            let x = e.sketch.exact_solid(crate::common::ent(&e,"part").i()).unwrap_or_else(|m| panic!("{places} places, {order:?}: {m}"));
            x.brep.check(1e-6).unwrap_or_else(|m| panic!("{places} places, {order:?}: {m}"));
            close(volume(&x.brep),exact,0.5*10f64.powi(-(places as i32))*area+1e-6,&format!("{places} places, {order:?}"));
        }
    }
}

/// Fixture 115's fragment as issue #961 gives it, from delta debugging: five decimals, some points
/// placed by their seeds alone.
const FRAGMENT: &str = "\
use std.solids (cut, union)
pxy := plane
pxz := plane
fix(origin == (-49.8088mm, -48.6mm, -2.67665mm)) pxz
pyz := plane
fix(origin == (-49.8088mm, -48.6mm, -2.67665mm)) pyz
fix(dir == (0, 1, 0)) pyz.u
in pxy {
  p1 := point hint((1.50769, 2.81538))
  p2 := point hint((96.09231, 2.81538))
  p3 := point hint((96.09231, 94.38462))
  p4 := point hint((1.50769, 94.38462))
  l5 := line(p1, p2)
  l6 := line(p2, p3)
  l7 := line(p3, p4)
  l8 := line(p4, p1)
  f9 := face(l5, l6, l7, l8)
}
in pxz {
  p48 := point hint((2.30769, 1))
  p49 := point hint((1.30769, 1))
  p50 := point hint((-0.81363, 1.87868))
  fix((-0.81363mm, 1.87868mm)) p50
  p51 := point hint((-10.56627, 11.63132))
  p52 := point hint((2.30769, 0))
  p53 := point hint((1.30769, 0))
  p54 := point hint((-1.52074, 1.17157))
  p55 := point hint((-11.27338, 10.92421))
  o57 := point hint((1.30769, 4))
  o62 := point hint((1.30769, 4))
  l56 := line(p48, p49)
  a58 := arc(center: o57, start: p50, end: p49) hint(r: 3)
  l59 := line(p50, p51)
  l60 := line(p51, p55)
  l61 := line(p55, p54)
  a63 := arc(center: o62, start: p54, end: p53) hint(r: 4)
  l64 := line(p53, p52)
  l65 := line(p52, p48)
  f66 := face(l56, a58, l59, l60, l61, a63, l64, l65)
}
in pyz {
  p10 := point hint((3.61538, 1))
  p11 := point hint((2.61538, 1))
  p12 := point hint((-0.38462, 4))
  p13 := point hint((-0.38462, 15.98462))
  p14 := point hint((3.61538, 0))
  p15 := point hint((2.61538, 0))
  p16 := point hint((-1.38462, 4))
  p17 := point hint((-1.38462, 15.98462))
  o19 := point hint((2.61538, 4))
  o24 := point hint((2.61538, 4))
  l18 := line(p10, p11)
  a20 := arc(center: o19, start: p12, end: p11) hint(r: 3)
  l21 := line(p12, p13)
  l22 := line(p13, p17)
  l23 := line(p17, p16)
  a25 := arc(center: o24, start: p16, end: p15) hint(r: 4)
  l26 := line(p15, p14)
  l27 := line(p14, p10)
  f28 := face(l18, a20, l21, l22, l23, a25, l26, l27)
}
plate := solid(std.solids.Prism(f9, from: 0mm, to: 1mm))
part := solid(plate)
flange_lo := solid(std.solids.Prism(f28, from: 1.50769mm, to: 96.09231mm))
tab_lo := solid(std.solids.Prism(f66, from: -31.8mm, to: -2.7mm))
flange_lo union part
tab_lo union part
";

/// **Two lines crossing at a grazing angle cross where they cross** (brep-bench
/// `tangent/cube-turned-a-hair`): a square's side and the side of the square turned `0.001°` meet
/// `5 tan(θ/2)` from the side's middle. Read through `1 − (d₁·d₂)²`, which loses half the digits
/// to cancellation when the lines lie `1.7e-5` rad apart, their crossing came out `3.7e-7` off —
/// thirty-seven tolerances — while the planes' own crossing stood where it is; read through
/// `|d₁ × d₂|²` it is exact to the rounding of what the lines are drawn through, moved along the
/// side by that rounding over `sin θ` (`1e-11` here), a tenth of the tolerance.
#[test]
fn lines_crossing_at_a_grazing_angle_cross_where_they_cross() {
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::query::curve_curve;
    let theta = 0.001f64.to_radians();
    let (c,s) = (theta.cos(),theta.sin());
    let turned = |x: f64,y: f64| [c*x-s*y,s*x+c*y,0.];
    let (from,to) = (turned(-5.,-5.),turned(5.,-5.));
    let length = ((to[0]-from[0]).powi(2)+(to[1]-from[1]).powi(2)).sqrt();
    let side = Curve::Line {p:[-5.,-5.,0.],d:[1.,0.,0.]};
    let other = Curve::Line {p:from,d:[(to[0]-from[0])/length,(to[1]-from[1])/length,0.]};
    let met = curve_curve(&side,[0.,10.],&other,[0.,length],tolerance(10.)).expect("two lines are read in closed form");
    assert_eq!(met.len(),1,"one crossing: {met:?}");
    close(met[0].1[0],5.*(theta/2.).tan(),0.1*tolerance(10.),"the crossing along the side");
    close(met[0].1[1],-5.,1e-12,"the crossing on the side");
}

/// **A cube united with itself turned a hair is the prism over the two squares' union** (brep-bench
/// `tangent/cube-turned-a-hair`): each pair of sides `0.001°` apart crosses near its middle, and
/// the union is larger than the cube by `8.7e-6` of its volume and area. Its crossings with the
/// cube's top and foot read two ways — the planes', and the edges' as lines — stood `3.7e-7`
/// apart, and the sliver of edge between them, along which both sides rise square, could not be
/// ordered round (`faces … coincide along it`).
#[test]
fn a_cube_united_with_itself_turned_a_hair() {
    use gcs_core::brep::boolean::combined;
    use gcs_core::brep::props::area;
    let cube = block([-5.,-5.,0.],[5.,5.,10.]);
    let turned = cube.moved(&Rigid::turn([0.;3],XY,0.001f64.to_radians()));
    for (x,y) in [(&cube,&turned),(&turned,&cube)] {
        let u = combined(x,y,Op::Union,0.).unwrap_or_else(|e| panic!("the union: {e}"));
        u.check(10.*tolerance(u.size())).unwrap_or_else(|e| panic!("the union: {e}"));
        close(volume(&u),1000.0087264939527,1e-9,"the union's volume");
        close(u.faces.iter().map(|f| area(&u,f)).sum(),600.0052358963717,1e-9,"the union's area");
    }
}
