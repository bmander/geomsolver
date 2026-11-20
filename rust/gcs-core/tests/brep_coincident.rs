//! **Faces on one surface but for the rounding of what they are drawn through are on one surface**
//! (#961): a pad resting on a plate, two blocks lying flush side by side, two rods of one radius on
//! one axis — each drawn through rounded points, so the faces meant to coincide lean a hair apart
//! (`1e-12` to `1e-8` rad) or stand a few tolerances off — are united and cut as the coincident
//! faces they are within the arrangement's reach, whichever operand comes first and however the
//! lean is turned: one surface wherever any phase asks (`meeting::Shared::index_over`,
//! `ssi::same_over`), their volumes those of the solids drawn and their boundaries closed. A lean
//! past the reach is two surfaces still.
use gcs_core::brep::boolean::{combined_written,Op};
use gcs_core::brep::build::{prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Frame,Rigid,Surface,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::ssi::{same,same_over};
use gcs_core::brep::topo::Brep;
use std::f64::consts::PI;

const XY: V = [0.,0.,1.];

fn poly(pts: &[V]) -> Vec<ProfileEdge> {
    (0..pts.len()).map(|i| ProfileEdge::Line {a:pts[i],b:pts[(i+1)%pts.len()]}).collect()
}
fn block(lo: V,hi: V) -> Brep {
    let p = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![poly(&[[lo[0],lo[1],0.],[hi[0],lo[1],0.],[hi[0],hi[1],0.],[lo[0],hi[1],0.]])]};
    prism(&p,lo[2],hi[2]).unwrap()
}
fn rod(c: V,r: f64,z: [f64;2]) -> Brep {
    let arc = ProfileEdge::Arc {frame:Frame::new([c[0],c[1],0.],XY,[1.,0.,0.]),r,span:None};
    prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![vec![arc]]},z[0],z[1]).unwrap()
}
fn close(a: f64,b: f64,within: f64,what: &str) { assert!((a-b).abs() <= within,"{what}: {a} against {b}"); }

/// How far a volume may move when a face of area `area` is read on another surface standing within
/// the arrangement's reach of it (`ON_CURVE` tolerances) over it: the solids' tolerance.
fn moved_by(a: &Brep,b: &Brep,area: f64) -> f64 { gcs_core::brep::tolerant::ON_CURVE*tolerance(a.size().max(b.size()))*area }

/// `a` combined with `b` by `op` as a body's Boolean is (`combined_written`), checked closed to the
/// ten tolerances its Boolean checks it to (an edge laid on a face of the other operand stands off
/// it by as much as the two faces stood apart there).
fn built(a: &Brep,b: &Brep,op: Op,what: &str) -> Brep {
    let out = combined_written(a,None,b,op,0.).unwrap_or_else(|e| panic!("{what}: {e}")).0;
    out.check(10.*tolerance(a.size().max(b.size()))).unwrap_or_else(|e| panic!("{what}: {e}"));
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
}

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
