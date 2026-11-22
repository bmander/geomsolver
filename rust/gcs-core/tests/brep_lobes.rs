//! **A boss, a bore or a rod on a lobe of a plate's outline, of the lobe's own radius** (#994,
//! fixture 141 of #961 item 3): an outline of lobes joined by concave blends tangent to them, a
//! cylinder standing on a lobe about its centre — the lobe's wall and the boss's one surface, or a
//! hair apart either way, within the arrangement's reach and past it — united or cut, closed and
//! to its volume. Where the radii stand a few tolerances apart the boss's rim overhangs the plate's
//! top along the lobe in a crescent that wide, and crosses each blend twice at a shallow angle, a
//! stretch apart.
use gcs_core::brep::boolean::{combined_written,Op};
use gcs_core::brep::build::{prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Curve,Frame,V};
use gcs_core::brep::props::volume;
use std::f64::consts::{PI,TAU};

use crate::brep::rod;

const XY: V = [0.,0.,1.];

fn arc(c: [f64;2],r: f64,a0: f64,a1: f64) -> ProfileEdge {
    ProfileEdge::Arc {frame:Frame::new([c[0],c[1],0.],XY,[1.,0.,0.]),r,span:Some([a0,a1])}
}
/// Half of `x dy - y dx` along an arc turning counter-clockwise from `a0` to `a1`.
fn swept(c: [f64;2],r: f64,a0: f64,a1: f64) -> f64 {
    0.5*(r*r*(a1-a0)+r*(c[0]*(a1.sin()-a0.sin())-c[1]*(a1.cos()-a0.cos())))
}
fn ang(d: [f64;2]) -> f64 { d[1].atan2(d[0]) }

/// A plate of `n` lobes of radius `r` on a pitch circle `p`, joined by concave blends of radius
/// `rho` tangent to the lobes either side (`r + rho` from both centres), around a hole of radius
/// `hole`: its profile, its area (Green's theorem over its arcs) and the lobes' centres.
fn lobed(n: usize,p: f64,r: f64,rho: f64,hole: f64) -> (Profile,f64,Vec<[f64;2]>) {
    let half = PI/n as f64;
    let q = p*half.cos()+((r+rho).powi(2)-(p*half.sin()).powi(2)).sqrt();
    let c: Vec<[f64;2]> = (0..n).map(|i| { let t = TAU*i as f64/n as f64; [p*t.cos(),p*t.sin()] }).collect();
    let b: Vec<[f64;2]> = (0..n).map(|i| { let t = TAU*i as f64/n as f64+half; [q*t.cos(),q*t.sin()] }).collect();
    let mut edges = Vec::new();
    let mut area = 0.;
    for i in 0..n {
        let (bp,bn) = (b[(i+n-1)%n],b[i]);
        let a0 = ang([bp[0]-c[i][0],bp[1]-c[i][1]]);
        let mut a1 = ang([bn[0]-c[i][0],bn[1]-c[i][1]]);
        while a1 <= a0 { a1 += TAU }
        edges.push(arc(c[i],r,a0,a1));
        area += swept(c[i],r,a0,a1);
        // the blend from lobe i to lobe i+1, turning clockwise
        let cn = c[(i+1)%n];
        let s0 = ang([c[i][0]-bn[0],c[i][1]-bn[1]]);
        let mut s1 = ang([cn[0]-bn[0],cn[1]-bn[1]]);
        while s1 >= s0 { s1 -= TAU }
        while s1 < s0-TAU { s1 += TAU }
        let (lo,hi) = if s0-s1 < PI { (s1,s0) } else { (s0,s1+TAU) };
        edges.push(arc(bn,rho,lo,hi));
        area -= swept(bn,rho,lo,hi);
    }
    let mut loops = vec![edges];
    if hole > 0. {
        loops.push(vec![ProfileEdge::Arc {frame:Frame::new([0.;3],XY,[1.,0.,0.]),r:hole,span:None}]);
        area -= PI*hole*hole;
    }
    (Profile {names:vec![],origin:[0.;3],normal:XY,loops},area,c)
}

/// The cases: the plates (lobes, pitch, lobe and blend radii), and what stands on a lobe — the
/// part of its height `[z0, z1]` and whether it is united or cut.
const PLATES: [(usize,f64,f64,f64);3] = [(12,40.,6.,6.),(6,30.,8.,8.),(4,25.,7.,12.)];
const STANDS: [(&str,[f64;2],Op);5] = [("a boss",[5.,12.],Op::Union),("a rod through",[0.,12.],Op::Union),
    ("a rod from below",[-3.,12.],Op::Union),("a counterbore",[2.,8.],Op::Cut),("a bore through",[-1.,8.],Op::Cut)];

/// What a rod of radius `rr` standing over `z` on a plate of volume `v0` 5 thick leaves, by `op`,
/// and how far the volume may be from it: the rod's wall read on the lobe's where the two are one
/// surface, within `reach` of it (as `brep_coincident::moved_by`), and past the lobe's circle (its
/// radius more than the lobe's `r`, or its axis `off` the lobe's), its ring over the plate's
/// thickness it overlaps.
fn expected(v0: f64,r: f64,rr: f64,off: f64,z: [f64;2],op: Op,reach: f64) -> (f64,f64) {
    let inside = (z[1].min(5.)-z[0].max(0.)).max(0.);
    let v = match op { Op::Union => v0+PI*rr*rr*(z[1]-z[0]-inside),_ => v0-PI*rr*rr*inside };
    let past = (rr-r).max(0.)+off;
    (v,2.*TAU*rr*past*inside+TAU*rr*(z[1]-z[0])*reach+1e-9*v0)
}

/// **A boss on a lobe, of its radius or a hair off it, is united with the plate; a bore of it is
/// cut** (fixture 141): every plate, on two of its lobes, each stand at the lobe's radius exactly,
/// within the arrangement's reach of it (`ON_CURVE` tolerances: one surface), and past it by a few
/// to a thousand tolerances, either way — and the rod's axis moved off the lobe's within the reach:
/// closed, to the closed form's volume. Refused before as "a shell of the
/// arrangement is kept on one side of a cylinder … two of its cells joined round an edge" and "a
/// plane's pieces do not close" at radii a micron apart.
#[test]
fn a_boss_or_a_bore_on_a_lobe_of_its_own_radius_builds() {
    let offs: [(f64,f64);15] = [(0.,0.),(1e-8,0.),(-1e-8,0.),(3e-7,0.),(-3e-7,0.),(1e-6,0.),(-1e-6,0.),(3e-6,0.),(-3e-6,0.),
        (1e-5,0.),(-1e-5,0.),(1e-4,0.),(-1e-4,0.),(0.,1e-8),(0.,3e-7)];
    for (n,p,r,rho) in PLATES {
        let (prof,area,c) = lobed(n,p,r,rho,10.);
        let plate = prism(&prof,0.,5.).unwrap();
        let v0 = volume(&plate);
        assert!((v0-5.*area).abs() <= 1e-9*v0,"the plate of {n} lobes: {v0} against {}",5.*area);
        for k in [0,1] { for (what,z,op) in STANDS { for (dr,dc) in offs {
            let rr = r+dr;
            let rod = rod([c[k][0]+0.6*dc,c[k][1]+0.8*dc,0.],rr,z);
            let said = format!("{what} on lobe {k} of {n}, its radius {dr:e} off and its axis {dc:e}");
            let out = combined_written(&plate,None,&rod,op,0.).unwrap_or_else(|e| panic!("{said}: {e}")).0;
            out.check(10.*tolerance(plate.size().max(rod.size()))).unwrap_or_else(|e| panic!("{said}: {e}"));
            let tol = tolerance(plate.size().max(rod.size()));
            let reach = gcs_core::brep::tolerant::ON_CURVE*tol;
            // (a rod whose end circle lies on the lobe's, concentric, its radius off by no more
            // than `FLUSH` tolerances, is given the lobe's radius before the two are combined —
            // #996's `flush::tubes_flushed`: the seam is then the tangent one)
            let on_lobe = dc == 0. && z.iter().any(|&h| h == 0. || h == 5.);
            let flushed = on_lobe && dr.abs() > 1e-12*rr && dr.abs() <= gcs_core::brep::flush::FLUSH*tol;
            let (want,within) = expected(v0,r,if flushed { r } else { rr },dc,z,op,reach);
            let v = volume(&out);
            assert!((v-want).abs() <= within,"{said}: {v} against {want}, within {within:e}");
        } } }
    }
}

/// **Two circles in one plane meet as far apart as they stand, not as their chord stands inside
/// one** (`query::curve_curve`): a circle overlapping another a few tolerances deep crosses it
/// twice, a stretch apart, as the cylinders through them meet in two lines; overlapping it within
/// the tolerance, it touches it once. Read by the chord they share, which stands inside the first
/// half as deep where their radii are one, a lens twice the tolerance deep was a touch between the
/// two crossings its edges' surfaces read, and a third vertex on the edge.
#[test]
fn two_circles_overlapping_a_few_tolerances_cross_twice() {
    use gcs_core::brep::query::curve_curve;
    let tol = 1e-7;
    let circle = |c: [f64;2],r: f64| Curve::Circle(Frame::new([c[0],c[1],0.],XY,[1.,0.,0.]),r);
    for (r,s) in [(8.,8.),(6.,9.),(9.,4.)] {
        let a = circle([0.,0.],r);
        for (deep,crossings) in [(1.5*tol,2),(10.*tol,2),(0.5*tol,1),(0.,1)] {
            let b = circle([r+s-deep,0.],s);
            let at = curve_curve(&a,[-PI,PI],&b,[0.,TAU],tol).expect("two circles have a closed form");
            assert_eq!(at.len(),crossings,"radii {r} and {s} overlapping {deep:e}: {at:?}");
            for (_,q) in &at {
                for (o,rad) in [([0.,0.],r),([r+s-deep,0.],s)] {
                    assert!(((q[0]-o[0]).hypot(q[1]-o[1])-rad).abs() <= tol,"radii {r} and {s}: {q:?} off a circle");
                }
            }
            if crossings == 2 {
                // a chord across the lens, its half `h`: `deep` is about h²(1/r + 1/s)/2
                let h = (2.*deep/(1./r+1./s)).sqrt();
                let apart = ((at[0].1[0]-at[1].1[0]).powi(2)+(at[0].1[1]-at[1].1[1]).powi(2)).sqrt();
                assert!((apart-2.*h).abs() <= 1e-3*h,"radii {r} and {s} overlapping {deep:e}: {apart:e} apart, against {:e}",2.*h);
            }
        }
    }
}
