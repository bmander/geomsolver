//! **The closed class** (#721, `solid::closed_class`, `brep::sweep::closed`): a sweep the other
//! classes refuse, of a polyhedral tool under one slide, turn or screw, admitted by its structure
//! (rows C1, C2: the kernel's `swept::closed::reads`) and cut from its blank on the kernel's one
//! arrangement (`swept::closed::cut_from`, #735), its cells kept where the kernel's membership
//! reads them no member of the sweep (C3). Each is a case another class refuses by name, its STL
//! held to the material field with no disagreement, and its volume to a figure made apart from the
//! build (the closed class is asked only where the others refuse, so a sweep read as closed is one
//! they refuse); and what the class's rows refuse stands as the first class's refusal.
use gcs_core::solid::admission::{Class,Condition};

use crate::closed_characteristics::square_bar;
use crate::solid_sweeps_examples::class;
use crate::turns_and_slides::{admit,exported,near,refused};

/// The push up the plate leaning toward x, `(0.3, 0, √(1 − 0.3²))`: its rise.
const RISE: f64 = 0.9539392014169457;

/// The push up the plate leaning toward x, as `punch.sv`'s ram's axis is fixed.
fn aslant() -> String { format!("fix(dir == (0.3, 0, {RISE})) up") }

/// **A bar pushed aslant wholly through a plate takes its shadow**: `closed_characteristics`'s bar
/// of square section, `|y| + |z + 6| ≤ 1` along x from 6 to 14, pushed 16 up through the 20 × 10 × 4
/// plate leaning toward x. The constant-twist class refuses it (S1: its silhouette turns at a
/// vertex onto a cap). Apart from the build: the bar passes wholly through, so each slice of the
/// sweep within the plate is the bar's shadow along the push, `(x − k z, y)` of the bar's points
/// with `k = Dx / Dz`: across `y` it is `8 + 2 k (1 − |y|)` long, `16 + 2k` in all, through the
/// plate's 4.
#[test]
fn a_bar_pushed_aslant_through_a_plate_takes_its_shadow() {
    let text = square_bar().replace("fix(dir == (0, 0, 1)) up",&aslant());
    let e = fixtures::read(&text);
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw.is_slide(),"a slide");
    assert_eq!((found.tool.faces.len(),found.tool.edges.len()),(6,12),"a bar of six planes and twelve lines");
    near(exported(&e,"part"),800.-plunged_away(16.),1e-9);
}

/// What the closed class's rows refuse stands as the class asked first refused it: the punch's
/// drum, a curved tool (C1), pushed aslant is the constant-twist class's refusal still.
#[test]
fn a_sweep_the_closed_class_refuses_keeps_its_first_refusal() {
    let drum = crate::closed_characteristics::punch().replace("fix(dir == (0, 0, 1)) up",&aslant());
    let r = refused(admit(&drum));
    assert_eq!(r.class(),"constant-twist class","{r}");
    assert!(!matches!(r.condition,Condition::Faces | Condition::Constant | Condition::Settled),"{r}");
}

/// **A small bar turned wholly through a ring sector bores its solid of revolution's sixth**:
/// `prism_twists`'s bar 1 × 1 × 1.2, `x` from 10 to 11, leaning 20° across the turn, turned about z
/// from −140° to 140° through `turn.sv`'s block, the ring from 8 to 12 at heights ±1, 60° round.
/// The constant-twist class refuses it (S1: a rim's ruling in contact all along it). Its edges
/// lean to the axis and turn into hyperboloids, its faces' contact lines into cones. Apart from
/// the build: the bar stands within the ring's radii and heights and passes wholly through its 60°,
/// so the ring loses a sixth of the bar's solid of revolution, `∫ π (ρ_max² − ρ_min²) dz` over its
/// slices' radial spans (`swept_closed::revolved_volume`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 12 s: a leaning bar's hyperboloids cut from a ring sector")]
fn a_leaning_bar_turned_wholly_through_a_ring_sector_bores_its_shadow() {
    use gcs_core::brep::geom::Rigid;
    let lean = 20.;
    let e = fixtures::read(&crate::prism_twists::turned_bar(lean,[[10.,-0.5],[11.,-0.5],[11.,0.5],[10.,0.5]],0.6,0.));
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw.is_turn(),"a turn");
    let bar = crate::brep::block([10.,-0.5,-0.6],[11.,0.5,0.6]).moved(&Rigid::turn([0.;3],[1.,0.,0.],-f64::to_radians(lean)));
    let whole = crate::swept_closed::revolved_volume(&bar);
    let ring = std::f64::consts::PI/3.*(144.-64.);
    near(exported(&e,"part"),ring-whole/6.,1e-9);
}

/// What `stopped_turn.sv`'s ring loses (X1, the bar's end pose trimmed against what it swept): the
/// bar upright, 1 × 1 × 1.2 from 10 out, turned from −140° to 30°, halfway across the sector's 60°.
/// Apart from the build: each circle of radius `ρ` meets the bar in the angles `±a(ρ)`, `a` the
/// bar's half-width `min(½, √(ρ² − 10²))` over `ρ`, so the turn covers the sector's angles from 0
/// to `a + 30°`; the ring loses `1.2 ∫ ρ (a(ρ) + π/6) dρ` from 10 to `√(11² + ½²)`, by Simpson's
/// rule either side of the kink at `√(10² + ½²)`.
pub(crate) fn turned_partway_away() -> f64 {
    let a = |r: f64| (0.5f64.min((r*r-100.).max(0.).sqrt())/r).asin();
    let f = |r: f64| r*(a(r)+std::f64::consts::PI/6.);
    1.2*(crate::brep::simpson(10.,100.25f64.sqrt(),4000,f)+crate::brep::simpson(100.25f64.sqrt(),121.25f64.sqrt(),4000,f))
}

/// What `plunge.sv`'s plate loses (X1): the bar pushed `push` up the slanted line. Apart from the
/// build: at each `y` the bar's section is the rectangle `[6, 14] × [−6 − w, −6 + w]` in `(x, z)`,
/// `w = 1 − |y|`, swept along the push `D` into a hexagon, of which the plate keeps `0 ≤ z ≤ 4`
/// (clipped and measured by the shoelace); the plate loses that integrated over `y`. (Pushed 16, the
/// bar passes wholly through: `4 (16 + 2k)`.)
pub(crate) fn plunged_away(push: f64) -> f64 {
    let d = [push*0.3,push*RISE];
    let section = |y: f64| -> f64 {
        let w = 1.-y.abs();
        let (x0,x1,z0,z1) = (6.,14.,-6.-w,-6.+w);
        // the rectangle swept along `d`: its corners and theirs moved, their hull, wound counter-clockwise
        let pts = [[x0,z0],[x1,z0],[x1+d[0],z0+d[1]],[x1+d[0],z1+d[1]],[x0+d[0],z1+d[1]],[x0,z1]];
        let clip = |poly: Vec<[f64;2]>,keep: &dyn Fn([f64;2]) -> f64| -> Vec<[f64;2]> {
            let mut out = Vec::new();
            for k in 0..poly.len() {
                let (p,q) = (poly[k],poly[(k+1)%poly.len()]);
                let (fp,fq) = (keep(p),keep(q));
                if fp >= 0. { out.push(p) }
                if (fp >= 0.) != (fq >= 0.) { let t = fp/(fp-fq); out.push([p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])]) }
            }
            out
        };
        let kept = clip(clip(pts.to_vec(),&|p| p[1]),&|p| 4.-p[1]);
        gcs_core::solid::ring_area(&kept.iter().map(|p| (p[0],p[1])).collect::<Vec<_>>()).abs()
    };
    crate::brep::simpson(-1.,0.,2000,section)+crate::brep::simpson(0.,1.,2000,section)
}

/// What `stopped_screw.sv`'s rod loses (X1 under a screw): `square_thread.sv`'s bar, its tip 4 from
/// the axis, 2 wide along it and standing `reach` either side of its plane, screwed at a lead of 6
/// from clear below the rod until its middle stands `stop` up it. Apart from the build: a point at
/// radius `ρ` and height `z` is cut where some roll `θ` carries it back into the bar, `|z − k θ| ≤ 1`
/// and its angle within `a(ρ) = min(acos(4 / ρ), asin(reach / ρ))` of the bar's plane, `k` the lead
/// a radian; so the circle there loses `|Θ| + 2 a(ρ)` of its angle, `Θ` the rolls of the sweep within
/// `[(z − 1) / k, (z + 1) / k]`, and the rod loses that integrated over `ρ` from 4 to 5 (split where
/// the reach takes over) and `z` over its 20 (Simpson's rule, between the heights where `Θ`'s ends
/// change, each window read just inside its ends: at `k θ₁ + 1` the covered set's measure jumps to
/// 0). Screwed past the rod's top at a reach of 8 it is `square_thread.sv`'s closed form.
pub(crate) fn screwed_partway_away(stop: f64,reach: f64) -> f64 {
    use std::f64::consts::TAU;
    let k = 6./TAU;
    let (t0,t1) = (-2./k,stop/k);
    let a = |r: f64| (4./r).min(1.).acos().min((reach/r).min(1.).asin());
    let covered = |r: f64,z: f64| -> f64 {
        let (lo,hi) = (((z-1.)/k).max(t0),((z+1.)/k).min(t1));
        if hi < lo { 0. } else { hi-lo+2.*a(r) }
    };
    // ρ from 4 to 5 as 4 + s², so the root at 4 is smooth in s, split where the reach takes over
    let kink = ((16.+reach*reach).sqrt()-4.).clamp(0.,1.).sqrt();
    let ring = |z: f64| [(0.,kink),(kink,1.)].iter().filter(|w| w.1 > w.0)
        .map(|&(s0,s1)| crate::brep::simpson(s0,s1,400,|s: f64| { let r = 4.+s*s; covered(r,z)*r*2.*s })).sum::<f64>();
    let mut heights = vec![0.,20.,k*t0-1.,k*t0+1.,k*t1-1.,k*t1+1.];
    heights.retain(|z| (0. ..=20.).contains(z));
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a,b| (*a-*b).abs() <= 1e-9);
    heights.windows(2).map(|w| crate::brep::simpson(w[0],w[1],400,|z: f64| ring(z.clamp(w[0]+1e-12,w[1]-1e-12)))).sum()
}

/// The independent figures held to what they reduce to: pushed wholly through, the plunge is the
/// bar's shadow along the push; screwed past the rod's top, the stopped screw is
/// `square_thread.sv`'s closed form.
#[test]
fn the_partway_figures_reduce_to_the_whole_ones() {
    use std::f64::consts::{PI,TAU};
    let k = 0.3/RISE;
    assert!((plunged_away(16.)-4.*(16.+2.*k)).abs() <= 1e-9,"{}",plunged_away(16.));
    let (r0,r,w,lead,l) = (4_f64,5_f64,2.,6.,20.);
    let turned = r*r/2.*(r0/r).acos()-r0/2.*(r*r-r0*r0).sqrt();
    let groove = TAU*l/lead*(w*(r*r-r0*r0)/2.+lead/PI*turned);
    assert!((screwed_partway_away(21.,8.)-groove).abs() <= 1e-8*groove,"{} against {groove}",screwed_partway_away(21.,8.));
}
