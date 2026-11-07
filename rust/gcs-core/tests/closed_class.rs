//! **The closed class** (#721, `solid::closed_class`, `brep::sweep::closed`): a sweep the other
//! classes refuse, of a polyhedral tool under one slide, turn or screw, admitted by its structure
//! (rows C1, C2: the kernel's `swept::closed::reads`) and cut from its blank on the kernel's one
//! arrangement (`swept::closed::cut_from`, #735), its cells kept where the kernel's membership
//! reads them no member of the sweep (C3). Each is a case another class refuses by name, its STL
//! held to the material field with no disagreement, and its volume to a figure made apart from the
//! build (the closed class is asked only where the others refuse, so a sweep read as closed is one
//! they refuse); and what the class's rows refuse stands as the first class's refusal.
use gcs_core::solid::admission::{Class,Condition};
use std::f64::consts::PI;

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
    assert!(found.screw().unwrap().is_slide(),"a slide");
    assert_eq!((found.tool.faces.len(),found.tool.edges.len()),(6,12),"a bar of six planes and twelve lines");
    near(exported(&e,"part"),800.-plunged_away(16.),1e-9);
}

/// **A drum pushed aslant wholly through a plate takes its shadow** (#721 F2c5, a curved tool under
/// a slide): `punch.sv`'s drum, radius 2 and 8 long along x at `z = −5`, pushed 16 up through the
/// plate leaning toward x, which the constant-twist class refuses (its first refusal, a negative
/// control here until the closed class took curved tools). Its side's two rulings square to the
/// push sweep their tangent planes, its rims oblique cylinders. Apart from the build: the drum
/// passes wholly through, so each slice within the plate is its shadow along the push, across `y`
/// the drum's length and `2k√(4 − y²)` more, `32 + 4πk` in all, through the plate's 4.
#[test]
fn a_drum_pushed_aslant_through_a_plate_takes_its_shadow() {
    let drum = crate::closed_characteristics::punch().replace("fix(dir == (0, 0, 1)) up",&aslant());
    let e = fixtures::read(&drum);
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_slide(),"a slide");
    near(exported(&e,"part"),800.-drum_plunged_away(16.),1e-9);
}

/// **A drum pushed aslant partway into a plate** (X1 with a curved tool): pushed 8, it stops with
/// its axis 2.63 up the plate's 4, so the plate keeps the drum's end pose where it stops — a
/// cylinder its side's tangent planes and its rims' oblique cylinders each touch along a line or at
/// a point. Apart from the build, `drum_plunged_away`.
#[test]
fn a_drum_pushed_aslant_partway_into_a_plate_leaves_its_end_pose() {
    let drum = crate::closed_characteristics::punch().replace("fix(dir == (0, 0, 1)) up",&aslant()).replace("slides(16mm)","slides(8mm)");
    let e = fixtures::read(&drum);
    let Class::Closed(_) = class(&e,"part") else { panic!("the closed class") };
    near(exported(&e,"part"),800.-drum_plunged_away(8.),1e-9);
}

/// **A drum turned partway into a ring sector stops there** (X1 with a curved tool under a turn): the
/// drum, radius 1 and 1 high, its axis along the turn's 10 from it, turned from −140° to 30°,
/// halfway across the sector's 60°. The constant-twist class refuses it (it ends in the blank); the
/// closed class builds it, the drum at its end pose touching along a line each cylinder its
/// generators turn into. Apart from the build: each circle of radius `ρ` meets the drum's section in
/// the angles `±a(ρ)`, `cos a = (ρ² + 10² − 1)/(20 ρ)`, so the turn covers the sector's angles from
/// 0 to `30° + a(ρ)`; the ring loses `∫ ρ (π/6 + a(ρ)) dρ` from 9 to 11, over `ρ = 10 + sin s` so
/// the roots at its ends are smooth.
#[test]
fn a_drum_turned_partway_into_a_ring_sector_stops_there() {
    let e = fixtures::read(&crate::closed_characteristics::turned_drum().replace("(-140deg, 140deg)","(-140deg, 30deg)"));
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_turn(),"a turn");
    near(exported(&e,"part"),PI/3.*80.-drum_turned_partway_away(),1e-9);
}

/// What the ring loses to the drum turned to 30° (`a_drum_turned_partway_into_a_ring_sector_stops_there`).
pub(crate) fn drum_turned_partway_away() -> f64 {
    use std::f64::consts::FRAC_PI_2;
    let a = |r: f64| ((r*r+99.)/(20.*r)).min(1.).acos();
    crate::brep::simpson(-FRAC_PI_2,FRAC_PI_2,4000,|s: f64| { let r = 10.+s.sin(); r*(PI/6.+a(r))*s.cos() })
}

/// What the closed class's rows refuse stands as the class asked first refused it: a tapered mill, a
/// cone about an axis leaning 10° to the screw's (C1: under a screw a cone about an axis along it
/// only, #780 C2), screwed half a millimetre a turn to a stop inside the ring sector is the
/// constant-twist class's refusal still.
#[test]
fn a_sweep_the_closed_class_refuses_keeps_its_first_refusal() {
    let (s,c) = 10f64.to_radians().sin_cos();
    let r = refused(admit(&crate::flat_faces::turned_mill(1.,2.,10.,0.).replace("use std.motion (turns)","use std.motion (screws)")
        .replace("swing turns(1) ax","swing screws(0.5mm) ax").replace("(-140deg, 140deg)","(-140deg, 30deg)")
        .replace("fix(dir == (0, 0, 1)) up",&format!("fix(dir == (0, {s}, {c})) up"))));
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
    assert!(found.screw().unwrap().is_turn(),"a turn");
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
    let section = |y: f64| swept_section(push,[6.,14.],-6.,1.-y.abs());
    crate::brep::simpson(-1.,0.,2000,section)+crate::brep::simpson(0.,1.,2000,section)
}

/// The area of the plate's slice `0 ≤ z ≤ 4` a rectangle `[x0, x1] × [zc − w, zc + w]` in `(x, z)`
/// sweeps pushed `push` along the slanted line: the hexagon it sweeps, clipped to the plate and
/// measured by the shoelace.
fn swept_section(push: f64,[x0,x1]: [f64;2],zc: f64,w: f64) -> f64 {
    let d = [push*0.3,push*RISE];
    let (z0,z1) = (zc-w,zc+w);
    // the rectangle swept along `d`: its corners and theirs moved, their hull, wound
    // counter-clockwise
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
}

/// What the plate loses to `punch.sv`'s drum pushed `push` up the slanted line: at each `y` the
/// drum's section is the rectangle `[6, 14] × [−5 − w, −5 + w]`, `w = √(4 − y²)`, swept and clipped
/// as `plunged_away`'s; integrated over `y = 2 sin s`, so the ends of the root are smooth, by
/// Simpson's rule between the kinks, where a corner of the hexagon (at `−5 ± w`, carried up the
/// push or not) crosses the plate's top or bottom. (Pushed 16, it passes wholly through:
/// `4 (32 + 4πk)`.)
pub(crate) fn drum_plunged_away(push: f64) -> f64 {
    use std::f64::consts::FRAC_PI_2;
    let mut kinks = vec![-FRAC_PI_2,FRAC_PI_2];
    for target in [0.,4.] { for shift in [0.,push*RISE] {
        let w = (target-(-5.+shift)).abs();
        if w < 2. { let s = (w/2.).acos(); kinks.extend([-s,s]) }
    } }
    kinks.sort_by(f64::total_cmp);
    kinks.windows(2).filter(|k| k[1] > k[0])
        .map(|k| crate::brep::simpson(k[0],k[1],2000,|s: f64| swept_section(push,[6.,14.],-5.,2.*s.cos())*2.*s.cos())).sum()
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
    // the drum's section, the unit disc, by the circles about the axis it meets
    let disc = 2.*crate::brep::simpson(-std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2,4000,|s: f64| {
        let r = 10.+s.sin();
        r*((r*r+99.)/(20.*r)).min(1.).acos()*s.cos()
    });
    assert!((disc-PI).abs() <= 1e-9*PI,"{disc}");
    let shadow = 4.*(32.+4.*PI*k);
    assert!((drum_plunged_away(16.)-shadow).abs() <= 1e-9*shadow,"{} against {shadow}",drum_plunged_away(16.));
    let (r0,r,w,lead,l) = (4_f64,5_f64,2.,6.,20.);
    let turned = r*r/2.*(r0/r).acos()-r0/2.*(r*r-r0*r0).sqrt();
    let groove = TAU*l/lead*(w*(r*r-r0*r0)/2.+lead/PI*turned);
    assert!((screwed_partway_away(21.,8.)-groove).abs() <= 1e-8*groove,"{} against {groove}",screwed_partway_away(21.,8.));
}

/// **The fast path and the one arrangement agree where both build** (#699 D, docs/swept-solid-design.md
/// §8, step 8): `punch.sv`'s drum pushed through its plate is the planar subtraction class's (read
/// from its B-rep it stands along its axis, square to which the push keeps every plane, #721 F1c) —
/// its region carried through the slab, the fast path — and, read in the kernel alone, the closed
/// class's too (`swept::closed::cut_from`, the plate the arrangement's domain). Two builds made
/// apart, one by a swept region, the other by an arrangement and membership: each the plate less
/// the drum's shadow, 672, and the same to 1e-9.
#[test]
fn the_fast_path_and_the_one_arrangement_agree_where_both_build() {
    use gcs_core::brep::trajectory::{Screw,Trajectory};
    let e = fixtures::read(&crate::closed_characteristics::punch());
    assert!(matches!(class(&e,"part"),Class::Subtraction(_)),"the planar subtraction class's");
    let fast = exported(&e,"part");
    let drum = gcs_core::brep::build::prism(&gcs_core::brep::build::disc([6.,0.,-5.],[1.,0.,0.],2.),0.,8.).unwrap();
    let plate = crate::brep::block([0.,-5.,0.],[20.,5.,4.]);
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:16.*std::f64::consts::TAU});
    let cut = gcs_core::brep::swept::closed::cut_from(&plate,&drum,&path,[0.,1.],0.002,1e-6).unwrap();
    let arranged = gcs_core::brep::props::volume(&cut);
    near(fast,672.,1e-9);
    near(arranged,fast,1e-9);
}

/// **A body cut by a sweep of the closed class beside one of another** (#721 F2c5's dispatch):
/// `punch.sv`'s plate punched by its drum straight up (the constant-twist class's) and plunged by
/// `plunge.sv`'s bar aslant partway (the closed class's), the bar moved 3.5 across so the two miss
/// each other. The bar's end face at its start pose stands in the plane x = 6 the slot's end wall
/// lies in, a fitted sheet, met there as one surface with it (#780 C4,
/// `a_fitted_sheet_lying_in_a_candidates_plane_is_one_surface_with_it`). The body is built by the
/// other class's road first, then the closed sweep cut from what it leaves on one arrangement
/// (`brep::sweep`'s `mixed_cut`). Apart from the build, the plate loses the drum's shadow, 128, and
/// the bar's plunge (`plunged_away(8)`).
#[test]
fn a_closed_sweep_is_cut_from_what_another_classs_sweep_leaves() {
    let punch = crate::closed_characteristics::punch();
    let plunge = include_str!("../../examples/solid_sweeps/plunge.sv");
    let bar = &plunge[plunge.find("// the bar: its section").unwrap()..];
    let bar = bar.replace("c0 coords(du: 0mm,","c0 coords(du: 3.5mm,").replace("c1 coords(du: 1mm,","c1 coords(du: 4.5mm,")
        .replace("c2 coords(du: 0mm,","c2 coords(du: 3.5mm,").replace("c3 coords(du: -1mm,","c3 coords(du: 2.5mm,")
        .replace("private up :=","private up2 :=").replace(") up\n",") up2\n").replace("ram :=","ram2 :=")
        .replace("ram slides","ram2 slides").replace("under: ram)","under: ram2)").replace("pocket cut plate","pocket cut part");
    let text = format!("{punch}\n{bar}");
    let e = fixtures::read(&text);
    let a = crate::turns_and_slides::admit(&text).unwrap();
    let classes: Vec<bool> = a.sweeps().iter().map(|s| matches!(s.class,Class::Closed(_))).collect();
    assert!(classes.contains(&true) && classes.contains(&false),"one sweep of each road: {classes:?}");
    near(exported(&e,"part"),800.-128.-plunged_away(8.),1e-9);
}

/// **A fitted sheet lying in a candidate's plane is one surface with it** (#780 C4, A2's
/// coincidence): `punch.sv`'s drum, its ends rounded into its side (the constant twist's, its rounds
/// tori: the plain drum stands along its axis and is the planar subtraction class's, #721 F1c),
/// pushed up through its plate leaves a slot whose walls are one fitted sheet cut into four faces, each
/// end wall a strip of the plane x = 6 or x = 14 its flat end sweeps. A closed sweep cut beside it — `plunge.sv`'s bar, its end face at its start pose in the
/// plane x = 6 (`a_closed_sweep_is_cut_from_what_another_classs_sweep_leaves`) — splits the plate
/// by that plane, and its arrangement reduced (`swept::dump`) is the plate and one block whose face
/// holds it. Read as a near touch, the sheet and the plane were two faces coinciding round the
/// slot's corner edges (`complex`'s "coincide along it"); read as one surface (`ssi::same`: every
/// pole of the sheet within the tolerance of the plane, the sheet within its poles' hull), the
/// end wall is a face of the complex naming both. The plane is the sheet over the sheet's patch
/// alone (`Located::face_holds`): the block's face beside the slot, past the patch, was read on
/// the end wall where the sheet's foot clamps to its edge, and dropped as one with it. The plate
/// splits into the part short of x = 6, 6 × 10 × 4, and the rest less the slot, its corners rounded,
/// `14 × 10 × 4 − 4 (32 − (4 − π)/4)` within 2e-6 at 0.001 mm.
#[test]
fn a_fitted_sheet_lying_in_a_candidates_plane_is_one_surface_with_it() {
    use gcs_core::brep::geom::{Frame,Surface};
    let e = fixtures::read(&crate::closed_characteristics::bull_nosed_punch());
    let tolerance = gcs_core::solid::export::Tolerance::new(0.001).unwrap();
    let plate = crate::turns_and_slides::exported_within(&e,"part",Some(tolerance)).solid;
    let wall = Surface::Plane(Frame::new([6.,0.,0.],[1.,0.,0.],[0.,1.,0.]));
    let sheets: Vec<usize> = (0..plate.faces.len()).filter(|&f| plate.faces[f].surface.kind() == "bspline").collect();
    assert_eq!(sheets.len(),4,"the slot's walls, one fitted sheet cut into four faces");
    let lying: Vec<usize> = sheets.iter().copied().filter(|&f| gcs_core::brep::ssi::same(&plate.faces[f].surface,&wall,1e-6)).collect();
    assert_eq!(lying.len(),1,"one end wall lies in x = 6");
    // a plane beside it, past the tolerance, is not the sheet's
    let aside = Surface::Plane(Frame::new([6.+1e-5,0.,0.],[1.,0.,0.],[0.,1.,0.]));
    assert!(!gcs_core::brep::ssi::same(&plate.faces[lying[0]].surface,&aside,1e-6));
    // the reduced Boolean: the plate and the block whose face holds the bar's end face's plane
    let block = crate::brep::block([-42.,-48.,-49.],[6.,48.,47.]);
    let complex = gcs_core::brep::boolean::arrangement_within(&plate,&[&block],1e-6).unwrap();
    let volumes = crate::complex::volumes(&complex);
    assert_eq!(volumes.len(),2,"{volumes:?}");
    near(volumes[0],240.,1e-9);
    near(volumes[1],560.-4.*(32.-(4.-std::f64::consts::PI)/4.),2e-6);
    // the end wall one face of the complex naming the plate's sheet and the block's face both
    let named = complex.origins.iter().filter(|o| o.len() == 2 && o.iter().any(|x| x.operand == 0 && x.face == lying[0])
        && o.iter().any(|x| x.operand == 1)).count();
    assert_eq!(named,1,"one face of the complex is the end wall and the block's face");
}
