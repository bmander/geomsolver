//! **Tumbling tori the tube road refuses, each as plain as it refuses** (#880 T1f): brep-bench's
//! crossed members (`Tumble::crossed`) fuzzed — their ten numbers jittered by a seed, each sweep
//! built by `closed::ring` as the bench's runner builds it, and those built held against the exact
//! sweep at random points (none was built wrong) — and each refusal's class (its words, its numbers
//! dropped) shrunk to the fewest numbers moved off the member and those rounded, the refusal kept.
//! Each is a gate of what is **not yet** built: the day one builds, its test fails, and becomes the
//! test that it builds — checked, and its volume against the exact sweep's. Three have: the ring
//! road's refusals of a whole torus are the tube road's to build (`closed::ring`), held to the
//! sweep's boundary (`tube::faces::held`).
use crate::common::bench::Tumble;

/// The tumble swept, refused, the refusal saying `why`.
fn refused(t: Tumble,why: &str) {
    let clock = std::time::Instant::now();
    match t.ring() {
        Ok(b) => panic!("{t:?} built, {} faces: the refusal ({why}) is gone — make this its build's test",b.faces.len()),
        Err(e) => {
            let said = format!("{e:?}");
            eprintln!("{t:?} refused in {:?}: {said}",clock.elapsed());
            assert!(said.contains(why),"{t:?} refused, but not for {why}: {said}");
        }
    }
}

/// The forward member, its torus's centre moved off `x`.
fn forward_moved(y: f64) -> Tumble { Tumble {centre:[3.,y,2.],..Tumble::crossed(1.)} }

/// **The forward member with its torus moved 0.015 off its plane, built**: refused at the sewing,
/// three faces along the collar's trim beside the pinch's rim (the double curve from the rim ran into
/// a fold at once and was never traced on the sheet), until the sheets' nets were stretched across the
/// near miss's line (#1027) — 14 faces, every mesh vertex within 3e-7 of the sweep's boundary. Moved
/// 0.021 it built before, 14 faces.
#[test]
fn moved_a_hair_is_built() {
    built(forward_moved(0.015),16.5496090184,16.549609);
}

/// **The forward member moved 0.02: a curve fits only to 1.6e-4**, against 4.4e-7 (0.03: 4.2e-5).
#[test]
fn moved_a_hair_a_curve_fits_short_of_the_tolerance() {
    refused(forward_moved(0.02),"a curve fits within");
}

/// **The forward member moved 0.025: a rim crossing not settled**, from (6.031, 1.745) — on the
/// roll's end, at 100°.
#[test]
fn moved_a_hair_a_rim_crossing_is_not_settled() {
    refused(forward_moved(0.025),"a rim crossing from");
}

/// **The reverse member with its torus moved 0.08 along `x`, its hub as far, and its observer's
/// heading 19.3°: a double curve is not joined to its swallowtail point.**
#[test]
fn a_double_curve_not_joined_to_its_swallowtail_point() {
    refused(Tumble {centre:[3.08,0.,2.],hub:2.08,heading:19.3,..Tumble::crossed(-1.)},
        "a double curve not joined to its swallowtail point");
}

/// **The forward member seen from 0.6 off its axis, not 0.7071, rolled on to 109°, built**: its trims
/// left no angle of its sheet `−` free for the windows' cuts — refused so until its cuts were read
/// clear of its points alone and crossed its trims (`faces::crossing_windows`, `cut_across`); then a
/// double curve crossing the sheet's half turn fitted short, until a trim fitted short was fitted again
/// held on a line turning smoothly (`Trim::at_held`); then refused at the sewing, as the forward member
/// moved a hair was, until the sheets' nets were stretched across the near miss's line (#1027). 15
/// faces, every mesh vertex within 3e-7 of the sweep's boundary.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, ~11 s")]
fn the_trims_leaving_no_angle_free_is_built() {
    built(Tumble {height:0.6,roll:[-100.,109.],..Tumble::crossed(1.)},16.8400036559,16.840004);
}

/// **The reverse member leaning 52°, not 45°: three instants touch a meridian whole** — refused
/// at once, the road's reading of a meridian's contact needing one.
#[test]
fn three_instants_touch_a_meridian_whole() {
    refused(Tumble {lean:52.,..Tumble::crossed(-1.)},"instants touch a meridian whole, not one");
}

/// **The forward member moved 0.1 off its plane, turned 0.6 a radian about a hub at 2.09: a switch
/// whose crossing is not on its trim's partner.**
#[test]
fn a_switch_whose_crossing_is_not_on_its_partner() {
    refused(Tumble {centre:[3.,0.1,2.],hub:2.09,ratio:0.6,..Tumble::crossed(1.)},"a switch whose crossing is not on the trim's partner");
}

/// **A tube 0.3 on a torus moved to (3.2, −0.4, 1.8), turned a radian about a hub at 2.3, its
/// observer heading 33° and the reverse's, rolled from −120°: a trim not joined to its corner**,
/// some 1e-4 off the end pose.
#[test]
fn a_trim_not_joined_to_its_corner() {
    refused(Tumble {centre:[3.2,-0.4,1.8],hub:2.3,tube:0.3,ratio:1.,heading:33.,roll:[-120.,100.],..Tumble::crossed(-1.)},
        "a trim not joined to its corner");
}

/// **A tube 0.2 on a torus at (3.1, 0, 2.07), turned 0.6 a radian about a hub at 1.9, leaning 44°,
/// heading 26°, the observer turning back 0.9 a radian, rolled on to 115.6°: a trim ends beside a
/// corner whose partner is no end pose.**
#[test]
fn a_trim_ends_beside_a_corner_its_partner_no_end_pose() {
    refused(Tumble {centre:[3.1,0.,2.07],hub:1.9,tube:0.2,ratio:0.6,lean:44.,heading:26.,observer:-0.9,
        roll:[-100.,115.6],..Tumble::crossed(-1.)},"beside a corner, its partner no end pose");
}

/// **The reverse member's tube 0.3, its torus at (3, 0, 2.2), its hub at 1.6 and its observer's axis
/// 1.0 off: the depth about a swallowtail point is within the tolerance on no disc.**
#[test]
fn the_depth_about_a_swallowtail_point_is_within_the_tolerance_on_no_disc() {
    refused(Tumble {centre:[3.,0.,2.2],hub:1.6,tube:0.3,height:1.,..Tumble::crossed(-1.)},
        "is not within the tolerance on any disc");
}

/// **A forward tumble leaning 66°, its observer turning 1.26 a radian, rolled from −100° to 66.31°:
/// a switch's rim crossing is not settled.** Not shrunk further: each number put back, or rounded
/// at all, it builds or is refused otherwise — its numbers to five places, "a window's edge closing
/// on itself"; to four, "a curve could not" be fitted.
#[test]
fn a_switchs_rim_crossing_is_not_settled() {
    refused(Tumble {centre:[2.632,0.10799512556957258,2.177758435195769],hub:2.21,tube:0.2231681623486219,
        ratio:0.6725618163473035,lean:66.,heading:21.87742494200873,height:0.73,observer:1.26,roll:[-100.,66.31]},
        "its rim crossing not settled");
}

/// The tumble swept, built, every face held to the sweep's boundary (`tube::faces::held`, which the
/// tube road reads before it returns), its volume `want` to 1e-9 — and cadrs's (MIT/Apache, its sweep
/// of a solid by the same two turns) `cadrs` to 1e-6.
fn built(t: Tumble,want: f64,cadrs: f64) {
    let clock = std::time::Instant::now();
    let b = t.ring().unwrap_or_else(|e| panic!("{t:?} refused: {e:?}"));
    let (_,_,_,tol) = t.sweep();
    b.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&b);
    eprintln!("{t:?} built in {:?}: {} faces, volume {v:.10}",clock.elapsed(),b.faces.len());
    assert!((v-want).abs() <= 1e-9,"the volume {v}, pinned {want}");
    assert!((v-cadrs).abs() <= 1e-6,"the volume {v}, cadrs's {cadrs}");
}

/// **A reverse tumble of tube 0.147 turned 0.377 a radian, leaning 58.9°, built by the tube road**:
/// the ring road leaves its boundary open along a sheet's edge, a piece of a torus the boundary
/// beside it (the arrangement's), and the tube road builds what it refuses — 9 faces.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the ring road refuses first, ~25 s")]
fn the_ring_roads_open_boundary_is_built_by_the_tube_road() {
    built(Tumble {centre:[3.23,-0.13,2.26],hub:2.03,tube:0.147,ratio:0.377,lean:58.9,heading:31.8,height:0.82,
        observer:-0.8,roll:[-100.,111.]},8.5463430810,8.546343);
}

/// **A reverse tumble seen from its own axis (`height` 0), turned 0.37 a radian about a hub at 1.87,
/// leaning and heading 37°, built by the tube road**: the ring road's contact sheet fits along the roll
/// only to 0.46, and the tube road builds what it refuses. Built first by the tube road at
/// 18.0825371883 — its folded sheet kept 0.15 inside the solid, a pocket beside a fold hanging from
/// the roll's end missed by the depth's search, every check of its own passed — now its depth held
/// to the grid (`tube::seams`'s `unexplained`) and its faces to the boundary (`held`).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the ring road refuses first, ~30 s")]
fn the_ring_roads_short_contact_sheet_is_built_by_the_tube_road() {
    built(Tumble {centre:[3.4,0.,2.2],hub:1.87,ratio:0.37,lean:37.,heading:37.,height:0.,..Tumble::crossed(-1.)},
        18.0818507296,18.081851);
}

/// **A forward tumble of tube 0.14 on a torus at (3, 0.4, 2), turned 0.58 a radian about a hub at
/// 2.3, leaning 31°, seen from 1.0 off the observer's axis turning 1.356 a radian, rolled from −122°
/// to 137°: a piece of a torus is the boundary facing behind** (the assembly's, past the road).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, refused after some 19 s")]
fn a_piece_of_a_torus_is_the_boundary_facing_behind() {
    refused(Tumble {centre:[3.,0.4,2.],hub:2.3,tube:0.14,ratio:0.58,lean:31.,height:1.,observer:1.356,roll:[-122.,137.],
        ..Tumble::crossed(1.)},"is the boundary, facing behind");
}

/// **A forward tumble of tube 0.27 on a torus at (2.8, 0, 2.28), turned 0.38 a radian about a hub at
/// 2.232, leaning 24°, heading 10°, seen from 0.95 off the observer's axis turning 0.7 a radian,
/// rolled on to 155.3°, built by the tube road**: the ring road's sheet's pieces end nowhere (the
/// arrangement's), and the tube road builds what it refuses — 10 faces.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the ring road refuses first, ~25 s")]
fn the_ring_roads_pieces_ending_nowhere_are_built_by_the_tube_road() {
    built(Tumble {centre:[2.8,0.,2.28],hub:2.232,tube:0.27,ratio:0.38,lean:24.,heading:10.,height:0.95,observer:0.7,
        roll:[-100.,155.3],..Tumble::crossed(1.)},21.4295434731,21.429543);
}

/// **A tube road's build that keeps a sheet inside the solid is caught, and built again right**
/// (`tube::faces::held`, `Tube::completed`): the tumble seen from its own axis, read by the tube road
/// alone — its sheet `+` folds from the roll's end back to it, and a pocket beside the fold (covered
/// by the end pose on one side, by an instant near its own on the other) lies inside cells the
/// depth's bound cleared by nine readings, none of them covered: the search traced nothing on the
/// sheet and the folded sheet was kept, every check of the solid's own passed (volume 18.0825371883).
/// Its faces read against the sweep, one stands 0.15 inside it and the build is refused there; built
/// again with each chart's depth held to the grid at the feature (the crossings no piece crosses
/// seeding the trace), every face is on the boundary, the volume cadrs's to 1e-6.
#[test]
fn a_tube_road_build_keeping_a_sheet_inside_is_caught_and_built_again() {
    use gcs_core::brep::swept::closed::Unbuilt;
    use gcs_core::brep::swept::tube::{Tube,faces};
    let t = Tumble {centre:[3.4,0.,2.2],hub:1.87,ratio:0.37,lean:37.,heading:37.,height:0.,..Tumble::crossed(-1.)};
    let (tool,path,roll,tol) = t.sweep();
    let ring = gcs_core::brep::swept::rings::Ring::read(&tool).unwrap();
    let mut tube = Tube::pinched(&ring,&path,roll,tol,0.25*ring.thickness).unwrap();
    let wrong = faces::solid(&tube,tol).unwrap();
    wrong.check(tol).unwrap();
    assert!(matches!(faces::held(&tube,&wrong,tol),Err(Unbuilt::Wrong {held:true,..})),"a face inside the sweep caught");
    tube.completed = true;
    let b = faces::solid(&tube,tol).unwrap();
    faces::held(&tube,&b,tol).unwrap();
    let v = gcs_core::brep::props::volume(&b);
    assert!((v-18.0818507296).abs() <= 1e-9,"the volume {v}, pinned 18.0818507296");
    assert!((v-18.081851).abs() <= 1e-6,"the volume {v}, cadrs's 18.081851");
}

/// **The fuzzed tumbles: each built right or refused by name, none wrong** (step 6 of
/// `docs/tube-trims-by-membership.md`): the crossed members' ten numbers jittered by a seed — the
/// 52 tumbles the fuzz read, at a twentieth, a tenth, three tenths and the whole of its ranges —
/// each swept as the runner sweeps it. A refusal is a refusal; a panic fails; a build is checked and
/// its point location read against the exact sweep (a point is in it where some instant of the roll
/// holds it within the tube: the tool's distance over the roll sampled and each least refined) at
/// 400 points of its box, none misplaced past a 250th of the tube. The count built only rises: 12 of
/// the 52 when the fuzz was read.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, 52 sweeps built or refused, ~20 min")]
fn the_fuzzed_tumbles_are_built_right_or_refused_by_name() {
    use gcs_core::brep::query::{Located,Place};
    // (the fuzz's generator: a xorshift from the seed, each number jittered by `scale` of its range)
    struct Draw(u64);
    impl Draw {
        fn next(&mut self) -> f64 { self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17; (self.0 >> 11) as f64/(1u64 << 53) as f64 }
        fn range(&mut self,a: f64,b: f64) -> f64 { a+(b-a)*self.next() }
    }
    let fuzzed = |seed: u64,scale: f64| -> Tumble {
        let mut r = Draw(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
        for _ in 0..8 { r.next(); }
        let sign = if r.next() < 0.5 { 1. } else { -1. };
        let b = Tumble::crossed(sign);
        let mut j = |x: f64,w: f64| x+scale*r.range(-w,w);
        let half = j(100.,50.);
        let skew = j(0.,20.);
        Tumble {centre:[j(b.centre[0],0.4),j(b.centre[1],0.4),j(b.centre[2],0.4)],hub:j(b.hub,0.5),tube:j(b.tube,0.1).max(0.02),
            ratio:j(b.ratio,0.35),lean:j(b.lean,25.),heading:j(b.heading,20.),height:j(b.height,0.3),observer:sign*j(1.,0.4),
            roll:[-half+skew,half+skew]}
    };
    let mut runs: Vec<(u64,f64)> = Vec::new();
    for sc in [0.05,0.1] { runs.extend((201..=208).map(|s| (s,sc))) }
    runs.extend((1..=12).map(|s| (s,0.3)));
    runs.extend((101..=124).map(|s| (s,1.)));
    let (mut built,mut refused) = (0,0);
    for (seed,scale) in runs {
        let t = fuzzed(seed,scale);
        let clock = std::time::Instant::now();
        let b = match std::panic::catch_unwind(|| t.ring()) {
            Err(_) => panic!("seed {seed} at {scale}: {t:?} panicked"),
            Ok(Err(e)) => { refused += 1; eprintln!("seed {seed} at {scale}: refused in {:?}: {e:?}",clock.elapsed()); continue }
            Ok(Ok(b)) => b,
        };
        let (_,path,roll,tol) = t.sweep();
        b.check(tol).unwrap_or_else(|e| panic!("seed {seed} at {scale}: {t:?} built, not checked: {e:?}"));
        // the exact sweep's depth at `p`: the tool's distance over the roll, each least refined
        let depth = |p: [f64;3]| -> f64 {
            let at = |s: f64| -> f64 {
                let Ok(m) = path.at(s) else { return f64::INFINITY };
                let q = m.inverse().point(p);
                let (x,y,z) = (q[0]-t.centre[0],q[1]-t.centre[1],q[2]-t.centre[2]);
                (x.hypot(y)-1.).hypot(z)-t.tube
            };
            let [a,z] = roll;
            const N: usize = 400;
            let h = (z-a)/N as f64;
            let vals: Vec<f64> = (0..=N).map(|k| at(a+h*k as f64)).collect();
            let mut best = vals.iter().copied().fold(f64::INFINITY,f64::min);
            for k in 0..=N {
                let (l,r) = (if k > 0 { vals[k-1] } else { f64::INFINITY },if k < N { vals[k+1] } else { f64::INFINITY });
                if vals[k] <= l && vals[k] <= r && vals[k] < best+0.2 {
                    let (mut lo,mut hi) = ((a+h*(k as f64-1.)).max(a),(a+h*(k as f64+1.)).min(z));
                    for _ in 0..60 { let (m1,m2) = (lo+(hi-lo)/3.,hi-(hi-lo)/3.); if at(m1) < at(m2) { hi = m2 } else { lo = m1 } }
                    best = best.min(at(0.5*(lo+hi)));
                }
            }
            best
        };
        let (lo,hi) = b.bounds();
        let located = Located::new(&b,tol);
        let mut draw = Draw(seed ^ 0xDEAD_BEEF_1234_5678);
        let bar = t.tube/250.;
        let points: Vec<[f64;3]> = (0..400).map(|_| std::array::from_fn(|k| draw.range(lo[k],hi[k]))).collect();
        let wrong = gcs_core::brep::util::par::map(&points,|&p| {
            let d = depth(p);
            if d.abs() <= bar { return None }
            let placed = located.solid_place(p);
            (!matches!((placed,d < 0.),(Place::In,true) | (Place::Out,false))).then_some((p,d,placed))
        });
        if let Some((p,d,placed)) = wrong.into_iter().flatten().next() {
            panic!("seed {seed} at {scale}: {t:?} built wrong: {p:?} placed {placed:?}, its depth in the sweep {d:e}")
        }
        built += 1;
        eprintln!("seed {seed} at {scale}: built in {:?}, {} faces, every point placed",clock.elapsed(),b.faces.len());
    }
    eprintln!("the fuzzed tumbles: {built} built, {refused} refused");
    assert!(built >= 12,"{built} built, fewer than the 12 the fuzz read");
}
