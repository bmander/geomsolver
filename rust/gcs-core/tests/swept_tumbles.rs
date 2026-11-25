//! **Tumbling tori the tube road refuses, each as plain as it refuses** (#880 T1f): brep-bench's
//! crossed members (`Tumble::crossed`) fuzzed — their ten numbers jittered by a seed, each sweep
//! built by `closed::ring` as the bench's runner builds it, and those built held against the exact
//! sweep at random points (none was built wrong) — and each refusal's class (its words, its numbers
//! dropped) shrunk to the fewest numbers moved off the member and those rounded, the refusal kept.
//! Each is a gate of what is **not yet** built: the day one builds, its test fails, and becomes the
//! test that it builds — checked, and its volume against the exact sweep's.
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

/// **The forward member with its torus moved 0.015 off its plane: three faces meet along an edge**
/// when the faces are sewn, near (3.34, 1.76, 1.51). Moved 0.021 it builds, 14 faces.
#[test]
fn moved_a_hair_three_faces_meet_along_an_edge() {
    refused(forward_moved(0.015),"three or more faces meet along the edge");
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

/// **The forward member seen from 0.6 off its axis, not 0.7071, rolled on to 109°: the trims leave
/// no angle of a sheet free for the windows' cuts.**
#[test]
fn the_trims_leave_no_angle_free_for_the_windows_cuts() {
    refused(Tumble {height:0.6,roll:[-100.,109.],..Tumble::crossed(1.)},"free for the windows' cuts");
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

/// **A reverse tumble of tube 0.147 turned 0.377 a radian, leaning 58.9°: the boundary is open along
/// a sheet's edge**, a piece of a torus the boundary beside it (the Boolean's assembly, past the road).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, refused after some 19 s")]
fn the_boundary_is_open_along_a_sheets_edge() {
    refused(Tumble {centre:[3.23,-0.13,2.26],hub:2.03,tube:0.147,ratio:0.377,lean:58.9,heading:31.8,height:0.82,
        observer:-0.8,roll:[-100.,111.]},"the boundary is open along a bspline edge");
}

/// **A reverse tumble seen from its own axis (`height` 0), turned 0.37 a radian about a hub at 1.87,
/// leaning and heading 37°: a ring's contact's sheet fits along the roll only to 0.46** (the ring
/// road's, not the tube's).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, refused after some 29 s")]
fn a_rings_contacts_sheet_fits_short_along_the_roll() {
    refused(Tumble {centre:[3.4,0.,2.2],hub:1.87,ratio:0.37,lean:37.,heading:37.,height:0.,..Tumble::crossed(-1.)},
        "a ring's contact's sheet fits within");
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
/// rolled on to 155.3°: a sheet's pieces end nowhere** (the assembly's, past the road).
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, refused after some 17 s")]
fn a_sheets_pieces_end_nowhere() {
    refused(Tumble {centre:[2.8,0.,2.28],hub:2.232,tube:0.27,ratio:0.38,lean:24.,heading:10.,height:0.95,observer:0.7,
        roll:[-100.,155.3],..Tumble::crossed(1.)},"a bspline's pieces end nowhere");
}
