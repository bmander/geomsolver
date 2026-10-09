//! **Regions** (§6.21, §9.6; #145 F1, #109): a set whose body bounds a number is a region, and
//! `p inside S` / `p outside S` put a point in or out of one — the set's body applied to the
//! point with its one number read as at most (`inside`) or at least (`outside`) what it says.
//! A bound on the point, so a choice of root: no row.
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::system::System;

use crate::common::{ent, read, refused};

/// A point on two circles about `a` and `b`, five from each — a root at (3, 4) and one at
/// (3, -4) — seeded below, and `c` held above at (3, 5).
const CROSSING: &str = "unit mm\nuse std\nin std.front {\n  a := point hint((0, 0))\n  \
                        b := point hint((6, 0))\n  c := point hint((3, 5))\n  \
                        p := point hint((3, -2))\n}\nfix((0, 0)) a\nfix((6, 0)) b\nfix((3, 5)) c\n\
                        p distance(5) a\np distance(5) b\n";

/// A point in space five from three held points — a root at z = √7 and its mirror at -√7 —
/// seeded below.
const SPACE: &str = "unit mm\nuse std\na := point hint((0, 0, 0))\nb := point hint((6, 0, 0))\n\
                     c := point hint((0, 6, 0))\nfix((0, 0, 0)) a\nfix((6, 0, 0)) b\n\
                     fix((0, 6, 0)) c\np := point hint((3, 3, -2))\np distance(5) a\n\
                     p distance(5) b\np distance(5) c\n";

fn p_at(e: &Elaborated) -> [f64; 3] {
    e.sketch.world_point(ent(e, "p").i())
}

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// Inside a sphere about `c` is the root near it; outside, the other.
#[test]
fn inside_a_sphere_is_within_its_radius() {
    let near = format!("{CROSSING}near := std.Sphere(c, r: 2)\n");
    let e = solved(&format!("{near}p inside near\n"));
    assert!((p_at(&e)[2] - 4.0).abs() < 1e-9, "{:?}", p_at(&e));
    let above = near.replace("hint((3, -2))", "hint((3, 2))");
    let e = solved(&format!("{above}p outside near\n"));
    assert!((p_at(&e)[2] + 4.0).abs() < 1e-9, "{:?}", p_at(&e));
}

/// In space: inside a cylinder about a line above the three points, and inside a cone pointing
/// up from below them, are the upper root; outside, the lower.
#[test]
fn inside_a_cylinder_and_a_cone_in_space() {
    let z = 7f64.sqrt();
    let solids = format!("{SPACE}u := point hint((0, 0, 5))\nv := point hint((1, 0, 5))\n\
                          fix((0, 0, 5)) u\nfix((1, 0, 5)) v\nrod := line(u, v)\n\
                          o := point hint((3, 3, -1))\nt := point hint((3, 3, 0))\n\
                          fix((3, 3, -1)) o\nfix((3, 3, 0)) t\nup := line(o, t)\n\
                          tube := std.Cylinder(rod, r: 4)\nfunnel := std.Cone(up, half: 30deg)\n");
    for (word, set, sign) in [("inside", "tube", 1.0), ("inside", "funnel", 1.0),
                              ("outside", "funnel", -1.0)] {
        let src = if sign < 0.0 { solids.replace("hint((3, 3, -2))", "hint((3, 3, 2))") }
                  else { solids.clone() };
        let e = solved(&format!("{src}p {word} {set}\n"));
        assert!((p_at(&e)[2] - sign * z).abs() < 1e-9, "{word} {set}: {:?}", p_at(&e));
    }
}

/// A cone's angle in space is read straight off its cosine: on the axis it reads nothing, and
/// behind the apex half a turn — never the -α or 2π - α a search along the cosine can find.
#[test]
fn a_cones_angle_reads_unsigned() {
    let src = "unit mm\nuse std\no := point hint((0, 0, 0))\nt := point hint((0, 0, 1))\n\
               fix((0, 0, 0)) o\nfix((0, 0, 1)) t\nup := line(o, t)\n\
               q := point hint((0, 0, 3))\nfix((0, 0, 3)) q\ns := point hint((0, 0, -3))\n\
               fix((0, 0, -3)) s\nw := point hint((3, 0, 3))\nfix((3, 0, 3)) w\n\
               funnel := std.Cone(up, half: 30deg)\nq inside funnel\ns outside funnel\nw inside funnel\n";
    let e = read(src);
    let deg: Vec<f64> = e.sketch.constraints.iter().filter(|c| c.bound.is_some())
        .map(|c| c.reading(&e.sketch).unwrap().to_degrees())
        .collect();
    assert_eq!(deg.len(), 3);
    for (got, want) in deg.iter().zip([0.0, 180.0, 45.0]) {
        assert!((got - want).abs() < 1e-7, "{deg:?}");
    }
}

/// A region written out: `inside` and `coincident` alike put the point in it, and `outside`
/// turns its bound round.
#[test]
fn a_written_region() {
    let region = format!("{CROSSING}ball := {{ q | q distance(<= 2) c }}\n");
    for word in ["inside", "coincident"] {
        let e = solved(&format!("{region}p {word} ball\n"));
        assert!((p_at(&e)[2] - 4.0).abs() < 1e-9, "{word}: {:?}", p_at(&e));
    }
    let above = region.replace("hint((3, -2))", "hint((3, 2))");
    let e = solved(&format!("{above}p outside ball\n"));
    assert!((p_at(&e)[2] + 4.0).abs() < 1e-9, "{:?}", p_at(&e));
}

/// Inside a set adds no row; a set whose body holds a statement besides its number keeps that
/// statement as a row (a disc: its line, here, and the bound).
#[test]
fn inside_adds_only_what_the_body_states_besides_its_number() {
    let count = |src: &str| {
        let e = read(src);
        let sys = System::new(&e.sketch);
        (sys.hard_rows().len(), sys.n_free)
    };
    let plain = count(CROSSING);
    assert_eq!(count(&format!("{CROSSING}near := std.Sphere(c, r: 2)\np inside near\n")), plain);
    // a free point inside a disc on a line: the line's row and a bound
    let disc = "unit mm\nuse std\nin std.front {\n  o := point hint((0, 0))\n  \
                e := point hint((10, 0))\n  q := point hint((20, 1))\n}\nfix((0, 0)) o\n\
                fix((10, 0)) e\nrail := line(o, e)\nslot := { s | s coincident rail; \
                s distance(5) o }\n";
    let (rows, free) = count(disc);
    assert_eq!(count(&format!("{disc}q inside slot\n")), (rows + 1, free));
    let e = solved(&format!("{disc}q inside slot\n"));
    let at = e.sketch.world_point(ent(&e, "q").i());
    assert!(at[2].abs() < 1e-9 && at[0].abs() <= 5.0 + 1e-6, "on the rail, within 5: {at:?}");
}

/// Conjunction is intersection: inside a region of several uses is inside each.  Outside it is
/// where any one fails — a choice of root, which `outside` cannot say.
#[test]
fn a_region_of_regions() {
    let both = format!("{CROSSING}near := std.Sphere(c, r: 2)\nfar := std.Sphere(a, r: 1)\n\
                        between := {{ q | q inside near; q outside far }}\n");
    let e = solved(&format!("{both}p inside between\n"));
    assert!((p_at(&e)[2] - 4.0).abs() < 1e-9, "{:?}", p_at(&e));
    assert_eq!(e.sketch.constraints.iter().filter(|c| c.bound.is_some()).count(), 2);
    refused(&format!("{both}p outside between\n"), "E040", "choice of root", "outside");
}

/// What `inside` and `outside` may be said of.
#[test]
fn a_region_is_said_of_one_number() {
    refused(&format!("{CROSSING}band := {{ q | q distance(in: (1, 3)) c }}\np outside band\n"),
            "E040", "either side", "outside");
    refused(&format!("{CROSSING}lens := {{ q | q distance(5) a; q distance(5) b }}\n\
                      p inside lens\n"), "E040", "states 2 numbers", "inside");
    refused(&format!("{CROSSING}near := std.Sphere(c, r: 2)\nl := line(a, b)\nl inside near\n"),
            "E040", "puts a point on it", "inside");
    refused(&format!("{CROSSING}ball := {{ q | q distance(<= 2) c }}\nl := line(a, b)\n\
                      l tangent ball\n"), "E040", "is a region", "tangent");
    refused(&format!("{CROSSING}up := line(a, c)\nfunnel := std.Cone(up, half: 30deg)\n\
                      p inside funnel\n"), "E040", "in space", "inside");
}


/// A point in a region is a bound on it: a program lifted from the drawing says the bound the
/// region put there, and the document keeps it.
#[test]
fn a_region_use_is_kept_as_its_bound() {
    let e = solved(&format!("{CROSSING}near := std.Sphere(c, r: 2)\np inside near\n"));
    let lifted = gcs_core::program::to_program(&e.sketch).text().to_string();
    assert!(lifted.contains("distance(<= 2mm)"), "{lifted}");
    let doc = gcs_core::io::from_json(&gcs_core::io::to_json(&e.sketch)).unwrap();
    assert_eq!(doc.constraints.iter().filter(|c| c.bound.is_some()).count(), 1);
}

/// No path touches a region: a plane at a point, as a line is refused above.
#[test]
fn a_plane_touches_no_region() {
    refused(&format!("{CROSSING}ball := {{ q | q distance(<= 2) c }}\n\
                      P := plane(u: std.x, v: std.y)\nP tangent(at: c) ball\n"),
            "E040", "is a region", "tangent(at: c)");
}
