//! **Words choose every root, and a drawing need seed no point** (#217, §9.6, §15): the solve
//! constructs an unseeded drawing a block at a time, each block held to the bounds it decides and
//! its root searched for where it comes to none or to one a bound refuses (`solve::Search`); a
//! drawing is judged at its own size, not the unit size its unseeded points start at
//! (`System::resized`); constants read off the pose are read again where a solve ends, and a
//! chart where a failed one ends; a skew distance is a magnitude whose side a bound chooses; and
//! an angle's bound is held to the solve's own tolerance, not a length's.  The spiral bevel
//! (`spiral_bevel/hypoid.sv`, `hypoid_layout.rs`) is the large case; these are the small ones,
//! each failing without the change it names.
use gcs_core::program::Elaborated;
use gcs_core::solve::solve;

use crate::common::{at, read};

/// Solved to the accuracy the recorded numbers are taken at.
fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, fixtures::accurate());
    assert!(r.success, "{r:?}\n{src}");
    e
}

/// A line at a skew distance from an axis is a magnitude (§9.2): both sides of the axis are roots,
/// and a bound on the line's point chooses, from no seed at all, either way.
#[test]
fn a_skew_distances_side_is_chosen_by_a_bound() {
    let base = "unit mm\nuse std\nuse std.place (skew)\n\
                gax := line(std.origin)\nfix((0, 0, 10)) gax.p2\n\
                a := point\nb := point\nfix(x == 0, z == 3) a\nfix(x == 10, z == 3) b\n\
                a level(std.y) b\npax := line(a, b)\ngax skew(theta: 90deg, e: 5mm) pax\n";
    for (bound, y) in [("std.origin distance(>= 0, along: std.y) a", 5.0),
                       ("std.origin distance(<= 0, along: std.y) a", -5.0)] {
        let e = solved(&format!("{base}{bound}\n"));
        let p = at(&e, "a");
        assert!((p[1] - y).abs() < 1e-9 && (at(&e, "b")[1] - y).abs() < 1e-9, "{bound}: {p:?}");
    }
}

/// A gear's right triangle 500 mm across, every point unseeded — about the origin at the size of
/// a unit, the point in space at the pitch plane's height, where the triangle's two sides of P are
/// a saddle — stands on the sides its words say: M beyond O along P's u, F below P.  Judged at the
/// unit size the start is, the triangle's squared lengths cannot meet 1e-12; judged at its own,
/// they do; and from the saddle the block's root is searched for.
#[test]
fn an_unseeded_triangle_stands_where_its_words_say_at_its_own_size() {
    let src = "unit mm\nuse std\n\
               O := point in std.top\nM := point in std.top\nO coincident std.top.origin\n\
               M level(v) std.top\nM distance(>= 0, along: u) std.top\n\
               foot := point\nto_foot := line(M, foot)\nax := line(O, foot)\n\
               O level(std.y) foot\nfoot inside std.top\nto_foot perpendicular ax\n\
               M distance(400mm) foot\nO distance(300mm) foot\n";
    let e = solved(src);
    let (m, f) = (at(&e, "M"), at(&e, "foot"));
    assert!((m[0] - 500.0).abs() < 1e-9 && m[1].abs() < 1e-9, "{m:?}");
    assert!(f[2] < -1.0 && f[1].abs() < 1e-9, "{f:?}");
    assert!(((f[0] - 180.0).abs() < 1e-9) && ((f[2] + 240.0).abs() < 1e-9), "{f:?}");
}

/// A line held parallel to a held axis is measured across the axis, not across the line as it was
/// seeded (`Parallel3`'s frame): an unseeded line starts square to where it ends, and taken across
/// its own seed its two rows hold on a root that is no root of the drawing's — the line tilted,
/// a cone's apex on it off the axis.  Held across the axis, and with every solve's constants read
/// again where it ends, the line stands up.
#[test]
fn a_line_parallel_to_a_held_axis_stands_along_it() {
    let src = "unit mm\nuse std\nuse std.sets\n\
               C := point in std.top\nq := point in std.top\nfix((10, 20)) C\nfix((30, 20)) q\n\
               ax := line(C)\nax parallel std.z\nC distance(30, along: std.z) ax.p2\n\
               k := std.sets.ConeThrough(ax, q, lean: 20deg)\n";
    let e = solved(src);
    let (p2, apex) = (at(&e, "ax.p2"), at(&e, "k.apex"));
    assert!((p2[0] - 10.0).abs() < 1e-9 && (p2[1] - 20.0).abs() < 1e-9, "{p2:?}");
    assert!((apex[0] - 10.0).abs() < 1e-9 && (apex[1] - 20.0).abs() < 1e-9, "{apex:?}");
    // the cone through q: the apex 20 / tan(20°) below C, where its generator meets the axis
    assert!((apex[2] + 20.0 / 20f64.to_radians().tan()).abs() < 1e-9, "{apex:?}");
}

/// A point on the surface `distance(<= d, side: inside)` reaches into a cone (§6.21: what is not
/// deeper than `d` inside it), the cone's axis 300 mm across the drawing and narrowing along it:
/// the side the point's offset is read on is an angle about the cone's apex, a few 1e-4 radians —
/// under a length's tolerance on a drawing that size read as an angle, so the offset on the cone's
/// other side held.  Held to the solve's own tolerance, the point stands inside the cone by the
/// grow.
#[test]
fn an_angles_bound_is_held_to_the_solves_tolerance() {
    let src = "unit mm\nuse std\nuse std.sets\n\
               C := point in std.top\nq := point in std.top\nfix((0, 0)) C\nfix((300, 0)) q\n\
               ax := line(C)\nax parallel std.z\nC distance(100, along: std.z) ax.p2\n\
               k := std.sets.ConeThrough(ax, q, lean: -10deg)\n\
               grown := { p | p distance(<= 0.5mm, side: inside) k }\n\
               kink := point in std.front\nkink coincident grown.surface\n\
               C distance(5mm, along: ax) kink\nC distance(>= 0mm, along: std.x) kink\n";
    let e = solved(src);
    let p = at(&e, "kink");
    // the cone 5 above C: its radius 300 less 5 tan(10°); the grow reaches 0.5 / cos(10°) further
    // in across the axis's normal plane (the foot held by the grow's square: to 1e-8)
    let want = 300.0 - 5.0 * 10f64.to_radians().tan() - 0.5 / 10f64.to_radians().cos();
    assert!((p[0] - want).abs() < 1e-8 && (p[2] - 5.0).abs() < 1e-9, "{p:?} against {want}");
}

/// A point on an offset surface, unseeded in a view through the cone's axis: its foot's
/// derivatives run along a chart chosen where the point started (`Sketch::choose_charts`), about
/// the origin and so beside the axis, where the surface's normal runs another way than where the
/// solve takes it — a direction no tangent there takes, and the rows cannot hold.  Gauged again
/// where the failed solve ends (`System::recharted`), the point stands on the surface.
#[test]
fn a_chart_is_gauged_where_the_solve_ends() {
    let src = "unit mm\nuse std\nuse std.sets\n\
               C := point in std.top\nq := point in std.top\nfix((0, 0)) C\nfix((50, 0)) q\n\
               ax := line(C)\nax parallel std.z\nC distance(100, along: std.z) ax.p2\n\
               k := std.sets.ConeThrough(ax, q, lean: 20deg)\n\
               grown := { p | p distance(<= 0.5mm, side: inside) k }\n\
               kink := point in std.front\nkink coincident grown.surface\n\
               C distance(-2mm, along: ax) kink\nC distance(>= 0mm, along: std.x) kink\n";
    let e = solved(src);
    let p = at(&e, "kink");
    let want = 50.0 - 2.0 * 20f64.to_radians().tan() - 0.5 / 20f64.to_radians().cos();
    assert!((p[0] - want).abs() < 1e-9 && (p[2] + 2.0).abs() < 1e-9, "{p:?} against {want}");
}
