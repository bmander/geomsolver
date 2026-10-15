//! **The loft series** (`rust/examples/lofts/`, issue #243): each document solved to DOF 0 and
//! each of its solids held to the closed form its comments give, its volume read off the exact
//! B-rep (`props::volume`, its boundary checked by `Brep::check`) to a part in 10⁹ — Pappus along
//! a path, Cavalieri under a twist, frusta and prismatoids where the section's area is quadratic
//! in the running number, and a quadrature exact for its degree where the rail is a cubic.
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parsed::determined;

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// The volume of the solid `name` off its exact B-rep, its boundary checked.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// The solid `name` of `e` within a part in 10⁹ of `want`.
fn holds(e: &Elaborated, name: &str, want: f64) {
    let got = volume(e, name);
    assert!((got - want).abs() <= 1e-9 * want.abs(), "`{name}`: {got} against {want}");
}

/// `∫₀¹ f` by the five-point Gauss–Legendre rule, exact for a polynomial of degree nine.
fn gauss5(f: impl Fn(f64) -> f64) -> f64 {
    let (a, b) = (0.538_469_310_105_683_1, 0.906_179_845_938_664);
    let (wa, wb) = (0.478_628_670_499_366_5, 0.236_926_885_056_189_1);
    let at = |x: f64| f(0.5 + 0.5 * x);
    0.5 * (128.0 / 225.0 * at(0.0) + wa * (at(-a) + at(a)) + wb * (at(-b) + at(b)))
}

/// **`pipe.sv`**: a 4 mm pipe along three straights of 20 and two quarter turns of R25 is its
/// area times the path's length.
#[test]
fn the_pipe_is_its_area_times_its_path() {
    let e = determined(include_str!("../../examples/lofts/pipe.sv"));
    holds(&e, "pipe", 16.0 * PI * (60.0 + 25.0 * PI));
}

/// **`twisted_duct.sv`**: the 20 mm square over 40 is 16000 at a quarter turn, at three and a
/// half turns and eased in and out.
#[test]
fn the_twisted_ducts_are_cavalieri() {
    let e = determined(include_str!("../../examples/lofts/twisted_duct.sv"));
    for name in ["duct", "auger", "eased"] {
        holds(&e, name, 16000.0);
    }
}

/// **`tapered.sv`**: the square shrinking to half as it turns is a square frustum, the disc
/// shrinking to half a cone's, and the disc swelling on a sine its integral.
#[test]
fn the_tapers_are_frusta_and_the_swell_its_integral() {
    let e = determined(include_str!("../../examples/lofts/tapered.sv"));
    holds(&e, "taper", 40.0 / 3.0 * (400.0 + 100.0 + 200.0));
    holds(&e, "cone", PI * 40.0 / 3.0 * 175.0);
    holds(&e, "swell", 4000.0 * PI * (1.0 + 1.2 / PI + 0.045));
}

/// **`thread.sv`**: the square thread is Pappus about its centroid's circle, the flight its area
/// times its rise.
#[test]
fn the_thread_and_the_flight_are_pappus_and_cavalieri() {
    let e = determined(include_str!("../../examples/lofts/thread.sv"));
    holds(&e, "thread", 4.0 * 2.0 * PI * 10.0 * 3.0);
    holds(&e, "flight", 8.0 * 12.0 * 2.5);
}

/// **`spring.sv`**: the wire square to its helix is its area times the helix's length.
#[test]
fn the_spring_is_its_area_times_its_helix() {
    let e = determined(include_str!("../../examples/lofts/spring.sv"));
    let helix = 4.0 * ((20.0 * PI).powi(2) + 25.0).sqrt();
    holds(&e, "spring", PI * 1.5 * 1.5 * helix);
}

/// **`bottle.sv`**: ruled, the sum of its two frusta; smooth, π∫x² dy along the drawn rail, x
/// read from its own axis 80 mm along (`apart`), a quadrature exact on each span (the integrand
/// x²·y′ of a cubic is of degree eight).
#[test]
fn the_bottle_is_its_frusta_and_its_rails_integral() {
    let e = determined(include_str!("../../examples/lofts/bottle.sv"));
    let frustum = |h: f64, a: f64, b: f64| PI * h / 3.0 * (a * a + a * b + b * b);
    holds(&e, "ruled", frustum(60.0, 30.0, 28.0) + frustum(30.0, 28.0, 12.0));
    let rail = index(&e, "rail");
    let smooth: f64 = (0..2).map(|k| gauss5(|u| {
        let f = gcs_core::curve::eval(&e.sketch, rail, k as f64 + u);
        let x = f.p.0 - 80.0;
        PI * x * x * f.d1.1
    })).sum();
    holds(&e, "smooth", smooth);
}

/// **`squares.sv`**: three squares on four rails with no guide are two prismatoids.
#[test]
fn the_squares_with_no_guide_are_prismatoids() {
    let e = determined(include_str!("../../examples/lofts/squares.sv"));
    holds(&e, "body", 14400.0);
}

/// **`two_faces.sv`**: the square to the diamond, and to the diamond walked from its second
/// corner, are their prismatoids.
#[test]
fn two_faces_and_a_connector_are_prismatoids() {
    let e = determined(include_str!("../../examples/lofts/two_faces.sv"));
    holds(&e, "body", 40.0 / 6.0 * 1856.0);
    holds(&e, "turned", 40.0 / 6.0 * (400.0 + 4.0 * 52.0 + 288.0));
}

/// **`triangle_to_square.sv`**: the square to the triangle walked through its left side's middle
/// is its prismatoid.
#[test]
fn a_split_edge_lofts_a_square_to_a_triangle() {
    let e = determined(include_str!("../../examples/lofts/triangle_to_square.sv"));
    holds(&e, "body", 30.0 / 6.0 * (400.0 + 4.0 * 287.5 + 200.0));
}

/// **`pyramid.sv`**: a third of base by height, leaning.
#[test]
fn the_pyramid_is_a_third_of_base_by_height() {
    let e = determined(include_str!("../../examples/lofts/pyramid.sv"));
    holds(&e, "pyramid", 400.0 * 30.0 / 3.0);
}

/// **`blade.sv`**: the 20 × 3 profile scaled by its rails' spread, which closes and turns.
#[test]
fn the_blade_is_its_profile_scaled_by_its_rails() {
    let e = determined(include_str!("../../examples/lofts/blade.sv"));
    holds(&e, "blade", 60.0 * 50.0 / 400.0 * (400.0 - 240.0 + 48.0 + 12.0));
}

/// **`three_rails.sv`**: the 6 mm square carried by the affine map of its frame onto the rails'
/// triangle, its area 36/50 of the triangle's.
#[test]
fn three_rails_carry_a_section_by_its_area_ratio() {
    let e = determined(include_str!("../../examples/lofts/three_rails.sv"));
    holds(&e, "body", 40.0 / 6.0 * 0.72 * (200.0 + 4.0 * 146.875 + 87.5));
}

/// **`square_to_circle.sv`**: the rounded square whose flats run to points is its area's
/// integral, 8000 + 1000π.
#[test]
fn the_square_to_the_circle_is_its_areas_integral() {
    let e = determined(include_str!("../../examples/lofts/square_to_circle.sv"));
    holds(&e, "transition", 8000.0 + 1000.0 * PI);
}
