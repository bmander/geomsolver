//! **The sundials** (`examples/sundial/`): each hour line is where the plane through the style and
//! that hour's sun meets the dial, the sun a ring of 24 hourly places about the style
//! (`sun.sv`), and nothing in either document states where a line goes.  The horizontal dial
//! comes out at the dialist's closed form, tan θ = sin φ · tan h (θ from the noon line, φ the
//! latitude, h the hour angle), at every latitude, and its first and last light where the longest
//! day's sunrise puts them; the polar dial, the one for the equator, at `height · tan(h)`.
use std::path::PathBuf;

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::at_path;

fn project() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/sundial")
}

/// `file` of the project with `from` replaced by `to`, read with its module beside it, solved,
/// and held to DOF 0 with nothing said twice.
fn solved(file: &str, from: &str, to: &str) -> Elaborated {
    let text = std::fs::read_to_string(project().join(file)).unwrap();
    assert!(text.contains(from), "{file} says `{from}`");
    let mut e = fixtures::read_beside(&text.replace(from, to), &project(), &mut |_, t| t);
    assert!(e.ok(), "{to}: {:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{to}: {}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well), "{to}");
    assert_eq!(d.n_params, d.n_equations, "{to}: no row says anything twice");
    e
}

/// The bearing of `name`, a point on the dial, from north toward east.
fn bearing(e: &Elaborated, name: &str) -> f64 {
    let [x, y, z] = e.sketch.world_point(at_path(e, name).i());
    assert!(z.abs() < 1e-9, "{name} is on the dial");
    x.atan2(y)
}

/// Two bearings alike, round the circle.
fn alike(got: f64, want: f64) -> bool {
    use std::f64::consts::{PI, TAU};
    ((got - want + PI).rem_euclid(TAU) - PI).abs() < 1e-9
}

#[test]
fn the_horizontal_dials_hour_lines_come_out_where_the_dialists_formula_puts_them() {
    for (latitude, hours) in [(47.6, 7), (10.0, 6), (35.0, 7), (62.0, 9), (66.0, 11)] {
        let e = solved(
            "horizontal.sv",
            "param latitude := 47.6deg\nparam hours := 7",
            &format!("param latitude := {latitude}deg\nparam hours := {hours}"),
        );
        let phi = f64::to_radians(latitude);
        // the hour lines, `12 - hours` o'clock to `12 + hours`, in the quadrant each hour is in
        for i in 0..=2 * hours {
            let h = ((i as f64) - hours as f64) * 15f64.to_radians();
            let want = (phi.sin() * h.sin()).atan2(h.cos());
            let got = bearing(&e, &format!("tip[{i}]"));
            assert!(alike(got, want), "{latitude}°, hour {i}: {got} against {want}");
        }
        // midsummer's sunrise, cos H = -tan φ · tan 23.44°, and the shadow then
        let rise = (-phi.tan() * 23.44f64.to_radians().tan()).acos();
        for (name, h) in [("dawn", -rise), ("dusk", rise)] {
            let want = (phi.sin() * h.sin()).atan2(h.cos());
            let got = bearing(&e, name);
            assert!(alike(got, want), "{latitude}°, {name}: {got} against {want}");
        }
    }
}

#[test]
fn the_polar_dials_hour_lines_are_parallel_at_height_tan_h() {
    for height in [25.0, 40.0] {
        let e = solved("polar.sv", "height := 25mm", &format!("height := {height}mm"));
        // seven o'clock to five
        for i in 0..11 {
            let want = height * (((i as f64) - 5.0) * 15f64).to_radians().tan();
            for end in ["s", "n"] {
                let [x, _, z] = e.sketch.world_point(at_path(&e, &format!("{end}[{i}]")).i());
                assert!(z.abs() < 1e-9, "on the plate");
                assert!((x - want).abs() < 1e-9, "{height}, hour {i}, {end}: {x} against {want}");
            }
        }
    }
}
