//! **Rounds and chamfers at an offset cone's corner** (#207, #145): a region about one axis whose
//! bound is a cone grown or shrunk (`std.Offset`, `outside(d)`) is still lowered into its
//! meridian where `std.Rounded` or `std.Chamfered` meets that bound — the grown cone's generator
//! and its rounded apex are steps of the meridian like any other — so its exact solid is built.
//! Each is held to the closed form (Pappus over the corner worked out by hand) to 1e-9, and to the
//! region read point by point beside its boundary.  A flat beside a sphere is not lowered: the
//! sum of distances bows off the straight flat there, so it is read point by point.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// A point held at the origin, the axis up from it, a cone of half 30° opening up from the
/// origin, the cone grown by 1 (`tip`), a sphere of radius 9 about the origin, a cylinder of
/// radius 3 about the axis and the half-spaces below 10 (`lid`) and above -0.8 (`floor`).
const FRAME: &str = "unit mm\nuse std\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
                     t := point hint((0, 0, 1))\nfix((0, 0, 1)) t\nup := line(c, t)\n\
                     funnel := std.Cone(up, half: 30deg)\ntip := std.Offset(funnel, d: 1mm)\n\
                     big := std.Sphere(c, r: 9)\ntube := std.Cylinder(up, r: 3)\n\
                     lid := { q | q distance(<= 10mm, along: n) std.top }\n\
                     floor := { q | q distance(>= -0.8mm, along: n) std.top }\n";

/// A step of a meridian's boundary in `(r, z)`: a line to a point, or an arc about a centre
/// through `turn` radians from where the last step ended.
enum Go {
    To([f64; 2]),
    Arc { c: [f64; 2], turn: f64 },
}

/// The volume a closed walk in the meridian turns out (Pappus, by Green: `2π ∮ r²/2 dz`, the walk
/// anticlockwise), each step integrated in closed form.
fn pappus(start: [f64; 2], walk: &[Go]) -> f64 {
    let mut at = start;
    let mut sum = 0.0;
    for go in walk {
        match *go {
            Go::To(b) => {
                let ([r0, z0], [r1, z1]) = (at, b);
                sum += (z1 - z0) / 6.0 * (r0 * r0 + r0 * r1 + r1 * r1);
                at = b;
            }
            Go::Arc { c: [a, b], turn } => {
                let rho = ((at[0] - a).powi(2) + (at[1] - b).powi(2)).sqrt();
                let p0 = (at[1] - b).atan2(at[0] - a);
                let p1 = p0 + turn;
                // ∫ (a + ρ cos φ)² / 2 · ρ cos φ dφ
                let f = |p: f64| {
                    rho / 2.0 * (a * a * p.sin() + a * rho * (p + (2.0 * p).sin() / 2.0)
                                 + rho * rho * (p.sin() - p.sin().powi(3) / 3.0))
                };
                sum += f(p1) - f(p0);
                at = [a + rho * p1.cos(), b + rho * p1.sin()];
            }
        }
    }
    let closed = (at[0] - start[0]).abs() < 1e-12 && (at[1] - start[1]).abs() < 1e-12;
    assert!(closed, "{at:?} ≠ {start:?}");
    2.0 * PI * sum
}

/// The angle from `c` to `p`.
fn bearing(c: [f64; 2], p: [f64; 2]) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}

/// The turn about `c` from `p` to `q`, less than a half turn either way.
fn turn(c: [f64; 2], p: [f64; 2], q: [f64; 2]) -> f64 {
    (bearing(c, q) - bearing(c, p) + PI).rem_euclid(2.0 * PI) - PI
}

/// Solid `name` lowered into its meridian, its exact volume `want` to 1e-9, and its field the
/// region read point by point's wherever that reads clear of the boundary, about the axis
/// between heights `z0` and `z1` out to `r1`.
fn lowered(e: &Elaborated, name: &str, want: f64, r1: f64, [z0, z1]: [f64; 2]) {
    let i = ent(e, name).i();
    let m = gcs_core::solid::region::meridian(&e.sketch, i);
    assert!(m.is_ok(), "`{name}`: {:?}", m.err());
    assert!(!gcs_core::solid::region::in_general(&e.sketch, i), "{name}");
    let ex = e.sketch.exact_solid(i).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    let got = gcs_core::brep::props::volume(&ex.brep) / ex.mm.powi(3);
    assert!((got - want).abs() <= 1e-9 * want, "`{name}`: {got} against {want}");
    let field = MaterialField::read(&e.sketch, i, 1e-10).unwrap();
    let general = gcs_core::solid::region::general(&e.sketch, i).unwrap();
    let mut k = 0x2545f4914f6cdd1du64;
    let mut next = || {
        k ^= k << 13;
        k ^= k >> 7;
        k ^= k << 17;
        (k >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut seen = [0, 0];
    for _ in 0..4000 {
        let (phi, rho, z) = (next() * 2.0 * PI, r1 * next(), z0 + (z1 - z0) * next());
        let p = [rho * phi.cos(), rho * phi.sin(), z];
        let g = general.value(p);
        if g.abs() > 1e-6 {
            assert_eq!(field.side(p) < 0.0, g < 0.0, "`{name}` at {p:?}: {g}");
            seen[(g < 0.0) as usize] += 1;
        }
    }
    assert!(seen[0] > 100 && seen[1] > 100, "`{name}`: {seen:?}");
}

fn sc() -> (f64, f64, f64) {
    let h = (30f64).to_radians();
    (h.sin(), h.cos(), h.tan())
}

/// The grown cone's generator meets the lid in a convex corner, rounded by a ball of 0.5: the
/// ball's centre 0.5 below the lid and 0.5 within the generator, which the cone grown by 0.5 holds.
#[test]
fn a_round_where_a_grown_cone_meets_a_plane() {
    let e = solved(&format!("{FRAME}cup := {{ p | p inside tip; p outside std.top; p inside lid; \
                             p inside std.Rounded(tip, lid, r: 0.5mm) }}\nglass := solid(cup)\n"));
    let ((s, co, ta), g, rho) = (sc(), 1.0, 0.5);
    // the generator grown by g: r = z tan + g / cos; its outward normal (cos, -sin)
    let centre = [(10.0 - rho) * ta + (g - rho) / co, 10.0 - rho];
    let on_cone = [centre[0] + rho * co, centre[1] - rho * s];
    let on_lid = [centre[0], 10.0];
    let want = pappus([0.0, 0.0], &[
        Go::To([g / co, 0.0]),
        Go::To(on_cone),
        Go::Arc { c: centre, turn: bearing(centre, on_lid) - bearing(centre, on_cone) },
        Go::To([0.0, 10.0]),
        Go::To([0.0, 0.0]),
    ]);
    lowered(&e, "glass", want, 8.0, [-0.5, 10.5]);
}

/// The hypoid's end: the grown cone meets a sphere about its apex, rounded by a ball of 0.5 whose
/// centre is on the sphere shrunk by 0.5 and the cone grown by 0.5.
#[test]
fn a_round_where_a_grown_cone_meets_a_sphere() {
    let e = solved(&format!("{FRAME}bead := {{ p | p inside tip; p outside std.top; p inside big; \
                             p inside std.Rounded(tip, big, r: 0.5mm) }}\nb := solid(bead)\n"));
    let ((s, co, ta), g, rho, big) = (sc(), 1.0, 0.5, 9.0);
    // the line r = z tan + k against the circle r² + z² = R²
    let (k, rr) = ((g - rho) / co, big - rho);
    let (qa, qb, qc) = (1.0 + ta * ta, 2.0 * ta * k, k * k - rr * rr);
    let z = (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
    let centre = [z * ta + k, z];
    let on_cone = [centre[0] + rho * co, centre[1] - rho * s];
    let on_ball = [centre[0] * big / rr, centre[1] * big / rr];
    let want = pappus([0.0, 0.0], &[
        Go::To([g / co, 0.0]),
        Go::To(on_cone),
        Go::Arc { c: centre, turn: bearing(centre, on_ball) - bearing(centre, on_cone) },
        Go::Arc { c: [0.0, 0.0], turn: PI / 2.0 - bearing([0.0, 0.0], on_ball) },
        Go::To([0.0, 0.0]),
    ]);
    lowered(&e, "b", want, 9.0, [-0.5, 9.5]);
}

/// The cone shrunk by 1 (`outside(1mm)`, its apex 2 further up) leaves the cylinder's material
/// below it: where it meets the cylinder's wall, a convex corner, rounded by a ball of 0.5.
#[test]
fn a_round_where_a_shrunk_cone_meets_a_cylinder() {
    let e = solved(&format!("{FRAME}cleared := {{ q | q outside(1mm) funnel }}\n\
                             plug := {{ p | p inside tube; p outside std.top; p inside lid; \
                             p inside cleared; p inside std.Rounded(cleared, tube, r: 0.5mm) }}\n\
                             s := solid(plug)\n"));
    let ((s, co, _), rho, wall) = (sc(), 0.5, 3.0);
    // the generator from (0, 2), material to its right: the signed distance r cos - (z - 2) sin
    let zc = 2.0 + ((wall - rho) * co - rho) / s;
    let centre = [wall - rho, zc];
    let on_wall = [wall, zc];
    let on_cone = [centre[0] - rho * co, centre[1] + rho * s];
    let want = pappus([0.0, 0.0], &[
        Go::To([wall, 0.0]),
        Go::To(on_wall),
        Go::Arc { c: centre, turn: bearing(centre, on_cone) - bearing(centre, on_wall) },
        Go::To([0.0, 2.0]),
        Go::To([0.0, 0.0]),
    ]);
    lowered(&e, "s", want, 3.5, [-0.5, 10.5]);
}

/// Grown, the cone's apex is a ball of radius 1: where a floor at -0.8 meets it, rounded by a
/// ball of 0.2 whose centre is 0.2 above the floor on the apex's ball shrunk by 0.2.
#[test]
fn a_round_where_a_grown_cones_apex_meets_a_plane() {
    let e = solved(&format!("{FRAME}nib := {{ p | p inside tip; p inside floor; p inside lid; \
                             p inside std.Rounded(tip, floor, r: 0.2mm) }}\nn := solid(nib)\n"));
    let ((s, co, ta), g, rho, low) = (sc(), 1.0f64, 0.2, -0.8);
    let zc = low + rho;
    let centre = [((g - rho).powi(2) - zc * zc).sqrt(), zc];
    let on_floor = [centre[0], low];
    let on_apex = [centre[0] * g / (g - rho), centre[1] * g / (g - rho)];
    // where the apex's ball leaves for the generator
    let leaves = [g * co, -g * s];
    let want = pappus([0.0, low], &[
        Go::To(on_floor),
        Go::Arc { c: centre, turn: bearing(centre, on_apex) - bearing(centre, on_floor) },
        Go::Arc { c: [0.0, 0.0], turn: bearing([0.0, 0.0], leaves) - bearing([0.0, 0.0], on_apex) },
        Go::To([10.0 * ta + g / co, 10.0]),
        Go::To([0.0, 10.0]),
        Go::To([0.0, low]),
    ]);
    lowered(&e, "n", want, 8.0, [-1.2, 10.5]);
}

/// Between the grown cone and the lid the flat is straight: through the generator's point 0.5
/// below the lid and the lid's point 0.5 within the generator.
#[test]
fn a_flat_where_a_grown_cone_meets_a_plane() {
    let e = solved(&format!("{FRAME}cup := {{ p | p inside tip; p outside std.top; p inside lid; \
                             p inside std.Chamfered(tip, lid, la: 0.5mm, lb: 0.5mm) }}\n\
                             glass := solid(cup)\n"));
    let ((_, co, ta), g, d) = (sc(), 1.0, 0.5);
    let on_cone = [(10.0 - d) * ta + g / co, 10.0 - d];
    let on_lid = [10.0 * ta + g / co - d / co, 10.0];
    let want = pappus([0.0, 0.0], &[
        Go::To([g / co, 0.0]),
        Go::To(on_cone),
        Go::To(on_lid),
        Go::To([0.0, 10.0]),
        Go::To([0.0, 0.0]),
    ]);
    lowered(&e, "glass", want, 8.0, [-0.5, 10.5]);
}

/// Beside the sphere the sum of distances bows off the straight flat (#208), so a chamfer there
/// is not lowered: the region is read point by point, still chamfered.
#[test]
fn a_flat_beside_a_sphere_is_read_point_by_point() {
    let e = solved(&format!("{FRAME}bead := {{ p | p inside tip; p outside std.top; p inside big; \
                             p inside std.Chamfered(tip, big, la: 0.5mm, lb: 0.5mm) }}\n\
                             b := solid(bead)\n"));
    let i = ent(&e, "b").i();
    let why = gcs_core::solid::region::meridian(&e.sketch, i).unwrap_err();
    assert!(why.contains("bows off the straight flat"), "{why}");
    assert!(gcs_core::solid::region::in_general(&e.sketch, i));
}

/// `rounds/blank.sv`: the shell between the pitch cone grown by 1.5 and shrunk by 2, outside the
/// toe's sphere (12) and inside the heel's (20), the tip rounded by 0.8 at both — exported by the
/// Rust kernel within 10 µm (`gcs-cli`'s `the_rounds_export_and_measure_within_their_tolerance`).
#[test]
fn the_blank_example_is_built_exactly() {
    let e = solved(include_str!("../../examples/rounds/blank.sv"));
    let ((s, co, ta), g, rho) = (sc(), 1.5f64, 0.8);
    let (toe, heel, o) = (12.0f64, 20.0f64, [0.0, 0.0]);
    // where the line r = z tan + k meets the circle about the apex of radius `big`, above it
    let meets = |k: f64, big: f64| {
        let (qa, qb, qc) = (1.0 + ta * ta, 2.0 * ta * k, k * k - big * big);
        let z = (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
        [z * ta + k, z]
    };
    // the root: the cone shrunk by 2, its apex 4 up
    let root = -4.0 * ta;
    let (root_toe, root_heel) = (meets(root, toe), meets(root, heel));
    // each round's centre, 0.8 within the tip and 0.8 within the sphere's side
    let k = (g - rho) / co;
    let (c_toe, c_heel) = (meets(k, toe + rho), meets(k, heel - rho));
    let tip_foot = |c: [f64; 2]| [c[0] + rho * co, c[1] - rho * s];
    let (tip_toe, tip_heel) = (tip_foot(c_toe), tip_foot(c_heel));
    let on_toe = c_toe.map(|x| x * toe / (toe + rho));
    let on_heel = c_heel.map(|x| x * heel / (heel - rho));
    let want = pappus(tip_toe, &[
        Go::To(tip_heel),
        Go::Arc { c: c_heel, turn: turn(c_heel, tip_heel, on_heel) },
        Go::Arc { c: o, turn: turn(o, on_heel, root_heel) },
        Go::To(root_toe),
        Go::Arc { c: o, turn: turn(o, root_toe, on_toe) },
        Go::Arc { c: c_toe, turn: turn(c_toe, on_toe, tip_toe) },
    ]);
    lowered(&e, "blank", want, 12.0, [5.0, 20.5]);
}
