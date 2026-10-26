//! **Rounds and chamfers at an offset cone's corner** (#207, #145): a region about one axis whose
//! bound is a cone grown or shrunk (`std.sets.Offset`, `distance(<= d, side: inside)`) is still
//! lowered into its meridian where `std.rounds.Rounded` or `std.rounds.Chamfered` meets that bound
//! — the grown cone's generator and its rounded apex are steps of the meridian like any other — so
//! its exact solid is built.  Each is held to the closed form (Pappus over the corner worked out by
//! hand) to 1e-9, and to the region read point by point beside its boundary.  A flat beside a
//! sphere bows off the straight flat, the sum of distances a parabola's arc there, and is lowered
//! so (#196).
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// A point held at the origin, the axis up from it, a cone of half 30° opening up from the
/// origin, the cone grown by 1 (`tip`), a sphere of radius 9 about the origin, a cylinder of
/// radius 3 about the axis and the half-spaces below 10 (`lid`) and above -0.8 (`floor`).
const FRAME: &str = "unit mm\nuse std\nuse std.sets\nuse std.rounds\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
                     t := point hint((0, 0, 1))\nfix((0, 0, 1)) t\nup := line(c, t)\n\
                     funnel := std.sets.Cone(up, half: 30deg)\ntip := std.sets.Offset(funnel, d: 1mm)\n\
                     big := std.sets.Sphere(c, r: 9)\ntube := std.sets.Cylinder(up, r: 3)\n\
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

/// The turn about `c` from `p` to `q`, less than a half turn either way.
fn turn(c: [f64; 2], p: [f64; 2], q: [f64; 2]) -> f64 {
    let bearing = |x: [f64; 2]| (x[1] - c[1]).atan2(x[0] - c[0]);
    (bearing(q) - bearing(p) + PI).rem_euclid(2.0 * PI) - PI
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
                             p inside std.rounds.Rounded(tip, lid, r: 0.5mm) }}\nglass := solid(cup)\n"));
    let ((s, co, ta), g, rho) = (sc(), 1.0, 0.5);
    // the generator grown by g: r = z tan + g / cos; its outward normal (cos, -sin)
    let centre = [(10.0 - rho) * ta + (g - rho) / co, 10.0 - rho];
    let on_cone = [centre[0] + rho * co, centre[1] - rho * s];
    let on_lid = [centre[0], 10.0];
    let want = pappus([0.0, 0.0], &[
        Go::To([g / co, 0.0]),
        Go::To(on_cone),
        Go::Arc { c: centre, turn: turn(centre, on_cone, on_lid) },
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
                             p inside std.rounds.Rounded(tip, big, r: 0.5mm) }}\nb := solid(bead)\n"));
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
        Go::Arc { c: centre, turn: turn(centre, on_cone, on_ball) },
        Go::Arc { c: [0.0, 0.0], turn: turn([0.0, 0.0], on_ball, [0.0, big]) },
        Go::To([0.0, 0.0]),
    ]);
    lowered(&e, "b", want, 9.0, [-0.5, 9.5]);
}

/// The cone shrunk by 1 (at most 1mm inside, its apex 2 further up) leaves the cylinder's material
/// below it: where it meets the cylinder's wall, a convex corner, rounded by a ball of 0.5.
#[test]
fn a_round_where_a_shrunk_cone_meets_a_cylinder() {
    let e = solved(&format!("{FRAME}cleared := {{ q | q distance(<= 1mm, side: inside) funnel }}\n\
                             plug := {{ p | p inside tube; p outside std.top; p inside lid; \
                             p inside cleared; p inside std.rounds.Rounded(cleared, tube, r: 0.5mm) }}\n\
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
        Go::Arc { c: centre, turn: turn(centre, on_wall, on_cone) },
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
                             p inside std.rounds.Rounded(tip, floor, r: 0.2mm) }}\nn := solid(nib)\n"));
    let ((s, co, ta), g, rho, low) = (sc(), 1.0f64, 0.2, -0.8);
    let zc = low + rho;
    let centre = [((g - rho).powi(2) - zc * zc).sqrt(), zc];
    let on_floor = [centre[0], low];
    let on_apex = [centre[0] * g / (g - rho), centre[1] * g / (g - rho)];
    // where the apex's ball leaves for the generator
    let leaves = [g * co, -g * s];
    let want = pappus([0.0, low], &[
        Go::To(on_floor),
        Go::Arc { c: centre, turn: turn(centre, on_floor, on_apex) },
        Go::Arc { c: [0.0, 0.0], turn: turn([0.0, 0.0], on_apex, leaves) },
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
                             p inside std.rounds.Chamfered(tip, lid, la: 0.5mm, lb: 0.5mm) }}\n\
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

/// Beside the sphere the sum of distances bows off the straight flat (#208), and that is what is
/// lowered (#196, [0.73]): a parabola about the sphere's centre, its focus — on it `ρ (1 − sin(30° −
/// ψ)) = 9.5`, `ψ` from the axis, since `cos 30° · d_tip + (9 − ρ) = 0.5` with the tip `1 / cos 30°`
/// out at the apex — between where it reaches the tip (ρ 8.5) and the sphere (ρ 9).  The volume is
/// the walk with the chord in its place (Pappus), less the chord's flux, plus the parabola's.
#[test]
fn a_flat_beside_a_sphere_is_a_parabola_turned() {
    let e = solved(&format!("{FRAME}bead := {{ p | p inside tip; p outside std.top; p inside big; \
                             p inside std.rounds.Chamfered(tip, big, la: 0.5mm, lb: 0.5mm) }}\n\
                             b := solid(bead)\n"));
    let (_, co, _) = sc();
    let k = 1.0 / co;
    // the parabola's radius from the apex, and its derivative, at `ψ`
    let rho = |psi: f64| 9.5 / (1.0 - (PI / 6.0 - psi).sin());
    let drho = |psi: f64| -9.5 * (PI / 6.0 - psi).cos() / (1.0 - (PI / 6.0 - psi).sin()).powi(2);
    let at = |psi: f64| [rho(psi) * psi.sin(), rho(psi) * psi.cos()];
    let psi_of = |rho: f64| PI / 6.0 - (1.0 - 9.5 / rho).asin();
    let (pa, pb) = (psi_of(8.5), psi_of(9.0));
    let (a, b) = (at(pa), at(pb));
    // ∫ r²/2 dz along the parabola from `a` to `b`: eight-point Gauss–Legendre over 16 stretches
    let flux: f64 = (0..16).map(|j| gcs_core::integral::GAUSS8.iter().map(|&(x, w)| {
        let psi = pa + (pb - pa) * (j as f64 + 0.5 + 0.5 * x) / 16.0;
        let dz = drho(psi) * psi.cos() - rho(psi) * psi.sin();
        w * at(psi)[0].powi(2) / 2.0 * dz * (pb - pa) / 32.0
    }).sum::<f64>()).sum();
    let chord = (b[1] - a[1]) / 6.0 * (a[0] * a[0] + a[0] * b[0] + b[0] * b[0]);
    let walk = pappus([0.0, 0.0], &[
        Go::To([k, 0.0]),
        Go::To(a),
        Go::To(b),
        Go::Arc { c: [0.0, 0.0], turn: turn([0.0, 0.0], b, [0.0, 9.0]) },
        Go::To([0.0, 0.0]),
    ]);
    lowered(&e, "b", walk + 2.0 * PI * (flux - chord), 9.5, [-0.5, 9.5]);
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
