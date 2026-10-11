//! **A region in general position** (§6.21, #183): a region solid whose terms share no axis is read
//! point by point by the field (`solid::field::RegionField`), its boundary the field's surface,
//! where one whose terms share an axis is still its meridian turned.  Held to a closed form.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{ent, read};

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// The origin, and lines from it along x and z.
const AXES: &str = "unit mm\nuse std\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
                    xe := point hint((1, 0, 0))\nfix((1, 0, 0)) xe\nze := point hint((0, 0, 1))\n\
                    fix((0, 0, 1)) ze\nxl := line(c, xe)\nzl := line(c, ze)\n";

/// The volume where `field` is material within the box `lo`–`hi`: an octree pruned by the field's
/// one-Lipschitz value, each last cell (`h` across) read at a point placed in it by a hash of where
/// it is (unbiased; a centre misses a cusp thinner than half a cell along a face the cells line up
/// with).
pub fn field_volume(field: &MaterialField, lo: [f64; 3], hi: [f64; 3], h: f64) -> f64 {
    fn cell(f: &MaterialField, lo: [f64; 3], side: f64, h: f64) -> f64 {
        let c = [lo[0] + side / 2.0, lo[1] + side / 2.0, lo[2] + side / 2.0];
        let v = f.side(c);
        let reach = side * 3f64.sqrt() / 2.0;
        if v < -reach {
            return side * side * side;
        }
        if v > reach {
            return 0.0;
        }
        if side <= h * 1.5 {
            let mut k = lo.iter().fold(0x9e37_79b9_7f4a_7c15u64, |k, x| {
                (k ^ x.to_bits()).wrapping_mul(0x100_0000_01b3).rotate_left(29)
            });
            let mut next = || {
                k = k.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (k >> 11) as f64 / (1u64 << 53) as f64
            };
            let q = [lo[0] + side * next(), lo[1] + side * next(), lo[2] + side * next()];
            return if f.side(q) < 0.0 { side * side * side } else { 0.0 };
        }
        let half = side / 2.0;
        (0..8).map(|m| {
            let q = [lo[0] + (m & 1) as f64 * half, lo[1] + (m >> 1 & 1) as f64 * half,
                     lo[2] + (m >> 2 & 1) as f64 * half];
            cell(f, q, half, h)
        }).sum()
    }
    let side = (0..3).map(|k| hi[k] - lo[k]).fold(0.0f64, f64::max);
    let n = |k: usize| ((hi[k] - lo[k]) / side * 4.0).ceil() as usize;
    let step = side / 4.0;
    let mut total = 0.0;
    for i in 0..n(0) {
        for j in 0..n(1) {
            for k in 0..n(2) {
                let q = [lo[0] + i as f64 * step, lo[1] + j as f64 * step, lo[2] + k as f64 * step];
                total += cell(field, q, step, h);
            }
        }
    }
    total
}

/// Steinmetz's bicylinder: two cylinders of radius 2 crossing square, bounded by the two together
/// though neither is — read point by point.
#[test]
fn two_crossing_cylinders_are_a_bicylinder() {
    let e = solved(&format!("{AXES}a := std.Cylinder(xl, r: 2)\nb := std.Cylinder(zl, r: 2)\n\
                             both := {{ p | p inside a; p inside b }}\nbi := solid(both)\n"));
    let bi = ent(&e, "bi").i();
    assert!(gcs_core::solid::region::in_general(&e.sketch, bi));
    let field = MaterialField::read(&e.sketch, bi, 1e-10).unwrap();
    for (p, inside) in [([0.0, 1.0, 0.0], true), ([1.9, 0.0, 0.0], true), ([0.0, 1.5, 1.5], false),
                        ([1.5, 0.0, 1.5], true), ([0.0, 2.1, 0.0], false), ([0.0, 0.0, 1.9], true)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}");
    }
    let want = 16.0 * 8.0 / 3.0;
    let got = field_volume(&field, [-2.0; 3], [2.0; 3], 2.0 / 64.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
    assert!(e.sketch.exact_solid(bi).is_err_and(|m| m.contains("read by the field")));
}

/// A ball of radius 5 within two planes square to each other, 3 from its centre: the ball less two
/// caps, plus what the caps share, by quadrature.  Meshed, its surface's volume too.
#[test]
fn a_ball_within_two_planes() {
    let e = solved(&format!("{AXES}x3 := axis hint(dir: (0, 1, 0))\ny3 := axis hint(dir: (0, 0, 1))\n\
                             fix(dir == (0, 1, 0), origin == (3, 0, 0)) x3\n\
                             fix(dir == (0, 0, 1), origin == (3, 0, 0)) y3\n\
                             at_x := plane(u: x3, v: y3)\n\
                             zz := axis hint(dir: (0, 0, 1))\nxx := axis hint(dir: (1, 0, 0))\n\
                             fix(dir == (0, 0, 1), origin == (0, 3, 0)) zz\n\
                             fix(dir == (1, 0, 0), origin == (0, 3, 0)) xx\n\
                             at_y := plane(u: zz, v: xx)\nball := std.Sphere(c, r: 5)\n\
                             cut := {{ p | p inside ball; p inside at_x; p inside at_y }}\n\
                             s := solid(cut)\n"));
    let s = ent(&e, "s").i();
    assert!(gcs_core::solid::region::in_general(&e.sketch, s));
    let (r, a) = (5.0f64, 3.0f64);
    let cap = std::f64::consts::PI * (r - a).powi(2) * (3.0 * r - (r - a)) / 3.0;
    // what both caps hold: x, y > 3 within the ball, `2 √(25 − x² − y²)` high
    let n = 4000;
    let shared: f64 = (0..n).map(|i| {
        let x = a + (r - a) * (i as f64 + 0.5) / n as f64;
        let top = (r * r - x * x).max(0.0).sqrt();
        if top <= a { return 0.0; }
        // ∫ 2 √(R² − y²) dy from 3 to top, R² = 25 − x²
        let big = r * r - x * x;
        let prim = |y: f64| y * (big - y * y).max(0.0).sqrt() + big * (y / big.sqrt()).clamp(-1.0, 1.0).asin();
        (prim(top) - prim(a)) * (r - a) / n as f64
    }).sum();
    let want = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3) - 2.0 * cap + shared;
    let field = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
    let got = field_volume(&field, [-5.0; 3], [5.0; 3], 5.0 / 64.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A ball whose centre stands off a cylinder's line: the cylinder taken out of it, not coaxial, so
/// read point by point; and a ball on the line, coaxial, still its meridian.
#[test]
fn a_ball_bored_off_its_centre() {
    let e = solved(&format!("{AXES}o := point hint((2, 0, 0))\nfix((2, 0, 0)) o\n\
                             ball := std.Sphere(o, r: 5)\nbore := std.Cylinder(zl, r: 1)\n\
                             bored := {{ p | p inside ball; p outside bore }}\nb := solid(bored)\n\
                             on := std.Sphere(c, r: 5)\nkept := {{ p | p inside on; p outside bore }}\n\
                             k := solid(kept)\n"));
    let (b, k) = (ent(&e, "b").i(), ent(&e, "k").i());
    assert!(gcs_core::solid::region::in_general(&e.sketch, b));
    assert!(!gcs_core::solid::region::in_general(&e.sketch, k));
    let field = MaterialField::read(&e.sketch, b, 1e-10).unwrap();
    for (p, inside) in [([0.0, 0.0, 0.0], false), ([0.0, 0.0, 3.5], false), ([1.5, 0.0, 0.0], true),
                        ([6.5, 0.0, 0.0], true), ([7.5, 0.0, 0.0], false), ([2.0, 4.0, 0.0], true)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}");
    }
}

/// A region bounded in no direction it can be read along has no end, whichever way it is read.
#[test]
fn a_region_with_no_end_is_refused() {
    let src = format!("{AXES}a := std.Cylinder(xl, r: 2)\nb := std.Cylinder(xl, r: 1)\n\
                       pipe := {{ p | p inside a; p outside b }}\nt := solid(pipe)\n");
    let e = solved(&src);
    let t = ent(&e, "t").i();
    assert!(gcs_core::solid::validate(&e.sketch, t).is_err_and(|m| m.contains("no end")));
}
