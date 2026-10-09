//! The numerics of a curve stated by its energy (#144), on hand-built problems: the shape the
//! Euler–Lagrange flow integrates to, against closed forms, its derivatives against
//! differences, and the verdict.

use gcs_core::extremal::{self, shoot, Ends, Lagrangian, Stop};
use gcs_core::units::Units;
use gcs_core::variational::Extremum;

fn lag(text: &str, maximize: bool) -> Lagrangian {
    Lagrangian::compile(&[(1.0, text)], maximize, Units::default()).unwrap()
}

/// The catenary through (0, 0) and (d, 0) of length `l`: `a` from `2a sinh(d/2a) = l`, and its
/// height at `x`.
fn catenary(d: f64, l: f64) -> impl Fn(f64) -> f64 {
    let (mut lo, mut hi) = (1e-6f64, 1e9f64);
    for _ in 0..400 {
        let mid = (lo * hi).sqrt();
        if 2.0 * mid * (d / (2.0 * mid)).sinh() > l {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let a = (lo * hi).sqrt();
    move |x: f64| a * ((x - d / 2.0) / a).cosh() - a * (d / (2.0 * a)).cosh()
}

/// The largest height error along the shape against `y(x)`.
fn off(lag: &Lagrangian, sh: &shoot::Shape, y: impl Fn(f64) -> f64) -> f64 {
    (0..=200)
        .map(|i| {
            let a = shoot::at(lag, sh, i as f64 / 200.0, true).unwrap();
            (a.z[1] - y(a.z[0])).abs()
        })
        .fold(0.0, f64::max)
}

#[test]
fn a_rope_hangs_as_the_catenary() {
    let l = lag("p.y", false);
    for (d, len) in [(100.0, 150.0), (20.0, 150.0)] {
        let ends = Ends { a: [0.0, 0.0], b: [d, 0.0], len };
        let sh = shoot::solve(&l, &ends, &[], None).expect("a shape");
        let e = off(&l, &sh, catenary(d, len));
        assert!(e < 1e-9, "chord {d}: off the catenary by {e}");
        assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
    }
}

/// Bisection on `f(x) = 0` over `[lo, hi]`, `f(lo)` and `f(hi)` of opposite signs.
fn bisect(mut lo: f64, mut hi: f64, f: impl Fn(f64) -> f64) -> f64 {
    let flo = f(lo);
    for _ in 0..300 {
        let mid = 0.5 * (lo + hi);
        if (f(mid) > 0.0) == (flo > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Dido's strip, 130 long on a shore 100 wide, takes the arc of a circle: `R sin α = 50`,
/// `2Rα = 130`, bulging to the side where the loop back along the shore runs counter-clockwise.
#[test]
fn didos_strip_is_the_circular_arc() {
    let l = lag("(p.x * t.y - p.y * t.x) / 2", true);
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 0.0], len: 130.0 };
    let sh = shoot::solve(&l, &ends, &[], None).expect("a shape");
    let alpha = bisect(1e-6, std::f64::consts::PI - 1e-9, |a| a.sin() / a - 100.0 / 130.0);
    let r = 65.0 / alpha;
    let centre = [50.0, r * alpha.cos()];
    let e = (0..=200)
        .map(|i| {
            let z = shoot::at(&l, &sh, i as f64 / 200.0, true).unwrap().z;
            ((z[0] - centre[0]).hypot(z[1] - centre[1]) - r).abs()
        })
        .fold(0.0, f64::max);
    assert!(e < 1e-9, "off the arc by {e}");
    let below = shoot::at(&l, &sh, 0.5, true).unwrap().z[1];
    assert!(below < 0.0, "the strip bulges below the shore, ccw: {below}");
    // the Lagrangian is the area negated, so its minimum is the area's maximum
    assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
}

/// `maximizes` the height: the rope stands up as the catenary's mirror, its Lagrangian's minimum.
#[test]
fn the_arch_is_the_catenary_turned_over() {
    let l = lag("p.y", true);
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 0.0], len: 150.0 };
    let sh = shoot::solve(&l, &ends, &[], None).expect("a shape");
    let cat = catenary(100.0, 150.0);
    let e = off(&l, &sh, |x| -cat(x));
    assert!(e < 1e-9, "off the arch by {e}");
    assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
}

/// The catenary through `(x1, y1)` and `(x2, y2)` of length `len`: its height at `x`.
fn catenary_through(p: [f64; 2], q: [f64; 2], len: f64) -> impl Fn(f64) -> f64 {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let flat = (len * len - dy * dy).sqrt();
    let a = bisect(1e-6, 1e6, |a| 2.0 * a * (dx / (2.0 * a)).sinh() - flat);
    let x0 = 0.5 * (p[0] + q[0]) - a * (dy / len).atanh();
    let y0 = p[1] - a * ((p[0] - x0) / a).cosh();
    move |x: f64| y0 + a * ((x - x0) / a).cosh()
}

/// A rope draped over a peg above where it would hang: two catenaries meeting at the peg in a
/// corner, here mirror images, each the catenary through its ends of half the length.
#[test]
fn a_peg_makes_a_corner_of_two_catenaries() {
    let l = lag("p.y", false);
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 0.0], len: 150.0 };
    let peg = [50.0, -40.0];
    let sh = shoot::solve(&l, &ends, &[Stop::Peg(peg)], None).expect("a shape");
    assert!((sh.places[1] - 75.0).abs() < 1e-9, "the peg halves the rope: {}", sh.places[1]);
    let left = catenary_through([0.0, 0.0], peg, 75.0);
    let right = catenary_through(peg, [100.0, 0.0], 75.0);
    let e = off(&l, &sh, |x| if x <= 50.0 { left(x) } else { right(x) });
    assert!(e < 1e-9, "off the two catenaries by {e}");
    // the corner: the direction turns at the peg
    let (a, b) = (shoot::at(&l, &sh, 0.5 - 1e-9, true).unwrap(), shoot::at(&l, &sh, 0.5 + 1e-9, true).unwrap());
    assert!((a.theta - b.theta).abs() > 0.1, "a corner at the peg: {} {}", a.theta, b.theta);
    assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
}

/// The sphere seen stereographically: `∫ 2/(1 + |p|²) ds` is length on the unit sphere, so its
/// free-length extremals are great circles — in the plane, circles of centre `c` and radius
/// `√(1 + |c|²)`, through antipodal points `p` and `−p/|p|²`.  From `a = (−1, 0)` round the circle
/// through `(0, −0.24)`, the arc is a minimum until it reaches the antipode `(1, 0)` (53° on) and a
/// saddle past it: the antipode is conjugate to `a`.  Walked there as a drag would walk it, each
/// arc at its own length; each is on the circle and transversal (`H = 0`).
#[test]
fn a_great_circle_is_a_minimum_short_of_the_antipode_and_a_saddle_past_it() {
    let l = lag("2 / (1 + p.x^2 + p.y^2)", false);
    let c = [0.0f64, 2.0];
    let rho = (1.0 + c[1] * c[1]).sqrt();
    let on = |phi: f64| [c[0] + rho * phi.cos(), c[1] + rho * phi.sin()];
    let pa = (-2.0f64).atan2(-1.0);
    let mut sh: Option<shoot::Shape> = None;
    let mut verdicts = Vec::new();
    for deg in (10..=70).step_by(5) {
        let sweep = (deg as f64).to_radians();
        let ends = Ends { a: on(pa), b: on(pa + sweep), len: rho * sweep };
        let next = shoot::solve(&l, &ends, &[], sh.as_ref()).unwrap_or_else(|| panic!("a shape at {deg}°"));
        let e = (0..=100)
            .map(|i| {
                let z = shoot::at(&l, &next, i as f64 / 100.0, true).unwrap().z;
                ((z[0] - c[0]).hypot(z[1] - c[1]) - rho).abs()
            })
            .fold(0.0, f64::max);
        assert!(e < 1e-9, "{deg}°: off the great circle by {e}");
        let h = shoot::at(&l, &next, 1.0, true).unwrap().point.h;
        assert!(h.abs() < 1e-9, "{deg}°: transversal, H = {h}");
        if deg == 40 || deg == 70 {
            verdicts.push(extremal::verdict(&l, &next, true));
        }
        sh = Some(next);
    }
    assert_eq!(verdicts, [Extremum::Minimum, Extremum::Saddle]);
}

/// The shape's derivatives — in `u` and in the ends and the length, the unknowns' dependence by
/// the implicit function theorem — against central differences of solves.
#[test]
fn the_shapes_derivatives_are_its_differences() {
    let l = lag("p.y", false);
    for pegs in [vec![], vec![Stop::Peg([45.0, -40.0])]] {
        let ends = Ends { a: [0.0, 0.0], b: [100.0, 5.0], len: 150.0 };
        let sh = shoot::solve(&l, &ends, &pegs, None).expect("a shape");
        for u in [0.13, 0.6] {
            let at = shoot::at(&l, &sh, u, true).unwrap();
            let o = ends.outer();
            for c in 0..5 {
                let h = 1e-5;
                let z = |sign: f64| {
                    let mut oo = o;
                    oo[c] += sign * h;
                    let s = shoot::solve(&l, &Ends::of(&oo), &pegs, Some(&sh)).unwrap();
                    shoot::at(&l, &s, u, true).unwrap().z
                };
                let (p, m) = (z(1.0), z(-1.0));
                for i in 0..4 {
                    let fd = (p[i] - m[i]) / (2.0 * h);
                    let got = at.dz[i * 6 + 1 + c];
                    assert!((fd - got).abs() <= 1e-5 * (1.0 + fd.abs()), "pegs {pegs:?} u {u} z{i} / o{c}: {got} against {fd}");
                }
            }
            let du = 1e-6;
            let (p, m) = (shoot::at(&l, &sh, u + du, true).unwrap().z, shoot::at(&l, &sh, u - du, true).unwrap().z);
            for i in 0..4 {
                let fd = (p[i] - m[i]) / (2.0 * du);
                assert!((fd - at.dz[i * 6]).abs() <= 1e-5 * (1.0 + fd.abs()), "z{i} / u: {} against {fd}", at.dz[i * 6]);
            }
        }
    }
}

