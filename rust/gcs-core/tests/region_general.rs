//! **A region in general position** (§6.21, #183): a region solid whose terms share no axis is read
//! point by point by the field (`solid::field::RegionField`), its boundary the field's surface,
//! where one whose terms share an axis is still its meridian turned.  Held to a closed form.
use gcs_core::solid::MaterialField;

use crate::common::{ent, text::solved};

/// The origin, and lines from it along x and z.
const AXES: &str = "unit mm\nuse std\nuse std.sets\nc := point hint((0, 0, 0))\nfix((0, 0, 0)) c\n\
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
    let [nx, ny, nz] = [n(0), n(1), n(2)];
    // the cells side by side, summed in order: the same total as one walk over them
    gcs_core::par::indices(nx * ny * nz, |c| {
        let (i, j, k) = (c / (ny * nz), c / nz % ny, c % nz);
        let q = [lo[0] + i as f64 * step, lo[1] + j as f64 * step, lo[2] + k as f64 * step];
        cell(field, q, step, h)
    })
    .into_iter()
    .fold(0.0, |total, v| total + v)
}

/// Steinmetz's bicylinder: two cylinders of radius 2 crossing square, bounded by the two together
/// though neither is — read point by point.
#[test]
fn two_crossing_cylinders_are_a_bicylinder() {
    let e = solved(&format!("{AXES}a := std.sets.Cylinder(xl, r: 2)\nb := std.sets.Cylinder(zl, r: 2)\n\
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
    // and built exactly as the common of the two (#238, `tests/region_exact.rs`)
    assert!(e.sketch.exact_solid(bi).is_ok());
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
                             at_y := plane(u: zz, v: xx)\nball := std.sets.Sphere(c, r: 5)\n\
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
                             ball := std.sets.Sphere(o, r: 5)\nbore := std.sets.Cylinder(zl, r: 1)\n\
                             bored := {{ p | p inside ball; p outside bore }}\nb := solid(bored)\n\
                             on := std.sets.Sphere(c, r: 5)\nkept := {{ p | p inside on; p outside bore }}\n\
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
    let src = format!("{AXES}a := std.sets.Cylinder(xl, r: 2)\nb := std.sets.Cylinder(xl, r: 1)\n\
                       pipe := {{ p | p inside a; p outside b }}\nt := solid(pipe)\n");
    let e = solved(&src);
    let t = ent(&e, "t").i();
    assert!(gcs_core::solid::validate(&e.sketch, t).is_err_and(|m| m.contains("no end")));
}

/// **A bound on measurements** (#183 F3): an ellipsoid of revolution by its foci, the points whose
/// distances from the two sum to at most 10 — semi-axes 5 and 4 about foci 3 from its centre.  Its
/// foci on one line, its meridian is an ellipse (#238), and the field reads that turned.
#[test]
fn an_ellipsoid_by_its_foci() {
    let e = solved(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                             f2 := point hint((-3, 0, 0))\nfix((-3, 0, 0)) f2\n\
                             egg := {{ p | distance(p, f1) + distance(p, f2) <= 10mm }}\n\
                             s := solid(egg)\n"));
    let s = ent(&e, "s").i();
    assert!(!gcs_core::solid::region::in_general(&e.sketch, s));
    let field = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
    for (p, inside) in [([4.9, 0.0, 0.0], true), ([5.1, 0.0, 0.0], false), ([0.0, 3.9, 0.0], true),
                        ([0.0, 0.0, 4.1], false), ([3.0, 2.0, 2.0], true), ([4.0, 2.0, 2.0], false)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}");
    }
    let want = 4.0 / 3.0 * std::f64::consts::PI * 5.0 * 4.0 * 4.0;
    let got = field_volume(&field, [-5.0, -4.0, -4.0], [5.0, 4.0, 4.0], 5.0 / 64.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A Voronoi cell within a ball: the points of the ball nearer one centre than the other, the
/// half of it on that centre's side; and the bound turned round, the other half.
#[test]
fn a_voronoi_cell_is_half_a_ball() {
    let e = solved(&format!("{AXES}a := point hint((2, 0, 0))\nfix((2, 0, 0)) a\n\
                             b := point hint((-2, 0, 0))\nfix((-2, 0, 0)) b\n\
                             ball := std.sets.Sphere(c, r: 5)\n\
                             near_a := {{ p | p inside ball; distance(p, a) <= distance(p, b) }}\n\
                             near_b := {{ p | p inside ball; distance(p, a) >= distance(p, b) }}\n\
                             sa := solid(near_a)\nsb := solid(near_b)\n"));
    let half = 2.0 / 3.0 * std::f64::consts::PI * 125.0;
    for (name, x) in [("sa", 1.0), ("sb", -1.0)] {
        let s = ent(&e, name).i();
        let field = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
        assert!(field.side([x, 0.5, 0.5]) < 0.0 && field.side([-x, 0.5, 0.5]) > 0.0, "{name}");
        let got = field_volume(&field, [-5.0; 3], [5.0; 3], 5.0 / 64.0);
        assert!((got - half).abs() < 2e-3 * half, "{name}: {got} against {half}");
    }
}

/// A bound on measurements stands only where a region solid's point is measured, and only as a
/// sum of measurements times numbers.
#[test]
fn a_bound_on_measurements_is_refused_off_a_region_and_off_a_sum() {
    crate::common::refused(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                                     egg := {{ p | distance(p, f1) * distance(p, c) <= 10mm }}\n\
                                     s := solid(egg)\n"),
                           "E040", "two measured are not multiplied", "s := solid(egg)");
    crate::common::refused(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                                     distance(c, f1) <= 10mm\n"),
                           "E040", "measures nothing of a region solid's point",
                           "distance(c, f1) <= 10mm");
}


/// **A bound on measurements is parsed once** (#190): each side a tree whose measurements name
/// what they measure by reference — a labelled one (`at:`) included — and whatever measures
/// nothing one number's text, however much arithmetic or how many calls it is.
#[test]
fn a_bound_on_measurements_is_parsed_into_references() {
    use gcs_core::expr::{Measure, Op};
    use gcs_core::syntax::{Measuring, StmtKind};
    let src = "2 * distance(p, f.a) + angle(b.east, b.south, at: p) / 2 <= max(8mm, 10mm) - 3 1/2\n\
               -distance(p, q) >= (1mm + 2mm) ** 2\n";
    let (prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let bounds: Vec<_> = prog.stmts().filter_map(|st| match &st.kind {
        StmtKind::Measured(m) => Some(m),
        _ => None,
    }).collect();
    let [one, two] = bounds.as_slice() else { panic!("two bounds: {bounds:?}") };
    let Measuring::Bin(Op::Add, a, b) = &one.lhs else { panic!("{:?}", one.lhs) };
    let Measuring::Bin(Op::Mul, k, d) = &**a else { panic!("{a:?}") };
    assert!(matches!(&**k, Measuring::Number { text, .. } if text == "2"));
    let Measuring::Measure(d) = &**d else { panic!("{d:?}") };
    assert_eq!(d.measure, Measure::Distance);
    assert_eq!((d.args.len(), d.text()), (2, "distance(p, f.a)".to_string()));
    let Measuring::Bin(Op::Div, x, _) = &**b else { panic!("{b:?}") };
    let Measuring::Measure(x) = &**x else { panic!("{x:?}") };
    assert_eq!(x.measure, Measure::Angle);
    assert_eq!(x.args.len(), 2);
    assert_eq!(x.at.as_ref().map(|r| r.root.text.as_str()), Some("p"));
    assert_eq!(x.text(), "angle(b.east, b.south, at: p)");
    // a call over numbers, a mixed fraction: one number, as written
    assert!(matches!(&one.rhs, Measuring::Number { text, .. } if text == "max(8mm, 10mm) - 3 1/2"));
    assert!(matches!(&two.lhs, Measuring::Neg(_)));
    assert!(matches!(&two.rhs, Measuring::Number { text, .. } if text == "(1mm + 2mm) ** 2"));
    // and printed back as it reads
    assert_eq!(one.lhs.text(), "2 * distance(p, f.a) + angle(b.east, b.south, at: p) / 2");
    assert_eq!(two.lhs.text(), "-distance(p, q)");
}

/// A measurement takes names, the one label `at:` on an angle, and nothing else.
#[test]
fn a_measurement_takes_names_and_one_label() {
    for (src, needle) in [
        ("distance(p, at: q) <= 1\n", "takes no `at:`"),
        ("angle(a, b, from: p) <= 1\n", "takes no `from:`"),
        ("distance(p, 2) <= 1\n", "its arguments are names"),
        ("distance(p, q, r) <= 1\n", "takes 2 argument(s)"),
        ("distance(p) <= 1\n", "takes 2 argument(s)"),
    ] {
        let (_, errs) = gcs_core::syntax::parse(src);
        assert!(errs.iter().any(|e| e.message.contains(needle)), "{src}: {errs:?}");
    }
}

/// The ellipsoid again, its sides written with a call and parentheses: the same region.
#[test]
fn a_bound_reads_its_numbers_through_calls() {
    let e = solved(&format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                             f2 := point hint((-3, 0, 0))\nfix((-3, 0, 0)) f2\n\
                             half := 5mm\n\
                             egg := {{ p | (distance(p, f1) + distance(p, f2)) / 2 <= \
                             max(half, 4mm) }}\n\
                             s := solid(egg)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "s").i(), 1e-10).unwrap();
    for (p, inside) in [([4.9, 0.0, 0.0], true), ([5.1, 0.0, 0.0], false), ([0.0, 3.9, 0.0], true),
                        ([0.0, 0.0, 4.1], false)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}");
    }
}

/// **A hidden key is no name a source may write** (#190): the region's point is `s#probe` to the
/// flattener, which a measurement reaches by reference, and neither a measurement's argument nor
/// a number may spell it.
#[test]
fn a_hidden_key_is_not_writable_in_a_bound() {
    let head = format!("{AXES}f1 := point hint((3, 0, 0))\nfix((3, 0, 0)) f1\n\
                        egg := {{ p | distance(p, f1) <= 10mm }}\ns := solid(egg)\n");
    let (_, errs, _) = gcs_core::library::parse_linked(&format!(
        "{head}near := {{ q | distance(q, s#probe) <= 1mm }}\n"));
    assert!(!errs.is_empty(), "a measurement's argument spelled a hidden key");
    crate::common::refused(&format!("{head}near := {{ q | distance(q, f1) <= s#probe.x }}\n\
                                     t := solid(near)\n"),
                           "E040", "unexpected `#probe.x`", "t := solid(near)");
}
