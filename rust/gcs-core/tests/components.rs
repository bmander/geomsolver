//! A component's entity formal is a *port alias* (Solvent §8), and an alias is reached the way
//! every other name is: through the prefixes a statement is nested in.  Issue #43 found the
//! three places the old bare-name key fell short, and each is held here — a formal read from a
//! `repeat` in the component's own body, a formal forwarded into a nested instance, and an
//! instance inside a block, whose copies must each bind their own actual.  The third drew the
//! wrong thing without a word: three copies of one constraint on one point, reported redundant.

use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::Sketch;

fn drawn(src: &str) -> Sketch {
    let (prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect::<Vec<_>>()
    );
    let mut sk = e.sketch;
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "does not solve: {}", r.message);
    sk
}

#[test]
fn a_formal_is_visible_inside_a_repeat_in_the_body() {
    let sk = drawn(
        "component Fan(hub: point) {
           repeat 3 as i {
             point tip hint(x: 20 + i, y: i * 5)
             hub distance(10) tip
           }
         }
         point h hint(x: 0, y: 0)
         f: Fan(h)
         ground h",
    );
    let d = diagnose(&mut sk.clone(), DiagnoseOptions::default());
    assert_eq!(sk.points.len(), 4);
    assert_eq!(d.structural_rank, 3, "three spokes, three independent lengths");
    assert!(d.over.is_empty(), "over: {:?}", d.over);
    let (hx, hy) = sk.point_xy(0);
    for i in 1..4 {
        let (x, y) = sk.point_xy(i);
        assert!(((x - hx).hypot(y - hy) - 10.0).abs() < 1e-6, "spoke {i}");
    }
}

#[test]
fn a_formal_is_forwarded_into_a_nested_instance() {
    let sk = drawn(
        "component Inner(p: point) {
           point z hint(x: 10, y: 0)
           p distance(20) z
         }
         component Outer(q: point) {
           i: Inner(q)
         }
         point a hint(x: 0, y: 0)
         o: Outer(a)
         ground a",
    );
    assert_eq!(sk.points.len(), 2);
    let (zx, zy) = sk.point_xy(1);
    let d = zx.hypot(zy);
    assert!((d - 20.0).abs() < 1e-6, "z is 20 from a, not from nothing: {d}");
}

#[test]
fn each_copy_of_an_instance_in_a_block_binds_its_own_actual() {
    let sk = drawn(
        "component Peg(a: point, b: point) {
           a distance(10) b
         }
         point hub hint(x: 0, y: 0)
         ground hub
         repeat 3 as i {
           point tip hint(x: 20 + i, y: i * 5)
           s: Peg(hub, tip)
         }",
    );
    let d = diagnose(&mut sk.clone(), DiagnoseOptions::default());
    assert_eq!(d.structural_rank, 3, "three pegs on three tips: {}", gcs_core::diagnose::summary(&d));
    assert!(d.over.is_empty(), "nothing is said twice: {:?}", d.over);
    for i in 1..4 {
        let (x, y) = sk.point_xy(i);
        assert!((x.hypot(y) - 10.0).abs() < 1e-6, "tip {i} is on its own peg");
    }
}

/// A `cycle` of instances is the same rule with the wrap: six spokes, six lengths.
#[test]
fn a_cycle_of_instances_binds_per_copy() {
    let sk = drawn(
        "component Spoke(c: point, t: point) {
           c distance(40) t
         }
         point hub hint(x: 0, y: 0)
         ground hub
         cycle 6 as i {
           point tip hint(x: 40 * cos(60 * i), y: 40 * sin(60 * i))
           s: Spoke(hub, tip)
         }",
    );
    let d = diagnose(&mut sk.clone(), DiagnoseOptions::default());
    assert_eq!(d.structural_rank, 6, "{}", gcs_core::diagnose::summary(&d));
    assert!(d.over.is_empty(), "over: {:?}", d.over);
}

/// #45.3 — a repetition inside a component is reached from outside by indexing the dotted
/// name: `l.p[1]` is copy 1 of the `p` the block in `l` declares, and a field of the copy
/// follows the index.  A copy that is not there is still nothing, and through a second
/// instance the same spelling reaches the other one.
#[test]
fn a_copy_inside_an_instance_is_indexed_from_outside() {
    let sk = drawn(
        "component Ladder(o: point, n: Int) {\n\
         \x20 repeat n as i {\n\
         \x20   point p hint(x: 0, y: i * 10)\n\
         \x20   line e(p, hint(x: 5, y: i * 10))\n\
         \x20 }\n\
         \x20 o coincident p[0]\n\
         }\n\
         point o hint(x: 0, y: 0)\n\
         point o2 hint(x: 50, y: 0)\n\
         l: Ladder(o, n: 3)\n\
         m: Ladder(o2, n: 2)\n\
         ground o\n\
         ground o2\n\
         l.p[0] vertical l.p[1]\n\
         l.p[0] distance(10) l.p[1]\n\
         l.p[1] vertical l.p[2]\n\
         l.p[1] distance(10) l.p[2]\n\
         horizontal l.e[2]\n\
         l.e[2].p1 distance(7) l.e[2].p2\n\
         m.p[1] distance(12) o2\n\
         m.p[0] vertical m.p[1]\n",
    );
    // l.p[2] is 20 up from o; l.e[2]'s far end is 7 further along, level with it
    let n = sk.points.len();
    assert!(n >= 6, "{n}");
    let ys: Vec<f64> = (0..n).map(|i| sk.point_xy(i).1).collect();
    assert!(ys.iter().any(|&y| (y - 20.0).abs() < 1e-6), "{ys:?}");
    assert!(ys.iter().any(|&y| (y - 12.0).abs() < 1e-6), "{ys:?}");
    let xs: Vec<f64> = (0..n).map(|i| sk.point_xy(i).0).collect();
    assert!(xs.iter().any(|&x| (x - 7.0).abs() < 1e-6), "{xs:?}");
    // past the copies, and a copy of a copy, are nothing
    let refused = |src: &str| {
        let (prog, _) = gcs_core::syntax::parse(src);
        let e = gcs_core::program::elaborate(&prog);
        assert!(!e.ok(), "{src}");
    };
    refused("component L(n: Int) { repeat n { point p } }\nl: L(n: 2)\nground l.p[2]\n");
    refused("component L(n: Int) { repeat n { point p } }\nl: L(n: 2)\nground l.p[0][0]\n");
}
