//! The block-triangular solve (`solve::BlockMode`): each strongly connected block of the equations
//! minimised on its own in order, then one whole-system DogLeg from there.  As a rescue — the
//! default — it runs only after a whole-system DogLeg that did not settle (a failure, or a stop on
//! the iteration limit) and is kept only if it solves, so a document the DogLeg settles, and one
//! that cannot be solved, come out as they did; `First` runs it before the DogLeg, which is how a
//! document with over- and under-determined parts is seen through the polish here.  The
//! spiral-bevel layout from rough seeds is `hypoid_layout.rs`'s gate.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::examples;
use gcs_core::solve::{solve, BlockMode, SolveOpts};

use crate::common::{apart, bits, build, ent, with_blocks};

/// A chain of triangles off a grounded point, the first side stated twice (an over-determined
/// part), a point on nothing (under-determined columns) and a point on a circle about the last
/// corner (an under-determined row).
const PARTS: &str = "\
a := point
b := point hint(x: 10, y: 1)
c := point hint(x: 5, y: 9)
d := point hint(x: 15, y: 8)
e := point hint(x: 20, y: 1)
f := point hint(x: 30, y: 30)
g := point hint(x: 26, y: 2)
fix(x == 0, y == 0) a
a distance(10) b
a horizontal b
a distance(10) b
a distance(10) c
b distance(10) c
b distance(10) d
c distance(10) d
c distance(10) e
d distance(10) e
e distance(5) g
";

#[test]
fn the_polish_settles_the_over_and_under_determined_parts() {
    let e = build(PARTS);
    let mut start = e.sketch.clone();
    start.perturb(1.5, 11);
    let mut sk = start.clone();
    let res = solve(&mut sk, with_blocks(BlockMode::First));
    assert!(res.success && res.method == "blocks", "{res:?}");
    // the free point is on no row: the minimum-norm polish leaves it where it was
    let f = sk.entity_params(ent(&e, "f"));
    for &p in &f {
        assert_eq!(sk.params[p as usize].value, start.params[p as usize].value);
    }
    // every row holds, the repeated one and the one with a freedom left in it included
    let mut sys = gcs_core::system::System::new(&sk);
    let z = sys.z0(&sk);
    assert!(sys.max_relative_residual(&z) < 1e-6);
    let order = sys.block_order();
    assert_eq!((order.blocks.len(), order.over_rows.len(), order.under_rows.len()), (3, 3, 1));
    // and the same start, solved whole and rescued, comes to a solution too
    let mut sk = start.clone();
    assert!(solve(&mut sk, with_blocks(BlockMode::Rescue)).success);
}

/// A genuine conflict is not rescued: the block pass cannot solve it either, so the pose the
/// diagnosis reads is the whole-system solve's, to the bit, and the conflict set is the one the
/// three impossible lengths make.
#[test]
fn a_conflict_is_diagnosed_as_before() {
    let src = PARTS.replace("c distance(10) e", "c distance(30) e").replace("d distance(10) e",
        "d distance(5) e");
    let e = build(&src);
    let (mut off, mut rescue) = (e.sketch.clone(), e.sketch.clone());
    let a = solve(&mut off, with_blocks(BlockMode::Off));
    let b = solve(&mut rescue, with_blocks(BlockMode::Rescue));
    assert!(!a.success && !b.success);
    assert_eq!(a.method, b.method);
    assert_eq!(bits(&off), bits(&rescue));
    let d = diagnose(&mut rescue, DiagnoseOptions::default());
    assert_eq!(d.status, State::Conflict);
    let (c, dd, ee) = (ent(&e, "c"), ent(&e, "d"), ent(&e, "e"));
    let between = |p, q, v: f64| e.sketch.constraints.iter().find(|k| k.kind == CKind::Distance
        && ((k.args[0].ent(), k.args[1].ent()) == (p, q)) && (k.args[2].num() - v).abs() < 1e-9)
        .unwrap().id;
    let mut want = vec![between(c, dd, 10.0), between(c, ee, 30.0), between(dd, ee, 5.0)];
    want.sort_unstable();
    let mut got = d.conflicts.clone().expect("a conflict set");
    got.sort_unstable();
    assert_eq!(got, want);
}

/// Where the whole-system DogLeg settles, the rescue never runs: the library's cases — the
/// conflicting, redundant and under-constrained ones among them — come out to the same bits.  A
/// case the whole solve cannot settle may be rescued, and then it solves.
#[test]
fn a_document_the_whole_solve_settles_is_not_touched() {
    for name in ["rect_fillets", "rect_fillets_conflict", "rect_fillets_under", "truss",
        "truss_redundant", "truss_conflict", "truss_floating", "impossible_triangle",
        "polygon_chain", "k33", "altitudes", "pythagoras", "spline_follower", "peaucellier", "jansen", "bracket"]
    {
        let sk = examples::example(name).unwrap();
        for jitter in [0.0, 2.0] {
            let (mut off, mut rescue) = (sk.clone(), sk.clone());
            off.perturb(jitter, 3);
            rescue.perturb(jitter, 3);
            let a = solve(&mut off, with_blocks(BlockMode::Off));
            let b = solve(&mut rescue, with_blocks(BlockMode::Rescue));
            if a.success || b.method != "blocks" {
                assert_eq!(bits(&off), bits(&rescue), "{name} jittered {jitter}");
                assert_eq!((a.success, &a.method), (b.success, &b.method), "{name}");
            } else {
                assert!(b.success, "{name} jittered {jitter}: a rescue is kept only if it solves");
            }
        }
    }
}

/// A chain of `n` triangles off a grounded, levelled base: `2n + 1` two-row blocks in a line.
fn chain(n: usize) -> String {
    let mut s = String::from("p0 := point\np1 := point hint(x: 10, y: 0)\nfix(x == 0, y == 0) p0\n\
        p0 horizontal p1\np0 distance(10) p1\n");
    for k in 2..n + 2 {
        s += &format!("p{k} := point hint(x: {}, y: {})\n", 5 * k, if k % 2 == 0 { 8 } else { 0 });
        s += &format!("p{} distance(10) p{k}\np{} distance(10) p{k}\n", k - 2, k - 1);
    }
    s
}

/// **A solve that stops on its iteration limit is finished where it stopped**
/// (docs/iteration-limit-rescue-plan.md).  A chain of twelve triangles seeded far off, given eight
/// DogLeg iterations where it needs ten: the eighth leaves it under the interactive acceptance
/// and short of the solution — a success on status 4, which is all a solve did before the rescue
/// (`BlockMode::Off`).  By default that stop is not settled, and the block pass *from the stop*
/// settles it on the solution the whole-system DogLeg was making for: the one it reaches given
/// its hundred iterations.  The pass from the start — what the rescue after a failure runs — does
/// not: each triangle solved alone from those seeds finds its other orientation.
#[test]
fn a_stop_on_the_iteration_limit_is_finished_where_it_stopped() {
    let e = build(&chain(12));
    let mut start = e.sketch.clone();
    start.perturb(6.0, 7);
    let mut full = start.clone();
    assert!(solve(&mut full, SolveOpts::default()).settled());
    let short = SolveOpts { max_iter: 8, ..SolveOpts::default() };
    let mut stop = start.clone();
    let a = solve(&mut stop, SolveOpts { blocks: BlockMode::Off, ..short });
    assert!(a.success && a.status == 4 && a.method == "dogleg", "{a:?}");
    assert!(apart(&stop, &full) > 1e-7, "{:e} from the full solve", apart(&stop, &full));
    let mut sk = start.clone();
    let b = solve(&mut sk, short);
    assert!(b.settled() && b.method == "blocks", "{b:?}");
    assert!(apart(&sk, &full) < 1e-12, "{:e} from the full solve", apart(&sk, &full));
    let mut from_start = start.clone();
    solve(&mut from_start, SolveOpts { retry: false, blocks: BlockMode::First, ..short });
    assert!(apart(&from_start, &full) > 0.1, "{:e}", apart(&from_start, &full));
}
