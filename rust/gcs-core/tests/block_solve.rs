//! The block-triangular solve (`solve::BlockMode`): each strongly connected block of the equations
//! minimised on its own in order, then one whole-system DogLeg from there.  As a rescue — the
//! default — it runs only after the whole-system DogLeg fails and is kept only if it solves, so a
//! document that solves, and one that cannot, come out as they did; `First` runs it before the
//! DogLeg, which is how a document with over- and under-determined parts is seen through the
//! polish here.  The spiral-bevel layout from rough seeds is `hypoid_layout.rs`'s gate.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::examples;
use gcs_core::model::Sketch;
use gcs_core::solve::{solve, BlockMode, SolveOpts};

use crate::common::{build, ent};

fn opts(blocks: BlockMode) -> SolveOpts {
    SolveOpts { blocks, ..SolveOpts::default() }
}

fn bits(sk: &Sketch) -> Vec<u64> {
    sk.get_x().into_iter().map(f64::to_bits).collect()
}

/// A chain of triangles off a grounded point, the first side stated twice (an over-determined
/// part), a point on nothing (under-determined columns) and a point on a circle about the last
/// corner (an under-determined row).
const PARTS: &str = "\
point a hint(x: 0, y: 0)
point b hint(x: 10, y: 1)
point c hint(x: 5, y: 9)
point d hint(x: 15, y: 8)
point e hint(x: 20, y: 1)
point f hint(x: 30, y: 30)
point g hint(x: 26, y: 2)
ground a
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
    let res = solve(&mut sk, opts(BlockMode::First));
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
    assert!(solve(&mut sk, opts(BlockMode::Rescue)).success);
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
    let (a, b) = (solve(&mut off, opts(BlockMode::Off)), solve(&mut rescue, opts(BlockMode::Rescue)));
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

/// Where the whole-system DogLeg solves, the rescue never runs: the library's cases — the
/// conflicting, redundant and under-constrained ones among them — come out to the same bits.  A
/// case the whole solve cannot settle may be rescued, and then it solves.
#[test]
fn a_document_the_whole_solve_settles_is_not_touched() {
    for name in ["rect_fillets", "rect_fillets_conflict", "rect_fillets_under", "truss",
        "truss_redundant", "truss_conflict", "truss_floating", "impossible_triangle", "polygon_chain",
        "k33", "altitudes", "pythagoras", "spline_follower", "peaucellier", "jansen", "bracket"]
    {
        let sk = examples::example(name).unwrap();
        for jitter in [0.0, 2.0] {
            let (mut off, mut rescue) = (sk.clone(), sk.clone());
            off.perturb(jitter, 3);
            rescue.perturb(jitter, 3);
            let a = solve(&mut off, opts(BlockMode::Off));
            let b = solve(&mut rescue, opts(BlockMode::Rescue));
            if a.success || b.method != "blocks" {
                assert_eq!(bits(&off), bits(&rescue), "{name} jittered {jitter}");
                assert_eq!((a.success, &a.method), (b.success, &b.method), "{name}");
            } else {
                assert!(b.success, "{name} jittered {jitter}: a rescue is kept only if it solves");
            }
        }
    }
}
