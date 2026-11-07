//! **The numeric rank, part by part** (#88): past the dense limit the diagnosis factors the
//! block-triangular order's parts one at a time.  Below it the whole SVD runs, and here the two
//! readings are held to one another over the library: the same rank, DOF and status, the same
//! constraints implied, over and expected, the same parameters free.
use gcs_core::diagnose::{diagnose, DiagnoseOptions, Diagnosis, NUMERIC_MAX};
use gcs_core::examples;
use gcs_core::linalg::Mat;
use gcs_core::model::Sketch;
use gcs_core::solve::{solve, SolveOpts};
use std::collections::BTreeSet;

/// The whole reading and the parts' reading of one sketch at one pose.
fn both(sk: &mut Sketch) -> (Diagnosis, Diagnosis) {
    let whole = diagnose(sk, DiagnoseOptions { numeric: Some(true), ..DiagnoseOptions::default() });
    let parts = diagnose(sk, DiagnoseOptions { numeric_max: 0, ..DiagnoseOptions::default() });
    assert!(parts.by_parts && !whole.by_parts);
    (whole, parts)
}

/// What a reading says, as sets: one line to compare and to print.
fn said(d: &Diagnosis) -> String {
    let set = |v: &[u32]| v.iter().copied().collect::<BTreeSet<u32>>();
    format!(
        "rank {:?} dof {} {:?} implied {:?} over {:?} expected {:?} free {:?}",
        d.numeric_rank, d.dof, d.status, set(&d.implied), set(&d.over), set(&d.expected),
        set(&d.under_params)
    )
}

/// Every library case of free parameters within `free` whose two readings differ, said.  The
/// cases are read side by side (`par`), answered in the library's order.
fn disagreements(free: std::ops::RangeInclusive<usize>) -> Vec<String> {
    gcs_core::par::map(&examples::CASES, |(_, key, _)| {
        if examples::left_to_slow_tier(key) {
            return None;
        }
        let mut sk = examples::case(key)?;
        solve(&mut sk, SolveOpts::default());
        if !free.contains(&gcs_core::system::System::new(&sk).n_free) {
            return None;
        }
        let (whole, parts) = both(&mut sk);
        (whole.numeric_rank.is_some() && said(&whole) != said(&parts))
            .then(|| format!("{key}:\n  whole {}\n  parts {}", said(&whole), said(&parts)))
    })
    .into_iter()
    .flatten()
    .collect()
}

/// Below the dense limit, where the whole SVD runs anyway: the parts read every case alike.
#[test]
fn the_parts_read_the_library_as_the_whole_does() {
    let differ = disagreements(0..=NUMERIC_MAX);
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// The most free parameters a case is held to the whole SVD at: the V-twin's 993 take seconds,
/// the engine's 4,859 two SVDs of minutes each.  Past it a case's parts are held to the Jacobian
/// itself (`the_parts_reading_of_every_large_case_holds_on_its_jacobian`).
const WHOLE_MAX: usize = 1000;

/// Past it — the V-twin, the long truss, the plate — the parts read each large case as the whole
/// SVD does, up to `WHOLE_MAX`.  The library's rod (`examples::SLOW`, its solve minutes) joins
/// them in the slow tier.
#[test]
fn the_parts_read_the_large_cases_as_the_whole_does() {
    let differ = disagreements(NUMERIC_MAX + 1..=WHOLE_MAX);
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// The parts' motions and dependencies of `sk` held to its Jacobian: each sent to zero by it
/// (`J n`, `Jᵀ w`, to 1e-8 of its largest entry), each set independent, and the rank they leave
/// read forward over the columns, `n − motions`, the same as read backward over the rows,
/// `m − dependencies` — two readings by different parts in different orders — and as its
/// diagnosis `d` says it.  What is wrong, said; nothing where it holds.
fn held_on_its_jacobian(key: &str, sk: &Sketch, d: &Diagnosis) -> Vec<String> {
    let (j, rows, null, left) = gcs_core::diagnose::parts_null_spaces(sk);
    let n = j.n_cols;
    let big = (0..j.n_rows()).flat_map(|r| j.row(r).map(|(_, v)| v.abs())).fold(0.0, f64::max);
    let mut wrong = Vec::new();
    for (k, v) in null.iter().enumerate() {
        let off = (0..j.n_rows()).map(|r| j.row_dot(r, v).abs()).fold(0.0, f64::max);
        if !(off <= 1e-8 * big) {
            wrong.push(format!("{key}: motion {k} moves a row by {off:e}"));
        }
    }
    for (k, w) in left.iter().enumerate() {
        let mut jt = vec![0.0; n];
        for (r, &wr) in w.iter().enumerate().filter(|(_, wr)| **wr != 0.0) {
            for (c, a) in j.row(r) {
                jt[c] += wr * a;
            }
        }
        let off = jt.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        if !(off <= 1e-8 * big) {
            wrong.push(format!("{key}: dependency {k} leaves {off:e} of the rows"));
        }
    }
    let independent = |vs: &[Vec<f64>]| {
        vs.is_empty()
            || gcs_core::linalg::rank_rrqr(&Mat::from_vec(vs.len(), vs[0].len(), vs.concat()), 1e-9)
                == vs.len()
    };
    if !independent(&null) || !independent(&left) {
        wrong.push(format!("{key}: {} motions or {} dependencies not independent", null.len(),
                           left.len()));
    }
    let (forward, backward) = (n - null.len(), rows - left.len());
    if forward != backward || d.numeric_rank != Some(forward) {
        wrong.push(format!("{key}: rank {forward} over the columns, {backward} over the rows, {:?} \
                            said", d.numeric_rank));
    }
    wrong
}

/// **Every large case's parts hold on its Jacobian** (#88, #780), the engine's 4,859 columns among
/// them: what the whole SVD would be asked, certified by the Jacobian itself in milliseconds a
/// case — every motion and dependency the parts read is one, and there are as many as the rank
/// read both ways leaves.
#[test]
fn the_parts_reading_of_every_large_case_holds_on_its_jacobian() {
    let wrong: Vec<String> = gcs_core::par::map(&examples::CASES, |(_, key, _)| {
        if examples::left_to_slow_tier(key) {
            return Vec::new();
        }
        let Some(mut sk) = examples::case(key) else { return Vec::new() };
        solve(&mut sk, SolveOpts::default());
        if gcs_core::system::System::new(&sk).n_free <= NUMERIC_MAX {
            return Vec::new();
        }
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        held_on_its_jacobian(key, &sk, &d)
    }).into_iter().flatten().collect();
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// A rigid framework of `n` points past the dense limit (`fixtures::laman`, Henneberg's moves:
/// minimally rigid by construction, grounded), with `more` bars added between points it already
/// holds and the first `fewer` of its bars taken away.  Its numbers are measured off its points,
/// so it stands solved.
fn framework(n: usize, seed: u32, more: usize, fewer: usize) -> Sketch {
    use gcs_core::constraints::{CKind, Constraint};
    use gcs_core::model::EntRef;
    let mut sk = gcs_core::fixtures::laman(n, seed, true);
    let bars: Vec<u32> = sk.constraints.iter().filter(|c| c.kind == CKind::Distance).map(|c| c.id)
        .take(fewer).collect();
    for id in bars {
        sk.remove(id);
    }
    let mut rng = gcs_core::rng::Rng::new(seed + 1);
    for _ in 0..more {
        let s = rng.sample(n, 2);
        let ((ax, ay), (bx, by)) = (sk.point_xy(s[0]), sk.point_xy(s[1]));
        let d = (ax - bx).hypot(ay - by);
        sk.add(Constraint::distance(EntRef::point(s[0]), EntRef::point(s[1]), d));
    }
    sk
}

/// **A framework past the dense limit whose answer is known reads as it must, by parts and by the
/// whole alike** (#88, #780): Henneberg's frameworks of 152 points (302 free numbers, just past
/// the limit) are rigid by construction, so held as made they are determined, and with `k` bars
/// fewer `k` freedoms open.  At 302 columns the whole SVD takes a fraction of a second, so each
/// seed is read both ways, and the parts held to the Jacobian.  With a bar more the rank is the
/// same and its dependency is over, read by parts alone: naming what is over is the cost (three
/// bars more took ten seconds a reading at 358 columns), and the library's small cases hold the
/// two readings' `over` to one another (`the_parts_read_the_library_as_the_whole_does`).
#[test]
fn a_framework_past_the_dense_limit_reads_as_it_was_built() {
    let held = |sk: &Sketch, d: &Diagnosis, what: &str| {
        let wrong = held_on_its_jacobian(what, sk, d);
        assert!(wrong.is_empty(), "{}", wrong.join("\n"));
    };
    for seed in 0..2 {
        for fewer in [0, 2] {
            let mut sk = framework(152, 780 + seed, 0, fewer);
            let what = format!("seed {seed}, {fewer} fewer");
            let (whole, parts) = both(&mut sk);
            assert!(whole.n_params > NUMERIC_MAX, "{what}: {} free", whole.n_params);
            assert_eq!(said(&whole), said(&parts), "{what}");
            assert_eq!((parts.dof, parts.numeric_rank), (fewer as i64, Some(302 - fewer)),
                       "{what}");
            assert!(parts.over.is_empty() && parts.implied.is_empty(), "{what}: {:?}", parts.over);
            held(&sk, &parts, &what);
        }
    }
    let mut sk = framework(152, 780, 1, 0);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert!(d.by_parts && d.n_params > NUMERIC_MAX);
    assert_eq!((d.dof, d.numeric_rank), (0, Some(302)));
    assert!(!d.over.is_empty(), "a bar more is over: {:?}", d.implied);
    held(&sk, &d, "a bar more");
}

/// **Past the dense limit the rank is read, not skipped** (#88): the V-twin's free parameters are
/// diagnosed part by part and found held, its one freedom — the crank — driven by `t`.
#[test]
fn the_v_twin_is_read_past_the_dense_limit() {
    let mut sk = examples::case("vtwin").unwrap();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert!(d.n_params > NUMERIC_MAX && d.by_parts && !d.numeric_skipped);
    assert_eq!((d.dof, d.numeric_rank.is_some()), (0, true));
    assert!(d.over.is_empty() && d.implied.is_empty(), "{:?} {:?}", d.over, d.implied);
    assert_eq!(d.parameters.iter().map(|p| (p.name.as_str(), p.readers.len())).collect::<Vec<_>>(),
               [("t", 1)]);
}

/// A cone touched by a plane at a point, its apex drawn there (#148), with `n` copies of a point
/// held off the touch by two ordinates beside it: determined, however many.
fn cone_and(n: usize) -> String {
    format!(
        "unit mm\nuse std\nuse std.sets\nO := point in std.top\nfix((0, 0)) O\nM := point in std.top\n\
         fix((40, 0)) M\nT := point hint((20, 0, -30))\ngax := line(O, T)\nlength(60) gax\n\
         gc := std.sets.Cone(gax, half: 30deg)\nM in boundary(gc)\nstd.top tangent(at: M) boundary(gc)\n\
         repeat {n} {{\n  q := point hint((50, 5)) in std.top\n  \
         M distance(10, along: std.x) q\n  M distance(5, along: std.y) q\n}}\n"
    )
}

/// **A tangency's own dependency, past the dense limit** (#148 with #88): the cone touched by a
/// plane with its apex drawn there, beside enough determined geometry to pass 300 parameters, is
/// read as its small twin is — determined, the dependency expected and said nowhere.
#[test]
fn a_large_twin_reads_as_its_small_one() {
    let read = |n: usize| {
        let (prog, errs, _) = gcs_core::library::parse_linked(&cone_and(n));
        assert!(errs.is_empty(), "{errs:?}");
        let mut e = gcs_core::program::elaborate(&prog);
        assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
        assert!(solve(&mut e.sketch, SolveOpts::default()).success);
        diagnose(&mut e.sketch, DiagnoseOptions::default())
    };
    let (small, large) = (read(1), read(160));
    assert!(!small.by_parts && large.by_parts && large.n_params > NUMERIC_MAX);
    for d in [&small, &large] {
        assert_eq!((d.dof, d.status), (0, gcs_core::diagnose::State::Well), "{:?}", d.warnings);
        assert!(d.over.is_empty() && d.implied.is_empty(), "{:?} {:?}", d.over, d.implied);
        assert_eq!(d.expected.len(), 1);
    }
}

/// **A sub-Jacobian's rank, read part by part, is its dense rank** (#851): any rows of a library
/// case over any of its columns — a part's rows over its own unknowns, as the reading through the
/// parts asks (`parts::read`, D3) — ranked by `rank_by_parts`, which cuts them into their own
/// block-triangular order and each part into the pieces its structure falls into, and by one SVD
/// of the whole selection, agree.  Three draws a case, rows and columns sampled by the seeded rng.
#[test]
fn a_sub_jacobian_ranked_by_parts_is_its_dense_rank() {
    use gcs_core::linalg::{rank_and_nullspace_with, Tol};
    use gcs_core::system::RANK_TOL;
    let wrong: Vec<String> = gcs_core::par::map(&examples::CASES, |(_, key, _)| {
        if examples::left_to_slow_tier(key) {
            return Vec::new();
        }
        let Some(mut sk) = examples::case(key) else { return Vec::new() };
        solve(&mut sk, SolveOpts::default());
        if gcs_core::system::System::new(&sk).n_free > NUMERIC_MAX {
            return Vec::new();
        }
        let (j, _, _, _) = gcs_core::diagnose::parts_null_spaces(&sk);
        let (m, n) = (j.n_rows(), j.n_cols);
        if m == 0 || n == 0 {
            return Vec::new();
        }
        let mut rng = gcs_core::rng::Rng::new(851);
        let mut wrong = Vec::new();
        for draw in 0..3 {
            let mut rows = rng.sample(m, (3 * m / 4).max(1));
            let mut cols = rng.sample(n, (n / 2).max(1));
            rows.sort_unstable();
            cols.sort_unstable();
            let by_parts = gcs_core::diagnose::rank_by_parts(&j, &rows, &cols);
            let dense = rank_and_nullspace_with(&j.dense(&rows, &cols), Tol::Abs(RANK_TOL)).rank;
            if by_parts != dense {
                wrong.push(format!("{key}, draw {draw}: rank {by_parts} by parts, {dense} dense \
                                    ({} rows over {} columns)", rows.len(), cols.len()));
            }
        }
        wrong
    }).into_iter().flatten().collect();
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// A chain of `n` parts, each standing on the tip of the one before with its z along the world's
/// — the mate of `std.place.seats` without its turn — so each turns freely about its seat.
/// Every part's turn moves every part after it: the drawing's under-determined part is the whole
/// chain.
fn turning_chain(n: usize) -> String {
    let mut src = String::from(
        "use std\n\ncomponent Part() {\n  in std.top {\n    tip := point\n    \
         fix((10mm, 0mm)) tip\n  }\n}\n\nin std.top {\n  s := point\n  fix((0mm, 0mm)) s\n}\n",
    );
    for k in 0..n {
        let on = if k == 0 { "s".to_string() } else { format!("p{}.tip", k - 1) };
        src += &format!(
            "p{k} := Part()\np{k}.std.origin coincident {on}\n\
             p{k}.std.z parallel(sense: same) std.z\n"
        );
    }
    src
}

/// **A chain's under-determined part is read a link at a time** (#858): the under part of the
/// order, cut into its own (`graph::under_blocks`) — its free columns, then square blocks in a
/// solve order — over the entries the Jacobian has where the chain stands, is block
/// lower-triangular, covers the part once, and no block of it is wider than a part's own numbers,
/// however long the chain; read whole it was one matrix of all of them, and the planes a chain of
/// seated parts places were its cube (forty links: 0.3 s, a hundred and sixty: 47 s).  Read
/// so, the chain reads as the whole SVD reads it, and holds on its Jacobian.
#[test]
fn a_turning_chain_is_read_a_link_at_a_time() {
    let n = 24;
    let mut sk = crate::common::build(&turning_chain(n)).sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let (j, _, _, _) = gcs_core::diagnose::parts_null_spaces(&sk);
    let rows: Vec<usize> = (0..j.n_rows()).collect();
    let adj: Vec<Vec<usize>> = rows.iter()
        .map(|&r| j.row(r).filter(|&(_, v)| v != 0.0).map(|(c, _)| c).collect())
        .collect();
    let dm = gcs_core::graph::dulmage_mendelsohn(&adj, j.n_cols);
    let (free, under) = gcs_core::graph::under_blocks(&adj, &dm);
    // each under column once: free, or matched to a row of exactly one block
    let mut part_of = vec![usize::MAX; j.n_cols];
    for &c in &free {
        part_of[c] = 0;
    }
    let mut seen = BTreeSet::new();
    for (k, block) in under.rows.iter().enumerate() {
        for &r in block {
            assert!(seen.insert(r), "row {r} in two blocks");
            let c = dm.mate_row[r] as usize;
            assert_eq!(part_of[c], usize::MAX, "column {c} twice");
            part_of[c] = k + 1;
        }
    }
    assert_eq!(seen, dm.under_rows.iter().copied().collect::<BTreeSet<usize>>());
    let mut under_cols: Vec<usize> = (0..j.n_cols).filter(|&c| part_of[c] != usize::MAX).collect();
    under_cols.sort_unstable();
    assert_eq!(under_cols, dm.under_cols);
    // a block's rows read the free columns, earlier blocks' and its own, never a later block's
    for (k, block) in under.rows.iter().enumerate() {
        for &r in block {
            for &c in &adj[r] {
                assert!(part_of[c] == usize::MAX || part_of[c] <= k + 1,
                        "block {k} reads block {}'s column {c}", part_of[c] - 1);
            }
        }
    }
    let widest = under.rows.iter().map(Vec::len).max().unwrap_or(0);
    assert!(dm.under_cols.len() > 10 * n, "{} under columns", dm.under_cols.len());
    assert!(widest <= 6, "a block of {widest} rows of {}: the chain is not cut",
            dm.under_rows.len());
    // and the reading through those parts is the whole SVD's, every motion a motion
    let (whole, parts) = both(&mut sk);
    assert_eq!(said(&whole), said(&parts));
    assert_eq!(parts.dof, n as i64, "each part turns about its seat");
    let wrong = held_on_its_jacobian("chain", &sk, &parts);
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}
