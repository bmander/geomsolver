//! **The numeric rank, part by part** (#88): past the dense limit the diagnosis factors the
//! block-triangular order's parts one at a time.  Below it the whole SVD runs, and here the two
//! readings are held to one another over the library: the same rank, DOF and status, the same
//! constraints implied, over and expected, the same parameters free.
use gcs_core::diagnose::{diagnose, DiagnoseOptions, Diagnosis, NUMERIC_MAX};
use gcs_core::examples;
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

/// Every library case of at most `limit` free parameters whose two readings differ, said.
fn disagreements(limit: usize) -> Vec<String> {
    let mut differ = Vec::new();
    for (_, key, _) in examples::CASES {
        let Some(mut sk) = examples::case(key) else { continue };
        solve(&mut sk, SolveOpts::default());
        if gcs_core::system::System::new(&sk).n_free > limit {
            continue;
        }
        let (whole, parts) = both(&mut sk);
        if whole.numeric_rank.is_some() && said(&whole) != said(&parts) {
            differ.push(format!("{key}:\n  whole {}\n  parts {}", said(&whole), said(&parts)));
        }
    }
    differ
}

/// Below the dense limit, where the whole SVD runs anyway: the parts read every case alike.
#[test]
fn the_parts_read_the_library_as_the_whole_does() {
    let differ = disagreements(NUMERIC_MAX);
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// Past it — the V-twin, the engine, the long truss — forcing the whole SVD to compare against
/// takes minutes, which the slow tier affords.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, minutes: the whole SVD of every large case")]
fn the_parts_read_the_large_cases_as_the_whole_does() {
    let differ = disagreements(usize::MAX);
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// **Past the dense limit the rank is read, not skipped** (#88): the V-twin's 925 free parameters
/// are diagnosed part by part, its one freedom — the crank — found.
#[test]
fn the_v_twin_is_read_past_the_dense_limit() {
    let mut sk = examples::case("vtwin").unwrap();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert!(d.n_params > NUMERIC_MAX && d.by_parts && !d.numeric_skipped);
    assert_eq!((d.dof, d.numeric_rank.is_some()), (1, true));
    assert!(d.over.is_empty() && d.implied.is_empty(), "{:?} {:?}", d.over, d.implied);
}

/// A cone touched by a plane at a point, its apex drawn there (#148), with `n` copies of a point
/// held off the touch by two ordinates beside it: determined, however many.
fn cone_and(n: usize) -> String {
    format!(
        "unit mm\nuse std\nO := point in std.top\nfix((0, 0)) O\nM := point in std.top\n\
         fix((40, 0)) M\nT := point hint((20, 0, -30))\ngax := line(O, T)\ndistance(60) gax\n\
         gc := std.Cone(gax, half: 30deg)\nM coincident gc\nstd.top tangent(at: M) gc\n\
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
