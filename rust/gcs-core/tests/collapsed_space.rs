//! **A line of no length is square to nothing in space either** (#496): `perpendicular` and
//! `parallel` written across views lower to `Perpendicular3` and `Parallel3`, whose rows read the
//! two lines' unit directions — `â·b̂` and `(â × b̂)·e_k` — so, like the words in a view (#479),
//! they hold at a line whose ends are together whatever the other line, and so does a point on a
//! line in space and a line along a plane's normal.  Which lines a row reads so was a hand list
//! that named the words in a view only, and such a figure was reported solved; the list is now
//! exhaustive over the kinds (`Constraint::direction_lines`), and the solve fails `COLLAPSED`,
//! the relations named (`Diagnosis::collapsed`).
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::solve::{solve, SolveOpts, COLLAPSED};

use crate::common::read;

/// A line in the side view held where it stands, and one in the front view seeded with a length
/// (a line in space with none is refused where it is written) whose ends the solve brings
/// together at a point: `la` ends with no direction.
const HELD: &str = "unit mm\nuse std\n\
                    in std.side {\n  lb := line(hint((0, 0)), hint((10, 5)))\n  \
                    fix((0, 0)) lb.p1\n  fix((10, 5)) lb.p2\n}\n\
                    in std.front {\n  la := line(hint((3, 4)), hint((8, 6)))\n  \
                    fix((3, 4)) la.p1\n  la.p2 coincident la.p1\n}\n";

/// Every statement holds on the collapsed line, so the solve ends there with nowhere else to
/// go: it fails, never solved, and the one relation reading `la` by direction is named —
/// whichever word in space it is.
#[test]
fn a_line_held_at_a_point_is_not_square_in_space() {
    for (stmt, kind) in [
        ("la perpendicular lb", CKind::Perpendicular3),
        ("la parallel lb", CKind::Parallel3),
        ("lb parallel la", CKind::Parallel3),
        ("lb.p2 coincident la", CKind::PointOnLine3),
        ("la perpendicular std.side", CKind::LinePerpendicularPlane),
    ] {
        let mut e = read(&format!("{HELD}{stmt}\n"));
        let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
        assert!(kinds.contains(&kind), "{stmt}: {kinds:?}");
        let r = solve(&mut e.sketch, SolveOpts::default());
        assert!(!r.success && r.status == COLLAPSED, "{stmt}: {r:?}");
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        assert_eq!(d.collapsed.len(), 1, "{stmt}: {d:?}");
    }
}

/// The controls: a line of the front view with a length, one end held, square to the side
/// view's line and along a vertical one, solves as it did, nothing collapsed.
#[test]
fn a_line_with_a_length_is_still_square_in_space() {
    let free = "unit mm\nuse std\n\
                in std.side {\n  lb := line(hint((0, 0)), hint((10, 5)))\n  \
                lv := line(hint((20, 0)), hint((20, 10)))\n  fix((0, 0)) lb.p1\n  \
                fix((10, 5)) lb.p2\n  fix((20, 0)) lv.p1\n  fix((20, 10)) lv.p2\n}\n\
                in std.front {\n  la := line(hint((0, 0)), hint((7, 3)))\n  \
                fix((0, 0)) la.p1\n  length(10mm) la\n}\n";
    for stmt in ["la perpendicular lb", "la parallel lv"] {
        let mut e = read(&format!("{free}{stmt}\n"));
        let r = solve(&mut e.sketch, SolveOpts::default());
        assert!(r.success, "{stmt}: {r:?}");
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        assert!(d.collapsed.is_empty(), "{stmt}: {d:?}");
    }
}
