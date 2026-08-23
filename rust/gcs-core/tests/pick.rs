/* What a click picks.  The hit test is the core's, in world units, because what is drawn is
 * what is picked — and only the core knows what is drawn. */
use gcs_core::model::{self, EntKind, Sketch};

fn kinds(sk: &Sketch, x: f64, y: f64, tol: f64) -> Option<(EntKind, usize)> {
    model::pick(sk, x, y, tol).map(|e| (e.kind, e.i()))
}

#[test]
fn a_line_is_picked_along_the_segment_and_not_past_its_ends() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    let l = sk.line(a, b);
    assert_eq!(kinds(&sk, 5.0, 0.4, 1.0), Some((EntKind::Line, l)));
    // on the infinite line, well past the end: a dimension would measure zero here, but
    // nothing is drawn there to click on
    assert_eq!(kinds(&sk, 25.0, 0.0, 1.0), None);
    assert_eq!(kinds(&sk, 5.0, 3.0, 1.0), None);
}

#[test]
fn an_arc_is_picked_over_its_sweep_only() {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, false, "c");
    let s = sk.point(10.0, 0.0, false, "s");
    let e = sk.point(0.0, 10.0, false, "e");
    let arc = sk.arc(c, s, e, "a1");
    let d = 0.5_f64.sqrt() * 10.0;                    // the quarter's midpoint, on the ring
    assert_eq!(kinds(&sk, d, d, 0.5), Some((EntKind::Arc, arc)));
    // the same ring, three quadrants round: on the circle the arc lies on, but not drawn
    assert_eq!(kinds(&sk, -d, -d, 0.5), None);
}

#[test]
fn a_point_within_reach_wins_over_a_nearer_edge() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    sk.line(a, b);
    // dead on the segment, and a little off its end point: the point is what was meant
    assert_eq!(kinds(&sk, 9.4, 0.0, 1.0), Some((EntKind::Point, b)));
}

#[test]
fn nothing_within_the_tolerance_is_nothing() {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, false, "c");
    sk.circle(c, 10.0, "c1");
    assert_eq!(kinds(&sk, 14.0, 0.0, 1.0), None);
    assert_eq!(kinds(&sk, 14.0, 0.0, 5.0).map(|(k, _)| k), Some(EntKind::Circle));
    // and the tolerance is a world length, so it reaches inside the ring as well
    assert_eq!(kinds(&sk, 6.0, 0.0, 5.0).map(|(k, _)| k), Some(EntKind::Circle));
}

#[test]
fn a_curve_is_picked_where_it_runs() {
    let mut sk = Sketch::new();
    let pts: Vec<usize> = [(0.0, 0.0), (5.0, 10.0), (10.0, 10.0), (15.0, 0.0)]
        .iter()
        .map(|&(x, y)| sk.point(x, y, false, "p"))
        .collect();
    let sp = sk.spline(&pts).unwrap();
    let (x, y) = gcs_core::curve::sample(&sk, sp, 9)[4];
    assert_eq!(kinds(&sk, x, y + 0.05, 0.5), Some((EntKind::Spline, sp)));
    assert_eq!(kinds(&sk, x, y + 9.0, 0.5), None);
}
