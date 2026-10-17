//! **Relations over a parameter** (#391 stage 4): a relation reading a declared parameter
//! (`param t: Angle over (a, b)`) holds for every value of it, and the drawing is followed along
//! it by continuation (`follow::follow`) from the interval's start, where it is drawn — one branch
//! carried, a singular instant reported where the march stops.  The gate: a four-bar's coupler
//! curve against its closed form, a slider-crank's slider against its own, a branch held through a
//! near-singular instant, and singular instants reported where the closed forms put them.

use gcs_core::follow::{follow, Kind};
use gcs_core::model::{EntKind, Sketch};

use crate::common::text::solved;

/// A four-bar: ground `a`–`d` of length `g`, crank `a`–`b` of `r` driven by `t`, coupler `b`–`c`
/// of `l`, rocker `c`–`d` of `k`; `m` the coupler's midpoint, `extra` more statements.
fn four_bar(g: f64, r: f64, l: f64, k: f64, over: &str, extra: &str) -> String {
    format!("unit mm\nuse std\nparam t: Angle over {over}\nin std.front {{\n  a := point\n  \
        d := point\n  b := point hint(({r}, 0))\n  c := point hint(({g}, {k}))\n  m := point\n  \
        fix((0, 0)) a\n  fix(({g}, 0)) d\n  a distance({r}) b\n  b distance({l}) c\n  \
        c distance({k}) d\n  bc := line(b, c)\n  m midpoint bc\n  ground := line(a, d)\n  \
        crank := line(a, b)\n  ground angle(t) crank\n{extra}}}\n")
}

/// Where a point named `n` stands in `sk`.
fn at(sk: &Sketch, e: &gcs_core::program::Elaborated, n: &str) -> (f64, f64) {
    let p = e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"));
    assert_eq!(p.kind, EntKind::Point);
    sk.point_xy(p.i())
}

/// The four-bar's rocker pin at crank angle `t` (degrees), on the branch where `c` stands to the
/// left of `b → d` (`side` 1) or to the right (`-1`): the two circles about `b` and `d` met.
pub(crate) fn closed_c(g: f64, r: f64, l: f64, k: f64, t: f64, side: f64) -> Option<(f64, f64)> {
    let (s, co) = t.to_radians().sin_cos();
    let b = (r * co, r * s);
    let (dx, dy) = (g - b.0, -b.1);
    let e = dx.hypot(dy);
    let along = (l * l - k * k + e * e) / (2.0 * e);
    let h2 = l * l - along * along;
    if h2 < 0.0 {
        return None;
    }
    let h = h2.sqrt() * side;
    let (ux, uy) = (dx / e, dy / e);
    Some((b.0 + along * ux - h * uy, b.1 + along * uy + h * ux))
}

#[test]
fn a_four_bars_coupler_curve_is_its_closed_form() {
    let (g, r, l, k) = (40.0, 10.0, 35.0, 25.0);
    let e = solved(&four_bar(g, r, l, k, "(0deg, 360deg)", ""));
    let f = follow(&e.sketch, "t").expect("followed");
    assert!(f.singular.is_none(), "{:?}", f.singular);
    assert_eq!(f.reached(), 360.0);
    assert!(f.stations.len() > 100, "{}", f.stations.len());
    let mut sk = e.sketch.clone();
    let mut worst = 0.0f64;
    for s in &f.stations {
        sk.set_x(&s.x);
        let want = closed_c(g, r, l, k, s.at, 1.0).expect("a crank-rocker closes everywhere");
        let got = at(&sk, &e, "c");
        worst = worst.max((got.0 - want.0).hypot(got.1 - want.1));
        let (b, m) = (at(&sk, &e, "b"), at(&sk, &e, "m"));
        let mid = ((b.0 + want.0) / 2.0, (b.1 + want.1) / 2.0);
        worst = worst.max((m.0 - mid.0).hypot(m.1 - mid.1));
    }
    assert!(worst < 1e-8, "{worst}");
}

/// Every station of `src`'s march over `t` against the four-bar's closed form on `side`: the
/// largest distance of `c` from it, and the parameter the march reached.
fn worst_c(src: &str, (g, r, l, k): (f64, f64, f64, f64), side: f64) -> (f64, f64) {
    let e = solved(src);
    let f = follow(&e.sketch, "t").expect("followed");
    let mut sk = e.sketch.clone();
    let mut worst = 0.0f64;
    for s in &f.stations {
        sk.set_x(&s.x);
        let want = closed_c(g, r, l, k, s.at, side)
            .unwrap_or_else(|| panic!("the four-bar does not close at {}", s.at));
        let got = at(&sk, &e, "c");
        worst = worst.max((got.0 - want.0).hypot(got.1 - want.1));
    }
    (worst, f.reached())
}

#[test]
fn a_slider_cranks_slider_is_its_closed_form() {
    let (r, l) = (10.0, 30.0);
    let src = format!("unit mm\nuse std\nparam t: Angle over (-90deg, 270deg)\n\
        in std.front {{\n  a := point\n  e := point\n  b := point hint((0, -{r}))\n  \
        s := point hint(({l}, 0))\n  fix((0, 0)) a\n  fix((100, 0)) e\n  rail := line(a, e)\n  \
        a distance({r}) b\n  b distance({l}) s\n  s coincident rail\n  \
        rail angle(t) line(a, b)\n}}\n");
    let e = solved(&src);
    let f = follow(&e.sketch, "t").expect("followed");
    assert!(f.singular.is_none(), "{:?}", f.singular);
    let mut sk = e.sketch.clone();
    let mut worst = 0.0f64;
    for st in &f.stations {
        sk.set_x(&st.x);
        let (sn, cs) = st.at.to_radians().sin_cos();
        let want = r * cs + (l * l - r * r * sn * sn).sqrt();
        let got = at(&sk, &e, "s");
        worst = worst.max((got.0 - want).abs()).max(got.1.abs());
    }
    assert!(worst < 1e-8, "{worst}");
    // through both dead centres, where the slider stands still, the motion is regular
    assert_eq!(f.reached(), 270.0);
}

#[test]
fn a_branch_is_held_through_a_near_singular_instant() {
    // a Grashof four-bar a ten-thousandth of a millimetre from its change point: at `t = 180deg`
    // it is all but folded, its two assembly modes 0.05mm apart, and the march stays on its own
    let bar = (40.0, 10.0, 30.0, 20.0001);
    let (g, r, l, k) = bar;
    let (worst, reached) = worst_c(&four_bar(g, r, l, k, "(0deg, 360deg)", ""), bar, 1.0);
    assert_eq!(reached, 360.0);
    assert!(worst < 1e-6, "{worst}");
}

#[test]
fn a_singular_instant_is_reported_where_the_motion_turns_back() {
    // a non-Grashof four-bar whose crank cannot turn round: its limit is where the coupler and the
    // rocker stand in line, `|bd| = l + k`
    let (g, r, l, k): (f64, f64, f64, f64) = (40.0, 20.0, 25.0, 30.0);
    let limit = ((r * r + g * g - (l + k) * (l + k)) / (2.0 * r * g)).acos().to_degrees();
    let e = solved(&four_bar(g, r, l, k, "(0deg, 360deg)", ""));
    let f = follow(&e.sketch, "t").expect("followed");
    let s = f.singular.expect("the crank stops");
    assert_eq!(s.kind, Kind::Limit);
    assert!((s.at - limit).abs() < 1e-3, "{} against {limit}", s.at);
    // said to the six figures a reading is
    let said = format!("singular at t = {limit:.3}deg: no drawing stands past it, the motion turns \
                        back");
    assert_eq!(s.text(&f.over, e.sketch.units), said);
    assert_eq!(gcs_core::report::parameter_text(&e.sketch, "t"),
               format!("t: followed from 0deg to {limit:.3}deg, {said}"));
    // the stations up to it are the branch it started on
    let (worst, _) = worst_c(&four_bar(g, r, l, k, "(0deg, 360deg)", ""), (g, r, l, k), 1.0);
    assert!(worst < 1e-6, "{worst}");
}

#[test]
fn a_singular_instant_is_reported_where_two_branches_cross() {
    // a four-bar at its change point, `r + g = l + k`: at `t = 180deg` it folds flat, and its two
    // assembly modes cross there
    let (g, r, l, k) = (40.0, 10.0, 30.0, 20.0);
    let e = solved(&four_bar(g, r, l, k, "(0deg, 360deg)", ""));
    let f = follow(&e.sketch, "t").expect("followed");
    let s = f.singular.expect("the four-bar folds");
    assert!((s.at - 180.0).abs() < 1e-2, "{s:?}");
    assert_eq!(s.kind, Kind::Branch, "{s:?}");
}

#[test]
fn a_drawing_not_held_is_not_followed() {
    // no rocker: the coupler swings free at every instant, a free function of `t`
    let src = "unit mm\nuse std\nparam t: Angle over (0deg, 90deg)\nin std.front {\n  \
        a := point\n  d := point\n  b := point hint((10, 0))\n  c := point hint((40, 20))\n  \
        fix((0, 0)) a\n  fix((40, 0)) d\n  a distance(10) b\n  b distance(35) c\n  \
        line(a, d) angle(t) line(a, b)\n}\n";
    let e = solved(src);
    let why = follow(&e.sketch, "t").expect_err("not held");
    assert!(why.contains("not held at t = 0deg: 1 degree of freedom, and it is a free function \
                          of `t`"), "{why}");
    // a parameter nothing reads is not followed either
    let e = solved(&four_bar(40.0, 10.0, 35.0, 25.0, "(0deg, 360deg)", "")
        .replace("ground angle(t) crank", "ground angle(30deg) crank"));
    assert!(follow(&e.sketch, "t").is_err_and(|w| w.contains("nothing the drawing states reads")));
}

/// The curve named `n` in `e`.
fn curve(e: &gcs_core::program::Elaborated, n: &str) -> usize {
    let c = e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"));
    assert_eq!(c.kind, EntKind::Curve);
    c.i()
}

#[test]
fn a_point_traced_over_a_parameter_is_the_curve_it_runs_on() {
    // the epic's four-bar: `trace(m over t)`, the coupler's midpoint as the crank turns round
    let (g, r, l, k) = (40.0, 10.0, 35.0, 25.0);
    let src = format!("{}coupler := trace(m over t)\n", four_bar(g, r, l, k, "(0deg, 360deg)", ""));
    let e = solved(&src);
    let i = curve(&e, "coupler");
    let sk = &e.sketch;
    assert_eq!(sk.curve_domain(i), (0.0, 360.0));
    // every sample where the closed form puts the midpoint, and the curve closes on itself
    let poly = sk.curve_polyline(i);
    assert!(poly.len() > 100, "{}", poly.len());
    let n = poly.len() - 1;
    let worst = poly.iter().enumerate().map(|(j, p)| {
        let t = 360.0 * j as f64 / n as f64;
        let c = closed_c(g, r, l, k, t, 1.0).expect("closes");
        let (s, co) = t.to_radians().sin_cos();
        let mid = ((r * co + c.0) / 2.0, (r * s + c.1) / 2.0);
        (p.0 - mid.0).hypot(p.1 - mid.1)
    }).fold(0.0, f64::max);
    assert!(worst < 1e-8, "{worst}");
    assert!(sk.curve_closed(i));
    let at = sk.curve_point(i, 90.0);
    let c = closed_c(g, r, l, k, 90.0, 1.0).expect("closes");
    assert!((at.0 - c.0 / 2.0).hypot(at.1 - (r + c.1) / 2.0) < 1e-8, "{at:?}");

    // a stretch of the interval, written in place
    let src = format!("{}coupler := trace(m over t in (30deg, 90deg))\n",
                      four_bar(g, r, l, k, "(0deg, 360deg)", ""));
    let e = solved(&src);
    let i = curve(&e, "coupler");
    assert_eq!(e.sketch.curve_domain(i), (30.0, 90.0));
    assert!(!e.sketch.curve_closed(i));
}

#[test]
fn a_trace_stops_where_its_drawing_does() {
    // the crank that cannot turn round: its coupler's curve ends at the limit
    let (g, r, l, k): (f64, f64, f64, f64) = (40.0, 20.0, 25.0, 30.0);
    let limit = ((r * r + g * g - (l + k) * (l + k)) / (2.0 * r * g)).acos().to_degrees();
    let src = format!("{}path := trace(c over t)\n", four_bar(g, r, l, k, "(0deg, 360deg)", ""));
    let e = solved(&src);
    let i = curve(&e, "path");
    let end = *e.sketch.curve_polyline(i).last().expect("drawn");
    let want = closed_c(g, r, l, k, limit, 1.0).expect("closes at its limit");
    assert!((end.0 - want.0).hypot(end.1 - want.1) < 1e-2, "{end:?} against {want:?}");
    assert!(!e.sketch.curve_closed(i));
}

#[test]
fn a_trace_over_a_parameter_prints_back_and_refuses_what_it_cannot_mean() {
    use crate::common::lenient::read;
    let bar = four_bar(40.0, 10.0, 35.0, 25.0, "(0deg, 360deg)", "");
    let src = format!("{bar}coupler := trace(m over t)\n");
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert!(printed.contains("coupler := trace(m over t)"), "{printed}");
    let back = solved(&printed);
    let i = curve(&back, "coupler");
    assert_eq!(back.sketch.curve_polyline(i), solved(&src).sketch.curve_polyline(i));

    let refused = |text: &str, want: &str| {
        let (_, said) = read(text);
        assert!(said.iter().any(|m| m.contains(want)), "{text}\n{said:?}");
    };
    // held to, where what moves with `t` leaves a number free: the ground's far end, no longer
    // fixed, but only at its distance (#398 A)
    let loose = bar.replace("fix((40, 0)) d", "a distance(40) d");
    refused(&format!("{loose}coupler := trace(m over t)\nin std.front {{\n  q := point\n  \
        q coincident coupler\n}}\n"),
        "is traced as `t` runs, and the solve cannot hold to it: the rows that move with `t` \
         leave 1 of the numbers they place free");
    // a name no parameter, with no interval of its own
    refused(&format!("{bar}coupler := trace(m over u)\n"), "E103: `u` is no parameter the document \
        declares");
    // a stretch outside the parameter's interval
    refused(&format!("{bar}coupler := trace(m over t in (-10deg, 90deg))\n"),
            "runs over an increasing stretch of its interval, `(0, 360)`");
    // a point in space
    refused(&format!("{bar}s := point hint((1, 2, 3))\nfix((1, 2, 3)) s\nk := trace(s over t)\n"),
            "a point traced over a parameter is drawn in a plane, and `s` stands in space");
}

/// The diagnosis of `src`, read and solved.
fn diagnosis(src: &str) -> (gcs_core::program::Elaborated, gcs_core::diagnose::Diagnosis) {
    let mut e = solved(src);
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, Default::default());
    (e, d)
}

#[test]
fn the_ledger_counts_the_freedom_each_parameter_drives() {
    // a four-bar: one freedom, its crank's, driven by `t` — and nothing left free
    let (e, d) = diagnosis(&four_bar(40.0, 10.0, 35.0, 25.0, "(0deg, 360deg)", ""));
    assert_eq!(d.dof, 0);
    let [p] = d.parameters.as_slice() else { panic!("{:?}", d.parameters) };
    assert_eq!((p.name.as_str(), p.dof), ("t", Some(1)));
    let driver = e.sketch.constraints.iter().find(|c| c.kind == gcs_core::constraints::CKind::Angle)
        .expect("the driver").id;
    assert_eq!(p.readers, vec![driver]);
    let said = gcs_core::diagnose::summary(&d);
    assert!(said.contains("DOF 0; 1 DOF over t;"), "{said}");
    assert!(!said.contains("free function"), "{said}");

    // a five-bar, both cranks driven by the one parameter: two freedoms over it, beside a
    // triangle that reads none
    let five = "unit mm\nuse std\nparam t: Angle over (0deg, 90deg)\nin std.front {\n  \
        a := point\n  e := point\n  b := point hint((10, 0))\n  d := point hint((50, 10))\n  \
        c := point hint((25, 30))\n  fix((0, 0)) a\n  fix((40, 0)) e\n  ground := line(a, e)\n  \
        a distance(10) b\n  e distance(10) d\n  b distance(30) c\n  c distance(30) d\n  \
        ground angle(t) line(a, b)\n  ground angle(90deg + 2 * t) line(e, d)\n  \
        p := point\n  q := point hint((5, -20))\n  fix((0, -20)) p\n  p distance(5) q\n  \
        horizontal line(p, q)\n}\n";
    let (_, d) = diagnosis(five);
    assert_eq!(d.dof, 0);
    assert_eq!((d.parameters[0].readers.len(), d.parameters[0].dof), (2, Some(2)));
    assert!(gcs_core::diagnose::summary(&d).contains("2 DOF over t"));

    // the freedom left beside a parameter is a free function of it
    let (_, d) = diagnosis(&four_bar(40.0, 10.0, 35.0, 25.0, "(0deg, 360deg)", "")
        .replace("  c distance(25) d\n", ""));
    assert_eq!((d.dof, d.parameters[0].dof), (1, Some(1)));
    let said = gcs_core::diagnose::summary(&d);
    assert!(said.contains("DOF 1; 1 DOF over t; each DOF left a free function of t"), "{said}");

    // a parameter nothing reads drives nothing, and the summary says nothing of it
    let (_, d) = diagnosis(&four_bar(40.0, 10.0, 35.0, 25.0, "(0deg, 360deg)", "")
        .replace("ground angle(t) crank", "ground angle(30deg) crank"));
    assert_eq!(d.parameters[0].readers, Vec::<u32>::new());
    assert!(!gcs_core::diagnose::summary(&d).contains(" over t"));
}
