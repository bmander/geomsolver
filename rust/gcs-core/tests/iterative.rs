//! **A solid read in three dimensions, read faster** (#510): a solid's stiffness solved by CG
//! over its corners' space (`pcg`, `interior::ITERATIVE`), and a filled body's boundary read by
//! its distance — its readings the distance and its gradient, so a refinement places a crossing
//! by Newton and clears its bracket by what each reading reads (`refine::Readings::Distance`).
//! The rules are `docs/contract/analysis.md`'s.
use gcs_core::delaunay::refine::{volume, Cells, Criteria, Domain, Readings};
use gcs_core::model::Sketch;
use gcs_core::solid::faces::BodyFaces;

type P = [f64; 3];

/// The library's solids, each solved by CG: a body cut by another (filled from its boundary), a
/// solid of revolution (filled), a panel and a plank (stacked prisms).
const SOLIDS: [&str; 4] = ["pin_lug", "turned_shaft", "pressed_panel", "sized_plank"];

/// The library case `key`, solved, and its first scenario's solid read in three dimensions.
fn case(key: &str) -> (Sketch, gcs_core::model::Scenario, usize) {
    let e = crate::common::read(gcs_core::examples::source(key).unwrap());
    let sk = crate::common::solved(&e);
    let sc = e.scenarios[0].scenario.clone();
    let solid = (0..sk.solids.len()).find(|&s| sk.solids[s].material.is_some()).expect("a solid");
    (sk, sc, solid)
}

/// **CG over the corners' space agrees with the factor** (#510): each library solid's moves
/// under its first scenario, solved by CG to `pcg::TOLERANCE` of its residual, are the factor's
/// to 1e-8 of the largest — and were CG's, not a factor's it fell back to.
#[test]
fn cg_over_the_corners_agrees_with_the_factor() {
    for key in SOLIDS {
        let (sk, sc, solid) = case(key);
        let (s, x) = gcs_core::bodies::structure(&sk, &sc, solid, 1.0).expect("a structure");
        let factored = s.solve_by(&x, false).expect("factored");
        let cg = s.solve_by(&x, true).expect("by CG");
        assert!(!factored.by_cg() && cg.by_cg(), "{key}: CG was not used");
        let most = factored.u.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let off = factored.u.iter().zip(&cg.u).fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));
        assert!(most > 0.0 && off <= 1e-8 * most, "{key}: CG is {off} off moves of {most}");
        eprintln!("{key}: {} unknowns, CG {off:.2e} off {most:.3e}", factored.u.len());
    }
}

/// **A body's reading is its distance and the distance's gradient** (#510): at points about the
/// pin lug, in it and out of it, the reading's value is the signed distance, its gradient a unit
/// vector, and a step back along it by the distance lands on the boundary — the nearest point —
/// within its tolerance; the distance is one-Lipschitz between neighbouring points; and every
/// face being a plane or a cylinder, every edge a line or a circle, it is the distance itself, no
/// edge with a slack (`distance_slack`).
#[test]
fn a_bodys_reading_is_its_distance_and_its_gradient() {
    let (sk, _, solid) = case("pin_lug");
    let body = BodyFaces::whole(&sk, solid).expect("its faces");
    assert_eq!(body.distance_slack(), Some(Vec::new()));
    let (lo, hi) = body.reach();
    let extent = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
    let mut last: Option<(P, f64)> = None;
    let n = 9;
    for i in 0..=n {
        for j in 0..=n {
            for k in 0..=n {
                let t = |a: usize, c: usize| lo[c] - 0.1 * extent
                    + (hi[c] - lo[c] + 0.2 * extent) * (a as f64 + 0.31) / (n as f64 + 1.0);
                let p = [t(i, 0), t(j, 1), t(k, 2)];
                let (v, _, g) = body.reading(p);
                assert_eq!(v, body.signed_distance(p).0, "at {p:?}");
                let unit = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
                assert!((unit - 1.0).abs() <= 1e-9, "at {p:?} the gradient is {unit} long");
                let foot = [p[0] - v * g[0], p[1] - v * g[1], p[2] - v * g[2]];
                let on = body.signed_distance(foot).0;
                assert!(on.abs() <= 1e-9 * extent, "at {p:?} the foot reads {on}");
                if let Some((q, w)) = last {
                    let d = ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2))
                        .sqrt();
                    assert!((v - w).abs() <= d * (1.0 + 1e-12), "not one-Lipschitz at {p:?}");
                }
                last = Some((p, v));
            }
        }
    }
}

/// A ball's signed distance, read as a domain with readings of the kind asked.
struct Ball(Readings);

impl Domain for Ball {
    fn value(&mut self, p: P) -> f64 {
        (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - 1.0
    }
    fn readings(&self) -> Readings {
        self.0
    }
    fn reading(&mut self, p: P, _: bool) -> (f64, P) {
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt().max(1e-300);
        (r - 1.0, [p[0] / r, p[1] / r, p[2] / r])
    }
}

/// **A distance's readings clear a crossing's bracket** (`Readings::Distance`): a ball filled on
/// its distance's readings so is filled as on the same readings merely agreeing — its crossings
/// all Newton's, its cells as many within a few — in under 0.6 of the readings (4.3 a crossing
/// against 8.7), and fewer than five a crossing.
#[test]
fn a_distances_readings_clear_a_crossings_bracket() {
    let criteria = Criteria { facet_size: 0.25, facet_distance: 0.0125, facet_angle: 25.0,
                              edge_size: 0.25, bisection: 1e-10, max_points: 200_000,
                              normal_angle: 0.0 };
    let cells = Cells { radius_edge: 2.0, size: 0.25, dihedral: 12.0 };
    let fill = |r: Readings| volume(Ball(r), [0.0; 3], 1.4, &[], &criteria, &cells).unwrap();
    let (agreeing, distance) = (fill(Readings::Agreeing), fill(Readings::Distance));
    let (a, d) = (&agreeing.report.surface, &distance.report.surface);
    eprintln!("agreeing {} readings over {} crossings; distance {} over {}", a.readings,
              a.crossings, d.readings, d.crossings);
    assert_eq!((a.fallbacks, d.fallbacks), (0, 0));
    assert_eq!(d.newton, d.crossings);
    assert!(10 * d.readings < 6 * a.readings, "{} readings, against {}", d.readings, a.readings);
    assert!(d.readings < 5 * d.crossings, "{} readings over {} crossings", d.readings,
            d.crossings);
    let (na, nd) = (agreeing.tets.len() as f64, distance.tets.len() as f64);
    assert!((na - nd).abs() <= 0.05 * na, "{nd} cells, against {na}");
}

/// The engine's connecting rod (`engine/conrod.sv`, its preview), solved, and its body.
fn conrod() -> (gcs_core::program::Elaborated, usize) {
    let project = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let source = std::fs::read_to_string(project.join("engine/conrod.sv")).unwrap();
    let e = fixtures::read_beside(&source, &project, &mut |_, text| text);
    let solid = fixtures::solid(&e, "rod.body");
    (e, solid)
}

/// **The rod's reading is pinned by the work it takes** (#510): filled and solved, the engine's
/// connecting rod under its gas load (`fired`) takes no more than this — counts, not times, so a
/// slower machine passes and a lost speed-up fails.  Every crossing of its boundary is Newton's on
/// the distance's readings, under 3.5 readings a crossing (2.8; bisection took 23), its queries
/// within a fifth of 395,358, and CG over its corners settles in 45 iterations (37).  At four
/// times its reading's size, so the suite pays seconds; its own size is the slow tier's
/// (`the_rods_reading_at_its_own_size_is_pinned`).
#[test]
fn the_rods_reading_is_pinned_by_the_work_it_takes() {
    pinned(4.0, 480_000, 3.5, 45);
}

/// **The rod at its reading's own size** (#510, the slow tier): 77,648 TET10 and 385,368
/// equations, its queries within a fifth of 1,419,046, 2.5 readings a crossing, CG in 40
/// iterations (pinned at 50).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, a minute: the rod at its own size")]
fn the_rods_reading_at_its_own_size_is_pinned() {
    pinned(1.0, 1_700_000, 3.0, 50);
}

fn pinned(coarser: f64, queries: usize, readings: f64, iterations: usize) {
    let (e, solid) = conrod();
    let mesh = gcs_core::bodies::mesh(&e.sketch, solid, &[], coarser).expect("meshed");
    let r = mesh.report.as_ref().expect("a filled body's report");
    let s = &r.surface;
    eprintln!("{} elements: {} queries, {} readings over {} crossings, {} fallbacks",
              mesh.tets.len(), s.queries, s.readings, s.crossings, s.fallbacks);
    assert!(s.crossings > 0 && s.newton == s.crossings && s.fallbacks == 0, "{s:?}");
    assert!((s.readings as f64) < readings * s.crossings as f64, "{s:?}");
    assert!(s.queries <= queries, "{} queries, pinned at {queries}", s.queries);
    let sc = &e.scenarios[0].scenario;
    let (st, x) = gcs_core::bodies::structure(&e.sketch, sc, solid, coarser).expect("a structure");
    let solved = st.solve(&x).expect("solved");
    let its = solved.iterations().expect("solved by CG");
    eprintln!("{} degrees of freedom, CG in {its} iterations", solved.u.len());
    assert!(its <= iterations, "CG took {its} iterations, pinned at {iterations}");
}
