//! **A load scenario seen in the workspace** (#510): where its lines fall on the drawing
//! (`overview::scenario::marks`, no analysis made) and how the drawing moves under it
//! (`deformed::Picture`, each body it falls on drawn as the mesh that was solved).  The rules are
//! `docs/contract/analysis.md`'s, "A scenario drawn".
use gcs_core::deformed::{self, Picture};
use gcs_core::diagnose::analysis::{direction, scenario_pictured};
use gcs_core::model::{Applied, EntKind, Output};
use gcs_core::overview::scenario::{marks, Kind};
use gcs_core::space::{distance, dot};

/// The library's documents that state a scenario.
const LIBRARY: [&str; 16] = [
    "cantilever_plate", "pressure_pipe", "tuning_fork", "shelf_bracket", "pin_lug",
    "flat_bar_strut", "rounded_bracket", "pressed_panel", "sized_cantilever", "portal_frame",
    "turned_shaft", "drilled_plate", "reach_cantilever", "sized_plate", "gravity_wall",
    "sized_plank",
];

/// A steel cantilever `L` long along x on the front, built in at `root`, and `P` at its tip along
/// `down`: its section bends in the front about the front's normal, so its tip sinks
/// `P L³ / 3 E Iz`.  `hold` is how its root is held.
fn cantilever(hold: &str) -> String {
    format!("\
unit mm
use std
L := 1000mm
P := 100N
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
in std.front {{
  root := point
  tip := point
  fix((0mm, 0mm)) root
  fix((L, 0mm)) tip
  arm := line(root, tip)
}}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
bar := beam(arm, A: 100 * 1mm^2, Iy: 1000 * 1mm^4, Iz: 2000 * 1mm^4, J: 500 * 1mm^4) of steel
scenario tip_load {{
  {hold} root
  load tip P along: down
}}
")
}

/// The vertices of `p` nearest `at` (every one within a hair of the least distance): a frame's
/// ring about a place of its centre line, a node of a region or a solid.
fn nearest(p: &Picture, at: [f64; 3]) -> Vec<([f64; 3], [f64; 3])> {
    let all: Vec<_> = p.bodies.iter()
        .flat_map(|b| b.rest.iter().copied().zip(b.moved.iter().copied())).collect();
    let least = all.iter().map(|(r, _)| distance(*r, at)).fold(f64::INFINITY, f64::min);
    let near = least + 1e-9 * p.extent;
    all.into_iter().filter(|(r, _)| distance(*r, at) <= near).collect()
}

/// **A frame's picture is its cubic** (#510): drawn under supports and loads alone — no
/// requirement asks for it — its tip moves `P L³ / 3 E Iz`, and a place sampled inside an element
/// moves as the closed form `P x² (3L − x) / 6 E Iz` says, every vertex of the ring about it, to
/// 1e-9 of the tip's move.  The beam is cut into `PIECES` elements, each a tube of `SIDES`.
#[test]
fn a_cantilevers_picture_is_its_cubic() {
    let e = crate::common::text::solved(&cantilever("fixed"));
    let (words, _, p) = scenario_pictured(&e.sketch, &e.scenarios[0].scenario);
    assert!(words.is_empty() && p.words.is_empty(), "{words:?} {:?}", p.words);
    assert_eq!(p.bodies.len(), 1);
    let b = &p.bodies[0];
    assert_eq!(b.kind, "frame");
    let rings = gcs_core::beams::PIECES * deformed::SAMPLES;
    assert_eq!(b.rest.len(), rings * deformed::SIDES);
    // the document's units: E in kg/(mm s²), P in kg mm/s²
    let (young, force, iz, l) = (200e9 * 1e-3, 100.0 * 1e3, 2000.0, 1000.0);
    let sag = |x: f64| force * x * x * (3.0 * l - x) / (6.0 * young * iz);
    let tip = sag(l);
    assert!((p.largest - tip).abs() <= 1e-9 * tip, "the tip moves {}, not {tip}", p.largest);
    for x in [l, l / 2.0, l / 32.0, 3.0 * l / 32.0] {
        let ring = nearest(&p, [x, 0.0, 0.0]);
        assert!(!ring.is_empty());
        for (_, u) in ring {
            assert!((u[2] + sag(x)).abs() <= 1e-9 * tip, "at {x}: {u:?}, not {}", -sag(x));
            assert!(u[0].abs() <= 1e-9 * tip && u[1].abs() <= 1e-9 * tip, "at {x}: {u:?}");
        }
    }
    // a tenth of the extent, rounded down to 1, 2 or 5 of a decade, and never below true scale
    assert_eq!(p.scale, 1.0, "a tip sagging {tip} on a beam {l} long is drawn as it is");
    assert_eq!(deformed::factor(p.extent, tip / 100.0), 100.0);
}

/// **What the picture is drawn at** (#510): the largest move as a tenth of the extent, rounded
/// down to 1, 2 or 5 times a power of ten, and never below true scale.
#[test]
fn the_default_factor_is_a_tenth_of_the_extent_rounded_down() {
    assert_eq!(deformed::factor(1000.0, 0.0137), 5000.0);
    assert_eq!(deformed::factor(1000.0, 0.05), 2000.0);
    assert_eq!(deformed::factor(100.0, 0.1), 100.0);
    assert_eq!(deformed::factor(100.0, 0.3), 20.0);
    assert_eq!(deformed::factor(10.0, 5.0), 1.0);
    assert_eq!(deformed::factor(100.0, 0.0), 1.0);
}

/// **A frame its supports leave free is not drawn, and says why** (#510): pinned at its root, a
/// cantilever turns there freely — no displacement to give, so no body, and the solver's words.
#[test]
fn a_frame_left_free_draws_no_deflection_and_says_why() {
    let e = crate::common::text::solved(&cantilever("pinned"));
    let (_, _, p) = scenario_pictured(&e.sketch, &e.scenarios[0].scenario);
    assert!(p.bodies.is_empty(), "{:?}", p.bodies.len());
    assert!(p.words.iter().any(|w| w.contains("free")), "{:?}", p.words);
    assert_eq!(p.largest, 0.0);
}

/// **The picture is the answers' reading** (#510): over every scenario of the library, each
/// deflection of a point along a direction that has an answer is the move along it of the
/// vertices drawn nearest that point, to 1e-9 of the largest move — the same bodies, analysed
/// once for both — and every body a scenario falls on is drawn, with nothing said against it.
#[test]
fn the_picture_moves_each_point_by_its_answer() {
    for key in LIBRARY {
        let Some(src) = gcs_core::examples::source(key) else { panic!("{key} is no case") };
        let e = crate::common::read(src);
        let sk = crate::common::solved(&e);
        for s in &e.scenarios {
            let (_, read, p) = scenario_pictured(&sk, &s.scenario);
            assert!(!p.bodies.is_empty(), "{key}: nothing drawn, {:?}", p.words);
            assert!(p.words.is_empty(), "{key}: {:?}", p.words);
            // a scenario of modes alone loads nothing, so nothing moves
            let loaded = !s.scenario.loads.is_empty();
            assert!((p.largest > 0.0) == loaded && p.scale >= 1.0, "{key}: {} ×{}", p.largest,
                    p.scale);
            for b in &p.bodies {
                assert_eq!(b.rest.len(), b.moved.len());
                let n = b.rest.len() as u32;
                assert!(b.triangles.iter().flatten().all(|&i| i < n), "{key}");
                assert!(b.edges.iter().flatten().all(|&i| i < n), "{key}");
            }
            for (k, r) in s.scenario.requirements.iter().enumerate() {
                let (Output::Deflection, Applied::Ent(at), Some(along)) = (r.output, &r.of, r.along)
                else { continue };
                if at.kind != EntKind::Point {
                    continue;
                }
                let Some(Ok(v)) = read.answers[k] else { continue };
                let d = direction(&sk, along).unwrap();
                for (_, u) in nearest(&p, sk.world_point(at.i())) {
                    assert!((dot(u, d) - v).abs() <= 1e-9 * p.largest,
                            "{key}: `{}` {} moves {} along it, its answer {v}",
                            s.scenario.name, s.heads[k], dot(u, d));
                }
            }
        }
    }
}

/// **Every line of a scenario is a mark** (#510): over the library, each support, load,
/// requirement and reading in the plane is drawn at least once, in the order written, with its
/// words, and its span in the document is the line it was written on.
#[test]
fn every_line_of_a_scenario_is_a_mark() {
    for key in LIBRARY {
        let src = gcs_core::examples::source(key).unwrap();
        let e = crate::common::read(src);
        for (k, s) in e.scenarios.iter().enumerate() {
            let sc = &s.scenario;
            let ms = marks(&e.sketch, s);
            let lines = sc.supports.len() + sc.loads.len() + sc.requirements.len()
                + sc.readings.len();
            assert_eq!(ms.len(), lines, "{key}: {ms:?}");
            for m in &ms {
                assert!(!m.at.is_empty() && !m.text.is_empty(), "{key}: {m:?}");
                assert!(m.dir.is_empty() || m.dir.len() == m.at.len(), "{key}: {m:?}");
                assert!(m.size > 0.0 && m.size <= 1.0, "{key}: {m:?}");
            }
            let json = gcs_core::report::scenario_marks_json(&e, &e.sketch, k);
            for m in json.arr() {
                let span = m.get("span").unwrap().arr();
                let (lo, hi) = (span[0].as_i64() as usize, span[1].as_i64() as usize);
                let line = &src[lo..hi];
                let first = line.split(|c: char| c.is_whitespace() || c == '(').next().unwrap();
                let kind = m.get("kind").unwrap().as_str();
                let written = match kind {
                    "requirement" => first == "claim" || Output::parse(first).is_some(),
                    kind => first == kind,
                };
                assert!(written, "{key}: a {kind} mark's line is `{line}`");
            }
        }
    }
}

/// **A pressure points into its face** (#510): the panel's pressure on its near cap is drawn at
/// places spread over the cap, each pushing against the cap's outward normal there, the cap
/// tinted; a support on a side is drawn on that side, its normals square to the pressure's.
#[test]
fn a_pressure_points_into_its_face() {
    let e = crate::common::read(gcs_core::examples::source("pressed_panel").unwrap());
    let s = &e.scenarios[0];
    let ms = marks(&e.sketch, s);
    let pressure = ms.iter().find(|m| m.kind == Kind::Pressure).expect("a pressure");
    assert!(pressure.at.len() > 4 && !pressure.tint.is_empty(), "{pressure:?}");
    for (d, n) in pressure.dir.iter().zip(&pressure.normal) {
        assert!((dot(*d, *n) + 1.0).abs() <= 1e-9, "{d:?} against {n:?}");
    }
    let n = pressure.normal[0];
    for side in ms.iter().filter(|m| m.kind == Kind::Fixed) {
        assert!(!side.tint.is_empty(), "{side:?}");
        for m in &side.normal {
            assert!(dot(*m, n).abs() <= 1e-9, "a side's normal {m:?} against the cap's {n:?}");
        }
    }
}

/// **A mark adds nothing to the drawing** (#510): the sketch's JSON and its parameters are what
/// they were before every scenario of the library was drawn.
#[test]
fn a_mark_adds_nothing_to_the_drawing() {
    for key in LIBRARY {
        let e = crate::common::read(gcs_core::examples::source(key).unwrap());
        let before = gcs_core::io::to_json(&e.sketch).dump(None);
        let x: Vec<f64> = e.sketch.params.iter().map(|p| p.value).collect();
        for k in 0..e.scenarios.len() {
            gcs_core::report::scenario_marks_json(&e, &e.sketch, k);
        }
        assert_eq!(gcs_core::io::to_json(&e.sketch).dump(None), before, "{key}");
        let after: Vec<f64> = e.sketch.params.iter().map(|p| p.value).collect();
        assert_eq!(after, x, "{key}");
    }
}

/// **The answers beside a picture are the answers** (#510): `scenarios_pictured_json`'s
/// scenarios are `scenarios_json`'s, word for word, and its picture is the one asked for —
/// `null` when none is.
#[test]
fn the_answers_beside_a_picture_are_the_answers() {
    let e = crate::common::read(gcs_core::examples::source("shelf_bracket").unwrap());
    let sk = crate::common::solved(&e);
    let alone = gcs_core::report::scenarios_json(&sk, &e.scenarios).dump(None);
    let drawn = gcs_core::report::scenarios_pictured_json(&sk, &e.scenarios, Some(0));
    assert_eq!(drawn.get("scenarios").unwrap().dump(None), alone);
    let p = drawn.get("picture").unwrap();
    assert_eq!(p.get("bodies").unwrap().arr().len(), 1);
    let b = &p.get("bodies").unwrap().arr()[0];
    assert_eq!(b.get("rest").unwrap().arr().len(), b.get("moved").unwrap().arr().len());
    assert_eq!(b.get("size").unwrap().arr().len() * 3, b.get("rest").unwrap().arr().len());
    let none = gcs_core::report::scenarios_pictured_json(&sk, &e.scenarios, None);
    assert!(matches!(none.get("picture"), Some(gcs_core::json::Json::Null)));
}
