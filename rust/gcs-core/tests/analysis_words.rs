//! **Diagnosis words for analysis** (#510 P7): a rigid-body motion named from the supports, a
//! structure's free motion named, unbounded stress at a re-entrant edge, a degenerate modal pair
//! and the ring that makes it one, and a requirement out of reach of what drives it.  The rules
//! are `docs/contract/analysis.md`'s.
use gcs_core::diagnose::analysis::{
    self, freedoms, freedoms_text, reach, repeated, williams, Freedom, Held, Reach,
};
use gcs_core::eigen::{self, Opts};
use crate::interior::{braced, two_bar};
use crate::linalg::spring;
use gcs_core::interior::{Bar, Failure, Structure};
use gcs_core::rng::Rng;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::sparse::Sym;
use std::f64::consts::PI;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

fn texts(fs: &[Freedom], dim: usize) -> Vec<String> {
    fs.iter().map(|f| f.text(dim)).collect()
}

/// Every freedom moves no held point along a held direction, and turns nowhere a held turn is.
fn assert_free_under(dim: usize, fs: &[Freedom], holds: &[Held]) {
    for f in fs {
        for h in holds {
            if h.turn {
                assert!(matches!(f, Freedom::Slide { .. }), "{f:?} turns under a held turn");
            }
            for p in &h.at {
                let v = f.velocity(*p);
                let dirs: Vec<[f64; 3]> = match h.along {
                    Some(d) => vec![d],
                    None => (0..dim).map(|k| { let mut e = [0.0; 3]; e[k] = 1.0; e }).collect(),
                };
                for d in dirs {
                    let along: f64 = (0..dim).map(|k| v[k] * d[k]).sum();
                    let l: f64 = (0..dim).map(|k| d[k] * d[k]).sum::<f64>().sqrt();
                    let size = 1.0 + p.iter().map(|x| x.abs()).sum::<f64>();
                    assert!(along.abs() <= 1e-9 * l * size,
                            "{f:?} moves {p:?} along {d:?} by {along}");
                }
            }
        }
    }
}

fn pin(p: [f64; 3]) -> Held {
    Held { at: vec![p], along: None, turn: false }
}

/// **A rigid-body motion is named from the supports**: a body held nowhere is free in every
/// direction and about every axis, one pin leaves its three turns about the pin, a pin and a
/// roller leave the turns about the line through them, a clamp holds everything — and in the
/// plane a pin leaves a turn about it and a roller a slide and a turn.  World axes are named.
#[test]
fn a_rigid_body_motion_is_named_from_the_supports() {
    assert_eq!(
        texts(&freedoms(3, &[], &[]), 3),
        ["slide along x", "slide along y", "slide along z", "turn about x through (0, 0, 0)",
         "turn about y through (0, 0, 0)", "turn about z through (0, 0, 0)"]
    );
    let one = [pin([1.0, 2.0, 3.0])];
    assert_eq!(
        texts(&freedoms(3, &[], &one), 3),
        ["turn about x through (1, 2, 3)", "turn about y through (1, 2, 3)",
         "turn about z through (1, 2, 3)"]
    );
    // pinned at the origin, and held along y sixty along x: free about x (the line through both)
    // and about y (which moves the roller along z)
    let roller = [
        pin([0.0; 3]),
        Held { at: vec![[60.0, 0.0, 0.0]], along: Some([0.0, -1.0, 0.0]), turn: false },
    ];
    let fs = freedoms(3, &[], &roller);
    assert_eq!(texts(&fs, 3),
               ["turn about x through (30, 0, 0)", "turn about y through (0, 0, 0)"]);
    assert_free_under(3, &fs, &roller);
    assert_eq!(freedoms_text(&fs, 3).unwrap(),
               "free to turn about x through (30, 0, 0) and turn about y through (0, 0, 0)");
    let clamp = [Held { at: vec![[5.0, 0.0, 0.0]], along: None, turn: true }];
    assert!(freedoms(3, &[], &clamp).is_empty());
    assert_eq!(freedoms_text(&[], 3), None);
    // in the plane
    assert_eq!(texts(&freedoms(2, &[], &[pin([10.0, -4.0, 0.0])]), 2), ["turn about (10, -4)"]);
    let rolled = [Held { at: vec![[10.0, 0.0, 0.0]], along: Some([0.0, 1.0, 0.0]), turn: false }];
    assert_eq!(texts(&freedoms(2, &[], &rolled), 2), ["slide along x", "turn about (10, 0)"]);
    let beam = [pin([0.0; 3]), rolled[0].clone()];
    assert!(freedoms(2, &[], &beam).is_empty());
}

/// Held along random single directions at random points, a body keeps `6 − rank` freedoms, each
/// moving no held point along its direction — a screw among them where the holds couple a slide
/// to a turn — and a twist that moves no point of the body is none of them.
#[test]
fn the_freedoms_are_what_the_supports_leave() {
    let mut rng = Rng::new(7);
    let mut screws = 0;
    for count in 0..=6 {
        for _ in 0..8 {
            let holds: Vec<Held> = (0..count).map(|_| Held {
                at: vec![[0, 1, 2].map(|_| rng.uniform(-10.0, 10.0))],
                along: Some([0, 1, 2].map(|_| rng.uniform(-1.0, 1.0))),
                turn: false,
            }).collect();
            let fs = freedoms(3, &[], &holds);
            assert_eq!(fs.len(), 6 - count, "{count} generic holds");
            assert_free_under(3, &fs, &holds);
            screws += fs.iter()
                .filter(|f| matches!(f, Freedom::Turn { pitch, .. } if *pitch != 0.0))
                .count();
        }
    }
    assert!(screws > 0, "generic holds leave screws");
    let screw = Freedom::Turn { about: [0.0, 0.0, 1.0], through: [1.0, 0.0, 0.0], pitch: 2.5 };
    assert_eq!(screw.text(3), "screw about z through (1, 0, 0), advancing 2.5 a radian");
    // a body all on the x axis, pinned at one end: its turn about x moves none of it
    let rod = [[0.0; 3], [10.0, 0.0, 0.0]];
    assert_eq!(
        texts(&freedoms(3, &rod, &[pin([0.0; 3])]), 3),
        ["turn about y through (0, 0, 0)", "turn about z through (0, 0, 0)"]
    );
}

/// **A structure's free motion is named** (`Structure::free`, P5's `Failure::Free` said): a node
/// held only along a line of bars is a mechanism of that node, a node no bar reaches is one,
/// two bars held at one end turn about it and fold, and a braced structure is held.  A bar in
/// space held only across at its far end turns about that end and slides along itself, and the
/// turn about its own line, which moves no node, is not said.
#[test]
fn a_structure_names_what_leaves_it_free() {
    let line = Structure {
        dim: 2,
        nodes: 3,
        bars: vec![Bar { a: 0, b: 1, e: 1.0 }, Bar { a: 1, b: 2, e: 1.0 }],
        beams: Vec::new(),
        fixed: vec![true, true, false, false, true, true],
        load: vec![0.0, 0.0, 0.0, -1.0, 0.0, 0.0],
        weight: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
    };
    let x = vec![0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 1.0, 1.0];
    assert_eq!(line.solve(&x).err(), Some(Failure::Free));
    let free = line.free(&x);
    assert!(free.rigid.is_empty());
    assert_eq!(free.mechanism, Some(vec![1]));
    assert_eq!(free.text(2).unwrap(), "a mechanism: node 1 moves with no bar stretched");

    let mut lone = line.clone();
    lone.bars.truncate(1);
    lone.fixed = vec![true, true, true, true, false, false];
    assert_eq!(lone.free(&x[..7]).mechanism, Some(vec![2]));

    // two bars to an apex, held at one foot alone: the whole turns about it, and folds
    let two = Structure {
        dim: 2,
        nodes: 3,
        bars: vec![Bar { a: 0, b: 2, e: 1.0 }, Bar { a: 1, b: 2, e: 1.0 }],
        beams: Vec::new(),
        fixed: vec![true, true, false, false, false, false],
        load: vec![0.0; 6],
        weight: vec![0.0; 6],
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
    };
    let xt = vec![-3.0, 0.0, 3.0, 0.0, 0.0, 4.0, 1.0, 1.0];
    let free = two.free(&xt);
    assert_eq!(texts(&free.rigid, 2), ["turn about (-3, 0)"]);
    assert!(free.mechanism.as_ref().is_some_and(|m| m.contains(&1)), "{free:?}");
    assert_eq!(free.text(2).unwrap(),
               "free to turn about (-3, 0); a mechanism: nodes 1 and 2 move with no bar stretched");

    let mut held = two.clone();
    held.fixed = vec![true, true, true, true, false, false];
    assert_eq!(held.free(&xt).text(2), None);
    for dim in [2, 3] {
        let (s, x) = braced(dim, 9, &mut Rng::new(5));
        let free = s.free(&x);
        assert!(free.rigid.is_empty() && free.mechanism == Some(vec![]), "{dim}D: {free:?}");
    }

    let bar = Structure {
        dim: 3,
        nodes: 2,
        bars: vec![Bar { a: 0, b: 1, e: 1.0 }],
        beams: Vec::new(),
        fixed: vec![false, false, false, false, true, true],
        load: vec![0.0; 6],
        weight: vec![0.0; 6],
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
    };
    let xb = vec![0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0];
    let free = bar.free(&xb);
    assert_eq!(
        texts(&free.rigid, 3),
        ["slide along x", "turn about y through (10, 0, 0)", "turn about z through (10, 0, 0)"]
    );
    assert_eq!(free.mechanism, Some(vec![]));
}

/// **Williams' exponent**: ½ at a crack, 0.5445 at three quarters of a turn (each the least root
/// of `sin(λα) + λ sin α`), and 1 — bounded — at a half turn or less.
#[test]
fn williams_exponent_is_the_wedges_least_root() {
    assert!(close(williams(2.0 * PI), 0.5, 1e-12));
    assert!(close(williams(1.5 * PI), 0.544_483_736_782_464, 1e-10), "{}", williams(1.5 * PI));
    for a in [PI, 0.5 * PI, 0.1] {
        assert_eq!(williams(a), 1.0);
    }
    for a in [1.01 * PI, 1.25 * PI, 1.5 * PI, 1.75 * PI, 1.99 * PI] {
        let l = williams(a);
        assert!(l > 0.5 - 1e-12 && l < 1.0, "{a}: {l}");
        assert!(((l * a).sin() + l * a.sin()).abs() < 1e-12, "a root at {a}");
        // and the least: no change of sign below it
        let below = (1..100).all(|i| {
            let m = l * i as f64 / 100.0;
            (m * a).sin() + m * a.sin() > 0.0
        });
        assert!(below, "the least root at {a}");
    }
}

/// An L of plate 60 × 40, its inner corner at (20, 20), with a scenario stating its safety, and
/// the same L with that corner rounded.
fn ell(rounded: bool) -> String {
    format!("\
unit mm
use std (round)
in std.front {{
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  b4 := point
  b5 := point
  fix((0, 0)) b0
  fix((60, 0)) b1
  fix((60, 20)) b2
  fix((20, 20)) b3
  fix((20, 40)) b4
  fix((0, 40)) b5
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (ledge := line(b2, b3)) -> \
(riser := line(b3, b4)) -> (top := line(b4, b5)) -> (west := line(b5, b0)) -> close
  outline := face(south, east, ledge, riser, top, west)
}}
plate := solid(outline, depth: 4mm)
{}
scenario lift {{
  fixed plate.west
  load b1 (0, -500N, 0)
  claim safety(plate) >= 2
}}
scenario loose {{
  pinned b0
}}
", if rounded { "plate.ledge round(r: 3mm) plate.riser" } else { "" })
}

/// Elaborated and solved, and each scenario's words.
fn words(src: &str) -> Vec<(String, Vec<String>)> {
    let e = crate::common::read(src);
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).settled());
    e.scenarios.iter()
        .map(|s| (s.scenario.name.clone(), analysis::scenario_words(&sk, &s.scenario)))
        .collect()
}

/// **A scenario's words** (`scenario_words`): a scenario that clamps a face is held, and one
/// that pins a point leaves the body its turns about it.  A `safety` requirement on a body with
/// a sharp re-entrant edge has no answer — the stress there is unbounded, `r^-0.456` at the L's
/// inner corner — and once that edge is rounded there is no corner to say it of.
#[test]
fn a_scenario_says_what_leaves_it_free_and_where_stress_is_unbounded() {
    let got = words(&ell(false));
    assert_eq!(got[0].0, "lift");
    let [lift] = got[0].1.as_slice() else { panic!("{:?}", got[0].1) };
    let opens = "scenario `lift`: `safety(plate)` has no answer: stress is unbounded at the edge \
                 of `";
    assert!(lift.starts_with(opens), "{lift}");
    for want in ["`ledge`", "`riser`", "near (20, ", "the material's angle is 270°",
                 "r^-0.456 toward the edge; round it"] {
        assert!(lift.contains(want), "{want} in {lift}");
    }
    let [loose] = got[1].1.as_slice() else { panic!("{:?}", got[1].1) };
    assert_eq!(loose, "scenario `loose` leaves its structure free to turn about x through \
                       (0, 0, 0), turn about y through (0, 0, 0) and turn about z through \
                       (0, 0, 0)");

    let rounded = words(&ell(true));
    assert!(rounded[0].1.is_empty(), "{:?}", rounded[0].1);
}

/// A support the drawing cannot read is said, never taken as holding nothing: a face path
/// naming no face of its solid.
#[test]
fn a_support_on_no_face_is_said() {
    let src = ell(false).replace("fixed plate.west", "fixed plate.nowhere");
    let got = words(&src);
    let said = "scenario `lift`: `plate.nowhere` names no face of `plate`";
    assert!(got[0].1.iter().any(|w| w == said), "{:?}", got[0].1);
}

/// **A degenerate modal pair is named, with its ring**: a free ring of 120 masses has every mode
/// twice (`4(k/m) sin²(πj/N)`), and one step round the ring turns mode pair j by `2πj/120` — its
/// wave.  The rigid turn at zero is the supports' word, not a pair, and the tenth value, cut from
/// its pair by the count, is not one.  A pair no step of the ring turns (two equal springs
/// apart) is degenerate but not the ring's.
#[test]
fn a_degenerate_pair_is_named_with_its_ring() {
    let (nodes, k, m) = (120usize, 3.0, 2.0);
    let mut t = Vec::new();
    for a in 0..nodes {
        spring(&mut t, a, (a + 1) % nodes, k);
    }
    let stiff = Sym::from_triplets(nodes, &t);
    let mass = Sym::from_triplets(nodes, &(0..nodes).map(|i| (i, i, m)).collect::<Vec<_>>());
    let got = eigen::lowest(&stiff, &mass, 10, &Opts::default()).expect("the modes");
    let runs = repeated(&got);
    assert_eq!(runs.iter().map(|r| (r.first, r.count)).collect::<Vec<_>>(),
               [(1, 2), (3, 2), (5, 2), (7, 2)]);
    let step = |x: &[f64]| (0..nodes).map(|i| x[(i + nodes - 1) % nodes]).collect::<Vec<f64>>();
    for (j, r) in runs.iter().enumerate() {
        assert_eq!(r.wave(&got, &mass, &step, nodes), Some(j + 1), "pair {j}");
    }
    assert_eq!(
        runs[0].text(Some((1, nodes))),
        "modes 2 and 3 are one degenerate pair, symmetric under the ring of 120 (wave 1): no \
         derivative is defined there, so a requirement reads them together and not either alone"
    );

    // two masses on two equal springs to the ground, and a third stiffer: a pair, but the step
    // that keeps each is no turn of theirs
    let apart = Sym::from_triplets(3, &[(0, 0, 2.0), (1, 1, 2.0), (2, 2, 5.0)]);
    let unit = Sym::from_triplets(3, &[(0, 0, 1.0), (1, 1, 1.0), (2, 2, 1.0)]);
    let got = eigen::lowest(&apart, &unit, 3, &Opts::default()).expect("the modes");
    let runs = repeated(&got);
    assert_eq!(runs.iter().map(|r| (r.first, r.count)).collect::<Vec<_>>(), [(0, 2)]);
    assert_eq!(runs[0].wave(&got, &unit, &|x: &[f64]| x.to_vec(), 4), None);
    assert!(runs[0].text(None).starts_with("modes 1 and 2 are one degenerate pair: "));
}

/// **A requirement out of reach of what drives it**: the apex of two bars sinks
/// `P(w² + h²)^{3/2} / (2EAh²)`, least at `h = √2 w`, where it is `3√3 P w / (4EA)`.  Driving the
/// rise to sink half that, a damped Newton on the row stops at that rise, and the verdict is the
/// most the rise reaches — the closed form's.  Where the rise still moves it there is no
/// verdict, where every number it reads is held it is held, and where it holds it is reached.
#[test]
fn a_requirement_out_of_reach_says_so() {
    let (w, e, a, p) = (300.0, 200e3, 50.0, 10e3);
    let least = 3.0 * 3f64.sqrt() * p * w / (4.0 * e * a);
    let target = -0.5 * least;
    let (s, mut x) = two_bar(w, w, e, a, p);
    let free: Vec<bool> = (0..s.columns()).map(|i| i == 5).collect();
    assert_eq!(s.reach(&x, target, &free), Ok(Reach::Moves));
    // Levenberg on the rise alone: its step shrinks the residual or the step does
    let row = |x: &[f64]| {
        let st = s.solve(x).unwrap();
        let mut g = vec![0.0; s.columns()];
        s.gradient(&st, x, &mut g);
        (s.output(&st) - target, g[5])
    };
    let mut mu = 1e-6;
    for _ in 0..500 {
        let (r, g) = row(&x);
        let step = -r * g / (g * g + mu);
        let mut y = x.clone();
        y[5] += step;
        if row(&y).0.abs() < r.abs() {
            x = y;
            mu *= 0.3;
        } else {
            mu *= 4.0;
        }
        if step.abs() <= 1e-13 * x[5] {
            break;
        }
    }
    assert!(close(x[5], 2f64.sqrt() * w, 1e-6), "the rise stops at {} against {}", x[5],
            2f64.sqrt() * w);
    let got = s.reach(&x, target, &free).unwrap();
    let Reach::Extreme { value, least: false } = got else { panic!("{got:?}") };
    assert!(close(value, -least, 1e-9), "{value} against {}", -least);
    assert_eq!(
        got.text(&|v| format!("{v:.4}mm")).unwrap(),
        format!("out of reach of the dimensions that drive it: {:.4}mm is the most they reach here",
                -least)
    );
    assert_eq!(s.reach(&x, target, &vec![false; s.columns()]), Ok(Reach::Held));
    assert_eq!(Reach::Held.text(&|v| v.to_string()).unwrap(),
               "out of reach: every number it reads is held, so nothing drives it");
    assert_eq!(s.reach(&x, value, &free), Ok(Reach::Reached));
    assert_eq!(reach(1.0, 1.0 + 1e-12, &[0.0], &[1.0], &[true]), Reach::Reached);
}
