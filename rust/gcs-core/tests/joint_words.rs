//! **A frame's joint as relation words** (Solvent §6.14 [0.153]; issue #464): `std.sv` states a
//! joint as a word with the frame the subject and what it moves about or along the object —
//! `f turns(r) l`, `f screws(a) l`, `f spins(r) o`, `f slides(a) l` — each its one number bare.
//! Each moves the frame as the component it replaces did (`std.Turn`, `std.Spin`, `std.Slide`,
//! kept for one release), and no document of the library calls those components any more.
use std::path::Path;

use gcs_core::motion::Family;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{ent, read, refused};
use crate::every_example::sources;

/// A point `o`, a line `l` up and to the right, and a point `x` off both, held in the front; `m`
/// a frame whose joint `joint` states.
fn drawing(joint: &str) -> String {
    format!("\
unit mm
use std (turns, screws, spins, slides)
in std.front {{
  o := point
  fix((5, 2)) o
  a := point
  fix((0, 0)) a
  b := point
  fix((10, 20)) b
  l := line(a, b)
  x := point
  fix((40, 10)) x
}}
m := frame
{joint}
")
}

/// Where `m`'s motion carries `x` at each of `degrees`, solved.
fn carried(joint: &str, degrees: &[f64]) -> Vec<[f64; 3]> {
    let e = read(&drawing(joint));
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{joint}: {}", r.message);
    let m = sk.motions.iter().position(|m| m.name == "m").unwrap_or_else(|| panic!("{joint}"));
    let family = Family::read(&sk, m).unwrap();
    let x = sk.world_point(ent(&e, "x").i());
    degrees.iter().map(|&d| family.at(d.to_radians()).unwrap().point(x)).collect()
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
}

const ROLLS: [f64; 5] = [-150.0, -30.0, 0.0, 45.0, 360.0];

/// **Each word is the component it replaces**: the frame stands where the component's did at
/// every roll, a turn, a screw, a turn in the view and a slide.
#[test]
fn each_joint_word_moves_the_frame_as_its_component_did() {
    for (word, component) in [
        ("m turns(-1.5) l", "std.Turn(m, about: l, ratio: -1.5)"),
        ("m screws(4mm) l", "std.Turn(m, about: l, ratio: 1, advance: 4mm)"),
        ("m screws(-4mm) l", "std.Turn(m, about: l, ratio: 1, advance: -4mm)"),
        ("m spins(2) o", "std.Spin(m, about: o, on: std.front, ratio: 2)"),
        ("m slides(6mm) l", "std.Slide(m, along: l, advance: 6mm)"),
    ] {
        let (said, was) = (carried(word, &ROLLS), carried(component, &ROLLS));
        for ((s, w), deg) in said.iter().zip(&was).zip(ROLLS) {
            assert!(near(*s, *w), "`{word}` at {deg}°: {s:?}, `{component}` {w:?}");
        }
    }
}

/// What each moves, said directly: a slide `6` a turn along `l` and turning not at all; a screw
/// `4` a turn along `l` and back where it turned from after a whole turn; a turn in the view
/// about `o`, square to the front, twice the roll.
#[test]
fn each_joint_word_says_what_it_moves() {
    let [x, turn, back] = carried("m slides(6mm) l", &[0.0, 360.0, -60.0])[..] else { panic!() };
    let (len, dir) = (500f64.sqrt(), [10.0, 0.0, 20.0]);
    let along = |k: f64| [x[0] + k * dir[0] / len, x[1], x[2] + k * dir[2] / len];
    assert!(near(turn, along(6.0)) && near(back, along(-1.0)), "{turn:?} {back:?}");
    assert!(near(carried("m screws(4mm) l", &[360.0])[0], along(4.0)));
    // about `o` = (5, 0, 2) square to the front, counter-clockwise as the front is drawn
    let at = carried("m spins(2) o", &[45.0])[0];
    let (s, c) = 90f64.to_radians().sin_cos();
    let (dx, dz) = (40.0 - 5.0, 10.0 - 2.0);
    let want = [5.0 + c * dx - s * dz, x[1], 2.0 + s * dx + c * dz];
    assert!(near(at, want), "{at:?} against {want:?}");
}

/// `spins` reads its view from its point: one standing in space is drawn in none, E040 where the
/// word is written.
#[test]
fn a_spin_about_a_point_in_space_is_refused() {
    refused(
        "unit mm\nuse std (spins)\np := point hint((1, 2, 7))\nm := frame\nm spins(1) p\n",
        "E040",
        "drawn in no plane",
        "m spins(1) p",
    );
}

/// **A joint a word holds is not written again at the root**: the word's `fix` is its body's, so
/// the editor's reconcile after a solve leaves each joint where the word says it, as it did the
/// component's (`SourceMap::held_within`).
#[test]
fn a_hold_a_word_says_is_not_written_again_at_the_root() {
    for joint in ["m turns(2) l", "m screws(4mm) l", "m spins(1) o", "m slides(3mm) l"] {
        let src = drawing(joint);
        let mut e = read(&src);
        assert!(solve(&mut e.sketch, SolveOpts::default()).success, "{joint}");
        let sk = std::mem::take(&mut e.sketch);
        let edit = gcs_core::edit::reconcile(&mut e, &sk);
        assert!(edit.refused.is_none(), "{joint}: {:?}", edit.refused);
        assert_eq!(edit.text.matches("fix(").count(), src.matches("fix(").count(), "{}", edit.text);
        assert!(edit.text.contains(joint), "{}", edit.text);
        read(&edit.text);
    }
}

/// **The library says a joint in words**: no document under `rust/examples/` and no module of
/// `rust/lib/` calls `std.Turn`, `std.Spin` or `std.Slide` (#464's 67 calls, migrated).
#[test]
fn no_document_calls_a_joint_component() {
    let rust = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let mut files = Vec::new();
    sources(&rust.join("examples"), &mut files);
    sources(&rust.join("lib"), &mut files);
    assert!(files.len() > 100, "only {} documents", files.len());
    let calls: Vec<String> = files.iter()
        .flat_map(|p| {
            let text = std::fs::read_to_string(p).unwrap();
            text.lines().enumerate()
                .filter(|(_, l)| {
                    // `std.Turn(` in a document, `Turn(` bare in `std.sv`, and not `Turned(`
                    let code = l.split("//").next().unwrap_or("");
                    !code.trim_start().starts_with("component ")
                        && ["Turn(", "Spin(", "Slide("].iter().any(|c| {
                            code.match_indices(c).any(|(i, _)| {
                                !code[..i].ends_with(|ch: char| ch.is_alphanumeric() || ch == '_')
                            })
                        })
                })
                .map(|(n, l)| format!("{}:{}: {}", p.display(), n + 1, l.trim()))
                .collect::<Vec<_>>()
        })
        .collect();
    assert!(calls.is_empty(), "{calls:#?}");
}
