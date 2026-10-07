//! `ring N about C { … }` (Solvent §12.3–12.6, issue #96): copies that are one another's turns
//! about a centre, solved over the first — the representative — with the rest worked out from
//! it (`model::Turn`).  Held here against the same drawing stated as a `cycle` with every turn
//! written out, and against each refusal the spec names.

use crate::common::{at_path as ent, read, refused};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::Sketch;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::system::System;

fn xy(sk: &Sketch, e: gcs_core::model::EntRef) -> (f64, f64) {
    let p = &sk.points[e.i()];
    (sk.params[p.x as usize].value, sk.params[p.y as usize].value)
}

fn solved(src: &str) -> (gcs_core::program::Elaborated, Sketch) {
    let e = read(src);
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    (e, sk)
}

const RING: &str = "
unit mm
use std
in std.front {
  hub := point
  fix((1, 2)) hub
  ring 3 about hub {
    tip := point hint((11, 3))
    hub distance(10) tip
    chord := line(tip, next.tip)
  }
  hub horizontal tip[0]
}
";

/// The same triangle as a `cycle`, every turn written out: the oracle.
const CYCLE: &str = "
unit mm
use std
in std.front {
  hub := point
  fix((1, 2)) hub
  cycle 3 as i {
    tip := point hint((1 + 10 * cos(i * 120deg), 2 + 10 * sin(i * 120deg)))
    hub distance(10) tip
    spoke := line(hub, tip)
    chord := line(tip, next.tip)
  }
  spoke[0] angle(120deg) spoke[1]
  spoke[1] angle(120deg) spoke[2]
  hub horizontal tip[0]
}
";

#[test]
fn a_ring_is_its_cycle_with_the_turns_held_and_one_copy_solved() {
    let (re, rs) = solved(RING);
    let (ce, cs) = solved(CYCLE);
    for k in 0..3 {
        let (a, b) = (xy(&rs, ent(&re, &format!("tip[{k}]"))), xy(&cs, ent(&ce, &format!("tip[{k}]"))));
        assert!((a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9, "tip[{k}]: {a:?} {b:?}");
    }
    // one tip is the unknowns, against the cycle's three
    assert_eq!(System::new(&rs).n_free, 2);
    assert_eq!(System::new(&cs).n_free, 6);
    // the copies are entities like any other: three tips, three chords
    assert_eq!(rs.lines.len(), cs.lines.len() - 3);
}

#[test]
fn a_ring_states_no_closing_turn() {
    let (_, mut sk) = solved(RING);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.n_params, d.dof, d.n_redundant), (2, 0, 0), "{d:?}");
    assert!(d.warnings.is_empty(), "{:?}", d.warnings);
}

#[test]
fn a_row_on_a_copy_moves_the_representative() {
    let src = RING.replace("  hub horizontal tip[0]\n", "  tip[1] distance(4, along: x) hub\n");
    let (e, sk) = solved(&src);
    let (h, t1) = (xy(&sk, ent(&e, "hub")), xy(&sk, ent(&e, "tip[1]")));
    assert!((h.0 - t1.0 - 4.0).abs() < 1e-9);
    let t0 = xy(&sk, ent(&e, "tip[0]"));
    let th = std::f64::consts::TAU / 3.0;
    let (dx, dy) = (t0.0 - h.0, t0.1 - h.1);
    assert!((h.0 + th.cos() * dx - th.sin() * dy - t1.0).abs() < 1e-9, "still a turn");
}

#[test]
fn a_ring_about_a_held_axis_turns_points_in_space() {
    let (e, sk) = solved("
unit mm
use std
ring 4 about std.z {
  p := point hint((5, 0, 2))
  fix((5, 0, 2)) p
}
");
    let p = ent(&e, "p[1]");
    let w = sk.world_point(p.i());
    assert!(w[0].abs() < 1e-12 && (w[1] - 5.0).abs() < 1e-12 && w[2] == 2.0, "{w:?}");
}

#[test]
fn an_index_read_in_a_ring_is_refused() {
    refused("
unit mm
use std
in std.front {
  hub := point
  ring 3 about hub as k {
    tip := point hint((k, 0))
  }
}
", "E015", "is a `ring`'s index", "k");
}

#[test]
fn what_a_turn_would_move_is_refused_inside_a_ring() {
    let head = "unit mm\nuse std\nin std.front {\n  hub := point\n  o := point\n  l := line(hub, o)\n";
    let ring = |body: &str| format!("{head}  ring 3 about hub {{\n    tip := point\n    {body}\n  }}\n}}\n");
    refused(&ring("tip coincident l"), "E021", "does not turn with it", "l");
    refused(&ring("tip distance(3) o"), "E021", "does not turn with it", "o");
    refused(&ring("tip distance(3, along: u) std.front"), "E021", "does not turn with it", "std.front");
    refused(&ring("tip distance(3) tip[2]"), "E021", "reach a neighbour", "tip[2]");
    // the centre, a circle about it, a neighbour by `next`: all read alike from every copy
    read(&format!("{head}  k := circle(center: hub) hint(r: 4)\n  ring 3 about hub {{\n    tip := \
        point\n    tip coincident k\n    tip distance(3) next.tip\n    hub distance(4) tip\n  }}\n}}\n"));
}

#[test]
fn a_ring_in_a_ring_is_refused() {
    refused("
unit mm
use std
in std.front {
  hub := point
  ring 3 about hub {
    q := point
    ring 2 about hub {
      p := point
    }
  }
}
", "E022", "inside a `ring`", "ring 2 about hub {\n      p := point\n    }");
}

#[test]
fn what_a_ring_cannot_turn_is_refused() {
    let doc = |body: &str| format!("unit mm\nuse std\n{body}\n");
    // about a line
    refused(&doc("in std.front {\n  a := point\n  a2 := point\n  l := line(a, a2)\n  ring 3 about l {\n    p := point\n  }\n}"),
        "E023", "turns about a point or an axis", "l");
    // a hold on a copy
    refused(&doc("in std.front {\n  hub := point\n  ring 3 about hub {\n    p := point\n  }\n  fix((1, 1)) p[1]\n}"),
        "E023", "hold that instead", "ring 3 about hub {\n    p := point\n  }");
    // an axis whose direction is free
    refused(&doc("t := axis hint(dir: (0, 0, 1))\nring 3 about t {\n  p := point hint((1, 0, 0))\n}"),
        "E023", "is not held", "t");
    // a point drawn in a view about an axis
    refused(&doc("ring 3 about std.z {\n  p := point in std.front\n}"),
        "E023", "is not a point in space", "ring 3 about std.z {\n  p := point in std.front\n}");
}

#[test]
fn a_ring_names_its_centre() {
    let (_, errs) = crate::common::parse("hub := point\nring 3 {\n  p := point\n}\n");
    assert!(errs.iter().any(|e| e.message.contains("`ring N about C")), "{errs:?}");
}

#[test]
fn a_solve_writes_the_representatives_seed() {
    let (prog, _, _) = gcs_core::library::parse_linked(RING);
    let e = gcs_core::program::elaborate(&prog);
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let edit = gcs_core::edit::commit_seeds(&e, &sk, &prog);
    // one statement, three copies: the first's pose is written, the turns follow from it
    assert!(edit.text.contains("tip := point hint((11, 2))"), "{}", edit.text);
}

#[test]
fn dragging_a_copy_turns_the_representative() {
    // free to turn about the held hub
    let (e, mut sk) = solved(&RING.replace("  hub horizontal tip[0]\n", ""));
    let (t0, t1) = (ent(&e, "tip[0]"), ent(&e, "tip[1]"));
    let (x, y) = xy(&sk, t1);
    let mut d = gcs_core::decompose::PlanDrag::new(&sk, t1.i(), x, y, None, 0.05);
    for k in 1..=10 {
        // round to straight above the hub
        let (tx, ty) = (x + (1.0 - x) * k as f64 / 10.0, y + (12.0 - y) * k as f64 / 10.0);
        assert!(d.move_to(&mut sk, None, tx, ty).success);
    }
    d.end();
    let (a, b) = (xy(&sk, t1), xy(&sk, t0));
    assert!((a.0 - 1.0).abs() < 1e-6 && (a.1 - 12.0).abs() < 1e-6, "{a:?}");
    // the representative a third of a turn back: at -30°
    let th = (-30f64).to_radians();
    assert!((b.0 - (1.0 + 10.0 * th.cos())).abs() < 1e-6 && (b.1 - (2.0 + 10.0 * th.sin())).abs() < 1e-6, "{b:?}");
}

/// A curve written in place reads its entities through its instance's arguments: a datum line
/// from outside the ring is read by the representative's tooth alone, and is refused as any
/// reference is (the traced gear gives each tooth a spoke of its own instead).
#[test]
fn a_curve_reading_what_the_turn_would_move_is_refused() {
    let src = gcs_core::examples::GEAR_TRACE.replace(
        "  fix((0, 0)) center\n",
        "  fix((0, 0)) center\n  far := point\n  fix((R, 0)) far\n  outside := line(center, far)\n",
    ).replace("t := Tooth(base, datum,", "t := Tooth(base, outside,");
    refused(&src, "E021", "`datum` is outside the `ring`", "ring N about center {\n    // the first tooth's datum points along x; the others are turned with their teeth\n    private anchor := point\n    fix((R, 0)) anchor\n    private datum := line(center, anchor)\n    t := Tooth(base, outside, root, tip, a0: 0deg, half: half, u0: u0, u1: u1)\n    gap := line(t.l.lo, next.t.r.lo)\n  }");
}

#[test]
fn a_plane_in_a_ring_and_a_relation_on_a_turned_curve_are_refused() {
    refused("
unit mm
use std
hub := point in std.front
ring 3 about hub {
  q := point in std.front
  p := plane(u: std.x, v: std.z)
}
", "E023", "declare it outside", "ring 3 about hub {\n  q := point in std.front\n  p := plane(u: std.x, v: std.z)\n}");
    // the gear's first flank of the second tooth, read from outside the ring
    let src = gcs_core::examples::GEAR.replace(
        "    gap := line(t.l.lo, next.t.r.lo)\n  }\n",
        "    gap := line(t.l.lo, next.t.r.lo)\n  }\n  probe := point\n  probe coincident t[1].r.e\n",
    );
    refused(&src, "E023", "turned copy of a curve", "probe coincident t[1].r.e");
}
