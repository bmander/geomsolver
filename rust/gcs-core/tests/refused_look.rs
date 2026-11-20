//! **A look at a solid whose round the kernel refuses** (#961): the solid is drawn from its
//! material field (`EvaluatedSolid::unbuilt`, solids.md), and that field reads the round point by
//! point, each value a nearest-point search over the faces it rounds.  A mesher reads such a field
//! from an adaptive distance field at the resolution its pass needs (`SpatialField::cached`), as
//! it reads a sweep, so the searches follow the facets drawn; a field read in closed form is read
//! exactly, as ever.  And the refusal is known before the refinement starts, so it is said first
//! (`Sketch::field_meshed_ahead`).
use gcs_core::solid::{ApproximationPolicy, DrawnFrom, MaterialField, Query, Resolution, Source, Want};

use crate::common::{ent, text::solved};

/// A post 14 high on a D — an arc of radius 5 about the origin closed by a chord at `x = 3` —
/// cut by a lid tilted 20° about the y axis through (0, 0, 10), its rim along the arc `round`ed:
/// to 5, the post's own radius, the ball stands still on the axis and the meeting is a stretch of
/// the top's loop ending where the chord's face cuts across it, which the kernel refuses
/// (`fillet::standing`, "no stretch of its dome is carried past and cut off by a face yet"); to 2,
/// it rolls, and its end against the chord is refused instead.
fn tilted_d_post(round: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids (cut, union, common)\nuse std.rounds (round)\n\
        in std.top {{\n  o := point\n  fix((0, 0)) o\n  a := point\n  fix((3, -4)) a\n  b := point\n\
        \x20 fix((3, 4)) b\n  side := arc(center: o, start: b, end: a) hint(r: 5)\n  flat := line(a, b)\n}}\n\
        post := solid(std.solids.Prism(face(side, flat), from: 0mm, to: 14mm))\n\
        across := axis hint(dir: (0, 1, 0))\nacross parallel std.y\n\
        u := axis hint(dir: (0.94, 0, -0.34))\nu perpendicular across\nu angle(20deg, about: std.y) std.x\n\
        slant := plane(u: u, v: across)\nfix(origin == (0mm, 0mm, 10mm)) slant\n\
        in slant {{\n  p := point\n  q := point\n  r := point\n  s := point\n\
        \x20 fix((-20, -20)) p\n  fix((20, -20)) q\n  fix((20, 20)) r\n  fix((-20, 20)) s\n\
        \x20 (pq := line(p, q)) -> (qr := line(q, r)) -> (rs := line(r, s)) -> (sp := line(s, p)) -> close\n}}\n\
        lid := solid(std.solids.Prism(face(pq, qr, rs, sp), from: 0mm, to: 30mm))\n\
        drum := solid(post)\nlid cut drum\ndrum.lid {round} drum.post.side\n")
}

/// The same D square-topped, its arc's rim rounded to the post's radius: refused by the lowering
/// itself, before the kernel is asked ("a stretch of a circle its end does not cut off").
fn square_d_post() -> String {
    "unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\nin std.top {\n  o := point\n  fix((0, 0)) o\n\
     \x20 a := point\n  fix((3, -4)) a\n  b := point\n  fix((3, 4)) b\n\
     \x20 side := arc(center: o, start: b, end: a) hint(r: 5)\n  flat := line(a, b)\n}\n\
     drum := solid(std.solids.Prism(face(side, flat), from: 0mm, to: 10mm))\n\
     drum.near round(r: 5mm) drum.side\n".to_string()
}

/// A field's reading at `p` from a mesher at `resolution` (`Source::Cached`), and exactly.
fn read(field: &MaterialField, p: [f64; 3], resolution: Resolution) -> (f64, f64) {
    let cached = field.query(p, &mut Query { source: Source::Cached(resolution), ..Query::at(p) }).value;
    (cached, field.query(p, &mut Query::at(p)).value)
}

/// Points on a lattice through the box `lo`..`hi`, `n` a side.
fn lattice(lo: [f64; 3], hi: [f64; 3], n: usize) -> Vec<[f64; 3]> {
    let at = |k: usize, i: usize| lo[k] + (hi[k] - lo[k]) * (i as f64 + 0.37) / n as f64;
    (0..n * n * n).map(|i| [at(0, i % n), at(1, i / n % n), at(2, i / (n * n))]).collect()
}

/// Each refusal, the kernel's and the lowering's, is known before the field is meshed, in the
/// words an export of it says; asking meshes nothing.  A round the kernel builds has nothing to
/// say, and nor does a solid with no round.
#[test]
fn a_refused_round_is_said_before_its_field_is_meshed() {
    for (src, why) in [(tilted_d_post("round(r: 5mm)"),
                        "stands still, and their meeting ends where `post.flat` cuts across it"),
                       (square_d_post(), "a stretch of a circle its end does not cut off")] {
        let e = solved(&src);
        let i = ent(&e, "drum").i();
        let clock = std::time::Instant::now();
        let ahead = e.sketch.field_meshed_ahead(i).expect("the field is to be meshed");
        assert!(clock.elapsed().as_secs_f64() < 2.0, "asking meshed nothing");
        assert_eq!(ahead.from, DrawnFrom::Field);
        assert!(ahead.why.contains(why), "{}", ahead.why);
    }
    let e = solved(&tilted_d_post("round(r: 5mm)"));
    assert_eq!(e.sketch.field_meshed_ahead(ent(&e, "post").i()), None);
    let dome = solved(include_str!("../../examples/rounds/dome.sv"));
    assert_eq!(dome.sketch.field_meshed_ahead(ent(&dome, "drum").i()), None);
}

/// A mesher reads a field that searches from its adaptive distance field: within the cells' reach
/// of the exact value everywhere, its sign wherever the surface is farther than a cell, and the
/// same field read exactly by anything else.  One read in closed form — the post alone, a prism —
/// is read exactly whatever is asked, bit for bit.
#[test]
fn a_mesher_reads_a_round_that_searches_from_its_distance_field() {
    let e = solved(&tilted_d_post("round(r: 5mm)"));
    let drum = MaterialField::read(&e.sketch, ent(&e, "drum").i(), 1e-10).unwrap();
    let post = MaterialField::read(&e.sketch, ent(&e, "post").i(), 1e-10).unwrap();
    let resolution = Resolution { finest: 0.05, coarsest: 0.8, tolerance: 0.005 };
    // a cell's reach: no interpolated value is farther than a coarsest cell's diagonal off
    let reach = 3f64.sqrt() * resolution.coarsest;
    let (mut off, mut differ) = (0.0f64, 0);
    for p in lattice([-6.0, -6.0, -1.0], [6.0, 6.0, 15.0], 9) {
        let (cached, exact) = read(&drum, p, resolution);
        off = off.max((cached - exact).abs());
        assert!((cached - exact).abs() <= reach, "{p:?}: {cached} against {exact}");
        if exact.abs() > reach { assert_eq!(cached < 0.0, exact < 0.0, "{p:?}"); }
        if cached != exact { differ += 1; }
        let (cached, exact) = read(&post, p, resolution);
        assert_eq!(cached.to_bits(), exact.to_bits(), "{p:?}: the prism is read exactly");
        // a sign alike
        let sign = |s: Source| post.query(p, &mut Query { want: Want::Sign, source: s, ..Query::at(p) }).value;
        assert_eq!(sign(Source::Cached(resolution)).to_bits(), sign(Source::Exact).to_bits());
    }
    // the round is read from the cache, not the search
    assert!(differ > 0 && off > 0.0, "the searched field is read from its cache");
}

/// The seconds this thread has run, where the system says (Linux's `schedstat`).
fn thread_seconds() -> Option<f64> {
    let stat = std::fs::read_to_string("/proc/thread-self/schedstat").ok()?;
    Some(stat.split_whitespace().next()?.parse::<f64>().ok()? * 1e-9)
}

/// **Drawn in time that follows the facets** (#961's sibling refusal): meshing the field of a
/// post whose round the lowering refuses took 18–25 s natively before the round was read from a
/// distance field and about 2.5 s after (the kernel's refusal, the issue's: about a minute and
/// 6 s); under the test profile it is held to thirty seconds of its thread's time, about three
/// alone.  What was said ahead is what the evaluation carries, and the surface it gives holds
/// most of the D's prism.
#[test]
fn a_solid_whose_round_is_refused_is_drawn_in_bounded_time() {
    let e = solved(&square_d_post());
    let i = ent(&e, "drum").i();
    let ahead = e.sketch.field_meshed_ahead(i).expect("the field is to be meshed");
    let (clock, cpu) = (std::time::Instant::now(), thread_seconds());
    let solid = e.sketch.evaluated_solid(i, ApproximationPolicy::Mesh).expect("drawn from its field");
    // the thread's own time where the system keeps it, so a loaded machine is not read as a slow
    // mesh (the whole suite at once made the wall clock ten times this test's alone)
    let took = match (cpu, thread_seconds()) {
        (Some(a), Some(b)) => b - a,
        _ => clock.elapsed().as_secs_f64(),
    };
    assert!(took < 30.0, "{took} s");
    assert_eq!(solid.unbuilt(), Some(&ahead));
    // drawn, a second look meshes nothing more, and says the same
    assert_eq!(e.sketch.field_meshed_ahead(i).as_ref(), Some(&ahead));
    // the D's area, 10 high, less what the round takes: under the prism, and most of it
    let d = 25.0 * (std::f64::consts::PI - (0.6f64).acos()) + 3.0 * 4.0;
    let v = solid.volume();
    assert!(v < 10.0 * d && v > 0.5 * 10.0 * d, "{v} against {}", 10.0 * d);
}
