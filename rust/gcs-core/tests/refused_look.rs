//! **A look at a solid whose round the kernel refuses** (#961): the solid is drawn from its
//! material field (`EvaluatedSolid::unbuilt`, solids.md), and that field reads the round point by
//! point, each value a nearest-point search over the faces it rounds.  A mesher reads such a field
//! from an adaptive distance field at the resolution its pass needs (`SpatialField::cached`), as
//! it reads a sweep, so the searches follow the facets drawn; a field read in closed form is read
//! exactly, as ever.  And the refusal is known before the refinement starts, so it is said first
//! (`Sketch::field_meshed_ahead`).
use gcs_core::solid::{ApproximationPolicy, DrawnFrom, MaterialField, Query, Resolution, Source, Want};

use crate::common::{ent, text::solved};

/// A post 14 high on a disc of radius 5 about the origin with a chord each side at `x = ±3` (its
/// rim two arcs), cut by a lid tilted 20° about the y axis through (0, 0, 10), its rim along the
/// arcs `round`ed: to 5, the post's own radius, the ball stands still on the axis and each arc's
/// meeting is a stretch of the top's loop ending where the chords' faces cut across it, and past
/// one stretch the meeting comes back within both chords at the other, which the kernel refuses
/// (`fillet::dome_cut`, "halfway there is no section of a ball standing still"; with one chord, the
/// whole dome the chord cuts is built since #994: `standing_ends`); to 2, it rolls, and its end
/// against a chord is refused instead.
fn tilted_d_post(round: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids (cut, union, common)\nuse std.rounds (round)\n\
        in std.top {{\n  o := point\n  fix((0, 0)) o\n  a := point\n  fix((3, -4)) a\n  b := point\n\
        \x20 fix((3, 4)) b\n  c := point\n  fix((-3, 4)) c\n  d := point\n  fix((-3, -4)) d\n  \
        side := arc(center: o, start: b, end: c) hint(r: 5)\n  back := line(c, d)\n  \
        other := arc(center: o, start: d, end: a) hint(r: 5)\n  flat := line(a, b)\n}}\n\
        post := solid(std.solids.Prism(face(side, back, other, flat), from: 0mm, to: 14mm))\n\
        across := axis hint(dir: (0, 1, 0))\nacross parallel std.y\n\
        u := axis hint(dir: (0.94, 0, -0.34))\nu perpendicular across\nu angle(20deg, about: std.y) std.x\n\
        slant := plane(u: u, v: across)\nfix(origin == (0mm, 0mm, 10mm)) slant\n\
        in slant {{\n  p := point\n  q := point\n  r := point\n  s := point\n\
        \x20 fix((-20, -20)) p\n  fix((20, -20)) q\n  fix((20, 20)) r\n  fix((-20, 20)) s\n\
        \x20 (pq := line(p, q)) -> (qr := line(q, r)) -> (rs := line(r, s)) -> (sp := line(s, p)) -> close\n}}\n\
        lid := solid(std.solids.Prism(face(pq, qr, rs, sp), from: 0mm, to: 30mm))\n\
        drum := solid(post)\nlid cut drum\ndrum.lid {round} drum.post.side\n")
}

/// `bracket.sv` (an L bracket, its foot filled into its plate) with a short slot across the
/// crease, along it, in place of its slots: each run ends on one of the slot's round ends, which
/// its section reaches past — into the slot between them, where neither run's end is the other's —
/// refused by the lowering itself, before the kernel is asked ("a run its curved end does not cut
/// off", `round_lowering::a_refused_rounds_piece_is_said_at_its_round`).  `round` false leaves the
/// fill out.  (`tilted_d_post` with one chord, square-topped, its arc's rim rounded to the post's
/// radius, was refused here as "a stretch of a circle its end does not cut off"; its ring is now
/// carried round to the middle of the flat and built, #962 item 3:
/// `curved_ends::a_ds_rim_rounded_to_its_radius_is_a_dome_its_flat_cuts`.)
fn slotted_bracket(round: bool) -> String {
    let src = include_str!("../../examples/rounds/bracket.sv");
    let slots = src.find("  // a slot along the foot").unwrap();
    let fill = src.find("  body.foot round").unwrap();
    let slot = "  slot := Block(x0: 5.5mm, x1: 6.5mm, y0: -1mm, y1: 1mm, z0: -1mm, z1: 7mm)\n  \
        slot.body cut body\n  near := Post(x: 6mm, y: -1mm, d: 1mm, z0: -1mm, z1: 7mm)\n  \
        near.body cut body\n  far := Post(x: 6mm, y: 1mm, d: 1mm, z0: -1mm, z1: 7mm)\n  \
        far.body cut body\n";
    let line = "  body.foot round(r: r) body.plate\n";
    let rest = if round { &src[fill..] } else { &src[fill + line.len()..] };
    format!("{}{slot}{rest}", &src[..slots])
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
    let kernels = "their meeting comes back within the faces it ends on past it";
    let lowerings = "a run its curved end does not cut off";
    for (src, solid, why) in [(tilted_d_post("round(r: 5mm)"), "drum", kernels),
                              (slotted_bracket(true), "bracket.body", lowerings)] {
        let e = solved(&src);
        let i = ent(&e, solid).i();
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
/// post whose round the lowering refused took 18–25 s natively before the round was read from a
/// distance field and about 2.5 s after (the kernel's refusal, the issue's: about a minute and
/// 6 s); the slotted bracket's, about 3 s alone under the test profile, is held to thirty seconds
/// of its thread's time.  What was said ahead is what the evaluation carries, and the surface it
/// gives holds the bracket unrounded and no more than the fill all along its edge adds.
#[test]
fn a_solid_whose_round_is_refused_is_drawn_in_bounded_time() {
    let e = solved(&slotted_bracket(true));
    let i = ent(&e, "bracket.body").i();
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
    // the bracket built exactly without its fill, and that fill's section, (1 − π/4) r² with r 3,
    // along all 40 of its edge: the fill adds to the one, and no more than the other (the slot
    // takes some of it)
    let bare = solved(&slotted_bracket(false));
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let unrounded = gcs_core::brep::export::exact(&bare.sketch, ent(&bare, "bracket.body").i(), None,
        None, &say).expect("the bracket unrounded is built").solid;
    let unrounded = gcs_core::brep::props::volume(&unrounded);
    let fill = (1.0 - std::f64::consts::FRAC_PI_4) * 9.0 * 40.0;
    let v = solid.volume();
    assert!(v > unrounded && v < unrounded + fill, "{v} against {unrounded} and {fill} more");
}

/// **The nearest face and edge read by their boxes are the ones every face and edge read gives**
/// (#996): a field over a body's faces asks, at every step of every query, which of a group of
/// faces has the nearest patch (`BodyFaces::nearest_patch`) and which edge is nearest
/// (`nearest_edge`), and each reads only what its box lets come within the best so far.  At points
/// all round bodies with rounds — a tee's crotch rolled along its saddle, its edges traced; a slab
/// drilled twice and rounded all over; the bracket whose fill the lowering refuses, its stratum —
/// each answer is the scan's, to the bit and the first among equals: the face, its distance, the
/// edge, its point.  Each is read from no warm start (`util::warm::fresh`): a search's start is
/// the last foot its thread found on that surface, which moves the last bits of what it settles on,
/// so two reads of one patch are alike only from the same start.
#[test]
fn the_nearest_face_and_edge_read_by_their_boxes_are_the_scans() {
    use gcs_core::brep::util::warm::fresh;
    use gcs_core::solid::faces::BodyFaces;
    let bodies: Vec<(gcs_core::program::Elaborated, &str, bool)> = vec![
        (solved(include_str!("../../examples/rounds/tee.sv")), "tee", true),
        (solved(include_str!("../../examples/rounds/bores.sv")), "slab", true),
        (solved(&slotted_bracket(true)), "bracket.body", false),
    ];
    let mut rng = gcs_core::rng::Rng::new(996);
    for (e, name, whole) in &bodies {
        let i = ent(e, name).i();
        let body = if *whole { BodyFaces::whole(&e.sketch, i) } else { gcs_core::solid::faces::cached(&e.sketch, i) }
            .unwrap_or_else(|why| panic!("{name}: {why}"));
        let (b, _, _) = body.brep();
        let (lo, hi) = b.bounds();
        let n = b.faces.len();
        let all: Vec<usize> = (0..n).collect();
        let edges: Vec<u32> = (0..b.edges.len() as u32).collect();
        // the groups a field asks of: every face, every third, and each face with its neighbours
        let mut groups: Vec<Vec<usize>> = vec![all.clone(), all.iter().copied().step_by(3).collect()];
        groups.extend((0..n).map(|fi| {
            let mut g: Vec<usize> = body.face_edges(fi).iter().flat_map(|&e| body.faces_of(e).to_vec()).collect();
            g.sort_unstable();
            g.dedup();
            g
        }));
        let patch = |fi: usize, p: [f64; 3]| fresh(|| body.patch_distance(fi, p));
        let edge = |es: &[u32], p: [f64; 3]| fresh(|| body.nearest_edge(es, p));
        for k in 0..300 {
            // within the body's box grown by a fifth each way (millimetres), read in model units
            let q: [f64; 3] = std::array::from_fn(|a| {
                let (l, h) = (lo[a], hi[a]);
                l - 0.2 * (h - l) + 1.4 * (h - l) * rng.uniform(0.0, 1.0)
            });
            let p = body.back(q);
            // a face's patch distance: its foot's where that is on it, else its edges' each read alone
            for fi in 0..n {
                let scan = match fresh(|| body.off_face(fi, p)) {
                    None => patch(fi, p),
                    Some(_) => body.face_edges(fi).iter().map(|&e| edge(&[e], p).unwrap().2)
                        .fold(f64::INFINITY, f64::min),
                };
                assert_eq!(patch(fi, p).to_bits(), scan.to_bits(), "{name}: face {fi} at {p:?}");
            }
            for g in &groups {
                let scan = g.iter().map(|&fi| (patch(fi, p), fi)).min_by(|a, b| a.0.total_cmp(&b.0));
                let read = fresh(|| body.nearest_patch(g, p));
                assert_eq!(read.map(|(d, f)| (d.to_bits(), f)), scan.map(|(d, f)| (d.to_bits(), f)),
                           "{name}: point {k} {p:?} among {g:?}");
            }
            for es in [&edges[..], body.face_edges(k % n)] {
                let scan = es.iter().map(|&e| edge(&[e], p).unwrap()).min_by(|a, b| a.2.total_cmp(&b.2));
                let read = edge(es, p);
                assert_eq!(read.map(|(e, x, d)| (e, x.map(f64::to_bits), d.to_bits())),
                           scan.map(|(e, x, d)| (e, x.map(f64::to_bits), d.to_bits())), "{name}: point {k} {p:?}");
            }
        }
    }
}
