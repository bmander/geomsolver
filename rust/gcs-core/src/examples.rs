//! Reference sketches used by the tests, the benchmarks and the app's case library.

use crate::constraints::CKind;
use crate::model::Sketch;
use crate::rng::Rng;

/// Rectangle w x h with four equal fillets of radius r — `rect_fillets.sv`, with the document's
/// own `param` lines given the caller's numbers.  `jitter` moves the start off the solution.
pub fn rect_fillets(w: f64, h: f64, r: f64, jitter_amount: f64) -> Sketch {
    let src = with_params(RECT_FILLETS, &[("w", w), ("h", h), ("r", r)]);
    let mut sk = document(&src, "rect_fillets");
    jitter(&mut sk, jitter_amount, 0);
    sk
}

/// Obround slot with two concentric holes — `slotted_link.sv`.
pub fn slotted_link(length: f64, r: f64, hole_r: f64) -> Sketch {
    let src = with_params(SLOTTED_LINK, &[("length", length), ("r", r), ("hole_r", hole_r)]);
    document(&src, "slotted_link")
}

/// A Warren truss of `bays` bays, every member dimensioned — `truss.sv`.  `dims` off drops the
/// lengths, leaving the shape free: the one thing the document does not say twice.
pub fn truss(bays: usize, span: f64, height: f64, dims: bool) -> Sketch {
    let src = with_params(TRUSS, &[("bays", bays as f64), ("span", span), ("height", height)]);
    let mut sk = document(&src, "truss");
    if !dims {
        let ids: Vec<u32> =
            sk.constraints.iter().filter(|c| c.kind == CKind::Distance).map(|c| c.id).collect();
        for id in ids {
            sk.remove(id);
        }
    }
    sk
}

/// A square stated as one line and one corner round a `cycle` — `square.sv`.  The case for a
/// body ending mid-joint (issue #38): the trailing joint threads each copy's side onto the
/// next's, and the wrap closes the loop with no `close`, no names and no written points.
pub fn square() -> Sketch {
    document(SQUARE, "square")
}

/// A regular n-gon from one component — `ngon.sv`.  The parametric sibling of `square.sv`:
/// `Ngon(n: Int, side: Length)` is a corner on a circle and a side round a `cycle` whose body
/// ends mid-joint, the instance line picking the count and the seeds picking the winding.
pub fn ngon() -> Sketch {
    document(NGON, "ngon")
}

/// A closed ring of equal-length links — `polygon_chain.sv`.
pub fn polygon_chain(n: usize, radius: f64) -> Sketch {
    let src = with_params(POLYGON_CHAIN, &[("n", n as f64), ("radius", radius)]);
    document(&src, "polygon_chain")
}

/// A K3,3 bar framework: rigid and triangle-free — `k33.sv`.
pub fn k33() -> Sketch {
    document(K33, "k33")
}

/// The filleted rectangle with a second, contradicting width — `rect_fillets_conflict.sv`.
pub fn rect_fillets_conflict() -> Sketch {
    document(RECT_FILLETS_CONFLICT, "rect_fillets_conflict")
}

/// The filleted rectangle without its width dimension — `rect_fillets_under.sv`.
pub fn rect_fillets_under() -> Sketch {
    document(RECT_FILLETS_UNDER, "rect_fillets_under")
}

/// The truss with one member more than it needs — `truss_redundant.sv`.
pub fn truss_redundant() -> Sketch {
    document(TRUSS_REDUNDANT, "truss_redundant")
}

/// The truss with a member that cannot be there — `truss_conflict.sv`.
pub fn truss_conflict() -> Sketch {
    document(TRUSS_CONFLICT, "truss_conflict")
}

/// The truss with nothing holding it down: 3 DOF of rigid motion — `truss_floating.sv`.
pub fn truss_floating(bays: usize) -> Sketch {
    let src = with_params(TRUSS_FLOATING, &[("bays", bays as f64)]);
    document(&src, "truss_floating")
}

/// Structurally fine, geometrically impossible — `impossible_triangle.sv`.
pub fn impossible_triangle() -> Sketch {
    document(IMPOSSIBLE_TRIANGLE, "impossible_triangle")
}

/// Three altitudes and a point on all three — `altitudes.sv`.
pub fn altitudes() -> Sketch {
    document(ALTITUDES, "altitudes")
}

/// Parallels, perpendiculars and direction classes — `parallels.sv`.
pub fn parallels() -> Sketch {
    document(PARALLELS, "parallels")
}

/// The Pythagorean theorem drawn, with `a` and `b` the document's own named dimensions and
/// `c = hypot(a, b)` a claim the diagnosis judges a theorem — `pythagoras.sv`.
pub fn pythagoras(a: f64, b: f64) -> Sketch {
    let src = with_params(PYTHAGORAS, &[("la", a), ("lb", b)]);
    document(&src, "pythagoras")
}

/// The four sketches the regression suite parametrises over.
pub const EXAMPLES: [&str; 5] =
    ["rect_fillets", "slotted_link", "truss", "polygon_chain", "spline_follower"];

/// A cubic B-spline with a follower held tangent to it and a point riding along it —
/// `spline_follower.sv`.  The control points are written out, so their number is the document's
/// and not an argument: a spline names its children one by one, and no count can stand in.
pub fn spline_follower() -> Sketch {
    document(SPLINE_FOLLOWER, "spline_follower")
}

/// Build a named example.  `None` for an unknown name.
/// `copies` disjoint staircases of `n` points, every link levelled and none given a length —
/// `zigzag.sv`.
pub fn zigzag(n: usize, copies: usize) -> Sketch {
    let src = with_params(ZIGZAG, &[("n", n as f64), ("copies", copies as f64)]);
    document(&src, "zigzag")
}

/// A belt over two pulleys, tangent at each end — `belt_tangency.sv`.
pub fn belt_tangency() -> Sketch {
    document(BELT_TANGENCY, "belt_tangency")
}

/// The Peaucellier–Lipkin cell, over the three lengths its document names — `peaucellier.sv`.
/// The theorem is a theorem at any of them, which is what the arguments are for.
pub fn peaucellier(arm: f64, side: f64, crank: f64) -> Sketch {
    let src = with_params(PEAUCELLIER, &[("arm", arm), ("side", side), ("crank", crank)]);
    document(&src, "peaucellier")
}

/// The same cell, proved against a grounded rail instead of a traced locus —
/// `peaucellier_rail.sv`.  It takes no arguments on purpose: the anchor states where the line
/// is, so the theorem holds at these three lengths and no others.
pub fn peaucellier_rail() -> Sketch {
    document(PEAUCELLIER_RAIL, "peaucellier_rail")
}

/// Build a named example.  `None` for an unknown name.
pub fn example(name: &str) -> Option<Sketch> {
    Some(match name {
        "rect_fillets" => rect_fillets(100.0, 60.0, 10.0, 0.0),
        "slotted_link" => slotted_link(80.0, 15.0, 6.0),
        "truss" => truss(8, 20.0, 15.0, true),
        "square" => square(),
        "ngon" => ngon(),
        "polygon_chain" => polygon_chain(12, 50.0),
        "rect_fillets_conflict" => rect_fillets_conflict(),
        "rect_fillets_under" => rect_fillets_under(),
        "truss_redundant" => truss_redundant(),
        "truss_conflict" => truss_conflict(),
        "truss_floating" => truss_floating(8),
        "impossible_triangle" => impossible_triangle(),
        "altitudes" => altitudes(),
        "parallels" => parallels(),
        "pythagoras" => pythagoras(30.0, 40.0),
        "k33" => k33(),
        "laman" => crate::fixtures::laman(10, 0, true),
        "zigzag" => zigzag(32, 3),
        "spline_follower" => spline_follower(),
        "belt_tangency" => belt_tangency(),
        "peaucellier" => peaucellier(100.0, 60.0, 40.0),
        "peaucellier_rail" => peaucellier_rail(),
        _ => return None,
    })
}

/// The case library shown in the app: (label, key, one-line description).
pub const CASES: [(&str, &str, &str); 25] = [
    ("Rectangle with fillets", "rect_fillets", "fully constrained; tangent arcs, equal radii, two dimensions"),
    ("Square, one line round a cycle", "square", "`cycle 4 { line s -> perpendicular equal }` — the body ends mid-joint, so each side welds to the next copy's and the wrap closes the loop (issue #38); 1 DOF: it swings about its grounded corner"),
    ("Regular n-gon (component)", "ngon", "a parametric `Ngon(n, side)` component: corners on a circle, equal sides, the open-jointed cycle welding them round — pure relations, so the closure equality is implied rather than Over, and the seeds walk once round the circle to pick the convex winding no residual can state (1 DOF: it spins about its hub)"),
    ("Slotted link", "slotted_link", "obround slot with two holes; fully constrained"),
    ("Truss (8 bays)", "truss", "~30-entity Warren truss, every member dimensioned"),
    ("Truss (50 bays)", "truss50", "300 entities — drag a node"),
    ("Truss (200 bays)", "truss200", "1200 entities — solver/plan timing"),
    ("Truss, floating", "truss_floating", "rigid body with nothing fixed: 3 DOF, drag it around"),
    ("Polygon chain (12)", "polygon_chain", "under-constrained equal-length ring; the EqualLength cycle is a redundancy the graph can't see"),
    ("Rect, missing width", "rect_fillets_under", "under-constrained: the right side slides (null-space colouring)"),
    ("Rect, conflicting width", "rect_fillets_conflict", "conflict: two contradicting width dimensions"),
    ("Truss, redundant member", "truss_redundant", "structurally over-constrained but consistent (amber)"),
    ("Truss, impossible member", "truss_conflict", "conflict: a 999-long member; the minimal conflict set is a path plus it"),
    ("Impossible triangle", "impossible_triangle", "structurally fine, geometrically impossible (triangle inequality)"),
    ("K3,3 framework", "k33", "rigid but triangle-free: the decomposition needs a core merge"),
    ("Concurrent altitudes", "altitudes", "theorem-type dependency: the third incidence is implied (Diagnose → witness); 3 DOF to animate"),
    ("Parallels & perpendiculars", "parallels", "direction classes: parallel/perpendicular/vertical (1 DOF left: slide along the base)"),
    ("Pythagoras, graphically", "pythagoras", "four a×b right triangles in a square of side a + b leave a square of side c; `claim distance == c = hypot(a, b)` is judged a theorem — edit a or b and it stays one"),
    ("Curve and follower", "spline_follower", "a cubic B-spline with a face held tangent to it and a point riding on it — drag a control point and the contact slides along the curve, across knots and all"),
    ("Belt over two pulleys", "belt_tangency", "each end on its circle and the line tangent to it — a double root: rank-deficient at every solution, yet nothing can move.  The second-order screen calls it rigid rather than 2 DOF"),
    ("Spur gear (30 teeth)", "gear", "written as a Solvent program: a curve family the document itself defines, one flank as a component, repeated round a cycle — open the Program panel (Edit ▸ Program) to read it"),
    ("Spur gear, traced (12 teeth)", "gear_trace", "the same wheel with the involute *traced* rather than computed: `trace p where { … }` states the taut string — on the circle, perpendicular to the radius, as long as the arc unwound — and the solver finds every point of the flank"),
    ("Levelled zigzags (3×32)", "zigzag", "three separate staircases of free-length H/V segments — a drag costs one staircase, not three"),
    ("Peaucellier straight line", "peaucellier", "the 1864 cell: circling rods whose pen draws an exact straight line — the path is a trace, the straightness is `claim vertical(rail)`, and the diagnosis judges the claim a theorem.  Drag the pen along the rail it cannot leave"),
    ("Peaucellier, proved by rail", "peaucellier_rail", "the same cell with no curve in it: the pen is joined to a grounded point and `claim vertical(rail)` asks whether saying so costs the crank a freedom.  It does not, so the claim is a theorem — but this one has to be told where the line is, where its sibling discovers it"),
];

/// A spur gear, written as a Solvent program rather than built here.
///
/// The one case in the library that is a *document* and not a function: a tooth is a component
/// and the wheel is that component round a cycle, which is what the language is for and what no
/// amount of `Sketch::add` says as clearly.  It is also the regression test for elaboration —
/// components, instances, parameters, `next`, and expressions worked out at elaboration time all
/// have to hold for it to come out round.
pub const GEAR: &str = include_str!("../../examples/gear.sv");

/// The same wheel with the involute *traced* rather than computed — the family's body is
/// `trace p where { … }` (spec §6.5.1), the taut string stated as constraints, and the solver
/// finds every point of the flank.  Twelve teeth, so it also lives in the stub-tooth regime.
pub const GEAR_TRACE: &str = include_str!("../../examples/gear_trace.sv");

pub fn gear() -> Sketch {
    document(GEAR, "gear")
}

pub fn gear_trace() -> Sketch {
    document(GEAR_TRACE, "gear_trace")
}

fn document(src: &str, name: &str) -> Sketch {
    let (p, errs) = crate::syntax::parse(src);
    debug_assert!(errs.is_empty(), "the {name} does not parse: {errs:?}");
    let e = crate::program::elaborate(&p);
    debug_assert!(e.ok(), "the {name} does not elaborate");
    e.sketch
}

/// A document's `param` line, given another number — the one way a case that takes arguments is
/// still *one* implementation.
///
/// A drawing written as a document already names the numbers it is drawn from (`param w = 100`),
/// so a caller asking for another width is asking for that line to read differently.  Rewriting
/// it is a splice on the source, which is what every other edit in this project is; building a
/// second copy of the rectangle in Rust to take the argument is what it is not.  A name the
/// document does not declare is a caller's mistake and says so in debug.
fn with_params(src: &str, kv: &[(&str, f64)]) -> String {
    // the two ways a document names a number: `param w = 100` at elaboration, and `== a = 30`,
    // which names a dimension the drawing states and other dimensions read
    let heads = |name: &str| [format!("param {name} = "), format!("== {name} = ")];
    let mut out = String::with_capacity(src.len());
    let mut hit = vec![false; kv.len()];
    for line in src.lines() {
        let mut written: Option<String> = None;
        for (i, &(name, v)) in kv.iter().enumerate() {
            for head in heads(name) {
                let Some(at) = line.find(&head) else { continue };
                let keep = &line[..at + head.len()];
                written = Some(format!("{keep}{}", crate::json::fmt_g(v, 12)));
                hit[i] = true;
            }
        }
        out.push_str(written.as_deref().unwrap_or(line));
        out.push('\n');
    }
    debug_assert!(
        hit.iter().all(|&h| h),
        "a case was given a number its document does not name",
    );
    out
}

/// Every point moved a little, so a case starts somewhere its constraints do not already hold.
/// A start is not a drawing: the document says what the figure *is*, and this says where the
/// solve begins, which is why it is a function of the sketch rather than a second document.
pub fn jitter(sk: &mut Sketch, amount: f64, seed: u32) {
    if amount == 0.0 {
        return;
    }
    let mut rng = Rng::new(seed);
    for i in 0..sk.points.len() {
        let [px, py] = sk.point_params(i);
        sk.params[px as usize].value += rng.uniform(-amount, amount);
        sk.params[py as usize].value += rng.uniform(-amount, amount);
    }
}

/// The *source* of a case, for the library that has one.
///
/// A case written as a document has a text somebody wrote, and that text — its comments, its
/// components, the reasons in it — is the case.  Lifting the sketch it elaborates to would print
/// a hundred and twenty `point` declarations and none of the explanation, which is a different
/// document about the same drawing.
///
/// Every case has one.  What has no source is not a case: `fixtures::laman` makes a random graph
/// and measures where it happened to put the nodes, so there is no statement behind its numbers
/// for a document to hold — which is why it lives in `fixtures` and not here.
pub fn source(key: &str) -> Option<&'static str> {
    match key.split(':').next().unwrap_or("") {
        "gear" => Some(GEAR),
        "gear_trace" => Some(GEAR_TRACE),
        "impossible_triangle" => Some(IMPOSSIBLE_TRIANGLE),
        "altitudes" => Some(ALTITUDES),
        "parallels" => Some(PARALLELS),
        "belt_tangency" => Some(BELT_TANGENCY),
        "rect_fillets" => Some(RECT_FILLETS),
        "slotted_link" => Some(SLOTTED_LINK),
        "rect_fillets_conflict" => Some(RECT_FILLETS_CONFLICT),
        "rect_fillets_under" => Some(RECT_FILLETS_UNDER),
        "square" => Some(SQUARE),
        "ngon" => Some(NGON),
        "polygon_chain" => Some(POLYGON_CHAIN),
        "truss" => Some(TRUSS),
        "truss_redundant" => Some(TRUSS_REDUNDANT),
        "truss_conflict" => Some(TRUSS_CONFLICT),
        "truss_floating" => Some(TRUSS_FLOATING),
        "zigzag" => Some(ZIGZAG),
        "k33" => Some(K33),
        "pythagoras" => Some(PYTHAGORAS),
        "spline_follower" => Some(SPLINE_FOLLOWER),
        "peaucellier" => Some(PEAUCELLIER),
        "peaucellier_rail" => Some(PEAUCELLIER_RAIL),
        _ => None,
    }
}

pub const IMPOSSIBLE_TRIANGLE: &str = include_str!("../../examples/impossible_triangle.sv");
pub const ALTITUDES: &str = include_str!("../../examples/altitudes.sv");
pub const PARALLELS: &str = include_str!("../../examples/parallels.sv");
pub const BELT_TANGENCY: &str = include_str!("../../examples/belt_tangency.sv");
pub const RECT_FILLETS: &str = include_str!("../../examples/rect_fillets.sv");
pub const SLOTTED_LINK: &str = include_str!("../../examples/slotted_link.sv");
pub const RECT_FILLETS_CONFLICT: &str = include_str!("../../examples/rect_fillets_conflict.sv");
pub const RECT_FILLETS_UNDER: &str = include_str!("../../examples/rect_fillets_under.sv");
pub const SQUARE: &str = include_str!("../../examples/square.sv");
pub const NGON: &str = include_str!("../../examples/ngon.sv");
pub const POLYGON_CHAIN: &str = include_str!("../../examples/polygon_chain.sv");
pub const TRUSS: &str = include_str!("../../examples/truss.sv");
pub const TRUSS_REDUNDANT: &str = include_str!("../../examples/truss_redundant.sv");
pub const TRUSS_CONFLICT: &str = include_str!("../../examples/truss_conflict.sv");
pub const TRUSS_FLOATING: &str = include_str!("../../examples/truss_floating.sv");
pub const ZIGZAG: &str = include_str!("../../examples/zigzag.sv");
pub const K33: &str = include_str!("../../examples/k33.sv");
pub const PYTHAGORAS: &str = include_str!("../../examples/pythagoras.sv");

/// The Peaucellier–Lipkin straight-line cell: a linkage of circling rods whose pen draws an
/// exact straight line, the path stated as a `trace` locus over a scratch copy of the linkage.
/// The straight-line property is stated outright — `claim vertical(rail)` (Solvent §9.7) — and
/// the diagnosis judges the claim a theorem: true, and adding no rank the drawing does not
/// already have.  The case for claims, as `altitudes` is for implied relations.
pub const PEAUCELLIER: &str = include_str!("../../examples/peaucellier.sv");
pub const PEAUCELLIER_RAIL: &str = include_str!("../../examples/peaucellier_rail.sv");
pub const SPLINE_FOLLOWER: &str = include_str!("../../examples/spline_follower.sv");

/// The case library's factory.  Keys are either a plain name or `name:arg[:arg]`, so a front end
/// can ask for `truss:50` or `laman:12:1` without a table of its own.
pub fn case(key: &str) -> Option<Sketch> {
    let mut parts = key.split(':');
    let name = parts.next().unwrap_or("");
    let args: Vec<f64> = parts.filter_map(|p| p.parse().ok()).collect();
    let n = |i: usize, d: usize| args.get(i).map(|&v| v as usize).unwrap_or(d);
    let u = |i: usize, d: u32| args.get(i).map(|&v| v as u32).unwrap_or(d);
    Some(match name {
        "truss50" => truss(50, 20.0, 15.0, true),
        "truss200" => truss(200, 20.0, 15.0, true),
        "laman0" => crate::fixtures::laman(10, 0, true),
        "laman1" => crate::fixtures::laman(12, 1, true),
        "truss" if !args.is_empty() => truss(n(0, 8), 20.0, 15.0, true),
        "truss_floating" if !args.is_empty() => truss_floating(n(0, 8)),
        "polygon_chain" if !args.is_empty() => polygon_chain(n(0, 12), 50.0),
        "gear" => gear(),
        "gear_trace" => gear_trace(),
        "laman" => crate::fixtures::laman(n(0, 10), u(1, 0), true),
        "zigzag" if !args.is_empty() => zigzag(n(0, 32), n(1, 1)),
        "rect_fillets" if args.len() >= 3 => rect_fillets(args[0], args[1], args[2], 0.0),
        "pythagoras" if args.len() >= 2 => pythagoras(args[0], args[1]),
        "peaucellier" if args.len() >= 3 => peaucellier(args[0], args[1], args[2]),
        _ => return example(name),
    })
}
