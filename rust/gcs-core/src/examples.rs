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

/// A plate with a tab on every edge, the tab stated once — `edge_tabs.sv`, the case for
/// `repeat e in CHAIN { … }`: a copy per edge of the outline, in the order the chain walks them.
pub fn edge_tabs() -> Sketch {
    document(EDGE_TABS, "edge_tabs")
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

/// The Pythagorean theorem drawn, with `a` and `b` the document's inputs and
/// `c = hypot(a, b)` a claim the diagnosis judges a theorem — `pythagoras.sv`.
pub fn pythagoras(a: f64, b: f64) -> Sketch {
    let src = with_params(PYTHAGORAS, &[("a", a), ("b", b)]);
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

/// An open belt over two pulleys placed by the length it wraps the big one — `belt_wrap.sv`, the
/// case for `length(L) arc`.
pub fn belt_wrap() -> Sketch {
    document(BELT_WRAP, "belt_wrap")
}

/// The law of reflection: the strike on a mirror placed by two angles stated equal, and the
/// classical proof claimed — `reflection.sv`, the case for `l1 angle(l3, l4) l2`.
pub fn reflection() -> Sketch {
    document(REFLECTION, "reflection")
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

/// Jansen's linkage — the Strandbeest walking leg, eleven rods in the "holy numbers" and the
/// toe's stride traced from the same statements — `jansen.sv`.
pub fn jansen() -> Sketch {
    document(JANSEN, "jansen")
}

/// Build a named example.  `None` for an unknown name.
pub fn example(name: &str) -> Option<Sketch> {
    Some(match name {
        "solid_flange" | "solid_pulley" | "solid_elbow" | "solid_tray" | "solid_loft" | "nurbs"
        | "catenary" | "dido" => {
            document(source(name)?, name)
        }
        "rect_fillets" => rect_fillets(100.0, 60.0, 10.0, 0.0),
        "slotted_link" => slotted_link(80.0, 15.0, 6.0),
        "truss" => truss(8, 20.0, 15.0, true),
        "square" => square(),
        "ngon" => ngon(),
        "edge_tabs" => edge_tabs(),
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
        "belt_wrap" => belt_wrap(),
        "reflection" => reflection(),
        "peaucellier" => peaucellier(100.0, 60.0, 40.0),
        "peaucellier_rail" => peaucellier_rail(),
        "jansen" => jansen(),
        "bracket" => bracket(),
        "engine" => engine(),
        "vtwin" => vtwin(),
        "vtwin_cylinder" => vtwin_cylinder(),
        "vtwin_plate" => vtwin_plate(),
        "vtwin_piston" => vtwin_piston(),
        "vtwin_disc" => vtwin_disc(),
        "vtwin_flywheel" => vtwin_flywheel(),
        "vtwin_throttle" => vtwin_throttle(),
        _ => return None,
    })
}

/// A four-cylinder engine in three views, written as modules — the dimension table, the parts,
/// the valvetrain and one component per view each in a file of its own, linked by `use` (§14.4).
pub fn engine() -> Sketch {
    document(ENGINE, "engine")
}

/// A V-twin oscillating-cylinder air engine in two views, written as modules: the frame, the
/// crank train and one bank each a component designed in one place, the side view placed by
/// projection from the view along the crank axis (§14.4, §6.7).
pub fn vtwin() -> Sketch {
    document(VTWIN, "vtwin")
}

/// The V-twin's cylinder alone: the part `vtwin.cylinder` designs, upright in three views with
/// the dimensions a printer needs — the same component the assembly draws twice, rocked.
pub fn vtwin_cylinder() -> Sketch {
    document(VTWIN_CYLINDER, "vtwin_cylinder")
}

/// The V-twin's frame plate alone: the part sheet.
pub fn vtwin_plate() -> Sketch {
    document(VTWIN_PLATE, "vtwin_plate")
}

/// The V-twin's piston and rod alone: the part sheet.
pub fn vtwin_piston() -> Sketch {
    document(VTWIN_PISTON, "vtwin_piston")
}

/// The V-twin's crank disc alone: the part sheet.
pub fn vtwin_disc() -> Sketch {
    document(VTWIN_DISC, "vtwin_disc")
}

/// The V-twin's flywheel alone: the part sheet.
pub fn vtwin_flywheel() -> Sketch {
    document(VTWIN_FLYWHEEL, "vtwin_flywheel")
}

/// The V-twin's throttle barrel alone: the part sheet.
pub fn vtwin_throttle() -> Sketch {
    document(VTWIN_THROTTLE, "vtwin_throttle")
}

/// An L-bracket in three views and an auxiliary view — descriptive geometry on one sheet.
pub fn bracket() -> Sketch {
    document(BRACKET, "bracket")
}

/// The case library shown in the app: (label, key, one-line description).
pub const CASES: [(&str, &str, &str); 46] = [
    ("Mounting flange · solids", "solid_flange", "One stepped radial section turned about its axis, then a circular pattern of through holes; editable dimensions and three solid views."),
    ("V-belt pulley · solids", "solid_pulley", "A full revolution of a stepped section, with a revolved V-groove cutter and a shaft bore."),
    ("Hollow duct elbow · solids", "solid_elbow", "A hollow square section swept along a constrained circular arc; edit the guide angle, bend radius, or wall thickness."),
    ("Hollow reducer · loft", "solid_loft", "Two hollow square component sections joined along a dimensioned line; change either size, the wall, or the length."),
    ("Pocketed tray · solids", "solid_tray", "Named profile extrusions and nested Boolean bodies: cut a pocket, then add four annular standoffs while preserving the floor."),
    ("Rectangle with fillets", "rect_fillets", "fully constrained; tangent arcs, equal radii, two dimensions"),
    ("Square, one line round a cycle", "square", "`cycle 4 { (s := line) -> perpendicular equal }` — the body ends mid-joint, so each side welds to the next copy's and the wrap closes the loop (issue #38); 1 DOF: it swings about its grounded corner"),
    ("Regular n-gon (component)", "ngon", "a parametric `Ngon(n, side)` component: corners on a circle, equal sides, the open-jointed cycle welding them round — pure relations, so the closure equality is implied rather than Over, and the seeds walk once round the circle to pick the convex winding no residual can state (1 DOF: it spins about its hub)"),
    ("Tabs on every edge · repeat over a chain", "edge_tabs", "a rectangle written as a named chain, and `repeat e in outline { … }` stating one tab: the flattener makes a copy per edge, in the order the chain walks them, `e` naming that copy's edge — change the width, the height or the rise and all four follow; add an edge to the outline and it gets a tab too"),
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
    ("Pythagoras, graphically", "pythagoras", "four a×b right triangles in a square of side a + b leave a square of side c; `claim P1 distance(c := hypot(a, b)) P2` is judged a theorem — edit a or b and it stays one"),
    ("Curve and follower", "spline_follower", "a cubic B-spline with a face held tangent to it and a point riding on it — drag a control point and the contact slides along the curve, across knots and all"),
    ("Rational spline · exact circle", "nurbs", "four control points weighted `[1, w, w, 1]` draw a quarter circle exactly: a bead riding it is proved to stay at the radius, where the same points unweighted bulge to 30.53 and the claim is refuted; the weighted quarter turned is a hemisphere of ⅔πr³ to the last digit, written to STEP as a rational curve"),
    ("Hanging rope · catenary from its principle", "catenary", "a free curve `spline(a, b)` 150 long between two points, and `minimize integral(p.y over p in rope)`: the shape whose height integrated along it is least among every shape that long — the catenary, though nothing says so, and the report calls it a minimum.  Drag `b` and the rope hangs again"),
    ("Dido's problem · most area for its length", "dido", "a strip 130 long on a shore 100 wide, and `maximize` the area the two enclose, written as a line integral of the point and its tangent along the strip: the arc of a circle, reported a maximum"),
    ("Belt over two pulleys", "belt_tangency", "each end on its circle and the line tangent to it — a double root: rank-deficient at every solution, yet nothing can move.  The second-order screen calls it rigid rather than 2 DOF"),
    ("Belt wrap · arc length", "belt_wrap", "an open belt over two pulleys, closed as one tangent chain, with nothing saying how far apart the pulleys are: `length(wrap) big` states the belt in contact with the big pulley — its radius times its sweep — and the centre distance follows.  Edit `wrap` and the second pulley moves"),
    ("Law of reflection · equal angles", "reflection", "a ray from a source strikes a mirror and goes on to a target, the strike placed by `incoming angle(m, outgoing) m` — the angle from the incoming ray to the mirror stated as the angle from the mirror to the outgoing one, with no number.  The classical proof, that the source's image, the strike and the target are collinear, is a `claim` the diagnosis judges a theorem"),
    ("Spur gear (30 teeth)", "gear", "written as a Solvent program: the involute is a component with one computed point, a flank is that point over a roll, a tooth is two flanks, repeated round a cycle — open the Program panel (Edit ▸ Program) to read it"),
    ("Spur gear, traced (12 teeth)", "gear_trace", "the same wheel with the involute *traced* rather than computed: a component states the taut string — on the circle, perpendicular to the radius, as long as the arc unwound — and the flank is its far end as the string unwinds, every point of it the solver's"),
    ("Levelled zigzags (3×32)", "zigzag", "three separate staircases of free-length H/V segments — a drag costs one staircase, not three"),
    ("Peaucellier straight line", "peaucellier", "the 1864 cell: circling rods whose pen draws an exact straight line — the cell is one component, drawn with its crank free and traced from that drawing; the straightness is `claim vertical(rail)`, and the diagnosis judges the claim a theorem.  Drag the pen along the rail it cannot leave"),
    ("Peaucellier, proved by rail", "peaucellier_rail", "the same cell with no curve in it: the pen is joined to a grounded point and `claim vertical(rail)` asks whether saying so costs the crank a freedom.  It does not, so the claim is a theorem — but this one has to be told where the line is, where its sibling discovers it"),
    ("Jansen's linkage", "jansen", "Theo Jansen's walking leg as one component: a crank at one fixed point drives two rigid triangles hinged on another, and nothing but rod lengths is stated — the ccw/cw lines pick each joint's pose.  Drawn with its crank angle unbound (1 DOF: drag `leg.pin` round its circle and the leg steps), and `path` is the same leg asked where its toe goes over a turn"),
    ("L-bracket in three views", "bracket", "descriptive geometry on one sheet: front, top and right views as `plane`s, every corner tied across them by `project`, and an auxiliary view folded at the inclined face's own bearing that shows the face true-size — edit a dimension in the front view and the other three views follow"),
    ("Four-cylinder engine, three views", "engine", "a whole engine as modules, each part designed in one place and drawn in every view it shows in (`engine.block`, `engine.head`, `engine.crankshaft`, `engine.conrod`): the block with its bores, bulkheads and main bearings, the head on its gasket with the valves and camshafts in their bearings, the crankshaft with its flywheel, the rods, the pistons, and the timing belt over its pulleys — the end view owns the heights, the side view the lengths, and the plan is placed by projection from both.  Rigged as a four-stroke: `cycle` in `engine/dims.sv` is cylinder 1's angle in its 720° cycle, the firing order places the others, and the valve timing (intake and exhaust open/close, in crank degrees) sizes every cam lobe and lifts every valve — edit any of them and the section's valves and the side view's lobes follow"),
    ("V-twin air engine, two views", "vtwin", "an oscillating-cylinder (\"wobbler\") V-twin run on shop air, as modules: two printed cylinders rock on studs through one plate, their rods sharing a crank pin, and the rocking is the valve gear — a port in each cylinder's face sweeps across an intake and an exhaust port in the plate.  One bank is one component instanced twice, so the ports come out rotated rather than mirrored; the plenum inside the plate, the inlet boss with its 1/4\" NPT coupling and a rotary barrel throttle are the frame's; the side view's heights are all projected.  One DOF, and it is the crank angle: `crank.theta` is a free variable the arm's callout reads, so drag the pin and both cylinders rock, both pistons move, and the side view follows; `tau` in `vtwin/dims.sv` turns the throttle"),
    ("V-twin cylinder, part sheet", "vtwin_cylinder", "the cylinder component's upright preview; its part sheet and the assembly share components/cylinder.sv"),
    ("V-twin frame plate, part sheet", "vtwin_plate", "the V-twin's frame plate alone, upright in three views with every dimension a printer needs: the ports and the pivots by radius and bearing from the crank axis, every hole and pocket, the plenum and the inlet in section.  The same `vtwin.frame` component the assembly draws"),
    ("V-twin piston and rod, part sheet", "vtwin_piston", "the V-twin's piston and rod alone, upright in three views with every dimension a printer needs: the O-ring groove sized to a #014 ring, the eye and its hole for the clevis pin.  The same `vtwin.piston` component the assembly draws"),
    ("V-twin crank disc, part sheet", "vtwin_disc", "the V-twin's crank disc alone, upright in three views with every dimension a printer needs: the shaft bore, the pin's hole and head pocket, the set screw with its trapped nut.  The same `vtwin.disc` component the assembly draws"),
    ("V-twin flywheel, part sheet", "vtwin_flywheel", "the V-twin's flywheel alone, upright in three views with every dimension a printer needs: a plain disc on the shaft with a set screw in a trapped nut.  The same `vtwin.flywheel` component the assembly draws"),
    ("V-twin throttle barrel, part sheet", "vtwin_throttle", "the V-twin's throttle barrel alone, upright in three views with every dimension a printer needs: the cross-hole, an O-ring groove either side of it and a retaining one behind the boss, the lever.  The same `vtwin.throttle` component the assembly draws"),
];

/// A spur gear, written as a Solvent program rather than built here.
///
/// The one case in the library that is a *document* and not a function: a tooth is a component
/// and the wheel is that component round a cycle, which is what the language is for and what no
/// amount of `Sketch::add` says as clearly.  It is also the regression test for elaboration —
/// components, instances, parameters, `next`, and expressions worked out at elaboration time all
/// have to hold for it to come out round.
pub const GEAR: &str = include_str!("../../examples/gear.sv");

/// The same wheel with the involute *traced* rather than computed — the flank is a point of a
/// component whose body states the taut string as constraints (spec §6.5), and the solver
/// finds every point of the flank.  Twelve teeth, so it also lives in the stub-tooth regime.
pub const GEAR_TRACE: &str = include_str!("../../examples/gear_trace.sv");

pub fn gear() -> Sketch {
    document(GEAR, "gear")
}

pub fn gear_trace() -> Sketch {
    document(GEAR_TRACE, "gear_trace")
}

fn document(src: &str, name: &str) -> Sketch {
    let (p, errs, linked) = crate::library::parse_linked(src);
    debug_assert!(errs.is_empty(), "the {name} does not parse: {errs:?}");
    debug_assert!(linked.is_empty(), "the {name} does not link: {linked:?}");
    let e = crate::program::elaborate(&p);
    debug_assert!(e.ok(), "the {name} does not elaborate");
    e.sketch
}

/// A document's inputs, given other numbers — the one way a case that takes arguments is still
/// *one* implementation.
///
/// A drawing written as a document already declares the numbers it is drawn from as its inputs
/// (`param w := 100`), so a caller asking for another width is asking for that line to read
/// differently.  Rewriting it is a splice on the source, which is what every other edit in this
/// project is; building a second copy of the rectangle in Rust to take the argument is what it
/// is not.  Only a `param` is an input: a name the document does not declare as one is a
/// caller's mistake and says so in debug.
fn with_params(src: &str, kv: &[(&str, f64)]) -> String {
    let (prog, _) = crate::syntax::parse(src);
    let mut values: Vec<(crate::syntax::Span, String)> = kv
        .iter()
        .filter_map(|&(name, v)| {
            prog.root().body.iter().find_map(|st| match &st.kind {
                crate::syntax::StmtKind::Param(p)
                    if p.input.is_some() && p.bound() && p.name.text == name =>
                {
                    Some((p.span, crate::json::fmt_g(v, 12)))
                }
                _ => None,
            })
        })
        .collect();
    debug_assert_eq!(values.len(), kv.len(), "a case was given a number that is no input");
    values.sort_by_key(|(at, _)| std::cmp::Reverse(at.lo));
    let mut out = src.to_string();
    for (at, v) in values {
        out.replace_range(at.lo as usize..at.hi as usize, &v);
    }
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
        "solid_flange" => Some(include_str!("../../examples/solid_flange.sv")),
        "solid_pulley" => Some(include_str!("../../examples/solid_pulley.sv")),
        "solid_elbow" => Some(include_str!("../../examples/solid_elbow.sv")),
        "solid_tray" => Some(include_str!("../../examples/solid_tray.sv")),
        "solid_loft" => Some(include_str!("../../examples/solid_loft.sv")),
        "nurbs" => Some(include_str!("../../examples/nurbs.sv")),
        "catenary" => Some(include_str!("../../examples/catenary.sv")),
        "dido" => Some(include_str!("../../examples/dido.sv")),
        "gear" => Some(GEAR),
        "gear_trace" => Some(GEAR_TRACE),
        "impossible_triangle" => Some(IMPOSSIBLE_TRIANGLE),
        "altitudes" => Some(ALTITUDES),
        "parallels" => Some(PARALLELS),
        "belt_tangency" => Some(BELT_TANGENCY),
        "belt_wrap" => Some(BELT_WRAP),
        "reflection" => Some(REFLECTION),
        "rect_fillets" => Some(RECT_FILLETS),
        "slotted_link" => Some(SLOTTED_LINK),
        "rect_fillets_conflict" => Some(RECT_FILLETS_CONFLICT),
        "rect_fillets_under" => Some(RECT_FILLETS_UNDER),
        "square" => Some(SQUARE),
        "ngon" => Some(NGON),
        "edge_tabs" => Some(EDGE_TABS),
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
        "jansen" => Some(JANSEN),
        "bracket" => Some(BRACKET),
        "engine" => Some(ENGINE),
        "vtwin" => Some(VTWIN),
        "vtwin_cylinder" => Some(VTWIN_CYLINDER),
        "vtwin_plate" => Some(VTWIN_PLATE),
        "vtwin_piston" => Some(VTWIN_PISTON),
        "vtwin_disc" => Some(VTWIN_DISC),
        "vtwin_flywheel" => Some(VTWIN_FLYWHEEL),
        "vtwin_throttle" => Some(VTWIN_THROTTLE),
        _ => None,
    }
}

/// The engine's document — its modules are the library's (`library::MODULES`).
pub const ENGINE: &str = include_str!("../../examples/engine.sv");
/// The V-twin's document — its modules are the library's too.
pub const VTWIN: &str = include_str!("../../examples/vtwin/assembly.sv");
/// The V-twin cylinder's part sheet.
pub const VTWIN_CYLINDER: &str = include_str!("../../examples/vtwin/components/cylinder.sv");
pub const VTWIN_PLATE: &str = include_str!("../../examples/vtwin/components/frame.sv");
pub const VTWIN_PISTON: &str = include_str!("../../examples/vtwin/components/piston.sv");
pub const VTWIN_DISC: &str = include_str!("../../examples/vtwin/components/disc.sv");
pub const VTWIN_FLYWHEEL: &str = include_str!("../../examples/vtwin/components/flywheel.sv");
pub const VTWIN_THROTTLE: &str = include_str!("../../examples/vtwin/components/throttle.sv");

pub const IMPOSSIBLE_TRIANGLE: &str = include_str!("../../examples/impossible_triangle.sv");
pub const ALTITUDES: &str = include_str!("../../examples/altitudes.sv");
pub const PARALLELS: &str = include_str!("../../examples/parallels.sv");
pub const BELT_TANGENCY: &str = include_str!("../../examples/belt_tangency.sv");
pub const BELT_WRAP: &str = include_str!("../../examples/belt_wrap.sv");
pub const REFLECTION: &str = include_str!("../../examples/reflection.sv");
pub const RECT_FILLETS: &str = include_str!("../../examples/rect_fillets.sv");
pub const SLOTTED_LINK: &str = include_str!("../../examples/slotted_link.sv");
pub const RECT_FILLETS_CONFLICT: &str = include_str!("../../examples/rect_fillets_conflict.sv");
pub const RECT_FILLETS_UNDER: &str = include_str!("../../examples/rect_fillets_under.sv");
pub const SQUARE: &str = include_str!("../../examples/square.sv");
pub const NGON: &str = include_str!("../../examples/ngon.sv");
pub const EDGE_TABS: &str = include_str!("../../examples/edge_tabs.sv");
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
pub const JANSEN: &str = include_str!("../../examples/jansen.sv");
pub const BRACKET: &str = include_str!("../../examples/bracket.sv");
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
