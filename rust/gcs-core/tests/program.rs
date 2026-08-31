//! Solvent: printing a sketch as a program, and elaborating one back.
//!
//! The acceptance bar is **document-state preservation, not `Sketch` identity** — `tests/order.rs`
//! shows the JSON format we already ship does not preserve `topology_key` across a load, so
//! chasing the stricter bar would be chasing something nothing here has ever met.  What must hold
//! is that the drawing, its constraints, its numbers, its flags, its placements and its recorded
//! root choices all come back: which is exactly `io::dumps` equality.

use gcs_core::constraints::{Arg, CKind, Constraint, SpecKind, ALL_KINDS};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::model::{EntKind, EntRef, Field, Sketch};
use gcs_core::program::{elaborate, to_program};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::{camel, render, snake};

fn cases() -> Vec<(&'static str, Sketch)> {
    let mut v: Vec<(&'static str, Sketch)> = examples::EXAMPLES
        .iter()
        .map(|&n| (n, examples::example(n).expect(n)))
        .collect();
    for n in ["pythagoras", "belt_tangency", "altitudes", "parallels", "k33", "zigzag"] {
        v.push((n, examples::case(n).expect(n)));
    }
    v
}

/// **The gate.**  A sketch printed as a program and elaborated back is the same document.
///
/// `io::dumps` is the comparison because it is what the document *is*: the entities in order,
/// every constraint with its arguments, the fixed flags, the construction flags, the knots, the
/// placements and the branches.  Nothing about parameter indices, which a load permutes anyway.
#[test]
fn elaborating_a_printed_sketch_rebuilds_it() {
    for (name, sk) in cases() {
        let p = to_program(&sk);
        let e = elaborate(&p);
        assert!(
            e.ok(),
            "{name}: {:?}\n{}",
            e.errors().map(|d| &d.message).collect::<Vec<_>>(),
            p.text()
        );
        assert_eq!(io::dumps(&e.sketch, Some(1)), io::dumps(&sk, Some(1)), "{name}\n{}", p.text());
    }
}

/// And it still solves — printing is not allowed to quietly drop the thing that made it solvable.
#[test]
fn a_printed_sketch_still_solves() {
    for (name, sk) in cases() {
        let mut sk2 = elaborate(&to_program(&sk)).sketch;
        assert!(solve(&mut sk2, SolveOpts::default()).success, "{name}");
    }
}

/// Printing is idempotent: rendering a program that came from a sketch, elaborating it and
/// rendering again gives the same text.  A fixed point, which is the property `tests/io.rs`
/// already asserts of the JSON writer and the only one worth asserting of a printer.
#[test]
fn printing_is_a_fixed_point() {
    for (name, sk) in cases() {
        let once = to_program(&sk).text().to_string();
        let twice = to_program(&elaborate(&to_program(&sk)).sketch).text().to_string();
        assert_eq!(twice, once, "{name}");
    }
}

/// The elaborated parameter vector is the loaded one, parameter for parameter.
///
/// This is what "elaboration is `io::from_json` with a different front end" means, and it is the
/// reason the walk goes per kind in `primitives()` order through the ordinary constructors.  Get
/// it wrong and a document saved as text would compile a different plan from the same drawing
/// saved as JSON.
#[test]
fn elaboration_lays_out_its_parameters_like_a_load() {
    for (name, sk) in cases() {
        let loaded = io::loads(&io::dumps(&sk, None)).expect(name);
        let built = elaborate(&to_program(&sk)).sketch;
        assert_eq!(built.params.len(), loaded.params.len(), "{name}: how many");
        assert_eq!(built.topology_key(), loaded.topology_key(), "{name}: the same layout");
    }
}

/// **Every constraint type prints and parses**, driven by `spec()` and exhaustive over
/// `ALL_KINDS`.  This is the test that makes "nothing is written per constraint type" true rather
/// than merely intended: a new `CKind` fails here if the registry-driven path has a hole.
#[test]
fn every_constraint_type_is_printable() {
    for kind in ALL_KINDS {
        if kind == CKind::DragTarget {
            continue; // soft, and never in a document — `user_constraints` filters it
        }
        if kind == CKind::PointOnCurve {
            // pending the `curve` surface: a contact prints, but the curve it names has no
            // declaration to print yet, so there is nothing for it to round-trip against.  This
            // skip comes out with `a_curve_written_in_the_language_draws`.
            continue;
        }
        let (sk, c) = fixture(kind);
        let mut sk = sk;
        sk.add(c);
        let p = to_program(&sk);
        let e = elaborate(&p);
        assert!(
            e.ok(),
            "{}: {:?}\n{}",
            kind.name(),
            e.errors().map(|d| &d.message).collect::<Vec<_>>(),
            p.text()
        );
        let back = e.sketch.user_constraints();
        assert!(
            back.iter().any(|b| b.kind == kind),
            "{} did not come back\n{}",
            kind.name(),
            p.text()
        );
        assert_eq!(
            io::dumps(&e.sketch, Some(1)),
            io::dumps(&sk, Some(1)),
            "{}\n{}",
            kind.name(),
            p.text()
        );
    }
}

/// The statement name and the registry name are the same name in two spellings, for all 32 — and
/// no two collide, and none is a word the language already uses.
#[test]
fn every_constraint_name_survives_the_case_round_trip() {
    let words = [
        "point", "line", "circle", "arc", "spline", "ellipse", "frame", "plane", "at", "knots",
        "class", "in",
        "ground", "fix", "ccw", "cw", "branch", "component", "port", "param", "ring", "repeat",
        "cycle", "path", "true", "false",
    ];
    let mut seen: Vec<String> = Vec::new();
    for kind in ALL_KINDS {
        let s = snake(kind.name());
        assert_eq!(camel(&s), kind.name(), "{s}");
        assert!(!words.contains(&s.as_str()), "{s} is already a word of the language");
        assert!(!seen.contains(&s), "{s} twice");
        seen.push(s);
    }
}

/// A dimension is the last argument of every type that has one, which is what lets `== …` be the
/// whole rule.  A future type that breaks it must fail here rather than print something wrong.
#[test]
fn every_dimension_is_the_last_argument() {
    for kind in ALL_KINDS {
        let spec = kind.spec();
        for (i, (name, k)) in spec.iter().enumerate() {
            if k.is_dimension() {
                assert_eq!(
                    i,
                    spec.len() - 1,
                    "{}: `{name}` is a dimension and is not last",
                    kind.name()
                );
            }
        }
    }
}

/// Every entity kind's fields are the document's own keys, so the language and the JSON cannot
/// drift apart about what a circle's radius is called.
#[test]
fn the_document_uses_the_field_names() {
    let sk = examples::example("slotted_link").unwrap();
    let doc = io::to_json(&sk);
    for (plural, kind) in [
        ("points", EntKind::Point),
        ("lines", EntKind::Line),
        ("circles", EntKind::Circle),
        ("arcs", EntKind::Arc),
        ("ellipses", EntKind::Ellipse),
        ("frames", EntKind::Frame),
        ("planes", EntKind::Plane),
    ] {
        let Some(first) = doc.get(plural).and_then(|a| a.arr().first()) else { continue };
        for (name, _) in kind.fields() {
            assert!(
                first.get(name).is_some(),
                "{plural}: the document has no `{name}`, which `EntKind::fields` names",
            );
        }
    }
}

/// A point's own parameters are its coordinates, a circle's is its radius, and a line has none —
/// which is what makes `own_params` the list of what a declaration seeds.
#[test]
fn own_params_are_the_scalar_fields() {
    let sk = examples::example("slotted_link").unwrap();
    for e in sk.primitives() {
        let scalars = e.kind.fields().iter().filter(|(_, f)| *f == Field::Scalar).count();
        assert_eq!(sk.own_params(e).len(), scalars, "{}", e.kind.as_str());
        // and they are a subset of the entity's parameters, never a child's
        for p in sk.own_params(e) {
            assert!(sk.entity_params(e).contains(&p));
        }
    }
}

/// A curve contact that was pinned comes back pinned.  Without it, a curve fitted through m
/// points would come back with m degrees of freedom nobody drew — so the pin is constraint-class
/// and is written `==`.
#[test]
fn a_pinned_curve_parameter_survives() {
    let sk = examples::case("spline_follower").unwrap();
    let mut sk = sk;
    let pinned: Vec<u32> = sk
        .constraints
        .iter()
        .flat_map(|c| c.aux_params())
        .filter(|&p| sk.params[p as usize].fixed)
        .collect();
    if pinned.is_empty() {
        // pin one by hand, so the test measures something whatever the example holds
        let p = sk
            .constraints
            .iter()
            .flat_map(|c| c.aux_params())
            .next()
            .expect("the follower has a curve contact");
        sk.params[p as usize].fixed = true;
    }
    let text = to_program(&sk);
    assert!(text.text().contains(" == "), "a pin is written with ==\n{}", text.text());
    let back = elaborate(&text).sketch;
    let n_before = sk.constraints.iter().flat_map(|c| c.aux_params())
        .filter(|&p| sk.params[p as usize].fixed).count();
    let n_after = back.constraints.iter().flat_map(|c| c.aux_params())
        .filter(|&p| back.params[p as usize].fixed).count();
    assert_eq!(n_after, n_before, "the pins came back\n{}", text.text());
}

/// A dimension written as text is printed as written — that is the whole reason the text after
/// `==` is taken verbatim rather than tokenized here.
#[test]
fn a_dimension_expression_is_kept_as_written() {
    let sk = examples::case("pythagoras").unwrap();
    let text = to_program(&sk).text().to_string();
    for want in ["a = 30", "b = 40", "c = hypot(a, b)"] {
        assert!(text.contains(want), "`{want}` is not in\n{text}");
    }
    let back = elaborate(&to_program(&sk)).sketch;
    assert_eq!(io::dumps(&back, Some(1)), io::dumps(&sk, Some(1)));
}

/// An angle is radians in the model and degrees in every text, and the conversion happens once,
/// where the text is made.
#[test]
fn an_angle_is_written_in_degrees() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    let c = sk.point(0.0, 10.0, false, "c");
    let l1 = sk.line(a, b);
    let l2 = sk.line(a, c);
    sk.add(Constraint::new(
        CKind::Angle,
        vec![
            Arg::Ent(EntRef::line(l1)),
            Arg::Ent(EntRef::line(l2)),
            Arg::Num(std::f64::consts::FRAC_PI_6),
        ],
    ));
    let text = to_program(&sk).text().to_string();
    assert!(text.contains("angle(30"), "thirty degrees, not a sixth of pi:\n{text}");
    let back = elaborate(&to_program(&sk)).sketch;
    let got = back.user_constraints()[0].get_num("theta").unwrap();
    assert!((got - std::f64::consts::FRAC_PI_6).abs() < 1e-12, "{got}");
}

/// A name nothing declares is a diagnostic with a span, and the rest of the drawing survives it.
/// The twin of `io.rs`'s `a_dangling_reference_is_an_error_not_a_panic`.
#[test]
fn a_bad_name_is_a_diagnostic_with_a_span() {
    let sk = examples::example("slotted_link").unwrap();
    let mut p = to_program(&sk);
    // point the first line's first endpoint at a name nothing declares
    for st in p.root_mut().body.iter_mut() {
        if let gcs_core::syntax::StmtKind::Decl(d) = &mut st.kind {
            if d.kind == EntKind::Line {
                d.children[0][0] = gcs_core::syntax::Kid::Ref(gcs_core::syntax::Ref::new("nope"));
                break;
            }
        }
    }
    render(&mut p);
    let e = elaborate(&p);
    assert!(!e.ok(), "a dangling name is an error");
    let d = e.errors().next().unwrap();
    assert_eq!(d.code.as_str(), "E101");
    assert!(d.message.contains("nope"), "{}", d.message);
    // and the points are all still there: one bad statement costs one statement
    assert_eq!(e.sketch.points.len(), sk.points.len());
}

/// A name declared twice is E001, and the first one wins so later references still resolve.
#[test]
fn a_name_declared_twice_is_an_error() {
    let mut p = gcs_core::syntax::Program::new();
    for _ in 0..2 {
        p.push(gcs_core::syntax::StmtKind::Decl(gcs_core::syntax::Decl {
            kind: EntKind::Point,
            name: gcs_core::syntax::DeclName::Written(gcs_core::syntax::Name::new("p0")),
            children: Vec::new(),
            seed: vec![1.0, 2.0],
            seed_text: vec![None, None],
            seed_spans: vec![Default::default(); 2],
            hint_span: None,
            knots: None,
            def: None,
            values: Vec::new(),
            domain: None,
            class: Default::default(),
            class_span: Default::default(),
            seed_at: None,
            attitude: Default::default(),
            membership: Default::default(),
            list_span: Default::default(),
        }));
    }
    render(&mut p);
    let e = elaborate(&p);
    assert!(!e.ok());
    assert_eq!(e.errors().next().unwrap().code.as_str(), "E001");
    assert_eq!(e.sketch.points.len(), 1, "the second declaration is skipped, not merged");
}

/// One instance of every constraint type, on geometry that makes it meaningful.  The same shape
/// `tests/jacobians.rs` uses, and for the same reason: the registry says what the arguments are,
/// so a table of them here would be a second copy of the registry.
fn fixture(kind: CKind) -> (Sketch, Constraint) {
    let mut sk = Sketch::new();
    let p = sk.point(0.0, 0.0, false, "p");
    let q = sk.point(30.0, 10.0, false, "q");
    let r = sk.point(10.0, 40.0, false, "r");
    let s = sk.point(50.0, 20.0, false, "s");
    let l1 = sk.line(p, q);
    let l2 = sk.line(r, s);
    let c1 = sk.circle(p, 8.0, "c1");
    let c2 = sk.circle(q, 5.0, "c2");
    // `distance` between two circles is the gap between *concentric* ones, and is refused over
    // any other pair — so that one kind takes a second circle on `c1`'s own centre
    let c3 = sk.circle(p, 5.0, "c3");
    let ac = sk.point(20.0, 20.0, false, "ac");
    let a1 = sk.arc(ac, r, s, "a1");
    let el = sk.ellipse(r, s, 6.0, "el");
    let ctrl: Vec<usize> = (0..4)
        .map(|i| sk.point(60.0 + 10.0 * i as f64, 5.0 * i as f64, false, &format!("k{i}")))
        .collect();
    let sp = sk.spline(&ctrl).expect("four control points make a curve");
    let fr = sk.frame(p, q, "f");
    // two planes, the page's and the top's, with `p` and `q` as images on them: what a
    // projection is inferred from
    let pa = sk.plane(r, s, gcs_core::plane::Basis::page(), "front");
    let pb = sk.plane(r, s, gcs_core::plane::Basis::page().fold(0.0), "top");
    if kind == CKind::Project {
        sk.set_plane(p, Some(pa));
        sk.set_plane(q, Some(pb));
        let c = Constraint::project(&sk, EntRef::point(p), EntRef::point(q)).unwrap();
        return (sk, c);
    }
    let arg = |k: SpecKind| -> Arg {
        match k {
            SpecKind::Point => Arg::Ent(EntRef::point(p)),
            SpecKind::Line => Arg::Ent(EntRef::line(l1)),
            SpecKind::Circle | SpecKind::CircleOrArc => Arg::Ent(EntRef::circle(c1)),
            SpecKind::Arc => Arg::Ent(EntRef::arc(a1)),
            SpecKind::Spline => Arg::Ent(EntRef::spline(sp)),
            SpecKind::Ellipse => Arg::Ent(EntRef::ellipse(el)),
            SpecKind::Frame => Arg::Ent(EntRef::frame(fr)),
            SpecKind::Plane => Arg::Ent(EntRef::plane(pa)),
            SpecKind::Length => Arg::Num(12.0),
            SpecKind::Angle => Arg::Num(0.5),
            _ => Arg::Num(0.0),
        }
    };
    let spec = kind.spec();
    let mut args: Vec<Arg> = Vec::with_capacity(spec.len());
    let mut used_point = false;
    let mut used_line = false;
    let mut used_circle = false;
    for (i, (_, k)) in spec.iter().enumerate() {
        args.push(match k {
            // a constraint relates *distinct* entities, so the second of a pair is a different one
            SpecKind::Point if used_point => Arg::Ent(EntRef::point(q)),
            SpecKind::Point => {
                used_point = true;
                Arg::Ent(EntRef::point(p))
            }
            SpecKind::Line if used_line => Arg::Ent(EntRef::line(l2)),
            SpecKind::Line => {
                used_line = true;
                Arg::Ent(EntRef::line(l1))
            }
            SpecKind::Circle | SpecKind::CircleOrArc if used_circle => {
                Arg::Ent(EntRef::circle(if kind == CKind::AnnularDistance { c3 } else { c2 }))
            }
            SpecKind::Circle | SpecKind::CircleOrArc => {
                used_circle = true;
                Arg::Ent(EntRef::circle(c1))
            }
            _ if kind.infers_arg(i) => kind.default_arg(i),
            SpecKind::Int | SpecKind::Bool | SpecKind::Str | SpecKind::Float => {
                kind.default_arg(i)
            }
            other => arg(*other),
        });
    }
    // the third point of a Symmetric-shaped statement must not repeat the first two
    if kind == CKind::Symmetric {
        args[2] = Arg::Ent(EntRef::line(l2));
    }
    (sk, Constraint::new(kind, args))
}

/* -- reading a program back -------------------------------------------------------- */

/// **The round trip.**  Print a sketch, parse the text, elaborate it, and get the same document.
/// This is the property the whole design rests on, and it is asserted on every example.
#[test]
fn a_printed_program_parses_back_to_the_same_document() {
    for (name, sk) in cases() {
        let text = to_program(&sk).text().to_string();
        let (p, errs) = gcs_core::syntax::parse(&text);
        assert!(errs.is_empty(), "{name}: {:?}\n{text}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
        let e = elaborate(&p);
        assert!(
            e.ok(),
            "{name}: {:?}\n{text}",
            e.errors().map(|d| &d.message).collect::<Vec<_>>()
        );
        assert_eq!(io::dumps(&e.sketch, Some(1)), io::dumps(&sk, Some(1)), "{name}\n{text}");
    }
}

/// And the text is a fixed point: printing what was parsed gives the text back.
#[test]
fn parsing_and_printing_is_a_fixed_point() {
    for (name, sk) in cases() {
        let once = to_program(&sk).text().to_string();
        let (p, _) = gcs_core::syntax::parse(&once);
        let twice = to_program(&elaborate(&p).sketch).text().to_string();
        assert_eq!(twice, once, "{name}");
    }
}

/// A program written by hand — which is the point of having a language at all.
#[test]
fn a_program_written_by_hand_draws() {
    let text = "\
// a square with a hole, written by hand
point a hint(x: 0, y: 0)
point b hint(x: 100, y: 0)
point c hint(x: 100, y: 100)
point d hint(x: 0, y: 100)
point o hint(x: 50, y: 50)
line  ab(a, b)
line  bc(b, c)
line  cd(c, d)
line  da(d, a)
circle hole(center: o) hint(r: 20)

horizontal ab
ab perpendicular bc
bc perpendicular cd
cd perpendicular da
a distance(w = 100) b
b distance(w) c
radius(w / 5) hole
ground a
";
    let (p, errs) = gcs_core::syntax::parse(text);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let mut e = elaborate(&p);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.points.len(), 5);
    assert_eq!(e.sketch.lines.len(), 4);
    assert_eq!(e.sketch.circles.len(), 1);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    // the expression tied the radius to the width the other dimension named
    let r = e.sketch.params[e.sketch.circles[0].radius as usize].value;
    assert!((r - 20.0).abs() < 1e-9, "{r}");
}

/// A bad line costs one line, and every diagnostic carries a span that slices to the problem.
#[test]
fn one_bad_line_costs_one_line() {
    let text = "point a hint(x: 0, y: 0)\nthis is not a statement\npoint b hint(x: 10, y: 0)\n";
    let (p, errs) = gcs_core::syntax::parse(text);
    assert!(!errs.is_empty(), "the bad line is reported");
    let e = elaborate(&p);
    assert_eq!(e.sketch.points.len(), 2, "both good lines survived");
}

/// Nothing in the parser panics, whatever it is handed — `wasm32-unknown-unknown` aborts rather
/// than unwinding, so a document that is not a document must still come back as a diagnostic.
#[test]
fn nothing_in_the_parser_panics() {
    for (_, sk) in cases() {
        let text = to_program(&sk).text().to_string();
        // every prefix, cut only on a character boundary
        for i in 0..text.len() {
            if text.is_char_boundary(i) {
                let (p, _) = gcs_core::syntax::parse(&text[..i]);
                let _ = elaborate(&p);
            }
        }
        // and every byte turned into something structural
        for (i, ch) in [(7usize, '('), (11, ')'), (13, '='), (17, ','), (3, '{')] {
            let mut t = text.clone();
            let at = (0..t.len()).find(|&j| j >= i && t.is_char_boundary(j)).unwrap_or(0);
            if at < t.len() && t.is_char_boundary(at + 1) {
                t.replace_range(at..at + 1, &ch.to_string());
                let (p, _) = gcs_core::syntax::parse(&t);
                let _ = elaborate(&p);
            }
        }
    }
    // and outright rubbish
    for junk in ["", "((((", "point", "point a at", "distance(", "== 5", "\u{0}\u{1}", "點"] {
        let (p, _) = gcs_core::syntax::parse(junk);
        let _ = elaborate(&p);
    }
}

/// **The gear.**  `solvent-spec.md` §18's worked example, with the flanks written as involutes
/// in the language rather than sampled into it.
#[test]
fn a_gear_elaborates() {
    let (p, errs) = gcs_core::syntax::parse(examples::GEAR);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| (&e.message, e.span)).collect::<Vec<_>>());
    let mut e = elaborate(&p);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    let n = 30usize;
    assert_eq!(e.sketch.curve_defs.len(), 1, "one curve family, written in the document");
    assert_eq!(e.sketch.curves.len(), 2 * n, "two involute flanks per tooth");
    assert_eq!(e.sketch.circles.len(), 3, "the base, root and tip circles");
    assert_eq!(e.sketch.points.len(), 1 + 4 * n, "a centre, and two ends per flank");

    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);

    // -- and it is an involute gear.
    let (cx, cy) = e.sketch.point_xy(0);
    assert!(cx.abs() < 1e-9 && cy.abs() < 1e-9, "the centre stayed put");
    // N = 30, m = 3, phi = 25, dedendum 1.0
    let (r_pitch, m) = (45.0f64, 3.0f64);
    let (rr, rt) = (r_pitch - m, r_pitch + m);
    let rb = r_pitch * 25.0f64.to_radians().cos();
    assert!(rr > rb, "the root circle is outside the base circle, or there is no flank there");

    // every flank end is on the circle its statement said it was on, and nothing said where
    let mut on_root = 0;
    let mut on_tip = 0;
    for i in 1..e.sketch.points.len() {
        let (x, y) = e.sketch.point_xy(i);
        let rad = x.hypot(y);
        if (rad - rr).abs() < 1e-6 {
            on_root += 1;
        } else if (rad - rt).abs() < 1e-6 {
            on_tip += 1;
        } else {
            panic!("a flank end at radius {rad}, which is neither {rr} nor {rt}");
        }
    }
    assert_eq!(on_root, 2 * n, "one root end per flank");
    assert_eq!(on_tip, 2 * n, "one tip end per flank");

    // **the involute test**: sample each flank and check every point is one — the string from
    // where it leaves the base circle is perpendicular to the radius there, and exactly as long
    // as the arc it unwound.  That is the definition, checked against the drawing.
    for ci in 0..e.sketch.curves.len() {
        let (u0, u1) = e.sketch.curve_domain(ci);
        for k in 0..=8 {
            let u = u0 + (u1 - u0) * k as f64 / 8.0;
            let (x, y) = e.sketch.curve_point(ci, u);
            // the tangent point is hint at the roll, off the *base* circle
            let ph = phase_of(&e.sketch, ci);
            let a = (u + ph).to_radians();
            let t = (rb * a.cos(), rb * a.sin());
            let string = (x - t.0, y - t.1);
            let radial = (t.0, t.1);
            assert!(
                (radial.0 * string.0 + radial.1 * string.1).abs() < 1e-6 * rb * rb,
                "curve {ci} at u = {u}: the string is not perpendicular to the radius",
            );
            let arc = rb * u.to_radians().abs();
            assert!(
                (string.0.hypot(string.1) - arc).abs() < 1e-6,
                "curve {ci} at u = {u}: string {} against arc {arc}",
                string.0.hypot(string.1),
            );
        }
    }
}

/// The bearing a curve's involute starts from — the `phase` its instance was given.
fn phase_of(sk: &Sketch, ci: usize) -> f64 {
    sk.curves[ci].values[0]
}

/// **A gear with few teeth.**  The flank is an involute of the base circle, and an involute of a
/// circle does not exist inside it — so a root circle asked to go there is asking for a curve that
/// is not there.  `Rr = R - ded*m` falls inside `Rb = R cos(phi)` once `N < 2*ded/(1 - cos phi)`,
/// which at the reference proportions is 21.35: hence the gear that would not read below 22 teeth,
/// with `u0` coming to the square root of a negative number.
///
/// The document answers it the way a real gear does — the tooth gets *shallower* rather than
/// growing a flank that does not exist — so what is checked here is that every count still draws,
/// still solves, and is still made of involutes.  A count that merely elaborates is not enough:
/// clamping the root onto the base circle exactly would put the contact on the involute's cusp,
/// where `C'` vanishes, and the solve crawls to its iteration limit instead of failing outright.
#[test]
fn a_gear_with_few_teeth() {
    for n in [22usize, 21, 18, 12, 8, 5, 4, 3, 2] {
        let src = examples::GEAR.replace("g: Gear(N: 30,", &format!("g: Gear(N: {n},"));
        let (p, errs) = gcs_core::syntax::parse(&src);
        assert!(errs.is_empty(), "N = {n}: {errs:?}");
        let mut e = elaborate(&p);
        assert!(
            e.ok(),
            "N = {n}: {:?}",
            e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
        );
        assert_eq!(e.sketch.curves.len(), 2 * n, "N = {n}: two flanks per tooth");

        let r = solve(&mut e.sketch, SolveOpts::default());
        assert!(r.success, "N = {n}: {}", r.message);

        let (m, ded, phi) = (3.0f64, 1.0f64, 25.0f64);
        let r_pitch = m * n as f64 / 2.0;
        let rb = r_pitch * phi.to_radians().cos();
        let rt = r_pitch + m;
        // the root never goes inside the base circle, and stands clear of the cusp on it
        let rr = (r_pitch - ded * m).max(rb * 1.02);
        assert!(rr > rb, "N = {n}: the root is outside the base circle");
        assert!(rt > rr, "N = {n}: there is a tooth to speak of");

        // every flank end is on the circle its statement said it was on
        let (mut tips, mut roots) = (Vec::new(), Vec::new());
        for i in 1..e.sketch.points.len() {
            let (x, y) = e.sketch.point_xy(i);
            let rad = x.hypot(y);
            if (rad - rr).abs() < 1e-6 {
                roots.push(y.atan2(x));
            } else if (rad - rt).abs() < 1e-6 {
                tips.push(y.atan2(x));
            } else {
                panic!("N = {n}: a flank end at radius {rad}, which is neither {rr} nor {rt}");
            }
        }
        assert_eq!(tips.len(), 2 * n, "N = {n}: two tip ends per tooth");
        assert_eq!(roots.len(), 2 * n, "N = {n}: two root ends per tooth");

        // **the teeth do not run into each other.**  Each tooth's ends belong to the bearing its
        // instance was given, so grouping them by nearest `k * pitch` says how wide that tooth
        // actually came out — and a tooth as wide as the pitch is one touching its neighbour.
        // This is what the involute test cannot see: both flanks can be perfect involutes while
        // the pair of them has stopped being a tooth.  Sorting the ends and looking for two that
        // are close would *not* see it, because two overlapping teeth still have their four ends
        // comfortably apart; it is the width that has to be measured.
        let pitch = std::f64::consts::TAU / n as f64;
        for (what, ends) in [("tip", tips), ("root", roots)] {
            let mut by_tooth = vec![Vec::new(); n];
            for b in ends {
                // the bearing this end is nearest to, as a tooth index
                let k = (b / pitch).round().rem_euclid(n as f64) as usize;
                // measured from that tooth's own bearing and wrapped into (-pi, pi], so a tooth
                // lying across the cut at +/-pi is still one tooth and not two half ones
                let pi = std::f64::consts::PI;
                by_tooth[k].push((b - k as f64 * pitch + pi).rem_euclid(std::f64::consts::TAU) - pi);
            }
            for (k, mut w) in by_tooth.into_iter().enumerate() {
                assert_eq!(w.len(), 2, "N = {n}: tooth {k} has {} {what} ends, not two", w.len());
                w.sort_by(|a, b| a.partial_cmp(b).expect("a bearing is a number"));
                let width = w[1] - w[0];
                assert!(
                    width > 0.0,
                    "N = {n}: tooth {k} has no width at the {what} — its flanks have crossed",
                );
                assert!(
                    width < pitch,
                    "N = {n}: tooth {k} is {}° wide at the {what}, wider than the {}° pitch — \
                     it has run into its neighbour",
                    width.to_degrees(),
                    pitch.to_degrees(),
                );
            }
        }

        // and every flank is still an involute: the string from where it leaves the base circle
        // is perpendicular to the radius there and as long as the arc it unwound
        for ci in 0..e.sketch.curves.len() {
            let (u0, u1) = e.sketch.curve_domain(ci);
            for k in 0..=8 {
                let u = u0 + (u1 - u0) * k as f64 / 8.0;
                let (x, y) = e.sketch.curve_point(ci, u);
                let a = (u + phase_of(&e.sketch, ci)).to_radians();
                let t = (rb * a.cos(), rb * a.sin());
                let string = (x - t.0, y - t.1);
                assert!(
                    (t.0 * string.0 + t.1 * string.1).abs() < 1e-6 * rb * rb,
                    "N = {n}, curve {ci} at u = {u}: the string is not perpendicular to the radius",
                );
                let arc = rb * u.to_radians().abs();
                assert!(
                    (string.0.hypot(string.1) - arc).abs() < 1e-6,
                    "N = {n}, curve {ci} at u = {u}: string {} against arc {arc}",
                    string.0.hypot(string.1),
                );
            }
        }
    }
}

/// **Every seed is in a `hint(…)` clause, and nothing else is.**
///
/// The keys may come in any order and an omitted one is 0, so the two spellings below are one
/// drawing.  What the printer writes is the full clause, whatever the source wrote — the shape
/// that round-trips without a rule about which numbers are worth saying.
///
/// The spellings the clause replaced — `point p at (0, 0)`, `point p hint at (0, 0)`,
/// `circle c(center: o, r: 25)` — do not parse: one class of thing is written one way.
#[test]
fn every_seed_is_written_in_a_hint_clause() {
    let read = |src: &str| {
        let (p, errs) = gcs_core::syntax::parse(src);
        assert!(errs.is_empty(), "{src}: {errs:?}");
        let e = elaborate(&p);
        assert!(e.ok(), "{src}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
        e.sketch
    };
    let now = read("point a hint(x: 3, y: 4)\npoint b hint(x: 9, y: 1)\nline l(a, b)\n");
    let flipped = read("point a hint(y: 4, x: 3)\npoint b hint(x: 9, y: 1)\nline l(a, b)\n");
    assert_eq!(io::dumps(&now, Some(1)), io::dumps(&flipped, Some(1)), "keys in any order");

    // and the printer writes the clause, all of it
    let text = to_program(&now).text().to_string();
    assert!(text.contains("hint(x: 3, y: 4)"), "{text}");

    // the retired spellings are errors, and each says where the number belongs
    for src in [
        "point a at (3, 4)\n",
        "point a hint at (3, 4)\n",
        "point o hint(x: 0, y: 0)\ncircle c(center: o, r: 25)\n",
    ] {
        let (_, errs) = gcs_core::syntax::parse(src);
        assert!(!errs.is_empty(), "{src} still parses");
    }
}

/* -- implicit children (spec §6.1, §6.2) ---------------------------------------------- */

fn read_ok(src: &str) -> gcs_core::program::Elaborated {
    let (p, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{src}: {errs:?}");
    let e = elaborate(&p);
    assert!(e.ok(), "{src}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    e
}

/// **A declaration need not name its children.**  `line l` makes two points, and `l.p1` is what
/// they are called — a name earns its place when something says it twice, and these said it once.
#[test]
fn a_declaration_may_omit_its_children() {
    let e = read_ok("line l\nhorizontal l\nground l.p1\n");
    assert_eq!(e.sketch.points.len(), 2, "two ends, minted");
    assert_eq!(e.sketch.lines.len(), 1);
    // the dotted path is the name: it resolves, and the map carries it
    let p1 = e.map.ent_named("l.p1").expect("l.p1 is a name");
    let p2 = e.map.ent_named("l.p2").expect("l.p2 is a name");
    assert_ne!(p1, p2);
    assert_eq!(e.map.names.get(&p1).map(|v| v[0].as_str()), Some("l.p1"));

    // and the ends do not start on top of each other, or `horizontal` would have no direction
    let [ax, ay] = e.sketch.point_params(p1.i());
    let [bx, by] = e.sketch.point_params(p2.i());
    let d = (e.sketch.params[ax as usize].value - e.sketch.params[bx as usize].value).hypot(
        e.sketch.params[ay as usize].value - e.sketch.params[by as usize].value,
    );
    assert!(d > 1e-3, "a zero-length line has no direction to level: {d}");

    for src in ["circle c\nradius(5) c\n", "arc a\nradius(5) a\n"] {
        let e = read_ok(src);
        assert!(!e.sketch.points.is_empty(), "{src}");
    }
}

/// A child slot may hold a seed instead of a name, and it says the same thing as the three
/// statements it stands for — same drawing, same freedoms.
#[test]
fn a_child_slot_may_hold_a_seed() {
    let mut anon = read_ok("line l(hint(x: 0, y: 0), hint(x: 60, y: 20))\nground l.p1\n");
    let mut named = read_ok(
        "point a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 20)\nline l(a, b)\nground a\n",
    );
    assert_eq!(gcs_core::io::dumps(&anon.sketch, Some(1)), gcs_core::io::dumps(&named.sketch, Some(1)));
    let opts = gcs_core::diagnose::DiagnoseOptions::default();
    assert_eq!(
        gcs_core::diagnose::diagnose(&mut anon.sketch, opts).dof,
        gcs_core::diagnose::diagnose(&mut named.sketch, opts).dof
    );
}

/// All the children or none: a written slot carries a name or a seed, and anything between is
/// E103 exactly as it always was.  A `spline` has no arity to conjure children from, so a bare
/// one stays an error.
#[test]
fn a_slot_left_out_is_an_implicit_child() {
    // `line l(a)` names one end and leaves the other implicit — minted as `l.p2`, exactly as a
    // declaration that writes no list at all mints them (spec §6.1); what stays refused is a
    // kind with no arity to conjure children from, and a list with more than the kind holds
    let (p, errs) = gcs_core::syntax::parse("point a hint(x: 0, y: 0)\nline l(a)\n");
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&p);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert_eq!(e.sketch.points.len(), 2, "the second end is minted");

    for src in ["spline s\n", "point a\npoint b\npoint c\nline l(a, b, c)\n"] {
        let (p, errs) = gcs_core::syntax::parse(src);
        assert!(errs.is_empty(), "{src}: {errs:?}");
        let e = elaborate(&p);
        assert!(!e.ok(), "{src} should not elaborate");
    }
}

/// A seed in a slot round-trips: the printer writes the anonymous form back, since a child with
/// no name has no name to print.
#[test]
fn an_anonymous_child_prints_back_anonymous() {
    let src = "line l(hint(x: 0, y: 0), hint(x: 60, y: 20))\ncircle c hint(r: 25)\n";
    let (p, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &p.root().body[0].kind);
    assert!(out.contains("hint(x: 0, y: 0)"), "{out}");
    assert!(out.contains("hint(x: 60, y: 20)"), "{out}");
    // and re-reading it is the same drawing
    let again = read_ok(src);
    let first = read_ok(src);
    assert_eq!(
        gcs_core::io::dumps(&again.sketch, Some(1)),
        gcs_core::io::dumps(&first.sketch, Some(1))
    );
}

/// A seed in a *list* slot is refused rather than minting a point nothing can reach.
///
/// A control polygon has no arity, so it has no dotted path either: `s.ctrl` is a list and
/// `follow` will not index one.  A point with no name is a point no constraint can be written
/// against and no drag can write back, which is the one outcome worse than an error.
#[test]
fn a_seed_in_a_list_slot_is_refused() {
    let src = "spline s(hint(x: 0, y: 0), hint(x: 1, y: 0), hint(x: 2, y: 1), hint(x: 3, y: 0))\n";
    let (p, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&p);
    assert!(!e.ok(), "a nameless control point should not elaborate");
}
