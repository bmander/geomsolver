//! Dimension expressions: the language, the document's evaluation order, its errors, and how
//! an expression travels through the document's I/O and rebuilds.
use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::expr::{self, eval, parse, Expr};
use gcs_core::io;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::report;
use gcs_core::solve::{solve, SolveOpts};
use std::collections::BTreeMap;

fn ev(text: &str) -> f64 {
    eval(&parse(text).unwrap().body, &BTreeMap::new()).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * (1.0 + a.abs().max(b.abs()))
}

#[test]
fn arithmetic_reads_as_on_paper() {
    assert_eq!(ev("1 + 2 * 3"), 7.0);
    assert_eq!(ev("(1 + 2) * 3"), 9.0);
    assert_eq!(ev("2 ^ 3 ^ 2"), 512.0);         // right-associative
    assert_eq!(ev("-2 ^ 2"), -4.0);             // the sign binds looser than the power
    assert_eq!(ev("2 ^ -1"), 0.5);
    assert_eq!(ev("2 ** 3"), 8.0);
    assert_eq!(ev("10 / 4"), 2.5);
    assert_eq!(ev("1e3 + .5"), 1000.5);
    assert_eq!(ev("--3"), 3.0);
    assert!(close(ev("pi"), std::f64::consts::PI));
}

#[test]
fn functions_work_in_degrees() {
    assert!(close(ev("sin(30)"), 0.5));
    assert!(close(ev("cos(60)"), 0.5));
    assert!(close(ev("tan(45)"), 1.0));
    assert!(close(ev("asin(0.5)"), 30.0));
    assert!(close(ev("atan2(1, 1)"), 45.0));
    assert!(close(ev("sqrt(16) + abs(-2) + hypot(3, 4)"), 11.0));
    assert!(close(ev("min(3, 1, 2) + max(3, 1, 2)"), 4.0));
    assert!(close(ev("floor(2.7) + ceil(2.2) + round(2.5)"), 8.0));
    assert!(close(ev("ln(exp(2))"), 2.0));
    assert!(close(ev("log(1000)"), 3.0));
}

#[test]
fn syntax_errors_say_where() {
    assert!(parse("1 +").unwrap_err().contains("end"));
    assert!(parse("(1 + 2").unwrap_err().contains("expected `)`"));
    assert!(parse("1 2").unwrap_err().contains("unexpected"));
    assert!(parse("1 $ 2").unwrap_err().contains("`$`"));
    assert!(parse("foo(1)").unwrap_err().contains("unknown function"));
    assert!(parse("sin(1, 2)").unwrap_err().contains("takes 1"));
    assert!(parse("min(1)").unwrap_err().contains("at least 2"));
    assert!(parse("pi = 3").unwrap_err().contains("built in"));
    assert!(parse("sin = 3").unwrap_err().contains("built in"));
    assert!(parse("a = b = 1").is_err());
    let deep = "(".repeat(200) + "1" + &")".repeat(200);
    assert!(parse(&deep).unwrap_err().contains("deeply"));
    assert!(parse(&"1+".repeat(600)).unwrap_err().contains("longer"));
}

#[test]
fn names_and_dependencies() {
    let p = parse("h = w * 2 + sin(a) - pi").unwrap();
    assert_eq!(p.name.as_deref(), Some("h"));
    let deps: Vec<String> = p.body.deps().into_iter().collect();
    assert_eq!(deps, vec!["a".to_string(), "w".to_string()]);   // pi is not a dependency
    assert_eq!(parse(" w=1 ").unwrap().name.as_deref(), Some("w"));
    assert_eq!(expr::name_of("sin(h*10)"), None);
    assert_eq!(expr::literal(" -2.5 "), Some(-2.5));
    assert_eq!(expr::literal("1e3"), Some(1000.0));
    assert_eq!(expr::literal("inf"), None);
    assert_eq!(expr::literal("w"), None);
}

/// Three free segments with one dimension each.
fn three(exprs: [&str; 3]) -> (Sketch, [u32; 3]) {
    let mut sk = Sketch::new();
    let mut ids = [0u32; 3];
    for (i, text) in exprs.iter().enumerate() {
        let a = sk.point(10.0 * i as f64, 0.0, false, "a");
        let b = sk.point(10.0 * i as f64 + 5.0, 0.0, false, "b");
        let c = Constraint::new(
            CKind::Distance,
            vec![
                Arg::Ent(EntRef::point(a)),
                Arg::Ent(EntRef::point(b)),
                Arg::Expr(Expr { text: text.to_string(), value: 0.0 }),
            ],
        );
        ids[i] = sk.add(c);
    }
    (sk, ids)
}

fn value(sk: &Sketch, id: u32) -> f64 {
    sk.constraint(id).unwrap().args[2].num()
}

#[test]
fn evaluated_in_dependency_order_whatever_the_document_order() {
    // the reader comes first in the document, the definition last
    let (mut sk, ids) = three(["sin(h * 10)", "h = w * 2", "w = 1"]);
    let items = expr::evaluate(&mut sk);
    assert!(items.iter().all(|it| it.error.is_none()), "{items:?}");
    let order: Vec<u32> = items.iter().map(|it| it.id).collect();
    assert_eq!(order, vec![ids[2], ids[1], ids[0]]);
    assert_eq!(value(&sk, ids[2]), 1.0);
    assert_eq!(value(&sk, ids[1]), 2.0);
    assert!(close(value(&sk, ids[0]), 20f64.to_radians().sin()));
    assert_eq!(items[1].deps, vec!["w".to_string()]);
    assert_eq!(items[1].name.as_deref(), Some("h"));
    // and the solver sees the numbers
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!(close(sk.line_length_of(ids[1]), 2.0));
}

trait Len {
    fn line_length_of(&self, id: u32) -> f64;
}
impl Len for Sketch {
    fn line_length_of(&self, id: u32) -> f64 {
        let c = self.constraint(id).unwrap();
        let (ax, ay) = self.point_xy(c.args[0].ent().i());
        let (bx, by) = self.point_xy(c.args[1].ent().i());
        (ax - bx).hypot(ay - by)
    }
}

#[test]
fn a_constant_added_as_text_is_evaluated_on_add() {
    // `Sketch::add` evaluates when the new constraint carries an expression
    let (sk, ids) = three(["w = 4", "w / 2", "1"]);
    assert_eq!(value(&sk, ids[0]), 4.0);
    assert_eq!(value(&sk, ids[1]), 2.0);
    assert_eq!(value(&sk, ids[2]), 1.0);
}

#[test]
fn angles_are_written_in_degrees() {
    let mut sk = Sketch::new();
    let l1 = sk.line_xy(0.0, 0.0, 10.0, 0.0, "l1");
    let l2 = sk.line_xy(0.0, 0.0, 10.0, 5.0, "l2");
    let ang = sk.add(Constraint::new(
        CKind::Angle,
        vec![
            Arg::Ent(EntRef::line(l1)),
            Arg::Ent(EntRef::line(l2)),
            Arg::Expr(Expr { text: "a = 30".to_string(), value: 0.0 }),
        ],
    ));
    let p = sk.point(0.0, 0.0, false, "p");
    let q = sk.point(1.0, 0.0, false, "q");
    let d = sk.add(Constraint::new(
        CKind::Distance,
        vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::point(q)),
             Arg::Expr(Expr { text: "a * 2".to_string(), value: 0.0 })],
    ));
    // the argument holds radians; the name is the degrees a person wrote
    assert!(close(value(&sk, ang), 30f64.to_radians()));
    assert_eq!(value(&sk, d), 60.0);
    let items = expr::evaluate(&mut sk);
    assert_eq!(items[0].value, 30.0);
    assert_eq!(io::dimension_text(sk.constraint(ang).unwrap()).unwrap(), "a=30°");
    assert_eq!(io::describe(sk.constraint(d).unwrap()), "Distance(P4, P5, a * 2 = 60)");
}

#[test]
fn errors_name_the_problem_and_keep_the_last_value() {
    let (mut sk, ids) = three(["w = 1", "h = q * 2", "h + 1"]);
    let items = expr::evaluate(&mut sk);
    let by_id = |id: u32| items.iter().find(|it| it.id == id).unwrap();
    assert!(by_id(ids[0]).error.is_none());
    assert_eq!(by_id(ids[1]).error.as_deref(), Some("`q` is not defined"));
    assert_eq!(by_id(ids[2]).error.as_deref(), Some("`h` could not be evaluated"));
    assert_eq!(value(&sk, ids[1]), 0.0);   // what it had

    // define q elsewhere and everything downstream computes
    assert_eq!(expr::set_dimension(&mut sk, ids[0], "d", "q = 5").unwrap(), None);
    assert_eq!(value(&sk, ids[1]), 10.0);
    assert_eq!(value(&sk, ids[2]), 11.0);
    // change q: the change flows
    expr::set_dimension(&mut sk, ids[0], "d", "q = 6").unwrap();
    assert_eq!(value(&sk, ids[2]), 13.0);
    // a plain number drops the definition, and the readers say so but keep their numbers
    assert_eq!(expr::set_dimension(&mut sk, ids[0], "d", "7").unwrap(), None);
    assert_eq!(sk.constraint(ids[0]).unwrap().args[2], Arg::Num(7.0));
    assert_eq!(value(&sk, ids[1]), 12.0);
    let items = expr::evaluate(&mut sk);
    assert_eq!(items.iter().find(|it| it.id == ids[1]).unwrap().error.as_deref(),
               Some("`q` is not defined"));
}

#[test]
fn duplicates_cycles_and_non_numbers() {
    let (mut sk, ids) = three(["w = 1", "w = 2", "w + 1"]);
    let items = expr::evaluate(&mut sk);
    for it in &items {
        assert_eq!(it.error.as_deref(), Some("`w` is defined more than once"), "{it:?}");
    }
    assert_eq!(value(&sk, ids[2]), 0.0);

    let (mut sk, ids) = three(["a = b + 1", "b = a + 1", "a"]);
    let items = expr::evaluate(&mut sk);
    let by_id = |id: u32| items.iter().find(|it| it.id == id).unwrap().error.clone().unwrap();
    assert_eq!(by_id(ids[0]), "circular: a → b → a");
    assert_eq!(by_id(ids[1]), "circular: b → a → b");
    assert_eq!(by_id(ids[2]), "`a` could not be evaluated");

    let (mut sk, ids) = three(["r = sqrt(-1)", "1 / 0", "r"]);
    let items = expr::evaluate(&mut sk);
    let by_id = |id: u32| items.iter().find(|it| it.id == id).unwrap().error.clone().unwrap();
    assert_eq!(by_id(ids[0]), "does not evaluate to a number");
    assert_eq!(by_id(ids[1]), "does not evaluate to a number");
    assert_eq!(by_id(ids[2]), "`r` could not be evaluated");
}

#[test]
fn set_dimension_rejects_what_does_not_parse_and_reports_what_cannot_compute() {
    let (mut sk, ids) = three(["1", "2", "3"]);
    assert!(expr::set_dimension(&mut sk, ids[0], "d", "1 +").is_err());
    assert_eq!(value(&sk, ids[0]), 1.0);
    assert!(expr::set_dimension(&mut sk, ids[0], "p", "1").unwrap_err().contains("not a dimension"));
    assert!(expr::set_dimension(&mut sk, 999, "d", "1").is_err());
    assert_eq!(expr::set_dimension(&mut sk, ids[0], "d", "w * 2").unwrap().as_deref(),
               Some("`w` is not defined"));
    assert_eq!(sk.constraint(ids[0]).unwrap().expr_text("d"), Some("w * 2"));
    assert_eq!(value(&sk, ids[0]), 1.0);   // the old number stands until it computes
    // a bare number for an angle is degrees, like the expression would be
    let mut sk2 = Sketch::new();
    let l1 = sk2.line_xy(0.0, 0.0, 10.0, 0.0, "l1");
    let l2 = sk2.line_xy(0.0, 0.0, 10.0, 5.0, "l2");
    let ang = sk2.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(0.0)],
    ));
    expr::set_dimension(&mut sk2, ang, "theta", "45").unwrap();
    assert_eq!(sk2.constraint(ang).unwrap().args[2], Arg::Num(45f64.to_radians()));
    // set_num on an expression means the number
    let mut c = sk.constraint(ids[0]).unwrap().clone();
    assert!(c.set_num("d", 9.0));
    assert_eq!(c.args[2], Arg::Num(9.0));
}

#[test]
fn documents_carry_text_and_value_and_accept_bare_strings() {
    let (sk, ids) = three(["w = 3", "h = w * 2", "5"]);
    let s = io::dumps(&sk, Some(1));
    assert!(s.contains("\"expr\": \"h = w * 2\""), "{s}");
    let sk2 = io::loads(&s).unwrap();
    assert_eq!(io::dumps(&sk2, Some(1)), s);
    assert_eq!(sk2.constraint(ids[1]).unwrap().args[2],
               Arg::Expr(Expr { text: "h = w * 2".to_string(), value: 6.0 }));
    // a hand-written document: a string is an expression, evaluated on load
    let hand = r#"{"points": [{"x": 0, "y": 0}, {"x": 5, "y": 0}],
                   "constraints": [{"type": "Distance", "args": [["point", 0], ["point", 1], "w = 2 + 2"]}]}"#;
    let sk3 = io::loads(hand).unwrap();
    assert_eq!(sk3.constraints[0].args[2].num(), 4.0);
    // a broken one loads with the value it carried
    let stale = r#"{"points": [{"x": 0, "y": 0}, {"x": 5, "y": 0}],
                    "constraints": [{"type": "Distance", "args": [["point", 0], ["point", 1],
                                     {"expr": "gone * 2", "value": 8}]}]}"#;
    let sk4 = io::loads(stale).unwrap();
    assert_eq!(sk4.constraints[0].args[2].num(), 8.0);
    // and text that is not an expression at all is refused
    let bad = r#"{"points": [{"x": 0, "y": 0}, {"x": 5, "y": 0}],
                  "constraints": [{"type": "Distance", "args": [["point", 0], ["point", 1], true]}]}"#;
    assert!(io::loads(bad).is_err());
}

#[test]
fn the_binding_record_keeps_numbers_and_adds_the_text() {
    let (mut sk, ids) = three(["w = 3", "h = w * 2", "5"]);
    expr::set_dimension(&mut sk, ids[2], "d", "5").unwrap();   // a bare number: no expression
    let j = report::constraint_json(&sk, sk.constraint(ids[1]).unwrap());
    assert_eq!(j.get("args").unwrap().arr()[2].as_f64(), 6.0);
    assert_eq!(j.get("exprs").unwrap().get("d").unwrap().as_str(), "h = w * 2");
    let j = report::constraint_json(&sk, sk.constraint(ids[2]).unwrap());
    assert!(j.get("exprs").is_none());
    // a binding may also add a constraint with a text dimension
    let v = gcs_core::json::parse(
        r#"{"type": "Distance", "args": [["point", 0], ["point", 1], "w + 1"]}"#).unwrap();
    let c = report::constraint_from_json(&sk, &v).unwrap();
    let id = sk.add(c);
    assert_eq!(value(&sk, id), 4.0);
    let bad = gcs_core::json::parse(
        r#"{"type": "Distance", "args": [["point", 0], ["point", 1], "w +"]}"#).unwrap();
    assert!(report::constraint_from_json(&sk, &bad).is_err());
    // a bare number as text is a constant, in the units a person writes — the rule a dimension
    // typed into the app at creation follows, the same as `set_dimension`'s
    let num = gcs_core::json::parse(
        r#"{"type": "Distance", "args": [["point", 0], ["point", 1], " 7 "]}"#).unwrap();
    assert_eq!(report::constraint_from_json(&sk, &num).unwrap().args[2], Arg::Num(7.0));
    let mut sk2 = Sketch::new();
    let l1 = sk2.line_xy(0.0, 0.0, 10.0, 0.0, "l1");
    let l2 = sk2.line_xy(0.0, 0.0, 10.0, 5.0, "l2");
    let ang = gcs_core::json::parse(
        r#"{"type": "Angle", "args": [["line", 0], ["line", 1], "30"]}"#).unwrap();
    let _ = (l1, l2);
    assert_eq!(report::constraint_from_json(&sk2, &ang).unwrap().args[2],
               Arg::Num(30f64.to_radians()));
    let ang = gcs_core::json::parse(
        r#"{"type": "Angle", "args": [["line", 0], ["line", 1], "a = 30"]}"#).unwrap();
    let id = sk2.add(report::constraint_from_json(&sk2, &ang).unwrap());
    assert!((value(&sk2, id) - 30f64.to_radians()).abs() < 1e-12);
    // the report lists them in evaluation order
    let items = report::exprs_json(&mut sk);
    let texts: Vec<&str> = items.arr().iter().map(|it| it.get("text").unwrap().as_str()).collect();
    assert_eq!(texts, vec!["w = 3", "h = w * 2", "w + 1"]);
}

#[test]
fn describe_and_callout_text() {
    let (sk, ids) = three(["w = 3", "h = w * 2", "sin(h * 5)"]);
    let c = |i: usize| sk.constraint(ids[i]).unwrap();
    assert_eq!(io::describe(c(0)), "Distance(P0, P1, w = 3 = 3)");
    assert_eq!(io::describe(c(1)), "Distance(P2, P3, h = w * 2 = 6)");
    assert_eq!(io::describe(c(2)), "Distance(P4, P5, sin(h * 5) = 0.5)");
    assert_eq!(io::dimension_text(c(0)).unwrap(), "w=3");
    assert_eq!(io::dimension_text(c(1)).unwrap(), "h=6");
    assert_eq!(io::dimension_text(c(2)).unwrap(), "=0.5");
}

#[test]
fn expressions_survive_rebuilds_and_a_paste_reports_its_duplicates() {
    let (sk, ids) = three(["w = 3", "h = w * 2", "h + 1"]);
    // deleting the definition: the readers keep their numbers and say what is missing
    let sk2 = io::without(&sk, &[], &[ids[0]]);
    assert_eq!(sk2.constraints.len(), 2);
    assert_eq!(sk2.constraints[0].args[2].num(), 6.0);
    let mut sk2 = sk2;
    let items = expr::evaluate(&mut sk2);
    assert_eq!(items[0].error.as_deref(), Some("`w` is not defined"));
    // a copy of everything pasted back: every name is now defined twice
    let clip = io::copy(&sk, &sk.primitives());
    let mut doc = sk.clone();
    io::paste(&mut doc, &clip, 100.0, 0.0);
    assert_eq!(doc.constraints.len(), 6);
    let items = expr::evaluate(&mut doc);
    let dups = items.iter().filter(|it| it.error.as_deref() == Some("`w` is defined more than once")).count();
    assert!(dups >= 2, "{items:?}");
    assert_eq!(doc.constraints[4].args[2].num(), 6.0);   // the pasted h kept its number
    // the part a drag works on is a rebuild too: its expressions come along as numbers
    let part = io::Part::around(&sk, EntRef::point(2));
    assert_eq!(part.sketch.constraints[0].args[2].num(), 6.0);
}

/// The graphical proof of the Pythagorean theorem, as a sketch: four a×b right triangles in a
/// square of side a + b leave a square whose side is dimensioned `c = hypot(a, b)`.  The figure
/// satisfies that equation without being made to — it is redundant and consistent — and goes on
/// satisfying it when a leg is edited, which is what makes it a proof and not a coincidence.
#[test]
fn pythagoras_drawn_with_expressions_holds_and_stays_true_when_a_leg_is_edited() {
    let mut sk = examples::pythagoras(30.0, 40.0);
    let hypotenuses = |sk: &Sketch| -> Vec<f64> {
        let n = sk.lines.len();
        (n - 4..n).map(|i| sk.line_length(i)).collect()
    };
    let by_text = |sk: &Sketch, t: &str| sk.constraints.iter().find(|c| c.expr_text("d") == Some(t)).unwrap().id;
    let check = |sk: &mut Sketch, a: f64, b: f64| {
        assert!(solve(sk, SolveOpts::default()).success);
        let c = a.hypot(b);
        for h in hypotenuses(sk) {
            assert!((h - c).abs() < 1e-6, "hypotenuse {h} for legs {a}, {b}");
        }
        let cc = sk.constraint(by_text(sk, "c = hypot(a, b)")).unwrap();
        assert!((cc.args[2].num() - c).abs() < 1e-9);   // the expression computed it
        assert!(cc.error(sk) < 1e-6);                      // and the figure agrees
        assert_eq!(io::dimension_text(cc).unwrap(), format!("c={}", gcs_core::json::fmt_g(c, 4)));
        let d = diagnose(sk, DiagnoseOptions::default());
        assert_eq!(d.dof, 0);
        assert_eq!(d.n_redundant, 1, "the theorem is one equation the construction already holds");
        assert!(d.violated.is_empty() && d.conflicts.as_deref().unwrap_or(&[]).is_empty(),
                "redundant but consistent");
    };
    check(&mut sk, 30.0, 40.0);
    // edit a leg: everything that reads `a` follows, and the theorem still holds
    let a_id = by_text(&sk, "a = 30");
    assert_eq!(expr::set_dimension(&mut sk, a_id, "d", "a = 50").unwrap(), None);
    check(&mut sk, 50.0, 40.0);
    let b_id = by_text(&sk, "b = 40");
    assert_eq!(expr::set_dimension(&mut sk, b_id, "d", "b = 12").unwrap(), None);
    check(&mut sk, 50.0, 12.0);
    // and the case library builds it with any legs
    let sk2 = examples::case("pythagoras:5:12").unwrap();
    assert!((hypotenuses(&sk2)[0] - 13.0).abs() < 1e-9);
}

/* -- mixed numbers ------------------------------------------------------------ */

fn val(text: &str) -> f64 {
    let p = expr::parse(text).unwrap_or_else(|e| panic!("{text}: {e}"));
    expr::eval(&p.body, &BTreeMap::new()).unwrap_or_else(|e| panic!("{text}: {e}"))
}

#[test]
fn a_number_may_be_written_as_a_mixed_fraction() {
    for (text, want) in [
        ("3 1/2", 3.5),
        ("0 3/4", 0.75),
        ("12 15/16", 12.9375),
        ("3   1/2", 3.5),          // any run of space
        ("-3 1/2", -3.5),          // the sign is the parser's, as for any number
        ("2 * 3 1/2", 7.0),        // it is one number, so it multiplies as one
        ("3 1/2 + 1/4", 3.75),
        ("(1 1/2) * 4", 6.0),
    ] {
        assert!((val(text) - want).abs() < 1e-12, "{text} came to {}", val(text));
    }
}

#[test]
fn a_fraction_without_a_whole_number_is_still_a_division() {
    assert_eq!(val("1/2"), 0.5);
    assert_eq!(val("31/2"), 15.5); // no space: not three-and-a-bit
    assert_eq!(val("3.5/2"), 1.75); // a decimal takes no fraction
}

#[test]
fn what_only_looks_like_a_mixed_number_is_read_the_ordinary_way() {
    // a name is not a numerator, so this stays three, x, over two — and juxtaposition is an error
    assert!(expr::parse("3 x/2").is_err());
    assert!(expr::parse("3 1/").is_err());
    assert!(expr::parse("3 1/0").is_err(), "a fraction over nothing is not a number");
    // a whole number followed by another whole number is not a number either
    assert!(expr::parse("3 1").is_err());
    // the fraction is written tight, the way a drawing writes it; loosening it would only
    // widen what can be mistaken for a mixed number
    assert!(expr::parse("3 1 / 2").is_err());
}

#[test]
fn a_mixed_number_is_kept_as_written_rather_than_collapsed() {
    // `literal` decides whether typed text is stored as a bare number or kept as text.  A mixed
    // fraction is deliberately kept: it is a number, but written a particular way, and the way
    // is worth having.
    assert_eq!(expr::literal("3 1/2"), None);
    assert_eq!(expr::literal("  5  "), Some(5.0));
    assert_eq!(expr::literal("-2.5"), Some(-2.5));
    assert_eq!(expr::literal("1e3"), Some(1000.0));
    assert_eq!(expr::literal("inf"), None);

    // `notation` is what says the kept text is a number and not a computation, so the drawing
    // prints it as written
    assert_eq!(expr::notation("3 1/2"), Some(3.5));
    assert_eq!(expr::notation("-2 3/8"), Some(-2.375));
    assert_eq!(expr::notation("  12 15/16 "), Some(12.9375));
    assert_eq!(expr::notation("5"), None, "digits already; nothing to remember");
    assert_eq!(expr::notation("-2.5"), None);
    assert_eq!(expr::notation("w = 3 1/2"), None, "a name is not a notation");
    assert_eq!(expr::notation("3 1/2 + w"), None);
    assert_eq!(expr::notation("1/2"), None, "a division is a computation");
}

#[test]
fn a_dimension_written_as_a_fraction_is_drawn_as_one() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(3.125, 0.0, false, "b");
    let id = sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 1.0));
    expr::set_dimension(&mut sk, id, "d", "3 1/8").unwrap();
    let c = sk.constraint(id).unwrap();

    assert_eq!(io::dimension_text(c).as_deref(), Some("3 1/8"), "the callout lost the fraction");
    assert_eq!(io::describe(c), "Distance(P0, P1, 3 1/8)");
    assert_eq!(c.args[2].num(), 3.125, "and the graph still has the number");
    assert!(solve(&mut sk, SolveOpts::default()).success);

    // a formula still shows what it came to, because that is the part a reader cannot work out
    expr::set_dimension(&mut sk, id, "d", "w = 2 1/4").unwrap();
    assert_eq!(io::dimension_text(sk.constraint(id).unwrap()).as_deref(), Some("w=2.25"));
}

#[test]
fn an_angle_written_as_a_fraction_keeps_its_degrees() {
    let mut sk = Sketch::new();
    let l1 = sk.line_xy(0.0, 0.0, 10.0, 0.0, "l1");
    let l2 = sk.line_xy(0.0, 0.0, 10.0, 1.0, "l2");
    let id = sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(0.1)],
    ));
    expr::set_dimension(&mut sk, id, "theta", "22 1/2").unwrap();
    let c = sk.constraint(id).unwrap();
    assert_eq!(io::dimension_text(c).as_deref(), Some("22 1/2°"));
    // the text is degrees, the value is radians, as for every other written angle
    assert!((c.args[2].num() - 22.5_f64.to_radians()).abs() < 1e-12);
}
