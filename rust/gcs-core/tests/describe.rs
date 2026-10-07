//! What a constraint says when it is read out — `io::describe`, `io::dimension_text` — and
//! the three ways issue #43 found it saying it wrong: an angle written `60deg` drawn as
//! `60deg°` (#14), a bare number printed to every digit a double has in the list and to four
//! on the drawing (#15), and culprits named `P0` when the source called the point `corner`
//! (#16).  One reading per number and one namer per report, both the core's.

use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "does not elaborate: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    e
}

fn dims(sk: &Sketch) -> Vec<String> {
    sk.user_constraints().iter().filter_map(|c| io::dimension_text(c)).collect()
}

#[test]
fn an_angle_that_names_its_unit_is_not_given_a_second_one() {
    let e = read(
        "\
use std
in std.front {
o := point
         a := point hint((40, 0))
         b := point hint((20, 30))
         oa := line(o, a)
         ob := line(o, b)
         oa angle(60deg) ob
         fix((0, 0)) o
}
",
    );
    assert_eq!(dims(&e.sketch), ["60deg"], "the unit as written, once");
    // a bare number in an angle slot takes the sign a reader expects; a fraction is a bare number
    let e = read("use std\nin std.front {\no := point hint((0, 0))\na := point hint((40, 0))\nb := point hint((20, 30))\noa := line(o, a)\nob := line(o, b)\noa angle(60) ob\n}\n");
    assert_eq!(dims(&e.sketch), ["60°"]);
    let e = read("use std\nin std.front {\no := point hint((0, 0))\na := point hint((40, 0))\nb := point hint((20, 30))\noa := line(o, a)\nob := line(o, b)\noa angle(22 1/2) ob\n}\n");
    assert_eq!(dims(&e.sketch), ["22 1/2°"]);
    // and a length that names its unit is left as it was, in either slot
    let e = read("unit mm\nuse std\nin std.front {\na := point hint((0, 0))\nb := point hint((40, 0))\na distance(4cm) b\n}\n");
    assert_eq!(dims(&e.sketch), ["4cm"]);
}

#[test]
fn the_list_and_the_callout_print_one_number_one_way() {
    // a `param` substituted inside a block leaves a bare number, and that number is what a
    // double made of 100 * sin(30°): 49.99999999999999
    let e = read(
        "pcd := 100
         n := 6
         repeat 1 as i {
           a := point hint((50, 0))
           b := point hint((25, 43))
           a distance(pcd * sin(180deg / n)) b
         }",
    );
    let c = &e.sketch.user_constraints()[0];
    let on_drawing = io::dimension_text(c).unwrap();
    let in_list = io::describe(c);
    assert_eq!(on_drawing, "50", "{on_drawing}");
    assert!(in_list.ends_with("distance(50) P1"), "{in_list}");
    // six digits keeps what four dropped
    let e = read("use std\nin std.front {\na := point hint((0, 0))\nb := point hint((1234.5, 0))\na distance(1234.5) b\n}\n");
    let c = &e.sketch.user_constraints()[0];
    assert_eq!(io::dimension_text(c).unwrap(), "1234.5");
    assert_eq!(io::describe(c), "P0 distance(1234.5) P1");
    // an angle in a list is in degrees and carries no sign, as the source writes it
    let e = read("use std\nin std.front {\no := point hint((0, 0))\na := point hint((40, 0))\nb := point hint((20, 30))\noa := line(o, a)\nob := line(o, b)\noa angle(60) ob\n}\n");
    assert_eq!(io::describe(&e.sketch.user_constraints()[0]), "L0 angle(60) L1");
}

#[test]
fn a_culprit_is_named_as_the_source_names_it() {
    let e = read(
        "\
use std
in std.front {
corner := point
         along := point  hint((60, 0))
         base := line(corner, along)
         horizontal base
         corner distance(60) along
         fix((0, 0)) corner
}
",
    );
    let name = |x| e.map.name_of(x).cloned();
    let texts: Vec<String> =
        e.sketch.user_constraints().iter().map(|c| io::describe_with(c, &name)).collect();
    assert_eq!(texts, ["horizontal base", "corner distance(60) along"]);
    // a named line with an anonymous child: the line by its name, and without a namer the
    // sketch's own label
    let e = read("use std\nin std.front {\np := point hint((0, 0))\nl := line(p, hint((10, 0)))\nhorizontal l\n}\n");
    let name = |x| e.map.name_of(x).cloned();
    assert_eq!(io::describe_with(&e.sketch.user_constraints()[0], &name), "horizontal l");
    // without a namer, the sketch's own labels
    assert_eq!(io::describe(&e.sketch.user_constraints()[0]), "horizontal L0");
}

/// **The report carries coordinates** (issue #48, item 3).  `report::positions` is the source
/// map's names against the sketch's own numbers and nothing else: an anonymous child answers by
/// the dotted path that *is* its name, a declaration inside an instance answers under the
/// instance's prefix, and a formal is no second key — aliasing makes one entity of two names, so
/// what the caller passed answers under the name it was declared with.
#[test]
fn the_report_says_where_a_name_landed() {
    let e = read(
        "\
unit mm
use std (vertical)
component Arm(hub: point, tip: point) { hub distance(40) tip }
in std.front {
o := point
t := point hint((5, 40))
fix((0, 0)) o
o vertical t
a := Arm(o, t)
c := circle(center: o) hint(r: 25)
radius(25) c
l := line(o, hint((30, 0)))
horizontal l
o distance(30) l.p2
}
",
    );
    let mut sk = e.sketch;
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let p: std::collections::BTreeMap<String, f64> =
        gcs_core::report::positions(&sk, &e.map).into_iter().collect();
    let at = |n: &str| *p.get(n).unwrap_or_else(|| panic!("no `{n}` in {:?}", p.keys()));
    assert_eq!((at("o.x"), at("o.y")), (0.0, 0.0));
    // a circle's own number, and its centre under the child's name
    assert!((at("c.r") - 25.0).abs() < 1e-9);
    assert_eq!(at("c.center.x"), at("o.x"));
    // the anonymous end of the line, by the path that is its name
    assert!((at("l.p2.x") - 30.0).abs() < 1e-6 && at("l.p2.y").abs() < 1e-6);
    // what the instance was given answers under its own name, and the arm placed it
    assert!((at("t.y") - 40.0).abs() < 1e-6, "{}", at("t.y"));
    assert!(!p.contains_key("a.hub.x"), "a formal is not a second key: {:?}", p.keys());
    // a declaration *inside* the instance would answer under `a.`, and this one makes none
    assert!(p.keys().all(|k| !k.starts_with("a.")), "{:?}", p.keys());
}

/// A plane reports which way it faces and where it stands, its axes' directions and its origin.
#[test]
fn a_plane_reports_its_attitude_and_its_place() {
    let e = read("unit mm\nuse std\nv := plane\nfix(origin == (0, 5, 0)) v\nfix(dir == (0, 0, 1)) v.u\nfix(dir == (1, 0, 0)) v.v\n");
    let mut sk = e.sketch;
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let p: std::collections::BTreeMap<String, f64> =
        gcs_core::report::positions(&sk, &e.map).into_iter().collect();
    // right along z, out along z × x, which is y, standing 5 along it
    assert!((p["v.u.z"] - 1.0).abs() < 1e-12 && (p["v.n.y"] - 1.0).abs() < 1e-12, "{p:?}");
    assert_eq!(p["v.y"], 5.0);
    assert_eq!(p["v.origin.x"], 0.0);
}

#[test]
fn a_dimension_written_over_a_param_is_drawn_with_the_name() {
    // the flattener settles `w` to 100 before the sketch sees it; the callout draws what was
    // written
    let e = read(
        "\
use std
w := 100
         a := 30deg
         in std.front {
         o := point hint((0, 0))
         p := point hint((100, 0))
         q := point hint((100, 50))
         l1 := line(o, p)
         l2 := line(o, q)
         o distance(w) p
         p distance(w / 2) q
         o distance(112) q
         l1 angle(a) l2
         }
",
    );
    assert_eq!(dims(&e.sketch), ["w", "w / 2", "112", "a"]);
    let c = &e.sketch.user_constraints()[0];
    assert_eq!(c.args[2].num(), 100.0, "the number is still what the solve reads");
    // writing a number is what the statement says from then on
    let mut sk = e.sketch.clone();
    let id = sk.user_constraints()[0].id;
    sk.set_constraint_num(id, "d", 120.0);
    assert_eq!(io::dimension_text(sk.constraint(id).unwrap()).unwrap(), "120");
    let id = sk.user_constraints()[1].id;
    gcs_core::expr::set_dimension(&mut sk, id, "d", "60").unwrap();
    assert_eq!(io::dimension_text(sk.constraint(id).unwrap()).unwrap(), "60");
    // a component written in this file draws its formula over its formals, which is true of
    // every instance
    let e = read(
        "\
use std
component Bar(a: point, b: point, len: Length, d: group) {
           a distance(len) b
           a distance(d.w / 2) b
           a distance(40) b
         }
         in std.front {
         o := point hint((0, 0))
         p := point hint((40, 0))
         dims := {w: 80}
         one := Bar(o, p, len: 40, d: dims)
         two := Bar(o, p, len: 40, d: dims)
         }
",
    );
    assert_eq!(dims(&e.sketch), ["len", "d.w / 2", "40", "len", "d.w / 2", "40"]);
    // a block's copies share one label and one callout, so they keep the number they came to
    let e = read(
        "\
use std
component Bar(a: point, b: point, len: Length) {
           repeat 1 as i { a distance(len) b }
         }
         in std.front {
         o := point hint((0, 0))
         p := point hint((40, 0))
         bar := Bar(o, p, len: 40)
         }
",
    );
    assert_eq!(dims(&e.sketch), ["40"]);
    // a module's body is a text the document is not: its number is drawn.  And a formal named
    // like a used module keeps its name, since a closed body reads no module's number
    let (mut prog, errs) = gcs_core::syntax::parse(
        "\
use std
use parts
         component Local(a: point, b: point, parts: group) { a distance(parts.w / 2) b }
         in std.front {
         o := point hint((0, 0))
         p := point hint((40, 0))
         dims := {w: 80}
         bar := parts.Bar(o, p, len: 40)
         local := Local(o, p, parts: dims)
         }
",
    );
    assert!(errs.is_empty(), "{errs:?}");
    let linked = gcs_core::modules::link(&mut prog, &mut |name| {
        (name == "parts").then(|| {
            "component Bar(a: point, b: point, len: Length) { a distance(len) b }\n".to_string()
        })
        .or_else(|| gcs_core::library::resolve(name))
    });
    assert!(linked.is_empty(), "{linked:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(dims(&e.sketch), ["40", "parts.w / 2"]);
}

#[test]
fn a_number_worked_out_with_its_unit_is_read_at_six_digits() {
    // a formal worked out inside a module's body reaches the sketch as the full-precision text
    // it is computed from, `13.333333333333334mm`; the drawing and the list read it as they read
    // a bare number, and a number somebody wrote with six digits or fewer is left as written
    let (mut prog, errs) = gcs_core::syntax::parse(
        "\
unit mm
use std
         use parts
         in std.front {
         o := point hint((0, 0))
         p := point hint((40, 0))
         bar := parts.Bar(o, p, len: 40mm / 3, turn: 100deg / 3)
         }
",
    );
    assert!(errs.is_empty(), "{errs:?}");
    let linked = gcs_core::modules::link(&mut prog, &mut |name| {
        (name == "parts").then(|| {
            "component Bar(a: point, b: point, len: Length, turn: Angle) {
               a distance(len) b
               a distance(len / 3) b
               a distance(1' 6 3/16\") b
               a distance(2.5mm) b
               l := line(a, b)
               m := line(a, b)
               l angle(turn) m
             }\n"
                .to_string()
        })
        .or_else(|| gcs_core::library::resolve(name))
    });
    assert!(linked.is_empty(), "{linked:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(dims(&e.sketch), ["13.3333mm", "4.44444mm", "1' 6 3/16\"", "2.5mm", "33.3333deg"]);
    let list: Vec<String> = e.sketch.user_constraints().iter().map(|c| io::describe(c)).collect();
    assert!(list[0].contains("distance(13.3333mm)"), "{}", list[0]);
}

/// Which way is drawn, not printed: a distance stated `side: right` of a line, or a run stated
/// `along: left`, is labelled with its magnitude, the figure standing where the word puts it.
/// Only a directed angle keeps its minus, since its arc sweeps that way (`refusals.rs`).  The
/// side used to reach the angle's negation, so a flange's radii read `-15`.
#[test]
fn a_side_or_a_way_is_drawn_and_not_printed_as_a_sign() {
    let e = read(
        "\
unit mm
use std
         r := 15mm
         in std.front {
         o := point
         t := point
         p := point hint((15, 5))
         q := point hint((-60, 0))
         ax := line(o, t)
         fix((0, 0)) o
         fix((0, 1)) t
         p distance(15, side: right) ax
         p distance(r, side: right) ax
         p distance(15, side: left) ax
         o distance(60, along: left) q
         }
",
    );
    assert_eq!(dims(&e.sketch), ["15", "r", "15", "60"]);
}
