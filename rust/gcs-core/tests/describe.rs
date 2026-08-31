//! What a constraint says when it is read out — `io::describe`, `io::dimension_text` — and
//! the three ways issue #43 found it saying it wrong: an angle written `60deg` drawn as
//! `60deg°` (#14), a bare number printed to every digit a double has in the list and to four
//! on the drawing (#15), and culprits named `P0` when the source called the point `corner`
//! (#16).  One reading per number and one namer per report, both the core's.

use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = gcs_core::syntax::parse(src);
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
        "point o hint(x: 0, y: 0)
         point a hint(x: 40, y: 0)
         point b hint(x: 20, y: 30)
         line oa(o, a)
         line ob(o, b)
         oa angle(60deg) ob
         ground o",
    );
    assert_eq!(dims(&e.sketch), ["60deg"], "the unit as written, once");
    // a bare number in an angle slot takes the sign a reader expects; a fraction is a bare number
    let e = read("point o hint(x: 0, y: 0)\npoint a hint(x: 40, y: 0)\npoint b hint(x: 20, y: 30)\nline oa(o, a)\nline ob(o, b)\noa angle(60) ob\n");
    assert_eq!(dims(&e.sketch), ["60°"]);
    let e = read("point o hint(x: 0, y: 0)\npoint a hint(x: 40, y: 0)\npoint b hint(x: 20, y: 30)\nline oa(o, a)\nline ob(o, b)\noa angle(22 1/2) ob\n");
    assert_eq!(dims(&e.sketch), ["22 1/2°"]);
    // and a length that names its unit is left as it was, in either slot
    let e = read("unit mm\npoint a hint(x: 0, y: 0)\npoint b hint(x: 40, y: 0)\na distance(4cm) b\n");
    assert_eq!(dims(&e.sketch), ["4cm"]);
}

#[test]
fn the_list_and_the_callout_print_one_number_one_way() {
    // a `param` substituted inside a block leaves a bare number, and that number is what a
    // double made of 100 * sin(30°): 49.99999999999999
    let e = read(
        "param pcd = 100
         param n = 6
         repeat 1 as i {
           point a hint(x: 50, y: 0)
           point b hint(x: 25, y: 43)
           a distance(pcd * sin(180deg / n)) b
         }",
    );
    let c = &e.sketch.user_constraints()[0];
    let on_drawing = io::dimension_text(c).unwrap();
    let in_list = io::describe(c);
    assert_eq!(on_drawing, "50", "{on_drawing}");
    assert!(in_list.ends_with("distance(50) P1"), "{in_list}");
    // six digits keeps what four dropped
    let e = read("point a hint(x: 0, y: 0)\npoint b hint(x: 1234.5, y: 0)\na distance(1234.5) b\n");
    let c = &e.sketch.user_constraints()[0];
    assert_eq!(io::dimension_text(c).unwrap(), "1234.5");
    assert_eq!(io::describe(c), "P0 distance(1234.5) P1");
    // an angle in a list is in degrees and carries no sign, as the source writes it
    let e = read("point o hint(x: 0, y: 0)\npoint a hint(x: 40, y: 0)\npoint b hint(x: 20, y: 30)\nline oa(o, a)\nline ob(o, b)\noa angle(60) ob\n");
    assert_eq!(io::describe(&e.sketch.user_constraints()[0]), "L0 angle(60) L1");
}

#[test]
fn a_culprit_is_named_as_the_source_names_it() {
    let e = read(
        "point corner hint(x: 0, y: 0)
         point along  hint(x: 60, y: 0)
         line base(corner, along)
         horizontal base
         corner distance(60) along
         ground corner",
    );
    let name = |x| e.map.name_of(x).cloned();
    let texts: Vec<String> =
        e.sketch.user_constraints().iter().map(|c| io::describe_with(c, &name)).collect();
    assert_eq!(texts, ["horizontal base", "corner distance(60) along"]);
    // a named line with an anonymous child: the line by its name, and without a namer the
    // sketch's own label
    let e = read("point p hint(x: 0, y: 0)\nline l(p, hint(x: 10, y: 0))\nhorizontal l\n");
    let name = |x| e.map.name_of(x).cloned();
    assert_eq!(io::describe_with(&e.sketch.user_constraints()[0], &name), "horizontal l");
    // without a namer, the sketch's own labels
    assert_eq!(io::describe(&e.sketch.user_constraints()[0]), "horizontal L0");
}
