//! The advisory component classifier (#452, `docs/solve-boundaries-plan.md`): one component of
//! each class, read through an instance in a solved document — the review's two-root point
//! among them, locally determined with its branch left to the seeds.
use crate::common::linked::solved;
use gcs_core::boundary::{classify, Class, Classified, Reads};

/// Every component of `src`, classified at its solve.
fn classes(src: &str) -> Vec<Classified> {
    let e = solved(src);
    classify(&e, &e.sketch)
}

fn named<'a>(cs: &'a [Classified], name: &str) -> &'a Classified {
    cs.iter().find(|c| c.name == name).unwrap_or_else(|| panic!("no component `{name}`"))
}

/// `components` and `calls` over two points six apart in the front plane, classified.
fn over_two_points(components: &str, calls: &str) -> Vec<Classified> {
    classes(&format!("\
unit mm
use std (horizontal)
{components}
in std.front {{
  a := point
  b := point
  fix((0, 0)) a
  fix((6, 0)) b
}}
{calls}"))
}

/// The review's counterexample (§1): `x distance(5) a`, `x distance(5) b`, six apart.  Full rank
/// at `(3, 4)` and at `(3, −4)`: locally determined, and the search finds both roots, so which it
/// returns is the seeds' choice.  Its seed is written from its operands, so it reads nothing of
/// the plane's frame; the same body seeded at `(3, 4)` in the plane does.
#[test]
fn the_two_root_point_is_locally_determined_and_its_branch_not_chosen() {
    let cs = over_two_points("\
component Apex(view: plane, a: point, b: point) {
  in view {
    x := point hint(at: a, toward: b, by: 0.8333, turn: 53.13deg)
    x distance(5mm) a
    x distance(5mm) b
  }
}
component Paged(view: plane, a: point, b: point) {
  in view {
    x := point hint((3, 4))
    x distance(5mm) a
    x distance(5mm) b
  }
}
", "\
one := Apex(std.front, a, b)
two := Paged(std.front, a, b)
");
    let apex = named(&cs, "Apex");
    assert_eq!(apex.class, Class::LocallyDetermined, "{apex:?}");
    let a = apex.analysis.as_ref().unwrap();
    assert_eq!((a.unknowns, a.rank_here, a.rank), (2, 2, 2));
    let roots = a.roots.unwrap();
    assert_eq!((roots.admitted, roots.refused), (2, 0), "{roots:?}");
    assert!(a.world.is_empty(), "{:?}", a.world);
    let paged = named(&cs, "Paged");
    assert_eq!(paged.class, Class::LocallyDetermined);
    let w = &paged.analysis.as_ref().unwrap().world;
    let seed = (Reads::Seed, "seeds `x` at coordinates in its plane".to_string());
    assert_eq!(w, &[seed], "{w:?}");
}

/// With `ccw(a, b, x)` stated, one of the two roots is refused by the choice the body states: the
/// branch is chosen, by a heuristic and by a choice the document's solve holds it to (#157).  With
/// `b` on `a` the two circles are one, and `x` slides round it: rank 1 at the instance's own
/// inputs, though 2 at a generic input, where its roots are searched for.
#[test]
fn a_stated_orientation_chooses_the_branch_and_a_degenerate_input_is_singular_there() {
    let cs = over_two_points("\
component Chosen(view: plane, a: point, b: point) {
  in view {
    x := point hint(at: a, toward: b, by: 0.8333, turn: 53.13deg)
    x distance(5mm) a
    x distance(5mm) b
    ccw(a, b, x)
  }
}
", "\
two := Chosen(std.front, a, b)
");
    let c = named(&cs, "Chosen");
    assert_eq!(c.class, Class::BranchChosen, "{c:?}");
    let a = c.analysis.as_ref().unwrap();
    assert_eq!(a.stated, 1);
    let roots = a.roots.unwrap();
    assert_eq!((roots.admitted, roots.refused), (1, 1), "{roots:?}");

    let cs = classes("\
unit mm
use std
component Apex(view: plane, a: point, b: point) {
  in view {
    x := point hint((3, 4))
    x distance(5mm) a
    x distance(5mm) b
  }
}
in std.front {
  a := point
  b := point
  fix((0, 0)) a
  fix((0, 0)) b
}
one := Apex(std.front, a, b)
");
    let a = named(&cs, "Apex").analysis.as_ref().unwrap();
    assert_eq!((a.rank_here, a.rank), (1, 2), "{a:?}");
    assert!(a.roots.unwrap().moved);
}

/// A body whose one row needs only its own point is determined there and has one root: its
/// branch is chosen, there being none to choose.  A point held at a distance from one operand
/// slides round it: it leaves a freedom, named.
#[test]
fn a_determined_body_has_its_branch_and_a_sliding_point_is_a_freedom() {
    let cs = over_two_points("\
component Middle(view: plane, a: point, b: point) {
  in view {
    m := point hint(at: a)
    m midpoint line(a, b)
  }
}
component Slider(view: plane, a: point, b: point) {
  in view {
    s := point hint(at: a, toward: b, by: 0.5, turn: 30deg)
    s distance(5mm) a
  }
}
", "\
mid := Middle(std.front, a, b)
sl := Slider(std.front, a, b)
");
    let m = named(&cs, "Middle");
    assert_eq!(m.class, Class::BranchChosen, "{m:?}");
    assert_eq!(m.analysis.as_ref().unwrap().roots.unwrap().admitted, 1);
    let s = named(&cs, "Slider");
    assert_eq!(s.class, Class::LeavesFreedoms, "{s:?}");
    let a = s.analysis.as_ref().unwrap();
    assert_eq!((a.unknowns, a.rank), (2, 1));
    assert_eq!(a.freedoms, ["sl.s"]);
    assert!(a.roots.is_none());
}

/// A body that declares nothing and relates its operands is a verb (FR1): `a distance(6) b` over
/// two formals, said as the row over its operands alone.  `std.Spin` is one too: it holds its
/// frame's numbers by a `fix` and places its axis by rows.
#[test]
fn a_body_relating_only_its_operands_asserts_only() {
    let cs = over_two_points("\
component Apart(a: point, b: point) {
  a distance(6mm) b
}
", "\
ap := Apart(a, b)
in std.front {
  c := point
  fix((0, 0)) c
}
f := frame
std.Spin(f, about: c, on: std.front, ratio: 1)
");
    let ap = named(&cs, "Apart");
    assert_eq!(ap.class, Class::AssertsOnly, "{ap:?}");
    let a = ap.analysis.as_ref().unwrap();
    assert_eq!((a.unknowns, a.facts), (0, 1));
    assert_eq!(a.fact_rows, ["a distance(6mm) b"]);
    let spin = named(&cs, "std.Spin");
    assert_eq!(spin.class, Class::AssertsOnly, "{spin:?}");
    assert!(!spin.analysis.as_ref().unwrap().held.is_empty(), "{spin:?}");
}

/// FR2: a crank's angle left unbound is the parent's unknown, read by the body as an input — the
/// crank is determined against it, and says it exports it.  Without the angle the tip swings
/// round the pivot: a freedom the body leaves and no formal names.
#[test]
fn an_unbound_formal_is_an_exported_input_and_an_unknown_of_its_own_a_freedom() {
    let cs = classes("\
unit mm
use std
component Crank(view: plane, o: point, ref: line, theta: Angle) {
  in view {
    tip := point hint(at: o, along: ref, by: 10)
    o distance(10mm) tip
    ref angle(theta) line(o, tip)
  }
}
component Loose(view: plane, o: point, ref: line) {
  in view {
    tip := point hint(at: o, along: ref, by: 10)
    o distance(10mm) tip
  }
}
in std.front {
  o := point
  r := point
  fix((0, 0)) o
  fix((20, 0)) r
  ref := line(o, r)
}
cr := Crank(std.front, o, ref, theta: hint(30deg))
lo := Loose(std.front, o, ref)
");
    let c = named(&cs, "Crank");
    assert!(matches!(c.class, Class::BranchChosen | Class::LocallyDetermined), "{c:?}");
    assert_eq!(c.analysis.as_ref().unwrap().exported, ["cr.theta"]);
    let l = named(&cs, "Loose");
    assert_eq!(l.class, Class::LeavesFreedoms, "{l:?}");
    assert_eq!(l.analysis.as_ref().unwrap().freedoms, ["lo.tip"]);
}

/// Reading the world, beside the class: a row over the plane's own axes, a point held at
/// coordinates, a seed in a plane's frame, one in coordinates and a coordinate read are each
/// said, in words.
#[test]
fn what_a_body_reads_of_the_world_is_said() {
    let cs = over_two_points("\
component Level(view: plane, a: point) {
  in view {
    y := point hint(at: view, (5, 1))
    y horizontal a
    a distance(5mm) y
    private o := point
    fix((1, 1)) o
    z := point hint((a.x + 2mm, a.y + 2mm))
    z distance(2mm) o
    z distance(3mm) y
  }
}
", "\
lv := Level(std.front, a)
");
    let l = named(&cs, "Level");
    let w = &l.analysis.as_ref().unwrap().world;
    for said in ["`lv.y horizontal a` reads its plane's axes", "`fix` holds `lv.o` at coordinates",
                 "seeds `y` at coordinates in `view`'s frame", "reads a coordinate, `a.x`",
                 "seeds `z` at coordinates in its plane"] {
        assert!(w.iter().any(|x| x.1 == said), "{said:?} not in {w:?}");
    }
}

/// A component nothing instantiates is listed and not read; a family of sets is a set.
#[test]
fn an_unused_component_and_a_set_are_not_read() {
    let cs = over_two_points("\
component Idle(a: point) {
  b := point
  a distance(1mm) b
}
component Ball(c: point, r: Length) := { p | p distance(r) c }
", "\
ball := Ball(a, r: 3mm)
");
    assert_eq!(named(&cs, "Idle").class, Class::Unused);
    assert_eq!(named(&cs, "Ball").class, Class::Set);
    assert!(named(&cs, "Idle").analysis.is_none());
}

/// The text `solventc --components` prints is the core's: a line per component, its class first.
#[test]
fn the_text_names_the_class_first() {
    let cs = over_two_points("\
component Apart(a: point, b: point) {
  a distance(6mm) b
}
", "\
ap := Apart(a, b)
");
    let t = gcs_core::boundary::text(named(&cs, "Apart"));
    assert_eq!(t[0], "component `Apart` (1 instance, `ap` read): asserts only");
    assert!(t.iter().any(|l| l.contains("states 1 fact about its operands")), "{t:?}");
}

/// CL3: a count left unbound is an unknown of the drawing today, and no boundary may leave one —
/// said, beside the class.
#[test]
fn an_unbound_count_is_said_to_be_no_boundary() {
    let cs = over_two_points("\
component Rung(view: plane, a: point, n: Int) {
  in view {
    b := point hint(at: view, (3, 4))
    a distance(n * 1mm) b
  }
}
", "\
r := Rung(std.front, a)
");
    let a = named(&cs, "Rung").analysis.as_ref().unwrap();
    assert_eq!(a.counts, ["r.n"]);
    assert!(a.exported.is_empty());
}

/// A component instantiated only in place inside a curve (`trace(Unwind(…).p over u …)`) is a
/// curve's body: the locus solves it along the curve, and nothing of it is drawn to read here.
#[test]
fn a_component_traced_in_a_curve_is_a_curves_body() {
    let cs = classes(include_str!("../../examples/gear_trace.sv"));
    let unwind = named(&cs, "Unwind");
    assert_eq!(unwind.class, Class::Traced, "{unwind:?}");
    assert!(unwind.instances > 0 && unwind.analysis.is_none());
}
