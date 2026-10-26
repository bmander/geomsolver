//! A definition's standard datums are those of the frame it stands in (#524, #542): `std.front`
//! in a body names the datum of the frame its instance stands in — the world's at the root, and
//! the frame's own under a frame: its axes, the planes over them, and its origin, standing where
//! the frame does.  A body is then drawn once and placed by its frame: what it makes in a frame
//! moved by a rigid motion is what it makes at the root, moved by that motion.  And from outside
//! (#548), `b.std.D` names the same datums of instance `b`, which then stands in a square frame of
//! its own that relations on them place.
use gcs_core::diagnose::{diagnose, view_freedoms, DiagnoseOptions};
use gcs_core::program::{elaborate, Code, Elaborated};
use gcs_core::syntax::parse;
use gcs_core::modules::link;
use crate::common::at;
use crate::common::linked::{build, solved};

/// A post standing on the front's origin, `len` up and 3 out: drawn in its view by its own
/// `std` reads, or handed the view and the origin.
const READS: &str = "\
unit mm
use std
component Post(len: Length) {
  in std.front {
    a := point
    a coincident std.origin
    b := point hint((3, len))
    b distance(3mm, along: u) std.front
    a distance(len) b
  }
}
";
const HANDED: &str = "\
unit mm
use std
component Post(view: plane, o: point, len: Length) {
  in view {
    a := point
    a coincident o
    b := point hint((3, len))
    b distance(3mm, along: u) view
    a distance(len) b
  }
}
";

/// A body reading every standard datum: a point in each plane, placed by ordinates in it, and
/// one beside each along an axis.  The points, by name.
const PROBE: &str = "\
unit mm
use std
component Probe(len: Length) {
  in std.front {
    a := point
    a coincident std.origin
    b := point hint((3, len))
    b distance(3mm, along: u) std.front
    a distance(len) b
  }
  in std.top {
    c := point hint((5, 7))
    c distance(5mm, along: u) std.top
    c distance(7mm, along: v) std.top
    j := point hint((5, 20))
    c distance(13mm, along: std.y) j
    c level(std.x) j
    g := point hint((-6, 7))
    c distance(11mm, along: std.back) g
    c level(std.y) g
  }
  in std.side {
    d := point hint((2, 9))
    d distance(2mm, along: u) std.side
    d distance(9mm, along: v) std.side
    h := point hint((2, 20))
    d distance(11mm, along: std.z) h
    d level(std.y) h
  }
  in std.up {
    e := point hint((4, 6))
    e distance(4mm, along: u) std.up
    e distance(6mm, along: v) std.up
    m := point hint((4, -5))
    e distance(11mm, along: std.x) m
    e level(std.z) m
  }
}
";
const POINTS: [&str; 9] = ["a", "b", "c", "j", "g", "d", "h", "e", "m"];

/// A frame's pose, by its axes and origin: `fix` statements placing `f` there.
struct Pose {
    u: [f64; 3],
    v: [f64; 3],
    o: [f64; 3],
}

impl Pose {
    const WORLD: Pose = Pose { u: [1.0, 0.0, 0.0], v: [0.0, 1.0, 0.0], o: [0.0, 0.0, 0.0] };
    /// Turned off every axis and moved.
    const MOVED: Pose = Pose { u: [0.6, 0.8, 0.0], v: [-0.48, 0.36, 0.8], o: [10.0, -20.0, 5.0] };

    fn place(&self, f: &str) -> String {
        let t = |x: [f64; 3]| format!("({}, {}, {})", x[0], x[1], x[2]);
        format!("fix(dir == {}) {f}.u\nfix(dir == {}) {f}.v\nfix(origin == {}) {f}\n",
                t(self.u), t(self.v), t(self.o))
    }

    /// `p` at the root, where this pose takes it: `o + x·u + y·v + z·(u × v)`.
    fn image(&self, p: [f64; 3]) -> [f64; 3] {
        let (u, v) = (self.u, self.v);
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        [0, 1, 2].map(|k| self.o[k] + p[0] * u[k] + p[1] * v[k] + p[2] * n[k])
    }
}

/// Two places alike to 1e-8.
fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| crate::common::near(a[k], b[k]))
}

/// Every probe point of `framed`'s instance `inst` where `pose` takes the root's `p`.
fn moved(root: &Elaborated, framed: &Elaborated, inst: &str, pose: &Pose) {
    for name in POINTS {
        let (want, got) = (pose.image(at(root, &format!("p.{name}"))),
                           at(framed, &format!("{inst}.{name}")));
        assert!(near(got, want), "{inst}.{name}: {got:?} against {want:?}");
    }
}

#[test]
fn a_body_reading_its_datums_at_the_root_stands_where_one_handed_them_does() {
    let read = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    let handed = solved(&format!("{HANDED}p := Post(std.front, std.origin, len: 40mm)\n"));
    for name in ["p.a", "p.b"] {
        let (r, h) = (at(&read, name), at(&handed, name));
        assert!(near(r, h), "{name}: {r:?} against {h:?}");
    }
    // the same entities, and nothing of the world made twice
    assert_eq!(read.sketch.points.len(), handed.sketch.points.len());
}

#[test]
fn a_frame_at_the_worlds_pose_gives_the_roots_solve() {
    let root = solved(&format!("{PROBE}p := Probe(len: 40mm)\n"));
    let framed = solved(&format!("{PROBE}in (f := frame) {{\n  p := Probe(len: 40mm)\n}}\n{}",
                                 Pose::WORLD.place("f")));
    for name in POINTS {
        let name = format!("p.{name}");
        assert!(near(at(&framed, &name), at(&root, &name)), "{name}: {:?} against {:?}",
                at(&framed, &name), at(&root, &name));
    }
    // the frame's datums are the world's
    for (datum, world) in [("front", "std.front"), ("top", "std.top"), ("side", "std.side"),
                           ("up", "std.up")] {
        let (f, w) = (crate::common::ent(&framed, &format!("f#std.{datum}")),
                      crate::common::ent(&framed, world));
        let (bf, bw) = (framed.sketch.basis(f.i()), framed.sketch.basis(w.i()));
        assert!(near(bf.u, bw.u) && near(bf.v, bw.v) && near(bf.o, bw.o), "{datum}");
    }
}

#[test]
fn a_frame_turned_and_moved_moves_the_body_with_it() {
    let root = solved(&format!("{PROBE}p := Probe(len: 40mm)\n"));
    let framed = solved(&format!("{PROBE}in (f := frame) {{\n  p := Probe(len: 40mm)\n}}\n{}",
                                 Pose::MOVED.place("f")));
    moved(&root, &framed, "p", &Pose::MOVED);
    // the frame's own datums, made once, and drawn in without a word (W115 says nothing)
    assert!(framed.diags.iter().all(|d| d.code != Code::W115), "{:?}", framed.diags);
    // and the same instance placed by a trailer, `in f`, rather than a block
    let trailer = solved(&format!("{PROBE}f := frame\np := Probe(len: 40mm) in f\n{}",
                                  Pose::MOVED.place("f")));
    moved(&root, &trailer, "p", &Pose::MOVED);
}

#[test]
fn two_instances_of_one_definition_stand_in_two_frames() {
    let other = Pose { u: [0.0, 0.0, 1.0], v: [1.0, 0.0, 0.0], o: [-3.0, 4.0, 12.0] };
    let root = solved(&format!("{PROBE}p := Probe(len: 40mm)\n"));
    let src = format!("{PROBE}in (f := frame) {{\n  p := Probe(len: 40mm)\n}}\n\
                       in (g := frame) {{\n  q := Probe(len: 40mm)\n  r := Probe(len: 40mm)\n}}\n\
                       {}{}", Pose::MOVED.place("f"), other.place("g"));
    let e = solved(&src);
    moved(&root, &e, "p", &Pose::MOVED);
    moved(&root, &e, "q", &other);
    moved(&root, &e, "r", &other);
    // each frame's datums are made once, however many instances read them: the frames of the
    // root's, and of each frame its own and its five planes
    let frames = |e: &Elaborated| e.sketch.frames.len();
    assert_eq!(frames(&e), frames(&root) + 2 * 6);
}

#[test]
fn a_bare_frame_placed_by_relations_on_its_datums_is_fully_held() {
    // a seat in space, a bore's axis through it, and a datum across: the frame stands at the
    // seat, its u along the bore and its v up, and nothing is free
    let src = format!("{READS}\
seat := point
fix((10, 20, 30)) seat
bore := axis
fix(dir == (0, 1, 0), origin == (10, 0, 30)) bore
in (f := frame) {{
  p := Post(len: 40mm)
}}
f.origin coincident seat
f.u parallel bore
f.v parallel std.z
");
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    let pose = Pose { u: [0.0, 1.0, 0.0], v: [0.0, 0.0, 1.0], o: [10.0, 20.0, 30.0] };
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    for name in ["p.a", "p.b"] {
        let (want, got) = (pose.image(at(&root, name)), at(&e, name));
        assert!(near(got, want), "{name}: {got:?} against {want:?}");
    }
}

#[test]
fn a_frame_left_unplaced_reports_its_freedoms() {
    let mut e = build(&format!("{READS}in (f := frame) {{\n  p := Post(len: 40mm)\n}}\n"));
    assert!(e.ok(), "{:?}", e.diags);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    // the frame's own seven numbers — two axes' turns and where it stands — and nothing of its
    // datums', which stand where it does
    assert_eq!(d.dof, 7);
    assert_eq!(view_freedoms(&e.sketch, &d), vec!["f.attitude", "f.origin"]);
}

#[test]
fn a_definition_in_a_frame_with_a_joint_rides_it_at_a_roll() {
    // the arm turns about z, and the post drawn in it is carried at a roll with no `under:`
    let src = format!("{READS}\
in std.top {{
  o := point
  fix((0, 0)) o
}}
in (arm := frame) {{
  p := Post(len: 40mm)
}}
{}arm spins(1) o
X := point hint((0, 3, 40))
X coincident(t == 90deg) p.b
", Pose::WORLD.place("arm")).replace("use std\n", "use std\nuse std.motion (spins)\n");
    let e = solved(&src);
    // at rest the post stands as drawn, and at a quarter turn it has turned about z
    let b = at(&e, "p.b");
    assert!(near(at(&e, "X"), [-b[1], b[0], b[2]]), "{:?} against {b:?}", at(&e, "X"));
    let arm = crate::common::ent(&e, "arm").i() as u32;
    assert_eq!(e.sketch.carriers(crate::common::ent(&e, "p.b")).unwrap(), vec![arm]);
}

#[test]
fn an_instance_stands_in_the_innermost_frame_which_rides_those_outside_it() {
    // the inner frame is turned and moved in the world at rest; the outer one turns about z
    let inner = Pose { u: [0.0, 1.0, 0.0], v: [-1.0, 0.0, 0.0], o: [20.0, 0.0, 0.0] };
    let src = format!("{READS}\
in std.top {{
  o := point
  fix((0, 0)) o
}}
in (g := frame) {{
  in (h := frame) {{
    p := Post(len: 40mm)
  }}
}}
{}{}g spins(1) o
X := point hint((0, 20, 40))
X coincident(t == 90deg) p.b
", Pose::WORLD.place("g"), inner.place("h")).replace("use std\n", "use std\nuse std.motion (spins)\n");
    let e = solved(&src);
    assert!(e.diags.iter().all(|d| d.code != Code::W115), "{:?}", e.diags);
    // the post reads the inner frame's datums
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    let b = inner.image(at(&root, "p.b"));
    assert!(near(at(&e, "p.b"), b), "{:?} against {b:?}", at(&e, "p.b"));
    // and rides the inner frame on the outer: a quarter turn of the outer carries it
    assert!(near(at(&e, "X"), [-b[1], b[0], b[2]]), "{:?} against {b:?}", at(&e, "X"));
}

#[test]
fn nested_instances_read_the_same_datums() {
    let src = format!("{READS}\
component Pair() {{
  one := Post(len: 40mm)
  in std.side {{
    two := Post(len: 25mm)
  }}
}}
pair := Pair()
");
    let e = solved(&src);
    // a nested call stands where its parent does, and a view is no frame: both posts read the
    // world's front
    let flat = solved(&format!("{READS}one := Post(len: 40mm)\ntwo := Post(len: 25mm)\n"));
    for (nested, top) in [("pair.one.b", "one.b"), ("pair.two.b", "two.b")] {
        assert!(near(at(&e, nested), at(&flat, top)), "{nested}: {:?} against {:?}",
                at(&e, nested), at(&flat, top));
    }
}

/// A module whose definitions read the standard datums, and one whose file says no `use std`.
fn read_with_module(src: &str) -> Elaborated {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let bare = "component Bare() {\n  in std.front {\n    a := point\n  }\n}\n";
    let linked = link(&mut prog, &mut |name| match name {
        "lib.posts" => Some(READS.to_string()),
        "lib.bare" => Some(bare.to_string()),
        _ => gcs_core::library::resolve(name),
    });
    assert!(linked.is_empty(), "{linked:?}");
    elaborate(&prog)
}

/// The E101 said at the first place `what` is written.
fn refused<'a>(e: &'a Elaborated, src: &str, what: &str) -> &'a gcs_core::program::Diag {
    let at = src.find(what).unwrap_or_else(|| panic!("`{what}` is not in the text"));
    e.diags.iter().find(|d| d.code == Code::E101 && d.span.lo as usize == at)
        .unwrap_or_else(|| panic!("`{what}` is not refused: {:?}", e.diags))
}

#[test]
fn a_modules_definition_reads_its_datums_by_the_modules_own_use() {
    // the document says no `use std`: the module does, and that is what its body reads by
    let mut e = read_with_module("unit mm\nuse lib.posts\np := lib.posts.Post(len: 40mm)\n");
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let handed = solved(&format!("{HANDED}p := Post(std.front, std.origin, len: 40mm)\n"));
    for name in ["p.a", "p.b"] {
        assert!(near(at(&e, name), at(&handed, name)), "{name}");
    }
    // and in a frame, the frame's own, by the module's `use` still
    let src = format!("unit mm\nuse lib.posts\nin (f := frame) {{\n  p := lib.posts.Post(len: \
                       40mm)\n}}\n{}", Pose::MOVED.place("f"));
    let mut e = read_with_module(&src);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    for name in ["p.a", "p.b"] {
        let want = Pose::MOVED.image(at(&handed, name));
        assert!(near(at(&e, name), want), "{name}: {:?} against {want:?}", at(&e, name));
    }
    // and the document reads none by it: `std` is a name of the files that say `use std`
    let src = "unit mm\nuse lib.posts\nq := point in std.front\n";
    let e = read_with_module(src);
    refused(&e, src, "std.front");
    // a module that says none reads none, whatever the document says
    let src = "unit mm\nuse std\nuse lib.bare\nb := lib.bare.Bare()\n";
    let e = read_with_module(src);
    assert!(e.diags.iter().any(|d| d.code == Code::E101 && d.message.contains("std.front")),
            "{:?}", e.diags);
}

#[test]
fn a_body_reads_the_datums_of_the_frame_around_its_call_wherever_that_is_written() {
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    let want = |name: &str| Pose::MOVED.image(at(&root, &format!("p.{name}")));
    // a frame around a call further out, and a frame declared in a body around its call
    for (src, post) in [
        (format!("{READS}component Arm() {{\n  p := Post(len: 40mm)\n}}\n\
                  in (arm := frame) {{\n  a := Arm()\n}}\n{}", Pose::MOVED.place("arm")), "a.p"),
        (format!("{READS}component Arm() {{\n  in (g := frame) {{\n    p := Post(len: 40mm)\n  \
                  }}\n  {}}}\na := Arm()\n", Pose::MOVED.place("g").replace('\n', "\n  ")),
         "a.p"),
    ] {
        let e = solved(&src);
        for name in ["a", "b"] {
            let got = at(&e, &format!("{post}.{name}"));
            assert!(near(got, want(name)), "{post}.{name}: {got:?} against {:?}", want(name));
        }
    }
}

#[test]
fn a_frames_datum_is_handed_on_as_any_name_is() {
    // handed to a call as its view and origin, and to a relation word as its operand: what the
    // frame's own are, made once a body reads one, is read through the alias as any name is
    let src = format!("{}\
component Outer() {{
  p := Post(std.front, std.origin, len: 40mm)
  in std.front {{
    c := point hint((5, 0))
    c on_u(d: 5mm) std.front
  }}
}}
in (f := frame) {{
  o := Outer()
}}
{}", HANDED.replace("use std\n", "use std\nuse std.place (on_u)\n"), Pose::MOVED.place("f"));
    let e = solved(&src);
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    for name in ["a", "b"] {
        let want = Pose::MOVED.image(at(&root, &format!("p.{name}")));
        let got = at(&e, &format!("o.p.{name}"));
        assert!(near(got, want), "o.p.{name}: {got:?} against {want:?}");
    }
    assert!(near(at(&e, "o.c"), Pose::MOVED.image([5.0, 0.0, 0.0])), "{:?}", at(&e, "o.c"));
}

#[test]
fn a_frames_own_datums_are_read_by_no_name_outside_a_body() {
    // made under a key no source can spell: outside a body standing in the frame, `std` is the
    // world's, and the frame's are no name at all
    let src = format!("{READS}in (f := frame) {{\n  p := Post(len: 40mm)\n}}\n\
                       q := point in std.front\n");
    let e = build(&src);
    assert!(e.ok(), "{:?}", e.diags);
    let q = crate::common::ent(&e, "q");
    assert_eq!(e.sketch.points[q.i()].plane, Some(crate::common::ent(&e, "std.front").idx));
}

#[test]
fn a_plane_is_no_frame() {
    // drawn on a view, an instance's `std` still names the world's
    let src = "\
unit mm
use std
component Tick() {
  a := point hint((4, 1))
  b := point
  a distance(5mm, along: u) std.top
  a distance(2mm, along: v) std.top
  b coincident std.top.origin
}
t := Tick() in std.top
";
    let e = solved(src);
    assert!(near(at(&e, "t.a"), [5.0, 2.0, 0.0]), "{:?}", at(&e, "t.a"));
    assert!(near(at(&e, "t.b"), [0.0, 0.0, 0.0]), "{:?}", at(&e, "t.b"));
}

#[test]
fn a_relation_word_still_reads_only_its_operands() {
    let src = "\
unit mm
use std
a lifted(d) b := a distance(d, along: v) std.front
in std.front {
  p := point
  q := point
  p lifted(5) q
}
";
    let e = build(src);
    assert!(e.diags.iter().any(|d| d.code == Code::E101 && d.message.contains("std.front")),
            "{:?}", e.diags);
}

// ---- an instance is a frame (#548): its datums read from outside, `b.std.z` ----

/// A seat in space, a bore's axis through it, and a datum across: what `READS`'s post is mated
/// to, its x along the bore and its y up, by relations on the datums written `x`, `y` and `o`.
fn mated(x: &str, y: &str, o: &str) -> String {
    format!("{SEAT}{o} coincident seat\n{x} parallel bore\n{y} parallel std.z\n")
}

/// The seat and the bore `mated` places the post on.
const SEAT: &str = "\
seat := point
fix((10, 20, 30)) seat
bore := axis
fix(dir == (0, 1, 0), origin == (10, 0, 30)) bore
";

/// Where `mated` puts the post.
const SEATED: Pose = Pose { u: [0.0, 1.0, 0.0], v: [0.0, 0.0, 1.0], o: [10.0, 20.0, 30.0] };

#[test]
fn an_instance_whose_std_nothing_outside_reads_stands_as_before() {
    // no frame of its own: the same entities as before, at the root and in a frame
    for src in [format!("{READS}p := Post(len: 40mm)\n"),
                format!("{READS}in (f := frame) {{\n  p := Post(len: 40mm)\n}}\n{}",
                        Pose::MOVED.place("f"))] {
        let e = solved(&src);
        assert!(e.map.ent_named("p#frame").is_none());
        assert!(e.sketch.plane_names.values().all(|n| n != "p"), "{:?}", e.sketch.plane_names);
    }
    // and a body's own `std` read inside it is no read from outside
    let e = solved(&format!("{READS}component Pair() {{\n  one := Post(len: 40mm)\n  \
                             q := point in std.front\n  q coincident one.a\n}}\npair := Pair()\n"));
    assert!(e.map.ent_named("pair.one#frame").is_none() && e.map.ent_named("pair#frame").is_none());
}

#[test]
fn an_instances_datums_place_it_as_a_frame_written_around_it_does() {
    let by_hand = solved(&format!("{READS}in (f := frame) {{\n  p := Post(len: 40mm)\n}}\n\
                                   f.u perpendicular f.v\n{}", mated("f.u", "f.v", "f.origin")));
    let read = solved(&format!("{READS}p := Post(len: 40mm)\n{}",
                               mated("p.std.x", "p.std.y", "p.std.origin")));
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    for name in ["p.a", "p.b"] {
        let (h, r) = (at(&by_hand, name), at(&read, name));
        assert!(near(h, r), "{name}: {r:?} against {h:?}");
        assert!(near(r, SEATED.image(at(&root, name))), "{name}: {r:?}");
    }
    // the same unknowns, and one row fewer: the instance owns its squareness
    let unknowns = |e: &Elaborated| e.sketch.params.iter().filter(|p| !p.fixed).count();
    assert_eq!(unknowns(&read), unknowns(&by_hand));
    assert_eq!(read.sketch.user_constraints().len() + 1, by_hand.sketch.user_constraints().len());
    // and its datums are the frame's own, read by the body as its `std`
    let p = crate::common::ent(&read, "p.a");
    let front = crate::common::ent(&read, "p#frame#std.front");
    assert_eq!(read.sketch.points[p.i()].plane, Some(front.idx));
}

#[test]
fn a_mate_on_an_instances_datums_holds_it_wholly() {
    let mut e = solved(&format!("{READS}p := Post(len: 40mm)\n{}",
                                mated("p.std.x", "p.std.y", "p.std.origin")));
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    // the other datums agree: `std.z` is the post's x × y, its front the plane over x and z
    let mut e = solved(&format!("{READS}p := Post(len: 40mm)\n{}\
                                 n := axis\nfix(dir == (1, 0, 0), origin == (0, 0, 0)) n\n\
                                 claim p.std.z parallel n\nclaim p.std.back parallel bore\n",
                                mated("p.std.x", "p.std.y", "p.std.origin")));
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let front = e.sketch.basis(crate::common::ent(&e, "p#frame#std.front").i());
    assert!(near(front.u, [0.0, 1.0, 0.0]) && near(front.v, [1.0, 0.0, 0.0]), "{front:?}");
}

#[test]
fn an_unplaced_instance_reports_six_freedoms_under_its_own_name() {
    // its origin read by a seed alone, which places nothing
    let mut e = build(&format!("{READS}p := Post(len: 40mm)\nq := point hint(at: p.std.origin)\n\
                                fix((1, 2, 3)) q\n"));
    assert!(e.ok(), "{:?}", e.diags);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.dof, 6);
    assert_eq!(view_freedoms(&e.sketch, &d), vec!["p.attitude", "p.origin"]);
}

#[test]
fn two_instances_of_one_definition_are_placed_by_two_mates() {
    let src = format!("{READS}p := Post(len: 40mm)\nq := Post(len: 25mm)\n{}\
                       other := point\nfix((10, -20, 5)) other\n\
                       d1 := axis\nfix(dir == (0.6, 0.8, 0), origin == (0, 0, 0)) d1\n\
                       d2 := axis\nfix(dir == (-0.48, 0.36, 0.8), origin == (0, 0, 0)) d2\n\
                       q.std.origin coincident other\nq.std.x parallel d1\nq.std.y parallel d2\n",
                      mated("p.std.x", "p.std.y", "p.std.origin"));
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    let (r40, r25) = (solved(&format!("{READS}p := Post(len: 40mm)\n")),
                      solved(&format!("{READS}p := Post(len: 25mm)\n")));
    for (inst, root, pose) in [("p", &r40, &SEATED), ("q", &r25, &Pose::MOVED)] {
        for name in ["a", "b"] {
            let (got, want) = (at(&e, &format!("{inst}.{name}")),
                               pose.image(at(root, &format!("p.{name}"))));
            assert!(near(got, want), "{inst}.{name}: {got:?} against {want:?}");
        }
    }
}

#[test]
fn an_instance_in_a_frame_with_a_joint_rides_it() {
    // the post's own frame is mated at rest, and rides the arm around its call, which turns
    // about z: a quarter turn carries the post whole
    let src = format!("{READS}\
in std.top {{
  o := point
  fix((0, 0)) o
}}
in (arm := frame) {{
  p := Post(len: 40mm)
}}
{}arm spins(1) o
{}X := point hint((-20, 10, 70))
X coincident(t == 90deg) p.b
", Pose::WORLD.place("arm"), mated("p.std.x", "p.std.y", "p.std.origin"))
        .replace("use std\n", "use std\nuse std.motion (spins)\n");
    let e = solved(&src);
    let b = at(&e, "p.b");
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    assert!(near(b, SEATED.image(at(&root, "p.b"))), "{b:?}");
    assert!(near(at(&e, "X"), [-b[1], b[0], b[2]]), "{:?} against {b:?}", at(&e, "X"));
    let arm = crate::common::ent(&e, "arm").i() as u32;
    assert_eq!(e.sketch.carriers(crate::common::ent(&e, "p.b")).unwrap(), vec![arm]);
}

#[test]
fn an_instances_frame_starts_where_the_frame_around_its_call_stands() {
    // read by a seed alone, so where it starts is where it stands
    let src = format!("{READS}in (g := frame) {{\n  p := Post(len: 40mm)\n}}\n{}\
                       q := point hint(at: p.std.origin)\n", Pose::MOVED.place("g"));
    let e = build(&src);
    assert!(e.ok(), "{:?}", e.diags);
    let f = e.sketch.basis(crate::common::ent(&e, "p#frame").i());
    assert!(near(f.u, Pose::MOVED.u) && near(f.v, Pose::MOVED.v) && near(f.o, Pose::MOVED.o),
            "{f:?}");
    assert!(near(at(&e, "q"), Pose::MOVED.o), "{:?}", at(&e, "q"));
}

#[test]
fn an_instance_nested_in_a_body_is_read_as_a_frame_by_its_path() {
    // the root reads a datum of an instance inside another, by the path through it: placing what
    // `Pair` makes from outside it, which `Pair` is refused for leaving free (#562, E108) — and
    // which places it all the same
    let src = format!("{READS}component Pair() {{\n  one := Post(len: 40mm)\n}}\n\
                       pair := Pair()\n{}",
                      mated("pair.one.std.x", "pair.one.std.y", "pair.one.std.origin"));
    let mut e = build(&src);
    let codes: Vec<&str> = e.errors().map(|d| d.code.as_str()).collect();
    assert_eq!(codes, ["E108"], "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    let want = SEATED.image(at(&root, "p.b"));
    assert!(near(at(&e, "pair.one.b"), want), "{:?} against {want:?}", at(&e, "pair.one.b"));
}

#[test]
fn an_instance_whose_body_reads_no_datum_is_refused_its_frame() {
    // placed by what it is given: a mate on its frame would place nothing
    let src = format!("{HANDED}p := Post(std.front, std.origin, len: 40mm)\nq := point\n\
                       p.std.origin coincident q\n");
    let e = build(&src);
    let at = src.find("p.std.origin coincident").unwrap();
    let d = e.diags.iter().find(|d| d.code == Code::E111 && d.span.lo as usize == at)
        .unwrap_or_else(|| panic!("not refused: {:?}", e.diags));
    assert!(d.message.contains("`Post`'s body reads no standard datum"), "{}", d.message);
    // and a member that is no datum is no frame's, refused as it was
    let e = build(&format!("{READS}p := Post(len: 40mm)\nq := point\np.std.zork coincident q\n"));
    assert!(!e.ok() && e.diags.iter().all(|d| d.code != Code::E111), "{:?}", e.diags);
}

/// A mate said by a relation word the document defines, over the instance as its operand (#595).
const SEATS: &str = "\
g seats_at(up, east) p := {
  g.std.origin coincident p
  g.std.x parallel east
  g.std.y parallel up
}
";

/// That `e`'s `p`, a `Post(len: 40mm)`, stands where `pose` takes the root's.
fn post_at(e: &Elaborated, pose: &Pose) {
    let root = solved(&format!("{READS}p := Post(len: 40mm)\n"));
    for name in ["p.a", "p.b"] {
        let (got, want) = (at(e, name), pose.image(at(&root, name)));
        assert!(near(got, want), "{name}: {got:?} against {want:?}");
    }
}

#[test]
fn a_relation_word_over_an_instance_reads_its_frame_and_places_it() {
    // the mate's three statements said once: the instance stands in a frame of its own though no
    // statement of the document writes `p.std`
    let src = format!("{READS}{SEATS}{SEAT}p := Post(len: 40mm)\n\
                       p seats_at(up: std.z, east: bore) seat\n");
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    post_at(&e, &SEATED);
    // and a word whose body uses that word, its operand passed on, reads the frame through it;
    // the culprit reads as the word written
    let src = format!("{READS}{SEATS}{SEAT}\
                       g upright(up, east) p := g seats_at(up: up, east: east) p\n\
                       g upright_twice(up, east) p := g upright(up: up, east: east) p\n\
                       p := Post(len: 40mm)\n\
                       p upright_twice(up: std.z, east: bore) seat\n");
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    post_at(&e, &SEATED);
    let said = crate::common::said(&e);
    assert!(said.iter().any(|s| s == "p upright_twice(up: std.z, east: bore) seat"), "{said:?}");
}

#[test]
fn a_relation_word_given_an_instance_as_a_parameter_reads_its_frame() {
    // the instance named in the parentheses rather than as an operand
    let src = format!("{READS}{SEAT}x seat_of(part, up) p := {{\n  part.std.origin coincident p\n  \
                       part.std.x parallel x\n  part.std.y parallel up\n}}\n\
                       p := Post(len: 40mm)\n\
                       bore seat_of(part: p, up: std.z) seat\n");
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    post_at(&e, &SEATED);
}

#[test]
fn std_seats_a_part_on_a_station_raised_from_a_point_in_one_line_each() {
    // the radiator's placement (#595): a station 25 up from a point drawn in the top, and the
    // post seated on it, its x turned 30° from the world's about z
    let src = format!("{READS}\
in std.top {{
  base := point
  fix((10, 20)) base
}}
st := point
st raised(d: 25mm, along: std.z) base
p := Post(len: 40mm)
p seats(turn: 30deg, up: std.z, east: std.x) st
").replace("use std\n", "use std\nuse std.place (seats, raised)\n");
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    assert!(near(at(&e, "st"), [10.0, 20.0, 25.0]), "{:?}", at(&e, "st"));
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    post_at(&e, &Pose { u: [c, s, 0.0], v: [-s, c, 0.0], o: [10.0, 20.0, 25.0] });
    // a station below, along an axis of the drawing's own: the axis's way, not the seed's
    let src = format!("{READS}\
in std.top {{
  base := point
  fix((10, 20)) base
}}
tilt := axis
fix(dir == (0.6, 0, 0.8), origin == (0, 0, 0)) tilt
st := point hint((0, 0, 0))
st raised(d: -10mm, along: tilt) base
").replace("use std\n", "use std\nuse std.place (raised)\n");
    let mut e = solved(&src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    assert!(near(at(&e, "st"), [4.0, 20.0, -8.0]), "{:?}", at(&e, "st"));
}

/// A disc of radius 1 about (3, 1) in its part's top, and a station 75 above a point drawn in the
/// world's (#629).
const DISC: &str = "\
unit mm
use std
use std.place (seats, raised, coords)
component Part() {
  in std.top {
    c := point hint((3, 1))
    c coords(du: 3mm, dv: 1mm) std.top
    rim := radius(1mm) circle(center: c)
  }
  b := solid(face(rim), from: 1mm, to: 2mm)
}
in std.top {
  c := point hint((-100, 20))
  c coords(du: -100mm, dv: 20mm) std.top
}
at := point
at raised(d: 75mm, along: std.z) c
";

#[test]
fn a_part_seated_a_half_turn_round_on_a_raised_station_stands_upright() {
    // near a half turn the frame's normal could come out along `up` with its `v` left where it
    // was seeded: a mirrored frame, every row holding, and the part upside down (#629)
    for turn in [176.0, 179.0, 180.0, -179.0, 90.0, 0.0] {
        let src = format!("{DISC}p := Part()\n\
                           p seats(turn: {turn}deg, up: std.z, east: std.x) at\n");
        let mut e = solved(&src);
        assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
        let t = f64::to_radians(turn);
        let pose = Pose { u: [t.cos(), t.sin(), 0.0], v: [-t.sin(), t.cos(), 0.0],
                          o: [-100.0, 20.0, 75.0] };
        let (got, want) = (at(&e, "p.c"), pose.image([3.0, 1.0, 0.0]));
        assert!(near(got, want), "at {turn}°: {got:?} against {want:?}");
        // and the frame is right handed, so the solid placed by it stands up off the seat
        let f = crate::common::ent(&e, "p#frame").i();
        let pl = &e.sketch.frames[f];
        let d = |r: u32| crate::common::dir(&e.sketch, r as usize);
        let up = gcs_core::space::cross(d(pl.u), d(pl.v));
        assert!(near(up, [0.0, 0.0, 1.0]), "at {turn}°: u × v is {up:?}");
    }
}

#[test]
fn a_relation_word_over_an_instance_whose_body_reads_no_datum_is_refused_at_the_use() {
    let src = format!("{HANDED}{SEATS}p := Post(std.front, std.origin, len: 40mm)\nq := point\n\
                       p seats_at(up: std.z, east: std.x) q\n");
    let e = build(&src);
    let at = src.rfind("seats_at(up").unwrap();
    let d = e.diags.iter().find(|d| d.code == Code::E111)
        .unwrap_or_else(|| panic!("not refused: {:?}", e.diags));
    assert!(d.message.contains("`Post`'s body reads no standard datum"), "{}", d.message);
    assert_eq!(d.span.lo as usize, at, "{d:?}");
}

/// A peg whose cone about its spindle passes through a point drawn in its front (#624).
const PEG: &str = "\
unit mm
use std
use std.place (vertical, coords, raised, seats)
use std.motion (spins)
use std
use std.sets
component Peg() {
  in std.front {
    spindle := vertical line(std.origin, hint((0, 10)))
    length(10mm) spindle
    q := point hint((5, 3))
    q coords(du: 5mm, dv: 3mm) std.front
  }
  cone := std.sets.ConeThrough(spindle, q, lean: -45deg)
}
";

#[test]
fn a_seated_parts_cone_reads_a_point_drawn_in_its_front() {
    // the cone's lines stand in the part's own frame and read `q` where its front draws it
    // (#624): the seated part is the unseated one carried whole
    let seat = "at := point\nat raised(d: 20mm, along: std.x) std.origin\n";
    let mut e = solved(&format!("{PEG}placed := Peg()\n{seat}\
                                 placed seats(turn: 30deg, up: std.z, east: std.x) at\n"));
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    let root = solved(&format!("{PEG}placed := Peg()\n"));
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let pose = Pose { u: [c, s, 0.0], v: [-s, c, 0.0], o: [20.0, 0.0, 0.0] };
    for name in ["placed.q", "placed.cone.apex", "placed.cone.foot"] {
        let (got, want) = (at(&e, name), pose.image(at(&root, name)));
        assert!(near(got, want), "{name}: {got:?} against {want:?}");
    }
    // and riding a frame with a joint, the line through `q` rides it as the point does
    let e = solved(&format!("{PEG}in std.top {{\n  o := point\n  fix((0, 0)) o\n}}\n\
                             in (arm := frame) {{\n  placed := Peg()\n}}\n{}arm spins(1) o\n{seat}\
                             placed seats(turn: 0deg, up: std.z, east: std.x) at\n",
                            Pose::WORLD.place("arm")));
    let arm = crate::common::ent(&e, "arm").i() as u32;
    for name in ["placed.q", "placed.cone.spine"] {
        let carried = e.sketch.carriers(crate::common::ent(&e, name)).unwrap();
        assert_eq!(carried, vec![arm], "{name}");
    }
}
