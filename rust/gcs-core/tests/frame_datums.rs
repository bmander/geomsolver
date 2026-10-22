//! A definition's standard datums are those of the frame it stands in (#524): `std.front` in a
//! body names the datum of the frame its instance stands in, the world's at the root, and a body
//! standing in a frame other than the world is refused rather than given the world's.
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

/// Two places alike to 1e-8.
fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| crate::common::near(a[k], b[k]))
}

/// The E101 said at the first place `what` is written.
fn refused<'a>(e: &'a Elaborated, src: &str, what: &str) -> &'a gcs_core::program::Diag {
    let at = src.find(what).unwrap_or_else(|| panic!("`{what}` is not in the text"));
    e.diags.iter().find(|d| d.code == Code::E101 && d.span.lo as usize == at)
        .unwrap_or_else(|| panic!("`{what}` is not refused: {:?}", e.diags))
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
fn a_body_standing_in_a_frame_is_refused_its_datums() {
    // a frame around the call, as a block or as the call's own `in`, and around a call further out
    for (src, frame) in [
        (format!("{READS}in (f := frame) {{\n  p := Post(len: 40mm)\n}}\n"), "`f`"),
        (format!("{READS}f := frame\np := Post(len: 40mm) in f\n"), "`f`"),
        (format!("{READS}component Arm() {{\n  p := Post(len: 40mm)\n}}\n\
                  in (arm := frame) {{\n  a := Arm()\n}}\n"), "`arm`"),
        (format!("{READS}component Arm() {{\n  in (g := frame) {{\n    p := Post(len: 40mm)\n  }}\n}}\n\
                  a := Arm()\n"), "`g`"),
        // a frame read at an instant is a frame
        (format!("{READS}f := frame\nf turns(1) std.z\nn := f at 30deg\n\
                  p := Post(len: 40mm) in n\n"), "`n`"),
    ] {
        let e = build(&src);
        let d = refused(&e, &src, "std.front");
        for word in ["`std.front`", frame, "frame its instance stands in", "not drawn yet"] {
            assert!(d.message.contains(word), "{word}: {}", d.message);
        }
        // every standard datum it reads, and nothing given the world's in its place
        refused(&e, &src, "std.origin");
    }
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
  b coincident std.origin
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
