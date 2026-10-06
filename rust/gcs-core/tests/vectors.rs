//! **A vector is written `(a, b)` or `(a, b, c)`**, and an entity's numbers are its members: a
//! point *is* its place (`fix((0, 0)) p`, `hint((3, 4))`), an axis has a direction and an origin
//! (`dir`, `origin`), a plane an origin.  A member of one is `x`, `dir.x`, `origin.z`; a vector
//! held whole is as long as the vector it holds, so `(0, 0)` cannot leave a point in space's
//! height free unsaid.  A named vector (`o := (10mm, 20mm)`) is a group of its components.

use crate::common::{ent, read, refused};
use gcs_core::edit;
use gcs_core::model::EntRef;
use gcs_core::program::Elaborated;
use gcs_core::syntax::{highlight, Tint};

/// Every number of `e`'s own, `(value, held)`, in `members` order.
fn own(e: &Elaborated, r: EntRef) -> Vec<(f64, bool)> {
    let sk = &e.sketch;
    sk.own_params(r).iter().map(|&p| (sk.params[p as usize].value, sk.params[p as usize].fixed)).collect()
}

fn reconciled(e: &mut Elaborated) -> edit::Edit {
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(e, &sk);
    e.sketch = sk;
    out
}

#[test]
fn a_point_is_held_whole_or_by_member() {
    let e = read("use std\nin std.front {\na := point\nfix((3, 4)) a\nb := point hint((5, 6))\nfix(x == 7) b\n}\n");
    assert_eq!(own(&e, ent(&e, "a")), [(3.0, true), (4.0, true)]);
    assert_eq!(own(&e, ent(&e, "b")), [(7.0, true), (6.0, false)], "y left to the solve, seeded");
    let e = read("use std\nc := point\nfix((1, 2, 3)) c\n");
    assert_eq!(own(&e, ent(&e, "c")), [(1.0, true), (2.0, true), (3.0, true)]);
}

#[test]
fn a_vector_held_whole_is_as_long_as_what_it_holds() {
    refused("use std\nc := point\nfix((1, 2)) c\n", "E105", "a point in space has three", "(1, 2)");
    refused("use std\nin std.front {\nc := point\nfix((1, 2, 3)) c\n}\n",
            "E105", "a point in a plane has two", "(1, 2, 3)");
    refused("use std\nt := axis\nfix(dir == (1, 0)) t\n", "E105", "`dir` has three", "dir == (1, 0)");
    refused("use std\nin std.front {\nc := point\nfix(dir == (1, 0, 0)) c\n}\n",
            "E105", "a point has x and y, not `dir`", "dir == (1, 0, 0)");
    refused("use std\nin std.front {\no := point\nk := circle(center: o)\nfix((1, 2)) k\n}\n",
            "E105", "a circle is no vector: it has r", "(1, 2)");
}

#[test]
fn an_axis_has_a_direction_and_an_origin() {
    let e = read("use std\np := point\nfix((5, 0, 0)) p\nt := axis\n\
                  fix(dir == (0, 0, 1), origin == (5, 0, 0)) t\np coincident t\n");
    assert_eq!(own(&e, ent(&e, "t")),
               [(0.0, true), (0.0, true), (1.0, true), (5.0, true), (0.0, true), (0.0, true)]);
    // one member of one, and a seed by its member key
    let e = read("use std\nt := axis hint(dir.y: 1)\nfix(dir.x == 0) t\n");
    let t = own(&e, ent(&e, "t"));
    assert_eq!((t[0], t[1].0), ((0.0, true), 1.0));
    refused("use std\nt := axis\nfix(x == 1) t\n", "E105", "an axis has dir and origin, not `x`",
            "fix(x == 1) t");
}

#[test]
fn a_plane_stands_at_its_origin() {
    let e = read("use std\nback := plane hint(origin: (0, 7, 0))\nfix(origin == (0, 12, 0)) back\n");
    assert_eq!(own(&e, ent(&e, "back")), [(0.0, true), (12.0, true), (0.0, true)]);
    let e = read("use std\np := plane(u: hint(dir: (0, 1, 0)), v: std.z)\n");
    let u = ent(&e, "p.u");
    assert_eq!(own(&e, u)[..3], [(0.0, false), (1.0, false), (0.0, false)]);
}

#[test]
fn a_seed_is_a_vector_or_its_members() {
    let e = read("use std\nin std.front {\na := point hint((3, 4))\nb := point hint(y: 7)\n\
                  l := line(hint((1, 2)), b)\n}\nc := point hint((1, 2, 3))\n");
    assert_eq!(own(&e, ent(&e, "a")), [(3.0, false), (4.0, false)]);
    assert_eq!(own(&e, ent(&e, "b"))[1], (7.0, false));
    assert_eq!(own(&e, ent(&e, "l.p1")), [(1.0, false), (2.0, false)]);
    assert_eq!(own(&e, ent(&e, "c")), [(1.0, false), (2.0, false), (3.0, false)]);
    // a grouped expression is no vector
    let e = read("use std\nw := (2 + 3) * 2\nin std.front {\na := point hint(((w + 1), w))\n}\n");
    assert_eq!(own(&e, ent(&e, "a")), [(11.0, false), (10.0, false)]);
}

/// What a gesture holds is written as the vector it is where all of it is held, and member by
/// member where only some is — a point in space held across the page keeps its height free.
#[test]
fn a_hold_is_written_as_a_vector_where_all_of_it_is_held() {
    let mut e = read("use std\nin std.front {\na := point hint((3, 4))\n}\n");
    let a = ent(&e, "a").i();
    e.sketch.fix_point(a, true);
    let out = reconciled(&mut e);
    assert!(out.text.contains("fix((3, 4)) a\n"), "{}", out.text);
    let mut e = read("use std\nc := point hint((1, 2, 3))\n");
    let c = ent(&e, "c");
    for &p in &e.sketch.own_params(c)[..2] {
        e.sketch.params[p as usize].fixed = true;
    }
    let out = reconciled(&mut e);
    assert!(out.text.contains("fix(x == 1, y == 2) c\n"), "{}", out.text);
    read(&out.text);
}

#[test]
fn a_named_vector_is_read_by_its_members() {
    let e = read("unit mm\nuse std\no := (10mm, 20mm)\nin std.front {\np := point\n\
                  fix((o.x, o.y)) p\n}\n");
    assert_eq!(own(&e, ent(&e, "p")), [(10.0, true), (20.0, true)]);
    let (prog, errs, _) = gcs_core::library::parse_linked(
        "unit mm\nuse std\no := (10mm, 20mm)\nin std.front {\np := point\nfix((o.z, 0)) p\n}\n");
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("`o` has `x` and `y`, not `z`")),
        "{:?}", e.diags);
}

/// A number inside a vector is what the clause makes it: a seed in a `hint(…)`, and in a `fix`
/// the number a pin holds, as `t == 4`'s is.
#[test]
fn a_vector_is_coloured_by_its_clause() {
    let src = "a := point hint((31, 4))\nfix((57, 6)) a\n";
    let tint = |what: &str| {
        let at = src.find(what).unwrap();
        highlight(src).into_iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| t)
    };
    assert_eq!(tint("31"), Some(Tint::Seed));
    assert_eq!(tint("57"), Some(Tint::Num));
}
