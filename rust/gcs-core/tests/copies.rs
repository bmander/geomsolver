//! Indexing a block's copies from outside: `p[1]` is copy 1 of a declaration, and `cyl[0].small`
//! is a point of copy 0's *instance* `cyl` — the copy's absolute prefix, and the rest of the path
//! read under it, greedily, as every other reference is.

use gcs_core::program::elaborate;
use gcs_core::syntax::parse;

#[test]
fn an_instance_inside_a_copy_is_indexed_like_a_declaration() {
    let src = "\
component Rung(a: point) {
  point b hint(x: a.x + 10, y: a.y)
  line l(a, b)
  a distance(10) b
  horizontal l
}
repeat 3 as i {
  point p hint(x: 0, y: i * 20)
  r: Rung(p)
}
ground p[0]
p[1] distance(5) r[2].b
r[0].b distance(20, along: y) r[1].b
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.lines.len(), 3);
    // `r[2].b` reached copy 2's rung's port, so the distance names two distinct points
    let c = e.sketch.constraints.iter().find(|c| c.kind == gcs_core::constraints::CKind::Distance && (c.args[2].num() - 5.0).abs() < 1e-12).expect("the distance");
    assert_ne!(c.args[0].ent(), c.args[1].ent());
}
