//! **The model outline** (#435, `overview::outline`): every instance the document makes and its
//! members, as a tree in source order — and what hiding a row takes out of the workspace: not
//! drawn, not picked and not framed, while it still solves.
use gcs_core::model::{EntKind, EntRef};
use gcs_core::overview::outline::{outline, Node, NodeKind};
use gcs_core::overview::workspace::{
    bounds, bounds_of, overlapping, pick, pick_solid, under, Projection, Under,
};
use gcs_core::overview::{scene3d_of, Layer, Part};
use std::f64::consts::FRAC_PI_2;
use crate::common::gestures::build;

/// A component instance, a ring, an object made of two features and `use std`.
const DOC: &str = "unit mm\nuse std\n\
component Pin(c: point) {\n  q := point hint((3, 0))\n  c distance(3) q\n  arm := line(c, q)\n}\n\
in std.front {\n  hub := point\n  fix((0, 0)) hub\n\
  ring 3 about hub {\n    tip := point hint((10, 1))\n    hub distance(10) tip\n\
    chord := line(tip, next.tip)\n  }\n\
  pin := Pin(hub)\n  o := point hint((40, 0))\n  fix((40, 0)) o\n\
  rim := circle(center: o) hint(r: 20)\n  h := circle(center: o) hint(r: 5)\n  line(o, hub)\n\
  seg := line(hint((0, -30)), hint((10, -30)))\n}\n\
stock := solid(face(rim), depth: 8)\nbore := solid(face(h), depth: 8)\nbore cut stock\n";

fn labels(rows: &[Node]) -> Vec<&str> {
    rows.iter().map(|n| n.label.as_str()).collect()
}

fn paths(rows: &[Node]) -> Vec<&str> {
    rows.iter().map(|n| n.path.as_str()).collect()
}

/// The root body's statements in source order, an instance with its members, a ring with a row
/// per copy, a line's ends under it, an object with its features under it, `std`'s datums last.
#[test]
fn the_outline_is_the_documents_structure() {
    let e = build(DOC);
    let rows = outline(&e);
    assert_eq!(labels(&rows), ["hub", "{tip, chord}[0‥2]", "pin : Pin", "o", "rim", "h", "line",
                               "seg", "stock", "std"]);
    let text = |n: &Node| n.span.map(|s| &e.text()[s.lo as usize..s.hi as usize]);
    assert_eq!(text(&rows[0]), Some("hub := point"));

    // a ring: one child per copy, carried by `public_path`
    let ring = &rows[1];
    assert_eq!(ring.kind, NodeKind::Block);
    assert_eq!(labels(&ring.children), ["{tip, chord}[0]", "{tip, chord}[1]", "{tip, chord}[2]"]);
    assert!(ring.children.iter().all(|c| c.kind == NodeKind::Copy));
    assert_eq!(paths(&ring.children[2].children), ["tip[2]", "chord[2]"]);
    assert!(text(ring).is_some_and(|t| t.starts_with("ring 3 about hub")));

    // an instance, its members under it by their names in it
    let pin = &rows[2];
    assert_eq!((pin.kind, pin.component.as_deref(), pin.path.as_str()),
               (NodeKind::Instance, Some("Pin"), "pin"));
    assert_eq!(text(pin), Some("pin := Pin(hub)"));
    assert_eq!(labels(&pin.children), ["q", "arm"]);
    assert_eq!(paths(&pin.children), ["pin.q", "pin.arm"]);
    // the member's span is its statement in the definition, which the document wrote
    assert_eq!(text(&pin.children[1]), Some("arm := line(c, q)"));

    // an anonymous row is listed, by its kind, and carried by nothing
    assert_eq!((rows[6].label.as_str(), rows[6].path.as_str()), ("line", ""));
    // a line's implicit ends nest under it
    assert_eq!(paths(&rows[7].children), ["seg.p1", "seg.p2"]);

    // the object, its face, its stock and its cutter under it — the cutter taken from where it
    // was written, listed for navigation only
    let stock = &rows[8];
    assert_eq!((stock.kind, stock.solid, stock.operand), (NodeKind::Solid, Some(0), false));
    assert_eq!(labels(&stock.children), ["face in stock", "solid in stock", "bore"]);
    assert!(!stock.children[0].operand && stock.children[1].operand && stock.children[2].operand);
    assert_eq!(stock.children[2].solid, e.map.ent_named("bore").map(|r| r.i()));
    assert_eq!(text(&stock.children[2]), Some("bore := solid(face(h), depth: 8)"));
    assert!(rows.iter().all(|n| n.path != "bore"), "the cutter is not listed twice");

    // std's datums: one group, last, with no span in the document
    let std = &rows[9];
    assert_eq!((std.kind, std.path.as_str(), std.span), (NodeKind::Datums, "std", None));
    let front = std.find("std.front").expect("the front plane");
    assert_eq!(front.label, "front");
    assert_eq!(paths(&front.children), ["std.front.origin"]);
    assert!(std.children.iter().all(|c| c.span.is_none()));
}

/// The entities and solids a row and the rows under it stand for.
fn part(n: &Node) -> (Vec<EntRef>, Vec<usize>) {
    let (mut ents, mut solids) = (Vec::new(), Vec::new());
    n.subtree(&mut ents, &mut solids);
    (ents, solids)
}

/// **Hidden means not drawn, not lit and not picked**: what a hidden row made is passed by
/// every question the workspace asks, and a point declared at the root that a hidden instance's
/// line reads stays.  It still solves.
#[test]
fn what_is_hidden_is_not_picked_drawn_or_framed() {
    let mut e = build(DOC);
    let rows = outline(&e);
    let row = |p: &str| rows.iter().find_map(|n| n.find(p)).unwrap_or_else(|| panic!("{p}"));
    let ent = |n: &str| e.map.ent_named(n).unwrap();
    let (q, arm, seg, hub) = (ent("pin.q"), ent("pin.arm"), ent("seg"), ent("hub"));
    let stock = ent("stock").i();
    let (az, el) = (-FRAC_PI_2, 0.0);
    let (tol, unit) = (0.5, 0.05);
    let at_q = {
        let proj = Projection::new(&e.sketch, az, el);
        proj.point(&e.sketch, q.i())
    };
    let disc = (50.0, 0.0);

    let before = |sk: &gcs_core::model::Sketch| {
        let proj = Projection::new(sk, az, el);
        (pick(sk, &proj, at_q, tol, unit), pick_solid(sk, &proj, disc).map(|h| h.solid),
         under(sk, &proj, disc, tol, unit),
         overlapping(sk, &proj, (at_q.0 - 0.2, at_q.1 - 0.2), (at_q.0 + 0.2, at_q.1 + 0.2), unit),
         bounds(sk, &proj, unit).unwrap())
    };
    let (picked, solid, beneath, band, b) = before(&e.sketch);
    assert_eq!(picked, Some(q));
    assert_eq!(solid, Some(stock));
    assert!(beneath.contains(&Under::Object(stock)));
    assert!(band.contains(&q) && band.contains(&arm));
    assert!(b.1 <= -30.0 + 1e-9, "{b:?}");

    // the instance, the line at the bottom and the object
    let (mut ents, mut solids) = part(row("pin"));
    for p in ["seg", "stock"] {
        let (e2, s2) = part(row(p));
        ents.extend(e2);
        solids.extend(s2);
    }
    assert!(ents.contains(&q) && ents.contains(&arm) && ents.contains(&seg));
    assert!(!ents.contains(&hub), "what the instance reads is not what it made");
    e.sketch.set_hidden(ents, solids);

    let (picked, solid, beneath, band, b) = before(&e.sketch);
    assert_ne!(picked, Some(q));
    assert_ne!(picked, Some(arm));
    assert_eq!(solid, None, "an object hidden is not picked");
    assert!(!beneath.iter()
        .any(|u| matches!(u, Under::Object(_) | Under::Face(_) | Under::Edge { .. })));
    assert!(!band.contains(&q) && !band.contains(&arm), "{band:?}");
    assert!(b.1 > -30.0 + 1e-6, "the hidden line is not framed: {b:?}");
    // the point declared at the root still is
    let proj = Projection::new(&e.sketch, az, el);
    assert_eq!(pick(&e.sketch, &proj, (0.0, 0.0), tol, unit), Some(hub));

    // and the scene the box draws has none of it
    let scene = scene3d_of(&e.sketch, 0.0, Layer::All);
    for gone in [arm, seg] {
        assert!(!scene.iter().any(|it| it.of == Some(gone)), "{gone:?}");
    }
    assert!(!scene.iter().any(|it| it.what == Part::Solid), "the hidden object's creases");
    assert!(scene.iter().any(|it| it.of.is_some_and(|r| r.kind == EntKind::Line)),
            "the rest is still drawn");

    // a plane hidden takes its pane with it
    let front = ent("std.front");
    let panes = |sk: &gcs_core::model::Sketch| scene3d_of(sk, 0.0, Layer::Datums).iter()
        .filter(|it| it.what == Part::Face && it.of == Some(front)).count();
    assert!(panes(&e.sketch) > 0);
    e.sketch.set_hidden([front], []);
    assert_eq!(panes(&e.sketch), 0);
    // set whole: what was hidden before is shown again
    let proj = Projection::new(&e.sketch, az, el);
    assert_eq!(pick(&e.sketch, &proj, at_q, tol, unit), Some(q));

    // hidden geometry still solves and holds its constraints
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
}

/// Framing a row frames its subtree and nothing else: the instance's box is round the pin, the
/// object's round the disc — hidden or not, since a row is framed to be found.
#[test]
fn a_row_is_framed_alone() {
    let mut e = build(DOC);
    let rows = outline(&e);
    let row = |p: &str| rows.iter().find_map(|n| n.find(p)).unwrap_or_else(|| panic!("{p}"));
    let (az, el) = (-FRAC_PI_2, 0.0);
    let (ents, solids) = part(row("pin"));
    let proj = Projection::new(&e.sketch, az, el);
    let b = bounds_of(&e.sketch, &proj, &ents, &solids, 0.05).expect("the pin");
    assert!(b.0 >= -3.0 - 1e-6 && b.2 <= 3.0 + 1e-6 && b.1 >= -3.0 - 1e-6 && b.3 <= 3.0 + 1e-6,
            "{b:?}");
    let all = bounds(&e.sketch, &proj, 0.05).unwrap();
    assert!(all.2 - all.0 > b.2 - b.0);

    let (ents, solids) = part(row("stock"));
    e.sketch.set_hidden(ents.clone(), solids.clone());
    let proj = Projection::new(&e.sketch, az, el);
    let b = bounds_of(&e.sketch, &proj, &ents, &solids, 0.05).expect("the object, hidden");
    for (got, want) in [(b.0, 20.0), (b.2, 60.0), (b.1, -20.0), (b.3, 20.0)] {
        assert!((got - want).abs() < 0.5, "{b:?}");
    }
    assert_eq!(bounds_of(&e.sketch, &proj, &[], &[], 0.05), None);
}
