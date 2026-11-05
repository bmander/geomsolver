//! **Sets in the notation of sets** (#783, [0.205]): `p in S`, `p not in S`, `boundary(S)`, `or`
//! in a set's body, `some t …: p in X at(…) t`, a distance from what is not in a set and `x at f`,
//! each lowered by the parser to the form it has always read (`syntax::notation`) and marked as
//! written so (`Written::lowered`); what needs the operand's kind read once it is known; the
//! spellings it retires refused naming their replacements; and the notation printed back as it
//! was read.
use gcs_core::library::parse_linked;
use gcs_core::program::elaborate;
use gcs_core::syntax::{parse, write_stmt_to, Lowered, OpArg, Seg, StmtKind, Written};

use crate::common::{refused, text::solved};

/// Two points `c` (the origin) and `e` (30 along `u`) in the front plane, and the line between.
const HEAD: &str = "unit mm\nuse std\nuse std.sets\nuse std.solids\nuse std.motion (turns)\n\
    in std.front {\n  c := point\n  fix((0, 0)) c\n  e := point\n  fix((30, 0)) e\n\
    ax := line(c, e)\n}\n";

/// The relations a text's statements are written as — those in sets' bodies too — in order.
fn written(src: &str) -> Vec<Written> {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    fn walk(stmts: &[gcs_core::syntax::Stmt], out: &mut Vec<Written>) {
        for st in stmts {
            match &st.kind {
                StmtKind::Relation(r) => out.extend(r.form.written().cloned()),
                StmtKind::Set(s) => walk(&s.lit.body, out),
                _ => {}
            }
        }
    }
    let mut out = Vec::new();
    walk(&prog.root().body, &mut out);
    out
}

/// The one relation `statement` is, written after `HEAD`.
fn one(statement: &str) -> Written {
    let all = written(&format!("{HEAD}{statement}\n"));
    all.into_iter().last().unwrap_or_else(|| panic!("no relation in `{statement}`"))
}

/// What the parser says of `src`, every message.
fn syntax(src: &str) -> Vec<String> {
    parse(src).1.into_iter().map(|e| e.message).collect()
}

/// `src` is refused by the parser with a message holding `needle`.
fn retired(statement: &str, needle: &str) {
    let said = syntax(&format!("{HEAD}{statement}\n"));
    assert!(said.iter().any(|m| m.contains(needle)), "`{statement}`: {said:?}");
}

#[test]
fn membership_is_lowered_to_the_side_words_and_marked() {
    let w = one("q := point\nq in { p | p distance(<= 5mm) c }");
    assert_eq!((w.word.text.as_str(), w.lowered), ("inside", Lowered::Set));
    let w = one("q := point\nq not in std.sets.Ball(c, r: 5mm)");
    assert_eq!((w.word.text.as_str(), w.lowered), ("outside", Lowered::Set));
    // the boundary: the set's surface, a point on it
    let w = one("ball := std.sets.Ball(c, r: 5mm)\nq := point\nq in boundary(ball)");
    assert_eq!(w.word.text, "coincident");
    assert!(matches!(w.ops[1].path.last(), Some(Seg::Field(f)) if f.text == "surface"), "{:?}", w.ops[1]);
}

#[test]
fn a_face_is_the_face_and_its_side_its_materials() {
    let body = "component Side(a: face) := { p | p in a.side }\n\
        component Patch(a: face) := { q | q in a }\n";
    let (prog, errs) = parse(&format!("{HEAD}{body}"));
    assert!(errs.is_empty(), "{errs:?}");
    let all: Vec<Written> = prog.components.iter().filter_map(|c| c.set.as_ref())
        .flat_map(|lit| lit.body.iter())
        .filter_map(|st| match &st.kind { StmtKind::Relation(r) => r.form.written().cloned(), _ => None })
        .collect();
    assert_eq!(all.len(), 2, "{all:?}");
    let side = &all[0];
    assert_eq!((side.word.text.as_str(), side.lowered), ("inside", Lowered::Side));
    assert_eq!(side.ops[1].root.text, "a");
    assert!(side.ops[1].path.is_empty(), "`.side` is taken off: {:?}", side.ops[1]);
    assert_eq!(all[1].word.text, "coincident");
}

#[test]
fn a_quantifier_is_the_sweep_it_says() {
    // over an interval written in place, under a motion
    let w = one("f := frame\nf turns(1) ax\nb := { p | p distance(<= 2mm) e }\n\
        s := { p | some t in (0deg, 90deg): p in b at(under: f) t }");
    assert_eq!(w.word.text, "inside");
    let over = w.args.iter().find_map(|a| match a {
        OpArg::Over { key, ends } => Some((key.text.clone(), ends.as_ref().map(|e| (e[0].0.clone(), e[1].0.clone())))),
        _ => None,
    });
    assert_eq!(over, Some(("t".into(), Some(("0deg".into(), "90deg".into())))));
    assert!(w.args.iter().any(|a| matches!(a, OpArg::Named(n, _) if n.text == "under")));
    // the whole roll: no interval, no declared parameter, so no `over`
    let w = one("f := frame\nf turns(1) ax\nb := { p | p distance(<= 2mm) e }\n\
        s := { p | some t: p in b at(under: f) t }");
    assert!(!w.args.iter().any(|a| matches!(a, OpArg::Over { .. })), "{:?}", w.args);
    // a declared parameter's interval
    let w = one("param t: Angle over (0deg, 30deg)\nf := frame\nf turns(1) ax\n\
        b := { p | p distance(<= 2mm) e }\ns := { p | some t: p in b at(under: f) t }");
    assert!(w.args.iter().any(|a| matches!(a, OpArg::Over { key, ends: None } if key.text == "t")));
    // the prism through a face, and what is not in it
    let face = "in std.front {\n  k := circle(center: c) hint(r: 3)\n  radius(3) k\n  sec := face(k)\n}\n";
    let w = one(&format!("{face}s := {{ p | some h: p in sec at(along: n) h }}"));
    assert_eq!((w.word.text.as_str(), w.args.is_empty()), ("inside", true));
    let w = one(&format!("{face}s := {{ p | not some h: p in sec at(along: n) h }}"));
    assert_eq!(w.word.text, "outside");
}

#[test]
fn a_distance_from_what_is_not_in_a_set_is_its_inside_side() {
    let w = one("heel := std.sets.Ball(c, r: 10mm)\nk := point\nk distance(3mm) { q | q not in heel }");
    assert_eq!((w.word.text.as_str(), w.sel("side").as_deref(), w.lowered.any()), ("distance", Some("inside"), true));
    assert_eq!(w.ops[1].root.text, "heel");
}

#[test]
fn the_retired_spellings_are_refused_naming_their_replacements() {
    retired("ball := std.sets.Ball(c, r: 5mm)\nq := point\nq inside ball", "`p in S`");
    retired("ball := std.sets.Ball(c, r: 5mm)\nq := point\nq outside ball", "`p not in S`");
    retired("ball := std.sets.Ball(c, r: 5mm)\nq := point\nq coincident ball.surface", "`boundary(S)`");
    retired("b := { p | p distance(<= 2mm) e }\nf := frame\ns := solid({ p | p coincident(over t) b })",
            "`some s: p in F at s`");
    retired("f := frame\nq := c in f", "`n := x at f`");
    retired("s := { p | some t in [0deg, 90deg]: p in std.sets.Ball(c, r: 1mm) at t }", "`(a, b)`");
    retired("s := { p | every t in (0deg, 90deg): p in std.sets.Ball(c, r: 1mm) }", "reserved");
    retired("s := { p | p distance(<= 1mm) c or p distance(<= 1mm) e }", "membership");
    // the claim word between two solids, one within another, is no membership and stays
    assert!(syntax(&format!("{HEAD}a := solid(std.sets.Ball(c, r: 2mm))\n\
        b := solid(std.sets.Ball(c, r: 3mm))\nclaim a inside b\n")).is_empty());
}

#[test]
fn what_the_operand_is_decides_the_rest() {
    let src = |s: &str| format!("{HEAD}{s}\n");
    // a set with no inside: `in` is on it, and nothing is outside it
    let e = solved(&src("ball := std.sets.Sphere(c, r: 10mm)\nk := point hint((5, 1, 0))\nk in ball"));
    let k = crate::common::at(&e, "k");
    assert!(((k[0] * k[0] + k[1] * k[1] + k[2] * k[2]).sqrt() - 10.0).abs() < 1e-6, "{k:?}");
    refused(&src("ball := std.sets.Sphere(c, r: 10mm)\nk := point hint((5, 1, 0))\nk not in ball"),
            "E040", "no inside", "not in");
    // the ordinary distance: outside a region, and refused from a set with no inside
    let e = solved(&src("ball := std.sets.Ball(c, r: 10mm)\nk := point hint((15, 1, 0))\nk distance(3mm) ball"));
    let k = crate::common::at(&e, "k");
    assert!(((k[0] * k[0] + k[1] * k[1] + k[2] * k[2]).sqrt() - 13.0).abs() < 1e-6, "{k:?}");
    refused(&src("ball := std.sets.Sphere(c, r: 10mm)\nk := point hint((15, 1, 0))\nk distance(3mm) ball"),
            "E040", "root either side", "distance(3mm)");
    // the retired spellings the flattener and the elaborator see
    refused(&src("ball := std.sets.Ball(c, r: 10mm)\nk := point hint((5, 1, 0))\nk coincident ball"),
            "E040", "`coincident` against a set is retired", "coincident");
    refused(&src("ball := std.sets.Ball(c, r: 10mm)\nk := point hint((15, 1, 0))\nk distance(3mm, side: outside) ball"),
            "E040", "`side: outside` is retired", "side");
    refused(&src("k := point hint((5, 1, 0))\nk in std.top"), "E040", "`p coincident P`", "k in std.top");
}

#[test]
fn a_union_of_solids_is_the_body_term() {
    let (prog, errs, _) = parse_linked(&format!("{HEAD}a := solid(std.sets.Ball(c, r: 10mm))\n\
        b := solid(std.sets.Ball(e, r: 10mm))\nboth := solid({{ p | p in a or p in b }})\n\
        body := solid(a)\nb union body\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let i = |n: &str| crate::common::ent(&e, n).i();
    let volume = |n: &str| gcs_core::brep::props::volume(&e.sketch.exact_solid(i(n)).unwrap().brep);
    assert_eq!(volume("both").to_bits(), volume("body").to_bits());
}

#[test]
fn the_notation_prints_as_it_was_read() {
    for statement in [
        "q in ball",
        "q not in ball",
        "q in boundary(ball)",
        "q distance(3mm) { r | r not in ball }",
    ] {
        let src = format!("{HEAD}ball := std.sets.Ball(c, r: 5mm)\nq := point\n{statement}\n");
        let (prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{errs:?}");
        let st = prog.root().body.last().unwrap();
        let mut out = String::new();
        write_stmt_to(&mut out, &st.kind).unwrap();
        // what is printed reads back as what was read
        let again = format!("{HEAD}ball := std.sets.Ball(c, r: 5mm)\nq := point\n{}\n", out.trim());
        let (back, errs) = parse(&again);
        assert!(errs.is_empty(), "`{out}`: {errs:?}");
        let mut twice = String::new();
        write_stmt_to(&mut twice, &back.root().body.last().unwrap().kind).unwrap();
        assert_eq!(out, twice, "`{statement}`");
        assert_eq!(out.trim(), statement);
    }
}
