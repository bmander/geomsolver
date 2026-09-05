use gcs_core::program::{elaborate, Code};
use gcs_core::syntax::{parse, parse_from, parse_with_limits, ParseLimits, StmtKind};

#[test]
fn parser_bounds_nesting_before_expansion() {
    let source = format!("{}point p\n{}point tail\n", "repeat 1 {\n".repeat(80), "}\n".repeat(80));
    let (p, errors) = parse(&source);
    assert!(errors.iter().any(|e| e.message.contains("nest more than")), "{errors:?}");
    assert!(p
        .root()
        .body
        .iter()
        .any(|s| matches!(&s.kind, StmtKind::Decl(d) if d.name.key().text == "tail")));
}

#[test]
fn swept_claim_bodies_share_limits_and_source_lookup() {
    let source = "claim over reach in (0, 1) { a clear(1) b }\npoint tail";
    let (p, errors) = parse_with_limits(source, ParseLimits { max_depth: 1, max_statements: 2 });
    assert!(errors.iter().any(|e| e.message.contains("more than 2 statements")), "{errors:?}");
    assert_eq!(p.stmts().count(), 2);
    let StmtKind::ClaimOver(c) = &p.root().body[0].kind else { panic!("expected a swept claim") };
    assert_eq!(p.stmt(c.body[0].id).unwrap().span.slice(source), "a clear(1) b");
    let (_, errors) = parse_with_limits(source, ParseLimits { max_depth: 0, max_statements: 10 });
    assert!(errors.iter().any(|e| e.message.contains("nest more than 0")), "{errors:?}");
}

#[test]
fn statement_budget_is_shared_by_components_blocks_and_the_root() {
    let limits = ParseLimits { max_depth: 8, max_statements: 4 };
    let source = "component Part() {\npoint p\n}\nrepeat 1 {\npoint q\n}\npoint extra\n";
    let (program, errors) = parse_with_limits(source, limits);
    assert_eq!(errors.iter().filter(|e| e.message.contains("more than 4 statements")).count(), 1);
    assert_eq!(program.stmts().count(), 3); // component header consumes the fourth slot
    assert!(!program
        .stmts()
        .any(|st| matches!(&st.kind, StmtKind::Decl(d) if d.name.key().text == "extra")));
}

#[test]
fn chain_generated_statements_share_the_budget() {
    let limits = ParseLimits { max_depth: 8, max_statements: 2 };
    let (program, errors) = parse_with_limits("horizontal vertical line l\npoint tail\n", limits);
    assert_eq!(program.stmts().count(), 2);
    assert!(errors.iter().any(|e| e.message.contains("more than 2 statements")), "{errors:?}");
}

#[test]
fn module_statement_ids_cannot_wrap() {
    let (_, errors) = parse_from("point p", 0, u32::MAX);
    assert!(errors.iter().any(|e| e.message.contains("statement IDs")), "{errors:?}");
}

#[test]
fn expansion_reports_typed_diagnostics() {
    for (source, code) in [
        ("param x = 1\nparam x = 2\npoint p", Code::E001),
        ("line l(missing, absent)", Code::E101),
        ("param x = x + 1\npoint p", Code::E041),
    ] {
        let (program, errors) = parse(source);
        assert!(errors.is_empty(), "{errors:?}");
        let expansion = gcs_core::flatten::expand(&program, Default::default());
        assert!(expansion.diagnostics.iter().any(|d| d.code == code), "{source}");
        assert!(elaborate(&program).diags.iter().any(|d| d.code == code), "{source}");
    }
}

#[test]
fn unsupported_exports_preserve_source_and_fragments() {
    use gcs_core::syntax::{render_flat, write_stmt_to};
    for source in [
        "component Empty() {}\npoint p",
        "use library.parts\npoint p",
        "plane top\nin top { point p }",
        "repeat 2 { point p }",
        "claim over reach in (0, 1) { a clear(1) b }",
        "part: Part()",
    ] {
        let (mut p, errors) = parse(source);
        assert!(errors.is_empty(), "{source}: {errors:?}");
        let before = format!("{p:?}");
        assert!(render_flat(&mut p).is_err(), "{source}");
        assert_eq!(format!("{p:?}"), before);
        assert_eq!(p.text(), source);
        for st in p.stmts() {
            if matches!(st.kind, StmtKind::Block(_) | StmtKind::ClaimOver(_)) {
                let mut out = "existing text".to_string();
                assert!(write_stmt_to(&mut out, &st.kind).is_err());
                assert_eq!(out, "existing text");
            }
        }
    }
}

#[test]
fn lowering_preserves_source_and_distinguishes_instances_and_copies() {
    use gcs_core::ir::{Operation, PathStep};
    use gcs_core::syntax::RelationForm;
    let source = "component Part() {\nrepeat 2 {\nhorizontal line l\n}\n}\na: Part()\nb: Part()";
    let (p, errors) = parse(source);
    assert!(errors.is_empty(), "{errors:?}");
    let before = format!("{p:?}");
    let expansion = gcs_core::flatten::expand(&p, Default::default());
    assert!(expansion.diagnostics.is_empty(), "{:?}", expansion.diagnostics);
    let declarations: Vec<_> =
        expansion.flat.iter().filter(|s| matches!(s.kind, Operation::Decl(_))).collect();
    assert_eq!(declarations.len(), 4);
    for st in &declarations {
        assert!(matches!(st.path.as_slice(), [PathStep::Instance(_), PathStep::Copy { .. }]));
        assert_eq!(st.span, p.stmt(st.id).unwrap().span);
    }
    let paths: std::collections::BTreeSet<_> = declarations.iter().map(|s| &s.path).collect();
    assert_eq!(paths.len(), 4);
    let e = elaborate(&p);
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    for st in declarations {
        assert!(e.map.of_entity.values().any(|site| site.stmt == st.id && site.path.0 == st.path));
    }
    assert_eq!(format!("{p:?}"), before);
    assert!(p.stmts().any(
        |s| matches!(&s.kind, StmtKind::Relation(r) if matches!(r.form, RelationForm::Written(_)))
    ));
}
