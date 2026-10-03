use gcs_core::{
    drawing, io, library, model::EntRef, program, semantics::GeometryRoles, solve, syntax,
};
use std::collections::BTreeMap;

fn build(src: &str) -> program::Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(
        errors.is_empty() && linked.is_empty(),
        "{errors:?} {linked:?}"
    );
    program::elaborate(&p)
}
fn good(src: &str) -> program::Elaborated {
    let e = build(src);
    assert!(e.ok(), "{:?}", e.diags);
    e
}
fn private_error(src: &str) {
    let e = build(src);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("private member")),
        "{:?}",
        e.diags
    );
}

#[test]
fn private_names_work_locally_but_not_through_external_paths() {
    let component = "component Part() { private center := point\nfix(x == 0, y == 0) center\npublic := line(center, hint(x: 10,y: 0)) }\np := Part()\n";
    let e = good(component);
    assert!(e.map.entity_path(&e.sketch, "p.center").is_none());
    assert!(e.map.entity_path(&e.sketch, "p.public.p1").is_some());
    for suffix in [
        "fix(x == 0, y == 0) p.center",
        "q := point hint(x: p.center.x, y: 0)",
        "component Use(c: point) { fix(x == 0, y == 0) c }\nUse(p.center)",
        "leaked := {c: p.center}",
    ] {
        private_error(&format!("{component}{suffix}"));
    }
    // Visibility is independent of privacy.
    let center = e.map.ent_named("p.center").unwrap();
    assert_eq!(e.sketch.roles_of(center), GeometryRoles::default());
}

#[test]
fn private_geometry_can_be_explicitly_passed_and_forwarded() {
    good(
        "component Use(c: point) { fix(x == 0, y == 0) c }\n\
          component Forward(c: point) { Use(c) }\n\
          component Owner() { Forward(secret)\nprivate secret := point }\nOwner()",
    );
    good(
        "component Layout() { p := point }\n\
          component Use(g: group) { fix(x == 0, y == 0) g.p }\n\
          component Owner() { Use(layout)\nprivate layout := Layout() }\nOwner()",
    );
    private_error(
        "component Layout() { private p := point }\n\
          component Use(g: group) { fix(x == 0, y == 0) g.p }\n\
          component Owner() { Use(layout)\nprivate layout := Layout() }\nOwner()",
    );
}

#[test]
fn privacy_survives_nested_instances_repetition_and_forward_references() {
    good(
        "component Cell() { private p := point\nfix(x == 0, y == 0) p }\n\
          component Owner() { private cell := Cell() }\nOwner()",
    );
    private_error(
        "fix(x == 0, y == 0) x.hidden.p\ncomponent Child() { p := point }\n\
          component Owner() { private hidden := Child() }\nx := Owner()",
    );
    private_error(
        "component Owner() { repeat 3 { private p := point\nfix(x == 0, y == 0) p } }\n\
          x := Owner()\nfix(x == 0, y == 0) x.p[1]",
    );
    private_error(
        "component Child() { private p := point }\n\
          component Owner() { child := Child()\nfix(x == 0, y == 0) child.p }\nx := Owner()",
    );
    private_error(
        "component Owner() { private p := point }\nx := Owner()\n\
          alias := {layout: x}\nfix(x == 0, y == 0) alias.layout.p",
    );
}

#[test]
fn semantic_instance_roles_do_not_escape_to_arguments_or_consumers() {
    let e = good(
        "component Layout(c: point) { spoke := line(c, hint(x: 10,y: 0))\n\
          ring := radius(3) circle(center: c) }\nborrowed := point\n\
          private construction centerline helper := Layout(borrowed)\n\
          plain := Layout(borrowed)\nhole := radius(2) circle(center: helper.spoke.p2)",
    );
    let role = |name| e.sketch.roles_of(e.map.ent_named(name).unwrap());
    assert_eq!(role("borrowed"), GeometryRoles::default());
    assert_eq!(role("hole"), GeometryRoles::default());
    assert_eq!(role("plain.spoke"), GeometryRoles::default());
    assert_eq!(role("plain.spoke.p2"), GeometryRoles::default());
    for name in ["helper.spoke", "helper.spoke.p2", "helper.ring"] {
        assert_eq!(
            role(name),
            GeometryRoles {
                construction: true,
                centerline: true
            }
        );
    }
}

#[test]
fn roles_preserve_the_constraint_problem_and_round_trip() {
    let tagged = "private construction centerline axis := line(p2: hint(x: 10,y: 0))\n\
                  fix(x == 0, y == 0) axis.p1\nhorizontal axis\ndistance(10) axis";
    let mut e = good(tagged);
    let plain = good(&tagged.replace("private construction centerline ", ""));
    assert_eq!(e.sketch.params.len(), plain.sketch.params.len());
    assert_eq!(e.sketch.constraints.len(), plain.sketch.constraints.len());
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let saved = io::dumps(&e.sketch, None);
    let restored = io::loads(&saved).unwrap();
    assert_eq!(restored.roles, e.sketch.roles);
    let mut lifted = program::to_program(&restored);
    let text = syntax::render_flat(&mut lifted).unwrap();
    let again = good(text);
    assert_eq!(again.sketch.roles, restored.roles);
    let copied = io::copy(&e.sketch, &[EntRef::line(0)]);
    assert_eq!(copied.roles, e.sketch.roles);
    let mut destination = plain.sketch.clone();
    io::paste(&mut destination, &copied, 20.0, 0.0);
    assert_eq!(
        destination.roles_of(EntRef::line(0)),
        GeometryRoles::default()
    );
    assert_eq!(
        destination.roles_of(EntRef::line(1)),
        e.sketch.roles_of(EntRef::line(0))
    );
}

#[test]
fn modifier_syntax_prints_and_invalid_uses_are_diagnosed() {
    let e = good(
        "component Part() { private construction guide := radius(4) circle }\n\
                  centerline p := Part()",
    );
    // Printing a component call and declaration preserves the semantic prefix.
    for st in &e.program.component("Part").unwrap().body {
        if let syntax::StmtKind::Decl(_) = &st.kind {
            let mut text = String::new();
            syntax::write_stmt_to(&mut text, &st.kind).unwrap();
            assert!(text.starts_with("private construction guide := circle"), "{text}");
        }
    }
    for src in [
        "private ground p",
        "construction n := 3",
        "centerline repeat 2 { l := line }",
        "private private p := point",
        "construction construction l := line",
    ] {
        assert!(!syntax::parse(src).1.is_empty(), "{src}");
    }
}

fn sheet(e: &program::Elaborated, body: &str) -> Result<String, drawing::Error> {
    let doc = drawing::parse(&format!("sheet s {{ sketch v(m) at (30,30) {body} }}")).unwrap();
    let models = BTreeMap::from([(
        "m".into(),
        drawing::Model {
            sketch: &e.sketch,
            names: &e.map,
        },
    )]);
    drawing::render(&doc, &models, None)
}

#[test]
fn sheets_hide_construction_by_default_and_can_reveal_it_semantically() {
    let e = good("component Part() { private construction centerline helper := line(hint(x: 0,y: 0), hint(x: 10,y: 0)) }\np := Part()");
    assert!(e.sketch.style_of(EntRef::line(0)).shown());
    let hidden = sheet(&e, "").unwrap();
    assert!(!hidden.contains("stroke-dasharray"), "{hidden}");
    let shown = sheet(
        &e,
        "style .construction { display: inline; color: #ab1234 }",
    )
    .unwrap();
    assert!(shown.contains("#ab1234"), "{shown}");
    assert!(
        shown.contains("12,3,2,3") || shown.contains("12 3 2 3"),
        "{shown}"
    );
    assert!(sheet(&e, "style m.p.helper { display: inline }").is_err());
    assert!(e.sketch.style_of(EntRef::line(0)).shown());
}

#[test]
fn bolt_pattern_exposes_holes_and_keeps_its_layout_and_cutters_private() {
    let e = good(gcs_core::examples::source("solid_flange").unwrap());
    assert!(e.map.entity_path(&e.sketch, "pattern.hole[0]").is_some());
    assert!(e
        .map
        .entity_path(&e.sketch, "pattern.layout.v[0]")
        .is_none());
    assert!(e.map.entity_path(&e.sketch, "pattern.drill[0]").is_none());
    let hole = e.map.entity_path(&e.sketch, "pattern.hole[0]").unwrap();
    assert_eq!(e.sketch.roles_of(hole), GeometryRoles::default());
    assert!(
        e.sketch
            .roles_of(EntRef::point(e.sketch.circles[hole.i()].center as usize))
            .construction
    );
    assert_eq!(
        e.sketch.roles_of(e.map.ent_named("std.origin").unwrap()),
        GeometryRoles::default()
    );
}

#[test]
fn removing_a_unary_constraint_and_updating_hints_preserve_modifiers() {
    let e = good("private construction guide := radius(4) circle hint(r: 4)");
    let cid = e.sketch.user_constraints()[0].id;
    let changed = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[], &[cid]);
    assert!(changed.refused.is_none(), "{:?}", changed.refused);
    let again = good(&changed.text);
    assert!(again.sketch.roles_of(EntRef::circle(0)).construction);
    assert!(again.map.entity_path(&again.sketch, "guide").is_none());
    let e = good("private construction helper := point hint(x: 1,y: 2)");
    let mut moved = e.sketch.clone();
    moved.params[0].value = 7.0;
    let changed = gcs_core::edit::commit_seeds(&e, &moved, &e.program);
    assert!(changed.text.contains("private construction helper := point"));
    assert_eq!(good(&changed.text).sketch.point_xy(0), (7.0, 2.0));
}

#[test]
fn private_points_cannot_be_selected_as_an_anonymous_curve_output() {
    private_error(
        "component Hidden(u: Angle) { private p := point(x: cos(u), y: sin(u)) }\n\
                   c := Hidden(u: 0deg).p over u in (0deg, 360deg)",
    );
}

#[test]
fn tagged_chains_preserve_the_roles_of_borrowed_links() {
    let e = good("borrowed := line(hint(x: 0,y: 0), hint(x: 10,y: 0))\n\
                  private construction profile := borrowed -> (end := line(borrowed.p2, hint(x: 10,y: 10)))");
    assert!(
        !e.sketch
            .roles_of(e.map.ent_named("borrowed").unwrap())
            .construction
    );
    assert!(
        e.sketch
            .roles_of(e.map.ent_named("end").unwrap())
            .construction
    );
    assert!(e.map.entity_path(&e.sketch, "profile").is_none());
}

#[test]
fn editor_does_not_publish_a_component_private_name_into_root_source() {
    for (source, allowed) in [
        ("component Part() { private p := point }\npart := Part()", false),
        ("private p := point", true),
        ("component Part() { p := point }\nprivate part := Part()", true),
    ] {
        let mut e = good(source);
        let mut changed = e.sketch.clone();
        let tip = changed.point(10.0, 0.0, false, "tip");
        changed.line(0, tip);
        let result = gcs_core::edit::reconcile(&mut e, &changed);
        if allowed {
            assert!(result.refused.is_none(), "{:?}", result.refused);
            good(&result.text);
        } else {
            assert!(
                result
                    .refused
                    .as_ref()
                    .is_some_and(|s| s.contains("private")),
                "{result:?}"
            );
        }
    }
}
