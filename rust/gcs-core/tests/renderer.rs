use gcs_core::{
    model::SolidDef,
    plane::Basis,
    program,
    renderer::{Renderer, View},
    solid::{ApproximationPolicy, PageFrame},
    solve, syntax,
};

fn document() -> program::Elaborated {
    let source = include_str!("fixtures/solid_issue51/view_layout_0.sv");
    let (p, errors) = syntax::parse(source);
    assert!(errors.is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok());
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    e
}
fn frame(basis: Basis) -> PageFrame {
    PageFrame::new(basis, (1.0, 0.0, (0.0, 0.0)))
}
fn close(actual: [f64; 4], expected: [f64; 4]) {
    for (a, b) in actual.into_iter().zip(expected) {
        assert!((a - b).abs() < 1e-8, "{actual:?} != {expected:?}");
    }
}

#[test]
fn one_prepared_solid_supplies_multiple_views_and_page_placements() {
    let e = document();
    let i = e.map.ent_named("result").unwrap().i();
    let solid = e.sketch.evaluated_solid(i, ApproximationPolicy::Report).unwrap();
    drop(e); // the renderer only needs the evaluated snapshot
    let renderer = Renderer::prepare(&solid);
    let side = Basis { u: [0.0, 1.0, 0.0], v: [0.0, 0.0, 1.0], o: [0.0; 3] };
    for (frame, expected) in [
        (frame(Basis::page()), [0.0, 0.0, 10.0, 10.0]),
        (frame(side), [0.0, 0.0, 1.0, 10.0]),
        (PageFrame::new(Basis::page(), (0.0, 1.0, (100.0, 200.0))), [90.0, 200.0, 100.0, 210.0]),
    ] {
        close(renderer.bounds(frame).unwrap(), expected);
        let drawing = renderer.project(View { frame, section: None });
        close(drawing.bounds.unwrap(), expected);
        assert_eq!(drawing.strokes.len(), 4);
        assert!(drawing.strokes.iter().all(|s| !s.hidden && !s.silhouette));
        assert!(drawing.strokes.iter().all(|s| s.path.starts_with("result.")));
    }
}

#[test]
fn sections_keep_source_paths_and_report_empty_drawings_without_bounds() {
    let e = document();
    let i = e.map.ent_named("result").unwrap().i();
    let solid = e.sketch.evaluated_solid(i, ApproximationPolicy::Report).unwrap();
    let renderer = Renderer::prepare(&solid);
    let frame = frame(Basis::page());
    let section = |y| Some(Basis { o: [0.0, y, 0.0], ..Basis::page() });
    let middle = renderer.project(View { frame, section: section(0.5) });
    close(middle.bounds.unwrap(), [0.0, 0.0, 10.0, 10.0]);
    assert_eq!(middle.strokes.len(), 4);
    assert!(middle.strokes.iter().all(|s| !s.hidden && !s.path.is_empty()));
    let outside = renderer.project(View { frame, section: section(2.0) });
    assert!(outside.strokes.is_empty());
    assert!(outside.bounds.is_none());
    // Conservative bounds still cover the uncut solid; they never run section visibility.
    close(renderer.bounds(frame).unwrap(), [0.0, 0.0, 10.0, 10.0]);
}

#[test]
fn editing_a_document_cannot_change_a_prepared_renderers_snapshot() {
    let mut e = document();
    let i = e.map.ent_named("result").unwrap().i();
    let solid = e.sketch.evaluated_solid(i, ApproximationPolicy::Report).unwrap();
    let renderer = Renderer::prepare(&solid);
    let view = View {
        frame: frame(Basis { u: [0.0, 1.0, 0.0], v: [0.0, 0.0, 1.0], o: [0.0; 3] }),
        section: None,
    };
    let before = renderer.project(view);
    let SolidDef::Prism { from, .. } = &mut e.sketch.solids[i].def else { panic!("prism") };
    from.value = -3.0;
    let changed = e.sketch.evaluated_solid(i, ApproximationPolicy::Report).unwrap();
    let after = Renderer::prepare(&changed).project(view);
    close(before.bounds.unwrap(), [0.0, 0.0, 1.0, 10.0]);
    close(after.bounds.unwrap(), [0.0, 0.0, 3.0, 10.0]);
    assert_eq!(renderer.project(view), before);
}
