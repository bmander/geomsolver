use gcs_core::{
    model::SolidDef,
    plane::Basis,
    program,
    renderer::{Renderer, View},
    solid::{ApproximationPolicy, PageFrame},
    solve,
};

fn document() -> program::Elaborated {
    let source = include_str!("fixtures/solid_issue51/view_layout_0.legacy");
    let (p, errors) = crate::common::parse_legacy(source);
    assert!(errors.is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok());
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    e
}
fn frame(basis: Basis) -> PageFrame {
    PageFrame::new(basis)
}
fn close(actual: [f64; 4], expected: [f64; 4]) {
    for (a, b) in actual.into_iter().zip(expected) {
        assert!((a - b).abs() < 1e-8, "{actual:?} != {expected:?}");
    }
}

#[test]
fn one_prepared_solid_supplies_multiple_views() {
    let e = document();
    let i = e.map.ent_named("result").unwrap().i();
    let solid = e.sketch.evaluated_solid(i, ApproximationPolicy::Report).unwrap();
    drop(e); // the renderer only needs the evaluated snapshot
    let renderer = Renderer::prepare(&solid);
    let side = Basis { u: [0.0, 1.0, 0.0], v: [0.0, 0.0, 1.0], o: [0.0; 3] };
    for (frame, expected) in [
        (frame(Basis::page()), [0.0, 0.0, 10.0, 10.0]),
        (frame(side), [0.0, 0.0, 1.0, 10.0]),
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

// Independent exhaustive visibility reference: a spatial index may reject impossible hits,
// but every material interval and section limit must retain the original answer.
fn exhaustive_visibility(solid: &gcs_core::solid::EvaluatedSolid, m: [f64; 3], eye: [f64; 3], limit: Option<f64>) -> bool {
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = |a: [f64; 3], b: [f64; 3]| [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
    let eps = solid.epsilon();
    let bounds = solid.bounds();
    if bounds.is_empty() { return false; }
    let far = (0..3).map(|k| eye[k] * (if eye[k] >= 0.0 { bounds.hi[k] } else { bounds.lo[k] } - m[k])).sum::<f64>();
    let end = limit.unwrap_or(far + eps).min(far + eps);
    if end <= eps { return false; }
    let at = |t| std::array::from_fn(|k| m[k] + t * eye[k]);
    let mut cuts = vec![eps, end];
    for face in solid.boundary() {
        let den = dot(face.n, eye);
        if den.abs() <= 1e-12 { continue; }
        let t = dot(face.n, std::array::from_fn(|k| face.pts[0][k] - m[k])) / den;
        if t <= eps || t >= end { continue; }
        let x: [f64; 3] = at(t);
        if face.pts.iter().enumerate().all(|(i, a)| {
            let b = face.pts[(i + 1) % face.pts.len()];
            let edge = std::array::from_fn(|k| b[k] - a[k]);
            dot(face.n, cross(edge, std::array::from_fn(|k| x[k] - a[k]))) >= -eps * dot(edge, edge).sqrt()
        }) { cuts.push(t); }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.windows(2).any(|w| w[1] - w[0] > eps * 1e-3
        && solid.contains(gcs_core::solid::LocalPoint(at((w[0] + w[1]) * 0.5))))
}

#[test]
fn spatial_visibility_matches_exhaustive_rays_near_faces_and_section_limits() {
    use gcs_core::solid::LocalPoint;
    let mut throttle = gcs_core::examples::vtwin_throttle();
    assert!(solve::solve(&mut throttle, Default::default()).success);
    for sk in [throttle, document().sketch] {
        let ids: std::collections::BTreeSet<_> = sk.derived.iter().map(|d| d.solid as usize).collect();
        for i in ids {
            let solid = sk.evaluated_solid(i, ApproximationPolicy::View { unit: 0.4 }).unwrap();
            let renderer = Renderer::prepare(&solid);
            let eps = solid.epsilon();
            let mut origins = Vec::new();
            for face in solid.boundary().iter().step_by((solid.boundary().len() / 24).max(1)) {
                for p in [face.centroid(), face.pts[0]] {
                    for offset in [-eps, 0.0, eps] {
                        origins.push(std::array::from_fn(|k| p[k] + offset * face.n[k]));
                    }
                }
            }
            let b = solid.bounds();
            for i in 0..32 {
                origins.push(std::array::from_fn(|k| b.lo[k] + (b.hi[k] - b.lo[k])
                    * (1.2 * (i as f64 * [0.618, 0.414, 0.732][k]).fract() - 0.1)));
            }
            for p in origins {
                for eye in [[1.0,0.0,0.0], [-1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,-1.0,0.0],
                            [0.0,0.0,1.0], [0.0,0.0,-1.0], [0.6,0.8,0.0], [1.0,1e-12,0.0]] {
                    for limit in [None, Some(2.0 * eps), Some(1.0)] {
                        assert_eq!(renderer.occludes(LocalPoint(p), eye, limit),
                            exhaustive_visibility(&solid, p, eye, limit), "{p:?}, {eye:?}, {limit:?}");
                    }
                }
            }
        }
    }
}

#[test]
fn spatial_queries_skip_most_unrelated_geometry_in_the_throttle() {
    let mut sk = gcs_core::examples::vtwin_throttle();
    assert!(solve::solve(&mut sk, Default::default()).success);
    // Presentation is supplied by the caller, independently of the example model.
    let body = sk.solids.iter().position(|s| s.name.ends_with(".body")).unwrap();
    sk.derived.push(gcs_core::model::DerivedE { solid: body as u32, plane: None,
        at: None, dims: false, name: "front".into(), class: Default::default() });
    let (drawing, stats) = gcs_core::renderer::layout_with_stats(&sk, 0.15);
    assert!(!drawing.is_empty());
    assert!(stats.visibility_rays > 0);
    assert!(stats.boundary_candidates < stats.boundary_exhaustive / 2, "{stats:?}");
    assert!(stats.crossing_candidates < stats.crossing_exhaustive / 2, "{stats:?}");
}
