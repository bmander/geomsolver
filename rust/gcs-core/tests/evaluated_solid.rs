use gcs_core::{
    clear,
    constraints::SolidWord,
    gltf, hidden,
    model::SolidDef,
    plane::Basis,
    program,
    solid::{ApproximationPolicy as Policy, LocalPoint, PageFrame, PagePoint, WorldPoint},
    solve, syntax,
};
use std::rc::Rc;

fn read(source: &str) -> program::Elaborated {
    let (p, errors) = syntax::parse(source);
    assert!(errors.is_empty(), "{errors:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    e
}
const BOX: &str = include_str!("fixtures/solid_issue51/view_layout_0.sv");
const BORE: &str = include_str!("fixtures/solid_issue51/ghost_dimension_through.sv");
fn index(e: &program::Elaborated) -> usize {
    e.map.ent_named("result").unwrap().i()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9 * b.abs().max(1.0), "{a} != {b}");
}

#[test]
fn coordinate_conversions_preserve_depth_rotation_and_page_placement() {
    let mut e = read(BOX);
    e.sketch.planes[0].basis.o = [123.0, -456.0, 789.0];
    let s = e.sketch.evaluated_solid(index(&e), Policy::Report).unwrap();
    let local = LocalPoint([2.0, 0.5, 3.0]);
    assert_eq!(s.to_local(s.to_world(local)), local);
    let page_frame = PageFrame::new(
        Basis {
            o: s.origin().0,
            ..Basis::page()
        },
        (1.0, 0.0, (0.0, 0.0)),
    );
    assert_eq!(
        s.from_page(s.to_page(local, page_frame), -0.5, page_frame),
        local
    );
    assert!(s.contains(local));
    assert!(s.contains_world(s.to_world(local)));
    let basis = Basis {
        o: s.origin().0,
        ..Basis::page()
    };
    let frame = PageFrame::new(basis, (0.0, 1.0, (20.0, 30.0)));
    assert_eq!(s.to_page(local, frame), PagePoint((17.0, 32.0)));
    assert_eq!(frame.project(s.to_world(local)), s.to_page(local, frame));
    assert_eq!(
        frame.unproject(s.to_page(local, frame), -0.5),
        s.to_world(local)
    );
    assert_eq!(
        s.to_local(WorldPoint([123.0, -456.0, 789.0])),
        LocalPoint([0.0; 3])
    );
}

#[test]
fn policies_coexist_and_geometry_placement_and_provenance_invalidate_cache() {
    let mut e = read(BORE);
    let i = index(&e);
    let a = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    let mesh = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
    let view = e
        .sketch
        .evaluated_solid(i, Policy::View { unit: 0.2 })
        .unwrap();
    assert_eq!(mesh.policy(), Policy::Mesh);
    assert_eq!(view.policy(), Policy::View { unit: 0.2 });
    assert!(!Rc::ptr_eq(&a, &mesh));
    assert!(Rc::ptr_eq(
        &a,
        &e.sketch.evaluated_solid(i, Policy::Report).unwrap()
    ));
    assert!(Rc::ptr_eq(
        &mesh,
        &e.sketch.evaluated_solid(i, Policy::Mesh).unwrap()
    ));
    e.sketch.point(1e12, 1e12, true, "unrelated");
    assert!(Rc::ptr_eq(
        &a,
        &e.sketch.evaluated_solid(i, Policy::Report).unwrap()
    ));
    let tool = e.map.ent_named("fp").unwrap().i();
    let x = e.sketch.points[tool].x as usize;
    let y = e.sketch.points[tool].y as usize;
    e.sketch.params[x].value = 5.0;
    e.sketch.params[y].value = 5.0;
    let cut = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    assert!(!Rc::ptr_eq(&a, &cut));
    assert!(cut.volume() < a.volume());
    assert_eq!(a.round_features().len(), 0);
    assert_eq!(cut.round_features().len(), 1);
    assert_eq!(
        e.sketch
            .solid_cache
            .borrow()
            .keys()
            .filter(|(s, _)| *s == i)
            .count(),
        1
    );
    // Same scalar values, different operation: changing a cutter to an addition is a new solid.
    if let SolidDef::Body { on, through, .. } = &mut e.sketch.solids[i].def {
        *on = std::mem::take(through);
    }
    let added = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    close(added.volume(), 100.0);
    assert!(!Rc::ptr_eq(&cut, &added));
    let stock = e.map.ent_named("stock").unwrap().i();
    e.sketch.solids[stock].name = "renamed_stock".into();
    let renamed = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    assert!(renamed
        .surviving_faces()
        .iter()
        .any(|p| p.starts_with("renamed_stock.")));
    assert!(!Rc::ptr_eq(&added, &renamed));

    let mut e = read(BOX);
    let i = index(&e);
    let a = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    e.sketch.planes[0].basis.o = [1e9, 1e9, 1e9];
    let moved = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    assert!(!Rc::ptr_eq(&a, &moved));
    close(a.volume(), moved.volume());
    assert_eq!(a.mesh().positions, moved.mesh().positions);
}

#[test]
fn invalid_geometry_policy_and_cycles_return_diagnostics() {
    let mut e = read(BOX);
    let i = index(&e);
    for unit in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(e.sketch.evaluated_solid(i, Policy::View { unit }).is_err());
    }
    let valid = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    if let SolidDef::Prism { from, to, .. } = &mut e.sketch.solids[i].def {
        to.value = from.value;
    }
    assert!(e
        .sketch
        .evaluated_solid(i, Policy::Report)
        .unwrap_err()
        .contains("distinct"));
    assert!(valid.volume() > 0.0); // old snapshot stays immutable
    assert!(gltf::checked_glb(&e.sketch, &[i], Policy::Mesh).is_err());
    assert_eq!(
        clear::judge(&e.sketch, SolidWord::Inside, i, i, 0.0, 0.0).holds,
        None
    );
    e.sketch.solids[i].def = SolidDef::Body {
        stock: i as u32,
        on: vec![],
        through: vec![],
    };
    assert!(e
        .sketch
        .evaluated_solid(i, Policy::Mesh)
        .unwrap_err()
        .contains("cyclic"));
}

#[test]
fn translation_keeps_local_geometry_collision_and_export_triangles() {
    let mut e = read(BOX);
    let i = index(&e);
    let original = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
    let before = clear::judge_evaluated(SolidWord::Clear, &original, &original, 0.0);
    let original_glb = gltf::checked_glb(&e.sketch, &[i], Policy::Mesh).unwrap();
    e.sketch.planes[0].basis.o = [1e12, -1e12, 1e12];
    let moved = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
    assert_eq!(original.mesh().positions, moved.mesh().positions);
    assert_eq!(original.epsilon(), moved.epsilon());
    let after = clear::judge_evaluated(SolidWord::Clear, &moved, &moved, 0.0);
    close(before.measured, after.measured);
    assert_eq!(before.holds, after.holds);
    let moved_glb = gltf::checked_glb(&e.sketch, &[i], Policy::Mesh).unwrap();
    let binary = |bytes: Vec<u8>| {
        let len = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
        bytes[28 + len..].to_vec()
    };
    assert_eq!(binary(original_glb), binary(moved_glb));
    assert!(moved.stl().unwrap_err().contains("float32 STL"));
}

#[test]
fn scale_and_page_layout_do_not_supply_geometric_tolerances() {
    for scale in [1e-6, 1.0, 1e6] {
        let mut e = read(BOX);
        let i = index(&e);
        for p in &e.sketch.points {
            e.sketch.params[p.x as usize].value *= scale;
            e.sketch.params[p.y as usize].value *= scale;
        }
        if let SolidDef::Prism { from, to, .. } = &mut e.sketch.solids[i].def {
            from.value *= scale;
            to.value *= scale;
        }
        let solid = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
        close(solid.volume() / scale.powi(3), 100.0);
        close(solid.epsilon() / scale, 1e-5);
        let strokes = hidden::layout(&e.sketch, 0.0);
        assert_eq!(strokes.len(), 4);
    }
    let mut e = read(BOX);
    let i = index(&e);
    let a = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
    for p in &e.sketch.points {
        e.sketch.params[p.x as usize].value += 1e9;
        e.sketch.params[p.y as usize].value += 1e9;
    }
    let b = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
    assert_eq!(a.mesh().positions, b.mesh().positions);
    close(a.volume(), b.volume());
    assert_eq!(a.epsilon(), b.epsilon());
}

#[test]
fn two_solid_queries_keep_relative_placement_under_translation() {
    let mut e = read(include_str!("fixtures/solid_issue51/cross_clear.sv"));
    let (a, b) = (index(&e), e.map.ent_named("other").unwrap().i());
    let before = clear::judge(&e.sketch, SolidWord::Clear, a, b, 0.1, 0.0);
    for p in &e.sketch.points {
        e.sketch.params[p.x as usize].value += 1e9;
        e.sketch.params[p.y as usize].value -= 1e9;
    }
    let after = clear::judge(&e.sketch, SolidWord::Clear, a, b, 0.1, 0.0);
    assert_eq!(after.holds, Some(false));
    close(before.measured, after.measured);
}

#[test]
fn curved_mesh_policy_keeps_its_own_cost_and_glb_reports_precision_failure() {
    let mut e = read(&BORE.replace("x: 20, y: 20", "x: 5, y: 5"));
    let i = index(&e);
    let mesh = e.sketch.evaluated_solid(i, Policy::Mesh).unwrap();
    let report = e.sketch.evaluated_solid(i, Policy::Report).unwrap();
    assert!(mesh.mesh().positions.len() < report.mesh().positions.len());
    assert!(Rc::ptr_eq(
        &mesh,
        &e.sketch.evaluated_solid(i, Policy::Mesh).unwrap()
    ));
    // Unit conversion can make otherwise valid local geometry unrepresentable in float32.
    e.sketch.units.length.as_mut().unwrap().1 = 1e-47;
    assert!(gltf::checked_glb(&e.sketch, &[i], Policy::Mesh)
        .unwrap_err()
        .contains("float32 GLB"));
}
