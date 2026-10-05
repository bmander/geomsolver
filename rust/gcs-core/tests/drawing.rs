use gcs_core::{drawing, program, solve};
use std::collections::BTreeMap;

const MODEL: &str = "\
unit mm
use std
in std.front {
o := point
fix(x == 0, y == 0) o
rim := circle(center: o) hint(r: 10)
}
param r := 10mm
in std.front {
radius(r) rim
}
stock := solid(face(rim), depth: 4mm)
body := solid(stock)
";

fn solved() -> program::Elaborated {
    let (p, errs) = crate::common::parse(MODEL);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve::solve(&mut e.sketch, solve::SolveOpts::default()).success);
    e
}

#[test]
fn vtwin_paper_side_view_is_upright_and_meets_the_cylinders() {
    let (p, errors, linked) = gcs_core::library::parse_linked(
        gcs_core::examples::source("vtwin").unwrap());
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let doc = drawing::parse(include_str!("../../examples/vtwin/assembly.svd")).unwrap();
    let sheet = &doc.sheets[0];
    // the side plane's sketch, where the side view's points are drawn
    let source = sheet.views.iter().find(|v| v.sketch && v.name == "side").unwrap();
    let cylinder = e.sketch.point_xy(e.map.ent_named("side.cylB.b").unwrap().i());
    let px_mm = 96.0 / 25.4;
    let cylinder_back_x = (source.at.0 + cylinder.0 * source.scale.unwrap_or(sheet.scale)) * px_mm;
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &e.sketch, names: &e.map })]);
    let svg = drawing::render(&doc, &models, None).unwrap();
    let edge = svg.lines().find(|line| line.contains("data-path=\"plate.blank.stock.near\""))
        .expect("the plate's front face must appear in the paper side view");
    let points: Vec<(f64, f64)> = edge.split("points=\"").nth(1).unwrap()
        .split('"').next().unwrap().split_whitespace().map(|p| {
            let (x, y) = p.split_once(',').unwrap();
            (x.parse().unwrap(), y.parse().unwrap())
        }).collect();
    // The projected plate face is vertical and shares the cylinders' back face.
    // This catches both a quarter-turn projection and a reversed thickness offset.
    assert!(points.iter().all(|p| (p.0 - cylinder_back_x).abs() < 1e-3),
        "plate edge {points:?} does not meet cylinder back at {cylinder_back_x}");
    let height = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max)
        - points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    assert!(height > 10.0 * px_mm, "the plate must stand upright: {points:?}");
}

#[test]
fn sheets_read_one_model_without_changing_its_geometry_or_presentation() {
    let e = solved();
    let before = gcs_core::io::to_json(&e.sketch);
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &e.sketch, names: &e.map })]);
    let doc = drawing::parse("model m from \"part.sv\"
        sheet one { size A4 scale 2 view front(m.body) from front at (50mm, 60mm)
          dimensions in front style .visible { color: #ff0000 } }
        sheet two { size (100mm, 80mm) view top(m.body) from top at (20mm, 30mm)
          style .visible { color: #0000ff } }").unwrap();
    let a = drawing::render(&doc, &models, Some("one")).unwrap();
    let b = drawing::render(&doc, &models, Some("two")).unwrap();
    assert!(a.contains("width=\"210mm\"") && a.contains("#ff0000") && a.contains("<text"));
    assert!(b.contains("width=\"100mm\"") && b.contains("#0000ff"));
    assert_ne!(a, b);
    assert_eq!(before, gcs_core::io::to_json(&e.sketch));
    assert!(drawing::render(&doc, &models, None).is_err());
}

#[test]
fn drawing_references_are_checked_and_cannot_be_model_statements() {
    let e = solved();
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &e.sketch, names: &e.map })]);
    for text in [
        "sheet s { view a(m.missing) at (0,0) }",
        "sheet s { view a(m.rim) at (0,0) }",
        "sheet s { sketch a(m) at (0,0) dimension m.missing in a }",
        "sheet s { sketch a(m) at (0,0) style m.missing { width: 2 } }",
    ] {
        assert!(drawing::render(&drawing::parse(text).unwrap(), &models, None).is_err(), "{text}");
    }
    for text in ["use std\nin std.front {\np := point\n}\n", "sheet s { p := point }", "sheet s { scale 0 }",
        "sheet s { view a(m.body) at (NaN,0) }", "model m from \"unfinished",
        "sheet s { style .visible { color: red } }"] {
        assert!(drawing::parse(text).is_err(), "{text}");
    }
}

#[test]
fn model_sources_refuse_presentation_but_keep_geometry_and_claims() {
    for extra in ["style .hidden { width: 2 }", "view(body) in front",
        "a := point class hidden", "extra := solid(face(rim) class hidden, depth: 2mm)",
        "o distance(20) o at (1,2)"] {
        let (_, errs) = crate::common::parse(&format!("{MODEL}\n{extra}"));
        assert!(errs.iter().any(|e| e.message.contains(".svd")), "{errs:?}");
    }
    let (_, errs) = crate::common::parse(&format!("{MODEL}\nclaim radius(10mm) rim"));
    assert!(errs.is_empty(), "{errs:?}");
}

#[test]
fn requested_dimensions_survive_statement_reordering() {
    let e = solved();
    let doc = drawing::parse("sheet s { sketch v(m) at (50,50) dimension m.r in v at (0,15) }").unwrap();
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &e.sketch, names: &e.map })]);
    let a = drawing::render(&doc, &models, None).unwrap();
    assert!(a.contains("<text"));
    let text = MODEL.replace("radius(r) rim\n", "") + "radius(r) rim\n";
    let (p, errs) = crate::common::parse(&text); assert!(errs.is_empty());
    let mut other = program::elaborate(&p);
    assert!(solve::solve(&mut other.sketch, solve::SolveOpts::default()).success);
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &other.sketch, names: &other.map })]);
    assert_eq!(a, drawing::render(&doc, &models, None).unwrap());
}

#[test]
fn host_loading_is_relative_cached_and_cycle_checked() {
    let mut reads = Vec::new();
    let svg = drawing::compile("model m from \"part.sv\" use \"ink.svd\"
        sheet s { view v(m.body) at (50,50) }", "drawing.svd", None, &mut |path, from| {
            reads.push((path.to_string(), from.to_string()));
            match path {
                "part.sv" => Some((path.into(), MODEL.into())),
                "ink.svd" => Some((path.into(), "style .visible { color: #123456 }".into())),
                _ => None,
            }
        }).unwrap();
    assert!(svg.contains("#123456"));
    assert!(reads.contains(&("part.sv".into(), "drawing.svd".into())));
    let err = drawing::compile("use \"loop.svd\"", "root.svd", None,
        &mut |_, _| Some(("loop.svd".into(), "use \"loop.svd\"".into()))).unwrap_err();
    assert!(err.message.contains("cycle"));
}

#[test]
fn indexed_members_and_field_measurements_survive_reordering() {
    let model = "\
unit mm
use std
in std.front {
repeat 3 as i {
  p := point
  fix(x == i * 10, y == 0) p
}
bar := line(p[0], p[2])
}
";
    let drawing = "model m from \"part.sv\" sheet s {
        sketch v(m) at (30mm,40mm)
        measure distance(m.bar.p1, m.p[2]) in v offset 8mm
    }";
    let render = |text: &str| drawing::compile(drawing, "drawing.svd", None,
        &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), text.into()))).unwrap();
    let a = render(model);
    assert!(a.contains(">20</text>"), "{a}");
    assert_eq!(a, render(&("unused := 7\n".to_string() + model)));
}

#[test]
fn measurements_refuse_foreshortening_and_sections_check_the_cut_plane() {
    let model = format!("{MODEL}in std.front {{\nb := point\nfix(x == 0, y == 10) b\n}}\n");
    let compile = |text: &str| drawing::compile(text, "drawing.svd", None,
        &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), model.clone())));
    let err = compile("model m from \"part.sv\" sheet s {
        view v(m.body) from top at (30,40)
        measure distance(m.o,m.b) in v
    }").unwrap_err();
    assert!(err.message.contains("foreshortens"), "{err:?}");
    let err = compile("model m from \"part.sv\" sheet s {
        section v(m.body) from front cut m.o at (30,40)
    }").unwrap_err();
    assert!(err.message.contains("not a model plane"), "{err:?}");
}

#[test]
fn styles_can_show_one_point_and_hide_selected_dimensions() {
    let e = solved();
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &e.sketch, names: &e.map })]);
    let doc = drawing::parse("sheet s { sketch v(m) at (30,40)
        dimension m.r in v
        style m.o { display: inline; color: #ff0000 }
        style .dimension { display: none }
    }").unwrap();
    let svg = drawing::render(&doc, &models, None).unwrap();
    assert!(svg.contains("fill=\"#ff0000\""), "{svg}");
    assert!(!svg.contains("<text"), "{svg}");
}

#[test]
fn isometric_camera_matches_the_old_helper_plane_without_model_geometry() {
    let plain = solved();
    let source = format!("{MODEL}\niu := ray hint(x: 1, y: -1, z: 0)\nfix(x == 1, y == -1, z == 0) iu\n\
        iv := ray hint(x: 1, y: 1, z: 2)\nfix(x == 1, y == 1, z == 2) iv\n\
        iso := plane(u: iu, v: iv)\nfix(x == 0, y == 0, z == 0) iso");
    let (p, errs) = crate::common::parse(&source);
    assert!(errs.is_empty(), "{errs:?}");
    let mut with_helper = program::elaborate(&p);
    assert!(with_helper.ok(), "{:?}", with_helper.diags);
    assert!(solve::solve(&mut with_helper.sketch, Default::default()).success);
    let render = |model: &program::Elaborated, from: &str| {
        let doc = drawing::parse(&format!("sheet s {{ view v(m.body) from {from} at (50,50) }}")).unwrap();
        let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &model.sketch, names: &model.map })]);
        drawing::render(&doc, &models, None).unwrap()
    };
    let before = gcs_core::io::dumps(&plain.sketch, None);
    assert_eq!(render(&plain, "isometric"), render(&with_helper, "m.iso"));
    assert_eq!(gcs_core::io::dumps(&plain.sketch, None), before);
    // the camera made no plane of the model's: only the standard ones `use std` brings
    assert_eq!(plain.sketch.planes.len(), 4);
}
