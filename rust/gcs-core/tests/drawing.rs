use gcs_core::{drawing, program, solve};
use std::collections::BTreeMap;

const MODEL: &str = "\
unit mm
use std
use std.solids
in std.front {
o := point
fix((0, 0)) o
rim := circle(center: o) hint(r: 10)
}
param r := 10mm
in std.front {
radius(r) rim
}
stock := solid(std.solids.Prism(face(rim), from: -4mm, to: 0))
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
    for text in ["use std\nuse std.solids\nin std.front {\np := point\n}\n", "sheet s { p := point }", "sheet s { scale 0 }",
        "sheet s { view a(m.body) at (NaN,0) }", "model m from \"unfinished",
        "sheet s { style .visible { color: red } }"] {
        assert!(drawing::parse(text).is_err(), "{text}");
    }
}

#[test]
fn model_sources_refuse_presentation_but_keep_geometry_and_claims() {
    for extra in ["style .hidden { width: 2 }", "view(body) in front",
        "a := point class hidden", "extra := solid(std.solids.Prism(face(rim) class hidden, from: -2mm, to: 0))",
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
                "part.sv" => Some(Ok((path.into(), MODEL.into()))),
                "ink.svd" => Some(Ok((path.into(), "style .visible { color: #123456 }".into()))),
                _ => None,
            }
        }).unwrap();
    assert!(svg.contains("#123456"));
    assert!(reads.contains(&("part.sv".into(), "drawing.svd".into())));
    let err = drawing::compile("use \"loop.svd\"", "root.svd", None,
        &mut |_, _| Some(Ok(("loop.svd".into(), "use \"loop.svd\"".into())))).unwrap_err();
    assert!(err.message.contains("cycle"));
}

#[test]
fn indexed_members_and_field_measurements_survive_reordering() {
    let model = "\
unit mm
use std
use std.solids
in std.front {
repeat 3 as i {
  p := point
  fix((i * 10, 0)) p
}
bar := line(p[0], p[2])
}
";
    let drawing = "model m from \"part.sv\" sheet s {
        sketch v(m) at (30mm,40mm)
        measure distance(m.bar.p1, m.p[2]) in v offset 8mm
    }";
    let render = |text: &str| drawing::compile(drawing, "drawing.svd", None,
        &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), text.into())).map(Ok)).unwrap();
    let a = render(model);
    assert!(a.contains(">20</text>"), "{a}");
    assert_eq!(a, render(&("unused := 7\n".to_string() + model)));
}

#[test]
fn measurements_refuse_foreshortening_and_sections_check_the_cut_plane() {
    let model = format!("{MODEL}in std.front {{\nb := point\nfix((0, 10)) b\n}}\n");
    let compile = |text: &str| drawing::compile(text, "drawing.svd", None,
        &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), model.clone())).map(Ok));
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

/// **A style for one view** (#461): `style SELECTOR in VIEW { … }` in a sheet styles that view of
/// the sheet alone, so one part may be ghosted in one view and drawn plainly in another; a view
/// the sheet does not have, or such a rule outside a sheet, is refused.
#[test]
fn a_style_may_be_for_one_view_of_its_sheet() {
    let e = solved();
    let models = BTreeMap::from([("m".into(), drawing::Model { sketch: &e.sketch, names: &e.map })]);
    let doc = drawing::parse("sheet s { sketch a(m) at (30,40)
        sketch b(m) at (130,40)
        style m.rim in a { color: #ff0000 }
    }").unwrap();
    let svg = drawing::render(&doc, &models, None).unwrap();
    let view = |name: &str| {
        let lo = svg.find(&format!("<g id=\"{name}\"")).unwrap();
        svg[lo..lo + svg[lo..].find("</g>").unwrap()].to_string()
    };
    assert!(view("a").contains("#ff0000") && !view("b").contains("#ff0000"), "{svg}");
    let at = |src: &str| drawing::parse(src).err().map(|e| e.message).unwrap_or_default();
    assert_eq!(at("sheet s { sketch a(m) at (30,40)\n style m.rim in c { color: #ff0000 } }"),
        "no view named `c`");
    assert!(at("style m.rim in a { color: #ff0000 }").contains("stands in that view's sheet"));
}

#[test]
fn isometric_camera_matches_the_old_helper_plane_without_model_geometry() {
    let plain = solved();
    let source = format!("{MODEL}\niu := axis hint(dir: (1, -1, 0))\nfix(dir == (1, -1, 0)) iu\n\
        iv := axis hint(dir: (1, 1, 2))\nfix(dir == (1, 1, 2)) iv\n\
        iso := plane(u: iu, v: iv)\nfix(origin == (0, 0, 0)) iso");
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
    assert_eq!(plain.sketch.frames.len(), 4);
}

#[test]
fn a_sketch_draws_the_plane_it_is_from_in_that_planes_coordinates() {
    // one line in the front and one in the top, at different places in their own coordinates
    let model = "\
unit mm
use std
use std.solids
in std.front {
  f := line
  fix((0, 0)) f.p1
  fix((10, 0)) f.p2
}
in std.top {
  t := line
  fix((0, 0)) t.p1
  fix((0, 20)) t.p2
}
";
    let compile = |sheet: &str| drawing::compile(&format!("model m from \"part.sv\" sheet s {{ {sheet} }}"),
        "drawing.svd", None,
        &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), model.into())).map(Ok));
    // every drawn line's two ends, in paper pixels
    let lines = |svg: &str| svg.lines().filter(|l| l.starts_with("<line")).map(|l| {
        ["x1", "y1", "x2", "y2"].map(|k| l.split(&format!(" {k}=\"")).nth(1).unwrap()
            .split('"').next().unwrap().parse::<f64>().unwrap())
    }).collect::<Vec<_>>();
    let near = |a: &[[f64; 4]], b: [f64; 4]| a.len() == 1 && (0..4).all(|k| (a[0][k] - b[k]).abs() < 1e-3);
    let mm = 96.0 / 25.4;
    let front = compile("sketch v(m) at (0mm, 0mm)").unwrap();
    let top = compile("sketch v(m) from m.std.top at (0mm, 0mm)").unwrap();
    // the front's line alone, 10 across; the top's alone, 20 up its own v (the paper's -y)
    assert!(near(&lines(&front), [0.0, 0.0, 10.0 * mm, 0.0]), "{front}");
    assert!(near(&lines(&top), [0.0, 0.0, 0.0, -20.0 * mm]), "{top}");
    let err = compile("sketch v(m) from m.f at (0mm, 0mm)").unwrap_err();
    assert!(err.message.contains("not a model plane"), "{err:?}");
}

/// **A model shown at an instant** (#398 L): `model m from "…" at t = 90deg` stands the drawing
/// there, on the branch its march carries — presentation, so the same file is drawn at home under
/// another alias — and an instant the drawing cannot be shown at is refused, said why.
#[test]
fn a_model_is_shown_at_the_instant_its_drawing_states() {
    let model = include_str!("../../examples/mechanisms/four_bar.sv");
    let compile = |text: &str| drawing::compile(text, "drawing.svd", None,
        &mut |path, _| (path == "four_bar.sv").then(|| ("four_bar.sv".into(), model.into())).map(Ok));
    let svg = compile("model home from \"four_bar.sv\"
        model quarter from \"four_bar.sv\" at t = 90deg
        sheet s {
          sketch a(home) at (30mm, 40mm)
          measure distance(home.a, home.c) in a
          sketch b(quarter) at (130mm, 40mm)
          measure distance(quarter.a, quarter.c) in b
        }").unwrap();
    // the rocker pin from the fixed pivot: 42.72 at home, 39.9994 a quarter turn on
    assert!(svg.contains(">42.72</text>"), "{svg}");
    assert!(svg.contains(">39.9994</text>"), "{svg}");
    // one word, and the same instant again
    assert_eq!(svg, compile("model home from \"four_bar.sv\"
        model quarter from \"four_bar.sv\" at t=90deg
        sheet s {
          sketch a(home) at (30mm, 40mm)
          measure distance(home.a, home.c) in a
          sketch b(quarter) at (130mm, 40mm)
          measure distance(quarter.a, quarter.c) in b
        }").unwrap());
    let refused = |at: &str| compile(&format!("model m from \"four_bar.sv\" at {at}
        sheet s {{ sketch a(m) at (30mm, 40mm) }}")).unwrap_err().message;
    assert_eq!(refused("t = 400deg"), "model `four_bar.sv` at `t`: 400 is not reached");
    assert_eq!(refused("s = 1"), "model `four_bar.sv` at `s`: `s` is no parameter the drawing \
               declares");
    assert_eq!(refused("t = 5mm"), "model `four_bar.sv` at `t`: `5mm`: `t` is Angle, and this is \
               Length");
}

/// **A sketch draws what lies in its plane** (#398 K): a part drawn in a frame of its own, the
/// frame turned a quarter turn in the front plane, is drawn in a sketch of the front where it
/// stands — its line up the page, and its arc as round as it is.
#[test]
fn a_sketch_draws_a_frame_lying_in_its_plane() {
    let model = "unit mm\nuse std\nuse std.solids\nin std.front {\n  up := point\n  fix((0, 10)) up\n  \
                 axes := std.Turned(std.origin, up)\n}\nin axes.axes {\n  a := point hint((10, 0))\n  \
                 fix((10, 0)) a\n  stick := line(axes.axes.origin, a)\n  c := point hint((4, 0))\n  \
                 fix((4, 0)) c\n  s := point hint((6, 0))\n  e := point hint((4, 2))\n  \
                 bend := arc(c, s, e)\n  radius(2) bend\n  fix((6, 0)) s\n}\n";
    let sheet = "model m from \"part.sv\"\nsheet s {\n  sketch a(m) at (100mm, 100mm)\n}\n";
    let svg = drawing::compile(sheet, "drawing.svd", None,
        &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), model.into())).map(Ok)).unwrap();
    // 10mm up the page from (100mm, 100mm), in CSS pixels (96 an inch)
    assert!(svg.contains("<line x1=\"377.953\" y1=\"377.953\" x2=\"377.953\" y2=\"340.157\""),
            "{svg}");
    // the arc about (0, 4)mm from its start 2mm above that to its end 2mm left, its radius 2mm
    // whichever way its frame is turned
    assert!(svg.contains("M 377.953 355.276 A 7.55906 7.55906 0 0 0 370.394 362.835"), "{svg}");
}

/// **A round outline's extent is its own** (#398 K): the crank disc, drawn in a frame of its own
/// turned to its pin, is dimensioned across its rim at 36 — the circle's own reach, not the
/// corners its tessellation happens to put nearest the extent.
#[test]
fn a_parts_extent_is_read_off_its_round_outline() {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../examples/vtwin/");
    let text = std::fs::read_to_string(format!("{root}disc.svd")).unwrap();
    let svg = drawing::compile(&text, "disc.svd", None, &mut |path, from| {
        let p = drawing::relative_path(path, from);
        std::fs::read_to_string(format!("{root}{p}")).ok().map(|t| Ok((p, t)))
    }).unwrap();
    assert_eq!(svg.matches(">36</text>").count(), 3, "{svg}");
    assert!(!svg.contains(">35.9999</text>"), "{svg}");
}

/// The views a sheet names by a word (`drawing::named_view`), which `solventc --view` names too:
/// each stands square on where the standard planes do, the isometric from the front's left.
#[test]
fn a_view_named_by_a_word_is_the_eye_square_on_to_it() {
    use gcs_core::overview::workspace::look_at;
    let deg = |(az, el): (f64, f64)| (az.to_degrees().round(), el.to_degrees().round());
    for (name, eye) in [("front", (-90.0, 0.0)), ("back", (90.0, 0.0)), ("right", (0.0, 0.0)),
                        ("left", (180.0, 0.0)), ("top", (-90.0, 90.0)), ("bottom", (-90.0, -90.0)),
                        ("isometric", (-135.0, 35.0))] {
        let b = drawing::named_view(name).unwrap();
        assert_eq!(deg(look_at(&b)), eye, "{name}");
    }
    assert!(drawing::named_view("std.front").is_none(), "a model's plane is the model's to say");
}

/// **A drawing gives its models values** (#940): `model m from "…" with g = 3` draws the file with
/// the value given, a bare number in the unit of the one it replaces; a run's values (`solventc
/// drawing.svd --set w=2`) go to every model binding the name, but where an import's own `with`
/// gives it; and a name no model binds, or a `with` naming no value of its model, is refused.
#[test]
fn a_drawing_gives_its_models_values() {
    let model = "unit in\nuse std\ng := 1\"\nw := 10\nin std.front {\n  a := point\n  \
                 b := point hint((5, 0))\n  fix((0, 0)) a\n  a level(v) b\n}\n\
                 a distance(max(g, 2in) * w) b\n";
    let compile = |text: &str, set: &[(&str, &str)]| {
        let set: Vec<(String, String)> =
            set.iter().map(|(n, v)| (n.to_string(), v.to_string())).collect();
        drawing::compile_with(text, "drawing.svd", None, &set,
            &mut |path, _| (path == "part.sv").then(|| ("part.sv".into(), model.into())).map(Ok))
    };
    let sheet = "sheet s {\n  sketch a(m) at (0mm, 0mm)\n  measure distance(m.a, m.b) in a\n  \
                 sketch b(n) at (0mm, 100mm)\n  measure distance(n.a, n.b) in b\n}\n";
    let text = format!("model m from \"part.sv\" with g = 3, w=4\nmodel n from \"part.sv\"\n{sheet}");
    let c = compile(&text, &[]).unwrap();
    // g is 3 inches, so the distance is 3 × 4; the other import is the file's, 2 × 10
    assert!(c.svg.contains(">12</text>") && c.svg.contains(">20</text>"), "{}", c.svg);
    let said = |c: &drawing::Compiled| c.rebound.iter().map(|(alias, r)| (alias.clone(),
        r.iter().map(gcs_core::report::rebound_text).collect::<Vec<_>>())).collect::<Vec<_>>();
    assert_eq!(said(&c), [
        ("m".to_string(), vec!["set g := 3\" (the document writes 1\")".to_string(),
                               "set w := 4 (the document writes 10)".to_string()]),
        ("n".to_string(), vec![]),
    ]);
    // the run's value goes to both, the import's own over it
    let c = compile(&text, &[("w", "1"), ("g", "5")]).unwrap();
    assert!(c.svg.contains(">12</text>") && c.svg.contains(">5</text>"), "{}", c.svg);
    assert_eq!(compile(&text, &[("q", "1")]).unwrap_err().message,
        "--set: no model this drawing imports binds a value `q` at its top level");
    let refused = compile(&format!("model m from \"part.sv\" with q = 1\nmodel n from \
        \"part.sv\"\n{sheet}"), &[]).unwrap_err().message;
    assert!(refused.contains("the document binds no value `q`"), "{refused}");
    let twice = drawing::parse("model m from \"part.sv\" with g = 1, g = 2").unwrap_err();
    assert_eq!(twice.message, "`g` is given a value twice");
    // an expression is its words, as far as the next statement
    let doc = drawing::parse("model m from \"part.sv\" with g = max(1, 2) * 3in model n from \
        \"part.sv\"").unwrap();
    assert_eq!(doc.models[0].with[0].text, "max(1, 2) * 3in");
    assert_eq!(doc.models.len(), 2);
}

/// Two discs on one axis, the small one nearer the front view's eye and over the large one's rim.
const PAIR: &str = "\
unit mm
use std
use std.solids
in std.front {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 10)
  radius(10) rim
  p := point
  fix((8, 0)) p
  cap := circle(center: p) hint(r: 4)
  radius(4) cap
}
back := solid(std.solids.Prism(face(rim), from: -20mm, to: -10mm))
front := solid(std.solids.Prism(face(cap), from: -4mm, to: 0mm))
";

fn pair(sheet: &str) -> Result<String, drawing::Error> {
    drawing::compile(&format!("model m from \"pair.sv\"\nstyle .hidden {{ dash: 4 3 }}\n\
        sheet s {{\n{sheet}\n}}\n"), "drawing.svd", None,
        &mut |path, _| (path == "pair.sv").then(|| ("pair.sv".into(), PAIR.into())).map(Ok))
}

/// Every polyline's points in paper pixels, with its path and whether it is dashed.
fn polylines(svg: &str) -> Vec<(String, bool, Vec<(f64, f64)>)> {
    svg.lines().filter(|l| l.starts_with("<polyline")).map(|l| {
        let attr = |k: &str| l.split(&format!("{k}=\"")).nth(1).map(|r| r.split('"').next().unwrap());
        let pts = attr("points").unwrap().split_whitespace().map(|p| {
            let (x, y) = p.split_once(',').unwrap();
            (x.parse().unwrap(), y.parse().unwrap())
        }).collect();
        (attr("data-path").unwrap().to_string(), attr("stroke-dasharray").is_some(), pts)
    }).collect()
}

/// **An assembly is viewed whole** (#940): a view of the model, or of a scope holding objects,
/// projects every object under it in one picture, the part behind another dashed where the
/// nearer one stands in front of it — which a view of the part alone, drawing nothing else, never
/// is.
#[test]
fn an_assembly_view_hides_one_part_behind_another() {
    let whole = polylines(&pair("view f(m) from front at (100mm, 100mm)").unwrap());
    let back = |ls: &[(String, bool, Vec<(f64, f64)>)], dashed: bool| {
        ls.iter().any(|(path, d, _)| path.starts_with("back") && *d == dashed)
    };
    assert!(back(&whole, true) && back(&whole, false), "{whole:?}");
    // the small disc in front is never behind anything
    assert!(whole.iter().filter(|(p, _, _)| p.starts_with("front")).all(|(_, d, _)| !d));
    let alone = polylines(&pair("view f(m.back) from front at (100mm, 100mm)").unwrap());
    assert!(back(&alone, false) && !back(&alone, true), "{alone:?}");
    assert!(alone.iter().all(|(p, _, _)| p.starts_with("back")));
    // a scope holding no object is refused, saying what a solid view takes
    let refused = pair("view f(m.o) from front at (100mm, 100mm)").unwrap_err().message;
    assert!(refused.contains("a solid view names a solid"), "{refused}");
    // seen from the side the two stand apart, and neither hides the other's outline
    let side = polylines(&pair("view r(m) from right at (100mm, 100mm)").unwrap());
    assert!(side.iter().all(|(_, d, _)| !d), "{side:?}");
}

/// **A view is placed by its own box, and fitted to one** (#940): `at topleft (x, y)` puts the box
/// round what the view draws there, whatever its origin; `scale fit (W, H)` is the largest scale
/// that fits it in W by H; an assembly's `dimensions` are its overall extents, written as the
/// sheet's `format` asks.
#[test]
fn a_view_is_placed_and_fitted_by_its_box() {
    let px = 96.0 / 25.4;
    let extent = |svg: &str| {
        let pts: Vec<(f64, f64)> = polylines(svg).into_iter().flat_map(|(_, _, p)| p).collect();
        let lo = pts.iter().fold((f64::INFINITY, f64::INFINITY), |m, p| (m.0.min(p.0), m.1.min(p.1)));
        let hi = pts.iter().fold((f64::NEG_INFINITY, f64::NEG_INFINITY), |m, p| (m.0.max(p.0), m.1.max(p.1)));
        (lo, hi)
    };
    // the box runs from x = -10 to 12 and z = -10 to 10
    let (lo, hi) = extent(&pair("view f(m) from front at topleft (10mm, 20mm)").unwrap());
    assert!((lo.0 - 10.0 * px).abs() < 1e-3 && (lo.1 - 20.0 * px).abs() < 1e-3, "{lo:?}");
    assert!((hi.0 - 32.0 * px).abs() < 1e-3 && (hi.1 - 40.0 * px).abs() < 1e-3, "{hi:?}");
    let (lo, hi) = extent(&pair("view f(m) from front at bottomright (100mm, 100mm)").unwrap());
    assert!((hi.0 - 100.0 * px).abs() < 1e-3 && (hi.1 - 100.0 * px).abs() < 1e-3, "{hi:?}");
    assert!((lo.0 - 78.0 * px).abs() < 1e-3, "{lo:?}");
    let (lo, hi) = extent(&pair("view f(m) from front at center (50mm, 50mm) scale fit (44mm, 80mm)")
        .unwrap());
    // twice: 44 across for 22, the height 40 then within 80
    assert!(((hi.0 - lo.0) - 44.0 * px).abs() < 1e-3 && ((hi.1 - lo.1) - 40.0 * px).abs() < 1e-3);
    assert!((0.5 * (lo.0 + hi.0) - 50.0 * px).abs() < 1e-3);
    let svg = pair("format decimal 2\nview f(m) from front at (100mm, 100mm)\ndimensions in f").unwrap();
    assert!(svg.contains(">22.00</text>") && svg.contains(">20.00</text>"), "{svg}");
    let svg = pair("view f(m) from front at (100mm, 100mm)\nmeasure distance(m.o, m.p) in f")
        .unwrap();
    assert!(svg.contains(">8</text>"), "{svg}");
    for bad in ["format decimal 2.5", "format fraction 3/8", "format lots",
                "view f(m) from front at (0, 0) scale fit (0mm, 10mm)"] {
        assert!(pair(bad).is_err(), "{bad}");
    }
}

/// **A figure is written as the drawing asks** (#940): to six figures, to so many places, or to the
/// nearest fraction — an inch's, marked, in an inch or a foot model.
#[test]
fn a_figure_is_written_as_the_drawing_asks() {
    use drawing::Figures;
    let inch = gcs_core::units::Units::with_length("in").unwrap();
    let foot = gcs_core::units::Units::with_length("ft").unwrap();
    let mm = gcs_core::units::Units::default();
    assert_eq!(Figures::Reading.text(87.98953, inch), "87.9895");
    assert_eq!(Figures::Decimal(2).text(87.98953, inch), "87.99");
    assert_eq!(Figures::Decimal(0).text(87.5, mm), "88");
    assert_eq!(Figures::Decimal(2).text(-0.001, mm), "0.00");
    assert_eq!(Figures::Fraction(8).text(87.98953, inch), "88\"", "to the nearest eighth");
    assert_eq!(Figures::Fraction(8).text(87.87, inch), "87 7/8\"");
    assert_eq!(Figures::Fraction(16).text(87.5, inch), "87 1/2\"");
    assert_eq!(Figures::Fraction(8).text(0.375, inch), "3/8\"");
    assert_eq!(Figures::Fraction(8).text(-2.0, inch), "-2\"");
    assert_eq!(Figures::Fraction(8).text(87.87 / 12.0, foot), "7' 3 7/8\"");
    assert_eq!(Figures::Fraction(8).text(0.5, foot), "6\"");
    assert_eq!(Figures::Fraction(2).text(12.25, mm), "12 1/2");
}

/// **A sketch's dimensions narrowed** (#940): `dimensions m.one in v` calls out the dimensions
/// whose geometry is all under `one`, and not its twin's; a bare `dimensions in v` all of them.
#[test]
fn a_sketchs_dimensions_are_narrowed_to_a_scope() {
    let model = "unit mm\nuse std\ncomponent Bar(f: group, y: Length) {\n  in f.front {\n    \
                 a := point hint((0, y))\n    b := point hint((10, y))\n    fix((0, y)) a\n    \
                 a level(v) b\n  }\n  a distance(13) b\n}\nframe := { front: std.front }\n\
                 one := Bar(frame, y: 0mm)\ntwo := Bar(frame, y: 20mm)\n";
    let compile = |dims: &str| drawing::compile(&format!("model m from \"bars.sv\"\nsheet s {{\n  \
        sketch v(m) at (50mm, 50mm)\n  {dims}\n}}\n"), "drawing.svd", None,
        &mut |path, _| (path == "bars.sv").then(|| ("bars.sv".into(), model.into())).map(Ok));
    let count = |svg: &str| svg.matches(">13</text>").count();
    assert_eq!(count(&compile("dimensions in v").unwrap()), 2);
    assert_eq!(count(&compile("dimensions m.one in v").unwrap()), 1);
    assert_eq!(count(&compile("dimensions m.one, m.two in v").unwrap()), 2);
    assert_eq!(count(&compile("dimensions m.one in v\ndimensions in v").unwrap()), 2);
    assert_eq!(count(&compile("").unwrap()), 0);
    assert!(compile("dimensions m.nope in v").is_err());
}
