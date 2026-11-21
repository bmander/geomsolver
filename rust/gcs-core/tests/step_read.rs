//! **The kernel reads STEP** (`brep::step_read`; #993 stage 1): every file the kernel writes
//! reads back to the same solid, each face on its own surface and each edge on its own curve.
//! What the reader does not read is refused naming the entity, never read wrong.
use gcs_core::brep::boolean::{combined, Op};
use gcs_core::brep::build::{cone, disc, ellipsoid, prism, revolve, sphere, square, torus, Profile, ProfileEdge};
use gcs_core::brep::affine::Affine;
use gcs_core::brep::geom::{Curve, V};
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::props::{area, volume};
use gcs_core::brep::step::{write, Fit};
use gcs_core::brep::step_check::{verify, Solid};
use gcs_core::brep::step_read::{read, Refusal, Why};
use gcs_core::brep::topo::{Brep, EdgeCurve};
use std::f64::consts::{FRAC_1_SQRT_2, TAU};
use std::sync::Arc;

const Z: V = [0., 0., 1.];

/// Every solid of the round trip: its name, its B-rep, and how near its read volume and area
/// must come to its own, relatively. Where every curve the file carries is exact (a line, a conic,
/// a spline's own poles) the two are the kernel's own measure of one solid, read in another
/// order: 1e-9. Where the writer fits a curve (a traced meeting's cubic, held to `step::EXACT`, a
/// thousandth of a micron; a loft's net to `step::FIT`) the face it trims moves by as much along
/// its edges: 1e-7 of the solid's measures is some hundred times what a micron's strip along every
/// edge of these could move them.
fn solids() -> Vec<(&'static str, Brep, f64)> {
    let tilted = [0.2, -0.3, 1.];
    let block = prism(&square([0., 0., 0.], Z, 5.), 0., 10.).unwrap();
    let pin = prism(&disc([0., 0., -1.], Z, 3.), 0., 12.).unwrap();
    let along_x = prism(&disc([-15., 0., 0.], [1., 0., 0.], 5.), 0., 30.).unwrap();
    let across = prism(&disc([0., -15., 1.], [0., 1., 0.], 3.), 0., 30.).unwrap();
    let ball = sphere([0., 0., 0.], [0.3, 0.2, 1.], 5.).unwrap();
    let above = prism(&square([0., 0., 0.], Z, 10.), 0., 10.).unwrap();
    // a half ellipse from pole to pole, two rational quarters, turned once: a revolution
    let (south, north): (V, V) = ([0., 0., -1.], [0., 0., 1.]);
    let arc = BSpline::rational(2, vec![0., 0., 0., 0.5, 0.5, 1., 1., 1.], vec![south, [2., 0., -1.], [2., 0., 0.], [2., 0., 1.], north],
        vec![1., FRAC_1_SQRT_2, 1., FRAC_1_SQRT_2, 1.]).unwrap();
    let half = Profile { origin: south, normal: [0., -1., 0.], names: vec![],
        loops: vec![vec![ProfileEdge::Spline(Arc::new(arc)), ProfileEdge::Line { a: north, b: south }]] };
    // a cubic bump closed by a line: an extrusion
    let bump = BSpline::new(3, vec![0., 0., 0., 0., 1., 1., 1., 1.], vec![[2., 0., 0.], [2., 1.5, 0.], [0., 1.5, 0.], [0., 0., 0.]]).unwrap();
    let lobe = Profile { origin: [0.; 3], normal: Z, names: vec![],
        loops: vec![vec![ProfileEdge::Line { a: [0., 0., 0.], b: [2., 0., 0.] }, ProfileEdge::Spline(Arc::new(bump))]] };
    let shear = Affine::new([[1., 0.4, 0.2], [0., 1., -0.3], [0.1, 0., 1.]], [1., -2., 0.5]).unwrap();
    // a box with an edge rounded: the round's cylinder and its ends
    let boxed = prism(&square([0., 0., 0.], Z, 2.), 0., 3.).unwrap();
    let round = gcs_core::brep::fillet::roll(&boxed, [2., 2., 1.5], 0.5, 1e-9).unwrap();
    let rounded = combined(&boxed, &round.piece, if round.concave { Op::Union } else { Op::Cut }, 0.).unwrap();
    let fitted = 1e-7;
    vec![
        ("block", block.clone(), 1e-9),
        ("block drilled", combined(&block, &pin, Op::Cut, 0.).unwrap(), 1e-9),
        ("cylinder", cone([0., 0., 0.], tilted, 1.2, 1.2, 3.).unwrap(), 1e-9),
        ("frustum", cone([0., 0., 0.], tilted, 1.5, 0.6, 2.).unwrap(), 1e-9),
        ("cone with its apex", cone([0., 0., 0.], tilted, 1.5, 0., 2.).unwrap(), 1e-9),
        ("cone opening against its axis", cone([0., 0., 0.], tilted, 0.6, 1.5, 2.).unwrap(), 1e-9),
        ("sphere", sphere([0.5, 0., 0.], tilted, 1.3).unwrap(), 1e-9),
        ("hemisphere", combined(&ball, &above, Op::Common, 0.).unwrap(), 1e-9),
        ("whole torus", torus([0., 0., 0.], tilted, 3., 1.).unwrap(), 1e-9),
        ("spheroid of revolution", revolve(&half, [0.; 3], Z, TAU).unwrap(), 1e-9),
        ("extruded spline", prism(&lobe, 0., 1.).unwrap(), 1e-9),
        ("crossing cylinders", combined(&along_x, &across, Op::Union, 0.).unwrap(), fitted),
        ("ellipsoid", ellipsoid([0.1, 0.2, 0.3], tilted, [1., 0., 0.], [2., 1.2, 0.7]).unwrap(), fitted),
        ("sheared torus", torus([0., 0., 0.], Z, 3., 1.).unwrap().mapped(&shear), fitted),
        ("rounded box", rounded, fitted),
    ]
}

fn total_area(b: &Brep) -> f64 { b.faces.iter().map(|f| area(b, f)).sum() }

fn close(what: &str, a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs().max(1.), "{what}: {a} read, {b} built ({:e})", (a - b) / b.abs().max(1.));
}

/// The kind a written edge's curve is read as: a line, a conic and a spline as themselves, any
/// other curve as the spline the writer fits it with.
fn written_kind(c: &EdgeCurve) -> &'static str {
    match c { EdgeCurve::Curve(Curve::Line { .. }) => "line", EdgeCurve::Curve(Curve::Circle(..)) => "circle",
        EdgeCurve::Curve(Curve::Ellipse(..)) => "ellipse", EdgeCurve::Degenerate => "degenerate", _ => "bspline" }
}

/// **Every solid the kernel writes reads back to itself**: one solid, as many faces each on a
/// surface of the kind and numbers written and of the same sense (`step_check::Solid`), as many
/// edges each on a curve of the kind written and lying on one of the solid's, the boundary checked
/// (`read` holds it to `Brep::check`), its volume and area the solid's, and the solid read written
/// again a file `step_check::verify` holds to it.
#[test]
fn every_solid_the_kernel_writes_reads_back_to_itself() {
    for (name, b, rel) in solids() {
        let text = write(&b, name, 1e-7, Fit::Exact).unwrap_or_else(|e| panic!("{name}: {e}"));
        let given = read(&text).unwrap_or_else(|r| panic!("{name}: refused {r:?}"));
        assert_eq!(given.len(), 1, "{name}");
        let g = &given[0];
        let r = &g.brep;
        assert_eq!(g.faces.len(), r.faces.len(), "{name}");
        let (written, got) = (Solid::of(&b), Solid::of(r));
        assert_eq!((got.edges, got.vertices, got.faces.len()), (written.edges, written.vertices, written.faces.len()), "{name}: counts");
        for (k, (x, y)) in got.faces.iter().zip(&written.faces).enumerate() {
            assert_eq!((x.kind, x.reversed), (y.kind, y.reversed), "{name}: face {k}'s kind and sense");
            // (a swept surface's frame is the reader's own, its curve the one written)
            if x.kind == 7 || x.kind == 8 { continue }
            assert_eq!(x.numbers.len(), y.numbers.len(), "{name}: face {k}");
            for (p, q) in x.numbers.iter().zip(&y.numbers) { close(&format!("{name}: face {k}'s numbers"), *p, *q, 1e-12); }
        }
        // each edge on a curve of the kind written, on one of the solid's edges
        let mut kinds: Vec<&str> = r.edges.iter().map(|e| written_kind(&e.curve)).filter(|&k| k != "degenerate").collect();
        let mut built: Vec<&str> = b.edges.iter().map(|e| written_kind(&e.curve)).filter(|&k| k != "degenerate").collect();
        kinds.sort();
        built.sort();
        assert_eq!(kinds, built, "{name}: the edges' curves");
        for e in &r.edges {
            let EdgeCurve::Curve(c) = &e.curve else { continue };
            let m = c.point(0.5 * (e.t[0] + e.t[1]));
            let off = b.edges.iter().filter_map(|o| match &o.curve { EdgeCurve::Curve(oc) => Some(oc), _ => None })
                .map(|oc| gcs_core::space::distance(oc.point(oc.inverse(m)), m)).fold(f64::INFINITY, f64::min);
            assert!(off <= 1e-6, "{name}: an edge's middle {m:?} is {off:e} off the solid's edges");
        }
        close(&format!("{name}: volume"), volume(r), volume(&b), rel);
        close(&format!("{name}: area"), total_area(r), total_area(&b), rel);
        // the solid read, written again, is the file of itself
        let again = write(r, name, 1e-7, Fit::Exact).unwrap_or_else(|e| panic!("{name}: written again: {e}"));
        verify(&again, &Solid::of(r)).unwrap_or_else(|e| panic!("{name}: written again: {e}"));
    }
}

/// The block drilled, written.
fn drilled() -> (Brep, String) {
    let block = prism(&square([0., 0., 0.], Z, 5.), 0., 10.).unwrap();
    let pin = prism(&disc([0., 0., -1.], Z, 3.), 0., 12.).unwrap();
    let b = combined(&block, &pin, Op::Cut, 0.).unwrap();
    let text = write(&b, "drilled", 1e-7, Fit::Exact).unwrap();
    (b, text)
}

/// The number of the first entity of `kind` in `text`, and its line.
fn first(text: &str, kind: &str) -> (u32, String) {
    let line = text.lines().find(|l| l.split(" = ").nth(1).is_some_and(|r| r.starts_with(&format!("{kind}(")))).unwrap();
    (line[1..line.find(' ').unwrap()].parse().unwrap(), line.to_string())
}

/// **A file's units are read, not assumed**: the same file in centimetres is the solid ten times
/// the size.
#[test]
fn a_files_unit_of_length_scales_what_it_reads() {
    let (b, text) = drilled();
    let cm = text.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT(.CENTI.,.METRE.)");
    let given = read(&cm).unwrap();
    let r = &given[0].brep;
    close("volume in centimetres", volume(r), 1e3 * volume(&b), 1e-9);
    close("area in centimetres", total_area(r), 1e2 * total_area(&b), 1e-9);
}

/// **What is not read is refused naming the entity**: a surface of a kind the kernel holds no form
/// of, a face missing from the shell (open), a reference to no entity and one to an entity of the
/// wrong kind (malformed) — each with its `Why` and the file's own `#n`.
#[test]
fn what_is_not_read_is_refused_naming_the_entity() {
    let (_, text) = drilled();
    let refused = |t: &str| -> Refusal { read(t).map(|_| ()).expect_err("refused") };
    // a plane become an offset surface
    let (plane, line) = first(&text, "PLANE");
    let offset = line.replace("PLANE('',", "OFFSET_SURFACE('',").replace(");", ",1.,.F.);");
    let r = refused(&text.replace(&line, &offset));
    assert_eq!((r.why, r.entity), (Why::Unsupported, Some(plane)), "{r}");
    // the shell less its first face: open, at an edge of that face
    let (_, shell) = first(&text, "CLOSED_SHELL");
    let at = shell.find(",(#").unwrap() + 2;
    let open = format!("{}{}", &shell[..at], &shell[at + shell[at..].find(',').unwrap() + 1..]);
    let r = refused(&text.replace(&shell, &open));
    assert_eq!(r.why, Why::Open, "{r}");
    let witness = r.entity.unwrap();
    assert!(text.lines().any(|l| l.starts_with(&format!("#{witness} = EDGE_CURVE("))), "{r}");
    // an oriented edge's edge a reference to no entity, then to a point
    let (oriented, line) = first(&text, "ORIENTED_EDGE");
    let edge = line.split(",#").nth(1).unwrap().split(',').next().unwrap().to_string();
    let r = refused(&text.replace(&line, &line.replace(&format!(",#{edge},"), ",#999999,")));
    assert_eq!((r.why, r.entity), (Why::Malformed, Some(oriented)), "{r}");
    let (point, _) = first(&text, "CARTESIAN_POINT");
    let r = refused(&text.replace(&line, &line.replace(&format!(",#{edge},"), &format!(",#{point},"))));
    assert_eq!((r.why, r.entity), (Why::Malformed, Some(oriented)), "{r}");
}

/// A file another writer might write: no curve in any face's parameters, no seam; `data` its
/// entities after the units and the context (`#1` to `#4`), its solid `#51` represented in `#4`.
fn file(data: &str) -> String {
    format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\n\
        FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n\
        #1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
        #2 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
        #3 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','');\n\
        #4 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#3)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2)) REPRESENTATION_CONTEXT('','') );\n\
        #10 = CARTESIAN_POINT('',(0.,0.,0.));\n#11 = DIRECTION('',(0.,0.,1.));\n#12 = DIRECTION('',(1.,0.,0.));\n\
        #13 = AXIS2_PLACEMENT_3D('',#10,#11,#12);\n{data}\
        #52 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#13,#51),#4);\nENDSEC;\nEND-ISO-10303-21;\n")
}

/// **A face holding a pole inside it is joined to it by a seam the reader lays**, and a file that
/// gives no curve in any face's parameters is read by walking its loops: a ball's cap, a sphere's
/// face bounded by one circle round its pole and a plane's by the same circle, read to its closed
/// form, `π h² (3R − h) / 3` and `2πRh + π r²`.
#[test]
fn a_cap_written_with_no_seam_reads_to_its_closed_form() {
    let text = file("#14 = SPHERICAL_SURFACE('',#13,5.);\n#20 = CARTESIAN_POINT('',(0.,0.,3.));\n\
        #21 = AXIS2_PLACEMENT_3D('',#20,#11,#12);\n#22 = PLANE('',#21);\n#30 = CIRCLE('',#21,4.);\n\
        #31 = CARTESIAN_POINT('',(4.,0.,3.));\n#32 = VERTEX_POINT('',#31);\n#33 = EDGE_CURVE('',#32,#32,#30,.T.);\n\
        #40 = ORIENTED_EDGE('',*,*,#33,.T.);\n#41 = EDGE_LOOP('',(#40));\n#42 = FACE_BOUND('',#41,.T.);\n\
        #43 = ADVANCED_FACE('',(#42),#14,.T.);\n#44 = ORIENTED_EDGE('',*,*,#33,.F.);\n#45 = EDGE_LOOP('',(#44));\n\
        #46 = FACE_BOUND('',#45,.T.);\n#47 = ADVANCED_FACE('',(#46),#22,.F.);\n#50 = CLOSED_SHELL('',(#43,#47));\n\
        #51 = MANIFOLD_SOLID_BREP('',#50);\n");
    let given = read(&text).unwrap_or_else(|r| panic!("{r:?}"));
    let b = &given[0].brep;
    let (r, h) = (5., 2.);
    close("the cap's volume", volume(b), std::f64::consts::PI * h * h * (3. * r - h) / 3., 1e-9);
    close("the cap's area", total_area(b), std::f64::consts::PI * (2. * r * h + 16.), 1e-9);
    // the seam and the pole it laid named by no entity of the file's
    assert_eq!(given[0].edges.iter().filter(|&&e| e == 0).count(), b.edges.len() - 1);
    // a cylinder's band between two circles, no seam between them: not read yet, said so
    let band = file("#14 = CYLINDRICAL_SURFACE('',#13,5.);\n#20 = CARTESIAN_POINT('',(0.,0.,3.));\n\
        #21 = AXIS2_PLACEMENT_3D('',#20,#11,#12);\n#22 = PLANE('',#21);\n#23 = PLANE('',#13);\n\
        #30 = CIRCLE('',#21,5.);\n#34 = CIRCLE('',#13,5.);\n\
        #31 = CARTESIAN_POINT('',(5.,0.,3.));\n#32 = VERTEX_POINT('',#31);\n#33 = EDGE_CURVE('',#32,#32,#30,.T.);\n\
        #35 = CARTESIAN_POINT('',(5.,0.,0.));\n#36 = VERTEX_POINT('',#35);\n#37 = EDGE_CURVE('',#36,#36,#34,.T.);\n\
        #40 = ORIENTED_EDGE('',*,*,#33,.F.);\n#41 = EDGE_LOOP('',(#40));\n#42 = FACE_BOUND('',#41,.T.);\n\
        #38 = ORIENTED_EDGE('',*,*,#37,.T.);\n#39 = EDGE_LOOP('',(#38));\n#48 = FACE_BOUND('',#39,.T.);\n\
        #43 = ADVANCED_FACE('',(#42,#48),#14,.T.);\n#44 = ORIENTED_EDGE('',*,*,#33,.T.);\n#45 = EDGE_LOOP('',(#44));\n\
        #46 = FACE_BOUND('',#45,.T.);\n#47 = ADVANCED_FACE('',(#46),#22,.T.);\n#54 = ORIENTED_EDGE('',*,*,#37,.F.);\n\
        #55 = EDGE_LOOP('',(#54));\n#56 = FACE_BOUND('',#55,.T.);\n#57 = ADVANCED_FACE('',(#56),#23,.F.);\n\
        #50 = CLOSED_SHELL('',(#43,#47,#57));\n#51 = MANIFOLD_SOLID_BREP('',#50);\n");
    let r = read(&band).map(|_| ()).expect_err("a band with no seam");
    assert_eq!((r.why, r.entity), (Why::Unsupported, Some(43)), "{r}");
}
