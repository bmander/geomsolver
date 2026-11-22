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
    // a plane become a surface swept along a curve with a fixed reference
    let (plane, line) = first(&text, "PLANE");
    let offset = line.replace("PLANE('',", "FIXED_REFERENCE_SWEPT_SURFACE('',").replace(");", ",$,$);");
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
}

/// A cylinder of radius 5 from `z = 0` to `3` written as Parasolid-family writers write it: its
/// side bounded by its two circles and no seam, each circle's vertex where `at` puts it (the
/// angle round the axis of the top's, the bottom's at 0).
fn drum(at: f64) -> String {
    let (c, s) = (5. * at.cos(), 5. * at.sin());
    file(&format!("#14 = CYLINDRICAL_SURFACE('',#13,5.);\n#20 = CARTESIAN_POINT('',(0.,0.,3.));\n\
        #21 = AXIS2_PLACEMENT_3D('',#20,#11,#12);\n#22 = PLANE('',#21);\n#23 = PLANE('',#13);\n\
        #30 = CIRCLE('',#21,5.);\n#34 = CIRCLE('',#13,5.);\n\
        #31 = CARTESIAN_POINT('',({},{},3.));\n#32 = VERTEX_POINT('',#31);\n#33 = EDGE_CURVE('',#32,#32,#30,.T.);\n\
        #35 = CARTESIAN_POINT('',(5.,0.,0.));\n#36 = VERTEX_POINT('',#35);\n#37 = EDGE_CURVE('',#36,#36,#34,.T.);\n\
        #40 = ORIENTED_EDGE('',*,*,#33,.F.);\n#41 = EDGE_LOOP('',(#40));\n#42 = FACE_BOUND('',#41,.T.);\n\
        #38 = ORIENTED_EDGE('',*,*,#37,.T.);\n#39 = EDGE_LOOP('',(#38));\n#48 = FACE_BOUND('',#39,.T.);\n\
        #43 = ADVANCED_FACE('',(#42,#48),#14,.T.);\n#44 = ORIENTED_EDGE('',*,*,#33,.T.);\n#45 = EDGE_LOOP('',(#44));\n\
        #46 = FACE_BOUND('',#45,.T.);\n#47 = ADVANCED_FACE('',(#46),#22,.T.);\n#54 = ORIENTED_EDGE('',*,*,#37,.F.);\n\
        #55 = EDGE_LOOP('',(#54));\n#56 = FACE_BOUND('',#55,.T.);\n#57 = ADVANCED_FACE('',(#56),#23,.F.);\n\
        #50 = CLOSED_SHELL('',(#43,#47,#57));\n#51 = MANIFOLD_SOLID_BREP('',#50);\n", real(c), real(s)))
}

/// **A band between two loops winding round a period with no seam is joined by a seam the reader
/// lays** (the commonest habit of Parasolid-family writers): along the surface at one value of the
/// period, from a vertex of the one loop to a vertex of the other standing there, or — the
/// vertices a quarter turn apart — to the point of the other's edge there, that edge cut into two
/// pieces of its one entity. A drum reads to `π r² h` and `2π r h + 2π r²` either way, every
/// entity of the file read once and the seam named by none; a torus's sector between two tube
/// circles (its band round `v`, the seam laid round the axis) to `π² R r² / 2` and `π² R r + 2π r²`.
#[test]
fn a_band_written_with_no_seam_reads_to_its_closed_form() {
    let pi = std::f64::consts::PI;
    for at in [0., 0.5 * pi] {
        let given = read(&drum(at)).unwrap_or_else(|r| panic!("{at}: {r:?}"));
        let (g, b) = (&given[0], &given[0].brep);
        close(&format!("{at}: the drum's volume"), volume(b), pi * 25. * 3., 1e-9);
        close(&format!("{at}: the drum's area"), total_area(b), 2. * pi * 5. * 3. + 2. * pi * 25., 1e-9);
        let named: std::collections::BTreeSet<u32> = g.edges.iter().copied().filter(|&e| e != 0).collect();
        assert_eq!(named.into_iter().collect::<Vec<_>>(), vec![33, 37], "{at}: the file's edges");
        assert_eq!(g.edges.iter().filter(|&&e| e == 0).count(), 1, "{at}: the seam laid");
    }
    // a quarter of a torus of radii 4 and 1 round the axis, cut by the planes `y = 0` and `x = 0`
    let sector = file("#14 = TOROIDAL_SURFACE('',#13,4.,1.);\n\
        #20 = CARTESIAN_POINT('',(4.,0.,0.));\n#21 = DIRECTION('',(0.,-1.,0.));\n#22 = AXIS2_PLACEMENT_3D('',#20,#21,#12);\n\
        #23 = CIRCLE('',#22,1.);\n#24 = PLANE('',#22);\n#25 = CARTESIAN_POINT('',(5.,0.,0.));\n#26 = VERTEX_POINT('',#25);\n\
        #27 = EDGE_CURVE('',#26,#26,#23,.T.);\n\
        #30 = CARTESIAN_POINT('',(0.,4.,0.));\n#31 = DIRECTION('',(1.,0.,0.));\n#32 = DIRECTION('',(0.,1.,0.));\n\
        #33 = AXIS2_PLACEMENT_3D('',#30,#31,#32);\n#34 = CIRCLE('',#33,1.);\n#35 = PLANE('',#33);\n\
        #36 = CARTESIAN_POINT('',(0.,5.,0.));\n#37 = VERTEX_POINT('',#36);\n#38 = EDGE_CURVE('',#37,#37,#34,.T.);\n\
        #40 = ORIENTED_EDGE('',*,*,#27,.F.);\n#41 = EDGE_LOOP('',(#40));\n#42 = FACE_BOUND('',#41,.T.);\n\
        #43 = ORIENTED_EDGE('',*,*,#38,.T.);\n#44 = EDGE_LOOP('',(#43));\n#45 = FACE_BOUND('',#44,.T.);\n\
        #46 = ADVANCED_FACE('',(#42,#45),#14,.T.);\n\
        #53 = ORIENTED_EDGE('',*,*,#27,.T.);\n#54 = EDGE_LOOP('',(#53));\n#55 = FACE_BOUND('',#54,.T.);\n\
        #56 = ADVANCED_FACE('',(#55),#24,.T.);\n\
        #57 = ORIENTED_EDGE('',*,*,#38,.F.);\n#58 = EDGE_LOOP('',(#57));\n#59 = FACE_BOUND('',#58,.T.);\n\
        #60 = ADVANCED_FACE('',(#59),#35,.F.);\n\
        #50 = CLOSED_SHELL('',(#46,#56,#60));\n#51 = MANIFOLD_SOLID_BREP('',#50);\n");
    let given = read(&sector).unwrap_or_else(|r| panic!("the torus's sector: {r:?}"));
    let b = &given[0].brep;
    close("the sector's volume", volume(b).abs(), pi * pi * 4. / 2., 1e-9);
    close("the sector's area", total_area(b), pi * pi * 4. + 2. * pi, 1e-9);
    assert!(volume(b) > 0., "the sector turned inside out");
}

/// `x` written as STEP writes a real: always with its point.
fn real(x: f64) -> String { let s = format!("{x:?}"); if s.contains('.') || s.contains('e') { s.replace("e", "E") } else { format!("{s}.") } }

/// A point or a direction written.
fn triple(p: V) -> String { format!("({},{},{})", real(p[0]), real(p[1]), real(p[2])) }

/// Two balls' common part, written as a writer of lenses writes it — the larger ball's cap bounded
/// by the circle they meet in alone, the smaller's by the circle and a meridian seam from its south
/// pole to the circle's vertex — the pole `inside` within the larger ball: the circle passing that
/// far from it, its vertex the circle's point nearest it; or, `inside` nought, the circle through
/// the pole and its vertex a micron round from it. The file, its edge on the circle's entity (#34),
/// and the lens's closed volume `Σ π h² (3r − h) / 3` and area `Σ 2π r h`.
fn lens(inside: f64) -> (String, f64, f64) {
    use gcs_core::space::{add, cross, dot, norm, normalised, scale, sub};
    let (ra, rb) = (5., 3.);
    // the smaller ball's centre placed so that its south pole lies `inside` within the larger and
    // its north pole outside it
    let c: V = [4., 2., 3. - ((ra - inside) * (ra - inside) - 20f64).sqrt()];
    let pole = sub(c, [0., 0., rb]);
    let n = normalised(c).unwrap();
    let dc = norm(c);
    let d1 = (dc * dc + ra * ra - rb * rb) / (2. * dc);
    let rho = (ra * ra - d1 * d1).sqrt();
    let centre = scale(n, d1);
    let toward = sub(sub(pole, centre), scale(n, dot(sub(pole, centre), n)));
    let x = normalised(toward).unwrap();
    let y = cross(n, x);
    // the circle's vertex: its point nearest the pole, or a micron round from it
    let t = if inside > 0. { 0. } else { 1e-3 / rho };
    let v = add(centre, add(scale(x, rho * t.cos()), scale(y, rho * t.sin())));
    let m = normalised(cross(sub(pole, c), sub(v, c))).unwrap();
    let px = normalised(sub(pole, c)).unwrap();
    assert!((norm(pole) - (ra - inside)).abs() < 1e-12 && gcs_core::space::distance(pole, v) < 1e-2);
    let text = file(&format!("#14 = SPHERICAL_SURFACE('',#13,{ra});\n\
        #15 = CARTESIAN_POINT('',{});\n#16 = AXIS2_PLACEMENT_3D('',#15,#11,#12);\n#17 = SPHERICAL_SURFACE('',#16,{rb});\n\
        #20 = CARTESIAN_POINT('',{});\n#21 = DIRECTION('',{});\n#22 = DIRECTION('',{});\n#23 = AXIS2_PLACEMENT_3D('',#20,#21,#22);\n\
        #24 = CIRCLE('',#23,{});\n#25 = DIRECTION('',{});\n#26 = DIRECTION('',{});\n#27 = AXIS2_PLACEMENT_3D('',#15,#25,#26);\n\
        #28 = CIRCLE('',#27,{rb});\n#30 = CARTESIAN_POINT('',{});\n#31 = VERTEX_POINT('',#30);\n\
        #32 = CARTESIAN_POINT('',{});\n#33 = VERTEX_POINT('',#32);\n#34 = EDGE_CURVE('',#31,#31,#24,.T.);\n\
        #35 = EDGE_CURVE('',#33,#31,#28,.T.);\n\
        #40 = ORIENTED_EDGE('',*,*,#34,.T.);\n#41 = EDGE_LOOP('',(#40));\n#42 = FACE_BOUND('',#41,.T.);\n\
        #43 = ADVANCED_FACE('',(#42),#14,.T.);\n#44 = ORIENTED_EDGE('',*,*,#35,.T.);\n#45 = ORIENTED_EDGE('',*,*,#34,.F.);\n\
        #46 = ORIENTED_EDGE('',*,*,#35,.F.);\n#47 = EDGE_LOOP('',(#44,#45,#46));\n#48 = FACE_BOUND('',#47,.T.);\n\
        #49 = ADVANCED_FACE('',(#48),#17,.T.);\n#50 = CLOSED_SHELL('',(#43,#49));\n#51 = MANIFOLD_SOLID_BREP('',#50);\n",
        triple(c), triple(centre), triple(n), triple(x), real(rho), triple(m), triple(px), triple(v), triple(pole)));
    let (ha, hb) = (ra - d1, rb - (dc - d1));
    let pi = std::f64::consts::PI;
    (text, pi * ha * ha * (3. * ra - ha) / 3. + pi * hb * hb * (3. * rb - hb) / 3., 2. * pi * (ra * ha + rb * hb))
}

/// **A lens whose circle passes a hair from a sphere's pole reads to its closed form** (#993): the
/// smaller ball's circle sweeps half a turn of its chart's `u` in a hair of its span either side
/// of its vertex, which a fixed rule over its knots misses; the faces are integrated along the
/// parameter a pole collapses (`props`), settled a stretch at a time, and read to `1e-9`. **The
/// same circle through the pole itself, its vertex a micron past it**, has its `u` jump half a turn
/// there, the way that keeps the face on its left (`track`): every point of its walk within half a
/// turn of the line between its ends, which the kernel's inverse curve reads its branch by, it
/// reads to its closed form (its area to `1e-7`, the integrand's corner at the pole read by the
/// adaptive rule) — a walk not within that is refused naming the edge.
#[test]
fn a_lens_whose_circle_passes_a_pole_reads_to_its_closed_form() {
    for inside in [1e-3, 1e-6] {
        let (text, v, a) = lens(inside);
        let given = read(&text).unwrap_or_else(|r| panic!("{inside}: {r:?}"));
        let b = &given[0].brep;
        close(&format!("{inside}: the lens's volume"), volume(b), v, 1e-9);
        close(&format!("{inside}: the lens's area"), total_area(b), a, 1e-9);
    }
    // through the pole itself: the walk's jump there the way that keeps the face on the left, the
    // inverse's branch the walk's (within half a turn of the line between its ends) — read by the
    // topology to the closed form; whether a loop touching itself at the pole is sound is `sound`'s
    let (text, v, a) = lens(0.);
    let file = gcs_core::brep::step_read::context::parse(&text).unwrap();
    let found = gcs_core::brep::step_read::context::solids(&file).unwrap().remove(0);
    let g = gcs_core::brep::step_read::topology::solid(&file, found).unwrap_or_else(|r| panic!("through the pole: {r:?}"));
    close("through the pole: the lens's volume", volume(&g.brep), v, 1e-9);
    close("through the pole: the lens's area", total_area(&g.brep), a, 1e-7);
    if let Ok(given) = read(&text) { close("through the pole: read", volume(&given[0].brep), v, 1e-9); }
}

/// **The value a use arrives at a pole along weighs nothing** (`props`): a ball's faces integrated
/// along the parameter its poles collapse, so that the integral gathered there is nought, the
/// ball's measures are the same with every pole's degenerate edge moved along the pole — a jump in
/// the loop's parameters there, which the loop's closing segments run along — as without. A
/// file's edge standing off its face by its tolerance at a pole leaves the value it arrives along
/// to within that gap; a fifth of a turn here.
#[test]
fn the_value_a_use_arrives_at_a_pole_along_weighs_nothing() {
    use gcs_core::brep::topo::Pcurve;
    let b = sphere([0.5, 0., 0.], [0.3, 0.2, 1.], 1.3).unwrap();
    let mut moved = b.clone();
    let mut jumps = 0;
    for f in &mut moved.faces {
        for c in f.loops.iter_mut().flatten() {
            if !matches!(b.edges[c.edge as usize].curve, EdgeCurve::Degenerate) { continue }
            let Pcurve::Line { a, b: z } = c.pcurve else { continue };
            let k = if (a[0] - z[0]).abs() > (a[1] - z[1]).abs() { 0 } else { 1 };
            let mut z = z;
            z[k] += 0.2 * TAU;
            c.pcurve = Pcurve::Line { a, b: z };
            jumps += 1;
        }
    }
    assert!(jumps > 0, "a ball with no pole edge");
    close("the ball's volume", volume(&moved), volume(&b), 1e-12);
    close("the ball's area", total_area(&moved), total_area(&b), 1e-12);
    close("the ball's volume, its closed form", volume(&b), 4. / 3. * std::f64::consts::PI * 1.3f64.powi(3), 1e-12);
}

/// **A spindle torus's area is the integral of `|R + r cos v|`** (`props`): a torus whose tube
/// crosses its axis (`R < r`, an offset grown past its hole), written whole with its two seams,
/// reads to the volume `2π² R r²` its signed flux gives and the area
/// `2π r ∫|R + r cos v| dv = 4π r (2R s₀ + 2r sin s₀ − πR)`, `cos s₀ = −R/r` — the integrand's
/// sign changes at `s₀`, which a Gauss rule across it reads to some 1e-4.
#[test]
fn a_spindle_torus_reads_to_its_closed_form() {
    let (big, r) = (4., 5.);
    let text = file("#14 = TOROIDAL_SURFACE('',#13,4.,5.);\n#20 = CARTESIAN_POINT('',(9.,0.,0.));\n#21 = VERTEX_POINT('',#20);\n\
        #22 = CARTESIAN_POINT('',(4.,0.,0.));\n#23 = DIRECTION('',(0.,1.,0.));\n#24 = AXIS2_PLACEMENT_3D('',#22,#23,#12);\n\
        #25 = CIRCLE('',#24,5.);\n#26 = EDGE_CURVE('',#21,#21,#25,.T.);\n#27 = CIRCLE('',#13,9.);\n#28 = EDGE_CURVE('',#21,#21,#27,.T.);\n\
        #30 = ORIENTED_EDGE('',*,*,#26,.T.);\n#31 = ORIENTED_EDGE('',*,*,#28,.T.);\n#32 = ORIENTED_EDGE('',*,*,#26,.F.);\n\
        #33 = ORIENTED_EDGE('',*,*,#28,.F.);\n#34 = EDGE_LOOP('',(#30,#31,#32,#33));\n#35 = FACE_OUTER_BOUND('',#34,.T.);\n\
        #36 = ADVANCED_FACE('',(#35),#14,.T.);\n#50 = CLOSED_SHELL('',(#36));\n#51 = MANIFOLD_SOLID_BREP('',#50);\n");
    let given = read(&text).unwrap_or_else(|r| panic!("{r:?}"));
    let b = &given[0].brep;
    let pi = std::f64::consts::PI;
    let s0 = (-big / r as f64).acos();
    close("the spindle's volume", volume(b), 2. * pi * pi * big * r * r, 1e-9);
    close("the spindle's area", total_area(b), 4. * pi * r * (2. * big * s0 + 2. * r * s0.sin() - pi * big), 1e-9);
}

/// **A shell of no face is refused naming it**: it bounds no solid (`Why::Empty`), not one of
/// volume and area nought.
#[test]
fn an_empty_shell_is_refused_naming_it() {
    let r = read(&file("#50 = CLOSED_SHELL('',());\n#51 = MANIFOLD_SOLID_BREP('',#50);\n")).map(|_| ()).expect_err("an empty shell");
    assert_eq!((r.why, r.entity), (Why::Empty, Some(50)), "{r}");
}

/// **A sphere's curve in its face's parameters lying a whole turn of `v` off is read where the
/// face's `v` is** (Open CASCADE writes some so): the sphere's points are its formula's at every
/// whole turn of `v`, its chart's half turn the face's. A ball's cap bounded by two half circles,
/// each with its line in the sphere's parameters, the second's a turn below the face, reads to
/// the cap, `π h² (3R − h) / 3` and `2πRh + π r²`.
#[test]
fn a_spheres_curve_a_turn_off_in_v_reads_where_the_face_is() {
    let v = (3f64 / 5.).asin();
    let text = file(&format!("#14 = SPHERICAL_SURFACE('',#13,5.);\n#20 = CARTESIAN_POINT('',(0.,0.,3.));\n\
        #21 = AXIS2_PLACEMENT_3D('',#20,#11,#12);\n#22 = PLANE('',#21);\n#30 = CIRCLE('',#21,4.);\n\
        #31 = CARTESIAN_POINT('',(4.,0.,3.));\n#32 = VERTEX_POINT('',#31);\n#24 = CARTESIAN_POINT('',(-4.,0.,3.));\n\
        #25 = VERTEX_POINT('',#24);\n#60 = DIRECTION('',(1.,0.));\n#61 = VECTOR('',#60,1.);\n\
        #62 = CARTESIAN_POINT('',(0.,{}));\n#63 = LINE('',#62,#61);\n#64 = DEFINITIONAL_REPRESENTATION('',(#63),#4);\n\
        #65 = PCURVE('',#14,#64);\n#66 = SURFACE_CURVE('',#30,(#65),.PCURVE_S1.);\n\
        #67 = CARTESIAN_POINT('',({},{}));\n#68 = LINE('',#67,#61);\n#69 = DEFINITIONAL_REPRESENTATION('',(#68),#4);\n\
        #70 = PCURVE('',#14,#69);\n#71 = SURFACE_CURVE('',#30,(#70),.PCURVE_S1.);\n\
        #33 = EDGE_CURVE('',#32,#25,#66,.T.);\n#34 = EDGE_CURVE('',#25,#32,#71,.T.);\n\
        #40 = ORIENTED_EDGE('',*,*,#33,.T.);\n#41 = ORIENTED_EDGE('',*,*,#34,.T.);\n#35 = EDGE_LOOP('',(#40,#41));\n\
        #42 = FACE_BOUND('',#35,.T.);\n#43 = ADVANCED_FACE('',(#42),#14,.T.);\n#44 = ORIENTED_EDGE('',*,*,#34,.F.);\n\
        #36 = ORIENTED_EDGE('',*,*,#33,.F.);\n#45 = EDGE_LOOP('',(#44,#36));\n\
        #46 = FACE_BOUND('',#45,.T.);\n#47 = ADVANCED_FACE('',(#46),#22,.F.);\n#50 = CLOSED_SHELL('',(#43,#47));\n\
        #51 = MANIFOLD_SOLID_BREP('',#50);\n", real(v), real(0.), real(v - TAU)));
    let given = read(&text).unwrap_or_else(|r| panic!("{r:?}"));
    let b = &given[0].brep;
    let (r, h) = (5., 2.);
    close("the cap's volume", volume(b), std::f64::consts::PI * h * h * (3. * r - h) / 3., 1e-9);
    close("the cap's area", total_area(b), std::f64::consts::PI * (2. * r * h + 16.), 1e-9);
}
