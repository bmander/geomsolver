//! **What the kernel writes is in the standard's parameters** (docs/contract/brep.md, #993 stage
//! 1): every curve in a face's parameters (`PCURVE`) the file writes lies on its edge's curve at the
//! edge's own parameter, read as ISO 10303-42 defines the surface's parameters — a cone's `v` along
//! its axis, not its generator — and a net's (an affine image's) curve written as a line only where
//! it is one; every surface it writes within the standard's own rules (a torus's radii positive, a
//! spindle torus's lemon a degenerate torus). Measured on the file alone, by the reader's
//! evaluation of each entity, which knows nothing of how the writer chose its numbers.
use gcs_core::brep::affine::Affine;
use gcs_core::brep::boolean::{boolean, Op};
use gcs_core::brep::build::{cone, ellipsoid, prism, revolve, disc, sphere, torus, Profile, ProfileEdge};
use gcs_core::brep::geom::{Curve, Frame};
use gcs_core::brep::props::{area, volume};
use gcs_core::brep::step::{write, Fit, EXACT, FIT};
use gcs_core::brep::step_check::{verify, Solid};
use gcs_core::brep::step_read::context::{parse, solids as represented};
use gcs_core::brep::step_read::geometry::Geometry;
use gcs_core::brep::step_read::read;
use gcs_core::brep::topo::Brep;
use gcs_core::space::distance;
use std::f64::consts::TAU;

/// How far, over the file, a pcurve's image on its face's surface stands from its edge's curve at
/// the same parameter: each `EDGE_CURVE`'s span read from its vertices on its curve (a whole turn
/// where it starts and ends at one vertex), and each of its pcurves read at 64 points of it. Faces
/// whose surface the reader holds only provisionally (a revolution or an extrusion seated as a net)
/// are passed over: their parameters are not the file's.
fn pcurves_off(text: &str) -> (f64, usize) {
    let file = parse(text).unwrap();
    let found = represented(&file).unwrap();
    let g = Geometry { file: &file, context: found[0].context };
    let (mut worst, mut read) = (0f64, 0);
    for e in &file.entities {
        let Some(("EDGE_CURVE", v)) = e.simple() else { continue };
        let vertex = |k: usize| {
            let r = v[k].reference().unwrap();
            let Some(("VERTEX_POINT", p)) = file.get(r).simple() else { panic!("#{r}: a vertex") };
            g.point(r, p[1].reference()).unwrap()
        };
        let closed = v[1].reference() == v[2].reference();
        let (a, z) = (vertex(1), vertex(2));
        let Some((_, sc)) = file.get(v[3].reference().unwrap()).simple() else { panic!("#{}: a surface curve", e.id) };
        let spatial = g.curve(e.id, sc[1].reference()).unwrap();
        let (c, rate) = (&spatial.curve, spatial.rate);
        let t0 = c.inverse(a);
        let t1 = if closed {
            match c { Curve::BSpline(b) => { let d = b.domain(); t0 + d[1] - d[0] } _ => t0 + TAU }
        } else {
            let t = c.inverse(z);
            if t <= t0 && matches!(c, Curve::Circle(..) | Curve::Ellipse(..)) { t + TAU } else { t }
        };
        let pcs = refs(&sc[2]);
        for pc in pcs {
            let Some(("PCURVE", p)) = file.get(pc).simple() else { panic!("#{pc}: a pcurve") };
            let sheet = g.surface(pc, p[1].reference()).unwrap();
            if sheet.seat.is_some() { continue }
            let rep = p[2].reference().unwrap();
            let Some(("DEFINITIONAL_REPRESENTATION", d)) = file.get(rep).simple() else { panic!("#{rep}") };
            let items = refs(&d[1]);
            let flat = g.flat(rep, Some(items[0])).unwrap().scaled(sheet.scale).expect("a pcurve scaled");
            read += 1;
            for k in 0..=64 {
                let t = t0 + (t1 - t0) * k as f64 / 64.;
                let off = distance(sheet.surface.point(flat.at(t / rate)), c.point(t));
                worst = worst.max(off);
            }
        }
    }
    (worst, read)
}

/// A parameter list's references.
fn refs(v: &gcs_core::brep::step_file::Value) -> Vec<u32> {
    v.list().unwrap_or_else(|| panic!("a list: {v:?}")).iter().filter_map(|x| x.reference()).collect()
}

/// The lemon: a spindle torus's inner part, its one arc about a centre 3 behind the axis, of
/// radius 5, from its point below to its point above — the kernel's torus of major radius −3.
fn lemon() -> Brep {
    let half = 0.8f64.atan2(0.6);
    let arc = ProfileEdge::Arc { frame: Frame::new([-3., 0., 0.], [0., -1., 0.], [1., 0., 0.]), r: 5., span: Some([-half, half]) };
    let axis = ProfileEdge::Line { a: [0., 0., 4.], b: [0., 0., -4.] };
    revolve(&Profile { origin: [0., 0., -4.], normal: [0., -1., 0.], names: vec![], loops: vec![vec![arc, axis]] }, [0.; 3], [0., 0., 1.], TAU)
        .unwrap()
}

/// The solids each kind of face and edge is written for: cones each way and to their apex, the
/// conics' surfaces, the lemon, and nets of affine images; a cone drilled, its traced edges.
fn solids() -> Vec<(&'static str, Brep)> {
    let tilted = [0.2, -0.3, 1.];
    let shear = Affine::new([[1., 0.4, 0.2], [0., 1., -0.3], [0.1, 0., 1.]], [1., -2., 0.5]).unwrap();
    let frustum = cone([0., 0., 0.], [0., 0., 1.], 20., 8., 30.).unwrap();
    let rod = prism(&disc([-40., 3., 10.], [1., 0., 0.2], 4.), 0., 80.).unwrap();
    vec![
        ("frustum", cone([0., 0., 0.], tilted, 1.5, 0.6, 2.).unwrap()),
        ("cone opening against its axis", cone([0., 0., 0.], tilted, 0.6, 1.5, 2.).unwrap()),
        ("cone with its apex", cone([0., 0., 0.], tilted, 1.5, 0., 2.).unwrap()),
        ("cylinder", cone([0., 0., 0.], tilted, 1.2, 1.2, 3.).unwrap()),
        ("sphere", sphere([0.5, 0., 0.], tilted, 1.3).unwrap()),
        ("torus", torus([0., 0., 0.], tilted, 3., 1.).unwrap()),
        ("lemon", lemon()),
        ("drilled frustum", boolean(&frustum, &rod, Op::Cut, 1e-9).unwrap()),
        ("sheared torus", torus([0., 0., 0.], [0., 0., 1.], 3., 1.).unwrap().mapped(&shear)),
        ("sheared frustum", cone([0., 0., 0.], tilted, 1.5, 0.6, 2.).unwrap().mapped(&shear)),
        ("ellipsoid", ellipsoid([0.1, 0.2, 0.3], tilted, [1., 0., 0.], [2., 1.2, 0.7]).unwrap()),
    ]
}

/// **Every pcurve lies on its edge at the edge's parameter**, as the standard reads the face's
/// surface: within the fit an edge's curve is written to (`FIT`, a fitted edge's own; a traced
/// one's and each fitted pcurve's `EXACT`) and twice `EXACT` beside — where a cone's pcurves ran
/// along its generator they stood a tenth of a millimetre off on the frustum, and a sheared
/// torus's lines in its net's arc parameters 0.075 mm off between the eighths of a turn the
/// writer's test of a line looked at.
#[test]
fn every_pcurve_lies_on_its_edge_at_the_edges_parameter() {
    let mut off_edges = Vec::new();
    for (name, b) in solids() {
        let text = write(&b, name, 1e-7, Fit::Exact).unwrap();
        verify(&text, &Solid::of(&b)).unwrap_or_else(|e| panic!("{name}: {e}"));
        let (off, read) = pcurves_off(&text);
        eprintln!("{name}: {read} pcurves, the farthest {off:e} off its edge");
        assert!(read > 0, "{name}: no pcurve read");
        if !(off <= FIT + 2. * EXACT) { off_edges.push(format!("{name} {off:e}")); }
    }
    assert!(off_edges.is_empty(), "a pcurve off its edge: {}", off_edges.join(", "));
}

/// **A surface is written within the standard's rules**: a toroidal surface's radii are positive
/// lengths, so the lemon — the kernel's torus of major radius −3 — is written as the degenerate
/// torus of major radius 3 and minor 5 whose inner part it is (`select_outer` false), its face's
/// sense turned with its `v`; and it reads back to the same solid.
#[test]
fn a_lemon_is_written_as_a_degenerate_torus() {
    let b = lemon();
    let text = write(&b, "lemon", 1e-7, Fit::Exact).unwrap();
    assert!(text.contains("DEGENERATE_TOROIDAL_SURFACE('',#"), "the lemon's surface");
    assert!(text.contains(",3.0,5.0,.F.)"), "of radii 3 and 5, its inner part: {text}");
    assert!(!text.lines().any(|l| l.contains(" = TOROIDAL_SURFACE(")), "no toroidal surface");
    verify(&text, &Solid::of(&b)).unwrap();
    let given = read(&text).unwrap();
    let r = &given[0].brep;
    let total = |b: &Brep| b.faces.iter().map(|f| area(b, f)).sum::<f64>();
    assert!((volume(r) - volume(&b)).abs() <= 1e-12 * volume(&b), "{} read, {} built", volume(r), volume(&b));
    assert!((total(r) - total(&b)).abs() <= 1e-12 * total(&b), "{} read, {} built", total(r), total(&b));
}
