//! **The kernel: a solid is a term, and every answer is a classification against it.**
//!
//! The numbers here are arithmetic a reader can do — a block is `w · h · d`, a block less a bore
//! is that less the faceted circle's own area times the depth — because a kernel checked against
//! another kernel is two implementations agreeing and a kernel checked against arithmetic is one
//! that is right.  The faceted area is *exact*: a tessellated circle is a polygon, and its area
//! is a closed form, so the comparison has no tolerance to argue about beyond the solve's.

use gcs_core::mesh;
use gcs_core::model::{EntRef, Extent, Sense, Sketch, SolidDef};
use gcs_core::solid;

const UNIT: f64 = solid::REPORT_UNIT;

/// A closed loop of lines through the given page points, as a face.
fn poly_face(sk: &mut Sketch, pts: &[(f64, f64)], name: &str) -> usize {
    let ids: Vec<usize> = pts.iter().map(|&(x, y)| sk.point(x, y, false, "")).collect();
    let mut edges = Vec::new();
    let mut names = Vec::new();
    for i in 0..ids.len() {
        let j = (i + 1) % ids.len();
        edges.push(EntRef::line(sk.line(ids[i], ids[j])));
        names.push(format!("e{i}"));
    }
    sk.face(edges, names, name)
}

fn rect_face(sk: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64, name: &str) -> usize {
    poly_face(sk, &[(x0, y0), (x1, y0), (x1, y1), (x0, y1)], name)
}

fn circle_face(sk: &mut Sketch, c: (f64, f64), r: f64, name: &str) -> usize {
    let ctr = sk.point(c.0, c.1, false, "");
    let ci = sk.circle(ctr, r, "");
    sk.face(vec![EntRef::circle(ci)], vec!["rim".into()], name)
}

fn prism(sk: &mut Sketch, face: usize, from: f64, to: f64, name: &str) -> usize {
    sk.solid(
        SolidDef::Prism { face: face as u32, from: Extent::at(from), to: Extent::at(to) },
        name,
    )
}

fn volume(sk: &Sketch, si: usize) -> f64 {
    mesh::volume(&sk.solid_boundary(si, UNIT))
}

/// The area of the polygon a circle of radius `r` is tessellated into at `unit` — the exact
/// number a faceted bore removes, so the test compares one closed form against another.
fn facet_area(r: f64, unit: f64) -> f64 {
    let tol = gcs_core::curve::flatness(unit);
    let step = if r > tol { 2.0 * (1.0 - tol / r).acos() } else { std::f64::consts::TAU };
    let n = ((std::f64::consts::TAU / step).ceil() as usize).clamp(2, 4096) as f64;
    0.5 * n * r * r * (std::f64::consts::TAU / n).sin()
}

#[test]
fn a_block_is_its_own_arithmetic() {
    let mut sk = Sketch::new();
    let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
    let b = prism(&mut sk, f, -30.0, 0.0, "block");
    assert!(
        (volume(&sk, b) - 60.0 * 40.0 * 30.0).abs() < 1e-9,
        "a 60 × 40 face swept 30 is 72000, and got {}",
        volume(&sk, b)
    );
    let bb = mesh::bounds(&sk.solid_boundary(b, UNIT));
    // the page is the front view: u = x, v = z, n = −y, so a prism runs along −n = +y
    assert!((bb.hi[0] - 60.0).abs() < 1e-9 && (bb.hi[2] - 40.0).abs() < 1e-9);
    assert!((bb.hi[1] - bb.lo[1] - 30.0).abs() < 1e-9, "thirty deep");
}

#[test]
fn a_bore_takes_exactly_the_polygon_it_is_faceted_into() {
    let mut sk = Sketch::new();
    let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
    let block = prism(&mut sk, f, -30.0, 0.0, "block");
    let bf = circle_face(&mut sk, (30.0, 20.0), 5.0, "bore_f");
    let bore = prism(&mut sk, bf, -30.0, 0.0, "bore");
    let body = sk.solid(
        SolidDef::Body { stock: block as u32, on: vec![], through: vec![bore as u32] },
        "body",
    );
    let want = (60.0 * 40.0 - facet_area(5.0, UNIT)) * 30.0;
    let got = volume(&sk, body);
    assert!((got - want).abs() < 1e-6 * want.abs(), "block less a through bore: want {want}, got {got}");
    assert!(facet_area(5.0, UNIT) <= std::f64::consts::PI * 25.0, "a facet is inside the circle");
}

#[test]
fn a_flush_bore_and_one_drilled_past_are_the_same_solid() {
    // the coplanar case, and the one a tolerant kernel gets wrong: the bore's cap lands exactly
    // on the block's own face, so a sliver either survives as a face or eats one
    let make = |depth: f64| {
        let mut sk = Sketch::new();
        let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
        let block = prism(&mut sk, f, -30.0, 0.0, "block");
        let bf = circle_face(&mut sk, (30.0, 20.0), 5.0, "bore_f");
        let bore = prism(&mut sk, bf, -depth, 0.0, "bore");
        let body = sk.solid(
            SolidDef::Body { stock: block as u32, on: vec![], through: vec![bore as u32] },
            "body",
        );
        let b = sk.solid_boundary(body, UNIT);
        (mesh::volume(&b), b.len())
    };
    let flush = make(30.0);
    let past = make(32.0);
    let hair = make(30.0 + 1e-10);
    assert!((flush.0 - past.0).abs() < 1e-6, "flush {} vs drilled past {}", flush.0, past.0);
    assert_eq!(flush.1, past.1, "and the same faces, not a sliver more");
    // a hair's difference in the input is a hair's difference in the answer and *no new face*:
    // the sliver a tolerant kernel leaves at a coincidence is what this is watching for
    assert_eq!(flush.1, hair.1, "a cap a hair past the face makes no extra face");
    assert!((flush.0 - hair.0).abs() < 1e-6, "and no measurable difference in volume");
}

#[test]
fn a_boss_adds_and_the_shared_face_is_counted_once() {
    let mut sk = Sketch::new();
    let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
    let block = prism(&mut sk, f, -30.0, 0.0, "block");
    let bf = rect_face(&mut sk, 20.0, 10.0, 40.0, 30.0, "boss_f");
    let boss = prism(&mut sk, bf, 0.0, 10.0, "boss");
    let body = sk.solid(
        SolidDef::Body { stock: block as u32, on: vec![boss as u32], through: vec![] },
        "body",
    );
    let want = 60.0 * 40.0 * 30.0 + 20.0 * 20.0 * 10.0;
    let got = volume(&sk, body);
    assert!((got - want).abs() < 1e-6, "a boss on a block adds: want {want}, got {got}");
    // the block's own near face is drawn once, and where the boss stands on it, not at all
    let b = sk.solid_boundary(body, UNIT);
    let near: f64 = b.iter().filter(|p| p.path == "block.near").map(|p| p.area()).sum();
    assert!(
        (near - (60.0 * 40.0 - 20.0 * 20.0)).abs() < 1e-6,
        "the near face less the boss's footprint, and got {near}"
    );
    assert!(!b.iter().any(|p| p.path == "boss.far"), "the boss's own base is inside the block");
}

#[test]
fn a_revolution_is_pappus() {
    // a rectangle turned about a line beside it is a ring: 2πR·A, to the faceting
    let mut sk = Sketch::new();
    let f = rect_face(&mut sk, 10.0, 0.0, 14.0, 6.0, "sec");
    let a = sk.point(0.0, 0.0, false, "");
    let b = sk.point(0.0, 10.0, false, "");
    let ax = sk.line(a, b);
    let full = sk.solid(
        SolidDef::Revolve {
            face: f as u32,
            axis: ax as u32,
            sweep: Extent::at(std::f64::consts::TAU),
            sense: Sense::Ccw,
        },
        "ring",
    );
    let want = std::f64::consts::TAU * 12.0 * (4.0 * 6.0);
    let got = volume(&sk, full);
    assert!((got - want).abs() < 2e-3 * want, "Pappus: want ≈ {want}, got {got}");
    assert!(got <= want + 1e-9, "a faceted turn is inside the true one");

    let quarter = sk.solid(
        SolidDef::Revolve {
            face: f as u32,
            axis: ax as u32,
            sweep: Extent::at(std::f64::consts::FRAC_PI_2),
            sense: Sense::Ccw,
        },
        "quarter",
    );
    let q = volume(&sk, quarter);
    assert!((q - got / 4.0).abs() < 2e-3 * want, "a quarter turn is a quarter of it: {q}");
}

#[test]
fn the_answer_does_not_depend_on_the_order_the_features_were_written() {
    // P2, checked in the kernel: both groups of the body rule are sets
    let build = |swap: bool| {
        let mut sk = Sketch::new();
        let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
        let block = prism(&mut sk, f, -30.0, 0.0, "block");
        let h1 = circle_face(&mut sk, (15.0, 20.0), 4.0, "h1");
        let b1 = prism(&mut sk, h1, -30.0, 0.0, "b1");
        let h2 = circle_face(&mut sk, (45.0, 20.0), 4.0, "h2");
        let b2 = prism(&mut sk, h2, -30.0, 0.0, "b2");
        let through =
            if swap { vec![b2 as u32, b1 as u32] } else { vec![b1 as u32, b2 as u32] };
        let body =
            sk.solid(SolidDef::Body { stock: block as u32, on: vec![], through }, "body");
        sk.solid_boundary(body, UNIT)
    };
    // the same *solid* either way round — the two groups of the body rule are sets, so the
    // volume and the face count cannot depend on the order the statements were written in.
    // The triangulation may: `a − b1 − b2` and `a − b2 − b1` cut the same surface along
    // different seams, which is a fact about the mesh and not about the object
    let a = build(false);
    let b = build(true);
    assert!(
        (gcs_core::mesh::volume(&a) - gcs_core::mesh::volume(&b)).abs() < 1e-6,
        "one solid, however the features were written down"
    );
    assert_eq!(a.len(), b.len(), "and the same number of faces");
    // and the *same* document twice is bit-identical, which is what determinism means
    let again = build(false);
    assert_eq!(
        gcs_core::mesh::triangles(&a),
        gcs_core::mesh::triangles(&again),
        "the same document gives the same mesh, bit for bit"
    );
}

#[test]
fn a_cylinder_shows_two_silhouettes_and_no_facet_seam() {
    // the draughtsman's rule: a cylinder is two lines and two rims, at every zoom.  Every seam
    // around its wall is a tessellation joint and is `smooth`, so a view draws none of them.
    for unit in [2.0, 0.5, 0.05] {
        let mut sk = Sketch::new();
        let f = circle_face(&mut sk, (0.0, 0.0), 10.0, "rim");
        let cyl = prism(&mut sk, f, -25.0, 0.0, "cyl");
        let es = sk.solid_edges(cyl, unit);
        assert!(!es.is_empty(), "a cylinder has edges at unit {unit}");
        assert!(
            es.iter().filter(|e| !e.smooth).count() > 0,
            "its two rims are corners of the design"
        );
        let creases: Vec<_> = es.iter().filter(|e| !e.smooth).collect();
        // every hard edge is a rim: it lies at one of the two ends
        for e in &creases {
            // the page's normal is −y, so a prism swept from −25 to 0 stands in y ∈ [0, 25]
            let at_end = |p: [f64; 3]| p[1].abs() < 1e-6 || (p[1] - 25.0).abs() < 1e-6;
            assert!(at_end(e.a) && at_end(e.b), "a hard edge of a cylinder is a rim, at {unit}");
        }
    }
}
