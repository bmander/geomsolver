//! A field-read body written without OCCT (#904 A15): the static blank of a body with swept cuts
//! read by the core's own kernel (`blank_features::KernelBlank`) for the features the field's
//! refinement protects, and a body whose exact export is refused told what writes its field.
use crate::common::{ent, read, solved};
use gcs_core::solid::blank_features::{self, BlankTopology, KernelBlank};
use gcs_core::space::{dot, norm, sub};

const SWEPT_TORUS: &str = include_str!("../../examples/swept_torus.sv");
const SWEPT_TUMBLE: &str = include_str!("../../examples/swept_tumble.sv");

/// **The post's blank as the core's kernel builds it**: a cylinder, its side one face round a
/// whole turn (a seam, which bounds nothing) and its two caps; its two rims the only edges two
/// faces meet along, each convex all round, the faces' outward normals square across it; a face's
/// middle in it, not on its trim, its normal outward, and a cap's corner of its parameters off it.
#[test]
fn the_core_kernel_reads_a_blanks_edges_and_faces() {
    let e = read(SWEPT_TORUS);
    let sk = solved(&e);
    let part = ent(&e, "part").i();
    let blank = KernelBlank::of(&sk, part).unwrap();
    assert_eq!(blank.edges().unwrap(), 2, "the two rims");
    for edge in 0..2 {
        for k in 0..=8 {
            let p = blank.edge_point(edge, k as f64 / 8.).unwrap();
            assert!((p.dihedral + std::f64::consts::FRAC_PI_2).abs() < 1e-9, "convex, square: {}", p.dihedral);
            assert!(dot(p.normals[0], p.normals[1]).abs() < 1e-9);
            assert!(p.normals.iter().all(|n| (norm(*n) - 1.).abs() < 1e-12));
        }
    }
    assert_eq!(blank.faces().unwrap(), 3);
    let seams: Vec<[bool; 2]> = (0..3).map(|f| blank.face_seams(f).unwrap()).collect();
    assert_eq!(seams.iter().filter(|s| **s == [true, false]).count(), 1, "{seams:?}");
    assert_eq!(seams.iter().filter(|s| **s == [false, false]).count(), 2, "{seams:?}");
    // the post's axis runs from (4, 0, 0.5) to (4, 0, 2): its middle
    let centre = [4., 0., 1.25];
    for face in 0..3 {
        let p = blank.face_point(face, 0.5, 0.5, 1e-9).unwrap().expect("a face's middle is in it");
        assert!(!p.on_trim);
        assert!(dot(p.normal, sub(p.position, centre)) > 0., "outward: {p:?}");
    }
    let cap = seams.iter().position(|s| *s == [false, false]).unwrap();
    assert!(blank.face_point(cap, 0., 0., 1e-9).unwrap().is_none(), "a disc's box's corner is off it");
}

/// **The features the field's refinement protects, read off that blank**: the tumbling torus
/// takes the post's top cap and the top of its side, so what is left of its rims is the bottom
/// one whole, and the swept boundary crosses the side in one closed contour — each a closed line
/// round the post's side, on the part's boundary (its field nought there).
#[test]
fn a_blanks_features_are_read_by_the_core_kernel() {
    let e = read(SWEPT_TORUS);
    let sk = solved(&e);
    let part = ent(&e, "part").i();
    let field = gcs_core::solid::MaterialField::read(&sk, part, gcs_core::solid::cad::AXIS_TOLERANCE).unwrap();
    let blank = KernelBlank::of(&sk, part).unwrap();
    let found = blank_features::features(&blank, &field, 1., 1.6, 32).unwrap();
    assert_eq!(found.lines.len(), 2, "the bottom rim and the cut's contour");
    let radius = |p: &[f64; 3]| ((p[0] - 4.).powi(2) + p[1].powi(2)).sqrt();
    for l in &found.lines {
        assert!(l.len() > 16 && l[0] == *l.last().unwrap(), "closed: {} points", l.len());
        assert!(l.iter().all(|p| (radius(p) - 0.4).abs() < 1e-9), "round the side");
        let worst = l.iter().map(|p| field.reading(*p).value.abs()).fold(0f64, f64::max);
        assert!(worst < 1e-3, "a feature's point off the part's boundary by {worst}");
    }
    let heights: Vec<(f64, f64)> = found.lines.iter()
        .map(|l| l.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[2]), hi.max(p[2])))).collect();
    assert!(heights.iter().any(|&(lo, hi)| (lo - 0.5).abs() < 1e-12 && (hi - 0.5).abs() < 1e-12), "the bottom rim: {heights:?}");
    assert!(heights.iter().any(|&(lo, hi)| lo > 0.6 && hi < 1.9), "the cut across the side: {heights:?}");
}

/// **A body refused exactly is told what writes its field** (#904 A15): the tumbled torus, which
/// no exact construction builds, is read by its field, and the faceted output the host names
/// writes that; a static solid has no field to offer.
#[test]
fn a_field_read_body_is_offered_its_fields_writing() {
    let e = read(SWEPT_TUMBLE);
    let sk = solved(&e);
    let tumbled = ent(&e, "tumbled").i();
    let said = gcs_core::report::field_read_instead(&sk, tumbled, "`--stl-backend mesh`").expect("read by its field");
    assert!(said.starts_with("`tumbled` is read by its material field, and `--stl-backend mesh` writes that \
        reading instead"), "{said}");
    assert!(gcs_core::report::field_read_instead(&sk, ent(&e, "torus").i(), "it").is_none());
}

