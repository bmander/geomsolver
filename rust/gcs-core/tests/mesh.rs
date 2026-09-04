//! **The mesh a printer and a viewer take** — welded, and grouped by the faces the document names.
//!
//! Two properties, and each is a thing a boundary evaluation does not give on its own.
//!
//! *Watertight*: every directed edge has its reverse.  A boundary cuts neighbouring facets by
//! different planes, so one leaves a vertex partway along an edge the other still spans whole —
//! a **T-junction**, which makes a surface geometrically closed and topologically not.  The
//! V-twin cylinder's mesh had 4,474 such edges of 141,468, and a strict validator refuses that.
//!
//! *Grouped*: a viewer wants the faces of the object, not a heap of facets — so that selecting
//! one names `body.bore.wall` and a round face shades round.
//!
//! Neither wanted a B-rep.  A viewer and a slicer both take triangles; what they want of them is
//! that the edges pair up and that the triangles say which face they belong to.

use gcs_core::model::{EntRef, Extent, Sketch, SolidDef};
use gcs_core::{mesh, solid};
use std::collections::BTreeMap;

const UNIT: f64 = solid::REPORT_UNIT;

fn rect_face(sk: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64, name: &str) -> usize {
    let ids: Vec<usize> = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        .iter()
        .map(|&(x, y)| sk.point(x, y, false, ""))
        .collect();
    let mut edges = Vec::new();
    let mut names = Vec::new();
    for i in 0..4 {
        edges.push(EntRef::line(sk.line(ids[i], ids[(i + 1) % 4])));
        names.push(format!("e{i}"));
    }
    sk.face(edges, names, name)
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

/// A block with a bore through it — the case whose mesh had thousands of unpaired edges.
fn bored() -> (Sketch, usize) {
    let mut sk = Sketch::new();
    let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
    let block = prism(&mut sk, f, -30.0, 0.0, "block");
    let bf = circle_face(&mut sk, (30.0, 20.0), 8.0, "bore_f");
    let bore = prism(&mut sk, bf, -30.0, 0.0, "bore");
    let body = sk.solid(
        SolidDef::Body { stock: block as u32, on: vec![], through: vec![bore as u32] },
        "body",
    );
    (sk, body)
}

/// How many directed edges have no matching reverse — zero for a closed, manifold surface.
fn unpaired(t: &[f64]) -> usize {
    let key = |p: &[f64]| {
        [
            (p[0] * 1e7).round() as i64,
            (p[1] * 1e7).round() as i64,
            (p[2] * 1e7).round() as i64,
        ]
    };
    let mut edge: BTreeMap<([i64; 3], [i64; 3]), i64> = BTreeMap::new();
    for c in t.chunks_exact(9) {
        let (a, b, d) = (key(&c[0..3]), key(&c[3..6]), key(&c[6..9]));
        for (p, q) in [(a, b), (b, d), (d, a)] {
            *edge.entry((p, q)).or_default() += 1;
        }
    }
    edge.iter().filter(|((p, q), k)| edge.get(&(*q, *p)).copied().unwrap_or(0) != **k).count()
}

#[test]
fn the_mesh_a_printer_gets_is_closed() {
    let (sk, body) = bored();
    let pieces = sk.solid_boundary(body, UNIT);
    // the boundary as it comes: right in every way that matters to a *reading* of it…
    let raw = mesh::triangles(&pieces);
    let vol = mesh::volume(&pieces);
    // …and not closed, which is what the weld is for
    let welded = mesh::grouped(&pieces).positions;
    assert_eq!(unpaired(&welded), 0, "every edge has its partner");
    assert!(unpaired(&raw) > 0, "and it did not before — otherwise this test proves nothing");
    // and the object is the same object: the weld inserts vertices and moves none
    let after: f64 = {
        let mut v = 0.0;
        for c in welded.chunks_exact(9) {
            let a = [c[0], c[1], c[2]];
            let b = [c[3], c[4], c[5]];
            let d = [c[6], c[7], c[8]];
            let cr = [
                b[1] * d[2] - b[2] * d[1],
                b[2] * d[0] - b[0] * d[2],
                b[0] * d[1] - b[1] * d[0],
            ];
            v += a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2];
        }
        v / 6.0
    };
    assert!((after - vol).abs() < 1e-6 * vol.abs(), "same volume: {after} against {vol}");
}

#[test]
fn a_solid_that_needs_no_welding_is_left_alone() {
    // a plain block's facets already meet corner to corner, so the weld has nothing to insert
    let mut sk = Sketch::new();
    let f = rect_face(&mut sk, 0.0, 0.0, 60.0, 40.0, "sec");
    let b = prism(&mut sk, f, -30.0, 0.0, "block");
    let m = mesh::grouped(&sk.solid_boundary(b, UNIT));
    assert_eq!(unpaired(&m.positions), 0);
    // six faces, each named as the section drew it
    let paths: Vec<&str> = m.groups.iter().map(|g| g.path.as_str()).collect();
    assert_eq!(paths, vec!["block.e0", "block.e1", "block.e2", "block.e3", "block.far", "block.near"]);
    assert!(m.groups.iter().all(|g| !g.smooth), "a block has no round face");
}

#[test]
fn the_mesh_is_grouped_by_the_faces_the_document_names() {
    let (sk, body) = bored();
    let m = sk.solid_mesh(body, UNIT);
    let paths: Vec<&str> = m.groups.iter().map(|g| g.path.as_str()).collect();
    assert!(paths.contains(&"block.near"), "the faces of the stock: {paths:?}");
    assert!(paths.contains(&"bore.rim"), "and the wall of the bore: {paths:?}");
    // **the bore's wall is one round face, not sixty flats** — which is what lets a viewer shade
    // it smooth and what lets a click on it name the feature
    let wall = m.groups.iter().find(|g| g.path == "bore.rim").expect("the bore's wall");
    assert!(wall.smooth, "a tessellated surface says so");
    assert!(wall.count > 60, "and it is many triangles under one name: {}", wall.count);
    assert!(m.groups.iter().find(|g| g.path == "block.near").is_some_and(|g| !g.smooth),
            "while a flat face is flat");
    // every triangle belongs to exactly one face, and the groups tile the buffer in order
    let mut at = 0;
    for g in &m.groups {
        assert_eq!(g.start, at, "the groups run end to end");
        at += g.count;
    }
    assert_eq!(at * 9, m.positions.len(), "and cover it");
    assert_eq!(m.normals.len(), m.positions.len(), "a normal a vertex");
}

#[test]
fn a_round_face_shades_round_and_a_corner_stays_a_corner() {
    let (sk, body) = bored();
    let m = sk.solid_mesh(body, UNIT);
    let g = m.groups.iter().find(|g| g.path == "bore.rim").expect("the bore's wall");
    // the averaged normals along the wall vary continuously: no two adjacent triangles share a
    // normal exactly, and none of them is far from the facet's own
    let mut distinct = 0;
    let mut last = [f64::NAN; 3];
    for t in g.start..g.start + g.count {
        let n = [m.normals[t * 9], m.normals[t * 9 + 1], m.normals[t * 9 + 2]];
        if (n[0] - last[0]).abs() > 1e-12 {
            distinct += 1;
        }
        last = n;
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!((len - 1.0).abs() < 1e-9, "a normal is a unit vector, and got {len}");
    }
    assert!(distinct > 10, "a round wall's normals turn with it: {distinct}");
    // the cap is flat, so every one of its normals is the same
    let cap = m.groups.iter().find(|g| g.path == "block.near").expect("the near cap");
    let n0 = [m.normals[cap.start * 9], m.normals[cap.start * 9 + 1], m.normals[cap.start * 9 + 2]];
    for t in cap.start..cap.start + cap.count {
        for k in 0..3 {
            assert!((m.normals[t * 9 + k] - n0[k]).abs() < 1e-12, "a flat face is one normal");
        }
    }
}

#[test]
fn a_mesh_is_cut_to_the_object_and_a_volume_to_the_report() {
    // **Two requirements, and giving them one number costs an order of magnitude.**  A volume is
    // quoted to four digits, so `REPORT_UNIT` is fine; a mesh is looked at and printed, and the
    // V-twin cylinder's inherited that and came out at 98,000 triangles where 8,000 are
    // indistinguishable — a bore cut into 257 flats, a sagitta a hundred times under what any
    // printer resolves.
    let (sk, body) = bored();
    let mu = solid::mesh_unit(&sk, body);
    assert!(mu > solid::REPORT_UNIT, "a mesh is cut coarser than a report: {mu}");

    let fine = mesh::grouped(&sk.solid_boundary(body, solid::REPORT_UNIT)).positions.len() / 9;
    let mesh_at = mesh::grouped(&sk.solid_boundary(body, mu));
    let coarse = mesh_at.positions.len() / 9;
    // how much cheaper depends on how round the part is — the V-twin's cylinder, with four
    // holes in it, is twelve times cheaper; this block with one is three
    assert!(coarse * 3 < fine, "and far cheaper for it: {coarse} against {fine}");
    // still closed — the whole point of the weld, and the cheap fan must not cost it
    assert_eq!(unpaired(&mesh_at.positions), 0, "a mesh cut to the object is closed too");

    // and it is *scale-free*: the same part ten times bigger is cut into the same triangles, so
    // a document in inches and one in millimetres get the same file
    let mut big = Sketch::new();
    let f = rect_face(&mut big, 0.0, 0.0, 600.0, 400.0, "sec");
    let block = prism(&mut big, f, -300.0, 0.0, "block");
    let bf = circle_face(&mut big, (300.0, 200.0), 80.0, "bore_f");
    let bore = prism(&mut big, bf, -300.0, 0.0, "bore");
    let b2 = big.solid(
        SolidDef::Body { stock: block as u32, on: vec![], through: vec![bore as u32] },
        "body",
    );
    let n2 = mesh::grouped(&big.solid_boundary(b2, solid::mesh_unit(&big, b2))).positions.len() / 9;
    assert!(
        (n2 as f64 - coarse as f64).abs() < coarse as f64 * 0.2,
        "ten times the size is the same mesh: {n2} against {coarse}"
    );
}

#[test]
fn the_cheap_fan_is_only_taken_where_it_is_safe() {
    // a piece the weld never touched has only corners and takes `n − 2` triangles; one it
    // stitched has a vertex that is not a corner and takes `n`, because a vertex fan there would
    // drop the sub-edge and re-open the T-junction.  Both are in this solid at once.
    let (sk, body) = bored();
    let pieces = sk.solid_boundary(body, solid::mesh_unit(&sk, body));
    let welded = mesh::weld(&pieces);
    let sides: usize = welded.iter().map(|p| p.pts.len()).sum();
    let tris = mesh::triangles(&welded).len() / 9;
    // between the two bounds: every piece a centroid fan would be `sides`, every piece a vertex
    // fan would be `sides − 2·pieces`
    assert!(tris < sides, "not every piece paid the centroid's price: {tris} of {sides}");
    assert!(
        tris > sides - 2 * welded.len(),
        "and some did: {tris} against {}",
        sides - 2 * welded.len()
    );
    assert_eq!(unpaired(&mesh::triangles(&welded)), 0, "and it is still closed");
}
