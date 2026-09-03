//! What a boundary comes to: triangles, a volume, a box, and the file a printer wants.
//!
//! Everything here reads `csg::boundary` and nothing else, so a mesh, a volume and an STL are
//! three readings of one answer rather than three walks that might disagree.

use crate::csg::Piece;
use crate::plane;
use crate::solid::Box3;

/// The boundary as triangles, nine doubles each, in the pieces' own order — deterministic, since
/// the boundary walk is.
pub fn triangles(pieces: &[Piece]) -> Vec<f64> {
    let mut out = Vec::new();
    for p in pieces {
        for i in 1..p.pts.len().saturating_sub(1) {
            for v in [p.pts[0], p.pts[i], p.pts[i + 1]] {
                out.extend(v);
            }
        }
    }
    out
}

/// The volume the boundary encloses, by the divergence theorem: `Σ a·(b × c) / 6` over the
/// oriented triangles.  A boundary with a hole in it — which a degenerate face would give —
/// comes out wrong rather than refusing, which is why the elaborator refuses one first.
pub fn volume(pieces: &[Piece]) -> f64 {
    let t = triangles(pieces);
    let mut v = 0.0;
    for c in t.chunks_exact(9) {
        let a = [c[0], c[1], c[2]];
        let b = [c[3], c[4], c[5]];
        let d = [c[6], c[7], c[8]];
        v += plane::dot(a, plane::cross(b, d));
    }
    v / 6.0
}

/// The total area of the boundary, and of one named face of it — what a report says when a
/// boolean has eaten part of a face and the document still names it.
pub fn area(pieces: &[Piece]) -> f64 {
    pieces.iter().map(|p| p.area()).sum()
}

pub fn bounds(pieces: &[Piece]) -> Box3 {
    let mut b = Box3::empty();
    for p in pieces {
        for q in &p.pts {
            b.add(*q);
        }
    }
    b
}

/// Binary STL: an eighty-byte header, a count, and fifty bytes a triangle.  Written here because
/// a printer is the one consumer of a solid that is not a picture, and because writing it needs
/// nothing the crate does not already have.
pub fn stl(pieces: &[Piece], name: &str) -> Vec<u8> {
    let t = triangles(pieces);
    let n = t.len() / 9;
    let mut out = Vec::with_capacity(84 + n * 50);
    let mut header = [0u8; 80];
    let label = format!("solvent {name}");
    for (i, b) in label.bytes().take(79).enumerate() {
        header[i] = b;
    }
    out.extend(header);
    out.extend((n as u32).to_le_bytes());
    for c in t.chunks_exact(9) {
        let a = [c[0], c[1], c[2]];
        let b = [c[3], c[4], c[5]];
        let d = [c[6], c[7], c[8]];
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
        let nrm = plane::unit(plane::cross(e1, e2)).unwrap_or([0.0, 0.0, 0.0]);
        for v in nrm {
            out.extend((v as f32).to_le_bytes());
        }
        for v in [a, b, d] {
            for x in v {
                out.extend((x as f32).to_le_bytes());
            }
        }
        out.extend(0u16.to_le_bytes());
    }
    out
}
