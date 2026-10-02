//! What a boundary comes to: triangles, a volume, a box, and the files a printer and a viewer
//! want.
//!
//! Everything here reads `csg::boundary` and nothing else, so a mesh, a volume and an STL are
//! three readings of one answer rather than three walks that might disagree.
//!
//! **Two things a boundary is not yet, and this is where it becomes them.**
//!
//! A boundary evaluation cuts each facet by the planes that reach it, and two neighbours are cut
//! by *different* planes — so one leaves a vertex partway along an edge the other still spans
//! whole.  That is a **T-junction**, and it makes a surface that is geometrically closed and
//! topologically not: the V-twin cylinder's own mesh had 4,774 such edges out of 141,468, its
//! volume exact to the digit and 3.4% of its edges without a partner.  A slicer will usually
//! repair it and a strict validator will refuse it, and neither should have to.  `weld` is the
//! answer, and it needs no B-rep: find the vertices that lie on an edge's interior and put them
//! in it.
//!
//! And a boundary is a heap of facets, where a *viewer* wants the faces of the object — so that
//! a bore's wall shades as one round surface rather than sixty flats, and so that clicking it
//! selects `body.bore.wall` and not a triangle.  `grouped` is that: the same triangles, in the
//! order a face path puts them, with the normals each face deserves.

use crate::csg::Piece;
use crate::plane;
use crate::solid::Box3;
use std::collections::BTreeMap;

/// The boundary as triangles, nine doubles each, in the pieces' own order — deterministic, since
/// the boundary walk is.
///
/// A fan, which is exact because every piece is convex: the facets are triangles and quads, and
/// cutting a convex polygon by a plane leaves convex polygons.
///
/// **From a corner where the piece has only corners, and from the centroid where it does not.**
/// A vertex fan is the cheap and obvious one — `n − 2` triangles — and it covers every boundary
/// edge *except* the sub-edges of the two edges its apex stands on.  Where the weld has inserted
/// a vertex partway along one of those, the fan triangle over it is zero-area; dropping it (which
/// a printer's validator wants) takes the sub-edge with it and re-opens exactly the T-junction
/// the weld had just closed.  The centroid of a convex polygon is strictly inside it, so a
/// centroid fan covers every boundary edge and none of its triangles is degenerate — at the cost
/// of two more triangles.
///
/// So the test is per piece and is the condition itself: **has this polygon a vertex that is not
/// a corner?**  Most have not — a quad the weld never touched is two triangles, as it always was
/// — and the ones that have pay for what they need.
pub fn triangles(pieces: &[Piece]) -> Vec<f64> {
    let mut out = Vec::new();
    for p in pieces {
        fan(&p.pts, |a, b, c| {
            for v in [a, b, c] {
                out.extend(v);
            }
        });
    }
    out
}

/// Every triangle of one piece, handed to `emit` in order.  The one triangulation, so a mesh, a
/// volume and an STL cut a face the same way.
fn fan(pts: &[[f64; 3]], mut emit: impl FnMut([f64; 3], [f64; 3], [f64; 3])) {
    if pts.len() < 3 {
        return;
    }
    if all_corners(pts) {
        for i in 1..pts.len() - 1 {
            if !degenerate(pts[0], pts[i], pts[i + 1]) {
                emit(pts[0], pts[i], pts[i + 1]);
            }
        }
        return;
    }
    let c = centroid(pts);
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        if !degenerate(c, a, b) {
            emit(c, a, b);
        }
    }
}

fn centroid(pts: &[[f64; 3]]) -> [f64; 3] {
    let k = 1.0 / pts.len() as f64;
    let mut c = [0.0; 3];
    for p in pts {
        for i in 0..3 {
            c[i] += p[i] * k;
        }
    }
    c
}

/// Is every vertex a true corner — no three in a row collinear?  Exactly the condition under
/// which a vertex fan loses no boundary edge.
fn all_corners(pts: &[[f64; 3]]) -> bool {
    (0..pts.len()).all(|i| {
        !degenerate(pts[(i + pts.len() - 1) % pts.len()], pts[i], pts[(i + 1) % pts.len()])
    })
}

/// A triangle with no area — three collinear points, which a zero-length side would give.
pub fn degenerate(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> bool {
    let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = plane::norm(plane::cross(e1, e2));
    let scale = plane::norm(e1) * plane::norm(e2);
    n <= scale * 1e-12
}

/// How near two vertices must be, relative to the object, to be one vertex.  Far above the noise
/// a plane intersection leaves and far below anything a drawing states.
const WELD: f64 = 1e-9;

/// **Close the surface**: one vertex where two pieces meant one, and no vertex left standing in
/// the middle of a neighbour's edge.
///
/// Two passes, and the second is the one that matters.  *Welding* makes coincident corners
/// literally the same numbers, so a consumer that hashes vertices sees the surface join up.
/// *Stitching* then walks every edge and inserts the welded vertices that lie along it — which
/// is what a T-junction is, and what no amount of vertex merging alone can fix: the neighbour
/// spans the whole edge and simply has no vertex there to merge with.
///
/// It is a bounded, local operation on the output, and it is worth being clear that this is the
/// **whole** of what a mesh needed a B-rep for.  A viewer and a slicer both take triangles; what
/// they want of them is that the edges pair up.
pub fn weld(pieces: &[Piece]) -> Vec<Piece> {
    if pieces.is_empty() {
        return Vec::new();
    }
    let b = bounds(pieces);
    let scale = (0..3).fold(1.0f64, |m, i| m.max(b.hi[i] - b.lo[i]));
    // A long thin solid still has small features across its short dimensions. A tolerance
    // based only on its length can stitch nearby cap diagonals that never actually meet.
    let feature = (0..3).map(|i| b.hi[i] - b.lo[i]).filter(|d| *d > 0.0)
        .fold(scale, f64::min);
    let coordinate = b.lo.iter().chain(&b.hi).fold(0.0f64, |m, v| m.max(v.abs()));
    let tol = (feature * WELD).max(coordinate * f64::EPSILON * 4.0);
    // **The cell is not the tolerance.**  A cell only has to be *at least* `tol` across, so that
    // two vertices within `tol` always share a cell or a neighbouring one; how much bigger it is
    // is a cost trade and nothing else, since the answer comes from the distance test inside.
    // Sized at `tol` an edge would walk its own length in cells — hundreds of millions of them
    // on a part forty across — so it is sized to the object instead.
    // a grid on the coordinates: a vertex is looked for in its own cell and the twenty-six
    // around it, so two within `tol` are found however the rounding fell
    let mut cells = crate::space::Grid::new((scale / 128.0).max(tol));
    let mut verts: Vec<[f64; 3]> = Vec::new();
    let canon = |p: [f64; 3], cells: &mut crate::space::Grid, verts: &mut Vec<[f64; 3]>| -> usize {
        let mut found = None;
        cells.around(p, |i| {
            let q = verts[i as usize];
            if found.is_none() && plane::norm([p[0] - q[0], p[1] - q[1], p[2] - q[2]]) <= tol {
                found = Some(i as usize);
            }
        });
        if let Some(i) = found {
            return i;
        }
        verts.push(p);
        cells.insert(p, (verts.len() - 1) as u32);
        verts.len() - 1
    };
    // -- weld: every piece, as indices into one vertex table. Most corners are the same bits as
    // one already welded (each is shared by the pieces around it), and the same bits go where
    // they went the first time without searching the cells again; looked up, never iterated.
    let mut seen: std::collections::HashMap<[u64; 3], usize> = std::collections::HashMap::new();
    let loops: Vec<Vec<usize>> = pieces
        .iter()
        .map(|p| p.pts.iter().map(|&q| {
            *seen.entry(q.map(f64::to_bits)).or_insert_with(|| canon(q, &mut cells, &mut verts))
        }).collect())
        .collect();

    // -- stitch: a vertex on the interior of an edge belongs in that edge. Nothing is filed from
    // here on, so the grid is read packed: a cell is an array index, not a tree walk, and an edge
    // looks in some thirty of them.
    // Each edge is walked from both of its pieces: what lies on it is found once, from its
    // lower-numbered end, and read backwards from the other.
    let cells = cells.pack();
    let mut between: std::collections::HashMap<(usize, usize), Vec<[f64; 3]>> = std::collections::HashMap::new();
    let mut out = Vec::with_capacity(pieces.len());
    for (p, idx) in pieces.iter().zip(&loops) {
        let mut pts: Vec<[f64; 3]> = Vec::with_capacity(idx.len());
        for i in 0..idx.len() {
            let (ia, ib) = (idx[i], idx[(i + 1) % idx.len()]);
            pts.push(verts[ia]);
            if let Some(on) = between.get(&(ia.min(ib), ia.max(ib))) {
                if ia < ib { pts.extend(on.iter().copied()) } else { pts.extend(on.iter().rev().copied()) }
                continue;
            }
            let (a, b) = (verts[ia.min(ib)], verts[ia.max(ib)]);
            let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let len = plane::norm(d);
            if len <= tol {
                continue;
            }
            // the cells the edge's own box covers, each looked at once — a thin box for the
            // thin thing an edge is, and no set to remember what has been seen
            let mut on: Vec<(f64, [f64; 3])> = Vec::new();
            let (ka, kb) = (cells.key(a), cells.key(b));
            let lo = [ka[0].min(kb[0]) - 1, ka[1].min(kb[1]) - 1, ka[2].min(kb[2]) - 1];
            let hi = [ka[0].max(kb[0]) + 1, ka[1].max(kb[1]) + 1, ka[2].max(kb[2]) + 1];
            cells.in_keys(lo, hi, |vi| {
                let q = verts[vi as usize];
                let w = [q[0] - a[0], q[1] - a[1], q[2] - a[2]];
                let u = plane::dot(w, d) / (len * len);
                if u <= 0.0 || u >= 1.0 {
                    return;
                }
                let foot = [a[0] + u * d[0], a[1] + u * d[1], a[2] + u * d[2]];
                let off = plane::norm([q[0] - foot[0], q[1] - foot[1], q[2] - foot[2]]);
                // strictly *on* the edge and strictly between its ends
                if off <= tol && u * len > tol && (1.0 - u) * len > tol {
                    on.push((u, q));
                }
            });
            on.sort_by(|x, y| x.0.partial_cmp(&y.0).expect("a finite mesh"));
            let on: Vec<[f64; 3]> = on.into_iter().map(|(_, q)| q).collect();
            if ia < ib { pts.extend(on.iter().copied()) } else { pts.extend(on.iter().rev().copied()) }
            between.insert((ia.min(ib), ia.max(ib)), on);
        }
        if pts.len() >= 3 {
            out.push(Piece { pts, ..p.clone() });
        }
    }
    out
}

/// The volume the boundary encloses, by the divergence theorem: `Σ a·(b × c) / 6` over the
/// oriented triangles.  A boundary with a hole in it — which a degenerate face would give —
/// comes out wrong rather than refusing, which is why the elaborator refuses one first.
pub fn volume(pieces: &[Piece]) -> f64 {
    let t = triangles(pieces);
    let origin = pieces.iter().find_map(|p| p.pts.first()).copied().unwrap_or([0.0; 3]);
    let local = |p: &[f64]| [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
    let mut v = 0.0;
    let mut correction = 0.0;
    for c in t.chunks_exact(9) {
        let term = plane::dot(local(&c[0..3]), plane::cross(local(&c[3..6]), local(&c[6..9])));
        let adjusted = term - correction;
        let next = v + adjusted;
        correction = (next - v) - adjusted;
        v = next;
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
///
/// **Welded**, so the file a slicer gets is closed: every edge has its partner, which a boundary
/// evaluation does not give on its own and which a strict validator refuses without.
pub fn stl(pieces: &[Piece], name: &str) -> Vec<u8> {
    stl_triangles(&grouped(pieces).positions, [0.0; 3], name)
}

/// Binary STL of triangles given as nine coordinates each, in order (no welding: a mesh whose
/// shared points are already the same doubles, as a B-rep's mesh is).
pub fn stl_of(triangles: &[f64], name: &str) -> Vec<u8> { stl_triangles(triangles, [0.0; 3], name) }

// Placement is applied only when encoding vertices. Never weld/triangulate rounded world
// coordinates: that could discard collapsed triangles before the export can diagnose them.
fn stl_triangles(t: &[f64], origin: [f64; 3], name: &str) -> Vec<u8> {
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
            for k in 0..3 {
                out.extend(((v[k] + origin[k]) as f32).to_le_bytes());
            }
        }
        out.extend(0u16.to_le_bytes());
    }
    out
}

/// Refuse an export whose float32 coordinates collapse otherwise valid triangles.
/// Geometry can be representable in the f64 model yet too small at its world position for STL.
pub fn checked_stl(pieces: &[Piece], name: &str) -> Result<Vec<u8>, String> {
    check_stl(stl(pieces, name), name)
}

/// Binary STL of an already indexed, already manifold triangle list, written
/// without any weld: a host that arranged the mesh itself keeps its own vertex
/// identities. Coordinates are checked for float32 collapse as always.
pub fn indexed_stl(vertices: &[[f64; 3]], triangles: &[[u32; 3]], name: &str) -> Result<Vec<u8>, String> {
    let mut positions = Vec::with_capacity(triangles.len() * 9);
    for t in triangles {
        for &i in t {
            positions.extend(vertices.get(i as usize).ok_or("STL triangle index out of range")?);
        }
    }
    check_stl(stl_triangles(&positions, [0.0; 3], name), name)
}

pub(crate) fn placed_stl(mesh: &Mesh, origin: [f64; 3], name: &str) -> Result<Vec<u8>, String> {
    check_stl(stl_triangles(&mesh.positions, origin, name), name)
}

/// A surface still being refined, as it stands: it may be coarse or open, and a triangle float32
/// flattens is dropped rather than refusing the file, since a preview is a look and not a part.
pub(crate) fn preview_stl(mesh: &Mesh, origin: [f64; 3], name: &str) -> Result<Vec<u8>, String> {
    let bytes = stl_triangles(&mesh.positions, origin, name);
    let kept: Vec<&[u8]> = stl_points(&bytes)?.zip(bytes[84..].chunks_exact(50))
        .filter(|(v, _)| v.iter().flatten().all(|x| x.is_finite()) && !degenerate(v[0], v[1], v[2]))
        .map(|(_, record)| record).collect();
    if kept.is_empty() { return Err(format!("`{name}` has no surface to export yet")); }
    let mut out = bytes[..80].to_vec();
    out.extend((kept.len() as u32).to_le_bytes());
    for record in kept { out.extend_from_slice(record); }
    Ok(out)
}

fn check_stl(bytes: Vec<u8>, name: &str) -> Result<Vec<u8>, String> {
    for v in stl_points(&bytes)? {
        if v.iter().flatten().any(|x| !x.is_finite()) || degenerate(v[0], v[1], v[2]) {
            return Err(format!("`{name}` cannot be represented at this position and scale by float32 STL coordinates; move the solid nearer the origin or change its export units"));
        }
    }
    Ok(bytes)
}

fn stl_points(bytes: &[u8]) -> Result<impl Iterator<Item=[[f64;3];3]> + '_,String> {
    if bytes.len() < 84 { return Err("incomplete binary STL header".into()); }
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let expected = count.checked_mul(50).and_then(|n| n.checked_add(84));
    if expected != Some(bytes.len()) {
        return Err("binary STL triangle count does not match its byte length".into());
    }
    Ok(bytes[84..].chunks_exact(50).map(|triangle| std::array::from_fn(|i|
        std::array::from_fn(|k| {
            let offset = 12+i*12+k*4;
            f32::from_le_bytes(triangle[offset..offset+4].try_into().unwrap()) as f64
        }))))
}

/// How far float32 encoding may have moved a vertex of a binary STL, read off the file alone: a
/// coordinate rounded to nearest stands within half the step to the next float32 away from zero
/// (the larger of its two neighbours' steps), and a vertex within the length of those three
/// halves. Every point of a written triangle is a weighted mean of its corners, so the written
/// surface lies within this of the triangles that were encoded: a world placement far from the
/// origin spends a tolerance before the mesh has sagged at all (a face at x = 10⁶ mm moves by
/// up to 31 µm). A bound and not a measurement: never less than the displacement, and more where
/// a coordinate was written exactly.
pub fn stl_rounding(bytes: &[u8]) -> Result<f64,String> {
    let half_step = |x: f64| {
        let a = (x as f32).abs();
        if !a.is_finite() { return f64::INFINITY }
        (f32::from_bits(a.to_bits()+1) as f64-a as f64)/2.
    };
    Ok(stl_points(bytes)?.flatten().map(|v| v.map(half_step).iter().map(|h| h*h).sum::<f64>().sqrt()).fold(0.,f64::max))
}

/// Reconstruct and validate the topology of the actual binary STL coordinates.
/// Exactly equal encoded positions share one identity (+0 and -0 compare equal);
/// no distance tolerance repairs the file. Also refuses nonfinite/degenerate triangles.
/// This proves one closed oriented manifold, not geometric non-self-intersection
/// or a bound on deviation from the original analytic surfaces.
pub fn stl_topology(bytes: &[u8]) -> Result<crate::topology::ClosedShell,String> {
    let (vertices,triangles) = stl_indices(bytes)?;
    crate::topology::ClosedShell::from_triangles(vertices,&triangles)
        .map_err(|e| format!("invalid binary STL shell topology: {e:?}"))
}

/// Validate every connected shell of an encoded STL, including cavity walls and
/// separate bodies. Connectivity uses exactly equal float32 positions; no repair
/// or distance-based welding occurs. Touching vertex fans remain one component
/// and must pass the manifold check. This does not check geometric intersection.
pub fn stl_shells(bytes: &[u8]) -> Result<Vec<crate::topology::ClosedShell>,String> {
    let (vertices,triangles) = stl_indices(bytes)?;
    if triangles.is_empty() { return Err("binary STL has no triangles".into()); }
    let mut incident = vec![Vec::new();vertices];
    for (i,t) in triangles.iter().enumerate() {
        for &v in t { incident[v].push(i); }
    }
    let mut seen = vec![false;triangles.len()];
    let mut shells = Vec::new();
    // each shell's vertices numbered as its walk first meets them: a vertex's number, and the shell
    // it was given it in
    let mut local = vec![(usize::MAX,0usize);vertices];
    for start in 0..triangles.len() {
        if seen[start] { continue; }
        let shell = shells.len();
        let mut count = 0;
        let mut pending = vec![start];
        let mut faces = Vec::new();
        while let Some(i) = pending.pop() {
            if seen[i] { continue; }
            seen[i] = true;
            faces.push(triangles[i].map(|v| {
                pending.append(&mut incident[v]);
                if local[v].1 != shell || local[v].0 == usize::MAX { local[v] = (count,shell); count += 1; }
                local[v].0
            }));
        }
        shells.push(crate::topology::ClosedShell::from_triangles(count,&faces)
            .map_err(|e| format!("invalid binary STL shell topology: {e:?}"))?);
    }
    Ok(shells)
}

/// Each key's number in the order the keys first appear (the numbering an ordered map's
/// `entry(key).or_insert(len)` gives walking them), and how many distinct keys there are: by sorting,
/// where a map pays a search a key.
/// Keys are at most 96 bits (and fewer than 2^32 of them), each sorted with its position packed
/// below it, on every core (`par::sort`).
pub(crate) fn first_seen(keys: &[u128]) -> (Vec<usize>,usize) {
    assert!((keys.len() as u128) < 1 << 32 && keys.iter().all(|&k| k < 1 << 96),"first_seen takes 96-bit keys");
    let mut order: Vec<u128> = keys.iter().enumerate().map(|(i,&k)| k << 32 | i as u128).collect();
    crate::par::sort(&mut order);
    let at = |packed: u128| (packed & 0xffff_ffff) as usize;
    // each run of equal keys led by its first appearance
    let mut first = vec![0usize;keys.len()];
    let mut leaders = Vec::new();
    let mut k = 0;
    while k < order.len() {
        let lead = at(order[k]);
        leaders.push(lead);
        let mut j = k;
        while j < order.len() && order[j] >> 32 == order[k] >> 32 { first[at(order[j])] = lead; j += 1; }
        k = j;
    }
    crate::par::sort(&mut leaders);
    let mut number = vec![usize::MAX;keys.len()];
    for (n,&lead) in leaders.iter().enumerate() { number[lead] = n; }
    (first.iter().map(|&lead| number[lead]).collect(),leaders.len())
}

fn stl_indices(bytes: &[u8]) -> Result<(usize,Vec<[usize;3]>),String> {
    let mut keys = vec![];
    for p in stl_points(bytes)? {
        if p.iter().flatten().any(|x| !x.is_finite()) || degenerate(p[0],p[1],p[2]) {
            return Err("binary STL has a nonfinite or degenerate triangle".into());
        }
        keys.extend(p.map(|p| p.map(|v| if v == 0. { 0 } else { (v as f32).to_bits() }))
            .map(|[x,y,z]| (x as u128) << 64 | (y as u128) << 32 | z as u128));
    }
    let (numbers,count) = first_seen(&keys);
    Ok((count,numbers.chunks_exact(3).map(|t| [t[0],t[1],t[2]]).collect()))
}

// -- the mesh a viewer wants --------------------------------------------------------------------

/// **The boundary as a viewer wants it**: triangles grouped by the face they belong to, with the
/// normals each face deserves.
///
/// A boundary evaluation hands back a heap of facets.  What a viewer wants is the *faces of the
/// object* — so that clicking one selects `body.bore.wall` and not a triangle, and so that a
/// bore's wall shades as one round surface rather than sixty flats.  Both come from what the
/// pieces already carry: the path the document reaches a face by, and whether the surface it
/// belongs to is a tessellation of something curved.
///
/// This is a **flat buffer plus a table**, not a JSON blob: a mesh is tens of thousands of
/// numbers and every one of them is the same kind of number, which is what `gcs_entity_params`
/// already crosses the ABI as.  The table is small and says where each face starts.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    /// Nine doubles a triangle: three vertices, in the order the groups name.
    pub positions: Vec<f64>,
    /// Nine more, a normal per vertex — the face's own where it is flat, and the average of the
    /// facets meeting there where it is round.
    pub normals: Vec<f64>,
    pub groups: Vec<Group>,
}

/// One face of the object, as a run of triangles.
#[derive(Clone, Debug)]
pub struct Group {
    /// `body.bore.wall` — what the document calls this face.
    pub path: String,
    /// Where its triangles start, and how many: an index into `positions` divided by nine.
    pub start: usize,
    pub count: usize,
    /// A tessellation of something curved, so its normals are averaged and a viewer should shade
    /// it smooth.  A flat face says false and is shaded flat, which is what a corner needs.
    pub smooth: bool,
}

/// The boundary, welded and grouped.
///
/// The order is the face path's, so it is the document's own and not the walk's — two runs of the
/// same drawing give the same buffer, and a viewer that remembers which face was selected finds
/// it in the same place.
pub fn grouped(pieces: &[Piece]) -> Mesh {
    grouped_welded(weld(pieces))
}

/// `grouped` for pieces already welded: every shared corner the same bits and no vertex partway
/// along another's edge — a surface that came as an indexed mesh, which a weld would only search.
pub fn grouped_welded(welded: Vec<Piece>) -> Mesh {
    if welded.is_empty() {
        return Mesh::default();
    }
    let b = bounds(&welded);
    let scale = (0..3).fold(1.0f64, |m, i| m.max(b.hi[i] - b.lo[i]));
    let key = |p: [f64; 3]| {
        let g = scale * WELD;
        [(p[0] / g).round() as i64, (p[1] / g).round() as i64, (p[2] / g).round() as i64]
    };
    // by face path, so a group is a face and the order is the document's
    let mut by: BTreeMap<&str, Vec<&Piece>> = BTreeMap::new();
    for p in &welded {
        by.entry(p.path.as_str()).or_default().push(p);
    }
    let mut mesh = Mesh::default();
    for (path, group) in by {
        let smooth = group.iter().all(|p| p.smooth);
        // **a round face's normal at a vertex is the average of the facets meeting there** — but
        // only of the facets *in this face*, so the rim where a bore meets a cap stays a corner
        // and does not get smeared round it
        let mut at: BTreeMap<[i64; 3], [f64; 3]> = BTreeMap::new();
        if smooth {
            for p in &group {
                for q in &p.pts {
                    let e = at.entry(key(*q)).or_insert([0.0; 3]);
                    for k in 0..3 {
                        e[k] += p.n[k];
                    }
                }
            }
        }
        let start = mesh.positions.len() / 9;
        for p in &group {
            // the same triangulation `triangles` uses, so a mesh and an STL cut a face alike
            let inner = centroid(&p.pts);
            fan(&p.pts, |u, v, w| {
                for x in [u, v, w] {
                    mesh.positions.extend(x);
                    // a fan's own apex is interior to the facet where it is the centroid, so it
                    // takes the facet's normal whatever the face is doing at its edges
                    let n = if smooth && x != inner {
                        at.get(&key(x)).and_then(|a| plane::unit(*a)).unwrap_or(p.n)
                    } else {
                        p.n
                    };
                    mesh.normals.extend(n);
                }
            });
        }
        let count = mesh.positions.len() / 9 - start;
        if count > 0 {
            mesh.groups.push(Group { path: path.to_string(), start, count, smooth });
        }
    }
    mesh
}
