//! Closing the pieces into one shell: coincident vertices of different
//! pieces welded, and every remaining boundary loop zipped to the loop that
//! lies alongside it (a cap's ragged rim to a sheet's end column, a dying
//! column to the column born beside it). A zip joins two loops of judged
//! boundary points, so its band lies along the boundary between them; the
//! certificate still probes every one of its triangles.
use super::adjacency::Live;
use super::trim::{KeptMesh,boundary_loops,walk_loops};
use crate::space::Grid;
use crate::solid::zip_polylines;

type V3 = [f64;3];

use crate::space::distance;

/// Identify vertices within `tolerance` of one another and drop the
/// triangles that collapse. The first vertex of a group keeps its position.
pub fn weld(mesh: &mut KeptMesh,tolerance: f64) {
    let mut grid = Grid::new(tolerance.max(f64::MIN_POSITIVE)*4.);
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.vertices.len());
    let mut kept: Vec<V3> = Vec::new();
    for p in &mesh.vertices {
        // the first vertex kept within the tolerance, cell by cell
        let mut found = None;
        grid.around(*p,|i| if found.is_none() && distance(kept[i as usize],*p) <= tolerance { found = Some(i); });
        let i = match found { Some(i) => i,None => { kept.push(*p); let i = (kept.len()-1) as u32; grid.insert(*p,i); i } };
        remap.push(i);
    }
    mesh.vertices = kept;
    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    let mut sheet = Vec::with_capacity(mesh.sheet.len());
    for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) {
        let t = t.map(|v| remap[v as usize]);
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] { continue; }
        triangles.push(t); sheet.push(*s);
    }
    mesh.triangles = triangles; mesh.sheet = sheet;
}

/// Merge every pair of vertices closer than `shortest`, in the order of their indices,
/// each into the one of the two with more triangles on it, refusing a merge that would
/// turn any triangle round either over or flatten one that does not vanish
/// with the pair. Two samplings of one crease, a
/// fragment's corner beside its neighbour's, a cut's crossing a whisker
/// from a column point, a cap's rim vertex beside a plane's corner: all
/// land within a sagitta of each other, joined by an edge or not, and
/// nothing thinner than a sagitta is a feature of the boundary. Returns
/// how many pairs were merged.
pub fn collapse_short_edges(mesh: &mut KeptMesh,shortest: f64) -> usize {
    use crate::space::stable_normal;
    let mut collapsed = 0;
    loop {
        // the vertices in use, and every pair of them within the distance
        let mut used = vec![false;mesh.vertices.len()];
        for t in &mesh.triangles { for &v in t { used[v as usize] = true; } }
        let mut grid = Grid::new(shortest.max(f64::MIN_POSITIVE));
        for (v,p) in mesh.vertices.iter().enumerate() { if used[v] { grid.insert(*p,v as u32); } }
        let mut pairs: Vec<(f64,u32,u32)> = Vec::new();
        for (k,list) in grid.cells() {
            grid.in_keys(k.map(|x| x-1),k.map(|x| x+1),|b| for &a in list {
                if b <= a { continue; }
                let l = distance(mesh.vertices[a as usize],mesh.vertices[b as usize]);
                if l < shortest { pairs.push((l,a,b)); }
            });
        }
        if pairs.is_empty() { break; }
        // by the pair's vertices: ordered by length, pairs made equal by symmetry (a plane's
        // grid, a fixed point's fan) took the order rounding gave their lengths, and which merges
        // a round made with them
        pairs.sort_by(|p,q| p.1.cmp(&q.1).then(p.2.cmp(&q.2)));
        pairs.dedup_by(|p,q| p.1 == q.1 && p.2 == q.2);
        // Which triangles each vertex is on, at the round's start. A merge relabels only its
        // absorbed vertex's triangles, and a vertex a merge has touched takes no further part
        // in the round, so an untouched vertex's triangles are still these (the degenerate
        // ones a merge makes go only at the round's end, and are counted as before).
        let mut on: Vec<Vec<usize>> = vec![Vec::new();mesh.vertices.len()];
        for (t,tri) in mesh.triangles.iter().enumerate() {
            for k in 0..3 { if !tri[..k].contains(&tri[k]) { on[tri[k] as usize].push(t); } }
        }
        let mut done_any = false;
        let mut touched: std::collections::BTreeSet<u32> = Default::default();
        for (_,a,b) in pairs {
            // one merge per vertex per round, so the distances stay true
            if touched.contains(&a) || touched.contains(&b) { continue; }
            // the pair meets at the vertex with more triangles on it, the
            // more constrained of the two: a polyhedral case keeps its
            // exact corners, and a crease point moves onto the column
            let count = |v: u32| on[v as usize].len();
            let (a,b) = if count(b) > count(a) { (b,a) } else { (a,b) };
            let mid: V3 = mesh.vertices[a as usize];
            let mut ok = true;
            for &i in on[a as usize].iter().chain(&on[b as usize]) {
                let t = &mesh.triangles[i];
                let on_a = t.contains(&a); let on_b = t.contains(&b);
                if !(on_a || on_b) || (on_a && on_b) { continue; }
                let before = t.map(|v| mesh.vertices[v as usize]);
                let after = t.map(|v| if v == a || v == b { mid } else { mesh.vertices[v as usize] });
                match (stable_normal(before[0],before[1],before[2]),stable_normal(after[0],after[1],after[2])) {
                    (Some(n0),Some(n1)) if n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2] > 0.5 => {}
                    _ => { ok = false; break; }
                }
            }
            if !ok { continue; }
            mesh.vertices[a as usize] = mid;
            for &i in &on[b as usize] { for v in mesh.triangles[i].iter_mut() { if *v == b { *v = a; } } }
            touched.insert(a); touched.insert(b);
            collapsed += 1; done_any = true;
        }
        let mut triangles = Vec::with_capacity(mesh.triangles.len());
        let mut sheet = Vec::with_capacity(mesh.sheet.len());
        for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) { if t[0] != t[1] && t[1] != t[2] && t[2] != t[0] { triangles.push(*t); sheet.push(*s); } }
        mesh.triangles = triangles; mesh.sheet = sheet;
        dedupe(mesh);
        if !done_any { break; }
    }
    collapsed
}

/// The farthest any vertex of one loop is from the other loop's polyline.
fn apart(a: &[V3],b: &[V3]) -> f64 {
    let segment = |p: V3,x: V3,y: V3| -> f64 {
        let d = [y[0]-x[0],y[1]-x[1],y[2]-x[2]]; let w = [p[0]-x[0],p[1]-x[1],p[2]-x[2]];
        let l = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
        let f = if l > 0. { ((w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l).clamp(0.,1.) } else { 0. };
        distance(p,[x[0]+f*d[0],x[1]+f*d[1],x[2]+f*d[2]])
    };
    let to = |from: &[V3],onto: &[V3]| from.iter().map(|p| (0..onto.len()).map(|k| segment(*p,onto[k],onto[(k+1)%onto.len()])).fold(f64::INFINITY,f64::min)).fold(0_f64,f64::max);
    to(a,b).max(to(b,a))
}

/// Fill every boundary loop of three or four vertices (a hole the size of
/// a triangle, never a seam), zip every other loop to the nearest other
/// loop within `within` of it (both ways), each pair once, the new
/// triangles wound against the edges they close; then every loop left that
/// doubles back on itself (a cap's ragged rim walked out and the sheet's
/// column walked back, joined at their ends) is zipped across, arc to arc. Two runs along one curve
/// sampled twice (a cap's rim on the tool's edge and the sheet's column on
/// the same edge) are first made one by `split_at_vertices` within `split`,
/// so no band is ever laid between them. Returns the pairs zipped and the
/// loops left unpaired.
/// A boundary walk meeting a vertex twice is two walks joined there, and this is which. Two
/// pieces of surface touching at one point and sharing no edge (a cap's cut crossing the tool's
/// rim leaves exactly that) give one walk that passes through the point twice; it bounds no disc,
/// so nothing could fill or pair it and it was refused whole. Cut at the repeat, each piece is a
/// walk of the same boundary edges that does bound one, and the passes below take them as they
/// take any other. The cut is on vertex identity, not on any tolerance: where the walk returns to
/// a vertex, the stretch between is closed already.
pub fn unpinch(l: Vec<u32>) -> Vec<Vec<u32>> {
    let mut out = Vec::new();
    let mut walk: Vec<u32> = Vec::new();
    let mut at: std::collections::BTreeMap<u32,usize> = Default::default();
    for v in l {
        if let Some(&i) = at.get(&v) {
            // the stretch from where `v` was last seen to here closes on itself
            let closed: Vec<u32> = walk.split_off(i);
            for u in &closed { at.remove(u); }
            if closed.len() >= 3 { out.push(closed); }
        }
        at.insert(v,walk.len());
        walk.push(v);
    }
    if walk.len() >= 3 { out.push(walk); }
    out
}

/// `closeable` is asked of a loop no pass could pair, with its points and the outward normal of
/// the mesh triangle owning each of its edges: a loop the field says the boundary spans is a hole
/// and is filled however many vertices it has, and one it does not is left to be refused. It is
/// asked last, so every case that closed without it closes the same way.
pub fn rim_zip(mesh: &mut KeptMesh,within: f64,split: f64,thin: f64,closeable: &mut dyn FnMut(&[V3],&[V3]) -> bool) -> (usize,Vec<Vec<u32>>) {
    // Welding here, before anything is laid, was tried and **refused** (2026-09-12). The reasoning
    // was sound and the local result good: the rounds below weld only *after* `zip_round` lays its
    // bands, so the weld never sees the mesh as it arrives, and those bands are what it cannot then
    // merge past — of the 146 doublings the junction merges would create on the tumbling cylinder,
    // 106 involve a band the zip itself laid. Welding first gave that case 821 vertices merged,
    // **17 loops to 14**, volume held at 9.2905 against 9.2929, every directed edge walked once.
    //
    // It is refused for what it does to the **sliding dumbbell**, which the five-case gate caught
    // and no single-case measurement could: that case went from refusing at `UnpairedRim {[4,4]}`
    // with nothing failing the certificate to `NonManifoldVertex { vertex: 724 }` with **20
    // triangles failing it**. Closing the two loops let the pipeline reach `certify` and arrive
    // broken. The cause is that this weld's guard is **edge**-based — `walks_once` checks directed
    // edges — so it admits a vertex whose triangle fan is no disc, which 821 merges duly produced.
    // A case that closes on a refused certificate is a failure, never a warning. The prism and the
    // turned box also moved (1436 to 1404, 1304 to 1296 triangles) while staying closed and
    // certified.
    split_at_vertices(mesh,split);
    let mut pairs = 0;
    // A round's passes all read the one walk of the boundary it began with, so a loop another
    // pass made — or left where a band was declined — is never looked at again: a three-vertex
    // hole the first pass fills outright survives the round because it did not exist when that
    // pass ran. So the boundary is walked again and the passes run until a round lays nothing.
    //
    // What ends this is the break below — a round that lays nothing, welds nothing and collapses
    // nothing — and not the count, which was measured to be the binding constraint instead: on the
    // tumbling cylinder's finished mesh the small-hole fill would still have taken two more loops.
    for _ in 0..32 {
        let before = mesh.triangles.len();
        pairs += zip_round(mesh,within,closeable);
        dedupe(mesh);
        // Two samplings of one boundary junction, which no split can pair, are merged instead —
        // guarded, so a merge that would leave an edge walked twice the same way is refused. The
        // boundary is then walked again: what the merge joins may be a loop a pass can take.
        let welded = weld_boundary_ends(mesh,split);
        // and a needle standing on the boundary is taken away, since a quarter of its height is
        // nothing and no vertex beside it can ever split the edge it owns
        let needles = collapse_needles(mesh,thin,within);
        if mesh.triangles.len() == before && welded == 0 && needles == 0 { break; }
    }
    // One last split, over the finished mesh. The split at the top runs before any round, so it
    // never sees the T-junctions the zip's own laying makes — a band's vertex landing on an edge
    // another triangle spans. Measured on the tumbling cylinder once `caps` stopped dropping the
    // z = 0 band: six three-vertex loops, each a zero-area *gap* between three triangles (no facet
    // on their corners at all) whose vertices are collinear with one projecting inside the opposite
    // edge — at +0.3333, +0.6603, +0.2731, +0.3408 and +0.4342, off the line by 1e-9 to 1e-16 — and
    // two of them the field called holes. Filling such a loop is rightly refused: its fan triangles
    // are degenerate, and none of the loops is blocked by an already-walked edge.
    //
    // Placed *inside* the round this is worse, and that was measured before it was believed: it
    // interleaves with the zip and changes what the zip then pairs, giving 23 loops against 21,
    // volume 9.2581 against 9.2929, and still one loop the field called a hole. Run once here it
    // takes them cleanly — 4 splits, 21 loops to 17, none the field calls a hole — and the zip has
    // nothing left to add afterwards (a full `rim_zip` over the split mesh paired 0), so there is
    // no case for looping again.
    split_where(mesh,split,&|_,_| true,&|_| true);
    // What is still open is read off the finished mesh, not the walk this began with: a pass that
    // closes part of a loop leaves the rest of it boundary, and the opening walk would name
    // vertices whose edges are now used twice.
    let unpaired: Vec<Vec<u32>> = boundary_loops(&mesh.triangles).into_iter().flat_map(unpinch).collect();
    (pairs,unpaired)
}

/// One round of the passes over the boundary as it stands: the small-hole fill, the pairing, the
/// slits, and last what the field says is a hole. Returns how many loops it closed or paired.
fn zip_round(mesh: &mut KeptMesh,within: f64,closeable: &mut dyn FnMut(&[V3],&[V3]) -> bool) -> usize {
    // Every directed edge the mesh walks as the round begins, kept up to date as the passes lay
    // their bands: a band along an edge already walked would give it a third use, so such a
    // triangle is left out. Read afresh every round, since `dedupe` may since have dropped a
    // triangle whose edges an earlier round recorded, and a stale record declines a good band.
    let mut walked: std::collections::BTreeSet<(u32,u32)> = Default::default();
    for t in &mesh.triangles { for k in 0..3 { walked.insert((t[k],t[(k+1)%3])); } }
    // Every corner set the mesh already has. `dedupe` drops a triangle repeating one after each
    // round, and a band wound the other way round an existing triangle's three vertices walks none
    // of its directed edges, so it passes the test above and is dropped again: the loop it closed
    // comes back, every round, and the count of rounds is all that ends it. `lay` and `dedupe` must
    // agree. Measured on the tumbling cylinder: the one loop the field still calls `Spanned` is
    // exactly this — three boundary edges used once each with a triangle already on those three
    // vertices, so that triangle is their only user and the loop bounds a lone facet with no
    // neighbour. Laying it again the other way makes a zero-volume flap, not a closed hole.
    // `facets`, not `corners`: the slit pass below names a loop's reflex corners that.
    let mut facets: std::collections::BTreeSet<[u32;3]> =
        mesh.triangles.iter().map(|t| { let mut k = *t; k.sort(); k }).collect();
    let loops: Vec<Vec<u32>> = boundary_loops(&mesh.triangles).into_iter().flat_map(unpinch).collect();
    // the directed boundary edges as the mesh walks them
    let mut boundary: std::collections::BTreeSet<(u32,u32)> = Default::default();
    for l in &loops { for k in 0..l.len() { boundary.insert((l[k],l[(k+1)%l.len()])); } }
    let points: Vec<Vec<V3>> = loops.iter().map(|l| l.iter().map(|&v| mesh.vertices[v as usize]).collect()).collect();
    // each loop's box: two loops whose boxes are farther apart than `within` are, too
    let boxes: Vec<(V3,V3)> = points.iter().map(|p| p.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),q| (std::array::from_fn(|k| lo[k].min(q[k])),std::array::from_fn(|k| hi[k].max(q[k]))))).collect();
    let gap = |i: usize,j: usize| -> f64 { let ((a0,a1),(b0,b1)) = (boxes[i],boxes[j]); (0..3).map(|k| (b0[k]-a1[k]).max(a0[k]-b1[k]).max(0.).powi(2)).sum::<f64>().sqrt() };
    let mut paired = vec![false;loops.len()];
    let mut pairs = 0;
    // A loop that visits a vertex twice is pinched there (slits touching at a corner, where the
    // walk round a fan that is no disc breaks off), and a two-vertex walk is a fragment of one:
    // neither bounds a hole a band can close, and zipped they lay bands over triangles already
    // there. They are left unpaired, to be refused.
    let simple = |l: &Vec<u32>| l.len() >= 3 && { let mut s = l.clone(); s.sort_unstable(); s.windows(2).all(|w| w[0] != w[1]) };
    // Every band triangle has an edge along a loop, and a seam must walk it against the way the
    // mesh already does; each triangle is turned by its own loop edge, since a zip of arcs winds
    // each arc for itself. Said once here, so the test of whether the mesh can take a triangle
    // and the laying of it cannot come to disagree about which way it goes.
    // Orienting a band by the **live** `walked` set instead — taking whichever of the two windings
    // walks no directed edge the mesh already walks, and falling back to the rule below only where
    // neither or both clash — was tried and **changed nothing at all**: the tumbling cylinder came
    // back at the same 6713 triangles, the same volume 9.3057, the same loops [4,7,5,7,4,3,3], and
    // the certificate's identical 169 `Reversed` with the identical split (142 bands, and 3/11/1/2/10
    // over sheets 0/10/2/4/9) down to the same reversed area of 0.8267. So the `unwrap_or` default
    // below is **not** what produces the reversed bands: on every band actually laid, the clash test
    // and the boundary test agree. The reversals come from somewhere else — the band's own
    // construction, or the sheet it attaches to.
    let wind = |tri: [u32;3]| -> [u32;3] {
        let flip = (0..3).find_map(|k| {
            let (p,q) = (tri[k],tri[(k+1)%3]);
            if boundary.contains(&(p,q)) { Some(true) } else if boundary.contains(&(q,p)) { Some(false) } else { None }
        }).unwrap_or(false);
        let mut tri = tri; if flip { tri.swap(1,2); } tri
    };
    // Whether the mesh can take this triangle: it has area, and no triangle walks any of its
    // directed edges already. `walked` is a parameter rather than a capture so a caller may ask
    // this while holding it.
    let takes = |mesh: &KeptMesh,walked: &std::collections::BTreeSet<(u32,u32)>,
        corners: &std::collections::BTreeSet<[u32;3]>,tri: [u32;3]| -> bool {
        let tri = wind(tri);
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { return false; }
        let [p,q,r] = tri.map(|v| mesh.vertices[v as usize]);
        !crate::space::degenerate(p,q,r) && !(0..3).any(|k| walked.contains(&(tri[k],tri[(k+1)%3])))
            && { let mut key = tri; key.sort(); !corners.contains(&key) }
    };
    // Returns how much of the band it took. A band that lays nothing has closed nothing, and the
    // loop it was claimed for must be left to a later pass: measured on the tumbling cylinder, 29
    // of its 33 loops were claimed by the pairing or the slit pass and laid **nothing**, so only
    // two of them ever reached the field pass at all.
    let lay = |mesh: &mut KeptMesh,walked: &mut std::collections::BTreeSet<(u32,u32)>,
        corners: &mut std::collections::BTreeSet<[u32;3]>,band: Vec<[u32;3]>| -> usize {
        let mut laid = 0;
        for tri in band {
            let tri = wind(tri);
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { continue; }
            // a zero-area triangle (rim vertices collinear with the column
            // along a straight edge) closes nothing
            let [p,q,r] = tri.map(|v| mesh.vertices[v as usize]);
            // Measured on every failing case: the bands declined here have altitude **exactly**
            // zero with sides that sum exactly (a + b = c to the last digit), so their vertices
            // are exactly collinear. The loops they would close are zero-width slits — the walk
            // runs out along a line and back along collinear points — and no triangle can span
            // one. Such a loop is collapsed, not filled; see the note in `rim_zip`.
            if crate::space::degenerate(p,q,r) { continue; }
            // Measured on every failing case: a band declined here is never free wound the other
            // way, and the edge that blocks it is never a loop edge but an interior one already
            // used twice — by a triangle on the band's own three vertices, wound the other way.
            // The band is a facet the mesh already has, so the loop is no hole: it is a false
            // boundary, where two samplings of one surface meet at different vertices and the
            // edges never paired. Nothing may be laid there; the pairing belongs upstream.
            if (0..3).any(|k| walked.contains(&(tri[k],tri[(k+1)%3]))) { continue; }
            // and a facet the mesh already has, which `dedupe` would drop again: the insert is the
            // test, so a corner set is claimed exactly once
            let mut key = tri; key.sort();
            if !corners.insert(key) { continue; }
            for k in 0..3 { walked.insert((tri[k],tri[(k+1)%3])); }
            mesh.triangles.push(tri); mesh.sheet.push(u32::MAX);
            laid += 1;
        }
        laid
    };
    // A loop of three or four vertices is a hole the size of a triangle, never a seam: it is
    // filled outright, fanned against its own walk, and only where the mesh can take the whole
    // fan. A loop it cannot is left unpaired rather than counted as filled — claiming it hid the
    // hole from the report, though never from the shell.
    //
    // A quad has two diagonals, and where the fan's own is already an interior edge two triangles
    // walk, the other is often free. Trying each start vertex was measured and **refused**: it
    // closed the 30° tilted cylinder (every edge used twice, no boundary left) by laying two
    // facets the certificate then read `Reversed`, the field giving exterior a probe inside them
    // and material a probe outside (+0.0342/-0.0368). Such a loop is no hole but a false
    // boundary, and the free diagonal reaches across open air. A fan that does not fit is the
    // construction saying so; nothing may be invented there.
    for i in 0..loops.len() {
        if paired[i] || !simple(&loops[i]) || loops[i].len() > 4 { continue; }
        let (l,n) = (&loops[i],loops[i].len());
        let fan: Vec<[u32;3]> = (1..n-1).map(|k| [l[0],l[k+1],l[k]]).collect();
        if !fan.iter().all(|&tri| takes(mesh,&walked,&facets,tri)) { continue; }
        paired[i] = true; pairs += 1;
        lay(mesh,&mut walked,&mut facets,fan);
    }
    for i in 0..loops.len() {
        if paired[i] || !simple(&loops[i]) { continue; }
        let candidates: Vec<(f64,usize)> = (0..loops.len()).filter(|&j| j != i && !paired[j] && simple(&loops[j]) && gap(i,j) <= within*(1.+1e-9))
            .map(|j| (apart(&points[i],&points[j]),j)).filter(|(d,_)| *d <= within).collect();
        // the nearest, a loop within rounding of it and before it taken instead
        let least = candidates.iter().map(|c| c.0).fold(f64::INFINITY,f64::min);
        let best = candidates.into_iter().find(|c| c.0 <= least+1e-9*within.max(1.));
        let Some((_,j)) = best else { continue };
        let band = zip_loops(&loops[i],&loops[j],&mesh.vertices);
        // Committed only on a band that closed something. Marking both loops paired first, as this
        // did, consumed them: a band `lay` declines whole took two loops out of every later pass
        // and out of every later round too, the pairing being decided the same way each time.
        if lay(mesh,&mut walked,&mut facets,band) == 0 { continue; }
        paired[i] = true; paired[j] = true; pairs += 1;
    }
    // where a walk turns back on itself: a slit's end, read the same way by both passes below
    let corners_of = |p: &[V3]| -> Vec<usize> {
        let n = p.len();
        (0..n).filter(|&k| {
            let (a,b,c) = (p[(k+n-1)%n],p[k],p[(k+1)%n]);
            let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-b[0],c[1]-b[1],c[2]-b[2]]);
            let (lu,lw) = (distance(a,b),distance(b,c));
            lu > 0. && lw > 0. && (u[0]*w[0]+u[1]*w[1]+u[2]*w[2])/(lu*lw) < -0.5
        }).collect()
    };
    // slits: a loop turning back on itself at exactly two corners, its two
    // arcs alongside each other
    for i in 0..loops.len() {
        if paired[i] || loops[i].len() < 4 || !simple(&loops[i]) { continue; }
        let (l,p) = (&loops[i],&points[i]);
        let n = l.len();
        let corners: Vec<usize> = corners_of(p);
        // Two reflex corners are a slit's ends wherever the sampling turns sharply at both; a
        // slit whose ends are rounded, or sampled at one vertex, has fewer and is still two arcs
        // running alongside each other, its ends the loop's own diameter. What admits a slit is
        // the `apart` test below, so reading the ends more widely cannot take in a loop whose
        // arcs do not lie together.
        let (c0,c1) = if corners.len() == 2 { (corners[0],corners[1]) } else {
            let mut best = (f64::NEG_INFINITY,0,0);
            for a in 0..n { for b in a+1..n { let d = distance(p[a],p[b]); if d > best.0 { best = (d,a,b); } } }
            (best.1,best.2)
        };
        if c0 == c1 { continue; }
        let arc_a: Vec<u32> = (c0..=c1).map(|k| l[k]).collect();
        let arc_b: Vec<u32> = (c1..=c0+n).map(|k| l[k%n]).rev().collect();
        let pts = |arc: &[u32]| arc.iter().map(|&v| mesh.vertices[v as usize]).collect::<Vec<_>>();
        let (pa,pb) = (pts(&arc_a),pts(&arc_b));
        if apart(&pa,&pb) > within { continue; }
        let band: Vec<[u32;3]> = zip_polylines(&pa,&pb,false).into_iter().map(|t| t.map(|(on_b,k)| if on_b { arc_b[k as usize] } else { arc_a[k as usize] })).collect();
        // committed only on a band that closed something, as the pairing is
        if lay(mesh,&mut walked,&mut facets,band) == 0 { continue; }
        paired[i] = true; pairs += 1;
    }
    // A wide seam that folds back on itself looks like a slit — most of its vertices have a
    // counterpart a spacing away further along the walk (60 of 82 and 44 of 64 on the tumbling
    // cylinder) — and sewing it side to side along that correspondence was tried and **refused**.
    // It should have added no area, being two sides joined rather than a span; instead the
    // tumbling cylinder's volume rose 6.6509 to 6.6865, half a percent of surface invented where
    // the field says `Open` and no boundary runs, while its loops went 33 to 39 and its seams only
    // 42/44 to 40/36. The sides fold but do not lie close enough to sew, so the `Open` verdict
    // stands and the surface is genuinely missing: the tracer's to generate, not the stitch's to
    // invent. Pairing the arcs by consecutive index instead shattered them outright (2 seams
    // became 73 loops).
    // Last, and only for what nothing else could pair: a loop the field says the boundary runs
    // through the whole of is a hole, whatever its vertex count, and is fanned shut. The fill
    // above takes three and four vertices because a hole that small can be nothing else; this
    // asks instead of counting, so a wide seam across several sheets is still refused.
    let owners = super::adjacency::Edges::new(&mesh.triangles);
    for i in 0..loops.len() {
        if paired[i] || !simple(&loops[i]) { continue; }
        let l = &loops[i];
        let normals: Vec<V3> = (0..l.len()).map(|k| {
            let (a,b) = (l[k],l[(k+1)%l.len()]);
            owners.owner(a,b).and_then(|t| {
                let [p,q,r] = mesh.triangles[t].map(|v| mesh.vertices[v as usize]);
                crate::space::stable_normal(p,q,r)
            }).unwrap_or([0.;3])
        }).collect();
        if !closeable(&points[i],&normals) { continue; }
        // Fanned from a new vertex at the loop's own centroid: the very triangulation the field
        // judged, and the only one whose chords cannot already be edges of the mesh. A fan from a
        // loop vertex lays the chord l[0]-l[k], which a triangle beside the loop often walks
        // already, and the shell then has an edge used three times.
        let n = l.len();
        let apex: V3 = std::array::from_fn(|k| points[i].iter().map(|p| p[k]).sum::<f64>()/n as f64);
        mesh.vertices.push(apex);
        let a = (mesh.vertices.len()-1) as u32;
        let fan: Vec<[u32;3]> = (0..n).map(|k| [a,l[k],l[(k+1)%n]]).collect();
        // The standing puzzle — a loop the field calls `Spanned` refused here all the same — was
        // neither of the two causes once named in this place. Measured on the tumbling cylinder:
        // of its 33 loops, 29 never reached this pass, the pairing and the slit pass having marked
        // them paired *before* calling `lay`, which then declined every triangle of the band; only
        // the two wide seams arrived, and the field calls those `Open`. Both passes now commit
        // only on a band that laid something. Of those 33 the field calls 22 `Open` (no boundary
        // across the span: that surface is genuinely missing, and generating it is the tracer's,
        // not the stitch's), 10 `Spanned`, and refuses one with `ReversedNormal`.
        //
        // A fan that lays nothing leaves its apex unused, so the vertex goes back: nothing can
        // reference it, and a stray vertex would travel into the shell and the export.
        if lay(mesh,&mut walked,&mut facets,fan) == 0 { mesh.vertices.pop(); continue; }
        paired[i] = true; pairs += 1;
    }
    pairs
}

/// The triangles zipping two boundary loops of one mesh, wound as a strip
/// from the first loop to the second walked the first loop's way. The two
/// loops of a seam between outward pieces run opposite ways, so the second
/// is reversed before the strip zip pairs points along it; zipped as given
/// the rungs cross and the band dips deep into the material. Loops that
/// share no vertex are zipped closed; loops that share vertices (a
/// tessellation vertex on the contact curve, a fan meeting a face on a tool
/// edge) are zipped arc by arc between the shared vertices, each arc pair
/// open and each arc of the second taken start to end like the first's, so a
/// shared vertex is never made a rung's end and no loop edge is doubled.
pub fn zip_loops(a: &[u32],b: &[u32],vertices: &[V3]) -> Vec<[u32;3]> {
    let pts = |l: &[u32]| l.iter().map(|&v| vertices[v as usize]).collect::<Vec<_>>();
    let shared: Vec<usize> = (0..a.len()).filter(|&i| b.contains(&a[i])).collect();
    if shared.is_empty() {
        // the second loop the way whose rungs are shortest: two loops of one
        // seam run opposite ways when both pieces face outward, and zipped
        // the wrong way round the rungs cross and the band dips into the material
        let rungs = |b: &[u32]| -> (f64,Vec<[u32;3]>) {
            let tris: Vec<[u32;3]> = zip_polylines(&pts(a),&pts(b),true).into_iter().map(|t| t.map(|(on_b,k)| if on_b { b[k as usize] } else { a[k as usize] })).collect();
            let cost: f64 = tris.iter().map(|t| { let [p,q,r] = t.map(|v| vertices[v as usize]); distance(p,q)+distance(q,r)+distance(r,p) }).sum();
            (cost,tris)
        };
        let reversed: Vec<u32> = b.iter().rev().copied().collect();
        let (forward,backward) = (rungs(b),rungs(&reversed));
        return if backward.0 <= forward.0+1e-9*forward.0.max(1.) { backward.1 } else { forward.1 };
    }
    // a from its first shared vertex, cut at every shared vertex; each arc
    // is paired with whichever of b's two arcs between the same vertices
    // lies alongside it
    let a: Vec<u32> = a[shared[0]..].iter().chain(&a[..shared[0]]).copied().collect();
    let shared_a: Vec<usize> = (0..a.len()).filter(|&i| b.contains(&a[i])).collect();
    let nb = b.len();
    let along = |i0: usize,i1: usize,forward: bool| -> Vec<u32> {
        // from b[i0] to b[i1] stepping forward or backward round the loop,
        // the whole loop when the two coincide
        let mut arc = vec![b[i0]];
        let mut i = i0;
        loop {
            i = if forward { (i+1)%nb } else { (i+nb-1)%nb };
            arc.push(b[i]);
            if i == i1 { break; }
        }
        arc
    };
    let mut out = Vec::new();
    let cuts_a: Vec<usize> = shared_a.iter().copied().chain(std::iter::once(a.len())).collect();
    for w in cuts_a.windows(2) {
        let arc_a: Vec<u32> = (w[0]..=w[1]).map(|i| a[i%a.len()]).collect();
        let (start,end) = (arc_a[0],arc_a[arc_a.len()-1]);
        let (i0,i1) = (b.iter().position(|&x| x == start).unwrap(),b.iter().position(|&x| x == end).unwrap());
        let (forward,backward) = (along(i0,i1,true),along(i0,i1,false));
        let pa = pts(&arc_a);
        let (af,ab) = (apart(&pa,&pts(&forward)),apart(&pa,&pts(&backward)));
        let arc_b = if af <= ab+1e-9*ab.max(1.) { forward } else { backward };
        for t in zip_polylines(&pa,&pts(&arc_b),false) {
            out.push(t.map(|(on_b,k)| if on_b { arc_b[k as usize] } else { arc_a[k as usize] }));
        }
    }
    out
}

/// Resolve the T-junctions between boundary edges: a boundary vertex lying
/// within `tolerance` of the interior of a boundary edge it does not end
/// splits the triangle on that edge in two at the vertex (which stays where
/// it is), so two runs along one curve sampled twice (a cap's rim on the
/// tool's edge and a sheet's column on the same edge, a planar fragment's
/// corner on its neighbour's edge) come to share every vertex and every edge
/// between them, where a band between them would be zero-area or a fold.
/// With the tolerance a sagitta or so, either sampling's chords hold the
/// other's vertices.
pub fn split_at_vertices(mesh: &mut KeptMesh,tolerance: f64) { split_where(mesh,tolerance,&|_,_| true,&|_| true); }

/// `split_at_vertices` over the boundary edges `edge_ok` admits, at the
/// boundary vertices `vertex_ok` admits. Returns how many splits were made.
pub fn split_where(mesh: &mut KeptMesh,tolerance: f64,edge_ok: &dyn Fn(u32,u32) -> bool,vertex_ok: &dyn Fn(u32) -> bool) -> usize {
    let mut made = 0;
    // the mesh's edges and fans, kept as the rounds split its triangles
    let mut live = Live::new(mesh.vertices.len(),&mesh.triangles);
    for _ in 0..64 {
        let loops = walk_loops(&mesh.triangles,live.boundary().clone(),|a,b| live.uses(a,b) == 1,|a,b| live.owner(a,b));
        let boundary_vertices: Vec<u32> = loops.iter().flatten().copied().filter(|&v| vertex_ok(v)).collect();
        // the boundary vertices by position, filed by their place in `boundary_vertices` so an
        // edge reads those beside it in that order; cells the size of a boundary edge
        let (mut length,mut count) = (0.,0);
        for l in &loops { for k in 0..l.len() { length += distance(mesh.vertices[l[k] as usize],mesh.vertices[l[(k+1)%l.len()] as usize]); count += 1; } }
        let mut grid = Grid::new((length/count.max(1) as f64).max(tolerance).max(f64::MIN_POSITIVE));
        for (i,&v) in boundary_vertices.iter().enumerate() { grid.insert(mesh.vertices[v as usize],i as u32); }
        let pad = tolerance*(1.+1e-6)+1e-12;
        let mut near: Vec<u32> = Vec::new();
        let mut split: Vec<(usize,u32,u32,u32)> = Vec::new(); // triangle, edge a, edge b, vertex
        let mut used: std::collections::BTreeSet<usize> = Default::default();
        // the directed edges this round's splits will make: `live` learns of them only when the
        // batch is applied, so they are claimed here or a second split makes one of them again
        let mut claimed: std::collections::BTreeSet<(u32,u32)> = Default::default();
        for l in &loops { for k in 0..l.len() {
            let (a,b) = (l[k],l[(k+1)%l.len()]);
            if !edge_ok(a,b) { continue; }
            let Some(t) = live.owner(a,b) else { continue };
            if used.contains(&t) { continue; }
            let own = mesh.triangles[t];
            let (pa,pb) = (mesh.vertices[a as usize],mesh.vertices[b as usize]);
            let d = [pb[0]-pa[0],pb[1]-pa[1],pb[2]-pa[2]];
            let l2 = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
            if l2 <= 0. { continue; }
            // the triangle's height over this edge: a vertex farther off the
            // edge than a quarter of it would fold the split (a pole's fan of
            // slivers, each vertex a whisker from its neighbours' edges)
            let height = {
                let c = own.iter().copied().find(|&v| v != a && v != b).map(|v| mesh.vertices[v as usize]).unwrap_or(pa);
                let w = [c[0]-pa[0],c[1]-pa[1],c[2]-pa[2]];
                let f = (w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l2;
                distance(c,[pa[0]+f*d[0],pa[1]+f*d[1],pa[2]+f*d[2]])
            };
            // the vertex nearest the middle of the edge, of another loop
            let mut best: Option<(f64,u32)> = None;
            // only a vertex within the tolerance of the edge can split it
            near.clear();
            grid.in_box(std::array::from_fn(|k| pa[k].min(pb[k])-pad),std::array::from_fn(|k| pa[k].max(pb[k])+pad),|i| near.push(i));
            near.sort_unstable();
            for &v in near.iter().map(|&i| &boundary_vertices[i as usize]) {
                if own.contains(&v) { continue; }
                let p = mesh.vertices[v as usize];
                let w = [p[0]-pa[0],p[1]-pa[1],p[2]-pa[2]];
                let f = (w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l2;
                // A vertex on this edge's line but beyond its ends is skipped here, before `off` is
                // ever measured — and measured, that is the largest refusal of all on the failing
                // cases (29 against 14, 10 and 6 on the 30° cylinder; 10122 against 830 and 2719
                // on the tumbling one). Those are the collinear T-junctions: the opposite side's
                // samples sit just past an end, so no edge they lie *within* is ever offered them.
                // The answer is to give such a vertex to the edge whose span does hold it, or to
                // merge it into the near endpoint — never to widen a tolerance.
                if f <= 0. || f >= 1. { continue; }
                let foot = [pa[0]+f*d[0],pa[1]+f*d[1],pa[2]+f*d[2]];
                let off = distance(p,foot);
                // a vertex at an end is that end (welded already, or as
                // near as makes no triangle); one merely near it splits
                // and it must lie along the edge, not beside an end: a
                // vertex a whisker from the edge's start but as far from
                // its line folds the split's first triangle
                let along = f.min(1.-f)*l2.sqrt();
                // Measured, and the gate is right: the candidates it turns away on the 30° cylinder
                // sit 0.0064 to 0.0190 off the line of an edge 0.385 long — 1.7% to 5% of it, not
                // rounding — while the owning triangle is a needle only 0.00625 tall. `off` there
                // equals or exceeds the owner's whole height, so such a split really would fold
                // it, and no floor on `off` is defensible. What wants removing is the needle: at
                // 0.0063 across it sits just above `shortest_edge`, so `collapse_short_edges`
                // never takes it and `drop_doubled_slivers` only takes one doubled over another
                // source's edge. A lone needle has nothing to remove it.
                // nearer the middle by more than rounding, or the first found
                if off <= tolerance && off <= height/4. && off <= along/4. && best.map_or(true,|(g,_)| (f-0.5).abs() < (g-0.5).abs()-1e-9) { best = Some((f,v)); }
            }
            // the split must leave two triangles facing the way the one did,
            // along edges no triangle already walks that way
            if let Some((_,v)) = best {
                let c = own.iter().copied().find(|&x| x != a && x != b).unwrap_or(a);
                // Splitting [a,b,c] at v makes exactly these four directed edges; `c`->`a` and
                // `b`->`c` the triangle walked already. A triangle walking any of them would
                // then walk it twice the same way, and the edge be used three times.
                let makes = [(a,v),(v,b),(v,c),(c,v)];
                if makes.iter().any(|&(p,q)| live.walks(p,q)) { continue; }
                // `live` is told of this round's splits only when the batch is applied, so they
                // are claimed here too: two boundary edges out of one vertex, split at the same
                // vertex, each walk `a`->`v`, and being distinct triangles `dedupe` keeps both.
                // A candidate whose edges another has claimed waits for the next round.
                if makes.iter().any(|e| claimed.contains(e)) { continue; }
                let (pc,pv) = (mesh.vertices[c as usize],mesh.vertices[v as usize]);
                let normal = |p: V3,q: V3,r: V3| -> V3 { let (u,w) = ([q[0]-p[0],q[1]-p[1],q[2]-p[2]],[r[0]-p[0],r[1]-p[1],r[2]-p[2]]); [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]] };
                let (n0,n1,n2) = (normal(pa,pb,pc),normal(pa,pv,pc),normal(pv,pb,pc));
                let dot = |x: V3,y: V3| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
                let least = 1e-6*dot(n0,n0).sqrt();
                if !(dot(n0,n1) > least*dot(n1,n1).sqrt() && dot(n0,n2) > least*dot(n2,n2).sqrt() && dot(n1,n1).sqrt() > least && dot(n2,n2).sqrt() > least) { continue; }
                // a vertex beside the edge with a triangle of its own in
                // the edge's plane, between it and the edge: the split
                // would lay a new triangle over that one (two sources'
                // fragments of one plane, a vertex of one a whisker past
                // the other's edge, whose own sliver reaches back to it)
                if lies_over(mesh,live.on(v),v,[a,v,c],[v,b,c],n0) { continue; }
                split.push((t,a,b,v)); used.insert(t);
                claimed.extend(makes);
            }
        } }
        if split.is_empty() { break; }
        made += split.len();
        for (t,a,b,v) in split {
            let tri = mesh.triangles[t];
            let k = (0..3).find(|&k| tri[k] == a && tri[(k+1)%3] == b).unwrap();
            let c = tri[(k+2)%3];
            mesh.triangles[t] = [a,v,c];
            mesh.triangles.push([v,b,c]);
            let s = mesh.sheet[t]; mesh.sheet.push(s);
            live.replace(t,tri,[a,v,c]);
            live.push(mesh.triangles.len()-1,[v,b,c]);
        }
        live.settle();
        // two splits of one round can make one triangle twice
        // a dropped repeat renumbers the triangles after it: the index is made again
        if dedupe(mesh) > 0 { live = Live::new(mesh.vertices.len(),&mesh.triangles); }
    }
    made
}

/// Whether either of two triangles to be made at `v` overlaps, in area, a
/// triangle already on `v` that lies in their plane (normal `n`, not unit)
/// facing their way.
fn lies_over(mesh: &KeptMesh,on: &[u32],v: u32,first: [u32;3],second: [u32;3],n: V3) -> bool {
    use super::planar::{P2,area2,overlap};
    let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
    if !(len > 0.) { return false; }
    let n = n.map(|x| x/len);
    let least = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
    let mut axis = [0.;3]; axis[least] = 1.;
    let u = { let c = [n[1]*axis[2]-n[2]*axis[1],n[2]*axis[0]-n[0]*axis[2],n[0]*axis[1]-n[1]*axis[0]]; let l = (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt(); c.map(|x| x/l) };
    let w = [n[1]*u[2]-n[2]*u[1],n[2]*u[0]-n[0]*u[2],n[0]*u[1]-n[1]*u[0]];
    let origin = mesh.vertices[v as usize];
    let to2 = |p: V3| -> P2 { let r = [p[0]-origin[0],p[1]-origin[1],p[2]-origin[2]]; (None,r[0]*u[0]+r[1]*u[1]+r[2]*u[2],r[0]*w[0]+r[1]*w[1]+r[2]*w[2]) };
    let polygon = |t: [u32;3]| -> Vec<P2> { let mut p: Vec<P2> = t.iter().map(|&x| to2(mesh.vertices[x as usize])).collect(); if area2(&p) < 0. { p.reverse(); } p };
    let scale = [first,second].iter().flatten().map(|&x| distance(mesh.vertices[x as usize],origin)).fold(0_f64,f64::max).max(f64::MIN_POSITIVE);
    let eps = 1e-9*scale;
    let news = [polygon(first),polygon(second)];
    for &i in on {
        let t = mesh.triangles[i as usize];
        let [p,q,r] = t.map(|x| mesh.vertices[x as usize]);
        let Some(m) = crate::space::stable_normal(p,q,r) else { continue };
        if m[0]*n[0]+m[1]*n[1]+m[2]*n[2] < 0.999 { continue; }
        if [p,q,r].iter().any(|x| ((x[0]-origin[0])*n[0]+(x[1]-origin[1])*n[1]+(x[2]-origin[2])*n[2]).abs() > 1e-6*scale) { continue; }
        let old = polygon(t);
        if news.iter().any(|new| overlap(new,&old,eps)) { return true; }
    }
    false
}

/// Drop every sliver (no altitude above `thin`) that walks an edge the way
/// another triangle already does: a piece of one source lying along a
/// tool edge over another source's region, which the merge of their
/// vertices has laid on the same edges. Returns how many went.
pub fn drop_doubled_slivers(mesh: &mut KeptMesh,thin: f64) -> usize {
    let mut owners: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
    for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owners.entry((t[k],t[(k+1)%3])).or_default().push(i); } }
    let altitude = |t: &[u32;3]| -> f64 { let [a,b,c] = t.map(|v| mesh.vertices[v as usize]); crate::space::altitude(a,b,c) };
    let doomed: std::collections::BTreeSet<usize> = owners.values().filter(|v| v.len() > 1).flatten().copied().filter(|&i| altitude(&mesh.triangles[i]) < thin).collect();
    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    let mut sheet = Vec::with_capacity(mesh.sheet.len());
    for (i,(t,s)) in mesh.triangles.iter().zip(&mesh.sheet).enumerate() { if !doomed.contains(&i) { triangles.push(*t); sheet.push(*s); } }
    mesh.triangles = triangles; mesh.sheet = sheet;
    doomed.len()
}

/// Merge a boundary vertex into another it lies within `tol` of, nearest pair first. Measured:
/// a collinear boundary run leaves vertices on a boundary edge's line but *past* its end, so no
/// split can take them — a vertex is only ever offered to an edge whose span holds it — and being
/// past the end puts them within the tolerance of that **endpoint** instead. They are two
/// samplings of one boundary junction, and merging them is the resolution the split cannot reach.
///
/// Every merge is tried and kept only if it leaves **no directed edge walked more than once**,
/// which is the rule the whole stitch keeps and which the mesh already satisfies here. So the
/// manifold cannot regress: a merge that would break it is refused and nothing changes. That
/// guard is the whole difference from welding by proximity alone, which folded strips into edges
/// used three times. Returns how many vertices were merged away.
pub fn weld_boundary_ends(mesh: &mut KeptMesh,tol: f64) -> usize {
    let walks_once = |triangles: &[[u32;3]]| -> bool {
        let mut seen: std::collections::BTreeSet<(u32,u32)> = Default::default();
        triangles.iter().all(|t| (0..3).all(|k| seen.insert((t[k],t[(k+1)%3]))))
    };
    let edges = super::adjacency::Edges::new(&mesh.triangles);
    let verts: std::collections::BTreeSet<u32> = edges.counts().into_iter()
        .filter(|(_,(f,b))| f+b == 1).flat_map(|((a,b),_)| [a,b]).collect();
    let vs: Vec<u32> = verts.into_iter().collect();
    // the pairs within reach, nearest first and then by index, so the walk is the same every run
    let mut pairs: Vec<(f64,u32,u32)> = Vec::new();
    for i in 0..vs.len() { for j in i+1..vs.len() {
        let d = distance(mesh.vertices[vs[i] as usize],mesh.vertices[vs[j] as usize]);
        if d > 0. && d <= tol { pairs.push((d,vs[i],vs[j])); }
    } }
    pairs.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));
    let mut merged = 0;
    for (_,a,b) in pairs {
        let onto = |t: &[u32;3]| -> [u32;3] { t.map(|v| if v == b { a } else { v }) };
        // A merge can leave a triangle whose corners are distinct but now collinear. Dropping such
        // a triangle is surface lost and the volume drifts with it (measured: the tumbling
        // cylinder 6.6390 to 6.6304), and a certified boundary may not quietly shed material — so
        // the merge is refused whole instead. Leaving a loop open is an honest refusal; closing it
        // by discarding surface is not. Only a triangle this merge moved is judged: one already
        // flat elsewhere is not this pass's business.
        let flattens = (0..mesh.triangles.len()).any(|i| {
            let old = mesh.triangles[i];
            let t = onto(&old);
            if t == old || t[0] == t[1] || t[1] == t[2] || t[2] == t[0] { return false; }
            let [p,q,r] = t.map(|v| mesh.vertices[v as usize]);
            crate::space::degenerate(p,q,r)
        });
        if flattens { continue; }
        // what remains are the triangles the merge folded onto a segment: the two merged vertices
        // were corners of one triangle, which is the ordinary edge collapse
        let kept: Vec<usize> = (0..mesh.triangles.len())
            .filter(|&i| { let t = onto(&mesh.triangles[i]); t[0] != t[1] && t[1] != t[2] && t[2] != t[0] }).collect();
        // nothing to do where one of the two is already gone
        if kept.len() == mesh.triangles.len() && !mesh.triangles.iter().any(|t| t.contains(&b)) { continue; }
        let trial: Vec<[u32;3]> = kept.iter().map(|&i| onto(&mesh.triangles[i])).collect();
        if !walks_once(&trial) { continue; }
        mesh.sheet = kept.iter().map(|&i| mesh.sheet[i]).collect();
        mesh.triangles = trial;
        merged += 1;
    }
    merged
}

/// Collapse a needle standing on the boundary: a triangle owning a boundary edge whose height over
/// that edge is under `thin` has its apex all but on the edge's line. Measured, that is what stops
/// the T-junction split — a candidate farther off the line than a quarter of the owner's height is
/// refused, and on a needle 0.00625 tall over an edge 0.385 long a quarter of the height is
/// nothing. The apex is merged into the nearer end of the edge, which takes the needle away and
/// pulls its neighbours with it, losing only its own negligible area.
///
/// `thin` is the certificate's least probe distance, which is already its definition of a sliver:
/// a triangle with no altitude above it has no normal worth probing along. Every collapse is tried
/// and kept only if no directed edge is then walked more than once and no triangle it moved is left
/// flat — the guard `weld_boundary_ends` keeps — so a collapse can neither break the manifold nor
/// shed surface, and one that would is refused. Returns how many were collapsed.
pub fn collapse_needles(mesh: &mut KeptMesh,thin: f64,longest: f64) -> usize {
    let walks_once = |triangles: &[[u32;3]]| -> bool {
        let mut seen: std::collections::BTreeSet<(u32,u32)> = Default::default();
        triangles.iter().all(|t| (0..3).all(|k| seen.insert((t[k],t[(k+1)%3]))))
    };
    let mut collapsed = 0;
    for _ in 0..8 {
        let edges = super::adjacency::Edges::new(&mesh.triangles);
        // the needles over a boundary edge, each with the apex to move and the end to move it to
        let mut moves: Vec<(f64,u32,u32)> = Vec::new(); // height, apex, the end it merges into
        for ((a,b),(f,r)) in edges.counts() {
            if f+r != 1 { continue; }
            let Some(t) = edges.owner(a,b).or_else(|| edges.owner(b,a)) else { continue };
            let tri = mesh.triangles[t];
            let Some(c) = tri.iter().copied().find(|&v| v != a && v != b) else { continue };
            let (pa,pb,pc) = (mesh.vertices[a as usize],mesh.vertices[b as usize],mesh.vertices[c as usize]);
            let h = crate::space::altitude(pa,pb,pc);
            if !(h < thin) { continue; }
            // the apex goes to whichever end it already stands nearer
            let end = if distance(pc,pa) <= distance(pc,pb) { a } else { b };
            moves.push((h,c,end));
        }
        // flattest first, then by index, so the walk is the same every run
        moves.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));
        let before = collapsed;
        for (_,apex,end) in moves {
            if apex == end { continue; }
            let onto = |t: &[u32;3]| -> [u32;3] { t.map(|v| if v == apex { end } else { v }) };
            // a collapse that would flatten a triangle it moved is refused, as a merge is
            let flattens = (0..mesh.triangles.len()).any(|i| {
                let old = mesh.triangles[i];
                let t = onto(&old);
                if t == old || t[0] == t[1] || t[1] == t[2] || t[2] == t[0] { return false; }
                let [p,q,r] = t.map(|v| mesh.vertices[v as usize]);
                crate::space::degenerate(p,q,r)
            });
            if flattens { continue; }
            // Nor may a collapse **stretch** an edge past the mesh's own sampling. The apex goes
            // to the nearer end of its base, a distance bounded by that base and not by `thin`,
            // and the pass runs to a fixpoint, so the stretching compounds. Measured on the
            // tumbling cylinder: the finished mesh's longest edge was 0.9859, twice the spacing,
            // on a triangle *this* pass had rewritten — it still carried its own sheet id, where a
            // band the zip lays carries `u32::MAX` — and 0.5330 with this pass disabled, while
            // disabling the weld instead left 0.9964. A long edge is a chord that bows far from
            // the surface, which is what the certificate then has to probe across.
            let stretches = (0..mesh.triangles.len()).any(|i| {
                let old = mesh.triangles[i];
                if !old.contains(&apex) { return false; }
                let t = onto(&old);
                if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] { return false; }
                (0..3).any(|k| distance(mesh.vertices[t[k] as usize],mesh.vertices[t[(k+1)%3] as usize]) > longest)
            });
            if stretches { continue; }
            let kept: Vec<usize> = (0..mesh.triangles.len())
                .filter(|&i| { let t = onto(&mesh.triangles[i]); t[0] != t[1] && t[1] != t[2] && t[2] != t[0] }).collect();
            if kept.len() == mesh.triangles.len() { continue; }
            let trial: Vec<[u32;3]> = kept.iter().map(|&i| onto(&mesh.triangles[i])).collect();
            if !walks_once(&trial) { continue; }
            mesh.sheet = kept.iter().map(|&i| mesh.sheet[i]).collect();
            mesh.triangles = trial;
            collapsed += 1;
        }
        if collapsed == before { break; }
    }
    collapsed
}

/// Drop every triangle that repeats another's three vertices.
pub fn dedupe(mesh: &mut KeptMesh) -> usize {
    // each triangle's sorted corners with its index, sorted: a repeat follows the first of its kind
    let mut keys: Vec<([u32;3],u32)> = mesh.triangles.iter().enumerate().map(|(i,t)| { let mut k = *t; k.sort(); (k,i as u32) }).collect();
    keys.sort_unstable();
    let mut repeat = vec![false;mesh.triangles.len()];
    let mut dropped = 0;
    for w in keys.windows(2) { if w[0].0 == w[1].0 { repeat[w[1].1 as usize] = true; dropped += 1; } }
    if dropped == 0 { return 0; }
    let mut triangles = Vec::with_capacity(mesh.triangles.len()-dropped);
    let mut sheet = Vec::with_capacity(mesh.sheet.len()-dropped);
    for ((t,s),r) in mesh.triangles.iter().zip(&mesh.sheet).zip(repeat) { if !r { triangles.push(*t); sheet.push(*s); } }
    mesh.triangles = triangles; mesh.sheet = sheet;
    dropped
}
