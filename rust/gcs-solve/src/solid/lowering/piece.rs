//! **What a recognised region is built as** (#194): its faces read off the stratum (`BodyFaces`)
//! and a piece per edge between them — a section swept along a straight edge (`run_piece`), turned
//! round a circle (`ring_piece`), rolled along any other — with a ball's cell at each corner three
//! faces share (`corner_at`), each carried past its ends and kept within what it ends on.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::chain::Next;
use crate::solid::faces::EndBy;
use super::recognise::Faces;
use super::{Across, Beyond, Corner, Kept, Piece, Ring, Rolled, Run, Seam, Section, Spill, Stroke, Surf, V};
use crate::space::{add, cross, distance, dot, norm, normalised, scale, sub};

/// What a run ends on, as `BodyFaces::end_faces` reads it.
type Ends = [(EndBy, f64, Option<V>)];


/// Why an entry that recognised a region does not build it.
pub(super) fn unbuilt(why: &str) -> String {
    format!("{why}: no exact construction is made of it yet")
}

/// Why a section reaching across a seam into the next face of a chain (#422) is not built: the
/// refusals by name.
pub(super) fn spilled(why: &str) -> String {
    format!("its section reaches across the seam where a face goes on tangent into another {why}: no exact \
        construction is made of it yet, and the field reads it")
}

/// From a face toward a region (its outward normal `n`): into the material at a convex edge, the
/// void at a concave one.
pub(super) fn toward(n: V, within: bool) -> V { if within { scale(n, -1.0) } else { n } }

/// The region's faces read off its body (the stratum), and the pieces it is (none: it holds
/// nowhere).
pub(super) fn build(body: &crate::solid::faces::BodyFaces, f: &Faces, section: Section, spill: &Spill,
    units: crate::units::Units) -> Result<Vec<Piece>, String> {
    // each reference's faces: one, or a face another operand split in pieces (a plate's top a rod
    // is sunk along); one a boolean ate, or none of that name, holds nowhere
    let mut groups = Vec::new();
    for r in &f.faces {
        let fs = body.faces(r);
        if fs.is_empty() {
            return Ok(Vec::new());
        }
        groups.push(fs);
    }
    let mut pieces = Vec::new();
    for &[i, j] in f.pairs() {
        let ps = run(body, &groups[i], &groups[j], f.within, section, spill, units)?;
        if f.faces.len() == 2 {
            stands_within(body, f, [&groups[i], &groups[j]], section, &ps, spill, units)?;
        }
        if ps.is_empty() {
            return Ok(Vec::new());
        }
        pieces.extend(ps);
    }
    let (Section::Round(r), 3) = (section, groups.len()) else { return Ok(pieces) };
    // three faces: the ball touching all three, at each vertex a face of each shares (three planes
    // meet at one; a D's flat, its round and a cap at both ends of the flat's edge), each of them
    // the face of its reference standing there — one piece of a face split in pieces (a plate's
    // side a post stands across)
    let refs: Vec<&[usize]> = groups.iter().map(Vec::as_slice).collect();
    let mut corners = Vec::new();
    for (vertex, at) in body.vertices_among(&refs) {
        let one = |g: &Vec<usize>| match g[..] { [fi] => Some(fi), _ => None };
        let Some(ids) = at.iter().map(one).collect::<Option<Vec<usize>>>() else {
            return Err(unbuilt("a corner where two pieces of one face meet"));
        };
        let (corner, toward) = corner_at(body, &ids, vertex, f.within, r)?;
        corners.push((corner, toward, ids));
    }
    if corners.is_empty() {
        return Ok(Vec::new());
    }
    // three planes: every run within each edge's ends, so the region is their union and nothing
    // cuts them
    let planar = |c: &Corner| c.faces.iter().all(|s| matches!(s, Surf::Plane { .. }));
    if let [(corner, toward, ids)] = &corners[..] {
        if !planar(corner) {
            pieces.extend(corners.into_iter().map(|(c, ..)| Piece::Corner(c)));
            return Ok(pieces);
        }
        let mut bounds = Vec::new();
        for &[i, j] in f.pairs() {
            let es = body.edges_between(&[ids[i]], &[ids[j]]);
            bounds.extend(ends_at_all(body, &body.end_faces(&es, &[ids[i], ids[j]])));
        }
        let runs: Vec<&Run> = pieces.iter().filter_map(|p| match p { Piece::Run(r) => Some(r), _ => None }).collect();
        let size = runs.iter().map(|r| r.span[1] - r.span[0]).fold(r, f64::max);
        // (a run cut off at a slant by an end is within that end already)
        let within = |p: V, own: &[(V, f64)]| bounds.iter().filter(|b| !own.contains(b))
            .all(|&(n, d)| dot(n, p) <= d + 1e-9 * size);
        let (feet, touches) = crate::brep::fillet::corner_points(corner.vertex, corner.centre, r, *toward);
        let cell = [corner.vertex, corner.centre].into_iter().chain(feet).chain(touches);
        let runs_within = runs.iter().all(|r| r.hull().into_iter().all(|p| within(p, &r.within)));
        if !runs_within || !cell.into_iter().all(|p| within(p, &[])) {
            return Err(unbuilt("a ball larger than the edges it rounds"));
        }
    }
    pieces.extend(corners.into_iter().map(|(c, ..)| Piece::Corner(c)));
    Ok(pieces)
}

/// **A ball that does not stand within the faces it rolls against** (#962): a round's ball
/// centres are the points within both faces' smooth surfaces (or outside both) at least its radius
/// off each, and a run or a ring is the set only where its ball, standing against its two faces,
/// is one of them all along the edge.  Where a face goes on smooth into others bulging toward the
/// ball (`BodyFaces::bulging`: a pocket's walls, a boss's sides) its ball may come nearer one of
/// those than its radius — at the end of a wall rounded alone, whose ball stops at the next wall
/// short of the edge's end; between walls closer together than the ball is wide — and there what
/// no ball reaches is more than the run's section swept: refused, saying which face the ball comes
/// how near.  A flat the same: each point of its section nearer its own face than any other of
/// the chain's (where its section was built against the chain beside the edge, #422, the chain's
/// nearest piece is its own, and that is the chain's to say).  Checked at the ends of a straight edge (the centres within a convex chain are a
/// convex set, so the ends hold the whole run) and round a ring at sixteen turns of its own.  A
/// chain no side of which is convex toward the ball is read through its nearest face by the
/// field, and left to the gate.
fn stands_within(body: &crate::solid::faces::BodyFaces, f: &Faces, groups: [&[usize]; 2], section: Section,
    pieces: &[Piece], spill: &Spill, units: crate::units::Units) -> Result<(), String> {
    // a flat built against the chain beside its edge (#422) measures each point from the chain's
    // nearest piece already, the next face's where it has crossed the seam
    if matches!(section, Section::Flat { .. }) && groups.iter().any(|g| g.iter().any(|fi| spill.contains_key(fi))) {
        return Ok(());
    }
    // toward the side the region is on: into the material within, the void outside
    let sigma = if f.within { -1.0 } else { 1.0 };
    // each smooth face's chain, the others of it, and whether the field reads it face by face (a
    // chain bulging toward the ball, `BodyFaces::bulging`) or through its nearest face (any other:
    // a drafted shell's outline turning both ways)
    let chains: Vec<(usize, Vec<usize>, bool)> = (0..2).filter(|&k| f.faces[k].smooth).map(|k| {
        let chain = body.smooth(groups[k]);
        let carried = body.bulging(&chain, true, !f.within).is_some_and(|(_, carried)| carried);
        (k, chain.into_iter().filter(|g| !groups[k].contains(g)).collect(), carried)
    }).collect();
    if chains.iter().all(|(_, others, _)| others.is_empty()) {
        return Ok(());
    }
    // the points each section is read at: its ball's centre, or its flat's own points; each where
    // the piece stands along its edge
    let points = |run: &Run| -> Vec<V> {
        match section {
            Section::Round(_) => run.centre().into_iter().collect(),
            Section::Flat { .. } => run.strokes.iter().flat_map(|(s, _)| match *s {
                Stroke::Line { from, to } => vec![from, to],
                Stroke::Conic { a, b, .. } => vec![a, b],
                Stroke::Arc { .. } => Vec::new(),
            }).collect(),
        }
    };
    let mut at: Vec<V> = Vec::new();
    for p in pieces {
        match p {
            Piece::Run(run) => for q in points(run) {
                // its middle first, then its ends, then between: where walls closer than the ball
                // pass it, the middle sees the wall across, not the arc it ends on; and the centres
                // within a chain turning both ways are no convex set
                at.extend([0.5, 0.0, 1.0, 0.125, 0.25, 0.375, 0.625, 0.75, 0.875]
                    .map(|s| add(q, scale(run.along, s * run.edge))));
            },
            Piece::Ring(ring) => for q in points(&ring.section) {
                at.extend((0..=16).map(|k| crate::brep::geom::Rigid::turn(ring.axis_origin, ring.axis,
                    ring.angle * k as f64 / 16.0).point(q)));
            },
            _ => {}
        }
    }
    let size = section.size();
    let toward = |g: usize, q: V| sigma * body.carried_side_grad(g, q).0;
    for &q in &at {
        let tol = 1e-9 * (size + norm(q));
        for (k, others, carried) in &chains {
            // how far the ball must stand off the others: its radius, or a flat's point as far as
            // from its own face
            let need = match section {
                Section::Round(r) => r,
                Section::Flat { .. } => groups[*k].iter().map(|&g| body.patch_distance(g, q)).fold(f64::INFINITY, f64::min),
            };
            // read as the field reads the centres: off each face's carried side where the chain
            // bulges toward the ball — of the faces it comes too near, the one whose patch is
            // nearest (a wall, not the arc carried on along the wall's plane past its seam) — else
            // off the side of the face whose patch is nearest, where that is another's
            let by_patch = |a: &(usize, f64), b: &(usize, f64)| body.patch_distance(a.0, q).total_cmp(&body.patch_distance(b.0, q));
            let found = if let Section::Flat { .. } = section {
                // a flat's sum reads each face's patch (`distance(p, b.smooth)`): its point's
                // nearest piece of the chain must be its own
                others.iter().map(|&g| (g, body.patch_distance(g, q))).filter(|&(_, d)| d < need - tol)
                    .min_by(|a, b| a.1.total_cmp(&b.1))
            } else if *carried {
                others.iter().map(|&g| (g, toward(g, q))).filter(|&(_, d)| d < need - tol).min_by(by_patch)
            } else {
                let nearest = groups[*k].iter().chain(others).map(|&g| (g, 0.0)).min_by(by_patch);
                nearest.filter(|&(g, _)| !groups[*k].contains(&g)).map(|(g, _)| (g, sigma * body.side(g, q)))
                    .filter(|&(_, d)| d < need - tol)
            };
            let Some((g, d)) = found else { continue };
            let said = |fi: usize| body.paths().get(fi).map(|p| p.text()).filter(|t| !t.is_empty())
                .map_or_else(|| "a face".to_string(), |t| format!("`{t}`"));
            let own = said(groups[*k][0]);
            // the arc of the chain tighter than the ball between, where it meets the other face: the
            // corner's radius where the round runs (a drafted one's there), the ball's against it
            let tight = others.iter().filter(|&&h| body.plane(h).is_none()).flat_map(|&h| {
                body.edges_between(&[h], groups[1 - *k]).into_iter()
                    .filter_map(move |e| body.circle(e).map(|(_, _, radius, _)| (h, radius)))
            }).filter(|&(_, radius)| radius < need - tol)
                .min_by(|a, b| body.patch_distance(a.0, q).total_cmp(&body.patch_distance(b.0, q)));
            // to the fewest figures, three at least, that tell the ball's radius from how near it
            // comes and from the arc's
            let apart = |a: f64, b: f64| (3..16).find(|&n| figured(a, n) != figured(b, n)).unwrap_or(16);
            let n = apart(need, d).max(tight.map_or(3, |(_, radius)| apart(need, radius)));
            let length = |x: f64| crate::units::Dim::LENGTH.number_text(figured(x, n), units);
            let near = if d >= 0.0 { format!("within {} of", length(d)) } else { format!("{} past", length(-d)) };
            let past = tight.map_or_else(String::new, |(h, radius)| {
                format!(" past {}, of radius {} where the {} runs and tighter than the {}", said(h), length(radius),
                    if matches!(section, Section::Round(_)) { "round" } else { "chamfer" },
                    if matches!(section, Section::Round(_)) { "ball" } else { "flat's leg" })
            });
            return Err(match section {
                Section::Round(r) => format!("its ball, of radius {}, rolled against {own} comes {near} {}, which \
                    {own} goes on into smoothly{past}: no ball of that radius stands there within both faces, and \
                    what none reaches is more than the round of the edge, so none is made of it", length(r), said(g)),
                Section::Flat { .. } => format!("its flat, reaching {} from {own}, comes {near} {}, which {own} \
                    goes on into smoothly{past}, nearer than to {own} itself, so no chamfer of the edge is made of it",
                    length(need), said(g)),
            });
        }
    }
    Ok(())
}

/// The corner at `vertex` where the faces `ids` meet — planes, or a cylinder or a sphere among
/// them (#218's 4) — and the way from each face toward its ball. The ball stands `r` from each
/// surface on the region's side (`within`, the material's): from where it would stand were each
/// face its tangent plane at the vertex (a 3×3 solve: there for three planes), by Newton's steps
/// on the three signed distances. The cell is the ball's normal cone cut by the faces, so the
/// vertex stands in that cone, back from the centre along each face's normal: `c − v = Σ μᵢ tᵢ`,
/// every `μᵢ > 0` (a corner whose shrunk body loses the vertex's face there — a sliver of a face
/// between two nearly alike — is no ball's cell).
fn corner_at(body: &crate::solid::faces::BodyFaces, ids: &[usize], vertex: V, within: bool, r: f64)
    -> Result<(Corner, [V; 3]), String> {
    let faces: Vec<Surf> = ids.iter().map(|&fi| Surf::of(body, fi)
        .ok_or_else(|| "a corner where a face neither a plane, a cylinder, a cone nor a sphere meets two others".to_string()))
        .collect::<Result<_, _>>().map_err(|e| unbuilt(&e))?;
    let faces = [faces[0], faces[1], faces[2]];
    let tangents: [V; 3] = std::array::from_fn(|i| toward(body.normal(ids[i], vertex), within));
    if dot(tangents[0], cross(tangents[1], tangents[2])).abs() <= 1e-9 {
        return Err(unbuilt("a corner whose faces share a direction"));
    }
    // the signed distance from each surface (positive away from the material) the centre stands at
    let want = if within { -r } else { r };
    let size = r + norm(vertex);
    let start = add(vertex, crate::brep::ssi::solve3(tangents, [r; 3]));
    let centre = crate::roots::newton_onto::<3>(start, r,
        |c| Some(std::array::from_fn(|i| { let (s, g) = body.side_grad(ids[i], c); (g, s - want) })),
        |at| at.iter().all(|(_, m)| m.abs() <= 1e-13 * size))
        .ok_or_else(|| unbuilt("a corner where no ball touches all three faces"))?;
    let toward: [V; 3] = std::array::from_fn(|i| toward(body.side_grad(ids[i], centre).1, within));
    let columns = std::array::from_fn(|i| [toward[0][i], toward[1][i], toward[2][i]]);
    if crate::brep::ssi::solve3(columns, sub(centre, vertex)).iter().any(|&m| !(m > 1e-9)) {
        return Err(unbuilt("a corner whose ball stands beyond one of its faces' edges"));
    }
    Ok((Corner { vertex, centre, r, faces }, toward))
}

/// The run along the edges where the faces `ga` (a reference's) meet the faces `gb`: a piece per
/// edge, each where the edge turns the side the region asks (`within`, convex) — none where they
/// meet in none — and all within where the run of them ends, as the field reads it.
fn run(body: &crate::solid::faces::BodyFaces, ga: &[usize], gb: &[usize], within: bool, section: Section, spill: &Spill,
    units: crate::units::Units) -> Result<Vec<Piece>, String> {
    let edges = body.edges_between(ga, gb);
    // an edge's two faces, `ga`'s first; none where it is not between two
    let ordered = |e: u32| match *body.faces_of(e) {
        [f0, f1] => Some(if ga.contains(&f0) { (f0, f1) } else { (f1, f0) }),
        _ => None,
    };
    // an edge no line or circle about the faces' common axis carries: the ball rolled along the
    // whole run of them, traced
    // (a straight edge carried only between planes and cylinders along it: a cone's generator is
    // rolled along, its section square to it no circle: `BodyFaces::carried`)
    if edges.iter().any(|&e| !body.carried(e)) {
        let spilt = [ga, gb].map(|g| g.iter().find_map(|f| spill.get(f).map(|seams| (*f, seams))));
        let traced = body.runs(&edges).into_iter().filter(|run| body.convex(run[0]) == Some(within));
        // a flat whose face goes on across a seam (#422): cut along the run short of the seam's
        // normal surface, the piece beyond it taking the rest
        if let Some((c, (p, seams))) = chained(spilt.map(Ok))? {
            return traced.map(|run| traced_beyond(body, &run, [ga, gb], c, p, seams, within, section)).collect();
        }
        // a piece for each run of them (two generators of a half cone: two runs): a ball rolled
        // along it, or a flat cut along it — its legs told apart by a point beside face `a`'s
        return traced.map(|run| {
            let e = run[0];
            let at = body.middle(e);
            let ends: Vec<V> = body.run_ends(&run).into_iter().map(|(v, ..)| v).collect();
            Ok(Piece::Rolled(match section {
                Section::Round(r) => Rolled { at, r, flat: None, ends, cuts: Vec::new(), kept: Vec::new(), wedge: Vec::new(),
                    beyond: None },
                Section::Flat { la, lb } => {
                    let Some((fa, fb)) = ordered(e) else {
                        return Err(unbuilt("an edge not between two faces"));
                    };
                    let na = body.normal(fa, at);
                    let t = toward(body.normal(fb, at), within);
                    let w = normalised(sub(t, scale(na, dot(t, na)))).ok_or("faces meeting tangent")?;
                    Rolled { at, r: 0.0, flat: Some(([la, lb], add(at, scale(w, 0.5 * la)))), ends, cuts: Vec::new(),
                        kept: Vec::new(), wedge: Vec::new(), beyond: None }
                }
            }))
        }).collect();
    }
    let all: Vec<usize> = ga.iter().chain(gb).copied().collect();
    let ends = body.end_faces(&edges, &all);
    let mut built = Vec::new();
    for &edge in &edges {
        let Some((fa, fb)) = ordered(edge) else {
            return Err(unbuilt("an edge not between two faces"));
        };
        built.push(piece(body, edge, fa, fb, within, section, &ends, spill, units));
    }
    // pieces of one line a curved face cuts across, which some of the section's lines pass by: one
    // run over them, kept outside it
    for (group, span, inner) in across_cuts(body, &edges, &built, &ends) {
        let (fa, fb) = ordered(group[0]).expect("a straight edge between two faces");
        let (fa, fb, section) = in_body_order(fa, fb, section);
        let whole = run_piece(body, span, fa, fb, within, section, &ends, &inner, spill);
        for (k, &e) in edges.iter().enumerate() {
            if group.contains(&e) {
                built[k] = Ok(Vec::new());
            }
        }
        let k = edges.iter().position(|&e| e == group[0]).expect("a group's edge");
        built[k] = whole.map(|p| vec![p]);
    }
    let mut pieces = Vec::new();
    for b in built {
        pieces.extend(b?);
    }
    Ok(pieces)
}

/// Why a straight run is not cut off by a curved face it ends on: some line along it through its
/// section passes the face by (a bore across the crease, a pin through it), so the run would go on
/// past that end — into the run beyond it, where the face cuts the edge in two.
const PASSED_BY: &str = "a run its curved end does not cut off";

/// Why a piece is not built, as `run` reads it: a run its curved end does not cut off
/// (`PASSED_BY`), which `across_cuts` joins with the run beyond that end, or any other refusal,
/// as it is printed.  Printed, the first is `unbuilt(PASSED_BY)`.
enum Refused {
    PassedBy,
    Why(String),
}

impl From<String> for Refused {
    fn from(why: String) -> Refused { Refused::Why(why) }
}

impl From<Refused> for String {
    fn from(r: Refused) -> String {
        match r {
            Refused::PassedBy => unbuilt(PASSED_BY),
            Refused::Why(why) => why,
        }
    }
}

/// Faces `fa` and `fb` in the body's order, the section turned with them, so two regions over one
/// edge build one run.
fn in_body_order(fa: usize, fb: usize, section: Section) -> (usize, usize, Section) {
    if fb < fa { (fb, fa, section.swapped()) } else { (fa, fb, section) }
}

/// **Runs of one line a curved face cuts across** (a bore through the crease that only some of
/// the section's lines meet, so the section passes by it): where a straight edge's piece is not cut
/// off by the curved face it ends on (`PASSED_BY`), and the edge beyond that face along the same
/// line, between the same faces, ends on it alone too, both are one run over the two and what
/// lies between, kept outside the face — what the field reads, a point read against the run
/// nearest it and within both runs' ends outside the face (`conditions::ends`).  Each group of
/// `edges` (several where faces cut a line in more than two), its span along the line and the faces
/// it is kept outside of; none where every piece was built (`built`, `piece`'s for each edge).
fn across_cuts(body: &crate::solid::faces::BodyFaces, edges: &[u32],
    built: &[Result<Vec<Piece>, Refused>], ends: &Ends) -> Vec<(Vec<u32>, [V; 2], Vec<Kept>)> {
    let passed = |k: usize| matches!(&built[k], Err(Refused::PassedBy));
    if !(0..edges.len()).any(passed) {
        return Vec::new();
    }
    // the surface a straight edge's end at `p` stands on alone (one face, or the pieces of one a
    // seam splits), a curved one kept outside
    let cut_at = |p: V| match ends_at(body, ends, p) {
        Ok((planes, bent)) if planes.is_empty() => match &bent[..] {
            [k, rest @ ..] if !k.inside && rest.iter().all(|x| x.inside == k.inside && x.face.same(&k.face)) =>
                Some(k.clone()),
            _ => None,
        },
        _ => None,
    };
    let same = |x: &Kept, y: &Kept| x.inside == y.inside && x.face.same(&y.face);
    let mut out: Vec<(Vec<u32>, [V; 2], Vec<Kept>)> = Vec::new();
    let mut taken = vec![false; edges.len()];
    for k in (0..edges.len()).filter(|&k| passed(k)) {
        if taken[k] {
            continue;
        }
        let Some([a0, a1]) = body.segment(edges[k]) else { continue };
        let Some(along) = normalised(sub(a1, a0)) else { continue };
        let faces = sorted(body.faces_of(edges[k]));
        // the straight edges between the same faces along the same line, each as its span along it
        let on_line = |p: V| norm(cross(sub(p, a0), along)) <= 1e-9 * (1.0 + norm(p) + norm(a0));
        let mut line: Vec<(usize, [f64; 2])> = (0..edges.len()).filter(|&j| !taken[j]).filter_map(|j| {
            let [b0, b1] = body.segment(edges[j])?;
            (sorted(body.faces_of(edges[j])) == faces && on_line(b0) && on_line(b1)).then(|| {
                let (s0, s1) = (dot(sub(b0, a0), along), dot(sub(b1, a0), along));
                (j, [s0.min(s1), s0.max(s1)])
            })
        }).collect();
        line.sort_by(|x, y| x.1[0].total_cmp(&y.1[0]));
        let at = |s: f64| add(a0, scale(along, s));
        // each two edges next along the line, joined where the face between them is one and the
        // same curved face kept outside, and one of them is passed by it
        let mut group: Vec<(usize, [f64; 2])> = Vec::new();
        let mut inner: Vec<Kept> = Vec::new();
        let mut flush = |group: &mut Vec<(usize, [f64; 2])>, inner: &mut Vec<Kept>, out: &mut Vec<_>| {
            if group.len() > 1 && group.iter().any(|&(j, _)| passed(j)) {
                for &(j, _) in group.iter() {
                    taken[j] = true;
                }
                let span = [at(group[0].1[0]), at(group[group.len() - 1].1[1])];
                out.push((group.iter().map(|&(j, _)| edges[j]).collect(), span, std::mem::take(inner)));
            }
            group.clear();
            inner.clear();
        };
        for &(j, s) in &line {
            let joined = group.last().and_then(|&(_, prev): &(usize, [f64; 2])| {
                let (x, y) = (cut_at(at(prev[1]))?, cut_at(at(s[0]))?);
                (same(&x, &y) && s[0] > prev[1]).then_some(x)
            });
            match joined {
                Some(cut) => {
                    if !inner.iter().any(|k| same(k, &cut)) {
                        inner.push(cut);
                    }
                }
                None => flush(&mut group, &mut inner, &mut out),
            }
            group.push((j, s));
        }
        flush(&mut group, &mut inner, &mut out);
    }
    out
}

/// A pair of face ids in order.
fn sorted(faces: &[usize]) -> Vec<usize> {
    let mut f = faces.to_vec();
    f.sort_unstable();
    f
}

/// The piece along `edge`, where faces `fa` and `fb` meet — a straight one's section swept along
/// it (`run_piece`), a circle's turned about its axis (`ring_piece`) — kept within the half-spaces
/// of the run's `ends` standing at its own ends and reaching into it; `None` where it turns the
/// other way from the side the region asks (`within`, convex).
#[allow(clippy::too_many_arguments)]
fn piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool, section: Section,
    ends: &Ends, spill: &Spill, units: crate::units::Units) -> Result<Vec<Piece>, Refused> {
    // the faces in the body's order, so two regions over one edge build one run
    let (fa, fb, section) = in_body_order(fa, fb, section);
    // the gate: an edge turning the other way, or none (faces meeting tangent), holds nowhere
    if body.convex(edge) != Some(within) {
        return Ok(Vec::new());
    }
    // (`run` rolls a circle whose faces are not all about its axis)
    if body.circle(edge).is_some() {
        return Ok(ring_piece(body, edge, fa, fb, within, section, ends, spill, units)?);
    }
    let Some(span) = body.segment(edge) else { return Err(unbuilt("a curved edge").into()) };
    run_piece(body, span, fa, fb, within, section, ends, &[], spill).map(|p| vec![p])
}

/// Round a circle `edge` where a plane, a cylinder, a cone or a sphere meet about one axis: the
/// section in the meridian through its start, turned through the circle's angle — a whole turn,
/// or a stretch ending where a face goes on from one of these tangent (an obround's flat side),
/// whose own run goes on from the same section.
#[allow(clippy::too_many_arguments)]
fn ring_piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool,
    section: Section, ends: &Ends, spill: &Spill, units: crate::units::Units) -> Result<Vec<Piece>, String> {
    let (centre, axis, radius, x) = body.circle(edge).expect("a circle's edge");
    let closed = body.closed(edge);
    let [t0, t1] = if closed { [0.0, std::f64::consts::TAU] } else { body.span(edge) };
    let radial_at = |t: f64| crate::brep::geom::Rigid::turn([0.0; 3], axis, t).vector(x);
    let [p0, p1] = [t0, t1].map(|t| add(centre, scale(radial_at(t), radius)));
    // a stretch ends at seams, or at a plane through the axis standing where its turn starts
    // or stops (a groove's rim at the plate's top): that end's own meridian, the sector's
    let meridian = |&(n, d): &(V, f64)| dot(n, axis).abs() <= 1e-9
        && (dot(n, centre) - d).abs() <= 1e-9 * (1.0 + norm(centre))
        && dot(n, body.middle(edge)) < d;
    // in the meridian a plane, a cylinder and a cone are lines, a sphere a circle about its
    // centre (on the axis, so in every meridian), a torus its tube's circle about where the
    // tube's centre crosses the meridian
    let trace = |fi: usize, p: V| match (body.sphere(fi), body.torus(fi)) {
        (Some((centre, radius)), _) => Trace::Circle { centre, radius, normal: body.normal(fi, p) },
        (_, Some((o, d, big, tube))) => {
            let w = sub(p, o);
            let radial = normalised(sub(w, scale(d, dot(w, d)))).unwrap_or(x);
            Trace::Circle { centre: add(o, scale(radial, big)), radius: tube, normal: body.normal(fi, p) }
        }
        _ => Trace::Line(body.normal(fi, p)),
    };
    // a face going on across a seam its section reaches (#422): a circle about the same axis, the
    // face beyond turned about it too, the section against the chain in every meridian
    let chain = chained([fa, fb].map(|fi| spill.get(&fi).map(|seams| seam_round(body, seams, centre, axis)).transpose()))?;
    // one not about the ring's axis: the ring kept short of the seam's normal surface, the piece
    // beyond it taking the rest (#422)
    let (chain, askew) = match chain {
        Some((c, Round::Askew)) => (None, Some(c)),
        Some((c, Round::About(o, r, g))) => (Some((c, ((o, r), g))), None),
        None => (None, None),
    };
    // the section against the chain at the turn `t`, its seams each a circle (centre, radius) and
    // the face going on across it (none: the corner of the edge's faces alone); and its footprint
    // on the chain
    let against = |t: f64, c: usize, seams: &[((V, f64), usize)]| {
        let p = add(centre, scale(radial_at(t), radius));
        let next: Vec<Next> = seams.iter().map(|&((o, r), g)| {
            let s = add(o, scale(radial_at(t), r));
            Next { seam: s, trace: trace(g, s) }
        }).collect();
        super::chain::cut_chain(p, cross(axis, radial_at(t)), [trace(fa, p), trace(fb, p)], (c, &next), within, section)
    };
    // the chain from the edge's face on, each further seam about the axis
    let (c, seams) = match chain {
        None => (0, Vec::new()),
        Some((c, first)) => (c, chain_from(body, [fa, fb][c], first, |seams| against(t0, c, seams),
            |across| match seam_round(body, across, centre, axis)? {
                Round::About(o, r, h) => Ok(((o, r), h)),
                Round::Askew => Err(spilled("round a circle beside a chain whose further seam is not about its axis")),
            })?.0),
    };
    let section_at = |t: f64| against(t, c, &seams).map(|(run, _)| run);
    // a ball wider than the circle it rolls round — or a flat's leg longer than its radius — where
    // the ball's centre would stand toward the axis: past it, its section crosses the axis
    wider(body, [fa, fb], section, section_at(t0)?, (centre, axis, radius, radial_at(t0)), closed, units)?;
    // at an end standing on a plane not through the axis (a D's round rim ending at its flat):
    // carried on round past it until the whole section has crossed that plane, and kept within
    // it (#218's 2) — each point of the section's hull turned to its first crossing (`n·x`
    // along a turn is a sinusoid: its roots in closed form); at a cylinder or a ball it runs
    // into, a quarter of the way into what every point spends in that solid (a point turning
    // round a circle may come back out of it, and the other end may run into the same solid
    // from its other side)
    let (mut from, mut to, mut halves, mut kepts) = (t0, t1, Vec::new(), Vec::new());
    if !closed {
        for (k, p, t) in [(0, p0, t0), (1, p1, t1)] {
            let (planes, bent) = ends_at(body, ends, p)?;
            let planes: Vec<(V, f64)> = planes.into_iter().filter(|h| !meridian(h)).collect();
            if planes.is_empty() && bent.is_empty() {
                continue;
            }
            let way = if k == 0 { -1.0 } else { 1.0 };
            let hull = section_at(t)?.section_hull();
            let turned = |q: V, phi: f64| crate::brep::geom::Rigid::turn(centre, axis, way * phi).point(q);
            let mut reach: f64 = 0.0;
            for &(n, d) in &planes {
                for &q in &hull {
                    let w = sub(q, centre);
                    let rim = sub(w, scale(axis, dot(w, axis)));
                    let (a, b) = (dot(n, rim), way * dot(n, cross(axis, rim)));
                    let c = dot(n, sub(q, rim)) - d;
                    reach = reach.max(first_over(a, b, c)
                        .ok_or_else(|| unbuilt("a stretch of a circle its end does not cut off"))?);
                }
            }
            for kept in &bent {
                let (mut entry, mut exit) = (0.0_f64, std::f64::consts::TAU);
                for &q in &hull {
                    let [first, back] = dwell(|phi| kept.away(turned(q, phi)))
                        .ok_or_else(|| unbuilt("a stretch of a circle its curved end does not cut off"))?;
                    entry = entry.max(first);
                    exit = exit.min(back);
                }
                if !(entry < exit) {
                    return Err(unbuilt("a stretch of a circle its curved end does not cut off"));
                }
                reach = reach.max(entry + 0.25 * (exit - entry));
            }
            if k == 0 { from -= reach } else { to += reach }
            halves.extend(planes);
            kepts.extend(bent);
            // and no further than the edge the face it ends on makes with its own goes (a curved
            // face only where it reaches the ring: every point of the ring as far from its centre
            // as the section's are)
            let (planes, bent) = ends_beyond(body, ends, p)?;
            let reach = hull.iter().map(|&q| distance(q, centre)).fold(0.0, f64::max);
            halves.extend(planes.into_iter().filter(|&(n, d)| dot(n, centre) - d >= -reach));
            kepts.extend(bent.into_iter().filter(|kept| kept.away(centre) >= -reach));
        }
        if to - from >= std::f64::consts::TAU {
            return Err(unbuilt("a stretch of a circle carried on round its whole turn"));
        }
    }
    let mut start = section_at(from)?;
    start.within = halves;
    start.kept = kepts;
    let mut ring = Ring { section: start, axis_origin: centre, axis, angle: to - from, beyond: None };
    if let Some(c) = askew {
        let seams = spill.get(&[fa, fb][c]).expect("the face going on");
        ring.beyond = Some(beyond(body, edge, [fa, fb], c, seams, within, section, &mut ring)?);
    }
    Ok(vec![Piece::Ring(ring)])
}

/// `x` to `n` significant figures.
fn figured(x: f64, n: i32) -> f64 {
    if x == 0.0 { return 0.0 }
    let k = 10f64.powi(n - 1 - x.abs().log10().floor() as i32);
    (x * k).round() / k
}

/// **A ball wider than the circle it rolls round** (#962): where the section of a ring stretches
/// past the circle's axis — a ball's centre standing toward the axis by more than the circle's
/// radius, a pocket's corner rounded wider than it, a boss's rim or a drafted shell's where its
/// corner has shrunk under the ball; a flat's leg along the face square to the axis past it — no
/// turn of it is the set.  The ball touches the faces beside the corner and never the corner's
/// own: what `std.rounds`' set holds there is not a ring about the corner's axis, and it does not
/// end with the edge — it runs on along the corner's face away from the edge, its centres every
/// ball's within both faces, so it rounds the corner itself to the ball's radius as far as that
/// face goes (up a pocket's corner, out of the body and on without end; down a boss's, into what it
/// stands on).  A face alone — a whole cylinder, a cone, an arc between creases — has no faces
/// either side: no ball of that radius stands on the round's side of it at all, its centres are
/// none, and the set is all of that side along its whole length.  Refused, saying the corner's
/// radius where the round runs against the ball's (or
/// the flat's leg), in the document's unit.  At the radius itself the section's arc is centred on
/// the axis and turns into a sphere (`dome.sv`, a pocket's corner rounded to its own radius); a
/// hair short, a spindle torus the same set reads, and both are built.
fn wider(body: &crate::solid::faces::BodyFaces, faces: [usize; 2], section: Section, start: Run,
    (centre, axis, radius, radial): (V, V, f64, V), closed: bool, units: crate::units::Units) -> Result<(), String> {
    let tol = 1e-9 * (radius + section.size());
    // the section's own points where it turns: each stroke's ends (an arc's or a conic's never
    // reach past the line through them toward the axis, a round's quarter turn and a flat's bow
    // lying between their ends' radii)
    let ends = start.strokes.iter().flat_map(|(s, _)| match *s {
        Stroke::Line { from, to } => vec![from, to],
        Stroke::Arc { centre, radius, x, angles: [a0, a1] } => {
            let frame = crate::brep::geom::Frame::new(centre, start.along, x);
            [a0, a1].map(|a| { let (s, c) = a.dsin_cos(); frame.at([radius * c, radius * s, 0.0]) }).to_vec()
        }
        Stroke::Conic { a, b, .. } => vec![a, b],
    });
    let past = ends.map(|q| {
        let w = sub(q, centre);
        dot(sub(w, scale(axis, dot(w, axis))), radial)
    }).fold(f64::INFINITY, f64::min);
    if past >= -tol {
        return Ok(());
    }
    // to the fewest figures, three at least, that tell the two apart
    let size = match section { Section::Round(r) => r, Section::Flat { la, lb } => la.max(lb) };
    let figures = (3..16).find(|&n| figured(size, n) != figured(radius, n)).unwrap_or(16);
    let length = |x: f64| crate::units::Dim::LENGTH.number_text(figured(x, figures), units);
    // the face the circle's radius is of: a cylinder's or a cone's about the axis, else the other
    let curved = faces.into_iter().find(|&fi| body.plane(fi).is_none()).unwrap_or(faces[0]);
    let other = if curved == faces[0] { faces[1] } else { faces[0] };
    let said = |fi: usize, none: &str| body.paths().get(fi).map(|p| p.text()).filter(|t| !t.is_empty())
        .map_or_else(|| none.to_string(), |t| format!("`{t}`"));
    let (face, from) = (said(curved, "the corner's face"), said(other, "the other face"));
    let what = if closed { "circle" } else { "arc" };
    Err(match section {
        // the curved face goes on smooth into others (a pocket's walls): the ball stands against
        // those, and the set rounds the corner itself along the face away from the edge
        Section::Round(_) if body.smooth(&[curved]).len() > 1 => format!("its ball, of radius {}, is wider \
            than the {what} it rolls round, of radius {} where the round runs: it touches the faces either side \
            of {face} and never {face} itself, and what no ball reaches runs on along {face} away from the edge, \
            rounding it to the ball's radius, so no round of the edge is made of it", length(size), length(radius)),
        // alone (a whole cylinder, a cone, an arc between creases): no ball of that radius stands on
        // the round's side of it at all — the centres are none, and the set all of that side
        Section::Round(_) => format!("its ball, of radius {}, is wider than the {what} it rolls round, of radius \
            {} where the round runs: no ball of that radius stands on the round's side of {face} at all, so \
            what none reaches is everything on that side of it from {from} on, along its whole length and past \
            the body without end, and no round of the edge is made of it", length(size), length(radius)),
        Section::Flat { .. } => format!("its flat's leg, {}, is longer than the radius of the {what} it \
            runs round, {} where the chamfer runs: the flat crosses the axis of {face}, so no chamfer of the \
            edge is made of it", length(size), length(radius)),
    })
}

/// The ring round circle `edge` (between `faces`, face `c` a plane going on tangent across the
/// circle `seams` into a torus about its normal but not about the ring's axis) kept short of the
/// seam's normal surface — the cylinder through the seam along the plane's normal — and the piece
/// beyond it (#422): a chamfer's flat against the torus there; or a round's ring a stretch between
/// where its ball touches the seam, and the ball rolled against the torus between.  Refused by name
/// at a stretch of a circle, and where the edge or the wall's strip crosses that surface or the leg
/// crosses no seam; the chain's own refusals are `chain_seam`'s.
#[allow(clippy::too_many_arguments)]
fn beyond(body: &crate::solid::faces::BodyFaces, edge: u32, faces: [usize; 2], c: usize, seams: &[(u32, usize)],
    within: bool, section: Section, ring: &mut Ring) -> Result<Beyond, String> {
    if !body.closed(edge) {
        return Err(spilled("round a stretch of a circle off the seam's axis"));
    }
    let (centre, axis, radius) = (ring.axis_origin, ring.axis, body.circle(edge).expect("a circle's edge").2);
    let chained = chain_seam(body, faces[c], seams, centre)?;
    let Seam::Circle(so, sn, sr) = chained.seam else { unreachable!("`seam_round` reads circles alone") };
    let size = section.size();
    // the legs in the ring's section, on the plane (`c`) and on the wall
    let ends_of = |s: &(Stroke, &'static str), start: bool| match *s {
        (Stroke::Line { from, to }, _) => Some(if start { from } else { to }),
        _ => None,
    };
    let strokes = &ring.section.strokes;
    let (on_plane, on_wall) = if c == 0 {
        (ends_of(&strokes[0], false), ends_of(&strokes[strokes.len() - 1], true))
    } else {
        (ends_of(&strokes[strokes.len() - 1], true), ends_of(&strokes[0], false))
    };
    let on_plane = on_plane.ok_or_else(|| unbuilt("a leg on a plane no line"))?;
    let off_axis = |q: V| { let v = sub(q, centre); norm(sub(v, scale(axis, dot(v, axis)))) };
    let off_seam = |q: V| { let v = sub(q, so); norm(sub(v, scale(sn, dot(v, sn)))) };
    // the edge and the wall's strip short of the seam's normal surface, all the way round
    let d = off_seam(centre);
    let wall_reach = on_wall.map_or(radius, off_axis).max(radius);
    if d + wall_reach >= sr - 1e-9 * (sr + size) {
        return Err(spilled("round a circle its edge or its wall's strip crosses the seam's normal surface"));
    }
    // where the leg on the plane (or the ball's touch), a circle about the ring's axis, crosses the
    // seam
    let rl = off_axis(on_plane);
    let u = normalised(sub(so, centre)).ok_or_else(|| spilled("round a circle about the seam's axis"))?;
    let crate::brep::plane2::Circles::Cross(a, h) = crate::brep::plane2::circles(d, rl, sr, crate::brep::plane2::Touch::Exact)
    else {
        return Err(spilled("round a circle whose leg does not cross the seam"));
    };
    let across = cross(axis, u);
    let mid = add(centre, scale(u, a));
    let ends = [add(mid, scale(across, h)), add(mid, scale(across, -h))];
    let at = add(centre, scale(normalised(sub(on_plane, centre)).unwrap_or(u), radius));
    let beyond = beyond_of(body, &chained, faces[1 - c], at, ends, c, within, section)?;
    if let Section::Flat { .. } = section {
        ring.section.kept.push(Kept { face: chained.split, inside: true });
        return Ok(beyond);
    }
    // a round's ring: the stretch from where its ball touches the seam at one end round to the other,
    // the short way between them the piece beyond's
    let x = normalised(sub(ring.section.origin, centre)).ok_or("a ring of no radius")?;
    let angle = |q: V| { let w = sub(q, centre); dot(w, cross(axis, x)).datan2(dot(w, x)) };
    let tau = std::f64::consts::TAU;
    let (t0, t1) = (angle(ends[0]), angle(ends[1]));
    let (from, span) = if (t1 - t0).rem_euclid(tau) < std::f64::consts::PI { (t1, (t1 - t0).rem_euclid(tau)) }
        else { (t0, (t0 - t1).rem_euclid(tau)) };
    ring.section = super::assemble::turned(&ring.section, &crate::brep::geom::Rigid::turn(centre, axis, from));
    ring.angle = tau - span;
    Ok(beyond)
}

/// A flat cut along the traced `run` of edges between the groups `faces`, face `p` of group `c` — a
/// plane — going on tangent across a seam its leg reaches (`seams`, each with the face beyond: a
/// line into a cylinder along it, or a circle into a torus about the plane's normal): the flat along
/// the run kept short of the seam's normal surface (the plane through a line seam along the
/// plane's normal, the cylinder through a circle one), and the piece beyond it, sectioned square to
/// the run between where the leg on the plane crosses the seam (#422); a round's ball rolled along
/// the run with the stretch where it touches past the seam cut away — between the planes of its arc
/// where it touches the seam — and the ball rolled against the next face there, glued at both: the
/// piece beyond held by the run's (`Rolled::beyond`).  Refused by name where the run, or the leg
/// more than twice, crosses the seam's normal surface; the chain's own refusals are `chain_seam`'s.
#[allow(clippy::too_many_arguments)]
fn traced_beyond(body: &crate::solid::faces::BodyFaces, run: &[u32], faces: [&[usize]; 2], c: usize, p: usize,
    seams: &[(u32, usize)], within: bool, section: Section) -> Result<Piece, String> {
    let at = body.middle(run[0]);
    let chained = chain_seam(body, p, seams, at)?;
    let split = chained.split;
    let samples: Vec<V> = run.iter().flat_map(|&e| (0..=64).map(move |k| (e, k as f64 / 64.0)))
        .map(|(e, s)| body.along(e, s)).collect();
    if samples.iter().any(|&q| split.side(q) >= 0.0) {
        return Err(spilled("along a traced edge that crosses the seam's normal surface"));
    }
    // where the leg on the plane crosses the seam: its footprint read along the run, and bisected
    let o = 1 - c;
    let fo = |e: u32| body.faces_of(e).iter().copied().find(|f| faces[o].contains(f));
    let leg = |e: u32, s: f64| -> Option<V> { super::footprint(body, p, fo(e)?, body.along(e, s), within, section, c) };
    let mut ends: Vec<V> = Vec::new();
    for &e in run {
        let n_ = 64;
        let mut last: Option<(f64, f64)> = None;
        for j in 0..=n_ {
            let s = j as f64 / n_ as f64;
            let Some(q) = leg(e, s) else { last = None; continue };
            let side = split.side(q);
            if let Some((s0, side0)) = last {
                if (side0 < 0.0) != (side < 0.0) {
                    let (mut a, mut z) = (s0, s);
                    for _ in 0..60 {
                        let m = 0.5 * (a + z);
                        let Some(qm) = leg(e, m) else { break };
                        if (split.side(qm) < 0.0) == (side0 < 0.0) { a = m } else { z = m }
                    }
                    ends.extend(leg(e, 0.5 * (a + z)));
                }
            }
            last = Some((s, side));
        }
    }
    let [q0, q1] = ends[..] else {
        return Err(spilled("along a traced edge whose section's footprint crosses the seam other than twice"))
    };
    let w = fo(run[0]).ok_or_else(|| unbuilt("an edge not between two faces"))?;
    let beyond = beyond_of(body, &chained, w, at, [q0, q1], c, within, section)?;
    // the piece along the run from its first edge's middle, as a run's with no seam beyond it (a
    // closed run's piece is seamed where it starts, which may cross the piece beyond: `glued`
    // takes either side's crossing in pieces, #424), the flat's legs told apart by a point beside
    // the first group's face there
    let e = run[0];
    let ends_of: Vec<V> = body.run_ends(run).into_iter().map(|(v, ..)| v).collect();
    let own = match section {
        Section::Round(r) => {
            // the ball's centres where it touches the seam, and the planes of its arc there, square
            // to its centre's path (where the plane's offset meets the wall's), each's normal into
            // the stretch between
            let n = chained.normal;
            let centres = [q0, q1].map(|q| add(q, scale(toward(n, within), r)));
            let wedge: Vec<(V, V)> = (0..2).map(|k| {
                let d = normalised(cross(n, beyond.wall.gradient(centres[k])))
                    .ok_or("a wall along the plane's normal")?;
                Ok((centres[k], if dot(d, sub(centres[1 - k], centres[k])) < 0.0 { scale(d, -1.0) } else { d }))
            }).collect::<Result<_, String>>()?;
            Rolled { at, r, flat: None, ends: ends_of, cuts: Vec::new(), kept: Vec::new(), wedge, beyond: None }
        }
        Section::Flat { la, lb } => {
            let of = |k: usize| body.faces_of(e).iter().copied().find(|f| faces[k].contains(f))
                .ok_or("an edge not between two faces");
            let (fa, fb) = (of(0)?, of(1)?);
            let na = body.normal(fa, at);
            let t = toward(body.normal(fb, at), within);
            let wa = normalised(sub(t, scale(na, dot(t, na)))).ok_or("faces meeting tangent")?;
            Rolled { at, r: 0.0, flat: Some(([la, lb], add(at, scale(wa, 0.5 * la)))), ends: ends_of,
                cuts: Vec::new(), kept: vec![Kept { face: split, inside: true }], wedge: Vec::new(), beyond: None }
        }
    };
    Ok(Piece::Rolled(Rolled { beyond: Some(beyond), ..own }))
}

/// A chain's seam as the piece beyond it reads it (#426), where face `p` goes on tangent across
/// `seams` (its pieces, each with the face beyond): the plane's unit normal, the face beyond, the
/// seam, its normal surface and the next face's surface.
struct Chained {
    normal: V,
    next: usize,
    seam: Seam,
    /// The seam's normal surface, negative on the edge's side: the plane through a line seam along
    /// the plane's normal, turned so; the cylinder through a circle one.
    split: Surf,
    chain: Surf,
}

/// The chain's seam a ring off the seam's axis (`beyond`) and a traced run (`traced_beyond`) both
/// read, `at` a point of the edge (#426): face `p` a plane going on into one face — a cylinder along
/// a straight seam, or a torus about the plane's normal round a circle about it — across every piece
/// of one line or circle.  Each refusal is written once, here, for both.
fn chain_seam(body: &crate::solid::faces::BodyFaces, p: usize, seams: &[(u32, usize)], at: V)
    -> Result<Chained, String> {
    let Some((n, _)) = body.plane(p) else { return Err(spilled("from a face no plane")) };
    let next = next_face(body, seams)?;
    let chain = Surf::of(body, next);
    let size = 1.0 + norm(at);
    // every piece of the seam on its first's line or circle
    let one = |s: u32| match (body.segment(seams[0].0), body.circle(seams[0].0)) {
        (Some([q0, q1]), _) => body.segment(s).is_some_and(|ends| ends.iter().all(|&q| {
            let d = normalised(sub(q1, q0)).unwrap_or([0.0; 3]);
            norm(cross(sub(q, q0), d)) <= 1e-9 * size
        })),
        (_, Some((o, m, r, _))) => body.circle(s).is_some_and(|(p, k, q, _)| distance(p, o) <= 1e-9 * (1.0 + norm(o))
            && norm(cross(m, k)) <= 1e-9 && (q - r).abs() <= 1e-9 * (1.0 + r)),
        _ => false,
    };
    if !seams.iter().all(|&(s, _)| one(s)) {
        return Err(spilled("beside two seams"));
    }
    let (seam, split, chain) = match (body.segment(seams[0].0), body.circle(seams[0].0), chain) {
        (Some([q0, q1]), _, Some(t @ Surf::Cylinder { axis, .. }))
            if norm(cross(axis, sub(q1, q0))) <= 1e-9 * distance(q0, q1) => {
            let d = normalised(sub(q1, q0)).ok_or("a seam of no length")?;
            let m = normalised(cross(d, n)).ok_or("a seam along the plane's normal")?;
            let m = if dot(m, sub(at, q0)) > 0.0 { scale(m, -1.0) } else { m };
            (Seam::Line(q0, d), Surf::Plane { origin: q0, normal: m }, t)
        }
        (_, Some((o, axis, r, _)), Some(t @ Surf::Torus { axis: ta, .. }))
            if norm(cross(axis, n)) <= 1e-9 && norm(cross(ta, n)) <= 1e-9 => {
            (Seam::Circle(o, axis, r), Surf::Cylinder { origin: o, axis, radius: r }, t)
        }
        _ => return Err(spilled("into a face no cylinder along a straight seam nor torus about the plane's normal")),
    };
    Ok(Chained { normal: n, next, seam, split, chain })
}

/// The piece beyond a chain's seam (#426): `chained` as `chain_seam` reads it, the edge's other face
/// `w` (the wall), `at` a point of the edge, `ends` where the leg on the plane (or the ball's touch)
/// crosses the seam, and the section — a flat's legs on the chain from the wall first, face `c` of
/// the edge's two the chain's.
#[allow(clippy::too_many_arguments)]
fn beyond_of(body: &crate::solid::faces::BodyFaces, chained: &Chained, w: usize, at: V, ends: [V; 2], c: usize,
    within: bool, section: Section) -> Result<Beyond, String> {
    let wall = Surf::of(body, w)
        .ok_or_else(|| spilled("beside a face neither a plane, a cylinder, a cone nor a sphere"))?;
    let orient = |fi: usize, s: &Surf, q: V| toward_region(body, fi, s, q, within, section.size());
    let n = chained.normal;
    Ok(Beyond {
        face: Surf::Plane { origin: at, normal: n }, toward: toward(n, within), chain: chained.chain, wall,
        sides: [orient(chained.next, &chained.chain, ends[0]), orient(w, &wall, at)], seam: chained.seam,
        section: match section {
            Section::Flat { la, lb } => {
                Across::Flat { legs: if c == 0 { [la, lb] } else { [lb, la] }, split: chained.split }
            }
            Section::Round(radius) => Across::Round { radius },
        },
        ends,
    })
}

/// Along a straight edge from `p0` to `p1` (or several edges of one line, a curved face cutting
/// across between them that the run is kept outside of all along, `inner`: `across_cuts`): the
/// section beside it swept along it, carried on past each end until every face there cuts it off.
#[allow(clippy::too_many_arguments)]
fn run_piece(body: &crate::solid::faces::BodyFaces, [p0, p1]: [V; 2], fa: usize, fb: usize, within: bool,
    section: Section, ends: &Ends, inner: &[Kept], spill: &Spill) -> Result<Piece, Refused> {
    let length = norm(sub(p1, p0));
    let along = scale(sub(p1, p0), 1.0 / length);
    // each face as the section sees it: a plane its line, a cylinder about a line along the edge
    // its circle
    let trace = |fi: usize, at: V| -> Result<Trace, String> {
        if let Some((n, _)) = body.plane(fi) {
            return Ok(Trace::Line(n));
        }
        match body.cylinder(fi) {
            Some((o, d, radius)) if norm(cross(d, along)) <= 1e-9 => Ok(Trace::Circle {
                centre: add(o, scale(d, dot(sub(p0, o), d))), radius, normal: body.normal(fi, at),
            }),
            _ => Err(unbuilt("an edge beside a curved face not turned about a line along it")),
        }
    };
    let traces = [trace(fa, p0)?, trace(fb, p0)?];
    // a face going on across a seam its section reaches (#422): a line along the edge, the face
    // beyond a plane or a cylinder along it too, the section against the chain all along
    let chain = chained([fa, fb].map(|fi| spill.get(&fi).map(|seams| seam_along(body, seams, p0, along, length)).transpose()))?;
    // carried on past each end until every face there, on the side the edge runs on, cuts it
    // off, however it stands (a run-out): a face square to the edge where the edge ends, one at a
    // slant as far past as the section reaches, kept within its half-space
    let (planes0, bent0) = ends_at(body, ends, p0)?;
    let (planes1, bent1) = ends_at(body, ends, p1)?;
    // the section against the chain, its seams each a point of the section and the face going on
    // across it (none: the corner of the edge's faces alone)
    let against = |c: usize, seams: &[(V, usize)]| {
        let next = seams.iter().map(|&(s, g)| Ok(Next { seam: s,
            trace: trace(g, s).map_err(|_| spilled("into a face not turned about a line along its edge"))? }))
            .collect::<Result<Vec<Next>, String>>()?;
        super::chain::cut_chain(p0, along, traces, (c, &next), within, section)
    };
    // the chain from the edge's face on, each further seam along the edge
    let mut run = match chain {
        None => against(0, &[])?.0,
        Some((c, first)) => chain_from(body, [fa, fb][c], first, |seams| against(c, seams),
            |across| seam_along(body, across, p0, along, length))?.1,
    };
    let outline = run.section_hull();
    run.span = [0.0, length];
    run.edge = length;
    let mut halves = planes0;
    for h in planes1 {
        if !halves.contains(&h) {
            halves.push(h);
        }
    }
    for &(n, d) in &halves {
        let slope = dot(n, along);
        if slope.abs() < 1e-6 {
            return Err(unbuilt("a run carried on along the face it ends on").into());
        }
        for &q in &outline {
            let s = (d - dot(n, q)) / slope;
            if slope > 0.0 { run.span[1] = run.span[1].max(s) } else { run.span[0] = run.span[0].min(s) }
        }
    }
    // a curved face it ends on (#218's 2), a cylinder or a ball: carried on into it and kept on the
    // side the edge runs on — until every line along the prism through its section has left the
    // side kept. Into a solid kept outside of: as far as the deepest entry, short of every exit,
    // so the prism's end lies wholly within it (the entry over the section is convex, the exit
    // concave: each reads its extreme at the hull's points). Out of one kept within: past every
    // exit and the section's width more (a line leaving a convex solid never comes back). A D's
    // flat ends on its round at both ends.
    let width = outline.iter().map(|&q| distance(q, p0)).fold(0.0, f64::max);
    for (forward, bent) in [(false, bent0), (true, bent1)] {
        // each line's crossings, as far along the way it runs on from the edge's start
        let way = if forward { along } else { scale(along, -1.0) };
        for kept in bent {
            let lines: Vec<Option<[f64; 2]>> = outline.iter().map(|&q| kept.face.crossings(q, way)).collect();
            let to = if kept.inside {
                let from = if forward { length } else { 0.0 };
                lines.iter().flatten().map(|l| l[1]).fold(from, f64::max) + 2.0 * width
            } else {
                let deepest = lines.iter().flatten().map(|l| l[0]).fold(f64::NEG_INFINITY, f64::max);
                if !lines.iter().all(|l| l.is_some_and(|l| deepest <= l[1])) {
                    return Err(Refused::PassedBy);
                }
                deepest
            };
            if forward { run.span[1] = run.span[1].max(to) } else { run.span[0] = run.span[0].min(-to) }
            if !run.kept.contains(&kept) {
                run.kept.push(kept);
            }
        }
    }
    // a curved face cutting across between edges of the line, the run kept outside it all along
    for kept in inner {
        if !run.kept.contains(kept) {
            run.kept.push(kept.clone());
        }
    }
    // a run-out carried along the face it ends on no further than that face's edge goes: kept
    // within where that edge ends too, however far the faces at the end carried it (a curved one
    // only where it reaches the prism)
    let reach = bounding(&run.hull());
    for p in [p0, p1] {
        let (planes, bent) = ends_beyond(body, ends, p)?;
        for h in planes {
            if !halves.contains(&h) {
                halves.push(h);
            }
        }
        for kept in bent {
            if !run.kept.contains(&kept) && kept.away(reach.0) >= -reach.1 {
                run.kept.push(kept);
            }
        }
    }
    // only those reaching into the prism: a square end is the prism's own
    let hull = run.hull();
    run.within = halves.into_iter()
        .filter(|&(n, d)| hull.iter().any(|&p| dot(n, p) > d + 1e-9 * (1.0 + length))).collect();
    Ok(Piece::Run(run))
}

/// Which of an edge's two faces goes on across a seam its section reaches, and what it reads of it
/// (none where neither does); refused where both do.
fn chained<T>(sides: [Result<Option<T>, String>; 2]) -> Result<Option<(usize, T)>, String> {
    let [a, b] = sides;
    match (a?, b?) {
        (Some(_), Some(_)) => Err(spilled("on both sides of its edge")),
        (Some(x), None) => Ok(Some((0, x))),
        (None, Some(x)) => Ok(Some((1, x))),
        (None, None) => Ok(None),
    }
}

/// Which way surface `s` (face `fi`'s) runs toward the region beside it at `q` (±1): its side a
/// step out of the face, the region within the material (`within`) or outside it.
fn toward_region(body: &crate::solid::faces::BodyFaces, fi: usize, s: &Surf, q: V, within: bool, size: f64) -> f64 {
    let region = if within { -1.0 } else { 1.0 };
    region * s.side(add(q, scale(body.normal(fi, q), 1e-3 * size))).signum()
}

/// How many faces a chain the section stands against may have past the edge's own.
const CHAIN: usize = 4;

/// The chain a section stands against from the edge's face `own`, going on across the seam `first`
/// (what the section reads of it, and the face beyond): each face the section's footprint lands
/// past going on into the next across another seam (`onto`), read off the edges between them by
/// `seam`; the seams, and the section against them all (`against`).
fn chain_from<S>(body: &crate::solid::faces::BodyFaces, own: usize, first: (S, usize),
    against: impl Fn(&[(S, usize)]) -> Result<(Run, V), String>,
    seam: impl Fn(&[(u32, usize)]) -> Result<(S, usize), String>) -> Result<(Vec<(S, usize)>, Run), String> {
    let (mut seams, mut prev) = (vec![first], own);
    loop {
        let (run, foot) = against(&seams)?;
        let g = seams.last().expect("a seam").1;
        let Some(h) = onto(body, prev, g, foot)? else { return Ok((seams, run)) };
        if seams.len() >= CHAIN {
            return Err(spilled("past the faces it goes on into (a chain of more than four)"));
        }
        // (each surface's pieces one face: `pieces`)
        let beyond = super::pieces(body, h);
        let across: Vec<(u32, usize)> = body.edges_between(&super::pieces(body, g), &beyond)
            .into_iter().filter_map(|e| {
                body.faces_of(e).iter().copied().find(|f| beyond.contains(f)).map(|f| (e, f))
            }).collect();
        seams.push(seam(&across)?);
        prev = g;
    }
}

/// Where the section's footprint on the last face of a chain (`foot`, where its leg or its ball's
/// touch is) lands, face `g` going on from `prev`: on it (none), or past a seam where it goes on
/// tangent into another (that face), the chain going on; refused past any other edge of it.  The
/// pieces of one surface are one face (`pieces`): of `g`'s, the footprint on any is on it, and
/// one of `prev`'s is no face it goes on into.
fn onto(body: &crate::solid::faces::BodyFaces, prev: usize, g: usize, foot: V) -> Result<Option<usize>, String> {
    let surface = super::pieces(body, g);
    let Some(e) = super::off_pieces(body, &surface, foot) else { return Ok(None) };
    match body.faces_of(e).iter().copied().find(|h| !surface.contains(h)) {
        Some(h) if !super::pieces(body, prev).contains(&h) && body.seam(e) => Ok(Some(h)),
        _ => Err(spilled("past the face it goes on into, across an edge no seam")),
    }
}

/// The seam along a straight edge (from `p0` along the unit `along`, `length` long) where a face
/// goes on into the next its section reaches (`seams`, its pieces, each with that face): its point
/// in the section through `p0` — every piece of it a line along the edge, one line, running the
/// whole edge — and that face.
fn seam_along(body: &crate::solid::faces::BodyFaces, seams: &[(u32, usize)], p0: V, along: V, length: f64)
    -> Result<(V, usize), String> {
    let g = next_face(body, seams)?;
    let size = length + norm(p0);
    let (mut at, mut cover): (Option<V>, Vec<[f64; 2]>) = (None, Vec::new());
    for &(s, _) in seams {
        let Some([q0, q1]) = body.segment(s) else { return Err(spilled("beside a straight edge along a curved seam")) };
        if norm(cross(normalised(sub(q1, q0)).unwrap_or([0.0; 3]), along)) > 1e-9 {
            return Err(spilled("along a seam that is not along its edge"));
        }
        let foot = add(q0, scale(along, dot(sub(p0, q0), along)));
        if at.is_some_and(|a| distance(a, foot) > 1e-9 * size) {
            return Err(spilled("beside two seams"));
        }
        at = Some(foot);
        let (a, b) = (dot(sub(q0, p0), along), dot(sub(q1, p0), along));
        cover.push([a.min(b), a.max(b)]);
    }
    cover.sort_by(|x, y| x[0].total_cmp(&y[0]));
    let mut reach = 0.0;
    for [a, b] in cover {
        if a > reach + 1e-9 * size {
            break;
        }
        reach = f64::max(reach, b);
    }
    if reach < length - 1e-9 * size {
        return Err(spilled("beside a seam that runs only part of its edge"));
    }
    Ok((at.expect("a seam"), g))
}

/// The seam round a circle (about `centre` on the unit `axis`) where a face goes on into the next
/// its section reaches (`seams`, its pieces, each with that face): a circle, whole — about the same
/// axis, the face beyond turned about it too, or not — and that face.
fn seam_round(body: &crate::solid::faces::BodyFaces, seams: &[(u32, usize)], centre: V, axis: V)
    -> Result<Round, String> {
    let g = next_face(body, seams)?;
    let mut turn = 0.0;
    let mut circle = None;
    let mut about = true;
    for &(s, _) in seams {
        let Some((o, n, r, _)) = body.circle(s) else { return Err(spilled("round a circle along a seam no circle")) };
        if circle.is_some_and(|(p, m, q): (V, V, f64)| distance(p, o) > 1e-9 * (1.0 + norm(o)) || norm(cross(m, n)) > 1e-9
            || (q - r).abs() > 1e-9 * (1.0 + r)) {
            return Err(spilled("beside two seams"));
        }
        let off = norm(cross(sub(o, centre), axis));
        about &= off <= 1e-9 * (1.0 + norm(centre)) && norm(cross(n, axis)) <= 1e-9 && body.coaxial(g, centre, axis);
        let [t0, t1] = body.span(s);
        turn += if body.closed(s) { std::f64::consts::TAU } else { (t1 - t0).abs() };
        circle = Some((o, n, r));
    }
    if turn < std::f64::consts::TAU - 1e-9 {
        return Err(spilled("beside a seam that runs only part of its edge"));
    }
    let (o, _, r) = circle.expect("a seam");
    Ok(if about { Round::About(o, r, g) } else { Round::Askew })
}

/// The seam round a circle a section reaches across: about the circle's own axis (its centre and
/// radius, and the face beyond), or not (`chain_seam` reads it).
enum Round {
    About(V, f64, usize),
    Askew,
}

/// The one face a seam's pieces (`seams`, each with the face going on across it) go on into: the
/// first's, the others pieces of its surface (`pieces`).
fn next_face(body: &crate::solid::faces::BodyFaces, seams: &[(u32, usize)])
    -> Result<usize, String> {
    let g = seams.first().ok_or_else(|| spilled("into a face it meets in no edge"))?.1;
    let surface = super::pieces(body, g);
    if seams.iter().any(|&(_, h)| !surface.contains(&h)) {
        return Err(spilled("into more than one face"));
    }
    Ok(g)
}

/// The faces of a run's `ends` (`end_faces`' faces and the sides it runs on) standing at `p`, one
/// end of an edge of it: each plane's half-space `n·x <= d` it is kept within, and each cylinder,
/// cone or ball it is kept within (where that side holds its axis, its centre) or outside of.
fn ends_at(body: &crate::solid::faces::BodyFaces, ends: &Ends, p: V) -> Result<(Vec<(V, f64)>, Vec<Kept>), String> {
    let at = |&&(c, _, far): &&(EndBy, f64, Option<V>)| {
        far.is_none() && body.end_side(&c, p).0.abs() <= 1e-7 * (1.0 + norm(p))
    };
    kept_by(body, ends.iter().filter(at))
}

/// The faces of a run's `ends` where the edge a run-out at its end `p` runs along ends
/// (`end_faces`), as `ends_at` reads them: the run carried past `p` is kept within them too,
/// however far it is carried past the faces at `p`.
fn ends_beyond(body: &crate::solid::faces::BodyFaces, ends: &Ends, p: V) -> Result<(Vec<(V, f64)>, Vec<Kept>), String> {
    let beyond = |&&(_, _, far): &&(EndBy, f64, Option<V>)| {
        far.is_some_and(|v| distance(v, p) <= 1e-9 * (1.0 + norm(p)))
    };
    kept_by(body, ends.iter().filter(beyond))
}

/// A ball holding `points`: its centre (their box's) and radius.
fn bounding(points: &[V]) -> (V, f64) {
    let (lo, hi) = points.iter().fold(([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]), |(lo, hi), p| {
        (std::array::from_fn(|i| lo[i].min(p[i])), std::array::from_fn(|i| hi[i].max(p[i])))
    });
    let centre = scale(add(lo, hi), 0.5);
    (centre, points.iter().map(|&p| distance(p, centre)).fold(0.0, f64::max))
}

/// Faces of a run's ends, each with the side it runs on: each plane's half-space `n·x <= d`, and
/// each cylinder, cone or ball kept within or outside of.
fn kept_by<'a>(body: &crate::solid::faces::BodyFaces,
               ends: impl Iterator<Item = &'a (EndBy, f64, Option<V>)>)
    -> Result<(Vec<(V, f64)>, Vec<Kept>), String> {
    let (mut planes, mut bent) = (Vec::new(), Vec::new());
    for &(by, sign, _) in ends {
        let c = match by {
            EndBy::Face(c) => c,
            // square to the run where an edge a run-out goes along ends
            EndBy::Square { at, out } => {
                planes.push(half(out, at, sign));
                continue;
            }
        };
        match Surf::of(body, c) {
            Some(Surf::Plane { origin, normal }) => planes.push(half(normal, origin, sign)),
            Some(face) => bent.push(Kept { face, inside: sign * body.side(c, face.within()) < 0.0 }),
            None => {
                return Err(unbuilt("an edge cut off by a face neither a plane, a cylinder, a cone nor a sphere"))
            }
        }
    }
    Ok((planes, bent))
}

/// Along a turn from where `over` is nothing or less, where it first goes over nothing and where
/// it next comes back to nothing (a whole turn where it never does): sampled at a 720th of a turn,
/// each crossing refined; none where it never goes over.
fn dwell(over: impl Fn(f64) -> f64) -> Option<[f64; 2]> {
    let step = std::f64::consts::TAU / 720.0;
    let values: Vec<f64> = (0..=720).map(|j| over(j as f64 * step)).collect();
    let root = |j: usize| {
        let (a, b) = ((j - 1) as f64 * step, j as f64 * step);
        crate::roots::bracketed_root(|x| Some(over(x)), a, values[j - 1], b, values[j], 1e-15)
    };
    let j = values.iter().position(|&v| v > 0.0)?;
    let first = if j == 0 { 0.0 } else { root(j) };
    let back = (j + 1..=720).find(|&i| values[i] <= 0.0).map_or(std::f64::consts::TAU, root);
    Some([first, back])
}

/// Of `a cos φ + b sin φ + c` over `φ >= 0`, where it first goes over nothing (at once where it
/// starts over); none where it never does.  `a cos φ + b sin φ` is `A cos(φ − phase)`, rising
/// through `−c` at `phase − spread` and falling at `phase + spread`: from nothing or less it first
/// goes over where it rises, never where it falls (#948).  A section's point on the end's plane
/// (the edge's own end, `a + c` nothing) rises at the start itself, its root read a rounding below
/// it — a whole turn on, which the falling root came before: the start.
fn first_over(a: f64, b: f64, c: f64) -> Option<f64> {
    use std::f64::consts::TAU;
    if a + c > 0.0 {
        return Some(0.0);
    }
    let amplitude = a.dhypot(b);
    if amplitude <= -c {
        return None;
    }
    let (phase, spread) = (b.datan2(a), (-c / amplitude).dacos());
    let rise = (phase - spread).rem_euclid(TAU);
    Some(if rise > TAU - 1e-9 { 0.0 } else { rise })
}

/// The half-spaces `n·x <= d` of the planes among a run's `ends` (`end_faces`' faces and the
/// sides it runs on).
fn ends_at_all(body: &crate::solid::faces::BodyFaces, ends: &Ends) -> Vec<(V, f64)> {
    ends.iter().filter_map(|&(by, sign, _)| match by {
        EndBy::Face(c) => body.plane(c).map(|(n, o)| half(n, o, sign)),
        EndBy::Square { at, out } => Some(half(out, at, sign)),
    }).collect()
}

/// The half-space `n·x <= d` of the plane through `at` square to `n`, on the side `sign` keeps.
fn half(n: V, at: V, sign: f64) -> (V, f64) {
    (scale(n, sign), sign * dot(n, at))
}

/// A face as a section square to the edge sees it at the edge's point: a line, its unit normal
/// away from the material; or a circle about `centre` (in the section), its normal there.
#[derive(Clone, Copy, Debug)]
pub(super) enum Trace {
    Line(V),
    Circle { centre: V, radius: f64, normal: V },
}

impl Trace {
    pub(super) fn normal(self) -> V { match self { Trace::Line(n) | Trace::Circle { normal: n, .. } => n } }
}

/// The section's corner as `brep::corners` reads a meridian's: in the plane square to `along` at
/// `p0` (coordinates along `x` and `along × x`, so a turn to the left is one about `along`), face
/// `a`'s trace a step arriving at the corner and `b`'s one leaving it, each `reach` long, and the
/// side the region lies on (+1 the left of travel). A round or a flat there (`chain::cut_chain`) is
/// then what it is in a meridian, by the same rule (`corners::arc_between`, `corners::sum_between`):
/// between lines and circles, a circle's flat the conic its sum of distances makes.
pub(super) struct Corner2 {
    p0: V,
    x: V,
    y: V,
    pub(super) steps: [crate::brep::planar::Seg; 2],
    pub(super) side: f64,
}

impl Corner2 {
    pub(super) fn new(p0: V, along: V, traces: [Trace; 2], [ua, ub]: [V; 2], reach: f64) -> Result<Corner2, String> {
        use crate::brep::planar::Seg;
        let x = normalised(sub(ua, scale(along, dot(ua, along)))).ok_or("a face square to the edge")?;
        let y = cross(along, x);
        let flat = |p: V| [dot(sub(p, p0), x), dot(sub(p, p0), y)];
        // each face's step: from the edge along the face away from the other (toward the region
        // from it), its arc no longer than a quarter turn
        let step = |t: Trace, u: V| -> Result<Seg, String> {
            let n = t.normal();
            let e = normalised(sub(u, scale(n, dot(u, n)))).ok_or("faces lying in one plane")?;
            Ok(match t {
                Trace::Line(_) => Seg::Line { a: [0.0; 2], b: flat(add(p0, scale(e, reach))) },
                Trace::Circle { centre, radius, .. } => {
                    let c = flat(centre);
                    let a0 = (-c[1]).datan2(-c[0]);
                    let way = dot(cross(sub(p0, centre), e), along).signum();
                    Seg::Arc { c, r: radius, a0, sweep: way * (reach / radius).min(0.5 * std::f64::consts::PI) }
                }
            })
        };
        let (sa, sb) = (step(traces[0], ub)?, step(traces[1], ua)?);
        // the region's side, left of travel arriving along `a`: `a`'s way in is the step's way out
        let t = sa.tangent(0.0);
        let toward = flat(add(p0, add(ua, ub)));
        let side = if -t[1] * toward[0] + t[0] * toward[1] < 0.0 { 1.0 } else { -1.0 };
        Ok(Corner2 { p0, x, y, steps: [sa.reversed(), sb], side })
    }

    pub(super) fn lift(&self, q: [f64; 2]) -> V { add(self.p0, add(scale(self.x, q[0]), scale(self.y, q[1]))) }

    /// A point of the section's plane in its coordinates.
    pub(super) fn at2(&self, p: V) -> [f64; 2] { [dot(sub(p, self.p0), self.x), dot(sub(p, self.p0), self.y)] }

    /// A step of the section's plane as a stroke in space: an arc's angles from `x`, turning about
    /// the edge's way (the section's `x` to its `y`).
    pub(super) fn stroke(&self, g: &crate::brep::planar::Seg) -> Stroke {
        use crate::brep::planar::Seg;
        match *g {
            Seg::Line { a, b } => Stroke::Line { from: self.lift(a), to: self.lift(b) },
            Seg::Arc { c, r, a0, sweep } => Stroke::Arc {
                centre: self.lift(c), radius: r, x: self.x, angles: if sweep > 0.0 { [a0, a0 + sweep] } else { [a0 + sweep, a0] },
            },
            Seg::Conic { a, m, b, w } => Stroke::Conic { a: self.lift(a), m: self.lift(m), b: self.lift(b), w },
        }
    }
}
