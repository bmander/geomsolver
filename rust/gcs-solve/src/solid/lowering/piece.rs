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
        // each edge's ends as the field reads them: the run of the references' edges through the
        // corner, a face in pieces of one name (a top split by a cut) going on across its seams,
        // so the end where a straight run ends on no face is where the reference's edge ends
        // (#962), and not where one piece of it does
        for &[i, j] in f.pairs() {
            let own = body.edges_between(&[ids[i]], &[ids[j]]);
            let es = body.runs(&body.edges_between(&groups[i], &groups[j])).into_iter()
                .find(|run| own.iter().any(|e| run.contains(e))).unwrap_or(own);
            let faces: Vec<usize> = groups[i].iter().chain(&groups[j]).copied().collect();
            bounds.extend(ends_at_all(body, &body.end_faces(&es, &faces)));
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
    // a drafted shell's outline turning both ways) — as far as the field reads it (#962): a name in
    // pieces apart, each piece's own chain, read at a point as the one whose piece is nearest it
    // — each named face with the pieces of its surface (`going_on`): a face the same surface goes
    // on into is no other face of the chain, the section as far from it as from its own (#994)
    type Alt = (Vec<usize>, Vec<usize>, bool);
    let surface: [Vec<usize>; 2] = groups.map(|g| super::going_on(body, g).0);
    let chains: Vec<(usize, Vec<Alt>)> = (0..2).filter(|&k| f.faces[k].smooth).map(|k| {
        (k, body.chains_apart(&surface[k]).into_iter().map(|(named, chain)| {
            let carried = body.bulging(&chain, true, !f.within) == Some(true);
            (named, chain.into_iter().filter(|g| !surface[k].contains(g)).collect(), carried)
        }).collect())
    }).collect();
    if chains.iter().all(|(_, alts)| alts.iter().all(|(_, others, _)| others.is_empty())) {
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
                // within a chain turning both ways are no convex set (over its edge, or as much of
                // it as it runs along: one ending in a standing ball's section, `stops_short`)
                let (lo, hi) = (run.span[0].max(0.0), run.span[1].min(run.edge));
                at.extend([0.5, 0.0, 1.0, 0.125, 0.25, 0.375, 0.625, 0.75, 0.875]
                    .map(|s| add(q, scale(run.along, lo + s * (hi - lo)))));
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
        for (k, alts) in &chains {
            let Some((_, others, carried)) = body.nearest_named(q, alts, |a| &a.0[..]) else {
                continue;
            };
            // how far the ball must stand off the others: its radius, or a flat's point as far as
            // from its own face
            let need = match section {
                Section::Round(r) => r,
                Section::Flat { .. } => surface[*k].iter().map(|&g| body.patch_distance(g, q)).fold(f64::INFINITY, f64::min),
            };
            // read as the field reads the centres: off each face's carried side where the chain
            // bulges toward the ball — of the faces it comes too near, the one whose patch is
            // nearest (a wall, not the arc carried on along the wall's plane past its seam) — else
            // off the side of the face whose patch is nearest, where that is another's
            let found = if let Section::Flat { .. } = section {
                // a flat's sum reads each face's patch (`distance(p, b.smooth)`): its point's
                // nearest piece of the chain must be its own
                others.iter().map(|&g| (g, body.patch_distance(g, q))).filter(|&(_, d)| d < need - tol)
                    .min_by(|a, b| a.1.total_cmp(&b.1))
            } else if *carried {
                nearest_patch(body, q, others.iter().map(|&g| (g, toward(g, q))).filter(|&(_, d)| d < need - tol))
            } else {
                let nearest = nearest_patch(body, q, surface[*k].iter().chain(others).map(|&g| (g, 0.0)));
                nearest.filter(|&(g, _)| !surface[*k].contains(&g)).map(|(g, _)| (g, sigma * body.side(g, q)))
                    .filter(|&(_, d)| d < need - tol)
            };
            let Some((g, d)) = found else { continue };
            let said = |fi: usize| face_said(body, fi, "a face");
            let own = said(groups[*k][0]);
            // the arc of the chain tighter than the ball between, where it meets the other face: the
            // corner's radius where the round runs (a drafted one's there), the ball's against it
            let tight = nearest_patch(body, q, others.iter().filter(|&&h| body.plane(h).is_none()).flat_map(|&h| {
                body.edges_between(&[h], groups[1 - *k]).into_iter()
                    .filter_map(move |e| body.circle(e).map(|(_, _, radius, _)| (h, radius)))
            }).filter(|&(_, radius)| radius < need - tol));
            // to the fewest figures, three at least, that tell the ball's radius from how near it
            // comes and from the arc's
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
                Section::Round(r) => {
                    // a face as tight as the ball: it may stand still against it
                    let tight = |g: &usize| body.cylinder(*g).map(|c| c.2).or_else(|| body.torus(*g).map(|t| t.3))
                        .is_some_and(|radius| (radius - r).abs() <= 1e-9 * (r + norm(at)));
                    let stands = ga.iter().chain(gb).any(tight);
                    Rolled { at, r, flat: None, ends, cuts: Vec::new(), kept: Vec::new(), wedge: Vec::new(),
                        beyond: None, beside: None, stands }
                }
                Section::Flat { la, lb } => {
                    let Some((fa, fb)) = ordered(e) else {
                        return Err(unbuilt("an edge not between two faces"));
                    };
                    let na = body.normal(fa, at);
                    let t = toward(body.normal(fb, at), within);
                    let w = normalised(sub(t, scale(na, dot(t, na)))).ok_or("faces meeting tangent")?;
                    Rolled { at, r: 0.0, flat: Some(([la, lb], add(at, scale(w, 0.5 * la)))), ends, cuts: Vec::new(),
                        kept: Vec::new(), wedge: Vec::new(), beyond: None, beside: None, stands: false }
                }
            }))
        }).collect();
    }
    let all: Vec<usize> = ga.iter().chain(gb).copied().collect();
    let ends = body.end_faces(&edges, &all);
    // (the set's edges one run: a point is read against the meeting within that run's ends alone,
    // so where it comes back past an end is the set's own extent there; in several runs each point
    // is read against the ends of the run nearest it, #881)
    let one = body.runs(&edges).len() == 1;
    if let Section::Round(r) = section {
        // the kernel's tolerance over the body (`geom::tolerance` of its box's diagonal)
        let tol = { let (lo, hi) = body.bounds(); crate::brep::geom::tolerance(distance(lo, hi)) };
        for &edge in edges.iter().filter(|&&e| body.convex(e) == Some(within)) {
            if let Some(why) = nearly_tangent(body, edge, &ends, r, tol, units.name()) {
                return Err(why);
            }
        }
    }
    let mut built = Vec::new();
    for &edge in &edges {
        let Some((fa, fb)) = ordered(edge) else {
            return Err(unbuilt("an edge not between two faces"));
        };
        built.push(piece(body, edge, fa, fb, within, section, &ends, spill, one, units));
    }
    // pieces of one line a curved face cuts across, which some of the section's lines pass by: one
    // run over them, kept outside it
    for (group, span, inner) in across_cuts(body, &edges, &built, &ends) {
        let (fa, fb) = ordered(group[0]).expect("a straight edge between two faces");
        let (fa, fb, section) = in_body_order(fa, fb, section);
        let whole = run_piece(body, span, fa, fb, within, section, &ends, &inner, spill, one);
        for (k, &e) in edges.iter().enumerate() {
            if group.contains(&e) {
                built[k] = Ok(Vec::new());
            }
        }
        let k = edges.iter().position(|&e| e == group[0]).expect("a group's edge");
        built[k] = whole.map(|p| vec![p]);
    }
    // one no run beyond the face joins: carried to the plane halfway across what the face cuts from
    // its line (`run_piece`'s `halfway`)
    for (k, &edge) in edges.iter().enumerate() {
        if one && matches!(built[k], Err(Refused::PassedBy)) {
            let (fa, fb) = ordered(edge).expect("a straight edge between two faces");
            let (fa, fb, section) = in_body_order(fa, fb, section);
            let span = body.segment(edge).expect("a straight edge");
            built[k] = run_piece(body, span, fa, fb, within, section, &ends, &[], spill, true).map(|p| vec![p]);
        }
    }
    let mut pieces = Vec::new();
    for b in built {
        pieces.extend(b?);
    }
    Ok(pieces)
}

/// **A corner too nearly tangent to round** (#962): where a round's edge ends on a curved face (a
/// cylinder, a cone or a sphere, `R` its radius there) that meets one of the edge's own faces at a
/// turn `θ` past the body's reading of tangent (`faces::TANGENT`) — so a corner, and no seam the
/// round goes on across — the ball's surface, tangent to the edge's face, meets the curved face at
/// the corner and again `2R sin θ` on round it, where the face's plane comes back across its
/// surface.  The kernel reads a point within its tolerance `tol` of two surfaces meeting at `θ` as
/// on their meeting as far as `ON_CURVE tol / sin θ` off it (`brep::tolerant::on_meeting`), so it
/// tells the two meetings apart only where `2R sin θ` is more than twice that: `sin² θ > ON_CURVE
/// tol / R`.  Short of it the pieces' Booleans read one meeting where there are two and leave a
/// face that does not close ("a cylinder's pieces do not close"; #962's 117 stepped plate with its
/// side turned `1e-5` to `2e-4` off the lobe it leaves, a teardrop's tangents stated to three
/// places).  A chamfer's flat crosses the edge's face, no tangent to it, and is built.  Said by the
/// two faces, the turn, the bound, and what is true to do: state the tangency, or make the corner
/// one.  `tol` is the kernel's over the body (`geom::tolerance` of its box's diagonal), and the
/// bound the kernel's own (`tolerant::least_sine_apart`).
fn nearly_tangent(body: &crate::solid::faces::BodyFaces, edge: u32, ends: &Ends, r: f64, tol: f64, unit: &str)
    -> Option<String> {
    let faces = body.faces_of(edge);
    for p in [body.along(edge, 0.0), body.along(edge, 1.0)] {
        for &(by, _, far) in ends.iter() {
            let (EndBy::Face(c), None) = (by, far) else { continue };
            if body.end_side(&by, p).0.abs() > 1e-7 * (1.0 + norm(p)) {
                continue;
            }
            // the curved face's radius at the corner: a sphere's, or the distance from its axis
            let radius = match Surf::of(body, c)? {
                Surf::Sphere { radius, .. } => radius,
                Surf::Cylinder { origin, axis, .. } | Surf::Cone { origin, axis, .. } => {
                    let w = sub(p, origin);
                    norm(sub(w, scale(axis, dot(w, axis))))
                }
                _ => continue,
            };
            let n = body.normal(c, p);
            let Some((own, turn)) = faces.iter().map(|&f| (f, dot(body.normal(f, p), n).clamp(-1.0, 1.0).dacos()))
                .min_by(|a, b| a.1.total_cmp(&b.1)) else { continue };
            let bound = crate::brep::tolerant::least_sine_apart(radius, tol).dasin();
            // (tangent to the body's reading, no corner: a face going on tangent from the edge's own
            // that carries no round on, which the run ends on running out along it, #994)
            if turn < crate::solid::faces::TANGENT || !(turn < bound) {
                continue;
            }
            let name = |fi: usize| body.said(fi);
            let fig = |x: f64| crate::json::fmt_g(x, 2);
            // (the witness to the power of ten under the kernel's tolerance)
            let grid = 10f64.powi(tol.max(1e-12).log10().floor() as i32);
            let fig6 = |x: f64| crate::json::fmt_g((x / grid).round() * grid, 6);
            return Some(format!("{} and {} meet at a turn of {} rad at [{}, {}, {}]{unit}: not tangent, so their \
                edge is a corner and no seam a round goes on across, and too nearly tangent a corner for this \
                kernel to round at {}{unit} against {}'s radius of {}{unit} — the round meets {} again {}{unit} on round it, \
                closer than the kernel tells two meetings apart at that turn (a turn of {} rad or more is \
                built). State the tangency ({} tangent {}, or a chain's `-> tangent` joint) rather than \
                coordinates rounded off it, or make it a corner of more than {} rad",
                name(own), name(c), fig(turn), fig6(p[0]), fig6(p[1]), fig6(p[2]), fig(r), name(c), fig(radius),
                name(c), fig(2.0 * radius * turn.dsin()), fig(bound), name(own), name(c), fig(bound)));
        }
    }
    None
}

/// Why a straight run is not cut off by a curved face it ends on: some line along it through its
/// section passes the face by (a bore across the crease, a pin through it), so the run goes on past
/// that end — into the run beyond it, where the face cuts the edge in two (`across_cuts`), else to
/// the plane halfway across the chord the face cuts from the edge's line (`run_piece`'s
/// `halfway`), and refused by this name only where the line never comes back out within the
/// faces the run ends on.
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
/// line, between the same surfaces (a floor the face cuts in two is two faces of one plane), ends
/// on it alone too, both are one run over the two and what
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
        // the straight edges between the same surfaces along the same line, each as its span along
        // it: a face the curved one cuts in two (a floor a trench crosses) is two faces of one
        // surface, and a section stands against the surface (#952's `pieces`)
        let between = |e: u32| -> Option<[Surf; 2]> {
            let [f0, f1] = *body.faces_of(e) else { return None };
            Some([Surf::of(body, f0)?, Surf::of(body, f1)?])
        };
        let faces = between(edges[k]);
        let alike = |e: u32| match (between(e), faces) {
            (Some([x0, x1]), Some([y0, y1])) => x0.same(&y0) && x1.same(&y1) || x0.same(&y1) && x1.same(&y0),
            _ => false,
        };
        let on_line = |p: V| norm(cross(sub(p, a0), along)) <= 1e-9 * (1.0 + norm(p) + norm(a0));
        let mut line: Vec<(usize, [f64; 2])> = (0..edges.len()).filter(|&j| !taken[j]).filter_map(|j| {
            let [b0, b1] = body.segment(edges[j])?;
            (alike(edges[j]) && on_line(b0) && on_line(b1)).then(|| {
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

/// The piece along `edge`, where faces `fa` and `fb` meet — a straight one's section swept along
/// it (`run_piece`), a circle's turned about its axis (`ring_piece`) — kept within the half-spaces
/// of the run's `ends` standing at its own ends and reaching into it; `None` where it turns the
/// other way from the side the region asks (`within`, convex).
#[allow(clippy::too_many_arguments)]
fn piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool, section: Section,
    ends: &Ends, spill: &Spill, one: bool, units: crate::units::Units) -> Result<Vec<Piece>, Refused> {
    // the faces in the body's order, so two regions over one edge build one run
    let (fa, fb, section) = in_body_order(fa, fb, section);
    // the gate: an edge turning the other way, or none (faces meeting tangent), holds nowhere
    if body.convex(edge) != Some(within) {
        return Ok(Vec::new());
    }
    // (`run` rolls a circle whose faces are not all about its axis)
    if body.circle(edge).is_some() {
        return Ok(ring_piece(body, edge, fa, fb, within, section, ends, spill, one, units)?);
    }
    let Some(span) = body.segment(edge) else { return Err(unbuilt("a curved edge").into()) };
    let mut built = run_piece(body, span, fa, fb, within, section, ends, &[], spill, false)?;
    // an end whose ball the face going on round a tighter round stops short of it: the run ends in
    // the standing ball's section, the ball's piece past it to the crease's end the region's too
    let mut besides = Vec::new();
    if let (Piece::Run(run), Section::Round(r)) = (&mut built, section) {
        if let Some(centre) = run.centre() {
            let (along, length) = (run.along, norm(sub(span[1], span[0])));
            let size = norm(span[0]) + length + r;
            for (k, p) in [(0, span[0]), (1, span[1])] {
                let at = |s: f64| Some(add(centre, scale(along, s)));
                let crease = |s: f64| add(span[0], scale(along, s));
                if let Some((s, beside)) = stops_short(body, [fa, fb], within, r, &at, &crease, [0.0, length], k, p) {
                    run.span[k] = s;
                    run.release(p, size);
                    besides.push(beside);
                }
            }
        }
    }
    Ok(std::iter::once(built).chain(besides).collect())
}

/// Round a circle `edge` where a plane, a cylinder, a cone or a sphere meet about one axis: the
/// section in the meridian through its start, turned through the circle's angle — a whole turn,
/// or a stretch ending where a face goes on from one of these tangent (an obround's flat side),
/// whose own run goes on from the same section.
#[allow(clippy::too_many_arguments)]
fn ring_piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool,
    section: Section, ends: &Ends, spill: &Spill, one: bool, units: crate::units::Units)
    -> Result<Vec<Piece>, String> {
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
    let mut whole = false;
    // an end whose ball the face going on round a tighter round stops short of it: the ring ends in
    // the standing ball's section, the ball's piece past it to the crease's end the region's too
    let mut besides = Vec::new();
    let mut stopped = [false; 2];
    if let (Section::Round(r), false) = (section, closed) {
        let centre_at = |t: f64| section_at(t).ok().and_then(|s| s.centre());
        for (k, p) in [(0, p0), (1, p1)] {
            let crease = |t: f64| add(centre, scale(radial_at(t), radius));
            if let Some((t, beside)) = stops_short(body, [fa, fb], within, r, &centre_at, &crease, [t0, t1], k, p) {
                if k == 0 { from = t } else { to = t }
                stopped[k] = true;
                besides.push(beside);
            }
        }
    }
    if !closed {
        for (k, p, t) in [(0, p0, t0), (1, p1, t1)] {
            if stopped[k] {
                continue;
            }
            let (planes, bent) = ends_at(body, ends, p)?;
            let planes: Vec<(V, f64)> = planes.into_iter().filter(|h| !meridian(h)).collect();
            if planes.is_empty() && bent.is_empty() {
                // an end at a seam, where the chain the ring goes on into ends on a face reaching
                // back over it (`BodyFaces::chain_ends`): the stretch ends at the seam's meridian,
                // carried no further, and kept within that face where it reaches the ring (#994)
                let (planes, bent) = ends_beyond(body, ends, p)?;
                let reach = section_at(t)?.section_hull().iter().map(|&q| distance(q, centre)).fold(0.0, f64::max);
                halves.extend(planes.into_iter().filter(|&(n, d)| dot(n, centre) - d >= -reach));
                kepts.extend(bent.into_iter().filter(|kept| kept.away(centre) >= -reach));
                continue;
            }
            let way = if k == 0 { -1.0 } else { 1.0 };
            let hull = section_at(t)?.section_hull();
            // a plane square to the axis: along the stretch, cutting it lengthwise or not at all
            // (`along_run`), no end of it
            let (lengthwise, planes): (Vec<(V, f64)>, Vec<(V, f64)>) = planes.into_iter()
                .partition(|&(n, _)| norm(cross(n, axis)) < 1e-6);
            let reach_of = 1e-9 * (1.0 + norm(centre) + radius);
            halves.extend(lengthwise.into_iter().filter(|&h| along_run(&hull, h, reach_of)));
            if planes.is_empty() && bent.is_empty() {
                continue;
            }
            let turned = |q: V, phi: f64| crate::brep::geom::Rigid::turn(centre, axis, way * phi).point(q);
            // the set's own reach past this end: the meridian halfway to where the circle comes
            // back within every face the run ends on (`resumed`) — past it a point's nearest point
            // of the meeting is off the crease — and the ring is that far round, kept within its
            // ends' faces, whatever of its section they cut off
            let gap = std::f64::consts::TAU - (t1 - t0);
            let tol = body.tolerance().max(1e-9 * (1.0 + norm(centre)));
            let back = resumed(|s| ends.iter().map(|&(by, sign, _)| {
                sign * body.end_side(&by, add(centre, scale(radial_at(t + way * s), radius))).0
            }).fold(f64::NEG_INFINITY, f64::max), gap, tol);
            // (within the boundary's tolerance of the crease's other end, read at it: that end's
            // own face, which the circle reaches there)
            let half = 0.5 * if radius * (gap - back) <= 10.0 * tol { gap } else { back };
            // (in several runs, read against the nearest's ends, the meeting is no one circle's
            // within one set of faces: carried only as far as each end cuts its whole section off)
            let half = if one { half } else { f64::INFINITY };
            let mut short = None;
            // short of it, where every point of the section's hull has crossed a plane at this end
            // (#218's 2: what lies past is cut off, and comes back within only past `half`), or
            // into a curved face, a quarter of the way through what each spends in its solid
            let mut reach: Option<f64> = Some(0.0);
            // the section's width as a turn at its rim, for `clear_of`
            let rim = |q: V| { let w = sub(q, centre); sub(w, scale(axis, dot(w, axis))) };
            let turn = hull.iter().map(|&q| distance(q, p)).fold(0.0, f64::max)
                / hull.iter().map(|&q| norm(rim(q))).fold(0.0, f64::max);
            for &(n, d) in &planes {
                // each point's turn past the plane, from where it first crosses it to where it
                // comes back
                let past: Option<Vec<[f64; 2]>> = hull.iter().map(|&q| {
                    let rim = rim(q);
                    let (a, b) = (dot(n, rim), way * dot(n, cross(axis, rim)));
                    over_from(a, b, dot(n, sub(q, rim)) - d)
                }).collect();
                let Some(past) = past else {
                    reach = None;
                    short = short.or(Some("a stretch of a circle its end does not cut off"));
                    continue;
                };
                let first = past.iter().map(|s| s[0]).fold(f64::INFINITY, f64::min);
                let last = past.iter().map(|s| s[0]).fold(0.0, f64::max);
                let back = past.iter().map(|s| s[1]).fold(f64::INFINITY, f64::min);
                // carried clear of it where it reaches in, short of where any point comes back
                let to = clear_of(first, last, turn).min(0.5 * (last + back)).max(last);
                reach = reach.map(|r| r.max(to));
            }
            for kept in &bent {
                let (mut entry, mut exit) = (0.0_f64, std::f64::consts::TAU);
                for &q in &hull {
                    let Some([first, back]) = dwell(|phi| kept.away(turned(q, phi))) else {
                        (entry, exit) = (1.0, 0.0);
                        break;
                    };
                    entry = entry.max(first);
                    exit = exit.min(back);
                }
                reach = if entry < exit { reach.map(|r| r.max(entry + 0.25 * (exit - entry))) } else { None };
                short = short.or(reach.is_none().then_some("a stretch of a circle its curved end does not cut off"));
            }
            let reach = match reach {
                Some(r) => r.min(half),
                None if half.is_finite() => half,
                None => return Err(unbuilt(short.unwrap_or("a stretch of a circle its end does not cut off"))),
            };
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
        // (carried on from both ends to the middle of what lies between them: a whole turn, begun
        // at the edge's middle, so its seam stands where nothing it is kept within cuts it)
        if to - from >= std::f64::consts::TAU - 1e-12 {
            if !one {
                return Err(unbuilt("a stretch of a circle carried on round its whole turn"));
            }
            whole = true;
            from = 0.5 * (t0 + t1);
            to = from + std::f64::consts::TAU;
        }
    }
    let mut start = section_at(from)?;
    start.within = halves;
    start.kept = kepts;
    let angle = if whole { std::f64::consts::TAU } else { to - from };
    let mut ring = Ring { section: start, axis_origin: centre, axis, angle, beyond: None };
    if let Some(c) = askew {
        let seams = spill.get(&[fa, fb][c]).expect("the face going on");
        ring.beyond = Some(beyond(body, edge, [fa, fb], c, seams, within, section, &mut ring)?);
    }
    Ok(std::iter::once(Piece::Ring(ring)).chain(besides).collect())
}

/// **A run's ball stopped short of its crease's end by a round it goes on into** (#962): where
/// face `f` of `faces` goes on tangent into a face whose offset by the ball's radius `r` closes (a
/// frame's floor into its inside edge filled `r`, the fill's offset its axis), the run's ball —
/// its centre `centre_at(s)` over its extent `span`, `k` the end at `vertex` — stands within its
/// faces only up to where its centre meets what that offset closes onto: there it touches the
/// face going on too, and stands still against the other face's meeting with it (`brep::fillet`'s
/// standing ball). Where the end `k` is past that, the run ends in the standing ball's section, and
/// the part of the standing ball's piece past its own stretch's end at `vertex` is this region's
/// (`Rolled::beside`), `crease_at(s)` the run's edge there. The turn or length the run stops at,
/// and that piece; `None` where its ball
/// stands within its faces to the end.
#[allow(clippy::too_many_arguments)]
fn stops_short(body: &crate::solid::faces::BodyFaces, faces: [usize; 2], within: bool, r: f64,
    centre_at: &dyn Fn(f64) -> Option<V>, crease_at: &dyn Fn(f64) -> V, span: [f64; 2], k: usize, vertex: V)
    -> Option<(f64, Piece)> {
    let sigma = if within { -1.0 } else { 1.0 };
    let size = r + norm(vertex);
    let tol = 1e-9 * size;
    // how far from what a face's offset by `r` closes onto a point stands: its axis, or its middle
    // circle; none where the face is no cylinder or torus of radius `r` (whose offset that is)
    let closed = |g: usize| -> Option<Box<dyn Fn(V) -> f64>> {
        if let Some((o, a, _)) = body.cylinder(g).filter(|c| (c.2 - r).abs() <= tol) {
            return Some(Box::new(move |c: V| { let w = sub(c, o); norm(sub(w, scale(a, dot(w, a)))) }));
        }
        let (o, a, big, _) = body.torus(g).filter(|t| (t.3 - r).abs() <= tol)?;
        Some(Box::new(move |c: V| {
            let w = sub(c, o);
            let h = dot(w, a);
            (norm(sub(w, scale(a, h))) - big).hypot(h)
        }))
    };
    let inner = 0.5 * (span[0] + span[1]);
    for (f, &own) in faces.iter().enumerate() {
        // the faces `own` goes on into smoothly, its own surface's pieces aside (`going_on`)
        let chain: Vec<usize> = super::going_on(body, &[own]).1;
        for &g in &chain {
            let Some(off) = closed(g) else { continue };
            // the ball stands within the face going on to its end: nothing stops it
            let past = |s: f64| centre_at(s).map(|c| sigma * body.carried_side_grad(g, c).0 - r);
            if past(span[k]).is_none_or(|d| d >= -tol) {
                continue;
            }
            // where between its middle and that end the centre comes nearest what the offset
            // closes onto: on it, the ball touches the face going on round its circle there
            let far = |s: f64| centre_at(s).map_or(f64::INFINITY, |c| off(c));
            let n = 256;
            let at = |i: usize| inner + (span[k] - inner) * i as f64 / n as f64;
            let (i, _) = (0..=n).map(|i| (i, far(at(i)))).min_by(|a, b| a.1.total_cmp(&b.1))?;
            let (mut lo, mut hi) = (at(i.saturating_sub(1)), at((i + 1).min(n)));
            // (down to the numbers' own resolution there; the bracket runs either way, toward
            // either end)
            let least = 4.0 * f64::EPSILON * (1.0 + lo.abs().max(hi.abs()));
            for _ in 0..200 {
                if (hi - lo).abs() <= least {
                    break;
                }
                let (x, y) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                if far(x) <= far(y) { hi = y } else { lo = x }
            }
            let stop = 0.5 * (lo + hi);
            if far(stop) > 1e-7 * size {
                continue;
            }
            // the other face's meeting with the face going on (a piece of it, where the other face
            // splits it), from this end's vertex: the stretch the ball stands over
            let other = faces[1 - f];
            let ends_at = |e: u32| distance(body.along(e, 0.0), vertex).min(distance(body.along(e, 1.0), vertex));
            let Some(e) = body.edges_between(&[other], &chain).into_iter().filter(|&e| ends_at(e) <= 1e-6 * size)
                .min_by(|a, b| ends_at(*a).total_cmp(&ends_at(*b))) else { continue };
            // (its `ends` where this run now ends: on its edge, the section's corner there)
            return Some((stop, Piece::Rolled(super::Rolled { at: body.middle(e), r, flat: None,
                ends: vec![crease_at(stop)], cuts: Vec::new(), kept: Vec::new(), wedge: Vec::new(),
                beyond: None, beside: Some(vertex), stands: false })));
        }
    }
    None
}

/// `x` to `n` significant figures.
fn figured(x: f64, n: i32) -> f64 {
    if x == 0.0 { return 0.0 }
    let k = 10f64.powi(n - 1 - x.abs().log10().floor() as i32);
    (x * k).round() / k
}

/// The fewest figures, three at least, that tell `a` from `b`.
fn apart(a: f64, b: f64) -> i32 { (3..16).find(|&n| figured(a, n) != figured(b, n)).unwrap_or(16) }

/// Face `fi` as a refusal names it (`BodyFaces::said`), or `none` where it has no name.
fn face_said(body: &crate::solid::faces::BodyFaces, fi: usize, none: &str) -> String {
    match body.paths().get(fi) {
        Some(p) if !p.text().is_empty() => body.said(fi),
        _ => none.to_string(),
    }
}

/// Of `(face, value)` pairs, the one whose face's patch is nearest `q` (`BodyFaces::patch_distance`,
/// read once each).
fn nearest_patch(body: &crate::solid::faces::BodyFaces, q: V, of: impl Iterator<Item = (usize, f64)>)
    -> Option<(usize, f64)> {
    of.map(|(g, d)| (g, d, body.patch_distance(g, q))).min_by(|a, b| a.2.total_cmp(&b.2)).map(|(g, d, _)| (g, d))
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
    let size = section.size();
    let figures = apart(size, radius);
    let length = |x: f64| crate::units::Dim::LENGTH.number_text(figured(x, figures), units);
    // the face the circle's radius is of: a cylinder's or a cone's about the axis, else the other
    let curved = faces.into_iter().find(|&fi| body.plane(fi).is_none()).unwrap_or(faces[0]);
    let other = if curved == faces[0] { faces[1] } else { faces[0] };
    let (face, from) = (face_said(body, curved, "the corner's face"), face_said(body, other, "the other face"));
    let what = if closed { "circle" } else { "arc" };
    Err(match section {
        // the curved face goes on smooth into others (a pocket's walls): the ball stands against
        // those, and the set rounds the corner itself along the face away from the edge
        Section::Round(_) if !super::going_on(body, &[curved]).1.is_empty() => format!("its ball, of radius {}, is wider \
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
            Rolled { at, r, flat: None, ends: ends_of, cuts: Vec::new(), kept: Vec::new(), wedge, beyond: None,
                beside: None, stands: false }
        }
        Section::Flat { la, lb } => {
            let of = |k: usize| body.faces_of(e).iter().copied().find(|f| faces[k].contains(f))
                .ok_or("an edge not between two faces");
            let (fa, fb) = (of(0)?, of(1)?);
            let na = body.normal(fa, at);
            let t = toward(body.normal(fb, at), within);
            let wa = normalised(sub(t, scale(na, dot(t, na)))).ok_or("faces meeting tangent")?;
            Rolled { at, r: 0.0, flat: Some(([la, lb], add(at, scale(wa, 0.5 * la)))), ends: ends_of,
                cuts: Vec::new(), kept: vec![Kept { face: split, inside: true }], wedge: Vec::new(), beyond: None,
                beside: None, stands: false }
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
        run: None,
        seamed: [false; 2],
    })
}

/// Along a straight edge from `p0` to `p1` (or several edges of one line, a curved face cutting
/// across between them that the run is kept outside of all along, `inner`: `across_cuts`): the
/// section beside it swept along it, carried on past each end until every face there cuts it off —
/// or, where some line of it passes a curved one by and `halfway` is asked, to the plane halfway
/// across the chord that face cuts from the edge's line (where the meeting comes back), square.
#[allow(clippy::too_many_arguments)]
fn run_piece(body: &crate::solid::faces::BodyFaces, [p0, p1]: [V; 2], fa: usize, fb: usize, within: bool,
    section: Section, ends: &Ends, inner: &[Kept], spill: &Spill, halfway: bool) -> Result<Piece, Refused> {
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
    // — or a line crossing the edge's footprint aslant (#994), the section against the chain only past
    // where the touch crosses it (`aslant`)
    let beside = [fa, fb].map(|fi| spill.get(&fi).map(|seams| match seam_aslant(body, seams, along)? {
        Some(line) => Ok(Beside::Aslant(line)),
        None => seam_along(body, seams, p0, along, length).map(Beside::Along),
    }).transpose());
    let (chain, aslant) = match chained(beside)? {
        Some((c, Beside::Along(first))) => (Some((c, first)), None),
        Some((c, Beside::Aslant(line))) => (None, Some((c, line))),
        None => (None, None),
    };
    // carried on past each end until every face there, on the side the edge runs on, cuts it
    // off, however it stands (a run-out): a face square to the edge where the edge ends, one at a
    // slant as far past as the section reaches and clear of it (`clear_of`), kept within its
    // half-space
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
    // a seam along the edge into a face no plane nor cylinder along it (#962): the section in the
    // planes square to the edge is no prism's, and the piece beyond the seam runs the edge's whole
    // length; the edge's own run, a flat's, is the corner of its two faces kept short of the seam's
    // normal plane
    let spread = match chain {
        Some((c, (s, g))) if trace(g, s).is_err() => {
            Some(along_beyond(body, [p0, p1], [p0, p1], [fa, fb], c, (s, along, g), within, section, [false; 2])?)
        }
        _ => None,
    };
    // the chain from the edge's face on, each further seam along the edge
    let mut run = match chain {
        Some((c, first)) if spread.is_none() => chain_from(body, [fa, fb][c], first, |seams| against(c, seams),
            |across| seam_along(body, across, p0, along, length))?.1,
        _ => against(0, &[])?.0,
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
    let width = outline.iter().map(|&q| distance(q, p0)).fold(0.0, f64::max);
    for &(n, d) in &halves {
        let slope = dot(n, along);
        // a face along the run cuts it lengthwise or not at all (`along_run`): no end of it
        if slope.abs() < 1e-6 {
            continue;
        }
        // how far along each line of the section the plane stands, the way the run is carried
        let way = slope.signum();
        let (first, last) = outline.iter().map(|&q| way * (d - dot(n, q)) / slope)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), s| (a.min(s), b.max(s)));
        // where it cuts the prism at all — some line crossing it short of the edge's end, or past
        // it — carried clear past it (`clear_of`)
        let end = way * if way > 0.0 { length } else { 0.0 };
        let eps = 1e-12 * (1.0 + norm(p0).max(norm(p1)));
        if first < end - eps || last > end + eps {
            let to = way * clear_of(first, last, width);
            if way > 0.0 {
                run.span[1] = run.span[1].max(to);
            } else {
                run.span[0] = run.span[0].min(to);
            }
        }
    }
    // a curved face it ends on (#218's 2), a cylinder or a ball: carried on into it and kept on the
    // side the edge runs on — until every line along the prism through its section has left the
    // side kept. Into a solid kept outside of: as far as the deepest entry, short of every exit,
    // so the prism's end lies wholly within it (the entry over the section is convex, the exit
    // concave: each reads its extreme at the hull's points). Out of one kept within: past every
    // exit and the section's width more (a line leaving a convex solid never comes back). A D's
    // flat ends on its round at both ends.
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
                    if !halfway {
                        return Err(Refused::PassedBy);
                    }
                    // some line passes it by: the run is the set's as far as the plane halfway to
                    // where the edge's line comes back out of that solid within every face the run
                    // ends on — past it a point's nearest point of the meeting is off the crease —
                    // square there, and kept outside the solid
                    let p = if forward { p1 } else { p0 };
                    let out = |s: f64| ends.iter().map(|&(by, sign, _)| sign * body.end_side(&by, add(p, scale(way, s))).0)
                        .fold(f64::NEG_INFINITY, f64::max);
                    let tol = body.tolerance().max(1e-9 * (1.0 + norm(p)));
                    // (where the edge's line touches the face at the end — a face closing down onto
                    // the crease tangent, a band tapering to nothing — it never goes in: the
                    // meeting comes back at once, and the run ends square there)
                    let touches = (dot(kept.face.gradient(p), way).abs() < crate::solid::faces::TANGENT)
                        .then_some(0.0);
                    let chord = kept.face.crossings(p, way).map(|[_, b]| b).filter(|&b| b > tol).or(touches);
                    let Some(chord) = chord.filter(|&b| out(b + tol) <= tol) else {
                        return Err(Refused::PassedBy);
                    };
                    (if forward { length } else { 0.0 }) + 0.5 * chord
                } else {
                    deepest
                }
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
    // a seam crossing the edge's footprint aslant (#994): the run short of where the touch crosses it,
    // and the piece beyond from there
    if let Some((c, line)) = aslant {
        let piece = aslant_pieces(body, [p0, p1], [fa, fb], c, line, within, section, &mut run, &halves, ends)?;
        if let Some(piece) = piece {
            return Ok(piece);
        }
    }
    // the piece beyond a seam along the edge (#962): carried past each end and cut off by every
    // plane the run is (a curved end refused), the edge's own run a flat's alone, kept short of the
    // seam's normal plane
    if let Some(mut beyond) = spread {
        if !run.kept.is_empty() {
            return Err(spilled("beside a seam along a straight edge into a face no cylinder along it, its run ending on a \
                curved face").into());
        }
        beyond.run = Some(halves.clone());
        let split = match beyond.section {
            Across::Round { .. } => return Ok(Piece::Along { run: None, beyond }),
            Across::Flat { split: Surf::Plane { origin, normal }, .. } => (normal, dot(normal, origin)),
            Across::Flat { .. } => unreachable!("a straight seam's normal surface is a plane"),
        };
        let hull = run.hull();
        run.within = halves.into_iter()
            .filter(|&(n, d)| hull.iter().any(|&p| dot(n, p) > d + 1e-9 * (1.0 + length))).collect();
        run.within.push(split);
        return Ok(Piece::Along { run: Some(run), beyond });
    }
    // only those reaching into the prism: a square end is the prism's own
    let hull = run.hull();
    run.within = halves.into_iter()
        .filter(|&(n, d)| hull.iter().any(|&p| dot(n, p) > d + 1e-9 * (1.0 + length))).collect();
    run.flush = flush_only(body, ends, [p0, p1]);
    Ok(Piece::Run(run))
}

/// **A piece carried past a plane it is cut off at goes clear of it** (#962 item 4): from where
/// the plane first meets the lines along the piece through its section (`first`) to where the
/// last crosses it (`last`), at least a thousandth of the section's `width` past the first, so the
/// plane crosses the piece's sides and lies nowhere along its end.  Carried only to the last, a
/// piece's end was the plane turned about that line by however far the face was off square: a
/// drawing states its numbers to five or six places, so a side is square to the edge only to
/// those — #962's 122 has one through a point 0.8 µm off its other end's line across 67 mm, a
/// turn of 1.2e-8 — and the piece was cut along its own end by a plane a hundred-millionth of a
/// millimetre off it, a sliver the Boolean could not classify ("a piece of `far` (a plane)
/// touches `far` (a plane) … along no face of either"; a little further off, "a shell of the
/// arrangement is kept on one side of a plane … and dropped on another"); a piece ending square
/// but for a hundred-millionth was left with its cap that far off the face, which the Boolean
/// joining it to its body could not read either.  Cut where it crosses the piece's sides, the end
/// is the face's plane itself, exactly, at any slant.  A ring's stretch is turned so (`first`,
/// `last` and `width` turns), no further than halfway to where a point of it comes back.
fn clear_of(first: f64, last: f64, width: f64) -> f64 { last.max(first + 1e-3 * width) }

/// **Two faces flush only to the drawing's numbers where a run ends** (#962 item 4): a plane the
/// run ends on at one of its ends `ps` and another plane of the body through that end, turned from
/// it by less than a thousandth of a radian and more than the kernel reads as parallel — a boss's
/// end drawn flush with a side the solve leaves a hundred-millionth of a radian off it.  The run is
/// cut off exactly at the face it ends on (`clear_of`), and the other stands nanometres beside its
/// end; where the Boolean joining the run to its body cannot close that, its refusal says so, in
/// the faces' names, by how far they are turned, and where.
fn flush_only(body: &crate::solid::faces::BodyFaces, ends: &Ends, ps: [V; 2]) -> Option<String> {
    let name = |f: usize| {
        let path = &body.paths()[f];
        path.route.iter().chain(std::iter::once(&path.face)).cloned().collect::<Vec<_>>().join(".")
    };
    for p in ps {
        let near = |n: V, o: V, by: f64| dot(n, sub(p, o)).abs() <= by * (1.0 + norm(p));
        for &(by, _, far) in ends {
            let (EndBy::Face(c), None) = (by, far) else { continue };
            let Some((n, _)) = body.plane(c).filter(|&(n, o)| near(n, o, 1e-7)) else { continue };
            for g in (0..body.paths().len()).filter(|&g| g != c) {
                let Some((m, _)) = body.plane(g).filter(|&(m, q)| near(m, q, 1e-6)) else { continue };
                let turn = norm(cross(n, m));
                if turn > 1e-12 && turn < 1e-3 && dot(n, m) > 0.0 {
                    return Some(format!("where `{}` and `{}` are flush only to the drawing's numbers, turned {turn:.1e} \
                        of a radian from one another at [{:.4}, {:.4}, {:.4}]", name(c), name(g), p[0], p[1], p[2]));
                }
            }
        }
    }
    None
}

/// **A face along a run cuts it lengthwise or not at all** (#962 item 4): a plane the run ends on
/// that lies along it — a boss's underside where it overhangs the side the run ends at, in the
/// plane of the floor the run stands on; for a ring's stretch, a plane square to its axis — stands
/// at no distance along it, so it is no end and carries the piece nowhere.  Its half-space, the
/// side `BodyFaces::end_faces` keeps (the material's behind it where the run lies in it), holds the
/// section or cuts it along its length: whether some point of the section's `hull` stands past it
/// by more than `eps`.  Where none does it is no cut (the fill on its floor); where one does the
/// piece is kept within it, a prism or a revolution clipped by a plane along it, exactly.
fn along_run(hull: &[V], (n, d): (V, f64), eps: f64) -> bool { hull.iter().any(|&q| dot(n, q) > d + eps) }

/// **The piece beyond a seam along a straight edge, the edge's whole length** (#962): the edge from
/// `p0` to `p1` between `faces`, face `c` a plane going on tangent across the seam through `s` (its
/// point in the section through `p0`, a line along the edge) into face `g`, which no line along the
/// edge turns about — a drafted side's plane going on into its rounded corner's cone, a foot standing
/// against the side beside the corner.  Every distance in the plane is the same along the edge, so
/// the section reaches across the seam all along it or nowhere: a round's ball touches `g` beyond
/// the seam's normal plane (the plane through the seam along the face's normal) the whole way, and
/// a flat changes face on that plane the whole way.  Refused by name where the section reaches
/// past `g`, or where `g`'s or the wall's surface is none the kernel offsets.  Its `run` (the planes
/// it is cut off by) the caller's.  The seam is the line through `s` along `way` — along the edge,
/// or aslant it (#994, `aslant_pieces`: then `p0` to `p1` is the stretch the piece runs, from where
/// the ball's touch crosses the seam at an end `seamed`, and the section is read at points of it
/// within the edge from `edge[0]` to `edge[1]`, where the next face is).
#[allow(clippy::too_many_arguments)]
fn along_beyond(body: &crate::solid::faces::BodyFaces, [p0, p1]: [V; 2], edge: [V; 2], faces: [usize; 2], c: usize,
    (s, way, g): (V, V, usize), within: bool, section: Section, seamed: [bool; 2]) -> Result<Beyond, String> {
    let along = normalised(sub(p1, p0)).ok_or("an edge of no length")?;
    let w = faces[1 - c];
    let Some((n, _)) = body.plane(faces[c]) else { return Err(spilled("from a face no plane")) };
    let wall = Surf::of(body, w)
        .ok_or_else(|| spilled("beside a face neither a plane, a cylinder, a cone nor a sphere"))?;
    let chain = Surf::of(body, g)
        .ok_or_else(|| spilled("into a face neither a plane, a cylinder, a cone nor a sphere"))?;
    // the seam's normal plane, negative on the edge's side
    let mid = scale(add(p0, p1), 0.5);
    let m = normalised(cross(way, n)).ok_or("a seam along the plane's normal")?;
    let m = if dot(m, sub(mid, s)) > 0.0 { scale(m, -1.0) } else { m };
    let split = Surf::Plane { origin: s, normal: m };
    // where the section reaches on the next face, along the edge (short of its ends, where the face
    // may end with it): on it, past the seam's normal plane
    // (over as much of the edge as the piece runs along, #994: from a crossing past its end, the
    // next face may end with it)
    let [t0, t1] = edge.map(|e| dot(sub(e, p0), along));
    let (from, to) = (t0.min(t1).max(0.0), t0.max(t1).min(distance(p0, p1)));
    for k in [0.25, 0.5, 0.75] {
        let q = add(p0, scale(along, from + k * (to - from)));
        let (reach, centre) = reach_on(body, g, w, q, along, n, within, section, c)
            .ok_or_else(|| spilled("beside a seam along a straight edge, where it touches the next face nowhere"))?;
        if onto(body, faces[c], g, reach)?.is_some() {
            return Err(spilled("past the face it goes on into beside a seam along a straight edge (a chain of more than \
                one face into a face no cylinder along it)"));
        }
        if split.side(centre.unwrap_or(reach)) <= 0.0 {
            return Err(unbuilt("a section beside a seam along its edge reaching the next face short of the seam's normal \
                plane"));
        }
    }
    let size = section.size();
    let at_seam = add(s, scale(way, dot(sub(mid, s), way)));
    let orient = |fi: usize, sf: &Surf, q: V| toward_region(body, fi, sf, q, within, size);
    Ok(Beyond {
        face: Surf::Plane { origin: p0, normal: n }, toward: toward(n, within), chain, wall,
        sides: [orient(g, &chain, at_seam), orient(w, &wall, mid)], seam: Seam::Line(s, way),
        section: match section {
            Section::Flat { la, lb } => Across::Flat { legs: if c == 0 { [la, lb] } else { [lb, la] }, split },
            Section::Round(radius) => Across::Round { radius },
        },
        ends: [p0, p1],
        run: None,
        seamed,
    })
}

/// A seam beside a straight edge as its run reads it (#422): along the edge (its point in the
/// section through the edge's start, and the face beyond), or a line crossing it aslant (#994: a
/// point of it, its unit direction, and the face beyond).
enum Beside {
    Along((V, usize)),
    Aslant((V, V, usize)),
}

/// **A seam crossing a straight edge's footprint aslant** (#994): where the pieces of the seam the
/// section reaches (`seams`, each with the face going on across it) are all one straight line, and
/// that line is not along the edge (unit `along`) — a drafted side's plane going on into its corner's
/// cone along a generator, and a foot's upright side meeting the side; a foot's leaning side
/// meeting an upright one going on into its corner's cylinder — that line (a point and its unit
/// direction) and the face beyond.  None where a piece is no line, or the line is along the edge
/// (`seam_along` reads those).
fn seam_aslant(body: &crate::solid::faces::BodyFaces, seams: &[(u32, usize)], along: V)
    -> Result<Option<(V, V, usize)>, String> {
    let Some(lines) = seams.iter().map(|&(s, _)| body.segment(s)).collect::<Option<Vec<[V; 2]>>>() else {
        return Ok(None);
    };
    let Some(&[q0, q1]) = lines.first() else { return Ok(None) };
    let d = normalised(sub(q1, q0)).ok_or("a seam of no length")?;
    if norm(cross(d, along)) <= 1e-9 {
        return Ok(None);
    }
    let size = 1.0 + norm(q0) + distance(q0, q1);
    if !lines.iter().flatten().all(|&q| norm(cross(sub(q, q0), d)) <= 1e-9 * size) {
        return Err(spilled("beside two seams"));
    }
    Ok(Some((q0, d, next_face(body, seams)?)))
}

/// **A round beside a seam crossing its edge's footprint aslant** (#994, fixture 126: a foot
/// standing against a drafted side a little short of where the side's plane goes on tangent into
/// its rounded corner's cone, along a generator the foot's edge is not along).  The edge from `p0`
/// to `p1` is between `faces`, face `c` a plane going on across the seam (`line`: a point, its unit
/// direction and the face beyond); `run` is the section against the two faces alone, swept along
/// the edge and carried past its ends (`halves` the planes it is kept within there).  In the plane
/// the ball's touch runs along the edge, a line, so it crosses the seam once, where it is linear in
/// the way along — at the section through the edge's point `x`: short of there the ball touches the
/// plane and the run is the region's, past it the ball touches the next face, and at `x` the
/// two are one — the touch on the seam, where the next face's normal is the plane's, so the
/// ball's centre runs along the edge there and the plane of its arc is the section square to it.
/// The run is cut off at that section, and the piece beyond (`along_beyond`, `brep::spill`'s
/// `round_beyond`) starts there (`Beyond::seamed`), carried past the other end and cut off by the
/// planes the run is kept within there, glued to the run on the section (`Piece::Along`).  The
/// crossing may lie past the end the run is carried from: the piece beyond starts there, cut off by
/// that end's planes, and there is no run; where it is further off than the edge's length and four
/// sections' widths (a seam all but along the edge) the piece beyond runs the edge's whole length,
/// carried past both ends.  Where it lies past where the run is carried to, the run is the whole of
/// it (`None`: the caller's piece).  Refused by name: a chamfer's flat, a face no plane, a crossing
/// past the end the touch goes on past, within the stretch carried past it, and a curved face the
/// piece beyond ends on.
#[allow(clippy::too_many_arguments)]
fn aslant_pieces(body: &crate::solid::faces::BodyFaces, [p0, p1]: [V; 2], faces: [usize; 2], c: usize,
    (q, d, g): (V, V, usize), within: bool, section: Section, run: &mut Run, halves: &[(V, f64)], ends: &Ends)
    -> Result<Option<Piece>, String> {
    let Section::Round(r) = section else {
        return Err(spilled("along a straight edge, a chamfer's flat beside a seam crossing its footprint aslant"));
    };
    let Some((n, _)) = body.plane(faces[c]) else { return Err(spilled("from a face no plane")) };
    let (along, length) = (run.along, distance(p0, p1));
    let centre = run.centre().ok_or_else(|| unbuilt("a round's section with no ball"))?;
    // the touch on the plane in the section through `p0`, and the seam's normal plane, the edge's
    // side of it negative
    let touch = sub(centre, scale(toward(n, within), r));
    let m = normalised(cross(d, n)).ok_or("a seam along the plane's normal")?;
    let mid = scale(add(p0, p1), 0.5);
    let m = if dot(m, sub(mid, q)) > 0.0 { scale(m, -1.0) } else { m };
    let slope = dot(m, along);
    let crossing = -dot(m, sub(touch, q)) / slope;
    let eps = 1e-9 * (1.0 + length + norm(p0));
    let [lo, hi] = run.span;
    // the way along the edge the touch goes on past the seam: the end there, and the run's
    let forward = slope > 0.0;
    let (short, past) = if forward { (lo, hi) } else { (hi, lo) };
    let ahead = |s: f64| if forward { s } else { -s };
    if ahead(crossing) >= ahead(past) - eps {
        return Ok(None);
    }
    let line = (q, d, g);
    let end = if forward { length } else { 0.0 };
    if ahead(crossing) >= ahead(end) - eps {
        return Err(spilled("where its ball's touch crosses a seam aslant its edge past the edge's end, in the stretch its \
            run is carried past it"));
    }
    let width = run.section_hull().into_iter().map(|p| distance(p, run.origin)).fold(r, f64::max);
    if ahead(crossing) < ahead(short) - length - 4.0 * width {
        // past the seam all along and far beyond the edge's end (a seam all but along the edge): the
        // piece beyond the edge's whole length, carried past both ends
        if !run.kept.is_empty() {
            return Err(spilled("beside a seam crossing its edge aslant, its run ending on a curved face"));
        }
        let mut beyond = along_beyond(body, [p0, p1], [p0, p1], faces, c, line, within, section, [false; 2])?;
        beyond.run = Some(halves.to_vec());
        return Ok(Some(Piece::Along { run: None, beyond }));
    }
    // the piece beyond from the crossing's section — within the edge, or past the end the run is
    // carried from (its stretch there cut off by the faces at that end) — to the end past which the
    // touch goes on, carried past it and cut off there
    let x = add(p0, scale(along, crossing));
    let far = if forward { p1 } else { p0 };
    let (_, bent) = ends_at(body, ends, far)?;
    let (_, bent_beyond) = ends_beyond(body, ends, far)?;
    let keeps = ahead(crossing) > ahead(short) + eps;
    if !bent.is_empty() || !bent_beyond.is_empty() || !keeps && !run.kept.is_empty() {
        return Err(spilled("beside a seam crossing its edge aslant, its run ending on a curved face"));
    }
    let (stretch, seamed) = if forward { ([x, p1], [true, false]) } else { ([p0, x], [false, true]) };
    let mut beyond = along_beyond(body, stretch, [p0, p1], faces, c, line, within, section, seamed)?;
    beyond.run = Some(halves.to_vec());
    if !keeps {
        return Ok(Some(Piece::Along { run: None, beyond }));
    }
    // the run from where it is carried to on the other side, to the crossing's section
    run.span[if forward { 1 } else { 0 }] = crossing;
    let hull = run.hull();
    run.within = halves.iter().copied()
        .filter(|&(n, dd)| hull.iter().any(|&p| dot(n, p) > dd + 1e-9 * (1.0 + length))).collect();
    run.flush = flush_only(body, ends, [p0, p1]);
    Ok(Some(Piece::Along { run: Some(run.clone()), beyond }))
}

/// Where the section beside the edge at its point `q` reaches on face `g` going on from the edge's
/// face `c` (`n` its plane's normal) across a seam along the edge, against the other face `w`, in
/// the plane through `q` square to `along`: a flat's leg on `g` from `w`'s surface; a round's
/// touch on `g`, with its ball's centre `r` from both surfaces — on the region's side of each
/// (`within`), by Newton's steps (none where they find none).
#[allow(clippy::too_many_arguments)]
fn reach_on(body: &crate::solid::faces::BodyFaces, g: usize, w: usize, q: V, along: V, n: V, within: bool,
    section: Section, c: usize) -> Option<(V, Option<V>)> {
    let want = if within { -1.0 } else { 1.0 };
    let (into_k, into_o) = (toward(n, within), toward(body.normal(w, q), within));
    let size = section.size();
    let settled = |at: &[(V, f64); 3]| at.iter().all(|(_, m)| m.abs() <= 1e-12 * (size + norm(q)));
    let square = (along, dot(along, q));
    match section {
        Section::Flat { la, lb } => {
            let l = if c == 0 { la } else { lb };
            let way = normalised(sub(into_o, scale(n, dot(into_o, n))))?;
            let start = add(q, scale(way, l / dot(way, into_o).max(0.1)));
            let leg = crate::roots::newton_onto::<3>(start, size, |x| {
                let (sg, gg) = body.side_grad(g, x);
                let (sw, gw) = body.side_grad(w, x);
                Some([(gg, sg), (gw, sw - want * l), (square.0, dot(square.0, x) - square.1)])
            }, settled)?;
            Some((leg, None))
        }
        Section::Round(r) => {
            let start = crate::brep::fillet::across(q, [into_k, into_o], r);
            let centre = crate::roots::newton_onto::<3>(start, size, |x| {
                let (sg, gg) = body.side_grad(g, x);
                let (sw, gw) = body.side_grad(w, x);
                Some([(gg, sg - want * r), (gw, sw - want * r), (square.0, dot(square.0, x) - square.1)])
            }, settled)?;
            let (sg, gg) = body.side_grad(g, centre);
            Some((sub(centre, scale(gg, sg)), Some(centre)))
        }
    }
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

/// **Where a circle's meeting comes back past an end** (#962): of `out`, how far a point of the
/// circle stands outside the faces a run ends on at a turn `s` on past one end, the first turn in
/// `(0, gap]` at which it is within them all again (`tol`), the gap itself where it is not before
/// (the crease's other end).  The meeting a ring's points are read against is the circle within
/// those faces, so halfway there is where the crease stops being the nearest of it: the set's
/// meridian at that end.  Sampled at a 720th of a turn, the crossing bisected; nothing where the
/// circle goes on within them at once (a face that does not cut it there).
fn resumed(out: impl Fn(f64) -> f64, gap: f64, tol: f64) -> f64 {
    let n = ((720.0 * gap / std::f64::consts::TAU).ceil() as usize).max(8);
    let step = gap / n as f64;
    let mut last = 0.0;
    for j in 1..=n {
        let s = j as f64 * step;
        if out(s) <= tol {
            if j == 1 && out(0.5 * step) <= tol {
                return 0.0;
            }
            let (mut a, mut b) = (last, s);
            for _ in 0..60 {
                let m = 0.5 * (a + b);
                if out(m) <= tol { b = m } else { a = m }
            }
            return b;
        }
        last = s;
    }
    gap
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

/// Of `a cos φ + b sin φ + c` over `φ >= 0`, where it first goes over nothing (`first_over`) and
/// where it next comes back to nothing after (its falling root, never where it is always over);
/// none where it never goes over.
fn over_from(a: f64, b: f64, c: f64) -> Option<[f64; 2]> {
    use std::f64::consts::TAU;
    let rise = first_over(a, b, c)?;
    let amplitude = a.dhypot(b);
    if amplitude <= c {
        return Some([rise, f64::INFINITY]);
    }
    let fall = (b.datan2(a) + (-c / amplitude).dacos()).rem_euclid(TAU);
    Some([rise, if fall < rise { fall + TAU } else { fall }])
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
