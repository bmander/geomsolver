//! **What a recognised region is built as** (#194): its faces read off the stratum (`BodyFaces`)
//! and a piece per edge between them — a section swept along a straight edge (`run_piece`), turned
//! round a circle (`ring_piece`), rolled along any other — with a ball's cell at each corner three
//! faces share (`corner_at`), each carried past its ends and kept within what it ends on.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::chain::Next;
use super::recognise::Faces;
use super::{Beyond, Corner, Kept, Piece, Ring, Rolled, Run, Section, Spill, Stroke, Surf, V};
use crate::space::{add, cross, distance, dot, norm, normalised, scale, sub};


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
pub(super) fn build(body: &crate::solid::faces::BodyFaces, f: &Faces, section: Section, spill: &Spill)
    -> Result<Vec<Piece>, String> {
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
        let ps = run(body, &groups[i], &groups[j], f.within, section, spill)?;
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
            bounds.extend(ends_at_all(body, &body.end_sides(&es, &[ids[i], ids[j]])));
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
fn run(body: &crate::solid::faces::BodyFaces, ga: &[usize], gb: &[usize], within: bool, section: Section, spill: &Spill)
    -> Result<Vec<Piece>, String> {
    let edges = body.edges_between(ga, gb);
    // an edge no line or circle about the faces' common axis carries: the ball rolled along the
    // whole run of them, traced
    // (a straight edge carried only between planes and cylinders along it: a cone's generator is
    // rolled along, its section square to it no circle)
    let carried = |e: u32| match body.segment(e) {
        Some([p0, p1]) => {
            let d = normalised(sub(p1, p0)).unwrap_or([0.0; 3]);
            body.faces_of(e).iter().all(|&f| body.plane(f).is_some()
                || body.cylinder(f).is_some_and(|(_, axis, _)| norm(cross(axis, d)) <= 1e-9))
        }
        None => body.circle(e).is_some_and(|(centre, axis, ..)| {
            body.faces_of(e).iter().all(|&f| body.coaxial(f, centre, axis))
        }),
    };
    if edges.iter().any(|&e| !carried(e)) {
        let spilt = [ga, gb].map(|g| g.iter().find_map(|f| spill.get(f).map(|seams| (*f, seams))));
        let traced = body.runs(&edges).into_iter().filter(|run| body.convex(run[0]) == Some(within));
        // a flat whose face goes on across a seam (#422): cut along the run short of the seam's
        // normal surface, the piece beyond it taking the rest
        if spilt.iter().any(Option::is_some) {
            let c = match spilt { [Some(_), Some(_)] => return Err(spilled("on both sides of its edge")), [Some(_), _] => 0, _ => 1 };
            let (p, seams) = spilt[c].expect("a face going on");
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
                    let &[f0, f1] = body.faces_of(e) else {
                        return Err(unbuilt("an edge not between two faces"));
                    };
                    let (fa, fb) = if ga.contains(&f0) { (f0, f1) } else { (f1, f0) };
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
    let ends = body.end_sides(&edges, &all);
    let mut pieces = Vec::new();
    for &edge in &edges {
        let &[f0, f1] = body.faces_of(edge) else {
            return Err(unbuilt("an edge not between two faces"));
        };
        let (fa, fb) = if ga.contains(&f0) { (f0, f1) } else { (f1, f0) };
        pieces.extend(piece(body, edge, fa, fb, within, section, &ends, spill)?);
    }
    Ok(pieces)
}

/// The piece along `edge`, where faces `fa` and `fb` meet — a straight one's section swept along
/// it (`run_piece`), a circle's turned about its axis (`ring_piece`) — kept within the half-spaces
/// of the run's `ends` standing at its own ends and reaching into it; `None` where it turns the
/// other way from the side the region asks (`within`, convex).
#[allow(clippy::too_many_arguments)]
fn piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool, section: Section,
    ends: &[(usize, f64)], spill: &Spill) -> Result<Vec<Piece>, String> {
    // the faces in the body's order, so two regions over one edge build one run
    let (fa, fb, section) = if fb < fa { (fb, fa, section.swapped()) } else { (fa, fb, section) };
    // the gate: an edge turning the other way, or none (faces meeting tangent), holds nowhere
    if body.convex(edge) != Some(within) {
        return Ok(Vec::new());
    }
    // (`run` rolls a circle whose faces are not all about its axis)
    if body.circle(edge).is_some() {
        return ring_piece(body, edge, fa, fb, within, section, ends, spill);
    }
    run_piece(body, edge, fa, fb, within, section, ends, spill).map(|p| vec![p])
}

/// Round a circle `edge` where a plane, a cylinder, a cone or a sphere meet about one axis: the
/// section in the meridian through its start, turned through the circle's angle — a whole turn,
/// or a stretch ending where a face goes on from one of these tangent (an obround's flat side),
/// whose own run goes on from the same section.
#[allow(clippy::too_many_arguments)]
fn ring_piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool,
    section: Section, ends: &[(usize, f64)], spill: &Spill) -> Result<Vec<Piece>, String> {
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
        Some((c, Round::Askew(seam, g))) => (None, Some((c, seam, g))),
        Some((c, Round::About(o, r, g))) => (Some((c, (o, r, g))), None),
        None => (None, None),
    };
    // the section against the chain at the turn `t`, its seams each a circle (centre, radius) and
    // the face going on across it; and its footprint on the chain
    let against = |t: f64, c: usize, seams: &[(V, f64, usize)]| {
        let p = add(centre, scale(radial_at(t), radius));
        let next: Vec<Next> = seams.iter().map(|&(o, r, g)| {
            let s = add(o, scale(radial_at(t), r));
            Next { seam: s, trace: trace(g, s) }
        }).collect();
        super::chain::cut_chain(p, cross(axis, radial_at(t)), [trace(fa, p), trace(fb, p)], (c, &next), within, section)
    };
    // the chain from the edge's face on: each face the footprint lands past going on into the
    // next across another seam about the axis
    let chain = match chain {
        None => None,
        Some((c, first)) => {
            let mut seams = vec![first];
            let mut prev = [fa, fb][c];
            loop {
                let (_, foot) = against(t0, c, &seams)?;
                let g = seams.last().expect("a seam").2;
                let Some(h) = onto(body, prev, g, foot)? else { break };
                if seams.len() >= CHAIN {
                    return Err(spilled("past the faces it goes on into (a chain of more than four)"));
                }
                let across: Vec<(u32, usize)> = body.edges_between(&[g], &[h]).into_iter().map(|e| (e, h)).collect();
                match seam_round(body, &across, centre, axis)? {
                    Round::About(o, r, h) => seams.push((o, r, h)),
                    Round::Askew(..) => return Err(spilled("round a circle beside a chain whose further seam is not about its axis")),
                }
                prev = g;
            }
            Some((c, seams))
        }
    };
    let section_at = |t: f64| match &chain {
        None => {
            let p = add(centre, scale(radial_at(t), radius));
            cut(p, cross(axis, radial_at(t)), [trace(fa, p), trace(fb, p)], within, section)
        }
        Some((c, seams)) => against(t, *c, seams).map(|(run, _)| run),
    };
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
        }
        if to - from >= std::f64::consts::TAU {
            return Err(unbuilt("a stretch of a circle carried on round its whole turn"));
        }
    }
    let mut start = section_at(from)?;
    start.within = halves;
    start.kept = kepts;
    let mut ring = Ring { section: start, axis_origin: centre, axis, angle: to - from, beyond: None };
    if let Some((c, seam, g)) = askew {
        ring.beyond = Some(beyond(body, edge, [fa, fb], c, seam, g, within, section, &mut ring)?);
    }
    Ok(vec![Piece::Ring(ring)])
}

/// The ring round circle `edge` (between `faces`, face `c` a plane going on tangent across the
/// circle `seam` into a torus, face `g`, about the plane's normal but not about the ring's axis)
/// kept short of the seam's normal surface — the cylinder through the seam along the plane's
/// normal — and the piece beyond it (#422): a chamfer's flat against the torus there; or a round's
/// ring a stretch between where its ball touches the seam, and the ball rolled against the torus
/// between.  Refused by name at a stretch of a circle, an edge or a wall crossing that surface.
#[allow(clippy::too_many_arguments)]
fn beyond(body: &crate::solid::faces::BodyFaces, edge: u32, faces: [usize; 2], c: usize, (so, sn, sr): (V, V, f64),
    g: usize, within: bool, section: Section, ring: &mut Ring) -> Result<Beyond, String> {
    let (la, lb, r) = match section { Section::Flat { la, lb } => (la, lb, 0.0), Section::Round(r) => (r, r, r) };
    if !body.closed(edge) {
        return Err(spilled("round a stretch of a circle off the seam's axis"));
    }
    let (p, w) = (faces[c], faces[1 - c]);
    let Some((n, _)) = body.plane(p) else { return Err(spilled("from a face no plane, round a circle off the seam's axis")) };
    let chain = match Surf::of(body, g) {
        Some(t @ Surf::Torus { axis, .. }) if norm(cross(axis, n)) <= 1e-9 && norm(cross(sn, n)) <= 1e-9 => t,
        _ => return Err(spilled("into a face no torus about the plane's normal")),
    };
    let wall = Surf::of(body, w).ok_or_else(|| spilled("beside a face neither a plane, a cylinder, a cone nor a sphere"))?;
    let (centre, axis, radius) = (ring.axis_origin, ring.axis, body.circle(edge).expect("a circle's edge").2);
    let size = la.max(lb);
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
    let orient = |fi: usize, s: &Surf, q: V| toward_region(body, fi, s, q, within, size);
    let split = Surf::Cylinder { origin: so, axis: sn, radius: sr };
    let inside = split.side(centre) < 0.0;
    let legs = if r > 0.0 { [0.0; 2] } else if c == 0 { [la, lb] } else { [lb, la] };
    let beyond = Beyond {
        along: super::Along::Ring { axis_origin: centre, axis, crease: (centre, radius) }, toward: toward(n, within), chain,
        wall, split, sides: [orient(g, &chain, ends[0]), orient(w, &wall, at), if inside { 1.0 } else { -1.0 }],
        seam: super::Seam::Circle(so, sn, sr), legs, radius: r, ends,
    };
    if r == 0.0 {
        ring.section.kept.push(Kept { face: split, inside });
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
/// more than twice, crosses the seam's normal surface.
#[allow(clippy::too_many_arguments)]
fn traced_beyond(body: &crate::solid::faces::BodyFaces, run: &[u32], faces: [&[usize]; 2], c: usize, p: usize,
    seams: &[(u32, usize)], within: bool, section: Section) -> Result<Piece, String> {
    let (la, lb, r) = match section { Section::Flat { la, lb } => (la, lb, 0.0), Section::Round(r) => (r, r, r) };
    let Some((n, _)) = body.plane(p) else { return Err(spilled("from a face no plane, along a traced edge")) };
    let g = seams[0].1;
    if seams.iter().any(|&(_, h)| h != g) {
        return Err(spilled("into more than one face"));
    }
    let chain = Surf::of(body, g).ok_or_else(|| spilled("into a face neither a cylinder nor a torus"))?;
    // the seam and its normal surface, the run's side of it within
    let (seam, split) = match (body.segment(seams[0].0), body.circle(seams[0].0), chain) {
        (Some([q0, q1]), _, Surf::Cylinder { axis, .. }) if norm(cross(axis, sub(q1, q0))) <= 1e-9 * distance(q0, q1) => {
            let d = normalised(sub(q1, q0)).ok_or("a seam of no length")?;
            let m = normalised(cross(d, n)).ok_or("a seam along the plane's normal")?;
            (super::Seam::Line(q0, d), Surf::Plane { origin: q0, normal: m })
        }
        (_, Some((o, axis, r, _)), Surf::Torus { axis: ta, .. }) if norm(cross(axis, n)) <= 1e-9 && norm(cross(ta, n)) <= 1e-9 => {
            (super::Seam::Circle(o, axis, r), Surf::Cylinder { origin: o, axis, radius: r })
        }
        _ => return Err(spilled("along a traced edge, into a face that is no cylinder along a straight seam nor torus about \
            the plane's normal")),
    };
    let at = body.middle(run[0]);
    let flip = if split.side(at) > 0.0 { -1.0 } else { 1.0 };
    let split = match split {
        Surf::Plane { origin, normal } => Surf::Plane { origin, normal: scale(normal, flip) },
        s => s,
    };
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
    let wall = Surf::of(body, w).ok_or_else(|| spilled("beside a face neither a plane, a cylinder, a cone nor a sphere"))?;
    let size = la.max(lb);
    let orient = |fi: usize, s: &Surf, q: V| toward_region(body, fi, s, q, within, size);
    let legs = if r > 0.0 { [0.0; 2] } else if c == 0 { [la, lb] } else { [lb, la] };
    let beyond = Beyond {
        along: super::Along::Meeting { face: Surf::Plane { origin: at, normal: n } }, toward: toward(n, within), chain, wall,
        split, sides: [orient(g, &chain, q0), orient(w, &wall, at), 1.0], seam, legs, radius: r, ends: [q0, q1],
    };
    // the flat along the run, told its legs apart by a point beside the first group's face, from
    // its point furthest from the seam's normal surface (a closed run's flat is seamed where it
    // starts: there, away from the piece beyond, which it is glued to whole)
    let (e, at) = run.iter().flat_map(|&e| (0..64).map(move |k| (e, body.along(e, k as f64 / 64.0))))
        .min_by(|x, y| split.side(x.1).total_cmp(&split.side(y.1))).expect("a run of edges");
    let ends_of: Vec<V> = body.run_ends(run).into_iter().map(|(v, ..)| v).collect();
    if r > 0.0 {
        // the ball's centres where it touches the seam, and the planes of its arc there, square to
        // its centre's path (where the plane's offset meets the wall's), each's normal into the
        // stretch between
        let centres = [q0, q1].map(|q| add(q, scale(toward(n, within), r)));
        let wedge: Vec<(V, V)> = (0..2).map(|k| {
            let d = normalised(cross(n, wall.gradient(centres[k]))).ok_or("a wall along the plane's normal")?;
            Ok((centres[k], if dot(d, sub(centres[1 - k], centres[k])) < 0.0 { scale(d, -1.0) } else { d }))
        }).collect::<Result<_, String>>()?;
        return Ok(Piece::Rolled(Rolled { at, r, flat: None, ends: ends_of, cuts: Vec::new(), kept: Vec::new(), wedge,
            beyond: Some(beyond) }));
    }
    let fa = body.faces_of(e).iter().copied().find(|f| faces[0].contains(f)).ok_or("an edge not between two faces")?;
    let fb = body.faces_of(e).iter().copied().find(|f| faces[1].contains(f)).ok_or("an edge not between two faces")?;
    let na = body.normal(fa, at);
    let t = toward(body.normal(fb, at), within);
    let wa = normalised(sub(t, scale(na, dot(t, na)))).ok_or("faces meeting tangent")?;
    Ok(Piece::Rolled(Rolled { at, r: 0.0, flat: Some(([la, lb], add(at, scale(wa, 0.5 * la)))), ends: ends_of,
        cuts: Vec::new(), kept: vec![Kept { face: split, inside: true }], wedge: Vec::new(), beyond: Some(beyond) }))
}

/// Along a straight `edge`: the section beside it swept along it, carried on past each end until
/// every face there cuts it off.
#[allow(clippy::too_many_arguments)]
fn run_piece(body: &crate::solid::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool,
    section: Section, ends: &[(usize, f64)], spill: &Spill) -> Result<Piece, String> {
    let Some([p0, p1]) = body.segment(edge) else { return Err(unbuilt("a curved edge")) };
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
    let mut run = match chain {
        None => cut(p0, along, traces, within, section)?,
        Some((c, first)) => {
            // the chain from the edge's face on: each face the footprint lands past going on into
            // the next across another seam along the edge
            let mut seams = vec![first];
            let mut prev = [fa, fb][c];
            loop {
                let next = seams.iter().map(|&(s, g)| Ok(Next { seam: s,
                    trace: trace(g, s).map_err(|_| spilled("into a face not turned about a line along its edge"))? }))
                    .collect::<Result<Vec<Next>, String>>()?;
                let (run, foot) = super::chain::cut_chain(p0, along, traces, (c, &next), within, section)?;
                let g = seams.last().expect("a seam").1;
                let Some(h) = onto(body, prev, g, foot)? else { break run };
                if seams.len() >= CHAIN {
                    return Err(spilled("past the faces it goes on into (a chain of more than four)"));
                }
                let across: Vec<(u32, usize)> = body.edges_between(&[g], &[h]).into_iter().map(|e| (e, h)).collect();
                seams.push(seam_along(body, &across, p0, along, length)?);
                prev = g;
            }
        }
    };
    let outline = run.section_hull();
    run.span = [0.0, length];
    let mut halves = planes0;
    for h in planes1 {
        if !halves.contains(&h) {
            halves.push(h);
        }
    }
    for &(n, d) in &halves {
        let slope = dot(n, along);
        if slope.abs() < 1e-6 {
            return Err(unbuilt("a run carried on along the face it ends on"));
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
                    return Err(unbuilt("a run its curved end does not cut off"));
                }
                deepest
            };
            if forward { run.span[1] = run.span[1].max(to) } else { run.span[0] = run.span[0].min(-to) }
            if !run.kept.contains(&kept) {
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

/// Where the section's footprint on the last face of a chain (`foot`, where its leg or its ball's
/// touch is) lands, face `g` going on from `prev`: on it (none), or past a seam where it goes on
/// tangent into another (that face), the chain going on; refused past any other edge of it.
fn onto(body: &crate::solid::faces::BodyFaces, prev: usize, g: usize, foot: V) -> Result<Option<usize>, String> {
    let Some(e) = body.off_face(g, foot) else { return Ok(None) };
    match body.faces_of(e).iter().copied().find(|&h| h != g) {
        Some(h) if h != prev && body.seam(e) => Ok(Some(h)),
        _ => Err(spilled("past the face it goes on into, across an edge no seam")),
    }
}

/// The seam along a straight edge (from `p0` along the unit `along`, `length` long) where a face
/// goes on into the next its section reaches (`seams`, its pieces, each with that face): its point
/// in the section through `p0` — every piece of it a line along the edge, one line, running the
/// whole edge — and that face.
fn seam_along(body: &crate::solid::faces::BodyFaces, seams: &[(u32, usize)], p0: V, along: V, length: f64)
    -> Result<(V, usize), String> {
    let g = seams.first().ok_or_else(|| spilled("into a face it meets in no edge"))?.1;
    if seams.iter().any(|&(_, h)| h != g) {
        return Err(spilled("into more than one face"));
    }
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
    let g = seams.first().ok_or_else(|| spilled("into a face it meets in no edge"))?.1;
    if seams.iter().any(|&(_, h)| h != g) {
        return Err(spilled("into more than one face"));
    }
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
    let (o, n, r) = circle.expect("a seam");
    Ok(if about { Round::About(o, r, g) } else { Round::Askew((o, n, r), g) })
}

/// The seam round a circle a section reaches across: about the circle's own axis (its centre and
/// radius, and the face beyond), or not (its centre, unit normal and radius, and the face beyond).
enum Round {
    About(V, f64, usize),
    Askew((V, V, f64), usize),
}

/// The faces of a run's `ends` (`end_sides`' faces and the sides it runs on) standing at `p`, one
/// end of an edge of it: each plane's half-space `n·x <= d` it is kept within, and each cylinder,
/// cone or ball it is kept within (where that side holds its axis, its centre) or outside of.
fn ends_at(body: &crate::solid::faces::BodyFaces, ends: &[(usize, f64)], p: V) -> Result<(Vec<(V, f64)>, Vec<Kept>), String> {
    let (mut planes, mut bent) = (Vec::new(), Vec::new());
    for &(c, sign) in ends.iter().filter(|&&(c, _)| body.side(c, p).abs() <= 1e-7 * (1.0 + norm(p))) {
        match Surf::of(body, c) {
            Some(Surf::Plane { origin, normal }) => planes.push((scale(normal, sign), sign * dot(normal, origin))),
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
/// starts over); none where it never does.
fn first_over(a: f64, b: f64, c: f64) -> Option<f64> {
    if a + c > 0.0 {
        return Some(0.0);
    }
    let amplitude = a.dhypot(b);
    if amplitude <= -c {
        return None;
    }
    let (phase, spread) = (b.datan2(a), (-c / amplitude).dacos());
    Some([phase - spread, phase + spread].map(|x| x.rem_euclid(std::f64::consts::TAU)).into_iter().fold(f64::INFINITY, f64::min))
}

/// The half-spaces `n·x <= d` of the planes among a run's `ends` (`end_sides`' faces and the
/// sides it runs on).
fn ends_at_all(body: &crate::solid::faces::BodyFaces, ends: &[(usize, f64)]) -> Vec<(V, f64)> {
    ends.iter().filter_map(|&(c, sign)| body.plane(c).map(|(n, o)| (scale(n, sign), sign * dot(n, o)))).collect()
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

    /// From `from` to `to` along the face (both on it): a line, or the shorter arc of the circle.
    fn stroke(self, from: V, to: V, along: V) -> Stroke {
        match self {
            Trace::Line(_) => Stroke::Line { from, to },
            Trace::Circle { centre, radius, .. } => arc(centre, radius, from, to, along),
        }
    }
}

/// The shorter arc about `centre` from `from` to `to`, turning about the unit `along`.
fn arc(centre: V, radius: f64, from: V, to: V, along: V) -> Stroke {
    let x = scale(sub(from, centre), 1.0 / radius);
    let to = sub(to, centre);
    let phi = dot(to, cross(along, x)).datan2(dot(to, x));
    Stroke::Arc { centre, radius, x, angles: if phi > 0.0 { [0.0, phi] } else { [phi, 0.0] } }
}

/// The section's corner as `brep::corners` reads a meridian's: in the plane square to `along` at
/// `p0` (coordinates along `x` and `along × x`, so a turn to the left is one about `along`), face
/// `a`'s trace a step arriving at the corner and `b`'s one leaving it, each `reach` long, and the
/// side the region lies on (+1 the left of travel). A round or a flat there is then what it is in a
/// meridian, by the same rule (`corners::ball`, `corners::sum`): between lines and circles, a
/// circle's flat the conic its sum of distances makes.
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

    /// The ball of radius `r` in the corner: its centre, and where it touches `a` and `b`.
    fn ball(&self, r: f64) -> Result<(V, V, V), String> {
        let (ta, tb, arc) = crate::brep::corners::ball(&self.steps[0], &self.steps[1], self.side, r)
            .map_err(|e| unbuilt(&e))?;
        let crate::brep::planar::Seg::Arc { c, .. } = arc else { unreachable!("a ball's arc") };
        Ok((self.lift(c), self.lift(self.steps[0].point(ta)), self.lift(self.steps[1].point(tb))))
    }

    /// The flat with legs `la` on `a` and `lb` on `b`, each from the other face's surface: its ends
    /// on `a` and `b` and the stroke between them.
    fn flat(&self, [la, lb]: [f64; 2]) -> Result<(V, V, Stroke), String> {
        use crate::brep::planar::Seg;
        // (a Cartesian oval is no construction left to make: no rational curve holds it)
        let (ta, tb, cut) = crate::brep::corners::sum(&self.steps[0], &self.steps[1], self.side, [la, lb])
            .map_err(|e| if e.contains("no rational curve") { e } else { unbuilt(&e) })?;
        let (qa, qb) = (self.lift(self.steps[0].point(ta)), self.lift(self.steps[1].point(tb)));
        Ok((qa, qb, match cut {
            Seg::Conic { m, w, .. } => Stroke::Conic { a: qa, m: self.lift(m), b: qb, w },
            _ => Stroke::Line { from: qa, to: qb },
        }))
    }
}

/// The section beside an edge at `p0`, in the plane square to the unit `along` there (its stretch
/// nothing): between faces seen there as `traces` (normals away from the material, each square to
/// `along`), on the region's side of both (`within`: the material's).
fn cut(p0: V, along: V, traces: [Trace; 2], within: bool, section: Section) -> Result<Run, String> {
    let (ua, ub) = (toward(traces[0].normal(), within), toward(traces[1].normal(), within));
    if 1.0 + dot(ua, ub) <= 1e-9 {
        return Err(unbuilt("faces folded back on one another"));
    }
    let size = section.size();
    let corner = Corner2::new(p0, along, traces, [ua, ub], 8.0 * size)?;
    let (ta, middle, tb) = match section {
        // the ball's centre `r` from both faces, and where it touches each
        Section::Round(r) => {
            let (o, ta, tb) = corner.ball(r)?;
            (ta, (arc(o, r, ta, tb, along), "round"), tb)
        }
        // along each face away from the edge, to where it stands its leg from the other face
        Section::Flat { la, lb } => {
            let (qa, qb, flat) = corner.flat([la, lb])?;
            (qa, (flat, "flat"), qb)
        }
    };
    let strokes = vec![(traces[0].stroke(p0, ta, along), "a"), middle, (traces[1].stroke(tb, p0, along), "b")];
    Ok(Run { origin: p0, along, span: [0.0; 2], strokes, within: Vec::new(), kept: Vec::new() })
}
