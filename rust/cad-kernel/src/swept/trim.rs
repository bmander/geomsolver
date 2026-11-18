//! **Trimming by certified membership, face by face** (#880 E1; docs/swept-solid-design.md §12):
//! the closed swept solid built from its candidates with no cell of their arrangement built, and
//! only the meetings that may bound it sought.
//!
//! The candidates are given (entered by hand, in E1): solids or sheets whose faces hold the swept
//! boundary — the tool at its end poses, the sheets its contacts and its edges sweep. Then:
//!
//! 1. **Zones** (P3). Each candidate is meshed, and each triangle — a ball about its middle holding
//!    it and the mesh's sag — is read against the sweep's depth `m(x)`, the least signed distance
//!    from the tool of `x` carried back over the roll (`member::Swept::beyond`): **interior** where
//!    `m < −r` at its middle, **exterior** where `m > r`, each proved for every point of the ball
//!    since `m` is 1-Lipschitz; else halved, down to the trim's `resolution`, and what is left
//!    **near** the boundary.
//! 2. **Pairs** (P4). Two faces of two candidates are met only where a near triangle of each stand
//!    within reach of one another: a meeting of two faces lying wholly in the sweep's interior (or
//!    outside it) bounds nothing and is never sought. Read through a grid of the resolution's size.
//! 3. **The trim** (`boolean::trimmed`): each face split by the faces it is paired with alone, each
//!    piece kept where membership a hair to one side of it differs from a hair to the other — turned
//!    so the member side is behind it — and the kept pieces assembled on their shared edges.
//! 4. **Judged** (#777): `Brep::check`, then `closed::judge`, before anything is returned.
//!
//! What it measures beside the arrangement (`Report`): the pairs of faces whose boxes meet, those
//! met, the meetings of surfaces sought (`util::meter::Meter::meetings`), and the zones. Its work is
//! counted against `closed::BUDGET`, as a closed build's is. `SOLVENT_TRIM_DEBUG` narrates the pairs
//! met, each piece's reading and the result's open edges; `SOLVENT_TRIM_EVERY` meets every pair, to
//! tell a fault of the pairing from one of the assembly.
//!
//! Candidates are entered by hand in E1 (`planes_past`, `screwed_segment`, `sheet_of`,
//! `contour_cylinder`, `contour_arc`); #880 T1 makes them from any face under any trajectory.
use crate::boolean::Given;
use crate::tolerant::ON_CURVE;
use crate::topo::Brep;
use crate::trajectory::Trajectory;
use crate::util::space::{add,distance,scale,sub};
use super::closed::Unbuilt;
use super::member::{Solid,Swept};
use std::collections::{BTreeMap,BTreeSet};

type V = [f64;3];

/// What a trim did, beside what an arrangement of the same candidates does.
#[derive(Clone,Debug,Default)]
pub struct Report {
    /// The candidates' faces, all together.
    pub faces: usize,
    /// Two faces of two candidates whose boxes meet: what an arrangement meets.
    pub boxed: usize,
    /// Of those, the pairs met: a near triangle of each within reach of the other's.
    pub paired: usize,
    /// Triangles proved interior, exterior, and left near the boundary.
    pub zones: [usize;3],
    /// The meetings of two surfaces sought, found in closed form, traced and given (`Meter::meetings`).
    pub meetings: [u64;3],
    /// Membership's steps (the zones', the pieces' sides' and the judge's).
    pub membership: u64,
    /// Pieces kept, of the pieces read.
    pub pieces: [usize;2],
}

/// How a trim reads its candidates.
#[derive(Clone,Copy,Debug)]
pub struct Trim {
    /// The least a near triangle is halved to (its ball's radius), in the model's units: the
    /// coarser, the fewer membership reads and the more pairs met.
    pub resolution: f64,
    /// How far to either side of a piece its membership is read, in the model's units: at least four
    /// times the membership mesh's sag, within which a side does not settle.
    pub hair: f64,
    /// **The box the sweep is wanted within** (#880 T1c), where it is cut from a blank the box holds:
    /// the trim builds the sweep's part inside it — the candidates' pieces outside it dropped, and
    /// met only where they come near it, the box's own faces kept where the sweep holds them — the
    /// same cut from the blank, since a blank inside the box loses only what of the sweep lies in
    /// it. What the sweep does beyond — a fold's cusp where the tool stands at an end of a roll run
    /// past the blank (the twist drill's steep flute wheel) — is not asked.
    pub reach: Option<(V,V)>,
}

/// **The closed swept solid of `tool` under `path` over `roll`, trimmed from `candidates`** (E1):
/// each candidate a solid or a sheet holding some of the swept boundary, `given` the meetings
/// known by construction between them. Membership reads the tool meshed within `bar`; the result
/// is built to `tol`, checked, and judged by the sweep's definition before it is returned.
#[allow(clippy::too_many_arguments)]
pub fn trimmed(tool: &Brep,path: &Trajectory,roll: [f64;2],candidates: &[Brep],given: &[Given],bar: f64,tol: f64,trim: Trim)
    -> Result<(Brep,Report),Unbuilt> {
    use crate::util::meter;
    let m = meter::Meter::new(super::closed::BUDGET);
    let clock = crate::util::clock::Instant::now();
    let out = meter::within(&m,|| self::trim(tool,path,roll,candidates,given,bar,tol,trim));
    if meter::profiling() {
        eprintln!("{}  took {:?}: {}",m.report(),clock.elapsed(),match &out { Ok((b,_)) => format!("trimmed, {} faces",b.faces.len()),Err(e) => format!("{e:?}") });
    }
    if let Some(o) = m.over() { return Err(meter::Stopped(o).into()) }
    let (b,mut report) = out?;
    report.meetings = m.meetings();
    report.membership = m.count(meter::Stage::Membership);
    Ok((b,report))
}

/// **A piece missing beside the boundary left open** (#942 F2): where the trimmed solid does not
/// close, each edge it uses once is read across on every candidate face through its middle — a
/// point `ACROSS` hairs from it either way, on that face.  Where the face kept beside the edge
/// holds that point and membership proves it clear of the boundary, the piece kept is mixed; where
/// a face not kept there holds it and membership reads it as the boundary, its piece was left out.
/// Either way a meeting the trim was not given parts what was read from what was not.  Said with
/// the edge and the point; none where neither is found (the boundary open for some other reason).
/// `read` is the trim's own reading of a point of a face: `Some` only where it settles, and a point
/// off the boundary only where it is proved clear.
fn missing_across(out: &Brep,candidates: &[&Brep],read: &dyn Fn(&crate::topo::Face,V) -> Option<Option<bool>>,
    hair: f64,tol: f64) -> Option<String> {
    use crate::util::space::{cross,normalised};
    let uses = out.uses();
    let located: Vec<crate::query::Located> = candidates.iter().map(|c| crate::query::Located::new(c,tol)).collect();
    let kept = crate::query::Located::new(out,tol);
    for (e,u) in uses.iter().enumerate().filter(|(_,u)| u.len() == 1) {
        let edge = &out.edges[e];
        let crate::topo::EdgeCurve::Curve(c) = &edge.curve else { continue };
        let t = 0.5*(edge.t[0]+edge.t[1]);
        let (m,along) = (c.point(t),c.tangent(t));
        for (k,cand) in candidates.iter().enumerate() {
            for (i,f) in cand.faces.iter().enumerate() {
                if f.surface.implicit(m).abs() > ON_CURVE*tol.max(edge.tol) { continue }
                let Some(across) = normalised(cross(f.surface.gradient(m),along)) else { continue };
                for side in [-1.,1.] {
                    let q = f.surface.point(f.surface.inverse(add(m,scale(across,side*ACROSS*hair))));
                    if !located[k].face_holds(i,q) { continue }
                    let on_kept = kept.face_holds(u[0].0,q);
                    match read(f,q) {
                        Some(None) if on_kept =>
                            return Some(format!("the boundary is open along a {} edge through {m:?}: the {} piece kept beside it \
                                (candidate and face ({k}, {i})) is not the boundary at {q:?}, a meeting the trim was not given \
                                parting it from where it was read",c.kind(),f.surface.kind())),
                        Some(Some(_)) if !on_kept =>
                            return Some(format!("the boundary is open along a {} edge through {m:?}: a piece of a {} (candidate and \
                                face ({k}, {i})) is the boundary beside it at {q:?} and was not kept, a meeting the trim was not \
                                given parting it from where its piece was read",c.kind(),f.surface.kind())),
                        _ => {}
                    }
                }
            }
        }
    }
    None
}

/// How many hairs across an edge the boundary leaves open a face beside it is read (#942 F2).
const ACROSS: f64 = 8.;

/// How many more of a piece's points that settle hold it to what its first said (#942 F2).
const CONFIRMED: usize = 7;

/// How many hairs clear of the boundary, proved, a point read off it stands to witness a piece mixed
/// (#942 F2).
const CLEAR: f64 = 4.;

/// A ball holding part of a candidate's face: its middle and radius.
#[derive(Clone,Copy)]
struct Ball { c: V,r: f64 }

/// `trimmed` under the meter its caller installed (`closed::trimmed`, `closed`'s candidates).
#[allow(clippy::too_many_arguments)]
pub(crate) fn trim(tool: &Brep,path: &Trajectory,roll: [f64;2],candidates: &[Brep],given: &[Given],bar: f64,tol: f64,trim: Trim)
    -> Result<(Brep,Report),Unbuilt> {
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll};
    if crate::timing() { eprintln!("trim: the tool meshed for membership: {} triangles, sag {:.2e}",solid.triangles(),solid.sag) }
    let mut report = Report {faces:candidates.iter().map(|c| c.faces.len()).sum(),..Report::default()};
    // (the reach: a ball reaches it where it comes within its radius of the box; a point is past it
    // farther than the tolerance outside)
    let reaches = |b: ([f64;3],[f64;3]),r: f64| trim.reach.is_none_or(|reach| crate::boolean::overlap(&b,&reach,r));
    let held_box = trim.reach.map(|(lo,hi)| crate::build::block(lo,hi)).transpose().map_err(Unbuilt::Failed)?;
    let candidates: Vec<&Brep> = candidates.iter().chain(held_box.as_ref()).collect();
    let boxed = held_box.is_some().then(|| candidates.len()-1);
    // 1. each candidate's faces zoned: the balls near the boundary, by (candidate, face)
    let clock = crate::util::clock::Instant::now();
    let mut near: Vec<(usize,usize,Ball)> = Vec::new();
    for (k,cand) in candidates.iter().enumerate() {
        // (the box is no candidate to zone; a candidate clear of it bounds nothing within it)
        if Some(k) == boxed || !reaches(cand.bounds(),0.) { continue }
        let coarse = (cand.size()/8.).max(4.*trim.resolution);
        // meshed until its sag is under a quarter of the resolution, so a ball can shrink to it
        // (the triangles are halved flat, so their sag is the mesh's)
        let (mut angular,mut coarse) = (0.5,coarse);
        let mesh = loop {
            let mesh = crate::mesh::mesh(cand,coarse,angular)?;
            if mesh.sag <= 0.25*trim.resolution || coarse < trim.resolution { break mesh }
            angular *= 0.5;
            coarse *= 0.5;
        };
        let sag = mesh.sag;
        if crate::timing() { let (lo,hi) = cand.bounds(); eprintln!("trim: candidate {k}: {} faces ({}), from {lo:.1?} to {hi:.1?}, {} triangles, sag {sag:.3}",
            cand.faces.len(),cand.faces.iter().map(|f| f.surface.kind()).collect::<Vec<_>>().join(" "),mesh.tris.len()) }
        let tris: Vec<(usize,[V;3])> = mesh.tris.iter().zip(&mesh.of).map(|(t,&f)| (f as usize,t.map(|i| mesh.pts[i as usize]))).collect();
        let read = crate::util::par::map(&tris,|&(_,t)| zoned(&swept,t,sag,trim.resolution));
        let mut here = [0;3];
        for ((f,_),r) in tris.iter().zip(read) {
            let (balls,z) = r?;
            for k2 in 0..3 { report.zones[k2] += z[k2]; here[k2] += z[k2] }
            near.extend(balls.into_iter().filter(|b| reaches((b.c,b.c),b.r)).map(|b| (k,*f,b)));
        }
        if crate::timing() {
            let reads = crate::util::meter::current().map_or(0,|m| m.count(crate::util::meter::Stage::Membership));
            eprintln!("trim: candidate {k} zoned {here:?}, membership {reads} so far, in {:?}",clock.elapsed());
        }
    }
    if crate::timing() { eprintln!("trim: zoned in {:?}: {:?}",clock.elapsed(),report.zones) }
    // 2. the pairs of faces met: near balls of two candidates within reach, through a grid
    let cell = 2.*trim.resolution;
    let mut grid: BTreeMap<[i64;3],Vec<usize>> = BTreeMap::new();
    for (i,&(_,_,b)) in near.iter().enumerate() {
        let lo = sub(b.c,[b.r;3]).map(|x| (x/cell).floor() as i64);
        let hi = add(b.c,[b.r;3]).map(|x| (x/cell).floor() as i64);
        for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] { for z in lo[2]..=hi[2] { grid.entry([x,y,z]).or_default().push(i) } } }
    }
    let mut pairs: BTreeSet<([usize;2],[usize;2])> = BTreeSet::new();
    let pad = 4.*tol;
    for list in grid.values() {
        // the faces in the cell, each once: one ball standing for its face's
        let mut by_face: BTreeMap<(usize,usize),Vec<Ball>> = BTreeMap::new();
        for &i in list { let (k,f,b) = near[i]; by_face.entry((k,f)).or_default().push(b) }
        let faces: Vec<(&(usize,usize),&Vec<Ball>)> = by_face.iter().collect();
        for (x,&(a,ba)) in faces.iter().enumerate() { for &(b,bb) in &faces[x+1..] {
            if a.0 == b.0 || pairs.contains(&([a.0,a.1],[b.0,b.1])) { continue }
            if ba.iter().any(|p| bb.iter().any(|q| distance(p.c,q.c) <= p.r+q.r+pad)) { pairs.insert(([a.0,a.1],[b.0,b.1])); }
        } }
    }
    let boxes: Vec<Vec<([f64;3],[f64;3])>> = candidates.iter().map(|c| {
        let located = crate::query::Located::new(c,tol);
        (0..c.faces.len()).map(|f| located.face_box(f)).collect()
    }).collect();
    if let Some(b) = boxed {
        for g in 0..candidates[b].faces.len() {
            for (k,c) in candidates.iter().enumerate().filter(|&(k,_)| k != b) { for f in 0..c.faces.len() {
                if crate::boolean::overlap(&boxes[k][f],&boxes[b][g],pad) { pairs.insert(([k,f],[b,g])); }
            } }
        }
    }
    // **faces whose surfaces were given a meeting are always met** (#942 F2): the given curves are the
    // meeting, laid with no search, and a pair the balls did not bring together leaves them unlaid
    // (a thin stretch's face and the plane it crosses, its balls a resolution apart)
    let bearing: Vec<Vec<(usize,usize)>> = given.iter().map(|g| {
        candidates.iter().enumerate().flat_map(|(k,c)| c.faces.iter().enumerate()
            .filter(|(_,f)| g.surfaces.iter().any(|s| crate::ssi::same(&f.surface,s,tol))).map(move |(f,_)| (k,f)))
            .collect()
    }).collect();
    for (g,on) in given.iter().zip(&bearing) {
        for &(k,f) in on { for &(b,h) in on {
            let face = |k: usize,f: usize| &candidates[k].faces[f].surface;
            if k >= b || !(crate::ssi::same(face(k,f),&g.surfaces[0],tol) && crate::ssi::same(face(b,h),&g.surfaces[1],tol)
                || crate::ssi::same(face(k,f),&g.surfaces[1],tol) && crate::ssi::same(face(b,h),&g.surfaces[0],tol)) { continue }
            if crate::boolean::overlap(&boxes[k][f],&boxes[b][h],pad) { pairs.insert(([k,f],[b,h])); }
        } }
    }
    report.paired = pairs.len();
    if debug() {
        for (a,b) in &pairs {
            let kind = |x: [usize;2]| candidates[x[0]].faces[x[1]].surface.kind();
            eprintln!("trim: paired {a:?} ({}) with {b:?} ({})",kind(*a),kind(*b));
        }
    }
    for (i,a) in boxes.iter().enumerate() { for b in &boxes[i+1..] {
        report.boxed += a.iter().map(|ba| b.iter().filter(|bb| crate::boolean::overlap(ba,bb,pad)).count()).sum::<usize>();
    } }
    if crate::timing() { eprintln!("trim: {} of {} pairs met, in {:?}",report.paired,report.boxed,clock.elapsed()) }
    // 3. the trim: each piece kept where membership differs a hair to either side of it
    let refs = &candidates;
    let every = std::env::var_os("SOLVENT_TRIM_EVERY").is_some();
    let meet = |a: [usize;2],b: [usize;2]| { let (x,y) = if a <= b { (a,b) } else { (b,a) }; every || pairs.contains(&(x,y)) };
    // (a hair past the membership mesh's sag, within which a side does not settle)
    let hair = trim.hair.max(4.*solid.sag);
    // (the tool posed at the roll's ends, read exactly: a piece inside either is inside the sweep)
    let posed: Vec<Brep> = roll.iter().map(|&t| -> Result<Brep,Unbuilt> {
        let q = path.pose(t)?;
        Ok(tool.moved(&crate::geom::Rigid {r:q.r,t:q.p}))
    }).collect::<Result<_,_>>()?;
    let located: Vec<crate::query::Located> = posed.iter().map(|b| crate::query::Located::new(b,tol)).collect();
    let read = std::sync::atomic::AtomicUsize::new(0);
    let kept = std::sync::atomic::AtomicUsize::new(0);
    // what one point of a piece says of it: dropped (`None`, and why), kept facing out ahead or
    // behind, or nothing settled there (`Err`)
    let verdict = |f: &crate::topo::Face,p: V| -> Result<Option<bool>,&'static str> {
        if !reaches((p,p),tol) { return Ok(None) }
        let Some(n) = f.outward(p) else { return Err("no normal") };
        // **a piece proved inside or outside the sweep at a point of it bounds nothing** (#880
        // Q1b): the sweep's depth there more than a few tolerances from nought, a piece of the
        // boundary standing on it to its fit — read before its sides, which a hair either side
        // of a piece shallower than the hair misread (a stretch's sheet dipping into the tool
        // posed at its end by less than the hair, near where it touches it along its rim, read
        // as a boundary and kept)
        if let Ok(Some(_)) = swept.beyond(p,ON_CURVE*tol) {
            if debug() {
                let of = candidates.iter().enumerate().find_map(|(k,c)| c.faces.iter().position(|g| std::ptr::eq(g,f)).map(|i| (k,i)));
                eprintln!("trim: a piece of a {} ({of:?}) at {p:?}: proved off the boundary",f.surface.kind())
            }
            return Ok(None)
        }
        // **nor one inside the tool where it stands at an end of the roll** (#880 Q1b S5): read
        // exactly, where membership's mesh is within its sag of the tool — a sheet near where its
        // contact's branches cross, covered by both end poses less deeply than that sag, read
        // as a boundary and kept along with its rail there (the cone spun about a line through it)
        if located.iter().any(|l| l.solid_place(p) == crate::query::Place::In) {
            if debug() { eprintln!("trim: a piece of a {} at {p:?}: inside an end pose",f.surface.kind()) }
            return Ok(None)
        }
        let (mut ahead,mut behind) = (swept.member(add(p,scale(n,hair))),swept.member(sub(p,scale(n,hair))));
        // **a piece of the tool's own face at an end of the roll, read near that end** (#880 Q1b
        // S5): behind it is the tool; ahead of it the sweep reaches just past that end where the
        // face leads the motion there, and not where it trails — read so where membership, a
        // hair off a piece narrower than the hair, stands within its sag of the tool posed a
        // little way on, and behind read a hair through the tool's base past it (the start cone
        // of a cone screwed through a rod, along its base rim)
        if let Err(super::member::Unsettled::Near {at}) = ahead {
            let span = roll[1]-roll[0];
            let end = (0..2).find(|&e| (at-roll[e]).abs() <= 1e-2*span.abs() && posed[e].faces.iter().any(|g| crate::ssi::same(&f.surface,&g.surface,tol)));
            if let Some(e) = end {
                let h = 1e-6*span*if e == 0 { 1. } else { -1. };
                if let (Ok(q0),Ok(q1)) = (path.pose(roll[e]),path.pose(roll[e]+h)) {
                    let on = q0.inverse().then(q1).point(p);
                    let lead = crate::util::space::dot(sub(on,p),n);
                    if lead.abs() > 1e-9*distance(on,p).max(1e-300) { ahead = Ok(lead > 0.); behind = Ok(true) }
                }
            }
        }
        if debug() {
            let of = candidates.iter().enumerate().find_map(|(k,c)| c.faces.iter().position(|g| std::ptr::eq(g,f)).map(|i| (k,i)));
            eprintln!("trim: a piece of a {} ({of:?}) at {p:?}, normal {n:?}: ahead {ahead:?}, behind {behind:?}",f.surface.kind())
        }
        let (Ok(ahead),Ok(behind)) = (ahead,behind) else { return Err("its sides unsettled") };
        Ok((ahead != behind).then_some(ahead))
    };
    // **a piece is one thing all over** (#942 F2): read at its first point that settles, and held to
    // it at as many as `CONFIRMED` more — a piece the trim reads as the boundary at one point and not
    // at another (or facing the other way) spans a meeting the trim was not given, and is refused
    // with the two points, the witness where that meeting crosses between them; kept or dropped
    // whole, it left the boundary open there without saying where (the cube tumbling 75°: 28 open
    // edges)
    let sided = |f: &crate::topo::Face,points: &mut dyn Iterator<Item=V>| -> Result<Option<bool>,String> {
        read.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
        let mut first = None;
        let own = boxed.is_some_and(|b| candidates[b].faces.iter().any(|g| std::ptr::eq(g,f)));
        let mut settled: Option<(V,Option<bool>)> = None;
        let mut confirmed = 0;
        for p in points.take(8) {
            first.get_or_insert(p);
            if own {
                // (a piece of the box: kept, facing out, where the sweep holds it)
                match swept.member(p) { Ok(true) => { kept.fetch_add(1,std::sync::atomic::Ordering::Relaxed); return Ok(Some(false)) },
                    Ok(false) => return Ok(None),Err(_) => continue }
            }
            let Ok(here) = verdict(f,p) else { continue };
            // (a point read off the boundary witnesses a piece mixed only where it stands clear of
            // the boundary by `CLEAR` hairs, proved: a point a hair from it beside the piece's edge
            // reads the sweep on both its sides, and is no witness — the rod grooved by a leaning
            // torus, its posed tool's piece read inside at a point 0.09 from its boundary)
            let clear = |x: V| matches!(swept.beyond(x,CLEAR*hair),Ok(Some(_)));
            match settled {
                None => settled = Some((p,here)),
                Some((q,there)) if there != here && !clear(if here.is_none() { p } else { q }) => continue,
                Some((q,there)) if there != here => {
                    let of = candidates.iter().enumerate().find_map(|(k,c)| c.faces.iter().position(|g| std::ptr::eq(g,f)).map(|i| (k,i)));
                    let say = |v: Option<bool>| match v { None => "off the boundary",Some(true) => "the boundary, facing ahead",Some(false) => "the boundary, facing behind" };
                    return Err(format!("a piece of a {} (candidate and face {of:?}) is {} at {q:?} and {} at {p:?}: a meeting the trim was \
                        not given parts them",f.surface.kind(),say(there),say(here)))
                }
                Some(_) => { confirmed += 1; if confirmed >= CONFIRMED { break } }
            }
        }
        match settled {
            Some((_,v)) => { if v.is_some() { kept.fetch_add(1,std::sync::atomic::Ordering::Relaxed); } Ok(v) }
            None => Err(format!("a piece of a {} whose sides membership does not settle, at {first:?}",f.surface.kind())),
        }
    };
    // (each piece's reading timed, where `SOLVENT_BREP_TIME` is set: what membership costs beside the
    // Boolean's own meetings, #942 F4)
    let reading = std::sync::atomic::AtomicU64::new(0);
    let side = |f: &crate::topo::Face,points: &mut dyn Iterator<Item=V>| -> Result<Option<bool>,String> {
        let at = crate::util::clock::Instant::now();
        let out = sided(f,points);
        reading.fetch_add(at.elapsed().as_nanos() as u64,std::sync::atomic::Ordering::Relaxed);
        out
    };
    let boolean = crate::util::clock::Instant::now();
    let out = crate::boolean::trimmed(&refs,given,&meet,&side,tol);
    if crate::timing() {
        eprintln!("trim: the Boolean in {:?}, its pieces read by membership in {:.2} s of the cores' time",boolean.elapsed(),
            reading.load(std::sync::atomic::Ordering::Relaxed) as f64*1e-9)
    }
    let out = out?;
    report.pieces = [kept.into_inner(),read.into_inner()];
    if crate::timing() { eprintln!("trim: {} of {} pieces kept, {} faces, in {:?}",report.pieces[0],report.pieces[1],out.faces.len(),clock.elapsed()) }
    // 4. checked and judged
    if debug() {
        let uses = out.uses();
        for (e,u) in uses.iter().enumerate() {
            if u.len() == 2 { continue }
            let edge = &out.edges[e];
            let ends = edge.v.map(|v| out.vertices[v as usize].p);
            let kind = match &edge.curve { crate::topo::EdgeCurve::Curve(c) => c.kind(),_ => "degenerate" };
            eprintln!("trim: edge {e} ({kind}) from {:.4?} to {:.4?} through {:.6?} used by {:?}",ends[0],ends[1],edge.point(0.5*(edge.t[0]+edge.t[1]),&out.vertices),
                u.iter().map(|&(f,_,_)| (f,out.faces[f].surface.kind())).collect::<Vec<_>>());
            // (and any other edge running along it: a meeting laid twice, each copy used once)
            let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&out.vertices);
            for (g,other) in out.edges.iter().enumerate() {
                if g == e { continue }
                let crate::topo::EdgeCurve::Curve(c) = &other.curve else { continue };
                let near = (0..=64).map(|k| crate::util::space::distance(other.point(other.t[0]+(other.t[1]-other.t[0])*k as f64/64.,&out.vertices),mid)).fold(f64::INFINITY,f64::min);
                if near <= 1e-3 { eprintln!("trim:   beside it, edge {g} ({}) {near:.2e} off, used by {:?}",c.kind(),uses[g].iter().map(|&(f,_,_)| f).collect::<Vec<_>>()) }
            }
        }
    }
    if let Err(why) = out.check(tol) {
        if let Some(witness) = missing_across(&out,&candidates,&|f,p| match verdict(f,p) {
            Ok(None) if !matches!(swept.beyond(p,CLEAR*hair),Ok(Some(_))) => None,
            Ok(v) => Some(v),
            Err(_) => None,
        },hair,tol) { return Err(Unbuilt::Failed(witness)) }
        if debug() {
            for (fi,f) in out.faces.iter().enumerate().filter(|(_,f)| f.loops.len() > 1) {
                for (li,l) in f.loops.iter().enumerate() {
                    let ends: Vec<_> = l.iter().map(|c| { let e = &out.edges[c.edge as usize]; let kind = e.curve.kind();
                        (c.edge,kind,c.pcurve.at(e.t[0],e,&f.surface,&out.vertices),c.pcurve.at(e.t[1],e,&f.surface,&out.vertices),c.reversed) }).collect();
                    eprintln!("trim: face {fi} ({}) loop {li}: {ends:?}",f.surface.kind());
                }
            }
            for (fi,f) in out.faces.iter().enumerate() {
                for c in f.loops.iter().flatten() {
                    let e = &out.edges[c.edge as usize];
                    let crate::topo::EdgeCurve::Curve(cv) = &e.curve else { continue };
                    let off = (0..=64).map(|k| { let p = e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/64.,&out.vertices); (f.surface.implicit(p).abs(),k) }).max_by(|x,y| x.0.total_cmp(&y.0)).unwrap();
                    if off.0 > 0.5*tol { eprintln!("trim: face {fi} ({}) edge {} ({}) over {:?} from {:?} to {:?}: {:.2e} off the face at {}/64",f.surface.kind(),c.edge,cv.kind(),e.t,out.vertices[e.v[0] as usize].p,out.vertices[e.v[1] as usize].p,off.0,off.1) }
                }
            }
        }
        return Err(why.into())
    }
    if debug() {
        let flux = crate::props::fluxes(&out);
        eprintln!("trim: built {} faces, volume {:.6}",out.faces.len(),flux.iter().sum::<f64>()/3.);
        for (i,x) in flux.iter().enumerate() {
            if x.is_finite() { continue }
            let f = &out.faces[i];
            eprintln!("trim:   face {i} ({}): flux {x}, {} loops",f.surface.kind(),f.loops.len());
            for c in f.loops.iter().flatten() {
                let e = &out.edges[c.edge as usize];
                let ends = [c.pcurve.at(e.t[0],e,&f.surface,&out.vertices),c.pcurve.at(e.t[1],e,&f.surface,&out.vertices)];
                let kind = match &e.curve { crate::topo::EdgeCurve::Curve(cv) => cv.kind(),_ => "degenerate" };
                eprintln!("trim:     edge {} ({kind}) over {:?}, reversed {}, from {:?} to {:?}, ends in its face {:?}",c.edge,e.t,c.reversed,out.vertices[e.v[0] as usize].p,out.vertices[e.v[1] as usize].p,ends);
            }
        }
    }
    let grown;
    let domain = match &held_box {
        // (within the reach the solid is the sweep)
        Some(b) => b,
        None => {
            let (lo,hi) = out.bounds();
            let grow = 0.1*distance(lo,hi);
            let foot = [0.5*(lo[0]+hi[0]),0.5*(lo[1]+hi[1]),lo[2]-grow];
            let half = 0.5*(hi[0]-lo[0]).max(hi[1]-lo[1])+grow;
            grown = crate::build::prism(&crate::build::square(foot,[0.,0.,1.],half),0.,hi[2]-lo[2]+2.*grow)?;
            &grown
        }
    };
    let clock = crate::util::clock::Instant::now();
    super::closed::judge(&out,&swept,&domain,None,&[],tol)?;
    if crate::timing() { eprintln!("trim: judged in {:?}",clock.elapsed()) }
    Ok((out,report))
}

/// A triangle `t` of a candidate's mesh (its sag `sag`) zoned against the sweep's depth: the balls
/// left near the boundary, and the triangles proved interior, exterior and left near. Halved across
/// its longest edge until proved, or its ball no wider than `resolution`.
fn zoned(swept: &Swept,t: [V;3],sag: f64,resolution: f64) -> Result<(Vec<Ball>,[usize;3]),Unbuilt> {
    let mut balls = Vec::new();
    let mut zones = [0;3];
    let mut stack = vec![t];
    while let Some(t) = stack.pop() {
        let c = scale(add(add(t[0],t[1]),t[2]),1./3.);
        let r = t.iter().map(|&p| distance(p,c)).fold(0.,f64::max)+sag;
        match swept.beyond(c,r).map_err(|e| Unbuilt::Failed(format!("a candidate's zone not read: {e:?}")))? {
            Some(true) => zones[0] += 1,
            Some(false) => zones[1] += 1,
            None if r <= resolution => { zones[2] += 1; balls.push(Ball {c,r}) }
            None => {
                // halved across its longest edge, so a sliver (a ruled face's, long along its
                // rulings) comes to its width in as many halvings as its length is widths
                let k = (0..3).max_by(|&i,&j| distance(t[i],t[(i+1)%3]).total_cmp(&distance(t[j],t[(j+1)%3]))).unwrap_or(0);
                let (p,q,o) = (t[k],t[(k+1)%3],t[(k+2)%3]);
                let m = scale(add(p,q),0.5);
                stack.extend([[p,m,o],[m,q,o]]);
            }
        }
    }
    Ok((balls,zones))
}

/// **A candidate a segment screws into** (E1's hand-entered candidates): the segment from `p` to
/// `q` carried by `screw` over the roll `[a, b]`, a little past its ends and the roll's (`closed`'s
/// `Cutter::Patch`): a patch of the ruled helicoid (a helicoid where the segment is square to the
/// axis), one face.
pub fn screwed_segment(screw: &crate::trajectory::Screw,p: V,q: V,roll: [f64;2]) -> Result<Brep,Unbuilt> {
    let path = Trajectory::Twist(*screw);
    let pose_at = |t: f64| path.pose(t).map_err(Unbuilt::from);
    super::closed::patch(screw,&pose_at,p,q,roll)
}

/// **The faces of `solid` that `keep` keeps, as a sheet** (E1's hand-entered candidates): a
/// cylinder's side without its ends, the one face a candidate is.
pub fn sheet_of(solid: &Brep,keep: &dyn Fn(&crate::topo::Face) -> bool) -> Brep {
    let mut b = solid.clone();
    b.faces.retain(|f| keep(f));
    b.compacted()
}

/// `SOLVENT_TRIM_DEBUG` is set: each piece's reading is narrated.
fn debug() -> bool { std::env::var_os("SOLVENT_TRIM_DEBUG").is_some() }

/// **A tool's planar faces at a pose, each a sheet past its edges** (E1's hand-entered candidates:
/// the end poses): every plane of `tool` a square sheet in its plane about its face, `margin` past
/// the face's box each way — so no edge of the tool is an edge of a candidate, and a sheet an edge
/// sweeps, holding that edge at the roll's end, meets the face's plane across the face (a meeting
/// crossing an edge that lies in the other surface is one the arrangement does not lay). The
/// faces' material is the tool's: each sheet faces as its face does.
pub fn planes_past(tool: &Brep,margin: f64) -> Result<Vec<Brep>,Unbuilt> {
    let located = crate::query::Located::new(tool,0.);
    let mut out = Vec::new();
    for (fi,f) in tool.faces.iter().enumerate() {
        let crate::geom::Surface::Plane(frame) = &f.surface else {
            return Err(Unbuilt::Curved {at:f.surface.point([0.,0.])})
        };
        let (lo,hi) = located.face_box(fi);
        let c = scale(add(lo,hi),0.5);
        let n = if f.reversed { scale(frame.z,-1.) } else { frame.z };
        let o = sub(c,scale(n,crate::util::space::dot(sub(c,frame.o),n)));
        let half = 0.5*distance(lo,hi)+margin;
        let slab = crate::build::prism(&crate::build::square(o,n,half),0.,1.)?;
        // the slab's face in the plane, turned to face as `f` does
        let mut sheet = sheet_of(&slab,&|g| g.surface.implicit(o).abs() <= 1e-9*(1.+half) && matches!(g.surface,crate::geom::Surface::Plane(_)));
        for g in &mut sheet.faces {
            let m = g.outward(o).unwrap_or(n);
            if crate::util::space::dot(m,n) < 0. { *g = crate::complex::faced(g,true) }
        }
        out.push(sheet);
    }
    Ok(out)
}

/// **The cylinder a slid torus's contact sweeps, over its whole apparent contour** (E1's
/// hand-entered candidates, `closed`'s `Cutter::Contour`): the two halves' contour extruded along
/// the slide over `span` — its side alone, a sheet.
pub fn contour_cylinder(halves: &[crate::slid::Slid;2],span: [f64;2]) -> Result<Brep,Unbuilt> {
    let solid = super::closed::contour_solid(halves,span)?;
    Ok(sheet_of(&solid,&|f| !matches!(f.surface,crate::geom::Surface::Plane(_))))
}

/// **The sheet a slid torus's contact sweeps over one arc of its apparent contour** (E1's
/// hand-entered candidates, `closed`'s `Cutter::Sheet`): the arc extruded along the slide over
/// `span`, one face — an arc between two folds of the contour, so a contour that folds is entered
/// in its regular arcs.
pub fn contour_arc(arc: &crate::slid::Slid,span: [f64;2]) -> Result<Brep,Unbuilt> {
    Ok(super::closed::contour_sheet(arc,span)?)
}
