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
    // 1. each candidate's faces zoned: the balls near the boundary, by (candidate, face)
    let clock = crate::util::clock::Instant::now();
    let mut near: Vec<(usize,usize,Ball)> = Vec::new();
    for (k,cand) in candidates.iter().enumerate() {
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
            near.extend(balls.into_iter().map(|b| (k,*f,b)));
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
    report.paired = pairs.len();
    if debug() {
        for (a,b) in &pairs {
            let kind = |x: [usize;2]| candidates[x[0]].faces[x[1]].surface.kind();
            eprintln!("trim: paired {a:?} ({}) with {b:?} ({})",kind(*a),kind(*b));
        }
    }
    let boxes: Vec<Vec<([f64;3],[f64;3])>> = candidates.iter().map(|c| {
        let located = crate::query::Located::new(c,tol);
        (0..c.faces.len()).map(|f| located.face_box(f)).collect()
    }).collect();
    for (i,a) in boxes.iter().enumerate() { for b in &boxes[i+1..] {
        report.boxed += a.iter().map(|ba| b.iter().filter(|bb| crate::boolean::overlap(ba,bb,pad)).count()).sum::<usize>();
    } }
    if crate::timing() { eprintln!("trim: {} of {} pairs met, in {:?}",report.paired,report.boxed,clock.elapsed()) }
    // 3. the trim: each piece kept where membership differs a hair to either side of it
    let refs: Vec<&Brep> = candidates.iter().collect();
    let every = std::env::var_os("SOLVENT_TRIM_EVERY").is_some();
    let meet = |a: [usize;2],b: [usize;2]| { let (x,y) = if a <= b { (a,b) } else { (b,a) }; every || pairs.contains(&(x,y)) };
    // (a hair past the membership mesh's sag, within which a side does not settle)
    let hair = trim.hair.max(4.*solid.sag);
    let read = std::sync::atomic::AtomicUsize::new(0);
    let kept = std::sync::atomic::AtomicUsize::new(0);
    let side = |f: &crate::topo::Face,points: &mut dyn Iterator<Item=V>| -> Result<Option<bool>,String> {
        read.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
        let mut first = None;
        for p in points.take(8) {
            first.get_or_insert(p);
            let Some(n) = f.outward(p) else { continue };
            // **a piece proved inside or outside the sweep at a point of it bounds nothing** (#880
            // Q1b): the sweep's depth there more than a few tolerances from nought, a piece of the
            // boundary standing on it to its fit — read before its sides, which a hair either side
            // of a piece shallower than the hair misread (a stretch's sheet dipping into the tool
            // posed at its end by less than the hair, near where it touches it along its rim, read
            // as a boundary and kept)
            if let Ok(Some(_)) = swept.beyond(p,ON_CURVE*tol) {
                if debug() { eprintln!("trim: a piece of a {} at {p:?}: proved off the boundary",f.surface.kind()) }
                return Ok(None)
            }
            let (ahead,behind) = (swept.member(add(p,scale(n,hair))),swept.member(sub(p,scale(n,hair))));
            if debug() {
                let of = candidates.iter().enumerate().find_map(|(k,c)| c.faces.iter().position(|g| std::ptr::eq(g,f)).map(|i| (k,i)));
                eprintln!("trim: a piece of a {} ({of:?}) at {p:?}, normal {n:?}: ahead {ahead:?}, behind {behind:?}",f.surface.kind())
            }
            let (Ok(ahead),Ok(behind)) = (ahead,behind) else { continue };
            if ahead == behind { return Ok(None) }
            kept.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
            return Ok(Some(ahead))
        }
        Err(format!("a piece of a {} whose sides membership does not settle, at {first:?}",f.surface.kind()))
    };
    let out = crate::boolean::trimmed(&refs,given,&meet,&side,tol)?;
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
        }
    }
    out.check(tol)?;
    if debug() { eprintln!("trim: built {} faces, volume {:.6}",out.faces.len(),crate::props::volume(&out)) }
    let (lo,hi) = out.bounds();
    let grow = 0.1*distance(lo,hi);
    let foot = [0.5*(lo[0]+hi[0]),0.5*(lo[1]+hi[1]),lo[2]-grow];
    let half = 0.5*(hi[0]-lo[0]).max(hi[1]-lo[1])+grow;
    let domain = crate::build::prism(&crate::build::square(foot,[0.,0.,1.],half),0.,hi[2]-lo[2]+2.*grow)?;
    super::closed::judge(&out,&swept,&domain,None,&[],tol)?;
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
