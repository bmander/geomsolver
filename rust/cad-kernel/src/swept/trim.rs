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
    /// The curves of meetings the trim was not given that it completed from their witnesses
    /// (#942 F2).
    pub completed: usize,
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
    /// **The region the sweep is wanted within** (#880 T1c), where it is cut from a blank the
    /// region holds: the trim builds the sweep's part inside it — the candidates' pieces outside it
    /// dropped, and met only where they come near it, the region's own faces kept where the sweep
    /// holds them — the same cut from the blank, since a blank inside the region loses only what of
    /// the sweep lies in it. What the sweep does beyond — a fold's cusp where the tool stands at an
    /// end of a roll run past the blank (the twist drill's steep flute wheel) — is not asked.
    pub reach: Option<Reach>,
}

/// **A region the sweep is wanted within** (`Trim::reach`): a box, or a cylinder about the motion's
/// axis (`Turned`), which a screw, a turn or a slide about that axis carries onto itself, so that
/// no point of the tool outside it ever enters it (`closed`'s contact road, cut from a blank).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Reach {
    /// The box from `lo` to `hi`.
    Box(V,V),
    /// The cylinder of radius `r` about the line through `o` along the unit `z`, from `span[0]` to
    /// `span[1]` along it.
    Turned { o: V,z: V,r: f64,span: [f64;2] },
}

impl Reach {
    /// A box holding the region.
    pub fn bounds(&self) -> (V,V) {
        match *self {
            Reach::Box(lo,hi) => (lo,hi),
            Reach::Turned {o,z,r,span} => {
                let ends = span.map(|t| add(o,scale(z,t)));
                // (a circle of radius `r` square to `z` reaches `r √(1 − z_k²)` along axis `k`)
                let reach: V = std::array::from_fn(|k| r*(1.-z[k]*z[k]).max(0.).sqrt());
                (std::array::from_fn(|k| ends[0][k].min(ends[1][k])-reach[k]),std::array::from_fn(|k| ends[0][k].max(ends[1][k])+reach[k]))
            }
        }
    }
    /// Whether the box `b` comes within `r` of the region: its box, and for the cylinder, no point
    /// of `b` farther than `r` past its side (the box's nearest distance from the axis, read at its
    /// middle less its half diagonal).
    pub fn reaches(&self,b: (V,V),r: f64) -> bool {
        let (lo,hi) = self.bounds();
        if !crate::boolean::overlap(&b,&(lo,hi),r) { return false }
        match *self {
            Reach::Box(..) => true,
            Reach::Turned {o,z,r: radius,..} => {
                let c = scale(add(b.0,b.1),0.5);
                let q = sub(c,o);
                let off = crate::util::space::norm(sub(q,scale(z,crate::util::space::dot(q,z))));
                off-0.5*distance(b.0,b.1) <= radius+r
            }
        }
    }
    /// The region as a solid.
    pub fn solid(&self) -> Result<Brep,String> {
        match *self {
            Reach::Box(lo,hi) => crate::build::block(lo,hi),
            Reach::Turned {o,z,r,span} => crate::build::prism(&crate::build::disc(o,z,r),span[0],span[1]),
        }
    }
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
/// A point is on a kept face where a face of the solid on its candidate face's own surface holds
/// it, not where the face kept beside the edge does: one tangent to that candidate along the edge
/// holds its points across it too.
/// Either way a meeting the trim was not given parts that point from where its piece was read
/// (`readings`: each piece's face, the point it was settled at and what it read there), which is
/// the witness's other point.  None where neither is found (the boundary open for some other
/// reason).  `read` is the trim's own reading of a point of a face: `Some` only where it settles,
/// and a point off the boundary only where it is proved clear.
#[allow(clippy::too_many_arguments,clippy::type_complexity)]
fn missing_across(out: &Brep,candidates: &[&Brep],read: &dyn Fn(&crate::topo::Face,V) -> Option<Option<bool>>,
    readings: &[((usize,usize),V,Option<bool>)],hair: f64,tol: f64) -> Option<Witness> {
    use crate::util::space::{cross,normalised};
    let uses = out.uses();
    let located: Vec<crate::query::Located> = candidates.iter().map(|c| crate::query::Located::new(c,tol)).collect();
    let kept = crate::query::Located::new(out,tol);
    // (where the piece of face `face` holding `q` was read as `kept` or not: the nearest such reading)
    let read_at = |face: (usize,usize),q: V,kept: bool| readings.iter().filter(|r| r.0 == face && r.2.is_some() == kept)
        .map(|r| r.1).min_by(|x,y| distance(*x,q).total_cmp(&distance(*y,q))).unwrap_or(q);
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
                    // (kept where a face of the solid on that candidate face's own surface holds it:
                    // the face kept beside the edge, read alone, holds a point of a candidate face
                    // tangent to it along the edge — the full turn's start torus, its rim the
                    // tube's, read as the tube's kept piece, and the refusal named the torus)
                    let on_kept = out.faces.iter().enumerate().any(|(j,g)| g.surface == f.surface && kept.face_holds(j,q));
                    if debug() && on_kept && matches!(read(f,q),Some(None)) {
                        // (the kept face's loops: where its piece is bounded)
                        let kf = &out.faces[u[0].0];
                        for (li,l) in kf.loops.iter().enumerate() { for c in l {
                            let e = &out.edges[c.edge as usize];
                            eprintln!("trim: the kept face {} loop {li}: edge {} ({}) from {:?} to {:?}, used {}",u[0].0,c.edge,e.curve.kind(),
                                out.vertices[e.v[0] as usize].p,out.vertices[e.v[1] as usize].p,uses[c.edge as usize].len());
                        } }
                    }
                    match read(f,q) {
                        Some(None) if on_kept => return Some(Witness {face:(k,i),at:[read_at((k,i),q,true),q],
                            said:format!("the boundary is open along a {} edge through {m:?}: the {} piece kept beside it (candidate and \
                                face ({k}, {i})) is not the boundary at {q:?}, a meeting the trim was not given parting it from where \
                                it was read",c.kind(),f.surface.kind())}),
                        Some(Some(_)) if !on_kept => return Some(Witness {face:(k,i),at:[read_at((k,i),q,false),q],
                            said:format!("the boundary is open along a {} edge through {m:?}: a piece of a {} (candidate and face \
                                ({k}, {i})) is the boundary beside it at {q:?} and was not kept, a meeting the trim was not given \
                                parting it from where its piece was read",c.kind(),f.surface.kind())}),
                        _ => {}
                    }
                }
            }
        }
    }
    None
}

/// **What a trim refused shows** (#942 F2): a face of a candidate (its candidate and face) and two
/// points of it membership read apart — one as the boundary, the other not (or facing the other
/// way) — so a meeting the trim was not given crosses the face between them; and the refusal said.
#[derive(Clone,Debug)]
struct Witness { face: (usize,usize),at: [V;2],said: String }

/// **A meeting completed from its witness** (#942 F2): the face's surface walked between the
/// witness's two points in its own parameters, and wherever another candidate's face crosses it
/// there — its surface's signed distance changing sign, the root on that face — that the trim did
/// not meet (no given curve through it, nor the pair met by search), the two surfaces' meeting is
/// traced from the root (`ssi::trace`, else marched in their parameters as the general road's
/// branches beside its rails are) and given: added to the pair's given curves where it has some, a
/// new given meeting where it has none, and every two faces bearing the two surfaces met
/// (`pair_given`).  How many curves it gave; none, and the trim is refused with the witness — or,
/// where another face of the same surface crosses the walk (a wall at other instants passing back
/// through itself), with that: a surface's meeting with itself is no meeting two surfaces are given.
#[allow(clippy::too_many_arguments)]
fn completed(w: &Witness,candidates: &[&Brep],located: &[crate::query::Located],bounding: &BTreeSet<(usize,usize)>,
    boxes: &[Vec<(V,V)>],given: &mut Vec<Given>,pairs: &mut BTreeSet<([usize;2],[usize;2])>,buried: &dyn Fn(V) -> bool,hair: f64,tol: f64)
    -> Result<usize,String> {
    const WALK: usize = 64;
    let (k,i) = w.face;
    let s = &candidates[k].faces[i].surface;
    let [a,b] = w.at;
    if distance(a,b) <= hair { return Ok(0) }
    // (the two points' parameters on the face's own branch of its surface: read from its parameters,
    // a folded wall's nearest foot anywhere may be on another of its sheets)
    let ua = located[k].uv(i,a);
    let ub = s.inverse_near(b,ua);
    let at = |t: f64| s.point([ua[0]+(ub[0]-ua[0])*t,ua[1]+(ub[1]-ua[1])*t]);
    let walk: Vec<V> = (0..=WALK).map(|j| at(j as f64/WALK as f64)).collect();
    let lo: V = std::array::from_fn(|d| walk.iter().map(|p| p[d]).fold(f64::INFINITY,f64::min)-hair);
    let hi: V = std::array::from_fn(|d| walk.iter().map(|p| p[d]).fold(f64::NEG_INFINITY,f64::max)+hair);
    let same_pair = |g: &Given,x: &crate::geom::Surface,y: &crate::geom::Surface| {
        alike(&g.surfaces[0],x,tol) && alike(&g.surfaces[1],y,tol)
            || alike(&g.surfaces[0],y,tol) && alike(&g.surfaces[1],x,tol)
    };
    let mut gave = 0;
    let mut missed = None;
    for (k2,c) in candidates.iter().enumerate() {
        for (g,face) in c.faces.iter().enumerate() {
            if (k2,g) == (k,i) || !crate::boolean::overlap(&boxes[k2][g],&(lo,hi),4.*tol) { continue }
            let sg = &face.surface;
            if alike(sg,s,tol) {
                if let Some(x) = crossed_by_itself(&walk,sg,&located[k2],g) {
                    return Err(format!("{}; there face ({k2}, {g}) of the same {} crosses it at {x:?}, the surface passing back through \
                        itself: a meeting of a surface with itself, which the trim is not given",w.said,s.kind()))
                }
                continue
            }
            let h: Vec<f64> = walk.iter().map(|&x| sg.implicit(x)).collect();
            for j in 0..WALK {
                if !(h[j]*h[j+1] <= 0.) || h[j] == h[j+1] { continue }
                let (mut t0,mut t1,mut h0) = (j as f64/WALK as f64,(j+1) as f64/WALK as f64,h[j]);
                for _ in 0..60 {
                    let tm = 0.5*(t0+t1);
                    let hm = sg.implicit(at(tm));
                    if hm*h0 > 0. { (t0,h0) = (tm,hm) } else { t1 = tm }
                }
                let x = at(0.5*(t0+t1));
                if !located[k2].face_holds(g,x) {
                    if debug() { eprintln!("trim: completing from {:?}: face ({k2}, {g})'s surface crosses it at {x:?}, outside the face",w.at) }
                    continue
                }
                // (a crossing the sweep is proved to hold more than a few tolerances deep bounds nothing:
                // a meeting there is buried, and needs no cut — the full turn's start and end tori, where
                // their tubes touch among the instants between)
                if buried(x) {
                    if debug() { eprintln!("trim: completing from {:?}: face ({k2}, {g}), a {}, crosses it at {x:?}, buried",w.at,sg.kind()) }
                    continue
                }
                let key = if [k,i] <= [k2,g] { ([k,i],[k2,g]) } else { ([k2,g],[k,i]) };
                let had = given.iter().position(|gv| same_pair(gv,s,sg));
                let through = had.is_some_and(|n| given[n].curves.iter().any(|cv| distance(cv.point(cv.inverse(x)),x) <= hair));
                // (met there already; or met by search, which missed it: not the trim's to give — a pair
                // given what one trace finds loses what its search found — but named)
                if through {
                    if debug() { eprintln!("trim: completing from {:?}: face ({k2}, {g}), a {}, crosses it at {x:?}, on a curve given",w.at,sg.kind()) }
                    continue
                }
                if had.is_none() && pairs.contains(&key) { missed.get_or_insert((k2,g,x)); continue }
                let (la,ha) = candidates[k].bounds();
                let (lb,hb) = c.bounds();
                let l: V = std::array::from_fn(|d| la[d].max(lb[d])-tol);
                let u: V = std::array::from_fn(|d| ha[d].min(hb[d])+tol);
                // (by the kernel's one fallback: traced in space, and where that is refused as shallow
                // marched in both surfaces' parameters, over the two candidates' own)
                let (dk,dg) = (super::general::parameters(candidates[k],tol),super::general::parameters(c,tol));
                let traced = crate::meeting::through(s,dk,sg,dg,&[x],l,u,tol).unwrap_or_default();
                let traced: Vec<crate::geom::Curve> = traced.into_iter()
                    .filter(|cv| distance(cv.point(cv.inverse(x)),x) <= hair).collect();
                if debug() {
                    eprintln!("trim: completed from {:?} at {x:?}, a {} and a {} met in {} curves",w.at,s.kind(),sg.kind(),traced.len())
                }
                if traced.is_empty() { continue }
                gave += traced.len();
                let n = match had {
                    Some(n) => { given[n].curves.extend(traced); n }
                    None => { given.push(Given {surfaces:[s.clone(),sg.clone()],curves:traced,touches:Vec::new()}); given.len()-1 }
                };
                pairs.insert(key);
                pair_given(&given[n],candidates,bounding,boxes,pairs,4.*tol,tol);
            }
        }
    }
    match missed {
        Some((k2,g,x)) if gave == 0 => Err(format!("{}; there face ({k2}, {g}), a {}, crosses it at {x:?}: the two were met by search, \
            which missed that",w.said,candidates[k2].faces[g].surface.kind())),
        _ => Ok(gave),
    }
}

/// **Where a face of a surface crosses a walk over the same surface elsewhere** (#942 F2): each
/// point of the walk's foot on face `g` read by continuation from the face's own parameters
/// (`Located::uv`), so on its branch of the surface and not the walk's own, and where the walk
/// passes from one side of it to the other within the face, near it, the point between. None where
/// it does not.
fn crossed_by_itself(walk: &[V],s: &crate::geom::Surface,located: &crate::query::Located,g: usize) -> Option<V> {
    let step = walk.windows(2).map(|w| distance(w[0],w[1])).fold(0.,f64::max);
    let mut uv = located.uv(g,walk[0]);
    let mut last: Option<(V,f64)> = None;
    for &x in walk {
        uv = s.inverse_near(x,uv);
        let foot = s.point(uv);
        let here = s.normal(uv).filter(|_| located.holds_parameters(g,uv) && distance(foot,x) <= 2.*step)
            .map(|n| crate::util::space::dot(sub(x,foot),n));
        if let (Some((y,before)),Some(d)) = (last,here) { if before*d < 0. { return Some(scale(add(x,y),0.5)) } }
        last = here.map(|d| (x,d));
    }
    None
}

/// Every two faces of two candidates bearing the two surfaces `g` gives a meeting of, each near the
/// boundary somewhere (`bounding`), met where their boxes come within `pad` (#942 F2).
fn pair_given(g: &Given,candidates: &[&Brep],bounding: &BTreeSet<(usize,usize)>,boxes: &[Vec<(V,V)>],
    pairs: &mut BTreeSet<([usize;2],[usize;2])>,pad: f64,tol: f64) {
    let bearing = |s: &crate::geom::Surface| -> Vec<(usize,usize)> { candidates.iter().enumerate().flat_map(|(k,c)| c.faces.iter().enumerate()
        .filter(|(_,f)| alike(&f.surface,s,tol)).map(move |(f,_)| (k,f))).collect() };
    let (x,y) = (bearing(&g.surfaces[0]),bearing(&g.surfaces[1]));
    for &(k,f) in &x { for &(b,h) in &y {
        if k == b || !bounding.contains(&(k,f)) || !bounding.contains(&(b,h)) || !crate::boolean::overlap(&boxes[k][f],&boxes[b][h],pad) { continue }
        pairs.insert(if [k,f] <= [b,h] { ([k,f],[b,h]) } else { ([b,h],[k,f]) });
    } }
}

/// Whether two surfaces are one: equal, as the arrangement's index of surfaces first reads them
/// (a face family's wall copied), or the same by `ssi::same`.
fn alike(a: &crate::geom::Surface,b: &crate::geom::Surface,tol: f64) -> bool { a == b || crate::ssi::same(a,b,tol) }

/// How many rounds a trim completes meetings from its witnesses before it refuses (#942 F2).
const ROUNDS: usize = 8;

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
    let swept = Swept::new(&solid,path,roll);
    if crate::timing() { eprintln!("trim: the tool meshed for membership: {} triangles, sag {:.2e}",solid.triangles(),solid.sag) }
    let mut report = Report {faces:candidates.iter().map(|c| c.faces.len()).sum(),..Report::default()};
    // (the reach: a ball reaches it where it comes within its radius of the box; a point is past it
    // farther than the tolerance outside)
    let reaches = |b: ([f64;3],[f64;3]),r: f64| trim.reach.is_none_or(|reach| reach.reaches(b,r));
    let held_box = trim.reach.map(|reach| reach.solid()).transpose().map_err(Unbuilt::Failed)?;
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
        // (a triangle clear of the reach, by more than the sag, holds no ball within it: not zoned)
        let tris: Vec<(usize,[V;3])> = mesh.tris.iter().zip(&mesh.of).map(|(t,&f)| (f as usize,t.map(|i| mesh.pts[i as usize])))
            .filter(|(_,t)| reaches((std::array::from_fn(|k| t[0][k].min(t[1][k]).min(t[2][k])),std::array::from_fn(|k| t[0][k].max(t[1][k]).max(t[2][k]))),sag))
            .collect();
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
    // **faces near the boundary whose surfaces were given a meeting are always met** (#942 F2): the
    // given curves are the meeting, laid with no search, and a pair the balls did not bring together
    // leaves them unlaid (a thin stretch's face and the plane it crosses, its balls a resolution
    // apart); a face with no ball near the boundary bounds nothing, and its meeting is buried (#880
    // P4: met, a sphere's piece inside the sweep left with a hole in no piece)
    let bounding: BTreeSet<(usize,usize)> = near.iter().map(|&(k,f,_)| (k,f)).collect();
    for g in given { pair_given(g,&candidates,&bounding,&boxes,&mut pairs,pad,tol) }
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
    // (each candidate's faces by where they stand, to name the face a piece is of; what a refusal
    // of a piece saw, and each piece's face, the point it settled at and its verdict there: what
    // the trim completes a missing meeting from, #942 F2)
    let index: BTreeMap<usize,(usize,usize)> = candidates.iter().enumerate()
        .flat_map(|(k,c)| c.faces.iter().enumerate().map(move |(i,f)| (f as *const crate::topo::Face as usize,(k,i)))).collect();
    let of = |f: &crate::topo::Face| index.get(&(f as *const crate::topo::Face as usize)).copied();
    let witnesses: std::sync::Mutex<Vec<Witness>> = std::sync::Mutex::new(Vec::new());
    let readings: std::sync::Mutex<Vec<((usize,usize),V,Option<bool>)>> = std::sync::Mutex::new(Vec::new());
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
            // (the end the instant membership was stuck at is nearer, the piece lying on the tool's
            // surface posed there: the end that holds it, #942 F6)
            let held = |e: usize| (at-roll[e]).abs() <= (at-roll[1-e]).abs();
            let end = (0..2).find(|&e| held(e) && posed[e].faces.iter().any(|g| crate::ssi::same(&f.surface,&g.surface,tol)));
            if let Some(e) = end {
                // (the way the tool's point at `p` moves as the roll runs on into it, read off the
                // motion's own velocity there, #942 F6: its part along the normal is nought where
                // its rounding leaves it unread)
                if let (Ok(q0),Ok(m)) = (path.pose(roll[e]),path.at(roll[e])) {
                    let v = m.velocity(q0.inverse().point(p));
                    let lead = crate::util::space::dot(v,n)*if e == 0 { 1. } else { -1. };
                    if lead.abs() > crate::util::fmath::HALF_READ*crate::util::space::norm(v) { ahead = Ok(lead > 0.); behind = Ok(true) }
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
                    let say = |v: Option<bool>| match v { None => "off the boundary",Some(true) => "the boundary, facing ahead",Some(false) => "the boundary, facing behind" };
                    let said = format!("a piece of a {} (candidate and face {:?}) is {} at {q:?} and {} at {p:?}: a meeting the trim was \
                        not given parts them",f.surface.kind(),of(f),say(there),say(here));
                    if let Some(face) = of(f) { witnesses.lock().unwrap().push(Witness {face,at:[q,p],said:said.clone()}); }
                    return Err(said)
                }
                Some(_) => { confirmed += 1; if confirmed >= CONFIRMED { break } }
            }
        }
        match settled {
            Some((p,v)) => {
                if v.is_some() { kept.fetch_add(1,std::sync::atomic::Ordering::Relaxed); }
                if let Some(face) = of(f) { readings.lock().unwrap().push((face,p,v)); }
                Ok(v)
            }
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
    // **a meeting the trim was not given, completed from its witness** (#942 F2): a piece read apart
    // or the boundary left open beside a piece read so, its missing meeting traced from where it
    // crosses between the two points and given, and the trim run again — up to `ROUNDS` times; a
    // witness nothing missing crosses is the refusal
    let completing: Vec<crate::query::Located> = candidates.iter().map(|c| crate::query::Located::new(c,tol)).collect();
    let mut given: Vec<Given> = given.to_vec();
    let mut round = 0;
    let out = loop {
        witnesses.lock().unwrap().clear();
        readings.lock().unwrap().clear();
        let meet = |a: [usize;2],b: [usize;2]| { let (x,y) = if a <= b { (a,b) } else { (b,a) }; every || pairs.contains(&(x,y)) };
        let boolean = crate::util::clock::Instant::now();
        let out = crate::boolean::trimmed(&refs,&given,&meet,&side,tol);
        if crate::timing() {
            eprintln!("trim: the Boolean in {:?}, its pieces read by membership in {:.2} s of the cores' time",boolean.elapsed(),
                reading.load(std::sync::atomic::Ordering::Relaxed) as f64*1e-9)
        }
        let seen = match out {
            // (the witness the refusal said: the pieces are read across the cores, and the Boolean
            // says the first refusal in its own order)
            Err(why) => match witnesses.lock().unwrap().iter().find(|w| w.said == why) { Some(w) => w.clone(),None => return Err(why.into()) },
            Ok(out) => match out.check(tol) {
                Ok(()) => break out,
                Err(why) => {
                    if debug() { eprintln!("trim: the check refused: {why}"); narrate_open(&out,tol) }
                    let readings = readings.lock().unwrap().clone();
                    let across = missing_across(&out,&candidates,&|f,p| match verdict(f,p) {
                        Ok(None) if !matches!(swept.beyond(p,CLEAR*hair),Ok(Some(_))) => None,
                        Ok(v) => Some(v),
                        Err(_) => None,
                    },&readings,hair,tol);
                    match across {
                        Some(w) => w,
                        None => { if debug() { narrate_open(&out,tol) } return Err(why.into()) }
                    }
                }
            },
        };
        round += 1;
        let gave = if round > ROUNDS { 0 } else {
            completed(&seen,&candidates,&completing,&bounding,&boxes,&mut given,&mut pairs,&|x| matches!(swept.beyond(x,ON_CURVE*tol),Ok(Some(_))),hair,tol).map_err(Unbuilt::Failed)?
        };
        if debug() || crate::timing() { eprintln!("trim: round {round}: {} curves completed from {:?}",gave,seen.at) }
        if gave == 0 { return Err(Unbuilt::Failed(seen.said)) }
        report.completed += gave;
    };
    report.pieces = [kept.into_inner(),read.into_inner()];
    if crate::timing() { eprintln!("trim: {} of {} pieces kept, {} faces, in {:?}",report.pieces[0],report.pieces[1],out.faces.len(),clock.elapsed()) }
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
    // (within a region not a box, its box's corners outside it: the solid holds nothing there)
    let within = matches!(trim.reach,Some(Reach::Turned {..})).then_some(domain);
    super::closed::judge_within(&out,&swept,domain,None,within,&[],tol)?;
    if crate::timing() { eprintln!("trim: judged in {:?}",clock.elapsed()) }
    Ok((out,report))
}

/// The trim's open edges narrated, where `SOLVENT_TRIM_DEBUG` is set: each edge not used twice, any
/// other running along it, each face's loops and each edge standing off its face.
fn narrate_open(out: &Brep,tol: f64) {
    {
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
    {
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
