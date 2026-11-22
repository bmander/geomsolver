//! **The distance from a set of several conditions** (#183 F2): `distance(p, S)` where `S`'s body
//! states several things of its point — on two faces (an edge), on three (a corner), within two
//! faces' sides kept off them (a round's ball centres), on two surfaces (where they meet).  Read
//! from the set's terms (`model::Target::Set`), never from what the set is for:
//! - the points on several faces' patches are the boundary's own edges or vertices there, read
//!   off it exactly (`solid::faces`);
//! - any other conjunction of a face's side or surface, or a distance from a point, is the
//!   nearest point satisfying every condition, found by sequential projection onto the
//!   conditions linearised where the last step stood (each step a small quadratic program,
//!   solved exactly by its active set): a local nearest point, from the point measured;
//! - within (or beyond) every face of a body and nothing more, the body's own signed distance
//!   (`BodyFaces::signed_distance`), where each face's foot is its nearest point: no one surface's
//!   side bounds a body with a concave edge, and the projection onto the nearest face's side runs
//!   on past its patch at a convex corner.
use super::faces::{along_run, BodyFaces, Reading};
use crate::model::{RegionShape, RegionTerm, Sketch, Target};
use crate::space::{add, dot, norm, scale, sub};
use crate::vocab::Cmp;
use std::sync::Arc;

type V = [f64; 3];
/// A condition's value and gradient at a point.
type Fv = Arc<dyn Fn(V) -> (f64, V) + Send + Sync>;

/// One condition on the set's point: `f ≤ 0`, or `f = 0`; `convex` where `f` is known convex
/// (affine, for an equality), so the set of points keeping it is.
#[derive(Clone)]
pub struct Cond {
    f: Fv,
    eq: bool,
    convex: bool,
}

/// How a set's distance is read.
#[derive(Clone)]
pub enum SetDistance {
    /// The points on several faces' patches: edges two groups share, read off the boundary.
    Edges { body: Arc<BodyFaces>, edges: Vec<u32>, faces: Vec<usize> },
    /// Points: the vertices three groups share.
    Points(Vec<V>),
    /// One face's patch.
    Patch(Arc<Reading>),
    /// Anything else, as conditions.
    Conditions(Conds),
    /// The nearest of several: a set within a body's faces where the body is in pieces, each
    /// shell's conditions apart (within all of them at once is within none).
    Nearest(Vec<SetDistance>),
    /// The material within every face of a body, or (`beyond`) what is outside it.
    Body { body: Arc<BodyFaces>, beyond: bool },
    /// A drawn face's region read through its plane (#737), or (`beyond`) what is outside it: the
    /// distance in the plane from the point's foot to it.
    Section { at: crate::plane::Basis, region: Arc<crate::solid::field::PlanarField>, beyond: bool },
}

pub use super::faces::Read;

/// A set's conditions, and which they are: a number of their own, shared by their clones (the same
/// conditions), by which the last nearest point found on a thread is remembered (`nearest`).
#[derive(Clone)]
pub struct Conds {
    cs: Vec<Cond>,
    id: u64,
}

thread_local! {
    /// The last nearest point found on this thread: of which conditions (`Conds::id`), from which
    /// point (its bits), and the point found.
    static LAST: std::cell::Cell<Option<(u64, [u64; 3], Option<V>)>> = const { std::cell::Cell::new(None) };
}

impl Conds {
    fn new(cs: Vec<Cond>) -> Conds {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        Conds { cs, id: NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) }
    }

    /// The point keeping every condition nearest `p` (`nearest`).  A region's terms ask it twice
    /// from one point — `std.rounds`' `distance(p, centres) >= r` and `distance(p, rolled) <=
    /// distance(p, centres) + r`, which reads `rolled` from `centres`' nearest point (#994) — so
    /// the last answer on each thread is remembered against the conditions and the point, bit for
    /// bit: the same steps from the same start onto the same conditions, taken once.
    fn nearest(&self, p: V) -> Option<V> {
        let at = p.map(f64::to_bits);
        if let Some((id, from, q)) = LAST.get() {
            if id == self.id && from == at {
                return q;
            }
        }
        let q = nearest(&self.cs, p);
        LAST.set(Some((self.id, at, q)));
        q
    }
}

/// The face `f` names, as its patch (`F`, not `F.surface`).
fn patch(f: &crate::model::FaceRef) -> crate::model::FaceRef {
    crate::model::FaceRef { surface: false, ..f.clone() }
}

/// The side of the faces `r` reads, and its gradient: one for each surface where they lie on
/// several meeting convex and `split` is asked (`Reading::convex_pieces`), else the side of the
/// face whose patch is nearest.
fn sides(r: Reading, split: bool) -> Vec<Fv> {
    let r = Arc::new(r);
    match split.then(|| r.convex_pieces()).flatten() {
        Some(pieces) => pieces.into_iter().map(|fi| {
            let body = r.body.clone();
            Arc::new(move |x| body.side_grad(fi, x)) as Fv
        }).collect(),
        None => vec![Arc::new(move |x| r.side_grad(x))],
    }
}

/// **A smooth chain read face by face as well** (#994): one measure for each face of `r`'s chain —
/// a smooth chain of several faces, one chain (not pieces apart), read through its nearest face —
/// beside the chain's own reading, each under `d` only where the point is nearer than `d` to a
/// patch of the chain.  Every patch is of the chain's surface, so a point at least `d` off that
/// surface is at least `d` off each patch, and each of these says only what the distance from the
/// surface says — however the chain's own reading picks its face, at its rim too
/// (`BodyFaces::nearest_face`): the set is the same.  What they add is the shape of the offset
/// where the nearest face turns from one to another: where an arc's offset closes onto its axis — a
/// pocket's floor rounded as tight as its corner, in a pocket whose reflex corner keeps its chain
/// from bulging one way (`BodyFaces::bulging`); a rim rounded to the uprights' radius over a
/// block's outline with a tab, its sides, rounds and fills one chain — the centres' corner is the
/// two walls' offsets meeting on the axis, and a projection onto the nearest face's side alone
/// steps from one wall's to the other's, never onto both (`nearest`).  Each face reads:
/// - where the point's foot lies on its patch, its surface's side, the patch's distance;
/// - past a seam into a face of the chain bulging toward the ball on its own (`BodyFaces::bulges`:
///   a wall into a pocket's corner arc, the arc into the next wall), the plane tangent to that
///   next face where the seam's point nearest the point is (`BodyFaces::tangent_side`, turned by
///   `beyond` toward the side the point is on), where the point is nearer that face than `d` —
///   the chain read as a convex one is about the corner, so the step sees the wall beyond the arc
///   as well as the one the point is nearest; the next face's plane, not this one's, as they are
///   one where the seam is tangent, and where it is tangent only to the drawing's numbers (a turn
///   of a hair, #994's `rim_seams`) it is the side of the face the point stands beyond, so it
///   reads `d` where that face's patch does, and a centre on its offset is kept however its
///   distance rounds — and nothing (`d`, no gradient) where the point is not nearer than `d`: the
///   next face is no further from the point than this one's patch, so it says all this one would;
/// - past a seam into any other face of the chain (a reflex corner's arc or fill, whose tangent
///   plane runs on through the pocket or the tab beyond it), nothing: that face is nearer, and
///   says it;
/// - past any other edge, its side carried on where the point is nearer the patch than `d` (a
///   tangent plane through the patch's nearest point is no further than it), nothing where not.
/// A face far from the point holds with no gradient, no row of a step's program (`nearest`), so a
/// step chooses among the faces near the point.  Empty for one face, and for chains apart.
fn faces_apart(r: &Reading, c: f64, beyond: bool, d: f64) -> Vec<Fv> {
    if !r.smooth || r.faces.len() < 2 || !r.chains.is_empty() {
        return Vec::new();
    }
    let sigma = if beyond { 1.0 } else { -1.0 };
    // (looked up, not scanned, as each face's condition is read: a chain grows with a body, #994)
    let chain: Arc<std::collections::BTreeSet<usize>> = Arc::new(r.faces.iter().copied().collect());
    // the faces of the chain each bulging toward the ball on its own
    let toward: Arc<std::collections::BTreeSet<usize>> =
        Arc::new(r.faces.iter().copied().filter(|&g| r.body.bulges(g, beyond, true)).collect());
    r.faces.iter().map(|&fi| {
        let (body, chain, toward) = (r.body.clone(), chain.clone(), toward.clone());
        // the faces it goes on into across a seam, which a point near them reads through it
        let beside: Vec<usize> = r.body.edges_of(fi).iter().filter(|&&e| r.body.seam(e))
            .flat_map(|&e| r.body.faces_of(e).to_vec()).filter(|&h| h != fi && chain.contains(&h))
            .collect();
        Arc::new(move |x| {
            // far from its patch and from every face it goes on into, by their boxes, nothing:
            // where the set's point is, a face holding it back is within `d` (#994), so a chain
            // grown with a body is read only where the point is
            let nothing = (c * d, [0.0; 3]);
            let far = |h: usize| body.patch_gap(h, x) > 2.0 * d;
            if far(fi) && beside.iter().all(|&h| far(h)) {
                return nothing;
            }
            let (past, (s, g)) = body.carried_past(fi, x);
            let side = (c * sigma * s, scale(g, c * sigma));
            let Some((e, off, at)) = past else { return (c * s.abs(), scale(g, c * s.signum())) };
            let next = body.faces_of(e).iter().copied().find(|&h| h != fi && chain.contains(&h));
            match next {
                Some(h) if body.seam(e) => {
                    if !toward.contains(&h) || body.patch_distance(h, x) >= d {
                        return nothing;
                    }
                    // the next face's tangent plane there: this one's, where the seam is tangent,
                    // and where it is tangent only to the drawing's numbers (a turn of a hair), the
                    // side of the face the point stands beyond
                    let (s, g) = body.tangent_side(h, at, x);
                    (c * sigma * s, scale(g, c * sigma))
                }
                _ => if off < d { side } else { nothing },
            }
        }) as Fv
    }).collect()
}

/// The face a term reads, as a patch (`Some(false)`) or a surface (`Some(true)`) held to
/// nothing further: `q coincident F` / `q coincident F.surface`.
fn on_face(t: &RegionTerm, sk: &Sketch) -> Option<(crate::model::FaceRef, bool)> {
    let (f, c) = super::faces::measured_face(t)?;
    let at = t.lo.value(sk).ok()?;
    (c > 0.0 && t.cmp == Cmp::Le && at == 0.0 && t.grow.is_none()).then(|| (f.clone(), f.surface))
}

impl SetDistance {
    /// A set's distance, from its terms.
    pub fn of(sk: &Sketch, terms: &[RegionTerm], read: &mut Read) -> Result<SetDistance, String> {
        if terms.is_empty() {
            return Err("a set measured from states nothing of its point".to_string());
        }
        // on the patches of several faces: the boundary's own edges or corners there
        let patches: Option<Vec<crate::model::FaceRef>> = terms.iter()
            .map(|t| on_face(t, sk).filter(|(_, surface)| !surface).map(|(f, _)| f)).collect();
        if let Some(fs) = patches {
            // its edges or corners there: of one body (#961)
            let readings = super::faces::one_body(sk, &fs.iter().collect::<Vec<_>>(), read)?;
            let body = readings[0].body.clone();
            return Ok(match readings.as_slice() {
                [one] => SetDistance::Patch(Arc::new(Reading { body, faces: one.faces.clone(), smooth: one.smooth,
                    chains: one.chains.clone(), held: Vec::new() })),
                [a, b] => SetDistance::Edges {
                    edges: body.edges_between(&a.faces, &b.faces),
                    faces: a.faces.iter().chain(&b.faces).copied().collect(),
                    body,
                },
                _ => {
                    let groups: Vec<&[usize]> = readings.iter().map(|r| r.faces.as_slice()).collect();
                    SetDistance::Points(body.vertices_among(&groups).into_iter().map(|(v, _)| v).collect())
                }
            });
        }
        // within (or beyond) a drawn face's region, read through its plane (#737)
        if let [t @ RegionTerm { shape: RegionShape::Section { face }, hi: None, .. }] = terms {
            if t.cmp != Cmp::In && t.value(sk)? == (0.0, 0.0) {
                let (at, region, _) = crate::solid::field::section(sk, *face as usize)?;
                return Ok(SetDistance::Section { at, region: Arc::new(region), beyond: t.cmp == Cmp::Ge });
            }
        }
        // within (or beyond) every face of a body, and nothing more: its material, read by its
        // signed distance where that is exact
        if let [t @ RegionTerm { shape: RegionShape::FaceSide { face }, hi: None, .. }] = terms {
            if t.cmp != Cmp::In && t.value(sk)? == (0.0, 0.0) {
                let r = super::faces::reading(sk, &patch(face), read)?;
                if r.faces.len() == r.body.brep().0.faces.len() && r.body.distance_slack().is_some() {
                    return Ok(SetDistance::Body { body: r.body.clone(), beyond: t.cmp == Cmp::Ge });
                }
            }
        }
        // the faces the set's point is within: a distance off one of those in pieces is read
        // piece by piece (`Reading::convex_pieces`)
        let within: Vec<crate::model::FaceRef> = terms.iter().filter_map(|t| match &t.shape {
            RegionShape::FaceSide { face } if t.cmp == Cmp::Le => Some(patch(face)),
            _ => None,
        }).collect();
        // and the faces it is outside of (a concave edge's ball, outside both)
        let beyond: Vec<crate::model::FaceRef> = terms.iter().filter_map(|t| match &t.shape {
            RegionShape::FaceSide { face } if t.cmp == Cmp::Ge => Some(patch(face)),
            _ => None,
        }).collect();
        let sides_of = Sides { within: &within, beyond: &beyond };
        // within the faces of a body in pieces: the conditions of each shell apart, the nearest
        // read
        let mut pieces: Vec<Arc<BodyFaces>> = Vec::new();
        for f in &within {
            let r = super::faces::reading(sk, f, read)?;
            let (of, _) = r.body.shells();
            let apart = r.faces.first().is_some_and(|&f0| r.faces.iter().any(|&fi| of[fi] != of[f0]));
            if apart && !pieces.iter().any(|b| Arc::ptr_eq(b, &r.body)) {
                pieces.push(r.body.clone());
            }
        }
        if let [body] = &pieces[..] {
            let mut each = Vec::new();
            for k in 0..body.shells().1 {
                // a shell none of a term's faces is on holds none of the set
                let terms = terms.iter().map(|t| conditions(sk, t, read, &sides_of, Some((body, k))))
                    .collect::<Result<Vec<_>, String>>()?;
                if let Some(conds) = terms.into_iter().collect::<Option<Vec<_>>>() {
                    each.push(SetDistance::Conditions(Conds::new(conds.into_iter().flatten().collect())));
                }
            }
            return Ok(SetDistance::Nearest(each));
        }
        let mut conds = Vec::new();
        for t in terms {
            conds.extend(conditions(sk, t, read, &sides_of, None)?.expect("every term holds somewhere unsplit"));
        }
        Ok(SetDistance::Conditions(Conds::new(conds)))
    }

    /// The distance from the set (infinite from an empty one).
    pub fn distance(&self, p: V) -> f64 {
        match self {
            SetDistance::Edges { body, edges, .. } => body.nearest_edge(edges, p).map_or(f64::INFINITY, |e| e.2),
            SetDistance::Points(ps) => ps.iter().map(|q| norm(sub(p, *q))).fold(f64::INFINITY, f64::min),
            SetDistance::Patch(r) => r.distance(p, false),
            SetDistance::Conditions(cs) => cs.nearest(p).map_or(f64::INFINITY, |q| norm(sub(p, q))),
            SetDistance::Nearest(each) => each.iter().map(|s| s.distance(p)).fold(f64::INFINITY, f64::min),
            SetDistance::Body { body, beyond } => {
                let v = body.signed_distance(p).0;
                (if *beyond { -v } else { v }).max(0.0)
            }
            SetDistance::Section { at, region, beyond } => {
                let (x, y) = at.view_coords(p);
                let v = region.value([x, y]);
                (if *beyond { -v } else { v }).max(0.0)
            }
        }
    }

    /// The set's point nearest `p`, where it is read as conditions.
    pub fn nearest(&self, p: V) -> Option<V> {
        match self {
            SetDistance::Conditions(cs) => cs.nearest(p),
            SetDistance::Nearest(each) => each.iter().filter_map(|s| s.nearest(p))
                .min_by(|a, b| norm(sub(p, *a)).total_cmp(&norm(sub(p, *b)))),
            _ => None,
        }
    }
}

/// `sign · (m − k)`, and its gradient.
fn shift(m: Fv, k: f64, sign: f64) -> Fv {
    Arc::new(move |x| { let (v, g) = m(x); (sign * (v - k), scale(g, sign)) })
}

/// The faces a set says its point is within, and those it says it is outside of.
struct Sides<'a> {
    within: &'a [crate::model::FaceRef],
    beyond: &'a [crate::model::FaceRef],
}

/// A term of a set measured from, as conditions on its point, on the `sides` of the faces the set
/// says its point is on — the faces of `shell` alone where one is named (a body, and which of its
/// shells): `None` where the term's faces have none on it.
fn conditions(sk: &Sketch, t: &RegionTerm, read: &mut Read, sides_of: &Sides,
    shell: Option<(&Arc<BodyFaces>, usize)>) -> Result<Option<Vec<Cond>>, String> {
    let within = sides_of.within;
    // the faces a face reference names, kept to the shell
    let mut reading = |f: &crate::model::FaceRef| -> Result<Option<Reading>, String> {
        let r = super::faces::reading(sk, f, read)?;
        Ok(match shell {
            Some((body, k)) if Arc::ptr_eq(body, &r.body) => {
                let (of, _) = body.shells();
                let faces: Vec<usize> = r.faces.iter().copied().filter(|&fi| of[fi] == k).collect();
                (!faces.is_empty()).then_some(Reading { faces, ..r })
            }
            _ => Some(r),
        })
    };
    let (lo, grow) = t.value(sk)?;
    let hi = t.hi.unwrap_or(lo);
    // a measure `m` held by the term: `m ≤ lo`, `m ≥ lo`, `lo ≤ m ≤ hi` (`m = lo` where they meet)
    // `convex` where every condition made is: `m` affine, or convex held below a number (concave
    // held above one)
    let held_as = |m: Fv, convex: bool| -> Vec<Cond> {
        match t.cmp {
            Cmp::Le => vec![Cond { f: shift(m, lo + grow, 1.0), eq: false, convex }],
            Cmp::Ge => vec![Cond { f: shift(m, lo + grow, -1.0), eq: false, convex }],
            Cmp::In if hi == lo => vec![Cond { f: shift(m, lo, 1.0), eq: true, convex }],
            Cmp::In => vec![Cond { f: shift(m.clone(), hi, 1.0), eq: false, convex },
                            Cond { f: shift(m, lo, -1.0), eq: false, convex }],
        }
    };
    let held = |m: Fv| held_as(m, false);
    // the faces' sides carried on past them (`BodyFaces::carried_side_grad`), each `σ·side` held:
    // `σ` grows it into the side the point is on, where the side is that face's convex solid's, so
    // it is that solid's signed distance turned round there and the condition convex — one
    // condition each face
    let carried = |r: &Reading, past: bool, sigma: f64| -> Vec<Cond> {
        // a plane's side carried on past it is its own (`carried_side_grad`), so the faces of a
        // chain on one plane — a face and the caps a round leaves on its surface beside it, one
        // per corner of every pocket it is the top of — say one condition, not one each (#994)
        r.body.one_per_plane(&r.faces).into_iter().flat_map(|fi| {
            let body = r.body.clone();
            held_as(Arc::new(move |x| {
                let (s, g) = if past { body.carried_side_grad(fi, x) } else { body.side_grad(fi, x) };
                (sigma * s, scale(g, sigma))
            }), true)
        }).collect()
    };
    // which way the point stands of face `f`, where the set says: within (`Some(false)`) or
    // outside (`Some(true)`)
    let side_of = |f: &crate::model::FaceRef| -> Option<bool> {
        let f = patch(f);
        if sides_of.beyond.contains(&f) { Some(true) } else if within.contains(&f) { Some(false) } else { None }
    };
    match &t.shape {
        // within faces on several surfaces meeting convex: within each surface's side
        RegionShape::FaceSide { face } => {
            let Some(r) = reading(face)? else { return Ok(None) };
            // within (or outside) a face convex toward the point, or a smooth chain bulging that
            // way: each face's side, a chain's carried on past it
            if t.cmp != Cmp::In && t.hi.is_none() && r.convex_pieces().is_none() {
                if let Some(past) = r.bulging(t.cmp == Cmp::Ge) {
                    return Ok(Some(carried(&r, past, 1.0)));
                }
            }
            Ok(Some(sides(r, t.cmp == Cmp::Le).into_iter().flat_map(held).collect()))
        }
        RegionShape::Measured { of } => {
            let [(target, c)] = of.as_slice() else {
                return Err("a set measured from bounds one measurement at a time".to_string());
            };
            let c = *c;
            let m: Fv = match target {
                Target::Face(f) if f.surface => {
                    let Some(r) = reading(f)? else { return Ok(None) };
                    // on the surface: its side is nothing, a smooth condition
                    if t.cmp == Cmp::Le && lo == 0.0 && grow == 0.0 {
                        let r = Arc::new(r);
                        return Ok(Some(vec![Cond { f: Arc::new(move |x| r.side_grad(x)), eq: true, convex: false }]));
                    }
                    // at least a distance off faces on several surfaces meeting convex, within them:
                    // off each (outside them, another piece's surface runs on past the crease)
                    let split = t.cmp == Cmp::Ge && within.contains(&patch(f));
                    // within them by as far as their surface's radius: on what it closes onto
                    if split && c > 0.0 {
                        if let Some(on) = closed_onto(&r, (lo + grow) / c) {
                            return Ok(Some(on));
                        }
                    }
                    // at least a distance off a face convex toward the side the point is on, or a
                    // smooth chain bulging toward it: off each face's side, a chain's carried on
                    // past it (`BodyFaces::bulging`), each its own condition, so a projection sees
                    // every face meeting at a corner of the offset, and the steps stop on it
                    if t.cmp == Cmp::Ge && c > 0.0 && r.convex_pieces().is_none() {
                        if let Some(beyond) = side_of(f) {
                            if let Some(past) = r.bulging(beyond) {
                                return Ok(Some(carried(&r, past, if beyond { c } else { -c })));
                            }
                        }
                    }
                    // any other: off the side of the face whose patch is nearest — and, a smooth
                    // chain bulging no one way (a pocket's walls with a reflex corner), off each
                    // face's patch besides (`faces_apart`): what that reading says already, so the
                    // set is the same, but a projection sees each face meeting at a corner of the
                    // offset
                    let pieces = split && r.convex_pieces().is_some();
                    let apart = match side_of(f) {
                        Some(beyond) if t.cmp == Cmp::Ge && c > 0.0 && !pieces => {
                            faces_apart(&r, c, beyond, (lo + grow) / c)
                        }
                        _ => Vec::new(),
                    };
                    let mut conds: Vec<Cond> = sides(r, split).into_iter()
                        .flat_map(|side| held(Arc::new(move |x| {
                            let (s, g) = side(x);
                            (c * s.abs(), scale(g, c * s.signum()))
                        }))).collect();
                    conds.extend(apart.into_iter().flat_map(held));
                    return Ok(Some(conds));
                }
                Target::Ent(e) if e.kind == crate::model::EntKind::Point => {
                    let o = sk.world_point(e.i());
                    Arc::new(move |x| {
                        let d = sub(x, o);
                        let n = norm(d);
                        (c * n, if n > 0.0 { scale(d, c / n) } else { [0.0; 3] })
                    })
                }
                _ => return Err("a set measured from is on faces, within their sides or at \
                                 distances from their surfaces or from points"
                    .to_string()),
            };
            Ok(Some(held(m)))
        }
        // within the faces a run of edges ends on, as it runs: `σ·side ≤ 0` for each — of the run
        // nearest the point where the edges are in several (`EndRuns`): the `k`th condition the
        // `k`th face that run ends on, nothing where it ends on fewer
        RegionShape::Ends { edge } => {
            let Target::Set { terms: inner, .. } = edge.as_ref() else {
                return Err("where an edge ends is read of a set".to_string());
            };
            let runs = Arc::new(EndRuns::of(&SetDistance::of(sk, inner, read)?, None)?);
            let most = runs.runs.iter().map(|(_, sides)| sides.len()).max().unwrap_or(0);
            Ok(Some((0..most).map(|k| {
                let runs = runs.clone();
                Cond { convex: false, f: Arc::new(move |x| match runs.nearest(x).get(k) {
                    Some(&(c, sign)) => {
                        let (v, g) = runs.body.end_side(&c, x);
                        (sign * v, scale(g, sign))
                    }
                    None => (-1.0, [0.0; 3]),
                }), eq: false }
            }).collect()))
        }
        // a ball's, a cylinder's, a cone's or a half-space's: its measure's signed distance
        // from the surface its number makes, by differences
        shape => {
            use crate::solid::field::region::Term;
            let Some(measure) = super::region::measure_of(sk, shape)? else {
                return Err("a set measured from is on faces, within their sides or at distances \
                            from their surfaces, from points or from its own bounds".to_string());
            };
            let term = Arc::new(Term { measure, cmp: t.cmp, lo, hi: t.hi, grow });
            let numeric = |f: Arc<dyn Fn(V) -> f64 + Send + Sync>| -> Fv {
                Arc::new(move |x: V| {
                    let h = 1e-6 * (1.0 + norm(x));
                    let g: V = std::array::from_fn(|k| {
                        let (mut a, mut b) = (x, x);
                        a[k] += h;
                        b[k] -= h;
                        (f(a) - f(b)) / (2.0 * h)
                    });
                    (f(x), g)
                })
            };
            Ok(Some(if t.cmp == Cmp::In && hi == lo {
                let t2 = term.clone();
                vec![Cond { f: numeric(Arc::new(move |x| t2.from_surface(x, lo))), eq: true, convex: false }]
            } else {
                let t2 = term.clone();
                vec![Cond { f: numeric(Arc::new(move |x| t2.value(x))), eq: false, convex: false }]
            }))
        }
    }
}

/// **Within faces and at least their surface's radius off it** (`side ≤ −d`, `d` the radius of the
/// one surface the faces `r` reads lie on: `q inside F` with `q distance(>= d) F.surface`, a round's
/// ball centres): the surface offset inward by `d` closes onto what it is drawn about
/// (`Surface::offset`) — a cylinder its axis, a sphere its centre, a torus's tube its middle
/// circle — so the points within the faces by `d` are that curve or point alone, the centres of a
/// ball as wide as a cylinder rolled inside it.  Held to it as conditions with a gradient
/// everywhere on it: two planes through a line, three through a point, a plane and a cylinder
/// about a circle's axis — where the side's own turns over at the axis, and the nearest point's
/// steps toward it never settle.  `None` where the faces lie on several surfaces or the offset is
/// still a surface (or nothing).
fn closed_onto(r: &Reading, d: f64) -> Option<Vec<Cond>> {
    use crate::brep::geom::{Curve, Offset};
    let (b, mm, _) = r.body.brep();
    let first = &b.faces[*r.faces.first()?];
    // the side is the surface's signed distance, turned where the face is: within by `d` is the
    // surface's offset to `∓d` (in the boundary's millimetres)
    let level = if first.reversed { d } else { -d } * mm;
    let onto = first.surface.offset(level, r.body.tolerance() * mm)?;
    if matches!(onto, Offset::Surface(_)) || r.surfaces().0.len() != 1 {
        return None;
    }
    let plane = |o: V, n: V| Cond { f: Arc::new(move |x: V| (dot(n, sub(x, o)), n)), eq: true, convex: true };
    Some(match onto {
        Offset::Point(c) => {
            let o = r.body.back(c);
            vec![plane(o, [1.0, 0.0, 0.0]), plane(o, [0.0, 1.0, 0.0]), plane(o, [0.0, 0.0, 1.0])]
        }
        Offset::Curve(Curve::Line { p, d: a }) => {
            let (o, (e1, e2)) = (r.body.back(p), crate::space::across(a));
            vec![plane(o, e1), plane(o, e2)]
        }
        Offset::Curve(Curve::Circle(f, big)) => {
            let (o, a, big) = (r.body.back(f.o), f.z, big / mm);
            // in its plane, and as far from its axis as its radius
            let round = Cond {
                f: Arc::new(move |x: V| {
                    let w = sub(x, o);
                    let radial = sub(w, scale(a, dot(w, a)));
                    let rho = norm(radial);
                    (rho - big, if rho > 0.0 { scale(radial, 1.0 / rho) } else { crate::space::across(a).0 })
                }),
                eq: true,
                convex: false,
            };
            vec![plane(o, a), round]
        }
        _ => return None,
    })
}

/// The point satisfying every condition nearest `p`: from `p`, each step the nearest point of the
/// conditions linearised where the last one stood.  Where every condition is convex its
/// linearisation holds all of its points, so a step's point keeping every condition is the
/// nearest: the steps stop there.  (Their own fixed point is no steadier: a step onto a surface
/// curved tightly about a point far off — a ball's centres a hair short of closing onto a pocket
/// corner's axis, seen from the corner — moves along it by the distance over the radius times any
/// error in where the last step stood, and the steps wander round it.)  Where the steps do not
/// settle, the nearest point they stood on that keeps every condition (#994): a condition holding
/// with no gradient is no row of a step (`faces_apart`'s face whose patch the point is not near),
/// so a step from a point keeping it, its row gone, can land back where it fails, and the two
/// alternate — a ball's centres about a blend ended by a side an eye stands out past, its patch
/// beyond the blend's end holding a centre back only near it.  Such a point is of the set, and
/// its distance no less than the nearest's: a centre found, never a set read as empty.  And where
/// the conditions read where a step landed have no point in common, the step is taken back half
/// way, a few times, before the nearest point stood on is all there is.
fn nearest(cs: &[Cond], p: V) -> Option<V> {
    let tol = 1e-12 * (1.0 + norm(p));
    let kept = 1e-9 * (1.0 + norm(p));
    let bar = 1e-7 * (1.0 + norm(p));
    let convex = cs.iter().all(|c| c.convex);
    let mut x = p;
    let mut rows: Vec<(V, f64, bool)> = Vec::with_capacity(cs.len());
    // the nearest point stood on keeping every condition, and how far
    let mut kept_at: Option<(f64, V)> = None;
    let (mut last, mut halved) = (p, 0);
    for k in 0..80 {
        rows.clear();
        let mut off: f64 = 0.0;
        rows.extend(cs.iter().filter_map(|c| {
            let (v, g) = (c.f)(x);
            off = off.max(if c.eq { v.abs() } else { v.max(0.0) });
            // a · y ≤ b, the condition linearised at x — none where it holds with no gradient
            // (`faces_apart`'s faces far from the point), so the rows a step chooses among are
            // the conditions near it
            (c.eq || v > 0.0 || g != [0.0; 3]).then(|| (g, dot(g, x) - v, c.eq))
        }));
        // (every condition just read there keeps within `kept`, inside the final reading's bar)
        if convex && k > 0 && off <= kept {
            return Some(x);
        }
        if k > 0 && off <= bar {
            let d = norm(sub(x, p));
            if kept_at.is_none_or(|(e, _)| d < e) {
                kept_at = Some((d, x));
            }
        }
        // (where the conditions read where a step landed have no point in common — it stood
        // where a chain is read through another face, a hair inside a body its patches are of —
        // back half way toward where the last step stood, a few times)
        let Some(y) = project(p, &rows, tol) else {
            if halved < 8 && k > 0 {
                halved += 1;
                x = scale(add(x, last), 0.5);
                continue;
            }
            return kept_at.map(|(_, y)| y);
        };
        let step = norm(sub(y, x));
        last = x;
        x = y;
        if step <= tol {
            break;
        }
    }
    // only a point that keeps every condition: where the steps ended, or the nearest stood on
    let off = cs.iter().map(|c| {
        let v = (c.f)(x).0;
        if c.eq { v.abs() } else { v.max(0.0) }
    }).fold(0.0, f64::max);
    if off <= bar { Some(x) } else { kept_at.map(|(_, y)| y) }
}

/// The nearest point to `p` of `{y : a·y ≤ b}` (`= b` where marked), at most three rows active:
/// each choice of active rows solved exactly, the one keeping every row with non-negative
/// multipliers kept (the convex program's one answer).
fn project(p: V, rows: &[(V, f64, bool)], tol: f64) -> Option<V> {
    let m = rows.len();
    let eqs: Vec<usize> = (0..m).filter(|&i| rows[i].2).collect();
    let ineqs: Vec<usize> = (0..m).filter(|&i| !rows[i].2).collect();
    let mut best: Option<(f64, V)> = None;
    let mut fallback: Option<(f64, V)> = None;
    let mut choose = |active: &[usize]| {
        let Some((y, lambda)) = onto(p, rows, active) else { return };
        let keeps = rows.iter().all(|(a, b, eq)| {
            let v = dot(*a, y) - b;
            if *eq { v.abs() <= tol * (1.0 + norm(*a)) * 1e3 } else { v <= tol * (1.0 + norm(*a)) * 1e3 }
        });
        if !keeps {
            return;
        }
        let d = norm(sub(y, p));
        let signs = active.iter().zip(&lambda).all(|(&i, &l)| rows[i].2 || l >= -1e-12);
        let slot = if signs { &mut best } else { &mut fallback };
        if slot.is_none_or(|(e, _)| d < e) {
            *slot = Some((d, y));
        }
    };
    if eqs.len() > 3 {
        return None;
    }
    // every choice of at most three rows: every equality, and inequalities beside them, each
    // set of them once — among all of them, so a row is never left out of the choice (held on the
    // stack: this runs thousands of times a field query)
    let free = 3 - eqs.len();
    let mut active = [0usize; 3];
    active[..eqs.len()].copy_from_slice(&eqs);
    let k = eqs.len();
    choose(&active[..k]);
    // (as many of them as are free to stand beside the equalities)
    let upto = |f: usize| if free >= f { ineqs.len() } else { 0 };
    for (i, &first) in ineqs.iter().enumerate().take(upto(1)) {
        active[k] = first;
        choose(&active[..k + 1]);
        for (j, &second) in ineqs.iter().enumerate().skip(i + 1).take(upto(2)) {
            active[k + 1] = second;
            choose(&active[..k + 2]);
            for &third in ineqs.iter().skip(j + 1).take(upto(3)) {
                active[k + 2] = third;
                choose(&active[..k + 3]);
            }
        }
    }
    best.or(fallback).map(|(_, y)| y)
}

/// `p` projected onto the rows `active` (at most three) held as equalities: the point and its
/// multipliers.
fn onto(p: V, rows: &[(V, f64, bool)], active: &[usize]) -> Option<(V, [f64; 3])> {
    let k = active.len();
    if k == 0 {
        return Some((p, [0.0; 3]));
    }
    let a: [V; 3] = std::array::from_fn(|i| if i < k { rows[active[i]].0 } else { [0.0; 3] });
    let r: [f64; 3] = std::array::from_fn(|i| if i < k { dot(rows[active[i]].0, p) - rows[active[i]].1 } else { 0.0 });
    // the Gram matrix G λ = r
    let mut g = [[0.0; 3]; 3];
    for i in 0..k {
        for j in 0..k {
            g[i][j] = dot(a[i], a[j]);
        }
    }
    let lambda = solve(&g, &r, k)?;
    let mut y = p;
    for i in 0..k {
        y = sub(y, scale(a[i], lambda[i]));
    }
    Some((y, lambda))
}

/// A small symmetric system by elimination with partial pivoting; `None` where singular.
fn solve(g: &[[f64; 3]; 3], r: &[f64; 3], k: usize) -> Option<[f64; 3]> {
    let mut m = [[0.0; 4]; 3];
    let size = (0..k).map(|i| g[i][i].abs()).fold(0.0, f64::max).max(1e-300);
    for i in 0..k {
        for j in 0..k {
            m[i][j] = g[i][j];
        }
        m[i][3] = r[i];
    }
    for c in 0..k {
        let piv = (c..k).max_by(|&a, &b| m[a][c].abs().total_cmp(&m[b][c].abs()))?;
        if m[piv][c].abs() <= 1e-12 * size {
            return None;
        }
        m.swap(c, piv);
        for i in 0..k {
            if i != c {
                let f = m[i][c] / m[c][c];
                for j in c..4 {
                    m[i][j] -= f * m[c][j];
                }
            }
        }
    }
    Some(std::array::from_fn(|i| if i < k { m[i][3] / m[i][i] } else { 0.0 }))
}

/// **The rest of a meeting** (#183 F2): `distance(p, E) <= distance(p, M)`, `E` the points on
/// faces' patches and `M` on their surfaces, holds where `p`'s nearest point of `M` is on `E` —
/// everywhere there `distance(p, M)` *is* `distance(p, E)`, so the difference is nothing where it
/// holds, and no field reads a region from that.  The same set is where `distance(p, E)` is at
/// most the distance from the rest of `M`, its points off `E` (closed): where `E` ends, and `M`'s
/// nearest point when that is off `E`.  So a sum `c·distance(p, E) − c·distance(p, M)` is read as
/// `c·distance(p, E) − c·distance(p, rest)`, negative inside: the same points.  `M` may be cut
/// where an edge ends (`q inside E.ends`): its nearest point is then the nearest within the cut,
/// and an end of `E` is the rest's only where `M` carries on past it within the cut.
pub struct Rest {
    pub meeting: SetDistance,
    pub edge: SetDistance,
    pub ends: Vec<V>,
}

impl Rest {
    /// The rest of meeting `m` past edge `e`, where `e`'s terms are on the patches of the faces
    /// `m`'s are on the surfaces of (and `m`'s others say where an edge ends); `None` otherwise.
    /// `edge` and `meeting` are the two sets' distances, read already.
    pub fn of(sk: &Sketch, e: &[RegionTerm], m: &[RegionTerm], edge: &SetDistance,
              meeting: &SetDistance, read: &mut Read) -> Option<Rest> {
        let fe = e.iter().map(|t| on_face(t, sk).filter(|(_, s)| !s).map(|(f, _)| patch(&f)))
            .collect::<Option<Vec<_>>>()?;
        // `m` on surfaces, and cut only where an edge ends: a cut each run lies within by how its
        // sides are chosen (`end_faces`), so `E` stays within `M`
        let (mut fm, mut within) = (Vec::new(), Vec::new());
        for t in m {
            match on_face(t, sk) {
                Some((f, true)) => fm.push(patch(&f)),
                None if matches!(t.shape, RegionShape::Ends { .. }) => {
                    within.extend(conditions(sk, t, read, &Sides { within: &[], beyond: &[] }, None).ok()??)
                }
                _ => return None,
            }
        }
        if fe.len() < 2 || fe.len() != fm.len() || !fe.iter().all(|f| fm.contains(f)) {
            return None;
        }
        let (edge, meeting) = (edge.clone(), meeting.clone());
        let ends = match &edge {
            SetDistance::Edges { body, edges, .. } => {
                body.run_ends(edges).into_iter().filter(|(v, d, _)| {
                    // where the meeting carries on past the end within the cut
                    within.iter().all(|c| (c.f)(along_run(*v, *d, -1.0)).0 <= 0.0)
                }).map(|(v, _, _)| v).collect()
            }
            _ => Vec::new(),
        };
        Some(Rest { meeting, edge, ends })
    }

    /// The distance from the meeting's points off the edge.
    pub fn distance(&self, p: V) -> f64 {
        let mut best = self.ends.iter().map(|v| norm(sub(p, *v))).fold(f64::INFINITY, f64::min);
        if let Some(q) = self.meeting.nearest(p) {
            let off = self.edge.distance(q);
            if off > 1e-7 * (1.0 + norm(q)) {
                best = best.min(norm(sub(p, q)));
            }
        }
        best
    }
}

/// **Where an edge ends** (#183 F7): for each end of the run of edges a set on two faces is,
/// each other face at that end, and which side of it the edge runs on (`-1` its material
/// side): the region within all of them, `max σ·side`.  A closed run ends nowhere.  Where the
/// edges are in several runs (#881: a crease a slot cuts across), a point is within the ends of
/// the run nearest it (`EndRuns`).
pub fn ends(set: &SetDistance, reach: Option<f64>) -> Result<impl Fn(V) -> f64 + Send + Sync, String> {
    let runs = EndRuns::of(set, reach)?;
    Ok(move |p: V| {
        runs.nearest(p).iter().map(|&(c, sign)| sign * runs.body.end_side(&c, p).0)
            .fold(f64::NEG_INFINITY, f64::max)
    })
}

/// **Where each run of a set's edges ends** (#881): the faces each run (`BodyFaces::runs`) ends
/// on, each with the side the run is on (`-1` its material side), a closed run's none.  A crease
/// a slot cuts across is in runs, and the runs on either side of the slot end on its sides,
/// facing each other: within every run's ends at once is within none, and within any run's is
/// too much — a run ending on a post's side goes on past the next post.  A point is read against
/// the run nearest it, the run whose edge the round beside it is built along (`lowering`'s
/// pieces, a run each, each carried on past its own ends and cut off there); where the runs end
/// alike (on the posts' sides, kept outside every post), the same as within all their ends.
struct EndRuns {
    body: Arc<BodyFaces>,
    /// Each run's edges and the faces it ends on (`BodyFaces::end_runs`).
    runs: Vec<(Vec<u32>, Vec<(super::faces::EndBy, f64)>)>,
}

impl EndRuns {
    fn of(set: &SetDistance, reach: Option<f64>) -> Result<EndRuns, String> {
        let SetDistance::Edges { body, edges, faces } = set else {
            return Err("where a set ends is read of an edge: a set on two faces".to_string());
        };
        Ok(EndRuns { body: body.clone(), runs: body.end_runs(edges, faces, reach) })
    }

    /// The faces the run nearest `p` ends on, and the side it runs on of each (none where the set
    /// has no edge; the first of runs equally near).
    fn nearest(&self, p: V) -> &[(super::faces::EndBy, f64)] {
        match &self.runs[..] {
            [] => &[],
            [(_, one)] => one,
            runs => runs.iter()
                .filter_map(|(es, sides)| Some((self.body.nearest_edge(es, p)?.2, sides)))
                .min_by(|a, b| a.0.total_cmp(&b.0)).map_or(&[], |(_, sides)| sides),
        }
    }
}
