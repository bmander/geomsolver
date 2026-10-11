//! **The distance from a set of several conditions** (#183 F2): `distance(p, S)` where `S`'s body
//! states several things of its point — on two faces (an edge), on three (a corner), within two
//! faces' sides kept off them (a round's ball centres), on two surfaces (where they meet).  Read
//! from the set's terms (`model::Target::Set`), never from what the set is for:
//! - the points on several faces' patches are the boundary's own edges or vertices there, read
//!   off it exactly (`solid::faces`);
//! - any other conjunction of a face's side or surface, or a distance from a point, is the
//!   nearest point satisfying every condition, found by sequential projection onto the
//!   conditions linearised where the last step stood (each step a small quadratic program,
//!   solved exactly by its active set): a local nearest point, from the point measured.
use super::faces::{BodyFaces, Reading};
use crate::model::{RegionShape, RegionTerm, Sketch, Target, TermNumber};
use crate::space::{add, dot, norm, scale, sub};
use crate::syntax::Cmp;
use std::sync::Arc;

type V = [f64; 3];
/// A condition's value and gradient at a point.
type Fv = Arc<dyn Fn(V) -> (f64, V) + Send + Sync>;

/// One condition on the set's point: `f ≤ 0`, or `f = 0`.
#[derive(Clone)]
struct Cond {
    f: Fv,
    eq: bool,
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
    Conditions(Vec<CondPub>),
}

/// A condition, as the field holds it.
#[derive(Clone)]
pub struct CondPub(Cond);

/// The bodies already read, each once.
pub type Read = Vec<(u32, Arc<BodyFaces>)>;

fn number(n: &TermNumber, sk: &Sketch) -> Result<f64, String> { n.value(sk) }

/// The face a term reads, as a patch (`Some(false)`) or a surface (`Some(true)`) held to
/// nothing further: `q coincident F` / `q coincident F.surface`.
fn on_face(t: &RegionTerm, sk: &Sketch) -> Option<(crate::model::FaceRef, bool)> {
    let RegionShape::Measured { of } = &t.shape else { return None };
    let [(Target::Face(f), c)] = of.as_slice() else { return None };
    let at = t.lo.value(sk).ok()?;
    (*c > 0.0 && t.cmp == Cmp::Le && at == 0.0 && t.grow.is_none()).then(|| (f.clone(), f.surface))
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
            let readings = fs.iter().map(|f| super::faces::reading(sk, f, read))
                .collect::<Result<Vec<_>, String>>()?;
            if readings.iter().any(|r| !Arc::ptr_eq(&r.body, &readings[0].body)) {
                return Err("the faces a set is on are one body's".to_string());
            }
            let body = readings[0].body.clone();
            return Ok(match readings.as_slice() {
                [one] => SetDistance::Patch(Arc::new(Reading { body, faces: one.faces.clone() })),
                [a, b] => SetDistance::Edges {
                    edges: body.edges_between(&a.faces, &b.faces),
                    faces: a.faces.iter().chain(&b.faces).copied().collect(),
                    body,
                },
                _ => {
                    let groups: Vec<&[usize]> = readings.iter().map(|r| r.faces.as_slice()).collect();
                    SetDistance::Points(body.vertices_among(&groups))
                }
            });
        }
        let mut conds = Vec::new();
        for t in terms {
            conds.extend(conditions(sk, t, read)?.into_iter().map(CondPub));
        }
        Ok(SetDistance::Conditions(conds))
    }

    /// The distance from the set (infinite from an empty one).
    pub fn distance(&self, p: V) -> f64 {
        match self {
            SetDistance::Edges { body, edges, .. } => body.nearest_edge(edges, p).map_or(f64::INFINITY, |e| e.2),
            SetDistance::Points(ps) => ps.iter().map(|q| norm(sub(p, *q))).fold(f64::INFINITY, f64::min),
            SetDistance::Patch(r) => r.distance(p, false),
            SetDistance::Conditions(cs) => nearest(cs, p).map_or(f64::INFINITY, |q| norm(sub(p, q))),
        }
    }

    /// The set's point nearest `p`, where it is read as conditions.
    pub fn nearest(&self, p: V) -> Option<V> {
        match self {
            SetDistance::Conditions(cs) => nearest(cs, p),
            _ => None,
        }
    }
}

/// A term of a set measured from, as conditions on its point.
fn conditions(sk: &Sketch, t: &RegionTerm, read: &mut Read) -> Result<Vec<Cond>, String> {
    let lo = number(&t.lo, sk)?;
    let grow = t.grow.as_ref().map_or(Ok(0.0), |g| number(g, sk))?;
    let hi = t.hi.unwrap_or(lo);
    // a measure `m` held by the term: `m ≤ lo`, `m ≥ lo`, `lo ≤ m ≤ hi` (`m = lo` where they meet)
    let held = |m: Fv| -> Vec<Cond> {
        let shift = |m: Fv, k: f64, sign: f64| -> Fv {
            Arc::new(move |x| { let (v, g) = m(x); (sign * (v - k), scale(g, sign)) })
        };
        match t.cmp {
            Cmp::Le => vec![Cond { f: shift(m, lo + grow, 1.0), eq: false }],
            Cmp::Ge => vec![Cond { f: shift(m, lo + grow, -1.0), eq: false }],
            Cmp::In if hi == lo => vec![Cond { f: shift(m, lo, 1.0), eq: true }],
            Cmp::In => vec![Cond { f: shift(m.clone(), hi, 1.0), eq: false },
                            Cond { f: shift(m, lo, -1.0), eq: false }],
        }
    };
    match &t.shape {
        RegionShape::FaceSide { face } => {
            let r = Arc::new(super::faces::reading(sk, face, read)?);
            Ok(held(Arc::new(move |x| r.side_grad(x))))
        }
        RegionShape::Measured { of } => {
            let [(target, c)] = of.as_slice() else {
                return Err("a set measured from bounds one measurement at a time".to_string());
            };
            let c = *c;
            let m: Fv = match target {
                Target::Face(f) if f.surface => {
                    let r = Arc::new(super::faces::reading(sk, f, read)?);
                    // on the surface: its side is nothing, a smooth condition
                    if t.cmp == Cmp::Le && lo == 0.0 && grow == 0.0 {
                        return Ok(vec![Cond { f: Arc::new(move |x| r.side_grad(x)), eq: true }]);
                    }
                    Arc::new(move |x| {
                        let (s, g) = r.side_grad(x);
                        (c * s.abs(), scale(g, c * s.signum()))
                    })
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
            Ok(held(m))
        }
        _ => Err("a set measured from is on faces, within their sides or at distances from \
                  their surfaces or from points"
            .to_string()),
    }
}

/// The point satisfying every condition nearest `p`: from `p`, each step the nearest point of the
/// conditions linearised where the last one stood.
fn nearest(cs: &[CondPub], p: V) -> Option<V> {
    let tol = 1e-12 * (1.0 + norm(p));
    let mut x = p;
    for _ in 0..80 {
        let rows: Vec<(V, f64, bool)> = cs.iter().map(|CondPub(c)| {
            let (v, g) = (c.f)(x);
            // a · y ≤ b, the condition linearised at x
            (g, dot(g, x) - v, c.eq)
        }).collect();
        let y = project(p, &rows, tol)?;
        let step = norm(sub(y, x));
        x = y;
        if step <= tol {
            break;
        }
    }
    // only a point that keeps every condition
    let off = cs.iter().map(|CondPub(c)| {
        let v = (c.f)(x).0;
        if c.eq { v.abs() } else { v.max(0.0) }
    }).fold(0.0, f64::max);
    (off <= 1e-7 * (1.0 + norm(p))).then_some(x)
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
        if active.len() > 3 {
            return;
        }
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
    // every subset of the inequalities, beside every equality
    let n = ineqs.len();
    for mask in 0u32..(1 << n.min(10)) {
        if (mask.count_ones() as usize) + eqs.len() > 3 {
            continue;
        }
        let mut active = eqs.clone();
        active.extend((0..n).filter(|k| mask >> k & 1 == 1).map(|k| ineqs[k]));
        choose(&active);
    }
    best.or(fallback).map(|(_, y)| y)
}

/// `p` projected onto the rows `active` held as equalities: the point and its multipliers.
fn onto(p: V, rows: &[(V, f64, bool)], active: &[usize]) -> Option<(V, Vec<f64>)> {
    let k = active.len();
    if k == 0 {
        return Some((p, Vec::new()));
    }
    let a: Vec<V> = active.iter().map(|&i| rows[i].0).collect();
    let r: Vec<f64> = active.iter().map(|&i| dot(rows[i].0, p) - rows[i].1).collect();
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
fn solve(g: &[[f64; 3]; 3], r: &[f64], k: usize) -> Option<Vec<f64>> {
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
    Some((0..k).map(|i| m[i][3] / m[i][i]).collect())
}

/// **The rest of a meeting** (#183 F2): `distance(p, E) <= distance(p, M)`, `E` the points on
/// faces' patches and `M` on their surfaces, holds where `p`'s nearest point of `M` is on `E` —
/// everywhere there `distance(p, M)` *is* `distance(p, E)`, so the difference is nothing where it
/// holds, and no field reads a region from that.  The same set is where `distance(p, E)` is at
/// most the distance from the rest of `M`, its points off `E` (closed): where `E` ends, and `M`'s
/// nearest point when that is off `E`.  So a sum `c·distance(p, E) − c·distance(p, M)` is read as
/// `c·distance(p, E) − c·distance(p, rest)`, negative inside: the same points.
pub struct Rest {
    pub meeting: SetDistance,
    pub edge: SetDistance,
    pub ends: Vec<V>,
}

impl Rest {
    /// The rest of meeting `m` past edge `e`, where `e`'s terms are on the patches of the faces
    /// `m`'s are on the surfaces of; `None` otherwise.
    pub fn of(sk: &Sketch, e: &[RegionTerm], m: &[RegionTerm], read: &mut Read) -> Option<Rest> {
        let faces = |ts: &[RegionTerm], surface: bool| -> Option<Vec<crate::model::FaceRef>> {
            ts.iter().map(|t| on_face(t, sk).filter(|(_, s)| *s == surface).map(|(f, _)| {
                crate::model::FaceRef { surface: false, ..f }
            })).collect()
        };
        let (fe, fm) = (faces(e, false)?, faces(m, true)?);
        if fe.len() < 2 || fe.len() != fm.len() || !fe.iter().all(|f| fm.contains(f)) {
            return None;
        }
        let edge = SetDistance::of(sk, e, read).ok()?;
        let meeting = SetDistance::of(sk, m, read).ok()?;
        let ends = match &edge {
            SetDistance::Edges { body, edges, .. } => body.run_ends(edges).into_iter().map(|(v, _, _)| v).collect(),
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
/// side): the region within all of them, `max σ·side`.  A closed run ends nowhere.
pub fn ends(set: &SetDistance) -> Result<impl Fn(V) -> f64 + Send + Sync, String> {
    let SetDistance::Edges { body, edges, faces } = set else {
        return Err("where a set ends is read of an edge: a set on two faces".to_string());
    };
    let mut sides: Vec<(usize, f64)> = Vec::new();
    for (v, d, vid) in body.run_ends(edges) {
        for c in body.faces_at(vid, faces) {
            let step = 1e-6 * (1.0 + norm(v));
            let s = body.side(c, add(v, scale(d, step)));
            sides.push((c, if s <= 0.0 { 1.0 } else { -1.0 }));
        }
    }
    let body = body.clone();
    Ok(move |p: V| sides.iter().map(|&(c, sign)| sign * body.side(c, p)).fold(f64::NEG_INFINITY, f64::max))
}
