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
//! - within every face of a body with a concave edge, nothing within the material and else the
//!   nearest of its faces' patches (`Reading::material`), where no one surface's side bounds it.
use super::faces::{along_run, BodyFaces, Reading};
use crate::model::{RegionShape, RegionTerm, Sketch, Target};
use crate::space::{dot, norm, scale, sub};
use crate::vocab::Cmp;
use std::sync::Arc;

type V = [f64; 3];
/// A condition's value and gradient at a point.
type Fv = Arc<dyn Fn(V) -> (f64, V) + Send + Sync>;

/// One condition on the set's point: `f ≤ 0`, or `f = 0`.
#[derive(Clone)]
pub struct Cond {
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
    Conditions(Vec<Cond>),
    /// The nearest of several: a set within a body's faces where the body is in pieces, each
    /// shell's conditions apart (within all of them at once is within none).
    Nearest(Vec<SetDistance>),
    /// The material within every face of a body with a concave edge (`Reading::material`).
    Material(Arc<Reading>),
}

pub use super::faces::Read;

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
            let readings = fs.iter().map(|f| super::faces::reading(sk, f, read))
                .collect::<Result<Vec<_>, String>>()?;
            if readings.iter().any(|r| !Arc::ptr_eq(&r.body, &readings[0].body)) {
                return Err("the faces a set is on are one body's".to_string());
            }
            let body = readings[0].body.clone();
            return Ok(match readings.as_slice() {
                [one] => SetDistance::Patch(Arc::new(Reading { body, faces: one.faces.clone(), smooth: one.smooth })),
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
        // within every face of a body with a concave edge, and nothing more: its material
        if let [t] = terms {
            if let RegionShape::FaceSide { face } = &t.shape {
                if t.cmp == Cmp::Le && t.hi.is_none() && t.value(sk)? == (0.0, 0.0) {
                    let r = super::faces::reading(sk, &patch(face), read)?;
                    if r.material() {
                        return Ok(SetDistance::Material(Arc::new(r)));
                    }
                }
            }
        }
        // the faces the set's point is within: a distance off one of those in pieces is read
        // piece by piece (`Reading::convex_pieces`)
        let within: Vec<crate::model::FaceRef> = terms.iter().filter_map(|t| match &t.shape {
            RegionShape::FaceSide { face } if t.cmp == Cmp::Le => Some(patch(face)),
            _ => None,
        }).collect();
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
                let terms = terms.iter().map(|t| conditions(sk, t, read, &within, Some((body, k))))
                    .collect::<Result<Vec<_>, String>>()?;
                if let Some(conds) = terms.into_iter().collect::<Option<Vec<_>>>() {
                    each.push(SetDistance::Conditions(conds.into_iter().flatten().collect()));
                }
            }
            return Ok(SetDistance::Nearest(each));
        }
        let mut conds = Vec::new();
        for t in terms {
            conds.extend(conditions(sk, t, read, &within, None)?.expect("every term holds somewhere unsplit"));
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
            SetDistance::Nearest(each) => each.iter().map(|s| s.distance(p)).fold(f64::INFINITY, f64::min),
            SetDistance::Material(r) => r.material_distance(p),
        }
    }

    /// The set's point nearest `p`, where it is read as conditions.
    pub fn nearest(&self, p: V) -> Option<V> {
        match self {
            SetDistance::Conditions(cs) => nearest(cs, p),
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

/// A term of a set measured from, as conditions on its point, `within` the faces the set says
/// its point is within — the faces of `shell` alone where one is named (a body, and which of its
/// shells): `None` where the term's faces have none on it.
fn conditions(sk: &Sketch, t: &RegionTerm, read: &mut Read, within: &[crate::model::FaceRef],
    shell: Option<(&Arc<BodyFaces>, usize)>) -> Result<Option<Vec<Cond>>, String> {
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
    let held = |m: Fv| -> Vec<Cond> {
        match t.cmp {
            Cmp::Le => vec![Cond { f: shift(m, lo + grow, 1.0), eq: false }],
            Cmp::Ge => vec![Cond { f: shift(m, lo + grow, -1.0), eq: false }],
            Cmp::In if hi == lo => vec![Cond { f: shift(m, lo, 1.0), eq: true }],
            Cmp::In => vec![Cond { f: shift(m.clone(), hi, 1.0), eq: false },
                            Cond { f: shift(m, lo, -1.0), eq: false }],
        }
    };
    match &t.shape {
        // within faces on several surfaces meeting convex: within each surface's side
        RegionShape::FaceSide { face } => {
            let Some(r) = reading(face)? else { return Ok(None) };
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
                        return Ok(Some(vec![Cond { f: Arc::new(move |x| r.side_grad(x)), eq: true }]));
                    }
                    // at least a distance off faces on several surfaces meeting convex, within them:
                    // off each (outside them, another piece's surface runs on past the crease)
                    let split = t.cmp == Cmp::Ge && within.contains(&patch(f));
                    return Ok(Some(sides(r, split).into_iter().flat_map(|side| held(Arc::new(move |x| {
                        let (s, g) = side(x);
                        (c * s.abs(), scale(g, c * s.signum()))
                    }))).collect()));
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
        // within the faces a run of edges ends on, as it runs: `σ·side ≤ 0` for each
        RegionShape::Ends { edge } => {
            let Target::Set { terms: inner, .. } = edge.as_ref() else {
                return Err("where an edge ends is read of a set".to_string());
            };
            let (body, sides) = end_sides(&SetDistance::of(sk, inner, read)?)?;
            Ok(Some(sides.into_iter().map(|(c, sign)| {
                let body = body.clone();
                Cond { f: shift(Arc::new(move |x| body.side_grad(c, x)), 0.0, sign), eq: false }
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
                vec![Cond { f: numeric(Arc::new(move |x| t2.from_surface(x, lo))), eq: true }]
            } else {
                let t2 = term.clone();
                vec![Cond { f: numeric(Arc::new(move |x| t2.value(x))), eq: false }]
            }))
        }
    }
}

/// The point satisfying every condition nearest `p`: from `p`, each step the nearest point of the
/// conditions linearised where the last one stood.
fn nearest(cs: &[Cond], p: V) -> Option<V> {
    let tol = 1e-12 * (1.0 + norm(p));
    let mut x = p;
    let mut rows: Vec<(V, f64, bool)> = Vec::with_capacity(cs.len());
    for _ in 0..80 {
        rows.clear();
        rows.extend(cs.iter().map(|c| {
            let (v, g) = (c.f)(x);
            // a · y ≤ b, the condition linearised at x
            (g, dot(g, x) - v, c.eq)
        }));
        let y = project(p, &rows, tol)?;
        let step = norm(sub(y, x));
        x = y;
        if step <= tol {
            break;
        }
    }
    // only a point that keeps every condition
    let off = cs.iter().map(|c| {
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
    // every subset of the inequalities, beside every equality (at most three rows in all, held on
    // the stack: this runs thousands of times a field query)
    let n = ineqs.len();
    let mut active = [0usize; 3];
    active[..eqs.len()].copy_from_slice(&eqs);
    for mask in 0u32..(1 << n.min(10)) {
        if (mask.count_ones() as usize) + eqs.len() > 3 {
            continue;
        }
        let mut k = eqs.len();
        for (bit, &i) in ineqs.iter().enumerate().take(10) {
            if mask >> bit & 1 == 1 {
                active[k] = i;
                k += 1;
            }
        }
        choose(&active[..k]);
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
        // sides are chosen (`end_sides`), so `E` stays within `M`
        let (mut fm, mut within) = (Vec::new(), Vec::new());
        for t in m {
            match on_face(t, sk) {
                Some((f, true)) => fm.push(patch(&f)),
                None if matches!(t.shape, RegionShape::Ends { .. }) => {
                    within.extend(conditions(sk, t, read, &[], None).ok()??)
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
/// side): the region within all of them, `max σ·side`.  A closed run ends nowhere.
pub fn ends(set: &SetDistance) -> Result<impl Fn(V) -> f64 + Send + Sync, String> {
    let (body, sides) = end_sides(set)?;
    Ok(move |p: V| {
        sides.iter().map(|&(c, sign)| sign * body.side(c, p)).fold(f64::NEG_INFINITY, f64::max)
    })
}

/// The faces a run of edges ends on, each with the side the run is on (`-1` its material side).
fn end_sides(set: &SetDistance) -> Result<(Arc<BodyFaces>, Vec<(usize, f64)>), String> {
    let SetDistance::Edges { body, edges, faces } = set else {
        return Err("where a set ends is read of an edge: a set on two faces".to_string());
    };
    Ok((body.clone(), body.end_sides(edges, faces)))
}
