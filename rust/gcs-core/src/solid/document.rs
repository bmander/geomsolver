//! Document dependencies, cache reads and resolved CSG terms.
use super::*;

// -- resolving a document's solids into terms ---------------------------------------------------

/// **Every number a solid was built from**, in one flat vector — the memo key.
///
/// `curve_polyline`'s bargain: a result is remembered against what it reads rather than
/// invalidated by whoever writes, so a repaint over a drawing that has not changed costs a
/// comparison.  Every scalar of every edge of every face the term reaches, each plane's pose,
/// basis and origin, and every extent.
pub fn reads(sk: &Sketch, si: usize, unit: f64) -> Vec<f64> {
    let mut v = vec![unit];
    let mut seen = std::collections::BTreeSet::new();
    let mut stack = vec![si as u32];
    while let Some(s) = stack.pop() {
        if !seen.insert(s) {
            continue;
        }
        let Some(sol) = sk.solids.get(s as usize) else { continue };
        v.push(s as f64);
        name_read(&sol.name, &mut v);
        match &sol.def {
            SolidDef::Loft { face, end, guide } => {
                v.extend([4.0, end.map_or(-1.0, |f| f as f64)]);
                face_reads(sk, *face, &mut v);
                if let Some(f) = end { face_reads(sk, *f, &mut v); }
                loft::reads(sk, *guide, &mut v);
            }
            SolidDef::Prism { face, from, to } => {
                v.push(0.0);
                v.push(from.value);
                v.push(to.value);
                face_reads(sk, *face, &mut v);
            }
            SolidDef::Through { face, body } => {
                v.extend([3.0, *body as f64]);
                face_reads(sk, *face, &mut v);
                // Read the material graph too, including changes to additions and placement.
                stack.push(*body);
            }
            SolidDef::Revolve { face, axis, sweep, sense } => {
                v.extend([1.0, *axis as f64]);
                v.push(sweep.value);
                v.push(if *sense == Sense::Cw { -1.0 } else { 1.0 });
                face_reads(sk, *face, &mut v);
                if let Some(l) = sk.lines.get(*axis as usize) {
                    for p in [l.p1, l.p2] {
                        let q = sk.point_xy(p as usize);
                        v.push(q.0);
                        v.push(q.1);
                    }
                }
            }
            SolidDef::Body { stock, on, through } => {
                v.extend([2.0, *stock as f64, on.len() as f64]);
                v.extend(on.iter().map(|&i| i as f64));
                v.push(through.len() as f64);
                v.extend(through.iter().map(|&i| i as f64));
            }
        }
        stack.extend(sol.operands());
    }
    v
}

fn name_read(name: &str, v: &mut Vec<f64>) {
    v.push(name.len() as f64);
    v.extend(name.bytes().map(f64::from));
}

fn face_reads(sk: &Sketch, fi: u32, v: &mut Vec<f64>) {
    v.push(fi as f64);
    let Some(f) = sk.faces.get(fi as usize) else { return };
    let plane = match f.support {
        crate::model::FaceSupport::Plane(p) => p,
        crate::model::FaceSupport::Surface(s) => {
            v.extend([-2.,s.kind as u32 as f64,s.idx as f64]);
            return;
        }
    };
    v.extend([plane.map_or(-1.0, |p| p as f64), f.holes.len() as f64]);
    for (edges, edge_names) in f.boundaries() {
        v.push(edge_names.len() as f64);
        for name in edge_names { name_read(name, v); }
        v.push(edges.len() as f64);
        for e in edges {
            v.extend([e.kind as u32 as f64, e.i() as f64]);
            // Closure depends on point identity, even when points share coordinates/parameters.
            let ends = crate::model::edge_ends(sk, *e);
            v.extend(ends.map_or([-1.0; 2], |(a, b)| [a as f64, b as f64]));
            let params = sk.entity_params(*e);
            v.push(params.len() as f64);
            v.extend(params.iter().map(|&p| sk.params[p as usize].value));
        }
    }
    if let Some(p) = plane {
        if let Some(pl) = sk.planes.get(p as usize) {
            v.extend(pl.basis.u);
            v.extend(pl.basis.v);
            v.extend(pl.basis.o);
            v.push(sk.params[pl.frame.c as usize].value);
            v.push(sk.params[pl.frame.s as usize].value);
            let o = sk.point_xy(pl.frame.origin as usize);
            v.push(o.0);
            v.push(o.1);
        }
    }
}

/// Primitive material sources for a target: stock and additions, recursively, never cuts.
/// Keeping this walk separate allows a cutter to span the body it cuts without a cycle.
fn material_sources(sk: &Sketch, target: u32) -> Result<Vec<u32>, String> {
    let mut pending = vec![(target, false)];
    let mut active = std::collections::BTreeSet::new();
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    while let Some((i, ready)) = pending.pop() {
        if seen.contains(&i) { continue; }
        let sol = sk.solids.get(i as usize).ok_or_else(|| format!("no solid at index {i}"))?;
        if ready { active.remove(&i); seen.insert(i); continue; }
        if !active.insert(i) { return Err(format!("`{}`: cyclic material extents", sol.name)); }
        pending.push((i, true));
        if let SolidDef::Body { stock, on, .. } = &sol.def {
            pending.extend(std::iter::once(stock).chain(on).rev().map(|&o| (o, false)));
        } else {
            out.push(i);
        }
    }
    Ok(out)
}

/// Dependencies required to evaluate a solid, as distinct from its Boolean operands.
pub(crate) fn evaluation_operands(sk: &Sketch, i: usize) -> Result<Vec<u32>, String> {
    let s = sk.solids.get(i).ok_or_else(|| format!("no solid at index {i}"))?;
    match s.def {
        SolidDef::Through { body, .. } => material_sources(sk, body),
        _ => Ok(s.operands()),
    }
}

/// **Resolve a solid into primitives and a term.**  The term walk: a body is its stock, plus
/// everything `on` it, minus everything that `cut`s it, each operand resolved the same way.
///
/// The document's order is irrelevant and the *statement* order inside a body is too: both
/// groups are sets. An explicit work stack accepts deep acyclic bodies and detects cycles in
/// sketches built directly through the model API as well as elaborated documents.
pub fn resolve(sk: &Sketch, si: usize, unit: f64) -> Csg {
    resolve_at(sk, si, unit, [0.0; 3])
}

// Choose a reachable source point before sweeping; never add a large world placement to
// local triangles only to subtract it again. Page input precision is still bounded by f64.
pub(super) fn frame_origin(sk: &Sketch, si: usize, unit: f64) -> [f64; 3] {
    let mut pending = vec![si];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(s) = sk.solids.get(i) else { continue };
        if let Some(f) = s.face() {
            if let Some(p) = face_poly(sk, f as usize, unit) { return p.lift(0); }
        }
        pending.extend(s.operands().into_iter().rev().map(|o| o as usize));
    }
    [0.0; 3]
}

pub(super) fn resolve_at(sk: &Sketch, si: usize, unit: f64, origin: [f64; 3]) -> Csg {
    let mut prims = Vec::new();
    let mut names = BTreeMap::new();
    let mut pending = vec![(si as u32, false)];
    let mut active = std::collections::BTreeSet::new();
    while let Some((i, ready)) = pending.pop() {
        if names.contains_key(&i) { continue; }
        if sk.solids.get(i as usize).is_none() { continue; }
        if ready {
            let term = build(sk, i, unit, origin, &mut prims, &names);
            names.insert(i, term);
            active.remove(&i);
        } else if active.insert(i) {
            pending.push((i, true));
            pending.extend(evaluation_operands(sk, i as usize).unwrap_or_default().into_iter().rev().map(|o| (o, false)));
        } else {
            names.insert(i, Term::Empty);
        }
    }
    let term = names.remove(&(si as u32)).unwrap_or(Term::Empty);
    // Extent sources are evaluation inputs, not part of a standalone cutter's geometry.
    // Preserve primitive order and avoid recursion for deeply nested terms.
    let mut ids = vec![None; prims.len()];
    let mut pending = vec![&term];
    while let Some(t) = pending.pop() {
        match t {
            Term::Prim(i) => { ids[*i] = Some(0); }
            Term::Union(a, b) | Term::Diff(a, b) => { pending.extend([a.as_ref(), b.as_ref()]); }
            Term::Empty => {}
        }
    }
    let mut kept = Vec::new();
    for (i, p) in prims.into_iter().enumerate() {
        if ids[i].is_some() { ids[i] = Some(kept.len()); kept.push(p); }
    }
    let mut term = term;
    let mut pending = vec![&mut term];
    while let Some(t) = pending.pop() {
        match t {
            Term::Prim(i) => { *i = ids[*i].expect("referenced primitive was retained"); }
            Term::Union(a, b) | Term::Diff(a, b) => { pending.extend([a.as_mut(), b.as_mut()]); }
            Term::Empty => {}
        }
    }
    Csg { prims: kept, term }
}

/// The named route from a body to each primitive, relative to the requested solid.
pub(crate) fn operand_paths(sk: &Sketch, si: usize) -> BTreeMap<String, String> {
    let mut paths = BTreeMap::new();
    let mut pending = vec![(si as u32, String::new())];
    let mut seen = std::collections::BTreeSet::new();
    while let Some((i, path)) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(s) = sk.solids.get(i as usize) else { continue };
        match &s.def {
            SolidDef::Body { .. } => {
                for o in s.operands().into_iter().rev() {
                    let Some(operand) = sk.solids.get(o as usize) else { continue };
                    let name = operand.name.rsplit('.').next().unwrap_or(&operand.name);
                    let next = if path.is_empty() { name.to_string() } else { format!("{path}.{name}") };
                    pending.push((o, next));
                }
            }
            _ => { paths.insert(s.name.clone(), path); }
        }
    }
    paths
}

fn build(
    sk: &Sketch,
    si: u32,
    unit: f64,
    origin: [f64; 3],
    prims: &mut Vec<Prim>,
    names: &BTreeMap<u32, Term>,
) -> Term {
    let Some(sol) = sk.solids.get(si as usize) else { return Term::Empty };
    let name = sol.name.clone();
    let local = |mut p: FacePoly| {
        p.basis.o = std::array::from_fn(|k| p.basis.o[k] - origin[k]); p
    };
    if let SolidDef::Body { stock, on, through } = &sol.def {
        let operand = |i: &u32| names.get(i).cloned().unwrap_or(Term::Empty);
        let mut t = operand(stock);
        for a in on { t = Term::Union(Box::new(t), Box::new(operand(a))); }
        for b in through { t = Term::Diff(Box::new(t), Box::new(operand(b))); }
        return t;
    }
    if let SolidDef::Loft { face, end, guide } = sol.def {
        let Ok(loft) = loft::prepare(sk, face, end, guide, unit) else { return Term::Empty };
        prims.push(loft.primitive(origin, &name));
        return Term::Prim(prims.len()-1);
    }
    let Some(face) = sol.face() else { return Term::Empty };
    let Ok(polys) = face_polys(sk, face as usize, unit) else { return Term::Empty };
    let through_extent = if let SolidDef::Through { body, .. } = sol.def {
        let mut bounds = Box3::empty();
        let Ok(sources) = material_sources(sk, body) else { return Term::Empty };
        for i in sources {
            let Some(mut t) = names.get(&i) else { return Term::Empty };
            // Each material source is a sweep, possibly minus its section holes.
            while let Term::Diff(outer, _) = t { t = outer; }
            let Term::Prim(pi) = t else { return Term::Empty };
            bounds.add(prims[*pi].bbox.lo);
            bounds.add(prims[*pi].bbox.hi);
        }
        if bounds.is_empty() { return Term::Empty; }
        let mut basis = polys[0].basis;
        basis.o = std::array::from_fn(|k| basis.o[k] - origin[k]);
        let n = basis.normal();
        let (lo, hi) = (0..3).fold((0.0, 0.0), |(lo, hi), k| {
            let a = n[k] * (bounds.lo[k] - basis.o[k]);
            let b = n[k] * (bounds.hi[k] - basis.o[k]);
            (lo + a.min(b), hi + a.max(b))
        });
        let diagonal = (0..3).map(|k| (bounds.hi[k] - bounds.lo[k]).powi(2)).sum::<f64>().sqrt();
        let pad = diagonal * EPS * 4.0;
        Some((lo - pad, hi + pad))
    } else { None };
    let mut term = Term::Empty;
    for (i, p) in polys.into_iter().map(local).enumerate() {
        let built = match &sol.def {
            SolidDef::Prism { from, to, .. } => prism(&p, from.value, to.value, &name),
            SolidDef::Through { .. } => {
                let (lo, hi) = through_extent.expect("through extent was resolved before sweeping");
                prism(&p, lo, hi, &name)
            }
            SolidDef::Revolve { axis, sweep, sense, .. } => {
                let Some(l) = sk.lines.get(*axis as usize) else { return Term::Empty };
                let (c, s, o) = p.pose;
                let a = plane::in_view(c, s, o, sk.point_xy(l.p1 as usize));
                let b = plane::in_view(c, s, o, sk.point_xy(l.p2 as usize));
                revolve(&p, (a, b), sweep.value, *sense, unit, &name)
            }
            SolidDef::Body { .. } | SolidDef::Loft { .. } => unreachable!(),
        };
        let Some(p) = built else { return Term::Empty };
        prims.push(p);
        let next = Term::Prim(prims.len() - 1);
        term = if i == 0 { next } else { Term::Diff(Box::new(term), Box::new(next)) };
    }
    term
}
