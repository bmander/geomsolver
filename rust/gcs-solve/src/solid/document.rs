//! A document's solids resolved: the material each is made of and what it reaches (the
//! dependency walks), and its term (`Csg`), each operand named by its route.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::model::SolidE;

// -- resolving a document's solids into terms ---------------------------------------------------

/// Primitive material sources for a target: stock and additions, recursively, never cuts.
/// Keeping this walk separate allows a cutter to span the body it cuts without a cycle.
pub(crate) fn material_sources(sk: &Sketch, target: u32) -> Result<Vec<u32>, String> {
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
pub fn evaluation_operands(sk: &Sketch, i: usize) -> Result<Vec<u32>, String> {
    let s = sk.solids.get(i).ok_or_else(|| format!("no solid at index {i}"))?;
    match s.def {
        SolidDef::Through { body, .. } => material_sources(sk, body),
        // a roll through a body reads its tool and the body's material (#271)
        SolidDef::Swept { source, roll: Roll::Through { body }, .. } => {
            Ok(std::iter::once(source).chain(material_sources(sk, body)?).collect())
        }
        _ => Ok(s.operands()),
    }
}

/// The solids solid `i` is built on (#572): what it is evaluated over (`evaluation_operands`, its
/// operands where that walk refuses), and a region's the solids its terms read — the body whose
/// faces a round reads.  Each once, in index order; nothing for no solid.
pub fn built_on(sk: &Sketch, i: usize) -> Vec<usize> {
    let Some(s) = sk.solids.get(i) else { return Vec::new() };
    let mut on: Vec<usize> = evaluation_operands(sk, i).unwrap_or_else(|_| s.operands())
        .into_iter().map(|o| o as usize).collect();
    if let SolidDef::Region { terms, .. } = &s.def {
        on.extend(terms.iter().flat_map(|t| t.shape.entities())
            .filter(|e| e.kind == crate::model::EntKind::Solid).map(|e| e.i()));
    }
    on.sort_unstable();
    on.dedup();
    on
}

/// Does `from` reach `goal` through its operands?  The guard on the term walk, and the one thing
/// a document can write that has no object behind it.
pub fn reaches(sk: &Sketch, from: u32, goal: u32) -> bool {
    let mut pending = vec![from];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue; }
        if sk.solids.get(i as usize).is_some() {
            let Ok(operands) = evaluation_operands(sk, i as usize) else { return true };
            for o in operands {
                if o == goal { return true; }
                pending.push(o);
            }
        }
    }
    false
}

/// **Resolve a solid into primitives and a term.**  The term walk: a body is its stock, plus
/// everything in `union` with it, minus everything that `cut`s it, each operand resolved the
/// same way.
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
    let mut pending = vec![(si,crate::envelope::Motion::identity())];
    let mut seen = std::collections::BTreeSet::new();
    while let Some((i,pose)) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(s) = sk.solids.get(i) else { continue };
        if let SolidDef::Placed {source,motion,at} = &s.def {
            if let Ok(local) = crate::motion::Family::read(sk,*motion as usize).and_then(|m| m.at(at.value)) {
                pending.push((*source as usize,local.then(pose)));
            }
            continue;
        }
        if let Some(f) = s.face() {
            if let Some(p) = face_poly(sk, f as usize, unit) { return pose.point(p.lift(0)); }
        }
        pending.extend(s.operands().into_iter().rev().map(|o| (o as usize,pose)));
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
            Term::Union(a, b) | Term::Diff(a, b) | Term::Inter(a, b) => { pending.extend([a.as_ref(), b.as_ref()]); }
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
            Term::Union(a, b) | Term::Diff(a, b) | Term::Inter(a, b) => { pending.extend([a.as_mut(), b.as_mut()]); }
            Term::Empty => {}
        }
    }
    Csg { prims: kept, term }
}

/// The named route from a body to each primitive, relative to the requested solid.
pub fn operand_paths(sk: &Sketch, si: usize) -> BTreeMap<String, String> {
    operand_primitives(sk, si).into_iter().map(|(k, (route, _))| (k, route)).collect()
}

/// Each primitive a solid is made of, keyed as its faces are named while the exact boundary is
/// built (`solid::exact`: the primitive's own name, a placed copy's under the placement): its
/// route from the solid (`plate`, `turned.boss`, empty for the solid itself) and its index.
/// The one walk a face's name is read by (#188).
pub(crate) fn operand_primitives(sk: &Sketch, si: usize) -> BTreeMap<String, (String, u32)> {
    let mut done: BTreeMap<u32, BTreeMap<String, (String, u32)>> = BTreeMap::new();
    let mut pending = vec![(si as u32,false)];
    let mut seen = std::collections::BTreeSet::new();
    while let Some((i,ready)) = pending.pop() {
        if done.contains_key(&i) { continue; }
        let Some(s) = sk.solids.get(i as usize) else { continue };
        if !ready {
            if !seen.insert(i) { continue; }
            pending.push((i,true));
            pending.extend(s.operands().into_iter().rev().map(|o| (o,false)));
            continue;
        }
        let mut paths = BTreeMap::new();
        match &s.def {
            SolidDef::Placed {source,..} => {
                if let Some(source) = done.get(source) {
                    for (path, prim) in source.values() {
                        paths.insert(placed_name(&s.name,path),(path.clone(),*prim));
                    }
                }
            }
            SolidDef::Body { .. } => {
                for o in s.operands().into_iter().rev() {
                    let Some(operand) = sk.solids.get(o as usize) else { continue };
                    let name = operand.step();
                    let name = name.as_str();
                    if let Some(source) = done.get(&o) {
                        for (primitive,(path,prim)) in source {
                            // a swept solid's own sweep, split off as its stock, is the solid
                            // itself and adds no step to the route
                            let route = if operand.name == s.name { path.clone() }
                                else { placed_name(name,path) };
                            paths.insert(primitive.clone(),(route,*prim));
                        }
                    }
                }
            }
            _ => { paths.insert(s.name.clone(),(String::new(),i)); }
        }
        done.insert(i,paths);
    }
    done.remove(&(si as u32)).unwrap_or_default()
}

/// **The step a body's operand adds to a face's route** (#188, [0.149]): its own name, the last
/// segment of its absolute name, under the label of each named use of a word it was made in
/// (`rim#17`, `flatten::keys::labelled`) — `rim.taken` for the `taken` a round named `rim` made,
/// so two named rounds' faces stay apart.  The one reading of a route's steps: the faces are
/// named by it (`operand_primitives`, `brep::recipe`) and reached by it (`faces::face_source`,
/// `carrier`), a copy the document wrote with its index past it (`SolidE::step`, [0.157]).
pub fn route_step(name: &str) -> String {
    let segs: Vec<&str> = name.split('.').collect();
    let Some((leaf, before)) = segs.split_last() else { return name.to_string() };
    fn label<'s>(s: &&'s str) -> Option<&'s str> {
        let (l, id) = s.split_once('#')?;
        (!l.is_empty() && !id.is_empty() && id.bytes().all(|b| b.is_ascii_digit())).then_some(l)
    }
    before.iter().filter_map(label).chain(std::iter::once(*leaf)).collect::<Vec<_>>().join(".")
}

/// Whether `route` begins with `operand`'s step (`SolidE::step`), and the rest of it past: a
/// step written without the copy's index is any copy's (`faces::same_step`).
pub fn past_step<'r, S: AsRef<str>>(operand: &SolidE, route: &'r [S]) -> Option<&'r [S]> {
    let step = operand.step();
    let n = step.split('.').count();
    let same = |(a, b): (&str, &S)| super::faces::same_step(b.as_ref(), a);
    (route.len() >= n && step.split('.').zip(route).all(same)).then(|| &route[n..])
}

pub(super) fn placed_name(name: &str,path: &str) -> String {
    if path.is_empty() { name.to_string() } else { format!("{name}.{path}") }
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
    if let SolidDef::Placed {source,motion,at} = &sol.def {
        let Ok(pose) = crate::motion::Family::read(sk,*motion as usize).and_then(|m| m.at(at.value))
            else { return Term::Empty };
        let paths = operand_paths(sk,*source as usize);
        let Some(source) = names.get(source) else { return Term::Empty };
        let mut term = source.clone();
        let mut mapped = BTreeMap::new();
        let mut pending = vec![&mut term];
        while let Some(t) = pending.pop() {
            match t {
                Term::Prim(i) => {
                    *i = *mapped.entry(*i).or_insert_with(|| {
                        let p = &prims[*i];
                        let facets = p.facets.iter().map(|f| Facet {
                            pts:f.pts.iter().map(|&p| {
                                let world = pose.point(std::array::from_fn(|k| p[k]+origin[k]));
                                std::array::from_fn(|k| world[k]-origin[k])
                            }).collect(),n:pose.vector(f.n),face:f.face,smooth:f.smooth,
                        }).collect();
                        let path = paths.get(&p.of).map(String::as_str).unwrap_or(&p.of);
                        let copy = finish(facets,p.faces.clone(),&placed_name(&name,path));
                        let next = prims.len(); prims.push(copy); next
                    });
                }
                Term::Union(a,b) | Term::Diff(a,b) | Term::Inter(a,b) => pending.extend([a.as_mut(),b.as_mut()]),
                Term::Empty => {}
            }
        }
        return term;
    }
    let local = |mut p: FacePoly| {
        p.basis.o = std::array::from_fn(|k| p.basis.o[k] - origin[k]); p
    };
    if let SolidDef::Body { stock, on, through, bound } = &sol.def {
        let operand = |i: &u32| names.get(i).cloned().unwrap_or(Term::Empty);
        let mut t = operand(stock);
        for a in on { t = Term::Union(Box::new(t), Box::new(operand(a))); }
        for b in through { t = Term::Diff(Box::new(t), Box::new(operand(b))); }
        for c in bound { t = Term::Inter(Box::new(t), Box::new(operand(c))); }
        return t;
    }
    if let SolidDef::Family { .. } = sol.def {
        let Ok(family) = super::family::built(sk, si as usize, unit) else { return Term::Empty };
        prims.push(family.primitive(origin, &name));
        return Term::Prim(prims.len()-1);
    }
    // a region: its meridian's loops turned once about its axis, the holes taken out
    if let SolidDef::Region { .. } = sol.def {
        let Ok(super::region::Form::Meridian(m)) = super::region::form(sk, si as usize) else {
            return Term::Empty;
        };
        let mut term = Term::Empty;
        let (axis, full) = (((0.0, 0.0), (0.0, 1.0)), std::f64::consts::TAU);
        for (i, p) in super::region::face_polys(&m, unit).into_iter().map(local).enumerate() {
            let Some(prim) = revolve(&p, axis, full, crate::model::Sense::Ccw, unit, &name) else {
                return Term::Empty;
            };
            prims.push(prim);
            let next = Term::Prim(prims.len() - 1);
            term = if i == 0 { next } else { Term::Diff(Box::new(term), Box::new(next)) };
        }
        return term;
    }
    let Some(face) = sol.face() else { return Term::Empty };
    let Ok(polys) = face_polys(sk, face as usize, unit) else { return Term::Empty };
    let through_extent = if let SolidDef::Through { body, .. } = sol.def {
        let mut bounds = Box3::empty();
        let Ok(sources) = material_sources(sk, body) else { return Term::Empty };
        for i in sources {
            let Some(t) = names.get(&i) else { return Term::Empty };
            // Placed composite sources can contain unions. Extents include only
            // additive material; holes and other subtractions never enlarge them.
            let mut pending = vec![t];
            while let Some(t) = pending.pop() {
                match t {
                    Term::Prim(pi) => { bounds.add(prims[*pi].bbox.lo); bounds.add(prims[*pi].bbox.hi); }
                    Term::Union(a,b) => pending.extend([a.as_ref(),b.as_ref()]),
                    Term::Diff(a,_) | Term::Inter(a,_) => pending.push(a),
                    Term::Empty => {}
                }
            }
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
        let diagonal = (0..3).map(|k| (bounds.hi[k] - bounds.lo[k]).dpowi(2)).sum::<f64>().sqrt();
        let pad = diagonal * EPS * 4.0;
        Some((lo - pad, hi + pad))
    } else { None };
    let taper = match &sol.def { SolidDef::Prism { draft: Some(d), .. } => d.value.dtan(), _ => 0.0 };
    let mut term = Term::Empty;
    for (i, p) in polys.into_iter().map(local).enumerate() {
        let built = match &sol.def {
            // a draft shrinks the outer loop and grows each hole: their prisms are subtracted
            SolidDef::Prism { from, to, .. } => prism(&p, from.value, to.value, primitive::loop_taper(i, taper), &name),
            SolidDef::Through { .. } => {
                let (lo, hi) = through_extent.expect("through extent was resolved before sweeping");
                prism(&p, lo, hi, 0.0, &name)
            }
            SolidDef::Revolve { face, axis, sweep, sense } => {
                let Some(ab) = sk.revolve_axis(*face, *axis) else { return Term::Empty };
                revolve(&p, ab, sweep.value, *sense, unit, &name)
            }
            SolidDef::Body { .. } | SolidDef::Placed { .. }
                | SolidDef::Swept { .. } | SolidDef::Region { .. }
                | SolidDef::Family { .. } => unreachable!(),
        };
        let Some(p) = built else { return Term::Empty };
        prims.push(p);
        let next = Term::Prim(prims.len() - 1);
        term = if i == 0 { next } else { Term::Diff(Box::new(term), Box::new(next)) };
    }
    term
}
