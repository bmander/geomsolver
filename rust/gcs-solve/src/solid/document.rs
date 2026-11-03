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

/// **A body an unbounded tool is applied to** (#737): a prism through everything, or a body over
/// its whole roll, is bounded by no body of its own but by the context it is applied in — every
/// body it, or a copy of it placed at a roll (a pattern's), is cut from or kept in common with.
/// `pose` is where the copies' placements put the tool for that body: none where the tool itself
/// is written there.
#[derive(Clone, Copy, Debug)]
pub struct Use {
    pub body: u32,
    pub pose: Option<crate::envelope::Motion>,
}

/// Each place tool `tool` is applied to a body, in the order the bodies stand (cuts before
/// commons): the body, and the placements `(motion, at)` from the copy written there down to the
/// tool, the outermost first — none where the tool itself is written there.  A body cutting a
/// member's sweep that a union's sweep is distributed into (`distributed::lowered`) cuts the
/// union's.
pub(crate) fn each_use(sk: &Sketch, tool: usize, mut each: impl FnMut(u32, &[(u32, f64)])) {
    let mut chain = Vec::new();
    for (b, s) in sk.solids.iter().enumerate() {
        let SolidDef::Body { through, bound, .. } = &s.def else { continue };
        for &c in through.iter().chain(bound) {
            chain.clear();
            let mut at = c as usize;
            // a placement is of a solid built before it, but a model a program writes is held to
            // no such order: a walk longer than the solids are many is a cycle, and is no use
            for _ in 0..=sk.solids.len() {
                // a member's sweep a union's distributes into is a use of the union's (#684 C)
                if at == tool || super::distributed::union_of(sk, at) == tool {
                    each(b as u32, &chain);
                    break;
                }
                let Some(SolidDef::Placed { source, motion, at: value }) =
                    sk.solids.get(at).map(|s| &s.def) else { break };
                chain.push((*motion, value.value));
                at = *source as usize;
            }
        }
    }
}

/// The bodies tool `tool` is applied to (`Use`), each once, in index order: what an unbounded
/// tool is evaluated over, with no pose read.
pub fn applied_to(sk: &Sketch, tool: usize) -> Vec<u32> {
    let mut bodies = Vec::new();
    each_use(sk, tool, |b, _| bodies.push(b));
    bodies.sort_unstable();
    bodies.dedup();
    bodies
}

/// **Every use of tool `tool`** (`Use`), each with the pose its copies' placements put the tool
/// at; a copy whose placement does not read is no use, as it builds nothing.  Each motion is read
/// once however many copies it places.
pub fn uses(sk: &Sketch, tool: usize) -> Vec<Use> {
    let mut families: BTreeMap<u32, Option<crate::motion::Family>> = BTreeMap::new();
    let mut out = Vec::new();
    each_use(sk, tool, |body, chain| {
        let mut pose: Option<crate::envelope::Motion> = None;
        // the outermost placement is applied last: each one below it goes first
        for &(motion, at) in chain.iter().rev() {
            let family = families.entry(motion)
                .or_insert_with(|| crate::motion::Family::read(sk, motion as usize).ok());
            let Some(local) = family.as_ref().and_then(|f| f.at(at).ok()) else { return };
            pose = Some(pose.map_or(local, |p| p.then(local)));
        }
        out.push(Use { body, pose });
    });
    out
}

/// The material of `bodies`: the stock and additions of each (`material_sources`), each source
/// once, in the order the bodies give them — what an unbounded tool applied to them is evaluated
/// over.
pub(crate) fn material_of(sk: &Sketch, bodies: &[u32]) -> Result<Vec<u32>, String> {
    let mut out: Vec<u32> = Vec::new();
    for &b in bodies {
        for m in material_sources(sk, b)? {
            if !out.contains(&m) { out.push(m); }
        }
    }
    Ok(out)
}

/// **The span of a prism through everything** (#737) along the normal `n` of its section through
/// `o`: over each use of `tool`, the box `bounds_of` gives about that use's body's material (asked
/// once a body), read from where the use puts the section (`brep::geom::through_span`).  `None`
/// for no use bounded.
pub(crate) fn through_span(sk: &Sketch, tool: usize, o: [f64; 3], n: [f64; 3],
                           mut bounds_of: impl FnMut(u32) -> Result<Box3, String>)
    -> Result<Option<(f64, f64)>, String> {
    let mut boxes: BTreeMap<u32, Box3> = BTreeMap::new();
    let mut spans = Vec::new();
    for u in uses(sk, tool) {
        if !boxes.contains_key(&u.body) {
            boxes.insert(u.body, bounds_of(u.body)?);
        }
        let b = boxes[&u.body];
        let (o, n) = u.pose.map_or((o, n), |p| (p.point(o), p.vector(n)));
        spans.push((o, n, (b.lo, b.hi)));
    }
    Ok(crate::brep::geom::through_span(spans))
}

/// Dependencies required to evaluate a solid, as distinct from its Boolean operands: an unbounded
/// tool's the material of the bodies it is applied to (`material_of`).
pub fn evaluation_operands(sk: &Sketch, i: usize) -> Result<Vec<u32>, String> {
    let s = sk.solids.get(i).ok_or_else(|| format!("no solid at index {i}"))?;
    match s.def {
        SolidDef::Through { .. } => material_of(sk, &applied_to(sk, i)),
        // a roll through everything reads its tool and that material (#271)
        SolidDef::Swept { source, roll: Roll::Through, .. } => {
            Ok(std::iter::once(source).chain(material_of(sk, &applied_to(sk, i))?).collect())
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
    first_placed(sk,si,|f,by| face_poly(sk,f as usize,unit).map(|p| by.point(p.lift(0))))
        .unwrap_or([0.0; 3])
}

/// Where the motion `motion` puts what a placement at `at` (radians) carries.
pub(super) fn placement(sk: &Sketch, motion: u32, at: f64) -> Result<crate::envelope::Motion,String> {
    crate::motion::Family::read(sk,motion as usize).and_then(|m| m.at(at))
}

/// What `pick` makes of the first face of solid `si` it takes, under where the placements above
/// that face put it (#697): its operands walked in order, a placement's source under the
/// placement's pose and a placement whose pose does not read passed over — the one walk of what
/// a solid's frame is (`frame_origin`) and what its shape key is read relative to (`reads`).
pub(super) fn first_placed<T>(sk: &Sketch, si: usize,
                              mut pick: impl FnMut(u32,crate::envelope::Motion) -> Option<T>)
    -> Option<T> {
    let mut pending = vec![(si,crate::envelope::Motion::identity())];
    let mut seen = std::collections::BTreeSet::new();
    while let Some((i,pose)) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(s) = sk.solids.get(i) else { continue };
        if let SolidDef::Placed {source,motion,at} = &s.def {
            if let Ok(local) = placement(sk,*motion,at.value) {
                pending.push((*source as usize,local.then(pose)));
            }
            continue;
        }
        if let Some(found) = s.face().and_then(|f| pick(f,pose)) { return Some(found); }
        pending.extend(s.operands().into_iter().rev().map(|o| (o as usize,pose)));
    }
    None
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
                    let name = step_through(s, operand);
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

/// **A use's step** (#188, [0.149]): what `step_through` reads from the first segment a
/// statement's use put in an operand's name: the last segment, under the label of each named use
/// of a word it was made in (`rim#17`, `flatten::keys::labelled`) — `rim.taken` for the `taken` a
/// round named `rim` made, so two named rounds' faces stay apart.
fn route_step(name: &str) -> String {
    let segs: Vec<&str> = name.split('.').collect();
    let Some((leaf, before)) = segs.split_last() else { return name.to_string() };
    fn label<'s>(s: &&'s str) -> Option<&'s str> {
        let (l, id) = s.split_once('#')?;
        (!l.is_empty() && !id.is_empty() && id.bytes().all(|b| b.is_ascii_digit())).then_some(l)
    }
    before.iter().filter_map(label).chain(std::iter::once(*leaf)).collect::<Vec<_>>().join(".")
}

/// **The step operand `operand` adds to the route of a face through body `body`** (#188,
/// [0.149], #760): the operand's path as the body's scope writes it — `hub.body` for the `body`
/// of an instance `hub` beside it, `joint[1].pad` for the `pad` of an instance a `repeat`'s copy
/// holds (`SolidE::written`), `drill[1]` for the copy itself — so no two operands a document
/// names apart are named alike.  From the first segment a statement's use put there (`#17`, a
/// named use's `rim#17`) the rest is read by `route_step`, a named round's `rim.taken`; an
/// operand outside the body's scope (a solid an instance was given) is read by it whole.  The
/// one reading of a route's steps: the faces are named by it (`operand_primitives`, the recipe's
/// `steps`) and reached by it (`past_step`).
pub fn step_through(body: &SolidE, operand: &SolidE) -> String {
    step_between(body.written_name(), operand.written_name())
}

/// `step_through` by the two solids' written names (`SolidE::written_name`, `public_path`): what
/// a reader with no `SolidE` to hand walks a body by (the flattener's carrier of a face).
pub fn step_between(body: &str, written: &str) -> String {
    let scope = body.rsplit_once('.').map_or("", |(s, _)| s);
    let within = if scope.is_empty() { Some(written) }
        else { written.strip_prefix(scope).and_then(|r| r.strip_prefix('.')) };
    let Some(within) = within.filter(|w| !w.is_empty()) else { return route_step(written) };
    let segs: Vec<&str> = within.split('.').collect();
    match segs.iter().position(|s| s.contains('#')) {
        None => within.to_string(),
        Some(0) => route_step(within),
        Some(k) => format!("{}.{}", segs[..k].join("."), route_step(&segs[k..].join("."))),
    }
}

/// `step_through` without its leaf: the labels of the named uses (and the written steps) an
/// operand's faces are named under through `body`, empty for an operand of one step — what the
/// recipe lumps a body's lowered regions by and the stratum's layers read ([0.149]).
pub fn step_labels(body: &SolidE, operand: &SolidE) -> String {
    let mut step = step_through(body, operand);
    step.truncate(step.rfind('.').unwrap_or(0));
    step
}

/// Whether `route` begins with `operand`'s step through `body` (`step_through`), and the rest
/// of it past: a step written without a copy's index is any copy's (`faces::same_step`).
pub fn past_step<'r, S: AsRef<str>>(body: &SolidE, operand: &SolidE, route: &'r [S])
    -> Option<&'r [S]> {
    route_past(&step_through(body, operand), route)
}

/// Whether `route` begins with `step` (`step_through`'s), and the rest of it past.
pub fn route_past<'r, S: AsRef<str>>(step: &str, route: &'r [S]) -> Option<&'r [S]> {
    let n = step.split('.').count();
    let same = |(a, b): (&str, &S)| super::faces::same_step(b.as_ref(), a);
    (route.len() >= n && step.split('.').zip(route).all(same)).then(|| &route[n..])
}

pub(super) fn placed_name(name: &str,path: &str) -> String {
    if path.is_empty() { name.to_string() } else { format!("{name}.{path}") }
}

/// `bounds` widened to hold the additive material of term `t`: placed composite sources can contain
/// unions, and holes and other subtractions never enlarge an extent.
fn additive_bounds(t: &Term, prims: &[Prim], bounds: &mut Box3) {
    let mut pending = vec![t];
    while let Some(t) = pending.pop() {
        match t {
            Term::Prim(pi) => { bounds.add(prims[*pi].bbox.lo); bounds.add(prims[*pi].bbox.hi); }
            Term::Union(a, b) => pending.extend([a.as_ref(), b.as_ref()]),
            Term::Diff(a, _) | Term::Inter(a, _) => pending.push(a),
            Term::Empty => {}
        }
    }
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
        let Ok(pose) = placement(sk,*motion,at.value) else { return Term::Empty };
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
    // a developable panel's band (#645): faceted from its rulings
    if let Some(band) = super::sheet::band(sk, si as usize, unit) {
        let Ok(band) = band else { return Term::Empty };
        prims.push(band.primitive(origin, &name));
        return Term::Prim(prims.len() - 1);
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
    // through everything (#737): across the material of every body it is applied to, read where
    // each use puts the section — the terms stand `origin` off the world, and a pose moves the
    // world, so the section and each box are read in the world
    let through_extent = if let SolidDef::Through { .. } = sol.def {
        let basis = polys[0].basis;
        let bounds = |body: u32| -> Result<Box3, String> {
            let mut bounds = Box3::empty();
            for i in material_sources(sk, body)? {
                let t = names.get(&i).ok_or("a material source not built before its cutter")?;
                additive_bounds(t, &prims, &mut bounds);
            }
            if bounds.is_empty() { return Ok(bounds); }
            Ok(Box3::of([bounds.lo, bounds.hi].map(|p| std::array::from_fn(|k| p[k] + origin[k]))))
        };
        match through_span(sk, si as usize, basis.o, basis.normal(), bounds) {
            Ok(Some(span)) => Some(span),
            _ => return Term::Empty,
        }
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
