//! **Faces read apart where the drawing is seeded, read again where it is solved** (#625, #761).
//!
//! `when a meets b` is decided as the block is elaborated, before the solve, so it reads the body
//! where the drawing is seeded (`flatten::Apart`).  A pair it read apart that meets where the
//! drawing solves is a copy that should have made something — an edge a round left sharp.  Where
//! one does, the elaboration solves the drawing and decides `meets` again there (`at_solve`), so
//! a shape whose seed is not its size or its place — a circle's radius left to `radius(r)`, an
//! instance placed by a mate — rounds as it would seeded where it stands.  A pair that still
//! meets where the drawing solves, read apart where it was decided, is said here, at the
//! document's statement that made the block, rather than built wrong.
use super::{public_path, Code, Diag, Elaborated, SourceMap};
use crate::flatten::Listed;
use crate::model::{EntKind, Sketch};
use crate::solid::faces::{face_paths, Meets};
use crate::syntax::Span;
use std::collections::{BTreeMap, BTreeSet};

/// Where the drawing of `e` solves, where a pair of faces `meets` read apart where it is seeded
/// meets there (#761): each solid's faces in `faces` read again at the solved pose, and every
/// entity's own numbers there by its key, for a solid whose faces are named again in a later pass
/// to be read at that pose too (`posed`).  `None` where no pair was read apart, where the solve
/// does not settle — a pose it stopped at is no answer to read faces at (#571) — or where every
/// pair read apart is apart there too: the elaboration stands as it is.
pub(super) fn at_solve(e: &Elaborated, faces: &mut BTreeMap<String, Listed>)
    -> Option<BTreeMap<String, Vec<f64>>>
{
    if e.map.apart.is_empty() || !e.ok() {
        return None;
    }
    let mut sk = e.sketch.clone();
    if !crate::solve::solve(&mut sk, crate::solve::SolveOpts::default()).settled() {
        return None;
    }
    if apart(&sk, &e.map).is_empty() {
        return None;
    }
    for (abs, listed) in faces.iter_mut() {
        if let Some(s) = e.map.ent_named(abs).filter(|s| s.kind == EntKind::Solid) {
            listed.meets = Meets::read(&sk, s.i(), &listed.paths);
        }
    }
    Some(e.map.keys().map(|(k, x)| {
        (k.clone(), sk.own_params(x).iter().map(|&q| sk.params[q as usize].value).collect())
    }).collect())
}

/// `sk` stood at `pose` (`at_solve`): each entity `map` names under a key of it given the numbers
/// the solve gave it there, where it has as many.  An entity the pose has no key for — one only
/// this pass made — keeps its seed.
pub(super) fn posed(sk: &Sketch, map: &SourceMap, pose: &BTreeMap<String, Vec<f64>>) -> Sketch {
    let mut at = sk.clone();
    for (key, values) in pose {
        let Some(e) = map.ent_named(key) else { continue };
        let own = at.own_params(e);
        if own.len() == values.len() {
            for (q, &v) in own.into_iter().zip(values) {
                at.params[q as usize].value = v;
            }
        }
    }
    at
}

/// One E080 for each statement and solid whose faces read apart meet where the drawing stands
/// in `sk`: the first such pair named, and how many more.
pub(super) fn apart(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut read: BTreeMap<&str, Option<Meets>> = BTreeMap::new();
    let mut met: Vec<(Span, &str, &[String; 2], usize)> = Vec::new();
    // each pair once, whichever way round it was asked
    let mut said = BTreeSet::new();
    for a in &map.apart {
        let [x, y] = &a.faces;
        if !said.insert((a.at, a.solid.as_str(), x.min(y), x.max(y))) {
            continue;
        }
        let meets = read.entry(a.solid.as_str()).or_insert_with(|| {
            let e = map.ent_named(&a.solid).filter(|e| e.kind == EntKind::Solid)?;
            Meets::read(sk, e.i(), &face_paths(sk, e.i()))
        });
        if !meets.as_ref().is_some_and(|m| m.meets(x, y)) {
            continue;
        }
        match met.iter_mut().find(|m| m.0 == a.at && m.1 == a.solid) {
            Some(m) => m.3 += 1,
            None => met.push((a.at, &a.solid, &a.faces, 0)),
        }
    }
    met.into_iter().map(|(at, solid, [a, b], more)| {
        let more = match more {
            0 => String::new(),
            1 => " (and one pair more)".to_string(),
            n => format!(" (and {n} pairs more)"),
        };
        Diag::new(Code::E080, at, None, format!(
            "`{}`'s faces `{a}` and `{b}` meet where the drawing solves, but not where it is \
             seeded, where `meets` read them apart{more}: what they would make is not made — seed \
             the drawing nearer where it solves",
            public_path(solid)
        ))
    }).collect()
}
