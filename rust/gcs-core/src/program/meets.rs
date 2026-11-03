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
use super::{own_values, public_path, Code, Diag, Elaborated, SourceMap, Stood};
use crate::model::{EntKind, Sketch};
use crate::solid::faces::{face_paths, Meets};
use crate::syntax::Span;
use std::collections::{BTreeMap, BTreeSet};

/// Where the drawing of `e` solves, where a pair of faces `meets` read apart where it is seeded
/// meets there (#761): every entity's own numbers there by its key, for the faces a block runs
/// over to be named again at that pose (`elaborate`, `stand`).  Empty where no pair was read
/// apart, where the solve does not settle — a pose it stopped at is no answer to read faces at
/// (#571) — or where every pair read apart is apart there too: the elaboration stands as it is.
pub(super) fn at_solve(e: &Elaborated) -> Stood {
    if e.map.apart.is_empty() || !e.ok() {
        return Stood::new();
    }
    let mut sk = e.sketch.clone();
    if !crate::solve::solve(&mut sk, crate::solve::SolveOpts::default()).settled()
        || apart(&sk, &e.map).is_empty()
    {
        return Stood::new();
    }
    e.map.keys().map(|(k, x)| (k.clone(), own_values(&sk, x))).collect()
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
