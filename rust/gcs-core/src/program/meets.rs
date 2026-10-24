//! **Faces read apart where the drawing is seeded, read again where it is solved** (#625).
//!
//! `when a meets b` is decided as the block is elaborated, before the solve, so it reads the body
//! where the drawing is seeded (`flatten::Apart`).  A pair it read apart that meets where the
//! drawing solves is a copy that should have made something — an edge a round left sharp — and
//! is said here, at the document's statement that made the block, rather than built wrong.
use super::{public_path, Code, Diag, SourceMap};
use crate::model::{EntKind, Sketch};
use crate::solid::faces::{face_paths, Meets};
use std::collections::BTreeMap;

/// One E080 for each statement and solid whose faces read apart meet where the drawing stands
/// in `sk`: the first such pair named, and how many more.
pub(super) fn apart(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut read: BTreeMap<&str, Option<Meets>> = BTreeMap::new();
    let mut met: Vec<(crate::syntax::Span, &str, &[String; 2], usize)> = Vec::new();
    // each pair once, whichever way round it was asked
    let mut said = std::collections::BTreeSet::new();
    for a in &map.apart {
        let [x, y] = &a.faces;
        if !said.insert((a.at, a.solid.as_str(), x.min(y), x.max(y))) {
            continue;
        }
        let meets = read.entry(a.solid.as_str()).or_insert_with(|| {
            let e = map.ent_named(&a.solid).filter(|e| e.kind == EntKind::Solid)?;
            Meets::read(sk, e.i(), &face_paths(sk, e.i()))
        });
        if !meets.as_ref().is_some_and(|m| m.meets(&a.faces[0], &a.faces[1])) {
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
