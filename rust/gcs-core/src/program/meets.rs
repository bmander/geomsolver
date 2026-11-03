//! **Faces read apart where the drawing is seeded, read again where it is solved** (#625).
//!
//! `when a meets b` is decided as the block is elaborated, before the solve, so it reads the body
//! where the drawing is seeded (`flatten::Apart`).  A pair it read apart that meets where the
//! drawing solves is a copy that should have made something — an edge a round left sharp — and
//! is said here, at the document's statement that made the block, rather than built wrong.
//!
//! So too `when a parallel l` and `when a perpendicular l` (#763): a face read along a direction
//! where the drawing is seeded that is not where it solves, or the other way round, made a copy
//! that should not have been made, or left out one that should — said at the same statement.
use super::{public_path, Code, Diag, SourceMap};
use crate::model::{EntKind, EntRef, Sketch};
use crate::solid::faces::{face_paths, Lays, Meets};
use crate::syntax::Span;
use std::collections::{BTreeMap, BTreeSet};

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

/// **The directions entity `e`, named `name`, draws** (#763), as the drawing stands in `sk`, each
/// by the absolute name a `when` reads it by: an axis's own, a line's from its first point to its
/// second, and a frame's `u`, `v` and `normal`; nothing for any other kind.
pub(super) fn directions(sk: &Sketch, name: &str, e: EntRef) -> Vec<(String, [f64; 3])> {
    let value = |q: u32| sk.params[q as usize].value;
    match e.kind {
        EntKind::Axis => vec![(name.to_string(), sk.axes[e.i()].d.map(value))],
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
            vec![(name.to_string(), crate::space::sub(b, a))]
        }
        EntKind::Frame => {
            let b = sk.basis(e.i());
            [("u", b.u), ("v", b.v), ("normal", b.normal())].into_iter()
                .map(|(m, d)| (format!("{name}.{m}"), d)).collect()
        }
        _ => Vec::new(),
    }
}

/// The direction `name` stands for where the drawing stands in `sk`: drawn under its own name,
/// or a frame's axis under the frame's.
fn direction_named(sk: &Sketch, map: &SourceMap, name: &str) -> Option<[f64; 3]> {
    let own = map.ent_named(name).map(|e| (name, e));
    let frame = || {
        let (of, _) = name.rsplit_once('.')?;
        map.ent_named(of).filter(|e| e.kind == EntKind::Frame).map(|e| (of, e))
    };
    let (at, e) = own.or_else(frame)?;
    directions(sk, at, e).into_iter().find(|(n, _)| n == name).map(|(_, d)| d)
}

/// One E080 for each statement and solid with a face read along a direction where the drawing is
/// seeded that is not where it stands in `sk`, or the other way round: the first such face named,
/// and how many more.
pub(super) fn along(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut read: BTreeMap<&str, Option<Lays>> = BTreeMap::new();
    let mut changed: Vec<(Span, &str, &crate::flatten::AlongRead, usize)> = Vec::new();
    let mut said = BTreeSet::new();
    for a in &map.along {
        if !said.insert((a.at, a.solid.as_str(), a.face.as_str(), a.along, a.line.as_str())) {
            continue;
        }
        let lays = read.entry(a.solid.as_str()).or_insert_with(|| {
            let e = map.ent_named(&a.solid).filter(|e| e.kind == EntKind::Solid)?;
            Lays::read(sk, e.i(), &face_paths(sk, e.i()))
        });
        let Some(d) = direction_named(sk, map, &a.line) else { continue };
        let now = lays.as_ref().and_then(|l| l.holds(&a.face, a.along, d)).and_then(Result::ok);
        if now.is_none_or(|h| h == a.held) {
            continue;
        }
        match changed.iter_mut().find(|m| m.0 == a.at && m.1 == a.solid) {
            Some(m) => m.3 += 1,
            None => changed.push((a.at, &a.solid, a, 0)),
        }
    }
    changed.into_iter().map(|(at, solid, a, more)| {
        let more = match more {
            0 => String::new(),
            1 => " (and one face more)".to_string(),
            n => format!(" (and {n} faces more)"),
        };
        let (seeded, solved, block) = if a.held {
            ("", "not ", "made")
        } else {
            ("not ", "", "left out")
        };
        Diag::new(Code::E080, at, None, format!(
            "`{}`'s face `{}` is {seeded}`{} {}` where the drawing is seeded, but {solved}where \
             it solves{more}: the `when` over it was decided at the seed, so what it makes is \
             {block} — seed the drawing nearer where it solves",
            public_path(solid), a.face, a.along.word(), public_path(&a.line)
        ))
    }).collect()
}
