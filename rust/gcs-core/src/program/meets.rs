//! **Faces read apart where the drawing is seeded, read again where it is solved** (#625, #761).
//!
//! `when a meets b` is decided as the block is elaborated, before the solve, so it reads the body
//! where the drawing is seeded (`flatten::Apart`).  A pair it read apart that meets where the
//! drawing solves is a copy that should have made something — an edge a round left sharp.  So too
//! `when a parallel l` and `when a perpendicular l` (#763): a block over faces read along a
//! direction that, decided again with the faces as the drawing solves, comes out the other way
//! made a copy that should not have been made, or left out one that should.  Where either does,
//! the elaboration solves the drawing and decides the conditions again there (`at_solve`), so a
//! shape whose seed is not its size or its place — a circle's radius left to `radius(r)`, an
//! instance placed by a mate, a line held along an axis — rounds as it would seeded where it
//! stands.  What still comes out otherwise where the drawing solves is said here, at the
//! document's statement that made the block, rather than built wrong.
use super::{own_values, public_path, Code, Diag, Elaborated, SourceMap, Stood};
use crate::flatten::{Apart, AlongRead};
use crate::model::{EntKind, EntRef, Sketch};
use crate::solid::faces::{face_paths, Lays, ListedFaces, Meets};
use crate::syntax::Span;
use std::collections::{BTreeMap, BTreeSet};

/// Where the drawing of `e` solves, where a pair of faces `meets` read apart where it is seeded
/// meets there, or a block over faces read along a direction comes out the other way (#761,
/// #763): every entity's own numbers there by its key, for the faces a block runs over to be named
/// again at that pose (`elaborate`, `stand`).  Empty where nothing was read so, where the solve
/// does not settle — a pose it stopped at is no answer to read faces at (#571) — or where every
/// condition comes out there as it did: the elaboration stands as it is.
pub(super) fn at_solve(e: &Elaborated) -> Stood {
    if (e.map.apart.is_empty() && e.map.along.is_empty()) || !e.ok() {
        return Stood::new();
    }
    let mut sk = e.sketch.clone();
    if !crate::solve::solve(&mut sk, crate::solve::SolveOpts::default()).settled()
        || read_again(&sk, &e.map).is_empty()
    {
        return Stood::new();
    }
    e.map.keys().map(|(k, x)| (k.clone(), own_values(&sk, x))).collect()
}

/// Both checks where the drawing stands in `sk`, each solid they name read once.
pub(super) fn read_again(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let solids: BTreeSet<&str> = map.apart.iter().map(|a| a.solid.as_str())
        .chain(map.along.iter().flat_map(|w| &w.faces).map(|a| a.solid.as_str())).collect();
    let read: Read = solids.into_iter().filter_map(|solid| {
        let e = map.ent_named(solid).filter(|e| e.kind == EntKind::Solid)?;
        let listed = ListedFaces::read(sk, e.i(), &face_paths(sk, e.i()))?;
        Some((solid, (Meets::of(&listed), Lays::of(&listed))))
    }).collect();
    let mut diags = apart(map, &read);
    diags.extend(along(sk, map, &read));
    diags
}

/// Each solid's faces where the drawing is solved, by its absolute name: where its boundary says.
type Read<'m> = BTreeMap<&'m str, (Meets, Lays)>;

/// One E080 for each statement and solid whose faces read apart meet where the drawing is solved:
/// the first such pair named, and how many more.
fn apart(map: &SourceMap, read: &Read) -> Vec<Diag> {
    // each pair once, whichever way round it was asked
    let mut said = BTreeSet::new();
    let met = map.apart.iter().filter(|a| {
        let [x, y] = &a.faces;
        said.insert((a.at, a.solid.as_str(), x.min(y), x.max(y)))
            && read.get(a.solid.as_str()).is_some_and(|(m, _)| m.meets(x, y))
    });
    grouped(met.map(|a: &Apart| (a.at, a.solid.as_str(), a))).into_iter().map(|(at, a, n)| {
        let [x, y] = &a.faces;
        Diag::new(Code::E080, at, None, format!(
            "`{}`'s faces `{x}` and `{y}` meet where the drawing solves, but not where it is \
             seeded, where `meets` read them apart{}: what they would make is not made — seed \
             the drawing nearer where it solves",
            public_path(&a.solid), more(n, "pair")
        ))
    }).collect()
}

/// One E080 for each statement and solid with a `when` over faces read along a direction that,
/// decided again with the faces as the drawing is solved, is made where it was not or the other
/// way round: the first face read the other way named, and how many more.
fn along(sk: &Sketch, map: &SourceMap, read: &Read) -> Vec<Diag> {
    let solved = |a: &AlongRead| {
        let d = direction(sk, |n| map.ent_named(n), &a.line)?;
        read.get(a.solid.as_str())?.1.holds(&a.face, a.along, d)?.ok()
    };
    let turned = map.along.iter().flat_map(|w| {
        let now: Vec<bool> = w.faces.iter().map(|a| solved(a).unwrap_or(a.held)).collect();
        let decided_again = w.truth.holds(&|k| now[k]) != w.made;
        w.faces.iter().zip(now).filter(move |(a, now)| decided_again && *now != a.held)
            .map(move |(a, _)| (w.at, a.solid.as_str(), (a, w.made)))
    });
    grouped(turned).into_iter().map(|(at, (a, made), n)| {
        let (seeded, solved) = if a.held { ("", "not ") } else { ("not ", "") };
        let block = if made { "made" } else { "left out" };
        Diag::new(Code::E080, at, None, format!(
            "`{}`'s face `{}` is {seeded}`{} {}` where the drawing is seeded, but {solved}where \
             it solves{}: the `when` over it was decided at the seed, so what it makes is \
             {block} — seed the drawing nearer where it solves",
            public_path(&a.solid), a.face, a.along.word(), public_path(&a.line), more(n, "face")
        ))
    }).collect()
}

/// Each statement and solid's first item, and how many more it has, in the order they come.
fn grouped<'a, T>(items: impl Iterator<Item = (Span, &'a str, T)>) -> Vec<(Span, T, usize)> {
    let mut out: Vec<(Span, &str, T, usize)> = Vec::new();
    for (at, solid, t) in items {
        match out.iter_mut().find(|m| m.0 == at && m.1 == solid) {
            Some(m) => m.3 += 1,
            None => out.push((at, solid, t, 0)),
        }
    }
    out.into_iter().map(|(at, _, t, n)| (at, t, n)).collect()
}

/// `(and one {noun} more)`, or how many more, after the first said.
fn more(n: usize, noun: &str) -> String {
    match n {
        0 => String::new(),
        1 => format!(" (and one {noun} more)"),
        n => format!(" (and {n} {noun}s more)"),
    }
}

/// **The direction `name` stands for** (#763), where the drawing stands in `sk`, each entity by
/// `of`: an axis's own, a line's from its first point to its second (`analysis::direction`), or a
/// frame's `u`, `v` or `normal` under the frame's name.
pub(super) fn direction(sk: &Sketch, of: impl Fn(&str) -> Option<EntRef>, name: &str)
    -> Option<[f64; 3]> {
    if let Some(e) = of(name) {
        return crate::diagnose::analysis::direction(sk, e).ok();
    }
    let (frame, member) = name.rsplit_once('.')?;
    let b = sk.basis(of(frame).filter(|e| e.kind == EntKind::Frame)?.i());
    match member {
        "u" => Some(b.u),
        "v" => Some(b.v),
        "normal" => Some(b.normal()),
        _ => None,
    }
}
