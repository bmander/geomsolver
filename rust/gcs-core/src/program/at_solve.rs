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
use crate::flatten::FaceRead;
use crate::model::{EntKind, EntRef, Sketch};
use crate::solid::faces::{face_paths, Layered, Layers};
use crate::syntax::Span;
use std::collections::{BTreeMap, BTreeSet};

/// Where the drawing of `e` solves, where a pair of faces `meets` read apart where it is seeded
/// meets there, a block over faces read along a direction comes out the other way (#761, #763),
/// or a body a block read along a direction was not built where it is seeded (#948): every
/// entity's own numbers there by its key, for the faces a block runs over to be named again at
/// that pose (`elaborate`, `stand`).  Empty where nothing was read so, where the solve
/// does not settle — a pose it stopped at is no answer to read faces at (#571) — or where every
/// condition comes out there as it did: the elaboration stands as it is.
pub(super) fn at_solve(e: &mut Elaborated) -> Stood {
    // a solid a `when` could not read where the drawing is seeded is read again where it solves,
    // though that pass refused the block (#948), where such refusals are all it refused
    let unread = !e.map.unread.is_empty()
        && e.errors().all(|d| d.code == Code::E103 && e.map.unread.contains(&d.span));
    let read = !(e.map.apart.is_empty() && e.map.pruned.is_empty() && e.map.along.is_empty());
    if !unread && (!read || !e.ok()) {
        return Stood::new();
    }
    let mut sk = e.sketch.clone();
    if !crate::solve::solve(&mut sk, crate::solve::SolveOpts::default()).settled() {
        return Stood::new();
    }
    let again = unread || !read_again(&sk, &e.map).is_empty();
    // what reading the solids where the drawing solves built is the drawing's to start from: each
    // entry is keyed by everything it read, and that pose is where a front end solves it (#994)
    e.sketch.caches = std::mem::take(&mut sk.caches).carried();
    if !again {
        return Stood::new();
    }
    e.map.keys().map(|(k, x)| (k.clone(), own_values(&sk, x))).collect()
}

/// Both checks where the drawing stands in `sk`, each solid they name read once: which of its
/// faces meet where a pair was read of it, how each lies where a face was read along a direction.
pub(super) fn read_again(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    // each solid read once over one reading held still (#994)
    let _holding = sk.caches.held.open();
    let reads = map.along.iter().flat_map(|w| &w.reads);
    let meets: BTreeSet<&str> = map.apart.iter().map(|a| a.solid.as_str())
        .chain(map.pruned.iter().map(|p| p.solid.as_str()))
        .chain(reads.clone().filter_map(|r| match r {
            FaceRead::Meets { solid, .. } => Some(solid.as_str()),
            _ => None,
        })).collect();
    let lays: BTreeSet<&str> = reads.filter_map(|r| match r {
        FaceRead::Along(a) => Some(a.solid.as_str()),
        _ => None,
    }).collect();
    let read: Read = meets.union(&lays).filter_map(|&solid| {
        let e = map.ent_named(solid).filter(|e| e.kind == EntKind::Solid)?;
        Some((solid, Layered::read(sk, &Layers::of(sk, e.i()), &face_paths(sk, e.i())).ok()?))
    }).collect();
    let mut diags = apart(map, &read);
    diags.extend(along(sk, map, &read));
    diags
}

/// **A round or a chamfer that selects no edge** (#962, W120): each statement whose blocks over a
/// solid's faces paired them by `meets` (`flatten::Unmade`), read where the drawing stands in `sk`
/// edge by edge (`ListedFaces::meeting`, in the layer each pair was asked in): where no pair it
/// asked meets in an edge — the faces apart or one of them taken in by the body's other terms, at
/// a corner alone, or along seams where one goes on tangent into the other, which are no edges —
/// or its blocks ran over one face alone, it makes nothing, and says so once at the statement,
/// naming what it looked at and why.  Never where a pair meets in an edge, where the boundary
/// cannot say of a face, nor at a statement `said` already (`read_again`'s E080).
pub(super) fn unmade(sk: &Sketch, map: &SourceMap, said: &[Diag]) -> Vec<Diag> {
    use crate::solid::faces::{FacePath, ListedFaces, Meeting};
    // each solid read once over one reading held still (#994)
    let _holding = sk.caches.held.open();
    // each solid's layers and faces read once, and each of its layers' listed faces once
    let mut bodies: BTreeMap<usize, (Layers, Vec<FacePath>)> = BTreeMap::new();
    let mut listed: BTreeMap<(usize, usize), Option<ListedFaces>> = BTreeMap::new();
    let mut out = Vec::new();
    'statements: for u in &map.unmade {
        if said.iter().any(|d| d.span == u.at) {
            continue;
        }
        let Some(e) = map.ent_named(&u.solid).filter(|e| e.kind == EntKind::Solid) else { continue };
        let solid = public_path(&u.solid);
        let path = |rest: &[String]| public_path(&std::iter::once(u.solid.as_str())
            .chain(rest.iter().map(String::as_str)).collect::<Vec<_>>().join("."));
        // a round's `A` and its `B`; `round(r) X` is `X` with itself
        let (a, b) = match &u.blocks[..] {
            [a] => (a, a),
            [a, b, ..] => (a, b),
            [] => continue,
        };
        let between = if a.0 == b.0 {
            format!("no two of `{}`'s faces meet in an edge of `{solid}`", path(&a.0))
        } else {
            format!("`{}` and `{}` meet in no edge of `{solid}`", path(&a.0), path(&b.0))
        };
        // the pairs whose edge the word takes: two faces, one of `A` and one of `B` — what else a
        // `when` asked (a corner's third face against a face of its own side, a face against
        // itself) selects no edge of its own
        let ina: BTreeSet<&str> = a.1.iter().map(String::as_str).collect();
        let inb: BTreeSet<&str> = b.1.iter().map(String::as_str).collect();
        let asked = u.pairs.iter().map(|([x, y], layer)| ([x.as_str(), y.as_str()], *layer));
        // and those its blocks made no copy of, read apart (#994)
        let pruned = map.pruned.iter().filter(|p| p.at == u.at).flat_map(|p| p.pairs());
        let pairs: Vec<([&str; 2], usize)> = asked.chain(pruned)
            .filter(|([x, y], _)| x != y && (ina.contains(x) && inb.contains(y)
                || inb.contains(x) && ina.contains(y))).collect();
        let why = if pairs.is_empty() {
            if a.1.iter().chain(&b.1).collect::<BTreeSet<_>>().len() != 1 {
                continue;
            }
            format!("`{}` is one face, and an edge is where two meet", path(&a.0))
        } else {
            // nothing is said where a pair meets in an edge, or where the boundary cannot say
            let (layers, paths) = bodies.entry(e.i())
                .or_insert_with(|| (Layers::of(sk, e.i()), face_paths(sk, e.i())));
            let mut worst = Meeting::Apart;
            let mut eaten = BTreeSet::new();
            for (faces, layer) in pairs {
                let read = listed.entry((e.i(), layer))
                    .or_insert_with(|| layers.listed(sk, paths, layer));
                let Some(read) = read.as_ref() else { continue 'statements };
                eaten.extend(faces.into_iter().filter(|f| read.eaten(f)));
                match read.meeting(faces[0], faces[1]) {
                    Some(Meeting::Edge) | None => continue 'statements,
                    Some(m) => worst = worst.max(m),
                }
            }
            // a block every face of which the body took in says so before anything else
            let gone = [a, b].into_iter()
                .find(|blk| blk.1.iter().all(|f| eaten.contains(f.as_str())));
            match (gone, worst) {
                (Some(blk), _) => format!("none of `{}`'s faces is left on `{solid}`'s \
                    boundary, the body's other terms having taken them in", path(&blk.0)),
                (None, Meeting::Apart) => "they meet nowhere".to_string(),
                (None, Meeting::Corner) => "they meet only at a corner".to_string(),
                (None, _) => "they meet only where one goes on tangent into the other, a seam \
                    and no edge".to_string(),
            }
        };
        out.push(Diag::new(Code::W120, u.at, None,
            format!("{between}: {why}, so nothing is made over them")));
    }
    out
}

/// Each solid's faces where the drawing is solved, by its absolute name, where its boundary says:
/// which meet and how each lies, in each of its layers — a read is asked again in the layer it
/// was read in (#879).
type Read<'m> = BTreeMap<&'m str, Layered>;

/// One E080 for each statement and solid whose faces read apart meet where the drawing is solved:
/// the first such pair named, and how many more.
fn apart(map: &SourceMap, read: &Read) -> Vec<Diag> {
    // each pair once, whichever way round it was asked
    let mut said = BTreeSet::new();
    // every pair read apart: one by one, and those a block made no copy of (#994)
    let each = map.apart.iter().map(|a| (a.at, a.solid.as_str(), [a.faces[0].as_str(),
                                         a.faces[1].as_str()], a.layer))
        .chain(map.pruned.iter()
            .flat_map(|p| p.pairs().map(|(f, l)| (p.at, p.solid.as_str(), f, l))));
    let met = each.filter(|&(at, solid, [x, y], layer)| {
        said.insert((at, solid, x.min(y), x.max(y)))
            && read.get(solid).is_some_and(|r| r.meets(layer).meets(x, y))
    });
    grouped(met.map(|(at, solid, faces, _)| (at, solid, (solid, faces)))).into_iter()
        .map(|(at, (solid, [x, y]), n)| {
            Diag::new(Code::E080, at, None, format!(
                "`{}`'s faces `{x}` and `{y}` meet where the drawing solves, but not where it is \
                 seeded, where `meets` read them apart{}: what they would make is not made — seed \
                 the drawing nearer where it solves",
                public_path(solid), more(n, "pair")
            ))
        }).collect()
}

/// One E080 for each statement and solid with a `when` over faces read along a direction that,
/// decided again with what it read of faces as the drawing is solved, is made where it was not or
/// the other way round: the first read that came out otherwise named, and how many more.
fn along(sk: &Sketch, map: &SourceMap, read: &Read) -> Vec<Diag> {
    let solved = |r: &FaceRead| match r {
        FaceRead::Meets { solid, faces: [x, y], layer, held } => read.get(solid.as_str())
            .map_or(*held, |r| r.meets(*layer).meets(x, y)),
        FaceRead::Along(a) => (|| {
            let d = direction(sk, |n| map.ent_named(n), &a.line)?;
            read.get(a.solid.as_str())?.lays(a.layer).holds(&a.face, a.along, d)?.ok()
        })().unwrap_or(a.held),
        FaceRead::Fixed(held) => *held,
    };
    let turned = map.along.iter().flat_map(|w| {
        let now: Vec<bool> = w.reads.iter().map(solved).collect();
        let made = w.truth.value();
        let again = w.truth.holds(&|k| now[k]) != made;
        w.reads.iter().zip(now).filter(move |(r, now)| again && *now != r.held())
            .filter_map(move |(r, _)| Some((w.at, said(r)?, made)))
    });
    let turned = turned.map(|(at, (solid, what), made)| (at, solid, (solid, what, made)));
    grouped(turned).into_iter().map(|(at, (solid, what, made), n)| {
        let block = if made { "made" } else { "left out" };
        Diag::new(Code::E080, at, None, format!(
            "`{}`'s {what}{}: the `when` over it was decided at the seed, so what it makes is \
             {block} — seed the drawing nearer where it solves",
            public_path(solid), more(n, "face")
        ))
    }).collect()
}

/// The solid a read is of, and how it came out where the drawing is seeded and where it solves,
/// it having come out the other way there: nothing for one no pose changes.
fn said(r: &FaceRead) -> Option<(&str, String)> {
    Some(match r {
        FaceRead::Along(a) => {
            let (seeded, solved) = if a.held { ("", "not ") } else { ("not ", "") };
            (a.solid.as_str(), format!(
                "face `{}` is {seeded}`{} {}` where the drawing is seeded, but {solved}where it \
                 solves", a.face, a.along.word(), public_path(&a.line)
            ))
        }
        FaceRead::Meets { solid, faces: [x, y], held, .. } => {
            let (seeded, solved) = if *held { ("meet", "not") } else { ("do not meet", "do") };
            (solid.as_str(), format!(
                "faces `{x}` and `{y}` {seeded} where the drawing is seeded, but {solved} where it \
                 solves"
            ))
        }
        FaceRead::Fixed(_) => return None,
    })
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
/// frame's `u`, `v` or `normal` (`n`) under the frame's name.
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
        "n" | super::NORMAL => Some(b.normal()),
        _ => None,
    }
}
