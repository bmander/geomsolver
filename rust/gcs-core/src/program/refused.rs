//! **One cause, said once** (#572): the solids the solve leaves unbuildable, said where the fault
//! is and nowhere else in its words.
//!
//! A solid is refused (E080) in its own words only where no solid it is built on is refused
//! (`solid::built_on`, read through bodies, which are never refused themselves: their operands
//! are).  Every other refused solid follows from one that is, so the statement that made it says
//! once that it is not built (N080), naming the refused solids at the root and pointing at the
//! first (`Diag::cause`) — however many solids it made: a round is a `taken` and a `filled` for
//! every edge it reaches, and none of them is a diagnostic of its own.  A face a body bears on
//! (E082) of a body that is not built follows the same way.  Each is said where the document holds
//! it (`Placing`): a round's pieces at the round, not inside the word `std` defines for it.

use super::{Code, Diag, Site, SourceMap};
use crate::model::{EntKind, EntRef, SolidDef, Sketch};
use crate::syntax::{Program, Span, StmtId};
use std::collections::{BTreeMap, BTreeSet};

/// Where a diagnostic of what a statement made is said: at its statement where the document holds
/// it, else at the deepest instance on its path the document holds (the round that used
/// `std.rounds.round`), else as it stands, for `modules::localize` to show at the `use`.
pub(super) struct Placing<'a> {
    p: &'a Program,
    spans: std::cell::OnceCell<BTreeMap<StmtId, Span>>,
}

impl<'a> Placing<'a> {
    pub(super) fn new(p: &'a Program) -> Placing<'a> {
        Placing { p, spans: Default::default() }
    }

    fn in_document(&self, span: Span) -> bool {
        self.p.source_at(span.lo as usize).0.is_none()
    }

    /// `site`'s span and statement as the document says them; nothing where there is no site.
    pub(super) fn site(&self, site: Option<&Site>) -> (Span, Option<StmtId>) {
        let Some(site) = site else { return (Span::default(), None) };
        if self.in_document(site.span) {
            return (site.span, Some(site.stmt));
        }
        // where every statement is, gathered only once a site is in a module
        let spans = self.spans.get_or_init(|| self.p.stmts().map(|st| (st.id, st.span)).collect());
        site.path.0.iter().rev()
            .filter_map(|step| match step {
                crate::ir::PathStep::Instance(st) => Some((*spans.get(st)?, *st)),
                crate::ir::PathStep::Copy { .. } => None,
            })
            .find(|&(span, _)| self.in_document(span))
            .map_or((site.span, Some(site.stmt)), |(span, st)| (span, Some(st)))
    }
}

/// **Where a refusal is said** (#881): the document's statement that made the solid at the root
/// of its words — a refusal of what is built on another reads as a chain, `` `a`: `b`'s faces:
/// `c`: why ``, and the innermost solid it names so (`` `c`: `` or `` `c`'s ``) is the cause —
/// placed as `solid_diagnostics` places it (`Placing`: what a word of a module made, at the
/// document's statement that used it), so a round's piece refused deep in an export (`` `b.#…
/// filled`: its exact construction disagrees with the field ``) is said at its round, and a
/// chamfer read over a body that round left unbuilt is said there too (#948).  A solid is named
/// by its key or as the document calls it (`Sketch::solid_name`): copies a name goes by alike (a
/// block's within a block's copy) are made by one statement, so placed there; a name solids made
/// apart go by (two rounds over one body each make a `filled[3]`) places none, so a refusal calls
/// such a solid by its key (`Sketch::solid_said`).  `None` where the message names no solid, or
/// none the document holds.
pub fn refusal_site(sk: &Sketch, map: &SourceMap, p: &Program, message: &str)
    -> Option<(Span, Option<StmtId>)> {
    refusal_cause(sk, map, p, message).map(|(span, stmt, _)| (span, stmt))
}

/// `refusal_site`, with where in `message` its cause's words begin: the backtick opening the
/// innermost solid's name, from which on the message says the cause alone.
pub fn refusal_cause(sk: &Sketch, map: &SourceMap, p: &Program, message: &str)
    -> Option<(Span, Option<StmtId>, usize)> {
    let place = Placing::new(p);
    let site = |i: usize| map.site_of(EntRef::solid(i)).map(|s| place.site(Some(s)));
    let solid = |name: &str| {
        if let Some(i) = sk.solids.iter().position(|s| s.name == name) {
            return site(i);
        }
        let mut sites = (0..sk.solids.len()).filter(|&i| sk.solid_name(i) == name).map(site);
        let first = sites.next()??;
        sites.all(|s| s == Some(first)).then_some(first)
    };
    let names = backticked(message);
    let chained = names.iter().rev().filter(|n| n.2);
    let (span, stmt, from) = chained.chain(names.iter().filter(|n| !n.2))
        .find_map(|&(open, name, _)| solid(name).map(|(span, stmt)| (span, stmt, open)))?;
    place.in_document(span).then_some((span, stmt, from))
}

/// Each name between backticks in `message`, where its opening backtick stands, and whether a
/// chain of refusals goes on through it (`` `x`: `` or `` `x`'s ``).
fn backticked(message: &str) -> Vec<(usize, &str, bool)> {
    let mut names = Vec::new();
    let mut at = 0;
    while let Some(open) = message[at..].find('`').map(|k| at + k) {
        let Some(close) = message[open + 1..].find('`').map(|k| open + 1 + k) else { break };
        let after = &message[close + 1..];
        let chained = after.starts_with(": ") || after.starts_with("'s ");
        names.push((open, &message[open + 1..close], chained));
        at = close + 1;
    }
    names
}

/// The innermost solid a refusal's chain names (`refusal_cause`), as a name picks one
/// (`Sketch::solid_called`), and where its words begin.
fn chained_cause(sk: &Sketch, message: &str) -> Option<(usize, usize)> {
    backticked(message).into_iter().rev().filter(|n| n.2)
        .find_map(|(open, name, _)| Some((sk.solid_called(name).ok()?, open)))
}

/// What each solid is built on, which of them are refused (in their own words), and which of
/// those are roots.
struct Causes<'a> {
    sk: &'a Sketch,
    refused: BTreeMap<usize, String>,
    on: Vec<Vec<usize>>,
    /// The refused solids that reach no other refused solid: each said in its own words.
    root: BTreeSet<usize>,
}

impl<'a> Causes<'a> {
    fn new(sk: &'a Sketch) -> Causes<'a> {
        // every solid but a body, refused where it is not built (`solid::refusal`: a sweep whose
        // field cannot be read too, #588): a body's refusal is its operands'
        let mut refused: BTreeMap<usize, String> = sk.solids.iter().enumerate()
            .filter(|(_, s)| !matches!(s.def, SolidDef::Body { .. }))
            .filter_map(|(i, _)| Some((i, crate::solid::refusal(sk, i).err()?)))
            .collect();
        let mut on: Vec<Vec<usize>> =
            (0..sk.solids.len()).map(|i| crate::solid::built_on(sk, i)).collect();
        // and on the solid its own words lead to (#948): a round's piece reading the faces of a
        // body another round left unbuilt says `` `x`: `part`'s faces: `filled[3]`: … ``, built
        // on that one though no operand of it is — refused in those words, where the kernel met
        // it only building the body's faces, and only where it is not built alone either (a
        // swept body's operand whose field is unread is built, and the sweep is the fault)
        // — never on itself: a stock split off its body (`as_body`) goes by the body's name, which
        // picks the body, so its own `` `part`: … `` led to the body built on it, and with the
        // body's rounds built on that, nothing was a root and every piece said it again (#962)
        let led: Vec<(usize, usize, String)> = refused.iter().filter_map(|(&i, message)| {
            let (j, from) = chained_cause(sk, message)
                .filter(|&(j, _)| j != i && sk.solids[j].name != sk.solids[i].name)?;
            let unbuilt = refused.contains_key(&j) || sk.exact_preimage(j).is_err();
            unbuilt.then(|| (i, j, message[from..].to_string()))
        }).collect();
        for (i, j, words) in led {
            refused.entry(j).or_insert(words);
            on[i].push(j);
        }
        let mut causes = Causes { sk, refused, on, root: BTreeSet::new() };
        causes.root = causes.refused.keys().copied()
            .filter(|&i| causes.below(&causes.on[i]).iter().all(|&j| j == i))
            .collect();
        causes
    }

    /// The refused solids among `from` and everything they are built on, all the way down.
    fn below(&self, from: &[usize]) -> BTreeSet<usize> {
        let mut seen = BTreeSet::new();
        let mut pending = from.to_vec();
        while let Some(i) = pending.pop() {
            if i < self.on.len() && seen.insert(i) {
                pending.extend(&self.on[i]);
            }
        }
        seen.into_iter().filter(|i| self.refused.contains_key(i)).collect()
    }

    /// The roots a solid built on `from` follows from: none where none of what it reaches is
    /// refused, or where the refused solids it reaches are built on one another all round.
    fn roots(&self, from: &[usize]) -> Vec<usize> {
        self.below(from).into_iter().filter(|j| self.root.contains(j)).collect()
    }

    /// The solids a reader names between `from` and its roots: the first named solid on each walk
    /// from `from` that is no root (`body`, which a round's piece reads the faces of), none for
    /// one built on its root directly.  A solid a statement made for another (`#…`) is walked
    /// through.
    fn via(&self, from: &[usize], roots: &[usize]) -> BTreeSet<String> {
        let mut seen = BTreeSet::new();
        let mut out = BTreeSet::new();
        let mut pending = from.to_vec();
        while let Some(i) = pending.pop() {
            if i >= self.on.len() || !seen.insert(i) || roots.contains(&i) {
                continue;
            }
            let name = &self.sk.solids[i].name;
            if name.contains('#') {
                pending.extend(&self.on[i]);
            } else {
                out.insert(name.clone());
            }
        }
        out
    }
}

/// A statement that is not built: what it follows from, and the solids named between.
#[derive(Default)]
struct Unbuilt {
    stmt: Option<StmtId>,
    roots: BTreeSet<Root>,
    via: BTreeSet<String>,
}

/// **What a statement's solids follow from** (#572, #962): a solid refused in its own words, or a
/// face of the drawing no solid can be made of (`solid::face_fault`) — an operand's fault, said
/// once at the face however many solids are swept from it, each of which says only that it is
/// not built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Root {
    Solid(usize),
    Face(usize),
}

/// **What a refused solid is built from that the drawing leaves free** (#996): the numbers of the
/// entities its term is written over (`Sketch::children`, read through, and each one's own
/// numbers, `Sketch::own_params`) that the solve could still move — `Diagnosis::under_params`,
/// read structurally — each named as the document names its entity and with the value its seed
/// left it at, and the freedoms of the parts of the drawing they are in. Nothing where every number
/// the solid reads is held. A solid refused over a near-miss the drawing never meant (a turn's tube
/// a millionth wider than the legs tangent to it, its arc's radius said by no dimension and left
/// where its ends' hints put it) is so told why its numbers are what they are.
pub(super) fn free_beneath(sk: &Sketch, map: &SourceMap, solid: usize, under: &Under) -> String {
    let mut seen = BTreeSet::new();
    let mut stack = vec![EntRef::solid(solid)];
    let mut named: Vec<(String, f64)> = Vec::new();
    let mut params = BTreeSet::new();
    while let Some(e) = stack.pop() {
        if !seen.insert(e) {
            continue;
        }
        stack.extend(sk.children(e));
        let free: Vec<u32> = sk.own_params(e).into_iter().filter(|q| under.params.contains(q)).collect();
        if free.is_empty() {
            continue;
        }
        let Some(name) = map.name_of(e) else { continue };
        // each number by its scalar's name, paired as `report::positions_where` pairs them
        let all = sk.entity_params(e);
        let names = sk.scalar_names(e, &crate::vocab::public_path(name)).filter(|n| n.len() == all.len());
        for q in free {
            let said = names.as_ref().and_then(|n| all.iter().position(|&p| p == q).map(|k| n[k].clone()))
                .unwrap_or_else(|| name.clone());
            if params.insert(q) {
                named.push((said, sk.params[q as usize].value));
            }
        }
    }
    if named.is_empty() {
        return String::new();
    }
    named.sort_by(|a, b| a.0.cmp(&b.0));
    let unit = sk.units.name();
    let list: Vec<String> = named.iter().map(|(n, v)| format!("`{n}` at {}{unit}", crate::json::fmt_g(*v, 10))).collect();
    let freedom: i64 = under.components.iter().filter(|(ps, _)| ps.iter().any(|q| params.contains(q))).map(|(_, d)| *d).sum();
    let list = crate::vocab::all_of(&list);
    let (stand, hints, them, meant) = if named.len() == 1 { ("stands", "its hint", "it", "the number") }
        else { ("stand", "their hints", "them", "the numbers") };
    let degrees = if freedom == 1 { "degree" } else { "degrees" };
    let among = if named.len() == 1 { "" } else { " among them" };
    format!("; what it is built from is not all held by the drawing: {list} {stand} where \
        {hints} left {them} ({freedom} {degrees} of freedom{among}), so its shape is the seeds', \
        not the drawing's — state {meant} meant")
}

/// `Free::with_free` of a drawing asked once: a refusal said alone.
pub fn with_free(sk: &Sketch, map: &SourceMap, solid: usize, words: String) -> String {
    Free::default().with_free(sk, map, solid, words)
}

/// **What a drawing leaves free, read once for the refusals a front end says of it** (#996): the
/// structural reading (`Under`) made the first time a refusal asks, and kept for the rest of the
/// run — one diagnosis of a copy of the drawing a run, not one an export refused.
#[derive(Default)]
pub struct Free(std::cell::OnceCell<Under>);

impl Free {
    /// **A refusal of solid `solid`'s build in the words a reader is given**: `words`, and what the
    /// solid is built from that the drawing `sk` leaves free (`free_beneath`), where anything is —
    /// what a front end says of an export it was refused, as `solids` says a refused root.  `sk`
    /// is the one drawing every refusal of the run is of.
    pub fn with_free(&self, sk: &Sketch, map: &SourceMap, solid: usize, words: String) -> String {
        words + &free_beneath(sk, map, solid, self.0.get_or_init(|| Under::read(sk)))
    }
}

/// The numbers the solve could still move, and each part of the drawing's: its numbers and its
/// freedom (`free_beneath`), read structurally, once, only where a solid is refused.
pub(super) struct Under {
    params: BTreeSet<u32>,
    components: Vec<(Vec<u32>, i64)>,
}

impl Under {
    pub(super) fn read(sk: &Sketch) -> Under {
        let opts = crate::diagnose::DiagnoseOptions { numeric: Some(false), conflicts: Some(false),
            solid_claims: false, refusals: false, ..crate::diagnose::DiagnoseOptions::default() };
        let d = crate::diagnose::diagnose(&mut sk.clone(), opts);
        Under { params: d.under_params.iter().copied().collect(),
                components: d.components.iter().map(|c| (c.params.clone(), c.dof)).collect() }
    }
}

/// **A face's fault in the document's words** (#962): the face, the edge of no length by its name
/// and its ends' (a line's two points, which coincide or how far apart they are), or why its loops
/// bound no region — and that no solid can be made of it.
fn face_said(sk: &Sketch, map: &SourceMap, fi: usize, fault: &crate::solid::FaceFault) -> String {
    use crate::solid::FaceFault;
    let called = |e: EntRef| super::rings::called(map, e);
    let face = called(EntRef::new(EntKind::Face, fi));
    let length = |x: f64| crate::units::Dim::LENGTH.number_text(x, sk.units);
    match fault {
        FaceFault::NoLength { edge, length: long, .. } => {
            let point = |p: u32| called(EntRef::new(EntKind::Point, p as usize));
            let how = match edge.kind {
                EntKind::Line => {
                    let l = &sk.lines[edge.i()];
                    let apart = if *long == 0.0 { "which coincide".to_string() }
                                else { format!("which are {} apart", length(*long)) };
                    format!("it runs from {} to {}, {apart}", point(l.p1), point(l.p2))
                }
                EntKind::Arc => {
                    let a = &sk.arcs[edge.i()];
                    format!("it turns from {} to {} about {}, {} long", point(a.start),
                            point(a.end), point(a.center), length(*long))
                }
                _ => format!("it is {} long", length(*long)),
            };
            format!("{face}'s edge {} has no length: {how}, so no solid can be made of {face}",
                    called(*edge))
        }
        FaceFault::Outline(why) => format!("no solid can be made of {face}: {why}"),
    }
}

/// The refused roots in their own words, then the note that each statement whose solids follow
/// from one is not built (N080), then the faces a body bears on that it does not have.  A root
/// swept from a face that has a fault of its own (`solid::face_fault`) follows from the face: the
/// fault is said once at the face, and the solid's statement says it is not built.
pub(super) fn solids(sk: &Sketch, map: &SourceMap, place: &Placing) -> Vec<Diag> {
    let causes = Causes::new(sk);
    // each root swept from a face with a fault (a prism, a prism through everything, a
    // revolution: `Solid::face`), by the face (#962), each face's fault read once
    let mut faults: BTreeMap<usize, Option<crate::solid::FaceFault>> = BTreeMap::new();
    let faulted: BTreeMap<usize, (usize, crate::solid::FaceFault)> = causes.root.iter()
        .filter_map(|&i| {
            let f = sk.solids[i].face()? as usize;
            let fault = faults.entry(f).or_insert_with(|| crate::solid::face_fault(sk, f));
            Some((i, (f, fault.clone()?)))
        })
        .collect();
    let key = |i: usize| faulted.get(&i).map_or(Root::Solid(i), |&(f, _)| Root::Face(f));
    // what the solve leaves free, read only once a root is refused
    let under_read = std::cell::OnceCell::new();
    let mut roots: BTreeMap<Root, Diag> = BTreeMap::new();
    // each statement not built, by where it is said
    let mut unbuilt: BTreeMap<Span, Unbuilt> = BTreeMap::new();
    let mut note = |(span, stmt): (Span, Option<StmtId>), from: &[usize], under: Vec<usize>| {
        let u = unbuilt.entry(span).or_insert_with(|| Unbuilt { stmt, ..Unbuilt::default() });
        u.via.extend(causes.via(from, &under));
        u.roots.extend(under.into_iter().map(key));
    };
    for (&i, message) in &causes.refused {
        let at = place.site(map.site_of(EntRef::solid(i)));
        let under = if causes.root.contains(&i) { Vec::new() } else { causes.roots(&causes.on[i]) };
        if let Some((f, fault)) = faulted.get(&i) {
            // the face's fault, at the face, once; the solid swept from it is not built
            let face = place.site(map.site_of(EntRef::new(crate::model::EntKind::Face, *f)));
            let (span, stmt) = if face.1.is_some() { face } else { at };
            roots.entry(Root::Face(*f)).or_insert_with(|| {
                let free = free_beneath(sk, map, i, under_read.get_or_init(|| Under::read(sk)));
                Diag::new(Code::E080, span, stmt, format!("{}{free}", face_said(sk, map, *f, fault)))
            });
            // (a stock split off its body has no statement of its own: the body's says it)
            if at.1.is_some() && span != at.0 {
                note(at, &[], vec![i]);
            }
        } else if under.is_empty() {
            // a solid with no root below it (refused solids built on one another all round) is
            // said in its own words as well, as the document reads it: a key a kernel or a shared
            // name put in it, its path
            let free = free_beneath(sk, map, i, under_read.get_or_init(|| Under::read(sk)));
            roots.insert(Root::Solid(i),
                         Diag::new(Code::E080, at.0, at.1, format!("{}{free}", crate::vocab::public_words(message))));
        } else {
            note(at, &causes.on[i], under);
        }
    }
    let mut bearing = Vec::new();
    for (b, message) in crate::solid::bearing_errors(sk) {
        // the body borne on, refused itself or built on what is
        let solid = b.solid as usize;
        let under = causes.roots(&[solid]);
        if under.is_empty() {
            bearing.push(Diag::new(Code::E082, b.span, Some(StmtId(b.stmt)), message));
        } else {
            note((b.span, Some(StmtId(b.stmt))), &[solid], under);
        }
    }
    // a statement whose `when a meets b` read a body not built where the drawing stands (#994),
    // and not read again where the drawing solves or not built there either: no copy made for
    // its faces, so the statement is not built, as built on that body — or, the body built where
    // the drawing stands now but its faces not read here, said so at the statement
    for u in &map.unbuilt {
        let Some(e) = map.ent_named(&u.solid).filter(|e| e.kind == EntKind::Solid) else { continue };
        let under = causes.roots(&[e.i()]);
        if under.is_empty() {
            bearing.push(Diag::new(Code::E080, u.at, None, format!(
                "`{}`'s faces are not read where they are named: its boundary is not built where \
                 the drawing is seeded ({}), and they were not read again where it solves (it \
                 does not solve, or is refused besides), so nothing of this statement is made",
                crate::vocab::public_path(&u.solid), crate::vocab::public_words(&u.why))));
        } else {
            note((u.at, None), &[e.i()], under);
        }
    }
    // a root is said as the document shows it before a note points at it
    let (ids, mut out): (Vec<Root>, Vec<Diag>) = roots.into_iter().unzip();
    // a statement a root is said at says nothing more (a body and the stock split off it, #962)
    let at_root: BTreeSet<Span> = out.iter().map(|d| d.span).collect();
    crate::modules::localize(place.p, &mut out);
    let said: BTreeMap<Root, Span> = ids.into_iter().zip(out.iter().map(|d| d.span)).collect();
    for (span, u) in unbuilt.into_iter().filter(|(span, _)| !at_root.contains(span)) {
        let named: Vec<String> = u.roots.iter().map(|r| {
            let (line, col) = crate::syntax::line_col(place.p.text(), said[r].lo);
            // its path, as the document reads it: where it is said tells it from a namesake
            let name = match *r {
                Root::Solid(i) => crate::vocab::public_path(&sk.solids[i].name),
                Root::Face(f) => crate::vocab::public_path(&sk.faces[f].name),
            };
            format!("`{name}` ({line}:{col})")
        }).collect();
        let named = match named.split_last() {
            Some((last, [])) => last.clone(),
            Some((last, rest)) => format!("{} and {last}", rest.join(", ")),
            None => continue,
        };
        let message = if u.via.is_empty() {
            format!("not built: it is built on {named}, which could not be built")
        } else {
            let via: Vec<String> = u.via.iter().map(|v| format!("`{v}`")).collect();
            format!("not built: it is built on {}, made of {named}, which could not be built",
                via.join(", "))
        };
        let mut d = Diag::new(Code::N080, span, u.stmt, message);
        d.cause = u.roots.first().map(|r| said[r]);
        out.push(d);
    }
    out.extend(bearing);
    out
}
