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
use crate::model::{EntRef, SolidDef, Sketch};
use crate::syntax::{Program, Span, StmtId};
use std::collections::{BTreeMap, BTreeSet};

/// Where a diagnostic of what a statement made is said: at its statement where the document holds
/// it, else at the deepest instance on its path the document holds (the round that used
/// `std.round`), else as it stands, for `modules::localize` to show at the `use`.
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
        // the statement ids of every statement, gathered only once a site is in a module
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

/// The refused solids, by index, in their own words: every solid but a body, validated.
fn refused(sk: &Sketch) -> BTreeMap<usize, String> {
    sk.solids.iter().enumerate()
        .filter(|(_, s)| !matches!(s.def, SolidDef::Body { .. }))
        .filter_map(|(i, _)| Some((i, crate::solid::validate(sk, i).err()?)))
        .collect()
}

/// What each solid is built on, which of them are refused, and which of those are roots.
pub(super) struct Causes<'a> {
    sk: &'a Sketch,
    refused: BTreeMap<usize, String>,
    on: Vec<Vec<usize>>,
    /// The refused solids that reach no other refused solid: each said in its own words.
    root: BTreeSet<usize>,
}

impl<'a> Causes<'a> {
    pub(super) fn new(sk: &'a Sketch) -> Causes<'a> {
        let on = (0..sk.solids.len()).map(|i| crate::solid::built_on(sk, i)).collect();
        let mut causes = Causes { sk, refused: refused(sk), on, root: BTreeSet::new() };
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

    /// What solid `i` is built on.
    fn on(&self, i: usize) -> &[usize] {
        &self.on[i]
    }
}

/// The notes that each statement whose solids follow from a refused one is not built (N080),
/// and every other diagnostic `solid_diagnostics` says of the solids, in the order: the refused
/// roots, each statement's note, then the faces a body bears on that it does not have.
pub(super) fn solids(sk: &Sketch, map: &SourceMap, p: &Program, place: &Placing) -> Vec<Diag> {
    let causes = Causes::new(sk);
    let mut roots: BTreeMap<usize, Diag> = BTreeMap::new();
    // each statement that is not built, by where it is said: its statement, its roots and the
    // solids named between
    let mut notes: BTreeMap<Span, (Option<StmtId>, BTreeSet<usize>, BTreeSet<String>)> =
        BTreeMap::new();
    let mut note = |at: (Span, Option<StmtId>), from: &[usize], under: Vec<usize>| {
        let via = causes.via(from, &under);
        let entry = notes.entry(at.0).or_insert_with(|| (at.1, BTreeSet::new(), BTreeSet::new()));
        entry.1.extend(under);
        entry.2.extend(via);
    };
    for (&i, message) in &causes.refused {
        let at = place.site(map.site_of(EntRef::solid(i)));
        let under: Vec<usize> =
            causes.roots(causes.on(i)).into_iter().filter(|&r| r != i).collect();
        if under.is_empty() {
            roots.insert(i, Diag::new(Code::E080, at.0, at.1, message.clone()));
        } else {
            note(at, causes.on(i), under);
        }
    }
    let mut bearing = Vec::new();
    for (b, message) in crate::solid::bearing_errors(sk) {
        let solid = b.solid as usize;
        // the body borne on, refused itself or built on what is
        let under = causes.roots(&[solid]);
        let at = (b.span, Some(StmtId(b.stmt)));
        if under.is_empty() {
            bearing.push(Diag::new(Code::E082, at.0, at.1, message));
        } else {
            note(at, &[solid], under);
        }
    }
    // a root is said as the document shows it before a note points at it
    let mut out: Vec<Diag> = roots.values().cloned().collect();
    crate::modules::localize(p, &mut out);
    let said: BTreeMap<usize, Span> =
        roots.keys().copied().zip(out.iter().map(|d| d.span)).collect();
    for (span, (stmt, under, via)) in notes {
        let named: Vec<String> = under.iter().map(|&r| {
            let (line, col) = crate::syntax::line_col(p.text(), said[&r].lo);
            format!("`{}` ({line}:{col})", sk.solids[r].name)
        }).collect();
        let named = match named.split_last() {
            Some((last, [])) => last.clone(),
            Some((last, rest)) => format!("{} and {last}", rest.join(", ")),
            None => continue,
        };
        let via: Vec<String> = via.iter().map(|v| format!("`{v}`")).collect();
        let message = if via.is_empty() {
            format!("not built: it is built on {named}, which could not be built")
        } else {
            format!("not built: it is built on {}, made of {named}, which could not be built",
                via.join(", "))
        };
        let mut d = Diag::new(Code::N080, span, stmt, message);
        d.cause = under.first().map(|r| said[r]);
        out.push(d);
    }
    out.extend(bearing);
    out
}
