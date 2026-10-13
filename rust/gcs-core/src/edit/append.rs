//! Adding: where an appended statement goes, the names it may take, and an appended text held
//! to the elaborator.

use super::*;

/* -- adding ------------------------------------------------------------------------ */

/// Where a new statement goes: the end of the root component's body.
///
/// The *end of its last statement*, not the end of the file — a program may have a trailing
/// comment, and a drawing tool should not write past it.
pub(super) fn append_at(prog: &Program) -> (Span, String) {
    let root = prog.root();
    // past the statement, or past the `in` block it was hoisted out of: an appended statement
    // writes its own membership, which inside the block would be said twice
    let after = |st: &syntax::Stmt| {
        let hi = prog.in_blocks.iter()
            .find(|b| b.header.lo <= st.span.lo && st.span.hi <= b.close.hi)
            .map_or(st.span.hi, |b| b.close.hi) as usize;
        (Span::new(hi, hi), "\n".to_string())
    };
    if let Some(preview) = prog.preview {
        if let Some(st) = root.body.iter().rev().find(|st| preview.contains(st.span.lo)) {
            return after(st);
        }
        let close = preview.hi as usize - 1;
        return (Span::new(close, close), "\n".into());
    }
    match root.body.last() {
        Some(st) => after(st),
        None => {
            let n = prog.text().len();
            let lead = if prog.text().ends_with('\n') || n == 0 { "" } else { "\n" };
            (Span::new(n, n), lead.to_string())
        }
    }
}

/// A name nothing has taken, in the kind's own alphabet: `p0`, `l1`, `c0`.
pub fn mint(prog: &Program, kind: EntKind) -> String {
    next_name(&mut taken_names(prog), kind)
}

/// Every name a statement in the program already binds — declarations, values, instances and
/// groups, component bodies included, and the `#a…` keys anonymous declarations carry among them.  What "already spoken for" means, said
/// once, beside the loop that consumes it.
pub(super) fn taken_names(prog: &Program) -> std::collections::BTreeSet<String> {
    prog.stmts().filter_map(|s| s.kind.bound_name().map(|n| n.text.clone())).collect()
}

/// The next name `taken` does not hold, in the kind's own alphabet — and now taken.  The one
/// spelling of the fresh-name rule: `mint` asks it for a caller, and `reconcile` asks it for a
/// new entity and again for an anonymous declaration a statement is about to reference.
pub(super) fn next_name(taken: &mut std::collections::BTreeSet<String>, kind: EntKind) -> String {
    let c = syntax::kind_initial(kind);
    let name = (0..)
        .map(|i| format!("{c}{i}"))
        .find(|n| !taken.contains(n))
        .expect("an unbounded sequence always holds a fresh name");
    taken.insert(name.clone());
    name
}

/// Append one statement, whatever it is.
pub(super) fn append(prog: &Program, kind: StmtKind, names: Vec<String>) -> Edit {
    match appended(prog, &[kind]) {
        Ok((text, _)) => Edit { text, kind: Kind::Structural, names, refused: None, kept: 0 },
        Err(e) => Edit::none(prog, Some(e)),
    }
}

/// The text with statements appended, one splice, and where in it they went.
fn appended(prog: &Program, stmts: &[StmtKind]) -> Result<(String, Span), String> {
    let (at, lead) = append_at(prog);
    let mut with = lead;
    for (i, k) in stmts.iter().enumerate() {
        if i > 0 { with.push('\n'); }
        syntax::write_stmt_to(&mut with, k).map_err(|e| e.to_string())?;
    }
    let span = Span::new(at.lo as usize, at.lo as usize + with.len());
    Ok((splice(prog.text(), vec![Splice { at, with }]), span))
}

/// `use NAME`, so the document reaches a module it did not — the workspace adds `use std` when
/// something is first drawn on one of the standard planes.  A line of its own, after the last
/// `use` the document writes, or else before its first statement (below a heading comment and
/// `unit`); nothing when the document already says it.
pub fn add_use(prog: &Program, name: &str) -> Edit {
    if prog.uses.iter().any(|u| u.name == name) {
        return Edit::none(prog, None);
    }
    Edit {
        text: splice(prog.text(), vec![use_line(prog, &format!("use {name}"))]),
        kind: Kind::Structural,
        names: Vec::new(),
        refused: None,
        kept: 0,
    }
}

/// A `use` line written where `add_use` puts one: after the last `use` the document writes, or
/// else before its first statement (below a heading comment and `unit`).
pub(super) fn use_line(prog: &Program, line: &str) -> Splice {
    let text = prog.text();
    let at = match prog.uses.iter().map(|u| u.span.hi as usize).filter(|&hi| hi <= text.len()).max() {
        // the end of the last `use`'s line
        Some(hi) => match text[hi..].find('\n') {
            Some(k) => hi + k + 1,
            None => text.len(),
        },
        // the start of the line the first statement or component definition begins on — not
        // `unit`, which a `use` reads after, and not the document's own anonymous root, which is
        // a component spanning the whole text
        None => {
            let root = prog.root();
            let first = root.body.iter()
                .filter(|st| !matches!(st.kind, StmtKind::Unit(_)))
                .map(|st| st.span.lo as usize)
                .chain(prog.components.iter().filter(|c| !std::ptr::eq(*c, root)).map(|c| c.span.lo as usize))
                .chain(prog.words.iter().filter(|d| d.module.is_none()).map(|d| d.span.lo as usize))
                .filter(|&lo| lo <= text.len())
                .min();
            match first {
                Some(lo) => text[..lo].rfind('\n').map_or(0, |k| k + 1),
                None => text.len(),
            }
        }
    };
    let lead = if at == text.len() && !text.is_empty() && !text.ends_with('\n') { "\n" } else { "" };
    Splice { at: Span::new(at, at), with: format!("{lead}{line}\n") }
}

/// Append several statements as one splice, then hold the result to the elaborator: the edit is
/// refused, with the elaborator's words, when anything it appended is an error.  An error
/// elsewhere in the document was there before the gesture and is not its fault.
pub(super) fn append_checked(prog: &Program, stmts: &[StmtKind], names: Vec<String>)
    -> Result<Edit, String> {
    let (text, ours) = appended(prog, stmts)?;
    checked(prog, text, ours, names)
}

/// A spliced text held to the elaborator: refused, in its words, for a parse error or an error
/// the elaborator raises inside `ours` (or at a statement that starts there) — the text the edit
/// wrote.  An error elsewhere was there before the edit and is not its fault.
pub(super) fn checked(prog: &Program, text: String, ours: Span, names: Vec<String>)
    -> Result<Edit, String> {
    checked_in(prog, text, &[ours], names)
}

/// `checked` over an edit that wrote in several places: refused for an error inside any of them.
pub(super) fn checked_in(prog: &Program, text: String, regions: &[Span], names: Vec<String>)
    -> Result<Edit, String> {
    let ours = |lo: u32| regions.iter().any(|r| r.contains(lo));
    let (mut next, errs) = syntax::parse(&text);
    if let Some(e) = errs.iter().find(|e| ours(e.span.lo)) {
        return Err(e.message.clone());
    }
    crate::modules::relink(&mut next, prog);
    let e = crate::program::elaborate(&next);
    let in_ours = |d: &crate::program::Diag| {
        ours(d.span.lo) || d.stmt.is_some_and(|id| {
            next.stmts().find(|s| s.id == id).is_some_and(|s| ours(s.span.lo))
        })
    };
    let refused = e.errors().find(|d| in_ours(d)).map(|d| d.message.clone());
    match refused {
        Some(why) => Err(why),
        None => Ok(Edit { text, kind: Kind::Structural, names, refused: None, kept: 0 }),
    }
}
