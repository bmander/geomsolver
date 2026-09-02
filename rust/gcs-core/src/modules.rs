//! Modules: `use engine.crank` (Solvent §14.4).
//!
//! A module is a Solvent document read for its **components**.  `use NAME` at the top of a
//! document asks for one; what the name resolves to is the *host's* question — the core takes text
//! and has no filesystem — so `link` takes a resolver, `NAME -> text`, and the CLI answers it from
//! the files beside the document while the browser answers it from the library compiled into the
//! core (`library.rs`).  A module's own `use`s are followed the same way, once each, so a diamond
//! is one copy and a cycle is not a hang.
//!
//! What a module contributes is exactly its component definitions, plus the top-level `param`s
//! its components read (§6.3).  Its own drawing — whatever loose statements it holds — is its own
//! and is not drawn here, so `gear.sv` is a module as it stands.  Two definitions of one component
//! name, wherever they come from, are refused: there is no shadowing (§5).
//!
//! **Every span stays one integer.**  A module is parsed at a `base` past the end of everything
//! linked before it (`syntax::parse_from`), so its statements' spans and ids join the document's
//! without a second coordinate anywhere; `Program::source_at` says which text an offset is in, and
//! `program::elaborate` shows a module's diagnostic at the `use` that brought the module in, its
//! own place named in the message.

use crate::program::{Code, Diag};
use crate::syntax::{parse_from, Component, Module, Program, Span};
use std::collections::{BTreeSet, VecDeque};

/// The most modules one document may link, transitively — the same kind of bound `MAX_STMTS` is.
pub const MAX_MODULES: usize = 64;

/// Resolve every `use` in `prog` through `resolve`, and every `use` inside what it brings in.
///
/// The modules' components join `prog.components` before the root.  Diagnostics come back as
/// the elaborator's: a module nothing resolves (E070, at the `use`), a component defined twice
/// (E071, at the later definition), and a module's own parse errors (E100, at their place in the
/// module).  Nothing is said twice: a module asked for by two `use`s is linked once.
pub fn link(prog: &mut Program, resolve: &mut dyn FnMut(&str) -> Option<String>) -> Vec<Diag> {
    let mut diags = Vec::new();
    // (name, the `use` that asked, the document's `use` this descends from)
    let mut queue: VecDeque<(String, Span, Span)> =
        prog.uses.iter().map(|u| (u.name.clone(), u.span, u.span)).collect();
    let mut seen: BTreeSet<String> = BTreeSet::new();
    while let Some((name, at, via)) = queue.pop_front() {
        if !seen.insert(name.clone()) {
            continue;
        }
        if prog.modules.len() >= MAX_MODULES {
            diags.push(Diag {
                code: Code::E070,
                span: at,
                stmt: None,
                message: format!("a document may not link more than {MAX_MODULES} modules"),
            });
            break;
        }
        let Some(text) = resolve(&name) else {
            diags.push(Diag {
                code: Code::E070,
                span: at,
                stmt: None,
                message: format!("no module `{name}`"),
            });
            continue;
        };
        let base = prog.virtual_len();
        let (mp, errs) = parse_from(&text, base, prog.next_stmt());
        prog.set_next_stmt(mp.next_stmt());
        for e in errs {
            diags.push(Diag { code: Code::E100, span: e.span, stmt: None, message: e.message });
        }
        let k = prog.modules.len();
        let mut comps = mp.components;
        // the module's root is last, as every program's is — where it has one: a file holding
        // nothing loose has its last *component* standing there (`Program::root`), and that is
        // a component to keep.  The root's params are kept; its drawing is not.
        let root = match comps.last() {
            Some(c) if c.name.is_none() => comps.pop().unwrap_or_default(),
            _ => Component::default(),
        };
        for mut c in comps {
            c.module = Some(k);
            let Some(cname) = c.name.as_ref() else { continue };
            if let Some(other) = prog.components.iter().find(|o| {
                o.name.as_ref().is_some_and(|n| n.text == cname.text)
            }) {
                // at the definition a reader of the document can edit — its own, where the
                // clash is with one; otherwise the module's, shown at the `use`
                let (span, where_) = match other.module {
                    Some(j) => (c.span, format!("in `{}`", prog.modules[j].name)),
                    None => (other.span, "in this document".to_string()),
                };
                diags.push(Diag {
                    code: Code::E071,
                    span,
                    stmt: None,
                    message: format!(
                        "`{}` is defined twice: in `{name}` and {where_}",
                        cname.text
                    ),
                });
                continue;
            }
            let root_at = prog.components.len() - 1;
            prog.components.insert(root_at, c);
        }
        for u in &mp.uses {
            queue.push_back((u.name.clone(), u.span, via));
        }
        let uses = mp.uses.iter().map(|u| u.name.clone()).collect();
        prog.modules.push(Module { name, text, base, via, root, uses });
    }
    localize(prog, &mut diags);
    diags
}

/// A diagnostic inside a module, shown to a reader of the document: at the `use` that brought
/// the module in, with the module's own name, line and column in front of the message.  One
/// wording, so the CLI and the panel cannot describe one fault two ways; and one place, so no
/// consumer of a span ever meets an offset the document does not have.
pub fn localize(p: &Program, diags: &mut [Diag]) {
    for d in diags.iter_mut() {
        if let (Some(k), local) = p.source_at(d.span.lo as usize) {
            let m = &p.modules[k];
            let (line, col) = crate::syntax::line_col(&m.text, local as u32);
            d.message = format!("{}:{line}:{col}: {}", m.name, d.message);
            d.span = m.via;
            d.stmt = None;
        }
    }
}

/// Link `prog` exactly as `like` was linked — from the module texts `like` already holds — for
/// a re-parse of the same document that must not ask the host again.
pub fn relink(prog: &mut Program, like: &Program) -> Vec<Diag> {
    link(prog, &mut |name| like.module_text(name))
}

/// The component a program defines under `name`, wherever it was read from.
pub fn component<'a>(prog: &'a Program, name: &str) -> Option<&'a Component> {
    prog.components.iter().find(|c| c.name.as_ref().is_some_and(|n| n.text == name))
}
