//! **Relation words** (§9.9): a statement written with a word a file defines is that word's body
//! **applied** (#103, `flatten::apply`): its operands are the word's bound variables, each an
//! alias of what the statement wrote; its parameters, given by label, are numbers bound in the
//! body's scope or selector words put where the body's words stand; and the body is closed over
//! them as a component's is.  A body is one relation, or several and the declarations they need.
//!
//! What the application makes is the statement's: its id, span, placement and classes, every
//! place in it the word's as written — so a fault inside it is said where the word is used — and
//! a dimension that is a parameter at the argument it was given, where its callout reads it.
//! The word itself is kept on each relation (`Relation::word`), so a culprit reads `p horizontal
//! q`.

use super::*;
use super::apply::Use;
use std::rc::Rc;
use crate::constraints::{builtin_word, Fixity};
use crate::syntax::{Arg, OpArg, Relation, WordDef, Worded, Written};

/// How a word's parameter is read in its body: a number bound in its scope, or a selector word
/// put where a word stands (`side: s`, `level(s)`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Param {
    Number,
    Word,
}

/// Every reference a written relation holds: its operands, the entities in its parentheses and a
/// direction named by reference.
pub(super) fn refs_mut(w: &mut Written) -> Vec<&mut Ref> {
    let mut out: Vec<&mut Ref> = w.ops.iter_mut().collect();
    for a in w.args.iter_mut() {
        match a {
            OpArg::Ent(r) | OpArg::Named(_, Arg::Ref(r)) => out.push(r),
            _ => {}
        }
    }
    out
}

/// Every number a written relation holds as text, with its span: the dimension, a defined word's
/// argument, a pin or a seed written as an expression.
pub(super) fn texts_mut(w: &mut Written) -> Vec<(&mut String, &mut Span)> {
    fn arg(a: &mut Arg) -> Option<(&mut String, &mut Span)> {
        match a {
            Arg::Dim { text, span } | Arg::SeedExpr { text, span, .. } => Some((text, span)),
            _ => None,
        }
    }
    let mut out = Vec::new();
    for a in w.args.iter_mut() {
        match a {
            OpArg::Dim(text, span) => out.push((text, span)),
            OpArg::Named(_, v) | OpArg::Slot { arg: v, .. } => out.extend(arg(v)),
            OpArg::Vector { parts, .. } => out.extend(parts.iter_mut().filter_map(arg)),
            OpArg::Ent(_) => {}
        }
    }
    out
}

/// The names a number's text reads, as the expression parser sees them.  Read in millimetres so
/// a literal naming its unit parses whatever the file's unit is: only the names are wanted.
fn reads(text: &str) -> BTreeSet<String> {
    let units = Units::with_length("mm").unwrap_or_default();
    expr::parse_in(text, units).map(|p| p.body.deps()).unwrap_or_default()
}

/// Every relation a word's body writes.
fn relations(def: &WordDef) -> impl Iterator<Item = &Written> {
    def.body.iter().filter_map(|st| match &st.kind {
        StmtKind::Relation(r) => r.form.written(),
        _ => None,
    })
}

/// How each parameter of a word is read in its body — or, for one read both ways, the fault.
fn param_kinds(def: &WordDef) -> Result<BTreeMap<String, Param>, (Span, String)> {
    let mut out: BTreeMap<String, Param> = BTreeMap::new();
    let mut said = |name: &str, how: Param, span: Span| -> Result<(), (Span, String)> {
        if !def.params.iter().any(|p| p.text == name) {
            return Ok(());
        }
        match out.insert(name.to_string(), how) {
            Some(was) if was != how => Err((span, format!(
                "`{name}` is read as a number and as a word in `{}`; a parameter is one or the \
                 other",
                def.word.text
            ))),
            _ => Ok(()),
        }
    };
    for w in relations(def) {
        let mut w = w.clone();
        for a in &w.args {
            match a {
                OpArg::Named(_, Arg::Word(v)) => said(v, Param::Word, def.word.span)?,
                OpArg::Ent(r) | OpArg::Named(_, Arg::Ref(r)) if r.path.is_empty() => {
                    said(&r.root.text, Param::Word, r.span)?
                }
                _ => {}
            }
        }
        for (text, span) in texts_mut(&mut w) {
            for name in reads(text) {
                said(&name, Param::Number, *span)?;
            }
        }
    }
    for p in &def.params {
        out.entry(p.text.clone()).or_insert(Param::Number);
    }
    Ok(out)
}

/// **What is wrong with a word's definition, as written** (§9.9): a word the language keeps for
/// itself, an operand or a parameter named twice, and a body reading anything it was neither
/// given nor declared itself.  Asked of every definition by `program::words`, used or not; the
/// application asks it too, and applies no definition it faults, so a fault is said once, at the
/// definition.
pub(crate) fn faults(def: &WordDef) -> Vec<(Code, Span, String)> {
    let mut out = Vec::new();
    let word = &def.word.text;
    let place = match def.fixity {
        Fixity::Infix => "between two operands",
        _ => "before its operand",
    };
    if crate::syntax::reserved_word(word) {
        out.push((Code::E071, def.word.span, format!(
            "`{word}` is a word of the language's own, and no relation word may be defined as it"
        )));
    } else if builtin_word(word, def.fixity) {
        out.push((Code::E071, def.word.span, format!(
            "`{word}` is already a constraint word written {place}; a relation word may not be \
             defined over one"
        )));
    }
    let mut seen: BTreeSet<&str> = BTreeSet::new();
    for n in def.operands.iter().chain(&def.params) {
        if !seen.insert(n.text.as_str()) {
            out.push((Code::E001, n.span, format!(
                "`{}` names two of `{word}`'s operands or parameters", n.text
            )));
        }
    }
    let kinds = match param_kinds(def) {
        Ok(k) => k,
        Err((span, m)) => {
            out.push((Code::E040, span, m));
            return out;
        }
    };
    // what the body declares for itself (`private m := point`, a set written in place) it may
    // read
    let locals: BTreeSet<&str> = def.body.iter().filter_map(|st| match &st.kind {
        StmtKind::Decl(d) => Some(d.name.key().text.as_str()),
        other => other.bound_name().map(|n| n.text.as_str()),
    }).collect();
    let reach = |name: &str| format!(
        "`{name}` is not an operand or a parameter of `{word}`: a relation word reads only what \
         it is given (§5)"
    );
    let known = |r: &Ref| {
        let root = r.root.text.as_str();
        def.operands.iter().any(|o| o.text == root)
            || locals.contains(root)
            || (r.path.is_empty() && kinds.get(root) == Some(&Param::Word))
            || r.direction_word().is_some()
    };
    for st in &def.body {
        match &st.kind {
            StmtKind::Relation(rel) => {
                let Some(w) = rel.form.written() else { continue };
                let mut w = w.clone();
                for r in refs_mut(&mut w) {
                    if !known(r) {
                        out.push((Code::E101, r.span, reach(&crate::syntax::ref_text(r))));
                    }
                }
                for (text, span) in texts_mut(&mut w) {
                    for name in reads(text) {
                        if !def.params.iter().any(|p| p.text == name) {
                            out.push((Code::E101, *span, reach(&name)));
                        }
                    }
                }
            }
            StmtKind::Decl(d) => {
                for r in d.children.iter().flatten().flat_map(|k| k.refs()) {
                    if !known(r) {
                        out.push((Code::E101, r.span, reach(&crate::syntax::ref_text(r))));
                    }
                }
            }
            _ => {}
        }
    }
    out
}

impl<'a> Walk<'a> {
    /// **A statement written with a word a file defines, applied** — `true` where the statement
    /// is such a use, whatever became of it; `false` for a relation in the language's own words.
    /// What it cannot be is said once, at the word as written.
    pub(super) fn apply_word(
        &mut self,
        st: &Stmt,
        rel: &Relation,
        path: &[PathStep],
        scope: &Scope,
        depth: usize,
    ) -> bool {
        let used = match rel.form.written() {
            Some(w) if !builtin_word(&w.word.text, w.fixity) => w,
            _ => return false,
        };
        // a word reached again inside its own application is defined in terms of itself, said
        // where the outermost use is written
        let outer = self.applying_words.first().map_or(used.word.span, |(_, s)| *s);
        let Some(k) = self.prog.resolve_word(&used.word.text, used.fixity, scope.module) else {
            let m = self.no_word(used, scope.module);
            self.once(Code::E102, used.word.span, m);
            return true;
        };
        if self.applying_words.iter().any(|(j, _)| *j == k) || self.applying_words.len() >= MAX_DEPTH {
            let m = format!("`{}` is defined in terms of itself", used.word.text);
            self.once(Code::E003, outer, m);
            return true;
        }
        let Some(kinds) = self.word_kinds(k) else { return true };
        let def = &self.prog.words[k];
        let Some(values) = self.word_args(def, &kinds, used) else { return true };
        // the numbers, bound as the body's own, worked out where the statement is written
        let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
        let mut dims: BTreeMap<String, Span> = BTreeMap::new();
        for (name, (text, span)) in &values {
            if kinds.get(name) != Some(&Param::Number) {
                continue;
            }
            match value_aff(text, &scope.vals, self.units) {
                Ok(v) => {
                    vals.insert(name.clone(), v);
                    dims.insert(name.clone(), *span);
                }
                Err(e) => {
                    self.once(Code::E103, *span, format!("`{text}`: {e}"));
                    return true;
                }
            }
        }
        // the selector words, put where the body's words stand
        let words: BTreeMap<&str, &str> = values
            .iter()
            .filter(|(n, _)| kinds.get(*n) == Some(&Param::Word))
            .map(|(n, (t, _))| (n.as_str(), t.as_str()))
            .collect();
        let mut body = def.body.clone();
        for st in body.iter_mut() {
            let StmtKind::Relation(r) = &mut st.kind else { continue };
            let Some(w) = r.form.written_mut() else { continue };
            for a in w.args.iter_mut() {
                if let OpArg::Named(_, Arg::Word(v)) = a {
                    if let Some(given) = words.get(v.as_str()) {
                        *v = given.to_string();
                    }
                }
            }
            for r in refs_mut(w) {
                if let Some(given) = words.get(r.root.text.as_str()).filter(|_| r.path.is_empty()) {
                    r.root.text = given.to_string();
                }
            }
        }
        let at = used.word.span;
        let closure = Scope { closed: true, module: def.module, vals, ..Scope::default() };
        let u = Use { st, rel, path, scope, at };
        let app = self.begin(&u, &closure);
        for (operand, given) in def.operands.iter().zip(&used.ops) {
            self.bind_to_use(&app, &operand.text, given.clone(), &u);
        }
        let walks = [self.inherited_twin(&app, &u).into()];
        let worded = Worded {
            word: used.word.text.clone(),
            ops: used.ops.clone(),
            args: crate::syntax::written_parts(&used.args).0.join(", "),
            sets: Vec::new(),
            span: at,
        };
        self.applying_words.push((k, outer));
        self.apply(&u, app, body, &dims, &walks, worded, depth);
        self.applying_words.pop();
        true
    }

    /// The parameters a use gives, by label (§4.1) — each name to the text it was given and
    /// where — or `None`, said at the use, where they are not the word's.
    fn word_args(
        &mut self,
        def: &WordDef,
        kinds: &BTreeMap<String, Param>,
        given: &Written,
    ) -> Option<BTreeMap<String, (String, Span)>> {
        let word = &def.word.text;
        let mut values: BTreeMap<String, (String, Span)> = BTreeMap::new();
        for a in &given.args {
            match a {
                OpArg::Named(n, Arg::Dim { text, span }) => {
                    if !def.params.iter().any(|p| p.text == n.text) {
                        let m = format!("`{word}` has no parameter `{}`", n.text);
                        self.once(Code::E040, n.span, m);
                        return None;
                    }
                    if values.insert(n.text.clone(), (text.clone(), *span)).is_some() {
                        self.once(Code::E040, n.span, format!("`{}` is given twice", n.text));
                        return None;
                    }
                }
                OpArg::Dim(text, span) => {
                    let p = def.params.first().map_or("p", |p| p.text.as_str());
                    let m = format!(
                        "a relation word's parameters are given by label: `{word}({p}: {text})`"
                    );
                    self.once(Code::E004, *span, m);
                    return None;
                }
                _ => {
                    let m = format!("`{word}` takes its parameters by label");
                    self.once(Code::E040, given.word.span, m);
                    return None;
                }
            }
        }
        if let Some(p) = def.params.iter().find(|p| !values.contains_key(&p.text)) {
            let m = format!("`{word}` needs `{}`: `{word}({}: …)`", p.text, p.text);
            self.once(Code::E040, given.word.span, m);
            return None;
        }
        for (name, (text, span)) in &values {
            if kinds.get(name) == Some(&Param::Word) && !crate::syntax::is_name(text)
                && crate::constraints::Toward::of(text).is_none()
            {
                let m = format!("`{name}` is a word in `{word}`, not `{text}`");
                self.once(Code::E040, *span, m);
                return None;
            }
        }
        Some(values)
    }

    /// How each parameter of word `k` is read in its body, worked out once a walk; `None` for a
    /// definition `faults` refuses — said where it is written (`program::words`), not again at
    /// every statement that writes it.
    fn word_kinds(&mut self, k: usize) -> Option<Rc<BTreeMap<String, Param>>> {
        let def = &self.prog.words[k];
        self.word_kinds
            .entry(k)
            .or_insert_with(|| match faults(def).is_empty() {
                true => param_kinds(def).ok().map(Rc::new),
                false => None,
            })
            .clone()
    }

    /// Why a word that is no constraint of the language's names nothing here — with the import
    /// that would make it one where a module defines it: one the file uses first, then any linked
    /// at all (the standard library is linked wherever any file says `use std`).
    fn no_word(&self, w: &Written, from: Option<usize>) -> String {
        let word = &w.word.text;
        let used = self.prog.uses_of(from);
        let defining: Vec<&str> = self.prog.words.iter()
            .filter(|d| d.word.text == *word && d.fixity == w.fixity)
            .filter_map(|d| d.module.map(|k| self.prog.modules[k].name.as_str()))
            .collect();
        if let Some(path) = defining.iter().find(|p| used.contains(p)).or(defining.first()) {
            return format!("`{path}` defines `{word}`: import it, `use {path} ({word})`");
        }
        let other = match w.fixity {
            Fixity::Infix => Fixity::Prefix,
            _ => Fixity::Infix,
        };
        if builtin_word(word, other) {
            return match other {
                Fixity::Prefix => format!("`{word}` stands before its one operand: `{word} x`"),
                _ => format!("`{word}` stands between two operands: `a {word} b`"),
            };
        }
        format!("no relation word `{word}`")
    }
}
