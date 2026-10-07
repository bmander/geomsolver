//! **Relation words** (§9.9): a statement written with a word a file defines is that word's body
//! with the operands and the parameters put in, expanded where the statement stands — the way a
//! component's body is, and closed the same way: the body reads only what the word is given.
//!
//! The expansion keeps the statement's identity (its id, span, placement and classes) and puts
//! every span of the body at the word as written, so a fault inside it is reported where the
//! word is used.  The word itself is kept on the relation (`Relation::word`), so the constraint
//! the body states is described as the statement wrote it.

use super::*;
use std::rc::Rc;
use crate::constraints::{builtin_word, Fixity};
use crate::syntax::{Arg, OpArg, Relation, RelationForm, WordDef, Worded, Written};

/// How a word's parameter is read in its body: a number put into the body's texts, or a selector
/// word put where a word stands (`side: s`, `level(s)`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Param {
    Number,
    Word,
}

/// Every reference a written relation holds: its operands, the entities in its parentheses and a
/// direction named by reference.
fn refs_mut(w: &mut Written) -> Vec<&mut Ref> {
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
fn texts_mut(w: &mut Written) -> Vec<(&mut String, &mut Span)> {
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

/// How each parameter of a word is read in its body — or, for one read both ways, the fault.
fn param_kinds(def: &WordDef) -> Result<BTreeMap<String, Param>, (Span, String)> {
    let mut out: BTreeMap<String, Param> = BTreeMap::new();
    let Some(w) = def.body.form.written() else { return Ok(out) };
    let mut w = w.clone();
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
    for p in &def.params {
        out.entry(p.text.clone()).or_insert(Param::Number);
    }
    Ok(out)
}

/// **What is wrong with a word's definition, as written** (§9.9): a word the language keeps for
/// itself, an operand or a parameter named twice, and a body reading anything it was not given.
/// Asked of every definition by `program::words`, used or not; the expansion asks it too, and
/// expands no definition it faults, so a fault is said once, at the definition.
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
    let Some(w) = def.body.form.written() else { return out };
    let mut w = w.clone();
    let reach = |name: &str| format!(
        "`{name}` is not an operand or a parameter of `{word}`: a relation word reads only what \
         it is given (§5)"
    );
    for r in refs_mut(&mut w) {
        let root = r.root.text.as_str();
        let operand = def.operands.iter().any(|o| o.text == root);
        let param = r.path.is_empty() && kinds.get(root) == Some(&Param::Word);
        if !operand && !param && r.direction_word().is_none() {
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
    out
}

/// A reference written in a body, with an operand put in: the statement's own reference, the
/// body's path after it (`a.p1` of `a` given `l` is `l.p1`).
fn put_operand(r: &Ref, given: &Ref) -> Ref {
    let mut out = given.clone();
    out.path.extend(r.path.iter().cloned());
    out
}

impl<'a> Walk<'a> {
    /// **The relation a statement states**: itself, or — written with a word a file defines —
    /// that word's body, expanded.  `None` where it cannot be, said once at the word as written.
    pub(super) fn expand_word(&mut self, rel: &Relation, scope: &Scope) -> Option<Relation> {
        let used = match rel.form.written() {
            Some(w) if !builtin_word(&w.word.text, w.fixity) => w,
            _ => return Some(rel.clone()),
        };
        let prog = self.prog;
        let mut current: Option<Written> = None;
        let mut from = scope.module;
        let mut through: Vec<usize> = Vec::new();
        loop {
            let w = current.as_ref().unwrap_or(used);
            if builtin_word(&w.word.text, w.fixity) {
                break;
            }
            let Some(k) = prog.resolve_word(&w.word.text, w.fixity, from) else {
                let m = self.no_word(w, from);
                self.once(Code::E102, used.word.span, m);
                return None;
            };
            if through.contains(&k) || through.len() >= MAX_DEPTH {
                let m = format!("`{}` is defined in terms of itself", w.word.text);
                self.once(Code::E003, used.word.span, m);
                return None;
            }
            through.push(k);
            let def = &prog.words[k];
            let kinds = self.word_kinds(k)?;
            current = Some(self.instantiate(def, &kinds, w, used)?);
            from = def.module;
        }
        let mut out = rel.clone();
        out.form = RelationForm::Written(current?);
        out.word = Some(Worded {
            word: used.word.text.clone(),
            ops: used.ops.clone(),
            args: crate::syntax::written_parts(&used.args).0.join(", "),
            set: None,
            span: used.word.span,
        });
        Some(out)
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

    /// One step of the expansion: `def`'s body with `given`'s operands and parameters put in,
    /// every span of it at the word `used` was written with.
    fn instantiate(
        &mut self,
        def: &WordDef,
        kinds: &BTreeMap<String, Param>,
        given: &Written,
        used: &Written,
    ) -> Option<Written> {
        let word = &def.word.text;
        // the parameters, by label (§4.1): a value given by position is the call's E004
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
                    self.once(Code::E040, used.word.span, format!("`{word}` takes its parameters by label"));
                    return None;
                }
            }
        }
        if let Some(p) = def.params.iter().find(|p| !values.contains_key(&p.text)) {
            let m = format!("`{word}` needs `{}`: `{word}({}: …)`", p.text, p.text);
            self.once(Code::E040, used.word.span, m);
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
        let Some(body) = def.body.form.written() else { return None };
        let mut out = body.clone();
        let at = used.word.span;
        let nowhere = Span::new(at.lo as usize, at.lo as usize);
        // the operands, and a word parameter wherever the body names it as a reference
        let operands: BTreeMap<&str, &Ref> =
            def.operands.iter().map(|o| o.text.as_str()).zip(given.ops.iter()).collect();
        for r in refs_mut(&mut out) {
            if let Some(g) = operands.get(r.root.text.as_str()) {
                *r = put_operand(r, g);
            } else if let Some((v, _)) = values.get(&r.root.text).filter(|_| r.path.is_empty()) {
                *r = Ref { root: Name { text: v.clone(), span: at }, path: Vec::new(), span: at };
            } else {
                r.root.span = at;
                r.span = at;
            }
        }
        // a word parameter where a selector's word stands
        for a in out.args.iter_mut() {
            if let OpArg::Named(_, Arg::Word(v)) = a {
                if let Some((given, _)) = values.get(v.as_str()) {
                    *v = given.clone();
                }
            }
        }
        // the numbers: each name of a parameter written in as the text it was given, once over
        // the whole text so a value is never read again for a name inside it
        for (text, span) in texts_mut(&mut out) {
            let exact = values.get(text.trim()).filter(|_| kinds.get(text.trim()) == Some(&Param::Number));
            *span = exact.map_or(nowhere, |(_, s)| *s);
            *text = substitute_with(text, |n| match kinds.get(n) {
                Some(Param::Number) => values.get(n).map(|(v, _)| match exact {
                    Some(_) => v.clone(),
                    None => format!("({v})"),
                }),
                _ => None,
            });
        }
        // and every other place in the body is the word's own place
        out.word.span = at;
        out.span = used.span;
        for a in out.args.iter_mut() {
            match a {
                OpArg::Named(n, _) => n.span = at,
                OpArg::Slot { key, .. } => key.span = at,
                OpArg::Vector { key, span, .. } => {
                    *span = at;
                    if let Some(k) = key {
                        k.span = at;
                    }
                }
                OpArg::Ent(_) | OpArg::Dim(..) => {}
            }
        }
        Some(out)
    }
}
