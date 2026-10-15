//! **Relation words** (§9.9): a statement written with a word a file defines is that word's body
//! **applied** (#103, `flatten::apply`): its operands are the word's bound variables, each an
//! alias of what the statement wrote; its parameters, given by label, are numbers bound in the
//! body's scope, selector words put where the body's words stand, or entities, each an alias of
//! the reference given as an operand is (`n square(up: std.z) g`, #250); and the body is closed
//! over them as a component's is.  A body is one relation, or several and the declarations they
//! need.
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

/// How a word's parameter is read in its body: a number bound in its scope, a selector word put
/// where a word stands (`side: s`), or an entity (#250).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Param {
    Number,
    Word,
    /// Read where an entity stands — an operand (`P.v parallel up`), an entity in the
    /// parentheses (`level(w)`, `along: w`), a declaration's child — and bound at the use as an
    /// alias of the reference given (`up: std.z`), as an operand is; given a direction word
    /// (`w: up`), that word is put where it stands.
    Entity,
    /// Read only as the whole of another defined word's argument (`a twice(e) b := a square(up:
    /// e) b`): what it is is the inner word's to say, so the use decides by what it is given —
    /// a number where the text is one, else a direction word or a reference.
    Passed,
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
            OpArg::Bound { hi, grow, .. } => {
                out.extend(hi.iter_mut().chain(grow.iter_mut()).map(|(t, s)| (t, s)))
            }
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
        let now = match (out.get(name).copied(), how) {
            (None, h) | (Some(Param::Passed), h) => h,
            (Some(was), Param::Passed) => was,
            (Some(was), h) if was == h => h,
            (Some(Param::Number), _) | (Some(_), Param::Number) => {
                return Err((span, format!(
                    "`{name}` is read as a number and as a word or an entity in `{}`; a \
                     parameter is one or the other",
                    def.word.text
                )))
            }
            // a selector and where an entity stands: a word, put in both places
            _ => Param::Word,
        };
        out.insert(name.to_string(), now);
        Ok(())
    };
    for w in relations(def) {
        let mut w = w.clone();
        for a in &w.args {
            if let OpArg::Named(_, Arg::Word(v)) = a {
                said(v, Param::Word, def.word.span)?
            }
        }
        for r in refs_mut(&mut w) {
            said(&r.root.text, Param::Entity, r.span)?
        }
        // a defined word's argument that is only a parameter's name is passed on to it
        let defined = !builtin_word(&w.word.text, w.fixity);
        for a in &w.args {
            if let OpArg::Named(_, Arg::Dim { text, span }) = a {
                if defined && crate::syntax::is_name(text.trim()) {
                    said(text.trim(), Param::Passed, *span)?
                }
            }
        }
        for (text, span) in texts_mut(&mut w) {
            if defined && crate::syntax::is_name(text.trim()) {
                continue;
            }
            for name in reads(text) {
                said(&name, Param::Number, *span)?;
            }
        }
    }
    // what a declaration is made of, and the curve an energy is over
    for st in &def.body {
        match &st.kind {
            StmtKind::Decl(d) => {
                for r in d.children.iter().flatten().flat_map(|k| k.refs()) {
                    said(&r.root.text, Param::Entity, r.span)?
                }
            }
            StmtKind::Minimize(m) => said(&m.curve.root.text, Param::Entity, m.curve.span)?,
            _ => {}
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
            || match kinds.get(root) {
                Some(Param::Entity | Param::Passed) => true,
                Some(Param::Word) => r.path.is_empty(),
                _ => false,
            }
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
                // an operand or a local passed whole to another word, as an entity (#250)
                let defined = !builtin_word(&w.word.text, w.fixity);
                for (text, span) in texts_mut(&mut w) {
                    let whole = text.trim();
                    if defined && crate::syntax::is_name(whole)
                        && (def.operands.iter().any(|o| o.text == whole) || locals.contains(whole))
                    {
                        continue;
                    }
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
            // an energy is over an operand, and its integrand reads the point it runs along, its
            // tangent and the word's numbers (#121)
            StmtKind::Minimize(m) => {
                if !known(&m.curve) {
                    out.push((Code::E101, m.curve.span, reach(&crate::syntax::ref_text(&m.curve))));
                }
                for t in &m.terms {
                    let bound = |name: &str| {
                        let root = name.split('.').next().unwrap_or(name);
                        root == t.point.text || t.tangent.as_ref().is_some_and(|n| n.text == root)
                    };
                    for name in reads(&t.body) {
                        if !bound(&name) && !def.params.iter().any(|p| p.text == name) {
                            out.push((Code::E101, t.body_span, reach(&name)));
                        }
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
        let outer = self.words.applying.first().map_or(used.word.span, |(_, s)| *s);
        let Some(k) = self.prog.resolve_word(&used.word.text, used.fixity, scope.module) else {
            let m = self.no_word(used, scope.module);
            self.once(Code::E102, used.word.span, m);
            return true;
        };
        let applying = &self.words.applying;
        if applying.iter().any(|(j, _)| *j == k) || applying.len() >= MAX_DEPTH {
            let m = format!("`{}` is defined in terms of itself", used.word.text);
            self.once(Code::E003, outer, m);
            return true;
        }
        let Some(kinds) = self.word_kinds(k) else { return true };
        let def = &self.prog.words[k];
        let Some(values) = self.word_args(def, &kinds, used) else { return true };
        // the numbers, bound as the body's own, worked out where the statement is written; the
        // entities, each the reference given or a direction word put where it stands (#250)
        let mut vals: BTreeMap<String, Aff> = BTreeMap::new();
        let mut dims: BTreeMap<String, Span> = BTreeMap::new();
        let mut words: BTreeMap<&str, &str> = BTreeMap::new();
        // a number over a face family's running number that is not affine in it, or over the
        // geometry a station was given, written into the body as text: worked out at each
        // station (#243 L5, L8), as a component's is
        let mut texts: BTreeMap<&str, String> = BTreeMap::new();
        let mut entities: Vec<(&str, Ref)> = Vec::new();
        for (name, (text, span)) in &values {
            let kind = kinds.get(name).copied().unwrap_or(Param::Number);
            let number = match kind {
                Param::Number => Some(value_aff(text, &scope.vals, self.units)),
                Param::Passed => value_aff(text, &scope.vals, self.units).ok().map(Ok),
                _ => None,
            };
            match number {
                Some(Ok(v)) => {
                    vals.insert(name.clone(), v);
                    dims.insert(name.clone(), *span);
                }
                Some(Err(e)) => match self.held_text(text, &scope.vals, scope)
                    .or_else(|| self.station_text(text, *span, &scope.vals, scope)) {
                    Some(t) => {
                        texts.insert(name.as_str(), format!("({})", t.trim()));
                    }
                    None => {
                        self.once(Code::E103, *span, format!("`{text}`: {e}"));
                        return true;
                    }
                },
                None if kind == Param::Word
                    || (crate::constraints::Toward::of(text).is_some()
                        && !self.names_here(text, scope)) =>
                {
                    words.insert(name.as_str(), text.as_str());
                }
                None => match crate::syntax::reference_at(text, span.lo as usize) {
                    Some(r) => entities.push((name.as_str(), r)),
                    None => {
                        let m = format!(
                            "`{name}` is an entity in `{}`: a name or a direction word, not \
                             `{text}`",
                            def.word.text
                        );
                        self.once(Code::E040, *span, m);
                        return true;
                    }
                },
            }
        }
        let mut body = def.body.clone();
        for st in body.iter_mut() {
            let StmtKind::Relation(r) = &mut st.kind else { continue };
            let Some(w) = r.form.written_mut() else { continue };
            for a in w.args.iter_mut() {
                match a {
                    OpArg::Named(_, Arg::Word(v)) => {
                        if let Some(given) = words.get(v.as_str()) {
                            *v = given.to_string();
                        }
                    }
                    // passed on to another word as it stands
                    OpArg::Named(_, Arg::Dim { text, .. }) => {
                        if let Some(given) = words.get(text.trim()) {
                            *text = given.to_string();
                        } else if !texts.is_empty() {
                            *text = substitute_with(text, |w| texts.get(w).cloned());
                        }
                    }
                    OpArg::Dim(text, _) | OpArg::Slot { arg: Arg::Dim { text, .. }, .. }
                        if !texts.is_empty() =>
                    {
                        *text = substitute_with(text, |w| texts.get(w).cloned());
                    }
                    _ => {}
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
        for (name, given) in entities {
            self.bind_to_use(&app, name, given, &u);
        }
        let walks = [self.inherited_twin(&app, &u).into()];
        let worded = Worded {
            word: used.word.text.clone(),
            ops: used.ops.clone(),
            args: crate::syntax::written_parts(&used.args).0.join(", "),
            sets: Vec::new(),
            through: Vec::new(),
            span: at,
        };
        self.words.applying.push((k, outer));
        self.apply(&u, app, body, &dims, &walks, worded, depth);
        self.words.applying.pop();
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

    /// Whether `name` is something declared or bound where `scope` reads it — a formal called
    /// `up` given on (`square(up: up)`) is that formal, not the direction word.
    fn names_here(&self, name: &str, scope: &Scope) -> bool {
        scope.prefixes.iter().any(|p| {
            let key = format!("{p}{name}");
            self.names.contains(&key) || self.aliases.iter().any(|(a, _, _)| *a == key)
        })
    }

    /// How each parameter of word `k` is read in its body, worked out once a walk; `None` for a
    /// definition `faults` refuses — said where it is written (`program::words`), not again at
    /// every statement that writes it.
    fn word_kinds(&mut self, k: usize) -> Option<Rc<BTreeMap<String, Param>>> {
        let def = &self.prog.words[k];
        self.words.kinds
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
