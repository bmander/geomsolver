//! **An instance is a frame** (#548): an instance whose datums the scope declaring it, or one
//! around that, reads by their written path — `cyl.std.origin`, `cyl.std.y` — stands in a frame
//! of its own, `{cyl}#frame` (`keys::instance_frame`), declared beside its call as `in (f :=
//! frame) { cyl := … }` would declare it: in the `in`s around the call, so it rides the frame its
//! call stands in, and its body standing in it reads its `std` as that frame's own (#542).  From
//! outside, `cyl.std.D` names those same datums (`Lookup::instance_datum`).  What decides it is a
//! pre-pass over the text, as `instance_numbers` reads `inst.formal`: an instance whose `std`
//! nothing outside reads stands in the frame around its call, exactly as before.

use super::keys;
use super::resolve::read_segments;
use super::{InPlane, Scope, Walk};
use crate::ir::PathStep;
use crate::constraints::builtin_word;
use crate::syntax::{Arg, Chained, Component, Decl, DeclName, Name, OpArg, Ref, Span, Stmt, StmtKind,
                    Written};

impl<'a> Walk<'a> {
    /// **The pre-pass** (#548): every instance a body's statements read the datums of from
    /// outside, `X.std.D` with `X` not empty, put by absolute name — `X` under the scope's prefix
    /// — in `Datums::outside`.  Read once each time a body is walked, before its instances are,
    /// so what an enclosing scope or a later statement writes is known to every call it names.
    pub(super) fn read_outside(&mut self, body: &[Stmt], scope: &Scope) {
        if self.standard.files.is_empty() {
            return;
        }
        let mut read = Vec::new();
        for st in body {
            self.paths(st, &mut |parts| {
                if let Some(of) = self.datum_of(parts) {
                    read.push(format!("{}{}", scope.prefix(), of.join(".")));
                }
            });
            // and a relation word's use reads what its body reads of what it is given (#595)
            self.words_used(st, &mut |w| {
                for parts in self.use_reads(w, scope.module, &mut Vec::new()) {
                    read.push(format!("{}{}", scope.prefix(), parts.join(".")));
                }
            });
        }
        self.standard.outside.extend(read);
    }

    /// Every relation statement `st` writes with a word a file defines, a block's included.
    fn words_used(&self, st: &'a Stmt, f: &mut dyn FnMut(&'a Written)) {
        match &st.kind {
            StmtKind::Relation(r) => {
                let defined = r.form.written().filter(|w| !builtin_word(&w.word.text, w.fixity));
                if let Some(w) = defined {
                    f(w);
                }
            }
            StmtKind::Block(b) => b.stmts().for_each(|inner| self.words_used(inner, f)),
            _ => {}
        }
    }

    /// **The datums a word's use reads** (#595): the paths, as the use writes them, of what it
    /// gives the word — an operand or an entity parameter — whose datums its body reads from
    /// outside, `g.std.origin` of an operand `g`, directly or through a word it uses in turn.  A
    /// word is an instance's mate when its body reads the instance's frame, so the instance it is
    /// given stands in one.  `seen` ends a recursion, which the application refuses (E003).
    fn use_reads(&self, w: &Written, from: Option<usize>, seen: &mut Vec<usize>)
        -> Vec<Vec<String>> {
        let Some(k) = self.prog.resolve_word(&w.word.text, w.fixity, from) else {
            return Vec::new();
        };
        if seen.contains(&k) {
            return Vec::new();
        }
        seen.push(k);
        let def = &self.prog.words[k];
        // what the body reads past each name it holds, before the `std`
        let mut read: Vec<Vec<String>> = Vec::new();
        for st in &def.body {
            self.paths(st, &mut |parts| read.extend(self.datum_of(parts).map(<[String]>::to_vec)));
            self.words_used(st, &mut |inner| {
                read.extend(self.use_reads(inner, def.module, seen));
            });
        }
        seen.pop();
        // each put in the use's terms: an operand as the use writes it, a parameter as the
        // reference it was given
        let given = |name: &str| -> Option<Vec<String>> {
            if let Some(j) = def.operands.iter().position(|o| o.text == name) {
                return w.ops.get(j).map(read_segments);
            }
            let (text, span) = w.args.iter().find_map(|a| match a {
                OpArg::Named(n, Arg::Dim { text, span }) if n.text == name => Some((text, span)),
                // a word of one parameter takes it bare
                OpArg::Dim(text, span) if def.params.len() == 1 && def.params[0].text == name => {
                    Some((text, span))
                }
                _ => None,
            })?;
            crate::syntax::reference_at(text, span.lo as usize).map(|r| read_segments(&r))
        };
        read.into_iter().filter_map(|parts| {
            let mut out = given(&parts[0])?;
            out.extend(parts[1..].iter().cloned());
            Some(out)
        }).collect()
    }

    /// Where a path reads a standard datum: the first segment `std` with a datum's name after it.
    fn datum_at(&self, parts: &[String]) -> Option<usize> {
        (0..parts.len().saturating_sub(1))
            .find(|&k| parts[k] == "std" && self.names.contains(&format!("std.{}", parts[k + 1])))
    }

    /// What a path reads the datums of from outside: the segments before its `std`, where there
    /// are any (`cyl` of `cyl.std.y`).
    fn datum_of<'p>(&self, parts: &'p [String]) -> Option<&'p [String]> {
        self.datum_at(parts).filter(|&k| k > 0).map(|k| &parts[..k])
    }

    /// Every dotted path statement `st` writes, as its segments, to `f`: those its text reads
    /// (`syntax::name_uses`), the frames it is drawn in — an `in` block's header is no part of
    /// the statements it holds — and a block's statements'.
    fn paths(&self, st: &Stmt, f: &mut dyn FnMut(&[String])) {
        if let Some(text) = self.prog.span_text(st.span).filter(|t| t.contains("std")) {
            for u in crate::syntax::name_uses(text) {
                let parts: Vec<String> = u.parts.into_iter().map(|(p, _)| p).collect();
                f(&parts);
            }
        }
        let frames: Vec<&Ref> = match &st.kind {
            StmtKind::Decl(d) => d.membership.path().collect(),
            StmtKind::Instance(i) => i.membership.path().collect(),
            StmtKind::Relation(r) => r.within.iter().collect(),
            StmtKind::Block(b) => {
                b.stmts().for_each(|inner| self.paths(inner, f));
                return;
            }
            _ => return,
        };
        for r in frames {
            f(&read_segments(r));
        }
    }

    /// **Whether a definition's body reads a standard datum** (#548): a `std.D` its file reads by
    /// a `use std` of its own, written in the body or in the body of a definition it calls, each
    /// of which stands in the frame its instance does.  `seen` ends a recursion.
    fn reads_datums(&self, comp: &'a Component, seen: &mut Vec<&'a Component>) -> bool {
        if seen.iter().any(|c| std::ptr::eq(*c, comp)) {
            return false;
        }
        seen.push(comp);
        let mut direct = false;
        if self.standard.files.contains(&comp.module) {
            for st in &comp.body {
                self.paths(st, &mut |parts| direct |= self.datum_at(parts) == Some(0));
            }
        }
        direct || calls(&comp.body).into_iter().any(|c| {
            self.prog.resolve_component(c, comp.module)
                .is_ok_and(|k| self.reads_datums(&self.prog.components[k], seen))
        })
    }

    /// **An instance's own frame** (#548): where something outside reads instance `inst`'s
    /// datums, the frame it stands in — `{inst}#frame`, declared bare where the call is, in the
    /// `in`s around it, made with axes (`Expansion::charts`) — as the call's scope writes it.
    /// `None` where nothing reads them, and where its body reads no datum, which is refused at the
    /// reading (`Lookup::missing`): a body placed by what it is given has no frame a mate places.
    pub(super) fn own_frame(
        &mut self,
        comp: &'a Component,
        inst: &crate::syntax::Instance,
        st: &Stmt,
        scope: &Scope,
        path: &[PathStep],
    ) -> Option<InPlane> {
        let prefix = scope.prefix().to_string();
        let abs = format!("{prefix}{}", inst.name.text);
        if !self.standard.outside.contains(&abs) {
            return None;
        }
        if !self.reads_datums(comp, &mut Vec::new()) {
            let name = comp.name.as_ref().map_or(String::new(), |n| n.text.clone());
            self.standard.refused.insert(abs, name);
            return None;
        }
        let at = Span::at(st.span.lo as usize);
        let name = Name { text: keys::instance_frame(&inst.name.text), span: at };
        let d = Decl { membership: inst.membership.clone(),
                       ..Decl::frame(DeclName::Key(name.clone())) };
        let made = Stmt { id: st.id, kind: StmtKind::Decl(d.clone()), span: st.span,
                          chained: Chained::No };
        self.walk_decl(&d, &made, scope, &scope.vals, path, &prefix);
        self.charts.insert(keys::instance_frame(&abs));
        Some(InPlane::at(&Ref { root: name, path: Vec::new(), span: at }, scope))
    }
}

/// The definitions statements call, by the name each call writes, blocks' included.
fn calls<'s>(stmts: impl IntoIterator<Item = &'s Stmt>) -> Vec<&'s str> {
    let mut out = Vec::new();
    for st in stmts {
        match &st.kind {
            StmtKind::Instance(i) => out.push(i.component.text.as_str()),
            StmtKind::Block(b) => out.extend(calls(b.stmts())),
            _ => {}
        }
    }
    out
}
