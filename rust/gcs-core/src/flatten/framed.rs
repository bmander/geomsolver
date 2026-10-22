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
use crate::syntax::{Chained, Component, Decl, DeclName, Name, Ref, Span, Stmt, StmtKind};

impl Walk<'_> {
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
                let datum = |k: usize| self.names.contains(&format!("std.{}", parts[k + 1]));
                let at = (1..parts.len().saturating_sub(1))
                    .find(|&k| parts[k] == "std" && datum(k));
                if let Some(k) = at {
                    read.push(format!("{}{}", scope.prefix(), parts[..k].join(".")));
                }
            });
        }
        self.standard.outside.extend(read);
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
        let refs: Vec<&Ref> = match &st.kind {
            StmtKind::Decl(d) => d.membership.path().collect(),
            StmtKind::Instance(i) => i.membership.path().collect(),
            StmtKind::Relation(r) => r.within.iter().collect(),
            StmtKind::Block(b) => {
                for inner in b.stmts() {
                    self.paths(inner, f);
                }
                Vec::new()
            }
            _ => Vec::new(),
        };
        for r in refs {
            f(&read_segments(r));
        }
    }

    /// **Whether a definition's body reads a standard datum** (#548): a `std.D` its file reads by
    /// a `use std` of its own, written in the body or in the body of a definition it calls, each
    /// of which stands in the frame its instance does.  `seen` ends a recursion.
    fn reads_datums(&self, comp: &Component, seen: &mut Vec<*const Component>) -> bool {
        if seen.contains(&(comp as *const Component)) {
            return false;
        }
        seen.push(comp);
        let file = self.standard.files.contains(&comp.module);
        let mut direct = false;
        for st in &comp.body {
            self.paths(st, &mut |parts| {
                direct |= file && parts.len() > 1 && parts[0] == "std"
                    && self.names.contains(&format!("std.{}", parts[1]));
            });
        }
        direct || calls(&comp.body).into_iter().any(|c| {
            self.prog.resolve_component(c, comp.module)
                .is_ok_and(|k| self.reads_datums(&self.prog.components[k], seen))
        })
    }

    /// **An instance's own frame** (#548): where something outside reads instance `abs`'s
    /// datums, the frame it stands in — `{inst}#frame`, declared bare where the call is, in the
    /// `in`s around it, made with axes (`Expansion::charts`) — as the call's scope writes it.
    /// `None` where nothing reads them, and where its body reads no datum, which is refused at the
    /// reading (`Lookup::missing`): a body placed by what it is given has no frame a mate places.
    pub(super) fn own_frame(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        st: &Stmt,
        scope: &Scope,
        path: &[PathStep],
        abs: &str,
    ) -> Option<InPlane> {
        if !self.standard.outside.contains(abs) {
            return None;
        }
        if !self.reads_datums(comp, &mut Vec::new()) {
            let name = comp.name.as_ref().map_or(String::new(), |n| n.text.clone());
            self.standard.refused.insert(abs.to_string(), name);
            return None;
        }
        let at = Span::at(st.span.lo as usize);
        let name = Name { text: keys::instance_frame(&inst.name.text), span: at };
        let mut d = Decl::point(DeclName::Key(name.clone()), [0.0; 2], None);
        d.kind = crate::model::EntKind::Frame;
        d.bare = true;
        d.seed = vec![0.0; 3];
        d.seed_text = vec![None; 3];
        d.children = vec![Vec::new(); 3];
        d.membership = inst.membership.clone();
        let made = Stmt { id: st.id, kind: StmtKind::Decl(d.clone()), span: st.span,
                          chained: Chained::No };
        let prefix = scope.prefix().to_string();
        self.walk_decl(&d, &made, scope, &scope.vals, path, &prefix);
        self.charts.insert(format!("{prefix}{}", name.text));
        Some(InPlane::at(&Ref { root: name, path: Vec::new(), span: at }, scope))
    }
}

/// The definitions a body calls, by the name each call writes, blocks' included.
fn calls(body: &[Stmt]) -> Vec<&str> {
    let mut out = Vec::new();
    for st in body {
        match &st.kind {
            StmtKind::Instance(i) => out.push(i.component.text.as_str()),
            StmtKind::Block(b) => out.extend(calls(&b.body)),
            _ => {}
        }
    }
    out
}
