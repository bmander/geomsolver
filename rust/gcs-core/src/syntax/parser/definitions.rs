//! Relation words defined in the language (§9.9): the definition at the top of a file, and the
//! shapes a statement writing one takes.

use super::P;
use crate::constraints::{is_operator, Fixity};
use crate::syntax::lexer::Tok;
use crate::syntax::words::{past_args, MODIFIERS};
use crate::syntax::{reserved_word, Arg, Chained, Name, OpArg, Span, StmtKind, WordDef};

impl<'a> P<'a> {
    /// Names separated by commas up to the `)` closing a list already opened: a word's
    /// parameters, the names a `use` imports.
    pub(super) fn ident_list(&mut self) -> Option<Vec<Name>> {
        let mut out = Vec::new();
        while !self.eat_p(')') {
            out.push(self.ident()?);
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        Some(out)
    }

    /// **`a word(params) b :=` or `word(params) l :=` at the head of a statement** — a relation
    /// word being defined (§9.9): its fixity, or `None` and nothing eaten.  Names and a word,
    /// with at most one parenthesised list after the word, then `:=`; a modifier in front
    /// (`param w := …`, `construction c := …`) is a definition of another kind.
    pub(super) fn word_definition_ahead(&self) -> Option<Fixity> {
        let first = self.word_at(self.i)?;
        if MODIFIERS.contains(&first) {
            return None;
        }
        // the word at `w`: its parameters, if any, then the last operand and `:=`
        let tail = |w: usize| {
            let j = past_args(&self.t, w);
            self.word_at(j).is_some() && self.t.get(j + 1).map(|(t, _)| t) == Some(&Tok::Define)
        };
        if self.word_at(self.i + 1).is_some() && tail(self.i + 1) {
            return Some(Fixity::Infix);
        }
        tail(self.i).then_some(Fixity::Prefix)
    }

    /// `a horizontal b := a level(up) b` — the head's names, the word and its parameters, and
    /// what it stands for after `:=`: one relation, or `{ … }` holding several and the
    /// declarations they need (#103).
    pub(super) fn word_definition(&mut self, fixity: Fixity, next_id: &mut u32) -> Option<WordDef> {
        let lo = self.here().lo as usize;
        let mut operands = Vec::new();
        if fixity == Fixity::Infix {
            operands.push(self.ident()?);
        }
        let word = self.ident()?;
        let params = if self.eat_p('(') { self.ident_list()? } else { Vec::new() };
        operands.push(self.ident()?);
        self.i += 1; // `:=`
        if matches!(self.peek(), Some(Tok::Nl) | None) {
            self.fail("a relation word says what it stands for: `a horizontal b := a level(up) b`");
            return None;
        }
        // braces hold several statements; the line, one — read as any statement is, so a set
        // written in place there is a statement of the body beside the relation
        let body = if self.peek() == Some(&Tok::P('{')) {
            let (body, joint) = self.braced_body(next_id)?;
            self.no_open_joint(joint, "a relation word");
            body
        } else {
            let mut body = Vec::new();
            self.chain_or_one(next_id, &mut body)?;
            body
        };
        for st in &body {
            let refused = match &st.kind {
                // where the callout sits and how it looks are the statement's that writes the word
                StmtKind::Relation(r) if r.place.is_some() || !r.class.is_empty() => Some(
                    "a relation word's body states the relation alone: a placement or a class \
                     is written where the word is used",
                ),
                StmtKind::Relation(_) | StmtKind::Decl(_) => None,
                // a set written in place where the body uses it
                StmtKind::Set(_) | StmtKind::Instance(_) if st.chained == Chained::Link => None,
                _ => Some(
                    "a relation word's body states relations, and declares the geometry they \
                     need",
                ),
            };
            if let Some(m) = refused {
                self.errs.push(crate::syntax::SynErr { span: st.span, message: m.to_string() });
                return None;
            }
        }
        if body.is_empty() {
            self.fail("a relation word says what it stands for: `a horizontal b := a level(up) b`");
            return None;
        }
        Some(WordDef {
            word,
            fixity,
            operands,
            params,
            body,
            span: Span::new(lo, self.prev_hi()),
            module: None,
        })
    }

    /// **A word the language does not have, standing before one operand** — `flat l`, `flat(d:
    /// 2) l` — which a file may define (§9.9).  Asked where a statement starts, ahead of the
    /// call it would otherwise be read as: a component call has nothing after its parentheses
    /// but a trailing clause, and this has its operand.
    pub(super) fn defined_prefix_ahead(&self) -> bool {
        let Some(w) = self.word_at(self.i) else { return false };
        if is_operator(w) || reserved_word(w) {
            return false;
        }
        let j = past_args(&self.t, self.i);
        if self.word_at(j).is_some_and(|w| MODIFIERS.contains(&w)) {
            return false;
        }
        let Some(end) = self.past_ref(j) else { return false };
        match self.t.get(end).map(|(t, _)| t) {
            None | Some(Tok::Nl) | Some(Tok::P('}')) => true,
            Some(Tok::Ident(w)) => w == "class" || w == "at" || w == "hint",
            _ => false,
        }
    }

    /// A defined word's parentheses (§9.9): its parameters given by label, each value kept as the
    /// text it was written as, since whether it is a number or a selector word is the
    /// definition's to say.  An unlabelled item is kept to be refused where the word is
    /// expanded (§4.1, E004).
    pub(super) fn word_args(&mut self) -> Option<Vec<OpArg>> {
        let mut out = Vec::new();
        while !self.eat_p(')') {
            let label = match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t)) {
                (Some(Tok::Ident(n)), Some(Tok::P(':'))) => {
                    let name = Name { text: n, span: self.here() };
                    self.i += 2;
                    Some(name)
                }
                _ => None,
            };
            let (_, text, span) = self.value_text()?;
            out.push(match label {
                Some(name) => OpArg::Named(name, Arg::Dim { text, span }),
                None => OpArg::Dim(text, span),
            });
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        Some(out)
    }
}
