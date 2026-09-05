//! Constraint arguments and dimension spellings.

use super::P;
use crate::constraints::{call_word, is_operator, Fixity};
use crate::style::Classes;
use crate::syntax::lexer::Tok;
use crate::syntax::{seed_arg, Arg, Name, OpArg, Ref, Relation, Span, Written};

impl<'a> P<'a> {
    /// Parse an operator relation (§9.1), preserving its written operands and arguments
    /// for later kind resolution.
    pub(super) fn relation(&mut self) -> Option<Relation> {
        let lo = self.here().lo as usize;
        let (word, fixity, ops, args) = match self.peek().cloned() {
            // `ccw(a, b, c)`: every operand in the parentheses and none after them
            Some(Tok::Ident(w)) if call_word(&w) => {
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text)?;
                (word, Fixity::Call, Vec::new(), args)
            }
            Some(Tok::Ident(w)) if is_operator(&w) => {
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text)?;
                let r = self.refr()?;
                (word, Fixity::Prefix, vec![r], args)
            }
            _ => {
                let left = self.refr()?;
                let Some(Tok::Ident(w)) = self.peek().cloned() else {
                    self.fail("a statement relates two things with a word between them");
                    return None;
                };
                if !is_operator(&w) {
                    self.fail(&format!("`{w}` is not a constraint"));
                    return None;
                }
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text)?;
                let right = self.refr()?;
                (word, Fixity::Infix, vec![left, right], args)
            }
        };
        self.relation_tail(word, fixity, ops, args, lo)
    }

    /// Everything a relation statement may carry after its operands.
    fn relation_tail(
        &mut self,
        word: Name,
        fixity: Fixity,
        ops: Vec<Ref>,
        args: Vec<OpArg>,
        lo: usize,
    ) -> Option<Relation> {
        let mut args = args;
        let mut place = None;
        let mut place_span = Span::default();
        let mut class = Classes::default();
        let mut class_span = Span::default();
        loop {
            if self.eat_hint_clause().is_some() {
                // a seed for a slot the constraint owns — the same clause as everywhere else,
                // and read by the same body, so one hint is parsed in one place
                for h in self.hint_body("t: 0.4")? {
                    args.push(h.into());
                }
            } else if class.is_empty() && self.peek_word("class") {
                let (c, sp) = self.class_clause(self.here().lo as usize);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
                class_span = sp;
            } else if place.is_none() && self.peek_word("at") {
                let at = self.here().lo as usize;
                self.i += 1;
                if !self.want_p('(') {
                    return None;
                }
                let t = self.number()?;
                if !self.want_p(',') {
                    return None;
                }
                let r = self.number()?;
                if !self.want_p(')') {
                    return None;
                }
                place = Some((t, r));
                place_span = Span::new(at, self.prev_hi());
            } else {
                break;
            }
        }
        self.end_of_stmt();
        // where a placement would go when none was written — an empty span at the insertion
        // point, `Decl::hint_span`'s device, spliced by `reconcile` when a callout is dragged
        if place.is_none() {
            let at = self.prev_hi();
            place_span = Span::new(at, at);
        }
        Some(Relation {
            place,
            place_span,
            form: crate::syntax::RelationForm::Written(Written {
                word,
                fixity,
                ops,
                args,
                span: Span::new(lo, self.prev_hi()),
            }),
            claim: false,
            class,
            class_span,
        })
    }

    /// Read optional operator arguments. Slot pins stay in parentheses; seeds come
    /// from the separate hint clause.
    pub(super) fn op_args(&mut self, word: &str) -> Option<Vec<OpArg>> {
        if !self.eat_p('(') {
            return Some(Vec::new());
        }
        let takes_entity = word == "symmetry" || call_word(word);
        let mut out = Vec::new();
        while !self.eat_p(')') {
            match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t.clone())) {
                (Some(Tok::Ident(n)), Some(Tok::P(':'))) => {
                    let name = Name { text: n, span: self.here() };
                    self.i += 2;
                    let v = self.sel_value()?;
                    out.push(OpArg::Named(name, v));
                }
                // `t == 0.4` — the same slot the `hint(…)` clause seeds, *pinned*.  The whole
                // of the value is kept, expression and all: written inside a component a pin
                // reads the parameters in scope (`t == t0`), which `flatten` settles later, and
                // taking only `value` here would pin every one of them at 0.
                (Some(Tok::Ident(key)), Some(Tok::EqEq)) => {
                    let at = self.here();
                    self.i += 2;
                    let (value, text, span) = self.value_text()?;
                    let arg = seed_arg(value, text, span, true);
                    out.push(OpArg::Slot { key: Name { text: key, span: at }, arg });
                }
                _ if takes_entity => out.push(OpArg::Ent(self.refr()?)),
                _ => {
                    let from = self.here().lo as usize;
                    let mut depth = 0i32;
                    while !self.done() {
                        match self.peek() {
                            Some(Tok::P('(')) | Some(Tok::P('[')) => depth += 1,
                            Some(Tok::P(')')) | Some(Tok::P(']')) if depth == 0 => break,
                            Some(Tok::P(')')) | Some(Tok::P(']')) => depth -= 1,
                            Some(Tok::P(',')) if depth == 0 => break,
                            Some(Tok::Nl) => break,
                            _ => {}
                        }
                        self.i += 1;
                    }
                    let text = self.text_from(from).trim().to_string();
                    if text.is_empty() {
                        self.fail("expected the number this states");
                        return None;
                    }
                    let hi = self.prev_hi();
                    out.push(OpArg::Dim(text, Span::new(from, hi)));
                }
            }
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        Some(out)
    }

    /// A selector's value: `side: -1`, `at: start`, `external: true`, `along: x`.
    fn sel_value(&mut self) -> Option<Arg> {
        match self.peek().cloned() {
            Some(Tok::Num(_)) | Some(Tok::P('-')) | Some(Tok::P('+')) => {
                Some(Arg::Num(self.number()?))
            }
            Some(Tok::Ident(w)) if w == "true" || w == "false" => {
                self.i += 1;
                Some(Arg::Bool(w == "true"))
            }
            Some(Tok::Ident(w)) => {
                self.i += 1;
                Some(Arg::Word(w))
            }
            _ => {
                self.fail("expected a value");
                None
            }
        }
    }

    /// Read the dimension after `==` through the logical line end, preserving its text
    /// and span for expression evaluation and edits.
    pub(super) fn raw_dimension(
        &mut self,
        from: usize,
    ) -> (String, Span, Option<((f64, f64), Span)>, usize) {
        let bytes = self.src.as_bytes();
        let mut end = from;
        while end < bytes.len() && bytes[end] != b'\n' && bytes[end] != b';' {
            if bytes[end] == b'/' && matches!(bytes.get(end + 1), Some(b'/') | Some(b'*')) {
                break;
            }
            end += 1;
        }
        let mut text = &self.src[from..end];
        // a trailing `hint(…)` ends the dimension and starts the statement's trailing clauses;
        // stopping *here* hands those tokens back to the loop that reads them, rather than
        // teaching this function a second grammar.  (No kind carries both a dimension and a
        // `Param` slot today, so this is a guard rather than a path anything walks.)
        if let Some(h) = text.find(" hint(") {
            end = from + h;
            text = &text[..h];
        }
        let mut place = None;
        if let Some(at) = text.rfind(" at (") {
            let tail = &text[at + 5..];
            if let Some(close) = tail.find(')') {
                if tail[close + 1..].trim().is_empty() {
                    let nums: Vec<f64> = tail[..close]
                        .split(',')
                        .filter_map(|s| s.trim().parse::<f64>().ok())
                        .collect();
                    if nums.len() == 2 {
                        // the span of `at (...)` itself, leading space excluded: a replacement
                        // keeps the space, and a removal takes it with `back_over_spaces`
                        let lo = from + at + 1;
                        let hi = from + at + 5 + close + 1;
                        place = Some(((nums[0], nums[1]), Span::new(lo, hi)));
                        text = &text[..at];
                    }
                }
            }
        }
        // the span of the *trimmed* text, not of the slice it was cut from: an edit splices
        // this, and a span that included the space after `==` would eat it and write `==140`
        let lead = text.len() - text.trim_start().len();
        let trimmed = text.trim();
        let span = Span::new(from + lead, from + lead + trimmed.len());
        (trimmed.to_string(), span, place, end)
    }
}
