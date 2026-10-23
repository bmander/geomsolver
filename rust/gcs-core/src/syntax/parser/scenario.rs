//! `scenario lift { … }` — a load case (#510 P4): its supports, loads and requirements, and how
//! it reads a solid in its plane (#510 S2), each a line of its own.  The body is no body of
//! statements: it declares nothing and relates nothing, so its lines are read here, by their first
//! word, and nowhere else.

use super::P;
use crate::expr::Cmp;
use crate::model::{Hold, LoadKind, Output, InPlane};
use crate::syntax::lexer::Tok;
use crate::syntax::{Item, ItemKind, LoadText, RequirementText, Scenario, Span};

/// What a scenario's body holds, said where a line is none of it.
const HOLDS: &str = "a scenario holds supports (`fixed X`, `pinned X`), loads (`load X (fx, fy, \
                     fz)`, `load X f along: d`, `pressure F p`, `accelerate B (ax, ay, az)`), \
                     requirements (`deflection(X) <= 0.5mm`, `claim mode(B, 1) >= 120Hz`) and \
                     readings (`plane stress B`, `plane strain B`)";

impl P<'_> {
    /// Whether `scenario NAME` stands here.
    pub(super) fn scenario_ahead(&self) -> bool {
        self.peek_word("scenario")
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(_)))
    }

    /// `scenario NAME [: BASE] { … }`, `scenario` next.
    pub(super) fn scenario(&mut self) -> Option<Scenario> {
        let lo = self.here().lo as usize;
        self.i += 1;
        let name = self.ident()?;
        let extends = if self.eat_p(':') { Some(self.ident()?) } else { None };
        self.skip_ends();
        if !self.want_p('{') {
            return None;
        }
        self.in_body += 1;
        let items = self.scenario_items();
        self.in_body -= 1;
        Some(Scenario { name, extends, items: items?, span: Span::new(lo, self.prev_hi()) })
    }

    /// The body's lines through its `}`: one bad line costs that line.
    fn scenario_items(&mut self) -> Option<Vec<Item>> {
        let mut items = Vec::new();
        loop {
            self.skip_ends();
            if self.eat_p('}') {
                return Some(items);
            }
            if self.done() {
                if !self.exhausted {
                    self.fail("a scenario with no closing `}`");
                }
                return None;
            }
            let lo = self.here().lo as usize;
            match self.scenario_item() {
                Some(kind) => {
                    items.push(Item { kind, span: Span::new(lo, self.prev_hi()) });
                    self.end_of_stmt();
                }
                // past the line, or up to the `}` that ends the body on it
                None => {
                    while !self.done() && !matches!(self.peek(), Some(Tok::Nl | Tok::P('}'))) {
                        self.i += 1;
                    }
                }
            }
        }
    }

    fn scenario_item(&mut self) -> Option<ItemKind> {
        // a load patch, or anything else a scenario would draw, is drawn in the body: geometry
        // standing only under one load case would make the solved drawing depend on which
        if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Define)) {
            self.fail(
                "a scenario adds no geometry: draw what a load is applied to in the body, and \
                 name it here",
            );
            return None;
        }
        let Some(Tok::Ident(w)) = self.peek().cloned() else {
            self.fail(HOLDS);
            return None;
        };
        let call = matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('(')));
        if w == "claim" {
            self.i += 1;
            return self.requirement(true).map(ItemKind::Requirement);
        }
        if let Some(hold) = Hold::parse(&w) {
            self.i += 1;
            let at = self.refr()?;
            let along = self.along()?;
            return Some(ItemKind::Support { hold, at, along });
        }
        if let Some(kind) = LoadKind::parse(&w) {
            self.i += 1;
            let at = self.refr()?;
            let value = self.load_value(kind)?;
            return Some(ItemKind::Load { kind, at, value });
        }
        if call && (Output::parse(&w).is_some() || crate::model::Quantity::parse(&w).is_some()) {
            return self.requirement(false).map(ItemKind::Requirement);
        }
        // `plane stress B`, `plane strain B` (#510 S2)
        if w == "plane" {
            self.i += 1;
            let Some(plane) = self.word_at(self.i).and_then(InPlane::parse) else {
                self.fail("a solid is read in its plane in `plane stress` or `plane strain`: \
                           `plane stress B`");
                return None;
            };
            self.i += 1;
            let of = self.refr()?;
            return Some(ItemKind::Reading { plane, of });
        }
        self.fail(HOLDS);
        None
    }

    /// `along: d` where one is written: `Some(None)` where none is.
    fn along(&mut self) -> Option<Option<crate::syntax::Ref>> {
        let at = self.here();
        match self.slot_label() {
            None => Some(None),
            Some(l) if l == "along" => Some(Some(self.refr()?)),
            Some(l) => {
                self.fail_at(at, &format!("a scenario's line takes `along:`, not `{l}:`"));
                None
            }
        }
    }

    /// A load's number: a vector, a magnitude `along:` a direction, or a pressure's one number.
    fn load_value(&mut self, kind: LoadKind) -> Option<LoadText> {
        let at = self.here();
        if kind.directed() && self.tuple_ahead() {
            let parts: Vec<_> = self.vector("")?.into_iter().map(|(_, (_, t, s))| (t, s)).collect();
            if parts.len() != 3 {
                self.fail_at(
                    Span::new(at.lo as usize, self.prev_hi()),
                    &format!(
                        "a {}'s vector is `(x, y, z)`, in the world's axes; in a plane, write its \
                         size along a direction: `{} X f along: d`",
                        kind.word(),
                        kind.word()
                    ),
                );
                return None;
            }
            return Some(LoadText::Vector(parts));
        }
        let stop = |t: &Tok| matches!(t, Tok::P('}')) || matches!(t, Tok::Ident(w) if w == "along");
        let magnitude = self.raw_until(stop, &format!("a {} states its number", kind.word()))?;
        match (kind.directed(), self.along()?) {
            (true, Some(along)) => Some(LoadText::Along { magnitude, along }),
            (false, None) => Some(LoadText::Scalar(magnitude)),
            (true, None) => {
                self.fail_at(
                    magnitude.1,
                    &format!(
                        "a {} has a direction: `(x, y, z)` in the world's axes, or `f along: d`",
                        kind.word()
                    ),
                );
                None
            }
            (false, Some(_)) => {
                self.fail_at(at, "a pressure pushes into the face it is on, and takes no `along:`");
                None
            }
        }
    }

    /// `deflection(X) <= 0.5mm`, the output's word next (`claim` read already where it is one).
    fn requirement(&mut self, claim: bool) -> Option<RequirementText> {
        // what a body is made of answers no load: the drawing states it, never a scenario
        if let Some(q) = self.word_at(self.i).and_then(crate::model::Quantity::parse) {
            let w = q.word();
            self.fail(&format!("a {w} answers to no load, so it is no scenario's: the drawing \
                                states it, `{w}(…) arm`, or claims it, `claim {w}(<= …) arm`"));
            return None;
        }
        let Some(output) = self.word_at(self.i).and_then(Output::parse) else {
            let words = crate::syntax::one_of(&Output::ALL.map(Output::word));
            self.fail(&format!("a requirement is written over an output: {words}"));
            return None;
        };
        self.i += 1;
        if !self.want_p('(') {
            return None;
        }
        let of = self.refr()?;
        let (mut along, mut count) = (None, None);
        while self.eat_p(',') {
            let at = self.here();
            match self.slot_label() {
                Some(l) if l == "along" && output.directed() && along.is_none() => {
                    along = Some(self.refr()?)
                }
                None if output.counted() && count.is_none() => {
                    count = Some(self.raw_until(|t| *t == Tok::P(','), "expected a count")?)
                }
                _ => {
                    self.fail_at(at, &format!("`{}` takes {}", output.word(), takes(output)));
                    return None;
                }
            }
        }
        if !self.want_p(')') {
            return None;
        }
        if output.counted() && count.is_none() {
            self.fail_at(of.span, &format!("`{}` takes {}", output.word(), takes(output)));
            return None;
        }
        let compare = match self.peek() {
            Some(Tok::EqEq) => {
                self.i += 1;
                Cmp::Eq
            }
            Some(Tok::P('<' | '>')) => match self.closed_cmp()? {
                crate::syntax::Cmp::Le => Cmp::Le,
                _ => Cmp::Ge,
            },
            _ => {
                self.fail("a requirement compares its output with a number: `<=`, `>=` or `==`");
                return None;
            }
        };
        let value = self.raw_until(|t| *t == Tok::P('}'), "expected the number it is compared with")?;
        Some(RequirementText { claim, output, of, along, count, compare, value })
    }
}

/// What an output's parentheses hold, said where they hold otherwise.
fn takes(o: Output) -> &'static str {
    if o.counted() {
        "what it is of and which, counted from 1: `mode(B, 1)`"
    } else if o.directed() {
        "what it is of, and may take `along:` a direction: `deflection(X, along: d)`"
    } else {
        "what it is of alone"
    }
}
