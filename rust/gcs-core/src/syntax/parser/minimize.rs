//! `minimize` and `maximize` (#121): a sum of integrals over curves, each with a constant
//! coefficient — `minimize integral(p.y over p in rope)`, `maximize integral((p.x * t.y - p.y *
//! t.x) / 2 over (p, t) in k)`.  The integrand stays text, read by `expr` once the flattener has
//! written the scope's numbers into it.

use super::P;
use crate::syntax::lexer::Tok;
use crate::syntax::{Integral, Minimize, Name, Span};

impl<'a> P<'a> {
    pub(super) fn minimize(&mut self, maximize: bool) -> Option<Minimize> {
        let lo = self.here().lo as usize;
        self.i += 1;
        let mut terms = Vec::new();
        let mut sign = 1.0;
        loop {
            terms.push(self.integral_term(sign)?);
            if self.eat_p('+') {
                sign = 1.0;
            } else if self.eat_p('-') {
                sign = -1.0;
            } else {
                break;
            }
        }
        self.end_of_stmt();
        Some(Minimize { maximize, terms, span: Span::new(lo, self.prev_hi()) })
    }

    /// `[c *] integral(EXPR over p in k)`, `c` a number, the sign before it `sign`.
    fn integral_term(&mut self, sign: f64) -> Option<Integral> {
        let lo = self.here().lo as usize;
        let mut coef = sign;
        if self.eat_p('-') {
            coef = -coef;
        }
        if matches!(self.peek(), Some(Tok::Num(_))) {
            coef *= self.number()?;
            if !self.want_p('*') {
                return None;
            }
        }
        if !self.peek_word("integral") {
            self.fail("a term of an energy is `integral(EXPR over p in curve)`, times a number");
            return None;
        }
        self.i += 1;
        if !self.want_p('(') {
            return None;
        }
        let (body, body_span) = self.expr_until_word("over")?;
        if !self.peek_word("over") {
            self.fail("an integral runs `over p in curve`, or `over (p, t) in curve`");
            return None;
        }
        self.i += 1;
        let (point, tangent) = if self.eat_p('(') {
            let p = self.ident()?;
            if !self.want_p(',') {
                return None;
            }
            let t = self.ident()?;
            if !self.want_p(')') {
                return None;
            }
            (p, Some(t))
        } else {
            (self.ident()?, None)
        };
        if tangent.as_ref().is_some_and(|t: &Name| t.text == point.text) {
            self.fail("the point and its tangent need names of their own");
            return None;
        }
        if !self.peek_word("in") {
            self.fail("an integral runs `over p in curve`");
            return None;
        }
        self.i += 1;
        let curve = self.refr()?;
        if !self.want_p(')') {
            return None;
        }
        Some(Integral {
            coef,
            point,
            tangent,
            curve,
            body,
            body_span,
            span: Span::new(lo, self.prev_hi()),
        })
    }
}
