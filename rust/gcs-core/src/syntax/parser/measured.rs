//! `distance(p, f1) + distance(p, f2) <= 10mm` — a bound on measurements (#183 F3), **parsed
//! once** (#190): each side a `Measuring` tree, its measurements over references read by `refr`
//! as every other operand is, and whatever measures nothing kept as one number's text, which
//! `expr` reads once the flattener has written the scope's numbers into it.

use super::P;
use crate::expr::{Op, MEASURES};
use crate::syntax::lexer::Tok;
use crate::syntax::{Measured, Measurement, Measuring, Span};

impl<'a> P<'a> {
    /// `LHS <= RHS` or `LHS >= RHS`, from the statement's first token.
    pub(super) fn measured(&mut self) -> Option<Measured> {
        let lo = self.here().lo as usize;
        let lhs = self.measuring(self.level_end(true))?;
        if !matches!(self.peek(), Some(Tok::P('<' | '>'))) {
            self.fail("a bound on measurements compares two sides, `<=` or `>=`");
            return None;
        }
        let cmp = self.closed_cmp()?;
        let rhs = self.measuring(self.level_end(false))?;
        self.end_of_stmt();
        Some(Measured { lhs, cmp, rhs, span: Span::new(lo, self.prev_hi()), through: Vec::new() })
    }

    /// One side, every token up to `end`.
    fn measuring(&mut self, end: usize) -> Option<Measuring> {
        if self.i >= end {
            self.fail("expected an expression");
            return None;
        }
        let m = self.m_sum(end)?;
        if self.i < end {
            self.fail("a bound on measurements is arithmetic over measurements and numbers");
            return None;
        }
        Some(m)
    }

    /// What was read from the token `from` on: itself where it measures something, else the
    /// number its text is.
    fn m_folded(&self, from: usize, m: Measuring) -> Measuring {
        if !m.measures().is_empty() {
            return m;
        }
        self.m_text(from)
    }

    /// The tokens from `from` to here, as the number they are written as.
    fn m_text(&self, from: usize) -> Measuring {
        let lo = self.t.get(from).map_or(self.src.len(), |(_, s)| s.lo as usize);
        Measuring::Number { text: self.text_from(lo).to_string(), span: Span::new(lo, self.prev_hi()) }
    }

    fn m_op(&self, end: usize, ops: &[char]) -> Option<Op> {
        if self.i >= end {
            return None;
        }
        let op = match self.peek()? {
            Tok::P('*') if self.t.get(self.i + 1).map(|(t, _)| t) == Some(&Tok::P('*')) => Op::Pow,
            Tok::P('+') => Op::Add,
            Tok::P('-') => Op::Sub,
            Tok::P('*') => Op::Mul,
            Tok::P('/') => Op::Div,
            Tok::P('^') => Op::Pow,
            _ => return None,
        };
        let sign = match op {
            Op::Add => '+',
            Op::Sub => '-',
            Op::Mul => '*',
            Op::Div => '/',
            Op::Pow => '^',
        };
        ops.contains(&sign).then_some(op)
    }

    fn m_sum(&mut self, end: usize) -> Option<Measuring> {
        let from = self.i;
        let mut a = self.m_product(end)?;
        while let Some(op) = self.m_op(end, &['+', '-']) {
            self.i += 1;
            let b = self.m_product(end)?;
            a = Measuring::Bin(op, Box::new(a), Box::new(b));
        }
        Some(self.m_folded(from, a))
    }

    fn m_product(&mut self, end: usize) -> Option<Measuring> {
        let from = self.i;
        let mut a = self.m_unary(end)?;
        while let Some(op) = self.m_op(end, &['*', '/']) {
            self.i += 1;
            let b = self.m_unary(end)?;
            a = Measuring::Bin(op, Box::new(a), Box::new(b));
        }
        Some(self.m_folded(from, a))
    }

    fn m_unary(&mut self, end: usize) -> Option<Measuring> {
        let from = self.i;
        if self.m_op(end, &['-']).is_some() {
            self.i += 1;
            let a = self.m_unary(end)?;
            return Some(self.m_folded(from, Measuring::Neg(Box::new(a))));
        }
        self.m_power(end)
    }

    /// Right-associative, and the exponent may carry its own sign: `2^-1`.
    fn m_power(&mut self, end: usize) -> Option<Measuring> {
        let from = self.i;
        let base = self.m_atom(end)?;
        if self.m_op(end, &['^']).is_none() {
            return Some(base);
        }
        self.i += if self.peek() == Some(&Tok::P('^')) { 1 } else { 2 };
        let e = self.m_unary(end)?;
        Some(self.m_folded(from, Measuring::Bin(Op::Pow, Box::new(base), Box::new(e))))
    }

    /// The token closing the bracket open at `self.i`, or `end` where none does before it.
    fn m_close(&self, end: usize) -> usize {
        let mut depth = 0i32;
        for j in self.i..end {
            match self.t[j].0 {
                Tok::P('(') | Tok::P('[') => depth += 1,
                Tok::P(')') | Tok::P(']') => {
                    depth -= 1;
                    if depth == 0 {
                        return j;
                    }
                }
                _ => {}
            }
        }
        end
    }

    fn m_atom(&mut self, end: usize) -> Option<Measuring> {
        let from = self.i;
        let call = matches!(self.t.get(self.i + 1), Some((Tok::P('('), _))) && self.i + 1 < end;
        match self.peek().cloned() {
            Some(Tok::P('(')) => {
                let close = self.m_close(end);
                self.i += 1;
                let inner = self.measuring(close)?;
                if !self.want_p(')') {
                    return None;
                }
                Some(self.m_folded(from, inner))
            }
            Some(Tok::Ident(w)) if call && MEASURES.iter().any(|m| m.0 == w) => {
                self.measurement(&w).map(Measuring::Measure)
            }
            // a function: of numbers, a number; of a measurement, kept for the elaborator to
            // refuse where it stands
            Some(Tok::Ident(_)) if call => {
                let name = self.ident()?;
                let close = self.m_close(end);
                self.i += 1;
                let mut args = Vec::new();
                if self.i < close {
                    loop {
                        let stop = self.m_arg_end(close);
                        args.push(self.measuring(stop)?);
                        if self.i >= close {
                            break;
                        }
                        self.i += 1;
                    }
                }
                if !self.want_p(')') {
                    return None;
                }
                Some(self.m_folded(from, Measuring::Call(name, args)))
            }
            Some(t) if self.i < end
                && !matches!(t, Tok::P(',' | ')' | ']'))
                && self.m_op(end, &['+', '-', '*', '/', '^']).is_none() =>
            {
                self.m_number(end);
                Some(self.m_text(from))
            }
            _ => {
                self.fail("expected a number or a measurement");
                None
            }
        }
    }

    /// Where the argument from here ends: at a `,` outside every bracket, or `close`.
    fn m_arg_end(&self, close: usize) -> usize {
        let mut depth = 0i32;
        for j in self.i..close {
            match self.t[j].0 {
                Tok::P('(') | Tok::P('[') => depth += 1,
                Tok::P(')') | Tok::P(']') => depth -= 1,
                Tok::P(',') if depth == 0 => return j,
                _ => {}
            }
        }
        close
    }

    /// A number's tokens, up to the operator after it: `10mm`, `design.module`, `1' 6"` — and a
    /// mixed fraction (`3 1/2`, `6 3/16"`), whose `/` is the literal's and no division.
    fn m_number(&mut self, end: usize) {
        let from = self.i;
        while self.i < end {
            if self.m_op(end, &['/']).is_some() {
                let num = |k: usize| matches!(self.t.get(k), Some((Tok::Num(_), _)));
                let mixed =
                    self.i >= from + 2 && num(self.i - 1) && num(self.i - 2) && num(self.i + 1);
                if !mixed {
                    return;
                }
                self.i += 2;
                continue;
            }
            if self.m_op(end, &['+', '-', '*', '^']).is_some() {
                return;
            }
            match self.peek() {
                Some(Tok::P(',')) => return,
                Some(Tok::P('(') | Tok::P('[')) => self.i = self.m_close(end) + 1,
                _ => self.i += 1,
            }
        }
        self.i = self.i.min(end);
    }

    /// `distance(p, f1)`, `angle(a, b, at: p)`: what it measures, by reference.
    fn measurement(&mut self, word: &str) -> Option<Measurement> {
        let lo = self.here().lo as usize;
        let &(_, measure, n) = MEASURES.iter().find(|m| m.0 == word).expect("a measurement");
        self.i += 1;
        if !self.want_p('(') {
            return None;
        }
        let mut args = Vec::new();
        let mut at = None;
        loop {
            let labelled = matches!(self.t.get(self.i + 1), Some((Tok::P(':'), _)));
            match self.peek().cloned() {
                // the point an angle between faces is read nearest (#183 F9), the one label a
                // measurement takes
                Some(Tok::Ident(l)) if labelled => {
                    if l != "at" || measure != crate::expr::Measure::Angle || args.len() != n {
                        self.fail(&format!("`{word}` takes no `{l}:`"));
                        return None;
                    }
                    self.i += 2;
                    at = Some(self.refr()?);
                }
                Some(Tok::Ident(_)) if at.is_none() && args.len() < n => args.push(self.refr()?),
                Some(Tok::Ident(_)) => {
                    self.fail(&format!("`{word}` takes {n} argument(s)"));
                    return None;
                }
                _ => {
                    self.fail(&format!("`{word}` measures geometry: its arguments are names"));
                    return None;
                }
            }
            if self.eat_p(')') {
                break;
            }
            if !self.want_p(',') {
                return None;
            }
        }
        if args.len() != n {
            self.fail(&format!("`{word}` takes {n} argument(s)"));
            return None;
        }
        Some(Measurement { measure, args, at, span: Span::new(lo, self.prev_hi()) })
    }
}
