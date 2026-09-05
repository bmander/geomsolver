//! Tokenize and parse dimension expressions and unit notation.

use super::{is_builtin, Ast, Op, Parsed, FUNCTIONS, MAX_DEPTH, MAX_TEXT};
use crate::units::{unit, Dim, Units};

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(f64, Dim),
    Ident(String),
    Op(char),
    Pow,
    LParen,
    RParen,
    Comma,
    Assign,
    End,
}

/// The `n/d` of a mixed number, if one follows `from` across at least one space: the digits,
/// the slash and the digits, written tight the way a drawing writes them.  `None` for anything
/// else, which leaves the tokens to be read the ordinary way — `3 x/2` is still three, x, over
/// two, and `3 1 / 2` is the juxtaposition it looks like.
fn mixed_fraction(chars: &[char], from: usize) -> Option<(f64, f64, usize)> {
    let mut i = from;
    while chars.get(i).is_some_and(|c| c.is_whitespace()) {
        i += 1;
    }
    if i == from {
        return None; // no space: `31/2` is a division, not three and a half
    }
    let digits = |i: &mut usize| {
        let start = *i;
        while chars.get(*i).is_some_and(|c| c.is_ascii_digit()) {
            *i += 1;
        }
        chars[start..*i].iter().collect::<String>().parse::<f64>().ok()
    };
    let num = digits(&mut i)?;
    if chars.get(i) != Some(&'/') {
        return None;
    }
    i += 1;
    let den = digits(&mut i)?;
    // a decimal point or another slash means this was not a mixed number after all
    if chars.get(i).is_some_and(|&c| c == '.' || c.is_ascii_digit()) {
        return None;
    }
    Some((num, den, i))
}

/// Parse a numeric unit suffix, including feet/inches and mixed fractions.
/// Convert to document units and retain the dimension for type checking.
fn suffix(chars: &[char], i: &mut usize, v: f64, units: Units) -> Result<(f64, Dim), String> {
    // `'` and `"` are punctuation, not words, so they are read here rather than from the table
    let mark = |c: Option<&char>| match c {
        Some('\'') => unit("ft"),
        Some('"') => unit("in"),
        _ => None,
    };
    if let Some(u) = mark(chars.get(*i)) {
        *i += 1;
        let mut total = units.convert(v, u)?;
        // feet *and* inches: the second half across a space, in the smaller unit
        let mut j = *i;
        while chars.get(j).is_some_and(|c| c.is_whitespace()) {
            j += 1;
        }
        if j > *i {
            if let Some((inches, end)) = inch_part(chars, j) {
                total += units.convert(inches, unit("in").expect("in"))?;
                *i = end;
            }
        }
        return Ok((total, Dim::LENGTH));
    }
    // a word: `mm`, `deg`, …  Only immediately after the digits — `2 mm` is a juxtaposition and
    // not a unit, the same rule the mark above follows.
    let start = *i;
    let mut j = *i;
    while chars.get(j).is_some_and(|c| c.is_ascii_alphabetic()) {
        j += 1;
    }
    if j == start {
        return Ok((v, Dim::SCALAR));
    }
    let word: String = chars[start..j].iter().collect();
    match unit(&word) {
        Some(u) => {
            *i = j;
            Ok((units.convert(v, u)?, u.dim))
        }
        // not a unit: it is a name, and `2r` is two times r as it always was
        None => Ok((v, Dim::SCALAR)),
    }
}

/// The inches half of `1' 6 3/16"` — digits, optionally a mixed fraction, then the inch mark.
/// `None` for anything else, which leaves the tokens to be read the ordinary way.
fn inch_part(chars: &[char], from: usize) -> Option<(f64, usize)> {
    let mut i = from;
    let start = i;
    while chars.get(i).is_some_and(|c| c.is_ascii_digit() || *c == '.') {
        i += 1;
    }
    if i == start {
        return None;
    }
    let mut v: f64 = chars[start..i].iter().collect::<String>().parse().ok()?;
    if let Some((num, den, end)) = mixed_fraction(chars, i) {
        if den == 0.0 {
            return None;
        }
        v += num / den;
        i = end;
    }
    (chars.get(i) == Some(&'"')).then(|| (v, i + 1))
}

fn tokenize(text: &str, units: Units) -> Result<Vec<(Tok, usize)>, String> {
    let chars: Vec<char> = text.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        let at = i;
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        if c.is_ascii_digit() || (c == '.' && chars.get(i + 1).is_some_and(|d| d.is_ascii_digit()))
        {
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                i += 1;
            }
            if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
                let mut j = i + 1;
                if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
                    j += 1;
                }
                if j < chars.len() && chars[j].is_ascii_digit() {
                    i = j;
                    while i < chars.len() && chars[i].is_ascii_digit() {
                        i += 1;
                    }
                }
            }
            let s: String = chars[start..i].iter().collect();
            let mut v: f64 = s.parse().map_err(|_| format!("bad number `{s}` at {}", start + 1))?;
            // `3 1/2` is three and a half, the way a drawing writes it.  Only a whole number
            // takes a fraction, and only across a space: `31/2` is still a division, and so is
            // a bare `1/2`.
            if !s.contains(['.', 'e', 'E']) {
                if let Some((num, den, end)) = mixed_fraction(&chars, i) {
                    if den == 0.0 {
                        return Err(format!("`/0` in the fraction at {}", start + 1));
                    }
                    v += num / den;
                    i = end;
                }
            }
            // a unit on the number — `80mm`, `45deg`, `1' 6 3/16"`.  A *space* is what tells
            // the readings apart, which is the rule `mixed_fraction` already keeps: `1' 6"` is
            // one literal for the same reason `3 1/2` is.
            let (v, d) = suffix(&chars, &mut i, v, units)?;
            out.push((Tok::Num(v, d), at));
            continue;
        }
        // A name may begin with `#`: a block copy's prefix (`#3.0.`) is what the flattener puts
        // in front of a name declared inside a `cycle` or a `repeat`, so a dimension named in
        // one — or an unbound formal of an instance in one — is `#3.0.w`, and the graph has to
        // read it.  The digits and dots after the `#` are the copy's, so they are part of the
        // name here where `3.0` alone would be a number.
        let key =
            c == '#' && chars.get(i + 1).is_some_and(|d| d.is_ascii_alphanumeric() || *d == '_');
        if c.is_ascii_alphabetic() || c == '_' || key {
            let start = i;
            if key {
                i += 1;
            }
            while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                i += 1;
            }
            // A dot *between* identifier characters is part of the name: `c.center.x` is one
            // coordinate a curve is written over, not a name and a decimal point.  Only there —
            // `a.5` and a trailing `a.` are left alone, so `.5` is still the number it always was
            // and nothing that used to parse now parses differently.
            while i + 1 < chars.len()
                && chars[i] == '.'
                && (chars[i + 1].is_ascii_alphabetic()
                    || chars[i + 1] == '_'
                    || (key && chars[i + 1].is_ascii_digit()))
            {
                i += 1;
                while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                    i += 1;
                }
            }
            out.push((Tok::Ident(chars[start..i].iter().collect()), at));
            continue;
        }
        i += 1;
        let t = match c {
            '+' | '-' | '/' => Tok::Op(c),
            '*' => {
                if chars.get(i) == Some(&'*') {
                    i += 1;
                    Tok::Pow
                } else {
                    Tok::Op('*')
                }
            }
            '^' => Tok::Pow,
            '(' => Tok::LParen,
            ')' => Tok::RParen,
            ',' => Tok::Comma,
            '=' => Tok::Assign,
            _ => return Err(format!("unexpected `{c}` at {}", at + 1)),
        };
        out.push((t, at));
    }
    out.push((Tok::End, chars.len()));
    Ok(out)
}

struct Parser {
    toks: Vec<(Tok, usize)>,
    pos: usize,
    depth: usize,
}

impl Parser {
    fn peek(&self) -> &Tok {
        &self.toks[self.pos].0
    }

    fn here(&self) -> usize {
        self.toks[self.pos].1 + 1
    }

    fn next(&mut self) -> Tok {
        let t = self.toks[self.pos].0.clone();
        if self.pos + 1 < self.toks.len() {
            self.pos += 1;
        }
        t
    }

    fn enter(&mut self) -> Result<(), String> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return Err("expression nested too deeply".to_string());
        }
        Ok(())
    }

    fn leave(&mut self) {
        self.depth -= 1;
    }

    fn expr(&mut self) -> Result<Ast, String> {
        self.enter()?;
        let mut lhs = self.term()?;
        loop {
            let op = match self.peek() {
                Tok::Op('+') => Op::Add,
                Tok::Op('-') => Op::Sub,
                _ => break,
            };
            self.next();
            let rhs = self.term()?;
            lhs = Ast::Bin(op, Box::new(lhs), Box::new(rhs));
        }
        self.leave();
        Ok(lhs)
    }

    fn term(&mut self) -> Result<Ast, String> {
        let mut lhs = self.unary()?;
        loop {
            let op = match self.peek() {
                Tok::Op('*') => Op::Mul,
                Tok::Op('/') => Op::Div,
                _ => break,
            };
            self.next();
            let rhs = self.unary()?;
            lhs = Ast::Bin(op, Box::new(lhs), Box::new(rhs));
        }
        Ok(lhs)
    }

    /// A sign binds looser than `^`: `-2^2` is −4, as it is on paper.
    fn unary(&mut self) -> Result<Ast, String> {
        self.enter()?;
        let r = match self.peek() {
            Tok::Op('-') => {
                self.next();
                Ast::Neg(Box::new(self.unary()?))
            }
            Tok::Op('+') => {
                self.next();
                self.unary()?
            }
            _ => self.power()?,
        };
        self.leave();
        Ok(r)
    }

    /// Right-associative, and the exponent may carry its own sign: `2^-1`.
    fn power(&mut self) -> Result<Ast, String> {
        let base = self.atom()?;
        if *self.peek() == Tok::Pow {
            self.next();
            let e = self.unary()?;
            return Ok(Ast::Bin(Op::Pow, Box::new(base), Box::new(e)));
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<Ast, String> {
        let at = self.here();
        match self.next() {
            Tok::Num(v, d) => Ok(Ast::Num(v, d)),
            Tok::Ident(name) => {
                if *self.peek() != Tok::LParen {
                    return Ok(Ast::Var(name));
                }
                self.next();
                let mut args = Vec::new();
                if *self.peek() != Tok::RParen {
                    loop {
                        args.push(self.expr()?);
                        match self.next() {
                            Tok::Comma => continue,
                            Tok::RParen => break,
                            _ => return Err(format!("expected `,` or `)` at {}", self.here())),
                        }
                    }
                } else {
                    self.next();
                }
                let Some(&(_, lo, hi)) = FUNCTIONS.iter().find(|f| f.0 == name) else {
                    return Err(format!("unknown function `{name}`"));
                };
                if args.len() < lo || args.len() > hi {
                    let want = if lo == hi {
                        format!("{lo}")
                    } else if hi == usize::MAX {
                        format!("at least {lo}")
                    } else {
                        format!("{lo} to {hi}")
                    };
                    return Err(format!("`{name}` takes {want} argument(s)"));
                }
                Ok(Ast::Call(name, args))
            }
            Tok::LParen => {
                let e = self.expr()?;
                if self.next() != Tok::RParen {
                    return Err(format!("expected `)` at {at}"));
                }
                Ok(e)
            }
            Tok::End => Err("expected a value at the end".to_string()),
            _ => Err(format!("expected a value at {at}")),
        }
    }
}

/// Parse `name = body` or `body`.  A syntax error, an unknown function, a wrong arity, or a
/// definition of a built-in name is an `Err`; a name nothing defines is not — that is the
/// document's business, not the text's.
/// Parse in **drawing units**: a document that names none, where a unit suffix is an error.
/// Every caller that has a document uses `parse_in`.
pub fn parse(text: &str) -> Result<Parsed, String> {
    parse_in(text, Units::default())
}

pub fn parse_in(text: &str, units: Units) -> Result<Parsed, String> {
    if text.len() > MAX_TEXT {
        return Err(format!("expression longer than {MAX_TEXT} characters"));
    }
    let toks = tokenize(text, units)?;
    let mut p = Parser { toks, pos: 0, depth: 0 };
    let mut name = None;
    if let (Tok::Ident(n), Some((Tok::Assign, _))) = (p.peek().clone(), p.toks.get(1)) {
        if is_builtin(&n) {
            return Err(format!("`{n}` is built in and cannot be defined"));
        }
        name = Some(n);
        p.next();
        p.next();
    }
    let body = p.expr()?;
    if *p.peek() != Tok::End {
        return Err(format!("unexpected `{}` at {}", tok_text(p.peek()), p.here()));
    }
    Ok(Parsed { name, body })
}

fn tok_text(t: &Tok) -> String {
    match t {
        Tok::Num(v, _) => format!("{v}"),
        Tok::Ident(s) => s.clone(),
        Tok::Op(c) => c.to_string(),
        Tok::Pow => "^".to_string(),
        Tok::LParen => "(".to_string(),
        Tok::RParen => ")".to_string(),
        Tok::Comma => ",".to_string(),
        Tok::Assign => "=".to_string(),
        Tok::End => "end".to_string(),
    }
}

/// The name an expression defines, if it parses and has one.
pub fn name_of(text: &str) -> Option<String> {
    parse(text).ok().and_then(|p| p.name)
}

/// A bare number in digits — `5`, `-2.5`, `1e3` — is a constant, not an expression.  `None` for
/// anything else (names, operators, or a non-finite literal such as `inf`).
///
/// A mixed fraction is deliberately *not* one.  `3 1/8` is a number, but it is a number written
/// a particular way, and collapsing it to 3.125 throws away the way — see `notation`.
pub fn literal(text: &str) -> Option<f64> {
    text.trim().parse::<f64>().ok().filter(|v| v.is_finite())
}

/// Recognize literal notation such as `3 1/8`, as distinct from an expression.
/// Edits may replace literal notation, but must preserve computed expressions.
pub fn notation(text: &str) -> bool {
    let t = text.trim();
    if t.parse::<f64>().is_ok() {
        return false;
    }
    let mm = Units::with_length("mm").expect("mm is a unit");
    let Ok(toks) = tokenize(t, mm) else { return false };
    // a sign in front changes nothing about whether the text is one number
    let v = match toks.as_slice() {
        [(Tok::Num(v, _), _), (Tok::End, _)]
        | [(Tok::Op('-' | '+'), _), (Tok::Num(v, _), _), (Tok::End, _)] => *v,
        _ => return false,
    };
    v.is_finite()
}

/// Whether a text that is one number (`notation`) wrote its unit — `45deg`, `3mm`, `1' 6"` —
/// as against a bare `3 1/8`, which takes the unit of the slot it stands in.  What a printer
/// asks before putting a degree sign after it.
pub fn names_unit(text: &str) -> bool {
    let mm = Units::with_length("mm").expect("mm is a unit");
    let Ok(toks) = tokenize(text.trim(), mm) else { return false };
    match toks.as_slice() {
        [(Tok::Num(_, d), _), (Tok::End, _)]
        | [(Tok::Op('-' | '+'), _), (Tok::Num(_, d), _), (Tok::End, _)] => !d.is_scalar(),
        _ => false,
    }
}
