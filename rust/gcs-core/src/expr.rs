//! Dimension expressions: `w = 1`, `h = w * 2`, `sin(h * 10)`.
//!
//! A dimension's number may be written as an arithmetic expression, and an expression may name
//! its value (`w = 1`) so other dimensions can use it.  The names make a graph over the
//! document's dimensions — `h = w * 2` hangs off `w`, `sin(h * 10)` off `h` — and evaluating it
//! is a topological walk of that graph: a dimension is computed once everything it names is.  A
//! name defined twice, a name nothing defines, a cycle, or a value that is not a number is an
//! error on the expressions it touches; those keep the number they last evaluated to, so the
//! solver always has a constant for every row, and the report says what is wrong.
//!
//! The language: numbers, names, `+ - * / ^` (`**` also raises), parentheses, `pi`, and the
//! functions `sqrt abs sin cos tan asin acos atan atan2 exp ln log floor ceil round min max
//! hypot`.  Trigonometry is in degrees, as every angle a person types or reads here is, and an
//! angle expression's value is degrees — the stored argument (radians) is converted on the way
//! in, exactly as the callout converts on the way out.

use crate::constraints::{Arg, Constraint, SpecKind};
use crate::model::Sketch;
use std::collections::{BTreeMap, BTreeSet};

/// A dimension argument written as text, with the number it last evaluated to in the argument's
/// own units (radians for an angle), so `Arg::num` and the kernels never see the text.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    pub text: String,
    pub value: f64,
}

/// A dimension written in terms of a *free* variable: a name no expression defines, which is
/// therefore an unknown of the sketch rather than a number.  Two dimensions reading the same
/// free name say the same thing about themselves, so they are tied to each other and the value
/// they share is left to the solver — one degree of freedom where two stated numbers would have
/// been none.
///
/// The tie is affine, and that is not a simplification for its own sake: `value = m * a + c` is
/// the whole of what a fixed-width kernel block can carry, one extra column and two constants.
/// `m` and `c` are in the argument's own units (radians for an angle) so a kernel needs no
/// conversion; the unknown itself is in the units a person writes, like every expression value
/// here.
///
/// It hangs off the `Constraint`, not off the `Expr`, because that is where the rest of the code
/// already assumes it: one free column appended by `params_on`, one `(m, c)` pair from
/// `consts_on`, one twin kernel.  Derived state, and the only writer is `evaluate` — so a
/// document, a paste and a rebuild carry the text and the number, and let the next evaluation
/// work the rest out again rather than inheriting an index into somebody else's parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Free {
    /// The unknown, as an index into `Sketch::params`.
    pub param: u32,
    pub m: f64,
    pub c: f64,
}

impl Expr {
    /// A dimension written as text, standing at the number it is worth until it is evaluated.
    pub fn new(text: impl Into<String>, value: f64) -> Expr {
        Expr { text: text.into(), value }
    }
}

/// Longest expression a document may carry; the parser is linear and depth-limited, this just
/// keeps an untrusted document from handing it a megabyte.
pub const MAX_TEXT: usize = 1000;
const MAX_DEPTH: usize = 64;

pub const CONSTANTS: &[(&str, f64)] = &[("pi", std::f64::consts::PI)];

/// (name, minimum arity, maximum arity)
pub const FUNCTIONS: &[(&str, usize, usize)] = &[
    ("sqrt", 1, 1),
    ("abs", 1, 1),
    ("sin", 1, 1),
    ("cos", 1, 1),
    ("tan", 1, 1),
    ("asin", 1, 1),
    ("acos", 1, 1),
    ("atan", 1, 1),
    ("atan2", 2, 2),
    ("exp", 1, 1),
    ("ln", 1, 1),
    ("log", 1, 1),
    ("floor", 1, 1),
    ("ceil", 1, 1),
    ("round", 1, 1),
    ("min", 2, usize::MAX),
    ("max", 2, usize::MAX),
    ("hypot", 2, 2),
];

fn is_builtin(name: &str) -> bool {
    CONSTANTS.iter().any(|&(n, _)| n == name) || FUNCTIONS.iter().any(|&(n, _, _)| n == name)
}

/* -- syntax --------------------------------------------------------------------- */

#[derive(Clone, Debug, PartialEq)]
pub enum Ast {
    Num(f64),
    Var(String),
    Neg(Box<Ast>),
    Bin(Op, Box<Ast>, Box<Ast>),
    Call(String, Vec<Ast>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Pow,
}

/// `name = body`, or just `body`.
#[derive(Clone, Debug, PartialEq)]
pub struct Parsed {
    pub name: Option<String>,
    pub body: Ast,
}

impl Ast {
    /// Every name the expression reads, once each, in name order.
    pub fn deps(&self) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        self.collect_deps(&mut out);
        out
    }

    fn collect_deps(&self, out: &mut BTreeSet<String>) {
        match self {
            Ast::Num(_) => {}
            Ast::Var(v) => {
                if !CONSTANTS.iter().any(|&(n, _)| n == v) {
                    out.insert(v.clone());
                }
            }
            Ast::Neg(a) => a.collect_deps(out),
            Ast::Bin(_, a, b) => {
                a.collect_deps(out);
                b.collect_deps(out);
            }
            Ast::Call(_, args) => {
                for a in args {
                    a.collect_deps(out);
                }
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(f64),
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

fn tokenize(text: &str) -> Result<Vec<(Tok, usize)>, String> {
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
        if c.is_ascii_digit() || (c == '.' && chars.get(i + 1).is_some_and(|d| d.is_ascii_digit())) {
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
            out.push((Tok::Num(v), at));
            continue;
        }
        if c.is_ascii_alphabetic() || c == '_' {
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                i += 1;
            }
            // A dot *between* identifier characters is part of the name: `c.center.x` is one
            // coordinate a curve is written over, not a name and a decimal point.  Only there —
            // `a.5` and a trailing `a.` are left alone, so `.5` is still the number it always was
            // and nothing that used to parse now parses differently.
            while i + 1 < chars.len()
                && chars[i] == '.'
                && (chars[i + 1].is_ascii_alphabetic() || chars[i + 1] == '_')
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
            Tok::Num(v) => Ok(Ast::Num(v)),
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
pub fn parse(text: &str) -> Result<Parsed, String> {
    if text.len() > MAX_TEXT {
        return Err(format!("expression longer than {MAX_TEXT} characters"));
    }
    let toks = tokenize(text)?;
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
        Tok::Num(v) => format!("{v}"),
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

/// A text that is a *number*, written in a notation rather than in digits — `3 1/8`.
///
/// This is what lets a dimension keep the form it was typed in.  `literal` does not claim such a
/// text, so it is stored as text with the value it came to, like any other written dimension;
/// what `notation` adds is that it is a notation and not a computation, so the drawing and the
/// constraint list print it as written rather than as "what it came to".  `3 1/8` on a callout
/// tells a reader something 3.125 does not, and it is what they typed.
///
/// `None` for an ordinary decimal, which prints as itself and has nothing to remember, and for
/// anything carrying a name, an operator or a function.
pub fn notation(text: &str) -> Option<f64> {
    let t = text.trim();
    if t.parse::<f64>().is_ok() {
        return None;
    }
    let toks = tokenize(t).ok()?;
    let v = match toks.as_slice() {
        [(Tok::Num(v), _), (Tok::End, _)] => *v,
        [(Tok::Op('-'), _), (Tok::Num(v), _), (Tok::End, _)] => -*v,
        [(Tok::Op('+'), _), (Tok::Num(v), _), (Tok::End, _)] => *v,
        _ => return None,
    };
    Some(v).filter(|v| v.is_finite())
}

/* -- evaluation --------------------------------------------------------------- */

fn call(name: &str, a: &[f64]) -> f64 {
    match name {
        "sqrt" => a[0].sqrt(),
        "abs" => a[0].abs(),
        "sin" => a[0].to_radians().sin(),
        "cos" => a[0].to_radians().cos(),
        "tan" => a[0].to_radians().tan(),
        "asin" => a[0].asin().to_degrees(),
        "acos" => a[0].acos().to_degrees(),
        "atan" => a[0].atan().to_degrees(),
        "atan2" => a[0].atan2(a[1]).to_degrees(),
        "exp" => a[0].exp(),
        "ln" => a[0].ln(),
        "log" => a[0].log10(),
        "floor" => a[0].floor(),
        "ceil" => a[0].ceil(),
        "round" => a[0].round(),
        "min" => a.iter().copied().fold(f64::INFINITY, f64::min),
        "max" => a.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        "hypot" => a[0].hypot(a[1]),
        _ => f64::NAN,
    }
}

/// What an expression comes to: a number, or `m` times the one free name it reads plus `c`.
///
/// Ordinary evaluation is the `free: None` case, and everything that is not a bare number is
/// written as a number here — so there is one evaluator, not two, and the rule that a free name
/// may only be scaled and offset falls out of the arithmetic rather than being checked for.
#[derive(Clone, Debug, PartialEq)]
pub struct Aff {
    /// The free name this stands in terms of; `None` for a plain number.
    pub free: Option<String>,
    pub m: f64,
    pub c: f64,
}

impl Aff {
    pub fn num(v: f64) -> Aff {
        Aff { free: None, m: 0.0, c: v }
    }

    fn of(name: &str) -> Aff {
        Aff { free: Some(name.to_string()), m: 1.0, c: 0.0 }
    }

    /// The number this is worth when the free name stands at `a`.
    pub fn at(&self, a: f64) -> f64 {
        self.m * a + self.c
    }

    /// The plain number, if it is one.
    pub fn number(&self) -> Option<f64> {
        self.free.is_none().then_some(self.c)
    }
}

/// Which free name a combination of two is in terms of.  One of them being a plain number is
/// what makes a product or a quotient legal; two different free names are not something an
/// affine form can hold, and neither is a second-degree one.
fn together(a: &Aff, b: &Aff) -> Result<Option<String>, String> {
    match (&a.free, &b.free) {
        (None, f) | (f, None) => Ok(f.clone()),
        (Some(x), Some(y)) if x == y => Ok(Some(x.clone())),
        (Some(x), Some(y)) => Err(free_pair(x, y)),
    }
}

fn free_pair(x: &str, y: &str) -> String {
    format!("`{x}` and `{y}` are both free, and a dimension can only follow one")
}

fn not_affine(name: &str) -> String {
    format!("`{name}` is free, so it can only be scaled and offset here")
}

/// Evaluate with the given names.  A name the environment lacks is *free* — an unknown the
/// solver moves rather than an error — so the result may be an affine form in it; `Err` is for
/// a free name used in a way an affine form cannot hold.  A result that is not a number
/// (`sqrt(-1)`, `1/0`) comes back as is, for the caller to judge.
pub fn eval(ast: &Ast, env: &BTreeMap<String, Aff>) -> Result<Aff, String> {
    Ok(match ast {
        Ast::Num(v) => Aff::num(*v),
        Ast::Var(name) => match CONSTANTS.iter().find(|&&(n, _)| n == name) {
            Some(&(_, v)) => Aff::num(v),
            None => match env.get(name) {
                Some(a) => a.clone(),
                None => Aff::of(name),
            },
        },
        Ast::Neg(a) => {
            let x = eval(a, env)?;
            Aff { free: x.free, m: -x.m, c: -x.c }
        }
        Ast::Bin(op, a, b) => {
            let (x, y) = (eval(a, env)?, eval(b, env)?);
            match op {
                Op::Add => Aff { free: together(&x, &y)?, m: x.m + y.m, c: x.c + y.c },
                Op::Sub => Aff { free: together(&x, &y)?, m: x.m - y.m, c: x.c - y.c },
                Op::Mul => match (x.number(), y.number()) {
                    (Some(k), _) => Aff { free: y.free, m: k * y.m, c: k * y.c },
                    (_, Some(k)) => Aff { free: x.free, m: k * x.m, c: k * x.c },
                    _ => return Err(free_pair_of(&x, &y)),
                },
                Op::Div => match y.number() {
                    Some(k) => Aff { free: x.free, m: x.m / k, c: x.c / k },
                    None => return Err(not_affine(y.free.as_deref().unwrap_or(""))),
                },
                Op::Pow => match (x.number(), y.number()) {
                    (Some(p), Some(q)) => Aff::num(p.powf(q)),
                    _ => return Err(free_pair_of(&x, &y)),
                },
            }
        }
        Ast::Call(name, args) => {
            let mut vals = Vec::with_capacity(args.len());
            for a in args {
                let v = eval(a, env)?;
                match v.number() {
                    Some(n) => vals.push(n),
                    None => return Err(not_affine(v.free.as_deref().unwrap_or(""))),
                }
            }
            Aff::num(call(name, &vals))
        }
    })
}

/// The complaint about a pair neither of which is a plain number: two free names cannot be
/// followed at once, and one free name cannot be multiplied by itself.
fn free_pair_of(x: &Aff, y: &Aff) -> String {
    match (&x.free, &y.free) {
        (Some(a), Some(b)) if a != b => free_pair(a, b),
        (Some(a), _) | (_, Some(a)) => not_affine(a),
        _ => "not a number".to_string(),
    }
}

/// Expression values are in the units a person writes: degrees for an angle.
pub fn to_arg_units(kind: SpecKind, v: f64) -> f64 {
    if kind == SpecKind::Angle {
        v.to_radians()
    } else {
        v
    }
}

pub fn to_user_units(kind: SpecKind, v: f64) -> f64 {
    if kind == SpecKind::Angle {
        v.to_degrees()
    } else {
        v
    }
}

/* -- the document's expressions ------------------------------------------------- */

/// One expression in the document, after evaluation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExprItem {
    /// The constraint it is an argument of, and which argument.
    pub id: u32,
    pub attr: &'static str,
    pub text: String,
    /// The name it defines, if any.
    pub name: Option<String>,
    /// Its value in the units a person reads (degrees for an angle) — the number it last
    /// evaluated to, when `error` is set.
    pub value: f64,
    /// The names it reads.
    pub deps: Vec<String>,
    /// The free names among them — the ones nothing defines, which are unknowns the solver
    /// moves rather than numbers.  At most one, since a dimension can only follow one; a list
    /// because that is what a reader wants to be handed, and because the deps beside it are one.
    pub free: Vec<String>,
    pub error: Option<String>,
}

struct Node {
    ci: usize,
    ai: usize,
    kind: SpecKind,
    parsed: Result<Parsed, String>,
}

/// Evaluate every expression in the sketch and write the results into the arguments, in an
/// order where each is computed after the names it reads; the report comes back in that order,
/// with the ones that could not be computed (and kept their last value) saying why.
///
/// Kahn's walk over the name graph: a node is ready when every name it reads has been computed.
/// Among the ready ones the earliest in the document goes first, so the order is reproducible
/// and reads naturally; what is never ready is on a cycle.
pub fn evaluate(sk: &mut Sketch) -> Vec<ExprItem> {
    let mut nodes: Vec<Node> = Vec::new();
    // Every binding to a free variable is worked out again from here, so a text that has stopped
    // reading one — or stopped parsing, or stopped being text at all — cannot leave a column
    // behind naming an unknown it no longer has anything to say about.  Clearing all of them and
    // not just the ones still carrying an expression is the point: the constraint that lost its
    // expression is exactly the one whose binding has to go.
    for (ci, c) in sk.constraints.iter_mut().enumerate() {
        c.free = None;
        for (ai, (_, kind)) in c.kind.spec().iter().enumerate() {
            if let Arg::Expr(e) = &c.args[ai] {
                nodes.push(Node { ci, ai, kind: *kind, parsed: parse(&e.text) });
            }
        }
    }
    let n = nodes.len();
    let mut errors: Vec<Option<String>> = vec![None; n];
    for (i, nd) in nodes.iter().enumerate() {
        if let Err(e) = &nd.parsed {
            errors[i] = Some(e.clone());
        }
    }
    // who defines what; a name defined twice is nobody's, and every definer is told
    let mut definers: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    for (i, nd) in nodes.iter().enumerate() {
        if let Ok(Parsed { name: Some(name), .. }) = &nd.parsed {
            definers.entry(name.clone()).or_default().push(i);
        }
    }
    let mut def: BTreeMap<String, usize> = BTreeMap::new();
    for (name, who) in &definers {
        if who.len() == 1 {
            def.insert(name.clone(), who[0]);
        } else {
            for &i in who {
                errors[i] = Some(format!("`{name}` is defined more than once"));
            }
        }
    }
    // edges: reader ← definer.  A name nothing defines at all is neither an edge nor an error:
    // it is a free variable, and what it is worth is the solver's business.  A name several
    // definitions claim is still an error — it is not undefined, it is ambiguous.
    let deps: Vec<Vec<String>> = nodes
        .iter()
        .map(|nd| match &nd.parsed {
            Ok(p) => p.body.deps().into_iter().collect(),
            Err(_) => Vec::new(),
        })
        .collect();
    let mut readers: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut indeg = vec![0usize; n];
    for i in 0..n {
        for name in &deps[i] {
            match def.get(name) {
                Some(&d) => {
                    readers[d].push(i);
                    indeg[i] += 1;
                }
                None => {
                    if errors[i].is_none() && definers.contains_key(name) {
                        errors[i] = Some(format!("`{name}` is defined more than once"));
                    }
                }
            }
        }
    }
    // the walk
    let mut ready: BTreeSet<usize> = (0..n).filter(|&i| indeg[i] == 0).collect();
    let mut order: Vec<usize> = Vec::with_capacity(n);
    let mut env: BTreeMap<String, Aff> = BTreeMap::new();
    let mut values: Vec<f64> = nodes
        .iter()
        .map(|nd| to_user_units(nd.kind, sk.constraints[nd.ci].args[nd.ai].num()))
        .collect();
    let mut free_of: Vec<Vec<String>> = vec![Vec::new(); n];
    // the free names actually bound this time round, and what one unit of each is worth in world
    // length — the largest any of its readers makes it, since that is the motion a step buys
    let mut bound: BTreeMap<String, f64> = BTreeMap::new();
    while let Some(&i) = ready.iter().next() {
        ready.remove(&i);
        order.push(i);
        let nd = &nodes[i];
        if errors[i].is_none() {
            let parsed = nd.parsed.as_ref().unwrap();
            let unready =
                deps[i].iter().filter(|d| def.contains_key(*d)).find(|d| !env.contains_key(*d));
            if let Some(name) = unready {
                errors[i] = Some(format!("`{name}` could not be evaluated"));
            } else {
                match eval(&parsed.body, &env) {
                    Ok(a) => match write_value(sk, nd, &a, &mut bound) {
                        Ok(v) => {
                            values[i] = v;
                            free_of[i] = a.free.iter().cloned().collect();
                            if let Some(name) = &parsed.name {
                                env.insert(name.clone(), a);
                            }
                        }
                        Err(e) => errors[i] = Some(e),
                    },
                    Err(e) => errors[i] = Some(e),
                }
            }
        }
        for &r in &readers[i] {
            indeg[r] -= 1;
            if indeg[r] == 0 {
                ready.insert(r);
            }
        }
    }
    retire_free(sk, &bound);
    // whatever never became ready is on a cycle, or downstream of one
    let stuck: Vec<usize> = (0..n).filter(|&i| indeg[i] > 0).collect();
    for &i in &stuck {
        if errors[i].is_none() {
            errors[i] = Some(cycle_text(i, &nodes, &deps, &def, &indeg));
        }
        order.push(i);
    }
    order
        .into_iter()
        .map(|i| {
            let nd = &nodes[i];
            let c = &sk.constraints[nd.ci];
            ExprItem {
                id: c.id,
                attr: c.spec()[nd.ai].0,
                text: match &c.args[nd.ai] {
                    Arg::Expr(e) => e.text.clone(),
                    _ => String::new(),
                },
                name: nd.parsed.as_ref().ok().and_then(|p| p.name.clone()),
                value: values[i],
                deps: deps[i].clone(),
                free: free_of[i].clone(),
                error: errors[i].clone(),
            }
        })
        .collect()
}

/// Write what one expression came to into its argument, and return it in the units a person
/// reads.  A plain number is stored as one; a form in a free name binds the argument to that
/// unknown, allocating it if this is the first expression to read it.
///
/// The seed is the obvious one: the dimension keeps the number it already stated, so writing `a`
/// over a dimension of 30 makes 30 what `a` is worth and nothing moves.  A dimension that states
/// *nothing* — one written as a name from the start, `Distance(p, q, "a")`, which arrives at
/// zero because there was never a number — takes what the geometry reads instead; see `settle`.
///
/// Only the *first* reader seeds it.  A second dimension saying `a / 2` is stating a relation,
/// and it is the geometry that must move to meet it, not the variable that must bend to keep the
/// geometry still.
fn write_value(
    sk: &mut Sketch,
    nd: &Node,
    a: &Aff,
    bound: &mut BTreeMap<String, f64>,
) -> Result<f64, String> {
    let (ci, ai, kind) = (nd.ci, nd.ai, nd.kind);
    let text = match &sk.constraints[ci].args[ai] {
        Arg::Expr(e) => e.text.clone(),
        _ => unreachable!("a node is an expression argument"),
    };
    let Some(name) = a.free.clone() else {
        if !a.c.is_finite() {
            return Err("does not evaluate to a number".to_string());
        }
        sk.constraints[ci].args[ai] = Arg::Expr(Expr::new(text, to_arg_units(kind, a.c)));
        return Ok(a.c);
    };
    if !a.m.is_finite() || !a.c.is_finite() {
        return Err("does not evaluate to a number".to_string());
    }
    // only a stated number can become an unknown, and only where there is a kernel to read it as
    // a column.  The two go together — `every_dimension_can_be_written_free` — so this is the
    // belt to that braces: an expression somewhere it was never meant to be says so rather than
    // selecting a kernel that does not exist.
    if !kind.is_dimension() || sk.constraints[ci].kind.free_kernel().is_none() {
        return Err(format!("`{name}` is free, and this is not a dimension it can be"));
    }
    // a claim compiles to no rows, so an unknown bound here would sit in no equation at all — a
    // degree of freedom minted by a statement that promised to add nothing
    if sk.constraints[ci].claim {
        return Err(format!("`{name}` is free, and a claim may not bind an unknown"));
    }
    // a form that does not actually move with the variable states nothing about it, and there
    // would be no way back from the dimension to a value for it
    if a.m == 0.0 {
        return Err(format!("`{name}` does not affect this dimension"));
    }
    let stated = sk.constraints[ci].args[ai].num();
    let seed = (to_user_units(kind, stated) - a.c) / a.m;
    let (param, fresh) = free_param(sk, &name, seed);
    let (m, c) = (to_arg_units(kind, a.m), to_arg_units(kind, a.c));
    // one unit of the variable is worth this much world length through this dimension: an angle
    // in degrees moves the drawing by the arm it turns, everything else is a length already
    let reach = m.abs() * if kind == SpecKind::Angle { sk.extent().max(1.0) } else { 1.0 };
    let was = bound.entry(name).or_insert(0.0);
    *was = was.max(reach);
    let free = Free { param, m, c };
    if fresh && stated == 0.0 {
        // the bound copy is what `settle` measures through: the columns and the constants it
        // asks for are the ones this constraint selects once the binding is in
        let mut bound_copy = sk.constraints[ci].clone();
        bound_copy.free = Some(free);
        settle(sk, &bound_copy, param, reach);
    }
    let value = a.at(sk.params[param as usize].value);
    sk.constraints[ci].free = Some(free);
    sk.constraints[ci].args[ai] = Arg::Expr(Expr::new(text, to_arg_units(kind, value)));
    Ok(value)
}

/// Move a newly allocated free variable to where the dimension it first appears on is satisfied
/// at the current geometry.  This is for the dimension that states no number to seed from — one
/// written as a name from the start — and the answer it wants is "leave the drawing alone".
///
/// That row is a function of the one unknown, and Newton on it is the whole method.  It asks the
/// kernel and nothing else, so a new dimension type is seeded correctly by declaring one, with no
/// table here to extend — and since the seed is only a starting point, an ill-conditioned row
/// costs the best value found and never correctness.
///
/// Starting from nothing is exactly where the derivative can vanish: a distance is written
/// squared, so at zero the row is flat and Newton has nowhere to go.  A step that is not a number
/// is answered by moving the dimension one extent instead, which `reach` — the world length one
/// unit of the variable is worth — converts into a step in the variable.
fn settle(sk: &mut Sketch, c: &Constraint, param: u32, reach: f64) {
    let kid = c.kernel_id();
    if crate::kernels::kernel_by_id(kid).n_res != 1 {
        return;
    }
    let ps = c.params(sk);
    let col = ps.len() - 1;   // the free column comes last — see `Constraint::params_on`
    let consts = c.consts(sk);
    let kick = sk.extent().max(1.0) / if reach.is_finite() && reach > 0.0 { reach } else { 1.0 };
    let err = |sk: &Sketch| {
        let v: Vec<f64> = ps.iter().map(|&p| sk.params[p as usize].value).collect();
        let (r, j) = crate::kernels::eval_one(kid, &v, &consts);
        (r[0], j[col])
    };
    let start = sk.params[param as usize].value;
    let start_err = err(sk).0.abs();
    for _ in 0..24 {
        let (r, dr) = err(sk);
        let here = sk.params[param as usize].value;
        let step = r / dr;
        sk.params[param as usize].value = if step.is_finite() { here - step } else { here + kick };
        if step.is_finite() && step.abs() <= 1e-12 * (1.0 + here.abs()) {
            break;
        }
    }
    // a seed is only a starting point, so a walk that ended worse than it began is discarded
    if !(err(sk).0.abs() < start_err) {
        sk.params[param as usize].value = start;
    }
}

/// The unknown a free name stands for, and whether this is the first expression to read it since
/// it last meant anything — which is what decides whether it takes a seed.  A name still in use
/// keeps both its unknown and its value, so an edit elsewhere in the document does not disturb it.
///
/// A name that had been retired gets its old slot back rather than a new one.  Reusing it is what
/// keeps `Sketch::params` from growing every time a dimension is toggled between `q` and a
/// number: the slot is the sketch's name for the variable, and the parameter *count* is part of
/// `topology_key`, so allocating a fresh one would also miss the plan cache on the way back to a
/// shape already compiled.  It is still seeded, since as far as the document is concerned the
/// variable is new.
fn free_param(sk: &mut Sketch, name: &str, seed: f64) -> (u32, bool) {
    if let Some(&p) = sk.free_vars.get(name) {
        let retired = sk.params[p as usize].fixed;
        sk.params[p as usize].fixed = false;
        if retired {
            sk.params[p as usize].value = seed;
        }
        return (p, retired);
    }
    let p = sk.param(seed, false, &format!("${name}")) as u32;
    sk.free_vars.insert(name.to_string(), p);
    (p, true)
}

/// Retire the free variables nothing reads any more and give the rest their step scale.
///
/// Retiring is to `fixed`, as it is for a constraint's own unknowns: every index above a
/// parameter names something, so the slot stays and stops being an unknown.  A free parameter no
/// equation mentions is a degree of freedom the sketch does not have, and diagnosis would say so.
/// The *name* keeps the slot too, so that reading it again reuses the unknown instead of leaking
/// a new one — see `free_param`.  The rebuild walk is what reclaims both.
fn retire_free(sk: &mut Sketch, bound: &BTreeMap<String, f64>) {
    let gone: Vec<u32> = sk
        .free_vars
        .iter()
        .filter(|(n, _)| !bound.contains_key(*n))
        .map(|(_, &p)| p)
        .collect();
    for p in gone {
        sk.params[p as usize].fixed = true;
    }
    for (name, &reach) in bound {
        if let Some(&p) = sk.free_vars.get(name) {
            sk.params[p as usize].scale = if reach.is_finite() && reach > 0.0 { reach } else { 1.0 };
        }
    }
}

/// Bring every dimension written in terms of a free variable up to the number that variable now
/// stands at.  The binding is what the kernels read, so a solve needs nothing from this; the
/// *text* of a dimension does, and so does anyone asking what it says without a sketch in hand.
pub fn sync_free(sk: &mut Sketch) {
    if sk.free_vars.is_empty() {
        return;   // a document with no free variable in it pays nothing
    }
    let Sketch { constraints, params, .. } = sk;
    for c in constraints {
        let Some(f) = c.free else { continue };
        let v = f.m * params[f.param as usize].value + f.c;
        // a constraint carrying a binding has exactly one dimension, and it is written as text —
        // that is what having one means
        if let Some(Arg::Expr(e)) = c.args.iter_mut().find(|a| matches!(a, Arg::Expr(_))) {
            e.value = v;
        }
    }
}

/// `circular: w → h → w`, found by walking definitions from `i` through the stuck nodes; or,
/// for a node that only reads from a cycle without being on one, which name it waits for.
fn cycle_text(
    i: usize,
    nodes: &[Node],
    deps: &[Vec<String>],
    def: &BTreeMap<String, usize>,
    indeg: &[usize],
) -> String {
    // depth-first along unresolved definitions, looking for a way back to `i`
    let mut path: Vec<usize> = vec![i];
    let mut stack: Vec<(usize, usize)> = vec![(i, 0)];   // (node, next dep index)
    let mut seen: BTreeSet<usize> = BTreeSet::new();
    while let Some(&(u, k)) = stack.last() {
        if k >= deps[u].len() {
            stack.pop();
            path.pop();
            continue;
        }
        stack.last_mut().unwrap().1 += 1;
        let name = &deps[u][k];
        let Some(&d) = def.get(name) else { continue };
        if indeg[d] == 0 {
            continue;   // resolved: not part of the tangle
        }
        if d == i {
            let names: Vec<String> = path
                .iter()
                .map(|&p| nodes[p].parsed.as_ref().ok().and_then(|q| q.name.clone()))
                .map(|n| n.unwrap_or_else(|| "?".to_string()))
                .collect();
            return format!("circular: {} → {}", names.join(" → "), names[0]);
        }
        if seen.insert(d) {
            path.push(d);
            stack.push((d, 0));
        }
    }
    let waiting = deps[i]
        .iter()
        .find(|n| def.get(*n).is_some_and(|&d| indeg[d] > 0))
        .cloned()
        .unwrap_or_default();
    format!("`{waiting}` could not be evaluated")
}

/// Write a dimension from text: a bare number becomes a constant (in the argument's units), and
/// anything else an expression, evaluated along with the rest of the document.  `Err` when the
/// text does not parse or names no dimension, and nothing is changed; `Ok(Some(why))` when it
/// was stored but could not be computed (a name nothing defines yet), so a caller can say so.
pub fn set_dimension(sk: &mut Sketch, id: u32, attr: &str, text: &str) -> Result<Option<String>, String> {
    let (i, kind, value) = {
        let c = sk.constraint(id).ok_or_else(|| format!("no constraint {id}"))?;
        let i = c.arg_index(attr).ok_or_else(|| format!("`{attr}` is not an argument"))?;
        let kind = c.spec()[i].1;
        if !kind.is_dimension() {
            return Err(format!("`{attr}` is not a dimension"));
        }
        (i, kind, c.args[i].num())   // the last number, until the expression is computed
    };
    let text = text.trim();
    if let Some(v) = literal(text) {
        sk.constraint_mut(id).unwrap().args[i] = Arg::Num(to_arg_units(kind, v));
        evaluate(sk);   // whatever read a name this used to define
        return Ok(None);
    }
    parse(text)?;
    sk.constraint_mut(id).unwrap().args[i] = Arg::Expr(Expr::new(text, value));
    let mine = evaluate(sk).into_iter().find(|it| it.id == id && it.attr == attr);
    Ok(mine.and_then(|it| it.error))
}

/// Whether a constraint carries any expression — what decides if adding it needs an evaluation.
pub fn has_expr(args: &[Arg]) -> bool {
    args.iter().any(|a| matches!(a, Arg::Expr(_)))
}
