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

use crate::constraints::{Arg, SpecKind};
use crate::model::Sketch;
use std::collections::{BTreeMap, BTreeSet};

/// A dimension argument written as text, with the number it last evaluated to in the argument's
/// own units (radians for an angle), so `Arg::num` and the kernels never see the text.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    pub text: String,
    pub value: f64,
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
            let v: f64 = s.parse().map_err(|_| format!("bad number `{s}` at {}", start + 1))?;
            out.push((Tok::Num(v), at));
            continue;
        }
        if c.is_ascii_alphabetic() || c == '_' {
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                i += 1;
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

/// A bare number — `5`, `-2.5`, `1e3` — is a constant, not an expression.  `None` for
/// anything else (names, operators, or a non-finite literal such as `inf`).
pub fn literal(text: &str) -> Option<f64> {
    text.trim().parse::<f64>().ok().filter(|v| v.is_finite())
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

/// Evaluate with the given names.  `Err` names the first name the environment lacks; a result
/// that is not a number (`sqrt(-1)`, `1/0`) comes back as is, for the caller to judge.
pub fn eval(ast: &Ast, env: &BTreeMap<String, f64>) -> Result<f64, String> {
    Ok(match ast {
        Ast::Num(v) => *v,
        Ast::Var(name) => match CONSTANTS.iter().find(|&&(n, _)| n == name) {
            Some(&(_, v)) => v,
            None => *env.get(name).ok_or_else(|| format!("`{name}` is not defined"))?,
        },
        Ast::Neg(a) => -eval(a, env)?,
        Ast::Bin(op, a, b) => {
            let (x, y) = (eval(a, env)?, eval(b, env)?);
            match op {
                Op::Add => x + y,
                Op::Sub => x - y,
                Op::Mul => x * y,
                Op::Div => x / y,
                Op::Pow => x.powf(y),
            }
        }
        Ast::Call(name, args) => {
            let mut vals = Vec::with_capacity(args.len());
            for a in args {
                vals.push(eval(a, env)?);
            }
            call(name, &vals)
        }
    })
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
    for (ci, c) in sk.constraints.iter().enumerate() {
        for (ai, (_, kind)) in c.spec().iter().enumerate() {
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
    // edges: reader ← definer
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
                    if errors[i].is_none() {
                        errors[i] = Some(if definers.contains_key(name) {
                            format!("`{name}` is defined more than once")
                        } else {
                            format!("`{name}` is not defined")
                        });
                    }
                }
            }
        }
    }
    // the walk
    let mut ready: BTreeSet<usize> = (0..n).filter(|&i| indeg[i] == 0).collect();
    let mut order: Vec<usize> = Vec::with_capacity(n);
    let mut env: BTreeMap<String, f64> = BTreeMap::new();
    let mut values: Vec<f64> = nodes
        .iter()
        .map(|nd| to_user_units(nd.kind, sk.constraints[nd.ci].args[nd.ai].num()))
        .collect();
    while let Some(&i) = ready.iter().next() {
        ready.remove(&i);
        order.push(i);
        let nd = &nodes[i];
        if errors[i].is_none() {
            let parsed = nd.parsed.as_ref().unwrap();
            let unready = deps[i].iter().find(|d| !env.contains_key(*d));
            if let Some(name) = unready {
                errors[i] = Some(format!("`{name}` could not be evaluated"));
            } else {
                match eval(&parsed.body, &env) {
                    Ok(v) if v.is_finite() => {
                        values[i] = v;
                        if let Some(name) = &parsed.name {
                            env.insert(name.clone(), v);
                        }
                        let text = match &sk.constraints[nd.ci].args[nd.ai] {
                            Arg::Expr(e) => e.text.clone(),
                            _ => unreachable!(),
                        };
                        sk.constraints[nd.ci].args[nd.ai] =
                            Arg::Expr(Expr { text, value: to_arg_units(nd.kind, v) });
                    }
                    Ok(_) => errors[i] = Some("does not evaluate to a number".to_string()),
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
                error: errors[i].clone(),
            }
        })
        .collect()
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
    sk.constraint_mut(id).unwrap().args[i] = Arg::Expr(Expr { text: text.to_string(), value });
    let mine = evaluate(sk).into_iter().find(|it| it.id == id && it.attr == attr);
    Ok(mine.and_then(|it| it.error))
}

/// Whether a constraint carries any expression — what decides if adding it needs an evaluation.
pub fn has_expr(args: &[Arg]) -> bool {
    args.iter().any(|a| matches!(a, Arg::Expr(_)))
}
