//! A compiled expression, and its derivatives.
//!
//! `expr.rs` reads the little language a person writes a number in and evaluates it once.  A
//! *curve* written in that language has to be evaluated in the solver's inner loop, thousands of
//! times, together with its derivatives — so it is compiled here into a flat list of operations
//! over a small variable table, and differentiated forward as it runs.
//!
//! The variables are the curve's parameter and whatever coordinates of the sketch the expression
//! reads: `u`, and `c.center.x`, `c.center.y`, `c.r` for a curve written over a circle.  A
//! contact with the curve needs `∂C/∂u` to know which way along it the point may slide, and
//! `∂C/∂θ` for each of the rest to know how the curve moves when the geometry it is written over
//! does — so the gradient is taken in all of them at once and there is one pass, not two.
//!
//! **Units are `expr.rs`'s and are not restated here.**  Its trigonometry is in degrees, so a
//! derivative of `sin` carries the `π/180` that convention implies and `atan`'s carries its
//! inverse.  `tests/tape.rs` checks every value against `expr::eval` and every derivative against
//! a finite difference of this evaluator, so neither the units nor the calculus can drift from
//! the language they belong to.
//!
//! A tape is `Copy`-free but cheap to clone and encodes to a flat `Vec<f64>`, which is how it
//! reaches a kernel: it rides in the constraint's constants, where a block already carries
//! per-constraint numbers, so no kernel signature has to learn about curves.

use crate::expr::{Ast, Op as EOp};
use std::collections::BTreeMap;

/// How many operations one expression may compile to.  A document is untrusted input and
/// `wasm32-unknown-unknown` aborts rather than unwinding.
pub const MAX_OPS: usize = 4096;

/// How many variables one tape may read: a curve parameter and the coordinates of the geometry
/// it is written over.  Small on purpose — it is also the width of every gradient.
pub const MAX_VARS: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fn1 {
    Sqrt,
    Abs,
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Exp,
    Ln,
    Log,
    Floor,
    Ceil,
    Round,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fn2 {
    Atan2,
    Min,
    Max,
    Hypot,
}

/// One instruction.  Operands are indices of *earlier* slots, so a tape runs front to back with
/// no stack and no branching beyond the opcode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Op {
    Const(f64),
    Var(u32),
    Neg(u32),
    Add(u32, u32),
    Sub(u32, u32),
    Mul(u32, u32),
    Div(u32, u32),
    Pow(u32, u32),
    Call1(Fn1, u32),
    Call2(Fn2, u32, u32),
}

/// How many `f64` one instruction takes in the flat form.
pub const WORD: usize = 4;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tape {
    pub ops: Vec<Op>,
    /// The same instructions as plain numbers, which is how a tape reaches a kernel: it rides in
    /// the constraint's constants, where a block already carries per-constraint `f64`s.
    ///
    /// This — not `ops` — is what `eval` walks, so the evaluator a kernel runs and the evaluator
    /// a test checks are the same code.  A second walker over `ops` would be a second copy of the
    /// calculus, and the two would drift the first time one was edited.
    pub flat: Vec<f64>,
    /// How many variables the gradient is taken in.
    pub n_vars: usize,
}

/// The opcodes of the flat form.  `[code, a, b, c]` per instruction.
mod code {
    pub const CONST: f64 = 0.0;
    pub const VAR: f64 = 1.0;
    pub const NEG: f64 = 2.0;
    pub const ADD: f64 = 3.0;
    pub const SUB: f64 = 4.0;
    pub const MUL: f64 = 5.0;
    pub const DIV: f64 = 6.0;
    pub const POW: f64 = 7.0;
    pub const CALL1: f64 = 8.0;
    pub const CALL2: f64 = 9.0;
}

/// Scratch a tape runs in: one value and one gradient per slot.  Held by the caller and reused,
/// because a kernel evaluates the same tape for every constraint in its block and allocating per
/// evaluation is the one thing the compile-to-plan seam exists to avoid.
pub struct Scratch {
    v: Vec<f64>,
    g: Vec<f64>,
}

impl Scratch {
    pub fn new() -> Scratch {
        Scratch { v: Vec::new(), g: Vec::new() }
    }

    fn fit(&mut self, n_ops: usize, n_vars: usize) {
        self.v.resize(n_ops, 0.0);
        self.g.resize(n_ops * n_vars, 0.0);
    }
}

impl Default for Scratch {
    fn default() -> Scratch {
        Scratch::new()
    }
}

/// What one evaluation came to.
#[derive(Clone, Copy, Debug, Default)]
pub struct Value {
    pub v: f64,
    /// `d[i]` is `∂v/∂x[i]`, for the variables the tape was compiled over.
    pub d: [f64; MAX_VARS],
}

impl Tape {
    /// Compile an expression over a named variable table.
    ///
    /// A name the table does not hold and `expr` does not know as a constant is an error here
    /// rather than a free variable: a curve is written over geometry that exists, and a name
    /// nothing declares is a misspelling, not an unknown for the solver to answer.
    pub fn compile(ast: &Ast, vars: &[String]) -> Result<Tape, String> {
        if vars.len() > MAX_VARS {
            return Err(format!("a curve may read at most {MAX_VARS} coordinates"));
        }
        let index: BTreeMap<&str, u32> =
            vars.iter().enumerate().map(|(i, n)| (n.as_str(), i as u32)).collect();
        let mut t = Tape { ops: Vec::new(), flat: Vec::new(), n_vars: vars.len() };
        t.walk(ast, &index)?;
        t.flat = t.encode();
        Ok(t)
    }

    /// The instructions as plain numbers.
    fn encode(&self) -> Vec<f64> {
        let mut f = Vec::with_capacity(self.ops.len() * WORD);
        for op in &self.ops {
            let (c, a, b, d) = match *op {
                Op::Const(v) => (code::CONST, v, 0.0, 0.0),
                Op::Var(i) => (code::VAR, i as f64, 0.0, 0.0),
                Op::Neg(a) => (code::NEG, a as f64, 0.0, 0.0),
                Op::Add(a, b) => (code::ADD, a as f64, b as f64, 0.0),
                Op::Sub(a, b) => (code::SUB, a as f64, b as f64, 0.0),
                Op::Mul(a, b) => (code::MUL, a as f64, b as f64, 0.0),
                Op::Div(a, b) => (code::DIV, a as f64, b as f64, 0.0),
                Op::Pow(a, b) => (code::POW, a as f64, b as f64, 0.0),
                Op::Call1(f, a) => (code::CALL1, a as f64, fn1_code(f), 0.0),
                Op::Call2(f, a, b) => (code::CALL2, a as f64, b as f64, fn2_code(f)),
            };
            f.extend_from_slice(&[c, a, b, d]);
        }
        f
    }

    fn push(&mut self, op: Op) -> Result<u32, String> {
        if self.ops.len() >= MAX_OPS {
            return Err(format!("an expression may not be longer than {MAX_OPS} operations"));
        }
        self.ops.push(op);
        Ok(self.ops.len() as u32 - 1)
    }

    fn walk(&mut self, ast: &Ast, index: &BTreeMap<&str, u32>) -> Result<u32, String> {
        Ok(match ast {
            Ast::Num(v) => self.push(Op::Const(*v))?,
            Ast::Var(name) => match index.get(name.as_str()) {
                Some(&i) => self.push(Op::Var(i))?,
                None => match crate::expr::CONSTANTS.iter().find(|&&(n, _)| n == name) {
                    Some(&(_, v)) => self.push(Op::Const(v))?,
                    None => return Err(format!("`{name}` is not something this curve can read")),
                },
            },
            Ast::Neg(a) => {
                let x = self.walk(a, index)?;
                self.push(Op::Neg(x))?
            }
            Ast::Bin(op, a, b) => {
                let (x, y) = (self.walk(a, index)?, self.walk(b, index)?);
                self.push(match op {
                    EOp::Add => Op::Add(x, y),
                    EOp::Sub => Op::Sub(x, y),
                    EOp::Mul => Op::Mul(x, y),
                    EOp::Div => Op::Div(x, y),
                    EOp::Pow => Op::Pow(x, y),
                })?
            }
            Ast::Call(name, args) => {
                let a: Vec<u32> =
                    args.iter().map(|x| self.walk(x, index)).collect::<Result<_, _>>()?;
                match (fn1(name), fn2(name), a.len()) {
                    (Some(f), _, 1) => self.push(Op::Call1(f, a[0]))?,
                    (_, Some(f), 2) => self.push(Op::Call2(f, a[0], a[1]))?,
                    // `min`/`max` take any number; fold them pairwise
                    (_, Some(f), n) if n >= 1 && matches!(f, Fn2::Min | Fn2::Max) => {
                        let mut acc = a[0];
                        for &x in &a[1..] {
                            acc = self.push(Op::Call2(f, acc, x))?;
                        }
                        acc
                    }
                    _ => return Err(format!("`{name}` cannot be called with {} here", a.len())),
                }
            }
        })
    }

    /// Evaluate, with the gradient in every variable.
    pub fn eval(&self, x: &[f64], s: &mut Scratch) -> Value {
        eval_flat(&self.flat, self.n_vars, x, s)
    }
}

/// Run a tape in its flat form: the one evaluator, and the one place the calculus is written.
///
/// A kernel calls this with the slice of its constants the tape occupies; `Tape::eval` calls it
/// with its own.  Neither has a second walker to keep in step with the first.
pub fn eval_flat(flat: &[f64], n_vars: usize, x: &[f64], s: &mut Scratch) -> Value {
    let n_ops = flat.len() / WORD;
    let n = n_vars.min(MAX_VARS);
    s.fit(n_ops.max(1), n.max(1));
    let (v, g) = (&mut s.v, &mut s.g);
    for i in 0..n_ops {
        let w = &flat[i * WORD..i * WORD + WORD];
        let (a, b) = (w[1] as usize, w[2] as usize);
        let (val, grad): (f64, [f64; MAX_VARS]) = if w[0] == code::CONST {
            (w[1], [0.0; MAX_VARS])
        } else if w[0] == code::VAR {
            let mut d = [0.0; MAX_VARS];
            if a < n {
                d[a] = 1.0;
            }
            (x.get(a).copied().unwrap_or(0.0), d)
        } else if w[0] == code::NEG {
            let (av, ad) = get(v, g, n, a);
            (-av, map(&ad, n, |p| -p))
        } else if w[0] == code::ADD {
            let ((av, ad), (bv, bd)) = (get(v, g, n, a), get(v, g, n, b));
            (av + bv, zip(&ad, &bd, n, |p, q| p + q))
        } else if w[0] == code::SUB {
            let ((av, ad), (bv, bd)) = (get(v, g, n, a), get(v, g, n, b));
            (av - bv, zip(&ad, &bd, n, |p, q| p - q))
        } else if w[0] == code::MUL {
            let ((av, ad), (bv, bd)) = (get(v, g, n, a), get(v, g, n, b));
            (av * bv, zip(&ad, &bd, n, |p, q| p * bv + av * q))
        } else if w[0] == code::DIV {
            let ((av, ad), (bv, bd)) = (get(v, g, n, a), get(v, g, n, b));
            let inv = 1.0 / bv;
            (av * inv, zip(&ad, &bd, n, |p, q| (p * bv - av * q) * inv * inv))
        } else if w[0] == code::POW {
            let ((av, ad), (bv, bd)) = (get(v, g, n, a), get(v, g, n, b));
            let val = av.powf(bv);
            // d(a^b) = a^b (b' ln a + b a'/a); the second term is written a^(b-1) b a' so a
            // constant exponent over a zero base is finite rather than a logarithm of one
            let p1 = if av == 0.0 { 0.0 } else { av.powf(bv - 1.0) * bv };
            let l = if av > 0.0 { val * av.ln() } else { 0.0 };
            (val, zip(&ad, &bd, n, |p, q| p1 * p + l * q))
        } else if w[0] == code::CALL1 {
            let (av, ad) = get(v, g, n, a);
            let (val, d) = call1(fn1_of(w[2]), av);
            (val, map(&ad, n, |p| d * p))
        } else {
            let ((av, ad), (bv, bd)) = (get(v, g, n, a), get(v, g, n, b));
            let (val, da, db) = call2(fn2_of(w[3]), av, bv);
            (val, zip(&ad, &bd, n, |p, q| da * p + db * q))
        };
        v[i] = val;
        g[i * n..i * n + n].copy_from_slice(&grad[..n]);
    }
    match n_ops {
        0 => Value::default(),
        k => {
            let mut d = [0.0; MAX_VARS];
            d[..n].copy_from_slice(&g[(k - 1) * n..(k - 1) * n + n]);
            Value { v: v[k - 1], d }
        }
    }
}

fn get(v: &[f64], g: &[f64], n: usize, i: usize) -> (f64, [f64; MAX_VARS]) {
    let mut d = [0.0; MAX_VARS];
    d[..n].copy_from_slice(&g[i * n..i * n + n]);
    (v[i], d)
}

fn map(a: &[f64; MAX_VARS], n: usize, f: impl Fn(f64) -> f64) -> [f64; MAX_VARS] {
    let mut o = [0.0; MAX_VARS];
    for i in 0..n {
        o[i] = f(a[i]);
    }
    o
}

fn zip(
    a: &[f64; MAX_VARS],
    b: &[f64; MAX_VARS],
    n: usize,
    f: impl Fn(f64, f64) -> f64,
) -> [f64; MAX_VARS] {
    let mut o = [0.0; MAX_VARS];
    for i in 0..n {
        o[i] = f(a[i], b[i]);
    }
    o
}

/// The value and the derivative of a one-argument function.
///
/// The trigonometry is in degrees because `expr.rs`'s is, so every derivative through it carries
/// the `π/180` that convention costs — and every inverse carries `180/π` back.  Getting this
/// wrong is silent: the curve draws correctly and the solver takes the wrong step.  It is checked
/// against a finite difference of this same evaluator in `tests/tape.rs`.
fn call1(f: Fn1, a: f64) -> (f64, f64) {
    const K: f64 = std::f64::consts::PI / 180.0;
    match f {
        Fn1::Sqrt => {
            let s = a.sqrt();
            (s, if s == 0.0 { 0.0 } else { 0.5 / s })
        }
        Fn1::Abs => (a.abs(), a.signum()),
        Fn1::Sin => ((a * K).sin(), (a * K).cos() * K),
        Fn1::Cos => ((a * K).cos(), -(a * K).sin() * K),
        Fn1::Tan => {
            let c = (a * K).cos();
            ((a * K).tan(), K / (c * c))
        }
        Fn1::Asin => (a.asin().to_degrees(), 1.0 / (1.0 - a * a).sqrt() / K),
        Fn1::Acos => (a.acos().to_degrees(), -1.0 / (1.0 - a * a).sqrt() / K),
        Fn1::Atan => (a.atan().to_degrees(), 1.0 / (1.0 + a * a) / K),
        Fn1::Exp => (a.exp(), a.exp()),
        Fn1::Ln => (a.ln(), 1.0 / a),
        Fn1::Log => (a.log10(), 1.0 / (a * std::f64::consts::LN_10)),
        // a step has no slope anywhere it is defined, and no derivative where it is not
        Fn1::Floor => (a.floor(), 0.0),
        Fn1::Ceil => (a.ceil(), 0.0),
        Fn1::Round => (a.round(), 0.0),
    }
}

fn call2(f: Fn2, a: f64, b: f64) -> (f64, f64, f64) {
    const K: f64 = std::f64::consts::PI / 180.0;
    match f {
        Fn2::Atan2 => {
            let r2 = a * a + b * b;
            if r2 == 0.0 {
                (0.0, 0.0, 0.0)
            } else {
                (a.atan2(b).to_degrees(), b / r2 / K, -a / r2 / K)
            }
        }
        Fn2::Min => {
            if a <= b {
                (a, 1.0, 0.0)
            } else {
                (b, 0.0, 1.0)
            }
        }
        Fn2::Max => {
            if a >= b {
                (a, 1.0, 0.0)
            } else {
                (b, 0.0, 1.0)
            }
        }
        Fn2::Hypot => {
            let h = a.hypot(b);
            if h == 0.0 {
                (0.0, 0.0, 0.0)
            } else {
                (h, a / h, b / h)
            }
        }
    }
}

const FN1S: [Fn1; 14] = [
    Fn1::Sqrt, Fn1::Abs, Fn1::Sin, Fn1::Cos, Fn1::Tan, Fn1::Asin, Fn1::Acos, Fn1::Atan,
    Fn1::Exp, Fn1::Ln, Fn1::Log, Fn1::Floor, Fn1::Ceil, Fn1::Round,
];
const FN2S: [Fn2; 4] = [Fn2::Atan2, Fn2::Min, Fn2::Max, Fn2::Hypot];

fn fn1_code(f: Fn1) -> f64 {
    FN1S.iter().position(|&x| x == f).unwrap_or(0) as f64
}
fn fn2_code(f: Fn2) -> f64 {
    FN2S.iter().position(|&x| x == f).unwrap_or(0) as f64
}
fn fn1_of(c: f64) -> Fn1 {
    FN1S[(c as usize).min(FN1S.len() - 1)]
}
fn fn2_of(c: f64) -> Fn2 {
    FN2S[(c as usize).min(FN2S.len() - 1)]
}

fn fn1(name: &str) -> Option<Fn1> {
    Some(match name {
        "sqrt" => Fn1::Sqrt,
        "abs" => Fn1::Abs,
        "sin" => Fn1::Sin,
        "cos" => Fn1::Cos,
        "tan" => Fn1::Tan,
        "asin" => Fn1::Asin,
        "acos" => Fn1::Acos,
        "atan" => Fn1::Atan,
        "exp" => Fn1::Exp,
        "ln" => Fn1::Ln,
        "log" => Fn1::Log,
        "floor" => Fn1::Floor,
        "ceil" => Fn1::Ceil,
        "round" => Fn1::Round,
        _ => return None,
    })
}

fn fn2(name: &str) -> Option<Fn2> {
    Some(match name {
        "atan2" => Fn2::Atan2,
        "min" => Fn2::Min,
        "max" => Fn2::Max,
        "hypot" => Fn2::Hypot,
        _ => return None,
    })
}
