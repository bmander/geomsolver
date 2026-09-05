//! Dimension expression types, built-in names, and public entry points.

mod document;
mod eval;
mod parser;

pub use document::{evaluate, has_expr, set_dimension, sync_free, ExprError, ExprItem, Fault};
pub use eval::{eval, to_arg_units, to_user_units, Aff};
pub use parser::{literal, name_of, names_unit, notation, parse, parse_in};

use crate::units::Dim;
use std::collections::BTreeSet;

/// A dimension argument written as text, with the number it last evaluated to in the argument's
/// own units (radians for an angle), so `Arg::num` and the kernels never see the text.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    pub text: String,
    pub value: f64,
}

/// An affine dimension `value = m * variable + c`. The variable is a solver
/// parameter; `m` and `c` use argument units (radians for angles).
/// Derived by `evaluate` and stored on the constraint; rebuild after copying.
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

/// Built-in constants with dimensions: `pi` is scalar; `tau` and `turn` are angles.
pub const CONSTANTS: &[(&str, f64, Dim)] = &[
    ("pi", std::f64::consts::PI, Dim::SCALAR),
    ("tau", 360.0, Dim::ANGLE),
    ("turn", 360.0, Dim::ANGLE),
];

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

/// What a name is built in *as*, for the two callers that have to say so — the refusal here and
/// the shadowing warning in `program` — or `None` where the document is free to use it.  One
/// table, since "is this built in" and "what is it" are the same question asked twice.
pub fn builtin(name: &str) -> Option<&'static str> {
    if CONSTANTS.iter().any(|&(n, _, _)| n == name) {
        Some("a built-in constant")
    } else if FUNCTIONS.iter().any(|&(n, _, _)| n == name) {
        Some("a built-in function")
    } else {
        None
    }
}

fn is_builtin(name: &str) -> bool {
    builtin(name).is_some()
}

#[derive(Clone, Debug, PartialEq)]
pub enum Ast {
    /// A literal, and what it *is*.  A bare number is `Dim::SCALAR` and takes the dimension of
    /// wherever it stands; a suffixed one — `45deg`, `1' 6 3/16"` — says so itself, and the
    /// tokenizer has already converted it into the document's own units.
    Num(f64, Dim),
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
            Ast::Num(..) => {}
            Ast::Var(v) => {
                if !CONSTANTS.iter().any(|&(n, _, _)| n == v) {
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
