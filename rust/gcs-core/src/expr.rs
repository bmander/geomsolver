//! Dimension expression types, built-in names, and public entry points.

mod document;
mod eval;
mod names;
mod parser;

pub use document::{evaluate, has_expr, set_dimension, sync_free, ExprError, ExprItem, Fault};
pub(crate) use document::{bind_free, eval_declared, with_declared};
pub use eval::{decide, eval, eval_measured, to_arg_units, to_user_units, Aff, Measurer};
pub use names::{measure_call, name_at, NameScan, LEXER};
pub use parser::{is_literal, literal, names_unit, notation, parse, parse_condition_in, parse_in};

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

/// **A measurement of the solved drawing**: `length(l)`, `radius(c)`, `distance(a, b)`,
/// `angle(l1, l2)`.  Its arguments are *names of geometry*, never numbers, and what it comes to
/// is a fact about where the solve put that geometry, so it may stand only in a statement read
/// **after** the solve: a motion's `ratio:`, `phase:` and `advance:`.  Everywhere else (a
/// `param`, a seed, a constraint's number, a solid's extent) the number is needed before there
/// is a solve to read, and an expression holding one is refused with E107.  The strata run one
/// way: the sketch solves, then what reads the solve is worked out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Measure {
    /// A line's length, or an arc's, in space.
    Length,
    /// A circle's or an arc's radius.
    Radius,
    /// Between two points, or from a point to a line (the line produced), in space.
    Distance,
    /// Between two lines' directions (`p1 → p2`), in space: 0° to 180°.
    Angle,
}

/// (name, what it measures, arity): the one table the parser, the substitution guard and the
/// refusals read.  A call is a measurement exactly when its name is here, so a `param` named
/// `length` still reads as the param wherever it is not followed by `(`.
pub const MEASURES: &[(&str, Measure, usize)] = &[
    ("length", Measure::Length, 1),
    ("radius", Measure::Radius, 1),
    ("distance", Measure::Distance, 2),
    ("angle", Measure::Angle, 2),
];

impl Measure {
    pub fn name(self) -> &'static str {
        MEASURES.iter().find(|m| m.1 == self).map_or("?", |m| m.0)
    }

    /// `length(gen_g)`, as the call is written.
    pub fn text(self, args: &[String]) -> String {
        format!("{}({})", self.name(), args.join(", "))
    }
}

/// Why a measurement is refused where it stands: the one sentence every refusal says, and the
/// words (`MEASURE_MARK`) the elaborator gives E107 by, whichever path carried the message.
pub fn measure_refusal(call: &str) -> String {
    format!(
        "`{call}` {MEASURE_MARK}, and a number is needed before the solve: a param, a seed, a \
         constraint's number, a solid's extent and a frame's ratio or advance are stated from \
         the document's own numbers"
    )
}

/// The words every measurement refusal carries.
pub const MEASURE_MARK: &str = "measures the solved drawing";

/// The first measurement a text makes, as written, if it parses and makes one: the question
/// each refusal asks.
pub fn measurement_in(text: &str, units: crate::units::Units) -> Option<String> {
    let p = parse_in(text, units).ok()?;
    p.body.measures().first().map(|(m, args)| m.text(args))
}

/// Every name a text's measurements read, as written — what a deletion asks of a statement.
/// Read in millimetres, which admits every unit suffix: the names do not depend on the unit.
pub fn measured_names(text: &str) -> Vec<String> {
    let mm = crate::units::Units::with_length("mm").expect("mm is a unit");
    parse_in(text, mm).map_or(Vec::new(), |p| {
        p.body.measures().into_iter().flat_map(|(_, args)| args).collect()
    })
}

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
    /// A measurement of the solved drawing, over the names of the geometry it reads.
    Measure(Measure, Vec<String>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Pow,
}

/// How two numbers of one dimension compare (#210): `<`, `<=`, `>`, `>=`, `==`, `!=`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

impl Cmp {
    /// The comparison as written.
    pub fn text(self) -> &'static str {
        match self {
            Cmp::Lt => "<",
            Cmp::Le => "<=",
            Cmp::Gt => ">",
            Cmp::Ge => ">=",
            Cmp::Eq => "==",
            Cmp::Ne => "!=",
        }
    }

    /// Whether `a` stands so to `b`: exactly, as the numbers are — no tolerance, so `<` and
    /// `==` are never both true.
    pub fn holds(self, a: f64, b: f64) -> bool {
        match self {
            Cmp::Lt => a < b,
            Cmp::Le => a <= b,
            Cmp::Gt => a > b,
            Cmp::Ge => a >= b,
            Cmp::Eq => a == b,
            Cmp::Ne => a != b,
        }
    }
}

/// **A condition** (#210): comparisons of numbers joined by `and`, `or` and `not` — what `when
/// COND { … }` decides a block by.  Not an `Ast`: a truth is no number, so no arithmetic can read
/// one and no number slot can hold one; only `parse_condition_in` makes one.
///
/// Two names may also be compared for **identity** (#267): `a == b` and `a != b` where a side
/// names geometry rather than a number, and `c in B`, a name among the faces another names.
/// What a name stands for is the elaborator's to say, so `decide` refuses those, and
/// `flatten::values::condition` decides them.
#[derive(Clone, Debug, PartialEq)]
pub enum Cond {
    Cmp(Cmp, Ast, Ast),
    /// `c in B`: the name on the left among what the name on the right stands for.
    In(String, String),
    And(Box<Cond>, Box<Cond>),
    Or(Box<Cond>, Box<Cond>),
    Not(Box<Cond>),
}

impl Cond {
    /// Every name the condition reads, once each, in name order.
    pub fn deps(&self) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        self.collect_deps(&mut out);
        out
    }

    fn collect_deps(&self, out: &mut BTreeSet<String>) {
        match self {
            Cond::Cmp(_, a, b) => {
                a.collect_deps(out);
                b.collect_deps(out);
            }
            Cond::And(a, b) | Cond::Or(a, b) => {
                a.collect_deps(out);
                b.collect_deps(out);
            }
            Cond::Not(a) => a.collect_deps(out),
            Cond::In(a, b) => {
                out.insert(a.clone());
                out.insert(b.clone());
            }
        }
    }
}

/// A parsed expression.
#[derive(Clone, Debug, PartialEq)]
pub struct Parsed {
    pub body: Ast,
}

impl Ast {
    /// The expression with each name `f` answers written as what it answers, every other part as
    /// it is.
    pub fn map_vars(&self, f: &impl Fn(&str) -> Option<Ast>) -> Ast {
        match self {
            Ast::Var(v) => f(v).unwrap_or_else(|| self.clone()),
            Ast::Neg(a) => Ast::Neg(Box::new(a.map_vars(f))),
            Ast::Bin(o, a, b) => Ast::Bin(*o, Box::new(a.map_vars(f)), Box::new(b.map_vars(f))),
            Ast::Call(n, args) => {
                Ast::Call(n.clone(), args.iter().map(|a| a.map_vars(f)).collect())
            }
            Ast::Num(..) | Ast::Measure(..) => self.clone(),
        }
    }

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
            // a measurement reads geometry, not a number: its names are not dependencies
            Ast::Measure(..) => {}
        }
    }

    /// Every measurement the expression makes, in the order it is written.
    pub fn measures(&self) -> Vec<(Measure, Vec<String>)> {
        let mut out = Vec::new();
        self.collect_measures(&mut out);
        out
    }

    fn collect_measures(&self, out: &mut Vec<(Measure, Vec<String>)>) {
        match self {
            Ast::Num(..) | Ast::Var(_) => {}
            Ast::Neg(a) => a.collect_measures(out),
            Ast::Bin(_, a, b) => {
                a.collect_measures(out);
                b.collect_measures(out);
            }
            Ast::Call(_, args) => {
                for a in args {
                    a.collect_measures(out);
                }
            }
            Ast::Measure(m, args) => out.push((*m, args.clone())),
        }
    }
}
