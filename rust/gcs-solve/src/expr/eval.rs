//! Evaluate typed arithmetic and affine expressions.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::{Ast, Cmp, Cond, ExprError, Measure, Op, CONSTANTS};
use crate::constraints::SpecKind;
use crate::units::Dim;
use std::collections::BTreeMap;

fn call(name: &str, a: &[f64]) -> f64 {
    match name {
        "sqrt" => a[0].sqrt(),
        "abs" => a[0].abs(),
        "sin" => a[0].to_radians().dsin(),
        "cos" => a[0].to_radians().dcos(),
        "tan" => a[0].to_radians().dtan(),
        "asin" => a[0].dasin().to_degrees(),
        "acos" => a[0].dacos().to_degrees(),
        "atan" => a[0].datan().to_degrees(),
        "atan2" => a[0].datan2(a[1]).to_degrees(),
        "exp" => a[0].dexp(),
        "ln" => a[0].dln(),
        "log" => a[0].dlog10(),
        "floor" => a[0].floor(),
        "ceil" => a[0].ceil(),
        "round" => a[0].round(),
        "min" => a.iter().copied().fold(f64::INFINITY, f64::min),
        "max" => a.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        "hypot" => a[0].dhypot(a[1]),
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
    /// What it *is* — see `units.rs`.  Carried beside the value rather than worked out again by
    /// a second walk: the dimension of `a * b` is a fact about the same two operands the number
    /// came from, and two walks would be two answers the moment one of them was edited.
    pub dim: Dim,
}

impl Aff {
    pub fn num(v: f64) -> Aff {
        Aff { free: None, m: 0.0, c: v, dim: Dim::SCALAR }
    }

    /// A number that says what it is: a literal with a unit on it, or a value whose dimension
    /// something else declared.
    pub fn of_dim(v: f64, dim: Dim) -> Aff {
        Aff { free: None, m: 0.0, c: v, dim }
    }

    fn of(name: &str) -> Aff {
        Aff::free(name, Dim::SCALAR)
    }

    /// A free name of this dimension: the unknown a declared number stands for.
    pub fn free(name: impl Into<String>, dim: Dim) -> Aff {
        Aff { free: Some(name.into()), m: 1.0, c: 0.0, dim }
    }

    /// This and `k` times `o` (`-1` subtracts), one affine form in this one's dimension, or why
    /// not: two free names.  Whether the two agree is the caller's to ask.
    pub fn plus(&self, o: &Aff, k: f64) -> Result<Aff, String> {
        Ok(Aff { free: together(self, o)?, m: self.m + k * o.m, c: self.c + k * o.c, dim: self.dim })
    }

    /// This times a plain number.
    pub fn times(&self, k: f64) -> Aff {
        Aff { m: self.m * k, c: self.c * k, ..self.clone() }
    }

    /// The same value, said to be of this dimension.  What a declared formal and a named slot
    /// do to a number that arrived bare.
    pub fn as_dim(mut self, dim: Dim) -> Aff {
        self.dim = dim;
        self
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
///
/// A measurement is refused here: nothing has been solved for it to read (`measure_refusal`).
pub fn eval(ast: &Ast, env: &BTreeMap<String, Aff>) -> Result<Aff, ExprError> {
    ev(ast, env, None)
}

/// What an expression's names are read from: the value a name has, or `None` where it is free.
/// A map is one; the elaborator's scopes are another, a table of frames read outward (#405), and
/// a value is handed out rather than lent so that a reader may work one out as it is asked.
pub trait Env {
    fn value(&self, name: &str) -> Option<Aff>;
}

impl Env for BTreeMap<String, Aff> {
    fn value(&self, name: &str) -> Option<Aff> {
        self.get(name).cloned()
    }
}

/// `eval` over any `Env`.
pub fn eval_in(ast: &Ast, env: &dyn Env) -> Result<Aff, ExprError> {
    ev(ast, env, None)
}

/// Whether a condition holds (#210), its names worked out from `env`.  Each comparison is
/// between two plain numbers of one dimension, as `+` is between two operands (`Dim::agree`):
/// `d > 0mm`, never `d > 0`.  Every comparison is worked out, under `and` and `or` too, so a
/// mistake on the side a short circuit would skip is still said.  A name `env` has no number
/// for is refused here; the caller, which knows an unknown of the drawing from a name nothing
/// declares, asks first.
pub fn decide(c: &Cond, env: &BTreeMap<String, Aff>) -> Result<bool, ExprError> {
    decide_in(c, env)
}

/// `decide` over any `Env`.
pub fn decide_in(c: &Cond, env: &dyn Env) -> Result<bool, ExprError> {
    decide_by(c, env, Cmp::holds)
}

/// `decide_in` as a claim over values is judged (#596): each comparison kept within a hair of
/// rounding (`Cmp::keeps`), never exactly.
pub fn judge_in(c: &Cond, env: &dyn Env) -> Result<bool, ExprError> {
    decide_by(c, env, Cmp::keeps)
}

/// `decide_in`, each comparison of two numbers decided by `holds`.
fn decide_by(c: &Cond, env: &dyn Env, holds: fn(Cmp, f64, f64) -> bool)
    -> Result<bool, ExprError>
{
    Ok(match c {
        Cond::Cmp(op, a, b) => {
            let (x, y) = (ev(a, env, None)?, ev(b, env, None)?);
            x.dim.agree(y.dim).ok_or_else(|| {
                ExprError::dimension(format!(
                    "{} cannot be compared with {}: `{}` needs one dimension, not two",
                    x.dim.name(),
                    y.dim.name(),
                    op.text()
                ))
            })?;
            let number = |v: &Aff| match v.number() {
                Some(n) if n.is_finite() => Ok(n),
                Some(n) => Err(ExprError::from(format!("a side of `{}` comes to {n}", op.text()))),
                None => Err(ExprError::from(format!(
                    "`{}` is not a number here",
                    v.free.as_deref().unwrap_or("")
                ))),
            };
            holds(*op, number(&x)?, number(&y)?)
        }
        Cond::And(a, b) => {
            let (p, q) = (decide_by(a, env, holds)?, decide_by(b, env, holds)?);
            p && q
        }
        Cond::Or(a, b) => {
            let (p, q) = (decide_by(a, env, holds)?, decide_by(b, env, holds)?);
            p || q
        }
        Cond::Not(a) => !decide_by(a, env, holds)?,
        Cond::In(a, b) => {
            return Err(ExprError::from(format!(
                "`{a} in {b}` asks what `{a}` stands for, which is no number"
            )))
        }
    })
}

/// What reads a measurement: its kind and the names it is over, to what it comes to.
pub type Measurer<'a> = &'a dyn Fn(Measure, &[String]) -> Result<Aff, String>;

/// `eval` in a context read **after** the solve, where `measure` says what each measurement
/// comes to: a motion's numbers, worked out from the geometry the solve left.
pub fn eval_measured(
    ast: &Ast,
    env: &BTreeMap<String, Aff>,
    measure: Measurer,
) -> Result<Aff, ExprError> {
    ev(ast, env, Some(measure))
}

fn ev(
    ast: &Ast,
    env: &dyn Env,
    ms: Option<Measurer>,
) -> Result<Aff, ExprError> {
    Ok(match ast {
        Ast::Num(v, d) => Aff::of_dim(*v, *d),
        Ast::Var(name) => match CONSTANTS.iter().find(|&&(n, _, _)| n == name) {
            Some(&(_, v, d)) => Aff::of_dim(v, d),
            None => match env.value(name) {
                Some(a) => a,
                None => Aff::of(name),
            },
        },
        Ast::Neg(a) => {
            let x = ev(a, env, ms)?;
            Aff { free: x.free, m: -x.m, c: -x.c, dim: x.dim }
        }
        Ast::Bin(op, a, b) => {
            let (x, y) = (ev(a, env, ms)?, ev(b, env, ms)?);
            match op {
                // `+` and `-` demand agreement, where a *bare number* takes the other's
                // dimension: `90 / N + ivp` is an angle because `ivp` is, and `w + phi` is an
                // error because both said what they were and disagreed
                Op::Add => Aff {
                    free: together(&x, &y)?,
                    m: x.m + y.m,
                    c: x.c + y.c,
                    dim: sum(&x, &y, "+")?,
                },
                Op::Sub => Aff {
                    free: together(&x, &y)?,
                    m: x.m - y.m,
                    c: x.c - y.c,
                    dim: sum(&x, &y, "-")?,
                },
                // `*` and `/` derive: the exponents add and subtract, and nothing can disagree
                Op::Mul => {
                    let dim = x.dim.mul(y.dim);
                    match (x.number(), y.number()) {
                        (Some(k), _) => Aff { free: y.free, m: k * y.m, c: k * y.c, dim },
                        (_, Some(k)) => Aff { free: x.free, m: k * x.m, c: k * x.c, dim },
                        _ => return Err(free_pair_of(&x, &y).into()),
                    }
                }
                Op::Div => {
                    let dim = x.dim.div(y.dim);
                    match y.number() {
                        Some(k) => Aff { free: x.free, m: x.m / k, c: x.c / k, dim },
                        None => return Err(not_affine(y.free.as_deref().unwrap_or("")).into()),
                    }
                }
                Op::Pow => match (x.number(), y.number()) {
                    (Some(p), Some(q)) => {
                        if !y.dim.is_scalar() {
                            return Err(ExprError::dimension(format!(
                                "a power is a plain number, and this one is {}",
                                y.dim.name()
                            )));
                        }
                        let dim = x.dim.powf(q).ok_or_else(|| {
                            ExprError::dimension(format!(
                                "{} to the power {q} is not a dimension — a dimensioned base \
                                 takes a whole power",
                                x.dim.name()
                            ))
                        })?;
                        Aff::of_dim(p.dpowf(q), dim)
                    }
                    _ => return Err(free_pair_of(&x, &y).into()),
                },
            }
        }
        Ast::Call(name, args) => {
            let mut vals = Vec::with_capacity(args.len());
            let mut dims = Vec::with_capacity(args.len());
            for a in args {
                let v = ev(a, env, ms)?;
                dims.push(v.dim);
                match v.number() {
                    Some(n) => vals.push(n),
                    None => return Err(not_affine(v.free.as_deref().unwrap_or("")).into()),
                }
            }
            Aff::of_dim(call(name, &vals), signature(name, &dims).map_err(ExprError::dimension)?)
        }
        Ast::Measure(m, args) => match ms {
            Some(f) => f(*m, args)?,
            None => return Err(ExprError::measure(&m.text(args))),
        },
    })
}

/// What `a + b` is, when a bare number takes the other's dimension and anything else must agree.
fn sum(x: &Aff, y: &Aff, op: &str) -> Result<Dim, ExprError> {
    x.dim.agree(y.dim).ok_or_else(|| {
        ExprError::dimension(format!(
            "`{}` and `{}` cannot be added: {op} needs one dimension, not two",
            x.dim.name(),
            y.dim.name()
        ))
    })
}

/// A function's dimensions: what it takes and what it gives back (spec §3.3).
///
/// `floor`, `ceil` and `round` are Scalar-only deliberately: rounding a dimensioned quantity
/// depends on which unit you round in, and a language that silently picked one would be wrong
/// half the time.
fn signature(name: &str, a: &[Dim]) -> Result<Dim, String> {
    let scalar_in = || -> Result<Dim, String> {
        match a.iter().find(|d| !d.is_scalar()) {
            Some(d) => Err(format!("`{name}` takes a plain number, and this one is {}", d.name())),
            None => Ok(Dim::SCALAR),
        }
    };
    // from the *first* argument, not from `Scalar`: `max(a, b)` is whatever a and b are, and
    // starting at Scalar would make every dimensioned call disagree with nothing
    let agreeing = || -> Result<Dim, String> {
        let mut d = *a.first().unwrap_or(&Dim::SCALAR);
        for x in &a[1..] {
            d = d.agree(*x).ok_or_else(|| {
                format!(
                    "`{name}` needs its arguments in one dimension, and {} is not {}",
                    x.name(),
                    d.name()
                )
            })?;
        }
        Ok(d)
    };
    Ok(match name {
        "sin" | "cos" | "tan" => {
            if !a[0].fits(Dim::ANGLE) {
                return Err(format!("`{name}` takes an angle, and this is {}", a[0].name()));
            }
            Dim::SCALAR
        }
        "asin" | "acos" | "atan" => {
            scalar_in()?;
            Dim::ANGLE
        }
        "atan2" => {
            agreeing()?;
            Dim::ANGLE
        }
        "sqrt" => a[0].sqrt(),
        "abs" | "min" | "max" | "hypot" => agreeing()?,
        "exp" | "ln" | "log" | "floor" | "ceil" | "round" => scalar_in()?,
        // every name in `FUNCTIONS` is stated above, and the parser admits no other — so this
        // arm is reached only by a function added to that list and not to this one, which would
        // otherwise be dimensionless and take anything in silence
        other => {
            debug_assert!(false, "`{other}` is in FUNCTIONS with no dimension signature");
            Dim::SCALAR
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

/// Significant digits a stored number is read to — on a callout, in a list, in a report.
///
/// **One constant, read through `reading`**, so `dimension_text` and `describe` cannot print
/// one number two ways: a bare `Arg::Num` that a `param` substitution left at
/// 49.99999999999999 is `50` on the drawing and `50` in the list (#43.15), where the list
/// used to print the float and the callout four digits of it.  Six, because that is where a
/// double's noise ends and a drawing's own numbers do not — `1234.5` keeps its half, which at
/// four it did not.
pub const READING_SIG: usize = 6;

/// A stored number as a reader reads it: in the units they read (degrees for an angle), to
/// `READING_SIG`.  Every place a constraint's number is turned into text for a person goes
/// through here — the callout, the constraint list, the CLI's culprits.
pub fn reading(kind: SpecKind, v: f64) -> String {
    crate::json::fmt_g(to_user_units(kind, v), READING_SIG)
}
