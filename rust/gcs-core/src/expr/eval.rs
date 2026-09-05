//! Evaluate typed arithmetic and affine expressions.

use super::{Ast, Op, CONSTANTS};
use crate::constraints::SpecKind;
use crate::units::Dim;
use std::collections::BTreeMap;

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
        Aff { free: Some(name.to_string()), m: 1.0, c: 0.0, dim: Dim::SCALAR }
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
pub fn eval(ast: &Ast, env: &BTreeMap<String, Aff>) -> Result<Aff, String> {
    Ok(match ast {
        Ast::Num(v, d) => Aff::of_dim(*v, *d),
        Ast::Var(name) => match CONSTANTS.iter().find(|&&(n, _, _)| n == name) {
            Some(&(_, v, d)) => Aff::of_dim(v, d),
            None => match env.get(name) {
                Some(a) => a.clone(),
                None => Aff::of(name),
            },
        },
        Ast::Neg(a) => {
            let x = eval(a, env)?;
            Aff { free: x.free, m: -x.m, c: -x.c, dim: x.dim }
        }
        Ast::Bin(op, a, b) => {
            let (x, y) = (eval(a, env)?, eval(b, env)?);
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
                        _ => return Err(free_pair_of(&x, &y)),
                    }
                }
                Op::Div => {
                    let dim = x.dim.div(y.dim);
                    match y.number() {
                        Some(k) => Aff { free: x.free, m: x.m / k, c: x.c / k, dim },
                        None => return Err(not_affine(y.free.as_deref().unwrap_or(""))),
                    }
                }
                Op::Pow => match (x.number(), y.number()) {
                    (Some(p), Some(q)) => {
                        if !y.dim.is_scalar() {
                            return Err(format!(
                                "a power is a plain number, and this one is {}",
                                y.dim.name()
                            ));
                        }
                        let dim = x.dim.powf(q).ok_or_else(|| {
                            format!(
                                "{} to the power {q} is not a dimension — a dimensioned base \
                                 takes a whole power",
                                x.dim.name()
                            )
                        })?;
                        Aff::of_dim(p.powf(q), dim)
                    }
                    _ => return Err(free_pair_of(&x, &y)),
                },
            }
        }
        Ast::Call(name, args) => {
            let mut vals = Vec::with_capacity(args.len());
            let mut dims = Vec::with_capacity(args.len());
            for a in args {
                let v = eval(a, env)?;
                dims.push(v.dim);
                match v.number() {
                    Some(n) => vals.push(n),
                    None => return Err(not_affine(v.free.as_deref().unwrap_or(""))),
                }
            }
            Aff::of_dim(call(name, &vals), signature(name, &dims)?)
        }
    })
}

/// What `a + b` is, when a bare number takes the other's dimension and anything else must agree.
fn sum(x: &Aff, y: &Aff, op: &str) -> Result<Dim, String> {
    x.dim.agree(y.dim).ok_or_else(|| {
        format!(
            "`{}` and `{}` cannot be added: {op} needs one dimension, not two",
            x.dim.name(),
            y.dim.name()
        )
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
