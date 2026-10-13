//! **A bound on measurements** (§6.21, #183 F3, #190): `distance(p, f1) + distance(p, f2) <=
//! 10mm`, each side a tree of measurements and numbers as written.

use super::{names, Cmp, Name, Ref, Span};

/// `distance(p, f1) + distance(p, f2) <= 10mm` — a bound on an expression of measurements
/// (§6.21, #183 F3): in a set's body, where the point the set is about may be, as the distances
/// from it say.  **Parsed once** (#190): each side is a `Measuring` tree whose measurements name
/// what they measure by `Ref`s, rewritten by the flattener as every other operand is, and whose
/// numbers are texts it settles over the scope.  Closed, as every bound is.
#[derive(Clone, Debug)]
pub struct Measured {
    pub lhs: Measuring,
    pub cmp: Cmp,
    pub rhs: Measuring,
    pub span: Span,
    /// The sets it was applied through, outermost first (#212, #214): `Worded::through` for a
    /// bound on measurements, so the faces a region's term makes are named by the set its body
    /// used (`inner_round` for `p inside inner_round`).  Empty as written.
    pub through: Vec<String>,
}

/// One side of a bound on measurements, as written: arithmetic over measurements and numbers.
/// Whatever measures nothing is one `Number`, however much arithmetic it is (`2 * r`, `(a +
/// b)`), so it is read by `expr` as any dimension's text is.
#[derive(Clone, Debug)]
pub enum Measuring {
    /// A number as written (`10mm`, `0.5 * w`, `(a + b)`): the flattener writes the scope's
    /// numbers in (`flatten::values::substitute`), `expr` reads it.
    Number { text: String, span: Span },
    /// `distance(p, f1)`, `angle(a, b, at: p)`.
    Measure(Measurement),
    Neg(Box<Measuring>),
    Bin(crate::expr::Op, Box<Measuring>, Box<Measuring>),
    /// A function of a measurement (`sqrt(distance(p, q))`), which no region reads: kept so the
    /// elaborator can say so where it stands.
    Call(Name, Vec<Measuring>),
}

/// A measurement in a bound: what it measures (`crate::expr::Measure`) and the geometry it
/// names, by reference — never by text, so a hidden key the flattener writes in (`blank#probe`,
/// `#12.m0`) is a reference's name and never something a source could spell.
#[derive(Clone, Debug)]
pub struct Measurement {
    pub measure: crate::expr::Measure,
    pub args: Vec<Ref>,
    /// The point an angle between faces is read nearest (#183 F9): `angle(a, b, at: p)`.
    pub at: Option<Ref>,
    pub span: Span,
}

impl Measurement {
    /// As it is written: `distance(p, f1)`, `angle(a, b, at: p)`.
    pub fn text(&self) -> String {
        let mut args: Vec<String> = self.args.iter().map(names::ref_text).collect();
        if let Some(at) = &self.at {
            args.push(format!("at: {}", names::ref_text(at)));
        }
        format!("{}({})", self.measure.name(), args.join(", "))
    }

    /// Every reference it reads, the `at:` point last.
    pub fn refs(&self) -> impl Iterator<Item = &Ref> {
        self.args.iter().chain(self.at.iter())
    }

    pub fn refs_mut(&mut self) -> impl Iterator<Item = &mut Ref> {
        self.args.iter_mut().chain(self.at.iter_mut())
    }
}

impl Measuring {
    /// Every measurement, in the order written.
    pub fn measures(&self) -> Vec<&Measurement> {
        let mut out = Vec::new();
        self.walk(&mut |m| {
            if let Measuring::Measure(x) = m {
                out.push(x);
            }
        });
        out
    }

    fn walk<'a>(&'a self, f: &mut impl FnMut(&'a Measuring)) {
        f(self);
        match self {
            Measuring::Number { .. } | Measuring::Measure(_) => {}
            Measuring::Neg(a) => a.walk(f),
            Measuring::Bin(_, a, b) => {
                a.walk(f);
                b.walk(f);
            }
            Measuring::Call(_, args) => args.iter().for_each(|a| a.walk(f)),
        }
    }

    /// Every measurement, mutably: what the flattener rewrites the references of.
    pub fn measures_mut(&mut self) -> Vec<&mut Measurement> {
        let mut out = Vec::new();
        self.collect_mut(&mut |m| {
            if let Measuring::Measure(x) = m {
                out.push(x);
            }
        });
        out
    }

    /// Every number's text, mutably: what the flattener writes the scope's numbers into.
    pub fn numbers_mut(&mut self) -> Vec<&mut String> {
        let mut out = Vec::new();
        self.collect_mut(&mut |m| {
            if let Measuring::Number { text, .. } = m {
                out.push(text);
            }
        });
        out
    }

    fn collect_mut<'a>(&'a mut self, f: &mut impl FnMut(&'a mut Measuring)) {
        match self {
            Measuring::Neg(a) => a.collect_mut(f),
            Measuring::Bin(_, a, b) => {
                a.collect_mut(f);
                b.collect_mut(f);
            }
            Measuring::Call(_, args) => args.iter_mut().for_each(|a| a.collect_mut(f)),
            leaf => f(leaf),
        }
    }

    /// As it is written, parenthesised where the tree needs it.
    pub fn text(&self) -> String {
        use crate::expr::Op;
        // how tightly an operator binds, as `expr`'s parser reads it
        fn prec(m: &Measuring) -> u8 {
            match m {
                Measuring::Bin(Op::Add | Op::Sub, ..) => 1,
                Measuring::Bin(Op::Mul | Op::Div, ..) => 2,
                Measuring::Neg(_) => 3,
                Measuring::Bin(Op::Pow, ..) => 4,
                _ => 5,
            }
        }
        let wrapped = |m: &Measuring, at_least: u8| {
            let t = m.text();
            if prec(m) < at_least { format!("({t})") } else { t }
        };
        match self {
            Measuring::Number { text, .. } => text.clone(),
            Measuring::Measure(m) => m.text(),
            Measuring::Neg(a) => format!("-{}", wrapped(a, 3)),
            Measuring::Bin(op, a, b) => {
                let p = prec(self);
                // left-associative but for the power, which is right-associative
                let (l, r) = if *op == Op::Pow { (p + 1, p) } else { (p, p + 1) };
                let sign = match op {
                    Op::Add => "+",
                    Op::Sub => "-",
                    Op::Mul => "*",
                    Op::Div => "/",
                    Op::Pow => "^",
                };
                format!("{} {sign} {}", wrapped(a, l), wrapped(b, r))
            }
            Measuring::Call(f, args) => {
                let args: Vec<String> = args.iter().map(Measuring::text).collect();
                format!("{}({})", f.text, args.join(", "))
            }
        }
    }
}
