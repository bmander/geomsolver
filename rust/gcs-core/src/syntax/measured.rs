//! **A bound on measurements** (§6.21, #183 F3, #190): `distance(p, f1) + distance(p, f2) <=
//! 10mm`, each side a tree of measurements and numbers as written — and **an equation of
//! integrals** (§6.25): `integral(density(p) over p in boat) == M`, stated of the drawing or
//! claimed of it, each side the same tree with integrals among its terms.

use super::{names, Cmp, Name, Ref, Span};

/// `distance(p, f1) + distance(p, f2) <= 10mm` — a bound on an expression of measurements
/// (§6.21, #183 F3): in a set's body, where the point the set is about may be, as the distances
/// from it say.  **Parsed once** (#190): each side is a `Measuring` tree whose measurements name
/// what they measure by `Ref`s, rewritten by the flattener as every other operand is, and whose
/// numbers are texts it settles over the scope.  Closed, as every bound is.
#[derive(Clone, Debug)]
pub struct Measured {
    pub lhs: Measuring,
    /// The comparison: a bound, `<=` or `>=` — or `None`, `==`, an equation, which only one of
    /// integrals is (§6.25).
    pub cmp: Option<Cmp>,
    pub rhs: Measuring,
    pub span: Span,
    /// `claim` before it: judged on the solved drawing rather than held by the solve (§6.25).
    pub claim: bool,
    /// What a report calls it where a word's body stated it (§6.25): the word's use as written,
    /// `mass(10kg) arm`, the outermost where words nest.  `None` as written.
    pub head: Option<String>,
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
    /// **An integral over a set** (§6.25): `integral(EXPR over p in S)`, its body a tree over
    /// the point it binds.
    Integral(Box<Integration>),
    /// **The density of what point `p` is in** (§6.25): `density(p)`, in an integral's body.
    Density(Ref, Span),
}

/// **`integral(EXPR over p in S)`** (§6.25): `body` integrated over `over` — a solid, a face of
/// one, a beam or a line one is drawn on, or an instance's bodies — by its own measure (a volume,
/// an area, a member's section along its length), `var` the point it binds.  The body names the
/// point by `var` alone, which is no name of the drawing: no rewrite touches it.
#[derive(Clone, Debug)]
pub struct Integration {
    pub body: Measuring,
    pub var: Name,
    pub over: Ref,
    pub span: Span,
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
    /// The direction a distance is signed along (§6.25): `distance(g, p, along: std.x)`, the
    /// ordinate of the second point from the first.
    pub along: Option<Ref>,
    pub span: Span,
}

impl Measurement {
    /// As it is written: `distance(p, f1)`, `angle(a, b, at: p)`.
    pub fn text(&self) -> String {
        let mut args: Vec<String> = self.args.iter().map(names::ref_text).collect();
        if let Some(at) = &self.at {
            args.push(format!("at: {}", names::ref_text(at)));
        }
        if let Some(along) = &self.along {
            args.push(format!("along: {}", names::ref_text(along)));
        }
        format!("{}({})", self.measure.name(), args.join(", "))
    }

    /// Every reference it reads, the `at:` point and the `along:` direction last.
    pub fn refs(&self) -> impl Iterator<Item = &Ref> {
        self.args.iter().chain(self.at.iter()).chain(self.along.iter())
    }

    pub fn refs_mut(&mut self) -> impl Iterator<Item = &mut Ref> {
        self.args.iter_mut().chain(self.at.iter_mut()).chain(self.along.iter_mut())
    }
}

/// Whether reference `r` reads something of the drawing rather than a point an enclosing
/// integral binds (`bound`, innermost last): the one rule both reference walkers keep.
fn free(r: &Ref, bound: &[String]) -> bool {
    !(r.path.is_empty() && bound.iter().any(|b| *b == r.root.text))
}

impl Measured {
    /// Whether either side integrates (§6.25): an equation of the drawing or a claim of it, never
    /// a set's bound.
    pub fn integrates(&self) -> bool {
        self.lhs.integrates() || self.rhs.integrates()
    }

    /// The comparison as written: `<=`, `>=` or `==`.
    pub fn cmp_text(&self) -> &'static str {
        self.cmp.map_or("==", Cmp::text)
    }

    /// Every reference either side reads, but the points its integrals bind, mutably: what the
    /// flattener rewrites.
    pub fn refs_mut(&mut self) -> Vec<&mut Ref> {
        let mut out = self.lhs.refs_mut();
        out.extend(self.rhs.refs_mut());
        out
    }

    /// Every reference either side reads but the points its integrals bind.
    pub fn refs(&self) -> Vec<&Ref> {
        let mut out = self.lhs.refs();
        out.extend(self.rhs.refs());
        out
    }

    /// Every number's text, either side, inside its integrals too, read only.
    pub fn numbers(&self) -> Vec<&str> {
        let mut out = self.lhs.numbers();
        out.extend(self.rhs.numbers());
        out
    }

    /// Every number's text, either side, inside its integrals too.
    pub fn numbers_mut(&mut self) -> Vec<&mut String> {
        let mut out = self.lhs.numbers_mut();
        out.extend(self.rhs.numbers_mut());
        out
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

    /// Whether it reads geometry — a measurement, an integral, a density — rather than being a
    /// number alone.
    pub fn reads_geometry(&self) -> bool {
        let mut found = false;
        self.walk(&mut |m| found |= matches!(m, Measuring::Measure(_) | Measuring::Integral(_)
                                                 | Measuring::Density(..)));
        found
    }

    /// Whether it holds an integral (§6.25).
    pub fn integrates(&self) -> bool {
        let mut found = false;
        self.walk(&mut |m| found |= matches!(m, Measuring::Integral(_)));
        found
    }

    /// Every integral, outermost first, in the order written.
    pub fn integrals(&self) -> Vec<&Integration> {
        let mut out = Vec::new();
        self.walk(&mut |m| {
            if let Measuring::Integral(i) = m {
                out.push(&**i);
            }
        });
        out
    }

    /// Every reference it reads but the points its integrals bind, mutably — a measurement's,
    /// a density's, what an integral is over — the flattener's to rewrite as every operand.
    pub fn refs_mut(&mut self) -> Vec<&mut Ref> {
        let mut out = Vec::new();
        self.refs_bound_mut(&mut Vec::new(), &mut out);
        out
    }

    /// `refs_mut`, read only.
    pub fn refs(&self) -> Vec<&Ref> {
        fn go<'a>(m: &'a Measuring, bound: &mut Vec<String>, out: &mut Vec<&'a Ref>) {
            match m {
                Measuring::Number { .. } => {}
                Measuring::Measure(x) => out.extend(x.refs().filter(|r| free(r, bound))),
                Measuring::Density(r, _) => {
                    if free(r, bound) {
                        out.push(r);
                    }
                }
                Measuring::Neg(a) => go(a, bound, out),
                Measuring::Bin(_, a, b) => {
                    go(a, bound, out);
                    go(b, bound, out);
                }
                Measuring::Call(_, args) => args.iter().for_each(|a| go(a, bound, out)),
                Measuring::Integral(i) => {
                    if free(&i.over, bound) {
                        out.push(&i.over);
                    }
                    bound.push(i.var.text.clone());
                    go(&i.body, bound, out);
                    bound.pop();
                }
            }
        }
        let mut out = Vec::new();
        go(self, &mut Vec::new(), &mut out);
        out
    }

    fn refs_bound_mut<'a>(&'a mut self, bound: &mut Vec<String>, out: &mut Vec<&'a mut Ref>) {
        match self {
            Measuring::Number { .. } => {}
            Measuring::Measure(m) => {
                out.extend(m.refs_mut().filter(|r| free(r, bound)));
            }
            Measuring::Density(r, _) => {
                if free(r, bound) {
                    out.push(r);
                }
            }
            Measuring::Neg(a) => a.refs_bound_mut(bound, out),
            Measuring::Bin(_, a, b) => {
                a.refs_bound_mut(bound, out);
                b.refs_bound_mut(bound, out);
            }
            Measuring::Call(_, args) => {
                for a in args {
                    a.refs_bound_mut(bound, out);
                }
            }
            Measuring::Integral(i) => {
                let Integration { body, var, over, .. } = &mut **i;
                if free(over, bound) {
                    out.push(over);
                }
                bound.push(var.text.clone());
                body.refs_bound_mut(bound, out);
                bound.pop();
            }
        }
    }

    fn walk<'a>(&'a self, f: &mut impl FnMut(&'a Measuring)) {
        f(self);
        match self {
            Measuring::Number { .. } | Measuring::Measure(_) | Measuring::Density(..) => {}
            Measuring::Integral(i) => i.body.walk(f),
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

    /// Every number's text, in the order written, inside its integrals too.
    pub fn numbers(&self) -> Vec<&str> {
        let mut out = Vec::new();
        self.walk(&mut |m| {
            if let Measuring::Number { text, .. } = m {
                out.push(text.as_str());
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
            Measuring::Integral(i) => i.body.collect_mut(f),
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
            Measuring::Integral(i) => format!("integral({} over {} in {})", i.body.text(),
                                              i.var.text, names::ref_text(&i.over)),
            Measuring::Density(r, _) => format!("density({})", names::ref_text(r)),
        }
    }
}
