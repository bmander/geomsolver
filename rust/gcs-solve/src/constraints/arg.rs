//! A constraint's arguments in `spec` order, and the bound on its number.

use super::*;

/// One constructor argument, in `spec` order.
#[derive(Clone, Debug, PartialEq)]
pub enum Arg {
    Ent(EntRef),
    Num(f64),
    Int(i64),
    Bool(bool),
    Str(String),
    /// A dimension written as text (`w = 1`, `h = w * 2`), carrying the number it evaluates to —
    /// see `expr`.  Only a `Length` or `Angle` slot holds one.
    Expr(crate::expr::Expr),
    /// An index into `Sketch::params`: the unknown this constraint owns, filled in by
    /// `Sketch::add`.  Only a `Param` slot holds one, and only after the constraint has been
    /// added — before that the slot carries an `Arg::Seed` (or a bare `Arg::Num`).
    Param(u32),
    /// What a `Param` slot holds on the way in: the number the unknown starts at, and whether
    /// the caller means it to *stay* there.  A pinned unknown is one somebody has already
    /// worked out — a fit knows where along the curve each of its points sits — so the solver
    /// is not to move it.
    ///
    /// Both halves travel together and are consumed together by `Sketch::add`, which is the one
    /// seam that turns a number into a Param.  That is the point of the variant: a path that
    /// carries the value cannot drop the pin, so a document, a paste, a rebuild and a
    /// constructor are all correct without knowing pins exist.
    Seed { value: f64, pinned: bool },
    /// What a `Param` slot holds on the way in when the unknown is **shared**: the name every
    /// contact owning it is written against (`t == s`, issue #70, part 2), and where it starts
    /// if this is the first of them.  `Sketch::add` allocates the unknown for the first and hands
    /// the same index to the rest (`Sketch::shared`), so the slot is an `Arg::Param` after it,
    /// as every owned unknown is.
    Shared { name: String, seed: Option<f64> },
}

impl Arg {
    pub fn ent(&self) -> EntRef {
        match self {
            Arg::Ent(e) => *e,
            _ => panic!("argument is not an entity"),
        }
    }
    /// The Param index of an allocated unknown.
    pub fn param(&self) -> u32 {
        match self {
            Arg::Param(i) => *i,
            _ => panic!("argument is not an allocated parameter"),
        }
    }
    /// What this argument is worth as a number, resolving an owned unknown through the sketch
    /// that holds it — the one place that lookup lives, for the document writer, `graft`, the
    /// bindings' records and the contact accessors alike.
    pub fn value(&self, sk: &Sketch) -> f64 {
        match self {
            Arg::Param(i) => sk.params[*i as usize].value,
            a => a.num(),
        }
    }
    pub fn num(&self) -> f64 {
        match self {
            Arg::Num(v) => *v,
            Arg::Seed { value, .. } => *value,
            Arg::Shared { seed, .. } => seed.unwrap_or(0.0),
            Arg::Int(v) => *v as f64,
            Arg::Bool(b) => {
                if *b {
                    1.0
                } else {
                    0.0
                }
            }
            Arg::Expr(e) => e.value,
            _ => panic!("argument is not a number"),
        }
    }
}

/// What a bound says of its constraint's number, the low end (§9.6): which way it runs, and an
/// interval's high end, in the number's own units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bound {
    pub cmp: crate::vocab::Cmp,
    pub hi: Option<f64>,
}

impl Bound {
    /// Whether a reading `m` keeps a bound whose low end is `lo`, within `tol`.
    pub fn holds(&self, lo: f64, m: f64, tol: f64) -> bool {
        use crate::vocab::Cmp;
        match self.cmp {
            Cmp::Ge => m >= lo - tol,
            Cmp::Le => m <= lo + tol,
            Cmp::In => m >= lo - tol && m <= self.hi.unwrap_or(lo) + tol,
        }
    }

    /// Where a solve steering a reading `m` back across the bound aims, nearest first: its mirror
    /// in the bound's edge, which is where the other root of a mirror-symmetric pair reads, then
    /// twice, four and eight times as far past the edge — the other root of a pair mirrored in
    /// some other measure reads farther off.  Kept within what the reading can come to (`range`):
    /// an aim past its limit is halfway from the edge to it, and the last, since one farther
    /// would be the same.  An interval aims at its middle alone.
    pub fn aims(&self, lo: f64, m: f64, range: (f64, f64)) -> Vec<f64> {
        if self.cmp == crate::vocab::Cmp::In {
            return vec![(lo + self.hi.unwrap_or(lo)) / 2.0];
        }
        let first = 2.0 * lo - m;
        let mut out = Vec::with_capacity(4);
        for k in [1.0, 2.0, 4.0, 8.0] {
            let aim = lo + (first - lo) * k;
            let limit = if aim < range.0 {
                Some(range.0)
            } else {
                (aim > range.1).then_some(range.1)
            };
            match limit {
                Some(l) => {
                    out.push((l + lo) / 2.0);
                    break;
                }
                None => out.push(aim),
            }
        }
        out
    }
}
