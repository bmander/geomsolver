//! The few words and types the model shares with the language, defined below both.
//!
//! A bound's direction (`Cmp`), a body word (`BodyWord`), where a statement was written (`Span`),
//! the classes a declaration carries (`Classes`) and the spelling helpers every message uses
//! (`num`, `snake`, `one_of`) are read by the model, the constraints and the solids as much as by
//! the parser.  They live here, depending on nothing, so the solver does not import the parser to
//! name them; `syntax` re-exports each (`style` re-exports `Classes`), and the language reads them
//! there.

/// A half-open byte range into the program text.
///
/// Bytes, not characters: the front end slices the same `&str` the core parsed, and a UTF-8
/// boundary is the only thing the two ever have to agree about.  Line and column are *not* stored
/// — `syntax::line_col` computes them on demand, so there is nothing to keep in step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Span {
    pub lo: u32,
    pub hi: u32,
}

impl Span {
    pub fn new(lo: usize, hi: usize) -> Span {
        Span { lo: lo as u32, hi: hi as u32 }
    }

    /// The empty span at `at`: where something not written would go.
    pub fn at(at: usize) -> Span {
        Span::new(at, at)
    }

    pub fn slice(self, text: &str) -> &str {
        text.get(self.lo as usize..self.hi as usize).unwrap_or("")
    }

    pub fn contains(self, off: u32) -> bool {
        self.lo <= off && off < self.hi
    }

    pub fn len(self) -> u32 {
        self.hi.saturating_sub(self.lo)
    }

    pub fn is_empty(self) -> bool {
        self.len() == 0
    }
}

/// Which way a bound runs (§9.6).  Closed, every one: a solution may stand on its edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmp {
    /// `>= d`
    Ge,
    /// `<= d`
    Le,
    /// `in: (a, b)`
    In,
}

impl Cmp {
    /// As written, and as a document stores it.
    pub fn text(self) -> &'static str {
        match self {
            Cmp::Ge => ">=",
            Cmp::Le => "<=",
            Cmp::In => "in",
        }
    }

    pub fn of(text: &str) -> Option<Cmp> {
        [Cmp::Ge, Cmp::Le, Cmp::In].into_iter().find(|c| c.text() == text)
    }

    /// The other way: a bound on a measure, read on its negative.  `None` for an interval, which
    /// is either side of it.
    pub fn turned(self) -> Option<Cmp> {
        match self {
            Cmp::Ge => Some(Cmp::Le),
            Cmp::Le => Some(Cmp::Ge),
            Cmp::In => None,
        }
    }
}

/// Which side of the body rule a statement fills.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyWord {
    /// `boss union cyl` — material.
    Union,
    /// `bore cut cyl` — the body rule's subtraction (§6.9). Relations are folded into
    /// the stock body after declarations are built.
    Cut,
    /// `tip common cyl` — the body rule's intersection: the body keeps what it has in common
    /// with every solid stated so.  Through 0.84 the word was `bound` (#270).
    Common,
}

impl BodyWord {
    /// The word as written, or `None` for one that is not a body word.
    pub fn parse(w: &str) -> Option<BodyWord> {
        [BodyWord::Union, BodyWord::Cut, BodyWord::Common].into_iter().find(|b| b.as_str() == w)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            BodyWord::Union => "union",
            BodyWord::Cut => "cut",
            BodyWord::Common => "common",
        }
    }
}

/// `PointOnLine` → `point_on_line`.  A run of capitals stays together, so `K33`-shaped names do
/// not come apart, though none of the 32 has one.
pub fn snake(name: &str) -> String {
    let mut out = String::with_capacity(name.len() + 4);
    let cs: Vec<char> = name.chars().collect();
    for (i, &c) in cs.iter().enumerate() {
        if c.is_ascii_uppercase() {
            let starts_word = i > 0
                && (!cs[i - 1].is_ascii_uppercase()
                    || cs.get(i + 1).is_some_and(|n| n.is_ascii_lowercase()));
            if starts_word {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

/// `point_on_line` → `PointOnLine`, the inverse of `snake` on every name the registry holds —
/// which `tests/syntax.rs` checks for all 32 rather than assuming.
pub fn camel(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    let mut up = true;
    for c in name.chars() {
        if c == '_' {
            up = true;
        } else if up {
            out.extend(c.to_uppercase());
            up = false;
        } else {
            out.push(c);
        }
    }
    out
}

/// `x` or `y`; `start`, `end` or `p1` — the words a key will take, as a reader reads a list.
/// Every refusal that names a vocabulary comes through here, so two of them cannot punctuate the
/// same list differently (issue #48, item 4).
pub fn one_of(words: &[&str]) -> String {
    match words {
        [] => "nothing".to_string(),
        [a] => format!("`{a}`"),
        [rest @ .., last] => {
            let head: Vec<String> = rest.iter().map(|w| format!("`{w}`")).collect();
            format!("{} or `{last}`", head.join(", "))
        }
    }
}

/// The classes a declaration carries, in the order they were written: `class centerline heavy`.
///
/// Several, because one costs nothing now and forecloses nothing: on a conflicting property the
/// later class wins, so `class centerline heavy` is a centreline drawn thick.
///
/// **Tags, to the model** (#409, step 5).  An entity and a constraint carry their classes as
/// names and nothing else: what a class looks like is the sheet's (`style::Sheet`), resolved by
/// `style`, and nothing that solves, diagnoses or decomposes reads one.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Classes(pub Vec<String>);

impl Classes {
    pub fn one(name: &str) -> Classes {
        Classes(vec![name.to_string()])
    }

    pub fn has(&self, name: &str) -> bool {
        self.0.iter().any(|c| c == name)
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Add or remove one class, keeping the order of the rest — which is what decides a
    /// conflicting property, so a toggle must not reshuffle them.
    pub fn set(&mut self, name: &str, on: bool) {
        match (on, self.has(name)) {
            (true, false) => self.0.push(name.to_string()),
            (false, true) => self.0.retain(|c| c != name),
            _ => {}
        }
    }
}

/// A number as text: shortest round-trip, which is what a document needs.
///
/// Never `json::fmt_g` — that rounds for display, and a value rounded on the way out is a drawing
/// that moved because somebody looked at it.
pub fn num(v: f64) -> String {
    if !v.is_finite() {
        // a document should never hold one, and printing `inf` would produce a program that does
        // not parse; 0 is wrong in a way somebody will notice, which is the point
        return "0".to_string();
    }
    // a negative zero is zero: `-0` is a number nobody wrote, and an inverted map makes them
    if v == 0.0 {
        return "0".to_string();
    }
    format!("{v}")
}
