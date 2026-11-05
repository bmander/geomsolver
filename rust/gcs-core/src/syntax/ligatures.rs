//! **The signs an editor may draw the notation of sets as** (#783 P2): `in` as ∈, `not in` as ∉,
//! `some` as ∃, and `every` as ∀ (P3).  The document stays ASCII — one spelling, parsed, diffed
//! and typed on any keyboard — and these are runs of it an editor *draws* as one sign over exactly
//! the characters they stand for, the ASCII still there beneath for the caret, the selection and
//! the clipboard.
//!
//! Which runs are signs is the core's answer and the colouring's own scan (`highlight::tints`),
//! never a pattern a front end matches: a word in a comment is no token, and a word the text
//! gives as a name, or a label (`distance(in: (a, b))`), is tinted otherwise and so is no sign.
//! `in` is a sign only where it stands between two operands, as membership does: `p in S`, `t
//! in (a, b)`, `repeat e in S`, `p := point in P` — not the block `in P { … }`, which has nothing
//! before it.  `or` stays a word: ∨ is logic, and ∪ would be a second reading of it.

use super::highlight::{tints, Tint};
use super::lexer::{lex, Tok};
use super::words::word_at;
use super::{Span, MAX_TEXT};

/// One sign, and the words it is drawn over.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ligature {
    /// `in`, membership: ∈
    In,
    /// `not in`, its negation: ∉, over both words and the space between them
    NotIn,
    /// `some`, the quantifier: ∃
    Some,
    /// `every`, the universal one (#783 P3): ∀
    Every,
}

impl Ligature {
    /// The sign drawn.
    pub fn glyph(self) -> &'static str {
        match self {
            Ligature::In => "∈",
            Ligature::NotIn => "∉",
            Ligature::Some => "∃",
            Ligature::Every => "∀",
        }
    }
}

/// The runs of a program an editor may draw as signs, in order and disjoint, each over its own
/// words' bytes.  A function of the text, as the colouring is: the program being looked at is the
/// one half-typed.
pub fn ligatures(src: &str) -> Vec<(Ligature, Span)> {
    if src.len() > MAX_TEXT {
        return Vec::new();
    }
    let (lexed, _) = lex(src);
    let t = &lexed.toks;
    let tint = tints(t);
    let word = |i: usize| tint.get(i) == Some(&Some(Tint::Word));
    // an operand ends at `i`: a name, a number, or a bracket closed
    let ends = |i: usize| matches!(t[i].0, Tok::Ident(_) | Tok::Num(_) | Tok::P(')' | ']' | '}'));
    // and one starts after `i`, on its line: not a label's colon, not a line's end
    let starts = |i: usize| {
        matches!(t.get(i + 1).map(|x| &x.0), Some(Tok::Ident(_) | Tok::Num(_) | Tok::P('(' | '{')))
    };
    let mut out = Vec::new();
    let mut i = 0;
    while i < t.len() {
        match word_at(t, i) {
            // `p not in S`, `when w not in A`
            Some("not") if word_at(t, i + 1) == Some("in") && word(i + 1) && i > 0 && ends(i - 1)
                && starts(i + 1) =>
            {
                out.push((Ligature::NotIn, Span::new(t[i].1.lo as usize, t[i + 1].1.hi as usize)));
                i += 2;
                continue;
            }
            Some("in") if word(i) && i > 0 && ends(i - 1) && starts(i) => {
                out.push((Ligature::In, t[i].1));
            }
            // `some t: …`, `some t in (a, b): …`, `every q in a: …` — a quantifier, before the
            // name it binds
            Some(w @ ("some" | "every")) if word_at(t, i + 1).is_some()
                && (matches!(t.get(i + 2).map(|x| &x.0), Some(Tok::P(':')))
                    || word_at(t, i + 2) == Some("in")) =>
            {
                out.push((if w == "some" { Ligature::Some } else { Ligature::Every }, t[i].1));
            }
            _ => {}
        }
        i += 1;
    }
    out
}
