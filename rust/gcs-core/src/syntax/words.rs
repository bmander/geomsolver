//! Shared keyword and operator lookahead rules.

use super::lexer::{ident_char, ident_start, Tok};
use super::Span;
use crate::constraints::{is_operator, CKind};
use crate::model::EntKind;

// `port`, `frame` and `ellipse` are retired and `ring` is not yet (bmander/geomsolver#47), and
// each is kept here only so a document written with it is told what to write instead of reading
// the word as a name
pub(super) const OPENERS: [&str; 16] = [
    "preview",
    "claim",
    "component",
    "port",
    "unit",
    "style",
    "branch",
    "repeat",
    "cycle",
    "ring",
    "use",
    "frame",
    "ellipse",
    "view",
    "section",
    "dimensions",
];

pub(super) const BLOCKS: [&str; 3] = ["repeat", "cycle", "ring"];

/// `root := fillet(boss, plate, r: 3mm)`: the value of a definition is a fillet — a solid spelled
/// by what it rounds (issue #66) — where the token at `at` is the word and a bracket follows. The
/// parser and the colouring both ask it.
pub(super) fn fillet_at(toks: &[(Tok, Span)], at: usize) -> bool {
    matches!(
        (toks.get(at), toks.get(at + 1)),
        (Some((Tok::Ident(w), _)), Some((Tok::P('('), _))) if w == "fillet"
    )
}

/// `repeat e in rack.profile`: a block runs over a chain's edges when the tokens from `at` (the
/// one after its word) are a name and `in`, which no count expression can begin with, `in` being
/// no operator.  The parser and the colouring both ask it.
pub(super) fn over_chain(toks: &[(Tok, Span)], at: usize) -> bool {
    matches!(
        (toks.get(at), toks.get(at + 1)),
        (Some((Tok::Ident(_), _)), Some((Tok::Ident(w), _))) if w == "in"
    )
}

/// Whether the operator registry admits this joint word.
pub(super) fn joint_word(w: &str) -> bool {
    is_operator(w)
}

/// The words that shape a statement without naming anything — a modifier the parser eats where it
/// stands.  `as` binds a name after it, which is why `highlight` treats that one specially.
pub(super) const MODIFIERS: [&str; 11] = [
    "over", "as", "at", "hint", "class", "from", "in", "private", "construction", "centerline",
    "param",
];

/// The words that may follow a declaration's own, so `class a b` knows where its list ends.
/// A chain's joints are here too: `arc a(center: c) class construction tangent …` is one link.
const TRAILERS: [&str; 6] = ["knots", "weights", "hint", "class", "close", "in"];

/// Resolve `equal` by operand kinds: line lengths or circle/arc radii.
pub fn equal_kind(left: EntKind, right: EntKind) -> Option<CKind> {
    match (left, right) {
        (EntKind::Line, EntKind::Line) => Some(CKind::EqualLength),
        (EntKind::Circle | EntKind::Arc, EntKind::Circle | EntKind::Arc) => {
            Some(CKind::EqualRadius)
        }
        _ => None,
    }
}

/// Whether a word may stand *before* its one operand — `horizontal`, `vertical`, `radius`,
/// `distance` (spec §9.1).  Derived from the operator table by asking it, so a word given a
/// prefix reading later joins the grammar with nothing here to edit.
pub(super) fn prefix_word(w: &str) -> bool {
    [EntKind::Line, EntKind::Circle, EntKind::Arc]
        .iter()
        .any(|&k| crate::constraints::prefix_op(w, k).is_some())
}

/// The identifier at a token position, shared by parsing and highlighting.
pub(super) fn word_at(toks: &[(Tok, Span)], i: usize) -> Option<&str> {
    match toks.get(i).map(|(t, _)| t) {
        Some(Tok::Ident(n)) => Some(n.as_str()),
        _ => None,
    }
}

/// Skip an operator and its optional parentheses for parser and highlighter lookahead.
/// Stop at a statement boundary if the list is unclosed; parsing reports the error.
pub(super) fn past_args(toks: &[(Tok, Span)], i: usize) -> usize {
    let mut j = i + 1;
    // `horizontal (l := line)` — the parentheses are the named link the word stands before,
    // not the word's own arguments
    if toks.get(j).map(|(t, _)| t) != Some(&Tok::P('(')) || named_link_at(toks, j) {
        return j;
    }
    let mut depth = 0i32;
    loop {
        match toks.get(j).map(|(t, _)| t) {
            Some(Tok::P('(')) => depth += 1,
            Some(Tok::P(')')) => {
                depth -= 1;
                if depth == 0 {
                    return j + 1;
                }
            }
            // an unclosed list is a syntax error the parser proper reports; this lookahead only
            // has to stop rather than run off the end
            Some(Tok::Nl) | None => return j,
            _ => {}
        }
        j += 1;
    }
}

/// `(name := line …)` — a chain link named where it stands.  The kind keyword after `:=` is what
/// tells one from an operator's own parentheses holding a named dimension, `distance(w := 60)`.
pub(super) fn named_link_at(toks: &[(Tok, Span)], j: usize) -> bool {
    matches!(
        (toks.get(j), toks.get(j + 1), toks.get(j + 2), word_at(toks, j + 3)),
        (Some((Tok::P('('), _)), Some((Tok::Ident(_), _)), Some((Tok::Define, _)), Some(w))
            if EntKind::parse(w).is_some()
    )
}

/// The word a link opens with at `j`: the element keyword inside a named link, or the word there.
pub(super) fn link_word(toks: &[(Tok, Span)], j: usize) -> Option<&str> {
    if named_link_at(toks, j) {
        word_at(toks, j + 3)
    } else {
        word_at(toks, j)
    }
}

/// Shared chain lookahead: a declaration, or a prefix before an element or another
/// prefix. A standalone call such as `horizontal(bottom)` remains a relation.
pub(super) fn opens_link(w: &str, next: Option<&str>) -> bool {
    if EntKind::parse(w).is_some() {
        return true; // a declaration names itself — or nothing at all, the name being optional
    }
    // the lookahead: it is a pointer test, where `prefix_word` scans the operator table
    let Some(n) = next else { return false };
    (EntKind::parse(n).is_some() || prefix_word(n)) && prefix_word(w)
}

/// A valid identifier that is not reserved by the grammar — no element keyword, trailing clause
/// or joint, and none of the words a statement is shaped by.  Used by source edits that
/// introduce names.  `at` stays reserved so the retired seed spelling gets its diagnostic.
pub fn is_name(s: &str) -> bool {
    let mut cs = s.chars();
    matches!(cs.next(), Some(c) if ident_start(c))
        && cs.all(ident_char)
        && EntKind::parse(s).is_none()
        && !trails_decl(s)
        && !BODY_WORDS.contains(&s)
        && !MODIFIERS.contains(&s)
        && !OPENERS.contains(&s)
}

/// The words of the body rule (§6.9), which no name may be.
const BODY_WORDS: [&str; 3] = ["cut", "union", "bound"];

/// **A word the grammar keeps for itself** (§9.9): an element keyword, a trailing clause, a body
/// word, a modifier or a word opening a statement — what no relation word may be defined as,
/// whatever its fixity.  The constraint words are not here: which of them a definition collides
/// with depends on the fixity (`constraints::builtin_word`).
pub fn reserved_word(s: &str) -> bool {
    EntKind::parse(s).is_some()
        || TRAILERS.contains(&s)
        || BODY_WORDS.contains(&s)
        || ["through", "next", "prev"].contains(&s)
        || MODIFIERS.contains(&s)
        || OPENERS.contains(&s)
}

/// A trailing clause or joint; also terminates class lists and reserves optional names.
pub(super) fn trails_decl(w: &str) -> bool {
    TRAILERS.contains(&w) || joint_word(w)
}
