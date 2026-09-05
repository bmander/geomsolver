//! Shared keyword and operator lookahead rules.

use super::lexer::{ident_char, ident_start, Tok};
use super::Span;
use crate::constraints::{is_operator, CKind};
use crate::model::EntKind;

// `port`, `frame` and `ellipse` are retired and `ring` is not yet (bmander/geomsolver#47), and
// each is kept here only so a document written with it is told what to write instead of reading
// the word as a name
pub(super) const OPENERS: [&str; 16] = [
    "claim",
    "component",
    "param",
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

pub(super) const BLOCKS: [&str; 2] = ["repeat", "cycle"];

/// Whether the operator registry admits this joint word.
pub(super) fn joint_word(w: &str) -> bool {
    is_operator(w)
}

/// The words that shape a statement without naming anything — a modifier the parser eats where it
/// stands.  `as` binds a name after it, which is why `highlight` treats that one specially.
pub(super) const MODIFIERS: [&str; 7] = ["over", "as", "at", "hint", "class", "from", "in"];

/// The words that may follow a declaration's own, so `class a b` knows where its list ends.
/// A chain's joints are here too: `arc a(center: c) class construction tangent …` is one link.
const TRAILERS: [&str; 5] = ["knots", "hint", "class", "close", "in"];

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
    if toks.get(j).map(|(t, _)| t) != Some(&Tok::P('(')) {
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

/// An optional declaration name cannot consume an element keyword or trailing clause.
/// `at` stays reserved so the retired seed spelling gets its diagnostic.
pub(super) fn names_decl(w: &str) -> bool {
    EntKind::parse(w).is_none() && !trails_decl(w) && w != "at" && w != "cut"
}

/// A valid identifier that is not reserved by the grammar. Used by source edits
/// that introduce names.
pub fn is_name(s: &str) -> bool {
    let mut cs = s.chars();
    matches!(cs.next(), Some(c) if ident_start(c))
        && cs.all(ident_char)
        && names_decl(s)
        && !MODIFIERS.contains(&s)
        && !OPENERS.contains(&s)
}

/// A trailing clause or joint; also terminates class lists and reserves optional names.
pub(super) fn trails_decl(w: &str) -> bool {
    TRAILERS.contains(&w) || joint_word(w)
}
