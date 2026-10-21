//! Shared keyword and operator lookahead rules.

use super::lexer::{ident_char, ident_start, Tok};
use super::Span;
use crate::constraints::{is_operator, CKind, Fixity};
use crate::model::EntKind;

// `port` and `ellipse` are retired and `ring` is not yet (bmander/geomsolver#47), and each is kept
// here only so a document written with it is told what to write instead of reading the word as a
// name; `frame`, retired in 0.42, is an element keyword again (#383)
pub(super) const OPENERS: [&str; 20] = [
    "preview",
    "scenario",
    "claim",
    "initially",
    "rate",
    "component",
    "interface",
    "port",
    "unit",
    "style",
    "branch",
    "repeat",
    "cycle",
    "ring",
    "when",
    "use",
    "ellipse",
    "view",
    "section",
    "dimensions",
];

pub(super) const BLOCKS: [&str; 4] = ["repeat", "cycle", "ring", "when"];

/// `repeat e in rack.profile`: a block runs over a chain's edges when the tokens from `at` (the
/// one after its word) are a name and `in`, which no count expression can begin with, `in` being
/// no operator.  The parser and the colouring both ask it.
pub(super) fn over_chain(toks: &[(Tok, Span)], at: usize) -> bool {
    matches!(
        (toks.get(at), toks.get(at + 1)),
        (Some((Tok::Ident(_), _)), Some((Tok::Ident(w), _))) if w == "in"
    ) || over_walks(toks, at)
}

/// Whether a block's header runs over several walks in step: `(ea, eb) in (…)` (#243 L8) — a
/// bracketed list of names, then `in`.
pub(super) fn over_walks(toks: &[(Tok, Span)], at: usize) -> bool {
    if !matches!(toks.get(at), Some((Tok::P('('), _))) {
        return false;
    }
    let mut i = at + 1;
    loop {
        if !matches!(toks.get(i), Some((Tok::Ident(_), _))) {
            return false;
        }
        match toks.get(i + 1) {
            Some((Tok::P(','), _)) => i += 2,
            Some((Tok::P(')'), _)) => break,
            _ => return false,
        }
    }
    matches!(toks.get(i + 2), Some((Tok::Ident(w), _)) if w == "in")
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
/// `length` (spec §9.1).  Derived from the operator table by asking it, so a word given a
/// prefix reading later joins the grammar with nothing here to edit; a retired prefix spelling
/// (`distance` on a line) is still read, to be refused with the word it became.
pub(super) fn prefix_word(w: &str) -> bool {
    use crate::constraints::{prefix_op, retired_prefix, step_word};
    step_word(w)
        || [EntKind::Line, EntKind::Circle, EntKind::Arc]
            .iter()
            .any(|&k| prefix_op(w, k).is_some() || retired_prefix(w, k).is_some())
}

/// The identifier at a token position, shared by parsing and highlighting.
pub(super) fn word_at(toks: &[(Tok, Span)], i: usize) -> Option<&str> {
    match toks.get(i).map(|(t, _)| t) {
        Some(Tok::Ident(n)) => Some(n.as_str()),
        _ => None,
    }
}

/// **A set literal opening at `i`** (§6.21): `{`, one name, `|`, line breaks between them read
/// past as the lexer reads past them — the token index of the name it binds, or `None`.  The
/// parser and the colouring both ask it.
pub(super) fn set_at(toks: &[(Tok, Span)], i: usize) -> Option<usize> {
    if toks.get(i).map(|(t, _)| t) != Some(&Tok::P('{')) {
        return None;
    }
    let mut rest = toks.iter().enumerate().skip(i + 1).filter(|(_, (t, _))| *t != Tok::Nl);
    let (name, _) = rest.next().filter(|(_, (t, _))| matches!(t, Tok::Ident(_)))?;
    rest.next().is_some_and(|(_, (t, _))| *t == Tok::P('|')).then_some(name)
}

/// Which brackets a scan (`level_end`) steps over.
#[derive(Clone, Copy)]
pub(super) enum Brackets {
    /// `(` and `[`: an expression's.
    Expr,
    /// `(` alone: a word's parentheses.
    Round,
    /// `[` alone: an index.
    Square,
    /// `(`, `[` and `{`: whatever a statement's text holds.
    All,
}

impl Brackets {
    fn opens(self, c: char) -> bool {
        match self {
            Brackets::Expr => matches!(c, '(' | '['),
            Brackets::Round => c == '(',
            Brackets::Square => c == '[',
            Brackets::All => matches!(c, '(' | '[' | '{'),
        }
    }

    fn closes(self, c: char) -> bool {
        match self {
            Brackets::Expr => matches!(c, ')' | ']'),
            Brackets::Round => c == ')',
            Brackets::Square => c == ']',
            Brackets::All => matches!(c, ')' | ']' | '}'),
        }
    }
}

/// **The one bracket scanner**, for the parser's raw texts and every lookahead: from token
/// `from`, stepping over what `brackets` opens, the index of the first token that closes the level
/// the scan began on, that `stop` takes at that level, or that ends the line — else `end` (no
/// further than the tokens go).  Any closing bracket closes the level: an unbalanced text is the
/// parser's to report, and the scan only has to stop.
pub(super) fn level_end(
    toks: &[(Tok, Span)],
    from: usize,
    end: usize,
    brackets: Brackets,
    stop: impl Fn(&Tok) -> bool,
) -> usize {
    let end = end.min(toks.len());
    let mut depth = 0i32;
    for (j, (t, _)) in toks.iter().enumerate().take(end).skip(from) {
        match t {
            Tok::P(c) if brackets.opens(*c) => depth += 1,
            Tok::P(c) if brackets.closes(*c) => {
                if depth == 0 {
                    return j;
                }
                depth -= 1;
            }
            Tok::Nl => return j,
            _ if depth == 0 && stop(t) => return j,
            _ => {}
        }
    }
    end
}

/// The token index just past the bracket closing the one open at `open`, or where the line ends
/// first (an unclosed list is a syntax error the parser proper reports; a lookahead only has to
/// stop rather than run off the end).
pub(super) fn past_bracket(toks: &[(Tok, Span)], open: usize, brackets: Brackets) -> usize {
    let j = level_end(toks, open + 1, toks.len(), brackets, |_| false);
    match toks.get(j) {
        Some((Tok::P(c), _)) if brackets.closes(*c) => j + 1,
        _ => j,
    }
}

/// **`part Name(…) {` at `i`** (#461): a part's definition — the one place `part` is a word, so
/// it stays a name a document may give (`part := solid(…)`, `part above(7) q`).  The parser and
/// the colouring both ask it.
pub(super) fn part_at(toks: &[(Tok, Span)], i: usize) -> bool {
    let at = |k: usize| toks.get(k).map(|(t, _)| t);
    matches!(at(i), Some(Tok::Ident(w)) if w == "part")
        && matches!(at(i + 1), Some(Tok::Ident(_)))
        && at(i + 2) == Some(&Tok::P('('))
        && at(past_bracket(toks, i + 2, Brackets::Round)) == Some(&Tok::P('{'))
}

/// Skip an operator and its optional parentheses for parser and highlighter lookahead.
/// Stop at a statement boundary if the list is unclosed; parsing reports the error.
pub(super) fn past_args(toks: &[(Tok, Span)], i: usize) -> usize {
    let j = i + 1;
    // `horizontal (l := line)` — the parentheses are the named link the word stands before,
    // not the word's own arguments
    if toks.get(j).map(|(t, _)| t) != Some(&Tok::P('(')) || named_link_at(toks, j) {
        return j;
    }
    past_bracket(toks, j, Brackets::Round)
}

/// **`a word(params) b :=` or `word(params) l :=` at `i`** — a relation word being defined
/// (§9.9): its fixity, or `None`.  Names and a word, with at most one parenthesised list after
/// the word, then `:=`; a modifier in front (`param w := …`, `construction c := …`) is a
/// definition of another kind.  The parser and the colouring both ask it.
pub(super) fn word_definition_at(toks: &[(Tok, Span)], i: usize) -> Option<Fixity> {
    let first = word_at(toks, i)?;
    if MODIFIERS.contains(&first) {
        return None;
    }
    // the word at `w`: its parameters, if any, then the last operand and `:=`
    let tail = |w: usize| {
        let j = past_args(toks, w);
        word_at(toks, j).is_some() && toks.get(j + 1).map(|(t, _)| t) == Some(&Tok::Define)
    };
    if word_at(toks, i + 1).is_some() && tail(i + 1) {
        return Some(Fixity::Infix);
    }
    tail(i).then_some(Fixity::Prefix)
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

/// `frame` at `j` as a whole value (#383): the last word of its statement, or before its class
/// or the bracket that refuses it.  Read so only there, so `frame` stays a name a document may
/// give anything else.
pub(super) fn bare_frame_at(toks: &[(Tok, Span)], j: usize) -> bool {
    word_at(toks, j) == Some("frame")
        && match toks.get(j + 1).map(|(t, _)| t) {
            None | Some(Tok::Nl) | Some(Tok::P('}')) => true,
            // its trailers, which it refuses but `class`, said where they are
            Some(Tok::Ident(w)) => ["class", "hint", "in"].contains(&w.as_str()),
            _ => false,
        }
}

/// `(name := frame)` at `j`: an `in` block's header declaring the frame it draws in (#383).
pub(super) fn named_frame_at(toks: &[(Tok, Span)], j: usize) -> bool {
    matches!(
        (toks.get(j), toks.get(j + 1), toks.get(j + 2), word_at(toks, j + 3), toks.get(j + 4)),
        (Some((Tok::P('('), _)), Some((Tok::Ident(_), _)), Some((Tok::Define, _)), Some("frame"),
         Some((Tok::P(')'), _)))
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
        && !ENERGY_WORDS.contains(&s)
        && !MODIFIERS.contains(&s)
        && !OPENERS.contains(&s)
}

/// The words of the body rule (§6.9), which no name may be.
const BODY_WORDS: [&str; 3] = ["cut", "union", "common"];

/// What a curve is stationary for (#121), `rope minimizes …`; no name either.
pub(super) const ENERGY_WORDS: [&str; 2] = ["minimizes", "maximizes"];

/// **A word the grammar keeps for itself** (§9.9): an element keyword, a trailing clause, a body
/// word, a modifier or a word opening a statement — what no relation word may be defined as,
/// whatever its fixity.  The constraint words are not here: which of them a definition collides
/// with depends on the fixity (`constraints::builtin_word`).
pub fn reserved_word(s: &str) -> bool {
    EntKind::parse(s).is_some()
        || TRAILERS.contains(&s)
        || BODY_WORDS.contains(&s)
        || ENERGY_WORDS.contains(&s)
        || ["through", "bound", "next", "prev"].contains(&s)
        || MODIFIERS.contains(&s)
        || OPENERS.contains(&s)
}

/// A trailing clause or joint; also terminates class lists and reserves optional names.
pub(super) fn trails_decl(w: &str) -> bool {
    TRAILERS.contains(&w) || joint_word(w)
}
