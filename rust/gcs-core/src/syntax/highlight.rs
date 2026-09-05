//! Syntax colouring for complete and partially written programs.

use super::lexer::{lex, Tok};
use super::words::{
    joint_word, names_decl, opens_link, past_args, trails_decl, word_at, BLOCKS, MODIFIERS,
};
use super::{Span, Ty, MAX_TEXT};
use crate::constraints::is_operator;
use crate::model::EntKind;

/// A highlighting category. Unclassified gaps remain ordinary text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tint {
    Comment,
    Num,
    /// `component`, `param`, `cycle`, `point`, `over`, `construction` — a word that starts a
    /// statement or shapes one
    Word,
    /// `Angle`, `circle`, `Tooth` — a word in the place a type is written
    Type,
    /// a constraint's name, where the statement is one: `distance`, `ground`, `point_on_circle`
    Relation,
    /// the name a statement gives what it declares, and the binder a block counts by
    Def,
    /// `r:` — a slot named where it is filled
    Label,
    /// A number inside a hint clause.
    Seed,
    /// The `==` constraint operator.
    Claim,
    /// `class centerline`, and the `.centerline` a `style` block names — presentation, which is
    /// a different statement from what the drawing is and reads as one
    Class,
}

impl Tint {
    /// The class a front end styles it by.  Named here so the core says what the colours are *of*
    /// and a stylesheet only says what they look like.
    pub fn as_str(self) -> &'static str {
        match self {
            Tint::Comment => "comment",
            Tint::Num => "number",
            Tint::Word => "word",
            Tint::Type => "type",
            Tint::Relation => "relation",
            Tint::Def => "def",
            Tint::Label => "label",
            Tint::Seed => "seed",
            Tint::Claim => "claim",
            Tint::Class => "class",
        }
    }
}

/// The grammatical role expected of the next word.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Next {
    /// begins a statement, so it is the word that says what the statement *is*
    Start,
    /// names a class: every plain word after `class`, and the one a `style` block declares
    Class,
    /// names the document's unit: the one word after `unit`
    Unit,
    /// names what the statement declares
    Def,
    /// the same, after an element keyword — where the name is *optional* (§6.1), so a word
    /// reserved for what may follow a declaration keeps its own reading instead
    DeclName,
    /// names the component an instance is of — the one type written without a `:` before it
    Inst,
    /// nothing in particular
    Word,
}

/// Colour a program.
///
/// Spans, in order, over the classified runs only: whatever falls between two of them is ordinary
/// text and a caller writes it plainly, so nothing here has to describe whitespace.  Never fails —
/// a program half-typed is exactly the one being looked at, and it is coloured as far as it goes.
pub fn highlight(src: &str) -> Vec<(Tint, Span)> {
    if src.len() > MAX_TEXT {
        return Vec::new();
    }
    let (lexed, _) = lex(src);
    let mut out: Vec<(Tint, Span)> = Vec::with_capacity(lexed.toks.len());
    let mut at = Next::Start;
    // how deep inside a `hint(…)` clause we are, and 0 outside one.  A number inside one is a
    // seed and every other number is not — the whole of §4.3's lexical rule, and the reason the
    // colouring can say which numbers a solve may rewrite without elaborating anything.
    let mut hint = 0i32;
    // and how deep inside a `style .name { … }` block, whose body is `property: value` pairs
    // rather than statements
    let mut style = 0i32;
    for (i, (t, span)) in lexed.toks.iter().enumerate() {
        let prev = i.checked_sub(1).map(|j| &lexed.toks[j].0);
        match t {
            Tok::P('(') if matches!(prev, Some(Tok::Ident(w)) if w == "hint") => hint = 1,
            Tok::P('(') | Tok::P('[') if hint > 0 => hint += 1,
            Tok::P(')') | Tok::P(']') if hint > 0 => hint -= 1,
            Tok::Nl => hint = 0,
            _ => {}
        }
        if matches!(t, Tok::Ident(w) if w == "style") && at == Next::Start {
            style = 1;
        } else if matches!(t, Tok::P('}')) && style > 0 {
            style = 0;
        }
        let tint = match t {
            Tok::Nl => {
                at = Next::Start;
                continue;
            }
            Tok::Num(_) if hint > 0 => Some(Tint::Seed),
            Tok::Num(_) => Some(Tint::Num),
            // `param w = 100` and `curve e = involute(…)` — an assignment, and not a seed: the
            // clause is what says a number is one now
            Tok::Eq => None,
            Tok::EqEq => Some(Tint::Claim),
            // the joint marker is structure, the way `close` is
            Tok::Arrow => Some(Tint::Word),
            // a body is made of statements, so a brace begins one the way a newline does
            Tok::P('{') | Tok::P('}') => {
                at = Next::Start;
                None
            }
            Tok::P(_) => None,
            Tok::Ident(w) => {
                let (tint, then) = tint_word(w, prev, &lexed.toks, i, at);
                at = then;
                tint
            }
        };
        // anything at all leaves the opening word behind; a name the statement is still owed
        // (`Def`, `Inst`) survives the punctuation in between, which is why only `Start` lapses
        if at == Next::Start && !matches!(t, Tok::P('{') | Tok::P('}')) {
            at = Next::Word;
        }
        // a declaration's name stands against its keyword or not at all — the name is optional,
        // so `line(p1, p2)` must not read `p1` as one once the bracket has gone by
        if at == Next::DeclName && !matches!(t, Tok::Ident(_)) {
            at = Next::Word;
        }
        // a `style` block's body is `property: value` pairs, not statements: the brace above
        // reset the state to `Start`, where `dash:` would read as an instance
        if matches!(t, Tok::P('{')) && style > 0 {
            at = Next::Word;
        }
        if let Some(tint) = tint {
            out.push((tint, *span));
        }
    }
    if lexed.comments.is_empty() {
        return out;
    }
    // the comments were never tokens; put them back where they were written.  Both runs are
    // already in order — the tokens by the walk above and the comments by the lexer — so this is
    // a merge, and sorting the two together would be throwing that away and buying it back.
    let mut merged = Vec::with_capacity(out.len() + lexed.comments.len());
    let mut cs = lexed.comments.into_iter().peekable();
    for run in out {
        while cs.peek().is_some_and(|c| c.lo < run.1.lo) {
            merged.push((Tint::Comment, cs.next().expect("just peeked")));
        }
        merged.push(run);
    }
    merged.extend(cs.map(|c| (Tint::Comment, c)));
    merged
}

/// What one word is, given where in its statement it fell, and what the word after it will be.
/// Split out because it is the whole of the rule and the loop around it is only bookkeeping.
fn tint_word(
    w: &str,
    prev: Option<&Tok>,
    toks: &[(Tok, Span)],
    i: usize,
    at: Next,
) -> (Option<Tint>, Next) {
    let next = toks.get(i + 1).map(|(t, _)| t);
    match at {
        Next::Unit => (Some(Tint::Type), Next::Word),
        Next::Class => {
            // the list runs to the next thing a declaration may say — another trailing clause,
            // or a chain's joint.  The same predicate the parser stops on, asked once.
            if trails_decl(w) {
                return tint_word(w, prev, toks, i, Next::Word);
            }
            (Some(Tint::Class), Next::Class)
        }
        Next::Def => (Some(Tint::Def), Next::Word),
        Next::DeclName => {
            // the name is optional: what follows the keyword may be the next thing the
            // statement says — a clause, a joint, the next link — and those words keep their
            // own colour.  The same predicate the parser decides by, asked once.
            if !names_decl(w) {
                return tint_word(w, prev, toks, i, Next::Word);
            }
            (Some(Tint::Def), Next::Word)
        }
        Next::Inst => (Some(Tint::Type), Next::Word),
        Next::Start => {
            if super::is_name(w) && next == Some(&Tok::Eq) {
                return (Some(Tint::Def), Next::Word);
            }
            // `point p`, `component Gear(…)`, `curve involute(…)`, `param R = …`
            if EntKind::parse(w).is_some() {
                return (Some(Tint::Word), after_kind(w));
            }
            if matches!(w, "component" | "param" | "use") {
                return (Some(Tint::Word), Next::Def);
            }
            // `style .construction { … }` — the class it names is the thing it declares
            if w == "style" {
                return (Some(Tint::Word), Next::Class);
            }
            // `unit mm` — the word, and the unit it names (spec §3.3)
            if w == "unit" {
                return (Some(Tint::Word), Next::Unit);
            }
            // `in top { … }` — the membership block (§6.7): the word, then the plane it names
            if w == "in" {
                return (Some(Tint::Word), Next::Word);
            }
            if BLOCKS.contains(&w) {
                return (Some(Tint::Word), Next::Word);
            }
            // a raw branch: a statement the parser knows by name
            if w == "branch" {
                return (Some(Tint::Relation), Next::Word);
            }
            // `t: Tooth(…)` — a name, a colon and a component
            if next == Some(&Tok::P(':')) {
                return (Some(Tint::Def), Next::Inst);
            }
            // `claim vertical(rail)`: the word after it is a statement start again, so the
            // relation it qualifies is tinted exactly as it would be standing alone
            if w == "claim" {
                return (Some(Tint::Word), Next::Start);
            }
            // the operator table, not the registry's names: `on` and `equal` are constraints
            // the language writes and are not any `CKind`'s name, and `point_on_circle` is a
            // name no document writes any more (spec §9.1)
            (is_operator(w).then_some(Tint::Relation), Next::Word)
        }
        Next::Word => {
            // `c: circle`, `phase: Angle` — the one place a bare word is a type
            if prev == Some(&Tok::P(':')) && Ty::parse(w).is_some() {
                return (Some(Tint::Type), Next::Word);
            }
            if next == Some(&Tok::P(':')) {
                return (Some(Tint::Label), Next::Word);
            }
            // `3in` is one literal to the parser and two tokens here: the inch mark after a
            // number is a unit, plain like `mm` and `deg`, and not the membership clause
            if w == "in" && matches!(prev, Some(Tok::Num(_))) {
                return (None, Next::Word);
            }
            if w == "cut" && prev != Some(&Tok::P('.')) {
                return (Some(Tint::Relation), Next::Word);
            }
            if MODIFIERS.contains(&w) {
                // `cycle N as i` — the binder is a name the block declares; `class a b` names
                // classes until the clause ends
                return (
                    Some(Tint::Word),
                    match w {
                        "as" => Next::Def,
                        "class" => Next::Class,
                        _ => Next::Word,
                    },
                );
            }
            // a chain (spec §6.6): the element keyword mid-line, the words standing prefix to
            // it, the joints between links, and `close`.  Each is claimed only in the company a
            // chain puts it in, so a point *named* `tangent` in an argument list stays plain.
            // past the operator's own parentheses, which is where its right operand is:
            // `radius(25) circle base(…)` and `p distance(80) q` are the prefix and the joint
            // they would be without a number on the word.  The same lookahead `chain_starts`
            // reads, so a word this colours as a relation is one the parser settles as one —
            // and computed *here*, in the one arm that reads it, since the loop around this runs
            // per keystroke and every other arm has already returned.
            let j = past_args(toks, i);
            let next_word = word_at(toks, j);
            // both questions off the one cursor: a line ending in a joint word continues its
            // chain onto the next, and `p distance(80)` ends a line as surely as `p equal` does.
            // A body's `}` ends a statement as a line break does (`end_of_stmt`), so a word
            // standing at one — a one-line body's trailing joint — reads the same way.
            let at_line_end =
                matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Nl | Tok::P('}')) | None);
            if opens_link(w, next_word) {
                // the element keyword names what the link declares; a prefix states a relation
                return match EntKind::parse(w) {
                    Some(_) => (Some(Tint::Word), after_kind(w)),
                    None => (Some(Tint::Relation), Next::Word),
                };
            }
            let at_marker = matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Arrow));
            if (next_word.is_some() || at_line_end || at_marker) && joint_word(w) {
                // `at_marker` is the far-side marker: `A -> equal -> B`
                return (Some(Tint::Relation), Next::Word);
            }
            // Chains close at a line end; inside a face's list it is the marker, not the
            // closing bracket, that distinguishes `-> close` from an edge named `close`.
            if w == "close" && (at_line_end || prev == Some(&Tok::Arrow)) {
                return (Some(Tint::Word), Next::Word);
            }
            (None, Next::Word)
        }
    }
}

/// What follows an element keyword: a name, and the name is optional — except after `curve`,
/// whose form is `curve name = family(…)` and whose name a contact addresses.  `decl()` makes
/// the same exception, so this is the colouring's half of one rule.
fn after_kind(w: &str) -> Next {
    if w == "curve" {
        Next::Def
    } else {
        Next::DeclName
    }
}
