//! Syntax colouring for complete and partially written programs.

use std::collections::BTreeSet;

use super::lexer::{lex, Tok};
use super::words::{
    joint_word, link_word, opens_link, over_chain, past_args, trails_decl, word_at,
    word_definition_at, BLOCKS, MODIFIERS,
};
use super::{Span, Ty, MAX_TEXT};
use crate::constraints::{is_operator, Fixity};
use crate::expr::{CONSTANTS, FUNCTIONS, MEASURES};
use crate::model::EntKind;

/// A highlighting category. Unclassified gaps remain ordinary text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tint {
    Comment,
    Num,
    /// `component`, `cycle`, `point`, `over`, `construction` — a word that starts a statement or
    /// shapes one
    Word,
    /// `Angle`, `circle`, `Tooth` — a word in the place a type is written
    Type,
    /// a constraint's name, where the statement is one: `distance`, `fix`, `point_on_circle`
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
    /// `sqrt`, `pi`, `length(…)` — what an expression knows before the document says anything:
    /// `expr`'s functions, constants and measurements
    Builtin,
    /// `"nurbs.sv"` — a quoted string, which only a drawing (`.svd`) has; Solvent has none
    Str,
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
            Tint::Builtin => "builtin",
            Tint::Str => "string",
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
    /// names a module: the dotted path after `use`
    Module,
    /// names what the statement declares
    Def,
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
    let toks = &lexed.toks;
    let known = relation_words(toks);
    let mut out: Vec<(Tint, Span)> = Vec::with_capacity(toks.len());
    let mut at = Next::Start;
    // how deep inside a `hint(…)` clause we are, and 0 outside one.  A number inside one is a
    // seed and every other number is not — the whole of §4.3's lexical rule, and the reason the
    // colouring can say which numbers a solve may rewrite without elaborating anything.
    let mut hint = 0i32;
    // and how deep inside a `style .name { … }` block, whose body is `property: value` pairs
    // rather than statements
    let mut style = 0i32;
    // how deep inside an `integral(…)`, and whether its `over` has been passed: the names after
    // it are the binders the integrand reads (`over (p, t)`)
    let mut integral = 0i32;
    let mut binding = false;
    // a relation word's definition: the head runs to its `:=`, and the word it defines is the one
    // name in it the statement declares — its operands and parameters are the body's to read
    let mut head: Option<(usize, usize)> = None;
    // a `ring` header, the one place `about` is a word rather than a name
    let mut ring = false;
    for (i, (t, span)) in toks.iter().enumerate() {
        let prev = i.checked_sub(1).map(|j| &toks[j].0);
        // a unit written against its number is one literal to the parser (`20mm`, `90deg`, `1'
        // 6"`), so it is coloured as the number it belongs to: the run is widened over it
        if let (Some(Tok::Num(_)), Some(last)) = (prev, out.last_mut()) {
            let unit = match t {
                Tok::Ident(w) => crate::units::unit(w).is_some(),
                Tok::P('\'') | Tok::P('"') => true,
                _ => false,
            };
            if unit && last.1.hi == span.lo && toks[i - 1].1.hi == span.lo {
                last.1.hi = span.hi;
                continue;
            }
        }
        match t {
            Tok::P('(') if matches!(prev, Some(Tok::Ident(w)) if w == "integral") => integral = 1,
            Tok::P('(') if integral > 0 => integral += 1,
            Tok::P(')') if integral > 0 => {
                integral -= 1;
                binding &= integral > 0;
            }
            Tok::Ident(w) if integral > 0 && w == "over" => binding = true,
            Tok::Nl => {
                integral = 0;
                binding = false;
            }
            _ => {}
        }
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
        if at == Next::Start && head.is_none() {
            head = defined_word(toks, i);
        }
        if let Some((word, end)) = head {
            if i < end {
                if i == word {
                    out.push((Tint::Def, *span));
                }
                at = Next::Word;
                continue;
            }
            head = None;
        }
        if matches!(t, Tok::Nl | Tok::P('{')) {
            ring = false;
        }
        let tint = match t {
            Tok::Nl => {
                at = Next::Start;
                continue;
            }
            Tok::Ident(w) if binding && w != "over" => Some(Tint::Def),
            Tok::Num(_) if hint > 0 => Some(Tint::Seed),
            Tok::Num(_) => Some(Tint::Num),
            // `w := 100` — a definition, and not a seed: the clause is what says a number is one.
            // What follows is read as a statement's head is: an element keyword, a call.
            Tok::Define => {
                at = Next::Start;
                continue;
            }
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
                let (tint, then) = tint_word(w, prev, toks, i, at, &known, ring);
                ring |= at == Next::Start && w == "ring";
                at = then;
                tint
            }
        };
        // anything at all leaves the opening word behind; a name the statement is still owed
        // (`Def`, `Inst`) survives the punctuation in between, which is why only `Start` lapses
        let modifier = matches!(t, Tok::Ident(w) if ["private", "construction", "centerline"].contains(&w.as_str()));
        if at == Next::Start && !matches!(t, Tok::P('{') | Tok::P('}')) && !modifier {
            at = Next::Word;
        }
        // a `style` block's body is `property: value` pairs, not statements: the brace above
        // reset the state to `Start`, where `dash:` would read as an instance
        if matches!(t, Tok::P('{')) && style > 0 {
            at = Next::Word;
        }
        // nor is a group's (`dims := {width: 20mm}`): its members are labelled values, as a
        // group's written in place as a member is (`{cyl: {bore: 16mm}}`)
        if matches!(t, Tok::P('{')) && matches!(prev, Some(Tok::Define) | Some(Tok::P(':'))) {
            at = Next::Word;
        }
        if let Some(tint) = tint {
            out.push((tint, *span));
        }
    }
    with_comments(out, lexed.comments)
}

/// The comments were never tokens; put them back where they were written.  Both runs are already
/// in order — the tokens by the walk and the comments by the lexer — so this is a merge, and
/// sorting the two together would be throwing that away and buying it back.  A drawing's
/// colouring (`drawing::highlight`) ends the same way.
pub(crate) fn with_comments(out: Vec<(Tint, Span)>, comments: Vec<Span>) -> Vec<(Tint, Span)> {
    if comments.is_empty() {
        return out;
    }
    let mut merged = Vec::with_capacity(out.len() + comments.len());
    let mut cs = comments.into_iter().peekable();
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
    known: &BTreeSet<&str>,
    ring: bool,
) -> (Option<Tint>, Next) {
    let next = toks.get(i + 1).map(|(t, _)| t);
    // `{ p | p distance(r) c }` — the point a set is made of is the name it binds (§6.21), and a
    // set may open where a statement does or where a group's list would
    if prev == Some(&Tok::P('{')) && next == Some(&Tok::P('|')) {
        return (Some(Tint::Def), Next::Word);
    }
    match at {
        Next::Unit => (Some(Tint::Type), Next::Word),
        // `use engine.parts` — each name of the path, up to the list it reads bare
        Next::Module => {
            let then = if next == Some(&Tok::P('.')) { Next::Module } else { Next::Word };
            (Some(Tint::Type), then)
        }
        Next::Class => {
            // the list runs to the next thing a declaration may say — another trailing clause,
            // or a chain's joint.  The same predicate the parser stops on, asked once.
            if trails_decl(w) {
                return tint_word(w, prev, toks, i, Next::Word, known, ring);
            }
            (Some(Tint::Class), Next::Class)
        }
        Next::Def => (Some(Tint::Def), Next::Word),
        Next::Start => {
            if ["private", "construction", "centerline"].contains(&w) {
                return (Some(Tint::Word), Next::Start);
            }
            // `param bore: Length := 50mm` — the word, then the name it declares
            if w == "param" {
                return (Some(Tint::Word), Next::Def);
            }
            // `w := 100`, `c := circle(…)` — the name a definition gives its value
            if next == Some(&Tok::Define) {
                return (Some(Tint::Def), Next::Word);
            }
            // `point hint(…)`, `line(a, b)`
            if EntKind::parse(w).is_some() {
                return (Some(Tint::Word), Next::Word);
            }
            if w == "component" {
                return (Some(Tint::Word), Next::Def);
            }
            if w == "use" {
                return (Some(Tint::Word), Next::Module);
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
            if w == "preview" || BLOCKS.contains(&w) {
                // `repeat e in rack.profile` — the edge is a name the block declares, as the
                // index after `as` is
                let over = BLOCKS.contains(&w) && over_chain(toks, i + 1);
                return (Some(Tint::Word), if over { Next::Def } else { Next::Word });
            }
            // a raw branch: a statement the parser knows by name
            if w == "branch" {
                return (Some(Tint::Relation), Next::Word);
            }
            // `flat l`, `hangs(L) rope` — a word this file defines or imports, standing before
            // its operand; a component call has no operand after its parentheses
            if known.contains(w)
                && word_at(toks, past_args(toks, i)).is_some_and(|o| !MODIFIERS.contains(&o))
            {
                return (Some(Tint::Relation), Next::Word);
            }
            if builtin(w, prev, next, false) {
                return (Some(Tint::Builtin), Next::Word);
            }
            if next == Some(&Tok::P('(')) && !is_operator(w) {
                return (Some(Tint::Type), Next::Word);
            }
            // `claim vertical(rail)`: the word after it is a statement start again, so the
            // relation it qualifies is tinted exactly as it would be standing alone
            if w == "claim" {
                return (Some(Tint::Word), Next::Start);
            }
            // the operator table, not the registry's names: `coincident` and `equal` are constraints
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
            // `(ab := line(a, b))`, `distance(w := 60)` — a name defined where it stands
            if next == Some(&Tok::Define) {
                return (Some(Tint::Def), Next::Word);
            }
            // `engine.parts.Crank(…)` — a used module's component, named by its full path
            if prev == Some(&Tok::P('.')) && next == Some(&Tok::P('(')) {
                return (Some(Tint::Type), Next::Word);
            }
            // `integral(p.y over p)` — a term of an energy (§9.10), read by the parser by name
            if w == "integral" && next == Some(&Tok::P('(')) {
                return (Some(Tint::Word), Next::Word);
            }
            // `ring N about c` — the turn's centre
            if w == "about" && ring {
                return (Some(Tint::Word), Next::Word);
            }
            // `next.v` — the copy after this one, in a block
            if matches!(w, "next" | "prev") && next == Some(&Tok::P('.')) {
                return (Some(Tint::Word), Next::Word);
            }
            // `spline(…) weights [1, w, w, 1]`, and its knots — the curve's own data
            if matches!(w, "knots" | "weights") && next == Some(&Tok::P('[')) {
                return (Some(Tint::Word), Next::Word);
            }
            // `k from p to q` — a stretch of a curve, `to` closing what `from` opened
            if w == "to" && after_from(toks, i) {
                return (Some(Tint::Word), Next::Word);
            }
            if builtin(w, prev, next, false) {
                return (Some(Tint::Builtin), Next::Word);
            }
            if (w == "cut" || w == "union" || w == "bound" || w == "minimizes" || w == "maximizes")
                && prev != Some(&Tok::P('.'))
            {
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
            let next_word = link_word(toks, j);
            // both questions off the one cursor: a line ending in a joint word continues its
            // chain onto the next, and `p distance(80)` ends a line as surely as `p equal` does.
            // A body's `}` ends a statement as a line break does (`end_of_stmt`), so a word
            // standing at one — a one-line body's trailing joint — reads the same way.
            let at_line_end =
                matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Nl | Tok::P('}')) | None);
            if opens_link(w, next_word) {
                // the element keyword names what the link declares; a prefix states a relation
                return match EntKind::parse(w) {
                    Some(_) => (Some(Tint::Word), Next::Word),
                    None => (Some(Tint::Relation), Next::Word),
                };
            }
            let at_marker = matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Arrow));
            if (next_word.is_some() || at_line_end || at_marker) && joint_word(w) {
                // `at_marker` is the far-side marker: `A -> equal -> B`
                return (Some(Tint::Relation), Next::Word);
            }
            // `centre right_of(d: 5) o` — a word this file defines or imports, between operands
            if (next_word.is_some() || at_line_end) && known.contains(w) {
                return (Some(Tint::Relation), Next::Word);
            }
            // Chains close at a line end; inside a face's list it is the marker, not the
            // closing bracket, that distinguishes `-> close` from an edge named `close`.
            if w == "close" && (at_line_end || prev == Some(&Tok::Arrow)) {
                return (Some(Tint::Word), Next::Word);
            }
            // `ratio: length(a) / length(b)` — a measurement, where no relation took the word
            (builtin(w, prev, next, true).then_some(Tint::Builtin), Next::Word)
        }
    }
}

/// What `expr` knows by name: a function or (with `measures`) a measurement called, or a
/// constant read.  The tables are `expr`'s own, so a word is coloured built in exactly when an
/// expression reads it as one.
fn builtin(w: &str, prev: Option<&Tok>, next: Option<&Tok>, measures: bool) -> bool {
    if next == Some(&Tok::P('(')) {
        FUNCTIONS.iter().any(|f| f.0 == w) || (measures && MEASURES.iter().any(|m| m.0 == w))
    } else {
        CONSTANTS.iter().any(|c| c.0 == w)
            && prev != Some(&Tok::P('.'))
            && !matches!(next, Some(Tok::P(':') | Tok::P('.') | Tok::Define))
    }
}

/// `a word(params) b :=` or `word(params) l :=` at `i`: where the word being defined is, and
/// where its head ends (the `:=`).  The parser's own lookahead, `word_definition_at`.
fn defined_word(toks: &[(Tok, Span)], i: usize) -> Option<(usize, usize)> {
    let word = match word_definition_at(toks, i)? {
        Fixity::Infix => i + 1,
        _ => i,
    };
    Some((word, past_args(toks, word) + 1))
}

/// `t.r.e from t.r.lo to t.r.hi`: whether the `to` at `i` closes a `from`, the reference
/// between them a dotted name.
fn after_from(toks: &[(Tok, Span)], i: usize) -> bool {
    let mut j = i;
    while j > 0 {
        j -= 1;
        match &toks[j].0 {
            Tok::Ident(w) if w == "from" => return j + 1 < i,
            Tok::Ident(_) | Tok::P('.') => {}
            _ => return false,
        }
    }
    false
}

/// The relation words a file may use bare: those it defines (`b right_of(d) a := …`) and those
/// its `use` lines name (`use std (right_of)`).  A text question, as the rest of the colouring is.
fn relation_words(toks: &[(Tok, Span)]) -> BTreeSet<&str> {
    let mut out = BTreeSet::new();
    let mut start = true;
    for i in 0..toks.len() {
        let here = start;
        start = matches!(toks[i].0, Tok::Nl | Tok::P('{') | Tok::P('}'));
        if !here {
            continue;
        }
        match defined_word(toks, i) {
            Some((word, _)) => out.extend(word_at(toks, word)),
            None if word_at(toks, i) == Some("use") => {
                let mut j = i + 1;
                while matches!(toks.get(j).map(|(t, _)| t), Some(Tok::Ident(_) | Tok::P('.'))) {
                    j += 1;
                }
                if toks.get(j).map(|(t, _)| t) == Some(&Tok::P('(')) {
                    j += 1;
                    while let Some((t, _)) = toks.get(j) {
                        match t {
                            Tok::Ident(w) => {
                                out.insert(w.as_str());
                            }
                            Tok::P(')') | Tok::Nl => break,
                            _ => {}
                        }
                        j += 1;
                    }
                }
            }
            None => {}
        }
    }
    out
}
