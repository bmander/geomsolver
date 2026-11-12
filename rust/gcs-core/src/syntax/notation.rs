//! **Sets in the notation of sets** (#783): the words a set is written with, lowered token by
//! token to the forms the parser reads, and the spellings they retire refused, each naming its
//! replacement.
//!
//! | written | read as |
//! |---|---|
//! | `p in S` | `p inside S`: the set's statements at `p`, as written (a region's bounds kept) |
//! | `p not in S` | `p outside S`: its complement |
//! | `p in a.side`, `p not in a.side` | `p inside a`, `p outside a`: the material side of a face |
//! | `p in a`, `a` a face | `p coincident a`: the face itself |
//! | `p in boundary(S)` | `p coincident S.surface` (of `a.side`, `a.surface`) |
//! | `boundary(S)` elsewhere | `S.surface`, as above (`a.smooth`'s itself) |
//! | `p in A or p not in B` | `p outside { p \| p outside A; p inside B }`, in a body |
//!
//! A sweep's operand that is a face (drawn, a `face` formal, a family's member read at a declared
//! parameter) is lowered with `coincident` where a body's is with `inside`, and `over t` stands in
//! its parentheses where `t` is a declared `param` (none for the whole roll).
//! | `some t in (a, b): p in X at(under: m) t` | `p inside(under: m, over t in (a, b)) X` |
//! | `some t: p in X at(under: m) t` | `p inside(under: m) X`, the whole roll |
//! | `some s: p in F at s` | `p coincident(over s) F`: a face family over a declared parameter |
//! | `some h: p in F at(along: n) h` | `p inside F`: the prism through a face |
//! | `not some h: p in F at(along: n) h` | `p outside F`: what is not in it |
//! | `some q in A, r in B: p in X` | `some q inside A, r inside B: p inside X` ([0.212]) |
//! | `p distance(…) { q \| q not in S }` | `p distance(…, side: inside) S` |
//! | `n := x at f`, `f` a frame | `n := x in f`: drawn in the jointed frame at its value |
//!
//! The relation words a lowering makes are recorded by span (`Text::words`), so the relation
//! the parser reads from them knows it was written in the set notation (`Written::lowered`): what
//! the flattener refuses of `coincident` against a set or of `side:` on one, it never refuses of
//! its own lowering.  The source is read where it stands: no span moves, and a lowered token
//! carries the span of the word it stands for.

use super::lexer::Tok;
use super::words::{word_at as word, QUALIFIERS};
use super::{Lowered, Member, Span, SynErr};
use std::collections::{BTreeMap, BTreeSet};

type T = (Tok, Span);

/// What the lowering gives back: the tokens the parser reads, how each relation word it lowered
/// was written (by where the word starts, its span widened over its parentheses as it may be),
/// and the retired spellings it met.
pub(super) struct Text {
    pub toks: Vec<T>,
    pub words: BTreeMap<u32, Lowered>,
    pub errs: Vec<SynErr>,
}

/// The names a document declares that the lowering reads by kind: its declared parameters (a
/// quantifier over one ranges its interval), its frames (`x at f` is a placement), and its faces
/// (`p in a`, `a` a face, is the face).
#[derive(Default)]
struct Kinds {
    params: BTreeSet<String>,
    frames: BTreeSet<String>,
    /// faces drawn, `a := face(…)`
    faces: BTreeSet<String>,
    /// face formals, `a: face`, each with the tokens of the definition it is a formal of
    formals: Vec<(usize, usize, String)>,
}

fn punct(t: &[T], i: usize, c: char) -> bool {
    t.get(i).map(|x| &x.0) == Some(&Tok::P(c))
}

fn ident(text: &str, span: Span) -> T {
    (Tok::Ident(text.to_string()), span)
}

fn p(c: char, span: Span) -> T {
    (Tok::P(c), span)
}

/// Whether `[a, b)` ends in `.w` with something before it: `a.side`, `a.smooth`.
fn suffix(t: &[T], a: usize, b: usize, w: &str) -> bool {
    b >= a + 3 && punct(t, b - 2, '.') && word(t, b - 1) == Some(w)
}

fn at_end(span: Span) -> Span {
    Span::at(span.hi as usize)
}

use super::words::matching as close;

/// Past a reference at `j`: a name, then `.name` and `[…]` steps.
fn past_ref(t: &[T], j: usize) -> Option<usize> {
    word(t, j)?;
    steps(t, j + 1)
}

/// Past the `.name` and `[…]` steps from `j`.
fn steps(t: &[T], mut j: usize) -> Option<usize> {
    loop {
        if punct(t, j, '.') && word(t, j + 1).is_some() {
            j += 2;
        } else if punct(t, j, '[') {
            j = close(t, j)? + 1;
        } else {
            return Some(j);
        }
    }
}

/// Past an operand at `j`: a set written in place, `boundary(…)`, a call with what follows it
/// (`Station(g, s).f`), or a reference.
fn operand_end(t: &[T], j: usize) -> Option<usize> {
    if super::words::set_at(t, j).is_some() {
        return Some(close(t, j)? + 1);
    }
    let mut k = j;
    word(t, k)?;
    k += 1;
    while punct(t, k, '.') && word(t, k + 1).is_some() {
        k += 2;
    }
    if !punct(t, k, '(') {
        return past_ref(t, j);
    }
    steps(t, close(t, k)? + 1)
}

/// Whether a statement may start at `i`: the first token, or one after a statement's end, a
/// body's opening brace or a set's bar.
fn starts(t: &[T], i: usize) -> bool {
    // a set's bound name after its `{` is no statement: its first stands after the `|`
    i == 0 || matches!(t[i - 1].0, Tok::Nl | Tok::P('|'))
        || (t[i - 1].0 == Tok::P('{') && super::words::set_at(t, i - 1).is_none())
        // a word's body written on its head's line, `a opens(h) l := a in boundary(…)`: its
        // statement after the `:=`, where more than a name stands before it
        || (t[i - 1].0 == Tok::Define && i >= 3 && !matches!(t[i - 3].0, Tok::Nl)
            && !word(t, i - 3).is_some_and(|w| QUALIFIERS.contains(&w)))
}

/// The statement's end from `i`: the line break or `;` that ends it, or the brace closing the body
/// it stands in, at its own depth.
fn statement_end(t: &[T], i: usize) -> usize {
    let mut depth = 0i32;
    let mut k = i;
    while k < t.len() {
        match t[k].0 {
            Tok::P('(' | '[' | '{') => depth += 1,
            Tok::P(')' | ']' | '}') => {
                if depth == 0 {
                    return k;
                }
                depth -= 1;
            }
            Tok::Nl if depth == 0 => return k,
            _ => {}
        }
        k += 1;
    }
    k
}

/// The open bracket each token stands inside, the innermost.
fn enclosing(t: &[T]) -> Vec<Option<usize>> {
    let mut stack: Vec<usize> = Vec::new();
    let mut out = Vec::with_capacity(t.len());
    for (k, (tok, _)) in t.iter().enumerate() {
        out.push(stack.last().copied());
        match tok {
            Tok::P('(' | '[' | '{') => stack.push(k),
            Tok::P(')' | ']' | '}') => {
                stack.pop();
            }
            _ => {}
        }
    }
    out
}

impl Kinds {
    /// Whether `name`, read at token `at`, is a face: drawn, or a formal of the definition `at`
    /// stands in.
    fn face(&self, name: &str, at: usize) -> bool {
        self.faces.contains(name)
            || self.formals.iter().any(|(lo, hi, n)| n == name && *lo <= at && at < *hi)
    }

    fn of(t: &[T]) -> Kinds {
        let mut k = Kinds::default();
        for i in 0..t.len() {
            match (word(t, i), t.get(i + 1).map(|x| &x.0), word(t, i + 2)) {
                (Some("param"), _, _) => {
                    if let Some(n) = word(t, i + 1) {
                        k.params.insert(n.to_string());
                    }
                }
                (Some(n), Some(Tok::Define), Some("frame")) => {
                    k.frames.insert(n.to_string());
                }
                (Some(n), Some(Tok::Define), Some("face")) if punct(t, i + 3, '(') => {
                    k.faces.insert(n.to_string());
                }
                // a formal, `name: frame` or `name: face`, in a component's or a word's header
                (Some(n), Some(Tok::P(':')), Some("frame")) => {
                    k.frames.insert(n.to_string());
                }
                (Some(n), Some(Tok::P(':')), Some("face")) => {
                    // the definition the header opens: to the brace closing its body
                    let body = (i..t.len()).find(|&j| punct(t, j, '{'));
                    let end = body.and_then(|j| close(t, j)).unwrap_or(t.len());
                    k.formals.push((i, end, n.to_string()));
                }
                _ => {}
            }
        }
        k
    }
}

/// How an operand of `in` reads once lowered.
#[derive(Clone, Copy, PartialEq)]
enum Reads {
    /// a set, a solid or anything else: `inside` / `outside` it
    Set,
    /// a face's `.side`: the side of its surface its material is on
    Side,
    /// `boundary(S)`: on the surface
    Boundary,
    /// a face (`a`, a face formal or a drawn face): on the face
    Face,
}

struct Lowering<'a> {
    t: &'a [T],
    kinds: Kinds,
    encl: Vec<Option<usize>>,
    words: BTreeMap<u32, Lowered>,
    errs: Vec<SynErr>,
}

/// Lower the set notation in `toks` and refuse the spellings it retires (#783).
pub(super) fn lower(toks: Vec<T>) -> Text {
    let mut l = Lowering {
        t: &toks,
        kinds: Kinds::of(&toks),
        encl: enclosing(&toks),
        words: BTreeMap::new(),
        errs: Vec::new(),
    };
    l.retired();
    let mut out = Vec::with_capacity(toks.len());
    l.range(0, toks.len(), &mut out);
    let Lowering { words, errs, .. } = l;
    Text { toks: out, words, errs }
}

impl Lowering<'_> {
    fn fail(&mut self, span: Span, message: impl Into<String>) {
        self.errs.push(SynErr { span, message: message.into() });
    }

    /// The relation word lowered at `at`, its operand read as `reads`: a face's side marks it
    /// so, whatever else is marked at the same place.
    fn mark(&mut self, at: Span, reads: Reads) {
        let w = self.words.entry(at.lo).or_insert(Lowered::Set);
        if reads == Reads::Side {
            *w = Lowered::Side;
        }
    }

    /// The spellings #783 retires, refused where they are written.
    fn retired(&mut self) {
        let t = self.t;
        for i in 0..t.len() {
            let span = t[i].1;
            match word(t, i) {
                // `inside` and `outside` as a relation's word: after an operand, before one
                // the claim word `inside` between two solids (§9.8), one within another, is
                // what holds of every point of the one (#783 P3)
                Some("inside") if self.claimed(i) => {
                    self.fail(span, "`claim a inside b` is retired (#783): one solid within \
                        another is what holds of every point of it, `claim every q in a: q in b`");
                }
                Some(w @ ("inside" | "outside")) if i > 0
                    && matches!(t[i - 1].0, Tok::Ident(_) | Tok::P(')' | ']'))
                    && matches!(t.get(i + 1).map(|x| &x.0),
                                Some(Tok::Ident(_) | Tok::P('(' | '{'))) =>
                {
                    let m = if w == "inside" {
                        "`inside` is retired (#783): a point in a set is `p in S` — the set's own \
                         statements at it, a region's bounds as written (`std.sets.Ball(c, r)`, \
                         not a sphere's radius read as one); on a face's material side, `p in \
                         a.side`; swept, `some t in (a, b): p in X at(under: m) t`"
                    } else {
                        "`outside` is retired (#783): a point not in a set is `p not in S`; off \
                         a face's material side, `p not in a.side`"
                    };
                    self.fail(span, m);
                }
                Some("surface") if i > 0 && punct(t, i - 1, '.') => {
                    self.fail(span, "`.surface` is retired (#783): the boundary of a set is \
                        `boundary(S)` — of a face's material side, `boundary(a.side)`, the \
                        surface the face lies on");
                }
                Some("over") if self.encl[i]
                    .is_some_and(|o| o > 0 && word(t, o - 1) == Some("coincident")) =>
                {
                    self.fail(span, "`over` in a relation's parentheses is retired (#783): the \
                        points a face occupies at some value are `some s: p in F at s`, and \
                        under a motion `some t in (a, b): p in F at(under: m) t`");
                }
                _ => {}
            }
        }
    }

    /// Whether the token at `i` stands in a claim: its statement opens with `claim`.
    fn claimed(&self, i: usize) -> bool {
        let t = self.t;
        let start = (0..i).rev().find(|&k| starts(t, k)).unwrap_or(0);
        // or in a claim's block, `claim over r in (a, b) { a inside b }`
        word(t, start) == Some("claim")
            || self.encl[start].is_some_and(|o| punct(t, o, '{') && self.claimed(o))
    }

    /// Lower the tokens `[lo, hi)` into `out`.
    fn range(&mut self, lo: usize, hi: usize, out: &mut Vec<T>) {
        let t = self.t;
        let mut i = lo;
        while i < hi {
            if starts(t, i) {
                if let Some(next) = self.statement(i, hi, out) {
                    i = next;
                    continue;
                }
            }
            if let Some(next) = self.boundary(i, out) {
                i = next;
                continue;
            }
            if let Some(next) = self.distance(i, out) {
                i = next;
                continue;
            }
            out.push(t[i].clone());
            i += 1;
        }
    }

    /// A statement's form at `i`, lowered through the operand its word reads: `None` where it is
    /// none the notation lowers.
    fn statement(&mut self, i: usize, hi: usize, out: &mut Vec<T>) -> Option<usize> {
        let t = self.t;
        match word(t, i)? {
            "claim" if word(t, past_ref(t, i + 1).unwrap_or(i + 1)) == Some("inside") => None,
            w if (w == "claim" || QUALIFIERS.contains(&w)) && i + 1 < hi => {
                let mut held = Vec::new();
                let next = self.statement(i + 1, hi, &mut held)?;
                out.push(t[i].clone());
                out.extend(held);
                Some(next)
            }
            // a condition is the expression reader's (`expr::parse_cond`): `when w not in A`
            "when" | "repeat" | "cycle" | "claim_over" => None,
            // more of a body's set, `B or { p | p in x }` (#783 P3): the set's own statements
            // are lowered where they stand, past its `|`
            _ if super::words::joined_at(t, i) => None,
            "some" => self.some(i, false, out),
            // what holds of every point of a solid (#783 P3): claimed, one solid within another
            "every" if word(t, i + 1).is_some() => Some(self.every(i, out)),
            // the complement of the prism through a face, `not some h: p in F at(along: n) h`
            "not" if word(t, i + 1) == Some("some") => self.some(i + 1, true, out),
            _ => self.placement(i, out).or_else(|| self.membership(i, hi, out)).or_else(|| {
                // `or` joins memberships, and nothing else
                // to a `:=`, past which a word's body is a statement of its own (`starts`)
                let end = statement_end(t, i);
                let depth0 = |k: usize| self.encl[k] == self.encl[i];
                let end = (i..end).find(|&k| t[k].0 == Tok::Define && depth0(k)).unwrap_or(end);
                let or = (i..end).find(|&k| word(t, k) == Some("or") && depth0(k))?;
                self.fail(t[or].1, "each side of `or` is a membership of the set's point: `p in A \
                    or p not in B`; a bound is a set of its own, `{ q | q distance(<= r) c }`");
                Some(end)
            }),
        }
    }

    /// The operand at `[a, b)` lowered: `boundary(S)` as `S.surface`, `a.side` as `a`, anything
    /// else as its own tokens (a set written in place lowered through) — and how `in` reads it.
    fn operand(&mut self, a: usize, b: usize, out: &mut Vec<T>) -> Reads {
        let t = self.t;
        if word(t, a) == Some("boundary") && punct(t, a + 1, '(') {
            if let Some(c) = close(t, a + 1).filter(|&c| c + 1 == b) {
                self.surface_of(a + 2, c, t[a].1, out);
                return Reads::Boundary;
            }
        }
        if suffix(t, a, b, "side") {
            self.range(a, b - 2, out);
            return Reads::Side;
        }
        let bare = b == a + 1 && word(t, a).is_some_and(|w| self.kinds.face(w, a));
        self.range(a, b, out);
        if bare { Reads::Face } else { Reads::Set }
    }

    /// `boundary(S)`'s operand `[a, b)` as the surface it names: `S.surface`, a face's material
    /// side's `a.surface`, and `a.smooth` as itself (the face's surface carried over its seams).
    fn surface_of(&mut self, a: usize, b: usize, at: Span, out: &mut Vec<T>) {
        let t = self.t;
        let first = out.len();
        // a set named, or one written in place by a call (`boundary(std.sets.Cone(l, half: h))`)
        if operand_end(t, a) != Some(b) {
            self.fail(at, "`boundary(…)` takes a set, a solid or a face's side, by name or by a \
                call: `boundary(S)`, `boundary(a.side)`, `boundary(std.sets.Ball(c, r: 5mm))`");
        }
        let end = t[b.saturating_sub(1).max(a)].1;
        if suffix(t, a, b, "side") {
            out.extend(t[a..b - 1].iter().cloned());
            out.push(ident(Member::Surface.word(), t[b - 1].1));
        } else if suffix(t, a, b, Member::Smooth.word()) {
            out.extend(t[a..b].iter().cloned());
        } else {
            out.extend(t[a..b].iter().cloned());
            out.push(p('.', at_end(end)));
            out.push(ident(Member::Surface.word(), at_end(end)));
        }
        // the reference spans the whole of `boundary(…)`, where it was written
        if let Some(close) = t.get(b).filter(|x| x.0 == Tok::P(')')) {
            if let Some(x) = out.get_mut(first) {
                x.1 = Span::new(at.lo as usize, x.1.hi as usize);
            }
            if let Some(x) = out.last_mut() {
                x.1 = Span::new(x.1.lo as usize, close.1.hi as usize);
            }
        }
    }

    /// `every q in A: q in B`, claimed (#783 P3): every point of the solid `A` in `B`, one solid
    /// within another — the claim the solid claims judge (§9.8, `A inside B`), measured and never
    /// acting.  Read in a claim only, where it states nothing: a statement that every value of a
    /// running number, or every acceptable part, satisfies something is the tolerances' (#783).
    fn every(&mut self, i: usize, out: &mut Vec<T>) -> usize {
        let t = self.t;
        let end = statement_end(t, i);
        let shape = "`every` reads one solid within another, claimed: `claim every q in a: q in b`";
        if !self.claimed(i) {
            self.fail(t[i].1, "`every` states what holds of every point of a solid, and is read \
                in a claim for now: `claim every q in a: q in b`, one solid within another");
            return end;
        }
        let parsed = (|| {
            let q = word(t, i + 1)?;
            (word(t, i + 2) == Some("in")).then_some(())?;
            let a1 = past_ref(t, i + 3)?;
            punct(t, a1, ':').then_some(())?;
            (word(t, a1 + 1) == Some(q) && word(t, a1 + 2) == Some("in")).then_some(())?;
            let b1 = past_ref(t, a1 + 3)?;
            (b1 == end).then_some((a1, b1))
        })();
        let Some((a1, b1)) = parsed else {
            self.fail(t[i].1, shape);
            return end;
        };
        // `A inside B`, the claim word, its span the membership's `in`, marked so the printer
        // writes the claim back as written
        out.extend(t[i + 3..a1].iter().cloned());
        self.words.insert(t[a1 + 2].1.lo, Lowered::Every);
        out.push(ident("inside", t[a1 + 2].1));
        out.extend(t[a1 + 3..b1].iter().cloned());
        b1
    }

    /// `boundary(S)` standing as any relation's operand.
    fn boundary(&mut self, i: usize, out: &mut Vec<T>) -> Option<usize> {
        let t = self.t;
        if word(t, i) != Some("boundary") || !punct(t, i + 1, '(')
            || (i > 0 && punct(t, i - 1, '.'))
        {
            return None;
        }
        let c = close(t, i + 1)?;
        self.surface_of(i + 2, c, t[i].1, out);
        Some(c + 1)
    }

    /// `p distance(…) { q | q not in S }`: the distance from what is not in `S`, which the
    /// evaluator reads as the distance from `S`'s surface on its inside.
    fn distance(&mut self, i: usize, out: &mut Vec<T>) -> Option<usize> {
        let t = self.t;
        if word(t, i) != Some("distance") || i == 0 || !punct(t, i + 1, '(') {
            return None;
        }
        let c = close(t, i + 1)?;
        let bound = super::words::set_at(t, c + 1)?;
        let q = word(t, bound)?;
        let end = close(t, c + 1)?;
        // the body: `q not in S`, one statement
        let mut k = bound + 2;
        while matches!(t.get(k).map(|x| &x.0), Some(Tok::Nl)) {
            k += 1;
        }
        if word(t, k) != Some(q) || word(t, k + 1) != Some("not") || word(t, k + 2) != Some("in") {
            return None;
        }
        let s_end = past_ref(t, k + 3)?;
        if (s_end..end).any(|j| t[j].0 != Tok::Nl) {
            return None;
        }
        self.mark(t[i].1, Reads::Set);
        out.push(t[i].clone());
        out.extend(t[i + 1..c].iter().cloned());
        let at = Span::at(t[c].1.lo as usize);
        if c > i + 2 {
            out.push(p(',', at));
        }
        out.push(ident("side", at));
        out.push(p(':', at));
        out.push(ident("inside", at));
        out.push(t[c].clone());
        let mut s = Vec::new();
        self.operand(k + 3, s_end, &mut s);
        if let Some(last) = s.last_mut() {
            last.1 = Span::new(last.1.lo as usize, t[end].1.hi as usize);
        }
        out.extend(s);
        Some(end + 1)
    }

    /// `n := x at f`, `f` a frame: `x` drawn in the jointed frame at its value; and `n := x in f`
    /// refused, naming it.
    fn placement(&mut self, i: usize, out: &mut Vec<T>) -> Option<usize> {
        let t = self.t;
        if word(t, i).is_none() || t.get(i + 1).map(|x| &x.0) != Some(&Tok::Define) {
            return None;
        }
        let r = i + 2;
        let first = word(t, r)?;
        if crate::model::EntKind::parse(first).is_some() {
            return None;
        }
        let e = past_ref(t, r)?;
        let ends = |k: usize| k >= t.len() || matches!(t[k].0, Tok::Nl | Tok::P('}'));
        match word(t, e) {
            Some("in") if word(t, e + 1).is_some() && past_ref(t, e + 1).is_some_and(ends)
                && !punct(t, e, '(') =>
            {
                self.fail(t[e].1, "`x in f` is retired as a placement (#783): `in` is \
                    membership; drawn in a jointed frame at its value is `n := x at f`");
                None
            }
            Some("at") => {
                let f = word(t, e + 1)?;
                let past = past_ref(t, e + 1)?;
                if !self.kinds.frames.contains(f) || past != e + 2 || !ends(past) {
                    return None;
                }
                out.extend(t[i..e].iter().cloned());
                out.push(ident("in", t[e].1));
                out.push(t[e + 1].clone());
                Some(e + 2)
            }
            _ => None,
        }
    }

    /// `p in X`, `p not in X`, and memberships joined by `or` in a set's body.
    fn membership(&mut self, i: usize, hi: usize, out: &mut Vec<T>) -> Option<usize> {
        let t = self.t;
        // a declaration's element word is no point: `p := point in P` draws one in a plane
        if word(t, i).is_some_and(|w| crate::model::EntKind::parse(w).is_some()) {
            return None;
        }
        let (point, x0, not, at) = self.member_word(i)?;
        let x1 = operand_end(t, x0)?;
        if word(t, x1) == Some("or") {
            return self.disjunction(i, point, hi, out);
        }
        let mut x = Vec::new();
        let reads = self.operand(x0, x1, &mut x);
        let lowered = match (reads, not) {
            (Reads::Set | Reads::Side, false) => "inside",
            (Reads::Set | Reads::Side, true) => "outside",
            (Reads::Boundary | Reads::Face, false) => "coincident",
            (Reads::Boundary, true) => {
                self.fail(at, "a set's boundary has no inside, so no point is outside it: \
                    `p not in S` is outside the set itself");
                "outside"
            }
            (Reads::Face, true) => {
                self.fail(at, "a face has no inside, so no point is outside it: off the side \
                    its material is on is `p not in a.side`");
                "outside"
            }
        };
        out.extend(t[i..point].iter().cloned());
        self.mark(at, reads);
        out.push(ident(lowered, at));
        out.extend(x);
        Some(x1)
    }

    /// A membership's point and word from `i`: past the point, where its operand starts, whether
    /// it was `not in`, and the word's span.
    fn member_word(&self, i: usize) -> Option<(usize, usize, bool, Span)> {
        let t = self.t;
        let point = past_ref(t, i)?;
        match (word(t, point), word(t, point + 1)) {
            (Some("in"), _) => Some((point, point + 1, false, t[point].1)),
            (Some("not"), Some("in")) => {
                let at = Span::new(t[point].1.lo as usize, t[point + 1].1.hi as usize);
                Some((point, point + 2, true, at))
            }
            _ => None,
        }
    }

    /// `p in A or p not in B …` in a set's body: the outside of where each fails (De Morgan),
    /// `p outside { p | p outside A; p inside B }`, which the evaluator reads as the union.
    fn disjunction(&mut self, i: usize, point: usize, hi: usize, out: &mut Vec<T>)
        -> Option<usize> {
        let t = self.t;
        let in_set = self.encl[i].is_some_and(|o| super::words::set_at(t, o).is_some());
        let mut parts: Vec<(usize, usize, bool, Span)> = Vec::new();
        let mut k = i;
        loop {
            let Some((pt, x0, not, at)) = self.member_word(k) else {
                self.fail(t[k.min(hi.saturating_sub(1))].1, "each side of `or` is a membership \
                    of the set's point: `p in A or p not in B`");
                return None;
            };
            if t[k..pt].iter().map(|x| &x.0).ne(t[i..point].iter().map(|x| &x.0)) {
                self.fail(t[k].1, "each side of `or` is a membership of the set's own point");
                return None;
            }
            let x1 = operand_end(t, x0)?;
            parts.push((x0, x1, not, at));
            if word(t, x1) != Some("or") {
                break;
            }
            k = x1 + 1;
        }
        let (first, last) = (parts[0].3, t[parts.last()?.1 - 1].1);
        if !in_set {
            self.fail(first, "`or` joins what a set's point satisfies, in its body: a drawn \
                point in one set or another is a choice of root, said by a bound or a word");
        }
        // the outer relation: outside the set of where every membership fails
        out.extend(t[i..point].iter().cloned());
        self.mark(first, Reads::Set);
        out.push(ident("outside", first));
        out.push(p('{', Span::at(first.hi as usize)));
        out.extend(t[i..point].iter().cloned());
        out.push(p('|', Span::at(first.hi as usize)));
        for (n, &(x0, x1, not, at)) in parts.iter().enumerate() {
            if n > 0 {
                out.push((Tok::Nl, Span::at(at.lo as usize)));
            }
            out.extend(t[i..point].iter().cloned());
            let mut x = Vec::new();
            let reads = self.operand(x0, x1, &mut x);
            if !matches!(reads, Reads::Set | Reads::Side) {
                self.fail(at, "`or` joins memberships of sets with an inside: a region, a \
                    solid or a face's side");
            }
            self.mark(at, reads);
            out.push(ident(if not { "inside" } else { "outside" }, at));
            out.extend(x);
        }
        out.push(p('}', at_end(last)));
        Some(parts.last()?.1)
    }

    /// `some q in A, r in B: p in X` ([0.212]): the points `p` some point of each set puts in `X`
    /// — each binder's membership lowered as a membership is (`inside`, or `coincident` with a
    /// face or a boundary), the body lowered as a statement is; the parser reads the quantifier
    /// (`P::points`).  One quantifier binds every point: a body that is another is refused.
    fn points(&mut self, i: usize, out: &mut Vec<T>) -> usize {
        let t = self.t;
        let mut end = statement_end(t, i);
        let shape = "`some` binds points of sets before the membership it reads them in: `some q \
            in A, r in B: p in Segment(q, r)`";
        out.push(t[i].clone());
        let mut k = i + 1;
        loop {
            let (Some(_), Some("in")) = (word(t, k), word(t, k + 1)) else {
                self.fail(t[k.min(t.len() - 1)].1, shape);
                return end;
            };
            let Some(x1) = operand_end(t, k + 2).filter(|&x1| x1 < end) else {
                self.fail(t[k + 1].1, shape);
                return end;
            };
            let mut x = Vec::new();
            let reads = self.operand(k + 2, x1, &mut x);
            let at = t[k + 1].1;
            let lowered = match reads {
                Reads::Boundary | Reads::Face => "coincident",
                _ => "inside",
            };
            out.push(t[k].clone());
            self.mark(at, reads);
            out.push(ident(lowered, at));
            out.extend(x);
            out.push(t[x1].clone());
            // the quantifier runs on past a line break after a binder's `,` or its `:`, as a
            // list in parentheses does: the breaks dropped, so the parser reads one statement
            let mut next = x1 + 1;
            while matches!(t.get(next).map(|x| &x.0), Some(Tok::Nl)) {
                next += 1;
            }
            if next > x1 + 1 {
                end = statement_end(t, next);
            }
            let more = punct(t, x1, ',');
            if !more && !punct(t, x1, ':') {
                self.fail(t[x1].1, shape);
                return end;
            }
            k = next;
            if !more {
                break;
            }
        }
        if word(t, k) == Some("some") {
            self.fail(t[k].1, "one `some` binds every point its membership reads: `some q in A, \
                r in B: p in X`");
            return end;
        }
        // a membership lowered; any other relation (a ball about a bound point) as written
        let next = self.statement(k, end, out).unwrap_or(k);
        self.range(next, end, out);
        end
    }

    /// `some t [in (a, b)]: p in X at(…) t`: the points `X` occupies at some instant — a sweep, a
    /// face family, a face turned, or the prism through a face.
    fn some(&mut self, i: usize, not: bool, out: &mut Vec<T>) -> Option<usize> {
        let t = self.t;
        // some point of a set ([0.212]): `in` a set, where a running number's is an interval
        if word(t, i + 2) == Some("in") && !punct(t, i + 3, '(') && !punct(t, i + 3, '[') {
            if not {
                self.fail(t[i].1, "`not some` reads the complement of the prism through a face, \
                    `not some h: p in F at(along: n) h`; what no point of a set reaches is `p not \
                    in S` of the set `S` the quantifier makes");
                return Some(statement_end(t, i));
            }
            return Some(self.points(i, out));
        }
        let shape = "`some` reads one instant of a running number: `some t in (a, b): p in X \
            at(under: m) t`, `some s: p in F at s` or `some h: p in F at(along: n) h`";
        let Some(var) = word(t, i + 1) else {
            self.fail(t[i].1, shape);
            return Some(statement_end(t, i));
        };
        let mut k = i + 2;
        let mut interval = None;
        if word(t, k) == Some("in") && punct(t, k + 1, '[') {
            self.fail(t[k + 1].1, "an interval is `(a, b)`, closed at both ends, as `over t in \
                (a, b)` and `distance(in: (a, b))` write it: `[a, b]` is a list, and `t in [a, b]` \
                its two members");
            return Some(statement_end(t, i));
        }
        if word(t, k) == Some("in") && punct(t, k + 1, '(') {
            let c = close(t, k + 1)?;
            interval = Some((k, c + 1));
            k = c + 1;
        }
        let parsed = (|| {
            if !punct(t, k, ':') {
                return None;
            }
            let point = past_ref(t, k + 1)?;
            if word(t, point) != Some("in") {
                return None;
            }
            let x0 = point + 1;
            let x1 = operand_end(t, x0)?;
            if word(t, x1) != Some("at") {
                return None;
            }
            let (labels, v) = if punct(t, x1 + 1, '(') {
                let c = close(t, x1 + 1)?;
                (Some((x1 + 2, c)), c + 1)
            } else {
                (None, x1 + 1)
            };
            Some((point, x0, x1, labels, v))
        })();
        let Some((point, x0, x1, labels, v)) = parsed else {
            self.fail(t[i].1, shape);
            return Some(statement_end(t, i));
        };
        if word(t, v) != Some(var) {
            self.fail(t.get(v).map_or(t[i].1, |x| x.1), format!("the instant `{var}` ranges \
                over is the one `X` is read at: `some {var}: p in X at(…) {var}`"));
            return Some(statement_end(t, i));
        }
        let end = v + 1;
        let mut x = Vec::new();
        let reads = self.operand(x0, x1, &mut x);
        let lo = Span::new(t[i].1.lo as usize, t[k + 1].1.hi as usize);
        let in_at = t[point].1;
        let mut emitted: Vec<T> = Vec::new();
        // the point, its span reaching back over the quantifier
        emitted.extend(t[k + 1..point].iter().cloned());
        if let Some(first) = emitted.first_mut() {
            first.1 = Span::new(lo.lo as usize, first.1.hi as usize);
        }
        let along = labels.is_some_and(|(a, b)| word(t, a) == Some("along") && b == a + 3);
        if along {
            // the prism through a face: its section slid along its normal, the whole way
            let (a, _) = labels?;
            if word(t, a + 2) != Some("n") {
                self.fail(t[a + 2].1, "a face is slid along its own normal, `at(along: n) h`");
            }
            if let Some((ia, _)) = interval {
                self.fail(t[ia].1, "a face slid over an interval is a prism: \
                    `std.solids.Prism(F, from: a, to: b)`, its caps `near` and `far`");
            }
            self.mark(in_at, Reads::Set);
            emitted.push(ident(if not { "outside" } else { "inside" }, in_at));
        } else {
            if not {
                self.fail(t[i].1, "`not some` reads the complement of the prism through a face, \
                    `not some h: p in F at(along: n) h`: what a sweep leaves is the body it is \
                    cut from, `swept cut body`");
            }
            // a face (drawn, a formal, a family's member) is swept as a face is; a body as a body
            let member = (x0..x1).any(|k| punct(t, k, '(')) && punct(t, x1.saturating_sub(2), '.');
            let face = reads == Reads::Face || (labels.is_none() && member);
            let lowered = if face { "coincident" } else { "inside" };
            self.mark(in_at, Reads::Set);
            emitted.push(ident(lowered, in_at));
            let at = t[x1].1;
            emitted.push(p('(', at));
            let mut items = 0;
            if let Some((a, b)) = labels {
                emitted.extend(t[a..b].iter().cloned());
                items += 1;
            }
            let over = interval.is_some() || self.kinds.params.contains(var) || labels.is_none();
            if over {
                if items > 0 {
                    emitted.push(p(',', at));
                }
                emitted.push(ident("over", t[i].1));
                emitted.push(t[i + 1].clone());
                if let Some((ia, ib)) = interval {
                    emitted.extend(t[ia..ib].iter().cloned());
                }
                items += 1;
            }
            if items == 0 {
                // `p inside() X` reads no frame: the whole roll needs a motion
                self.fail(t[x1].1, "the whole roll is read under a motion: `some t: p in X \
                    at(under: m) t`");
            }
            emitted.push(p(')', at));
        }
        emitted.extend(x);
        if let Some(last) = emitted.last_mut() {
            last.1 = Span::new(last.1.lo as usize, t[v].1.hi as usize);
        }
        if not {
            if let Some(first) = emitted.first_mut() {
                first.1 = Span::new(t[i - 1].1.lo as usize, first.1.hi as usize);
            }
        }
        out.extend(emitted);
        Some(end)
    }
}
