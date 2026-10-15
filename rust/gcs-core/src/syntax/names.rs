//! Name, reference, and numeric source spellings.

use super::{DeclName, Name, Ref, Seg, Span};
use crate::model::{EntKind, EntRef};

/// What the language calls an entity: the first letter of its kind and its index — `p0`, `l1`,
/// `c0`, `a2`, `s0`, `e0`.  The six initials are distinct, which is the whole of why this works.
///
/// Lowercase, unlike `io::entity_name`'s `P0`: that one labels a thing on a drawing, this one is
/// an identifier in a program, and the two are read in different places.
pub fn entity_name(e: EntRef) -> String {
    format!("{}{}", kind_initial(e.kind), e.idx)
}

/// Prefix for generated entity names. Distinguish kinds whose keywords share an initial.
pub fn kind_initial(k: EntKind) -> char {
    match k {
        EntKind::Plane => 'v',
        EntKind::Curve => 'k',
        // `face` and `solid` share an `s` with `spline`: a face's labels run `f0`, and a
        // solid's `b0` — a *body*, since `s` is taken and a solid is what a body is
        EntKind::Face => 'f',
        EntKind::Solid => 'b',
        EntKind::Surface => 't',
        EntKind::Motion => 'm',
        EntKind::Envelope => 'e',
        EntKind::Patch => 'h',
        EntKind::Seam => 'j',
        EntKind::Vertex => 'w',
        EntKind::Edge => 'k',
        EntKind::Meshing => 'g',
        // a body traced in space: `t` and `l` are taken, so its `r`
        EntKind::Traced => 'r',
        // `a` is the arc's: an aXis
        EntKind::Axis => 'x',
        EntKind::Point | EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline => {
            k.as_str().chars().next().expect("every kind name has a letter")
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
/// How a message spells a declaration whose name is optional: the name where the source wrote
/// one, and the bare kind where it did not.  Both the parser's errors and the elaborator's
/// diagnostics ask, so an anonymous declaration is described one way and never by its key —
/// `written`, not `shown`, because a block copy's prefixed name is no better in a message than
/// the bare kind and used to be refused here by the same character test the printer ran.
pub(crate) fn decl_head(kind: EntKind, name: &DeclName) -> String {
    match name.written() {
        Some(n) => format!("{} := {}", n.text, kind.as_str()),
        None => kind.as_str().to_string(),
    }
}

/// Internal resolution keys contain `#`, which identifiers cannot contain.
/// Use `DeclName` when available to distinguish copies from anonymous declarations.
pub fn hidden(name: &str) -> bool {
    name.contains('#')
}

/// A reference respelled under another root — `s.p1` becomes `next.s.p1` — the spelling of a
/// name that crosses a block's copy seam, written here and read by the flattener's weld so the
/// two cannot come to address the seam differently.
pub(crate) fn under_root(root: &str, r: &Ref, span: Span) -> Ref {
    let mut path = vec![Seg::Field(r.root.clone())];
    path.extend(r.path.iter().cloned());
    Ref { root: Name { text: root.to_string(), span }, path, span }
}

/// Where an entity kind stands in phase 2's build order: per kind in `EntKind` declaration
/// order (`primitives()` order), then in statement order.  The kind half of `builds_first`,
/// shared with the flattener's weld so the two cannot come to order one pair differently.
pub(crate) fn build_rank(k: EntKind) -> usize {
    k as usize
}

/// Whether two references name the same thing — the comparison the two sides of a joint are
/// held to.  Not `==`: a `Ref` carries the span it was written at, so the same name written in
/// two places is two unequal values.
pub(super) fn refs_eq(a: &Ref, b: &Ref) -> bool {
    a.root.text == b.root.text
        && a.path.len() == b.path.len()
        && a.path.iter().zip(&b.path).all(|(x, y)| match (x, y) {
            (Seg::Field(f), Seg::Field(g)) => f.text == g.text,
            (Seg::Index(i), Seg::Index(j)) => i == j,
            _ => false,
        })
}

pub(super) fn write_ref(out: &mut String, r: &Ref) {
    out.push_str(&r.root.text);
    for seg in &r.path {
        match seg {
            Seg::Field(f) => {
                out.push('.');
                out.push_str(&f.text);
            }
            Seg::Index(t) => out.push_str(&format!("[{t}]")),
            Seg::Member(m) => {
                out.push('.');
                out.push_str(m.word());
            }
        }
    }
}

/// A reference as written, for a message about it — and for a writeback that has to spell a
/// reference the source never wrote (a chain-minted `l1.p2`).
pub(crate) fn ref_text(r: &Ref) -> String {
    let mut s = String::new();
    write_ref(&mut s, r);
    s
}

/// **A name a text reads**, as the lexer sees it: `std.front` is one use of two parts, each with
/// where it stands.  What `name_uses` hands back.
#[derive(Clone, Debug, PartialEq)]
pub struct NameUse {
    /// The dotted parts as written, `std` then `front`, each at its span.
    pub parts: Vec<(String, Span)>,
    /// Whether a `(` follows: a call, `std.Polygon(…)`, which names no entity.
    pub called: bool,
}

/// Every name a text reads, in order: an identifier and the fields dotted after it.  Not a field
/// (one after a `.`), a label (one before a `:`), the name a `:=` defines, nor a unit (one
/// standing against the number before it).  Lexical, so it asks nothing of what the names
/// are: a keyword or a relation word comes back too, and a caller matching against names it
/// knows is what makes the answer mean anything.  Spans are offsets into `text`.  What
/// `edit::make_component` reads both the statements it moves and the ones it leaves with.
pub fn name_uses(text: &str) -> Vec<NameUse> {
    use super::lexer::{lex, Tok};
    let toks = lex(text).0.toks;
    let ident = |k: usize| match toks.get(k) {
        // a shorthand is read as its word (`@` is `coincident`), and is no name either way
        Some((Tok::Ident(s), at)) if at.len() as usize == s.len() => Some((s.clone(), *at)),
        _ => None,
    };
    let mut out = Vec::new();
    let mut k = 0;
    while k < toks.len() {
        let Some(first) = ident(k) else {
            k += 1;
            continue;
        };
        let field = k > 0 && toks[k - 1].0 == Tok::P('.');
        let unit = k > 0 && matches!(toks[k - 1], (Tok::Num(_), at) if at.hi == first.1.lo);
        let mut parts = vec![first];
        k += 1;
        while toks.get(k).is_some_and(|t| t.0 == Tok::P('.')) {
            match ident(k + 1) {
                Some(part) => {
                    parts.push(part);
                    k += 2;
                }
                None => break,
            }
        }
        let next = toks.get(k).map(|t| &t.0);
        if field || unit || matches!(next, Some(Tok::P(':')) | Some(Tok::Define)) {
            continue;
        }
        out.push(NameUse { parts, called: next == Some(&Tok::P('(')) });
    }
    out
}
