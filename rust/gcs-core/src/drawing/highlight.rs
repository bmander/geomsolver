//! Colouring a drawing (`.svd`): the drawing lexer's own scan, as `syntax::highlight` is the
//! model parser's, so a colour and the drawing parser cannot disagree about what a word is.

use super::parser::{scan, Token};
use crate::syntax::{with_comments, Span, Tint, MAX_TEXT};

/// The words a drawing is written in: its statements, a sheet's items and the words inside them.
const WORDS: [&str; 18] = [
    "model", "use", "style", "sheet", "from", "size", "scale", "view", "section", "sketch",
    "dimensions", "dimension", "measure", "label", "at", "cut", "in", "offset",
];

/// The words after which the next one is the name the statement declares.
const NAMING: [&str; 5] = ["model", "sheet", "view", "section", "sketch"];

/// Colour a drawing.  Spans in order over the classified runs, as `syntax::highlight` gives
/// them; never fails, since a drawing half-typed is the one being looked at.
pub fn highlight(src: &str) -> Vec<(Tint, Span)> {
    if src.len() > MAX_TEXT {
        return Vec::new();
    }
    let (toks, comments, _) = scan(src);
    let mut out: Vec<(Tint, Span)> = Vec::with_capacity(toks.len());
    // how deep inside parentheses, where every word is a reference (`sketch front(m)`)
    let mut parens = 0i32;
    // inside a style rule's braces, whose body is `property: value` pairs
    let mut rule = false;
    // the last word, outside parentheses, that says what the next one is
    let mut after: Option<&str> = None;
    // whether the last run was a style rule's selector, whose brace opens the rule's body
    let mut selector = false;
    for (i, Token { text, quoted, span }) in toks.iter().enumerate() {
        let next = toks.get(i + 1).filter(|t| !t.quoted).map(|t| t.text.as_str());
        let tint = if *quoted {
            after = None;
            Some(Tint::Str)
        } else {
            match text.as_str() {
                "(" => {
                    parens += 1;
                    None
                }
                ")" => {
                    parens = (parens - 1).max(0);
                    None
                }
                "{" => {
                    rule = selector;
                    after = None;
                    None
                }
                "}" => {
                    rule = false;
                    None
                }
                "," | ":" | ";" => None,
                w if rule => {
                    if next == Some(":") {
                        Some(Tint::Label)
                    } else if w.starts_with('#') || number(w) {
                        Some(Tint::Num)
                    } else {
                        None
                    }
                }
                w if number(w) => Some(Tint::Num),
                // `sketch front(m)`, `measure distance(a, b)` — what the parentheses hold is
                // named in the model
                _ if parens > 0 => None,
                w => {
                    let tint = match after {
                        Some("style") => Some(Tint::Class),
                        Some(n) if NAMING.contains(&n) => Some(Tint::Def),
                        Some("measure") if w == "distance" => Some(Tint::Relation),
                        Some("size") if matches!(w, "A4" | "A3" | "Letter") => Some(Tint::Type),
                        // `from isometric`, and not `from m.std.top`, a reference
                        Some("from") if !w.contains('.') => Some(Tint::Type),
                        _ if WORDS.contains(&w) => Some(Tint::Word),
                        _ => None,
                    };
                    // a word in the table says what follows, unless it was itself the name or
                    // the value a word before it asked for
                    after = (tint == Some(Tint::Word)).then_some(w);
                    tint
                }
            }
        };
        selector = tint == Some(Tint::Class);
        if let Some(tint) = tint {
            out.push((tint, *span));
        }
    }
    with_comments(out, comments)
}

/// `80mm`, `1.4`, `-2` — what the parser reads as a number or a page length.
fn number(w: &str) -> bool {
    let w = w.strip_prefix(['-', '+']).unwrap_or(w);
    w.starts_with(|c: char| c.is_ascii_digit())
        || (w.starts_with('.') && w[1..].starts_with(|c: char| c.is_ascii_digit()))
}
