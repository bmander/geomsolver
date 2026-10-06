//! Tokens and byte spans shared by parsing and highlighting.

use super::{Span, SynErr};

#[derive(Clone, Debug, PartialEq)]
pub(super) enum Tok {
    Ident(String),
    Num(f64),
    /// `(` `)` `[` `]` `,` `:` `.` `-` `{` `}`
    P(char),
    /// `:=` — a definition: the name before it is the value after it (§5).
    Define,
    /// `==` — a constraint
    EqEq,
    /// `->` — a chain's joint marker: the two links beside it share a boundary point (§6.6)
    Arrow,
    /// end of a statement: a newline or a `;`
    Nl,
}

pub(super) struct Lexed {
    pub(super) toks: Vec<(Tok, Span)>,
    /// Where the comments were.  The parser has no use for them — they are not the program — but
    /// they *are* the document, and `highlight` is the one reader that has to show them.
    pub(super) comments: Vec<Span>,
}

/// Tokenize.  Errors are collected rather than thrown: one bad character costs one statement, and
/// the rest of the drawing still comes back.
pub(super) fn lex(src: &str) -> (Lexed, Vec<SynErr>) {
    let b = src.as_bytes();
    let mut toks: Vec<(Tok, Span)> = Vec::new();
    let mut comments: Vec<Span> = Vec::new();
    let mut errs: Vec<SynErr> = Vec::new();
    let mut i = 0usize;
    // A newline ends a statement, but not inside brackets: an argument list may be written across
    // several lines, and a line break there is a separator like any other whitespace.  A body's
    // braces do *not* count — a body is made of statements, and those still end at their line's
    // end — but a group's do (`dims := {width: 20mm, origin: o}`): a brace straight after `:=`
    // opens a list of members, as one after a member's `:` does (`{cyl: {bore: 16mm}}`), and
    // `braces` remembers which kind each open brace is.
    let mut depth = 0i32;
    let mut braces: Vec<bool> = Vec::new();
    while i < b.len() {
        // Decode UTF-8 before classifying; every arm must consume a whole character.
        let c = src[i..].chars().next().unwrap_or(' ');
        let lo = i;
        match c {
            ' ' | '\t' | '\r' => i += 1,
            '\n' => {
                i += 1;
                if depth == 0 {
                    toks.push((Tok::Nl, Span::new(lo, i)));
                }
            }
            ';' => {
                i += 1;
                toks.push((Tok::Nl, Span::new(lo, i)));
            }
            '/' if b.get(i + 1) == Some(&b'/') => {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
                comments.push(Span::new(lo, i));
            }
            '/' if b.get(i + 1) == Some(&b'*') => {
                i += 2;
                while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
                    i += 1;
                }
                i = (i + 2).min(b.len());
                comments.push(Span::new(lo, i));
            }
            // `==` pins a slot; a lone `=` is no token the grammar has, and is read as stray
            // punctuation so the statement holding it is refused where it stands
            '=' if b.get(i + 1) == Some(&b'=') => {
                i += 2;
                toks.push((Tok::EqEq, Span::new(lo, i)));
            }
            ':' if b.get(i + 1) == Some(&b'=') => {
                i += 2;
                toks.push((Tok::Define, Span::new(lo, i)));
            }
            // `->` is the joint marker (§6.6).  `>` is not a token on its own, so the pair is
            // claimed here before `-` can read as punctuation.
            '-' if b.get(i + 1) == Some(&b'>') => {
                i += 2;
                toks.push((Tok::Arrow, Span::new(lo, i)));
            }
            // Shorthands, read as the word they stand for: `@` is `coincident` (`p @ c`) and `~`
            // is `hint` (`point ~((0, 0))`).  The token's span is the one character, so a
            // splice edits what was written and the parser never knows.
            '@' | '~' => {
                i += 1;
                let word = if c == '@' { "coincident" } else { "hint" };
                toks.push((Tok::Ident(word.to_string()), Span::new(lo, i)));
            }
            // `'` and `"` are the foot and inch marks (spec §3.3), and `|` is what a raw
            // branch key separates its points with.  The language has **no string literal**:
            // a quote after a number is a unit, and there is nothing else for one to be.
            '(' | ')' | '[' | ']' | ',' | ':' | '{' | '}' | '-' | '+' | '\'' | '"' | '|' => {
                i += 1;
                match c {
                    '(' | '[' => depth += 1,
                    ')' | ']' => depth = (depth - 1).max(0),
                    '{' => {
                        let group = match toks.last() {
                            Some((Tok::Define, _)) => true,
                            Some((Tok::P(':'), _)) => braces.last() == Some(&true),
                            _ => false,
                        };
                        braces.push(group);
                        if group {
                            depth += 1;
                        }
                    }
                    '}' if braces.pop() == Some(true) => depth = (depth - 1).max(0),
                    _ => {}
                }
                toks.push((Tok::P(c), Span::new(lo, i)));
            }
            '.' if !b.get(i + 1).is_some_and(|d| d.is_ascii_digit()) => {
                i += 1;
                toks.push((Tok::P('.'), Span::new(lo, i)));
            }
            c if c.is_ascii_digit() || c == '.' => {
                while i < b.len()
                    && ((b[i] as char).is_ascii_digit()
                        || b[i] == b'.'
                        || b[i] == b'e'
                        || b[i] == b'E'
                        || ((b[i] == b'+' || b[i] == b'-')
                            && matches!(b.get(i - 1), Some(b'e') | Some(b'E'))))
                {
                    i += 1;
                }
                let text = &src[lo..i];
                match text.parse::<f64>() {
                    Ok(v) if v.is_finite() => toks.push((Tok::Num(v), Span::new(lo, i))),
                    _ => errs.push(SynErr {
                        span: Span::new(lo, i),
                        message: format!("`{text}` is not a number"),
                    }),
                }
            }
            c if ident_start(c) => {
                // `ident_start` implies `ident_char`, so this consumes at least `c`
                while let Some(ch) = src[i..].chars().next().filter(|&ch| ident_char(ch)) {
                    i += ch.len_utf8();
                }
                toks.push((Tok::Ident(src[lo..i].to_string()), Span::new(lo, i)));
            }
            // Anything else becomes a token rather than an error.  Most of what lands here is
            // arithmetic — `w / 5`, `2 * a + 5` — inside a dimension, which the parser takes from
            // the source verbatim and never asks the lexer about; erroring on it here would
            // report the one part of a program this file deliberately does not read.  A character
            // that really is out of place is caught where it is *used*, with a span on the
            // statement that wanted something else.
            other => {
                i += other.len_utf8();
                toks.push((Tok::P(other), Span::new(lo, i)));
            }
        }
    }
    (Lexed { toks, comments }, errs)
}

/// Identifier character rules, shared with name validation.
pub(super) fn ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_'
}

pub(super) fn ident_char(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}
