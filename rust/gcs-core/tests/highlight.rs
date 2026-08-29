//! Colouring a program.
//!
//! `syntax::highlight` is the parser's own scan, kept rather than thrown away, so the two cannot
//! disagree about what a word is.  What is worth testing is therefore not the colours — they are
//! a stylesheet's business — but that the spans **tile the text exactly**, in order and without
//! overlap, and that the words the parser gives a meaning to are the words that come back tinted.

use gcs_core::examples::GEAR;
use gcs_core::syntax::{highlight, Tint};

/// Every span is inside the text, they are in order, and no two overlap — which is the whole of
/// what a front end needs to write the runs out with the plain text between them.
#[test]
fn the_spans_tile_the_text() {
    for src in [
        GEAR,
        "",
        "point p hint(x: 0, y: 0)",
        "// nothing but a comment",
        "/* unclosed",
        "horizontal line a(p1, p2) -> tangent\narc k(center: c) hint(r: 5) -> close",
    ] {
        let mut end = 0usize;
        for (tint, s) in highlight(src) {
            assert!(s.lo as usize >= end, "{tint:?} at {} overlaps the run before it", s.lo);
            assert!(s.hi as usize <= src.len(), "{tint:?} runs past the end of the text");
            assert!(s.lo < s.hi, "{tint:?} is an empty run");
            assert!(src.is_char_boundary(s.lo as usize) && src.is_char_boundary(s.hi as usize));
            end = s.hi as usize;
        }
    }
}

/// The tint of the first run that *begins* with `what` — so a needle may be written with as much
/// of what follows it as it takes to be unambiguous, and one that also occurs inside a longer word
/// (`as` in `base`) finds the run rather than the letters.
fn tint_of(src: &str, what: &str) -> Option<Tint> {
    let mut runs = highlight(src).into_iter();
    let hit = runs.find(|(_, s)| src[s.lo as usize..].starts_with(what));
    match hit {
        Some((t, _)) => Some(t),
        None => {
            assert!(src.contains(what), "`{what}` is not in the text at all");
            None
        }
    }
}

/// One statement of each shape the parser knows, and the word that says which shape it is.
#[test]
fn a_statement_is_coloured_by_what_it_declares() {
    let src = "\
component Gear(N: Int, m: Length, c: circle) {
  param R = m * N / 2
  port hub: point
  point center hint(x: 0, y: 0)
  circle base(center: center) hint(r: R) class construction
  radius(R) base
  ground center
  cycle N as i {
    t: Tooth(base, a0: i * R)
  }
}
g: Gear(N: 30, m: 3)  // one wheel
";
    assert_eq!(tint_of(src, "component"), Some(Tint::Word));
    assert_eq!(tint_of(src, "Gear"), Some(Tint::Def));
    assert_eq!(tint_of(src, "N:"), Some(Tint::Label));
    assert_eq!(tint_of(src, "Int"), Some(Tint::Type));
    assert_eq!(tint_of(src, "circle)"), Some(Tint::Type));
    assert_eq!(tint_of(src, "param"), Some(Tint::Word));
    assert_eq!(tint_of(src, "R ="), Some(Tint::Def));
    assert_eq!(tint_of(src, "2\n"), Some(Tint::Num));
    assert_eq!(tint_of(src, "port"), Some(Tint::Word));
    assert_eq!(tint_of(src, "hub"), Some(Tint::Def));
    assert_eq!(tint_of(src, "point\n"), Some(Tint::Type));
    assert_eq!(tint_of(src, "point center"), Some(Tint::Word));
    assert_eq!(tint_of(src, "center hint"), Some(Tint::Def));
    assert_eq!(tint_of(src, "hint("), Some(Tint::Word));
    assert_eq!(tint_of(src, "base(center"), Some(Tint::Def));
    assert_eq!(tint_of(src, "class"), Some(Tint::Word));
    assert_eq!(tint_of(src, "construction"), Some(Tint::Class));
    assert_eq!(tint_of(src, "radius"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "radius(R)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "ground"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "cycle"), Some(Tint::Word));
    assert_eq!(tint_of(src, "as"), Some(Tint::Word));
    assert_eq!(tint_of(src, "i {"), Some(Tint::Def));
    // an instance is a name, a colon and a component — the one type written without a `:` first
    assert_eq!(tint_of(src, "t:"), Some(Tint::Def));
    assert_eq!(tint_of(src, "Tooth"), Some(Tint::Type));
    assert_eq!(tint_of(src, "g:"), Some(Tint::Def));
    assert_eq!(tint_of(src, "// one wheel"), Some(Tint::Comment));
}

/// A seed and a claim are different statements about the same number, and read as such.
///
/// The colouring does not key on `=` any more — `param w = 100` is written with one and is not a
/// seed.  What makes a number a seed is the clause it stands in, which is §4.3's whole rule: a
/// number inside a `hint(…)` is a seed, and every other number is not.
#[test]
fn a_seed_and_a_claim_are_told_apart() {
    let src = "lo on e hint(u: 3)\ns on(t == 4) k\nparam w = 100";
    assert_eq!(tint_of(src, "on e"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "hint("), Some(Tint::Word));
    assert_eq!(tint_of(src, "u:"), Some(Tint::Label));
    assert_eq!(tint_of(src, "3)"), Some(Tint::Seed));
    assert_eq!(tint_of(src, "== 4"), Some(Tint::Claim));
    assert_eq!(tint_of(src, "100"), Some(Tint::Num), "a param is not a seed");
}

/// A curve family is a declaration like any other, and its body is arithmetic the parser never
/// tokenizes — so the numbers in it are still numbers, and nothing there is mistaken for a word.
#[test]
fn a_curve_family_is_a_declaration() {
    let src = "curve involute(c: circle, phase: Angle)(u) =\n  ( c.center.x + c.r * 180, 0 )";
    assert_eq!(tint_of(src, "curve"), Some(Tint::Word));
    assert_eq!(tint_of(src, "involute"), Some(Tint::Def));
    assert_eq!(tint_of(src, "Angle"), Some(Tint::Type));
    assert_eq!(tint_of(src, "180"), Some(Tint::Num));
}

/// A trace family reads as what it is: `trace` and `where` are keywords, the traced point is a
/// name the family declares, and the block's statements colour like any other statements.
#[test]
fn a_trace_family_is_coloured() {
    let src = "curve unwind(c: circle)(u) =\n  trace p where {\n\
               point p\n    p on c\n  }";
    assert_eq!(tint_of(src, "trace"), Some(Tint::Word));
    assert_eq!(tint_of(src, "p where"), Some(Tint::Def));
    assert_eq!(tint_of(src, "where"), Some(Tint::Word));
    assert_eq!(tint_of(src, "on c"), Some(Tint::Relation));
}

/// The reference document, coloured.  A cheap guard that the rules above reach the real thing:
/// the gear names a component, a curve family, a cycle and half a dozen relations.
#[test]
fn the_gear_is_coloured() {
    let ts = highlight(GEAR);
    for want in [Tint::Comment, Tint::Word, Tint::Type, Tint::Relation, Tint::Def, Tint::Num] {
        assert!(ts.iter().any(|&(t, _)| t == want), "nothing in the gear is {want:?}");
    }
}

/// A block comment is *one* run, however many lines it spans — the thing a front end scanning for
/// `//` by line would get wrong, and the reason this is the core's scan and not a second one.
#[test]
fn a_block_comment_is_one_run() {
    let src = "point p\n/* two\n   lines */\nline l(p, p)";
    let ts = highlight(src);
    let (tint, span) = ts.iter().find(|&&(t, _)| t == Tint::Comment).expect("a comment");
    assert_eq!(*tint, Tint::Comment);
    assert_eq!(&src[span.lo as usize..span.hi as usize], "/* two\n   lines */");
    assert_eq!(ts.iter().filter(|&&(t, _)| t == Tint::Comment).count(), 1);
    // and the word after it still starts a statement
    assert_eq!(tint_of(src, "line"), Some(Tint::Word));
}

/// **Presentation reads as presentation.**  A class on a declaration and the `style` block that
/// says what it looks like are a different statement from what the drawing *is*, and the
/// colouring says so: the class name has a tint of its own, wherever it stands.
#[test]
fn a_class_and_a_style_block_read_as_presentation() {
    let src = "\
style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }
point a hint(x: 0, y: 0)
line ab(a, a) class centerline heavy
";
    assert_eq!(tint_of(src, "style"), Some(Tint::Word));
    assert_eq!(tint_of(src, "centerline {"), Some(Tint::Class));
    assert_eq!(tint_of(src, "dash"), Some(Tint::Label));
    assert_eq!(tint_of(src, "12"), Some(Tint::Num), "a sheet's lengths are not seeds");
    assert_eq!(tint_of(src, "class"), Some(Tint::Word));
    assert_eq!(tint_of(src, "centerline heavy"), Some(Tint::Class));
    assert_eq!(tint_of(src, "heavy"), Some(Tint::Class), "every class in the list");
}

/// **An operator carries its number on the word, and is still an operator.**
///
/// `p distance(80) q` is the ordinary spelling of a dimension now (spec §9.1), so the token after
/// the word is `(` and not the right operand.  The joint test read `i + 1` and found punctuation,
/// which left every parenthesised operator — most of the constraint statements in a document —
/// uncoloured while the paren-less ones beside them were tinted.  `word_past_args` is the one
/// lookahead both this and `chain_starts` ask, so a word coloured as a relation here is a word
/// the parser settles as one.
#[test]
fn an_operator_is_coloured_through_its_own_parentheses() {
    let src = "\
point p hint(x: 0, y: 0)
point q hint(x: 60, y: 0)
point r hint(x: 60, y: 40)
p distance(80) q
p distance(20, along: y) r
q equal r
horizontal p
";
    assert_eq!(tint_of(src, "distance(80)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "distance(20"), Some(Tint::Relation), "a selector beside the number");
    assert_eq!(tint_of(src, "equal"), Some(Tint::Relation), "and the paren-less form as before");
    assert_eq!(tint_of(src, "horizontal"), Some(Tint::Relation));
}

/// A declaration's name is **optional** (issue #33), so the word after an element keyword is a
/// name only when it could be one: a trailing clause's word or an operator there keeps its own
/// reading — the reservation `names_decl` states, asked by the parser and the colouring alike —
/// and a name that spells an operator stays plain where it is *used*, a bare name in an argument
/// list being followed by `,` or `)`.
#[test]
fn an_anonymous_declaration_gives_its_name_tint_to_nobody() {
    let src = "\
point a hint(x: 0, y: 0)
point hint(x: 1, y: 0)
line class construction
line -> tangent arc -> tangent line
";
    assert_eq!(tint_of(src, "a hint"), Some(Tint::Def), "a written name still tints");
    assert_eq!(tint_of(src, "hint(x: 1"), Some(Tint::Word), "an anonymous point's clause");
    assert_eq!(tint_of(src, "class"), Some(Tint::Word), "an anonymous line's clause");
    assert_eq!(tint_of(src, "tangent arc"), Some(Tint::Relation), "a joint on an anonymous link");
    // a curve's name is not optional (`decl()` makes the same exception), so its next word is a
    // name whatever it spells — the parser accepts `curve tangent = …` and the colour agrees
    assert_eq!(tint_of("curve tangent = involute(c)\n", "tangent"), Some(Tint::Def));
}
