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
        "use std\nin std.front {\np := point hint((0, 0))\n}\n",
        "// nothing but a comment",
        "/* unclosed",
        "use std\nin std.front {\nhorizontal (a := line(p1, p2)) -> tangent\n(k := arc(center: c) hint(r: 5)) -> close\n}\n",
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
  R := m * N / 2
  hub := point hint((0, 5))
  center := point
  base := circle(center: center) hint(r: R) class construction
  radius(R) base
  fix((0, 0)) center
  cycle N as i {
    t := Tooth(base, a0: i * R)
  }
}
g := Gear(N: 30, m: 3)  // one wheel
";
    assert_eq!(tint_of(src, "component"), Some(Tint::Word));
    assert_eq!(tint_of(src, "Gear"), Some(Tint::Def));
    assert_eq!(tint_of(src, "N:"), Some(Tint::Label));
    assert_eq!(tint_of(src, "Int"), Some(Tint::Type));
    assert_eq!(tint_of(src, "circle)"), Some(Tint::Type));
    assert_eq!(tint_of(src, "R :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "2\n"), Some(Tint::Num));
    assert_eq!(tint_of(src, "point\n"), Some(Tint::Word));
    assert_eq!(tint_of(src, "hub"), Some(Tint::Def));
    assert_eq!(tint_of(src, "point hint"), Some(Tint::Word));
    assert_eq!(tint_of(src, "center :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "hint("), Some(Tint::Word));
    assert_eq!(tint_of(src, "base :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "circle(center"), Some(Tint::Word));
    assert_eq!(tint_of(src, "class"), Some(Tint::Word));
    assert_eq!(tint_of(src, "construction"), Some(Tint::Class));
    assert_eq!(tint_of(src, "radius"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "radius(R)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "fix(("), Some(Tint::Relation));
    assert_eq!(tint_of(src, "cycle"), Some(Tint::Word));
    assert_eq!(tint_of(src, "as"), Some(Tint::Word));
    assert_eq!(tint_of(src, "i {"), Some(Tint::Def));
    // every definition is a name before `:=`, and an instance's component reads as a type
    assert_eq!(tint_of(src, "t :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "Tooth"), Some(Tint::Type));
    assert_eq!(tint_of(src, "g :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "// one wheel"), Some(Tint::Comment));
}

/// A seed and a claim are different statements about the same number, and read as such.
///
/// The colouring does not key on `:=` — `w := 100` is written with one and is not a seed.  What makes a number a seed is the clause it stands in, which is §4.3's whole rule: a
/// number inside a `hint(…)` is a seed, and every other number is not.
#[test]
fn a_seed_and_a_claim_are_told_apart() {
    let src = "lo coincident e hint(t: 3)\ns coincident(t == 4) k\nw := 100";
    assert_eq!(tint_of(src, "coincident e"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "hint("), Some(Tint::Word));
    assert_eq!(tint_of(src, "t: 3"), Some(Tint::Label));
    assert_eq!(tint_of(src, "3)"), Some(Tint::Seed));
    assert_eq!(tint_of(src, "== 4"), Some(Tint::Claim));
    assert_eq!(tint_of(src, "100"), Some(Tint::Num), "a param is not a seed");
}

/// A computed point is arithmetic the parser never tokenizes — so the numbers in it are still
/// numbers, and nothing there is mistaken for a word.
#[test]
fn a_computed_point_is_arithmetic() {
    let src = "component Involute(c: circle, phase: Angle, u: Angle) {\n\
               p := point(x: c.center.x + c.r * 180, y: 0)\n}";
    assert_eq!(tint_of(src, "component"), Some(Tint::Word));
    assert_eq!(tint_of(src, "Involute"), Some(Tint::Def));
    assert_eq!(tint_of(src, "Angle"), Some(Tint::Type));
    assert_eq!(tint_of(src, "180"), Some(Tint::Num));
}

/// A curve reads as what it is: a definition, its `over` and `in` the words that shape it,
/// and the component's statements colour like any other statements.
#[test]
fn a_curve_is_coloured() {
    let src = "component unwind(c: circle, u: Angle) {\n  p := point\n  p coincident c\n}\n\
               w := unwind(c).p over u in (0, 1)";
    assert_eq!(tint_of(src, "w :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "over"), Some(Tint::Word));
    assert_eq!(tint_of(src, "in ("), Some(Tint::Word));
    assert_eq!(tint_of(src, "coincident c"), Some(Tint::Relation));
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
    let src = "use std\nin std.front {\np := point\n/* two\n   lines */\nl := line(p, p)\n}\n";
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
a := point hint((0, 0))
ab := line(a, a) class centerline heavy
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
use std
in std.front {
p := point hint((0, 0))
q := point hint((60, 0))
r := point hint((60, 40))
p distance(80) q
p distance(20, along: y) r
q equal r
horizontal p
}
";
    assert_eq!(tint_of(src, "distance(80)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "distance(20"), Some(Tint::Relation), "a selector beside the number");
    assert_eq!(tint_of(src, "equal"), Some(Tint::Relation), "and the paren-less form as before");
    assert_eq!(tint_of(src, "horizontal"), Some(Tint::Relation));
}

#[test]
fn a_faces_close_marker_is_distinct_from_an_edge_named_close() {
    for src in ["f := face(a, b, c, -> close)", "f := face(a, b, c, -> close,)"] {
        assert_eq!(tint_of(src, "close"), Some(Tint::Word), "{src}");
    }
    assert_eq!(tint_of("f := face(ab, bc, close)", "close"), None);
}

/// A declaration's name is **optional** (issue #33), so the word after an element keyword is a
/// name only when it could be one: a trailing clause's word or an operator there keeps its own
/// reading — the reservation `names_decl` states, asked by the parser and the colouring alike —
/// and a name that spells an operator stays plain where it is *used*, a bare name in an argument
/// list being followed by `,` or `)`.
#[test]
fn an_anonymous_declaration_gives_its_name_tint_to_nobody() {
    let src = "\
a := point hint((0, 0))
point hint((1, 0))
line class construction
line -> tangent arc -> tangent line
";
    assert_eq!(tint_of(src, "a :="), Some(Tint::Def), "a written name still tints");
    assert_eq!(tint_of(src, "hint((1"), Some(Tint::Word), "an anonymous point's clause");
    assert_eq!(tint_of(src, "class"), Some(Tint::Word), "an anonymous line's clause");
    assert_eq!(tint_of(src, "tangent arc"), Some(Tint::Relation), "a joint on an anonymous link");
    // a link named where it stands names itself, and the word after it is still a joint
    let named = "use std\nin std.front {\n(ab := line) -> tangent arc\n}\n";
    assert_eq!(tint_of(named, "ab"), Some(Tint::Def));
    assert_eq!(tint_of(named, "line"), Some(Tint::Word));
    assert_eq!(tint_of(named, "tangent"), Some(Tint::Relation));
}

/// A used module's component is called by its full path, and the component's name reads as one.
#[test]
fn a_module_call_is_coloured_by_its_component() {
    let src = "use engine.parts\nc := engine.parts.Crank(o, dims: engine.dims.engine_dims)\n";
    assert_eq!(tint_of(src, "Crank"), Some(Tint::Type));
    assert_eq!(tint_of(src, "c :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "dims:"), Some(Tint::Label));
}

/// A group's braces open a list of labelled members, not a body of statements, so a member's
/// label is a label (where a statement's head would read as an instance) and the list may run
/// across lines.
#[test]
fn a_group_is_coloured_as_labelled_members() {
    let src = "dims := {\n  width: 20mm,\n  origin: o\n}\n";
    assert_eq!(tint_of(src, "dims"), Some(Tint::Def));
    assert_eq!(tint_of(src, "width"), Some(Tint::Label));
    assert_eq!(tint_of(src, "origin"), Some(Tint::Label));
    assert_eq!(tint_of(src, "20mm"), Some(Tint::Num));
    // and a member written as a group in place is labelled members too
    let src = "dims := {\n  cyl: {\n    bore: 16mm,\n    ax: datum\n  }\n}\n";
    assert_eq!(tint_of(src, "cyl"), Some(Tint::Label));
    assert_eq!(tint_of(src, "bore"), Some(Tint::Label));
    assert_eq!(tint_of(src, "ax"), Some(Tint::Label));
    assert_eq!(tint_of(src, "16mm"), Some(Tint::Num));
}

/// `param` is a word, the name after it is the name it declares, and its type is a type — the
/// colouring of a formal, at the top of a document.
#[test]
fn an_input_is_coloured_like_a_formal() {
    let src = "param beta: Angle hint(30deg)\nparam bore := 50\n";
    assert_eq!(tint_of(src, "param beta"), Some(Tint::Word));
    assert_eq!(tint_of(src, "beta:"), Some(Tint::Def));
    assert_eq!(tint_of(src, "Angle"), Some(Tint::Type));
    assert_eq!(tint_of(src, "bore"), Some(Tint::Def));
}

/// An energy (§9.10) is read by its words: `minimizes`, `integral`, and the `over` naming the
/// point the integrand is taken at — a binder, as a set's point is.
#[test]
fn an_energy_is_coloured() {
    let src = "rope minimizes integral(p.y over p)\n\
               k maximizes integral((p.x * t.y - p.y * t.x) / 2 over (p, t))\n";
    assert_eq!(tint_of(src, "minimizes"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "integral(p"), Some(Tint::Word));
    assert_eq!(tint_of(src, "over p"), Some(Tint::Word));
    assert_eq!(tint_of(src, "p)\n"), Some(Tint::Def), "the binder");
    assert_eq!(tint_of(src, "p.y over"), None, "the integrand reads it plainly");
    assert_eq!(tint_of(src, "p, t)"), Some(Tint::Def));
    assert_eq!(tint_of(src, "t))"), Some(Tint::Def));
    assert_eq!(tint_of(src, "2 over"), Some(Tint::Num));
}

/// **A relation word's definition declares the word** (§9.9): the word is the name the statement
/// gives, its operands and parameters are the body's to read, and where the file uses it — or a
/// file importing it by name — it reads as the relation it is.
#[test]
fn a_relation_word_is_coloured_where_defined_and_used() {
    let src = "\
a horizontal b := a level(up) b
b right_of(d) a := {
  a distance(d, along: right) b
}
flat(d) l := l distance(d, along: up) l
p right_of(d: 5) q
flat l
";
    assert_eq!(tint_of(src, "horizontal b"), Some(Tint::Def));
    assert_eq!(tint_of(src, "b :="), None, "an operand is not what the statement declares");
    assert_eq!(tint_of(src, "level(up)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "right_of(d) a"), Some(Tint::Def));
    assert_eq!(tint_of(src, "a := {"), None);
    assert_eq!(tint_of(src, "flat(d) l :="), Some(Tint::Def));
    assert_eq!(tint_of(src, "right_of(d: 5)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "d: 5"), Some(Tint::Label));
    assert_eq!(tint_of(src, "flat l\n"), Some(Tint::Relation));
    // an imported word, and the module it comes from
    let src = "use std (right_of)\ncentre right_of(d: ring) std.origin\nx := Turned(o, t)\n";
    assert_eq!(tint_of(src, "std ("), Some(Tint::Type));
    assert_eq!(tint_of(src, "right_of)"), None);
    assert_eq!(tint_of(src, "right_of(d"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "Turned"), Some(Tint::Type), "a component call is still a call");
    let src = "use engine.parts\n";
    assert_eq!(tint_of(src, "engine"), Some(Tint::Type));
    assert_eq!(tint_of(src, "parts"), Some(Tint::Type));
}

/// A set's point is the name its body binds (§6.21), however the set is written.
#[test]
fn a_set_binds_its_point() {
    let src = "S := { p | p distance(5) c }\n\
               component Ball(c: point, r: Length) := { q | q distance(r) c }\n";
    assert_eq!(tint_of(src, "p |"), Some(Tint::Def));
    assert_eq!(tint_of(src, "distance(5)"), Some(Tint::Relation));
    assert_eq!(tint_of(src, "q |"), Some(Tint::Def));
}

/// The smaller words a statement is shaped by: a ring's centre, the copy after this one, a
/// spline's weights and a curve's stretch.
#[test]
fn the_shaping_words_are_coloured() {
    let src = "\
ring n about c {
  v := point
  e := line(v, next.v)
}
k := spline(a, b, c, d) weights [1, w, w, 1]
f := face(e from lo to hi, -> close)
x := Cylinder(about: l, r: 5)
";
    assert_eq!(tint_of(src, "ring"), Some(Tint::Word));
    assert_eq!(tint_of(src, "about c"), Some(Tint::Word));
    assert_eq!(tint_of(src, "next.v"), Some(Tint::Word));
    assert_eq!(tint_of(src, "weights"), Some(Tint::Word));
    assert_eq!(tint_of(src, "from lo"), Some(Tint::Word));
    assert_eq!(tint_of(src, "to hi"), Some(Tint::Word));
    assert_eq!(tint_of(src, "about: l"), Some(Tint::Label), "a label is a label");
    // and `about`, `to` used as names stay names
    let src = "c := Cylinder(about, r: 5)\nm := line(a, to)\n";
    assert_eq!(tint_of(src, "about,"), None);
    assert_eq!(tint_of(src, "to)"), None);
}

/// A unit written against its number is one literal to the parser, and is coloured as one: a
/// seed's unit is a seed's.
#[test]
fn a_unit_is_coloured_with_its_number() {
    let src = "unit mm\nw := 20mm\np := point hint(at: o, bearing: 90deg)\nh := 1' 6\"\nt := 3in\n";
    let runs = highlight(src);
    let run = |what: &str| {
        let lo = src.find(what).expect("in the text");
        let (t, s) = *runs.iter().find(|(_, s)| s.lo as usize == lo)?;
        Some((t, &src[s.lo as usize..s.hi as usize]))
    };
    assert_eq!(run("20mm"), Some((Tint::Num, "20mm")));
    assert_eq!(run("90deg"), Some((Tint::Seed, "90deg")));
    assert_eq!(run("1'"), Some((Tint::Num, "1'")));
    assert_eq!(run("6\""), Some((Tint::Num, "6\"")));
    assert_eq!(run("3in"), Some((Tint::Num, "3in")));
    assert_eq!(tint_of(src, "mm\nw"), Some(Tint::Type), "the document's unit");
}

/// What an expression knows by name is coloured built in, from `expr`'s own tables.
#[test]
fn a_builtin_is_coloured() {
    let src = "\
Rr := max(R - 2, Rb * (1 + clear))
r := sqrt(3) * pi
q := point hint(at: o, toward: t, turn: 90deg)
k := motion(about: o, ratio: -length(a) / length(b))
length(150mm) rope
";
    assert_eq!(tint_of(src, "max("), Some(Tint::Builtin));
    assert_eq!(tint_of(src, "sqrt"), Some(Tint::Builtin));
    assert_eq!(tint_of(src, "pi"), Some(Tint::Builtin));
    assert_eq!(tint_of(src, "turn:"), Some(Tint::Label));
    assert_eq!(tint_of(src, "length(a)"), Some(Tint::Builtin));
    assert_eq!(tint_of(src, "length(b)"), Some(Tint::Builtin));
    assert_eq!(tint_of(src, "length(150mm)"), Some(Tint::Relation), "a relation is a relation");
}

/// Every model in the library and the standard library is coloured end to end, the spans tiling
/// the text: the colouring is asked of whatever the corpus writes.
#[test]
fn the_corpus_tiles() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let mut files = Vec::new();
    let mut dirs = vec![root.join("examples"), root.join("lib")];
    while let Some(d) = dirs.pop() {
        for e in std::fs::read_dir(&d).expect("a directory") {
            let p = e.expect("an entry").path();
            if p.is_dir() {
                dirs.push(p);
            } else if matches!(p.extension().and_then(|x| x.to_str()), Some("sv" | "svd")) {
                files.push(p);
            }
        }
    }
    assert!(files.len() > 100, "the corpus is there");
    for f in files {
        let src = std::fs::read_to_string(&f).expect("readable");
        let drawing = f.extension().is_some_and(|x| x == "svd");
        let runs =
            if drawing { gcs_core::drawing::highlight(&src) } else { highlight(&src) };
        let mut end = 0usize;
        for (tint, s) in runs {
            assert!(s.lo as usize >= end && s.lo < s.hi, "{f:?}: {tint:?} at {}", s.lo);
            assert!(s.hi as usize <= src.len());
            end = s.hi as usize;
        }
    }
}

/// A drawing is coloured by its own grammar: its statements, the names they declare, strings,
/// page lengths and its style rules.
#[test]
fn a_drawing_is_coloured() {
    let src = "\
// presentation
model m from \"nurbs.sv\"
sheet nurbs {
  size A3
  label \"RATIONAL\" at (14mm, 18mm)
  view dome(m.dome) from isometric at (215mm, 120mm) scale 1.4
  sketch top(m) from m.std.top at (80mm, 90mm)
  measure distance(m.o, m.bead) in dome offset 4mm
  dimensions in top
  style m.toe_r { color: #7a7a7a; dash: 3 3 }
}
";
    let tint = |what: &str| {
        let lo = src.find(what).expect("in the text");
        let runs = gcs_core::drawing::highlight(src);
        runs.into_iter().find(|(_, s)| s.lo as usize == lo).map(|r| r.0)
    };
    assert_eq!(tint("// presentation"), Some(Tint::Comment));
    assert_eq!(tint("model"), Some(Tint::Word));
    assert_eq!(tint("m from"), Some(Tint::Def));
    assert_eq!(tint("from \""), Some(Tint::Word));
    assert_eq!(tint("\"nurbs.sv\""), Some(Tint::Str));
    assert_eq!(tint("sheet"), Some(Tint::Word));
    assert_eq!(tint("nurbs {"), Some(Tint::Def));
    assert_eq!(tint("A3"), Some(Tint::Type));
    assert_eq!(tint("label"), Some(Tint::Word));
    assert_eq!(tint("14mm"), Some(Tint::Num));
    assert_eq!(tint("dome("), Some(Tint::Def));
    assert_eq!(tint("m.dome"), None, "a reference into the model");
    assert_eq!(tint("isometric"), Some(Tint::Type));
    assert_eq!(tint("m.std.top"), None);
    assert_eq!(tint("1.4"), Some(Tint::Num));
    assert_eq!(tint("distance"), Some(Tint::Relation));
    assert_eq!(tint("offset"), Some(Tint::Word));
    assert_eq!(tint("in top"), Some(Tint::Word));
    assert_eq!(tint("m.toe_r"), Some(Tint::Class));
    assert_eq!(tint("color"), Some(Tint::Label));
    assert_eq!(tint("#7a7a7a"), Some(Tint::Num));
    // a drawing half-typed is coloured as far as it goes
    let half = "model m from \"open";
    let runs = gcs_core::drawing::highlight(half);
    assert_eq!(runs.last().map(|r| r.0), Some(Tint::Str));
}
