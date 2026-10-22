//! The names a text gives and reads (`syntax::symbols`): what an editor lights when its caret is
//! on a name, and what going to a definition or renaming one will stand on.
//!
//! Each test writes the answer back into the text: every place the symbol under the caret is
//! named is marked, `«given»` where it is declared and `‹read›` where it is read, so what the
//! index says is read off the drawing it was asked of.

use gcs_core::syntax::{symbols, SymbolKind};

/// `src` with every occurrence of the symbol named at the `nth` (from 0) whole-word `word` marked.
fn lit(src: &str, word: &str, nth: usize) -> String {
    let at = whole_words(src, word).nth(nth).unwrap_or_else(|| panic!("no {word} #{nth}"));
    let index = symbols(src);
    let o = index.at(at).unwrap_or_else(|| panic!("{word} #{nth} is no name"));
    let mut out = String::new();
    let mut end = 0usize;
    for o in index.of(o.symbol) {
        let (lo, hi) = (o.span.lo as usize, o.span.hi as usize);
        out.push_str(&src[end..lo]);
        let (open, close) = if o.def { ("«", "»") } else { ("‹", "›") };
        out.push_str(open);
        out.push_str(&src[lo..hi]);
        out.push_str(close);
        end = hi;
    }
    out.push_str(&src[end..]);
    out
}

/// Where `word` stands as a whole word.
fn whole_words<'a>(src: &'a str, word: &'a str) -> impl Iterator<Item = usize> + 'a {
    let ident = |c: char| c.is_alphanumeric() || c == '_';
    src.match_indices(word).map(|(k, _)| k).filter(move |&k| {
        !src[..k].chars().next_back().is_some_and(ident)
            && !src[k + word.len()..].chars().next().is_some_and(ident)
    })
}

/// Whether the `nth` `word` names anything at all.
fn names(src: &str, word: &str, nth: usize) -> bool {
    let at = whole_words(src, word).nth(nth).expect("the word is there");
    symbols(src).at(at).is_some()
}

#[test]
fn a_name_is_lit_where_it_is_given_and_everywhere_it_is_read() {
    let src = "a := point\nb := point\nl := line(a, b)\na distance(5) b\n";
    assert_eq!(
        lit(src, "a", 1),
        "«a» := point\nb := point\nl := line(‹a›, b)\n‹a› distance(5) b\n"
    );
    // from its declaration as from a read
    assert_eq!(lit(src, "a", 0), lit(src, "a", 2));
}

#[test]
fn the_caret_just_past_a_name_means_it() {
    let src = "rim := circle\nradius(5) rim\n";
    let index = symbols(src);
    let end = src.find(" :=").unwrap();
    assert_eq!(index.at(end).map(|o| o.span.lo), Some(0), "rim| is rim");
    assert!(index.at(end + 1).is_none(), "a space after it is not");
}

#[test]
fn words_the_grammar_keeps_name_nothing() {
    let src = "use std\nin std.front {\np := point hint((0, 0))\nl := line(p2: p)\n}\n\
               fix(x == 0) p\nw := sqrt(2) * 20mm\nrepeat 3 as i {\nq := point\n}\n";
    for (word, nth) in [("in", 0), ("point", 0), ("hint", 0), ("line", 0), ("p2", 0), ("fix", 0),
                        ("x", 0), ("sqrt", 0), ("repeat", 0), ("as", 0)] {
        assert!(!names(src, word, nth), "`{word}` names nothing");
    }
    assert!(symbols(src).at(src.find("mm").unwrap() + 1).is_none(), "a unit is its number's");
}

#[test]
fn a_components_formal_is_its_own_and_not_the_documents() {
    let src = "a := point\nb := point\n\
               component Rung(a: point, b: point, len: Length) {\n\
                 e := line(a, b)\n  a distance(len) b\n}\n\
               t0 := Rung(a, b, len: 50)\n";
    assert_eq!(
        lit(src, "a", 1),
        "a := point\nb := point\n\
         component Rung(«a»: point, b: point, len: Length) {\n\
           e := line(‹a›, b)\n  ‹a› distance(len) b\n}\n\
         t0 := Rung(a, b, len: 50)\n"
    );
    assert_eq!(
        lit(src, "a", 4),
        "«a» := point\nb := point\n\
         component Rung(a: point, b: point, len: Length) {\n\
           e := line(a, b)\n  a distance(len) b\n}\n\
         t0 := Rung(‹a›, b, len: 50)\n"
    );
    // the label at the call fills the formal, so it is the formal's
    assert_eq!(
        lit(src, "len", 2),
        "a := point\nb := point\n\
         component Rung(a: point, b: point, «len»: Length) {\n\
           e := line(a, b)\n  a distance(‹len›) b\n}\n\
         t0 := Rung(a, b, ‹len›: 50)\n"
    );
    // and the component is lit where it is defined and where it is called
    assert_eq!(lit(src, "Rung", 1), lit(src, "Rung", 0));
    let index = symbols(src);
    let at = src.find("Rung").unwrap();
    assert_eq!(index.symbols[index.at(at).unwrap().symbol].kind, SymbolKind::Component);
}

#[test]
fn a_member_read_through_an_instance_is_the_one_its_component_declares() {
    let src = "component Rung(a: point) {\n  mid := point\n  a distance(5) mid\n}\n\
               p := point\nt0 := Rung(p)\nt1 := Rung(p)\nt0.mid distance(3) t1.mid\n\
               k := trace(Rung(p).mid over t in (0, 1))\n";
    assert_eq!(
        lit(src, "mid", 2),
        "component Rung(a: point) {\n  «mid» := point\n  a distance(5) ‹mid›\n}\n\
         p := point\nt0 := Rung(p)\nt1 := Rung(p)\nt0.‹mid› distance(3) t1.‹mid›\n\
         k := trace(Rung(p).‹mid› over t in (0, 1))\n"
    );
    // and the instance is itself alone
    assert_eq!(
        lit(src, "t0", 1),
        "component Rung(a: point) {\n  mid := point\n  a distance(5) mid\n}\n\
         p := point\n«t0» := Rung(p)\nt1 := Rung(p)\n‹t0›.mid distance(3) t1.mid\n\
         k := trace(Rung(p).mid over t in (0, 1))\n"
    );
}

#[test]
fn a_member_nothing_declares_is_one_per_thing_it_is_read_of() {
    let src = "l := line\nm := line\nl.p1 coincident m.p1\nhorizontal l\nl.p1 distance(4) m.p2\n";
    assert_eq!(
        lit(src, "p1", 0),
        "l := line\nm := line\nl.‹p1› coincident m.p1\nhorizontal l\nl.‹p1› distance(4) m.p2\n"
    );
}

#[test]
fn a_modules_names_are_read_through_its_use() {
    let src = "use std\nin std.front {\n  p := point\n}\nin std.top {\n  q := point\n}\n\
               x := std.front\n";
    assert_eq!(
        lit(src, "std", 1),
        "use «std»\nin ‹std›.front {\n  p := point\n}\nin ‹std›.top {\n  q := point\n}\n\
         x := ‹std›.front\n"
    );
    assert_eq!(
        lit(src, "front", 0),
        "use std\nin std.‹front› {\n  p := point\n}\nin std.top {\n  q := point\n}\n\
         x := std.‹front›\n"
    );
    let index = symbols(src);
    assert_eq!(index.symbols[index.at(4).unwrap().symbol].kind, SymbolKind::Module);
}

#[test]
fn a_blocks_binder_is_its_own_and_its_declarations_the_documents() {
    let src = "repeat 3 as i {\n  p := point hint((i * 10, 0))\n}\n\
               repeat 2 as i {\n  q := point hint((i, 1))\n}\n\
               p[0] distance(5) p[1]\n";
    assert_eq!(
        lit(src, "i", 0),
        "repeat 3 as «i» {\n  p := point hint((‹i› * 10, 0))\n}\n\
         repeat 2 as i {\n  q := point hint((i, 1))\n}\n\
         p[0] distance(5) p[1]\n"
    );
    assert_eq!(
        lit(src, "p", 1),
        "repeat 3 as i {\n  «p» := point hint((i * 10, 0))\n}\n\
         repeat 2 as i {\n  q := point hint((i, 1))\n}\n\
         ‹p›[0] distance(5) ‹p›[1]\n"
    );
}

#[test]
fn next_and_prev_read_the_blocks_own_names() {
    let src = "cycle 4 as i {\n  v := point\n  (e := line(v, next.v))\n}\n";
    assert_eq!(
        lit(src, "v", 2),
        "cycle 4 as i {\n  «v» := point\n  (e := line(‹v›, next.‹v›))\n}\n"
    );
}

#[test]
fn a_sets_point_is_bound_for_its_body() {
    let src = "p := point\nc := point\nr := 5\n\
               ball := { p | p distance(r) c }\nhole := { q | q distance(r) p }\n";
    assert_eq!(
        lit(src, "p", 1),
        "p := point\nc := point\nr := 5\n\
         ball := { «p» | ‹p› distance(r) c }\nhole := { q | q distance(r) p }\n"
    );
    assert_eq!(
        lit(src, "p", 0),
        "«p» := point\nc := point\nr := 5\n\
         ball := { p | p distance(r) c }\nhole := { q | q distance(r) ‹p› }\n"
    );
}

#[test]
fn a_relation_words_operands_and_parameters_are_its_own() {
    let src = "use std\nd := 3\n\
               b right_of(d) a := {\n  a.y == b.y\n  b.x - a.x == d\n}\n\
               p := point\nq := point\nq right_of(d: d) p\n";
    // the parameter, from its use in the body, and the label that fills it
    assert_eq!(
        lit(src, "d", 3),
        "use std\nd := 3\n\
         b right_of(«d») a := {\n  a.y == b.y\n  b.x - a.x == ‹d›\n}\n\
         p := point\nq := point\nq right_of(‹d›: d) p\n"
    );
    // the document's own `d`, which the call hands it
    assert_eq!(
        lit(src, "d", 0),
        "use std\n«d» := 3\n\
         b right_of(d) a := {\n  a.y == b.y\n  b.x - a.x == d\n}\n\
         p := point\nq := point\nq right_of(d: ‹d›) p\n"
    );
    assert_eq!(
        lit(src, "right_of", 1),
        "use std\nd := 3\n\
         b «right_of»(d) a := {\n  a.y == b.y\n  b.x - a.x == d\n}\n\
         p := point\nq := point\nq ‹right_of›(d: d) p\n"
    );
    // the operand, and its members through it
    assert_eq!(
        lit(src, "a", 0),
        "use std\nd := 3\n\
         b right_of(d) «a» := {\n  ‹a›.y == b.y\n  b.x - ‹a›.x == d\n}\n\
         p := point\nq := point\nq right_of(d: d) p\n"
    );
}

#[test]
fn a_member_typed_by_an_interface_is_the_interfaces() {
    let src = "interface Size { length: Length }\n\
               interface Layout {\n  origin: point\n  size: Size\n}\n\
               component Bar(layout: Layout) {\n  tip := point\n  \
               layout.origin distance(layout.size.length) tip\n}\n";
    assert_eq!(
        lit(src, "length", 0),
        "interface Size { «length»: Length }\n\
         interface Layout {\n  origin: point\n  size: Size\n}\n\
         component Bar(layout: Layout) {\n  tip := point\n  \
         layout.origin distance(layout.size.‹length›) tip\n}\n"
    );
    assert_eq!(
        lit(src, "Layout", 1),
        "interface Size { length: Length }\n\
         interface «Layout» {\n  origin: point\n  size: Size\n}\n\
         component Bar(layout: ‹Layout›) {\n  tip := point\n  \
         layout.origin distance(layout.size.length) tip\n}\n"
    );
}

#[test]
fn a_groups_labels_are_its_members() {
    let src = "o := point\ndims := {width: 20, at: o}\nw := dims.width * 2\n";
    assert_eq!(
        lit(src, "width", 1),
        "o := point\ndims := {«width»: 20, at: o}\nw := dims.‹width› * 2\n"
    );
    assert_eq!(
        lit(src, "o", 1),
        "«o» := point\ndims := {width: 20, at: ‹o›}\nw := dims.width * 2\n"
    );
}

#[test]
fn a_name_nothing_gives_lights_its_other_spellings_in_its_body() {
    // half-typed: `widht` is given nowhere, and still the two places it is written are one
    let src = "a := point\nb := point\na distance(widht) b\nb distance(widht) a\n\
               component C(p: point) {\n  q := point\n  p distance(widht) q\n}\n";
    assert_eq!(
        lit(src, "widht", 0),
        "a := point\nb := point\na distance(‹widht›) b\nb distance(‹widht›) a\n\
         component C(p: point) {\n  q := point\n  p distance(widht) q\n}\n"
    );
    let index = symbols(src);
    let at = src.find("widht").unwrap();
    assert_eq!(index.symbols[index.at(at).unwrap().symbol].kind, SymbolKind::Unknown);
}

#[test]
fn a_definition_reads_its_files_constants() {
    let src = "r := 3mm\ncomponent Pin(c: point) {\n  k := circle(center: c)\n  radius(r) k\n}\n";
    assert_eq!(
        lit(src, "r", 1),
        "«r» := 3mm\ncomponent Pin(c: point) {\n  k := circle(center: c)\n  radius(‹r›) k\n}\n"
    );
}

/// Every model in the library and the standard library is indexed end to end: in order, never
/// overlapping, every occurrence the word its symbol is named, and none of a word the colouring
/// says is no name — the index asked of whatever the corpus writes.
#[test]
fn the_corpus_is_indexed() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let mut files = Vec::new();
    let mut dirs = vec![root.join("examples"), root.join("lib")];
    while let Some(d) = dirs.pop() {
        for e in std::fs::read_dir(&d).expect("a directory") {
            let p = e.expect("an entry").path();
            if p.is_dir() {
                dirs.push(p);
            } else if p.extension().is_some_and(|x| x == "sv") {
                files.push(p);
            }
        }
    }
    assert!(files.len() > 50, "the corpus is there");
    let mut named = 0usize;
    for f in files {
        let src = std::fs::read_to_string(&f).expect("readable");
        let index = symbols(&src);
        let mut end = 0u32;
        for o in &index.occurrences {
            assert!(o.span.lo >= end && o.span.lo < o.span.hi, "{f:?}: at {}", o.span.lo);
            let word = o.span.slice(&src);
            assert_eq!(word, index.symbols[o.symbol].name, "{f:?}: at {}", o.span.lo);
            end = o.span.hi;
        }
        named += index.occurrences.len();
    }
    assert!(named > 1000, "the corpus names things: {named}");
}
