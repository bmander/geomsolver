//! One cause, said once (#572): a body that cannot be built is refused at the operand that cannot
//! be, and each statement built on it says once that it is not built (N080), pointing back at the
//! refusal — never once per piece of a region a round made, nor inside the word `std` defines.
use gcs_core::program::{self, Code, Diag, Severity};
use gcs_core::solve::{solve, SolveOpts};

/// A slot drafted 8° over 16 mm: each end's half circle (r 2) is drawn smaller than the draft's
/// 2.25 mm inset, so the draft takes it to nothing.  The body has one bad operand; `rounds` of
/// its two rounds follow.
fn slot(rounds: usize) -> String {
    let mut src = String::from("\
unit mm
use std
use std.solids
use std.rounds (round)
in std.top {
  c0 := point
  c1 := point
  fix((0, 0)) c0
  fix((20, 0)) c1
  s0 := point hint((0, -2))
  s1 := point hint((20, -2))
  s2 := point hint((20, 2))
  s3 := point hint((0, 2))
  outline := (lower := line(s0, s1)) -> tangent (east := arc(center: c1)) ->
             tangent (upper := line(s2, s3)) -> tangent (west := arc(center: c0)) -> close
  radius(2mm) east
  radius(2mm) west
  horizontal lower
}
slot := solid(std.solids.Drafted(outline, from: -16mm, to: 0mm, draft: 8deg))
body := solid(slot)
");
    for cap in ["far", "near"].into_iter().take(rounds) {
        src.push_str(&format!("body.slot.{cap} round(r: 0.5mm) body.slot\n"));
    }
    src
}

/// The document's diagnostics of its solids, each with its line, solved first.
fn said(src: &str) -> (Vec<Diag>, Vec<u32>) {
    let mut e = crate::common::build(src);
    solve(&mut e.sketch, SolveOpts::default());
    let diags = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    let lines = diags.iter().map(|d| d.at(e.text()).0).collect();
    (diags, lines)
}

#[test]
fn an_unbuildable_operand_is_said_once_and_each_round_on_it_once() {
    for rounds in 0..=2 {
        let (diags, lines) = said(&slot(rounds));
        assert_eq!(diags.len(), 1 + rounds, "{rounds} rounds: {diags:#?}");
        // the operand's own words, where the operand is
        let root = &diags[0];
        assert_eq!((root.code, lines[0]), (Code::E080, 20), "{root:?}");
        assert!(root.message.starts_with("`slot`: a draft of `8deg` takes the side"), "{root:?}");
        // each round once, at its own line, pointing back at the operand
        for (k, d) in diags.iter().enumerate().skip(1) {
            let at = (d.code, d.severity(), lines[k]);
            assert_eq!(at, (Code::N080, Severity::Note, 21 + k as u32));
            assert_eq!(d.cause, Some(root.span), "{d:?}");
            assert_eq!(d.message, "not built: it is built on `body`, made of `slot` (20:1), which \
                could not be built");
        }
    }
}

#[test]
fn no_piece_of_a_region_is_a_diagnostic() {
    let (diags, _) = said(&slot(2));
    // a round's pieces, `taken` and `filled` under their generated paths, are never named
    for d in &diags {
        assert!(!d.message.contains('#') && !d.message.contains("taken")
            && !d.message.contains("filled"), "{d:?}");
    }
}

/// **A refusal names what the document reads, where the document made it** (#948): a key a
/// block made is said by its path (`#71.0.#7.1.taken.round` is `taken[1].round`), an issue's
/// number is left alone; a name two rounds' pieces both go by places neither, so a refusal of one
/// of them names it by its key (`Sketch::solid_said`), which places it; and the bullnose's root
/// round, crossing the seam off its section, refused in an export after another round over the
/// slab, is said at the root round in its own words, no key left in them.
#[test]
fn a_refusal_is_said_at_its_round_by_the_names_the_document_reads() {
    use gcs_core::vocab::public_words;
    assert_eq!(public_words("`#71.0.#7.1.taken.round` with `b1.post.rim` (#218): #3.0.cut"),
               "`taken[1].round` with `b1.post.rim` (#218): cut[0]");
    let no_key = "a key `#12` is none, nor is #4's";
    assert_eq!(public_words(no_key), no_key);
    let src = include_str!("../../examples/rounds/bullnose.sv")
        .replace("fix((0, 0)) q", "fix((0, -1)) q")
        .replace("part.boss round(r: 1mm) part.slab",
                 "part.slab.cd round(r: 1mm) part.slab\npart.boss round(r: 1mm) part.slab");
    let mut e = crate::common::build(&src);
    solve(&mut e.sketch, SolveOpts::default());
    let sk = &e.sketch;
    let line = |span: gcs_core::syntax::Span| {
        let (l, _) = gcs_core::syntax::line_col(e.program.text(), span.lo);
        src.lines().nth(l as usize - 1).map(str::trim).unwrap_or_default().to_string()
    };
    // a name both rounds' pieces go by: placed by neither, each placed by its key at its round
    let at = |i: usize| {
        let by_key = format!("`{}`: refused", sk.solids[i].name);
        program::refusal_site(sk, &e.map, &e.program, &by_key).map(|(span, _)| line(span))
    };
    let names: Vec<String> = (0..sk.solids.len()).map(|i| sk.solid_name(i)).collect();
    let shared = (0..names.len()).find(|&i| names[i].contains('[')
        && (0..names.len()).any(|j| j != i && names[j] == names[i] && at(j) != at(i)))
        .unwrap_or_else(|| panic!("a piece's name both rounds make: {names:?}"));
    assert_eq!(sk.solid_said(shared), sk.solids[shared].name);
    assert_eq!(sk.solid_said(0), "slab", "a name one solid goes by is said");
    let by_name = format!("`{}`: refused", names[shared]);
    assert!(program::refusal_site(sk, &e.map, &e.program, &by_name).is_none(), "{by_name}");
    let by_key = format!("`{}`: refused", sk.solid_said(shared));
    let (span, _) = program::refusal_site(sk, &e.map, &e.program, &by_key).expect("placed");
    assert!(line(span).contains(" round(r: 1mm) "), "{}", line(span));
    // the export's refusal, at the root round, in its own words
    use gcs_core::brep::{export, sweep::Say};
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let part = e.map.ent_named("part").unwrap().i();
    let why = export::exact(sk, part, None, None, &say).err().expect("refused").message;
    let (span, _, from) = program::refusal_cause(sk, &e.map, &e.program, &why).expect("placed");
    assert_eq!(line(span), "part.boss round(r: 1mm) part.slab", "{why}");
    let words = public_words(&why[from..]);
    assert!(!words.contains('#') && words.contains("crosses their seam"), "{words}");
}
