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

/// **…and so is a round straight over the operand** (#962): rounded where it is written, the slot
/// is a body and its prism a stock split off it under its name, so the prism's own refusal
/// (`` `slot`: … ``) named the body built on it, the body's rounds built on the body, and nothing
/// was a root: every piece of the round said the draft again.  Now the slot says it once, at its
/// statement — which says nothing more of the body it is — and the round that it is not built.
#[test]
fn a_round_straight_over_an_unbuildable_operand_is_said_once() {
    let src = format!("{}slot.far round(r: 0.5mm) slot\n", slot(0));
    let (diags, lines) = said(&src);
    assert_eq!(diags.len(), 2, "{diags:#?}");
    assert_eq!((diags[0].code, lines[0]), (Code::E080, 20), "{diags:#?}");
    assert!(diags[0].message.starts_with("`slot`: a draft of `8deg` takes the side"), "{diags:#?}");
    assert_eq!((diags[1].code, lines[1]), (Code::N080, 22), "{diags:#?}");
    assert_eq!(diags[1].cause, Some(diags[0].span));
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

/// A plate 40 × 30 whose south edge stops `gap` short of its corner, the rest of the way a line
/// `stub` from `b` to `b2`; swept 10 deep as `part`, and `rounds` rounds over it after.  At no gap
/// the stub has no length.
fn stubbed(gap: &str, rounds: &[&str]) -> String {
    let mut src = format!("\
unit mm
use std
use std.solids
use std.rounds (round, chamfer)
in std.top {{
  a := point
  b := point
  b2 := point
  c := point
  d := point
  fix((0, 0)) a
  fix((40 - {gap}, 0)) b
  fix((40, 0)) b2
  fix((40, 30)) c
  fix((0, 30)) d
  south := line(a, b)
  stub := line(b, b2)
  east := line(b2, c)
  north := line(c, d)
  west := line(d, a)
  outline := face(south, stub, east, north, west)
}}
part := solid(std.solids.Prism(outline, from: 0mm, to: 10mm))
");
    for r in rounds {
        src.push_str(r);
        src.push('\n');
    }
    src
}

/// **A face's own fault is the face's, said once at the face** (#962): an edge of a face's outline
/// with no length (here a line whose two points coincide) is a fault of the face, whatever is
/// built of it — said at the face, in its words (the edge, its points, how far apart they are),
/// never as a kernel's `loop 0`; the prism swept from it says only that it is not built, and each
/// round over the prism the same, once each, never once per piece.  A stub of a micron is a face
/// like any other, and the rounds over it are built.
#[test]
fn an_edge_of_no_length_is_said_once_at_its_face() {
    let rounds = ["round(r: 1mm) part", "part.near chamfer(d: 1mm) part"];
    for k in 0..=2 {
        let src = stubbed("0", &rounds[..k]);
        let (diags, lines) = said(&src);
        assert_eq!(diags.len(), 2 + k, "{k} rounds: {diags:#?}");
        let root = &diags[0];
        assert_eq!((root.code, lines[0]), (Code::E080, 21), "{root:?}");
        assert_eq!(root.message, "`outline`'s edge `stub` has no length: it runs from `b` to \
            `b2`, which coincide, so no solid can be made of `outline`");
        for (j, d) in diags.iter().enumerate().skip(1) {
            assert_eq!((d.code, lines[j]), (Code::N080, 22 + j as u32), "{d:?}");
            assert_eq!(d.cause, Some(root.span), "{d:?}");
            assert!(d.message.starts_with("not built: it is built on ") && d.message
                .ends_with("`outline` (21:3), which could not be built"), "{d:?}");
        }
        for d in &diags {
            assert!(!d.message.contains("loop") && !d.message.contains("taken"), "{d:?}");
        }
    }
    // short of nothing, how far apart its points are
    let (diags, _) = said(&stubbed("0.000000001", &[]));
    let m = &diags[0].message;
    assert!(m.contains("it runs from `b` to `b2`, which are ") && m.contains("mm apart"), "{m}");
    // a micron is an edge: the face is swept, its volume the plate's
    let src = stubbed("0.001", &[]);
    let (diags, _) = said(&src);
    assert!(diags.is_empty(), "{diags:#?}");
    let e = crate::common::build(&src);
    let part = e.sketch.evaluated_solid(e.map.ent_named("part").unwrap().i(),
        gcs_core::solid::ApproximationPolicy::Report).expect("built");
    assert!((part.volume() - 12000.0).abs() < 1e-6, "{}", part.volume());
}

/// **Every way a face bounds no solid is the face's** (#962): a loop crossing itself (`face_polys`'
/// refusal) is said at the face as an edge of no length is, the prism swept from it not built;
/// a face two solids are swept from is said once, each solid not built; and a revolution's face
/// the same.
#[test]
fn a_face_no_solid_is_made_of_is_said_once_for_every_solid_of_it() {
    let crossed = stubbed("0", &[]).replace("fix((40, 30)) c", "fix((0, 30)) c")
        .replace("fix((0, 30)) d", "fix((40, 30)) d")
        .replace("fix((40 - 0, 0)) b", "fix((20, 0)) b");
    let (diags, lines) = said(&crossed);
    assert_eq!((diags[0].code, lines[0]), (Code::E080, 21), "{diags:#?}");
    assert!(diags[0].message.starts_with("no solid can be made of `outline`: "), "{diags:#?}");
    assert_eq!((diags.len(), diags[1].code, lines[1]), (2, Code::N080, 23), "{diags:#?}");
    // swept twice, and turned
    let twice = stubbed("0", &["other := solid(std.solids.Prism(outline, from: 20mm, to: 30mm))",
        "turned := solid(std.solids.Revolution(outline, about: west))"]);
    let (diags, lines) = said(&twice);
    assert_eq!(diags.iter().filter(|d| d.code == Code::E080).count(), 1, "{diags:#?}");
    assert_eq!(diags.len(), 4, "{diags:#?}");
    assert_eq!(lines[1..], [23, 24, 25]);
    assert!(diags[1..].iter().all(|d| d.code == Code::N080 && d.cause == Some(diags[0].span)));
}
