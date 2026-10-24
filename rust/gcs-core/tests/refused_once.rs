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
use std (round)
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
slot := solid(outline, from: -16mm, to: 0mm, draft: 8deg)
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
        assert_eq!((root.code, lines[0]), (Code::E080, 19), "{root:?}");
        assert!(root.message.starts_with("`slot`: a draft of `8deg` takes the side"), "{root:?}");
        // each round once, at its own line, pointing back at the operand
        for (k, d) in diags.iter().enumerate().skip(1) {
            let at = (d.code, d.severity(), lines[k]);
            assert_eq!(at, (Code::N080, Severity::Note, 20 + k as u32));
            assert_eq!(d.cause, Some(root.span), "{d:?}");
            assert_eq!(d.message, "not built: it is built on `body`, made of `slot` (19:1), which \
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
