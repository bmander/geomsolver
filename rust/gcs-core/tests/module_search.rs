//! Where a host looks for a module (#993 stage 2): `modules::search_paths` names `NAME.sv`, then
//! `NAME.step`, then `NAME.stp` in each directory, the model's own first and then its ancestors,
//! nearest first; `modules::is_step` tells an exchange file from Solvent by its first token.  The
//! core opens no file: these are the names every host tries, in the order it tries them.

use gcs_core::modules::{is_step, search_paths};

#[test]
fn each_directory_is_tried_for_every_kind_nearest_first() {
    assert_eq!(search_paths("bracket", "doc.sv"), ["bracket.sv", "bracket.step", "bracket.stp"]);
    assert_eq!(search_paths("vendor.bracket", "parts/arm/doc.sv"), [
        "vendor/bracket.sv", "vendor/bracket.step", "vendor/bracket.stp",
        "../vendor/bracket.sv", "../vendor/bracket.step", "../vendor/bracket.stp",
        "../../vendor/bracket.sv", "../../vendor/bracket.step", "../../vendor/bracket.stp",
    ]);
    // a Windows path counts its directories the same
    assert_eq!(search_paths("b", "parts\\doc.sv").len(), 6);
}

/// The first candidate that exists is the module: in one directory the Solvent document wins
/// over an exchange file, and a nearer exchange file wins over a farther document.
#[test]
fn the_first_that_exists_wins() {
    let first = |files: &[&str]| -> Option<String> {
        search_paths("bracket", "parts/doc.sv").into_iter().find(|c| files.contains(&c.as_str()))
    };
    assert_eq!(first(&["bracket.stp", "bracket.step", "bracket.sv"]).as_deref(), Some("bracket.sv"));
    assert_eq!(first(&["bracket.stp", "bracket.step"]).as_deref(), Some("bracket.step"));
    assert_eq!(first(&["bracket.stp"]).as_deref(), Some("bracket.stp"));
    assert_eq!(first(&["../bracket.sv", "bracket.step"]).as_deref(), Some("bracket.step"));
    assert_eq!(first(&["../bracket.sv", "../bracket.stp"]).as_deref(), Some("../bracket.sv"));
    assert_eq!(first(&[]), None);
}

#[test]
fn an_exchange_file_is_told_by_its_first_token() {
    assert!(is_step("ISO-10303-21;\nHEADER;\n"));
    assert!(is_step("\u{feff}  \r\n\tISO-10303-21;\nHEADER;"));
    let file = include_str!("step_file/drilled_ap214.step");
    assert!(is_step(file));
    assert!(!is_step("unit mm\nuse bracket\n"));
    assert!(!is_step("// ISO-10303-21;\n"));
    assert!(!is_step("ISO-10303-21\n"));
    assert!(!is_step(""));
}
