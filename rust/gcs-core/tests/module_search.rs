//! Where a host looks for a module (#993 stage 2): `modules::search_paths` names `NAME.sv`, then
//! `NAME.step`, then `NAME.stp` in each directory, the model's own first and then its ancestors,
//! nearest first; `modules::is_step` tells an exchange file from Solvent by its first token, and
//! `modules::exchange` a path found by an exchange file's name.  The core opens no file: these are
//! the names every host tries, in the order it tries them.

use gcs_core::modules::{decode, exchange, is_step, search_paths, Found};

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

/// **A document is never its own module** (#993 stage 2): the candidate that is the document
/// itself is not named, however its path is written, so `foo.sv`'s `use foo` reads `foo.step`, and
/// `engine/parts.sv` does not find itself as `../engine/parts.sv`.
#[test]
fn a_document_is_never_among_its_own_candidates() {
    assert_eq!(search_paths("foo", "foo.sv"), ["foo.step", "foo.stp"]);
    assert_eq!(search_paths("foo", "./foo.sv")[..3], ["foo.step", "foo.stp", "../foo.sv"]);
    assert_eq!(search_paths("foo", "/home/a/foo.sv")[..2], ["foo.step", "foo.stp"]);
    assert_eq!(search_paths("foo", "parts\\foo.sv")[..2], ["foo.step", "foo.stp"]);
    let parts = search_paths("engine.parts", "engine/parts.sv");
    assert!(!parts.contains(&"../engine/parts.sv".to_string()), "{parts:?}");
    assert!(parts.contains(&"../engine/parts.step".to_string()), "{parts:?}");
    // a document of another name, or another kind, is a candidate as before
    assert_eq!(search_paths("foo", "foo.svd")[0], "foo.sv");
    assert_eq!(search_paths("foo", "bar/../foo.sv")[..3], ["foo.step", "foo.stp", "../foo.sv"]);
    // linked, a `use` nothing but the document answers is no module (E070), said once at the use
    let mut p = gcs_core::syntax::parse("unit mm\nuse self\n").0;
    let diags = gcs_core::modules::link(&mut p, &mut |name| {
        search_paths(name, "self.sv").iter().find(|c| *c == "self.sv")
            .map(|_| "unit mm\n".to_string())
    });
    assert_eq!(diags.len(), 1);
    assert!(diags[0].message.contains("no module `self`"), "{:?}", diags[0]);
}

/// **A file found by an exchange file's name is one whatever its text** (#993 stage 2): the host
/// says where it found a text (`Found::at`), so an empty or non-STEP `NAME.step`/`NAME.stp` is
/// given to the kernel's reader, which refuses it at the `use` (E072) in its own words; found by
/// another name, or handed on by none, the same text is what its content says.  `decode` reads the
/// file's own text, lossily only where it is an exchange file.
#[test]
fn a_file_found_as_an_exchange_file_is_read_as_one_and_refused_if_it_is_none() {
    let file = include_str!("step_file/drilled_ap214.step");
    assert_eq!(decode("parts/bracket.step", file.as_bytes().to_vec()).unwrap(), file);
    assert_eq!(decode("", file.as_bytes().to_vec()).unwrap(), file);
    let doc = "unit mm\nuse bracket\npart := solid(bracket)\n";
    let errors = |found: Found| {
        let mut p = gcs_core::syntax::parse(doc).0;
        let linked = gcs_core::modules::link(&mut p, &mut |_| Some(found.clone()));
        let e = gcs_core::program::elaborate(&p);
        let said: Vec<String> = linked.iter().chain(e.errors())
            .map(|d| format!("{:?}: {}", d.code, d.message)).collect();
        (p, said)
    };
    for text in ["", "component Rung(a: point) {\n}\n", "unit mm\n"] {
        for path in ["bracket.step", "parts/bracket.stp"] {
            assert!(exchange(path));
            // the file's own text, whatever it holds
            let decoded = decode(path, text.as_bytes().to_vec()).unwrap();
            assert_eq!(decoded, text);
            let (p, said) = errors(Found::at(path, decoded));
            assert!(p.modules[0].given.is_some() && p.modules[0].text.is_empty());
            assert_eq!(said.len(), 1, "{said:?}");
            assert!(said[0].starts_with("E072: `bracket`: ")
                    && said[0].contains("the file does not start `ISO-10303-21;`"), "{said:?}");
        }
        // found as a Solvent document, or with no name, it is read as one
        assert!(!exchange("bracket.sv") && !exchange(""));
        for found in [Found::at("bracket.sv", text.to_string()), Found::from(text.to_string())] {
            let (p, said) = errors(found);
            assert!(p.modules[0].given.is_none());
            assert!(said.iter().all(|s| !s.starts_with("E072")), "{said:?}");
        }
    }
    // a Solvent document that is not UTF-8 is refused; found as an exchange file, read lossily
    assert!(decode("dims.sv", b"w := 1 // M\xfcller\n".to_vec()).is_err());
    assert_eq!(decode("dims.step", b"w := 1 // M\xfcller\n".to_vec()).unwrap(),
               "w := 1 // M\u{fffd}ller\n");
}

/// **A text read before is found by the digest the host took** (#993 stage 2): a fresh link of a
/// file handed with its digest finds the reading the last link made, the text never hashed or
/// compared again; a text of another length under that digest is another file, read afresh.
#[test]
fn a_text_read_before_is_found_by_its_digest() {
    let file = include_str!("step_file/drilled_ap214.step");
    let read = |found: &Found| {
        let mut p = gcs_core::syntax::parse("unit mm\nuse bracket\n").0;
        assert!(gcs_core::modules::link(&mut p, &mut |_| Some(found.clone())).is_empty());
        p.modules[0].given.clone().expect("a given module")
    };
    let digest = Some(gcs_core::modules::digest(file));
    let first = read(&Found { digest, ..Found::at("bracket.step", file.to_string()) });
    let again = read(&Found { digest, ..Found::at("other/bracket.stp", file.to_string()) });
    assert!(std::sync::Arc::ptr_eq(&first, &again));
    assert_eq!((again.digest, again.len), (digest.unwrap(), file.len()));
    let other = format!("{file}\n");
    let fresh = read(&Found { digest, ..Found::at("bracket.step", other) });
    assert!(!std::sync::Arc::ptr_eq(&first, &fresh));
}
