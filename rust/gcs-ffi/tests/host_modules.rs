//! A module the host hands over resolves a `use` before the library's copy does — the CLI's
//! "beside the document first" rule, given to a host with no filesystem.

use gcs::*;

/// A string block the ABI handed back — `[u32 len][bytes]`, owned by the caller — as a `String`.
fn take(p: *mut u8) -> String {
    unsafe {
        let len = gcs_str_len(p) as usize;
        let s = std::str::from_utf8(std::slice::from_raw_parts(gcs_str_ptr(p), len))
            .unwrap_or("")
            .to_string();
        gcs_str_free(p);
        s
    }
}

fn report_of(doc: &str) -> String {
    unsafe {
        let h = gcs_program_elaborate(doc.as_ptr(), doc.len());
        assert!(!h.is_null());
        let r = take(gcs_elab_report(h));
        gcs_elab_free(h);
        r
    }
}

#[test]
fn a_host_module_resolves_a_use_and_is_forgotten_on_request() {
    let doc = "use std\nuse demo.parts\nin std.front {\no := point\nfix((0, 0)) o\nr := Rung(o)\n}\n";
    let module = "component Rung(a: point) {\n  b := point\n  e := line(a, b)\n  horizontal e\n  a distance(10) b\n}\n";
    let name = "demo.parts";
    unsafe {
        // what the document asks for, as written
        assert_eq!(take(gcs_program_uses(doc.as_ptr(), doc.len())), "[\"std\",\"demo.parts\"]");
        // nothing handed over, nothing in the library: E070 at the `use`
        assert!(report_of(doc).contains("E070"), "an unknown module is E070");
        let path = "demo/parts.sv";
        gcs_module_set(name.as_ptr(), name.len(), path.as_ptr(), path.len(),
                       module.as_ptr(), module.len());
        assert_eq!(take(gcs_module_source(name.as_ptr(), name.len())), module);
        // and the path it was handed with, which a host passing it on passes too
        assert_eq!(take(gcs_module_path(name.as_ptr(), name.len())), path);
        assert!(gcs_module_path(b"std".as_ptr(), 3).is_null(), "the library's is no host's");
        // its digest names its text, the core's own, without the text crossing the ABI
        assert_eq!(take(gcs_module_digest(name.as_ptr(), name.len())),
                   format!("{:016x}", gcs_core::modules::digest(module)));
        assert_eq!(take(gcs_module_digest(b"std".as_ptr(), 3)).len(), 16, "the library's too");
        assert!(gcs_module_digest(b"nosuch".as_ptr(), 6).is_null());
        assert!(take(gcs_module_source(b"std".as_ptr(), 3)).contains("component StandardDatums"));
        let r = report_of(doc);
        assert!(!r.contains("E070"), "the host's module resolves the use: {r}");
        gcs_module_forget();
        assert!(gcs_module_source(name.as_ptr(), name.len()).is_null());
        assert!(gcs_module_digest(name.as_ptr(), name.len()).is_null());
        assert!(gcs_module_path(name.as_ptr(), name.len()).is_null());
        assert!(report_of(doc).contains("E070"), "forgotten, the use is unresolved again");
    }
}

/// An exchange file handed over as a module (#993 stage 2) is kept as the file's text, its bytes
/// that are not UTF-8 (Latin-1 in a string) read lossily; it `use`s nothing, answered without the
/// Solvent parser; and a text that is neither UTF-8 nor an exchange file is refused aloud, never
/// stored as an empty module.
#[test]
fn an_exchange_file_is_kept_lossily_and_uses_nothing() {
    let file = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"),
                                     "/../gcs-core/tests/step_file/drilled_ap214.step")).unwrap();
    // the header's author in Latin-1, as real files carry: `Müller` with `ü` as the one byte 0xFC
    let at = file.windows(8).position(|w| w == b"'Author'").unwrap();
    let mut latin1 = file.clone();
    latin1.splice(at..at + 8, b"'M\xfcller'".iter().copied());
    assert!(std::str::from_utf8(&latin1).is_err());
    let name = "vendor.bracket";
    unsafe {
        // where the host found it, or no path at all (a worker handed the page's texts)
        for (text, path) in [(&file, "vendor/bracket.step"), (&latin1, ""), (&latin1, "vendor/bracket.stp")] {
            assert_eq!(gcs_module_set(name.as_ptr(), name.len(), path.as_ptr(), path.len(),
                                      text.as_ptr(), text.len()), 1);
            assert_eq!(gcs_module_provided(name.as_ptr(), name.len()), 1);
            let kept = take(gcs_module_source(name.as_ptr(), name.len()));
            assert!(kept.starts_with("ISO-10303-21;") && kept.len() + 2 >= text.len(), "{}", kept.len());
            assert_eq!(take(gcs_program_uses(text.as_ptr(), text.len())), "[]");
        }
        assert!(take(gcs_module_source(name.as_ptr(), name.len())).contains("('M\u{fffd}ller')"));
        // Solvent that is not UTF-8 is refused, and the older text under the name goes with it
        let bad = b"component Rung(a: point) {\n  // M\xfcller\n}\n";
        let path = "vendor/bracket.sv";
        assert_eq!(gcs_module_set(name.as_ptr(), name.len(), path.as_ptr(), path.len(),
                                  bad.as_ptr(), bad.len()), 0);
        let why = take(gcs_last_error());
        assert!(why.contains("vendor.bracket") && why.contains("UTF-8"), "{why}");
        assert_eq!(gcs_module_provided(name.as_ptr(), name.len()), 0);
        assert!(gcs_module_source(name.as_ptr(), name.len()).is_null());
        gcs_module_forget();
    }
}

/// A file the host found as `NAME.step` whose text is no exchange file — empty, or Solvent-looking
/// text — is handed over as one, with the path it was found at, and refused by the kernel's reader
/// at the `use` (E072), once and in its words: never linked as an empty Solvent module (only E101
/// at the read), never parsed as Solvent (a flood of E100).  The core keeps the file's own text,
/// marked by nothing: where it was found is its path, which a host passes on with it.
#[test]
fn a_file_found_as_an_exchange_file_that_is_none_is_refused_at_the_use() {
    let doc = "unit mm\nuse bracket\npart := solid(bracket)\n";
    let name = "bracket";
    for text in ["", "hello world\nthis is no step\n"] {
        unsafe {
            let path = "bracket.step";
            assert_eq!(gcs_module_set(name.as_ptr(), name.len(), path.as_ptr(), path.len(),
                                      text.as_ptr(), text.len()), 1);
            assert_eq!(take(gcs_module_source(name.as_ptr(), name.len())), text);
            assert_eq!(take(gcs_module_path(name.as_ptr(), name.len())), path);
            let r = report_of(doc);
            assert!(r.contains("E072") && r.contains("`bracket`: ")
                    && r.contains("the file does not start `ISO-10303-21;`"), "{text:?}: {r}");
            assert!(!r.contains("E100") && !r.contains("E101"), "said once: {r}");
            // handed on as a worker is handed it, with its path, it is read the same
            gcs_module_forget();
            assert_eq!(gcs_module_set(name.as_ptr(), name.len(), path.as_ptr(), path.len(),
                                      text.as_ptr(), text.len()), 1);
            assert_eq!(report_of(doc), r);
            gcs_module_forget();
        }
    }
}
