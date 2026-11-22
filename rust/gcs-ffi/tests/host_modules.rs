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
        gcs_module_set(name.as_ptr(), name.len(), module.as_ptr(), module.len());
        assert_eq!(take(gcs_module_source(name.as_ptr(), name.len())), module);
        assert!(take(gcs_module_source(b"std".as_ptr(), 3)).contains("component StandardDatums"));
        let r = report_of(doc);
        assert!(!r.contains("E070"), "the host's module resolves the use: {r}");
        gcs_module_forget();
        assert!(gcs_module_source(name.as_ptr(), name.len()).is_null());
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
        for text in [&file, &latin1] {
            assert_eq!(gcs_module_set(name.as_ptr(), name.len(), text.as_ptr(), text.len()), 1);
            assert_eq!(gcs_module_provided(name.as_ptr(), name.len()), 1);
            let kept = take(gcs_module_source(name.as_ptr(), name.len()));
            assert!(kept.starts_with("ISO-10303-21;") && kept.len() + 2 >= text.len(), "{}", kept.len());
            assert_eq!(take(gcs_program_uses(text.as_ptr(), text.len())), "[]");
        }
        assert!(take(gcs_module_source(name.as_ptr(), name.len())).contains("('M\u{fffd}ller')"));
        // Solvent that is not UTF-8 is refused, and the older text under the name goes with it
        let bad = b"component Rung(a: point) {\n  // M\xfcller\n}\n";
        assert_eq!(gcs_module_set(name.as_ptr(), name.len(), bad.as_ptr(), bad.len()), 0);
        let why = take(gcs_last_error());
        assert!(why.contains("vendor.bracket") && why.contains("UTF-8"), "{why}");
        assert_eq!(gcs_module_provided(name.as_ptr(), name.len()), 0);
        assert!(gcs_module_source(name.as_ptr(), name.len()).is_null());
        gcs_module_forget();
    }
}
