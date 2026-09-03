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
    let doc = "use demo.parts\npoint o hint(x: 0, y: 0)\nground o\nr: Rung(o)\n";
    let module = "component Rung(a: point) {\n  point b\n  line e(a, b)\n  horizontal e\n  a distance(10) b\n}\n";
    let name = "demo.parts";
    unsafe {
        // what the document asks for, as written
        assert_eq!(take(gcs_program_uses(doc.as_ptr(), doc.len())), "[\"demo.parts\"]");
        // nothing handed over, nothing in the library: E070 at the `use`
        assert!(report_of(doc).contains("E070"), "an unknown module is E070");
        gcs_module_set(name.as_ptr(), name.len(), module.as_ptr(), module.len());
        let r = report_of(doc);
        assert!(!r.contains("E070"), "the host's module resolves the use: {r}");
        gcs_module_forget();
        assert!(report_of(doc).contains("E070"), "forgotten, the use is unresolved again");
    }
}
