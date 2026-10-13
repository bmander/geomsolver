//! Solvent: the program a sketch is written as, elaborated, coloured and reported.

use super::*;

/* -- Solvent: the program a sketch is written as ----------------------------------- */

/// Read and elaborate a program.
///
/// Never null except when there is no memory: a program full of errors still comes back, with
/// whatever geometry it could build and the diagnostics beside it, because a panel has to show
/// the drawing *and* the error.  Whether to adopt the result is the caller's, from the report.
#[no_mangle]
pub unsafe extern "C" fn gcs_program_elaborate(ptr: *const u8, len: usize) -> *mut Elaborated {
    guard(std::ptr::null_mut(), move || {
        let src = as_str(ptr, len);
        // the browser has no filesystem, so a `use` resolves against what the host handed over
        // (`gcs_module_set`) and then the library compiled in — the CLI's own order, beside the
        // document first
        let (mut prog, errs) = syntax::parse(src);
        let mut linked = gcs_core::modules::link(&mut prog, &mut resolve_module);
        let mut e = program::elaborate(&prog);
        // the parser's complaints come first, in the order they were found: they are about the
        // text, and everything after them is about a text that was already wrong
        let mut all: Vec<program::Diag> = errs
            .iter()
            .map(|s| program::Diag {
                code: program::Code::E100,
                span: s.span,
                stmt: None,
                message: s.message.clone(),
            })
            .collect();
        all.append(&mut linked);
        all.append(&mut e.diags);
        e.diags = all;
        Box::into_raw(Box::new(e))
    })
}

thread_local! {
    /// Module texts the host has handed over.  `solventc` reads `engine.parts` as
    /// `engine/parts.sv` beside the document and comes to the library second; a browser has no
    /// beside unless its page has a server to ask, and this is where what it fetched goes.
    /// Asked before the library, so a text the host supplies wins over the copy compiled in.
    static HOST_MODULES: RefCell<BTreeMap<String, String>> = RefCell::new(BTreeMap::new());
}

/// Resolve a `use`: what the host handed over first, then the library compiled in.
fn resolve_module(name: &str) -> Option<String> {
    HOST_MODULES
        .with(|m| m.borrow().get(name).cloned())
        .or_else(|| gcs_core::library::resolve(name))
}

/// The module text an elaboration would use: the host's copy, then the built-in library.
/// Returns null for an unknown module; otherwise the caller owns the string block.
#[no_mangle]
pub unsafe extern "C" fn gcs_module_source(ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        resolve_module(as_str(ptr, len)).map_or(std::ptr::null_mut(), out_str)
    })
}

/// 1 when the host handed module `name` over (`gcs_module_set`), 0 when a `use` of it reads the
/// library's copy or nothing: so a host passing its modules on passes only its own.
#[no_mangle]
pub unsafe extern "C" fn gcs_module_provided(ptr: *const u8, len: usize) -> i32 {
    guard(0, move || HOST_MODULES.with(|m| m.borrow().contains_key(as_str(ptr, len))) as i32)
}

/// Hand the core a module's text under the name a `use` asks for (`engine.parts`).  Kept until
/// `gcs_module_forget`; a second `set` under one name replaces the first.  Nothing already
/// elaborated changes: an edit relinks from the texts it has (`program::reparse`), so the table
/// is read only when a program is read whole.
#[no_mangle]
pub unsafe extern "C" fn gcs_module_set(np: *const u8, nn: usize, tp: *const u8, tn: usize) {
    guard((), move || {
        let (name, text) = (as_str(np, nn).to_string(), as_str(tp, tn).to_string());
        HOST_MODULES.with(|m| {
            m.borrow_mut().insert(name, text);
        });
    })
}

/// Forget every module the host handed over: the next program read links against the library.
#[no_mangle]
pub extern "C" fn gcs_module_forget() {
    guard((), || HOST_MODULES.with(|m| m.borrow_mut().clear()))
}

/// The module names a text `use`s, directly, as written: `["std", "engine.parts"]`.  A host
/// with somewhere to fetch from asks this of the document, hands each text over with
/// `gcs_module_set`, and asks it again of each text it fetched, since a module's own `use`s are
/// followed too.  A parse error costs nothing here: the `use` lines that did parse are listed.
#[no_mangle]
pub unsafe extern "C" fn gcs_program_uses(ptr: *const u8, len: usize) -> *mut u8 {
    json_out(move || {
        let (prog, _) = syntax::parse(as_str(ptr, len));
        Json::Arr(prog.uses.iter().map(|u| Json::Str(u.name.clone())).collect())
    })
}

/// Colour a program: `[[class, lo, hi], …]`, in order, over the classified runs only.
///
/// A function of the *text* and nothing else, so a panel colours what somebody is halfway through
/// typing — which never elaborates and is exactly the program being looked at.
#[no_mangle]
pub unsafe extern "C" fn gcs_program_highlight(ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_runs(syntax::highlight(as_str(ptr, len))))
}

/// Colour a drawing (`.svd`), as `gcs_program_highlight` colours a program: the drawing lexer's
/// own scan, in the same `[[class, lo, hi], …]`.
#[no_mangle]
pub unsafe extern "C" fn gcs_drawing_highlight(ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_runs(gcs_core::drawing::highlight(as_str(ptr, len)))
    })
}

fn out_runs(runs: Vec<(syntax::Tint, syntax::Span)>) -> *mut u8 {
    let runs: Vec<Json> = runs
        .into_iter()
        .map(|(t, s)| {
            Json::Arr(vec![
                Json::Str(t.as_str().to_string()),
                Json::Int(s.lo as i64),
                Json::Int(s.hi as i64),
            ])
        })
        .collect();
    out_json(Json::Arr(runs))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_elab_free(h: *mut Elaborated) {
    if !h.is_null() {
        drop(Box::from_raw(h));
    }
}

/// Take the sketch out.  The caller owns it and frees it with `gcs_sketch_free`; a second call
/// returns null, because there was only ever one.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_take_sketch(h: *mut Elaborated) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        let e = &mut *h;
        match e.taken {
            true => std::ptr::null_mut(),
            false => {
                e.taken = true;
                Box::into_raw(Box::new(std::mem::take(&mut e.sketch)))
            }
        }
    })
}

/// The text the program was read from — which a splice may have moved on.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_text(h: *mut Elaborated) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str((*h).text().to_string()))
}

/// `{"ok": bool, "diagnostics": [{severity, code, message, lo, hi, line, col}], "map": {...}}`
///
/// Line and column are computed here, so no binding scans the text a second time and the two
/// cannot disagree about where line 12 is.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_report(h: *mut Elaborated) -> *mut u8 {
    json_out(move || {
        let e = &*h;
        let diags: Vec<Json> = e
            .diags
            .iter()
            .map(|d| {
                let (line, col) = d.at(e.text());
                json::object([
                    ("severity", Json::Str(format!("{:?}", d.severity()).to_lowercase())),
                    ("code", Json::Str(d.code.as_str().to_string())),
                    ("message", Json::Str(d.message.clone())),
                    ("lo", Json::Int(d.span.lo as i64)),
                    ("hi", Json::Int(d.span.hi as i64)),
                    ("line", Json::Int(line as i64)),
                    ("col", Json::Int(col as i64)),
                ])
            })
            .collect();
        let ents: Vec<Json> = e
            .map
            .of_entity
            .iter()
            .map(|(r, site)| {
                Json::Arr(vec![
                    Json::Str(r.kind.as_str().to_string()),
                    Json::Int(r.idx as i64),
                    // what the source calls it, which for an anonymous declaration is nothing:
                    // the key it resolves by is derived from the statement's offset, so a
                    // selection keyed on it would go stale — or land on another entity — after
                    // any edit above it.  The map files a key under resolution alone, so there
                    // is nothing here to screen out; the span still says where it was written.
                    Json::Str(e.map.name_of(*r).cloned().unwrap_or_default()),
                    Json::Int(site.span.lo as i64),
                    Json::Int(site.span.hi as i64),
                ])
            })
            .collect();
        let cons: Vec<Json> = e
            .map
            .of_constraint
            .iter()
            .map(|(id, site)| {
                Json::Arr(vec![
                    Json::Int(*id as i64),
                    Json::Int(site.span.lo as i64),
                    Json::Int(site.span.hi as i64),
                ])
            })
            .collect();
        json::object([
            ("ok", Json::Bool(e.ok())),
            ("diagnostics", Json::Arr(diags)),
            ("entities", Json::Arr(ents)),
            ("constraints", Json::Arr(cons)),
        ])
    })
}
