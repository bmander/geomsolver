//! Modules: `use engine.crank` (Solvent §14.4).
//!
//! A module is a Solvent document read for its **components**.  `use NAME` at the top of a
//! document asks for one; what the name resolves to is the *host's* question — the core takes text
//! and has no filesystem — so `link` takes a resolver, `NAME -> text`, and the CLI answers it from
//! the files beside the document while the browser answers it from the library compiled into the
//! core (`library.rs`).  A module's own `use`s are followed the same way, once each, so a diamond
//! is one copy and a cycle is not a hang.
//!
//! What a module contributes is exactly its component definitions and relation words, plus the
//! top-level parameters and groups available to the importing root (§6.3, §9.9) — each by its
//! full path, or bare where the importing `use` names it (`use std.place (horizontal)`).  Its own
//! drawing — whatever loose statements it holds, its `preview` and the parameters that run over
//! an interval for it (#593) — is its own and is not drawn here, so `gear.sv` is a module as it
//! stands.  Two definitions of one component name, wherever they come from, are refused: there is
//! no shadowing (§5).
//!
//! **Every span stays one integer.**  A module is parsed at a `base` past the end of everything
//! linked before it (`syntax::parse_from`), so its statements' spans and ids join the document's
//! without a second coordinate anywhere; `Program::source_at` says which text an offset is in, and
//! `program::elaborate` shows a module's diagnostic at the `use` that brought the module in, its
//! own place named in the message.
//!
//! **A given module** (#993 stage 2): a text that is an ISO 10303-21 exchange file (`is_step`) is
//! never parsed as Solvent.  It contributes one thing, the body its file gives, named by the
//! module's whole path (`flatten::given`).  Its `Module::text` is empty — one byte of the virtual
//! text, so no later module's spans are pushed past a file of megabytes — and its file, what the
//! kernel read of it and the text's digest stand beside it (`Module::given`, `Given`), shared:
//! `relink` hands the same reading on, so an edit never reads the file again, and a fresh link of
//! a text read before on this thread finds it in a small memo by its digest.

use crate::program::{Code, Diag};
use crate::syntax::{parse_from, Component, Module, Program, Span, StmtKind};
use std::cell::RefCell;
use std::collections::{BTreeSet, VecDeque};
use std::sync::Arc;

/// The most modules one document may link, transitively — the same kind of bound `MAX_STMTS` is.
pub const MAX_MODULES: usize = 64;

/// The extensions a module's file may carry, in the order a host tries them in one directory: a
/// Solvent document, then an ISO 10303-21 exchange file that gives a body (#993 stage 2).
pub const MODULE_EXTENSIONS: [&str; 3] = ["sv", "step", "stp"];

/// The text of a module a host read as bytes (a file, a buffer handed over the ABI) where it found
/// them, `path` (as `search_paths` names it, or a path in the page's project; empty where the host
/// has none, a worker handed the page's texts).  Told by its content as `is_step` tells it: UTF-8
/// as it stands; else, an exchange file read lossily — real Part 21 files carry Latin-1 bytes in
/// their strings (a product's name, the header's author), and the geometry they give is all ASCII,
/// so nothing a body is read from is touched; else refused, for a Solvent document that is not
/// UTF-8 is not one to guess at.  Every host reads through this, so a module file that exists and
/// does not read is said, never taken for no module.
///
/// **A file found by an exchange file's name** (`NAME.step`, `NAME.stp`) is one whatever its text:
/// one that is no exchange file — empty, or not starting `ISO-10303-21;` — is read lossily and
/// handed on marked so (`is_step` answers it), so `link` gives it to the kernel's reader, which
/// refuses it in its own words at the `use` (E072), and it is never parsed as Solvent.  The mark
/// is the host's word for where the text was found, carried in the text because a resolver
/// answers a name with a text; the reader is given the file's own text.
pub fn decode(path: &str, bytes: Vec<u8>) -> Result<String, std::string::FromUtf8Error> {
    let exchange = path.rsplit_once('.').is_some_and(|(_, ext)| MODULE_EXTENSIONS[1..].contains(&ext));
    if exchange {
        let text = String::from_utf8(bytes)
            .unwrap_or_else(|e| String::from_utf8_lossy(e.as_bytes()).into_owned());
        return Ok(match is_step(&text) {
            true => text,
            false => format!("{FOUND}{text}"),
        });
    }
    String::from_utf8(bytes).or_else(|e| {
        let head = &e.as_bytes()[..e.as_bytes().len().min(1024)];
        match is_step(&String::from_utf8_lossy(head)) {
            true => Ok(String::from_utf8_lossy(e.as_bytes()).into_owned()),
            false => Err(e),
        }
    })
}

/// What `decode` puts before the text of a file found by an exchange file's name that is no
/// exchange file: no Solvent document or Part 21 file starts with a NUL.
const FOUND: &str = "\u{0}found as an exchange file\u{0}";

/// Candidate paths relative to a root model, nearest directory first, and in each directory
/// `NAME.sv`, then `NAME.step`, then `NAME.stp`: a nearer file of any kind wins over a farther
/// one, and in one directory the Solvent document wins.  Looking in its ancestors lets a component
/// opened directly keep the same project imports as its assembly.  The host reads the first that
/// exists and hands its text over; the library is asked only when none does.
///
/// **A document is never its own module**: the candidate that is `source` itself (`foo.sv`'s
/// `use foo`, `engine/parts.sv`'s `use engine.parts` as `../engine/parts.sv`) is not named, so
/// `foo.sv` reads the `foo.step` beside it, and a `use` nothing else answers is a module there is
/// none of (E070).  Told by the paths as written, `.` and `..` read lexically.
pub fn search_paths(name: &str, source: &str) -> Vec<String> {
    let rel = name.replace('.', "/");
    let source = source.replace('\\', "/");
    let depth = source.split('/').count();
    let own = lexical(&source);
    let dir = source.rsplit_once('/').map_or("", |(dir, _)| dir).to_string();
    (0..depth).flat_map(|n| {
        let up = "../".repeat(n);
        let rel = &rel;
        MODULE_EXTENSIONS.iter().map(move |ext| format!("{up}{rel}.{ext}"))
    }).filter(|c| lexical(&format!("{dir}/{c}")) != own).collect()
}

/// A path's parts, `.` and empty parts dropped and `..` taken back where it can be.
fn lexical(path: &str) -> Vec<String> {
    let mut parts: Vec<String> = Vec::new();
    for p in path.split('/') {
        match p {
            "" | "." => {}
            ".." if parts.last().is_some_and(|l| *l != "..") => { parts.pop(); }
            p => parts.push(p.to_string()),
        }
    }
    parts
}

/// Whether a module's text is an ISO 10303-21 exchange file rather than Solvent: its first token,
/// after an optional UTF-8 byte-order mark and ASCII whitespace, is `ISO-10303-21;`.  Told by the
/// content and not by a file's name, so every host — and a text a host was handed with no name —
/// answers alike; a host asks it before putting a text through the Solvent parser.  A text
/// `decode` marked as found by an exchange file's name is one too, whatever follows the mark.
pub fn is_step(text: &str) -> bool {
    if text.starts_with(FOUND) {
        return true;
    }
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    text.trim_start_matches(|c: char| c.is_ascii_whitespace()).starts_with("ISO-10303-21;")
}

/// Resolve every `use` in `prog` through `resolve`, and every `use` inside what it brings in.
///
/// The modules' components join `prog.components` before the root.  Diagnostics come back as
/// the elaborator's: a module nothing resolves (E070, at the `use`), a component defined twice
/// (E071, at the later definition), and a module's own parse errors (E100, at their place in the
/// module).  Nothing is said twice: a module asked for by two `use`s is linked once.
/// Where module statements' ids start: past any a document is ever given (`parse` caps a
/// document far below it), so a module's ids depend only on the modules before it and its text.
pub const MODULE_STMTS: u32 = 1 << 30;

/// Where the ids of statements no text holds start, past every module's (#993's review, R10):
/// a given body's declaration (`flatten::given`) is `GIVEN_STMTS` plus its module's index, and a
/// distance's foot's statements (`flatten::sets`, `FOOT_STMTS`) count down from the top — so
/// neither is ever a statement of the document, a module, or each other.
pub const GIVEN_STMTS: u32 = 3 << 30;

/// The highest statement id, the first of the three a distance's foot adds to a set's body
/// (`flatten::sets`), each applied under a prefix of its own; below it, down to `GIVEN_STMTS`'s
/// range, nothing is minted.
pub const FOOT_STMTS: u32 = u32::MAX;

pub fn link(prog: &mut Program, resolve: &mut dyn FnMut(&str) -> Option<String>) -> Vec<Diag> {
    link_from(prog, resolve, None)
}

/// `link`, handing on what `like` read of each given module rather than asking for its file.
fn link_from(
    prog: &mut Program,
    resolve: &mut dyn FnMut(&str) -> Option<String>,
    like: Option<&Program>,
) -> Vec<Diag> {
    let mut diags = Vec::new();
    // (name, the `use` that asked, the document's `use` this descends from)
    let mut queue: VecDeque<(String, Span, Span)> =
        prog.uses.iter().map(|u| (u.name.clone(), u.span, u.span)).collect();
    let mut seen: BTreeSet<String> = BTreeSet::new();
    while let Some((name, at, via)) = queue.pop_front() {
        if !seen.insert(name.clone()) {
            continue;
        }
        if prog.modules.len() >= MAX_MODULES {
            diags.push(Diag::new(
                Code::E070,
                at,
                None,
                format!("a document may not link more than {MAX_MODULES} modules"),
            ));
            break;
        }
        // a given module linked before is handed on whole: its file is not asked for again
        let before = like.and_then(|l| l.modules.iter().find(|m| m.name == name))
            .and_then(|m| m.given.clone());
        if let Some(given) = before {
            push_given(prog, name, via, given);
            continue;
        }
        let Some(text) = resolve(&name) else {
            diags.push(Diag::new(
                Code::E070,
                at,
                None,
                format!("no module `{name}`"),
            ));
            continue;
        };
        if is_step(&text) {
            push_given(prog, name, via, given(text));
            continue;
        }
        let base = prog.virtual_len();
        // a module's statements are numbered in a range of their own, so they keep their ids
        // when the document grows: a statement appended to the document, then relinked, must not
        // renumber the library's, or every map entry naming one points at the wrong statement
        let (mp, errs) = parse_from(&text, base, prog.next_stmt().max(MODULE_STMTS));
        prog.set_next_stmt(mp.next_stmt());
        for e in errs {
            diags.push(Diag::new(Code::E100, e.span, None, e.message));
        }
        let k = prog.modules.len();
        let mut comps = mp.components;
        // The anonymous file root is last. Its params are kept; its geometry is not imported.
        let mut root = match comps.last() {
            Some(c) if c.name.is_none() => comps.pop().unwrap_or_default(),
            _ => Component::default(),
        };
        // A preview is only the file's own setup, including its units, params and dimensions.
        // Keep ordinary module params, but never export numbers from the preview into callers.
        if let Some(preview) = mp.preview {
            root.body.retain(|st| !preview.contains(st.span.lo));
        }
        // and a parameter that runs (`param spin: Angle over (…)`) belongs to the drawing that
        // reads it, which is not drawn here (#593): a component never sees one, so a file opened
        // to scrub its joint is used as it stands
        root.body.retain(|st| !matches!(&st.kind, StmtKind::Param(pd) if pd.over().is_some()));
        let mut comps_seen: BTreeSet<String> = BTreeSet::new();
        for mut c in comps {
            c.module = Some(k);
            let Some(cname) = c.name.as_ref() else { continue };
            // a module's names are its own (§14.4): only two definitions in one file clash
            if comps_seen.contains(&cname.text) {
                diags.push(Diag::new(
                    Code::E071,
                    c.span,
                    None,
                    format!("`{}` is defined twice in `{name}`", cname.text),
                ));
                continue;
            }
            comps_seen.insert(cname.text.clone());
            let root_at = prog.components.len() - 1;
            prog.components.insert(root_at, c);
        }
        for u in &mp.uses {
            queue.push_back((u.name.clone(), u.span, via));
        }
        // its relation words, said to be its own (§9.9); one defined twice is `words::check`'s
        prog.words.extend(mp.words.into_iter().map(|mut d| {
            d.module = Some(k);
            d
        }));
        // and its interfaces (§8.1 [0.84]); one defined twice is `interfaces::check`'s
        prog.interfaces.extend(mp.interfaces.into_iter().map(|mut f| {
            f.module = Some(k);
            f
        }));
        let previewed = mp.preview.is_some();
        prog.modules.push(Module {
            name, text, base, via, root, uses: mp.uses, previewed, given: None,
        });
    }
    localize(prog, &mut diags);
    diags
}

/// A diagnostic inside a module, shown to a reader of the document: at the `use` that brought
/// the module in, with the module's own name, line and column in front of the message.  One
/// wording, so the CLI and the panel cannot describe one fault two ways; and one place, so no
/// consumer of a span ever meets an offset the document does not have.
pub fn localize(p: &Program, diags: &mut [Diag]) {
    for d in diags.iter_mut() {
        if let (Some(k), local) = p.source_at(d.span.lo as usize) {
            let m = &p.modules[k];
            let (line, col) = crate::syntax::line_col(&m.text, local as u32);
            d.message = format!("{}:{line}:{col}: {}", m.name, d.message);
            d.span = m.via;
            d.stmt = None;
        }
    }
}

/// Link `prog` exactly as `like` was linked — from the module texts `like` already holds — for
/// a re-parse of the same document that must not ask the host again.  A module `like` had not
/// linked, a `use` the edit itself wrote (`use std.rounds (round)`, the Fillet tool's), comes from
/// the library compiled into the core, which asks no host either.
///
/// A given module is handed on as `like` read it (`Module::given`): its file is neither asked
/// for nor read again.
pub fn relink(prog: &mut Program, like: &Program) -> Vec<Diag> {
    let mut resolve = |name: &str| like.module_text(name).or_else(|| crate::library::resolve(name));
    link_from(prog, &mut resolve, Some(like))
}

/// A given module, linked: an empty text of its own (one byte of the virtual text), no
/// definitions, no `use`s, and its file's reading beside it.
fn push_given(prog: &mut Program, name: String, via: Span, given: Arc<Given>) {
    let base = prog.virtual_len();
    prog.modules.push(Module {
        name,
        text: String::new(),
        base,
        via,
        root: Component::default(),
        uses: Vec::new(),
        previewed: false,
        given: Some(given),
    });
}

/// **A digest of a text**, the same on every target (a worker matches builds by it): a hash over
/// its bytes, little-endian eight at a time, under fixed keys — no seed of the process's, no
/// dependency.
pub fn digest(text: &str) -> u64 {
    const K: u64 = 0x9e37_79b9_7f4a_7c15;
    const M: u64 = 0xff51_afd7_ed55_8ccd;
    let b = text.as_bytes();
    let mut h: u64 = 0x243f_6a88_85a3_08d3 ^ (b.len() as u64).wrapping_mul(K);
    let mut words = b.chunks_exact(8);
    for w in &mut words {
        let mut x = [0u8; 8];
        x.copy_from_slice(w);
        h = (h ^ u64::from_le_bytes(x)).wrapping_mul(K);
        h ^= h >> 29;
    }
    let mut x = [0u8; 8];
    x[..words.remainder().len()].copy_from_slice(words.remainder());
    h = (h ^ u64::from_le_bytes(x)).wrapping_mul(K);
    h ^= h >> 32;
    h = h.wrapping_mul(M);
    h ^ (h >> 33)
}

/// **What a given module's file gives** (#993 stage 2): the text as the host gave it (what
/// `Program::module_text` answers), its digest, and the one solid the kernel read from it — or
/// why there is none the language reads, in the kernel's words (`Refusal`'s, its entity `#n` in
/// front) or saying how many solids the file gives.  Shared by every program linked from it.
pub struct Given {
    pub text: String,
    pub digest: u64,
    pub body: Result<Arc<Body>, String>,
}

/// What a given module's file gives the model: its B-rep and the text's digest.
pub type Body = crate::model::Given;

/// The digest and what was read, never the text: a file may be megabytes.
impl std::fmt::Debug for Given {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let read = match &self.body {
            Ok(_) => "a solid".to_string(),
            Err(why) => format!("refused: {why}"),
        };
        write!(f, "Given {{ digest: {:016x}, bytes: {}, {read} }}", self.digest, self.text.len())
    }
}

/// How many files a thread keeps what it read of: a host elaborates a document often, and every
/// fresh link of it hands the same texts again.
const READ_KEPT: usize = 4;

thread_local! {
    /// The files read last on this thread, the latest at the end.
    static READ: RefCell<Vec<Arc<Given>>> = const { RefCell::new(Vec::new()) };
}

/// **A STEP text read**, once per thread for as long as it is among the last few read: reading is
/// unbudgeted, and a file of megabytes takes seconds.  Found by its digest, and the text held to
/// be the same, so no two files share a reading.
fn given(text: String) -> Arc<Given> {
    // a text marked as found by an exchange file's name is read as the file's own
    let text = match text.starts_with(FOUND) { true => text[FOUND.len()..].to_string(), false => text };
    let digest = digest(&text);
    let kept = READ.with(|r| {
        let mut r = r.borrow_mut();
        let k = r.iter().position(|g| g.digest == digest && g.text == text)?;
        let g = r.remove(k);
        r.push(g.clone());
        Some(g)
    });
    if let Some(g) = kept {
        return g;
    }
    let body = read(&text, digest);
    let g = Arc::new(Given { text, digest, body });
    READ.with(|r| {
        let mut r = r.borrow_mut();
        if r.len() >= READ_KEPT {
            r.remove(0);
        }
        r.push(g.clone());
    });
    g
}

/// The one solid a STEP text gives, or why none: the kernel's refusal as it says it, or how many
/// solids it gives where it gives several — one given body is one solid.
fn read(text: &str, digest: u64) -> Result<Arc<Body>, String> {
    let mut reading = crate::brep::step_read::reading(text).map_err(|r| r.to_string())?;
    match reading.solids.len() {
        1 => Ok(Arc::new(Body::new(reading.solids.pop().expect("one solid").brep, digest))),
        n => Err(format!("the file gives {n} solids, and a given body is one solid")),
    }
}
