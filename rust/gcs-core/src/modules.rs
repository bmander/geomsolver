//! Modules: `use engine.crank` (Solvent §14.4).
//!
//! A module is a Solvent document read for its **components**.  `use NAME` at the top of a
//! document asks for one; what the name resolves to is the *host's* question — the core takes text
//! and has no filesystem — so `link` takes a resolver, `NAME -> Found` (the text, and where the
//! host found it), and the CLI answers it from the files beside the document while the browser
//! answers it from what the page handed over and the library compiled into the core
//! (`library.rs`).  A module's own `use`s are followed the same way, once each, so a diamond
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
//! **A given module** (#993 stage 2): a text that is an ISO 10303-21 exchange file (`is_step`), or
//! one the host found by an exchange file's name (`Found::exchange`), is never parsed as Solvent.
//! It contributes one thing, the body its file gives, named by the module's whole path
//! (`flatten::given`).  Its `Module::text` is empty — one byte of the virtual text, so no later
//! module's spans are pushed past a file of megabytes — and what the kernel read of its file and
//! the text's digest stand beside it (`Module::given`, `Given`), shared: `relink` hands the same
//! reading on, so an edit never reads the file again, and a fresh link of a text read before on
//! this thread finds it in a small memo by its digest.  The file's text is the host's: the core
//! keeps none of it once read.

use crate::drawing::relative_path;
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

/// **What a host found for a `use`**: the module's text, whether it was found as an exchange file
/// (`NAME.step`, `NAME.stp`: `Found::at`), and the text's `digest` where the host took it already
/// (the ABI's, as a text is handed over), so a text read before is known without being hashed
/// again.  A resolver answering with a bare `String` answers a text found by no name, told by its
/// content alone (`From<String>`).  The text is shared, so a host keeping it hands it on uncopied.
#[derive(Clone)]
pub struct Found {
    pub text: Arc<str>,
    pub exchange: bool,
    pub digest: Option<u64>,
}

impl Found {
    /// A text the host found at `path`: an exchange file where the path names one.
    pub fn at(path: &str, text: String) -> Found {
        Found { text: text.into(), exchange: exchange(path), digest: None }
    }

    /// Whether `link` reads it as a given module: found as an exchange file, or one by its text.
    fn given(&self) -> bool {
        self.exchange || is_step(&self.text)
    }
}

impl From<String> for Found {
    fn from(text: String) -> Found {
        Found { text: text.into(), exchange: false, digest: None }
    }
}

/// Whether a path names an exchange file (`NAME.step`, `NAME.stp`, `MODULE_EXTENSIONS` past the
/// first): **a file found by an exchange file's name is one whatever its text**, so one that is
/// no exchange file — empty, or not starting `ISO-10303-21;` — is given to the kernel's reader,
/// which refuses it in its own words at the `use` (E072), and it is never parsed as Solvent.
pub fn exchange(path: &str) -> bool {
    path.rsplit_once('.').is_some_and(|(_, ext)| MODULE_EXTENSIONS[1..].contains(&ext))
}

/// The text of a module a host read as bytes (a file, a buffer handed over the ABI) where it found
/// them, `path` (as `search_paths` names it, or a path in the page's project; empty where the host
/// has none).  Told as `link` tells it: UTF-8 as it stands; else, an exchange file — one found by
/// an exchange file's name (`exchange`), or one by its text (`is_step`) — read lossily: real
/// Part 21 files carry Latin-1 bytes in their strings (a product's name, the header's author), and
/// the geometry they give is all ASCII, so nothing a body is read from is touched; else refused,
/// for a Solvent document that is not UTF-8 is not one to guess at.  Every host reads through
/// this, so a module file that exists and does not read is said, never taken for no module.
pub fn decode(path: &str, bytes: Vec<u8>) -> Result<String, std::string::FromUtf8Error> {
    String::from_utf8(bytes).or_else(|e| {
        let head = &e.as_bytes()[..e.as_bytes().len().min(1024)];
        match exchange(path) || is_step(&String::from_utf8_lossy(head)) {
            true => Ok(String::from_utf8_lossy(e.as_bytes()).into_owned()),
            false => Err(e),
        }
    })
}

/// Candidate paths relative to a root model, nearest directory first, and in each directory
/// `NAME.sv`, then `NAME.step`, then `NAME.stp`: a nearer file of any kind wins over a farther
/// one, and in one directory the Solvent document wins.  Looking in its ancestors lets a component
/// opened directly keep the same project imports as its assembly.  The host reads the first that
/// exists and hands its text over; the library is asked only when none does.
///
/// **A document is never its own module**: the candidate that is `source` itself (`foo.sv`'s
/// `use foo`, `engine/parts.sv`'s `use engine.parts` as `../engine/parts.sv`) is not named, so
/// `foo.sv` reads the `foo.step` beside it, and a `use` nothing else answers is a module there is
/// none of (E070).  Told by the paths as written, `.` and `..` read lexically
/// (`drawing::relative_path`).
pub fn search_paths(name: &str, source: &str) -> Vec<String> {
    let rel = name.replace('.', "/");
    let source = source.replace('\\', "/");
    let depth = source.split('/').count();
    let own = relative_path(&source, "");
    (0..depth).flat_map(|n| {
        let up = "../".repeat(n);
        let rel = &rel;
        MODULE_EXTENSIONS.iter().map(move |ext| format!("{up}{rel}.{ext}"))
    }).filter(|c| relative_path(c, &source) != own).collect()
}

/// Whether a module's text is an ISO 10303-21 exchange file rather than Solvent: its first token,
/// after an optional UTF-8 byte-order mark and ASCII whitespace, is `ISO-10303-21;`.  Told by the
/// content and not by a file's name, so every host — and a text a host was handed with no name —
/// answers alike; a host asks it before putting a text through the Solvent parser.  A text found
/// by an exchange file's name is one whatever it holds, which the host says (`Found::exchange`).
pub fn is_step(text: &str) -> bool {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    text.trim_start_matches(|c: char| c.is_ascii_whitespace()).starts_with("ISO-10303-21;")
}

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

/// Resolve every `use` in `prog` through `resolve`, and every `use` inside what it brings in.
///
/// The modules' components join `prog.components` before the root.  Diagnostics come back as
/// the elaborator's: a module nothing resolves (E070, at the `use`), a component defined twice
/// (E071, at the later definition), and a module's own parse errors (E100, at their place in the
/// module).  Nothing is said twice: a module asked for by two `use`s is linked once.
///
/// `resolve` answers what the host found (`Found`), or a bare text a host found by no name.
pub fn link<F: Into<Found>>(prog: &mut Program, resolve: &mut dyn FnMut(&str) -> Option<F>)
    -> Vec<Diag> {
    link_from(prog, &mut |name| resolve(name).map(Into::into), None)
}

/// `link`, handing on what `like` read of each given module rather than asking for its file.
fn link_from(
    prog: &mut Program,
    resolve: &mut dyn FnMut(&str) -> Option<Found>,
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
        let Some(found) = resolve(&name) else {
            diags.push(Diag::new(
                Code::E070,
                at,
                None,
                format!("no module `{name}`"),
            ));
            continue;
        };
        if found.given() {
            push_given(prog, name, via, given(&found));
            continue;
        }
        let text = found.text.to_string();
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
    let mut resolve = |name: &str| {
        like.module_text(name).or_else(|| crate::library::resolve(name)).map(Found::from)
    };
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

/// **What a given module's file gives** (#993 stage 2): the text's digest and length, and the one
/// solid the kernel read from it — or why there is none the language reads, in the kernel's words
/// (`Refusal`'s, its entity `#n` in front) or saying how many solids the file gives.  Shared by
/// every program linked from it.  The text itself is the host's, never kept here: a file may be
/// megabytes.
#[derive(Debug)]
pub struct Given {
    pub digest: u64,
    pub len: usize,
    pub body: Result<Arc<Body>, String>,
}

/// What a given module's file gives the model: its B-rep and the text's digest.
pub type Body = crate::model::Given;

/// How many files a thread keeps what it read of: a host elaborates a document often, and every
/// fresh link of it hands the same texts again.
const READ_KEPT: usize = 4;

thread_local! {
    /// The files read last on this thread, the latest at the end.
    static READ: RefCell<Vec<Arc<Given>>> = const { RefCell::new(Vec::new()) };
}

/// **A STEP text read**, once per thread for as long as it is among the last few read: reading is
/// unbudgeted, and a file of megabytes takes seconds.  Found by its digest — the host's where it
/// took one — and its length, never by comparing the text; a text found by an exchange file's name
/// is read as the file's own.
fn given(found: &Found) -> Arc<Given> {
    let digest = found.digest.unwrap_or_else(|| digest(&found.text));
    let len = found.text.len();
    let kept = READ.with(|r| {
        let mut r = r.borrow_mut();
        let k = r.iter().position(|g| g.digest == digest && g.len == len)?;
        let g = r.remove(k);
        r.push(g.clone());
        Some(g)
    });
    if let Some(g) = kept {
        return g;
    }
    let g = Arc::new(Given { digest, len, body: read(&found.text, digest) });
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
