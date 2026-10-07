//! Source text, byte spans, statement identities, and program traversal.

use super::{Chained, Component, InBlock, Stmt, StmtKind, WordDef};
use crate::constraints::Fixity;

/// How long a program may be.  A document is untrusted input and `wasm32-unknown-unknown` aborts
/// rather than unwinding, so the size is checked here rather than left to an allocator.
pub const MAX_TEXT: usize = 1 << 20;

/// How many statements one may hold.
pub const MAX_STMTS: usize = 100_000;

/// A half-open byte range into the program text.
///
/// Bytes, not characters: the front end slices the same `&str` the core parsed, and a UTF-8
/// boundary is the only thing the two ever have to agree about.  Line and column are *not* stored
/// — `line_col` computes them on demand, so there is nothing to keep in step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Span {
    pub lo: u32,
    pub hi: u32,
}

impl Span {
    pub fn new(lo: usize, hi: usize) -> Span {
        Span { lo: lo as u32, hi: hi as u32 }
    }

    pub fn slice(self, text: &str) -> &str {
        text.get(self.lo as usize..self.hi as usize).unwrap_or("")
    }

    pub fn contains(self, off: u32) -> bool {
        self.lo <= off && off < self.hi
    }

    pub fn len(self) -> u32 {
        self.hi.saturating_sub(self.lo)
    }

    pub fn is_empty(self) -> bool {
        self.len() == 0
    }
}

/// 1-based line and column at a byte offset.  Columns count characters, not bytes, because that
/// is what an editor's caret does.
pub fn line_col(text: &str, off: u32) -> (u32, u32) {
    let off = (off as usize).min(text.len());
    let head = &text[..off];
    let line = 1 + head.bytes().filter(|&b| b == b'\n').count() as u32;
    let bol = head.rfind('\n').map(|i| i + 1).unwrap_or(0);
    (line, 1 + text[bol..off].chars().count() as u32)
}

/// A name as it was written, and where.
#[derive(Clone, Debug, PartialEq)]
pub struct Name {
    pub text: String,
    pub span: Span,
}

impl Name {
    pub fn new(text: impl Into<String>) -> Name {
        Name { text: text.into(), span: Span::default() }
    }
}

/// A statement's identity, minted by whoever built it and **preserved by every edit**.  Not a
/// position: inserting a statement above must not renumber one a caller is holding on to, since a
/// selection outlives the elaboration that resolved it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct StmtId(pub u32);

/// A whole program.  It owns its text, and every `Span` in it indexes that text — which is why
/// source edits reparse their result. `render_flat` is a canonical export of the flat subset.
#[derive(Clone, Debug, Default)]
pub struct Program {
    pub(super) text: String,
    /// One, anonymous, in the flat subset; `component` is a parser addition rather than a change
    /// of shape.  A module's components stand here too once `modules::link` has run, before the
    /// root, each saying which module it came from.
    pub components: Vec<Component>,
    /// The `in PLANE { … }` blocks' own text (§6.7).  Their statements are hoisted into the
    /// body at parse, so nothing but the header and the brace is the block's, and this is
    /// where `edit::remove` finds them when the plane goes.
    pub in_blocks: Vec<InBlock>,
    /// Optional file-level `preview { … }`. Its statements join the root solve, but are
    /// omitted when the file is imported. The span includes the keyword and braces.
    pub preview: Option<Span>,
    /// `use engine.crank` — the modules the document asks for (§14.4), in written order.  What a
    /// name resolves to is the host's business (`modules::link`): the core takes text and has
    /// no filesystem.
    pub uses: Vec<Use>,
    /// The relation words defined at the top of a file (`a horizontal b := a level(up) b`, §9.9):
    /// the document's own, then each module's once `modules::link` has run, each saying which
    /// file it was written in.
    pub words: Vec<WordDef>,
    /// The modules linked in, in the order they were resolved.  Each one's text is kept, so a
    /// re-parse of the document (`retext`) can link again without asking the host.
    pub modules: Vec<Module>,
    pub(super) next_stmt: u32,
}

/// `use engine.crank` — a module the document reads its components from — and, in brackets,
/// the names it reads bare: `use std (horizontal, vertical)` (§14.4 [0.48]).
#[derive(Clone, Debug)]
pub struct Use {
    /// The dotted name as written, `engine.crank`.
    pub name: String,
    pub span: Span,
    /// The names imported bare, as written; empty for a `use` that imports none.
    pub names: Vec<Name>,
}

/// A linked module. Spans use virtual offsets: document text, then modules separated
/// by one-byte gaps. `Program::source_at` maps an offset to its source.
#[derive(Clone, Debug)]
pub struct Module {
    pub name: String,
    pub text: String,
    /// The offset this module's text starts at in the virtual text.
    pub base: usize,
    /// The `use` in the **document** that brought it in — directly, or through another module —
    /// which is where a diagnostic inside it is shown to a reader of the document.
    pub via: Span,
    /// The module's own top-level body: its `param`s are what its components may read (§6.3).
    /// Nothing else in it is drawn — a module's drawing is its own.
    pub root: Component,
    /// The modules this one `use`s, whose params its file reads in turn, and the names each
    /// imports bare.
    pub uses: Vec<Use>,
}

impl Program {
    pub fn new() -> Program {
        Program {
            text: String::new(),
            components: vec![Component::default()],
            in_blocks: Vec::new(),
            preview: None,
            uses: Vec::new(),
            words: Vec::new(),
            modules: Vec::new(),
            next_stmt: 0,
        }
    }

    /// The id the next statement minted into this program takes — what a module's statements
    /// are numbered from, so no two statements in one program share an id.
    pub fn next_stmt(&self) -> u32 {
        self.next_stmt
    }

    pub(crate) fn set_next_stmt(&mut self, n: u32) {
        self.next_stmt = n;
    }

    /// Where the virtual text ends: the offset the next module linked in starts at.
    pub fn virtual_len(&self) -> usize {
        match self.modules.last() {
            Some(m) => m.base + m.text.len() + 1,
            None => self.text.len() + 1,
        }
    }

    /// Which text an offset is in — the document (`None`) or a module — and the offset in it.
    pub fn source_at(&self, off: usize) -> (Option<usize>, usize) {
        for (k, m) in self.modules.iter().enumerate() {
            if off >= m.base && off <= m.base + m.text.len() {
                return (Some(k), off - m.base);
            }
        }
        (None, off.min(self.text.len()))
    }

    /// The characters a span covers, in whichever text it is in: the document's or a module's.
    pub fn span_text(&self, span: Span) -> Option<&str> {
        let (lo, hi) = (span.lo as usize, span.hi as usize);
        match self.source_at(lo) {
            (Some(k), at) => self.modules[k].text.get(at..at + hi.checked_sub(lo)?),
            (None, _) => self.text.get(lo..hi),
        }
    }

    /// Whether a span is in the document's own text — what a splice may touch.
    pub fn owns(&self, span: Span) -> bool {
        span.hi as usize <= self.text.len()
    }

    /// 1-based line and column of an offset, in whichever text it is in.
    pub fn line_col(&self, off: usize) -> (u32, u32) {
        match self.source_at(off) {
            (Some(k), local) => line_col(&self.modules[k].text, local as u32),
            (None, local) => line_col(&self.text, local as u32),
        }
    }

    /// A module's text by name, for a re-parse that links again without asking the host.
    pub fn module_text(&self, name: &str) -> Option<String> {
        self.modules.iter().find(|m| m.name == name).map(|m| m.text.clone())
    }

    /// The text this program was last rendered or parsed from.  Every `Span` indexes it.
    pub fn text(&self) -> &str {
        &self.text
    }

    /// The root component — the one the drawing is.  The flat subset has exactly one.
    pub fn root(&self) -> &Component {
        self.components.last().expect("a program has a root")
    }

    pub fn root_mut(&mut self) -> &mut Component {
        self.components.last_mut().expect("a program has a root")
    }

    /// A fresh statement identity.  Monotonic, and never reused, so a caller holding one can only
    /// find the statement it named or nothing at all.
    pub fn mint(&mut self) -> StmtId {
        self.next_stmt += 1;
        StmtId(self.next_stmt)
    }

    pub fn push(&mut self, kind: StmtKind) -> StmtId {
        let id = self.mint();
        let st = Stmt { id, kind, span: Span::default(), chained: Chained::No };
        self.root_mut().body.push(st);
        id
    }

    /// The document's own component of that name — not one a module defines, which is reached
    /// by its qualified name (`resolve_component`).
    pub fn component(&self, name: &str) -> Option<&Component> {
        self.component_in(None, name)
    }

    /// The component `name` defined in `module` (`None`: the document's own).
    pub fn component_in(&self, module: Option<usize>, name: &str) -> Option<&Component> {
        self.components
            .iter()
            .find(|c| c.module == module && c.name.as_ref().is_some_and(|n| n.text == name))
    }

    /// The modules a body read `from` the document (`None`) or a module may name: exactly the
    /// ones its own file `use`s (§14.4) — a module brought in by another is that one's business.
    pub fn uses_of(&self, from: Option<usize>) -> Vec<&str> {
        self.use_stmts(from).iter().map(|u| u.name.as_str()).collect()
    }

    /// The `use` statements of the document (`None`) or a module, as written.
    pub fn use_stmts(&self, from: Option<usize>) -> &[Use] {
        match from {
            None => &self.uses,
            Some(k) => self.modules.get(k).map_or(&[], |m| &m.uses),
        }
    }

    /// **The module a bare name is imported from** in the file `from` (§14.4 [0.48]): the first
    /// `use` whose brackets name it, and only a module that was linked.  Two imports of one name
    /// are refused where the second is written (`words::check`), so the first is the answer.
    pub fn imported(&self, name: &str, from: Option<usize>) -> Option<usize> {
        let u = self.use_stmts(from).iter().find(|u| u.names.iter().any(|n| n.text == name))?;
        self.module_named(&u.name)
    }

    /// The linked module of that path (`engine.parts`), by index.
    pub fn module_named(&self, path: &str) -> Option<usize> {
        self.modules.iter().position(|m| m.name == path)
    }

    /// The relation word `word` of that fixity defined in `module` (`None`: the document's own),
    /// by index into `words`.
    pub fn word_in(&self, module: Option<usize>, word: &str, fixity: Fixity) -> Option<usize> {
        self.words
            .iter()
            .position(|d| d.module == module && d.fixity == fixity && d.word.text == word)
    }

    /// **The relation word a statement writes, from where it is written** (§9.9): one its own
    /// file defines, else one a `use` of the file imports by name — of the same fixity, since an
    /// infix word and a prefix one of the same spelling are two words.  The index into `words`.
    pub fn resolve_word(&self, word: &str, fixity: Fixity, from: Option<usize>) -> Option<usize> {
        let find = |module| self.word_in(module, word, fixity);
        find(from).or_else(|| self.imported(word, from).and_then(|m| find(Some(m))))
    }

    /// **The component a call names, from where it is written** (§14.4): a bare name is one its
    /// own file defines, and a module's is reached only by its full path, `engine.parts.Crank`,
    /// through a `use` of that file.  The index into `components`, or why there is none.
    pub fn resolve_component(&self, name: &str, from: Option<usize>) -> Result<usize, String> {
        let find = |module: Option<usize>, base: &str| {
            self.components.iter().position(|c| {
                c.module == module && c.name.as_ref().is_some_and(|n| n.text == base)
            })
        };
        let uses = self.uses_of(from);
        if let Some((path, base)) = name.rsplit_once('.') {
            if !uses.contains(&path) {
                return Err(format!("no module `{path}` is used here: write `use {path}`"));
            }
            let m = self.module_named(path);
            return find(m, base).ok_or_else(|| format!("`{path}` defines no component `{base}`"));
        }
        if let Some(i) = find(from, name) {
            return Ok(i);
        }
        // a name the file imports bare (`use engine.parts (Crank)`)
        if let Some(i) = self.imported(name, from).and_then(|m| find(Some(m), name)) {
            return Ok(i);
        }
        // a module this file uses defines it: the one mistake worth naming the fix for
        for path in uses {
            let m = self.module_named(path);
            if m.is_some() && find(m, name).is_some() {
                return Err(format!("no component named `{name}` here: `{path}` defines one, \
                                    written `{path}.{name}`, or imported `use {path} ({name})`"));
            }
        }
        Err(format!("no component named `{name}`"))
    }

    /// A component's identity across the whole program: its module's path in front of its name
    /// (`engine.parts.Crank`), and its bare name where the document defines it.
    pub fn component_id(&self, i: usize) -> String {
        let c = &self.components[i];
        let name = c.name.as_ref().map_or("", |n| n.text.as_str());
        match c.module.and_then(|k| self.modules.get(k)) {
            Some(m) => format!("{}.{name}", m.name),
            None => name.to_string(),
        }
    }

    pub fn stmt(&self, id: StmtId) -> Option<&Stmt> {
        fn find(st: &Stmt, id: StmtId) -> Option<&Stmt> {
            if st.id == id {
                return Some(st);
            }
            match &st.kind {
                StmtKind::Block(b) => b.stmts().find_map(|inner| find(inner, id)),
                StmtKind::ClaimOver(c) => c.body.iter().find_map(|inner| find(inner, id)),
                _ => None,
            }
        }
        self.components.iter().find_map(|c| c.body.iter().find_map(|st| find(st, id)))
    }

    /// The innermost statement covering a byte offset — a caret, turned into what it is written
    /// on.  A linear scan: this runs on a click, never on a frame.
    pub fn at_offset(&self, off: u32) -> Option<&Stmt> {
        self.stmts().filter(|s| s.span.contains(off)).min_by_key(|s| s.span.len())
    }

    /// Visit all statements, including nested blocks, in source order.
    pub fn stmts(&self) -> impl Iterator<Item = &Stmt> {
        fn walk<'a>(st: &'a Stmt, out: &mut Vec<&'a Stmt>) {
            out.push(st);
            match &st.kind {
                StmtKind::Block(b) => {
                    for inner in b.stmts() {
                        walk(inner, out);
                    }
                }
                StmtKind::ClaimOver(c) => {
                    for inner in &c.body {
                        walk(inner, out);
                    }
                }
                _ => {}
            }
        }
        let mut out = Vec::new();
        for c in self.components.iter() {
            for st in c.body.iter() {
                walk(st, &mut out);
            }
        }
        out.into_iter()
    }
}

/// What the parser could not make of the text.  A code and a span, so it lands in the same
/// gutter as everything elaboration and the solver have to say — see `program::Diag`, which this
/// becomes.
#[derive(Clone, Debug)]
pub struct SynErr {
    pub span: Span,
    pub message: String,
}
