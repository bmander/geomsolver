//! Source text, byte spans, statement identities, and program traversal.

use super::{Chained, Component, InBlock, Stmt, StmtKind};

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
    /// `use engine.crank` — the modules the document asks for (§14.4), in written order.  What a
    /// name resolves to is the host's business (`modules::link`): the core takes text and has
    /// no filesystem.
    pub uses: Vec<Use>,
    /// The modules linked in, in the order they were resolved.  Each one's text is kept, so a
    /// re-parse of the document (`retext`) can link again without asking the host.
    pub modules: Vec<Module>,
    pub(super) next_stmt: u32,
}

/// `use engine.crank` — a module the document reads its components from.
#[derive(Clone, Debug)]
pub struct Use {
    /// The dotted name as written, `engine.crank`.
    pub name: String,
    pub span: Span,
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
    /// The modules this one `use`s, whose params its file reads in turn.
    pub uses: Vec<String>,
}

impl Program {
    pub fn new() -> Program {
        Program {
            text: String::new(),
            components: vec![Component::default()],
            in_blocks: Vec::new(),
            uses: Vec::new(),
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

    /// Find a named component, including components imported from linked modules.
    pub fn component(&self, name: &str) -> Option<&Component> {
        self.components.iter().find(|c| c.name.as_ref().map(|n| n.text.as_str()) == Some(name))
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
