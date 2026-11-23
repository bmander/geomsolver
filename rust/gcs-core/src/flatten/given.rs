//! **A given body read by its name** (#993 stage 2): `use bracket`, `bracket` a STEP file
//! (`modules::is_step`), contributes one thing, the body the file gives, named by the module's
//! whole path — `bracket`, `vendor.bracket` — and read by a file that `use`s it wherever a solid's
//! name is read.  It is a name of the document's root like a declaration's (`Walk::new` puts it
//! among `names`), read through the one lookup (`Lookup::raw`, which notes each read here), and
//! **what is not read is not there**: the solid is declared (`Walk::given_bodies`, one per module
//! however many files use it) only once some statement has read it, so `use bracket` alone makes
//! nothing, and `part := solid(bracket)` one object, `part`, its stock the given body.
//!
//! **Where it stands** is the document's own space: the file's origin and axes are the world's.
//! A read where the body would have to be placed — in a definition's body, which its instance's
//! frame places; in a `ring`, which turns each copy; or under an `in` of a frame that is not the
//! world's — is refused at the read (E106: placing a given body is later work, #993 stage 5),
//! never left at the world.  A plane of the world (`in std.top { … }`) moves nothing.
//!
//! A file the kernel refuses is said once, at the `use` (`program::words::given`, E072); its name
//! is still known here, so a read of it says nothing more, and nothing is declared for it.

use super::{Scope, Walk};
use crate::program::Code;
use crate::syntax::{Decl, DeclName, Name, Program, Ref, Span, Stmt, StmtId, StmtKind, Sweep};
use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

/// The given modules a walk reads, and what it has read of them.  Shared by every clone of the
/// lookup's context (`Datums`), so no read is lost.
#[derive(Clone, Default)]
pub(super) struct Given {
    /// Each given module, by path: its index, the files that `use` it, and what its file gave.
    modules: BTreeMap<String, Module>,
    /// The document's `in` blocks of a frame that is not the world's: what each covers of the
    /// text, and the frame as written.
    away: Vec<(u32, u32, String)>,
    /// The given bodies read where they stand, by path.
    read: Rc<RefCell<BTreeSet<String>>>,
    /// Each read refused (where it is written): the path, and where it would have to be placed.
    misplaced: Rc<RefCell<BTreeMap<Span, (String, Away)>>>,
}

#[derive(Clone)]
struct Module {
    k: usize,
    files: BTreeSet<Option<usize>>,
    file: std::sync::Arc<crate::modules::Given>,
}

impl Module {
    /// Whether the kernel refused its file.
    fn refused(&self) -> bool {
        self.file.body.is_err()
    }
}

/// Where a read would have to place the body.
#[derive(Clone, Debug)]
enum Away {
    /// in a definition's body, placed by its instance's frame
    Definition,
    /// in a `ring`, each copy turned
    Ring,
    /// under `in` a frame that is not the world's
    Frame(String),
}

impl Given {
    /// The given modules of `prog`, and the files that use each: what the walk reads.
    pub(super) fn new(prog: &Program) -> Given {
        let mut modules = BTreeMap::new();
        for (k, m) in prog.modules.iter().enumerate() {
            let Some(g) = &m.given else { continue };
            let files = std::iter::once(None).chain((0..prog.modules.len()).map(Some))
                .filter(|&f| prog.uses_of(f).contains(&m.name.as_str()))
                .collect();
            modules.insert(m.name.clone(), Module { k, files, file: g.clone() });
        }
        if modules.is_empty() {
            return Given::default();
        }
        // a plane of the world is `std`'s own datum, read at the document's root
        let world = |r: &Ref| r.root.text == "std" && r.path.len() == 1;
        let away = prog.in_blocks.iter().filter(|b| !world(&b.plane))
            .map(|b| (b.header.lo, b.close.hi, crate::syntax::ref_text(&b.plane)))
            .collect();
        Given { modules, away, ..Given::default() }
    }

    /// The paths the document's root reads by name: each given module the document `use`s.
    pub(super) fn document_names(&self) -> impl Iterator<Item = &String> {
        self.modules.iter().filter(|(_, m)| m.files.contains(&None)).map(|(p, _)| p)
    }

    /// `segs` read in `sc` where nothing by name answers it — a closed body's — as a given
    /// body its file `use`s: the longest path that is one, and the rest past it.  Noted as read
    /// there (`note`), which refuses it.
    pub(super) fn unplaced(&self, segs: &[String], sc: &Scope) -> Option<(String, Vec<String>)> {
        (1..=segs.len()).rev().find_map(|take| {
            let path = segs[..take].join(".");
            let m = self.modules.get(&path)?;
            m.files.contains(&sc.module).then(|| (path, segs[take..].to_vec()))
        })
    }

    /// A reference `r`, read in `sc`, found to name `abs`: where that is a given body, the read
    /// noted — where it stands, or refused where it would have to be placed.
    pub(super) fn note(&self, abs: &str, r: &Ref, sc: &Scope) {
        if !self.modules.contains_key(abs) {
            return;
        }
        let away = if sc.stands.is_some() {
            Some(Away::Definition)
        } else if sc.block.ring.is_some() {
            Some(Away::Ring)
        } else {
            let at = r.span.lo;
            self.away.iter().rev().find(|(lo, hi, _)| *lo <= at && at < *hi)
                .map(|(_, _, f)| Away::Frame(f.clone()))
        };
        match away {
            None => {
                self.read.borrow_mut().insert(abs.to_string());
            }
            Some(why) => {
                self.misplaced.borrow_mut().entry(r.span).or_insert((abs.to_string(), why));
            }
        }
    }

    /// Whether `path` is a given module's, its body a solid however its file was read: what a
    /// reading at an instant of it places (`Walk::settle_readings`).
    pub(super) fn gives(&self, path: &str) -> bool {
        self.modules.contains_key(path)
    }

    /// Every given module's path, its body a solid's name however its file was read.
    pub(super) fn paths(&self) -> impl Iterator<Item = String> + '_ {
        self.modules.keys().cloned()
    }

    /// The paths of the given modules the walk reads whose file the kernel refused: names a
    /// read of finds, and nothing is declared for (`Walk::unmade_by_refusal`).
    pub(super) fn refused_paths(&self) -> BTreeSet<String> {
        self.modules.iter().filter(|(_, m)| m.refused()).map(|(p, _)| p.clone()).collect()
    }

    /// Whether `path` is a given module whose file the kernel refused: a name known, read
    /// silently, since the refusal is said at the `use`.
    pub(super) fn refused(&self, path: &str) -> bool {
        self.modules.get(path).is_some_and(Module::refused)
    }
}

impl Walk<'_> {
    /// **The given bodies read**, declared: one solid each, named by its module's path, its
    /// definition the file's reading (`Sweep::Given`), its statement the document's `use` — and
    /// each refused read said where it is written.  Read once the walk has resolved every
    /// reference, so every read is known; each declaration reads nothing, so there is nothing of
    /// it to resolve, and it is lowered as it stands.  With them, each read refused, by where it
    /// is read: the body's path (`Expansion::misplaced`).
    pub(super) fn given_bodies(&mut self)
        -> (Vec<crate::ir::Statement>, BTreeMap<Span, String>) {
        let given = self.standard.given.clone();
        let misplaced = std::mem::take(&mut *given.misplaced.borrow_mut());
        let mut read = given.read.borrow().clone();
        let mut refused = BTreeMap::new();
        for (span, (path, why)) in misplaced {
            if given.refused(&path) {
                continue;
            }
            refused.insert(span, path.clone());
            // declared all the same, so nothing reading it says more than this
            read.insert(path.clone());
            let place = match why {
                Away::Definition => "a definition's body, which its instance's frame places".into(),
                Away::Ring => "a `ring`, which turns each copy".to_string(),
                Away::Frame(f) => format!("`in {f}`, a frame that is not the world's"),
            };
            self.once(Code::E106, span, format!(
                "`{path}` is the body a file gives, which stands in the document's own space, \
                 and this reads it in {place}: placing a given body is not built yet (#993 \
                 stage 5), so read it at the document's root"));
        }
        let mut made = Vec::new();
        for path in read {
            let Some(m) = given.modules.get(&path).filter(|m| !m.refused()) else { continue };
            let at = self.prog.modules[m.k].via;
            let name = Name { text: path.clone(), span: at };
            let d = Decl {
                kind: crate::model::EntKind::Solid,
                seed: Vec::new(),
                seed_text: Vec::new(),
                sweep: Some(Sweep::Given(m.file.clone())),
                ..Decl::point(DeclName::Written(name), [0.0; 2], None)
            };
            // an identity no statement of the document, a module or a set's foot is minted
            let id = StmtId(crate::modules::GIVEN_STMTS + m.k as u32);
            let st = Stmt { id, kind: StmtKind::Decl(d), span: at, chained: Default::default() };
            match crate::ir::Statement::lower(st, Vec::new()) {
                Ok(st) => made.push(st),
                Err(d) => self.diagnostic(d),
            }
        }
        (made, refused)
    }
}
