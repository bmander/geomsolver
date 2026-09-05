//! Statement identities, sketch provenance, and span updates after edits.

use super::{Diag, Severity};
use crate::model::{EntRef, Sketch};
use crate::syntax::{Named, Program, Span, StmtId, StmtKind};
use std::collections::{BTreeMap, BTreeSet};

/// The instances and block copies enclosing a statement, outermost first.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct InstPath(pub Vec<crate::ir::PathStep>);

/// Where something in the sketch came from.
#[derive(Clone, Debug)]
pub struct Site {
    pub stmt: StmtId,
    pub span: Span,
    pub path: InstPath,
}

/// Provenance for one elaboration. Entity indices and constraint IDs can change
/// on rebuild; retain statement IDs to carry selections across elaborations.
#[derive(Clone, Debug, Default)]
pub struct SourceMap {
    pub of_entity: BTreeMap<EntRef, Site>,
    pub of_constraint: BTreeMap<u32, Site>,
    /// Display names, excluding anonymous resolution keys. `by_name` contains all keys;
    /// `writable` further excludes expanded block copies.
    pub names: BTreeMap<EntRef, Vec<String>>,
    /// Of those, the entities whose name a statement may also be **written** with.  Strictly
    /// narrower than `names`, and a block's copy is the whole of the difference: `#3.0.p` is
    /// what the source calls that copy and carries a `#` no tokenizer will give back.
    writable: BTreeSet<EntRef>,
    by_name: BTreeMap<String, EntRef>,
    made: BTreeMap<StmtId, Vec<Made>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Made {
    Ent(EntRef),
    Con(u32),
    /// A gauge or a flag: it names something already in the map rather than making anything.
    Gauge,
}

impl SourceMap {
    pub fn ent_named(&self, n: &str) -> Option<EntRef> {
        self.by_name.get(n).copied()
    }

    pub fn made_by(&self, s: StmtId) -> &[Made] {
        self.made.get(&s).map(|v| v.as_slice()).unwrap_or(&[])
    }

    pub fn site_of(&self, e: EntRef) -> Option<&Site> {
        self.of_entity.get(&e)
    }

    pub fn site_of_constraint(&self, id: u32) -> Option<&Site> {
        self.of_constraint.get(&id)
    }

    /// Bind a resolution key and record its display and writeback eligibility.
    pub(crate) fn bind(&mut self, name: &str, e: EntRef, named: Named) {
        self.by_name.insert(name.to_string(), e);
        if named.shown() {
            self.names.entry(e).or_default().push(name.to_string());
        }
        if named.writable() {
            self.writable.insert(e);
        }
    }

    /// What the source calls an entity, where it calls it anything.
    pub fn name_of(&self, e: EntRef) -> Option<&String> {
        self.names.get(&e)?.first()
    }

    /// The same, where a statement may also be **written** with it — so `None` for one copy of a
    /// block, which the source calls `#3.0.p` and cannot say.
    pub(crate) fn writable_name(&self, e: EntRef) -> Option<&String> {
        self.writable.contains(&e).then(|| self.name_of(e)).flatten()
    }

    /// Every entity a statement made, in the order `program::build` made them — the declaration's
    /// own entity first, then the children it minted.  Two callers walk it for that reason
    /// (`edit::commit_seeds`, `edit::reconcile`), so the order is stated once, here.
    pub(crate) fn ents_made_by(&self, s: StmtId) -> impl Iterator<Item = EntRef> + '_ {
        self.made_by(s).iter().filter_map(|m| match *m {
            Made::Ent(e) => Some(e),
            _ => None,
        })
    }

    pub(super) fn record(&mut self, st: &crate::ir::Statement, what: Made) {
        let site = Site { stmt: st.id, span: st.span, path: InstPath(st.path.clone()) };
        match what {
            Made::Ent(e) => {
                self.of_entity.insert(e, site);
            }
            Made::Con(id) => {
                self.of_constraint.insert(id, site);
            }
            Made::Gauge => {}
        }
        self.made.entry(st.id).or_default().push(what);
    }
}

/// A sketch, where each of its parts came from, and what was wrong with the program.
#[derive(Default)]
pub struct Elaborated {
    pub sketch: Sketch,
    pub map: SourceMap,
    pub diags: Vec<Diag>,
    /// The program every span in here indexes.  Carried rather than borrowed: a span without the
    /// text it cuts is a pair of numbers about nothing, an edit needs the statements as well as
    /// the characters, and the caller that most needs all of it is across an ABI where a borrow
    /// cannot follow.
    pub program: Program,
    /// Whether the sketch has been moved out.  There was only ever one, and a second taker would
    /// get an empty sketch that looked like a real one.
    pub taken: bool,
}

impl Elaborated {
    /// The source every span indexes.
    pub fn text(&self) -> &str {
        self.program.text()
    }

    /// Update source and spans after a numeric edit without rebuilding the sketch.
    /// The caller must preserve statement order and identity. Return false without
    /// changes if parsing fails or a mapped statement disappears.
    pub fn retext(&mut self, text: &str) -> bool {
        let Some(prog) = reparse(text, &self.program) else { return false };
        let at = spans(&prog);
        // "the same statements" is the caller's claim; if it is false, refuse rather than quietly
        // dropping what the map knows.  The caller then elaborates, which is always correct.
        if !self.sites().all(|s| at.contains_key(&s.stmt)) {
            return false;
        }
        self.restamp(&at, Keep::All);
        self.program = prog;
        true
    }

    /// Adopt appended statements for geometry already in the sketch. `made` matches
    /// the appended statements in order. Reparse and update spans without rebuilding;
    /// return false without changes if parsing fails or too few statements remain.
    pub fn adopt(&mut self, text: &str, made: &[Made]) -> bool {
        let Some(prog) = reparse(text, &self.program) else { return false };
        let body = &prog.root().body;
        if body.len() < made.len() {
            return false;
        }
        let tail = &body[body.len() - made.len()..];
        // a statement may have gone as well as arrived, and one that went takes its entries with it
        self.restamp(&spans(&prog), Keep::Live);
        for (st, m) in tail.iter().zip(made) {
            let site = Site { stmt: st.id, span: st.span, path: InstPath::default() };
            match *m {
                Made::Ent(r) => {
                    if let StmtKind::Decl(d) = &st.kind {
                        self.map.bind(&d.name.key().text, r, d.name.named());
                    }
                    self.map.of_entity.insert(r, site);
                }
                Made::Con(id) => {
                    self.map.of_constraint.insert(id, site);
                }
                Made::Gauge => {}
            }
            self.map.made.entry(st.id).or_default().push(*m);
        }
        self.program = prog;
        true
    }

    /// Every site in the map: what the drawing was made from, entities and constraints alike.
    fn sites(&self) -> impl Iterator<Item = &Site> {
        self.map.of_entity.values().chain(self.map.of_constraint.values())
    }

    /// Point every site at where its statement now is.  The one mechanism `retext` and `adopt`
    /// share: both take a source the core spliced and have to bring the map onto it.
    fn restamp(&mut self, at: &BTreeMap<StmtId, Span>, keep: Keep) {
        if keep == Keep::Live {
            self.map.of_entity.retain(|_, s| at.contains_key(&s.stmt));
            self.map.of_constraint.retain(|_, s| at.contains_key(&s.stmt));
        }
        for s in self.map.of_entity.values_mut().chain(self.map.of_constraint.values_mut()) {
            if let Some(&sp) = at.get(&s.stmt) {
                s.span = sp;
            }
        }
    }

    /// Whether the program said anything the elaborator could not honour.  A warning does not
    /// count: an expression that will not compute is a thing to report, not a thing to refuse.
    pub fn ok(&self) -> bool {
        !self.diags.iter().any(|d| d.severity() == Severity::Error)
    }

    pub fn errors(&self) -> impl Iterator<Item = &Diag> {
        self.diags.iter().filter(|d| d.severity() == Severity::Error)
    }
}
/// What to do with a site whose statement is no longer in the program.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Keep {
    /// Every site's statement is expected to still be there; the caller has already checked.
    All,
    /// A statement may have been deleted, and its sites go with it.
    Live,
}

/// Parse a source that is expected to be good.  `None` when it is not: both callers are taking a
/// text the core itself just spliced, so a parse error means the splice was wrong, and the answer
/// is to refuse and let the caller elaborate from scratch.
fn reparse(text: &str, like: &Program) -> Option<Program> {
    let (mut prog, errs) = crate::syntax::parse(text);
    // the same modules, from the texts already in hand: a re-parse asks the host nothing
    let linked = crate::modules::relink(&mut prog, like);
    (errs.is_empty() && linked.is_empty()).then_some(prog)
}

/// Where every statement in a program sits, blocks included — a statement inside one is still
/// reached by its id.
fn spans(prog: &Program) -> BTreeMap<StmtId, Span> {
    prog.stmts().map(|st| (st.id, st.span)).collect()
}
