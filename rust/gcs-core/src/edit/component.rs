//! **Make component**: what is selected, packaged as a `component` of the document and drawn
//! again as one instance of it.
//!
//! A splice like every edit here, and a larger one: the statements that make the selection move
//! — their text, comments and all — into a new definition, `component Part(…) { … }`, and one
//! statement, `part0 := Part(…)`, stands where the first of them stood.  Three things follow from
//! a component's scope being closed (Solvent §8):
//!
//! * **What the moved text reads from outside becomes a formal**, named as the document names it,
//!   so the text inside reads as it did: a point or a plane by position (`a: point`), an
//!   instance or a group as a `group`, a value by label (`w: Length`).  A module's datum has no
//!   bare name to keep, so `std.front` is handed in as `front` and the moved text says `front`.
//! * **What stays outside and named something that moved names it through the instance**: `ab
//!   parallel cd`, with `ab` moved, becomes `part0.ab parallel cd`.
//! * **A relation goes with what it relates**: one every name of which moved goes too, whether
//!   or not it was selected (`horizontal ab`, `fix((0, 0)) a`), which is `remove`'s rule — a
//!   constraint comes along exactly when all its entities did.
//!
//! What a component or a block made moves as the statement that made it — the instance, the
//! whole `repeat` — since there is no statement of its own to move; what a module made (`std`'s
//! datums, which a band over the origin catches) stays, counted in `Edit::kept`.  The result is
//! **held to the elaborator** before it is handed back: an error it did not have is the edit's,
//! and refuses it in the elaborator's words.

use std::collections::{BTreeMap, BTreeSet};

use super::remove::mentions;
use super::{in_root, splice, Edit, Kind, Splice};
use crate::ir::PathStep;
use crate::model::{EntRef, Sketch};
use crate::program::{Elaborated, Site};
use crate::syntax::{self, Program, Span, Stmt, StmtId, StmtKind};

/// Package the selected entities and constraints into a new component, `name` or a fresh one
/// (`Part`, `Part2`, …), drawn where they were by one instance of it.  `Edit::names` is the
/// component and then the instance; `kept`, what was selected and made by a module.
///
/// The sketch is passed for its units (a value handed in is typed by them) and the front end's
/// reason in `remove`: it holds the live one.
pub fn make_component(
    e: &Elaborated,
    prog: &Program,
    sk: &Sketch,
    ents: &[EntRef],
    cons: &[u32],
    name: Option<&str>,
) -> Edit {
    component_edit(e, prog, sk, ents, cons, name).unwrap_or_else(|why| Edit::none(prog, Some(why)))
}

/// One formal of the new component, and what the instance hands it.
struct Formal {
    name: String,
    /// The type as written: `point`, `group`, `Length`.
    ty: String,
    /// The argument, as the document spells it where the instance stands: `a`, `std.front`.
    actual: String,
    /// A number, given by label (§4.1); everything else is given by position.
    number: bool,
}

/// A run of root statements sharing lines — almost always one statement on its own line, but a
/// chain or a `;` puts several on one, and a line moves whole or not at all.
struct Unit<'a> {
    stmts: Vec<&'a Stmt>,
    /// The text that moves: the statements, and the indentation and trailing comment of their
    /// line where they are all it holds.
    lo: usize,
    hi: usize,
    /// Whether `lo..hi` is whole lines, so that taking it out takes the line break too.
    whole: bool,
    /// The `in PLANE { … }` block it is written in (`Program::in_blocks`).
    block: Option<usize>,
}

fn component_edit(
    e: &Elaborated,
    prog: &Program,
    sk: &Sketch,
    ents: &[EntRef],
    cons: &[u32],
    name: Option<&str>,
) -> Result<Edit, String> {
    let text = prog.text();
    let body = &prog.root().body;

    // -- what moves: the root statement each selected thing came from
    let from_root = |site: &Site| -> Option<StmtId> {
        let id = match site.path.0.first() {
            None => site.stmt,
            Some(PathStep::Instance(id)) => *id,
            Some(PathStep::Copy { block, .. }) => *block,
        };
        in_root(prog, id).then_some(id)
    };
    let mut moved: BTreeSet<StmtId> = BTreeSet::new();
    let mut kept = 0;
    for r in ents {
        match e.map.of_entity.get(r).and_then(from_root) {
            Some(id) => {
                moved.insert(id);
            }
            None => kept += 1,
        }
    }
    for c in cons {
        if let Some(id) = e.map.of_constraint.get(c).and_then(from_root) {
            moved.insert(id);
        }
    }
    if moved.is_empty() {
        return Err(match kept {
            0 => "select what the component is to be made of".to_string(),
            _ => "that comes from a module or a component, which has no statement here to \
                  package"
                .to_string(),
        });
    }

    // -- the names the root binds, each to its statement; a block's are the block's
    let mut binder: BTreeMap<String, &Stmt> = BTreeMap::new();
    fn bind<'a>(owner: &'a Stmt, st: &'a Stmt, out: &mut BTreeMap<String, &'a Stmt>) {
        if let Some(n) = st.kind.bound_name() {
            out.entry(n.text.clone()).or_insert(owner);
        }
        if let StmtKind::Block(b) = &st.kind {
            for inner in b.stmts() {
                bind(owner, inner, out);
            }
        }
    }
    for st in body {
        bind(st, st, &mut binder);
    }
    let named: BTreeSet<String> = binder.keys().cloned().collect();
    let inside = |moved: &BTreeSet<StmtId>| -> BTreeSet<String> {
        binder.iter().filter(|(_, st)| moved.contains(&st.id)).map(|(n, _)| n.clone()).collect()
    };
    // -- and every relation whose names all moved goes with them, and an anonymous declaration
    // so too — the `line(o, q)` a relation is written over — to a fixed point, since the
    // relation names the line by its key
    loop {
        let names_in = inside(&moved);
        let mut grew = false;
        for st in body {
            let follows = match &st.kind {
                StmtKind::Relation(_) | StmtKind::Measured(_) | StmtKind::Minimize(_)
                | StmtKind::SolidRel(_) => true,
                StmtKind::Decl(d) => d.name.written().is_none(),
                _ => false,
            };
            if !follows || moved.contains(&st.id) {
                continue;
            }
            let m = mentions(st, &named);
            if !m.is_empty() && m.iter().all(|n| names_in.contains(n)) {
                moved.insert(st.id);
                grew = true;
            }
        }
        if !grew {
            break;
        }
    }
    let names_in = inside(&moved);
    for st in body.iter().filter(|st| moved.contains(&st.id)) {
        if let Some(why) = unmovable(st) {
            return Err(why);
        }
    }

    // -- the lines, and which of them move
    let units = units_of(text, body, &prog.in_blocks);
    let mut moving: Vec<&Unit> = Vec::new();
    for u in &units {
        let n = u.stmts.iter().filter(|st| moved.contains(&st.id)).count();
        if n == 0 {
            continue;
        }
        if n < u.stmts.len() {
            return Err(format!(
                "`{}` is written on one line with what is not selected; select all of it, or \
                 put each statement on a line of its own",
                text[u.lo..u.hi].trim()
            ));
        }
        moving.push(u);
    }
    // a block holding nothing that stays goes with what it held
    let emptied: BTreeSet<usize> = (0..prog.in_blocks.len())
        .filter(|&b| {
            let holds: Vec<&Unit> = units.iter().filter(|u| u.block == Some(b)).collect();
            let all = |u: &&Unit| u.stmts.iter().all(|st| moved.contains(&st.id));
            !holds.is_empty() && holds.iter().all(all)
        })
        .collect();

    // -- the names: the component's and the instance's
    // the document's own: a module's names are reached through the module, and shadow nothing
    let mut taken: BTreeSet<String> = prog
        .stmts()
        .filter(|st| st.span.hi as usize <= text.len())
        .filter_map(|st| st.kind.bound_name().map(|n| n.text.clone()))
        .collect();
    let comp = match name {
        Some(n) if !syntax::is_name(n) || syntax::reserved_word(n) => {
            return Err(format!("`{n}` cannot name a component"));
        }
        Some(n) if taken.contains(n) || prog.component(n).is_some() => {
            return Err(format!("`{n}` is already a name in this document"));
        }
        Some(n) => n.to_string(),
        None => std::iter::once("Part".to_string())
            .chain((2..).map(|i| format!("Part{i}")))
            .find(|n| !taken.contains(n) && prog.component(n).is_none())
            .expect("an unbounded sequence always holds a fresh name"),
    };
    taken.insert(comp.clone());
    let stem = {
        let mut cs = comp.chars();
        let first = cs.next().map(|c| c.to_lowercase().collect::<String>()).unwrap_or_default();
        first + cs.as_str()
    };
    let inst = (0..)
        .map(|i| format!("{stem}{i}"))
        .find(|n| !taken.contains(n))
        .expect("an unbounded sequence always holds a fresh name");
    taken.insert(inst.clone());

    // -- what the moved text reads from outside: the formals, and where a name is rewritten
    let modules: BTreeSet<&str> =
        prog.uses.iter().filter_map(|u| u.name.split('.').next()).collect();
    let values = Values::new(prog, sk.units);
    let mut formals: Vec<Formal> = Vec::new();
    let mut inner: Vec<Splice> = Vec::new();
    let mut read = |lo: usize, hi: usize, formals: &mut Vec<Formal>| -> Result<(), String> {
        for u in syntax::name_uses(&text[lo..hi]) {
            let root = &u.parts[0].0;
            let at = |k: usize| {
                let s = u.parts[k].1;
                Span::new(lo + s.lo as usize, lo + s.hi as usize)
            };
            if names_in.contains(root) {
                continue;
            }
            if let Some(st) = binder.get(root) {
                if formals.iter().any(|f| &f.actual == root) {
                    continue;
                }
                let (ty, number) = formal_type(root, st, &values)?;
                formals.push(Formal { name: root.clone(), ty, actual: root.clone(), number });
                continue;
            }
            if u.called || !modules.contains(root.as_str()) {
                continue;
            }
            // a module's datum: the longest dotted head that names an entity
            let Some((k, r)) = (2..=u.parts.len()).rev().find_map(|k| {
                let dotted: Vec<&str> = u.parts[..k].iter().map(|p| p.0.as_str()).collect();
                e.map.ent_named(&dotted.join(".")).map(|r| (k, r))
            }) else {
                continue;
            };
            let actual: Vec<&str> = u.parts[..k].iter().map(|p| p.0.as_str()).collect();
            let actual = actual.join(".");
            let fname = match formals.iter().find(|f| f.actual == actual) {
                Some(f) => f.name.clone(),
                None => {
                    let last = &u.parts[k - 1].0;
                    let fname = std::iter::once(last.clone())
                        .chain((2..).map(|i| format!("{last}{i}")))
                        .find(|n| {
                            !names_in.contains(n) && !named.contains(n) && n != &comp
                                && n != &inst && crate::expr::builtin(n).is_none()
                                && !formals.iter().any(|f| &f.name == n)
                        })
                        .expect("an unbounded sequence always holds a fresh name");
                    let ty = r.kind.as_str().to_string();
                    formals.push(Formal { name: fname.clone(), ty, actual, number: false });
                    fname
                }
            };
            let span = Span::new(at(0).lo as usize, at(k - 1).hi as usize);
            inner.push(Splice { at: span, with: fname });
        }
        Ok(())
    };
    let mut headers: BTreeMap<usize, String> = BTreeMap::new();
    for u in &moving {
        read(u.lo, u.hi, &mut formals)?;
        if let Some(b) = u.block.filter(|b| !headers.contains_key(b)) {
            let h = prog.in_blocks[b].header;
            read(h.lo as usize, h.hi as usize, &mut formals)?;
            headers.insert(b, String::new());
        }
    }
    // the text each moved span comes to, its outside names rewritten
    let rewritten = |lo: usize, hi: usize| -> String {
        let mine: Vec<Splice> = inner
            .iter()
            .filter(|s| lo <= s.at.lo as usize && s.at.hi as usize <= hi)
            .map(|s| Splice {
                at: Span::new(s.at.lo as usize - lo, s.at.hi as usize - lo),
                with: s.with.clone(),
            })
            .collect();
        splice(&text[lo..hi], mine)
    };
    for (b, h) in headers.iter_mut() {
        let at = prog.in_blocks[*b].header;
        *h = rewritten(at.lo as usize, at.hi as usize).trim().to_string();
    }

    // -- the definition: the moved text, in order, a block's runs in the block again
    let mut def = format!("component {comp}(");
    let mut order: Vec<&Formal> = formals.iter().filter(|f| !f.number).collect();
    order.extend(formals.iter().filter(|f| f.number));
    let typed: Vec<String> = order.iter().map(|f| format!("{}: {}", f.name, f.ty)).collect();
    def.push_str(&typed.join(", "));
    def.push_str(") {\n");
    let mut open: Option<usize> = None;
    for u in &moving {
        if open != u.block {
            if open.is_some() {
                def.push_str("  }\n");
            }
            if let Some(b) = u.block {
                def.push_str(&format!("  {}\n", headers[&b]));
            }
            open = u.block;
        }
        let pad = if open.is_some() { "    " } else { "  " };
        def.push_str(&indented(&rewritten(u.lo, u.hi), pad));
    }
    if open.is_some() {
        def.push_str("  }\n");
    }
    def.push('}');

    // -- the instance, which hands each formal what the document calls it
    let args: Vec<String> = order
        .iter()
        .map(|f| if f.number { format!("{}: {}", f.name, f.actual) } else { f.actual.clone() })
        .collect();
    let call = format!("{inst} := {comp}({})", args.join(", "));

    // -- the splices: each moved line out, the instance where the first was, the definition
    // above the drawing, and every name left outside that named what moved, through the instance
    let mut edits: Vec<Splice> = Vec::new();
    let first = moving[0];
    let line_lo = |at: usize| text[..at].rfind('\n').map_or(0, |k| k + 1);
    for u in &moving {
        let anchor = std::ptr::eq(*u, first) && u.block.is_none();
        let mut at = Span::new(u.lo, u.hi);
        let mut with = String::new();
        if u.whole {
            at = Span::new(u.lo, (u.hi + 1).min(text.len()));
        }
        if anchor {
            with = match u.whole {
                true => format!("{}{call}\n", &text[u.lo..u.lo + indent_of(&text[u.lo..u.hi])]),
                false => call.clone(),
            };
        }
        edits.push(Splice { at, with });
    }
    match first.block {
        // the instance stands before the block the first moved line was in, which stays
        Some(b) if !emptied.contains(&b) => {
            let at = line_lo(prog.in_blocks[b].header.lo as usize);
            edits.push(Splice { at: Span::new(at, at), with: format!("{call}\n") });
        }
        _ => {}
    }
    for (k, b) in prog.in_blocks.iter().enumerate().filter(|(k, _)| emptied.contains(k)) {
        let instance_here = first.block == Some(k);
        for (i, part) in [b.header, b.close].into_iter().enumerate() {
            let (lo, hi) = (part.lo as usize, part.hi as usize);
            let ls = line_lo(lo);
            let le = text[hi..].find('\n').map_or(text.len(), |n| hi + n);
            let alone = text[ls..lo].trim().is_empty() && text[hi..le].trim().is_empty();
            let at = if alone { Span::new(ls, (le + 1).min(text.len())) } else { part };
            let with = if instance_here && i == 0 {
                if alone { format!("{call}\n") } else { call.clone() }
            } else {
                String::new()
            };
            edits.push(Splice { at, with });
        }
    }
    let mut outside: Vec<(usize, usize)> = units
        .iter()
        .filter(|u| !u.stmts.iter().any(|st| moved.contains(&st.id)))
        .map(|u| (u.lo, u.hi))
        .collect();
    outside.extend(
        prog.in_blocks
            .iter()
            .enumerate()
            .filter(|(k, _)| !emptied.contains(k))
            .map(|(_, b)| (b.header.lo as usize, b.header.hi as usize)),
    );
    for (lo, hi) in outside {
        for u in syntax::name_uses(&text[lo..hi]) {
            if names_in.contains(&u.parts[0].0) {
                let at = lo + u.parts[0].1.lo as usize;
                edits.push(Splice { at: Span::new(at, at), with: format!("{inst}.") });
            }
        }
    }
    let (at, with) = definition_at(prog, text, &def);
    match edits.iter_mut().find(|s| s.at.lo as usize == at) {
        Some(s) => s.with.insert_str(0, &with),
        None => edits.push(Splice { at: Span::new(at, at), with }),
    }
    let out = splice(text, edits);
    held(e, prog, &out)?;
    Ok(Edit { text: out, kind: Kind::Structural, names: vec![comp, inst], refused: None, kept })
}

/// Why a statement cannot be moved into a component, where it cannot: what a selection reaches
/// is a declaration, an instance, a block or a relation, and the rest is the document's own.
fn unmovable(st: &Stmt) -> Option<String> {
    match &st.kind {
        StmtKind::Decl(_) | StmtKind::Relation(_) | StmtKind::Instance(_) | StmtKind::Block(_)
        | StmtKind::Measured(_) | StmtKind::Minimize(_) | StmtKind::SolidRel(_)
        | StmtKind::Chain(_) | StmtKind::Set(_) | StmtKind::Branch(_) => None,
        _ => Some("only geometry and the relations on it can be made into a component".into()),
    }
}

/// The type a formal standing for `name`, which `st` binds, is written with, and whether it is
/// a number.  A chain, a view and a block's copies are no one thing a formal can be.
fn formal_type(name: &str, st: &Stmt, values: &Values) -> Result<(String, bool), String> {
    Ok(match &st.kind {
        StmtKind::Decl(d) => (d.kind.as_str().to_string(), false),
        StmtKind::Instance(_) | StmtKind::Group(_) => ("group".to_string(), false),
        StmtKind::Set(_) => ("set".to_string(), false),
        StmtKind::Param(p) => (values.ty(&p.name.text).to_string(), true),
        _ => {
            return Err(format!(
                "what is selected reads `{name}`, which cannot be handed to a component; select \
                 it too, or what it is made of"
            ));
        }
    })
}

/// The document's own values, worked out as far as they go: which of `Length`, `Angle` and
/// `Scalar` a formal standing for one is — or the type an input declared, `Int` among them.
struct Values {
    env: BTreeMap<String, crate::expr::Aff>,
    typed: BTreeMap<String, syntax::Ty>,
}

impl Values {
    fn new(prog: &Program, units: crate::units::Units) -> Values {
        let mut env = BTreeMap::new();
        let mut typed = BTreeMap::new();
        let params: Vec<&syntax::ParamDecl> = prog
            .root()
            .body
            .iter()
            .filter_map(|st| match &st.kind {
                StmtKind::Param(p) => Some(p),
                _ => None,
            })
            .collect();
        for p in &params {
            if let Some(ty) = p.input.as_ref().and_then(|i| i.ty) {
                typed.insert(p.name.text.clone(), ty);
                if !p.bound() {
                    env.insert(p.name.text.clone(), crate::expr::Aff::free(&p.name.text, ty.dim()));
                }
            }
        }
        // a value may read one defined below it: round again while a round settles another
        loop {
            let mut grew = false;
            for p in params.iter().filter(|p| p.bound()) {
                if env.contains_key(&p.name.text) {
                    continue;
                }
                if let Ok(a) = crate::flatten::value_aff(&p.text, &env, units) {
                    env.insert(p.name.text.clone(), a);
                    grew = true;
                }
            }
            if !grew {
                break;
            }
        }
        Values { env, typed }
    }

    fn ty(&self, name: &str) -> &'static str {
        if let Some(w) = self.typed.get(name).and_then(|t| t.word()) {
            return w;
        }
        match self.env.get(name) {
            Some(a) if a.dim == crate::units::Dim::LENGTH => "Length",
            Some(a) if a.dim == crate::units::Dim::ANGLE => "Angle",
            _ => "Scalar",
        }
    }
}

/// The root's statements as the lines they are written on, in order.
fn units_of<'a>(text: &str, body: &'a [Stmt], blocks: &[syntax::InBlock]) -> Vec<Unit<'a>> {
    let line_lo = |at: usize| text[..at].rfind('\n').map_or(0, |k| k + 1);
    let line_hi = |at: usize| text[at..].find('\n').map_or(text.len(), |k| at + k);
    let mut units: Vec<Unit> = Vec::new();
    for st in body {
        let (lo, hi) = (st.span.lo as usize, st.span.hi as usize);
        if hi > text.len() || lo > hi {
            continue; // a module's, past the document's own text
        }
        match units.last_mut() {
            Some(u) if line_lo(lo) <= line_hi(u.hi) => {
                u.stmts.push(st);
                u.lo = u.lo.min(lo);
                u.hi = u.hi.max(hi);
            }
            _ => units.push(Unit { stmts: vec![st], lo, hi, whole: false, block: None }),
        }
    }
    for u in &mut units {
        let (ls, le) = (line_lo(u.lo), line_hi(u.hi));
        let before = text[ls..u.lo].trim().is_empty();
        let tail = text[u.hi..le].trim();
        let after = tail.is_empty() || tail.starts_with("//");
        if before && after {
            u.lo = ls;
            u.hi = le;
            u.whole = true;
        } else if after && !tail.is_empty() {
            u.hi = le; // a statement's own comment goes with it
        }
        let (lo, hi) = (u.stmts[0].span.lo, u.stmts[u.stmts.len() - 1].span.hi);
        u.block = blocks.iter().position(|b| b.header.hi <= lo && hi <= b.close.lo);
    }
    units
}

/// How far a line is indented.
fn indent_of(line: &str) -> usize {
    line.len() - line.trim_start().len()
}

/// Lines of text moved under a new indentation: what they had in common taken off, `pad` put on.
fn indented(s: &str, pad: &str) -> String {
    let lines: Vec<&str> = s.lines().collect();
    let common =
        lines.iter().filter(|l| !l.trim().is_empty()).map(|l| indent_of(l)).min().unwrap_or(0);
    let mut out = String::new();
    for l in lines {
        if !l.trim().is_empty() {
            out.push_str(pad);
            out.push_str(l.get(common..).unwrap_or(l.trim_start()).trim_end());
        }
        out.push('\n');
    }
    out
}

/// Where the definition goes, and the text that puts it there: after the document's last own
/// component; else before its preview; else above its first statement (below `unit` and `use`).
fn definition_at(prog: &Program, text: &str, def: &str) -> (usize, String) {
    let root = prog.root();
    let own = prog
        .components
        .iter()
        .filter(|c| !std::ptr::eq(*c, root) && c.module.is_none())
        .map(|c| c.span.hi as usize)
        .filter(|&hi| hi <= text.len())
        .max();
    if let Some(hi) = own {
        return (hi, format!("\n\n{def}"));
    }
    if let Some(preview) = prog.preview {
        return (preview.lo as usize, format!("{def}\n\n"));
    }
    let first = root
        .body
        .iter()
        .filter(|st| !matches!(st.kind, StmtKind::Unit(_)))
        .map(|st| st.span.lo as usize)
        .chain(prog.in_blocks.iter().map(|b| b.header.lo as usize))
        .filter(|&lo| lo <= text.len())
        .min()
        .unwrap_or(text.len());
    let at = text[..first].rfind('\n').map_or(0, |k| k + 1);
    // set off from a heading, `unit` or `use` above it by a blank line, as below
    let lead = if at == 0 || text[..at].ends_with("\n\n") { "" } else { "\n" };
    (at, format!("{lead}{def}\n\n"))
}

/// The new text held to the elaborator: refused, in its words, for an error the document did not
/// have before.
fn held(e: &Elaborated, prog: &Program, text: &str) -> Result<(), String> {
    let (mut next, errs) = syntax::parse(text);
    let before = syntax::parse(prog.text()).1;
    if let Some(err) = errs.iter().find(|x| !before.iter().any(|b| b.message == x.message)) {
        return Err(err.message.clone());
    }
    crate::modules::relink(&mut next, prog);
    let after = crate::program::elaborate(&next);
    let had: BTreeSet<&str> = e.errors().map(|d| d.message.as_str()).collect();
    let new = after.errors().find(|d| !had.contains(d.message.as_str())).map(|d| d.message.clone());
    match new {
        Some(why) => Err(why),
        None => Ok(()),
    }
}
