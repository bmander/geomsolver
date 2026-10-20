//! **The model outline** (#435): every instance the document makes and its members, as a tree —
//! what a front end lists to find its way round a large document and to take things out of the
//! way (`Sketch::set_hidden`).
//!
//! It is an *outline* and not a feature tree: Solvent has no ordered history (a solid is a named
//! term, and order lives inside the term), so the tree is the document's **structure**, read off
//! the source map — every entity's statement and the instances and block copies enclosing it
//! (`Site`, `InstPath`).  The root body's statements come in source order; an instance is a row
//! with its members under it; a block or ring is a row with one child per copy; what a statement
//! made beside its own entity (a line's implicit ends, a plane's origin, a sweep's face) nests
//! under it; an object nests where it was made with its operands — features and cutters — under
//! it, listed for navigation only; and what a module's datums (`use std`) made is one group,
//! last.

use crate::ir::PathStep;
use crate::model::{EntKind, EntRef, Sketch};
use crate::program::{public_path, Elaborated};
use crate::syntax::{BlockKind, Span, StmtId, StmtKind};
use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet};

/// What a row of the outline stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeKind {
    /// `crank : Crank`, an instance of a component, its members under it.
    Instance,
    /// A `repeat`, `cycle` or `ring`, one child per copy.
    Block,
    /// One copy of a block.
    Copy,
    /// An entity a statement made.
    Entity,
    /// A solid: an object where it was made, or an operand listed under its object.
    Solid,
    /// What a module's datums made (`use std`), one group.
    Datums,
}

impl NodeKind {
    pub fn as_str(self) -> &'static str {
        match self {
            NodeKind::Instance => "instance",
            NodeKind::Block => "block",
            NodeKind::Copy => "copy",
            NodeKind::Entity => "entity",
            NodeKind::Solid => "solid",
            NodeKind::Datums => "datums",
        }
    }
}

/// One row of the outline.
#[derive(Clone, Debug)]
pub struct Node {
    pub label: String,
    pub kind: NodeKind,
    /// The component an instance is of.
    pub component: Option<String>,
    /// The name the row is carried across an edit by — the source map's name, `public_path` for
    /// a copy (`cyl[0]`) — or empty for an anonymous row, which cannot be carried, as a selection
    /// cannot (`line in crank`).
    pub path: String,
    /// The statement's span in the document; `None` for a module's statement.
    pub span: Option<Span>,
    /// The entities the row itself stands for: an entity row's own entity.
    pub ents: Vec<EntRef>,
    /// A solid row's solid.
    pub solid: Option<usize>,
    /// A feature or cutter listed under its object: there to be found, not to be hidden.
    pub operand: bool,
    pub children: Vec<Node>,
}

impl Node {
    fn new(label: String, kind: NodeKind, path: String, span: Option<Span>) -> Node {
        Node { label, kind, component: None, path, span, ents: Vec::new(), solid: None,
               operand: false, children: Vec::new() }
    }

    /// Every entity and solid of the row and the rows under it — what hiding it takes out of
    /// the drawing.
    pub fn subtree(&self, ents: &mut Vec<EntRef>, solids: &mut Vec<usize>) {
        ents.extend(&self.ents);
        solids.extend(self.solid);
        for c in &self.children {
            c.subtree(ents, solids);
        }
    }

    /// The row carried by `path`, here or under it.
    pub fn find(&self, path: &str) -> Option<&Node> {
        if self.path == path {
            return Some(self);
        }
        self.children.iter().find_map(|c| c.find(path))
    }
}

/// **The outline of a document**, its drawing being the elaboration's own sketch.
pub fn outline(e: &Elaborated) -> Vec<Node> {
    outline_with(e, &e.sketch)
}

/// The outline over `sk`, the drawing the elaboration made — handed beside it once a front end
/// has taken the sketch out (`gcs_elab_take_sketch`).
pub fn outline_with(e: &Elaborated, sk: &Sketch) -> Vec<Node> {
    let mut made_pos: BTreeMap<EntRef, usize> = BTreeMap::new();
    let mut ents: Vec<Placed> = Vec::new();
    for (&r, site) in &e.map.of_entity {
        // a twin is its point drawn again, called by the point's name: one row is the point's
        if e.map.twin_of.contains_key(&r) {
            continue;
        }
        if !made_pos.contains_key(&r) {
            for (k, m) in e.map.ents_made_by(site.stmt).enumerate() {
                made_pos.entry(m).or_insert(k);
            }
        }
        ents.push(Placed { e: r, stmt: site.stmt, path: site.path.0.clone(), at: site.span.lo });
    }
    let reader = Reader { e, made_pos, stmts: RefCell::new(BTreeMap::new()) };
    // what a module made at the root, beside no instance of the document's, is its datums
    let (datums, drawing): (Vec<Placed>, Vec<Placed>) = ents.into_iter()
        .partition(|p| p.path.is_empty() && reader.span(p.stmt).is_none());
    let mut rows = reader.level(drawing, 0, "");
    let mut groups: BTreeMap<String, Vec<Placed>> = BTreeMap::new();
    for p in datums {
        let module = match e.program.source_at(p.at as usize).0 {
            Some(k) => e.program.modules[k].name.clone(),
            None => String::new(),
        };
        groups.entry(module).or_default().push(p);
    }
    for (module, members) in groups {
        let label = if module.is_empty() { "datums".to_string() } else { module.clone() };
        let mut g = Node::new(label, NodeKind::Datums, module.clone(), None);
        g.children = reader.level(members, 0, &module);
        rows.push(g);
    }
    with_operands(rows, sk)
}

/// An entity and where it was made.
#[derive(Clone)]
struct Placed {
    e: EntRef,
    stmt: StmtId,
    path: Vec<PathStep>,
    /// Where its statement was written (a module's in the program's virtual text): what rows
    /// are put in order by.
    at: u32,
}

/// What a level of the tree is grouped by: the next step of the instance path, or, past the
/// last, the statement.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Key {
    Instance(StmtId),
    Block(StmtId),
    Stmt(StmtId),
}

struct Reader<'a> {
    e: &'a Elaborated,
    /// Each entity's place among what its statement made: a declaration's own entity first.
    made_pos: BTreeMap<EntRef, usize>,
    /// Each statement asked after: where it was written in the virtual text, and its span where
    /// the document wrote it — found once, since finding one walks the program.
    stmts: RefCell<BTreeMap<StmtId, (u32, Option<Span>)>>,
}

impl Reader<'_> {
    /// Statement `s`: where it was written, and its span if the document wrote it.
    fn stmt(&self, s: StmtId) -> (u32, Option<Span>) {
        *self.stmts.borrow_mut().entry(s).or_insert_with(|| {
            let p = &self.e.program;
            (p.stmt(s).map_or(u32::MAX, |st| st.span.lo), p.document_stmt(s).map(|st| st.span))
        })
    }

    /// Where statement `s` was written, in the program's virtual text.
    fn at(&self, s: StmtId) -> u32 {
        self.stmt(s).0
    }

    /// Statement `s`'s span, where the document wrote it.
    fn span(&self, s: StmtId) -> Option<Span> {
        self.stmt(s).1
    }

    /// The rows of one level: `ents` all share their first `depth` path steps, which `prefix`
    /// names (the source map's own prefix, `#9.0` for a copy, `pin` for an instance).
    fn level(&self, ents: Vec<Placed>, depth: usize, prefix: &str) -> Vec<Node> {
        let mut by: BTreeMap<Key, Vec<Placed>> = BTreeMap::new();
        for p in ents {
            let key = match p.path.get(depth) {
                Some(PathStep::Instance(s)) => Key::Instance(*s),
                Some(PathStep::Copy { block, .. }) => Key::Block(*block),
                None => Key::Stmt(p.stmt),
            };
            by.entry(key).or_default().push(p);
        }
        let mut rows: Vec<(u32, Node)> = Vec::new();
        for (key, members) in by {
            match key {
                Key::Instance(s) => {
                    rows.push((self.at(s), self.instance(s, members, depth, prefix)))
                }
                Key::Block(s) => rows.push((self.at(s), self.block(s, members, depth, prefix))),
                Key::Stmt(s) => {
                    let at = self.at(s).min(members.iter().map(|p| p.at).min().unwrap_or(u32::MAX));
                    rows.extend(self.statement(s, members, prefix).into_iter().map(|n| (at, n)));
                }
            }
        }
        // stable: two rows of one statement keep the order it made them in
        rows.sort_by_key(|(at, _)| *at);
        rows.into_iter().map(|(_, n)| n).collect()
    }

    /// An instance's row, its members under it.
    fn instance(&self, s: StmtId, members: Vec<Placed>, depth: usize, prefix: &str) -> Node {
        let (name, component) = match self.e.program.stmt(s).map(|st| &st.kind) {
            Some(StmtKind::Instance(i)) => (i.name.text.clone(), Some(i.component.text.clone())),
            _ => (String::new(), None),
        };
        let inner = join(prefix, &name);
        let path = carried(&inner);
        let label = match (&path, &component) {
            (p, Some(c)) if !p.is_empty() => format!("{} : {c}", tail(p, prefix)),
            (_, Some(c)) => in_place(c, &carried(prefix)),
            _ => "instance".to_string(),
        };
        let mut n = Node::new(label, NodeKind::Instance, path, self.span(s));
        n.component = component;
        n.children = self.level(members, depth + 1, &inner);
        n
    }

    /// A block's row, one child per copy — a copy that made one row is that row.
    fn block(&self, s: StmtId, members: Vec<Placed>, depth: usize, prefix: &str) -> Node {
        let names = self.bound_in(s);
        let mut copies: BTreeMap<u32, Vec<Placed>> = BTreeMap::new();
        for p in members {
            if let Some(PathStep::Copy { index, .. }) = p.path.get(depth) {
                copies.entry(*index).or_default().push(p);
            }
        }
        let count = copies.keys().next_back().map_or(0, |k| k + 1);
        let base = match names.len() {
            0 => String::new(),
            1 => names[0].clone(),
            _ => format!("{{{}}}", names.join(", ")),
        };
        // inside an instance the source cannot name, the block is carried by nothing either
        let path = match (prefix.is_empty(), carried(prefix)) {
            (true, _) => names.join(","),
            (false, outer) if outer.is_empty() || names.is_empty() => String::new(),
            (false, outer) => format!("{outer}.{}", names.join(",")),
        };
        let word = match self.e.program.stmt(s).map(|st| &st.kind) {
            Some(StmtKind::Block(b)) => match b.kind {
                BlockKind::Repeat => "repeat ",
                BlockKind::Cycle => "cycle ",
                BlockKind::Ring => "ring ",
                BlockKind::When => "when ",
            },
            _ => "block ",
        };
        let label = format!("{}[0‥{}]", if base.is_empty() { word } else { &base },
                            count.saturating_sub(1));
        let span = self.span(s);
        let mut n = Node::new(label, NodeKind::Block, path.clone(), span);
        for (index, members) in copies {
            let inner = join(prefix, &format!("#{}.{index}", s.0));
            let mut rows = self.level(members, depth + 1, &inner);
            if rows.len() == 1 {
                n.children.push(rows.remove(0));
                continue;
            }
            let label = format!("{base}[{index}]");
            let carried = if path.is_empty() { String::new() } else { format!("{path}[{index}]") };
            let mut copy = Node::new(label, NodeKind::Copy, carried, span);
            copy.children = rows;
            n.children.push(copy);
        }
        n
    }

    /// The names a block's body binds, as written.
    fn bound_in(&self, s: StmtId) -> Vec<String> {
        let Some(StmtKind::Block(b)) = self.e.program.stmt(s).map(|st| &st.kind) else {
            return Vec::new();
        };
        b.body.iter().filter_map(|st| match &st.kind {
            StmtKind::Decl(d) => d.name.shown().map(|n| n.text.clone()),
            k => k.bound_name().map(|n| n.text.clone()),
        }).filter(|n| !n.starts_with('#')).collect()
    }

    /// The rows one statement made at one level: each named entity under the longest other one
    /// its name extends (`std.front.origin` under `std.front`), an anonymous point, face or solid
    /// under the statement's first row (a line's ends, a sweep's face and stock), and the rest
    /// rows of their own.
    fn statement(&self, s: StmtId, mut members: Vec<Placed>, prefix: &str) -> Vec<Node> {
        members.sort_by_key(|p| (self.made_pos.get(&p.e).copied().unwrap_or(usize::MAX), p.e));
        let span = self.span(s);
        let names: Vec<String> = members.iter().map(|p| self.name(p.e)).collect();
        // which row each entity hangs under, by its index in `members`
        let mut under: Vec<Option<usize>> = vec![None; members.len()];
        for k in 0..members.len() {
            if names[k].is_empty() {
                continue;
            }
            under[k] = (0..members.len())
                .filter(|&j| j != k && !names[j].is_empty()
                    && names[k].strip_prefix(names[j].as_str()).is_some_and(|t| t.starts_with('.')))
                .max_by_key(|&j| names[j].len());
        }
        let first = (0..members.len()).find(|&k| under[k].is_none()
            && (!names[k].is_empty() || !Self::hangs(members[k].e.kind)));
        for k in 0..members.len() {
            if names[k].is_empty() && Self::hangs(members[k].e.kind) && first != Some(k) {
                under[k] = first;
            }
        }
        let row = |k: usize, parent: &str| -> Node {
            let e = members[k].e;
            let path = names[k].clone();
            let label = if path.is_empty() { in_place(e.kind.as_str(), &carried(parent)) }
                else { tail(&path, parent) };
            let mut n = Node::new(label, NodeKind::Entity, path, span);
            if e.kind == EntKind::Solid {
                n.kind = NodeKind::Solid;
                n.solid = Some(e.i());
            } else {
                n.ents.push(e);
            }
            n
        };
        fn hang(k: usize, parent: &str, names: &[String], under: &[Option<usize>],
                row: &dyn Fn(usize, &str) -> Node) -> Node {
            let mut n = row(k, parent);
            let here = if names[k].is_empty() { parent } else { &names[k] };
            n.children = (0..names.len()).filter(|&j| under[j] == Some(k))
                .map(|j| hang(j, here, names, under, row)).collect();
            n
        }
        let parent = carried(prefix);
        (0..members.len()).filter(|&k| under[k].is_none())
            .map(|k| hang(k, &parent, &names, &under, &row))
            .collect()
    }

    /// The kinds an anonymous entity of a statement hangs under its first row by.
    fn hangs(k: EntKind) -> bool {
        matches!(k, EntKind::Point | EntKind::Face | EntKind::Solid)
    }

    /// What the source calls an entity, as a reader writes it (`tip[0]`), or nothing.
    fn name(&self, e: EntRef) -> String {
        self.e.map.name_of(e).map(|n| carried(n)).unwrap_or_default()
    }
}

/// `prefix.name`, the source map's way of naming what an instance or a copy holds.
fn join(prefix: &str, name: &str) -> String {
    if prefix.is_empty() { name.to_string() } else { format!("{prefix}.{name}") }
}

/// The name a row is carried by: the source map's internal name as a reader writes it, or
/// nothing where it is a key no reader can (an unnamed call's `#i…`).
fn carried(name: &str) -> String {
    let p = public_path(name);
    if p.contains('#') { String::new() } else { p }
}

/// `name` relative to the row it is listed under: `q` under `pin`, `origin` under `std.front`.
fn tail(name: &str, parent: &str) -> String {
    match name.strip_prefix(parent).and_then(|t| t.strip_prefix('.')) {
        Some(t) if !parent.is_empty() => t.to_string(),
        _ => name.to_string(),
    }
}

/// What an anonymous row is called: what it is, and the named row it is in (`line in pin`), or
/// what it is alone at the root.
fn in_place(what: &str, place: &str) -> String {
    if place.is_empty() { what.to_string() } else { format!("{what} in {place}") }
}

/// **An object's operands under it.**  Each solid that is no object but some object's operand
/// — a feature, a cutter, a sweep's stock — is taken from where it was made and listed under
/// every object it is an operand of, marked `operand`, its own operands under it in turn.
fn with_operands(mut rows: Vec<Node>, sk: &Sketch) -> Vec<Node> {
    let objects: BTreeSet<usize> = super::objects(sk).into_iter().collect();
    let mut operand = BTreeSet::new();
    let mut stack: Vec<usize> = objects.iter().copied().collect();
    while let Some(i) = stack.pop() {
        for o in sk.solids.get(i).map(|s| s.operands()).unwrap_or_default() {
            let o = o as usize;
            if !objects.contains(&o) && operand.insert(o) {
                stack.push(o);
            }
        }
    }
    let mut taken: BTreeMap<usize, Node> = BTreeMap::new();
    take(&mut rows, &operand, &mut taken);
    place_operands(&mut rows, sk, &taken, 0);
    rows
}

/// Take every row of an operand solid out of `rows`, wherever it is, into `taken`.
fn take(rows: &mut Vec<Node>, operand: &BTreeSet<usize>, taken: &mut BTreeMap<usize, Node>) {
    let mut k = 0;
    while k < rows.len() {
        if let Some(i) = rows[k].solid.filter(|i| operand.contains(i)) {
            let mut n = rows.remove(k);
            take(&mut n.children, operand, taken);
            taken.entry(i).or_insert(n);
            continue;
        }
        take(&mut rows[k].children, operand, taken);
        k += 1;
    }
}

/// How deep operands are listed under operands: a term is acyclic (the elaborator refuses a
/// cycle), and this bounds the walk whatever it is handed.
const OPERAND_DEPTH: usize = 64;

/// Under each solid row, its operands' rows.
fn place_operands(rows: &mut [Node], sk: &Sketch, taken: &BTreeMap<usize, Node>, depth: usize) {
    for n in rows.iter_mut() {
        place_operands(&mut n.children, sk, taken, depth);
        let Some(i) = n.solid else { continue };
        if depth >= OPERAND_DEPTH {
            continue;
        }
        for o in sk.solids.get(i).map(|s| s.operands()).unwrap_or_default() {
            let Some(row) = taken.get(&(o as usize)) else { continue };
            let mut row = row.clone();
            row.operand = true;
            let mut one = vec![row];
            place_operands(&mut one, sk, taken, depth + 1);
            n.children.extend(one);
        }
    }
}
