//! **The names a text gives and reads** — the index an editor's features stand on: which word is
//! a name, which declaration it names, and every other place the text names it.  Asked of the
//! word under a caret it is the other places to light; asked of a symbol, where it was given
//! (go to definition) and all of its spellings (find references, rename).
//!
//! A function of the *text*, as the colouring is, and for the same reason: the program being
//! looked at is usually half-typed, which does not elaborate.  It stands on the colouring's own
//! scan (`highlight`), so what a word is coloured as and what this says it names are one answer —
//! a word coloured `Def` gives a name, a plain one reads a name, a `Type` before its parentheses
//! calls a component, a `Relation` the file defines or imports reads that word, a `Label` inside a
//! component's or a word's call names the formal it fills.  On that it lays the language's scopes
//! (§5): a component's, interface's or relation word's body is a scope of its own, holding
//! its formals and its own declarations; a set's body (`{ p | … }`), a `claim { … }`'s and a
//! `preview`'s are scopes their declarations stay in; a repetition (`repeat`, `cycle`, `ring`,
//! `when`) or an `in` block shares the scope around it, but for the binders its header gives
//! (`as i`, `repeat e in …`), as an `integral(… over p)` keeps its own.  A name is the innermost
//! declaration of it in scope, read outwards to the document's own; a body that reads a name its
//! scope does not give is reading what the file states, as a body reads its file's constants.
//!
//! **A member is reached through what it is read of**: `t0.mid`, where `t0 := Rung(…)`, is the
//! `mid` that `Rung`'s body declares, and so is `Rung(…).mid`; `layout.size.length`, through a
//! formal typed by an interface, is the interface's `length`; a group's member is its label
//! (`dims := {width: 20mm}` gives `dims.width`).  Where the text says no more — `l.p1` of a line,
//! `std.front` of a module — the member is one symbol per name per thing it is read of, so every
//! `l.p1` is one and no `p1` of anything else is.  `next.v` and `prev.v` in a cycle are `v`.
//!
//! A name nothing in scope gives is still a symbol, one per name per body (a component's, a
//! word's, or the document's), so the half-typed name lights its other spellings too.  Words the
//! grammar keeps (`over`, `in`, `at`, …) name nothing, nor does a built-in, a unit, a kind, a
//! label filling a kind's slot (`line(p2: c)`, `hint(r: 25)`) or a member held by `fix(x == 0)`.

use std::collections::BTreeMap;

use super::highlight::{defined_word, tints, Tint};
use super::lexer::{group_ahead, lex, Tok};
use super::words::{is_name, opens_block, set_at, QUALIFIERS};
use super::{Span, MAX_TEXT};
use crate::model::EntKind;

/// What gave a symbol its name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SymbolKind {
    /// `w := 100`, `p := point`, `(ab := line(a, b))`, `scenario lift` — what a statement
    /// declares
    Name,
    /// `param t: Angle over (…)`
    Param,
    /// a component's formal, an interface's member, a relation word's operand or parameter
    Formal,
    /// `as i`, `repeat e in …`, `{ p | … }`, `integral(… over p)` — bound for one body
    Binder,
    /// `component Rung(…)`, `interface Size { … }`
    Component,
    /// `a right_of(d) b := …` — a relation word the file defines
    Word,
    /// `use std` — the first name of a module's path
    Module,
    /// `use std (offset)` — a name a module's list brings in
    Import,
    /// `l.p1`, `std.front`, `dims.width` — a member, where nothing in the text declares it
    Member,
    /// a name read that nothing in its scope gives: half-typed, misspelt, or the core's own
    Unknown,
}

impl SymbolKind {
    /// The word a front end reads it by.
    pub fn as_str(self) -> &'static str {
        match self {
            SymbolKind::Name => "name",
            SymbolKind::Param => "param",
            SymbolKind::Formal => "formal",
            SymbolKind::Binder => "binder",
            SymbolKind::Component => "component",
            SymbolKind::Word => "word",
            SymbolKind::Module => "module",
            SymbolKind::Import => "import",
            SymbolKind::Member => "member",
            SymbolKind::Unknown => "unknown",
        }
    }
}

/// One thing the text names.
#[derive(Clone, Debug, PartialEq)]
pub struct Symbol {
    /// As written: the last part of a dotted name (`p1` of `l.p1`).
    pub name: String,
    pub kind: SymbolKind,
    /// What a member is read of: `l`'s symbol, for `l.p1`.  `None` but for a `Member`.
    pub parent: Option<usize>,
}

/// One place the text names a symbol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Occurrence {
    /// The word, in the text's bytes.
    pub span: Span,
    /// Index into `Symbols::symbols`.
    pub symbol: usize,
    /// Where the name is *given* — declared, bound, a formal, a group's label — rather than read.
    pub def: bool,
}

/// Every name a text gives and reads (`symbols`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Symbols {
    pub symbols: Vec<Symbol>,
    /// In text order, no two overlapping.
    pub occurrences: Vec<Occurrence>,
}

impl Symbols {
    /// The occurrence a caret at byte `offset` is in, or stands just past — `rim|` means `rim`,
    /// as it does in every editor.
    pub fn at(&self, offset: usize) -> Option<&Occurrence> {
        let off = offset as u32;
        let k = self.occurrences.partition_point(|o| o.span.hi < off);
        self.occurrences.get(k).filter(|o| o.span.lo <= off)
    }

    /// Every place `symbol` is named, in text order.
    pub fn of(&self, symbol: usize) -> impl Iterator<Item = &Occurrence> {
        self.occurrences.iter().filter(move |o| o.symbol == symbol)
    }
}

/// Index the names of a program (module docs).  Never fails: a text half-typed is indexed as far
/// as its colouring reads it, and one past `MAX_TEXT` has nothing indexed, as it has nothing
/// coloured.
pub fn symbols(src: &str) -> Symbols {
    if src.len() > MAX_TEXT {
        return Symbols::default();
    }
    let toks = lex(src).0.toks;
    let mut b = Build::default();
    b.scopes.push(Scope::new(0, Close::Never, true, true, false));
    b.walk(src, &toks, &tints(&toks));
    b.finish()
}

/// When a scope ends.
#[derive(Clone, Copy, PartialEq)]
enum Close {
    /// the document's
    Never,
    /// at the end of the statement opening it, at this depth of braces: a component, a word, a
    /// block, an `in`, a `claim { … }` or a `preview`
    Statement(i32),
    /// at the brace closing back to this depth: a set's body
    Brace(i32),
    /// at the parenthesis closing back to this depth: an `integral(…)`
    Paren(i32),
}

struct Scope<'a> {
    /// The scope around it; the document's is its own.
    parent: usize,
    close: Close,
    /// The declarations made in it are its own; otherwise they belong to the scope around it,
    /// as a repetition's do.
    owner: bool,
    /// A component's, interface's, word's or the document's: what a name nothing gives
    /// is told apart by.
    unit: bool,
    /// Names given here now are the scope's binders: a block's header, a set's binder, an
    /// integral's `over`.
    binds: bool,
    /// An interface's body: a `name: Type` line is a member it declares.
    interface: bool,
    /// Where a component's formals are, while they are being read: the depth of parentheses
    /// inside its list.
    formals: Option<i32>,
    names: BTreeMap<&'a str, usize>,
}

impl Scope<'_> {
    fn new(parent: usize, close: Close, owner: bool, unit: bool, binds: bool) -> Self {
        Scope {
            parent,
            close,
            owner,
            unit,
            binds,
            interface: false,
            formals: None,
            names: BTreeMap::new(),
        }
    }
}

/// How a word is resolved, once every declaration is known.
#[derive(Clone, Copy)]
enum Base {
    /// already: a declaration, and so where its name is given
    Given(usize),
    /// by name from this scope outwards, an unknown where nothing gives it
    Read(usize),
    /// by name from this scope outwards, nothing where nothing gives it: a relation word, which
    /// may be the core's own
    Word(usize),
    /// a member of what the node before it is
    Field(usize),
    /// the formal of what the call at this node is, nothing where it has none
    Label(usize),
    /// `next` or `prev`: no name of its own (it names nothing), its field read in this scope
    Copy(usize),
}

struct Node<'a> {
    name: &'a str,
    span: Span,
    base: Base,
}

/// What a name declared just now is waiting for, to know what its members are.
#[derive(Clone, Copy, PartialEq)]
enum Typing {
    /// `t0` of `t0 := Rung(…)`: the name before the call's `(`
    Call,
    /// `layout` of `layout: Layout`: the interface's name
    Formal,
}

#[derive(Default)]
struct Build<'a> {
    scopes: Vec<Scope<'a>>,
    symbols: Vec<Symbol>,
    /// the scope a component or a word opens, by symbol
    opens: BTreeMap<usize, usize>,
    /// the node naming what a symbol is an instance of, or the interface it is typed by
    types: BTreeMap<usize, usize>,
    nodes: Vec<Node<'a>>,
    members: BTreeMap<(usize, &'a str), usize>,
    unknowns: BTreeMap<(usize, &'a str), usize>,
    resolved: Vec<Option<Option<usize>>>,
}

impl<'a> Build<'a> {
    fn walk(&mut self, src: &str, toks: &'a [(Tok, Span)], tints: &[Option<Tint>]) {
        let tok = |k: usize| toks.get(k).map(|(t, _)| t);
        let mut cur = 0usize;
        let mut braces = 0i32;
        let mut parens = 0i32;
        let mut start = true;
        // the node a `.` or `[` continues, and the one the `.` just read continues
        let mut path: Option<usize> = None;
        let mut field: Option<usize> = None;
        let mut squares: Vec<Option<usize>> = Vec::new();
        // per open parenthesis: the node it calls, and whether it is `fix`'s
        let mut calls: Vec<(Option<usize>, bool)> = Vec::new();
        // per open brace that opens a group's members: the symbol they are members of, and the
        // depth of parentheses its labels stand at
        let mut groups: Vec<Option<(Option<usize>, i32)>> = Vec::new();
        let mut typing: Option<(usize, Typing)> = None;
        // the last label a group gave, which a group written in place under it is the members of
        let mut member: Option<usize> = None;
        // a relation word's head: the word's token, and the `:=` ending it
        let mut head: Option<(usize, usize)> = None;
        // `component` or `interface` just read: the name after it opens the body
        let mut unit: Option<bool> = None;
        // `use`: 1 the module's path, 2 the list it imports
        let mut using = 0u8;
        for i in 0..toks.len() {
            let (t, span) = (&toks[i].0, toks[i].1);
            let prev = i.checked_sub(1).and_then(|j| tok(j));
            let after_dot = field.take();
            if !matches!(t, Tok::Ident(_) | Tok::P('.' | '(' | ':' | '{') | Tok::Define) {
                typing = None;
            }
            match t {
                Tok::Nl => {
                    cur = self.close_while(cur, |c| match c {
                        Close::Statement(d) => d >= braces,
                        Close::Paren(_) => true,
                        _ => false,
                    });
                    start = true;
                    path = None;
                    using = 0;
                    unit = None;
                    calls.clear();
                    squares.clear();
                    parens = 0;
                    continue;
                }
                Tok::P('{') => {
                    let set = set_at(toks, i).is_some();
                    // the lexer's rule: after `:=` a list of members, and after a member's `:`
                    // where the brace around it is one
                    let group = match prev {
                        Some(Tok::Define) => !set && group_ahead(&src[span.hi as usize..]),
                        Some(Tok::P(':')) => groups.last().is_some_and(|g| g.is_some()),
                        _ => false,
                    };
                    let owner = match prev {
                        Some(Tok::Define) => typing.map(|(s, _)| s),
                        _ => member,
                    };
                    groups.push(group.then_some((owner, parens)));
                    typing = None;
                    if set {
                        cur = self.push(cur, Close::Brace(braces), true, false, true);
                    } else if !group {
                        let s = &mut self.scopes[cur];
                        s.binds &= !matches!(s.close, Close::Statement(_));
                        s.formals = None;
                    }
                    braces += 1;
                    start = !group;
                    path = None;
                    unit = None;
                    continue;
                }
                Tok::P('}') => {
                    braces = (braces - 1).max(0);
                    groups.pop();
                    cur = self.close_while(cur, |c| match c {
                        Close::Brace(d) => d >= braces,
                        Close::Statement(d) => d > braces,
                        Close::Paren(_) => true,
                        Close::Never => false,
                    });
                    start = true;
                    path = None;
                    continue;
                }
                Tok::P('|') => {
                    // a set's binder is given: the rest of its body declares, as a body does
                    if matches!(self.scopes[cur].close, Close::Brace(_)) {
                        self.scopes[cur].binds = false;
                    }
                    path = None;
                    continue;
                }
                Tok::P('(') => {
                    if let (Some((s, Typing::Call)), Some(n)) = (typing, path) {
                        self.types.insert(s, n);
                        typing = None;
                    }
                    if using == 1 {
                        using = 2;
                    }
                    let fix = matches!(prev, Some(Tok::Ident(w)) if w == "fix");
                    calls.push((path, fix));
                    if matches!(prev, Some(Tok::Ident(w)) if w == "integral") {
                        cur = self.push(cur, Close::Paren(parens), false, false, true);
                    }
                    parens += 1;
                    path = None;
                    continue;
                }
                Tok::P(')') => {
                    parens = (parens - 1).max(0);
                    let (callee, _) = calls.pop().unwrap_or((None, false));
                    cur = self.close_while(cur, |c| matches!(c, Close::Paren(d) if d >= parens));
                    if self.scopes[cur].formals.is_some_and(|d| d > parens) {
                        self.scopes[cur].formals = None;
                    }
                    // `Comp(…).face` — a member of what was called
                    path = callee;
                    start = false;
                    continue;
                }
                Tok::P('[') => {
                    squares.push(path);
                    path = None;
                    continue;
                }
                Tok::P(']') => {
                    path = squares.pop().flatten();
                    continue;
                }
                Tok::P('.') => {
                    field = path;
                    path = None;
                    continue;
                }
                Tok::P(':') => {
                    path = None;
                    continue;
                }
                Tok::Define => {
                    typing = typing.filter(|&(_, k)| k == Typing::Call);
                    start = false;
                    path = None;
                    continue;
                }
                Tok::Ident(_) => {}
                _ => {
                    start = false;
                    path = None;
                    continue;
                }
            }
            let Tok::Ident(w) = t else { unreachable!("every other token continued above") };
            let w = w.as_str();
            let next = tok(i + 1);
            let at_start = std::mem::replace(&mut start, false);
            path = None;

            // what a statement opens, read at its first word
            if at_start {
                if let Some(h) = defined_word(toks, i) {
                    head = Some(h);
                    cur = self.push(cur, Close::Statement(braces), true, true, false);
                } else if w == "component" || w == "interface" {
                    unit = Some(w == "interface");
                } else if w == "use" {
                    using = 1;
                    continue;
                } else if opens_block(toks, i) {
                    cur = self.push(cur, Close::Statement(braces), false, false, w != "when");
                } else if w == "in" {
                    cur = self.push(cur, Close::Statement(braces), false, false, false);
                } else if (w == "claim" || w == "preview") && next == Some(&Tok::P('{')) {
                    cur = self.push(cur, Close::Statement(braces), true, false, false);
                } else if QUALIFIERS.contains(&w) {
                    start = true;
                }
            }

            // a relation word's head: the word is the document's, its operands and parameters
            // the body's
            if let Some((word, _)) = head.filter(|&(_, end)| i < end) {
                if i == word {
                    let owner = self.up(self.scopes[cur].parent, |s| s.owner);
                    let s = self.define(owner, w, SymbolKind::Word);
                    self.opens.insert(s, cur);
                    self.given(s, w, span);
                } else if prev != Some(&Tok::P(':')) && is_name(w) {
                    let s = self.define(cur, w, SymbolKind::Formal);
                    self.given(s, w, span);
                }
                continue;
            }

            if using > 0 {
                if let Some(of) = after_dot {
                    path = Some(self.node(w, span, Base::Field(of)));
                } else if using == 1 {
                    let owner = self.up(cur, |s| s.owner);
                    let s = self.define(owner, w, SymbolKind::Module);
                    path = Some(self.given(s, w, span));
                } else if is_name(w) {
                    let owner = self.up(cur, |s| s.owner);
                    let s = self.define(owner, w, SymbolKind::Import);
                    self.given(s, w, span);
                }
                continue;
            }

            // a member: `t0.mid`, `std.front`, `Comp(…).face`, `next.v`
            if let Some(of) = after_dot {
                let base = match self.nodes[of].base {
                    Base::Copy(sc) => Base::Read(sc),
                    _ => Base::Field(of),
                };
                let n = self.node(w, span, base);
                path = Some(n);
                if let Some((s, Typing::Formal)) = typing {
                    self.types.insert(s, n);
                }
                continue;
            }

            // an interface's member: `origin: point`
            if at_start && self.scopes[cur].interface && next == Some(&Tok::P(':')) {
                let s = self.define(cur, w, SymbolKind::Formal);
                self.given(s, w, span);
                typing = Some((s, Typing::Formal));
                continue;
            }

            match tints[i] {
                Some(Tint::Def) => {
                    let binds = self.scopes[cur].binds;
                    let kind = match unit {
                        Some(_) => SymbolKind::Component,
                        None if binds => SymbolKind::Binder,
                        None if matches!(prev, Some(Tok::Ident(p)) if p == "param") => {
                            SymbolKind::Param
                        }
                        None => SymbolKind::Name,
                    };
                    let into = if binds { cur } else { self.up(cur, |s| s.owner) };
                    let s = self.define(into, w, kind);
                    self.given(s, w, span);
                    if let Some(interface) = unit.take() {
                        cur = self.push(cur, Close::Statement(braces), true, true, false);
                        self.opens.insert(s, cur);
                        self.scopes[cur].interface = interface;
                        self.scopes[cur].formals = (!interface).then_some(parens + 1);
                    }
                    typing = (next == Some(&Tok::Define)).then_some((s, Typing::Call));
                    continue;
                }
                Some(Tint::Label) => {
                    let group = groups.last().copied().flatten().filter(|&(_, d)| d == parens);
                    if self.scopes[cur].formals == Some(parens) {
                        let s = self.define(cur, w, SymbolKind::Formal);
                        self.given(s, w, span);
                        typing = Some((s, Typing::Formal));
                        continue;
                    } else if let Some((Some(owner), _)) = group {
                        let s = self.member(owner, w);
                        self.given(s, w, span);
                        member = Some(s);
                    } else if let Some(&(Some(callee), _)) = calls.last() {
                        self.node(w, span, Base::Label(callee));
                    }
                }
                // a call of a component, or of what the core makes (`trace(…)`), which no
                // declaration gives and so names nothing
                Some(Tint::Type) if next == Some(&Tok::P('(')) && EntKind::parse(w).is_none() => {
                    path = Some(self.node(w, span, Base::Word(cur)));
                }
                Some(Tint::Relation) => {
                    path = Some(self.node(w, span, Base::Word(cur)));
                }
                Some(Tint::Word) if (w == "next" || w == "prev") && next == Some(&Tok::P('.')) => {
                    path = Some(self.node(w, span, Base::Copy(cur)));
                }
                None if is_name(w) => {
                    // `fix(x == 0, y == 0) p` — a member the word holds, of the operand after it
                    let held = next == Some(&Tok::EqEq) && calls.last().is_some_and(|c| c.1);
                    if !held {
                        let n = self.node(w, span, Base::Read(cur));
                        path = Some(n);
                        if let Some((s, Typing::Formal)) = typing {
                            self.types.insert(s, n);
                        }
                    }
                }
                _ => {}
            }
            if path.is_none() {
                typing = None;
            }
        }
    }

    /// The scope around `cur` past every innermost one that ends at what `ends` says.
    fn close_while(&self, mut cur: usize, ends: impl Fn(Close) -> bool) -> usize {
        while ends(self.scopes[cur].close) {
            cur = self.scopes[cur].parent;
        }
        cur
    }

    fn push(&mut self, cur: usize, close: Close, owner: bool, unit: bool, binds: bool) -> usize {
        self.scopes.push(Scope::new(cur, close, owner, unit, binds));
        self.scopes.len() - 1
    }

    /// The innermost scope from `s` outwards that `is` says, the document's at the last — which
    /// is every kind there is.
    fn up(&self, mut s: usize, is: impl Fn(&Scope) -> bool) -> usize {
        while !is(&self.scopes[s]) {
            s = self.scopes[s].parent;
        }
        s
    }

    /// The symbol `name` is in scope `s`, made now if this is its first declaration there.  A
    /// name declared twice in one scope is one symbol, the copies of a block's statement or a
    /// mistake, and the elaborator says which.
    fn define(&mut self, s: usize, name: &'a str, kind: SymbolKind) -> usize {
        intern(&mut self.symbols, &mut self.scopes[s].names, name, name, kind, None)
    }

    fn member(&mut self, of: usize, name: &'a str) -> usize {
        intern(&mut self.symbols, &mut self.members, (of, name), name, SymbolKind::Member, Some(of))
    }

    fn node(&mut self, name: &'a str, span: Span, base: Base) -> usize {
        self.nodes.push(Node { name, span, base });
        self.nodes.len() - 1
    }

    fn given(&mut self, s: usize, name: &'a str, span: Span) -> usize {
        self.node(name, span, Base::Given(s))
    }

    fn lookup(&self, mut s: usize, name: &str) -> Option<usize> {
        loop {
            if let Some(&k) = self.scopes[s].names.get(name) {
                return Some(k);
            }
            if s == 0 {
                return None;
            }
            s = self.scopes[s].parent;
        }
    }

    /// What `s` declares as `name`: a component's or a word's own, or that of what `s` is an
    /// instance of or typed by.
    fn declared_in(&mut self, s: usize, name: &str) -> Option<usize> {
        let holder = match self.symbols[s].kind {
            SymbolKind::Component | SymbolKind::Word => s,
            _ => self.resolve(*self.types.get(&s)?)?,
        };
        let scope = *self.opens.get(&holder)?;
        self.scopes[scope].names.get(name).copied()
    }

    fn resolve(&mut self, n: usize) -> Option<usize> {
        if let Some(r) = self.resolved[n] {
            return r;
        }
        // a cycle of types (`a := a.b(…)`) resolves to nothing rather than forever
        self.resolved[n] = Some(None);
        let name = self.nodes[n].name;
        let r = match self.nodes[n].base {
            Base::Given(s) => Some(s),
            Base::Read(sc) => Some(match self.lookup(sc, name) {
                Some(s) => s,
                None => {
                    let key = (self.up(sc, |s| s.unit), name);
                    let kind = SymbolKind::Unknown;
                    intern(&mut self.symbols, &mut self.unknowns, key, name, kind, None)
                }
            }),
            Base::Word(sc) => self.lookup(sc, name),
            Base::Copy(_) => None,
            Base::Field(of) => {
                let p = self.resolve(of)?;
                Some(match self.declared_in(p, name) {
                    Some(s) => s,
                    None => self.member(p, name),
                })
            }
            Base::Label(call) => {
                let c = self.resolve(call)?;
                self.declared_in(c, name)
            }
        };
        self.resolved[n] = Some(r);
        r
    }

    fn finish(mut self) -> Symbols {
        self.resolved = vec![None; self.nodes.len()];
        let mut occurrences = Vec::with_capacity(self.nodes.len());
        for n in 0..self.nodes.len() {
            if let Some(symbol) = self.resolve(n) {
                let node = &self.nodes[n];
                let def = matches!(node.base, Base::Given(_));
                occurrences.push(Occurrence { span: node.span, symbol, def });
            }
        }
        Symbols { symbols: self.symbols, occurrences }
    }
}

/// The symbol `key` is in `map`, made now — named `name`, of `kind` — if it is not there yet.
fn intern<K: Ord>(
    symbols: &mut Vec<Symbol>,
    map: &mut BTreeMap<K, usize>,
    key: K,
    name: &str,
    kind: SymbolKind,
    parent: Option<usize>,
) -> usize {
    *map.entry(key).or_insert_with(|| {
        symbols.push(Symbol { name: name.to_string(), kind, parent });
        symbols.len() - 1
    })
}
