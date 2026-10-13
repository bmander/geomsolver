//! **Interfaces** (§8.1 [0.84]): a `group` formal typed by a named shape, `gear: Gear`.
//!
//! A group formal reads whatever its argument carries; one typed by an interface asks the call
//! for every member the interface declares, each of its type, and gives the body only those.
//! Structural, as the rest of the language is: an instance or a group conforms by having the
//! members, never by saying so.  Three places hold it, each where what it judges is known:
//!
//! * the **numbers** at the call (`Walk::interface_numbers`, from `bind`): each a number of the
//!   member's dimension, and only the declared ones copied into the body's `vals`;
//! * the **geometry, sets and groups** once every name is known (`Walk::conform`, from
//!   `resolve`): each present, public, and a set or a group where one is asked for; an entity's
//!   kind is the elaborator's to judge (`Expansion::kinds`), as a `path` formal's is;
//! * the **body** wherever a reference is resolved (`undeclared`, asked by `lookup`): a member
//!   the interface does not declare is no name there, E101.
//!
//! An argument read through the caller's own interface is held to it as well (`lacks`): a body
//! may hand on only what its interface promised, so what one declares is all it relies on.
//! Handed to an untyped `group` formal, it is read as any group is.

use super::{typed, Code, Scope, Walk};
use crate::expr::Aff;
use crate::syntax::{Component, Formal, Program, Span, Ty};
use std::collections::BTreeMap;
use std::rc::Rc;

/// How deep interfaces may nest: past it, one reaches itself (refused where it is defined).
const MAX_NEST: usize = 16;

/// An interface, resolved: what each member must be, nested interfaces included.
#[derive(Debug)]
pub(super) struct Shape {
    /// What the program calls it (`Gear`, `spiral.Gear`), for what is said about it.
    pub name: String,
    pub members: Vec<(String, Member)>,
}

/// What an interface's member must be.
#[derive(Clone, Debug)]
pub(super) enum Member {
    /// A number of this type — `Length`, `Angle`, `Scalar`, `Int`.
    Number(Ty),
    /// An entity of a kind (`Ty::Ent`), or any path (`Ty::Path`).
    Entity(Ty),
    Set,
    /// A group or an instance: any (`group`), or one of an interface's shape.
    Group(Option<Rc<Shape>>),
}

impl Member {
    /// What a member of this kind is called where it is missing or wrong: `a line`, `a Length`.
    fn a(&self) -> String {
        match self {
            Member::Number(ty) => format!("a number ({})", ty.word().unwrap_or("Scalar")),
            Member::Entity(Ty::Ent(k)) => k.a(),
            Member::Entity(_) => "a path".to_string(),
            Member::Set => "a set".to_string(),
            Member::Group(None) => "a group".to_string(),
            Member::Group(Some(s)) => format!("a `{}`", s.name),
        }
    }
}

impl Shape {
    pub fn member(&self, name: &str) -> Option<&Member> {
        self.members.iter().find(|(n, _)| n == name).map(|(_, m)| m)
    }

    /// The first member a dotted read past a formal of this shape names that the interface does
    /// not declare — the interface's name and the member's — following nested interfaces, and
    /// stopping at whatever is not one (a line's `p1` is the line's).
    pub fn undeclared<'s>(&self, mut segs: impl Iterator<Item = &'s str>) -> Option<(String, String)> {
        let Some(first) = segs.next() else { return None };
        match self.member(first) {
            None => Some((self.name.clone(), first.to_string())),
            Some(Member::Group(Some(inner))) => inner.undeclared(segs),
            Some(_) => None,
        }
    }
}

/// The member a reference reads through a formal its scope's interface types, where the
/// interface does not declare it: the interface's name and the member's.
pub(super) fn undeclared(r: &crate::syntax::Ref, sc: &Scope) -> Option<(String, String)> {
    let shape = sc.interfaces.get(&r.root.text)?;
    let fields = r.path.iter().map_while(|s| match s {
        crate::syntax::Seg::Field(f) => Some(f.text.as_str()),
        _ => None,
    });
    shape.undeclared(fields)
}

/// The same of a dotted name a text reads (`design.width`).
pub(super) fn undeclared_name(name: &str, sc: &Scope) -> Option<(String, String)> {
    let (head, rest) = name.split_once('.')?;
    sc.interfaces.get(head)?.undeclared(rest.split('.'))
}

/// Whether an entity of this kind is a path (#251): what a point runs along.
fn runs_along(k: crate::model::EntKind) -> bool {
    use crate::model::EntKind::{Arc, Circle, Curve, Line, Spline};
    matches!(k, Line | Arc | Circle | Spline | Curve)
}

/// What a member of `want` has no counterpart for in `have`, or holds of another type: the
/// first such, said.  `None` where `have` declares everything `want` asks for — an untyped group
/// member of `have` standing for any shape, since what it is given is judged at that call.
fn lacks(want: &Shape, have: &Shape, depth: usize) -> Option<String> {
    for (name, w) in &want.members {
        let Some(h) = have.member(name) else {
            return Some(format!("declares no `{name}`, which `{}` asks for", want.name));
        };
        let fits = match (w, h) {
            (Member::Number(a), Member::Number(b)) => a.dim() == b.dim(),
            (Member::Entity(Ty::Path), Member::Entity(Ty::Ent(k))) => runs_along(*k),
            (Member::Entity(a), Member::Entity(b)) => a == b,
            (Member::Set, Member::Set) => true,
            (Member::Group(None), Member::Group(_)) | (Member::Group(_), Member::Group(None)) => true,
            (Member::Group(Some(a)), Member::Group(Some(b))) => {
                if depth < MAX_NEST {
                    if let Some(why) = lacks(a, b, depth + 1) {
                        return Some(format!("declares `{name}` a `{}`, which {why}", b.name));
                    }
                }
                true
            }
            _ => false,
        };
        if !fits {
            return Some(format!("declares `{name}` {}, where `{}` asks for {}", h.a(), want.name, w.a()));
        }
    }
    None
}

impl<'a> Walk<'a> {
    /// Interface `i` resolved, nested interfaces with it; `None` past `MAX_NEST`, where it
    /// reaches itself.  What is wrong with how one is written is said once, of the text
    /// (`check`): here an unresolved member type is read as an untyped group.
    fn shape(&self, i: usize, depth: usize) -> Option<Rc<Shape>> {
        if depth > MAX_NEST {
            return None;
        }
        let f = &self.prog.interfaces[i];
        let members = f.members.iter().map(|m| {
            let member = match m.ty {
                _ if m.interface.is_some() => Member::Group(
                    m.interface.as_ref()
                        .and_then(|n| self.prog.resolve_interface(&n.text, f.module).ok())
                        .and_then(|j| self.shape(j, depth + 1)),
                ),
                ty if ty.number() => Member::Number(ty),
                Ty::Set => Member::Set,
                Ty::Group | Ty::Side => Member::Group(None),
                ty => Member::Entity(ty),
            };
            (m.name.text.clone(), member)
        });
        let members = members.collect();
        Some(Rc::new(Shape { name: self.prog.interface_id(i), members }))
    }

    /// The interface a component's formal is typed by, resolved from the file it was written in.
    pub(super) fn formal_shape(&self, comp: &Component, f: &Formal) -> Option<Rc<Shape>> {
        let n = f.interface.as_ref()?;
        let i = self.prog.resolve_interface(&n.text, comp.module).ok()?;
        self.shape(i, 0)
    }

    /// Every typed formal of a component, by name: what its body's references are held to.
    pub(super) fn formal_shapes(&self, comp: &Component) -> BTreeMap<String, Rc<Shape>> {
        comp.formals.iter()
            .filter_map(|f| Some((f.name.text.clone(), self.formal_shape(comp, f)?)))
            .collect()
    }

    /// The numbers an interface asks of `actual` (as written in the caller, `gear_cone`),
    /// each of its member's dimension, copied in under the formal's name (`gear.pitch`) — and
    /// no others: a number the interface does not declare is no name in the body.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn interface_numbers(
        &mut self,
        shape: &Shape,
        actual: &str,
        formal: &str,
        vals: &BTreeMap<String, Aff>,
        sub: &mut BTreeMap<String, Aff>,
        span: Span,
        say: bool,
        depth: usize,
    ) -> bool {
        let mut ok = true;
        for (name, member) in &shape.members {
            let (from, to) = (format!("{actual}.{name}"), format!("{formal}.{name}"));
            match member {
                Member::Number(ty) => {
                    let why = match vals.get(&from).map(|v| typed(v.clone(), *ty, &to)) {
                        Some(Ok(v)) => {
                            sub.insert(to, v);
                            continue;
                        }
                        Some(Err(e)) => e,
                        None => format!(
                            "`{actual}` has no number `{name}`, which `{}` asks for ({})",
                            shape.name, ty.word().unwrap_or("Scalar")
                        ),
                    };
                    if say {
                        self.err(Code::E103, span, why);
                    }
                    // the call is refused (`nonconforming`): what stands in keeps the body's
                    // numbers from saying so again at every read
                    sub.insert(to, Aff::num(0.0).as_dim(ty.dim()));
                    ok = false;
                }
                Member::Group(Some(inner)) if depth < MAX_NEST => {
                    ok &= self.interface_numbers(inner, &from, &to, vals, sub, span, say, depth + 1)
                }
                // a group or a set carries what it carries (`s.r` of a sphere)
                Member::Group(_) | Member::Set => {
                    let start = format!("{from}.");
                    for (k, v) in vals {
                        if let Some(rest) = k.strip_prefix(&start) {
                            sub.insert(format!("{to}.{rest}"), v.clone());
                        }
                    }
                }
                Member::Entity(_) => {}
            }
        }
        ok
    }

    /// An argument read through the caller's own interface (`gear` in a body typed `gear:
    /// Gear`, or `gear.cone`) is held to what that interface declares: a body hands on only what
    /// it was promised.
    /// Whether it is (`false` said).
    pub(super) fn handed_on(&mut self, want: &Shape, r: &crate::syntax::Ref, scope: &Scope, span: Span) -> bool {
        let Some(mut have) = scope.interfaces.get(&r.root.text) else { return true };
        for seg in &r.path {
            let crate::syntax::Seg::Field(f) = seg else { return true };
            match have.member(&f.text) {
                Some(Member::Group(Some(inner))) => have = inner,
                // an untyped group's members are its own; anything else is no group
                _ => return true,
            }
        }
        let Some(why) = lacks(want, have, 0) else { return true };
        let what = super::written(r);
        self.err(Code::E103, span, format!("`{what}` is a `{}`, which {why}", have.name));
        false
    }

    /// What an interface asks of the group or instance `target` (absolute, `pair.gear_cone`)
    /// that a number is not: each member present and public, a set or a group where one is asked
    /// for, and an entity's kind left to the elaborator.  `written` is the argument as the call
    /// wrote it, with the path to the member reached so far.
    pub(super) fn conform(
        &mut self,
        shape: &Shape,
        target: &str,
        written: &str,
        alias: &BTreeMap<String, String>,
        span: Span,
        depth: usize,
    ) -> bool {
        let mut ok = true;
        for (name, member) in &shape.members {
            if matches!(member, Member::Number(_)) {
                continue; // `interface_numbers`, at the call
            }
            let key = format!("{target}.{name}");
            let here = format!("{written}.{name}");
            let present = self.names.contains(&key)
                || alias.contains_key(&key)
                || self.sets.contains_key(&key)
                || self.group_names.contains(&key);
            if !present {
                self.err(Code::E103, span, format!(
                    "`{written}` has no `{name}`, which `{}` asks for ({})",
                    shape.name, member.a()
                ));
                ok = false;
                continue;
            }
            if self.private_names.contains_key(&key) {
                self.err(Code::E103, span, format!(
                    "`{here}` is private, and `{}` asks for it", shape.name
                ));
                ok = false;
                continue;
            }
            let to = alias.get(&key).cloned().unwrap_or(key);
            let set = self.sets.contains_key(&to);
            let group = self.group_names.contains(&to);
            let wrong = match member {
                Member::Set if !set => format!("`{here}` is not a set, which `{}` asks for", shape.name),
                Member::Group(_) if !group => format!(
                    "`{here}` is not a group or an instance, which `{}` asks for", shape.name
                ),
                Member::Entity(_) if set || group => format!(
                    "`{here}` is {}, where `{}` asks for {}",
                    if set { "a set" } else { "an instance or a group" },
                    shape.name, member.a()
                ),
                _ => String::new(),
            };
            if !wrong.is_empty() {
                self.err(Code::E103, span, wrong);
                ok = false;
                continue;
            }
            match member {
                Member::Group(Some(inner)) if depth < MAX_NEST => {
                    ok &= self.conform(inner, &to, &here, alias, span, depth + 1)
                }
                Member::Entity(ty) => self.kinds.push(super::MemberKind {
                    target: to,
                    span,
                    ty: *ty,
                    member: here,
                    interface: shape.name.clone(),
                }),
                _ => {}
            }
        }
        ok
    }
}

/// **What is wrong with an interface as written**, asked of the text (§8.1 [0.84]): every
/// definition in every file, and every type a formal or a member names — defined twice in one
/// file E071, a member twice E001, a `Side` member E103 (a group holds no side), a type naming
/// no interface E101, one reaching itself E003.
pub(crate) fn check(p: &Program, diags: &mut Vec<crate::program::Diag>) {
    use crate::program::Diag;
    let mut said: Vec<Diag> = Vec::new();
    let mut say = |code: Code, span: Span, message: String| {
        said.push(Diag { code, span, stmt: None, message });
    };
    let mut first: BTreeMap<(Option<usize>, &str), Span> = BTreeMap::new();
    for f in &p.interfaces {
        if let Some(&was) = first.get(&(f.module, f.name.text.as_str())) {
            let line = p.line_col(was.lo as usize).0;
            say(Code::E071, f.name.span,
                format!("`{}` is defined twice; the first is at line {line}", f.name.text));
            continue;
        }
        first.insert((f.module, f.name.text.as_str()), f.name.span);
        if p.component_in(f.module, &f.name.text).is_some() {
            say(Code::E071, f.name.span, format!(
                "`{}` names a component of this file too: an interface takes a name of its own",
                f.name.text
            ));
        }
        let mut seen: Vec<&str> = Vec::new();
        for m in &f.members {
            if seen.contains(&m.name.text.as_str()) {
                say(Code::E001, m.name.span, format!("`{}` is declared twice", m.name.text));
            }
            seen.push(&m.name.text);
            if m.ty == Ty::Side && m.interface.is_none() {
                say(Code::E103, m.span, format!(
                    "`{}`: a group holds no side; an interface's member is a number, an entity, \
                     a set or a group", m.name.text
                ));
            }
        }
    }
    // every type that names an interface, from the file it is written in
    let mut types: Vec<(&crate::syntax::Name, Option<usize>)> = Vec::new();
    for c in &p.components {
        types.extend(c.formals.iter().filter_map(|f| f.interface.as_ref()).map(|n| (n, c.module)));
    }
    for f in &p.interfaces {
        types.extend(f.members.iter().filter_map(|m| m.interface.as_ref()).map(|n| (n, f.module)));
    }
    for (n, from) in types {
        if let Err(why) = p.resolve_interface(&n.text, from) {
            say(Code::E101, n.span, why);
        }
    }
    // one reaching itself through its members can never be given
    for (i, f) in p.interfaces.iter().enumerate() {
        if reaches(p, i, i, 0) {
            say(Code::E003, f.name.span, format!(
                "`{}` holds itself through its members, so no group could ever be one",
                f.name.text
            ));
        }
    }
    crate::modules::localize(p, &mut said);
    said.sort_by_key(|d| d.span.lo);
    diags.append(&mut said);
}

/// Whether interface `from`'s members reach interface `to`.
fn reaches(p: &Program, from: usize, to: usize, depth: usize) -> bool {
    if depth > MAX_NEST {
        return false;
    }
    let f = &p.interfaces[from];
    f.members.iter().filter_map(|m| m.interface.as_ref()).any(|n| {
        p.resolve_interface(&n.text, f.module)
            .is_ok_and(|j| j == to || reaches(p, j, to, depth + 1))
    })
}
