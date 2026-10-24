//! **Interfaces** (§8.1 [0.84]): a `group` formal typed by a named shape, `gear: Gear`.
//!
//! A group formal reads whatever its argument carries; one typed by an interface is **a group
//! of its own** under the formal's key, holding only what the interface declares (`Walk::typed`,
//! from `bind`): each entity, set or group member an alias of the argument's, each nested
//! interface a group again, each number copied into the body's `vals`.  A member it does not
//! declare is no name in the body, and nothing handed on carries more.  The call is held to the
//! interface where what it judges is known: the numbers at the call (`interface_numbers`), the
//! rest once every name is known (`conform`, from `resolve`), an entity's kind by the elaborator
//! (`Expansion::kinds`), as a `path` formal's is.  Structural, as the rest of the language is: an
//! instance or a group conforms by having the members, never by saying so.

use super::{typed, Code, Vals, Walk};
use crate::expr::Aff;
use crate::syntax::{Component, Formal, Name, Program, Ref, Seg, Span, Ty};
use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

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
            Member::Number(ty) | Member::Entity(ty) => match ty {
                Ty::Ent(k) => k.a(),
                ty => crate::model::article(ty.word().unwrap_or_default()),
            },
            Member::Set => "a set".to_string(),
            Member::Group(None) => "a group".to_string(),
            Member::Group(Some(s)) => format!("a `{}`", s.name),
        }
    }
}

/// **Every interface of the program, resolved once** (`Walk::new`): nested interfaces shared,
/// and one that reaches itself — refused where it is defined (`check`) — read past the cycle
/// as an untyped group, so each is a finite tree.
pub(super) fn shapes(p: &Program) -> Vec<Rc<Shape>> {
    fn build(p: &Program, i: usize, memo: &mut Vec<Option<Rc<Shape>>>, visiting: &mut BTreeSet<usize>)
        -> Option<Rc<Shape>>
    {
        if let Some(s) = &memo[i] {
            return Some(s.clone());
        }
        if !visiting.insert(i) {
            return None;
        }
        let f = &p.interfaces[i];
        let members = f.members.iter().map(|m| {
            let member = match (&m.interface, m.ty) {
                (Some(n), _) => Member::Group(
                    p.resolve_interface(&n.text, f.module).ok().and_then(|j| build(p, j, memo, visiting)),
                ),
                (None, ty) if ty.number() => Member::Number(ty),
                (None, Ty::Set) => Member::Set,
                (None, Ty::Group | Ty::Side) => Member::Group(None),
                (None, ty) => Member::Entity(ty),
            };
            (m.name.text.clone(), member)
        }).collect();
        visiting.remove(&i);
        let shape = Rc::new(Shape { name: p.interface_id(i), members });
        memo[i] = Some(shape.clone());
        Some(shape)
    }
    let mut memo = vec![None; p.interfaces.len()];
    (0..p.interfaces.len())
        .map(|i| build(p, i, &mut memo, &mut BTreeSet::new()).expect("nothing visiting at the top"))
        .collect()
}

/// `r.m` — the reference a member of what `r` names is reached by, as the call wrote `r`.
fn member_ref(r: &Ref, m: &str) -> Ref {
    let mut r = r.clone();
    r.path.push(Seg::Field(Name { text: m.to_string(), span: r.span }));
    r
}

impl<'a> Walk<'a> {
    /// The interface a component's formal is typed by, resolved from the file it was written in.
    pub(super) fn formal_shape(&self, comp: &Component, f: &Formal) -> Option<Rc<Shape>> {
        let n = f.interface.as_ref()?;
        let i = self.prog.resolve_interface(&n.text, comp.module).ok()?;
        self.shapes.get(i).cloned()
    }

    /// The formal's key (`bar.gear`) made a group of the interface's shape over `r`, the
    /// argument as the call wrote it in `scope`: each entity, set or untyped group member an
    /// alias of `r`'s, each nested interface a group of its own again.
    pub(super) fn typed(&mut self, shape: &Shape, key: &str, r: &Ref, scope: &super::Scope) {
        self.names.insert(key.to_string());
        self.group_names.insert(key.to_string());
        self.typed.insert(key.to_string(), shape.name.clone());
        for (name, member) in &shape.members {
            let (k, m) = (format!("{key}.{name}"), member_ref(r, name));
            match member {
                Member::Number(_) => {}
                Member::Group(Some(inner)) => self.typed(inner, &k, &m, scope),
                _ => self.aliases.push((k, m, scope.clone())),
            }
        }
    }

    /// The numbers an interface asks of `actual` (as written in the caller, `gear_cone`),
    /// each of its member's dimension, copied in under the formal's name (`gear.pitch`) — and
    /// no others.  One that is not there stands in at nothing, said once here.
    pub(super) fn interface_numbers(
        &mut self,
        shape: &Shape,
        actual: &str,
        formal: &str,
        vals: &Vals,
        sub: &mut BTreeMap<String, Aff>,
        span: Span,
    ) {
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
                            shape.name, member.a()
                        ),
                    };
                    self.err(Code::E103, span, why);
                    sub.insert(to, Aff::num(0.0).as_dim(ty.dim()));
                }
                Member::Group(Some(inner)) => {
                    self.interface_numbers(inner, &from, &to, vals, sub, span)
                }
                // a group or a set carries what it carries (`s.r` of a sphere)
                Member::Group(None) | Member::Set => super::copy_members(vals, &from, &to, sub),
                Member::Entity(_) => {}
            }
        }
    }

    /// What an interface asks of the group or instance `target` (absolute, `pair.gear_cone`)
    /// that a number is not: each member present, a set or a group where one is asked for, and
    /// an entity's kind left to the elaborator.  `written` is the argument as the call wrote it,
    /// with the path to the member reached so far.
    pub(super) fn conform(
        &mut self,
        shape: &Shape,
        target: &str,
        written: &str,
        alias: &BTreeMap<String, String>,
        span: Span,
    ) {
        for (name, member) in &shape.members {
            if matches!(member, Member::Number(_)) {
                continue; // `interface_numbers`, at the call
            }
            let key = format!("{target}.{name}");
            let here = format!("{written}.{name}");
            let present = self.names.contains(&key)
                || alias.contains_key(&key)
                || self.sets.made.contains_key(&key)
                || self.group_names.contains(&key);
            if !present {
                self.err(Code::E103, span, format!(
                    "`{written}` has no `{name}`, which `{}` asks for ({})",
                    shape.name, member.a()
                ));
                continue;
            }
            let to = alias.get(&key).cloned().unwrap_or(key);
            let set = self.sets.made.contains_key(&to);
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
                continue;
            }
            match member {
                Member::Group(Some(inner)) => self.conform(inner, &to, &here, alias, span),
                Member::Entity(ty) => self.checks.kinds.push(super::KindAsked {
                    target: to,
                    span,
                    ty: *ty,
                    what: here,
                    interface: Some(shape.name.clone()),
                }),
                _ => {}
            }
        }
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
        said.push(Diag::new(code, span, None, message));
    };
    let mut first: BTreeMap<(Option<usize>, &str), Span> = BTreeMap::new();
    for f in &p.interfaces {
        if let Some(&was) = first.get(&(f.module, f.name.text.as_str())) {
            let again = crate::program::defined_again(p, &f.name, was);
            say(again.code, again.span, again.message);
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
        if reaches(p, i, i, &mut BTreeSet::new()) {
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

/// Whether interface `from`'s members reach interface `to`, each interface visited once.
fn reaches(p: &Program, from: usize, to: usize, seen: &mut BTreeSet<usize>) -> bool {
    let f = &p.interfaces[from];
    f.members.iter().filter_map(|m| m.interface.as_ref()).any(|n| {
        p.resolve_interface(&n.text, f.module)
            .is_ok_and(|j| j == to || (seen.insert(j) && reaches(p, j, to, seen)))
    })
}
