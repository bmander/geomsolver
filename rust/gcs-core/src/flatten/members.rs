//! **An instance's numbers read from outside it** (`shield.top`, `uno.jack_y`): a number its
//! body defines, worked out from what its call gives it where the call is written — among the
//! definitions of the body writing the call (`Walk::params`), so a definition there, a dimension
//! or another call reads it as it reads `ball.r`, and its formula is written once, in the
//! component.
//!
//! The body is not walked for it: its numbers are worked out on demand, each over the numbers its
//! call gives its formals (`Walk::call_numbers`), the constants of the file it is written in
//! (`Walk::file_numbers`) and the body's own numbers it reads in turn, a number of an instance
//! it writes among them.  Nothing is said of the body here: its walk says what is wrong with it,
//! where it is written.

use super::*;
use super::values::{is_reference, members_of, value_aff};
use crate::syntax::{InstVal, Instance};
use std::rc::Rc;

/// The constants of the file a body is written in, and which file that is: the one its calls
/// name their components in.
#[derive(Clone)]
pub(super) struct Constants {
    pub module: Option<usize>,
    pub table: Rc<Vals>,
}

/// One body read from outside: its numbers worked out on demand, over its call's.
struct Read<'c> {
    /// The instance's key (`shield.`), under which an unknown its call leaves is named.
    key: String,
    table: Vals,
    defs: BTreeMap<String, (String, Option<Ty>)>,
    calls: BTreeMap<&'c str, &'c Instance>,
    constants: Constants,
    /// The numbers being worked out, which one reading itself reaches again.
    busy: BTreeSet<String>,
}

/// **The numbers a body defines** that are read from outside it, by name: each `name := …` and
/// `param` with a value — the text after `:=` and the type it declared — and each number of a
/// group it writes (`dims.w`).  A `when`'s are its own, and a parameter axis runs.
pub(super) fn defined(comp: &Component) -> BTreeMap<String, (String, Option<Ty>)> {
    let mut out = BTreeMap::new();
    for st in &comp.body {
        match &st.kind {
            StmtKind::Param(pd) if pd.bound() && pd.over().is_none() => {
                let ty = pd.input.as_ref().and_then(|i| i.ty);
                out.entry(pd.name.text.clone()).or_insert((pd.text.clone(), ty));
            }
            StmtKind::Group(g) => {
                let mut members = Vec::new();
                members_of(&g.name.text, &g.fields, &mut members);
                for (name, _, field) in members {
                    let text = match &field.value {
                        InstVal::Expr(t) => t.clone(),
                        InstVal::Ref(r) => crate::syntax::ref_text(r),
                        _ => continue,
                    };
                    out.entry(name).or_insert((text, None));
                }
            }
            _ => {}
        }
    }
    out
}

/// **How much of a path into an instance of `comp` names one of its numbers** (`top` of
/// `shield.top`, `base.top` of `st.base.top`), by the longest prefix that does — a numeric formal
/// of its component, a number its body defines, or, through an instance its body writes, one of
/// that instance's — and whether that number's text is a name and nothing else (`alias := sizes`),
/// which may name a group rather than a number.  `None` where it names none (geometry, or
/// nothing), and where the path goes on past it: a number has no members.
pub(super) fn offers(prog: &Program, comp: &Component, parts: &[&str], depth: usize)
    -> Option<(usize, bool)> {
    let defs = defined(comp);
    let formal = |m: &str| comp.formals.iter().any(|f| f.ty.number() && f.name.text == m);
    let number = (1..=parts.len()).rev().map(|k| (k, parts[..k].join(".")))
        .find(|(_, m)| defs.contains_key(m) || formal(m));
    if let Some((k, m)) = number {
        let named = defs.get(&m).is_some_and(|(t, _)| is_reference(t));
        return (k == parts.len()).then_some((k, named));
    }
    let (head, rest) = parts.split_first()?;
    if rest.is_empty() || depth >= MAX_DEPTH {
        return None;
    }
    let inst = calls(&comp.body).get(head).copied()?;
    let k = prog.resolve_component(&inst.component.text, comp.module).ok()?;
    offers(prog, &prog.components[k], rest, depth + 1).map(|(k, named)| (k + 1, named))
}

/// The texts a call writes for its arguments, a group's written in place among them.
pub(super) fn texts(args: &[crate::syntax::InstArg], out: &mut Vec<String>) {
    for a in args {
        match &a.value {
            InstVal::Expr(t) | InstVal::Hint(t, _) => out.push(t.clone()),
            InstVal::Ref(r) => out.push(crate::syntax::ref_text(r)),
            InstVal::Group(fields) => texts(fields, out),
        }
    }
}

/// The instances a body writes, by name: the calls whose numbers a reader may name.
fn calls(body: &[Stmt]) -> BTreeMap<&str, &Instance> {
    body.iter().filter_map(|st| match &st.kind {
        StmtKind::Instance(i) if !i.name.text.starts_with('#') => Some((i.name.text.as_str(), i)),
        _ => None,
    }).collect()
}

impl Walk<'_> {
    /// **`member` of the instance `inst`**, a call written in a body whose numbers are `vals`
    /// (`scope`'s): the number its call gives the formal of that name, or the number its body
    /// defines by it, worked out from the call — or why it cannot be.
    pub(super) fn instance_value(
        &mut self,
        inst: &Instance,
        member: &str,
        vals: &Vals,
        scope: &Scope,
    ) -> Result<Aff, String> {
        // a body of the file being worked out reads the constants worked out so far, which a
        // definition reading it waits for (`call_reads`); any other file's are all known
        let table = if scope.closed {
            self.file_numbers(scope.module).0
        } else {
            let mut own = vals.under("");
            own.retain(|_, v| v.free.is_none());
            Rc::new(Vals::of(own))
        };
        let constants = Constants { module: scope.module, table };
        self.member_of(inst, member, vals, &constants, scope.prefix(), 0)
    }

    /// `instance_value`, for a call written under `prefix` in the file whose constants are
    /// `constants`: one level of what may be a body read from inside another read so.
    fn member_of(
        &mut self,
        inst: &Instance,
        member: &str,
        vals: &Vals,
        constants: &Constants,
        prefix: &str,
        depth: usize,
    ) -> Result<Aff, String> {
        if depth >= MAX_DEPTH {
            return Err(format!("`{member}` is read through instances nested too deep"));
        }
        let k = self.prog.resolve_component(&inst.component.text, constants.module)?;
        let comp = &self.prog.components[k];
        let (given, failed) = self.call_numbers(comp, inst, vals, prefix, true);
        if let Some(e) = failed.into_iter().next() {
            return Err(e);
        }
        if comp.formals.iter().any(|f| f.ty.number() && f.name.text == member) {
            return given.get(member).cloned()
                .ok_or_else(|| format!("`{member}` is given nothing"));
        }
        let constants = match comp.module == constants.module {
            true => constants.clone(),
            false => Constants { module: comp.module, table: self.file_numbers(comp.module).0 },
        };
        let mut read = Read {
            key: format!("{prefix}{}.", inst.name.text),
            table: Vals::over_with(constants.table.clone(), given),
            defs: defined(comp),
            calls: calls(&comp.body),
            constants,
            busy: BTreeSet::new(),
        };
        self.worked(&mut read, member, depth)
    }

    /// One number of a body read from outside, and first each of its own it reads.
    fn worked(&mut self, read: &mut Read, name: &str, depth: usize) -> Result<Aff, String> {
        // the body's own, which hide the file's constants of their names
        if let Some(a) = read.table.own().get(name) {
            return Ok(a.clone());
        }
        if !read.busy.insert(name.to_string()) {
            return Err(format!("`{name}` is defined in terms of itself"));
        }
        let a = match read.defs.get(name).cloned() {
            Some((text, ty)) => {
                self.ready(read, &text, depth)?;
                let a = value_aff(&text, &read.table, self.units)?;
                match ty {
                    Some(ty) => typed(a, ty, name)?,
                    None => a,
                }
            }
            // a number of an instance the body writes, read through it in turn, once what its
            // call reads of the body is
            None => {
                let inner = name.split_once('.')
                    .and_then(|(head, rest)| Some((*read.calls.get(head)?, rest)));
                let Some((inst, rest)) = inner else {
                    return Err(format!("`{name}` is not a number here — nothing in scope gives \
                                        it one"));
                };
                let mut given = Vec::new();
                texts(&inst.args, &mut given);
                for text in given {
                    self.ready(read, &text, depth)?;
                }
                let key = read.key.clone();
                self.member_of(inst, rest, &read.table, &read.constants, &key, depth + 1)?
            }
        };
        read.busy.remove(name);
        read.table.insert(name.to_string(), a.clone());
        Ok(a)
    }

    /// Each number of the body a text reads, worked out first: one it defines, or one of an
    /// instance it writes, or each of a group it writes and the text names whole (`Part(dims)`);
    /// and one read through a name it gives a group (`d := s`), the group's.
    fn ready(&mut self, read: &mut Read, text: &str, depth: usize) -> Result<(), String> {
        let deps = expr::parse_in(text.trim(), self.units).map(|p| p.body.deps())
            .unwrap_or_default();
        for dep in deps {
            let head = dep.split_once('.').map(|(h, _)| h);
            if read.defs.contains_key(&dep) || head.is_some_and(|h| read.calls.contains_key(h)) {
                self.worked(read, &dep, depth)?;
                continue;
            }
            if read.table.has(&dep) {
                continue;
            }
            let under = format!("{dep}.");
            let members: Vec<String> =
                read.defs.keys().filter(|k| k.starts_with(&under)).cloned().collect();
            for m in &members {
                self.worked(read, m, depth)?;
            }
            let Some((head, rest)) = dep.split_once('.') else { continue };
            let Some((to, _)) = read.defs.get(head).filter(|(t, _)| is_reference(t)) else {
                continue;
            };
            let through = format!("{}.{rest}", to.trim());
            let a = match read.table.get(&through) {
                Some(a) => a.clone(),
                None if read.defs.contains_key(&through) => self.worked(read, &through, depth)?,
                None => continue,
            };
            read.table.insert(dep, a);
        }
        Ok(())
    }
}
