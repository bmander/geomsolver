//! Gauges and orientation predicates (§9.2, issue #47): `fix` and `ccw`/`cw`, read as relations
//! are and applied, holding numbers or choosing roots, never constraints the sketch holds.

use super::resolve::Resolver;
use super::{Code, Diag};
use crate::constraints::CKind;
use crate::ir::{Relation, ResolvedRelation, Statement as Stmt};
use crate::model::{EntKind, Sketch};
use crate::syntax::{Arg, Ref, RelationForm, Span, StmtId};
use crate::decompose;
use std::collections::BTreeMap;

/// `fix` states every number it holds, pinned whole or by member (§9.2: inside the parentheses
/// `==` pins and `:` selects): `fix((0, 0)) p`, `fix(x == 0) p`, `fix(dir == (1, 0, 0)) t`.  A
/// bare `fix p`, a held number without its name and one written as a selector are each refused
/// where they stand, with the spelling.
pub(super) fn fix_spelling(w: &crate::syntax::Written) -> Result<(), (Span, String)> {
    use crate::syntax::OpArg;
    let of = w.ops.first().map_or("p".to_string(), |r| r.root.text.clone());
    let refused = |key: &str| format!("`fix` holds a point itself, `fix((0, 0)) {of}`, or what an \
        entity has — `x`, `r`, `half`, `dir`, `origin`, or one component, `dir.x` — not `{key}`");
    for a in &w.args {
        match a {
            OpArg::Named(key, _) => {
                let m = format!("`fix` pins a number with `==`: `fix({} == …) {of}`", key.text);
                return Err((key.span, m))
            }
            OpArg::Dim(text, span) => {
                return Err((*span, format!("`fix` names each number it holds: \
                    `fix(r == {text}) {of}`, and a point's place is a vector, `fix((0, 0)) {of}`")))
            }
            OpArg::Slot { key, .. } => {
                if !CKind::Fix.spec()[1..].iter().any(|(n, _)| *n == key.text) {
                    return Err((key.span, refused(&key.text)))
                }
            }
            OpArg::Vector { key: Some(key), .. } => {
                if !matches!(key.text.as_str(), "dir" | "origin") {
                    return Err((key.span, refused(&key.text)))
                }
            }
            _ => {}
        }
    }
    if !w.args.iter().any(|a| matches!(a, OpArg::Slot { .. } | OpArg::Vector { .. })) {
        return Err((w.word.span, format!("`fix` states the numbers it holds: `fix((0, 0)) {of}`")));
    }
    Ok(())
}

/// Whether a statement is a `fix`, which is applied before the seeds that read geometry are
/// worked out (`program::build`), so a seed reading a held point reads where it is held.
pub(super) fn is_fix(r: &Relation) -> bool {
    match &r.form {
        RelationForm::Written(w) => w.word.text == "fix",
        RelationForm::Canonical { kind, .. } => *kind == CKind::Fix,
    }
}

/// A gauge or an orientation predicate, **applied** (issue #47, item 5): written and settled as
/// every other relation — an operator, a class, a placement — but holding parameters or
/// recording a root choice instead of becoming a constraint the sketch holds, so `constrain`
/// returns no id for it and the map never knows it.  The checks are the ones the two statement
/// kinds always made, in their words.  What it holds or chooses goes into `gauges`, applied once
/// every gauge is in (`Gauges::hold_all`, `choose_all`).
pub(super) fn apply_gauge(
    sk: &mut Sketch,
    res: &Resolver,
    r: &ResolvedRelation<'_>,
    st: &Stmt,
    gauges: &mut Gauges,
    diags: &mut Vec<Diag>,
) {
    let refs: Vec<&Ref> = r
        .args
        .iter()
        .filter_map(|a| match a {
            Some(Arg::Ref(rf)) => Some(rf),
            _ => None,
        })
        .collect();
    let mut bad = |code: Code, span: Span, message: String| {
        diags.push(Diag::within(code, span, st.id, message))
    };
    match r.kind {
        CKind::Fix => {
            let Some(rf) = refs.first().copied() else {
                bad(Code::E103, st.span, "`fix` names what it holds".to_string());
                return;
            };
            let Some(e) = res.entity(sk, rf) else {
                bad(Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text));
                return;
            };
            // a plane holds where it stands; an axis its direction and its origin
            let members = e.kind.members();
            let spec = r.kind.spec();
            let own = sk.own_params(e);
            let owned = &members[..own.len().min(members.len())];
            let kind = e.kind.as_str();
            let article = if kind.starts_with(['a', 'e', 'i', 'o', 'u']) { "an" } else { "a" };
            // a vector held whole is as long as the vector it holds: a point in a plane is two
            // numbers and one in space three, so `(0, 0)` cannot leave a height free unsaid
            for a in r.written.map_or(&[][..], |w| &w.args[..]) {
                let crate::syntax::OpArg::Vector { key, parts, span } = a else { continue };
                let k = key.as_ref().map_or("", |k| k.text.as_str());
                let Some(&(_, first)) = e.kind.vectors().iter().find(|(v, _)| *v == k) else {
                    let m = match k {
                        _ if owned.is_empty() => format!("{article} {kind} has no number of its \
                                                          own to fix"),
                        "" => format!("{article} {kind} is no vector: it has {}", said(e.kind, owned)),
                        _ => format!("{article} {kind} has {}, not `{k}`", said(e.kind, owned)),
                    };
                    bad(Code::E105, *span, m);
                    return;
                };
                let n = owned.len().saturating_sub(first).min(3);
                if parts.len() != n {
                    let m = match (k, n) {
                        ("", 2) => "a point in a plane has two coordinates: `(x, y)`".to_string(),
                        ("", _) => "a point in space has three coordinates: `(x, y, z)`".to_string(),
                        _ => format!("`{k}` has three components: `{k} == (x, y, z)`"),
                    };
                    bad(Code::E105, *span, m);
                    return;
                }
            }
            for (i, a) in r.args.iter().enumerate().skip(1) {
                let Some(a) = a else { continue };
                let field = spec[i].0;
                let Some(at) = owned.iter().position(|&n| n == field) else {
                    bad(
                        Code::E105,
                        st.span,
                        if owned.is_empty() {
                            format!("{article} {kind} has no number of its own to fix")
                        } else {
                            format!("{article} {kind} has {}, not `{field}`", said(e.kind, owned))
                        },
                    );
                    continue;
                };
                // a `fix` holds a number, and an unknown is not one
                if let Arg::Tie { span, .. } = a {
                    let m = format!("`fix` holds a number, and `{field}` is pinned to an unknown");
                    bad(Code::E040, *span, m);
                    continue;
                }
                // what the flattener settled the pin to; an expression it could not work out was
                // reported there
                let Arg::Seed { value, .. } = a else { continue };
                let v = *value;
                // where an axis is (its numbers after the direction's three), held: placed first,
                // so no later relation reading its place frees what this holds
                if e.kind == EntKind::Axis && at >= 3 {
                    sk.place_axis(e.i());
                }
                gauges.hold(own[at], v, field, crate::syntax::ref_text(rf), st);
            }
        }
        CKind::Ccw | CKind::Cw => {
            if refs.len() != 3 {
                bad(Code::E103, st.span, "an orientation names three points".to_string());
                return;
            }
            let mut pts = [0usize; 3];
            for (i, rf) in refs.iter().enumerate() {
                match res.entity(sk, rf) {
                    Some(e) if e.kind == EntKind::Point => pts[i] = e.i(),
                    _ => {
                        bad(Code::E101, rf.span, format!("no such point: `{}`", rf.root.text));
                        return;
                    }
                }
            }
            // canonical, so the choice the document states and the one the plan replays are one
            // record and not two that never meet (issue #48, item 4)
            let (key, v) = decompose::branch_record(pts, r.kind == CKind::Ccw);
            let names: Vec<String> = refs.iter().map(|rf| crate::syntax::ref_text(rf)).collect();
            gauges.orient(key, v, format!("the triangle `{}`", names.join("`, `")), st);
        }
        _ => unreachable!("{:?} is not a gauge", r.kind),
    }
}

/// What an entity has to hold, as the source names it: a point's coordinates (`x and y`), and
/// for anything else its vectors whole (`dir and origin`) beside its scalars (`r`).
fn said(kind: EntKind, owned: &[&str]) -> String {
    let mut names: Vec<&str> = Vec::new();
    for m in owned {
        let head = m.split_once('.').map_or(*m, |(v, _)| v);
        if !names.contains(&head) {
            names.push(head);
        }
    }
    if kind == EntKind::Point && names.len() == 3 {
        return "x, y and z".to_string();
    }
    names.join(" and ")
}

/// Every number the `fix`es hold and every root the orientations choose, gathered before any is
/// applied (issue #113).  A gauge adds no row, so the diagnosis never sees two of them disagree,
/// and applied as each was read the later would silently win: an outcome hanging on the order
/// of statements (P2).  So each is applied once all are in — those that agree as the one gauge
/// they are (copies of a block holding a shared point alike), and those that disagree not at
/// all, **E031** at every statement taking part.
#[derive(Default)]
pub(super) struct Gauges {
    /// each held parameter: its value, then where, per statement holding it, with the field and
    /// the entity as that statement names them
    holds: BTreeMap<u32, Vec<(f64, Span, StmtId, &'static str, String)>>,
    /// each canonical triangle's choices (`decompose::branch_record`), and a raw key's
    branches: BTreeMap<String, Vec<(i32, String, Span, StmtId)>>,
}

impl Gauges {
    fn hold(&mut self, param: u32, v: f64, field: &'static str, of: String, st: &Stmt) {
        self.holds.entry(param).or_default().push((v, st.span, st.id, field, of));
    }

    fn orient(&mut self, key: String, v: i32, what: String, st: &Stmt) {
        self.branches.entry(key).or_default().push((v, what, st.span, st.id));
    }

    /// A `branch(KEY, ±1)` statement: a root choice under a key no triple of points spells.
    pub(super) fn branch(&mut self, b: &crate::syntax::Branch, st: &Stmt) {
        self.orient(b.key.clone(), b.value, format!("the root choice `{}`", b.key), st);
    }

    /// Hold every number its holds agree on.  Values a rounding apart are one hold — two
    /// spellings of one number (`w / 3 * 3`, `w`) — held at the least, whichever came first.
    /// A statement holding several numbers in dispute is told once, naming each.
    pub(super) fn hold_all(&mut self, sk: &mut Sketch, diags: &mut Vec<Diag>) {
        let mut disputed: BTreeMap<(Span, StmtId), (Vec<&'static str>, String)> = BTreeMap::new();
        for (param, holds) in std::mem::take(&mut self.holds) {
            let lo = holds.iter().map(|h| h.0).fold(f64::INFINITY, f64::min);
            let hi = holds.iter().map(|h| h.0).fold(f64::NEG_INFINITY, f64::max);
            if hi - lo <= 1e-12 * lo.abs().max(hi.abs()).max(1.0) {
                let p = &mut sk.params[param as usize];
                p.value = lo;
                p.fixed = true;
                continue;
            }
            for (_, span, id, field, of) in holds {
                let (fields, _) = disputed.entry((span, id)).or_insert_with(|| (Vec::new(), of));
                if !fields.contains(&field) {
                    fields.push(field);
                }
            }
        }
        for ((span, id), (fields, of)) in disputed {
            let are = if fields.len() == 1 { "is" } else { "are" };
            let fields = fields.iter().map(|f| format!("`{f}`")).collect::<Vec<_>>().join(" and ");
            let message = format!(
                "{fields} of `{of}` {are} held at two values: a number is held once, and another \
                 `fix` holding it says otherwise"
            );
            diags.push(Diag::within(Code::E031, span, id, message));
        }
    }

    /// Record every root choice its statements agree on.
    pub(super) fn choose_all(&mut self, sk: &mut Sketch, diags: &mut Vec<Diag>) {
        for (key, choices) in std::mem::take(&mut self.branches) {
            if choices.iter().all(|c| c.0 == choices[0].0) {
                sk.branches.insert(key, choices[0].0);
                continue;
            }
            let at: BTreeMap<(Span, StmtId), String> =
                choices.into_iter().map(|(_, what, span, id)| ((span, id), what)).collect();
            for ((span, id), what) in at {
                let message = format!(
                    "{what} is oriented both ways: one statement chooses one root, and another \
                     the other"
                );
                diags.push(Diag::within(Code::E031, span, id, message));
            }
        }
    }
}
