//! **A set that says a sweep is read as that sweep** (#737, #744): `solid(S)` is the points inside
//! the set `S`, and where `S`'s body says what a prism's, a drafted prism's or a revolution's
//! says — read off its statements and their trees, never off a word's name or a statement's
//! printed text — the solid is written out as the sweep of the same points (`syntax::Sweep`), so
//! every reader of a solid's text before it is built (a prism's sides for an envelope, a
//! revolution's named side, the carriers a sketch on a face stands on, the gestures that resize
//! it) reads it as it reads one, and the elaborator builds it as one.  The set is a family's
//! instance, its formals bound to what the call gives them (`binding`), or a set written by hand,
//! its names what they name where the solid is (`Bound::Own`): either is read alike.  A set this
//! does not read stays a set, which the elaborator's `solid::defined` may still recognise from its
//! terms.
//!
//! | the body (`p` its point, `F` a face) | written as |
//! |---|---|
//! | `p inside F`, `p distance(>= a, along: n) F`, `p distance(<= b, along: n) F` | `Sweep::Prism { from: a, to: b }` |
//! | the two ordinates, `x := { q \| q outside F }`, `y := { q \| q inside F }`, `distance(p, x) - distance(p, y) >= tan(θ) * distance(p, F)` | `Sweep::Prism { …, draft: θ }` |
//! | `p coincident(under: l, over t in (a, b)) F`, `l` a line, one end nothing | `Sweep::Revolve { axis: l, … }` |
//! | `p inside F` alone | `Sweep::Through`, through the body it is cut from |
use crate::syntax::{Arg, Cmp, Component, InstVal, Instance, Measuring, OpArg, Ref, SetLit,
                    Span, StmtKind, Sweep, Written};

/// What a name of the body is bound to: a reference, or a number's text as written.
#[derive(Clone, Debug)]
pub(crate) enum Bound {
    /// A reference, and the kind of entity the formal it is given for is — a face, a line —
    /// which only a formal's type says before the elaborator does.
    Ref(Ref, Option<crate::model::EntKind>),
    /// The name itself, an entity of this kind where the set is read: a set written by hand
    /// names the drawing's faces and lines directly.
    Own(crate::model::EntKind),
    Text(String, Span),
}

/// The binding of a set family's formals to an instance's arguments: by label, else by place,
/// else the formal's default.
pub(crate) fn binding<'a>(comp: &'a Component, inst: &'a Instance)
    -> impl Fn(&str) -> Option<Bound> + 'a {
    let formals = &comp.formals;
    let positional: Vec<_> = inst.args.iter().filter(|a| a.label.is_none()).collect();
    move |name: &str| {
        let k = formals.iter().position(|f| f.name.text == name)?;
        let given = inst.args.iter().find(|a| a.label.as_ref().is_some_and(|l| l.text == name))
            .or_else(|| positional.get(k).copied());
        let Some(a) = given else {
            return formals[k].default.clone().map(|(t, s)| Bound::Text(t, s));
        };
        match &a.value {
            InstVal::Ref(r) => {
                let kind = match formals[k].ty {
                    crate::syntax::Ty::Ent(e) => Some(e),
                    _ => None,
                };
                Some(Bound::Ref(r.clone(), kind))
            }
            InstVal::Expr(t) => Some(Bound::Text(t.clone(), a.span)),
            _ => None,
        }
    }
}

/// The sweep `lit` says and the face it sweeps, its names bound by `bind` (a name no formal is
/// is itself) — `None` where it says none of the table's.
pub(crate) fn said(lit: &SetLit, bind: &dyn Fn(&str) -> Option<Bound>) -> Option<(Sweep, Ref)> {
    let p = lit.bound.text.as_str();
    // a face a formal of that type is given, or one a set written by hand names: a name the
    // walk knows no kind of (a plane, a solid, a set) is the elaborator's to tell apart
    // (`solid::defined` reads those)
    use crate::model::EntKind;
    let typed = |r: &Ref, kinds: &[EntKind]| -> Option<Ref> {
        match bind(&r.root.text) {
            Some(Bound::Ref(a, Some(k))) if r.path.is_empty() && kinds.contains(&k) => Some(a),
            Some(Bound::Own(k)) if r.path.is_empty() && kinds.contains(&k) => Some(r.clone()),
            _ => None,
        }
    };
    let face = |r: &Ref| typed(r, &[EntKind::Face]);
    // and a line a revolution turns about
    let line = |r: &Ref| typed(r, &[EntKind::Line, EntKind::Axis]);
    // a number given as a name is that name's text where the call is written; any other number
    // is its own text there, so one reading a formal under arithmetic (`-t`, `2 * t`) is no
    // number of the call's, and the set is left to the elaborator, which reads it in its scope
    let number = |text: &str, span: Span| -> Option<Arg> {
        let text = text.trim();
        Some(match bind(text) {
            Some(Bound::Text(t, s)) => Arg::Dim { text: t, span: s },
            Some(Bound::Ref(r, _)) => Arg::Dim { text: crate::syntax::ref_text(&r), span: r.span },
            Some(Bound::Own(_)) | None => {
                let read = crate::expr::parse(text).ok()?.body.deps();
                if read.iter().any(|n| matches!(bind(n), Some(Bound::Ref(..) | Bound::Text(..)))) {
                    return None;
                }
                Arg::Dim { text: text.to_string(), span }
            }
        })
    };
    let of_p = |w: &Written| w.ops.first().is_some_and(|o| o.root.text == p && o.path.is_empty());
    let mut inside = None;
    let (mut lo, mut hi) = (None, None);
    let mut sets: Vec<(String, &SetLit)> = Vec::new();
    let mut lean = None;
    let mut turn = None;
    for st in &lit.body {
        match &st.kind {
            StmtKind::Relation(r) => {
                let w = r.form.written()?;
                if !of_p(w) || w.ops.len() != 2 {
                    return None;
                }
                let f = face(&w.ops[1])?;
                // within the face, as written (`p inside F`) or lowered (`p coincident(<=) F`)
                if w.side() == Some((Cmp::Le, None)) {
                    if inside.replace(f).is_some() {
                        return None;
                    }
                    continue;
                }
                match (w.word.text.as_str(), w.args.as_slice()) {
                    // an ordinate along the face's normal, bounded one way
                    ("distance", args) => {
                        let along_n = w.sel("along").as_deref() == Some("n");
                        let (Some((cmp, None)), Some((t, s))) = (w.bound(), w.dim()) else {
                            return None;
                        };
                        if !along_n || w.grow().is_some() || args.len() != 3 {
                            return None;
                        }
                        let slot = match cmp { Cmp::Ge => &mut lo, Cmp::Le => &mut hi, Cmp::In => return None };
                        if slot.replace((f, number(t, s)?)).is_some() {
                            return None;
                        }
                    }
                    ("coincident", args) if turn.is_none() => {
                        let mut under = None;
                        let mut over = None;
                        for a in args {
                            match a {
                                OpArg::Named(n, Arg::Ref(r)) if n.text == "under" => under = Some(r.clone()),
                                OpArg::Over { key, ends: Some([a, b]) } if key.text == "t" => {
                                    over = Some((number(&a.0, a.1)?, number(&b.0, b.1)?));
                                }
                                _ => return None,
                            }
                        }
                        let axis = line(&under?)?;
                        turn = Some((f, axis, over?));
                    }
                    _ => return None,
                }
            }
            StmtKind::Set(s) => sets.push((s.name.text.clone(), &s.lit)),
            StmtKind::Measured(m) if lean.is_none() => lean = Some(m),
            _ => return None,
        }
    }
    if let Some((f, axis, (from, to))) = turn {
        if inside.is_some() || lo.is_some() || hi.is_some() || !sets.is_empty() || lean.is_some() {
            return None;
        }
        return crate::syntax::revolve_between(axis, &from, &to).map(|s| (s, f));
    }
    // a section unbounded along its normal: the prism through the body it is cut from, joined
    // where the body word joins it (`syntax::ROLLED`)
    if let (Some(f), None, None, true, None) = (&inside, &lo, &hi, sets.is_empty(), lean) {
        return Some((Sweep::Through { body: Ref::new(crate::syntax::ROLLED) }, f.clone()));
    }
    let ((fa, from), (fb, to)) = (lo?, hi?);
    if fa.root.text != fb.root.text {
        return None;
    }
    // undrafted: the section is the set's own; drafted: what the lean says
    let draft = match (inside, lean) {
        (Some(f), None) if f.root.text == fa.root.text && sets.is_empty() => None,
        (None, Some(m)) => Some(lean_of(m, &sets, p, &fa, &face, &number)?),
        _ => return None,
    };
    Some((Sweep::Prism { from, to, draft }, fa))
}

/// The draft a lean states, `distance(p, x) - distance(p, y) >= tan(θ) * distance(p, F)` over `x
/// := { q | q outside F }` and `y := { q | q inside F }`, `F` the face `on`, read off the bound's
/// tree: `θ`, bound.
fn lean_of(m: &crate::syntax::Measured, sets: &[(String, &SetLit)], p: &str, on: &Ref,
           face: &dyn Fn(&Ref) -> Option<Ref>, number: &dyn Fn(&str, Span) -> Option<Arg>)
    -> Option<Arg> {
    use crate::expr::Op;
    // the face a set's one statement puts its point within or without
    let side = |lit: &SetLit| -> Option<(Cmp, Ref)> {
        let [st] = lit.body.as_slice() else { return None };
        let w = match &st.kind { StmtKind::Relation(r) => r.form.written()?, _ => return None };
        let of_q = w.ops.first().is_some_and(|o| o.root.text == lit.bound.text && o.path.is_empty());
        match w.side()? {
            (cmp @ (Cmp::Le | Cmp::Ge), None) if of_q => Some((cmp, face(&w.ops[1])?)),
            _ => None,
        }
    };
    let [(a, x), (b, y)] = sets else { return None };
    let ((ca, fa), (cb, fb)) = (side(x)?, side(y)?);
    let (out, into) = match (ca, cb) {
        (Cmp::Ge, Cmp::Le) => (a, b),
        (Cmp::Le, Cmp::Ge) => (b, a),
        _ => return None,
    };
    let Measuring::Bin(Op::Sub, beyond, within) = &m.lhs else { return None };
    if m.cmp != Cmp::Ge || distance(beyond, p)?.root.text != *out
        || distance(within, p)?.root.text != *into {
        return None;
    }
    // `tan(θ) * distance(p, F)`, either way round
    let Measuring::Bin(Op::Mul, l, r) = &m.rhs else { return None };
    let (text, span, plane) = match (l.as_ref(), r.as_ref()) {
        (Measuring::Number { text, span }, d) | (d, Measuring::Number { text, span }) => {
            (text, *span, distance(d, p)?)
        }
        _ => return None,
    };
    let f = face(plane)?;
    if [&fa, &fb, &f].iter().any(|g| g.root.text != on.root.text) {
        return None;
    }
    number(&tangent_of(text)?, span)
}

/// `distance(p, s)`, `s` named bare: what it is measured to.
fn distance<'m>(t: &'m Measuring, p: &str) -> Option<&'m Ref> {
    use crate::expr::Measure;
    let Measuring::Measure(d) = t else { return None };
    let [from, to] = d.args.as_slice() else { return None };
    let bare = |r: &Ref| r.path.is_empty();
    (d.measure == Measure::Distance && d.at.is_none() && bare(from) && bare(to)
        && from.root.text == p).then_some(to)
}

/// `θ` as written, where `text` is the one call `tan(θ)` — read off the expression's tree, its
/// argument the text within the call's parentheses.
fn tangent_of(text: &str) -> Option<String> {
    let crate::expr::Ast::Call(name, args) = crate::expr::parse(text).ok()?.body else {
        return None;
    };
    if name != "tan" || args.len() != 1 {
        return None;
    }
    let t = text.trim();
    let inner = t.strip_prefix("tan")?.trim_start().strip_prefix('(')?.strip_suffix(')')?;
    Some(inner.trim().to_string())
}
