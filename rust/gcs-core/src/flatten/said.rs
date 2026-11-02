//! **A set that says a sweep is read as that sweep** (#737): `solid(S)` is the points inside the
//! set `S`, and where `S`'s body says what a prism's, a drafted prism's or a revolution's says —
//! read off its statements, its formals bound to what the call gives them, never off the word's
//! name — the solid is written out as the sweep of the same points (`syntax::Sweep`), so every
//! reader of a solid's text before it is built (a prism's sides for an envelope, a revolution's
//! named side, the carriers a sketch on a face stands on, the gestures that resize it) reads it as
//! it reads one, and the elaborator builds it as one.  A set this does not read stays a set, which
//! the elaborator's `solid::defined` may still recognise from its terms.
//!
//! | the body (`p` its point, `F` a face) | written as |
//! |---|---|
//! | `p inside F`, `p distance(>= a, along: n) F`, `p distance(<= b, along: n) F` | `Sweep::Prism { from: a, to: b }` |
//! | the two ordinates, `x := { q \| q outside F }`, `y := { q \| q inside F }`, `distance(p, x) - distance(p, y) >= tan(θ) * distance(p, F)` | `Sweep::Prism { …, draft: θ }` |
//! | `p coincident(under: l, over t in (a, b)) F`, `l` a line | `Sweep::Revolve { axis: l, from: a, to: b }`, unless both ends evaluate and neither is nothing |
//! | `p inside F` alone | `Sweep::Through`, through the body it is cut from |
use crate::syntax::{Arg, Cmp, Component, InstVal, Instance, OpArg, Ref, SetLit, Span,
                    StmtKind, Sweep, Written};

/// What a name of the body is bound to: a reference, or a number's text as written.
#[derive(Clone, Debug)]
pub(super) enum Bound {
    /// A reference, and the kind of entity the formal it is given for is — a face, a line —
    /// which only a formal's type says before the elaborator does.
    Ref(Ref, Option<crate::model::EntKind>),
    Text(String, Span),
}

/// The binding of a set family's formals to an instance's arguments: by label, else by place,
/// else the formal's default.
pub(super) fn binding<'a>(comp: &'a Component, inst: &'a Instance)
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
pub(super) fn said(lit: &SetLit, bind: &dyn Fn(&str) -> Option<Bound>) -> Option<(Sweep, Ref)> {
    let p = lit.bound.text.as_str();
    // a face a formal of that type is given: a set written by hand names a face, a plane or a
    // solid alike, which the elaborator tells apart (`solid::defined` reads those)
    use crate::model::EntKind;
    let typed = |r: &Ref, kinds: &[EntKind]| -> Option<Ref> {
        match bind(&r.root.text) {
            Some(Bound::Ref(a, Some(k))) if r.path.is_empty() && kinds.contains(&k) => Some(a),
            _ => None,
        }
    };
    let face = |r: &Ref| typed(r, &[EntKind::Face]);
    // and a line a revolution turns about
    let line = |r: &Ref| typed(r, &[EntKind::Line, EntKind::Axis]);
    // a number given as a name is that name's text where the call is written
    let number = |text: &str, span: Span| -> Arg {
        match bind(text.trim()) {
            Some(Bound::Text(t, s)) => Arg::Dim { text: t, span: s },
            Some(Bound::Ref(r, _)) => Arg::Dim { text: crate::syntax::ref_text(&r), span: r.span },
            None => Arg::Dim { text: text.trim().to_string(), span },
        }
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
                match (w.word.text.as_str(), w.args.as_slice()) {
                    ("inside", []) if inside.is_none() => inside = Some(f),
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
                        if slot.replace((f, number(t, s))).is_some() {
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
                                    over = Some((number(&a.0, a.1), number(&b.0, b.1)));
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
        // the interval as written: which end is nothing is a number's to say, where the ends are
        // evaluated (`Walk::off_its_face`, `program::solids::build`)
        return Some((Sweep::Revolve { axis, from: Some(from), to: Some(to) }, f));
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
        (None, Some(m)) => Some(lean_of(m, &sets, p, &number)?),
        _ => return None,
    };
    Some((Sweep::Prism { from, to, draft }, fa))
}

/// The draft a lean states, `distance(p, x) - distance(p, y) >= tan(θ) * distance(p, F)` over `x
/// := { q | q outside F }` and `y := { q | q inside F }`: `θ`, bound.
fn lean_of(m: &crate::syntax::Measured, sets: &[(String, &SetLit)], p: &str,
           number: &dyn Fn(&str, Span) -> Arg) -> Option<Arg> {
    let side = |lit: &SetLit| -> Option<&'static str> {
        let [st] = lit.body.as_slice() else { return None };
        let w = match &st.kind { StmtKind::Relation(r) => r.form.written()?, _ => return None };
        let ok = w.ops.len() == 2 && w.ops[0].root.text == lit.bound.text && w.args.is_empty();
        match w.word.text.as_str() {
            "outside" if ok => Some("outside"),
            "inside" if ok => Some("inside"),
            _ => None,
        }
    };
    let [(a, x), (b, y)] = sets else { return None };
    let (out, into) = match (side(x)?, side(y)?) {
        ("outside", "inside") => (a, b),
        ("inside", "outside") => (b, a),
        _ => return None,
    };
    let lhs = m.lhs.text().replace(' ', "");
    let rhs = m.rhs.text();
    if m.cmp != Cmp::Ge || lhs != format!("distance({p},{out})-distance({p},{into})") {
        return None;
    }
    let rhs = rhs.trim();
    let inner = rhs.strip_prefix("tan(")?;
    let close = inner.rfind(") * distance(")?;
    let (theta, rest) = (&inner[..close], &inner[close..]);
    if !rest.replace(' ', "").starts_with(&format!(")*distance({p},")) {
        return None;
    }
    Some(number(theta, m.span))
}
