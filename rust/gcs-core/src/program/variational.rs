//! `k minimizes …` and `k maximizes …` (#121, #144), lowered: one `Stationary` constraint per term
//! over the statement's free curve, its integrand compiled once here so a bad one is refused where
//! it was written.  The curve's definition, pegs and free length are the sketch's
//! (`Sketch::settle_variational`), worked out once every row is stated.

use super::resolve::{follow, Resolver};
use super::{Code, Diag, Made, SourceMap};
use crate::constraints::{Arg, CKind, Constraint};
use crate::ir::Statement;
use crate::model::{EntKind, Sketch};
use crate::syntax::Minimize;

pub(super) fn state(
    sk: &mut Sketch,
    res: &Resolver,
    m: &Minimize,
    st: &Statement,
    map: &mut SourceMap,
    diags: &mut Vec<Diag>,
) {
    let err = |code: Code, span, message: String| Diag { code, span, stmt: Some(st.id), message };
    let Some(e) = res.lookup(&m.curve).and_then(|e| follow(sk, e, &m.curve.path).ok()) else {
        diags.push(err(Code::E101, m.curve.span, format!("`{}` names nothing", m.curve.root.text)));
        return;
    };
    if !(e.kind == EntKind::Curve && sk.curve_extremal(e.i())) {
        let what = if e.kind == EntKind::Curve { "a curve of its own definition".to_string() } else { e.kind.a().to_string() };
        diags.push(err(
            Code::E040,
            m.curve.span,
            format!(
                "an energy states the shape of a free curve, `{} := curve(a, b)`, and `{}` is {what}",
                m.curve.root.text, m.curve.root.text,
            ),
        ));
        return;
    }
    let mut degree = None;
    for t in &m.terms {
        let d = match crate::variational::integrand(&t.body, sk.units) {
            Ok(d) => d,
            Err(why) => {
                diags.push(err(Code::E103, t.body_span, why));
                continue;
            }
        };
        if degree.is_some_and(|was| was != d) {
            diags.push(err(
                Code::E103,
                t.span,
                "the terms of one energy are of one dimension: this one is not the first's".into(),
            ));
            continue;
        }
        degree = Some(d);
        let c = Constraint::new(
            CKind::Stationary,
            vec![
                Arg::Ent(e),
                Arg::Num(t.coef),
                Arg::Str(t.body.clone()),
                Arg::Int(d as i64),
                Arg::Bool(m.maximize),
            ],
        );
        let id = sk.add_quiet(c);
        map.record(st, Made::Con(id));
    }
}

/// The energies brought into step once every row is stated, each refusal said at the statement
/// the refused row came from.
pub(super) fn settle(sk: &mut Sketch, map: &SourceMap, diags: &mut Vec<Diag>) {
    for (id, why) in sk.settle_variational() {
        let site = map.site_of_constraint(id);
        diags.push(Diag {
            code: Code::E040,
            span: site.map(|s| s.span).unwrap_or_default(),
            stmt: site.map(|s| s.stmt),
            message: why,
        });
    }
    // a free curve whose length nothing holds and settles nowhere: said where its energy is
    for i in sk.seed_extremals() {
        let Some(lead) = sk.energy_of(i).and_then(|e| e.leader()) else { continue };
        let Some(site) = map.site_of_constraint(lead) else { continue };
        let e = crate::model::EntRef::new(EntKind::Curve, i);
        let name = map.name_of(e).cloned().unwrap_or_else(|| "the curve".into());
        diags.push(Diag {
            code: Code::W114,
            span: site.span,
            stmt: Some(site.stmt),
            message: format!(
                "nothing holds `{name}`'s length and no length makes its energy stationary: state its \
                 length, or something that fixes it (a point it passes, a line it touches)"
            ),
        });
    }
    // a free curve whose shape nothing states
    for i in 0..sk.curves.len() {
        if !sk.curve_extremal(i) || sk.energy_of(i).is_some() {
            continue;
        }
        let e = crate::model::EntRef::new(EntKind::Curve, i);
        let Some(site) = map.site_of(e) else { continue };
        let name = map.name_of(e).cloned().unwrap_or_else(|| "the curve".into());
        diags.push(Diag {
            code: Code::E040,
            span: site.span,
            stmt: Some(site.stmt),
            message: format!(
                "`{name}` is a free curve, and no energy states its shape: say what it is, \
                 `{name} minimizes integral(p.y over p)`"
            ),
        });
    }
}
