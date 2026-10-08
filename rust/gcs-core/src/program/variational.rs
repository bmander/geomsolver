//! `minimize` and `maximize` (#121), lowered: one `Stationary` constraint per term, its curve
//! resolved, its integrand compiled once here so a bad one is refused where it was written.  The
//! groups, gauges and multipliers are the sketch's (`Sketch::settle_variational`), worked out
//! once every row is stated.

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
    let mut degree = None;
    for t in &m.terms {
        let err = |code: Code, span, message: String| Diag { code, span, stmt: Some(st.id), message };
        let Some(e) = res.lookup(&t.curve).and_then(|e| follow(sk, e, &t.curve.path).ok()) else {
            diags.push(err(Code::E101, t.curve.span, format!("`{}` names nothing", t.curve.root.text)));
            continue;
        };
        if e.kind != EntKind::Spline {
            diags.push(err(
                Code::E040,
                t.curve.span,
                format!(
                    "an energy varies a spline's shape, and `{}` is {}",
                    t.curve.root.text,
                    e.kind.a()
                ),
            ));
            continue;
        }
        let d = match crate::variational::integrand(&t.body, sk.units) {
            Ok(f) => f.degree,
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
}
