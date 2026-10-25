//! **Developable panels, built** (#645): what the flattener recognised (`flatten::developable`)
//! made into the drawing — a band's term at its region solid's probe (`RegionShape::Sheet`), and a
//! development's laid rails, `a` and `b`, curves drawn in its plane over its home ruling's ends
//! (`model::Developed`).

use super::elaborate::Elab;
use super::{Code, Diag, Made};
use crate::flatten::developable::{DevelopedInfo, RailInfo, SheetInfo};
use crate::model::{EntKind, EntRef, Rail};

/// A rail by the entities the drawing built for its names, or the name that names none.
fn rail(cx: &Elab<'_>, r: &RailInfo) -> Result<Rail, String> {
    let links = r.links.iter().map(|n| {
        cx.res.of.get(n).copied().filter(|e| matches!(e.kind,
            EntKind::Line | EntKind::Arc | EntKind::Circle | EntKind::Spline | EntKind::Curve
            | EntKind::Traced)).ok_or_else(|| format!("`{}` is no path a panel runs along",
                crate::program::public_path(n)))
    }).collect::<Result<Vec<EntRef>, String>>()?;
    Ok(Rail { links, chain: r.chain, closed: r.closed, reversed: r.reversed })
}

/// **Each band's term at its probe** (#645): the panel's rails and the band's side and thickness,
/// once the probes are known — its region solid takes it as it takes every term of its probe.
pub(super) fn sheets(cx: &mut Elab<'_>, sheets: &[SheetInfo]) {
    for s in sheets {
        let Some(p) = cx.res.of.get(&s.probe).copied() else { continue };
        let rails = match (rail(cx, &s.rails[0]), rail(cx, &s.rails[1])) {
            (Ok(a), Ok(b)) => [a, b],
            (Err(m), _) | (_, Err(m)) => {
                cx.diags.push(Diag::new(Code::E040, s.span, None, m));
                continue;
            }
        };
        let term = crate::model::RegionTerm {
            shape: crate::model::RegionShape::Sheet { rails: Box::new(rails), side: s.side },
            cmp: crate::vocab::Cmp::Le,
            lo: crate::model::TermNumber::stated(s.d),
            hi: None,
            grow: None,
            name: crate::program::public_path(&s.name),
        };
        cx.probes.terms.entry(p.i()).or_default().push(term);
    }
}

/// **Each development's laid rails** (#645), once the geometry is built and before a relation
/// reads them: `{flat}.a` and `{flat}.b`, curves over its home ruling's ends `f` and `g`, the
/// panel built between the rails and laid from the home point (`model::Developed`).
pub(super) fn developments(cx: &mut Elab<'_>, developed: &[DevelopedInfo],
                           stmts: &[&crate::ir::Statement]) {
    for dv in developed {
        let at = |cx: &mut Elab<'_>, m: String| {
            cx.diags.push(Diag::new(Code::E040, dv.span, None, m));
        };
        let point = |cx: &Elab<'_>, n: &str| cx.res.of.get(n).copied()
            .filter(|e| e.kind == EntKind::Point);
        let (Some(f), Some(g)) = (point(cx, &format!("{}f", dv.prefix)),
                                  point(cx, &format!("{}g", dv.prefix))) else { continue };
        let Some(s) = point(cx, &dv.home) else {
            at(cx, format!("`{}` is no point to lay a panel from", crate::program::public_path(&dv.home)));
            continue;
        };
        let rails = match (rail(cx, &dv.rails[0]), rail(cx, &dv.rails[1])) {
            (Ok(a), Ok(b)) => [a, b],
            (Err(m), _) | (_, Err(m)) => {
                at(cx, m);
                continue;
            }
        };
        // the statement the instance was written as: what the curves are recorded against
        let st = stmts.iter().find(|st| st.span == dv.span).copied();
        for (which, name) in ["a", "b"].into_iter().enumerate() {
            let mut dev = crate::model::Developed { rails: rails.clone(), s: s.idx, d: dv.d,
                                                    which: which as u8, n_const: 0 };
            dev.n_const = match dev.consts(&cx.sk) {
                Ok(k) => k.len(),
                Err(m) => {
                    at(cx, m);
                    break;
                }
            };
            let key = format!("{}{name}", dv.prefix);
            cx.sk.curve_defs.push(crate::model::developed_def(format!("developed:{key}"), dev));
            cx.sk.curves.push(crate::model::CurveE {
                def: (cx.sk.curve_defs.len() - 1) as u32,
                args: vec![f, g],
                unknowns: Vec::new(),
                values: Vec::new(),
                over: crate::model::Running::new("t", crate::units::Dim::SCALAR, 0.0, 1.0),
                home: crate::model::Home::At(0.0),
                pose: Vec::new(),
                class: Default::default(),
                trim: None,
                extrusion: false,
                length: None,
                pegs: Vec::new(),
                slides: Vec::new(),
            });
            let e = EntRef::new(EntKind::Curve, cx.sk.curves.len() - 1);
            // the home ruling's image as long as the ruling: one row, minted with the first
            if which == 0 {
                let mut c = crate::constraints::Constraint::new(
                    crate::constraints::CKind::DevelopedLength,
                    vec![crate::constraints::Arg::Ent(e)]);
                c.intrinsic = true;
                cx.sk.add_quiet(c);
            }
            cx.res.of.insert(key.clone(), e);
            cx.map.bind(&key, e, crate::syntax::Named::Written);
            if let Some(st) = st {
                cx.map.record(st, Made::Ent(e));
            }
        }
    }
}
