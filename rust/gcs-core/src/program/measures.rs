//! **What a body is made of, stated of the drawing** (#510): `mass(10kg) arm`,
//! `volume(42 * 1cm^3) web`, `claim mass(<= 1kg) plank` — a measure of a beam, of a line a beam
//! is drawn on, or of a solid.  It answers to no load, so it is no scenario's: it is a relation of
//! the drawing, read once every beam and solid is built (`elaborate::finish`), since a beam is
//! read last and is no entity a relation's operand could otherwise name.  Stated, it is a row
//! (`CKind::Mass`, `CKind::Volume`) the solve holds; claimed, it is judged on the solved drawing
//! (`measured::read`, `report::measure_answers`).

use super::resolve::{Operand, Resolver};
use super::source_map::{Made, SourceMap};
use super::{Code, Diag};
use crate::constraints::{Arg, CKind, Constraint};
use crate::ir::{Operation as StmtKind, Statement};
use crate::model::{Applied, EntKind, EntRef, Measured, MeshedKey, Quantity, Sketch};
use crate::syntax::{OpArg, Span, StmtId};

/// **A mass or a volume and where it was written**: its statement, the statement's span, what a
/// report says it of (`mass(arm)`), its index on `Sketch::measured`, and its row where it drives
/// one.
#[derive(Clone, Debug)]
pub struct MeasureStated {
    pub stmt: StmtId,
    pub span: Span,
    pub head: String,
    pub index: usize,
    pub driven: Option<u32>,
}

/// The quantity a relation is written with, where it is a mass or a volume.
pub(super) fn quantity(r: &crate::ir::Relation) -> Option<Quantity> {
    r.form.written().and_then(|w| Quantity::parse(&w.word.text))
}

/// **Every mass and volume the drawing states**, in the order written: each read onto
/// `Sketch::measured`, a stated one compiled to its row (refused where the solve cannot read it,
/// **E040**), a claimed one left to be judged.  What cannot be read is said where it is written.
pub(super) fn measures(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    stating: &[&Statement],
    diags: &mut Vec<Diag>,
) -> Vec<MeasureStated> {
    let mut out = Vec::new();
    for st in stating {
        let StmtKind::Relation(r) = &st.kind else { continue };
        let Some(q) = quantity(r) else { continue };
        let w = r.form.written().expect("a written relation");
        let measured = match read(sk, res, r, w, q) {
            Ok(m) => m,
            Err((span, message)) => {
                diags.push(Diag::within(Code::E040, span.unwrap_or(st.span), st.id, message));
                continue;
            }
        };
        let head = format!("{}({})", q.word(), crate::syntax::ref_text(&w.ops[0]));
        let index = sk.measured.len();
        let claim = measured.claim;
        let value = measured.value;
        sk.measured.push(measured);
        let driven = if claim { None } else { drive(sk, map, st, q, index, value, &head, diags) };
        if !claim && driven.is_none() {
            continue;
        }
        out.push(MeasureStated { stmt: st.id, span: st.span, head, index, driven });
    }
    if out.iter().any(|m| m.driven.is_some()) {
        // the unknowns the members' areas are read over are bound once the rows read them
        crate::expr::evaluate(sk);
    }
    out
}

/// A measure's statement read, or why it cannot be: one operand, a beam, a line a beam is drawn
/// on or a solid; one number the drawing states, of the quantity's dimension and more than
/// nothing; and a bound (`<=`, `>=`) only where it is claimed — a bound on a mass chooses no root.
fn read(
    sk: &Sketch,
    res: &Resolver,
    r: &crate::ir::Relation,
    w: &crate::syntax::Written,
    q: Quantity,
) -> Result<Measured, (Option<Span>, String)> {
    let word = q.word();
    let [op] = &w.ops[..] else {
        return Err((None, format!("a {word} measures one thing: `{word}(…) arm`")));
    };
    let of = match res.operand(sk, op).map_err(|m| (Some(op.span), m))? {
        Operand::Beam(b) => Applied::Beam(b),
        Operand::Ent(e) | Operand::Body(e) if measurable(sk, e) => Applied::Ent(e),
        other => {
            let named = crate::syntax::ref_text(op);
            let what = match other.ent() {
                Some(e) => e.kind.a().to_string(),
                None => "no beam, line or solid".to_string(),
            };
            return Err((Some(op.span), format!(
                "a {word} is of a beam, a line a beam is drawn on, or a solid, and `{named}` is \
                 {what}")));
        }
    };
    for a in &w.args {
        match a {
            OpArg::Dim(..) => {}
            OpArg::Bound { hi: None, .. } => {}
            OpArg::Bound { span, .. } => {
                return Err((Some(*span), format!("a {word} is claimed at most or at least a \
                                                  number, never within two")));
            }
            _ => return Err((None, format!("a {word} takes its number alone: `{word}(10kg) arm`, \
                                            `claim {word}(<= 10kg) arm`"))),
        }
    }
    let Some((text, span)) = w.dim() else {
        return Err((None, format!("a {word} states its number: `{word}(…) arm`")));
    };
    let value = super::scenarios::stated(sk, text, q.dim(), word,
                                         "a measure's number is one the drawing states")
        .map_err(|(_, m)| (Some(span), m))?;
    if !(value > 0.0) {
        return Err((Some(span), format!("a {word} is more than nothing, and `{text}` is not")));
    }
    let compare = match w.bound() {
        None => crate::expr::Cmp::Eq,
        Some((crate::syntax::Cmp::Le, _)) if r.claim => crate::expr::Cmp::Le,
        Some((_, _)) if r.claim => crate::expr::Cmp::Ge,
        Some(_) => return Err((None, format!(
            "a bound on a {word} chooses no root: claim it, `claim {word}(<= …) {}`, or state it",
            crate::syntax::ref_text(op)))),
    };
    Ok(Measured { quantity: q, of, compare, value, claim: r.claim })
}

/// Whether a measure may be of entity `e`: a line a beam is drawn on, or a solid.
fn measurable(sk: &Sketch, e: EntRef) -> bool {
    match e.kind {
        EntKind::Line => sk.beams.iter().any(|b| b.lines.contains(&e.i())),
        EntKind::Solid => true,
        _ => false,
    }
}

/// A stated measure compiled to its row, `Sketch::measured[index]`: a prism's meshed once, here,
/// and held before the row is validated so the validation reads that mesh — or refused where it
/// is written (**E040**), the solve unable to read it.
#[allow(clippy::too_many_arguments)]
fn drive(
    sk: &mut Sketch,
    map: &mut SourceMap,
    st: &Statement,
    q: Quantity,
    index: usize,
    value: f64,
    head: &str,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    let kind = match q {
        Quantity::Mass => CKind::Mass,
        Quantity::Volume => CKind::Volume,
    };
    let args = vec![Arg::Int(index as i64), Arg::Num(value)];
    let key = MeshedKey::Measure(index);
    if let Ok(crate::constraints::InteriorRow::Meshed(row)) =
        Constraint::new(kind, args.clone()).interior_row(sk)
    {
        sk.meshed.insert(key, row);
    }
    let name = |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| crate::model::entity_name(e));
    if let Err(m) = crate::constraints::validate(sk, kind, &args, &name) {
        sk.meshed.remove(&key);
        diags.push(Diag::on(Code::E040, st, format!("`{head}` cannot drive the drawing: {m}")));
        return None;
    }
    let id = sk.add_quiet(Constraint::new(kind, args));
    map.record(st, Made::Con(id));
    Some(id)
}
