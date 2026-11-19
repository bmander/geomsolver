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

/// **A mass or a volume and where it was written**: its statement, what a report says it of
/// (`mass(arm)`), its index on `Sketch::measured`, and its row where it drives one.
#[derive(Clone, Debug)]
pub struct MeasureStated {
    pub stmt: StmtId,
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
        // a point's is its statement (`b centroid immersed`), any other its word over what it is
        // of (`mass(arm)`)
        let of = crate::syntax::ref_text(w.ops.last().expect("a measure's operand"));
        let head = match &w.ops[..] {
            [g, _] if q.point() => format!("{} {} {of}", crate::syntax::ref_text(g), q.word()),
            _ => format!("{}({of})", q.word()),
        };
        let index = sk.measured.len();
        let claim = measured.claim;
        // a point a centroid puts (#953) is three rows, one an ordinate of the world each
        let rows = if q.point() && !claim { 3 } else { 1 };
        for k in 0..rows {
            sk.measured.push(Measured { k, ..measured.clone() });
        }
        let mut driven = None;
        if !claim {
            for k in 0..rows {
                match drive(sk, map, st, index + k, &head, diags) {
                    Some(id) if k == 0 => driven = Some(id),
                    Some(_) => {}
                    None => {
                        driven = None;
                        break;
                    }
                }
            }
        }
        if !claim && driven.is_none() {
            continue;
        }
        out.push(MeasureStated { stmt: st.id, head, index, driven });
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
    let a = q.a();
    // a point a centroid puts (#953): `g centroid B`, the point first
    if q.point() {
        return point_read(sk, res, r, w, q);
    }
    let [op] = &w.ops[..] else {
        return Err((None, format!("{a} measures one thing: `{word}(…) arm`")));
    };
    let of = match res.operand(sk, op).map_err(|m| (Some(op.span), m))? {
        // a face's measure is of a face of a solid (#953)
        Operand::Face(f) if q.of_face() => Applied::Face(f),
        Operand::Beam(b) if !q.of_face() => Applied::Beam(b),
        Operand::Ent(e) | Operand::Body(e) if !q.of_face() && measurable(sk, e) => Applied::Ent(e),
        other => {
            let named = crate::syntax::ref_text(op);
            let what = match (other.ent(), &other) {
                (_, Operand::Face(_)) => "a face".to_string(),
                (Some(e), _) => e.kind.a().to_string(),
                (None, _) => "no beam, line or solid".to_string(),
            };
            let of = if q.of_face() { "a face of a solid (`hull.water`)" }
                     else { "a beam, a line a beam is drawn on, or a solid" };
            return Err((Some(op.span), format!("{a} is of {of}, and `{named}` is {what}")));
        }
    };
    let mut about = None;
    for arg in &w.args {
        match arg {
            OpArg::Dim(..) => {}
            OpArg::Bound { hi: None, .. } => {}
            OpArg::Bound { span, .. } => {
                return Err((Some(*span), format!("{a} is claimed at most or at least a \
                                                  number, never within two")));
            }
            // a second moment's line (#953)
            OpArg::Named(n, crate::syntax::Arg::Ref(l)) if n.text == "about"
                && q == Quantity::Moment => {
                about = Some(axis_named(sk, res, l)?);
            }
            _ if q == Quantity::Moment => return Err((None, "a second moment takes its number \
                and the line it is about: `moment(I, about: centreline) hull.water`".into())),
            _ => return Err((None, format!("{a} takes its number alone: `{word}(10kg) arm`, \
                                            `claim {word}(<= 10kg) arm`"))),
        }
    }
    if q == Quantity::Moment && about.is_none() {
        return Err((None, "a second moment is about a line: `moment(I, about: centreline) \
                           hull.water`".into()));
    }
    let Some((text, span)) = w.dim() else {
        return Err((None, format!("{a} states its number: `{word}(…) arm`")));
    };
    // a stated measure's number may read one unknown the drawing declares (#953), which the
    // row then determines: `mass(M) boat`
    let v = crate::flatten::value_read(text, &std::collections::BTreeMap::new(), &sk.declared,
                                       sk.units)
        .map_err(|m| (Some(span), format!("`{text}`: {m}")))?;
    v.dim.require(q.dim(), word).map_err(|m| (Some(span), format!("`{text}`: {m}")))?;
    let (value, free) = match (v.number(), v.free.clone()) {
        (Some(x), _) if x.is_finite() => (x, None),
        (None, Some(name)) if !r.claim && w.bound().is_none() && v.m.is_finite()
            && v.c.is_finite() && v.m != 0.0 => (v.c, Some((name, v.m))),
        (None, Some(_)) if r.claim => return Err((Some(span), format!(
            "`{text}` reads an unknown of the drawing, and a claim binds none: state the {word}, \
             `{word}(…) {}`, or claim a number", crate::syntax::ref_text(op)))),
        _ => return Err((Some(span), format!("`{text}` comes to no number the {word} can state"))),
    };
    if free.is_none() && !(value > 0.0) {
        return Err((Some(span), format!("{a} is more than nothing, and `{text}` is not")));
    }
    let compare = match w.bound() {
        None => crate::expr::Cmp::Eq,
        Some((crate::syntax::Cmp::Le, _)) if r.claim => crate::expr::Cmp::Le,
        Some((_, _)) if r.claim => crate::expr::Cmp::Ge,
        Some(_) => return Err((None, format!(
            "a bound on {a} chooses no root: claim it, `claim {word}(<= …) {}`, or state it",
            crate::syntax::ref_text(op)))),
    };
    Ok(Measured { quantity: q, of, compare, value, claim: r.claim, at: None, about, k: 0, free,
                  bodies: Vec::new() })
}

/// **A point a measure puts** (#953): `g centroid B`, `g centerofmass B` — a declared point and a
/// solid (or, for a centroid, a face of one), and nothing in parentheses.
fn point_read(
    sk: &Sketch,
    res: &Resolver,
    r: &crate::ir::Relation,
    w: &crate::syntax::Written,
    q: Quantity,
) -> Result<Measured, (Option<Span>, String)> {
    let word = q.word();
    let a = q.a();
    let [g, op] = &w.ops[..] else {
        return Err((None, format!("a point is put at {a} of one body: `g {word} hull`")));
    };
    let at = match res.operand(sk, g).map_err(|m| (Some(g.span), m))? {
        Operand::Ent(e) if e.kind == EntKind::Point => e.i(),
        _ => return Err((Some(g.span), format!("`{}` is no point: {a} puts a point, `g \
            {word} hull`", crate::syntax::ref_text(g)))),
    };
    // of a solid, a face of one, or every body of an instance (#953): what a body is made of
    // does not depend on how bodies are grouped into components
    let named = crate::syntax::ref_text(op);
    let bodies = match res.operand(sk, op) {
        Err(_) if op.path.is_empty() => res.bodies_under(sk, &op.root.text),
        _ => Vec::new(),
    };
    let of = match bodies.first() {
        Some(&b) => Applied::Ent(EntRef::solid(b)),
        None => match res.operand(sk, op).map_err(|m| (Some(op.span), m))? {
            Operand::Ent(e) | Operand::Body(e) if e.kind == EntKind::Solid => Applied::Ent(e),
            Operand::Face(f) if q == Quantity::Centroid => Applied::Face(f),
            _ => return Err((Some(op.span), format!("{a} is a solid's{}, or an instance's \
                bodies', and `{named}` is none", if q == Quantity::Centroid { ", a face's" }
                                                  else { "" }))),
        },
    };
    if !w.args.is_empty() {
        return Err((None, format!("{a} takes nothing in parentheses: `g {word} hull`")));
    }
    Ok(Measured { quantity: q, of, compare: crate::expr::Cmp::Eq, value: 0.0, claim: r.claim,
                  at: Some(at), about: None, k: 0, free: None, bodies })
}

/// The line or axis a second moment is about (#953), or why it names none.
fn axis_named(sk: &Sketch, res: &Resolver, l: &crate::syntax::Ref)
    -> Result<EntRef, (Option<Span>, String)>
{
    match res.operand(sk, l).map_err(|m| (Some(l.span), m))? {
        Operand::Ent(e) if matches!(e.kind, EntKind::Line | EntKind::Axis) => Ok(e),
        _ => Err((Some(l.span), format!("`{}` is no line and no axis: a second moment is about \
                                         one", crate::syntax::ref_text(l)))),
    }
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
fn drive(
    sk: &mut Sketch,
    map: &mut SourceMap,
    st: &Statement,
    index: usize,
    head: &str,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    let m = sk.measured[index].clone();
    let kind = match m.quantity {
        Quantity::Mass => CKind::Mass,
        Quantity::Volume => CKind::Volume,
        Quantity::Area => CKind::Area,
        Quantity::Moment => CKind::Moment,
        Quantity::Centroid => CKind::Centroid,
        Quantity::CenterOfMass => CKind::CenterOfMass,
    };
    let args = if m.quantity.point() {
        vec![Arg::Int(index as i64), Arg::Int(m.k as i64), Arg::Num(0.0)]
    } else {
        vec![Arg::Int(index as i64), Arg::Num(m.value)]
    };
    let key = MeshedKey::Measure(index);
    // the unknowns the row reads are its columns, bound from their seeds before it is made — a
    // waterline's height a volume sizes (#953); `evaluate` keeps them bound once the row reads them
    let mut unknowns = crate::beams::measured_unknowns(sk, &m.of).unwrap_or_default();
    unknowns.extend(m.free.iter().map(|(n, _)| n.clone()));
    for name in unknowns {
        if let Some(seed) = sk.declared.get(&name).and_then(|d| d.seed) {
            crate::expr::bind_free(sk, &name, seed);
        }
    }
    // a point's later ordinates read the first's mesh and hull, filled once (#953): only the
    // ordinate its row compares differs
    let first = (m.quantity.point() && m.k > 0)
        .then(|| sk.meshed.get(&MeshedKey::Measure(index - m.k)).cloned()).flatten();
    if let Some(row) = first {
        let mut row = (*row).clone();
        row.structure.gauge.what = crate::interior::What::Centroid { k: m.k };
        sk.meshed.insert(key, std::sync::Arc::new(row));
    } else if let Ok(crate::constraints::InteriorRow::Meshed(row)) =
        Constraint::new(kind, args.clone()).made_row(sk)
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
