//! Region terms (§6.21): what a bound on a region solid's probe bounds, read off the constraint
//! or the written relation, and the sets its terms measure from.

use super::relations::to_arg;
use super::resolve::Resolver;
use super::{Code, Diag};
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::ir::Statement as Stmt;
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Arg, Ref, Span};
use std::collections::BTreeMap;
use crate::expr;


/// The hidden points region solids' bodies are applied to (`solid(R)`, §6.21), each with the
/// bounds stated on it: gathered as relations are stated, taken by each solid as it is built
/// (`SolidDef::Region`).  Never a row: a probe's bound describes material and chooses no root.
#[derive(Default)]
pub(super) struct Probes {
    pub terms: BTreeMap<usize, Vec<crate::model::RegionTerm>>,
    /// Per probe, the sets a bound of it measured from with nothing (a chamfer of no leg): read by
    /// no term, so held when its solid is built, as `fill_sets` holds a set it reads.
    pub idle: BTreeMap<usize, Vec<usize>>,
}

/// The region solid's probe a constraint reads, where it reads one: an operand, or a line's end.
pub(super) fn probe_of(sk: &Sketch, c: &Constraint, probes: &Probes) -> Option<usize> {
    if probes.terms.is_empty() {
        return None;
    }
    points_read(sk, c).find(|p| probes.terms.contains_key(p))
}

/// The points a constraint reads, as a probe is read: each operand a point, and each edge's ends.
pub(super) fn points_read<'a>(sk: &'a Sketch, c: &Constraint) -> impl Iterator<Item = usize> + 'a {
    c.entities().into_iter().flat_map(|e| match e.kind {
        EntKind::Point => vec![e.i()],
        _ => crate::model::edge_ends(sk, e)
            .map_or(Vec::new(), |(a, b)| vec![a as usize, b as usize]),
    })
}

/// The axis of the cone an `Angle3` at a probe states: the angle from a line seen from its start,
/// the probe on the other line, which begins where the axis does.
pub(super) fn cone_axis(sk: &Sketch, c: &Constraint, probe: usize) -> Result<EntRef, String> {
    let ends = |l: EntRef| [sk.lines[l.i()].p1 as usize, sk.lines[l.i()].p2 as usize];
    let (a, g) = (c.args[0].ent(), c.args[1].ent());
    let (axis, seen) = if ends(g).contains(&probe) { (a, g) } else { (g, a) };
    // the apex one point, though the axis drawn in a view takes its twin there (§6.7)
    let apex = ends(seen).into_iter().find(|&p| p != probe);
    if !apex.is_some_and(|p| sk.twinned(p, ends(axis)[0])) {
        return Err("a cone's angle is seen from its axis's start: the line to the point begins \
                    where the axis does"
            .to_string());
    }
    Ok(axis)
}

/// What a bound on a region solid's probe bounds (§6.21), read off the constraint's **kind** and
/// never the set it came from: a distance from a point is a ball, from a line a cylinder, the
/// angle from a line seen from its start a cone, an ordinate along a plane's normal a half-space.
pub(super) fn region_term(sk: &Sketch, c: &Constraint, probe: usize, grow: Option<crate::model::TermNumber>,
    name: String) -> Result<crate::model::RegionTerm, String> {
    use crate::model::{RegionShape, RegionTerm};
    let kind = crate::syntax::snake(c.kind.name());
    // a set measured from holds its point's equalities as bounds too, `in: (d, d)`: the
    // flattener says so (#192), so one rule reads every probe
    let Some(b) = c.bound else {
        return Err(format!(
            "a region solid's set states `{kind}`, which bounds nothing: its body bounds numbers"
        ));
    };
    let at = c.kind.dimension_slot().expect("a bound is on a number");
    let lo = term_number(&c.args[at], sk, c.kind.dimension_kind())
        .ok_or("a region's number is stated or reads one unknown of the drawing")?;
    let ent = |i: usize| c.args[i].ent();
    let other =
        |a: EntRef, b: EntRef| if a.i() == probe && a.kind == EntKind::Point { b } else { a };
    let shape = match c.kind {
        CKind::Distance | CKind::Distance3 => {
            RegionShape::Ball { center: other(ent(0), ent(1)).i() as u32 }
        }
        CKind::PointLineDistance | CKind::PointLine3 => {
            RegionShape::Cylinder { line: ent(1).i() as u32 }
        }
        CKind::Angle3 => RegionShape::Cone { axis: cone_axis(sk, c, probe)?.i() as u32 },
        CKind::Ordinate => {
            let from = ent(0);
            let normal = matches!(&c.args[4], CArg::Str(w) if w == "n");
            // along a plane's normal from its origin (`inside`/`outside` the plane), or along a
            // line or an axis from any point (`std.HalfSpace`)
            let along = match c.args[2] {
                CArg::Ent(t) if matches!(t.kind, EntKind::Line | EntKind::Axis) => Some(t),
                _ => sk.plane_of_origin(from.i()).filter(|_| normal).map(EntRef::plane),
            };
            let Some(along) = along else {
                return Err("a region's ordinate is along a plane's normal (`inside`/`outside` \
                            the plane), a line or an axis"
                    .to_string());
            };
            RegionShape::Plane {
                along,
                from: from.i() as u32,
                sign: if ent(1).i() == probe { 1.0 } else { -1.0 },
            }
        }
        _ => {
            return Err(format!("a region solid bounds distances, angles and planes, not `{kind}`"))
        }
    };
    if grow.is_some() && (b.hi.is_some() || !matches!(shape, RegionShape::Cone { .. } | RegionShape::Plane { .. })) {
        return Err("an offset is of a cone's surface or a plane's, on one side of it".to_string());
    }
    Ok(RegionTerm { shape, cmp: b.cmp, lo, hi: b.hi, grow, name })
}

/// The name of a region's term (#212): the set the region's body used, where it used one (`tip`
/// for `p inside tip` in `region`'s body), else the region's own — the second of the sets an
/// application went through, else the first.
pub(super) fn term_name(through: &[String]) -> String {
    through.get(1).or(through.first()).cloned().unwrap_or_default()
}

/// A region term's number affine in an unknown, in the kernels' units (an angle in radians);
/// stated where it reads none, or reads it times nothing.
fn term_of(a: expr::Aff, kind: SpecKind) -> crate::model::TermNumber {
    crate::model::TermNumber {
        c: expr::to_arg_units(kind, a.c),
        free: a.free.filter(|_| a.m != 0.0).map(|name| (name, expr::to_arg_units(kind, a.m))),
    }
}

/// A region term's number (§6.21): a literal or a text reading no unknown as it stands, else one
/// affine in an unknown of the drawing, read once it is solved (`TermNumber::value`).  `None` for
/// a text that is neither.
pub(super) fn term_number(a: &CArg, sk: &Sketch, kind: SpecKind) -> Option<crate::model::TermNumber> {
    use crate::model::TermNumber;
    let e = match a {
        CArg::Num(v) => return Some(TermNumber::stated(*v)),
        CArg::Expr(e) => e,
        _ => return None,
    };
    let read = written_aff(a, sk)?;
    Some(if read.free.is_none() { TermNumber::stated(e.value) } else { term_of(read, kind) })
}

/// A number as the kernels read it (an angle in radians) where it is one before `expr::evaluate`:
/// a literal, or a text reading no unknown (`to_arg` values it).  `None` where it reads one.
pub(super) fn settled_number(a: &CArg, sk: &Sketch) -> Option<f64> {
    match a {
        CArg::Num(v) => Some(*v),
        CArg::Expr(e) => written_number(a, sk).map(|_| e.value),
        _ => None,
    }
}

/// What a dimension's text comes to as written, in the document's units (degrees for an angle):
/// `None` for a number with no text, or a text that reads an unknown.
pub(super) fn written_number(a: &CArg, sk: &Sketch) -> Option<f64> {
    written_aff(a, sk).and_then(|a| a.number())
}

/// A dimension's text worked out over no scope, in the document's units: a number, or affine in
/// the one unknown it reads.  `None` for a number with no text, or a text that is neither.  A
/// declared unknown is what its declaration says (`param m: Length`), so `0.05 * m + 0.3mm` adds
/// two lengths.
fn written_aff(a: &CArg, sk: &Sketch) -> Option<expr::Aff> {
    let CArg::Expr(e) = a else { return None };
    let parsed = expr::parse_in(&e.text, sk.units).ok()?;
    expr::eval_declared(&parsed.body, &sk.declared).ok()
}

/// A sum of measurements and a number: `Σ cᵢ mᵢ + c`, each `mᵢ` a measurement as written, and
/// `c` stated or affine in one unknown of the drawing (a radius reading the solved module).
type Linear<'m> = (Vec<(&'m crate::syntax::Measurement, f64)>, expr::Aff);

/// What a side of a bound on measurements comes to as a sum of them times numbers (#183 F3), or
/// why it is not one: a product or a quotient of two measurements, or one read through a function.
fn linear<'m>(m: &'m crate::syntax::Measuring, sk: &Sketch) -> Result<Linear<'m>, String> {
    use crate::syntax::Measuring;
    use expr::{Aff, Op};
    let scaled = |(terms, c): Linear<'m>, k: f64| -> Linear<'m> {
        (terms.into_iter().map(|(m, x)| (m, x * k)).collect(), c.times(k))
    };
    // a measurement's coefficient is a plain number: an unknown times a measurement is no bound
    // on measurements a region can read
    let number = |a: &'m Measuring| -> Option<f64> {
        a.measures().is_empty().then(|| linear(a, sk).ok().and_then(|(_, c)| c.number())).flatten()
    };
    Ok(match m {
        // a declared unknown is what it was declared, as in every other number (`with_declared`)
        Measuring::Number { text, .. } => {
            let v = expr::eval_declared(&expr::parse_in(text, sk.units)?.body, &sk.declared)
                .map_err(|e| e.message)?;
            (Vec::new(), v)
        }
        Measuring::Measure(x) => (vec![(x, 1.0)], Aff::num(0.0)),
        Measuring::Neg(a) => scaled(linear(a, sk)?, -1.0),
        Measuring::Bin(op @ (Op::Add | Op::Sub), a, b) => {
            let (mut terms, c) = linear(a, sk)?;
            let k = if *op == Op::Add { 1.0 } else { -1.0 };
            let (more, d) = linear(b, sk)?;
            terms.extend(more.into_iter().map(|(m, x)| (m, x * k)));
            (terms, one_unknown(&c, &d, k)?)
        }
        Measuring::Bin(Op::Mul, a, b) => match (number(a), number(b)) {
            (Some(k), _) => scaled(linear(b, sk)?, k),
            (_, Some(k)) => scaled(linear(a, sk)?, k),
            _ => return Err("a bound on measurements is a sum of them, each times a number: \
                             two measured are not multiplied".to_string()),
        },
        Measuring::Bin(Op::Div, a, b) => match number(b) {
            Some(k) if k != 0.0 => scaled(linear(a, sk)?, 1.0 / k),
            _ => return Err("a bound on measurements is a sum of them, each times a number: \
                             a measurement divides nothing".to_string()),
        },
        _ => return Err("a bound on measurements is a sum of them, each times a number: none is \
                         read through a function or a power".to_string()),
    })
}

/// `a + k·b` of a bound on measurements' numbers, which read one unknown between them.
fn one_unknown(a: &expr::Aff, b: &expr::Aff, k: f64) -> Result<expr::Aff, String> {
    a.plus(b, k).map_err(|_| {
        let (x, y) = (a.free.clone().unwrap_or_default(), b.free.clone().unwrap_or_default());
        format!("a bound on measurements reads one unknown, and `{x}` and `{y}` are two")
    })
}

/// `distance(p, f1) + distance(p, f2) <= 10mm` (#183 F3): a term of the region solid whose probe
/// it measures — each measurement a distance between the probe and what else it names, the
/// bound on their sum — or refused (E040): read only of a region solid's point.  What each
/// measurement names is a reference the flattener has made absolute (#190).
pub(super) fn measured_term(
    sk: &Sketch,
    res: &Resolver,
    m: &crate::syntax::Measured,
    st: &Stmt,
    probes: &mut Probes,
    diags: &mut Vec<Diag>,
) {
    let mut refuse = |message: String| {
        diags.push(Diag::on(Code::E040, st, message))
    };
    let (lhs, rhs) = match (linear(&m.lhs, sk), linear(&m.rhs, sk)) {
        (Ok(l), Ok(r)) => (l, r),
        (Err(e), _) | (_, Err(e)) => return refuse(e),
    };
    // `Σ cᵢ mᵢ cmp k`, every measurement on the left
    let mut terms = lhs.0;
    terms.extend(rhs.0.into_iter().map(|(m, c)| (m, -c)));
    // the number the sum is bounded by, stated or affine in the one unknown either side reads
    let bound = match one_unknown(&rhs.1, &lhs.1, -1.0) {
        Ok(n) => n,
        Err(e) => return refuse(e),
    };
    let (k, free) = (bound.c, bound.free.as_ref().filter(|_| bound.m != 0.0));
    use crate::model::Target;
    let target = |r: &Ref| -> Option<Target> {
        let of = res.operand(sk, r).ok()?;
        of.faces().map(Target::Face).or_else(|| of.ent().map(Target::Ent))
    };
    let probe_at = |t: &Option<Target>| match t {
        Some(Target::Ent(e)) if e.kind == EntKind::Point && probes.terms.contains_key(&e.i()) => Some(e.i()),
        _ => None,
    };
    // the material's angle where two faces meet, at the point of their edge nearest the region's
    // point (#183 F9): a bound of its own
    if let [(x, c)] = terms.as_slice() {
        if x.measure == expr::Measure::Angle {
            let face = |r: &Ref| match target(r) {
                Some(Target::Face(f)) if !f.surface => Some(f),
                _ => None,
            };
            let read = match (x.args.as_slice(), &x.at) {
                ([a, b], Some(at)) => face(a).zip(face(b)).zip(probe_at(&target(at))),
                _ => None,
            };
            let Some(((fa, fb), p)) = read else {
                return refuse("a region's angle is between two faces, read `at:` its point: \
                               `angle(a, b, at: p)`".to_string());
            };
            if free.is_some() {
                return refuse("a region's angle is bounded by a number stated".to_string());
            }
            let cmp = if *c > 0.0 { m.cmp } else { m.cmp.turned().expect("one-sided") };
            let lo = expr::to_arg_units(SpecKind::Angle, k / c);
            let term = crate::model::RegionTerm::new(
                crate::model::RegionShape::Angle { a: fa, b: fb },
                cmp,
                crate::model::TermNumber::stated(lo),
            );
            probes.terms.get_mut(&p).expect("a probe's terms").push(term);
            return;
        }
    }
    let mut probe = None;
    let mut of: Vec<(Target, f64)> = Vec::new();
    for (x, c) in terms {
        if x.measure != expr::Measure::Distance || x.at.is_some() {
            return refuse(format!(
                "a region's bound measures distances from its point; `{}` is not read yet",
                x.text()
            ));
        }
        let ends: Vec<Option<Target>> = x.args.iter().map(target).collect();
        // the point the set is about is the first operand where both are such points
        let Some(at) = (0..ends.len()).find(|&k| probe_at(&ends[k]).is_some()) else {
            return refuse(format!(
                "`{}` measures nothing of a region solid's point: a bound on measurements is read \
                 where a set is applied to one (`solid(S)`)",
                x.text()
            ));
        };
        let p = probe_at(&ends[at]).expect("found");
        if probe.is_some_and(|q| q != p) {
            return refuse("a bound on measurements reads one point".to_string());
        }
        probe = Some(p);
        let Some(mut to) = ends[1 - at].clone() else {
            return refuse(format!("`{}` names nothing to measure from",
                                  crate::syntax::ref_text(&x.args[1 - at])));
        };
        // a set measured from has a point of its own, whose bounds it is (#183 F2)
        if let Some(q) = probe_at(&ends[1 - at]) {
            to = Target::Set { probe: q as u32, terms: Vec::new() };
        }
        if let Target::Ent(e) = &to {
            if !matches!(e.kind, EntKind::Point | EntKind::Line | EntKind::Axis | EntKind::Plane) {
                return refuse(format!(
                    "a region's bound measures the distance from a point, a line, an axis, a \
                     plane, a face or a set, not {}",
                    e.kind.a()
                ));
            }
        }
        match of.iter_mut().find(|(e, _)| *e == to) {
            Some((_, x)) => *x += c,
            None => of.push((to, c)),
        }
    }
    let Some(p) = probe else {
        return refuse("a bound on measurements measures its point".to_string());
    };
    for (t, _) in of.iter().filter(|(_, c)| *c == 0.0) {
        if let Target::Set { probe: q, .. } = t {
            probes.idle.entry(p).or_default().push(*q as usize);
        }
    }
    of.retain(|&(_, c)| c != 0.0);
    // every measurement weighed nothing (a chamfer of no leg): `0 cmp k` holds everywhere, so it
    // bounds nothing, or nowhere
    if of.is_empty() {
        use crate::syntax::Cmp;
        if free.is_some() {
            return refuse("a bound on measurements weighing nothing is bounded by a number stated"
                .to_string());
        }
        let holds = match m.cmp { Cmp::Ge => 0.0 >= k, Cmp::Le => 0.0 <= k, Cmp::In => false };
        if !holds {
            refuse("a bound on measurements weighed nothing that holds nowhere".to_string());
        }
        return;
    }
    let shape = crate::model::RegionShape::Measured { of };
    let number = term_of(bound, SpecKind::Length);
    let name = term_name(&m.through);
    let term = crate::model::RegionTerm { name, ..crate::model::RegionTerm::new(shape, m.cmp, number) };
    probes.terms.get_mut(&p).expect("a probe's terms").push(term);
}

/// `p inside F`, `p outside F`, `p distance(<= d) F`, `p coincident F.surface` … (`Shape::Face`):
/// the term of the region solid whose point `p` is, or why not.
pub(super) fn face_term(sk: &Sketch, res: &Resolver, w: &crate::syntax::Written, point: EntRef,
             face: &crate::model::FaceRef, probes: &Probes)
    -> Result<(usize, crate::model::RegionTerm), String> {
    use crate::model::{RegionShape, RegionTerm, Target, TermNumber};
    use crate::syntax::{Cmp, OpArg};
    if !probes.terms.contains_key(&point.i()) {
        return Err(format!(
            "`{}` is a drawn point: a point is read against a face where a set is applied to a \
             region solid's point (`solid(S)`)",
            w.ops[0].root.text
        ));
    }
    let probe = point.i();
    let word = w.word.text.as_str();
    // a side as written or as a set's body carries it (`Written::side`, #192)
    let side = w.side();
    let number = |text: &str, span: Span| -> Result<TermNumber, String> {
        let arg = Arg::Dim { text: text.to_string(), span };
        to_arg(sk, res, SpecKind::Length, &arg).ok().as_ref()
            .and_then(|a| term_number(a, sk, SpecKind::Length))
            .ok_or_else(|| format!("`{text}` is no length"))
    };
    let dim = w.args.iter().find_map(|a| match a {
        OpArg::Dim(t, sp) => Some((t.clone(), *sp)),
        _ => None,
    });
    let bound = w.bound();
    let term = |shape, cmp, lo, hi| RegionTerm { hi, ..RegionTerm::new(shape, cmp, lo) };
    let made = (|| -> Result<RegionTerm, String> {
        // the signed distance from the face's surface toward its outside, bounded at its level
        if let Some((cmp, level)) = &side {
            if face.surface {
                return Err("a side is read off a face: name the face, not its surface".to_string());
            }
            let lo = match level {
                Some((t, sp)) => number(t, *sp)?,
                None => TermNumber::stated(0.0),
            };
            return Ok(term(RegionShape::FaceSide { face: face.clone() }, *cmp, lo, None));
        }
        match word {
            "distance" => {
                let Some((t, sp)) = &dim else { return Err("a distance states a number".to_string()) };
                let lo = number(t, *sp)?;
                let of = vec![(Target::Face(face.clone()), 1.0)];
                Ok(match &bound {
                    Some((Cmp::In, Some((hi, sp)))) => {
                        let hi = number(hi, *sp)?;
                        term(RegionShape::Measured { of }, Cmp::In, lo, Some(hi.c))
                    }
                    Some((cmp, _)) => term(RegionShape::Measured { of }, *cmp, lo, None),
                    None => {
                        let at = lo.c;
                        term(RegionShape::Measured { of }, Cmp::In, lo, Some(at))
                    }
                })
            }
            "coincident" => {
                let of = vec![(Target::Face(face.clone()), 1.0)];
                Ok(term(RegionShape::Measured { of }, Cmp::Le, TermNumber::stated(0.0), None))
            }
            _ => Err(format!("a region's point is `inside`, `outside`, `coincident` with or at a \
                              `distance` from a face, not `{word}`")),
        }
    })();
    made.map(|t| (probe, t))
}

/// `p inside S.ends` (#183 F7, `Shape::Ends`), `S` a set measured from at its own point
/// (`flatten::sets`): `p` on the side the edge `S` is runs on of each face it ends on.
pub(super) fn ends_term(w: &crate::syntax::Written, point: EntRef, edge: EntRef, probes: &Probes)
    -> Result<(usize, crate::model::RegionTerm), String> {
    use crate::model::{RegionShape, RegionTerm, Target, TermNumber};
    if !probes.terms.contains_key(&point.i()) || !probes.terms.contains_key(&edge.i()) {
        return Err(format!(
            "`{}` is a drawn point: where an edge ends is read of a region solid's point, where a \
             set is applied to one (`solid(S)`)",
            w.ops[0].root.text
        ));
    }
    let Some((cmp, None)) = w.side() else {
        return Err("where an edge ends is a side of it: `inside S.ends`".to_string());
    };
    if cmp != crate::syntax::Cmp::Le {
        return Err("past where an edge ends is no region: say `inside S.ends`".to_string());
    }
    let shape = RegionShape::Ends {
        edge: Box::new(Target::Set { probe: edge.i() as u32, terms: Vec::new() }),
    };
    Ok((point.i(), RegionTerm::new(shape, cmp, TermNumber::stated(0.0))))
}

/// The terms of the sets a region's terms (those stated of `probe`) measure from (#183 F2, F7),
/// from the points their bodies were applied to, those points held (they stand for no place);
/// `Err` for a set measured in terms of itself.
pub(super) fn fill_sets(sk: &mut Sketch, terms: &mut [crate::model::RegionTerm], probes: &Probes,
                        probe: usize, depth: usize) -> Result<(), String> {
    use crate::model::{RegionShape, Target};
    if depth > 16 {
        return Err("a set measured from is measured in terms of itself".to_string());
    }
    // a set's point held and its own terms filled, in turn
    let held = |q: usize, sk: &mut Sketch| -> Result<Vec<crate::model::RegionTerm>, String> {
        let mut inner = probes.terms.get(&q).cloned().unwrap_or_default();
        fill_sets(sk, &mut inner, probes, q, depth + 1)?;
        for x in sk.own_params(EntRef::point(q)) {
            sk.params[x as usize].fixed = true;
        }
        Ok(inner)
    };
    let fill = |t: &mut Target, sk: &mut Sketch| -> Result<(), String> {
        if let Target::Set { probe, terms } = t {
            if probes.terms.get(&(*probe as usize)).is_none_or(|t| t.is_empty()) {
                return Err("a set measured from states nothing of its point".to_string());
            }
            *terms = held(*probe as usize, sk)?;
        }
        Ok(())
    };
    // a set no term reads still has its point held (`Probes::idle`)
    for &q in probes.idle.get(&probe).into_iter().flatten() {
        held(q, sk)?;
    }
    for t in terms.iter_mut() {
        match &mut t.shape {
            RegionShape::Measured { of } => {
                for (target, _) in of.iter_mut() {
                    fill(target, sk)?;
                }
            }
            RegionShape::Ends { edge } => fill(edge, sk)?,
            _ => {}
        }
    }
    Ok(())
}
