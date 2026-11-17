//! **Claims over measurements of the solved drawing** (#940), judged after the solve:
//! `claim length(post) <= 96"`, `claim distance(a, b) <= 8ft`, `claim angle(rafter, joist) >=
//! 20deg`.  The flattener worked out the numbers each reads where it stands
//! (`flatten::MeasuredClaim`); what it measures is read here, where the solve put the geometry,
//! in the document's units — a length in its unit, an angle in degrees, as every number a claim
//! compares is.

use super::{Code, Diag, SourceMap};
use crate::expr::{self, Aff, Measure};
use crate::flatten::{MeasuredClaim, Valued};
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::Span;
use crate::units::Dim;

/// A claim over measurements, judged at the solved pose.
#[derive(Clone, Debug)]
pub struct MeasuredJudged {
    pub text: String,
    pub span: Span,
    /// Whether it holds where the drawing solved to (within rounding, as a claim over values is
    /// judged), or why it has no answer there.
    pub holds: Result<bool, String>,
    /// Each measurement it makes, as written (`length(post)`), then each number it reads by
    /// absolute name, with what it came to.
    pub reads: Vec<(String, Valued)>,
}

/// **Judge a claim over measurements** at the drawing `sk` solved to, its names read by `map`.
pub fn judge_measured(sk: &Sketch, map: &SourceMap, c: &MeasuredClaim) -> MeasuredJudged {
    let measured = std::cell::RefCell::new(Vec::<(String, Valued)>::new());
    let measure = |m: Measure, args: &[String]| -> Result<Aff, String> {
        let (value, dim) = measure_of(sk, map, &c.prefix, m, args)?;
        // said by the names a reader writes from the document's top, an instance's included
        let named: Vec<String> = args.iter()
            .map(|a| super::public_path(&format!("{}{}", c.prefix, a.trim()))).collect();
        let said = m.text(&named);
        let mut seen = measured.borrow_mut();
        if !seen.iter().any(|(t, _)| *t == said) {
            seen.push((said, Valued { value, dim }));
        }
        Ok(Aff::of_dim(value, dim))
    };
    let holds = expr::judge_measured(&c.cond, &c.env, &measure).map_err(|e| e.message);
    let mut reads = measured.into_inner();
    reads.extend(c.reads.iter().cloned());
    MeasuredJudged { text: c.text.clone(), span: c.span, holds, reads }
}

/// What each claim over measurements asks of names that are not there, or of geometry that has
/// no such measure — a line's radius, the angle of two points — said where it stands, before the
/// solve: the names and the kinds are the document's, wherever the solve puts them.
pub(super) fn checked(sk: &Sketch, map: &SourceMap, claims: &[MeasuredClaim]) -> Vec<Diag> {
    let mut out = Vec::new();
    for c in claims {
        for (m, args) in c.cond.measures() {
            if let Err(why) = measure_of(sk, map, &c.prefix, m, &args) {
                out.push(Diag::new(Code::E101, c.span, None, format!("`{}`: {why}", c.text)));
                break;
            }
        }
    }
    out
}

/// The geometry `name` names under `prefix`: by its key, else by the path a reader writes.
fn named(sk: &Sketch, map: &SourceMap, prefix: &str, name: &str) -> Result<EntRef, String> {
    let key = format!("{prefix}{name}");
    map.ent_named(&key)
        .or_else(|| map.entity_path(sk, &key))
        .or_else(|| map.entity_path(sk, &super::public_path(&key)))
        .ok_or_else(|| format!("`{name}` names no geometry here"))
}

/// Where a point stands in space.
fn at(sk: &Sketch, e: EntRef) -> [f64; 3] {
    sk.world_point(e.i())
}

/// A line's two ends in space.
fn ends(sk: &Sketch, e: EntRef) -> ([f64; 3], [f64; 3]) {
    let l = &sk.lines[e.i()];
    (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    crate::space::sub(a, b)
}

/// **What a measurement comes to** on `sk` (`expr::Measure`): its value in the document's units
/// and its dimension — a line's length in space, an arc's along it, a circle's round it, a curve's
/// along its polyline; a circle's or an arc's radius; the distance between two points, or from a
/// point to a line produced; the angle between two lines' directions, 0° to 180°.
pub fn measure_of(sk: &Sketch, map: &SourceMap, prefix: &str, m: Measure, args: &[String])
    -> Result<(f64, Dim), String>
{
    let ents = args.iter().map(|a| named(sk, map, prefix, a.trim()))
        .collect::<Result<Vec<_>, _>>()?;
    let what = |e: EntRef| format!("`{}` is {}", args[ents.iter().position(|x| *x == e)
        .unwrap_or(0)].trim(), e.kind.a());
    match (m, ents.as_slice()) {
        (Measure::Length, [e]) => Ok((match e.kind {
            EntKind::Line => {
                let (a, b) = ends(sk, *e);
                crate::space::norm(sub(b, a))
            }
            EntKind::Arc => sk.arc_length(e.i()),
            EntKind::Circle => std::f64::consts::TAU * sk.radius_value(*e),
            EntKind::Curve => sk.curve_polyline(e.i()).windows(2)
                .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1)).sum(),
            _ => return Err(format!("{}, and a length is a line's, an arc's, a circle's or a \
                                     curve's", what(*e))),
        }, Dim::LENGTH)),
        (Measure::Radius, [e]) => match e.kind {
            EntKind::Circle | EntKind::Arc => Ok((sk.radius_value(*e), Dim::LENGTH)),
            _ => Err(format!("{}, and a radius is a circle's or an arc's", what(*e))),
        },
        (Measure::Distance, [a, b]) => {
            let from_line = |p: EntRef, l: EntRef| {
                let (x, y) = ends(sk, l);
                let d = sub(y, x);
                let len = crate::space::norm(d);
                let q = sub(at(sk, p), x);
                if len == 0.0 { crate::space::norm(q) }
                else { crate::space::norm(crate::space::cross(d, q)) / len }
            };
            match (a.kind, b.kind) {
                (EntKind::Point, EntKind::Point) => {
                    Ok((crate::space::norm(sub(at(sk, *b), at(sk, *a))), Dim::LENGTH))
                }
                (EntKind::Point, EntKind::Line) => Ok((from_line(*a, *b), Dim::LENGTH)),
                (EntKind::Line, EntKind::Point) => Ok((from_line(*b, *a), Dim::LENGTH)),
                _ => Err(format!("{} and {}: a distance is between two points, or from a point \
                                  to a line", what(*a), what(*b))),
            }
        }
        (Measure::Angle, [a, b]) => {
            if a.kind != EntKind::Line || b.kind != EntKind::Line {
                let e = if a.kind != EntKind::Line { *a } else { *b };
                return Err(format!("{}, and an angle is between two lines", what(e)));
            }
            let (p, q) = ends(sk, *a);
            let (r, s) = ends(sk, *b);
            let (u, v) = (sub(q, p), sub(s, r));
            let n = crate::space::norm(u) * crate::space::norm(v);
            if n == 0.0 { return Err("a line of no length has no direction".into()) }
            let cos = (crate::space::dot(u, v) / n).clamp(-1.0, 1.0);
            Ok((cos.acos().to_degrees(), Dim::ANGLE))
        }
        _ => Err(format!("`{}` measures {} {}", m.name(),
            match m { Measure::Length | Measure::Radius => "one thing", _ => "two things" },
            "named in it")),
    }
}
