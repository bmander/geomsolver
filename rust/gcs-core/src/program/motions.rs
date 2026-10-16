//! **Frames' joints** (#383, `docs/frames-plan.md` stage 3): a frame's motion is three members —
//! `f.axis`, `f.ratio`, `f.advance` — held by relations like anything else, and lowered here to the
//! motion node every reader of a motion reads (`Sketch::motions`).  What kind of motion a joint is
//! has to be known before the solve by the readers that build with the drawing (a profile in the
//! plane, an envelope the solve holds), so it is read off **the statements holding its axis**, as
//! `views::origins_on_axes` reads which axes are wholly held: the axis on a drawn line is a turn
//! about that line (or with no ratio a slide along it); through a point of a view and parallel to
//! the view's normal, a turn in that view about the point; anything else a joint about the axis
//! as the solve leaves it, which only the readers after the solve take.  Its numbers are stated
//! in this stage, by a `fix` (which `std.Turn`, `std.Spin` and `std.Slide` write).
//!
//! A reading names a frame `under:` and, where it was a relative motion, one `seen_from:`
//! (`reading`): the relative of the two frames' nodes, minted once.
use super::resolve::Resolver;
use super::{Code, Diag, SourceMap};
use crate::ir::{Operation, Statement};
use crate::model::{EntKind, EntRef, MotionDef, MotionE, Sketch};
use crate::syntax::{Arg, OpArg, Ref, StmtId};
use std::collections::BTreeSet;

/// What the statements say of one joint: the numbers its `fix` states, and what its axis is held
/// to.
#[derive(Default)]
struct Held {
    ratio: Option<f64>,
    advance: Option<f64>,
    line: Option<usize>,
    point: Option<usize>,
    parallel: Option<EntRef>,
}

/// Every jointed frame's motion node, read off the statements that hold it.
pub(super) fn motions(sk: &mut Sketch, res: &Resolver, map: &SourceMap, body: &[&Statement],
                      skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    for f in 0..sk.frames.len() {
        let Some(joint) = sk.frames[f].joint.clone() else { continue };
        let frame = EntRef::frame(f);
        let axis = EntRef::new(EntKind::Axis, joint.axis as usize);
        let name = map.name_of(frame).cloned().unwrap_or_else(|| crate::io::entity_name(frame));
        let mut held = Held::default();
        for st in body.iter().filter(|st| !skip.contains(&st.id)) {
            let Operation::Relation(r) = &st.kind else { continue };
            let Some(w) = r.form.written() else { continue };
            if r.claim {
                continue;
            }
            let ent = |x: &Ref| res.entity(sk, x);
            match w.word.text.as_str() {
                "fix" if w.ops.first().and_then(ent) == Some(frame) => {
                    for a in &w.args {
                        let OpArg::Slot { key, arg } = a else { continue };
                        let slot = match key.text.as_str() {
                            "ratio" => &mut held.ratio,
                            "advance" => &mut held.advance,
                            _ => continue,
                        };
                        if let Arg::Seed { value, .. } = arg {
                            *slot = Some(*value);
                        }
                    }
                }
                "coincident" | "parallel" if w.ops.len() == 2 && w.args.is_empty() => {
                    let (a, b) = (ent(&w.ops[0]), ent(&w.ops[1]));
                    let other = match (a, b) {
                        (Some(a), Some(b)) if a == axis => b,
                        (Some(a), Some(b)) if b == axis => a,
                        _ => continue,
                    };
                    match (w.word.text.as_str(), other.kind) {
                        ("coincident", EntKind::Line) => held.line = Some(other.i()),
                        ("coincident", EntKind::Point) => held.point = Some(other.i()),
                        ("parallel", EntKind::Axis | EntKind::Line) => held.parallel = Some(other),
                        _ => {}
                    }
                }
                _ => {}
            }
        }
        let say = |diags: &mut Vec<Diag>, message: String| {
            let site = map.site_of(frame);
            diags.push(Diag::new(Code::E080, site.map(|s| s.span).unwrap_or_default(),
                                 site.map(|s| s.stmt), message));
        };
        let (Some(ratio), Some(advance)) = (held.ratio, held.advance) else {
            let missing = if held.ratio.is_none() { "ratio" } else { "advance" };
            say(diags, format!("`{name}`'s joint states no {missing}: a joint's numbers are \
                stated — `{missing}:` to `std.Turn`, `std.Spin` or `std.Slide`, or \
                `fix(ratio == r, advance == a) {name}`"));
            continue;
        };
        if !ratio.is_finite() || !advance.is_finite() {
            say(diags, format!("`{name}`'s joint needs a finite ratio and advance"));
            continue;
        }
        // a turn in a view: the axis through a point of it, square to it
        let in_view = held.point.and_then(|p| {
            let view = sk.plane_of(p)?;
            let normal = sk.frames[view].chart.as_ref()?.normal?;
            (held.parallel == Some(EntRef::new(EntKind::Axis, normal as usize))).then_some(p)
        });
        let def = match (held.line, in_view) {
            (Some(l), _) if ratio == 0.0 => MotionDef::Translation { axis: l as u32, advance },
            (Some(l), _) => MotionDef::Rotation { axis: l as u32, ratio, phase: 0.0, advance },
            (None, Some(_)) if advance != 0.0 => {
                say(diags, format!("`{name}` turns in a view about a point of it, and a turn in a \
                    view has no advance: a screw turns about a line, `std.Turn(f, about: l, \
                    ratio: r, advance: a)`"));
                continue;
            }
            (None, Some(c)) => MotionDef::Turn { centre: c as u32, ratio, phase: 0.0 },
            (None, None) => MotionDef::Joint { axis: joint.axis, ratio, advance },
        };
        sk.motions.push(MotionE { def, name: name.clone(), class: sk.frames[f].class.clone(),
                                  measured: Vec::new() });
        let m = sk.motions.len() - 1;
        if let Some(j) = sk.frames[f].joint.as_mut() {
            j.motion = Some(m as u32);
        }
    }
}

/// **A reading's motion** (#383): the frame named `under:`, seen from the one named `seen_from:`
/// where one is — its node, or the relative of the two (`n⁻¹·m`, minted once).  A frame seen from
/// one with no joint is seen from the world; one with no joint itself carries nothing, and is
/// refused naming how to turn it.
pub(super) fn reading(sk: &mut Sketch, res: &Resolver, under: &Ref, seen_from: Option<&Ref>)
    -> Result<usize, String> {
    let source = frame_motion(sk, res, under)?.ok_or_else(|| {
        let name = crate::syntax::ref_text(under);
        format!("`{name}` does not move: a frame's motion is its joint, held by relations — \
                 `std.Turn({name}, about: l, ratio: r)`, `std.Spin`, `std.Slide`")
    })?;
    let Some(g) = seen_from else { return Ok(source) };
    let Some(observer) = frame_motion(sk, res, g)? else { return Ok(source) };
    Ok(relative(sk, source, observer))
}

/// The motion node of the frame `r` names: its joint's, `None` for a frame with none.
pub(super) fn frame_motion(sk: &Sketch, res: &Resolver, r: &Ref) -> Result<Option<usize>, String> {
    let e = res.lookup(r).and_then(|e| super::resolve::follow_building(sk, res, e, r).ok())
        .ok_or_else(|| format!("no such frame: `{}`", crate::syntax::ref_text(r)))?;
    if e.kind != EntKind::Frame {
        return Err(format!("`{}` is {}, and a motion is read off a frame (`under: f`, `seen_from: \
            g`)", crate::syntax::ref_text(r), e.kind.a()));
    }
    Ok(sk.frames[e.i()].joint.as_ref().and_then(|j| j.motion).map(|m| m as usize))
}

/// The relative of motion nodes `source` seen from `observer`, `observer⁻¹·source`, minted the
/// first time a reading asks for it.
pub(super) fn relative(sk: &mut Sketch, source: usize, observer: usize) -> usize {
    let (s, o) = (source as u32, observer as u32);
    let found = sk.motions.iter().position(|m| matches!(m.def,
        MotionDef::Relative { source, observer } if source == s && observer == o));
    if let Some(i) = found {
        return i;
    }
    let name = format!("{} seen from {}", sk.motions[source].name, sk.motions[observer].name);
    sk.motions.push(MotionE { def: MotionDef::Relative { source: s, observer: o }, name,
                              class: Default::default(), measured: Vec::new() });
    sk.motions.len() - 1
}
