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
                        // a number pinned to an unknown is the solve's to find (stage 5), seeded
                        // where the unknown is
                        match arg {
                            Arg::Seed { value, .. } => *slot = Some(*value),
                            Arg::Tie { seed, .. } => {
                                let q = if key.text == "ratio" { joint.ratio }
                                    else { joint.advance };
                                if let Some(v) = seed {
                                    sk.params[q as usize].value = *v;
                                }
                            }
                            _ => {}
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
        // **a number no `fix` states is the solve's to find** (stage 5): its parameter is an
        // unknown, a column of every row reading the motion, read where it now stands
        let free = held.ratio.is_none() || held.advance.is_none();
        let value = |q: u32| sk.params[q as usize].value;
        let (ratio, advance) = (held.ratio.unwrap_or(value(joint.ratio)),
                                held.advance.unwrap_or(value(joint.advance)));
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
            (Some(l), _) if ratio == 0.0 && held.ratio.is_some() => {
                MotionDef::Translation { axis: l as u32, advance }
            }
            (Some(l), _) => MotionDef::Rotation { axis: l as u32, ratio, phase: 0.0, advance },
            (None, Some(_)) if advance != 0.0 || held.advance.is_none() => {
                say(diags, format!("`{name}` turns in a view about a point of it, and a turn in a \
                    view has no advance: a screw turns about a line, `std.Turn(f, about: l, \
                    ratio: r, advance: a)`"));
                continue;
            }
            (None, Some(c)) => MotionDef::Turn { centre: c as u32, ratio, phase: 0.0 },
            (None, None) => MotionDef::Joint { axis: joint.axis, ratio, advance },
        };
        let numbers = free.then_some([joint.ratio, joint.advance]);
        sk.motions.push(MotionE { def, name: name.clone(), class: sk.frames[f].class.clone(),
                                  numbers });
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

/// The motion node of the frame `r` names: its joint's after the joints of the frames it rides
/// on (#383), `None` for a frame nothing moves.
pub(super) fn frame_motion(sk: &mut Sketch, res: &Resolver, r: &Ref)
    -> Result<Option<usize>, String> {
    let e = res.lookup(r).and_then(|e| super::resolve::follow_building(sk, res, e, r).ok())
        .ok_or_else(|| format!("no such frame: `{}`", crate::syntax::ref_text(r)))?;
    if e.kind != EntKind::Frame {
        return Err(format!("`{}` is {}, and a motion is read off a frame (`under: f`, `seen_from: \
            g`)", crate::syntax::ref_text(r), e.kind.a()));
    }
    let carriers = sk.carriers(e).unwrap_or_default();
    carried(sk, &carriers)
}

/// **The motion of a path of carriers** (#383): their joints chained, outermost first —
/// `outer(τ)·inner(τ)`, `Sketch::chain`, each pair minted once — `None` for the world.
pub(super) fn carried(sk: &mut Sketch, carriers: &[u32]) -> Result<Option<usize>, String> {
    let nodes: Vec<usize> = carriers.iter()
        .filter_map(|&f| sk.frames.get(f as usize)?.joint.as_ref()?.motion)
        .map(|m| m as usize).collect();
    let Some((&first, rest)) = nodes.split_first() else { return Ok(None) };
    let mut m = first;
    for &n in rest {
        let name = format!("{}·{}", sk.motions[m].name, sk.motions[n].name);
        m = match sk.motions.iter().position(|x| x.name == name) {
            Some(i) => i,
            None => sk.chain(m, n, &name)?,
        };
    }
    Ok(Some(m))
}

/// The inverse of motion node `m`, minted the first time a reading asks for it: what a frame at
/// rest sees of the world from one that moves.
pub(super) fn inverse(sk: &mut Sketch, m: usize) -> usize {
    let of = m as u32;
    let found = sk.motions.iter()
        .position(|x| matches!(x.def, MotionDef::Inverse { of: o } if o == of));
    if let Some(i) = found {
        return i;
    }
    let name = format!("{}⁻", sk.motions[m].name);
    sk.motions.push(MotionE { def: MotionDef::Inverse { of }, name, class: Default::default(),
                              numbers: None });
    sk.motions.len() - 1
}

/// A name for `e` in a refusal: the document's, where the resolver has one.
pub(super) fn name_of(res: &Resolver, e: EntRef) -> String {
    res.of.iter().find(|(k, v)| **v == e && !k.starts_with('#')).map(|(k, _)| k.clone())
        .unwrap_or_else(|| crate::io::entity_name(e))
}

/// The jointed frames `e` rides on, outermost first, or why its parts ride on different ones.
pub(super) fn carriers(sk: &Sketch, res: &Resolver, e: EntRef) -> Result<Vec<u32>, String> {
    sk.carriers(e).map_err(|(a, b)| {
        let on = |x: EntRef| {
            let frames: Vec<String> = sk.carriers(x).unwrap_or_default().iter()
                .map(|&f| format!("`{}`", name_of(res, EntRef::frame(f as usize)))).collect();
            if frames.is_empty() { "on nothing".to_string() }
            else { format!("on {}", frames.join(" in ")) }
        };
        format!("`{}` rides {} and `{}` {}: what one reading carries rides on one path of frames",
                name_of(res, a), on(a), name_of(res, b), on(b))
    })
}

/// **A reading that leaves `under:` out** (#383): the motion of what `tool` rides on, seen from
/// the frame `seen_from` names where it names one — `Ok(None)` where `tool` rides on nothing,
/// which such a reading cannot carry.
pub(super) fn riding(sk: &mut Sketch, res: &Resolver, tool: EntRef, seen_from: Option<&Ref>)
    -> Result<Option<usize>, String> {
    let path = carriers(sk, res, tool)?;
    let Some(source) = carried(sk, &path)? else { return Ok(None) };
    let Some(g) = seen_from else { return Ok(Some(source)) };
    let Some(observer) = frame_motion(sk, res, g)? else { return Ok(Some(source)) };
    Ok(Some(relative(sk, source, observer)))
}

/// **Two flanks meshing, read on their frames** (#383): the second's frames seen from the
/// first's — or from the frame `seen_from` names — where the meshing leaves `under:` out.
pub(super) fn meshing(sk: &mut Sketch, res: &Resolver, first: EntRef, second: EntRef,
                      seen_from: Option<&Ref>) -> Result<usize, String> {
    let s = carriers(sk, res, second)?;
    let source = carried(sk, &s)?.ok_or_else(|| format!("`{}` rides on no frame that moves: \
        a meshing names its second flank's frame `under:`, or the flank is drawn in it",
        name_of(res, second)))?;
    let observer = match seen_from {
        Some(g) => frame_motion(sk, res, g)?,
        None => {
            let o = carriers(sk, res, first)?;
            carried(sk, &o)?
        }
    };
    Ok(match observer {
        Some(o) => relative(sk, source, o),
        None => source,
    })
}

/// **What a reading of `tool` is under** (#383): the frame `under` names, seen from the one
/// `seen_from` names (refused where `tool` rides already), or with `under:` left out what `tool`
/// rides on — refused where it rides on nothing that moves.
pub(super) fn reading_of(sk: &mut Sketch, res: &Resolver, tool: Option<EntRef>,
                         under: Option<&Ref>, seen_from: Option<&Ref>) -> Result<usize, String> {
    let Some(tool) = tool else {
        return match under {
            Some(u) => reading(sk, res, u, seen_from),
            None => Err("a reading names the frame it is under, `under: f`".into()),
        };
    };
    match under {
        Some(u) => match under_riding(sk, res, tool) {
            Some(why) => Err(why),
            None => reading(sk, res, u, seen_from),
        },
        None => riding(sk, res, tool, seen_from)?.ok_or_else(|| format!(
            "`{}` rides on no frame that moves: a reading of it names one, `under: f`, or it is \
             drawn `in` the frame that carries it", name_of(res, tool))),
    }
}

/// **`under:` names the frame of an operand that rides on none** (#383): one drawn riding already
/// is read where it rides, its `under:` left out — what a reading made rides on its observer and
/// may be read under another frame.  Why, for `tool` riding.
pub(super) fn under_riding(sk: &Sketch, res: &Resolver, tool: EntRef) -> Option<String> {
    let path = sk.drawn_carriers(tool).ok().filter(|p| !p.is_empty())?;
    let frames: Vec<String> = path.iter()
        .map(|&f| format!("`{}`", name_of(res, EntRef::frame(f as usize)))).collect();
    Some(format!("`{}` rides on {} already: a reading of it leaves `under:` out, and `under:` \
                  names the frame of what rides on none", name_of(res, tool),
                 frames.join(" in ")))
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
                              class: Default::default(), numbers: None });
    sk.motions.len() - 1
}
