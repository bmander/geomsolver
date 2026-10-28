//! **Frames' joints** (#383, `docs/frames-plan.md`): a frame's motion is three members —
//! `f.axis`, `f.ratio`, `f.advance` — held by relations like anything else, and lowered here to the
//! motion node every reader of a motion reads (`Sketch::motions`).  What kind of motion a joint is
//! has to be known before the solve by the readers that build with the drawing (a profile in the
//! plane, an envelope the solve holds), so it is read off **the statements holding its axis**, as
//! `views::origins_on_axes` reads which axes are wholly held: the axis on a drawn line is a turn
//! about that line (or with a ratio stated 0 a slide along it); through a point of a view and
//! parallel to the view's normal, a turn in that view about the point; anything else a joint about
//! the axis as the solve leaves it, which only the readers after the solve take.  Its numbers are
//! what a `fix` states (which `std.Turn`, `std.Spin` and `std.Slide` write), and a number none
//! states is the solve's to find (`MotionE::numbers`).
//!
//! A reading names a frame `under:`, or leaves it out for what its tool rides on, and may name the
//! frame it is seen from, `seen_from:` (`reading_of`): the relative of the two frames' nodes,
//! minted once.
use super::resolve::Resolver;
use super::{Code, Diag, SourceMap};
use crate::ir::{Operation, Statement};
use crate::model::{entity_name, EntKind, EntRef, MotionDef, MotionE, Sketch};
use crate::syntax::{Arg, OpArg, Ref, StmtId};
use std::borrow::Cow;
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
    /// The path the axis is held along, and the statement saying so (#398 C).
    guide: Option<(EntRef, StmtId)>,
}

/// Every jointed frame's motion node, read off the statements that hold it.
pub(super) fn motions(sk: &mut Sketch, res: &mut Resolver, map: &SourceMap, body: &[&Statement],
                      skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    for f in 0..sk.frames.len() {
        let Some(joint) = sk.frames[f].joint.clone() else { continue };
        let frame = EntRef::frame(f);
        let axis = EntRef::new(EntKind::Axis, joint.axis as usize);
        let name = map.name_of(frame).cloned().unwrap_or_else(|| entity_name(frame));
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
                // the joint's words hold its axis the way the line it is held to runs (#535)
                "coincident" | "parallel" if w.ops.len() == 2 && w.args.iter().all(|a| {
                    matches!(a, OpArg::Named(k, _) if k.text == "sense")
                }) => {
                    let (a, b) = (ent(&w.ops[0]), ent(&w.ops[1]));
                    let other = match (a, b) {
                        (Some(a), Some(b)) if a == axis => b,
                        (Some(a), Some(b)) if b == axis => a,
                        _ => continue,
                    };
                    let path = matches!(other.kind, EntKind::Arc | EntKind::Spline | EntKind::Curve)
                        || super::relations::traced_curve(sk, other);
                    match (w.word.text.as_str(), other.kind) {
                        // an axis held along a path: the frame rides along it (#398 C)
                        ("coincident", _) if path => held.guide = Some((other, st.id)),
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
        // **a frame along a path** (#398 C): its pose read off the path, turning with none of
        // its torsion (ratio 0) or all of it (1), its axis the path's tangent and no number of
        // its own — what the axis is held to states no row
        if let Some((guide, stmt)) = held.guide {
            res.guides.insert(stmt);
            if held.ratio.is_none() || held.advance.is_none() || advance != 0.0 {
                say(diags, format!("`{name}` rides along a path, which says where it stands: its \
                    joint states its ratio, 0 or 1, and no advance — \
                    `std.stations.Along(f, g, up: u)`"));
                continue;
            }
            if ratio != 0.0 && ratio != 1.0 {
                say(diags, format!("`{name}` turns {} of its path's torsion: a frame along a path \
                    turns with none of it (ratio 0, the rotation-minimising frame) or all of it \
                    (ratio 1, the path's own)", crate::syntax::num(ratio)));
                continue;
            }
            let ax = sk.axes[joint.axis as usize].clone();
            for q in ax.a.into_iter().chain(ax.d) {
                sk.params[q as usize].fixed = true;
            }
            sk.motions.push(MotionE { def: MotionDef::Guided { guide, frame: f as u32,
                                                               frenet: ratio == 1.0 },
                                      name: name.clone(), class: sk.frames[f].class.clone(),
                                      numbers: None });
            let m = sk.motions.len() - 1;
            if let Some(j) = sk.frames[f].joint.as_mut() {
                j.motion = Some(m as u32);
            }
            continue;
        }
        let def = match (held.line, in_view) {
            (Some(l), _) if ratio == 0.0 && held.ratio.is_some() => {
                MotionDef::Translation { axis: l as u32, advance }
            }
            (Some(l), _) => MotionDef::Rotation { axis: l as u32, ratio, advance },
            (None, Some(_)) if advance != 0.0 || held.advance.is_none() => {
                say(diags, format!("`{name}` turns in a view about a point of it, and a turn in a \
                    view has no advance: a screw turns about a line, `f screws(a) l`"));
                continue;
            }
            (None, Some(c)) => MotionDef::Turn { centre: c as u32, ratio },
            (None, None) => MotionDef::Joint { axis: joint.axis, ratio, advance },
        };
        let numbers = free.then_some([joint.ratio, joint.advance]);
        sk.motions.push(MotionE { def, name: name.clone(), class: sk.frames[f].class.clone(),
                                  numbers });
        let m = sk.motions.len() - 1;
        if let Some(j) = sk.frames[f].joint.as_mut() {
            j.motion = Some(m as u32);
        }
        // a value written over a declared parameter drives the joint (#391 S5): minted beside it,
        // the node every reading over the roll reads
        if let Some((law, over)) = res.values.get(&f).and_then(|(text, _)| {
            Some((text.clone(), runs(sk, text).into_iter().next()?))
        }) {
            sk.motions.push(MotionE { def: MotionDef::Driven { of: m as u32, law, over },
                                      name: format!("{name} driven"),
                                      class: sk.frames[f].class.clone(), numbers: None });
        }
    }
}

/// **The parameters a joint's value runs over** (#391 S5): the declared ones its text reads, in
/// name order — the one that drives it, as `values` refuses one reading two.
fn runs(sk: &Sketch, text: &str) -> Vec<String> {
    crate::expr::parse_in(text, sk.units).map(|p| p.body.deps()).unwrap_or_default().into_iter()
        .filter(|d| sk.parameters.contains_key(d)).collect()
}

/// The node a reading of joint node `m` reads: the drive minted beside it, where its frame's value
/// drives it, else `m` itself.
fn driven(sk: &Sketch, m: usize) -> usize {
    sk.motions.iter()
        .position(|x| matches!(x.def, MotionDef::Driven { of, .. } if of as usize == m))
        .unwrap_or(m)
}

/// The joint a drive was minted beside (#391 S5), or `m` where it is no drive: what `x in f`
/// reads, at the value worked out where the drawing stands, so a row reads no law.
pub(super) fn undriven(sk: &Sketch, m: usize) -> usize {
    match sk.motions[m].def {
        MotionDef::Driven { of, .. } => of as usize,
        _ => m,
    }
}

/// **The drive a motion node reads**, through relatives and inverses: the parameter it is driven
/// by, which a relation the solve holds reads only as a point carried to a roll (#391 S5, #398 G).
pub(super) fn drive_in(sk: &Sketch, m: usize) -> Option<(String, String)> {
    read_in(sk, m, &|node| match &node.def {
        MotionDef::Driven { of, over, .. } => Some((sk.motions[*of as usize].name.clone(),
                                                    over.clone())),
        _ => None,
    })
}

/// The first node `m` reads, through relatives and inverses, that `leaf` answers for.
fn read_in(sk: &Sketch, m: usize, leaf: &dyn Fn(&MotionE) -> Option<(String, String)>)
    -> Option<(String, String)> {
    let node = sk.motions.get(m)?;
    match &node.def {
        MotionDef::Relative { source, observer } => read_in(sk, *source as usize, leaf)
            .or_else(|| read_in(sk, *observer as usize, leaf)),
        MotionDef::Inverse { of } => read_in(sk, *of as usize, leaf),
        _ => leaf(node),
    }
}

/// **A joint's value** (#391 S3): `fix(value == τ) f` states the instant of the shared roll at
/// which the jointed frame `f` stands — its joint turned `ratio · τ` and risen `advance · τ / 360°`
/// from where it is drawn, so the value is linear in `τ` with the joint's ratio and advance as
/// its slopes, and `0deg`, the joint at rest, where none is stated.  It is an angle worked out
/// as a dimension is (the flattener settled it), so it may read a face family's running number
/// and is read again at every station, or name an unknown the solve finds.  Recorded by the frame
/// (`Resolver::values`) for what is drawn in it (`n := x in f`) to be read at, and no number the
/// `fix` holds (`holding`).  Refused: a value of anything but a jointed frame, one given twice,
/// a frame with a value riding on another, and a frame with a value something rides on — what is
/// drawn in its block stands where it is drawn, not where the value puts it, so the frame draws
/// it as `n := base in f` instead.  A value reading a declared parameter is a law of it, which
/// drives the frame (#391 S5, minted by `motions`), and reads one parameter alone.
pub(super) fn values(sk: &Sketch, res: &mut Resolver, body: &[&Statement],
                     skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    use crate::constraints::JOINT_VALUE;
    let held = sk.held_numbers();
    for st in body.iter().filter(|st| !skip.contains(&st.id)) {
        let Operation::Relation(r) = &st.kind else { continue };
        let Some(w) = r.form.written().filter(|w| w.word.text == "fix") else { continue };
        let value = w.args.iter().find_map(|a| match a {
            OpArg::Slot { key, arg: Arg::Dim { text, span } } if key.text == JOINT_VALUE => {
                Some((text.clone(), *span))
            }
            _ => None,
        });
        let Some((text, span)) = value else { continue };
        let say = |diags: &mut Vec<Diag>, code: Code, message: String| {
            diags.push(Diag::within(code, span, st.id, message));
        };
        let frame = w.ops.first().and_then(|o| res.entity(sk, o));
        let Some(f) = frame.filter(|e| e.kind == EntKind::Frame && sk.frames[e.i()].joint.is_some())
        else {
            let name = w.ops.first().map(crate::syntax::ref_text).unwrap_or_default();
            say(diags, Code::E080, format!("a value is a jointed frame's, the instant it stands \
                at: `{name}` has no joint — `f turns(r) l`, `f spins(r) o`, \
                `f slides(a) l`"));
            continue;
        };
        let name = name_of(res, f);
        if r.claim {
            say(diags, Code::E040, format!("`{name}`'s value is stated, not claimed"));
            continue;
        }
        if res.values.contains_key(&f.i()) {
            say(diags, Code::E040, format!("`{name}`'s value is given twice"));
            continue;
        }
        // an instant, whatever reads it: said once, here, rather than at every reading
        let angle = crate::flatten::value_read(&text, &held, &sk.declared, sk.units)
            .and_then(|a| a.dim.require(crate::units::Dim::ANGLE, JOINT_VALUE));
        if let Err(m) = angle {
            say(diags, Code::E103, format!("`{text}`: {m}"));
            continue;
        }
        // a drive runs over one parameter (#391 S5): the roll stands for one at a time
        let over = runs(sk, &text);
        if over.len() > 1 {
            say(diags, Code::E103, format!("`{text}` reads {}: a joint's value is driven by one \
                declared parameter, the one the roll stands for", over.iter()
                .map(|r| format!("`{r}`")).collect::<Vec<_>>().join(" and ")));
            continue;
        }
        let carriers = sk.carriers_of(&sk.frames[f.i()].rides);
        if !carriers.is_empty() {
            say(diags, Code::E080, format!("`{name}` rides on {}, and a frame with a value rides \
                on none yet (#391): its value is an instant of its own joint alone",
                path_names(res, &carriers)));
            continue;
        }
        // what rides on it stands where it is drawn, which its value would contradict
        let rides = |path: &[u32]| sk.carriers_of(path).contains(&(f.i() as u32));
        let riding = sk.rides.iter().find(|(_, path)| rides(path)).map(|(e, _)| *e)
            .or_else(|| (0..sk.frames.len()).find(|&g| rides(&sk.frames[g].rides))
                .map(EntRef::frame));
        if let Some(e) = riding {
            say(diags, Code::E080, format!("{} rides on `{name}`, which has a value: what is drawn \
                in its block stands where it is drawn, not yet where the value puts it (#391) — \
                draw a frame in it, `n := base in {name}`, and draw in that",
                match name_of(res, e) { n if n.starts_with('#') => "a point".into(),
                                        n => format!("`{n}`") }));
            continue;
        }
        res.values.insert(f.i(), (text, span));
    }
}

/// A frame's value `text` worked out where the drawing stands (`Sketch::held_numbers`), in
/// degrees: the instant a solid drawn in it is placed at.
pub(super) fn value_now(sk: &Sketch, text: &str) -> Result<f64, String> {
    let a = crate::flatten::value_read(text, &sk.held_numbers(), &sk.declared, sk.units)?;
    a.dim.require(crate::units::Dim::ANGLE, crate::constraints::JOINT_VALUE)?;
    a.number().ok_or_else(|| format!("`{text}` reads an unknown, and a solid is placed once"))
}

/// The relation a `fix` states of the numbers it holds (#391 S3): itself, or without a joint's
/// value, which holds none — `None` where the value was all it said.
pub(super) fn holding(r: &crate::ir::Relation) -> Option<Cow<'_, crate::ir::Relation>> {
    let Some(w) = r.form.written() else { return Some(Cow::Borrowed(r)) };
    let value = |a: &OpArg| matches!(a, OpArg::Slot { key, .. }
        if key.text == crate::constraints::JOINT_VALUE);
    if !w.args.iter().any(value) {
        return Some(Cow::Borrowed(r));
    }
    let mut w = w.clone();
    w.args.retain(|a| !value(a));
    if w.args.is_empty() {
        return None;
    }
    let mut out = r.clone();
    out.form = crate::syntax::RelationForm::Written(w);
    Some(Cow::Owned(out))
}

/// The instant a reading of something drawn in the frame `f` names is read at (#391 S3): its
/// value as stated, with where it was written, or the joint at rest, written nowhere — `None`
/// where `f` names no frame, which the reading refuses as it refuses any.
pub(super) fn value_of(sk: &Sketch, res: &Resolver, f: &Ref)
    -> Option<(String, crate::syntax::Span)> {
    let f = res.entity(sk, f).filter(|e| e.kind == EntKind::Frame)?;
    Some(res.values.get(&f.i()).cloned().unwrap_or_else(|| ("0deg".into(), Default::default())))
}

/// **A reading's motion** (#383): the frame named `under:`, seen from the one named `seen_from:`
/// where one is — its node, or the relative of the two (`n⁻¹·m`, minted once).  A frame seen from
/// one with no joint is seen from the world; one with no joint itself carries nothing, and is
/// refused naming how to turn it.  `over` is the declared parameter the reading runs over, where
/// it runs over one: a frame with no joint is then where the drawing puts it as that runs (#398 F,
/// `followed`).
pub(super) fn reading(sk: &mut Sketch, res: &Resolver, under: &Ref, seen_from: Option<&Ref>,
                      over: Option<&str>) -> Result<usize, String> {
    let source = frame_motion(sk, res, under, over)?.ok_or_else(|| {
        let name = crate::syntax::ref_text(under);
        format!("`{name}` does not move: a frame's motion is its joint, held by relations — \
                 `{name} turns(r) l`, `{name} spins(r) o`, `{name} slides(a) l` — or the \
                 drawing moves it as a declared parameter runs, read over it, `over t`")
    })?;
    let Some(g) = seen_from else { return Ok(source) };
    let Some(observer) = frame_motion(sk, res, g, over)? else { return Ok(source) };
    Ok(relative(sk, source, observer))
}

/// The motion node of the frame `r` names: its joint's after the joints of the frames it rides
/// on (#383); for a frame with no joint riding on none, read over the declared parameter `over`,
/// where the drawing puts it as `over` runs (#398 F) — and `None` for a frame nothing moves.
pub(super) fn frame_motion(sk: &mut Sketch, res: &Resolver, r: &Ref, over: Option<&str>)
    -> Result<Option<usize>, String> {
    let e = res.lookup(r).and_then(|e| super::resolve::follow_building(sk, res, e, r).ok())
        .ok_or_else(|| format!("no such frame: `{}`", crate::syntax::ref_text(r)))?;
    if e.kind != EntKind::Frame {
        return Err(format!("`{}` is {}, and a motion is read off a frame (`under: f`, `seen_from: \
            g`)", crate::syntax::ref_text(r), e.kind.a()));
    }
    let carriers = sk.carriers(e).unwrap_or_default();
    if carriers.is_empty() {
        if let Some(m) = over.and_then(|o| followed(sk, res, e.i(), o)) {
            return Ok(Some(m));
        }
    }
    carried(sk, &carriers)
}

/// **A frame the drawing moves** (#398 F): frame `f`, with axes and no joint, read over the
/// declared parameter `over` — where the drawing followed along `over` puts it, carried from where
/// it stands (`MotionDef::Followed`, minted once).  `None` for a frame with no axes, which has no
/// pose to move, a frame held outright (`std.front`), which nothing moves, and a name the document
/// declares no parameter by.
fn followed(sk: &mut Sketch, res: &Resolver, f: usize, over: &str) -> Option<usize> {
    let frame = sk.frames.get(f)?;
    if frame.joint.is_some() || frame.chart.is_none() || !sk.parameters.contains_key(over)
        || sk.plane_fixed(f) {
        return None;
    }
    let found = sk.motions.iter().position(|m| matches!(&m.def,
        MotionDef::Followed { frame, over: o } if *frame as usize == f && o == over));
    if let Some(i) = found {
        return Some(i);
    }
    let name = name_of(res, EntRef::frame(f));
    let class = sk.frames[f].class.clone();
    sk.motions.push(MotionE { def: MotionDef::Followed { frame: f as u32, over: over.to_string() },
                              name, class, numbers: None });
    Some(sk.motions.len() - 1)
}

/// The declared parameter a reading over an interval names, where it names one (`over t`): what
/// a frame the drawing moves is read over (#398 F).
pub(super) fn declared(over: Option<&crate::syntax::RunsOver>) -> Option<&str> {
    over.filter(|o| o.declared).map(|o| o.name.text.as_str())
}

/// The declared parameter a declaration's reading over an interval names, where it names one.
pub(super) fn declared_of(d: &crate::ir::Decl) -> Option<&str> {
    declared(d.angular_span.as_ref().and_then(|s| s.over.as_ref()))
}

/// The one parameter the document declares, where it declares one: what a reading at an instant
/// (`x at(under: f) τ`), which names none, reads a frame the drawing moves over (#398 F).
pub(super) fn sole(sk: &Sketch) -> Option<String> {
    let mut names = sk.parameters.keys();
    match (names.next(), names.next()) {
        (Some(n), None) => Some(n.clone()),
        _ => None,
    }
}

/// **A frame the drawing moves a motion node reads**, through relatives and inverses: the
/// parameter it is read over, which a reading over the roll reads as an angle (#398 F).
pub(super) fn followed_in(sk: &Sketch, m: usize) -> Option<(String, String)> {
    read_in(sk, m, &|node| match &node.def {
        MotionDef::Followed { over, .. } => Some((node.name.clone(), over.clone())),
        _ => None,
    })
}

/// **The motion of a path of carriers** (#383): their joints chained, outermost first —
/// `outer(τ)·inner(τ)`, `Sketch::chain`, each pair minted once — `None` for the world.
pub(super) fn carried(sk: &mut Sketch, carriers: &[u32]) -> Result<Option<usize>, String> {
    // a driven joint's readings read its drive (#391 S5)
    let nodes: Vec<usize> = carriers.iter()
        .filter_map(|&f| sk.frames.get(f as usize)?.joint.as_ref()?.motion)
        .map(|m| driven(sk, m as usize)).collect();
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
        .unwrap_or_else(|| entity_name(e))
}

/// The jointed frames `e` rides on, outermost first, or why its parts ride on different ones.
pub(super) fn carriers(sk: &Sketch, res: &Resolver, e: EntRef) -> Result<Vec<u32>, String> {
    sk.carriers(e).map_err(|(a, b)| {
        let on = |x: EntRef| {
            let path = sk.carriers(x).unwrap_or_default();
            if path.is_empty() { "on nothing".to_string() }
            else { format!("on {}", path_names(res, &path)) }
        };
        format!("`{}` rides {} and `{}` {}: what one reading carries rides on one path of frames",
                name_of(res, a), on(a), name_of(res, b), on(b))
    })
}

/// The frames of `path` named for a refusal, outermost first: "`f` in `g`".
fn path_names(res: &Resolver, path: &[u32]) -> String {
    let frames: Vec<String> = path.iter()
        .map(|&f| format!("`{}`", name_of(res, EntRef::frame(f as usize)))).collect();
    frames.join(" in ")
}

/// Motion node `source` seen from `observer`: the relative of the two, `source` alone where the
/// observer rests, the observer's inverse where the source rests, and `None` where both do.
pub(super) fn seen(sk: &mut Sketch, source: Option<usize>, observer: Option<usize>)
    -> Option<usize> {
    match (source, observer) {
        (Some(s), Some(o)) => Some(relative(sk, s, o)),
        (Some(s), None) => Some(s),
        (None, Some(o)) => Some(inverse(sk, o)),
        (None, None) => None,
    }
}

/// **A reading that leaves `under:` out** (#383): the motion of what `tool` rides on, seen from
/// the frame `seen_from` names where it names one — `Ok(None)` where `tool` rides on nothing,
/// which such a reading cannot carry.
pub(super) fn riding(sk: &mut Sketch, res: &Resolver, tool: EntRef, seen_from: Option<&Ref>,
                     over: Option<&str>) -> Result<Option<usize>, String> {
    let path = carriers(sk, res, tool)?;
    let Some(source) = carried(sk, &path)? else { return Ok(None) };
    let observer = match seen_from {
        Some(g) => frame_motion(sk, res, g, over)?,
        None => None,
    };
    Ok(seen(sk, Some(source), observer))
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
        Some(g) => frame_motion(sk, res, g, None)?,
        None => {
            let o = carriers(sk, res, first)?;
            carried(sk, &o)?
        }
    };
    Ok(seen(sk, Some(source), observer).unwrap_or(source))
}

/// **What a reading of `tool` is under** (#383): the frame `under` names, seen from the one
/// `seen_from` names (refused where `tool` rides already), or with `under:` left out what `tool`
/// rides on — refused where it rides on nothing that moves.
pub(super) fn reading_of(sk: &mut Sketch, res: &Resolver, tool: Option<EntRef>,
                         under: Option<&Ref>, seen_from: Option<&Ref>, over: Option<&str>)
    -> Result<usize, String> {
    let m = read_under(sk, res, tool, under, seen_from, over)?;
    // the roll is an angle, and a drive (#391 S5) or a frame the drawing moves (#398 F) read over
    // it runs over one
    let what = |sk: &Sketch, over: &str| {
        match sk.parameters.get(over).map(|p| p.dim) {
            Some(crate::units::Dim::ANGLE) => None,
            dim => Some(dim.map_or("no parameter".into(), |d| format!("a {}", d.name()))),
        }
    };
    if let Some((f, over)) = drive_in(sk, m) {
        if let Some(dim) = what(sk, &over) {
            return Err(format!("`{f}` is driven by `{over}`, {dim}, and a reading over the roll \
                runs over an angle: draw in it, `n := x in {f}`, to follow it along `{over}`"));
        }
    }
    if let Some((f, over)) = followed_in(sk, m) {
        if let Some(dim) = what(sk, &over) {
            return Err(format!("`{f}` stands where the drawing puts it as `{over}` runs, {dim}, \
                and a reading over the roll runs over an angle"));
        }
    }
    Ok(m)
}

fn read_under(sk: &mut Sketch, res: &Resolver, tool: Option<EntRef>, under: Option<&Ref>,
              seen_from: Option<&Ref>, over: Option<&str>) -> Result<usize, String> {
    let Some(tool) = tool else {
        return match under {
            Some(u) => reading(sk, res, u, seen_from, over),
            None => Err("a reading names the frame it is under, `under: f`".into()),
        };
    };
    match under {
        Some(u) => match under_riding(sk, res, tool) {
            Some(why) => Err(why),
            None => reading(sk, res, u, seen_from, over),
        },
        None => riding(sk, res, tool, seen_from, over)?.ok_or_else(|| format!(
            "`{}` rides on no frame that moves: a reading of it names one, `under: f`, or it is \
             drawn `in` the frame that carries it", name_of(res, tool))),
    }
}

/// **`under:` names the frame of an operand that rides on none** (#383): one drawn riding already
/// is read where it rides, its `under:` left out — what a reading made rides on its observer and
/// may be read under another frame.  Why, for `tool` riding.
pub(super) fn under_riding(sk: &Sketch, res: &Resolver, tool: EntRef) -> Option<String> {
    let path = sk.drawn_carriers(tool).ok().filter(|p| !p.is_empty())?;
    Some(format!("`{}` rides on {} already: a reading of it leaves `under:` out, and `under:` \
                  names the frame of what rides on none", name_of(res, tool),
                 path_names(res, &path)))
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
