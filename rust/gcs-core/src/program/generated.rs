//! Planar envelopes: `cut := envelope(tool, under: m, from: a, to: b)` over a tool of the sheet
//! and a planar motion, compiled to a curve of the drawing (`generate.rs`, spec §6.15.1).
//!
//! Built once every primitive and motion is, beside the curves and before any constraint, since
//! a contact names the curve.  An envelope of a spatial operand (a surface) is the spatial one
//! (§6.15), left to `envelopes` after the solve: the operand's stratum is what tells them apart.
use super::{resolve::{follow_building, Resolver}, Code, Diag, Made, SourceMap};
use crate::generate::{Generated, Op, Tool, ToolBody};
use crate::ir::{Decl, Kid, Operation, Statement};
use crate::model::{CurveBody, CurveDef, CurveE, EntKind, EntRef, Home, MotionDef, Sketch};
use crate::syntax::StmtId;
use std::collections::BTreeSet;

/// What a tool of the sheet cuts as.
fn tool_of(sk: &Sketch, e: EntRef) -> Result<Tool, String> {
    Ok(match e.kind {
        EntKind::Point => Tool::Point,
        EntKind::Line => Tool::Line,
        EntKind::Circle => Tool::Circle,
        EntKind::Arc => Tool::Arc,
        EntKind::Curve => match &sk.curve_defs[sk.curves[e.i()].def as usize].body {
            CurveBody::Exprs { .. } => Tool::Curve,
            // a profile cut by a line, circle, arc or point: exact to the third order, so what it
            // cuts in turn is exact to the second
            CurveBody::Envelope(g) if g.tool().is_some_and(Tool::analytic) => Tool::Envelope,
            CurveBody::Envelope(_) => return Err("a generated curve cuts in its turn only when its own \
                                                  tool is a point, line, circle or arc".into()),
            CurveBody::Trace(_) => return Err("a tool curve is one written as a computed point or \
                                               generated; a traced curve cannot cut yet".into()),
        },
        k => return Err(format!("{} cannot cut a profile in the plane", k.a())),
    })
}

/// A step of the motion program before its columns are placed: which argument or value each
/// number is read from.
enum Step {
    Turn { centre: usize, ratio: usize, phase: usize },
    Slide { line: usize, advance: usize },
    Relative,
}

/// The motion as a postfix program, appending the geometry it is written over to `args`, its
/// numbers to `values`, and its shape (for the definition's key) to `shape`.
fn program(sk: &Sketch, m: usize, steps: &mut Vec<Step>, args: &mut Vec<EntRef>,
           values: &mut Vec<f64>, shape: &mut String, depth: usize) -> Result<(), String> {
    if depth > 16 {
        return Err("a planar motion nests too deeply".into());
    }
    let me = sk.motions.get(m).ok_or("no such motion")?;
    if !me.measured.is_empty() {
        return Err(format!("`{}`'s numbers are measured after the solve, and a profile in the \
                            plane is generated within it: state them", me.name));
    }
    match me.def {
        MotionDef::Turn { centre, ratio, phase } => {
            args.push(EntRef::point(centre as usize));
            steps.push(Step::Turn { centre: args.len() - 1, ratio: values.len(), phase: values.len() + 1 });
            values.extend([ratio, phase]);
            shape.push('T');
        }
        MotionDef::Translation { axis, advance } => {
            args.push(EntRef::line(axis as usize));
            steps.push(Step::Slide { line: args.len() - 1, advance: values.len() });
            values.push(advance);
            shape.push('S');
        }
        MotionDef::Relative { source, observer } => {
            shape.push('(');
            program(sk, source as usize, steps, args, values, shape, depth + 1)?;
            program(sk, observer as usize, steps, args, values, shape, depth + 1)?;
            steps.push(Step::Relative);
            shape.push(')');
        }
        MotionDef::Rotation { .. } => {
            return Err(format!("`{}` turns about a line in space: a tool of the sheet is carried \
                                by a turn about a point of its view (a surface's envelope is \
                                the spatial one)", me.name))
        }
    }
    Ok(())
}

/// The curve a tool of the sheet cuts under motion `motion`, over the rolls and on the side the
/// envelope's declaration `d` states.
fn profile(sk: &mut Sketch, d: &Decl, tool: EntRef, motion: usize) -> Result<CurveE, String> {
    let kind = tool_of(sk, tool)?;
    let span = d.angular_span.as_ref().ok_or("an envelope needs `from:` and `to:` rolls")?;
    let (from, to) = (super::bound_angle(sk, &span.from, "envelope roll")?,
                      super::bound_angle(sk, &span.to, "envelope roll")?);
    if from >= to {
        return Err("an envelope needs increasing rolls".into());
    }
    let side = match span.side.as_ref().map(|n| n.text.as_str()) {
        None | Some("near") => 1.0,
        Some("far") => -1.0,
        Some(w) => return Err(format!("`side: {w}` — an envelope's side is `near` or `far` of the instant centre")),
    };
    let mut named = vec![tool];
    let (mut steps, mut values, mut shape) = (Vec::new(), Vec::new(), String::new());
    program(sk, motion, &mut steps, &mut named, &mut values, &mut shape, 0)?;
    // where each argument's columns start in `[t, θ…, values…]` — a centre the tool is
    // already written over (a generated tool's own pinion) is read where it already is,
    // not given a second column
    let mut cols: Vec<u32> = Vec::new();
    let (mut args, mut at) = (Vec::new(), Vec::with_capacity(named.len()));
    for &a in &named {
        let ps = sk.entity_params(a);
        let found = (0..cols.len().saturating_sub(ps.len() - 1))
            .find(|&k| !ps.is_empty() && cols[k..].starts_with(&ps));
        match found {
            Some(k) if !args.is_empty() => at.push(1 + k),
            _ => {
                at.push(1 + cols.len());
                cols.extend(&ps);
                args.push(a);
            }
        }
    }
    let n_theta = cols.len();
    if 1 + n_theta > crate::tape::MAX_VARS || 1 + n_theta + values.len() > crate::generate::OUTER_MAX {
        return Err(format!("an envelope is written over at most {} coordinates; this tool \
                            and motion are over {n_theta}", crate::tape::MAX_VARS - 1));
    }
    let value = |i: usize| 1 + n_theta + i;
    let ops: Vec<Op> = steps.iter().map(|s| match *s {
        Step::Turn { centre, ratio, phase } =>
            Op::Turn { centre: at[centre], ratio: value(ratio), phase: value(phase) },
        Step::Slide { line, advance } => Op::Slide { line: at[line], advance: value(advance) },
        Step::Relative => Op::Relative,
    }).collect();
    // a curve or generated tool carries its own constants and interval, so its curve is
    // its own: definitions are shared only between analytic tools
    let tool_body = match kind {
        Tool::Curve | Tool::Envelope => {
            let cv = &sk.curves[tool.i()];
            let (a, b) = sk.curve_domain(tool.i());
            let domain = (a.min(b), a.max(b));
            let n_theta = sk.entity_params(tool).len();
            match &sk.curve_defs[cv.def as usize].body {
                CurveBody::Exprs { x, y } =>
                    ToolBody::Curve { n_theta, x: &x.flat, y: &y.flat, values: &cv.values, domain },
                CurveBody::Envelope(g) => ToolBody::Envelope {
                    n_theta, flat: &g.flat, values: &cv.values, anchor: sk.curve_home(tool.i()), domain,
                },
                CurveBody::Trace(_) => unreachable!("refused by tool_of"),
            }
        }
        _ => ToolBody::None,
    };
    let generated = Generated::new(n_theta, kind, side, &ops, tool_body);
    let key = format!("envelope:{kind:?}:{shape}:{side}");
    let shared = kind.analytic().then(|| sk.curve_defs.iter().position(|x| x.name == key)).flatten();
    let def = match shared {
        Some(i) => i,
        None => {
            let mut vars = vec!["t".to_string()];
            vars.extend((0..n_theta).map(|k| format!("θ{k}")));
            vars.extend((0..values.len()).map(|k| format!("v{k}")));
            sk.curve_defs.push(CurveDef {
                name: key,
                component: "envelope".into(),
                port: d.name.key().text.clone(),
                formals: args.iter().enumerate().map(|(k, a)| (format!("a{k}"), a.kind)).collect(),
                columns: Vec::new(),
                values: (0..values.len()).map(|k| format!("v{k}")).collect(),
                param: "t".into(),
                turns: true,
                vars,
                body: CurveBody::Envelope(generated),
                pose_of: Vec::new(),
            });
            sk.curve_defs.len() - 1
        }
    };
    Ok(CurveE {
        def: def as u32,
        args,
        unknowns: Vec::new(),
        values,
        domain: (from, to),
        home: Home::At(from),
        pose: Vec::new(),
        class: d.class.clone(),
        trim: None,
        extrusion: false,
    })
}

/// Each planar envelope, built as a curve; returns the statements built, which the spatial
/// pass skips.
pub(super) fn planar_envelopes(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) -> BTreeSet<StmtId>
{
    let mut made = BTreeSet::new();
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Envelope || skip.contains(&st.id) {
            continue;
        }
        let operand = |i: usize| -> Option<Result<EntRef, String>> {
            let [Kid::Ref(r)] = d.children.get(i)?.as_slice() else { return None };
            let e = res.lookup(r)?;
            Some(follow_building(sk, res, e, r))
        };
        // an envelope of a spatial operand is the spatial one, built after the solve
        let tool = match operand(0) {
            Some(Ok(e)) if !e.kind.spatial() => e,
            _ => continue,
        };
        made.insert(st.id);
        let motion_ref = operand(1);
        let built = match motion_ref {
            Some(Ok(e)) if e.kind == EntKind::Motion && e.i() < sk.motions.len() => profile(sk, d, tool, e.i()),
            _ => Err("an envelope is generated `under:` a motion".into()),
        };
        bind(sk, res, map, st, d, built, diags);
    }
    made
}

/// An envelope built as a curve, bound to its name; or why not, said at its statement.
fn bind(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap, st: &Statement, d: &Decl,
        built: Result<CurveE, String>, diags: &mut Vec<Diag>) {
    match built {
        Ok(cv) => {
            let e = EntRef::new(EntKind::Curve, sk.curves.len());
            sk.curves.push(cv);
            res.of.insert(d.name.key().text.clone(), e);
            map.bind(&d.name.key().text, e, d.name.named());
            map.record(st, Made::Ent(e));
        }
        Err(message) => {
            diags.push(Diag { code: Code::E080, span: st.span, stmt: Some(st.id), message });
            res.of.remove(&d.name.key().text);
        }
    }
}

/// Each envelope of a prism's side under a motion keeping the prism's view, built as the curve it
/// stands for (issue #70): `flank := envelope(side, under: m, …)` with `side :=
/// surface(prism, edge: e)` is the surface `e` sweeps extruded square to the view, so it is the
/// planar envelope of `e` (`profile`) marked an extrusion, which a point in space is
/// `coincident` with by its place in the view.  Built with the drawing — before any relation,
/// so a contact may name it — where the surface and the prism are not built yet: the prism is
/// read off its declaration (a face swept by `depth:` or `from:`/`to:`), and the surface checks
/// the edge is a side of it when it is built.  Run once memberships are in, since the view is read off them.  Returns the
/// statements built, which the spatial pass skips.
pub(super) fn extruded_envelopes(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) -> BTreeSet<StmtId>
{
    let decls: std::collections::BTreeMap<&str, &Decl> = body.iter().filter_map(|st| match &st.kind {
        Operation::Decl(d) => Some((d.name.key().text.as_str(), &**d)),
        _ => None,
    }).collect();
    let decl = |name: &str| decls.get(name).copied();
    let mut made = BTreeSet::new();
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Envelope || skip.contains(&st.id) {
            continue;
        }
        let [Kid::Ref(surface)] = d.children.first().map(Vec::as_slice).unwrap_or_default() else { continue };
        let Some(sd) = decl(&surface.root.text).filter(|sd| sd.kind == EntKind::Surface && surface.path.is_empty()) else {
            continue;
        };
        let (Some([Kid::Ref(solid)]), Some([Kid::Ref(edge)])) =
            (sd.children.first().map(Vec::as_slice), sd.children.get(1).map(Vec::as_slice)) else { continue };
        let prism = decl(&solid.root.text).is_some_and(|x| x.kind == EntKind::Solid && solid.path.is_empty()
            && matches!(x.sweep, Some(crate::syntax::Sweep::Depth { .. } | crate::syntax::Sweep::Prism { .. })));
        let motion = match d.children.get(1).map(Vec::as_slice) {
            Some([Kid::Ref(r)]) => res.lookup(r).filter(|e| e.kind == EntKind::Motion && e.i() < sk.motions.len()),
            _ => None,
        };
        let (true, Some(motion)) = (prism, motion) else { continue };
        // a turn about a line in space keeps no view: the spatial envelope's, after the solve
        if program(sk, motion.i(), &mut Vec::new(), &mut Vec::new(), &mut Vec::new(), &mut String::new(), 0).is_err() {
            continue;
        }
        made.insert(st.id);
        let built = (|| -> Result<CurveE, String> {
            let tool = res.lookup(edge).ok_or_else(|| format!("no such entity: `{}`", edge.root.text))?;
            let tool = follow_building(sk, res, tool, edge)?;
            if !matches!(tool.kind, EntKind::Line | EntKind::Arc) || tool.i() >= sk.count(tool.kind) {
                return Err("a prism's side generates from an edge of its face, a line or an arc".into());
            }
            let mut cv = profile(sk, d, tool, motion.i())?;
            // the view the prism's face is in is the one the motion keeps: every point the curve
            // is written over stands in it, and it stands where it is drawn
            let view = super::planes::plane_of_entity(sk, tool);
            if cv.args.iter().any(|&a| super::planes::plane_of_entity(sk, a) != view) {
                return Err(format!("`{}` generates a surface where its motion keeps the prism's view: the motion's \
                    centres and lines are drawn in that view", d.name.key().text));
            }
            let Some(view) = view else {
                return Err("the prism's face is drawn in no plane: draw it `in` one".into());
            };
            if !sk.plane_fixed(view) {
                return Err("a prism's side generates in a plane that is fixed, not one solved for".into());
            }
            cv.extrusion = true;
            Ok(cv)
        })();
        bind(sk, res, map, st, d, built, diags);
    }
    made
}
