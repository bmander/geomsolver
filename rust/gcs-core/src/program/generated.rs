//! Planar envelopes: `cut := envelope(tool, under: m, from: a, to: b)` over a tool of the sheet
//! and a planar motion, compiled to a curve of the drawing (`generate.rs`, spec §6.15.1).
//!
//! Built once every primitive and motion is, beside the curves and before any constraint, since
//! a contact names the curve.  An envelope of a *surface* is the spatial one (§6.15), left to
//! `envelopes` after the solve; the operand's kind is what tells the two apart.
use super::{resolve::{follow_building, Resolver}, Code, Diag, Made, SourceMap};
use crate::generate::{self, Generated};
use crate::ir::{Kid, Operation, Statement};
use crate::model::{CurveBody, CurveDef, CurveE, EntKind, EntRef, Home, MotionDef, Sketch};
use crate::syntax::{Arg, StmtId};
use crate::units::Dim;
use std::collections::{BTreeMap, BTreeSet};

/// The kinds a planar envelope's tool may be.
fn tool_code(sk: &Sketch, e: EntRef) -> Result<f64, String> {
    Ok(match e.kind {
        EntKind::Point => generate::TOOL_POINT,
        EntKind::Line => generate::TOOL_LINE,
        EntKind::Circle => generate::TOOL_CIRCLE,
        EntKind::Arc => generate::TOOL_ARC,
        EntKind::Curve => match &sk.curve_defs[sk.curves[e.i()].def as usize].body {
            CurveBody::Exprs { .. } => generate::TOOL_CURVE,
            // a profile cut by a line, circle, arc or point: exact to the third order, so what it
            // cuts in turn is exact to the second
            CurveBody::Envelope(g) if matches!(g.flat.get(1).copied(),
                Some(generate::TOOL_POINT | generate::TOOL_LINE | generate::TOOL_CIRCLE | generate::TOOL_ARC))
                => generate::TOOL_ENVELOPE,
            CurveBody::Envelope(_) => return Err("a generated curve cuts in its turn only when its own \
                                                  tool is a point, line, circle or arc".into()),
            CurveBody::Trace(_) => return Err("a tool curve is one written as a computed point or \
                                               generated; a traced curve cannot cut yet".into()),
        },
        k => return Err(format!("a {} cannot cut a profile in the plane", k.as_str())),
    })
}

/// The motion as a postfix program over the curve's columns and values, appending the geometry
/// it is written over to `args` and its numbers to `values`.  `base` is where the motion's
/// columns start in the outer vector, `vbase` where the values do — both known only once every
/// argument is, so the program is written with placeholders and fixed up by the caller.
fn program(sk: &Sketch, m: usize, ops: &mut Vec<(f64, Vec<Slot>)>, args: &mut Vec<EntRef>,
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
            ops.push((generate::OP_TURN, vec![Slot::Arg(args.len() - 1), Slot::Value(values.len()),
                                              Slot::Value(values.len() + 1)]));
            values.extend([ratio, phase]);
            shape.push('T');
        }
        MotionDef::Translation { axis, advance } => {
            args.push(EntRef::line(axis as usize));
            ops.push((generate::OP_SLIDE, vec![Slot::Arg(args.len() - 1), Slot::Value(values.len())]));
            values.push(advance);
            shape.push('S');
        }
        MotionDef::Relative { source, observer } => {
            shape.push('(');
            program(sk, source as usize, ops, args, values, shape, depth + 1)?;
            program(sk, observer as usize, ops, args, values, shape, depth + 1)?;
            ops.push((generate::OP_RELATIVE, Vec::new()));
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

/// Where an op reads: an argument's first column, or a value.
#[derive(Clone, Copy)]
enum Slot {
    Arg(usize),
    Value(usize),
}

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
        // a surface's envelope is the spatial one, built after the solve
        let tool = match operand(0) {
            Some(Ok(e)) if !matches!(e.kind, EntKind::Surface | EntKind::Patch | EntKind::Envelope) => e,
            _ => continue,
        };
        made.insert(st.id);
        let motion_ref = operand(1);
        let built = (|| -> Result<CurveE, String> {
            let motion = match motion_ref {
                Some(Ok(e)) if e.kind == EntKind::Motion && e.i() < sk.motions.len() => e.i(),
                _ => return Err("an envelope is generated `under:` a motion".into()),
            };
            let code = tool_code(sk, tool)?;
            let span = d.angular_span.as_ref().ok_or("an envelope needs `from:` and `to:` rolls")?;
            let roll = |a: &Arg| -> Result<f64, String> {
                let Arg::Dim { text, .. } = a else { return Err("a roll is an angle".into()) };
                let v = crate::flatten::value_aff(text, &BTreeMap::new(), sk.units)?;
                v.dim.require(Dim::ANGLE, "envelope roll")?;
                v.number().filter(|x| x.is_finite()).ok_or_else(|| "a roll must be a number".to_string())
            };
            let (from, to) = (roll(&span.from)?, roll(&span.to)?);
            if from >= to {
                return Err("an envelope needs increasing rolls".into());
            }
            let side = match span.side.as_ref().map(|n| n.text.as_str()) {
                None | Some("near") => 1.0,
                Some("far") => -1.0,
                Some(w) => return Err(format!("`side: {w}` — an envelope's side is `near` or `far` of the instant centre")),
            };
            let mut named = vec![tool];
            let (mut ops, mut values, mut shape) = (Vec::new(), Vec::new(), String::new());
            program(sk, motion, &mut ops, &mut named, &mut values, &mut shape, 0)?;
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
            if 1 + n_theta > crate::tape::MAX_VARS || 1 + n_theta + values.len() > generate::OUTER_MAX {
                return Err(format!("an envelope is written over at most {} coordinates; this tool \
                                    and motion are over {n_theta}", crate::tape::MAX_VARS - 1));
            }
            let mut flat = vec![n_theta as f64, code, side, ops.len() as f64];
            for (op, slots) in &ops {
                flat.push(*op);
                flat.extend(slots.iter().map(|s| match *s {
                    Slot::Arg(i) => at[i] as f64,
                    Slot::Value(i) => (1 + n_theta + i) as f64,
                }));
            }
            let mut key = format!("envelope:{code}:{shape}:{side}");
            if code == generate::TOOL_CURVE {
                let cv = &sk.curves[tool.i()];
                let td = &sk.curve_defs[cv.def as usize];
                let CurveBody::Exprs { x, y } = &td.body else { unreachable!("checked by tool_code") };
                let n_t = sk.entity_params(tool).len();
                let (a, b) = sk.curve_domain(tool.i());
                flat.extend([td.vars.len() as f64, n_t as f64, x.flat.len() as f64, y.flat.len() as f64]);
                flat.extend_from_slice(&x.flat);
                flat.extend_from_slice(&y.flat);
                flat.push(cv.values.len() as f64);
                flat.extend_from_slice(&cv.values);
                flat.extend([a.min(b), a.max(b)]);
                // the tool's constants and interval are baked in: one definition per tool curve
                key.push_str(&format!(":{}", tool.i()));
            }
            if code == generate::TOOL_ENVELOPE {
                let cv = &sk.curves[tool.i()];
                let CurveBody::Envelope(g) = &sk.curve_defs[cv.def as usize].body else {
                    unreachable!("checked by tool_code")
                };
                let n_t = sk.entity_params(tool).len();
                let (a, b) = sk.curve_domain(tool.i());
                flat.extend([n_t as f64, cv.values.len() as f64]);
                flat.extend_from_slice(&cv.values);
                flat.extend([sk.curve_home(tool.i()), a.min(b), a.max(b), g.flat.len() as f64]);
                flat.extend_from_slice(&g.flat);
                key.push_str(&format!(":{}", tool.i()));
            }
            let def = match sk.curve_defs.iter().position(|x| x.name == key) {
                Some(i) if code != generate::TOOL_CURVE && code != generate::TOOL_ENVELOPE => i,
                _ => {
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
                        body: CurveBody::Envelope(Generated { flat }),
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
            })
        })();
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
    made
}
