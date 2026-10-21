//! JSON (de)serialization of sketches.
//!
//! Entities are referenced by `[kind, index]` into the sketch's ordered lists; constraints
//! serialize their constructor arguments per `spec`.  Intrinsic constraints are not stored — the
//! primitives recreate them, and neither are soft ones (a drag target saved mid-drag would come
//! back as geometry the user never drew).

use crate::constraints::{seed_omitted, Arg, CKind, Constraint, SpecKind};
use crate::decompose;
use crate::expr::{self, reading, READING_SIG};
use crate::json::{fmt_g, object, parse, Json};
use crate::model::{entity_name, EntKind, EntRef, Sketch};
use crate::style::Classes;
use std::collections::BTreeSet;

/// The classes an entity carries.  Written as a list, and **only when there is one** — the key
/// is absent from an ordinary entity, so a document with no presentation in it exports none.
fn class_json(c: &Classes) -> Json {
    Json::Arr(c.0.iter().map(|s| Json::Str(s.clone())).collect())
}

/// The classes a stored entity carries, and the one key that is read and never written:
/// `"construction": true` loads as the class `construction`, so an export from before there
/// were classes still opens.  JSON is the export format, not the document.
fn read_class(v: &Json) -> Classes {
    if let Some(Json::Arr(a)) = v.get("class") {
        return Classes(a.iter().map(|x| x.as_str().to_string()).collect());
    }
    if v.get("construction").map(|x| x.as_bool()).unwrap_or(false) {
        return Classes::one("construction");
    }
    Classes::default()
}

fn ref_json(e: EntRef) -> Json {
    Json::Arr(vec![Json::Str(e.kind.as_str().to_string()), Json::Int(e.idx as i64)])
}

/// `ref_json`'s reader: `[kind, index]`, the index held to the entities of that kind `sk` has.
/// `what` names the reference in what is refused.
fn ref_from_json(sk: &Sketch, j: &Json, what: &str) -> Result<EntRef, String> {
    let a = j.arr();
    if a.len() != 2 {
        return Err(format!("{what} must be [kind, index]"));
    }
    let kind = EntKind::parse(a[0].as_str())
        .ok_or_else(|| format!("unknown {what} kind {:?}", a[0].as_str()))?;
    Ok(EntRef::new(kind, index(a[1].as_i64(), sk.count(kind), what)?))
}

fn arg_json(sk: &Sketch, a: &Arg) -> Json {
    match a {
        Arg::Ent(e) => ref_json(*e),
        // A hidden unknown saves as the number it currently holds — Param indices are not
        // stable across a load, and the value is what a reload wants to start from anyway.  One
        // that has been *pinned* saves that too: a fit chose its parameters, and a document that
        // came back with them free would be a document with degrees of freedom nobody drew.
        Arg::Param(i) if sk.params[*i as usize].fixed => {
            object([("value", Json::Num(a.value(sk))), ("pinned", Json::Bool(true))])
        }
        Arg::Param(i) => match sk.shared_name(*i) {
            Some(name) => {
                arg_json(sk, &Arg::Shared { name: name.to_string(), seed: Some(a.value(sk)) })
            }
            None => Json::Num(a.value(sk)),
        },
        // a shared one saves the name its contacts are pinned to, which is what makes it one
        Arg::Shared { name, .. } => object([
            ("value", Json::Num(a.value(sk))),
            ("shared", Json::Str(name.clone())),
        ]),
        // a seed only exists before `Sketch::add`, so a document never sees one; writing the
        // number keeps `describe`/`arg_text` honest for a constraint someone built by hand
        Arg::Seed { value, .. } => Json::Num(*value),
        Arg::Num(v) => Json::Num(*v),
        Arg::Int(v) => Json::Int(*v),
        Arg::Bool(b) => Json::Bool(*b),
        Arg::Str(s) => Json::Str(s.clone()),
        // the text and the number it last made: a document whose expression no longer computes
        // (a name deleted from it by hand) still loads with every dimension a number
        Arg::Expr(e) => object([("expr", Json::Str(e.text.clone())), ("value", Json::Num(e.value))]),
    }
}

fn arg_from_json(sk: &Sketch, kind: SpecKind, v: &Json) -> Result<Arg, String> {
    Ok(match kind {
        k if k.is_dimension() && !matches!(v, Json::Num(_) | Json::Int(_)) => {
            // `{"expr": text, "value": n}` as saved, or a bare string as a person writes one
            let (text, value) = match v {
                Json::Str(s) => match expr::literal(s) {
                    Some(n) => return Ok(Arg::Num(expr::to_arg_units(k, n))),   // `"30"`: a number
                    None => (s.trim().to_string(), 0.0),
                },
                Json::Obj(_) => match v.get("expr") {
                    Some(Json::Str(s)) => (s.clone(), v.get("value").map(|x| x.as_f64()).unwrap_or(0.0)),
                    _ => return Err("a dimension is a number, a string or {\"expr\": ...}".into()),
                },
                _ => return Err("a dimension is a number, a string or {\"expr\": ...}".into()),
            };
            if text.len() > expr::MAX_TEXT {
                return Err(format!("expression longer than {} characters", expr::MAX_TEXT));
            }
            Arg::Expr(expr::Expr::new(text, value))
        }
        k if k.is_entity() => {
            let e = ref_from_json(sk, v, "entity reference")?;
            // and it must be a kind the slot takes: a document is untrusted input, and every
            // reader past this one indexes the list its *spec* names (a projection's planes
            // reach `sk.frames`), so a mismatch here is a panic there — an abort under wasm
            if !crate::constraints::kind_matches(kind, e.kind) {
                return Err(format!("{} slot does not take {}", kind.a(), e.kind.a()));
            }
            Arg::Ent(e)
        }
        SpecKind::Int => Arg::Int(v.as_i64()),
        SpecKind::Bool => Arg::Bool(v.as_bool()),
        // a word slot given a number is a document written before the word existed — `side: -1`
        // for a tangency, which is `side: right` now (issue #48, item 4).  The `"construction":
        // true` bargain: read, never written.
        SpecKind::Str => match v {
            Json::Int(_) | Json::Num(_) => {
                Arg::Str(crate::constraints::side_word(v.as_f64() as i64).to_string())
            }
            _ => Arg::Str(v.as_str().to_string()),
        },
        // a hidden unknown arrives as its number, or `{"value", "pinned"}` when the document
        // said it was worked out rather than solved for; `Sketch::add` consumes both halves
        SpecKind::Param => match v.get("shared") {
            Some(name) => Arg::Shared {
                name: name.as_str().to_string(),
                seed: v.get("value").map(|x| x.as_f64()),
            },
            None => Arg::Seed {
                value: v.get("value").map(|x| x.as_f64()).unwrap_or_else(|| v.as_f64()),
                pinned: v.get("pinned").map(|x| x.as_bool()).unwrap_or(false),
            },
        },
        _ => Arg::Num(v.as_f64()),
    })
}

/// Whether a stored argument says nothing, so the core should read it off the geometry.
pub(crate) fn omitted(v: Option<&Json>) -> bool {
    matches!(v, None | Some(Json::Null))
}

/// A cap on how long a control polygon a document may declare.  A document is untrusted input
/// and `wasm32-unknown-unknown` aborts rather than unwinding, so the size is checked here.
pub const MAX_CTRL: usize = 4096;

/// A stored index, checked against what the document has actually declared so far.  A document is
/// untrusted input: every reference is validated here so a bad one is an `Err` the caller can show,
/// never an out-of-bounds index deeper in the model.
fn index(i: i64, n: usize, what: &str) -> Result<usize, String> {
    if i < 0 || i as usize >= n {
        return Err(format!("{what} index {i} out of range (0..{n})"));
    }
    Ok(i as usize)
}

pub fn to_json(sk: &Sketch) -> Json {
    let points: Vec<Json> = (0..sk.points.len())
        .map(|i| {
            let (x, y) = sk.point_xy(i);
            let mut o =
                object([("x", x.into()), ("y", y.into()), ("fixed", sk.point_fixed(i).into())]);
            // only when set, so a 2D sketch dumps exactly as it always has
            if let Some(p) = sk.plane_of(i) {
                o.set("plane", Json::Int(p as i64));
            }
            if let Some(z) = sk.points[i].z {
                o.set("z", sk.params[z as usize].value.into());
            }
            // drawn in a spatial frame's chart, its three numbers along its axes (#383)
            if let Some(f) = sk.points[i].chart {
                o.set("chart", Json::Int(f as i64));
            }
            o
        })
        .collect();
    let lines: Vec<Json> = sk
        .lines
        .iter()
        .map(|l| {
            object([
                ("p1", (l.p1 as i64).into()),
                ("p2", (l.p2 as i64).into()),
                ("class", class_json(&l.class)),
            ])
        })
        .collect();
    let circles: Vec<Json> = sk
        .circles
        .iter()
        .map(|c| {
            object([
                ("center", (c.center as i64).into()),
                ("r", sk.params[c.radius as usize].value.into()),
                ("fixed", sk.params[c.radius as usize].fixed.into()),
                ("class", class_json(&c.class)),
            ])
        })
        .collect();
    let arcs: Vec<Json> = sk
        .arcs
        .iter()
        .map(|a| {
            object([
                ("center", (a.center as i64).into()),
                ("start", (a.start as i64).into()),
                ("end", (a.end as i64).into()),
                ("r", sk.params[a.radius as usize].value.into()),
                ("fixed", sk.params[a.radius as usize].fixed.into()),
                ("class", class_json(&a.class)),
            ])
        })
        .collect();
    let splines: Vec<Json> = sk
        .splines
        .iter()
        .map(|s| {
            let mut o = object([
                ("ctrl", Json::Arr(s.ctrl.iter().map(|&c| Json::Int(c as i64)).collect())),
                ("knots", Json::Arr(s.knots.iter().map(|&k| Json::Num(k)).collect())),
                ("class", class_json(&s.class)),
            ]);
            // a polynomial spline writes none
            if let Some(w) = &s.weights {
                o.set("weights", Json::Arr(w.iter().map(|&x| Json::Num(x)).collect()));
            }
            o
        })
        .collect();
    let planes: Vec<Json> = sk
        .frames
        .iter()
        .map(|f| {
            // a frame declared bare owns nothing to write (#383)
            let Some(p) = f.chart.as_ref() else {
                return object([("bare", true.into()), ("class", class_json(&f.class))]);
            };
            let g = |k: u32| &sk.params[k as usize];
            // where it stands under the field names, `EntKind::fields`'s
            let mut o = object([
                ("u", (p.u as i64).into()),
                ("v", (p.v as i64).into()),
                ("origin", (p.origin as i64).into()),
                ("x", Json::Num(g(p.o[0]).value)),
                ("y", Json::Num(g(p.o[1]).value)),
                ("z", Json::Num(g(p.o[2]).value)),
                ("fixed", Json::Arr(p.o.iter().map(|&k| g(k).fixed.into()).collect())),
                ("class", class_json(&f.class)),
            ]);
            // its normal, by axis, only where one was read — a document with none dumps as it did
            if let Some(n) = p.normal {
                o.set("normal", (n as i64).into());
            }
            // a spatial frame's chart: a point in it has three numbers
            if !p.planar {
                o.set("planar", false.into());
            }
            o
        })
        .collect();
    let user = sk.user_constraints();
    let constraints: Vec<Json> = user
        .iter()
        .map(|c| {
            // where the callout sits rides *in* the constraint (Solvent §13.1): document state
            // attaches to the statement it qualifies, never to a position in a list.  Keyed by
            // position it followed the position — reorder the list without remapping and a
            // placement reappeared on some other dimension, silently.
            let mut o = object([
                ("type", c.type_name().into()),
                ("args", Json::Arr(c.args.iter().map(|a| arg_json(sk, a)).collect())),
            ]);
            if let (Some(&(t, r)), Json::Obj(fields)) = (sk.placements.get(&c.id), &mut o) {
                fields.push(("place".to_string(), Json::Arr(vec![Json::Num(t), Json::Num(r)])));
            }
            // only when set, so a document with no claim in it dumps exactly as it always has
            if c.claim {
                o.set("claim", Json::Bool(true));
            }
            // the claim block it was written in (#460), likewise only when set
            if let Some(w) = c.witness {
                o.set("witness", w.into());
            }
            // a row of the home alone (#339 D3)
            if c.initially {
                o.set("initially", Json::Bool(true));
            }
            // a bound (§9.6): its direction and an interval's high end, in the number's units
            if let Some(b) = c.bound {
                let mut j = object([("cmp", b.cmp.text().into())]);
                if let Some(hi) = b.hi {
                    j.set("hi", Json::Num(hi));
                }
                o.set("bound", j);
            }
            // a set's derivative row (§6.21): its use's derivative, by index into `duals`
            if let Some(d) = c.along {
                o.set("along", Json::Int(d as i64));
            }
            if !c.class.is_empty() {
                o.set("class", Json::Arr(c.class.0.iter().map(|s| Json::Str(s.clone())).collect()));
            }
            o
        })
        .collect();
    let branches: Vec<(String, Json)> =
        sk.branches.iter().map(|(k, &v)| (k.clone(), Json::Int(v as i64))).collect();
    let mut doc = object([
        ("version", Json::Int(1)),
        ("points", Json::Arr(points)),
        ("lines", Json::Arr(lines)),
        ("circles", Json::Arr(circles)),
        ("arcs", Json::Arr(arcs)),
        ("splines", Json::Arr(splines)),
        ("planes", Json::Arr(planes)),
        ("constraints", Json::Arr(constraints)),
        ("branches", Json::Obj(branches)),
        // written only where the document named one: a drawing in millimetres by default says
        // nothing, so loading and saving it adds no unit it did not have
        ("unit", sk.units.stated().map(|n| Json::Str(n.to_string())).unwrap_or(Json::Null)),
    ]);
    let roles = sk.roles.iter().filter(|(e, _)| !e.kind.spatial() && e.kind != EntKind::Curve)
        .map(|(&e, roles)| {
            let mut o = object([
                ("entity", ref_json(e)),
                ("construction", Json::Bool(roles.construction)),
                ("centerline", Json::Bool(roles.centerline)),
            ]);
            // a claim's witness (#460) only where there is one, so a document with none dumps
            // exactly as it always has
            if let Some(w) = roles.witness {
                o.set("witness", w.into());
            }
            o
        }).collect::<Vec<_>>();
    if !roles.is_empty() { doc.set("roles", Json::Arr(roles)); }
    // a ring's turned copies, only when there is one (a curve is no JSON entity, so its turn is
    // not either): which copy is which representative turned how far about what
    let turns: Vec<Json> = sk.turns.iter().filter(|tn| tn.copy.kind != EntKind::Curve)
        .map(|tn| object([
            ("copy", ref_json(tn.copy)),
            ("rep", ref_json(tn.rep)),
            ("about", ref_json(tn.about)),
            ("k", Json::Int(tn.k as i64)),
            ("n", Json::Int(tn.n as i64)),
        ])).collect();
    if !turns.is_empty() { doc.set("turns", Json::Arr(turns)); }
    // the root choices the document states (#157), only when there is one: each also a record of
    // `branches`, which the plan rewrites; these a solve holds the drawing to
    if !sk.orientations.is_empty() {
        let stated = sk.orientations.iter().map(|(k, &v)| (k.clone(), Json::Int(v as i64)));
        doc.set("orientations", Json::Obj(stated.collect()));
    }
    // the axes, only when there is one: a direction and its fixed flags, and where it is (read
    // back, since a relation placing it is re-added on load and frees the place again)
    let axes: Vec<Json> = sk
        .axes
        .iter()
        .map(|r| {
            let nums = |ps: &[u32; 3]| Json::Arr(ps.iter().map(|&p| sk.params[p as usize].value.into()).collect());
            let mut o = object([
                ("d", nums(&r.d)),
                ("a", nums(&r.a)),
                ("fixed", Json::Arr(r.d.iter().map(|&p| sk.params[p as usize].fixed.into()).collect())),
                ("class", class_json(&r.class)),
            ]);
            // which numbers of where it stands are held, once something reads it
            if r.placed {
                o.set("a_fixed", Json::Arr(r.a.iter().map(|&p| sk.params[p as usize].fixed.into()).collect()));
            }
            o
        })
        .collect();
    if !axes.is_empty() {
        doc.set("axes", Json::Arr(axes));
    }
    // each tangency's derivative, only when there is one (§6.21): the point it moves, what moves
    // it and the geometry the use made
    let duals: Vec<Json> = sk.duals.iter().map(dual_json).collect();
    if !duals.is_empty() {
        doc.set("duals", Json::Arr(duals));
    }
    // each point drawn in further planes, by its twins (§6.7): `[point, twin]`, re-tied on load
    let twins: Vec<Json> = sk.twins.iter()
        .flat_map(|(&p, ts)| {
            ts.iter().map(move |&t| Json::Arr(vec![Json::Int(p as i64), Json::Int(t as i64)]))
        })
        .collect();
    if !twins.is_empty() {
        doc.set("twins", Json::Arr(twins));
    }
    doc
}

fn dual_json(d: &crate::model::Dual) -> Json {
    use crate::model::Toward;
    let mut o = object([
        ("point", Json::Int(d.point as i64)),
        ("owned", Json::Arr(d.owned.iter().map(|&e| ref_json(e)).collect())),
    ]);
    match d.toward {
        Toward::Line(l) => o.set("line", Json::Int(l as i64)),
        Toward::Axis(a) => o.set("axis", Json::Int(a as i64)),
        Toward::Chart { k, axis } => o.set("chart", Json::Arr(vec![
            Json::Int(k as i64),
            axis.map_or(Json::Null, |c| Json::Int(c as i64)),
        ])),
    }
    o
}

/// A tangency's derivative, read from a document (§6.21).
fn dual_from_json(sk: &Sketch, j: &Json) -> Result<(usize, crate::model::Toward, Vec<EntRef>), String> {
    use crate::model::Toward;
    let point = index(j.get("point").map_or(-1, |v| v.as_i64()), sk.points.len(), "dual point")?;
    let toward = match (j.get("line"), j.get("axis"), j.get("chart")) {
        (Some(l), ..) => Toward::Line(index(l.as_i64(), sk.lines.len(), "dual line")?),
        (None, Some(a), _) => Toward::Axis(index(a.as_i64(), sk.axes.len(), "dual axis")?),
        (None, None, Some(c)) => {
            let c = c.arr();
            let k = c.first().map_or(-1, |v| v.as_i64());
            if !(0..=1).contains(&k) {
                return Err("a dual's chart is the first or the second".to_string());
            }
            let axis = match c.get(1) {
                Some(a) if !omitted(Some(a)) => Some(index(a.as_i64(), 3, "dual chart axis")? as u8),
                _ => None,
            };
            Toward::Chart { k: k as u8, axis }
        }
        (None, None, None) => {
            return Err("a dual moves its point along a line, an axis or in a chart".to_string())
        }
    };
    let owned = j.get("owned").map(|v| v.arr()).unwrap_or_default().iter()
        .map(|e| ref_from_json(sk, e, "dual entity"))
        .collect::<Result<_, _>>()?;
    Ok((point, toward, owned))
}

pub fn from_json(d: &Json) -> Result<Sketch, String> {
    let mut sk = Sketch::new();
    // the unit first, for `elaborate`'s reason: it is document preamble, and every number read
    // below — a dimension's text above all — is read in it.  Set after the evaluation at the
    // foot of this function it would be set after the numbers it governs had been worked out.
    if let Json::Str(name) = d.get("unit").unwrap_or(&Json::Null) {
        sk.units = crate::units::Units::with_length(name)?;
    }
    let empty = Json::Arr(Vec::new());
    for (i, p) in d.get("points").unwrap_or(&empty).arr().iter().enumerate() {
        let k = sk.point(
            p.get("x").map(|v| v.as_f64()).unwrap_or(0.0),
            p.get("y").map(|v| v.as_f64()).unwrap_or(0.0),
            p.get("fixed").map(|v| v.as_bool()).unwrap_or(false),
            &format!("p{i}"),
        );
        // a point in space carries its height; one drawn in a plane is given its membership
        // once the planes are read
        if let Some(z) = p.get("z").filter(|v| !omitted(Some(v))) {
            sk.give_place(k, z.as_f64());
        }
    }
    let np = sk.points.len();
    for l in d.get("lines").unwrap_or(&empty).arr() {
        // v1 stored a bare pair
        let (p1, p2) = match l {
            Json::Arr(a) if a.len() == 2 => (a[0].as_i64(), a[1].as_i64()),
            _ => (
                l.get("p1").map(|v| v.as_i64()).unwrap_or(0),
                l.get("p2").map(|v| v.as_i64()).unwrap_or(0),
            ),
        };
        let ln = sk.line(index(p1, np, "line.p1")?, index(p2, np, "line.p2")?);
        sk.lines[ln].class = read_class(l);
    }
    for c in d.get("circles").unwrap_or(&empty).arr() {
        let centre = c.get("center").map(|v| v.as_i64()).unwrap_or(0);
        let centre = index(centre, np, "circle.center")?;
        let ci = sk.circle(centre, c.get("r").map(|v| v.as_f64()).unwrap_or(0.0), "");
        let rp = sk.circles[ci].radius as usize;
        sk.params[rp].fixed = c.get("fixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.circles[ci].class = read_class(c);
    }
    for a in d.get("arcs").unwrap_or(&empty).arr() {
        let g = |k: &str| index(a.get(k).map(|v| v.as_i64()).unwrap_or(0), np, k);
        let ai = sk.arc(g("center")?, g("start")?, g("end")?, "");
        let rp = sk.arcs[ai].radius as usize;
        sk.params[rp].value = a.get("r").map(|v| v.as_f64()).unwrap_or(0.0);
        sk.params[rp].fixed = a.get("fixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.arcs[ai].class = read_class(a);
    }
    for s in d.get("splines").unwrap_or(&empty).arr() {
        let raw = s.get("ctrl").map(|v| v.arr().to_vec()).unwrap_or_default();
        if raw.len() > MAX_CTRL {
            return Err(format!("spline has more than {MAX_CTRL} control points"));
        }
        let mut ctrl = Vec::with_capacity(raw.len());
        for c in &raw {
            ctrl.push(index(c.as_i64(), np, "spline.ctrl")?);
        }
        let knots = s
            .get("knots")
            .map(|v| v.arr().iter().map(|k| k.as_f64()).collect::<Vec<f64>>())
            .filter(|k| !k.is_empty());
        let weights = s.get("weights").map(|v| v.arr().iter().map(|w| w.as_f64()).collect::<Vec<f64>>());
        let si = sk.spline_weighted(&ctrl, knots, weights).ok_or_else(|| {
            format!(
                "a spline needs more than {} control points, a matching knot vector and a \
                 positive weight for each control point it weighs",
                crate::curve::DEGREE
            )
        })?;
        sk.splines[si].class = read_class(s);
    }
    // an ellipse is a library component now (`use std`, `curve e = Ellipse(f, a: …, b: …).p
    // over u in (0, 360)` — issue #47, item 4) and a sketch document cannot say so, so one that
    // carries the old table is refused rather than read short
    if d.get("ellipses").is_some_and(|a| !a.arr().is_empty()) {
        return Err(
            "this document holds an ellipse entity, which is a library component now: write \
             it as `e := trace(Ellipse(f, a: …, b: …).p over u in (0, 360))` under `use std`"
                .to_string(),
        );
    }
    // the axes before the planes built over them, `primitives()`'s order
    let mut places = Vec::new();
    for r in d.get("axes").unwrap_or(&empty).arr() {
        let three = |key: &str| -> [f64; 3] {
            let v = r.get(key).map(|v| v.arr()).unwrap_or_default();
            [0, 1, 2].map(|k| v.get(k).map(|x| x.as_f64()).unwrap_or(0.0))
        };
        let flags = |key: &str| -> [bool; 3] {
            let v = r.get(key).map(|v| v.arr()).unwrap_or_default();
            [0, 1, 2].map(|k| v.get(k).is_some_and(|x| x.as_bool()))
        };
        let ri = sk.axis_restored(three("d"), flags("fixed"), read_class(r));
        places.push((ri, three("a"), flags("a_fixed")));
    }
    // the planes over their axes, each with its origin a point already read
    let nr = sk.axes.len();
    let mut normals = Vec::new();
    for p in d.get("planes").unwrap_or(&empty).arr() {
        // a frame declared bare (#383)
        if p.get("bare").is_some_and(|v| v.as_bool()) {
            sk.frames.push(crate::model::FrameE { class: read_class(p), ..crate::model::FrameE::bare() });
            continue;
        }
        let g = |key: &str, n: usize| index(p.get(key).map(|v| v.as_i64()).unwrap_or(0), n, key);
        let o = ["x", "y", "z"].map(|k| p.get(k).map(|v| v.as_f64()).unwrap_or(0.0));
        let fixed = p.get("fixed").map(|v| v.arr()).unwrap_or_default();
        let held = [0, 1, 2].map(|k| fixed.get(k).map(|v| v.as_bool()).unwrap_or(false));
        let pi = sk.plane_restored(g("u", nr)?, g("v", nr)?, o, held, g("origin", np)?, read_class(p));
        if p.get("normal").is_some() {
            normals.push((pi, g("normal", nr)?));
        }
        if p.get("planar").is_some_and(|v| !v.as_bool()) {
            if sk.points[g("origin", np)?].z.is_none() {
                return Err("a spatial frame's origin has three numbers".into());
            }
            sk.spatial_chart(pi);
        }
    }
    // each normal held to its plane, its rows the plane's own and never written (`stand_normal`)
    for (pi, n) in normals {
        sk.stand_normal(pi, n);
    }
    for (ri, a, held) in places {
        sk.place_restored(ri, a, held);
    }
    // memberships once the planes exist to be members of: a point's `"plane"` names one by
    // index, and the point was read before any plane was
    for (i, pt) in d.get("points").unwrap_or(&empty).arr().iter().enumerate() {
        if let Some(v) = pt.get("plane") {
            if !omitted(Some(v)) {
                sk.set_plane(i, Some(index(v.as_i64(), sk.frames.len(), "point.plane")?));
            }
        }
        if let Some(v) = pt.get("chart").filter(|v| !omitted(Some(v))) {
            let f = index(v.as_i64(), sk.frames.len(), "point.chart")?;
            if sk.frames[f].chart.as_ref().is_none_or(|c| c.planar) || sk.points[i].z.is_none() {
                return Err("a point's chart is a spatial frame's, and the point has three numbers".into());
            }
            if sk.frames[f].origin as usize != i {
                sk.draw_in_chart(i, f);
            }
        }
    }
    // the derivatives before the rows that read them
    for j in d.get("duals").unwrap_or(&empty).arr() {
        let (point, toward, owned) = dual_from_json(&sk, j)?;
        sk.add_dual(point, toward, owned);
    }
    let mut ids = Vec::new();
    for c in d.get("constraints").unwrap_or(&empty).arr() {
        let name = c.get("type").map(|v| v.as_str().to_string()).unwrap_or_default();
        let kind = CKind::from_name(&name)
            .ok_or_else(|| format!("unknown constraint type: {name}"))?;
        let spec = kind.spec();
        let raw = c.get("args").unwrap_or(&empty).arr();
        // fewer than the type has is a document written before a slot existed — `Constraint::new`
        // fills the tail with the defaults, which is what the type meant without them
        if raw.len() > spec.len() {
            return Err(format!("{name}: expected {} args, got {}", spec.len(), raw.len()));
        }
        let mut args = Vec::with_capacity(spec.len());
        for (i, (_, k)) in spec.iter().enumerate().take(raw.len()) {
            // a slot the core infers may be left out — a projection's planes — and holds a
            // placeholder until `seed_omitted` fills it
            if kind.infers_arg(i) && omitted(raw.get(i)) {
                args.push(kind.default_arg(i));
            } else {
                args.push(arg_from_json(&sk, *k, &raw[i])?);
            }
        }
        // …but a slot the core reads off the geometry is never a slot a document predates: a
        // relation in space written without its skew side has it inferred, as a `null` does,
        // rather than defaulted to a side the drawing may not stand on
        if kind.spatial() {
            for i in (raw.len()..spec.len()).take_while(|&i| kind.infers_arg(i)) {
                args.push(kind.default_arg(i));
            }
        }
        // and an ordinate's form is never a slot a document need write: the core reads it
        if kind.form_slot().is_some() {
            for i in raw.len()..spec.len() {
                args.push(kind.default_arg(i));
            }
        }
        seed_omitted(&sk, kind, &mut args, |i| omitted(raw.get(i)))?;
        // `add_quiet`, because the evaluation below is the document's: adding one at a time
        // would parse every expression again for each
        let mut nc = Constraint::new(kind, args);
        // a document is untrusted input: a claim on a kind that owns an unknown would mint a
        // degree of freedom no equation mentions, so the flag is dropped rather than honoured
        nc.claim = c.get("claim").map(|v| v.as_bool()).unwrap_or(false) && kind.claimable();
        // and a bound on a kind no reading steers along is dropped, as such a claim is
        if let Some(b) = c.get("bound").filter(|_| kind.boundable() && !nc.claim) {
            if let Some(cmp) = b.get("cmp").and_then(|v| crate::syntax::Cmp::of(v.as_str())) {
                let hi = b.get("hi").map(|v| v.as_f64());
                nc.bound = Some(crate::constraints::Bound { cmp, hi });
            }
        }
        if let Some(a) = c.get("along") {
            nc.along = Some(along_from_json(&sk, &nc, a)?);
        }
        // a witness's row is a claim's, or nothing: on anything else it would name a block
        // whose rows the drawing solves (#460)
        nc.witness = c.get("witness").filter(|_| nc.claim)
            .map(|v| v.as_i64()).and_then(|w| u32::try_from(w).ok());
        nc.initially = c.get("initially").is_some_and(|v| v.as_bool());
        if let Some(cls) = c.get("class") {
            nc.class = crate::style::Classes(
                cls.arr().iter().map(|s| s.as_str().to_string()).filter(|s| !s.is_empty()).collect(),
            );
        }
        let id = sk.add_quiet(nc);
        // §13.1: the placement rides in the statement it qualifies
        if let Some(a) = c.get("place") {
            let a = a.arr();
            if a.len() == 2 {
                sk.placements.insert(id, (a[0].as_f64(), a[1].as_f64()));
            }
        }
        ids.push(id);
    }
    // each twin tied to its point after the relations, as the elaborator ties it (§6.7), and held
    // on its plane where nothing read above holds it
    for j in d.get("twins").unwrap_or(&empty).arr() {
        let pair = j.arr();
        let at = |k: usize| index(pair.get(k).map_or(-1, |v| v.as_i64()), sk.points.len(), "twin");
        let (p, t) = (at(0)?, at(1)?);
        sk.tie_twin(p, t);
    }
    sk.hold_twins();
    expr::evaluate(&mut sk);   // every expression against the whole document, in order
    // a document written before §13.1: placements in a table of their own, keyed by position in
    // the constraint list.  Read, never written — a document does not have to be re-saved to be
    // readable, and a placement that has already moved onto its constraint wins.
    if let Some(Json::Obj(kv)) = d.get("placements") {
        for (k, v) in kv {
            let a = v.arr();
            if let (Ok(i), 2) = (k.parse::<usize>(), a.len()) {
                if let Some(&id) = ids.get(i) {
                    sk.placements.entry(id).or_insert((a[0].as_f64(), a[1].as_f64()));
                }
            }
        }
    }
    for (field, stated) in [("branches", false), ("orientations", true)] {
        let Some(Json::Obj(kv)) = d.get(field) else { continue };
        let to = if stated { &mut sk.orientations } else { &mut sk.branches };
        for (k, v) in kv {
            let v = v.as_i64() as i32;
            // a root choice is *one* record of a triangle, in ascending order with the sign read
            // against it (`decompose::branch_record`) — so a key that names its three points in
            // some other order is re-recorded here rather than kept as a second record nothing
            // will look up.  Untrusted input, and a document written before the rule, come to the
            // same thing: the `"construction": true` bargain again (issue #48, item 4).
            match decompose::branch_key_points(k) {
                Some(p) => {
                    let (k, v) = decompose::branch_record(p, v >= 0);
                    to.insert(k, v);
                }
                None => {
                    to.insert(k.clone(), v);
                }
            }
        }
    }
    for item in d.get("roles").unwrap_or(&empty).arr() {
        let entity = item.get("entity").ok_or("geometry role needs an entity")?;
        let e = ref_from_json(&sk, entity, "geometry role entity")?;
        let roles = crate::semantics::GeometryRoles {
            construction: item.get("construction").is_some_and(Json::as_bool),
            centerline: item.get("centerline").is_some_and(Json::as_bool),
            witness: item.get("witness").and_then(|w| u32::try_from(w.as_i64()).ok()),
        };
        if roles != Default::default() { sk.roles.insert(e, roles); }
    }
    for item in d.get("turns").unwrap_or(&empty).arr() {
        let ent = |key: &str| -> Result<EntRef, String> {
            let a = item.get(key).ok_or(format!("a turn needs `{key}`"))?;
            ref_from_json(&sk, a, &format!("a turn's `{key}`"))
        };
        let (copy, rep, about) = (ent("copy")?, ent("rep")?, ent("about")?);
        let n = item.get("n").map(Json::as_i64).unwrap_or(0);
        let k = item.get("k").map(Json::as_i64).unwrap_or(0);
        let fits = copy.kind == rep.kind && matches!(copy.kind, EntKind::Point | EntKind::Circle | EntKind::Arc)
            && matches!(about.kind, EntKind::Point | EntKind::Axis);
        if !fits || n < 2 || k < 1 || k >= n {
            return Err("a turn is a point, circle or arc turned k of n steps about a point or an axis".into());
        }
        sk.turn(crate::model::Turn { copy, rep, about, k: k as u32, n: n as u32 });
    }
    // a copy is worked out from what is not one: once, and never from another copy
    let copies: BTreeSet<EntRef> = sk.turns.iter().map(|tn| tn.copy).collect();
    if copies.len() != sk.turns.len()
        || sk.turns.iter().any(|tn| copies.contains(&tn.rep) || copies.contains(&tn.about))
    {
        return Err("a turned copy is turned once, from an entity that is not itself a copy".into());
    }
    sk.settle_turns();
    // a chart a document did not gauge is gauged where its point stands (§6.21)
    sk.choose_charts();
    // each free curve's energy, read again from the statements (#121, #144)
    sk.settle_variational();
    Ok(sk)
}

pub fn dumps(sk: &Sketch, indent: Option<usize>) -> String {
    to_json(sk).dump(indent)
}

pub fn loads(s: &str) -> Result<Sketch, String> {
    from_json(&parse(s)?)
}

/// One argument as a person reads it: an entity by name, an angle in degrees, everything else as
/// a number.  The constraint list, the reports and the dimension callouts on the drawing all
/// print the same value the same way because they all come through here.
pub fn arg_text(kind: SpecKind, a: &Arg) -> String {
    match (kind, a) {
        (k, Arg::Ent(e)) if k.is_entity() => entity_name(*e),
        (_, Arg::Param(i)) => format!("@{i}"),
        // a number written a particular way keeps the way: `3 1/8` says more than 3.125 does,
        // and it is what somebody typed.  A *formula* still shows what it came to.
        (k, Arg::Expr(e)) if expr::notation(&e.text) => as_written(k, &e.text),
        // the formula and what it came to: `h = w * 2 = 80`, `sin(h * 10) = 0.342`
        (k, Arg::Expr(e)) => {
            format!("{} = {}", read_literals(&e.text), arg_text(k, &Arg::Num(e.value)))
        }
        (SpecKind::Angle, a) => format!("{}°", reading(kind, a.num())),
        (SpecKind::Length, a) | (SpecKind::Float, a) => reading(kind, a.num()),
        (_, Arg::Bool(b)) => if *b { "True" } else { "False" }.to_string(),
        (_, Arg::Int(i)) => format!("{i}"),
        (_, Arg::Str(s)) => s.clone(),
        (_, a) => reading(kind, a.num()),
    }
}

/// A dimension as it was written: the text somebody typed, carrying the degree sign an angle is
/// read in — unless the text names its unit already, since `45deg` followed by `°` says it
/// twice (#43.14).  Trimming is all the tidying there is — their spacing is theirs.
fn as_written(kind: SpecKind, text: &str) -> String {
    let t = read_literals(text.trim());
    if kind == SpecKind::Angle && !expr::names_unit(&t) { format!("{t}°") } else { t }
}

/// A dimension's text with every decimal literal longer than `READING_SIG` significant digits
/// read at `READING_SIG`, its unit and everything around it untouched: `13.333333333333334mm`
/// is drawn `13.3333mm`, as the bare number would be.  Such a literal is the flattener's — a
/// formal or a `param` worked out and written back in full (`flatten::fold`), since that text is
/// what the value is computed from — and printed whole it was sixteen digits on the drawing.
/// Only for a reader: nothing parses what this returns.  A word's digits are left alone (`x1`,
/// a copy key's `#3.0.w`), and a literal of six digits or fewer is printed as it was written, so
/// `2.5`, `3 1/8` and `1' 6 3/16"` read as they always did.
fn read_literals(text: &str) -> String {
    let b = text.as_bytes();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    while i < b.len() {
        let in_word = i > 0 && (b[i - 1].is_ascii_alphanumeric() || b"_.#".contains(&b[i - 1]));
        if !b[i].is_ascii_digit() || in_word {
            let ch = text[i..].chars().next().unwrap_or_default();
            out.push(ch);
            i += ch.len_utf8().max(1);
            continue;
        }
        let start = i;
        while i < b.len() && (b[i].is_ascii_digit() || b[i] == b'.') {
            i += 1;
        }
        // an exponent only where digits follow it: `2em` is a number and a word
        if i < b.len() && (b[i] | 0x20) == b'e' {
            let mut j = i + 1;
            if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
                j += 1;
            }
            if j < b.len() && b[j].is_ascii_digit() {
                while j < b.len() && b[j].is_ascii_digit() {
                    j += 1;
                }
                i = j;
            }
        }
        let lit = &text[start..i];
        let mantissa = lit.split(['e', 'E']).next().unwrap_or_default();
        let digits =
            mantissa.trim_start_matches(['0', '.']).bytes().filter(u8::is_ascii_digit).count();
        match lit.parse::<f64>() {
            Ok(v) if digits > READING_SIG => out.push_str(&fmt_g(v, READING_SIG)),
            _ => out.push_str(lit),
        }
    }
    out
}

/// The number a dimensioned constraint states, as its callout prints it — the first Length or
/// Angle in its spec, whichever argument that happens to be.  `None` for a constraint that
/// states no number.
///
/// A dimension written as an expression is drawn **as written**: `h = w / 2` on the callout says
/// what `40` does not, the same bargain `3 1/8` strikes, and it is what somebody typed.  What it
/// came to can be measured off the drawing; where it came from cannot.  `arg_text` — the
/// constraint list — is where a formula prints both.
pub fn dimension_text(c: &Constraint) -> Option<String> {
    let (i, _, kind) = c.dimensions().into_iter().next()?;
    // a drawing shows the number the statement *makes*: an angle written `angle(a, sense: cw)`
    // states −a, and the arc beside the label sweeps that way, so the two must agree (§9.4).  The
    // side of a distance (`side: right`) and the way of a run (`along: left`) share the table
    // but not the rule: the figure is drawn where the word puts it and its label is the
    // magnitude, which is how a drawing has always dimensioned an offset.
    if c.kind == CKind::Angle && c.sense() < 0.0 {
        return Some(arg_text(kind, &Arg::Num(-c.args[i].num())));
    }
    // a dimension the elaborator worked out from a name is drawn with the name — `w`, not the
    // `100` the flattener settled it to — and verbatim: it names something, so no degree sign
    if let Some(w) = &c.written {
        return Some(w.trim().to_string());
    }
    Some(match &c.args[i] {
        Arg::Expr(e) => as_written(kind, &e.text),
        a => arg_text(kind, a),
    })
}

/// A derivative row's dual, read from a document (§6.21) and held to the rule the elaborator's
/// are (`Constraint::differentiable`).
fn along_from_json(sk: &Sketch, c: &Constraint, j: &Json) -> Result<usize, String> {
    let d = index(j.as_i64(), sk.duals.len(), "along")?;
    match c.differentiable(sk, d)? {
        true => Ok(d),
        false => Err(format!("`along` on a {} that reads nothing its use moves", c.type_name())),
    }
}

/// Human-readable one-liner: `P0 distance(80) P1`; angles shown in degrees.  Entities are named
/// `P0`, `L1` — what a sketch with no source calls them; a caller holding a source map gives
/// `describe_with` the names the document uses.
pub fn describe(c: &Constraint) -> String {
    describe_with(c, &|_| None)
}

/// `describe`, with an entity named as the source names it where `name` answers — so a
/// culprit reads `corner distance(60) along` and a reader can find the statement (#43.16).
/// The wording is still this function's: a front end supplies names and never composes the
/// line, which is what keeps the app and the CLI describing one constraint one way.
pub fn describe_with(c: &Constraint, name: &dyn Fn(EntRef) -> Option<String>) -> String {
    let claim = if c.claim { "claim " } else { "" };
    // **a defined word is read as it was written** (§9.9): `a horizontal b`, not the level its
    // body states — the statement a reader can find is the one in the word
    if let Some(w) = &c.word {
        let mut ops: Vec<String> =
            w.ops.iter().map(|&e| name(e).unwrap_or_else(|| entity_name(e))).collect();
        // a set is no entity, and reads as the statement named it (§6.21)
        for (i, set) in &w.sets {
            ops.insert((*i).min(ops.len()), set.clone());
        }
        let args = if w.args.is_empty() { String::new() } else { format!("({})", w.args) };
        return match ops.as_slice() {
            [a, b] => format!("{claim}{a} {}{args} {b}", w.word),
            ops => format!("{claim}{}{args} {}", w.word, ops.join(" ")),
        };
    }
    // an energy's term reads as the statement it came from wrote it, its point and tangent
    // under the names the integrand is stated in (#121)
    if c.kind == CKind::Stationary {
        let (Arg::Str(text), Arg::Ent(e)) = (&c.args[2], &c.args[0]) else { return String::new() };
        let verb = if crate::variational::maximizes(c) { "maximizes" } else { "minimizes" };
        let w = c.args[1].num();
        let w = if w == 1.0 { String::new() } else { format!("{} * ", reading(SpecKind::Float, w)) };
        let over = if text.contains("t.") { "(p, t)" } else { "p" };
        let curve = name(*e).unwrap_or_else(|| entity_name(*e));
        return format!("{curve} {verb} {w}integral({text} over {over})");
    }
    // **the operator, as a document writes it** (spec §9.1) — `syntax::operator_text` is the one
    // place a constraint becomes its spelling, so the drawing, the constraint list and the
    // program panel cannot come to say one constraint three ways.  Hidden unknowns are left out
    // there too: a curve parameter is the solver's business, not a reader's.
    let mut args: Vec<Option<crate::syntax::Arg>> = c
        .kind
        .spec()
        .iter()
        .zip(&c.args)
        .map(|((_, kind), v)| lift_arg(*kind, v, name))
        .collect();
    // a dimension written in terms of a free variable states no number: what it says is which
    // other dimensions it is tied to, and the number beside the formula is only where the solver
    // has currently put it — so it is marked as the reading it is, *on the dimension*
    if c.free.is_some() {
        for ((_, kind), a) in c.kind.spec().iter().zip(args.iter_mut()) {
            if kind.is_dimension() {
                if let Some(crate::syntax::Arg::Dim { text, .. }) = a {
                    text.push_str(" (free)");
                }
            }
        }
    }
    // a bound is said as one (§9.6)
    let unit = c.kind.dimension_kind();
    let hi = c.bound.and_then(|b| b.hi).and_then(|h| lift_arg(unit, &Arg::Num(h), name));
    let args = crate::syntax::bounded(c.kind, &args, c.bound.map(|b| (b.cmp, hi.as_ref())));
    let text = crate::syntax::operator_text(c.kind, &args);
    // a claim is a different statement from the relation it is written over — it is judged, not
    // solved for — so it says so wherever a constraint is read out, in the word the document
    // spells it with.
    format!("{claim}{text}")
}

/// One stored argument, as the syntax that would have written it — the bridge `describe` crosses
/// to print a constraint the library holds in the words a document uses.
///
/// A bare number goes as its *reading* and not as the number: `syntax::num` is the source
/// printer and prints every digit a double has, which is right for a splice and wrong for a
/// person — the list said `49.99999999999999` where the callout said `50`.
fn lift_arg(
    kind: SpecKind,
    a: &Arg,
    name: &dyn Fn(EntRef) -> Option<String>,
) -> Option<crate::syntax::Arg> {
    use crate::syntax::Arg as S;
    Some(match a {
        Arg::Ent(e) => {
            S::Ref(crate::syntax::Ref::new(name(*e).unwrap_or_else(|| entity_name(*e))))
        }
        Arg::Param(_) => return None,
        Arg::Seed { value, pinned } => S::Seed { value: *value, pinned: *pinned },
        Arg::Shared { name, seed } => {
            S::Tie { name: name.clone(), seed: *seed, span: crate::syntax::Span::default() }
        }
        Arg::Num(v) => S::Dim { text: reading(kind, *v), span: crate::syntax::Span::default() },
        Arg::Int(v) => S::Int(*v),
        Arg::Bool(b) => S::Bool(*b),
        Arg::Str(t) => S::Word(t.clone()),
        // the formula and what it came to: `h = w * 2 = 80`, `sin(h * 10) = 0.342`.  A number
        // written a particular way keeps the way — `3 1/8` says more than 3.125 does.
        Arg::Expr(e) if expr::notation(&e.text) => S::Dim {
            text: read_literals(&e.text),
            span: crate::syntax::Span::default(),
        },
        Arg::Expr(e) => S::Dim {
            text: format!("{} = {}", read_literals(&e.text), arg_text(kind, &Arg::Num(e.value))),
            span: crate::syntax::Span::default(),
        },
    })
}
