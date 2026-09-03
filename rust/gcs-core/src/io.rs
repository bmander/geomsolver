//! JSON (de)serialization of sketches.
//!
//! Entities are referenced by `[kind, index]` into the sketch's ordered lists; constraints
//! serialize their constructor arguments per `spec`.  Intrinsic constraints are not stored — the
//! primitives recreate them, and neither are soft ones (a drag target saved mid-drag would come
//! back as geometry the user never drew).

use crate::constraints::{Arg, CKind, Constraint, SpecKind};
use crate::decompose;
use crate::expr;
use crate::json::{fmt_g, object, parse, Json};
use crate::model::{expand, EntKind, EntRef, Sketch};
use crate::style::Classes;
use std::collections::{BTreeMap, BTreeSet};

/// `P0` / `L3` / `C1` / `A2` — the short label the UI and `describe` use.
pub fn entity_name(e: EntRef) -> String {
    // the same letter a minted name starts with, so the label on the drawing and the name in
    // the program cannot disagree about which kind `V0` is
    let c = crate::syntax::kind_initial(e.kind).to_ascii_uppercase();
    format!("{c}{}", e.idx)
}

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
        Arg::Param(_) => Json::Num(a.value(sk)),
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
            let a = v.arr();
            if a.len() != 2 {
                return Err("entity reference must be [kind, index]".into());
            }
            let ek = EntKind::parse(a[0].as_str())
                .ok_or_else(|| format!("unknown entity kind {:?}", a[0].as_str()))?;
            // and it must be a kind the slot takes: a document is untrusted input, and every
            // reader past this one indexes the list its *spec* names (a projection's planes
            // reach `sk.planes`), so a mismatch here is a panic there — an abort under wasm
            if !crate::constraints::kind_matches(kind, ek) {
                return Err(format!("a {} slot does not take a {}", kind.as_str(), ek.as_str()));
            }
            Arg::Ent(EntRef::new(ek, index(a[1].as_i64(), sk.count(ek), ek.as_str())?))
        }
        SpecKind::Int => Arg::Int(v.as_i64()),
        SpecKind::Bool => Arg::Bool(v.as_bool()),
        SpecKind::Str => Arg::Str(v.as_str().to_string()),
        // a hidden unknown arrives as its number, or `{"value", "pinned"}` when the document
        // said it was worked out rather than solved for; `Sketch::add` consumes both halves
        SpecKind::Param => Arg::Seed {
            value: v.get("value").map(|x| x.as_f64()).unwrap_or_else(|| v.as_f64()),
            pinned: v.get("pinned").map(|x| x.as_bool()).unwrap_or(false),
        },
        _ => Arg::Num(v.as_f64()),
    })
}

/// Whether a stored argument says nothing, so the core should read it off the geometry.
pub(crate) fn omitted(v: Option<&Json>) -> bool {
    matches!(v, None | Some(Json::Null))
}

/// Fill in what the caller left out, from the geometry — the hidden unknowns, and the entity
/// slots the core infers (a projection's planes) — then whatever the kind refuses once its
/// arguments are all in.  The one place the rule lives, shared by the elaborator, the document
/// reader, the bindings' constraint records and the Rust constructors, so a constraint is
/// refused by one rule wherever it comes from.  `Err` is the reason, in the caller's words.
pub fn seed_omitted(
    sk: &Sketch,
    kind: CKind,
    args: &mut [Arg],
    left_out: impl Fn(usize) -> bool,
) -> Result<(), String> {
    for (i, _) in kind.param_slots() {
        if left_out(i) {
            args[i] = Arg::Num(crate::constraints::seed_param(sk, kind, args, i));
        }
    }
    for (i, (_, k)) in kind.spec().iter().enumerate() {
        if k.is_entity() && kind.infers_arg(i) && left_out(i) {
            args[i] = Arg::Ent(crate::constraints::infer_entity(sk, kind, args, i)?);
        }
    }
    crate::constraints::validate(sk, kind, args)
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

/// The same lookup `graft`'s `remap` makes, for the one pass that has to run before it exists:
/// a curve's arguments, which may be of any other kind and so must be resolved once every other
/// kind has been grafted.
#[allow(clippy::too_many_arguments)]
fn remap_early(
    pt_index: &dyn Fn(usize) -> Option<usize>,
    line_map: &[Option<usize>],
    circle_map: &[Option<usize>],
    arc_map: &[Option<usize>],
    spline_map: &[Option<usize>],
    plane_map: &[Option<usize>],
    e: EntRef,
) -> Option<EntRef> {
    match e.kind {
        EntKind::Point => pt_index(e.i()).map(EntRef::point),
        EntKind::Line => line_map[e.i()].map(EntRef::line),
        EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
        EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
        EntKind::Spline => spline_map[e.i()].map(EntRef::spline),
        EntKind::Plane => plane_map[e.i()].map(EntRef::plane),
        // a curve is never another curve's argument: nothing in the language says so
        EntKind::Curve => None,
    }
}

pub fn to_json(sk: &Sketch) -> Json {
    let points: Vec<Json> = (0..sk.points.len())
        .map(|i| {
            let (x, y) = sk.point_xy(i);
            let mut o =
                object([("x", x.into()), ("y", y.into()), ("fixed", sk.point_fixed(i).into())]);
            // only when set, so a document with no plane in it dumps exactly as it always has
            if let Some(p) = sk.plane_of(i) {
                o.set("plane", Json::Int(p as i64));
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
            object([
                ("ctrl", Json::Arr(s.ctrl.iter().map(|&c| Json::Int(c as i64)).collect())),
                ("knots", Json::Arr(s.knots.iter().map(|&k| Json::Num(k)).collect())),
                ("class", class_json(&s.class)),
            ])
        })
        .collect();
    let planes: Vec<Json> = sk
        .planes
        .iter()
        .map(|p| {
            let f = &p.frame;
            let v3 = |a: [f64; 3]| Json::Arr(a.iter().map(|&x| Json::Num(x)).collect());
            object([
                ("origin", (f.origin as i64).into()),
                ("toward", (f.toward as i64).into()),
                ("c", sk.params[f.c as usize].value.into()),
                ("s", sk.params[f.s as usize].value.into()),
                ("cfixed", sk.params[f.c as usize].fixed.into()),
                ("sfixed", sk.params[f.s as usize].fixed.into()),
                ("u", v3(p.basis.u)),
                ("v", v3(p.basis.v)),
                ("class", class_json(&f.class)),
            ])
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
            if !c.class.is_empty() {
                o.set("class", Json::Arr(c.class.0.iter().map(|s| Json::Str(s.clone())).collect()));
            }
            o
        })
        .collect();
    let branches: Vec<(String, Json)> =
        sk.branches.iter().map(|(k, &v)| (k.clone(), Json::Int(v as i64))).collect();
    object([
        ("version", Json::Int(1)),
        ("points", Json::Arr(points)),
        ("lines", Json::Arr(lines)),
        ("circles", Json::Arr(circles)),
        ("arcs", Json::Arr(arcs)),
        ("splines", Json::Arr(splines)),
        ("planes", Json::Arr(planes)),
        ("constraints", Json::Arr(constraints)),
        ("branches", Json::Obj(branches)),
        // written only where the document named one: a drawing in drawing units says nothing
        ("unit", sk.units.name().map(|n| Json::Str(n.to_string())).unwrap_or(Json::Null)),
    ])
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
        sk.point(
            p.get("x").map(|v| v.as_f64()).unwrap_or(0.0),
            p.get("y").map(|v| v.as_f64()).unwrap_or(0.0),
            p.get("fixed").map(|v| v.as_bool()).unwrap_or(false),
            &format!("p{i}"),
        );
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
        let si = sk.spline_with(&ctrl, knots).ok_or_else(|| {
            format!(
                "a spline needs more than {} control points and a matching knot vector",
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
             it as `curve e = Ellipse(f, a: …, b: …).p over u in (0, 360)` under `use std`"
                .to_string(),
        );
    }
    // a `frame` from a document written before it was folded into `plane` (issue #47, item 6):
    // read as a plane with the page's attitude, which is what a datum is; never written
    for f in d.get("frames").unwrap_or(&empty).arr() {
        let g = |k: &str| index(f.get(k).map(|v| v.as_i64()).unwrap_or(0), np, k);
        let pi = sk.plane(g("origin")?, g("toward")?, crate::plane::Basis::page(), "");
        let (cp, sp) = (sk.planes[pi].frame.c as usize, sk.planes[pi].frame.s as usize);
        // the saved rotor over the recomputed one: an unsolved document's pose survives a
        // round trip
        if let Some(v) = f.get("c") {
            sk.params[cp].value = v.as_f64();
        }
        if let Some(v) = f.get("s") {
            sk.params[sp].value = v.as_f64();
        }
        sk.params[cp].fixed = f.get("cfixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.params[sp].fixed = f.get("sfixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.planes[pi].frame.class = read_class(f);
    }
    for (k, p) in d.get("planes").unwrap_or(&empty).arr().iter().enumerate() {
        let g = |key: &str| index(p.get(key).map(|v| v.as_i64()).unwrap_or(0), np, key);
        let v3 = |key: &str| -> [f64; 3] {
            let a = p.get(key).map(|v| v.arr()).unwrap_or_default();
            [0, 1, 2].map(|i| a.get(i).map(|v| v.as_f64()).unwrap_or(0.0))
        };
        let basis = crate::plane::Basis::explicit(v3("u"), v3("v"))
            .ok_or_else(|| format!("plane {k}: u and v do not span a plane"))?;
        let pi = sk.plane(g("origin")?, g("toward")?, basis, "");
        let f = &sk.planes[pi].frame;
        let (cp, sp) = (f.c as usize, f.s as usize);
        if let Some(v) = p.get("c") {
            sk.params[cp].value = v.as_f64();
        }
        if let Some(v) = p.get("s") {
            sk.params[sp].value = v.as_f64();
        }
        sk.params[cp].fixed = p.get("cfixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.params[sp].fixed = p.get("sfixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.planes[pi].frame.class = read_class(p);
    }
    // memberships once the planes exist to be members of: a point's `"plane"` names one by
    // index, and the point was read before any plane was
    for (i, pt) in d.get("points").unwrap_or(&empty).arr().iter().enumerate() {
        if let Some(v) = pt.get("plane") {
            if !omitted(Some(v)) {
                sk.set_plane(i, Some(index(v.as_i64(), sk.planes.len(), "point.plane")?));
            }
        }
    }
    let mut ids = Vec::new();
    for c in d.get("constraints").unwrap_or(&empty).arr() {
        let name = c.get("type").map(|v| v.as_str().to_string()).unwrap_or_default();
        let kind = CKind::from_name(&name)
            .ok_or_else(|| format!("unknown constraint type: {name}"))?;
        let spec = kind.spec();
        let raw = c.get("args").unwrap_or(&empty).arr();
        if raw.len() != spec.len() {
            return Err(format!("{name}: expected {} args, got {}", spec.len(), raw.len()));
        }
        let mut args = Vec::with_capacity(spec.len());
        for (i, (_, k)) in spec.iter().enumerate() {
            // a slot the core infers may be left out — a projection's planes — and holds a
            // placeholder until `seed_omitted` fills it
            if kind.infers_arg(i) && omitted(raw.get(i)) {
                args.push(kind.default_arg(i));
            } else {
                args.push(arg_from_json(&sk, *k, &raw[i])?);
            }
        }
        seed_omitted(&sk, kind, &mut args, |i| omitted(raw.get(i)))?;
        // `add_quiet`, because the evaluation below is the document's: adding one at a time
        // would parse every expression again for each, and would make a dimension whose
        // definition is further down the file briefly a free variable — allocating an unknown
        // this pass then retires, in a document that has no free variables in it at all
        let mut nc = Constraint::new(kind, args);
        // a document is untrusted input: a claim on a kind that owns an unknown would mint a
        // degree of freedom no equation mentions, so the flag is dropped rather than honoured
        nc.claim = c.get("claim").map(|v| v.as_bool()).unwrap_or(false) && kind.claimable();
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
    if let Some(Json::Obj(kv)) = d.get("branches") {
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
                    sk.branches.insert(k, v);
                }
                None => {
                    sk.branches.insert(k.clone(), v);
                }
            }
        }
    }
    Ok(sk)
}

pub fn dumps(sk: &Sketch, indent: Option<usize>) -> String {
    to_json(sk).dump(indent)
}

pub fn loads(s: &str) -> Result<Sketch, String> {
    from_json(&parse(s)?)
}

/// Graft the entities of `src` that `keep` accepts onto `dst`, moved by `offset`, and return
/// what that made, in `src` order.
///
/// This is the one rebuild walk in the project: every surviving entity is renumbered into `dst`
/// and every reference to it follows — which is what deletion, copying and pasting all are,
/// differing only in what they keep, where they land and what they land on.  A constraint comes
/// along exactly when every entity it names came along, so the three operations cannot disagree
/// about what a constraint belongs to.
fn graft(dst: &mut Sketch, src: &Sketch, keep: &dyn Fn(EntRef) -> bool, drop_c: &[u32],
         offset: (f64, f64)) -> Vec<EntRef> {
    let base = dst.points.len();
    let fresh = base == 0;   // a rebuild owns the whole document; a paste only adds to it
    if fresh {
        // a rebuild, a deletion and a copy all keep the document they came from — which is what
        // makes a clipboard say what its numbers are in, and `paste` able to convert them
        dst.units = src.units;
        dst.sheet = src.sheet.clone();
    }
    let mut made = Vec::new();
    let mut keep_pts = Vec::new();
    let mut pt_map: Vec<Option<usize>> = vec![None; src.points.len()];
    for i in 0..src.points.len() {
        if keep(EntRef::point(i)) {
            pt_map[i] = Some(base + keep_pts.len());
            keep_pts.push(i);
        }
    }
    let pt_index = |i: usize| pt_map[i];
    for &i in &keep_pts {
        let (x, y) = src.point_xy(i);
        let n = dst.point(x + offset.0, y + offset.1, src.point_fixed(i),
                          &format!("p{}", dst.points.len()));
        made.push(EntRef::point(n));
    }
    let mut line_map: Vec<Option<usize>> = vec![None; src.lines.len()];
    for i in 0..src.lines.len() {
        if !keep(EntRef::line(i)) {
            continue;
        }
        let l = &src.lines[i];
        let (Some(p1), Some(p2)) = (pt_index(l.p1 as usize), pt_index(l.p2 as usize)) else {
            continue;
        };
        let ni = dst.line(p1, p2);
        dst.lines[ni].class = l.class.clone();
        line_map[i] = Some(ni);
        made.push(EntRef::line(ni));
    }
    let mut circle_map: Vec<Option<usize>> = vec![None; src.circles.len()];
    for i in 0..src.circles.len() {
        if !keep(EntRef::circle(i)) {
            continue;
        }
        let c = &src.circles[i];
        let Some(centre) = pt_index(c.center as usize) else { continue };
        let ni = dst.circle(centre, src.params[c.radius as usize].value, "");
        let rp = dst.circles[ni].radius as usize;
        dst.params[rp].fixed = src.params[c.radius as usize].fixed;
        dst.circles[ni].class = c.class.clone();
        circle_map[i] = Some(ni);
        made.push(EntRef::circle(ni));
    }
    let mut arc_map: Vec<Option<usize>> = vec![None; src.arcs.len()];
    for i in 0..src.arcs.len() {
        if !keep(EntRef::arc(i)) {
            continue;
        }
        let a = &src.arcs[i];
        let (Some(c), Some(st), Some(e)) = (
            pt_index(a.center as usize),
            pt_index(a.start as usize),
            pt_index(a.end as usize),
        ) else {
            continue;
        };
        let ni = dst.arc(c, st, e, "");
        let rp = dst.arcs[ni].radius as usize;
        dst.params[rp].value = src.params[a.radius as usize].value;
        dst.params[rp].fixed = src.params[a.radius as usize].fixed;
        dst.arcs[ni].class = a.class.clone();
        arc_map[i] = Some(ni);
        made.push(EntRef::arc(ni));
    }
    let mut spline_map: Vec<Option<usize>> = vec![None; src.splines.len()];
    for i in 0..src.splines.len() {
        if !keep(EntRef::spline(i)) {
            continue;
        }
        let sp = &src.splines[i];
        // the control points that came along, and the ones that did not: a curve that has lost
        // some is shortened, one interior knot going with each, and only dies when too few are
        // left to draw with
        let (mut ctrl, mut gone) = (Vec::new(), Vec::new());
        for (k, &c) in sp.ctrl.iter().enumerate() {
            match pt_index(c as usize) {
                Some(n) => ctrl.push(n),
                None => gone.push(k),
            }
        }
        // with nothing gone this gives back the knots unchanged, so there is no case to split
        let knots = crate::curve::knots_without(&sp.knots, &gone, ctrl.len());
        let class = sp.class.clone();
        let Some(ni) = dst.spline_with(&ctrl, Some(knots)) else { continue };
        dst.splines[ni].class = class;
        spline_map[i] = Some(ni);
        made.push(EntRef::spline(ni));
    }
    let mut plane_map: Vec<Option<usize>> = vec![None; src.planes.len()];
    for i in 0..src.planes.len() {
        if !keep(EntRef::plane(i)) {
            continue;
        }
        let p = &src.planes[i];
        let f = &p.frame;
        let (Some(o), Some(t)) = (pt_index(f.origin as usize), pt_index(f.toward as usize))
        else {
            continue;
        };
        let ni = dst.plane(o, t, p.basis, "");
        let (nc, ns) = (dst.planes[ni].frame.c as usize, dst.planes[ni].frame.s as usize);
        dst.params[nc].value = src.params[f.c as usize].value;
        dst.params[ns].value = src.params[f.s as usize].value;
        dst.params[nc].fixed = src.params[f.c as usize].fixed;
        dst.params[ns].fixed = src.params[f.s as usize].fixed;
        dst.planes[ni].frame.class = f.class.clone();
        plane_map[i] = Some(ni);
        made.push(EntRef::plane(ni));
    }
    // a membership follows its plane across, and a plane that did not come — deleted, or
    // missing a point — takes the memberships that named it with it
    for i in 0..src.points.len() {
        if let (Some(ni), Some(p)) = (pt_index(i), src.plane_of(i)) {
            dst.set_plane(ni, plane_map[p]);
        }
    }
    // curves last: a curve's arguments may be of any other kind, so every map it reads has to
    // be filled before this one is built.  The *definition* travels with it — a document that
    // came apart from the curve family it is written in would be a document that cannot be drawn.
    let mut curve_map: Vec<Option<usize>> = vec![None; src.curves.len()];
    for (i, cv) in src.curves.iter().enumerate() {
        let mut args = Vec::with_capacity(cv.args.len());
        let mut whole = true;
        for &a in &cv.args {
            match remap_early(&pt_index, &line_map, &circle_map, &arc_map, &spline_map,
                              &plane_map, a) {
                Some(r) => args.push(r),
                None => whole = false,
            }
        }
        if !whole || !keep(EntRef::new(EntKind::Curve, i)) {
            continue;
        }
        let def = &src.curve_defs[cv.def as usize];
        let at = match dst.curve_defs.iter().position(|d| d.name == def.name) {
            Some(k) => k,
            None => {
                dst.curve_defs.push(def.clone());
                dst.curve_defs.len() - 1
            }
        };
        // the home pose follows the entities it is read from, and is whole or nothing: a
        // pose with a hole in it is no pose, and the seeds stand in
        let pose: Vec<_> = cv
            .pose
            .iter()
            .filter_map(|&(e, j)| {
                remap_early(&pt_index, &line_map, &circle_map, &arc_map, &spline_map,
                            &plane_map, e)
                    .map(|r| (r, j))
            })
            .collect();
        dst.curves.push(crate::model::CurveE {
            def: at as u32,
            args,
            values: cv.values.clone(),
            domain: cv.domain,
            home: cv.home.clone(),
            pose: crate::model::whole(pose, cv.pose.len()),
            class: cv.class.clone(),
        });
        curve_map[i] = Some(dst.curves.len() - 1);
        made.push(EntRef::new(EntKind::Curve, dst.curves.len() - 1));
    }
    let remap = |e: EntRef| -> Option<EntRef> {
        match e.kind {
            EntKind::Point => pt_index(e.i()).map(EntRef::point),
            EntKind::Line => line_map[e.i()].map(EntRef::line),
            EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
            EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
            EntKind::Spline => spline_map[e.i()].map(EntRef::spline),
            EntKind::Plane => plane_map[e.i()].map(EntRef::plane),
            EntKind::Curve => curve_map[e.i()].map(|i| EntRef::new(EntKind::Curve, i)),
        }
    };
    let mut expr = false;
    for c in src.user_constraints() {
        if drop_c.contains(&c.id) {
            continue;
        }
        let mut args = Vec::with_capacity(c.args.len());
        let mut ok = true;
        for a in &c.args {
            match a {
                Arg::Ent(e) => match remap(*e) {
                    Some(n) => args.push(Arg::Ent(n)),
                    None => {
                        ok = false;
                        break;
                    }
                },
                // the destination allocates its own: a Param index is this sketch's name for
                // it, and the pin rides along in the seed
                Arg::Param(p) => args.push(Arg::Seed {
                    value: a.value(src),
                    pinned: src.params[*p as usize].fixed,
                }),
                other => args.push(other.clone()),
            }
        }
        if ok {
            expr |= crate::expr::has_expr(&args);
            // `add_quiet`: the walk evaluates once at the end, not once per constraint — a
            // dimension whose definition has not been grafted yet is not a free variable, it is
            // one whose turn has not come
            let mut nc = Constraint::new(c.kind, args);
            nc.claim = c.claim;
            nc.class = c.class.clone();
            let id = dst.add_quiet(nc);
            if let Some(&place) = src.placements.get(&c.id) {
                dst.placements.insert(id, place);   // a dimension keeps where it was dragged to
            }
        }
    }
    if expr {
        expr::evaluate(dst);
    }
    // recorded root choices are keyed by sketch point index; grafting renumbers points, so the
    // keys travel with them.  One naming a point that did not come is dropped — replaying it
    // would apply a chirality to whatever triangle inherited those indices.
    for (k, &v) in &src.branches {
        match decompose::branch_key_points(k) {
            None => {
                if fresh {
                    dst.branches.insert(k.clone(), v);   // not a triangle: nothing to renumber
                }
            }
            Some(pts) => {
                let mapped: Option<Vec<usize>> = pts.iter().map(|&p| pt_index(p)).collect();
                if let Some(m) = mapped {
                    // renumbering can reorder the three, and the sign is read against the key's
                    // own order — so it is re-recorded rather than carried across
                    let (k, v) = decompose::branch_record([m[0], m[1], m[2]], v >= 0);
                    dst.branches.insert(k, v);
                }
            }
        }
    }
    made
}

/// Copy of the sketch with the given entities/constraints removed, plus everything that depends on
/// a removed entity.  Deletion by rebuild — simple, and keeps `Sketch`'s invariants trivially true.
pub fn without(sk: &Sketch, entities: &[EntRef], constraints: &[u32]) -> Sketch {
    let dead: Vec<EntRef> = entities.to_vec();
    // An entity survives while enough of its children do.  For a line, a circle or an arc that
    // is all of them, which is the old rule; a spline is defined by a list, so losing one
    // control point shortens the curve rather than deleting it.
    let alive = |e: EntRef| {
        if dead.contains(&e) {
            return false;
        }
        let kids = sk.children(e);
        kids.iter().filter(|c| !dead.contains(c)).count() >= sk.min_children(e, &kids)
    };
    let mut tmp = Sketch::new();
    graft(&mut tmp, sk, &alive, constraints, (0.0, 0.0));
    tmp
}

/// The selection as a sketch of its own: every entity picked, the points that define it, and
/// every constraint all of whose entities came along.  Copying is keeping, which is deleting
/// everything else — so it goes through the same rule, and a constraint that survives a copy is
/// exactly one that would have survived deleting the rest.
pub fn copy(sk: &Sketch, entities: &[EntRef]) -> Sketch {
    let keep: BTreeSet<EntRef> = expand(sk, entities).into_iter().collect();
    let drop: Vec<EntRef> = sk.primitives().into_iter().filter(|e| !keep.contains(e)).collect();
    without(sk, &drop, &[])
}

/// Add everything in `clip` to `sk`, moved by (dx, dy), and return what that made.  The pasted
/// geometry brings its own constraints and nothing else: it is not joined to what is already
/// there, so it can be put where it belongs before being tied down.
/// Paste a figure in, **converting it** where the two documents are in different units.
///
/// A figure copied out of a drawing in inches and pasted into one in millimetres is the same
/// figure, and 2 inches is 50.8 mm: converting is what keeps it so.  The scaling is on a copy of
/// the clipboard, so a clipboard may be pasted into any number of documents and is unchanged by
/// each.  Two documents that say nothing about their units are in the same drawing units and
/// nothing is scaled.
pub fn paste(sk: &mut Sketch, clip: &Sketch, dx: f64, dy: f64) -> Vec<EntRef> {
    match (clip.units.length, sk.units.length) {
        (Some((_, from)), Some((_, to))) if from != to => {
            let mut scaled = clip.clone();
            scaled.rescale(from / to);
            graft(sk, &scaled, &|_| true, &[], (dx, dy))
        }
        _ => graft(sk, clip, &|_| true, &[], (dx, dy)),
    }
}

/// The part of a sketch one gesture can move, as a sketch of its own.
///
/// Dragging a point can only move what is connected to it: the entities it belongs to, what
/// shares their points, what a constraint ties to those — and nothing past a fixed entity,
/// which does not move whatever is done on its other side.  Everything else in the document is
/// unaffected by the drag, so it is not worth compiling, decomposing or solving for it; a drag
/// on a sketch of many separate figures costs what the one figure costs.
///
/// The part is made by the same rebuild walk as deletion, copying and pasting (`graft`), so it is
/// an ordinary sketch: the plan and the systems are built on it unchanged, and the gesture writes
/// the moved parameters back through `write_back`.  Point indices differ between the two, so the
/// things a drag exchanges with its caller by point index — guard triangles, recorded flips,
/// branch keys — go through `point_in` / `point_out`.
pub struct Part {
    pub sketch: Sketch,
    /// part point index → document point index
    to_doc: Vec<usize>,
    /// document point index → part point index, where it came along
    to_part: Vec<Option<usize>>,
    /// (part param, document param) for every parameter of the part
    params: Vec<(usize, usize)>,
}

impl Part {
    /// The part around `seed`.  A fixed entity is a wall: it comes along (a constraint naming
    /// it must keep all of its entities) but nothing is reached through it.
    pub fn around(sk: &Sketch, seed: EntRef) -> Part {
        let prims = sk.primitives();
        // who contains a point, and which constraints name an entity
        let mut parents: Vec<Vec<EntRef>> = vec![Vec::new(); sk.points.len()];
        for &e in &prims {
            for c in sk.children(e) {
                parents[c.i()].push(e);
            }
        }
        // Which constraints name an entity, and — because two dimensions written in terms of the
        // same free variable share an unknown, which is as real a tie as a shared point — which
        // of them read each free variable.  The walk follows both, so a part that reaches one of
        // a tied group reaches all of it.
        let mut named: BTreeMap<EntRef, Vec<usize>> = BTreeMap::new();
        let mut by_free: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for (ci, c) in sk.constraints.iter().enumerate() {
            // a claim constrains nothing, so it welds nothing: two figures a claim spans stay
            // two parts, and a drag of one costs the other nothing
            if c.claim {
                continue;
            }
            for e in c.entities() {
                named.entry(e).or_default().push(ci);
            }
            if let Some(f) = c.free {
                by_free.entry(f.param).or_default().push(ci);
            }
        }
        let wall = |e: EntRef| sk.entity_params(e).iter().all(|&p| sk.params[p as usize].fixed);
        let mut keep: BTreeSet<EntRef> = BTreeSet::new();
        let mut followed: BTreeSet<u32> = BTreeSet::new();
        let mut queue = vec![seed];
        keep.insert(seed);
        while let Some(e) = queue.pop() {
            if wall(e) {
                continue;
            }
            let mut next = sk.children(e);
            if e.kind == EntKind::Point {
                next.extend(parents[e.i()].iter().copied());
            }
            for &ci in named.get(&e).map(|v| v.as_slice()).unwrap_or(&[]) {
                next.extend(sk.constraints[ci].entities());
                // a tied group only has to be opened once, however many of its dimensions the
                // walk arrives at
                if let Some(f) = sk.constraints[ci].free.filter(|f| followed.insert(f.param)) {
                    for &cj in by_free.get(&f.param).map(|v| v.as_slice()).unwrap_or(&[]) {
                        next.extend(sk.constraints[cj].entities());
                    }
                }
            }
            for n in next {
                if keep.insert(n) {
                    queue.push(n);
                }
            }
        }
        let mut sketch = Sketch::new();
        let made = graft(&mut sketch, sk, &|e| keep.contains(&e), &[], (0.0, 0.0));
        // `graft` makes entities kind by kind in document order, which is `primitives` order
        let srcs: Vec<EntRef> = prims.into_iter().filter(|e| keep.contains(e)).collect();
        debug_assert_eq!(srcs.len(), made.len());
        let mut to_doc = Vec::new();
        let mut to_part = vec![None; sk.points.len()];
        let mut params = Vec::new();
        for (&s, &m) in srcs.iter().zip(&made) {
            if s.kind == EntKind::Point {
                to_part[s.i()] = Some(m.i());
                to_doc.push(s.i());
            }
            for (a, b) in sketch.entity_params(m).into_iter().zip(sk.entity_params(s)) {
                params.push((a as usize, b as usize));
            }
        }
        // the free variables came along by name: the rebuild allocated the part's own unknown
        // for each, seeded off the number the dimension carried, so the two are already in step
        for (name, &doc) in &sk.free_vars {
            if let Some(&mine) = sketch.free_vars.get(name) {
                params.push((mine as usize, doc as usize));
            }
        }
        Part { sketch, to_doc, to_part, params }
    }

    /// A document point's index in the part, if it came along.
    pub fn point_in(&self, doc: usize) -> Option<usize> {
        self.to_part.get(doc).copied().flatten()
    }

    /// A part point's index in the document.
    pub fn point_out(&self, part: usize) -> usize {
        self.to_doc[part]
    }

    /// A triangle of document points as part points — `None` if any did not come along.
    pub fn triangle_in(&self, t: (usize, usize, usize)) -> Option<(usize, usize, usize)> {
        Some((self.point_in(t.0)?, self.point_in(t.1)?, self.point_in(t.2)?))
    }

    /// The triangles of `ts` that came along, as part points.  One that did not is dropped: it
    /// names geometry this part cannot move, so there is no orientation here for it to guard.
    pub fn triangles_in(&self, ts: &[(usize, usize, usize)]) -> Vec<(usize, usize, usize)> {
        ts.iter().filter_map(|&t| self.triangle_in(t)).collect()
    }

    pub fn triangle_out(&self, t: (usize, usize, usize)) -> (usize, usize, usize) {
        (self.point_out(t.0), self.point_out(t.1), self.point_out(t.2))
    }

    /// Recorded root choices of the part, keyed as the document keys them.
    pub fn branches_out(&self, b: &BTreeMap<String, i32>) -> BTreeMap<String, i32> {
        b.iter()
            .map(|(k, &v)| match decompose::branch_key_points(k) {
                Some(p) => decompose::branch_record(
                    [self.point_out(p[0]), self.point_out(p[1]), self.point_out(p[2])],
                    v >= 0,
                ),
                None => (k.clone(), v),
            })
            .collect()
    }

    /// Copy the part's parameter values back into the document it came from.
    pub fn write_back(&self, sk: &mut Sketch) {
        for &(a, b) in &self.params {
            sk.params[b].value = self.sketch.params[a].value;
        }
        // this is a parameter write of its own — it does not go through `Sketch::set_x` — so the
        // dimensions written in terms of a free variable are brought up to date here too, or a
        // plan drag would leave the constraint list reading the number from before the gesture
        expr::sync_free(sk);
    }
}

/// Significant digits a stored number is read to — on a callout, in a list, in a report.
///
/// **One constant, read through `reading`**, so `dimension_text` and `describe` cannot print
/// one number two ways: a bare `Arg::Num` that a `param` substitution left at
/// 49.99999999999999 is `50` on the drawing and `50` in the list (#43.15), where the list
/// used to print the float and the callout four digits of it.  Six, because that is where a
/// double's noise ends and a drawing's own numbers do not — `1234.5` keeps its half, which at
/// four it did not.
pub const READING_SIG: usize = 6;

/// A stored number as a reader reads it: in the units they read (degrees for an angle), to
/// `READING_SIG`.  Every place a constraint's number is turned into text for a person goes
/// through here — the callout, the constraint list, the CLI's culprits.
pub fn reading(kind: SpecKind, v: f64) -> String {
    fmt_g(expr::to_user_units(kind, v), READING_SIG)
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
        (k, Arg::Expr(e)) => format!("{} = {}", e.text, arg_text(k, &Arg::Num(e.value))),
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
    let t = text.trim();
    if kind == SpecKind::Angle && !expr::names_unit(t) { format!("{t}°") } else { t.to_string() }
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
    Some(match &c.args[i] {
        Arg::Expr(e) => as_written(kind, &e.text),
        a => arg_text(kind, a),
    })
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
    let text = crate::syntax::operator_text(c.kind, &args);
    // a claim is a different statement from the relation it is written over — it is judged, not
    // solved for — so it says so wherever a constraint is read out, in the word the document
    // spells it with.
    let claim = if c.claim { "claim " } else { "" };
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
        Arg::Num(v) => S::Dim { text: reading(kind, *v), span: crate::syntax::Span::default() },
        Arg::Int(v) => S::Int(*v),
        Arg::Bool(b) => S::Bool(*b),
        Arg::Str(t) => S::Word(t.clone()),
        // the formula and what it came to: `h = w * 2 = 80`, `sin(h * 10) = 0.342`.  A number
        // written a particular way keeps the way — `3 1/8` says more than 3.125 does.
        Arg::Expr(e) if expr::notation(&e.text) => S::Dim {
            text: e.text.clone(),
            span: crate::syntax::Span::default(),
        },
        Arg::Expr(e) => S::Dim {
            text: format!("{} = {}", e.text, arg_text(kind, &Arg::Num(e.value))),
            span: crate::syntax::Span::default(),
        },
    })
}
