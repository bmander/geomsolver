//! JSON (de)serialization of sketches.
//!
//! Entities are referenced by `[kind, index]` into the sketch's ordered lists; constraints
//! serialize their constructor arguments per `spec`.  Intrinsic constraints are not stored — the
//! primitives recreate them, and neither are soft ones (a drag target saved mid-drag would come
//! back as geometry the user never drew).

use crate::constraints::{Arg, CKind, Constraint, SpecKind};
use crate::decompose;
use crate::json::{fmt_g, object, parse, Json};
use crate::model::{expand, EntKind, EntRef, Sketch};
use std::collections::BTreeSet;

/// `P0` / `L3` / `C1` / `A2` — the short label the UI and `describe` use.
pub fn entity_name(e: EntRef) -> String {
    let c = e.kind.as_str().chars().next().unwrap().to_ascii_uppercase();
    format!("{c}{}", e.idx)
}

fn ref_json(e: EntRef) -> Json {
    Json::Arr(vec![Json::Str(e.kind.as_str().to_string()), Json::Int(e.idx as i64)])
}

fn arg_json(a: &Arg) -> Json {
    match a {
        Arg::Ent(e) => ref_json(*e),
        Arg::Num(v) => Json::Num(*v),
        Arg::Int(v) => Json::Int(*v),
        Arg::Bool(b) => Json::Bool(*b),
        Arg::Str(s) => Json::Str(s.clone()),
    }
}

fn arg_from_json(sk: &Sketch, kind: SpecKind, v: &Json) -> Result<Arg, String> {
    Ok(match kind {
        k if k.is_entity() => {
            let a = v.arr();
            if a.len() != 2 {
                return Err("entity reference must be [kind, index]".into());
            }
            let ek = EntKind::parse(a[0].as_str())
                .ok_or_else(|| format!("unknown entity kind {:?}", a[0].as_str()))?;
            Arg::Ent(EntRef::new(ek, index(a[1].as_i64(), sk.count(ek), ek.as_str())?))
        }
        SpecKind::Int => Arg::Int(v.as_i64()),
        SpecKind::Bool => Arg::Bool(v.as_bool()),
        SpecKind::Str => Arg::Str(v.as_str().to_string()),
        _ => Arg::Num(v.as_f64()),
    })
}

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
            object([("x", x.into()), ("y", y.into()), ("fixed", sk.point_fixed(i).into())])
        })
        .collect();
    let lines: Vec<Json> = sk
        .lines
        .iter()
        .map(|l| {
            object([
                ("p1", (l.p1 as i64).into()),
                ("p2", (l.p2 as i64).into()),
                ("construction", l.construction.into()),
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
                ("construction", c.construction.into()),
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
                ("construction", a.construction.into()),
            ])
        })
        .collect();
    // one list, walked twice: the placements below are keyed by position in it, so they and the
    // constraints they name have to be the same walk
    let user = sk.user_constraints();
    let constraints: Vec<Json> = user
        .iter()
        .map(|c| {
            object([
                ("type", c.type_name().into()),
                ("args", Json::Arr(c.args.iter().map(arg_json).collect())),
            ])
        })
        .collect();
    let branches: Vec<(String, Json)> =
        sk.branches.iter().map(|(k, &v)| (k.clone(), Json::Int(v as i64))).collect();
    // Callout placements travel by position in the constraint list above, not by constraint id:
    // loading a document assigns fresh ids in that order, so the index is the only name for a
    // constraint that both sides of a save agree on.
    let placements: Vec<(String, Json)> = user
        .iter()
        .enumerate()
        .filter_map(|(i, c)| {
            sk.placements
                .get(&c.id)
                .map(|&(t, r)| (i.to_string(), Json::Arr(vec![Json::Num(t), Json::Num(r)])))
        })
        .collect();
    object([
        ("version", Json::Int(1)),
        ("points", Json::Arr(points)),
        ("lines", Json::Arr(lines)),
        ("circles", Json::Arr(circles)),
        ("arcs", Json::Arr(arcs)),
        ("constraints", Json::Arr(constraints)),
        ("branches", Json::Obj(branches)),
        ("placements", Json::Obj(placements)),
    ])
}

pub fn from_json(d: &Json) -> Result<Sketch, String> {
    let mut sk = Sketch::new();
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
        sk.lines[ln].construction = l.get("construction").map(|v| v.as_bool()).unwrap_or(false);
    }
    for c in d.get("circles").unwrap_or(&empty).arr() {
        let centre = c.get("center").map(|v| v.as_i64()).unwrap_or(0);
        let centre = index(centre, np, "circle.center")?;
        let ci = sk.circle(centre, c.get("r").map(|v| v.as_f64()).unwrap_or(0.0), "");
        let rp = sk.circles[ci].radius as usize;
        sk.params[rp].fixed = c.get("fixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.circles[ci].construction =
            c.get("construction").map(|v| v.as_bool()).unwrap_or(false);
    }
    for a in d.get("arcs").unwrap_or(&empty).arr() {
        let g = |k: &str| index(a.get(k).map(|v| v.as_i64()).unwrap_or(0), np, k);
        let ai = sk.arc(g("center")?, g("start")?, g("end")?, "");
        let rp = sk.arcs[ai].radius as usize;
        sk.params[rp].value = a.get("r").map(|v| v.as_f64()).unwrap_or(0.0);
        sk.params[rp].fixed = a.get("fixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.arcs[ai].construction = a.get("construction").map(|v| v.as_bool()).unwrap_or(false);
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
            args.push(arg_from_json(&sk, *k, &raw[i])?);
        }
        ids.push(sk.add(Constraint::new(kind, args)));
    }
    if let Some(Json::Obj(kv)) = d.get("placements") {
        for (k, v) in kv {
            let a = v.arr();
            if let (Ok(i), 2) = (k.parse::<usize>(), a.len()) {
                if let Some(&id) = ids.get(i) {
                    sk.placements.insert(id, (a[0].as_f64(), a[1].as_f64()));
                }
            }
        }
    }
    if let Some(Json::Obj(kv)) = d.get("branches") {
        for (k, v) in kv {
            sk.branches.insert(k.clone(), v.as_i64() as i32);
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
    let mut made = Vec::new();
    let mut keep_pts = Vec::new();
    for i in 0..src.points.len() {
        if keep(EntRef::point(i)) {
            keep_pts.push(i);
        }
    }
    let pt_index = |i: usize| keep_pts.iter().position(|&p| p == i).map(|n| base + n);
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
        dst.lines[ni].construction = l.construction;
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
        dst.circles[ni].construction = c.construction;
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
        dst.arcs[ni].construction = a.construction;
        arc_map[i] = Some(ni);
        made.push(EntRef::arc(ni));
    }
    let remap = |e: EntRef| -> Option<EntRef> {
        match e.kind {
            EntKind::Point => pt_index(e.i()).map(EntRef::point),
            EntKind::Line => line_map[e.i()].map(EntRef::line),
            EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
            EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
        }
    };
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
                other => args.push(other.clone()),
            }
        }
        if ok {
            let id = dst.add(Constraint::new(c.kind, args));
            if let Some(&place) = src.placements.get(&c.id) {
                dst.placements.insert(id, place);   // a dimension keeps where it was dragged to
            }
        }
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
                    dst.branches.insert(decompose::branch_key([m[0], m[1], m[2]]), v);
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
    let alive = |e: EntRef| !dead.contains(&e) && !sk.children(e).iter().any(|c| dead.contains(c));
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
pub fn paste(sk: &mut Sketch, clip: &Sketch, dx: f64, dy: f64) -> Vec<EntRef> {
    graft(sk, clip, &|_| true, &[], (dx, dy))
}

/// One argument as a person reads it: an entity by name, an angle in degrees, everything else as
/// a number.  The constraint list, the reports and the dimension callouts on the drawing all
/// print the same value the same way because they all come through here.
pub fn arg_text(kind: SpecKind, a: &Arg) -> String {
    match (kind, a) {
        (k, Arg::Ent(e)) if k.is_entity() => entity_name(*e),
        (SpecKind::Angle, a) => format!("{}°", fmt_g(a.num().to_degrees(), 3)),
        (SpecKind::Length, a) | (SpecKind::Float, a) => fmt_g(a.num(), 4),
        (_, Arg::Bool(b)) => if *b { "True" } else { "False" }.to_string(),
        (_, Arg::Int(i)) => format!("{i}"),
        (_, Arg::Str(s)) => s.clone(),
        (_, a) => fmt_g(a.num(), 4),
    }
}

/// The number a dimensioned constraint states, as its callout prints it — the first Length or
/// Angle in its spec, whichever argument that happens to be.  `None` for a constraint that
/// states no number.
pub fn dimension_text(c: &Constraint) -> Option<String> {
    let (i, _, kind) = c.dimensions().into_iter().next()?;
    Some(arg_text(kind, &c.args[i]))
}

/// Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees.
pub fn describe(c: &Constraint) -> String {
    let parts: Vec<String> = c
        .kind
        .spec()
        .iter()
        .zip(&c.args)
        .map(|((_, kind), v)| arg_text(*kind, v))
        .collect();
    format!("{}({})", c.type_name(), parts.join(", "))
}
