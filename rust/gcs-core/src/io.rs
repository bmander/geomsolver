//! JSON (de)serialization of sketches.
//!
//! Entities are referenced by `[kind, index]` into the sketch's ordered lists; constraints
//! serialize their constructor arguments per `spec`.  Intrinsic constraints are not stored — the
//! primitives recreate them, and neither are soft ones (a drag target saved mid-drag would come
//! back as geometry the user never drew).

use crate::constraints::{Arg, CKind, Constraint, SpecKind};
use crate::json::{fmt_g, object, parse, Json};
use crate::model::{expand, EntKind, EntRef, Sketch};

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

fn arg_from_json(kind: SpecKind, v: &Json) -> Result<Arg, String> {
    Ok(match kind {
        k if k.is_entity() => {
            let a = v.arr();
            if a.len() != 2 {
                return Err("entity reference must be [kind, index]".into());
            }
            let ek = EntKind::parse(a[0].as_str())
                .ok_or_else(|| format!("unknown entity kind {:?}", a[0].as_str()))?;
            Arg::Ent(EntRef::new(ek, a[1].as_i64() as usize))
        }
        SpecKind::Int => Arg::Int(v.as_i64()),
        SpecKind::Bool => Arg::Bool(v.as_bool()),
        SpecKind::Str => Arg::Str(v.as_str().to_string()),
        _ => Arg::Num(v.as_f64()),
    })
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
    let constraints: Vec<Json> = sk
        .user_constraints()
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
    object([
        ("version", Json::Int(1)),
        ("points", Json::Arr(points)),
        ("lines", Json::Arr(lines)),
        ("circles", Json::Arr(circles)),
        ("arcs", Json::Arr(arcs)),
        ("constraints", Json::Arr(constraints)),
        ("branches", Json::Obj(branches)),
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
    for l in d.get("lines").unwrap_or(&empty).arr() {
        // v1 stored a bare pair
        let (p1, p2) = match l {
            Json::Arr(a) if a.len() == 2 => (a[0].as_i64() as usize, a[1].as_i64() as usize),
            _ => (
                l.get("p1").map(|v| v.as_i64()).unwrap_or(0) as usize,
                l.get("p2").map(|v| v.as_i64()).unwrap_or(0) as usize,
            ),
        };
        let ln = sk.line(p1, p2);
        sk.lines[ln].construction = l.get("construction").map(|v| v.as_bool()).unwrap_or(false);
    }
    for c in d.get("circles").unwrap_or(&empty).arr() {
        let centre = c.get("center").map(|v| v.as_i64()).unwrap_or(0) as usize;
        let ci = sk.circle(centre, c.get("r").map(|v| v.as_f64()).unwrap_or(0.0), "");
        let rp = sk.circles[ci].radius as usize;
        sk.params[rp].fixed = c.get("fixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.circles[ci].construction =
            c.get("construction").map(|v| v.as_bool()).unwrap_or(false);
    }
    for a in d.get("arcs").unwrap_or(&empty).arr() {
        let g = |k: &str| a.get(k).map(|v| v.as_i64()).unwrap_or(0) as usize;
        let ai = sk.arc(g("center"), g("start"), g("end"), "");
        let rp = sk.arcs[ai].radius as usize;
        sk.params[rp].value = a.get("r").map(|v| v.as_f64()).unwrap_or(0.0);
        sk.params[rp].fixed = a.get("fixed").map(|v| v.as_bool()).unwrap_or(false);
        sk.arcs[ai].construction = a.get("construction").map(|v| v.as_bool()).unwrap_or(false);
    }
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
            args.push(arg_from_json(*k, &raw[i])?);
        }
        sk.add(Constraint::new(kind, args));
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

/// Copy of the sketch with the given entities/constraints removed, plus everything that depends on
/// a removed entity.  Deletion by rebuild — simple, and keeps `Sketch`'s invariants trivially true.
pub fn without(sk: &Sketch, entities: &[EntRef], constraints: &[u32]) -> Sketch {
    let dead: Vec<EntRef> = entities.to_vec();
    let alive = |e: EntRef| !dead.contains(&e) && !sk.children(e).iter().any(|c| dead.contains(c));

    let mut tmp = Sketch::new();
    // rebuild through the JSON shape: the surviving entities are renumbered, and every reference
    // follows, which is exactly what "delete by rebuild" means
    let mut keep_pts = Vec::new();
    for i in 0..sk.points.len() {
        if alive(EntRef::point(i)) {
            keep_pts.push(i);
        }
    }
    let pt_index = |i: usize| keep_pts.iter().position(|&p| p == i);
    for &i in &keep_pts {
        let (x, y) = sk.point_xy(i);
        tmp.point(x, y, sk.point_fixed(i), &format!("p{}", tmp.points.len()));
    }
    let mut line_map: Vec<Option<usize>> = vec![None; sk.lines.len()];
    for i in 0..sk.lines.len() {
        if !alive(EntRef::line(i)) {
            continue;
        }
        let l = &sk.lines[i];
        let (Some(p1), Some(p2)) = (pt_index(l.p1 as usize), pt_index(l.p2 as usize)) else {
            continue;
        };
        let ni = tmp.line(p1, p2);
        tmp.lines[ni].construction = l.construction;
        line_map[i] = Some(ni);
    }
    let mut circle_map: Vec<Option<usize>> = vec![None; sk.circles.len()];
    for i in 0..sk.circles.len() {
        if !alive(EntRef::circle(i)) {
            continue;
        }
        let c = &sk.circles[i];
        let Some(centre) = pt_index(c.center as usize) else { continue };
        let ni = tmp.circle(centre, sk.params[c.radius as usize].value, "");
        let rp = tmp.circles[ni].radius as usize;
        tmp.params[rp].fixed = sk.params[c.radius as usize].fixed;
        tmp.circles[ni].construction = c.construction;
        circle_map[i] = Some(ni);
    }
    let mut arc_map: Vec<Option<usize>> = vec![None; sk.arcs.len()];
    for i in 0..sk.arcs.len() {
        if !alive(EntRef::arc(i)) {
            continue;
        }
        let a = &sk.arcs[i];
        let (Some(c), Some(s), Some(e)) = (
            pt_index(a.center as usize),
            pt_index(a.start as usize),
            pt_index(a.end as usize),
        ) else {
            continue;
        };
        let ni = tmp.arc(c, s, e, "");
        let rp = tmp.arcs[ni].radius as usize;
        tmp.params[rp].value = sk.params[a.radius as usize].value;
        tmp.params[rp].fixed = sk.params[a.radius as usize].fixed;
        tmp.arcs[ni].construction = a.construction;
        arc_map[i] = Some(ni);
    }
    let remap = |e: EntRef| -> Option<EntRef> {
        match e.kind {
            EntKind::Point => pt_index(e.i()).map(EntRef::point),
            EntKind::Line => line_map[e.i()].map(EntRef::line),
            EntKind::Circle => circle_map[e.i()].map(EntRef::circle),
            EntKind::Arc => arc_map[e.i()].map(EntRef::arc),
        }
    };
    for c in sk.user_constraints() {
        if constraints.contains(&c.id) {
            continue;
        }
        if expand(sk, &c.entities()).iter().any(|e| dead.contains(e)) {
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
            tmp.add(Constraint::new(c.kind, args));
        }
    }
    tmp.branches = sk.branches.clone();
    tmp
}

/// Human-readable one-liner: `Distance(P0, P1, 80)`; angles shown in degrees.
pub fn describe(c: &Constraint) -> String {
    let parts: Vec<String> = c
        .kind
        .spec()
        .iter()
        .zip(&c.args)
        .map(|((_, kind), v)| match (kind, v) {
            (k, Arg::Ent(e)) if k.is_entity() => entity_name(*e),
            (SpecKind::Angle, a) => format!("{}°", fmt_g(a.num().to_degrees(), 3)),
            (SpecKind::Length, a) | (SpecKind::Float, a) => fmt_g(a.num(), 4),
            (_, Arg::Bool(b)) => if *b { "True" } else { "False" }.to_string(),
            (_, Arg::Int(i)) => format!("{i}"),
            (_, Arg::Str(s)) => s.clone(),
            (_, a) => fmt_g(a.num(), 4),
        })
        .collect();
    format!("{}({})", c.type_name(), parts.join(", "))
}
