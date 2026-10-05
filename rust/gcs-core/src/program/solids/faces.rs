//! Named chains and ordered planar face boundaries.
use super::*;

/// A named closed traversal uses the existing face representation. Its joints have
/// already shared endpoints, so face construction must not manufacture a closing edge.
pub(super) fn chain_face(c: &crate::syntax::NamedChain) -> Decl {
    Decl {
        roles: c.annotations.roles,
        kind: EntKind::Face, name: c.name.clone(),
        children: vec![c.links.iter().cloned().map(Kid::Ref).collect()],
        seed: Vec::new(), seed_text: Vec::new(), seed_spans: Vec::new(),
        unseeded: false, seed_explicit: Vec::new(), closed: false, knots: None,
        curve: None, computed: None, class: Classes::default(), seed_at: None,
        seed_names: Vec::new(), sweep: None, motion: None, angular_span: None, membership: crate::syntax::Membership::default(),
    }
}

pub(super) fn validate_chain(
    sk: &Sketch,
    res: &Resolver,
    c: &crate::syntax::NamedChain,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> bool {
    let mut previous = None;
    for r in &c.links {
        let e = res.lookup(r).filter(|e| e.i() < sk.count(e.kind))
            .and_then(|e| super::super::resolve::follow(sk, e, &r.path).ok());
        let pair = e.filter(|e| matches!(e.kind, EntKind::Line | EntKind::Arc))
            .and_then(|e| crate::model::edge_ends(sk, e));
        let Some((start, end)) = pair else {
            diags.push(Diag {
                code: Code::E080, span: r.span, stmt: Some(st.id),
                message: format!("a named chain traverses lines and arcs; `{}` is not one",
                    crate::syntax::ref_text(r)),
            });
            return false;
        };
        if previous.is_some_and(|p| p != start) {
            diags.push(Diag {
                code: Code::E080, span: r.span, stmt: Some(st.id),
                message: format!("`{}` does not share the preceding link's endpoint in chain `{}`",
                    crate::syntax::ref_text(r), c.name.key().text),
            });
            return false;
        }
        previous = Some(end);
    }
    true
}

/// Expanded references keep the source's last field as the boundary's public name.
fn boundary_name(r: &crate::syntax::Ref) -> &str {
    match r.path.last() {
        Some(crate::syntax::Seg::Field(n)) => &n.text,
        _ => r.root.text.rsplit('.').next().unwrap_or(&r.root.text),
    }
}

/// Turn a resolved hole reference back into a stable path within its containing scope.
/// Expansion encodes a repeated name as `#<statement>.<copy>.<name>`; the public spelling
/// is `name[copy]`, which does not change when an unrelated statement is inserted.
fn boundary_path(name: &str, scope: Option<&str>) -> String {
    let relative = scope.and_then(|p| name.strip_prefix(p)).unwrap_or(name);
    super::super::public_path(relative)
}

fn fresh_boundary_name(prefix: &str, next: &mut usize, reserved: &BTreeSet<&str>) -> String {
    loop {
        let name = format!("{prefix}{next}");
        *next += 1;
        if !reserved.contains(name.as_str()) {
            return name;
        }
    }
}

/// Build a face's ordered boundary on one plane (§6.8).
/// Points introduce straight runs; existing edges take their direction from a neighbour
/// they meet.  Only `-> close` permits a gap from the last item back to the first.  Generated
/// lines carry `.closure`, hidden by the base sheet, and introduce no points or unknowns.
fn build_loop(
    sk: &mut Sketch,
    res: &Resolver,
    kids: &[Kid],
    closed: bool,
    stmt: StmtId,
    span: Span,
    diags: &mut Vec<Diag>,
) -> Option<(crate::model::FaceLoop, Option<u32>)> {
    let mut report = |code: Code, span: Span, m: String| {
        diags.push(Diag { code, span, stmt: Some(stmt), message: m });
    };
    // each item: a reference, or a curve with the two points its stretch runs between
    let mut refs: Vec<(&crate::syntax::Ref, Option<[&crate::syntax::Ref; 2]>)> = Vec::new();
    for k in kids {
        let r = match k {
            Kid::Ref(r) => r,
            Kid::Trim { curve, from, to, .. } => {
                refs.push((curve, Some([from, to])));
                continue;
            }
            _ => {
                report(Code::E080, span, "a face names the edges it is bounded by; a seed places a point".into());
                return None;
            }
        };
        let chain = res.chains.get(&r.root.text).filter(|_| r.path.is_empty());
        let count = chain.map_or(1, |c| c.links.len());
        if count > crate::flatten::MAX_FLAT.saturating_sub(refs.len()) {
            report(Code::E080, span, "a face expands to too many chain edges".into());
            return None;
        }
        if let Some(c) = chain {
            refs.extend(c.links.iter().map(|l| (l, None)));
        } else {
            refs.push((r, None));
        }
    }
    if refs.is_empty() {
        report(Code::E080, span, "a face is a loop of edges: `f := face(ab, bc, cd, da)`".into());
        return None;
    }
    // what one item of the walk is: an edge, or a corner the loop goes straight to
    struct Item {
        entity: EntRef,
        name: String,
    }
    let reserved: BTreeSet<&str> = refs.iter().map(|(r, _)| boundary_name(r)).collect();
    let mut anonymous = 0;
    let mut items: Vec<Item> = Vec::with_capacity(refs.len());
    let mut trimmed: BTreeSet<u32> = BTreeSet::new();
    let entity = |sk: &Sketch, r: &crate::syntax::Ref| {
        res.lookup(r).and_then(|e| super::super::resolve::follow(sk, e, &r.path).ok())
            .ok_or_else(|| (r.span, format!("no such entity: `{}`", r.root.text)))
    };
    for &(r, trim) in &refs {
        let mut e = match entity(sk, r) {
            Ok(e) => e,
            Err((sp, m)) => { report(Code::E101, sp, m); return None }
        };
        // `flank from p to q`: the stretch of a curve between two points its contacts hold
        // on it, minted as a curve of its own (hidden, as a closing line is) whose ends the
        // walk can meet
        if let Some([from, to]) = trim {
            let (p, q) = match (entity(sk, from), entity(sk, to)) {
                (Ok(p), Ok(q)) => (p, q),
                (Err((sp, m)), _) | (_, Err((sp, m))) => { report(Code::E101, sp, m); return None }
            };
            if e.kind != EntKind::Curve {
                report(Code::E080, r.span, format!("`{} from … to …` runs along a curve, and `{}` is a {}",
                    r.root.text, r.root.text, e.kind.as_str()));
                return None;
            }
            for (x, xr) in [(p, from), (q, to)] {
                if x.kind != EntKind::Point {
                    report(Code::E080, xr.span, format!("a stretch of `{}` runs between points, and `{}` is a {}",
                        r.root.text, xr.root.text, x.kind.as_str()));
                    return None;
                }
                if sk.contact_param(x.idx, e.idx).is_none() {
                    report(Code::E080, xr.span, format!("`{}` is not held on `{}`: a face runs along a curve \
                        between points on it, so write `{} coincident {}`", xr.root.text, r.root.text,
                        xr.root.text, r.root.text));
                    return None;
                }
            }
            if p == q {
                report(Code::E080, r.span, format!("`{}` runs from `{}` to itself", r.root.text, from.root.text));
                return None;
            }
            if !trimmed.insert(e.idx) {
                report(Code::E080, r.span, format!("`{}` bounds this face twice: one stretch of a curve \
                    is one edge", r.root.text));
                return None;
            }
            let mut cv = sk.curves[e.i()].clone();
            cv.trim = Some(crate::model::Trim { of: e.idx, from: p.idx, to: q.idx });
            cv.class = Classes::one("closure");
            sk.curves.push(cv);
            e = EntRef::new(EntKind::Curve, sk.curves.len() - 1);
        }
        // **the leaf, not the absolute name.**  By the time a face is built the flattener has
        // rewritten `lid` into `cyl.lid`, and a face path is already prefixed by the solid it
        // belongs to — so keeping the whole thing spells `cyl.body.block.cyl.lid`, saying
        // "cylinder" twice about one face.  The leaf is what the source wrote.
        let leaf = boundary_name(r);
        let name = if leaf.starts_with('#') {
            fresh_boundary_name("edge", &mut anonymous, &reserved)
        } else {
            leaf.to_string()
        };
        match e.kind {
            EntKind::Line | EntKind::Arc | EntKind::Circle | EntKind::Spline | EntKind::Point => {
                items.push(Item { entity: e, name });
            }
            EntKind::Curve if trim.is_some() => items.push(Item { entity: e, name }),
            // a curve standing alone that comes back to where it started is a whole loop, as a
            // circle is (every reader of the loop asks again of the solved curve)
            EntKind::Curve if refs.len() == 1 && !closed && sk.curve_closed(e.i()) => items.push(Item { entity: e, name }),
            EntKind::Curve => {
                report(Code::E080, r.span, format!("a curve runs on past where a face needs it: name the stretch, \
                    `{} from p to q`, between two points held on it", r.root.text));
                return None;
            }
            _ => {
                report(Code::E080,
                    r.span,
                    format!(
                        "a face is bounded by lines, arcs, circles, splines and stretches of \
                         curves, and turns at points; `{}` is a {}",
                        r.root.text,
                        e.kind.as_str()
                    ),
                );
                return None;
            }
        }
    }
    // **a circle is a loop by itself, and may not stand in one** — and so is a closed curve
    let lone_circle = items.len() == 1 && (items[0].entity.kind == EntKind::Circle
        || (items[0].entity.kind == EntKind::Curve && sk.curves[items[0].entity.i()].trim.is_none()));
    if !lone_circle && items.iter().any(|i| i.entity.kind == EntKind::Circle) {
        report(Code::E080, span, "a circle is a whole loop: it stands in a face by itself".into());
        return None;
    }
    // **the ends of each item**, which is what says whether two neighbours already meet.  A
    // point is both of its own.
    let mut ends = Vec::with_capacity(items.len());
    for it in items.iter().filter(|_| !lone_circle) {
        let e = it.entity;
        let pair = if e.kind == EntKind::Point {
            Some((e.idx, e.idx))
        } else {
            crate::model::edge_ends(sk, e)
        };
        let Some(pair) = pair else {
            report(Code::E080, span, format!("`{}` has no ends: a face is a loop, walked in order", it.name));
            return None;
        };
        ends.push(pair);
    }
    let n = items.len();
    // **one item is a loop only when it is a circle** — said here, so that a lone point or a
    // lone line is refused for what it is rather than as a gap between an item and itself
    // (an edge with two ends and the line `-> close` draws back across them is one: an arc and
    // its chord, the stretch of a curve and its)
    let chord = n == 1 && closed && !lone_circle && items[0].entity.kind != EntKind::Point
        && ends[0].0 != ends[0].1;
    if n == 1 && !lone_circle && !chord {
        report(Code::E080,
            span,
            format!(
                "`{}` is not a loop by itself: a face is a loop of edges and the corners \
                 between them",
                items[0].name
            ),
        );
        return None;
    }
    let contains = |i: usize, p: u32| ends[i].0 == p || ends[i].1 == p;
    // Whether each item shares an endpoint with its successor, including the wrap.
    // (a lone edge closed by its chord is its own successor, and meets it only across the gap)
    let meets: Vec<bool> = (0..ends.len())
        .map(|i| !chord && (contains((i + 1) % n, ends[i].0) || contains((i + 1) % n, ends[i].1)))
        .collect();
    // An edge with no meeting neighbour has two unstated readings.  Refuse it before
    // choosing directions, so inserting closing lines cannot choose a shape by accident.
    let walked = ends.len();
    for i in 0..walked {
        if !chord && items[i].entity.kind != EntKind::Point && !meets[(i + n - 1) % n] && !meets[i] {
            report(Code::E080,
                span,
                format!(
                    "`{}` meets neither of its neighbours: a face is a loop, walked in order",
                    items[i].name
                ),
            );
            return None;
        }
    }
    // Walk actual endpoints, not merely shared sets of endpoints.  Both ends can be shared
    // (an arc and its chord), so try either direction of the first item; each later direction
    // is fixed by the preceding exit or by the next neighbour when there is a gap.
    let orient = |reverse_first: bool| -> Result<Vec<(u32, u32)>, usize> {
        let mut walk: Vec<(u32, u32)> = Vec::with_capacity(walked);
        for i in 0..walked {
            let (a, b) = ends[i];
            let prev = (i + n - 1) % n;
            let next = (i + 1) % n;
            let candidates =
                if i == 0 && reverse_first { [(b, a), (a, b)] } else { [(a, b), (b, a)] };
            let (from, to) = candidates
                .into_iter()
                .find(|&(from, to)| {
                    let arrives = !meets[prev]
                        || if i == 0 { contains(prev, from) } else { walk[i - 1].1 == from };
                    let leaves = !meets[i] || contains(next, to);
                    arrives && leaves
                })
                .ok_or(i)?;
            walk.push((from, to));
        }
        if walked > 0 && meets[n - 1] && walk[n - 1].1 != walk[0].0 {
            return Err(n - 1);
        }
        Ok(walk)
    };
    let walk = match orient(false).or_else(|_| orient(true)) {
        Ok(walk) => walk,
        Err(i) => {
            report(Code::E080,
                span,
                format!(
                    "`{}` and its neighbours share no point along the walk: a face must \
                     enter and leave each edge in order",
                    items[i].name
                ),
            );
            return None;
        }
    };
    // -- the walk, and the straight runs it mints ------------------------------------------
    let mut edges: Vec<EntRef> = Vec::with_capacity(n);
    let mut names: Vec<String> = Vec::with_capacity(n);
    let reserved: BTreeSet<&str> =
        items.iter().filter(|i| i.entity.kind != EntKind::Point).map(|i| i.name.as_str()).collect();
    let mut minted = 0usize;
    for i in 0..n {
        if items[i].entity.kind != EntKind::Point {
            edges.push(items[i].entity);
            names.push(items[i].name.clone());
        }
        if lone_circle {
            continue;
        }
        let j = (i + 1) % n;
        let (from, to) = (walk[i].1, walk[j].0);
        if from == to {
            continue;
        }
        // a gap.  The wrap is minted only where `-> close` says so, and an interior one only
        // where a *point* is one of its sides
        if j == 0 {
            if !closed {
                report(Code::E080,
                    span,
                    format!(
                        "`{}` and `{}` share no point: a face is a loop, and one that does not \
                         come back to where it started closes with `-> close`",
                        items[i].name, items[j].name
                    ),
                );
                return None;
            }
        } else if items[i].entity.kind != EntKind::Point && items[j].entity.kind != EntKind::Point {
            report(Code::E080,
                span,
                format!(
                    "`{}` and `{}` share no point: a face is a loop, walked in order",
                    items[i].name, items[j].name
                ),
            );
            return None;
        }
        let li = sk.line(from as usize, to as usize);
        sk.lines[li].class = Classes::one("closure");
        edges.push(EntRef::line(li));
        // A generated side must not merge with a named side in reports or face selection.
        names.push(fresh_boundary_name("close", &mut minted, &reserved));
    }
    // **three corners, or a curve.**  A loop of straight runs between two points is a line
    // drawn twice, and a face with no area is a solid with no volume — worth saying here,
    // where there is a span, rather than letting the boundary evaluation quietly find nothing.
    if !edges.iter().any(|e| matches!(e.kind, EntKind::Arc | EntKind::Circle | EntKind::Spline | EntKind::Curve)) {
        let mut corners: Vec<u32> = edges
            .iter()
            .filter_map(|e| crate::model::edge_ends(sk, *e))
            .flat_map(|(a, b)| [a, b])
            .collect();
        corners.sort_unstable();
        corners.dedup();
        if corners.len() < 3 {
            report(Code::E080, span, "a face is a loop, and a straight one needs three corners".into());
            return None;
        }
    }
    // **one plane**, read off the memberships and never written on the face
    let mut plane: Option<Option<u32>> = None;
    for (e, n) in edges.iter().zip(&names) {
        for c in sk.children(*e).into_iter().chain([*e]) {
            if c.kind != EntKind::Point {
                continue;
            }
            let p = sk.plane_of(c.i()).map(|x| x as u32);
            // a point in no plane stands in space, where a loop bounds nothing: named as the
            // point, since an edge the face minted is no name the source wrote
            if p.is_none() {
                let x = &sk.params[sk.points[c.i()].x as usize].name;
                let point = x.strip_suffix(".x").unwrap_or(x);
                report(Code::E080, span,
                    format!("a face lies in one plane, and `{point}` stands in space: draw it `in` one"));
                return None;
            }
            match plane {
                None => plane = Some(p),
                Some(q) if q == p => {}
                Some(q) => {
                    let say = |x: Option<u32>| match x {
                        Some(i) => format!("`{}`", sk.plane_name(i as usize)),
                        None => "no plane".to_string(),
                    };
                    report(Code::E080,
                        span,
                        format!(
                            "a face lies in one plane, and `{n}` is on {} where the loop is on {}",
                            say(p),
                            say(q)
                        ),
                    );
                    return None;
                }
            }
        }
    }
    Some((crate::model::FaceLoop { edges, edge_names: names }, plane.flatten()))
}

/// Read each boundary without allocating intermediate faces. `owner` is the containing
/// declaration's name, so inline and named sections use the same relative source paths.
pub(super) fn build_face(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    owner: &str,
    stmt: StmtId,
    span: Span,
    diags: &mut Vec<Diag>,
) -> Option<usize> {
    // a face refused after its walk minted a curve's stretch leaves no stretch behind
    let curves = sk.curves.len();
    let built = build_face_whole(sk, res, d, owner, stmt, span, diags);
    if built.is_none() { sk.curves.truncate(curves); }
    built
}

fn build_face_whole(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    owner: &str,
    stmt: StmtId,
    span: Span,
    diags: &mut Vec<Diag>,
) -> Option<usize> {
    if d.children.get(2).is_some_and(|g| !g.is_empty()) {
        diags.push(Diag {code:Code::E080,span,stmt:Some(stmt),
            message:"a spatial face is not a planar sweep profile".into()});
        return None;
    }
    let (outer, plane) = build_loop(sk, res,
        d.children.first().map(Vec::as_slice).unwrap_or_default(), d.closed, stmt, span, diags)?;
    let scope = owner.rsplit_once('.').map(|(p, _)| format!("{p}."));
    let mut holes = Vec::new();
    let mut used: BTreeSet<String> = outer.edge_names.iter().cloned().collect();
    for kid in d.children.get(1).into_iter().flatten() {
        let reference = match kid {
            Kid::Ref(r) if r.path.is_empty() => {
                let chain = res.chains.get(&r.root.text).is_some_and(|c| c.closed);
                let circle = res.lookup(r).is_some_and(|e| e.kind == EntKind::Circle);
                (chain || circle).then_some((r, chain))
            }
            _ => None,
        };
        let Some((r, chain)) = reference else {
            diags.push(Diag { code: Code::E080, span, stmt: Some(stmt),
                message: "a hole is a circle or named closed loop".into() });
            return None;
        };
        let (mut hole, hole_plane) = build_loop(sk, res, std::slice::from_ref(kid), false, stmt, span, diags)?;
        let prefix = boundary_path(&r.root.text, scope.as_deref());
        for name in &mut hole.edge_names {
            *name = if chain { format!("{prefix}.{name}") } else { prefix.clone() };
        }
        if hole_plane != plane {
            diags.push(Diag { code: Code::E080, span: r.span, stmt: Some(stmt),
                message: "a face and its holes must lie in one plane".into() });
            return None;
        }
        if let Some(name) = hole.edge_names.iter().find(|n| !used.insert((*n).clone())) {
            diags.push(Diag { code: Code::E080, span: r.span, stmt: Some(stmt),
                message: format!("hole boundary name `{name}` collides with another boundary; rename the source edge") });
            return None;
        }
        holes.push(hole);
    }
    let i = sk.face(outer.edges, outer.edge_names, &d.name.key().text);
    sk.faces[i].holes = holes;
    sk.faces[i].support = crate::model::FaceSupport::Plane(plane);
    sk.faces[i].class = d.class.clone();
    Some(i)
}
