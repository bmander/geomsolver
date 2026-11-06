//! **Drawn in a frame with a value** (#398 B): `in screw { in std.front { … } }`, `screw` a
//! jointed frame whose value is stated (`fix(value == τ) screw`).  What its block draws stands
//! where the value puts it: each plane the block draws in is read as that plane drawn in the
//! frame (`P in screw`, #391 S3), minted once per body as a hidden twin, and the block's
//! declarations are drawn in the twin — so they stand where the frame's joint carries them at its
//! value, read again wherever the value is (a face family's every station), and ride on nothing
//! more.  A relation written in the block reads the planes it draws in the same way, so
//! `c on_u(d: r) std.front` holds `c` on the front plane as the screw carries it: the block is
//! the drawing at rest, carried whole.
//!
//! **A frame declared in the block rides on it** (#823) — a planet in its carrier — as the rest of
//! the block does: it rides on the valued frame no more, and its joint turns about its axis where
//! the relations hold it, which held to what the block draws (the planet's pin) is where the value
//! puts that.  What is drawn in its own block is drawn in the twins of the planes it draws in, so
//! it stands where the carrier's value puts it and rides on the inner frame: at a roll it turns on
//! the pin as carried, which is the carrier's turn and then the planet's.  Where the inner frame
//! has a value too, its block's planes are twinned again, `(P in carrier) in planet` — turned
//! about the carried pin after carrying, which is carrying after turning about the pin at rest —
//! so the frames of a train of valued frames compose outermost first, however deep they nest.
//! What is not drawn (a solid, a face, an instance making one) declared in the inner frame's block
//! and in no plane rides on the inner frame alone.
//!
//! Through #391 S5 anything drawn in such a block was refused (E080, `motions::values`): it would
//! have stood where it was drawn.  A declaration drawn in a valued frame and in no plane of it is
//! refused here, since a frame's value carries a plane.
use super::*;
use crate::syntax::OpArg;

/// One plane a train of valued frames' blocks draws in, and the twin it is read as.
struct Twin {
    /// The valued frames, outermost first, and the plane, by the segments the body writes them
    /// with.
    frames: Vec<Vec<String>>,
    plane: Vec<String>,
    /// The twin's name, one the source cannot write (`#in12`).
    name: String,
}

/// The segments a reference names, its root and the fields past it.
fn segments(r: &Ref) -> Vec<String> {
    std::iter::once(r.root.text.clone())
        .chain(r.path.iter().map_while(|s| match s {
            crate::syntax::Seg::Field(f) => Some(f.text.clone()),
            _ => None,
        }))
        .collect()
}

/// `r` read through `twin` where it names `plane` or a member of it (`std.front.origin`).
fn carry(r: &mut Ref, plane: &[String], twin: &str) {
    let segs = segments(r);
    if segs.len() < plane.len() || segs[..plane.len()] != *plane {
        return;
    }
    r.root = Name { text: twin.to_string(), span: r.root.span };
    r.path.drain(..plane.len() - 1);
}

/// The frames and planes a statement is in, as written: its membership's path, or the blocks
/// around a relation.
fn path_of(st: &Stmt) -> Vec<Ref> {
    match &st.kind {
        StmtKind::Decl(d) => d.membership.path().cloned().collect(),
        StmtKind::Instance(i) => i.membership.path().cloned().collect(),
        StmtKind::Relation(r) => r.within.clone(),
        _ => Vec::new(),
    }
}

impl Walk<'_> {
    /// **A body's statements, those drawn in a frame with a value read through their twins**: the
    /// twins to walk first, then the body rewritten — or `None` where nothing in the body is
    /// drawn in a frame the body gives a value, which is then walked as it stands.  `prefix` is
    /// where the body stands, what the twins' keys are made absolute under.
    pub(super) fn valued_blocks(&mut self, body: &[Stmt], prefix: &str) -> Option<Vec<Stmt>> {
        // the frames this body gives a value, as it writes them
        let valued: Vec<Vec<String>> = body.iter().filter_map(|st| {
            let StmtKind::Relation(r) = &st.kind else { return None };
            let w = r.form.written().filter(|w| w.word.text == "fix")?;
            let value = w.args.iter().any(|a| matches!(a, OpArg::Slot { key, .. }
                if key.text == crate::constraints::JOINT_VALUE));
            value.then(|| w.ops.first().map(segments)).flatten()
        }).collect();
        if valued.is_empty() {
            return None;
        }
        let is_valued = |r: &Ref| valued.contains(&segments(r));
        if !body.iter().any(|st| path_of(st).iter().any(is_valued)) {
            return None;
        }
        // the frames declared bare in a valued frame's block (#823): ridden on, never a plane —
        // each with the valued frames whose blocks it is declared in
        let inner: Vec<(Vec<String>, Vec<Vec<String>>)> = body.iter().filter_map(|st| {
            let StmtKind::Decl(d) = &st.kind else { return None };
            let bare = d.kind == crate::model::EntKind::Frame && d.carried.is_none();
            let around: Vec<Vec<String>> = path_of(st).iter().filter(|r| is_valued(r))
                .map(segments).collect();
            (bare && !around.is_empty()).then(|| (vec![d.name.key().text.clone()], around))
        }).collect();
        let is_inner = |r: &Ref| inner.iter().any(|(f, _)| *f == segments(r));
        // whether frame `g` is declared in frame `f`'s block
        let declared_in = |g: &Ref, f: &Ref| inner.iter()
            .any(|(n, around)| *n == segments(g) && around.contains(&segments(f)));
        // the valued frames a statement is in, outermost first
        let train = |path: &[Ref]| -> Vec<Vec<String>> {
            path.iter().filter(|r| is_valued(r)).map(segments).collect()
        };
        // each plane a declaration draws in inside a valued frame's block, once per train of
        // valued frames: `P in f`, then that in the next frame in, and so on
        let mut twins: Vec<Twin> = Vec::new();
        let mut out: Vec<Stmt> = Vec::new();
        for st in body {
            let path = path_of(st);
            // a frame declared in a valued frame's block is drawn in only inside that block: it
            // rides on the valued frame no more, so a block of it outside would not be carried
            let outside = path.iter().enumerate().find_map(|(k, g)| {
                let (_, around) = inner.iter().find(|(n, _)| *n == segments(g))?;
                let before: Vec<Vec<String>> = path[..k].iter().map(segments).collect();
                let f = around.iter().find(|f| !before.contains(f))?;
                Some((ref_text(g), f.join(".")))
            });
            if let Some((g, f)) = outside {
                let m = format!("`{}` is drawn in `{g}`, which is declared in `{f}`'s block, and \
                                 outside it: `{f}` has a value, which carries what is drawn in \
                                 its block — draw it there, `in {f} {{ in {g} {{ … }} }}`",
                                stmt_name(st));
                self.err(Code::E080, st.span, m);
                continue;
            }
            let frames: Vec<&Ref> = path.iter().filter(|r| is_valued(r)).collect();
            let Some(&frame) = frames.first() else { continue };
            // a valued frame in another's block is carried by it where it is declared there: one
            // declared elsewhere turns about an axis the outer one does not carry
            if let Some(k) = (1..frames.len()).find(|&k| !declared_in(frames[k], frames[k - 1])) {
                let (f, g) = (ref_text(frames[k - 1]), ref_text(frames[k]));
                let m = format!("`{}` is drawn in two frames with a value, `{f}` and `{g}`, and \
                                 `{g}` is declared outside `{f}`'s block: a frame with a value \
                                 carries what is drawn in its block, a frame declared there too, \
                                 `in {f} {{ in ({g} := frame) {{ … }} }}`", stmt_name(st));
                self.err(Code::E080, st.span, m);
                continue;
            }
            let (StmtKind::Decl(_) | StmtKind::Instance(_)) = &st.kind else { continue };
            if matches!(&st.kind, StmtKind::Decl(d) if d.kind == crate::model::EntKind::Frame) {
                continue;
            }
            // the plane it is drawn in: the innermost frame of its path neither valued nor ridden
            let Some(plane) = path.iter().rev().find(|r| !is_valued(r) && !is_inner(r)) else {
                // what is not drawn — a solid, a face, an instance making one — in a frame ridden
                // in the block rides on that frame alone
                let undrawn = match &st.kind {
                    StmtKind::Decl(d) => !d.kind.bears_points(),
                    _ => true,
                };
                if undrawn && path.iter().any(is_inner) {
                    continue;
                }
                let f = ref_text(frames.last().copied().unwrap_or(frame));
                let m = format!("`{}` is drawn in `{f}`, which has a value, and in no plane of it: \
                                 a value carries a plane, so draw it in one, `in {f} {{ in P {{ … \
                                 }} }}`", stmt_name(st));
                self.err(Code::E080, st.span, m);
                continue;
            };
            let p = segments(plane);
            let mut source = plane.clone();
            for k in 1..=frames.len() {
                let fs: Vec<Vec<String>> = frames[..k].iter().map(|r| segments(r)).collect();
                if let Some(t) = twins.iter().find(|t| t.frames == fs && t.plane == p) {
                    source = Ref::dotted(&t.name, plane.span);
                    continue;
                }
                let name = match k {
                    1 => format!("#in{}", plane.span.lo),
                    _ => format!("#in{}_{k}", plane.span.lo),
                };
                out.push(twin_stmt(st, &source, frames[k - 1], plane.span, &name));
                // what the source calls it, for a diagnostic to say (`std.top in carrier`)
                let said = match self.carried_planes.get(&format!("{prefix}{}", source.root.text)) {
                    Some(outer) => format!("({outer}) in {}", ref_text(frames[k - 1])),
                    None => format!("{} in {}", ref_text(plane), ref_text(frames[k - 1])),
                };
                self.carried_planes.insert(format!("{prefix}{name}"), said);
                source = Ref::dotted(&name, plane.span);
                twins.push(Twin { frames: fs, plane: p.clone(), name });
            }
        }
        // the body, what is drawn in a valued frame drawn in its twins
        for st in body {
            let path = path_of(st);
            let frames = train(&path);
            if frames.is_empty() {
                out.push(st.clone());
                continue;
            }
            let read = |r: &mut Ref| {
                for t in twins.iter().filter(|t| t.frames == frames) {
                    carry(r, &t.plane, &t.name);
                }
            };
            let gone = |r: &Ref| frames.contains(&segments(r));
            let mut st = st.clone();
            match &mut st.kind {
                // a frame ridden in the block rides on its valued frames no more
                StmtKind::Decl(d) if d.kind == crate::model::EntKind::Frame => {
                    if d.carried.is_none() {
                        d.membership.leave(gone);
                    }
                }
                StmtKind::Decl(d) => {
                    drawn_in_twin(&mut d.membership, &gone, &read);
                    for kid in d.children.iter_mut().flatten() {
                        kid.refs_mut().into_iter().for_each(read);
                    }
                }
                StmtKind::Instance(i) => {
                    drawn_in_twin(&mut i.membership, &gone, &read);
                    for a in &mut i.args {
                        if let crate::syntax::InstVal::Ref(r) = &mut a.value {
                            read(r);
                        }
                    }
                }
                StmtKind::Relation(r) => {
                    r.within.retain(|w| !gone(w));
                    if let Some(w) = r.form.written_mut() {
                        super::words::refs_mut(w).into_iter().for_each(read);
                    }
                }
                _ => {}
            }
            out.push(st);
        }
        Some(out)
    }
}

/// A membership drawn in its plane's twin: the valued frames `gone` says of taken out of its path
/// — what it draws stands where their values put it, and rides on them no more; where one was its
/// plane, the innermost of the frames it rides on takes its place — and the frames left read
/// through `read`.
fn drawn_in_twin(m: &mut crate::syntax::Membership, gone: &impl Fn(&Ref) -> bool,
                 read: &impl Fn(&mut Ref)) {
    m.leave(gone);
    if let Some(p) = m.plane_mut() {
        read(p);
    }
    m.rides_mut().iter_mut().for_each(read);
}

/// The twin of `plane` drawn in `frame`, `#in12 := plane in frame`, made by `st` — `plane` the
/// plane the block names, or its twin in the frames further out — at the plane's span `at`.
fn twin_stmt(st: &Stmt, plane: &Ref, frame: &Ref, at: Span, name: &str) -> Stmt {
    let word = Name { text: crate::syntax::DRAWN_IN.into(), span: at };
    let args = vec![OpArg::Named(word.clone(), crate::syntax::Arg::Ref(frame.clone()))];
    let named = crate::syntax::DeclName::Written(Name { text: name.into(), span: at });
    let decl = Decl {
        kind: crate::model::EntKind::Frame,
        seed: vec![0.0; 3],
        seed_text: vec![None; 3],
        carried: Some(crate::syntax::CarriedFrame { source: plane.clone(), word, args }),
        ..Decl::point(named, [0.0; 2], None)
    };
    Stmt { id: st.id, kind: StmtKind::Decl(decl), span: at,
           chained: crate::syntax::Chained::No }
}

/// What a refusal calls the statement: the name it declares, or its text's start.
fn stmt_name(st: &Stmt) -> String {
    match &st.kind {
        StmtKind::Decl(d) => d.name.key().text.clone(),
        StmtKind::Instance(i) => i.name.text.clone(),
        _ => "a relation".into(),
    }
}
