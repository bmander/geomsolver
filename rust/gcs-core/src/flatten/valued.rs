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
//! Through #391 S5 anything drawn in such a block was refused (E080, `motions::values`): it would
//! have stood where it was drawn.  A frame declared in the block still rides on the valued frame,
//! and that is still refused there; a declaration drawn in the valued frame and no plane of it is
//! refused here, since a frame's value carries a plane.
use super::*;
use crate::syntax::OpArg;

/// One plane a valued frame's block draws in, and the twin it is read as.
struct Twin {
    /// The valued frame and the plane, by the segments the body writes them with.
    frame: Vec<String>,
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

impl Walk<'_> {
    /// **A body's statements, those drawn in a frame with a value read through their twins**: the
    /// twins to walk first, then the body rewritten — or `None` where nothing in the body is
    /// drawn in a frame the body gives a value, which is then walked as it stands.
    pub(super) fn valued_blocks(&mut self, body: &[Stmt]) -> Option<Vec<Stmt>> {
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
        let path_of = |st: &Stmt| -> Vec<Ref> {
            match &st.kind {
                StmtKind::Decl(d) => d.membership.path().cloned().collect(),
                StmtKind::Instance(i) => i.membership.path().cloned().collect(),
                StmtKind::Relation(r) => r.within.clone(),
                _ => Vec::new(),
            }
        };
        if !body.iter().any(|st| path_of(st).iter().any(is_valued)) {
            return None;
        }
        // each plane a declaration draws in inside a valued frame's block, once per frame
        let mut twins: Vec<Twin> = Vec::new();
        let mut out: Vec<Stmt> = Vec::new();
        for st in body {
            let path = path_of(st);
            let frames: Vec<&Ref> = path.iter().filter(|r| is_valued(r)).collect();
            let Some(&frame) = frames.first() else { continue };
            if frames.len() > 1 {
                let m = format!("`{}` is drawn in two frames with a value, `{}` and `{}`: a value \
                                 carries what is drawn in its frame alone", stmt_name(st),
                                ref_text(frames[0]), ref_text(frames[1]));
                self.err(Code::E080, st.span, m);
                continue;
            }
            let (StmtKind::Decl(_) | StmtKind::Instance(_)) = &st.kind else { continue };
            if matches!(&st.kind, StmtKind::Decl(d) if d.kind == crate::model::EntKind::Frame) {
                continue;
            }
            // the plane it is drawn in: the innermost frame of its path but the valued one
            let Some(plane) = path.iter().rev().find(|r| !is_valued(r)) else {
                let f = ref_text(frame);
                let m = format!("`{}` is drawn in `{f}`, which has a value, and in no plane of it: \
                                 a value carries a plane, so draw it in one, `in {f} {{ in P {{ … \
                                 }} }}`", stmt_name(st));
                self.err(Code::E080, st.span, m);
                continue;
            };
            let (f, p) = (segments(frame), segments(plane));
            if !twins.iter().any(|t| t.frame == f && t.plane == p) {
                let name = format!("#in{}", plane.span.lo);
                out.push(twin_stmt(st, plane, frame, &name));
                twins.push(Twin { frame: f, plane: p, name });
            }
        }
        // the body, what is drawn in a valued frame drawn in its twins
        for st in body {
            let path = path_of(st);
            let Some(frame) = path.iter().find(|r| is_valued(r)).map(segments) else {
                out.push(st.clone());
                continue;
            };
            let read = |r: &mut Ref| {
                for t in twins.iter().filter(|t| t.frame == frame) {
                    carry(r, &t.plane, &t.name);
                }
            };
            let mut st = st.clone();
            match &mut st.kind {
                StmtKind::Decl(d) if d.kind != crate::model::EntKind::Frame => {
                    drawn_in_twin(&mut d.membership, &frame, &read);
                    for kid in d.children.iter_mut().flatten() {
                        kid.refs_mut().into_iter().for_each(read);
                    }
                }
                StmtKind::Instance(i) => {
                    drawn_in_twin(&mut i.membership, &frame, &read);
                    for a in &mut i.args {
                        if let crate::syntax::InstVal::Ref(r) = &mut a.value {
                            read(r);
                        }
                    }
                }
                StmtKind::Relation(r) => {
                    r.within.retain(|w| segments(w) != frame);
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

/// A membership drawn in its plane's twin: the valued frame `frame` taken out of its path — what
/// it draws stands where the value puts it, and rides on it no more; where the frame was its
/// plane, the innermost of the frames it rides on takes its place — and the frames left read
/// through `read`.
fn drawn_in_twin(m: &mut crate::syntax::Membership, frame: &[String], read: &impl Fn(&mut Ref)) {
    m.rides_mut().retain(|r| segments(r) != frame);
    if m.plane().is_some_and(|p| segments(p) == frame) {
        if let Some(inner) = m.rides_mut().pop() {
            if let Some(p) = m.plane_mut() {
                *p = inner;
            }
        }
    }
    if let Some(p) = m.plane_mut() {
        read(p);
    }
    m.rides_mut().iter_mut().for_each(read);
}

/// The twin of `plane` drawn in `frame`, `#in12 := plane in frame`, made by `st`.
fn twin_stmt(st: &Stmt, plane: &Ref, frame: &Ref, name: &str) -> Stmt {
    let word = Name { text: crate::syntax::DRAWN_IN.into(), span: plane.span };
    let args = vec![OpArg::Named(word.clone(), crate::syntax::Arg::Ref(frame.clone()))];
    let named = crate::syntax::DeclName::Written(Name { text: name.into(), span: plane.span });
    let decl = Decl {
        kind: crate::model::EntKind::Frame,
        seed: vec![0.0; 3],
        seed_text: vec![None; 3],
        carried: Some(crate::syntax::CarriedFrame { source: plane.clone(), word, args }),
        ..Decl::point(named, [0.0; 2], None)
    };
    Stmt { id: st.id, kind: StmtKind::Decl(decl), span: plane.span,
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
