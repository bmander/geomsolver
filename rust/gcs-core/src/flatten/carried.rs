//! **A reading at an instant** (#391 S2, Solvent §6.14 [0.140]): `n := x at τ` is `x` read at the
//! instant `τ` of the roll by the frames it rides on — or by the frame `at(under: m)` names, where
//! it rides on none.  What it makes depends on what `x` is, which the parser cannot see and the
//! walk's resolution can: so every reading is walked all three ways, each statement's scope
//! tagged with which (`Scope::reading`), and `Walk::settle_readings` keeps the one `x` asks for
//! once every name is known — a solid (`solid`), a point (`point`), and otherwise a frame:
//!
//! **A frame a motion carries** (#366, [0.138]): `base` carried to `τ`, its origin and its axes
//! with it.  Written out here, where a body is walked, as the statements a person would write
//! for it, so every reader after reads them as it reads any, and finds the carrier as it finds a
//! point's (`relations::riding_point`):
//!
//! * the source's frame, three points of it drawn in it and held there by `fix`, riding as it
//!   does: its origin and a point ten along each of its axes (`n#so`, `n#su`, `n#sv`);
//! * each carried to `τ`, a point in space riding on nothing (`n#o`, `n#u`, `n#v`): `X
//!   coincident x at τ` (`CKind::CarriedPoint` at a stated roll) with the declaration's own
//!   arguments;
//! * and the plane over the two lines from the carried origin, `n := plane(u: n#lu, v: n#lv)`.
//!
//! A rigid motion keeps lengths and angles, so the image is a frame as its source is, turned the
//! same way: its `u`, `v` and normal are the source's carried.  The roll is a dimension, so it may
//! be written over a face family's running number (`at 360deg * turns * s`) and is read again at
//! every station; `std.sv`'s station words are motions read so.  The statements share the
//! declaration's id, so whatever is said of one is said where it was written, and the source
//! keeps the one statement (`syntax::Decl::carried`).

use crate::semantics::{Annotations, GeometryRoles};
use crate::syntax::{
    Arg, Chained, Decl, DeclName, Kid, Membership, Name, OpArg, Ref, Relation, RelationForm,
    Span, Stmt, StmtKind, Sweep, Written,
};

/// How far along each of the source's axes the frame's second and third points stand, in the
/// document's unit: any length but none, since a rigid motion keeps it.
const REACH: f64 = 10.0;

/// What a reading at an instant turns out to be (#391 S2), once what it reads is known: the
/// kind of the source, a solid or a point, and a frame for anything else.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Read {
    Frame,
    Solid,
    Point,
}

/// Every statement `n := x at τ` may stand for, each with what `x` must be for it to be the one.
pub(super) fn readings(st: &Stmt, d: &Decl) -> Vec<(Stmt, Read)> {
    let mut out: Vec<(Stmt, Read)> = frame(st, d).into_iter().map(|s| (s, Read::Frame)).collect();
    out.push((solid(st, d), Read::Solid));
    out.extend(point(st, d).into_iter().map(|s| (s, Read::Point)));
    out
}

/// The frame a reading names on its `at`, `under:` or `seen_from:`.
fn named(c: &crate::syntax::CarriedFrame, label: &str) -> Option<Ref> {
    c.args.iter().find_map(|a| match a {
        OpArg::Named(n, Arg::Ref(r)) if n.text == label => Some(r.clone()),
        _ => None,
    })
}

/// `n := tool at θ`, `tool` a solid: the solid placed at that roll (`Sweep::Placed`), by the
/// frame named or, left out, the one it rides on, seen from the one named.
fn solid(st: &Stmt, d: &Decl) -> Stmt {
    let c = d.carried.as_ref().expect("a reading at an instant");
    let at = c.args.iter().find_map(|a| match a {
        OpArg::Dim(text, span) => Some(Arg::Dim { text: text.clone(), span: *span }),
        _ => None,
    }).expect("an instant states its roll");
    let placed = Decl {
        annotations: d.annotations,
        kind: crate::model::EntKind::Solid,
        children: vec![vec![Kid::Ref(c.source.clone())]],
        seed: Vec::new(),
        seed_text: Vec::new(),
        class: d.class.clone(),
        class_span: d.class_span,
        sweep: Some(Sweep::Placed { motion: named(c, "under"), at }),
        seen_from: named(c, "seen_from"),
        list_span: Span::new(c.source.span.lo as usize, st.span.hi as usize),
        ..Decl::point(d.name.clone(), [0.0; 2], None)
    };
    Stmt { id: st.id, kind: StmtKind::Decl(placed), span: st.span, chained: Chained::No }
}

/// `n := x at τ`, `x` a point: a point of its own, held to `x` carried there (`X coincident x at
/// τ`) — drawn where the declaration is, so in a plane it is held where that plane sees it.
fn point(st: &Stmt, d: &Decl) -> Vec<Stmt> {
    use crate::constraints::Fixity::Infix;
    let c = d.carried.as_ref().expect("a reading at an instant");
    let at = c.word.span;
    let p = Decl {
        annotations: d.annotations,
        class: d.class.clone(),
        class_span: d.class_span,
        membership: d.membership.clone(),
        carried: None,
        ..Decl::point(d.name.clone(), [0.0; 2], None)
    };
    let me = Ref::dotted(&d.name.key().text, d.name.span());
    let held = written("coincident", Infix, vec![me, c.source.clone()], c.args.clone(), at);
    let stmt = |kind: StmtKind| Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
    vec![stmt(StmtKind::Decl(p)), stmt(StmtKind::Relation(held))]
}

/// A relation as written, at the frame's word.
fn written(word: &str, fixity: crate::constraints::Fixity, ops: Vec<Ref>, args: Vec<OpArg>,
           at: Span) -> Relation {
    Relation {
        form: RelationForm::Written(Written {
            word: Name { text: word.to_string(), span: at },
            fixity,
            ops,
            args,
            span: at,
            rates: Vec::new(),
        }),
        place: None,
        place_span: Span::default(),
        claim: false,
        class: Default::default(),
        class_span: Span::default(),
        word: None,
        along: None,
        space: false,
        initially: false,
    }
}

/// The statements one carried frame stands for.
pub(super) fn frame(st: &Stmt, d: &Decl) -> Vec<Stmt> {
    use crate::constraints::Fixity::{Infix, Prefix};
    let c = d.carried.as_ref().expect("a carried frame");
    let n = d.name.key().text.clone();
    let at = c.word.span;
    let key = |k: &str| DeclName::Key(Name { text: format!("{n}#{k}"), span: at });
    let local = |k: &str| Ref::dotted(&format!("{n}#{k}"), at);
    let stmt = |kind: StmtKind| Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
    let hidden = Annotations { private: true, roles: GeometryRoles::default() };
    let mut out = Vec::new();
    // the source's frame: its origin and a point along each of its axes, drawn in it and held
    // there — riding where it rides, as its own origin is not (a minted point has no path)
    for (k, xy) in [("so", [0.0, 0.0]), ("su", [REACH, 0.0]), ("sv", [0.0, REACH])] {
        let p = Decl {
            annotations: hidden,
            membership: Membership::written_at(c.source.clone(), at),
            ..Decl::point(key(k), xy, None)
        };
        out.push(stmt(StmtKind::Decl(p)));
        let place = OpArg::Vector {
            key: None,
            parts: xy.iter().map(|&value| Arg::Seed { value, pinned: true }).collect(),
            span: at,
        };
        out.push(stmt(StmtKind::Relation(written("fix", Prefix, vec![local(k)], vec![place], at))));
    }
    // each carried to the roll, a point in space
    for (k, of) in [("o", local("so")), ("u", local("su")), ("v", local("sv"))] {
        let p = Decl { annotations: hidden, ..Decl::point(key(k), [0.0; 2], None) };
        out.push(stmt(StmtKind::Decl(p)));
        let ops = vec![local(k), of];
        out.push(stmt(StmtKind::Relation(written("coincident", Infix, ops, c.args.clone(), at))));
    }
    // the lines from the carried origin, and the plane over them by the name written
    for (k, to) in [("lu", "u"), ("lv", "v")] {
        let mut l = Decl::line(key(k), local("o"), local(to), Default::default());
        let roles = GeometryRoles { construction: true, centerline: false };
        l.annotations = Annotations { private: true, roles };
        out.push(stmt(StmtKind::Decl(l)));
    }
    let plane = Decl {
        annotations: d.annotations,
        kind: crate::model::EntKind::Frame,
        children: vec![vec![Kid::Ref(local("lu"))], vec![Kid::Ref(local("lv"))], Vec::new()],
        seed: vec![0.0; 3],
        seed_text: vec![None; 3],
        class: d.class.clone(),
        class_span: d.class_span,
        carried: None,
        ..Decl::point(d.name.clone(), [0.0; 2], None)
    };
    out.push(stmt(StmtKind::Decl(plane)));
    out
}
