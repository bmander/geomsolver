//! **A frame a motion carries** (#366, Solvent §6.14 [0.133]): `n := base carried(under: m, at:
//! τ)` is the plane `base` carried by the motion `m`'s pose at the roll `τ`, its origin and its
//! axes with it.  Written out here, where a body is walked, as the statements a person would
//! write for it, so every reader after reads them as it reads any:
//!
//! * the source's frame, three points of it: its origin, and a point ten along each of its axes,
//!   drawn in it (`n#su`, `n#sv`, held there by `fix`);
//! * each carried to `τ` by the motion, a point in space (`n#o`, `n#u`, `n#v`): `X
//!   coincident(under: m, at: τ) x` (`CKind::CarriedAt`) with the declaration's own arguments;
//! * and the plane over the two lines from the carried origin, `n := plane(u: n#lu, v: n#lv)`.
//!
//! A rigid motion keeps lengths and angles, so the image is a frame as its source is, turned the
//! same way: its `u`, `v` and normal are the source's carried.  The roll is a dimension, so it may
//! be written over a face family's running number (`at: 360deg * turns * s`) and is read again at
//! every station; `std.sv`'s station words are motions read so.  The statements share the
//! declaration's id, so whatever is said of one is said where it was written, and the source
//! keeps the one statement (`syntax::Decl::carried`).

use crate::semantics::{Annotations, GeometryRoles};
use crate::syntax::{
    Arg, Chained, Decl, DeclName, Kid, Membership, Name, OpArg, Ref, Relation, RelationForm, Seg,
    Span, Stmt, StmtKind, Written,
};

/// How far along each of the source's axes the frame's second and third points stand, in the
/// document's unit: any length but none, since a rigid motion keeps it.
const REACH: f64 = 10.0;

/// `body` with every carried frame written out where it stands.
pub(super) fn expand(body: &[Stmt]) -> Vec<Stmt> {
    let mut out = Vec::with_capacity(body.len());
    for st in body {
        match &st.kind {
            StmtKind::Decl(d) if d.carried.is_some() => out.extend(frame(st, d)),
            _ => out.push(st.clone()),
        }
    }
    out
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
fn frame(st: &Stmt, d: &Decl) -> Vec<Stmt> {
    use crate::constraints::Fixity::{Infix, Prefix};
    let c = d.carried.as_ref().expect("a carried frame");
    let n = d.name.key().text.clone();
    let at = c.word.span;
    let key = |k: &str| DeclName::Key(Name { text: format!("{n}#{k}"), span: at });
    let local = |k: &str| Ref::dotted(&format!("{n}#{k}"), at);
    let member = |m: &str| {
        let mut r = c.source.clone();
        r.path.push(Seg::Field(Name { text: m.to_string(), span: r.span }));
        r
    };
    let stmt = |kind: StmtKind| Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
    let hidden = Annotations { private: true, roles: GeometryRoles::default() };
    let mut out = Vec::new();
    // the source's frame: a point along each of its axes, drawn in it and held there
    for (k, xy) in [("su", [REACH, 0.0]), ("sv", [0.0, REACH])] {
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
    for (k, of) in [("o", member("origin")), ("u", local("su")), ("v", local("sv"))] {
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
