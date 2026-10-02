//! Convert a sketch back to Solvent syntax and text.

use super::plane_of_entity;
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::syntax::{
    entity_name, num, Arg, Attitude, Decl, DeclName, Kid, Name, PlaneHint, PlaneSolve, Position,
    Program, Ref, Relation, Span, StmtKind,
};
use crate::{curve, decompose, expr};

/// The canonical program for a sketch.
///
/// Every `.json` document ever saved becomes a program through this, which is the whole of the
/// migration — and, while the parser is still being written, the whole of the bootstrap: a panel
/// can show a program before anything can read one back.
pub fn to_program(sk: &Sketch) -> Program {
    let mut p = Program::new();
    // what its numbers are in, first: every number after it is read in them (spec §3.3.2)
    if let Some(n) = sk.units.name() {
        p.push(StmtKind::Unit(Name::new(n)));
    }
    for e in sk.primitives() {
        p.push(StmtKind::Decl(lift_decl(sk, e)));
    }
    // a hinge is a solved view's own statement, and so is the row a fold `along` a line puts
    // its first end in the view by: both are spelled by the plane's clauses (`lift_view`)
    let along = along_rows(sk);
    // and a mate's row is its `against` statement's, and solids are not lifted
    let lifted = |c: &&Constraint| {
        !c.kind.hinge() && c.kind != CKind::Mate && !along.contains(&c.id)
    };
    for c in sk.user_constraints().into_iter().filter(lifted) {
        p.push(StmtKind::Relation(lift_relation(sk, c)));
    }
    // a datum point the page-placement gauge holds is not grounded: the lifted views are solved
    // again (`lift_view`), and the gauge holds it again; every other held point is said
    for i in 0..sk.points.len() {
        if sk.point_fixed(i) && !sk.page_held.contains(&(i as u32)) {
            p.push(StmtKind::Relation(lift_gauge(&entity_name(EntRef::point(i)), None)));
        }
    }
    for e in sk.primitives() {
        if e.kind == EntKind::Point {
            continue;
        }
        let own = sk.own_params(e);
        let scalars: Vec<&str> =
            e.kind.fields().iter().filter(|(_, f)| *f == Field::Scalar).map(|(n, _)| *n).collect();
        for (i, &pi) in own.iter().enumerate() {
            if sk.params[pi as usize].fixed {
                let f = scalars.get(i).copied().unwrap_or("r");
                p.push(StmtKind::Relation(lift_gauge(&entity_name(e), Some(f))));
            }
        }
    }
    for (key, &v) in &sk.branches {
        p.push(match decompose::branch_key_points(key) {
            Some(t) => StmtKind::Relation(built(
                if v >= 0 { CKind::Ccw } else { CKind::Cw },
                t.iter()
                    .map(|&i| Some(Arg::Ref(Ref::new(entity_name(EntRef::point(i))))))
                    .collect(),
            )),
            // a key that is not a triple of points has no name to travel under; it is kept
            // verbatim so a document never silently loses one
            None => StmtKind::Branch(crate::syntax::Branch { key: key.clone(), value: v }),
        });
    }
    crate::syntax::render_flat(&mut p).expect("a lifted sketch uses the printable flat subset");
    p
}

pub(crate) fn lift_decl(sk: &Sketch, e: EntRef) -> Decl {
    let kids = sk.children(e);
    let mut children: Vec<Vec<Kid>> = Vec::new();
    let mut taken = 0usize;
    for (_, field) in e.kind.fields() {
        match field {
            Field::Child => {
                children.push(
                    kids.get(taken)
                        .map(|&k| vec![Kid::Ref(Ref::new(entity_name(k)))])
                        .unwrap_or_default(),
                );
                taken += 1;
            }
            Field::List => {
                children.push(
                    kids[taken..].iter().map(|&k| Kid::Ref(Ref::new(entity_name(k)))).collect(),
                );
                taken = kids.len();
            }
            Field::Scalar => {}
        }
    }
    let seed: Vec<f64> = sk.own_params(e).iter().map(|&p| sk.seed_value(e, p)).collect();
    // a knot vector prints only when it is not the one a control polygon of that length would
    // get anyway: it is document data, and most of it says nothing
    let knots = match e.kind {
        EntKind::Spline => {
            let u = &sk.splines[e.i()].knots;
            let d = curve::clamped_uniform(sk.splines[e.i()].ctrl.len());
            (u.len() != d.len() || u.iter().zip(&d).any(|(a, b)| a != b)).then(|| u.clone())
        }
        _ => None,
    };
    let (attitude, plane) = lift_view(sk, e).unwrap_or_else(|| (lift_attitude(sk, e), Default::default()));
    Decl {
        annotations: crate::semantics::Annotations { roles: sk.roles_of(e), private: false },
        kind: e.kind,
        name: DeclName::Written(Name::new(entity_name(e))),
        children,
        seed_text: vec![None; seed.len()],
        seed_spans: vec![Span::default(); seed.len()],
        hint_span: None,
        seed,
        curve: (e.kind == EntKind::Curve).then(|| lift_curve(sk, e.i())),
        computed: None,
        knots,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
        seed_names: Vec::new(),
        attitude,
        sweep: None, motion: None, angular_span: None, plane,
        membership: lift_plane(sk, e),
        list_span: Span::default(),
        close: None,
        mint_close: None,
    }
}

/// A curve as a statement spells it: an instance written in place — the component, what it was
/// given, the swept formal at the home — and the point, over the interval.  The component's
/// own text is not in the sketch, so a lifted program parses only beside it.
fn lift_curve(sk: &Sketch, i: usize) -> crate::syntax::CurveSpec {
    use crate::syntax::{CurveSpec, CurveTarget, InstArg, InstVal, Instance};
    let cv = &sk.curves[i];
    let def = &sk.curve_defs[cv.def as usize];
    let arg = |n: &str, v: InstVal| InstArg {
        label: Some(Name::new(n)),
        value: v,
        span: Span::default(),
    };
    let mut args: Vec<InstArg> = def
        .formals
        .iter()
        .zip(&cv.args)
        .map(|((n, _), a)| arg(n, InstVal::Ref(Ref::new(entity_name(*a)))))
        .collect();
    args.extend(
        def.values
            .iter()
            .zip(&cv.values)
            .map(|(n, v)| arg(n, InstVal::Expr(crate::syntax::num(*v)))),
    );
    if let crate::model::Home::At(u) = cv.home {
        args.push(arg(&def.param, InstVal::Expr(crate::syntax::num(u))));
    }
    CurveSpec {
        target: CurveTarget::Anon(
            Instance {
                annotations: Default::default(),
                name: Name::new("#c"),
                component: Name::new(def.component.clone()),
                args,
                span: Span::default(),
                membership: Default::default(),
                class: Default::default(),
            },
            Ref::new(def.port.clone()),
        ),
        swept: Name::new(def.param.clone()),
        domain: (crate::syntax::num(cv.domain.0), crate::syntax::num(cv.domain.1)),
        of: None,
    }
}

/// A plane's attitude as a statement spells it: nothing for the page's own basis, the basis
/// itself otherwise.  A lifted plane never says `from` — the sketch holds the resolved basis
/// and not the construction it came from.
fn lift_attitude(sk: &Sketch, e: EntRef) -> Attitude {
    if e.kind != EntKind::Plane {
        return Attitude::Page;
    }
    lift_attitude_at(sk, e, sk.basis(e.i()).o)
}

/// The same, standing at `o` rather than where the plane does — for a view whose offset is
/// solved, which carries its place along the normal in `hint(offset: …)` and only the rest here.
fn lift_attitude_at(sk: &Sketch, e: EntRef, o: [f64; 3]) -> Attitude {
    let b = crate::plane::Basis { o, ..sk.basis(e.i()) };
    let page = crate::plane::Basis::page();
    let same = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    // **where it stands, as well as how it turns**: a plane stood off the shared origin — by
    // an `offset:`, a mate, a fold from one that was — keeps its origin through the lift, as
    // `o:`; one at the origin writes nothing new, so a lifted page view is `Page` as before
    let at_origin = b.o.iter().all(|x| *x == 0.0);
    if same(b.u, page.u) && same(b.v, page.v) && at_origin {
        return Attitude::Page;
    }
    let dim = |x: f64| Arg::Dim { text: num(x), span: Span::default() };
    Attitude::Basis {
        u: [dim(b.u[0]), dim(b.u[1]), dim(b.u[2])],
        v: [dim(b.v[0]), dim(b.v[1]), dim(b.v[2])],
        o: (!at_origin).then(|| [dim(b.o[0]), dim(b.o[1]), dim(b.o[2])]),
    }
}

/// **A solved view as the clauses that solve it**: the hinge it is held by spelled as the
/// fold it came from — `from: P, fold: beta` with the fold where the solve left it as its seed,
/// `from: P, fold: along l`, `from: P, offset: k` — a free attitude as `attitude: free` seeded
/// with its solved `u` and `v`, and a solved offset as `offset: free` seeded where it stands (a
/// `through: M` comes back as that, beside the `M on Q` the views pass minted for it, which says
/// the same thing).  `None` for a plane with no attitude of its own, or one that is only held —
/// a stated view a solved one reads — which `lift_attitude` states as before.
fn lift_view(sk: &Sketch, e: EntRef) -> Option<(Attitude, PlaneSolve)> {
    if e.kind != EntKind::Plane {
        return None;
    }
    let pi = e.i();
    let a = sk.planes[pi].att.as_ref()?;
    let q_free = a.q.iter().any(|&k| !sk.params[k as usize].fixed);
    let d_free = !sk.params[a.d as usize].fixed;
    let hinge = sk.constraints.iter()
        .find(|c| c.kind.hinge() && matches!(c.args[0], CArg::Ent(p) if p == e));
    if hinge.is_none() && !q_free && !d_free {
        return None;
    }
    let b = sk.basis(pi);
    let dim = |t: String| Arg::Dim { text: t, span: Span::default() };
    let hint = |key: &str, args: Vec<Arg>| PlaneHint { key: Name::new(key), args, span: Span::default() };
    let mut hints = Vec::new();
    let parent = hinge.map(|c| c.args[1].ent().i());
    let parent_ref = |p: usize| Ref::new(entity_name(EntRef::plane(p)));
    // where it stands along its own normal from the origin its attitude's clause leaves it at:
    // the parent's, for a fold or a stand-off, and the shared origin otherwise
    let from_o = parent.map_or([0.0; 3], |p| sk.basis(p).o);
    let offset = crate::plane::dot(b.normal(), crate::space::sub(b.o, from_o));
    let attitude = match hinge.map(|c| c.kind) {
        Some(CKind::Hinge) => {
            let c = hinge.expect("matched");
            let deg = |rad: f64| format!("{}deg", num(rad.to_degrees()));
            let fold = match (&c.args[2], &c.free) {
                (CArg::Expr(x), Some(f)) => {
                    let now = f.m * sk.params[f.param as usize].value + f.c;
                    hints.push(hint("fold", vec![dim(deg(now))]));
                    x.text.clone()
                }
                (CArg::Expr(x), None) => x.text.clone(),
                (a, _) => deg(a.num()),
            };
            Attitude::From { plane: parent_ref(parent.expect("a hinge names its parent")), fold: dim(fold) }
        }
        Some(CKind::HingeAlong) => Attitude::Along {
            plane: parent_ref(parent.expect("a hinge names its parent")),
            line: Ref::new(entity_name(hinge.expect("matched").args[2].ent())),
        },
        Some(_) => Attitude::Offset {
            plane: parent_ref(parent.expect("a hinge names its parent")),
            offset: (!d_free).then(|| dim(num(offset))),
        },
        None if q_free => {
            let triple = |w: [f64; 3]| w.iter().map(|&x| dim(num(x))).collect::<Vec<_>>();
            hints.push(hint("u", triple(b.u)));
            hints.push(hint("v", triple(b.v)));
            Attitude::Free { span: Span::default() }
        }
        // where it stands along the normal is the offset's hint below, so only the origin's part
        // in its own plane is the basis's
        None if d_free => {
            let n = b.normal();
            let k = crate::plane::dot(n, b.o);
            let rest = [0, 1, 2].map(|t| b.o[t] - k * n[t]);
            // what is left of an origin on the normal is roundoff, and says nothing
            let tiny = crate::space::norm(rest) <= 1e-12 * (1.0 + k.abs());
            lift_attitude_at(sk, e, if tiny { [0.0; 3] } else { rest })
        }
        None => lift_attitude(sk, e),
    };
    // a fold `along` a line stands the view where the line is, which is its position already
    let position = if d_free && !matches!(attitude, Attitude::Along { .. }) {
        hints.push(hint("offset", vec![dim(num(offset))]));
        Position::Free(Span::default())
    } else {
        Position::Stated
    };
    Some((attitude, PlaneSolve { position, hints }))
}

/// The rows a fold `along` a line put its first end in the view by — the `PointOnPlane` beside
/// each `HingeAlong` — which the plane's `fold: along l` mints again, so lifting them as
/// relations would state them twice.
fn along_rows(sk: &Sketch) -> Vec<u32> {
    let mut out = Vec::new();
    for h in sk.constraints.iter().filter(|c| c.kind == CKind::HingeAlong) {
        let (child, l) = (h.args[0].ent(), h.args[2].ent());
        let p1 = EntRef::point(sk.lines[l.i()].p1 as usize);
        if let Some(c) = sk.constraints.iter().find(|c| {
            c.kind.attitude_twin(true) == CKind::PointOnPlane
                && c.args[0].ent() == p1
                && c.args[1].ent() == child
                && !out.contains(&c.id)
        }) {
            out.push(c.id);
        }
    }
    out
}

/// The plane an entity's points are all on, when they are all on one — the clause its
/// statement writes.  A point with none, or a line whose ends are on two planes (which no one
/// statement can say), lifts without one.
pub(crate) fn lift_plane(sk: &Sketch, e: EntRef) -> crate::syntax::Membership {
    match plane_of_entity(sk, e) {
        Some(p) => crate::syntax::Membership::lifted(Ref::new(entity_name(EntRef::plane(p)))),
        None => Default::default(),
    }
}

/// A `ground` or a `fix` statement, built: `ground p` for a point, `fix c.r` for one of an
/// entity's own numbers.  What `to_program` writes for every held parameter and what
/// `edit::reconcile` appends when the app holds one.
pub(crate) fn lift_gauge(name: &str, field: Option<&str>) -> Relation {
    match field {
        None => built(CKind::Ground, vec![Some(Arg::Ref(Ref::new(name.to_string())))]),
        Some(f) => built(CKind::Fix, vec![Some(Arg::Ref(Ref::field(name.to_string(), f)))]),
    }
}

/// A relation somebody built rather than wrote: the kind and its arguments, and nothing else.
fn built(kind: CKind, args: Vec<Option<Arg>>) -> Relation {
    Relation {
        form: crate::syntax::RelationForm::Canonical { kind, args },
        place: None,
        place_span: Span::default(),
        claim: false,
        class: Default::default(),
        class_span: Span::default(),
    }
}

pub(crate) fn lift_relation(sk: &Sketch, c: &Constraint) -> Relation {
    let spec = c.kind.spec();
    let mut args: Vec<Option<Arg>> = Vec::with_capacity(spec.len());
    for (i, (_, kind)) in spec.iter().enumerate() {
        args.push(lift_arg(sk, *kind, &c.args[i]));
    }
    Relation {
        form: crate::syntax::RelationForm::Canonical { kind: c.kind, args },
        place: None,
        place_span: Span::default(),
        claim: c.claim,
        class: Default::default(),
        class_span: Span::default(),
    }
}

fn lift_arg(sk: &Sketch, kind: SpecKind, a: &CArg) -> Option<Arg> {
    Some(match a {
        CArg::Ent(e) => Arg::Ref(Ref::new(entity_name(*e))),
        // a hidden unknown travels as the number it holds, and as `==` when it was pinned: a fit
        // chose it, and a document that came back with it free would have degrees of freedom
        // nobody drew
        CArg::Param(i) => {
            Arg::Seed { value: sk.params[*i as usize].value, pinned: sk.params[*i as usize].fixed }
        }
        CArg::Seed { value, pinned } => Arg::Seed { value: *value, pinned: *pinned },
        // a dimension is written as it was written: `h = w / 2` and `3 1/8` each tell a reader
        // what 40 and 3.125 do not
        CArg::Expr(e) => Arg::Dim { text: e.text.clone(), span: Span::default() },
        CArg::Num(v) if kind.is_dimension() => {
            Arg::Dim { text: num(expr::to_user_units(kind, *v)), span: Span::default() }
        }
        CArg::Num(v) => Arg::Num(*v),
        CArg::Int(v) => Arg::Int(*v),
        CArg::Bool(b) => Arg::Bool(*b),
        CArg::Str(s) => Arg::Word(s.clone()),
    })
}
/// A sketch as a program.  The counterpart of `io::dumps`.
pub fn dumps(sk: &Sketch) -> String {
    to_program(sk).text().to_string()
}
