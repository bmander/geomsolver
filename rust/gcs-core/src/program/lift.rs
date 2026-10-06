//! Convert a sketch back to Solvent syntax and text.

use super::plane_of_entity;
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::syntax::{
    num, Arg, Decl, DeclName, Input, Kid, Name, ParamDecl, Program, Ref, Relation, Span, StmtKind,
    Ty,
};
use crate::{curve, decompose, expr};

/// What a lifted program calls an entity: its own name, except a plane's origin, which is the
/// plane's — `v0.origin` — since the plane mints it.
fn name(sk: &Sketch, e: EntRef) -> String {
    if e.kind == EntKind::Point {
        if let Some(i) = sk.plane_of_origin(e.i()) {
            return format!("{}.origin", crate::syntax::entity_name(EntRef::plane(i)));
        }
    }
    crate::syntax::entity_name(e)
}

/// The canonical program for a sketch.
///
/// Every `.json` document ever saved becomes a program through this, which is the whole of the
/// migration — and, while the parser is still being written, the whole of the bootstrap: a panel
/// can show a program before anything can read one back.
pub fn to_program(sk: &Sketch) -> Program {
    // a point of a 2D sketch — a plane-less sketch built in code or read from an older document
    // — has no place a document can write but a plane: it is drawn in a front plane of its own
    let flat: Vec<usize> = (0..sk.points.len())
        .filter(|&i| sk.points[i].plane.is_none() && sk.points[i].z.is_none())
        .collect();
    let owned;
    let sk = if flat.is_empty() {
        sk
    } else {
        let mut s = sk.clone();
        let front = s.fixed_plane(crate::plane::Basis::page(), "front");
        for p in flat {
            s.set_plane(p, Some(front));
        }
        owned = s;
        &owned
    };
    let mut p = Program::new();
    // what its numbers are in, first: every number after it is read in them (spec §3.3.2)
    if let Some(n) = sk.units.name() {
        p.push(StmtKind::Unit(Name::new(n)));
    }
    // the unknowns its dimensions and contacts read, declared, each seeded where it stands
    for st in unknowns(sk) {
        p.push(st);
    }
    // a plane's origin is the plane's own, minted with it and never declared apart — minted with
    // the points, in statement order, so the plane is written where its origin stands among them
    for e in sk.primitives() {
        match e.kind {
            EntKind::Plane => continue,
            EntKind::Point => match sk.plane_of_origin(e.i()) {
                Some(pl) => p.push(StmtKind::Decl(lift_decl(sk, EntRef::plane(pl)))),
                None => p.push(StmtKind::Decl(lift_decl(sk, e))),
            },
            _ => p.push(StmtKind::Decl(lift_decl(sk, e))),
        };
    }
    // and a plane written over a drawn line says so with the axis it gave it: `r parallel l`,
    // and `p coincident r` for the end it stands on (`entities::axes_along`).  Where both of a
    // plane's axes stand on one point, the plane stands there by its own rows once read back.
    let along = sk.constraints.iter().filter(|c| {
        c.intrinsic
            && (c.kind == CKind::Parallel3 && c.args[0].ent().kind == EntKind::Axis
                || c.kind == CKind::PointOnAxis)
    });
    for c in sk.user_constraints().into_iter().chain(along) {
        p.push(StmtKind::Relation(lift_relation(sk, c)));
    }
    // every held number is said, with what it is held at; a plane's origin is not, since the
    // plane holds it at its own `(0, 0)` and no `fix` says so (`holds`)
    for e in sk.primitives() {
        let held = holds(sk, e);
        if !held.is_empty() {
            p.push(StmtKind::Relation(lift_gauge(&name(sk, e), e.kind, point_len(sk, e), &held)));
        }
    }
    for (key, &v) in &sk.branches {
        p.push(match decompose::branch_key_points(key) {
            Some(t) => StmtKind::Relation(built(
                if v >= 0 { CKind::Ccw } else { CKind::Cw },
                t.iter()
                    .map(|&i| Some(Arg::Ref(Ref::new(name(sk, EntRef::point(i))))))
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

/// `param beta: Angle hint(30)` for each unknown a dimension reads (`Sketch::free_vars`) and
/// each place contacts share (`Sketch::shared`): an unknown is declared, never implied (§6.3).
fn unknowns(sk: &Sketch) -> Vec<StmtKind> {
    let free = sk.free_vars.iter().filter(|(_, &p)| !sk.params[p as usize].fixed).map(|(n, &p)| {
        (n, p, sk.free_dimensions.get(n).map_or(Ty::Scalar, |&d| Ty::of_dim(d)))
    });
    let shared = sk.shared.iter().map(|(n, s)| (n, s.param, Ty::Scalar));
    free.chain(shared)
        .map(|(name, p, ty)| {
            StmtKind::Param(ParamDecl {
                name: Name::new(name),
                text: String::new(),
                span: Span::default(),
                input: Some(Input {
                    ty: Some(ty),
                    seed: Some((num(sk.params[p as usize].value), Span::default())),
                }),
            })
        })
        .collect()
}

pub(crate) fn lift_decl(sk: &Sketch, e: EntRef) -> Decl {
    let mut kids = sk.children(e);
    // a plane's origin is its own, never written: `plane(u: r0, v: r1)`
    if e.kind == EntKind::Plane {
        kids.truncate(2);
    }
    let mut children: Vec<Vec<Kid>> = Vec::new();
    let mut taken = 0usize;
    for (_, field) in e.kind.fields() {
        match field {
            Field::Child => {
                children.push(
                    kids.get(taken)
                        .map(|&k| vec![Kid::Ref(Ref::new(name(sk, k)))])
                        .unwrap_or_default(),
                );
                taken += 1;
            }
            Field::List => {
                children.push(
                    kids[taken..].iter().map(|&k| Kid::Ref(Ref::new(name(sk, k)))).collect(),
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
    Decl {
        annotations: crate::semantics::Annotations { roles: sk.roles_of(e), private: false },
        kind: e.kind,
        name: DeclName::Written(Name::new(name(sk, e))),
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
        sweep: None, motion: None, angular_span: None,
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
        .map(|((n, _), a)| arg(n, InstVal::Ref(Ref::new(name(sk, *a)))))
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

/// The plane an entity's points are all on, when they are all on one — the clause its
/// statement writes.  A point with none, or a line whose ends are on two planes (which no one
/// statement can say), lifts without one.
pub(crate) fn lift_plane(sk: &Sketch, e: EntRef) -> crate::syntax::Membership {
    match plane_of_entity(sk, e) {
        Some(p) => crate::syntax::Membership::lifted(Ref::new(name(sk, EntRef::plane(p)))),
        None => Default::default(),
    }
}

/// The numbers of an entity's own a `fix` holds, by member and at what — written as a hint
/// writes them (`Sketch::seed_value`: a cone's half-angle in degrees).  None for a plane's
/// origin, which the plane holds at its own `(0, 0)` and no `fix` says.
pub(crate) fn holds(sk: &Sketch, e: EntRef) -> Vec<(&'static str, f64)> {
    if e.kind == EntKind::Point && sk.plane_of_origin(e.i()).is_some() {
        return Vec::new();
    }
    // an axis's place is held while nothing reads it (`Sketch::place_axis`), which is no gauge of
    // the document's: a place is stated held only once something reads it
    let own = sk.own_params(e);
    let own = if e.kind == EntKind::Axis && !sk.axes[e.i()].placed { &own[..3] } else { &own[..] };
    e.kind.members()
        .iter()
        .zip(own.iter().copied())
        .filter(|&(_, p)| sk.params[p as usize].fixed)
        .map(|(n, p)| (*n, sk.seed_value(e, p)))
        .collect()
}

/// A `fix` statement, built: `fix((0, 0)) p`, `fix(x == 3) p`, `fix(r == 25) c` — the numbers it
/// holds, each vector whole where all of it is held (`point` coordinates for a point: two in a
/// plane, three in space), else by member.  What `to_program` writes for every held entity and
/// what `edit::reconcile` appends when the app holds one.
pub(crate) fn lift_gauge(name: &str, kind: EntKind, point: usize, held: &[(&str, f64)]) -> Relation {
    use crate::syntax::{Name, OpArg, Written};
    let members = kind.members();
    let given: Vec<usize> =
        held.iter().filter_map(|(m, _)| members.iter().position(|n| n == m)).collect();
    let value = |i: usize| {
        let v = held.iter().find(|(m, _)| *m == members[i]).map_or(0.0, |(_, v)| *v);
        Arg::Seed { value: v, pinned: true }
    };
    let args = crate::syntax::said(kind, point, &given)
        .into_iter()
        .map(|s| match s {
            crate::syntax::Said::Whole(key, comps) => OpArg::Vector {
                key: (!key.is_empty()).then(|| Name::new(key)),
                parts: comps.into_iter().map(value).collect(),
                span: Span::default(),
            },
            crate::syntax::Said::One(i) => OpArg::Slot { key: Name::new(members[i]), arg: value(i) },
        })
        .collect();
    Relation {
        form: crate::syntax::RelationForm::Written(Written {
            word: Name::new("fix"),
            fixity: crate::constraints::Fixity::Prefix,
            ops: vec![Ref::new(name.to_string())],
            args,
            span: Span::default(),
        }),
        place: None,
        place_span: Span::default(),
        claim: false,
        class: Default::default(),
        class_span: Span::default(),
    }
}

/// How many coordinates a point's own vector has: two drawn in a plane, three in space.
pub(crate) fn point_len(sk: &Sketch, e: EntRef) -> usize {
    if e.kind == EntKind::Point { sk.own_params(e).len() } else { 3 }
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
        CArg::Ent(e) => Arg::Ref(Ref::new(name(sk, *e))),
        // a hidden unknown travels as the number it holds, and as `==` when it was pinned: a fit
        // chose it, and a document that came back with it free would have degrees of freedom
        // nobody drew — or as the name it is shared under (`t == s`): `Sketch::owned_arg`
        CArg::Param(i) => return lift_arg(sk, kind, &sk.owned_arg(*i)),
        CArg::Seed { value, pinned } => Arg::Seed { value: *value, pinned: *pinned },
        CArg::Shared { name, seed } => {
            Arg::Tie { name: name.clone(), seed: *seed, span: Span::default() }
        }
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
