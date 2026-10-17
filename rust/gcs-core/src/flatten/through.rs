//! **A spline through a chain's corners** (#243 L7): `rail := spline(through: rims, start: l, end:
//! m)` is the cubic through the corners of the named open chain of lines `rims`, one span a link,
//! so its parameter is `k` at the chain's corner `k` — what a smooth loft's rail is, run in step
//! with a family over a chain of as many links.  Its ends are held by tangents: along `start:` at
//! the first corner and `end:` at the last, a line or an axis, and where none is written the
//! spline runs straight out of that end (its second derivative there nothing, the natural end).
//!
//! It is short for an ordinary spline and ordinary relations, made here once every chain is known
//! (a body is a set: the chain may be written after the spline):
//!
//! * the control points, `rail.k1` … `rail.kn` between the first corner and the last, which are
//!   its first and last control points — `n + 2` for `n` corners, clamped and uniform;
//! * each corner between the ends on it where its parameter is its index, `c coincident(t == k)
//!   rail`;
//! * at each end, the control point beside the corner a third of the way (a half, over one link)
//!   from it toward the next control point — the natural end — or, given a tangent, toward that
//!   point's foot on the line through the corner along it: the least bending the tangent leaves
//!   (its second derivative square to it).  That line is run to a point as far from the corner
//!   as the next control point, so it always has a direction (#363 S3); the foot is on it, the
//!   next control point nothing along it from the foot.  The foot, that point and the lines are
//!   the spline's own, private construction.
//!
//! Each relation is the declaration's statement, its span and id, as a step's are (`steps.rs`).

use super::*;
use crate::constraints::Fixity;
use crate::model::EntKind;
use crate::syntax::{Arg, AtRef, DeclName, OpArg, Relation, RelationForm, Through, Written};

/// A spline set aside until every chain is known: the declaration as written, the statement,
/// where the walk met it and the prefix its name takes.
pub(super) struct Fit {
    pub(super) d: Decl,
    pub(super) st: Stmt,
    pub(super) scope: Scope,
    pub(super) path: Vec<PathStep>,
    pub(super) prefix: String,
}

/// `r.name`, spoken where `at` is.
fn field(r: &Ref, name: &str, at: Span) -> Ref {
    let mut out = r.clone();
    out.path.push(Seg::Field(Name { text: name.to_string(), span: at }));
    out
}

/// `a word(args) b`, said at `at`.
fn infix(word: &str, ops: [Ref; 2], args: Vec<OpArg>, at: Span) -> Relation {
    let word = Name { text: word.to_string(), span: at };
    Relation::of(RelationForm::Written(Written {
        word,
        fixity: Fixity::Infix,
        ops: ops.to_vec(),
        args,
        span: at,
        rates: Vec::new(),
    }))
}

/// `t == v`, pinned.
fn at_t(v: f64, at: Span) -> OpArg {
    OpArg::Slot {
        key: Name { text: "t".into(), span: at },
        arg: Arg::Seed { value: v, pinned: true },
    }
}

impl Walk<'_> {
    /// Every spline through a chain's corners set aside, each made now that its chain can be
    /// found — or refused, said where the chain is named.
    pub(super) fn expand_fits(&mut self) {
        for fit in std::mem::take(&mut self.aside.fits) {
            self.fit(fit);
        }
    }

    /// The chain's corners as the spline's scope reads them, in walk order — the first link's
    /// `p1`, then each link's `p2` — or why there are none: the chain named, open and of lines.
    fn corners(&mut self, t: &Through, scope: &Scope) -> Result<Vec<Ref>, (Code, String)> {
        let alias = self.alias_table();
        let r = &t.chain;
        let Some((chain, sc)) = self.chain_named(&alias, r, scope) else {
            return Err((Code::E103, format!("a spline runs `through:` the corners of a named \
                chain, `rims := line(a, b) -> line(b, c)`, and `{}` is not one", ref_text(r))));
        };
        if chain.closed {
            return Err((Code::E080, format!("`{}` is a closed chain: a spline through its corners \
                would end where it starts — run it through an open one", ref_text(r))));
        }
        let mut corners = Vec::new();
        for (k, l) in chain.links.iter().enumerate() {
            let read = self.link_read(&alias, l, &sc, scope).filter(|(full, _)| {
                self.out.iter().any(|(st, _, _)| matches!(&st.kind, StmtKind::Decl(d)
                    if d.kind == EntKind::Line && d.name.key().text == *full))
            });
            let Some((_, local)) = read else {
                return Err((Code::E080, format!("a spline runs through the corners of a chain of \
                    lines, and `{}`'s link `{}` is not a line the spline can name", ref_text(r),
                    ref_text(l))));
            };
            let link = Ref::dotted(&local, r.span);
            if k == 0 {
                corners.push(field(&link, "p1", r.span));
            }
            corners.push(field(&link, "p2", r.span));
        }
        Ok(corners)
    }

    /// A point or a line the spline makes, walked as a declaration of its scope: drawn where
    /// the spline is, and private construction where it is no control point.
    fn made(&mut self, f: &Fit, mut d: Decl, private: bool) {
        d.annotations.private = private;
        d.annotations.roles.construction = private;
        d.membership = f.d.membership.clone();
        self.walk_decl(&d, &f.st, &f.scope, &f.scope.vals, &f.path, &f.prefix);
    }

    /// A point the spline makes, `name` under it, seeded at `seed`.
    fn made_point(&mut self, f: &Fit, name: String, seed: AtRef, private: bool) {
        let at = f.d.name.span();
        let p = Decl::point(DeclName::Written(Name { text: name, span: at }), [0.0; 2], Some(seed));
        self.made(f, p, private);
    }

    /// A private construction line the spline makes, `name` under it, from `a` to `b`.
    fn made_line(&mut self, f: &Fit, name: String, a: &Ref, b: &Ref) {
        let at = f.d.name.span();
        let mut l = Decl::point(DeclName::Written(Name { text: name, span: at }), [0.0; 2], None);
        l.kind = EntKind::Line;
        l.seed = Vec::new();
        l.seed_text = Vec::new();
        l.children = vec![vec![Kid::Ref(a.clone())], vec![Kid::Ref(b.clone())]];
        self.made(f, l, true);
    }

    /// A relation the spline states, as its statement.
    fn stated(&mut self, f: &Fit, r: Relation) {
        if let Some(r2) = self.settle_relation(&r, &f.scope.vals, &f.scope) {
            self.emit(StmtKind::Relation(r2), &f.st, &f.scope, &f.path);
        }
    }

    /// One spline through a chain's corners made (see the module).
    fn fit(&mut self, mut f: Fit) {
        let Some(t) = f.d.through.take() else { return };
        let corners = match self.corners(&t, &f.scope) {
            Ok(c) => c,
            Err((code, m)) => {
                self.err(code, t.chain.span, m);
                return;
            }
        };
        let n = corners.len();
        let at = t.chain.span;
        let own = f.d.name.key().text.clone();
        let dotted = |name: &str| Ref::dotted(&format!("{own}.{name}"), at);
        // the control points between the ends, each starting at its corner, the first and the
        // last a third of the way on toward their neighbours
        let third = || Some(("1 / 3".to_string(), at));
        let mut poles = vec![corners[0].clone()];
        for i in 1..=n {
            let seed = match i {
                1 => AtRef { toward: Some(corners[1].clone()), by: third(),
                             ..AtRef::at(corners[0].clone()) },
                _ if i == n => AtRef { toward: Some(corners[n - 2].clone()), by: third(),
                                       ..AtRef::at(corners[n - 1].clone()) },
                _ => AtRef::at(corners[i - 1].clone()),
            };
            self.made_point(&f, format!("{own}.k{i}"), seed, false);
            poles.push(dotted(&format!("k{i}")));
        }
        poles.push(corners[n - 1].clone());
        let spline = Ref::dotted(&own, at);
        let mut d = f.d.clone();
        d.children = vec![poles.iter().cloned().map(Kid::Ref).collect()];
        d.knots = None;
        self.walk_decl(&d, &f.st, &f.scope, &f.scope.vals, &f.path, &f.prefix);
        // each corner between the ends where the parameter is its index
        for (k, c) in corners.iter().enumerate().take(n - 1).skip(1) {
            let r = infix("coincident", [c.clone(), spline.clone()], vec![at_t(k as f64, at)], at);
            self.stated(&f, r);
        }
        // each end: the control point beside the corner part of the way toward the next one, or
        // toward its foot on the tangent
        let part = if n >= 3 { 1.0 / 3.0 } else { 0.5 };
        let ends = [("s", &t.start, 0usize, 1usize, 2usize), ("e", &t.end, n + 1, n, n - 1)];
        for (tag, tangent, corner, beside, next) in ends {
            let (corner, beside, next) = (&poles[corner], &poles[beside], &poles[next]);
            let toward = match tangent {
                None => next.clone(),
                Some(dir) => {
                    // the foot of the next control point on the line through the corner along
                    // the tangent: that line run to a point as far from the corner as the next
                    // control point — never no length, so its direction is always read, where a
                    // line from the corner to the foot had none as the foot crossed the corner
                    // and the solve stalled there (#363 S3) — the foot on it, and its distance
                    // from the next control point along it nothing
                    let seed = || AtRef { toward: Some(next.clone()),
                                          by: Some(("1 / 2".into(), at)),
                                          ..AtRef::at(corner.clone()) };
                    let foot = dotted(&format!("#{tag}foot"));
                    self.made_point(&f, format!("{own}.#{tag}foot"), seed(), true);
                    self.made_point(&f, format!("{own}.#{tag}reach"), seed(), true);
                    let reach = dotted(&format!("#{tag}reach"));
                    self.made_line(&f, format!("{own}.#{tag}run"), corner, &reach);
                    self.made_line(&f, format!("{own}.#{tag}span"), corner, next);
                    let run = dotted(&format!("#{tag}run"));
                    let span = dotted(&format!("#{tag}span"));
                    self.stated(&f, infix("parallel", [run.clone(), dir.clone()], Vec::new(), at));
                    self.stated(&f, infix("equal", [run.clone(), span], Vec::new(), at));
                    self.stated(&f, infix("coincident", [foot.clone(), run.clone()], Vec::new(), at));
                    let along = OpArg::Named(Name { text: "along".into(), span: at }, Arg::Ref(run));
                    let nothing = OpArg::Dim("0".into(), at);
                    self.stated(&f, infix("distance", [foot.clone(), next.clone()],
                                          vec![nothing, along], at));
                    foot
                }
            };
            self.made_line(&f, format!("{own}.#{tag}part"), corner, &toward);
            let part_line = dotted(&format!("#{tag}part"));
            let r = infix("coincident", [beside.clone(), part_line], vec![at_t(part, at)], at);
            self.stated(&f, r);
        }
    }
}
