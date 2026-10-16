//! Entity declarations, hints, styles, and plane and sweep arguments.

use super::P;
use crate::model::{EntKind, Field};
use crate::style::{Classes, Style};
use crate::syntax::lexer::Tok;
use crate::syntax::words::trails_decl;
use crate::syntax::{
    decl_head, Arg, AtRef, Decl, DeclName, DirRef, Kid, KidSeed, Membership, Name, Ref, Sense, Span, StmtKind, StyleRule, Sweep,
    Weight,
};

/// A solid's sweep arguments while the bracket list is being read.
#[derive(Default)]
pub(crate) struct SweepParts {
    /// A reading of a frame's motion's words, `under:`, `seen_from:` and `at:`: read so that
    /// their refusal names the reading that replaced them (#391 S2, `sweep_of`).
    pub(crate) under: Option<Ref>,
    pub(crate) seen_from: Option<Ref>,
    pub(crate) at: Option<Arg>,
    pub(crate) from: Option<Arg>,
    pub(crate) to: Option<Arg>,
    pub(crate) depth: Option<Arg>,
    pub(crate) about: Option<Ref>,
    pub(crate) through: Option<Ref>,
    pub(crate) sweep: Option<Arg>,
    pub(crate) sense: Option<Sense>,
    pub(crate) draft: Option<Arg>,
}

/// The labels a solid's brackets may carry beside the face or the operands — and `along`, retired,
/// read as one so that its refusal is said at the key (`sweep_arg`).
fn sweep_label(l: &str) -> bool {
    matches!(l, "from" | "to" | "depth" | "about" | "sweep" | "sense" | "through" | "along" | "under" | "at"
        | "draft" | "seen_from" | "relative_to")
}

/// `solid(tool, at: θ)`, retired (#391 S2) for the reading at an instant.
const AT_RETIRED: &str = "`solid(tool, at: θ)` is retired (#391): a solid is read at \
    an instant, `s := tool at θ`, its frames on the `at` (`tool at(under: m, seen_from: g) θ`)";

/// `solid(tool, under: m, from: a, to: b)`, retired (#391 S2) for the sweep over an interval.
pub(crate) const SWEPT_RETIRED: &str = "a solid a motion sweeps is `sweep(tool over t in (a, \
    b), under: m, seen_from: g)` (#391): `solid(tool, under: m, from: a, to: b)` is retired";

/// `solid(tool, under: m, through: body)`, retired (#391 S2) for the sweep through a body.
pub(crate) const ROLLED_RETIRED: &str = "a solid a motion rolls through a body is `sweep(tool, \
    through: body, under: m, seen_from: g)` (#391): `solid(tool, under: m, through: body)` is \
    retired";

/// What the sweep arguments a bracket list carried come to.  A solid is a prism, a revolution,
/// or a body over other solids — and a mixture is none of the three.
pub(crate) fn sweep_of(p: SweepParts) -> Result<Sweep, String> {
    // a solid a frame's motion moves is read at an instant or over an interval (#391 S2): the
    // `solid(…)` spellings of the readings are retired, each refused naming its own
    if p.at.is_some() {
        return Err(AT_RETIRED.into());
    }
    if p.under.is_some() || p.seen_from.is_some() {
        return Err(if p.through.is_some() { ROLLED_RETIRED } else { SWEPT_RETIRED }.into());
    }
    // **draft** (#245) tapers a prism's sides from the plane its face is drawn in, the parting
    // plane: nothing else is swept along a normal
    if p.draft.is_some() && (p.through.is_some() || p.about.is_some()
        || p.depth.is_none() && p.from.is_none() && p.to.is_none()) {
        return Err("`draft:` tapers a prism's sides from its face's plane: say `from:`/`to:` or \
                    `depth:`, and nothing else".into());
    }
    if let Some(body) = p.through {
        if p.from.is_some() || p.to.is_some() || p.depth.is_some() || p.about.is_some()
            || p.sweep.is_some() || p.sense.is_some() {
            return Err("`through:` cannot be combined with other sweep extents".into());
        }
        return Ok(Sweep::Through { body });
    }
    let turn = p.sense.unwrap_or_default();
    match (p.from, p.to, p.depth, p.about) {
        (None, None, None, None) => {
            if p.sweep.is_some() || p.sense.is_some() {
                return Err(
                    "`sweep` and `sense` turn a face about an axis: say `about:` too".into()
                );
            }
            Ok(Sweep::Body)
        }
        (None, None, None, Some(axis)) => Ok(Sweep::Revolve { axis, sweep: p.sweep, sense: turn }),
        (from, to, depth, None) => {
            if p.sweep.is_some() || p.sense.is_some() {
                return Err("`sweep` and `sense` turn a face about an axis, and this one is \
                            swept along its normal"
                    .into());
            }
            match (from, to, depth) {
                (Some(from), Some(to), None) => Ok(Sweep::Prism { from, to, draft: p.draft }),
                (None, None, Some(depth)) => Ok(Sweep::Depth { depth, draft: p.draft }),
                (Some(_), None, None) | (None, Some(_), None) => {
                    Err("a prism runs `from:` one ordinate `to:` another, or `depth:` behind the \
                         face"
                        .into())
                }
                _ => Err("a prism is `from:`/`to:` or `depth:`, not both".into()),
            }
        }
        _ => Err("a solid is a face swept along its normal (`from:`/`to:`, `depth:`) or turned \
                  about a line (`about:`), not both"
            .into()),
    }
}

/// Which child slot of a declaration an item of its bracket list fills: a face's edges or
/// holes; a labelled field's; or the next positional one, a List field taking every
/// positional argument from where it starts.
fn kid_slot(kind: EntKind, label: Option<&str>, positional: usize, in_holes: bool, slots: usize)
    -> usize {
    let fields = kind.fields();
    if kind == EntKind::Face {
        usize::from(in_holes)
    } else {
        match label {
            Some(l) => fields
                .iter()
                .filter(|(_, f)| *f != Field::Scalar)
                .position(|(n, _)| *n == l)
                .unwrap_or_else(|| positional.min(slots - 1)),
            None => {
                // a List field takes every positional argument from where it starts
                let n_named = fields.iter().filter(|(_, f)| *f == Field::Child).count();
                if positional >= n_named && slots > n_named {
                    n_named
                } else {
                    positional.min(slots.saturating_sub(1))
                }
            }
        }
    }
}

/// What a declaration's bracket list carried (`P::decl_list`).
struct DeclList {
    children: Vec<Vec<Kid>>,
    normal: Option<Ref>,
    sense: Option<Sense>,
    swp: SweepParts,
    surface_from: Option<Arg>,
    surface_to: Option<Arg>,
    close: Option<Span>,
    list_span: Span,
    through: Option<crate::syntax::Through>,
}

/// A declaration's scalars' seeds, as its `hint(…)` clause writes them (`P::seed_clause`): the
/// number, the text where it is no number, and where each is written.
struct Seeds {
    seed: Vec<f64>,
    text: Vec<Option<String>>,
    spans: Vec<Span>,
}

/// What a declaration's trailing clauses said (`P::decl_trailers`).  Where a clause *would* go if
/// it is not written is `insert`, the point before any of them: what writeback appends at.  How
/// an error spells the statement's head is worked out at the failure (`decl_head`), since every
/// declaration that parses would otherwise allocate a string nothing reads.
struct Trailers {
    knots: Option<Vec<f64>>,
    weights: Option<Vec<Weight>>,
    class: Classes,
    class_span: Span,
    seed_at: Option<AtRef>,
    membership: Membership,
    hint_span: Span,
    insert: usize,
}

impl<'a> P<'a> {
    /// A declaration, its kind keyword just read: what it is made of and its trailers.  The name
    /// was read before it (`name := line(…)`, `(name := line(…))`), or there is none.
    pub(super) fn decl(&mut self, kind: EntKind, name: Option<Name>) -> Option<Decl> {
        // **The name is optional** (issue #33): `line`, `line(p1, p2)` and `circle hint(r: 25)`
        // are all anonymous forms.  An anonymous declaration still needs a key the desugared
        // statements can resolve by — a chain's corner welds by *name* — so it is given one the
        // tokenizer can never produce, `#a` and its keyword's end (the flattener's block-prefix
        // device, marked apart); its span is empty at the keyword, the point a name's `name := `
        // would go, which is where `edit::reconcile` splices one the moment a statement must say
        // it (`Decl::mint_close` says when the name needs parentheses round the declaration).
        let kw = self.t.get(self.i.wrapping_sub(1)).map(|(_, s)| *s).unwrap_or_default();
        let name = match name {
            // an element keyword names a kind wherever it stands, so a reference to an element
            // called `face` would read as a new face; a param may still bear the word
            Some(n) if EntKind::parse(&n.text).is_some() => {
                let m =
                    format!("`{}` is an element keyword, and no element may be called it", n.text);
                self.fail_at(n.span, &m);
                return None;
            }
            Some(n) => DeclName::Written(n),
            None => DeclName::Key(Name {
                text: format!("#a{}", kw.hi),
                span: Span::at(kw.lo as usize),
            }),
        };
        if kind == EntKind::Envelope { return self.envelope_decl(name); }
        if kind == EntKind::Meshing { return self.meshing_decl(name); }
        // a curve is what `over` makes a definition's value, `k := trace(leg.toe over u in (a, b))`
        // — or, with the two points it runs between, a free curve whose shape its energy states
        // (`rope := curve(a, b)`, #144), read as any declaration's brackets are
        if kind == EntKind::Curve && self.peek() != Some(&Tok::P('(')) {
            self.fail(
                "a curve is `name := trace(instance.point over formal in (a, b))`, `name := \
                 trace(Component(args).point over formal in (a, b))`, or a free curve between two \
                 points, `name := curve(a, b)`",
            );
            return None;
        }
        // `p := point(x: xexpr, y: yexpr)` — a computed point (§6.5).  The brackets say what the
        // thing is made of, and this one is made of a formula: no children, no seed and no
        // trailer, since nothing on the sheet ever holds it and no solve writes it.  A point has
        // no children, so its brackets are free to hold the two coordinates.
        if kind == EntKind::Point && self.peek() == Some(&Tok::P('(')) {
            return self.computed_point(kind, name);
        }
        let scalars: Vec<&str> =
            kind.fields().iter().filter(|(_, f)| *f == Field::Scalar).map(|(n, _)| *n).collect();
        let DeclList { children, normal, sense, swp, surface_from, surface_to, close, list_span,
                       through } = self.decl_list(kind, &name, &scalars)?;
        let angular_span = match (surface_from,surface_to) {
            (None,None) => None,
            (Some(from),Some(to)) => {
                Some(crate::syntax::AngularSpan {from,to,over:None,side:None,ease:None})
            }
            _ => { self.fail("a surface span needs both `from:` and `to:` angles"); return None; }
        };
        let sweep = if kind == EntKind::Solid {
            match sweep_of(swp) {
                Ok(sw) => Some(sw),
                Err(m) => {
                    let head = decl_head(kind, &name);
                    self.fail(&format!("`{head}`: {m}"));
                    return None;
                }
            }
        } else {
            None
        };
        let mut seeds = Seeds {
            seed: vec![0.0; scalars.len()],
            text: vec![None; scalars.len()],
            spans: vec![Span::default(); scalars.len()],
        };
        let Trailers { knots, weights, class, class_span, seed_at, membership, hint_span, insert } =
            self.decl_trailers(kind, &name, &mut seeds)?;
        Some(Decl {
            annotations: Default::default(),
            kind,
            name,
            children,
            seed: seeds.seed,
            seed_text: seeds.text,
            seed_spans: seeds.spans,
            hint_span: Some(hint_span),
            knots,
            weights,
            curve: None,
            computed: None,
            class,
            class_span: if class_span.is_empty() { Span::at(insert) } else { class_span },
            seed_at,
            seed_names: Vec::new(),
            sweep,
            seen_from: None, angular_span,
            membership,
            list_span,
            close,
            mint_close: None,
            through,
            normal,
            sense,
            bare: false,
            carried: None,
        })
    }

    /// A computed point, its keyword just read and its `(` next: `p := point(x: xexpr, y: yexpr)`
    /// (§6.5).
    fn computed_point(&mut self, kind: EntKind, name: DeclName) -> Option<Decl> {
        if name.written().is_none() {
            self.fail("a computed point is named: `p := point(x: …, y: …)`");
            return None;
        }
        self.i += 1;
        let mut xy: [Option<(String, Span)>; 2] = [None, None];
        while !self.eat_p(')') {
            let Some(l) = self.slot_label() else {
                self.fail("a computed point's brackets are `(x: …, y: …)`");
                return None;
            };
            let k = match l.as_str() {
                "x" => 0,
                "y" => 1,
                _ => {
                    self.fail(&format!("a point is computed by `x:` and `y:`, not `{l}:`"));
                    return None;
                }
            };
            if xy[k].is_some() {
                self.fail(&format!("`{l}` is given twice"));
                return None;
            }
            xy[k] = Some(self.expr_until(',')?);
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        let [Some(x), Some(y)] = xy else {
            self.fail("a computed point gives both `x:` and `y:`");
            return None;
        };
        let computed = Some([x, y]);
        let end = self.prev_hi();
        let mut membership = Membership::default();
        membership.set_span(Span::at(end));
        return Some(Decl {
            annotations: Default::default(),
            kind,
            name,
            children: Vec::new(),
            seed: vec![0.0; 2],
            seed_text: vec![None; 2],
            seed_spans: vec![Span::default(); 2],
            hint_span: None,
            knots: None,
            weights: None,
            curve: None,
            computed,
            class: Classes::default(),
            class_span: Span::at(end),
            seed_at: None,
            seed_names: Vec::new(),
            sweep: None, seen_from: None, angular_span: None,
            membership,
            list_span: Span::at(end),
            close: None,
            mint_close: None,
            through: None,
            normal: None,
            sense: None,
            bare: false,
            carried: None,
        });
    }

    /// A declaration's bracket list, where one is written: what the thing is made of — its
    /// children, a solid's sweep, a surface's span, a face's `-> close`.
    fn decl_list(&mut self, kind: EntKind, name: &DeclName, scalars: &[&str]) -> Option<DeclList> {
        let mut children: Vec<Vec<Kid>> = Vec::new();
        let fields = kind.fields();
        // one slot per Child/List field, so the printer's shape and the parser's agree
        for (_, f) in fields {
            if *f != Field::Scalar {
                children.push(Vec::new());
            }
        }
        let mut swp = SweepParts::default();
        let mut surface_from = None;
        let mut surface_to = None;
        let mut close: Option<Span> = None;
        // a spline through a chain's corners: the chain, and its end tangents (#243 L7)
        let mut fit: [Option<Ref>; 3] = [None, None, None];
        // a circle about a point in space, square to an axis (#363 S6)
        let mut normal: Option<Ref> = None;
        // an arc drawn in a plane turning clockwise from its start (#363 S4)
        let mut sense: Option<Sense> = None;
        let name_end = self.prev_hi();
        let open = self.here().lo as usize;
        let mut list_span = Span::at(name_end);
        if self.eat_p('(') {
            let mut positional = 0usize;
            let mut in_holes = false;
            while !self.eat_p(')') {
                // **a face closes itself** (§6.8): `-> close` seals the loop back to the first
                // item with a straight edge, the chain's own word for the chain's own thing.
                // It fills no slot, so it is read before the labels rather than among them.
                if self.peek() == Some(&Tok::Arrow) {
                    let lo = self.here().lo as usize;
                    self.i += 1;
                    if !self.eat_word("close") {
                        self.fail("`->` in a list seals a face's loop: write `-> close`");
                        return None;
                    }
                    if kind != EntKind::Face {
                        self.fail(&format!(
                            "`-> close` seals a face's loop, and {} is not a loop",
                            kind.a()
                        ));
                        return None;
                    }
                    close = Some(Span::new(lo, self.prev_hi()));
                    self.eat_p(',');
                    if self.peek() != Some(&Tok::P(')')) {
                        self.fail("`-> close` seals the loop, so it is the last thing in the list");
                        return None;
                    }
                    continue;
                }
                // `name:` labels a field; anything else is positional
                let label = self.slot_label();
                if kind == EntKind::Face {
                    match label.as_deref() {
                        Some("holes") => in_holes = true,
                        Some("edges") if !in_holes => {},
                        Some(_) => {
                            self.fail("a face takes its edges and optional `holes:`");
                            return None;
                        }
                        None => {}
                    }
                }
                match label {
                    Some(l) if kind == EntKind::Spline
                        && matches!(l.as_str(), "through" | "start" | "end") =>
                    {
                        let k = ["through", "start", "end"].iter().position(|w| *w == l)
                            .unwrap_or(0);
                        if fit[k].is_some() {
                            self.fail(&format!("`{l}:` is given twice"));
                            return None;
                        }
                        fit[k] = Some(self.refr()?);
                    }
                    Some(l) if kind == EntKind::Arc && l == "sense" => {
                        if sense.is_some() {
                            self.fail("`sense:` is given twice");
                            return None;
                        }
                        sense = Some(self.sense_word()?);
                    }
                    Some(l) if kind == EntKind::Circle && l == "normal" => {
                        if normal.is_some() {
                            self.fail("`normal:` is given twice");
                            return None;
                        }
                        normal = Some(self.refr()?);
                    }
                    Some(l) if kind == EntKind::Surface && (l == "from" || l == "to") => {
                        let slot = if l == "from" { &mut surface_from } else { &mut surface_to };
                        if slot.is_some() { self.fail("a surface bound is given twice"); return None; }
                        let (text,span) = self.expr_until(',')?;
                        *slot = Some(Arg::Dim {text,span});
                    }
                    // **a solid's sweep is what it is made of**, so it stands in the brackets
                    // with the face
                    Some(l) if kind == EntKind::Solid && sweep_label(&l) => {
                        self.sweep_arg(&l, &mut swp)?;
                    }
                    // the brackets after the name are *what the thing is made of*; where the
                    // solve begins is the `hint(…)` after them (spec §6.4)
                    Some(l) if scalars.contains(&l.as_str()) => {
                        let head = decl_head(kind, name);
                        self.fail(&format!(
                            "`{l}` is a seed, and a seed goes in a `hint(…)` clause: \
                             `{head}(…) hint({l}: …)`"
                        ));
                        return None;
                    }
                    _ => {
                        // A slot names an entity, seeds a point, or holds a solid's inline
                        // section. An entity whose children are all unseeded writes no list.
                        let kid = self.list_kid(kind, label.as_deref(), positional)?;
                        let slots = children.len();
                        let slot = kid_slot(kind, label.as_deref(), positional, in_holes, slots);
                        if let Some(g) = children.get_mut(slot) {
                            g.push(kid);
                        }
                        positional += 1;
                    }
                }
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
            list_span = Span::new(open, self.prev_hi());
        }
        let [chain, start, end] = fit;
        let through = match chain {
            Some(_) if children.iter().any(|g| !g.is_empty()) => {
                self.fail("a spline through a chain's corners makes its own control points: \
                           write `spline(through: chain)` or the control points, not both");
                return None;
            }
            Some(chain) => Some(crate::syntax::Through { chain, start, end }),
            None if start.is_some() || end.is_some() => {
                self.fail("`start:` and `end:` hold the ends of a spline through a chain's \
                           corners: `spline(through: chain, start: l)`");
                return None;
            }
            None => None,
        };
        Some(DeclList { children, normal, sense, swp, surface_from, surface_to, close, list_span,
                        through })
    }

    /// One item of a declaration's bracket list, its label (if any) read: a slot names an
    /// entity, seeds a point, or holds a solid's inline section — or a face's stretch of a
    /// curve.
    fn list_kid(&mut self, kind: EntKind, label: Option<&str>, positional: usize) -> Option<Kid> {
        Some(match self.eat_hint_clause() {
            Some(lo) => {
                // a plane's `u:` and `v:` are axes, seeded by their direction
                let axis = kind == EntKind::Frame && match label {
                    Some(l) => l == "u" || l == "v",
                    None => positional < 2,
                };
                Kid::Hint(self.kid_seed(lo, axis)?)
            }
            None if self.peek_word("face")
                && self.t.get(self.i + 1).map(|t| &t.0) == Some(&Tok::P('(')) =>
            {
                if kind != EntKind::Solid {
                    self.fail("an inline face is a solid's section: \
                               write `name := solid(face(…), …)`");
                    return None;
                }
                self.i += 1;
                let decl = self.decl(EntKind::Face, None)?;
                Kid::Face { span: decl.list_span, decl: Box::new(decl) }
            }
            None => {
                let r = self.refr()?;
                // `flank from p to q`: the stretch of a curve a face runs along
                if kind == EntKind::Face && self.peek_word("from") {
                    let lo = r.span.lo as usize;
                    self.i += 1;
                    let from = self.refr()?;
                    if !self.eat_word("to") {
                        self.fail("a face's curve runs `from` one point `to` another: `flank from p to q`");
                        return None;
                    }
                    let to = self.refr()?;
                    Kid::Trim { curve: r, from, to, span: Span::new(lo, self.prev_hi()) }
                } else {
                    Kid::Ref(r)
                }
            }
        })
    }

    /// A declaration's trailing clauses, in any order: `hint(…)` (its seeds written into `seeds`),
    /// `knots [...]`, `weights [...]`, `class …`, `in PLANE`.
    fn decl_trailers(
        &mut self,
        kind: EntKind,
        name: &DeclName,
        seeds: &mut Seeds,
    ) -> Option<Trailers> {
        // trailing clauses, in any order: `hint(…)`, `knots [...]`, `weights [...]`, `class …`,
        // `in PLANE`.
        // Where a clause *would* go if it is not written is the point we are standing on now,
        // before any of them: that is what writeback appends at.
        let mut knots = None;
        let mut weights = None;
        let mut class = Classes::default();
        let mut class_span = Span::default();
        let mut seed_at: Option<AtRef> = None;
        let mut membership = Membership::default();
        let insert = self.prev_hi();
        let mut hint_span = Span::at(insert);
        loop {
            if let Some(lo) = self.eat_hint_clause() {
                self.seed_clause(kind, seeds, &mut seed_at)?;
                hint_span = Span::new(lo, self.prev_hi());
            } else if self.eat_word("knots") {
                knots = Some(self.knot_list()?);
            } else if self.eat_word("weights") {
                weights = Some(self.weight_list()?);
            } else if self.peek_word("class") {
                let (c, sp) = self.class_clause(insert);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
                class_span = sp;
            } else if self.peek_word("in") {
                membership = self.membership_clause(kind, &name, &membership)?;
            } else if self.peek_word("hint") || self.peek_word("at") {
                self.retired_seed(kind, &name);
                return None;
            } else {
                break;
            }
        }
        // where an `in` clause would go: after every trailer, so an appended one never races
        // `class_span` for one offset
        if membership.span().is_empty() {
            let end = self.prev_hi();
            membership.set_span(Span::at(end));
        }
        Some(Trailers {
            knots,
            weights,
            class,
            class_span,
            seed_at,
            membership,
            hint_span,
            insert,
        })
    }

    /// A declaration's `hint(…)` clause, its `hint(` just read: its scalars' seeds, or a place
    /// named geometrically (`seed_at`).
    fn seed_clause(
        &mut self,
        kind: EntKind,
        seeds: &mut Seeds,
        seed_at: &mut Option<AtRef>,
    ) -> Option<()> {
        // `hint((0, 12))` — keyed, keys in any order, an omitted scalar is 0 — or a
        // place named geometrically, `hint(at: t)`, `hint(at: c, bearing: u + phase)`,
        // `hint(at: a, toward: b, by: 0.5, turn: 90deg)`, `hint(at: a, along: l, by: 2)`:
        // the same clause, since a seed is what is inside one and nothing else is
        // (§4.3), and a place is a seed given as geometry rather than as numbers.  The
        // place keys stand beside the scalars; a clause naming a place carries no
        // coordinate, and every other place key says something about the place `at:`
        // names, so it needs one.
        let mut at: Option<Ref> = None;
        let mut toward: Option<Ref> = None;
        let mut along: Option<Ref> = None;
        let mut texts: [Option<(String, Span, Span)>; 3] = [None, None, None];
        let mut coord: Option<Span> = None;
        // `x:` and `y:` as written, which beside `at:` a plane are a place in it
        let mut xy: [Option<(String, Span)>; 3] = [None, None, None];
        let mut z_at: Option<Span> = None;
        let mut other_coord: Option<Span> = None;
        for h in self.hint_body("x: 0, y: 0")? {
            if let Some(what) = h.place {
                let slot = match h.key.as_str() {
                    "at" => &mut at,
                    "toward" => &mut toward,
                    "along" => &mut along,
                    _ => {
                        let m = format!(
                            "`{}:` reads a direction off geometry, which seeds a plane's \
                             axis: `plane(v: hint(dir: up))`; `{}` seeds its scalars",
                            h.key, kind.as_str()
                        );
                        self.fail_at(h.at, &m);
                        continue;
                    }
                };
                if slot.is_some() {
                    self.fail_at(h.at, &format!("`{}:` is written twice", h.key));
                    continue;
                }
                *slot = Some(what);
                continue;
            }
            if let Some(k) = ["bearing", "by", "turn"].iter().position(|&w| w == h.key) {
                if texts[k].is_some() {
                    self.fail_at(h.at, &format!("`{}:` is written twice", h.key));
                    continue;
                }
                texts[k] = Some((h.text, h.span, h.at));
                continue;
            }
            let Some(i) = kind.members().iter().position(|&s| s == h.key) else {
                // the key is the mistake, not the declaration: reported, and the rest
                // of the clause read on, so the entity is still declared and no
                // statement naming it fails for want of it (#43.19)
                let m = format!("`{}` has no scalar `{}` to seed", kind.as_str(), h.key);
                self.fail_at(h.at, &m);
                continue;
            };
            coord.get_or_insert(h.at);
            match h.key.as_str() {
                "x" => xy[0] = Some((h.text.clone(), h.span)),
                "y" => xy[1] = Some((h.text.clone(), h.span)),
                // a place in a spatial frame has a third, out along it (#383) — beside the
                // other two, never alone
                "z" => {
                    xy[2] = Some((h.text.clone(), h.span));
                    z_at = Some(h.at);
                }
                _ => {
                    other_coord.get_or_insert(h.at);
                }
            }
            seeds.seed[i] = h.value.unwrap_or(0.0);
            seeds.text[i] = (h.value.is_none()).then_some(h.text);
            seeds.spans[i] = h.span;
        }
        if xy[0].is_none() && xy[1].is_none() {
            other_coord = other_coord.or(z_at);
        }
        let [bearing, by, turn] = texts;
        let key_at = |t: &Option<(String, Span, Span)>| t.as_ref().map(|t| t.2);
        let text = |t: Option<(String, Span, Span)>| t.map(|(text, span, _)| (text, span));
        match at {
            Some(what) => {
                // a place in a plane, `hint(at: P, (3, 4))`: the plane's own
                // coordinates, read where the point is seen; any other scalar beside a
                // place is refused
                let in_plane = kind == EntKind::Point && coord.is_some() && other_coord.is_none();
                if let (Some(sp), false) = (coord, in_plane) {
                    let m = "`at:` names the place; a clause with it carries no scalar \
                             but a place's `x:` and `y:` in a plane";
                    self.fail_at(sp, m);
                }
                let step = toward.as_ref().or(along.as_ref()).map(|r| r.span);
                if let (true, Some(sp)) = (in_plane, step.or(key_at(&bearing))) {
                    let m = "`x:` and `y:` are a place in the plane `at:` names, and a \
                             step or a bearing is from a point: one or the other";
                    self.fail_at(sp, m);
                }
                if let (Some(_), Some(sp)) = (&toward, along.as_ref().map(|r| r.span)) {
                    let m = "a step is `toward:` a point or `along:` a line: one or the \
                             other";
                    self.fail_at(sp, m);
                }
                if let (Some(sp), Some(_)) = (key_at(&bearing), step) {
                    let m = "`bearing:` is a place on a circle's edge and a step is from \
                             a point: one or the other";
                    self.fail_at(sp, m);
                }
                if step.is_none() {
                    if let Some(sp) = key_at(&by).or(key_at(&turn)) {
                        let m = "`by:` and `turn:` say how far a step goes, and need \
                                 `toward:` or `along:`";
                        self.fail_at(sp, m);
                    }
                }
                if seed_at.is_some() {
                    self.fail_at(what.span, "the place is seeded twice");
                }
                *seed_at = Some(AtRef {
                    what,
                    bearing: text(bearing),
                    toward,
                    along,
                    by: text(by),
                    turn: text(turn),
                    x: if in_plane { xy[0].take() } else { None },
                    y: if in_plane { xy[1].take() } else { None },
                    z: if in_plane { xy[2].take() } else { None },
                });
                if in_plane {
                    // the place carries the numbers; the scalars stay unseeded
                    for i in 0..seeds.seed.len() {
                        seeds.seed[i] = 0.0;
                        seeds.text[i] = None;
                    }
                }
            }
            None => {
                let lone = [
                    ("toward", toward.as_ref().map(|r| r.span)),
                    ("along", along.as_ref().map(|r| r.span)),
                    ("bearing", key_at(&bearing)),
                    ("by", key_at(&by)),
                    ("turn", key_at(&turn)),
                ];
                let first = lone.into_iter().find(|(_, sp)| sp.is_some());
                if let Some((key, Some(sp))) = first {
                    let m = format!("`{key}:` says where from a place, and needs `at:`");
                    self.fail_at(sp, &m);
                }
            }
        }
        Some(())
    }

    /// A spline's `knots [...]`, the word just read.
    fn knot_list(&mut self) -> Option<Vec<f64>> {
        if !self.want_p('[') {
            return None;
        }
        let mut u = Vec::new();
        while !self.eat_p(']') {
            u.push(self.number()?);
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(']')) {
                self.fail("expected `,` or `]`");
                return None;
            }
        }
        Some(u)
    }

    /// A rational spline's `weights [...]`, the word just read.
    fn weight_list(&mut self) -> Option<Vec<Weight>> {
        if !self.want_p('[') {
            return None;
        }
        let mut w = Vec::new();
        while !self.eat_p(']') {
            let (value, text, span) = self.value_text()?;
            w.push(match value {
                Some(value) => Weight { value, text: None, span },
                None => Weight { value: f64::NAN, text: Some(text), span },
            });
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(']')) {
                self.fail("expected `,` or `]`");
                return None;
            }
        }
        Some(w)
    }

    /// A declaration's `in PLANE[, G…]` clause, `in` next: the membership it writes, where the
    /// declaration has said none yet.
    fn membership_clause(
        &mut self,
        kind: EntKind,
        name: &DeclName,
        membership: &Membership,
    ) -> Option<Membership> {
        // `in top` — every point this declaration mints or names is an image on that
        // plane (§6.7).  A datum has no points of its own to put there, and a curve is
        // its expressions.
        if !kind.bears_points() {
            self.fail(&format!(
                "`in` puts points on a plane, and {} has none of its own",
                kind.a()
            ));
            return None;
        }
        if membership.plane().is_some() {
            let head = decl_head(kind, name);
            self.fail(&format!("`{head}` is {}", membership.cause()));
            return None;
        }
        let lo = self.here().lo as usize;
        self.i += 1;
        let r = self.refr()?;
        // `in P, G` — on every plane named, so where they meet, and drawn in each (§6.7)
        let mut also = Vec::new();
        while self.peek() == Some(&Tok::P(','))
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(_)))
        {
            self.i += 1;
            also.push(self.refr()?);
        }
        if !also.is_empty() && kind != EntKind::Point {
            self.fail(&format!(
                "a point is drawn in several planes, where they meet; {} is drawn in \
                 one, and on two its points are",
                kind.a()
            ));
            return None;
        }
        Some(Membership::written_on(r, also, Span::new(lo, self.prev_hi())))
    }

    /// A retired seed spelling, `hint at …` or `at (…)` after a declaration: refused, saying
    /// what to write.
    fn retired_seed(&mut self, kind: EntKind, name: &DeclName) {
        // the retired spellings — `hint at (0, 0)` and a bare `at (0, 0)` for a pair of
        // coordinates, `hint at REF [bearing (…)]` for a place (#47.2) — are what every
        // document said until the clause took them in, so the reader most likely to meet
        // an error here is the one holding one of them
        let head = decl_head(kind, name);
        let place = self.peek_word("hint")
            && self.word_at(self.i + 1) == Some("at")
            && self.t.get(self.i + 2).map(|(t, _)| t) != Some(&Tok::P('('));
        let m = if place {
            format!("a place is keyed now: `{head} hint(at: REF, bearing: …)`")
        } else {
            format!("a coordinate seed is keyed now: `{head} hint((…, …))`")
        };
        self.fail(&m);
    }

    /// Which way, after a `sense:` label: a word, never a sign (§9.2) — a revolution's turn, an
    /// arc's (#363 S4).
    fn sense_word(&mut self) -> Option<Sense> {
        let w = self.ident()?;
        match w.text.as_str() {
            "ccw" => Some(Sense::Ccw),
            "cw" => Some(Sense::Cw),
            other => {
                self.fail(&format!("`sense` is `cw` or `ccw`, not `{other}`"));
                None
            }
        }
    }

    /// One of a solid's sweep arguments, the label already eaten: `from: EXPR`, `to: EXPR`,
    /// `depth: EXPR`, `about: REF`, `sweep: EXPR`, `sense: cw|ccw`.
    fn sweep_arg(&mut self, label: &str, parts: &mut SweepParts) -> Option<()> {
        let twice = |s: &mut Self| {
            s.fail(&format!("`{label}` is given twice"));
            None
        };
        match label {
            "under" => {
                if parts.under.is_some() { return twice(self); }
                parts.under = Some(self.refr()?);
            }
            "seen_from" => {
                if parts.seen_from.is_some() { return twice(self); }
                parts.seen_from = Some(self.refr()?);
            }
            "relative_to" => {
                self.fail(super::RELATIVE_TO);
                return None;
            }
            // retired (#243, L3): a loft is a face family, its section a component standing on
            // its guide, caught here so a document written against 0.99 says what to write
            "along" => {
                self.fail("`along:` is retired: a loft is a section drawn at a station of its guide, \
                           swept as the station runs — `sweep(I(g, …).f over s in (0, 1))`, `I` \
                           standing `std.OnLine(g, up: …, s: s)` or `std.OnArc` (spec §6.9)");
                return None;
            }
            "through" => {
                if parts.through.is_some() { return twice(self); }
                parts.through = Some(self.refr()?);
            }
            "about" => {
                if parts.about.is_some() {
                    return twice(self);
                }
                parts.about = Some(self.refr()?);
            }
            // **a selector is a word, never a sign** (§9.2): `sense: cw`, not a negative sweep
            "sense" => {
                if parts.sense.is_some() {
                    return twice(self);
                }
                parts.sense = Some(self.sense_word()?);
            }
            _ => {
                let slot = match label {
                    "at" => &mut parts.at,
                    "from" => &mut parts.from,
                    "to" => &mut parts.to,
                    "depth" => &mut parts.depth,
                    "draft" => &mut parts.draft,
                    _ => &mut parts.sweep,
                };
                if slot.is_some() {
                    return twice(self);
                }
                let (text, span) = self.expr_until(',')?;
                *slot = Some(Arg::Dim { text, span });
            }
        }
        Some(())
    }

}
impl<'a> P<'a> {
    /// `over t in (a, b)`, or `over t` (#391 S2), `over` next: the parameter a reading over an
    /// interval names and the ends of that interval — written in place, or a declared
    /// parameter's, which the flattener writes in (left empty here, at the name).
    pub(super) fn runs_over(&mut self) -> Option<(crate::syntax::RunsOver, Arg, Arg)> {
        self.i += 1; // `over`
        let name = self.ident()?;
        if !self.eat_word("in") {
            let end = || Arg::Dim { text: String::new(), span: name.span };
            let (a, b) = (end(), end());
            return Some((crate::syntax::RunsOver { name, declared: true }, a, b));
        }
        let [(a, a_span), (b, b_span)] = self.pair()?;
        Some((crate::syntax::RunsOver { name, declared: false },
              Arg::Dim { text: a, span: a_span }, Arg::Dim { text: b, span: b_span }))
    }

    fn envelope_decl(&mut self, name: DeclName) -> Option<Decl> {
        let start = self.here();
        if !self.want_p('(') { return None; }
        let mut surface = None; let mut motion = None; let mut seen_from = None;
        let mut from = None; let mut to = None; let mut side = None; let mut over = None;
        while !self.eat_p(')') {
            let label = self.slot_label();
            match label.as_deref() {
                Some("surface") | Some("under") | Some("motion") | None => {
                    let tool = match label.as_deref() {
                        Some("surface") => true,
                        None => surface.is_none(),
                        _ => false,
                    };
                    let slot = match label.as_deref() {
                        Some("surface") => &mut surface,
                        Some("under") | Some("motion") => &mut motion,
                        _ if surface.is_none() => &mut surface,
                        _ => &mut motion,
                    };
                    if slot.is_some() { self.fail("an envelope argument is given twice"); return None; }
                    *slot = Some(Kid::Ref(self.refr()?));
                    // the tool and the interval of the roll it runs over (#391 S2)
                    if self.peek_word("over") {
                        if !tool || over.is_some() {
                            self.fail("an envelope runs its tool over the roll: `envelope(tool \
                                       over t in (a, b), …)`");
                            return None;
                        }
                        let (o, a, b) = self.runs_over()?;
                        (over, from, to) = (Some(o), Some(a), Some(b));
                    }
                }
                Some("from") | Some("to") => {
                    self.fail("an envelope's `from:`/`to:` are retired (#391): its tool runs over \
                               an interval of the roll, `envelope(tool over t in (a, b), …)`");
                    return None;
                }
                Some("side") => {
                    if side.is_some() { self.fail("an envelope's side is given twice"); return None; }
                    side = Some(self.refr()?.root);
                }
                Some("seen_from") => {
                    if seen_from.is_some() {
                        self.fail("an envelope's observer is given twice"); return None;
                    }
                    seen_from = Some(self.refr()?);
                }
                _ => {
                    self.fail("an envelope takes a surface or tool over an interval of the roll \
                               (`tool over t in (a, b)`), `under:` a frame (`seen_from:` another), \
                               and `side:`");
                    return None;
                }
            }
            if self.peek() != Some(&Tok::P(')')) && !self.want_p(',') { return None; }
        }
        // a tool that rides leaves `under:` out (#383): the elaborator reads what it rides on
        let (Some(surface),Some(from),Some(to)) = (surface,from,to) else {
            self.fail("an envelope needs a surface or tool and the interval of the roll it runs \
                       over: `envelope(tool over t in (a, b), …)`"); return None;
        };
        let end = self.prev_hi();
        let (class,class_span) = self.class_clause(end);
        Some(Decl {
            annotations:Default::default(),kind:EntKind::Envelope,name,
            children:vec![vec![surface],motion.into_iter().collect()],
            seed:vec![],seed_text:vec![],seed_spans:vec![],hint_span:None,knots:None,weights:None,curve:None,
            computed:None,class,class_span,seed_at:None,seed_names:vec![],sweep:None,seen_from,
            angular_span:Some(crate::syntax::AngularSpan {from,to,over,side,ease:None}),
            membership:Membership::default(),list_span:Span::new(start.lo as usize,end),close:None,mint_close:None,through:None,normal:None,sense:None,bare:false,carried:None,
        })
    }

    /// `contact := meshing(pinion_flank, gear_flank over t in (a, b), under: mesh)` (#283): two
    /// flanks, the relative motion they turn under, and the pinion's phase interval — and the
    /// first flank's ease, `ease: M, along: l, profile: κₚ, lengthwise: κₗ` (#345).
    fn meshing_decl(&mut self, name: DeclName) -> Option<Decl> {
        let start = self.here();
        if !self.want_p('(') { return None; }
        let (mut first, mut second, mut motion) = (None, None, None);
        let mut seen_from = None;
        let mut from = None; let mut to = None; let mut over = None;
        let (mut about, mut along, mut profile, mut lengthwise) = (None, None, None, None);
        while !self.eat_p(')') {
            let label = self.slot_label();
            match label.as_deref() {
                // the printer labels every operand (`first:`, `second:`, `motion:`)
                None | Some("first") | Some("second") | Some("under") | Some("motion") => {
                    let second_flank = match label.as_deref() {
                        Some("second") => true,
                        None => first.is_some() && second.is_none(),
                        _ => false,
                    };
                    let slot = match label.as_deref() {
                        Some("first") => &mut first,
                        Some("second") => &mut second,
                        Some("under") | Some("motion") => &mut motion,
                        _ if first.is_none() => &mut first,
                        _ if second.is_none() => &mut second,
                        _ => &mut motion,
                    };
                    if slot.is_some() { self.fail("a meshing argument is given twice"); return None; }
                    *slot = Some(Kid::Ref(self.refr()?));
                    // the second flank and the interval of the roll the first's phase runs over
                    // (#391 S2)
                    if self.peek_word("over") {
                        if !second_flank || over.is_some() {
                            self.fail("a meshing runs its second flank over the roll: \
                                       `meshing(f1, f2 over t in (a, b), …)`");
                            return None;
                        }
                        let (o, a, b) = self.runs_over()?;
                        (over, from, to) = (Some(o), Some(a), Some(b));
                    }
                }
                Some("from") | Some("to") => {
                    self.fail("a meshing's `from:`/`to:` are retired (#391): its phase runs over \
                               an interval of the roll, `meshing(f1, f2 over t in (a, b), …)`");
                    return None;
                }
                Some("ease") | Some("along") => {
                    let ease = label.as_deref() == Some("ease");
                    let slot = if ease { &mut about } else { &mut along };
                    if slot.is_some() { self.fail("a meshing's ease is given twice"); return None; }
                    *slot = Some(self.refr()?);
                }
                Some("seen_from") => {
                    if seen_from.is_some() {
                        self.fail("a meshing's observer is given twice"); return None;
                    }
                    seen_from = Some(self.refr()?);
                }
                Some("profile") | Some("lengthwise") => {
                    let across = label.as_deref() == Some("profile");
                    let slot = if across { &mut profile } else { &mut lengthwise };
                    if slot.is_some() { self.fail("a meshing's ease is given twice"); return None; }
                    let (text,span) = self.expr_until(',')?;
                    *slot = Some(Arg::Dim {text,span});
                }
                _ => {
                    self.fail("a meshing takes two flanks, the second over the interval of the \
                               roll the first's phase runs over (`f2 over t in (a, b)`), `under:` \
                               the second's frame `seen_from:` the first's, and the first's ease: \
                               `ease:` a point, `along:` a line, `profile:` and `lengthwise:` \
                               curvatures");
                    return None;
                }
            }
            if self.peek() != Some(&Tok::P(')')) && !self.want_p(',') { return None; }
        }
        // flanks that ride leave `under:` out (#383): each is read on its own frames
        let (Some(first),Some(second),Some(from),Some(to)) = (first,second,from,to) else {
            self.fail("a meshing needs two flanks and the interval of the roll it runs over: \
                       `meshing(f1, f2 over t in (a, b), …)`"); return None;
        };
        let ease = match (about,along,profile,lengthwise) {
            (None,None,None,None) => None,
            (Some(about),Some(along),Some(profile),Some(lengthwise)) => {
                Some(Box::new(crate::syntax::EaseSpec {about,along,profile,lengthwise}))
            }
            _ => {
                self.fail("a meshing's ease needs `ease:` the point it is about, `along:` the line \
                           it runs lengthwise along, and `profile:` and `lengthwise:` its curvatures");
                return None;
            }
        };
        let end = self.prev_hi();
        let (class,class_span) = self.class_clause(end);
        Some(Decl {
            annotations:Default::default(),kind:EntKind::Meshing,name,
            children:vec![vec![first],vec![second],motion.into_iter().collect()],
            seed:vec![],seed_text:vec![],seed_spans:vec![],hint_span:None,knots:None,weights:None,curve:None,
            computed:None,class,class_span,seed_at:None,seed_names:vec![],sweep:None,seen_from,
            angular_span:Some(crate::syntax::AngularSpan {from,to,over,side:None,ease}),
            membership:Membership::default(),list_span:Span::new(start.lo as usize,end),close:None,mint_close:None,through:None,normal:None,sense:None,bare:false,carried:None,
        })
    }

    /// `f := frame` (#383): a spatial frame declared bare.  It is made of nothing, so it has no
    /// brackets, no seed and no place in a plane: what it is, relations say after the fact —
    /// its axes `f.u`, `f.v` and its origin `f.origin` are minted where they are named.  A class
    /// is presentation, and taken.
    pub(super) fn frame_decl(&mut self, name: DeclName) -> Option<Decl> {
        if self.peek() == Some(&Tok::P('(')) {
            self.fail("a frame is declared bare, `f := frame`, and constrained after the fact: \
                       name its axes `f.u`, `f.v` and its origin `f.origin` in relations (a flat \
                       frame over two axes is `plane(u: r1, v: r2)`)");
            return None;
        }
        let kw = self.t.get(self.i.wrapping_sub(1)).map(|(_, s)| *s).unwrap_or_default();
        let scalars: Vec<&str> = EntKind::Frame.fields().iter()
            .filter(|(_, f)| *f == Field::Scalar).map(|(n, _)| *n).collect();
        let mut seeds = Seeds {
            seed: vec![0.0; scalars.len()],
            text: vec![None; scalars.len()],
            spans: vec![Span::default(); scalars.len()],
        };
        let Trailers { class, class_span, seed_at, membership, hint_span, insert, .. } =
            self.decl_trailers(EntKind::Frame, &name, &mut seeds)?;
        if !hint_span.is_empty() || seed_at.is_some() {
            self.fail_at(hint_span, "a frame declared bare has no seed: hint its axes and its \
                                     origin where they are named");
            return None;
        }
        let fields = EntKind::Frame.fields().iter().filter(|(_, f)| *f != Field::Scalar).count();
        Some(Decl {
            annotations: Default::default(),
            kind: EntKind::Frame,
            name,
            children: vec![Vec::new(); fields],
            seed: seeds.seed,
            seed_text: seeds.text,
            seed_spans: seeds.spans,
            hint_span: None,
            knots: None,
            weights: None,
            curve: None,
            computed: None,
            class,
            class_span: if class_span.is_empty() { Span::at(insert) } else { class_span },
            seed_at: None,
            seed_names: Vec::new(),
            sweep: None,
            seen_from: None,
            angular_span: None,
            membership,
            list_span: Span::at(kw.hi as usize),
            close: None,
            mint_close: None,
            through: None,
            normal: None,
            sense: None,
            bare: true,
            carried: None,
        })
    }


    /// A `hint(…)` standing in a child slot, the opening paren already eaten.
    ///
    /// The same clause as everywhere else, so it is read by the same `hint_body`; what the keys
    /// mean is this table — an anonymous child is a point, `hint((3, 4))`, or a plane's axis,
    /// `hint(dir: (1, 0, 0))`.
    fn kid_seed(&mut self, lo: usize, axis: bool) -> Option<KidSeed> {
        let mut k = KidSeed { axis, ..KidSeed::default() };
        // a direction given as geometry: what it reads, what it turns about, and by how much
        let (mut what, mut about, mut turn) = (None, None, None);
        let mut numbers: Option<Span> = None;
        for h in self.hint_body("(0, 0)")? {
            if axis {
                let slot = match h.key.as_str() {
                    "dir" if h.place.is_some() => Some(&mut what),
                    "about" => Some(&mut about),
                    "turn" => {
                        if turn.replace((h.text, h.span)).is_some() {
                            self.fail_at(h.at, "`turn:` is written twice");
                        }
                        continue;
                    }
                    _ => None,
                };
                if let Some(slot) = slot {
                    if slot.replace(h.place.expect("a direction's keys are references")).is_some() {
                        self.fail_at(h.at, &format!("`{}:` is written twice", h.key));
                    }
                    continue;
                }
            }
            numbers.get_or_insert(h.at);
            let i = match (axis, h.key.as_str()) {
                (false, "x") | (true, "dir.x") => 0,
                (false, "y") | (true, "dir.y") => 1,
                // a point's third is refused where it is built
                (false, "z") | (true, "dir.z") => 2,
                _ => {
                    let m = match axis {
                        true => format!("an axis's seed is its direction, `hint(dir: (1, 0, 0))`, \
                                         not `{}`", h.key),
                        false => format!("a point's seed is its place, `hint((0, 0))`, not `{}`",
                                         h.key),
                    };
                    self.fail_at(h.at, &m);
                    return None;
                }
            };
            k.v[i] = h.value.unwrap_or(0.0);
            k.text[i] = (h.value.is_none()).then_some(h.text);
            k.spans[i] = h.span;
        }
        k.span = Span::new(lo, self.prev_hi());
        match what {
            Some(what) => {
                if let Some(at) = numbers {
                    self.fail_at(at, "`dir:` names the direction; a clause with it carries no \
                                      number but how far it is turned, `turn:`");
                    return None;
                }
                if about.is_some() != turn.is_some() {
                    let sp = about.as_ref().map(|r| r.span).or(turn.as_ref().map(|t| t.1));
                    self.fail_at(sp.unwrap_or(k.span), "a direction is turned `turn:` an angle \
                                                       `about:` a line or an axis: both or neither");
                    return None;
                }
                k.dir = Some(DirRef { what, about, turn });
            }
            None if about.is_some() || turn.is_some() => {
                self.fail_at(k.span, "`turn:` and `about:` turn a direction read off geometry, \
                                      `hint(dir: up, turn: 90deg, about: g)`");
                return None;
            }
            None => {}
        }
        Some(k)
    }

    /// Parse a style block, reporting unknown or invalid properties at their spans.
    pub(super) fn style_rule(&mut self) -> Option<StmtKind> {
        let lo = self.here().lo as usize;
        self.i += 1; // `style`
        if !self.want_p('.') {
            return None;
        }
        let name = self.ident()?;
        if !self.want_p('{') {
            return None;
        }
        let mut style = Style::default();
        let mut props: Vec<String> = Vec::new();
        while !self.eat_p('}') {
            if self.eat_p(';') || self.peek() == Some(&Tok::Nl) {
                self.i += usize::from(self.peek() == Some(&Tok::Nl));
                continue;
            }
            let Some(prop) = self.slot_label() else {
                self.fail("a style rule is `property: value`");
                return None;
            };
            let from = self.here().lo as usize;
            let mut values: Vec<f64> = Vec::new();
            while !matches!(
                self.peek(),
                Some(Tok::P(';')) | Some(Tok::P('}')) | Some(Tok::Nl) | None
            ) {
                if let Some(Tok::Num(v)) = self.peek() {
                    values.push(*v);
                }
                self.i += 1;
            }
            let text = self.text_from(from).trim().to_string();
            if !style.set(&prop, &values, &text) {
                // an unknown property is not an error, exactly as an unmatched class is not:
                // a sheet says what it knows how to say and the rest has no rule.  A value a
                // *known* property cannot read is another thing — `color: ;`, `width: nope` —
                // never anything but a mistake, and dropped silently a mistyped sheet looked
                // exactly like a working one (#43.20)
                if Style::knows(&prop) {
                    let m = if text.is_empty() {
                        format!("`{prop}:` is given no value")
                    } else {
                        format!("`{prop}` cannot read `{text}`")
                    };
                    self.fail_at(Span::new(from, self.prev_hi().max(from + 1)), &m);
                }
                continue;
            }
            props.push(prop);
        }
        Some(StmtKind::Style(StyleRule { name, style, props, span: Span::new(lo, self.prev_hi()) }))
    }

    /// Read classes until a trailing clause or joint begins. Preserve the clause span
    /// or its insertion point for source edits.
    pub(super) fn class_clause(&mut self, at: usize) -> (Classes, Span) {
        if !self.peek_word("class") {
            return (Classes::default(), Span::at(at));
        }
        let lo = self.here().lo as usize;
        self.i += 1;
        let mut c = Classes::default();
        while let Some(Tok::Ident(w)) = self.peek().cloned() {
            // `at` is a relation's placement clause, and no class is called that
            if trails_decl(&w) || w == "at" {
                break;
            }
            c.0.push(w);
            self.i += 1;
        }
        (c, Span::new(lo, self.prev_hi()))
    }
}
