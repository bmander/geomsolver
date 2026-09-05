//! Entity declarations, hints, styles, and plane and sweep arguments.

use super::P;
use crate::model::{EntKind, Field};
use crate::style::{Classes, Style};
use crate::syntax::lexer::Tok;
use crate::syntax::words::{names_decl, trails_decl};
use crate::syntax::{
    decl_head, Arg, AtRef, Attitude, Decl, DeclName, Kid, KidSeed, Membership, Name, Ref, Sense,
    Span, StmtKind, StyleRule, Sweep,
};

/// A solid's sweep arguments while the bracket list is being read.
#[derive(Default)]
struct SweepParts {
    from: Option<Arg>,
    to: Option<Arg>,
    depth: Option<Arg>,
    about: Option<Ref>,
    sweep: Option<Arg>,
    sense: Option<Sense>,
}

/// The labels a solid's brackets may carry beside the face or the operands.
fn sweep_label(l: &str) -> bool {
    matches!(l, "from" | "to" | "depth" | "about" | "sweep" | "sense")
}

/// What the sweep arguments a bracket list carried come to.  A solid is a prism, a revolution,
/// or a body over other solids — and a mixture is none of the three.
fn sweep_of(p: SweepParts) -> Result<Sweep, String> {
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
                (Some(from), Some(to), None) => Ok(Sweep::Prism { from, to }),
                (None, None, Some(depth)) => Ok(Sweep::Depth { depth }),
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

/// A plane's attitude arguments while the bracket list is being read: each may arrive once,
/// in any order, and `attitude_of` says which combinations make a plane.
#[derive(Default)]
struct AttParts {
    from: Option<Ref>,
    fold: Option<Arg>,
    offset: Option<Arg>,
    u: Option<[Arg; 3]>,
    v: Option<[Arg; 3]>,
}

/// The labels a plane's brackets may carry beside its children.
fn attitude_label(l: &str) -> bool {
    matches!(l, "from" | "fold" | "u" | "v" | "offset")
}

/// What the attitude arguments a bracket list carried come to: the page when it carried none,
/// a fold when it named a plane, a basis when it gave both vectors — and a complaint for the
/// halves and the mixtures.
fn attitude_of(p: AttParts) -> Result<Attitude, String> {
    match (p.from, p.fold, p.offset, p.u, p.v) {
        (None, None, None, None, None) => Ok(Attitude::Page),
        // **`from:` says which plane it is derived from; `fold:` and `offset:` say how.**  0.10
        // read a bare `from:` as `fold: 0deg`, a default no document in the corpus ever used;
        // a plane naming another and folding nothing most plainly says *the same plane, moved*,
        // which is what a stack is written in (§6.10) and what one `against` states.
        (Some(plane), Some(fold), None, None, None) => Ok(Attitude::From { plane, fold }),
        (Some(plane), None, offset, None, None) => Ok(Attitude::Offset { plane, offset }),
        (None, None, None, Some(u), Some(v)) => Ok(Attitude::Basis { u, v }),
        (None, Some(_), _, None, None) => Err("`fold` folds from a plane: say `from:` too".into()),
        (None, _, Some(_), None, None) => {
            Err("`offset` stands a plane off another: say `from:` too".into())
        }
        (Some(_), Some(_), Some(_), _, _) => {
            Err("a plane is folded from another (`fold:`) or stood off it (`offset:`), not both"
                .into())
        }
        (None, None, None, Some(_), None) | (None, None, None, None, Some(_)) => {
            Err("a basis is both `u:` and `v:`".into())
        }
        _ => Err("a plane is folded from another (`from:`, `fold:`), stood off it (`offset:`) \
                  or given a basis (`u:`, `v:`), not two of the three"
            .into()),
    }
}
impl<'a> P<'a> {
    pub(super) fn decl(&mut self, kind: EntKind) -> Option<Decl> {
        // **The name is optional**, independently of everything after it (issue #33): `line`
        // alone, `line(p1, p2)` and `circle hint(r: 25)` are all anonymous forms.  The token
        // after the kind keyword decides — an identifier that may be a name is one, and a word
        // reserved for what can follow a declaration (`names_decl`) is read as itself.  An
        // anonymous declaration still needs a key the desugared statements can resolve by — a
        // chain's corner welds by *name* — so it is given one the tokenizer can never produce,
        // `#a` and its own offset (the flattener's block-prefix device, marked apart); its
        // span is empty at the point a real name would go, which is where `edit::reconcile`
        // splices one the moment a statement must say it.  `curve` keeps requiring a name: its
        // form is `curve name = family(…)`, and the name is what the contacts address.
        // Curve first, and on its own: its name is required, so it reaches `ident()` — and that
        // error — whatever stands next, identifier or not.
        let named =
            kind == EntKind::Curve || matches!(self.peek(), Some(Tok::Ident(w)) if names_decl(w));
        let name = if named {
            DeclName::Written(self.ident()?)
        } else {
            // an Ident declined here was very possibly *meant* as a name — remembered, so a
            // line that then fails to parse can say so (`chain_or_one`)
            if let (Some(Tok::Ident(w)), None) = (self.peek(), &self.declined) {
                self.declined = Some((w.clone(), self.here()));
            }
            let at = self.prev_hi();
            DeclName::Key(Name { text: format!("#a{at}"), span: Span::new(at, at) })
        };
        // how an error spells this statement's head — computed at the failure, since every
        // declaration that parses would otherwise allocate a string nothing reads
        let head = || decl_head(kind, &name);
        // `curve path = leg.toe over theta in (0, 360)` — a point of a component, as one of its
        // numeric formals runs (§6.5).  The target is an instance's point, or an instance
        // written in place followed by the point's path.
        if kind == EntKind::Curve {
            if self.peek() != Some(&Tok::Eq) {
                self.fail(
                    "a curve is `curve name = instance.point over formal in (a, b)`, or \
                     `curve name = Component(args).point over formal in (a, b)`",
                );
                return None;
            }
            self.i += 1;
            let curve = self.curve_spec()?;
            let at = self.prev_hi();
            let (class, class_span) = self.class_clause(at);
            if self.peek_word("in") {
                self.fail("`in` puts points on a plane, and a curve has none of its own");
                return None;
            }
            return Some(Decl {
                kind,
                name,
                children: Vec::new(),
                seed: Vec::new(),
                seed_text: Vec::new(),
                seed_spans: Vec::new(),
                hint_span: None,
                knots: None,
                curve: Some(curve),
                class,
                class_span,
                computed: None,
                seed_at: None,
                seed_names: Vec::new(),
                attitude: Attitude::Page,
                sweep: None,
                membership: Membership::default(),
                list_span: Span::default(),
                close: None,
            });
        }
        // `point p = (xexpr, yexpr)` — a computed point (§6.5).  The brackets after a name say
        // what the thing is made of, and this one is made of a formula: no children, no seed
        // and no trailer, since nothing on the sheet ever holds it and no solve writes it.
        if self.peek() == Some(&Tok::Eq) {
            if kind != EntKind::Point {
                self.fail(&format!(
                    "only a point is computed (`point p = (x, y)`); a {} is made of its \
                     children",
                    kind.as_str()
                ));
                return None;
            }
            if name.written().is_none() {
                self.fail("a computed point is named: `point p = (x, y)`");
                return None;
            }
            self.i += 1;
            let computed = Some(self.pair()?);
            let end = self.prev_hi();
            let mut membership = Membership::default();
            membership.set_span(Span::new(end, end));
            return Some(Decl {
                kind,
                name,
                children: Vec::new(),
                seed: vec![0.0; 2],
                seed_text: vec![None; 2],
                seed_spans: vec![Span::default(); 2],
                hint_span: None,
                knots: None,
                curve: None,
                computed,
                class: Classes::default(),
                class_span: Span::new(end, end),
                seed_at: None,
                seed_names: Vec::new(),
                attitude: Attitude::Page,
                sweep: None,
                membership,
                list_span: Span::new(end, end),
                close: None,
            });
        }
        let mut children: Vec<Vec<Kid>> = Vec::new();
        let mut seed: Vec<f64> = Vec::new();
        let fields = kind.fields();
        // one slot per Child/List field, so the printer's shape and the parser's agree
        for (_, f) in fields {
            if *f != Field::Scalar {
                children.push(Vec::new());
            }
        }
        let scalars: Vec<&str> =
            fields.iter().filter(|(_, f)| *f == Field::Scalar).map(|(n, _)| *n).collect();
        seed.resize(scalars.len(), 0.0);
        let mut seed_text: Vec<Option<String>> = vec![None; scalars.len()];
        let mut seed_spans: Vec<Span> = vec![Span::default(); scalars.len()];
        let mut att = AttParts::default();
        let mut swp = SweepParts::default();
        let mut close: Option<Span> = None;
        let name_end = self.prev_hi();
        let open = self.here().lo as usize;
        let mut list_span = Span::new(name_end, name_end);
        if self.eat_p('(') {
            let mut positional = 0usize;
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
                            "`-> close` seals a face's loop, and a {} is not a loop",
                            kind.as_str()
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
                match label {
                    // **a solid's sweep is what it is made of**, so it stands in the brackets
                    // with the face — and it is read before the attitude's labels, since `from`
                    // is a word both constructs use and only one of them is a plane
                    Some(l) if kind == EntKind::Solid && sweep_label(&l) => {
                        self.sweep_arg(&l, &mut swp)?;
                    }
                    // a plane's attitude is what it is made of, so it stands in the brackets
                    // with the children — and no other kind has one to give
                    Some(l) if attitude_label(&l) => {
                        if kind != EntKind::Plane {
                            self.fail(&format!(
                                "`{l}` folds a plane, and a {} has no attitude to give",
                                kind.as_str()
                            ));
                            return None;
                        }
                        self.attitude_arg(&l, &mut att)?;
                    }
                    // the brackets after the name are *what the thing is made of*; where the
                    // solve begins is the `hint(…)` after them (spec §6.4)
                    Some(l) if scalars.contains(&l.as_str()) => {
                        let head = head();
                        self.fail(&format!(
                            "`{l}` is a seed, and a seed goes in a `hint(…)` clause: \
                             `{head}(…) hint({l}: …)`"
                        ));
                        return None;
                    }
                    _ => {
                        // a slot carries a name or a seed, and nothing else says "anonymous":
                        // an entity whose children are all unseeded writes no list at all
                        let kid = match self.eat_hint_clause() {
                            Some(lo) => Kid::Hint(self.kid_seed(lo)?),
                            None => Kid::Ref(self.refr()?),
                        };
                        let slot = match &label {
                            Some(l) => fields
                                .iter()
                                .filter(|(_, f)| *f != Field::Scalar)
                                .position(|(n, _)| n == l)
                                .unwrap_or_else(|| positional.min(children.len() - 1)),
                            None => {
                                // a List field takes every positional argument from where it starts
                                let n_named =
                                    fields.iter().filter(|(_, f)| *f == Field::Child).count();
                                if positional >= n_named && children.len() > n_named {
                                    n_named
                                } else {
                                    positional.min(children.len().saturating_sub(1))
                                }
                            }
                        };
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
        let attitude = match attitude_of(att) {
            Ok(a) => a,
            Err(m) => {
                let head = head();
                self.fail(&format!("`{head}`: {m}"));
                return None;
            }
        };
        let sweep = if kind == EntKind::Solid {
            match sweep_of(swp) {
                Ok(sw) => Some(sw),
                Err(m) => {
                    let head = head();
                    self.fail(&format!("`{head}`: {m}"));
                    return None;
                }
            }
        } else {
            None
        };
        // trailing clauses, in any order: `hint(…)`, `knots [...]`, `class …`, `in PLANE`.
        // Where a clause *would* go if it is not written is the point we are standing on now,
        // before any of them: that is what writeback appends at.
        let mut knots = None;
        let mut class = Classes::default();
        let mut class_span = Span::default();
        let mut seed_at: Option<AtRef> = None;
        let mut membership = Membership::default();
        let insert = self.prev_hi();
        let mut hint_span = Span::new(insert, insert);
        loop {
            if let Some(lo) = self.eat_hint_clause() {
                // `hint(x: 0, y: 12)` — keyed, keys in any order, an omitted scalar is 0 — or a
                // place named geometrically, `hint(at: t)`, `hint(at: c, bearing: u + phase)`:
                // the same clause, since a seed is what is inside one and nothing else is
                // (§4.3), and a place is a seed given as geometry rather than as numbers.
                // `at:` and `bearing:` are keys beside the scalars; a clause naming a place
                // carries no coordinate, and a bearing without a place says where on nothing.
                let mut bearing: Option<(String, Span)> = None;
                let mut bearing_at = Span::default();
                let mut coord: Option<Span> = None;
                for h in self.hint_body("x: 0, y: 0")? {
                    if let Some(what) = h.place {
                        if seed_at.is_some() {
                            self.fail_at(h.at, "`at:` is written twice");
                            continue;
                        }
                        seed_at = Some(AtRef { what, bearing: None });
                        continue;
                    }
                    if h.key == "bearing" {
                        bearing = Some((h.text, h.span));
                        bearing_at = h.at;
                        continue;
                    }
                    let Some(i) = scalars.iter().position(|&s| s == h.key) else {
                        // the key is the mistake, not the declaration: reported, and the rest
                        // of the clause read on, so the entity is still declared and no
                        // statement naming it fails for want of it (#43.19)
                        let m = format!("`{}` has no scalar `{}` to seed", kind.as_str(), h.key);
                        self.fail_at(h.at, &m);
                        continue;
                    };
                    coord.get_or_insert(h.at);
                    seed[i] = h.value.unwrap_or(0.0);
                    seed_text[i] = (h.value.is_none()).then_some(h.text);
                    seed_spans[i] = h.span;
                }
                match (&mut seed_at, bearing) {
                    (Some(at), b) => {
                        at.bearing = b;
                        if let Some(sp) = coord {
                            let m = "`at:` names the place; a clause with it carries no scalar";
                            self.fail_at(sp, m);
                        }
                    }
                    (None, Some(_)) => {
                        let m = "`bearing:` says where on a circle's edge, and needs `at:`";
                        self.fail_at(bearing_at, m);
                    }
                    (None, None) => {}
                }
                hint_span = Span::new(lo, self.prev_hi());
            } else if self.eat_word("knots") {
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
                knots = Some(u);
            } else if self.peek_word("class") {
                let (c, sp) = self.class_clause(insert);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
                class_span = sp;
            } else if self.peek_word("in") {
                // `in top` — every point this declaration mints or names is an image on that
                // plane (§6.7).  A datum has no points of its own to put there, and a curve is
                // its expressions.
                if !kind.bears_points() {
                    self.fail(&format!(
                        "`in` puts points on a plane, and a {} has none of its own",
                        kind.as_str()
                    ));
                    return None;
                }
                if membership.plane().is_some() {
                    let head = head();
                    self.fail(&format!("`{head}` is {}", membership.cause()));
                    return None;
                }
                let lo = self.here().lo as usize;
                self.i += 1;
                let r = self.refr()?;
                membership = Membership::written_at(r, Span::new(lo, self.prev_hi()));
            } else if self.peek_word("hint") || self.peek_word("at") {
                // the retired spellings — `hint at (0, 0)` and a bare `at (0, 0)` for a pair of
                // coordinates, `hint at REF [bearing (…)]` for a place (#47.2) — are what every
                // document said until the clause took them in, so the reader most likely to meet
                // an error here is the one holding one of them
                let head = head();
                let place = self.peek_word("hint")
                    && self.word_at(self.i + 1) == Some("at")
                    && self.t.get(self.i + 2).map(|(t, _)| t) != Some(&Tok::P('('));
                let m = if place {
                    format!("a place is keyed now: `{head} hint(at: REF, bearing: …)`")
                } else {
                    format!("a coordinate seed is keyed now: `{head} hint(x: …, y: …)`")
                };
                self.fail(&m);
                return None;
            } else {
                break;
            }
        }
        // where an `in` clause would go: after every trailer, so an appended one never races
        // `class_span` for one offset
        if membership.span().is_empty() {
            let end = self.prev_hi();
            membership.set_span(Span::new(end, end));
        }
        Some(Decl {
            kind,
            name,
            children,
            seed,
            seed_text,
            seed_spans,
            hint_span: Some(hint_span),
            knots,
            curve: None,
            computed: None,
            class,
            class_span: if class_span.is_empty() { Span::new(insert, insert) } else { class_span },
            seed_at,
            seed_names: Vec::new(),
            attitude,
            sweep,
            membership,
            list_span,
            close,
        })
    }

    /// One of a solid's sweep arguments, the label already eaten: `from: EXPR`, `to: EXPR`,
    /// `depth: EXPR`, `about: REF`, `sweep: EXPR`, `sense: cw|ccw`.
    fn sweep_arg(&mut self, label: &str, parts: &mut SweepParts) -> Option<()> {
        let twice = |s: &mut Self| {
            s.fail(&format!("`{label}` is given twice"));
            None
        };
        match label {
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
                let w = self.ident()?;
                parts.sense = Some(match w.text.as_str() {
                    "ccw" => Sense::Ccw,
                    "cw" => Sense::Cw,
                    other => {
                        self.fail(&format!("`sense` is `cw` or `ccw`, not `{other}`"));
                        return None;
                    }
                });
            }
            _ => {
                let slot = match label {
                    "from" => &mut parts.from,
                    "to" => &mut parts.to,
                    "depth" => &mut parts.depth,
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

    /// One of a plane's attitude arguments, the label already eaten: `from: REF`,
    /// `fold: EXPR`, `u: (E, E, E)`, `v: (E, E, E)`.
    fn attitude_arg(&mut self, label: &str, parts: &mut AttParts) -> Option<()> {
        let twice = |s: &mut Self| {
            s.fail(&format!("`{label}` is given twice"));
            None
        };
        match label {
            "from" => {
                if parts.from.is_some() {
                    return twice(self);
                }
                parts.from = Some(self.refr()?);
            }
            "fold" => {
                if parts.fold.is_some() {
                    return twice(self);
                }
                let (text, span) = self.expr_until(',')?;
                parts.fold = Some(Arg::Dim { text, span });
            }
            "offset" => {
                if parts.offset.is_some() {
                    return twice(self);
                }
                let (text, span) = self.expr_until(',')?;
                parts.offset = Some(Arg::Dim { text, span });
            }
            _ => {
                let slot = if label == "u" { &mut parts.u } else { &mut parts.v };
                if slot.is_some() {
                    return twice(self);
                }
                *slot = Some(self.triple()?);
            }
        }
        Some(())
    }

    /// `(E, E, E)` — three expressions, as written.
    fn triple(&mut self) -> Option<[Arg; 3]> {
        if !self.want_p('(') {
            return None;
        }
        let mut out: Vec<Arg> = Vec::with_capacity(3);
        for k in 0..3 {
            let (text, span) = self.expr_until(if k < 2 { ',' } else { ')' })?;
            out.push(Arg::Dim { text, span });
            if k < 2 && !self.want_p(',') {
                return None;
            }
        }
        if !self.want_p(')') {
            return None;
        }
        Some([out.remove(0), out.remove(0), out.remove(0)])
    }
}
impl<'a> P<'a> {
    /// A `hint(…)` standing in a child slot, the opening paren already eaten.
    ///
    /// The same clause as everywhere else, so it is read by the same `hint_body`; what the keys
    /// mean is this table — an anonymous child is a point, and a point has x and y.
    fn kid_seed(&mut self, lo: usize) -> Option<KidSeed> {
        let mut k = KidSeed::default();
        for h in self.hint_body("x: 0, y: 0")? {
            let i = match h.key.as_str() {
                "x" => 0,
                "y" => 1,
                _ => {
                    let m = format!("an anonymous point has no scalar `{}`; it has x and y", h.key);
                    self.fail_at(h.at, &m);
                    return None;
                }
            };
            k.v[i] = h.value.unwrap_or(0.0);
            k.text[i] = (h.value.is_none()).then_some(h.text);
            k.spans[i] = h.span;
        }
        k.span = Span::new(lo, self.prev_hi());
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
            return (Classes::default(), Span::new(at, at));
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
