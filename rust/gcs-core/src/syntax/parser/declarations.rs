//! Entity declarations, hints, styles, and plane and sweep arguments.

use super::P;
use crate::model::{EntKind, Field};
use crate::style::{Classes, Style};
use crate::syntax::lexer::Tok;
use crate::syntax::words::trails_decl;
use crate::syntax::{
    decl_head, Arg, AtRef, Decl, DeclName, Kid, KidSeed, Membership, Name, Ref, Sense, Span, StmtKind, StyleRule, Sweep,
    Weight,
};

/// A solid's sweep arguments while the bracket list is being read.
#[derive(Default)]
struct SweepParts {
    under: Option<Ref>,
    at: Option<Arg>,
    from: Option<Arg>,
    to: Option<Arg>,
    depth: Option<Arg>,
    about: Option<Ref>,
    through: Option<Ref>,
    along: Option<Ref>,
    sweep: Option<Arg>,
    sense: Option<Sense>,
}

/// The labels a solid's brackets may carry beside the face or the operands.
fn sweep_label(l: &str) -> bool {
    matches!(l, "from" | "to" | "depth" | "about" | "sweep" | "sense" | "through" | "along" | "under" | "at")
}

/// What the sweep arguments a bracket list carried come to.  A solid is a prism, a revolution,
/// or a body over other solids — and a mixture is none of the three.
fn sweep_of(p: SweepParts) -> Result<Sweep, String> {
    if p.under.is_some() || p.at.is_some() {
        if p.depth.is_some() || p.about.is_some() || p.through.is_some()
            || p.along.is_some() || p.sweep.is_some() || p.sense.is_some() {
            return Err("a named motion uses `at:` or angular `from:`/`to:`, without other sweep labels".into());
        }
        return match (p.under,p.at,p.from,p.to) {
            (Some(motion),Some(at),None,None) => Ok(Sweep::Placed {motion,at}),
            (Some(motion),None,Some(from),Some(to)) => Ok(Sweep::Swept {motion,from,to}),
            _ => Err("a named motion needs `under:` and either `at:` or both `from:` and `to:`".into()),
        };
    }
    if let Some(guide) = p.along {
        if p.from.is_some() || p.to.is_some() || p.depth.is_some() || p.about.is_some()
            || p.through.is_some() || p.sweep.is_some() || p.sense.is_some() {
            return Err("`along:` cannot be combined with other sweep extents".into());
        }
        return Ok(Sweep::Along { guide });
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
                span: Span::new(kw.lo as usize, kw.lo as usize),
            }),
        };
        // how an error spells this statement's head — computed at the failure, since every
        // declaration that parses would otherwise allocate a string nothing reads
        let head = || decl_head(kind, &name);
        if kind == EntKind::Motion { return self.motion_decl(name); }
        if kind == EntKind::Envelope { return self.envelope_decl(name); }
        // a curve is what `over` makes a definition's value, `k := leg.toe over u in (a, b)` — or,
        // with the two points it runs between, a free curve whose shape its energy states
        // (`rope := curve(a, b)`, #144), read as any declaration's brackets are
        if kind == EntKind::Curve && self.peek() != Some(&Tok::P('(')) {
            self.fail(
                "a curve is `name := instance.point over formal in (a, b)`, \
                 `name := Component(args).point over formal in (a, b)`, or a free curve between \
                 two points, `name := curve(a, b)`",
            );
            return None;
        }
        // `p := point(x: xexpr, y: yexpr)` — a computed point (§6.5).  The brackets say what the
        // thing is made of, and this one is made of a formula: no children, no seed and no
        // trailer, since nothing on the sheet ever holds it and no solve writes it.  A point has
        // no children, so its brackets are free to hold the two coordinates.
        if kind == EntKind::Point && self.peek() == Some(&Tok::P('(')) {
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
            membership.set_span(Span::new(end, end));
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
                class_span: Span::new(end, end),
                seed_at: None,
                seed_names: Vec::new(),
                sweep: None, motion: None, angular_span: None,
                membership,
                list_span: Span::new(end, end),
                close: None,
                mint_close: None,
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
        let mut swp = SweepParts::default();
        let mut surface_from = None;
        let mut surface_to = None;
        let mut close: Option<Span> = None;
        let name_end = self.prev_hi();
        let open = self.here().lo as usize;
        let mut list_span = Span::new(name_end, name_end);
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
                if kind == EntKind::Edge && label.as_deref()
                    .is_some_and(|s| !matches!(s,"seam" | "from" | "to" | "along")) {
                    self.fail("an edge names its `seam:`, `from:`, `to:` and `along:` operands");
                    return None;
                }
                if matches!(kind,EntKind::Seam | EntKind::Vertex)
                    && label.as_deref().is_some_and(|s| s != "first" && s != "second") {
                    self.fail("a seam or vertex names its `first:` and `second:` operands");
                    return None;
                }
                if kind == EntKind::Patch && !matches!(label.as_deref(),
                    Some("source" | "inside" | "outside")) && !(label.is_none() && positional == 0) {
                    self.fail("a patch names its source, then labels each solid `inside:` or `outside:`");
                    return None;
                }
                if kind == EntKind::Face {
                    match label.as_deref() {
                        Some("holes") => in_holes = true,
                        Some("edges") if !in_holes => {},
                        Some("on") => {},
                        Some(_) => {
                            self.fail("a face takes its edges, optional `holes:`, and `on:` for a spatial support");
                            return None;
                        }
                        None => {}
                    }
                }
                match label {
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
                        let head = head();
                        self.fail(&format!(
                            "`{l}` is a seed, and a seed goes in a `hint(…)` clause: \
                             `{head}(…) hint({l}: …)`"
                        ));
                        return None;
                    }
                    _ => {
                        // A slot names an entity, seeds a point, or holds a solid's inline
                        // section. An entity whose children are all unseeded writes no list.
                        let kid = match self.eat_hint_clause() {
                            Some(lo) => {
                                // a plane's `u:` and `v:` are axes, seeded by their direction
                                let axis = kind == EntKind::Plane && match &label {
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
                        };
                        let slot = if kind == EntKind::Face {
                            if label.as_deref() == Some("on") { 2 } else { usize::from(in_holes) }
                        } else {
                            match &label {
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
        let angular_span = match (surface_from,surface_to) {
            (None,None) => None,
            (Some(from),Some(to)) => Some(crate::syntax::AngularSpan {from,to,side:None}),
            _ => { self.fail("a surface span needs both `from:` and `to:` angles"); return None; }
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
        let mut hint_span = Span::new(insert, insert);
        loop {
            if let Some(lo) = self.eat_hint_clause() {
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
                let mut xy: [Option<(String, Span)>; 2] = [None, None];
                let mut other_coord: Option<Span> = None;
                for h in self.hint_body("x: 0, y: 0")? {
                    if let Some(what) = h.place {
                        let slot = match h.key.as_str() {
                            "at" => &mut at,
                            "toward" => &mut toward,
                            _ => &mut along,
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
                        _ => {
                            other_coord.get_or_insert(h.at);
                        }
                    }
                    seed[i] = h.value.unwrap_or(0.0);
                    seed_text[i] = (h.value.is_none()).then_some(h.text);
                    seed_spans[i] = h.span;
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
                        seed_at = Some(AtRef {
                            what,
                            bearing: text(bearing),
                            toward,
                            along,
                            by: text(by),
                            turn: text(turn),
                            x: if in_plane { xy[0].take() } else { None },
                            y: if in_plane { xy[1].take() } else { None },
                        });
                        if in_plane {
                            // the place carries the numbers; the scalars stay unseeded
                            for i in 0..seed.len() {
                                seed[i] = 0.0;
                                seed_text[i] = None;
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
            } else if self.eat_word("weights") {
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
                weights = Some(w);
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
                        "`in` puts points on a plane, and {} has none of its own",
                        kind.a()
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
                membership = Membership::written_on(r, also, Span::new(lo, self.prev_hi()));
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
                    format!("a coordinate seed is keyed now: `{head} hint((…, …))`")
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
            annotations: Default::default(),
            kind,
            name,
            children,
            seed,
            seed_text,
            seed_spans,
            hint_span: Some(hint_span),
            knots,
            weights,
            curve: None,
            computed: None,
            class,
            class_span: if class_span.is_empty() { Span::new(insert, insert) } else { class_span },
            seed_at,
            seed_names: Vec::new(),
            sweep,
            motion: None, angular_span,
            membership,
            list_span,
            close,
            mint_close: None,
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
            "under" => {
                if parts.under.is_some() { return twice(self); }
                parts.under = Some(self.refr()?);
            }
            "along" => {
                if parts.along.is_some() { return twice(self); }
                parts.along = Some(self.refr()?);
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
                    "at" => &mut parts.at,
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

}
impl<'a> P<'a> {
    fn envelope_decl(&mut self, name: DeclName) -> Option<Decl> {
        let start = self.here();
        if !self.want_p('(') { return None; }
        let mut surface = None; let mut motion = None;
        let mut from = None; let mut to = None; let mut side = None;
        while !self.eat_p(')') {
            let label = self.slot_label();
            match label.as_deref() {
                Some("surface") | Some("under") | Some("motion") | None => {
                    let slot = match label.as_deref() {
                        Some("surface") => &mut surface,
                        Some("under") | Some("motion") => &mut motion,
                        _ if surface.is_none() => &mut surface,
                        _ => &mut motion,
                    };
                    if slot.is_some() { self.fail("an envelope argument is given twice"); return None; }
                    *slot = Some(Kid::Ref(self.refr()?));
                }
                Some("from") | Some("to") => {
                    let slot = if label.as_deref() == Some("from") { &mut from } else { &mut to };
                    if slot.is_some() { self.fail("an envelope bound is given twice"); return None; }
                    let (text,span) = self.expr_until(',')?;
                    *slot = Some(Arg::Dim {text,span});
                }
                Some("side") => {
                    if side.is_some() { self.fail("an envelope's side is given twice"); return None; }
                    side = Some(self.refr()?.root);
                }
                _ => { self.fail("an envelope takes a surface or tool, `under:` a motion, `from:` and `to:` angles, and `side:`"); return None; }
            }
            if self.peek() != Some(&Tok::P(')')) && !self.want_p(',') { return None; }
        }
        let (Some(surface),Some(motion),Some(from),Some(to)) = (surface,motion,from,to) else {
            self.fail("an envelope needs a surface, motion and two angular bounds"); return None;
        };
        let end = self.prev_hi();
        let (class,class_span) = self.class_clause(end);
        Some(Decl {
            annotations:Default::default(),kind:EntKind::Envelope,name,
            children:vec![vec![surface],vec![motion]],
            seed:vec![],seed_text:vec![],seed_spans:vec![],hint_span:None,knots:None,weights:None,curve:None,
            computed:None,class,class_span,seed_at:None,seed_names:vec![],sweep:None,motion:None,angular_span:Some(crate::syntax::AngularSpan {from,to,side}),
            membership:Membership::default(),list_span:Span::new(start.lo as usize,end),close:None,mint_close:None,
        })
    }

    fn motion_decl(&mut self, name: DeclName) -> Option<Decl> {
        use crate::syntax::MotionSpec;
        let start = self.here();
        if !self.want_p('(') { return None; }
        let mut axis = None; let mut along = None; let mut source = None; let mut observer = None;
        let mut ratio = None; let mut phase = None; let mut advance = None;
        while !self.eat_p(')') {
            let label = self.slot_label();
            match label.as_deref() {
                Some("about") | Some("along") | Some("relative_to") | Some("of") | None => {
                    let slot = match label.as_deref() {
                        Some("about") => &mut axis,
                        Some("along") => &mut along,
                        Some("relative_to") => &mut observer,
                        _ => &mut source,
                    };
                    if slot.is_some() { self.fail("a motion argument is given twice"); return None; }
                    *slot = Some(self.refr()?);
                }
                Some("ratio") | Some("phase") | Some("advance") => {
                    let slot = match label.as_deref() {
                        Some("ratio") => &mut ratio,
                        Some("phase") => &mut phase,
                        _ => &mut advance,
                    };
                    if slot.is_some() { self.fail("a motion argument is given twice"); return None; }
                    let (text,span) = self.expr_until(',')?;
                    *slot = Some(Arg::Dim {text,span});
                }
                _ => { self.fail("a motion takes `about`, `ratio`, `phase`, `advance`; `along`, `advance`; or `of`, `relative_to`"); return None; }
            }
            if self.peek() != Some(&Tok::P(')')) && !self.want_p(',') { return None; }
        }
        let spec = match (axis,along,source,observer,ratio,phase,advance) {
            (Some(axis),None,None,None,ratio,phase,advance) => MotionSpec::Rotation {axis,ratio,phase,advance},
            (None,Some(axis),None,None,None,None,Some(advance)) => MotionSpec::Translation {axis,advance},
            (None,None,Some(source),Some(observer),None,None,None) => MotionSpec::Relative {source,observer},
            _ => {
                self.fail("a motion is a rotation `about:` a line (with `advance:` for a screw), a translation \
                    `along:` a line by `advance:` per turn, or `of:` another motion `relative_to:` an observer");
                return None;
            }
        };
        let end = self.prev_hi();
        let (class,class_span) = self.class_clause(end);
        Some(Decl {
            annotations:Default::default(),kind:EntKind::Motion,name,children:vec![vec![]],
            seed:vec![],seed_text:vec![],seed_spans:vec![],hint_span:None,knots:None,weights:None,curve:None,
            computed:None,class,class_span,seed_at:None,seed_names:vec![],sweep:None,motion:Some(spec),angular_span:None,membership:Membership::default(),
            list_span:Span::new(start.lo as usize,end),close:None,mint_close:None,
        })
    }

    /// A `hint(…)` standing in a child slot, the opening paren already eaten.
    ///
    /// The same clause as everywhere else, so it is read by the same `hint_body`; what the keys
    /// mean is this table — an anonymous child is a point, `hint((3, 4))`, or a plane's axis,
    /// `hint(dir: (1, 0, 0))`.
    fn kid_seed(&mut self, lo: usize, axis: bool) -> Option<KidSeed> {
        let mut k = KidSeed { axis, ..KidSeed::default() };
        for h in self.hint_body("(0, 0)")? {
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
