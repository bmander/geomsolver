//! Statements, component instances, and braced bodies.

use super::P;
use crate::model::EntKind;
use crate::style::Classes;
use crate::syntax::lexer::Tok;
use crate::syntax::words::{self, over_chain, over_walks, set_at, Brackets, BLOCKS, ENERGY_WORDS};
use crate::syntax::{
    Arg, Block, BlockKind, BodyWord, Branch, Chained, ClaimOver, Component, CurveSpec, CurveTarget,
    Decl, DeclName, EdgesOf, DerivedDecl, Formal, InBlock, InstArg, InstVal, Instance, Membership,
    Input, Name, OpArg, OpenJoint, ParamDecl, Ref, SetLit, SolidRel, Source, Span, Stmt, StmtKind,
    SynErr, Ty, Use,
};

/// What a `when` is, said where one is written otherwise (#210).
const WHEN: &str = "a `when` is decided by comparing numbers, or names: `when d > 0mm { … }`, \
                    `when b != a { … }`";

/// Mark every relation of an `initially` body a row of the home alone (#339 D3), through a
/// `when` in it; anything else in it is refused, since only a relation can hold at one place.
fn stamp_initially(stmts: &mut [Stmt], errs: &mut Vec<SynErr>) {
    for st in stmts {
        match &mut st.kind {
            StmtKind::Relation(r) => r.initially = true,
            StmtKind::Block(b) if b.kind == BlockKind::When => stamp_initially(&mut b.body, errs),
            _ => errs.push(SynErr {
                span: st.span,
                message: "`initially` holds relations, which hold at the home alone; what a \
                          station draws, it draws at every station"
                    .into(),
            }),
        }
    }
}

/// Apply block membership recursively. Faces and solids inherit their geometry's plane;
/// instances carry membership into expansion.  A statement already in a frame — a block inside
/// this one, or its own clause — **rides** on this one too (#383): the path is kept, outermost
/// first, and the elaborator draws it in the innermost frame of it with axes.  A frame declared
/// here takes the path as where it rides.
fn stamp_frame(stmts: &mut [Stmt], plane: &Ref, errs: &mut Vec<SynErr>) {
    for st in stmts {
        match &mut st.kind {
            StmtKind::Decl(d) => {
                // Faces and solids use the planes of their constituent geometry, and a free curve
                // the view of the points it runs between.
                if d.kind.spatial() || (d.kind == EntKind::Curve && d.curve.is_none()) {
                } else if !d.kind.bears_points() && d.kind != EntKind::Frame {
                    errs.push(SynErr {
                        span: st.span,
                        message: format!(
                            "`in` puts points on a plane, and {} has none of its own",
                            d.kind.a()
                        ),
                    });
                } else if !d.membership.join(plane, Source::Block) {
                    d.membership.ride_in(plane);
                }
            }
            StmtKind::Block(b) => stamp_frame(&mut b.body, plane, errs),
            // a step measured in no frame of its own is measured in the view its line is drawn
            // in (#330): the block's plane, handed on as a reference — which no source writes,
            // a written `from:` being kept as its text — so the flattener tells the two apart
            StmtKind::Relation(r) => {
                let Some(w) = r.form.written_mut() else { continue };
                if !w.is_step()
                    || w.args.iter().any(|a| matches!(a, OpArg::Named(n, _) if n.text == "from"))
                {
                    continue;
                }
                let from = Name { text: "from".into(), span: Span::at(plane.span.lo as usize) };
                w.args.push(OpArg::Named(from, Arg::Ref(plane.clone())));
            }
            // the instance joins whole: the flattener carries the plane into its expansion
            StmtKind::Instance(inst) => {
                if !inst.membership.join(plane, Source::Block) {
                    inst.membership.ride_in(plane);
                }
            }
            _ => {}
        }
    }
}

impl<'a> P<'a> {
    fn stmt(&mut self, next_id: &mut u32) -> Option<StmtKind> {
        // `distance(p, f1) + distance(p, f2) <= 10mm`, `2 * distance(p, a) >= …` — a bound on
        // measurements (#183 F3), told from a relation by a comparison outside every bracket —
        // and from a block's header, `when length(l) > 0mm { … }` (#210)
        let block = matches!(self.peek(), Some(Tok::Ident(w)) if BLOCKS.contains(&w.as_str()));
        if !block && self.compared_ahead() && self.measures_ahead() {
            return self.measured().map(StmtKind::Measured);
        }
        let Some(Tok::Ident(w)) = self.peek().cloned() else {
            self.fail("a statement starts with a word");
            return None;
        };
        match w.as_str() {
            "unit" => {
                self.i += 1;
                Some(StmtKind::Unit(self.ident()?))
            }
            "style" => self.style_rule(),
            "use" => {
                self.fail("a `use` stands at the top of a document, not inside a body");
                None
            }
            "interface" => {
                self.fail("an `interface` stands at the top of a file, not inside a body");
                self.skip_block();
                None
            }
            "preview" => {
                self.fail("a `preview` stands at the top of a document, not inside a body");
                self.i += 1;
                let _ = self.braced_body(next_id);
                None
            }
            // **`ccw` and `cw` keep a call.**  Every other statement is a prefix or an infix
            // operator, and under that rule these would be `a ccw(c) b` — which reorders three
            // points that are symmetric, since the predicate is about the *triangle* and not
            // about a pair with a decoration.  Spec §9.6 keeps the call for exactly that reason.
            "branch" => self.branch(),
            "ellipse" | "port" => {
                self.retired(&w);
                None
            }
            // 0.15's `frame f(origin: o, toward: q)` (#47 folded it into `plane`); a frame is
            // declared bare again, by definition (#383)
            "frame" if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(_))) => {
                self.fail(
                    "a frame is declared bare, `f := frame`, and constrained after the fact: \
                     its axes `f.u`, `f.v` and its origin `f.origin` take relations, and a flat \
                     frame over two axes is `f := plane(u: r1, v: r2)`",
                );
                None
            }
            b if BLOCKS.contains(&b) => {
                self.i += 1;
                let kind = match w.as_str() {
                    "repeat" => BlockKind::Repeat,
                    "ring" => BlockKind::Ring,
                    "when" => BlockKind::When,
                    _ => BlockKind::Cycle,
                };
                self.block(kind, next_id).map(StmtKind::Block)
            }
            // **`claim over crank.theta in (0deg, 360deg) { … }`** (§9.8): every claim in the
            // body judged as the drawing runs along one of its own free variables, and the worst
            // pose reported.  Every claim the language had until now is judged at one pose, and
            // a fact about a *cycle* — a disc clearing a cylinder mouth, a port's timing — is
            // not one of those.
            "claim" if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(w)) if w == "over") =>
            {
                self.claim_over(next_id)
            }
            // `claim vertical(rail)` — a relation stated as expected to add no rank
            "claim" => {
                self.i += 1;
                let mut r = self.relation()?;
                r.claim = true;
                Some(StmtKind::Relation(r))
            }
            // `view(cyl.body) in views.right` — a picture asked of a solid (§6.11).  The
            // brackets are what it is a picture of, `in` is the trailer it already is
            "view" | "section" | "dimensions" => {
                let at = self.here().lo as usize;
                let name = DeclName::Key(Name { text: format!("#a{at}"), span: Span::at(at) });
                self.derived(name)
            }
            // `bore cut cyl`, `boss union cyl` — the body rule's own words, and the one statement
            // whose shape is two names with a word between them that is not a constraint.  Read
            // by a lookahead rather than by `relation()`, since `cut` relates no geometry and
            // has no residual to be settled into
            // **past the dotted path, not past one token**: an operand may be a dotted path, so
            // the word after it is further away than one token — the lookahead `past_ref`
            _ if self.past_ref(self.i).and_then(|j| self.t.get(j)).is_some_and(
                |(t, _)| matches!(t, Tok::Ident(w) if w == "cut" || w == "union" || w == "common" || w == "bound"
                    || w == "through"),
            ) =>
            {
                self.body_word()
            }
            // `rope minimizes integral(p.y over p)` — what a curve is stationary for (#121), read
            // by the word past its curve, as the body words are
            _ if self.past_ref(self.i).and_then(|j| self.word_at(j)).is_some_and(|w| ENERGY_WORDS.contains(&w)) => {
                self.minimize().map(StmtKind::Minimize)
            }
            // a bare component call, when no source name is needed — its own file's component,
            // or a used module's by its full path (`engine.parts.Crank(…)`, §14.4)
            _ if self.call_at(self.i).is_some_and(|dotted| {
                dotted || !crate::constraints::is_operator(&w)
            }) && !self.defined_prefix_ahead() && !self.rate_ahead() =>
            {
                let lo = self.here().lo as usize;
                let name = Name { text: format!("#i{lo}"), span: Span::at(lo) };
                self.instance(name, lo).map(StmtKind::Instance)
            }
            _ => self.relation().map(StmtKind::Relation),
        }
    }

    /// `branch(KEY, ±1)`, the word next: a raw root choice.
    fn branch(&mut self) -> Option<StmtKind> {
        self.i += 1;
        if !self.want_p('(') {
            return None;
        }
        // the key is written **bare** — `branch(ppp:3|4|5, 1)`.  The language has no
        // string literal (a quote is the inch mark, §3.3), and a branch key is one run
        // of characters with no comma in it, so it is taken as the source text up to
        // the comma exactly as a dimension's is taken up to the end of its line.
        let from = self.here().lo as usize;
        while !matches!(self.peek(), Some(Tok::P(',')) | Some(Tok::Nl) | None) {
            self.i += 1;
        }
        let key = self.text_from(from).trim().to_string();
        if key.is_empty() {
            self.fail("a raw branch names the construction it decides");
            return None;
        }
        if !self.want_p(',') {
            return None;
        }
        let v = self.number()? as i32;
        if !self.want_p(')') {
            return None;
        }
        Some(StmtKind::Branch(Branch { key, value: v }))
    }

    /// A statement word the language retired, refused saying what to write now.
    fn retired(&mut self, w: &str) {
        match w {
            "ellipse" => {
                // a library component now (bmander/geomsolver#47, item 4): a computed point on
                // a datum, traced as a curve, whose contacts are the curve's
                self.fail(
                    "`ellipse` is a library component now: `use std`, then `e := \
                     trace(Ellipse(c, a: …, b: …, tilt: 0deg).p over u in (0deg, 360deg))` \
                     about a centre `c` — `p coincident e`, `e tangent l` and `e curvature k` are \
                     the curve's contacts",
                );
            }
            "port" => {
                // retired (bmander/geomsolver#47): everything an instance makes is reached by
                // its dotted name, so a port was a second name for a thing that had one
                self.fail(
                    "`port` is retired: an instance's entities are reached by dotted name \
                     (`inst.p`), so declare the point (`p := point hint(…)`), compute it \
                     (`p := point(x: …, y: …)`) or name the aliased entity itself",
                );
            }
            _ => {}
        }
    }

    /// `claim over NAME in (from, to) { … }`, `claim` next: claims judged as the drawing runs
    /// along one of its free variables (§9.8).
    fn claim_over(&mut self, next_id: &mut u32) -> Option<StmtKind> {
        let lo = self.here().lo as usize;
        self.i += 2;
        let formal = self.refr()?;
        // over a motion seen from another frame (#383): `claim over m seen_from g in (a, b)`
        let seen_from = if self.eat_word("seen_from") { Some(self.refr()?) } else { None };
        if !self.peek_word("in") {
            self.fail("a swept claim runs `over NAME in (from, to)`");
            return None;
        }
        self.i += 1;
        if !self.want_p('(') {
            return None;
        }
        let (from, fs) = self.expr_until(',')?;
        if !self.want_p(',') {
            return None;
        }
        let (to, ts) = self.expr_until(')')?;
        if !self.want_p(')') {
            return None;
        }
        let (body, _) = self.braced_body(next_id)?;
        Some(StmtKind::ClaimOver(ClaimOver {
            formal,
            seen_from,
            from: Arg::Dim { text: from, span: fs },
            to: Arg::Dim { text: to, span: ts },
            body,
            span: Span::new(lo, self.prev_hi()),
        }))
    }

    /// `X union B`, `X cut B`, `X common B` — a body statement, its first operand next.
    fn body_word(&mut self) -> Option<StmtKind> {
        let lo = self.here().lo as usize;
        let what = self.refr()?;
        let Some(Tok::Ident(w)) = self.peek().cloned() else {
            self.fail("a body statement is `X union B`, `X cut B` or `X common B`");
            return None;
        };
        if w == "through" {
            self.fail("Boolean `through` is now `cut`: write `X cut B`; `through:` specifies a cutter's extent");
            return None;
        }
        // through 0.84 the intersection was `bound`, the word a dimension's bound is (#270)
        if w == "bound" {
            self.fail("the body word `bound` is now `common`: write `X common B`, the body keeping what \
                       it has in common with `X`");
            return None;
        }
        let word = BodyWord::parse(&w).unwrap_or(BodyWord::Common);
        self.i += 1;
        let body = self.refr()?;
        self.end_of_stmt();
        Some(StmtKind::SolidRel(SolidRel {
            word,
            what,
            body,
            span: Span::new(lo, self.prev_hi()),
        }))
    }

    /// Whether the statement from here compares two sides outside every bracket (`<` or `>`).
    fn compared_ahead(&self) -> bool {
        matches!(self.t.get(self.level_end(true)), Some((Tok::P('<' | '>'), _)))
    }

    /// Whether a measurement is called before the comparison (`distance(`, `angle(`).
    fn measures_ahead(&self) -> bool {
        let end = self.level_end(true);
        (self.i..end).any(|j| {
            matches!(&self.t[j].0, Tok::Ident(w) if crate::expr::MEASURES.iter().any(|m| m.0 == w))
                && matches!(self.t.get(j + 1), Some((Tok::P('('), _)))
        })
    }

    /// Where the expression from here ends outside every bracket: at a closing bracket, the line's
    /// end, or (`at_cmp`) a `<` or `>`.
    pub(super) fn level_end(&self, at_cmp: bool) -> usize {
        let cmp = |t: &Tok| at_cmp && matches!(t, Tok::P('<' | '>'));
        words::level_end(&self.t, self.i, self.t.len(), Brackets::All, cmp)
    }

    /// `view(cyl.body) in views.right` — a picture asked of a solid (§6.11).  The brackets are
    /// what it is a picture of, `in` is the trailer it already is.
    fn derived(&mut self, name: DeclName) -> Option<StmtKind> {
        let lo = self.here().lo as usize;
        let w = self.word_at(self.i).unwrap_or_default().to_string();
        let sect = w == "section";
        let dims = w == "dimensions";
        self.i += 1;
        if !self.want_p('(') {
            return None;
        }
        let solid = self.refr()?;
        let mut at = None;
        if self.eat_p(',') {
            let Some(l) = self.slot_label() else {
                self.fail("a section is cut `at:` a plane");
                return None;
            };
            if l != "at" {
                self.fail(&format!(
                    "`{l}` is not what a {w} takes; a section is cut `at:` a plane"
                ));
                return None;
            }
            at = Some(self.refr()?);
        }
        if !self.want_p(')') {
            return None;
        }
        if sect && at.is_none() {
            self.fail("a section is cut at a plane: `section(body, at: mid) in front`");
            return None;
        }
        if !sect && at.is_some() {
            self.fail("`at:` cuts a section; a view of a solid takes none");
            return None;
        }
        if !self.peek_word("in") {
            self.fail("a picture is drawn in a view: `view(body) in views.right`");
            return None;
        }
        self.i += 1;
        let plane = self.refr()?;
        let end = self.prev_hi();
        let (class, _) = self.class_clause(end);
        self.end_of_stmt();
        Some(StmtKind::Derived(DerivedDecl {
            name,
            solid,
            plane,
            at,
            dims,
            class,
            span: Span::new(lo, self.prev_hi()),
        }))
    }

    /// `Tooth(base, a0: 30) in top class phantom` — the call and its trailers, the name already
    /// read (or minted, for a bare call).
    fn instance(&mut self, name: Name, lo: usize) -> Option<Instance> {
        let component = self.component_name()?;
        let args = self.inst_args()?;
        // the trailers an instance takes, in either order: `in top` — drawn in a view
        // (§6.7) — and `class phantom` — every declaration it makes carries the class
        let mut membership = Membership::default();
        let mut class = Classes::default();
        loop {
            if membership.plane().is_none() && self.peek_word("in") {
                let plo = self.here().lo as usize;
                self.i += 1;
                let r = self.refr()?;
                membership = Membership::written_at(r, Span::new(plo, self.prev_hi()));
            } else if class.is_empty() && self.peek_word("class") {
                let (c, _) = self.class_clause(self.here().lo as usize);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
            } else {
                break;
            }
        }
        self.end_of_stmt();
        Some(Instance {
            annotations: Default::default(),
            name,
            component,
            args,
            span: Span::new(lo, self.prev_hi()),
            membership,
            class,
        })
    }

    /// `Crank` or `engine.parts.Crank` — the component a call names, as one dotted name.
    pub(super) fn component_name(&mut self) -> Option<Name> {
        let first = self.ident()?;
        let mut text = first.text;
        while self.peek() == Some(&Tok::P('.')) {
            self.i += 1;
            text.push('.');
            text.push_str(&self.ident()?.text);
        }
        Some(Name { text, span: Span::new(first.span.lo as usize, self.prev_hi()) })
    }

    /// Whether a call stands at `j` — a name, or a dotted path, and its `(` — and whether the
    /// name is dotted, which only a module's component may be.
    pub(super) fn call_at(&self, j: usize) -> Option<bool> {
        let mut k = j;
        if !matches!(self.t.get(k).map(|(t, _)| t), Some(Tok::Ident(_))) {
            return None;
        }
        k += 1;
        while self.t.get(k).map(|(t, _)| t) == Some(&Tok::P('.'))
            && matches!(self.t.get(k + 1).map(|(t, _)| t), Some(Tok::Ident(_)))
        {
            k += 2;
        }
        (self.t.get(k).map(|(t, _)| t) == Some(&Tok::P('('))).then_some(k > j + 1)
    }

    /// `name :=` at the head of a statement, consumed — the name, or nothing and nothing eaten.
    fn defines(&mut self) -> Option<Name> {
        match (self.peek(), self.t.get(self.i + 1).map(|(t, _)| t)) {
            (Some(Tok::Ident(_)), Some(Tok::Define)) => {
                let name = self.ident()?;
                self.i += 1;
                Some(name)
            }
            _ => None,
        }
    }

    /// Whether the statement from here holds a token at the top bracket level before it ends —
    /// a word standing as itself, not as one segment of a dotted path (`lib.over.Long`).
    pub(super) fn statement_has(&self, want: impl Fn(&Tok) -> bool) -> bool {
        let dot = |j: usize| self.t.get(j).map(|(t, _)| t) == Some(&Tok::P('.'));
        let (mut paren, mut brace) = (0i32, 0i32);
        for (j, (t, _)) in self.t.iter().enumerate().skip(self.i) {
            if matches!(t, Tok::Ident(_)) && (dot(j + 1) || (j > 0 && dot(j - 1))) {
                continue;
            }
            match t {
                Tok::Nl => return false,
                Tok::P('(') | Tok::P('[') => paren += 1,
                Tok::P(')') | Tok::P(']') => paren -= 1,
                Tok::P('{') => brace += 1,
                Tok::P('}') if brace == 0 => return false,
                Tok::P('}') => brace -= 1,
                t if paren == 0 && brace == 0 && want(t) => return true,
                _ => {}
            }
        }
        false
    }

    /// Whether the brackets of the `solid(` at the cursor hold an `over` of their own: a face
    /// family (#243 L1), `sweep(Station(g).f over s in (0, 1))`.
    fn family_ahead(&self) -> bool {
        let mut depth = 0i32;
        for (t, _) in self.t.iter().skip(self.i + 1) {
            match t {
                Tok::Nl if depth == 0 => return false,
                Tok::P('(') | Tok::P('[') | Tok::P('{') => depth += 1,
                Tok::P(')') | Tok::P(']') | Tok::P('}') => {
                    depth -= 1;
                    if depth <= 0 {
                        return false;
                    }
                }
                Tok::Ident(w) if depth == 1 && w == "over" => return true,
                _ => {}
            }
        }
        false
    }

    /// `w := m * N / 2` — the number after `:=`, through the line's end.
    fn param(&mut self, name: Name) -> Option<StmtKind> {
        let (text, span) = self.value_after_define()?;
        Some(StmtKind::Param(ParamDecl { name, text, span, input: None }))
    }

    /// The expression after a `:=` just consumed, through the line's end or a trailing
    /// `hint(…)`, which is left for the caller.
    fn value_after_define(&mut self) -> Option<(String, Span)> {
        let after = self.t[self.i - 1].1.hi as usize;
        let (text, span, _, end) = self.raw_dimension(after);
        if text.is_empty() {
            self.fail("a definition says what the name is: `name := expression`");
            return None;
        }
        while self.i < self.t.len() && (self.t[self.i].1.lo as usize) < end {
            self.i += 1;
        }
        Some((text, span))
    }

    /// **`param NAME[: Type] [:= EXPR] [hint(EXPR)]`** — one of the document's inputs (§6.3):
    /// with a value, a number a host may give another; without one, an unknown the solve
    /// answers for, seeded by its `hint`.  At the top of the document only: a component's
    /// inputs are its formals, and a block's copies share the document's.
    ///
    /// **`param NAME: Type over (A, B)`** (#391 S1) is a parameter axis: a number that runs over
    /// the interval, neither a value nor an unknown, so it takes no `:=` and no `hint(…)`, and
    /// says what it is.
    fn input(&mut self) -> Option<StmtKind> {
        let word = self.here();
        self.i += 1; // `param`
        if self.in_body > u32::from(self.in_preview) {
            self.errs.push(SynErr {
                span: word,
                message: if self.in_comp {
                    "a component's inputs are its formals: `param` stands at the top of a document"
                } else {
                    "a `param` is an input of the document: it stands at the top level"
                }
                .into(),
            });
        }
        let name = self.ident()?;
        let ty = if self.eat_p(':') { Some(self.number_type()?) } else { None };
        let over = match self.peek_word("over") {
            true => Some(self.param_over(&name, ty)?),
            false => None,
        };
        let (text, span) = if self.peek() == Some(&Tok::Define) {
            self.i += 1;
            self.value_after_define()?
        } else {
            (String::new(), Span::at(self.prev_hi()))
        };
        let seed = self.hint_value()?;
        if let Some(o) = &over {
            let said = if !text.is_empty() {
                let ty = ty.and_then(|t| t.word()).unwrap_or("Angle");
                Some(format!("`{}` runs over an interval, so it has no value: `param {}: {ty} \
                              over (a, b)` and no `:=`", name.text, name.text))
            } else {
                seed.as_ref().map(|_| format!("`{}` runs over an interval and the solve never \
                    answers for it: a seed is for an unknown", name.text))
            };
            if let Some(message) = said {
                self.errs.push(SynErr { span: o.span, message });
            }
        }
        self.end_of_stmt();
        Some(StmtKind::Param(ParamDecl { name, text, span,
                                         input: Some(Input { ty, seed, over }) }))
    }

    /// `over (A, B)` after a `param`'s type (#391 S1), the word not yet eaten: the interval a
    /// parameter axis runs over, which says what it is (`Length`, `Angle` or `Scalar`).
    fn param_over(&mut self, name: &Name, ty: Option<Ty>) -> Option<crate::syntax::Over> {
        let span = self.here();
        self.i += 1; // `over`
        match ty {
            Some(Ty::Length | Ty::Angle | Ty::Scalar) => {}
            Some(_) => {
                self.errs.push(SynErr { span, message: format!(
                    "`{}` runs over an interval: a `Length`, an `Angle` or a `Scalar`",
                    name.text) });
            }
            None => {
                self.errs.push(SynErr { span, message: format!(
                    "a parameter that runs says what it is: `param {}: Angle over (a, b)`",
                    name.text) });
            }
        }
        let [from, to] = self.pair()?;
        Some(crate::syntax::Over { from, to, span })
    }

    /// `motion(…)` (#383), refused saying what each spelling is now — read past its brackets for
    /// the labels it was written with: one refusal for the declaration, naming the new words.
    fn retired_motion(&mut self) {
        let at = self.here();
        let mut labels: Vec<String> = Vec::new();
        let mut depth = 0i32;
        let mut j = self.i + 1;
        while let Some((t, _)) = self.t.get(j) {
            match t {
                Tok::P('(') => depth += 1,
                Tok::P(')') => {
                    depth -= 1;
                    if depth == 0 { break; }
                }
                Tok::Ident(w)
                    if depth == 1 && matches!(self.t.get(j + 1), Some((Tok::P(':'), _))) => {
                    labels.push(w.clone());
                }
                Tok::Nl if depth == 0 => break,
                _ => {}
            }
            j += 1;
        }
        let has = |l: &str| labels.iter().any(|x| x == l);
        let message = if has("relative_to") {
            "`motion(a, relative_to: b)` is retired (#383): a motion seen from another is said \
             where it is read, `under: a, seen_from: b` — on a carried point, an envelope, a \
             meshing and a swept or placed solid"
        } else if has("phase") {
            "`phase:` is retired (#383): roll 0 is as drawn, so a motion that stood turned by φ is \
             read at the roll shifted by φ / ratio (`t == τ + φ`, `from: start + φ`) — or its \
             member drawn turned"
        } else if has("along") {
            "`motion(along: l, advance: a)` is retired (#383): declare a frame and slide it, \
             `f := frame` then `std.Slide(f, along: l, advance: a)`"
        } else {
            "`motion(about: …)` is retired (#383): declare a frame and turn it, `f := frame` then \
             `std.Turn(f, about: l, ratio: r)` (a screw: `advance: a`), or about a point of a view \
             `std.Spin(f, about: o, on: P, ratio: r)`"
        };
        self.fail_at(at, message);
        self.i = j.min(self.t.len());
        if self.t.get(self.i).is_some() { self.i += 1; }
    }

    /// A formal's type after its `:` — the word, read through `Ty::parse`.
    fn formal_type(&mut self) -> Option<Ty> {
        let tname = self.ident()?;
        let Some(ty) = Ty::parse(&tname.text) else {
            self.errs.push(SynErr {
                span: tname.span,
                message: if tname.text == "motion" {
                    "a motion is a frame's now (#383): a formal `m: frame` is read `under: m`, and \
                     one that was a relative motion takes its observer as a second, `seen_from: \
                     frame`".to_string()
                } else {
                    format!("`{}` is not a type", tname.text)
                },
            });
            return None;
        };
        Some(ty)
    }

    /// A formal's type, or the interface it names (§8.1 [0.84]): a type word, else a name —
    /// dotted for a module's, `spiral.Gear` — which makes it a `group` formal typed by that
    /// interface.  Whether the name is one is the flattener's to say, once modules are linked.
    fn formal_or_interface(&mut self) -> Option<(Ty, Option<Name>)> {
        let word = self.word_at(self.i).map(str::to_string);
        // `motion` was a type until 0.135, and is refused as one rather than read as a name
        if word.as_deref().is_some_and(|w| Ty::parse(w).is_some() || w == "motion") {
            return self.formal_type().map(|ty| (ty, None));
        }
        Some((Ty::Group, Some(self.component_name()?)))
    }

    /// `name: type` — a component's formal, or an interface's member, written alike.
    fn formal(&mut self) -> Option<Formal> {
        let name = self.ident()?;
        if !self.want_p(':') {
            return None;
        }
        let frame = self.word_at(self.i) == Some("frame");
        let (ty, interface) = self.formal_or_interface()?;
        // a point formal may say the plane it is drawn in, which lets a call leave it out
        let within = if self.peek_word("in") {
            let at = self.here();
            self.i += 1; // `in`
            if ty != Ty::Ent(EntKind::Point) {
                let m = "only a `point` formal is drawn `in` a plane: `center: point in on`";
                self.fail_at(at, m);
                return None;
            }
            Some(self.ident()?)
        } else {
            None
        };
        // a number may say what it is where a call leaves it out (#383)
        let default = if self.peek() == Some(&Tok::P('=')) {
            let at = self.here();
            self.i += 1;
            if !ty.number() {
                self.fail_at(at, "only a number formal has a default: `advance: Length = 0mm`");
                return None;
            }
            Some(self.expr_until(',')?)
        } else {
            None
        };
        let span = Span::new(name.span.lo as usize, self.prev_hi());
        Some(Formal { name, ty, span, interface, within, default, frame })
    }

    /// `interface Gear { O: point, M: point, ax: line }` — a group's shape, named (§8.1 [0.84]):
    /// its members written as formals are, separated by commas or line breaks.
    pub(super) fn interface(&mut self) -> Option<super::super::Interface> {
        let lo = self.here().lo as usize;
        self.i += 1; // `interface`
        let name = self.ident()?;
        if Ty::parse(&name.text).is_some() {
            self.fail_at(name.span, &format!(
                "`{}` is a type of the language's own: an interface takes a name of its own",
                name.text
            ));
            return None;
        }
        if !self.want_p('{') {
            return None;
        }
        let mut members = Vec::new();
        loop {
            self.skip_ends();
            if self.eat_p('}') {
                break;
            }
            let member = self.formal()?;
            if let Some(w) = &member.within {
                self.fail_at(w.span, "an interface's member is given, never made: it is drawn \
                                      in no plane of its own");
                return None;
            }
            members.push(member);
            if !self.eat_p(',') && !matches!(self.peek(), Some(Tok::Nl) | Some(Tok::P('}'))) {
                self.fail("expected `,`, a line break or `}` after an interface's member");
                return None;
            }
        }
        self.end_of_stmt();
        Some(super::super::Interface { name, members, span: Span::new(lo, self.prev_hi()), module: None })
    }

    /// An input's type: a formal's, and a number — `Length`, `Angle`, `Scalar` or `Int`.
    fn number_type(&mut self) -> Option<Ty> {
        let at = self.here();
        let ty = self.formal_type()?;
        if matches!(ty, Ty::Side | Ty::Group | Ty::Set | Ty::Ent(_) | Ty::Path) {
            self.errs.push(SynErr {
                span: at,
                message: "a `param` is a number: `Length`, `Angle`, `Scalar` or `Int`".into(),
            });
            return None;
        }
        Some(ty)
    }

    /// **`name := expression`, the one way a name is defined** (§5): what stands after `:=` is
    /// the value, and the name is that value — a number (a param), an element, a chain joined
    /// by `->`, an instance, a group, a curve, a picture.
    fn definition(&mut self, name: Name, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let lo = name.span.lo as usize;
        if matches!(self.peek(), Some(Tok::Nl) | None) {
            self.fail("a definition says what the name is: `name := expression`");
            return None;
        }
        let word = self.word_at(self.i).map(str::to_string);
        let call = matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('(')));
        let dotted_call = self.call_at(self.i) == Some(true);
        let kind = match word.as_deref() {
            // `ball := { p | p distance(12mm) c }` — a set, the points its body holds (§6.21)
            None if self.set_ahead() => {
                let lit = self.set_lit(next_id)?;
                self.end_of_stmt();
                StmtKind::Set(crate::syntax::SetDecl { name, lit })
            }
            // `dims := {bore: 16mm, axis: datum}` — a group, its members named in braces
            None if self.peek() == Some(&Tok::P('{')) => {
                let fields = self.group_members()?;
                self.end_of_stmt();
                StmtKind::Group(crate::syntax::GroupDecl { name, fields })
            }
            // `o := (10mm, 20mm)` — a vector, a group of its components: `o.x`, `o.y`
            None if self.tuple_ahead() => {
                let fields = self
                    .vector("")?
                    .into_iter()
                    .map(|(key, (_, text, span))| crate::syntax::InstArg {
                        label: Some(Name { text: key, span }),
                        value: crate::syntax::InstVal::Expr(text),
                        span,
                    })
                    .collect();
                self.end_of_stmt();
                StmtKind::Group(crate::syntax::GroupDecl { name, fields })
            }
            // `f := frame` (#383): a frame declared bare — the word read so only as a definition's
            // whole value, since `frame` is a name a document may give
            Some("frame") if words::bare_frame_at(&self.t, self.i) || call => {
                self.i += 1;
                let d = self.frame_decl(DeclName::Written(name))?;
                self.end_of_stmt();
                StmtKind::Decl(d)
            }
            // `motion(…)` is retired (#383): what it said is a frame's joint, held by relations
            Some("motion") if call => {
                self.retired_motion();
                return None;
            }
            Some("view" | "section" | "dimensions") if call => {
                self.derived(DeclName::Written(name))?
            }
            // `sec := prof mapped(from -> a, to -> b) in n` — a face carried whole (#357)
            Some(_) if self.past_ref(self.i).is_some_and(|j| self.word_at(j) == Some("mapped")) => {
                StmtKind::Block(self.mapped_face(name)?)
            }
            // `n := base at τ` — a frame, a solid or a point read at an instant (#391 S2)
            Some(_) if self.past_ref(self.i).is_some_and(|j| self.word_at(j) == Some("at")) => {
                StmtKind::Decl(self.read_at(name)?)
            }
            // `n := base in f` — a frame, a solid or a point drawn in a jointed frame, where the
            // frame's value puts it (#391 S3); `p := point in P` is a declaration's membership
            Some(w) if EntKind::parse(w).is_none()
                && self.past_ref(self.i).is_some_and(|j| self.word_at(j) == Some("in")) =>
            {
                StmtKind::Decl(self.drawn_in(name)?)
            }
            // `carried(at: τ)` is retired (#391 S2) for the reading at an instant
            Some(_)
                if self.past_ref(self.i).is_some_and(|j| self.word_at(j) == Some("carried")) =>
            {
                self.i = self.past_ref(self.i).unwrap_or(self.i);
                self.fail("`carried(at: τ)` is retired (#391): a frame is read at an instant, \
                           `n := base at τ`, its frames on the `at` (`base at(under: m, \
                           seen_from: g) τ`)");
                return None;
            }
            // `duct := sweep(Station(g).f over s in (0, 1))` — a face of a component as one of
            // its formals runs, one dimension up from a curve (#243 L1) — and `s := sweep(tool
            // over t in (a, b))`, a solid a motion sweeps (#391 S2)
            Some("sweep") if call => StmtKind::Decl(self.sweep_decl(DeclName::Written(name))?),
            // `solid(… over s in (a, b))` is retired (#391 S2) for the sweep
            Some("solid") if call && self.family_ahead() => {
                self.fail("a face family is `sweep(Component(…).face over s in (a, b))` (#391): \
                           `solid(… over …)` is retired");
                return None;
            }
            // `k := trace(leg.toe over theta in (0, 360))` — a point of a component as one of
            // its formals runs (§6.5, #391 S2)
            Some("trace") if call => {
                self.i += 2; // `trace(`
                let curve = self.curve_spec()?;
                if !self.want_p(')') {
                    return None;
                }
                let at = self.prev_hi();
                let (class, class_span) = self.class_clause(at);
                if self.peek_word("in") {
                    self.fail("`in` puts points on a plane, and a curve has none of its own");
                    return None;
                }
                self.end_of_stmt();
                StmtKind::Decl(Decl::curve(DeclName::Written(name), curve, class, class_span))
            }
            // a curve written bare is retired (#391 S2) for the trace
            _ if self.statement_has(|t| matches!(t, Tok::Ident(w) if w == "over")) => {
                self.fail("a point of a component followed as one of its formals runs is \
                           `trace(leg.toe over theta in (a, b))` (#391): `k := leg.toe over …` \
                           is retired");
                return None;
            }
            // a param may bear an element keyword's spelling (`face := -(fw + d)`), so arithmetic
            // after one is a number read, not a declaration
            Some(_) if matches!(self.t.get(self.i + 1).map(|(t, _)| t),
                Some(Tok::P('+' | '-' | '*' | '/' | '^'))) => self.param(name)?,
            _ if dotted_call => StmtKind::Instance(self.instance(name, lo)?),
            _ if self.chain_starts() => return self.chain(next_id, out, Some(name)),
            Some(w) if call
                && !crate::constraints::is_operator(w)
                && crate::expr::builtin(w).is_none()
                && !crate::expr::MEASURES.iter().any(|m| m.0 == w) =>
            {
                StmtKind::Instance(self.instance(name, lo)?)
            }
            // anything else is a number worked out while elaborating: a param
            _ => self.param(name)?,
        };
        let span = Span::new(lo, self.prev_hi());
        let id = self.mint_stmt(next_id, span)?;
        out.push(Stmt { id, kind, span, chained: Chained::No });
        Some(())
    }

    /// `sec := prof mapped(from -> a, to -> b, third -> c) in n` (#357): the face `prof` carried
    /// whole into the plane `n`, its pairs read as a point's map reads them.  A `cycle` over the
    /// face's loop that makes no copies (`Block::image`): the flattener builds the image once the
    /// loop can be walked.
    fn mapped_face(&mut self, name: Name) -> Option<Block> {
        let lo = name.span.lo as usize;
        let prof = self.refr()?;
        let word = self.ident()?;
        let args = self.op_args(&word.text, crate::constraints::Fixity::Infix)?;
        if !self.eat_word("in") {
            self.fail("a face carried by a map is drawn in a plane: `sec := prof mapped(…) in n`");
            return None;
        }
        let plane = self.refr()?;
        self.end_of_stmt();
        let var = Name { text: format!("#map{lo}"), span: Span::at(word.span.lo as usize) };
        Some(Block {
            kind: BlockKind::Cycle,
            count: String::new(),
            head: word.span,
            over: Some(EdgesOf { var, chain: prof, zip: Vec::new() }),
            binder: None,
            body: Vec::new(),
            joint: None,
            about: None,
            index_reads: Vec::new(),
            span: Span::new(lo, self.prev_hi()),
            image: Some(crate::syntax::FaceImage { name, word, args, plane }),
        })
    }

    /// `n := x at τ` (#391 S2): `x` read at the instant `τ` of the roll, by the frames it rides
    /// on, or by the frame `at(under: m)` names where it rides on none, seen from `seen_from: g`
    /// — its arguments held as a carried point's are (`under:` and `seen_from:` references, the
    /// roll the relation's number), so its carrier is found as theirs is
    /// (`relations::riding_point`).  What it is the flattener says, once it knows what `x` is: a
    /// frame (`flatten::carried`), a solid placed at that roll, or a point held to `x` there.
    fn read_at(&mut self, name: Name) -> Option<Decl> {
        let source = self.refr()?;
        let word = Name { text: "at".into(), span: self.here() };
        let args = self.instant()?;
        let at = self.prev_hi();
        let (class, class_span) = self.class_clause(at);
        self.end_of_stmt();
        Some(Decl {
            kind: EntKind::Frame,
            seed: vec![0.0; 3],
            seed_text: vec![None; 3],
            class,
            class_span,
            carried: Some(crate::syntax::CarriedFrame { source, word, args }),
            ..Decl::point(DeclName::Written(name), [0.0; 2], None)
        })
    }

    /// `n := x in f` (#391 S3): `x` drawn in the jointed frame `f`, where `f`'s value puts it —
    /// the reading at an instant whose instant is the frame's (`fix(value == τ) f`) and not the
    /// statement's, held as a carried point's arguments are, `in: f` naming the frame.  What it is
    /// the flattener says, as for `x at τ`: a frame, a solid or a point.
    fn drawn_in(&mut self, name: Name) -> Option<Decl> {
        let source = self.refr()?;
        let word = Name { text: crate::syntax::DRAWN_IN.into(), span: self.here() };
        self.i += 1; // `in`
        let frame = self.refr()?;
        let args = vec![OpArg::Named(word.clone(), Arg::Ref(frame))];
        let at = self.prev_hi();
        let (class, class_span) = self.class_clause(at);
        self.end_of_stmt();
        Some(Decl {
            kind: EntKind::Frame,
            seed: vec![0.0; 3],
            seed_text: vec![None; 3],
            class,
            class_span,
            carried: Some(crate::syntax::CarriedFrame { source, word, args }),
            ..Decl::point(DeclName::Written(name), [0.0; 2], None)
        })
    }

    /// Parse a statement or desugar a chain, recovering at the next terminator on failure.
    pub(super) fn chain_or_one(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let start = self.here();
        let mut annotations = crate::semantics::Annotations::default();
        let mut marked = false;
        loop {
            let slot = if self.peek_word("private") { &mut annotations.private }
                else if self.peek_word("construction") { &mut annotations.roles.construction }
                else if self.peek_word("centerline") { &mut annotations.roles.centerline }
                else { break };
            if *slot { self.fail("a geometry modifier may be stated only once"); return None; }
            *slot = true;
            marked = true;
            self.i += 1;
        }
        if marked && self.peek_word("in") {
            self.fail("put geometry modifiers on declarations or component instances inside the plane block");
            return None;
        }
        let first = out.len();
        let result = self.unannotated_chain_or_one(next_id, out);
        if marked && result.is_some() {
            let mut geometry = false;
            for st in &mut out[first..] {
                let slot = match &mut st.kind {
                    StmtKind::Decl(d) => Some(&mut d.annotations),
                    StmtKind::Instance(i) => Some(&mut i.annotations),
                    StmtKind::Chain(c) => Some(&mut c.annotations),
                    // A chain emits its constraints beside the geometry it declares.
                    StmtKind::Relation(_) if st.chained != Chained::No => None,
                    _ => {
                        self.errs.push(SynErr { span: start,
                            message: "geometry modifiers apply to declarations, chains, or component instances".into() });
                        None
                    }
                };
                if let Some(slot) = slot { *slot = annotations; geometry = true; }
            }
            if !geometry { self.errs.push(SynErr { span: start,
                message: "a geometry modifier needs a declaration or component instance".into() }); }
            // The modifier belongs to geometry, not a desugared unary constraint. Removing
            // `radius(...)` from `construction c := radius(...) circle` must retain the role.
            if let Some(st) = out[first..].iter_mut().find(|st|
                matches!(st.kind, StmtKind::Decl(_) | StmtKind::Instance(_) | StmtKind::Chain(_))) {
                st.span.lo = start.lo;
            }
        }
        result
    }

    fn unannotated_chain_or_one(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        // the top level reads a definition before it gets here (`parse_at`), so this is one
        // written inside a body, where a word would be a name of the body's (§9.9)
        if self.word_definition_ahead().is_some() {
            self.fail("a relation word is defined at the top of a file, as a component is, \
                       not inside a body");
            return None;
        }
        // `in top { … }` before anything else: `in` opens no chain and no statement of any
        // other kind, and no name may be called `in`
        if self.peek_word("in")
            && (matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(_)))
                || words::named_link_at(&self.t, self.i + 1)
                || words::named_frame_at(&self.t, self.i + 1))
        {
            if !self.take_statement(self.here()) {
                return None;
            }
            return self.in_block(next_id, out);
        }
        // `initially { … }` (#339 D3): rows of a family's home alone, hoisted as an `in` body is
        if self.peek_word("initially")
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('{')))
        {
            if !self.take_statement(self.here()) {
                return None;
            }
            return self.initially(next_id, out);
        }
        if self.peek_word("param") {
            let lo = self.here().lo as usize;
            let kind = self.input()?;
            let span = Span::new(lo, self.prev_hi());
            let id = self.mint_stmt(next_id, span)?;
            out.push(Stmt { id, kind, span, chained: Chained::No });
            return Some(());
        }
        if let Some(name) = self.defines() {
            return self.definition(name, next_id, out);
        }
        if self.inline_set_ahead() {
            return self.inline_set_use(next_id, out);
        }
        if self.chain_starts() {
            return self.chain(next_id, out, None);
        }
        let lo = self.here().lo as usize;
        let kind = self.stmt(next_id)?;
        let id = self.mint_stmt(next_id, Span::new(lo, self.prev_hi()))?;
        out.push(Stmt { id, kind, span: Span::new(lo, self.prev_hi()), chained: Chained::No });
        Some(())
    }

    /// Hoist an `in PLANE` body and stamp its declarations with inherited membership.
    /// Keep the header and brace spans so deleting the plane can remove the wrapper.
    fn in_block(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let lo = self.here().lo as usize;
        self.i += 1; // `in`
        // `in (g := frame) { … }` (#383): the header declares the frame the block draws in,
        // `g := frame` then `in g { … }` — its declaration a statement of its own, before the body
        let declares = words::named_link_at(&self.t, self.i) || words::named_frame_at(&self.t, self.i);
        let plane = if declares {
            self.i += 1; // `(`
            let n = self.ident()?;
            self.i += 1; // `:=`
            let bare = self.word_at(self.i) == Some("frame");
            let kind = if bare { EntKind::Frame } else {
                EntKind::parse(self.word_at(self.i)?).expect("named_link_at")
            };
            if kind != EntKind::Frame {
                self.fail("an `in` block's header declares the frame it draws in: \
                           `in (f := frame) { … }`, or a plane, `in (P := plane(u: a, v: b)) { … }`");
                return None;
            }
            self.i += 1;
            let decl = if bare {
                self.frame_decl(DeclName::Written(n.clone()))?
            } else {
                self.decl(kind, Some(n.clone()))?
            };
            let span = Span::new(n.span.lo as usize, self.prev_hi());
            if !self.want_p(')') {
                return None;
            }
            let id = self.mint_stmt(next_id, span)?;
            out.push(Stmt { id, kind: StmtKind::Decl(decl), span, chained: Chained::No });
            Ref { span: n.span, root: n, path: Vec::new() }
        } else {
            self.refr()?
        };
        self.skip_ends();
        if !matches!(self.peek(), Some(Tok::P('{'))) {
            self.fail("an `in` block is `in PLANE { … }`");
            return None;
        }
        let header = Span::new(lo, self.here().hi as usize);
        // inside a component the block is the one way to write a part's view in one place —
        // the plane is a formal, and nothing the document deletes reaches the header; inside a
        // root block a header buried in another statement's span is a splice no deletion
        // could compose, so there the clause is written per declaration
        // `in` inside `in` nests (#383): what is drawn there rides on both, so only the
        // blocks that are no `in` count
        let nested = self.in_body - self.in_in > u32::from(self.in_preview);
        if nested && !self.in_comp {
            self.fail(
                "an `in` block stands at the top level, in another `in` block or in a component; \
                 inside a block here, write the clause on each declaration",
            );
        }
        self.in_in += 1;
        let got = self.braced_body(next_id);
        self.in_in -= 1;
        let (mut body, joint) = got?;
        self.no_open_joint(joint, "an `in` block");
        let close = self.t.get(self.i.wrapping_sub(1)).map(|(_, s)| *s).unwrap_or_default();
        stamp_frame(&mut body, &plane, &mut self.errs);
        // recorded only where a deletion could reach it: a nested block is already an error
        if !nested {
            self.in_blocks.push(InBlock { plane, header, close, declares });
        }
        out.extend(body);
        Some(())
    }

    /// Hoist an `initially { … }` body (#339 D3), each relation in it marked a row of the home
    /// alone.  It says only relations: what a station draws, it draws everywhere.
    fn initially(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        self.i += 1; // `initially`
        let (mut body, joint) = self.braced_body(next_id)?;
        self.no_open_joint(joint, "an `initially` block");
        stamp_initially(&mut body, &mut self.errs);
        out.extend(body);
        Some(())
    }

    /// `over (a, b)` — two expressions over whatever parameters are in scope.
    /// What follows a curve's `=`: `REF over IDENT in (a, b)`, or
    /// `Component(args).path over IDENT in (a, b)`.  A component name is told from a point's by
    /// the `(` after it, the same token that tells an instance from a reference elsewhere.
    pub(super) fn curve_spec(&mut self) -> Option<CurveSpec> {
        let target = if self.call_at(self.i).is_some() {
            let component = self.component_name()?;
            let lo = component.span.lo as usize;
            let args = self.inst_args()?;
            if !self.eat_p('.') {
                self.fail("the point the curve traces follows the instance: `Component(…).point`");
                return None;
            }
            let point = self.refr()?;
            let at = lo;
            let inst = Instance {
                annotations: Default::default(),
                // a key the source cannot write, as an anonymous declaration's is
                name: Name { text: format!("#c{at}"), span: Span::at(at) },
                component,
                args,
                span: Span::new(lo, self.prev_hi()),
                membership: Membership::default(),
                class: Classes::default(),
            };
            CurveTarget::Anon(inst, point)
        } else {
            CurveTarget::Drawn(self.refr()?)
        };
        if !self.eat_word("over") {
            self.fail("a curve says which formal runs: `over theta in (a, b)`");
            return None;
        }
        // two formals that run, each with its interval: a surface (#326 R5)
        let two = self.peek() == Some(&Tok::P('('));
        if two {
            self.i += 1;
        }
        let formal = |p: &mut Self| -> Option<(Name, (String, String))> {
            let swept = p.ident()?;
            if !p.eat_word("in") {
                p.fail("a curve says the interval its formal runs over: `in (a, b)`");
                return None;
            }
            let [(a, _), (b, _)] = p.pair()?;
            Some((swept, (a, b)))
        };
        let (swept, domain) = formal(self)?;
        let second = if two {
            if !self.want_p(',') {
                return None;
            }
            let s = formal(self)?;
            if !self.want_p(')') {
                return None;
            }
            Some(s)
        } else {
            None
        };
        Some(CurveSpec { target, swept, domain, second, of: None })
    }

    /// What follows a face family's `sweep` (#391 S2): `(Component(args).face over IDENT in (a,
    /// b))`, the instance written in place (#243 L1, finding 11).  The instance's key is one the
    /// source cannot write, as a curve's is; the face is read under it once the flattener has
    /// bound it.
    pub(super) fn family(&mut self, name: DeclName) -> Option<crate::syntax::Decl> {
        self.i += 1; // `sweep`
        let open = self.here().lo as usize;
        if !self.want_p('(') {
            return None;
        }
        if self.call_at(self.i).is_none() {
            self.fail(
                "a face family is the face of an instance written in place: \
                 `sweep(Component(…).face over s in (a, b))`",
            );
            return None;
        }
        let component = self.component_name()?;
        let lo = component.span.lo as usize;
        let args = self.inst_args()?;
        if !self.eat_p('.') {
            self.fail("the face a family sweeps follows the instance: `Component(…).face`");
            return None;
        }
        let face = self.refr()?;
        if !self.eat_word("over") {
            self.fail("a face family says which formal runs: `over s in (a, b)`");
            return None;
        }
        let swept = self.ident()?;
        if !self.eat_word("in") {
            self.fail("a face family says the interval its formal runs over: `in (a, b)`");
            return None;
        }
        let [(a, _), (b, _)] = self.pair()?;
        if !self.want_p(')') {
            return None;
        }
        let list_span = Span::new(open, self.prev_hi());
        let instance = Instance {
            annotations: Default::default(),
            name: Name { text: format!("#f{lo}"), span: Span::at(lo) },
            component,
            args,
            span: Span::new(lo, list_span.hi as usize - 1),
            membership: Membership::default(),
            class: Classes::default(),
        };
        let at = self.prev_hi();
        let (class, class_span) = self.class_clause(at);
        if self.peek_word("in") || self.peek_word("hint") {
            self.fail("a face family stands where its stations put it: it takes no `in` or `hint`");
            return None;
        }
        self.end_of_stmt();
        let end = self.prev_hi();
        let mut membership = Membership::default();
        membership.set_span(Span::at(end));
        let slots = EntKind::Solid.fields().iter()
            .filter(|(_, f)| *f != crate::model::Field::Scalar).count();
        let spec = crate::syntax::FamilySpec { instance, face, swept, domain: (a, b), held: None,
            legs: Vec::new() };
        Some(crate::syntax::Decl {
            annotations: Default::default(),
            kind: EntKind::Solid,
            name,
            children: vec![Vec::new(); slots],
            seed: Vec::new(),
            seed_text: Vec::new(),
            seed_spans: Vec::new(),
            hint_span: None,
            knots: None,
            weights: None,
            curve: None,
            computed: None,
            class,
            class_span: if class_span.is_empty() { Span::at(end) } else { class_span },
            seed_at: None,
            seed_names: Vec::new(),
            sweep: Some(crate::syntax::Sweep::Family(Box::new(spec))),
            seen_from: None,
            angular_span: None,
            membership,
            list_span,
            close: None,
            mint_close: None,
            through: None,
            normal: None,
            sense: None,
            bare: false,
            carried: None,
        })
    }

    /// What follows a definition's `sweep` (#391 S2): a face family, `sweep(Component(args).face
    /// over s in (a, b))` (`family`), or a solid a frame's motion sweeps — `sweep(tool over t in
    /// (a, b))` over an interval of the roll, `sweep(tool, through: body)` over the roll during
    /// which it meets a body — with `under:` the frame it is swept by where it rides on none and
    /// `seen_from:` the one it is seen from.
    fn sweep_decl(&mut self, name: DeclName) -> Option<crate::syntax::Decl> {
        if self.call_at(self.i + 2).is_some() {
            return self.family(name);
        }
        self.i += 1; // `sweep`
        let open = self.here().lo as usize;
        if !self.want_p('(') {
            return None;
        }
        let tool = self.refr()?;
        let mut over = None;
        if self.peek_word("over") {
            over = Some(self.runs_over()?);
        }
        let (mut under, mut seen_from, mut through) = (None, None, None);
        while self.eat_p(',') {
            let label = self.slot_label();
            let slot = match label.as_deref() {
                Some("under") => &mut under,
                Some("seen_from") => &mut seen_from,
                Some("through") => &mut through,
                _ => {
                    self.fail("a sweep takes its tool over an interval of the roll, and `under:` \
                               the frame it is swept by, `seen_from:` the one it is seen from, or \
                               `through:` a body");
                    return None;
                }
            };
            if slot.is_some() {
                self.fail(&format!("`{}` is given twice", label.unwrap_or_default()));
                return None;
            }
            *slot = Some(self.refr()?);
        }
        if !self.want_p(')') {
            return None;
        }
        let list_span = Span::new(open, self.prev_hi());
        use crate::syntax::Sweep;
        let sweep = match (over, through) {
            (Some((over, from, to)), None) => Sweep::Swept { motion: under, from, to, over },
            (None, Some(body)) => Sweep::Rolled { motion: under, body },
            _ => {
                self.fail("a sweep runs its tool over an interval of the roll, `sweep(tool over t \
                           in (a, b))`, or through a body, `sweep(tool, through: body)`");
                return None;
            }
        };
        let at = self.prev_hi();
        let (class, class_span) = self.class_clause(at);
        if self.peek_word("in") || self.peek_word("hint") {
            self.fail("a sweep stands where its motion puts it: it takes no `in` or `hint`");
            return None;
        }
        self.end_of_stmt();
        let end = self.prev_hi();
        let mut membership = Membership::default();
        membership.set_span(Span::at(end));
        Some(crate::syntax::Decl {
            kind: EntKind::Solid,
            children: vec![vec![crate::syntax::Kid::Ref(tool)]],
            seed: Vec::new(),
            seed_text: Vec::new(),
            class,
            class_span: if class_span.is_empty() { Span::at(end) } else { class_span },
            sweep: Some(sweep),
            seen_from,
            membership,
            list_span,
            ..crate::syntax::Decl::point(name, [0.0; 2], None)
        })
    }

    /// `( expr, expr )` — an interval, a computed point's coordinates.
    pub(super) fn pair(&mut self) -> Option<[(String, Span); 2]> {
        if !self.want_p('(') {
            return None;
        }
        let a = self.expr_until(',')?;
        if !self.want_p(',') {
            return None;
        }
        let b = self.expr_until(')')?;
        if !self.want_p(')') {
            return None;
        }
        Some([a, b])
    }

    /// `(arg, label: arg, …)` — what an instance is given, the `(` not yet eaten.
    fn inst_args(&mut self) -> Option<Vec<InstArg>> {
        self.args_between('(', ')')
    }

    /// A group's members in `{ … }`, every one named, the `{` not yet eaten.
    fn group_members(&mut self) -> Option<Vec<InstArg>> {
        let fields = self.args_between('{', '}')?;
        if fields.iter().any(|f| f.label.is_none()) {
            self.fail("every group member needs a name: `dims := {bore: 16mm}`");
            return None;
        }
        Some(fields)
    }

    /// A call's arguments in `( … )`, or a group's members in `{ … }`.
    fn args_between(&mut self, open: char, close: char) -> Option<Vec<InstArg>> {
        if !self.want_p(open) {
            return None;
        }
        let mut args = Vec::new();
        while !self.eat_p(close) {
            args.push(self.inst_arg()?);
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(close)) {
                self.fail(&format!("expected `,` or `{close}`"));
                return None;
            }
        }
        Some(args)
    }

    /// `component Name(formals) { body }`.
    /// `use engine.crank` — the dotted name, and where the statement stands.
    pub(super) fn use_stmt(&mut self) -> Option<Use> {
        let lo = self.here().lo as usize;
        self.i += 1; // `use`
        let mut name = self.ident()?.text;
        while self.eat_p('.') {
            name.push('.');
            name.push_str(&self.ident()?.text);
        }
        // `use std (horizontal, vertical)`: the names this file reads bare (§14.4 [0.48])
        let mut names = Vec::new();
        if self.eat_p('(') {
            names = self.ident_list()?;
            if names.is_empty() {
                self.fail("an import names what it reads bare: `use std (horizontal)`, or no \
                           brackets at all");
                return None;
            }
        }
        self.end_of_stmt();
        Some(Use { name, span: Span::new(lo, self.prev_hi()), names })
    }

    pub(super) fn component(&mut self, next_id: &mut u32) -> Option<Component> {
        let lo = self.here().lo as usize;
        self.i += 1; // `component`
        let name = self.ident()?;
        let mut formals = Vec::new();
        if self.eat_p('(') {
            while !self.eat_p(')') {
                formals.push(self.formal()?);
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
        }
        // `component Sphere(center: point, r: Length) := { p | … }` — a family of sets: the
        // instance is the set, and its body is the literal's (§6.21)
        if self.peek() == Some(&Tok::Define) {
            self.i += 1;
            if !self.set_ahead() {
                self.fail("a component defined with `:=` is a set: `component Name(…) := { p | … }`");
                return None;
            }
            let was = std::mem::replace(&mut self.in_comp, true);
            let got = self.set_lit(next_id);
            self.in_comp = was;
            let set = got?;
            self.end_of_stmt();
            return Some(Component {
                name: Some(name),
                formals,
                body: Vec::new(),
                span: Span::new(lo, self.prev_hi()),
                module: None,
                set: Some(set),
            });
        }
        let was = std::mem::replace(&mut self.in_comp, true);
        let got = self.braced_body(next_id);
        self.in_comp = was;
        let (body, joint) = got?;
        self.no_open_joint(joint, "a component");
        Some(Component {
            name: Some(name),
            formals,
            body,
            span: Span::new(lo, self.prev_hi()),
            module: None,
            set: None,
        })
    }

    /// Whether the statement from here uses a set written in place (#103): a reference, `coincident`
    /// or `tangent`, then a set literal or a call — `q coincident { p | … }`, `l tangent
    /// std.Cylinder(ax, r: 8mm)`.
    fn inline_set_ahead(&self) -> bool {
        // an element's word opens a declaration, never an operand
        if self.word_at(self.i).is_none_or(|w| crate::model::EntKind::parse(w).is_some()) {
            return false;
        }
        let Some(j) = self.past_ref(self.i) else { return false };
        if !matches!(self.word_at(j), Some("coincident" | "tangent" | "inside" | "outside")) {
            return false;
        }
        // a call is a component's: not a word of the language's own (`hint(…)`)
        let call = self.call_at(j + 1).is_some_and(|dotted| {
            dotted || self.word_at(j + 1).is_some_and(|w| !crate::syntax::reserved_word(w) && !crate::constraints::is_operator(w))
        });
        self.set_at(j + 1) || call
    }

    /// `q coincident { p | … }` or `l tangent std.Cylinder(ax, r: 8mm)`: the set written in place
    /// is its own statement, a set or an instance under a key the source cannot write, read by
    /// the relation as a name (#103).  The set is a link of the statement's text: deleting it
    /// alone is refused, as a chain's link is.
    fn inline_set_use(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let lo = self.here().lo as usize;
        let left = self.refr()?;
        let word = self.ident()?;
        let at = self.here().lo as usize;
        let key = |prefix: &str| Name { text: format!("#{prefix}{at}"), span: Span::at(at) };
        let (name, kind) = if self.set_ahead() {
            let name = key("s");
            let lit = self.set_lit(next_id)?;
            (name.clone(), StmtKind::Set(crate::syntax::SetDecl { name, lit }))
        } else {
            let name = key("i");
            let component = self.component_name()?;
            let args = self.inst_args()?;
            let inst = Instance {
                annotations: Default::default(),
                name: name.clone(),
                component,
                args,
                span: Span::new(at, self.prev_hi()),
                membership: Membership::default(),
                class: Classes::default(),
            };
            (name, StmtKind::Instance(inst))
        };
        let span = Span::new(at, self.prev_hi());
        let id = self.mint_stmt(next_id, span)?;
        out.push(Stmt { id, kind, span, chained: Chained::Link });
        let right = Ref { root: Name { text: name.text, span }, path: Vec::new(), span };
        let rel = self.relation_tail(word, crate::constraints::Fixity::Infix, vec![left, right], Vec::new(), lo)?;
        let span = Span::new(lo, self.prev_hi());
        let id = self.mint_stmt(next_id, span)?;
        out.push(Stmt { id, kind: StmtKind::Relation(rel), span, chained: Chained::No });
        Some(())
    }

    /// Whether a set literal opens here: `{`, one name, `|` (§6.21), line breaks between them
    /// read past as the lexer reads past them.
    fn set_ahead(&self) -> bool {
        self.set_at(self.i)
    }

    /// `set_ahead` at token `i` (`words::set_at`, which the colouring asks too).
    fn set_at(&self, i: usize) -> bool {
        set_at(&self.t, i).is_some()
    }

    /// `{ p | statement … }` — the bound point, then the body through the closing brace, each
    /// statement as a braced body's (§6.21).
    fn set_lit(&mut self, next_id: &mut u32) -> Option<SetLit> {
        let lo = self.here().lo as usize;
        self.i += 1; // `{`
        self.skip_ends();
        let bound = self.ident()?;
        self.skip_ends();
        self.i += 1; // `|`
        if self.in_body as usize >= self.limits.max_depth {
            self.fail(&format!("bodies may not nest more than {} deep", self.limits.max_depth));
            return None;
        }
        self.in_body += 1;
        let got = self.braced_stmts(next_id);
        self.in_body -= 1;
        let joint = self.open.take();
        let body = got?;
        self.no_open_joint(joint, "a set");
        if body.is_empty() {
            self.fail_at(Span::new(lo, self.prev_hi()), &format!(
                "a set says what its points satisfy: `{{ {} | {} distance(r) c }}`",
                bound.text, bound.text
            ));
            return None;
        }
        Some(SetLit { bound, body, span: Span::new(lo, self.prev_hi()) })
    }

    /// `(ea, eb) in (a, b)` — a block's walks run in step (#243 L8): as many names as walks,
    /// the first of each the block's own walk and the rest its `zip`.
    fn walks(&mut self) -> Option<(Name, Ref, Vec<(Name, Ref)>)> {
        self.i += 1;
        let mut vars = vec![self.ident()?];
        while self.eat_p(',') {
            vars.push(self.ident()?);
        }
        self.i += 2; // `)` and `in`
        if !self.eat_p('(') {
            self.fail("walks run in step are bracketed alike: `(ea, eb) in (a, b)`");
            return None;
        }
        let mut chains = vec![self.refr()?];
        while self.eat_p(',') {
            chains.push(self.refr()?);
        }
        if !self.eat_p(')') {
            self.fail("expected `,` or `)`");
            return None;
        }
        if chains.len() != vars.len() {
            self.fail(&format!(
                "{} names for {} walks: each walk run in step is named once, in order",
                vars.len(),
                chains.len()
            ));
            return None;
        }
        if vars.len() < 2 {
            self.fail("one walk is written without brackets: `repeat e in a`");
            return None;
        }
        let mut zip: Vec<(Name, Ref)> = vars.into_iter().zip(chains).collect();
        let (var, chain) = zip.remove(0);
        Some((var, chain, zip))
    }

    /// `repeat`, `cycle`, `ring` and `when`: a header, then a braced body.
    fn block(&mut self, kind: BlockKind, next_id: &mut u32) -> Option<Block> {
        let lo = self.prev_hi();
        // `repeat e in rack.profile` — the block runs over a chain's edges; a ring is counted,
        // since its turn is a fraction of a whole one
        if kind == BlockKind::Ring && over_chain(&self.t, self.i) {
            self.fail("a `ring` is counted and turned about a centre: `ring N about C { … }`");
            self.skip_block();
            return None;
        }
        // `when c in B` is a condition, not a chain to run over (#267)
        if kind != BlockKind::When && over_chain(&self.t, self.i) {
            let (var, chain, zip) = if over_walks(&self.t, self.i) {
                let Some(walks) = self.walks() else {
                    self.skip_block();
                    return None;
                };
                walks
            } else {
                let var = self.ident()?;
                self.i += 1;
                (var, self.refr()?, Vec::new())
            };
            // a filter on the copies is a `when` in the body (#267): `when b != a { … }`
            if matches!(self.peek(), Some(Tok::Ident(w)) if w == "if") {
                self.fail("which copies make anything is a `when` in the body: \
                           `repeat b in B { when b != a { … } }`");
                self.skip_block();
                return None;
            }
            let binder = if self.eat_word("as") { Some(self.ident()?) } else { None };
            let (body, joint) = self.braced_body(next_id)?;
            return Some(Block {
                kind,
                count: String::new(),
                head: Span::at(lo),
                over: Some(EdgesOf { var, chain, zip }),
                binder,
                body,
                joint,
                about: None,
                index_reads: Vec::new(),
                span: Span::new(lo, self.prev_hi()),
                image: None,
            });
        }
        // the count runs to `as` or `{`, and is an expression over what is in scope
        let from = self.here().lo as usize;
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('(')) => depth += 1,
                Some(Tok::P(')')) => depth -= 1,
                Some(Tok::P('{')) if depth == 0 => break,
                Some(Tok::Ident(w)) if depth == 0 && (w == "as" || w == "about") => break,
                Some(Tok::Nl) => break,
                _ => {}
            }
            self.i += 1;
        }
        let count = self.text_from(from).trim().to_string();
        let head = Span::new(from, self.prev_hi().max(from));
        if kind == BlockKind::When && count.is_empty() {
            self.fail(WHEN);
            self.skip_block();
            return None;
        }
        // a ring names what its copies turn about, and only a ring does
        let about = if self.eat_word("about") {
            let r = self.refr()?;
            if kind != BlockKind::Ring {
                self.fail_at(r.span, "only a `ring` turns about a centre; a `cycle` or a `repeat` names none");
            }
            Some(r)
        } else {
            if kind == BlockKind::Ring {
                self.fail("a `ring` names what its copies turn about: `ring N about C { … }`");
            }
            None
        };
        let binder = if self.eat_word("as") { Some(self.ident()?) } else { None };
        if let (BlockKind::When, Some(b)) = (kind, &binder) {
            self.fail_at(b.span, "a `when` is made once or not at all, so it has no index to name");
        }
        let body_at = self.i;
        let (body, mut joint) = self.braced_body(next_id)?;
        if kind == BlockKind::When {
            self.no_open_joint(joint.take(), "a `when`");
        }
        // a ring's index read in its body: a name token spelled as the binder, not a field
        // (`t.k`) nor a key (`k: …`)
        let mut index_reads = Vec::new();
        if let (BlockKind::Ring, Some(b)) = (kind, &binder) {
            for j in body_at..self.i {
                let Tok::Ident(w) = &self.t[j].0 else { continue };
                let field = j > 0 && self.t[j - 1].0 == Tok::P('.');
                let key = self.t.get(j + 1).is_some_and(|t| t.0 == Tok::P(':'));
                if *w == b.text && !field && !key {
                    index_reads.push(self.t[j].1);
                }
            }
        }
        let span = Span::new(lo, self.prev_hi());
        Some(Block {
            kind, count, head, over: None, binder, body, joint, about, index_reads, span,
            image: None,
        })
    }

    /// Step over a refused block: the rest of its header line and, where one opens there, the
    /// balanced braces after it — however many lines they span.
    pub(super) fn skip_block(&mut self) {
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('{')) => depth += 1,
                Some(Tok::P('}')) => {
                    depth -= 1;
                    if depth <= 0 {
                        self.i += 1;
                        return;
                    }
                }
                Some(Tok::Nl) if depth == 0 => return,
                _ => {}
            }
            self.i += 1;
        }
    }

    /// A braced body, and the open joint its last chain may have ended in — handed back rather
    /// than left in parser state, so every construct with a body is made to answer for a
    /// dangling joint: a block takes it, and anything else refuses it (`no_open_joint`).
    pub(super) fn braced_body(&mut self, next_id: &mut u32) -> Option<(Vec<Stmt>, Option<OpenJoint>)> {
        self.skip_ends();
        if !self.want_p('{') {
            return None;
        }
        if self.in_body as usize >= self.limits.max_depth {
            self.fail(&format!("bodies may not nest more than {} deep", self.limits.max_depth));
            // The opener has already been consumed. Skip this body without recursion.
            let mut depth = 1usize;
            while let Some((token, _)) = self.bump() {
                match token {
                    Tok::P('{') => depth += 1,
                    Tok::P('}') => {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    _ => {}
                }
            }
            return Some((Vec::new(), None));
        }
        self.in_body += 1;
        let got = self.braced_stmts(next_id);
        self.in_body -= 1;
        // taken even where the body failed, or a stale joint would leak into the next body
        let joint = self.open.take();
        got.map(|body| (body, joint))
    }

    fn braced_stmts(&mut self, next_id: &mut u32) -> Option<Vec<Stmt>> {
        let mut body = Vec::new();
        loop {
            self.skip_ends();
            if self.eat_p('}') {
                break;
            }
            if self.done() {
                if !self.exhausted {
                    self.fail("a body with no closing `}`");
                }
                return None;
            }
            if self.chain_or_one(next_id, &mut body).is_none() {
                self.resync();
                if self.done() {
                    return None;
                }
            }
        }
        Some(body)
    }

    /// Refuse the open joint a body just handed up, where the construct around it has no next
    /// copy for the chain to continue onto.
    pub(super) fn no_open_joint(&mut self, joint: Option<OpenJoint>, what: &str) {
        if let Some(j) = joint {
            self.errs.push(SynErr {
                span: j.span,
                message: format!(
                    "a chain ends mid-joint only in a `repeat` or a `cycle`, where it \
                     threads onto the next copy — {what} has none (spec §6.6)"
                ),
            });
        }
    }

    /// One argument of an instantiation: an entity by name, or a number worked out here.
    fn inst_arg(&mut self) -> Option<InstArg> {
        let lo = self.here().lo as usize;
        let label = match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t)) {
            (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                let n = Name { text: s, span: self.here() };
                self.i += 2;
                Some(n)
            }
            _ => None,
        };
        // a group written in place, `cyl: {bore: 16mm}`; refused in a call by the flattener
        if self.peek() == Some(&Tok::P('{')) {
            let fields = self.group_members()?;
            return Some(InstArg { label, value: InstVal::Group(fields), span: Span::new(lo, self.prev_hi()) });
        }
        // `beta: hint(15deg)` — an argument that leaves the formal unbound and seeds its unknown
        if let Some((text, at)) = self.hint_value()? {
            let span = Span::new(lo, self.prev_hi());
            return Some(InstArg { label, value: InstVal::Hint(text, at), span });
        }
        // A whole dotted/indexed path is a reference; a path followed by arithmetic
        // is an expression, e.g. `dims.bore / 2`.
        let checkpoint = self.i;
        let errors = self.errs.len();
        let reference = if matches!(self.peek(), Some(Tok::Ident(_))) {
            self.refr()
        } else { None };
        let bare = reference.is_some()
            && matches!(self.peek(), Some(Tok::P(',')) | Some(Tok::P(')')) | Some(Tok::P('}')));
        let value = if bare {
            InstVal::Ref(reference.unwrap())
        } else {
            self.i = checkpoint;
            self.errs.truncate(errors);
            let from = self.here().lo as usize;
            let stop = |t: &Tok| matches!(t, Tok::P(',' | '}'));
            let end = words::level_end(&self.t, self.i, self.t.len(), Brackets::Expr, stop);
            self.i = end.max(self.i);
            InstVal::Expr(self.text_from(from).trim().to_string())
        };
        Some(InstArg { label, value, span: Span::new(lo, self.prev_hi()) })
    }
}
