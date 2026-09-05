//! Statements, component instances, and braced bodies.

use super::P;
use crate::style::Classes;
use crate::syntax::lexer::Tok;
use crate::syntax::words::{names_decl, BLOCKS};
use crate::syntax::{
    Arg, Block, BlockKind, BodyWord, Branch, Chained, ClaimOver, Component, CurveSpec, CurveTarget,
    DeclName, DerivedDecl, Formal, InBlock, InstArg, InstVal, Instance, Membership, Name,
    OpenJoint, ParamDecl, Ref, SolidRel, Source, Span, Stmt, StmtKind, SynErr, Ty, Use,
};

/// Apply block membership recursively. Faces and solids inherit their geometry's plane;
/// instances carry membership into expansion. Conflicting clauses are diagnosed here.
fn stamp_plane(stmts: &mut [Stmt], plane: &Ref, errs: &mut Vec<SynErr>) {
    for st in stmts {
        match &mut st.kind {
            StmtKind::Decl(d) => {
                // Faces and solids use the planes of their constituent geometry.
                if d.kind.spatial() {
                } else if !d.kind.bears_points() {
                    errs.push(SynErr {
                        span: st.span,
                        message: format!(
                            "`in` puts points on a plane, and a {} has none of its own",
                            d.kind.as_str()
                        ),
                    });
                } else if !d.membership.join(plane, Source::Block) {
                    errs.push(SynErr {
                        span: d.membership.span(),
                        message: d.membership.cause().into(),
                    });
                }
            }
            StmtKind::Block(b) => stamp_plane(&mut b.body, plane, errs),
            // the instance joins whole: the flattener carries the plane into its expansion
            StmtKind::Instance(inst) => {
                if !inst.membership.join(plane, Source::Block) {
                    errs.push(SynErr {
                        span: inst.membership.span(),
                        message: inst.membership.cause().into(),
                    });
                }
            }
            _ => {}
        }
    }
}

impl<'a> P<'a> {
    fn stmt(&mut self, next_id: &mut u32) -> Option<StmtKind> {
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
            // **`ccw` and `cw` keep a call.**  Every other statement is a prefix or an infix
            // operator, and under that rule these would be `a ccw(c) b` — which reorders three
            // points that are symmetric, since the predicate is about the *triangle* and not
            // about a pair with a decoration.  Spec §9.6 keeps the call for exactly that reason.
            "branch" => {
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
            "ellipse" => {
                // a library component now (bmander/geomsolver#47, item 4): a computed point on
                // a datum, traced as a curve, whose contacts are the curve's
                self.fail(
                    "`ellipse` is a library component now: `use std`, then \
                     `curve e = Ellipse(f, a: …, b: …).p over u in (0, 360)` over a datum \
                     `plane f(origin: c, toward: m)` — `p on e`, `e tangent l` and \
                     `e curvature k` are the curve's contacts",
                );
                None
            }
            "frame" => {
                // folded into `plane` (bmander/geomsolver#47, item 6): a plane with no attitude
                // written is the datum a frame was, on the page
                self.fail(
                    "`frame` is folded into `plane`: write `plane f(origin: o, toward: q)` — a \
                     datum with no attitude written is a view of the page, and a formal \
                     `f: plane` offers `f.angle` as `frame` did",
                );
                None
            }
            "port" => {
                // retired (bmander/geomsolver#47): everything an instance makes is reached by
                // its dotted name, so a port was a second name for a thing that had one
                self.fail(
                    "`port` is retired: an instance's entities are reached by dotted name \
                     (`inst.p`), so declare the point (`point p hint(…)`), compute it \
                     (`point p = (x, y)`) or name the aliased entity itself",
                );
                None
            }
            "param" => {
                self.i += 1;
                let name = self.ident()?;
                if self.peek() != Some(&Tok::Eq) {
                    self.fail("a param is `param name = expression`");
                    return None;
                }
                let after = self.here().hi as usize;
                self.i += 1;
                let (text, span, _, end) = self.raw_dimension(after);
                while self.i < self.t.len() && (self.t[self.i].1.lo as usize) < end {
                    self.i += 1;
                }
                Some(StmtKind::Param(ParamDecl { name, text, span }))
            }
            b if BLOCKS.contains(&b) => {
                self.i += 1;
                let kind = match w.as_str() {
                    "repeat" => BlockKind::Repeat,
                    _ => BlockKind::Cycle,
                };
                self.block(kind, next_id).map(StmtKind::Block)
            }
            "ring" => {
                // Not a construct of this implementation (bmander/geomsolver#47, item 3): a
                // ring it could only unroll into the cycle it stood over, and then say so on
                // every run.  The word comes back with the fundamental-domain solve of spec
                // §12.4.  Reported once, the body consumed, so its statements are not read as
                // loose lines and one mistake told N ways.
                self.fail(
                    "`ring` is not yet a construct of this implementation: write `cycle N { … }`, \
                     whose copies are congruent by the numbers each is given (spec §12.3 is \
                     what `ring` will add)",
                );
                self.skip_block();
                None
            }
            // **`claim over crank.theta in (0deg, 360deg) { … }`** (§9.8): every claim in the
            // body judged as the drawing runs along one of its own free variables, and the worst
            // pose reported.  Every claim the language had until now is judged at one pose, and
            // a fact about a *cycle* — a disc clearing a cylinder mouth, a port's timing — is
            // not one of those.
            "claim" if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(w)) if w == "over") =>
            {
                let lo = self.here().lo as usize;
                self.i += 2;
                let formal = self.refr()?;
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
                    from: Arg::Dim { text: from, span: fs },
                    to: Arg::Dim { text: to, span: ts },
                    body,
                    span: Span::new(lo, self.prev_hi()),
                }))
            }
            // `claim vertical(rail)` — a relation stated as expected to add no rank.  The colon
            // guard keeps an instance *named* claim (`claim: Tooth(…)`) an instance.
            "claim" if !matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P(':'))) => {
                self.i += 1;
                let mut r = self.relation()?;
                r.claim = true;
                Some(StmtKind::Relation(r))
            }
            // `view(cyl.body) in views.right` — a picture asked of a solid (§6.11).  The
            // brackets are what it is a picture of, `in` is the trailer it already is
            "view" | "section" | "dimensions" => {
                let lo = self.here().lo as usize;
                let sect = w == "section";
                let dims = w == "dimensions";
                self.i += 1;
                let name = match self.peek() {
                    Some(Tok::Ident(n)) if names_decl(n) => DeclName::Written(self.ident()?),
                    _ => {
                        let at = self.prev_hi();
                        DeclName::Key(Name { text: format!("#a{at}"), span: Span::new(at, at) })
                    }
                };
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
            // `bore through cyl` — the body rule's own word, and the one statement whose shape
            // is two names with a word between them that is not a constraint.  Read by a
            // lookahead rather than by `relation()`, since `through` relates no geometry and
            // has no residual to be settled into
            // **past the dotted path, not past one token**: a mate names a *face* of a solid
            // (`cyl.block.far against plate.body.near`), so the word after the left operand is
            // three tokens away and not one — the lookahead `past_ref` exists for
            _ if self.past_ref(self.i).and_then(|j| self.t.get(j)).is_some_and(
                |(t, _)| matches!(t, Tok::Ident(w) if w == "through" || w == "against"),
            ) =>
            {
                let lo = self.here().lo as usize;
                let what = self.refr()?;
                let Some(Tok::Ident(w)) = self.peek().cloned() else {
                    self.fail("a body statement is `X through B` or `F against G`");
                    return None;
                };
                let word = if w == "through" { BodyWord::Through } else { BodyWord::Against };
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
            // `t: Tooth(...)` — a name, a colon and a component
            _ if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P(':'))) => {
                let name = self.ident()?;
                self.i += 1; // the colon
                let component = self.ident()?;
                let lo = name.span.lo as usize;
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
                Some(StmtKind::Instance(Instance {
                    name,
                    component,
                    args,
                    span: Span::new(lo, self.prev_hi()),
                    membership,
                    class,
                }))
            }
            _ => self.relation().map(StmtKind::Relation),
        }
    }

    /// Parse a statement or desugar a chain, recovering at the next terminator on failure.
    pub(super) fn chain_or_one(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        self.declined = None;
        // `in top { … }` before anything else: `in` opens no chain and no statement of any
        // other kind — the one look this could be confused with is an instance named `in`,
        // which `is_name` already refuses, and the colon after it tells even that apart
        if self.peek_word("in")
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::Ident(_)))
        {
            if !self.take_statement(self.here()) {
                return None;
            }
            return self.in_block(next_id, out);
        }
        let before = self.errs.len();
        let got = if self.named_chain_starts() || self.chain_starts() {
            self.chain(next_id, out)
        } else {
            let lo = self.here().lo as usize;
            let kind = self.stmt(next_id)?;
            let id = self.mint_stmt(next_id, Span::new(lo, self.prev_hi()))?;
            out.push(Stmt { id, kind, span: Span::new(lo, self.prev_hi()), chained: Chained::No });
            Some(())
        };
        // The name in a declaration is optional, and the words that may follow one are reserved
        // (§6.1) — so a statement that *named* a declaration with one of them no longer parses,
        // and what went wrong is a reservation the errors above cannot see.  Said only beside a
        // failure: `line tangent arc` is a chain, and needs no remark.
        if self.errs.len() > before {
            if let Some((w, span)) = self.declined.take() {
                self.errs.push(SynErr {
                    span,
                    message: format!(
                        "note: `{w}` cannot be a declaration's name — the words that may follow \
                         a declaration are reserved (spec §6.1)"
                    ),
                });
            }
        }
        got
    }

    /// Hoist an `in PLANE` body and stamp its declarations with inherited membership.
    /// Keep the header and brace spans so deleting the plane can remove the wrapper.
    fn in_block(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let lo = self.here().lo as usize;
        self.i += 1; // `in`
        let plane = self.refr()?;
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
        let nested = self.in_body > 0;
        if nested && !self.in_comp {
            self.fail(
                "an `in` block stands at the top level or in a component; inside a block here, \
                 write the clause on each declaration",
            );
        }
        let (mut body, joint) = self.braced_body(next_id)?;
        self.no_open_joint(joint, "an `in` block");
        let close = self.t.get(self.i.wrapping_sub(1)).map(|(_, s)| *s).unwrap_or_default();
        stamp_plane(&mut body, &plane, &mut self.errs);
        // recorded only where a deletion could reach it: a nested block is already an error
        if !nested {
            self.in_blocks.push(InBlock { plane, header, close });
        }
        out.extend(body);
        Some(())
    }

    /// `over (a, b)` — two expressions over whatever parameters are in scope.
    /// What follows a curve's `=`: `REF over IDENT in (a, b)`, or
    /// `Component(args).path over IDENT in (a, b)`.  A component name is told from a point's by
    /// the `(` after it, the same token that tells an instance from a reference elsewhere.
    pub(super) fn curve_spec(&mut self) -> Option<CurveSpec> {
        let target = if matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('('))) {
            let component = self.ident()?;
            let lo = component.span.lo as usize;
            let args = self.inst_args()?;
            if !self.eat_p('.') {
                self.fail("the point the curve traces follows the instance: `Component(…).point`");
                return None;
            }
            let point = self.refr()?;
            let at = lo;
            let inst = Instance {
                // a key the source cannot write, as an anonymous declaration's is
                name: Name { text: format!("#c{at}"), span: Span::new(at, at) },
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
        if self.peek_word("over")
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('(')))
        {
            self.fail(
                "`over` names the formal that runs, and `in` the interval: \
                 `over theta in (0, 360)`",
            );
            return None;
        }
        if !self.eat_word("over") {
            self.fail("a curve says which formal runs: `over theta in (a, b)`");
            return None;
        }
        let swept = self.ident()?;
        if !self.eat_word("in") {
            self.fail("a curve says the interval its formal runs over: `in (a, b)`");
            return None;
        }
        let [(a, _), (b, _)] = self.pair()?;
        Some(CurveSpec { target, swept, domain: (a, b), of: None })
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
        if !self.want_p('(') {
            return None;
        }
        let mut args = Vec::new();
        while !self.eat_p(')') {
            args.push(self.inst_arg()?);
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
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
        self.end_of_stmt();
        Some(Use { name, span: Span::new(lo, self.prev_hi()) })
    }

    pub(super) fn component(&mut self, next_id: &mut u32) -> Option<Component> {
        let lo = self.here().lo as usize;
        self.i += 1; // `component`
        let name = self.ident()?;
        let mut formals = Vec::new();
        if self.eat_p('(') {
            while !self.eat_p(')') {
                let fname = self.ident()?;
                if !self.want_p(':') {
                    return None;
                }
                let tname = self.ident()?;
                let Some(ty) = Ty::parse(&tname.text) else {
                    self.errs.push(SynErr {
                        span: tname.span,
                        message: if tname.text.eq_ignore_ascii_case("frame") {
                            "`frame` is folded into `plane`: a formal `f: plane` is the datum, \
                             and offers `f.angle`"
                                .to_string()
                        } else {
                            format!("`{}` is not a type", tname.text)
                        },
                    });
                    return None;
                };
                let span = Span::new(fname.span.lo as usize, self.prev_hi());
                formals.push(Formal { name: fname, ty, span });
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
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
        })
    }

    fn block(&mut self, kind: BlockKind, next_id: &mut u32) -> Option<Block> {
        let lo = self.prev_hi();
        // the count runs to `as` or `{`, and is an expression over what is in scope
        let from = self.here().lo as usize;
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('(')) => depth += 1,
                Some(Tok::P(')')) => depth -= 1,
                Some(Tok::P('{')) if depth == 0 => break,
                Some(Tok::Ident(w)) if depth == 0 && w == "as" => break,
                Some(Tok::Nl) => break,
                _ => {}
            }
            self.i += 1;
        }
        let count = self.text_from(from).trim().to_string();
        let binder = if self.eat_word("as") { Some(self.ident()?) } else { None };
        let (body, joint) = self.braced_body(next_id)?;
        Some(Block { kind, count, binder, body, joint, span: Span::new(lo, self.prev_hi()) })
    }

    /// Step over a refused block: the rest of its header line and, where one opens there, the
    /// balanced braces after it — however many lines they span.
    fn skip_block(&mut self) {
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
    fn braced_body(&mut self, next_id: &mut u32) -> Option<(Vec<Stmt>, Option<OpenJoint>)> {
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
    fn no_open_joint(&mut self, joint: Option<OpenJoint>, what: &str) {
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
        // a bare name is an entity; anything with an operator in it is a value expression, and
        // the two are told apart by what follows the first token rather than by a type
        let bare = matches!(self.peek(), Some(Tok::Ident(_)))
            && matches!(
                self.t.get(self.i + 1).map(|(t, _)| t),
                Some(Tok::P(',')) | Some(Tok::P(')')) | Some(Tok::P('.')) | Some(Tok::P('['))
            );
        let value = if bare {
            InstVal::Ref(self.refr()?)
        } else {
            let from = self.here().lo as usize;
            let mut depth = 0i32;
            while !self.done() {
                match self.peek() {
                    Some(Tok::P('(')) | Some(Tok::P('[')) => depth += 1,
                    Some(Tok::P(')')) | Some(Tok::P(']')) if depth == 0 => break,
                    Some(Tok::P(')')) | Some(Tok::P(']')) => depth -= 1,
                    Some(Tok::P(',')) if depth == 0 => break,
                    Some(Tok::Nl) => break,
                    _ => {}
                }
                self.i += 1;
            }
            InstVal::Expr(self.text_from(from).trim().to_string())
        };
        Some(InstArg { label, value, span: Span::new(lo, self.prev_hi()) })
    }
}
