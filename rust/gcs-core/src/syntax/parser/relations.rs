//! Constraint arguments and dimension spellings.

use super::P;
use crate::constraints::{builtin_word, call_word, is_operator, Fixity};
use crate::style::Classes;
use crate::syntax::lexer::Tok;
use crate::syntax::words;
use crate::syntax::{seed_arg, Arg, Cmp, Name, OpArg, Ref, Relation, Span, Written};

/// The words a rate is read by (#339 D2): each reads a direction, which a rate is.
/// `equal` reads two rates' speeds (#645), as it reads two lines' lengths.
const RATE_WORDS: [&str; 5] = ["parallel", "perpendicular", "angle", "distance", "equal"];

impl<'a> P<'a> {
    /// Parse an operator relation (§9.1), preserving its written operands and arguments
    /// for later kind resolution.
    pub(super) fn relation(&mut self) -> Option<Relation> {
        let lo = self.here().lo as usize;
        let mut rates = Vec::new();
        let (word, fixity, ops, args) = match self.peek().cloned() {
            // `ccw(a, b, c)`: every operand in the parentheses and none after them — and the
            // word with no `(` after it a name the document defined (#616), `cw.body clear(…) b`
            Some(Tok::Ident(w)) if call_word(&w) && !words::call_named(&self.t, self.i) => {
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text, Fixity::Call)?;
                (word, Fixity::Call, Vec::new(), args)
            }
            Some(Tok::Ident(w))
                if (is_operator(&w) && !call_word(&w) || self.defined_prefix_ahead())
                    && !self.rate_ahead() =>
            {
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text, Fixity::Prefix)?;
                let r = self.operand(0, &mut rates)?;
                (word, Fixity::Prefix, vec![r], args)
            }
            _ => {
                let left = self.operand(0, &mut rates)?;
                let Some(Tok::Ident(w)) = self.peek().cloned() else {
                    self.fail("a statement relates two things with a word between them");
                    return None;
                };
                // a word the language does not have may be one a file defines (§9.9), which
                // elaboration knows and the parser does not; a word the grammar keeps is neither
                if !is_operator(&w) && crate::syntax::reserved_word(&w) {
                    self.fail(&format!("`{w}` is not a constraint"));
                    return None;
                }
                let word = Name { text: w, span: self.here() };
                self.i += 1;
                let args = self.op_args(&word.text, Fixity::Infix)?;
                // **a rate's component along a direction** stands alone (#339 D2):
                // `rate(p over s) distance(1, along: tl)` — the rate is a displacement, and it
                // is measured from nothing
                if !rates.is_empty() && self.statement_ends() {
                    (word, Fixity::Infix, vec![left], args)
                } else {
                    self.connective(&word.text)?;
                    let right = self.operand(1, &mut rates)?;
                    let mut args = args;
                    self.coincident_at(&word.text, &mut args)?;
                    (word, Fixity::Infix, vec![left, right], args)
                }
            }
        };
        // and a rate in the parentheses, `angle(rate(m over u), l)` (#645)
        rates.append(&mut self.arg_rates);
        // a rate is a direction (a tangent, scaled): it is read by the words that read one
        if !rates.is_empty() && !RATE_WORDS.contains(&word.text.as_str()) {
            self.fail(&format!("a rate stands where a direction is read: `rate(x over s) \
                perpendicular y`, `parallel y`, `angle(…) y` or `distance(…, along: u)`, not \
                `{}`", word.text));
            return None;
        }
        let mut r = self.relation_tail(word, fixity, ops, args, lo)?;
        if let Some(w) = r.form.written_mut() {
            w.rates = rates;
        }
        Some(r)
    }

    /// Whether the tokens ahead are `rate(`, an operand read as its rate (#339 D2).
    pub(super) fn rate_ahead(&self) -> bool {
        matches!(self.peek(), Some(Tok::Ident(w)) if w == "rate")
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('(')))
    }

    /// Whether the statement ends here: a line break, the end, or a body's closing brace.
    fn statement_ends(&self) -> bool {
        self.done()
            || self.peek() == Some(&Tok::Nl)
            || (self.in_body > 0 && self.peek() == Some(&Tok::P('}')))
    }

    /// A relation's operand `k`: a reference, or `rate(x over s)` — `x` read as its rate along
    /// the formal `s` (#339 D2), recorded in `rates`.
    fn operand(&mut self, k: usize, rates: &mut Vec<crate::syntax::RateOf>) -> Option<Ref> {
        if !self.rate_ahead() {
            return self.refr();
        }
        let lo = self.here().lo as usize;
        self.i += 2;
        let mut r = self.refr()?;
        if !self.eat_word("over") {
            self.fail("a rate names the formal it runs over: `rate(x over s)`");
            return None;
        }
        let over = self.ident()?;
        if !self.want_p(')') {
            return None;
        }
        r.span = Span::new(lo, self.prev_hi());
        rates.push(crate::syntax::RateOf { op: k, over, slot: false });
        Some(r)
    }

    /// Everything a relation statement may carry after its operands.
    pub(super) fn relation_tail(
        &mut self,
        word: Name,
        fixity: Fixity,
        ops: Vec<Ref>,
        args: Vec<OpArg>,
        lo: usize,
    ) -> Option<Relation> {
        let mut args = args;
        let mut place = None;
        let mut place_span = Span::default();
        let mut class = Classes::default();
        let mut class_span = Span::default();
        loop {
            if self.eat_hint_clause().is_some() {
                // a seed for a slot the constraint owns — the same clause as everywhere else,
                // and read by the same body, so one hint is parsed in one place
                for h in self.hint_body("t: 0.4")? {
                    args.push(h.into());
                }
            } else if class.is_empty() && self.peek_word("class") {
                let (c, sp) = self.class_clause(self.here().lo as usize);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
                class_span = sp;
            } else if place.is_none() && self.peek_word("at") {
                let at = self.here().lo as usize;
                self.i += 1;
                if !self.want_p('(') {
                    return None;
                }
                let t = self.number()?;
                if !self.want_p(',') {
                    return None;
                }
                let r = self.number()?;
                if !self.want_p(')') {
                    return None;
                }
                place = Some((t, r));
                place_span = Span::new(at, self.prev_hi());
            } else {
                break;
            }
        }
        self.end_of_stmt();
        // where a placement would go when none was written — an empty span at the insertion
        // point, `Decl::hint_span`'s device, spliced by `reconcile` when a callout is dragged
        if place.is_none() {
            let at = self.prev_hi();
            place_span = Span::at(at);
        }
        Some(Relation {
            place,
            place_span,
            form: crate::syntax::RelationForm::Written(Written {
                word,
                fixity,
                ops,
                args,
                span: Span::new(lo, self.prev_hi()),
                rates: Vec::new(),
            }),
            claim: false,
            witness: None,
            class,
            class_span,
            word: None,
            along: None,
            space: false,
            initially: false,
            within: Vec::new(),
            name: None,
        })
    }

    /// The word an operator is followed by before its second operand, where it takes one
    /// (`crate::constraints::connective`): `flank regular(…) within blank`.
    pub(super) fn connective(&mut self, word: &str) -> Option<()> {
        let Some(c) = crate::constraints::connective(word) else { return Some(()) };
        if self.peek_word(c) {
            self.i += 1;
            return Some(());
        }
        self.fail(&format!("`{word}` reads its second operand after `{c}`: `a {word}(…) {c} b`"));
        None
    }

    /// Read optional operator arguments. Slot pins stay in parentheses; seeds come
    /// from the separate hint clause.
    pub(super) fn op_args(&mut self, word: &str, fixity: Fixity) -> Option<Vec<OpArg>> {
        // `horizontal (l := line)` — a named link after the word, not the word's own arguments
        if crate::syntax::words::named_link_at(&self.t, self.i) || !self.eat_p('(') {
            return Some(Vec::new());
        }
        // a side takes nothing in its parentheses: one grown by a distance is that distance from
        // the surface (§6.21, #266), and the set grown is `std.sets.Offset`
        if fixity == Fixity::Infix {
            if let Some(cmp) = crate::constraints::plane_side_word(word) {
                let side = if cmp == Cmp::Le { "outside" } else { "inside" };
                self.fail(&format!("`{word}` takes nothing in its parentheses: a point at most \
                    `d` on `S`'s {side} is `p distance(<= d, side: {side}) S`, and `S` grown by \
                    `d` is the set `std.sets.Offset(S, d: d)`"));
                return None;
            }
        }
        // a defined word's parameters, by label, each kept as written (§9.9) — a word the
        // language has in the other fixity included, as the expansion reads it
        if !builtin_word(word, fixity) || crate::constraints::step_word(word) {
            return self.word_args();
        }
        // `symmetry`'s line, `tangent`'s contact point (two cones touching at M) and `level`'s
        // direction stand in the parentheses unlabelled; none states a number, so nothing else
        // is read there
        let takes_entity =
            word == "symmetry" || word == "tangent" || word == "level" || call_word(word);
        // and `mapped`'s pairs, `from -> a`, each a point of the source and where it goes (#357):
        // read as two unlabelled references, the source first
        let pairs = word == "mapped";
        let mut out = Vec::new();
        // where each unlabelled item was read as text: `angle`'s may turn out to be two lines
        let mut texts: Vec<(usize, usize, usize)> = Vec::new();
        // the rates read among them (#645), `angle(rate(m over u), l)`
        let mut rated = 0usize;
        self.arg_rates.clear();
        while !self.eat_p(')') {
            match (self.peek().cloned(), self.t.get(self.i + 1).map(|(t, _)| t.clone())) {
                // an angle equal to one a rate makes (#645): the rate is the first of its lines,
                // operand 2 onward as the parentheses' lines are the relation's third and fourth
                _ if word == "angle" && self.rate_ahead() => {
                    let lo = self.here().lo as usize;
                    self.i += 2;
                    let mut r = self.refr()?;
                    if !self.eat_word("over") {
                        self.fail("a rate names the formal it runs over: `rate(x over s)`");
                        return None;
                    }
                    let over = self.ident()?;
                    if !self.want_p(')') {
                        return None;
                    }
                    r.span = Span::new(lo, self.prev_hi());
                    let op = crate::syntax::ARG_RATES + out.iter().filter(|a| matches!(a, OpArg::Ent(_))).count()
                        + texts.len();
                    self.arg_rates.push(crate::syntax::RateOf { op, over, slot: false });
                    out.push(OpArg::Ent(r));
                    rated += 1;
                }
                _ if self.tuple_ahead() || self.member_end(&Tok::EqEq).is_some() => {
                    out.push(self.pin()?)
                }
                // a bound's interval, `in: (a, b)` (§9.6): its low end the number, its high end
                // the bound's own
                (Some(Tok::Ident(n)), Some(Tok::P(':'))) if n == "in" => {
                    let at = self.here();
                    self.i += 2;
                    let span = Span::new(at.lo as usize, self.prev_hi());
                    let [(a, a_span), hi] = self.pair()?;
                    out.push(OpArg::Bound { cmp: Cmp::In, span, hi: Some(hi), grow: None });
                    out.push(OpArg::Dim(a, a_span));
                }
                // a contact over an interval, `t in (0, 1)` (#645): where on the curve, bounded
                (Some(Tok::Ident(n)), Some(Tok::Ident(w))) if word == "coincident" && w == "in" => {
                    let key = Name { text: n, span: self.here() };
                    self.i += 2;
                    let [lo, hi] = self.pair()?;
                    out.push(OpArg::Within { key, lo, hi });
                }
                // `coincident(at: τ)` is retired (#391 S2): the instant is the operand's
                (Some(Tok::Ident(n)), Some(Tok::P(':'))) if word == "coincident" && n == "at" => {
                    self.fail("`coincident(at: τ)` is retired (#391): a point is read at an \
                        instant where it is named, `X coincident x at τ`, its frames on the `at` \
                        (`x at(under: m, seen_from: g) τ`)");
                    return None;
                }
                (Some(Tok::Ident(n)), Some(Tok::P(':'))) => {
                    let name = Name { text: n, span: self.here() };
                    self.i += 2;
                    if name.text == "relative_to" {
                        self.fail_at(name.span, super::RELATIVE_TO);
                        return None;
                    }
                    // a direction is a word (`along: u`) or what it names (`along: std.x`): a
                    // word of the table is a word, unless it opens a path — and no selector word
                    // is a path, so a path is a reference whatever its key (`at: v1.origin`)
                    let along = name.text == "along" && self.reference_ahead();
                    // and `tangent(at: m)` names a point, unless it names an end of the line or
                    // the arc (`at: p1`, `at: start`): the tangent at a point already on (#251) —
                    // as `perpendicular(at: m)` and `curvature(at: m)` do, at a point already on
                    // an envelope (#283)
                    let point = matches!(word, "tangent" | "perpendicular" | "curvature")
                        && name.text == "at" && matches!(self.peek(),
                        Some(Tok::Ident(w)) if !["start", "end", "p1", "p2"].contains(&w.as_str()));
                    // and `coincident(under: m)` names the motion carrying the point (#326)
                    let motion = word == "coincident"
                        && (name.text == "under" || name.text == "seen_from");
                    // and `angle(θ, about: a)` the axis its turn is read about (#558)
                    let axis = matches!(word, "angle" | "perpendicular") && name.text == "about";
                    let v = if self.path_ahead() || along || point || motion || axis {
                        Arg::Ref(self.refr()?)
                    } else {
                        self.sel_value()?
                    };
                    out.push(OpArg::Named(name, v));
                }
                _ if takes_entity => out.push(OpArg::Ent(self.refr()?)),
                _ if pairs => {
                    out.push(OpArg::Ent(self.refr()?));
                    if self.peek() != Some(&Tok::Arrow) {
                        self.fail("`mapped` takes pairs, a point of the source and where it goes: \
                            `from -> a`");
                        return None;
                    }
                    self.i += 1;
                    out.push(OpArg::Ent(self.refr()?));
                }
                // a bound, `>= d` or `<= d` (§9.6)
                (Some(Tok::P('>' | '<')), _) => {
                    let at = self.here();
                    let cmp = self.closed_cmp()?;
                    let span = Span::new(at.lo as usize, self.prev_hi());
                    out.push(OpArg::Bound { cmp, span, hi: None, grow: None });
                    let (text, span) = self.raw_number()?;
                    out.push(OpArg::Dim(text, span));
                }
                _ => {
                    let first = self.i;
                    let (text, span) = self.raw_number()?;
                    texts.push((out.len(), first, self.i));
                    out.push(OpArg::Dim(text, span));
                }
            }
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        // **an angle stated as another angle**: `l1 angle(l3, l4) l2`.  A kind states at most one
        // number, so two unlabelled items in `angle`'s parentheses cannot be its number — they
        // are the pair of lines whose angle this one equals, read again as the references they
        // are.  Anything else in two is a mistake, and said so here rather than one of the two
        // being dropped where the arguments are assembled
        if word == "angle" && texts.len() + rated == 2 && !texts.is_empty() {
            let end = self.i;
            if !texts.iter().all(|&(_, from, to)| self.past_ref(from) == Some(to)) {
                self.i = texts[0].1;
                self.fail("`angle` takes one number, or the two lines of the angle it equals");
                return None;
            }
            for &(at, from, _) in &texts {
                self.i = from;
                out[at] = OpArg::Ent(self.refr()?);
            }
            self.i = end;
        }
        Some(out)
    }

    /// `X coincident x at τ` (#391 S2), the right operand just read: `x` read at an instant of
    /// the roll, its arguments joining the relation's, the frames it is read by on the `at` and
    /// never on the `coincident` beside it.  Nothing is read where `word` is another or no
    /// instant follows.
    pub(super) fn coincident_at(&mut self, word: &str, args: &mut Vec<OpArg>) -> Option<()> {
        if word != "coincident" || !self.instant_ahead() {
            return Some(());
        }
        let frames = |a: &OpArg| matches!(a, OpArg::Named(n, _)
            if n.text == "under" || n.text == "seen_from");
        if args.iter().any(frames) {
            self.fail("a point read at an instant names its frames on the `at`: `X coincident x \
                       at(under: m, seen_from: g) τ`");
            return None;
        }
        args.extend(self.instant()?);
        Some(())
    }

    /// Whether `at` here begins an instant (#391 S2) — `at τ` or `at(under: m) τ` — rather than
    /// a callout's placement, `at (t, r)`: two numbers in the brackets and nothing after.
    pub(super) fn instant_ahead(&self) -> bool {
        if !self.peek_word("at") {
            return false;
        }
        let tok = |k: usize| self.t.get(self.i + k).map(|(t, _)| t);
        if tok(1) != Some(&Tok::P('(')) {
            return !matches!(tok(1), None | Some(Tok::Nl));
        }
        // `(`, then a signed number, `,`, a signed number, `)`: a placement
        let mut k = 2;
        for close in [',', ')'] {
            if matches!(tok(k), Some(Tok::P('-' | '+'))) {
                k += 1;
            }
            if !matches!(tok(k), Some(Tok::Num(_))) || tok(k + 1) != Some(&Tok::P(close)) {
                return true;
            }
            k += 2;
        }
        false
    }

    /// **An instant** (#391 S2), `at` read past: `at τ`, or `at(under: m, seen_from: g) τ`
    /// naming the frame the reading is under where what it reads rides on none, and the frame it
    /// is seen from — as a carried point's arguments are held: the frames by their labels, the
    /// roll the relation's number, written as a dimension is and read to the end of the
    /// statement (or its `hint(…)`, `class` or placement).
    pub(super) fn instant(&mut self) -> Option<Vec<OpArg>> {
        self.i += 1; // `at`
        let mut out = Vec::new();
        let labelled = self.peek() == Some(&Tok::P('('))
            && matches!(self.t.get(self.i + 2).map(|(t, _)| t), Some(Tok::P(':')));
        if labelled {
            self.i += 1;
            while !self.eat_p(')') {
                let Some(Tok::Ident(n)) = self.peek().cloned() else {
                    self.fail("an instant names `under:` the frame it is read by and \
                        `seen_from:` the one it is seen from");
                    return None;
                };
                let name = Name { text: n, span: self.here() };
                if !matches!(name.text.as_str(), "under" | "seen_from") {
                    self.fail(&format!("an instant names `under:` the frame it is read by and \
                        `seen_from:` the one it is seen from, not `{}:`", name.text));
                    return None;
                }
                if out.iter().any(|a| matches!(a, OpArg::Named(m, _) if m.text == name.text)) {
                    self.fail(&format!("`{}` is given twice", name.text));
                    return None;
                }
                self.i += 2;
                out.push(OpArg::Named(name, Arg::Ref(self.refr()?)));
                if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                    self.fail("expected `,` or `)`");
                    return None;
                }
            }
        }
        // a placement after it is the relation's, `at (t, r)`
        let clause =
            |t: &Tok| matches!(t, Tok::Ident(w) if w == "hint" || w == "class" || w == "at");
        let (text, span) = self.raw_until(clause, "an instant states the roll it is read at: \
            `at τ`")?;
        out.push(OpArg::Dim(text, span));
        Some(out)
    }

    /// `<=` or `>=` here, read past: a bound is closed, since a solution may stand on its edge, so
    /// `<` and `>` alone are refused.
    pub(super) fn closed_cmp(&mut self) -> Option<Cmp> {
        let Some(Tok::P(c @ ('<' | '>'))) = self.peek().cloned() else { return None };
        if self.t.get(self.i + 1).map(|(t, _)| t) != Some(&Tok::P('=')) {
            self.fail(&format!("a bound is closed, `{c}=`: a solution may stand on its edge"));
            return None;
        }
        self.i += 2;
        Some(if c == '<' { Cmp::Le } else { Cmp::Ge })
    }

    /// The number an operator states, as written, up to the `,` or `)` that ends it.
    pub(super) fn raw_number(&mut self) -> Option<(String, Span)> {
        self.raw_until(|t| *t == Tok::P(','), "expected the number this states")
    }

    /// A pin: `t == 0.4`, one member of a vector, `dir.x == 1`, a vector the entity has,
    /// `dir == (1, 0, 0)`, or the entity itself, `fix((0, 0)) p`.
    fn pin(&mut self) -> Option<OpArg> {
        let lo = self.here();
        let pinned = |parts: Vec<(String, (Option<f64>, String, Span))>| -> Vec<Arg> {
            parts.into_iter().map(|(_, (v, text, span))| seed_arg(v, text, span, true)).collect()
        };
        if self.tuple_ahead() {
            let parts = pinned(self.vector("")?);
            return Some(OpArg::Vector { key: None, parts, span: Span::new(lo.lo as usize, self.prev_hi()) });
        }
        let name = Name { text: self.member_label(Tok::EqEq)?, span: lo };
        if self.tuple_ahead() {
            let parts = pinned(self.vector(&name.text)?);
            let span = Span::new(lo.lo as usize, self.prev_hi());
            return Some(OpArg::Vector { key: Some(name), parts, span });
        }
        // the same slot the `hint(…)` clause seeds, *pinned*.  The whole of the value is kept,
        // expression and all: written inside a component a pin reads the parameters in scope
        // (`t == t0`), which `flatten` settles later, and taking only `value` here would pin
        // every one of them at 0.
        let (value, text, span) = self.value_text()?;
        // a joint's value (#391 S3) is an instant worked out as a dimension is, so it may be
        // written over a face family's running number and read again at every station
        if name.text == crate::constraints::JOINT_VALUE {
            return Some(OpArg::Slot { key: name, arg: Arg::Dim { text, span } });
        }
        Some(OpArg::Slot { key: name, arg: seed_arg(value, text, span, true) })
    }

    /// A selector's value: `side: -1`, `at: start`, `external: true`, `along: u`.
    /// Whether what follows `along:` is a reference rather than one of the direction words: a
    /// name outside `constraints::ALONG`, or any name a path goes on from.
    fn reference_ahead(&self) -> bool {
        let Some(Tok::Ident(n)) = self.peek() else { return false };
        self.path_ahead() || crate::constraints::Toward::of(n).is_none()
    }

    /// A name that opens a path: `v1.origin`, `s[0]`.
    fn path_ahead(&self) -> bool {
        matches!(self.peek(), Some(Tok::Ident(_)))
            && matches!(self.t.get(self.i + 1).map(|(t, _)| t), Some(Tok::P('.' | '[')))
    }

    fn sel_value(&mut self) -> Option<Arg> {
        match self.peek().cloned() {
            Some(Tok::Num(_)) | Some(Tok::P('-')) | Some(Tok::P('+')) => {
                Some(Arg::Num(self.number()?))
            }
            Some(Tok::Ident(w)) if w == "true" || w == "false" => {
                self.i += 1;
                Some(Arg::Bool(w == "true"))
            }
            Some(Tok::Ident(w)) => {
                self.i += 1;
                Some(Arg::Word(w))
            }
            _ => {
                self.fail("expected a value");
                None
            }
        }
    }

    /// Read the dimension after `==` through the logical line end, preserving its text
    /// and span for expression evaluation and edits.
    pub(super) fn raw_dimension(
        &mut self,
        from: usize,
    ) -> (String, Span, Option<((f64, f64), Span)>, usize) {
        let bytes = self.src.as_bytes();
        let mut end = from;
        while end < bytes.len() && bytes[end] != b'\n' && bytes[end] != b';' {
            if bytes[end] == b'/' && matches!(bytes.get(end + 1), Some(b'/') | Some(b'*')) {
                break;
            }
            end += 1;
        }
        let mut text = &self.src[from..end];
        // a trailing `hint(…)` ends the dimension and starts the statement's trailing clauses;
        // stopping *here* hands those tokens back to the loop that reads them, rather than
        // teaching this function a second grammar.  (No kind carries both a dimension and a
        // `Param` slot today, so this is a guard rather than a path anything walks.)
        if let Some(h) = text.find(" hint(") {
            end = from + h;
            text = &text[..h];
        }
        let mut place = None;
        if let Some(at) = text.rfind(" at (") {
            let tail = &text[at + 5..];
            if let Some(close) = tail.find(')') {
                if tail[close + 1..].trim().is_empty() {
                    let nums: Vec<f64> = tail[..close]
                        .split(',')
                        .filter_map(|s| s.trim().parse::<f64>().ok())
                        .collect();
                    if nums.len() == 2 {
                        // the span of `at (...)` itself, leading space excluded: a replacement
                        // keeps the space, and a removal takes it with `back_over_spaces`
                        let lo = from + at + 1;
                        let hi = from + at + 5 + close + 1;
                        place = Some(((nums[0], nums[1]), Span::new(lo, hi)));
                        text = &text[..at];
                    }
                }
            }
        }
        // the span of the *trimmed* text, not of the slice it was cut from: an edit splices
        // this, and a span that included the space after `==` would eat it and write `==140`
        let lead = text.len() - text.trim_start().len();
        let trimmed = text.trim();
        let span = Span::new(from + lead, from + lead + trimmed.len());
        (trimmed.to_string(), span, place, end)
    }
}
