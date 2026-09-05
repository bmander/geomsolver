//! Chain parsing, boundary threading, and relation desugaring.

use super::P;
use crate::constraints::{is_operator, Fixity};
use crate::model::{EntKind, Field};
use crate::style::Classes;
use crate::syntax::lexer::Tok;
use crate::syntax::names::refs_eq;
use crate::syntax::words::{joint_word, opens_link, past_args, prefix_word, word_at, OPENERS};
use crate::syntax::{
    build_rank, ref_text, under_root, Arg, Chained, Decl, Fall, Kid, Name, OpArg, OpenJoint,
    OpenNamed, OpenSide, Ref, Relation, Seg, Span, Stmt, StmtKind, SynErr, Written,
};

/// A declaration exposes child slots for threading; a reference leaves its declaration
/// untouched. Only `->` requests a shared boundary point.
enum LinkBody {
    /// `line bottom(b1, b2)` — the chain declares it, so the keyword says what kind it is.
    /// Boxed because a `Decl` is many times a `Ref`, and a chain holds a `Vec` of these.
    Decl(Box<Decl>),
    /// `a_br` — the chain names one declared elsewhere.  What kind it is, only elaboration
    /// knows: a name may be declared further down the file, or come from a component.
    Ref(Ref),
}

/// A joint between two links: whether the `->` marker threads it, the words standing at it (each
/// with whatever stood in its parentheses), and where its text runs — from the marker (or the
/// first word, where there is no marker) through the last word's arguments, and through `close`
/// for the joint that seals a loop.  At least one of marker and word is present; the grammar has
/// no empty joint.
struct Joint {
    thread: bool,
    /// The relations stated at this joint — `-> equal angle(30deg)` states both at the corner
    /// just threaded.  Each word carries its own parentheses and its own span, so each desugars
    /// to a statement of its own.
    words: Vec<(String, Vec<OpArg>, Span)>,
    span: Span,
    /// The joint's own statements are skipped where its structure was refused — a threaded
    /// circle, a `close` over one link — so one mistake is one message.
    sound: bool,
}

/// One link of a chain while it is being read: the unary constraint words standing before it,
/// what it stands on, and where its text sits.
struct Link {
    /// The words standing before it, each with its own parentheses: `horizontal`, `radius(25)`.
    prefixes: Vec<(Name, Vec<OpArg>)>,
    body: LinkBody,
    /// Where the link's text runs, which is not the declaration's: it starts at the element
    /// keyword rather than at the name.
    span: Span,
}

impl Link {
    /// The entity this link is about, as a relation written over it would name it — the
    /// resolution key, since a corner welds by name and an anonymous link has nothing else.
    fn entity(&self) -> Ref {
        match &self.body {
            LinkBody::Decl(d) => {
                Ref { root: d.name.key().clone(), path: Vec::new(), span: d.name.span() }
            }
            LinkBody::Ref(r) => r.clone(),
        }
    }

    /// What kind of entity it is, where that is known *here* — a declaration says so with its
    /// keyword, and a name does not say until elaboration resolves it.
    fn kind(&self) -> Option<EntKind> {
        match &self.body {
            LinkBody::Decl(d) => Some(d.kind),
            LinkBody::Ref(_) => None,
        }
    }

    /// Where to point a complaint about this link.  An **anonymous** declaration's name span is
    /// empty — it marks where a name *would* go — and a caret with nothing under it says
    /// nothing, so the link's own text stands in: the keyword a reader can see.
    fn span_of_name(&self) -> Span {
        match &self.body {
            LinkBody::Decl(d) => d.name.shown().map(|n| n.span).unwrap_or(self.span),
            LinkBody::Ref(r) => r.span,
        }
    }
}

/// One boundary link of a body's trailing open joint, read while the chain's links are still
/// in hand — what `open_finish` turns into an `OpenSide` once desugaring has given the link's
/// declaration its statement id.
struct OpenEnd {
    kind: EntKind,
    slot: usize,
    declared: bool,
    boundary: Ref,
    entity: Ref,
}
/// What the field at a boundary slot is called, for a message about it.
fn boundary_name(k: EntKind, slot: usize) -> &'static str {
    k.fields()
        .iter()
        .filter(|(_, f)| *f != Field::Scalar)
        .nth(slot)
        .map(|(n, _)| *n)
        .unwrap_or("end")
}

/// How one of a joint's words records its spelling: a word with siblings spans itself alone
/// and is a `Member` of the one written joint, and an only word carries the joint's whole
/// span and its one-word doom.  Three joints state words — mid-chain, `close` and a block
/// body's open end — and this is the one protocol they share.
fn word_spelling(n: usize, wspan: Span, of: Span, fall: Fall) -> (Span, Chained) {
    if n > 1 {
        (wspan, Chained::Member { of, fall, out_of: n as u32 })
    } else {
        (of, fall.into())
    }
}

/// The reference standing in one of a link's child slots, where the link declares one.  A link
/// that only *names* an element has no list to read, and a slot seeded with `hint(…)` names
/// nothing, so both read as unfilled — the rule `thread` and an open joint's ends ask alike,
/// stated once.
fn slot_ref(link: &Link, slot: usize) -> Option<Ref> {
    match &link.body {
        LinkBody::Decl(d) => {
            d.children.get(slot).and_then(|v| v.first()).and_then(|kid| kid.as_ref()).cloned()
        }
        LinkBody::Ref(_) => None,
    }
}

/// The dotted name a mint gives a boundary child — the entity's own name and the field, since
/// the dotted path *is* the anonymous child's name (§6.2).
fn boundary_ref(root: Name, kind: EntKind, slot: usize) -> Ref {
    Ref {
        root,
        path: vec![Seg::Field(Name::new(boundary_name(kind, slot)))],
        span: Span::default(),
    }
}
impl<'a> P<'a> {
    /// Whether what stands here opens a declaration — possibly a chain of them.
    pub(super) fn chain_starts(&self) -> bool {
        let Some(Tok::Ident(w)) = self.peek() else { return false };
        let next = word_at(&self.t, past_args(&self.t, self.i));
        // `a_br equal a_tr` — a name, then a word that relates it to another.  Nothing else in
        // the language has that shape: a statement opening with a bare name is an instance, and
        // that is a name followed by a colon.  `claim parallel(…)` has the shape too — a binary
        // relation's name doubles as an infix joint word — but `claim` qualifies a statement,
        // it never names an element.
        // a word that *opens* a statement is not an operand, however the next word reads:
        // `param radius = 50` is a definition and not `param` related to `radius`
        if !OPENERS.contains(&w.as_str())
            && EntKind::parse(w).is_none()
            && !prefix_word(w)
            && !is_operator(w)
        {
            // the operand may be a dotted name — `l.p1 distance(6) l.p2` — so the word that
            // relates it is looked for past the whole reference, not at the next token.  The
            // retired `to` is still recognised here, so `a to k` reaches the chain loop and its
            // migration message rather than a generic refusal.
            if let Some(j) = self.past_ref(self.i) {
                if matches!(self.t.get(j).map(|(t, _)| t), Some(Tok::Arrow))
                    || self.word_at(j).is_some_and(|w| joint_word(w) || w == "to")
                {
                    return true;
                }
            }
        }
        opens_link(w, next)
    }

    /// The token index just past a reference beginning at `j` — a name, then any run of
    /// `.field` and `[index]` — or `None` where no reference begins there.
    pub(super) fn past_ref(&self, mut j: usize) -> Option<usize> {
        if !matches!(self.t.get(j).map(|(t, _)| t), Some(Tok::Ident(_))) {
            return None;
        }
        j += 1;
        loop {
            match self.t.get(j).map(|(t, _)| t) {
                Some(Tok::P('.')) => j += 2,
                Some(Tok::P('[')) => {
                    let mut d = 0i32;
                    loop {
                        match self.t.get(j).map(|(t, _)| t) {
                            Some(Tok::P('[')) => d += 1,
                            Some(Tok::P(']')) => {
                                d -= 1;
                                if d == 0 {
                                    j += 1;
                                    break;
                                }
                            }
                            Some(Tok::Nl) | None => return Some(j),
                            _ => {}
                        }
                        j += 1;
                    }
                }
                _ => return Some(j),
            }
        }
    }

    /// The identifier at a token position, where there is one — what `opens_link` reads ahead.
    pub(super) fn word_at(&self, i: usize) -> Option<&str> {
        word_at(&self.t, i)
    }

    /// `[prefix…] decl (joint [prefix…] decl)* [joint "close"]`.
    pub(super) fn chain(&mut self, next_id: &mut u32, out: &mut Vec<Stmt>) -> Option<()> {
        let lo = self.here().lo as usize;
        let mut links = vec![self.link()?];
        let mut joints: Vec<Joint> = Vec::new();
        let mut close: Option<Joint> = None;
        let mut open: Option<Joint> = None;
        loop {
            let start = self.here().lo as usize;
            // `->` says the two links beside it share a boundary point; a word beside it says
            // what else holds at the corner just threaded.  At least one of the two makes a
            // joint, and neither alone implies the other (spec §6.6).
            let mut thread = self.peek() == Some(&Tok::Arrow);
            // where the joint's own text ends — the last marker or word taken, so a doomed
            // joint's splice does not eat a line break the words stepped over
            let mut hi = self.prev_hi();
            if thread {
                self.i += 1;
                hi = self.prev_hi();
                // a line ending in `->` continues the chain on the next, exactly as a line
                // ending in a joint word does
                self.skip_ends();
            }
            // a run of words, each stating a relation at this joint: `-> tangent equal` is a
            // corner that is tangent there, between two links also equal in length.  A word
            // that opens a link is the next link's own — `-> vertical line right(…)` is a
            // plain corner onto a levelled line, not a `vertical` joint — which is the same
            // order of questions the colouring asks (`tint_word`), so the two cannot disagree
            let mut words: Vec<(String, Vec<OpArg>, Span)> = Vec::new();
            loop {
                let Some(Tok::Ident(w)) = self.peek() else { break };
                if !joint_word(w) || opens_link(w, self.word_at(past_args(&self.t, self.i))) {
                    break;
                }
                let w = w.clone();
                let lo = self.here().lo as usize;
                self.i += 1;
                // an infix operator carries its own parentheses: `p1 distance(80) p2` is a
                // chain of one joint, which is the unification that makes a lone statement
                // and a chain one grammar rather than two
                let args = self.op_args(&w)?;
                words.push((w, args, Span::new(lo, self.prev_hi())));
                hi = self.prev_hi();
                // the marker may stand on either side of the words, or both — `A -> equal -> B`
                // is the one joint `A -> equal B` is — and any marker threads.  Read beside its
                // word, before the line break, so a continuation onto the next line never picks
                // up a marker that was written to start a statement there
                if self.peek() == Some(&Tok::Arrow) {
                    thread = true;
                    self.i += 1;
                    hi = self.prev_hi();
                    break;
                }
                // a line ending in a joint word continues its chain onto the next
                self.skip_ends();
            }
            // the retired 0.8 list, caught so a document written against it says what to write
            if (thread || !words.is_empty()) && self.peek() == Some(&Tok::P('(')) {
                self.fail("a joint states its relations as bare words: `-> equal angle(30deg)` (spec §6.6)");
            }
            if !thread && words.is_empty() {
                // the retired corner word, caught here so a 0.7 document says what to write
                if self.peek_word("to") {
                    self.fail("`to` is retired: a corner is written `->` (spec §6.6)");
                    self.i += 1;
                    thread = true;
                    hi = self.prev_hi();
                } else {
                    break;
                }
            }
            // a line ending in a joint continues the chain on the next — the one place a
            // statement runs past its line's end
            self.skip_ends();
            if self.eat_word("close") {
                if !thread {
                    self.fail("a loop is a thread: a chain closes with `-> close`");
                }
                close = Some(Joint {
                    thread: true,
                    words,
                    span: Span::new(start, self.prev_hi()),
                    sound: true,
                });
                break;
            }
            // a block body may end mid-joint: the marker (with any words) standing at the
            // body's `}` threads the chain onto the next copy's first link (issue #38).  Only
            // a *threaded* joint may dangle — an unthreaded trailing word still wants its
            // right operand, and keeps its error.
            if thread && self.in_body > 0 && self.peek() == Some(&Tok::P('}')) {
                open = Some(Joint { thread, words, span: Span::new(start, hi), sound: true });
                break;
            }
            joints.push(Joint { thread, words, span: Span::new(start, hi), sound: true });
            links.push(self.link()?);
        }
        // the trailing clauses a statement may carry — a lone infix operator is a one-joint
        // chain, so it carries them here as it would anywhere else
        let mut place = None;
        let mut place_span = Span::default();
        let mut seeds: Vec<OpArg> = Vec::new();
        let mut class = Classes::default();
        let mut class_span = Span::default();
        loop {
            if self.eat_hint_clause().is_some() {
                for h in self.hint_body("t: 0.4")? {
                    seeds.push(h.into());
                }
            } else if class.is_empty() && !joints.is_empty() && self.peek_word("class") {
                // a class qualifies the line's one relation, as a placement does
                let (c, sp) = self.class_clause(self.here().lo as usize);
                if c.is_empty() {
                    self.fail("`class` names at least one class");
                    return None;
                }
                class = c;
                class_span = sp;
            } else if place.is_none() && !joints.is_empty() && self.peek_word("at") {
                // a placement qualifies a *dimension*, so it is read only where the line states
                // one — after a declaration, `at (…)` is not a clause the language has
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
        let whole = Span::new(lo, self.prev_hi());
        // whether the line's end can take a trailing clause appended later: a chain ending
        // in a name (or sealed by `close`) can, one ending in a declaration cannot — the
        // declaration reads a trailing `at` as its own retired seed spelling
        let open_end =
            close.is_some() || matches!(links.last().map(|l| &l.body), Some(LinkBody::Ref(_)));
        // the open joint's boundary links, read while they are in hand — desugar consumes them
        let ends = match &open {
            Some(j) => self.open_ends(j.span, &links),
            None => None,
        };
        let first = out.len();
        self.desugar(links, joints, close, open.is_some(), whole, next_id, out);
        if let (Some(j), Some((fe, le, named))) = (open, ends) {
            self.open_finish(j, fe, le, named, &out[first..], next_id);
        }
        // a placement qualifies exactly one dimension (§13.1), so it is attached to the one
        // relation the line states, wherever that fell among the links.  A line stating
        // several offers no way to say which — guessing the first put callouts on statements
        // nobody measured — so both none and several are refused.  Where no placement was
        // written and the line offers a spot, the spot one *would* take is recorded all the
        // same — an empty span at the insertion point, `Decl::hint_span`'s device — so
        // `reconcile` can write a dragged callout down without re-deriving the line.
        {
            let mut rels =
                out[first..].iter_mut().filter(|s| matches!(s.kind, StmtKind::Relation(_)));
            match (rels.next(), rels.next()) {
                (Some(one), None) => {
                    if let StmtKind::Relation(r) = &mut one.kind {
                        r.place = place;
                        r.place_span = match (place, open_end) {
                            (Some(_), _) => place_span,
                            (None, true) => Span::new(whole.hi as usize, whole.hi as usize),
                            (None, false) => Span::default(),
                        };
                        r.class = class.clone();
                        r.class_span = class_span;
                    }
                }
                (Some(_), Some(_)) if place.is_some() => self.errs.push(SynErr {
                    span: place_span,
                    message: "a placement qualifies one dimension, and this line states \
                              several relations (§13.1)"
                        .to_string(),
                }),
                (None, _) if place.is_some() => self.errs.push(SynErr {
                    span: place_span,
                    message: "a placement qualifies a dimension, and this line states no \
                              relation (§13.1)"
                        .to_string(),
                }),
                _ => {}
            }
        }
        // a seed qualifies the one statement the line states, which for a lone infix operator
        // is the statement itself
        if let Some(StmtKind::Relation(r)) = out.get_mut(first).map(|s| &mut s.kind) {
            if let Some(w) = r.form.written_mut() {
                w.args.extend(seeds);
            }
        }
        Some(())
    }

    /// `[prefix…] KIND name(…)`, or a bare name — the two things a link may stand on.
    fn link(&mut self) -> Option<Link> {
        let mut prefixes: Vec<(Name, Vec<OpArg>)> = Vec::new();
        let kind = loop {
            let Some(Tok::Ident(w)) = self.peek().cloned() else {
                self.fail("expected an element");
                return None;
            };
            if let Some(k) = EntKind::parse(&w) {
                break Some(k);
            }
            // a name, not a keyword: the link stands on something declared elsewhere.  A prefix
            // word only reaches here when an element follows it, so this cannot swallow one.
            if prefixes.is_empty() && !prefix_word(&w) {
                break None;
            }
            if !prefix_word(&w) {
                self.fail("expected an element");
                return None;
            }
            // a prefix word carries its own parentheses like any other operator:
            // `radius(25) circle base(center: c)`
            let name = Name { text: w, span: self.here() };
            self.i += 1;
            let args = self.op_args(&name.text)?;
            prefixes.push((name, args));
        };
        let lo = self.here().lo as usize;
        let Some(kind) = kind else {
            let r = self.refr()?;
            return Some(Link { prefixes, span: r.span, body: LinkBody::Ref(r) });
        };
        self.i += 1; // the kind keyword
        let decl = self.decl(kind)?;
        let body = LinkBody::Decl(Box::new(decl));
        Some(Link { prefixes, body, span: Span::new(lo, self.prev_hi()) })
    }

    /// The statements a chain is sugar for, in the order their text sits: each link's prefix
    /// relations, its declaration, and between links the joint that binds them.
    #[allow(clippy::too_many_arguments)]
    fn desugar(
        &mut self,
        mut links: Vec<Link>,
        mut joints: Vec<Joint>,
        mut close: Option<Joint>,
        open: bool,
        whole: Span,
        next_id: &mut u32,
        out: &mut Vec<Stmt>,
    ) {
        let chained = links.len() > 1 || close.is_some() || open;
        let n = links.len();
        // **A lone infix operator is a one-joint chain**, and that is a unification rather than
        // a new case — but the statement it makes occupies its whole line, so it is recorded as
        // the ordinary statement it is: a whole-line span to splice, and no chain to be part of.
        let lone = n == 2
            && joints.len() == 1
            && close.is_none()
            && !joints[0].thread
            && joints[0].words.len() == 1
            && links.iter().all(|l| l.kind().is_none());

        // **threading is stated at the joint, never inferred** (spec §6.6): `->` says the two
        // links beside it share a boundary point, and its absence says they do not — so a chain
        // may mix declarations and names freely, and each marker is judged where it stands.
        // A marker needs an end on each side it can see, which is exactly a line or an arc; a
        // side that only names an element has a kind only elaboration knows, and is trusted to
        // the point the other side names.
        if close.is_some() && n < 2 {
            self.errs.push(SynErr {
                span: links[0].span_of_name(),
                message: "a chain closes over at least two elements".to_string(),
            });
            if let Some(c) = &mut close {
                c.sound = false;
            }
        }
        let mut endless = vec![false; n]; // reported once per link, however many markers reach it
        for k in 0..joints.len() + usize::from(close.as_ref().is_some_and(|c| c.sound)) {
            let (thread, li, ri) = match joints.get(k) {
                Some(j) => (j.thread, k, k + 1),
                None => (true, n - 1, 0),
            };
            if !thread {
                continue;
            }
            let mut sound = true;
            for side in [li, ri] {
                if links[side].kind().is_some_and(|k| k.ends().is_none()) {
                    if !endless[side] {
                        self.errs.push(SynErr {
                            span: links[side].span_of_name(),
                            message: format!(
                                "a corner joins lines and arcs; a {} has no ends to thread",
                                links[side].kind().map(|k| k.as_str()).unwrap_or("thing")
                            ),
                        });
                        endless[side] = true;
                    }
                    sound = false;
                }
            }
            if !sound {
                match joints.get_mut(k) {
                    Some(j) => j.sound = false,
                    None => close.as_mut().expect("the close joint").sound = false,
                }
            }
        }

        // threading: at each threaded joint the shared point is named by exactly one side, by
        // both in agreement, or — between two declarations — by nobody, in which case the chain
        // mints it (`thread`).  An end no marker reaches is an implicit child like any other
        // unwritten slot (§6.2): `line l1 -> line l2` is two lines and three points, one shared.
        for k in 0..joints.len() {
            if joints[k].thread && joints[k].sound {
                self.thread(&mut links, k, k + 1, joints[k].span);
            }
        }
        if close.as_ref().is_some_and(|c| c.sound) {
            let sp = close.as_ref().expect("just checked").span;
            self.thread(&mut links, n - 1, 0, sp);
        }
        // the links are consumed below, so what a joint needs of them — the entity and its kind
        // where that is known — is taken first, and only where there are joints to need it
        let sig: Vec<(Ref, Option<EntKind>)> = match chained {
            true => links.iter().map(|l| (l.entity(), l.kind())).collect(),
            false => Vec::new(),
        };
        // how each worded joint splices when its statement is doomed (`Chained`): a threaded
        // one steps down to the bare corner; an unthreaded one becomes a statement break, its
        // span grown over a terminal name-link that a break would leave dangling; in a chain
        // that closes no break is safe — it would re-aim the `close` — so the joint is Stuck
        let spell: Vec<(Span, Fall)> = joints
            .iter()
            .enumerate()
            .map(|(k, j)| {
                if j.thread {
                    return (j.span, Fall::Joint);
                }
                if close.is_some() {
                    return (j.span, Fall::Stuck);
                }
                let mut sp = j.span;
                if k == 0 && links[0].kind().is_none() {
                    sp = Span::new(links[0].span.lo as usize, sp.hi as usize);
                }
                if k + 1 == n - 1 && links[n - 1].kind().is_none() {
                    // …and through the trailing clauses: a placement or a seed after the
                    // chain qualifies this line's statements, so text a break left standing
                    // behind the taken name-link would dangle
                    sp = Span::new(sp.lo as usize, whole.hi as usize);
                }
                (sp, Fall::Infix)
            })
            .collect();
        let at = |i: usize| (&sig[i].0, sig[i].1);
        for (i, link) in links.into_iter().enumerate() {
            if i > 0 && joints[i - 1].sound {
                let j = &joints[i - 1];
                for k in 0..j.words.len() {
                    let (w, args, wspan) = &j.words[k];
                    // a word with siblings spans itself alone — the blanks around it are the
                    // splice's business, so a comment or a line break between two words is
                    // never part of either — and carries the joint's one-word doom for when
                    // the whole joint falls; the joint's only word steps down to the marker
                    // or to a statement break (`spell`)
                    let (span, how) = if lone {
                        (whole, Chained::No)
                    } else {
                        let (of, fall) = spell[i - 1];
                        word_spelling(j.words.len(), *wspan, of, fall)
                    };
                    out.extend(self.joint_stmt(
                        w,
                        args,
                        *wspan,
                        span,
                        at(i - 1),
                        at(i),
                        j.thread,
                        how,
                        next_id,
                    ));
                }
            }
            let ent = link.entity();
            for (word, args) in &link.prefixes {
                let sp = word.span;
                let rel = Relation {
                    place: None,
                    place_span: Span::default(),
                    form: crate::syntax::RelationForm::Written(Written {
                        word: word.clone(),
                        fixity: Fixity::Prefix,
                        ops: vec![ent.clone()],
                        args: args.clone(),
                        span: sp,
                    }),
                    claim: false,
                    class: Classes::default(),
                    class_span: Span::default(),
                };
                let Some(id) = self.mint_stmt(next_id, sp) else { return };
                out.push(Stmt {
                    id,
                    kind: StmtKind::Relation(rel),
                    span: sp,
                    chained: Chained::Prefix,
                });
            }
            // a link that only *names* an element declares nothing, so it emits no statement of
            // its own — the whole of what it contributes is being one end of its joints
            let LinkBody::Decl(decl) = link.body else { continue };
            let decl = *decl;
            let Some(id) = self.mint_stmt(next_id, link.span) else { return };
            out.push(Stmt {
                id,
                kind: StmtKind::Decl(decl),
                span: link.span,
                chained: if chained { Chained::Link } else { Chained::No },
            });
        }
        if let Some(c) = close {
            let mut sealed = Vec::new();
            if c.sound {
                for k in 0..c.words.len() {
                    let (w, args, wspan) = &c.words[k];
                    let (span, how) = word_spelling(c.words.len(), *wspan, c.span, Fall::Close);
                    sealed.extend(self.joint_stmt(
                        w,
                        args,
                        *wspan,
                        span,
                        at(n - 1),
                        at(0),
                        true,
                        how,
                        next_id,
                    ));
                }
            }
            if sealed.is_empty() {
                // `-> close` states nothing, so no statement owns its words; the last link's
                // span grows over them, or an append would land in the middle of the chain
                if let Some(last) = out.last_mut() {
                    last.span = Span::new(last.span.lo as usize, c.span.hi as usize);
                }
            } else {
                out.extend(sealed);
            }
        }
    }

    /// The statement one joint states, where it states one — a plain corner states nothing.
    #[allow(clippy::too_many_arguments)]
    fn joint_stmt(
        &mut self,
        word: &str,
        args: &[OpArg],
        at: Span,
        span: Span,
        left: (&Ref, Option<EntKind>),
        right: (&Ref, Option<EntKind>),
        threaded: bool,
        chained: Chained,
        next_id: &mut u32,
    ) -> Option<Stmt> {
        let rel = self.joint_relation(word, args, at, left, right, threaded)?;
        let id = self.mint_stmt(next_id, span)?;
        Some(Stmt { id, kind: StmtKind::Relation(rel), span, chained })
    }

    /// One boundary link of an open joint, or the refusal that stops it: the joint threads the
    /// body's own declarations — a name-link's kind is elaboration's to say, and its boundary
    /// is what issue #35 will teach `thread` to read — and only a kind with ends threads.
    fn open_end(&mut self, link: &Link, entry: bool) -> Option<OpenEnd> {
        let Some(kind) = link.kind() else {
            self.errs.push(SynErr {
                span: link.span_of_name(),
                message: format!(
                    "`{}` is declared elsewhere, and an open joint threads the body's own \
                     elements",
                    ref_text(&link.entity())
                ),
            });
            return None;
        };
        let Some((en, ex)) = kind.ends() else {
            self.errs.push(SynErr {
                span: link.span_of_name(),
                message: format!(
                    "a corner joins lines and arcs; a {} has no ends to thread",
                    kind.as_str()
                ),
            });
            return None;
        };
        let slot = if entry { en } else { ex };
        let declared = slot_ref(link, slot);
        let entity = link.entity();
        let boundary = match &declared {
            Some(r) => r.clone(),
            // the mint's own spelling: the dotted path *is* the anonymous child's name (§6.2)
            None => boundary_ref(entity.root.clone(), kind, slot),
        };
        Some(OpenEnd { kind, slot, declared: declared.is_some(), boundary, entity })
    }

    /// The two boundary links of a body's open joint, read while the links are in hand: what
    /// the weld needs of each.  `None` where the joint cannot stand, each refusal its own
    /// message — mirroring `thread`, whose corner this is, stated across the copy seam.
    fn open_ends(&mut self, at: Span, links: &[Link]) -> Option<(OpenEnd, OpenEnd, OpenNamed)> {
        let fe = self.open_end(links.first()?, true);
        let le = self.open_end(links.last()?, false);
        let (fe, le) = (fe?, le?);
        // the shared point is named by at most one side: both declared name two different
        // points — copy i's exit and copy i+1's entry are different entities however alike
        // their leaves read — and welding them is the longhand's `coincident` to state
        let named = match (fe.declared, le.declared) {
            (true, true) => {
                self.errs.push(SynErr {
                    span: at,
                    message: format!(
                        "the joint names two points: `{}` leaves at `{}` and the next copy's \
                         `{}` arrives at `{}`",
                        ref_text(&le.entity),
                        ref_text(&le.boundary),
                        ref_text(&fe.entity),
                        ref_text(&fe.boundary)
                    ),
                });
                return None;
            }
            (true, false) => OpenNamed::First,
            (false, true) => OpenNamed::Last,
            (false, false) => OpenNamed::Neither,
        };
        Some((fe, le, named))
    }

    /// The open joint, settled: the boundary links' declaration statements are found among
    /// what the chain just emitted, and each word becomes the statement an in-chain joint's
    /// would — the right operand spelled through `next`, the flattener's own word for the
    /// sibling copy, so `-> tangent` gets the regular At-form and `-> angle(a)` the plain
    /// infix, per pair of copies.
    fn open_finish(
        &mut self,
        j: Joint,
        fe: OpenEnd,
        le: OpenEnd,
        named: OpenNamed,
        made: &[Stmt],
        next_id: &mut u32,
    ) {
        // both boundary links are declarations (`open_ends` said so), and links emit their
        // declarations in link order — so the slice's first and last `Decl` are theirs
        let mut decls = made.iter().filter(|s| matches!(s.kind, StmtKind::Decl(_)));
        let first_id = decls.next().map(|s| s.id);
        let last_id = decls.last().map(|s| s.id).or(first_id);
        let (Some(first_id), Some(last_id)) = (first_id, last_id) else { return };
        let lref = le.entity.clone();
        let rref = under_root("next", &fe.entity, j.span);
        let mut stmts = Vec::new();
        for k in 0..j.words.len() {
            let (w, args, wspan) = &j.words[k];
            let (span, how) = word_spelling(j.words.len(), *wspan, j.span, Fall::Joint);
            stmts.extend(self.joint_stmt(
                w,
                args,
                *wspan,
                span,
                (&lref, Some(le.kind)),
                (&rref, Some(fe.kind)),
                true,
                how,
                next_id,
            ));
        }
        self.open = Some(OpenJoint {
            stmts,
            words: j.words,
            last: OpenSide { stmt: last_id, kind: le.kind, slot: le.slot, boundary: le.boundary },
            first: OpenSide { stmt: first_id, kind: fe.kind, slot: fe.slot, boundary: fe.boundary },
            named,
            span: j.span,
        });
    }

    /// Resolve one threaded joint's shared point between link `li` (its exit) and link `ri`
    /// (its entry).
    fn thread(&mut self, links: &mut [Link], li: usize, ri: usize, at: Span) {
        // which slot each side threads through, where that side is declared here.  A link that
        // only *names* an element has no list to read or fill — its boundary is its own
        // declaration's business — so the declared side must say where the two meet, usually
        // by the existing element's own child (`line l(a, k.start) -> tangent k`).
        let exit = links[li].kind().and_then(|k| k.ends()).map(|(_, ex)| ex);
        let entry = links[ri].kind().and_then(|k| k.ends()).map(|(en, _)| en);
        // a joint threads a *name*: it welds two links to one point, and only a name says
        // which (`slot_ref`'s rule).
        let slot = |i: usize, k: Option<usize>| k.and_then(|k| slot_ref(&links[i], k));
        let (left, right) = (slot(li, exit), slot(ri, entry));
        // Write a name into a declared side's boundary slot.  A side that only names an element
        // has no list to fill; and a name that already denotes exactly that slot — the link's
        // own dotted boundary, which a written-back chain uses to name the shared point — is
        // left alone rather than written over itself, which would be a reference with no floor.
        fn fill(link: &mut Link, slot: Option<usize>, r: Ref) {
            let Some(k) = slot else { return };
            if let (Some(kind), LinkBody::Decl(d)) = (link.kind(), &mut link.body) {
                if let [Seg::Field(f)] = r.path.as_slice() {
                    if r.root.text == d.name.key().text && f.text == boundary_name(kind, k) {
                        return;
                    }
                }
                d.children[k] = vec![Kid::Ref(r)];
            }
        }
        // Whether link `a` is built before link `b`: phase 2 builds per kind in declaration
        // order of `EntKind` (`primitives()` order), and within a kind in statement order,
        // which for a chain is link order.
        fn builds_first(a: &Link, ia: usize, b: &Link, ib: usize) -> bool {
            let ord = |l: &Link| l.kind().map(build_rank).unwrap_or(usize::MAX);
            (ord(a), ia) < (ord(b), ib)
        }
        match (left, right) {
            (Some(l), Some(r)) => {
                if !refs_eq(&l, &r) {
                    self.errs.push(SynErr {
                        span: at,
                        message: format!(
                            "the joint names two points: `{}` leaves at `{}` and `{}` arrives \
                             at `{}`",
                            ref_text(&links[li].entity()),
                            ref_text(&l),
                            ref_text(&links[ri].entity()),
                            ref_text(&r),
                        ),
                    });
                }
            }
            (Some(l), None) => {
                fill(&mut links[ri], entry, l);
            }
            (None, Some(r)) => {
                fill(&mut links[li], exit, r);
            }
            (None, None) => {
                // between two declarations the chain mints the point itself: the boundary of
                // the side built first is an anonymous child with a name — the dotted path
                // *is* the name (§6.2) — so the other side's slot is filled with exactly that
                // name.  The side built later takes the fill, so the name exists by the time
                // it resolves; a side that only names an element has no kind to read a
                // boundary field off, so there the point must be named where it stands.
                let lf = links[li].kind().zip(exit);
                let rf = links[ri].kind().zip(entry);
                match (lf, rf) {
                    (Some((lk, ls)), Some((rk, rs))) => {
                        if builds_first(&links[li], li, &links[ri], ri) {
                            let r = boundary_ref(links[li].entity().root, lk, ls);
                            fill(&mut links[ri], entry, r);
                        } else {
                            let r = boundary_ref(links[ri].entity().root, rk, rs);
                            fill(&mut links[li], exit, r);
                        }
                    }
                    _ => self.errs.push(SynErr {
                        span: at,
                        message: format!(
                            "neither `{}` nor `{}` names the point where they meet",
                            ref_text(&links[li].entity()),
                            ref_text(&links[ri].entity())
                        ),
                    }),
                }
            }
        }
    }

    /// Lower a joint word for the operand kinds. Tangency depends on both kinds;
    /// threaded joints also validate that their boundaries can meet.
    fn joint_relation(
        &mut self,
        word: &str,
        extra: &[OpArg],
        at: Span,
        left: (&Ref, Option<EntKind>),
        right: (&Ref, Option<EntKind>),
        threaded: bool,
    ) -> Option<Relation> {
        use EntKind::{Arc, Line};
        let (lref, lk) = left;
        let (rref, rk) = right;
        // A joint is the infix operator its word already is, written between two links instead
        // of between two names — so it makes the same `Written` a lone statement does, and
        // `program::constrain` settles both.  The chain contributes the one thing it knows and
        // the operator cannot: *which end* two links meet at.
        let written = |w: &str, ops: Vec<Ref>, args: Vec<OpArg>| {
            Some(Relation {
                place: None,
                place_span: Span::default(),
                form: crate::syntax::RelationForm::Written(Written {
                    word: Name { text: w.to_string(), span: at },
                    fixity: Fixity::Infix,
                    ops,
                    args,
                    span: at,
                }),
                claim: false,
                class: Classes::default(),
                class_span: Span::default(),
            })
        };
        let end = |w: &str| {
            vec![OpArg::Named(Name { text: "at".into(), span: at }, Arg::Word(w.to_string()))]
        };
        // no marker, no corner: the word is the ordinary infix operator between the two, as it
        // is between two names — for `tangent`, the well-conditioned bare pair, which is the
        // correct statement exactly when the two are separate
        if !threaded {
            return written(word, vec![lref.clone(), rref.clone()], extra.to_vec());
        }
        match (lk, rk) {
            // the joint knows the shared point, so tangency is stated *at* it — the regular
            // form, with `at:` read off the direction of travel
            (Some(Line), Some(Arc)) if word == "tangent" => {
                written("tangent", vec![rref.clone(), lref.clone()], end("start"))
            }
            (Some(Arc), Some(Line)) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("end"))
            }
            // a corner between a fresh element and one declared elsewhere: the declared side
            // says which of its ends was threaded, and elaboration settles the pair once the
            // name's kind is known — the `at:` selector is what keeps the statement the regular
            // form there too, never the bare pair over a coincidence
            (Some(Arc), None) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("end"))
            }
            (None, Some(Arc)) if word == "tangent" => {
                written("tangent", vec![rref.clone(), lref.clone()], end("start"))
            }
            (Some(Line), None) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("p2"))
            }
            // the named side can only sensibly be an arc here (a line meeting a line tangent is
            // collinear, and needs both declared to say so); `at: end` is the arc's exit, and a
            // name of any other kind is refused where its kind becomes known
            (None, Some(Line)) if word == "tangent" => {
                written("tangent", vec![lref.clone(), rref.clone()], end("end"))
            }
            // two straight runs meeting tangent share a point and a direction: collinear
            (Some(Line), Some(Line)) if word == "tangent" => {
                written("parallel", vec![lref.clone(), rref.clone()], extra.to_vec())
            }
            // two arcs meeting at a corner already touch there, so `TangentCircleCircle` would
            // be a row that is zero at every solution — a *tangency between names* is a real
            // statement, but at a shared corner there is nothing left for it to say
            (Some(a), Some(b))
                if word == "tangent"
                    && matches!(a, Arc | EntKind::Circle)
                    && matches!(b, Arc | EntKind::Circle) =>
            {
                self.errs.push(SynErr {
                    span: at,
                    message: format!(
                        "`tangent` does not join a {} to a {} at a corner: they already meet \
                         there, and there is no regular form left to state",
                        a.as_str(),
                        b.as_str()
                    ),
                });
                None
            }
            _ => written(word, vec![lref.clone(), rref.clone()], extra.to_vec()),
        }
    }
}
