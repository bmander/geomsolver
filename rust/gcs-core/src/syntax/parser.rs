//! Program and statement parsing with error recovery.

mod chains;
mod declarations;
mod relations;
mod statements;

use super::lexer::{lex, Tok};
use super::names::write_ref;
use super::{
    seed_arg, Component, InBlock, Name, OpArg, OpenJoint, Program, Ref, Seg, Span, Stmt, SynErr,
    Use, MAX_STMTS, MAX_TEXT,
};

/// One `name: value` of a `hint(…)` clause, read but not yet resolved: the key, where it was
/// written, and the value exactly as `value_text` gave it back.
struct Hint {
    key: String,
    /// Where the key stands, so an unknown one is reported there rather than after its value.
    at: Span,
    value: Option<f64>,
    text: String,
    span: Span,
    /// `at: REF` — the one key whose value is a *reference* and not a number: a place (§6.4).
    /// Read wherever the clause stands, since the grammar is the clause's own; which tables
    /// take one is the caller's question, and a declaration's is the only one that does.
    place: Option<Ref>,
}

impl From<Hint> for OpArg {
    /// A hint read from a `hint(…)` clause is a **seed** for the slot it names — the only thing
    /// such a clause can be (spec §4.3).  A pin is the same argument with `pinned` set, built
    /// where `==` is read instead.
    fn from(h: Hint) -> OpArg {
        OpArg::Slot {
            key: Name { text: h.key, span: h.at },
            arg: seed_arg(h.value, h.text, h.span, false),
        }
    }
}

/// Resource limits for parsing one source file, including nested bodies and chain expansion.
#[derive(Clone, Copy, Debug)]
pub struct ParseLimits {
    pub max_depth: usize,
    pub max_statements: usize,
}

impl Default for ParseLimits {
    fn default() -> Self {
        Self { max_depth: 32, max_statements: MAX_STMTS }
    }
}

struct P<'a> {
    src: &'a str,
    limits: ParseLimits,
    statements: usize,
    exhausted: bool,
    t: Vec<(Tok, Span)>,
    i: usize,
    errs: Vec<SynErr>,
    /// A word `decl()` declined as a name because the language reserves it — kept until the
    /// statement ends, so a line that then fails to parse can say the likely cause.  A line
    /// that parses (`line tangent arc` is a chain) needs no saying, so this is only read
    /// beside a failure (`chain_or_one`).
    declined: Option<(String, Span)>,
    /// How many braced bodies are being read: a chain inside one may end mid-joint at the
    /// body's `}`, and a statement there ends at `}` as it would at a line break (issue #38).
    in_body: u32,
    /// A body's trailing open joint, handed up from the chain that read it to the block whose
    /// body it ends.  A `component` or a trace block takes it too — to refuse it, since
    /// neither has a next copy to continue onto.
    open: Option<OpenJoint>,
    /// The `in PLANE { … }` headers read so far — handed to the `Program` at the end.
    in_blocks: Vec<InBlock>,
    /// Whether the statements being read are a component's (§6.7): an `in PLANE { … }` block
    /// may stand there, over a plane the component was handed, where in a root block it may
    /// not — a header buried in a root statement's span is a splice no deletion could compose.
    in_comp: bool,
}

/// Parse as much geometry as possible, reporting errors and resuming at statement terminators.
pub fn parse(src: &str) -> (Program, Vec<SynErr>) {
    parse_with_limits(src, ParseLimits::default())
}

/// Parse with explicit resource limits. Limits may be tightened, but not raised above defaults.
pub fn parse_with_limits(src: &str, limits: ParseLimits) -> (Program, Vec<SynErr>) {
    parse_at(src, 0, 0, limits)
}

/// Read a text whose spans are to start at `base` and whose statements are numbered from
/// `first_id` — the form a module is read in (`modules::link`), so that its spans and ids join
/// the document's without a second coordinate anywhere.  The text is padded out to `base` before
/// it is read, which is what puts every span where it belongs at the cost of a scan over the
/// padding; the `Program` keeps the text itself, unpadded.
pub fn parse_from(src: &str, base: usize, first_id: u32) -> (Program, Vec<SynErr>) {
    parse_at(src, base, first_id, ParseLimits::default())
}

fn parse_at(src: &str, base: usize, first_id: u32, limits: ParseLimits) -> (Program, Vec<SynErr>) {
    let mut p = Program::new();
    p.text = src.to_string();
    if src.len() > MAX_TEXT {
        return (
            p,
            vec![SynErr {
                span: Span::new(base, base),
                message: format!("a program may not be longer than {MAX_TEXT} bytes"),
            }],
        );
    }
    let padded;
    let src = if base > 0 {
        padded = format!("{}{src}", " ".repeat(base));
        padded.as_str()
    } else {
        src
    };
    let (lexed, errs) = lex(src);
    let defaults = ParseLimits::default();
    let mut st = P {
        src,
        limits: ParseLimits {
            max_depth: limits.max_depth.min(defaults.max_depth),
            max_statements: limits.max_statements.min(defaults.max_statements),
        },
        statements: 0,
        exhausted: false,
        t: lexed.toks,
        i: 0,
        errs,
        declined: None,
        in_body: 0,
        open: None,
        in_blocks: Vec::new(),
        in_comp: false,
    };
    let mut body: Vec<Stmt> = Vec::new();
    let mut comps: Vec<Component> = Vec::new();
    let mut uses: Vec<Use> = Vec::new();
    let mut next_id = first_id;
    while !st.done() {
        st.skip_ends();
        if st.done() {
            break;
        }
        if st.peek_word("component") {
            if !st.take_statement(st.here()) {
                break;
            }
            match st.component(&mut next_id) {
                Some(c) => comps.push(c),
                None => st.resync(),
            }
            continue;
        }
        // `use engine.crank` — a module, named as a dotted path (§14.4).  At the top only: a
        // module is a set of components a document reads, and a body reads nothing but names.
        if st.peek_word("use") {
            if !st.take_statement(st.here()) {
                break;
            }
            match st.use_stmt() {
                Some(u) => uses.push(u),
                None => st.resync(),
            }
            continue;
        }
        // `curve name(` was a *family* until 0.11; a family is a component now (§6.5)
        if st.peek_word("curve") && matches!(st.t.get(st.i + 2).map(|(t, _)| t), Some(Tok::P('(')))
        {
            let span = st.here();
            st.errs.push(SynErr {
                span,
                message: "a curve family is a component now: write \
                          `component Name(c: circle, u: Angle) { … }` with the traced point \
                          inside it, and draw the curve as \
                          `curve e = Name(c).point over u in (a, b)`"
                    .to_string(),
            });
            st.resync();
            continue;
        }
        if st.chain_or_one(&mut next_id, &mut body).is_none() {
            st.resync();
        }
    }
    p.next_stmt = next_id;
    p.in_blocks = std::mem::take(&mut st.in_blocks);
    p.uses = uses;
    // named components first, the anonymous root last — `Program::root` takes the last, and a
    // program that declares components and nothing loose has its last component as the root
    let anon_empty = body.is_empty();
    p.components = comps;
    if !anon_empty || p.components.is_empty() {
        p.components.push(Component {
            name: None,
            formals: Vec::new(),
            body,
            span: Span::new(base, src.len()),
            module: None,
        });
    }
    let errs = std::mem::take(&mut st.errs);
    (p, errs)
}
impl<'a> P<'a> {
    fn take_statement(&mut self, span: Span) -> bool {
        if self.exhausted {
            return false;
        }
        if self.statements >= self.limits.max_statements {
            self.errs.push(SynErr {
                span,
                message: format!(
                    "a program may not hold more than {} statements",
                    self.limits.max_statements
                ),
            });
            self.exhausted = true;
            self.i = self.t.len();
            return false;
        }
        self.statements += 1;
        true
    }

    fn mint_stmt(&mut self, next_id: &mut u32, span: Span) -> Option<super::StmtId> {
        if !self.take_statement(span) {
            return None;
        }
        let Some(id) = next_id.checked_add(1) else {
            self.fail_at(span, "statement IDs exceed their supported range");
            self.exhausted = true;
            self.i = self.t.len();
            return None;
        };
        *next_id = id;
        Some(super::StmtId(id))
    }

    fn done(&self) -> bool {
        self.i >= self.t.len()
    }

    fn peek(&self) -> Option<&Tok> {
        self.t.get(self.i).map(|(t, _)| t)
    }

    fn here(&self) -> Span {
        self.t.get(self.i).map(|(_, s)| *s).unwrap_or(Span::new(self.src.len(), self.src.len()))
    }

    /// The source from an offset to wherever the last token ended.
    ///
    /// Clamped, because a run that consumed no tokens leaves the end *before* the start, and a
    /// reversed range is a panic — which on `wasm32-unknown-unknown` is an abort, so a program
    /// that is not a program would take the page with it.
    fn text_from(&self, from: usize) -> &'a str {
        let lo = from.min(self.src.len());
        let hi = self.prev_hi().clamp(lo, self.src.len());
        self.src.get(lo..hi).unwrap_or("")
    }

    fn prev_hi(&self) -> usize {
        self.t.get(self.i.saturating_sub(1)).map(|(_, s)| s.hi as usize).unwrap_or(self.src.len())
    }

    fn bump(&mut self) -> Option<(Tok, Span)> {
        let v = self.t.get(self.i).cloned();
        if v.is_some() {
            self.i += 1;
        }
        v
    }

    fn eat_p(&mut self, c: char) -> bool {
        if self.peek() == Some(&Tok::P(c)) {
            self.i += 1;
            true
        } else {
            false
        }
    }

    fn want_p(&mut self, c: char) -> bool {
        if self.eat_p(c) {
            return true;
        }
        self.fail(&format!("expected `{c}`"));
        false
    }

    /// Consume a hint clause opener and return its start offset.
    /// Only numbers in this clause are writable seeds.
    fn eat_hint_clause(&mut self) -> Option<usize> {
        if self.peek_word("hint") && self.t.get(self.i + 1).map(|(t, _)| t) == Some(&Tok::P('(')) {
            let lo = self.t[self.i].1.lo as usize;
            self.i += 2;
            return Some(lo);
        }
        None
    }

    /// `name:` at the head of a slot — the label and nothing else, consumed.
    ///
    /// Matched through the reference and cloned only in the winning arm, the way `eat_word` is:
    /// this is asked once per argument and once per hint, and most of those asks say no.
    fn slot_label(&mut self) -> Option<String> {
        match (self.peek(), self.t.get(self.i + 1).map(|(t, _)| t)) {
            (Some(Tok::Ident(s)), Some(Tok::P(':'))) => {
                let s = s.clone();
                self.i += 2;
                Some(s)
            }
            _ => None,
        }
    }

    /// Read named hint values through the closing parenthesis. The caller validates
    /// which keys its construct accepts.
    fn hint_body(&mut self, eg: &str) -> Option<Vec<Hint>> {
        let mut out = Vec::new();
        while !self.eat_p(')') {
            let at = self.here();
            let Some(key) = self.slot_label() else {
                self.fail(&format!("a hint names what it seeds: `hint({eg})`"));
                return None;
            };
            let (value, text, span, place) = if key == "at" {
                // a place, not a number — `at: pin`, `at: k` — so it is read as a reference;
                // `at: (3, 4)` is the coordinate pair the keys replaced, and says so
                if self.peek() == Some(&Tok::P('(')) {
                    self.fail("`at:` names a place; a coordinate seed is `hint(x: …, y: …)`");
                    return None;
                }
                let r = self.refr()?;
                let mut text = String::new();
                write_ref(&mut text, &r);
                (None, text, r.span, Some(r))
            } else {
                let (value, text, span) = self.value_text()?;
                (value, text, span, None)
            };
            out.push(Hint { key, at, value, text, span, place });
            if !self.eat_p(',') && self.peek() != Some(&Tok::P(')')) {
                self.fail("expected `,` or `)`");
                return None;
            }
        }
        Some(out)
    }

    fn eat_word(&mut self, w: &str) -> bool {
        if matches!(self.peek(), Some(Tok::Ident(s)) if s == w) {
            self.i += 1;
            true
        } else {
            false
        }
    }

    fn ident(&mut self) -> Option<Name> {
        match self.bump() {
            Some((Tok::Ident(s), span)) => Some(Name { text: s, span }),
            _ => {
                self.i = self.i.saturating_sub(1);
                self.fail("expected a name");
                None
            }
        }
    }

    fn fail(&mut self, msg: &str) {
        let span = self.here();
        self.fail_at(span, msg);
    }

    /// The same, at a span already read — a key whose value has since been consumed is reported
    /// where it was written and not where the parser has got to.
    fn fail_at(&mut self, span: Span, msg: &str) {
        // one message per position: a cascade after the first is noise
        if self.errs.last().map(|e| e.span.lo) != Some(span.lo) {
            self.errs.push(SynErr { span, message: msg.to_string() });
        }
    }

    fn skip_ends(&mut self) {
        while self.peek() == Some(&Tok::Nl) {
            self.i += 1;
        }
    }

    /// Past the next statement terminator, so one bad statement costs one statement.
    fn resync(&mut self) {
        while !self.done() && self.peek() != Some(&Tok::Nl) {
            self.i += 1;
        }
        self.skip_ends();
    }

    fn number(&mut self) -> Option<f64> {
        let neg = if self.eat_p('-') {
            true
        } else {
            self.eat_p('+');
            false
        };
        match self.bump() {
            Some((Tok::Num(v), _)) => Some(if neg { -v } else { v }),
            _ => {
                self.i = self.i.saturating_sub(1);
                self.fail("expected a number");
                None
            }
        }
    }

    /// A value written where a number goes.  Plain digits come back as the number they are;
    /// anything else — `Rr`, `R + m`, `tau / N` — comes back as text for expansion to work out
    /// against the parameters in scope.
    fn value_text(&mut self) -> Option<(Option<f64>, String, Span)> {
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
        let text = self.text_from(from).trim().to_string();
        if text.is_empty() {
            self.fail("expected a number");
            return None;
        }
        let span = Span::new(from, self.prev_hi());
        Some((text.parse::<f64>().ok().filter(|v| v.is_finite()), text, span))
    }

    fn refr(&mut self) -> Option<Ref> {
        let root = self.ident()?;
        let lo = root.span.lo as usize;
        let mut path = Vec::new();
        loop {
            if self.eat_p('.') {
                path.push(Seg::Field(self.ident()?));
            } else if self.eat_p('[') {
                let (text, _) = self.expr_until(']')?;
                if !self.want_p(']') {
                    return None;
                }
                path.push(Seg::Index(text));
            } else {
                break;
            }
        }
        Some(Ref { root, path, span: Span::new(lo, self.prev_hi()) })
    }

    /// The source up to `end` at the top bracket level, as written.
    fn expr_until(&mut self, end: char) -> Option<(String, Span)> {
        let from = self.here().lo as usize;
        let mut depth = 0i32;
        while !self.done() {
            match self.peek() {
                Some(Tok::P('(')) | Some(Tok::P('[')) => depth += 1,
                Some(Tok::P(c)) if *c == end && depth == 0 => break,
                Some(Tok::P(')')) | Some(Tok::P(']')) if depth == 0 => break,
                Some(Tok::P(')')) | Some(Tok::P(']')) => depth -= 1,
                Some(Tok::Nl) => break,
                _ => {}
            }
            self.i += 1;
        }
        let text = self.text_from(from).trim().to_string();
        if text.is_empty() {
            self.fail("expected an expression");
            return None;
        }
        Some((text, Span::new(from, self.prev_hi())))
    }

    fn peek_word(&self, w: &str) -> bool {
        matches!(self.peek(), Some(Tok::Ident(s)) if s == w)
    }

    fn end_of_stmt(&mut self) {
        if self.done() || self.peek() == Some(&Tok::Nl) {
            return;
        }
        // inside a braced body the closing `}` ends a statement as a line break does — so
        // `cycle 4 { distance(50) line -> angle(90) }` is writable on the one line it is
        // about — and the brace is left standing, being the body's to consume
        if self.in_body > 0 && self.peek() == Some(&Tok::P('}')) {
            return;
        }
        self.fail("more on this line than the statement wanted");
    }
}
