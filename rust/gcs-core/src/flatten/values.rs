//! Scoped scalar definitions, dimensional evaluation and text substitution.

use super::*;

/// The drawing's unknown a declared number stands for when nothing binds it — a formal no call
/// binds, or an input with no value (`param beta: Angle`) — under its declared dimension.
pub(super) fn free(name: String, ty: Ty) -> Aff {
    Aff { free: Some(name), m: 1.0, c: 0.0, dim: ty.dim() }
}

/// A number worked out for a name declared `ty` — a formal, a typed `param` — taken to be what
/// the declaration says: refused if it said otherwise (`Tooth(a0: 30mm)` is a mistake, not a
/// conversion), and a bare number becomes one of that dimension.
pub(super) fn typed(a: Aff, ty: Ty, name: &str) -> Result<Aff, String> {
    a.dim.require(ty.dim(), name).map(|()| a.as_dim(ty.dim()))
}

/// One definition of a body — a `param` or a group's member — worked out with the rest.
struct Def {
    name: String,
    /// The text after the `:=`.
    text: String,
    span: Span,
    group_ref: bool,
    /// The type a `param` declared, which its value is worked out under.
    ty: Option<Ty>,
}

/// Why a name a text reads is out of this component's reach — a document name it must be
/// handed as an argument, or a group member the argument does not have — or `None`.  Asked of a
/// dimension's text (`settle_text`) and of a pin to a name (`t == s`) alike.
fn out_of_reach(name: &str, vals: &BTreeMap<String, Aff>, scope: &Scope) -> Option<String> {
    if vals.contains_key(name) {
        return None;
    }
    if scope.forbidden.contains(name) {
        return Some(format!("`{name}` is outside this component; pass it as an argument"));
    }
    if name.split_once('.').is_some_and(|(head, _)| scope.groups.contains(head)) {
        return Some(format!("no numeric member `{name}` in the group argument"));
    }
    None
}

/// A number worked out while elaborating.
///
/// The language's angle is the degree — `sin(90)` is 1, and every dimension a person reads is in
/// degrees — so a whole turn is `tau = 360`, not 2π.  Keeping `tau` at 2π and the texts in degrees
/// would give two units one name; units say which of the two each is (`expr::CONSTANTS`), so
/// `pi` is the dimensionless constant and `tau` is an **angle**, and `tau == 2 * pi * 1rad`
/// holds where it used to be a coincidence of the same digits.
pub(super) fn value_of(text: &str, env: &BTreeMap<String, Aff>, units: Units) -> Result<f64, String> {
    let a = value_aff(text, env, units)?;
    a.number().ok_or_else(|| {
        format!(
            "`{}` is not a number here — a component's parameters are, and the document's \
             dimensions are not",
            a.free.unwrap_or_default()
        )
    })
}

/// The same, keeping what it *is* beside what it is worth — and what it is *in terms of*: a
/// text over a formal left unbound comes to an affine value in that unknown (`bind`), which a
/// `param`, an argument to a nested instance and a dimension all carry on.  Only a caller that
/// needs a number (`value_of`: a seed, a count, an index) refuses one.
///
/// A `param`'s dimension has to survive into the names that read it, or `R := m * N / 2`
/// would forget that `m` was declared a `Length` and `Rt := R + m` would read as a plain
/// number added to a length — the very thing the check exists to catch, missed because the check
/// threw the answer away.
pub(crate) fn value_aff(
    text: &str,
    env: &BTreeMap<String, Aff>,
    units: Units,
) -> Result<Aff, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("nothing to work out".to_string());
    }
    let p = expr::parse_in(t, units)?;
    let a = expr::eval(&p.body, env)?;
    match (a.number(), &a.free) {
        (Some(v), _) if !v.is_finite() => Err(format!("comes to {v}")),
        // an unknown the scope *bound* — a formal left unbound, a param over one — carries on;
        // a name nothing binds is the document's, and a component's numbers cannot read it
        (None, Some(n)) if !env.values().any(|b| b.free.as_deref() == Some(n)) => {
            // a member a group or a vector does not have — `o.z` of `o := (1, 2)` — is said so
            if let Some((owner, member)) = n.rsplit_once('.') {
                let prefix = format!("{owner}.");
                let has: Vec<&str> = env
                    .keys()
                    .filter_map(|k| k.strip_prefix(&prefix))
                    .filter(|m| !m.contains('.'))
                    .collect();
                if !has.is_empty() {
                    let mut has: Vec<String> = has.iter().map(|m| format!("`{m}`")).collect();
                    let last = has.pop().unwrap_or_default();
                    let has = if has.is_empty() { last } else { format!("{} and {last}", has.join(", ")) };
                    return Err(format!("`{owner}` has {has}, not `{member}`"));
                }
            }
            Err(format!("`{n}` is not a number here — nothing in scope gives it one"))
        }
        _ => Ok(a),
    }
}

/// A group's members by their names under `owner` (`dims.bore`), a group written in place
/// listed before its own members (`dims.cyl`, then `dims.cyl.bore`).
fn members_of<'a>(
    owner: &str,
    fields: &'a [crate::syntax::InstArg],
    out: &mut Vec<(String, &'a crate::syntax::Name, &'a crate::syntax::InstArg)>,
) {
    for field in fields {
        let Some(label) = &field.label else { continue };
        let name = format!("{owner}.{}", label.text);
        out.push((name.clone(), label, field));
        if let crate::syntax::InstVal::Group(inner) = &field.value {
            members_of(&name, inner, out);
        }
    }
}

/// A dimension's text with the names in scope written in (`sub`), folded.
///
/// A text that comes out a plain number is replaced by it: the formula was about parameters the
/// flat document no longer has, so printing it back would name things that are not there.  One
/// that still reads a document name — `w / 2` — keeps its form, and the reader keeps their
/// formula.
fn fold(sub: &str, text: &str, units: Units) -> Result<String, String> {
    // nothing of the component's in it: leave it exactly as written, so `h = w / 2` and `3 1/8`
    // reach the document with the form somebody typed
    if sub == text {
        return Ok(text.to_string());
    }
    let p = expr::parse_in(sub, units)?;
    // a measurement comes to a number only once the drawing is solved: kept as written, for
    // the context to read then or to refuse
    if !p.body.measures().is_empty() {
        return Ok(sub.to_string());
    }
    let env: BTreeMap<String, Aff> = BTreeMap::new();
    let evaluated = expr::eval(&p.body, &env);
    if p.body.deps().is_empty() {
        if let Err(e) = &evaluated { return Err(e.to_string()); }
    }
    if let Ok(a) = evaluated {
        if let Some(v) = a.number() {
            if v.is_finite() {
                return Ok(a.dim.number_text(v, units));
            }
        }
    }
    Ok(sub.to_string())
}

/// Every identifier the environment knows, replaced by the number it stands for.
///
/// At identifier boundaries, so `flank` in `cos(flank)` is replaced and the `flank` inside
/// `flank_out` is not; and parenthesised, so a negative value does not change what binds to what.
pub(super) fn substitute(text: &str, vals: &BTreeMap<String, Aff>, units: Units) -> String {
    substitute_with(text, of_vals(vals, units))
}

/// What a word stands for in `vals`, as text: a number, parenthesised; a value affine in the
/// drawing's unknown (a formal left unbound, or a `param` over one) as `(m * name + c)`, or
/// the bare name when that is all it is.  `tau` and `turn` are `expr::CONSTANTS` now, and an
/// angle rather than a number that happens to be 360, so they are left for the evaluator.
fn of_vals(vals: &BTreeMap<String, Aff>, units: Units) -> impl Fn(&str) -> Option<String> + '_ {
    move |w| {
        let a = vals.get(w)?;
        if let Some(v) = a.number() {
            // a number that knows what it is says so, or `phi + atan2(…)` would read a plain
            // number added to an angle once `phi: Angle` was written in as one
            return Some(format!("({})", a.dim.number_text(v, units)));
        }
        let n = a.free.as_ref()?;
        Some(if a.m == 1.0 && a.c == 0.0 {
            n.clone()
        } else {
            format!("({} * {n} + {})", crate::syntax::num(a.m), crate::syntax::num(a.c))
        })
    }
}

/// An integrand settled (#121): the scope's numbers written in, and the point and tangent it is
/// read at renamed `p` and `t` whatever the document called them — so what reaches the sketch is
/// text over `p.x`, `p.y`, `t.x`, `t.y` and nothing else.  A name it reads that is none of those
/// and no number in scope is refused here, naming it: an energy is over its curve's shape, and an
/// unknown inside one would be a second thing to minimise over.
pub(crate) fn settle_integrand(
    t: &crate::syntax::Integral,
    vals: &BTreeMap<String, Aff>,
    units: Units,
) -> Result<String, String> {
    let bound = |w: &str| -> Option<Result<String, String>> {
        let (root, field) = w.split_once('.').unwrap_or((w, ""));
        let canon = if root == t.point.text {
            "p"
        } else if t.tangent.as_ref().is_some_and(|n| n.text == root) {
            "t"
        } else {
            return None;
        };
        Some(match field {
            "x" | "y" => Ok(format!("{canon}.{field}")),
            _ => Err(format!(
                "`{w}`: {} has an `x` and a `y` and nothing else",
                if canon == "p" { "the point running along the curve" } else { "its tangent" }
            )),
        })
    };
    let fault = std::cell::RefCell::new(None);
    let number = of_vals(vals, units);
    let text = substitute_with(&t.body, |w| match bound(w) {
        Some(Ok(c)) => Some(c),
        Some(Err(e)) => {
            fault.borrow_mut().get_or_insert(e);
            None
        }
        None => number(w),
    });
    if let Some(e) = fault.into_inner() {
        return Err(e);
    }
    let parsed = expr::parse_in(&text, units)?;
    if !parsed.body.measures().is_empty() {
        return Err("an integrand reads the point it is integrated at, and measures nothing of \
                    the solved drawing"
            .to_string());
    }
    if let Some(d) = parsed.body.deps().iter().find(|d| !crate::variational::BOUND.contains(&d.as_str())) {
        return Err(format!(
            "`{d}` is no number in scope: an integrand reads the point `{}` it runs along{} and \
             numbers",
            t.point.text,
            t.tangent.as_ref().map_or(String::new(), |n| format!(", its tangent `{}`", n.text)),
        ));
    }
    Ok(text)
}

/// `substitute`, over whatever `of` says a word stands for.
/// Whether a seed's text reads a scalar of the geometry — `k.center.x`, `pin.y`, `base.r` — which
/// is the one kind of name a seed may keep past the flattener (§6.4): a dotted name, since a
/// param and a formal are bare words and an entity's scalar never is.
pub(super) fn reads_geometry(text: &str, units: Units) -> bool {
    expr::parse_in(text, units)
        .map(|p| p.body.deps().iter().any(|d| d.contains('.')))
        .unwrap_or(false)
}

/// Where a measurement call opens and closes when `word`, ending at `i`, names one: its `(`
/// after any space, and its `)` (the text's end where it is unclosed).
fn measure_call(b: &[char], word: &str, i: usize) -> Option<(usize, usize)> {
    if !expr::MEASURES.iter().any(|m| m.0 == word) {
        return None;
    }
    let j = i + b[i..].iter().take_while(|c| c.is_whitespace()).count();
    if b.get(j) != Some(&'(') {
        return None;
    }
    Some((j, b[j..].iter().position(|&c| c == ')').map_or(b.len(), |k| j + k)))
}

/// The text with every measurement's argument names replaced by what `of` says (where it says
/// anything), and nothing else touched: `length(gen_g) / 2` → `length(pair.gen_g) / 2`.
pub(super) fn map_measured(text: &str, of: impl Fn(&str) -> Option<String>) -> String {
    let b: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    let mut i = 0usize;
    while i < b.len() {
        let word_start = b[i].is_alphabetic() || b[i] == '_';
        let prev_ident = i > 0 && (b[i - 1].is_alphanumeric() || b[i - 1] == '_' || b[i - 1] == '.');
        if word_start && !prev_ident {
            let from = i;
            while i < b.len() && (b[i].is_alphanumeric() || b[i] == '_') {
                i += 1;
            }
            let word: String = b[from..i].iter().collect();
            out.push_str(&word);
            if let Some((j, close)) = measure_call(&b, &word, i) {
                let inner: String = b[j + 1..close].iter().collect();
                let args: Vec<String> = inner
                    .split(',')
                    .map(|a| a.trim())
                    .map(|a| of(a).unwrap_or_else(|| a.to_string()))
                    .collect();
                out.push('(');
                out.push_str(&args.join(", "));
                if close < b.len() {
                    out.push(')');
                }
                i = (close + 1).min(b.len());
            }
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    out
}

/// `text` with every name `of` answers for written as it says — at identifier boundaries, a dotted
/// path (`pc.half`) or a block copy's key (`#3.0.w`) read as one name, a measurement's arguments
/// left as written.
pub(crate) fn substitute_with(text: &str, of: impl Fn(&str) -> Option<String>) -> String {
    let mut out = String::with_capacity(text.len());
    let b: Vec<char> = text.chars().collect();
    let mut i = 0usize;
    let ident = |c: char| c.is_alphanumeric() || c == '_';
    while i < b.len() {
        // a word is what the expression lexer reads as one name: a dotted path is one word
        // (`c.center.x`, `t1.w`), and so is a block copy's key (`#3.0.w`) — never its head
        // alone, which is what let a second pass find a formal's name inside a name the first
        // had just written
        let key = b[i] == '#' && b.get(i + 1).is_some_and(|&c| ident(c));
        if b[i].is_alphabetic() || b[i] == '_' || key {
            let from = i;
            if key {
                i += 1;
            }
            loop {
                while i < b.len() && ident(b[i]) {
                    i += 1;
                }
                let dotted = i + 1 < b.len() && b[i] == '.' && ident(b[i + 1]);
                if dotted && (key || !b[i + 1].is_ascii_digit()) {
                    i += 1;
                } else {
                    break;
                }
            }
            let word: String = b[from..i].iter().collect();
            // a measurement's arguments are names of geometry, resolved as references are
            // (`rescope_measures`) and never read as numbers: the call is copied as written
            if let Some((_, close)) = measure_call(&b, &word, i) {
                let end = (close + 1).min(b.len());
                out.extend(&b[from..end]);
                i = end;
                continue;
            }
            match of(&word) {
                Some(t) => out.push_str(&t),
                None => out.push_str(&word),
            }
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    out
}

impl<'a> Walk<'a> {
    /// A text with what `substitute` writes in, and — in the symbolic mode — every name that
    /// came to text (`Sym::texts`, looked up through the scope's prefixes like any other name)
    /// written in as well.  A kept text was itself substituted when it was kept, so one pass
    /// is the fixed point.  The identity outside the symbolic mode.
    pub(super) fn subst_sym(&self, text: &str, vals: &BTreeMap<String, Aff>, scope: &Scope) -> String {
        let Some(sym) = &self.sym else { return text.to_string() };
        substitute_with(text, |w| {
            of_vals(vals, self.units)(w).or_else(|| {
                scope.prefixes.iter().find_map(|p| sym.texts.get(&format!("{p}{w}")).cloned())
            })
        })
    }

    /// A dimension's text, settled: the component's numbers written in, and the names that came
    /// to text as well — in one pass, since a second would find the numbers' names inside the
    /// names it had just written.
    ///
    /// The text after the `:=` is `expr.rs`'s language, so this is where a name written in a body
    /// is resolved (§5): a formal or a `param` reads as its number, or as the unknown it is
    /// (`beta`, `leg.theta`) where nothing bound it.  **A name nothing in scope declares is
    /// E101**: an unknown is declared (`param beta: Angle`, a formal), never made by a name
    /// nobody wrote down, which would turn a misspelling into a degree of freedom.  A dotted
    /// name is left for `resolve` (`note_dim_reads`), which tells a circle's radius from an
    /// instance's unknown once every name is known.
    pub(super) fn settle_text(
        &self,
        text: &str,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
    ) -> Result<String, (Code, String)> {
        let reads: BTreeSet<String> =
            expr::parse_in(text, self.units).map(|p| p.body.deps()).unwrap_or_default();
        if let Some(e) = reads.iter().find_map(|name| out_of_reach(name, vals, scope)) {
            return Err((Code::E103, e));
        }
        if let Some(name) = reads.iter().find(|n| self.undeclared(n, vals, scope)) {
            return Err((Code::E101, undefined(name)));
        }
        // a dotted name read in a body (`t.w`, a nested instance's unknown) is made absolute
        // under the instance it is read in; `resolve` judges whether it names anything
        let own = scope.instance_prefix();
        let sub = substitute_with(text, |w| {
            of_vals(vals, self.units)(w).or_else(|| self.kept(w, scope)).or_else(|| {
                let dotted = !own.is_empty() && w.contains('.') && reads.contains(w);
                dotted.then(|| format!("{own}{w}"))
            })
        });
        fold(&sub, text, self.units).map_err(|e| (Code::E103, e))
    }

    /// What a trace kept as text for a name (`Sym::texts`, through the scope's prefixes).
    fn kept(&self, name: &str, scope: &Scope) -> Option<String> {
        let sym = self.sym.as_ref()?;
        scope.prefixes.iter().find_map(|p| sym.texts.get(&format!("{p}{name}")).cloned())
    }

    /// Whether a bare name is one nothing in scope declares — no number or unknown of the scope,
    /// no built-in, no text a trace kept — which is E101 wherever it is read.  A dotted name is
    /// `resolve`'s to judge.
    fn undeclared(&self, name: &str, vals: &BTreeMap<String, Aff>, scope: &Scope) -> bool {
        crate::syntax::is_name(name)
            && !vals.contains_key(name)
            && expr::builtin(name).is_none()
            && self.kept(name, scope).is_none()
    }

    /// A dimension's text settled in place, and its failure said where it is written.  A name
    /// nothing declares refuses the statement (`Walk::refused`), so it never reaches the
    /// expression graph to become an unknown of its own making.
    pub(super) fn settle_dim(
        &mut self,
        text: &mut String,
        span: Span,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
    ) {
        self.note_dim_reads(text, span, vals, scope);
        match self.settle_text(text, vals, scope) {
            Ok(t) => *text = t,
            Err((code, e)) => {
                if code == Code::E101 {
                    self.refused.push(span);
                }
                self.err(code, span, format!("`{text}`: {e}"))
            }
        }
    }

    /// Set aside each dotted name a dimension's text reads that nothing numeric in scope answers
    /// to, as written, for `resolve` to judge (`Walk::dim_reads`).  Not in a trace, whose
    /// dimensions read the geometry's numbers as columns of the curve (§6.5).
    pub(super) fn note_dim_reads(&mut self, text: &str, span: Span, vals: &BTreeMap<String, Aff>, scope: &Scope) {
        if self.sym.is_some() {
            return;
        }
        let Ok(p) = expr::parse_in(text, self.units) else { return };
        for name in p.body.deps() {
            if name.contains('.') && !vals.contains_key(&name) {
                self.dim_reads.push((name, span, scope.clone()));
            }
        }
    }

    /// Keep a name as text — a value nothing here can work out, written in where it is read.
    /// The symbolic mode's one policy; `false` outside it, where the caller reports instead.
    pub(super) fn keep_text(
        &mut self,
        abs: String,
        text: &str,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
    ) -> bool {
        let t = format!("({})", self.subst_sym(text, vals, scope).trim());
        match self.sym.as_mut() {
            Some(sym) => {
                sym.texts.insert(abs, t);
                true
            }
            None => false,
        }
    }

    /// One argument, read over the parameters in scope: a dimension's text settled to text, a
    /// seed or a pin settled to its number.
    ///
    /// **The one walk**, because a statement carries its arguments twice — as the operator was
    /// written and in spec order — and both halves are `syntax::Arg`.  Written twice, a new kind
    /// of argument gets settled in one of them and silently keeps the component's own names in
    /// the other.
    pub(super) fn settle_arg(
        &mut self,
        a: &mut crate::syntax::Arg,
        vals: &BTreeMap<String, Aff>,
        scope: &Scope,
    ) {
        match a {
            crate::syntax::Arg::Dim { text, span } => self.settle_dim(text, *span, vals, scope),
            crate::syntax::Arg::SeedExpr { text, pinned, span } => {
                let bare = text.trim();
                // `t == s` over an unknown — an input nothing binds, a formal left unbound: the
                // contact's parameter *is* that unknown, and every contact pinned to it shares it
                let unknown =
                    vals.get(bare).filter(|a| a.m == 1.0 && a.c == 0.0).and_then(|a| a.free.clone());
                match (value_of(text, vals, self.units), unknown) {
                    (Ok(v), _) => *a = crate::syntax::Arg::Seed { value: v, pinned: *pinned },
                    (Err(_), Some(name)) if *pinned => {
                        let seed = self.unknowns.get(&name).and_then(|d| d.seed);
                        *a = crate::syntax::Arg::Tie { name, seed, span: *span };
                    }
                    (Err(_), _)
                        if out_of_reach(bare, vals, scope).is_none()
                            && self.undeclared(bare, vals, scope) =>
                    {
                        self.refused.push(*span);
                        self.err(Code::E101, *span, undefined(bare))
                    }
                    (Err(e), _) => self.err(Code::E103, *span, format!("`{text}`: {e}")),
                }
            }
            // a selector written with the name of a `Side` formal reads as the word the instance
            // was given (§9.2): `side: s` inside the body, `s: right` at the call.  A
            // word naming no side in scope is left alone — it is one of the slot's own, and the
            // elaborator checks it against the kind's vocabulary.
            crate::syntax::Arg::Word(w) => {
                if let Some(side) = scope.sides.get(w.as_str()) {
                    *w = side.clone();
                }
            }
            _ => {}
        }
    }

    /// Work out every definition a body declares — `w := 60`, `param bore := 50mm`, a group's
    /// members — before any statement of the body is walked (§6.3).
    ///
    /// A body is a set (spec P2): `h := w / 2` may stand above `w := 60`, so the definitions are
    /// taken in *dependency* order and not in line order — a definition is ready when none of the
    /// names it reads is another of this body still waiting, and the ready ones are worked out
    /// until none is left.  What remains then reads itself, through however many others, which is
    /// the cyclic definitional dependency spec §11 names E041.  A second `w` in one body is the E001
    /// a second `w := point` is, and the first stands (#43.13); a definition that fails is reported
    /// once, where it is written, and the values that read it are left unsaid rather than each
    /// repeating the cause (#45.1).
    ///
    /// **An input nothing binds is an unknown** (`param beta: Angle hint(30deg)`): it goes into
    /// `vals` as the free value an unbound formal is, so everything reading it is written in
    /// terms of it, and into `unknowns` with its seed.  Only the document's own: a module's
    /// numbers are read by files that do not draw it.
    pub(super) fn params(
        &mut self,
        body: &[Stmt],
        vals: &mut BTreeMap<String, Aff>,
        scope: &Scope,
    ) {
        let prefix = scope.prefix().to_string();
        let mut pending: Vec<Def> = Vec::new();
        let mut references = Vec::new();
        let mut here: BTreeSet<String> = BTreeSet::new();
        for st in body {
            if let StmtKind::Group(g) = &st.kind {
                let abs = format!("{prefix}{}", g.name.text);
                if !self.group_names.insert(abs.clone()) {
                    self.err(Code::E001, g.name.span, format!("`{}` is declared twice", g.name.text));
                }
                self.names.insert(abs);
                // a group written in place is the outer one's members under its own name, and
                // a group in its own right: `dims.cyl` may be handed on as `dims` may
                let mut members = Vec::new();
                members_of(&g.name.text, &g.fields, &mut members);
                for (name, label, field) in members {
                    if !here.insert(name.clone()) {
                        self.err(Code::E001, label.span, format!("`{name}` is declared twice"));
                        continue;
                    }
                    let text = match &field.value {
                        crate::syntax::InstVal::Ref(r) => {
                            references.push((name.clone(), r.clone()));
                            written(r)
                        }
                        crate::syntax::InstVal::Expr(t) => t.clone(),
                        crate::syntax::InstVal::Hint(..) => {
                            let m = "a `hint(…)` leaves a call's formal unbound and seeds it; \
                                     a group's member is a value";
                            self.err(Code::E103, field.span, m);
                            continue;
                        }
                        crate::syntax::InstVal::Group(_) => {
                            let abs = format!("{prefix}{name}");
                            self.group_names.insert(abs.clone());
                            self.names.insert(abs);
                            continue;
                        }
                    };
                    let group_ref = matches!(field.value, crate::syntax::InstVal::Ref(_));
                    pending.push(Def { name, text, span: field.span, group_ref, ty: None });
                }
                continue;
            }
            let StmtKind::Param(pd) = &st.kind else { continue };
            if !here.insert(pd.name.text.clone()) {
                self.err(Code::E001, pd.name.span, format!("`{}` is declared twice", pd.name.text));
                continue;
            }
            let ty = pd.input.as_ref().and_then(|i| i.ty);
            if let Some(input) = pd.input.as_ref().filter(|_| !pd.bound()) {
                self.unknown(pd, input, vals, scope);
                continue;
            }
            if let Some((_, span)) = pd.input.as_ref().and_then(|i| i.seed.as_ref()) {
                let m = format!("`{}` is stated; a seed is for an unknown, `param {}: Angle`",
                    pd.name.text, pd.name.text);
                self.err(Code::E040, *span, m);
            }
            pending.push(Def {
                name: pd.name.text.clone(),
                text: pd.text.clone(),
                span: pd.span,
                group_ref: false,
                ty,
            });
        }
        // A nested group exposes the selected group's numeric members, including forward
        // references. Keep ordinary dependency ordering for those members too.
        for depth in 0..MAX_DEPTH {
            let keys: BTreeSet<String> = vals.keys().cloned().chain(pending.iter().map(|d| d.name.clone())).collect();
            let mut extra = Vec::new();
            for d in &pending {
                if !d.group_ref { continue; }
                let start = format!("{}.", d.text);
                for key in &keys {
                    let Some(member) = key.strip_prefix(&start) else { continue };
                    let name = format!("{}.{member}", d.name);
                    if here.insert(name.clone()) {
                        if depth + 1 == MAX_DEPTH || name.split('.').count() > MAX_DEPTH {
                            self.err(Code::E041, d.span, "group nesting is cyclic or too deep");
                            return;
                        }
                        if pending.len() + extra.len() >= MAX_FLAT {
                            self.err(Code::E103, d.span, "group expansion is too large");
                            return;
                        }
                        extra.push(Def { name, text: key.clone(), span: d.span, group_ref: true,
                            ty: None });
                    }
                }
            }
            if extra.is_empty() { break; }
            pending.extend(extra);
        }
        // the names each definition reads; a text that does not parse reads nothing, and is
        // worked out at once so the parse error is the one reported
        let reads: Vec<BTreeSet<String>> = pending
            .iter()
            .map(|d| {
                expr::parse_in(d.text.trim(), self.units).map(|p| p.body.deps()).unwrap_or_default()
            })
            .collect();
        let mut waiting: Vec<usize> = (0..pending.len()).collect();
        let mut failed: BTreeSet<String> = BTreeSet::new();
        loop {
            if waiting.is_empty() { break; }
            let names: BTreeSet<&str> = waiting.iter().map(|&i| pending[i].name.as_str()).collect();
            let (ready, rest): (Vec<usize>, Vec<usize>) = waiting
                .iter()
                .partition(|&&i| !reads[i].iter().any(|n| names.contains(n.as_str())));
            if ready.is_empty() {
                for i in rest {
                    let d = &pending[i];
                    let through = reads[i]
                        .iter()
                        .find(|n| names.contains(n.as_str()) && **n != d.name)
                        .map(|n| format!(", through `{n}`"))
                        .unwrap_or_default();
                    self.err(
                        Code::E041,
                        d.span,
                        format!("`{}` is defined in terms of itself{through}", d.name),
                    );
                }
                return;
            }
            for i in ready {
                let d = &pending[i];
                if reads[i].iter().any(|n| failed.contains(n)) {
                    failed.insert(d.name.clone());
                    continue; // the cause is already reported, at the definition it reads
                }
                // a declared type is what the value is, as a formal's is: `param w: Length := 60`
                // is a length however the 60 was written, and `param w: Length := 60deg` is wrong
                let worked = value_aff(&d.text, vals, self.units).and_then(|a| match d.ty {
                    Some(ty) => typed(a, ty, &d.name),
                    None => Ok(a),
                });
                match worked {
                    Ok(a) => {
                        vals.insert(d.name.clone(), a);
                    }
                    // References to geometry and groups bind as aliases, not numeric values.
                    Err(_) if d.group_ref => {}
                    // a text a curve's variables leave no value to — kept, in the symbolic
                    // mode; a mistake, on the sheet
                    Err(e) => {
                        let abs = format!("{prefix}{}", d.name);
                        let (text, name, span) = (d.text.clone(), d.name.clone(), d.span);
                        if !self.keep_text(abs, &text, vals, scope) {
                            self.err(Code::E103, span, format!("`{name}`: {e}"));
                            failed.insert(name);
                        }
                    }
                }
            }
            waiting = rest;
        }
        for (name, reference) in references {
            if vals.contains_key(&name) { continue; }
            let abs = format!("{prefix}{name}");
            self.group_fields.push((abs.clone(), reference.span));
            self.aliases.push((abs, reference, Scope { vals: vals.clone(), ..scope.clone() }));
        }
    }

    /// `param beta: Angle hint(30deg)` — an input nothing binds: a free value in `vals`, named
    /// as written (the document's root has no prefix), and an entry in `unknowns`.
    fn unknown(
        &mut self,
        pd: &crate::syntax::ParamDecl,
        input: &crate::syntax::Input,
        vals: &mut BTreeMap<String, Aff>,
        scope: &Scope,
    ) {
        let name = &pd.name.text;
        if scope.module.is_some() || !scope.prefix().is_empty() {
            let m =
                format!("`{name}` has no value: an unknown belongs to the document that draws it");
            self.err(Code::E040, pd.name.span, m);
            return;
        }
        let ty = match input.ty {
            Some(ty @ (Ty::Length | Ty::Angle | Ty::Scalar)) => ty,
            Some(_) => {
                let m = format!("`{name}` is an unknown, so a `Length`, an `Angle` or a `Scalar`");
                self.err(Code::E040, pd.name.span, m);
                return;
            }
            None => {
                let m = format!(
                    "`{name}` has no value, so it is an unknown: say what it is, \
                     `param {name}: Length` or `param {name}: Angle`"
                );
                self.err(Code::E040, pd.name.span, m);
                return;
            }
        };
        let seed = input.seed.as_ref().and_then(|(text, span)| {
            match self.seed_number(text, ty, vals) {
                Ok(v) => Some(v),
                Err(e) => {
                    self.err(Code::E103, *span, format!("`{text}`: {e}"));
                    None
                }
            }
        });
        vals.insert(name.clone(), free(name.clone(), ty));
        self.unknowns.insert(name.clone(), crate::model::Declared { dim: ty.dim(), seed });
    }

    /// A seed for an unknown of type `ty`: a number, of that dimension, in the document's units.
    pub(super) fn seed_number(
        &self,
        text: &str,
        ty: Ty,
        vals: &BTreeMap<String, Aff>,
    ) -> Result<f64, String> {
        let a = value_aff(text, vals, self.units)?;
        a.dim.require(ty.dim(), "a seed")?;
        a.number().ok_or_else(|| "a seed is a number".to_string())
    }
}

/// E101's words for a name nothing declares.
pub(super) fn undefined(name: &str) -> String {
    if name.contains('.') {
        return format!("`{name}` is not defined: nothing by that name was drawn or declared");
    }
    format!(
        "`{name}` is not defined: a number is `{name} := …`, and an unknown the solve answers for \
         is declared, `param {name}: Length`"
    )
}
