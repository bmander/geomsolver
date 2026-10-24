//! Scoped scalar definitions, dimensional evaluation and text substitution.

use super::*;
use crate::expr::{Ast, Cmp, Cond};

/// The drawing's unknown a declared number stands for when nothing binds it — a formal no call
/// binds, or an input with no value (`param beta: Angle`) — under its declared dimension.
pub(super) fn free(name: String, ty: Ty) -> Aff {
    Aff::free(name, ty.dim())
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
/// handed as an argument (E103), or a group member the argument does not have, or its
/// interface does not declare (E101) — or `None`.  Asked of a dimension's text (`settle_text`)
/// and of a pin to a name (`t == s`) alike.
fn out_of_reach(name: &str, vals: &Vals, scope: &Scope) -> Option<(Code, String)> {
    if vals.contains_key(name) {
        return None;
    }
    if scope.forbidden.contains(name) {
        // a constant of the file is read (#461 PC1): what is out of reach is an unknown, or a
        // number of the caller's alone
        if scope.forbidden.unknown(name) {
            return Some((Code::E103, format!("`{name}` is an unknown of the drawing, and a \
                definition reads only constants: pass it as an argument")));
        }
        return Some((Code::E103, format!("`{name}` is outside this component; pass it as an argument")));
    }
    if name.split_once('.').is_some_and(|(head, _)| scope.groups.contains(head)) {
        return Some((Code::E101, format!("no numeric member `{name}` in the group argument")));
    }
    None
}

/// What a `when` asks the walk about names that are not numbers (#267): what one stands for,
/// as one absolute name, and the faces one names.
pub(super) trait Names {
    fn stands(&self, name: &str) -> Option<String>;
    fn faces(&self, name: &str) -> Option<Vec<String>>;
    /// Whether two faces, each by what its name stands for, share a point (#625): `None` where
    /// either is no face a solid lists.
    fn meets(&self, a: &str, b: &str) -> Option<bool>;
    /// Whether the unknown `name` is a face family's running number (#243 L5).
    fn running(&self, name: &str) -> bool;
}

/// Whether a `when`'s condition holds over the numbers in scope (#210), or the code and the
/// words for why it cannot be decided: a name nothing in scope gives a number is E101, one that
/// is or reads an unknown of the drawing E040 (the block is made or not before there is a solve
/// to read), two dimensions or a number that is no comparison E103, a measurement E107.
///
/// A name that is no number may still be compared for **identity** (#267): `a != b` holds where
/// the two stand for different things (`Names::stands`), `c in B` where what `c` stands for is
/// one of the faces `B` names, `a meets b` where two faces share a point (#625, `Names::meets`).
/// A number on one side and geometry on the other is E103.
pub(super) fn condition(
    text: &str,
    env: &Vals,
    scope: &Scope,
    units: Units,
    names: &dyn Names,
) -> Result<bool, (Code, String)> {
    let said = |code, e: String| (code, format!("`{text}`: {e}"));
    let c = expr::parse_condition_in(text, units).map_err(|e| said(Code::E103, e))?;
    holds(&c, env, scope, names).map_err(|(code, e)| said(code, e))
}

/// One condition of a `when`, decided: identities by what names stand for, comparisons of
/// numbers by `expr::decide`.  Both sides of an `and` or an `or` are worked out, so a mistake a
/// short circuit would skip is still said.
fn holds(
    c: &Cond,
    env: &Vals,
    scope: &Scope,
    names: &dyn Names,
) -> Result<bool, (Code, String)> {
    let nothing = |name: &str| {
        out_of_reach(name, env, scope).unwrap_or_else(|| (Code::E101, format!(
            "`{name}` is not a number here, nor anything drawn — nothing in scope gives it one"
        )))
    };
    let stands = |name: &str| names.stands(name).ok_or_else(|| nothing(name));
    Ok(match c {
        Cond::And(a, b) => {
            let (p, q) = (holds(a, env, scope, names), holds(b, env, scope, names));
            p? && q?
        }
        Cond::Or(a, b) => {
            let (p, q) = (holds(a, env, scope, names), holds(b, env, scope, names));
            p? || q?
        }
        Cond::Not(a) => !holds(a, env, scope, names)?,
        Cond::In(x, y) => {
            let me = stands(x)?;
            let Some(faces) = names.faces(y) else {
                if names.stands(y).is_none() && !env.contains_key(y.as_str()) {
                    return Err(nothing(y));
                }
                return Err((Code::E103, format!(
                    "`{y}` names no faces: `{x} in {y}` asks whether `{x}` is one of a solid's \
                     faces, as `B` is in `A round(r) B`"
                )));
            };
            faces.contains(&me)
        }
        Cond::Meets(x, y) => {
            let (p, q) = (stands(x)?, stands(y)?);
            names.meets(&p, &q).ok_or_else(|| (Code::E103, format!(
                "`{x} meets {y}` asks whether two faces of a solid share a point, and `{}` is \
                 no face: name a face a `repeat … in` runs over",
                if names.faces(x).is_none() { x } else { y }
            )))?
        }
        Cond::Cmp(op @ (Cmp::Eq | Cmp::Ne), Ast::Var(x), Ast::Var(y))
            if !env.contains_key(x.as_str()) || !env.contains_key(y.as_str()) =>
        {
            if let Some(n) = [x, y].into_iter().find(|n| env.contains_key(n.as_str())) {
                let other = if n == x { y } else { x };
                stands(other)?;
                return Err((Code::E103, format!(
                    "`{n}` is a number and `{other}` is drawn: `{}` compares two numbers, or two \
                     names for what they stand for",
                    op.text()
                )));
            }
            (stands(x)? == stands(y)?) == (*op == Cmp::Eq)
        }
        Cond::Cmp(..) => {
            for name in c.deps() {
                match env.get(&name).map(|a| a.free.as_deref()) {
                    None if names.stands(&name).is_some() => {
                        return Err((Code::E103, format!(
                            "`{name}` is drawn, not a number: a drawn thing is compared with \
                             another by `==` or `!=`, or put among faces by `in`"
                        )));
                    }
                    None => return Err(nothing(&name)),
                    Some(Some(u)) => {
                        // a law of the station is an expression in its running number; the
                        // station itself, what is made of it, is one shape over the whole run
                        if names.running(u) {
                            // the formal by its own name, never the station's key for it
                            let mine = u.rsplit('.').next() == Some(name.as_str());
                            let what = if mine { "is" } else { "reads" };
                            return Err((Code::E040, format!(
                                "`{name}` {what} a face family's running number, and a `when` is \
                                 made once, for the whole run: a law that changes with it is one \
                                 expression (`min`, `max` and `abs` say one in pieces)"
                            )));
                        }
                        return Err((Code::E040, format!(
                            "{} an unknown of the drawing, and a `when` is decided before the \
                             solve: compare numbers the document states",
                            reads_unknown(&name, u)
                        )));
                    }
                    Some(None) => {}
                }
            }
            expr::decide_in(c, env).map_err(|e| {
                let code = if e.fault == expr::Fault::Measure { Code::E107 } else { Code::E103 };
                (code, e.message)
            })?
        }
    })
}

/// How a name that is, or is written over, the unknown `u` is said to be one: `` `beta` is `` or
/// `` `k` reads `beta`, ``, ahead of what an unknown cannot be read by.
fn reads_unknown(name: &str, u: &str) -> String {
    if u == name { format!("`{name}` is") } else { format!("`{name}` reads `{u}`,") }
}

/// Whether a claim over values holds (#596), and each name it reads with its value; or the code
/// and the words for why it cannot be decided — a name that is no number here E101, one that is
/// or reads an unknown of the drawing E040 (a value is worked out before the solve), two
/// dimensions compared or a text that is no comparison E103, a measurement E107.
fn claimed(
    text: &str,
    env: &Vals,
    scope: &Scope,
    units: Units,
) -> Result<(bool, Vec<(String, super::Valued)>), (Code, String)> {
    let said = |code, e: String| (code, format!("`{text}`: {e}"));
    let c = expr::parse_condition_in(text, units).map_err(|e| said(Code::E103, e))?;
    fn membership(c: &Cond) -> bool {
        match c {
            Cond::In(..) | Cond::Meets(..) => true,
            Cond::And(a, b) | Cond::Or(a, b) => membership(a) || membership(b),
            Cond::Not(a) => membership(a),
            Cond::Cmp(..) => false,
        }
    }
    if membership(&c) {
        return Err(said(Code::E103, "a claim over values compares numbers; `in` and `meets` \
            ask of faces, and are a `when`'s".into()));
    }
    let mut reads = Vec::new();
    for name in c.deps() {
        match env.get(&name) {
            None => {
                let (code, e) = out_of_reach(&name, env, scope).unwrap_or_else(|| (Code::E101,
                    format!("`{name}` is not a number here: a claim over values compares the \
                             numbers the document names")));
                return Err(said(code, e));
            }
            Some(Aff { free: Some(u), .. }) => {
                return Err(said(Code::E040, format!(
                    "{} an unknown of the drawing, and a claim over values is judged before the \
                     solve: a relation claims what the solve gives",
                    reads_unknown(&name, u)
                )));
            }
            Some(a) => reads.push((name, super::Valued { value: a.c, dim: a.dim })),
        }
    }
    let holds = expr::judge_in(&c, env).map_err(|e| {
        let code = if e.fault == expr::Fault::Measure { Code::E107 } else { Code::E103 };
        said(code, e.message)
    })?;
    Ok((holds, reads))
}

/// A number worked out while elaborating.
///
/// The language's angle is the degree — `sin(90)` is 1, and every dimension a person reads is in
/// degrees — so a whole turn is `tau = 360`, not 2π.  Keeping `tau` at 2π and the texts in degrees
/// would give two units one name; units say which of the two each is (`expr::CONSTANTS`), so
/// `pi` is the dimensionless constant and `tau` is an **angle**, and `tau == 2 * pi * 1rad`
/// holds where it used to be a coincidence of the same digits.
pub(super) fn value_of<T: Table + ?Sized>(text: &str, env: &T, units: Units) -> Result<f64, String> {
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
pub(crate) fn value_aff<T: Table + ?Sized>(
    text: &str,
    env: &T,
    units: Units,
) -> Result<Aff, String> {
    value_read(text, env, &BTreeMap::new(), units)
}

/// `value_aff`, where the text may be one a scope wrote over an unknown of the drawing (`(0.3 * m
/// + 0.05mm)`, `of_vals`) and is read in a scope that binds none: each unknown it reads is what
/// its declaration says, as it was where the text was written (`expr::declared_reads`).
pub(crate) fn value_read<T: Table + ?Sized>(
    text: &str,
    env: &T,
    unknowns: &BTreeMap<String, crate::model::Declared>,
    units: Units,
) -> Result<Aff, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("nothing to work out".to_string());
    }
    let p = expr::parse_in(t, units)?;
    // each declared unknown it reads and `env` does not bind, read before `env` rather than
    // added to a copy of it
    let declared = expr::declared_reads(&p.body, |name| env.has(name), unknowns);
    // a group read as a number — `dims.handle_dims` of a module — is said to be one, rather
    // than as a member its owner lacks or a number of no dimension (#594): asked only of a text
    // that does not come to a number
    let group = || p.body.deps().into_iter().filter(|n| !env.has(n) && !declared.contains_key(n))
        .find_map(|n| env.under(&format!("{n}.")).into_keys().next().map(|one| format!(
            "`{n}` is a group, and a group is not a number: read one of its members, as `{one}`")));
    let a = expr::eval_in(&p.body, &Over { own: &declared, env })
        .map_err(|e| group().unwrap_or_else(|| e.to_string()))?;
    match (a.number(), &a.free) {
        (Some(v), _) if !v.is_finite() => Err(format!("comes to {v}")),
        // an unknown the scope *bound* — a formal left unbound, a param over one — carries on;
        // a name nothing binds is the document's, and a component's numbers cannot read it
        (None, Some(n)) if !declared.contains_key(n) && !env.binds(n) => {
            if let Some(e) = group() {
                return Err(e);
            }
            // a member a group or a vector does not have — `o.z` of `o := (1, 2)` — is said so
            if let Some((owner, member)) = n.rsplit_once('.') {
                let prefix = format!("{owner}.");
                let mut under = env.under(&prefix);
                under.extend(declared.under(&prefix));
                let has: Vec<&str> = under
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

/// A few names of its own, read before a table: the declared unknowns `value_read` adds.
struct Over<'a, T: ?Sized> {
    own: &'a BTreeMap<String, Aff>,
    env: &'a T,
}

impl<T: Table + ?Sized> expr::Env for Over<'_, T> {
    fn value(&self, name: &str) -> Option<Aff> {
        self.own.get(name).cloned().or_else(|| self.env.value(name))
    }
}

/// A dotted name's first segment (`dims` of `dims.handle_dims`).
fn head(name: &str) -> &str {
    name.split_once('.').map_or(name, |(h, _)| h)
}

/// Whether a definition's text is a name and nothing else (`sizes`, `dims.handle_dims`).
fn is_reference(text: &str) -> bool {
    text.trim().split('.').all(|seg| {
        let mut chars = seg.chars();
        chars.next().is_some_and(|c| c.is_alphabetic() || c == '_')
            && chars.all(|c| c.is_alphanumeric() || c == '_')
    })
}

/// A group's members by their names under `owner` (`dims.bore`), a group written in place
/// listed before its own members (`dims.cyl`, then `dims.cyl.bore`).
pub(super) fn members_of<'a>(
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
/// `held` is what each face family's running number is (`Held`, `of_vals_in`).
pub(super) fn substitute_in<T: Table + ?Sized>(
    text: &str,
    vals: &T,
    units: Units,
    held: &BTreeMap<String, (crate::units::Dim, f64)>,
) -> String {
    substitute_with(text, of_vals_in(vals, units, held))
}

/// No running number held: what every scope outside a face family's station reads.
static NO_HELD: BTreeMap<String, (crate::units::Dim, f64)> = BTreeMap::new();

/// What a word stands for in `vals`, as text: a number, parenthesised; a value affine in the
/// drawing's unknown (a formal left unbound, or a `param` over one) as `(m * name + c)`, or
/// the bare name when that is all it is.  `tau` and `turn` are `expr::CONSTANTS` now, and an
/// angle rather than a number that happens to be 360, so they are left for the evaluator.
fn of_vals<T: Table + ?Sized>(vals: &T, units: Units) -> impl Fn(&str) -> Option<String> + '_ {
    of_vals_in(vals, units, &NO_HELD)
}

/// `of_vals`, knowing what each face family's running number is (`Held`): a value affine in one
/// of another dimension (`24mm − 12mm·s`, `s` a plain number) writes its coefficient with the
/// dimension the two differ by, since the number reads the name as what it is.
fn of_vals_in<'a, T: Table + ?Sized>(
    vals: &'a T,
    units: Units,
    held: &'a BTreeMap<String, (crate::units::Dim, f64)>,
) -> impl Fn(&str) -> Option<String> + 'a {
    move |w| {
        let a = vals.value(w)?;
        if let Some(v) = a.number() {
            // a number that knows what it is says so, or `phi + atan2(…)` would read a plain
            // number added to an angle once `phi: Angle` was written in as one
            return Some(format!("({})", a.dim.number_text(v, units)));
        }
        let n = a.free.as_ref()?;
        if let Some(&(dim, _)) = held.get(n) {
            let k = a.dim.div(dim);
            let m = if k.is_scalar() { crate::syntax::num(a.m) } else { k.number_text(a.m, units) };
            return Some(match (a.m == 1.0 && k.is_scalar(), a.c == 0.0) {
                (true, true) => n.clone(),
                (_, true) => format!("({m} * {n})"),
                _ => format!("({m} * {n} + {})", a.dim.number_text(a.c, units)),
            });
        }
        // the constant is of the whole's dimension, so it says so as a number does above, or
        // `(-0.25 * m + 0)` adds a plain number to a length once `m: Length` is written in
        Some(if a.m == 1.0 && a.c == 0.0 {
            n.clone()
        } else if a.c == 0.0 {
            format!("({} * {n})", crate::syntax::num(a.m))
        } else {
            format!("({} * {n} + {})", crate::syntax::num(a.m), a.dim.number_text(a.c, units))
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
    vals: &Vals,
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

/// Whether a seed's text reads a scalar of the geometry — `k.center.x`, `pin.y`, `base.r` — which
/// is the one kind of name a seed may keep past the flattener (§6.4): a dotted name, since a
/// param and a formal are bare words and an entity's scalar never is.
pub(super) fn reads_geometry(text: &str, units: Units) -> bool {
    expr::parse_in(text, units)
        .map(|p| p.body.deps().iter().any(|d| d.contains('.')))
        .unwrap_or(false)
}

/// How `substitute_with` reads names (`expr::name_at`): the lexer's, but Unicode letters are a
/// name's, a copy's key is read only at a name's head (`side.#282.0.small` is `side` and the key
/// `#282.0.small`), and a name may begin straight after a number's digits (`deg` in `30deg`).
const SUBSTITUTED: expr::NameScan =
    expr::NameScan { unicode: true, keys: true, inner_keys: false, glued: true };


/// `text` with every name `of` answers for written as it says — a name as the expression lexer
/// reads one (`SUBSTITUTED`): a dotted path (`pc.half`) or a block copy's key (`#3.0.w`) is one
/// name, never its head alone, which is what let a second pass find a formal's name inside a
/// name the first had just written — and a measurement's arguments left as written.
pub(crate) fn substitute_with(text: &str, of: impl Fn(&str) -> Option<String>) -> String {
    let b: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    let mut i = 0usize;
    while i < b.len() {
        let Some(end) = expr::name_at(&b, i, SUBSTITUTED) else {
            out.push(b[i]);
            i += 1;
            continue;
        };
        let word: String = b[i..end].iter().collect();
        // a measurement's arguments are names of geometry, never read as numbers: the call is
        // copied as written
        if let Some((_, close)) = expr::measure_call(&b, &word, end) {
            let stop = (close + 1).min(b.len());
            out.extend(&b[i..stop]);
            i = stop;
            continue;
        }
        match of(&word) {
            Some(t) => out.push_str(&t),
            None => out.push_str(&word),
        }
        i = end;
    }
    out
}

impl<'a> Walk<'a> {
    /// A text with what `substitute` writes in, and — in the symbolic mode — every name that
    /// came to text (`Sym::texts`, looked up through the scope's prefixes like any other name)
    /// written in as well.  A kept text was itself substituted when it was kept, so one pass
    /// is the fixed point.  The identity outside the symbolic mode, but where a face family's
    /// station kept texts over its running number (`Held`), which are written in the same way.
    pub(super) fn subst_sym<T: Table + ?Sized>(&self, text: &str, vals: &T, scope: &Scope)
        -> String {
        if self.sym.is_none() && self.held.texts.is_empty() {
            return text.to_string();
        }
        let of = of_vals_in(vals, self.units, &self.held.names);
        substitute_with(text, |w| of(w).or_else(|| self.kept(w, scope)))
    }

    /// A text with the scope's numbers written in, where it then reads a face family's running
    /// number (`Held`).
    pub(super) fn held_text(&self, text: &str, vals: &Vals, scope: &Scope)
        -> Option<String> {
        if self.held.names.is_empty() {
            return None;
        }
        let of = of_vals_in(vals, self.units, &self.held.names);
        let sub = substitute_with(text, |w| of(w).or_else(|| self.kept(w, scope)));
        expr::parse_in(&sub, self.units)
            .is_ok_and(|p| p.body.deps().iter().any(|n| self.held.names.contains_key(n)))
            .then_some(sub)
    }

    /// A text with the scope's numbers written in, where it reads the geometry a face family's
    /// station was given (`a.x`, `e.p1.y`) and stands in the station: each such number is held
    /// where that geometry stands at each station (`HeldNumber::reads`, #243 L1), so the text is
    /// worked out there whatever it does with them — a product of two, a quotient — as a law in
    /// the running number is (#243 L8).  `None` outside a station, or for a text reading none.
    ///
    /// Each geometry read is made absolute and set aside here, where the number is written and
    /// its names mean what they say (`Checks::dim_reads`): the text is settled again where it is
    /// read, a word's body or an instance's, which sees none of them.
    pub(super) fn station_text(
        &mut self,
        text: &str,
        span: Span,
        vals: &Vals,
        scope: &Scope,
    ) -> Option<String> {
        let station = self.held.names.keys()
            .any(|h| scope.instance_prefix().starts_with(crate::model::station_prefix(h).as_str()));
        if !station || !reads_geometry(text, self.units) {
            return None;
        }
        let own = scope.prefix();
        let of = of_vals_in(vals, self.units, &self.held.names);
        // the dotted names nothing numeric here answers: the geometry read, each keyed under
        // the copy reading it
        let geometry: BTreeMap<String, String> = expr::parse_in(text, self.units).ok()?.body
            .deps().into_iter()
            .filter(|w| w.contains('.') && of(w).is_none() && self.kept(w, scope).is_none())
            .map(|w| (w.clone(), format!("{own}{w}")))
            .collect();
        let sub = substitute_with(text, |w| {
            of(w).or_else(|| self.kept(w, scope)).or_else(|| geometry.get(w).cloned())
        });
        for (w, key) in geometry {
            self.checks.dim_reads.push((w, span, scope.clone(), Some(key)));
        }
        let names = expr::parse_in(&sub, self.units).ok()?.body.deps();
        self.checks.settled.extend(names.into_iter().filter(|n| n.contains('.')));
        Some(sub)
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
        vals: &Vals,
        scope: &Scope,
    ) -> Result<String, (Code, String)> {
        let reads: BTreeSet<String> =
            expr::parse_in(text, self.units).map(|p| p.body.deps()).unwrap_or_default();
        // a text kept for a name answers for it, though the scope is closed to the name outside
        let reach = |name: &String| self.kept(name, scope).is_none()
            .then(|| out_of_reach(name, vals, scope)).flatten();
        if let Some(e) = reads.iter().find_map(reach) {
            return Err(e);
        }
        // a formal given a text over a face family's running number is that text, whatever the
        // document declares by its name (#398 B)
        let runs = |n: &&String| self.runs(n, vals) && self.kept(n, scope).is_none();
        if let Some(name) = reads.iter().find(runs) {
            return Err((Code::E103, runs_over(name)));
        }
        if let Some(name) = reads.iter().find(|n| self.undeclared(n, vals, scope)) {
            return Err((Code::E101, undefined(name)));
        }
        // a dotted name read in a body (`t.w`, a nested instance's unknown) is made absolute
        // under the instance it is read in; `resolve` judges whether it names anything
        let own = scope.instance_prefix();
        let of = of_vals_in(vals, self.units, &self.held.names);
        let sub = substitute_with(text, |w| {
            of(w).or_else(|| self.kept(w, scope)).or_else(|| {
                let dotted = !own.is_empty() && w.contains('.') && reads.contains(w)
                    && !self.checks.settled.contains(w);
                dotted.then(|| format!("{own}{w}"))
            })
        });
        fold(&sub, text, self.units).map_err(|e| (Code::E103, e))
    }

    /// What a trace kept as text for a name (`Sym::texts`, through the scope's prefixes).
    fn kept(&self, name: &str, scope: &Scope) -> Option<String> {
        let texts = match &self.sym {
            Some(sym) => &sym.texts,
            None => &self.held.texts,
        };
        scope.prefixes.iter().find_map(|p| texts.get(&format!("{p}{name}")).cloned())
    }

    /// Whether a bare name is a parameter axis the document declares (#391 S1) and nothing in
    /// scope gives a number — read as a number, which it is not.
    fn runs(&self, name: &str, vals: &Vals) -> bool {
        self.parameters.contains_key(name) && !vals.contains_key(name)
    }

    /// Whether a bare name is one nothing in scope declares — no number or unknown of the scope,
    /// no built-in, no text a trace kept — which is E101 wherever it is read.  A dotted name is
    /// `resolve`'s to judge.
    fn undeclared(&self, name: &str, vals: &Vals, scope: &Scope) -> bool {
        crate::syntax::is_name(name)
            && !vals.contains_key(name)
            && expr::builtin(name).is_none()
            && self.kept(name, scope).is_none()
    }

    /// A dimension's text settled in place, and its failure said where it is written.  A name
    /// nothing declares refuses the statement (`Checks::refused`), so it never reaches the
    /// expression graph to become an unknown of its own making.
    pub(super) fn settle_dim(
        &mut self,
        text: &mut String,
        span: Span,
        vals: &Vals,
        scope: &Scope,
    ) {
        // what it reads is judged once names are known only where the text settles: one it
        // cannot is said here, once
        match self.settle_text(text, vals, scope) {
            Ok(t) => {
                self.note_dim_reads(text, span, vals, scope);
                *text = t;
            }
            Err((code, e)) => {
                // a parameter axis read as a number is refused as a name nothing declares is,
                // so it never becomes an unknown of its own making
                let axis = || expr::parse_in(text, self.units)
                    .is_ok_and(|p| p.body.deps().iter().any(|n| self.runs(n, vals)));
                if code == Code::E101 || axis() {
                    self.checks.refused.push(span);
                }
                self.err(code, span, format!("`{text}`: {e}"))
            }
        }
    }

    /// Set aside each dotted name a dimension's text reads that nothing numeric in scope answers
    /// to, as written, for `resolve` to judge (`Checks::dim_reads`).  Not in a trace, whose
    /// dimensions read the geometry's numbers as columns of the curve (§6.5).
    pub(super) fn note_dim_reads(&mut self, text: &str, span: Span, vals: &Vals, scope: &Scope) {
        if self.sym.is_some() {
            return;
        }
        let Ok(p) = expr::parse_in(text, self.units) else { return };
        for name in p.body.deps() {
            // a face family's running number, written in as a station's text (#243 L5), is no
            // geometry's
            let held = self.held.names.contains_key(&name) || self.checks.settled.contains(&name);
            if name.contains('.') && !vals.contains_key(&name) && !held {
                self.checks.dim_reads.push((name, span, scope.clone(), None));
            }
        }
    }

    /// Keep a name as text — a value nothing here can work out, written in where it is read.
    /// The symbolic mode's one policy; `false` outside it, where the caller reports instead.
    pub(super) fn keep_text(
        &mut self,
        abs: String,
        text: &str,
        span: Span,
        vals: &Vals,
        scope: &Scope,
    ) -> bool {
        if self.sym.is_some() {
            let t = format!("({})", self.subst_sym(text, vals, scope).trim());
            if let Some(sym) = self.sym.as_mut() {
                sym.texts.insert(abs, t);
            }
            return true;
        }
        // on the sheet, a text over a face family's running number that is not affine in it, or
        // over the geometry a station was given: worked out at each station, so kept as text
        // (#243 L1, L8)
        let kept = self.held_text(text, vals, scope)
            .or_else(|| self.station_text(text, span, vals, scope));
        let Some(t) = kept else { return false };
        self.held.texts.insert(abs, format!("({})", t.trim()));
        true
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
        vals: &Vals,
        scope: &Scope,
    ) {
        match a {
            crate::syntax::Arg::Dim { text, span } => self.settle_dim(text, *span, vals, scope),
            crate::syntax::Arg::SeedExpr { text, pinned, span } => {
                let bare = text.trim();
                // `t == s` over an unknown — an input nothing binds, a formal left unbound: the
                // contact's parameter *is* that unknown, and every contact pinned to it shares it
                // — and over a chain, `s` on the link a leg stands on is that leg's own number,
                // `k` short of the chain's (#243 L4)
                let on_leg = |a: &Aff| self.held.on_leg.as_ref()
                    .is_some_and(|(h, k)| a.free.as_ref() == Some(h) && a.c == *k);
                let unknown = vals.get(bare).filter(|a| a.m == 1.0 && (a.c == 0.0 || on_leg(a)))
                    .and_then(|a| a.free.clone());
                // a face family's running number is no unknown and no place: the contact is
                // pinned to it, at whatever station the family stands (#243 L1)
                let held = unknown.as_ref().filter(|n| self.held.names.contains_key(*n)).cloned();
                if let (Some(name), true) = (held, *pinned) {
                    *a = crate::syntax::Arg::Tie { name, seed: None, span: *span };
                    return;
                }
                match (value_of(text, vals, self.units), unknown) {
                    (Ok(v), _) => *a = crate::syntax::Arg::Seed { value: v, pinned: *pinned },
                    (Err(_), Some(name)) if *pinned => {
                        let seed = self.unknowns.get(&name).and_then(|d| d.seed);
                        *a = crate::syntax::Arg::Tie { name, seed, span: *span };
                    }
                    // a pin over the swept formal in a trace (`t == u * u`, #248) is a place
                    // the block computes as it runs, kept as text as a dimension's is, for the
                    // tape the trace reads it through
                    (Err(_), _) if *pinned && self.sym.is_some() => {
                        match self.settle_text(text, vals, scope) {
                            Ok(t) => *text = t,
                            Err((code, e)) => self.err(code, *span, format!("`{text}`: {e}")),
                        }
                    }
                    (Err(_), _) if self.held_text(text, vals, scope).is_some() => {
                        // a leg's `s` is the chain's, which only a link of a chain the family
                        // runs reads as a place of its run
                        let leg = vals.get(bare).and_then(|a| a.free.as_ref())
                            .and_then(|n| self.held.legs.get(n));
                        let m = match leg {
                            Some(&(k, _)) => format!("`{text}`: the family runs over a chain, and \
                                on its link {k} `s` runs from {k}: a contact pinned to `s` \
                                stands on a chain it runs"),
                            None => format!("`{text}`: a contact in a face family is pinned to \
                                its running number itself, `t == s`, or to a number"),
                        };
                        self.err(Code::E103, *span, m)
                    }
                    (Err(_), _)
                        if out_of_reach(bare, vals, scope).is_none()
                            && self.undeclared(bare, vals, scope) =>
                    {
                        self.checks.refused.push(*span);
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
        vals: &mut Vals,
        scope: &Scope,
    ) {
        let prefix = scope.prefix().to_string();
        let (mut pending, references, mut here) = self.definitions(body, vals, scope, &prefix);
        if !self.nested_members(&mut pending, vals, &mut here) {
            return;
        }
        // the parameters the body declares, which no value reads (#391 S4)
        let running: BTreeSet<String> = body.iter().filter_map(|st| match &st.kind {
            StmtKind::Param(pd) if pd.over().is_some() => Some(pd.name.text.clone()),
            _ => None,
        }).collect();
        if !self.work_out(&pending, vals, scope, &prefix, &running) {
            return;
        }
        self.named_values(&pending, vals, &prefix);
        // a parameter axis's interval may read any number the body defines (#391 S1), and a
        // material's properties are read as one once they are worked out (#510 P3)
        for st in body {
            match &st.kind {
                StmtKind::Param(pd) => {
                    if let Some(over) = pd.over() {
                        self.parameter_axis(pd, over, vals, scope);
                    }
                }
                StmtKind::Group(g) if g.material.is_some() => self.material(g, vals, &prefix),
                _ => {}
            }
        }
        let references: Vec<_> =
            references.into_iter().filter(|(name, _, _)| !vals.contains_key(name)).collect();
        if references.is_empty() {
            return;
        }
        // one table for every alias the body makes, shared; a file's group named by the key
        // the file holds it under, read from nowhere in particular (`file_group`)
        let at = Scope { vals: std::rc::Rc::new(vals.clone()), ..scope.clone() };
        let root = Scope::absolute(scope.access.clone());
        for (name, reference, absolute) in self.named_first(references, &prefix) {
            let abs = format!("{prefix}{name}");
            self.checks.group_fields.push((abs.clone(), reference.span));
            let read = if absolute { &root } else { &at };
            self.aliases.push((abs, reference, read.clone()));
        }
    }

    /// A body's references ordered so a name for a group comes after the name it is for, where
    /// that is another of them (`tiny := smaller` above `smaller := small`): the alias table reads
    /// its aliases in the order they were made.  A cycle keeps its order, and is said elsewhere.
    fn named_first(&self, references: Vec<(String, Ref, bool)>, prefix: &str)
        -> Vec<(String, Ref, bool)> {
        let mut ordered = Vec::with_capacity(references.len());
        let mut left = references;
        while !left.is_empty() {
            let pending: BTreeSet<String> =
                left.iter().map(|(n, _, _)| format!("{prefix}{n}")).collect();
            let waits = |to: &String| pending.iter().any(|p| {
                to == p || to.strip_prefix(p.as_str()).is_some_and(|r| r.starts_with('.'))
            });
            let (ready, rest): (Vec<_>, Vec<_>) = left.into_iter().partition(|(n, _, _)| {
                self.group_aliases.get(&format!("{prefix}{n}")).is_none_or(|to| !waits(to))
            });
            if ready.is_empty() {
                ordered.extend(rest);
                break;
            }
            ordered.extend(ready);
            left = rest;
        }
        ordered
    }

    /// The group of the file a definition's body is written in that `text` names, by the key
    /// the walk holds it under, where nothing nearer does (#594): a body reads its file's
    /// constants, and a group of numbers alone among them is handed on as the file's root hands
    /// it — the file's own (`sizes`, or `{module}.sizes` in a module), a used module's by its
    /// path (`dims.handle_dims`), or one a `use` imports bare.  A formal, or a name the body
    /// declares, hides it; a group carrying geometry is the file's drawing, out of the body's
    /// reach as the drawing is.
    pub(super) fn file_group(&self, text: &str, scope: &Scope) -> Option<String> {
        self.file_group_any(text, scope).filter(|k| !self.carries_geometry(k, 0))
    }

    /// `file_group`, geometry or none.
    pub(super) fn file_group_any(&self, text: &str, scope: &Scope) -> Option<String> {
        if !scope.closed {
            return None;
        }
        let root = head(text);
        let near = |k: String| self.names.contains(&k) || self.group_known(&k);
        if scope.groups.contains(root) || scope.prefixes.iter().any(|p| near(format!("{p}{root}")))
        {
            return None;
        }
        let module = |k: usize| format!("{}.{text}", self.prog.modules[k].name);
        let own = scope.module.map_or_else(|| text.to_string(), module);
        let imported = self.prog.imported(root, scope.module).map(module);
        [Some(own), Some(text.to_string()), imported].into_iter().flatten()
            .find(|k| self.defined_groups.contains(k) || self.group_aliases.contains_key(k))
    }

    /// Whether `key` is a group the walk knows, or a name for one.
    fn group_known(&self, key: &str) -> bool {
        self.group_names.contains(key) || self.group_aliases.contains_key(key)
    }

    /// Whether the group `key` names geometry, itself or through a group it names.
    pub(super) fn carries_geometry(&self, key: &str, depth: usize) -> bool {
        if depth > MAX_DEPTH {
            return true;
        }
        if let Some(to) = self.group_aliases.get(key) {
            return self.carries_geometry(to, depth + 1);
        }
        let under = format!("{key}.");
        self.aliases.iter().filter(|(a, _, _)| a.starts_with(&under)).any(|(a, _, _)| {
            !self.group_aliases.contains_key(a) || self.carries_geometry(a, depth + 1)
        })
    }

    /// The group `text`, all a definition writes after its `:=` or a member after its label,
    /// names in scope (#594), by the key the walk holds it under — `None` where it names none:
    /// one the body defines or names (`here`), a group formal or a group around the body, one in
    /// force under the scope's prefixes, one a `use` imports bare, or one of a definition's file
    /// (`file_group`), which it is read from wherever it stands (the `bool`).
    fn group_named(
        &self,
        text: &str,
        here: &BTreeSet<String>,
        scope: &Scope,
    ) -> Option<(String, bool)> {
        let prefix = scope.prefix();
        if here.contains(head(text)) {
            return here.contains(text).then(|| (format!("{prefix}{text}"), false));
        }
        if scope.groups.contains(text) {
            return Some((format!("{prefix}{text}"), false));
        }
        let scoped = scope.prefixes.iter().map(|p| format!("{p}{text}"));
        let open = (!scope.closed).then(|| {
            let imported = self.prog.imported(head(text), scope.module)
                .map(|k| format!("{}.{text}", self.prog.modules[k].name));
            std::iter::once(text.to_string()).chain(imported)
        });
        if let Some(k) = scoped.chain(open.into_iter().flatten()).find(|k| self.group_known(k)) {
            return Some((k, false));
        }
        self.file_group(text, scope).map(|k| (k, true))
    }

    /// A name for a group (`abs`), written as `written`, an alias of the group `group_named`
    /// found: the reference it is an alias by — the file's group by its key, read from anywhere
    /// (the `bool`), or as written — recorded in `group_aliases`.
    fn alias_group(&mut self, abs: String, written: Ref, (to, file): (String, bool))
        -> (Ref, bool) {
        let r = if file { Ref::dotted(&to, written.span) } else { written };
        self.group_aliases.insert(abs, to);
        (r, file)
    }

    /// The definitions of a body to work out: every `param` (an unknown declared as it is met)
    /// and every group's members, each name once; the members naming geometry, which bind as
    /// aliases where no number answers; and the names defined here.
    #[allow(clippy::type_complexity)]
    fn definitions(
        &mut self,
        body: &[Stmt],
        vals: &mut Vals,
        scope: &Scope,
        prefix: &str,
    ) -> (Vec<Def>, Vec<(String, Ref, bool)>, BTreeSet<String>) {
        let mut pending: Vec<Def> = Vec::new();
        let mut references = Vec::new();
        let mut here: BTreeSet<String> = BTreeSet::new();
        // **a group is a value** (#594): `small := sizes` names the group `sizes` names, an alias
        // whose members are read through either name — found among the body's groups, its own
        // in place, and the definitions found so far to name one, `a := b` before `b := sizes`
        let mut groups: BTreeSet<String> = BTreeSet::new();
        for st in body {
            let StmtKind::Group(g) = &st.kind else { continue };
            let mut members = Vec::new();
            members_of(&g.name.text, &g.fields, &mut members);
            groups.insert(g.name.text.clone());
            groups.extend(members.iter()
                .filter(|(_, _, f)| matches!(f.value, crate::syntax::InstVal::Group(_)))
                .map(|(name, _, _)| name.clone()));
        }
        let mut aliased: BTreeMap<&str, (String, bool)> = BTreeMap::new();
        loop {
            let found: Vec<(&str, (String, bool))> = body.iter().filter_map(|st| match &st.kind {
                StmtKind::Param(pd) if pd.input.is_none() && is_reference(&pd.text)
                    && !aliased.contains_key(pd.name.text.as_str()) => {
                    let group = self.group_named(pd.text.trim(), &groups, scope)?;
                    Some((pd.name.text.as_str(), group))
                }
                _ => None,
            }).collect();
            if found.is_empty() { break; }
            for (name, group) in found {
                aliased.insert(name, group);
                groups.insert(name.to_string());
            }
        }
        for st in body {
            if let StmtKind::Group(g) = &st.kind {
                let abs = format!("{prefix}{}", g.name.text);
                if !self.group_names.insert(abs.clone()) {
                    self.err(Code::E001, g.name.span, format!("`{}` is declared twice", g.name.text));
                }
                self.defined_groups.insert(abs.clone());
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
                            let text = ref_text(r);
                            let abs = format!("{prefix}{name}");
                            let (r, file) = match self.group_named(&text, &groups, scope) {
                                Some(group) => self.alias_group(abs, r.clone(), group),
                                None => (r.clone(), false),
                            };
                            references.push((name.clone(), r, file));
                            text
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
                            self.defined_groups.insert(abs.clone());
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
            // a parameter axis is no definition to work out: its interval is read once they are
            if pd.over().is_some() {
                continue;
            }
            // a name for a group: its members named under it, as a member naming a group's are,
            // and the name an alias of the group
            if let Some(group) = aliased.get(pd.name.text.as_str()) {
                let text = pd.text.trim();
                let abs = format!("{prefix}{}", pd.name.text);
                let (r, file) = self.alias_group(abs, Ref::dotted(text, pd.span), group.clone());
                references.push((pd.name.text.clone(), r, file));
                pending.push(Def {
                    name: pd.name.text.clone(),
                    text: text.to_string(),
                    span: pd.span,
                    group_ref: true,
                    ty: None,
                });
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
        (pending, references, here)
    }

    /// Each group reference's members, named under the definition referring to it; `false`
    /// where the nesting is cyclic, too deep or too large, said at the definition.
    fn nested_members(
        &mut self,
        pending: &mut Vec<Def>,
        vals: &Vals,
        here: &mut BTreeSet<String>,
    ) -> bool {
        // A nested group exposes the selected group's numeric members, including forward
        // references. Keep ordinary dependency ordering for those members too.
        for depth in 0..MAX_DEPTH {
            let mut extra = Vec::new();
            for d in pending.iter() {
                if !d.group_ref { continue; }
                let start = format!("{}.", d.text);
                // what the group named holds: numbers in force, or defined beside it — read
                // under its name, never by walking every number in force (#405)
                let defined = pending.iter().map(|p| &p.name).filter(|n| n.starts_with(&start));
                let keys: BTreeSet<String> =
                    vals.under(&start).into_keys().chain(defined.cloned()).collect();
                for key in &keys {
                    let Some(member) = key.strip_prefix(&start) else { continue };
                    let name = format!("{}.{member}", d.name);
                    if here.insert(name.clone()) {
                        if depth + 1 == MAX_DEPTH || name.split('.').count() > MAX_DEPTH {
                            self.err(Code::E041, d.span, "group nesting is cyclic or too deep");
                            return false;
                        }
                        if pending.len() + extra.len() >= MAX_FLAT {
                            self.err(Code::E103, d.span, "group expansion is too large");
                            return false;
                        }
                        extra.push(Def { name, text: key.clone(), span: d.span, group_ref: true,
                            ty: None });
                    }
                }
            }
            if extra.is_empty() { break; }
            pending.extend(extra);
        }
        true
    }

    /// The definitions worked out into `vals`, earliest first among those whose reads are
    /// ready; `false` where some are defined in terms of themselves, said at each.
    fn work_out(
        &mut self,
        pending: &[Def],
        vals: &mut Vals,
        scope: &Scope,
        prefix: &str,
        running: &BTreeSet<String>,
    ) -> bool {
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
                return false;
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
                        // a value is worked out once, and a parameter runs: a relation reads it
                        let e = match reads[i].iter().find(|n| running.contains(*n)) {
                            Some(n) => format!("`{n}` runs over an interval, and a value is worked \
                                out once: a relation reads it, and holds for every value of it"),
                            None => e,
                        };
                        let abs = format!("{prefix}{}", d.name);
                        let (text, name, span) = (d.text.clone(), d.name.clone(), d.span);
                        if !self.keep_text(abs, &text, span, vals, scope) {
                            self.err(Code::E103, span, format!("`{name}`: {e}"));
                            failed.insert(name);
                        }
                    }
                }
            }
            waiting = rest;
        }
        true
    }

    /// Each definition worked out to a number, recorded under its absolute name for a reader to
    /// ask for (#596, `Expansion::values`): the drawing's, never a curve's compile over its
    /// variables, nor a hidden copy's (`#3.0.w`).
    fn named_values(&mut self, pending: &[Def], vals: &Vals, prefix: &str) {
        if self.sym.is_some() {
            return;
        }
        for d in pending {
            let abs = format!("{prefix}{}", d.name);
            let Some(a) = vals.get(&d.name) else { continue };
            let Some(value) = a.number().filter(|v| v.is_finite() && !abs.contains('#')) else {
                continue;
            };
            self.values.entry(abs).or_insert(super::Valued { value, dim: a.dim });
        }
    }

    /// **A claim over values** (#596), `claim torque >= 2Nm`: decided over the numbers in scope
    /// and recorded with each it reads (`Expansion::value_claims`), or what keeps it from being
    /// decided said where it stands.  Judged once where a body is walked again (§6.21).
    pub(super) fn value_claim(
        &mut self,
        c: &crate::syntax::ValueClaim,
        vals: &Vals,
        scope: &Scope,
        prefix: &str,
    ) {
        if self.sym.is_some() || !self.claims_judged.insert((c.span, prefix.to_string())) {
            return;
        }
        match claimed(&c.text, vals, scope, self.units) {
            Ok((holds, reads)) => {
                let reads = reads.into_iter().map(|(name, v)| (format!("{prefix}{name}"), v));
                self.value_claims.push(super::ValueClaimed {
                    text: c.text.clone(),
                    span: c.span,
                    holds,
                    reads: reads.collect(),
                });
            }
            Err((code, m)) => self.err(code, c.span, m),
        }
    }

    /// **A parameter axis** (#391 S1, `param t: Angle over (a, b)`): its interval worked out
    /// over the numbers in scope — two numbers of the declared type, the first below the second
    /// — and recorded once (`Expansion::parameters`).  Of the document, as an unknown is.
    fn parameter_axis(
        &mut self,
        pd: &crate::syntax::ParamDecl,
        over: &crate::syntax::Over,
        vals: &mut Vals,
        scope: &Scope,
    ) {
        let name = &pd.name.text;
        if scope.module.is_some() || !scope.prefix().is_empty() {
            let m = format!("`{name}` runs over an interval: a parameter belongs to the document \
                             that draws it");
            self.err(Code::E040, pd.name.span, m);
            return;
        }
        // the parser said what is wrong with a type that is no number
        let Some(ty @ (Ty::Length | Ty::Angle | Ty::Scalar)) = pd.input.as_ref().and_then(|i| i.ty)
        else {
            return;
        };
        let end = |(text, span): &(String, Span)| -> Result<f64, (Span, String)> {
            let said = |e: String| (*span, format!("`{}`: {e}", text.trim()));
            let a = value_aff(text, vals, self.units).and_then(|a| typed(a, ty, name))
                .map_err(said)?;
            match a.number() {
                Some(v) if v.is_finite() => Ok(v),
                Some(_) => Err(said("an end of a parameter's interval is finite".into())),
                None => Err(said("an end of a parameter's interval is a number, and this reads \
                                  an unknown of the drawing".into())),
            }
        };
        let (from, to) = match (end(&over.from), end(&over.to)) {
            (Ok(a), Ok(b)) => (a, b),
            (Err((span, m)), _) | (_, Err((span, m))) => {
                self.err(Code::E103, span, m);
                return;
            }
        };
        if from >= to {
            let m = format!("`{name}` runs over an increasing interval: from `{}` to `{}`",
                over.from.0.trim(), over.to.0.trim());
            self.err(Code::E103, over.span, m);
            return;
        }
        self.parameters.insert(name.clone(), crate::model::Running::new(name.clone(), ty.dim(),
                                                                         from, to));
        // a relation reading it holds for every value of it (#391 S4): it is read as a number
        // held where the drawing stands, its interval's start, as a face family's station reads
        // its running number — never an unknown, and never a value's
        vals.insert(name.clone(), free(name.clone(), ty));
        self.held.names.insert(name.clone(), (ty.dim(), from));
    }

    /// `steel := material(E: 200GPa, nu: 0.3)` (#510 P3): a group whose members, worked out as
    /// any group's are, are a material's properties — each one a material has, of its dimension
    /// and in its range (`model::Property::read`), never an unknown of the drawing — stated under
    /// its absolute name, or refused there with what is wrong said where it is written.
    fn material(&mut self, g: &crate::syntax::GroupDecl, vals: &Vals, prefix: &str) {
        use crate::model::{Material, Property};
        let abs = format!("{prefix}{}", g.name.text);
        if self.materials.contains_key(&abs) {
            return; // a body walked again (§6.21) declares nothing twice
        }
        // each field states one property or is said to be wrong, so every one stated is all
        let mut props = std::collections::BTreeMap::new();
        for f in &g.fields {
            let Some(label) = &f.label else { continue }; // the parser said
            let p = match Property::named(&label.text) {
                Ok(p) => p,
                Err(m) => { self.err(Code::E103, label.span, m); continue; }
            };
            if props.contains_key(&p) {
                self.err(Code::E001, label.span, format!("`{}` is given twice", label.text));
                continue;
            }
            if matches!(f.value, crate::syntax::InstVal::Group(_)) {
                let m = format!("`{}` is a number, and a material's property is no group",
                                p.name());
                self.err(Code::E103, f.span, m);
                continue;
            }
            // a member that did not work out was said where it did not
            let Some(a) = vals.get(&format!("{}.{}", g.name.text, label.text)) else { continue };
            let read = a.number()
                .ok_or_else(|| format!("`{}` is a number the drawing is made of, and this reads \
                                        an unknown of the drawing", p.name()))
                .and_then(|v| p.read(v, a.dim, self.units));
            match read {
                Ok(v) => { props.insert(p, v); }
                Err(m) => self.err(Code::E103, f.span, m),
            }
        }
        let material = if props.len() == g.fields.len() {
            match Material::new(abs.clone(), props) {
                Ok(m) => Some(m),
                Err(m) => {
                    self.err(Code::E103, g.name.span, m);
                    None
                }
            }
        } else {
            None
        };
        self.materials.insert(abs, material);
    }

    /// `param beta: Angle hint(30deg)` — an input nothing binds: a free value in `vals`, named
    /// as written (the document's root has no prefix), and an entry in `unknowns`.
    fn unknown(
        &mut self,
        pd: &crate::syntax::ParamDecl,
        input: &crate::syntax::Input,
        vals: &mut Vals,
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
        vals: &Vals,
    ) -> Result<f64, String> {
        let a = value_aff(text, vals, self.units)?;
        a.dim.require(ty.dim(), "a seed")?;
        a.number().ok_or_else(|| "a seed is a number".to_string())
    }
}

/// E103's words for a parameter axis read where the document's relations are not (#391 S1): a
/// component's body sees none.
pub(super) fn runs_over(name: &str) -> String {
    format!("`{name}` runs over an interval (`param {name}: … over (a, b)`), and only the \
             document's own relations read it")
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
