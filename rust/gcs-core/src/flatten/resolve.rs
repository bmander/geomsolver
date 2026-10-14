//! Alias resolution, private-member access and final reference rewriting.

use super::*;
use crate::model::EntKind;
use crate::program::NORMAL;
use crate::syntax::Member;

/// The geometry a kept seed text reads, renamed to the absolute names the sheet resolves — the
/// same `lookup` every reference goes through, so a formal reads as the actual it aliases and a
/// name inside a block's copy reads as that copy's.  The scalar (`x`, `y`, `r`, `b`) stays on the
/// end, since it is the build's question and not a name.
fn rescope_seeds(k: &mut StmtKind, sc: &Scope, look: Lookup, bad: &mut Vec<(Code, Span, String)>) {
    let StmtKind::Decl(d) = k else { return };
    let mut names: BTreeMap<String, String> = BTreeMap::new();
    for i in 0..d.seed_text.len() {
        let span = d.seed_spans.get(i).copied().unwrap_or_default();
        if let Some(t) = d.seed_text[i].as_deref() {
            rescope_text(t, span, sc, look, &mut names, bad);
        }
    }
    for kid in d.children.iter().flatten() {
        if let crate::syntax::Kid::Hint(ks) = kid {
            for i in 0..3 {
                if let Some(t) = ks.text[i].as_deref() {
                    rescope_text(t, ks.spans[i], sc, look, &mut names, bad);
                }
            }
            if let Some((t, span)) = ks.dir.as_ref().and_then(|d| d.turn.as_ref()) {
                rescope_text(t, *span, sc, look, &mut names, bad);
            }
        }
    }
    // a place's numbers are texts over the same names — `atan2(k2.center.y - k1.center.y, …)`
    for (t, span) in d.seed_at.iter().flat_map(|a| a.texts()) {
        rescope_text(t, *span, sc, look, &mut names, bad);
    }
    d.seed_names = names.into_iter().collect();
}

/// A post-solve number's measurements (`length(gen_g)`), each argument resolved to the absolute
/// name of what it denotes by the same `lookup` every reference goes through, and written back
/// into the text: a formal reads as its actual and a name inside a block copy as that copy's
/// (`side.#282.0.small`, which the expression lexer reads as one name for this reason).
fn rescope_measures(
    text: &mut String,
    span: Span,
    sc: &Scope,
    look: Lookup,
    solids: &BTreeSet<String>,
    bad: &mut Vec<(Code, Span, String)>,
) {
    let Ok(p) = expr::parse_in(text, look.units) else { return };
    let mut to: BTreeMap<String, String> = BTreeMap::new();
    for (_, args) in p.body.measures() {
        for a in args {
            if to.contains_key(&a) {
                continue;
            }
            let segs: Vec<&str> = a.split('.').collect();
            match resolve_dotted(&segs, span, sc, look) {
                Ok((abs, mut rest)) => {
                    // a face's surface, read here: spelled by a segment the source cannot write,
                    // so the elaborator reads the member and not the word (#188)
                    if solids.contains(&abs)
                        && rest.last().is_some_and(|f| Member::of(f) == Some(Member::Surface))
                    {
                        *rest.last_mut().expect("a last field") = Member::Surface.key();
                    }
                    to.insert(a, std::iter::once(abs).chain(rest).collect::<Vec<_>>().join("."));
                }
                Err((code, why)) => bad.push((code, span, why)),
            }
        }
    }
    if !to.is_empty() {
        *text = super::values::map_measured(text, |w| to.get(w).cloned());
    }
}

/// One kept seed text — see `rescope_seeds`: each dotted name in it, resolved.
fn rescope_text(
    t: &str,
    span: Span,
    sc: &Scope,
    look: Lookup,
    names: &mut BTreeMap<String, String>,
    bad: &mut Vec<(Code, Span, String)>,
) {
    let Ok(p) = expr::parse_in(t, look.units) else { return };
    for dep in p.body.deps() {
        if names.contains_key(&dep) {
            continue;
        }
        let mut segs: Vec<&str> = dep.split('.').collect();
        if segs.len() < 2 {
            continue;
        }
        let scalar = segs.pop().unwrap_or_default();
        match resolve_dotted(&segs, span, sc, look) {
            Ok((abs, rest)) => {
                let full = std::iter::once(abs).chain(rest).collect::<Vec<_>>().join(".");
                names.insert(dep.clone(), format!("{full}.{scalar}"));
            }
            Err((code, why)) => bad.push((code, span, why)),
        }
    }
}

/// A dotted name an expression reads (`gen_g`, `k.center`), resolved as the reference it spells
/// by the same `lookup` every reference goes through: the absolute name of what it denotes and
/// the fields past it, or the words for why nothing is.
fn resolve_dotted(segs: &[&str], span: Span, sc: &Scope, look: Lookup)
    -> Result<(String, Vec<String>), (Code, String)> {
    let r = Ref {
        root: Name { text: segs[0].to_string(), span },
        path: segs[1..].iter().map(|f| Seg::Field(Name::new(*f))).collect(),
        span,
    };
    look.find(&r, sc).ok_or_else(|| look.missing(&r, sc))
}

/// The absolute name `name` has inside copy `k` of whichever block declares it, under
/// `container` — the dotted path written before the indexed name (`l.` for `l.p[1]`, nothing
/// for `p[1]`), which is an instance the block was walked inside.
///
/// A block's copies are named `<enclosing>#<statement-id>.<k>.`, so this is a search for that
/// shape among the names the walk has already collected: for each scope the reference can see,
/// every block *within* the container there that declares `name`, at the copy asked for.  Two blocks in one scope
/// declaring the same name make the index ambiguous, and an ambiguous name resolves to nothing
/// rather than to whichever came first.
fn copy_of(
    container: &str,
    name: &str,
    k: usize,
    sc: &Scope,
    names: &BTreeSet<String>,
) -> Option<String> {
    let mut found: Option<String> = None;
    for p in &sc.prefixes {
        let from = format!("{p}{container}#");
        for abs in names.range(from.clone()..).take_while(|n| n.starts_with(&from)) {
            // `#<id>.<j>.<tail>` — the copy's own name, with no further block between
            let tail = &abs[from.len()..];
            let Some((_id, rest)) = tail.split_once('.') else { continue };
            let Some((j, leaf)) = rest.split_once('.') else { continue };
            // the copy's own declaration, or an *instance* the copy holds — `cyl[0].small` is
            // a point of copy 0's `cyl`, whose absolute prefix is what the rest is read under
            let is_inst = leaf.starts_with(name) && leaf[name.len()..].starts_with('.');
            if (leaf != name && !is_inst) || j.parse::<usize>() != Ok(k) {
                continue;
            }
            let copy = format!("{from}{_id}.{j}.{name}");
            if found.as_deref().is_some_and(|f| f != copy) {
                return None; // two blocks here declare it: say nothing rather than guess
            }
            found = Some(copy);
        }
        if found.is_some() {
            return found;
        }
    }
    found
}

fn under_member(target: &str, member: &str) -> bool {
    target == member || target.strip_prefix(member).is_some_and(|s| s.starts_with('.'))
}

/// What a reference is resolved against: the names declared so far, the aliases resolved
/// (`Walk::alias_table`) and the units an index is read in — the one context every reference's
/// lookup reads (`Walk::look`).
#[derive(Clone, Copy)]
pub(super) struct Lookup<'l> {
    pub names: &'l BTreeSet<String>,
    pub alias: &'l BTreeMap<String, String>,
    pub units: Units,
}

impl Lookup<'_> {
    /// An alias may denote a child (`f.origin`), including a child of another alias.
    /// Keep the field suffix while resolving its declared root; it is not a second entity.
    pub(super) fn alias_target(&self, target: &str, tail: &[String])
        -> Option<(String, Vec<String>)> {
        let Lookup { names, alias, .. } = *self;
        let mut segs: Vec<String> =
            target.split('.').map(str::to_string).chain(tail.iter().cloned()).collect();
        for _ in 0..MAX_DEPTH {
            let mut replaced = false;
            for take in (1..=segs.len()).rev() {
                let key = segs[..take].join(".");
                if names.contains(&key) {
                    return Some((key, segs[take..].to_vec()));
                }
                if let Some(next) = alias.get(&key) {
                    if next == &key { continue; }
                    let tail = segs[take..].iter().cloned();
                    segs = next.split('.').map(str::to_string).chain(tail).collect();
                    replaced = true;
                    break;
                }
            }
            if !replaced { return None; }
        }
        None
    }

    /// The absolute name a reference denotes, and whatever field path is left over.
    ///
    /// Greedy on the dotted name: `t.lead` is one name if something declared it, and
    /// `c0.center` is the entity `c0` and its field `center`.  Which it is cannot be told from
    /// the spelling, only from what exists — so the longest match that names something wins.
    pub(super) fn raw(&self, r: &Ref, sc: &Scope) -> Option<(String, Vec<String>)> {
        let Lookup { names, alias, units } = *self;
        // `p[k]` names *which copy* of a repeated statement, so it is resolved before anything
        // else and the rest of the path is read against the copy it picks.  The index may stand
        // on the root or on a dotted name — `l.p[1]` is copy 1 of the `p` a block inside the
        // instance `l` declares, which is how a component's repetition is reached from outside
        // (#45.3) — and what stands before it is the *container* the block was walked under,
        // never a field: an index selects a copy of the block a name was declared in, and a
        // field of a copy is still a field, so it comes after.  Only once — a copy of a copy is
        // a thing no statement makes.
        if let Some(at) = r.path.iter().position(|s| matches!(s, Seg::Index(_))) {
            let Seg::Index(text) = &r.path[at] else { unreachable!() };
            if r.path[at + 1..].iter().any(|s| matches!(s, Seg::Index(_))) {
                return None;
            }
            let k = match value_of(text, &sc.vals, units) {
                Ok(v) if v.is_finite() && v >= 0.0 && v.round() == v => v as usize,
                _ => return None,
            };
            let mut segs: Vec<&str> = vec![r.root.text.as_str()];
            for s in &r.path[..at] {
                if let Seg::Field(f) = s {
                    segs.push(f.text.as_str());
                }
            }
            let leaf = segs.pop()?;
            let abs = if segs.is_empty() {
                copy_of("", leaf, k, sc, names)?
            } else {
                // A layout argument may alias an instance whose body contains the repetition.
                // Resolve that container in the caller's scope before selecting its copy.
                let path = r.path[..at - 1].to_vec();
                let container = Ref { root: r.root.clone(), path, span: r.span };
                let (root, rest) = self.raw(&container, sc)?;
                let prefix = std::iter::once(root).chain(rest).collect::<Vec<_>>().join(".");
                let scope = Scope { prefixes: vec![format!("{prefix}.")], ..sc.clone() };
                copy_of("", leaf, k, &scope, names)?
            };
            // a member is no name (#188): the caller keeps it
            let rest: Vec<String> = r.path[at + 1..]
                .iter()
                .filter_map(|s| match s {
                    Seg::Field(f) => Some(f.text.clone()),
                    Seg::Index(t) => Some(t.clone()),
                    Seg::Member(_) => None,
                })
                .collect();
            // greedy under the copy, as below: `cyl[0].small` is the entity `…cyl.small` and no
            // field, where `p[1].x` is the entity `…p` and its field
            for take in (1..=rest.len()).rev() {
                let cand = format!("{abs}.{}", rest[..take].join("."));
                if names.contains(&cand) {
                    return Some((cand, rest[take..].to_vec()));
                }
                if let Some(t) = alias.get(&cand) {
                    return self.alias_target(t, &rest[take..]);
                }
            }
            return Some((abs, rest));
        }

        let mut segs: Vec<String> = vec![r.root.text.clone()];
        for s in &r.path {
            match s {
                Seg::Field(f) => segs.push(f.text.clone()),
                Seg::Index(t) => segs.push(t.clone()),
                Seg::Member(_) => {}
            }
        }
        // `next` and `prev` name the sibling copy, so the rest of the path is read in *its* scope
        let mut prefixes = sc.prefixes.clone();
        if (segs[0] == "next" || segs[0] == "prev") && segs.len() > 1 {
            let c = sc.block.cyc.as_ref()?;
            let k = match segs[0].as_str() {
                "next" => (c.k + 1) % c.n,
                _ => (c.k + c.n - 1) % c.n,
            };
            segs.remove(0);
            prefixes = vec![format!("{}{}.", c.prefix, k)];
        }
        for take in (1..=segs.len()).rev() {
            let cand = segs[..take].join(".");
            let rest: Vec<String> = segs[take..].to_vec();
            for p in &prefixes {
                let abs = format!("{p}{cand}");
                if names.contains(&abs) {
                    return Some((abs, rest));
                }
                if let Some(t) = alias.get(&abs) {
                    return self.alias_target(t, &rest);
                }
            }
            if !sc.closed {
                if names.contains(&cand) { return Some((cand, rest)); }
                if let Some(t) = alias.get(&cand) { return self.alias_target(t, &rest); }
            }
        }
        None
    }

    /// Explicit entity arguments grant access to that entity. Passing a layout grants its
    /// public interface, not access to private members further inside it.
    pub(super) fn private(&self, r: &Ref, sc: &Scope, target: &str) -> Option<String> {
        let alias = self.alias;
        let mut paths = vec![r.root.text.clone()];
        for seg in &r.path {
            let Seg::Field(f) = seg else { break };
            paths.push(format!("{}.{}", paths.last().unwrap(), f.text));
        }
        let owners =
            std::iter::successors(Some(target), |name| name.rsplit_once('.').map(|(up, _)| up));
        for member in owners {
            let Some(owner) = sc.access.get(member) else { continue };
            if sc.owner == *owner || sc.owner.starts_with(&format!("{member}.")) { continue; }
            let granted = paths.iter().any(|p| {
                sc.prefixes.iter().map(String::as_str).chain((!sc.closed).then_some(""))
                    .any(|prefix| alias.get(&format!("{prefix}{p}"))
                        .is_some_and(|actual| under_member(actual, member)))
            });
            if !granted { return Some(crate::program::public_path(member)); }
        }
        None
    }

    /// `raw`, unless what it finds is a member private to where `r` is read.
    pub(super) fn find(&self, r: &Ref, sc: &Scope) -> Option<(String, Vec<String>)> {
        let found = self.raw(r, sc)?;
        // a group an interface types holds what it declares and nothing past it (§8.1 [0.84])
        if !found.1.is_empty() && sc.typed.contains_key(&found.0) {
            return None;
        }
        self.private(r, sc, &found.0).is_none().then_some(found)
    }

    /// Why `find` found nothing for `r`: the code and the words.
    pub(super) fn missing(&self, r: &Ref, sc: &Scope) -> (Code, String) {
        if let Some((target, rest)) = self.raw(r, sc) {
            if let (Some(shape), Some(member)) = (sc.typed.get(&target), rest.first()) {
                return (Code::E101, format!(
                    "`{}` reads `{member}`, which `{shape}` does not declare: add it to the \
                     interface, or pass it on its own", ref_text(r)
                ));
            }
            if let Some(member) = self.private(r, sc, &target) {
                return (Code::E101, format!("`{}` names private member `{member}`", ref_text(r)));
            }
        }
        // `next` and `prev` name a sibling copy, which only a `cycle` has (§12.1)
        let word = r.root.text.as_str();
        if (word == "next" || word == "prev") && !r.path.is_empty() && sc.block.cyc.is_none() {
            return (Code::E020, format!(
                "`{word}` names the {} copy round a `cycle`, and no `cycle` closes the copies here",
                if word == "next" { "next" } else { "previous" }
            ));
        }
        (Code::E101, format!("no such entity: `{}`", ref_text(r)))
    }
}

/// A reference a statement or a call's argument makes, resolved: as written, where, and what it
/// came to (the absolute name and the fields after it, joined).  What a `ring` is judged by.
pub(super) struct Seen {
    written: String,
    span: Span,
    abs: String,
}

/// What a reference may resolve to once the walk is done: every name declared and every alias
/// resolved, read in the document's units (`look`), the sets (no entity of the drawing), the
/// solids, and the operands an application found naming nothing (said once already).
#[derive(Clone, Copy)]
struct Known<'k> {
    look: Lookup<'k>,
    sets: &'k BTreeSet<String>,
    /// Every solid declared, by absolute name: what a member `surface` or `body` is read past.
    solids: &'k BTreeSet<String>,
    failed: &'k BTreeSet<String>,
    /// The names bound to what a call or a use wrote (`WordState::arguments`).
    arguments: &'k BTreeSet<String>,
    /// Set where a reference names nothing that was said elsewhere, or reads past what an
    /// interface declares: the relation is not stated, so it is not said again.
    unresolved: &'k std::cell::Cell<bool>,
    /// What each reference reading a member `normal` reads it of — see `Expansion::normals`.
    normals: &'k std::cell::RefCell<BTreeSet<String>>,
}

/// Whether `r` reads a name bound to what a call or a use wrote — a formal, a bound variable —
/// which a ring's representative judges there, as written (#226).
fn through_argument(r: &Ref, sc: &Scope, arguments: &BTreeSet<String>) -> bool {
    sc.prefixes.iter().any(|p| arguments.contains(&format!("{p}{}", r.root.text)))
}

/// Whether `r` reads through a name already said to name nothing (`failed`): its root, or a
/// member past it (`d.far`, an interface's member its call did not give).
fn read_through(r: &Ref, sc: &Scope, failed: &BTreeSet<String>) -> bool {
    let mut name = r.root.text.clone();
    let mut names = vec![name.clone()];
    for seg in &r.path {
        let Seg::Field(f) = seg else { break };
        name = format!("{name}.{}", f.text);
        names.push(name.clone());
    }
    sc.prefixes.iter().any(|p| names.iter().any(|n| failed.contains(&format!("{p}{n}"))))
}

fn rewrite(
    k: &mut StmtKind,
    sc: &Scope,
    known: Known,
    bad: &mut Vec<(Code, Span, String)>,
    seen: &std::cell::RefCell<Vec<Seen>>,
) {
    let Known { look, sets, solids, failed, arguments, unresolved, normals } = known;
    // only a ring's representative is judged by what it reads (E021)
    let judged = sc.block.ring.as_ref().is_some_and(|r| !r.turned);
    // a member read already (a set's `ends`, `sets::measured_sets`) is kept; `surface` and
    // `body` past a solid are read here, where no name declared under it is written so (#188)
    let member = |r: &Ref, abs: &str, rest: &mut Vec<String>| -> Option<Seg> {
        let read = r.member().or_else(|| {
            let m = rest.last().and_then(|f| Member::of(f))
                .filter(|m| *m != Member::Ends && solids.contains(abs))?;
            rest.pop();
            Some(m)
        });
        read.map(Seg::Member)
    };
    let fix = |r: &mut Ref, bad: &mut Vec<(Code, Span, String)>| match look.find(r, sc)
    {
        // a set is no entity: what it means is said where it is used (§6.21)
        Some((abs, rest)) if rest.is_empty() && sets.contains(&abs) => bad.push((
            Code::E040,
            r.span,
            format!(
                "`{}` is a set, the points its body holds: a point is put on it by `coincident` \
                 or `inside` and `outside` it, at a distance from its surface by `distance(d, \
                 side: outside)` or `side: inside`, and a line touches it by `tangent`",
                ref_text(r)
            ),
        )),
        Some((abs, mut rest)) => {
            // a plane's normal is minted only where it is read (`Sketch::plane_normal`)
            if let Some((_, of)) = rest.split_last().filter(|(f, _)| *f == NORMAL) {
                normals.borrow_mut().insert(std::iter::once(&abs).chain(of).cloned()
                    .collect::<Vec<_>>().join("."));
            }
            // a formal reads what its call was given, judged at the argument as written
            let was = (judged && !through_argument(r, sc, arguments)).then(|| ref_text(r));
            let member = member(r, &abs, &mut rest);
            r.root = Name { text: abs, span: r.root.span };
            r.path = rest.into_iter().map(|f| Seg::Field(Name::new(f))).chain(member).collect();
            if let Some(was) = was {
                seen.borrow_mut().push(Seen { written: was, span: r.span, abs: ref_text(r) });
            }
        }
        // named as written, so an index that picked no copy says which one it was
        // read through an operand, or a member of one, already said to name nothing
        None if read_through(r, sc, failed) => unresolved.set(true),
        None => {
            // past what an interface declares (§8.1 [0.84]): said here, and only here
            let past = look.raw(r, sc)
                .is_some_and(|(t, rest)| !rest.is_empty() && sc.typed.contains_key(&t));
            unresolved.set(unresolved.get() || past);
            let (code, why) = look.missing(r, sc);
            bad.push((code, r.span, why))
        }
    };
    match k {
        StmtKind::Chain(chain) => {
            for r in &mut chain.links {
                fix(r, bad);
            }
        }
        StmtKind::Decl(d) => rewrite_decl(d, sc, look, solids, bad, &fix),
        // an energy names the curve it is over (#121)
        StmtKind::Minimize(m) => fix(&mut m.curve, bad),
        // a bound on measurements names what it measures, as a motion's numbers do (#183 F3)
        StmtKind::Measured(m) => {
            for x in m.lhs.measures_mut().into_iter().chain(m.rhs.measures_mut()) {
                x.refs_mut().for_each(|r| fix(r, bad));
            }
        }
        // `bore cut body` names two solids, and inside a component both wear the prefix
        StmtKind::SolidRel(r) => {
            fix(&mut r.what, bad);
            fix(&mut r.body, bad);
        }
        // and a picture names the solid it is of and the view it is drawn in
        StmtKind::Derived(d) => {
            fix(&mut d.solid, bad);
            fix(&mut d.plane, bad);
            if let Some(at) = d.at.as_mut() {
                fix(at, bad);
            }
        }
        StmtKind::Relation(rel) => rewrite_relation(rel, sc, look, bad, &fix),
        _ => {}
    }
}
/// A declaration's references rewritten (`rewrite`): its children, the plane it is drawn in —
/// one an instance gave it resolved in the caller's scope — its sweep, motion, curve, place and
/// a seeded axis's direction.
fn rewrite_decl(
    d: &mut Decl,
    sc: &Scope,
    look: Lookup,
    solids: &BTreeSet<String>,
    bad: &mut Vec<(Code, Span, String)>,
    fix: &impl Fn(&mut Ref, &mut Vec<(Code, Span, String)>),
) {
    for g in &mut d.children {
        for kid in g.iter_mut() {
            // a seeded slot names nothing, so there is nothing in it to rescope
            match kid {
                crate::syntax::Kid::Ref(_) | crate::syntax::Kid::Trim { .. } => {
                    for r in kid.refs_mut() { fix(r, bad); }
                }
                crate::syntax::Kid::Face { decl: face, .. } => {
                    // An inline section reads the same scope as its solid. Its loop
                    // contains references, trims or seeds, never another section.
                    for k in face.children.iter_mut().flatten() {
                        for r in k.refs_mut() { fix(r, bad); }
                    }
                }
                crate::syntax::Kid::Hint(_) => {}
            }
        }
    }
    // a plane an instance gave the statement was written at the instance, in the
    // caller's scope, and is resolved there — the component's own names are not in
    // the caller's sight, so they may not take it (#45.4)
    let from_instance = d.membership.source() == crate::syntax::Source::Instance;
    if let Some(r) = d.membership.plane_mut() {
        match (&sc.stamps.plane, from_instance) {
            (Some(ip), true) => {
                let outer = Scope { owner: ip.owner.clone(), prefixes: ip.prefixes.clone(), closed: ip.closed, ..sc.clone() };
                match look.find(r, &outer) {
                    Some((abs, rest)) => {
                        r.root = Name { text: abs, span: r.root.span };
                        r.path =
                            rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect();
                    }
                    None => {
                        let (code, why) = look.missing(r, &outer);
                        bad.push((code, r.span, why))
                    }
                }
            }
            _ => fix(r, bad),
        }
    }
    for r in d.membership.also_mut() {
        fix(r, bad);
    }
    // a revolution's axis is a line of the body like any other name it writes
    for r in d.sweep.as_mut().map(|s| s.references_mut()).unwrap_or_default() {
        fix(r, bad);
    }
    if let Some(motion) = d.motion.as_mut() {
        for r in motion.refs_mut() { fix(r, bad); }
        // a number that measures the drawing names geometry as a reference does, in
        // the scope it was written in; the sheet reads it by absolute name
        for a in motion.args_mut() {
            if let crate::syntax::Arg::Dim { text, span } = a {
                rescope_measures(text, *span, sc, look, solids, bad);
            }
        }
    }
    if let Some(crate::syntax::CurveSpec { target: CurveTarget::Drawn(r), .. }) =
        d.curve.as_mut()
    {
        fix(r, bad);
    }
    if let Some(at) = d.seed_at.as_mut() {
        fix(&mut at.what, bad);
        for r in at.toward.iter_mut().chain(at.along.iter_mut()) {
            fix(r, bad);
        }
    }
    // a direction a plane's axis is seeded along, read off geometry like a place
    for kid in d.children.iter_mut().flatten() {
        if let crate::syntax::Kid::Hint(crate::syntax::KidSeed { dir: Some(dir), .. }) = kid {
            for r in dir.refs_mut() {
                fix(r, bad);
            }
        }
    }
}

/// A relation's references rewritten (`rewrite`): a defined word's operands as written, a
/// derivative row's point and line, and its operands and arguments.
fn rewrite_relation(
    rel: &mut crate::syntax::Relation,
    sc: &Scope,
    look: Lookup,
    bad: &mut Vec<(Code, Span, String)>,
    fix: &impl Fn(&mut Ref, &mut Vec<(Code, Span, String)>),
) {
    // the operands a defined word was written with, read where it was written (§9.9);
    // the body holds the same references, so one that names nothing is said there
    if let Some(w) = rel.word.as_mut() {
        for r in w.ops.iter_mut() {
            if let Some((abs, rest)) = look.find(r, sc) {
                r.root = Name { text: abs, span: r.root.span };
                r.path = rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect();
            }
        }
    }
    // a set's derivative row: its point and the line it is taken along (§6.21)
    if let Some(a) = rel.along.as_mut() {
        fix(&mut a.point, bad);
        if let crate::syntax::AlongBy::Line(l) = &mut a.toward {
            fix(l, bad);
        }
    }
    for a in rel.form.canonical_args_mut().iter_mut().flatten() {
        if let crate::syntax::Arg::Ref(r) = a {
            fix(r, bad);
        }
    }
    // Written operators keep their operands outside the registry argument slots.
    if let Some(w) = rel.form.written_mut() {
        for r in w.ops.iter_mut() {
            fix(r, bad);
        }
        // `level(up)`: a direction word in level's parentheses names nothing to rescope
        let level = w.word.text == "level";
        for a in w.args.iter_mut() {
            match a {
                crate::syntax::OpArg::Ent(r) if level && r.direction_word().is_some() => {}
                crate::syntax::OpArg::Ent(r) => fix(r, bad),
                // a direction an ordinate is measured along, `along: f.axis`; a name that
                // is nothing is a direction word misspelt, which the elaborator says
                // what `along:` takes about
                crate::syntax::OpArg::Named(_, crate::syntax::Arg::Ref(r)) => {
                    if look.find(r, sc).is_some() {
                        fix(r, bad)
                    }
                }
                _ => {}
            }
        }
    }
}

impl<'a> Walk<'a> {
    /// A reference resolved against the names declared so far and `alias`, the walk's aliases
    /// as `alias_table` resolved them.
    pub(super) fn look<'l>(&'l self, alias: &'l BTreeMap<String, String>) -> Lookup<'l> {
        Lookup { names: &self.names, alias, units: self.units }
    }

    /// Every alias the walk has made, resolved to the absolute name it denotes — transitively,
    /// since a formal may be bound to another instance's formal.
    pub(super) fn alias_table(&self) -> BTreeMap<String, String> {
        let mut alias: BTreeMap<String, String> = BTreeMap::new();
        for (abs, r, sc) in &self.aliases {
            if let Some((target, rest)) = self.found(self.look(&alias).raw(r, sc)) {
                let to = std::iter::once(target).chain(rest).collect::<Vec<_>>().join(".");
                alias.insert(abs.clone(), to);
            }
        }
        for _ in 0..MAX_DEPTH {
            let mut moved = false;
            let keys: Vec<String> = alias.keys().cloned().collect();
            for k in keys {
                let v = alias[&k].clone();
                if let Some((root, rest)) = self.look(&alias).alias_target(&v, &[]) {
                    let next = std::iter::once(root).chain(rest).collect::<Vec<_>>().join(".");
                    if next != v {
                        alias.insert(k, next);
                        moved = true;
                    }
                }
            }
            if !moved {
                break;
            }
        }
        alias
    }

    /// What a reference was found to name, unless it is a field of a group or an instance that
    /// nothing under it declares (`d.far` of a group with no `far`): no name at all.
    fn found(&self, f: Option<(String, Vec<String>)>) -> Option<(String, Vec<String>)> {
        f.filter(|(target, rest)| rest.is_empty() || !self.group_names.contains(target))
    }

    /// The scope a set-aside block's chain is looked up in, with the access table the resolve
    /// pass would give it — so a private chain is no more reachable by iterating it.
    fn pending_scope(&self, p: &Pending) -> Scope {
        Scope { access: std::rc::Rc::new(self.private_names.clone()), ..p.scope.clone() }
    }

    /// The chain a set-aside block runs over, where it can be found yet: each link as the chain
    /// wrote it, with the scope it was written in, and whether the chain closes.
    fn chain_of(
        &self,
        p: &Pending,
        alias: &BTreeMap<String, String>,
    ) -> Option<(Vec<(Ref, Scope)>, bool)> {
        let StmtKind::Block(b) = &p.st.kind else { return None };
        let over = b.over.as_ref()?;
        let sc = self.pending_scope(p);
        let (abs, rest) = self.look(alias).find(&over.chain, &sc)?;
        if !rest.is_empty() {
            return None;
        }
        self.out.iter().find_map(|(st, _, sc)| match &st.kind {
            StmtKind::Chain(c) if c.name.key().text == abs => {
                Some((c.links.iter().map(|r| (r.clone(), sc.clone())).collect(), c.closed))
            }
            _ => None,
        })
    }

    /// Whether the walk declared a solid by this absolute name.
    fn solid_named(&self, abs: &str) -> bool {
        self.out.iter().any(|(st, _, _)| matches!(&st.kind,
            StmtKind::Decl(d) if d.kind == crate::model::EntKind::Solid && d.name.key().text == abs))
    }

    /// The faces a set-aside block runs over (#183 F4), where it runs over a solid's (`repeat a
    /// in pad.boss`) and the elaborator has named that solid's: each a reference to the face
    /// (`pad.boss.near`), those its conditions keep (F8).
    fn face_links(
        &self,
        p: &Pending,
        alias: &BTreeMap<String, String>,
    ) -> Option<(Vec<(Ref, Scope)>, bool)> {
        let StmtKind::Block(b) = &p.st.kind else { return None };
        let over = b.over.as_ref()?;
        let sc = self.pending_scope(p);
        // the faces a path names, by the rule a region's reading of them is (`FacePath::matches`)
        let faces_of = |r: &Ref| -> Option<Vec<String>> {
            let (abs, rest) = self.look(alias).find(r, &sc)?;
            let all = self.faces.given.get(&abs)?;
            Some(all.iter().filter(|f| f.matches(&rest)).map(|f| format!("{abs}.{}", f.text()))
                .collect())
        };
        let all = faces_of(&over.chain)?;
        // what a name in a condition stands for: the face itself, or as its reference resolves
        let stands = |name: &Ref, me: &str| -> Option<String> {
            if name.path.is_empty() && name.root.text == over.var.text {
                return Some(me.to_string());
            }
            let (abs, rest) = self.look(alias).find(name, &sc)?;
            Some(std::iter::once(abs).chain(rest).collect::<Vec<_>>().join("."))
        };
        let keeps = |face: &str| over.when.iter().all(|w| {
            let var = Ref { root: w.var.clone(), path: Vec::new(), span: w.var.span };
            let Some(me) = stands(&var, face) else { return false };
            if w.within {
                faces_of(&w.other).is_some_and(|fs| fs.iter().any(|f| *f == me))
            } else {
                stands(&w.other, face).is_some_and(|other| other != me)
            }
        });
        let links = all.iter().filter(|f| keeps(f)).map(|f| {
            let r = Ref::dotted(f, over.chain.span);
            // an absolute name, read from the document's root
            (r, Scope { prefixes: vec![String::new()], access: sc.access.clone(), ..Scope::default() })
        }).collect();
        Some((links, false))
    }

    /// Expand every block over a chain's edges (`repeat e in rack.profile { … }`), each where
    /// the walk met it, once its chain can be found.  A copy may itself hold such a block, or
    /// make the instance holding another's chain, so this runs until nothing more resolves; what
    /// is left names no chain, and says so at the reference.
    pub(super) fn expand_pending(&mut self) {
        if self.aside.blocks.is_empty() {
            return;
        }
        while !self.aside.blocks.is_empty() {
            let alias = self.alias_table();
            let found = self
                .aside.blocks
                .iter()
                .enumerate()
                .find_map(|(i, p)| {
                    self.chain_of(p, &alias).or_else(|| self.face_links(p, &alias)).map(|c| (i, c))
                });
            let Some((i, (links, closed))) = found else { break };
            let p = self.aside.blocks.remove(i);
            self.expand_edges(p, links, closed);
        }
        let alias = self.alias_table();
        for p in std::mem::take(&mut self.aside.blocks) {
            self.take_placeholder(&p);
            let StmtKind::Block(b) = &p.st.kind else { continue };
            let Some(over) = &b.over else { continue };
            let sc = self.pending_scope(&p);
            let r = &over.chain;
            // a block over a solid's faces, whose faces the elaborator names once it has built
            // the solid (#183 F4): none made this pass
            if let Some((abs, _)) = self.look(&alias).find(r, &sc)
                .filter(|(abs, _)| self.solid_named(abs))
            {
                self.faces.repeats.insert(abs);
                continue;
            }
            match self.look(&alias).find(r, &sc) {
                Some(_) => self.err(
                    Code::E103,
                    r.span,
                    format!(
                        "`{}` is not a named chain: `{} {} in …` runs over the edges of a \
                         chain written `name := line -> …`",
                        ref_text(r),
                        if b.kind.wraps() { "cycle" } else { "repeat" },
                        over.var.text
                    ),
                ),
                None => {
                    let (code, msg) = self.look(&alias).missing(r, &sc);
                    self.err(code, r.span, msg)
                }
            }
        }
    }

    /// Where a set-aside block stands in the walk's output — the block statement itself, at its
    /// own path, which no other statement is — taken out.
    fn take_placeholder(&mut self, p: &Pending) -> Option<usize> {
        let at = self.out.iter().position(|(st, path, _)| {
            st.id == p.st.id && matches!(st.kind, StmtKind::Block(_)) && *path == p.path
        })?;
        self.out.remove(at);
        Some(at)
    }

    /// One block over a chain's edges: a copy per link, made into a vector of their own and
    /// spliced in where the walk met the block, so the statements stand in source order.
    fn expand_edges(&mut self, p: Pending, links: Vec<(Ref, Scope)>, closed: bool) {
        let Some(at) = self.take_placeholder(&p) else { return };
        let StmtKind::Block(b) = &p.st.kind else { return };
        let Some(over) = &b.over else { return };
        if b.kind.wraps() && !closed {
            self.err(
                Code::E103,
                over.chain.span,
                format!(
                    "`{}` is an open chain, and a `cycle` closes — its last edge's `next` would \
                     be its first; write `repeat {} in …`",
                    ref_text(&over.chain),
                    over.var.text
                ),
            );
            return;
        }
        // a declaration of the body called what each copy calls its edge would hide it
        let var = &over.var.text;
        if b.body.iter().any(|st| st.kind.bound_name().is_some_and(|n| &n.text == var)) {
            self.err(Code::E001, over.var.span, format!("`{var}` is declared twice"));
            return;
        }
        let outer = std::mem::take(&mut self.out);
        self.aside.held = outer.len();
        let (b, var) = (b.clone(), over.var.clone());
        let edges = Some((&var, links.as_slice()));
        self.copies(&b, &p.st, &p.scope, &p.vals, &p.path, p.depth, links.len(), edges);
        let made = std::mem::replace(&mut self.out, outer);
        self.aside.held = 0;
        self.out.splice(at..at, made);
    }

    /// Turn every reference into the absolute name of what it denotes.
    pub(super) fn resolve(&mut self) -> (Vec<crate::ir::Statement>, BTreeMap<String, String>) {
        let access = std::rc::Rc::new(self.private_names.clone());
        let typed = std::rc::Rc::new(self.typed.clone());
        let scopes = self.out.iter_mut().map(|(_, _, sc)| sc);
        for sc in scopes.chain(self.aliases.iter_mut().map(|(_, _, sc)| sc)) {
            sc.access = access.clone();
            sc.typed = typed.clone();
        }
        let alias = self.alias_table();
        self.drawn_incidences(&alias);
        let failed = self.check_aliases(&alias);
        self.check_bindings(&alias);
        // what each instance was given, now that the aliases its arguments made are absolute
        for info in self.instances.iter_mut() {
            for (formal, actual) in info.ents.iter_mut() {
                *actual = alias.get(&format!("{}{formal}", info.prefix)).cloned();
            }
        }
        let refused = self.refuse_dim_reads(&alias);
        let about = self.resolve_rings(&alias);
        // what a ring's representative reads, judged once every declaration is resolved (E021),
        // and every circle's centre, which a circle about a ring's is judged by
        let mut judged: Vec<(String, Vec<Seen>)> = Vec::new();
        let mut centres: BTreeMap<String, Option<String>> = BTreeMap::new();
        let out = std::mem::take(&mut self.out);
        // a set drawn as an element is named as the element (#105)
        let sets: BTreeSet<String> =
            self.sets.made.keys().filter(|k| !self.sets.lowered.contains_key(*k)).cloned()
                .collect();
        let solids: BTreeSet<String> = out.iter().filter_map(|(st, ..)| match &st.kind {
            StmtKind::Decl(d) if d.kind == EntKind::Solid => Some(d.name.key().text.clone()),
            _ => None,
        }).collect();
        let mut flat = Vec::with_capacity(out.len());
        let normals = std::cell::RefCell::new(std::mem::take(&mut self.normals));
        for (mut st, path, sc) in out {
            // a refused relation is not emitted; a declaration is (a plane whose fold misspells
            // a name), so the error is the one said and not every reference to it after
            if !matches!(st.kind, StmtKind::Decl(_))
                && refused.iter().any(|r| st.span.lo <= r.lo && r.hi <= st.span.hi)
            {
                continue;
            }
            let mut bad: Vec<(Code, Span, String)> = Vec::new();
            let seen = std::cell::RefCell::new(Vec::new());
            let look = self.look(&alias);
            let arguments = &self.words.arguments;
            let (sets, solids, failed) = (&sets, &solids, &failed);
            let unresolved = std::cell::Cell::new(false);
            let known = Known { look, sets, solids, failed, arguments, unresolved: &unresolved, normals: &normals };
            rewrite(&mut st.kind, &sc, known, &mut bad, &seen);
            if let StmtKind::Decl(d) = &st.kind {
                // a centre the circle names, or `None` for one it mints (`c.center`)
                if d.kind == EntKind::Circle {
                    let named = match d.children.first().and_then(|g| g.first()) {
                        Some(Kid::Ref(r)) => Some(ref_text(r)),
                        _ => None,
                    };
                    centres.insert(d.name.key().text.clone(), named);
                }
            }
            if let Some(ring) = sc.block.ring.as_ref().filter(|r| !r.turned) {
                // a seed is no relation: it may start a copy anywhere (Invariant H), and a plane
                // a point is drawn in is a place, not a position
                let mut free = Vec::new();
                if let StmtKind::Decl(d) = &st.kind {
                    if let Some(at) = &d.seed_at {
                        free.extend(std::iter::once(at.what.span).chain(at.toward.iter().chain(&at.along).map(|r| r.span)));
                    }
                    for kid in d.children.iter().flatten() {
                        if let Kid::Hint(crate::syntax::KidSeed { dir: Some(dir), .. }) = kid {
                            free.extend(dir.refs().map(|r| r.span));
                        }
                    }
                    free.extend(d.membership.plane().map(|r| r.span));
                }
                let mut seen = seen.into_inner();
                seen.retain(|s| !free.contains(&s.span));
                judged.push((ring.prefix.clone(), seen));
            }
            // a seed that reads geometry names it in the scope it was written in, and is read
            // on the sheet, where only absolute names mean anything — so it is rescoped as the
            // statement's references were.  Not in a trace block, whose kept texts are read
            // off the block's own variable table by the formals' names.
            if self.sym.is_none() {
                rescope_seeds(&mut st.kind, &sc, self.look(&alias), &mut bad);
            }
            let clean = bad.is_empty();
            for (code, span, msg) in bad {
                self.err(code, span, msg);
            }
            // a relation reading what was said to name nothing is not stated: stated, it would
            // say so again
            if unresolved.get() && !matches!(st.kind, StmtKind::Decl(_)) {
                continue;
            }
            // a curve of a drawn instance's point: the point's name is absolute now, and the
            // instance it belongs to is the innermost one whose component has the formal
            if let StmtKind::Decl(Decl { curve: Some(c), .. }) = &mut st.kind {
                if let (CurveTarget::Drawn(r), true) = (&c.target, clean) {
                    let abs = ref_text(r);
                    match self.owner_of(&abs, &c.swept.text) {
                        Some(of) => c.of = Some(of),
                        None => self.err(
                            Code::E103,
                            r.span,
                            format!(
                                "`{abs}` is not a point of an instance whose component has a \
                                 numeric formal `{}`, and a curve is a point of a component \
                                 as one of its formals runs",
                                c.swept.text
                            ),
                        ),
                    }
                }
            }
            let turned = sc.block.ring.as_ref().is_some_and(|r| r.turned);
            match crate::ir::Statement::lower(st, path) {
                Ok(mut st) => {
                    st.turned = turned;
                    flat.push(st)
                }
                Err(d) => self.diagnostic(d),
            }
        }
        self.judge_arguments(&alias, &mut judged);
        self.normals = normals.into_inner();
        self.judge_rings(&about, judged, &centres);
        (flat, alias)
    }

    /// Every alias a call or a use made, checked once every name is known: one naming a private
    /// member is E101; one naming nothing is said at the operand — the names said so (or left
    /// to `conform`) are the ones returned, not said again where a body reads them.
    fn check_aliases(&mut self, alias: &BTreeMap<String, String>) -> BTreeSet<String> {
        // an operand a use wrote that names nothing is said there, once, and not again at every
        // place in the body that reads it (#103)
        let mut failed: BTreeSet<String> = BTreeSet::new();
        for (key, r, sc) in self.aliases.clone() {
            let raw = self.look(alias).raw(&r, &sc);
            // read past what an interface declares (§8.1 [0.84]), as the argument is written
            let past = raw.as_ref().is_some_and(|(t, rest)| !rest.is_empty() && self.typed.contains_key(t));
            match self.found(raw) {
                Some((target, _)) => {
                    if let Some(member) = self.look(alias).private(&r, &sc, &target) {
                        self.once(Code::E101, r.span, format!("`{}` names private member `{member}`", ref_text(&r)));
                    }
                }
                // a member an interface asks for that its call did not give: said at the call
                // (`conform`), and not again where the body reads it
                None if key.rsplit_once('.').is_some_and(|(group, _)| self.typed.contains_key(group)) => {
                    failed.insert(key);
                }
                None if past || self.words.use_aliases.contains(&key) => {
                    let (code, why) = self.look(alias).missing(&r, &sc);
                    self.once(code, r.span, why);
                    failed.insert(key);
                }
                None => {}
            }
        }
        failed
    }

    /// What each binding asked of what its alias reaches (`Checks`): a group member naming
    /// something, a group or a set formal given one, an interface's members, and a path
    /// formal's kind left for the elaborator (#251).
    fn check_bindings(&mut self, alias: &BTreeMap<String, String>) {
        for (key, span) in self.checks.group_fields.clone() {
            if !alias.contains_key(&key) {
                self.err(Code::E103, span, format!("group member `{key}` names no value or geometry"));
            }
        }
        // a group formal takes a group or an instance, a set formal a set — a named one or a set
        // family's instance, which is both (`s.r` of a sphere) — so a set handed where members
        // are read is said at the call, not as a missing member far from it (#191)
        for (key, span, ty) in self.checks.groups.clone() {
            let target = alias.get(&key);
            if ty == crate::syntax::Ty::Set {
                if !target.is_some_and(|t| self.sets.made.contains_key(t)) {
                    self.err(Code::E103, span, format!("`{key}` wants a set"));
                }
            } else if !target.is_some_and(|t| self.group_names.contains(t)) {
                let key = key.trim_end_matches(keys::GIVEN);
                self.err(Code::E103, span, format!("`{key}` wants a group or layout instance"));
            }
        }
        // what each interface asks of the group or instance its formal was given (§8.1 [0.84]),
        // where it is one: whatever it is not was said just above
        for (key, span, shape, actual) in std::mem::take(&mut self.checks.interfaces) {
            let Some(target) = alias.get(&key).filter(|t| self.group_names.contains(*t)) else {
                continue;
            };
            self.conform(&shape, &target.clone(), &actual, alias, span);
        }
        // what each path formal was given, for the elaborator to judge by its kind (#251)
        for (key, span, formal) in std::mem::take(&mut self.checks.paths) {
            if let Some(target) = alias.get(&key) {
                self.checks.kinds.push(super::KindAsked {
                    target: target.clone(),
                    span,
                    ty: crate::syntax::Ty::Path,
                    what: formal,
                    interface: None,
                });
            }
        }
    }

    /// The statements refused (by where each read a name): those a name nothing declares
    /// refused as their numbers were settled, and those a dimension reading a number of the
    /// geometry, or a dotted name nothing declares, refuses here.
    fn refuse_dim_reads(&mut self, alias: &BTreeMap<String, String>) -> Vec<Span> {
        // a dimension reading a number of the geometry (`k.r`, `k.center.x`): the longest name
        // its path resolves to is a declaration, not an instance, with a field left over.  A
        // traced body reads one as a column of its curve (§6.5); here the expression graph would
        // mint it a free variable of its own, and a drawn instance and its own trace would
        // disagree about what the component says.  Refused, the statement is not emitted
        let mut refused: Vec<Span> = std::mem::take(&mut self.checks.refused);
        for (name, span, sc) in std::mem::take(&mut self.checks.dim_reads) {
            let segs: Vec<&str> = name.split('.').collect();
            let r = Ref {
                root: Name { text: segs[0].to_string(), span },
                path: segs[1..].iter().map(|f| Seg::Field(Name::new(*f))).collect(),
                span,
            };
            // an unknown an instance left — a formal its call did not bind, `u.t.w` — is the one
            // dotted name a dimension may read that is no declaration's
            let known =
                || sc.prefixes.iter().any(|p| self.unknowns.contains_key(&format!("{p}{name}")));
            match self.look(alias).raw(&r, &sc) {
                Some((abs, rest)) if !rest.is_empty() && !self.group_names.contains(&abs) => {
                    refused.push(span);
                    self.err(
                        Code::E103,
                        span,
                        format!(
                            "`{name}` is a number of the geometry, which only a traced body's \
                             dimension reads (§6.5); here state the relation (`equal`, `radius`) \
                             or pass a number"
                        ),
                    );
                }
                // a dotted name nothing declares is a misspelling, as a bare one is: never an
                // unknown of its own making
                found if found.as_ref().is_none_or(|(_, rest)| !rest.is_empty()) && !known() => {
                    refused.push(span);
                    self.err(Code::E101, span, super::values::undefined(&name));
                }
                _ => {}
            }
        }
        refused
    }

    /// Each ring's centre, resolved (`Rings::infos`), by the prefix its copies' names start
    /// with.
    fn resolve_rings(&mut self, alias: &BTreeMap<String, String>) -> BTreeMap<String, String> {
        // each ring's centre, resolved where the block stands — outside every copy — and the
        // block, by the prefix its copies' names start with
        let mut about: BTreeMap<String, String> = BTreeMap::new();
        for (prefix, r, sc, n, span) in std::mem::take(&mut self.rings.written) {
            let Some((abs, rest)) = self.look(alias).find(&r, &sc) else {
                let (code, why) = self.look(alias).missing(&r, &sc);
                self.err(code, r.span, why);
                continue;
            };
            let mut resolved = r.clone();
            resolved.root = Name { text: abs, span: r.root.span };
            resolved.path = rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect();
            about.insert(prefix.clone(), ref_text(&resolved));
            self.rings.infos.push(RingInfo { prefix, about: resolved, n: n as u32, span });
        }
        about
    }

    /// What a ring's representative reads through an argument, as written where the call
    /// stands, added to `judged`.
    fn judge_arguments(
        &self,
        alias: &BTreeMap<String, String>,
        judged: &mut Vec<(String, Vec<Seen>)>,
    ) {
        // an instance's entity argument and a use's operand, as written where the call stands:
        // the body reading it reads the name it is bound to, and is not judged again (#226)
        for (key, r, sc) in &self.aliases {
            let Some(ring) = sc.block.ring.as_ref().filter(|r| !r.turned) else { continue };
            let arguments = &self.words.arguments;
            if !arguments.contains(key) || through_argument(r, sc, arguments) {
                continue;
            }
            if let Some((abs, rest)) = self.look(alias).find(r, sc) {
                let abs = std::iter::once(abs).chain(rest).collect::<Vec<_>>().join(".");
                let seen = Seen { written: ref_text(r), span: r.span, abs };
                judged.push((ring.prefix.clone(), vec![seen]));
            }
        }
    }

    /// E021: what a ring's representative reads must be what every turn of it would read alike —
    /// its own ring's entities (its own copy, or a neighbour by `next` and `prev`), the centre, a
    /// circle about the centre — or the copies would not be turns of it.  A value is no
    /// reference, and seeds and planes were set aside.
    fn judge_rings(
        &mut self,
        about: &BTreeMap<String, String>,
        judged: Vec<(String, Vec<Seen>)>,
        centres: &BTreeMap<String, Option<String>>,
    ) {
        for (prefix, seen) in judged {
            let Some(centre) = about.get(&prefix) else { continue };
            for s in seen {
                if s.abs.starts_with(&prefix) {
                    // a copy reached by index reads the same copy from every turn
                    let by_turn = s.written.starts_with("next.") || s.written.starts_with("prev.");
                    if !by_turn && !s.abs.starts_with(&format!("{prefix}0.")) {
                        self.once(Code::E021, s.span, format!("`{}` is one copy of this `ring`, read \
                            alike from every copy: reach a neighbour by `next` or `prev`", s.written));
                    }
                    continue;
                }
                let invariant = s.abs == *centre
                    || centres.get(&s.abs).is_some_and(|c| c.as_ref() == Some(centre))
                    || centres.contains_key(&s.abs) && *centre == format!("{}.center", s.abs);
                if !invariant {
                    self.once(Code::E021, s.span, format!("`{}` is outside the `ring` and does not \
                        turn with it: from inside, only its centre `{centre}`, a circle about it and \
                        values may be read — pass what turns as part of the ring", s.written));
                }
            }
        }
    }

    /// The instance a curve of the point `abs` is a curve *of*: the innermost drawn instance
    /// whose prefix the name starts with **and whose component declares `swept` as a numeric
    /// formal** — so `o.i.t over u` sweeps `Outer`'s `u` when `Inner` has none, and `Inner`'s
    /// own when it does.
    fn owner_of(&self, abs: &str, swept: &str) -> Option<crate::syntax::CurveOf> {
        let mut owners: Vec<&InstanceInfo> =
            self.instances.iter().filter(|i| i.drawn && abs.starts_with(&i.prefix)).collect();
        owners.sort_by_key(|i| std::cmp::Reverse(i.prefix.len()));
        owners
            .into_iter()
            .find(|i| {
                self.prog.components[i.comp].formals.iter()
                    .any(|f| f.name.text == swept && !matches!(f.ty, Ty::Ent(_) | Ty::Path | Ty::Group | Ty::Set))
            })
            .map(|i| crate::syntax::CurveOf {
                instance: i.prefix.clone(),
                point: abs[i.prefix.len()..].to_string(),
            })
    }
}
