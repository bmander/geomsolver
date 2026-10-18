//! Removing: statements doomed with what they declare or mention, spliced out.

use super::*;

/// The statements a removal takes, by id.
type Doomed = std::collections::BTreeSet<crate::syntax::StmtId>;

/// The names of what a removal takes: entities and declarations alike, by their keys.
type Gone = std::collections::BTreeSet<String>;

/* -- removing ---------------------------------------------------------------------- */

/// Take entities and constraints out of the source.
///
/// A statement goes when it declares something being removed, or when it *mentions* one — which
/// is the same rule `graft::without` follows on a sketch, said about text instead: a constraint
/// comes along exactly when all its entities did.
///
/// What was made inside a component or a block **stays**, and the rest goes: there is no one
/// statement to delete there — the statement makes N of them — and quietly deleting the whole
/// component would be a much larger edit than the gesture asked for.  So a selection swept over
/// a drawing that also caught `std`'s datums deletes the drawing; the edit counts what it left
/// (`Edit::kept`), and is refused, saying why, only when that is everything it was given.
///
/// The **sketch is passed**, as `reconcile` and `commit_seeds` take theirs and for the same
/// reason: a front end moves the sketch out of the elaboration into a handle of its own
/// (`Elaborated::taken`), so `e.sketch` is empty there.  The one question here only the model
/// can answer is which constraints name an entity their text never spells — a projection's
/// planes — and asked of an empty sketch it silently answers none.
pub fn remove(
    e: &Elaborated,
    prog: &Program,
    sk: &Sketch,
    ents: &[EntRef],
    cons: &[u32],
) -> Edit {
    let (mut doomed, mut names, left) = selected(e, prog, ents, cons);
    doom_mentioning(prog, &mut doomed, &mut names);
    // what the model builds over the gone without naming it — a solid placed under a frame
    // whose joint turned about a deleted line (#383) — goes too, and then what names that
    while doom_dependent(e, prog, sk, &mut doomed, &mut names) {
        doom_mentioning(prog, &mut doomed, &mut names);
    }
    doom_inferred(e, prog, sk, &names, &mut doomed);
    if doomed.is_empty() {
        let why = (left > 0).then(|| {
            "that comes from a component, so deleting it would edit the component and everything \
             else drawn from it"
                .to_string()
        });
        return Edit::none(prog, why);
    }
    // a link of a chain has no deletion splice, so the gesture is refused rather than half-done
    let refuse = || {
        Edit::none(
            prog,
            Some(
                "that is part of a chain, which deletion cannot unpick; edit the source instead"
                    .into(),
            ),
        )
    };
    let mut edits: Vec<Splice> = membership_splices(prog, &doomed, &names);
    // a frame declared in a block's header (`in (f := frame) { … }`, #383) goes with the header,
    // which the membership pass splices whole: its own statement's splice would overlap it
    let headers: Vec<Span> = prog.in_blocks.iter()
        .filter(|b| b.declares && names.contains(&b.plane.root.text)).map(|b| b.header).collect();
    for s in doomed_splices(prog.text(), &prog.root().body, &doomed) {
        match s {
            Some(e) if headers.iter().any(|h| e.at.lo < h.hi && h.lo < e.at.hi) => {}
            Some(e) => edits.push(e),
            None => return refuse(),
        }
    }
    // a prefix word stands inside the statement of the declaration it qualifies once a name is
    // written before both (`l := horizontal line(a, b)`): the declaration's splice takes it
    let inside = |e: &Splice, f: &Splice| {
        f.at.lo <= e.at.lo && e.at.hi <= f.at.hi && (f.at.lo, f.at.hi) != (e.at.lo, e.at.hi)
    };
    let edits: Vec<Splice> =
        edits.iter().filter(|e| !edits.iter().any(|f| inside(e, f))).cloned().collect();
    let text = splice(prog.text(), edits);
    if unpicked_badly(prog, &doomed, &text) {
        return refuse();
    }
    Edit { text, kind: Kind::Structural, names: Vec::new(), refused: None, kept: left }
}

/// What the selection itself dooms: each entity's statement and each constraint's, where it is
/// the root's own — with the names of the entities going, and how many were left standing
/// because a component or a block made them.
fn selected(e: &Elaborated, prog: &Program, ents: &[EntRef], cons: &[u32]) -> (Doomed, Gone, usize) {
    let mut doomed = Doomed::new();
    let mut names = Gone::new();
    let mut left = 0;
    for r in ents {
        let Some(site) = e.map.of_entity.get(r) else { continue };
        // a component's, which stays: the rest of what was asked for still goes
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            left += 1;
            continue;
        }
        doomed.insert(site.stmt);
        if let Some(d) = decl_of(prog, site) {
            names.insert(d.name.key().text.clone());
        }
    }
    for id in cons {
        if let Some(site) = e.map.of_constraint.get(id) {
            if site.path.0.is_empty() && in_root(prog, site.stmt) {
                doomed.insert(site.stmt);
            }
        }
    }
    (doomed, names, left)
}

/// And every statement that names one of the gone — to a fixed point, because a line that named
/// a deleted point is itself deleted, and the constraint on *that* line goes with it.
fn doom_mentioning(prog: &Program, doomed: &mut Doomed, names: &mut Gone) {
    loop {
        let mut grew = false;
        for st in prog.root().body.iter() {
            if doomed.contains(&st.id) || mentions(st, names).is_empty() {
                continue;
            }
            doomed.insert(st.id);
            if let StmtKind::Decl(d) = &st.kind {
                names.insert(d.name.key().text.clone());   // and now whatever names *it* goes too
            }
            grew = true;
        }
        if !grew {
            break;
        }
    }
}

/// And every root declaration the model builds over a motion one of the gone takes with it — a
/// placement under a frame whose joint turned about a deleted line (#383): a motion node has no
/// statement of its own to go, so what reads it goes in its place.  Whether any was found.
fn doom_dependent(e: &Elaborated, prog: &Program, sk: &Sketch, doomed: &mut Doomed,
                  names: &mut Gone) -> bool {
    let mut dead: std::collections::BTreeSet<EntRef> =
        names.iter().filter_map(|n| e.map.ent_named(n)).collect();
    if dead.is_empty() || sk.motions.is_empty() {
        return false;
    }
    let lost = |x: EntRef, dead: &std::collections::BTreeSet<EntRef>| {
        let kids = sk.children(x);
        !kids.is_empty()
            && kids.iter().filter(|c| !dead.contains(c)).count() < sk.min_children(x, &kids)
    };
    // the motions that go, a relative or an inverse with what it is over
    loop {
        let gone: Vec<EntRef> = (0..sk.motions.len()).map(|i| EntRef::new(EntKind::Motion, i))
            .filter(|m| !dead.contains(m) && lost(*m, &dead)).collect();
        if gone.is_empty() {
            break;
        }
        dead.extend(gone);
    }
    let mut found = false;
    let all = EntKind::ALL.iter().filter(|&&k| k != EntKind::Motion)
        .flat_map(|&k| (0..sk.count(k)).map(move |i| EntRef::new(k, i)));
    for x in all {
        let reads_gone = sk.children(x).iter()
            .any(|c| c.kind == EntKind::Motion && dead.contains(c));
        let Some(site) = e.map.of_entity.get(&x).filter(|_| reads_gone) else { continue };
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) || doomed.contains(&site.stmt) {
            continue;
        }
        let Some(d) = decl_of(prog, site) else { continue };
        doomed.insert(site.stmt);
        names.insert(d.name.key().text.clone());
        found = true;
    }
    found
}

/// And every statement whose *constraint* names one of the gone without its text doing so: a
/// projection's planes are its points' memberships, inferred and never spelled, so the model
/// drops it when a plane goes (`graft::without`'s rule) and its statement goes with it.
fn doom_inferred(e: &Elaborated, prog: &Program, sk: &Sketch, names: &Gone, doomed: &mut Doomed) {
    // only a kind with an inferred entity slot can be in that position, so a document with none
    // skips the walk — `user_constraints` and `entities` both allocate
    let gone: std::collections::BTreeSet<EntRef> = sk
        .constraints
        .iter()
        .any(|c| c.kind.spec().iter().enumerate().any(|(i, (_, k))| k.is_entity() && c.kind.infers_arg(i)))
        .then(|| names.iter().filter_map(|n| e.map.ent_named(n)).collect())
        .unwrap_or_default();
    for c in sk.user_constraints().into_iter().filter(|_| !gone.is_empty()) {
        if !c.entities().iter().any(|x| gone.contains(x)) {
            continue;
        }
        if let Some(site) = e.map.of_constraint.get(&c.id) {
            if site.path.0.is_empty() && in_root(prog, site.stmt) {
                doomed.insert(site.stmt);
            }
        }
    }
}

/// A membership names a plane without depending on it: a point whose plane goes stays, and only
/// its `in …` clause comes out — with the space that set it off — and an `in PLANE { … }` block
/// whose plane goes loses its header and brace, its statements staying.
fn membership_splices(prog: &Program, doomed: &Doomed, names: &Gone) -> Vec<Splice> {
    let mut clauses: Vec<Splice> = Vec::new();
    for st in prog.root().body.iter() {
        if doomed.contains(&st.id) {
            continue;
        }
        let m = match &st.kind {
            StmtKind::Decl(d) => &d.membership,
            StmtKind::Instance(i) => &i.membership,
            _ => continue,
        };
        // only a clause this statement wrote has a span here to take out; a block's comes out
        // with the block's own header below
        let (Some(p), at) = (m.written(), m.span()) else { continue };
        if at.is_empty() {
            continue;
        }
        // a point drawn in several planes keeps the ones that stay (§6.7)
        let planes: Vec<&crate::syntax::Ref> = std::iter::once(p).chain(m.also()).collect();
        let kept: Vec<String> = planes.iter()
            .filter(|r| !names.contains(&r.root.text))
            .map(|r| crate::syntax::ref_text(r))
            .collect();
        if kept.len() == planes.len() {
            continue;
        }
        let with = (!kept.is_empty()).then(|| format!("in {}", kept.join(", ")));
        clauses.push(clause_splice(prog.text(), at, with));
    }
    // and an `in PLANE { … }` block whose plane goes: the header and its brace come out, and
    // the statements stay — points in space now, exactly as a clause's point stays.  The block's
    // own decls have no clause span, so the pass above never reaches into the header.
    for b in &prog.in_blocks {
        if !names.contains(&b.plane.root.text) {
            continue;
        }
        for at in [b.header, b.close] {
            clauses.push(clause_splice(prog.text(), at, None));
        }
    }
    clauses
}

/// Whether the text a removal left no longer parses where it did.  Where a chain was touched,
/// the result must still parse: a chain can weave what no set of per-statement splices unpicks —
/// a name link left dangling between two doomed joints — and a deletion that corrupts the source
/// is worse than one that is refused.  Whole-line deletions cannot introduce an error, so they
/// skip both parses; a clean result cannot have more errors than the old text, so it skips the
/// baseline one.
fn unpicked_badly(prog: &Program, doomed: &Doomed, text: &str) -> bool {
    let woven = prog
        .root()
        .body
        .iter()
        .any(|s| doomed.contains(&s.id) && !matches!(s.chained, syntax::Chained::No));
    if woven {
        let errs = crate::syntax::parse(text).1.len();
        if errs > 0 && errs > crate::syntax::parse(prog.text()).1.len() {
            return true;
        }
    }
    false
}

/// The names a statement refers to, of those given.
pub(super) fn mentions(st: &Stmt, names: &std::collections::BTreeSet<String>) -> Vec<String> {
    let mut hit = Vec::new();
    let mut look = |r: &syntax::Ref| {
        if names.contains(&r.root.text) {
            hit.push(r.root.text.clone());
        }
    };
    match &st.kind {
        StmtKind::Group(g) => {
            let mut fields: Vec<&syntax::InstArg> = g.fields.iter().collect();
            while let Some(field) = fields.pop() {
                match &field.value {
                    syntax::InstVal::Ref(r) => look(r),
                    syntax::InstVal::Group(inner) => fields.extend(inner),
                    syntax::InstVal::Expr(_) | syntax::InstVal::Hint(..) => {}
                }
            }
        }
        StmtKind::Chain(c) => {
            for r in &c.links {
                look(r);
            }
        }
        StmtKind::Decl(d) => decl_refs(d, &mut look),
        // the body rule goes with either solid it names
        StmtKind::SolidRel(rel) => {
            look(&rel.what);
            look(&rel.body);
        }
        // an energy goes with the curve it is over (#121)
        StmtKind::Minimize(m) => look(&m.curve),
        // a bound on measurements goes with what it measures (#183 F3)
        StmtKind::Measured(m) => {
            for x in m.lhs.measures().into_iter().chain(m.rhs.measures()) {
                x.refs().for_each(&mut look);
            }
        }
        // a set goes with what its body names (§6.21), as a component's call goes with its
        // arguments
        StmtKind::Set(set) => {
            for inner in &set.lit.body {
                hit.extend(mentions(inner, names));
            }
        }
        StmtKind::Relation(rel) => relation_refs(rel, &mut look),
        // a call with no name makes nothing another statement can name, only what it states of
        // its arguments — `std.Turn(f, about: ax, ratio: 2)` (#383) — so it goes with any of them
        StmtKind::Instance(i) if i.name.text.starts_with('#') => {
            let mut args: Vec<&syntax::InstArg> = i.args.iter().collect();
            while let Some(a) = args.pop() {
                match &a.value {
                    syntax::InstVal::Ref(r) => look(r),
                    syntax::InstVal::Group(inner) => args.extend(inner),
                    syntax::InstVal::Expr(_) | syntax::InstVal::Hint(..) => {}
                }
            }
        }
        // a view or a section of a solid goes with it, and with the plane it is drawn in
        StmtKind::Derived(dv) => {
            look(&dv.solid);
            look(&dv.plane);
            if let Some(at) = &dv.at {
                look(at);
            }
        }
        StmtKind::Branch(_) | StmtKind::Instance(_) | StmtKind::Param(_) | StmtKind::Block(_)
        | StmtKind::Style(_) | StmtKind::ClaimOver(_) | StmtKind::Unit(_) => {}
    }
    hit
}

/// Each reference a declaration depends on: its children, what its motion turns about or
/// measures, what its curve is written over, and what its solid is swept about, through, along
/// or under.
fn decl_refs(d: &Decl, look: &mut impl FnMut(&syntax::Ref)) {
    for g in &d.children {
        for r in g.iter().flat_map(|k| k.refs()) {
            look(r);
        }
    }
    // a plane's axes are its children, looked at above; a membership (`in …`) is a
    // label the point survives losing, and is not counted
    if let Some(e) = d.angular_span.as_ref().and_then(|s| s.ease.as_ref()) {
        look(&e.about);
        look(&e.along);
    }
    // the frame a motion is seen from is what the reading is written over (#383)
    if let Some(g) = &d.seen_from {
        look(g);
    }
    // a curve is a point of an instance, and goes with what it is written over: the
    // instance's point, or — written in place — the entities the instance was given
    if let Some(c) = &d.curve {
        match &c.target {
            syntax::CurveTarget::Drawn(r) => look(r),
            syntax::CurveTarget::Anon(inst, _) => {
                for a in &inst.args {
                    if let syntax::InstVal::Ref(r) = &a.value {
                        look(r);
                    }
                }
            }
        }
    }
    // a solid goes with what it is swept about, through, along or under — and a face family
    // with the entities its instance, written in place, was given
    for r in d.sweep.as_ref().map(syntax::Sweep::references).unwrap_or_default() {
        look(r);
    }
    if let Some(syntax::Sweep::Family(f)) = &d.sweep {
        for a in &f.instance.args {
            if let syntax::InstVal::Ref(r) = &a.value {
                look(r);
            }
        }
    }
}

/// Each reference a relation names: its canonical arguments, or the operands a written operator
/// names and another entity it may name in parentheses.
fn relation_refs(rel: &syntax::Relation, look: &mut impl FnMut(&syntax::Ref)) {
    for a in rel.form.canonical_args().iter().flatten() {
        if let syntax::Arg::Ref(r) = a {
            look(r);
        }
    }
    // Written operators name operands and may name another entity in parentheses.
    if let Some(w) = rel.form.written() {
        for r in w.ops.iter() {
            look(r);
        }
        for a in &w.args {
            match a {
                syntax::OpArg::Ent(r) | syntax::OpArg::Named(_, syntax::Arg::Ref(r)) => {
                    look(r)
                }
                _ => {}
            }
        }
    }
}

/// One trailing clause, written where the parser said it goes or taken out of the line.
///
/// **The separator dance, once.**  A clause the source wrote is *replaced* between the spaces
/// it already had; one it did not is *inserted* and brings the space that sets it off (the
/// empty-span idiom `class_span`, `plane_span` and `place_span` all use); and one that goes
/// takes the blanks in front of it with it, or the line is left with a gap.  `class`, `in` and
/// a callout's placement are all this, and it had been written out at five sites — where the
/// fiddly half is the same at every one and the wrong half is invisible until a statement
/// prints with two spaces or none.
pub(super) fn clause_splice(text: &str, at: Span, with: Option<String>) -> Splice {
    match with {
        Some(s) if at.is_empty() => Splice { at, with: s },
        Some(s) => Splice { at, with: s.trim_start().to_string() },
        None => {
            let lo = back_over_spaces(text, at.lo as usize);
            Splice { at: Span::new(lo, at.hi as usize), with: String::new() }
        }
    }
}

/// Back over the blanks before an offset — so a clause deleted from the middle of a line does
/// not leave the space that set it off behind.
fn back_over_spaces(text: &str, mut lo: usize) -> usize {
    let b = text.as_bytes();
    while lo > 0 && (b[lo - 1] == b' ' || b[lo - 1] == b'\t') {
        lo -= 1;
    }
    lo
}

/// The splice that takes a doomed statement out of the text — `None` where there is none.
///
/// Almost always the statement's whole line.  A chain (spec §6.6) puts several statements on one
/// line, so which characters go is a question about *how the statement was written*, and the
/// parser answered it while desugaring: a joint steps down to `->` (the corner stays, the claim
/// goes, and the chain still parses), a prefix word is deleted where it stands with the spaces
/// that set it off, and a link has no splice at all — nothing takes one link out and leaves a
/// chain behind, so it is refused.  Reading the answer back out of the characters would instead
/// rest on "a longhand relation always carries a `(`", which nothing states and a qualified
/// joint would quietly break.
fn doom_splice(text: &str, st: &Stmt) -> Option<Splice> {
    doom_at(text, st.span, &st.chained)
}

/// The doom splices for a set of statements at once — and the composition the set needs: the
/// words of one joint hang together, so while each doomed alone splices out where it stands
/// and the rest hold the line, a joint whose every *written* word is doomed (an entity
/// deletion dooms every relation naming it) would leave two links with nothing between them.
/// Such a joint yields the one splice its only word's doom would be — the `fall` its members
/// carry — and its other members yield nothing.  Counted against `out_of`, the words as
/// written, so a word that was refused at desugar (emitting no statement) holds the joint's
/// text in place.  A `None` is a statement with no splice at all, the caller's refusal.
pub(super) fn doomed_splices(
    text: &str,
    body: &[Stmt],
    doomed: &std::collections::BTreeSet<syntax::StmtId>,
) -> Vec<Option<Splice>> {
    let mut fell: std::collections::BTreeMap<Span, usize> = Default::default();
    for st in body.iter().filter(|s| doomed.contains(&s.id)) {
        if let syntax::Chained::Member { of, .. } = st.chained {
            *fell.entry(of).or_insert(0) += 1;
        }
    }
    let mut out = Vec::new();
    for st in body.iter().filter(|s| doomed.contains(&s.id)) {
        match st.chained {
            syntax::Chained::Member { of, fall, out_of } => match fell.get(&of) {
                // every written word fell: the joint composes to its only word's doom, once
                // — taking the entry marks it done
                Some(&n) if n == out_of as usize => {
                    fell.remove(&of);
                    out.push(doom_at(text, of, &fall.into()));
                }
                // siblings hold the line: this word goes out alone
                Some(_) => out.push(doom_splice(text, st)),
                // the joint's one composed splice went with its first member
                None => {}
            },
            _ => out.push(doom_splice(text, st)),
        }
    }
    out
}

/// The splice for one doomed spelling over one span — `doom_splice`'s own body, split out so
/// `remove` can compose a joint whose every word fell at once: it dooms the `fall` the members
/// carry, over the joint's whole span.
fn doom_at(text: &str, at: Span, how: &syntax::Chained) -> Option<Splice> {
    let one_word = |with: &str| Some(Splice { at, with: with.to_string() });
    match how {
        syntax::Chained::No => Some(Splice { at: with_line(text, at), with: String::new() }),
        syntax::Chained::Link => None,
        // a threaded joint steps down to the bare corner: the claim goes, the weld stays
        syntax::Chained::Joint => one_word("->"),
        // an unthreaded joint states only the relation, so its span — grown at desugar time
        // over a terminal name-link that would otherwise dangle — becomes a statement break;
        // one that is the whole of its line would leave only a blank one, so the line goes
        syntax::Chained::Infix => {
            let lo = back_over_spaces(text, at.lo as usize);
            let hi = skip_spaces(text, at.hi as usize);
            let whole = (lo == 0 || text.as_bytes()[lo - 1] == b'\n')
                && matches!(text.as_bytes().get(hi), None | Some(&b'\n'));
            match whole {
                true => Some(Splice { at: with_line(text, at), with: String::new() }),
                false => Some(Splice { at, with: "\n".to_string() }),
            }
        }
        // an unthreaded joint in a chain that closes: a break would re-aim the `close` at
        // another link, so there is no splice and the gesture is refused
        syntax::Chained::Stuck => None,
        // one of a joint's several words, or a prefix word, goes out where it stands with
        // the blanks after it — a comment or a line break beside it survives.  A doomed
        // member leaves the corner and the joint's other statements standing; the whole
        // joint doomed at once is composed by `doomed_splices` from the `fall` it carries
        syntax::Chained::Member { .. } | syntax::Chained::Prefix => Some(Splice {
            at: Span::new(at.lo as usize, skip_spaces(text, at.hi as usize)),
            with: String::new(),
        }),
        // the joint word and `close` share a span, and only the claim goes
        syntax::Chained::Close => {
            let tail = at.slice(text);
            let close = tail.rfind("close").map(|i| &tail[i..]).unwrap_or("close");
            one_word(&format!("-> {close}"))
        }
    }
}

/// Past the blanks at an offset — what a deletion swallows so it leaves no ragged gap.
fn skip_spaces(text: &str, mut hi: usize) -> usize {
    let b = text.as_bytes();
    while hi < b.len() && (b[hi] == b' ' || b[hi] == b'\t' || b[hi] == b'\r') {
        hi += 1;
    }
    hi
}

/// A statement's span, grown to swallow the newline that ends it — so deleting one does not
/// leave a blank line where it stood.
fn with_line(text: &str, s: Span) -> Span {
    let mut hi = skip_spaces(text, s.hi as usize);
    if text.as_bytes().get(hi) == Some(&b'\n') {
        hi += 1;
    }
    Span::new(s.lo as usize, hi)
}
