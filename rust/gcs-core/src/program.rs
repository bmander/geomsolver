//! Elaborate Solvent into a sketch, preserving partial geometry alongside diagnostics.
//!
//! Build in registry order through model constructors, matching JSON parameter order.
//! Evaluate expressions once after all declarations and constraints are built.

mod curves;
mod diagnostics;
mod entities;
mod lift;
mod planes;
mod relations;
mod resolve;
mod solids;
mod source_map;

pub use diagnostics::{Code, Diag, Severity};

/// Geometry diagnostics belong after solving: a poor hint is not an invalid final profile.
pub fn solid_diagnostics(sk: &crate::model::Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut diags: Vec<_> = sk.solids.iter().enumerate().filter(|(_, s)| !matches!(s.def, crate::model::SolidDef::Body { .. }))
        .filter_map(|(i, _)| {
            let message = crate::solid::validate(sk, i).err()?;
            let site = map.site_of(crate::model::EntRef::solid(i));
            Some(Diag { code: Code::E080, span: site.map(|s| s.span).unwrap_or_default(),
                stmt: site.map(|s| s.stmt), message })
        }).collect();
    diags.extend(crate::solid::bearing_errors(sk).into_iter().map(|(b, message)| Diag {
        code: Code::E082, span: b.span, stmt: Some(crate::syntax::StmtId(b.stmt)), message,
    }));
    diags
}
pub use lift::{dumps, to_program};
pub use source_map::{Elaborated, InstPath, Made, Site, SourceMap};

use crate::expr;
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Name, Program, Stmt, StmtId, StmtKind};
pub(crate) use entities::child_names;
use entities::{build, settle_deferred, Deferred};
pub(crate) use lift::{lift_decl, lift_gauge, lift_relation};
use planes::{memberships, plane_bases};
pub(crate) use planes::{plane_of_entity, plane_of_entity_by};
use relations::constrain;
use resolve::Resolver;
use solids::{solid_claims, solids};
use std::collections::{BTreeMap, BTreeSet};

/// Warn when a declaration shadows a built-in name; expressions still read the
/// built-in. Check nested bodies and formals as well as root declarations.
fn shadowing(p: &Program, diags: &mut Vec<Diag>) {
    fn say(name: &Name, what: &str, stmt: Option<StmtId>, diags: &mut Vec<Diag>) {
        let Some(kind) = expr::builtin(&name.text) else { return };
        diags.push(Diag {
            code: Code::W112,
            span: name.span,
            stmt,
            message: format!(
                "`{}` is {kind}, and {what} of that name does not shadow it — an expression \
                 reads the built-in wherever a number is worked out (§3.3), so this name is \
                 read two ways.  Rename it.",
                name.text
            ),
        });
    }
    fn body(stmts: &[Stmt], diags: &mut Vec<Diag>) {
        for st in stmts {
            match &st.kind {
                StmtKind::Param(d) => say(&d.name, "a `param`", Some(st.id), diags),
                StmtKind::Block(b) => {
                    if let Some(i) = &b.binder {
                        say(i, "a block's index", Some(st.id), diags);
                    }
                    body(&b.body, diags);
                }
                _ => {}
            }
        }
    }
    // the root stands among the components, and a module's components with it; what a module's
    // own body adds is its top-level params, which its components read (§6.3)
    let mut said: Vec<Diag> = Vec::new();
    for c in &p.components {
        for f in &c.formals {
            say(&f.name, "a formal", None, &mut said);
        }
        body(&c.body, &mut said);
    }
    for m in &p.modules {
        body(&m.root.body, &mut said);
    }
    // in the order a reader meets them: the components come out of the program in link order,
    // and a document is read down the page
    said.sort_by_key(|d| d.span.lo);
    diags.append(&mut said);
}

pub fn elaborate(p: &Program) -> Elaborated {
    let mut diags: Vec<Diag> = Vec::new();
    let mut map = SourceMap::default();
    let mut sk = Sketch::new();

    // -- phase 0: the document's unit, before *anything* reads a number.
    //
    // A literal with a unit on it converts to this one, and a document that names none is in
    // drawing units (spec §3.3).  It is read from the **unexpanded root** because a unit is
    // document preamble and a component is reusable: a `unit` line anywhere else is refused
    // below rather than quietly doing nothing.  It comes before the curve families, because a
    // family's body is a number-bearing text like any other.
    let mut said = false;
    for st in &p.root().body {
        let crate::syntax::StmtKind::Unit(n) = &st.kind else { continue };
        if said {
            diags.push(Diag {
                code: Code::E040,
                span: n.span,
                stmt: Some(st.id),
                message: "the document's unit is already stated above — one document, one \
                          unit"
                    .to_string(),
            });
            continue;
        }
        said = true;
        match crate::units::Units::with_length(&n.text) {
            Ok(u) => sk.units = u,
            Err(message) => {
                diags.push(Diag { code: Code::E040, span: n.span, stmt: Some(st.id), message })
            }
        }
    }

    // -- a name a document declares over a built-in is said, before anything reads either.
    shadowing(p, &mut diags);

    // -- phase 1: names, in one pre-pass.  Indices come from declaration order within a kind,
    // which is `primitives()` order, which is the order phase 2 builds in.
    let expansion = crate::flatten::expand(p, sk.units);
    diags.extend(expansion.diagnostics.iter().cloned());
    let mut res = Resolver::default();
    let mut count: BTreeMap<EntKind, u32> = BTreeMap::new();
    // a redeclaration is skipped rather than merged, and *which* statement was skipped is
    // remembered here: inferring it later from the name would find the one that won
    let mut skip: BTreeSet<StmtId> = BTreeSet::new();
    use crate::ir::{Operation as StmtKind, Statement};
    let body: Vec<&Statement> = expansion.flat.iter().collect();
    // the style sheet, before anything is built: it says nothing about what the drawing is, so
    // it is collected once and never consulted again by anything here (spec §14)
    let mut sheet = crate::style::Sheet::new();
    for st in &body {
        // a `unit` inside a component or a block is expanded into the flat list and read by
        // nobody — phase 0 takes the root's alone, a component being reusable and a unit being
        // the document's.  Silence there is exactly what §13.1 forbids, so it is a diagnostic.
        if let StmtKind::Unit(n) = &st.kind {
            if !p.root().body.iter().any(|r| r.id == st.id) {
                diags.push(Diag {
                    code: Code::E040,
                    span: n.span,
                    stmt: Some(st.id),
                    message: "a document's unit is stated once, at the top — not inside a \
                              component or a block"
                        .to_string(),
                });
            }
        }
        if let StmtKind::Style(r) = &st.kind {
            // a class stated twice cascades, later over earlier — the same rule that decides a
            // conflicting property between two classes on one declaration
            sheet.entry(r.name.text.clone()).or_default().over(&r.style);
        }
    }
    sk.set_sheet(sheet);
    for st in &body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // the *key*, which is resolution's question: an anonymous key is its own offset and a
        // copy's carries its prefix, so only a name the source wrote twice can actually collide
        let key = &d.name.key().text;
        if let Some(&was) = res.declared_at.get(key) {
            // …but the message shows what the source calls it, and spells the kind where it
            // calls it nothing, so a key cannot leak here even if that argument ever breaks
            let who = d
                .name
                .shown()
                .map_or_else(|| crate::syntax::decl_head(d.kind, &d.name), |n| n.text.clone());
            diags.push(Diag {
                code: Code::E001,
                span: d.name.span(),
                stmt: Some(st.id),
                message: format!(
                    "`{who}` is declared twice; the first is at line {}",
                    p.line_col(was.lo as usize).0
                ),
            });
            skip.insert(st.id);
            continue; // the second, so every later reference still resolves to the first
        }
        let n = count.entry(d.kind).or_insert(0);
        res.of.insert(key.clone(), EntRef::new(d.kind, *n as usize));
        res.declared_at.insert(key.clone(), d.name.span());
        res.kids.insert(
            key.clone(),
            d.children
                .iter()
                .map(|g| match g.first() {
                    Some(crate::syntax::Kid::Ref(r)) if g.len() == 1 => Some(r.clone()),
                    _ => None,
                })
                .collect(),
        );
        *n += 1;
    }

    // every plane's attitude, before any plane is built: a plane folded from another needs
    // the parent's basis, and the build itself must stay in body order (phase 1 assigned the
    // indices in it), so the arithmetic is done here, memoised, and handed to the build
    let bases = plane_bases(&body, &res, &skip, sk.units, &mut diags);

    // -- phase 2: geometry, per kind in `primitives()` order.  The same walk `io::from_json`
    // makes, through the same constructors, so the two produce the same parameter vector.
    let mut built: BTreeMap<EntRef, bool> = BTreeMap::new();
    // seeds that read geometry, settled once every declaration has a seed of its own
    let mut deferred: Vec<Deferred> = Vec::new();
    // `primitives()` order, and **curves last**: a curve is written over other entities, so
    // every kind it may name has to exist before it does.  The same reason `io::graft` grafts
    // them last.
    for kind in [
        EntKind::Point,
        EntKind::Line,
        EntKind::Circle,
        EntKind::Arc,
        EntKind::Spline,
        EntKind::Plane,
        EntKind::Curve,
    ] {
        for st in &body {
            let StmtKind::Decl(d) = &st.kind else { continue };
            if d.kind != kind || skip.contains(&st.id) {
                continue;
            }
            let mut anon: Vec<(String, EntRef)> = Vec::new();
            match build(
                &mut sk,
                &res,
                d,
                st,
                &bases,
                &mut diags,
                &mut anon,
                &mut deferred,
                p,
                &expansion.instances,
            ) {
                Some(e) => {
                    built.insert(e, true);
                    map.bind(&d.name.key().text, e, d.name.named());
                    map.record(st, Made::Ent(e));
                    // an anonymous child's name *is* its dotted path, so it is bound like any
                    // other: that is what lets a dimension name it, a selection survive a
                    // re-elaboration, and a drag of it find the slot it came from.  Whether it
                    // is a name anyone may *say* is the declaration's answer, not its own —
                    // `l.p1` under a named line, nothing under an anonymous one
                    for (name, k) in anon {
                        built.insert(k, true);
                        map.bind(&name, k, d.name.named());
                        map.record(st, Made::Ent(k));
                    }
                }
                None => {
                    // a declaration that could not be built leaves its name unbound, so every
                    // reference to it is reported where the reference is — and made nothing,
                    // so every later entity of its kind sits one index below where phase 1
                    // put it: the resolver is shifted with them, or a reference to a later arc
                    // would read the one after it (or past the end of the list)
                    if let Some(gone) = res.of.remove(&d.name.key().text) {
                        for e in res.of.values_mut() {
                            if e.kind == gone.kind && e.idx > gone.idx {
                                e.idx -= 1;
                            }
                        }
                    }
                }
            }
        }
    }

    // seeds named by geometry, once every entity has a seed to be read: in statement order, so
    // a seed that reads a seed read from a third is settled after both (§6.4)
    settle_deferred(&mut sk, &res, &deferred, &mut diags);

    // memberships, once every kind is built and before any constraint reads one: `point a in
    // top` names a plane built after the point, and `project` infers its planes from these
    memberships(&mut sk, &res, &map, &body, &skip, &mut diags);

    // -- phase 3: constraints, in statement order
    for st in &body {
        let StmtKind::Relation(r) = &st.kind else { continue };
        if let Some(id) = constrain(&mut sk, &res, r, st, &mut diags) {
            map.record(st, Made::Con(id));
            if let Some(place) = r.place {
                sk.placements.insert(id, place);
            }
        }
    }

    // -- phase 3b: faces, then solids (§6.8, §6.9).  **After every other kind and after the
    // constraints**, because a face is written over edges the drawing already has and a solid
    // over faces and other solids; and *evaluated* rather than solved, because nothing about
    // either is an unknown.  This is the stratification as a phase: everything above it is the
    // drawing, everything below reads what the drawing came to.
    solids(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);

    // -- phase 4: every expression against the whole document, once.  Per-statement evaluation
    // would be quadratic in the expression count and would make a dimension whose definition is
    // further down the file briefly a free variable — allocating an unknown the next pass retires.
    // the claims about solids read the free variables `evaluate` allocates, so they come after
    // it — the one pass of the elaboration that is *below* the expressions
    let post_expr = expr::evaluate(&mut sk);
    solid_claims(&mut sk, &res, &mut map, &body, &skip, &mut diags);
    for item in post_expr {
        let span = map.site_of_constraint(item.id).map(|s| s.span).unwrap_or_default();
        let stmt = map.site_of_constraint(item.id).map(|s| s.stmt);
        if let Some(err) = &item.error {
            // what the fault *is* decides what is said (#43.11): a number that is not what its
            // slot takes is the E103 every `param` already gets — §3.3 names `distance(45deg)`
            // as an error — and a claim binding a free name is the E040 §9.7 promises;
            // only an expression that would not compute is a warning, since the last number
            // stands and the drawing goes on
            let (code, tail) = match err.fault {
                expr::Fault::Dimension => (Code::E103, ""),
                expr::Fault::ClaimFree => (Code::E040, ""),
                expr::Fault::Uncomputable => (Code::W110, " — the last number stands"),
            };
            diags.push(Diag { code, span, stmt, message: format!("`{}`: {err}{tail}", item.text) });
        } else if !item.free.is_empty() {
            diags.push(Diag {
                code: Code::W111,
                span,
                stmt,
                message: format!(
                    "`{}` is a free variable: the solver answers for it",
                    item.free.join("`, `")
                ),
            });
        }
    }

    // -- phase 5: a root choice under a key no triple of points spells, kept verbatim
    for st in &body {
        if let StmtKind::Branch(b) = &st.kind {
            sk.branches.insert(b.key.clone(), b.value);
        }
    }

    crate::modules::localize(p, &mut diags);
    Elaborated { sketch: sk, map, diags, program: p.clone(), taken: false }
}
