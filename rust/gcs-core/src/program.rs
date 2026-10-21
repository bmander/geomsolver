//! Elaborate Solvent into a sketch, preserving partial geometry alongside diagnostics.
//!
//! Build in registry order through model constructors, matching JSON parameter order.
//! Evaluate expressions once after all declarations and constraints are built.

mod curves;
mod diagnostics;
mod elaborate;
mod entities;
mod gauges;
mod lift;
mod planes;
mod views;
mod reading;
mod regions;
mod relations;
mod resolve;
mod solids;
mod surfaces;
mod motions;
mod envelopes;
mod generated;
mod rings;
mod source_map;
mod variational;
mod witnessed;
mod words;
mod traced;

pub use diagnostics::{Code, Diag, Severity};

/// Geometry diagnostics belong after solving: a poor hint is not an invalid final profile.
pub fn solid_diagnostics(sk: &crate::model::Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut diags: Vec<_> = sk.solids.iter().enumerate().filter(|(_, s)| !matches!(s.def, crate::model::SolidDef::Body { .. }))
        .filter_map(|(i, _)| {
            let message = crate::solid::validate(sk, i).err()?;
            let site = map.site_of(crate::model::EntRef::solid(i));
            Some(Diag::new(Code::E080, site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt), message))
        }).collect();
    diags.extend(crate::solid::bearing_errors(sk).into_iter().map(|(b, message)| Diag::within(
        Code::E082, b.span, crate::syntax::StmtId(b.stmt), message,
    )));
    // a prism's side is read in the drawing, where its envelope stands for it (`extruded_envelopes`)
    let prism = |i: usize| matches!(sk.solids[sk.surfaces[i].solid as usize].def, crate::model::SolidDef::Prism { .. });
    for i in (0..sk.surfaces.len()).filter(|&i| !prism(i)) {
        if let Err(message) = crate::solid::RevolvedSurface::named(sk,i) {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Surface,i));
            diags.push(Diag::new(Code::E080,site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),message));
        }
    }
    for i in 0..sk.motions.len() {
        let read = match &sk.motions[i].def {
            // a drive over a number that is no angle is read over the roll by nothing (#398 B): a
            // reading that would is refused where it is made (`motions::reading_of`), and what is
            // drawn in the frame reads the joint at its value, followed as the number runs
            crate::model::MotionDef::Driven { over, .. } if sk.parameters.get(over)
                .is_some_and(|p| p.dim != crate::units::Dim::ANGLE) => continue,
            // a frame along a path is read over its path's parameter, which need not reach 0
            crate::model::MotionDef::Guided { .. } => crate::motion::Family::read(sk,i).map(|_| ()),
            // a frame the drawing moves is read over its parameter, which need not reach 0
            crate::model::MotionDef::Followed { .. } => crate::motion::Family::read(sk,i).map(|_| ()),
            _ => crate::motion::evaluate(sk,i,0.).map(|_| ()),
        };
        if let Err(message) = read {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Motion,i));
            diags.push(Diag::new(Code::E080,site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),message));
        }
    }
    // a profile cut under a frame the drawing moves, read in its plane (#398 F)
    for i in 0..sk.curves.len() {
        let crate::model::CurveBody::Envelope(g) = &sk.curve_defs[sk.curves[i].def as usize].body
        else { continue };
        if let Err(message) = sk.generated_flat(i,&g.flat) {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Curve,i));
            diags.push(Diag::new(Code::E080,site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),message));
        }
    }
    diags.extend(views::degenerate(sk, map));
    diags.extend(views::coplanar(sk, map));
    for i in 0..sk.envelopes.len() {
        // a set's envelope has no side to read after the solve: its contacts are its rows (#283)
        if sk.envelopes[i].surface.is_none() { continue; }
        if let Err(message) = crate::envelope::GeneratedEnvelope::named(sk,i) {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Envelope,i));
            diags.push(Diag::new(Code::E080,site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),message));
        }
    }
    diags
}
pub use lift::{dumps, to_program};
pub use source_map::{public_path, Elaborated, InstPath, Made, Site, SourceMap};

use crate::expr;
use crate::model::{EntRef, Sketch};
use crate::syntax::{Name, Program, Span, Stmt, StmtId, StmtKind};
use elaborate::{elaborate_in, Elab};
pub(crate) use entities::child_names;
use entities::Deferred;
pub(crate) use lift::{holds_except, lift_decl, lift_gauge, lift_relation, point_len};
pub(crate) use planes::{drawn_plane, plane_of_entity, plane_of_entity_by};
pub(crate) use relations::written_zero;
pub(crate) use resolve::NORMAL;
use std::collections::{BTreeMap, BTreeSet};

/// Warn when a declaration shadows a built-in name; expressions still read the
/// built-in. Check nested bodies and formals as well as root declarations.
fn shadowing(p: &Program, diags: &mut Vec<Diag>) {
    fn say(name: &Name, what: &str, stmt: Option<StmtId>, diags: &mut Vec<Diag>) {
        let Some(kind) = expr::builtin(&name.text) else { return };
        diags.push(Diag::new(
            Code::W112,
            name.span,
            stmt,
            format!(
                "`{}` is {kind}, and {what} of that name does not shadow it — an expression \
                 reads the built-in wherever a number is worked out (§3.3), so this name is \
                 read two ways.  Rename it.",
                name.text
            ),
        ));
    }
    fn body(stmts: &[Stmt], diags: &mut Vec<Diag>) {
        for st in stmts {
            match &st.kind {
                StmtKind::Param(d) => say(&d.name, "a `param`", Some(st.id), diags),
                StmtKind::Group(d) => say(&d.name, "a `group`", Some(st.id), diags),
                StmtKind::Set(d) => say(&d.name, "a set", Some(st.id), diags),
                StmtKind::Block(b) => {
                    if let Some(i) = &b.binder {
                        say(i, "a block's index", Some(st.id), diags);
                    }
                    for (var, _) in b.over.iter().flat_map(|o| o.walks()) {
                        say(var, "a block's edge", Some(st.id), diags);
                    }
                    body(&b.body, diags);
                }
                _ => {}
            }
        }
    }
    // the root stands among the components, and a module's components with it; what a module's
    // own body adds is its exported top-level values (§6.3)
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

/// **A component defined twice in the document is E071**, as one defined twice in a module is
/// (`modules::link`): the first stands, so every call still reads one definition, and the second
/// is said where it is written.  Asked of the text, like `shadowing`.
fn defined_twice(p: &Program, diags: &mut Vec<Diag>) {
    let mut first: BTreeMap<&str, Span> = BTreeMap::new();
    for c in p.components.iter().filter(|c| c.module.is_none()) {
        let Some(n) = &c.name else { continue };
        match first.get(n.text.as_str()) {
            Some(&was) => diags.push(defined_again(p, n, was)),
            None => {
                first.insert(&n.text, n.span);
            }
        }
    }
}

/// E071 at `n`, defined in its file already at `was`.
pub(crate) fn defined_again(p: &Program, n: &crate::syntax::Name, was: Span) -> Diag {
    let line = p.line_col(was.lo as usize).0;
    let message = format!("`{}` is defined twice; the first is at line {line}", n.text);
    Diag::new(Code::E071, n.span, None, message)
}

pub fn elaborate(p: &Program) -> Elaborated {
    // a set the flattener drew as an element that is none here is walked again as a set, until
    // every element drawn is one (`lowering`): whether one is can rest on what another set's use
    // draws, so it is judged after the expansion, and a pass ends at the judgment, before any
    // constraint is stated.  Each pass refuses at least one more set, and a refusal is final
    let mut refused = BTreeSet::new();
    // **a bound chooses the seed's side** (§9.6): where the seeds break one, the point it
    // measures is carried across its edge and the document elaborated again from there, its
    // numbers written over its seed — as if the source had seeded it on the bound's side, so
    // what is seeded from it, the views folded through it and every seed read through those
    // views stand where they would have.  Once: a seed is where a solve begins, and the solve
    // steers what is left (`System::steer`)
    let mut crossed = BTreeMap::new();
    // **a block over a solid's faces** (#183 F4) runs over them once the solid is built: its
    // faces named, and the document elaborated again
    let mut faces = BTreeMap::new();
    // named again while a pass's solids give them otherwise — a round over another's faces, once
    // that one is made (#183 F4, [0.149]) — but no more than a few times: rounds that name each
    // other, refused, would give more every pass
    let mut renamed = 0;
    loop {
        match elaborate_in(p, &refused, &crossed, &faces, renamed >= FACE_PASSES) {
            Ok(e) => {
                if crossed.is_empty() {
                    crossed = crossed_seeds(&e);
                    if !crossed.is_empty() {
                        continue;
                    }
                }
                return e;
            }
            Err(Again::Refused(more)) => refused.extend(more),
            Err(Again::Faces(more)) => {
                renamed += more.keys().filter(|k| faces.contains_key(*k)).count().min(1);
                faces.extend(more);
            }
        }
    }
}

/// How many passes may name a solid's faces again (`elaborate`): one for each round over the faces
/// of another, nested.
const FACE_PASSES: usize = 8;

/// Why a pass of `elaborate` ends before a drawing is made: sets drawn as elements that are
/// none, or solids a block runs over the faces of, named.
enum Again {
    Refused(BTreeSet<String>),
    Faces(BTreeMap<String, Vec<crate::solid::faces::FacePath>>),
}

/// The points whose seeds break a bound, carried across (`solve::cross_bounds`): each by the key
/// it is bound under, with the numbers it now has.
fn crossed_seeds(e: &Elaborated) -> BTreeMap<String, Vec<f64>> {
    // read before anything is copied: most drawings' seeds break none
    let tol = crate::solve::bound_tol(&e.sketch);
    let reads = |c| crate::solve::bound_reading(&e.sketch, c, tol);
    if !e.sketch.constraints.iter().any(|c| reads(c).is_some_and(|r| !r.holds)) {
        return BTreeMap::new();
    }
    let mut sk = e.sketch.clone();
    let moved = crate::solve::cross_bounds(&mut sk);
    moved.into_iter().filter_map(|p| {
        let ent = EntRef::point(p);
        let key = e.map.keys().find(|&(_, x)| x == ent).map(|(k, _)| k.clone())?;
        Some((key, sk.own_params(ent).iter().map(|&i| sk.params[i as usize].value).collect()))
    }).collect()
}

/// An angle a declaration is bounded by (`from:`, `to:`), in degrees: written as an angle,
/// bound to a number and finite, each refused naming `what` — a surface's span, an envelope's
/// roll.  The one reader of such a bound.
fn bound_angle(sk: &Sketch, a: &crate::syntax::Arg, what: &str) -> Result<f64, String> {
    let crate::syntax::Arg::Dim { text, .. } = a else { return Err(format!("a {what} needs an angle")) };
    let value = crate::flatten::value_aff(text, &BTreeMap::new(), sk.units)?;
    value.dim.require(crate::units::Dim::ANGLE, what)?;
    let value = value.number().ok_or_else(|| format!("a {what} must be bound"))?;
    if !value.is_finite() {
        return Err(format!("a {what} must be finite"));
    }
    Ok(value)
}
