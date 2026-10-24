//! **Load scenarios, elaborated** (#510 P4): each `scenario NAME { … }` into a
//! `model::Scenario` — its targets resolved by the selectors every operand is resolved by
//! (`Resolver::operand`), its numbers worked out and held to what they are — read once the
//! drawing's geometry and solids are built.  A scenario of claims adds nothing to the drawing,
//! so the solve, its counts and its diagnosis are the drawing's with it deleted; a driving
//! deflection of a frame's joint is compiled into a row of the drawing (#510 S1, `drive`).

use super::resolve::{Operand, Resolver};
use super::{Code, Diag};
use crate::ir::{Operation, Statement};
use crate::model::{
    Applied, EntKind, EntRef, Grade, InPlane, Load, LoadKind, LoadValue, Output, PlaneReading,
    Requirement, Scenario, Sketch, Support,
};
use crate::syntax::{ItemKind, LoadText, Ref, Span, StmtId};
use crate::units::Dim;
use std::collections::BTreeMap;

/// **A scenario and where each of its lines was written**: the statement and the line's index in
/// its body, for every support, load and requirement in order — an extended scenario's lines
/// are its base's, written there.
#[derive(Clone, Debug)]
pub struct Stated {
    pub scenario: Scenario,
    pub stmt: StmtId,
    pub supports: Vec<Line>,
    pub loads: Vec<Line>,
    pub requirements: Vec<Line>,
    /// Each requirement's output as written, `deflection(tip, along: down)`: what a report says
    /// it of (#510 S0).
    pub heads: Vec<String>,
    /// Each requirement's row, by its constraint's id, where it drives one (#510 S1, `drive`).
    pub driven: Vec<Option<u32>>,
    /// Where each reading in the plane was written (#510 S2): an extended scenario's are its
    /// base's.
    pub readings: Vec<Line>,
}

/// One line of a scenario's body: the scenario statement, and the line's index in it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Line {
    pub stmt: StmtId,
    pub item: usize,
}

/// Every scenario of the document, in the order written; what cannot be read is said and left
/// out, a line at a time.
pub(super) fn scenarios(
    sk: &Sketch,
    res: &Resolver,
    body: &[&Statement],
    diags: &mut Vec<Diag>,
) -> Vec<Stated> {
    let mut out: Vec<Stated> = Vec::new();
    let mut by_name: BTreeMap<String, usize> = BTreeMap::new();
    // the scenario whose requirements drive dimensions
    let mut driving: Option<String> = None;
    let read = Reading { sk, res };
    for st in body {
        let Operation::Scenario(sc) = &st.kind else { continue };
        if by_name.contains_key(&sc.name.text) {
            diags.push(Diag::within(
                Code::E001,
                sc.name.span,
                st.id,
                format!("a scenario `{}` is stated already", sc.name.text),
            ));
            continue;
        }
        let mut made = Stated {
            scenario: Scenario {
                name: sc.name.text.clone(),
                extends: None,
                supports: Vec::new(),
                loads: Vec::new(),
                requirements: Vec::new(),
                readings: Vec::new(),
            },
            stmt: st.id,
            supports: Vec::new(),
            loads: Vec::new(),
            requirements: Vec::new(),
            heads: Vec::new(),
            driven: Vec::new(),
            readings: Vec::new(),
        };
        if let Some(base) = &sc.extends {
            match by_name.get(&base.text) {
                Some(&k) => {
                    let b = &out[k];
                    made.scenario.extends = Some(k);
                    made.scenario.supports = b.scenario.supports.clone();
                    made.scenario.loads = b.scenario.loads.clone();
                    made.supports = b.supports.clone();
                    made.loads = b.loads.clone();
                    made.scenario.readings = b.scenario.readings.clone();
                    made.readings = b.readings.clone();
                }
                None => diags.push(Diag::within(
                    Code::E101,
                    base.span,
                    st.id,
                    format!(
                        "no scenario `{}` is stated before this one: a scenario extends one \
                         written above it",
                        base.text
                    ),
                )),
            }
        }
        for (k, it) in sc.items.iter().enumerate() {
            let line = Line { stmt: st.id, item: k };
            let got = match &it.kind {
                ItemKind::Support { hold, at, along } => read.support(*hold, at, along.as_ref())
                    .map(|s| {
                        made.scenario.supports.push(s);
                        made.supports.push(line);
                    }),
                ItemKind::Load { kind, at, value } => read.load(*kind, at, value).map(|l| {
                    made.scenario.loads.push(l);
                    made.loads.push(line);
                }),
                ItemKind::Requirement(t) => read.requirement(t).map(|r| {
                    made.scenario.requirements.push(r);
                    made.requirements.push(line);
                    let mut head = String::new();
                    t.write_head(&mut head);
                    made.heads.push(head);
                    made.driven.push(None);
                }),
                ItemKind::Reading { plane, of } => read.reading(*plane, of).and_then(|r| {
                    let solid = r.of.solid();
                    match made.scenario.readings.iter().position(|o| o.of.solid() == solid) {
                        // a reading of its own replaces its base's
                        Some(k) if made.readings[k].stmt != st.id => {
                            made.scenario.readings[k] = r;
                            made.readings[k] = line;
                            Ok(())
                        }
                        Some(_) => Err((Code::E001, Some(of.span), format!(
                            "`{}` is read in the plane already in this scenario",
                            crate::syntax::ref_text(of)))),
                        None => {
                            made.scenario.readings.push(r);
                            made.readings.push(line);
                            Ok(())
                        }
                    }
                }),
            };
            if let Err((code, span, message)) = got {
                diags.push(Diag::within(code, span.unwrap_or(it.span), st.id, message));
            }
        }
        // one scenario's requirements drive dimensions, until requirements across scenarios
        // are kept together by an optimiser (#510 S4)
        let first = made.scenario.requirements.iter().position(|r| r.grade() == Grade::Driving);
        if let Some(k) = first {
            let span = sc.items[made.requirements[k].item].span;
            match &driving {
                None => driving = Some(sc.name.text.clone()),
                Some(other) => diags.push(Diag::within(
                    Code::E106,
                    span,
                    st.id,
                    format!(
                        "scenario `{other}` drives dimensions already, and one scenario's \
                         requirements may drive them until an optimiser keeps every \
                         scenario's together: state this one a claim, `claim …`"
                    ),
                )),
            }
        }
        by_name.insert(sc.name.text.clone(), out.len());
        out.push(made);
    }
    out
}

/// **Driving requirements compiled into rows** (#510 S1): every `deflection(p, along: d) == v` of
/// a frame's joint `p` (`beams::of_frame`), or (#510 S2) of a point of a region the scenario
/// reads in its plane (`prisms::of_region`), or (#510 S3) of a point of a solid it reads in three
/// dimensions (`bodies::of_body`), becomes a `CKind::Deflection` of the drawing, solved
/// under its scenario's supports and loads, which the sketch holds for it (`Sketch::scenarios`);
/// the row stands where the requirement is written.  One the solve cannot read — a frame or a
/// region the supports leave free, a direction the solve moves — is **E040** there.  Any other
/// driving requirement (a solid's that is no prism; a mode or a safety) compiles nothing yet and
/// is judged after the solve, as a claim is.
pub(super) fn drive(
    sk: &mut Sketch,
    map: &mut super::source_map::SourceMap,
    body: &[&Statement],
    scenarios: &mut [Stated],
    diags: &mut Vec<Diag>,
) {
    use crate::constraints::{Arg, CKind, Constraint};
    stand(sk, scenarios);
    let mut added = false;
    for s in scenarios.iter_mut() {
        let mut index: Option<i64> = None;
        for k in 0..s.scenario.requirements.len() {
            let r = &s.scenario.requirements[k];
            if r.grade() != Grade::Driving || !drives(sk, &s.scenario, r) {
                continue;
            }
            let line = s.requirements[k];
            let Some(st) = body.iter().find(|st| st.id == line.stmt) else { continue };
            let span = match &st.kind {
                Operation::Scenario(sc) => sc.items.get(line.item).map_or(st.span, |it| it.span),
                _ => st.span,
            };
            let at = *index.get_or_insert_with(|| {
                sk.scenarios.push(s.scenario.clone());
                sk.scenarios.len() as i64 - 1
            });
            let (Applied::Ent(p), Some(along)) = (&r.of, r.along) else { continue };
            let args = vec![Arg::Ent(*p), Arg::Ent(along), Arg::Int(at), Arg::Num(r.value)];
            let name =
                |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| crate::model::entity_name(e));
            // a region's or a solid's row is meshed once, here, and read so through every solve
            // (#510 S2, S3): held before it is validated, so the validation reads that mesh
            let row = Constraint::new(CKind::Deflection, args.clone());
            let key = row.meshed_key(sk);
            if let (Some(key), Ok(crate::constraints::InteriorRow::Meshed(row))) =
                (key, row.made_row(sk))
            {
                sk.meshed.insert(key, row);
            }
            if let Err(m) = crate::constraints::validate(sk, CKind::Deflection, &args, &name) {
                if let Some(key) = key {
                    sk.meshed.remove(&key);
                }
                diags.push(Diag::within(Code::E040, span, line.stmt, format!(
                    "`{}` cannot drive the drawing: {m}", s.heads[k])));
                continue;
            }
            let id = sk.add_quiet(Constraint::new(CKind::Deflection, args));
            // made by the scenario's statement, and stood where its requirement is written
            map.record(st, super::source_map::Made::Con(id));
            if let Some(site) = map.of_constraint.get_mut(&id) {
                site.span = span;
            }
            s.driven[k] = Some(id);
            added = true;
        }
    }
    // the unknowns the frames' sections are read over are bound once the rows read them
    if added {
        crate::expr::evaluate(sk);
    }
}

/// **A drawing whose seeds leave a solid unbuilt stands where it solves before its rows are made**
/// (#567): a driving row over a filled body is meshed where the drawing stands, off the body's
/// exact boundary, which a drawing that stands only at its seeds may not close — the engine's rod,
/// seeded view by view, until its views are solved together.  Where a driving deflection of a
/// point compiles no row where the drawing stands and a solid made of a material is not built
/// there, a copy of the drawing is solved, and where that settles the drawing stands where it did,
/// so the rows are made — and their meshes moved by the solve — from there.  Natively alone: the
/// page's core drives no filled body (`bodies::DRIVES_FILLED`).
fn stand(sk: &mut Sketch, scenarios: &[Stated]) {
    // the page's core drives no filled body, so it has none to stand for
    let wanted = crate::bodies::DRIVES_FILLED && scenarios.iter().any(|s| s.scenario.requirements.iter().any(|r| {
        r.grade() == Grade::Driving && r.output == Output::Deflection && r.along.is_some()
            && matches!(&r.of, Applied::Ent(p) if p.kind == EntKind::Point)
            && !drives(sk, &s.scenario, r)
    }));
    let unbuilt = || (0..sk.solids.len()).any(|s| {
        sk.solids[s].material.is_some() && crate::solid::faces::BodyFaces::whole(sk, s).is_err()
    });
    if !wanted || !unbuilt() {
        return;
    }
    let mut stood = sk.clone();
    if crate::solve::solve(&mut stood, crate::solve::SolveOpts::default()).success {
        sk.set_x(&stood.get_x());
    }
}

/// **Whether a driving requirement compiles to a row**: a deflection along a direction of a
/// frame's joint, of a point of a region read in its plane or of a solid read in three
/// dimensions.
fn drives(sk: &Sketch, sc: &Scenario, r: &Requirement) -> bool {
    match (r.output, &r.of) {
        (Output::Deflection, Applied::Ent(p)) => {
            p.kind == EntKind::Point && r.along.is_some()
                && (crate::beams::of_frame(sk, sc, &r.of)
                    || crate::prisms::of_region(sk, sc, &r.of)
                    || crate::bodies::of_body(sk, sc, &r.of))
        }
        _ => false,
    }
}

/// A line's fault: its code, where (the line's own span where `None`) and what.
type Fault = (Code, Option<Span>, String);

/// What a scenario's lines are read against.
struct Reading<'a> {
    sk: &'a Sketch,
    res: &'a Resolver,
}

impl Reading<'_> {
    fn support(
        &self,
        hold: crate::model::Hold,
        at: &Ref,
        along: Option<&Ref>,
    ) -> Result<Support, Fault> {
        let at = self.applied(at, hold.word())?;
        if let Applied::Beam(b) = at {
            return Err((Code::E040, None, format!(
                "a support holds what a frame stands on — a point, a line — and `{}` is a beam: \
                 hold its ends",
                self.sk.beams[b].name
            )));
        }
        let along = along.map(|d| self.direction(d)).transpose()?;
        Ok(Support { hold, at, along })
    }

    fn load(&self, kind: LoadKind, at: &Ref, value: &LoadText) -> Result<Load, Fault> {
        let applied = self.applied(at, kind.word())?;
        let what = match &applied {
            Applied::Ent(e) if e.kind != EntKind::Face => Some(e.kind.a().to_string()),
            Applied::Beam(_) => Some("a beam".to_string()),
            _ => None,
        };
        if let (LoadKind::Pressure, Some(what)) = (kind, what) {
            return Err((Code::E040, Some(at.span), format!(
                "a pressure is on a face: `{}` is {what}",
                crate::syntax::ref_text(at)
            )));
        }
        let what = kind.word();
        let value = match value {
            LoadText::Vector(parts) => {
                let mut v = [0.0; 3];
                for (x, (text, span)) in v.iter_mut().zip(parts) {
                    *x = self.number(text, *span, kind.dim(), what)?;
                }
                LoadValue::Vector(v)
            }
            LoadText::Along { magnitude: (text, span), along } => LoadValue::Along {
                magnitude: self.number(text, *span, kind.dim(), what)?,
                along: self.direction(along)?,
            },
            LoadText::Scalar((text, span)) => {
                LoadValue::Scalar(self.number(text, *span, kind.dim(), what)?)
            }
        };
        Ok(Load { kind, at: applied, value })
    }

    fn requirement(&self, r: &crate::syntax::RequirementText) -> Result<Requirement, Fault> {
        let word = r.output.word();
        let of = self.applied(&r.of, word)?;
        let along = r.along.as_ref().map(|d| self.direction(d)).transpose()?;
        let count = match &r.count {
            None => None,
            Some((text, span)) => {
                let n = self.number(text, *span, Dim::SCALAR, word)?;
                if n.fract() != 0.0 || !(1.0..=1e6).contains(&n) {
                    return Err((Code::E103, Some(*span), format!(
                        "`{word}` counts from 1, and `{text}` is no whole number of them"
                    )));
                }
                Some(n as u32)
            }
        };
        let (text, span) = &r.value;
        let value = self.number(text, *span, r.output.dim(), word)?;
        Ok(Requirement { output: r.output, of, along, count, compare: r.compare, value, claim: r.claim })
    }

    /// **`plane stress B`, `plane strain B`** (#510 S2): `B` a solid that is a prism, its face
    /// swept square to its plane with no draft — what a reading in the plane reads over that face,
    /// its depth the thickness — else **E040**.
    fn reading(&self, plane: InPlane, of: &Ref) -> Result<PlaneReading, Fault> {
        let at = Some(of.span);
        let named = crate::syntax::ref_text(of);
        let solid = match self.applied(of, "plane")? {
            Applied::Ent(e) if e.kind == EntKind::Solid => e.i(),
            _ => return Err((Code::E040, at, format!(
                "a solid is read in its plane, and `{named}` is no solid"))),
        };
        match &self.sk.solids[solid].def {
            crate::model::SolidDef::Prism { draft: None, .. } => {}
            _ => return Err((Code::E040, at, format!(
                "a solid read in its plane is a prism, its face swept square to its plane \
                 (`solid(F, from: …, to: …)`), and `{named}` is not"))),
        }
        Ok(PlaneReading { plane, of: Applied::Ent(EntRef::solid(solid)) })
    }

    /// What a support, a load or an output is of: geometry a body is made of or stands on.
    fn applied(&self, r: &Ref, what: &str) -> Result<Applied, Fault> {
        let at = Some(r.span);
        let named = crate::syntax::ref_text(r);
        match self.res.operand(self.sk, r).map_err(|m| (Code::E101, at, m))? {
            Operand::Beam(b) => Ok(Applied::Beam(b)),
            Operand::Face(f) => Ok(Applied::Face(f)),
            Operand::Body(e) => Ok(Applied::Ent(e)),
            Operand::Ent(e) if material(e.kind) => Ok(Applied::Ent(e)),
            Operand::Ent(e) => Err((Code::E040, at, format!(
                "`{what}` is applied to what a body is made of or stands on — a point, an \
                 edge, a face or a solid — and `{named}` is {}",
                e.kind.a()
            ))),
            Operand::Ends(_) => Err((Code::E040, at, format!(
                "`{what}` is applied to geometry, and `{named}` is where a set ends"
            ))),
        }
    }

    /// `along: d`: an axis, or a line read as its direction.
    fn direction(&self, r: &Ref) -> Result<EntRef, Fault> {
        let at = Some(r.span);
        let e = self.res.operand(self.sk, r).map_err(|m| (Code::E101, at, m))?.ent();
        match e {
            Some(e) if matches!(e.kind, EntKind::Axis | EntKind::Line) => Ok(e),
            _ => Err((Code::E040, at, format!(
                "`along:` names a direction, an axis or a line, and `{}` is none",
                crate::syntax::ref_text(r)
            ))),
        }
    }

    /// A number the drawing states, of the dimension `want`: never an unknown of the solve,
    /// since a requirement compares with it and a load is what the solve is read under.
    fn number(&self, text: &str, span: Span, want: Dim, what: &str) -> Result<f64, Fault> {
        stated(self.sk, text, want, what, "a scenario's numbers are ones it states")
            .map_err(|(code, m)| (code, Some(span), m))
    }
}

/// **A number the drawing states**, `text` worked out of the dimension `want` (said of as
/// `what`): never an unknown of the solve (**E040**, `why` saying what the number is for), the
/// wrong dimension **E103**.  A scenario's numbers (#510 P4) and a beam's section (#510 S0).
pub(super) fn stated(sk: &Sketch, text: &str, want: Dim, what: &str, why: &str)
    -> Result<f64, (Code, String)>
{
    // an unknown the document declared is read as one, to be refused as one
    let v = crate::flatten::value_read(text, &BTreeMap::new(), &sk.declared, sk.units)
        .map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    v.dim.require(want, what).map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    match v.number() {
        Some(x) if x.is_finite() => Ok(x),
        _ => Err((Code::E040, format!("`{text}` reads an unknown of the drawing, and {why}"))),
    }
}

/// Whether a support, a load or an output may be applied to an entity of this kind: what a body
/// is made of — its points, its edges and members (whatever a point runs along), its faces and
/// its solids.
fn material(kind: EntKind) -> bool {
    kind.runs_along() || matches!(kind, EntKind::Point | EntKind::Face | EntKind::Solid)
}
