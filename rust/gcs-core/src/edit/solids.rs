//! A gesture that makes a solid: a face, a sweep, an extrusion or a revolution and its size,
//! the body rule, rounds and chamfers.

use super::*;

/// A face bounded by drawn edges, `edges` in walk order, with holes.  A hole that is one circle
/// is named as it stands; any other loop is written first as a face of its own (`f1 :=
/// face(e, f, g, h)`), since a face is a hole as it stands (§6.8).  `names` is the face, then
/// each hole face written for it.
pub fn add_face(
    prog: &Program,
    edges: &[String],
    holes: &[Vec<String>],
    name: Option<&str>,
) -> Edit {
    Edit::refused_or(prog, face_edit(prog, edges, holes, name))
}

fn face_edit(
    prog: &Program,
    edges: &[String],
    holes: &[Vec<String>],
    name: Option<&str>,
) -> Result<Edit, String> {
    let (stmts, names) = face_stmts(&mut taken_names(prog), edges, holes, name)?;
    append_checked(prog, &stmts, names)
}

/// A face's statements — each hole loop that is not one circle written first as a face of its
/// own — and their names, the face first.
fn face_stmts(
    taken: &mut std::collections::BTreeSet<String>,
    edges: &[String],
    holes: &[Vec<String>],
    name: Option<&str>,
) -> Result<(Vec<StmtKind>, Vec<String>), String> {
    if edges.is_empty() || holes.iter().any(Vec::is_empty) {
        return Err("a face and each of its holes name the edges they are bounded by".into());
    }
    let face = chosen_name(taken, name, EntKind::Face)?;
    let mut stmts = Vec::new();
    let mut names = vec![face.clone()];
    let mut hole_refs = Vec::new();
    for hole in holes {
        // a lone edge is a whole loop — a circle, a closed curve — and a hole as it stands
        if hole.len() == 1 {
            hole_refs.push(hole[0].clone());
            continue;
        }
        let h = next_name(taken, EntKind::Face);
        stmts.push(StmtKind::Decl(gesture_decl(EntKind::Face, &h, vec![refs(hole)], &[])));
        names.push(h.clone());
        hole_refs.push(h);
    }
    let d = gesture_decl(EntKind::Face, &face, vec![refs(edges), refs(&hole_refs), Vec::new()], &[]);
    stmts.push(StmtKind::Decl(d));
    Ok((stmts, names))
}

/// How a gesture sweeps a face: along its normal (`depth`, or `from` and `to`), through a body,
/// or turned `about` a line (`sweep` and `sense` beside it).  Extents are the text written —
/// `"20"`, `"20mm"`, `"90deg"` — so a document without a `unit` stays without one.
#[derive(Clone, Debug, Default)]
pub struct SolidSweep {
    pub depth: Option<String>,
    pub from: Option<String>,
    pub to: Option<String>,
    pub through: Option<String>,
    pub about: Option<String>,
    pub sweep: Option<String>,
    pub sense: Option<syntax::Sense>,
}

/// A solid swept from `face`: `b0 := solid(f0, depth: 20)`.  A mixture of sweeps is refused in
/// the parser's own words (`sweep_of`), and a sweep the elaborator refuses (E080, E081) in its.
pub fn add_solid(prog: &Program, face: &str, how: &SolidSweep, name: Option<&str>) -> Edit {
    Edit::refused_or(prog, solid_edit(prog, face, how, name))
}

fn solid_edit(prog: &Program, face: &str, how: &SolidSweep, name: Option<&str>) -> Result<Edit, String> {
    let sweep = how.sweep()?;
    let name = chosen_name(&mut taken_names(prog), name, EntKind::Solid)?;
    append_checked(prog, &[solid_stmt(&name, face, sweep)], vec![name])
}

/// `name := solid(face, …sweep)`.
fn solid_stmt(name: &str, face: &str, sweep: syntax::Sweep) -> StmtKind {
    let mut d = gesture_decl(EntKind::Solid, name, vec![refs([face])], &[]);
    d.sweep = Some(sweep);
    StmtKind::Decl(d)
}

impl SolidSweep {
    /// The sweep these say, as the parser reads one (`sweep_of`): a mixture is refused in its
    /// words, and so is none at all, since a solid swept from a face says how.
    fn sweep(&self) -> Result<syntax::Sweep, String> {
        let dim = |t: &Option<String>| {
            t.as_ref().map(|text| syntax::Arg::Dim { text: text.clone(), span: Span::default() })
        };
        let reference = |t: &Option<String>| t.as_ref().map(|n| syntax::Ref::new(n.clone()));
        let parts = syntax::SweepParts {
            from: dim(&self.from),
            to: dim(&self.to),
            depth: dim(&self.depth),
            about: reference(&self.about),
            through: reference(&self.through),
            sweep: dim(&self.sweep),
            sense: self.sense,
            ..Default::default()
        };
        match syntax::sweep_of(parts)? {
            syntax::Sweep::Body => Err("a solid swept from a face says how: `depth:`, \
                                        `from:`/`to:`, `through:` or `about:`".into()),
            s => Ok(s),
        }
    }

    /// An extrusion `thick` thick (the text written) the `way` it runs from its face: `depth:`
    /// behind, `from: 0, to:` in front, or half each way — halved as a number where it is one,
    /// else as the expression `(thick) / 2`.
    pub fn thick(way: crate::model::Way, thick: &str) -> SolidSweep {
        use crate::model::Way;
        let (thick, n) = (thick.trim(), thick.trim().parse::<f64>().ok().filter(|n| n.is_finite()));
        let half = |sign: f64| match n {
            Some(n) => num(sign * n / 2.0),
            None => format!("{}({thick}) / 2", if sign < 0.0 { "-" } else { "" }),
        };
        let text = |t: &str| Some(t.to_string());
        match way {
            Way::Behind => SolidSweep { depth: text(thick), ..Default::default() },
            Way::Front => SolidSweep { from: text("0"), to: text(thick), ..Default::default() },
            Way::Both => {
                SolidSweep { from: Some(half(-1.0)), to: Some(half(1.0)), ..Default::default() }
            }
        }
    }
}

/// **Extrude** (#162 F2): a click inside a region, as one splice — the face over its loops
/// (`face_stmts`), the solid swept from it, and, with a body selected, the body rule:
/// `f0 := face(…)`, `b0 := solid(f0, depth: D)`, `b0 union body`.  `with` is the word and the
/// body; `through` sweeps the face through that body instead of to a depth (a cut through all).
/// The depth is a round fifth of the drawing's reach, `set_off`'s, a start the gesture then
/// sizes.  `names` is the solid, the face, then any hole faces.
pub fn extrude(
    prog: &Program,
    sk: &Sketch,
    outer: &[String],
    holes: &[Vec<String>],
    with: Option<(syntax::BodyWord, &str)>,
    through: bool,
) -> Edit {
    Edit::refused_or(prog, extrude_edit(prog, sk, outer, holes, with, through))
}

fn extrude_edit(
    prog: &Program,
    sk: &Sketch,
    outer: &[String],
    holes: &[Vec<String>],
    with: Option<(syntax::BodyWord, &str)>,
    through: bool,
) -> Result<Edit, String> {
    let sweep = match (through, with) {
        (true, Some((_, body))) => SolidSweep { through: Some(body.to_string()), ..Default::default() },
        (true, None) => return Err("a cut through all goes through a body: select it first".into()),
        (false, _) => SolidSweep { depth: Some(syntax::num(set_off(sk))), ..Default::default() },
    };
    region_solid(prog, outer, holes, &sweep, with)
}

/// **Revolve** (#162 F3): a click inside a region and the line or axis to turn it about, as one
/// splice — `f0 := face(…)`, `b0 := solid(f0, about: l)`, a full turn, and with a body selected
/// the body rule, as `extrude` writes it.  An axis that does not lie in the face's plane, or a
/// line drawn in another, is refused in the elaborator's words (E081).  `names` is the solid, the
/// face, then any hole faces.
pub fn revolve(
    prog: &Program,
    outer: &[String],
    holes: &[Vec<String>],
    about: &str,
    with: Option<(syntax::BodyWord, &str)>,
) -> Edit {
    let how = SolidSweep { about: Some(about.to_string()), ..Default::default() };
    Edit::refused_or(prog, region_solid(prog, outer, holes, &how, with))
}

/// A region's face, the solid swept from it `how`, and the body rule `with` says, in one splice.
fn region_solid(
    prog: &Program,
    outer: &[String],
    holes: &[Vec<String>],
    how: &SolidSweep,
    with: Option<(syntax::BodyWord, &str)>,
) -> Result<Edit, String> {
    let mut taken = taken_names(prog);
    let (mut stmts, faces) = face_stmts(&mut taken, outer, holes, None)?;
    let solid = chosen_name(&mut taken, None, EntKind::Solid)?;
    stmts.push(solid_stmt(&solid, &faces[0], how.sweep()?));
    if let Some((word, body)) = with {
        stmts.push(StmtKind::SolidRel(syntax::SolidRel {
            word,
            what: syntax::Ref::new(solid.clone()),
            body: syntax::Ref::new(body.to_string()),
            span: Span::default(),
        }));
    }
    let mut names = vec![solid];
    names.extend(faces);
    append_checked(prog, &stmts, names)
}

/// Rewrite how a solid is swept — `depth: 12`, `from: 0, to: 12`, `from: -6, to: 6` — over the
/// same face, its bracket list replaced and nothing else in the file touched.  The solid's own
/// statement, at the root: one a component or a block made is no statement to rewrite.
pub fn set_sweep(e: &Elaborated, prog: &Program, solid: usize, how: &SolidSweep) -> Edit {
    Edit::refused_or(prog, sweep_edit(e, prog, solid, how))
}

fn sweep_edit(e: &Elaborated, prog: &Program, solid: usize, how: &SolidSweep) -> Result<Edit, String> {
    // a draft written on the solid stays: the gesture sizes the extrusion, never its taper
    resweep(e, prog, solid, |was| {
        let mut s = how.sweep()?;
        if let Some(slot) = s.draft_mut() { *slot = was.draft().cloned(); }
        Ok(s)
    })
}

/// Rewrite how far revolution `solid` turns, about what it turns about: `angle` the text written,
/// in degrees.  A number is signed, positive right-handed about the axis, and read by
/// `model::turn_of` — `-90` is `sweep: 90, sense: cw`, a whole turn or more no `sweep:` at all;
/// an expression is written whole, the way the solid already turns.
pub fn set_turn(e: &Elaborated, prog: &Program, solid: usize, angle: &str) -> Edit {
    Edit::refused_or(prog, turn_edit(e, prog, solid, angle))
}

fn turn_edit(e: &Elaborated, prog: &Program, solid: usize, angle: &str) -> Result<Edit, String> {
    let angle = angle.trim();
    resweep(e, prog, solid, |was| {
        let syntax::Sweep::Revolve { axis, sense, .. } = was else {
            return Err("that solid is not turned about anything".into());
        };
        let (sweep, sense) = match angle.parse::<f64>() {
            Ok(n) => {
                let (how_far, sense) = crate::model::turn_of(n).ok_or("a turn turns some way")?;
                let sense = match sense {
                    crate::model::Sense::Cw => syntax::Sense::Cw,
                    crate::model::Sense::Ccw => syntax::Sense::Ccw,
                };
                (how_far.map(num), sense)
            }
            Err(_) => (Some(angle.to_string()), *sense),
        };
        let sweep = sweep.map(|text| syntax::Arg::Dim { text, span: Span::default() });
        Ok(syntax::Sweep::Revolve { axis: axis.clone(), sweep, sense })
    })
}

/// Solid `solid`'s own statement, its sweep replaced by what `with` makes of the one written —
/// its bracket list rewritten and nothing else in the file touched.  At the root: one a component
/// or a block made is no statement to rewrite.
fn resweep(
    e: &Elaborated,
    prog: &Program,
    solid: usize,
    with: impl FnOnce(&syntax::Sweep) -> Result<syntax::Sweep, String>,
) -> Result<Edit, String> {
    let site = e.map.of_entity.get(&EntRef::solid(solid)).ok_or("no statement makes that solid")?;
    if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
        return Err("that solid comes from a component or a block: change it there".into());
    }
    let d = decl_of(prog, site).filter(|d| d.kind == EntKind::Solid && d.sweep.is_some())
        .ok_or("that solid is made of others, not swept from a face")?;
    let mut d2 = d.clone();
    d2.sweep = Some(with(d.sweep.as_ref().expect("a swept solid"))?);
    let args = syntax::decl_args(&d2);
    let stmt = prog.stmt(site.stmt).map(|s| s.span).ok_or("no statement makes that solid")?;
    // the statement rewritten, so that where it lands is the text this edit wrote
    let edit = Splice::within(prog.text(), stmt, d.list_span, &args);
    let (text, landed) = splice_landing(prog.text(), vec![edit]);
    checked(prog, text, landed[0].ok_or("that solid's statement is no place in the source")?,
        Vec::new())
}

/// `what union body`, `what cut body`, `what common body` — the body rule, a statement for each
/// solid in `what`, written as one splice (§6.9): a Boolean gesture joins several tools at once.
pub fn add_body_word(prog: &Program, word: syntax::BodyWord, what: &[&str], body: &str) -> Edit {
    if what.is_empty() {
        return Edit::none(prog, Some(format!("nothing to {} `{body}`", word.as_str())));
    }
    let stmts: Vec<StmtKind> = what.iter().map(|w| StmtKind::SolidRel(syntax::SolidRel {
        word,
        what: syntax::Ref::new(w.to_string()),
        body: syntax::Ref::new(body.to_string()),
        span: Span::default(),
    })).collect();
    Edit::refused_or(prog, append_checked(prog, &stmts, Vec::new()))
}

/// The library's two words a gesture writes along a solid's edges (`std.sv`): `round`, rounded
/// by a ball of radius `r`, and `chamfer`, cut by a flat `d` along each face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeWord {
    Round,
    Chamfer,
}

impl EdgeWord {
    pub fn parse(s: &str) -> Option<EdgeWord> {
        match s {
            "round" => Some(EdgeWord::Round),
            "chamfer" => Some(EdgeWord::Chamfer),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            EdgeWord::Round => "round",
            EdgeWord::Chamfer => "chamfer",
        }
    }

    /// The word's one number, by its label: `round(r: 2mm)`, `chamfer(d: 1mm)`.
    fn key(self) -> &'static str {
        match self {
            EdgeWord::Round => "r",
            EdgeWord::Chamfer => "d",
        }
    }
}

/// **Round or chamfer edges** (the Fillet and Chamfer tools): edges of object `solid`, picked as
/// whole faces (every edge of each) and as single edges (the two faces meeting there, as
/// `mesh::face_edges` names them), written as the library's words over face paths, `size` the
/// radius or the leg as typed:
///
/// ```text
/// use std (round)
/// block.near round(r: 2) block       // a face: every edge of it
/// block.far round(r: 2) block.west   // an edge: where the two faces meet
/// round(r: 2) block                  // every face: every edge, its corners a ball's
/// ```
///
/// Each face once and each edge no face already gives once, so a chain of edges is a statement a
/// link; which way each turns (a convex edge cut, a concave one filled) is the word's to read.
/// `use std` is brought in naming the word, or made to name it.  One splice, held to the
/// elaborator: a face with no name the source can write, faces that do not meet, two objects, or
/// a size the language refuses is refused in its words.
pub fn add_rounds(
    prog: &Program,
    sk: &Sketch,
    solid: usize,
    word: EdgeWord,
    faces: &[String],
    edges: &[(String, String)],
    size: &str,
) -> Edit {
    Edit::refused_or(prog, rounds_edit(prog, sk, solid, word, faces, edges, size))
}

fn rounds_edit(
    prog: &Program,
    sk: &Sketch,
    solid: usize,
    word: EdgeWord,
    faces: &[String],
    edges: &[(String, String)],
    size: &str,
) -> Result<Edit, String> {
    let object = sk.solids.get(solid).ok_or("there is no such solid")?;
    let body = object.name.clone();
    let size = size.trim();
    if size.is_empty() {
        return Err(format!("a {} says how big: {}", word.as_str(), match word {
            EdgeWord::Round => "its radius",
            EdgeWord::Chamfer => "how far its flat runs along each face",
        }));
    }
    if faces.is_empty() && edges.is_empty() {
        return Err(format!("nothing to {}: pick edges or faces of `{body}`", word.as_str()));
    }
    let evaluated = sk.evaluated_solid(solid, crate::solid::ApproximationPolicy::Mesh)?;
    let meets = evaluated.face_edges();
    // the faces the document can name, by their paths from the object (`face_paths`, the walk the
    // words' own `repeat` reads)
    let named: std::collections::BTreeSet<String> = crate::solid::faces::face_paths(sk, solid)
        .iter().map(|f| f.text()).filter(|p| !p.contains('#')).collect();
    // every face of the object, by its mesh's path, and as the source writes it where it can
    let all: Vec<(&str, Option<String>)> = evaluated.mesh().groups.iter().map(|g| {
        (g.path.as_str(), evaluated.face_route(&g.path).filter(|r| named.contains(r)))
    }).collect();
    let written = |f: &str| -> Result<String, String> {
        match all.iter().find(|(p, _)| *p == f) {
            None => Err(format!("`{f}` is no face of `{body}`")),
            Some((_, None)) => {
                Err(format!("this face of `{body}` has no name the document can write"))
            }
            Some((_, Some(route))) => Ok(format!("{body}.{route}")),
        }
    };
    let mut picked: Vec<(&str, String)> = Vec::new();
    for f in faces {
        if !picked.iter().any(|(p, _)| p == f) {
            picked.push((f, written(f)?));
        }
    }
    let said = |ops: &[&str]| StmtKind::Relation(syntax::Relation::of(
        syntax::RelationForm::Written(syntax::Written {
            word: syntax::Name::new(word.as_str()),
            fixity: if ops.len() == 1 {
                crate::constraints::Fixity::Prefix
            } else {
                crate::constraints::Fixity::Infix
            },
            ops: ops.iter().map(|o| syntax::Ref::new(o.to_string())).collect(),
            args: vec![syntax::OpArg::Named(
                syntax::Name::new(word.key()),
                syntax::Arg::Dim { text: size.to_string(), span: Span::default() },
            )],
            span: Span::default(),
        }),
    ));
    let mut stmts: Vec<StmtKind> = Vec::new();
    if all.iter().all(|(f, w)| w.is_none() || picked.iter().any(|(p, _)| p == f)) {
        stmts.push(said(&[&body]));
    } else {
        stmts.extend(picked.iter().map(|(_, w)| said(&[w, &body])));
        let mut seen: Vec<(&str, &str)> = Vec::new();
        for (a, b) in edges {
            let (wa, wb) = (written(a)?, written(b)?);
            // in the order `face_edges` names a pair, whichever way a caller gave it
            let pair = if a <= b { (a.as_str(), b.as_str()) } else { (b.as_str(), a.as_str()) };
            if !meets.iter().any(|e| (e.faces.0.as_str(), e.faces.1.as_str()) == pair) {
                return Err(format!("`{wa}` and `{wb}` do not meet at an edge"));
            }
            let covered = picked.iter().any(|(p, _)| *p == pair.0 || *p == pair.1);
            if !covered && !seen.contains(&pair) {
                seen.push(pair);
                stmts.push(said(&[&wa, &wb]));
            }
        }
    }
    // what the word's own body refuses is said at the `use` that brought the library in: the
    // statements and a `use` the document wrote are both what this edit wrote, where they landed
    let (import, written_use) = imports(prog, word.as_str());
    let (text, landed) = splice_landing(prog.text(), vec![appending(prog, &stmts)?, import]);
    let regions: Vec<Span> = landed.into_iter().take(if written_use { 2 } else { 1 })
        .map(|l| l.ok_or("the edit is no place in the source"))
        .collect::<Result<_, _>>()?;
    checked_in(prog, text, &regions, Vec::new())
}

/// A round or chamfer's size to start from on object `solid`: a tenth of its least extent, to one
/// significant figure — what the tools offer before anybody types.  `None` for no object.
pub fn round_size(sk: &Sketch, solid: usize) -> Option<f64> {
    let s = sk.evaluated_solid(solid, crate::solid::ApproximationPolicy::Mesh).ok()?;
    let b = s.world_bounds();
    let least = (0..3).map(|k| b.hi[k] - b.lo[k]).filter(|d| *d > 0.0)
        .fold(f64::INFINITY, f64::min);
    least.is_finite().then(|| round_sig(least / 10.0, 1))
}

/// The splice that has the document's `use std` name `word` bare (§14.4) — the whole `use std`
/// the document writes, its list grown, one written (`use std` becomes `use std (round)`) or as
/// it stands where it already does; else a line of its own where `add_use` would put one — and
/// whether the document wrote that `use`.
fn imports(prog: &Program, word: &str) -> (Splice, bool) {
    let text = prog.text();
    let Some(u) = prog.uses.iter().find(|u| u.name == "std" && (u.span.hi as usize) <= text.len())
    else {
        return (use_line(prog, &format!("use std ({word})")), false);
    };
    let (lo, hi) = (u.span.lo as usize, u.span.hi as usize);
    let (at, with) = if u.names.iter().any(|n| n.text == word) {
        (hi, String::new())
    } else {
        match u.names.last() {
            Some(n) => (n.span.hi as usize, format!(", {word}")),
            None => (lo + text[lo..hi].trim_end().len(), format!(" ({word})")),
        }
    };
    (Splice::within(text, u.span, Span::new(at, at), &with), true)
}
