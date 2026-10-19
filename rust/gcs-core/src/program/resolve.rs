//! Resolve names and child paths, including forward references.

use crate::model::{EntKind, EntRef, FaceRef, Field, Sketch};
use crate::syntax::{Member, Ref, Seg, Span};
use std::collections::BTreeMap;

/// Preallocated entity names and declared child slots for forward references.
#[derive(Default)]
pub(super) struct Resolver {
    pub(super) of: BTreeMap<String, EntRef>,
    pub(super) declared_at: BTreeMap<String, Span>,
    /// Each declaration's child slots, as the names it wrote — what `follow_building` reads
    /// where the entity itself is not built yet (build order is per kind, so a child slot that
    /// reaches into an entity of a later kind — `t := line(p3, k.start)` with `k` an arc — has
    /// only the declaration to ask).  `None` where a slot holds a seed or nothing, since the
    /// point it mints does not exist until the parent is built.
    pub(super) kids: BTreeMap<String, Vec<Option<Ref>>>,
    /// Source-level traversals. Closed chains also resolve to faces; open chains can
    /// contribute their edges to an explicitly closed face without inventing geometry here.
    pub(super) chains: BTreeMap<String, crate::syntax::NamedChain>,
    /// Each plane's origin, by the plane's key: a point minted with the points, in statement
    /// order, so `P.origin` resolves wherever it is read — before the plane is built, as a circle
    /// about it is.
    pub(super) origins: BTreeMap<String, usize>,
    /// The planes whose normal the document reads (`Expansion::normals`), by key: each mints
    /// `P.normal` as it is built, so a later declaration may name it.
    pub(super) normals: std::collections::BTreeSet<String>,
    /// The frames whose axes or origin the document reads (`Expansion::charts`), by key: a frame
    /// declared bare among them mints its chart as it is built (#383).
    pub(super) charts: std::collections::BTreeSet<String>,
    /// The frames whose joint the document reads (`Expansion::joints`), by key (#383).
    pub(super) joints: std::collections::BTreeSet<String>,
    /// Each jointed frame's value, by the frame's index (#391 S3): the instant `fix(value == τ) f`
    /// states, as worked out text, and where it was written — what a reading of something drawn
    /// in the frame (`x in f`) is read at.  A frame stating none stands at rest, `0deg`.
    pub(super) values: BTreeMap<usize, (String, Span)>,
    /// The statements holding a joint along a path (#398 C), `g coincident f.axis`: read as the
    /// frame's motion (`motions::motions`, `MotionDef::Guided`) and stating no row.
    pub(super) guides: std::collections::BTreeSet<crate::syntax::StmtId>,
}

/// A plane's member naming its normal, `P.normal`: `u × v`, an axis (`Sketch::plane_normal`).
pub(crate) const NORMAL: &str = "normal";

/// **What an operand names, resolved once** (#188): an entity, its fields followed; the faces of
/// a solid a path past it names (`pad.plate.near`, `F.surface`); where the edge a set is ends
/// (`S.ends`, the set's own point, `flatten::sets::measured_sets`); or the solid a face path is
/// of (`A.body`).  Every reader of an operand asks this, so none meets a solid with fields or
/// matches a member's word.
#[derive(Clone, Debug, PartialEq)]
pub(super) enum Operand {
    Ent(EntRef),
    Face(FaceRef),
    Ends(EntRef),
    Body(EntRef),
}

impl Operand {
    /// The entity it is, where it is one.
    pub(super) fn ent(&self) -> Option<EntRef> {
        match self {
            Operand::Ent(e) => Some(*e),
            _ => None,
        }
    }

    /// The faces it names: a face path's, or a solid's every face where it is named bare.
    pub(super) fn faces(&self) -> Option<FaceRef> {
        match self {
            Operand::Face(f) => Some(f.clone()),
            Operand::Ent(e) if e.kind == EntKind::Solid => {
                Some(FaceRef { solid: e.i() as u32, path: Vec::new(), surface: false, smooth: false })
            }
            _ => None,
        }
    }
}

impl Resolver {
    pub(super) fn lookup(&self, r: &Ref) -> Option<EntRef> {
        self.of.get(&r.root.text).copied()
    }

    /// What `r` names (`Operand`), or why nothing.
    pub(super) fn operand(&self, sk: &Sketch, r: &Ref) -> Result<Operand, String> {
        let e = self.lookup(r).ok_or_else(|| format!("no such entity: `{}`", r.root.text))?;
        operand_at(sk, e, &r.path)
    }

    /// The entity `r` names, its fields followed — `None` for anything else.
    pub(super) fn entity(&self, sk: &Sketch, r: &Ref) -> Option<EntRef> {
        self.operand(sk, r).ok()?.ent()
    }

    /// An absolute dotted name as the flattener wrote it (`t1.pin`, `pair.gen_g.p2`): an entity's
    /// absolute name is dotted itself, so the entity is the longest head that names one, and the
    /// rest is fields into it for `follow`.
    pub(super) fn dotted(&self, segs: &[&str]) -> Option<(EntRef, Vec<Seg>)> {
        (1..=segs.len()).rev().find_map(|k| {
            let fields = segs[k..].iter().map(|f| Seg::Field(crate::syntax::Name::new(*f)));
            self.of.get(&segs[..k].join(".")).map(|e| (*e, fields.collect()))
        })
    }

    /// The name declaration `name` (of kind `kind`) wrote in its child field `f`, mirroring
    /// `follow`'s reading of the built entity — the same fields, the same refusals.
    fn kid(&self, name: &str, kind: EntKind, f: &str) -> Result<Option<Ref>, String> {
        let slots = self.kids.get(name).ok_or_else(|| format!("no such entity: `{name}`"))?;
        let mut at = 0usize;
        for (n, k) in kind.fields() {
            match k {
                Field::Scalar => {}
                Field::Child => {
                    if *n == f {
                        return Ok(slots.get(at).cloned().flatten());
                    }
                    at += 1;
                }
                Field::List => {
                    if *n == f {
                        return Err(format!("`{f}` is a list, so it needs an index"));
                    }
                    at += 1;
                }
            }
        }
        let named: Vec<&str> =
            kind.fields().iter().filter(|(_, k)| *k == Field::Child).map(|(n, _)| *n).collect();
        Err(if named.is_empty() {
            format!("{} has no parts", kind.a())
        } else {
            format!("{} has {}, not `{f}`", kind.a(), named.join(", "))
        })
    }
}

/// Follow child paths while building. Consult declared slots when a referenced
/// entity has not been constructed yet.
pub(super) fn follow_building(
    sk: &Sketch,
    res: &Resolver,
    e: EntRef,
    r: &Ref,
) -> Result<EntRef, String> {
    let mut e = e;
    let mut name = r.root.text.clone();
    let mut path: Vec<Seg> = r.path.clone();
    // two declarations naming their parts through each other would walk forever; the cap is
    // far past any real document, so hitting it is the cycle
    for _ in 0..64 {
        // an envelope built before the solids (#283) has its revolution's side bound after them:
        // until then its side is read off its declaration, as an envelope not yet built is
        let unbound = e.kind == EntKind::Envelope
            && matches!(path.first(), Some(Seg::Field(f)) if f.text == "surface")
            && sk.envelopes.get(e.i()).is_some_and(|v| v.surface.is_none());
        if path.is_empty() || (e.i() < sk.count(e.kind) && !unbound) {
            return follow(sk, e, &path);
        }
        let Seg::Field(f) = &path[0] else {
            return Err("an index names a copy, not a part".to_string());
        };
        if e.kind == EntKind::Frame && f.text == "origin" {
            if let Some(&o) = res.origins.get(&name) {
                return follow(sk, EntRef::point(o), &path[1..]);
            }
        }
        if e.kind == EntKind::Frame && (f.text == NORMAL || f.text == "axis") {
            return Err(format!(
                "`{name}` is declared after this, and its {} is minted with it: declare \
                 `{name}` first", if f.text == "axis" { "joint" } else { "normal" }
            ));
        }
        let Some(kid) = res.kid(&name, e.kind, &f.text)? else {
            return Err(format!(
                "`{name}` does not name its {}, and `{name}` is not built yet: name the point \
                 itself",
                f.text
            ));
        };
        let Some(ne) = res.lookup(&kid) else {
            return Err(format!("no such entity: `{}`", kid.root.text));
        };
        e = ne;
        name = kid.root.text.clone();
        path = kid.path.iter().chain(path[1..].iter()).cloned().collect();
    }
    Err(format!("`{}` names its parts in a circle", r.root.text))
}

/// What entity `e` and the path past it name (`Resolver::operand`).
fn operand_at(sk: &Sketch, e: EntRef, path: &[Seg]) -> Result<Operand, String> {
    let (fields, member) = match path.split_last() {
        Some((Seg::Member(m), rest)) => (rest, Some(*m)),
        _ => (path, None),
    };
    match (e.kind, member) {
        (EntKind::Solid, Some(Member::Body)) => Ok(Operand::Body(e)),
        (EntKind::Solid, Some(Member::Surface)) => Ok(Operand::Face(face_ref(e, fields, true))),
        (EntKind::Solid, Some(Member::Smooth)) => {
            Ok(Operand::Face(FaceRef { smooth: true, ..face_ref(e, fields, true) }))
        }
        (EntKind::Solid, None) if !fields.is_empty() => Ok(Operand::Face(face_ref(e, fields, false))),
        (EntKind::Point, Some(Member::Ends)) if fields.is_empty() => Ok(Operand::Ends(e)),
        (kind, Some(m)) => Err(format!("{} has no `{}`", kind.a(), m.word())),
        _ => follow(sk, e, path).map(Operand::Ent),
    }
}

/// A face reference from a solid and the fields written past it (#183 F1): `pad.plate.near` is
/// solid `pad`, path `plate.near`; `surface` its surfaces (the member, read past it).
fn face_ref(solid: EntRef, fields: &[Seg], surface: bool) -> FaceRef {
    let path = fields.iter().filter_map(|s| match s {
        Seg::Field(n) => Some(n.text.clone()),
        _ => None,
    }).collect();
    FaceRef { solid: solid.i() as u32, path, surface, smooth: false }
}

/// Follow a reference's field path to the sub-entity it names — `root.center` is the circle's
/// centre point, and `a0.start` an arc's.
///
/// A `Scalar` field is not an entity and is not followed here: `c0.r` is a *number*, and the one
/// statement that names one is `fix`, which reads the path itself.
pub(super) fn follow(sk: &Sketch, mut e: EntRef, path: &[Seg]) -> Result<EntRef, String> {
    for seg in path {
        let f = match seg {
            Seg::Field(f) => f,
            Seg::Index(_) => return Err("an index names a copy, not a part".to_string()),
            Seg::Member(m) => return Err(format!("{} has no `{}`", e.kind.a(), m.word())),
        };
        // a plane's normal is no field of its declaration: an axis minted where it is read
        if e.kind == EntKind::Frame && f.text == NORMAL {
            match sk.frames.get(e.i()).and_then(|p| p.chart.as_ref()?.normal) {
                Some(n) => {
                    e = EntRef::new(EntKind::Axis, n as usize);
                    continue;
                }
                None => return Err("a plane's normal is not built yet".to_string()),
            }
        }
        // and a frame's joint axis, minted where its joint is read (#383)
        if e.kind == EntKind::Frame && f.text == "axis" {
            match sk.frames.get(e.i()).and_then(|p| p.joint.as_ref()) {
                Some(j) => {
                    e = EntRef::new(EntKind::Axis, j.axis as usize);
                    continue;
                }
                None => return Err("a frame's joint is not built yet".to_string()),
            }
        }
        // an envelope's side is bound after the solids, and a set's has none (#283): its two
        // parts are read by name, not by where they stand among its children
        if e.kind == EntKind::Envelope && e.i() < sk.envelopes.len() {
            let v = &sk.envelopes[e.i()];
            e = match f.text.as_str() {
                "surface" => match v.surface {
                    Some(s) => EntRef::new(EntKind::Surface, s as usize),
                    None => return Err(format!("`{}` has no revolution's side to name", v.name)),
                },
                "motion" => EntRef::new(EntKind::Motion, v.motion as usize),
                _ => return Err(format!("an envelope has surface, motion, not `{}`", f.text)),
            };
            continue;
        }
        // a meshing's motion is read by name, and its flanks are what it was written over (#283)
        if e.kind == EntKind::Meshing && e.i() < sk.meshings.len() {
            let m = &sk.meshings[e.i()];
            e = match f.text.as_str() {
                "motion" => EntRef::new(EntKind::Motion, m.motion as usize),
                _ => return Err(format!("a meshing has motion, not `{}`", f.text)),
            };
            continue;
        }
        // an arc a chain walks from its end is drawn from there (#363 S4), and its `start` is
        // still the point written as its start
        let swapped;
        let f = match f.text.as_str() {
            "start" | "end" if e.kind == EntKind::Arc
                && sk.walked_back.contains(&(e.i() as u32)) =>
            {
                swapped = crate::syntax::Name::new(if f.text == "start" { "end" } else { "start" });
                &swapped
            }
            _ => f,
        };
        let fields = e.kind.fields();
        let mut at = 0usize;
        let mut found = None;
        for (name, kind) in fields {
            match kind {
                Field::Scalar => {}
                Field::Child => {
                    if *name == f.text {
                        // a kind built after what is reading it (a face) has no parts yet
                        if e.i() >= sk.count(e.kind) {
                            return Err(format!("{} is not built yet", e.kind.a()));
                        }
                        found = sk.children(e).get(at).copied();
                    }
                    at += 1;
                }
                Field::List => {
                    if *name == f.text {
                        return Err(format!("`{}` is a list, so it needs an index", f.text));
                    }
                }
            }
        }
        match found {
            Some(k) => e = k,
            None => {
                let named: Vec<&str> =
                    fields.iter().filter(|(_, k)| *k == Field::Child).map(|(n, _)| *n).collect();
                return Err(if named.is_empty() {
                    format!("{} has no parts", e.kind.a())
                } else {
                    format!("{} has {}, not `{}`", e.kind.a(), named.join(", "), f.text)
                });
            }
        }
    }
    Ok(e)
}
