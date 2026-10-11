//! **A face of a solid is a set** (§6.9, §6.21, #162 F5): `b0.near`, `b0.far`, a side `b0.ab`,
//! and through a body `body.b0.near`, each the surface the face lies on — its *carrier*, the
//! face carried past its edges.  Used where any set is: `m coincident b0.near` puts a point on
//! it, and `v1 tangent(at: v1.origin) b0.near` is a plane lying on it, which is sketching on a
//! face.
//!
//! The set is made here, from the solid's own text, before the uses are expanded, so every use
//! is an ordinary one.  Its body is written over what the solid is written over, so it follows
//! the solid's extents and its profile wherever they move:
//!
//! * a cap at the extent `D`: `{ p | p distance(D, along: n) f0#plane }` (`p coincident f0#plane`
//!   at `D = 0`), `near` at the higher ordinate and `far` at the lower;
//! * a side over the straight edge `e`: `{ p | p level(M) e.p1 }`, `M` a hidden axis square to
//!   `e` and parallel to the profile's plane — the plane through `e` along the normal.
//!
//! `f0#plane` is a name the source cannot write: the plane the face's edges are drawn in, which
//! only the elaborator knows (memberships), bound there before any relation reads it
//! (`Expansion::face_planes`).  The hidden axis is seeded there too (`Expansion::face_axes`).
//!
//! What has no carrier yet is refused at the use, in its words: a curved side, a `through:`
//! solid's caps (known only after the solve), and a face of anything but a prism.

use super::*;
use super::apply::local;
use crate::model::EntKind;
use crate::program::public_path;
use crate::constraints::Fixity::Infix;
use crate::syntax::{Arg, Chained, DeclName, OpArg, Relation, RelationForm, SetLit, Sweep};

/// The plane a face of a solid is drawn in, by the hidden name its carriers read it under — see
/// `Expansion::face_planes`.
#[derive(Clone, Debug)]
pub struct FacePlane {
    /// `{face}#plane`, absolute.
    pub key: String,
    /// What the face is made of — its edges or points — by absolute name, every one of which
    /// must be drawn in the one plane.
    pub edges: Vec<String>,
    /// The use that asked for it, where a refusal is said.
    pub span: Span,
    /// The face's name, for that refusal.
    pub face: String,
}

/// The hidden axis a side's carrier is level along — see `Expansion::face_axes`.
#[derive(Clone, Debug)]
pub struct FaceAxis {
    /// `{face}#{edge}#m`, absolute.
    pub key: String,
    /// The edge, absolute.
    pub edge: String,
    /// The face's plane, `FacePlane::key`.
    pub plane: String,
}

/// What a statement in the walk's output declares, read once for every face a use names.
struct Table {
    /// Every solid, by absolute name: its declaration and the scope it was written in.
    solids: BTreeMap<String, (Decl, Scope, crate::syntax::StmtId, Span)>,
    /// Every face, by absolute name: what it is made of as written, and its scope.
    faces: BTreeMap<String, (Vec<Ref>, Scope)>,
    /// Every line, by absolute name.
    lines: BTreeSet<String>,
    /// Each body's operands, by absolute name: its stock and what is united, cut or bound.
    operands: BTreeMap<String, Vec<String>>,
}

impl Walk<'_> {
    /// Every face of a solid a set's word names, made a set before the uses are expanded — each
    /// once, under `{solid}.{path}` — and a statement naming one that cannot be dropped, said
    /// once at the operand.
    pub(super) fn face_sets(&mut self) {
        let alias = self.alias_table();
        let mut wanted: Vec<(Ref, String, Vec<String>)> = Vec::new();
        for (st, _, sc) in &self.out {
            let StmtKind::Relation(r) = &st.kind else { continue };
            let Some(w) = r.form.written().filter(|w| sets::may_use(w)) else { continue };
            for op in &w.ops {
                if let Some((abs, rest)) = lookup_raw(op, sc, &self.names, &alias, self.units) {
                    if !rest.is_empty() && !self.sets.contains_key(&abs) {
                        wanted.push((op.clone(), abs, rest));
                    }
                }
            }
        }
        if wanted.is_empty() {
            return;
        }
        let table = self.table(&alias);
        let mut refused = BTreeSet::new();
        for (op, abs, rest) in wanted {
            if !table.solids.contains_key(&abs) && !table.operands.contains_key(&abs) {
                continue; // not a solid: whatever it is, it is someone else's to read
            }
            let key = format!("{abs}.{}", rest.join("."));
            if self.sets.contains_key(&key) || refused.contains(&key) {
                continue;
            }
            match self.carrier(&table, &abs, &rest, op.span) {
                Ok(site) => {
                    self.sets.insert(key, site);
                }
                Err(m) => {
                    self.once(Code::E040, op.span, m);
                    refused.insert(key);
                }
            }
        }
        // a statement over a face refused says nothing more: it was said at the operand
        if !refused.is_empty() {
            let units = self.units;
            let names = &self.names;
            self.out.retain(|(st, _, sc)| {
                let StmtKind::Relation(r) = &st.kind else { return true };
                let Some(w) = r.form.written() else { return true };
                !w.ops.iter().any(|op| {
                    lookup_raw(op, sc, names, &alias, units).is_some_and(|(abs, rest)| {
                        refused.contains(&format!("{abs}.{}", rest.join(".")))
                    })
                })
            });
        }
    }

    fn table(&self, alias: &BTreeMap<String, String>) -> Table {
        let mut t = Table {
            solids: BTreeMap::new(),
            faces: BTreeMap::new(),
            lines: BTreeSet::new(),
            operands: BTreeMap::new(),
        };
        let abs = |r: &Ref, sc: &Scope| {
            lookup_raw(r, sc, &self.names, alias, self.units)
                .filter(|(_, rest)| rest.is_empty())
                .map(|(a, _)| a)
        };
        for (st, _, sc) in &self.out {
            match &st.kind {
                StmtKind::Decl(d) => {
                    let key = d.name.key().text.clone();
                    match d.kind {
                        EntKind::Solid => {
                            if matches!(d.sweep, Some(Sweep::Body)) {
                                let stock = d.children.iter().flatten().filter_map(|k| match k {
                                    Kid::Ref(r) => abs(r, sc),
                                    _ => None,
                                });
                                t.operands.entry(key.clone()).or_default().extend(stock);
                            } else {
                                t.solids.insert(key, (d.clone(), sc.clone(), st.id, st.span));
                            }
                        }
                        EntKind::Face => {
                            let items = d.children.first().into_iter().flatten().filter_map(|k| {
                                match k {
                                    Kid::Ref(r) => Some(r.clone()),
                                    _ => None,
                                }
                            });
                            t.faces.insert(key, (items.collect(), sc.clone()));
                        }
                        EntKind::Line => {
                            t.lines.insert(key);
                        }
                        _ => {}
                    }
                }
                StmtKind::Chain(c) if c.closed => {
                    t.faces.insert(c.name.key().text.clone(), (c.links.clone(), sc.clone()));
                }
                StmtKind::SolidRel(r) => {
                    if let (Some(what), Some(body)) = (abs(&r.what, sc), abs(&r.body, sc)) {
                        t.operands.entry(body).or_default().push(what);
                    }
                }
                _ => {}
            }
        }
        t
    }

    /// The set `{solid}.{path}` is: the solid the path reaches through bodies' operands, and the
    /// carrier of its face — or why there is none.
    fn carrier(&mut self, t: &Table, abs: &str, rest: &[String], at: Span)
        -> Result<sets::Site, String> {
        let said = format!("{}.{}", public_path(abs), rest.join("."));
        // through bodies, operand by operand, by the name each kept (§6.9 [0.18])
        let mut solid = abs.to_string();
        for seg in &rest[..rest.len() - 1] {
            let leaf = |o: &String| o.rsplit('.').next() == Some(seg.as_str());
            let Some(next) = t.operands.get(&solid).and_then(|ops| ops.iter().find(|o| leaf(o)))
            else {
                return Err(format!("`{said}` names no face: `{seg}` is not a solid `{}` is made \
                                    of", public_path(&solid)));
            };
            solid = next.clone();
        }
        let face = rest.last().unwrap();
        let Some((decl, sc, stmt, span)) = t.solids.get(&solid) else {
            return Err(format!("`{said}` names no face: `{}` is a body, and its faces are its \
                                operands' — `{}.<operand>.{face}`", public_path(&solid),
                               public_path(&solid)));
        };
        let (from, to) = match &decl.sweep {
            Some(Sweep::Depth { depth }) => (format!("-({})", arg_text(depth)), "0".to_string()),
            Some(Sweep::Prism { from, to }) => (arg_text(from), arg_text(to)),
            Some(Sweep::Through { .. }) => {
                return Err(format!("`{said}`: a `through:` solid's ends are known only once the \
                                    drawing is solved, so they are no set to draw on"));
            }
            _ => {
                return Err(format!("`{said}`: only a prism's faces are sets yet — a revolution's, \
                                    a loft's and a sweep's are not"));
            }
        };
        // the face swept: one named, or one written in place, `solid(face(ab, bc, …), …)`
        let alias = self.alias_table();
        let (face_abs, items, face_sc, plane) = match decl.children.first().map(Vec::as_slice) {
            Some([Kid::Ref(r)]) => {
                let face_abs = lookup_raw(r, sc, &self.names, &alias, self.units)
                    .filter(|(_, rest)| rest.is_empty())
                    .map(|(a, _)| a)
                    .ok_or_else(|| format!("`{said}`: `{}` names no face", written(r)))?;
                let Some((items, face_sc)) = t.faces.get(&face_abs) else {
                    return Err(format!("`{said}`: `{}` is not a face", written(r)));
                };
                (face_abs, items.clone(), face_sc.clone(), suffixed(r, "#plane", at))
            }
            Some([Kid::Face { decl: f, .. }]) => {
                let items = f.children.first().into_iter().flatten().filter_map(Kid::as_ref);
                let face_abs = format!("{solid}#face");
                let plane = local(format!("{face_abs}#plane"), at);
                (face_abs, items.cloned().collect(), sc.clone(), plane)
            }
            _ => return Err(format!("`{said}`: its profile is no face to draw on")),
        };
        let (items, face_sc) = (&items, &face_sc);
        // the plane the face is drawn in, under a name only the elaborator binds
        let plane_key = format!("{face_abs}#plane");
        if self.names.insert(plane_key.clone()) {
            let alias = self.alias_table();
            let edges = items.iter().filter_map(|r| {
                lookup_raw(r, face_sc, &self.names, &alias, self.units).map(|(a, rest)| {
                    std::iter::once(a).chain(rest).collect::<Vec<_>>().join(".")
                })
            });
            let edges = edges.collect();
            let face = public_path(&face_abs);
            self.face_planes.push(FacePlane { key: plane_key.clone(), edges, span: at, face });
        }
        let p = local("p", at);
        let body = match face.as_str() {
            "near" | "far" => {
                // which end is nearer the viewer is the ordinates', where both are numbers here
                let (lo, hi) = match (number(&from), number(&to)) {
                    (Some(a), Some(b)) if a > b => (to, from),
                    _ => (from, to),
                };
                let d = if face == "near" { hi } else { lo };
                if number(&d) == Some(0.0) {
                    vec![relation("coincident", vec![p, plane], Vec::new(), at)]
                } else {
                    let along = OpArg::Named(Name { text: "along".into(), span: at },
                                             Arg::Word("n".into()));
                    vec![relation("distance", vec![p, plane], vec![OpArg::Dim(d, at), along], at)]
                }
            }
            edge => {
                let leaf = |r: &Ref| written(r).rsplit('.').next() == Some(edge);
                let Some(e) = items.iter().find(|r| leaf(r)) else {
                    return Err(format!("`{said}` names no face: `{}` has `near`, `far` and a side \
                                        for each edge it names", public_path(&solid)));
                };
                let alias = self.alias_table();
                let e_abs = lookup_raw(e, face_sc, &self.names, &alias, self.units)
                    .filter(|(_, r)| r.is_empty())
                    .map(|(a, _)| a);
                if !e_abs.as_ref().is_some_and(|a| t.lines.contains(a)) {
                    return Err(format!("`{said}` is a curved side, and a curved face is not yet a \
                                        set: sketch on a flat one"));
                }
                // square to the edge and along the face's plane: the side's own normal
                let m_key = format!("{face_abs}#{edge}#m");
                if self.names.insert(m_key.clone()) {
                    let mut axis = Decl::point(DeclName::Key(Name { text: m_key.clone(), span: at }),
                                               [0.0; 2], None);
                    axis.kind = EntKind::Axis;
                    axis.seed = vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
                    axis.seed_text = vec![None; 6];
                    let m = local(&m_key, at);
                    let (path, scope) = (Vec::new(), sc.clone());
                    let stmt = |kind| Stmt { id: *stmt, kind, span: *span, chained: Chained::No };
                    self.out.push((stmt(StmtKind::Decl(axis)), path.clone(), scope.clone()));
                    for (word, other) in [("perpendicular", e.clone()), ("parallel", plane.clone())] {
                        let r = relation(word, vec![m.clone(), other], Vec::new(), at);
                        self.out.push((stmt(StmtKind::Relation(r)), path.clone(), scope.clone()));
                    }
                    self.face_axes.push(FaceAxis {
                        key: m_key.clone(),
                        edge: e_abs.unwrap(),
                        plane: plane_key.clone(),
                    });
                }
                let start = suffixed(e, ".p1", at);
                let level = vec![OpArg::Ent(local(&m_key, at))];
                vec![relation("level", vec![p, start], level, at)]
            }
        };
        let body = body.into_iter().map(|r| Stmt {
            id: *stmt,
            kind: StmtKind::Relation(r),
            span: at,
            chained: Chained::No,
        });
        let lit = SetLit { bound: Name { text: "p".into(), span: at }, body: body.collect(), span: at };
        Ok(sets::Site::new(lit, sc.clone(), 0))
    }
}

/// `r` with `tail` on its last name: `f0` → `f0#plane`, `e` → `e.p1`.
fn suffixed(r: &Ref, tail: &str, at: Span) -> Ref {
    let mut text = written(r);
    text.push_str(tail);
    let mut segs = text.split('.');
    let root = Name { text: segs.next().unwrap().to_string(), span: at };
    let path = segs.map(|s| Seg::Field(Name { text: s.to_string(), span: at })).collect();
    Ref { root, path, span: at }
}

fn relation(word: &str, ops: Vec<Ref>, args: Vec<OpArg>, at: Span) -> Relation {
    let word = Name { text: word.to_string(), span: at };
    let w = crate::syntax::Written { word, fixity: Infix, ops, args, span: at };
    Relation::of(RelationForm::Written(w))
}

/// An extent as text, as the flattener settled it.
fn arg_text(a: &Arg) -> String {
    match a {
        Arg::Num(v) => crate::syntax::num(*v),
        Arg::Int(v) => v.to_string(),
        Arg::Dim { text, .. } | Arg::SeedExpr { text, .. } => text.clone(),
        // no extent is written otherwise; a number nothing reads is refused where it is built
        _ => "0".to_string(),
    }
}

/// An extent's number, where its text is one.
fn number(text: &str) -> Option<f64> {
    text.trim().parse::<f64>().ok()
}
