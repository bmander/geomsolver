//! **The faces of a body, read as sets** (#183 F1, F7, F9): a region's body that names a face of
//! a solid (`p inside pad.plate.near`, `distance(p, block.near)`) reads it off the solid's exact
//! boundary as the faces stand in it.  A face is its patch (the trimmed face) and has a surface
//! (`F.surface`, the whole plane or cylinder it lies on); `inside F` is the surface's side the
//! material is on.
//!
//! **The stratum** (F6): a region that reads a body's faces and is a term of that body reads the
//! body as made by the terms that do not read it, so a round never reads its own faces or
//! another round's over the same body — but for one whose faces it names by the name its
//! statement was given, which it reads the body as that one leaves it (`layers`, [0.149]).
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target};
use std::collections::BTreeSet;

mod body;
mod layers;
mod lays;
mod meets;
mod path;
pub(crate) use body::along_run;
pub use body::{not_one_body, one_body, reading, BodyFaces, Bound, EndBy, Level, Read, Reading};
pub(crate) use body::TANGENT;
pub use layers::layer;
pub use lays::Lays;
pub use meets::{Meeting, Meets};
pub use path::{face_paths, face_source, one_body_of, FacePath, FaceSource};
pub(crate) use path::uses_naming;
pub use path::same_step;
pub(crate) use path::face_primitive;

/// Whether `body`'s faces as its readers read them (`BodyFaces::read`) are the faces of the whole
/// body: none of its terms reads them, so its stratum leaves nothing out (`without`).
pub fn stratum_is_whole(sk: &Sketch, body: usize) -> bool {
    readers_of(sk, body).iter().all(|&r| r != body && !crate::solid::document::reaches(sk, body as u32, r as u32))
}

/// The solids that read the faces of `body`: the regions whose terms read them, and the bodies
/// standing on one (a shell's cavity, the body within its faces less each wall cut from it).
pub fn readers_of(sk: &Sketch, body: usize) -> BTreeSet<usize> {
    (0..sk.solids.len()).filter(|&i| reads_faces_of(sk, i, body)).collect()
}

/// Whether solid `i` reads the faces of `body` (`readers_of`'s test): a region whose terms read
/// them, or a body other than `body` whose stock does, so every term it has acts on what they
/// read.  A body's layer stands on the layer beneath, whose faces are its own, so reads nothing.
fn reads_faces_of(sk: &Sketch, i: usize, body: usize) -> bool {
    let mut at = i;
    // a stock is made before what stands on it, so the walk ends; the bound is a guard alone
    for _ in 0..=sk.solids.len() {
        match &sk.solids[at].def {
            SolidDef::Region { terms, .. } => return terms.iter().any(|t| reads_body(&t.shape, body)),
            SolidDef::Body { stock, .. } if at != body && *stock as usize != body => at = *stock as usize,
            _ => return false,
        }
    }
    false
}

/// Whether a term's shape reads the faces of `body`: `body` among the solids of
/// `RegionShape::entities`, sought without gathering them, and no further than the first.  A
/// drawing rounded all over asks it of every region for every `cached` key, so the gathering was
/// most of the asking.
fn reads_body(shape: &RegionShape, body: usize) -> bool {
    let solid = |e: &crate::model::EntRef| e.kind == crate::model::EntKind::Solid && e.i() == body;
    match shape {
        RegionShape::Ball { .. } | RegionShape::Cylinder { .. } | RegionShape::Cone { .. }
        | RegionShape::Sheet { .. } | RegionShape::Section { .. } | RegionShape::Enclosed { .. }
        | RegionShape::Swept { .. } => false,
        RegionShape::Plane { along, .. } => solid(along),
        RegionShape::Measured { of } => of.iter().any(|(t, _)| target_reads_body(t, body)),
        RegionShape::FaceSide { face } => face.solid as usize == body,
        RegionShape::Angle { a, b } => a.solid as usize == body || b.solid as usize == body,
        RegionShape::Ends { edge } => target_reads_body(edge, body),
        RegionShape::Outside { set } => target_reads_body(set, body),
        RegionShape::Join { of } => of.iter().any(|t| target_reads_body(t, body)),
    }
}

/// `reads_body` of what a measurement measures (`Target::entities`' solids).
fn target_reads_body(t: &Target, body: usize) -> bool {
    match t {
        Target::Ent(e) => e.kind == crate::model::EntKind::Solid && e.i() == body,
        Target::Face(face) => face.solid as usize == body,
        Target::Set { terms, .. } => terms.iter().any(|t| reads_body(&t.shape, body)),
    }
}

/// **The stratum's order** (#183 F6): body `body`'s fills `on` and cuts `cut` as
/// `[on, cut, on_late, cut_late]`, the late ones those reading its faces (`readers_of`), which act
/// on the body the rest make, after them, fills then cuts — the order the field, the recipe and the
/// native host apply them in.
pub fn stratum(sk: &Sketch, body: usize, on: impl IntoIterator<Item = u32>, cut: impl IntoIterator<Item = u32>)
    -> [Vec<u32>; 4] {
    let readers = readers_of(sk, body);
    let late = |o: &u32| readers.contains(&(*o as usize));
    let (on_late, on): (Vec<u32>, Vec<u32>) = on.into_iter().partition(late);
    let (cut_late, cut): (Vec<u32>, Vec<u32>) = cut.into_iter().partition(late);
    [on, cut, on_late, cut_late]
}

/// The drawing with `readers` taken out of every body's terms: the stratum they read.
fn without(sk: &Sketch, readers: &BTreeSet<usize>) -> Result<Sketch, String> {
    let mut out = sk.clone();
    // what it builds of the bodies beneath is the drawing's own (#994): its memos are shared, each
    // entry read only where its solid reads as it does in the drawing — a body's lists of terms
    // are read by every key reaching it, so nothing reached through one changed here is shared
    out.caches = sk.caches.shared();
    for (i, s) in out.solids.iter_mut().enumerate() {
        // a reader is none of the stratum: what it stands on is its own
        if readers.contains(&i) {
            continue;
        }
        if let SolidDef::Body { stock, on, through, bound } = &mut s.def {
            if readers.contains(&(*stock as usize)) {
                return Err(format!("`{}` stands on a solid that reads its own faces", s.name));
            }
            for v in [on, through, bound] {
                v.retain(|o| !readers.contains(&(*o as usize)));
            }
        }
    }
    Ok(out)
}

/// Whether a term's shape is one of a face's (`Target::Face` inside a sum included).
pub fn reads_faces(shape: &RegionShape) -> bool {
    match shape {
        RegionShape::FaceSide { .. } | RegionShape::Angle { .. } | RegionShape::Ends { .. } => true,
        RegionShape::Measured { of } => of.iter().any(|(t, _)| !matches!(t, Target::Ent(_))),
        _ => false,
    }
}

/// The one face a term measures from, and the weight it gives it: a sum over that face (or its
/// surface) alone, what `q coincident F` and `q distance(d) F` make.  The one reader of that
/// pattern; its callers ask the rest of the term themselves, and differ: the lowering table takes a
/// weight of one and a plain number (`lowering::one_face`), a set measured from any weight above
/// nothing at nothing (`conditions::on_face`).
pub fn measured_face(t: &RegionTerm) -> Option<(&FaceRef, f64)> {
    let RegionShape::Measured { of } = &t.shape else { return None };
    let [(Target::Face(f), c)] = of.as_slice() else { return None };
    Some((f, *c))
}

/// Whether any of a region's terms reads a body's faces (`reads_faces`).
pub fn terms_read_faces(terms: &[RegionTerm]) -> bool {
    terms.iter().any(|t| reads_faces(&t.shape))
}

/// **A solid's listed faces, read off its stratum** (#625, #763): the stratum (`cached`: the body
/// as made by the terms that do not read its faces) where the drawing stands, and each face the
/// solid lists (`face_paths`) by its name from the solid (`FacePath::text`) with its patches there
/// — none for a face a Boolean ate, and `None` for a face a round of the body made, which the
/// stratum may not have.  What `Meets` and `Lays` are read from, once for both.
pub struct ListedFaces {
    read: std::sync::Arc<BodyFaces>,
    faces: Vec<(String, Option<Vec<usize>>)>,
}

/// **Why a body's faces are not read where the drawing stands** (#994): its boundary is the
/// field's, or it does not build there.  The two are told apart because only the second may come
/// out otherwise where the drawing solves: a seed that puts a short edge's ends past each other
/// (a straight of 0.05 between two corner rounds, seeded from its corners) builds nothing where
/// the drawing is seeded, and its body where it solves.
#[derive(Clone, Debug, PartialEq)]
pub enum Unread {
    /// A body with a swept cut, whose boundary is the field's: its faces are not read anywhere.
    Field,
    /// A body that does not build where the drawing stands, and why (the stratum's refusal).
    Unbuilt(String),
}

impl ListedFaces {
    /// The faces `listed` of solid `body`, or `None` where its boundary is not read here: a body
    /// with a swept cut, whose boundary is the field's, or one that does not build.
    pub fn read(sk: &Sketch, body: usize, listed: &[FacePath]) -> Option<ListedFaces> {
        ListedFaces::read_above(sk, body, listed, &|_| false).ok()
    }

    /// `read`, a face made by a use that stands `below` this body (a layer beneath it, whose
    /// rounds its stratum has) read as the stratum has it: one it has not is no face there.
    /// Where the boundary is not read, why (`Unread`).
    fn read_above(sk: &Sketch, body: usize, listed: &[FacePath], below: &dyn Fn(&str) -> bool)
        -> Result<ListedFaces, Unread> {
        if body >= sk.solids.len() {
            return Err(Unread::Unbuilt(format!("solid {body} is not drawn")));
        }
        if super::cad::contains_sweep(sk, body) {
            return Err(Unread::Field);
        }
        let read = cached(sk, body).map_err(Unread::Unbuilt)?;
        // each patch under its face's name, in one pass over the stratum
        let mut by_name: std::collections::BTreeMap<(&[String], &str), Vec<usize>> =
            Default::default();
        for (k, p) in read.paths().iter().enumerate() {
            by_name.entry((&p.route, &p.face)).or_default().push(k);
        }
        let faces = listed.iter().map(|f| {
            // with the caps lying on it (`BodyFaces::with_caps`, #994), as a reference reads it
            let own = read.with_caps(by_name.get(&(&f.route[..], &f.face[..])).cloned().unwrap_or_default());
            // a round's face the stratum has not: it cannot say of that one, unless the round
            // is of a layer beneath, which the stratum holds whole
            let unread = own.is_empty() && f.of.as_deref().is_some_and(|of| !below(of));
            (f.text(), (!unread).then_some(own))
        }).collect();
        Ok(ListedFaces { read, faces })
    }
}

/// **A body's layers** ([0.149], #879): a body whose rounds name one another stands in layers
/// (`layer`), each a body of its name, the stock of the one above — and the layer each use that
/// reads the body's faces stands in (`use_key`).  One layer where its rounds name nothing of each
/// other.
#[derive(Clone, Debug)]
pub struct Layers {
    /// The bodies, the lowest first and the body itself the last.
    bodies: Vec<usize>,
    uses: std::collections::BTreeMap<String, usize>,
}

impl Layers {
    /// Body `body`'s layers.
    pub fn of(sk: &Sketch, body: usize) -> Layers {
        let bodies = layer_bodies(sk, body);
        let mut uses = std::collections::BTreeMap::new();
        for (k, &b) in bodies.iter().enumerate() {
            for r in readers_of(sk, b) {
                uses.insert(path::use_key(&sk.solids[r].name).to_string(), k);
            }
        }
        Layers { bodies, uses }
    }

    /// Whether `other` stands its uses in the same layers: the faces are named again where not.
    pub fn same_uses(&self, other: &Layers) -> bool { self.uses == other.uses }

    /// The layer a condition reads the faces `faces` in, asked under the absolute prefix `at`:
    /// the layer of the use it stands in, where that use reads the body; and never beneath the
    /// layer above each use that made one of the faces, which no layer under it has.
    pub fn layer(&self, at: &str, faces: &[&FacePath]) -> usize {
        if self.bodies.len() == 1 {
            return 0;
        }
        let stands = self.uses.get(path::use_key(at.trim_end_matches('.'))).copied().unwrap_or(0);
        let made = faces.iter().filter_map(|f| self.uses.get(f.of.as_deref()?)).map(|&l| l + 1);
        made.fold(stands, usize::max).min(self.bodies.len() - 1)
    }

    /// The faces `listed` of the body read in layer `layer` (the top's, past it): a face made by
    /// a use standing in a layer beneath it read as that layer's stratum has it
    /// (`ListedFaces::read_above`).  What `Layered::read` reads each layer by, and what a round
    /// asks how two faces meet edge by edge in (`ListedFaces::meeting`, #962).
    pub fn listed(&self, sk: &Sketch, listed: &[FacePath], layer: usize) -> Option<ListedFaces> {
        self.read(sk, listed, layer).ok()
    }

    /// `listed`, and where the boundary is not read, why.
    fn read(&self, sk: &Sketch, listed: &[FacePath], layer: usize) -> Result<ListedFaces, Unread> {
        let k = layer.min(self.bodies.len() - 1);
        let below = |of: &str| self.uses.get(of).is_some_and(|&l| l < k);
        ListedFaces::read_above(sk, self.bodies[k], listed, &below)
    }
}

/// Body `body` and the bodies of its layers beneath it, the lowest first: each a body of its
/// name standing as the stock of the one above (`layer`).
fn layer_bodies(sk: &Sketch, body: usize) -> Vec<usize> {
    let mut bodies = vec![body];
    let mut at = body;
    while let Some(SolidDef::Body { stock, .. }) = sk.solids.get(at).map(|s| &s.def) {
        let stock = *stock as usize;
        if stock == at || sk.solids[stock].name != sk.solids[at].name
            || !matches!(sk.solids[stock].def, SolidDef::Body { .. }) {
            break;
        }
        bodies.push(stock);
        at = stock;
    }
    bodies.reverse();
    bodies
}

/// **A body's listed faces read layer by layer** ([0.149], #879): which faces meet (`Meets`) and
/// how each lies (`Lays`) is a question of the layer asked in (`Layers::layer`).  The faces `ab`
/// and `bc` of a box meet where a round of their edge reads them, and no longer where a round over
/// that round's face reads them: read off the top alone, the first round's `when b meets a` came
/// out false once the second stood over it, and the round was not made.
#[derive(Clone, Debug)]
pub struct Layered {
    /// Each layer's reading, the lowest first.
    layers: Vec<(Meets, Lays)>,
}

impl Layered {
    /// The faces `listed` of solid `body` read in each of its `layers`, or why its boundary is
    /// not read (`Unread`, the first layer's that is not).
    pub fn read(sk: &Sketch, layers: &Layers, listed: &[FacePath]) -> Result<Layered, Unread> {
        let layers = (0..layers.bodies.len()).map(|k| {
            layers.read(sk, listed, k).map(|l| (Meets::of(&l), Lays::of(&l)))
        }).collect::<Result<Vec<_>, _>>()?;
        Ok(Layered { layers })
    }

    /// Layer `layer`'s reading (the top's, past it).
    fn at(&self, layer: usize) -> &(Meets, Lays) { &self.layers[layer.min(self.layers.len() - 1)] }

    /// Which faces meet in layer `layer`.
    pub fn meets(&self, layer: usize) -> &Meets { &self.at(layer).0 }

    /// How each face lies in layer `layer`.
    pub fn lays(&self, layer: usize) -> &Lays { &self.at(layer).1 }
}

/// `BodyFaces::read`, remembered against what it reads (`Caches::faces`): the stratum, so
/// a round's own numbers do not build its body again.
pub fn cached(sk: &Sketch, body: usize) -> Result<std::sync::Arc<BodyFaces>, String> {
    // where the drawing is held still the stratum is read once for the reading (#994), its key
    // walked once rather than by every region of every edge and corner over the body
    sk.caches.held.stratum(body, || {
        // each solid the walk reaches asked whether it reads `body`'s faces, rather than every
        // solid of the drawing gathered into a set first: a body rounded all over is asked this
        // by each of its rounds
        let key = super::MemoKey::except(sk, body, &|s| reads_faces_of(sk, s, body));
        sk.caches.faces.get_or(body, key, || BodyFaces::read(sk, body).map(std::sync::Arc::new))
    })
}

/// **The stratum of `body` as its rounds' reading made it** (#996): the written terms of the body
/// `cached` built, where it has built them for the drawing as it stands — never built here.  What
/// the body's own build goes on from (`solid::exact`), where it is made of the same B-reps.
pub(crate) fn stratum_made(sk: &Sketch, body: usize)
    -> Option<crate::brep::recipe::Offered> {
    let key = super::MemoKey::except(sk, body, &|s| reads_faces_of(sk, s, body));
    sk.caches.faces.get(body, &key)?.ok()?.stratum().cloned()
}

/// The strata of the bodies whose faces region terms `terms` read (`cached`'s, each body once, in
/// order): what a reading of them is keyed on beside its own numbers (`MemoKey::own`).
pub fn strata(sk: &Sketch, terms: &[RegionTerm]) -> Result<Vec<std::sync::Arc<BodyFaces>>, String> {
    terms_bodies(terms).iter().map(|&b| cached(sk, b)).collect()
}

/// The solids region terms `terms` read, each once, in order.
fn terms_bodies(terms: &[RegionTerm]) -> Vec<usize> {
    let mut bodies: Vec<usize> = terms.iter().flat_map(|t| t.shape.entities())
        .filter(|e| e.kind == crate::model::EntKind::Solid).map(|e| e.i()).collect();
    bodies.sort_unstable();
    bodies.dedup();
    bodies
}

/// The solids region solid `i`'s terms read (none for a solid of another kind), each once, in
/// order: the bodies whose stratum it may be built against (`stratum_made`).
pub(crate) fn bodies_read(sk: &Sketch, i: usize) -> Vec<usize> {
    match sk.solids.get(i).map(|s| &s.def) {
        Some(SolidDef::Region { terms, .. }) => terms_bodies(terms),
        _ => Vec::new(),
    }
}
