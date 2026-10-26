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
mod meets;
mod path;
pub(crate) use body::along_run;
pub use body::{reading, BodyFaces, Read, Reading};
pub use layers::layer;
pub use meets::Meets;
pub use path::{face_paths, face_source, FacePath, FaceSource};
pub use path::same_step;
pub(crate) use path::face_primitive;

/// Whether `body`'s faces as its readers read them (`BodyFaces::read`) are the faces of the whole
/// body: none of its terms reads them, so its stratum leaves nothing out (`without`).
pub fn stratum_is_whole(sk: &Sketch, body: usize) -> bool {
    readers_of(sk, body).iter().all(|&r| r != body && !crate::solid::document::reaches(sk, body as u32, r as u32))
}

/// The region solids whose terms read the faces of `body`.
pub fn readers_of(sk: &Sketch, body: usize) -> BTreeSet<usize> {
    (0..sk.solids.len()).filter(|&i| reads_faces_of(sk, i, body)).collect()
}

/// Whether solid `i` is a region whose terms read the faces of `body` (`readers_of`'s test).
fn reads_faces_of(sk: &Sketch, i: usize, body: usize) -> bool {
    matches!(&sk.solids[i].def,
        SolidDef::Region { terms, .. } if terms.iter().any(|t| reads_body(&t.shape, body)))
}

/// Whether a term's shape reads the faces of `body`: `body` among the solids of
/// `RegionShape::entities`, sought without gathering them, and no further than the first.  A
/// drawing rounded all over asks it of every region for every `cached` key, so the gathering was
/// most of the asking.
fn reads_body(shape: &RegionShape, body: usize) -> bool {
    let solid = |e: &crate::model::EntRef| e.kind == crate::model::EntKind::Solid && e.i() == body;
    match shape {
        RegionShape::Ball { .. } | RegionShape::Cylinder { .. } | RegionShape::Cone { .. } => false,
        RegionShape::Plane { along, .. } => solid(along),
        RegionShape::Measured { of } => of.iter().any(|(t, _)| target_reads_body(t, body)),
        RegionShape::FaceSide { face } => face.solid as usize == body,
        RegionShape::Angle { a, b } => a.solid as usize == body || b.solid as usize == body,
        RegionShape::Ends { edge } => target_reads_body(edge, body),
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
    for s in &mut out.solids {
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

/// `BodyFaces::read`, remembered against what it reads (`Caches::faces`): the stratum, so
/// a round's own numbers do not build its body again.
pub fn cached(sk: &Sketch, body: usize) -> Result<std::sync::Arc<BodyFaces>, String> {
    // each solid the walk reaches asked whether it reads `body`'s faces, rather than every solid
    // of the drawing gathered into a set first: a body rounded all over is asked this by each of
    // its rounds
    let key = super::MemoKey::except(sk, body, &|s| reads_faces_of(sk, s, body));
    sk.caches.faces.get_or(body, key, || BodyFaces::read(sk, body).map(std::sync::Arc::new))
}

/// The strata of the bodies whose faces region terms `terms` read (`cached`'s, each body once, in
/// order): what a reading of them is keyed on beside its own numbers (`MemoKey::own`).
pub fn strata(sk: &Sketch, terms: &[RegionTerm]) -> Result<Vec<std::sync::Arc<BodyFaces>>, String> {
    let mut bodies: Vec<usize> = terms.iter().flat_map(|t| t.shape.entities())
        .filter(|e| e.kind == crate::model::EntKind::Solid).map(|e| e.i()).collect();
    bodies.sort_unstable();
    bodies.dedup();
    bodies.iter().map(|&b| cached(sk, b)).collect()
}
