//! **A region as a solid** (§6.9, §6.21): `solid(R)` is the points inside the set `R`, each bound
//! its body put on the probe one term (`SolidDef::Region`).  Its terms share one axis, so the solid
//! is a revolution of one region in the half-plane through that axis — the **meridian**, made by
//! the kernel's exact line-and-arc Booleans (`brep::planar`) — and every evaluator reads that:
//! the exact solid a `revolve` of its profile (`brep::recipe::region_profile`), the field its
//! revolved region (`RevolvedRegion::of_meridian`), the facets its polygon turned once.
//!
//! A region whose terms share no axis is read point by point instead (`general`), and either may be
//! built exactly another way (`Route`): this module says which, and remembers each reading.
use crate::model::{Sketch, SolidDef};
use std::rc::Rc;

mod general;
mod meridian;
pub use general::general;
pub(crate) use general::measure_of;
pub use meridian::{Meridian, Refusal};
pub(crate) use meridian::face_polys;

/// The meridian of region solid `solid`, from its terms as the drawing stands: refused where its
/// terms share no axis (a ball off it, a cone about another line, a plane not square to it),
/// where they leave no material, or where they leave material without end.  Remembered against
/// what it reads (`Caches::meridian`).
pub fn meridian(sk: &Sketch, solid: usize) -> Result<Rc<Meridian>, String> {
    folded(sk, solid).map_err(|r| r.why)
}

/// `meridian`, with its refusal said whole.
fn folded(sk: &Sketch, solid: usize) -> Result<Rc<Meridian>, Refusal> {
    // a region reading a body's faces is refused by its own terms, whatever the body: its key
    // need not walk the body, and through it every other round of it (#197)
    let faces = matches!(&sk.solids.get(solid).map(|s| &s.def), Some(SolidDef::Region { terms, .. })
        if super::faces::terms_read_faces(terms));
    let key = if faces {
        super::MemoKey::own(sk, solid, Vec::new())
    } else {
        super::MemoKey::of(sk, solid)
    };
    sk.caches.meridian.get_or(solid, key, || meridian::fold(sk, solid).map(Rc::new))
}

/// Region solid `solid` read in general position, where its meridian refused for a reason a
/// reading point by point answers (its terms share no axis): that reading, or why it is none.
/// `None` where the meridian reads it or refuses it outright.  Remembered against what it reads
/// (`Caches::general`): its own terms and the strata of the bodies whose faces it reads
/// (`faces::cached`'s), as `lowering::lowered` keys its own — a key over the bodies' every term
/// walks each other round of them, for each (#197).
pub(crate) fn in_general_position(sk: &Sketch, solid: usize)
    -> Option<Result<crate::solid::field::RegionField, String>> {
    let Some(SolidDef::Region { terms, .. }) = sk.solids.get(solid).map(|s| &s.def) else {
        return None;
    };
    let key = match super::faces::strata(sk, terms) {
        Ok(strata) => super::MemoKey::own(sk, solid, strata),
        Err(_) => super::MemoKey::of(sk, solid),
    };
    sk.caches.general.get_or(solid, key, || match folded(sk, solid) {
        Ok(_) => None,
        Err(r) if r.outright => None,
        Err(_) => Some(general(sk, solid)),
    })
}

/// Whether region solid `solid` is read in general position, and that reading made.
pub fn in_general(sk: &Sketch, solid: usize) -> bool {
    in_general_position(sk, solid).is_some_and(|g| g.is_ok())
}

/// A region solid's reading as `validate` asks it: its meridian, or else its reading in general
/// position, or the meridian's refusal.
pub fn readable(sk: &Sketch, solid: usize) -> Result<(), String> {
    form(sk, solid).map(|_| ())
}

/// How region solid `solid` is read, as the field and the facets ask it: its meridian, or else
/// point by point (`general`); refused where neither reads it, with the meridian's refusal or the
/// reading's.
pub enum Form {
    Meridian(Rc<Meridian>),
    General(crate::solid::field::RegionField),
}

/// Region solid `solid`'s `Form`.
pub fn form(sk: &Sketch, solid: usize) -> Result<Form, String> {
    match in_general_position(sk, solid) {
        Some(g) => g.map(Form::General),
        None => meridian(sk, solid).map(Form::Meridian),
    }
}

/// **Which construction builds a region solid** — the one answer every reader of it matches: the
/// recipe (`cad`), whether its boundary is the field's (`has_sweep`), the facets (`document`).
/// A region on one axis is its meridian turned; one read in general position is built by an
/// entry of the lowering table where one recognises it (a region over a body's faces, #194), else
/// by the common of its terms where each is a primitive (#238), else by the field alone.  An
/// entry that recognises a region and refuses it leaves it the field's, its refusal said.
#[derive(Clone)]
pub enum Route {
    /// Its terms share an axis: its meridian, turned once about it.
    Meridian(Rc<Meridian>),
    /// Over a body's faces: the lowering table's pieces (none: it holds nowhere), or why the
    /// entry that recognised it refuses it.
    Lowered(Result<Rc<Vec<super::lowering::Piece>>, String>),
    /// In general position: the common of its terms' primitives, or why that construction
    /// refuses it.
    Bounded(Result<Rc<super::bounded::Bounded>, String>),
    /// In general position, read by the field alone: why no exact construction is made of it.
    Field(String),
}

impl Route {
    /// Whether its boundary is the field's alone: no exact construction builds it.
    pub fn field_only(&self) -> bool {
        matches!(self, Route::Field(_) | Route::Lowered(Err(_)) | Route::Bounded(Err(_)))
    }

    /// The lowering table's pieces, where it is that entry's and built.
    pub fn pieces(&self) -> Option<&Rc<Vec<super::lowering::Piece>>> {
        match self {
            Route::Lowered(Ok(p)) => Some(p),
            _ => None,
        }
    }
}

/// Region solid `solid`'s `Route`; refused where it is not read at all (`readable`'s refusal), or
/// is no region.  Each construction is remembered where it is built (`Caches::meridian`,
/// `Caches::general`, `Caches::lowering`, `Caches::bounded`).
pub fn route(sk: &Sketch, solid: usize) -> Result<Route, String> {
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Region { terms, .. } = &s.def else {
        return Err(format!("`{}` is not a region", s.name));
    };
    match form(sk, solid)? {
        Form::Meridian(m) => Ok(Route::Meridian(m)),
        Form::General(_) => {
            if let Some(l) = super::lowering::lowered(sk, solid) {
                return Ok(Route::Lowered(l));
            }
            if let Some(b) = super::bounded::built(sk, solid) {
                return Ok(Route::Bounded(b));
            }
            let what = if super::faces::terms_read_faces(terms) {
                "a region over a body's faces that no entry of the lowering table builds"
            } else {
                "a region in general position over a sum of measurements, an offset or a wide cone"
            };
            Ok(Route::Field(format!("{what} is read by the field; no exact construction is made of \
                                     it yet")))
        }
    }
}
