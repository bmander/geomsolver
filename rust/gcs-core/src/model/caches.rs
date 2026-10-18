//! What a sketch remembers and could work out again (`Caches`): every memo a reader keeps on the
//! drawing, in one place, with one way to forget it all.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

use crate::solid::Memo;

/// One evaluated solid remembered (`Caches::solid`): what it was built against and what it came
/// to, and the last place it was asked for since it moved, so a solid asked again where it now
/// stands is the one answer (its world box worked out once).
#[derive(Clone, Debug)]
pub struct SolidSlot {
    pub key: crate::solid::ShapeKey,
    pub built: Result<Rc<crate::solid::EvaluatedSolid>, String>,
    pub moved: Option<(crate::plane::Basis, Rc<crate::solid::EvaluatedSolid>)>,
}

/// **The sketch's memos**: each result remembered against everything it was worked out from, so
/// a miss works out what a hit remembers and none of it is state — a drawing reads the same with
/// every one empty (`clear`). Interior, so `&Sketch` readers share them.
///
/// **What a clone keeps is deliberate**: a cloned sketch starts with every entry its original had,
/// each entry's value shared (an `Rc` or an `Arc`: the evaluated and exact solids, the meridians,
/// the lowerings, the bounded and canal builds, the bodies' faces) or copied by value (the
/// polylines, the rolls and the regions in general position), and the two then remember apart —
/// what the clone works out never reaches the original. A clone made to read the drawing changed
/// a little reads every operand it did not change from these: `Sketch::preview_body` joining
/// tools to a copy, and the stratum a body's rounds read (`solid::faces::without`), which takes
/// their own terms out of a copy and builds what is left from the original's exact solids. A
/// clone made to solve (`views::place`'s part, `System::recharted`'s, a claim's sweep) copies
/// entries it never reads; cheap where nothing has been evaluated yet, as during elaboration.
/// Every key is what its entry read — numbers by value, a stratum by identity (`solid::MemoKey`)
/// — so an entry a clone keeps is read only where its drawing still reads as it did.
///
/// Not here: the surfaces a host supplies for swept solids (`Sketch::field_surfaces`), which no
/// miss works out again, and where swept solids are meshed (`Sketch::field_meshing`), a setting.
#[derive(Clone, Debug, Default)]
pub struct Caches {
    /// Each curve's polyline, remembered against everything it was computed from
    /// (`curve_polyline`).  A pick walks every drawn curve on every pointer move, and a traced
    /// curve's polyline is a march of `CURVE_STEPS` block solves — nine milliseconds a move on
    /// the gear, for a drawing that had not changed.
    pub polyline: RefCell<BTreeMap<usize, (Vec<f64>, Vec<(f64, f64)>)>>,
    /// Each solid's evaluated boundary, by solid and approximation, remembered against everything
    /// it was computed from (`solid::ShapeKey`).  `polyline`'s bargain, for the same reason: a
    /// repaint asks every derived view for its edges, and a boundary is a sweep of
    /// classifications over a drawing that has not changed.  Remembered by its *shape*, so a
    /// solid that has only moved rigidly since is the same evaluation under a new placement
    /// (#412): a part drawn in its own frame, scrubbed along a timeline, is built once.
    pub solid: RefCell<BTreeMap<(usize, (u8, u64)), SolidSlot>>,
    /// Each static solid's exact B-rep (`solid::Exact`), remembered against `solid::reads` alone:
    /// it does not depend on an approximation, so it is built once for a geometry and only meshed
    /// again for each one the evaluated solids above ask for.
    pub exact: Memo<Result<Rc<crate::solid::Exact>, String>>,
    /// Whether each solid's boundary is the field's (`solid::has_sweep`), against `solid::reads`:
    /// asked before every evaluation, a remembered one too, and it reads every operand's route.
    pub(crate) swept: Memo<bool>,
    /// Each region solid's meridian (`solid::region::meridian`), against `solid::reads`: every
    /// evaluator, the validation and admission read it, and it is a fold of exact Booleans.
    pub meridian:
        Memo<Result<Rc<crate::solid::region::Meridian>, crate::solid::region::Refusal>>,
    /// Each region solid's reading in general position (`solid::region::in_general_position`),
    /// against its own terms and the strata it reads: validation, the sweep test and the field
    /// each ask it.
    pub(crate) general: Memo<Option<Result<crate::solid::RegionField, String>>>,
    /// Each region solid's lowering (`solid::lowering::lowered`, #194), against its own terms and
    /// the stratum it reads: the recipe and the exact boundary each ask it, and its gate reads the
    /// field.
    pub(crate) lowering: Memo<Result<Rc<Vec<crate::solid::lowering::Piece>>, String>>,
    /// The latest constructions a lowering's gate built and meshed (`solid::lowering::gated`),
    /// each against the recipe node it was built from (and the stratum a rolled one rolls
    /// against, by identity): the regions a round makes for each order of the faces it names
    /// lower to one construction.  A few, oldest asked forgotten first: a mesh fine enough to
    /// gate by is large.
    #[allow(clippy::type_complexity)]
    pub(crate) gated: RefCell<Vec<(String, Option<std::sync::Arc<crate::solid::faces::BodyFaces>>,
                                   Rc<crate::solid::lowering::Gated>)>>,
    /// Each region solid in general position built as the common of its terms
    /// (`solid::bounded::built`, #238), against `solid::reads`: the sweep test, the recipe and
    /// the exact boundary each ask it, and its gate reads the field.
    pub(crate) bounded: Memo<Option<Result<Rc<crate::solid::bounded::Bounded>, String>>>,
    /// Each ball's sweep built as its tube (`solid::canal::built`, #238), against `solid::reads`:
    /// the sweep test, the recipe and the exact boundary each ask it, and its gate reads the field.
    pub(crate) canal: Memo<Option<Result<Rc<crate::solid::canal::Canal>, String>>>,
    /// Each sweep's roll through a body (`solid::roll::interval`, #271), against `solid::reads`:
    /// admission, the field and the exact construction each ask it.
    pub(crate) roll: Memo<Result<[f64; 2], String>>,
    /// Each body's faces as regions read them (`solid::faces::BodyFaces`, #183), against
    /// `solid::reads`: every round over one body reads the same stratum.
    pub faces: Memo<Result<std::sync::Arc<crate::solid::faces::BodyFaces>, String>>,
    /// Each face family stepped (`solid::family::built`, #243 L1), against `solid::reads` at the
    /// unit it was stepped at: validation and the evaluation each ask it, and it is a solve a
    /// station.
    pub(crate) family: Memo<Result<Rc<crate::solid::family::Family>, String>>,
    /// Each declared parameter's march (`follow::followed`, #391 S4), by its name, against every
    /// number of the drawing: a trace over it and the report each read it, and it is a solve a
    /// station.
    #[allow(clippy::type_complexity)]
    pub(crate) followed: RefCell<BTreeMap<String, (Vec<f64>, Rc<Result<crate::follow::Followed,
                                                                       String>>)>>,
}

impl Caches {
    /// Forget everything remembered: the next reader of each works it out again.
    pub fn clear(&self) {
        self.polyline.borrow_mut().clear();
        self.solid.borrow_mut().clear();
        self.exact.clear();
        self.swept.clear();
        self.meridian.clear();
        self.general.clear();
        self.lowering.clear();
        self.gated.borrow_mut().clear();
        self.bounded.clear();
        self.canal.clear();
        self.roll.clear();
        self.faces.clear();
        self.family.clear();
        self.followed.borrow_mut().clear();
    }
}
