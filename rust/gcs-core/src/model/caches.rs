//! What a sketch remembers and could work out again (`Caches`): every memo a reader keeps on the
//! drawing, in one place, with one way to forget it all.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

use crate::solid::Memo;

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
    /// it was computed from (`solid::reads`).  `polyline`'s bargain, for the same reason: a
    /// repaint asks every derived view for its edges, and a boundary is a sweep of
    /// classifications over a drawing that has not changed.
    #[allow(clippy::type_complexity)]
    pub solid: RefCell<
        BTreeMap<(usize, (u8, u64)), (Vec<f64>, Result<Rc<crate::solid::EvaluatedSolid>, String>)>,
    >,
    /// Each static solid's exact B-rep (`solid::Exact`), remembered against `solid::reads` alone:
    /// it does not depend on an approximation, so it is built once for a geometry and only meshed
    /// again for each one the evaluated solids above ask for.
    pub exact: Memo<Result<Rc<crate::solid::Exact>, String>>,
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
}

impl Caches {
    /// Forget everything remembered: the next reader of each works it out again.
    pub fn clear(&self) {
        self.polyline.borrow_mut().clear();
        self.solid.borrow_mut().clear();
        self.exact.clear();
        self.meridian.clear();
        self.general.clear();
        self.lowering.clear();
        self.bounded.clear();
        self.canal.clear();
        self.roll.clear();
        self.faces.clear();
    }
}
