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

/// A solid of a binding as its representative's result placed (#566, `Caches::placed`): the
/// result it was placed from, by identity, the motion it was placed by, and what that made.
#[derive(Clone, Debug)]
pub struct Placement<T> {
    pub(crate) of: Rc<T>,
    pub(crate) motion: crate::solid::Rigid,
    pub(crate) placed: Rc<T>,
}

impl<T> Placement<T> {
    /// What was placed, where it was placed from `of` by `motion`.
    pub(crate) fn hit(&self, of: &Rc<T>, motion: &crate::solid::Rigid) -> Option<Rc<T>> {
        (Rc::ptr_eq(&self.of, of) && self.motion == *motion).then(|| self.placed.clone())
    }
}

/// Forget solid `i`'s zoom history in a cache by solid and approximation once it holds eight:
/// its report and mesh entries are kept.
pub(crate) fn cap_zoom<V>(cache: &mut BTreeMap<(usize, (u8, u64)), V>, i: usize) {
    let zooms = (i, (2, 0))..=(i, (2, u64::MAX));
    if cache.range(zooms.clone()).count() >= 8 {
        let gone: Vec<_> = cache.range(zooms).map(|(k, _)| *k).collect();
        for k in gone {
            cache.remove(&k);
        }
    }
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
    /// Each solid of a binding read as its representative's evaluation placed (#566), by solid and
    /// approximation: the representative's shape under its names, renamed once for every place
    /// it is moved to, and the last placement, which a repaint asks again.
    pub(crate) placed: RefCell<BTreeMap<(usize, (u8, u64)),
        (crate::solid::Renamed, Placement<crate::solid::EvaluatedSolid>)>>,
    /// The same for each such solid's exact B-rep, by solid.
    pub(crate) placed_exact: RefCell<BTreeMap<(usize, bool), Placement<crate::solid::Exact>>>,
    /// Each static solid's exact B-rep (`solid::Exact`), remembered against `solid::reads` alone:
    /// it does not depend on an approximation, so it is built once for a geometry and only meshed
    /// again for each one the evaluated solids above ask for.
    pub exact: Memo<Result<Rc<crate::solid::Exact>, String>>,
    /// Each swept body another of its binding places (`Sketch::bound_where`), built exactly by
    /// this kernel (`brep::export::exact`) to the tolerance it was asked at, against
    /// `solid::reads`: a closed instance's gear, shaped once and placed for every copy.
    pub(crate) swept_exact: Memo<(Option<u64>, Rc<crate::brep::export::Exact>)>,
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
    /// against, by identity): the regions a round makes for each order of the faces it names,
    /// where it does not state each once (#415, `lowering::GATED`), lower to one construction.
    /// A few, oldest asked forgotten
    /// first: a mesh fine enough to gate by is large.
    pub(crate) gated: RefCell<Vec<Rc<crate::solid::lowering::Gated>>>,
    /// Each region solid in general position built as the common of its terms
    /// (`solid::bounded::built`, #238), against `solid::reads`: the sweep test, the recipe and
    /// the exact boundary each ask it, and its gate reads the field.
    pub(crate) bounded: Memo<Option<Result<Rc<crate::solid::bounded::Bounded>, String>>>,
    /// Each region solid within reach of a body built as the body and what lies within reach of
    /// each face, edge and corner glued to it (`solid::grown::built`), against `solid::reads`: the
    /// sweep test, the recipe and the exact boundary each ask it, and its gate reads the field.
    pub(crate) grown: Memo<Option<Result<Rc<crate::solid::grown::Grown>, String>>>,
    /// Each ball's sweep built as its tube or refused by its rows (`solid::canal::built`, #238, #684
    /// D), against `solid::reads`: the sweep test, the recipe, the exact boundary and admission each
    /// ask it, and its gate reads the field.
    pub(crate) canal: Memo<Option<Result<Rc<crate::solid::canal::Canal>, crate::solid::canal::Why>>>,
    /// Each sweep's roll through a body (`solid::roll::interval`, #271), against `solid::reads`:
    /// admission, the field and the exact construction each ask it.
    pub(crate) roll: Memo<Result<[f64; 2], String>>,
    /// Each body's faces as regions read them (`solid::faces::BodyFaces`, #183), against
    /// `solid::reads`: every round over one body reads the same stratum.
    pub faces: Memo<Result<std::sync::Arc<crate::solid::faces::BodyFaces>, String>>,
    /// Each body's faces as the drawing finishes it, its rounds made (`BodyFaces::whole`, #510
    /// P7), against `solid::reads`: every scenario and report over one body reads the same build.
    pub whole: Memo<Result<std::sync::Arc<crate::solid::faces::BodyFaces>, String>>,
    /// Each body built exactly that another's field reads by its own boundary
    /// (`solid::field::BuiltField`, #419), against `solid::reads`: none where it is not read so.
    pub(crate) built: Memo<Option<crate::solid::BuiltField>>,
    /// Each face family stepped (`solid::family::built`, #243 L1), against `solid::reads` at the
    /// unit it was stepped at: validation and the evaluation each ask it, and it is a solve a
    /// station.
    pub(crate) family: Memo<Result<Rc<crate::solid::family::Family>, String>>,
    /// Each developable panel's band faceted (`solid::sheet::band`, #645), against `solid::reads`
    /// at the unit it was faceted at.
    pub(crate) sheet: Memo<Result<Rc<crate::solid::sheet::Band>, String>>,
    /// Each declared parameter's march (`follow::followed`, #391 S4), by its name, against every
    /// number of the drawing: a trace over it and the report each read it, and it is a solve a
    /// station.
    #[allow(clippy::type_complexity)]
    pub(crate) followed: RefCell<BTreeMap<String, (Vec<f64>, Rc<Result<crate::follow::Followed,
                                                                       String>>)>>,
    /// Each point in space traced over a parameter, in space (`Sketch::followed_world`, #398 F),
    /// by curve, against the march's key and its stretch: the scene in space draws it every time
    /// it is built, and it is a solve an instant.
    #[allow(clippy::type_complexity)]
    pub(crate) traced3: RefCell<BTreeMap<usize, (Rc<Result<crate::follow::Followed, String>>,
                                                  [f64; 2], Vec<[f64; 3]>)>>,
    /// Each profile generated under frames the drawing moves, its encoding with their tables
    /// attached (`Sketch::generated_flat`, #398 F), by curve, against the rides it read and its
    /// plane: a curve's every point reads it.
    #[allow(clippy::type_complexity)]
    pub(crate) generated: RefCell<BTreeMap<usize, (Vec<crate::motion::Ride>, [f64; 9],
                                                    Rc<Vec<f64>>)>>,
    /// Each frame the drawing moves read as a motion (`motion::Track`, #398 F), by its parameter
    /// and its frame, against the march it was read off (by identity): a sweep and its bounds
    /// read it at every query, and it is a solve at every node it adds between the stations.
    #[allow(clippy::type_complexity)]
    pub(crate) tracks: RefCell<BTreeMap<(String, u32), (Rc<Result<crate::follow::Followed,
        String>>, std::sync::Arc<crate::motion::Track>)>>,
    /// The largest side of the box round the objects shown (`overview::panes`, #679), against
    /// every number of the drawing, its shape epoch and how many surfaces a host supplied, and
    /// which objects are shown: a pick asks it of every pane and axis on every pointer move,
    /// and it is a walk of every object's reads.
    #[allow(clippy::type_complexity)]
    pub objects_side: RefCell<Option<(Vec<f64>, [usize; 2], Vec<usize>, Option<f64>)>>,
}

impl Caches {
    /// Forget everything remembered: the next reader of each works it out again.
    pub fn clear(&self) {
        self.polyline.borrow_mut().clear();
        self.solid.borrow_mut().clear();
        self.exact.clear();
        self.placed.borrow_mut().clear();
        self.placed_exact.borrow_mut().clear();
        self.swept.clear();
        self.swept_exact.clear();
        self.meridian.clear();
        self.general.clear();
        self.lowering.clear();
        self.gated.borrow_mut().clear();
        self.bounded.clear();
        self.grown.clear();
        self.canal.clear();
        self.roll.clear();
        self.faces.clear();
        self.whole.clear();
        self.built.clear();
        self.family.clear();
        self.sheet.clear();
        self.followed.borrow_mut().clear();
        self.tracks.borrow_mut().clear();
        self.traced3.borrow_mut().clear();
        self.generated.borrow_mut().clear();
        self.objects_side.borrow_mut().take();
    }
}
