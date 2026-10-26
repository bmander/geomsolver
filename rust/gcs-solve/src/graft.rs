//! The rebuild walk: a sketch copied through a filter, every surviving entity renumbered and
//! every reference to it following.  Deletion (`without`), copying (`copy`), pasting (`paste`)
//! and the part of a document one gesture can move (`Part`) are all this one walk, so they cannot
//! disagree about what a constraint belongs to.
//!
//! The walk's answer is a `Renumbering`: where each entity and each constraint of the source
//! went.  What a value of the model refers to, and what it cannot be without, is the value's own
//! to say (`FaceE::remapped`, `SolidDef::remapped`, `Constraint::remapped`, …), each naming every
//! field it holds; the walk decides only the order the kinds are made in and what the
//! destination's own constructors mint.

use crate::constraints::{Arg, CKind, Constraint};
use crate::decompose;
use crate::expr;
use crate::model::{EntKind, EntRef, Sketch};
use std::collections::{BTreeMap, BTreeSet};

const KINDS: usize = EntKind::ALL.len();

/// **Where a rebuild put everything** (`rebuild`, `graft_onto`): each entity of the source to the
/// one it became, kind by kind, each constraint the document states to its copy, and what the
/// walk made, in the order it made it.  It is the walk's whole answer, so a caller pairing the
/// two sketches (`Part`) reads it rather than working it out again.
#[derive(Clone, Debug)]
pub struct Renumbering {
    /// Per kind (`EntKind as usize`): source index → destination index, from the moment the walk
    /// reaches the kind.  `None` before then, so a value read against a kind not yet made is
    /// caught rather than silently left behind.
    ents: [Option<Vec<Option<u32>>>; KINDS],
    /// Source constraint id → destination id.
    constraints: BTreeMap<u32, u32>,
    /// (source, destination) for every entity the walk made, in the order it made them: a plane
    /// it stood the source's on (`graft_onto`), and that plane's origin, are mapped but not made.
    made: Vec<(EntRef, EntRef)>,
}

impl Renumbering {
    fn new() -> Renumbering {
        Renumbering { ents: std::array::from_fn(|_| None), constraints: BTreeMap::new(),
                      made: Vec::new() }
    }

    /// The walk has reached `kind`, of which the source has `n`: nothing of it has come yet.
    fn open(&mut self, kind: EntKind, n: usize) {
        self.ents[kind as usize] = Some(vec![None; n]);
    }

    /// `src` is `dst` in the destination, which the walk did not make (a plane stood on).
    fn set(&mut self, src: EntRef, dst: usize) {
        let table = self.ents[src.kind as usize].as_mut().expect("the walk has reached the kind");
        table[src.i()] = Some(dst as u32);
    }

    /// `src` came along as `dst`, which the walk made.
    fn made(&mut self, src: EntRef, dst: usize) {
        self.set(src, dst);
        self.made.push((src, EntRef::new(src.kind, dst)));
    }

    /// A source entity as the destination's, if it came along.
    pub fn ent(&self, e: EntRef) -> Option<EntRef> {
        let Some(table) = &self.ents[e.kind as usize] else {
            debug_assert!(false, "{} read before the walk made any", e.kind.a());
            return None;
        };
        table.get(e.i()).copied().flatten().map(|i| EntRef::new(e.kind, i as usize))
    }

    /// A source point's index in the destination, if it came along.
    pub fn point(&self, i: usize) -> Option<usize> {
        self.ent(EntRef::point(i)).map(EntRef::i)
    }

    /// A source constraint's id in the destination, if it came along.
    pub fn constraint(&self, id: u32) -> Option<u32> {
        self.constraints.get(&id).copied()
    }

    /// (source, destination) for every entity the walk made, in the order it made them.
    pub fn pairs(&self) -> &[(EntRef, EntRef)] {
        &self.made
    }

    /// Every entity the walk made, in the order it made them: points, lines, circles, arcs,
    /// splines, axes, planes, curves, faces, motions, solids, surfaces, envelopes, traced bodies
    /// and meshings, each kind in source order.
    pub fn made_ents(&self) -> Vec<EntRef> {
        self.made.iter().map(|&(_, d)| d).collect()
    }

    /// Number the members of `kind` that `whole` holds of, as the greatest such set: a kind whose
    /// members name each other, forward too — a motion relative to two others, a cutter naming
    /// the body that subtracts it — has its table settled before any member is made, each kept
    /// one numbered from `base` in source order, and a member naming one that did not come goes
    /// too, until nothing more goes.
    fn allocate(&mut self, kind: EntKind, n: usize, base: usize, candidate: impl Fn(usize) -> bool,
                whole: impl Fn(usize, &Renumbering) -> bool) {
        let mut kept: Vec<bool> = (0..n).map(candidate).collect();
        loop {
            let mut next = base as u32;
            self.ents[kind as usize] = Some(kept.iter().map(|&k| {
                k.then(|| { next += 1; next - 1 })
            }).collect());
            let gone: Vec<usize> = (0..n).filter(|&i| kept[i] && !whole(i, self)).collect();
            if gone.is_empty() {
                break;
            }
            for i in gone {
                kept[i] = false;
            }
        }
    }
}

/// **Where a walk lands.**  A sketch of its own, which the rebuild makes whole — the source's
/// unit, its presentation, its root choices — or a document it is added to, moved by an offset,
/// onto the planes that document already has.  Said by the caller, never guessed from what the
/// destination holds: a paste into an empty document is still a paste, in that document's unit.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Landing {
    Fresh,
    Onto { offset: (f64, f64) },
}

/// The entities of `src` that `keep` accepts, and every constraint over them but those `drop`
/// names, as a sketch of their own — and where each went.
///
/// `keep` is the caller's word on each entity; what an entity cannot be without is the walk's
/// (a line goes with its points, a face with its edges and plane, a solid with its term), so a
/// caller says only what it removes or keeps itself, and the rest follows.
pub fn rebuild(src: &Sketch, keep: &dyn Fn(EntRef) -> bool, drop: &[u32])
    -> (Sketch, Renumbering) {
    let mut dst = Sketch::new();
    let r = Walk::new(&mut dst, src, keep, drop, Landing::Fresh).run();
    (dst, r)
}

/// Everything in `src` added to `dst`, moved by `offset`, and where each went.  A plane `dst`
/// already has, held where `src`'s is held and facing the same way, *is* that plane: what was
/// drawn in it is drawn in it again.  Nothing is converted — `paste` is what converts units.
pub fn graft_onto(dst: &mut Sketch, src: &Sketch, offset: (f64, f64)) -> Renumbering {
    Walk::new(dst, src, &|_| true, &[], Landing::Onto { offset }).run()
}

/// The walk: one pass per kind, in the order each kind's references need — a kind is made after
/// every kind it may name — and then everything that names entities: roles, duals, the
/// constraints, rates, held numbers, twins, root choices, turns, mirrors and pictures.
struct Walk<'a> {
    dst: &'a mut Sketch,
    src: &'a Sketch,
    keep: &'a dyn Fn(EntRef) -> bool,
    drop: &'a [u32],
    landing: Landing,
    r: Renumbering,
    /// Each source plane that is one of the destination's (`graft_onto`).
    reuse: Vec<Option<usize>>,
}

impl<'a> Walk<'a> {
    fn new(dst: &'a mut Sketch, src: &'a Sketch, keep: &'a dyn Fn(EntRef) -> bool,
           drop: &'a [u32], landing: Landing) -> Walk<'a> {
        let reuse = vec![None; src.frames.len()];
        Walk { dst, src, keep, drop, landing, r: Renumbering::new(), reuse }
    }

    fn run(mut self) -> Renumbering {
        self.document();
        self.reuse_planes();
        self.points();
        self.lines();
        self.circles();
        self.arcs();
        self.splines();
        self.axes_and_planes();
        self.memberships();
        // curves last of what a curve may name: a curve's arguments may be of any kind above
        self.curves();
        self.faces();
        self.motions();
        self.joints();
        self.solids();
        self.surfaces();
        self.envelopes();
        self.traced();
        self.meshings();
        // every entity is renumbered: what names one reads the whole table from here
        self.roles();
        let duals = self.duals();
        let expr = self.constraints(&duals);
        self.axes_along();
        self.rates();
        self.held_reads();
        self.twins();
        if expr {
            expr::evaluate(self.dst);
        }
        self.choices();
        self.turns_and_mirrors();
        self.pictures();
        self.dst.settle_turns();
        // each free curve's energy, its pegs and its definition, read again from what came (#144)
        self.dst.settle_variational();
        self.r
    }

    /// **Every field of a sketch, and what the walk does with it**, named with no `..`: a field
    /// added to `Sketch` stops the build here until it is decided — carried by a step of the
    /// walk, minted again by the destination, or left behind and said so.  It reads nothing; the
    /// steps it names do the work.
    #[allow(dead_code)]
    fn ledger(sk: &Sketch) {
        let Sketch {
            // carried, each by the step of the walk named
            roles: _,                     // `roles`
            points: _,                    // `points`, and every kind after it by its own step
            lines: _,
            circles: _,
            arcs: _,
            splines: _,
            axes: _,                      // `axes_and_planes`
            frames: _,
            curves: _,                    // `curves`, with the definitions they name
            curve_defs: _,
            faces: _,
            solids: _,
            surfaces: _,
            motions: _,
            envelopes: _,
            meshings: _,
            traced: _,
            reversed: _,                  // `arcs`
            duals: _,                     // `duals`
            twins: _,                     // `twins`
            constraints: _,               // `constraints`, `axes_along`; intrinsic rows minted
            placements: _,                // `constraints`
            rates: _,                     // `rates`
            held: _,                      // `document`, `held_reads`
            materials: _,                 // `document`
            units: _,                     // `document`, a rebuild's alone
            presentation: _,
            branches: _,                  // `choices`
            orientations: _,
            turns: _,                     // `turns_and_mirrors`
            mirrors: _,
            derived: _,                   // `pictures`
            // minted again by the destination as what reads them comes
            params: _,                    // by each constructor, valued from the source's
            zero: _,                      // `Sketch::origin_param`, for the first axis
            lifts: _,                     // `memberships` (`lift_point`)
            shared: _,                    // `add_quiet`, from the names `owned_arg` gives
            free_vars: _,                 // `expr::evaluate`
            free_dimensions: _,
            variational: _,               // `settle_variational`
            next_cid: _,                  // `add_quiet`: a copy's constraints are its own
            // left behind on purpose
            carried: _,                   // a carried row is freed by its family's part, which
                                          // this walk makes (`solid::family::part_of`)
            declared: _,                  // the source's alone (parameters.md)
            parameters: _,
            beams: _,                     // the source's alone (analysis.md, S1)
            scenarios: _,
            measured: _,
            meshed: _,                    // compiled with the rows the source alone carries
            meshed_at: _,
            rounds: _,                    // the elaborator's, derived and never saved
            standing: _,                  // presentation of an instant, never written
            hidden: _,                    // view state the front end sets on every drawing
            caches: _,                    // compiled artefacts, worked out again
            field_surfaces: _,            // a host's, supplied against the drawing it read
            field_meshing: _,
            style_epoch: _,               // a binding's cache keys, counted afresh
            shape_epoch: _,
            // not carried, though each names entities: a copy goes without them
            rides: _,
            solid_claims: _,
            solid_bearings: _,
            plane_names: _,
            spline_traced: _,
            stands_for: _,
            round_gauges: _,
            walked_back: _,
        } = sk;
    }

    /// What the document is as a whole: a rebuild keeps the document it came from — which is
    /// what makes a clipboard say what its numbers are in, and `paste` able to convert them —
    /// and both bring the materials and the running numbers the destination lacks.
    fn document(&mut self) {
        let (src, dst) = (self.src, &mut *self.dst);
        if self.landing == Landing::Fresh {
            dst.units = src.units;
            dst.presentation = src.presentation.clone();
        }
        // the materials the solids name (#510 P3), by name: a paste brings those the document
        // lacks
        for (name, m) in &src.materials {
            dst.materials.entry(name.clone()).or_insert_with(|| m.clone());
        }
        // a face family's running numbers, before any contact pinned to one is added (#243 L1)
        for (name, h) in &src.held {
            if !dst.held.contains_key(name) {
                dst.hold(name, src.params[h.param as usize].value, h.dim);
            }
        }
    }

    /// A paste onto a plane the document already has: one held where the clipboard's is held and
    /// facing the same way *is* that plane, so what was drawn in it is drawn in it again and no
    /// second plane stands on the first; its axes and its origin are the document's.
    fn reuse_planes(&mut self) {
        let (src, dst) = (self.src, &*self.dst);
        if self.landing == Landing::Fresh {
            return;
        }
        let same = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9);
        for (i, r) in self.reuse.iter_mut().enumerate() {
            if !(self.keep)(EntRef::frame(i)) || !src.plane_fixed(i)
                || src.frames[i].chart.is_none()
            {
                continue;
            }
            let b = src.basis(i);
            let planar = src.frames[i].planar();
            *r = dst.charted().into_iter().find(|&j| {
                let c = dst.basis(j);
                dst.frames[j].planar() == planar && dst.plane_fixed(j)
                    && same(b.u, c.u) && same(b.v, c.v) && same(b.o, c.o)
            });
        }
    }

    fn points(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        let offset = match self.landing {
            Landing::Fresh => (0.0, 0.0),
            Landing::Onto { offset } => offset,
        };
        // a reused plane's origin is the document's plane's
        let reused_origin: BTreeMap<usize, usize> = self.reuse.iter().enumerate()
            .filter_map(|(i, r)| r.map(|j| (src.frames[i].origin as usize,
                                            dst.frames[j].origin as usize)))
            .collect();
        r.open(EntKind::Point, src.points.len());
        let base = dst.points.len();
        let mut keep_pts = Vec::new();
        for i in 0..src.points.len() {
            if let Some(&o) = reused_origin.get(&i) {
                r.set(EntRef::point(i), o);
            } else if (self.keep)(EntRef::point(i)) {
                r.set(EntRef::point(i), base + keep_pts.len());
                keep_pts.push(i);
            }
        }
        for i in keep_pts {
            let (x, y) = src.point_xy(i);
            let n = dst.point(x + offset.0, y + offset.1, src.point_fixed(i),
                              &format!("p{}", dst.points.len()));
            if let Some(z) = src.points[i].z {
                dst.give_place(n, src.params[z as usize].value);
            }
            debug_assert_eq!(r.point(i), Some(n));
            r.made(EntRef::point(i), n);
        }
    }

    fn lines(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Line, src.lines.len());
        for (i, l) in src.lines.iter().enumerate() {
            if !(self.keep)(EntRef::line(i)) {
                continue;
            }
            let (Some(p1), Some(p2)) = (r.point(l.p1 as usize), r.point(l.p2 as usize)) else {
                continue;
            };
            let ni = dst.line(p1, p2);
            dst.lines[ni].class = l.class.clone();
            r.made(EntRef::line(i), ni);
        }
    }

    fn circles(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Circle, src.circles.len());
        for (i, c) in src.circles.iter().enumerate() {
            if !(self.keep)(EntRef::circle(i)) {
                continue;
            }
            let Some(centre) = r.point(c.center as usize) else { continue };
            let ni = dst.circle(centre, src.params[c.radius as usize].value, "");
            let rp = dst.circles[ni].radius as usize;
            dst.params[rp].fixed = src.params[c.radius as usize].fixed;
            dst.circles[ni].class = c.class.clone();
            r.made(EntRef::circle(i), ni);
        }
    }

    fn arcs(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Arc, src.arcs.len());
        for (i, a) in src.arcs.iter().enumerate() {
            if !(self.keep)(EntRef::arc(i)) {
                continue;
            }
            let point = |p: u32| r.point(p as usize);
            let (Some(c), Some(st), Some(e)) = (point(a.center), point(a.start), point(a.end))
            else {
                continue;
            };
            let ni = dst.arc(c, st, e, "");
            let rp = dst.arcs[ni].radius as usize;
            dst.params[rp].value = src.params[a.radius as usize].value;
            dst.params[rp].fixed = src.params[a.radius as usize].fixed;
            dst.arcs[ni].class = a.class.clone();
            // drawn from its end for its sense, it still reads its ends as written (#573)
            if src.arc_reversed(i) {
                dst.reversed.insert(ni as u32);
            }
            r.made(EntRef::arc(i), ni);
        }
    }

    fn splines(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Spline, src.splines.len());
        for (i, sp) in src.splines.iter().enumerate() {
            if !(self.keep)(EntRef::spline(i)) {
                continue;
            }
            // the control points that came along, and the ones that did not: a curve that has
            // lost some is shortened, one interior knot going with each, and only dies when too
            // few are left to draw with
            let (mut ctrl, mut gone) = (Vec::new(), Vec::new());
            for (k, &c) in sp.ctrl.iter().enumerate() {
                match r.point(c as usize) {
                    Some(n) => ctrl.push(n),
                    None => gone.push(k),
                }
            }
            // with nothing gone this gives back the knots unchanged, so there is no case to
            // split; a weight goes with its control point
            let knots = crate::curve::knots_without(&sp.knots, &gone, ctrl.len());
            let weights = sp.weights.as_ref().map(|w| {
                w.iter().enumerate().filter(|(k, _)| !gone.contains(k)).map(|(_, &x)| x).collect()
            });
            let Some(ni) = dst.spline_weighted(&ctrl, Some(knots), weights) else { continue };
            dst.splines[ni].class = sp.class.clone();
            r.made(EntRef::spline(i), ni);
        }
    }

    /// The axes, each its own: a direction, its fixed flags and its place — but a reused plane's,
    /// which are the document's plane's — and the planes over them.  A place is given once every
    /// plane stands, as the document's own construction gives it.
    fn axes_and_planes(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Axis, src.axes.len());
        for (i, reused) in self.reuse.iter().enumerate() {
            if let Some(j) = reused {
                r.set(EntRef::axis(src.frames[i].u as usize), dst.frames[*j].u as usize);
                r.set(EntRef::axis(src.frames[i].v as usize), dst.frames[*j].v as usize);
            }
        }
        let mut places = Vec::new();
        for (i, a) in src.axes.iter().enumerate() {
            if !(self.keep)(EntRef::axis(i)) || r.ent(EntRef::axis(i)).is_some() {
                continue;
            }
            let value = |q: u32| src.params[q as usize].value;
            let ni = dst.axis_restored(a.d.map(value), a.d.map(|q| src.params[q as usize].fixed),
                                       a.class.clone());
            // a place held while nothing reads it is no `fix`'s (`Sketch::place_axis`)
            let held = a.a.map(|q| a.placed && src.params[q as usize].fixed);
            places.push((ni, a.a.map(value), held));
            r.made(EntRef::axis(i), ni);
        }
        r.open(EntKind::Frame, src.frames.len());
        for (i, f) in src.frames.iter().enumerate() {
            if let Some(j) = self.reuse[i] {
                r.set(EntRef::frame(i), j);
                continue;
            }
            if !(self.keep)(EntRef::frame(i)) {
                continue;
            }
            // a frame declared bare comes as it is, owning nothing (#383)
            let Some(p) = f.chart.as_ref() else {
                dst.frames.push(crate::model::FrameE { class: f.class.clone(),
                                                       ..crate::model::FrameE::bare() });
                r.made(EntRef::frame(i), dst.frames.len() - 1);
                continue;
            };
            let axis = |a: u32| r.ent(EntRef::axis(a as usize)).map(EntRef::i);
            let (Some(u), Some(v), Some(origin)) =
                (axis(p.u), axis(p.v), r.point(p.origin as usize))
            else {
                continue;
            };
            let ni = dst.plane_restored(u, v, p.o.map(|k| src.params[k as usize].value),
                                        p.o.map(|k| src.params[k as usize].fixed), origin,
                                        f.class.clone());
            if !p.planar {
                dst.spatial_chart(ni);
            }
            let normal = p.normal.and_then(axis);
            r.made(EntRef::frame(i), ni);
            // its normal, where it came along too
            if let Some(n) = normal {
                dst.stand_normal(ni, n);
            }
        }
        for (ni, a, held) in places {
            dst.place_restored(ni, a, held);
        }
    }

    /// A membership follows its plane across, and a plane that did not come — deleted, or
    /// missing a point — takes the memberships that named it with it.  A hidden point comes with
    /// the point it lifts, seeded where that point lifts to now.
    fn memberships(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &self.r);
        for i in 0..src.points.len() {
            let Some(ni) = r.point(i) else { continue };
            if let Some(p) = src.plane_of(i) {
                dst.set_plane(ni, r.ent(EntRef::frame(p)).map(EntRef::i));
            }
            // and a point drawn in a spatial frame's chart, its three numbers kept (#383)
            if let Some(f) = src.points[i].chart {
                if let Some(g) = r.ent(EntRef::frame(f as usize)).map(EntRef::i).filter(|&g| {
                    !dst.frames[g].planar() && dst.frames[g].chart.is_some()
                        && dst.frames[g].origin as usize != ni
                }) {
                    dst.draw_in_chart(ni, g);
                }
            }
        }
        for l in &src.lifts {
            if let Some(ni) = r.point(l.point as usize) {
                dst.lift_point(ni);
            }
        }
    }

    /// The *definition* travels with a curve — a document that came apart from the curve family
    /// it is written in would be a document that cannot be drawn — and a free curve's length is
    /// its own number, which comes as a circle's radius does.
    fn curves(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Curve, src.curves.len());
        for (i, cv) in src.curves.iter().enumerate() {
            // a trim is minted after the curve it trims, so that curve's place is already known
            let Some(moved) = cv.remapped(&|e| r.ent(e)) else { continue };
            if !(self.keep)(EntRef::new(EntKind::Curve, i)) {
                continue;
            }
            let def = &src.curve_defs[cv.def as usize];
            // a followed point's lowering is the whole drawing's (#398 A): it travels unlowered,
            // as drawn, until the document it lands in is elaborated again
            let lowered = matches!(def.body, crate::model::CurveBody::Followed(Some(_)));
            let def = match lowered {
                true => crate::model::CurveDef {
                    name: format!("followed:{}", def.param),
                    columns: Vec::new(),
                    values: Vec::new(),
                    vars: Vec::new(),
                    body: crate::model::CurveBody::Followed(None),
                    ..def.clone()
                },
                false => def.clone(),
            };
            let at = match dst.curve_defs.iter().position(|d| d.name == def.name) {
                Some(k) => k,
                None => {
                    dst.curve_defs.push(def);
                    dst.curve_defs.len() - 1
                }
            };
            // a lowered followed point carries none of what it was lowered to
            let (unknowns, values, pose) = match lowered {
                true => Default::default(),
                false => (moved.unknowns, moved.values, moved.pose),
            };
            let length = cv.length.map(|l| {
                let p = &src.params[l as usize];
                dst.param(p.value, p.fixed, &p.name) as u32
            });
            // its pegs are the energy's to settle again
            dst.curves.push(crate::model::CurveE {
                def: at as u32, unknowns, values, pose, length, pegs: Vec::new(),
                slides: Vec::new(), ..moved
            });
            r.made(EntRef::new(EntKind::Curve, i), dst.curves.len() - 1);
        }
    }

    fn faces(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Face, src.faces.len());
        for (i, f) in src.faces.iter().enumerate() {
            if !(self.keep)(EntRef::face(i)) {
                continue;
            }
            let Some(f) = f.remapped(&|e| r.ent(e)) else { continue };
            dst.faces.push(f);
            r.made(EntRef::face(i), dst.faces.len() - 1);
        }
    }

    /// A motion goes with what it is written over, as a constraint goes with its entities — and a
    /// motion written over motions with them, so the table is settled before any is made.
    fn motions(&mut self) {
        let (src, dst, keep) = (self.src, &mut *self.dst, self.keep);
        let motion = |i: usize| EntRef::new(EntKind::Motion, i);
        self.r.allocate(EntKind::Motion, src.motions.len(), dst.motions.len(), |i| keep(motion(i)),
                        |i, r| src.motions[i].def.remapped(&|e| r.ent(e)).is_some());
        let r = &mut self.r;
        for (i, m) in src.motions.iter().enumerate() {
            let Some(next) = r.ent(motion(i)) else { continue };
            let m = m.remapped(&|e| r.ent(e)).expect("allocation kept what it names");
            dst.motions.push(m);
            debug_assert_eq!(next.i(), dst.motions.len() - 1);
            r.made(motion(i), next.i());
        }
    }

    /// Each frame's joint (#383): its axis, its two numbers, and the node it was lowered to.
    fn joints(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &self.r);
        for (i, f) in src.frames.iter().enumerate() {
            let (Some(j), Some(ni)) = (f.joint.as_ref(), r.ent(EntRef::frame(i))) else { continue };
            let ni = ni.i();
            if self.reuse[i].is_some() || dst.frames[ni].joint.is_some() {
                continue;
            }
            let Some(axis) = r.ent(EntRef::axis(j.axis as usize)) else { continue };
            let number = |dst: &mut Sketch, q: u32| {
                let p = &src.params[q as usize];
                dst.param(p.value, p.fixed, &p.name) as u32
            };
            let (ratio, advance) = (number(dst, j.ratio), number(dst, j.advance));
            let motion = j.motion.and_then(|m| r.ent(EntRef::new(EntKind::Motion, m as usize)))
                .map(|m| m.idx);
            dst.frames[ni].joint =
                Some(crate::model::Joint { axis: axis.idx, ratio, advance, motion });
            // a joint's numbers the solve finds are the frame's, renumbered with its joint
            if let (Some(m), Some(n)) = (motion, j.motion.map(|m| &src.motions[m as usize])) {
                dst.motions[m as usize].numbers = n.numbers.map(|_| [ratio, advance]);
            }
        }
    }

    /// Extent targets and Boolean operands may be forward references, including a cutter naming
    /// the body that subtracts it, so the table is settled before any solid is made.
    fn solids(&mut self) {
        let (src, dst, keep) = (self.src, &mut *self.dst, self.keep);
        self.r.allocate(EntKind::Solid, src.solids.len(), dst.solids.len(),
                        |i| keep(EntRef::solid(i)),
                        |i, r| src.solids[i].def.remapped(&|e| r.ent(e)).is_some());
        let r = &mut self.r;
        for (i, so) in src.solids.iter().enumerate() {
            let Some(next) = r.ent(EntRef::solid(i)) else { continue };
            // allocation kept exactly the solids whose every reference exists: dropping an
            // operand here would change the solid and the numbering
            let so = so.remapped(&|e| r.ent(e)).expect("allocation kept what it names");
            dst.solids.push(so);
            debug_assert_eq!(next.i(), dst.solids.len() - 1);
            r.made(EntRef::solid(i), next.i());
        }
    }

    fn surfaces(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Surface, src.surfaces.len());
        for (i, s) in src.surfaces.iter().enumerate() {
            let e = EntRef::new(EntKind::Surface, i);
            if !(self.keep)(e) {
                continue;
            }
            let Some(s) = s.remapped(&|e| r.ent(e)) else { continue };
            dst.surfaces.push(s);
            r.made(e, dst.surfaces.len() - 1);
        }
    }

    fn envelopes(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Envelope, src.envelopes.len());
        for (i, v) in src.envelopes.iter().enumerate() {
            let e = EntRef::new(EntKind::Envelope, i);
            if !(self.keep)(e) {
                continue;
            }
            let Some(v) = v.remapped(&|e| r.ent(e)) else { continue };
            dst.envelopes.push(v);
            r.made(e, dst.envelopes.len() - 1);
        }
    }

    /// A traced body may be traced over one before it, so the table fills as it goes.
    fn traced(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Traced, src.traced.len());
        for (i, t) in src.traced.iter().enumerate() {
            let e = EntRef::new(EntKind::Traced, i);
            if !(self.keep)(e) {
                continue;
            }
            let Some(t) = t.remapped(&|e| r.ent(e)) else { continue };
            dst.traced.push(t);
            r.made(e, dst.traced.len() - 1);
        }
    }

    fn meshings(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        r.open(EntKind::Meshing, src.meshings.len());
        for (i, m) in src.meshings.iter().enumerate() {
            let e = EntRef::new(EntKind::Meshing, i);
            if !(self.keep)(e) {
                continue;
            }
            let Some(m) = m.remapped(&|e| r.ent(e)) else { continue };
            dst.meshings.push(m);
            r.made(e, dst.meshings.len() - 1);
        }
    }

    fn roles(&mut self) {
        for (&e, &roles) in &self.src.roles {
            if let Some(target) = self.r.ent(e) {
                self.dst.roles.insert(target, roles);
            }
        }
    }

    /// Each tangency's derivative, while its point and what moves it came along, with the
    /// geometry the use made that did (§6.21): source dual → destination dual.
    fn duals(&mut self) -> Vec<Option<usize>> {
        let (src, dst, r) = (self.src, &mut *self.dst, &self.r);
        src.duals.iter().map(|d| {
            let point = r.point(d.point)?;
            let toward = d.toward.remapped(&|e| r.ent(e))?;
            let owned = d.owned.iter().filter_map(|&e| r.ent(e)).collect();
            Some(dst.add_dual(point, toward, owned))
        }).collect()
    }

    /// The constraints the document states, each where every entity it names came along, and
    /// whether any is written as an expression (evaluated once, at the end: `add_quiet`).
    fn constraints(&mut self, duals: &[Option<usize>]) -> bool {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        let mut expr = false;
        for c in src.user_constraints() {
            // a row the source alone carries reads what the walk does not copy (#510 S1)
            if self.drop.contains(&c.id) || c.kind.of_source() {
                continue;
            }
            // the destination allocates its own parameters: a Param index is this sketch's name
            // for one, and the pin rides along in the seed — or the name, where the unknown is
            // shared, so the contacts that owned one own one again
            let Some(nc) = c.remapped(&|e| r.ent(e), &|p| src.owned_arg(p), &|d| duals[d]) else {
                continue;
            };
            expr |= crate::expr::has_expr(&nc.args);
            let id = dst.add_quiet(nc);
            r.constraints.insert(c.id, id);
            if let Some(&place) = src.placements.get(&c.id) {
                dst.placements.insert(id, place);   // a dimension keeps where it was dragged to
            }
        }
        expr
    }

    /// A plane held along drawn lines (`program::entities::axes_along`) came over as a plane of
    /// its own: its axes' rows to the lines, and the point its origin stands at in place of its
    /// own `PlaneAxis` rows, are the elaborator's and not a plane's build, so they are carried
    /// across as they stand, or the copy would turn free of the lines it was drawn along.
    fn axes_along(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &mut self.r);
        let along: BTreeSet<usize> = src.constraints.iter()
            .filter(|c| c.intrinsic && c.kind == CKind::Parallel3)
            .filter(|c| c.args[0].ent().kind == EntKind::Axis)
            .filter(|c| c.args[1].ent().kind == EntKind::Line)
            .map(|c| c.args[0].ent().i())
            .collect();
        let carried = |c: &&Constraint| match c.kind {
            CKind::Parallel3 | CKind::PointOnAxis => c.entities().iter()
                .any(|e| e.kind == EntKind::Axis && along.contains(&e.i())),
            CKind::Coincident3 => src.plane_of_origin(c.args[0].ent().i()).is_some_and(|p| {
                let pl = &src.frames[p];
                along.contains(&(pl.u as usize)) && along.contains(&(pl.v as usize))
            }),
            _ => false,
        };
        let rows: Vec<&Constraint> =
            src.constraints.iter().filter(|c| c.intrinsic).filter(carried).collect();
        for c in rows {
            // every slot as it states them, its entities the copies: which way an axis runs
            // along its line among them (#535)
            let Some(args) = c.args.iter()
                .map(|a| match a {
                    Arg::Ent(e) => r.ent(*e).map(Arg::Ent),
                    a => Some(a.clone()),
                })
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            // a plane standing at a point has no `PlaneAxis` rows of its own for its in-plane
            // axes: they go, as `axes_along` takes them (its normal's stays)
            if c.kind == CKind::Coincident3 {
                if let Some(p) = dst.plane_of_origin(args[0].ent().i()) {
                    let (u, v) = (dst.frames[p].u as usize, dst.frames[p].v as usize);
                    let gone: Vec<u32> = dst.constraints.iter()
                        .filter(|d| d.intrinsic && d.kind == CKind::PlaneAxis)
                        .filter(|d| d.args[0].ent() == EntRef::frame(p))
                        .filter(|d| [u, v].contains(&d.args[1].ent().i()))
                        .map(|d| d.id)
                        .collect();
                    for id in gone {
                        dst.remove(id);
                    }
                }
            }
            let mut nc = Constraint::new(c.kind, args);
            nc.intrinsic = true;
            let id = dst.add_quiet(nc);
            r.constraints.insert(c.id, id);
        }
    }

    fn rates(&mut self) {
        for row in &self.src.rates {
            if let Some(row) = row.remapped(&|e| self.r.ent(e)) {
                self.dst.rates.push(row);
            }
        }
    }

    /// A held number that reads the geometry reads the copy of it, where that came along.
    fn held_reads(&mut self) {
        for (name, h) in &self.src.held {
            let reads = h.reads.and_then(|(e, f)| Some((self.r.ent(e)?, f)));
            if let Some(d) = self.dst.held.get_mut(name) {
                d.reads = reads;
            }
        }
    }

    /// Each twin that came along with its point, tied again after the relations (§6.7).
    fn twins(&mut self) {
        for (&p, ts) in &self.src.twins {
            let Some(p) = self.r.point(p) else { continue };
            for &t in ts {
                if let Some(t) = self.r.point(t) {
                    self.dst.tie_twin(p, t);
                }
            }
        }
        self.dst.hold_twins();
    }

    /// Recorded root choices are keyed by sketch point index; grafting renumbers points, so the
    /// keys travel with them.  One naming a point that did not come is dropped — replaying it
    /// would apply a chirality to whatever triangle inherited those indices.  The choices the
    /// document states travel alike (#157).
    fn choices(&mut self) {
        let (src, dst, r) = (self.src, &mut *self.dst, &self.r);
        let choices =
            [(&src.branches, &mut dst.branches), (&src.orientations, &mut dst.orientations)];
        for (from, to) in choices {
            for (k, &v) in from {
                match decompose::branch_key_points(k) {
                    None => {
                        if self.landing == Landing::Fresh {
                            to.insert(k.clone(), v);   // not a triangle: nothing to renumber
                        }
                    }
                    Some(pts) => {
                        let mapped: Option<Vec<usize>> = pts.iter().map(|&p| r.point(p)).collect();
                        if let Some(m) = mapped {
                            // renumbering can reorder the three, and the sign is read against
                            // the key's own order — so it is re-recorded rather than carried
                            let (k, v) = decompose::branch_record([m[0], m[1], m[2]], v >= 0);
                            to.insert(k, v);
                        }
                    }
                }
            }
        }
    }

    /// A ring's turns and a mirrored instance's images, each where everything it names came.
    fn turns_and_mirrors(&mut self) {
        let map = |e| self.r.ent(e);
        for t in &self.src.turns {
            if let Some(t) = t.remapped(&map) {
                self.dst.turn(t);
            }
        }
        for m in &self.src.mirrors {
            if let Some(m) = m.remapped(&map) {
                self.dst.mirror(m);
            }
        }
    }

    /// The pictures the document asks for (§6.11), each with its body and its planes.
    fn pictures(&mut self) {
        for d in &self.src.derived {
            if let Some(d) = d.remapped(&|e| self.r.ent(e)) {
                self.dst.derived.push(d);
            }
        }
    }
}

/// Copy of the sketch with the given entities/constraints removed, plus everything that depends on
/// a removed entity.  Deletion by rebuild — simple, and keeps `Sketch`'s invariants trivially true.
pub fn without(sk: &Sketch, entities: &[EntRef], constraints: &[u32]) -> Sketch {
    let dead: Vec<EntRef> = entities.to_vec();
    // An entity survives while enough of its children do.  For a line, a circle or an arc that
    // is all of them, which is the old rule; a spline is defined by a list, so losing one
    // control point shortens the curve rather than deleting it.
    let alive = |e: EntRef| {
        if dead.contains(&e) {
            return false;
        }
        // a plane's origin is the plane's, and goes with it
        if e.kind == EntKind::Point {
            if let Some(pl) = sk.plane_of_origin(e.i()) {
                if dead.contains(&EntRef::frame(pl)) {
                    return false;
                }
            }
        }
        let kids = sk.children(e);
        kids.iter().filter(|c| !dead.contains(c)).count() >= sk.min_children(e, &kids)
    };
    rebuild(sk, &alive, constraints).0
}

/// The selection as a sketch of its own: every entity picked, the points that define it, and
/// every constraint all of whose entities came along.  Copying is keeping, which is deleting
/// everything else — so it goes through the same rule, and a constraint that survives a copy is
/// exactly one that would have survived deleting the rest.
pub fn copy(sk: &Sketch, entities: &[EntRef]) -> Sketch {
    let mut keep = BTreeSet::new();
    let mut pending = entities.to_vec();
    while let Some(e) = pending.pop() {
        if !keep.insert(e) { continue; }
        pending.extend(sk.children(e));
        if e.kind == EntKind::Point {
            if let Some(p) = sk.plane_of(e.i()) { pending.push(EntRef::frame(p)); }
        }
    }
    let mut candidates = sk.primitives();
    for kind in [EntKind::Curve,EntKind::Face,EntKind::Solid,EntKind::Surface,EntKind::Motion,EntKind::Envelope,EntKind::Meshing] {
        candidates.extend((0..sk.count(kind)).map(|i| EntRef::new(kind,i)));
    }
    let drop: Vec<EntRef> = candidates.into_iter().filter(|e| !keep.contains(e)).collect();
    without(sk, &drop, &[])
}

/// Add everything in `clip` to `sk`, moved by (dx, dy), and return what that made.  The pasted
/// geometry brings its own constraints and nothing else: it is not joined to what is already
/// there, so it can be put where it belongs before being tied down.
/// Paste a figure in, **converting it** where the two documents are in different units.
///
/// A figure copied out of a drawing in inches and pasted into one in millimetres is the same
/// figure, and 2 inches is 50.8 mm: converting is what keeps it so.  The scaling is on a copy of
/// the clipboard, so a clipboard may be pasted into any number of documents and is unchanged by
/// each.  A document that says nothing about its unit is in millimetres, and is converted as
/// one; two in the same unit are not scaled.
pub fn paste(sk: &mut Sketch, clip: &Sketch, dx: f64, dy: f64) -> Vec<EntRef> {
    match (clip.units.mm_per(), sk.units.mm_per()) {
        (from, to) if from != to => {
            let mut scaled = clip.clone();
            scaled.rescale(from / to);
            graft_onto(sk, &scaled, (dx, dy)).made_ents()
        }
        _ => graft_onto(sk, clip, (dx, dy)).made_ents(),
    }
}

/// The part of a sketch one gesture can move, as a sketch of its own.
///
/// Dragging a point can only move what is connected to it: the entities it belongs to, what
/// shares their points, what a constraint ties to those — and nothing past a fixed entity,
/// which does not move whatever is done on its other side.  Everything else in the document is
/// unaffected by the drag, so it is not worth compiling, decomposing or solving for it; a drag
/// on a sketch of many separate figures costs what the one figure costs.
///
/// The part is made by the same rebuild walk as deletion, copying and pasting (`graft`), so it is
/// an ordinary sketch: the plan and the systems are built on it unchanged, and the gesture writes
/// the moved parameters back through `write_back`.  Point indices differ between the two, so the
/// things a drag exchanges with its caller by point index — guard triangles, recorded flips,
/// branch keys — go through `point_in` / `point_out`.
pub struct Part {
    pub sketch: Sketch,
    /// part point index → document point index
    to_doc: Vec<usize>,
    /// document point index → part point index, where it came along
    to_part: Vec<Option<usize>>,
    /// (part param, document param) for every parameter of the part
    params: Vec<(usize, usize)>,
    /// document entity → part entity, for every entity that came along
    ents: BTreeMap<EntRef, EntRef>,
}

impl Part {
    /// The part around `seed`.  A fixed entity is a wall: it comes along (a constraint naming
    /// it must keep all of its entities) but nothing is reached through it.
    pub fn around(sk: &Sketch, seed: EntRef) -> Part {
        Part::around_all(sk, &[seed])
    }

    /// The part around every one of `seeds` — a face family's station, from its face's edges
    /// (`solid::family`); a face among them comes along when its edges do.
    pub fn around_all(sk: &Sketch, seeds: &[EntRef]) -> Part {
        let prims = sk.primitives();
        // who contains a point, and which constraints name an entity
        let mut parents: Vec<Vec<EntRef>> = vec![Vec::new(); sk.points.len()];
        // a curve is no primitive, but it moves with the points it is written over — a free
        // curve's ends among them (#144)
        let curves = (0..sk.curves.len()).map(|i| EntRef::new(EntKind::Curve, i));
        for e in prims.iter().copied().chain(curves) {
            for c in sk.children(e).into_iter().filter(|c| c.kind == EntKind::Point) {
                parents[c.i()].push(e);
            }
        }
        // Which constraints name an entity, and — because two dimensions written in terms of the
        // same free variable share an unknown, which is as real a tie as a shared point — which
        // of them read each free variable.  The walk follows both, so a part that reaches one of
        // a tied group reaches all of it.
        let mut named: BTreeMap<EntRef, Vec<usize>> = BTreeMap::new();
        let mut by_free: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        // A curve written over a number its drawn instance left unknown (a rod's length) has that
        // unknown as a column, so it is tied to the dimensions reading it just as they are to
        // each other, and a contact on the curve may move it.
        let free_param = |name: &String| sk.free_vars.get(name).copied();
        let mut curves_by_free: BTreeMap<u32, Vec<EntRef>> = BTreeMap::new();
        for (i, cv) in sk.curves.iter().enumerate() {
            for f in cv.unknowns.iter().filter_map(free_param) {
                curves_by_free.entry(f).or_default().push(EntRef::new(EntKind::Curve, i));
            }
        }
        for (ci, c) in sk.constraints.iter().enumerate() {
            // a claim constrains nothing, so it welds nothing: two figures a claim spans stay
            // two parts, and a drag of one costs the other nothing — nor does a bound (§9.6)
            if !c.states_rows() {
                continue;
            }
            for e in c.entities() {
                named.entry(e).or_default().push(ci);
            }
            if let Some(f) = c.free {
                by_free.entry(f.param).or_default().push(ci);
            }
        }
        // a ring's turned copy moves with its representative and its centre, and they with it
        let mut turned: BTreeMap<EntRef, Vec<EntRef>> = BTreeMap::new();
        for tn in &sk.turns {
            for (a, b) in [(tn.copy, tn.rep), (tn.copy, tn.about)] {
                turned.entry(a).or_default().push(b);
                turned.entry(b).or_default().push(a);
            }
        }
        // as an image does with what it is the image of (#578)
        for m in &sk.mirrors {
            turned.entry(m.copy).or_default().push(m.rep);
            turned.entry(m.rep).or_default().push(m.copy);
        }
        // a plane is a wall when it cannot move: its axes held as well as where it stands
        let wall = |e: EntRef| match e.kind {
            EntKind::Frame => sk.plane_fixed(e.i()),
            // a plane's origin is held at its own (0, 0) and stands wherever the plane does: what
            // is tied to it moves with a plane that moves
            EntKind::Point if let Some(pl) = sk.plane_of_origin(e.i()) => sk.plane_fixed(pl),
            // a motion owns no number, and moves with what it is written over: its axes (#366)
            EntKind::Motion => false,
            _ => sk.entity_params(e).iter().all(|&p| sk.params[p as usize].fixed),
        };
        let mut keep: BTreeSet<EntRef> = BTreeSet::new();
        let mut followed: BTreeSet<u32> = BTreeSet::new();
        let mut queue = seeds.to_vec();
        keep.extend(seeds.iter().copied());
        // a tied group — the dimensions and curves reading one free variable — only has to be
        // opened once, however many of its members the walk arrives at
        let mut open = |f: u32, next: &mut Vec<EntRef>| {
            if followed.insert(f) {
                for &cj in by_free.get(&f).map(|v| v.as_slice()).unwrap_or(&[]) {
                    next.extend(sk.constraints[cj].entities());
                }
                next.extend(curves_by_free.get(&f).map(|v| v.as_slice()).unwrap_or(&[]));
            }
        };
        while let Some(e) = queue.pop() {
            // a point comes with the plane it is drawn in, which places it in space — a held
            // point too, which is drawn there all the same
            if let Some(pl) = (e.kind == EntKind::Point).then(|| sk.plane_of(e.i())).flatten() {
                if keep.insert(EntRef::frame(pl)) {
                    queue.push(EntRef::frame(pl));
                }
            }
            if wall(e) {
                // a held entity is followed no further, but it is rebuilt over what it is made of
                // — a held spline over its control points (#134) — and those are held with it, so
                // they stop here in turn, each with its plane.  A plane's own are kept below.  A
                // body traced in space owns no parameter and is read over what its component was
                // given, so a station standing on one keeps that too (#363 S1).
                let made_of = !sk.entity_params(e).is_empty() || e.kind == EntKind::Traced;
                if e.kind != EntKind::Frame && made_of {
                    for c in sk.children(e) {
                        if keep.insert(c) {
                            queue.push(c);
                        }
                    }
                }
                continue;
            }
            let mut next = sk.children(e);
            if e.kind == EntKind::Point {
                next.extend(parents[e.i()].iter().copied());
            }
            next.extend(turned.get(&e).map(|v| v.as_slice()).unwrap_or(&[]));
            if e.kind == EntKind::Curve {
                for f in sk.curves[e.i()].unknowns.iter().filter_map(free_param) {
                    open(f, &mut next);
                }
            }
            for &ci in named.get(&e).map(|v| v.as_slice()).unwrap_or(&[]) {
                next.extend(sk.constraints[ci].entities());
                if let Some(f) = sk.constraints[ci].free {
                    open(f.param, &mut next);
                }
            }
            for n in next {
                if keep.insert(n) {
                    queue.push(n);
                }
            }
        }
        // a plane is rebuilt over its axes and its origin, wherever the walk stopped at it
        for i in 0..sk.frames.len() {
            if keep.contains(&EntRef::frame(i)) {
                keep.extend(sk.children(EntRef::frame(i)));
            }
        }
        let (sketch, r) = rebuild(sk, &|e| keep.contains(&e), &[]);
        let to_part: Vec<Option<usize>> = (0..sk.points.len()).map(|i| r.point(i)).collect();
        let mut to_doc = vec![0; sketch.points.len()];
        for (doc, part) in to_part.iter().enumerate() {
            if let Some(p) = part {
                to_doc[*p] = doc;
            }
        }
        let ents: BTreeMap<EntRef, EntRef> = r.pairs().iter().copied().collect();
        let mut params = Vec::new();
        for &(doc, part) in r.pairs() {
            match doc.kind {
                // a primitive's own numbers, in the order both sketches list them
                EntKind::Point | EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline
                | EntKind::Axis | EntKind::Frame => {
                    let pairs = sketch.entity_params(part).into_iter().zip(sk.entity_params(doc));
                    params.extend(pairs.map(|(a, b)| (a as usize, b as usize)));
                }
                // a free curve's length is its own number (#144)
                EntKind::Curve => {
                    if let (Some(a), Some(b)) = (sketch.curves[part.i()].length,
                                                 sk.curves[doc.i()].length) {
                        params.push((a as usize, b as usize));
                    }
                }
                _ => {}
            }
        }
        // and a constraint's own unknowns (a contact's place along its curve) with its copy's
        let copies: BTreeMap<u32, &Constraint> =
            sketch.constraints.iter().map(|c| (c.id, c)).collect();
        for c in &sk.constraints {
            let Some(copy) = r.constraint(c.id).and_then(|id| copies.get(&id)) else { continue };
            for (a, b) in copy.aux_params().into_iter().zip(c.aux_params()) {
                params.push((a as usize, b as usize));
            }
        }
        // and a hidden point moves with the view point it lifts
        for l in &sk.lifts {
            if let Some(k) = to_part[l.point as usize].and_then(|p| sketch.lift_of(p)) {
                for (x, y) in sketch.lifts[k].x.iter().zip(&l.x) {
                    params.push((*x as usize, *y as usize));
                }
            }
        }
        // the free variables came along by name: the rebuild allocated the part's own unknown
        // for each, seeded off the number the dimension carried, so the two are already in step
        for (name, &doc) in &sk.free_vars {
            if let Some(&mine) = sketch.free_vars.get(name) {
                params.push((mine as usize, doc as usize));
            }
        }
        Part { sketch, to_doc, to_part, params, ents }
    }

    /// A document entity as the part's, if it came along.
    pub fn entity_in(&self, doc: EntRef) -> Option<EntRef> {
        self.ents.get(&doc).copied()
    }

    /// A document point's index in the part, if it came along.
    pub fn point_in(&self, doc: usize) -> Option<usize> {
        self.to_part.get(doc).copied().flatten()
    }

    /// A part point's index in the document.
    pub fn point_out(&self, part: usize) -> usize {
        self.to_doc[part]
    }

    /// A triangle of document points as part points — `None` if any did not come along.
    pub fn triangle_in(&self, t: (usize, usize, usize)) -> Option<(usize, usize, usize)> {
        Some((self.point_in(t.0)?, self.point_in(t.1)?, self.point_in(t.2)?))
    }

    /// The triangles of `ts` that came along, as part points.  One that did not is dropped: it
    /// names geometry this part cannot move, so there is no orientation here for it to guard.
    pub fn triangles_in(&self, ts: &[(usize, usize, usize)]) -> Vec<(usize, usize, usize)> {
        ts.iter().filter_map(|&t| self.triangle_in(t)).collect()
    }

    pub fn triangle_out(&self, t: (usize, usize, usize)) -> (usize, usize, usize) {
        (self.point_out(t.0), self.point_out(t.1), self.point_out(t.2))
    }

    /// Recorded root choices of the part, keyed as the document keys them.
    pub fn branches_out(&self, b: &BTreeMap<String, i32>) -> BTreeMap<String, i32> {
        b.iter()
            .map(|(k, &v)| match decompose::branch_key_points(k) {
                Some(p) => decompose::branch_record(
                    [self.point_out(p[0]), self.point_out(p[1]), self.point_out(p[2])],
                    v >= 0,
                ),
                None => (k.clone(), v),
            })
            .collect()
    }

    /// Copy the part's parameter values back into the document it came from.
    pub fn write_back(&self, sk: &mut Sketch) {
        for &(a, b) in &self.params {
            sk.params[b].value = self.sketch.params[a].value;
        }
        // this is a parameter write of its own — it does not go through `Sketch::set_x` — so the
        // dimensions written in terms of a free variable are brought up to date here too, or a
        // plan drag would leave the constraint list reading the number from before the gesture
        expr::sync_free(sk);
    }
}
