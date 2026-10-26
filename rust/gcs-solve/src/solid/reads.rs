//! What a solid reads (`reads`): every number it was built from, in one flat vector, and the
//! memo a result is remembered in against it (`MemoKey`, `Memo`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use super::document::material_sources;

/// **Every number a solid was built from**, in one flat vector — the memo key.
///
/// `curve_polyline`'s bargain: a result is remembered against what it reads rather than
/// invalidated by whoever writes, so a repaint over a drawing that has not changed costs a
/// comparison.  Every scalar of every edge of every face the term reaches, each plane's pose,
/// basis and origin, and every extent.
pub fn reads(sk: &Sketch, si: usize, unit: f64) -> Vec<f64> {
    reads_where(sk, si, unit, &|_| false)
}

/// What solid `si` reads of its own, following no other solid it names: a region's terms read
/// apart from the body whose faces they read (`lowering::lowered`, which keys that body apart).
pub fn reads_own(sk: &Sketch, si: usize, unit: f64) -> Vec<f64> {
    reads_where(sk, si, unit, &|_| true)
}

/// What a key holds for a number it cannot read (a placement whose motion refuses, a motion that
/// is not there): never a NaN, since a key must equal itself and `Vec<f64>`'s equality never holds
/// over one, which would build the solid again at every ask.
const UNREAD: f64 = f64::MAX;

/// `reads`, leaving out every solid but `si` that `skip` says (asked only of solids in range) and
/// what only they reach.
fn reads_where(sk: &Sketch, si: usize, unit: f64, skip: &dyn Fn(usize) -> bool) -> Vec<f64> {
    reads_with(sk, si, unit, skip, &mut Poses::World)
}

/// How a walk writes the pose of a plane it reads (#412).
enum Poses {
    /// In the world, with every other number: `reads`, the key every other cache keeps.
    World,
    /// Relative to the plane `to`, with `rigid` false once anything read stands in the world
    /// itself, and `reach` the largest number a pose was worked from: a `ShapeKey`'s.
    Relative { to: Basis, rigid: bool, reach: f64 },
}

impl Poses {
    /// Something read stands in the world and not in a plane the walk reads (a region's points, a
    /// motion's axis, a point in space): a rigid move of the planes does not carry it, so the
    /// solid is not one that only moved.
    fn world(&mut self) {
        if let Poses::Relative { rigid, .. } = self {
            *rigid = false;
        }
    }

    /// Plane `b`'s pose into `v`: in the world, or relative to the reference.
    fn plane(&mut self, b: &Basis, v: &mut Vec<f64>) {
        match self {
            Poses::World => {
                v.extend(b.u);
                v.extend(b.v);
                v.extend(b.o);
            }
            Poses::Relative { to, reach, .. } => {
                *reach = b.o.iter().fold(*reach, |m, x| m.max(x.abs()));
                let axes = [to.u, to.v, to.normal()];
                let local = |x: [f64; 3]| axes.map(|a| crate::plane::dot(a, x));
                v.extend(local(b.u));
                v.extend(local(b.v));
                v.extend(local(crate::space::sub(b.o, to.o)));
            }
        }
    }
}

/// The plane face `f` is drawn in, in space: the page for a face of none.
fn face_basis(sk: &Sketch, f: &crate::model::FaceE) -> Basis {
    match f.plane.filter(|&p| (p as usize) < sk.frames.len()) {
        Some(p) => sk.basis(p as usize),
        None => Basis::page(),
    }
}

/// **The reference plane of solid `si`** (#412): its first face's, found as its origin is
/// (`frame_origin`) — what its shape key reads every other plane relative to, and what a rigid
/// move of it is measured by.  The page for a solid with no face.
fn reference(sk: &Sketch, si: usize) -> Basis {
    let mut pending = vec![si];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(s) = sk.solids.get(i) else { continue };
        if let SolidDef::Placed { source, .. } = &s.def {
            pending.push(*source as usize);
            continue;
        }
        if let Some(f) = s.face().and_then(|f| sk.faces.get(f as usize)) {
            return face_basis(sk, f);
        }
        pending.extend(s.operands().into_iter().rev().map(|o| o as usize));
    }
    Basis::page()
}

/// Whether point `p` is drawn in `plane` (the page for none), its numbers that plane's own.
fn drawn_in(sk: &Sketch, p: usize, plane: Option<u32>) -> bool {
    sk.points.get(p).is_some_and(|q| q.z.is_none() && q.chart.is_none() && q.plane == plane)
}

/// `reads`, each plane's pose written as `poses` says.
fn reads_with(sk: &Sketch, si: usize, unit: f64, skip: &dyn Fn(usize) -> bool, poses: &mut Poses)
    -> Vec<f64> {
    let mut v = vec![unit];
    // a flag per solid: a body rounded all over has a term per edge and corner
    let mut seen = vec![false; sk.solids.len()];
    let mut stack = vec![si as u32];
    while let Some(s) = stack.pop() {
        let Some(sol) = sk.solids.get(s as usize) else { continue };
        if std::mem::replace(&mut seen[s as usize], true) || (s as usize != si && skip(s as usize)) {
            continue;
        }
        v.push(s as f64);
        name_read(&sol.name, &mut v);
        match &sol.def {
            SolidDef::Swept { source, motion, roll: Roll::Stated(r) } => {
                poses.world();
                let [from, to] = r.radians();
                v.extend([6.,*source as f64,*motion as f64,from,to]);
                motion_reads(sk,*motion,&mut v);
            }
            // a roll through a body reads its material, never its cuts (`material_sources`)
            SolidDef::Swept { source, motion, roll: Roll::Through { body } } => {
                poses.world();
                v.extend([8.,*source as f64,*motion as f64,*body as f64]);
                motion_reads(sk,*motion,&mut v);
                match material_sources(sk, *body) {
                    Ok(sources) => stack.extend(sources),
                    Err(_) => v.push(f64::MAX),
                }
            }
            SolidDef::Placed { source, motion, at } => {
                poses.world();
                v.extend([5.,*source as f64,*motion as f64,at.value]);
                match crate::motion::Family::read(sk,*motion as usize).and_then(|m| m.at(at.value)) {
                    Ok(pose) => {
                        v.extend(pose.point([0.;3]));
                        for i in 0..3 { let mut p = [0.;3]; p[i] = 1.; v.extend(pose.vector(p)); }
                    }
                    Err(_) => v.push(UNREAD),
                }
            }
            // a region: each term's numbers and where what it measures stands
            SolidDef::Region { terms, .. } => {
                poses.world();
                v.push(7.0);
                for t in terms {
                    // no NaN for an end not written: a key must equal itself
                    let hi = [f64::from(t.hi.is_some()), t.hi.unwrap_or(0.0)];
                    // an unknown's as solved; one nothing determines reads as a number no other
                    // key holds, and the solid refuses it where it is evaluated
                    let lo = t.lo.read(sk).unwrap_or(UNREAD);
                    let grow = t.grow_read(sk);
                    v.extend([t.cmp as u8 as f64, lo, hi[0], hi[1], grow.unwrap_or(UNREAD)]);
                    // a panel's band (#645): its rails as paths, read whole
                    if let crate::model::RegionShape::Sheet { rails, side } = &t.shape {
                        v.push(*side);
                        for r in rails.iter() {
                            match r.path(sk) {
                                Ok(p) => p.encode(&mut v),
                                Err(_) => v.push(UNREAD),
                            }
                        }
                        continue;
                    }
                    for e in t.shape.entities() {
                        match e.kind {
                            crate::model::EntKind::Point => v.extend(sk.world_point(e.i())),
                            crate::model::EntKind::Line => {
                                let l = &sk.lines[e.i()];
                                v.extend(sk.world_point(l.p1 as usize));
                                v.extend(sk.world_point(l.p2 as usize));
                            }
                            crate::model::EntKind::Axis => {
                                let x = &sk.axes[e.i()];
                                v.extend(x.a.iter().chain(&x.d).map(|&p| sk.params[p as usize].value));
                            }
                            // a body whose faces it reads (#183 F1): read as the body is
                            crate::model::EntKind::Solid => stack.push(e.i() as u32),
                            _ => {
                                let b = sk.basis(e.i());
                                v.extend(b.o);
                                v.extend(b.u);
                                v.extend(b.v);
                            }
                        }
                    }
                }
            }
            // a family reads what its stations are solved over: every parameter of the drawing
            // and every number a constraint states, since a dimension of the station's reads
            // the running number and so stands at none of the values it now holds
            SolidDef::Family { face, station, over, legs } => {
                poses.world();
                v.extend([9.0, station.len() as f64, over.from, over.to, legs.len() as f64]);
                name_read(&over.name, &mut v);
                face_reads(sk, *face, &mut v, poses);
                for l in legs {
                    v.extend([l.at, l.ents.len() as f64]);
                    name_read(&l.held, &mut v);
                    face_reads(sk, l.face, &mut v, poses);
                }
                v.extend(station.iter().flat_map(|e| [e.kind as u32 as f64, e.i() as f64]));
                v.extend(sk.params.iter().flat_map(|p| [p.value, f64::from(u8::from(p.fixed))]));
                for c in &sk.constraints {
                    v.push(c.kind as u32 as f64);
                    v.extend(c.args.iter().map(|a| match a {
                        crate::constraints::Arg::Num(x) => *x,
                        crate::constraints::Arg::Expr(e) => e.value,
                        crate::constraints::Arg::Param(p) => -1.0 - f64::from(*p),
                        crate::constraints::Arg::Int(k) => *k as f64,
                        _ => 0.0,
                    }));
                }
            }
            SolidDef::Prism { face, from, to, draft } => {
                v.push(0.0);
                v.push(from.value);
                v.push(to.value);
                v.push(draft.as_ref().map_or(0.0, |d| d.value));
                face_reads(sk, *face, &mut v, poses);
            }
            SolidDef::Through { face, body } => {
                v.extend([3.0, *body as f64]);
                face_reads(sk, *face, &mut v, poses);
                // Read the material graph too, including changes to additions and placement.
                stack.push(*body);
            }
            SolidDef::Revolve { face, axis, sweep, sense } => {
                v.extend([1.0, axis.kind as u8 as f64, axis.i() as f64]);
                v.push(sweep.value);
                v.push(if *sense == Sense::Cw { -1.0 } else { 1.0 });
                face_reads(sk, *face, &mut v, poses);
                // a line's ends or an axis's direction and foot; the plane is the face's, read
                v.extend(sk.entity_params(*axis).iter().map(|&p| sk.params[p as usize].value));
                // an axis stands in space, and a line in another plane is read in that one's
                let plane = sk.faces.get(*face as usize).and_then(|f| f.plane);
                let in_face = |(a, b): (u32, u32)| {
                    [a, b].iter().all(|&p| drawn_in(sk, p as usize, plane))
                };
                if axis.kind != EntKind::Line || !crate::model::edge_ends(sk, *axis).is_some_and(in_face) {
                    poses.world();
                }
            }
            SolidDef::Body { stock, on, through, bound } => {
                v.extend([2.0, *stock as f64, on.len() as f64]);
                v.extend(on.iter().map(|&i| i as f64));
                v.push(through.len() as f64);
                v.extend(through.iter().map(|&i| i as f64));
                v.push(bound.len() as f64);
                v.extend(bound.iter().map(|&i| i as f64));
            }
        }
        stack.extend(sol.operands());
    }
    v
}

/// **What a result built for a solid was built from** — the memo key every per-solid cache keeps
/// (`Memo`): the numbers it reads (`reads`, or one of its narrower walks) and the strata it reads
/// (`faces::cached`'s), each told by identity, since a stratum is the same one exactly while its
/// own key holds.
#[derive(Clone, Debug, Default)]
pub struct MemoKey {
    reads: Vec<f64>,
    strata: Vec<std::sync::Arc<super::faces::BodyFaces>>,
}

impl PartialEq for MemoKey {
    fn eq(&self, o: &MemoKey) -> bool {
        self.reads == o.reads && self.strata.len() == o.strata.len()
            && self.strata.iter().zip(&o.strata).all(|(a, b)| std::sync::Arc::ptr_eq(a, b))
    }
}

impl MemoKey {
    /// Everything solid `si` reads (`reads`).
    pub fn of(sk: &Sketch, si: usize) -> MemoKey {
        MemoKey { reads: reads(sk, si, 0.0), strata: Vec::new() }
    }

    /// What solid `si` reads, at the unit it is faceted at (`reads`).
    pub fn at(sk: &Sketch, si: usize, unit: f64) -> MemoKey {
        MemoKey { reads: reads(sk, si, unit), strata: Vec::new() }
    }

    /// What solid `si` reads of its own (`reads_own`), and the strata it reads.
    pub fn own(sk: &Sketch, si: usize, strata: Vec<std::sync::Arc<super::faces::BodyFaces>>) -> MemoKey {
        MemoKey { reads: reads_own(sk, si, 0.0), strata }
    }

    /// What solid `si` reads but the solids `skip` says and what only they reach (`reads_where`),
    /// asked of each it reaches: what a stratum reads (`faces::cached`).
    pub fn except(sk: &Sketch, si: usize, skip: &dyn Fn(usize) -> bool) -> MemoKey {
        MemoKey { reads: reads_where(sk, si, 0.0, skip), strata: Vec::new() }
    }
}

/// How near two keys' numbers must be to be one shape (#412, #498): the solve's own step
/// tolerance (`solve::XTOL`), of the larger number or of the key's reach where that is larger —
/// the largest number in the world a pose in it was worked from (`ShapeKey::reach`).  A document
/// solve takes no step shorter than `XTOL` of the size of its unknowns, of which the reach is
/// one, so where it settles a part it does not move (a drag's polish solves the whole drawing)
/// it may leave its numbers anywhere that near, and no nearer is a pose it can tell apart: within
/// it the shape is the one solved, and outside it something changed.  So the bound is the
/// solve's and the drawing's, not a number fitted to a case: a change to row scaling or to a
/// kernel's degree moves where a solve settles, not how far a step it does not take is.  The
/// rounding of a pose worked through the world is far inside it, and a drawing stood at an
/// instant leaves what the parameter does not reach to the bit (`follow::Still`).
const NEAR: f64 = crate::solve::XTOL;

/// **What a solid reads, split into its shape and its pose** (#412): everything `reads` reads,
/// but with every plane's pose rewritten relative to a reference plane (its first face's,
/// `reference`) — the *shape* — and that reference plane's world pose.  Two keys of one shape
/// are one solid moved rigidly: what was evaluated for one is the other's too, moved by the motion
/// that carries one reference onto the other (`Rigid::between`).  A solid that reads anything
/// standing in the world itself — a region's points, a face family's stations, a body placed or
/// swept under a motion, a curve that may be traced through anything — has its pose in its shape
/// as well, so it is the same shape only where it stands where it stood.
#[derive(Clone, Debug, PartialEq)]
pub struct ShapeKey {
    shape: Vec<f64>,
    pose: Basis,
    /// The largest number, in the world, a pose the shape reads was worked from: the reference's
    /// origin and each plane's, and one.
    reach: f64,
}

impl ShapeKey {
    /// Solid `si`'s key, as the drawing stands.
    pub fn of(sk: &Sketch, si: usize) -> ShapeKey {
        let pose = reference(sk, si);
        let reach = pose.o.iter().fold(1.0, |m: f64, x| m.max(x.abs()));
        let mut poses = Poses::Relative { to: pose, rigid: true, reach };
        let mut shape = reads_with(sk, si, 0.0, &|_| false, &mut poses);
        let Poses::Relative { rigid, reach, .. } = poses else { unreachable!() };
        if !rigid {
            shape.extend(pose.u.iter().chain(&pose.v).chain(&pose.o));
        }
        ShapeKey { shape, pose, reach }
    }

    /// Where its reference plane stands in the world.
    pub fn pose(&self) -> Basis {
        self.pose
    }

    /// Whether `other` is this solid's shape, wherever it stands: what was built against one, moved
    /// from this pose to the other's, is what the other would build — every number the same
    /// within `NEAR`.
    pub fn same_shape(&self, other: &ShapeKey) -> bool {
        self.shape.len() == other.shape.len()
            && near(&self.shape, &other.shape, self.reach.max(other.reach))
    }

    /// Whether `other` stands where this does, within `NEAR`: the same shape there is the same
    /// solid, unmoved.
    pub fn same_pose(&self, other: &ShapeKey) -> bool {
        let numbers = |b: &Basis| [b.u, b.v, b.o].concat();
        near(&numbers(&self.pose), &numbers(&other.pose), self.reach.max(other.reach))
    }
}

/// Whether two lists of numbers are one within `NEAR` of the larger of each pair, or of `reach`.
fn near(a: &[f64], b: &[f64], reach: f64) -> bool {
    a.iter().zip(b)
        .all(|(&a, &b)| a == b || (a - b).abs() <= NEAR * a.abs().max(b.abs()).max(reach))
}

/// **A result per solid, remembered against what it was built from** (`MemoKey`):
/// `curve_polyline`'s bargain, built again only where what it reads has moved.  Interior, so
/// `&Sketch` readers share it; a cache and not state, since a miss builds what a hit remembers.
#[derive(Clone, Debug)]
pub struct Memo<V>(std::cell::RefCell<std::collections::BTreeMap<usize, (MemoKey, V)>>);

impl<V> Default for Memo<V> {
    fn default() -> Memo<V> { Memo(Default::default()) }
}

impl<V> Memo<V> {
    /// Forget everything remembered.
    pub fn clear(&self) {
        self.0.borrow_mut().clear();
    }
}

impl<V: Clone> Memo<V> {
    /// What is remembered for solid `i`, where it was built against `key`.
    pub fn get(&self, i: usize, key: &MemoKey) -> Option<V> {
        self.0.borrow().get(&i).filter(|(old, _)| old == key).map(|(_, v)| v.clone())
    }

    /// Remember `value` for solid `i`, built against `key`.
    pub fn put(&self, i: usize, key: MemoKey, value: V) {
        self.0.borrow_mut().insert(i, (key, value));
    }

    /// What is remembered for solid `i` against `key`, else what `build` makes, remembered.  The
    /// memo is not borrowed while `build` runs, so it may ask this memo of other solids.
    pub fn get_or(&self, i: usize, key: MemoKey, build: impl FnOnce() -> V) -> V {
        if let Some(v) = self.get(i, &key) {
            return v;
        }
        let v = build();
        self.put(i, key, v.clone());
        v
    }
}

fn name_read(name: &str, v: &mut Vec<f64>) {
    v.push(name.len() as f64);
    v.extend(name.bytes().map(f64::from));
}

fn face_reads(sk: &Sketch, fi: u32, v: &mut Vec<f64>, poses: &mut Poses) {
    v.push(fi as f64);
    let Some(f) = sk.faces.get(fi as usize) else { return };
    v.extend([f.plane.map_or(-1.0, |p| p as f64), f.holes.len() as f64]);
    for (edges, edge_names) in f.boundaries() {
        v.push(edge_names.len() as f64);
        for name in edge_names { name_read(name, v); }
        v.push(edges.len() as f64);
        for e in edges {
            v.extend([e.kind as u32 as f64, e.i() as f64]);
            // Closure depends on point identity, even when points share coordinates/parameters.
            let ends = crate::model::edge_ends(sk, *e);
            v.extend(ends.map_or([-1.0; 2], |(a, b)| [a as f64, b as f64]));
            let params = sk.entity_params(*e);
            v.push(params.len() as f64);
            v.extend(params.iter().map(|&p| sk.params[p as usize].value));
            // a curve's stretch moves with more than its arguments: the contacts its interval is
            // read from, the unknown its trace is anchored at, the pose it starts from, and the
            // points it runs between (what `Sketch::curve_polyline` keys on, and the ends)
            // a point read in space or in another plane than the face's moves apart from it, and
            // a curve may be traced through anything (#412)
            if e.kind == EntKind::Curve || sk.children(*e).iter()
                .any(|c| c.kind == EntKind::Point && !drawn_in(sk, c.i(), f.plane)) {
                poses.world();
            }
            if e.kind == EntKind::Curve {
                let (a, b) = sk.curve_domain(e.i());
                v.extend(sk.curve_vars(e.i(), a));
                v.extend([b, sk.curve_home(e.i())]);
                v.extend(sk.curve_pose(e.i()).unwrap_or_default());
                if let Some((p, q)) = ends {
                    for i in [p, q] { let (x, y) = sk.point_xy(i as usize); v.extend([x, y]); }
                }
            }
        }
    }
    // the page stands still, so a face of none reads no pose in the world: relative to the
    // reference, it is one more plane that moves
    if f.plane.is_some_and(|p| (p as usize) < sk.frames.len()) || !matches!(poses, Poses::World) {
        poses.plane(&face_basis(sk, f), v);
    }
}

fn motion_reads(sk: &Sketch,motion: u32,values: &mut Vec<f64>) {
    let mut pending = vec![motion];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(m) = sk.motions.get(i as usize) else { values.push(UNREAD); continue; };
        values.push(i as f64);
        // a joint's number the solve finds is read as it now stands, so the key moves exactly
        // when the solve moves it (`MotionE::rotation`)
        match m.def {
            crate::model::MotionDef::Rotation {axis,..} => {
                let (ratio,advance) = m.rotation(sk).unwrap_or((UNREAD,UNREAD));
                values.extend([0.,axis as f64,ratio,advance]);
                if let Some(axis) = sk.lines.get(axis as usize) {
                    for p in [axis.p1,axis.p2] { values.extend(sk.world_point(p as usize)); }
                } else { values.push(UNREAD); }
            }
            crate::model::MotionDef::Translation {axis,..} => {
                values.extend([2.,axis as f64,m.advance(sk).unwrap_or(UNREAD)]);
                if let Some(axis) = sk.lines.get(axis as usize) {
                    for p in [axis.p1,axis.p2] { values.extend(sk.world_point(p as usize)); }
                } else { values.push(UNREAD); }
            }
            crate::model::MotionDef::Turn {centre,..} => {
                let (ratio,_) = m.rotation(sk).unwrap_or((UNREAD,UNREAD));
                values.extend([3.,centre as f64,ratio]);
                if (centre as usize) < sk.points.len() {
                    values.extend(sk.world_point(centre as usize));
                    let b = sk.plane_of(centre as usize).map(|i| sk.basis(i));
                    values.extend(b.map_or([0.,0.,1.],|b| b.normal()));
                } else { values.push(UNREAD); }
            }
            crate::model::MotionDef::Relative {source,observer} => {
                values.extend([1.,source as f64,observer as f64]);
                pending.extend([observer,source]);
            }
            crate::model::MotionDef::Inverse {of} => {
                values.extend([4.,of as f64]);
                pending.push(of);
            }
            // a drive's law, and every held number it may read where it now stands (#391 S5)
            crate::model::MotionDef::Driven {of,ref law,ref over} => {
                values.extend([6.,of as f64]);
                values.extend(law.bytes().chain([0]).chain(over.bytes()).map(f64::from));
                values.extend(sk.held.values().map(|h| sk.params[h.param as usize].value));
                pending.push(of);
            }
            // a frame along a path: the path's numbers and where the frame is drawn (#398 C)
            crate::model::MotionDef::Guided {guide,frame,frenet} => {
                values.extend([7.,guide.kind as u8 as f64,guide.i() as f64,frame as f64,
                               f64::from(u8::from(frenet))]);
                values.extend(sk.entity_params(guide).iter().map(|&q| sk.params[q as usize].value));
                for e in sk.children(guide) {
                    values.extend(sk.entity_params(e).iter().map(|&q| sk.params[q as usize].value));
                }
                match sk.frames.get(frame as usize).and_then(|f| f.chart.as_ref()) {
                    Some(c) => values.extend(c.o.iter().map(|&q| sk.params[q as usize].value)),
                    None => values.push(UNREAD),
                }
            }
            // a frame the drawing moves: the march it is read off, and where it now stands, from
            // which it carries (#398 F)
            crate::model::MotionDef::Followed {frame,ref over} => {
                values.extend([8.,frame as f64]);
                values.extend(over.bytes().map(f64::from));
                values.extend(crate::follow::march_key(sk,over));
                if (frame as usize) < sk.frames.len() {
                    let b = sk.basis(frame as usize);
                    values.extend(b.o.into_iter().chain(b.u).chain(b.v));
                } else { values.push(UNREAD); }
            }
            crate::model::MotionDef::Joint {axis,ratio,advance} => {
                values.extend([5.,axis as f64,ratio,advance]);
                match sk.axes.get(axis as usize) {
                    Some(r) => values.extend(r.a.iter().chain(&r.d)
                        .map(|&q| sk.params[q as usize].value)),
                    None => values.push(UNREAD),
                }
            }
        }
    }
}
