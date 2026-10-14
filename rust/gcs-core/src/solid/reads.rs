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
    reads_except(sk, si, unit, &std::collections::BTreeSet::new())
}

/// `reads`, with the solids `skip` and what only they reach left out: what a stratum reads
/// (`faces::cached`).
pub fn reads_except(sk: &Sketch, si: usize, unit: f64, skip: &std::collections::BTreeSet<usize>)
    -> Vec<f64> {
    reads_where(sk, si, unit, &|s| skip.contains(&s))
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

/// `reads`, leaving out every solid but `si` that `skip` says and what only they reach.
fn reads_where(sk: &Sketch, si: usize, unit: f64, skip: &dyn Fn(usize) -> bool) -> Vec<f64> {
    let mut v = vec![unit];
    let mut seen = std::collections::BTreeSet::new();
    let mut stack = vec![si as u32];
    while let Some(s) = stack.pop() {
        if !seen.insert(s) || (s as usize != si && skip(s as usize)) {
            continue;
        }
        let Some(sol) = sk.solids.get(s as usize) else { continue };
        v.push(s as f64);
        name_read(&sol.name, &mut v);
        match &sol.def {
            SolidDef::Swept { source, motion, roll: Roll::Stated { from, to } } => {
                v.extend([6.,*source as f64,*motion as f64,from.value,to.value]);
                motion_reads(sk,*motion,&mut v);
            }
            // a roll through a body reads its material, never its cuts (`material_sources`)
            SolidDef::Swept { source, motion, roll: Roll::Through { body } } => {
                v.extend([8.,*source as f64,*motion as f64,*body as f64]);
                motion_reads(sk,*motion,&mut v);
                match material_sources(sk, *body) {
                    Ok(sources) => stack.extend(sources),
                    Err(_) => v.push(f64::MAX),
                }
            }
            SolidDef::Placed { source, motion, at } => {
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
                v.push(7.0);
                for t in terms {
                    // no NaN for an end not written: a key must equal itself
                    let hi = [f64::from(t.hi.is_some()), t.hi.unwrap_or(0.0)];
                    // an unknown's as solved; one nothing determines reads as a number no other
                    // key holds, and the solid refuses it where it is evaluated
                    let lo = t.lo.read(sk).unwrap_or(UNREAD);
                    let grow = t.grow_read(sk);
                    v.extend([t.cmp as u8 as f64, lo, hi[0], hi[1], grow.unwrap_or(UNREAD)]);
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
            SolidDef::Family { face, station, held, from, to } => {
                v.extend([9.0, station.len() as f64, from.value, to.value]);
                name_read(held, &mut v);
                face_reads(sk, *face, &mut v);
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
                face_reads(sk, *face, &mut v);
            }
            SolidDef::Through { face, body } => {
                v.extend([3.0, *body as f64]);
                face_reads(sk, *face, &mut v);
                // Read the material graph too, including changes to additions and placement.
                stack.push(*body);
            }
            SolidDef::Revolve { face, axis, sweep, sense } => {
                v.extend([1.0, axis.kind as u8 as f64, axis.i() as f64]);
                v.push(sweep.value);
                v.push(if *sense == Sense::Cw { -1.0 } else { 1.0 });
                face_reads(sk, *face, &mut v);
                // a line's ends or an axis's direction and foot; the plane is the face's, read
                v.extend(sk.entity_params(*axis).iter().map(|&p| sk.params[p as usize].value));
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

    /// What solid `si` reads but the solids `skip` (`reads_except`).
    pub fn except(sk: &Sketch, si: usize, skip: &std::collections::BTreeSet<usize>) -> MemoKey {
        MemoKey { reads: reads_except(sk, si, 0.0, skip), strata: Vec::new() }
    }
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

fn face_reads(sk: &Sketch, fi: u32, v: &mut Vec<f64>) {
    v.push(fi as f64);
    let Some(f) = sk.faces.get(fi as usize) else { return };
    let plane = match f.support {
        crate::model::FaceSupport::Plane(p) => p,
        crate::model::FaceSupport::Surface(s) => {
            v.extend([-2.,s.kind as u32 as f64,s.idx as f64]);
            return;
        }
    };
    v.extend([plane.map_or(-1.0, |p| p as f64), f.holes.len() as f64]);
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
    if let Some(p) = plane.filter(|&p| (p as usize) < sk.planes.len()) {
        let basis = sk.basis(p as usize);
        v.extend(basis.u);
        v.extend(basis.v);
        v.extend(basis.o);
    }
}

fn motion_reads(sk: &Sketch,motion: u32,values: &mut Vec<f64>) {
    let mut pending = vec![motion];
    let mut seen = std::collections::BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue; }
        let Some(m) = sk.motions.get(i as usize) else { values.push(UNREAD); continue; };
        values.push(i as f64);
        // a number written as a measurement is read as it now comes to, so the key moves
        // exactly when the measured geometry does (`MotionE::rotation`)
        match m.def {
            crate::model::MotionDef::Rotation {axis,..} => {
                let (ratio,phase,advance) = m.rotation(sk).unwrap_or((UNREAD,UNREAD,UNREAD));
                values.extend([0.,axis as f64,ratio,phase,advance]);
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
                let (ratio,phase,_) = m.rotation(sk).unwrap_or((UNREAD,UNREAD,UNREAD));
                values.extend([3.,centre as f64,ratio,phase]);
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
        }
    }
}
