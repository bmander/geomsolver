//! **Intrinsic rows, derived from the model** (#831).  An intrinsic row is a constraint the
//! drawing adds itself — an axis's unit row, a plane's axes through its place, a normal square to
//! its plane, a point's lift, a twin's tie — that the document never states: each follows from
//! what an entity *is*.  Every such fact is a field of the entity it is about (`AxisE::placed`,
//! `Chart::square`, `Chart::along`, `Chart::place`, `ArcE::round`, a `CurveBody::Developed`, …),
//! and `Sketch::rows_of` is the one place a row is made from one: what an owner implies, as the
//! model now says.  The rows are a cache of it, stored in `constraints` so the solve, the block
//! order and the diagnosis read one list, each saying whose it is (`Origin::Intrinsic`) — and
//! never read back as a fact: a reader asks the field.
//!
//! Copying the model (`graft`, the JSON reader) copies entities and their fields, each through the
//! constructor that derives its rows, and then derives every owner's rows (`derive_all`), so a
//! structural fact is never lost by a copy that did not know to recognise its row.

use super::*;
use crate::constraints::{Arg, CKind, Constraint};

/// **Where a constraint comes from**: stated by the document, or implied by what an entity is,
/// and then whose it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Origin {
    Stated,
    Intrinsic(Owner),
}

/// **What an intrinsic row is derived from** — exhaustive: a new structural fact is a new arm, and
/// `Sketch::rows_of` stops the build until it says what the fact implies.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Owner {
    /// An axis: its direction of unit length, and its place the foot of the perpendicular from
    /// the origin once a relation reads it (`AxisE::placed`) — neither for an image's axis, whose
    /// representative's hold for it (#578).
    Axis(u32),
    /// An arc: its ends on its circle — none for a ring's turned copy or a mirrored image, which
    /// are their representative's turned or reflected.
    Arc(u32),
    /// A frame's chart: its axes through its place (`Chart::place`), square where it places an
    /// instance rigidly (`Chart::square`), held along drawn lines (`Chart::along`) and standing
    /// where they meet (`Chart::stands_at`).  None for a mirrored image's plane.
    Frame(u32),
    /// A frame's normal, by the frame (`Chart::normal`): square to its `u`, a quarter turn on
    /// from its `v`, through its origin.
    Normal(u32),
    /// A point's hidden point in space, by the point (`Sketch::lifts`): held where the point is
    /// drawn, in its plane or its spatial frame's chart.
    Lift(u32),
    /// A point and its twin (§6.7, `Sketch::twins`): the twin on the point's plane and folded
    /// onto it, and the point on the twin's plane where nothing else holds it there.
    Twin(u32, u32),
    /// An arc or a circle drawn in a plane through points in space (#363 S6, `RoundIn`).
    Round(EntRef),
    /// A development's rail (#645, `CurveBody::Developed`): the home ruling's image as long as
    /// the ruling, one row of the first rail.
    Developed(u32),
}

/// **An axis of a chart held along a drawn line** (#535): parallel to `line`, the same way, and
/// through `at`, the end of it where the chart's other axis meets it, else its start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Along {
    pub line: u32,
    pub at: u32,
}

/// **Where a chart stands** (#542): at a place of its own, or at frame `Of`'s — a datum of that
/// frame, whose `Chart::o` are the frame's three Params, the one place both stand at.  An axis
/// already through that place (one of a frame built before it standing there) is placed by no
/// row of the datum's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Place {
    Own,
    Of(u32),
}

/// **An arc's or a circle's plane in space** (#363 S6): drawn over points of its own in `plane`
/// — its centre the plane's origin, an arc's ends in it — each tied to the point in space the
/// document wrote, `ties` as (its own point, the point written); and for a circle the axis or line
/// it stands square to with the axis its plane's turn is gauged by (`normal`), which a solve
/// takes again where it leaves the normal near it (`Sketch::retake_round_gauges`).
#[derive(Clone, Debug, PartialEq)]
pub struct RoundIn {
    pub plane: u32,
    pub ties: Vec<[u32; 2]>,
    pub normal: Option<(EntRef, u32)>,
}

impl RoundIn {
    /// The same plane over the entities `map` renumbers, or `None` where one did not come.
    pub fn remapped(&self, map: &Remap) -> Option<RoundIn> {
        let RoundIn { plane, ties, normal } = self;
        let point = |p: u32| map(EntRef::point(p as usize)).map(|e| e.idx);
        Some(RoundIn {
            plane: map(EntRef::frame(*plane as usize))?.idx,
            ties: ties.iter().map(|&[a, b]| Some([point(a)?, point(b)?])).collect::<Option<_>>()?,
            normal: match normal {
                Some((n, g)) => Some((map(*n)?, map(EntRef::new(EntKind::Axis, *g as usize))?.idx)),
                None => None,
            },
        })
    }
}

impl Sketch {
    /// **The rows `owner` implies, as the model now says** — the only place an intrinsic row is
    /// made.  Empty for an owner that implies none now (an image's axis, a frame with no chart).
    pub fn rows_of(&self, owner: Owner) -> Vec<Constraint> {
        let axis = |a: u32| Arg::Ent(EntRef::new(EntKind::Axis, a as usize));
        let row = |kind: CKind, args: Vec<Arg>| Constraint::new(kind, args);
        match owner {
            Owner::Axis(i) => {
                let e = EntRef::new(EntKind::Axis, i as usize);
                if i as usize >= self.axes.len() || self.imaged(e) {
                    return Vec::new();
                }
                let mut rows = vec![row(CKind::AxisUnit, vec![Arg::Ent(e)])];
                if self.axes[i as usize].placed {
                    rows.push(row(CKind::AxisFoot, vec![Arg::Ent(e)]));
                }
                rows
            }
            Owner::Arc(i) => {
                let e = EntRef::arc(i as usize);
                if i as usize >= self.arcs.len() || self.imaged(e)
                    || self.turns.iter().any(|t| t.copy == e)
                {
                    return Vec::new();
                }
                let a = &self.arcs[i as usize];
                [a.start, a.end].map(|p| row(CKind::PointOnCircle,
                                             vec![Arg::Ent(EntRef::point(p as usize)), Arg::Ent(e)]))
                    .into()
            }
            Owner::Frame(f) => {
                let Some(c) = self.frames.get(f as usize).and_then(|g| g.chart.as_ref()) else {
                    return Vec::new();
                };
                if self.imaged(EntRef::frame(f as usize)) {
                    return Vec::new();
                }
                // a datum's axis already through its frame's place: one of a frame standing there
                // built before it (`plane_at`)
                let through: Vec<u32> = match c.place {
                    Place::Own => Vec::new(),
                    Place::Of(_) => self.frames[..f as usize].iter()
                        .filter_map(|g| g.chart.as_ref())
                        .filter(|g| g.o == c.o)
                        .flat_map(|g| g.axes())
                        .collect(),
                };
                let plane = Arg::Ent(EntRef::frame(f as usize));
                let mut rows = Vec::new();
                for ax in [c.u, c.v] {
                    if c.stands_at.is_none() && !through.contains(&ax) {
                        rows.push(row(CKind::PlaneAxis, vec![plane.clone(), axis(ax)]));
                    }
                }
                if c.square {
                    rows.push(row(CKind::Perpendicular3, vec![axis(c.u), axis(c.v)]));
                }
                for (ax, along) in [c.u, c.v].into_iter().zip(c.along) {
                    let Some(Along { line, at }) = along else { continue };
                    rows.push(row(CKind::Parallel3, vec![axis(ax), Arg::Ent(EntRef::line(line as usize)),
                                                        Arg::Str("same".to_string())]));
                    rows.push(row(CKind::PointOnAxis, vec![Arg::Ent(EntRef::point(at as usize)), axis(ax)]));
                }
                if let Some(at) = c.stands_at {
                    rows.push(row(CKind::Coincident3, vec![Arg::Ent(EntRef::point(c.origin as usize)),
                                                          Arg::Ent(EntRef::point(at as usize))]));
                }
                rows
            }
            Owner::Normal(f) => {
                let Some(c) = self.frames.get(f as usize).and_then(|g| g.chart.as_ref()) else {
                    return Vec::new();
                };
                let Some(n) = c.normal else { return Vec::new() };
                vec![
                    row(CKind::Perpendicular3, vec![axis(n), axis(c.u)]),
                    row(CKind::PerpendicularAbout,
                        vec![axis(c.v), axis(n), axis(c.u), Arg::Str("ccw".into())]),
                    row(CKind::PlaneAxis, vec![Arg::Ent(EntRef::frame(f as usize)), axis(n)]),
                ]
            }
            Owner::Lift(p) => {
                let p = p as usize;
                if p >= self.points.len() || self.lift_of(p).is_none() {
                    return Vec::new();
                }
                let pt = &self.points[p];
                let (kind, frame) = match (pt.z, pt.chart, pt.plane) {
                    (Some(_), Some(f), _) => (CKind::ChartLift, f),
                    (None, _, Some(v)) => (CKind::Lift, v),
                    _ => return Vec::new(),
                };
                vec![row(kind, vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::frame(frame as usize))])]
            }
            Owner::Twin(p, t) => {
                let (p, t) = (p as usize, t as usize);
                if !self.twins.get(&p).is_some_and(|ts| ts.contains(&t)) {
                    return Vec::new();
                }
                let mut rows = Vec::new();
                if let Some(plane) = self.plane_of(p) {
                    rows.push(row(CKind::PointOnPlane,
                                  vec![Arg::Ent(EntRef::point(t)), Arg::Ent(EntRef::frame(plane))]));
                    if let Ok(fold) = Constraint::project(self, EntRef::point(p), EntRef::point(t)) {
                        rows.push(fold);
                    }
                }
                // the point on its twin's plane, but where the plane already holds it: a row
                // saying so again would be a dependency the structural count cannot see (#88)
                if let Some(plane) = self.plane_of(t) {
                    let on = |c: &Constraint| c.kind == CKind::PointOnPlane
                        && c.args[0].ent() == EntRef::point(p) && c.args[1].ent() == EntRef::frame(plane);
                    let held = self.constraints.iter()
                        .any(|c| on(c) && c.origin != Origin::Intrinsic(owner));
                    if !held && !self.built_through(p, plane) {
                        rows.push(row(CKind::PointOnPlane,
                                      vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::frame(plane))]));
                    }
                }
                rows
            }
            Owner::Round(e) => {
                let round = match e.kind {
                    EntKind::Arc => self.arcs.get(e.i()).and_then(|a| a.round.as_ref()),
                    EntKind::Circle => self.circles.get(e.i()).and_then(|c| c.round.as_ref()),
                    _ => None,
                };
                let Some(r) = round else { return Vec::new() };
                let Some(c) = self.frames.get(r.plane as usize).and_then(|f| f.chart.as_ref()) else {
                    return Vec::new();
                };
                let mut rows: Vec<Constraint> = r.ties.iter().map(|&[p, q]| {
                    row(CKind::Coincident3, vec![Arg::Ent(EntRef::point(p as usize)),
                                                 Arg::Ent(EntRef::point(q as usize))])
                }).collect();
                let square = |a: Arg, b: Arg| row(CKind::Perpendicular3, vec![a, b]);
                match r.normal {
                    None => rows.push(square(axis(c.u), axis(c.v))),
                    Some((n, g)) => rows.extend([
                        square(axis(c.u), Arg::Ent(n)),
                        square(axis(c.v), Arg::Ent(n)),
                        square(axis(c.u), axis(c.v)),
                        square(axis(c.u), axis(g)),
                    ]),
                }
                rows
            }
            Owner::Developed(i) => {
                let Some(cv) = self.curves.get(i as usize) else { return Vec::new() };
                match &self.curve_defs[cv.def as usize].body {
                    CurveBody::Developed(d) if d.which == 0 => vec![row(CKind::DevelopedLength,
                        vec![Arg::Ent(EntRef::new(EntKind::Curve, i as usize))])],
                    _ => Vec::new(),
                }
            }
        }
    }

    /// Whether `e` is a mirrored instance's image (#578), whose rows are its representative's.
    fn imaged(&self, e: EntRef) -> bool {
        self.mirrors.iter().any(|m| m.copy == e)
    }

    /// **Replace `owner`'s rows with `rows_of(owner)`.**  A row it still implies stays where it
    /// stands, its id kept; one it no longer implies gives its place and id to a new row of its
    /// kind, else goes; the rest are added after everything, so a constructor deriving as it
    /// builds adds its rows in the order it builds.
    pub fn derive(&mut self, owner: Owner) {
        let mut want = self.rows_of(owner);
        for w in &mut want {
            w.kind = self.settled_kind(w);
            w.origin = Origin::Intrinsic(owner);
        }
        let have: Vec<u32> = self.constraints.iter()
            .filter(|c| c.origin == Origin::Intrinsic(owner) && c.along.is_none())
            .map(|c| c.id)
            .collect();
        let mut kept = vec![false; have.len()];
        let mut fresh = Vec::new();
        for w in want {
            // the twin a statement's planes feed is one statement (`CKind::attitude_twin`)
            let same = have.iter().enumerate().position(|(k, &id)| {
                !kept[k] && self.constraint(id).is_some_and(|c| {
                    c.kind.attitude_twin(false) == w.kind.attitude_twin(false) && c.args == w.args
                })
            });
            match same {
                Some(k) => kept[k] = true,
                None => fresh.push(w),
            }
        }
        let mut stale: Vec<u32> =
            have.iter().zip(&kept).filter(|(_, &k)| !k).map(|(&id, _)| id).collect();
        let mut added = Vec::new();
        for w in fresh {
            match stale.iter().position(|&id| self.constraint(id).is_some_and(|c| c.kind == w.kind)) {
                Some(k) => {
                    let id = stale.remove(k);
                    self.restate(id, w.args);
                }
                None => added.push(w),
            }
        }
        for w in added {
            self.add_quiet(w);
        }
        for id in stale {
            let gone = self.constraint(id).cloned();
            self.remove(id);
            // and its derivatives under the uses that move what it reads (§6.21)
            if let Some(g) = gone {
                let twins: Vec<u32> = self.constraints.iter()
                    .filter(|c| c.origin == g.origin && c.along.is_some() && c.kind == g.kind
                        && c.args == g.args)
                    .map(|c| c.id)
                    .collect();
                for t in twins {
                    self.remove(t);
                }
            }
        }
    }

    /// Row `id` restated over `args` where it stands — its id and its place in the list kept —
    /// and its derivatives with it; the hidden points and the axes' places it now reads minted,
    /// as `add_quiet` mints them, and an axis's place nothing reads now held again.
    fn restate(&mut self, id: u32, args: Vec<Arg>) {
        let Some(at) = self.index_of(id) else { return };
        let old = self.constraints[at].clone();
        let placed: Vec<usize> = old.axes_placed_by().collect();
        for c in self.constraints.iter_mut() {
            if c.id == id || c.origin == old.origin && c.along.is_some() && c.kind == old.kind
                && c.args == old.args
            {
                c.args = args.clone();
            }
        }
        let c = self.constraints[at].clone();
        for p in c.lifted_points(self) {
            self.lift_point(p);
        }
        for r in c.axes_placed_by() {
            if r < self.axes.len() {
                self.place_axis(r);
            }
        }
        for r in placed {
            if !self.constraints.iter().any(|c| c.axes_placed_by().any(|s| s == r)) {
                self.unplace_axis(r);
            }
        }
    }

    /// **Every owner's rows**, in a fixed order — axes, arcs, frames, normals, lifts, twins,
    /// rounds, developments — so the ids and the block order are deterministic: what a copy
    /// calls once it is whole (`graft`, the JSON reader), after the constructors it copied
    /// through have derived what they make, so a stated row keeps the id the source's order gave
    /// it.
    pub fn derive_all(&mut self) {
        let owners = self.owners();
        for o in owners {
            self.derive(o);
        }
        // an owner a copy brought but nothing implies now: a row of it goes
        let owners: std::collections::BTreeSet<Owner> = self.owners().into_iter().collect();
        let orphans: Vec<u32> = self.constraints.iter()
            .filter(|c| matches!(c.origin, Origin::Intrinsic(o) if !owners.contains(&o)))
            .map(|c| c.id)
            .collect();
        for id in orphans {
            self.remove(id);
        }
    }

    /// Every owner the model has, in `derive_all`'s order.
    pub fn owners(&self) -> Vec<Owner> {
        let n = |k: usize| 0..k as u32;
        let mut out: Vec<Owner> = n(self.axes.len()).map(Owner::Axis).collect();
        out.extend(n(self.arcs.len()).map(Owner::Arc));
        out.extend(n(self.frames.len()).map(Owner::Frame));
        out.extend(n(self.frames.len()).map(Owner::Normal));
        out.extend(self.lifts.iter().map(|l| Owner::Lift(l.point)));
        out.extend(self.twins.iter()
            .flat_map(|(&p, ts)| ts.iter().map(move |&t| Owner::Twin(p as u32, t as u32))));
        out.extend((0..self.arcs.len()).map(|i| Owner::Round(EntRef::arc(i))));
        out.extend((0..self.circles.len()).map(|i| Owner::Round(EntRef::circle(i))));
        out.extend(n(self.curves.len()).map(Owner::Developed));
        out
    }

    /// **Whether plane `plane` is built along a line point `p` is on** (`Chart::along`): an end of
    /// it, or stated on it.
    pub fn built_through(&self, p: usize, plane: usize) -> bool {
        let Some(c) = self.frames[plane].chart.as_ref() else { return false };
        let on = |l: usize| {
            let line = &self.lines[l];
            line.p1 as usize == p || line.p2 as usize == p
                || self.constraints.iter().any(|c| {
                    c.acts() && c.kind == CKind::PointOnLine
                        && c.args[0].ent() == EntRef::point(p) && c.args[1].ent() == EntRef::line(l)
                })
        };
        c.along.iter().flatten().any(|a| on(a.line as usize))
    }

    /// The axes held along drawn lines (`Chart::along`), each with its line.
    pub fn axes_along(&self) -> Vec<(usize, usize)> {
        self.frames.iter().filter_map(|f| f.chart.as_ref())
            .flat_map(|c| [c.u, c.v].into_iter().zip(c.along))
            .filter_map(|(ax, a)| a.map(|a| (ax as usize, a.line as usize)))
            .collect()
    }

    /// **The point in space point `p` is drawn for** (#363 S6): a round's own point tied to the
    /// point the document wrote (`RoundIn::ties`), or `None`.
    pub fn stands_for(&self, p: u32) -> Option<u32> {
        let rounds = self.arcs.iter().filter_map(|a| a.round.as_ref())
            .chain(self.circles.iter().filter_map(|c| c.round.as_ref()));
        rounds.flat_map(|r| r.ties.iter()).find(|t| t[0] == p).map(|t| t[1])
    }

    /// The gauges circles about points in space hold their planes' turn by: each gauge axis with
    /// the `u` and `v` of the plane it gauges (`RoundIn::normal`).
    pub fn round_gauges(&self) -> Vec<(u32, [u32; 2])> {
        self.circles.iter().filter_map(|c| c.round.as_ref())
            .filter_map(|r| {
                let (_, g) = r.normal?;
                let c = self.frames.get(r.plane as usize)?.chart.as_ref()?;
                Some((g, [c.u, c.v]))
            })
            .collect()
    }
}
