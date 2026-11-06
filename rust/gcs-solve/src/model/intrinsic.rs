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
        let point = |p: u32| renumbered(map, EntKind::Point, p);
        Some(RoundIn {
            plane: renumbered(map, EntKind::Frame, *plane)?,
            ties: ties.iter().map(|&[a, b]| Some([point(a)?, point(b)?])).collect::<Option<_>>()?,
            normal: match normal {
                Some((n, g)) => Some((map(*n)?, renumbered(map, EntKind::Axis, *g)?)),
                None => None,
            },
        })
    }
}

impl Along {
    /// The same line and point over the entities `map` renumbers, or `None` where one did not
    /// come.
    pub fn remapped(&self, map: &Remap) -> Option<Along> {
        let Along { line, at } = *self;
        Some(Along { line: renumbered(map, EntKind::Line, line)?,
                     at: renumbered(map, EntKind::Point, at)? })
    }
}

/// Whether `c` is `of` stated again as its derivative under a use (§6.21, `twin_intrinsics`).
fn derives_from(c: &Constraint, of: &Constraint) -> bool {
    c.origin == of.origin && c.along.is_some() && c.kind == of.kind && c.args == of.args
}

/// The same row whatever twin its planes feed (`CKind::attitude_twin`): what a derived row is
/// compared by.
pub fn same_row(a: &Constraint, b: &Constraint) -> bool {
    a.kind.attitude_twin(false) == b.kind.attitude_twin(false) && a.args == b.args
}

impl Sketch {
    /// **The rows `owner` implies, as the model now says** — the only place an intrinsic row is
    /// made.  Empty for an owner that implies none now (an image's axis, a frame with no chart).
    pub fn rows_of(&self, owner: Owner) -> Vec<Constraint> {
        let axis = |a: u32| EntRef::axis(a as usize);
        let ent = |e: EntRef| Arg::Ent(e);
        let point = |p: u32| Arg::Ent(EntRef::point(p as usize));
        let square = |a: EntRef, b: EntRef| Constraint::two_line(CKind::Perpendicular3, a, b);
        match owner {
            Owner::Axis(i) => {
                let e = axis(i);
                if i as usize >= self.axes.len() || self.imaged(e) {
                    return Vec::new();
                }
                let mut rows = vec![Constraint::new(CKind::AxisUnit, vec![ent(e)])];
                if self.axes[i as usize].placed {
                    rows.push(Constraint::new(CKind::AxisFoot, vec![ent(e)]));
                }
                rows
            }
            Owner::Arc(i) => {
                let e = EntRef::arc(i as usize);
                if i as usize >= self.arcs.len() || self.imaged(e) || self.turn_of(e).is_some() {
                    return Vec::new();
                }
                let a = &self.arcs[i as usize];
                [a.start, a.end].map(|p| Constraint::point_on_circle(EntRef::point(p as usize), e))
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
                let plane = ent(EntRef::frame(f as usize));
                let mut rows = Vec::new();
                for ax in [c.u, c.v] {
                    if c.stands_at.is_none() && !through.contains(&ax) {
                        rows.push(Constraint::new(CKind::PlaneAxis,
                                                  vec![plane.clone(), ent(axis(ax))]));
                    }
                }
                if c.square {
                    rows.push(square(axis(c.u), axis(c.v)));
                }
                for (ax, along) in [c.u, c.v].into_iter().zip(c.along) {
                    let Some(Along { line, at }) = along else { continue };
                    rows.push(Constraint::new(CKind::Parallel3, vec![
                        ent(axis(ax)), ent(EntRef::line(line as usize)), Arg::Str("same".into())]));
                    rows.push(Constraint::new(CKind::PointOnAxis, vec![point(at), ent(axis(ax))]));
                }
                if let Some(at) = c.stands_at {
                    rows.push(Constraint::new(CKind::Coincident3,
                                              vec![point(c.origin), point(at)]));
                }
                rows
            }
            Owner::Normal(f) => {
                let Some(c) = self.frames.get(f as usize).and_then(|g| g.chart.as_ref()) else {
                    return Vec::new();
                };
                let Some(n) = c.normal else { return Vec::new() };
                vec![
                    square(axis(n), axis(c.u)),
                    Constraint::new(CKind::PerpendicularAbout, vec![
                        ent(axis(c.v)), ent(axis(n)), ent(axis(c.u)), Arg::Str("ccw".into())]),
                    Constraint::new(CKind::PlaneAxis,
                                    vec![ent(EntRef::frame(f as usize)), ent(axis(n))]),
                ]
            }
            Owner::Lift(p) => {
                if p as usize >= self.points.len() || self.lift_of(p as usize).is_none() {
                    return Vec::new();
                }
                let pt = &self.points[p as usize];
                let (kind, frame) = match (pt.z, pt.chart, pt.plane) {
                    (Some(_), Some(f), _) => (CKind::ChartLift, f),
                    (None, _, Some(v)) => (CKind::Lift, v),
                    _ => return Vec::new(),
                };
                vec![Constraint::new(kind, vec![point(p), ent(EntRef::frame(frame as usize))])]
            }
            Owner::Twin(p, t) => {
                if !self.twins.get(&(p as usize)).is_some_and(|ts| ts.contains(&(t as usize))) {
                    return Vec::new();
                }
                let on = |q: u32, plane: usize| Constraint::new(CKind::PointOnPlane,
                    vec![point(q), ent(EntRef::frame(plane))]);
                let mut rows = Vec::new();
                if let Some(plane) = self.plane_of(p as usize) {
                    rows.push(on(t, plane));
                    let (a, b) = (EntRef::point(p as usize), EntRef::point(t as usize));
                    if let Ok(fold) = Constraint::project(self, a, b) {
                        rows.push(fold);
                    }
                }
                // the point on its twin's plane, but where the plane already holds it: a row
                // saying so again would be a dependency the structural count cannot see (#88)
                if let Some(plane) = self.plane_of(t as usize) {
                    let row = on(p, plane);
                    let held = self.constraints.iter()
                        .any(|c| c.kind == row.kind && c.args == row.args
                            && c.origin != Origin::Intrinsic(owner));
                    if !held && !self.built_through(p as usize, plane) {
                        rows.push(row);
                    }
                }
                rows
            }
            Owner::Round(e) => {
                let Some(r) = self.round(e) else { return Vec::new() };
                let plane = self.frames.get(r.plane as usize).and_then(|f| f.chart.as_ref());
                let Some(c) = plane else { return Vec::new() };
                let mut rows: Vec<Constraint> = r.ties.iter()
                    .map(|&[p, q]| Constraint::new(CKind::Coincident3, vec![point(p), point(q)]))
                    .collect();
                match r.normal {
                    None => rows.push(square(axis(c.u), axis(c.v))),
                    Some((n, g)) => rows.extend([
                        square(axis(c.u), n),
                        square(axis(c.v), n),
                        square(axis(c.u), axis(c.v)),
                        square(axis(c.u), axis(g)),
                    ]),
                }
                rows
            }
            Owner::Developed(i) => {
                let Some(cv) = self.curves.get(i as usize) else { return Vec::new() };
                match &self.curve_defs[cv.def as usize].body {
                    CurveBody::Developed(d) if d.which == 0 => {
                        let curve = ent(EntRef::new(EntKind::Curve, i as usize));
                        vec![Constraint::new(CKind::DevelopedLength, vec![curve])]
                    }
                    _ => Vec::new(),
                }
            }
        }
    }

    /// Whether `e` is a mirrored instance's image (#578), whose rows are its representative's.
    pub fn imaged(&self, e: EntRef) -> bool {
        self.mirror_of_entity(e).is_some()
    }

    /// An arc's or a circle's plane in space (`RoundIn`), where it is drawn in one.
    pub fn round(&self, e: EntRef) -> Option<&RoundIn> {
        match e.kind {
            EntKind::Arc => self.arcs.get(e.i())?.round.as_ref(),
            EntKind::Circle => self.circles.get(e.i())?.round.as_ref(),
            _ => None,
        }
    }

    /// Where an arc's or a circle's plane in space is kept, to set it.
    pub fn round_mut(&mut self, e: EntRef) -> Option<&mut Option<RoundIn>> {
        match e.kind {
            EntKind::Arc => Some(&mut self.arcs.get_mut(e.i())?.round),
            EntKind::Circle => Some(&mut self.circles.get_mut(e.i())?.round),
            _ => None,
        }
    }

    /// Every arc and circle, the arcs first: the entities a `RoundIn` may be of.
    fn rounded(&self) -> impl Iterator<Item = EntRef> + '_ {
        (0..self.arcs.len()).map(EntRef::arc).chain((0..self.circles.len()).map(EntRef::circle))
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
        // where its rows stand: nothing moves while they are matched
        let have: Vec<usize> = (0..self.constraints.len())
            .filter(|&i| self.constraints[i].origin == Origin::Intrinsic(owner)
                && self.constraints[i].along.is_none())
            .collect();
        let mut kept = vec![false; have.len()];
        let mut fresh = Vec::new();
        for w in want {
            match (0..have.len()).find(|&k| !kept[k] && same_row(&self.constraints[have[k]], &w)) {
                Some(k) => kept[k] = true,
                None => fresh.push(w),
            }
        }
        let mut stale: Vec<u32> = (0..have.len()).filter(|&k| !kept[k])
            .map(|k| self.constraints[have[k]].id).collect();
        let mut added = Vec::new();
        for w in fresh {
            let like = |id: &u32| self.constraint(*id).is_some_and(|c| c.kind == w.kind);
            match stale.iter().position(like) {
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
            let Some(gone) = self.constraint(id).cloned() else { continue };
            self.remove(id);
            // and its derivatives under the uses that move what it reads (§6.21)
            let twins: Vec<u32> = self.constraints.iter()
                .filter(|c| derives_from(c, &gone))
                .map(|c| c.id)
                .collect();
            for t in twins {
                self.remove(t);
            }
        }
    }

    /// Row `id` restated over `args` where it stands — its id and its place in the list kept —
    /// and its derivatives with it; the hidden points and the axes' places it now reads minted,
    /// as `add_quiet` mints them, and an axis's place nothing reads now held again.
    fn restate(&mut self, id: u32, args: Vec<Arg>) {
        let Some(at) = self.index_of(id) else { return };
        let old = self.constraints[at].clone();
        for c in self.constraints.iter_mut() {
            if c.id == id || derives_from(c, &old) {
                c.args = args.clone();
            }
        }
        let c = &self.constraints[at];
        let (lifted, placed): (Vec<usize>, Vec<usize>) =
            (c.lifted_points(self), c.axes_placed_by().collect());
        for p in lifted {
            self.lift_point(p);
        }
        for r in placed {
            if r < self.axes.len() {
                self.place_axis(r);
            }
        }
        self.release_axes(old.axes_placed_by().collect());
    }

    /// **Every owner's rows**, in a fixed order — axes, arcs, frames, normals, lifts, twins,
    /// rounds, developments — so the ids and the block order are deterministic: what a copy
    /// calls once it is whole (`graft`, the JSON reader), after the constructors it copied
    /// through have derived what they make, so a stated row keeps the id the source's order gave
    /// it.
    pub fn derive_all(&mut self) {
        for o in self.owners() {
            self.derive(o);
        }
        // an owner a copy brought but nothing implies now: a row of it goes — asked again, since
        // deriving mints the lifts the rows read (`add_quiet`), which are owners too
        let owners: std::collections::BTreeSet<Owner> = self.owners().into_iter().collect();
        let orphans: Vec<u32> = self.constraints.iter()
            .filter(|c| matches!(c.origin, Origin::Intrinsic(o) if !owners.contains(&o)))
            .map(|c| c.id)
            .collect();
        for id in orphans {
            self.remove(id);
        }
    }

    /// Every owner the model has, in `derive_all`'s order: each entity whose kind owns rows, and
    /// the normals, lifts, twins, rounds and developments it has.
    pub fn owners(&self) -> Vec<Owner> {
        let n = |k: usize| 0..k as u32;
        let mut out: Vec<Owner> = n(self.axes.len()).map(Owner::Axis).collect();
        out.extend(n(self.arcs.len()).map(Owner::Arc));
        out.extend(n(self.frames.len()).map(Owner::Frame));
        out.extend(n(self.frames.len())
            .filter(|&f| self.frames[f as usize].chart.as_ref().is_some_and(|c| c.normal.is_some()))
            .map(Owner::Normal));
        out.extend(self.lifts.iter().map(|l| Owner::Lift(l.point)));
        out.extend(self.twins.iter()
            .flat_map(|(&p, ts)| ts.iter().map(move |&t| Owner::Twin(p as u32, t as u32))));
        out.extend(self.rounded().filter(|&e| self.round(e).is_some()).map(Owner::Round));
        out.extend(n(self.curves.len())
            .filter(|&i| matches!(self.curve_defs[self.curves[i as usize].def as usize].body,
                                  CurveBody::Developed(_)))
            .map(Owner::Developed));
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
        self.rounded().filter_map(|e| self.round(e))
            .flat_map(|r| r.ties.iter()).find(|t| t[0] == p).map(|t| t[1])
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
