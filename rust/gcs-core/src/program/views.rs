//! What a solve says about planes that elaboration could not: two planes of a projection that
//! came out parallel, two lines meant to have a common perpendicular that came out parallel
//! (E065), and two planes that lie on one another (W113, `docs/planes-plan.md`).

#[allow(unused_imports)]
use crate::fmath::Det;
use super::{Code, Diag, SourceMap};
use crate::constraints::CKind;
use crate::model::{EntKind, EntRef, Sketch};
use std::collections::BTreeSet;

/// Whether every one of `ps` is held.
pub(super) fn held(sk: &Sketch, ps: &[u32]) -> bool {
    ps.iter().all(|&q| sk.params[q as usize].fixed)
}

/// A plane this close to another, in turn and in where it stands, lies on it.
const ON_PLANE: f64 = 1e-9;

/// E065 for every statement a solve left degenerate: a projection whose two planes came out
/// parallel, a skew distance whose two lines did.
pub(crate) fn degenerate(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut out = Vec::new();
    for c in &sk.constraints {
        let message = match c.kind {
            CKind::ProjectSolved => {
                let (a, b) = (c.args[2].ent().i(), c.args[3].ent().i());
                let (na, nb) = (sk.basis(a).normal(), sk.basis(b).normal());
                let parallel =
                    crate::space::norm(crate::space::cross(na, nb)) <= crate::plane::PARALLEL_TOL;
                // a point drawn in two planes the solve turned parallel: said where the point is
                // declared, as it is of two planes placed parallel before the solve (§6.7)
                let (p, t) = (c.args[0].ent().i(), c.args[1].ent().i());
                if parallel && c.intrinsic && sk.twins.get(&p).is_some_and(|ts| ts.contains(&t)) {
                    out.extend(super::planes::twin_parallel(sk, map, p, a, b));
                    continue;
                }
                parallel.then(|| format!(
                        "`{}` and `{}` came out parallel, so no fold line relates them and the \
                         projection says nothing",
                        sk.plane_name(a), sk.plane_name(b)
                    ))
            }
            CKind::LineLine3 => {
                let dir = |i: usize| {
                    let l = &sk.lines[c.args[i].ent().i()];
                    crate::space::sub(sk.lifted(l.p2 as usize), sk.lifted(l.p1 as usize))
                };
                let (a, b) = (dir(0), dir(1));
                let m = crate::space::norm(crate::space::cross(a, b));
                (m <= crate::plane::PARALLEL_TOL * crate::space::norm(a) * crate::space::norm(b))
                    .then(|| "the two lines came out parallel in space, and parallel lines have \
                              no common perpendicular to measure"
                        .to_string())
            }
            _ => None,
        };
        if let Some(message) = message {
            let site = map.site_of_constraint(c.id);
            out.push(Diag::new(
                Code::E065,
                site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),
                message,
            ));
        }
    }
    out
}

/// **A plane's axes pass through its origin** (#84), and where its axes are held, so is that: two
/// held axes hold the origin where they meet, and a plane held where its held axes are not is
/// refused.  E067 for axes that do not meet (or run alike), and for a plane standing off one.
/// Held numbers make no equation (`System`'s intrinsic rule), so this is said here or nowhere.
pub(crate) fn origins_on_axes(sk: &mut Sketch, map: &SourceMap) -> Vec<Diag> {
    use crate::space::{add, cross, dot, lerp, norm, normalised, scale, sub};
    let line = |sk: &Sketch, r: usize| {
        let ax = &sk.axes[r];
        let whole = held(sk, &ax.d) && held(sk, &ax.a);
        let d = normalised(ax.d.map(|q| sk.params[q as usize].value));
        (whole, ax.a.map(|q| sk.params[q as usize].value), d)
    };
    let tol = ON_PLANE * sk.extent().max(1.0);
    let mut out = Vec::new();
    for p in sk.charted() {
        let pl = &sk.frames[p];
        let (o, sides) = (pl.o, [(pl.u as usize, "u"), (pl.v as usize, "v")]);
        let lines = sides.map(|(r, _)| line(sk, r));
        let [(hu, au, du), (hv, av, dv)] = lines;
        let mut refuse = |message: String| {
            let site = map.site_of(EntRef::frame(p));
            out.push(Diag::new(Code::E067, site.map(|s| s.span).unwrap_or_default(),
                            site.map(|s| s.stmt), message));
        };
        let name = sk.plane_name(p);
        if held(sk, &o) {
            let at = o.map(|q| sk.params[q as usize].value);
            for ((h, a, d), (_, k)) in lines.into_iter().zip(sides) {
                let (true, Some(d)) = (h, d) else { continue };
                if norm(cross(sub(at, a), d)) > tol {
                    refuse(format!(
                        "`{name}` is held where its axis `{name}.{k}` is not, and a plane's axes \
                         pass through its origin: a plane parallel to another says so with \
                         `parallel` (`{name}.{k} parallel ….{k}`)"
                    ));
                }
            }
            continue;
        }
        let (true, true, Some(du), Some(dv)) = (hu, hv, du, dv) else { continue };
        // the two lines' nearest points: a + s·d for each
        let n = cross(du, dv);
        if norm(n) <= crate::plane::PARALLEL_TOL {
            refuse(format!("the axes of `{name}` run alike, and a plane needs two directions"));
            continue;
        }
        let w = sub(av, au);
        let nn = dot(n, n);
        let s = dot(cross(w, dv), n) / nn;
        let t = dot(cross(w, du), n) / nn;
        let (pu, pv) = (add(au, scale(du, s)), add(av, scale(dv, t)));
        if norm(sub(pu, pv)) > tol {
            refuse(format!(
                "the axes of `{name}` do not meet, and a plane's axes pass through its origin"
            ));
            continue;
        }
        let at = lerp(pu, pv, 0.5);
        for k in 0..3 {
            let q = &mut sk.params[o[k] as usize];
            q.value = at[k];
            q.fixed = true;
        }
    }
    out
}

/// **An instance's own frame starts where the frame around its call stands** (#548): each
/// instance read as a frame (`flatten::frame_instance`), in build order so one nested in another
/// reads it seeded, along the innermost frame with axes that it rides — a mate outside then
/// starts from the pose its call puts it in, and a frame no `in` encloses starts as the world.
pub(crate) fn instance_frames(sk: &mut Sketch, res: &super::resolve::Resolver) {
    let own = |key: &str, e: &EntRef| {
        e.kind == EntKind::Frame && crate::flatten::frame_instance(key).is_some()
    };
    let mut own: Vec<usize> =
        res.of.iter().filter(|(key, e)| own(key, e)).map(|(_, e)| e.i()).collect();
    own.sort_unstable();
    for f in own {
        let around = sk.frames[f].rides.iter().rev().map(|&g| g as usize)
            .find(|&g| sk.frames[g].chart.is_some());
        if let Some(g) = around {
            sk.seed_at_frame(f, g);
        }
    }
}

/// Each plane's axes stood through its origin again, the first plane over an axis taking it, as
/// the plane's build stood them (`Sketch::push_plane`) — once `fix` has said where the planes
/// stand and which way their axes run, so a plane held off the world origin carries its axes
/// with it and needs no `hint` saying where it is.  An axis whose place a `fix` holds stays.
pub(crate) fn stand_axes(sk: &mut Sketch) {
    let mut stood = vec![false; sk.axes.len()];
    for p in sk.charted() {
        let pl = &sk.frames[p];
        let o = pl.o.map(|q| sk.params[q as usize].value);
        for r in [pl.u as usize, pl.v as usize] {
            let free = sk.axes[r].placed && !sk.axes[r].a.iter().any(|&q| sk.params[q as usize].fixed);
            if free && !std::mem::replace(&mut stood[r], true) {
                sk.stand_axis_through(r, o);
            }
        }
    }
}

/// W113 for two planes that lie on one another — turned alike up to a turn in themselves and
/// standing in one place — where a relation reads points drawn in each: one plane in space, read
/// twice, and the relation is read in space where the plane would do.  Permitted (a part's plane
/// turned within the front is the common case, and two parts drawn on one plane relate nothing),
/// and said once, at the later plane's declaration.
pub(crate) fn coplanar(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    use std::collections::BTreeSet;
    // the planes each relation reads drawn points of — a plane's own origin is where the plane
    // stands, not drawing in it
    let mut pairs: BTreeSet<(usize, usize)> = BTreeSet::new();
    for c in sk.constraints.iter().filter(|c| c.acts() && !c.intrinsic) {
        let mut planes: BTreeSet<usize> = BTreeSet::new();
        for e in c.entities() {
            let mut pts = Vec::new();
            points(sk, e, &mut pts, 0);
            planes.extend(pts.iter().filter(|&&p| sk.plane_of_origin(p).is_none())
                .filter_map(|&p| sk.plane_of(p)));
        }
        for &i in &planes {
            for &j in planes.range(i + 1..) {
                pairs.insert((i, j));
            }
        }
    }
    let mut out = Vec::new();
    let mut said: BTreeSet<usize> = BTreeSet::new();
    let tol = ON_PLANE * sk.extent().max(1.0);
    for &(i, j) in &pairs {
        if !sk.basis(i).coplanar(&sk.basis(j), tol) || !said.insert(j) {
            continue;
        }
        let Some(site) = map.site_of(EntRef::frame(j)) else { continue };
        out.push(Diag::within(
            Code::W113,
            site.span,
            site.stmt,
            format!(
                "`{}` lies on `{}`, and a relation reads points drawn in each: one plane in \
                 space, so the relation is read in space where one plane would read it on the \
                 plane",
                sk.plane_name(j),
                sk.plane_name(i)
            ),
        ));
    }
    out
}

/// The points an entity stands on, its axes and planes apart.
fn points(sk: &Sketch, e: EntRef, out: &mut Vec<usize>, depth: u32) {
    match e.kind {
        EntKind::Point => out.push(e.i()),
        EntKind::Frame | EntKind::Axis => {}
        _ if depth < 4 => {
            for k in sk.children(e) {
                points(sk, k, out, depth + 1);
            }
        }
        _ => {}
    }
}

/// **Where the planes stand, before the drawing is solved** (`docs/planes-plan.md`): a seed
/// that reads a place in another plane is read through that plane's pose, and a plane whose
/// axes or place the document solves for starts where its seeds put it — a fold line at the
/// world origin, say, when it stands on a line a hundred millimetres off.  So the statements
/// that say where the planes are — about axes and planes themselves, and about points of planes
/// already placed — are solved first, alone, round by round: a statement is taken once every
/// point it reads stands in a placed plane (or is held, or is a plane's own origin), and an axis
/// or a plane is placed once the statements taken determine it.  The drawing in a plane never
/// places it: its seeds are what is to be read through it.  Nothing else moves; the hidden
/// points are put back where their points now stand.  What nothing places keeps its seed.
///
/// Which planes stand turned as the statements say, and not merely as they were seeded: each
/// plane both of whose axes are held or placed here.
pub(crate) fn place(sk: &mut Sketch) -> Vec<bool> {
    align(sk);
    // a plane's normal is read off its axes, never solved for here: placed with them, and turned
    // to them once they are
    sk.seed_normals();
    let mut placed = Placed::held(sk);
    if placed.axes.iter().all(|&b| b) && placed.planes.iter().all(|&b| b) {
        return placed.turned(sk);
    }
    let reads = statements(sk);
    let mut taken: BTreeSet<u32> = BTreeSet::new();
    let along = along(sk);
    for _ in 0..sk.frames.len() + sk.axes.len() + 1 {
        let newly = placed.carried(sk, &along);
        let before = taken.len();
        for r in &reads {
            let unplaced = r.axes.iter().any(|&k| !placed.axes[k])
                || r.planes.iter().any(|&p| !placed.planes[p]);
            let ready = r.through.iter().all(|&p| placed.planes[p]);
            if unplaced && ready {
                taken.insert(r.id);
            }
        }
        if taken.len() == before {
            if newly {
                continue;
            }
            break;
        }
        solve_taken(sk, &reads, &taken, &mut placed);
    }
    sk.seed_normals();
    relift(sk);
    placed.turned(sk)
}

/// Which axes and which planes `place` has placed so far.
struct Placed {
    axes: Vec<bool>,
    planes: Vec<bool>,
}

impl Placed {
    /// What stands before anything is solved: the axes held (a plane's normal with both its
    /// plane's axes) and the planes fixed.
    fn held(sk: &Sketch) -> Placed {
        let mut axes: Vec<bool> = (0..sk.axes.len()).map(|r| held(sk, &sk.axes[r].d)).collect();
        for pl in sk.frames.iter().filter_map(|f| f.chart.as_ref()) {
            if let Some(n) = pl.normal {
                axes[n as usize] = axes[pl.u as usize] && axes[pl.v as usize];
            }
        }
        let planes = (0..sk.frames.len()).map(|p| sk.plane_fixed(p)).collect();
        Placed { axes, planes }
    }

    /// Which planes stand turned as the statements say: each plane both of whose axes are placed.
    fn turned(&self, sk: &Sketch) -> Vec<bool> {
        sk.frames.iter().map(|f| f.chart.as_ref()
            .is_none_or(|pl| self.axes[pl.u as usize] && self.axes[pl.v as usize])).collect()
    }

    /// What a round places with nothing solved: an axis held along a line whose ends are placed
    /// is placed with them — turned to the line here, sense and all, and held so the solve below
    /// cannot turn it round — a plane held at a place over placed axes, and a normal over placed
    /// axes. Whether anything was.
    fn carried(&mut self, sk: &mut Sketch, along: &[(usize, usize)]) -> bool {
        let mut newly = false;
        for &(r, l) in along {
            let ends = [sk.lines[l].p1 as usize, sk.lines[l].p2 as usize];
            let fine = ends.iter().all(|&p| {
                sk.plane_of_origin(p).is_some()
                    || sk.plane_of(p).is_some_and(|pl| self.planes[pl])
                    || sk.point_held(p)
            });
            if fine && !self.axes[r] && sk.turn_axis_along(r, l) {
                self.axes[r] = true;
                newly = true;
            }
        }
        for p in sk.charted() {
            let pl = &sk.frames[p];
            if !self.planes[p] && held(sk, &pl.o) && self.axes[pl.u as usize] && self.axes[pl.v as usize] {
                self.planes[p] = true;
                newly = true;
            }
            if let Some(n) = pl.normal.map(|n| n as usize) {
                if !self.axes[n] && self.axes[pl.u as usize] && self.axes[pl.v as usize] {
                    self.axes[n] = true;
                    newly = true;
                }
            }
        }
        if newly {
            sk.seed_normals();
        }
        newly
    }
}

/// What a statement `place` may take reads: its axes and planes, a plane's origin standing for
/// its plane, the planes the other points it reads are drawn in (a held point waits for none),
/// and every point, whose hidden point follows the planes while it is solved.
struct Reads {
    id: u32,
    axes: Vec<usize>,
    planes: Vec<usize>,
    through: Vec<usize>,
    pts: Vec<usize>,
}

/// The statements that may place an axis or a plane, with what each reads (`Reads`): none of the
/// intrinsic rows, a normal's, a point measured from a plane's origin in space, or one reading a
/// point in space the drawing places.
fn statements(sk: &Sketch) -> Vec<Reads> {
    let mut reads: Vec<Reads> = Vec::new();
    let normals: BTreeSet<usize> = sk.frames.iter().filter_map(|f| f.chart.as_ref()?.normal.map(|n| n as usize))
        .collect();
    for c in sk.constraints.iter().filter(|c| c.acts()) {
        if matches!(c.kind, CKind::Lift | CKind::ChartLift | CKind::AxisUnit | CKind::AxisFoot | CKind::DragTarget
            | CKind::DragSeen) {
            continue;
        }
        if c.intrinsic && c.entities().iter().any(|e| e.kind == EntKind::Axis && normals.contains(&e.i())) {
            continue;
        }
        // a point measured from a plane's origin in space — an ordinate, a height off it — is the
        // drawing placed by the plane, not the plane by the drawing, unless the point is where a
        // plane stands
        let from_origin = || sk.plane_of_origin(c.args[0].ent().i()).is_some();
        if c.form().is_some_and(|f| f.in_space()) && from_origin() {
            let p = c.args[1].ent().i();
            if sk.plane_of_origin(p).is_none() && !sk.point_held(p) {
                continue;
            }
        }
        let (mut axes, mut planes, mut pts) = (Vec::new(), Vec::new(), Vec::new());
        for e in c.entities() {
            match e.kind {
                EntKind::Axis => axes.push(e.i()),
                EntKind::Frame => planes.push(e.i()),
                _ => points(sk, e, &mut pts, 0),
            }
        }
        let mut through = Vec::new();
        let mut drawing = false;
        for &p in &pts {
            if let Some(pl) = sk.plane_of_origin(p) {
                planes.push(pl);
            } else if sk.point_held(p) {
                // held: wherever its plane stands, it is where it is said to be
            } else if let Some(pl) = sk.plane_of(p) {
                through.push(pl);
            } else {
                // a point in space that is not held is the drawing's to place, never a plane's
                drawing = true;
            }
        }
        for &pl in &planes {
            axes.extend([sk.frames[pl].u as usize, sk.frames[pl].v as usize]);
        }
        if drawing || axes.is_empty() && planes.is_empty() {
            continue;
        }
        reads.push(Reads { id: c.id, axes, planes, through, pts });
    }
    reads
}

/// The taken statements solved alone: the unplaced axes and places they read free, everything
/// else held, the hidden points free to follow — their numbers written back, and placed what the
/// statements leave no freedom.
fn solve_taken(sk: &mut Sketch, reads: &[Reads], taken: &BTreeSet<u32>, placed: &mut Placed) {
    let mut part = sk.clone();
    let mut free: BTreeSet<u32> = BTreeSet::new();
    let mut read: BTreeSet<usize> = BTreeSet::new();
    for r in reads.iter().filter(|r| taken.contains(&r.id)) {
        read.extend(&r.pts);
        for &k in &r.axes {
            if !placed.axes[k] {
                free.extend(sk.axes[k].d);
            }
            // where an axis stands follows what is solved here whatever placed its direction
            // (a line it is held along, say): a plane's axes pass through its origin
            if sk.axes[k].placed {
                free.extend(sk.axes[k].a);
            }
        }
        for &p in r.planes.iter().filter(|&&p| !placed.planes[p]) {
            free.extend(sk.frames[p].o);
        }
    }
    let item: BTreeSet<u32> = free.clone();
    // the hidden points of the points read follow their planes; no other is in the part
    for l in &sk.lifts {
        if sk.lifted_off(l.point as usize) && read.contains(&(l.point as usize)) {
            free.extend(l.x);
        }
    }
    for (i, q) in part.params.iter_mut().enumerate() {
        q.fixed = q.fixed || !free.contains(&(i as u32));
    }
    // an axis's own rows go with its own numbers: its unit row with a direction solved here,
    // its foot row with a place
    let any_free = |ps: &[u32; 3]| ps.iter().any(|q| free.contains(q));
    part.constraints.retain(|c| {
        taken.contains(&c.id)
            || match c.kind {
                CKind::Lift | CKind::ChartLift => read.contains(&c.args[0].ent().i()),
                CKind::AxisUnit => any_free(&sk.axes[c.args[0].ent().i()].d),
                CKind::AxisFoot => any_free(&sk.axes[c.args[0].ent().i()].a),
                _ => false,
            }
    });
    crate::solve::solve(&mut part, crate::solve::SolveOpts::default());
    for &q in &item {
        sk.params[q as usize].value = part.params[q as usize].value;
    }
    // placed: what the statements taken leave no freedom
    let d = crate::diagnose::diagnose(&mut part, Default::default());
    let under: BTreeSet<u32> = d.under_params.iter().copied().collect();
    let fixed = |ps: &[u32]| ps.iter().all(|q| !under.contains(q));
    for k in 0..sk.axes.len() {
        if !placed.axes[k] && sk.axes[k].d.iter().all(|q| item.contains(q)) {
            placed.axes[k] = fixed(&sk.axes[k].d);
        }
    }
    for p in sk.charted() {
        let pl = &sk.frames[p];
        let o_ok = pl.o.iter().all(|q| !item.contains(q) || !under.contains(q))
            && (held(sk, &pl.o) || pl.o.iter().all(|q| item.contains(q)));
        if !placed.planes[p] {
            placed.planes[p] = o_ok && placed.axes[pl.u as usize] && placed.axes[pl.v as usize];
        }
    }
}

/// Every axis a plane holds along a drawn line (`program::entities::axes_along`) turned to the
/// line as it now stands, its sense included — `parallel` is satisfied both ways, so an axis
/// seeded before its line's points were placed may stand against it, and a plane over it turn
/// its back.
fn align(sk: &mut Sketch) {
    for (r, l) in along(sk) {
        if !sk.axes[r].d.iter().any(|&q| sk.params[q as usize].fixed) {
            sk.turn_axis_along(r, l);
        }
    }
}

/// The axes held along drawn lines, and their lines.
fn along(sk: &Sketch) -> Vec<(usize, usize)> {
    sk.constraints.iter()
        .filter(|c| c.intrinsic && c.along.is_none() && c.kind == CKind::Parallel3)
        .filter_map(|c| match (c.args[0].ent(), c.args[1].ent()) {
            (r, l) if r.kind == EntKind::Axis && l.kind == EntKind::Line => Some((r.i(), l.i())),
            _ => None,
        })
        .collect()
}

/// Every hidden point put where its point now stands — after a pass that moved planes or
/// points without the lift rows that tie them.
pub(crate) fn relift(sk: &mut Sketch) {
    for k in 0..sk.lifts.len() {
        let p = sk.lifts[k].point as usize;
        if sk.lifted_off(p) {
            let at = sk.world_point(p);
            for i in 0..3 {
                let q = sk.lifts[k].x[i] as usize;
                sk.params[q].value = at[i];
            }
        }
    }
}
