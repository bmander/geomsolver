//! **A face family** (#243 L1): the solid a face of a station sweeps as the station's running
//! number runs, `solid(I(…).f over s in (a, b))` — §6.5's curve one dimension up.
//!
//! The station is drawn once, hidden, at the middle of the interval (`flatten::Held`), and solved
//! there with the drawing.  Here it is solved again at every other station: the part of the
//! drawing it is (`io::Part`, everything else held where the drawing stands), its running number
//! moved (`Sketch::held`), its expressions worked out again (`expr::evaluate`) and its unknowns
//! solved, **each station warm-started from its neighbour**, so the branch the drawing chose is
//! the one carried — a step whose stations disagree with the chord between them more than the
//! sheet's flatness is halved, which is also what catches a jump to another branch.
//!
//! A station inside the interval must be **regular**: solved, and of full column rank (no degree
//! of freedom, no lost equation).  Only an end may lose rank — a section shrunk to its apex, an
//! edge to a point — and an end that does is **read as its limit** (`s → b⁻`, finding 9), never
//! solved: the stations close in on it, and the samples are carried to it quadratically.
//!
//! **Correspondence is identity**: each loop is the same edges, entered by the same ends, at every
//! station, so the edge `ab` here is `ab` there, and each edge is sampled at the same fractions of
//! its own run everywhere.  The topology is the station's and is fixed: an interior station whose
//! section crosses itself, loses an edge, turns an edge over or lets a hole touch is refused there,
//! and so is one whose section reaches past the others' (the swept volume crossing itself, which a
//! bend does where the section reaches its centre of curvature).
//!
//! What the stations sweep is faceted: neighbouring stations' samples joined by quads, the two
//! ends capped.  The caps are `start` and `end`; a wall is named by the path of the section's
//! edge under its instance (`sec.outer.ab`, finding 12), so a name never depends on position.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::io::Part;
use crate::model::Leg;
use crate::space::sub;
use crate::syntax::num;
use plane::{cross, dot, norm};
use crate::brep::family::Run;
use crate::brep::recipe::{Family as Walls, FamilyWall};
use std::collections::BTreeSet;
use std::rc::Rc;

type V = [f64; 3];


/// The most stations a family is cut into: past it a step is taken as it stands.
const MAX_STATIONS: usize = 4096;

/// How finely the interval is stepped at most, and at least, as fractions of it: the first step
/// and the coarsest, so a straight family still has stations enough to judge, and the finest a
/// step is halved to before a station that will not solve is refused there.
const FIRST_STEP: f64 = 1.0 / 64.0;
const COARSEST: f64 = 1.0 / 16.0;
const FINEST: f64 = 1e-6;

/// The most samples times stations a family is faceted into, the loft's budget: past it the
/// stations filled in between the solved ones are cut in proportion.
const FACETED: usize = 8192;

/// How near an end the stations close in before it is read as their limit, as a fraction of the
/// interval: the limit's error goes as its cube.
const LIMIT_GAP: f64 = 1e-4;

/// One edge of a loop as the family walks it: the edge, the end it is entered by, and its wall.
#[derive(Clone, Copy, Debug)]
struct Walked {
    edge: EntRef,
    from: u32,
    wall: usize,
}

/// One station: where it stands on the interval, the plane its section is drawn in, its loops
/// sampled — on the plane's page and in space — and the part's parameters there, so it can be
/// read again (empty where no solve stood at it: an end read as a limit, a station filled in).
#[derive(Clone, Debug)]
struct Station {
    s: f64,
    basis: Basis,
    page: Vec<Vec<(f64, f64)>>,
    world: Vec<Vec<V>>,
    x: Vec<f64>,
}

/// **A face family, stepped**: its stations in order of the running number, each loop's samples
/// joined to the next station's, and what each step of a loop is a piece of.
#[derive(Debug)]
pub(crate) struct Family {
    stations: Vec<Station>,
    /// Per loop, per sample: the wall the step from it to the next sample belongs to (an index
    /// into `names`), and whether that step is a chord of a curved edge.
    steps: Vec<Vec<(usize, bool)>>,
    /// The walls' names, by the paths of the section's edges under its instance.
    names: Vec<String>,
    /// Per loop, whether it runs clockwise on its plane's page at the drawn station.
    cw: Vec<bool>,
    /// Whether the section's normal points the way the family runs.
    forward: bool,
    /// The sheet's flatness the family was stepped at.
    tol: f64,
    /// The flow each leg's station was integrated by, where it states a rate (#339 D3; one a
    /// link of a chain, #363 S4), for its walls.
    flows: Vec<super::rated::Shared>,
}

/// The family solid `si` stepped at `unit`, remembered against everything it reads
/// (`Caches::family`).  Stepped no coarser than a report asks, so validation, a report and a
/// mesh, each asking at its own unit, share one stepping.
pub(crate) fn built(sk: &Sketch, si: usize, unit: f64) -> Result<Rc<Family>, String> {
    let unit = unit.min(REPORT_UNIT);
    sk.caches.family.get_or(si, MemoKey::at(sk, si, unit), || march(sk, si, unit).map(Rc::new))
}

/// The station's part of the drawing: everything the station is not held where the drawing
/// stands — the primitives' parameters, the planes' poses, the unknowns of the drawing — so the
/// part walked from its face (`Part::around_all`) stops at the geometry the station was given;
/// and the drawing's unknowns it was handed, by name, to hold at every station.
fn part_of(sk: &Sketch, face: u32, station: &[EntRef], prefix: &str) -> (Part, Vec<String>) {
    // a plane's normal moves with its plane, though it is minted where something reads it, under
    // no name of the station's (`Sketch::plane_normal`)
    let normals = station.iter().filter(|e| e.kind == EntKind::Frame)
        .filter_map(|e| sk.frames[e.i()].normal)
        .map(|n| EntRef::new(EntKind::Axis, n as usize));
    let inside: BTreeSet<EntRef> = station.iter().copied().chain(normals).collect();
    // what the station is made of moves with it, though something outside it shares it — a
    // closing edge another face of the instance made itself, over the station's own corners
    let own: BTreeSet<u32> = station.iter().flat_map(|&e| sk.entity_params(e)).collect();
    let mut held = sk.clone();
    // a copy made to solve reads none of what the drawing remembers
    held.caches = Default::default();
    // a station a chain's later leg carries, which the drawing held where it was seeded, is
    // solved here (#363 S4): its rows and its freedom given back
    for &p in &sk.carried {
        held.params[p as usize].fixed = false;
    }
    for c in &mut held.constraints {
        c.carried = false;
    }
    for e in sk.primitives() {
        if inside.contains(&e) {
            continue;
        }
        let mut ps = sk.entity_params(e);
        ps.retain(|p| !own.contains(p));
        if e.kind == EntKind::Frame {
            let pl = &sk.frames[e.i()];
            ps.extend(pl.o);
            for ax in [pl.u, pl.v] {
                ps.extend(sk.axes[ax as usize].a);
                ps.extend(sk.axes[ax as usize].d);
            }
        }
        for p in ps {
            held.params[p as usize].fixed = true;
        }
    }
    for l in sk.lifts.iter().filter(|l| !inside.contains(&EntRef::point(l.point as usize))) {
        for &x in &l.x {
            held.params[x as usize].fixed = true;
        }
    }
    let mut outer = Vec::new();
    for (name, &p) in sk.free_vars.iter().filter(|(n, _)| !n.starts_with(prefix)) {
        held.params[p as usize].fixed = true;
        outer.push(name.clone());
    }
    // the face and its edges, and the geometry the station's dimensions read (held, so tied to
    // nothing the walk follows)
    let f = &sk.faces[face as usize];
    let read = sk.held.iter().filter(|(n, _)| n.starts_with(prefix)).filter_map(|(_, h)| h.reads);
    let seeds: Vec<EntRef> = std::iter::once(EntRef::face(face as usize))
        .chain(f.boundaries().flat_map(|(edges, _)| edges.iter().copied()))
        .chain(read.map(|(e, _)| e))
        .collect();
    (Part::around_all(&held, &seeds), outer)
}

/// Each loop of face `fi` as walked (`profile::walk_loop`), each edge's wall, and the walls' names
/// under the instance `prefix`.
fn walk_face(sk: &Sketch, fi: usize, prefix: &str)
    -> Result<(Vec<Vec<Walked>>, Vec<String>), String> {
    let f = &sk.faces[fi];
    // the edges' names are the face's own scope's: under the instance, the section's path to them
    let scope = f.name.rsplit_once('.').map_or(String::new(), |(p, _)| format!("{p}."));
    let under = scope.strip_prefix(prefix).unwrap_or("");
    let mut loops = Vec::new();
    let mut names = Vec::new();
    for (edges, edge_names) in f.boundaries() {
        let lone = edges.len() == 1 && matches!(edges[0].kind, EntKind::Circle | EntKind::Curve);
        let ends: Vec<(EntRef, u32)> = if lone {
            vec![(edges[0], u32::MAX)]
        } else {
            super::profile::walk_loop(sk, edges)
                .ok_or("a section's loop is not walked end to end")?
                .into_iter().map(|(e, from, _)| (e, from)).collect()
        };
        let mut walked = Vec::new();
        for (i, (edge, from)) in ends.into_iter().enumerate() {
            let wall = names.len();
            let name = edge_names.get(i).cloned().unwrap_or_else(|| format!("e{wall}"));
            names.push(crate::program::public_path(&format!("{under}{name}")));
            walked.push(Walked { edge, from, wall });
        }
        loops.push(walked);
    }
    if names.iter().any(|n| n == "start" || n == "end") {
        return Err("edge name collides with a sweep cap (`start` or `end`); rename the source \
                    edge".into());
    }
    Ok((loops, names))
}

/// How many samples edge `w` needs to stray less than `tol` from itself: one for a line, a curve
/// by the sagitta rule the drawing is drawn by (`profile::arc_steps`).
fn needed(sk: &Sketch, w: &Walked, tol: f64) -> usize {
    let r = || sk.radius_value(w.edge).abs();
    match w.edge.kind {
        EntKind::Line => 1,
        EntKind::Circle => super::profile::arc_steps(r(), std::f64::consts::TAU, tol),
        EntKind::Arc => {
            let (a0, a1) = sk.arc_angles(w.edge.i());
            super::profile::arc_steps(r(), a1 - a0, tol)
        }
        EntKind::Spline => crate::curve::tessellate(sk, w.edge.i(), tol / crate::curve::FLATNESS_PX)
            .len().saturating_sub(1).clamp(4, 4096),
        _ => sk.curve_polyline_within(w.edge.i(), tol).len().saturating_sub(1).clamp(4, 4096),
    }
}

/// Edge `w` at `n` places on its run, from where the walk enters it up to (not at) where it
/// leaves: on the page of the plane it is drawn in.
fn sample(sk: &Sketch, w: &Walked, n: usize, out: &mut Vec<(f64, f64)>) {
    let frac = |k: usize| k as f64 / n as f64;
    let round = |c: (f64, f64), r: f64, start: f64, sweep: f64, out: &mut Vec<(f64, f64)>| {
        for k in 0..n {
            let (sn, cs) = (start + sweep * frac(k)).dsin_cos();
            out.push((c.0 + r * cs, c.1 + r * sn));
        }
    };
    match w.edge.kind {
        EntKind::Line => out.push(sk.point_xy(w.from as usize)),
        EntKind::Circle => {
            let c = sk.point_xy(sk.round_center(w.edge));
            round(c, sk.radius_value(w.edge).abs(), 0.0, std::f64::consts::TAU, out);
        }
        EntKind::Arc => {
            // how far an arc goes is the arc's; which way, the walk's (`profile::walk_edge`)
            let arc = &sk.arcs[w.edge.i()];
            let c = sk.point_xy(arc.center as usize);
            let (a0, a1) = sk.arc_angles(w.edge.i());
            let (start, sweep) = if w.from == arc.start { (a0, a1 - a0) } else { (a1, a0 - a1) };
            round(c, sk.radius_value(w.edge).abs(), start, sweep, out);
        }
        // a spline or a curve by its own parameter, the way the walk runs it; a closed curve
        // standing alone over its whole domain
        _ => {
            let [t0, t1] = super::profile::curved_span(sk, w.edge, w.from)
                .map_or_else(|| { let (a, b) = sk.curve_domain(w.edge.i()); [a, b] }, |(_, t)| t);
            for k in 0..n {
                out.push(super::profile::curved_point(sk, w.edge, t0 + (t1 - t0) * frac(k)));
            }
        }
    }
}

/// What the family reads off the part: its face, its running number's parameter, the loops as
/// walked, the unknowns of the drawing it holds, and the sheet's flatness and the interval it is
/// stepped over.
struct Reading {
    face: usize,
    held: u32,
    loops: Vec<Vec<Walked>>,
    /// The unknowns of the drawing the part was handed, held as they stand at every station.
    outer: Vec<String>,
    tol: f64,
    span: f64,
    /// Where the station states a rate along its running number (#339 D3), the flow it is
    /// integrated by from its home, marched once the home is solved (`start_flow`).
    rated: Option<String>,
    flow: std::cell::RefCell<super::rated::Shared>,
}

impl Reading {
    /// The station the part stands at now, its loops sampled `counts` times an edge.
    fn station(&self, sk: &Sketch, s: f64, counts: &[Vec<usize>]) -> Result<Station, String> {
        let plane = sk.faces[self.face].plane;
        let basis = plane.map_or(Basis::page(), |p| sk.basis(p as usize));
        let mut page = Vec::new();
        for (walked, n) in self.loops.iter().zip(counts) {
            let mut ring = Vec::new();
            for (w, &k) in walked.iter().zip(n) {
                sample(sk, w, k, &mut ring);
            }
            page.push(ring);
        }
        let world = page.iter()
            .map(|r| r.iter().map(|&(x, y)| basis.lift(x, y)).collect())
            .collect();
        Ok(Station { s, basis, page, world, x: sk.get_x() })
    }

    /// Each edge's count raised to what the part, where it stands, needs of it.
    fn need(&self, sk: &Sketch, counts: &mut [Vec<usize>]) {
        for (n, walked) in counts.iter_mut().zip(&self.loops) {
            for (k, w) in n.iter_mut().zip(walked) {
                *k = (*k).max(needed(sk, w, self.tol));
            }
        }
    }

    /// The station at `s`: integrated there, where it states a rate and its flow has been
    /// marched (no system: its rank is the flow's, checked at every step), else solved, warm from
    /// wherever the part stands — the compiled system, or why it does not solve.
    fn settle(&self, sk: &mut Sketch, s: f64) -> Result<Option<crate::system::System>, String> {
        if let Some(flow) = self.flow.borrow().0.clone() {
            flow.borrow_mut().put(sk, s)?;
            return Ok(None);
        }
        self.solve(sk, s).map(Some)
    }

    /// The station a leg starts from, at `s`: the drawn one, or the first of a leg carried on
    /// from the last (`carry_over`), solved and held to the rule every other is.  A carried leg
    /// whose station states a rate (#363 S4) has its home where the last leg's flow ended: its
    /// rows met from the state carried across the joint, its rows `initially` no rows of it (they
    /// hold at the drawn station alone), and its rank the flow's, so no regularity is asked.
    fn first(&self, sk: &mut Sketch, s: f64, carried: bool, regular: bool)
        -> Result<(), String> {
        if !(carried && self.rated.is_some()) {
            let mut sys = self.solve(sk, s)?;
            return if regular { Self::regular(&mut sys, sk) } else { Ok(()) };
        }
        for c in sk.constraints.iter_mut().filter(|c| c.initially) {
            c.claim = true;
        }
        self.solve(sk, s).map(|_| ())
    }

    /// The flow of a station that states a rate, marched from its home where the part stands,
    /// solved, over `(from, to)` — or the one `given`, marched already — nothing for one that
    /// states none.
    fn start_flow(&self, sk: &Sketch, from: f64, to: f64, given: &super::rated::Shared)
        -> Result<(), String> {
        let Some(held) = &self.rated else { return Ok(()) };
        let flow = match &given.0 {
            Some(f) => f.clone(),
            None => {
                let home = sk.params[self.held as usize].value;
                let f = super::rated::Flow::march(sk, held, home, from, to)?;
                std::rc::Rc::new(std::cell::RefCell::new(f))
            }
        };
        *self.flow.borrow_mut() = super::rated::Shared(Some(flow));
        Ok(())
    }

    /// Solve the station at `s`, warm from wherever the part stands: the compiled system, or why
    /// it does not solve.
    fn solve(&self, sk: &mut Sketch, s: f64) -> Result<crate::system::System, String> {
        sk.params[self.held as usize].value = s;
        for item in crate::expr::evaluate(sk) {
            if let Some(e) = item.error {
                return Err(format!("`{}` does not work out there: {e}", item.text));
            }
        }
        for n in &self.outer {
            if let Some(&p) = sk.free_vars.get(n) {
                sk.params[p as usize].fixed = true;
            }
        }
        let mut sys = crate::system::System::new(sk);
        if !sys.solve(sk, crate::solve::SolveOpts::default()).success {
            return Err("does not solve".into());
        }
        Ok(sys)
    }

    /// Whether the station `sys` solved is regular: of full column rank, no freedom left and no
    /// equation lost — asked only of a station the step keeps, since the rank is the dear part.
    fn regular(sys: &mut crate::system::System, sk: &Sketch) -> Result<(), String> {
        let z = sys.z0(sk);
        let rank = sys.rank(&z, crate::system::RANK_TOL, true);
        if rank < sys.n_free {
            let n = sys.n_free - rank;
            let plural = if n == 1 { "" } else { "s" };
            return Err(format!("is not held: {n} degree{plural} of freedom"));
        }
        Ok(())
    }
}

/// Put the part back where station `st` stood.
fn restore(sk: &mut Sketch, st: &Station) {
    put(sk, &st.x);
}

/// Put the part's parameters at `x`.
fn put(sk: &mut Sketch, x: &[f64]) {
    for (p, &v) in sk.params.iter_mut().zip(x) {
        p.value = v;
    }
}

/// Start the part's solve at `s` where the stations solved at `a` and `b` (each its running
/// number and its parameters) carry it, along the line through them.
fn carry(sk: &mut Sketch, (sa, xa): (f64, &[f64]), (sb, xb): (f64, &[f64]), s: f64) {
    let k = (s - sb) / (sb - sa);
    for ((p, &xa), &xb) in sk.params.iter_mut().zip(xa).zip(xb) {
        if !p.fixed {
            p.value = xb + k * (xb - xa);
        }
    }
}

/// The Lagrange weights of the stations at `at` read at `s`: the polynomial through them.
fn weights(at: &[f64], s: f64) -> Vec<f64> {
    (0..at.len()).map(|i| {
        (0..at.len()).filter(|&j| j != i).map(|j| (s - at[j]) / (at[i] - at[j])).product()
    }).collect()
}

/// The polynomial through stations `from`, read at `s`, of what `pick` reads of each.
fn mix(from: &[&Station], w: &[f64], pick: &dyn Fn(&Station) -> V) -> V {
    std::array::from_fn(|k| from.iter().zip(w).map(|(st, w)| w * pick(st)[k]).sum())
}

/// Every sample of the polynomial through stations `from`, read at `s`, in space.
fn world_at(from: &[&Station], w: &[f64]) -> Vec<Vec<V>> {
    (0..from[0].world.len()).map(|l| {
        (0..from[0].world[l].len()).map(|i| mix(from, w, &|st| st.world[l][i])).collect()
    }).collect()
}

/// The polynomial through stations `from`, read at `s`: every sample and the plane, a station no
/// solve stood at — one filled in between, or an end read as the limit of those before it.
fn through(from: &[&Station], s: f64) -> Result<Station, String> {
    let w = weights(&from.iter().map(|st| st.s).collect::<Vec<_>>(), s);
    let world = world_at(from, &w);
    let o = mix(from, &w, &|st| st.basis.o);
    let basis = Basis::explicit(mix(from, &w, &|st| st.basis.u), mix(from, &w, &|st| st.basis.v))
        .map(|bs| Basis { o, ..bs })
        .ok_or("its plane comes out of no direction")?;
    let page = world.iter().map(|r| r.iter().map(|&p| basis.view_coords(p)).collect()).collect();
    Ok(Station { s, basis, page, world, x: Vec::new() })
}

/// How far the samples of `st` stand from where the cubic through the four stations before it
/// carries them: an estimate, by the ratio of a cubic's extrapolation error to its interpolation
/// error, of how far a cubic through four such stations strays between them.
fn cubic_error(before: &[&Station], st: &Station) -> f64 {
    let w = weights(&before.iter().map(|b| b.s).collect::<Vec<_>>(), st.s);
    let off = world_at(before, &w).iter().flatten().zip(st.world.iter().flatten())
        .map(|(p, q)| norm(sub(*p, *q))).fold(0.0f64, f64::max);
    // extrapolating a step past four stations a step apart errs by h⁴|P⁗|; interpolating within
    // the middle of them, by 9/384 of that
    off * 9.0 / 384.0
}

/// The stations from `start` (solved, the part standing there) on to the end `to`, each warm from
/// where the stations before it carry it, and as far apart as a cubic through four of them stays
/// within the flatness between them; `before` the station on the other side of `start`, where
/// there is one.  Each kept station raises `need` to what its edges ask (`Reading::need`).
/// Whether the end would not solve, so the stations closed in on it to be carried there, comes
/// back beside them (`limit`).
fn run(
    sk: &mut Sketch,
    rd: &Reading,
    start: &Station,
    before: Option<&Station>,
    to: f64,
    counts: &[Vec<usize>],
    need: &mut [Vec<usize>],
) -> Result<(Vec<Station>, bool), String> {
    let dir = (to - start.s).signum();
    let (coarsest, finest) = (rd.span * COARSEST, rd.span * FINEST);
    let mut h = rd.span * FIRST_STEP;
    // the stations so far, `before` and `start` first: a step is judged by the four before it
    let mut trail: Vec<Station> = before.into_iter().cloned().chain([start.clone()]).collect();
    let skip = trail.len();
    // where the end will not solve: the stations close in on it, to be carried to it
    let mut closing = false;
    loop {
        let prev = trail.last().expect("the run starts somewhere");
        let left = (to - prev.s) * dir;
        if left <= 0.0 || trail.len() - skip >= MAX_STATIONS {
            break;
        }
        if closing && left <= rd.span * LIMIT_GAP {
            if trail.len() < 3 {
                return Err("its end does not solve, and no station stands near enough to read \
                            it from".into());
            }
            break;
        }
        let step = if closing { left / 2.0 } else { h.min(left) };
        let s = if step >= left { to } else { prev.s + dir * step };
        // the solve starts where the last two stations carry it
        restore(sk, prev);
        if let [.., a, b] = trail.as_slice() {
            carry(sk, (a.s, &a.x), (b.s, &b.x), s);
        }
        let settled = rd.settle(sk, s).and_then(|sys| {
            let st = rd.station(sk, s, counts)?;
            // a step whose stations stray from the cubic through the four before is halved
            let n = trail.len();
            if !closing && n >= 4 {
                let last: Vec<&Station> = trail[n - 4..].iter().collect();
                let e = cubic_error(&last, &st);
                if e > rd.tol && step > finest {
                    return Ok(None);
                }
                let grow = if e > 0.0 { (rd.tol / e).powf(0.25) * 0.9 } else { 2.0 };
                h = (step * grow.clamp(0.5, 2.0)).clamp(finest, coarsest);
            } else if !closing {
                h = (step * 1.5).min(coarsest);
            }
            if let Some(mut sys) = sys {
                Reading::regular(&mut sys, sk)?;
            }
            Ok(Some(st))
        });
        match settled {
            Ok(Some(st)) => {
                rd.need(sk, need);
                trail.push(st);
            }
            Ok(None) => h = step / 2.0,
            Err(why) if closing => {
                let (to, s) = (num(to), num(s));
                return Err(format!("the stations closing in on its end at {to} do not solve: the \
                                    station at {s} {why}"))
            }
            // only an end may lose rank: it is read as the limit of the stations closing in
            Err(_) if s == to => closing = true,
            Err(_) if step > finest => h = step / 2.0,
            Err(why) => return Err(format!("the station at {} {why}", num(s))),
        }
    }
    Ok((trail.split_off(skip), closing))
}

/// The end `to` read as the limit of the last three of `stations` closing in on it (finding 9):
/// every sample, and the plane, carried quadratically to the end.  A vanished section comes out
/// its apex, a vanished edge a point.
fn limit(stations: &[Station], to: f64) -> Result<Station, String> {
    let n = stations.len();
    through(&[&stations[n - 3], &stations[n - 2], &stations[n - 1]], to)
}

/// The solved stations filled in between (`through`, a cubic over the four nearest) as finely as
/// the chord rule asks of the samples' runs — the sagitta of a step within `tol`, as an arc is cut
/// — and, within the budget of samples times stations (`FACETED`), as a wall's twist asks of its
/// quads.
fn densify(solved: Vec<Station>, tol: f64, samples: usize) -> Result<Vec<Station>, String> {
    let n = solved.len();
    if n < 4 {
        return Ok(solved);
    }
    // per interval, how many steps its runs ask for, and its twist
    let (mut runs, mut twists) = (Vec::with_capacity(n - 1), Vec::with_capacity(n - 1));
    for k in 0..n - 1 {
        let (a, b) = (&solved[k], &solved[k + 1]);
        let c = if k + 2 < n { &solved[k + 2] } else { &solved[k - 1] };
        let (h1, h2) = (b.s - a.s, c.s - b.s);
        let (mut run, mut worst) = (0.0f64, 0.0f64);
        for l in 0..a.world.len() {
            let m = a.world[l].len();
            for i in 0..m {
                // the run's second difference; its sagitta over a step h1 is |P''| h1² / 8
                let (pa, pb, pc) = (a.world[l][i], b.world[l][i], c.world[l][i]);
                let d2: V = std::array::from_fn(|q| {
                    2.0 * ((pc[q] - pb[q]) / h2 - (pb[q] - pa[q]) / h1) / (h1 + h2)
                });
                run = run.max((norm(d2) * h1 * h1 / 8.0 / tol).sqrt());
                // how far the quad's fourth corner stands off the plane of the other three: a
                // twisted wall, which its fan meets an eighth of that from the ruled patch
                let j = (i + 1) % m;
                let (qb, qc) = (a.world[l][j], b.world[l][j]);
                let along = cross(sub(qb, pa), sub(pb, pa));
                if let Some(n) = plane::unit(along) {
                    worst = worst.max(dot(sub(sub(qc, qb), sub(pb, pa)), n).abs() / (8.0 * tol));
                }
            }
        }
        runs.push(run.ceil().max(1.0));
        twists.push(worst.ceil().max(1.0));
    }
    // the runs' steps are owed, to `MAX_STATIONS`; the twist's are taken within the budget (no
    // less than what the runs took), each interval's cut in proportion past it
    let fit = |want: &[f64], room: f64| {
        let total: f64 = want.iter().sum();
        (if total > room { room / total } else { 1.0 }, total.min(room))
    };
    let (runs_scale, owed) = fit(&runs, MAX_STATIONS as f64);
    let (twist_scale, _) = fit(&twists, ((FACETED / samples.max(1)) as f64).max(owed));
    let steps: Vec<usize> = runs.iter().zip(&twists)
        .map(|(r, t)| (r * runs_scale).max(t * twist_scale).floor().max(1.0) as usize)
        .collect();
    let mut filled: Vec<Vec<Station>> = Vec::with_capacity(n - 1);
    for (k, &m) in steps.iter().enumerate() {
        let lo = k.saturating_sub(1).min(n - 4);
        let near: Vec<&Station> = solved[lo..lo + 4].iter().collect();
        let (a, b) = (solved[k].s, solved[k + 1].s);
        filled.push((1..m).map(|j| through(&near, a + (b - a) * j as f64 / m as f64))
            .collect::<Result<_, _>>()?);
    }
    let mut out = Vec::with_capacity(n + steps.iter().sum::<usize>());
    for (st, between) in solved.into_iter().zip(filled.into_iter().map(Some).chain([None])) {
        out.push(st);
        out.extend(between.into_iter().flatten());
    }
    Ok(out)
}

/// One leg's station read off its part (#243 L4: a family over a chain is one leg a link; over
/// one path, the one): the part, what is read of it, and the walls' names.
struct LegPart {
    part: Part,
    rd: Reading,
    names: Vec<String>,
}

/// The part leg `leg` of family `si` is solved over, and what the family reads of it, stepped at
/// the flatness `tol`.
fn leg_part(sk: &Sketch, station: &[EntRef], leg: &Leg, tol: f64) -> Result<LegPart, String> {
    let prefix = crate::model::station_prefix(leg.held);
    let (part, outer) = part_of(sk, leg.face, station, &prefix);
    let pf = part.entity_in(EntRef::face(leg.face as usize))
        .ok_or("the station's face is not in its part")?.i();
    let held = part.sketch.held.get(leg.held).ok_or("its running number is not held")?.param;
    let (loops, names) = walk_face(&part.sketch, pf, &prefix)?;
    let rated = super::rated::states_rates(&part.sketch, leg.held).then(|| leg.held.to_string());
    if rated.is_none() && part.sketch.constraints.iter().any(|c| c.initially) {
        return Err("its station states rows `initially` and no rate along its running number: \
                    only a station integrated from its home has one".into());
    }
    let rd = Reading { face: pf, held, loops, outer, tol, span: leg.to - leg.from, rated,
                       flow: Default::default() };
    Ok(LegPart { part, rd, names })
}

/// **Where one link meets the next** (#243 L4): leg `to`'s part started where leg `from`'s
/// stands, each entity of its station (`FamilyLeg::ents`, paired by name) put where its twin in
/// the other is — a point at the same place in space, a plane in the same pose, a radius the
/// same — so the next link's first station is solved from the last one's, and the branch the
/// drawing chose on the first link is the one carried along the chain.  Fixed parameters are
/// left as they stand: what the part holds is the drawing's.
fn carry_over(from: &LegPart, to: &mut LegPart, pairs: &[(EntRef, EntRef)]) {
    // each pair as the parts have it, read before either is written
    let pairs: Vec<(EntRef, EntRef)> = pairs.iter()
        .filter_map(|&(ea, eb)| Some((from.part.entity_in(ea)?, to.part.entity_in(eb)?)))
        .collect();
    let (a, b) = (&from.part.sketch, &mut to.part.sketch);
    let set = |b: &mut Sketch, p: u32, v: f64| {
        let q = &mut b.params[p as usize];
        if !q.fixed && v.is_finite() {
            q.value = v;
        }
    };
    for (pa, pb) in pairs {
        if pa.kind != pb.kind {
            continue;
        }
        match pa.kind {
            EntKind::Point => {
                let w = a.world_point(pa.i());
                let q = b.points[pb.i()].clone();
                match (q.z, q.plane) {
                    (Some(z), _) => {
                        for (p, v) in [q.x, q.y, z].into_iter().zip(w) {
                            set(b, p, v);
                        }
                    }
                    (None, Some(pl)) => {
                        let (x, y) = b.basis(pl as usize).view_coords(w);
                        set(b, q.x, x);
                        set(b, q.y, y);
                    }
                    (None, None) => {}
                }
            }
            EntKind::Frame => {
                let (pl, ql) = (a.frames[pa.i()].clone(), b.frames[pb.i()].clone());
                let copy = |b: &mut Sketch, xs: [u32; 3], ys: [u32; 3]| {
                    for (x, y) in xs.into_iter().zip(ys) {
                        set(b, y, a.params[x as usize].value);
                    }
                };
                copy(b, pl.o, ql.o);
                for (u, v) in [(pl.u, ql.u), (pl.v, ql.v)] {
                    let (au, bv) = (a.axes[u as usize].clone(), b.axes[v as usize].clone());
                    copy(b, au.a, bv.a);
                    copy(b, au.d, bv.d);
                }
            }
            // a line is its ends, which are points of the station themselves
            EntKind::Line => {}
            _ => {
                let (xs, ys) = (a.entity_params(pa), b.entity_params(pb));
                if xs.len() == ys.len() {
                    for (x, y) in xs.into_iter().zip(ys) {
                        set(b, y, a.params[x as usize].value);
                    }
                }
            }
        }
    }
}

/// A leg stepped: its solved stations in order of its own running number, the drawn one's
/// index, whether either end would not solve (to be read as its limit), and the samples its
/// edges need.
struct Stepped {
    stations: Vec<Station>,
    drawn: usize,
    limits: [bool; 2],
    /// The samples the stations were read at, and what their edges came to need.
    counts: Vec<Vec<usize>>,
    need: Vec<Vec<usize>>,
}

/// One leg stepped (see the module): the drawn station, the runs up to its end and down to its
/// start, each edge's samples raised to what every station asks of it.  A leg after the first
/// of a chain (`carried`, its part started where the leg before ended: `carry_over`) is run from
/// its start instead, the station there solved first.
fn step(lp: &mut LegPart, leg: &Leg, carried: bool) -> Result<Stepped, String> {
    let rd = &lp.rd;
    let ps = &mut lp.part.sketch;
    // the drawn station, or the first of a leg carried on from the last, held to the rule every
    // other is
    let first = if carried { leg.from } else { ps.params[rd.held as usize].value };
    rd.first(ps, first, carried, true)
        .map_err(|why| format!("the station at {} {why}", num(first + leg.at)))?;
    // a station that states a rate is integrated from here, its home (#339 D3)
    rd.start_flow(ps, leg.from, leg.to, &Default::default())?;
    let counts: Vec<Vec<usize>> = rd.loops.iter().map(|l| vec![0; l.len()]).collect();
    let mut need = counts.clone();
    rd.need(ps, &mut need);
    let counts = need.clone();
    let centre = rd.station(ps, first, &counts)?;
    let (up, up_limit) = run(ps, rd, &centre, None, leg.to, &counts, &mut need)?;
    if carried {
        let stations = std::iter::once(centre).chain(up).collect();
        return Ok(Stepped { stations, drawn: 0, limits: [false, up_limit], counts, need });
    }
    restore(ps, &centre);
    let (down, down_limit) = run(ps, rd, &centre, up.first(), leg.from, &counts, &mut need)?;
    let drawn = down.len();
    let stations = down.into_iter().rev().chain(std::iter::once(centre)).chain(up).collect();
    Ok(Stepped { stations, drawn, limits: [down_limit, up_limit], counts, need })
}

/// Whether two legs' stations are one section: the same walls, loop by loop.
fn one_section(a: &LegPart, b: &LegPart) -> bool {
    a.names == b.names && a.rd.loops.iter().map(Vec::len).eq(b.rd.loops.iter().map(Vec::len))
}

/// Each pair of entities two neighbouring legs' stations correspond by (`FamilyLeg::ents`).
fn twins(def: &SolidDef) -> Vec<Vec<(EntRef, EntRef)>> {
    let SolidDef::Family { legs, .. } = def else { return Vec::new() };
    legs.windows(2).map(|w| w[0].ents.iter().copied().zip(w[1].ents.iter().copied()).collect())
        .collect()
}

/// **The family stepped** (see the module): each leg's part, its drawn station and the runs to
/// its ends, the samples settled across every leg, each end that would not solve read as its
/// limit, and the stations judged — over a chain, the legs joined where one link meets the next,
/// the station there solved on both and kept once.
fn march(sk: &Sketch, si: usize, unit: f64) -> Result<Family, String> {
    let def = &sk.solids[si].def;
    let SolidDef::Family { station, from, to, .. } = def else {
        return Err("not a face family".into());
    };
    let (a, b) = (from.value, to.value);
    if !(a.is_finite() && b.is_finite() && a < b) {
        return Err("a face family needs a finite interval running forward".into());
    }
    let legs = def.legs();
    let tol = crate::curve::flatness(unit);
    let mut parts: Vec<LegPart> =
        legs.iter().map(|l| leg_part(sk, station, l, tol)).collect::<Result<_, _>>()?;
    if parts.iter().any(|p| !one_section(&parts[0], p)) {
        return Err("the stations on the chain's links are not one section: their edges differ"
            .into());
    }
    // each leg after the first started where the one before it ends
    let pairs = twins(def);
    let mut stepped: Vec<Stepped> = Vec::with_capacity(legs.len());
    for k in 0..legs.len() {
        if k > 0 {
            // only the family's own ends are limits, never where one link meets the next
            if stepped[k - 1].limits[1] {
                return Err(format!("the station at {}, where one link of its chain meets the \
                                    next, does not solve", num(legs[k].at + legs[k].from)));
            }
            let end = stepped[k - 1].stations.last().ok_or("a leg has no stations")?;
            let (before, after) = parts.split_at_mut(k);
            put(&mut before[k - 1].part.sketch, &end.x);
            carry_over(&before[k - 1], &mut after[0], &pairs[k - 1]);
        }
        stepped.push(step(&mut parts[k], &legs[k], k > 0)?);
    }
    // every edge sampled as finely as its finest station on any leg needs, and each station
    // read there again
    let mut need = stepped[0].need.clone();
    for st in &stepped[1..] {
        for (n, m) in need.iter_mut().zip(&st.need) {
            for (x, &y) in n.iter_mut().zip(m) {
                *x = (*x).max(y);
            }
        }
    }
    let mut solved: Vec<Vec<Station>> = Vec::with_capacity(legs.len());
    let mut drawn = 0;
    for (k, ((lp, st), leg)) in parts.iter_mut().zip(&mut stepped).zip(&legs).enumerate() {
        let (rd, ps) = (&lp.rd, &mut lp.part.sketch);
        if st.counts != need {
            for station in &mut st.stations {
                restore(ps, station);
                *station = rd.station(ps, station.s, &need)?;
            }
        }
        let mut stations = std::mem::take(&mut st.stations);
        // an end that would not solve read as the limit of the stations closing in on it (a
        // carried leg has no `down`, and only the last an `up`: refused above)
        let [down, up] = st.limits;
        if down {
            let near: Vec<Station> = stations[..3].iter().rev().cloned().collect();
            stations.insert(0, limit(&near, leg.from)?);
        }
        if up {
            stations.push(limit(&stations, leg.to)?);
        }
        if k == 0 {
            drawn = st.drawn + usize::from(down);
        }
        for station in &mut stations {
            station.s += leg.at;
        }
        solved.push(stations);
    }
    let steps = parts[0].rd.loops.iter().zip(&need).map(|(walked, n)| {
        walked.iter().zip(n).flat_map(|(w, &k)| {
            let curved = !matches!(w.edge.kind, EntKind::Line);
            std::iter::repeat_n((w.wall, curved), k)
        }).collect()
    }).collect();
    let names = parts[0].names.clone();
    let flows = parts.iter().map(|p| p.rd.flow.borrow().clone()).collect();
    let mut fam = Family { stations: joined(solved.clone()), steps, names, cw: Vec::new(),
                           forward: true, tol, flows };
    fam.judge(drawn)?;
    let samples = fam.steps.iter().map(Vec::len).sum();
    let filled = solved.into_iter().map(|leg| densify(leg, tol, samples))
        .collect::<Result<Vec<_>, _>>()?;
    fam.stations = joined(filled);
    fam.ahead()?;
    Ok(fam)
}

/// The legs' stations as one run: where one leg ends the next begins, at the station both
/// solved, kept once.
fn joined(legs: Vec<Vec<Station>>) -> Vec<Station> {
    let mut out: Vec<Station> = Vec::new();
    for leg in legs {
        let skip = usize::from(out.last().zip(leg.first()).is_some_and(|(a, b)| a.s == b.s));
        out.extend(leg.into_iter().skip(skip));
    }
    out
}

impl Family {
    /// How near two samples are to be one: a station's end of a vanished edge, an apex.
    fn near(&self) -> f64 {
        self.tol * 1e-3
    }

    /// A station's loop as a polygon on its plane's page, coincident samples run together: what
    /// an end whose edge has vanished comes to.  An end's are run together within the sheet's
    /// flatness, where an edge solved to nothing stands as near nothing as the solve leaves it.
    fn poly(&self, k: usize, l: usize) -> FacePoly {
        let st = &self.stations[k];
        let near = if k == 0 || k + 1 == self.stations.len() { self.tol } else { self.near() };
        let apart = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).dhypot(a.1 - b.1);
        let mut pts: Vec<(f64, f64)> = Vec::new();
        let mut of = Vec::new();
        for (i, &p) in st.page[l].iter().enumerate() {
            if pts.last().is_some_and(|&q| apart(q, p) <= near) {
                continue;
            }
            pts.push(p);
            of.push(self.steps[l][i]);
        }
        while pts.len() > 1 && apart(pts[0], pts[pts.len() - 1]) <= near {
            pts.pop();
            of.pop();
        }
        // what a loop is asked here is its shape, never its edges' names
        FacePoly { pts, of, names: Vec::new(), basis: st.basis }
    }

    /// Hold every solved station to the station's topology (see the module), and read which way
    /// each loop runs at the drawn one, `drawn`.
    fn judge(&mut self, drawn: usize) -> Result<(), String> {
        let (n, tol) = (self.stations.len(), self.tol);
        if n < 2 {
            return Err("a face family needs more than one station".into());
        }
        let loops = self.steps.len();
        self.cw = (0..loops).map(|l| self.poly(drawn, l).area() < 0.0).collect();
        let area_tol = tol * tol;
        let edges: Vec<Vec<(usize, usize, usize)>> = (0..loops).map(|l| self.edges(l)).collect();
        // each edge's run at the station before, to tell one turned over
        let mut was: Vec<Vec<(f64, V)>> = Vec::new();
        for k in 0..n {
            let end = k == 0 || k == n - 1;
            let s = num(self.stations[k].s);
            let polys: Vec<FacePoly> = (0..loops).map(|l| self.poly(k, l)).collect();
            for (l, p) in polys.iter().enumerate() {
                let what = if l == 0 { "its section".to_string() } else { format!("its hole {l}") };
                // an end may lose an edge, or the whole section to its apex; nothing else may
                let area = p.area();
                if end && (p.pts.len() < 3 || area.abs() <= area_tol) {
                    continue;
                }
                if !p.valid() {
                    return Err(format!("{what} crosses itself or collapses at the station at {s}"));
                }
                if (area < 0.0) != self.cw[l] {
                    return Err(format!("{what} turns over at the station at {s}"));
                }
                // a hole inside the section, and apart from every hole before it
                if l > 0 && polys[0].pts.len() >= 3 {
                    let q = &polys[0];
                    let apart = polys[1..l].iter().filter(|h| h.pts.len() >= 3).all(|h| {
                        !section::loops_touch(h, p) && !section::inside_loop(h, p.pts[0])
                            && !section::inside_loop(p, h.pts[0])
                    });
                    if section::loops_touch(q, p) || !section::inside_loop(q, p.pts[0]) || !apart {
                        return Err(format!("{what} touches or leaves the section, or another \
                                            hole, at the station at {s}"));
                    }
                }
            }
            // an edge lost inside the interval, or turned over from one station to the next
            let mut runs = Vec::with_capacity(loops);
            for (l, edges) in edges.iter().enumerate() {
                let ring = &self.stations[k].world[l];
                let mut here = Vec::with_capacity(edges.len());
                for (e, &(wall, from, to)) in edges.iter().enumerate() {
                    let length: f64 =
                        (from..to).map(|i| norm(sub(ring[(i + 1) % ring.len()], ring[i]))).sum();
                    let chord = sub(ring[to % ring.len()], ring[from]);
                    let name = &self.names[wall];
                    if !end && length <= tol {
                        return Err(format!("the edge `{name}` vanishes at the station at {s}: \
                                            only an end may lose an edge"));
                    }
                    if let Some(&(before, then)) = was.get(l).and_then(|w| w.get(e)) {
                        let turned = dot(chord, then) < 0.0 && to - from == 1;
                        if length > tol && before > tol && turned {
                            let before = num(self.stations[k - 1].s);
                            return Err(format!("the edge `{name}` turns over between the \
                                                stations at {before} and {s}"));
                        }
                    }
                    here.push((length, chord));
                }
                runs.push(here);
            }
            was = runs;
        }
        Ok(())
    }

    /// The swept volume may not cross itself: each station wholly ahead of the one before, on the
    /// side the family moves to — and which side that is, read at the middle.
    fn ahead(&mut self) -> Result<(), String> {
        let n = self.stations.len();
        let (tol, gap) = (self.tol, self.near());
        let mid = (n / 2).min(n - 2);
        let (b, next) = (&self.stations[mid].basis, &self.stations[mid + 1]);
        let pts: Vec<&V> = next.world.iter().flatten().collect();
        let ahead = pts.iter().map(|&&p| dot(b.normal(), sub(p, b.o))).sum::<f64>()
            / pts.len().max(1) as f64;
        if ahead.abs() <= tol {
            return Err("its section does not move off its own plane: a face family sweeps a solid \
                        only where its stations stand one behind the next".into());
        }
        self.forward = ahead > 0.0;
        let sign = if self.forward { 1.0 } else { -1.0 };
        for k in 0..n - 1 {
            let (here, next) = (&self.stations[k], &self.stations[k + 1]);
            let (nh, nn) = (here.basis.normal(), next.basis.normal());
            let behind = next.world.iter().flatten()
                .any(|&p| sign * dot(nh, sub(p, here.basis.o)) <= gap)
                || here.world.iter().flatten()
                    .any(|&p| sign * dot(nn, sub(p, next.basis.o)) >= -gap);
            if behind {
                let (a, b) = (num(here.s), num(next.s));
                return Err(format!("the swept volume crosses itself between the stations at {a} \
                                    and {b}: the section reaches past its path's centre of \
                                    curvature, or turns back"));
            }
        }
        Ok(())
    }

    /// Each edge of loop `l`: its wall, the sample it starts at and the one past its last.
    fn edges(&self, l: usize) -> Vec<(usize, usize, usize)> {
        let mut at = 0;
        self.steps[l].chunk_by(|a, b| a.0 == b.0).map(|run| {
            at += run.len();
            (run[0].0, at - run.len(), at)
        }).collect()
    }

    /// The family faceted, in coordinates about `origin`: the two ends capped (`start`, `end`), a
    /// vanished one left open to its apex, and each wall the quads between neighbouring stations'
    /// samples, split where they do not lie flat.
    pub(crate) fn primitive(&self, origin: V, name: &str) -> Prim {
        let (tol, near) = (self.tol, self.near());
        let mut faces: Vec<String> = vec!["start".into(), "end".into()];
        faces.extend(self.names.iter().cloned());
        let mut facets: Vec<Facet> = Vec::new();
        // which facets meet the caps' strips, whose corners stand on the walls' edges there
        let mut edge_row: Vec<bool> = Vec::new();
        let mut push = |pts: Vec<V>, face: usize, smooth: bool, reverse: bool, row: bool| {
            let mut pts: Vec<V> = pts.into_iter().map(|p| sub(p, origin)).collect();
            pts.dedup_by(|a, b| norm(sub(*a, *b)) <= near);
            while pts.len() > 1 && norm(sub(pts[0], pts[pts.len() - 1])) <= near {
                pts.pop();
            }
            if pts.len() < 3 {
                return;
            }
            if reverse {
                pts.reverse();
            }
            if let Some(n) = facet_normal(&pts) {
                facets.push(Facet { pts, n, face, smooth });
                edge_row.push(row);
            }
        };
        let last = self.stations.len() - 1;
        let loops = self.steps.len();
        for (k, face, reverse) in [(0, 0, self.forward), (last, 1, !self.forward)] {
            let st = &self.stations[k];
            let polys: Vec<FacePoly> = (0..loops).map(|l| self.poly(k, l)).collect();
            if polys[0].pts.len() < 3 || polys[0].area() == 0.0 {
                continue;
            }
            let rings: Vec<&FacePoly> = polys.iter().filter(|p| p.pts.len() >= 3).collect();
            for quad in super::profile::cap(&rings) {
                let pts = quad.iter().map(|&(x, y)| st.basis.lift(x, y)).collect();
                push(pts, face, false, reverse, true);
            }
        }
        for l in 0..loops {
            let reverse = ((!self.forward) != (l > 0)) != self.cw[l];
            let m = self.steps[l].len();
            for k in 0..last {
                let row = k == 0 || k + 1 == last;
                let (p, q) = (&self.stations[k].world[l], &self.stations[k + 1].world[l]);
                for i in 0..m {
                    let j = (i + 1) % m;
                    let (wall, curved) = self.steps[l][i];
                    let (a, b, c, d) = (p[i], p[j], q[j], q[i]);
                    let normal = cross(sub(b, a), sub(c, a));
                    let off = dot(normal, sub(d, a)).abs();
                    let planar = off <= norm(normal) * norm(sub(d, a)) * 1e-10;
                    if planar {
                        push(vec![a, b, c, d], wall + 2, curved, reverse, row);
                    } else if off <= norm(normal) * tol {
                        push(vec![a, b, c], wall + 2, curved, reverse, row);
                        push(vec![a, c, d], wall + 2, curved, reverse, row);
                    } else {
                        // a twisted quad fanned from its middle: the four triangles hold the
                        // ruled patch's volume, where a diagonal would err to one side of it
                        let m: V = std::array::from_fn(|q| (a[q] + b[q] + c[q] + d[q]) / 4.0);
                        for (x, y) in [(a, b), (b, c), (c, d), (d, a)] {
                            push(vec![x, y, m], wall + 2, curved, reverse, row);
                        }
                    }
                }
            }
        }
        // a wall whose facets do not all lie in one plane is a curved face, its joints the
        // tessellation's
        let mut normal: Vec<Option<V>> = vec![None; faces.len()];
        let mut flat = vec![true; faces.len()];
        for f in &facets {
            match normal[f.face] {
                None => normal[f.face] = Some(f.n),
                Some(n) if dot(n, f.n) < 1.0 - 1e-9 => flat[f.face] = false,
                Some(_) => {}
            }
        }
        for f in &mut facets {
            f.smooth |= f.face >= 2 && !flat[f.face];
        }
        // the caps' strips cut the walls' edges at the ends: join those subdivisions before the
        // seams are read — only there, since a wall's quads meet their neighbours corner to corner
        let at_ends: Vec<usize> = (0..facets.len()).filter(|&i| edge_row[i]).collect();
        let pieces: Vec<_> = at_ends.iter().map(|&i| crate::csg::Piece {
            pts: facets[i].pts.clone(),
            n: facets[i].n,
            path: String::new(),
            prim: 0,
            smooth: facets[i].smooth,
        }).collect();
        let welded = crate::mesh::weld(&pieces);
        debug_assert_eq!(welded.len(), at_ends.len());
        for (&i, piece) in at_ends.iter().zip(welded) {
            facets[i].pts = piece.pts;
        }
        finish(facets, faces, name)
    }
}

/// How near the walls' polynomials in `s` must pass a station solved between their nodes, as a
/// part of the section's size: the exact surface's error, well under the kernel's tolerance.
const EXACT: f64 = 1e-10;

/// The longest piece of the interval the walls' polynomials span, as a fraction of it.
const LONGEST_PIECE: f64 = 0.25;

/// Where between a piece's nodes the station is solved again to check its polynomials, as a
/// fraction of the piece: beyond the fifth node, where interpolation on Gauss–Lobatto points errs
/// most.
const CHECKED_AT: f64 = 0.94;

/// Per loop, per edge in walk order, what the edge is (`brep::family::Run`) and whether it runs
/// round alone.
type Runs = Vec<Vec<(Run, bool)>>;

/// Per loop, per edge in walk order, the edge's numbers where the part stands
/// (`brep::family::Run`).
type Numbers = Vec<Vec<Vec<f64>>>;

/// A station the walls are read from: its running number, the part's parameters there, and its
/// edges' numbers.
#[derive(Clone)]
struct Node {
    s: f64,
    x: Vec<f64>,
    numbers: Numbers,
}

/// What each edge is, and whether it runs round alone; a traced curve has no exact wall yet.
fn runs(sk: &Sketch, rd: &Reading) -> Result<Runs, String> {
    rd.loops.iter().map(|walked| walked.iter().map(|w| Ok(match w.edge.kind {
        EntKind::Line => (Run::Line, false),
        EntKind::Circle => (Run::Round, true),
        EntKind::Arc => (Run::Round, false),
        EntKind::Spline => {
            let sp = &sk.splines[w.edge.i()];
            let (knots, weights) = (sp.knots.clone(), sp.weights.clone());
            (Run::Spline { degree: crate::curve::DEGREE, knots, weights }, false)
        }
        _ => return Err("a face family's section holds a traced curve, whose wall has no exact \
                         surface yet".to_string()),
    })).collect()).collect()
}

/// Each edge's numbers where the part stands: on its plane's page, lifted.
fn numbers(sk: &Sketch, rd: &Reading) -> Result<Numbers, String> {
    let plane = sk.faces[rd.face].plane;
    let basis = plane.map_or(Basis::page(), |p| sk.basis(p as usize));
    let point = |q: (f64, f64)| basis.lift(q.0, q.1);
    let vector = |x: f64, y: f64| sub(basis.lift(x, y), basis.lift(0.0, 0.0));
    Ok(rd.loops.iter().map(|walked| walked.iter().map(|w| {
        let mut n: Vec<f64> = Vec::new();
        match w.edge.kind {
            EntKind::Line => {
                let l = &sk.lines[w.edge.i()];
                let to = if l.p1 == w.from { l.p2 } else { l.p1 };
                n.extend(point(sk.point_xy(w.from as usize)));
                n.extend(point(sk.point_xy(to as usize)));
            }
            EntKind::Circle | EntKind::Arc => {
                let c = sk.point_xy(sk.round_center(w.edge));
                let r = sk.radius_value(w.edge).abs();
                // where the walk enters it and how far it turns, as `sample` reads it
                let (start, sweep) = if w.edge.kind == EntKind::Circle {
                    (0.0, std::f64::consts::TAU)
                } else {
                    let arc = &sk.arcs[w.edge.i()];
                    let (a0, a1) = sk.arc_angles(w.edge.i());
                    if w.from == arc.start { (a0, a1 - a0) } else { (a1, a0 - a1) }
                };
                let (sn, cs) = start.dsin_cos();
                n.extend(point(c));
                n.extend(vector(r * cs, r * sn));
                n.extend(vector(-r * sn, r * cs));
                n.push(sweep);
            }
            // a spline (`runs` refuses anything else)
            _ => {
                for &c in &sk.splines[w.edge.i()].ctrl {
                    n.extend(point(sk.point_xy(c as usize)));
                }
                let whole = || { let (a, b) = sk.curve_domain(w.edge.i()); [a, b] };
                let span = super::profile::curved_span(sk, w.edge, w.from);
                n.extend(span.map_or_else(whole, |(_, t)| t));
            }
        }
        n
    }).collect()).collect())
}

/// How far the edges of `got` stand from those of `want`, at five places along each.
fn apart(runs: &Runs, got: &Numbers, want: &Numbers) -> f64 {
    let mut worst = 0.0f64;
    for (l, edges) in runs.iter().enumerate() {
        for (e, (run, _)) in edges.iter().enumerate() {
            for j in 0..=4 {
                let u = j as f64 / 4.0;
                let (p, q) = (run.at(&got[l][e], u).0, run.at(&want[l][e], u).0);
                worst = worst.max(norm(sub(p, q)));
            }
        }
    }
    worst
}

/// The section's size where the part stands, by the spread of its edges.
fn spread(runs: &Runs, numbers: &Numbers) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for (edges, kinds) in numbers.iter().zip(runs) {
        for (n, (run, _)) in edges.iter().zip(kinds) {
            for j in 0..=4 {
                let p = run.at(n, j as f64 / 4.0).0;
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
    }
    norm(sub(hi, lo))
}

/// One piece of the interval as run: its span the way it was run, and its six nodes.
struct Piece {
    span: [f64; 2],
    nodes: Vec<Node>,
}

/// The pieces from the node the part stands at (`start`) on to the end `to`, each as long as its
/// polynomials stay within `tol` of a station solved between its nodes; an end that will not solve
/// read as its limit (the last piece's nodes stop `LIMIT_GAP` short of it).
fn pieces(sk: &mut Sketch, rd: &Reading, runs: &Runs, start: Node, to: f64, tol: f64)
    -> Result<Vec<Piece>, String> {
    use crate::brep::family::NODES;
    let dir = (to - start.s).signum();
    let (longest, finest) = (rd.span * LONGEST_PIECE, rd.span * FINEST);
    let mut h = rd.span * COARSEST;
    let mut here = start;
    // the node before `here`, so each solve starts where the two carry it
    let mut before: Option<Node> = None;
    let mut out = Vec::new();
    while (to - here.s) * dir > rd.span * 1e-12 {
        let left = (to - here.s) * dir;
        let step = h.min(left);
        let end = if step >= left { to } else { here.s + dir * step };
        let mut limit = false;
        let kept = 'tries: loop {
            // the nodes, the last stopping short of an end read as its limit
            let reach = if limit { end - dir * rd.span * LIMIT_GAP } else { end };
            let at = NODES.map(|t| here.s + (reach - here.s) * (t + 1.0) / 2.0);
            let mut nodes = vec![here.clone()];
            for (i, &s) in at.iter().enumerate().skip(1) {
                let last = nodes.last().unwrap();
                put(sk, &last.x);
                if let Some(a) = if i >= 2 { nodes.get(i - 2) } else { before.as_ref() } {
                    carry(sk, (a.s, &a.x), (last.s, &last.x), s);
                }
                match rd.settle(sk, s) {
                    Ok(_) => nodes.push(Node { s, x: sk.get_x(), numbers: numbers(sk, rd)? }),
                    // an end that will not solve is read as its limit
                    Err(_) if i == 5 && end == to && !limit => { limit = true; continue 'tries; }
                    Err(_) if step > finest && !limit => { h = step / 2.0; break 'tries None; }
                    Err(why) => return Err(format!("the station at {} {why}", num(s))),
                }
            }
            // a station solved between the last two nodes, held to the polynomials
            let s = here.s + (reach - here.s) * CHECKED_AT;
            put(sk, &nodes[5].x);
            carry(sk, (nodes[4].s, &nodes[4].x), (nodes[5].s, &nodes[5].x), s);
            let check = match rd.settle(sk, s).and_then(|_| numbers(sk, rd)) {
                Ok(c) => c,
                Err(_) if step > finest => { h = step / 2.0; break 'tries None; }
                Err(why) => return Err(format!("the station at {} {why}", num(s))),
            };
            let w = weights(&at, s);
            let carried: Numbers = check.iter().enumerate().map(|(l, edges)| {
                edges.iter().enumerate().map(|(e, n)| (0..n.len()).map(|q| {
                    (0..6).map(|i| w[i] * nodes[i].numbers[l][e][q]).sum()
                }).collect()).collect()
            }).collect();
            let miss = apart(runs, &carried, &check);
            let grow = if miss > 0.0 { 0.9 * (tol / miss).powf(1.0 / 6.0) } else { 2.0 };
            if miss > tol && step > finest && !limit {
                h = step * grow.clamp(0.1, 0.5);
                break 'tries None;
            }
            if miss > tol * 1e3 {
                return Err(format!("its stations do not run smoothly between {} and {}: the \
                                    walls' polynomials miss a station between them by {}",
                                   num(here.s), num(end), num(miss)));
            }
            h = (step * grow.clamp(0.5, 2.0)).clamp(finest, longest);
            break 'tries Some(Piece { span: [here.s, end], nodes });
        };
        let Some(piece) = kept else { continue };
        // past an end read as its limit, nothing is left to run
        if piece.span[1] == to {
            out.push(piece);
            break;
        }
        before = Some(piece.nodes[4].clone());
        here = piece.nodes[5].clone();
        out.push(piece);
    }
    Ok(out)
}

/// **The family's walls, exact** (#243 L2), as the recipe's `family` node carries them (in model
/// units): the faceted family judged first (its stations' topology, their rank, the swept volume),
/// then the station run from the middle to each end in pieces whose polynomials hold to a station
/// solved between their nodes within `EXACT` of the section's size.
pub(crate) fn walls(sk: &Sketch, si: usize) -> Result<Walls, String> {
    let fam = built(sk, si, REPORT_UNIT)?;
    let def = &sk.solids[si].def;
    let SolidDef::Family { station, .. } = def else {
        return Err("not a face family".into());
    };
    // each leg run in pieces, the legs one after the next in the family's running number — one
    // section on every leg, as the stepping has judged (`march`), so the first's walls
    let mut ordered: Vec<Piece> = Vec::new();
    let mut walls: Option<(Runs, Vec<String>)> = None;
    let pairs = twins(def);
    // the leg before, standing where it ends: where the next starts (`carry_over`)
    let mut before: Option<LegPart> = None;
    for (k, leg) in def.legs().into_iter().enumerate() {
        let mut lp = leg_part(sk, station, &leg, fam.tol)?;
        let carried = before.is_some();
        if let Some(b) = &before {
            carry_over(b, &mut lp, &pairs[k - 1]);
        }
        let ps = &mut lp.part.sketch;
        let rd = &lp.rd;
        let runs = runs(ps, rd)?;
        let first = if carried { leg.from } else { ps.params[rd.held as usize].value };
        rd.first(ps, first, carried, false)
            .map_err(|why| format!("the station at {} {why}", num(first + leg.at)))?;
        let flow = fam.flows.get(k).cloned().unwrap_or_default();
        rd.start_flow(ps, leg.from, leg.to, &flow)?;
        // an integrated station starts where its flow does, the one the stepping marched
        if rd.rated.is_some() {
            rd.settle(ps, first)
                .map_err(|why| format!("the station at {} {why}", num(first + leg.at)))?;
        }
        let centre = Node { s: first, x: ps.get_x(), numbers: numbers(ps, rd)? };
        let tol = EXACT * spread(&runs, &centre.numbers).max(f64::MIN_POSITIVE);
        let up = pieces(ps, rd, &runs, centre.clone(), leg.to, tol)?;
        let down = if carried { Vec::new() } else { pieces(ps, rd, &runs, centre, leg.from, tol)? };
        // the part left where the leg ends, for the next to start from
        if let Some(end) = up.last().and_then(|p| p.nodes.last()) {
            put(ps, &end.x);
        }
        // the pieces in order of the running number, each run down turned round
        ordered.extend(down.into_iter().rev().map(|mut p| {
            p.span.reverse();
            p.nodes.reverse();
            p
        }).chain(up).map(|mut p| {
            p.span = p.span.map(|s| s + leg.at);
            for n in &mut p.nodes {
                n.s += leg.at;
            }
            p
        }));
        if walls.is_none() {
            walls = Some((runs, lp.names.clone()));
        }
        before = Some(lp);
    }
    let (runs, names) = walls.ok_or("a face family has no legs")?;
    let spans = ordered.iter().map(|p| p.span).collect();
    let at = ordered.iter().map(|p| std::array::from_fn(|i| p.nodes[i].s)).collect();
    let mut names = names.into_iter();
    let loops = runs.iter().enumerate().map(|(l, edges)| {
        edges.iter().enumerate().map(|(e, (run, closed))| FamilyWall {
            name: names.next().unwrap_or_default(),
            run: run.clone(),
            closed: *closed,
            nodes: ordered.iter().flat_map(|p| &p.nodes).map(|n| n.numbers[l][e].clone())
                .collect(),
        }).collect()
    }).collect();
    Ok(Walls { spans, at, vanish: fam.tol, loops })
}

/// A face family's walls as the recipe writes them (`walls`): every point in millimetres about the
/// origin written about, every direction in millimetres, an arc's sweep and a spline's span as
/// they stand.
pub(crate) fn json(f: &Walls, w: &crate::brep::recipe::Writer) -> crate::json::Json {
    use crate::json::{object, Json};
    let numbers = |run: &Run, n: &[f64]| -> Json {
        let (pts, dirs, _) = run.shape(n.len());
        let mut out = n.to_vec();
        for i in 0..pts + dirs {
            let v: V = std::array::from_fn(|k| n[3 * i + k] * w.scale);
            let v = if i < pts { w.moved(v) } else { v };
            out[3 * i..3 * i + 3].copy_from_slice(&v);
        }
        Json::Arr(out.into_iter().map(Json::from).collect())
    };
    let list = |xs: &[f64]| Json::Arr(xs.iter().map(|&x| x.into()).collect());
    let loops = f.loops.iter().map(|walls| Json::Arr(walls.iter().map(|wall| {
        let mut o = object([("name", wall.name.clone().into()), ("run", wall.run.kind().into()),
            ("closed", Json::Bool(wall.closed)),
            ("nodes", Json::Arr(wall.nodes.iter().map(|n| numbers(&wall.run, n)).collect()))]);
        if let Run::Spline { degree, knots, weights } = &wall.run {
            o.set("degree", (*degree).into());
            o.set("knots", list(knots));
            if let Some(ws) = weights {
                o.set("weights", list(ws));
            }
        }
        o
    }).collect())).collect();
    object([("kind", "family".into()),
        ("spans", Json::Arr(f.spans.iter().map(|s| list(s)).collect())),
        ("at", Json::Arr(f.at.iter().map(|s| list(s)).collect())),
        ("vanish", (f.vanish * w.scale).into()),
        ("loops", Json::Arr(loops))])
}
