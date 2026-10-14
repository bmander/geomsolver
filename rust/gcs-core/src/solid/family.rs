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
use crate::space::sub;
use plane::{cross, dot, norm};
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
/// read again.
#[derive(Clone, Debug)]
struct Station {
    s: f64,
    basis: Basis,
    page: Vec<Vec<(f64, f64)>>,
    world: Vec<Vec<V>>,
    x: Vec<f64>,
    /// An end read as the limit of the stations closing in on it, not solved (`extrapolate`).
    limit: bool,
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
    /// Whether the section's normal points the way the family runs.
    forward: bool,
    /// How near two samples are to be one: a station's end of a vanished edge, an apex.
    near: f64,
}

/// The family solid `si` stepped at `unit`, remembered against everything it reads
/// (`Caches::family`).
pub(crate) fn built(sk: &Sketch, si: usize, unit: f64) -> Result<Rc<Family>, String> {
    sk.caches.family.get_or(si, MemoKey::at(sk, si, unit), || march(sk, si, unit).map(Rc::new))
}

/// The station's part of the drawing: everything the station is not held where the drawing
/// stands — the primitives' parameters, the planes' poses, the unknowns of the drawing — so the
/// part walked from its face (`Part::around_all`) stops at the geometry the station was given.
fn part_of(sk: &Sketch, face: u32, station: &[EntRef], prefix: &str) -> Part {
    let inside: BTreeSet<EntRef> = station.iter().copied().collect();
    let mut held = sk.clone();
    for e in sk.primitives() {
        if inside.contains(&e) {
            continue;
        }
        let mut ps = sk.entity_params(e);
        if e.kind == EntKind::Plane {
            let pl = &sk.planes[e.i()];
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
    for (name, &p) in &sk.free_vars {
        if !name.starts_with(prefix) {
            held.params[p as usize].fixed = true;
        }
    }
    // the face and its edges, and the geometry the station's dimensions read (held, so tied to
    // nothing the walk follows)
    let f = &sk.faces[face as usize];
    let read = sk.held.iter().filter(|(n, _)| n.starts_with(prefix)).filter_map(|(_, h)| h.reads);
    let seeds: Vec<EntRef> = std::iter::once(EntRef::face(face as usize))
        .chain(f.boundaries().flat_map(|(edges, _)| edges.iter().copied()))
        .chain(read.map(|(e, _)| e))
        .collect();
    Part::around_all(&held, &seeds)
}

/// Each loop of face `fi` as walked (`profile::loop_poly`'s rule: each edge entered where the last
/// one left), each edge's wall, and the walls' names under the instance `prefix`.
fn walk_face(sk: &Sketch, fi: usize, prefix: &str)
    -> Result<(Vec<Vec<Walked>>, Vec<String>), String> {
    let f = &sk.faces[fi];
    // the edges' names are the face's own scope's: under the instance, the section's path to them
    let scope = f.name.rsplit_once('.').map_or(String::new(), |(p, _)| format!("{p}."));
    let under = scope.strip_prefix(prefix).unwrap_or("");
    let mut loops = Vec::new();
    let mut names = Vec::new();
    for (edges, edge_names) in f.boundaries() {
        let mut walked = Vec::new();
        let mut at: Option<u32> = None;
        for (i, &e) in edges.iter().enumerate() {
            let wall = names.len();
            let name = edge_names.get(i).cloned().unwrap_or_else(|| format!("e{wall}"));
            names.push(crate::program::public_path(&format!("{under}{name}")));
            let lone = edges.len() == 1 && matches!(e.kind, EntKind::Circle | EntKind::Curve);
            if lone {
                walked.push(Walked { edge: e, from: u32::MAX, wall });
                continue;
            }
            let (a, b) = crate::model::edge_ends(sk, e).ok_or("a section's edge has no ends")?;
            let (from, to) = match at {
                None => {
                    let (la, lb) = edges.last().and_then(|&l| crate::model::edge_ends(sk, l))
                        .ok_or("a section's edge has no ends")?;
                    if a == la || a == lb { (a, b) } else { (b, a) }
                }
                Some(p) if p == a => (a, b),
                Some(p) if p == b => (b, a),
                Some(_) => return Err("a section's loop is not walked end to end".into()),
            };
            walked.push(Walked { edge: e, from, wall });
            at = Some(to);
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
/// by the sagitta rule the drawing is drawn by (`profile::tessellate_arc`'s).
fn needed(sk: &Sketch, w: &Walked, tol: f64) -> usize {
    let arc = |r: f64, sweep: f64| {
        let step = (if r > tol { 2.0 * (1.0 - tol / r).dacos() } else { std::f64::consts::TAU })
            .min(std::f64::consts::TAU / 64.0);
        ((sweep.abs() / step).ceil() as usize).clamp(2, 4096)
    };
    match w.edge.kind {
        EntKind::Line => 1,
        EntKind::Circle => arc(sk.radius_value(w.edge).abs(), std::f64::consts::TAU),
        EntKind::Arc => {
            let (a0, a1) = sk.arc_angles(w.edge.i());
            arc(sk.radius_value(w.edge).abs(), (a1 - a0).rem_euclid(std::f64::consts::TAU))
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
    match w.edge.kind {
        EntKind::Line => out.push(sk.point_xy(w.from as usize)),
        EntKind::Circle => {
            let (c, r) = (sk.point_xy(sk.round_center(w.edge)), sk.radius_value(w.edge).abs());
            for k in 0..n {
                let (sn, cs) = (std::f64::consts::TAU * frac(k)).dsin_cos();
                out.push((c.0 + r * cs, c.1 + r * sn));
            }
        }
        EntKind::Arc => {
            // how far an arc goes is the arc's; which way, the walk's (`profile::walk_edge`)
            let arc = &sk.arcs[w.edge.i()];
            let (c, r) = (sk.point_xy(arc.center as usize), sk.radius_value(w.edge).abs());
            let (a0, a1) = sk.arc_angles(w.edge.i());
            let mut extent = a1 - a0;
            while extent <= 0.0 {
                extent += std::f64::consts::TAU;
            }
            let ccw = w.from == arc.start;
            let (start, step) = if ccw { (a0, extent) } else { (a1, -extent) };
            for k in 0..n {
                let (sn, cs) = (start + step * frac(k)).dsin_cos();
                out.push((c.0 + r * cs, c.1 + r * sn));
            }
        }
        EntKind::Spline => {
            let (lo, hi) = crate::curve::domain(sk, w.edge.i());
            let forward = sk.splines[w.edge.i()].ctrl[0] == w.from;
            for k in 0..n {
                let t = if forward { lo + (hi - lo) * frac(k) } else { hi - (hi - lo) * frac(k) };
                out.push(crate::curve::point_at(sk, w.edge.i(), t));
            }
        }
        _ => {
            let (lo, hi) = sk.curve_domain(w.edge.i());
            let forward = sk.curves[w.edge.i()].trim.is_none_or(|t| t.from == w.from);
            for k in 0..n {
                let t = if forward { lo + (hi - lo) * frac(k) } else { hi - (hi - lo) * frac(k) };
                out.push(sk.curve_point(w.edge.i(), t));
            }
        }
    }
}

/// What the family reads off the part: its face, its running number's parameter, the loops as
/// walked and the names the walk gives them.
struct Reading {
    face: usize,
    held: u32,
    loops: Vec<Vec<Walked>>,
    /// The unknowns of the drawing the part was handed, held as they stand at every station.
    outer: Vec<String>,
}

impl Reading {
    /// The station the part stands at now, its loops sampled `counts` times an edge.
    fn station(&self, sk: &Sketch, s: f64, counts: &[Vec<usize>]) -> Result<Station, String> {
        let plane = sk.faces[self.face].plane()?;
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
        let x = sk.params.iter().map(|p| p.value).collect();
        Ok(Station { s, basis, page, world, x, limit: false })
    }

    /// The samples each edge needs here.
    fn counts(&self, sk: &Sketch, tol: f64) -> Vec<Vec<usize>> {
        self.loops.iter().map(|l| l.iter().map(|w| needed(sk, w, tol)).collect()).collect()
    }

    /// Solve the station at `s`, warm from wherever the part stands: `Err` saying why it is not
    /// regular — it does not solve, or solves with a freedom or an equation lost.
    fn settle(&self, sk: &mut Sketch, s: f64) -> Result<(), String> {
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
        let r = sys.solve(sk, crate::solve::SolveOpts::default());
        if !r.success {
            return Err("does not solve".into());
        }
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
    for (p, &v) in sk.params.iter_mut().zip(&st.x) {
        p.value = v;
    }
}

/// The Lagrange weights of the stations at `at` read at `s`: the polynomial through them.
fn weights(at: &[f64], s: f64) -> Vec<f64> {
    (0..at.len()).map(|i| {
        (0..at.len()).filter(|&j| j != i).map(|j| (s - at[j]) / (at[i] - at[j])).product()
    }).collect()
}

/// The polynomial through stations `from`, read at `s`: every sample and the plane, a station no
/// solve stood at.  `limit` says it is an end read so (`extrapolate`).
fn through(from: &[&Station], s: f64, limit: bool) -> Result<Station, String> {
    let w = weights(&from.iter().map(|st| st.s).collect::<Vec<_>>(), s);
    let mix = |pick: &dyn Fn(&Station) -> V| -> V {
        std::array::from_fn(|k| from.iter().zip(&w).map(|(st, w)| w * pick(st)[k]).sum())
    };
    let world: Vec<Vec<V>> = (0..from[0].world.len()).map(|l| {
        (0..from[0].world[l].len()).map(|i| mix(&|st| st.world[l][i])).collect()
    }).collect();
    let o = mix(&|st| st.basis.o);
    let basis = Basis::explicit(mix(&|st| st.basis.u), mix(&|st| st.basis.v))
        .map(|bs| Basis { o, ..bs })
        .ok_or("its plane comes out of no direction")?;
    let on_page = |p: V| {
        let q = sub(p, o);
        (dot(q, basis.u), dot(q, basis.v))
    };
    let page = world.iter().map(|r| r.iter().map(|&p| on_page(p)).collect()).collect();
    let x = from.last().map_or(Vec::new(), |st| st.x.clone());
    Ok(Station { s, basis, page, world, x, limit })
}

/// How far the samples of `st` stand from where the cubic through the four stations before it
/// carries them: an estimate, by the ratio of a cubic's extrapolation error to its interpolation
/// error, of how far a cubic through four such stations strays between them.
fn cubic_error(before: &[&Station], st: &Station) -> f64 {
    let Ok(guess) = through(before, st.s, false) else { return f64::INFINITY };
    let off = guess.world.iter().flatten().zip(st.world.iter().flatten())
        .map(|(p, q)| norm(sub(*p, *q))).fold(0.0f64, f64::max);
    // extrapolating a step past four stations a step apart errs by h⁴|P⁗|; interpolating within
    // the middle of them, by 9/384 of that
    off * 9.0 / 384.0
}

/// The stations from `start` (solved, the part standing there) on to the end `to`, each warm from
/// where the stations before it carry it, and as far apart as a cubic through four of them stays
/// within `tol` between them; `before` the station on the other side of `start`, where there is
/// one.
#[allow(clippy::too_many_arguments)]
fn run(
    sk: &mut Sketch,
    rd: &Reading,
    start: &Station,
    before: Option<&Station>,
    to: f64,
    span: f64,
    counts: &[Vec<usize>],
    tol: f64,
) -> Result<Vec<Station>, String> {
    let dir = (to - start.s).signum();
    let (coarsest, finest) = (span * COARSEST, span * FINEST);
    let mut h = span * FIRST_STEP;
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
        if closing && left <= span * LIMIT_GAP {
            let n = trail.len();
            if n < 3 {
                return Err("its end does not solve, and no station stands near enough to read \
                            it from".into());
            }
            trail.push(through(&[&trail[n - 3], &trail[n - 2], &trail[n - 1]], to, true)?);
            break;
        }
        let step = if closing { left / 2.0 } else { h.min(left) };
        let s = if step >= left { to } else { prev.s + dir * step };
        // the solve starts where the last two stations carry it
        restore(sk, prev);
        if let [.., a, b] = trail.as_slice() {
            let k = (s - b.s) / (b.s - a.s);
            for ((p, &xa), &xb) in sk.params.iter_mut().zip(&a.x).zip(&b.x) {
                if !p.fixed {
                    p.value = xb + k * (xb - xa);
                }
            }
        }
        match rd.settle(sk, s) {
            Ok(()) => {
                let st = rd.station(sk, s, counts)?;
                let n = trail.len();
                if n >= 4 && !closing {
                    let last: Vec<&Station> = trail[n - 4..].iter().collect();
                    let e = cubic_error(&last, &st);
                    if e > tol && step > finest {
                        h = step / 2.0;
                        continue;
                    }
                    let grow = if e > 0.0 { (tol / e).powf(0.25) * 0.9 } else { 2.0 };
                    h = (step * grow.clamp(0.5, 2.0)).clamp(finest, coarsest);
                } else if !closing {
                    h = (step * 1.5).min(coarsest);
                }
                trail.push(st);
            }
            // only an end may lose rank: it is read as the limit of the stations closing in
            Err(_) if s == to && !closing => closing = true,
            Err(why) if closing => {
                let (to, s) = (crate::syntax::num(to), crate::syntax::num(s));
                return Err(format!("the stations closing in on its end at {to} do not solve: the \
                                    station at {s} {why}"))
            }
            Err(_) if step > finest => h = step / 2.0,
            Err(why) => return Err(format!("the station at {} {why}", crate::syntax::num(s))),
        }
    }
    Ok(trail.split_off(skip))
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
    let (mut runs, mut twists): (Vec<f64>, Vec<f64>) = (Vec::with_capacity(n - 1), Vec::new());
    for k in 0..n - 1 {
        let (a, b) = (&solved[k], &solved[k + 1]);
        let c = if k + 2 < n { &solved[k + 2] } else { &solved[k - 1] };
        let (h, mut run, mut worst) = (b.s - a.s, 0.0f64, 0.0f64);
        for l in 0..a.world.len() {
            let m = a.world[l].len();
            for i in 0..m {
                // the run's second difference, and the quad's twist to the next sample
                let (pa, pb, pc) = (a.world[l][i], b.world[l][i], c.world[l][i]);
                let (h1, h2) = (b.s - a.s, c.s - b.s);
                let d2: V = std::array::from_fn(|q| {
                    2.0 * ((pc[q] - pb[q]) / h2 - (pb[q] - pa[q]) / h1) / (h1 + h2)
                });
                run = run.max((norm(d2) * h * h / 8.0 / tol).sqrt());
                // how far the quad's fourth corner stands off the plane of the other three: a
                // twisted wall, which its fan meets an eighth of that from the ruled patch
                let j = (i + 1) % m;
                let (qa, qb, qc, qd) = (a.world[l][i], a.world[l][j], b.world[l][j], b.world[l][i]);
                let along = cross(sub(qb, qa), sub(qd, qa));
                let twist = sub(sub(qc, qb), sub(qd, qa));
                if let Some(n) = plane::unit(along) {
                    worst = worst.max(dot(twist, n).abs() / (8.0 * tol));
                }
            }
        }
        runs.push(run.ceil().max(1.0));
        twists.push(worst.ceil().max(1.0));
    }
    // the runs' steps are owed, to `MAX_STATIONS`; the twist's are taken within the budget, each
    // interval's cut in proportion past it
    let owed: f64 = runs.iter().sum();
    let fit = |want: &[f64], room: f64| {
        let total: f64 = want.iter().sum();
        if total > room { room / total } else { 1.0 }
    };
    let runs_scale = fit(&runs, MAX_STATIONS as f64);
    let room = ((FACETED / samples.max(1)) as f64).max(owed.min(MAX_STATIONS as f64));
    let twist_scale = fit(&twists, room);
    let mut out = Vec::new();
    for k in 0..n - 1 {
        out.push(solved[k].clone());
        let m = (runs[k] * runs_scale).max(twists[k] * twist_scale).floor().max(1.0) as usize;
        let near: Vec<&Station> = {
            let lo = k.saturating_sub(1).min(n - 4);
            solved[lo..lo + 4].iter().collect()
        };
        for j in 1..m {
            let s = solved[k].s + (solved[k + 1].s - solved[k].s) * j as f64 / m as f64;
            let mut st = through(&near, s, false)?;
            st.x.clear();
            out.push(st);
        }
    }
    out.push(solved[n - 1].clone());
    Ok(out)
}

/// **The family stepped** (see the module): the part, the drawn station, the runs up to the end and
/// down to the start, the samples settled, and each station judged.
fn march(sk: &Sketch, si: usize, unit: f64) -> Result<Family, String> {
    let SolidDef::Family { face, station, held, from, to } = &sk.solids[si].def else {
        return Err("not a face family".into());
    };
    let (a, b) = (from.value, to.value);
    if !(a.is_finite() && b.is_finite() && a < b) {
        return Err("a face family needs a finite interval running forward".into());
    }
    let prefix = format!("{}.", held.rsplit_once('.').map_or("", |(p, _)| p));
    let mut part = part_of(sk, *face, station, &prefix);
    let pf = part.entity_in(EntRef::face(*face as usize))
        .ok_or("the station's face is not in its part")?.i();
    let held_param = part.sketch.held.get(held).ok_or("its running number is not held")?.param;
    let (loops, names) = walk_face(&part.sketch, pf, &prefix)?;
    let outer = part.sketch.free_vars.keys().filter(|n| !n.starts_with(&prefix)).cloned().collect();
    let rd = Reading { face: pf, held: held_param, loops, outer };
    let tol = crate::curve::flatness(unit);
    let span = b - a;
    // the drawn station, held to the rule every other is
    let ps = &mut part.sketch;
    let mid = ps.params[held_param as usize].value;
    rd.settle(ps, mid).map_err(|why| format!("the station at {} {why}", crate::syntax::num(mid)))?;
    let mut counts = rd.counts(ps, tol);
    let centre = rd.station(ps, mid, &counts)?;
    let up = run(ps, &rd, &centre, None, b, span, &counts, tol)?;
    restore(ps, &centre);
    let down = run(ps, &rd, &centre, up.first(), a, span, &counts, tol)?;
    let mut stations: Vec<Station> =
        down.into_iter().rev().chain(std::iter::once(centre)).chain(up).collect();
    // every edge sampled as finely as its finest station needs, and each station read there again
    let mut finer = counts.clone();
    for st in stations.iter().filter(|st| !st.limit) {
        restore(ps, st);
        for (f, c) in finer.iter_mut().flatten().zip(rd.counts(ps, tol).into_iter().flatten()) {
            *f = (*f).max(c);
        }
    }
    if finer != counts {
        counts = finer;
        for st in stations.iter_mut().filter(|st| !st.limit) {
            restore(ps, st);
            *st = rd.station(ps, st.s, &counts)?;
        }
        let n = stations.len();
        if stations[0].limit && n >= 4 {
            let near = [&stations[3], &stations[2], &stations[1]];
            stations[0] = through(&near, a, true)?;
        }
        if stations[n - 1].limit && n >= 4 {
            let near = [&stations[n - 4], &stations[n - 3], &stations[n - 2]];
            stations[n - 1] = through(&near, b, true)?;
        }
    }
    let steps = rd.loops.iter().zip(&counts).map(|(walked, n)| {
        walked.iter().zip(n).flat_map(|(w, &k)| {
            let curved = !matches!(w.edge.kind, EntKind::Line);
            std::iter::repeat_n((w.wall, curved), k)
        }).collect()
    }).collect();
    let mut fam = Family { stations, steps, names, forward: true, near: tol * 1e-3 };
    fam.judge(tol)?;
    let samples = fam.steps.iter().map(Vec::len).sum();
    fam.stations = densify(std::mem::take(&mut fam.stations), tol, samples)?;
    fam.ahead(tol)?;
    Ok(fam)
}

impl Family {
    /// A station's loop as a polygon on its plane's page, coincident samples run together: what
    /// an end whose edge has vanished comes to.  An end's are run together within the sheet's
    /// flatness, where an edge solved to nothing stands as near nothing as the solve leaves it.
    fn poly(&self, k: usize, l: usize) -> FacePoly {
        let st = &self.stations[k];
        let near = if k == 0 || k + 1 == self.stations.len() { self.near * 1e3 } else { self.near };
        let mut pts: Vec<(f64, f64)> = Vec::new();
        let mut of = Vec::new();
        let ring = &st.page[l];
        for (i, &p) in ring.iter().enumerate() {
            if pts.last().is_some_and(|q: &(f64, f64)| (q.0 - p.0).dhypot(q.1 - p.1) <= near) {
                continue;
            }
            pts.push(p);
            of.push((self.steps[l][i].0, self.steps[l][i].1));
        }
        let apart = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).dhypot(a.1 - b.1);
        while pts.len() > 1 && apart(pts[0], pts[pts.len() - 1]) <= near {
            pts.pop();
            of.pop();
        }
        FacePoly { pts, of, names: self.names.clone(), basis: st.basis, curved: Vec::new() }
    }

    /// Hold every solved station to the station's topology (see the module).
    fn judge(&mut self, tol: f64) -> Result<(), String> {
        let n = self.stations.len();
        if n < 2 {
            return Err("a face family needs more than one station".into());
        }
        let at = |s: f64| crate::syntax::num(s);
        let loops = self.steps.len();
        // which way each loop runs at the drawn station, and which way the family moves there
        let centre = self.stations.iter().position(|st| !st.limit).unwrap_or(0);
        let wind: Vec<f64> = (0..loops).map(|l| self.poly(centre, l).area().signum()).collect();
        let area_tol = tol * tol;
        for k in 0..n {
            let end = k == 0 || k == n - 1;
            let s = self.stations[k].s;
            let polys: Vec<FacePoly> = (0..loops).map(|l| self.poly(k, l)).collect();
            for (l, p) in polys.iter().enumerate() {
                let what = if l == 0 { "its section".to_string() } else { format!("its hole {l}") };
                // an end may lose an edge, or the whole section to its apex; nothing else may
                if end && (p.pts.len() < 3 || p.area().abs() <= area_tol) {
                    continue;
                }
                if !p.valid() {
                    return Err(format!("{what} crosses itself or collapses at the station at {}",
                                       at(s)));
                }
                if p.area().signum() != wind[l] {
                    return Err(format!("{what} turns over at the station at {}", at(s)));
                }
                if l > 0 && polys[0].pts.len() >= 3 {
                    let q = &polys[0];
                    if section::loops_touch(q, p) || !section::inside_loop(q, p.pts[0]) {
                        return Err(format!("{what} touches or leaves the section at the station \
                                            at {}", at(s)));
                    }
                }
            }
            // an edge lost inside the interval, or turned over from one station to the next
            for l in 0..loops {
                for (wall, from, to) in self.edges(l) {
                    let chord = |st: &Station| {
                        sub(st.world[l][to % st.world[l].len()], st.world[l][from])
                    };
                    let here = chord(&self.stations[k]);
                    let length = self.length(k, l, from, to);
                    let name = &self.names[wall];
                    if !end && length <= tol {
                        return Err(format!("the edge `{name}` vanishes at the station at {}: only \
                                            an end may lose an edge", at(s)));
                    }
                    if k > 0 && length > tol && self.length(k - 1, l, from, to) > tol
                        && dot(here, chord(&self.stations[k - 1])) < 0.0 && to - from == 1 {
                        let before = at(self.stations[k - 1].s);
                        return Err(format!("the edge `{name}` turns over between the stations at \
                                            {before} and {}", at(s)));
                    }
                }
            }
        }
        Ok(())
    }

    /// The swept volume may not cross itself: each station wholly ahead of the one before, on the
    /// side the family moves to — and which side that is.
    fn ahead(&mut self, tol: f64) -> Result<(), String> {
        let n = self.stations.len();
        let at = |s: f64| crate::syntax::num(s);
        let centre = self.stations.iter().position(|st| !st.limit).unwrap_or(0);
        let moved = |k: usize| -> f64 {
            let (b, next) = (&self.stations[k].basis, &self.stations[k + 1]);
            let pts: Vec<&V> = next.world.iter().flatten().collect();
            let along: f64 = pts.iter().map(|&&p| dot(b.normal(), sub(p, b.o))).sum();
            along / pts.len().max(1) as f64
        };
        let step = centre.min(n - 2);
        let ahead = moved(step);
        if ahead.abs() <= tol {
            return Err("its section does not move off its own plane: a face family sweeps a solid \
                        only where its stations stand one behind the next".into());
        }
        self.forward = ahead > 0.0;
        let sign = if self.forward { 1.0 } else { -1.0 };
        for k in 0..n - 1 {
            let (here, next) = (&self.stations[k], &self.stations[k + 1]);
            let (nh, nn) = (here.basis.normal(), next.basis.normal());
            let gap = tol * 1e-3;
            let behind = next.world.iter().flatten()
                .any(|&p| sign * dot(nh, sub(p, here.basis.o)) <= gap)
                || here.world.iter().flatten()
                    .any(|&p| sign * dot(nn, sub(p, next.basis.o)) >= -gap);
            if behind {
                let (a, b) = (at(here.s), at(next.s));
                return Err(format!("the swept volume crosses itself between the stations at {a} \
                                    and {b}: the section reaches past its path's centre of \
                                    curvature, or turns back"));
            }
        }
        Ok(())
    }

    /// Each edge of loop `l`: its wall, the sample it starts at and the one past its last.
    fn edges(&self, l: usize) -> Vec<(usize, usize, usize)> {
        let steps = &self.steps[l];
        let mut out: Vec<(usize, usize, usize)> = Vec::new();
        for (i, &(wall, _)) in steps.iter().enumerate() {
            match out.last_mut() {
                Some(e) if e.0 == wall && e.2 == i => e.2 = i + 1,
                _ => out.push((wall, i, i + 1)),
            }
        }
        out
    }

    /// How long loop `l`'s run from sample `from` to `to` is at station `k`, along its samples.
    fn length(&self, k: usize, l: usize, from: usize, to: usize) -> f64 {
        let ring = &self.stations[k].world[l];
        (from..to).map(|i| norm(sub(ring[(i + 1) % ring.len()], ring[i]))).sum()
    }

    /// The family faceted, in coordinates about `origin`: the two ends capped (`start`, `end`), a
    /// vanished one left open to its apex, and each wall the quads between neighbouring stations'
    /// samples, split where they do not lie flat.
    pub(crate) fn primitive(&self, origin: V, name: &str) -> Prim {
        let tol = self.near * 1e3;
        let mut faces: Vec<String> = vec!["start".into(), "end".into()];
        faces.extend(self.names.iter().cloned());
        let mut facets: Vec<Facet> = Vec::new();
        let near = self.near;
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
        let centre = self.stations.iter().position(|st| !st.limit).unwrap_or(0);
        for (k, face, reverse) in [(0, 0, self.forward), (last, 1, !self.forward)] {
            let st = &self.stations[k];
            let polys: Vec<FacePoly> = (0..loops).map(|l| self.poly(k, l)).collect();
            if polys[0].pts.len() < 3 || polys[0].area().abs() <= 0.0 {
                continue;
            }
            let rings: Vec<&FacePoly> = polys.iter().filter(|p| p.pts.len() >= 3).collect();
            for quad in super::loft::cap(&rings) {
                let pts = quad.iter().map(|&(x, y)| st.basis.lift(x, y)).collect();
                push(pts, face, false, reverse, true);
            }
        }
        for l in 0..loops {
            let cw = self.poly(centre, l).area() < 0.0;
            let reverse = ((!self.forward) != (l > 0)) != cw;
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
