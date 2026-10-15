//! **A station that states a rate** (#339 D3): a face family whose station reads a rate along its
//! running number (`rate(x over s) perpendicular y`, `model::RateRow`) is **integrated** from the
//! drawn station, its home, in both directions, instead of solved again at every station.
//!
//! The station's unknowns `x` are the state.  At a running number `s` they satisfy the station's
//! rows `g(x, s) = 0` — every row but those stated `initially`, which hold at the home alone — and
//! its rate rows `R(x)·x′ = 0`.  Differentiated along `s`, the rows are `G·x′ + g_s = 0`, `G` their
//! Jacobian and `g_s` how they move with the running number: a pin to it (`t == s`) its own column,
//! a number written over it (`distance(s * 10mm, …)`) by its free twin, the number's derivative
//! along `s` read off the number's text as a tape (`tape::derivatives_along`) — nothing is
//! differenced.  Stacked, `[G; R]·x′ = [−g_s; 0]` is the station's rate, wherever the stack is of
//! full column rank: that rank is the **index check**, asked at every evaluation, and a place where
//! it fails is refused there (`ode::Refusal`), as an inside station that loses rank is.  At the
//! home a stack short of rank is a station short of rate rows, reported with the freedom it
//! leaves.
//!
//! The flow is integrated by `ode::march` (Gragg–Bulirsch–Stoer, `ode::TOL`) from the home to each
//! end, and a station anywhere is one more step from the node before (`ode::Path::at`), so the
//! family's stepping and its exact walls read the integrated station where they solved one before
//! — and L2's check of a piece against a station between its nodes is a check against one more
//! integration.  An integrated station has one value at each place: there is no branch to carry.

use crate::constraints::{Arg, SpecKind};
use crate::linalg::{min_norm_lstsq, Mat};
use crate::model::{EntKind, EntRef, RateRow, RateWord, Sketch};
use crate::ode;
use crate::system::System;
use crate::syntax::num;

type V = [f64; 3];

/// How far the stacked rates may leave their rows unmet, against the rows' own size, before the
/// rows are said to disagree about the station's rate.
const CONSISTENT: f64 = 1e-8;

/// The pivot below which the stacked rows (each of unit length) are short of rank.
const INDEX_TOL: f64 = 1e-9;

/// How far a station read off its flow may leave its rows, each in its own units, before a
/// Newton step brings it back (`Rated::meet`), and how many steps are taken.
const MET: f64 = 1e-13;
const MEETS: usize = 4;

/// How many columns a station's flow is extrapolated over (order twice that): fewer than
/// `ode::STEPS` offers, so its steps are shorter and a station between two of its nodes — one more
/// step from the node before (`ode::Path::at`) — costs a third of the evaluations.
const COLUMNS: usize = 4;

/// Whether `sk` states a rate along the running number `held`.
pub(crate) fn states_rates(sk: &Sketch, held: &str) -> bool {
    sk.rates.iter().any(|r| r.over == held)
}

/// A number of the station written over the running number: constraint `ci`'s argument `ai`, of
/// slot `kind`, its text compiled over the held numbers.
struct Lever {
    ci: usize,
    ai: usize,
    kind: SpecKind,
    text: String,
    tape: Vec<f64>,
}

/// Every number of a station written over its running number, and what their tapes read: the held
/// numbers' parameters in name order, the running number `at` among them and `held` its own.
struct Levers {
    list: Vec<Lever>,
    vars: Vec<u32>,
    at: usize,
    held: u32,
}

impl Levers {
    /// The numbers of `sk` written over the running number `held`, or why one cannot be followed
    /// along it: only a dimension has a column to lean on (its free twin), one a statement.
    fn of(sk: &Sketch, held: &str) -> Result<Levers, String> {
        let names: Vec<String> = sk.held.keys().cloned().collect();
        let at = names.iter().position(|n| n == held).ok_or("its running number is not held")?;
        let vars: Vec<u32> = names.iter().map(|n| sk.held[n].param).collect();
        let mut list: Vec<Lever> = Vec::new();
        for (ci, c) in sk.constraints.iter().enumerate().filter(|(_, c)| c.states_rows()) {
            for (ai, &(_, kind)) in c.kind.spec().iter().enumerate() {
                let Arg::Expr(e) = &c.args[ai] else { continue };
                let parsed = crate::expr::parse_in(&e.text, sk.units)?;
                if !parsed.body.deps().contains(held) {
                    continue;
                }
                let formal = held.rsplit('.').next().unwrap_or(held);
                let said = format!("`{}`", e.text.replace(held, formal));
                let twin = kind.is_dimension()
                    && (c.kind.free_kernel().is_some() || c.kind.built());
                if !twin || c.free.is_some() || list.iter().any(|l| l.ci == ci) {
                    return Err(format!("its number {said} is read along its running number where \
                        no rate of it can be had: a station that states a rate reads its running \
                        number in a dimension, one a statement"));
                }
                let tape = crate::tape::Tape::compile_wide(&parsed.body, &names)
                    .map_err(|m| format!("its number {said}: {m}"))?;
                list.push(Lever { ci, ai, kind, text: e.text.clone(), tape: tape.flat });
            }
        }
        Ok(Levers { list, vars, at, held: sk.held[held].param })
    }

    /// Every number worked out at `s` in `sk`, and its constraint made its free twin over the
    /// running number there: `value = m·s + c`, `m` its derivative along the running number.
    /// `false` where one does not work out there.
    fn lean(&self, sk: &mut Sketch, s: f64) -> bool {
        let mut x: Vec<f64> = self.vars.iter().map(|&p| sk.params[p as usize].value).collect();
        x[self.at] = s;
        let mut dir = vec![0.0; x.len()];
        dir[self.at] = 1.0;
        for l in &self.list {
            let Some(d) = crate::tape::derivatives_along(&l.tape, &x, &dir) else { return false };
            if !(d[0].is_finite() && d[1].is_finite()) {
                return false;
            }
            let units = |x: f64| crate::expr::to_arg_units(l.kind, x);
            let (v, m) = (units(d[0]), units(d[1]));
            let c = &mut sk.constraints[l.ci];
            c.args[l.ai] = Arg::Expr(crate::expr::Expr::new(l.text.clone(), v));
            c.free = Some(crate::expr::Free { param: self.held, m, c: v - m * s });
        }
        true
    }
}

/// **The flow of a station**: the part's sketch without its rows of the home, its running number
/// a column of its own, and what its rate is read from.
struct Rated {
    sk: Sketch,
    sys: System,
    levers: Levers,
    /// The running number's column.
    hc: usize,
    /// The state: the system's other columns, and their parameters.
    cols: Vec<usize>,
    params: Vec<u32>,
    rows: Vec<RateRow>,
    /// Why the last rate asked for could not be had.
    why: String,
    /// The stacked rows a square solve reads, found independent where the flow first asked.
    keep: Option<Vec<usize>>,
}

impl Rated {
    /// The flow of the station `sk` stands at, solved at its home: `held` the running number's
    /// name.  Refused where a number written over the running number cannot be followed along it.
    fn new(sk: &Sketch, held: &str) -> Result<Rated, String> {
        let mut sk = sk.clone();
        sk.caches = Default::default();
        // the home's own rows are no rows of any other station
        for c in sk.constraints.iter_mut().filter(|c| c.initially) {
            c.claim = true;
        }
        let levers = Levers::of(&sk, held)?;
        sk.params[levers.held as usize].fixed = false;
        // each number over the running number made a column of it before the compile (whether
        // it works out at the home is the home's index check)
        let home = sk.params[levers.held as usize].value;
        levers.lean(&mut sk, home);
        let sys = System::new(&sk);
        let hc = sys.free.iter().position(|&p| p as u32 == levers.held)
            .ok_or("its running number reads nothing of the station")?;
        let cols: Vec<usize> = (0..sys.n_free).filter(|&c| c != hc).collect();
        let params = cols.iter().map(|&c| sys.free[c] as u32).collect();
        let rows = sk.rates.iter().filter(|r| r.over == held).cloned().collect();
        Ok(Rated { sk, sys, levers, hc, cols, params, rows, why: String::new(), keep: None })
    }

    /// The state where the part stands.
    fn state(&self, part: &Sketch) -> Vec<f64> {
        self.params.iter().map(|&p| part.params[p as usize].value).collect()
    }

    /// Each component's size, which a step's error is a fraction of: a world length over the
    /// motion one unit of the column is worth.
    fn scale(&self) -> Vec<f64> {
        let world = self.sys.extent.max(1.0);
        self.cols.iter().map(|&c| world / self.sys.col_scale[c].max(f64::MIN_POSITIVE)).collect()
    }

    /// The rate rows at the state the sketch stands at, over the state's columns in the system's
    /// units: one row a direction the rate is held square to.
    fn rate_rows(&self, out: &mut Vec<Vec<f64>>) -> Result<(), String> {
        let sk = &self.sk;
        let value = |p: u32| sk.params[p as usize].value;
        let dir = |e: EntRef| -> Option<V> {
            match e.kind {
                EntKind::Axis => crate::plane::unit(sk.axes[e.i()].d.map(value)),
                EntKind::Line => {
                    let l = &sk.lines[e.i()];
                    let (a, b) = (sk.lifted(l.p1 as usize), sk.lifted(l.p2 as usize));
                    crate::plane::unit(crate::space::sub(b, a))
                }
                EntKind::Plane => crate::plane::unit(sk.basis(e.i()).normal()),
                _ => None,
            }
        };
        for r in &self.rows {
            // the rate as a map of the state's rates: the axis's direction made unit, or the
            // point where it stands in space
            let (cols, jac): ([u32; 3], [[f64; 3]; 3]) = match r.of.kind {
                EntKind::Axis => {
                    let d = sk.axes[r.of.i()].d.map(value);
                    let n = crate::plane::norm(d);
                    if n == 0.0 {
                        return Err(format!("`{}` reads the rate of an axis of no direction",
                                           r.text));
                    }
                    let u = d.map(|q| q / n);
                    let p = std::array::from_fn(|i| std::array::from_fn(|j| {
                        (if i == j { 1.0 } else { 0.0 } - u[i] * u[j]) / n
                    }));
                    (sk.axes[r.of.i()].d, p)
                }
                _ => {
                    let p = r.of.i();
                    let cols = match (sk.points[p].z, sk.lift_of(p)) {
                        (Some(z), _) => [sk.points[p].x, sk.points[p].y, z],
                        (None, Some(k)) => sk.lifts[k].x,
                        (None, None) => {
                            return Err(format!("`{}` reads the rate of a point drawn in a plane, \
                                which is read in space only where it stands there", r.text));
                        }
                    };
                    (cols, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                }
            };
            let to = dir(r.to).ok_or_else(|| format!("`{}` is held against no direction", r.text))?;
            // square to `to` is one row along it; along it, two across it — and a plane's normal
            // the other way round
            let plane = r.to.kind == EntKind::Plane;
            let across = (r.word == RateWord::Parallel) != plane;
            let dirs: Vec<V> = if across { square_to(to).to_vec() } else { vec![to] };
            for e in dirs {
                let mut row = vec![0.0; self.cols.len()];
                for (k, &p) in cols.iter().enumerate() {
                    let Some(at) = self.params.iter().position(|&q| q == p) else { continue };
                    let w: f64 = (0..3).map(|i| e[i] * jac[i][k]).sum();
                    // over the system's units: `z = x · col_scale`
                    row[at] += w / self.sys.col_scale[self.cols[at]];
                }
                out.push(row);
            }
        }
        Ok(())
    }

    /// The system compiled again where a contact on a spline has run onto another span, whose
    /// control points its columns name (`curve::contact_spans`), as a solve compiles it again: its
    /// columns are the same parameters, and the rows found independent are found again.
    fn respan(&mut self) {
        if crate::curve::contact_spans(&self.sk) == *self.sys.spans() {
            return;
        }
        self.sys = System::new(&self.sk);
        let free = &self.sys.free;
        self.hc = free.iter().position(|&p| p as u32 == self.levers.held).unwrap_or(self.hc);
        self.cols = (0..self.sys.n_free).filter(|&c| c != self.hc).collect();
        debug_assert!(self.cols.iter().map(|&c| free[c] as u32).eq(self.params.iter().copied()));
        self.keep = None;
    }

    /// The station's rate at `(s, x)`, or why there is none: the stacked rows and their rank.
    fn rate_at(&mut self, s: f64, x: &[f64], f: &mut [f64]) -> Result<(), String> {
        for (&p, &v) in self.params.iter().zip(x) {
            self.sk.params[p as usize].value = v;
        }
        self.sk.params[self.levers.held as usize].value = s;
        if !self.levers.lean(&mut self.sk, s) {
            return Err("has a number over its running number that does not work out there".into());
        }
        self.respan();
        self.sys.refresh_consts(&self.sk);
        let z = self.sys.z0(&self.sk);
        let g = self.sys.conditioned(&z);
        let g = g.as_mat();
        // `G·z′ = −g_s`, the running number's column moving at `col_scale` a unit
        let ds = self.sys.col_scale[self.hc];
        let mut rows: Vec<Vec<f64>> = (0..g.rows)
            .map(|i| self.cols.iter().map(|&c| g.at(i, c)).collect())
            .collect();
        let mut rhs: Vec<f64> = (0..g.rows).map(|i| -g.at(i, self.hc) * ds).collect();
        self.rate_rows(&mut rows)?;
        rhs.resize(rows.len(), 0.0);
        // each row of unit length, so a rank is one statement about every row
        let n = self.cols.len();
        let mut a = Mat::zeros(rows.len(), n);
        let mut b = Mat::zeros(rows.len(), 1);
        for (i, (row, &r)) in rows.iter().zip(&rhs).enumerate() {
            let len = crate::linalg::norm(row);
            if len == 0.0 {
                if r != 0.0 {
                    return Err("has a row that moves with its running number and reads nothing \
                                that can".into());
                }
                continue;
            }
            for (j, &v) in row.iter().enumerate() {
                a.set(i, j, v / len);
            }
            b.set(i, 0, r / len);
        }
        let zp = match self.square(&a, &b) {
            Some(zp) => zp,
            None => verdict(&a, &b, !self.rows.is_empty())?,
        };
        let miss = a.mul_vec(&zp).iter().zip(&b.data).map(|(p, q)| (p - q).abs())
            .fold(0.0f64, f64::max);
        if miss > CONSISTENT * (1.0 + crate::linalg::absmax(&b.data)) {
            return Err("has rows and rates that disagree about how it moves".into());
        }
        for (k, &c) in self.cols.iter().enumerate() {
            f[k] = zp[k] / self.sys.col_scale[c];
        }
        Ok(())
    }

    /// **The station's rows met where the flow put it** (#363 S2): minimum-norm Newton steps over
    /// the state, the running number held at `s`, from the state integrated there — so what the
    /// rows determine (a place on the guide, a radius by its law) is the rows' to the solve's
    /// accuracy, whatever the integration made of it across a kink in a law, and only the
    /// freedom the rates hold (a minimum-norm step has no part along it) is the flow's.
    fn meet(&mut self, s: f64, y: &[f64]) {
        for (&p, &v) in self.params.iter().zip(y) {
            self.sk.params[p as usize].value = v;
        }
        self.sk.params[self.levers.held as usize].value = s;
        if !self.levers.lean(&mut self.sk, s) {
            return;
        }
        self.respan();
        self.sys.refresh_consts(&self.sk);
        let mut z = self.sys.z0(&self.sk);
        let hard = self.sys.hard_rows();
        for _ in 0..MEETS {
            let r = self.sys.residuals(&z);
            if hard.iter().all(|&i| r[i].abs() <= MET) {
                break;
            }
            let j = self.sys.jacobian_dense(&z);
            let mut a = Mat::zeros(hard.len(), self.cols.len());
            let mut b = Mat::zeros(hard.len(), 1);
            for (i, &row) in hard.iter().enumerate() {
                for (k, &c) in self.cols.iter().enumerate() {
                    a.set(i, k, j.at(row, c));
                }
                b.set(i, 0, -r[row]);
            }
            let (d, _) = min_norm_lstsq(&a, &b, INDEX_TOL);
            if !d.data.iter().all(|v| v.is_finite()) {
                break;
            }
            for (k, &c) in self.cols.iter().enumerate() {
                z[c] += d.data[k];
            }
        }
        let x = self.sys.full_x(&z);
        for &p in &self.params {
            self.sk.params[p as usize].value = x[p as usize];
        }
    }

    /// The stacked rows solved square: over the rows found independent where the station first
    /// stood (`keep`), by an LU — `None` where those rows are not of full rank here, for the
    /// rank-revealing solve to say why (`verdict`), or where none were found yet.
    fn square(&mut self, a: &Mat, b: &Mat) -> Option<Vec<f64>> {
        let n = a.cols;
        if self.keep.is_none() {
            let (rank, piv) = crate::linalg::rrqr(&a.transpose(), INDEX_TOL);
            if rank < n {
                return None;
            }
            let mut keep: Vec<usize> = piv[..n].iter().map(|&p| p as usize).collect();
            keep.sort_unstable();
            self.keep = Some(keep);
        }
        let keep = self.keep.as_ref()?;
        let mut m = Vec::with_capacity(n * n);
        for &r in keep {
            m.extend_from_slice(a.row(r));
        }
        let mut x: Vec<f64> = keep.iter().map(|&r| b.at(r, 0)).collect();
        let mut piv = Vec::with_capacity(n);
        if !crate::linalg::lu_factor(n, &mut m, &mut piv) {
            return None;
        }
        // a pivot this far below the largest is a rank lost, which the rank-revealing solve says
        let d = (0..n).map(|k| m[k * n + k].abs());
        let (lo, hi) = d.fold((f64::INFINITY, 0.0f64), |(lo, hi), v| (lo.min(v), hi.max(v)));
        if lo <= INDEX_TOL * hi {
            return None;
        }
        crate::linalg::lu_apply(n, &m, &piv, &mut x);
        Some(x)
    }
}

/// The stacked rows `a` solved by their minimum-norm least squares, or the freedom along the
/// running number they leave — `rates` whether any of them is a rate row: the index check's
/// verdict.
fn verdict(a: &Mat, b: &Mat, rates: bool) -> Result<Vec<f64>, String> {
    let (zp, rank) = min_norm_lstsq(a, b, INDEX_TOL);
    if rank < a.cols {
        let short = a.cols - rank;
        let free = if rates { "is short of rates" } else { "states no rate" };
        let plural = if short == 1 { "" } else { "s" };
        return Err(format!("{free}: {short} degree{plural} of freedom along its running number"));
    }
    Ok(zp.data)
}

impl ode::Rate for Rated {
    type Branch = ();

    fn width(&self) -> usize {
        self.params.len()
    }

    fn rate(&mut self, u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        // no sensitivity is carried: a family's walls are read, and nothing solves against them
        if jac.is_some() {
            return false;
        }
        match self.rate_at(u, y, f) {
            Ok(()) => true,
            Err(why) => {
                self.why = why;
                false
            }
        }
    }

    fn branch(&self) {}

    fn set_branch(&mut self, _: ()) {}
}

/// Two unit directions square to the unit `d`, and to each other.
fn square_to(d: V) -> [V; 2] {
    let k = (0..3).min_by(|&a, &b| d[a].abs().total_cmp(&d[b].abs())).unwrap_or(0);
    let mut e = [0.0; 3];
    e[k] = 1.0;
    let a = crate::plane::unit(crate::plane::cross(d, e)).unwrap_or([1.0, 0.0, 0.0]);
    [a, crate::plane::cross(d, a)]
}

/// A flow shared by everything that reads one family's stations: its faceted stepping and its
/// exact walls march it once.
#[derive(Clone, Default)]
pub(crate) struct Shared(pub Option<std::rc::Rc<std::cell::RefCell<Flow>>>);

impl std::fmt::Debug for Shared {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() { "Shared(flow)" } else { "Shared(none)" })
    }
}

/// **A station's flow, marched** from its home to each end of its leg.
pub(crate) struct Flow {
    rated: Rated,
    home: f64,
    up: ode::Path<()>,
    down: ode::Path<()>,
}

impl Flow {
    /// The flow of the station `part` stands at, its home `home`, over `(from, to)`: refused at
    /// the home where it is short of rates, and anywhere its rate cannot be had or its tolerance
    /// met.
    pub(crate) fn march(part: &Sketch, held: &str, home: f64, from: f64, to: f64)
        -> Result<Flow, String> {
        let mut rated = Rated::new(part, held)?;
        let y0 = rated.state(part);
        // the index check at the home, before any step is taken
        rated.rate_at(home, &y0, &mut vec![0.0; y0.len()])
            .map_err(|why| format!("the station at {} {why}", num(home)))?;
        let scale = rated.scale();
        let settings = ode::Settings { columns: COLUMNS, ..ode::Settings::default() };
        let leg = |span: f64, rated: &mut Rated| {
            ode::march(rated, home, &y0, 0, span, &scale, &settings).map_err(|r| match r {
                ode::Refusal::Undefined { at } => {
                    format!("the station at {} {}", num(at), rated.why)
                }
                ode::Refusal::Steps { at } => format!("its flow is not followed past {} in {} \
                    steps within its tolerance", num(at), settings.steps_max),
            })
        };
        let up = leg(to - home, &mut rated)?;
        let down = leg(from - home, &mut rated)?;
        Ok(Flow { rated, home, up, down })
    }

    /// Put the station at `s` into `part`: the state integrated there, the running number moved.
    pub(crate) fn put(&mut self, part: &mut Sketch, s: f64) -> Result<(), String> {
        let path = if s >= self.home { &self.up } else { &self.down };
        let rated = &mut self.rated;
        let node = path.at(rated, s, false)
            .map_err(|r| format!("the station at {} {}", num(r.at()), rated.why))?;
        rated.meet(s, &node.y);
        for &p in &rated.params {
            part.params[p as usize].value = rated.sk.params[p as usize].value;
        }
        part.params[rated.levers.held as usize].value = s;
        Ok(())
    }
}
