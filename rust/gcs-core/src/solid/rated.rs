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
use crate::model::{EntKind, EntRef, RateWord, Sketch};
use crate::ode;
use crate::system::System;
use crate::syntax::num;

type V = [f64; 3];

/// How far the stacked rates may leave their rows unmet, against the rows' own size, before the
/// rows are said to disagree about the station's rate.
const CONSISTENT: f64 = 1e-8;

/// The pivot below which the stacked rows (each of unit length) are short of rank.
const INDEX_TOL: f64 = 1e-9;

/// A number of the station written over the running number: constraint `ci`'s argument `ai`, of
/// slot `kind`, its text compiled over the held numbers (`Sketch::held`, in name order).
struct Lever {
    ci: usize,
    ai: usize,
    kind: SpecKind,
    text: String,
    tape: Vec<f64>,
}

/// A rate row as the state reads it: whose rate, against what, and how.
struct Row {
    of: EntRef,
    word: RateWord,
    to: EntRef,
    text: String,
}

/// **The flow of a station**: the part's sketch without its rows of the home, its running number
/// a column of its own, and what its rate is read from.
pub(crate) struct Rated {
    sk: Sketch,
    sys: System,
    /// The running number's parameter, and its column.
    held: u32,
    hc: usize,
    /// The state: the system's other columns, and their parameters.
    cols: Vec<usize>,
    params: Vec<u32>,
    levers: Vec<Lever>,
    /// The held numbers' parameters, in the tapes' variable order, and where the running number
    /// is among them.
    vars: Vec<u32>,
    at: usize,
    rows: Vec<Row>,
    /// Why the last rate asked for could not be had.
    why: String,
    /// The stacked rows a square solve reads, found independent where the flow first asked.
    keep: Option<Vec<usize>>,
}

/// The rate rows of `sk` over the running number `held`.
pub(crate) fn states_rates(sk: &Sketch, held: &str) -> bool {
    sk.rates.iter().any(|r| r.over == held)
}

impl Rated {
    /// The flow of the station `sk` stands at, solved at its home: `held` the running number's
    /// name.  Refused where a number written over the running number cannot be followed along it.
    pub(crate) fn new(sk: &Sketch, held: &str) -> Result<Rated, String> {
        let mut sk = sk.clone();
        sk.caches = Default::default();
        let h = *sk.held.get(held).ok_or("its running number is not held")?;
        // the home's own rows are no rows of any other station
        for c in sk.constraints.iter_mut().filter(|c| c.initially) {
            c.claim = true;
        }
        let names: Vec<String> = sk.held.keys().cloned().collect();
        let at = names.iter().position(|n| n == held).expect("the running number is held");
        let vars: Vec<u32> = names.iter().map(|n| sk.held[n].param).collect();
        let mut levers = Vec::new();
        for (ci, c) in sk.constraints.iter().enumerate() {
            if !c.states_rows() {
                continue;
            }
            for (ai, &(_, kind)) in c.kind.spec().iter().enumerate() {
                let Arg::Expr(e) = &c.args[ai] else { continue };
                let parsed = crate::expr::parse_in(&e.text, sk.units).map_err(|m| m.to_string())?;
                if !parsed.body.deps().contains(held) {
                    continue;
                }
                let said = format!("`{}`", e.text.replace(held, &public(held)));
                let twin = kind.is_dimension()
                    && (c.kind.free_kernel().is_some() || c.kind.built());
                if !twin || c.free.is_some() || levers.iter().any(|l: &Lever| l.ci == ci) {
                    return Err(format!("its number {said} is read along its running number where \
                        no rate of it can be had: a station that states a rate reads its running \
                        number in a dimension, one a statement"));
                }
                let tape = crate::tape::Tape::compile_wide(&parsed.body, &names)
                    .map_err(|m| format!("its number {said}: {m}"))?;
                levers.push(Lever { ci, ai, kind, text: e.text.clone(), tape: tape.flat });
            }
        }
        let rows = sk.rates.iter().filter(|r| r.over == held)
            .map(|r| Row { of: r.of, word: r.word, to: r.to, text: r.text.clone() })
            .collect::<Vec<_>>();
        for r in &rows {
            if r.of.kind == EntKind::Point && sk.points[r.of.i()].z.is_none()
                && sk.lift_of(r.of.i()).is_none() {
                return Err(format!("`{}` reads the rate of a point drawn in a plane, which is read \
                    in space only where it stands there", r.text));
            }
        }
        sk.params[h.param as usize].fixed = false;
        let s = sk.params[h.param as usize].value;
        let mut me = Rated { sys: System::new(&sk), sk, held: h.param, hc: 0, cols: Vec::new(),
                             params: Vec::new(), levers, vars, at, rows,
                             why: String::new(), keep: None };
        // each number over the running number made a column of it before the compile
        me.lean(s);
        me.sys = System::new(&me.sk);
        me.hc = me.sys.free.iter().position(|&p| p as u32 == h.param)
            .ok_or("its running number reads nothing of the station")?;
        me.cols = (0..me.sys.n_free).filter(|&c| c != me.hc).collect();
        me.params = me.cols.iter().map(|&c| me.sys.free[c] as u32).collect();
        Ok(me)
    }

    /// The state where the part stands.
    pub(crate) fn state(&self, part: &Sketch) -> Vec<f64> {
        self.params.iter().map(|&p| part.params[p as usize].value).collect()
    }

    /// Each component's size, which a step's error is a fraction of: a world length over the
    /// motion one unit of the column is worth.
    fn scale(&self) -> Vec<f64> {
        let world = self.sys.extent.max(1.0);
        self.cols.iter().map(|&c| world / self.sys.col_scale[c].max(f64::MIN_POSITIVE)).collect()
    }

    /// Every number written over the running number worked out at `s`, and made its column's
    /// twin there: `value = m·s + c`, `m` its derivative along the running number.
    fn lean(&mut self, s: f64) -> bool {
        let x: Vec<f64> = self.vars.iter().map(|&p| self.sk.params[p as usize].value).collect();
        let mut dir = vec![0.0; x.len()];
        dir[self.at] = 1.0;
        let mut x = x;
        x[self.at] = s;
        for l in &self.levers {
            let Some(d) = crate::tape::derivatives_along(&l.tape, &x, &dir) else { return false };
            if !(d[0].is_finite() && d[1].is_finite()) {
                return false;
            }
            let units = |x: f64| crate::expr::to_arg_units(l.kind, x);
            let (v, m) = (units(d[0]), units(d[1]));
            let c = &mut self.sk.constraints[l.ci];
            c.args[l.ai] = Arg::Expr(crate::expr::Expr::new(l.text.clone(), v));
            c.free = Some(crate::expr::Free { param: self.held, m, c: v - m * s });
        }
        true
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
                            return Err(format!("`{}` reads a point not in space", r.text));
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

    /// The station's rate at `(s, x)`, or why there is none: the stacked rows and their rank.
    fn rate_at(&mut self, s: f64, x: &[f64], f: &mut [f64]) -> Result<(), String> {
        for (&p, &v) in self.params.iter().zip(x) {
            self.sk.params[p as usize].value = v;
        }
        self.sk.params[self.held as usize].value = s;
        if !self.lean(s) {
            return Err("a number over its running number does not work out there".into());
        }
        self.sys.refresh_consts(&self.sk);
        let z = self.sys.z0(&self.sk);
        let g = self.sys.conditioned(&z);
        let g = g.as_mat();
        let n = self.cols.len();
        let mut rows: Vec<Vec<f64>> = Vec::with_capacity(g.rows + self.rows.len());
        let mut rhs = Vec::with_capacity(g.rows + self.rows.len());
        // `G·z′ = −g_s`, the running number's column moving at `col_scale` a unit
        let ds = self.sys.col_scale[self.hc];
        for i in 0..g.rows {
            rows.push(self.cols.iter().map(|&c| g.at(i, c)).collect());
            rhs.push(-g.at(i, self.hc) * ds);
        }
        let first = rows.len();
        self.rate_rows(&mut rows)?;
        rhs.resize(rows.len(), 0.0);
        // each row of unit length, so a rank is one statement about every row
        let mut a = Mat::zeros(rows.len(), n);
        let mut b = Mat::zeros(rows.len(), 1);
        for (i, (row, &r)) in rows.iter().zip(&rhs).enumerate() {
            let len = crate::linalg::norm(row);
            if len == 0.0 {
                if r != 0.0 {
                    return Err("a row moves with its running number and reads nothing that can"
                        .into());
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
            None => self.verdict(&a, &b, first, rows.len())?,
        };
        let miss = a.mul_vec(&zp).iter().zip(&b.data).map(|(p, q)| (p - q).abs())
            .fold(0.0f64, f64::max);
        if miss > CONSISTENT * (1.0 + crate::linalg::absmax(&b.data)) {
            return Err("its rows and its rates disagree about how it moves".into());
        }
        for (k, &c) in self.cols.iter().enumerate() {
            f[k] = zp[k] / self.sys.col_scale[c];
        }
        Ok(())
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

    /// The stacked rows solved by their minimum-norm least squares, or the freedom along the
    /// running number they leave: the index check's verdict.
    fn verdict(&self, a: &Mat, b: &Mat, first: usize, rows: usize) -> Result<Vec<f64>, String> {
        let n = a.cols;
        let (zp, rank) = min_norm_lstsq(a, b, INDEX_TOL);
        if rank < n {
            let short = n - rank;
            let free = if first == rows { "states no rate" } else { "is short of rates" };
            let plural = if short == 1 { "" } else { "s" };
            return Err(format!("{free}: {short} degree{plural} of freedom along its running \
                number"));
        }
        Ok(zp.data)
    }

    /// The index check where the station stands at its home: why its rate cannot be had there.
    pub(crate) fn check(&mut self, s: f64, x: &[f64]) -> Result<(), String> {
        let mut f = vec![0.0; x.len()];
        self.rate_at(s, x, &mut f)
    }
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

/// A held name as the source says it: the formal, without the station's key.
fn public(held: &str) -> String {
    held.rsplit('.').next().unwrap_or(held).to_string()
}

/// How many columns a station's flow is extrapolated over (order twice that): fewer than
/// `ode::STEPS` offers, so its steps are shorter and a station between two of its nodes — one more
/// step from the node before (`ode::Path::at`) — costs a third of the evaluations.
const COLUMNS: usize = 4;

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
        rated.check(home, &y0).map_err(|why| format!("the station at {} {why}", num(home)))?;
        let scale = rated.scale();
        let settings = ode::Settings { columns: COLUMNS, ..ode::Settings::default() };
        let leg = |span: f64, rated: &mut Rated| {
            ode::march(rated, home, &y0, 0, span, &scale, &settings).map_err(|r| match r {
                ode::Refusal::Undefined { at } => {
                    format!("the station at {} {}", num(at), rated.why)
                }
                ode::Refusal::Steps { at } => format!("its flow is not followed past {} in {} steps \
                    within its tolerance", num(at), settings.steps_max),
            })
        };
        let up = leg(to - home, &mut rated)?;
        let down = leg(from - home, &mut rated)?;
        Ok(Flow { rated, home, up, down })
    }

    /// Put the station at `s` into `part`: the state integrated there, the running number moved.
    pub(crate) fn put(&mut self, part: &mut Sketch, s: f64) -> Result<(), String> {
        let path = if s >= self.home { &self.up } else { &self.down };
        let node = path.at(&mut self.rated, s, false).map_err(|r| {
            format!("the station at {} has no rate", num(r.at()))
        })?;
        for (&p, &v) in self.rated.params.iter().zip(&node.y) {
            part.params[p as usize].value = v;
        }
        part.params[self.rated.held as usize].value = s;
        Ok(())
    }
}
