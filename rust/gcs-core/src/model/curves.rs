//! Compiled curve definitions, solved sampling and the model-side trace cache.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;

thread_local! {
    /// Scratch the model-side locus paths run in — a pick, a paint and a bounds query each
    /// evaluate every trace curve they touch, and building a fresh scratch per question would
    /// be an allocation per hover.
    static MODEL_LOCUS: std::cell::RefCell<crate::locus::Scratch> =
        std::cell::RefCell::new(crate::locus::Scratch::new());
}

/// How many steps a user-written curve is sampled at for measuring, picking and bounding.  A
/// B-spline is refined adaptively against its own basis; a curve written in the language has no
/// basis to refine against, so it is sampled evenly and finely enough that a pick test does not
/// lie about what the drawing shows.
pub const CURVE_STEPS: usize = 128;

/// A curve, compiled: **a point of a component, as one of the component's numeric formals
/// runs** (Solvent §6.5).  `C(u)` is where the component's own statements put the point, given
/// the formal's value `u` and the geometry the component is written over.
///
/// This is what makes an involute — or a cycloid, or a walking leg's stride — *library code*
/// rather than another entity kind with another pair of kernels: a component is written once,
/// drawn or not, and a curve is one of its points asked over an interval.  A definition names
/// the entities the component is written over, the numbers it takes besides the swept one, and
/// the swept formal; the body is compiled against one variable table, which is the swept formal
/// followed by every scalar the entity formals contribute, in `entity_params` order, then the
/// other numbers.  That order is the kernel's column order, so a tape's gradient *is* the
/// Jacobian row.  One definition serves every instance of the component asked for the same
/// point over the same formal.
#[derive(Clone, Debug)]
pub struct CurveDef {
    /// `CurveDef::key` — never a spelling.
    pub name: String,
    pub component: String,
    /// The point, by its name under the instance: `toe`, `sub.pt`.
    pub port: String,
    /// The entity formals, in order: what an instance must supply, and of what kind.
    pub formals: Vec<(String, EntKind)>,
    /// The numeric formals a drawn instance left unbound, in order: unknowns of the drawing
    /// (`leg.h`), so **columns** of the curve, after the entities' scalars — a rod's length a
    /// contact may solve for.  Part of the key: an instance giving every number shares no
    /// definition with one leaving a number free, since their kernels differ in width.
    pub columns: Vec<String>,
    /// The numeric formals other than the swept one and the columns, in order: constants.
    pub values: Vec<String>,
    /// The swept formal — what the curve runs on.
    pub param: String,
    /// Whether the swept formal is an `Angle`, so a whole turn of it may close the curve
    /// (`Sketch::curve_closed`).
    pub turns: bool,
    /// The variable table the body was compiled over: `param` first, then one name per scalar
    /// the formals contribute, then the columns, then the value parameters.  Kept so a definition can be re-read
    /// and printed.
    pub vars: Vec<String>,
    pub body: CurveBody,
    /// For a trace: each inner unknown's owner — the entity's name under the instance and
    /// which of its own scalars — which is where a **drawn** instance's pose is read off, so
    /// the trace's home is the pose on the sheet rather than the component's seeds.  Empty for
    /// a formula.
    pub pose_of: Vec<(String, usize)>,
}

impl CurveDef {
    /// What one definition is keyed by: the component, the point, the swept formal, and the
    /// numeric formals that are columns.
    pub fn key(component: &str, point: &str, swept: &str, columns: &[String]) -> String {
        if columns.is_empty() {
            format!("{component}.{point}/{swept}")
        } else {
            format!("{component}.{point}/{swept}+{}", columns.join(","))
        }
    }
}

/// How a family says where `C(u)` is: as two expressions, or as the constraints that force it.
/// The second is the Wikipedia sentence — "the curve traced by the end of a taut string as it
/// unwinds" — with the working left to the solver; see `locus`.
#[derive(Clone, Debug)]
pub enum CurveBody {
    Exprs { x: crate::tape::Tape, y: crate::tape::Tape },
    Trace(crate::locus::Locus),
    /// A tool carried by a planar motion, and the envelope it cuts (`generate.rs`).
    Envelope(crate::generate::Generated),
}

/// One curve, drawn: a definition, the entities it is written over, and the numbers it was
/// given.  It holds no parameters of its own — it *is* its expressions — so it moves exactly
/// when its arguments do.
#[derive(Clone, Debug)]
pub struct CurveE {
    pub def: u32,
    pub args: Vec<EntRef>,
    /// The drawing's unknowns standing in the definition's `columns`, by name (`leg.h`) —
    /// looked up in `free_vars` when read, as `Home::Free` is, since they are allocated after
    /// the curve is built.
    pub unknowns: Vec<String>,
    pub values: Vec<f64>,
    /// The interval *this* curve is drawn over — the piece of an involute between two circles
    /// rather than the whole spiral.
    pub domain: (f64, f64),
    /// The parameter value a trace is anchored at — the one place a block's orientation
    /// predicates are read (§6.5).  `Sketch::curve_home` reads it.
    pub home: Home,
    /// Where the anchor pose is read from, for a curve of a **drawn** instance: per inner unknown
    /// of the block, the entity on the sheet and which of its own scalars (`CurveDef::pose_of`,
    /// resolved).  Empty for an instance written in place, whose seeds start the trace.
    pub pose: Vec<(EntRef, usize)>,
    pub class: Classes,
    /// A face's stretch of another curve (`flank from p to q`, §6.8): minted by the face, the
    /// curve `of` over the parameters where `from` and `to` are held on it by their contacts, so
    /// the interval follows the solve.  `None` for a curve the document wrote.
    pub trim: Option<Trim>,
    /// The curve stands for a surface: what its tool sweeps extruded square to its view — a
    /// prism's side generating under a motion that keeps the view (`flank :=
    /// envelope(surface(prism, edge: e), under: m, …)`, §6.15, issue #70).  A point in space is
    /// `on` it by its place in the view (`CKind::PointOnExtrusion`).
    pub extrusion: bool,
}

/// Where a trimmed curve runs: along curve `of`, from point `from` to point `to`, each held on
/// it by a `PointOnCurve` whose parameter is the end of the stretch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Trim {
    pub of: u32,
    pub from: u32,
    pub to: u32,
}

/// Where a curve's trace is anchored in the parameter: a number the instance gave the swept
/// formal (or the interval's start, when it gave none), or the drawing's own unknown — a drawn
/// instance that left the formal unbound made it one (`leg.theta`), and the anchor is wherever
/// it stands.  Kept as the unknown's *name*, looked up in `free_vars` when read, since the
/// unknown is allocated after the curve is built and moves with every solve.
#[derive(Clone, Debug, PartialEq)]
pub enum Home {
    At(f64),
    Free(String),
}

/// A pose is whole or it is nothing: a list with a hole in it is not the pose the block's
/// `n_q` unknowns want, and the seeds stand in.  The one statement of the rule `cold_start`
/// relies on (`p.len() == v.n_q`), for the three seams that assemble one.
pub fn whole<T>(v: Vec<T>, n: usize) -> Vec<T> {
    if v.len() == n { v } else { Vec::new() }
}

impl Sketch {
    /// The variable vector a curve's tapes are evaluated at: the parameter, then every scalar
    /// its arguments contribute in `entity_params` order, then the numbers it was given.
    ///
    /// That order is the kernel's column order too, which is what lets a tape's gradient *be* a
    /// row of the Jacobian rather than something a kernel has to rearrange.  The unknowns a drawn
    /// instance left among its numbers are columns, so `entity_params` reads them with the
    /// arguments' scalars; the instance's own values come last precisely because they are not
    /// columns: they are constants of this curve, and the gradient in them is computed and
    /// ignored.
    pub fn curve_vars(&self, i: usize, u: f64) -> Vec<f64> {
        let mut v = Vec::with_capacity(8);
        v.push(u);
        for p in self.entity_params(EntRef::new(EntKind::Curve, i)) {
            v.push(self.params[p as usize].value);
        }
        v.extend(self.curves[i].values.iter().copied());
        v
    }

    /// Where a curve is at `u`.
    pub fn curve_point(&self, i: usize, u: f64) -> (f64, f64) {
        let d = &self.curve_defs[self.curves[i].def as usize];
        let x = self.curve_vars(i, u);
        match &d.body {
            CurveBody::Exprs { x: tx, y: ty } => {
                let mut s = crate::tape::Scratch::new();
                (tx.eval(&x, &mut s).v, ty.eval(&x, &mut s).v)
            }
            CurveBody::Trace(l) => MODEL_LOCUS.with(|s| {
                let s = &mut *s.borrow_mut();
                let pose = self.curve_pose(i);
                let anchor = crate::locus::Anchor { u: self.curve_home(i), pose: pose.as_deref() };
                let v = crate::locus::eval_flat(&l.flat, &x, anchor, s);
                (v.x, v.y)
            }),
            CurveBody::Envelope(g) => crate::generate::point(&g.flat, &x, self.curve_home(i)),
        }
    }

    /// The interval a curve is drawn over: the one written, or for a trim the parameters of its
    /// two points' contacts, from `from` to `to` — decreasing where the stretch runs against
    /// the curve's own sense.
    pub fn curve_domain(&self, i: usize) -> (f64, f64) {
        let cv = &self.curves[i];
        match cv.trim {
            Some(t) => match (self.contact_param(t.from, t.of), self.contact_param(t.to, t.of)) {
                (Some(a), Some(b)) => (a, b),
                _ => cv.domain,
            },
            None => cv.domain,
        }
    }

    /// Where along curve `curve` point `p` is held: the parameter of the first acting
    /// `PointOnCurve` between them.
    pub fn contact_param(&self, p: u32, curve: u32) -> Option<f64> {
        use crate::constraints::{Arg, CKind};
        self.constraints.iter().filter(|c| c.kind == CKind::PointOnCurve && c.acts()).find_map(|c| {
            match c.args.as_slice() {
                [Arg::Ent(a), Arg::Ent(k), Arg::Param(t), ..]
                    if a.kind == EntKind::Point && a.idx == p && k.kind == EntKind::Curve && k.idx == curve =>
                    Some(self.params[*t as usize].value),
                _ => None,
            }
        })
    }

    /// Whether a curve is **closed**: run over a whole turn of an angle (`over theta in (0,
    /// 360)`) and back where it started — a crank's coupler curve, a cam's profile.  A contact
    /// on one wraps round rather than stopping at the seam (`curve::clamp_contacts`), since the
    /// seam is where the interval was written to start, not an end of anything.  Read off the
    /// curve at both ends of the turn — two evaluations, asked only of a contact past an end: a
    /// body linear in its angle (an involute's string) comes back somewhere else and is open.
    pub fn curve_closed(&self, i: usize) -> bool {
        let cv = &self.curves[i];
        let (a, b) = cv.domain;
        if cv.trim.is_some() || !self.curve_defs[cv.def as usize].turns
            || ((b - a).abs() - 360.0).abs() > 1e-9
        {
            return false;
        }
        let (p, q) = (self.curve_point(i, a), self.curve_point(i, b));
        let size = 1.0 + p.0.abs().max(p.1.abs()).max(q.0.abs()).max(q.1.abs());
        size.is_finite() && (p.0 - q.0).dhypot(p.1 - q.1) <= 1e-9 * size
    }

    /// The view a curve is drawn in: its tool's (a generated profile's, and every point it is
    /// written over shares it), `None` on the page.
    pub fn curve_view(&self, i: usize) -> Option<usize> {
        let e = *self.curves[i].args.first()?;
        let p = match e.kind {
            EntKind::Point => e.i(),
            EntKind::Line => self.lines[e.i()].p1 as usize,
            EntKind::Circle | EntKind::Arc => self.round_center(e),
            _ => return None,
        };
        self.plane_of(p)
    }

    /// What a point in space on an extrusion reads of the curve's view (`kernels::EXTRUSION_FRAME`):
    /// its datum on the sheet — origin and rotor, the page's own where it is drawn on the page —
    /// and its basis in space.
    pub fn extrusion_frame(&self, i: usize) -> [f64; crate::kernels::EXTRUSION_FRAME] {
        let ((ox, oy), c, s, b) = match self.curve_view(i) {
            Some(v) => {
                let f = &self.planes[v].frame;
                (self.point_xy(f.origin as usize), self.params[f.c as usize].value, self.params[f.s as usize].value, self.basis(v))
            }
            None => ((0.0, 0.0), 1.0, 0.0, crate::plane::Basis::page()),
        };
        let mut k = [0.0; crate::kernels::EXTRUSION_FRAME];
        k[..4].copy_from_slice(&[ox, oy, c, s]);
        k[4..7].copy_from_slice(&b.u);
        k[7..10].copy_from_slice(&b.v);
        k[10..].copy_from_slice(&b.o);
        k
    }

    /// The parameter a curve's trace is anchored at — the drawing's unknown where the swept
    /// formal is one, else the number the instance gave.  An unknown no dimension has read yet
    /// is not allocated, and the interval's start stands in until it is.
    pub fn curve_home(&self, i: usize) -> f64 {
        let cv = &self.curves[i];
        match &cv.home {
            Home::At(u) => *u,
            Home::Free(n) => self
                .free_vars
                .get(n)
                .map(|&p| self.params[p as usize].value)
                .unwrap_or(cv.domain.0),
        }
    }

    /// The parameter at which a curve's polyline comes nearest by some measure — where a fresh
    /// contact starts: nearest a point for a contact, nearest a line for a tangency.  Over the
    /// drawn polyline, which is what a person points at, and to its resolution; the solve does
    /// the rest.
    pub fn curve_nearest_by(&self, i: usize, dist: impl Fn(f64, f64) -> f64) -> f64 {
        let (a, b) = self.curve_domain(i);
        let poly = self.curve_polyline(i);
        let n = poly.len().saturating_sub(1).max(1);
        poly.iter()
            .enumerate()
            .map(|(k, &(px, py))| (dist(px, py), k))
            .min_by(|p, q| p.0.total_cmp(&q.0))
            .map(|(_, k)| a + (b - a) * k as f64 / n as f64)
            .unwrap_or(a)
    }

    /// The pose a curve's trace is anchored at, read off the sheet — `None` for a curve whose
    /// instance is not drawn, or whose pose is not whole.
    pub fn curve_pose(&self, i: usize) -> Option<Vec<f64>> {
        let cv = &self.curves[i];
        if cv.pose.is_empty() {
            return None;
        }
        cv.pose
            .iter()
            .map(|&(e, j)| self.own_param(e, j).map(|p| self.params[p as usize].value))
            .collect()
    }

    /// The curve as a polyline, for measuring and for drawing.  Uniform in the parameter: a
    /// user-written curve has no basis to refine against, so evenly is the only honest default,
    /// and `CURVE_STEPS` is chosen fine enough that a pick test does not lie.  A trace family is
    /// one march across the domain, each sample warm-started from the last — which is also what
    /// carries its branch along the curve.
    pub fn curve_polyline(&self, i: usize) -> Vec<(f64, f64)> {
        let (a, b) = self.curve_domain(i);
        // what the polyline is a function of: the curve's variables at the interval's start
        // (the parameter first, then every coordinate it reads), the interval, the anchor and
        // the anchor pose — the same reading `curve_point` and `sweep` take
        let mut key = self.curve_vars(i, a);
        key.extend([b, self.curve_home(i)]);
        key.extend(self.curve_pose(i).unwrap_or_default());
        if let Some((k, poly)) = self.polyline_cache.borrow().get(&i) {
            if *k == key {
                return poly.clone();
            }
        }
        // a trim may run against its curve's sense: swept the curve's way, then read backwards
        let poly = if a <= b { self.curve_polyline_uncached(i, a, b) } else {
            let mut p = self.curve_polyline_uncached(i, b, a);
            p.reverse();
            p
        };
        self.polyline_cache.borrow_mut().insert(i, (key, poly.clone()));
        poly
    }

    /// The curve over its interval as chords no further than `tol` from it, with each sample's
    /// parameter: the chords doubled until each one's quarter, middle and three-quarter samples lie
    /// within `tol` of it (a cubic stretch centred on an inflection has its middle on its chord, so
    /// the quarters are asked too). A user-written curve has no basis to refine against, so uniform
    /// is the honest start. At the cap, `1 << 14` chords, the finest sampling is returned as it is.
    /// Runs from the interval's first end to its second.
    pub fn curve_polyline_within(&self, i: usize, tol: f64) -> Vec<(f64, (f64, f64))> {
        let (a, b) = self.curve_domain(i);
        let (lo, hi) = (a.min(b), a.max(b));
        let off = |p: (f64, f64), q: (f64, f64), m: (f64, f64)| {
            let (dx, dy) = (q.0 - p.0, q.1 - p.1);
            let l = dx.dhypot(dy);
            if l > 0.0 { ((m.0 - p.0) * dy - (m.1 - p.1) * dx).abs() / l } else { (m.0 - p.0).dhypot(m.1 - p.1) }
        };
        let mut n = CURVE_STEPS;
        let (samples, count) = loop {
            let fine = self.curve_sweep(i, lo, hi, 4 * n);
            let worst = (0..n).flat_map(|k| (1..4).map(move |j| (k, j)))
                .map(|(k, j)| off(fine[4 * k], fine[4 * k + 4], fine[4 * k + j]))
                .fold(0.0, f64::max);
            if worst <= tol { break (fine.into_iter().step_by(4).collect::<Vec<_>>(), n) }
            if 4 * n >= 1 << 14 { break (fine, 4 * n) }
            n *= 2;
        };
        let mut out: Vec<(f64, (f64, f64))> = samples.into_iter().enumerate()
            .map(|(k, p)| (lo + (hi - lo) * k as f64 / count as f64, p)).collect();
        if a > b { out.reverse(); }
        out
    }

    /// `n` chords of the curve over `[a, b]`, the same walk the polyline takes.
    pub(crate) fn curve_sweep(&self, i: usize, a: f64, b: f64, n: usize) -> Vec<(f64, f64)> {
        let d = &self.curve_defs[self.curves[i].def as usize];
        match &d.body {
            CurveBody::Exprs { x: tx, y: ty } => {
                let mut s = crate::tape::Scratch::new();
                (0..=n).map(|k| {
                    let x = self.curve_vars(i, a + (b - a) * k as f64 / n as f64);
                    (tx.eval(&x, &mut s).v, ty.eval(&x, &mut s).v)
                }).collect()
            }
            CurveBody::Trace(l) => MODEL_LOCUS.with(|s| {
                let pose = self.curve_pose(i);
                let anchor = crate::locus::Anchor { u: self.curve_home(i), pose: pose.as_deref() };
                crate::locus::sweep(&l.flat, &self.curve_vars(i, a), a, b, n, anchor, &mut s.borrow_mut())
            }),
            CurveBody::Envelope(g) => {
                crate::generate::sweep(&g.flat, &self.curve_vars(i, a), self.curve_home(i), a, b, n)
            }
        }
    }

    fn curve_polyline_uncached(&self, i: usize, a: f64, b: f64) -> Vec<(f64, f64)> {
        self.curve_sweep(i, a, b, CURVE_STEPS)
    }
}
