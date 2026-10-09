//! **Variational curves** (#121, #144): `rope minimizes integral(…)` over a free curve
//! `rope := curve(a, b)`.
//!
//! The curve is the solution of its energy's Euler–Lagrange equation (`extremal.rs`): a function
//! of its ends and its length, its shape solved inside it, so it adds no unknown to the drawing.
//! This module is what the drawing knows of it:
//!
//! - **The energy**: every `minimizes`/`maximizes` statement over one curve is one energy, its
//!   terms summed into the curve's definition (`CurveBody::Extremal`), compiled once per text.
//! - **What presses it**: a held point the curve is stated to pass (`peg coincident rope`, the
//!   point held) is part of its problem — a peg, a corner there — and its row is absorbed (it
//!   compiles none).  A row reading the curve and geometry still free is the drawing's, read
//!   through the curve's contract like any curve's: a free line tangent to a hanging rope moves
//!   onto it.  A row whose every other operand is held and is not a peg is refused: a held line
//!   pushed against the rope meets it at a corner, so no smooth tangency to it is a minimiser.
//! - **Its length**: the curve's own unknown (`CurveE::length`).  Held by a `length` row it is
//!   that; read by nothing that holds it, it is wherever the energy is stationary in it —
//!   transversality, `H = 0` at the end, the row the energy's first statement carries.
//! - **The verdict**: minimum, maximum, saddle or degenerate (`extremal::verdict`), or none found.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::constraints::{Arg, CKind, Constraint};
use crate::expr::Ast;
use crate::kernels::Kernel;
use crate::model::{EntKind, Sketch};
use std::collections::BTreeMap;

/// What an integrand may read: the point, and the unit tangent there.
pub const BOUND: [&str; 4] = ["p.x", "p.y", "t.x", "t.y"];

/// `text` over `p.x`, `p.y`, `t.x`, `t.y` (`flatten::values::settle_integrand`) checked: it
/// compiles, and the power of length it carries (`degree`).
pub fn integrand(text: &str, units: crate::units::Units) -> Result<u32, String> {
    crate::extremal::Lagrangian::compile(&[(1.0, text)], false, units)?;
    degree(&crate::expr::parse_in(text, units)?.body, units)
}

/// The power of length an integrand carries — `p.y` is one, a length's integrand none — so the
/// transversality row is of that degree.  Where the document names a unit the point is a length
/// (`Units::read_length`) and the integrand is dimension-checked, refused where it is no whole
/// power or mixes dimensions; where it names none every number is plain, and the degree is read
/// off how the integrand grows with the point.
fn degree(body: &Ast, units: crate::units::Units) -> Result<u32, String> {
    let at = |s: f64| -> Result<crate::expr::Aff, String> {
        let dim = units.read_length();
        let env: BTreeMap<String, crate::expr::Aff> = BOUND
            .iter()
            .zip([
                crate::expr::Aff::of_dim(1.3 * s, dim),
                crate::expr::Aff::of_dim(0.7 * s, dim),
                crate::expr::Aff::num(0.6),
                crate::expr::Aff::num(0.8),
            ])
            .map(|(k, v)| (k.to_string(), v))
            .collect();
        crate::expr::eval(body, &env).map_err(String::from)
    };
    if units.length.is_none() {
        let d = (at(2.0)?.c / at(1.0)?.c).abs().log2().round();
        return Ok(if d.is_finite() { d.clamp(0.0, 4.0) as u32 } else { 1 });
    }
    let a = at(1.0)?;
    if !a.dim.ang.is_zero() {
        return Err("an integrand is a length or a power of one, never an angle".into());
    }
    match a.dim.len.whole() {
        Some(n) if n >= 0 => Ok(n as u32),
        _ => Err("an integrand is a whole power of length".into()),
    }
}

/// Whether an energy constraint is a `maximizes`'s term.
pub fn maximizes(c: &Constraint) -> bool {
    matches!(c.args[4], Arg::Bool(true))
}

/// The definition key an energy's Lagrangian is shared under: its terms as written, signed, and
/// how many pegs its curves pass (which sets its contacts' constant widths).
fn key_of(terms: &[(f64, String)], maximize: bool, pegs: usize) -> String {
    let t: Vec<String> = terms.iter().map(|(c, s)| format!("{c}*({s})")).collect();
    format!("extremal:{}:{pegs}:{}", if maximize { "max" } else { "min" }, t.join("+"))
}

/// One free curve's energy, as `Sketch::settle_variational` last read it.
#[derive(Clone, Debug, Default)]
pub struct Energy {
    pub curve: usize,
    /// The energy's statements' constraints; the lowest carries the transversality row.
    pub members: Vec<u32>,
    pub maximize: bool,
    /// The held points it passes: the rows that state each, and the point.
    pub pegs: Vec<(u32, u32)>,
    /// The pegs' places along the curve (their contacts' own unknowns): the curve's problem says
    /// where they are, so while it has the peg they are no column of the drawing's
    /// (`Sketch::free_indices`) — kept out of the solve, never marked held, so nothing a copy or
    /// a save reads changes.
    pub held: Vec<u32>,
    /// No row holds its length: it is where the energy is stationary in it.
    pub free_len: bool,
    /// The number a row holding its length states, if one does.
    pub stated: Option<f64>,
    pub degree: u32,
}

impl Energy {
    pub fn leader(&self) -> Option<u32> {
        self.members.iter().min().copied()
    }
}

impl Sketch {
    /// Bring every free curve's energy into step with the constraints — at the seams where the
    /// expressions are (`add`, `remove`, `graft`, a document read, the end of elaboration): its
    /// definition compiled from its terms, its pegs, whether its length is free.  Returns what is
    /// refused, by constraint id: an energy over anything but a free curve, a held row pressing
    /// one that is not a peg.
    pub fn settle_variational(&mut self) -> Vec<(u32, String)> {
        let free: Vec<usize> = (0..self.curves.len()).filter(|&i| self.curve_extremal(i)).collect();
        let mut faults = Vec::new();
        let mut energies: BTreeMap<usize, Energy> =
            free.iter().map(|&i| (i, Energy { curve: i, free_len: true, ..Energy::default() })).collect();
        let mut terms: BTreeMap<usize, Vec<(f64, String)>> = BTreeMap::new();
        for c in &self.constraints {
            if c.kind != CKind::Stationary || c.claim {
                continue;
            }
            let e = c.args[0].ent();
            let Some(en) = (e.kind == EntKind::Curve).then(|| energies.get_mut(&e.i())).flatten() else {
                faults.push((c.id, "an energy states the shape of a free curve, `curve(a, b)`".into()));
                continue;
            };
            let Arg::Str(text) = &c.args[2] else { continue };
            en.members.push(c.id);
            en.maximize = maximizes(c);
            en.degree = c.args[3].num() as u32;
            terms.entry(e.i()).or_default().push((c.args[1].num(), text.clone()));
        }
        // what reads each free curve: its length, a peg, the drawing — or a held row refused
        for c in &self.constraints {
            if !c.acts() || c.kind == CKind::Stationary {
                continue;
            }
            let Some(e) = c.args.iter().find_map(|a| match a {
                Arg::Ent(e) if e.kind == EntKind::Curve && energies.contains_key(&e.i()) => Some(*e),
                _ => None,
            }) else {
                continue;
            };
            let own = self.entity_params(e);
            let aux = c.aux_params();
            let others_held = c
                .params_on(self, None)
                .iter()
                .all(|p| own.contains(p) || aux.contains(p) || self.params[*p as usize].fixed);
            let en = energies.get_mut(&e.i()).expect("a free curve");
            match c.kind {
                CKind::CurveLength if others_held => {
                    en.free_len = false;
                    en.stated = Some(c.args[1].num());
                }
                CKind::CurveLength => {}
                CKind::PointOnCurve if others_held => {
                    if let Arg::Ent(p) = c.args[0] {
                        en.pegs.push((c.id, p.idx));
                    }
                }
                _ if others_held => faults.push((
                    c.id,
                    "a held line or circle pushed against a free curve meets it at a corner, so it \
                     cannot be tangent there; a held point the curve passes is a peg"
                        .into(),
                )),
                _ => {}
            }
        }
        // each curve's definition, compiled once per energy as written
        for (&i, en) in &energies {
            let ts = terms.get(&i).cloned().unwrap_or_default();
            let key = key_of(&ts, en.maximize, en.pegs.len());
            let def = match self.curve_defs.iter().position(|d| d.name == key) {
                Some(d) => d,
                None => {
                    let lag = if ts.is_empty() {
                        None
                    } else {
                        let refs: Vec<(f64, &str)> = ts.iter().map(|(c, s)| (*c, s.as_str())).collect();
                        crate::extremal::Lagrangian::compile(&refs, en.maximize, self.units).ok()
                    };
                    self.curve_defs.push(crate::model::extremal_def(key, lag, en.pegs.len()));
                    self.curve_defs.len() - 1
                }
            };
            self.curves[i].def = def as u32;
            self.curves[i].pegs = en.pegs.iter().map(|&(_, p)| p).collect();
        }
        for en in energies.values_mut() {
            en.held = en
                .pegs
                .iter()
                .filter_map(|&(cid, _)| self.constraint(cid)?.family_contact().map(|(_, t)| t))
                .collect();
        }
        self.variational = energies.into_values().collect();
        faults
    }

    /// Seed what was seeded before the free curves had a shape — elaboration's last word on them,
    /// once each energy is known (`program::variational::settle`):
    ///
    /// - a length a row states, at its number;
    /// - a length nothing holds, where the energy is stationary in it (`H = 0` at the end,
    ///   `stationary_length`).  Near the chord the curve is taut and `H` steep, and under it no
    ///   curve is that short, so a Newton step from a guess is no seed;
    /// - a contact's place along a free curve that had no place to start (the curve had no shape
    ///   when the contact was stated: `curve_nearest_by` said NaN), where the curve now passes
    ///   nearest.
    pub fn seed_extremals(&mut self) {
        for k in 0..self.variational.len() {
            let e = &self.variational[k];
            let (i, stated) = (e.curve, e.stated);
            let Some(l) = self.curves[i].length else { continue };
            // its length where a row says, else where the energy is stationary in it — the
            // seed it was built with read its ends before they were placed
            match stated {
                Some(d) if d > 0.0 => self.params[l as usize].value = d,
                _ if e.free_len && !e.members.is_empty() => {
                    if let Some(len) = self.stationary_length(i) {
                        self.params[l as usize].value = len;
                    }
                }
                _ => {}
            }
        }
        let reseed: Vec<(usize, u32)> = (0..self.constraints.len())
            .filter_map(|j| {
                let (e, t) = self.constraints[j].family_contact()?;
                (self.curve_extremal(e.i()) && !self.params[t as usize].value.is_finite()).then_some((j, t))
            })
            .collect();
        for (j, t) in reseed {
            let c = &self.constraints[j];
            let (_, slot) = c.kind.contact_on(crate::constraints::SpecKind::Curve).expect("a curve contact");
            self.params[t as usize].value = crate::constraints::seed_param(self, c.kind, &c.args, slot);
        }
    }

    /// The length at which free curve `i`'s energy is stationary in it (`H = 0` at its end):
    /// bracketed walking up from just over the chord (through its pegs), the slack doubling, then
    /// by Illinois' regula falsi (`roots::bracketed_root`).
    fn stationary_length(&self, i: usize) -> Option<f64> {
        let k = self.extremal_consts(i)?;
        let v = self.curve_vars(i, 0.0);
        let mut path = vec![(v[1], v[2])];
        path.extend(self.curves[i].pegs.iter().map(|&p| self.point_xy(p as usize)));
        path.push((v[3], v[4]));
        let chord: f64 = path.windows(2).map(|w| (w[1].0 - w[0].0).dhypot(w[1].1 - w[0].1)).sum();
        if !(chord > 0.0) {
            return None;
        }
        let h_at = |len: f64| -> Option<f64> {
            let ends = crate::extremal::Ends::of(&[v[1], v[2], v[3], v[4], len]);
            let (lag, sh) = crate::extremal::shape_for(&k, &ends)?;
            Some(crate::extremal::shoot::at(&lag, &sh, 1.0, false)?.point.h)
        };
        let mut lo: Option<(f64, f64)> = None;
        for n in 0..24 {
            let len = chord * (1.0 + 1e-3 * 2f64.powi(n));
            let Some(h) = h_at(len) else { continue };
            match lo {
                Some((a, ha)) if h.signum() != ha.signum() => {
                    return Some(crate::roots::bracketed_root(h_at, a, ha, len, h, 1e-12 * chord));
                }
                _ => lo = Some((len, h)),
            }
        }
        None
    }

    /// The places of the pegs some free curve's problem has absorbed (`Energy::held`).
    pub fn pegged_places(&self) -> std::collections::BTreeSet<u32> {
        self.variational.iter().flat_map(|e| e.held.iter().copied()).collect()
    }

    /// The energy over free curve `i`, if one is stated.
    pub fn energy_of(&self, i: usize) -> Option<&Energy> {
        self.variational.iter().find(|e| e.curve == i && !e.members.is_empty())
    }

    /// Whether constraint `cid` is a peg some free curve's problem has absorbed.
    pub fn is_peg(&self, cid: u32) -> bool {
        self.variational.iter().any(|e| e.pegs.iter().any(|&(c, _)| c == cid))
    }
}

/* -- the length's stationarity ------------------------------------------------------------ */

/// The energy constraint `cid`'s rows: one, `H = 0` at the end, where it leads an energy whose
/// curve's length nothing holds; none otherwise.
pub fn rows(sk: &Sketch, cid: u32) -> usize {
    let Some(c) = sk.constraint(cid) else { return 0 };
    let e = c.args[0].ent();
    if e.kind != EntKind::Curve || sk.extremal_lagrangian(e.i()).is_none() {
        return 0;
    }
    match sk.energy_of(e.i()) {
        Some(en) if en.free_len && en.leader() == Some(cid) => 1,
        _ => 0,
    }
}

/// Its columns: the curve's, `a`, `b`, the length.
pub fn columns(sk: &Sketch, cid: u32) -> Vec<u32> {
    sk.constraint(cid).map(|c| sk.entity_params(c.args[0].ent())).unwrap_or_default()
}

/// Its constants: the curve's problem.
pub fn pack(sk: &Sketch, cid: u32) -> Vec<f64> {
    sk.constraint(cid).and_then(|c| sk.extremal_consts(c.args[0].ent().i())).unwrap_or_default()
}

/// Its kernel: `H` at the curve's end over `[a, b, L]`, of the integrand's degree.
pub fn kernel(sk: &Sketch, cid: u32) -> Kernel {
    let curve = sk.constraint(cid).map(|c| c.args[0].ent().i());
    let degree = curve.and_then(|i| sk.energy_of(i)).map_or(1, |e| e.degree);
    let n_const = match curve.map(|i| &sk.curve_defs[sk.curves[i].def as usize].body) {
        Some(crate::model::CurveBody::Extremal(x)) => x.n_const(),
        _ => 0,
    };
    Kernel {
        name: "transversal",
        n_res: 1,
        n_par: 5,
        n_const,
        degree,
        res: crate::extremal::transversal_res,
        jac: crate::extremal::transversal_jac,
        const_jac: None,
    }
}

/* -- the verdict -------------------------------------------------------------------------- */

/// What a stationary curve is, by the second order (`extremal::verdict`) — or that none was
/// found, where the curve's problem has no solution from where it starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Extremum {
    Minimum,
    Maximum,
    Saddle,
    /// A motion the second order cannot see either way: no verdict.
    Degenerate,
    /// No stationary shape was found.
    Unsolved,
}

impl Extremum {
    pub fn name(self) -> &'static str {
        match self {
            Extremum::Minimum => "minimum",
            Extremum::Maximum => "maximum",
            Extremum::Saddle => "saddle",
            Extremum::Degenerate => "degenerate",
            Extremum::Unsolved => "unsolved",
        }
    }

    /// The same verdict read for the energy's negation — what a `maximizes` asked about.
    fn turned(self) -> Extremum {
        match self {
            Extremum::Minimum => Extremum::Maximum,
            Extremum::Maximum => Extremum::Minimum,
            other => other,
        }
    }
}

/// Every energy statement's verdict, by constraint, as each asked it: a `maximizes` that found a
/// maximum is answered `maximum`.
pub fn extrema(sk: &Sketch) -> Vec<(u32, Extremum)> {
    let mut out = Vec::new();
    for e in &sk.variational {
        if e.members.is_empty() {
            continue;
        }
        let v = match sk.curve_shape(e.curve) {
            Some((lag, sh)) => crate::extremal::verdict(&lag, &sh, e.free_len),
            None => Extremum::Unsolved,
        };
        // the Lagrangian is negated for a `maximizes`, so its minimum is the energy's maximum
        let v = if e.maximize { v.turned() } else { v };
        out.extend(e.members.iter().map(|&m| (m, v)));
    }
    out
}

/// What energy constraint `cid`'s statement asked for: a minimum, or — `maximizes` — a maximum.
pub fn asked(sk: &Sketch, cid: u32) -> Extremum {
    match sk.constraint(cid).is_some_and(maximizes) {
        true => Extremum::Maximum,
        false => Extremum::Minimum,
    }
}
