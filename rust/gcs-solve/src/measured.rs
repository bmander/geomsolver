//! **An equation of integrals, read** (§6.25, `model::Measured`): `integral(density(p) over p in
//! arm) == 10kg`, `claim integral(1 over p in web) <= 42cm^3` — each integral read on the drawing
//! as the row that drives it reads it, and its two sides worked out over them and the unknowns.
//!
//! An equation is read one of two ways.  **Whole**: one integral whose body reads nothing of its
//! point but the density there, in proportion or not at all (`1`, `density(p)`, `2 * density(p)`)
//! — a number times a set's volume or mass, read over any set as its own row reads it: a frame's
//! members exactly (`beams::measure`), a prism its face through its depth (`prisms::measure`), any
//! other solid over its own tetrahedra (`bodies::measure`).  **Gauged**: anything else — a body
//! that reads its point or the drawing, a face's integral, an instance's, an equation of several
//! — every body its integrals are over filled once, as one structure, and each integral taken by
//! its elements' or its facets' quadrature (`interior::Gauge`), its nodes moved by its sheets and
//! what its bodies read read off the same columns (`hulls::bodies_hull`).
//!
//! Beside a claim's value, the error estimated in it, the change from the same reading twice as
//! coarse, as every output of a scenario has (`beams::Read::estimated`).

use crate::beams::ROUNDING;
use crate::bodies::Mesh;
use crate::constraints::InteriorRow;
use crate::expr::{Ast, Cmp};
use crate::interior::{Domain as Within, Drive, Gauge, Mover, Part, Reads, Structure, Tet10};
use crate::model::{integral_var, integrand_var, Applied, Domain, EntKind, Integral, Measured,
                   Reading, Sketch, POINT_VARS};
use crate::tape::{Scratch, Tape};
use std::collections::BTreeMap;
use std::rc::Rc;

/// **How an equation is read**: whole, its one integral a number (`scale`) times its set's volume
/// or mass (`reads`); or gauged, every integral over its bodies' filled meshes.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Path<'m> {
    Whole { reads: Reads, scale: f64, of: Whole<'m> },
    Gauged,
}

/// **What a whole is read over**: a frame's members (a beam, or a line one is drawn on), or a
/// solid.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Whole<'m> {
    Members(&'m Applied),
    Solid(usize),
}

/// How equation `m` is read, or why it cannot be: an integral over a beam or a line one is drawn
/// on reads its density alone, and nothing else is over anything but solids and faces of them.
fn path<'m>(sk: &Sketch, m: &'m Measured) -> Result<Path<'m>, String> {
    if let [i] = &m.integrals[..] {
        let over = (i.local(), &i.over);
        if let (false, Domain::Of(of @ (Applied::Beam(_) | Applied::Ent(_)))) = over {
            if let Some((reads, scale)) = weighed(i) {
                let of = match (crate::beams::members(sk, of).is_empty(), of.solid()) {
                    (false, _) => Whole::Members(of),
                    (true, Some(solid)) => Whole::Solid(solid),
                    (true, None) => {
                        return Err("it is over no beam, no member of one and no solid".into());
                    }
                };
                return Ok(Path::Whole { reads, scale, of });
            }
        }
    }
    for i in &m.integrals {
        match &i.over {
            Domain::Of(Applied::Beam(_)) => {
                return Err("an integral over a beam is of a number or its density in proportion, \
                            alone in its equation".into());
            }
            Domain::Of(Applied::Ent(e)) if e.kind != EntKind::Solid => {
                return Err(format!("an integral over {} is of a number or its density in \
                                    proportion, alone in its equation", e.kind.a()));
            }
            _ => {}
        }
    }
    Ok(Path::Gauged)
}

/// **A body that reads its point's density alone**, in proportion or not at all — affine in it
/// as written (`Ast::affine_in`, never by sampling): the measure it scales, a mass or a volume, and
/// by how much; `None` for any other body.
fn weighed(i: &Integral) -> Option<(Reads, f64)> {
    if !i.integrand.affine_in(&integrand_var(3)) {
        return None;
    }
    let tape = Tape::compile(&i.integrand, &i.vars()).ok()?;
    let mut s = Scratch::new();
    let (a, b) = (tape.eval(&[0.0; 4], &mut s).v, tape.eval(&[0.0, 0.0, 0.0, 1.0], &mut s).v);
    let slope = b - a;
    match (a.is_finite() && b.is_finite(), a == 0.0, slope == 0.0) {
        (false, ..) => None,
        (_, _, true) => Some((Reads::Volume, a)),
        (_, true, false) => Some((Reads::Mass, slope)),
        _ => None,
    }
}

/// **What the equation's integrals read, each once** (§6.25): the readings in the order first
/// named, and for each integral where each of its own readings' numbers start among them all.
fn readings(m: &Measured) -> (Vec<Reading>, Vec<Vec<usize>>) {
    let mut all: Vec<Reading> = Vec::new();
    let at = m.integrals.iter()
        .map(|i| i.reads.iter().map(|&r| Reading::slot(&mut all, r)).collect())
        .collect();
    (all, at)
}

/// **Integral `i`'s body over every reading's numbers** (`readings`): its own `#r` variables
/// renamed where they stand among them all, compiled over the point's four and all the readings'.
fn integrand(i: &Integral, starts: &[usize], width: usize) -> Result<Vec<f64>, String> {
    let mut own = 0;
    let mut rename = BTreeMap::new();
    for (r, &start) in i.reads.iter().zip(starts) {
        for t in 0..r.width() {
            rename.insert(integrand_var(POINT_VARS + own + t),
                          Ast::Var(integrand_var(POINT_VARS + start + t)));
        }
        own += r.width();
    }
    let body = i.integrand.map_vars(&|v| rename.get(v).cloned());
    let vars: Vec<String> = (0..POINT_VARS + width).map(integrand_var).collect();
    Ok(Tape::compile(&body, &vars)?.flat)
}

/// **The unknowns equation `m`'s sides read, as a row reads them**: the columns of those the solve
/// moves and their names, and each held one's value — or why one is bound to no column.
fn unknowns(sk: &Sketch, m: &Measured)
    -> Result<(Vec<u32>, Vec<String>, BTreeMap<String, Ast>), String>
{
    let (mut cols, mut names, mut held) = (Vec::new(), Vec::new(), BTreeMap::new());
    for u in &m.unknowns {
        let &q = sk.free_vars.get(u)
            .ok_or_else(|| format!("it reads `{u}`, which the solve has not bound"))?;
        let p = &sk.params[q as usize];
        if p.fixed {
            held.insert(u.clone(), Ast::Num(p.value, crate::units::Dim::SCALAR));
        } else {
            cols.push(q);
            names.push(u.clone());
        }
    }
    Ok((cols, names, held))
}

/// **The equation's two sides as a row reads them** (§6.25): tapes over its integrals and then
/// the unknowns the solve moves — the whole's integral times its `scale`, an unknown held at its
/// value written in — and those unknowns' columns.  An unknown bound to no column is refused.
pub fn sides(sk: &Sketch, m: &Measured, scale: f64)
    -> Result<(Vec<f64>, Vec<f64>, Vec<u32>), String>
{
    let (cols, names, held) = unknowns(sk, m)?;
    let first = integral_var(0);
    let map = |a: &Ast| a.map_vars(&|v| match v {
        _ if v == first && scale != 1.0 => Some(Ast::Bin(crate::expr::Op::Mul,
            Box::new(Ast::Num(scale, crate::units::Dim::SCALAR)),
            Box::new(Ast::Var(first.clone())))),
        _ => held.get(v).cloned(),
    });
    let vars: Vec<String> =
        (0..m.integrals.len()).map(integral_var).chain(names.iter().cloned()).collect();
    let lhs = Tape::compile(&map(&m.lhs), &vars)?.flat;
    let rhs = Tape::compile(&map(&m.rhs), &vars)?.flat;
    Ok((lhs, rhs, cols))
}

/// The unknowns' columns a row of `m` reads, as `sides` reads them; none where it reads none.
pub fn unknown_columns(sk: &Sketch, m: &Measured) -> Vec<u32> {
    unknowns(sk, m).map(|u| u.0).unwrap_or_default()
}

/// **The bodies filled and the hulls built for a run of equations** (§6.25): every equation over
/// the same bodies, where the drawing stands the same, reads one filling of each and one hull of
/// their sheets — a centroid's three equations, a report's every reading — each keyed by where the
/// drawing stands, so a binding between two equations fills again.
#[derive(Default)]
pub struct Fills {
    meshes: BTreeMap<(usize, u64, u64), Rc<Mesh>>,
    hulls: BTreeMap<(Vec<usize>, u64), crate::hulls::Bodies>,
}

/// Where the drawing stands, as one number: its parameters' values hashed (FNV-1a).
fn standing(sk: &Sketch) -> u64 {
    sk.params.iter().fold(0xcbf29ce484222325, |h, p| {
        (h ^ p.value.to_bits()).wrapping_mul(0x100000001b3)
    })
}

impl Fills {
    /// Solid `solid` filled `coarser` times a reading's size, once where the drawing stands.
    fn mesh(&mut self, sk: &Sketch, solid: usize, coarser: f64) -> Result<Rc<Mesh>, String> {
        let key = (solid, coarser.to_bits(), standing(sk));
        if let Some(m) = self.meshes.get(&key) {
            return Ok(m.clone());
        }
        let m = Rc::new(Mesh::filled(sk, solid, coarser)?);
        self.meshes.insert(key, m.clone());
        Ok(m)
    }

    /// The hull of `parts`' sheets at a reading's size, once where the drawing stands.
    fn hull(&mut self, sk: &Sketch, parts: &[(usize, &Mesh)])
        -> Result<&crate::hulls::Bodies, String>
    {
        let key = (parts.iter().map(|(b, _)| *b).collect(), standing(sk));
        if !self.hulls.contains_key(&key) {
            let h = crate::hulls::bodies_hull(sk, parts)?;
            self.hulls.insert(key.clone(), h);
        }
        Ok(&self.hulls[&key])
    }
}

/// **Every body equation `m`'s integrals are over, filled `coarser` times a reading's size, as
/// one structure** (§6.25): each body's elements a run of its tetrahedra, each weighed by its
/// density where an integral over it reads one, each integral a part over its runs or its face's
/// facets, the bodies' meshes and where their nodes stand, and the readings its parts' bodies read.
fn gauging(sk: &Sketch, m: &Measured, coarser: f64, fills: &mut Fills)
    -> Result<(Vec<(usize, Rc<Mesh>)>, Structure, Vec<f64>, Vec<Reading>), String>
{
    use crate::model::Property;
    use crate::prisms::property;
    let solids = m.solids();
    let (readings, starts) = readings(m);
    let width: usize = readings.iter().map(|r| r.width()).sum();
    if POINT_VARS + width > crate::tape::MAX_VARS {
        return Err(format!("its integrals read {} numbers of the drawing, and a row reads at \
                            most {}", width, crate::tape::MAX_VARS - POINT_VARS));
    }
    let reads_density = |i: &Integral| i.integrand.deps().contains(&integrand_var(3));
    if m.integrals.iter().any(|i| i.over.face().is_some() && reads_density(i)) {
        return Err("a face weighs nothing: its integral reads no density".into());
    }
    let mut bodies = Vec::new();
    let mut runs = Vec::new();
    let mut tets = Vec::new();
    let mut firsts = Vec::new();
    let mut x: Vec<f64> = Vec::new();
    for &b in &solids {
        let weighed = m.integrals.iter().any(|i| reads_density(i) && i.over.solids().contains(&b));
        let rho = if weighed { property(sk, b, Property::Rho)? } else { 0.0 };
        let mesh = fills.mesh(sk, b, coarser)?;
        let first = x.len() / 3;
        runs.push((tets.len(), mesh.tets.len()));
        tets.extend(mesh.tets.iter()
            .map(|&n| Tet10 { n: n.map(|n| n + first), e: 0.0, nu: 0.0, rho }));
        x.extend(mesh.nodes.iter().flatten());
        firsts.push(first);
        bodies.push((b, mesh));
    }
    let run_of = |b: usize| runs[solids.iter().position(|&s| s == b).expect("a body of it")];
    let mut parts = Vec::with_capacity(m.integrals.len());
    for (i, starts) in m.integrals.iter().zip(&starts) {
        let domain = match i.over.face() {
            Some(f) => {
                let solid = f.solid as usize;
                let k = solids.iter().position(|&s| s == solid).expect("a body of it");
                let on = |n: usize| n + firsts[k];
                Within::Facets(crate::bodies::face_facets(sk, solid, f, &bodies[k].1)?
                    .into_iter().map(|f| f.map(on)).collect())
            }
            None => Within::Tets(i.over.solids().into_iter().map(run_of).collect()),
        };
        parts.push(Part { domain, integrand: integrand(i, starts, width)?,
                          quadratic: i.quadratic() });
    }
    let n = x.len() / 3;
    let gauge = Gauge { parts, reads: width, ..Gauge::default() };
    let s = Structure { dim: 3, nodes: n, fixed: vec![false; 3 * n], load: vec![0.0; 3 * n],
                        weight: vec![0.0; 3 * n], tets, reads: Reads::Volume, gauge,
                        ..Structure::default() };
    Ok((bodies, s, x, readings))
}

/// **The row driving equation `index`** (`CKind::Integral`), meshed where the drawing stands at a
/// reading's size: its sides, its unknowns' count, and its structure read whole or gauged — or why
/// the solve cannot read it.
pub fn row(sk: &Sketch, index: usize) -> Result<InteriorRow, String> {
    row_in(sk, index, &mut Fills::default())
}

/// `row`, its bodies filled and its hull built once among `fills`.
pub fn row_in(sk: &Sketch, index: usize, fills: &mut Fills) -> Result<InteriorRow, String> {
    let m = sk.measured.get(index).ok_or("it names no equation the drawing states")?;
    let p = path(sk, m)?;
    let scale = match p {
        Path::Whole { scale, .. } => scale,
        Path::Gauged => 1.0,
    };
    let (lhs, rhs, unknowns) = sides(sk, m, scale)?;
    let sided = |s: &mut Structure| {
        s.gauge.lhs = lhs.clone();
        s.gauge.rhs = rhs.clone();
        s.gauge.unknowns = unknowns.len();
    };
    match p {
        Path::Whole { reads, of: Whole::Members(of), .. } => {
            let mut r = crate::beams::Row::measured(sk, of, reads)?;
            sided(&mut r.structure);
            Ok(InteriorRow::Frame(r))
        }
        Path::Whole { reads, of: Whole::Solid(solid), .. } => {
            let mut r = crate::prisms::Row::measured(sk, solid, reads)?;
            sided(&mut r.structure);
            Ok(InteriorRow::Meshed(r.into()))
        }
        Path::Gauged => {
            let (bodies, mut s, x, readings) = gauging(sk, m, 1.0, fills)?;
            sided(&mut s);
            let parts: Vec<(usize, &Mesh)> = bodies.iter().map(|(b, mesh)| (*b, &**mesh)).collect();
            let (hull, columns) = fills.hull(sk, &parts)?.clone().reading(sk, &readings)?;
            Ok(InteriorRow::Meshed(crate::prisms::Row { structure: s,
                drive: Drive::Moved(Mover::Hull(hull)), columns, x, coarser: 1.0 }.into()))
        }
    }
}

/// **What reading `r` comes to where the drawing stands**: a point's three numbers, a direction
/// made unit, or a line's first point and its direction.
fn reading_at(sk: &Sketch, r: Reading) -> Result<Vec<f64>, String> {
    use crate::space::{sub, unit};
    let line = |e: crate::model::EntRef| -> Result<([f64; 3], [f64; 3]), String> {
        match e.kind {
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
                Ok((a, unit(sub(b, a))))
            }
            EntKind::Axis => {
                let x = &sk.axes[e.i()];
                let v = |q: [u32; 3]| q.map(|q| sk.params[q as usize].value);
                Ok((v(x.a), unit(v(x.d))))
            }
            EntKind::Frame => {
                let b = sk.basis(e.i());
                Ok((b.o, unit(crate::space::cross(b.u, b.v))))
            }
            k => Err(format!("an integral reads a line, an axis or a plane here, not {}", k.a())),
        }
    };
    Ok(match r {
        Reading::Point(q) => sk.world_point(q).to_vec(),
        Reading::Direction(e) => line(e)?.1.to_vec(),
        Reading::Line(e) => {
            let (a, d) = line(e)?;
            [a, d].concat()
        }
    })
}

/// **Equation `m`'s integrals read on the drawing as it stands**, `coarser` times a reading's
/// size, each beside the integral of its body's size (`∫ |f|`) where `sized` asks for it — or why
/// it has none: a member of no length, a material of no density, a thing that is no beam, member
/// or solid.
pub fn integrals_at(sk: &Sketch, m: &Measured, coarser: f64, sized: bool, fills: &mut Fills)
    -> Result<(Vec<f64>, Vec<f64>), String>
{
    match path(sk, m)? {
        Path::Whole { reads, scale, of } => {
            let v = scale * match of {
                Whole::Members(of) => crate::beams::measure(sk, of, reads)?,
                Whole::Solid(solid) => crate::bodies::measure(sk, solid, reads, coarser)?,
            };
            Ok((vec![v], vec![v.abs()]))
        }
        Path::Gauged => {
            let (_, s, x, readings) = gauging(sk, m, coarser, fills)?;
            let mut r = Vec::new();
            for q in readings {
                r.extend(reading_at(sk, q)?);
            }
            let sizes = if sized { s.integral_sizes(&x, &r) } else { Vec::new() };
            Ok((s.integrals(&x, &r), sizes))
        }
    }
}

/// **An equation read**: its two sides, the error estimated in its left (none where the coarser
/// reading has no value), and the size its rounding is measured against.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Answer {
    pub lhs: f64,
    pub rhs: f64,
    pub error: Option<f64>,
    pub size: f64,
}

/// Its sides at integrals `ints`, the unknowns where the drawing stands, with their rates in the
/// integrals — as its row reads them (`sides`).
fn sides_at(sk: &Sketch, m: &Measured, ints: &[f64])
    -> Result<(crate::tape::Value, crate::tape::Value), String>
{
    let (lhs, rhs, cols) = sides(sk, m, 1.0)?;
    let u: Vec<f64> = cols.iter().map(|&q| sk.params[q as usize].value).collect();
    Ok(crate::interior::eval_sides(&lhs, &rhs, ints, &u))
}

/// **Equation `m` read where the drawing stands**: its sides, and the error estimated in its
/// left, the change from the reading twice as coarse — nothing where the two agree to rounding (a
/// frame's, a straight prism's), none where the coarser has no value.
pub fn read(sk: &Sketch, m: &Measured) -> Result<Answer, String> {
    read_in(sk, m, &mut Fills::default())
}

/// `read`, its bodies filled once among `fills`.
pub fn read_in(sk: &Sketch, m: &Measured, fills: &mut Fills) -> Result<Answer, String> {
    let (ints, sizes) = integrals_at(sk, m, 1.0, true, fills)?;
    let (lhs, rhs) = sides_at(sk, m, &ints)?;
    let size = lhs.v.abs() + rhs.v.abs()
        + sizes.iter().enumerate().map(|(j, s)| (lhs.d[j] - rhs.d[j]).abs() * s).sum::<f64>();
    let error = integrals_at(sk, m, 2.0, false, fills).ok()
        .and_then(|(c, _)| sides_at(sk, m, &c).ok())
        .map(|(l, _)| {
            let e = (lhs.v - l.v).abs();
            if e <= ROUNDING * lhs.v.abs().max(l.v.abs()) { 0.0 } else { e }
        });
    Ok(Answer { lhs: lhs.v, rhs: rhs.v, error, size })
}

/// **Whether `a` keeps `m`'s comparison**: its sides equal, or on their side, within the error
/// estimated in its left or a billionth of its size, whichever is more.
pub fn keeps(m: &Measured, a: &Answer) -> bool {
    let tol = (1e-9 * a.size).max(a.error.unwrap_or(0.0));
    match m.compare {
        Cmp::Eq => (a.lhs - a.rhs).abs() <= tol,
        Cmp::Le | Cmp::Lt => a.lhs <= a.rhs + tol,
        Cmp::Ge | Cmp::Gt => a.lhs >= a.rhs - tol,
        Cmp::Ne => (a.lhs - a.rhs).abs() > tol,
    }
}

/// **A side's value as a reader reads it**, six figures in its unit (`Dim::reading`): a mass in
/// kilograms, a volume in the drawing's length cubed.
pub fn value_text(sk: &Sketch, m: &Measured, v: f64) -> String {
    m.dim.reading(v, sk.units, 6)
}

/// The error estimated beside a value, to two figures: `± 0.0021 kg`.
pub fn error_text(sk: &Sketch, m: &Measured, e: f64) -> String {
    format!("± {}", m.dim.reading(e, sk.units, 2))
}
