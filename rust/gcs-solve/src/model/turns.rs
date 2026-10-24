//! Turned copies: what a `ring` is solved as (Solvent §12.3–12.4, issue #96).
//!
//! A ring's copies are entities like any other — drawn, picked, named `tip[2]`, read by a face
//! — but only the representative's (copy 0's) numbers are unknowns.  Copy `k`'s are the
//! representative's turned `k` steps of `τ/n` about the ring's centre: a **derived** parameter,
//! a fixed linear combination of parameters that are not (the representative's and the
//! centre's), so a free centre is no harder than a held one.  `System` solves the free columns
//! and folds a derived parameter's Jacobian into its bases by the chain rule (`Sketch::derived`);
//! nothing else in the sketch owns a column for it, and `settle_turns` writes its value wherever
//! a parameter vector is written.
//!
//! A turn about a **point** is in that point's view (or on the page), counter-clockwise in the
//! view's own coordinates; one about an **axis** is in space, right-handed about its direction,
//! which must be held — a free direction makes the turn nonlinear in it.  A curve owns no
//! parameter: a turned curve is evaluated as its representative and turned (`curve_point`).

use crate::fmath::Det;
use super::*;

/// Copy `copy` is the representative `rep` turned `k` steps of `τ/n` about `about`, a point or
/// an axis.  Of the same kind; points, circles, arcs and curves are recorded (the others own no
/// number a turn moves).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Turn {
    pub copy: EntRef,
    pub rep: EntRef,
    pub about: EntRef,
    pub k: u32,
    pub n: u32,
}

/// **An image** (#578): copy `copy` is `rep`, an entity of a mirrored instance's body, reflected
/// in a plane of the frame the instance stands in.  Every number an entity owns is local — a
/// point drawn in a plane its two there, one in a spatial frame its three along the frame's axes
/// — so the image is the same numbers, some negated, in the image of what the representative is
/// drawn in: a point in the image of its plane (or in the same frame, its mirrored number
/// negated), a circle's or an arc's radius its own, an image datum's axis its direction reversed
/// or not and its place its own, an image plane standing where its representative does.  Linear
/// with weights ±1 wherever the frame stands, so a free frame is no harder than a held one.
/// Points, circles, arcs, axes and planes are recorded; lines, splines and curves are made of
/// points, which are reflected themselves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mirror {
    pub copy: EntRef,
    pub rep: EntRef,
    /// Which of the copy's numbers (`Sketch::mirrored_params`, in that order) are the
    /// representative's negated: bit `k` for the `k`-th.
    pub negated: u8,
}

/// A derived parameter: its value is `Σ w·x[base]`, over parameters that are not derived.
#[derive(Clone, Debug, PartialEq)]
pub struct Derivation {
    pub param: u32,
    pub terms: Vec<(u32, f64)>,
}

impl Derivation {
    /// Its value over a parameter vector — the one sum `System` and `settle_derivation` both take,
    /// so the two agree to the bit.
    pub fn value(&self, x: &[f64]) -> f64 {
        let mut v = 0.0;
        for &(b, w) in &self.terms {
            v += w * x[b as usize];
        }
        v
    }
}

impl Turn {
    /// (cos, sin) of the turn — the core's own trigonometry, so every platform's bits agree.
    pub fn cos_sin(&self) -> (f64, f64) {
        let th = std::f64::consts::TAU * self.k as f64 / self.n as f64;
        let (s, c) = th.dsin_cos();
        (c, s)
    }
}

impl Sketch {
    /// Record a turn.  A turned arc's intrinsic rows — its ends on its circle — are the images of
    /// the representative's, so they go: every reader of the constraints, the solve's and the
    /// plan's alike, sees the representative's alone.
    pub fn turn(&mut self, t: Turn) {
        if t.copy.kind == EntKind::Arc {
            self.constraints.retain(|c| !(c.intrinsic && c.entities().contains(&t.copy)));
        }
        self.turns.push(t);
    }

    /// The turn copy `e` is, if it is one.
    pub fn turn_of(&self, e: EntRef) -> Option<&Turn> {
        self.turns.iter().find(|t| t.copy == e)
    }

    /// Record an image (#578).  Its rows are its representative's reflected, so an image arc's
    /// intrinsic rows go, as a turned arc's do, and an image axis's (its direction's length, its
    /// foot): the representative's hold for it.
    pub fn mirror(&mut self, m: Mirror) {
        if matches!(m.copy.kind, EntKind::Arc | EntKind::Axis) {
            self.constraints.retain(|c| !(c.intrinsic && c.entities().contains(&m.copy)));
        }
        self.mirrors.push(m);
    }

    /// **An image datum's axis reversed** (#578): axis `of`'s direction negated, standing where it
    /// does — no number of its own, and no row (`mirror`).
    pub fn image_axis(&mut self, of: usize, name: &str) -> usize {
        let d = self.axes[of].d.map(|p| -self.params[p as usize].value);
        let a = self.axis(d, name);
        let (copy, rep) = (EntRef::new(EntKind::Axis, a), EntRef::new(EntKind::Axis, of));
        self.mirror(Mirror { copy, rep, negated: 0b111 });
        a
    }

    /// **An image datum plane** (#578): the plane over axes `u` and `v` standing where frame `at`
    /// does — its place `at`'s, derived, its origin drawn in it at `(0, 0)` — and no row: what a
    /// point of a mirrored body's drawn in one of its frame's datums is drawn in, imaged.
    pub fn image_plane(&mut self, u: usize, v: usize, at: usize, name: &str) -> usize {
        let o = self.frames[at].chart.as_ref()
            .map_or([0.0; 3], |c| c.o.map(|q| self.params[q as usize].value));
        let op = [0, 1, 2].map(|k| {
            self.param(o[k], false, &format!("{name}.{}", ["x", "y", "z"][k])) as u32
        });
        let origin = self.point(0.0, 0.0, true, &format!("{name}.origin"));
        let pi = self.frames.len();
        self.points[origin].plane = Some(pi as u32);
        let chart = Chart { u: u as u32, v: v as u32, o: op, origin: origin as u32, normal: None,
                            planar: true };
        self.frames.push(FrameE::charted(chart));
        self.mirror(Mirror { copy: EntRef::frame(pi), rep: EntRef::frame(at), negated: 0 });
        pi
    }

    /// The image `e` is, if it is one.
    pub fn mirror_of_entity(&self, e: EntRef) -> Option<&Mirror> {
        self.mirrors.iter().find(|m| m.copy == e)
    }

    /// Whether `e` is a mirrored instance's preimage (#578): solved, and drawn as its image.
    pub fn is_preimage(&self, e: EntRef) -> bool {
        !self.mirrors.is_empty() && self.mirrors.iter().any(|m| m.rep == e)
    }

    /// **What each preimage is drawn as** (#578): its image, by the preimage — the entities a
    /// mirrored body made, and not the datums its images are drawn in.
    pub fn images(&self) -> std::collections::BTreeMap<EntRef, EntRef> {
        self.mirrors.iter().filter(|m| !matches!(m.copy.kind, EntKind::Axis | EntKind::Frame))
            .map(|m| (m.rep, m.copy)).collect()
    }

    /// Constraint `c` as it is drawn (#578): each entity it reads that is a mirrored body's
    /// preimage read as its image — `None` where it reads none.
    pub fn drawn_constraint(&self, c: &crate::constraints::Constraint,
                            images: &std::collections::BTreeMap<EntRef, EntRef>)
        -> Option<crate::constraints::Constraint> {
        use crate::constraints::Arg;
        let reads = |a: &Arg| matches!(a, Arg::Ent(e) if images.contains_key(e));
        if images.is_empty() || !c.args.iter().any(reads) {
            return None;
        }
        let mut drawn = c.clone();
        for a in &mut drawn.args {
            if let Arg::Ent(e) = a {
                if let Some(&i) = images.get(e) {
                    *e = i;
                }
            }
        }
        Some(drawn)
    }

    /// What a mirrored instance leaves undrawn (#578): every preimage, the image datums — a
    /// plane, an axis reversed — and those planes' origins, standing where the frame's own do.
    pub fn unseen_of_mirrors(&self) -> std::collections::BTreeSet<EntRef> {
        let mut out = std::collections::BTreeSet::new();
        for m in &self.mirrors {
            match m.copy.kind {
                // an image datum's representative is the frame's own, which is drawn
                EntKind::Axis | EntKind::Frame => {}
                _ => {
                    out.insert(m.rep);
                }
            }
            match m.copy.kind {
                EntKind::Axis => {
                    out.insert(m.copy);
                }
                EntKind::Frame => {
                    out.insert(m.copy);
                    if let Some(c) = &self.frames[m.copy.i()].chart {
                        out.insert(EntRef::point(c.origin as usize));
                    }
                }
                _ => {}
            }
        }
        out
    }

    /// The numbers an image's derivation pairs, in order: a point's two or three, a circle's or an
    /// arc's radius, an axis's direction then its place, a plane's place.
    pub fn mirrored_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Point => self.point_all_params(e.i()),
            EntKind::Circle | EntKind::Arc => vec![self.round_radius(e) as u32],
            EntKind::Axis => {
                let a = &self.axes[e.i()];
                a.d.iter().chain(&a.a).copied().collect()
            }
            EntKind::Frame => {
                self.frames[e.i()].chart.as_ref().map_or_else(Vec::new, |c| c.o.to_vec())
            }
            _ => Vec::new(),
        }
    }

    /// The rotation of a turn about an axis, from the axis's held direction: Rodrigues' formula,
    /// row-major.
    fn axis_rotation(&self, t: &Turn) -> [[f64; 3]; 3] {
        let d = self.axes[t.about.i()].d.map(|p| self.params[p as usize].value);
        let d = crate::space::normalised(d).unwrap_or([0.0, 0.0, 1.0]);
        let (c, s) = t.cos_sin();
        let k = 1.0 - c;
        [
            [c + d[0] * d[0] * k, d[0] * d[1] * k - d[2] * s, d[0] * d[2] * k + d[1] * s],
            [d[1] * d[0] * k + d[2] * s, c + d[1] * d[1] * k, d[1] * d[2] * k - d[0] * s],
            [d[2] * d[0] * k - d[1] * s, d[2] * d[1] * k + d[0] * s, c + d[2] * d[2] * k],
        ]
    }

    /// Every derived parameter, in turn order: a turned point's coordinates over the
    /// representative's and the centre's, a turned circle's or arc's radius over the
    /// representative's.  A turn whose entities do not fit it (a point in space about a point,
    /// a planar point about an axis) derives nothing — elaboration refuses those (E023).
    pub fn derived(&self) -> Vec<Derivation> {
        let mut out = Vec::new();
        for t in &self.turns {
            match t.copy.kind {
                EntKind::Point => {
                    let (p, q) = (&self.points[t.copy.i()], &self.points[t.rep.i()]);
                    match t.about.kind {
                        EntKind::Point if p.z.is_none() && q.z.is_none() => {
                            let o = &self.points[t.about.i()];
                            let (c, s) = t.cos_sin();
                            out.push(Derivation {
                                param: p.x,
                                terms: vec![(q.x, c), (q.y, -s), (o.x, 1.0 - c), (o.y, s)],
                            });
                            out.push(Derivation {
                                param: p.y,
                                terms: vec![(q.x, s), (q.y, c), (o.x, -s), (o.y, 1.0 - c)],
                            });
                        }
                        EntKind::Axis => {
                            let (Some(pz), Some(qz)) = (p.z, q.z) else { continue };
                            let r = self.axis_rotation(t);
                            let a = self.axes[t.about.i()].a;
                            let (copy, rep) = ([p.x, p.y, pz], [q.x, q.y, qz]);
                            for i in 0..3 {
                                let mut terms = Vec::with_capacity(6);
                                for j in 0..3 {
                                    terms.push((rep[j], r[i][j]));
                                }
                                for j in 0..3 {
                                    let w = if i == j { 1.0 - r[i][j] } else { -r[i][j] };
                                    terms.push((a[j], w));
                                }
                                out.push(Derivation { param: copy[i], terms });
                            }
                        }
                        _ => {}
                    }
                }
                EntKind::Circle | EntKind::Arc => out.push(Derivation {
                    param: self.round_radius(t.copy) as u32,
                    terms: vec![(self.round_radius(t.rep) as u32, 1.0)],
                }),
                _ => {}
            }
        }
        if self.mirrors.is_empty() {
            return out;
        }
        for m in &self.mirrors {
            let (copy, rep) = (self.mirrored_params(m.copy), self.mirrored_params(m.rep));
            if copy.len() != rep.len() {
                continue;
            }
            for (k, (c, r)) in copy.into_iter().zip(rep).enumerate() {
                let w = if m.negated >> k & 1 == 1 { -1.0 } else { 1.0 };
                out.push(Derivation { param: c, terms: vec![(r, w)] });
            }
        }
        // an image of a ring's copy, or a turn about an image: a base is never derived, so each
        // derived base is read as what it is derived from (an image is of no image)
        let at: std::collections::BTreeMap<u32, usize> =
            out.iter().enumerate().map(|(k, d)| (d.param, k)).collect();
        let expand = |terms: &[(u32, f64)]| -> Vec<(u32, f64)> {
            let mut done = Vec::with_capacity(terms.len());
            let mut pending: Vec<(u32, f64, u32)> = terms.iter().map(|&(b, w)| (b, w, 0)).collect();
            while let Some((b, w, depth)) = pending.pop() {
                match at.get(&b) {
                    Some(&k) if depth < 4 => {
                        pending.extend(out[k].terms.iter().map(|&(c, v)| (c, w * v, depth + 1)));
                    }
                    _ => done.push((b, w)),
                }
            }
            done.reverse();
            done
        };
        let expanded: Vec<Vec<(u32, f64)>> = out.iter().map(|d| {
            match d.terms.iter().any(|(b, _)| at.contains_key(b)) {
                true => expand(&d.terms),
                false => d.terms.clone(),
            }
        }).collect();
        for (d, terms) in out.iter_mut().zip(expanded) {
            d.terms = terms;
        }
        out
    }

    /// Whether each parameter is derived, by index — empty for a sketch with no ring and no
    /// image, which every reader takes as "none is".
    pub fn derived_mask(&self) -> Vec<bool> {
        if self.turns.is_empty() && self.mirrors.is_empty() {
            return Vec::new();
        }
        let mut m = vec![false; self.params.len()];
        for d in self.derived() {
            m[d.param as usize] = true;
        }
        m
    }

    /// Write every derived parameter's value from its bases — after anything writes the
    /// parameter vector (`set_x`), and after a build.
    pub fn settle_turns(&mut self) {
        if self.turns.is_empty() && self.mirrors.is_empty() {
            return;
        }
        for d in self.derived() {
            self.settle_derivation(&d);
        }
    }

    /// Write one derived parameter's value from its bases: a base is never derived, so it is read
    /// off the parameters as they stand, by the sum `Derivation::value` takes.
    pub fn settle_derivation(&mut self, d: &Derivation) {
        let mut v = 0.0;
        for &(b, w) in &d.terms {
            v += w * self.params[b as usize].value;
        }
        self.params[d.param as usize].value = v;
    }

    /// The turn of a turned curve's view, about its centre's place in that view: worked out once
    /// and applied to every sample.
    pub fn turn_in_view(&self, t: &Turn) -> impl Fn((f64, f64)) -> (f64, f64) {
        let (ox, oy) = self.point_xy(t.about.i());
        let (c, s) = t.cos_sin();
        move |(x, y)| {
            let (dx, dy) = (x - ox, y - oy);
            (ox + c * dx - s * dy, oy + s * dx + c * dy)
        }
    }

    /// The turn a curve is, with the curve it is a turn of — `None` for a curve of its own.
    pub fn curve_turn(&self, i: usize) -> Option<(usize, Turn)> {
        let t = *self.turn_of(EntRef::new(EntKind::Curve, i))?;
        (t.about.kind == EntKind::Point).then_some((t.rep.i(), t))
    }
}
