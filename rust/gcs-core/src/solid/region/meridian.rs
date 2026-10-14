//! **The meridian** (§6.21): a region solid whose terms share one axis is a revolution of one
//! region in the half-plane through that axis, made by the kernel's exact line-and-arc Booleans
//! (`brep::planar`), each term folded in as the region it bounds there (`Fold`); a round or a
//! chamfer between two of its own bounds is built at their corner, and held to the region read
//! point by point (`agrees`).
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::planar::{boolean, Op, Region, Seg};
use crate::model::{RegionShape, RegionTerm, Sketch, SolidDef};
use crate::solid::lowering::Section;
use crate::space::{across, cross, dot, norm, normalised, scale, sub};
use crate::syntax::Cmp;
use std::f64::consts::{FRAC_PI_2, PI};

type V = [f64; 3];

/// A region solid's meridian: the half-plane through `origin` along `axis` (a unit vector),
/// towards `seam` (a unit vector square to it), and the region in its (along `seam`, along
/// `axis`) coordinates, each step tagged by the term it came from (1 + its index; 0 none).
#[derive(Clone, Debug)]
pub struct Meridian {
    pub origin: V,
    pub axis: V,
    pub seam: V,
    pub region: Region,
    /// Each term's name, by its tag less one: what the faces it bounds are called.
    pub names: Vec<String>,
}

impl Meridian {
    /// The name of the face a step's tag makes.
    pub fn name(&self, tag: u32) -> String {
        match tag {
            0 => String::new(),
            t => self.names.get(t as usize - 1).cloned().unwrap_or_default(),
        }
    }
}

/// Why a meridian refused: its words, and whether no reading in general position answers either
/// — a region with no material or none bounded, which is the same region however it is read.
#[derive(Clone, Debug)]
pub struct Refusal {
    pub(super) why: String,
    pub(super) outright: bool,
}

impl From<String> for Refusal {
    fn from(why: String) -> Refusal { Refusal { why, outright: false } }
}

impl From<&str> for Refusal {
    fn from(why: &str) -> Refusal { why.to_string().into() }
}

/// Region solid `solid`'s meridian folded from its terms (`region::meridian`), or why not.
pub(super) fn fold(sk: &Sketch, solid: usize) -> Result<Meridian, Refusal> {
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Region { terms, .. } = &s.def else {
        return Err(format!("`{}` is not a region", s.name).into());
    };
    if terms.is_empty() {
        return Err(format!("`{}` bounds nothing: a solid is a region", s.name).into());
    }
    // a round or a chamfer in the region's own body is built in the meridian where it is one of
    // two of the region's bounds (`meridian_corner`, #183): any other term read point by point is
    // none
    let corners: Vec<Option<MeridianCorner>> =
        terms.iter().map(|t| meridian_corner(sk, terms, t)).collect();
    // and a sum of distances from points, a conic about the line through them (#238)
    let conics: Vec<Option<Vec<(V, f64)>>> = terms.iter().map(|t| foci(sk, t)).collect();
    if terms.iter().zip(&corners).zip(&conics)
        .any(|((t, l), c)| point_by_point(&t.shape) && l.is_none() && c.is_none()) {
        return Err(format!("`{}` bounds a sum of measurements or reads faces, which no meridian \
                            folds", s.name).into());
    }
    let (origin, axis) = axis_of(sk, terms, &conics).ok_or("a region with no axis")?;
    // each term's low end as the drawing stands (an unknown's, solved; a cone's half within the
    // half turn its row reads, `expr::sync_free`)
    let lows = terms.iter().map(|t| t.lo.value(sk).map_err(|m| format!("{}: {m}", s.name)))
        .collect::<Result<Vec<f64>, String>>()?;
    // and each offset, a signed distance from the surface the term's number makes (F4)
    let grows = terms.iter().map(|t| t.grow_value(sk).map_err(|m| format!("{}: {m}", s.name)))
        .collect::<Result<Vec<f64>, String>>()?;
    // how far everything stands from the origin, for the tolerances and the box
    let mut size: f64 = 1.0;
    for ((t, lo), g) in terms.iter().zip(&lows).zip(&grows) {
        size = size.max(lo.abs()).max(t.hi.unwrap_or(0.0).abs()).max(g.abs());
        size = stands(sk, t).into_iter().fold(size, |s, p| s.max(norm(sub(p, origin))));
    }
    let tol = 1e-7 * size;
    let f = Fold { sk, origin, axis, tol, reach: 8.0 * size };
    let reach = f.reach;
    let mut region = rect(0.0, reach, -reach, reach);
    for (k, ((t, &lo), &g)) in terms.iter().zip(&lows).zip(&grows).enumerate() {
        if corners[k].is_some() {
            continue;
        }
        let tag = k as u32 + 1;
        let fold = |region: &Region, v: f64, op: Op| -> Result<Region, String> {
            boolean(region, &f.within(t, v, g)?.tagged(&|_, _| tag), op, tol)
                .map_err(|e| format!("{}: {e}", what(t)))
        };
        region = match t.cmp {
            Cmp::Le => fold(&region, lo, Op::Common)?,
            Cmp::Ge => fold(&region, lo, Op::Cut)?,
            Cmp::In => {
                let hi = fold(&region, t.hi.unwrap_or(lo), Op::Common)?;
                fold(&hi, lo, Op::Cut)?
            }
        };
    }
    let region = rounded(region, &corners, &s.name)?;
    if region.loops.is_empty() || region.area().abs() <= tol * tol {
        let why = format!("`{}` leaves no material: its bounds have no point in common", s.name);
        return Err(Refusal { why, outright: true });
    }
    let touches = region.loops.iter().flatten().any(|step| {
        [step.start(), step.end()].iter().any(|p| p[0] >= reach - tol || p[1].abs() >= reach - tol)
    });
    if touches {
        let why = format!("`{}` has no end: bound it all round (a ball, two planes, …)", s.name);
        return Err(Refusal { why, outright: true });
    }
    // the half-plane's direction: square to the axis, toward the world axis least along it
    let seam = scale(across(axis).1, -1.0);
    // a round's or a chamfer's own scale: how far off the boundary the gate reads
    let least = corners.iter().flatten().map(|l| match l.kind {
        Section::Round(r) => r,
        Section::Flat { la, lb } => la.min(lb),
    }).fold(f64::INFINITY, f64::min);
    // a sum of distances folded is held so too, a thousandth of the region's size off (#238)
    let least = if conics.iter().any(Option::is_some) { least.min(size) } else { least };
    if least.is_finite() {
        let basis = crate::plane::Basis { u: seam, v: axis, o: origin };
        agrees(sk, solid, &region, &basis, (1e-3 * least).max(100.0 * tol))
            .map_err(|e| format!("`{}`: {e}", s.name))?;
    }
    let names = terms.iter().map(|t| t.name.clone()).collect();
    Ok(Meridian { origin, axis, seam, region, names })
}

/// The axis a region's terms turn about: a cone's or a cylinder's line, else a plane's normal
/// through a ball's centre or the plane's point, else the line through two balls' centres or
/// measured points (`conics`, each term's `foci`), else the z through one.
fn axis_of(sk: &Sketch, terms: &[RegionTerm], conics: &[Option<Vec<(V, f64)>>]) -> Option<(V, V)> {
    let ball = |t: &RegionTerm| match t.shape {
        RegionShape::Ball { center } => Some(point(sk, center)),
        _ => None,
    };
    let around = terms.iter().find_map(|t| match t.shape {
        RegionShape::Cylinder { line: l } | RegionShape::Cone { axis: l } => {
            let (a, b) = line(sk, l);
            normalised(sub(b, a)).map(|d| (a, d))
        }
        _ => None,
    });
    let normal = terms.iter().find_map(|t| match t.shape {
        RegionShape::Plane { from, .. } => {
            let at = terms.iter().find_map(ball).unwrap_or(point(sk, from));
            Some((at, t.shape.normal(sk)?))
        }
        _ => None,
    });
    let centres: Vec<V> = terms.iter().filter_map(ball)
        .chain(conics.iter().flatten().flatten().map(|&(p, _)| p)).collect();
    let through = || {
        let c0 = *centres.first()?;
        let other = centres.iter().find_map(|&c| normalised(sub(c, c0)));
        Some((c0, other.unwrap_or([0.0, 0.0, 1.0])))
    };
    around.or(normal).or_else(through)
}

/// Point `i` where it stands in space.
fn point(sk: &Sketch, i: u32) -> V { sk.world_point(i as usize) }

/// Line `l`'s ends where they stand in space.
fn line(sk: &Sketch, l: u32) -> (V, V) {
    let l = &sk.lines[l as usize];
    (point(sk, l.p1), point(sk, l.p2))
}

/// The points a term stands on: a ball's centre, a cylinder's or a cone's line's ends, a plane's
/// origin, the points a sum of distances measures from.
fn stands(sk: &Sketch, t: &RegionTerm) -> Vec<V> {
    match t.shape {
        RegionShape::Ball { center } => vec![point(sk, center)],
        RegionShape::Measured { .. } => foci(sk, t).into_iter().flatten().map(|(p, _)| p).collect(),
        RegionShape::Cylinder { line: l } | RegionShape::Cone { axis: l } => {
            let (a, b) = line(sk, l);
            vec![a, b]
        }
        RegionShape::Plane { from, .. } => vec![point(sk, from)],
        _ => Vec::new(),
    }
}

/// A term as a refusal names it.
fn what(t: &RegionTerm) -> String {
    if t.name.is_empty() { "a bound".to_string() } else { format!("`{}`", t.name) }
}

/// The rectangle `[r0, r1] × [z0, z1]` of the half-plane.
fn rect(r0: f64, r1: f64, z0: f64, z1: f64) -> Region {
    Region::plain(vec![vec![
        Seg::Line { a: [r0, z0], b: [r1, z0] },
        Seg::Line { a: [r1, z0], b: [r1, z1] },
        Seg::Line { a: [r1, z1], b: [r0, z1] },
        Seg::Line { a: [r0, z1], b: [r0, z0] },
    ]])
}

/// What a meridian is folded in: the half-plane through `origin` about the unit `axis`, its box
/// reaching `reach` each way, and the tolerance its Booleans take.
struct Fold<'a> {
    sk: &'a Sketch,
    origin: V,
    axis: V,
    tol: f64,
    reach: f64,
}

impl Fold<'_> {
    /// How far along the axis `p` stands.
    fn z(&self, p: V) -> f64 { dot(sub(p, self.origin), self.axis) }

    /// How far off the axis `p` stands.
    fn off(&self, p: V) -> f64 { norm(cross(sub(p, self.origin), self.axis)) }

    /// A cone narrower than square, apex at `za` on the axis, opening along `s`: the points no
    /// more than `g` outside its surface (§6.21, F4) — shrunk (`g < 0`) the cone whose surface
    /// stands `-g` within, its apex further along; grown, the cone standing `g` without, its apex
    /// rounded by the ball of radius `g` about the apex, where the nearest point of the surface
    /// is the apex (the Minkowski sum).
    fn narrow(&self, s: f64, half: f64, za: f64, g: f64) -> Region {
        let far = za + s * self.reach;
        if g <= 0.0 {
            let apex = za - s * g / half.dsin();
            let width = s * (far - apex) * half.dtan();
            if width <= 0.0 {
                return Region { loops: Vec::new() };
            }
            // a loop either way round: `Region::plain` turns it
            let pts = [[0.0, apex], [width, far], [0.0, far]];
            return Region::plain(vec![(0..3).map(|k| Seg::Line { a: pts[k], b: pts[(k + 1) % 3] })
                .collect()]);
        }
        let (sin, cos) = (half.dsin(), half.dcos());
        // where the grown surface leaves the ball, square to the generator from the apex
        let tangent = [g * cos, za - s * g * sin];
        let width = tangent[0] + s * (far - tangent[1]) * half.dtan();
        let line = |a: [f64; 2], b: [f64; 2]| Seg::Line { a, b };
        let steps = if s > 0.0 {
            // up the axis from below the apex: the ball's rim to the surface, out along it
            let arc = Seg::Arc { c: [0.0, za], r: g, a0: -FRAC_PI_2, sweep: FRAC_PI_2 - half };
            vec![arc, line(tangent, [width, far]), line([width, far], [0.0, far]),
                 line([0.0, far], [0.0, za - g])]
        } else {
            let arc = Seg::Arc { c: [0.0, za], r: g, a0: half, sweep: FRAC_PI_2 - half };
            vec![line([0.0, far], [width, far]), line([width, far], tangent), arc,
                 line([0.0, za + g], [0.0, far])]
        };
        Region::plain(vec![steps])
    }

    /// A sum of distances from points on the axis (`foci`): `Σ cᵢ dᵢ ≤ v`, in the half-plane.
    fn measured_within(&self, t: &RegionTerm, v: f64, g: f64) -> Result<Region, String> {
        if g != 0.0 {
            return Err(format!("{} is a sum of measurements grown, which no meridian folds", what(t)));
        }
        let (reach, tol) = (self.reach, self.tol);
        let all = || rect(0.0, reach, -reach, reach);
        let none = || Region { loops: Vec::new() };
        let less = |r: Region| boolean(&all(), &r, Op::Cut, tol);
        let ps = foci(self.sk, t).expect("a sum the meridian reads");
        Ok(match ps.as_slice() {
            // `c d ≤ v`: within a ball, or (`c` below nothing) outside one
            &[(p, c)] => {
                let r = v / c;
                match (c > 0.0, r > 0.0) {
                    (true, true) => Region::half_disc(self.z(p), r),
                    (true, false) => none(),
                    (false, true) => less(Region::half_disc(self.z(p), r))?,
                    (false, false) => all(),
                }
            }
            // `c (d₁ + d₂) ≤ v`: within an ellipsoid, or outside one
            &[(p, c), (q, d)] if c * d > 0.0 => {
                let two_a = v / c;
                let e = if two_a > 0.0 { half_ellipse(self.z(p), self.z(q), two_a, tol) } else { none() };
                if c > 0.0 { e } else if two_a > 0.0 { less(e)? } else { all() }
            }
            // `c (d₁ − d₂) ≤ v`, `c` above nothing once the two are named the other way round:
            // the side of the plane midway between them nearer the first, or a hyperboloid's lobe
            // about it, or all but the lobe about the second
            &[(p, c), (q, _)] => {
                let ((p, q), k) = if c > 0.0 { ((p, q), v / c) } else { ((q, p), -v / c) };
                let (zp, zq) = (self.z(p), self.z(q));
                let span = (zq - zp).abs();
                if k >= span - tol {
                    all()
                } else if k <= -span + tol {
                    none()
                } else if k.abs() <= tol {
                    let mid = (zp + zq) / 2.0;
                    if zp < zq { rect(0.0, reach, -reach, mid) } else { rect(0.0, reach, mid, reach) }
                } else if k < 0.0 {
                    lobe(zp, zq, -k, reach)
                } else {
                    less(lobe(zq, zp, k, reach))?
                }
            }
            _ => unreachable!("`foci` reads one point or two"),
        })
    }

    /// The points a term reads at most `v` at, or with an offset `g`, no more than `g` outside
    /// the surface where it reads `v` — in the half-plane.
    fn within(&self, t: &RegionTerm, v: f64, g: f64) -> Result<Region, String> {
        let (sk, reach, tol) = (self.sk, self.reach, self.tol);
        let plane = matches!(t.shape, RegionShape::Plane { .. });
        if !plane && stands(sk, t).into_iter().any(|p| self.off(p) > tol) {
            return Err(match t.shape {
                RegionShape::Ball { .. } => format!("{}'s centre is off the region's axis", what(t)),
                RegionShape::Measured { .. } => format!("{} measures from a point off the region's axis",
                    what(t)),
                _ => format!("{} is about a line other than the region's axis", what(t)),
            });
        }
        if let RegionShape::Measured { .. } = t.shape {
            return self.measured_within(t, v, g);
        }
        // a measure from a point or a line is never less than nothing (a distance's offset is its
        // number moved before it gets here: only a cone's is `g`, `region_term`)
        if !plane && v <= 0.0 {
            if g != 0.0 {
                return Err(format!("{} is a cone of no width, which has no offset", what(t)));
            }
            return Ok(Region { loops: Vec::new() });
        }
        Ok(match t.shape {
            RegionShape::Ball { center } => Region::half_disc(self.z(point(sk, center)), v),
            RegionShape::Cylinder { .. } => rect(0.0, v, -reach, reach),
            RegionShape::Cone { axis: l } => {
                let (a, b) = line(sk, l);
                let s = dot(sub(b, a), self.axis).signum();
                let za = self.z(a);
                if v < FRAC_PI_2 - 1e-12 {
                    self.narrow(s, v, za, g)
                } else if v >= PI {
                    rect(0.0, reach, -reach, reach)
                } else if (v - FRAC_PI_2).abs() <= 1e-12 {
                    // the side the axis points into, `g` further back
                    if s > 0.0 {
                        rect(0.0, reach, za - g, reach)
                    } else {
                        rect(0.0, reach, -reach, za + g)
                    }
                } else {
                    // wider than square: all but the cone the other way — whose outside it is —
                    // of what is left of half a turn, that one shrunk as this one grows
                    boolean(&rect(0.0, reach, -reach, reach), &self.narrow(-s, PI - v, za, -g), Op::Cut,
                        tol)?
                }
            }
            RegionShape::Plane { from, sign, .. } => {
                let n = t.shape.normal(sk).ok_or_else(|| format!("{} has no normal", what(t)))?;
                // square within the fold's tolerance at the region's size, as its points are on
                // the axis: two lines a solve holds on one axis to its tolerance, not to the bit
                if norm(cross(n, self.axis)) * reach > 8.0 * tol {
                    return Err(format!("{}'s plane is not square to the region's axis", what(t)));
                }
                // the ordinate, `sign · n · (x - from)`, along the axis
                let k = sign * dot(n, self.axis).signum();
                // a half-space grown moves its plane (#183: a region's own bound, kept off)
                let edge = self.z(point(sk, from)) + k * (v + g);
                if k > 0.0 { rect(0.0, reach, -reach, edge) } else { rect(0.0, reach, edge, reach) }
            }
            _ => unreachable!("refused before the fold"),
        })
    }
}

/// Each round and chamfer (`corners`, by its term) built into `region` at every corner of its two
/// bounds (`brep::corners`, X1): a round wherever the two bounds' carriers are lines or circles —
/// an offset cone's generator and its rounded apex included (`Fold::narrow`) — and checked
/// against the region read point by point once built (`agrees`); `solid` names the region.
fn rounded(mut region: Region, corners: &[Option<MeridianCorner>], solid: &str)
    -> Result<Region, Refusal> {
    for (k, l) in corners.iter().enumerate() {
        let Some(l) = l else { continue };
        let tag = k as u32 + 1;
        let (a, b) = (l.a as u32 + 1, l.b as u32 + 1);
        // exact only where it builds what the set says: a flat only where its sum of distances is
        // a line or a conic (`corners::sum`: between two arcs only with equal legs, else a
        // Cartesian oval)
        let curved = |x: &crate::brep::planar::Step| matches!(x.seg, Seg::Arc { .. });
        let between_arcs = || region.loops.iter().any(|lp| (0..lp.len()).any(|i| {
            let (st, nx) = (&lp[i], &lp[(i + 1) % lp.len()]);
            let named = (st.tag == a && nx.tag == b) || (st.tag == b && nx.tag == a);
            named && curved(st) && curved(nx)
        }));
        if matches!(l.kind, Section::Flat { la, lb } if (la - lb).abs() > 1e-12 * la.max(lb)) && between_arcs() {
            return Err(format!("`{solid}`: a flat between two curved faces with unequal legs is read point by \
                                point: its sum of distances is a Cartesian oval, which no rational curve holds").into());
        }
        region = match l.kind {
            Section::Round(r) => crate::brep::corners::round(&region, &[a], &[b], r, tag),
            Section::Flat { la, lb } => crate::brep::corners::summed(&region, &[a], &[b], [la, lb], tag),
        }.map_err(|e| format!("`{solid}`: {e}"))?;
    }
    Ok(region)
}

/// **The gate** (#207): a meridian with a round or a chamfer built in it stands only where the
/// region read point by point (`general`) agrees with it, as the lowering table's pieces are held
/// (`lowering::gate`) — `by` inside each of its faces material, `by` outside none, read at each
/// face's quarter points (no further off than a hundredth of the face's length; the axis is no
/// face).  Lowering is a representation, never a meaning: where they disagree the region is read
/// point by point.
fn agrees(sk: &Sketch, solid: usize, region: &Region, half_plane: &crate::plane::Basis, by: f64)
    -> Result<(), String> {
    let field = super::general(sk, solid)?;
    let lift = |q: [f64; 2]| half_plane.lift(q[0], q[1]);
    for step in region.loops.iter().flatten().filter(|s| s.tag != 0) {
        let off = by.min(1e-2 * step.length());
        for t in [0.25, 0.5, 0.75] {
            let (p, n) = (step.point(t), step.own_normal(t));
            let [inside, outside] = [-off, off].map(|o| lift([p[0] + o * n[0], p[1] + o * n[1]]));
            let (vi, vo) = (field.value(inside), field.value(outside));
            if !(vi < 0.0 && vo > 0.0) {
                let x = lift(p);
                return Err(format!("its rounds and chamfers built in the meridian disagree with \
                                    the region read point by point beside [{:.6}, {:.6}, {:.6}] \
                                    ({vi:e} within, {vo:e} without)", x[0], x[1], x[2]));
            }
        }
    }
    Ok(())
}

/// The meridian's loops as the faceted kernel turns them: polygons in the half-plane (`u` along
/// the seam, `v` along the axis), each arc chorded as a face's arcs are (`tessellate_arc`), a conic
/// as an arc of its length and turn would be, every side named by the term it came from — the loop
/// enclosing most first, the holes after.
pub(crate) fn face_polys(m: &Meridian, unit: f64) -> Vec<crate::solid::profile::FacePoly> {
    let basis = crate::plane::Basis { u: m.seam, v: m.axis, o: m.origin };
    m.region.outer_first().into_iter().map(|l| {
        let (mut pts, mut of, mut names) = (Vec::new(), Vec::new(), Vec::new());
        for step in l {
            let k = names.len();
            names.push(m.name(step.tag));
            match step.seg {
                Seg::Line { a, .. } => {
                    pts.push((a[0], a[1]));
                    of.push((k, false));
                }
                Seg::Conic { .. } => {
                    // chorded as an arc of its length through the turn between its ends would be
                    let (t0, t1) = (step.seg.tangent(0.0), step.seg.tangent(1.0));
                    let turn = (t0[0] * t1[1] - t0[1] * t1[0]).datan2(t0[0] * t1[0] + t0[1] * t1[1]).abs()
                        .max(1e-9);
                    let r = step.seg.length() / turn;
                    let n = crate::solid::profile::tessellate_arc((0.0, 0.0), r, 0.0, turn, unit).len() - 1;
                    for j in 0..n {
                        let p = step.seg.point(j as f64 / n as f64);
                        pts.push((p[0], p[1]));
                        of.push((k, true));
                    }
                }
                Seg::Arc { c, r, a0, sweep } => {
                    let ring = crate::solid::profile::tessellate_arc((c[0], c[1]), r, a0, sweep, unit);
                    for p in &ring[..ring.len() - 1] {
                        pts.push(*p);
                        of.push((k, true));
                    }
                    // the first vertex of an arc is a real corner, not a chord joint
                    let first = of.len() - (ring.len() - 1);
                    of[first].1 = ring.len() > 2;
                }
            }
        }
        crate::solid::profile::FacePoly { pts, of, names, basis }
    }).collect()
}

/// **A corner built in the meridian** (#183 step 7): a term a region's body states that the
/// meridian builds exactly — `std.Rounded(a, b, r)`, within reach of a ball of `r` lying `r` within
/// both of two of the region's own bounds, and `std.Chamfered(a, b, la, lb)`, short of the flat
/// between their surfaces — at the bounds `a` and `b` (their indices among the terms).
struct MeridianCorner {
    a: usize,
    b: usize,
    kind: Section,
}

/// The corner a term builds in the meridian, where it is one: its sets each the region's own
/// bound (a shape it states, at its level — moved in by the ball's radius for a round's centres),
/// never what the set is for.
fn meridian_corner(sk: &Sketch, terms: &[RegionTerm], t: &RegionTerm) -> Option<MeridianCorner> {
    use crate::model::Target;
    let RegionShape::Measured { of } = &t.shape else { return None };
    let lo = t.lo.read(sk)?;
    let level = |u: &RegionTerm| u.read(sk);
    // the outer bound with the same measure whose surface stands `want` outward of `u`'s (a
    // slab's two sides share a measure)
    let outer = |u: &RegionTerm, want: f64| -> Option<(usize, f64)> {
        let gap = |o: &RegionTerm| -> Option<f64> {
            let ((lo_o, g_o), (lo_u, g_u)) = (level(o)?, level(u)?);
            // the gap from the outer surface to `u`'s, toward the inside of the outer bound
            let inward = if o.cmp == Cmp::Le { 1.0 } else { -1.0 };
            match o.shape {
                RegionShape::Cone { .. } => {
                    if (lo_o - lo_u).abs() > 1e-12 * (1.0 + lo_o.abs()) { return None; }
                    Some(inward * (g_o - g_u))
                }
                _ => Some(inward * ((lo_o + g_o) - (lo_u + g_u))),
            }
        };
        terms.iter().enumerate()
            .filter(|(_, o)| o.shape == u.shape && !point_by_point(&o.shape) && o.cmp != Cmp::In)
            .filter_map(|(k, o)| gap(o).map(|g| (k, g)))
            .min_by(|x, y| (x.1 - want).abs().total_cmp(&(y.1 - want).abs()))
    };
    let tol = |x: f64| 1e-9 * (1.0 + x.abs());
    match (of.as_slice(), t.cmp) {
        // within `r` of the centres: a round
        ([(Target::Set { terms: inner, .. }, c)], Cmp::Le) if *c > 0.0 && inner.len() == 2 => {
            let r = lo / c;
            let (ka, ga) = outer(&inner[0], r)?;
            let (kb, gb) = outer(&inner[1], r)?;
            let side = |u: &RegionTerm, k: usize| u.cmp == terms[k].cmp;
            (ka != kb && side(&inner[0], ka) && side(&inner[1], kb) && r > 0.0
                && (ga - r).abs() <= tol(r) && (gb - r).abs() <= tol(r))
                .then_some(MeridianCorner { a: ka, b: kb, kind: Section::Round(r) })
        }
        // at least one from the flat between two surfaces: a chamfer
        ([(Target::Set { terms: sa, .. }, ca), (Target::Set { terms: sb, .. }, cb)], Cmp::Ge)
            if *ca > 0.0 && *cb > 0.0 && lo > 0.0 && sa.len() == 1 && sb.len() == 1 =>
        {
            // each set the outside of one of the region's bounds (or that bound's surface): the
            // distance from it is from the bound's surface, within the region
            let (ka, ga) = outer(&sa[0], 0.0)?;
            let (kb, gb) = outer(&sb[0], 0.0)?;
            let across = |u: &RegionTerm, k: usize| match u.cmp {
                Cmp::In => u.hi.is_some_and(|h| u.lo.read(sk) == Some(h)),
                c => Some(c) == terms[k].cmp.turned(),
            };
            (ka != kb && across(&sa[0], ka) && across(&sb[0], kb)
                && ga.abs() <= tol(lo) && gb.abs() <= tol(lo))
                .then_some(MeridianCorner { a: ka, b: kb, kind: Section::Flat { la: lo / cb, lb: lo / ca } })
        }
        _ => None,
    }
}

/// A sum of measurements the meridian folds (#238): the distance from one point, or the sum or the
/// difference of the distances from two alike in weight — each point given as it stands, with its
/// number.  About the line through the points such a bound is a ball, an ellipsoid's or a
/// hyperboloid's lobe turned, or the plane square to the line midway between them: a circle, a
/// conic or a line in the meridian.  `None` for any other sum.
fn foci(sk: &Sketch, t: &RegionTerm) -> Option<Vec<(V, f64)>> {
    use crate::model::{EntKind, Target};
    let RegionShape::Measured { of } = &t.shape else { return None };
    let points = of.iter().map(|(target, c)| match target {
        Target::Ent(e) if e.kind == EntKind::Point && *c != 0.0 && c.is_finite() => {
            Some((sk.world_point(e.i()), *c))
        }
        _ => None,
    }).collect::<Option<Vec<_>>>()?;
    match points.as_slice() {
        [_] => Some(points),
        [(_, a), (_, b)] if (a.abs() - b.abs()).abs() <= 1e-12 * a.abs() => Some(points),
        _ => None,
    }
}

/// The half of an ellipse of revolution in the meridian: its foci on the axis at `z1` and `z2`, the
/// sum of the distances from them `two_a` — two quarter ellipses (`Seg::Conic`, a quarter's middle
/// weighed `cos 45°`) and the axis between their ends.  Nothing where it is no wider than its foci.
fn half_ellipse(z1: f64, z2: f64, two_a: f64, tol: f64) -> Region {
    let (a, f) = (two_a / 2.0, (z2 - z1).abs() / 2.0);
    if a - f <= tol {
        return Region { loops: Vec::new() };
    }
    let (zc, b, w) = ((z1 + z2) / 2.0, ((a - f) * (a + f)).sqrt(), std::f64::consts::FRAC_1_SQRT_2);
    Region::plain(vec![vec![
        Seg::Conic { a: [0.0, zc - a], m: [b, zc - a], b: [b, zc], w },
        Seg::Conic { a: [b, zc], m: [b, zc + a], b: [0.0, zc + a], w },
        Seg::Line { a: [0.0, zc + a], b: [0.0, zc - a] },
    ]])
}

/// The lobe of a hyperboloid of two sheets in the meridian about the focus at `near`: the points
/// whose distance from it falls `k` short of that from the focus at `far` (`0 < k < |far − near|`),
/// out past `reach` from the axis and along it — the branch `z = zc ± a cosh u`, `r = b sinh u` in
/// conic arcs of at most a unit of `u` each (an arc's middle weighed `cosh` of half its span), and
/// the line across its far end and the axis back.
fn lobe(near: f64, far: f64, k: f64, reach: f64) -> Region {
    let (a, f) = (k / 2.0, (far - near).abs() / 2.0);
    let (zc, b, s) = ((near + far) / 2.0, ((f - a) * (f + a)).sqrt(), (near - far).signum());
    let (sinh, cosh) = (|u: f64| (u.dexp() - (-u).dexp()) / 2.0, |u: f64| (u.dexp() + (-u).dexp()) / 2.0);
    // out until it stands past the box either way
    let wide = 2.0 * reach / b;
    let long = ((2.0 * reach + zc.abs()) / a).max(1.0);
    let span = (wide + (wide * wide + 1.0).sqrt()).dln().min((long + (long * long - 1.0).sqrt()).dln());
    let n = span.ceil().max(1.0) as usize;
    let at = |u: f64| [b * sinh(u), zc + s * a * cosh(u)];
    let mut steps: Vec<Seg> = (0..n).map(|i| {
        let (u0, u1) = (span * i as f64 / n as f64, span * (i + 1) as f64 / n as f64);
        let (um, h) = ((u0 + u1) / 2.0, (u1 - u0) / 2.0);
        let m = [b * sinh(um) / cosh(h), zc + s * a * cosh(um) / cosh(h)];
        Seg::Conic { a: at(u0), m, b: at(u1), w: cosh(h) }
    }).collect();
    let end = at(span);
    steps.push(Seg::Line { a: end, b: [0.0, end[1]] });
    steps.push(Seg::Line { a: [0.0, end[1]], b: at(0.0) });
    Region::plain(vec![steps])
}

/// Whether a term is read only point by point: a sum of measurements, or a face's (#183).
fn point_by_point(shape: &RegionShape) -> bool {
    matches!(shape, RegionShape::Measured { .. }) || crate::solid::faces::reads_faces(shape)
}
