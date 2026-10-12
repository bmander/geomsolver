//! **A region as a solid** (§6.9, §6.21): `solid(R)` is the points inside the set `R`, each bound
//! its body put on the probe one term (`SolidDef::Region`).  Its terms share one axis, so the solid
//! is a revolution of one region in the half-plane through that axis — the **meridian**, made by
//! the kernel's exact line-and-arc Booleans (`brep::planar`) — and every evaluator reads that:
//! the exact solid a `revolve` of its profile (`brep::recipe::region_profile`), the field its
//! revolved region (`RevolvedRegion::of_meridian`), the facets its polygon turned once.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::planar::{boolean, Op, Region, Seg};
use crate::model::{RegionShape, RegionTerm, Sketch, SolidDef};
use crate::space::{across, cross, dot, norm, normalised, scale, sub};
use crate::syntax::Cmp;
use std::f64::consts::{FRAC_PI_2, PI};
use std::rc::Rc;

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

/// The meridian of region solid `solid`, from its terms as the drawing stands: refused where its
/// terms share no axis (a ball off it, a cone about another line, a plane not square to it),
/// where they leave no material, or where they leave material without end.  Remembered against
/// what it reads (`Sketch::meridian_cache`).
pub fn meridian(sk: &Sketch, solid: usize) -> Result<Rc<Meridian>, String> {
    folded(sk, solid).map_err(|r| r.why)
}

/// Why a meridian refused: its words, and whether no reading in general position answers either
/// — a region with no material or none bounded, which is the same region however it is read.
#[derive(Clone, Debug)]
pub struct Refusal {
    why: String,
    outright: bool,
}

impl From<String> for Refusal {
    fn from(why: String) -> Refusal { Refusal { why, outright: false } }
}

impl From<&str> for Refusal {
    fn from(why: &str) -> Refusal { why.to_string().into() }
}

/// `meridian`, with its refusal said whole.
fn folded(sk: &Sketch, solid: usize) -> Result<Rc<Meridian>, Refusal> {
    let key = super::reads(sk, solid, 0.0);
    if let Some((old, m)) = sk.meridian_cache.borrow().get(&solid) {
        if *old == key {
            return m.clone();
        }
    }
    let m = fold(sk, solid).map(Rc::new);
    sk.meridian_cache.borrow_mut().insert(solid, (key, m.clone()));
    m
}

fn fold(sk: &Sketch, solid: usize) -> Result<Meridian, Refusal> {
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Region { terms, .. } = &s.def else {
        return Err(format!("`{}` is not a region", s.name).into());
    };
    if terms.is_empty() {
        return Err(format!("`{}` bounds nothing: a solid is a region", s.name).into());
    }
    // a round or a chamfer in the region's own body is built in the meridian where it is one of
    // two of the region's bounds (`lowering`, #183): any other term read point by point is none
    let lowered: Vec<Option<Lowering>> = terms.iter().map(|t| lowering(sk, terms, t)).collect();
    if terms.iter().zip(&lowered).any(|(t, l)| point_by_point(&t.shape) && l.is_none()) {
        return Err(format!("`{}` bounds a sum of measurements or reads faces, which no meridian \
                            folds", s.name).into());
    }
    let point = |i: u32| sk.world_point(i as usize);
    let line = |l: u32| {
        let l = &sk.lines[l as usize];
        (point(l.p1), point(l.p2))
    };
    // the axis: a cone's or a cylinder's line, else a plane's normal through a ball's centre or the
    // plane's point, else the line through two balls' centres, else the z through one
    let ball = |t: &RegionTerm| match t.shape {
        RegionShape::Ball { center } => Some(point(center)),
        _ => None,
    };
    let around = terms.iter().find_map(|t| match t.shape {
        RegionShape::Cylinder { line: l } | RegionShape::Cone { axis: l } => {
            let (a, b) = line(l);
            normalised(sub(b, a)).map(|d| (a, d))
        }
        _ => None,
    });
    let normal = terms.iter().find_map(|t| match t.shape {
        RegionShape::Plane { plane, from, .. } => {
            let at = terms.iter().find_map(ball).unwrap_or(point(from));
            Some((at, sk.basis(plane as usize).normal()))
        }
        _ => None,
    });
    let centres: Vec<V> = terms.iter().filter_map(ball).collect();
    let through = || {
        let c0 = *centres.first()?;
        let other = centres.iter().find_map(|&c| normalised(sub(c, c0)));
        Some((c0, other.unwrap_or([0.0, 0.0, 1.0])))
    };
    let (origin, axis) = around.or(normal).or_else(through).ok_or("a region with no axis")?;
    // the points a term stands on: a ball's centre, a cylinder's or a cone's line's ends, a
    // plane's origin
    let stands = |t: &RegionTerm| match t.shape {
        RegionShape::Ball { center } => vec![point(center)],
        RegionShape::Cylinder { line: l } | RegionShape::Cone { axis: l } => {
            let (a, b) = line(l);
            vec![a, b]
        }
        RegionShape::Plane { from, .. } => vec![point(from)],
        _ => Vec::new(),
    };
    // each term's low end as the drawing stands (an unknown's, solved; a cone's half within the
    // half turn its row reads, `expr::sync_free`)
    let lows = terms.iter().map(|t| t.lo.value(sk).map_err(|m| format!("{}: {m}", s.name)))
        .collect::<Result<Vec<f64>, String>>()?;
    // and each offset, a signed distance from the surface the term's number makes (F4)
    let grows = terms.iter().map(|t| {
        t.grow.as_ref().map_or(Ok(0.0), |g| g.value(sk).map_err(|m| format!("{}: {m}", s.name)))
    }).collect::<Result<Vec<f64>, String>>()?;
    // how far everything stands from the origin, for the tolerances and the box
    let mut size: f64 = 1.0;
    for ((t, lo), g) in terms.iter().zip(&lows).zip(&grows) {
        size = size.max(lo.abs()).max(t.hi.unwrap_or(0.0).abs()).max(g.abs());
        size = stands(t).into_iter().fold(size, |s, p| s.max(norm(sub(p, origin))));
    }
    let tol = 1e-7 * size;
    let z = |p: V| dot(sub(p, origin), axis);
    let off = |p: V| norm(cross(sub(p, origin), axis));
    let reach = 8.0 * size;
    let rect = |r0: f64, r1: f64, z0: f64, z1: f64| {
        Region::plain(vec![vec![
            Seg::Line { a: [r0, z0], b: [r1, z0] },
            Seg::Line { a: [r1, z0], b: [r1, z1] },
            Seg::Line { a: [r1, z1], b: [r0, z1] },
            Seg::Line { a: [r0, z1], b: [r0, z0] },
        ]])
    };
    let what = |t: &RegionTerm| {
        if t.name.is_empty() { "a bound".to_string() } else { format!("`{}`", t.name) }
    };
    // a cone narrower than square, apex at `za` on the axis, opening along `s`: the points no
    // more than `g` outside its surface (§6.21, F4) — shrunk (`g < 0`) the cone whose surface
    // stands `-g` within, its apex further along; grown, the cone standing `g` without, its apex
    // rounded by the ball of radius `g` about the apex, where the nearest point of the surface
    // is the apex (the Minkowski sum)
    let narrow = |s: f64, half: f64, za: f64, g: f64| -> Region {
        let far = za + s * reach;
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
    };
    // the points a term reads at most `v` at, or with an offset `g`, no more than `g` outside
    // the surface where it reads `v` — in the half-plane
    let within = |t: &RegionTerm, v: f64, g: f64| -> Result<Region, String> {
        let plane = matches!(t.shape, RegionShape::Plane { .. });
        if !plane && stands(t).into_iter().any(|p| off(p) > tol) {
            return Err(match t.shape {
                RegionShape::Ball { .. } => format!("{}'s centre is off the region's axis", what(t)),
                _ => format!("{} is about a line other than the region's axis", what(t)),
            });
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
            RegionShape::Ball { center } => Region::half_disc(z(point(center)), v),
            RegionShape::Cylinder { .. } => rect(0.0, v, -reach, reach),
            RegionShape::Cone { axis: l } => {
                let (a, b) = line(l);
                let s = dot(sub(b, a), axis).signum();
                let za = z(a);
                if v < FRAC_PI_2 - 1e-12 {
                    narrow(s, v, za, g)
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
                    boolean(&rect(0.0, reach, -reach, reach), &narrow(-s, PI - v, za, -g), Op::Cut,
                        tol)?
                }
            }
            RegionShape::Plane { plane, from, sign } => {
                let n = sk.basis(plane as usize).normal();
                if norm(cross(n, axis)) > 1e-9 {
                    return Err(format!("{}'s plane is not square to the region's axis", what(t)).into());
                }
                // the ordinate, `sign · n · (x - from)`, along the axis
                let k = sign * dot(n, axis).signum();
                // a half-space grown moves its plane (#183: a region's own bound, kept off)
                let edge = z(point(from)) + k * (v + g);
                if k > 0.0 { rect(0.0, reach, -reach, edge) } else { rect(0.0, reach, edge, reach) }
            }
            _ => unreachable!("refused before the fold"),
        })
    };
    let mut region = rect(0.0, reach, -reach, reach);
    for (k, ((t, &lo), &g)) in terms.iter().zip(&lows).zip(&grows).enumerate() {
        if lowered[k].is_some() {
            continue;
        }
        let tag = k as u32 + 1;
        let fold = |region: &Region, v: f64, op: Op| -> Result<Region, String> {
            boolean(region, &within(t, v, g)?.tagged(&|_, _| tag), op, tol)
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
    // each round and chamfer, at every corner of its two bounds (`brep::corners`, X1)
    for (k, l) in lowered.iter().enumerate() {
        let Some(l) = l else { continue };
        let tag = k as u32 + 1;
        let (a, b) = (l.a as u32 + 1, l.b as u32 + 1);
        // exact only where it builds what the set says: no corner of a charted step (a chart's
        // step offsets as no line or circle), and a flat only where its sum of distances is a line
        // or a conic (`corners::sum`: not between two circles)
        for lp in &region.loops {
            for i in 0..lp.len() {
                let (st, nx) = (&lp[i], &lp[(i + 1) % lp.len()]);
                let named = (st.tag == a && nx.tag == b) || (st.tag == b && nx.tag == a);
                let curved = |x: &crate::brep::planar::Step| matches!(x.seg, Seg::Arc { .. });
                if named && (st.chart.is_some() || nx.chart.is_some()
                    || (matches!(l.kind, Lower::Flat { .. }) && curved(st) && curved(nx))) {
                    return Err(format!("`{}`: a round or a flat at a charted corner, or a flat \
                                        between two arcs, is read point by point", s.name).into());
                }
            }
        }
        region = match l.kind {
            Lower::Round { r } => crate::brep::corners::round(&region, &[a], &[b], r, tag),
            Lower::Flat { la, lb } => crate::brep::corners::summed(&region, &[a], &[b], [la, lb], tag),
        }.map_err(|e| format!("`{}`: {e}", s.name))?;
    }
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
    let names = terms.iter().map(|t| t.name.clone()).collect();
    Ok(Meridian { origin, axis, seam, region, names })
}

fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 { a[0] * b[1] - a[1] * b[0] }

/// The meridian's loops as the faceted kernel turns them: polygons in the half-plane (`u` along
/// the seam, `v` along the axis), each arc chorded as a face's arcs are (`tessellate_arc`), a conic
/// as an arc of its length and turn would be, every side named by the term it came from — the loop
/// enclosing most first, the holes after.
pub(super) fn face_polys(m: &Meridian, unit: f64) -> Vec<super::profile::FacePoly> {
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
                    let turn = cross2(t0, t1).datan2(t0[0] * t1[0] + t0[1] * t1[1]).abs().max(1e-9);
                    let r = step.seg.length() / turn;
                    let n = super::profile::tessellate_arc((0.0, 0.0), r, 0.0, turn, unit).len() - 1;
                    for j in 0..n {
                        let p = step.seg.point(j as f64 / n as f64);
                        pts.push((p[0], p[1]));
                        of.push((k, true));
                    }
                }
                Seg::Arc { c, r, a0, sweep } => {
                    let ring = super::profile::tessellate_arc((c[0], c[1]), r, a0, sweep, unit);
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
        let curved = vec![None; names.len()];
        super::profile::FacePoly { pts, of, names, basis, curved }
    }).collect()
}

/// **A lowering entry** (#183 step 7): a term a region's body states that the meridian builds
/// exactly — `std.Rounded(a, b, r)`, within reach of a ball of `r` lying `r` within both of two of
/// the region's own bounds, and `std.Chamfered(a, b, la, lb)`, short of the flat between their
/// surfaces — at the bounds `a` and `b` (their indices among the terms).
struct Lowering {
    a: usize,
    b: usize,
    kind: Lower,
}

enum Lower {
    Round { r: f64 },
    Flat { la: f64, lb: f64 },
}

/// The lowering a term is, where it is one: its sets each the region's own bound (a shape it
/// states, at its level — moved in by the ball's radius for a round's centres), never what the
/// set is for.
fn lowering(sk: &Sketch, terms: &[RegionTerm], t: &RegionTerm) -> Option<Lowering> {
    use crate::model::Target;
    let RegionShape::Measured { of } = &t.shape else { return None };
    let lo = t.lo.read(sk)?;
    let level = |u: &RegionTerm| -> Option<(f64, f64)> {
        Some((u.lo.read(sk)?, u.grow.as_ref().map_or(Some(0.0), |g| g.read(sk))?))
    };
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
                .then_some(Lowering { a: ka, b: kb, kind: Lower::Round { r } })
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
                .then_some(Lowering { a: ka, b: kb, kind: Lower::Flat { la: lo / cb, lb: lo / ca } })
        }
        _ => None,
    }
}

/// Whether a term is read only point by point: a sum of measurements, or a face's (#183).
fn point_by_point(shape: &RegionShape) -> bool {
    matches!(shape, RegionShape::Measured { .. } | RegionShape::FaceSide { .. }
        | RegionShape::Angle { .. } | RegionShape::Ends { .. })
}

/// Region solid `solid` read point by point (`solid::field::RegionField`): its terms in general
/// position, each the exact signed distance from its surface.  Its support is the box of the
/// planes every term lies within — a ball's six, a cylinder's four about its line, a cone's four
/// about its axis, a half-space's own — and refused where they close nothing.
pub fn general(sk: &Sketch, solid: usize) -> Result<crate::solid::field::RegionField, String> {
    use crate::solid::field::region::{polytope_box, Measure, Term};
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Region { terms, .. } = &s.def else {
        return Err(format!("`{}` is not a region", s.name));
    };
    let point = |i: u32| sk.world_point(i as usize);
    use crate::solid::field::region::Signed;
    let mut out = Vec::new();
    let mut planes: Vec<(V, f64)> = Vec::new();
    // the bodies whose faces it reads, each read once
    let mut read = Vec::new();
    let mut reads_faces = false;
    let named = |m: String| format!("{}: {m}", s.name);
    // the six planes of the cube about `c` reaching `r`
    let cube = |planes: &mut Vec<(V, f64)>, c: V, r: f64| {
        for k in 0..3 {
            let mut n = [0.0; 3];
            n[k] = 1.0;
            planes.push((n, c[k] + r));
            n[k] = -1.0;
            planes.push((n, -c[k] + r));
        }
    };
    // the half-spaces where the ordinate `n · x - d` holds to `t`'s number
    let slab = |planes: &mut Vec<(V, f64)>, t: &RegionTerm, n: V, d: f64, lo: f64| match t.cmp {
        Cmp::Le => planes.push((n, d + lo)),
        Cmp::Ge => planes.push((scale(n, -1.0), -(d + lo))),
        Cmp::In => {
            planes.push((n, d + t.hi.unwrap_or(lo)));
            planes.push((scale(n, -1.0), -(d + lo)));
        }
    };
    // a gate, or a term read by a closure: negative where it holds
    let signed = |f: std::sync::Arc<dyn Fn(V) -> f64 + Send + Sync>| {
        Term { measure: Measure::Signed(Signed(f)), cmp: Cmp::Le, lo: 0.0, hi: None, grow: 0.0 }
    };
    for t in terms {
        let lo = t.lo.value(sk).map_err(named)?;
        let grow = t.grow.as_ref().map_or(Ok(0.0), |g| g.value(sk)).map_err(named)?;
        let reach = t.hi.unwrap_or(lo).max(lo);
        let within = t.cmp != Cmp::Ge;
        reads_faces |= super::faces::reads_faces(&t.shape);
        let measure = match t.shape {
            RegionShape::Measured { ref of } => {
                // each set measured from, read once
                let sets: Vec<Option<Result<super::conditions::SetDistance, String>>> = of.iter()
                    .map(|(t, _)| match t {
                        crate::model::Target::Set { terms, .. } => {
                            Some(super::conditions::SetDistance::of(sk, terms, &mut read))
                        }
                        _ => None,
                    }).collect();
                if t.cmp != Cmp::In && never(of, &sets, t.cmp) {
                    return Ok(crate::solid::field::RegionField::empty());
                }
                let m = measured(sk, of, &sets, &mut read)?;
                // a sum of distances, each from a point, with no number below nothing, is within
                // the ball about each point whose distance alone reaches the bound
                if t.cmp == Cmp::Le && of.iter().all(|&(_, c)| c > 0.0) {
                    for (e, c) in of {
                        let crate::model::Target::Ent(e) = e else { continue };
                        if e.kind == crate::model::EntKind::Point {
                            cube(&mut planes, point(e.i() as u32), (lo + grow) / c);
                        }
                    }
                }
                let n: f64 = of.iter().map(|&(_, c)| c.abs()).sum();
                out.push(Term { measure: m, cmp: t.cmp, lo: lo / n, hi: t.hi.map(|h| h / n), grow: grow / n });
                continue;
            }
            RegionShape::FaceSide { ref face } => {
                let r = super::faces::reading(sk, face, &mut read).map_err(named)?;
                if let Some((n, d)) = r.half_space() {
                    slab(&mut planes, t, n, d, if t.cmp == Cmp::In { lo } else { lo + grow });
                }
                reads_faces = true;
                Measure::Signed(Signed(std::sync::Arc::new(move |p| r.side(p))))
            }
            RegionShape::Angle { ref a, ref b } => {
                let ra = super::faces::reading(sk, a, &mut read).map_err(named)?;
                let rb = super::faces::reading(sk, b, &mut read).map_err(named)?;
                let edges = ra.body.edges_between(&ra.faces, &rb.faces);
                let body = ra.body.clone();
                let (cmp, lo, hi) = (t.cmp, lo, t.hi.unwrap_or(lo));
                reads_faces = true;
                // which way an edge turns is the edge's: against a half turn, a gate no edge
                // passes holds nowhere
                let half = (lo - std::f64::consts::PI).abs() < 1e-12 && cmp != Cmp::In;
                if half && !edges.iter().any(|&e| body.convex(e).is_some_and(|c| c == (cmp == Cmp::Le))) {
                    return Ok(crate::solid::field::RegionField::empty());
                }
                // a gate: the field reads whether it holds, never how near it is to failing
                let gate = move |p: V| -> f64 {
                    let holds = body.angle(&edges, p).is_some_and(|a| match cmp {
                        Cmp::Le => a <= lo,
                        Cmp::Ge => a >= lo,
                        Cmp::In => a >= lo && a <= hi,
                    });
                    if holds { -GATE } else { GATE }
                };
                out.push(signed(std::sync::Arc::new(gate)));
                continue;
            }
            RegionShape::Ends { ref edge } => {
                let crate::model::Target::Set { terms: inner, .. } = edge.as_ref() else {
                    return Err(format!("{}: where an edge ends is read of a set", s.name));
                };
                let set = super::conditions::SetDistance::of(sk, inner, &mut read).map_err(named)?;
                let within = super::conditions::ends(&set).map_err(named)?;
                reads_faces = true;
                // a run that ends nowhere is cut off by nothing
                let f = move |p: V| { let v = within(p); if v == f64::NEG_INFINITY { -GATE } else { v } };
                out.push(signed(std::sync::Arc::new(f)));
                continue;
            }
            ref shape => {
                let m = measure_of(sk, shape)?.ok_or("a region bounds distances, angles and planes")?;
                match m {
                    Measure::Point(c) if within => cube(&mut planes, c, reach),
                    Measure::Line { a, d } if within => {
                        let (e1, e2) = across(d);
                        for e in [e1, e2, scale(e1, -1.0), scale(e2, -1.0)] {
                            planes.push((e, dot(e, a) + reach));
                        }
                    }
                    Measure::Cone { a, d } if within && reach + 1e-12 < FRAC_PI_2 && grow <= 0.0 => {
                        // within the four planes through the apex leaning `reach` off the axis
                        let (e1, e2) = across(d);
                        let tan = reach.dtan();
                        for e in [e1, e2, scale(e1, -1.0), scale(e2, -1.0)] {
                            let n = sub(e, scale(d, tan));
                            planes.push((n, dot(n, a)));
                        }
                    }
                    // the ordinate `n · (x - o)` against its number
                    Measure::Ordinate { o, n } => slab(&mut planes, t, n, dot(n, o), lo),
                    _ => {}
                }
                m
            }
        };
        out.push(Term { measure, cmp: t.cmp, lo, hi: t.hi, grow });
    }
    // a region of a body's faces has its support where its planes close it, and is otherwise
    // read with none known: a cut needs none, and a box asked of it says so
    let support = match polytope_box(&planes) {
        Some(b) => Some(b),
        None if reads_faces => None,
        None => {
            return Err(format!("`{}` has no end: bound it all round (a ball, two planes, …)", s.name))
        }
    };
    Ok(crate::solid::field::RegionField::new(out, support))
}

/// What a ball, a cylinder, a cone or a half-space measures (`None` for any other shape): the
/// distance from its centre or its line, the angle off its axis, the ordinate along its normal.
pub(super) fn measure_of(sk: &Sketch, shape: &RegionShape)
    -> Result<Option<crate::solid::field::region::Measure>, String> {
    use crate::solid::field::region::Measure;
    let point = |i: u32| sk.world_point(i as usize);
    let line = |l: u32, what: &str| -> Result<(V, V), String> {
        let l = &sk.lines[l as usize];
        let (a, b) = (point(l.p1), point(l.p2));
        Ok((a, normalised(sub(b, a)).ok_or(format!("a {what} about no line"))?))
    };
    Ok(Some(match *shape {
        RegionShape::Ball { center } => Measure::Point(point(center)),
        RegionShape::Cylinder { line: l } => { let (a, d) = line(l, "cylinder")?; Measure::Line { a, d } }
        RegionShape::Cone { axis } => { let (a, d) = line(axis, "cone")?; Measure::Cone { a, d } }
        RegionShape::Plane { plane, from, sign } => Measure::Ordinate {
            o: point(from),
            n: scale(sk.basis(plane as usize).normal(), sign),
        },
        _ => return Ok(None),
    }))
}

/// A sum of measurements of a point (`RegionShape::Measured`), divided through by the sum of its
/// numbers' sizes so it is one-Lipschitz as every term of the field is: each the distance from a
/// point, a line produced or an axis, or a plane.
fn measured(sk: &Sketch, of: &[(crate::model::Target, f64)],
            sets: &[Option<Result<super::conditions::SetDistance, String>>],
            read: &mut super::faces::Read)
    -> Result<crate::solid::field::region::Measure, String> {
    use crate::model::{EntKind, Target};
    use crate::solid::field::region::{Measure, Signed};
    type Dist = Box<dyn Fn(V) -> f64 + Send + Sync>;
    let n: f64 = of.iter().map(|&(_, c)| c.abs()).sum();
    if n <= 0.0 {
        return Err("a bound on measurements measures something".to_string());
    }
    let mut reads: Vec<(Dist, f64)> = Vec::new();
    // a set compared with one it lies in: the larger read past the smaller (`conditions::Rest`)
    let mut rest: Vec<Option<std::sync::Arc<super::conditions::Rest>>> = vec![None; of.len()];
    for (i, (ti, ci)) in of.iter().enumerate() {
        for (j, (tj, cj)) in of.iter().enumerate() {
            let (Target::Set { terms: e, .. }, Target::Set { terms: m, .. }) = (ti, tj) else { continue };
            let (Some(Ok(edge)), Some(Ok(meeting))) = (&sets[i], &sets[j]) else { continue };
            if *ci > 0.0 && (ci + cj).abs() <= 1e-12 * ci.abs() && rest[j].is_none() {
                rest[j] = super::conditions::Rest::of(sk, e, m, edge, meeting, read)
                    .map(std::sync::Arc::new);
            }
        }
    }
    for (k, (target, c)) in of.iter().enumerate() {
        let d: Dist = match target {
            Target::Set { .. } if rest[k].is_some() => {
                let r = rest[k].clone().expect("checked");
                Box::new(move |p| r.distance(p))
            }
            Target::Ent(e) => {
                let m = match e.kind {
                    EntKind::Point => Measure::Point(sk.world_point(e.i())),
                    EntKind::Line => {
                        let l = &sk.lines[e.i()];
                        let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
                        let d = normalised(sub(b, a)).ok_or("a distance from a line of no length")?;
                        Measure::Line { a, d }
                    }
                    EntKind::Axis => {
                        let x = &sk.axes[e.i()];
                        let read = |k: [u32; 3]| k.map(|p| sk.params[p as usize].value);
                        let d = normalised(read(x.d)).ok_or("a distance from an axis of no direction")?;
                        Measure::Line { a: read(x.a), d }
                    }
                    EntKind::Plane => {
                        let b = sk.basis(e.i());
                        Measure::Ordinate { o: b.o, n: b.normal() }
                    }
                    _ => return Err(format!("a distance from {} is not measured yet", e.kind.a())),
                };
                Box::new(move |p| m.distance(p))
            }
            Target::Face(f) => {
                let r = super::faces::reading(sk, f, read)?;
                let surface = f.surface;
                Box::new(move |p| r.distance(p, surface))
            }
            Target::Set { .. } => {
                let set = sets[k].clone().expect("a set's distance, read")?;
                Box::new(move |p| set.distance(p))
            }
        };
        reads.push((d, c / n));
    }
    Ok(Measure::Signed(Signed(std::sync::Arc::new(move |p| {
        let v: f64 = reads.iter().map(|(m, c)| c * m(p)).sum();
        // a distance from a set that holds nowhere is past every number: what it bounds fails
        if v.is_nan() { GATE } else { v.clamp(-GATE, GATE) }
    }))))
}

/// Whether a sum of measurements can hold nowhere: it bounds from above a distance from a set
/// that is empty (faces that do not meet), with nothing to take it back.
fn never(of: &[(crate::model::Target, f64)],
         sets: &[Option<Result<super::conditions::SetDistance, String>>], cmp: Cmp) -> bool {
    use super::conditions::SetDistance;
    let empty = |s: &Option<Result<SetDistance, String>>| match s {
        Some(Ok(SetDistance::Edges { edges, .. })) => edges.is_empty(),
        Some(Ok(SetDistance::Points(ps))) => ps.is_empty(),
        Some(Ok(SetDistance::Patch(r))) => r.faces.is_empty(),
        _ => false,
    };
    let up = |c: f64| if cmp == Cmp::Ge { -c } else { c };
    of.iter().zip(sets).any(|((_, c), s)| up(*c) > 0.0 && empty(s))
}

/// What a gate's term reads where it holds and where it fails: far past any distance, so the
/// field reads it only by its sign.
const GATE: f64 = 1e30;

/// Region solid `solid` read in general position, where its meridian refused for a reason a
/// reading point by point answers (its terms share no axis): that reading, or why it is none.
/// `None` where the meridian reads it or refuses it outright.  Remembered against what it reads
/// (`Sketch::general_cache`).
pub(crate) fn in_general_position(sk: &Sketch, solid: usize)
    -> Option<Result<crate::solid::field::RegionField, String>> {
    if !matches!(sk.solids.get(solid).map(|s| &s.def), Some(SolidDef::Region { .. })) {
        return None;
    }
    let key = super::reads(sk, solid, 0.0);
    if let Some((old, g)) = sk.general_cache.borrow().get(&solid) {
        if *old == key {
            return g.clone();
        }
    }
    let g = match folded(sk, solid) {
        Ok(_) => None,
        Err(r) if r.outright => None,
        Err(_) => Some(general(sk, solid)),
    };
    sk.general_cache.borrow_mut().insert(solid, (key, g.clone()));
    g
}

/// Whether region solid `solid` is read in general position, and that reading made.
pub fn in_general(sk: &Sketch, solid: usize) -> bool {
    in_general_position(sk, solid).is_some_and(|g| g.is_ok())
}

/// A region solid's reading as `validate` asks it: its meridian, or else its reading in general
/// position, or the meridian's refusal.
pub fn readable(sk: &Sketch, solid: usize) -> Result<(), String> {
    match in_general_position(sk, solid) {
        Some(g) => g.map(|_| ()),
        None => meridian(sk, solid).map(|_| ()),
    }
}
