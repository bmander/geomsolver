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
use crate::space::{add, cross, dot, norm, scale, sub};
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

/// The meridian of region solid `solid`, from its terms as the drawing stands: refused where its
/// terms share no axis (a ball off it, a cone about another line, a plane not square to it),
/// where they leave no material, or where they leave material without end.
pub fn meridian(sk: &Sketch, solid: usize) -> Result<Meridian, String> {
    let s = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Region { terms, .. } = &s.def else {
        return Err(format!("`{}` is not a region", s.name));
    };
    if terms.is_empty() {
        return Err(format!("`{}` bounds nothing: a solid is a region", s.name));
    }
    let point = |i: u32| sk.world_point(i as usize);
    let line = |l: u32| {
        let l = &sk.lines[l as usize];
        (point(l.p1), point(l.p2))
    };
    let unit = |v: V| {
        let n = norm(v);
        (n > 0.0).then(|| scale(v, 1.0 / n))
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
            unit(sub(b, a)).map(|d| (a, d))
        }
        _ => None,
    });
    let normal = terms.iter().find_map(|t| match t.shape {
        RegionShape::Plane { plane, from, .. } => {
            Some((terms.iter().find_map(ball).unwrap_or(point(from)), sk.basis(plane as usize).normal()))
        }
        _ => None,
    });
    let centres: Vec<V> = terms.iter().filter_map(ball).collect();
    let through = || {
        let c0 = *centres.first()?;
        let other = centres.iter().find_map(|&c| unit(sub(c, c0)));
        Some((c0, other.unwrap_or([0.0, 0.0, 1.0])))
    };
    let (origin, axis) = around.or(normal).or_else(through).ok_or("a region with no axis")?;
    // how far everything stands from the origin, for the tolerances and the box
    let mut size: f64 = 1.0;
    for t in terms {
        size = size.max(t.lo.abs()).max(t.hi.unwrap_or(0.0).abs());
        let at = match t.shape {
            RegionShape::Ball { center } => point(center),
            RegionShape::Cylinder { line: l } | RegionShape::Cone { axis: l } => line(l).0,
            RegionShape::Plane { from, .. } => point(from),
        };
        size = size.max(norm(sub(at, origin)));
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
    let empty = Region { loops: Vec::new() };
    let what = |t: &RegionTerm| if t.name.is_empty() { "a bound".to_string() } else { format!("`{}`", t.name) };
    // the points a term reads at most `v` at, in the half-plane
    let within = |t: &RegionTerm, v: f64| -> Result<Region, String> {
        Ok(match t.shape {
            RegionShape::Ball { center } => {
                let c = point(center);
                if off(c) > tol {
                    return Err(format!("{}'s centre is off the region's axis", what(t)));
                }
                if v <= 0.0 {
                    return Ok(empty.clone());
                }
                let zc = z(c);
                Region::plain(vec![vec![
                    Seg::Arc { c: [0.0, zc], r: v, a0: -FRAC_PI_2, sweep: PI },
                    Seg::Line { a: [0.0, zc + v], b: [0.0, zc - v] },
                ]])
            }
            RegionShape::Cylinder { line: l } => {
                let (a, b) = line(l);
                if off(a) > tol || off(b) > tol {
                    return Err(format!("{} is about a line other than the region's axis", what(t)));
                }
                if v <= 0.0 {
                    return Ok(empty.clone());
                }
                rect(0.0, v, -reach, reach)
            }
            RegionShape::Cone { axis: l } => {
                let (a, b) = line(l);
                if off(a) > tol || off(b) > tol {
                    return Err(format!("{} is about a line other than the region's axis", what(t)));
                }
                let s = dot(sub(b, a), axis).signum();
                let wedge = |s: f64, half: f64| {
                    let za = z(a);
                    let far = za + s * reach;
                    let width = reach * half.dtan();
                    let pts = if s > 0.0 {
                        [[0.0, za], [width, far], [0.0, far]]
                    } else {
                        [[0.0, za], [0.0, far], [width, far]]
                    };
                    Region::plain(vec![(0..3).map(|k| Seg::Line { a: pts[k], b: pts[(k + 1) % 3] })
                        .collect()])
                };
                if v <= 0.0 {
                    empty.clone()
                } else if v < FRAC_PI_2 - 1e-12 {
                    wedge(s, v)
                } else if v >= PI {
                    rect(0.0, reach, -reach, reach)
                } else if (v - FRAC_PI_2).abs() <= 1e-12 {
                    let za = z(a);
                    if s > 0.0 { rect(0.0, reach, za, reach) } else { rect(0.0, reach, -reach, za) }
                } else {
                    // wider than square: all but the cone the other way, of what is left of half
                    // a turn
                    boolean(&rect(0.0, reach, -reach, reach), &wedge(-s, PI - v), Op::Cut, tol)?
                }
            }
            RegionShape::Plane { plane, from, sign } => {
                let n = sk.basis(plane as usize).normal();
                if norm(cross(n, axis)) > 1e-9 {
                    return Err(format!("{}'s plane is not square to the region's axis", what(t)));
                }
                // the ordinate, `sign · n · (x - from)`, along the axis
                let k = sign * dot(n, axis).signum();
                let edge = z(point(from)) + k * v;
                if k > 0.0 { rect(0.0, reach, -reach, edge) } else { rect(0.0, reach, edge, reach) }
            }
        })
    };
    let mut region = rect(0.0, reach, -reach, reach);
    for (k, t) in terms.iter().enumerate() {
        let tag = k as u32 + 1;
        let tagged = |r: Region| r.tagged(&|_, _| tag);
        let fold = |region: &Region, v: f64, op: Op| -> Result<Region, String> {
            boolean(region, &tagged(within(t, v)?), op, tol)
                .map_err(|e| format!("{}: {e}", what(t)))
        };
        region = match t.cmp {
            Cmp::Le => fold(&region, t.lo, Op::Common)?,
            Cmp::Ge => fold(&region, t.lo, Op::Cut)?,
            Cmp::In => {
                let hi = fold(&region, t.hi.unwrap_or(t.lo), Op::Common)?;
                fold(&hi, t.lo, Op::Cut)?
            }
        };
    }
    if region.loops.is_empty() || region.area().abs() <= tol * tol {
        return Err(format!("`{}` leaves no material: its bounds have no point in common", s.name));
    }
    let touches = region.loops.iter().flatten().any(|step| {
        [step.start(), step.end()].iter().any(|p| p[0] >= reach - tol || p[1].abs() >= reach - tol)
    });
    if touches {
        return Err(format!("`{}` has no end: bound it all round (a ball, two planes, …)", s.name));
    }
    // the half-plane's direction: square to the axis, from the world axis least along it
    let least = (0..3).min_by(|&i, &j| axis[i].abs().total_cmp(&axis[j].abs())).unwrap_or(0);
    let mut e = [0.0; 3];
    e[least] = 1.0;
    let seam = unit(cross(cross(axis, e), axis)).ok_or("a region with no half-plane")?;
    let names = terms.iter().map(|t| t.name.clone()).collect();
    Ok(Meridian { origin, axis, seam, region, names })
}

/// The bounds over a region's whole extent, a point at a time: for a test or a probe, whether a
/// point in space is in it (`Region::contains` in its half-plane).
pub fn contains(m: &Meridian, p: V) -> bool {
    let d = sub(p, m.origin);
    let along = dot(d, m.axis);
    let r = norm(sub(d, scale(m.axis, along)));
    m.region.contains([r, along])
}

/// The point of the meridian's half-plane at `(r, along)`.
pub fn at(m: &Meridian, q: [f64; 2]) -> V {
    add(m.origin, add(scale(m.seam, q[0]), scale(m.axis, q[1])))
}

/// The meridian's loops as the faceted kernel turns them: polygons in the half-plane (`u` along
/// the seam, `v` along the axis), each arc chorded so no chord leaves it by more than `unit`, every
/// side named by the term it came from — the loop enclosing most first, the holes after.
pub(super) fn face_polys(m: &Meridian, unit: f64) -> Vec<super::profile::FacePoly> {
    let basis = crate::plane::Basis { u: m.seam, v: m.axis, o: m.origin };
    let mut loops: Vec<(f64, &Vec<crate::brep::planar::Step>)> = m.region.loops.iter()
        .map(|l| (Region { loops: vec![l.clone()] }.area().abs(), l))
        .collect();
    loops.sort_by(|a, b| b.0.total_cmp(&a.0));
    loops.into_iter().map(|(_, l)| {
        let (mut pts, mut of, mut names) = (Vec::new(), Vec::new(), Vec::new());
        for step in l {
            let k = names.len();
            names.push(m.name(step.tag));
            let n = match step.seg {
                Seg::Line { .. } => 1,
                Seg::Arc { r, sweep, .. } => {
                    let turn = (8.0 * unit.max(1e-12) / r.max(1e-12)).sqrt().min(FRAC_PI_2);
                    (sweep.abs() / turn).ceil().max(1.0) as usize
                }
            };
            for j in 0..n {
                let p = step.point(j as f64 / n as f64);
                pts.push((p[0], p[1]));
                of.push((k, matches!(step.seg, Seg::Arc { .. })));
            }
        }
        let curved = vec![None; names.len()];
        super::profile::FacePoly { pts, of, names, basis, curved }
    }).collect()
}
