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

fn fold(sk: &Sketch, solid: usize) -> Result<Meridian, String> {
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
    };
    // each term's low end as the drawing stands (an unknown's, solved; a cone's half within the
    // half turn its row reads, `expr::sync_free`)
    let lows = terms.iter().map(|t| t.lo.value(sk).map_err(|m| format!("{}: {m}", s.name)))
        .collect::<Result<Vec<f64>, String>>()?;
    // how far everything stands from the origin, for the tolerances and the box
    let mut size: f64 = 1.0;
    for (t, lo) in terms.iter().zip(&lows) {
        size = size.max(lo.abs()).max(t.hi.unwrap_or(0.0).abs());
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
    // the points a term reads at most `v` at, in the half-plane
    let within = |t: &RegionTerm, v: f64| -> Result<Region, String> {
        let plane = matches!(t.shape, RegionShape::Plane { .. });
        if !plane && stands(t).into_iter().any(|p| off(p) > tol) {
            return Err(match t.shape {
                RegionShape::Ball { .. } => format!("{}'s centre is off the region's axis", what(t)),
                _ => format!("{} is about a line other than the region's axis", what(t)),
            });
        }
        // a measure from a point or a line is never less than nothing
        if !plane && v <= 0.0 {
            return Ok(Region { loops: Vec::new() });
        }
        Ok(match t.shape {
            RegionShape::Ball { center } => Region::half_disc(z(point(center)), v),
            RegionShape::Cylinder { .. } => rect(0.0, v, -reach, reach),
            RegionShape::Cone { axis: l } => {
                let (a, b) = line(l);
                let s = dot(sub(b, a), axis).signum();
                let za = z(a);
                let wedge = |s: f64, half: f64| {
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
                if v < FRAC_PI_2 - 1e-12 {
                    wedge(s, v)
                } else if v >= PI {
                    rect(0.0, reach, -reach, reach)
                } else if (v - FRAC_PI_2).abs() <= 1e-12 {
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
    for (k, (t, &lo)) in terms.iter().zip(&lows).enumerate() {
        let tag = k as u32 + 1;
        let fold = |region: &Region, v: f64, op: Op| -> Result<Region, String> {
            boolean(region, &within(t, v)?.tagged(&|_, _| tag), op, tol)
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
    if region.loops.is_empty() || region.area().abs() <= tol * tol {
        return Err(format!("`{}` leaves no material: its bounds have no point in common", s.name));
    }
    let touches = region.loops.iter().flatten().any(|step| {
        [step.start(), step.end()].iter().any(|p| p[0] >= reach - tol || p[1].abs() >= reach - tol)
    });
    if touches {
        return Err(format!("`{}` has no end: bound it all round (a ball, two planes, …)", s.name));
    }
    // the half-plane's direction: square to the axis, toward the world axis least along it
    let seam = scale(across(axis).1, -1.0);
    let names = terms.iter().map(|t| t.name.clone()).collect();
    Ok(Meridian { origin, axis, seam, region, names })
}

/// The meridian's loops as the faceted kernel turns them: polygons in the half-plane (`u` along
/// the seam, `v` along the axis), each arc chorded as a face's arcs are (`tessellate_arc`), every
/// side named by the term it came from — the loop enclosing most first, the holes after.
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
