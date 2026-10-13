//! **A region read point by point** (#183): a region solid whose terms share no axis — so no
//! meridian folds them — read as `solid::field::RegionField`, each term the exact signed distance
//! from its surface, the region their largest; a sum of measurements and a body's faces leaves of
//! the same kind.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::model::{RegionShape, RegionTerm, Sketch, SolidDef};
use crate::space::{across, dot, normalised, scale, sub};
use crate::syntax::Cmp;
use std::f64::consts::FRAC_PI_2;

type V = [f64; 3];

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
        let (lo, grow) = t.value(sk).map_err(named)?;
        let reach = t.hi.unwrap_or(lo).max(lo);
        let within = t.cmp != Cmp::Ge;
        reads_faces |= crate::solid::faces::reads_faces(&t.shape);
        let measure = match t.shape {
            RegionShape::Measured { ref of } => {
                // each set measured from, read once
                let sets: Vec<Option<Result<crate::solid::conditions::SetDistance, String>>> = of.iter()
                    .map(|(t, _)| match t {
                        crate::model::Target::Set { terms, .. } => {
                            Some(crate::solid::conditions::SetDistance::of(sk, terms, &mut read))
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
                let r = crate::solid::faces::reading(sk, face, &mut read).map_err(named)?;
                if let Some((n, d)) = r.half_space() {
                    slab(&mut planes, t, n, d, if t.cmp == Cmp::In { lo } else { lo + grow });
                }
                reads_faces = true;
                Measure::Signed(Signed(std::sync::Arc::new(move |p| r.side(p))))
            }
            RegionShape::Angle { ref a, ref b } => {
                let ra = crate::solid::faces::reading(sk, a, &mut read).map_err(named)?;
                let rb = crate::solid::faces::reading(sk, b, &mut read).map_err(named)?;
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
                let set = crate::solid::conditions::SetDistance::of(sk, inner, &mut read).map_err(named)?;
                let within = crate::solid::conditions::ends(&set).map_err(named)?;
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
pub(crate) fn measure_of(sk: &Sketch, shape: &RegionShape)
    -> Result<Option<crate::solid::field::region::Measure>, String> {
    use crate::solid::field::region::Measure;
    let point = |i: u32| sk.world_point(i as usize);
    let line = |l: u32, what: &str| -> Result<(V, V), String> {
        line_of(sk, l as usize).ok_or(format!("a {what} about no line"))
    };
    Ok(Some(match *shape {
        RegionShape::Ball { center } => Measure::Point(point(center)),
        RegionShape::Cylinder { line: l } => { let (a, d) = line(l, "cylinder")?; Measure::Line { a, d } }
        RegionShape::Cone { axis } => { let (a, d) = line(axis, "cone")?; Measure::Cone { a, d } }
        RegionShape::Plane { from, sign, .. } => Measure::Ordinate {
            o: point(from),
            n: scale(shape.normal(sk).ok_or("a half-space along a line of no length")?, sign),
        },
        _ => return Ok(None),
    }))
}

/// Line `l` from its first end along its unit direction, `None` where it has no length: what a
/// cylinder or a cone turns about and a distance from a line is measured from.
fn line_of(sk: &Sketch, l: usize) -> Option<(V, V)> {
    let l = &sk.lines[l];
    let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
    Some((a, normalised(sub(b, a))?))
}

/// What a distance from an entity measures: from a point, a line produced or an axis, or a
/// plane.
fn ent_measure(sk: &Sketch, e: crate::model::EntRef) -> Result<crate::solid::field::region::Measure, String> {
    use crate::model::EntKind;
    use crate::solid::field::region::Measure;
    Ok(match e.kind {
        EntKind::Point => Measure::Point(sk.world_point(e.i())),
        EntKind::Line => {
            let (a, d) = line_of(sk, e.i()).ok_or("a distance from a line of no length")?;
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
    })
}

/// A sum of measurements of a point (`RegionShape::Measured`), divided through by the sum of its
/// numbers' sizes so it is one-Lipschitz as every term of the field is: each the distance from a
/// point, a line produced or an axis, or a plane.
fn measured(sk: &Sketch, of: &[(crate::model::Target, f64)],
            sets: &[Option<Result<crate::solid::conditions::SetDistance, String>>],
            read: &mut crate::solid::faces::Read)
    -> Result<crate::solid::field::region::Measure, String> {
    use crate::model::Target;
    use crate::solid::field::region::{Measure, Signed};
    type Dist = Box<dyn Fn(V) -> f64 + Send + Sync>;
    let n: f64 = of.iter().map(|&(_, c)| c.abs()).sum();
    if n <= 0.0 {
        return Err("a bound on measurements measures something".to_string());
    }
    let mut reads: Vec<(Dist, f64)> = Vec::new();
    // a set compared with one it lies in: the larger read past the smaller (`conditions::Rest`)
    let mut rest: Vec<Option<std::sync::Arc<crate::solid::conditions::Rest>>> = vec![None; of.len()];
    for (i, (ti, ci)) in of.iter().enumerate() {
        for (j, (tj, cj)) in of.iter().enumerate() {
            let (Target::Set { terms: e, .. }, Target::Set { terms: m, .. }) = (ti, tj) else { continue };
            let (Some(Ok(edge)), Some(Ok(meeting))) = (&sets[i], &sets[j]) else { continue };
            if *ci > 0.0 && (ci + cj).abs() <= 1e-12 * ci.abs() && rest[j].is_none() {
                rest[j] = crate::solid::conditions::Rest::of(sk, e, m, edge, meeting, read)
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
                let m = ent_measure(sk, *e)?;
                Box::new(move |p| m.distance(p))
            }
            Target::Face(f) => {
                let r = crate::solid::faces::reading(sk, f, read)?;
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
         sets: &[Option<Result<crate::solid::conditions::SetDistance, String>>], cmp: Cmp) -> bool {
    use crate::solid::conditions::SetDistance;
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
