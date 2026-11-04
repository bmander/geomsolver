//! The closed forms the rounds tests hold their volumes to: the Rust fillet stack's (#79–#110) —
//! a round's ring, run and section beside a cylinder, Steiner's rounded box, a mitre, the rolled
//! ball's quadrature (#85) and a chamfer's flat swept along its edge — and, in the meridian
//! (`meridian`), the knob's neck and the knob rounded whole.  Worked out here from planes,
//! cylinders and cones written out by hand, never asked of the core, so a test and the thing it
//! tests share no implementation.  `round_lowering`, `rounds`, `rounds_examples` and the two
//! exploration files X1 (`fillet_meridian`) and X2 (`fillet_definition`) read them from here.
use std::f64::consts::PI;

pub type V = [f64; 3];

pub fn add(a: V, b: V) -> V { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
pub fn sub(a: V, b: V) -> V { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
pub fn scale(a: V, k: f64) -> V { [a[0] * k, a[1] * k, a[2] * k] }
pub fn dot(a: V, b: V) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
pub fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V) -> f64 { dot(a, a).sqrt() }
pub fn unit(a: V) -> V { scale(a, 1.0 / norm(a)) }

/// A 3 × 3 system `rows · x = rhs`, by Cramer's rule; none where it is singular.
pub fn solve3(rows: [V; 3], rhs: V) -> Option<V> {
    let det = dot(rows[0], cross(rows[1], rows[2]));
    if det.abs() < 1e-12 {
        return None;
    }
    let col = |k: usize| {
        let mut m = rows;
        for (i, row) in m.iter_mut().enumerate() {
            row[k] = rhs[i];
        }
        dot(m[0], cross(m[1], m[2])) / det
    };
    Some([col(0), col(1), col(2)])
}

// -- closed forms ------------------------------------------------------------------------------


/// The stack's Steiner volume of a box `l × w × h` rounded all over: the inset box grown by a
/// ball.
pub fn rounded_box([l, w, h]: [f64; 3], r: f64) -> f64 {
    let (a, b, c) = (l - 2.0 * r, w - 2.0 * r, h - 2.0 * r);
    a * b * c + 2.0 * r * (a * b + b * c + a * c) + PI * r * r * (a + b + c)
        + 4.0 * PI * r * r * r / 3.0
}

/// The stack's mitre: a convex polygon's prism with one cap's rim rounded alone loses `∫δ` along
/// its perimeter less `∫δ²` at each corner (`cots` the sum of `cot(θ/2)` over its angles).
pub fn mitred(perimeter: f64, cots: f64, r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r * perimeter - (5.0 / 3.0 - PI / 2.0) * r * r * r * cots
}

/// The stack's ring a ball of radius `r` fills where a cylinder of radius `big` stands on a
/// plane (Pappus).
pub fn root_ring(big: f64, r: f64) -> f64 {
    let moment = r * r * (big + r / 2.0) - PI * r * r / 4.0 * (big + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * moment
}

/// A round section's area beside a square corner, `(1 − π/4)r²`: a straight run's section, the
/// one `root_ring` turns.
pub fn square_run(r: f64) -> f64 { (1.0 - PI / 4.0) * r * r }

/// The section beside a cylinder of radius `big` whose axis lies in a plane, where the ball of
/// radius `r` touches both: the triangle of the axis, the plane's touch and the ball's centre, less
/// the ball's sector and the cylinder's.
pub fn beside_cylinder(big: f64, r: f64) -> f64 {
    let a = ((big + r) * (big + r) - r * r).sqrt();
    a * r / 2.0 - r * r * (r / (big + r)).acos() / 2.0 - big * big * r.atan2(a) / 2.0
}

/// The section of the fill between a line `w` off a circle's centre, the material on the centre's
/// side, and the circle of radius `big`, filled by a ball of radius `r` outside both: the
/// quadrilateral of the ball's centre, its touch on the line, the corner and the circle's centre,
/// less the circle's sector from the corner to the ball's touch and the ball's own sector.  At
/// `w = 0` it is `beside_cylinder`'s (#763).
pub fn line_beside_circle(w: f64, big: f64, r: f64) -> f64 {
    let s = ((big + r) * (big + r) - (w + r) * (w + r)).sqrt();
    let k = (big * big - w * w).sqrt();
    // about the circle's centre: the ball's centre, its touch on the line, the corner
    let (c, t, q) = ([-s, w + r], [-s, w], [-k, w]);
    let det = |u: [f64; 2], v: [f64; 2]| u[0] * v[1] - u[1] * v[0];
    let quad = 0.5 * (det(c, t) + det(t, q)).abs();
    let angle = |u: [f64; 2], v: [f64; 2]| det(u, v).atan2(u[0] * v[0] + u[1] * v[1]).abs();
    quad - 0.5 * big * big * angle(q, c) - 0.5 * r * r * angle([0.0, -r], [-c[0], -c[1]])
}

/// Solid `name` of `e` built, written as STEP and as STL by this kernel (each read back and
/// checked as `solventc` checks it), and its B-rep.
pub fn exported(e: &gcs_core::program::Elaborated, name: &str) -> gcs_core::brep::topo::Brep {
    use gcs_core::brep::{export, sweep::Say};
    let body = super::ent(e, name).i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, body, None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}"));
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    exact.solid
}

/// How many of a B-rep's faces lie on each kind of surface.
pub fn kinds(b: &gcs_core::brep::topo::Brep) -> std::collections::BTreeMap<&'static str, usize> {
    let mut out = std::collections::BTreeMap::new();
    for f in &b.faces {
        *out.entry(f.surface.kind()).or_default() += 1;
    }
    out
}

/// The least positive root of `a t² + b t + c` (infinite where there is none).
fn first_root(a: f64, b: f64, c: f64) -> f64 {
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return f64::INFINITY;
    }
    [(-b - disc.sqrt()) / (2.0 * a), (-b + disc.sqrt()) / (2.0 * a)].into_iter()
        .filter(|&t| t > 1e-12).fold(f64::INFINITY, f64::min)
}

/// Where a ray from `o` along `w` first meets the cylinder of `radius` about the coordinate axis
/// `axis` through `at`.
fn ray_cylinder(o: V, w: V, at: V, axis: usize, radius: f64) -> f64 {
    let o = sub(o, at);
    let (i, j) = ((axis + 1) % 3, (axis + 2) % 3);
    let (a, b) = (w[i] * w[i] + w[j] * w[j], 2.0 * (o[i] * w[i] + o[j] * w[j]));
    first_root(a, b, o[i] * o[i] + o[j] * o[j] - radius * radius)
}

/// Where a ray from `o` along `w` first meets the cone `ρ = r0 − slope·z` about `z` (its nappe
/// with `ρ ≥ 0`).
fn ray_cone(o: V, w: V, r0: f64, slope: f64) -> f64 {
    let k = r0 - slope * o[2];
    let a = w[0] * w[0] + w[1] * w[1] - slope * slope * w[2] * w[2];
    let b = 2.0 * (o[0] * w[0] + o[1] * w[1]) + 2.0 * slope * w[2] * k;
    let c = o[0] * o[0] + o[1] * o[1] - k * k;
    let disc = (b * b - 4.0 * a * c).max(0.0).sqrt();
    [(-b - disc) / (2.0 * a), (-b + disc) / (2.0 * a)].into_iter()
        .filter(|&t| t > 0.0 && k - slope * w[2] * t >= 0.0).fold(f64::INFINITY, f64::min)
}

/// The stack's `rolled` (#85): a ball of radius `r` rolled round a closed spine `c(φ)`, by
/// quadrature over its sections square to the spine, swept with the spine's curvature.
fn rolled(c: &dyn Fn(f64) -> V, toward: [&dyn Fn(V) -> V; 2], hit: [&dyn Fn(V, V) -> f64; 2],
          r: f64) -> f64 {
    rolled_within(c, toward, hit, r, None)
}

/// `rolled`, kept within the half-space `n · x ≤ k` (`within`): the material a run carried on
/// past its edge's end leaves, cut off by the face it ends on (#110).  Each section's reach along
/// a direction is cut where the plane crosses it, exactly, so only the quadrature's own error is
/// left; the sections stand at the middles of their steps, never on a plane a section lies in.
fn rolled_within(c: &dyn Fn(f64) -> V, toward: [&dyn Fn(V) -> V; 2],
                 hit: [&dyn Fn(V, V) -> f64; 2], r: f64, within: Option<(V, f64)>) -> f64 {
    rolled_over(c, [0.0, 2.0 * PI], toward, hit, r, within.as_slice(), &[])
}

/// `rolled_within` along the stretch `span` of an open spine `c(φ)`, kept within every half-space
/// `n · x ≤ k` of `within` and every upright cylinder (about `(x, y)`, of a radius) of `drums`: a
/// run carried on past both its ends and cut off by the faces there.
pub fn rolled_over(c: &dyn Fn(f64) -> V, span: [f64; 2], toward: [&dyn Fn(V) -> V; 2],
                 hit: [&dyn Fn(V, V) -> f64; 2], r: f64, within: &[(V, f64)], drums: &[([f64; 2], f64)]) -> f64 {
    let (n_phi, n_psi) = (720, 96);
    let gauss = gauss_legendre(n_psi);
    let h = 1e-4;
    let mut total = 0.0;
    let step = (span[1] - span[0]) / n_phi as f64;
    for k in 0..n_phi {
        let phi = span[0] + step * (k as f64 + 0.5);
        let p = c(phi);
        let (pa, pb) = (c(phi - h), c(phi + h));
        let d1 = scale(sub(pb, pa), 1.0 / (2.0 * h));
        let d2 = scale(sub(add(pb, pa), scale(p, 2.0)), 1.0 / (h * h));
        let speed = dot(d1, d1).sqrt();
        let t = scale(d1, 1.0 / speed);
        let kn = scale(sub(d2, scale(t, dot(d2, t))), 1.0 / (speed * speed));
        let (ea, eb) = (toward[0](p), toward[1](p));
        let span = dot(ea, eb).clamp(-1.0, 1.0).acos();
        let side = unit(sub(eb, scale(ea, dot(ea, eb))));
        let dir = |psi: f64| [0, 1, 2].map(|i| ea[i] * psi.cos() + side[i] * psi.sin());
        let switch = {
            let gap = |psi: f64| hit[0](p, dir(psi)) - hit[1](p, dir(psi));
            let (mut lo, mut hi) = (0.0, span);
            for _ in 0..80 {
                let m = 0.5 * (lo + hi);
                if (gap(m) < 0.0) == (gap(lo) < 0.0) { lo = m } else { hi = m }
            }
            0.5 * (lo + hi)
        };
        let mut area = 0.0;
        for (a, b) in [(0.0, switch), (switch, span)] {
            for &(x, w) in &gauss {
                let psi = a + 0.5 * (b - a) * (x + 1.0);
                let d = dir(psi);
                let (mut lo, mut hi) = (r, hit[0](p, d).min(hit[1](p, d)));
                for &(n, k) in within {
                    let (along, room) = (dot(n, d), k - dot(n, p));
                    if along.abs() < 1e-15 {
                        if room < 0.0 {
                            hi = lo;
                        }
                    } else if along > 0.0 {
                        hi = hi.min(room / along);
                    } else {
                        lo = lo.max(room / along);
                    }
                }
                for &([cx, cy], radius) in drums {
                    let (ox, oy) = (p[0] - cx, p[1] - cy);
                    let (a, b, c) = (d[0] * d[0] + d[1] * d[1], 2.0 * (ox * d[0] + oy * d[1]), ox * ox + oy * oy - radius * radius);
                    let disc = b * b - 4.0 * a * c;
                    if a < 1e-300 || disc < 0.0 {
                        if c > 0.0 { hi = lo; }
                        continue;
                    }
                    lo = lo.max((-b - disc.sqrt()) / (2.0 * a));
                    hi = hi.min((-b + disc.sqrt()) / (2.0 * a));
                }
                if hi <= lo {
                    continue;
                }
                let q = dot(d, kn);
                let swept = |rho: f64| rho * rho / 2.0 - q * rho.powi(3) / 3.0;
                area += w * 0.5 * (b - a) * (swept(hi) - swept(lo));
            }
        }
        total += area * speed * step;
    }
    total
}

/// The stack's tee crotch: the main pipe of radius `big` about x, a branch of radius `small`
/// about z standing on it, the ball of radius `r` rolled round where they meet.
pub fn tee_crotch(big: f64, small: f64, r: f64) -> f64 {
    let (rb, rs) = (big + r, small + r);
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            [rs * co, rs * s, (rb * rb - rs * rs * s * s).sqrt()]
        },
        [&|p: V| unit([0.0, -p[1], -p[2]]), &|p: V| unit([-p[0], -p[1], 0.0])],
        [&|o, w| ray_cylinder(o, w, [0.0; 3], 0, big),
            &|o, w| ray_cylinder(o, w, [0.0; 3], 2, small)],
        r,
    )
}

/// The stack's bore rim: a bore of radius `small` about z drilled into a solid pipe of radius
/// `big` about x, its rim rolled off.
pub fn bore_rim(big: f64, small: f64, r: f64) -> f64 {
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            let (rb, rs) = (big - r, small + r);
            [rs * co, rs * s, (rb * rb - rs * rs * s * s).sqrt()]
        },
        [&|p: V| unit([0.0, p[1], p[2]]), &|p: V| unit([-p[0], -p[1], 0.0])],
        [&|o, w| ray_cylinder(o, w, [0.0; 3], 0, big),
            &|o, w| ray_cylinder(o, w, [0.0; 3], 2, small)],
        r,
    )
}

/// The stack's root of a boss of radius `big` about `(cx, cy)` standing through the plane
/// `z = h0 − s x`, the whole way round.
pub fn sloped_root(big: f64, at: [f64; 2], slope: [f64; 2], r: f64) -> f64 {
    sloped(big, at, slope, r, ROOT, None)
}

/// Which round `sloped` rolls: a boss's root, the ball outside the material (concave), or a
/// hole's rim, the ball within it (convex).
pub const ROOT: f64 = 1.0;
pub const RIM: f64 = -1.0;

/// The round where a cylinder of radius `radius` about `(cx, cy)`, square to `z`, meets the
/// plane `z = h0 − s x`, the material below it — a boss's root (`ROOT`) or a hole's rim (`RIM`)
/// — kept within `n · x ≤ k`: carried on past the face it ends on and cut off there
/// (`x0/runout.sv`'s boss and vent).
pub fn sloped(radius: f64, at: [f64; 2], slope: [f64; 2], r: f64, side: f64,
                     within: Option<(V, f64)>) -> f64 {
    sloped_clipped(radius, at, slope, r, side, within.as_slice(), &[])
}

/// `sloped` kept within every half-space of `within` and every upright cylinder of `drums`: a run
/// that ends at a corner of two faces, or runs out onto a curved face (#218's 7).
pub fn sloped_clipped(radius: f64, [cx, cy]: [f64; 2], [h0, s]: [f64; 2], r: f64, side: f64,
                     within: &[(V, f64)], drums: &[([f64; 2], f64)]) -> f64 {
    let n = unit([s, 0.0, 1.0]);
    let lift = side * r / n[2];
    rolled_over(
        &|phi: f64| {
            let (sn, co) = phi.sin_cos();
            let x = cx + (radius + r) * co;
            [x, cy + (radius + r) * sn, h0 - s * x + lift]
        },
        [0.0, 2.0 * PI],
        [&|_: V| scale(n, -side), &|p: V| unit([cx - p[0], cy - p[1], 0.0])],
        [&|o: V, w: V| {
            let d = dot(w, n);
            if side * d < 0.0 { -(dot(o, n) - h0 * n[2]) / d } else { f64::INFINITY }
        },
            &|o, w| ray_cylinder(o, w, [cx, cy, 0.0], 2, radius)],
        r,
        within,
        drums,
    )
}

/// The round along a frustum's generator where the plane `y = 0` through its axis (z) cuts it, the
/// material `y > 0` within the frustum: radius `r0 − slope · z`, the run cut off by the planes `z =
/// z0` and `z = z1` it ends at, the ball rolled with its centre `r` off the plane and on the cone
/// grown in by `r` (a hyperbola), carried on past both.
pub fn frustum_generator(r0: f64, slope: f64, [z0, z1]: [f64; 2], r: f64) -> f64 {
    let (cs, sn) = (1.0 / (1.0 + slope * slope).sqrt(), slope / (1.0 + slope * slope).sqrt());
    let reach = 3.0 * r;
    rolled_over(
        &|z: f64| {
            let rho = r0 - slope * z - r / cs;
            [(rho * rho - r * r).sqrt(), r, z]
        },
        [z0 - reach, z1 + reach],
        [&|_: V| [0.0, -1.0, 0.0], &|p: V| {
            let rho = p[0].hypot(p[1]);
            [cs * p[0] / rho, cs * p[1] / rho, sn]
        }],
        [&|o: V, w: V| if w[1] < 0.0 { -o[1] / w[1] } else { f64::INFINITY },
            &|o, w| ray_cone(o, w, r0, slope)],
        r,
        &[([0.0, 0.0, -1.0], -z0), ([0.0, 0.0, 1.0], z1)],
        &[],
    )
}

/// A chamfer's flat cut along a closed edge `c(φ)` (`φ` over a whole turn) where two faces meet:
/// the region within both — `d[k]`, each face's signed distance, positive toward the region, with
/// its gradient — short of the flat `d₀ / legs[1] + d₁ / legs[0] ≤ 1`, by quadrature over its
/// sections square to the edge (#218's 6). Each section is bounded by face 0 from the edge to its
/// leg (`d₁ = legs[0]`), the flat, and face 1 back from its leg (`d₀ = legs[1]`), each point pulled
/// onto its curve and the section's plane by Newton's steps; its area, swept with the edge's
/// curvature, `∬ (1 − κ x) dA`, is `∮ (x − κ x² / 2) dy` by Green's theorem (`x` along the
/// principal normal), each curve by Gauss–Legendre.
pub fn flat_along(c: &dyn Fn(f64) -> V, d: [&dyn Fn(V) -> f64; 2], grad: [&dyn Fn(V) -> V; 2], legs: [f64; 2])
    -> f64 {
    flat_within(c, d, grad, legs, &[])
}

/// `flat_along` kept within every half-space `n · x ≤ k` of `within`: a run carried on past the
/// faces it ends on and cut off there. A section a plane cuts is read as a polygon of its three
/// curves, densely, clipped by the half-spaces, `∮ (x − κ x² / 2) dy` exact along each side.
pub fn flat_within(c: &dyn Fn(f64) -> V, d: [&dyn Fn(V) -> f64; 2], grad: [&dyn Fn(V) -> V; 2], legs: [f64; 2],
    within: &[(V, f64)]) -> f64 {
    let (n_phi, n_t) = (720, 48);
    let gauss = gauss_legendre(n_t);
    // `x` solving `rows · x = rhs` (three rows)
    let solve = |m: [V; 3], rhs: V| -> V { solve3(m, rhs).expect("a section's planes meet in a point") };
    // the point near `q` where `f` (with its gradient `g`) is nothing, on the plane through `e`
    // square to `t`, and on the plane through `at` square to `along`
    let pull = |mut q: V, f: &dyn Fn(V) -> f64, g: &dyn Fn(V) -> V, e: V, t: V, at: V, along: V| -> V {
        for _ in 0..60 {
            let r = [f(q), dot(sub(q, e), t), dot(sub(q, at), along)];
            let step = solve([g(q), t, along], r);
            q = sub(q, step);
            if norm(step) < 1e-15 * (1.0 + norm(q)) { break; }
        }
        q
    };
    let flat = |q: V| d[0](q) / legs[1] + d[1](q) / legs[0] - 1.0;
    let flat_grad = |q: V| add(scale(grad[0](q), 1.0 / legs[1]), scale(grad[1](q), 1.0 / legs[0]));
    let h = 1e-4;
    let mut total = 0.0;
    for k in 0..n_phi {
        let phi = 2.0 * PI * (k as f64 + 0.5) / n_phi as f64;
        let e = c(phi);
        let (pa, pb) = (c(phi - h), c(phi + h));
        let d1 = scale(sub(pb, pa), 1.0 / (2.0 * h));
        let d2 = scale(sub(add(pb, pa), scale(e, 2.0)), 1.0 / (h * h));
        let speed = norm(d1);
        let t = scale(d1, 1.0 / speed);
        let kn = scale(sub(d2, scale(t, dot(d2, t))), 1.0 / (speed * speed));
        let nrm = if norm(kn) > 1e-12 { unit(kn) } else { unit(cross(t, if t[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] })) };
        let bin = cross(t, nrm);
        // each leg: on face `k`, `legs[k]` from the other
        let leg = |k: usize| -> V {
            let (fk, fo) = (d[k], d[1 - k]);
            let g = grad[1 - k](e);
            let n = unit(grad[k](e));
            let w = unit(sub(g, scale(n, dot(g, n))));
            let mut q = add(e, scale(w, legs[k]));
            for _ in 0..60 {
                let r = [fk(q), fo(q) - legs[k], dot(sub(q, e), t)];
                let step = solve([grad[k](q), grad[1 - k](q), t], r);
                q = sub(q, step);
                if norm(step) < 1e-15 * (1.0 + norm(q)) { break; }
            }
            q
        };
        let (l0, l1) = (leg(0), leg(1));
        // `∮ P dy` along the curve from `a` to `b` where `f` is nothing
        let along = |a: V, b: V, f: &dyn Fn(V) -> f64, g: &dyn Fn(V) -> V| -> f64 {
            let chord = unit(sub(b, a));
            let at = |s: f64| pull(add(a, scale(sub(b, a), s)), f, g, e, t, add(a, scale(sub(b, a), s)), chord);
            let p = |q: V| { let x = dot(sub(q, e), nrm); x - dot(kn, sub(q, e)) * x / 2.0 };
            gauss.iter().map(|&(x, w)| {
                let s = 0.5 * (x + 1.0);
                let dy = (dot(at(s + 1e-6), bin) - dot(at(s - 1e-6), bin)) / 2e-6;
                0.5 * w * p(at(s)) * dy
            }).sum()
        };
        // the section's outline, densely, where a plane may cut it
        let outline = || -> Vec<V> {
            let n = 2000;
            let mut pts = Vec::new();
            for (a, b, f, g) in [(e, l0, d[0], grad[0]), (l0, l1, &flat as &dyn Fn(V) -> f64, &flat_grad as &dyn Fn(V) -> V),
                (l1, e, d[1], grad[1])] {
                let chord = unit(sub(b, a));
                for j in 0..n {
                    let at = add(a, scale(sub(b, a), j as f64 / n as f64));
                    pts.push(if j == 0 { a } else { pull(at, f, g, e, t, at, chord) });
                }
            }
            pts
        };
        let section = if within.is_empty() {
            along(e, l0, d[0], grad[0]) + along(l0, l1, &flat, &flat_grad) + along(l1, e, d[1], grad[1])
        } else {
            let mut poly = outline();
            if poly.iter().all(|&q| within.iter().all(|&(n, k)| dot(n, q) <= k)) {
                along(e, l0, d[0], grad[0]) + along(l0, l1, &flat, &flat_grad) + along(l1, e, d[1], grad[1])
            } else {
                for &(n, k) in within {
                    let mut kept = Vec::new();
                    for i in 0..poly.len() {
                        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                        let (fa, fb) = (dot(n, a) - k, dot(n, b) - k);
                        if fa <= 0.0 { kept.push(a); }
                        if (fa < 0.0) != (fb < 0.0) && fa != fb { kept.push(add(a, scale(sub(b, a), fa / (fa - fb)))); }
                    }
                    poly = kept;
                }
                // `∮ P dy` along each side, `P` quadratic along it: Simpson's rule, exact
                let p = |q: V| { let x = dot(sub(q, e), nrm); x - dot(kn, sub(q, e)) * x / 2.0 };
                (0..poly.len()).map(|i| {
                    let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                    let dy = dot(sub(b, a), bin);
                    dy * (p(a) + 4.0 * p(scale(add(a, b), 0.5)) + p(b)) / 6.0
                }).sum()
            }
        };
        total += section.abs() * speed * 2.0 * PI / n_phi as f64;
    }
    total
}

/// A bore of radius `small` about the upright line through `(dx, 0)` drilled down a cone about z
/// (radius `r0 − slope·z`), its axis inside the bore, the rim where they meet rounded by a ball of
/// radius `r` within the material — the loop winding round both (#218's 7).
pub fn cone_bore_rim(r0: f64, slope: f64, small: f64, dx: f64, r: f64) -> f64 {
    let (cs, sn) = (1.0 / (1.0 + slope * slope).sqrt(), slope / (1.0 + slope * slope).sqrt());
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            let (x, y) = (dx + (small + r) * co, (small + r) * s);
            [x, y, (r0 - r / cs - x.hypot(y)) / slope]
        },
        [&|p: V| unit([dx - p[0], -p[1], 0.0]), &|p: V| {
            let rho = p[0].hypot(p[1]);
            [cs * p[0] / rho, cs * p[1] / rho, sn]
        }],
        [&|o, w| ray_cylinder(o, w, [dx, 0.0, 0.0], 2, small),
            &|o, w| ray_cone(o, w, r0, slope)],
        r,
    )
}

/// A boss of radius `big` about `z` standing on a top that runs from a flat land (`z = nose`, `y ≤
/// 0`) on smoothly into a bullnose (the cylinder of radius `nose` about `x`, `y ≥ 0`), its root
/// filled by a ball of radius `r` rolled the whole way round across the seam where they meet.
pub fn bullnose_root(big: f64, nose: f64, r: f64) -> f64 {
    let (rs, rn) = (big + r, nose + r);
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            let y = rs * s;
            [rs * co, y, if y <= 0.0 { rn } else { (rn * rn - y * y).sqrt() }]
        },
        [&|p: V| if p[1] <= 0.0 { [0.0, 0.0, -1.0] } else { unit([0.0, -p[1], -p[2]]) },
            &|p: V| unit([-p[0], -p[1], 0.0])],
        [&|o: V, w: V| {
            // the land where it is crossed at `y ≤ 0`, the bullnose where at `y ≥ 0`
            let land = if w[2] < 0.0 { (nose - o[2]) / w[2] } else { f64::INFINITY };
            let land = if land.is_finite() && o[1] + land * w[1] <= 0.0 { land } else { f64::INFINITY };
            let round = ray_cylinder(o, w, [0.0; 3], 0, nose);
            let round = if round.is_finite() && o[1] + round * w[1] >= 0.0 { round } else { f64::INFINITY };
            land.min(round)
        },
            &|o, w| ray_cylinder(o, w, [0.0; 3], 2, big)],
        r,
    )
}

/// A cylinder of radius `radius` about `(cx, cy)`, square to `z`, through the plane `z = h0 − s x`
/// (the material below it), the flat of legs `leg` where they meet — a boss's root (`ROOT`, outside
/// the cylinder) or a hole's rim (`RIM`, within the material, outside the hole) — kept within
/// `n · x ≤ k`: `sloped`'s chamfered.
pub fn sloped_flat(radius: f64, [cx, cy]: [f64; 2], [h0, s]: [f64; 2], leg: f64, side: f64,
                          within: &[(V, f64)]) -> f64 {
    let n = unit([s, 0.0, 1.0]);
    let rho = |p: V| (p[0] - cx).hypot(p[1] - cy);
    flat_within(
        &|phi: f64| { let (sn, co) = phi.sin_cos(); let x = cx + radius * co; [x, cy + radius * sn, h0 - s * x] },
        // the plane's distance, positive above it for a root (outside the material), below for a rim
        [&|p: V| side * (dot(p, n) - h0 * n[2]), &|p: V| rho(p) - radius],
        [&|_: V| scale(n, side), &|p: V| scale([p[0] - cx, p[1] - cy, 0.0], 1.0 / rho(p))],
        [leg, leg],
        within,
    )
}

/// Gauss–Legendre's nodes and weights on (−1, 1).
pub fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    (0..n).map(|i| {
        let mut x = (PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut dp = 0.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 { break; }
        }
        (x, 2.0 / ((1.0 - x * x) * dp * dp))
    }).collect()
}

/// The stack's tee crotch chamfered: the main pipe of radius `big` about x, a branch of radius
/// `small` about z standing on it, the flat of legs `leg` where they meet (outside both).
pub fn tee_flat(big: f64, small: f64, leg: f64) -> f64 {
    let rho = |p: V, i: usize, j: usize| p[i].hypot(p[j]);
    flat_along(
        &|phi: f64| { let (s, co) = phi.sin_cos(); [small * co, small * s, (big * big - small * small * s * s).sqrt()] },
        [&|p: V| rho(p, 1, 2) - big, &|p: V| rho(p, 0, 1) - small],
        [&|p: V| scale([0.0, p[1], p[2]], 1.0 / rho(p, 1, 2)), &|p: V| scale([p[0], p[1], 0.0], 1.0 / rho(p, 0, 1))],
        [leg, leg],
    )
}

/// The stack's bore rim chamfered: a bore of radius `small` about z drilled into a solid pipe of
/// radius `big` about x, the flat of legs `leg` along its rim (within the pipe, outside the bore).
pub fn bore_flat(big: f64, small: f64, leg: f64) -> f64 {
    let rho = |p: V, i: usize, j: usize| p[i].hypot(p[j]);
    flat_along(
        &|phi: f64| { let (s, co) = phi.sin_cos(); [small * co, small * s, (big * big - small * small * s * s).sqrt()] },
        [&|p: V| big - rho(p, 1, 2), &|p: V| rho(p, 0, 1) - small],
        [&|p: V| scale([0.0, p[1], p[2]], -1.0 / rho(p, 1, 2)), &|p: V| scale([p[0], p[1], 0.0], 1.0 / rho(p, 0, 1))],
        [leg, leg],
    )
}

/// The meridian's closed forms: a turned body's volume read off its `(ρ, z)` section.
pub mod meridian {
    use gcs_core::brep::planar::Seg;
    use gcs_core::integral::GAUSS8;
    use std::f64::consts::PI;

    use super::root_ring;

    pub type P = [f64; 2];

    pub fn sub(a: P, b: P) -> P { [a[0] - b[0], a[1] - b[1]] }
    pub fn add(a: P, b: P) -> P { [a[0] + b[0], a[1] + b[1]] }
    pub fn scale(a: P, k: f64) -> P { [a[0] * k, a[1] * k] }
    pub fn dot(a: P, b: P) -> f64 { a[0] * b[0] + a[1] * b[1] }
    pub fn cross(a: P, b: P) -> f64 { a[0] * b[1] - a[1] * b[0] }
    pub fn norm(a: P) -> f64 { a[0].hypot(a[1]) }
    pub fn dist(a: P, b: P) -> f64 { norm(sub(a, b)) }
    pub fn unit(a: P) -> P { scale(a, 1.0 / norm(a)) }
    pub fn left(a: P) -> P { [-a[1], a[0]] }

    /// `∮ ρ²/2 dz` along a step, `(ρ, z)` its coordinates, in closed form.
    pub fn flux(s: &Seg) -> f64 {
        match *s {
            Seg::Line { a: p, b: q } => {
                (q[1] - p[1]) / 2.0 * (p[0] * p[0] + p[0] * q[0] + q[0] * q[0]) / 3.0
            }
            Seg::Arc { c, r, a0, sweep } => {
                let prim = |phi: f64| {
                    let (sn, cs) = phi.sin_cos();
                    (c[0] * c[0] * r * sn + c[0] * r * r * (phi + sn * cs)
                        + r * r * r * (sn - sn.powi(3) / 3.0)) / 2.0
                };
                prim(a0 + sweep) - prim(a0)
            }
            // a rational quadratic: eight-point Gauss–Legendre over 16 stretches, to roundoff
            Seg::Conic { .. } => (0..16).map(|k| GAUSS8.iter().map(|&(x, w)| {
                let t = (k as f64 + 0.5 + 0.5 * x) / 16.0;
                let (p, d) = (s.point(t), s.tangent(t));
                w * p[0] * p[0] / 2.0 * d[1]
            }).sum::<f64>() / 32.0).sum(),
        }
    }

    /// The stack's closed form of #83's knob's neck: a rod of radius `rod` about the axis entering
    /// a ball of radius `big` about `s`, filled to `r`.  Its section is the corner, its foot on the
    /// rod, the arc of the ball rolled, its foot on the ball and the ball's arc back, turned.
    pub fn knob_neck(s: P, big: f64, rod: f64, r: f64) -> f64 {
        let corner = [rod, s[1] - (big * big - rod * rod).sqrt()];
        let centre = [rod + r, s[1] - ((big + r).powi(2) - (rod + r).powi(2)).sqrt()];
        let (on_rod, on_ball) = ([rod, centre[1]], add(s, scale(unit(sub(centre, s)), big)));
        let at = |c: P, p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
        let arc = |c: P, r: f64, from: P, to: P| {
            Seg::Arc { c, r, a0: at(c, from), sweep: at(c, to) - at(c, from) }
        };
        2.0 * PI * (flux(&Seg::Line { a: corner, b: on_rod })
            + flux(&arc(centre, r, on_rod, on_ball)) + flux(&arc(s, big, on_ball, corner))).abs()
    }

    /// #83's knob rounded, whole (`x0/knob.sv`, `rounds/knob.sv`): the disc (30 by 8), the rod up
    /// to the ball, the ball less the cap the rod holds, its root's ring (2) and its neck's
    /// (`knob_neck`).
    pub fn knob_rounded(s: P, big: f64, rod: f64, r: f64) -> f64 {
        let corner = s[1] - (big * big - rod * rod).sqrt();
        let cap = { let h = corner - (s[1] - big); PI * h * h * (3.0 * big - h) / 3.0 };
        PI * 30.0 * 30.0 * 8.0 + PI * rod * rod * (corner - 8.0) + 4.0 / 3.0 * PI * big.powi(3)
            - cap + root_ring(rod, 2.0) + knob_neck(s, big, rod, r)
    }
}

/// Solid `name` of `e` built exactly by this kernel (millimetres), refused by name otherwise.
pub fn exact(e: &gcs_core::program::Elaborated, name: &str) -> gcs_core::brep::topo::Brep {
    use gcs_core::brep::{export, sweep::Say};
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    export::exact(&e.sketch, super::ent(e, name).i(), None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}")).solid
}

/// The volume of `b` within the box `lo`–`hi`: `b` kept in common with the box, exactly.
pub fn volume_within(b: &gcs_core::brep::topo::Brep, lo: V, hi: V) -> f64 {
    use gcs_core::brep::boolean::{combined, Op};
    let boxed = combined(b, &crate::brep::block(lo, hi), Op::Common, 0.0)
        .unwrap_or_else(|why| panic!("kept in a box: {why:?}"));
    gcs_core::brep::props::volume(&boxed)
}

/// **The volume a round takes, exactly**: solid `name` built by the kernel before (`both[0]`) and
/// after (`both[1]`), each kept in the box `lo`–`hi`, before less after (negative where it adds) —
/// what `field_volume` read off the field to a part in five hundred, here to the kernel's own
/// precision.
pub fn taken_exactly(both: &[gcs_core::program::Elaborated; 2], name: &str, lo: V, hi: V) -> f64 {
    let [before, after] = both.each_ref().map(|e| volume_within(&exact(e, name), lo, hi));
    before - after
}

/// **The field's boundary is the kernel's, point by point** (#780): solid `name` of `e`, built
/// exactly and meshed within a tenth of `step`, read by the field a step inside and a step outside
/// the middle of each triangle in the box `lo`–`hi` (at most `most` of them, spread evenly), and
/// the field's side held to the kernel's own there — the exact build's signed distance off its
/// mesh, by the triangles' winding number (`swept::member::Solid`), a point within half a step of
/// that boundary (a step off a triangle beside a sharp edge can cross the other face) not read.
/// The kernel's volume of a round is its closed form (`taken_exactly`), so this is what holding the
/// field's volume to it held: that the field draws the kernel's boundary, with a few thousand of
/// the field's readings where its volume took millions.  How many points were read.
pub fn field_is_the_kernels(e: &gcs_core::program::Elaborated, name: &str, lo: V, hi: V,
                            step: f64, most: usize) -> usize {
    use gcs_core::brep::swept::member::Solid;
    use gcs_core::solid::MaterialField;
    assert_eq!(e.sketch.units.mm_per(), 1.0, "{name}: read in millimetres");
    let b = exact(e, name);
    let m = gcs_core::brep::mesh::mesh(&b, step / 10.0, 0.1).unwrap_or_else(|why| panic!("{name}: {why}"));
    let kernel = Solid::new(&b, step / 10.0, 1.0).unwrap_or_else(|why| panic!("{name}: {why}"));
    assert!(kernel.sag <= step / 4.0, "{name}: meshed within {} of a step {step}", kernel.sag);
    let field = MaterialField::read(&e.sketch, super::ent(e, name).i(), 1e-10).unwrap();
    let inside = |p: V| (0..3).all(|k| p[k] >= lo[k] && p[k] <= hi[k]);
    let points: Vec<V> = m.tris.iter().filter_map(|t| {
        let [a, b, c] = t.map(|i| m.pts[i as usize]);
        let middle = scale(add(add(a, b), c), 1.0 / 3.0);
        let n = cross(sub(b, a), sub(c, a));
        (inside(middle) && norm(n) > 0.0).then(|| {
            let n = unit(n);
            [sub(middle, scale(n, step)), add(middle, scale(n, step))]
        })
    }).flatten().collect();
    assert!(!points.is_empty(), "{name}: no triangle in the box");
    let every = points.len().div_ceil(2 * most).max(1);
    let chosen: Vec<V> = points.into_iter().step_by(every).collect();
    // each point's side by the kernel (none where it stands within half a step of its boundary)
    // and by the field
    let read: Vec<(V, bool, f64)> = gcs_core::par::map(&chosen, |&p| {
        let d = kernel.signed_distance(p);
        (d.abs() >= step / 2.0).then(|| (p, d < 0.0, field.side(p)))
    }).into_iter().flatten().collect();
    assert!(read.len() * 2 >= chosen.len(), "{name}: {} of {} points read", read.len(), chosen.len());
    let wrong: Vec<String> = read.iter().filter(|(_, ins, f)| f.is_nan() || if *ins { *f > 0.0 } else { *f < 0.0 })
        .map(|(p, ins, f)| format!("at {p:?}: the kernel's {}, the field reads {f:e}",
                                   if *ins { "inside" } else { "outside" })).collect();
    assert!(wrong.is_empty(), "{name}: the field is not the kernel's {} times of {}:\n{}", wrong.len(),
            read.len(), wrong.iter().take(8).cloned().collect::<Vec<_>>().join("\n"));
    // a reading of exactly nothing is on the field's boundary, not across it: where two operands'
    // faces meet in one plane inside the material (the pad's plate and its boss's cap) the field
    // reads zero over a set of no volume.  A few are let be; a wrong side never is.
    let on = read.iter().filter(|(_, _, f)| *f == 0.0).count();
    assert!(on * 20 <= read.len(), "{name}: the field reads its boundary {on} times of {}", read.len());
    read.len()
}
