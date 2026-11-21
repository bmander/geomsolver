//! **Rounds against a sphere** (#962's 7): vanes and bosses filled into a disc whose crown is a
//! sphere, and into a pocket whose floor is one. A plane meets a sphere in a circle about the line
//! through the sphere's centre square to the plane, and the ball rolled between them has its
//! centre on a circle about that line too: the fill is a ring, a torus's stretch, kept within the
//! planes of the vane's ends (the lowering's `Ring`). Where two such stretches meet at a vane's
//! corner, each one's cap lies in the plane the other's torus touches round a circle, and the
//! Boolean reads the cap's edge against that torus off the plane's own meeting with it. A side
//! about another axis than the sphere's centre — a cylinder off it — is rolled along
//! (`fillet::roll`), and the band its ball holds on the crown is the strip it sweeps, not every
//! point of the crown as near the side's whole cylinder.
//!
//! Each is held to its closed form: the part's volume to 1e-9, and the fills' own — a sliver of
//! it — to 1e-7 where each stretch is cut by a plane parallel to its axis, whose meeting with the
//! torus (a spiric section) the kernel fits; a rolled fill, its canal fitted, to the quadrature of
//! its sections (#85) at 1e-6.
use gcs_core::brep::props;
use std::f64::consts::PI;

use crate::common::close;
use crate::common::rounds::{crown_ball, crown_section, exported, kinds, rolled_over, square_run, under_circle};
use crate::common::rounds::{dot, first_root, quad, quad_by, ray_cylinder, scale, sub, unit, V};
use crate::common::text::{dof, solved};

/// A disc of radius 100 whose crown is a sphere of radius `big` about (0, 0, `zc`) — over the
/// disc's rim at height `rim`, convex where `zc` is below it, dished where above — and a vane
/// standing on it from z = 1 to `top`: in the top plane the rectangle `[e0, e1] × [s0, s1]` along
/// `m = (cos α, sin α)` and `n = (−sin α, cos α)`, unioned, its foot filled by `r`.
#[derive(Clone, Copy, Debug)]
struct Vane {
    big: f64,
    zc: f64,
    rim: f64,
    e: [f64; 2],
    s: [f64; 2],
    alpha: f64,
    top: f64,
    r: f64,
}

/// The issue's crown: through (100, 3) about (0, −797).
fn issue_big() -> f64 { 650_000f64.sqrt() }

impl Vane {
    /// The synthetic case of #962's 7: the issue's crown, a vane 4 thick from x = 20 to 90 on the
    /// x axis, 30 tall from z = 1.
    fn issue(r: f64) -> Vane {
        Vane { big: issue_big(), zc: -797.0, rim: 3.0, e: [20.0, 90.0], s: [-2.0, 2.0], alpha: 0.0, top: 31.0, r }
    }

    fn convex(&self) -> bool { self.zc < self.rim }

    fn document(&self) -> String {
        let (sa, ca) = self.alpha.sin_cos();
        let at = |u: f64, v: f64| (u * ca - v * sa, u * sa + v * ca);
        let corners = [at(self.e[0], self.s[0]), at(self.e[1], self.s[0]), at(self.e[1], self.s[1]), at(self.e[0], self.s[1])];
        let fix: String = ["a", "b", "d", "e"].iter().zip(corners)
            .map(|(p, (x, y))| format!("  {p} := point\n  fix(({x:.12}, {y:.12})) {p}\n")).collect();
        format!("{}in std.top {{\n{fix}  vf := face(a, b, d, e, -> close)\n}}\n\
            vane := solid(std.solids.Prism(vf, from: 1mm, to: {}mm))\npart := solid(disc)\nvane union part\n\
            part.vane round(r: {}mm) part.disc\n", disc(self.big, self.zc, self.rim), self.top, self.r)
    }

    /// The disc, and the vane over its crown: across the vane in closed form, along it by
    /// quadrature.
    fn stock(&self) -> f64 {
        let (big, zc) = (self.big, self.zc);
        let sigma = if self.convex() { 1.0 } else { -1.0 };
        let across = |u: f64| {
            let a2 = big * big - u * u;
            let [s0, s1] = self.s;
            (s1 - s0) * (self.top - zc) - sigma * (under_circle(a2, s1) - under_circle(a2, s0))
        };
        disc_volume(big, zc, self.convex()) + quad(self.e, &across)
    }

    /// The four fills: each side's, turned about the line from the sphere's centre square to it
    /// through what lies between the planes of the two sides beside it (`crown_ball`).
    fn fills(&self) -> f64 {
        let ([s0, s1], [e0, e1]) = (self.s, self.e);
        let ring = |c: f64, [a0, a1]: [f64; 2]| crown_ball(self.big, self.convex(), c, self.r, &|_| (0.0, a0), &|_| (0.0, a1));
        ring(-s1, self.e) + ring(s0, self.e) + ring(-e1, self.s) + ring(e0, self.s)
    }
}

/// The disc's section and its solid: a crown of radius `big` about (0, `zc`) over a rim at `rim`.
fn disc(big: f64, zc: f64, rim: f64) -> String {
    let arc = if zc < rim { "arc(center: c, start: t, end: top)" } else { "arc(center: c, start: top, end: t)" };
    let top = if zc < rim { zc + big } else { zc - big };
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union, cut)\nuse std.rounds (round, chamfer)\n\
        in std.front {{\n  o := point\n  fix((0, 0)) o\n  r := point\n  fix((100, 0)) r\n  t := point\n  fix((100, {rim})) t\n\
        c := point\n  fix((0, {zc})) c\n  top := point hint((0, {top}))\n  fix(x == 0mm) top\n\
        bottom := line(o, r)\n  edge := line(r, t)\n  crown := {arc}\n\
        spine := line(top, o)\n  section := face(bottom, edge, crown, spine)\n}}\n\
        disc := solid(std.solids.Revolution(section, about: std.z))\n")
}

/// The issue's disc and `rest` (whose `part := solid(disc)`, or a solid of it, takes the rest).
fn crowned(rest: &str) -> String { format!("{}{rest}", disc(issue_big(), -797.0, 3.0)) }

/// The disc turned: `∫ 2πx crown(x) dx` over its radius of 100.
fn disc_volume(big: f64, zc: f64, convex: bool) -> f64 {
    let sigma = if convex { 1.0 } else { -1.0 };
    2.0 * PI * (zc * 5000.0 + sigma * (big.powi(3) - (big * big - 1e4).powf(1.5)) / 3.0)
}

/// The issue's crown's height at `rho` from its axis.
fn crown(rho: f64) -> f64 { -797.0 + (650_000.0 - rho * rho).sqrt() }

/// `∫∫ f` over the disc of radius `radius` about `(x, y)`, in polar coordinates about it.
fn over_disc([x, y]: [f64; 2], radius: f64, f: &dyn Fn(V) -> f64) -> f64 {
    quad([0.0, 2.0 * PI], &|phi| quad([0.0, radius], &|rho| rho * f([x + rho * phi.cos(), y + rho * phi.sin(), 0.0])))
}

/// Where a ray from `o` along `w` first meets the sphere of `radius` about `centre` (infinitely far
/// where it misses it or meets it behind).
fn ray_sphere(o: V, w: V, centre: V, radius: f64) -> f64 {
    let d = sub(o, centre);
    first_root(dot(w, w), 2.0 * dot(d, w), dot(d, d) - radius * radius)
}

/// The ball of radius `r` rolled round where a cylinder of `radius` about `(x, y)` meets a sphere
/// of radius `big` about `centre`: from outside the cylinder (`side` +1) or within it (−1), outside
/// the sphere or within it (`outside`), the whole way round (the stack's quadrature, #85).
fn rolled_round(at: [f64; 2], radius: f64, side: f64, centre: V, big: f64, outside: bool, r: f64) -> f64 {
    let sigma = if outside { 1.0 } else { -1.0 };
    let (rb, ro) = (big + sigma * r, radius + side * r);
    // the sphere's cap the crown or the floor is: above its centre where the ball is outside it
    let spine = |phi: f64| {
        let (x, y) = (at[0] + ro * phi.cos(), at[1] + ro * phi.sin());
        let h = (rb * rb - (x - centre[0]).powi(2) - (y - centre[1]).powi(2)).sqrt();
        [x, y, centre[2] + sigma * h]
    };
    let to_sphere = |p: V| unit(scale(sub(centre, p), sigma));
    let to_drum = |p: V| unit([side * (at[0] - p[0]), side * (at[1] - p[1]), 0.0]);
    let on_sphere = |o: V, w: V| ray_sphere(o, w, centre, big);
    let on_drum = |o: V, w: V| ray_cylinder(o, w, [at[0], at[1], 0.0], 2, radius);
    rolled_over(&spine, [0.0, 2.0 * PI], [&to_sphere, &to_drum], [&on_sphere, &on_drum], r, &[], &[])
}

/// The ball of radius `r` rolled where an upright cylinder of `radius` about `(x, y)` meets a
/// crown — a sphere of radius `big` about `centre`, its cap above it — from outside the cylinder
/// (`side` +1) or within it, between the half-planes about the cylinder's axis at the angles
/// `wedge`: what each of those half-planes cuts of the fill, its moment about the axis by Green's
/// theorem round its boundary — the cylinder's line, the canal's trace and the crown's — summed
/// over the wedge (Pappus slice by slice). The canal's trace in a half-plane is its arcs' points
/// there, each found along the run by Newton's steps; every piece smooth, so Gauss–Legendre's
/// rule reads it exactly, where sections square to the run straddle a slanted end (`rolled_over`).
fn rolled_about(at: [f64; 2], radius: f64, side: f64, centre: V, big: f64, r: f64, wedge: [f64; 2]) -> f64 {
    let ro = radius + side * r;
    let crown = |x: f64, y: f64| centre[2] + (big * big - (x - centre[0]).powi(2) - (y - centre[1]).powi(2)).sqrt();
    let spine = |psi: f64| {
        let (x, y) = (at[0] + ro * psi.cos(), at[1] + ro * psi.sin());
        let h = ((big + r).powi(2) - (x - centre[0]).powi(2) - (y - centre[1]).powi(2)).sqrt();
        [x, y, centre[2] + h]
    };
    // the canal's point at `psi` along the run, `t` of the way round its arc from the crown's
    // contact to the cylinder's
    let canal = |psi: f64, t: f64| -> V {
        let p = spine(psi);
        let a = unit(sub(centre, p));
        let b = unit([side * (at[0] - p[0]), side * (at[1] - p[1]), 0.0]);
        let w = dot(a, b).clamp(-1.0, 1.0).acos();
        let (ka, kb) = (((1.0 - t) * w).sin() / w.sin(), (t * w).sin() / w.sin());
        std::array::from_fn(|i| p[i] + r * (ka * a[i] + kb * b[i]))
    };
    let azimuth = |q: V| (q[1] - at[1]).atan2(q[0] - at[0]);
    let polar = |q: V| ((q[0] - at[0]).hypot(q[1] - at[1]), q[2]);
    // the canal's point in the half-plane at `phi`
    let trace = |phi: f64, t: f64| {
        let mut psi = phi;
        for _ in 0..40 {
            let g = azimuth(canal(psi, t)) - phi;
            let dg = (azimuth(canal(psi + 1e-7, t)) - azimuth(canal(psi - 1e-7, t))) / 2e-7;
            let step = g / dg;
            psi -= step;
            if step.abs() < 1e-15 { break }
        }
        polar(canal(psi, t))
    };
    let line = |f: &dyn Fn(f64) -> (f64, f64)| quad_by(48, [0.0, 1.0], &|t| {
        let (rho, _) = f(t);
        let dz = (f(t + 1e-6).1 - f(t - 1e-6).1) / 2e-6;
        0.5 * rho * rho * dz
    });
    let slice = |phi: f64| {
        let (s, c) = phi.sin_cos();
        let on_crown = |rho: f64| crown(at[0] + rho * c, at[1] + rho * s);
        let (wall, touch) = (trace(phi, 1.0), trace(phi, 0.0));
        let corner = on_crown(radius);
        let up = |t: f64| (radius, corner + (wall.1 - corner) * t);
        let round = |t: f64| trace(phi, 1.0 - t);
        let down = |t: f64| { let rho = touch.0 + (radius - touch.0) * t; (rho, on_crown(rho)) };
        (line(&up) + line(&round) + line(&down)).abs()
    };
    quad(wedge, &slice)
}

/// The part built and exported, its volume the closed form's — `stock` and the fills' `fills` —
/// to 1e-9, and the fills' own to `bar`.
fn holds(src: &str, stock: f64, fills: f64, bar: f64) -> gcs_core::brep::topo::Brep {
    let b = exported(&solved(src), "part");
    let got = props::volume(&b);
    close(got, stock + fills);
    assert!((got - stock - fills).abs() <= bar * fills, "the fills: {} against {fills} ({:e})", got - stock,
        (got - stock - fills) / fills);
    b
}

/// The vane of `arc_vane`: in the top plane the annulus between `radii` about `centre` over the
/// angles `[from, to]` (degrees), standing from z = 1 to 31.
fn arc_vane(centre: [f64; 2], radii: [f64; 2], [from, to]: [f64; 2], word: &str) -> String {
    let at = |rad: f64, deg: f64| { let (s, c) = deg.to_radians().sin_cos(); (centre[0] + rad * c, centre[1] + rad * s) };
    let p = |name: &str, (x, y): (f64, f64)| format!("  {name} := point\n  fix(({x:.12}, {y:.12})) {name}\n");
    crowned(&format!("in std.top {{\n  k := point\n  fix(({}, {})) k\n{}{}{}{}  inner := arc(center: k, start: i0, end: i1)\n\
        outer := arc(center: k, start: q0, end: q1)\n  s0 := line(i0, q0)\n  s1 := line(i1, q1)\n\
        vf := face(inner, s1, outer, s0)\n}}\nvane := solid(std.solids.Prism(vf, from: 1mm, to: 31mm))\n\
        part := solid(disc)\nvane union part\npart.vane {word} part.disc\n", centre[0], centre[1],
        p("i0", at(radii[0], from)), p("i1", at(radii[0], to)), p("q0", at(radii[1], from)), p("q1", at(radii[1], to))))
}

/// The arc vane's volume over the crown: in polar coordinates about its centre.
fn arc_vane_stock(centre: [f64; 2], radii: [f64; 2], [from, to]: [f64; 2]) -> f64 {
    disc_volume(issue_big(), -797.0, true) + quad([from.to_radians(), to.to_radians()], &|phi| quad(radii, &|rho| {
        let (x, y) = (centre[0] + rho * phi.cos(), centre[1] + rho * phi.sin());
        rho * (31.0 - crown(x.hypot(y)))
    }))
}

/// The ring along a vane's straight end through `centre` at `deg` from x (an end of an arc vane
/// about `centre`, its sides the cylinders of `radii` about it), outward the way `out` (+1 at the
/// angle the vane's sweep ends at): turned about the line from the crown's centre square to the
/// end, through what the two cylinders keep — at height `h` off the end's plane, a point `ρ` from
/// that line stands `√(h² + (ρ sin θ − k)²)` from the cylinders' axis, `k` the axis's place along
/// the end.
fn end_ring(centre: [f64; 2], radii: [f64; 2], deg: f64, out: f64, r: f64, word: Option<f64>) -> f64 {
    let (s, c) = deg.to_radians().sin_cos();
    let (n, m) = ([-s * out, c * out], [c, s]);
    let height = -(n[0] * centre[0] + n[1] * centre[1]);
    let k = m[0] * centre[0] + m[1] * centre[1];
    let bound = |rad: f64| move |h: f64| (0.0, k + (rad * rad - h * h).sqrt());
    let (lo, hi) = (bound(radii[0]), bound(radii[1]));
    match word {
        None => crown_ball(issue_big(), true, height, r, &lo, &hi),
        Some(d) => crown_flat(issue_big(), height, d, &lo, &hi),
    }
}

/// `crown_section`'s chamfer of legs `d` on the convex crown: the conic `h + (|q − c| − big) = d`
/// from its leg on the plane to its leg on the sphere, polar about the sphere's centre.
fn crown_flat(big: f64, c: f64, d: f64, lo: &dyn Fn(f64) -> (f64, f64), hi: &dyn Fn(f64) -> (f64, f64)) -> f64 {
    let num = d - c + big;
    let conic = |b: f64| {
        let (sn, co) = b.sin_cos();
        let t = num / (sn + 1.0);
        let dt = -t * co / (sn + 1.0);
        (t * co, c + t * sn, dt * sn + t * co)
    };
    let l1 = [((big + d).powi(2) - c * c).sqrt(), 0.0];
    let l2 = [(big * big - (d - c).powi(2)).sqrt(), d];
    let angle = |p: [f64; 2]| (p[1] - c).atan2(p[0]);
    crown_section(big, true, c, &conic, [angle(l1), angle(l2)], lo, hi)
}

/// The moment `∫∫ ρ dA` about z of a section in a half-plane through z, by Green's theorem round
/// its boundary: `pieces`, each `(ρ, z, dz/dt)` over its span, end to end round the loop.
fn moment(pieces: &[(&dyn Fn(f64) -> (f64, f64, f64), [f64; 2])]) -> f64 {
    pieces.iter().map(|&(f, span)| quad(span, &|t| {
        let (rho, _, dz) = f(t);
        0.5 * rho * rho * dz
    })).sum::<f64>().abs()
}

/// The fill's section about z where a cylinder `ρ = w` meets a sphere of radius `big` about
/// `(0, zc)` — its cap above the centre (`up`) or below — on the side `s` of the cylinder (`+1`
/// outside it) and outside the sphere or within it (`outside`): a ball of radius `r` between them
/// (`Some(r)`), or else a chamfer's flat of legs `d` (the points whose two distances sum to `d`).
/// Its moment about z: times the turn, its volume (Pappus).
#[allow(clippy::too_many_arguments)]
fn meridian_fill(w: f64, s: f64, big: f64, zc: f64, up: bool, outside: bool, round: Option<f64>, d: f64) -> f64 {
    let (tau, sigma) = (if up { 1.0 } else { -1.0 }, if outside { 1.0 } else { -1.0 });
    let on_sphere = |rho: f64| zc + tau * (big * big - rho * rho).sqrt();
    let k = on_sphere(w);
    let short = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    let line = |z0: f64, z1: f64| move |t: f64| (w, z0 + (z1 - z0) * t, z1 - z0);
    let about = |o: [f64; 2], rad: f64| move |a: f64| (o[0] + rad * a.cos(), o[1] + rad * a.sin(), rad * a.cos());
    let angle = |o: [f64; 2], p: [f64; 2]| (p[1] - o[1]).atan2(p[0] - o[0]);
    let centre = [0.0, zc];
    match round {
        Some(r) => {
            let rb = big + sigma * r;
            let b = [w + s * r, zc + tau * (rb * rb - (w + s * r).powi(2)).sqrt()];
            let touch = [b[0] * big / rb, zc + (b[1] - zc) * big / rb];
            let (a0, a1) = (angle(b, [w, b[1]]), angle(b, touch));
            let (b0, b1) = (angle(centre, touch), angle(centre, [w, k]));
            moment(&[(&line(k, b[1]), [0.0, 1.0]), (&about(b, r), [a0, a0 + short(a1 - a0)]),
                (&about(centre, big), [b0, b0 + short(b1 - b0)])])
        }
        None => {
            // the leg on the cylinder, `d` from the sphere, and on the sphere, `d` from the cylinder
            let l1 = [w, zc + tau * ((big + sigma * d).powi(2) - w * w).sqrt()];
            let l2 = [w + s * d, on_sphere(w + s * d)];
            // the flat: `s(ρ − w) + σ(|q − c| − big) = d`, polar about the centre
            let flat = move |a: f64| {
                let (sn, co) = a.sin_cos();
                let t = (d + s * w + sigma * big) / (s * co + sigma);
                let dt = s * sn * t / (s * co + sigma);
                (t * co, zc + t * sn, dt * sn + t * co)
            };
            let (f0, f1) = (angle(centre, l1), angle(centre, l2));
            let (b0, b1) = (angle(centre, l2), angle(centre, [w, k]));
            moment(&[(&line(k, l1[1]), [0.0, 1.0]), (&flat, [f0, f0 + short(f1 - f0)]),
                (&about(centre, big), [b0, b0 + short(b1 - b0)])])
        }
    }
}

/// **#962's 7, built** (`rounds/vane.sv`): the vane's four fills are torus stretches, the sides'
/// two cut by the ends' planes and the ends' two by the sides' — a spiric section each — where on
/// `main` the Boolean refused the cap of one against the next one's torus, which touches the cap's
/// plane round a circle. A drawing of no freedom; 22 faces, six of them tori, 198988.300 mm³.
///
/// **The two faces thinner than the export's 10 µm are the set's own** (`sliver::slivers`): at each
/// of the vane's near corners the end's fill touches the end's plane along a circle that meets the
/// corner's upright edge at `z_e = √(ρ₀² − 4) − 797 = 11.0228` (`ρ₀ = √((R + r)² − 18²)`), and the
/// side's fill meets that edge lower, at `z_s = √((R + r)² − 20² − 4²) − 797 = 10.9684`; between
/// them the end's fill's cap in the side's plane goes on beside the vane's side in one plane, a face
/// of its own, which the ball's trace closes onto the edge at `z_e` as a cusp: `0.741 µm` across at
/// `z_s` (`20 − 18 − √(r² − (√(4 + (z_s + 797)²) − ρ₀)²)`), read `0.76` and `0.79 µm` off the
/// fitted trace. On a flat disc the two fills reach the edge at one height, `r` over the disc, and
/// the cusp's tip is a vertex (`a_vane_on_a_flat_disc_leaves_no_sliver`).
#[test]
fn a_vane_filled_into_a_spherical_crown_is_four_torus_stretches() {
    let src = include_str!("../../examples/rounds/vane.sv");
    assert_eq!(dof(src), 0);
    let v = Vane::issue(2.0);
    let b = holds(src, v.stock(), v.fills(), 1e-7);
    assert_eq!(kinds(&b).get("torus"), Some(&6), "{:?}", kinds(&b));
    assert_eq!(b.faces.len(), 22);
    assert!((props::volume(&b) - 198988.300).abs() < 5e-4, "{}", props::volume(&b));
    let (r, rb) = (2.0, issue_big() + 2.0);
    let rho0 = (rb * rb - 18f64.powi(2)).sqrt();
    let zs = (rb * rb - 400.0 - 16.0).sqrt();
    let across = 20.0 - 18.0 - (r * r - ((4.0 + zs * zs).sqrt() - rho0).powi(2)).sqrt();
    assert!((across - 0.741e-3).abs() < 1e-6, "{across:e}");
    let thin = gcs_core::brep::sliver::slivers(&b, 0.01);
    assert_eq!(thin.len(), 2, "{thin:?}");
    for s in &thin {
        assert!(matches!(s.thin, gcs_core::brep::sliver::Thin::Face(_)) && b.faces[s.of[0]].surface.kind() == "plane", "{s:?}");
        assert!(s.size >= across && s.size <= across + 0.1e-3, "{:e} against {across:e}", s.size);
        let at = [20.0 - 0.5 * across, s.at[1].signum() * 2.0, zs - 797.0];
        assert!(gcs_core::space::distance(s.at, at) <= 2e-3, "{:?} against {at:?}", s.at);
    }
}

/// The same vane on a disc whose top is flat (z = 3): each fill a prism of the corner's section
/// along its edge, cut by the planes of the sides beside it, and the side's and the end's reach the
/// corner's upright edge at one height, `3 + r`, so no face is thinner than a micrometre: the disc,
/// the vane over it and four runs, `(1 − π/4) r²` along each edge.
#[test]
fn a_vane_on_a_flat_disc_leaves_no_sliver() {
    let src = Vane::issue(2.0).document();
    let cut = src.find("in std.top").unwrap();
    let flat = format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
        in std.front {{\n  o := point\n  fix((0, 0)) o\n  r := point\n  fix((100, 0)) r\n  t := point\n  fix((100, 3)) t\n\
        top := point\n  fix((0, 3)) top\n  section := face(o, r, t, top, -> close)\n}}\n\
        disc := solid(std.solids.Revolution(section, about: std.z))\n{}", &src[cut..]);
    let b = exported(&solved(&flat), "part");
    let r = 2.0;
    close(props::volume(&b), PI * 1e4 * 3.0 + 70.0 * 4.0 * 28.0 + square_run(r) * 2.0 * (70.0 + 4.0));
    assert_eq!(gcs_core::brep::sliver::slivers(&b, 1e-3).len(), 0);
}

/// The vane at every radius of round its crown holds (`vane.sv`'s suggested edit), through the one
/// whose fill along the near end touches the crown at its top, where the disc's axis crosses it
/// (20, the end's distance from that axis), and past it, its strip on the crown over the crown's
/// pole — the sphere's chart collapsing there, the meridian the fill's two stretches are glued on
/// running into the pole and out the other side
/// (`brep::a_cut_through_a_pole_along_the_seams_meridian_parts_the_sphere`) — and, the vane moved
/// out to x = 25, at 21.
#[test]
fn the_vane_filled_at_every_radius_its_crown_holds() {
    let src = include_str!("../../examples/rounds/vane.sv");
    for r in [0.5, 3.9, 19.5, 20.0, 20.5] {
        let v = Vane::issue(r);
        holds(&src.replace("r := 2mm", &format!("r := {r}mm")), v.stock(), v.fills(), 1e-7);
    }
    let v = Vane { e: [25.0, 90.0], ..Vane::issue(21.0) };
    holds(&v.document(), v.stock(), v.fills(), 1e-7);
}

/// A vane whose side's plane holds the disc's axis (`y = 0`, from x = −40 to 40), and one standing
/// over the axis, from `y = −1` to 3: each side's fill a ring about the line square to it through
/// the crown's centre, the one on `y = 0` running over the crown's top. On `main` the vane's union
/// with the disc alone was refused, the crown's face "a sphere with a hole in no piece", its pole
/// on the side's meeting with it.
#[test]
fn a_vane_whose_side_runs_through_the_crowns_axis() {
    for v in [Vane { e: [-40.0, 40.0], s: [0.0, 4.0], ..Vane::issue(2.0) },
              Vane { e: [-40.0, 40.0], s: [-1.0, 3.0], ..Vane::issue(2.0) }] {
        holds(&v.document(), v.stock(), v.fills(), 1e-7);
    }
}

/// A crown of R150, and dished crowns — the disc's top a sphere's cap below its centre — of R797
/// and R150: the ball within the sphere, its fills tori all the same.
#[test]
fn a_vane_on_a_small_crown_and_on_dished_ones() {
    let rise = |big: f64| (big * big - 1e4).sqrt();
    for v in [
        Vane { big: 150.0, zc: 3.0 - rise(150.0), top: 60.0, ..Vane::issue(2.0) },
        Vane { big: 797.0, rim: 10.0, zc: 10.0 + rise(797.0), ..Vane::issue(2.0) },
        Vane { big: 150.0, rim: 50.0, zc: 50.0 + rise(150.0), top: 80.0, ..Vane::issue(2.0) },
    ] {
        holds(&v.document(), v.stock(), v.fills(), 1e-7);
    }
}

/// The vane anywhere on the crown: off its centre and skew to every meridian (turned 0.7 about z,
/// 15 to 19 off the line it is turned from), its mirror image, and across the crown's axis.
#[test]
fn a_vane_off_the_axis_skew_and_mirrored() {
    for v in [
        Vane { e: [10.0, 60.0], s: [15.0, 19.0], alpha: 0.7, ..Vane::issue(2.0) },
        Vane { e: [10.0, 60.0], s: [-19.0, -15.0], alpha: -0.7, ..Vane::issue(2.0) },
        Vane { e: [-30.0, 40.0], ..Vane::issue(1.0) },
    ] {
        holds(&v.document(), v.stock(), v.fills(), 1e-7);
    }
}

/// The fixture's vanes: sides arcs concentric with the crown's axis, 55 and 59 from it over ±5°.
/// Each side's fill is a ring about that axis (Pappus over the meridian's section), each end's a
/// torus stretch about the line square to it through the crown's centre, kept between the
/// cylinders; rounded, and chamfered (on `main` the chamfer's crease, a meeting of the sphere and
/// a cylinder fitted, was refused against the cylinder a ring's cap lies on).
#[test]
fn a_vane_whose_sides_are_concentric_with_the_crown() {
    let (centre, radii, angles) = ([0.0, 0.0], [55.0, 59.0], [-5.0, 5.0]);
    let turn = 10f64.to_radians();
    let stock = arc_vane_stock(centre, radii, angles);
    let side = |w: f64, s: f64, round: Option<f64>, d: f64| turn * meridian_fill(w, s, issue_big(), -797.0, true, true, round, d);
    let fills = side(55.0, -1.0, Some(2.0), 0.0) + side(59.0, 1.0, Some(2.0), 0.0)
        + end_ring(centre, radii, -5.0, -1.0, 2.0, None) + end_ring(centre, radii, 5.0, 1.0, 2.0, None);
    holds(&arc_vane(centre, radii, angles, "round(r: 2mm)"), stock, fills, 1e-7);
    let flats = side(55.0, -1.0, None, 2.0) + side(59.0, 1.0, None, 2.0)
        + end_ring(centre, radii, -5.0, -1.0, 0.0, Some(2.0)) + end_ring(centre, radii, 5.0, 1.0, 0.0, Some(2.0));
    holds(&arc_vane(centre, radii, angles, "chamfer(d: 2mm)"), stock, flats, 1e-7);
}

/// Vanes whose sides are arcs about a line 20 off the crown's axis (75 and 79 about (−20, 0)):
/// each side's meeting with the crown no circle, so its ball is rolled along it — refused on
/// `main` as larger than the crown can hold, the side's cylinder coming within the ball's radius
/// of the disc's rim across the disc — and each end's still a torus stretch, the end's plane off
/// the crown's centre.
#[test]
fn a_vane_whose_sides_are_about_another_axis_rolls_its_balls() {
    let (centre, radii, angles) = ([-20.0, 0.0], [75.0, 79.0], [-5.0, 5.0]);
    let r = 2.0;
    let stock = arc_vane_stock(centre, radii, angles);
    // each side's run between its ends' planes, which are half-planes about the sides' axis
    let wedge = [-5f64.to_radians(), 5f64.to_radians()];
    let side = |radius: f64, s: f64| rolled_about(centre, radius, s, [0.0, 0.0, -797.0], issue_big(), r, wedge);
    let rolled = side(75.0, -1.0) + side(79.0, 1.0);
    let rings = end_ring(centre, radii, -5.0, -1.0, r, None) + end_ring(centre, radii, 5.0, 1.0, r, None);
    holds(&arc_vane(centre, radii, angles, "round(r: 2mm)"), stock, rolled + rings, 1e-6);
}

/// The vane trimmed by a wedge as the fixture's is: cut by the plane through (40, 40) and
/// (100, −10) in the front plane, which meets the crown near x = 82 at 39.8° off upright. Its fill
/// there is a torus stretch about the line square to the plane through the crown's centre, kept
/// between the vane's sides; each side's is kept between the near end and that plane, which lies
/// along the side's ring's axis: a turn `β − π/2 + asin(d / ρ)`, `d` the plane's distance.
#[test]
fn a_vane_trimmed_by_a_wedge() {
    let r = 2.0;
    let src = crowned("in std.top {\n  a := point\n  b := point\n  d := point\n  e := point\n\
        fix((20, -2)) a\n  fix((90, -2)) b\n  fix((90, 2)) d\n  fix((20, 2)) e\n  vf := face(a, b, d, e, -> close)\n}\n\
        vane := solid(std.solids.Prism(vf, from: 1mm, to: 31mm))\n\
        in std.front {\n  w0 := point\n  w1 := point\n  w2 := point\n  fix((40, 40)) w0\n  fix((100, 40)) w1\n\
        fix((100, -10)) w2\n  wf := face(w0, w1, w2, -> close)\n}\n\
        wedge := solid(std.solids.Prism(wf, from: -10mm, to: 10mm))\nblade := solid(vane)\nwedge cut blade\n\
        part := solid(disc)\nblade union part\npart.blade round(r: 2mm) part.disc\n");
    let (big, zc) = (issue_big(), -797.0);
    // the slant `z = p − q x`, its outward normal β from upright toward x, `dist` from the
    // crown's centre
    let (p, q): (f64, f64) = (40.0 + 40.0 * 5.0 / 6.0, 5.0 / 6.0);
    let beta = q.atan();
    let dist = (p - zc) / (1.0 + q * q).sqrt();
    // the vane over the crown: along x in closed form, across it by quadrature
    let stock = disc_volume(big, zc, true) + quad([-2.0, 2.0], &|y| {
        let a2 = big * big - y * y;
        let f = |x: f64| under_circle(a2, x);
        let knee = (p - 31.0) / q;
        // where the slant comes down to the crown: (p − zc − q x)² = a² − x²
        let (pp, qq) = (p - zc, q);
        let (qa, qb, qc) = (1.0 + qq * qq, -2.0 * pp * qq, pp * pp - a2);
        let root = (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
        let root = if root > knee && root < 100.0 { root } else { (-qb - (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa) };
        (31.0 - zc) * (knee - 20.0) - (f(knee) - f(20.0))
            + (pp * (root - knee) - 0.5 * qq * (root * root - knee * knee)) - (f(root) - f(knee))
    });
    let sides = |c: f64| crown_ball(big, true, c, r, &|_| (0.0, 20.0), &|_| (beta - PI / 2.0, dist));
    let fills = 2.0 * sides(-2.0) + crown_ball(big, true, 20.0, r, &|_| (0.0, -2.0), &|_| (0.0, 2.0))
        + crown_ball(big, true, -dist, r, &|_| (0.0, -2.0), &|_| (0.0, 2.0));
    holds(&src, stock, fills, 1e-7);
}

/// A round boss standing from z = 1 to 30, its rim a circle of `radius` about `(x, 0)` in the
/// top plane.
fn boss(x: f64, radius: f64) -> String {
    format!("in std.top {{\n  k := point\n  fix(({x}, 0)) k\n  rim := circle(center: k) \
        hint(r: {radius})\n  radius({radius}mm) rim\n}}\n\
        boss := solid(std.solids.Prism(face(rim), from: 1mm, to: 30mm))\n")
}

/// A round boss on the crown: on its axis, a ring about it (Pappus); off it, 50 out, its foot no
/// circle, the ball rolled round it (the quadrature, #85).
#[test]
fn a_boss_on_the_crown_on_its_axis_and_off_it() {
    let (big, zc, r) = (issue_big(), -797.0, 2.0);
    let on_crown = |x: f64, radius: f64| crowned(&format!("{}part := solid(disc)\nboss union part\n\
        part.boss round(r: 2mm) part.disc\n", boss(x, radius)));
    let disc = disc_volume(big, zc, true);
    let over = |at: [f64; 2], radius: f64| over_disc(at, radius, &|p| 30.0 - crown(p[0].hypot(p[1])));
    holds(&on_crown(0.0, 10.0), disc + over([0.0, 0.0], 10.0),
        2.0 * PI * meridian_fill(10.0, 1.0, big, zc, true, true, Some(r), 0.0), 1e-9);
    let rolled = rolled_round([50.0, 0.0], 8.0, 1.0, [0.0, 0.0, zc], big, true, r);
    holds(&on_crown(50.0, 8.0), disc + over([50.0, 0.0], 8.0), rolled, 1e-6);
}

/// A pocket 40 in radius sunk into the crown, its floor a dish of R300 whose lowest point is at
/// z = 4, and a boss standing on the floor (the pocket named before the boss stands in it), filled
/// into it: on the floor's axis a ring, off it a rolled ball, within the floor's sphere.
#[test]
fn a_boss_on_a_spherical_pocket_floor() {
    let (big, zc, r) = (issue_big(), -797.0, 2.0);
    let pocket = |x: f64, radius: f64| {
        let rim = 304.0 - (300f64.powi(2) - 1600.0).sqrt();
        crowned(&format!("in std.front {{\n  p0 := point\n  fix((0, 4)) p0\n  p1 := point\n  fix((0, 20)) p1\n\
            p2 := point\n  fix((40, 20)) p2\n  p3 := point\n  fix((40, {rim:.12})) p3\n  pc := point\n  fix((0, 304)) pc\n\
            floor := arc(center: pc, start: p0, end: p3)\n  axis0 := line(p1, p0)\n  lid := line(p2, p1)\n\
            wall := line(p3, p2)\n  hollow := face(floor, wall, lid, axis0)\n}}\n\
            pocket := solid(std.solids.Revolution(hollow, about: std.z))\n{}\
            dished := solid(disc)\npocket cut dished\npart := solid(dished)\nboss union part\n\
            part.boss round(r: 2mm) part.dished\n", boss(x, radius)))
    };
    let floor = |rho: f64| 304.0 - (90_000.0 - rho * rho).sqrt();
    // the disc less the pocket under its crown, `∫ 2πρ (crown − floor)` within 40
    let dished = disc_volume(big, zc, true) - 2.0 * PI * (zc * 800.0 + (big.powi(3) - (big * big - 1600.0).powf(1.5)) / 3.0
        - 304.0 * 800.0 + (27e6 - (90_000f64 - 1600.0).powf(1.5)) / 3.0);
    let over = |at: [f64; 2], radius: f64| over_disc(at, radius, &|p| 30.0 - floor(p[0].hypot(p[1])));
    holds(&pocket(0.0, 8.0), dished + over([0.0, 0.0], 8.0),
        2.0 * PI * meridian_fill(8.0, 1.0, 300.0, 304.0, false, false, Some(r), 0.0), 1e-9);
    let rolled = rolled_round([20.0, 0.0], 6.0, 1.0, [0.0, 0.0, 304.0], 300.0, false, r);
    holds(&pocket(20.0, 6.0), dished + over([20.0, 0.0], 6.0), rolled, 1e-6);
}
