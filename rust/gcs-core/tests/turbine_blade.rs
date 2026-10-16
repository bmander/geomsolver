//! **The turbine blade** (`rust/examples/turbine_blade.sv`): an airfoil of circular-arc sections
//! at the hub, the mean and the tip, every number of the design table blended through the three
//! along the span, on a platform, seal wings and a fir-tree root.  Held here: the document solves
//! to DOF 0; each design section's centroid stands on the stacking axis, as its comments say; and
//! the airfoil read off the exact B-rep is the integral of the section the textbook construction
//! gives — built here apart from the document, its area off its five arcs in closed form — over
//! the span, to a part in 10⁹.
use gcs_core::program::Elaborated;

use crate::common::parsed::determined;

const BLADE: &str = include_str!("../../examples/turbine_blade.sv");

/// A design section: chord, stagger, inlet and exit angles, the two wedges, the edges' radii, the
/// suction side's aft radius, and the chord's middle from the stacking axis (mm and degrees).
#[derive(Clone, Copy)]
struct Design {
    chord: f64,
    stagger: f64,
    inlet: f64,
    exit: f64,
    le_wedge: f64,
    te_wedge: f64,
    r_le: f64,
    r_te: f64,
    r_aft: f64,
    mid: [f64; 2],
}

const HUB: Design = Design { chord: 30.0, stagger: -18.0, inlet: 40.0, exit: -65.0,
                             le_wedge: 40.0, te_wedge: 8.0, r_le: 1.6, r_te: 0.6, r_aft: 40.0,
                             mid: [0.772, -5.807] };
const MEAN: Design = Design { chord: 29.0, stagger: -22.5, inlet: 33.0, exit: -66.5,
                              le_wedge: 37.0, te_wedge: 7.0, r_le: 1.4, r_te: 0.55, r_aft: 42.0,
                              mid: [0.386, -5.406] };
const TIP: Design = Design { chord: 28.0, stagger: -28.0, inlet: 25.0, exit: -68.0,
                             le_wedge: 34.0, te_wedge: 6.0, r_le: 1.2, r_te: 0.5, r_aft: 45.0,
                             mid: [0.082, -5.014] };

/// The section at span fraction `s`: each number the quadratic through the hub's, the mean's and
/// the tip's, as `Station` blends them.
fn at(s: f64) -> Design {
    let (h, m, t) = ((1.0 - s) * (1.0 - 2.0 * s), 4.0 * s * (1.0 - s), s * (2.0 * s - 1.0));
    let b = |f: fn(&Design) -> f64| h * f(&HUB) + m * f(&MEAN) + t * f(&TIP);
    Design { chord: b(|d| d.chord), stagger: b(|d| d.stagger), inlet: b(|d| d.inlet),
             exit: b(|d| d.exit), le_wedge: b(|d| d.le_wedge), te_wedge: b(|d| d.te_wedge),
             r_le: b(|d| d.r_le), r_te: b(|d| d.r_te), r_aft: b(|d| d.r_aft),
             mid: [b(|d| d.mid[0]), b(|d| d.mid[1])] }
}

fn dir(deg: f64) -> [f64; 2] {
    let a = deg.to_radians();
    [a.cos(), a.sin()]
}
fn add(p: [f64; 2], q: [f64; 2], k: f64) -> [f64; 2] {
    [p[0] + k * q[0], p[1] + k * q[1]]
}
fn sub(p: [f64; 2], q: [f64; 2]) -> [f64; 2] {
    [p[0] - q[0], p[1] - q[1]]
}
fn dot(p: [f64; 2], q: [f64; 2]) -> f64 {
    p[0] * q[0] + p[1] * q[1]
}

/// An arc of the profile: its centre, its radius, and its bearings from the centre at the start
/// and the end, the end taken past the start the way the arc turns.
struct Arc {
    c: [f64; 2],
    r: f64,
    a0: f64,
    a1: f64,
}

impl Arc {
    fn new(c: [f64; 2], r: f64, from: [f64; 2], to: [f64; 2], cw: bool) -> Arc {
        let bearing = |p: [f64; 2]| (p[1] - c[1]).atan2(p[0] - c[0]);
        let (a0, mut a1) = (bearing(from), bearing(to));
        let turn = std::f64::consts::TAU;
        while cw && a1 > a0 {
            a1 -= turn;
        }
        while !cw && a1 < a0 {
            a1 += turn;
        }
        Arc { c, r, a0, a1 }
    }

    /// `∮ (x dy − y dx) / 2`, `∮ x² dy / 2` and `−∮ y² dx / 2` along it: its share of the area
    /// and of the first moments (Green's theorem), in closed form.
    fn moments(&self) -> [f64; 3] {
        let ([cx, cy], r) = (self.c, self.r);
        let area = |a: f64| r * cx * a.sin() - r * cy * a.cos() + r * r * a;
        let mx = |a: f64| r * cx * cx * a.sin() + cx * r * r * (a + (2.0 * a).sin() / 2.0)
            + r.powi(3) * (a.sin() - a.sin().powi(3) / 3.0);
        let my = |a: f64| -r * cy * cy * a.cos() + cy * r * r * (a - (2.0 * a).sin() / 2.0)
            + r.powi(3) * (-a.cos() + a.cos().powi(3) / 3.0);
        let d = |f: &dyn Fn(f64) -> f64| (f(self.a1) - f(self.a0)) / 2.0;
        [d(&area), d(&mx), d(&my)]
    }
}

/// The section's five arcs, counter-clockwise, built as the textbook builds them: the edges'
/// circles a chord apart at the stagger; the suction side leaving the leading edge at the inlet
/// angle plus half the wedge and reaching the trailing edge at the exit angle less half its
/// wedge, two arcs, the aft one of radius `r_aft`; the pressure side one arc leaving the leading
/// edge at the inlet angle less half the wedge and touching the trailing edge's circle.
fn profile(d: &Design) -> [Arc; 5] {
    let half = (d.chord - d.r_le - d.r_te) / 2.0;
    let cle = add(d.mid, dir(d.stagger), -half);
    let cte = add(d.mid, dir(d.stagger), half);
    let ss_in = d.inlet + d.le_wedge / 2.0;
    let ps_in = d.inlet - d.le_wedge / 2.0;
    let ss_out = d.exit - d.te_wedge / 2.0;
    let ss_le = add(cle, dir(ss_in + 90.0), d.r_le);
    let ps_le = add(cle, dir(ps_in - 90.0), d.r_le);
    let ss_te = add(cte, dir(ss_out + 90.0), d.r_te);
    // the pressure side's centre on the normal at `ps_le`, its circle touching the trailing
    // edge's from outside: |ps_le + R m − cte| = R + r_te
    let m = dir(ps_in - 90.0);
    let e = sub(ps_le, cte);
    let big = (d.r_te * d.r_te - dot(e, e)) / (2.0 * (dot(e, m) - d.r_te));
    let c_ps = add(ps_le, m, big);
    let ps_te = add(c_ps, sub(cte, c_ps), big / dot(sub(cte, c_ps), sub(cte, c_ps)).sqrt());
    // the suction side's forward centre on the normal at `ss_le`, its circle inside the aft
    // one's and touching it: |ss_le + r m − c_aft| = r_aft − r
    let c_aft = add(ss_te, dir(ss_out - 90.0), d.r_aft);
    let m = dir(ss_in - 90.0);
    let e = sub(ss_le, c_aft);
    let r_fwd = (d.r_aft * d.r_aft - dot(e, e)) / (2.0 * (dot(e, m) + d.r_aft));
    let c_fwd = add(ss_le, m, r_fwd);
    let k = sub(c_fwd, c_aft);
    let crest = add(c_aft, k, d.r_aft / dot(k, k).sqrt());
    [
        Arc::new(c_ps, big, ps_le, ps_te, true),
        Arc::new(cte, d.r_te, ps_te, ss_te, false),
        Arc::new(c_aft, d.r_aft, ss_te, crest, false),
        Arc::new(c_fwd, r_fwd, crest, ss_le, false),
        Arc::new(cle, d.r_le, ss_le, ps_le, false),
    ]
}

/// The section's area and its centroid.
fn area_and_centroid(d: &Design) -> (f64, [f64; 2]) {
    let mut sum = [0.0; 3];
    for a in profile(d) {
        for (s, m) in sum.iter_mut().zip(a.moments()) {
            *s += m;
        }
    }
    (sum[0], [sum[1] / sum[0], sum[2] / sum[0]])
}

/// The volume of the solid `name` off its exact B-rep, its boundary checked.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let i = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    let x = e.sketch.exact_solid(i).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// `∫ₐᵇ f` by the five-point Gauss–Legendre rule on `n` panels.
fn gauss(a: f64, b: f64, n: usize, f: impl Fn(f64) -> f64) -> f64 {
    let (p, q) = (0.538_469_310_105_683_1, 0.906_179_845_938_664);
    let (wp, wq) = (0.478_628_670_499_366_5, 0.236_926_885_056_189_1);
    let h = (b - a) / n as f64;
    (0..n).map(|k| {
        let at = |x: f64| f(a + h * (k as f64 + 0.5 + 0.5 * x));
        0.5 * h * (128.0 / 225.0 * at(0.0) + wp * (at(-p) + at(p)) + wq * (at(-q) + at(q)))
    }).sum()
}

/// **Stacked on their centroids**: the design table's `mid` puts each design section's centroid
/// on the stacking axis, to the table's three decimals.
#[test]
fn each_design_section_stands_on_its_centroid() {
    for (name, d) in [("hub", HUB), ("mean", MEAN), ("tip", TIP)] {
        let (area, [x, y]) = area_and_centroid(&d);
        assert!(area > 100.0, "{name}: {area}");
        assert!(x.hypot(y) <= 1e-3, "{name}'s centroid at ({x}, {y})");
    }
}

/// **The airfoil is its sections**: the document solves to DOF 0, and the airfoil it lofts —
/// 40 mm of span and the millimetre it runs into the platform — is the integral of the
/// construction's area over the span, to a part in 10⁹; the blade it is part of builds.
#[test]
fn the_airfoil_is_the_integral_of_its_sections() {
    let e = determined(BLADE);
    let (span, embed) = (40.0, 1.0);
    let want = span * gauss(-embed / span, 1.0, 16, |s| area_and_centroid(&at(s)).0);
    let got = volume(&e, "airfoil");
    assert!((got - want).abs() <= 1e-9 * want, "the airfoil: {got} against {want}");
    let blade = volume(&e, "blade");
    assert!(blade > got && blade.is_finite(), "the blade: {blade}");
}
