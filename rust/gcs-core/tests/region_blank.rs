//! **The hypoid's blank, as a region, held to its own faces** (#145 F2/F3, #212).  Each member
//! is cut from `solid(R)`, `R` its blank's region — within the tip cone and the heel sphere,
//! outside the toe sphere and the back cone, sized by the solved layout (`blank/member.sv`).  The
//! region's faces are named by the sets its body uses (`tip`, `heel`, `toe`, `back`), and each is
//! held here to its closed form, read off the layout's own points and nothing the region makes:
//! the spheres about the member's apex through its face span's ends, and the cones the pitch cone
//! (about the member's axis from its apex, through the mean point) moved out by the addendum and
//! in by the back depth, in normal modules.  The region's exact boundary has those four faces, each
//! on its surface; its volume is the closed form's; and its material is the closed form's at points
//! through the blank and round it — at every design the layout records (`fixtures::gear::designs`),
//! whose layout `tests/hypoid_layout.rs` holds to the recorded pair.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::space::{add, cross, dot, norm, scale, sub};

type V = [f64; 3];

fn named(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

fn point(e: &Elaborated, name: &str) -> V {
    e.sketch.world_point(named(e, name))
}

fn line(e: &Elaborated, name: &str) -> (V, V) {
    let l = &e.sketch.lines[named(e, name)];
    (e.sketch.world_point(l.p1 as usize), e.sketch.world_point(l.p2 as usize))
}

fn unit(v: V) -> V {
    scale(v, 1.0 / norm(v))
}

/// A member's blank in closed form: its apex and axis, its pitch cone's half angle, its end
/// spheres' radii, and its cones' offsets from the pitch cone (out by the addendum, in by the
/// back depth).
struct Closed { apex: V, axis: V, half: f64, toe: f64, heel: f64, tip: f64, back: f64 }

impl Closed {
    fn read(e: &Elaborated, m: &str) -> Closed {
        let (apex, foot) = line(e, &format!("pair.{m}.ax"));
        let axis = unit(sub(foot, apex));
        let (from, mean) = line(e, &format!("pair.{m}.pitch_line"));
        assert!(norm(sub(from, apex)) < 1e-9 * norm(sub(mean, apex)), "{m}: its pitch line runs from its apex");
        let to_mean = sub(mean, apex);
        let half = norm(cross(to_mean, axis)).atan2(dot(to_mean, axis));
        let radius = |end: &str| norm(sub(point(e, &format!("pair.{m}_blank.span.{end}")), apex));
        let nm = e.sketch.params[e.sketch.free_vars["pair.normal_module"] as usize].value;
        Closed { apex, axis, half, toe: radius("toe"), heel: radius("heel"), tip: nm, back: -4.0 * nm }
    }

    /// The signed distance from the pitch cone, out from the axis, of `p` (where it is nearer the
    /// cone's generators than its apex).
    fn off_pitch(&self, p: V) -> f64 {
        let q = sub(p, self.apex);
        let z = dot(q, self.axis);
        let r = norm(sub(q, scale(self.axis, z)));
        r * self.half.cos() - z * self.half.sin()
    }

    /// Whether `p` is the blank's material, by more than `margin` either way (`None` within it).
    fn holds(&self, p: V, margin: f64) -> Option<bool> {
        let rho = norm(sub(p, self.apex));
        let d = self.off_pitch(p);
        let slack = [rho - self.toe, self.heel - rho, self.tip - d, d - self.back];
        if slack.iter().all(|s| *s > margin) { Some(true) }
        else if slack.iter().any(|s| *s < -margin) { Some(false) }
        else { None }
    }

    /// The blank's volume: about the axis, between the end spheres, between the cones — at a
    /// distance `rho` from the apex the polar angles `half + asin(back / rho)` to `half + asin(tip /
    /// rho)` — integrated in `rho` by Gauss–Legendre over many panels.
    fn volume(&self) -> f64 {
        let (nodes, weights) = ([-0.906179845938664, -0.538469310105683, 0.0, 0.538469310105683, 0.906179845938664],
            [0.236926885056189, 0.478628670499366, 0.568888888888889, 0.478628670499366, 0.236926885056189]);
        let f = |rho: f64| {
            let (a, b) = (self.half + (self.back / rho).asin(), self.half + (self.tip / rho).asin());
            rho * rho * (a.cos() - b.cos())
        };
        let panels = 400;
        let h = (self.heel - self.toe) / panels as f64;
        let mut sum = 0.0;
        for k in 0..panels {
            let mid = self.toe + h * (k as f64 + 0.5);
            for (x, w) in nodes.iter().zip(weights) {
                sum += w * f(mid + 0.5 * h * x) * 0.5 * h;
            }
        }
        std::f64::consts::TAU * sum
    }
}

#[test]
fn the_blank_is_held_to_its_own_faces() {
    for (label, configuration) in fixtures::gear::designs() {
        let e = fixtures::gear::read_file("pair.sv", &configuration);
        for m in ["gear", "pinion"] {
            let closed = Closed::read(&e, m);
            let region = named(&e, &format!("pair.{m}_blank.material"));
            let x = e.sketch.exact_solid(region).unwrap_or_else(|why| panic!("{label} {m}: {why}"));
            // its faces, each on its closed form
            let mut faces: Vec<&str> = x.named.iter().map(|(_, f)| f.as_str()).collect();
            faces.sort();
            assert_eq!(faces, ["back", "heel", "tip", "toe"], "{label} {m}");
            let size = closed.heel;
            for (f, (_, name)) in x.brep.faces.iter().zip(&x.named) {
                for c in f.loops.iter().flatten() {
                    let edge = &x.brep.edges[c.edge as usize];
                    for k in 0..=8 {
                        let t = edge.t[0] + (edge.t[1] - edge.t[0]) * k as f64 / 8.0;
                        let p = add(scale(edge.point(t, &x.brep.vertices), 1.0 / x.mm), x.origin);
                        let off = match name.as_str() {
                            "toe" => norm(sub(p, closed.apex)) - closed.toe,
                            "heel" => norm(sub(p, closed.apex)) - closed.heel,
                            "tip" => closed.off_pitch(p) - closed.tip,
                            "back" => closed.off_pitch(p) - closed.back,
                            other => panic!("{label} {m}: a face `{other}`"),
                        };
                        assert!(off.abs() < 1e-9 * size, "{label} {m}: `{name}` is {off} off its closed form at {p:?}");
                    }
                }
            }
            // its volume, the closed form's
            let (v, want) = (gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3), closed.volume());
            assert!((v - want).abs() <= 1e-9 * want, "{label} {m}: {v} against {want}");
            // its material, the closed form's, at points through the blank and round it
            let field = MaterialField::read(&e.sketch, region, 1e-10).unwrap();
            let up = if closed.axis[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
            let across = unit(cross(closed.axis, up));
            let reach = closed.heel * 1.2;
            let (mut decided, mut agreed) = (0, 0);
            for i in 0..24 {
                for j in 0..24 {
                    let p = add(closed.apex, add(scale(closed.axis, reach * (i as f64 / 23.0 * 2.0 - 1.0)),
                        scale(across, reach * j as f64 / 23.0)));
                    let (a, b) = (field.side(p), closed.holds(p, 1e-6 * size));
                    let Some(b) = b else { continue };
                    if a.abs() < 1e-6 { continue }
                    decided += 1;
                    agreed += usize::from((a < 0.0) == b);
                }
            }
            assert!(decided > 400 && agreed == decided, "{label} {m}: {agreed} of {decided}");
        }
    }
}
