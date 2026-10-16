//! Envelopes the solve sees (#283, step 2 of the spiral bevel's restatement): a point held to the
//! envelope of a cone or a cylinder set, or of a named side of a revolution, under a motion in
//! space; a line or a plane touching it, a line along its normal and a circle osculating it at a
//! point held there.  Each is held here to something worked out apart from `envelope::held`: the
//! after-the-solve snapshot (`GeneratedEnvelope`), `motion::Family`'s poses, and closed forms.
use crate::common::build;
use gcs_core::constraints::CKind;
use gcs_core::envelope::held::{self, Contact, Parts};
use gcs_core::envelope::GeneratedEnvelope;
use gcs_core::model::{EntKind, Sketch};
use gcs_core::rng::Rng;
use gcs_core::solid::RevolvedSurface;
use gcs_core::solve;
use gcs_core::space::{cross, dot, norm, sub};

/// A revolution's two kinds of side — a line turned (a cone's frustum) and a circle turned (a
/// torus) — under the mesh of two skew shafts, one of them a screw.
const SIDES: &str = "\
unit mm
use std
in std.front {
  o := point
  q := point
  fix((0, 0)) o
  fix((0, 10)) q
  ax := line(o, q)
  a := point
  b := point
  c := point
  d := point
  fix((6, 2)) a
  fix((9, 7)) b
  fix((12, 7)) c
  fix((12, 2)) d
  l1 := line(a, b)
  l2 := line(b, c)
  l3 := line(c, d)
  l4 := line(d, a)
  k := point
  fix((20, 4)) k
  rim := circle(center: k) hint(r: 2)
  radius(2mm) rim
}
blade := solid(face(l1, l2, l3, l4), about: ax)
ring := solid(face(rim), about: ax)
side := surface(blade, l1)
round := surface(ring, rim)
g1 := point hint((5, 40, 3))
g2 := point hint((9, 41, 13))
fix((5, 40, 3)) g1
fix((9, 41, 13)) g2
gear_ax := line(g1, g2)
spin := frame
std.Turn(spin, about: ax, ratio: 1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.4, advance: 3mm)
f1 := envelope(side, under: spin, seen_from: gear_turn, from: -60deg, to: 60deg)
f2 := envelope(round, under: spin, seen_from: gear_turn, from: -60deg, to: 60deg)
";

fn solved(src: &str) -> Sketch {
    elaborated(src).sketch
}

fn elaborated(src: &str) -> gcs_core::program::Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

fn envelope(sk: &Sketch, name: &str) -> usize {
    sk.envelopes.iter().position(|e| e.name == name).unwrap_or_else(|| panic!("no envelope `{name}`"))
}

fn near(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    norm(sub(a, b)) <= tol * (1.0 + norm(a))
}

/// The solve's reading of a revolution's side at a place is the snapshot's, its chart mapped: `u`
/// along a line is the snapshot's, round a circle `2π` times it, and `v` the snapshot's times
/// the revolution's sweep.  The normal may face the other way; the meshing equation turns with it.
#[test]
fn a_revolutions_sides_read_as_the_snapshot_does() {
    let sk = solved(SIDES);
    let mut rng = Rng::new(7);
    for (name, round) in [("f1", false), ("f2", true)] {
        let i = envelope(&sk, name);
        let snapshot = GeneratedEnvelope::named(&sk, i).unwrap();
        let surface = RevolvedSurface::named(&sk, sk.envelopes[i].surface.unwrap() as usize).unwrap();
        let parts = Parts::of(&sk, i).unwrap();
        assert_eq!(parts.source, if round { held::Source::Round } else { held::Source::Line });
        let k = parts.consts(Contact::Point);
        let values = parts.values(&sk);
        for _ in 0..40 {
            let (u, v, t) = (rng.uniform(0.05, 0.95), rng.uniform(0.05, 0.95), rng.uniform(-0.9, 0.9));
            let theirs = snapshot.evaluate([u, v, t]).unwrap();
            let place = [if round { std::f64::consts::TAU * u } else { u }, v * surface.sweep(), t];
            let ours = held::read(&k, &values, place).unwrap();
            assert!(near(ours.position, theirs.position, 1e-12), "{name}: {ours:?} {theirs:?}");
            let facing = dot(ours.normal, theirs.normal);
            assert!((facing.abs() - 1.0).abs() < 1e-12, "{name}: {facing}");
            assert!(near(ours.velocity, theirs.velocity, 1e-12), "{name}");
            let g = facing.signum() * ours.normal_velocity;
            assert!((g - theirs.normal_velocity).abs() < 1e-11 * (1.0 + g.abs()), "{name}: {g} {}", theirs.normal_velocity);
        }
    }
}

/// A point held to each side's envelope solves onto it: two of its coordinates held where the
/// snapshot finds a point of the envelope, the third seeded off it, the solve puts it there — on
/// the envelope at the contact's own place, which the snapshot reads in its own chart.
#[test]
fn a_point_held_to_a_revolutions_envelope_is_on_it() {
    use gcs_core::envelope::IntersectionOptions;
    let sk = solved(SIDES);
    // a point of each envelope, the snapshot's: its chart's `u` and the roll held, `v` solved
    let on = |name: &str, u: f64, t: f64| -> [f64; 3] {
        let g = GeneratedEnvelope::named(&sk, envelope(&sk, name)).unwrap();
        let [_, v, _] = g.domain();
        let options = IntersectionOptions {
            bounds: [[0.0, 1.0], v, [-1.0, 1.0]],
            parameter_scale: [1.0; 3],
            residual_tolerance: [1e-12; 3],
            max_iterations: 100,
        };
        let seeds = (0..16).map(|k| (k as f64 + 0.5) / 16.0);
        let found = seeds.filter_map(|v| g.intersect(|p, _| [p[0] - u, p[2] - t], [u, v, t], options).ok())
            .next().expect("the envelope passes the section");
        found.contact.position
    };
    let (x1, x2) = (on("f1", 0.4, 0.2), on("f2", 0.3, -0.15));
    let src = format!("{SIDES}\
p := point hint(({}, {}, {}))
fix(x == {}, z == {}) p
p coincident f1
r := point hint(({}, {}, {}))
fix(x == {}, z == {}) r
r coincident f2
", x1[0], x1[1] + 0.3, x1[2], x1[0], x1[2], x2[0], x2[1] - 0.2, x2[2], x2[0], x2[2]);
    let sk = solved(&src);
    for (point, name, want) in [("p", "f1", x1), ("r", "f2", x2)] {
        let i = envelope(&sk, name);
        let c = sk.constraints.iter().find(|c| c.kind == CKind::PointOnEnvelope && c.args[1].ent().i() == i).unwrap();
        let place: Vec<f64> = (2..5).map(|s| sk.params[c.args[s].param() as usize].value).collect();
        let p = sk.world_point(c.args[0].ent().i());
        assert!(near(p, want, 1e-9), "{point}: {p:?} {want:?}");
        let parts = Parts::of(&sk, i).unwrap();
        let ours = held::read(&parts.consts(Contact::Point), &parts.values(&sk), [place[0], place[1], place[2]]).unwrap();
        assert!(near(ours.position, p, 1e-9), "{point}: {:?} {p:?}", ours.position);
        assert!(ours.normal_velocity.abs() < 1e-9, "{point}: {}", ours.normal_velocity);
        // and the snapshot, in its own chart, agrees
        let surface = RevolvedSurface::named(&sk, sk.envelopes[i].surface.unwrap() as usize).unwrap();
        let u = if name == "f2" { place[0] / std::f64::consts::TAU } else { place[0] };
        let v = (place[1] / surface.sweep()).rem_euclid(1.0);
        let theirs = GeneratedEnvelope::named(&sk, i).unwrap().evaluate([u.rem_euclid(1.0), v, place[2]]).unwrap();
        assert!(near(theirs.position, p, 1e-9), "{point}");
        assert!(theirs.normal_velocity.abs() < 1e-9, "{point}");
    }
}

/// A cone set's envelope (`std.ConeThrough`, the face mill's flank), held to a point under the
/// mesh of two skew shafts: the point is where `motion::Family` carries the cone's point at the
/// contact's place, and the cone's normal there is square to its velocity.
#[test]
fn a_point_held_to_a_cone_sets_envelope_is_on_it() {
    let e = elaborated(CONE);
    let sk = &e.sketch;
    let i = envelope(sk, "flank");
    let c = sk.constraints.iter().find(|c| c.kind == CKind::PointOnEnvelope).unwrap();
    let [u, v, t] = [2, 3, 4].map(|s| sk.params[c.args[s].param() as usize].value);
    // the cone as the drawing has it: its apex, its spine, the lean
    let apex = sk.world_point(e.map.ent_named("cone.apex").unwrap().i());
    let ax = sk.world_point(e.map.ent_named("z").unwrap().i());
    let d = ax.map(|x| x / norm(ax));
    let held::Parts { reference, .. } = Parts::of(sk, i).unwrap();
    let e1 = {
        let r = sub(reference, d.map(|x| x * dot(reference, d)));
        r.map(|x| x / norm(r))
    };
    let half = 20f64.to_radians();
    let at = |u: f64, v: f64| -> [f64; 3] {
        let g = [0, 1, 2].map(|k| half.cos() * d[k] + half.sin() * e1[k]);
        let w = g.map(|x| x * u);
        let (s, co) = v.sin_cos();
        let dw = cross(d, w);
        let along = dot(d, w) * (1.0 - co);
        [0, 1, 2].map(|k| apex[k] + w[k] * co + dw[k] * s + d[k] * along)
    };
    // the mesh: `spin` seen from `gear_turn`, the relative node the envelope's reading minted
    let joint = |n: &str| sk.frames[e.map.ent_named(n).unwrap().i()].joint.as_ref().unwrap()
        .motion.unwrap();
    let (source, observer) = (joint("spin"), joint("gear_turn"));
    let mesh = sk.motions.iter().position(|m| matches!(m.def,
        gcs_core::model::MotionDef::Relative { source: s, observer: o }
            if s == source && o == observer)).unwrap();
    let pose = gcs_core::motion::Family::read(sk, mesh).unwrap().at(t).unwrap();
    let h = 1e-6;
    let du = sub(at(u + h, v), at(u - h, v));
    let dv = sub(at(u, v + h), at(u, v - h));
    let surface = gcs_core::envelope::SurfacePoint { position: at(u, v), du, dv };
    let contact = gcs_core::envelope::contact(surface, pose).unwrap();
    let p = sk.world_point(c.args[0].ent().i());
    assert!(near(contact.position, p, 1e-9), "{:?} {p:?}", contact.position);
    assert!(contact.normal_velocity.abs() < 1e-6, "{}", contact.normal_velocity);
    let _ = EntKind::Envelope;
}

/// The face mill's flank of `hypoid.sv`, shrunk to a cone about the `z` axis through the point
/// `(10, 0, 0)` leaning 20°, generating under a mesh of shafts square to one another and offset.
const CONE: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((0, 0, 10))
fix((0, 0, 0)) o
fix((0, 0, 10)) z
ax := line(o, z)
q := point hint((10, 0, 0))
fix((10, 0, 0)) q
a := point hint((0, 30, 0))
b := point hint((10, 30, 0))
fix((0, 30, 0)) a
fix((10, 30, 0)) b
gear_ax := line(a, b)
cone := std.ConeThrough(ax, q, lean: 20deg)
spin := frame
std.Turn(spin, about: ax, ratio: 1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.5)
flank := envelope(cone, under: spin, seen_from: gear_turn, from: -30deg, to: 30deg)
p := point hint((9, 3, 1))
fix(x == 9, z == 1) p
p coincident flank
";

/// A cylinder set turning about an axis parallel to its own, 30 from it: its envelope is the two
/// cylinders about the turning axis, 25 and 35 in radius.  Held at a point of the outer one in
/// the plane square to the axes, a line along its normal there runs through the axis, a line in
/// its tangent plane level with the point runs round it, and the circle osculating its section by
/// that plane is the outer cylinder's own: centred on the axis, 35 in radius.
const ANNULUS: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((0, 0, 10))
fix((0, 0, 0)) o
fix((0, 0, 10)) z
ax := line(o, z)
c1 := point hint((30, 0, -5))
c2 := point hint((30, 0, 5))
fix((30, 0, -5)) c1
fix((30, 0, 5)) c2
cax := line(c1, c2)
cyl := std.Cylinder(cax, r: 5mm)
turn := frame
std.Turn(turn, about: ax, ratio: 1)
flank := envelope(cyl, under: turn, from: -30deg, to: 30deg)
in std.top {
  p := point hint((34, 9))
  kc := point hint((4, -3))
  k := circle(center: kc) hint(r: 30)
}
fix(y == 10) p
p coincident flank
flank curvature(at: p) k
m := point hint((20, 6, 2))
normal := line(p, m)
flank perpendicular(at: p) normal
p distance(12) m
w := point hint((31, 22, 1))
fix(z == 0) w
along := line(p, w)
flank tangent(at: p) along
p distance(5) w
";

#[test]
fn an_envelopes_normal_tangent_and_osculating_circle_are_its_own() {
    let e = elaborated(ANNULUS);
    let sk = &e.sketch;
    let at = |n: &str| sk.world_point(e.map.ent_named(n).unwrap().i());
    let p = at("p");
    let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
    assert!((r - 35.0).abs() < 1e-9 && (p[1] - 10.0).abs() < 1e-12 && p[2].abs() < 1e-12, "{p:?}");
    // the osculating circle: the outer cylinder's section
    let k = e.map.ent_named("k").unwrap();
    let centre = sk.world_point(sk.round_center(k));
    let radius = sk.params[sk.round_radius(k)].value;
    assert!(norm(centre) < 1e-8, "{centre:?}");
    assert!((radius - 35.0).abs() < 1e-8, "{radius}");
    // the normal: through the axis, square to it
    let m = at("m");
    let d = sub(m, p);
    let radial = [p[0] / r, p[1] / r, 0.0];
    assert!(norm(cross(d, radial)) < 1e-9 * norm(d), "{d:?}");
    // the tangent: square to the radius, level
    let w = at("w");
    let t = sub(w, p);
    assert!(dot(t, radial).abs() < 1e-9 && t[2].abs() < 1e-12, "{t:?}");
}

/// Under the mesh of two skew shafts, one a screw, the circle osculating the frustum's envelope's
/// section by the top view at a point held there is the circle through three points of that
/// section a hair apart — each found by the snapshot, the envelope's equation solved with the
/// view and an abscissa.
#[test]
fn an_osculating_circle_is_the_sections_under_a_skew_screw_mesh() {
    use gcs_core::envelope::IntersectionOptions;
    let sk = solved(SIDES);
    let g = GeneratedEnvelope::named(&sk, envelope(&sk, "f1")).unwrap();
    let options = IntersectionOptions {
        bounds: g.domain(),
        parameter_scale: [1.0; 3],
        residual_tolerance: [1e-13; 3],
        max_iterations: 100,
    };
    // where the section passes the abscissa `x`, from a place near it
    let section = |x: f64, seed: [f64; 3]| {
        g.intersect(|_, c| [c.position[2], c.position[0] - x], seed, options).unwrap()
    };
    // a first point of the section, from a place on the envelope well inside its domain
    let start = (0..32).find_map(|k| {
        let seed = [0.5, (k as f64 + 0.5) / 32.0, 0.1];
        g.intersect(|p, c| [c.position[2], p[0] - 0.5], seed, IntersectionOptions {
            bounds: [[0.5, 0.5], g.domain()[1], g.domain()[2]], ..options }).ok()
    }).expect("the envelope crosses the top view");
    let x0 = start.contact.position;
    let src = format!("{SIDES}\
in std.top {{
  p := point hint(({}, {}))
  oc := point hint(({}, {}))
  osc := circle(center: oc) hint(r: 10)
}}
fix(x == {}) p
p coincident f1
f1 curvature(at: p) osc
", x0[0], x0[1] + 0.05, x0[0] + 2.0, x0[1] + 1.0, x0[0]);
    let e = elaborated(&src);
    let sk = &e.sketch;
    let p = sk.world_point(e.map.ent_named("p").unwrap().i());
    assert!(near(p, x0, 1e-9), "{p:?} {x0:?}");
    let k = e.map.ent_named("osc").unwrap();
    let (centre, radius) = (sk.world_point(sk.round_center(k)), sk.params[sk.round_radius(k)].value);
    // the circle through three points of the section
    let h = 2e-3;
    let a = section(x0[0] - h, start.parameters).contact.position;
    let b = section(x0[0] + h, start.parameters).contact.position;
    let (ax, ay, bx, by, cx, cy) = (a[0], a[1], x0[0], x0[1], b[0], b[1]);
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    let sq = |x: f64, y: f64| x * x + y * y;
    let ux = (sq(ax, ay) * (by - cy) + sq(bx, by) * (cy - ay) + sq(cx, cy) * (ay - by)) / d;
    let uy = (sq(ax, ay) * (cx - bx) + sq(bx, by) * (ax - cx) + sq(cx, cy) * (bx - ax)) / d;
    let r3 = ((ax - ux) * (ax - ux) + (ay - uy) * (ay - uy)).sqrt();
    assert!((radius - r3).abs() < 1e-4 * r3, "{radius} {r3}");
    assert!(norm(sub(centre, [ux, uy, 0.0])) < 1e-4 * r3, "{centre:?} {ux} {uy}");
}

/// Every envelope contact prints and parses back the statement it was (#283): a document of a
/// revolution's two sides and a contact of each kind with them, printed flat and read again,
/// elaborates to the same relations.
#[test]
fn every_envelope_contact_prints_and_parses() {
    // the flat form prints no component instance: the joints written out as `std.Turn` writes them
    let sides = SIDES.replace("std.Turn(spin, about: ax, ratio: 1)",
        "ax coincident spin.axis\nfix(ratio == 1, advance == 0mm) spin")
        .replace("std.Turn(gear_turn, about: gear_ax, ratio: -0.4, advance: 3mm)",
        "gear_ax coincident gear_turn.axis\nfix(ratio == -0.4, advance == 3mm) gear_turn");
    let src = format!("{sides}\
p := point hint((3, 1, 8))
p coincident f1
in std.top {{
  oc := point hint((5, 2))
  osc := circle(center: oc) hint(r: 10)
}}
m := point hint((15, 5, 4))
nl := line(p, m)
t := axis hint(dir: (1, 2, 3))
f1 tangent(at: p) nl
f1 perpendicular(at: p) t
f1 tangent(at: p) std.front
f1 curvature(at: p) osc
");
    let kinds = |e: &gcs_core::program::Elaborated| {
        let mut k: Vec<String> = e.sketch.user_constraints().iter().map(|c| c.kind.name().to_string()).collect();
        k.sort();
        k
    };
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let first = gcs_core::program::elaborate(&prog);
    assert!(first.ok(), "{:?}", first.errors().map(|d| &d.message).collect::<Vec<_>>());
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    for want in ["p coincident f1", "f1 tangent(at: p) nl", "f1 perpendicular(at: p) t",
                 "f1 tangent(at: p) std.front", "f1 curvature(at: p) osc"] {
        assert!(printed.contains(want), "{want}\n{printed}");
    }
    let again = build(&printed);
    assert_eq!(kinds(&again), kinds(&first));
    assert!(kinds(&first).iter().any(|x| x == "PointOnEnvelope"));
    // the frame at the point, each word a rate of the contact (#365)
    use gcs_core::contact_rate::Word;
    for w in [Word::InPlane, Word::Normal, Word::Plane, Word::Curvature] {
        assert!(first.sketch.constraints.iter().any(|c| c.kind == CKind::Rate && c.rate_word() == w),
                "{w:?}");
    }
}

/// `spiral_bevel/mean_point.sv` (#283 step 2): the face mill's lean solves from the pressure angle
/// stated at the mean point, on the pinion's flank — the envelope of the gear's under the mesh.
/// Its axis leans `tilt` from the pitch plane's normal in the plane of that normal and the teeth's
/// normal, so the blades lean the pressure angle less the tilt, at each design; the contact is at
/// the roll the pair is drawn at, M on the gear's cone itself. The pinion flank's profile circle
/// is its section's: the circle through three points of that section a hair apart, each found by
/// the envelope's equation and the section's plane — the moving source's reading, not its
/// curvature.
#[test]
fn the_face_mills_lean_solves_from_the_pressure_angle_at_the_mean_point() {
    let text = include_str!("../../examples/spiral_bevel/mean_point.sv");
    for (pressure, tilt) in [(20.0, 5.0), (25.0, 3.0), (18.0, 8.0)] {
        let src = fixtures::gear::configure(text, &[("pressure", pressure), ("tilt", tilt)]);
        let e = elaborated(&src);
        let sk = &e.sketch;
        let half = sk.params[sk.free_vars["flank.half"] as usize].value;
        assert!((half - (pressure - tilt)).abs() < 1e-9, "{pressure} {tilt}: {half}");
        // the contact's place: at the roll the pair is drawn at, where M is on the gear's cone
        let c = sk.constraints.iter().find(|c| c.kind == CKind::PointOnEnvelope).unwrap();
        let place = [2, 3, 4].map(|s| sk.params[c.args[s].param() as usize].value);
        assert!(place[2].abs() < 1e-9, "the roll {}", place[2]);
        let at = |n: &str| sk.world_point(e.map.ent_named(n).unwrap().i());
        let (m, apex, centre) = (at("M"), at("apex"), at("C"));
        let (axis, to) = (sub(centre, apex), sub(m, apex));
        let seen = dot(axis, to) / (norm(axis) * norm(to));
        assert!((seen.acos().to_degrees() - half).abs() < 1e-9, "M is on the cone");

        // the profile: three points of the pinion flank's section by `section`
        let i = envelope(sk, "pinion_flank");
        let parts = Parts::of(sk, i).unwrap();
        let (k, values) = (parts.consts(Contact::Point), parts.values(sk));
        let plane = e.map.ent_named("section").unwrap().i();
        let basis = sk.basis(plane);
        let normal = cross(basis.u, basis.v);
        let unit = |x: [f64; 3]| x.map(|c| c / norm(x));
        let n0 = held::read(&k, &values, place).unwrap().normal;
        let along = unit(cross(unit(normal), n0));
        // where the section passes `s` along the profile's tangent from M
        let section = |s: f64| -> [f64; 3] {
            let rows = |x: [f64; 3]| -> [f64; 3] {
                let r = held::read(&k, &values, x).unwrap();
                let d = sub(r.position, m);
                [dot(d, normal), dot(d, along) - s, r.normal_velocity]
            };
            let mut x = place;
            for _ in 0..50 {
                let r = rows(x);
                if r.iter().all(|v| v.abs() < 1e-13) {
                    break;
                }
                let mut j = [[0.0; 3]; 3];
                for q in 0..3 {
                    let mut y = x;
                    y[q] += 1e-7;
                    let ry = rows(y);
                    for row in 0..3 {
                        j[row][q] = (ry[row] - r[row]) / 1e-7;
                    }
                }
                // Cramer's rule on the 3 × 3 step
                let det = |a: [[f64; 3]; 3]| a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                    - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                    + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
                let d = det(j);
                for q in 0..3 {
                    let mut a = j;
                    for row in 0..3 {
                        a[row][q] = r[row];
                    }
                    x[q] -= det(a) / d;
                }
            }
            held::read(&k, &values, x).unwrap().position
        };
        let h = 1e-3;
        let (a, b) = (section(-h), section(h));
        // the circle through a, M and b, in the plane they span
        let (ab, am) = (sub(b, a), sub(m, a));
        let w = cross(ab, am);
        let num = cross(w, ab).map(|x| x * dot(am, am));
        let num2 = cross(am, w).map(|x| x * dot(ab, ab));
        let mid = [0, 1, 2].map(|q| a[q] + (num[q] + num2[q]) / (2.0 * dot(w, w)));
        let fitted = norm(sub(mid, m));
        let p = e.map.ent_named("profile").unwrap();
        let (circle_centre, radius) = (sk.world_point(sk.round_center(p)), sk.params[sk.round_radius(p)].value);
        assert!((radius - fitted).abs() < 1e-4 * fitted, "{pressure} {tilt}: {radius} {fitted}");
        assert!(norm(sub(circle_centre, mid)) < 1e-4 * fitted, "{pressure} {tilt}");
    }
}

/// What touches an envelope may be written first; a relation at a point no contact holds on it is
/// refused naming the contact it wants; a set the solve cannot read as a cone or a cylinder is
/// refused where it is applied, and a contact with an envelope whose motion turns about a point of
/// a view says why it cannot hold to it.
#[test]
fn envelope_contacts_read_either_way_round_and_refuse_what_they_cannot_hold() {
    let src = format!("{ANNULUS}r := point hint((0, 35, 0))\nr coincident flank\nalong tangent(at: r) flank\n\
        k curvature(at: r) flank\n");
    let e = elaborated(&src);
    // each the envelope's frame at the point, read by a rate of its contact (#365)
    use gcs_core::contact_rate::Word;
    let words: Vec<Word> = e.sketch.constraints.iter().filter(|c| c.kind == CKind::Rate)
        .map(|c| c.rate_word()).collect();
    assert_eq!(words.iter().filter(|&&w| w == Word::InPlane).count(), 2);
    assert_eq!(words.iter().filter(|&&w| w == Word::Curvature).count(), 2);
    let refused = |src: &str, want: &str| {
        let (prog, errs) = crate::common::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(e.errors().any(|d| d.message.contains(want)),
                "{want}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    };
    refused(&ANNULUS.replace("p coincident flank\n", ""), "the curvature at `p` is taken where a contact holds");
    refused(&ANNULUS.replace("p coincident flank\n", ""), "the normal at `p` is taken where a contact holds");
    refused(&ANNULUS.replace("cyl := std.Cylinder(cax, r: 5mm)", "cyl := std.Sphere(c1, r: 5mm)"),
            "the solve reads the envelope of a cone's or a cylinder's surface");
    let planar = ANNULUS.replace("std.Turn(turn, about: ax, ratio: 1)",
        "in std.top {\n  tc := point hint((0, 0))\n}\nfix((0, 0)) tc\n\
         std.Spin(turn, about: tc, on: std.top, ratio: 1)");
    refused(&planar, "turns about a point of a view");
}

/// `ANNULUS`'s cylinder with its axis placed by relations alone, each end a point at a stated
/// place along a rail, and hinted elsewhere (`HINT`, #304).
const ANNULUS_PLACED: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((0, 0, 10))
fix((0, 0, 0)) o
fix((0, 0, 10)) z
ax := line(o, z)
a0 := point hint((0, 0, -5))
a1 := point hint((60, 0, -5))
b0 := point hint((0, 0, 5))
b1 := point hint((60, 0, 5))
fix((0, 0, -5)) a0
fix((60, 0, -5)) a1
fix((0, 0, 5)) b0
fix((60, 0, 5)) b1
low := line(a0, a1)
high := line(b0, b1)
c1 := point hint((HINT, -5))
c2 := point hint((HINT, 5))
c1 coincident(t == 0.5) low
c2 coincident(t == 0.5) high
cax := line(c1, c2)
cyl := std.Cylinder(cax, r: 5mm)
turn := frame
std.Turn(turn, about: ax, ratio: 1)
flank := envelope(cyl, under: turn, from: -30deg, to: 30deg)
in std.top {
  p := point hint((34, 9))
}
fix(y == 10) p
p coincident flank
";

/// **A contact starts where the solve places its source** (#304): a point held to an envelope is
/// seeded as it is stated, where the source stands at its hints, and is sought again where its
/// block comes up, after the blocks that place the cylinder.  Seeded only as stated, it lands on
/// the far side of the axis, beyond the roll's bounds (the cylinder hinted across the axis), or on
/// the inner sheet (a quarter turn round); sought again, on the outer sheet where it is drawn, at
/// a roll within the bounds — as `ANNULUS`'s does, whose cylinder is hinted where it is placed.
#[test]
fn an_envelope_contact_starts_where_the_relations_place_its_source() {
    for hint in ["-60, 0", "0, -60"] {
        let e = elaborated(&ANNULUS_PLACED.replace("HINT", hint));
        let sk = &e.sketch;
        let at = |n: &str| sk.world_point(e.map.ent_named(n).unwrap().i());
        let c1 = at("c1");
        assert!(near(c1, [30.0, 0.0, -5.0], 1e-12), "{hint}: {c1:?}");
        let p = at("p");
        let want = [(35f64 * 35.0 - 100.0).sqrt(), 10.0, 0.0];
        assert!(near(p, want, 1e-12), "{hint}: {p:?}");
        let c = sk.constraints.iter().find(|c| c.kind == CKind::PointOnEnvelope).unwrap();
        let roll = sk.params[c.args[4].param() as usize].value;
        assert!(roll.abs() <= 30f64.to_radians(), "{hint}: {roll}");
    }
}
