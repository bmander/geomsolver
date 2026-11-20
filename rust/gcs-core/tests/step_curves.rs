//! **A curve the STEP file fits is held to the file's tolerance, in as few poles as that allows**
//! (docs/contract/brep.md, #961 item 4): a traced intersection's cubic and each of its pcurves'
//! images stand within `step::curve_fit(tol)` of the exact curve — on both of its surfaces —
//! wherever they are read, their ends on the edge's vertices; a coarser tolerance writes fewer
//! poles and a finer one comes nearer.  The file is read back here, entity by entity, and every
//! written spline sampled far more finely than the fit measured it.  Held for crossing cylinders
//! (the issue's tube and its cross-hole, other places and radii of the hole, a hole leaning across
//! the tube), a ball drilled off its centre, a cone drilled across and a torus drilled through.
use gcs_core::brep::build::{cone, torus};
use gcs_core::brep::boolean::{boolean, Op};
use gcs_core::brep::geom::{Curve, Surface};
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::step::{curve_fit, write};
use gcs_core::brep::topo::{Brep, EdgeCurve};
use gcs_core::space::{distance, norm};
use std::collections::BTreeMap;

use crate::common::{ent, text::solved};

type V = [f64; 3];

/// The data section's simple entities by id: each one's name and its parameters' text.
fn entities(text: &str) -> BTreeMap<usize, (String, String)> {
    let mut out = BTreeMap::new();
    for line in text.lines() {
        let Some((id, rest)) = line.strip_prefix('#').and_then(|l| l.split_once(" = ")) else { continue };
        let Some(open) = rest.find('(') else { continue };
        if rest.starts_with('(') { continue }
        let body = rest.trim_end_matches(';');
        out.insert(id.parse().unwrap(), (body[..open].to_string(), body[open + 1..body.len() - 1].to_string()));
    }
    out
}

/// A parameter list's top-level items.
fn items(s: &str) -> Vec<&str> {
    let (mut out, mut depth, mut from, mut quoted) = (Vec::new(), 0, 0, false);
    for (k, c) in s.char_indices() {
        match c {
            '\'' => quoted = !quoted,
            '(' if !quoted => depth += 1,
            ')' if !quoted => depth -= 1,
            ',' if !quoted && depth == 0 => { out.push(s[from..k].trim()); from = k + 1; }
            _ => {}
        }
    }
    out.push(s[from..].trim());
    out
}

/// `(a,b,…)`'s items.
fn list(s: &str) -> Vec<&str> { items(&s[1..s.len() - 1]) }
fn reference(s: &str) -> usize { s.trim_start_matches('#').parse().unwrap() }
fn real(s: &str) -> f64 { s.parse().unwrap() }

/// The file read: its entities, which face each surface is, and how many traced edges it holds.
struct File { e: BTreeMap<usize, (String, String)>, face_of: BTreeMap<usize, usize> }

impl File {
    fn read(text: &str) -> File {
        let e = entities(text);
        // the faces are written in the solid's order
        let face_of = e.values().filter(|(n, _)| n == "ADVANCED_FACE").enumerate()
            .map(|(k, (_, p))| (reference(items(p)[2]), k)).collect();
        File { e, face_of }
    }
    fn get(&self, id: usize) -> (&str, Vec<&str>) { let (n, p) = &self.e[&id]; (n, items(p)) }
    fn point(&self, id: usize) -> V {
        let (n, p) = self.get(id);
        assert_eq!(n, "CARTESIAN_POINT");
        let c: Vec<f64> = list(p[1]).into_iter().map(real).collect();
        [c[0], c[1], c.get(2).copied().unwrap_or(0.)]
    }
    /// The B-spline `id` is, where it is a plain one.
    fn spline(&self, id: usize) -> Option<BSpline> {
        let (n, p) = self.get(id);
        if n != "B_SPLINE_CURVE_WITH_KNOTS" { return None }
        let degree: usize = p[1].parse().unwrap();
        let poles: Vec<V> = list(p[2]).into_iter().map(|r| self.point(reference(r))).collect();
        let mults: Vec<usize> = list(p[6]).into_iter().map(|m| m.parse().unwrap()).collect();
        let knots: Vec<f64> = list(p[7]).into_iter().zip(&mults).flat_map(|(k, &m)| std::iter::repeat_n(real(k), m)).collect();
        Some(BSpline::new(degree, knots, poles).unwrap())
    }
    /// Every edge as written, in the solid's order: its curve's entity, its vertices' points and
    /// its pcurves (each the face it is on and its curve's entity).
    fn edges(&self) -> Vec<(usize, [V; 2], Vec<(usize, usize)>)> {
        self.e.iter().filter(|(_, (n, _))| n == "EDGE_CURVE").map(|(_, (_, p))| {
            let p = items(p);
            let vertex = |s: &str| { let (_, v) = self.get(reference(s)); self.point(reference(v[1])) };
            let (_, g) = self.get(reference(p[3]));
            let pcurves = list(g[2]).into_iter().map(|pc| {
                let (_, pc) = self.get(reference(pc));
                let (_, rep) = self.get(reference(pc[2]));
                (self.face_of[&reference(pc[1])], reference(list(rep[1])[0]))
            }).collect();
            (reference(g[1]), [vertex(p[1]), vertex(p[2])], pcurves)
        }).collect()
    }
}

/// What the file says of a solid's traced edges: the poles of their curves and of their pcurves,
/// and the farthest each stands, read at 64 points a knot span — the cubic from the exact curve
/// (and from both its surfaces), a pcurve's image from the exact curve and from the cubic at the
/// same parameter, and an end of either from the edge's vertex.
#[derive(Debug, Default)]
struct Read { traced: usize, marched: usize, poles: usize, pcurve_poles: usize, curve: f64, surfaces: f64,
    pcurve: f64, same_parameter: f64, ends: f64 }

/// The span's parameters a spline is read at: 64 a knot span.
fn dense(s: &BSpline) -> Vec<f64> {
    let mut k: Vec<f64> = s.knots.clone();
    k.dedup();
    k.windows(2).flat_map(|w| (0..64).map(move |i| w[0] + (w[1] - w[0]) * i as f64 / 64.)).chain(k.last().copied()).collect()
}

/// A face's surface at a pcurve's point, as the file's surface reads it (a cone opening against
/// its axis is written about the reversed one, its parameters both negated).
fn on(s: &Surface, q: V) -> V {
    match s { Surface::Cone(_, _, a) if *a < 0. => s.point([-q[0], -q[1]]), _ => s.point([q[0], q[1]]) }
}

fn read(b: &Brep, text: &str) -> Read {
    let f = File::read(text);
    let edges: Vec<_> = b.edges.iter().filter(|e| matches!(e.curve, EdgeCurve::Curve(_))).collect();
    let written = f.edges();
    assert_eq!(edges.len(), written.len(), "every edge with a curve is written");
    let mut r = Read::default();
    for (e, (curve, ends, pcurves)) in edges.iter().zip(&written) {
        // the vertices are the solid's
        for k in 0..2 { assert!(distance(ends[k], b.vertices[e.v[k] as usize].p) <= 1e-9, "a vertex moved"); }
        let EdgeCurve::Curve(Curve::Traced(t)) = &e.curve else { continue };
        r.traced += 1;
        r.marched += t.pts.len();
        let s = f.spline(*curve).expect("a traced edge is written as a spline");
        assert_eq!(s.degree, 3, "a traced edge is written as a cubic");
        r.poles += s.poles.len();
        let ts = dense(&s);
        for (end, t) in [(ends[0], ts[0]), (ends[1], ts[ts.len() - 1])] { r.ends = r.ends.max(distance(s.point(t), end)); }
        for &x in &ts {
            let q = s.point(x);
            r.curve = r.curve.max(distance(q, t.project(q)));
            r.surfaces = r.surfaces.max(t.a.implicit(q).abs()).max(t.b.implicit(q).abs());
        }
        for &(fi, pc) in pcurves {
            let Some(p) = f.spline(pc) else { continue };
            r.pcurve_poles += p.poles.len();
            let surface = &b.faces[fi].surface;
            let pts = dense(&p);
            assert!(pts[0] == ts[0] && pts[pts.len() - 1] == ts[ts.len() - 1], "a pcurve runs over another parameter than its edge's cubic");
            for (end, x) in [(ends[0], pts[0]), (ends[1], pts[pts.len() - 1])] { r.ends = r.ends.max(distance(on(surface, p.point(x)), end)); }
            for &x in &pts {
                let q = on(surface, p.point(x));
                r.pcurve = r.pcurve.max(distance(q, t.project(q)));
                r.same_parameter = r.same_parameter.max(distance(q, s.point(x)));
            }
        }
    }
    r
}

/// The solid written at `tol`, read back and held to `curve_fit(tol)`: every traced cubic and
/// pcurve within it of the exact curve, the cubic within it of both surfaces, the two within twice
/// it (the file's stated uncertainty) of one another at every parameter, the ends on the vertices.
fn held(what: &str, b: &Brep, tol: f64) -> (Read, String) {
    let text = write(b, what, tol).unwrap_or_else(|e| panic!("{what}: {e}"));
    let r = read(b, &text);
    let bar = curve_fit(tol);
    eprintln!("{what} at {tol:e}: {} traced edges marched in {} points, written in {} poles and {} pcurve poles, {} bytes; \
        within {:.2e} of the curve ({:.2e} of its surfaces), pcurves {:.2e} ({:.2e} of the cubic), ends {:.1e} (bar {bar:.1e})",
        r.traced, r.marched, r.poles, r.pcurve_poles, text.len(), r.curve, r.surfaces, r.pcurve, r.same_parameter, r.ends);
    assert!(r.traced > 0, "{what}: no traced edge");
    assert!(r.curve <= bar && r.surfaces <= bar, "{what} at {tol:e}: the cubic stands {:e} off its curve, past {bar:e}", r.curve.max(r.surfaces));
    assert!(r.pcurve <= bar, "{what} at {tol:e}: a pcurve stands {:e} off the curve, past {bar:e}", r.pcurve);
    assert!(r.same_parameter <= 2. * bar, "{what} at {tol:e}: a pcurve stands {:e} off the cubic, past {:e}", r.same_parameter, 2. * bar);
    assert!(r.ends <= bar, "{what} at {tol:e}: an end stands {:e} off its vertex", r.ends);
    (r, text)
}

/// The solid held at a coarse, the default and a fine tolerance: each within its own bar, a finer
/// one nearer and in more poles, a coarser in fewer, and none much finer than its bar asks — the
/// file follows the tolerance, not the march.
fn follows(what: &str, b: &Brep) {
    let tolerances = [5e-4, 1e-4, 5e-6];
    let reads: Vec<(Read, String)> = tolerances.iter().map(|&tol| held(what, b, tol)).collect();
    for w in reads.windows(2) {
        let ((coarse, a), (fine, z)) = (&w[0], &w[1]);
        assert!(a != z, "{what}: two tolerances write one file");
        assert!(fine.poles > coarse.poles && fine.pcurve_poles > coarse.pcurve_poles, "{what}: a finer tolerance in no more poles");
        assert!(fine.curve < coarse.curve, "{what}: a finer tolerance no nearer ({:e} against {:e})", fine.curve, coarse.curve);
    }
    // as few poles as the bar allows: a cubic's miss goes as the fourth power of its pieces'
    // length, so pieces half as long would miss by a sixteenth — the fit is not that fine
    for ((read, _), &tol) in reads.iter().zip(&tolerances) {
        let miss = read.curve.max(read.pcurve);
        assert!(miss >= curve_fit(tol) / 16., "{what} at {tol:e}: within {miss:e}, finer than its bar {:e} asks", curve_fit(tol));
    }
}

fn part(src: &str, name: &str) -> Brep {
    let e = solved(src);
    let s = ent(&e, name).i();
    e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}")).brep.clone()
}

/// The issue's tube, its wall drilled across by a hole of radius `r` at `(x, z)` in the front view.
fn cross_hole(x: f64, z: f64, r: f64) -> Brep {
    part(&format!("unit mm\nuse std\nuse std.solids\nuse std.solids (cut)\n\
        in std.top {{\n  o := point\n  fix((0, 0)) o\n  outer := circle(center: o) hint(r: 100)\n  radius(100mm) outer\n\
        bore := circle(center: o) hint(r: 85)\n  radius(85mm) bore\n}}\n\
        tube := solid(std.solids.Prism(face(outer, holes: bore), from: 0mm, to: 60mm))\n\
        in std.front {{\n  h := point\n  fix(({x}, {z})) h\n  hc := circle(center: h) hint(r: {r})\n  radius({r}mm) hc\n}}\n\
        drill := solid(std.solids.Through(face(hc)))\npart := solid(tube)\ndrill cut part\n"), "part")
}

/// The issue's repro (#961 item 4): two cylinder–cylinder curves of some 3,370 poles each, the
/// same at every tolerance, now as few as the tolerance allows.
#[test]
fn a_cross_holes_curves_follow_the_tolerance() {
    follows("the cross-hole", &cross_hole(30., 30., 5.));
}

/// The hole elsewhere, wider, and nearer the tube's edge: no number of the repro's is read.
#[test]
fn other_cross_holes_follow_it_too() {
    for (x, z, r) in [(0., 20., 12.), (-55., 45., 3.), (70., 30., 8.)] {
        follows(&format!("a hole of r {r} at ({x}, {z})"), &cross_hole(x, z, r));
    }
}

/// A ball drilled off its centre: a sphere's meeting with a cylinder.
#[test]
fn a_ball_drilled_off_its_centre_follows_it() {
    let b = part("unit mm\nuse std\nuse std.solids\nuse std.solids (cut)\nuse std.sets\nuse std.place (right_of)\n\
        in std.front {\n  ax := point hint((5, 0))\n  ax right_of(d: 5mm) std.origin\n\
        rim := circle(center: ax) hint(r: 6)\n  radius(6mm) rim\n}\n\
        drill := solid(std.solids.Through(face(rim)))\n\
        part := solid(std.sets.Ball(std.origin, r: 20mm))\ndrill cut part\n", "part");
    follows("the drilled ball", &b);
}

/// A frustum drilled across, leaning: a cone's meeting with a cylinder; and a torus drilled
/// through its tube: a torus's.
#[test]
fn a_cone_and_a_torus_drilled_follow_it() {
    let frustum = cone([0., 0., 0.], [0., 0., 1.], 20., 8., 30.).unwrap();
    let rod = solid_rod([-40., 3., 10.], [1., 0., 0.2], 4., 80.);
    let b = boolean(&frustum, &rod, Op::Cut, 1e-9).unwrap();
    follows("the drilled frustum", &b);
    let ring = torus([0., 0., 0.], [0., 0., 1.], 20., 6.).unwrap();
    let rod = solid_rod([17., 1., -20.], [0.1, 0., 1.], 2.5, 40.);
    let b = boolean(&ring, &rod, Op::Cut, 1e-9).unwrap();
    follows("the drilled torus", &b);
}

/// A closed rod of radius `r` from `o` along `n` for `depth`.
fn solid_rod(o: V, n: V, r: f64, depth: f64) -> Brep {
    use gcs_core::brep::build::{disc, prism};
    let l = norm(n);
    let n = n.map(|x| x / l);
    prism(&disc(o, n, r), 0., depth).unwrap()
}

/// Where a meeting has a closed form the file carries, it is written so and fitted nowhere: two
/// cylinders of one radius crossing meet in ellipses, a plane and a cylinder in an ellipse.
#[test]
fn a_meeting_in_closed_form_is_written_exactly() {
    let a = solid_rod([-30., 0., 0.], [1., 0., 0.], 6., 60.);
    let b = solid_rod([0., -30., 0.], [0., 1., 0.], 6., 60.);
    let cross = boolean(&a, &b, Op::Union, 1e-9).unwrap();
    let leaning = solid_rod([0., 0., -10.], [0.3, 0., 1.], 4., 30.);
    let plate = gcs_core::brep::build::block([-20., -20., 0.], [20., 20., 5.]).unwrap();
    let drilled = boolean(&plate, &leaning, Op::Cut, 1e-9).unwrap();
    for (what, b) in [("the crossing rods", &cross), ("the leaning hole", &drilled)] {
        assert!(b.edges.iter().all(|e| !matches!(e.curve, EdgeCurve::Curve(Curve::Traced(_)))), "{what}: a meeting traced");
        let text = write(b, what, 1e-4).unwrap();
        assert!(text.contains("ELLIPSE("), "{what}: no ellipse written");
        let f = File::read(&text);
        assert!(f.edges().iter().all(|(c, ..)| f.get(*c).0 != "B_SPLINE_CURVE_WITH_KNOTS"), "{what}: a curve fitted");
    }
}
