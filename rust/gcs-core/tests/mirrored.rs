//! **A mirrored placement** (#578): `left := Cover(…) mirrored(front)` is the instance's binding
//! reflected in its own `front` plane.  Its body is solved where it stands unmirrored — its rows,
//! its freedoms and its diagnosis those of the same instance unmirrored — and what it publishes
//! is the image: every entity its body made, the same numbers (some negated) in the image of what
//! it is drawn in, and every solid its evaluation reflected.  Held here by the properties, not
//! only by counts: each image vertex is its preimage's reflection, the volume is equal, the
//! reflected B-rep is a valid one, an image's planar numbers are its preimage's, a left and a
//! right of one binding build once, a mirrored instance placed by mates is held wholly, and what
//! a reflection cannot be made of is refused.

use std::cell::RefCell;
use std::rc::Rc;

use crate::common::{read, refused};
use gcs_core::brep::props;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::{EntRef, Sketch};
use gcs_core::progress::{self, At, Kind, Listener, Work};
use gcs_core::solid::ApproximationPolicy as Policy;

/// A plate in the front plane, off its frame's origin and not square to it, so a reflection in
/// any plane moves it: `x` from 1 to `w`, `z` from 2 to `d` with a slanted top, `h` deep.
const PLATE: &str = "
component Plate(w: Length, d: Length, h: Length) {
  in std.front {
    a := point hint((1, 2))
    b := point hint((w, 2))
    c := point hint((w, d))
    e := point hint((1, d + 3mm))
    a offset(dx: 1mm, dy: 2mm) std.origin
    b offset(dx: w, dy: 2mm) std.origin
    c offset(dx: w, dy: d) std.origin
    e offset(dx: 1mm, dy: d + 3mm) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    rim := circle(center: a) hint(r: 0.5mm)
    radius(0.5mm) rim
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: h)
  }
}
";

/// A turned part in the side plane: a profile with slants (cones), a step (planes and a
/// cylinder) and a rounded corner, a quarter of a circle about `k` (a torus), turned about the
/// side plane's `u` — the world's `y` — off the mid-plane, so a mirror in the front moves it.
const TURNED: &str = "
component Turned(s0: Length) {
  in std.side {
    construction ax := line(std.side.origin, hint((10, 0)))
    horizontal ax
    length(10mm) ax
    p1 := point hint((s0, 2))
    p2 := point hint((s0 + 6mm, 2))
    p3 := point hint((s0 + 6mm, 5))
    p4 := point hint((s0 + 4mm, 6))
    k := point hint((s0 + 3mm, 6))
    p5 := point hint((s0 + 3mm, 7))
    p6 := point hint((s0, 9))
    p1 offset(dx: s0, dy: 2mm) std.side.origin
    p2 offset(dx: s0 + 6mm, dy: 2mm) std.side.origin
    p3 offset(dx: s0 + 6mm, dy: 5mm) std.side.origin
    p4 offset(dx: s0 + 4mm, dy: 6mm) std.side.origin
    k offset(dx: s0 + 3mm, dy: 6mm) std.side.origin
    p5 distance(s0 + 3mm, along: u) std.side
    p6 offset(dx: s0, dy: 9mm) std.side.origin
    profile := (l1 := line(p1, p2)) -> (l2 := line(p2, p3)) -> (l3 := line(p3, p4)) ->
               (round := arc(center: k, start: p4, end: p5)) -> (l5 := line(p5, p6)) ->
               (l6 := line(p6, p1)) -> close
    body := solid(profile, about: ax)
  }
}
";

fn doc(defs: &str, body: &str) -> String {
    format!("unit mm\nuse std (offset, horizontal)\n{defs}\n{body}")
}

fn solved(src: &str) -> (gcs_core::program::Elaborated, Sketch) {
    let e = read(src);
    let sk = crate::common::solved(&e);
    (e, sk)
}

fn solid(e: &gcs_core::program::Elaborated, name: &str) -> usize {
    crate::common::at_path(e, name).i()
}

/// The solids built while `work` runs, by index.
#[derive(Default)]
struct Builds(RefCell<Vec<usize>>);

impl Listener for Builds {
    fn begin(&self, work: &Work) {
        if let (Kind::Build, Some(At::Solid(i))) = (work.kind, work.at) {
            self.0.borrow_mut().push(i);
        }
    }
    fn end(&self, _: &Work, _: f64, _: f64) {}
}

fn builds<T>(work: impl FnOnce() -> T) -> (T, Vec<usize>) {
    let heard = Rc::new(Builds::default());
    let out = progress::listen(heard.clone(), work);
    let built = heard.0.borrow().clone();
    (out, built)
}

/// An exact solid's vertices in the world, in the drawing's units, each rounded to a nanometre.
fn corners(x: &gcs_core::solid::Exact) -> Vec<[i64; 3]> {
    let mut c: Vec<[i64; 3]> = x.brep.vertices.iter()
        .map(|v| std::array::from_fn(|k| ((x.origin[k] + v.p[k] / x.mm) * 1e6).round() as i64))
        .collect();
    c.sort();
    c
}

/// `corners`, each reflected by negating coordinate `k`.
fn reflected(mut c: Vec<[i64; 3]>, k: usize) -> Vec<[i64; 3]> {
    for p in &mut c {
        p[k] = -p[k];
    }
    c.sort();
    c
}

/// The image of what the source names `name` in a mirrored instance: what a reference from
/// outside it reads.
fn image(e: &gcs_core::program::Elaborated, name: &str) -> EntRef {
    e.map.ent_named(&format!("{name}#image")).unwrap_or_else(|| panic!("no image of `{name}`"))
}

#[test]
fn every_image_vertex_is_its_preimages_reflection_and_the_volume_is_equal() {
    for (word, k) in [("side", 0), ("front", 1), ("top", 2)] {
        let src = doc(PLATE, &format!("right := Plate(w: 30mm, d: 20mm, h: 5mm)\n\
            left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored({word})\n"));
        let (e, sk) = solved(&src);
        let (r, l) = (solid(&e, "right.body"), solid(&e, "left.body"));
        assert_eq!(sk.solids[l].binding.as_ref().unwrap().mirror, Some(k as u8));
        let (right, left) = (sk.exact_solid(r).unwrap(), sk.exact_solid(l).unwrap());
        assert_eq!(corners(&left), reflected(corners(&right), k), "mirrored({word})");
        let (vr, vl) = (props::volume(&right.brep), props::volume(&left.brep));
        assert!(vl > 0.0 && (vl - vr).abs() <= 1e-9 * vr, "mirrored({word}): {vl} against {vr}");
        left.brep.check(1e-9 * left.brep.size()).unwrap();
        // the faceted evaluation likewise: the same volume, its world mesh the reflection, each
        // triangle wound outward (the mesh's own signed volume is the solid's)
        let (fr, fl) = (sk.evaluated_solid(r, Policy::Report).unwrap(),
                        sk.evaluated_solid(l, Policy::Report).unwrap());
        assert!(fl.improper() && !fr.improper());
        assert!((fl.volume() - fr.volume()).abs() <= 1e-9 * fr.volume());
        let signed = |m: &gcs_core::mesh::Mesh| m.positions.chunks(9).map(|t| {
            let (a, b, c) = ([t[0], t[1], t[2]], [t[3], t[4], t[5]], [t[6], t[7], t[8]]);
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        }).sum::<f64>();
        let (mr, ml) = (signed(&fr.world_mesh()), signed(&fl.world_mesh()));
        assert!((ml - mr).abs() <= 1e-6 * mr.abs() && ml > 0.0,
                "mirrored({word}): {ml} against {mr}");
        // its faces go by the image's names, which are the preimage's
        assert!(left.brep.faces.iter().all(|f| f.name.starts_with("left.")));
    }
}

#[test]
fn a_reflected_brep_holds_over_planes_cylinders_cones_and_a_torus() {
    let src = doc(TURNED, "right := Turned(s0: 4mm)\nleft := Turned(s0: 4mm) mirrored(front)\n");
    let (e, sk) = solved(&src);
    let (r, l) = (solid(&e, "right.body"), solid(&e, "left.body"));
    let (right, left) = (sk.exact_solid(r).unwrap(), sk.exact_solid(l).unwrap());
    let kinds = |b: &gcs_core::brep::topo::Brep| {
        let mut k: Vec<String> = b.faces.iter().map(|f| format!("{:?}", f.surface)
            .split('(').next().unwrap().to_string()).collect();
        k.sort();
        k.dedup();
        k
    };
    assert_eq!(kinds(&left.brep), ["Cone", "Cylinder", "Plane", "Torus"]);
    assert_eq!(kinds(&left.brep), kinds(&right.brep));
    left.brep.check(1e-9 * left.brep.size()).unwrap();
    // every placement right-handed again
    for f in &left.brep.faces {
        let frame = f.surface.frame();
        assert!(gcs_core::space::dot(gcs_core::space::cross(frame.x, frame.y), frame.z) > 0.0);
    }
    assert_eq!(corners(&left), reflected(corners(&right), 1));
    let (vr, vl) = (props::volume(&right.brep), props::volume(&left.brep));
    assert!(vl > 0.0 && (vl - vr).abs() <= 1e-9 * vr, "{vl} against {vr}");
    // and a point on each image face is its preimage face's reflected
    for (a, b) in right.brep.faces.iter().zip(&left.brep.faces) {
        assert_eq!(a.name.replacen("right.", "left.", 1), b.name);
    }
}

/// Every face of `b` reflected as `rb` by the reflection in the plane `y = 0`, held to what a
/// reflection does: a point of the face is a point of its image reflected, the material normal
/// there the original's reflected, each use's pcurve still on its edge, and each loop walked the
/// way its face's sense says (counter-clockwise in its parameters for an unreversed face's outer
/// loop) — read off the loops alone, as `props` reads a volume.
fn reflected_faces_hold(b: &gcs_core::brep::topo::Brep, rb: &gcs_core::brep::topo::Brep) {
    use gcs_core::space::{dot, norm};
    let flip = |p: [f64; 3]| [p[0], -p[1], p[2]];
    let material = |f: &gcs_core::brep::topo::Face, uv| {
        let n = f.surface.normal(uv).unwrap();
        if f.reversed { n.map(|x| -x) } else { n }
    };
    // a loop's signed area in its face's parameters, its pcurves sampled
    use gcs_core::brep::topo::{Brep, Coedge, Face};
    let area = |b: &Brep, f: &Face, l: &[Coedge]| {
        let mut pts = Vec::new();
        for c in l {
            let e = &b.edges[c.edge as usize];
            let mut ts: Vec<f64> =
                (0..16).map(|k| e.t[0] + (e.t[1] - e.t[0]) * k as f64 / 16.0).collect();
            if c.reversed {
                ts = ts.into_iter().map(|t| e.t[0] + e.t[1] - t).collect();
            }
            pts.extend(ts.into_iter().map(|t| c.pcurve.at(t, e, &f.surface, &b.vertices)));
        }
        (0..pts.len()).map(|k| {
            let (a, c) = (pts[k], pts[(k + 1) % pts.len()]);
            a[0] * c[1] - c[0] * a[1]
        }).sum::<f64>() / 2.0
    };
    assert_eq!(b.faces.len(), rb.faces.len());
    for (f, g) in b.faces.iter().zip(&rb.faces) {
        let c = &f.loops[0][0];
        let e = &b.edges[c.edge as usize];
        let uv = c.pcurve.at(0.5 * (e.t[0] + e.t[1]), e, &f.surface, &b.vertices);
        // a step inside the face from its edge, along its material side
        let p = f.surface.point(uv);
        let q = flip(p);
        let image = g.surface.inverse(q);
        assert!(gcs_core::space::distance(g.surface.point(image), q) < 1e-9,
                "{}: off its image", f.name);
        let (n, m) = (flip(material(f, uv)), material(g, image));
        assert!(dot(n, m) > 1.0 - 1e-9 * norm(n) * norm(m),
                "{}: material normal {m:?} is not {n:?}", f.name);
        for (lf, lg) in f.loops.iter().zip(&g.loops) {
            let (a, ag) = (area(b, f, lf), area(rb, g, lg));
            assert_eq!((a > 0.0) != f.reversed, (ag > 0.0) != g.reversed,
                       "{}: the loop is walked against its sense", f.name);
            for cg in lg {
                let e = &rb.edges[cg.edge as usize];
                for t in [0.1, 0.5, 0.9] {
                    let x = e.t[0] + (e.t[1] - e.t[0]) * t;
                    let at = g.surface.point(cg.pcurve.at(x, e, &g.surface, &rb.vertices));
                    assert!(gcs_core::space::distance(at, e.point(x, &rb.vertices)) < 1e-9,
                            "{}: a pcurve off its edge", g.name);
                }
            }
        }
    }
}

#[test]
fn a_reflected_faces_material_side_pcurves_and_loops_follow() {
    use gcs_core::brep::geom::Rigid;
    let mirror = Rigid { r: [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]], t: [0.0; 3] };
    // a B-spline sheet, a saddle
    let pts: Vec<[f64; 3]> = (0..7).flat_map(|i| (0..6).map(move |j| {
        let (x, y) = (i as f64 * 1.5, 1.0 + j as f64 * 1.2);
        [x, y, 0.05 * (x * x - y * y)]
    })).collect();
    use gcs_core::brep::nurbs::{interpolate_net, Parametrization};
    let net = interpolate_net(&pts, 7, 6, Parametrization::ChordLength).unwrap();
    let sheet = gcs_core::brep::build::sheet(net).unwrap();
    reflected_faces_hold(&sheet, &sheet.reflected(&mirror).unwrap());
    // and every face of a turned part, its revolutions' `u` run the other way
    let src = doc(TURNED, "part := Turned(s0: 4mm)\n");
    let (e, sk) = solved(&src);
    let x = sk.exact_solid(solid(&e, "part.body")).unwrap();
    reflected_faces_hold(&x.brep, &x.brep.reflected(&mirror).unwrap());
}

#[test]
fn a_left_and_a_right_of_one_binding_build_once() {
    let src = doc(PLATE, "right := Plate(w: 30mm, d: 20mm, h: 5mm)\n\
                          left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(front)\n");
    let (e, sk) = solved(&src);
    let (r, l) = (solid(&e, "right.body"), solid(&e, "left.body"));
    let (a, b) = (sk.solids[r].binding.clone().unwrap(), sk.solids[l].binding.clone().unwrap());
    assert_eq!(a.key, b.key);
    assert_eq!((a.mirror, b.mirror), (None, Some(1)));
    let (_, built) = builds(|| {
        sk.evaluated_solid(r, Policy::Report).unwrap();
        sk.evaluated_solid(l, Policy::Report).unwrap();
        sk.exact_solid(l).unwrap();
    });
    assert_eq!(built, vec![r], "{built:?}");
    // the mirrored one first: still one build, its own preimage's
    let src = doc(PLATE, "left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(front)\n\
                          right := Plate(w: 30mm, d: 20mm, h: 5mm)\n");
    let (e, sk) = solved(&src);
    let (r, l) = (solid(&e, "right.body"), solid(&e, "left.body"));
    let ((right, left), built) =
        builds(|| (sk.exact_solid(r).unwrap(), sk.exact_solid(l).unwrap()));
    assert_eq!(built, vec![l], "{built:?}");
    assert_eq!(corners(&left), reflected(corners(&right), 1));
}

#[test]
fn an_images_planar_entities_hold_its_preimages_numbers() {
    let src = doc(PLATE, "left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(side)\n");
    let (e, sk) = solved(&src);
    for name in ["left.a", "left.b", "left.c", "left.e"] {
        let (q, p) = (crate::common::at_path(&e, name), image(&e, name));
        // the same two numbers, in the image of the plane
        assert_eq!(sk.point_xy(p.i()), sk.point_xy(q.i()), "{name}");
        let (wq, wp) = (sk.world_point(q.i()), sk.world_point(p.i()));
        assert!((wp[0] + wq[0]).abs() < 1e-12 && (wp[1] - wq[1]).abs() < 1e-12
                && (wp[2] - wq[2]).abs() < 1e-12, "{name}: {wp:?} is not {wq:?} reflected");
        // the image goes by its preimage's name, and is what is drawn
        assert_eq!(e.map.name_of(p), e.map.name_of(q));
        assert!(sk.drawn().contains(&p) && !sk.drawn().contains(&q));
    }
    let (rim, img) = (crate::common::at_path(&e, "left.rim"), image(&e, "left.rim"));
    assert_eq!(sk.params[sk.circles[img.i()].radius as usize].value,
               sk.params[sk.circles[rim.i()].radius as usize].value);
    // the image adds no freedom: the body's rows hold it, as they hold the preimage
    let mut sk = sk.clone();
    let mut plain = solved(&doc(PLATE, "left := Plate(w: 30mm, d: 20mm, h: 5mm)\n")).1;
    assert_eq!(diagnose(&mut sk, DiagnoseOptions::default()).dof,
               diagnose(&mut plain, DiagnoseOptions::default()).dof);
    assert_eq!(crate::common::dof(&mut sk), 0);
}

#[test]
fn a_relation_outside_reads_the_image() {
    // a point of the parent's held to a corner of the mirrored plate stands at the corner's image
    let src = doc(PLATE, "left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(side)\n\
                          in std.front {\n  q := point hint((-20, 5))\n  q coincident left.c\n}\n");
    let (e, sk) = solved(&src);
    let q = crate::common::at_path(&e, "q");
    let c = crate::common::at_path(&e, "left.c");
    let (wq, wc) = (sk.world_point(q.i()), sk.world_point(c.i()));
    assert!((wq[0] + wc[0]).abs() < 1e-9 && (wq[2] - wc[2]).abs() < 1e-9, "{wq:?} against {wc:?}");
    assert!(wq[0] < 0.0);
}

#[test]
fn a_mirrored_instance_placed_by_mates_is_held_wholly() {
    let src = doc(PLATE, "left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(front)\n\
        seat := point\nfix((10, 20, 30)) seat\n\
        bore := axis\nfix(dir == (1, 0, 0), origin == (0, 20, 30)) bore\n\
        left.std.origin coincident seat\nleft.std.x parallel bore\nleft.std.y parallel std.y\n");
    let (e, mut sk) = solved(&src);
    assert_eq!(diagnose(&mut sk, DiagnoseOptions::default()).dof, 0);
    // the image's x runs along the bore and its y up or down: `left.std.x × left.std.y =
    // −left.std.z` says it is mirrored, the frame it stands in right-handed still
    let axis = |name: &str| {
        let a = e.map.ent_named(&format!("left.#std.{name}")).unwrap();
        crate::common::unit(crate::common::dir(&sk, a.i()))
    };
    let (x, y, z) = (axis("x"), axis("y"), axis("z"));
    let xy = gcs_core::space::cross(x, y);
    let near = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| crate::common::near(a[k], b[k]));
    assert!(near(x, [1.0, 0.0, 0.0]) && near(y, [0.0, -1.0, 0.0]), "{x:?} {y:?}");
    assert!(near(xy, z.map(|c| -c)), "{xy:?} against {z:?}");
    // and its solid is the plate the mate put there, reflected in the frame's front: drawn there
    // it is swept along the front's normal, from y = 20 to 15, and its image from 20 to 25
    let l = solid(&e, "left.body");
    let x = sk.exact_solid(l).unwrap();
    let (lo, hi) = x.brep.bounds();
    let (y0, y1) = (lo[1] / x.mm + x.origin[1], hi[1] / x.mm + x.origin[1]);
    assert!((y0 - 20.0).abs() < 1e-6 && (y1 - 25.0).abs() < 1e-6, "{y0} … {y1}");
    assert!((lo[0] / x.mm + x.origin[0] - 11.0).abs() < 1e-6, "{lo:?}");
}

#[test]
fn what_a_reflection_cannot_be_made_of_is_refused() {
    // an entity it is given: reflected, it would no longer touch the body
    refused(&doc("component Lug(o: point) {\n  in std.front {\n    a := point\n    \
        a offset(dx: 1mm, dy: 1mm) o\n  }\n}\n", "in std.front {\n  p := point\n  \
        fix((0, 0)) p\n}\nleft := Lug(p) mirrored(front)\n"), "E112", "a closed one", "p");
    // in a ring, whose turn an image is not composed with yet
    refused(&doc(PLATE, "in std.front {\n  o := point\n  fix((50, 0)) o\n  ring 3 about o {\n    \
        m := Plate(w: 3mm, d: 2mm, h: 1mm) mirrored(front)\n  }\n}\n"), "E112", "in a `ring`",
        "mirrored(front)");
    // a body declaring a plane, whose image is no linear function of its numbers
    refused(&doc("component Tab() {\n  ax := axis\n  fix(dir == (1, 0, 0)) ax\n}\n",
                 "left := Tab() mirrored(side)\n"), "E112", "declares", "ax := axis");
    // a plane the mirror is not one of the body's datums by
    let (_, errs, _) = gcs_core::library::parse_linked(
        &doc(PLATE, "left := Plate(w: 3mm, d: 2mm, h: 1mm) mirrored(back)\n"));
    assert!(errs.iter().any(|d| d.message.contains("`mirrored(front)`, `mirrored(top)`")),
            "{errs:?}");
}

#[test]
fn a_document_that_says_mirrored_prints_it_back() {
    let src = doc(PLATE, "left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(front)\n");
    let e = read(&src);
    let st = e.program.root().body.iter().find(|s| matches!(&s.kind,
        gcs_core::syntax::StmtKind::Instance(i) if i.name.text == "left")).unwrap();
    let mut printed = String::new();
    gcs_core::syntax::write_stmt_to(&mut printed, &st.kind).unwrap();
    assert_eq!(printed, "left := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(front)");
}


