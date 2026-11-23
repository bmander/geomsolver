//! **The kernel writes every solid its reader reads** (#993 stage 2): a body a file gives is
//! exported again, so the writer and its check (`step::write`, `step_check::verify`) are handed
//! B-reps the kernel did not build — surfaces swept or turned from a line, a circle or an ellipse
//! (`Surface::Extrusion`, `Surface::Revolution`), a solid with a void — which they must write
//! exactly, or refuse with a reason, never panic over.
//!
//! The property, for each solid `r` a file reads to: `r` is written (`Fit::Exact`, stating the
//! file's own uncertainty), the file passes `step_check::verify` against `r` — its topology,
//! solids, shells and every face's surface entity for entity, a swept surface's basis curve and
//! axis or direction among its numbers — and it reads back to the same solid
//! (`step_roundtrip::same`: every face on its original surface, of its sense, each edge and vertex
//! from its own entity, and every face's area and flux, the volume and the area within the
//! measure's own noise, since nothing is fitted). Held over every file of `tests/step_file` the
//! reader reads, and over small files written here by hand whose faces lie on the surfaces a
//! reading makes and the kernel's own construction does not.
use crate::common::{step::Ents,unit};
use crate::step_roundtrip::{joined,measured,same,stages,Found};
use gcs_core::brep::geom::{Curve,Surface};
use gcs_core::brep::step::{write,Fit};
use gcs_core::brep::step_check::{verify,Solid};
use gcs_core::brep::step_read::Given;

type V = [f64; 3];

/// **A prism of the polygon `base` (counter-clockwise about `z`, at `z = 0`) along `d`** (a unit
/// vector rising along `z`): its caps planes, each side the extrusion of its base's edge's `LINE`.
fn prism(base: &[[f64; 2]], d: V, h: f64) -> String {
    let mut e = Ents::new();
    let n = base.len();
    let at = |p: [f64; 2], k: f64| -> V { [p[0] + k * d[0], p[1] + k * d[1], k * d[2]] };
    let low: Vec<u32> = base.iter().map(|&p| e.vertex(at(p, 0.))).collect();
    let high: Vec<u32> = base.iter().map(|&p| e.vertex(at(p, h))).collect();
    let mut lines = Vec::new();
    let (mut bottom, mut top, mut rise) = (Vec::new(), Vec::new(), Vec::new());
    for k in 0..n {
        let (p, q) = (base[k], base[(k + 1) % n]);
        let way = unit([q[0] - p[0], q[1] - p[1], 0.]);
        let lb = e.line(at(p, 0.), way, 1.);
        lines.push(lb);
        bottom.push(e.edge(low[k], low[(k + 1) % n], lb, true));
        let lt = e.line(at(p, h), way, 1.);
        top.push(e.edge(high[k], high[(k + 1) % n], lt, true));
        let lr = e.line(at(p, 0.), d, 1.);
        rise.push(e.edge(low[k], high[k], lr, true));
    }
    let mut faces = Vec::new();
    let (b, t) = (e.plane([0.; 3], [0., 0., 1.], [1., 0., 0.]), e.plane(at([0., 0.], h), [0., 0., 1.], [1., 0., 0.]));
    let down: Vec<(u32, bool)> = (0..n).rev().map(|k| (bottom[k], false)).collect();
    faces.push(e.outer_face(&down, b, false));
    let up: Vec<(u32, bool)> = (0..n).map(|k| (top[k], true)).collect();
    faces.push(e.outer_face(&up, t, true));
    for k in 0..n {
        let side = e.extrusion(lines[k], d);
        faces.push(e.outer_face(&[(bottom[k], true), (rise[(k + 1) % n], true), (top[k], false), (rise[k], false)], side, true));
    }
    e.file(&faces)
}

/// **A drum**: caps the planes `z = z0` and `z = z1` bounded each by the closed curve `ring`
/// makes there (counter-clockwise about `z`, starting at the point it gives), its side the surface
/// `side` makes (its normal out), its seam from the bottom ring's start to the top's on the curve
/// `seam` makes.
fn drum(ring: &dyn Fn(&mut Ents, f64) -> (u32, V), side: &dyn Fn(&mut Ents) -> u32, seam: &dyn Fn(&mut Ents, V, V) -> u32,
    z0: f64, z1: f64) -> String {
    let mut e = Ents::new();
    let (cb, pb) = ring(&mut e, z0);
    let (ct, pt) = ring(&mut e, z1);
    let (vb, vt) = (e.vertex(pb), e.vertex(pt));
    let (eb, et) = (e.edge(vb, vb, cb, true), e.edge(vt, vt, ct, true));
    let g = seam(&mut e, pb, pt);
    let es = e.edge(vb, vt, g, true);
    let s = side(&mut e);
    let (b, t) = (e.plane([0., 0., z0], [0., 0., 1.], [1., 0., 0.]), e.plane([0., 0., z1], [0., 0., 1.], [1., 0., 0.]));
    let faces = [e.outer_face(&[(eb, false)], b, false), e.outer_face(&[(et, true)], t, true),
        e.outer_face(&[(eb, true), (es, true), (et, false), (es, false)], s, true)];
    e.file(&faces)
}

/// A circle of radius `r` about `z` at height `z`, from `(r, 0, z)`.
fn round(r: f64) -> impl Fn(&mut Ents, f64) -> (u32, V) {
    move |e: &mut Ents, z: f64| (e.circle([0., 0., z], [0., 0., 1.], [1., 0., 0.], r), [r, 0., z])
}

/// The point at `t` of the conic of semi-axes `a` (along `x`) and `b` (along `z`) about `(c, 0, 0)`
/// in the plane `y = 0`, running up from `x` at `t = 0`.
fn conic(c: f64, a: f64, b: f64, t: f64) -> V { [c + a * t.cos(), 0., b * t.sin()] }

/// **The files written here**, each a solid on surfaces a reading makes and the kernel's own
/// construction does not: an extrusion of a line, of a circle and (a circle extruded aslant, read
/// as the ellipse square to its direction) of an ellipse, and of an ellipse as itself; a
/// revolution of a line along its axis (a cylinder) and of one meeting it (a cone's nappe), of a
/// circle and of an ellipse beside the axis (a torus's and a spheroidal ring's outer band).
fn written_here() -> Vec<(&'static str, String)> {
    let z: V = [0., 0., 1.];
    let aslant = unit([0.3, -0.2, 1.]);
    let (t30, c30) = (std::f64::consts::FRAC_PI_6, std::f64::consts::FRAC_PI_6.cos());
    vec![
        ("a prism on extrusions of lines", prism(&[[0., 0.], [3., 0.], [3., 4.], [0., 4.]], z, 5.)),
        ("a prism aslant on extrusions of lines", prism(&[[0., 0.], [3., 0.], [1., 2.]], aslant, 2.)),
        ("an extrusion of a circle", drum(&round(2.), &|e| { let c = e.circle([0.; 3], z, [1., 0., 0.], 2.); e.extrusion(c, z) },
            &|e, p, q| e.line(p, unit([q[0] - p[0], q[1] - p[1], q[2] - p[2]]), 1.), 0., 3.)),
        ("an extrusion of a circle aslant", drum(&|e, h| {
            let o = [aslant[0] * h / aslant[2], aslant[1] * h / aslant[2], h];
            (e.circle(o, z, [1., 0., 0.], 2.), [o[0] + 2., o[1], h])
        }, &|e| { let c = e.circle([0.; 3], z, [1., 0., 0.], 2.); e.extrusion(c, aslant) },
            &|e, p, _| e.line(p, aslant, 1.), 0., 3.)),
        ("an extrusion of an ellipse", drum(&|e, h| (e.ellipse([0., 0., h], z, [1., 0., 0.], 3., 1.5), [3., 0., h]),
            &|e| { let c = e.ellipse([0.; 3], z, [1., 0., 0.], 3., 1.5); e.extrusion(c, z) },
            &|e, p, _| e.line(p, z, 1.), 0., 2.)),
        ("a revolution of a line along its axis", drum(&round(2.), &|e| { let l = e.line([2., 0., 0.], z, 1.); e.revolution(l) },
            &|e, p, _| e.line(p, z, 1.), 0., 3.)),
        ("a revolution of a line meeting its axis", drum(&|e, h| round(2. - h / 2.)(e, h),
            &|e| { let l = e.line([2., 0., 0.], unit([-1., 0., 2.]), 1.); e.revolution(l) },
            &|e, p, q| e.line(p, unit([q[0] - p[0], q[1] - p[1], q[2] - p[2]]), 1.), 0., 2.)),
        ("a revolution of a circle", drum(&round(3. + c30), &|e| { let c = e.circle([3., 0., 0.], [0., -1., 0.], [1., 0., 0.], 1.); e.revolution(c) },
            &|e, _, _| e.circle([3., 0., 0.], [0., -1., 0.], [1., 0., 0.], 1.), conic(3., 1., 1., -t30)[2], conic(3., 1., 1., t30)[2])),
        ("a revolution of an ellipse", drum(&round(3. + c30), &|e| { let c = e.ellipse([3., 0., 0.], [0., -1., 0.], [1., 0., 0.], 1., 1.5); e.revolution(c) },
            &|e, _, _| e.ellipse([3., 0., 0.], [0., -1., 0.], [1., 0., 0.], 1., 1.5), conic(3., 1., 1.5, -t30)[2], conic(3., 1., 1.5, t30)[2])),
    ]
}

/// **`text`'s solids, each written again, verified and read back to the same solid**; with how
/// many solids it gave and how many of their faces lie on a swept or turned surface of a line or a
/// conic. `Err` says the first clause that fails.
fn rewritten(name: &str, text: &str) -> Result<(usize, usize), String> {
    let given = stages(text).map_err(|(by, r, _)| format!("{name}: refused by {by:?}: {r}"))?;
    let mut swept = 0;
    for (k, g) in given.iter().enumerate() {
        let who = format!("{name}, solid {k}");
        swept += g.brep.faces.iter().filter(|f| match &f.surface {
            Surface::Extrusion(_, c) | Surface::Revolution(_, c) => matches!(**c, Curve::Line { .. } | Curve::Circle(..) | Curve::Ellipse(..)),
            _ => false,
        }).count();
        let tol = g.context.uncertainty.unwrap_or(1e-7);
        let b = &g.brep;
        let again = std::panic::catch_unwind(|| write(b, "again", tol, Fit::Exact))
            .map_err(|_| format!("{who}: the writer panicked"))?.map_err(|e| format!("{who}: not written: {e}"))?;
        verify(&again, &Solid::of(b)).map_err(|e| format!("{who}: written, not verified: {e}"))?;
        let twice = stages(&again).map_err(|(by, r, _)| format!("{who}: written, read again refused by {by:?}: {r}"))?;
        let read: Given = joined(twice);
        same(b, &measured(b), &again, &read, Fit::Exact, tol, &mut Found::default()).map_err(|d| format!("{who}: read again, not the same: {d}"))?;
    }
    Ok((given.len(), swept))
}

/// **Every checked-in file the reader reads is written again, verified, and reads back the same**
/// (`rewritten`), and so is every file `written_here`; those holding faces on a swept or turned
/// line or conic hold some.
#[test]
fn every_solid_read_is_written_again() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/step_file");
    let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).filter(|p| p.extension().is_some_and(|x| x == "step")).collect();
    files.sort();
    let mut failed = Vec::new();
    let mut read = 0;
    for p in &files {
        let text = std::fs::read_to_string(p).unwrap();
        let name = p.file_name().unwrap().to_string_lossy().to_string();
        // (a file holding no solid, or an assembly, is refused before anything is written)
        if stages(&text).is_err() { eprintln!("{name}: not read, nothing to write"); continue }
        read += 1;
        match rewritten(&name, &text) { Ok((n, _)) => eprintln!("{name}: {n} solids written again, the same"), Err(e) => failed.push(e) }
    }
    assert!(read >= 8, "only {read} of the checked-in files read");
    for (name, text) in written_here() {
        match rewritten(name, &text) {
            Ok((n, swept)) if n == 1 && swept > 0 => eprintln!("{name}: written again, the same ({swept} faces swept or turned)"),
            Ok((n, swept)) => failed.push(format!("{name}: {n} solids, {swept} faces on a swept or turned line or conic")),
            Err(e) => failed.push(e),
        }
    }
    assert!(failed.is_empty(), "{}", failed.join("\n"));
}

/// **A body of several lumps, or with voids, is written as the solids it is** (`step::lumps`):
/// ISO 10303-42 holds a closed shell to be a connected face set, so each lump is its own solid of
/// the representation, a lump's voids a `BREP_WITH_VOIDS`'s shells, each an
/// `ORIENTED_CLOSED_SHELL` turned (`.F.`), a lump inside a void a solid of its own; the file
/// passes `step_check::verify`, reads to as many solids, and they are the body
/// (`step_roundtrip::trip`, twice over).
#[test]
fn lumps_and_voids_are_written_as_the_solids_they_are() {
    use crate::step_roundtrip::{trip,Outcome};
    use gcs_core::brep::boolean::{combined,Op};
    use gcs_core::brep::build::{block,sphere};
    let big = block([0.; 3], [10., 10., 10.]).unwrap();
    let hollow = combined(&big, &block([1.; 3], [9.; 3]).unwrap(), Op::Cut, 0.).unwrap();
    let island = hollow.joined(&block([3.; 3], [6.; 3]).unwrap());
    let one = combined(&big, &sphere([3., 3., 3.], [0., 0., 1.], 1.5).unwrap(), Op::Cut, 0.).unwrap();
    let two = combined(&one, &sphere([7., 6., 5.], [0., 0., 1.], 2.).unwrap(), Op::Cut, 0.).unwrap();
    let apart = big.joined(&block([20., 0., 0.], [25., 5., 5.]).unwrap());
    // (each: its solids, simple and with voids, and its voids)
    for (name, b, counts) in [("a block with a void holding an island", island, [1, 1, 1]), ("a block with two spherical voids", two, [0, 1, 2]),
        ("two blocks apart", apart, [2, 0, 0])] {
        let text = write(&b, name, 1e-7, Fit::Exact).unwrap_or_else(|e| panic!("{name}: {e}"));
        let n = |kind: &str| text.matches(&format!(" = {kind}(")).count();
        assert_eq!([n("MANIFOLD_SOLID_BREP"), n("BREP_WITH_VOIDS"), n("ORIENTED_CLOSED_SHELL")], counts, "{name}");
        assert!(text.lines().filter(|l| l.contains(" = ORIENTED_CLOSED_SHELL(")).all(|l| l.ends_with(",.F.);")), "{name}: a void's shell not turned");
        verify(&text, &Solid::of(&b)).unwrap_or_else(|e| panic!("{name}: {e}"));
        let (o, _) = trip(&b, name, 1e-7, Fit::Exact);
        assert!(matches!(o, Outcome::Same), "{name}: {}: {}", o.label(), o.detail());
    }
}

/// **The corpus written again** (a tool; `$STEP_CORPUS`, scripts/step-corpus/README.md): every
/// `.step` and `.stp` file under it (or under `STEP_REWRITE_ONLY`, a path within it) that the reader
/// reads, each solid written again, verified and read back (`rewritten`), on `STEP_REWRITE_THREADS`
/// threads (two by default); one line a file and a tally by outcome, each failure by its cause.
#[test]
#[ignore = "a tool: STEP_CORPUS names the corpus outside the repository"]
fn the_corpus_is_written_again() {
    let Ok(root) = std::env::var("STEP_CORPUS") else { eprintln!("no STEP_CORPUS"); return };
    let base = std::path::Path::new(&root).join(std::env::var("STEP_REWRITE_ONLY").unwrap_or_default());
    fn walk(p: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
        let Ok(dir) = std::fs::read_dir(p) else { return };
        let mut entries: Vec<_> = dir.map(|e| e.unwrap().path()).collect();
        entries.sort();
        for e in entries {
            if e.is_dir() { if !e.ends_with("build") { walk(&e, out) } }
            else if e.extension().is_some_and(|x| x == "step" || x == "stp") { out.push(e) }
        }
    }
    let mut files = Vec::new();
    walk(&base, &mut files);
    let threads: usize = std::env::var("STEP_REWRITE_THREADS").ok().and_then(|s| s.parse().ok()).unwrap_or(2);
    let next = std::sync::atomic::AtomicUsize::new(0);
    let rows = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|s| for _ in 0..threads { s.spawn(|| loop {
        let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let Some(p) = files.get(k) else { break };
        let name = p.strip_prefix(&root).unwrap_or(p).display().to_string();
        let Ok(text) = std::fs::read_to_string(p) else { continue };
        let started = std::time::Instant::now();
        let got = std::panic::catch_unwind(|| match stages(&text) {
            Err(_) => ("not read".to_string(), String::new()),
            Ok(_) => match rewritten(&name, &text) {
                Ok((n, swept)) => ("written again, verified, the same".to_string(), format!("{n} solids, {swept} faces swept or turned")),
                Err(e) => {
                    let class = ["the writer panicked", "not written", "not verified", "read again refused", "not the same"].into_iter()
                        .find(|c| e.contains(c)).unwrap_or("failed").to_string();
                    (class, e)
                }
            },
        }).unwrap_or_else(|_| ("panicked".to_string(), String::new()));
        eprintln!("REWRITE\t{name}\t{}\t{:.1?}\t{}", got.0, started.elapsed(), got.1);
        rows.lock().unwrap().push(got);
    }); });
    let mut tally: std::collections::BTreeMap<String, usize> = Default::default();
    for (class, _) in rows.into_inner().unwrap() { *tally.entry(class).or_default() += 1; }
    for (k, n) in &tally { eprintln!("REWRITE SUMMARY {n:4}  {k}"); }
}

/// **A face read on its sheet's rows is written again and read on them again**
/// (`topology::rows`, #993 stage 2): benchy's face #27487 under a slab (`step_kinds/
/// crumpled_corner.step`), its edges up to 4.4e-3 off the rows of its sheet's crumpled corner and
/// within 1e-4 of the crumple, is read on the rows; written, the file states 1e-2, what its edges
/// stand off those rows, and read again by the edges' nearest feet the face leaps into the crumple
/// as before. Read on the rows again, each row is no point: a row is collapsed to a pole's within
/// the tolerance the boundary is checked to, never within the farthest an edge may stand off its
/// face — a hundred times the uncertainty stated, a millimetre here, by which the 0.28 mm row the
/// face's fourth edge lies along was a point, that edge was laid on a row across it 0.28 mm off,
/// and the solid written of what was read was refused.
#[test]
fn a_face_read_on_its_sheets_rows_is_written_again() {
    let text = include_str!("step_kinds/crumpled_corner.step");
    let (n, _) = rewritten("crumpled_corner.step", text).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(n, 1);
}
