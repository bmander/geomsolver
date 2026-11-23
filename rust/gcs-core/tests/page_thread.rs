//! **The page's thread meshes no body a file gives** (#993 stage 2).  A page with a pool asks its
//! objects of workers (fields.md, app.md) and, on its own thread, only what draws and reports the
//! drawing: the objects not drawn and why (`overview::unbuilt`), each object's evaluation at the
//! box's approximation (from its build, `Sketch::supply_object`), the unit a mesh is cut at
//! (`solid::mesh_unit`, the ABI's `gcs_solid_mesh_unit`) and the diagnosis without refusals; its
//! report is the elaboration's diagnostics and names, and measures no body (`report::positions`,
//! which measures every body, is `solventc`'s).  An ordinary solid's are cheap by construction; a
//! given body's B-rep may be thousands of faces, and meshing it there held the page for half a
//! minute an edit.  So the gate counts the B-reps meshed on this thread (`solid::brep_meshes`)
//! around those calls — on opening, before the build arrives and after, and across an edit — and
//! holds the count at none.

use crate::common::linked::{linked_with, solving};
use gcs_core::modules::relink;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solid::{brep_meshes, ApproximationPolicy, ObjectState};
use gcs_core::syntax::parse;

const DRILLED: &str = include_str!("step_file/drilled_ap214.step");
const DOC: &str = "unit mm\nuse bracket\npart := solid(bracket)\n";

/// `src` linked as a host links it (`bracket` the drilled block's file), elaborated and solved.
fn read(src: &str) -> Elaborated {
    let (prog, diags) = linked_with(src, &|name| (name == "bracket").then(|| DRILLED.to_string()));
    assert!(diags.is_empty(), "{diags:?}");
    solving(elaborate(&prog))
}

/// What the page's thread asks of a drawing on opening and after every edit, each held to mesh
/// nothing: `when` says at which point of the page's life.
fn page_asks(e: &mut Elaborated, part: usize, when: &str) {
    let none = |what: &str, before: usize| {
        assert_eq!(brep_meshes(), before,
                   "{when}: {what} meshed a given body on the page's thread");
    };
    let at = brep_meshes();
    let _ = gcs_core::overview::unbuilt(&e.sketch);
    none("`overview::unbuilt`", at);
    let _ = gcs_core::overview::objects(&e.sketch);
    let _ = e.sketch.evaluated_solid(part, ApproximationPolicy::Mesh);
    none("the box's evaluation", at);
    assert!(gcs_core::solid::mesh_unit(&e.sketch, part) > 0.0);
    none("`solid::mesh_unit`", at);
    let _ = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions {
        refusals: false, solid_claims: false, ..Default::default() });
    none("the diagnosis a page asks", at);
}

#[test]
fn a_given_body_is_meshed_on_no_page_thread() {
    // the worker's build, made first: it meshes, as a worker does
    let host = read(DOC);
    let part = host.sketch.solids.iter().position(|s| s.name == "part").unwrap();
    let job = host.sketch.object_jobs().into_iter().find(|j| j.objects.contains(&part)).unwrap();
    let mut b = gcs_core::brep::export::Builder::new(&host.sketch, job.solid)
        .map_err(|r| r.message).unwrap();
    while !b.step().map_err(|r| r.message).unwrap() {}
    let display = b.display().unwrap().clone();

    // opened: read, its objects asked of the pool, and on their way
    let before = brep_meshes();
    let mut page = read(DOC);
    page.sketch.supply_objects(true);
    let job = page.sketch.object_jobs().into_iter().find(|j| j.objects.contains(&part)).unwrap();
    assert_eq!(brep_meshes(), before, "reading the document meshed a given body");
    assert_eq!(page.sketch.object_state(part), ObjectState::Building);
    page_asks(&mut page, part, "opened");
    // its build arrives, and is drawn
    assert!(page.sketch.supply_object(job.solid, display.clone()).unwrap());
    page_asks(&mut page, part, "built");
    assert!(gcs_core::overview::unbuilt(&page.sketch).is_empty());
    let drawn = page.sketch.evaluated_solid(part, ApproximationPolicy::Mesh).unwrap();
    assert!(drawn.volume() > 0.0);
    assert_eq!(brep_meshes(), before, "the page meshed a given body once its build arrived");
    // an edit: the program read again, linked as the last one was, its last build offered
    let (mut next, errs) = parse(&format!("{DOC}w := 5mm\n"));
    assert!(errs.is_empty());
    assert!(relink(&mut next, &page.program).is_empty());
    let mut edited = solving(elaborate(&next));
    edited.sketch.supply_objects(true);
    let job = edited.sketch.object_jobs().into_iter().find(|j| j.objects.contains(&part)).unwrap();
    edited.sketch.supply_object(job.solid, display).unwrap();
    page_asks(&mut edited, part, "edited");
    assert_eq!(brep_meshes(), before, "the page meshed a given body after an edit");
}

/// The unit a given body's mesh is cut at is of its own size, as an ordinary solid's is: its
/// edges' box (`Given::extent`) — the drilled block's 40 × 30 × 20 — not a mesh of it.
#[test]
fn a_given_bodys_mesh_unit_is_of_its_size() {
    let e = read(DOC);
    let part = e.sketch.solids.iter().position(|s| s.name == "part").unwrap();
    let diag = (40.0f64 * 40.0 + 30.0 * 30.0 + 20.0 * 20.0).sqrt();
    let want = (diag * gcs_core::solid::MESH_SAGITTA / gcs_core::curve::FLATNESS_PX)
        .max(gcs_core::solid::REPORT_UNIT);
    let before = brep_meshes();
    let unit = gcs_core::solid::mesh_unit(&e.sketch, part);
    assert_eq!(brep_meshes(), before);
    assert!((unit - want).abs() <= 1e-9 * want, "{unit} against {want}");
}
