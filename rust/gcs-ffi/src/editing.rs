//! Editing the source: every gesture as a splice the core computes.

use super::*;
use gcs_core::edit::{
    BodyWordRequest, DatumRequest, EntityRequest, ExtrudeRequest, FaceRequest, FaceSketchRequest,
    RevolveRequest, RoundsRequest, SelectionRequest, SolidRequest, SolidSweep,
};

/* -- editing the source ------------------------------------------------------------
 *
 * Every one of these returns `{text, kind, names, refused}` and changes nothing: the caller
 * applies the text it gets back by elaborating it, which is the only place a document is ever
 * replaced.  `kind` says what that costs — `numeric` means only numbers a solve may move
 * changed, so a compiled plan survives.  What a gesture asks is read by the core
 * (`edit::requests`); this only marshals.
 */

fn out_edit(e: gcs_core::edit::Edit) -> *mut u8 {
    out_json(json::object([
        ("text", Json::Str(e.text)),
        ("kind", Json::Str(e.kind.as_str().to_string())),
        ("names", Json::Arr(e.names.into_iter().map(Json::Str).collect())),
        ("refused", e.refused.map(Json::Str).unwrap_or(Json::Null)),
        ("kept", Json::Num(e.kept as f64)),
    ]))
}

/// `gcs_describe`, with every entity named as the document names it: the wording is the
/// core's either way, and this is where a front end that holds the source gets the names in.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_describe(h: *mut Elaborated, s: *mut Sketch, id: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e = &*h;
        let sk = sk(s);
        out_str(
            sk.constraint(id as u32)
                .map(|c| io::describe_with(c, &|x| e.map.name_of(x).cloned()))
                .unwrap_or_default(),
        )
    })
}

/// Put a solved sketch's coordinates back into the seeds they came from.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_commit_seeds(h: *mut Elaborated, s: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e = &*h;
        out_edit(gcs_core::edit::commit_seeds(e, sk(s), &e.program))
    })
}

/// The source after a gesture mutated the drawing: what is new gets a statement, what is gone
/// loses one, and every seed is committed.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_reconcile(h: *mut Elaborated, s: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_edit(gcs_core::edit::reconcile(&mut *h, sk(s))))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_point(h: *mut Elaborated, x: f64, y: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_edit(gcs_core::edit::add_point(&(*h).program, x, y))
    })
}

/// `use NAME`, as a line of the document's own — nothing when it already says it.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_use(h: *mut Elaborated, name: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_edit(gcs_core::edit::add_use(&(*h).program, &as_str(name, len)))
    })
}

/// A `Rectangle` component instance — and the component's definition, the first time.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_rectangle(
    h: *mut Elaborated,
    w: f64,
    hh: f64,
    plane: *const u8,
    plane_len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        // an empty name is no plane, which the core refuses: a rectangle is drawn in one
        out_edit(gcs_core::edit::add_rectangle(&(*h).program, w, hh, as_str(plane, plane_len)))
    })
}

/// `{"kind": "line", "args": ["p0", "p1"], "seed": [..]}`
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_entity(
    h: *mut Elaborated,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(r) = request(EntityRequest::from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        // a plane over two axes or lines, `args` their names, and the name asked for
        if r.kind == EntKind::Frame {
            return out_edit(gcs_core::edit::add_plane(&(*h).program, &r.args, r.name.as_deref()));
        }
        out_edit(gcs_core::edit::add_entity(&(*h).program, r.kind, &r.args, &r.seed))
    })
}

/// A request read by the core (`edit::requests`), or `None` with why it is not one in
/// `gcs_last_error()`.
fn request<T>(r: Result<T, String>) -> Option<T> {
    r.map_err(|why| set_error(why)).ok()
}

/// A face over drawn edges by name, `{edges: [..], holes: [[..]], name?}`: `edit::add_face`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_face(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = FaceRequest::from_json(&as_json(ptr, len));
        out_edit(gcs_core::edit::add_face(&(*h).program, &r.edges, &r.holes, r.name.as_deref()))
    })
}

/// A solid swept from a face by name, `{face, depth?, from?, to?, through?, about?, sweep?,
/// sense?, name?}`, every extent the text written: `edit::add_solid`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_solid(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = SolidRequest::from_json(&as_json(ptr, len));
        out_edit(gcs_core::edit::add_solid(&(*h).program, &r.face, &r.sweep, r.name.as_deref()))
    })
}

/// An extrusion of the region a click found — `{outer: [..], holes: [[..]], word?: "union" |
/// "cut", body?, through?}` — the face, the solid and the body rule in one splice, seeded off the
/// live drawing `s`: `edit::extrude`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_extrude(h: *mut Elaborated, s: *mut Sketch, ptr: *const u8,
                                          len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(r) = request(ExtrudeRequest::from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        let g = &r.region;
        out_edit(gcs_core::edit::extrude(&(*h).program, sk(s), &g.outer, &g.holes, g.with(),
                                         r.through))
    })
}

/// A revolution of the region a click found about a line or an axis — `{outer: [..], holes:
/// [[..]], about, word?: "union" | "cut", body?}` — the face, the solid and the body rule in one
/// splice: `edit::revolve`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_revolve(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(r) = request(RevolveRequest::from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        let g = &r.region;
        out_edit(gcs_core::edit::revolve(&(*h).program, &g.outer, &g.holes, &r.about, g.with()))
    })
}

/// Rewrite how solid `solid` is swept — `{way: "behind" | "front" | "both", thick}`, an
/// extrusion's thickness the way it runs, or a sweep outright, `{depth?, from?, to?, …}`:
/// `edit::set_sweep`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_set_sweep(h: *mut Elaborated, solid: i32, ptr: *const u8,
                                            len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let how = SolidSweep::from_request(&as_json(ptr, len));
        let Some(solid) = index(solid) else {
            set_error(format!("{solid} names no solid"));
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::set_sweep(&*h, &(*h).program, solid, &how))
    })
}
/// Rewrite how far revolution `solid` turns about what it turns about — `angle` the text, in
/// degrees, a number signed (negative clockwise): `edit::set_turn`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_set_turn(h: *mut Elaborated, solid: i32, ptr: *const u8,
                                           len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let angle = as_str(ptr, len);
        let Some(solid) = index(solid) else {
            set_error(format!("{solid} names no solid"));
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::set_turn(&*h, &(*h).program, solid, angle))
    })
}

/// Re-extrude prism `i` where it stands, `d` thick the way `way` says (0 behind, 1 in front, 2
/// both) — the preview an extrusion being sized shows before the source is written:
/// `Sketch::set_prism`.  1 done, 0 not a prism.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_prism(h: *mut Sketch, i: i32, way: i32, d: f64) -> i32 {
    use gcs_core::model::Way;
    let way = match way { 1 => Way::Front, 2 => Way::Both, _ => Way::Behind };
    guard(0, move || i32::from(index(i).is_some_and(|i| sk(h).set_prism(i, way, d))))
}

/// Re-turn revolution `i` where it stands through `degrees`, signed (negative clockwise about its
/// axis) — the preview a revolution being sized shows before the source is written:
/// `Sketch::set_turn`.  1 done, 0 not a revolution.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_turn(h: *mut Sketch, i: i32, degrees: f64) -> i32 {
    guard(0, move || i32::from(index(i).is_some_and(|i| sk(h).set_turn(i, degrees))))
}

/// Bumped whenever a solid's shape changes in place (`set_prism`, `set_turn`): what a cached mesh
/// keys on.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_shape_epoch(h: *mut Sketch) -> u32 {
    guard(0, move || sk(h).shape_epoch)
}

/// A datum dropped by a button — `{kind: "plane" | "axis", from: [names], current?, name?}`,
/// constrained to what `from` selects and seeded off the live drawing `s`: `edit::add_datum`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_datum(h: *mut Elaborated, s: *mut Sketch, ptr: *const u8,
                                            len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(r) = request(DatumRequest::from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::add_datum(&*h, sk(s), r.kind, &r.from, r.current.as_deref(),
                                           r.name.as_deref()))
    })
}

/// Sketch on a face of object `solid`, `{face, at?: [x, y, z], name?}` — a plane lying on face
/// `face`, seeded where the eye's ray met it on the live drawing `s`: `edit::sketch_on_face`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_sketch_on_face(h: *mut Elaborated, s: *mut Sketch, solid: i32,
                                                 ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = FaceSketchRequest::from_json(&as_json(ptr, len));
        let Some(solid) = index(solid) else {
            set_error("no solid to sketch on");
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::sketch_on_face(&*h, sk(s), solid, &r.face, r.at,
                                                r.name.as_deref()))
    })
}
/// The plane already lying on face `face` of object `solid`, by name, or `null`:
/// `edit::face_plane`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_face_plane_json(h: *mut Elaborated, s: *mut Sketch, solid: i32,
                                                  ptr: *const u8, len: usize) -> *mut u8 {
    json_out(move || {
        let face = as_str(ptr, len);
        let found = index(solid)
            .and_then(|i| gcs_core::edit::face_plane(&*h, sk(s), i, face));
        found.map_or(Json::Null, Json::Str)
    })
}

/// The body rule, `{word: "union" | "cut" | "common", what: [..], body}`, a statement for each
/// solid in `what`, in one splice: `edit::add_body_word`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_body_word(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(r) = request(BodyWordRequest::from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        let what: Vec<&str> = r.what.iter().map(String::as_str).collect();
        out_edit(gcs_core::edit::add_body_word(&(*h).program, r.word, &what, &r.body))
    })
}

/// Round or chamfer edges of object `solid`, `{word: "round" | "chamfer", faces: [..], edges:
/// [[a, b]..], whole?, size}` — faces and edges by the paths its mesh names them by, `whole` the
/// body picked whole — as the library's words, in one splice: `edit::add_rounds`, over the live
/// drawing `s`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_rounds(h: *mut Elaborated, s: *mut Sketch, solid: i32,
                                             ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(r) = request(RoundsRequest::from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        let Some(solid) = index(solid) else {
            set_error("no solid to round");
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::add_rounds(&(*h).program, sk(s), solid, r.word, &r.faces,
                                            &r.edges, r.whole, &r.size))
    })
}

/// The region of the drawing on plane `plane` (-1 the page) around (x, y) in its coordinates:
/// `{outer: [names], holes: [[names]], rings: [[[x, y]…]…]}`, `null` where no loop encloses it,
/// or `{refused}` with the cause — `overview::region::region_at`, named by the source.  The live
/// sketch is `s`, the elaboration's own having been taken out into it.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_region_json(h: *mut Elaborated, s: *mut Sketch, plane: i32,
                                              x: f64, y: f64, unit: f64) -> *mut u8 {
    json_out(move || {
        use gcs_core::overview::region;
        let plane = index(plane);
        let names = |n: Vec<String>| Json::Arr(n.into_iter().map(Json::Str).collect());
        let refused = |why: String| json::object([("refused", Json::Str(why))]);
        match region::region_at(sk(s), plane, (x, y), unit) {
            Ok(None) => Json::Null,
            Err(why) => refused(why),
            Ok(Some(r)) => match region::written(&(*h).map, &r) {
                Err(why) => refused(why),
                Ok((outer, holes)) => json::object([
                    ("outer", names(outer)),
                    ("holes", Json::Arr(holes.into_iter().map(names).collect())),
                    ("rings", Json::Arr(r.rings.iter().map(|ring| Json::Arr(ring.iter()
                        .map(|&(a, b)| Json::Arr(vec![Json::Num(a), Json::Num(b)]))
                        .collect())).collect())),
                ]),
            },
        }
    })
}

/// One constraint, in `report::constraint_from_json`'s shape but with entities by *name* — the
/// document's own way of saying which, rather than an index into a sketch it is about to replace:
/// `edit::relation_from_json`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_relation(
    h: *mut Elaborated,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(rel) = request(gcs_core::edit::relation_from_json(&as_json(ptr, len))) else {
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::add_relation(&(*h).program, rel))
    })
}

/// `{"entities": [["point", 3], ...], "constraints": [id, ...]}`
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_remove(
    h: *mut Elaborated,
    s: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = SelectionRequest::from_json(&as_json(ptr, len));
        // the live sketch, not the elaboration's: the binding took that one out into `s`
        let e = &*h;
        out_edit(gcs_core::edit::remove(e, &e.program, sk(s), &r.ents, &r.cons))
    })
}

/// `{"entities": […], "constraints": […], "name": "Bracket"}` — the selection packaged as a
/// component (`edit::make_component`), `name` optional; the edit's `names` are the component
/// and its instance.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_make_component(
    h: *mut Elaborated,
    s: *mut Sketch,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = SelectionRequest::from_json(&as_json(ptr, len));
        let e = &*h;
        let name = r.name.as_deref();
        out_edit(gcs_core::edit::make_component(e, &e.program, sk(s), &r.ents, &r.cons, name))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_elab_set_dimension(
    h: *mut Elaborated,
    cid: u32,
    ap: *const u8,
    an: usize,
    tp: *const u8,
    tn: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let e = &*h;
        out_edit(gcs_core::edit::set_dimension(
            e,
            &e.program,
            cid,
            as_str(ap, an),
            as_str(tp, tn),
        ))
    })
}

/// Take a new source that says the same statements, keeping the drawing.  0 when it cannot —
/// the caller then elaborates, which is always correct and only slower.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_retext(h: *mut Elaborated, p: *const u8, n: usize) -> i32 {
    guard(0, move || (*h).retext(&as_str(p, n)) as i32)
}

/// One curve as a polyline, over the interval that curve is drawn on.
///
/// The core lays it out: a curve is geometry, so where it goes is the core's answer and a front
/// end only strokes what it is handed — the same bargain a dimension callout and a B-spline
/// already strike.  Writes `2n` doubles and returns `n`; when `cap` is too small it writes
/// nothing and still returns the count it wanted, so a caller sizes its buffer by asking once.
#[no_mangle]
pub unsafe extern "C" fn gcs_curve_polyline(
    h: *mut Sketch,
    idx: i32,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = index(idx).filter(|&i| i < s.curves.len()) else { return -1 };
        let pts = s.curve_polyline(i);
        if (pts.len() as i32) <= cap && !out.is_null() {
            for (k, (x, y)) in pts.iter().enumerate() {
                *out.add(2 * k) = *x;
                *out.add(2 * k + 1) = *y;
            }
        }
        pts.len() as i32
    })
}

/// A sketch as a program — the lift, and the whole of the migration: every document ever saved
/// becomes text through this.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_to_program(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str(program::dumps(sk(h))))
}
