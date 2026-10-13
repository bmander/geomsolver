//! Editing the source: every gesture as a splice the core computes.

use super::*;

/* -- editing the source ------------------------------------------------------------
 *
 * Every one of these returns `{text, kind, names, refused}` and changes nothing: the caller
 * applies the text it gets back by elaborating it, which is the only place a document is ever
 * replaced.  `kind` says what that costs — `numeric` means only numbers a solve may move
 * changed, so a compiled plan survives.
 */

fn out_edit(e: gcs_core::edit::Edit) -> *mut u8 {
    use gcs_core::edit::Kind;
    out_json(json::object([
        ("text", Json::Str(e.text)),
        (
            "kind",
            Json::Str(
                match e.kind {
                    Kind::Structural => "structural",
                    Kind::Numeric => "numeric",
                    Kind::None => "none",
                }
                .to_string(),
            ),
        ),
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
        // an empty name is the page: the instance takes no clause
        let p = as_str(plane, plane_len);
        let p = (!p.is_empty()).then_some(p);
        out_edit(gcs_core::edit::add_rectangle(&(*h).program, w, hh, p.as_deref()))
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
        let v = as_json(ptr, len);
        let Some(kind) = EntKind::parse(v.get("kind").map(|k| k.as_str()).unwrap_or("")) else {
            set_error("unknown entity kind");
            return std::ptr::null_mut();
        };
        let args: Vec<String> = v.get("args").map(strings).unwrap_or_default();
        let seed: Vec<f64> =
            v.get("seed").map(|a| a.arr().iter().map(|x| x.as_f64()).collect()).unwrap_or_default();
        // a plane over two axes or lines, `args` their names, and the name asked for
        if kind == EntKind::Plane {
            let name = v.get("name").map(|n| n.as_str().to_string());
            return out_edit(gcs_core::edit::add_plane(&(*h).program, &args, name.as_deref()));
        }
        out_edit(gcs_core::edit::add_entity(&(*h).program, kind, &args, &seed))
    })
}

/// A JSON array of strings, as names; anything not a string reads as the empty name.
fn strings(a: &Json) -> Vec<String> {
    a.arr().iter().map(|x| x.as_str().to_string()).collect()
}

/// A face over drawn edges by name, `{edges: [..], holes: [[..]], name?}`: `edit::add_face`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_face(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let edges = v.get("edges").map(strings).unwrap_or_default();
        let holes: Vec<Vec<String>> =
            v.get("holes").map(|a| a.arr().iter().map(strings).collect()).unwrap_or_default();
        let name = v.get("name").map(|n| n.as_str().to_string());
        out_edit(gcs_core::edit::add_face(&(*h).program, &edges, &holes, name.as_deref()))
    })
}

/// A solid swept from a face by name, `{face, depth?, from?, to?, through?, about?, sweep?,
/// sense?, name?}`, every extent the text written: `edit::add_solid`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_solid(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let text = |k: &str| v.get(k).map(|x| x.as_str().to_string());
        let face = text("face").unwrap_or_default();
        let name = text("name");
        out_edit(gcs_core::edit::add_solid(&(*h).program, &face, &sweep_of(&v), name.as_deref()))
    })
}

/// A sweep as a JSON object says it: `{depth?, from?, to?, through?, about?, sweep?, sense?}`,
/// every extent the text written.
fn sweep_of(v: &Json) -> gcs_core::edit::SolidSweep {
    let text = |k: &str| v.get(k).map(|x| x.as_str().to_string());
    let sense = match v.get("sense").map(|x| x.as_str()) {
        Some("cw") => Some(gcs_core::syntax::Sense::Cw),
        Some("ccw") => Some(gcs_core::syntax::Sense::Ccw),
        _ => None,
    };
    gcs_core::edit::SolidSweep {
        depth: text("depth"), from: text("from"), to: text("to"), through: text("through"),
        about: text("about"), sweep: text("sweep"), sense,
    }
}

/// An extrusion of the region a click found — `{outer: [..], holes: [[..]], word?: "union" |
/// "cut", body?, through?}` — the face, the solid and the body rule in one splice, seeded off the
/// live drawing `s`: `edit::extrude`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_extrude(h: *mut Elaborated, s: *mut Sketch, ptr: *const u8,
                                          len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let Some((outer, holes, body)) = region_request(&v) else { return std::ptr::null_mut() };
        let with = body.as_ref().map(|(w, b)| (*w, b.as_str()));
        let through = v.get("through").is_some_and(|x| x.as_bool());
        out_edit(gcs_core::edit::extrude(&(*h).program, sk(s), &outer, &holes, with, through))
    })
}

/// What a region's solid is asked: its outer loop and holes by name, and the body rule it joins
/// with — `word` and `body`, or neither — or `None` with the error set, for a word that is not
/// one.
fn region_request(v: &Json)
    -> Option<(Vec<String>, Vec<Vec<String>>, Option<(gcs_core::syntax::BodyWord, String)>)> {
    let outer = v.get("outer").map(strings).unwrap_or_default();
    let holes = v.get("holes").map(|a| a.arr().iter().map(strings).collect()).unwrap_or_default();
    let body = match (v.get("word").map(|w| w.as_str()), v.get("body").map(|b| b.as_str())) {
        (None, _) => None,
        (Some(w), body) => match (gcs_core::syntax::BodyWord::parse(w), body) {
            (Some(word), Some(body)) => Some((word, body.to_string())),
            _ => {
                set_error(format!("`{w}` with a body is the body rule: union, cut or bound"));
                return None;
            }
        },
    };
    Some((outer, holes, body))
}

/// A revolution of the region a click found about a line or an axis — `{outer: [..], holes:
/// [[..]], about, word?: "union" | "cut", body?}` — the face, the solid and the body rule in one
/// splice: `edit::revolve`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_revolve(h: *mut Elaborated, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let Some((outer, holes, body)) = region_request(&v) else { return std::ptr::null_mut() };
        let with = body.as_ref().map(|(w, b)| (*w, b.as_str()));
        let about = v.get("about").map(|x| x.as_str().to_string()).unwrap_or_default();
        out_edit(gcs_core::edit::revolve(&(*h).program, &outer, &holes, &about, with))
    })
}

/// Rewrite how solid `solid` is swept — `{way: "behind" | "front" | "both", thick}`, an
/// extrusion's thickness the way it runs, or a sweep outright, `{depth?, from?, to?, …}`:
/// `edit::set_sweep`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_set_sweep(h: *mut Elaborated, solid: i32, ptr: *const u8,
                                            len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let way = v.get("way").and_then(|w| gcs_core::model::Way::parse(w.as_str()));
        let how = match (way, v.get("thick")) {
            (Some(way), Some(t)) => gcs_core::edit::SolidSweep::thick(way, t.as_str()),
            _ => sweep_of(&v),
        };
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
        let v = as_json(ptr, len);
        let text = |k: &str| v.get(k).map(|x| x.as_str().to_string());
        let kind = EntKind::parse(text("kind").as_deref().unwrap_or_default());
        let Some(kind) = kind.filter(|k| matches!(k, EntKind::Plane | EntKind::Axis)) else {
            set_error("a datum is a plane or an axis");
            return std::ptr::null_mut();
        };
        let from = v.get("from").map(strings).unwrap_or_default();
        out_edit(gcs_core::edit::add_datum(&*h, sk(s), kind, &from, text("current").as_deref(),
                                           text("name").as_deref()))
    })
}

/// Sketch on a face of object `solid`, `{face, at?: [x, y, z], name?}` — a plane lying on face
/// `face`, seeded where the eye's ray met it on the live drawing `s`: `edit::sketch_on_face`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_sketch_on_face(h: *mut Elaborated, s: *mut Sketch, solid: i32,
                                                 ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let face = v.get("face").map(|x| x.as_str().to_string()).unwrap_or_default();
        let at = v.get("at").and_then(|a| match a.arr() {
            [x, y, z] => Some([x.as_f64(), y.as_f64(), z.as_f64()]),
            _ => None,
        });
        let name = v.get("name").map(|x| x.as_str().to_string());
        let Some(solid) = index(solid) else {
            set_error("no solid to sketch on");
            return std::ptr::null_mut();
        };
        out_edit(gcs_core::edit::sketch_on_face(&*h, sk(s), solid, &face, at, name.as_deref()))
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
        let v = as_json(ptr, len);
        let text = |k: &str| v.get(k).map(|x| x.as_str()).unwrap_or_default();
        let Some(word) = gcs_core::syntax::BodyWord::parse(text("word")) else {
            set_error(format!("`{}` is not a body word: union, cut or bound", text("word")));
            return std::ptr::null_mut();
        };
        let what = v.get("what").map(strings).unwrap_or_default();
        let what: Vec<&str> = what.iter().map(String::as_str).collect();
        out_edit(gcs_core::edit::add_body_word(&(*h).program, word, &what, text("body")))
    })
}

/// Round or chamfer edges of object `solid`, `{word: "round" | "chamfer", faces: [..], edges:
/// [[a, b]..], size}` — faces and edges by the paths its mesh names them by — as the library's
/// words, in one splice: `edit::add_rounds`, over the live drawing `s`.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_rounds(h: *mut Elaborated, s: *mut Sketch, solid: i32,
                                             ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let text = |k: &str| v.get(k).map(|x| x.as_str().to_string()).unwrap_or_default();
        let Some(word) = gcs_core::edit::EdgeWord::parse(&text("word")) else {
            set_error(format!("`{}` is not an edge word: round or chamfer", text("word")));
            return std::ptr::null_mut();
        };
        let Some(solid) = index(solid) else {
            set_error("no solid to round");
            return std::ptr::null_mut();
        };
        let faces = v.get("faces").map(strings).unwrap_or_default();
        let edges: Vec<(String, String)> = v.get("edges").map(|a| a.arr().iter().filter_map(|p| {
            match &strings(p)[..] {
                [a, b] => Some((a.clone(), b.clone())),
                _ => None,
            }
        }).collect()).unwrap_or_default();
        out_edit(gcs_core::edit::add_rounds(&(*h).program, sk(s), solid, word, &faces, &edges,
                                            &text("size")))
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
/// document's own way of saying which, rather than an index into a sketch it is about to replace.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_add_relation(
    h: *mut Elaborated,
    ptr: *const u8,
    len: usize,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        // either spelling: `Horizontal`, the record shape both bindings build, or `horizontal`,
        // the way a statement says it.  One name for one thing, written twice.
        let name = v.get("type").map(|x| x.as_str().to_string()).unwrap_or_default();
        let kind = gcs_core::constraints::CKind::from_name(&name)
            .or_else(|| gcs_core::constraints::CKind::from_name(&gcs_core::syntax::camel(&name)));
        let Some(kind) = kind else {
            set_error(format!("unknown constraint type: {name}"));
            return std::ptr::null_mut();
        };
        let spec = kind.spec();
        let raw = v.get("args").cloned().unwrap_or(Json::Arr(Vec::new()));
        let raw = raw.arr();
        let mut args: Vec<Option<gcs_core::syntax::Arg>> = vec![None; spec.len()];
        for (i, (_, k)) in spec.iter().enumerate() {
            let Some(a) = raw.get(i) else { continue };
            if matches!(a, Json::Null) {
                continue; // left out: the core reads it off the geometry
            }
            args[i] = Some(if k.is_entity() {
                gcs_core::syntax::Arg::Ref(gcs_core::syntax::Ref::new(a.as_str().to_string()))
            } else if k.is_dimension() {
                gcs_core::syntax::Arg::Dim {
                    text: a.as_str().to_string(),
                    span: Default::default(),
                }
            } else {
                match a {
                    Json::Bool(b) => gcs_core::syntax::Arg::Bool(*b),
                    Json::Str(s) => gcs_core::syntax::Arg::Word(s.clone()),
                    Json::Int(n) => gcs_core::syntax::Arg::Int(*n),
                    other => gcs_core::syntax::Arg::Num(other.as_f64()),
                }
            });
        }
        out_edit(gcs_core::edit::add_relation(
            &(*h).program,
            // the app states constraints; a claim is written in the program panel
            gcs_core::syntax::Relation::of(gcs_core::syntax::RelationForm::Canonical {
                kind,
                args,
                bound: None,
            }),
        ))
    })
}

/// A selection as an edit is handed it: `{"entities": [["point", 3], ...], "constraints": [id,
/// ...]}`, each entity by kind and index.
fn selection_of(v: &Json) -> (Vec<EntRef>, Vec<u32>) {
    let ents: Vec<EntRef> = v
        .get("entities")
        .map(|a| {
            a.arr()
                .iter()
                .filter_map(|r| {
                    let p = r.arr();
                    (p.len() == 2)
                        .then(|| EntKind::parse(p[0].as_str()))
                        .flatten()
                        .map(|k| EntRef::new(k, p[1].as_i64() as usize))
                })
                .collect()
        })
        .unwrap_or_default();
    let cons: Vec<u32> = v
        .get("constraints")
        .map(|a| a.arr().iter().map(|x| x.as_i64() as u32).collect())
        .unwrap_or_default();
    (ents, cons)
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
        let (ents, cons) = selection_of(&as_json(ptr, len));
        // the live sketch, not the elaboration's: the binding took that one out into `s`
        let e = &*h;
        out_edit(gcs_core::edit::remove(e, &e.program, sk(s), &ents, &cons))
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
        let v = as_json(ptr, len);
        let (ents, cons) = selection_of(&v);
        let name = v.get("name").map(|n| n.as_str().to_string()).filter(|n| !n.is_empty());
        let e = &*h;
        let name = name.as_deref();
        out_edit(gcs_core::edit::make_component(e, &e.program, sk(s), &ents, &cons, name))
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
        let i = idx as usize;
        if i >= s.curves.len() {
            return -1;
        }
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
