//! The pictures: the overview, the derived views, the workspace every view stands in and the
//! callouts laid out on it.

use super::*;

/// The 3D overview: the glass box the views were unfolded from, and the object they are of,
/// projected to 2D world coordinates.  `unit` is the world length of one screen pixel; `az` and
/// `el` are the orbit in radians; `shaded` asks for the solid's surfaces as well as its edges,
/// back-face culled and handed over far first.  JSON for `gcs_callouts_json`'s reason — the scene is
/// heterogeneous records, not a fixed-width block of numbers.
#[no_mangle]
pub unsafe extern "C" fn gcs_overview_json(
    h: *mut Sketch,
    unit: f64,
    az: f64,
    el: f64,
    shaded: i32,
) -> *mut u8 {
    json_out(move || {
        report::overview_json(sk(h), unit, az, el, shaded != 0)
    })
}

/// **The pictures the document asked for** (§6.11): every `view` and `section`, laid out in page
/// coordinates with its ink resolved.  `unit` is the world length of one screen pixel, which is
/// what refines a round surface to the zoom it is looked at.
#[no_mangle]
pub unsafe extern "C" fn gcs_derived_json(h: *mut Sketch, unit: f64) -> *mut u8 {
    json_out(move || report::derived_json(sk(h), unit))
}

/// 1 where the projected pictures are the same at every pixel length (`renderer::detail_free`):
/// a host need not refine them once a zoom rests.
#[no_mangle]
pub unsafe extern "C" fn gcs_derived_detail_free(h: *mut Sketch) -> i32 {
    guard(0, move || gcs_core::renderer::detail_free(sk(h)) as i32)
}

/// Numeric projection inputs, for retaining an unchanged picture during a drag. Returns the
/// required number of doubles and writes as many as `cap` permits, like a curve polyline.
#[no_mangle]
pub unsafe extern "C" fn gcs_derived_inputs(h: *mut Sketch, out: *mut f64, cap: i32) -> i32 {
    guard(-1, move || {
        let values = gcs_core::renderer::inputs(sk(h));
        write(out, &values[..values.len().min(cap.max(0) as usize)]);
        values.len() as i32
    })
}

/// **The overview in space**: the glass box's panes, axes and drawn geometry as 3D polylines,
/// with no orbit applied.  A front end with a depth buffer takes this and the solids' meshes and
/// does its own projecting; one without takes `gcs_overview_json`, which flattens.  `layer` 0 is
/// the whole scene, 1 the datums (panes, their axes, axes in space), 2 the objects' creases.
#[no_mangle]
pub unsafe extern "C" fn gcs_overview3d_json(h: *mut Sketch, unit: f64, layer: i32) -> *mut u8 {
    use gcs_core::overview::Layer;
    let layer = match layer { 1 => Layer::Datums, 2 => Layer::Objects, _ => Layer::All };
    json_out(move || report::overview3d_json(sk(h), unit, layer))
}

/* -- the workspace: every view standing on its plane, seen by one orthographic eye --------- */

/// The sketch behind `h` and the eye at bearing `az`, elevation `el` that sees it: where every
/// question asked where the eye sees things starts (`workspace::Projection`).
unsafe fn seen<'a>(h: *mut Sketch, az: f64, el: f64)
    -> (&'a Sketch, gcs_core::overview::workspace::Projection) {
    let s: &Sketch = sk(h);
    (s, gcs_core::overview::workspace::Projection::new(s, az, el))
}

/// What of the workspace does not depend on the eye (`overview::workspace::Views`): each view's
/// place and the eye square on to it, and the view every point and drawn entity stands in.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || report::workspace_json(sk(h)))
}

/// Each view's map of its page onto the eye's picture plane at bearing `az` and elevation `el`
/// (radians), six doubles each, the page first: writes at most `cap` views and returns how many
/// there are.  Asked per frame, so a buffer and not JSON.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_maps(h: *mut Sketch, az: f64, el: f64, out: *mut f64,
                                            cap: usize) -> i32 {
    guard(-1, move || {
        let maps = gcs_core::overview::workspace::maps(sk(h), az, el);
        for (k, m) in maps.iter().take(cap).enumerate() {
            write(out.add(6 * k), m);
        }
        maps.len() as i32
    })
}

/// Where the eye at `az`, `el` sees each point in space: [x, y] per point, by index, NaN for a
/// point drawn in a plane (its view's map places it).  Returns the point count; writes at most
/// `cap` points.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_space_points(h: *mut Sketch, az: f64, el: f64, out: *mut f64,
                                                    cap: usize) -> i32 {
    guard(-1, move || {
        let seen = gcs_core::overview::workspace::space_points(sk(h), az, el);
        for (k, s) in seen.iter().take(cap).enumerate() {
            let (x, y) = s.unwrap_or((f64::NAN, f64::NAN));
            write(out.add(2 * k), &[x, y]);
        }
        seen.len() as i32
    })
}

/// What a click at (x, y) on the eye's picture plane picks within `tol` (an eye length): writes
/// [kind, index] and returns 1, or returns 0.  `unit` is the eye length of one screen pixel.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_pick(h: *mut Sketch, unit: f64, az: f64, el: f64,
                                            x: f64, y: f64, tol: f64, out: *mut f64) -> i32 {
    guard(0, move || {
        let (s, proj) = seen(h, az, el);
        match gcs_core::overview::workspace::pick(s, &proj, (x, y), tol, unit) {
            Some(e) => {
                write(out, &[kind_id(e.kind) as f64, e.i() as f64]);
                1
            }
            None => 0,
        }
    })
}

/// The object face the eye's ray through (x, y) meets nearest the viewer: `{solid, face, depth}`,
/// or `null` — `workspace::pick_solid`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_pick_solid_json(h: *mut Sketch, az: f64, el: f64,
                                                       x: f64, y: f64) -> *mut u8 {
    json_out(move || {
        let (s, proj) = seen(h, az, el);
        match gcs_core::overview::workspace::pick_solid(s, &proj, (x, y)) {
            Some(hit) => json::object([
                ("solid", Json::Int(hit.solid as i64)),
                ("face", Json::Str(hit.face)),
                ("depth", Json::Num(hit.depth)),
                ("at", Json::Arr(hit.at.iter().map(|&x| Json::Num(x)).collect())),
            ]),
            None => Json::Null,
        }
    })
}

/// Where two faces of object `solid` meet, in space: `[{faces: [a, b], segs: [[ax, ay, az, bx,
/// by, bz]…]}…]` — `workspace::solid_edges`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_solid_edges_json(h: *mut Sketch, solid: i32) -> *mut u8 {
    json_out(move || {
        let edges = index(solid).map_or(Vec::new(), |i| {
            gcs_core::overview::workspace::solid_edges(sk(h), i)
        });
        Json::Arr(edges.into_iter().map(|e| json::object([
            ("faces", Json::Arr(vec![Json::Str(e.faces.0), Json::Str(e.faces.1)])),
            ("segs", Json::Arr(e.segs.iter().map(|[a, b]| {
                Json::Arr(a.iter().chain(b).map(|&x| Json::Num(x)).collect())
            }).collect())),
        ])).collect())
    })
}

/// The size a round or chamfer on object `solid` starts from, or NaN for none —
/// `edit::round_size`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_round_size(h: *mut Sketch, solid: i32) -> f64 {
    guard(f64::NAN, move || {
        let size = index(solid).and_then(|i| gcs_core::edit::round_size(sk(h), i));
        size.unwrap_or(f64::NAN)
    })
}

/// The edge of an object the eye sees nearest (x, y), within `tol` on the picture plane:
/// `{solid, faces: [a, b], at: [x, y, z]}`, or `null` — `workspace::pick_edge`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_pick_edge_json(h: *mut Sketch, az: f64, el: f64,
                                                      x: f64, y: f64, tol: f64) -> *mut u8 {
    json_out(move || {
        let (s, proj) = seen(h, az, el);
        match gcs_core::overview::workspace::pick_edge(s, &proj, (x, y), tol) {
            Some(hit) => json::object([
                ("solid", Json::Int(hit.solid as i64)),
                ("faces", Json::Arr(vec![Json::Str(hit.faces.0), Json::Str(hit.faces.1)])),
                ("at", Json::Arr(hit.at.iter().map(|&x| Json::Num(x)).collect())),
            ]),
            None => Json::Null,
        }
    })
}

/// A point on the eye's picture plane, `[x, y]`.
fn pt(p: (f64, f64)) -> Json {
    Json::Arr(vec![Json::Num(p.0), Json::Num(p.1)])
}

/// The arrow prism `solid` is sized by, seen by the eye at `az`, `el`: `{base: [x, y], tip: [x, y]
/// | null, way: "behind" | "front" | "both" | null, thick}` — no tip seen straight down its
/// normal — or `null` for no prism.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_extrude_handle_json(h: *mut Sketch, az: f64, el: f64,
                                                           solid: i32) -> *mut u8 {
    json_out(move || {
        use gcs_core::overview::workspace::extrude_handle;
        let (s, proj) = seen(h, az, el);
        match solid_at(s, solid).and_then(|i| extrude_handle(s, &proj, i)) {
            Some(k) => json::object([
                ("base", pt(k.base)),
                ("tip", k.tip.map_or(Json::Null, pt)),
                ("way", k.way.map_or(Json::Null, |w| Json::Str(w.as_str().into()))),
                ("thick", Json::Num(k.thick)),
            ]),
            None => Json::Null,
        }
    })
}

/// How far along prism `solid`'s normal the point under (x, y) on the eye's picture plane is —
/// what dragging its arrow there sizes it to — or NaN: `workspace::extent_at`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_extent_at(h: *mut Sketch, az: f64, el: f64, solid: i32,
                                                 x: f64, y: f64) -> f64 {
    guard(f64::NAN, move || {
        use gcs_core::overview::workspace::extent_at;
        let (s, proj) = seen(h, az, el);
        solid_at(s, solid).and_then(|i| extent_at(s, &proj, i, (x, y))).unwrap_or(f64::NAN)
    })
}

/// The handle revolution `solid` is sized by, seen by the eye at `az`, `el`: `{base: [x, y], arc:
/// [[x, y], …], tip: [x, y] | null, turn}` — the arc its profile's middle turns through, no tip
/// seen along the turning plane, `turn` in degrees, positive right-handed about the axis — or
/// `null` for no revolution: `workspace::turn_handle`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_turn_handle_json(h: *mut Sketch, az: f64, el: f64,
                                                        solid: i32) -> *mut u8 {
    json_out(move || {
        use gcs_core::overview::workspace::turn_handle;
        let (s, proj) = seen(h, az, el);
        match solid_at(s, solid).and_then(|i| turn_handle(s, &proj, i)) {
            Some(k) => json::object([
                ("base", pt(k.base)),
                ("arc", Json::Arr(k.arc.into_iter().map(pt).collect())),
                ("tip", k.tip.map_or(Json::Null, pt)),
                ("turn", Json::Num(k.turn)),
            ]),
            None => Json::Null,
        }
    })
}

/// How far revolution `solid` turns, in signed degrees, where its handle is dragged to (x, y) on
/// the eye's picture plane, the whole turns round nearest `near` — or NaN: `workspace::turn_at`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_turn_at(h: *mut Sketch, az: f64, el: f64, solid: i32,
                                               x: f64, y: f64, near: f64) -> f64 {
    guard(f64::NAN, move || {
        use gcs_core::overview::workspace::turn_at;
        let (s, proj) = seen(h, az, el);
        solid_at(s, solid).and_then(|i| turn_at(s, &proj, i, (x, y), near)).unwrap_or(f64::NAN)
    })
}

/// The arrow the plane distance `id` is sized by, seen by the eye at `az`, `el`: `{base: [x, y],
/// tip: [x, y] | null, distance}` — or `null` for any other constraint: `workspace::offset_handle`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_offset_handle_json(h: *mut Sketch, az: f64, el: f64,
                                                          id: u32) -> *mut u8 {
    json_out(move || {
        use gcs_core::overview::workspace::offset_handle;
        let (s, proj) = seen(h, az, el);
        match offset_handle(s, &proj, id) {
            Some(k) => json::object([
                ("base", pt(k.base)),
                ("tip", k.tip.map_or(Json::Null, pt)),
                ("distance", Json::Num(k.distance)),
            ]),
            None => Json::Null,
        }
    })
}

/// What dragging the plane distance `id`'s arrow to (x, y) sizes it to, or NaN:
/// `workspace::offset_at`.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_offset_at(h: *mut Sketch, az: f64, el: f64, id: u32,
                                                 x: f64, y: f64) -> f64 {
    guard(f64::NAN, move || {
        use gcs_core::overview::workspace::offset_at;
        let (s, proj) = seen(h, az, el);
        offset_at(s, &proj, id, (x, y)).unwrap_or(f64::NAN)
    })
}

/// The planes whose panes (x, y) on the eye's picture plane falls inside, nearest the eye first:
/// writes up to `cap` plane indices and returns how many there are.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_panes_at(h: *mut Sketch, az: f64, el: f64, x: f64, y: f64,
                                                out: *mut f64, cap: usize) -> i32 {
    guard(-1, move || {
        let (s, proj) = seen(h, az, el);
        let hits = gcs_core::overview::workspace::panes_at(s, &proj, (x, y));
        for (k, &i) in hits.iter().take(cap).enumerate() {
            write(out.add(k), &[i as f64]);
        }
        hits.len() as i32
    })
}

/// The point nearest (x, y) on the eye's picture plane and how far it is (-1 when there is none).
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_nearest_point(h: *mut Sketch, az: f64, el: f64,
                                                     x: f64, y: f64, out_dist: *mut f64) -> i32 {
    guard(-1, move || {
        let (s, proj) = seen(h, az, el);
        let (i, d) = gcs_core::overview::workspace::nearest_point(s, &proj, (x, y));
        *out_dist = d;
        i.map_or(-1, |v| v as i32)
    })
}

/// The extent of everything the workspace shows on the eye's picture plane: writes (xmin, ymin,
/// xmax, ymax) and returns 1, or 0 when nothing is drawn.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_bounds(h: *mut Sketch, unit: f64, az: f64, el: f64,
                                              out: *mut f64) -> i32 {
    guard(0, move || {
        let (s, proj) = seen(h, az, el);
        match gcs_core::overview::workspace::bounds(s, &proj, unit) {
            Some((x0, y0, x1, y1)) => {
                write(out, &[x0, y0, x1, y1]);
                1
            }
            None => 0,
        }
    })
}

/// The entities a rubber band from (x0, y0) to (x1, y1) on the eye's picture plane touches.
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_overlapping_json(h: *mut Sketch, unit: f64, az: f64, el: f64,
                                                             x0: f64, y0: f64, x1: f64, y1: f64) -> *mut u8 {
    json_out(move || {
        report::workspace_overlapping_json(sk(h), unit, az, el, (x0, y0), (x1, y1))
    })
}

/// The dimension whose callout (x, y) on the eye's picture plane lands on, within `tol_px`
/// screen pixels, or -1 — and the view its figure is laid out in, written to `out_view` (-1 the
/// page, else the plane's index).
#[no_mangle]
pub unsafe extern "C" fn gcs_workspace_callout_pick(h: *mut Sketch, unit: f64, az: f64, el: f64,
                                                    x: f64, y: f64, tol_px: f64,
                                                    out_view: *mut i32) -> i32 {
    guard(-1, move || {
        let (s, proj) = seen(h, az, el);
        match callout::pick_seen(s, unit, &proj, (x, y), tol_px) {
            Some((id, view)) => {
                *out_view = view.map_or(-1, |v| v as i32);
                id as i32
            }
            None => -1,
        }
    })
}

/// The dimension callouts for the whole sketch.  `unit` is the world length of one screen pixel;
/// the layout is screen-constant through it.
#[no_mangle]
pub unsafe extern "C" fn gcs_callouts_json(h: *mut Sketch, unit: f64) -> *mut u8 {
    json_out(move || {
        report::callouts_json(sk(h), unit)
    })
}

/// Explicit constraint annotations for a model editor's active dimension operation.
#[no_mangle]
pub unsafe extern "C" fn gcs_selected_callouts_json(h: *mut Sketch, unit: f64,
    ids: *const u32, count: usize) -> *mut u8
{
    json_out(move || {
        let ids = if count == 0 { &[] } else { std::slice::from_raw_parts(ids, count) };
        report::selected_callouts_json(sk(h), unit, Some(ids))
    })
}

/// Take hold of dimension `id`'s callout at the world point (x, y): writes the two numbers the
/// caller hands back to `gcs_callout_drag` for the rest of the gesture, so the callout moves with
/// the pointer rather than jumping to it.  0 if the dimension has no callout to grab.
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_grab(h: *mut Sketch, id: i32, unit: f64, x: f64, y: f64,
                                          out: *mut f64) -> i32 {
    guard(0, move || match callout::grab(sk(h), unit, id as u32, (x, y)) {
        Some((a, b)) => {
            let o = std::slice::from_raw_parts_mut(out, 2);
            o[0] = a;
            o[1] = b;
            1
        }
        None => 0,
    })
}

/// Move dimension `id`'s callout so the point it was grabbed at follows the pointer to (x, y).
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_drag(h: *mut Sketch, id: i32, x: f64, y: f64,
                                          gu: f64, gv: f64) -> i32 {
    guard(0, move || callout::drag(sk(h), id as u32, (x, y), (gu, gv)) as i32)
}

/// Which of the three dimensions between two points a callout dropped at (px, py) states: 0 the
/// length (`Distance`), 1 the run between them and 2 the rise — an `Ordinate` along the view's
/// `x` or `y` (`callout::PairDimension`).
#[no_mangle]
pub unsafe extern "C" fn gcs_dimension_pair_kind(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    px: f64,
    py: f64,
) -> i32 {
    guard(-1, move || {
        match callout::pair_dimension((ax, ay), (bx, by), (px, py)) {
            callout::PairDimension::Length => 0,
            callout::PairDimension::Run => 1,
            callout::PairDimension::Rise => 2,
        }
    })
}

/// Put dimension `id`'s callout back wherever the layout would have put it; 1 if it had been
/// moved at all.
#[no_mangle]
pub unsafe extern "C" fn gcs_callout_reset(h: *mut Sketch, id: i32) -> i32 {
    guard(0, move || callout::reset(sk(h), id as u32) as i32)
}

#[no_mangle]
pub extern "C" fn gcs_fmt_g(v: f64, sig: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(json::fmt_g(v, sig.max(1) as usize))
    })
}
