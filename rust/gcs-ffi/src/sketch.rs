//! A sketch: making one, reading it whole, and the entities a binding draws into it.

use super::*;

/* -- sketch ---------------------------------------------------------------- */

#[no_mangle]
pub extern "C" fn gcs_sketch_new() -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(Sketch::new()))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_free(h: *mut Sketch) {
    guard((), move || {
        if !h.is_null() {
            drop(Box::from_raw(h));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_clone(h: *mut Sketch) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(sk(h).clone()))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_from_json(ptr: *const u8, len: usize) -> *mut Sketch {
    guard(std::ptr::null_mut(), move || {
        match io::loads(as_str(ptr, len)) {
            Ok(s) => Box::into_raw(Box::new(s)),
            Err(e) => {
                set_error(e);
                std::ptr::null_mut()
            }
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_to_json(h: *mut Sketch, indent: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let ind = if indent < 0 { None } else { Some(indent as usize) };
        out_str(io::dumps(sk(h), ind))
    })
}

/// The drawing as SVG, at a page width in pixels.
///
/// The writer is `gcs_core::svg`, in the core for the reason callout layout is: an "export SVG"
/// button and `solventc --output` must not be two implementations, or the same drawing would come
/// out of the two of them differently.  **An SVG has no screen, so the export chooses a `unit`**
/// — the world length of one screen pixel — from the page width, and every constant size follows.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_svg(h: *mut Sketch, width_px: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str(gcs_core::svg::render(sk(h), width_px)))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_preview(h: *mut Sketch, plane: i32) {
    guard((), move || gcs_core::drawing::preview(sk(h), index(plane)))
}

/// Render a .svd document from a host-supplied text bundle. JSON input:
/// {source, text, sheet?, files: {path: text}}. Result: {svg} or {error, offset}.
#[no_mangle]
pub unsafe extern "C" fn gcs_drawing_svg(ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let request = as_json(ptr, len);
        let text = request.get("text").map(Json::as_str).unwrap_or("");
        let source = request.get("source").map(Json::as_str).unwrap_or("drawing.svd");
        let sheet = request.get("sheet").filter(|s| matches!(s, Json::Str(_))).map(Json::as_str);
        let result = gcs_core::drawing::compile(text, source, sheet, &mut bundled(&request));
        let result = match result {
            Ok(svg) => json::object([("svg", Json::Str(svg))]),
            Err(e) => json::object([("error", Json::Str(e.message)), ("offset", Json::Int(e.span.lo as i64))]),
        };
        out_str(result.dump(None))
    })
}

/// A drawing's loader over a request's bundle (`files`, by path): a path read relative to the file
/// that names it, its identity that path, its text the bundle's.
pub(crate) fn bundled(request: &Json) -> impl FnMut(&str, &str) -> Option<(String, String)> + '_ {
    move |path, from| {
        let name = gcs_core::drawing::relative_path(path, from);
        request.get("files")?.get(&name).filter(|v| matches!(v, Json::Str(_)))
            .map(|v| (name, v.as_str().to_string()))
    }
}

/// Inspect .svd imports and sheet names without loading or solving a model.
#[no_mangle]
pub unsafe extern "C" fn gcs_drawing_info(ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let value = match gcs_core::drawing::parse(as_str(ptr, len)) {
            Ok(d) => json::object([
                ("models", Json::Arr(d.models.iter().map(|m| Json::Str(m.path.clone())).collect())),
                ("imports", Json::Arr(d.imports.into_iter().map(Json::Str).collect())),
                ("sheets", Json::Arr(d.sheets.into_iter().map(|s| Json::Str(s.name)).collect())),
            ]),
            Err(e) => json::object([("error", Json::Str(e.message))]),
        };
        out_str(value.dump(None))
    })
}

/// How many integers `gcs_sketch_counts` writes.
///
/// Asked rather than assumed: a binding that hard-codes the width writes past its buffer the day
/// a new entity kind is added, and the damage shows up as a crash somewhere else entirely.
#[no_mangle]
pub unsafe extern "C" fn gcs_counts_len() -> i32 {
    N_COUNTS as i32
}

const N_COUNTS: usize = 12;

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_counts(h: *mut Sketch, out: *mut i32) {
    guard((), move || {
        let s = sk(h);
        let v = [
            s.params.len(),
            s.points.len(),
            s.lines.len(),
            s.circles.len(),
            s.arcs.len(),
            s.constraints.len(),
            s.splines.len(),
            // slot 7 was the ellipses', until the ellipse became a library component
            // (#47, item 4), and slot 9 the frames', until `frame` was folded into `plane`
            0,
            s.curves.len(),
            // appended, never inserted: the positions above are what the bindings hard-code
            0,
            s.frames.len(),
            s.axes.len(),
        ];
        debug_assert_eq!(v.len(), N_COUNTS, "gcs_counts_len is what callers size their buffer by");
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x as i32;
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_point(
    h: *mut Sketch,
    x: f64,
    y: f64,
    fixed: i32,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).point(x, y, fixed != 0, as_str(name, name_len)) as i32
    })
}

/// A **point in space**, in no plane, where the eye at `az`, `el` (radians) sees `(x, y)` on its
/// picture plane (`workspace::unseen`): what a tool mints with no plane chosen.  Its index.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_point_seen(h: *mut Sketch, az: f64, el: f64, x: f64,
                                               y: f64) -> i32 {
    guard(-1, move || {
        let at = gcs_core::overview::workspace::unseen(az, el, (x, y));
        sk(h).point3(at, false, "") as i32
    })
}

/// **What the outline hides** (#435): `{ents: [[kind, idx]…], solids: [idx…]}`, exactly these
/// and nothing else — not drawn, lit or picked by the workspace, still solved
/// (`Sketch::set_hidden`).  View state, which the host re-applies to every drawing it is handed.
/// Returns 0.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_hidden(h: *mut Sketch, ptr: *const u8, len: usize) -> i32 {
    guard(-1, move || {
        let (ents, solids) = part_of(ptr, len);
        sk(h).set_hidden(ents, solids);
        0
    })
}

/// **The plane the workspace is sectioned by** (`Sketch::cut`): a plane's frame in the world,
/// nine doubles `u`, `v`, `o` (`gcs_plane_frame3`'s), moved `offset` along its normal and turned
/// to keep the other side where `flip` is nonzero (`Basis::moved_along`); a null `frame` takes the
/// section away.  View state, which the host re-applies to every drawing it is handed.  Returns 0,
/// or -1 for a frame that spans no plane.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_cut(h: *mut Sketch, frame: *const f64, flip: i32,
                                            offset: f64) -> i32 {
    guard(-1, move || {
        if frame.is_null() {
            sk(h).cut = None;
            return 0;
        }
        let Some(b) = frame_of(frame) else {
            set_error("a section's frame spans no plane");
            return -1;
        };
        sk(h).cut = Some(b.moved_along(offset, flip != 0));
        0
    })
}

/// The section as a renderer clips by: four doubles, a unit normal toward what the cut keeps and
/// a constant, so a point `p` is kept where `normal·p + constant` is positive — three.js's
/// `Plane`.  Returns 4, or 0 where the workspace is not sectioned.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_cut(h: *mut Sketch, out: *mut f64) -> i32 {
    guard(-1, move || {
        let Some(c) = sk(h).cut else { return 0 };
        let Some(n) = gcs_core::plane::unit(c.normal()) else { return 0 };
        let kept = n.map(|x| -x);
        let constant = gcs_core::plane::dot(n, c.o);
        write(out, &[kept[0], kept[1], kept[2], constant]);
        4
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_line(h: *mut Sketch, p1: i32, p2: i32) -> i32 {
    guard(-1, move || {
        sk(h).line(p1 as usize, p2 as usize) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_circle(
    h: *mut Sketch,
    center: i32,
    r: f64,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).circle(center as usize, r, as_str(name, name_len)) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_arc(
    h: *mut Sketch,
    center: i32,
    start: i32,
    end: i32,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h).arc(center as usize, start as usize, end as usize, as_str(name, name_len)) as i32
    })
}

/// A plane fixed where a basis `(u, v)` stands, at the origin, orthonormalised on the way in —
/// its two axes and its origin held.  -1 and an error when the two do not span a plane.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_sketch_plane(
    h: *mut Sketch,
    ux: f64,
    uy: f64,
    uz: f64,
    vx: f64,
    vy: f64,
    vz: f64,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        let Some(basis) = gcs_core::plane::Basis::explicit([ux, uy, uz], [vx, vy, vz]) else {
            set_error("u and v do not span a plane");
            return -1;
        };
        sk(h).fixed_plane(basis, as_str(name, name_len)) as i32
    })
}

/// Whether frame `idx` draws flat (#383): 1 for a plane, whose points have its two coordinates,
/// and 0 for a frame declared bare, whose points stand in space — along its axes where it has
/// them.  -1 where there is no such frame.
#[no_mangle]
pub unsafe extern "C" fn gcs_frame_planar(h: *mut Sketch, idx: i32) -> i32 {
    guard(-1, move || match sk(h).frames.get(idx as usize) {
        Some(f) => i32::from(f.planar()),
        None => -1,
    })
}

/// Whether frame `idx` moves (#383): 1 where it has a joint the document holds, 0 where not, -1
/// where there is no such frame.
#[no_mangle]
pub unsafe extern "C" fn gcs_frame_moves(h: *mut Sketch, idx: i32) -> i32 {
    guard(-1, move || match sk(h).frames.get(idx as usize) {
        Some(f) => i32::from(f.joint.as_ref().is_some_and(|j| j.motion.is_some())),
        None => -1,
    })
}

/// A plane's basis: six doubles, `u` then `v`.  Returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_plane_basis(h: *mut Sketch, idx: i32, out: *mut f64) -> i32 {
    guard(-1, move || {
        let b = sk(h).basis(idx as usize);
        for (i, x) in b.u.iter().chain(b.v.iter()).enumerate() {
            *out.add(i) = *x;
        }
        6
    })
}

/// A plane's whole frame in space: nine doubles, `u`, `v` and then its origin `o`, as the solve
/// left them (`Sketch::basis`) — six say how it turns, not where it stands.  Returns how many
/// were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_plane_frame3(h: *mut Sketch, idx: i32, out: *mut f64) -> i32 {
    guard(-1, move || {
        let b = sk(h).basis(idx as usize);
        for (i, x) in b.u.iter().chain(b.v.iter()).chain(b.o.iter()).enumerate() {
            *out.add(i) = *x;
        }
        9
    })
}

/// The datum glyph a plane is drawn as, in the plane's own coordinates: eight doubles, two
/// segments as `x1 y1 x2 y2` each — the arm along `u`, then the tick along `v` (`plane::glyph`).
/// Laid out by the core for a callout's reason, so the canvas and the SVG export stroke one
/// figure and not two.
#[no_mangle]
pub unsafe extern "C" fn gcs_plane_glyph(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    out: *mut f64,
) -> i32 {
    guard(-1, move || {
        let _ = (sk(h), idx);
        let g = gcs_core::plane::glyph(unit);
        for (k, (from, to)) in g.iter().enumerate() {
            *out.add(4 * k) = from.0;
            *out.add(4 * k + 1) = from.1;
            *out.add(4 * k + 2) = to.0;
            *out.add(4 * k + 3) = to.1;
        }
        8
    })
}

/// Which plane a point is drawn in: its index, or -1 for a point in space (or of a 2D sketch).
#[no_mangle]
pub unsafe extern "C" fn gcs_point_plane(h: *mut Sketch, idx: i32) -> i32 {
    guard(-1, move || sk(h).plane_of(idx as usize).map_or(-1, |p| p as i32))
}

/// Put a point on a plane (-1 takes it off).  0, or -1 with an error when the plane does not
/// exist — a document is untrusted, and so is a binding's index.
#[no_mangle]
pub unsafe extern "C" fn gcs_point_set_plane(h: *mut Sketch, idx: i32, plane: i32) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let plane = index(plane);
        if plane.is_some_and(|p| p >= s.frames.len()) {
            set_error("no such plane");
            return -1;
        }
        s.set_plane(idx as usize, plane);
        0
    })
}


/// A cubic B-spline over `n` control points, with the clamped uniform knot vector; -1 when
/// there are too few of them for a cubic.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_spline(h: *mut Sketch, ctrl: *const i32, n: usize) -> i32 {
    guard(-1, move || {
        let ids: Vec<usize> =
            std::slice::from_raw_parts(ctrl, n).iter().map(|&i| i as usize).collect();
        sk(h).spline(&ids).map(|i| i as i32).unwrap_or(-1)
    })
}

/// A cubic B-spline through `n` (x, y) pairs, in order — the control points are computed.
///
/// `hold` is `n` point indices, `-1` for a place that came from empty space; the rest become
/// `PointOnSpline` contacts pinned at the parameter the fit chose, so a curve fitted to
/// constrained points is itself fully constrained.  Pass null to hold none of them.
/// -1 when there are too few for a cubic, or they give no parameterisation.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_spline_through(
    h: *mut Sketch,
    pts: *const f64,
    n: usize,
    hold: *const i32,
) -> i32 {
    guard(-1, move || {
        let v = std::slice::from_raw_parts(pts, 2 * n);
        let q: Vec<(f64, f64)> = (0..n).map(|i| (v[2 * i], v[2 * i + 1])).collect();
        let held: Vec<Option<usize>> = if hold.is_null() {
            vec![None; n]
        } else {
            std::slice::from_raw_parts(hold, n)
                .iter()
                .map(|&i| index(i))
                .collect()
        };
        sk(h).spline_through_held(&q, &held).map(|i| i as i32).unwrap_or(-1)
    })
}

/// A cubic B-spline with a knot vector of its own; -1 when it does not fit the control polygon.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_spline_knots(
    h: *mut Sketch,
    ctrl: *const i32,
    n: usize,
    knots: *const f64,
    nk: usize,
) -> i32 {
    guard(-1, move || {
        let ids: Vec<usize> =
            std::slice::from_raw_parts(ctrl, n).iter().map(|&i| i as usize).collect();
        let ks = std::slice::from_raw_parts(knots, nk).to_vec();
        sk(h).spline_with(&ids, Some(ks)).map(|i| i as i32).unwrap_or(-1)
    })
}

/// A spline's knot vector; returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_knots(h: *mut Sketch, idx: i32, out: *mut f64) -> i32 {
    guard(-1, move || {
        let k = &sk(h).splines[idx as usize].knots;
        for (i, v) in k.iter().enumerate() {
            *out.add(i) = *v;
        }
        k.len() as i32
    })
}

/// The parameter interval a spline is drawn over (2 doubles).
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_domain(h: *mut Sketch, idx: i32, out: *mut f64) {
    guard((), move || {
        let (a, b) = curve::domain(sk(h), idx as usize);
        *out = a;
        *out.add(1) = b;
    })
}

/// C(t), C'(t) and C''(t) at one parameter (6 doubles).
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_eval(h: *mut Sketch, idx: i32, t: f64, out: *mut f64) {
    guard((), move || {
        let f = curve::eval(sk(h), idx as usize, t);
        let v = [f.p.0, f.p.1, f.d1.0, f.d1.1, f.d2.0, f.d2.1];
        for (i, x) in v.iter().enumerate() {
            *out.add(i) = *x;
        }
    })
}

/// The curve as a polyline, refined to `unit` (the world length of one screen pixel) exactly as
/// the callouts are: the core lays the figure out and the front end strokes what it is handed.
///
/// Returns how many points the curve *needs*, and writes x, y pairs for as many as `cap` allows.
/// A caller that guessed big enough is done in one call; one that did not sees a number larger
/// than its `cap` and calls again — which beats asking the length first, since that would
/// tessellate the curve twice on every call rather than on the rare miss.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_polyline(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let pts = curve::tessellate(sk(h), idx as usize, unit);
        for (i, p) in pts.iter().take(cap.max(0) as usize).enumerate() {
            *out.add(2 * i) = p.0;
            *out.add(2 * i + 1) = p.1;
        }
        pts.len() as i32
    })
}

/// Give the curve one more control point at `t`, without changing its shape — Boehm's knot
/// insertion.  Returns the new control Point's index, or -1 if `t` is not a place a knot can go.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_insert_control(h: *mut Sketch, idx: i32, t: f64) -> i32 {
    guard(-1, move || {
        curve::insert_control(sk(h), idx as usize, t).map(|p| p as i32).unwrap_or(-1)
    })
}

/// The parameter of the curve point nearest (x, y), and how far that is (2 doubles) — the pick
/// test, so a front end never converts a tolerance or evaluates a basis function.
#[no_mangle]
pub unsafe extern "C" fn gcs_spline_closest(
    h: *mut Sketch,
    idx: i32,
    x: f64,
    y: f64,
    out: *mut f64,
) {
    guard((), move || {
        let (t, d) = curve::closest(sk(h), idx as usize, x, y);
        *out = t;
        *out.add(1) = d;
    })
}

/// The three-point construction; -1 when the three points are collinear.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_arc_through(
    h: *mut Sketch,
    start: i32,
    end: i32,
    tx: f64,
    ty: f64,
    name: *const u8,
    name_len: usize,
) -> i32 {
    guard(-1, move || {
        sk(h)
            .arc_through(start as usize, end as usize, (tx, ty), as_str(name, name_len))
            .map(|i| i as i32)
            .unwrap_or(-1)
    })
}

/// Four lines and three perpendiculars; writes the line indices into `out` (4 ints).
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_rectangle(
    h: *mut Sketch,
    a: i32,
    x1: f64,
    y1: f64,
    name: *const u8,
    name_len: usize,
    out: *mut i32,
) {
    guard((), move || {
        let lines = sk(h).rectangle(a as usize, x1, y1, as_str(name, name_len));
        for (i, l) in lines.iter().enumerate() {
            *out.add(i) = *l as i32;
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_get_x(h: *mut Sketch, out: *mut f64) {
    guard((), move || {
        let s = sk(h);
        for (i, p) in s.params.iter().enumerate() {
            *out.add(i) = p.value;
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_set_x(h: *mut Sketch, x: *const f64, n: usize) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        if s.set_x(std::slice::from_raw_parts(x, n)) {
            0
        } else {
            set_error(format!("set_x: {n} values for {} params", s.params.len()));
            -1
        }
    })
}

/// **Where the sketch's meshes were last made again** (`Sketch::meshed_at`, #510 S2), written to
/// `out` as `gcs_sketch_get_x` writes its pose: 1, or 0 and nothing written while each is where
/// elaboration made it.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_meshed_at(h: *mut Sketch, out: *mut f64) -> i32 {
    guard(0, move || {
        let Some(x) = &sk(h).meshed_at else { return 0 };
        write(out, x);
        1
    })
}

/// **The sketch's meshes made where another drawing of its source stood** (`Sketch::mesh_at`):
/// 0, or −1 for a pose of another length.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_mesh_at(h: *mut Sketch, x: *const f64, n: usize) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        if s.mesh_at(std::slice::from_raw_parts(x, n)) {
            0
        } else {
            set_error(format!("mesh_at: {n} values for {} params", s.params.len()));
            -1
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_perturb(h: *mut Sketch, sigma: f64, seed: u32) {
    guard((), move || {
        sk(h).perturb(sigma, seed);
    })
}

/// What a compiled plan or System depends on — entity counts, constraint ids and types, fixed
/// flags.  A front end caching a compiled artefact keys on this.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_topology_key(h: *mut Sketch) -> *mut u8 {
    guard(std::ptr::null_mut(), move || out_str(sk(h).topology_key()))
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_extent(h: *mut Sketch) -> f64 {
    guard(f64::NAN, move || {
        sk(h).extent()
    })
}

/// `bbox` when `drawn == 0`, `drawn_bounds` otherwise; writes 4 doubles.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_bounds(h: *mut Sketch, drawn: i32, out: *mut f64) {
    guard((), move || {
        let s = sk(h);
        let b = if drawn != 0 { s.drawn_bounds() } else { s.bbox() };
        *out = b.0;
        *out.add(1) = b.1;
        *out.add(2) = b.2;
        *out.add(3) = b.3;
    })
}

/// Index of the nearest point and its distance (index -1 when the sketch has none).
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_nearest_point(
    h: *mut Sketch,
    x: f64,
    y: f64,
    out_dist: *mut f64,
) -> i32 {
    guard(-1, move || {
        let (i, d) = sk(h).nearest_point(x, y);
        *out_dist = d;
        i.map(|v| v as i32).unwrap_or(-1)
    })
}

/// What a click at (x, y) picks within `tol` (a world length): writes [kind, index] and
/// returns 1, or returns 0 when nothing drawn is within reach.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_pick(
    h: *mut Sketch,
    x: f64,
    y: f64,
    tol: f64,
    out: *mut f64,
) -> i32 {
    guard(0, move || match model::pick(sk(h), x, y, tol) {
        Some(e) => {
            write(out, &[kind_id(e.kind) as f64, e.i() as f64]);
            1
        }
        None => 0,
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_n_residuals(h: *mut Sketch) -> i32 {
    guard(-1, move || {
        sk(h).n_residuals() as i32
    })
}
