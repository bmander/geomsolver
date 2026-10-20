//! Drags: a point, a radius, a plan's, and a body turned whole.

use super::*;

/* -- drags ----------------------------------------------------------------- */

unsafe fn guards_from(ptr: *const i32, n: i32) -> Vec<Triangle> {
    if ptr.is_null() || n <= 0 {
        return Vec::new();
    }
    let v = std::slice::from_raw_parts(ptr, 3 * n as usize);
    (0..n as usize)
        .map(|i| (v[3 * i] as usize, v[3 * i + 1] as usize, v[3 * i + 2] as usize))
        .collect()
}

#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_drag_new(
    h: *mut Sketch,
    point: i32,
    x: f64,
    y: f64,
    method: i32,
    weight: f64,
    guards: *const i32,
    n_guards: i32,
    max_step_rel: f64,
) -> *mut Drag {
    guard(std::ptr::null_mut(), move || {
        let m = if method == 1 { Method::Lm } else { Method::DogLeg };
        Box::into_raw(Box::new(Drag::new(
            sk(h),
            point as usize,
            x,
            y,
            m,
            weight,
            guards_from(guards, n_guards),
            max_step_rel,
        )))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_move(
    d: *mut Drag,
    h: *mut Sketch,
    x: f64,
    y: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = (*d).move_to(sk(h), x, y);
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_end(d: *mut Drag, h: *mut Sketch) {
    guard((), move || {
        (*d).end(sk(h));
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_free(d: *mut Drag) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_drag_flips(d: *mut Drag) -> i32 {
    guard(-1, move || {
        (*d).flips.len() as i32
    })
}

/// The triangles whose orientation flipped, 3 point indices each; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_drag_flip_list(d: *mut Drag, out: *mut i32) -> i32 {
    guard(-1, move || {
        let f = &(*d).flips;
        for (i, t) in f.iter().enumerate() {
            *out.add(3 * i) = t.0 as i32;
            *out.add(3 * i + 1) = t.1 as i32;
            *out.add(3 * i + 2) = t.2 as i32;
        }
        f.len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_new(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    r: f64,
    method: i32,
) -> *mut RadiusDrag {
    guard(std::ptr::null_mut(), move || {
        let m = if method == 1 { Method::Lm } else { Method::DogLeg };
        Box::into_raw(Box::new(RadiusDrag::new(sk(h), ent(kind, idx), r, m)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_move(
    d: *mut RadiusDrag,
    h: *mut Sketch,
    r: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let res = (*d).move_to(sk(h), r);
        write_result(&res, out);
        out_str(res.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_end(d: *mut RadiusDrag, h: *mut Sketch) {
    guard((), move || {
        (*d).end(sk(h));
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_radius_drag_free(d: *mut RadiusDrag) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

/// **A body turned whole** (a shift-drag): `body` is the selection, JSON `[[kind, index], …]`.
/// It turns in space or in a plane, which `gcs_turn_drag_plane` says.
#[no_mangle]
pub unsafe extern "C" fn gcs_turn_drag_new(h: *mut Sketch, body: *const u8, len: usize)
    -> *mut TurnDrag {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(TurnDrag::new(sk(h), &ent_list(body, len))))
    })
}

/// Where the body turns: -2 in space, -1 on a 2D sketch's page, else the plane it is drawn in.
#[no_mangle]
pub unsafe extern "C" fn gcs_turn_drag_plane(d: *mut TurnDrag) -> i32 {
    guard(-3, move || match (*d).turning {
        Turning::InSpace => -2,
        Turning::InPlane(p) => p.map_or(-1, |p| p as i32),
    })
}

/// How many free numbers the turn carries: 0, and it can turn nothing.
#[no_mangle]
pub unsafe extern "C" fn gcs_turn_drag_carries(d: *mut TurnDrag) -> i32 {
    guard(0, move || (*d).carries as i32)
}

/// Turned on in its plane by the angle the pointer swept about the pivot from (`fx`, `fy`) to
/// (`tx`, `ty`), both on the plane's page.
#[no_mangle]
pub unsafe extern "C" fn gcs_turn_drag_sweep(
    d: *mut TurnDrag,
    h: *mut Sketch,
    fx: f64,
    fy: f64,
    tx: f64,
    ty: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = (*d).sweep(sk(h), (fx, fy), (tx, ty));
        write_result(&r, out);
        out_str(r.message)
    })
}

/// Turned on in space, seen by the eye at bearing `az` and elevation `el` (radians): `across`
/// radians about the eye's up, `down` about its right.
#[no_mangle]
pub unsafe extern "C" fn gcs_turn_drag_seen(
    d: *mut TurnDrag,
    h: *mut Sketch,
    az: f64,
    el: f64,
    across: f64,
    down: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let r = (*d).turn_seen(sk(h), (az, el), across, down);
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_turn_drag_free(d: *mut TurnDrag) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

/// A `PlanDrag` and the plan it was made on, if any: the caller keeps that plan alive for as long
/// as the drag, and it comes back to the core with every call.
pub struct PlanDragH {
    d: PlanDrag,
    ps: *mut PlanSolver,
}

/// The plan the drag was made on, handed back to the core with every call.
unsafe fn plan_of<'a>(h: *mut PlanDragH) -> Option<&'a Plan> {
    (*h).ps.as_ref().map(|p| &p.plan)
}

/// `ps` may be null: the drag then makes a plan of its own over the dragged point's part.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_plan_drag_new(
    h: *mut Sketch,
    ps: *mut PlanSolver,
    point: i32,
    x: f64,
    y: f64,
    guards: *const i32,
    n_guards: i32,
    max_step_rel: f64,
) -> *mut PlanDragH {
    guard(std::ptr::null_mut(), move || {
        let g = if n_guards < 0 { None } else { Some(guards_from(guards, n_guards)) };
        let d = if ps.is_null() {
            PlanDrag::new(sk(h), point as usize, x, y, g, max_step_rel)
        } else {
            PlanDrag::on(sk(h), &mut *ps, point as usize, x, y, g, max_step_rel)
        };
        Box::into_raw(Box::new(PlanDragH { d, ps }))
    })
}

/// A drag of a point in space, read where the eye at bearing `az` and elevation `el` (radians)
/// sees the pointer: (`x`, `y`) and every later move are on its picture plane.  `ps` as above.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn gcs_plan_drag_seen(
    h: *mut Sketch,
    ps: *mut PlanSolver,
    point: i32,
    x: f64,
    y: f64,
    az: f64,
    el: f64,
    max_step_rel: f64,
) -> *mut PlanDragH {
    guard(std::ptr::null_mut(), move || {
        let d = PlanDrag::seen(sk(h), ps.as_mut(), point as usize, x, y, (az, el), max_step_rel);
        Box::into_raw(Box::new(PlanDragH { d, ps }))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_move(
    d: *mut PlanDragH,
    h: *mut Sketch,
    x: f64,
    y: f64,
    out: *mut f64,
) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let plan = plan_of(d);
        let r = (*d).d.move_to(sk(h), plan, x, y);
        write_result(&r, out);
        out_str(r.message)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_usable(d: *mut PlanDragH) -> i32 {
    guard(-1, move || {
        (*d).d.usable() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_flips(d: *mut PlanDragH) -> i32 {
    guard(-1, move || {
        (*d).d.flips().len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_flip_list(d: *mut PlanDragH, out: *mut i32) -> i32 {
    guard(-1, move || {
        let f = (*d).d.flips();
        for (i, t) in f.iter().enumerate() {
            *out.add(3 * i) = t.0 as i32;
            *out.add(3 * i + 1) = t.1 as i32;
            *out.add(3 * i + 2) = t.2 as i32;
        }
        f.len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_branches_json(d: *mut PlanDragH) -> *mut u8 {
    json_out(move || {
        let plan = plan_of(d);
        let b: BTreeMap<String, i32> = (*d).d.branches(plan);
        Json::Obj(b.into_iter().map(|(k, v)| (k, Json::Int(v as i64))).collect())
    })
}

/// The order-type triangles a numeric fallback would guard (3 point indices each).
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_guards(
    d: *mut PlanDragH,
    _h: *mut Sketch,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let plan = plan_of(d);
        let g = (*d).d.guard_triangles(plan);
        for (i, t) in g.iter().enumerate() {
            *out.add(3 * i) = t.0 as i32;
            *out.add(3 * i + 1) = t.1 as i32;
            *out.add(3 * i + 2) = t.2 as i32;
        }
        g.len() as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_end(d: *mut PlanDragH, _h: *mut Sketch) {
    guard((), move || {
        (*d).d.end();
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_drag_free(d: *mut PlanDragH) {
    guard((), move || {
        if !d.is_null() {
            drop(Box::from_raw(d));
        }
    })
}

/// The closed-form triangles of a plan (3 point indices each); returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_ppp_triangles(p: *mut PlanSolver, out: *mut i32) -> i32 {
    guard(-1, move || {
        let t = decompose::ppp_triangles(&(*p).plan);
        for (i, x) in t.iter().enumerate() {
            *out.add(3 * i) = x.0 as i32;
            *out.add(3 * i + 1) = x.1 as i32;
            *out.add(3 * i + 2) = x.2 as i32;
        }
        t.len() as i32
    })
}
