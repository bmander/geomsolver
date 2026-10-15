//! Parameters and entities: their numbers, their styles, and the spatial ones sampled.

use super::*;

/* -- params ---------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_param_value(h: *mut Sketch, i: i32) -> f64 {
    guard(f64::NAN, move || {
        param(sk(h), i).value
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_set_value(h: *mut Sketch, i: i32, v: f64) {
    guard((), move || {
        param_mut(sk(h), i).value = v;
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_fixed(h: *mut Sketch, i: i32) -> i32 {
    guard(-1, move || {
        param(sk(h), i).fixed as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_set_fixed(h: *mut Sketch, i: i32, fixed: i32) {
    guard((), move || {
        param_mut(sk(h), i).fixed = fixed != 0;
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_param_name(h: *mut Sketch, i: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(param(sk(h), i).name.clone())
    })
}

/* -- entities -------------------------------------------------------------- */

/// Generating profile bounds before revolution: xyz min/max then du min/max, twelve doubles.
#[no_mangle]
pub unsafe extern "C" fn gcs_surface_profile_bounds(h: *mut Sketch,idx: i32,
    u_min: f64,u_max: f64,out: *mut f64) -> i32 {
    guard(0,move || {
        let result = gcs_core::solid::RevolvedSurface::named(sk(h),idx as usize).and_then(|s| {
            let u = gcs_core::interval::Interval::new(u_min,u_max)
                .map_err(|e| format!("invalid profile interval: {e:?}"))?;
            s.generating_profile_bounds(u).map_err(|e| format!("invalid profile bounds: {e:?}"))
        });
        match result {
            Ok((p,d)) => {
                for (i,v) in p.into_iter().chain(d).flat_map(|x| x.bounds()).enumerate() { out.add(i).write(v); }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}

/// A named surface's original-source parameter domain: u_min, u_max, v_min, v_max.
#[no_mangle]
pub unsafe extern "C" fn gcs_surface_domain(h: *mut Sketch,idx: i32,out: *mut f64) -> i32 {
    guard(0,move || match gcs_core::solid::RevolvedSurface::named(sk(h),idx as usize) {
        Ok(s) => {
            for (i,v) in s.domain().into_iter().flatten().enumerate() { out.add(i).write(v); }
            1
        }
        Err(message) => { set_error(message); 0 }
    })
}

/// A named envelope's domain, six doubles: source u bounds, v bounds, and roll radians.
#[no_mangle]
pub unsafe extern "C" fn gcs_envelope_domain(h: *mut Sketch,idx: i32,out: *mut f64) -> i32 {
    guard(0,move || match gcs_core::envelope::GeneratedEnvelope::named(sk(h),idx as usize) {
        Ok(s) => {
            for (i,v) in s.domain().into_iter().flatten().enumerate() { out.add(i).write(v); }
            1
        }
        Err(message) => { set_error(message); 0 }
    })
}

/// Exact surface position and tangents in world coordinates, nine doubles.
#[no_mangle]
pub unsafe extern "C" fn gcs_surface_sample(h: *mut Sketch, idx: i32, u: f64, v: f64,
    out: *mut f64) -> i32 {
    guard(0, move || {
        let point = gcs_core::solid::RevolvedSurface::named(sk(h),idx as usize)
            .and_then(|surface| surface.at(u,v).map_err(|e| format!("invalid surface sample: {e:?}")));
        match point {
            Ok(p) => {
                for (i,value) in p.position.into_iter().chain(p.du).chain(p.dv).enumerate() {
                    out.add(i).write(value);
                }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}

/// Exact supporting-surface projection and finite-patch incidence, ten doubles:
/// patch point (3), oriented support normal (3), unbounded parameters (2), signed
/// meridian residual, and distance to the returned finite-patch point.
#[no_mangle]
pub unsafe extern "C" fn gcs_surface_project(h: *mut Sketch, idx: i32,
    x: f64, y: f64, z: f64, out: *mut f64) -> i32 {
    guard(0, move || {
        let sample = gcs_core::solid::SurfaceProjector::named(sk(h),idx as usize)
            .and_then(|s| s.project([x,y,z]).map_err(|e| format!("invalid surface projection: {e:?}")));
        match sample {
            Ok(p) => {
                for (i,value) in p.point.into_iter().chain(p.normal).chain(p.parameters)
                    .chain([p.signed_residual,p.incidence_error]).enumerate() {
                    out.add(i).write(value);
                }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}

/// A trial of an implicit envelope: position, normal, velocity, normal velocity (10 doubles).
/// A nonzero final value is off the envelope; this call does not solve or trim its locus.
#[no_mangle]
pub unsafe extern "C" fn gcs_envelope_sample(h: *mut Sketch, idx: i32,
    u: f64, v: f64, roll: f64, out: *mut f64) -> i32 {
    guard(0, move || {
        let sample = gcs_core::envelope::GeneratedEnvelope::named(sk(h),idx as usize)
            .and_then(|e| e.evaluate([u,v,roll]).map_err(|e| format!("invalid envelope trial: {e:?}")));
        match sample {
            Ok(c) => {
                for (i,value) in c.position.into_iter().chain(c.normal).chain(c.velocity)
                    .chain(std::iter::once(c.normal_velocity)).enumerate() {
                    out.add(i).write(value);
                }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}

/// Position and velocity of a fixed source point under a named motion. The angle is
/// radians; velocity is distance per radian of that shared parameter. Six doubles.
#[no_mangle]
pub unsafe extern "C" fn gcs_motion_sample(h: *mut Sketch, idx: i32, angle: f64,
    x: f64, y: f64, z: f64, out: *mut f64) -> i32 {
    guard(0, move || {
        if ![x,y,z].iter().all(|v| v.is_finite()) {
            set_error("a motion sample point must be finite");
            return 0;
        }
        match gcs_core::motion::evaluate(sk(h),idx as usize,angle) {
            Ok(m) => {
                let values: Vec<_> = m.point([x,y,z]).into_iter().chain(m.velocity([x,y,z])).collect();
                if !values.iter().all(|v| v.is_finite()) {
                    set_error("a motion sample overflowed");
                    return 0;
                }
                for (i,value) in values.into_iter().enumerate() { out.add(i).write(value); }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}

pub(crate) fn kind_id(k: EntKind) -> i32 {
    match k {
        EntKind::Point => 0,
        EntKind::Line => 1,
        EntKind::Circle => 2,
        EntKind::Arc => 3,
        EntKind::Spline => 4,
        // the binding decodes an id through its own table (`model.ts`'s `KIND_ID`), so an id
        // keeps its number for good: the ellipse's (5) and the frame's (7) were taken up when
        // those kinds went (#47), and the sphere's, the cone's and the cylinder's when they
        // became library components (`std.Sphere`, `std.Cone`, `std.Cylinder`), and 12–15 when
        // the patch, the seam, the vertex and the edge left the language for the Rust API (#364)
        EntKind::Curve => 5,
        EntKind::Plane => 6,
        EntKind::Face => 7,
        EntKind::Solid => 8,
        EntKind::Surface => 9,
        EntKind::Motion => 10,
        EntKind::Envelope => 11,
        EntKind::Axis => 16,
        EntKind::Meshing => 17,
        EntKind::Traced => 18,
    }
}

pub(crate) fn ent(kind: i32, idx: i32) -> EntRef {
    let k = match kind {
        0 => EntKind::Point,
        1 => EntKind::Line,
        2 => EntKind::Circle,
        3 => EntKind::Arc,
        5 => EntKind::Curve,
        6 => EntKind::Plane,
        7 => EntKind::Face,
        8 => EntKind::Solid,
        9 => EntKind::Surface,
        10 => EntKind::Motion,
        11 => EntKind::Envelope,
        16 => EntKind::Axis,
        17 => EntKind::Meshing,
        18 => EntKind::Traced,
        _ => EntKind::Spline,
    };
    EntRef::new(k, idx as usize)
}

/// Every Param index of an entity, in the model's canonical order; returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_params(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let ps = sk(h).entity_params(ent(kind, idx));
        for (i, p) in ps.iter().enumerate() {
            *out.add(i) = *p as i32;
        }
        ps.len() as i32
    })
}

/// The point indices an entity is built from: line → (p1, p2); circle → (centre); arc →
/// (centre, start, end); ellipse → (centre, major end).  Returns how many were written.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_points(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let v: Vec<usize> = match kind {
            1 => vec![s.lines[idx as usize].p1 as usize, s.lines[idx as usize].p2 as usize],
            2 => vec![s.circles[idx as usize].center as usize],
            3 => {
                let a = &s.arcs[idx as usize];
                vec![a.center as usize, a.start as usize, a.end as usize]
            }
            4 => s.splines[idx as usize].ctrl.iter().map(|&c| c as usize).collect(),
            // a plane's one point is its origin
            6 => vec![s.planes[idx as usize].origin as usize],
            _ => vec![idx as usize],
        };
        for (i, p) in v.iter().enumerate() {
            *out.add(i) = *p as i32;
        }
        v.len() as i32
    })
}

/// The radius Param index of a circle or arc (-1 otherwise).
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_radius_param(h: *mut Sketch, kind: i32, idx: i32) -> i32 {
    guard(-1, move || {
        if kind != 2 && kind != 3 {
            return -1;
        }
        sk(h).round_radius(ent(kind, idx)) as i32
    })
}

/// The classes an entity carries, as a JSON array of strings.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_class(h: *mut Sketch, kind: i32, idx: i32) -> *mut u8 {
    json_out(move || {
        let c = sk(h).class_of(ent(kind, idx));
        Json::Arr(c.0.into_iter().map(Json::Str).collect())
    })
}

/// Give an entity a class, or take one away.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_set_class(
    h: *mut Sketch,
    kind: i32,
    idx: i32,
    name: *const u8,
    name_len: usize,
    on: i32,
) {
    guard((), move || {
        sk(h).set_class(ent(kind, idx), as_str(name, name_len), on != 0);
    })
}

/// What an entity is *drawn with*: dash, width and colour, resolved from the base sheet under
/// the document's (`style.rs`).  The core resolves and a front end strokes what it is handed —
/// the same seam callout layout and curve tessellation sit on.
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_style(h: *mut Sketch, kind: i32, idx: i32) -> *mut u8 {
    json_out(move || style_json(&sk(h).style_of(ent(kind, idx))))
}

/// Every resolved style in one document, in `primitives()` order per kind, plus the sheet's own
/// epoch.  One call a repaint, rather than one a shape: presentation almost never changes and
/// geometry changes every frame, and this is what lets the first be read at the first's rate.
#[no_mangle]
pub unsafe extern "C" fn gcs_styles_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let of = |k: EntKind| -> Json {
            Json::Arr(
                (0..s.count(k))
                    .map(|i| style_json(&s.style_of(EntRef::new(k, i))))
                    .collect(),
            )
        };
        json::object([
            ("epoch", Json::Int(s.style_epoch as i64)),
            ("line", of(EntKind::Line)),
            ("circle", of(EntKind::Circle)),
            ("arc", of(EntKind::Arc)),
            ("spline", of(EntKind::Spline)),
            ("plane", of(EntKind::Plane)),
            ("curve", of(EntKind::Curve)),
        ])
    })
}

/// What a named class list comes to, spelled as a declaration spells one — `"dimension"`, or
/// `"dimension reference"`.  A dimension callout is not an entity and carries no class of its
/// own, but its ink is shared by every callout in the document — which is what a class is.
#[no_mangle]
pub unsafe extern "C" fn gcs_style_named(h: *mut Sketch, name: *const u8, len: usize) -> *mut u8 {
    json_out(move || style_json(&sk(h).style_named(as_str(name, len))))
}

/// Bumped whenever the sheet or a class changes, so a caller may cache the table above.
#[no_mangle]
pub unsafe extern "C" fn gcs_style_epoch(h: *mut Sketch) -> i32 {
    guard(-1, move || sk(h).style_epoch as i32)
}

fn style_json(s: &gcs_core::style::Style) -> Json {
    json::object([
        (
            "dash",
            Json::Arr(s.dash.clone().unwrap_or_default().into_iter().map(Json::Num).collect()),
        ),
        ("width", s.width.map(Json::Num).unwrap_or(Json::Null)),
        ("color", s.color.clone().map(Json::Str).unwrap_or(Json::Null)),
        ("hidden", Json::Bool(!s.shown())),
    ])
}

/// Bounding box of one entity (4 doubles).
#[no_mangle]
pub unsafe extern "C" fn gcs_entity_bounds(h: *mut Sketch, kind: i32, idx: i32, out: *mut f64) {
    guard((), move || {
        let b = sk(h).bounds(ent(kind, idx));
        *out = b.0;
        *out.add(1) = b.1;
        *out.add(2) = b.2;
        *out.add(3) = b.3;
    })
}

/// `P0` / `L3` / `C1` / `A2` — the short label the constraint list and reports use.
#[no_mangle]
pub extern "C" fn gcs_entity_name(kind: i32, idx: i32) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        out_str(io::entity_name(ent(kind, idx)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_distance_between(
    h: *mut Sketch,
    ka: i32,
    ia: i32,
    kb: i32,
    ib: i32,
) -> f64 {
    guard(f64::NAN, move || {
        model::distance_between(sk(h), ent(ka, ia), ent(kb, ib))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_signed_point_to_line(
    h: *mut Sketch,
    x: f64,
    y: f64,
    line: i32,
) -> f64 {
    guard(f64::NAN, move || {
        model::signed_point_to_line(sk(h), x, y, line as usize)
    })
}

/// The arc's CCW sweep (2 doubles).
/// Signed CCW angle from line `a` to line `b`, in radians.
#[no_mangle]
pub unsafe extern "C" fn gcs_angle_between(h: *mut Sketch, a: i32, b: i32) -> f64 {
    guard(f64::NAN, move || {
        model::angle_between(sk(h), EntRef::line(a as usize), EntRef::line(b as usize))
    })
}

/// The point at distance `r` from (cx, cy) towards (tx, ty); 0 if the target is the centre.
#[no_mangle]
pub unsafe extern "C" fn gcs_on_radius(
    cx: f64,
    cy: f64,
    tx: f64,
    ty: f64,
    r: f64,
    out: *mut f64,
) -> i32 {
    guard(0, move || match model::on_radius(cx, cy, tx, ty, r) {
        Some((x, y)) => {
            write(out, &[x, y]);
            1
        }
        None => 0,
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_arc_angles(h: *mut Sketch, idx: i32, out: *mut f64) {
    guard((), move || {
        let (a0, a1) = sk(h).arc_angles(idx as usize);
        *out = a0;
        *out.add(1) = a1;
    })
}

/// The circumcircle construction: writes (cx, cy, r, a0, a1, swapped); 0 when collinear.
#[no_mangle]
pub unsafe extern "C" fn gcs_three_point_arc(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
    out: *mut f64,
) -> i32 {
    guard(-1, move || {
        match model::three_point_arc(ax, ay, bx, by, cx, cy, 1e-9) {
            None => 0,
            Some(g) => {
                let v = [g.cx, g.cy, g.r, g.a0, g.a1, if g.swapped { 1.0 } else { 0.0 }];
                for (i, x) in v.iter().enumerate() {
                    *out.add(i) = *x;
                }
                1
            }
        }
    })
}
