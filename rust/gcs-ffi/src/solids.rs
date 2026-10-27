//! Solids: meshed, exported, and the swept ones' surfaces meshed or built elsewhere.

use super::*;

/// **A solid's mesh**: nine doubles a triangle, welded so every edge pairs up and ordered by the
/// face each triangle belongs to.  Returns how many doubles were written, or -1 if the buffer is
/// too small — `gcs_entity_params`' shape, because a mesh is tens of thousands of numbers all of
/// one kind and JSON is for the ragged things.
///
/// `gcs_solid_normals` fills the matching normals and `gcs_solid_faces_json` says where each face
/// starts; the three read one memoised answer, so asking for all three costs one walk.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_mesh(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        let solid = match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)) {
            Ok(s) => s,
            Err(message) => { set_error(message); return -1; }
        };
        let m = solid.mesh();
        if m.positions.len() > cap.max(0) as usize {
            return -1;
        }
        let world = |p: [f64; 3]| solid.to_world(gcs_core::solid::LocalPoint(p)).0;
        write_world(&m.positions, solid.improper(), world, out);
        m.positions.len() as i32
    })
}

/// A mesh's triples (`Mesh::positions` or `normals`) written into `out` as `to_world` takes each,
/// each triangle's corners in the order it stands in the world: turned round where the solid's
/// placement reflects (#578), so it still faces out.
unsafe fn write_world(v: &[f64], reflected: bool, to_world: impl Fn([f64; 3]) -> [f64; 3],
                      out: *mut f64) {
    for (t, tri) in v.chunks_exact(9).enumerate() {
        for (k, corner) in [0, 1, 2].into_iter().enumerate() {
            let c = if reflected && corner > 0 { 3 - corner } else { corner };
            let w = to_world([tri[3 * c], tri[3 * c + 1], tri[3 * c + 2]]);
            for (j, value) in w.into_iter().enumerate() { out.add(9 * t + 3 * k + j).write(value); }
        }
    }
}

/// The normals of that mesh, one per vertex: a face's own where it is flat, and the average of
/// the facets meeting there where it is round, so a bore's wall shades as one surface.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_normals(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        let solid = match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)) {
            Ok(s) => s,
            Err(message) => { set_error(message); return -1; }
        };
        let m = solid.mesh();
        if m.normals.len() > cap.max(0) as usize {
            return -1;
        }
        write_world(&m.normals, solid.improper(), |n| solid.vector_to_world(n), out);
        m.normals.len() as i32
    })
}

/// **A solid in its own coordinates, as a renderer draws it** (#674): its mesh over shared
/// vertices and the edges a viewer inks (`mesh::Drawn`), before its placement
/// (`gcs_solid_placement`) stands it in the world.  A host builds one geometry of it while the
/// geometry holds, and moves it by the placement alone.  One length-prefixed byte buffer,
/// little-endian, every item four bytes: four counts (`u32`) — vertices `nv`, triangles `nt`,
/// sharp edges `ns`, tangent edges `nk` — then `nv` positions and `nv` normals (three `f32`
/// each), `nv` faces (`f32`, exact to 2²⁴: the group each vertex is of, as
/// `gcs_solid_faces_json` lists them, as a vertex attribute reads it), `nt` triangles (three
/// `u32`), and the `ns` sharp then `nk` tangent edges (six `f32` each).  Null where the solid does not evaluate (the reason in
/// `gcs_last_error`).
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_drawn(h: *mut Sketch, idx: i32, unit: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        let solid = match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)) {
            Ok(s) => s,
            Err(message) => { set_error(message); return std::ptr::null_mut(); }
        };
        let d = solid.drawn();
        let mut out: Vec<u8> = Vec::with_capacity(4 * (4 + 7 * d.positions.len()
            + 3 * d.triangles.len() + 6 * (d.sharp.len() + d.tangent.len())));
        for n in [d.positions.len(), d.triangles.len(), d.sharp.len(), d.tangent.len()] {
            out.extend((n as u32).to_le_bytes());
        }
        let reals = d.positions.iter().chain(&d.normals).flatten().map(|&x| x as f32)
            .chain(d.face.iter().map(|&f| f as f32));
        out.extend(reals.flat_map(f32::to_le_bytes));
        out.extend(d.triangles.iter().flatten().flat_map(|t| t.to_le_bytes()));
        let edges = d.sharp.iter().chain(&d.tangent).flatten().flatten().map(|&x| x as f32);
        out.extend(edges.flat_map(f32::to_le_bytes));
        out_bytes(out)
    })
}

/// **A solid cut by the workspace's section** (`Sketch::cut`, `EvaluatedSolid::section`), in its
/// own coordinates as `gcs_solid_drawn` gives it, so the host places it by the same matrix: how many
/// doubles of the face the cut leaves follow (nine a triangle, turned to the side taken away),
/// then those, then where the cut meets the surface (six doubles a segment) to the end.  Returns
/// how many doubles there are, the count included, writing them only where `cap` holds them all;
/// 0 where the workspace is not sectioned, -1 where the solid does not evaluate (the reason in
/// `gcs_last_error`).
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_section(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        let Some(cut) = s.cut else { return 0 };
        let solid = match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)) {
            Ok(s) => s,
            Err(message) => { set_error(message); return -1; }
        };
        let section = solid.section(&cut);
        let mut values = vec![section.triangles.len() as f64];
        values.extend(&section.triangles);
        values.extend(section.segments().into_iter().flat_map(|[a, b]| a.into_iter().chain(b)));
        if values.len() <= cap.max(0) as usize {
            std::ptr::copy_nonoverlapping(values.as_ptr(), out, values.len());
        }
        values.len() as i32
    })
}

/// **Where a solid stands** (#412), eighteen doubles: its shape's serial (the same exactly while
/// the solid is the same evaluation, however it has moved, so a host keeps its geometry against
/// it), its geometry's (`EvaluatedSolid::geometry_serial`: the same for every solid of one
/// binding, so a host holds their triangles once, #674), then its placement as a 4×4 matrix,
/// column-major, from its own coordinates (`gcs_solid_drawn`) to the world's.  With `built`
/// nonzero, only where the core has the shape already (`Sketch::built_solid`), returning 0 where
/// it would have to evaluate it — what a gesture asks.  Returns 18, or -1 where the solid does
/// not evaluate or `cap` is short.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_placement(
    h: *mut Sketch,
    idx: i32,
    unit: f64,
    built: i32,
    out: *mut f64,
    cap: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        let policy = gcs_core::solid::ApproximationPolicy::from_unit(unit);
        let solid = if built != 0 { s.built_solid(i, policy) } else {
            Some(s.evaluated_solid(i, policy))
        };
        let solid = match solid {
            None => return 0,
            Some(Ok(s)) => s,
            Some(Err(message)) => { set_error(message); return -1; }
        };
        if cap < 18 { return -1; }
        out.write(solid.shape_serial() as f64);
        out.add(1).write(solid.geometry_serial() as f64);
        for (k, value) in solid.matrix().into_iter().enumerate() { out.add(2 + k).write(value); }
        18
    })
}

/* -- swept solids' surfaces, meshed elsewhere ---------------------------------- */

/// A page that meshes swept solids in a worker says so: a swept solid with no supplied
/// surface is then refused here rather than meshed on the thread that draws.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_defer_fields(h: *mut Sketch, on: i32) {
    use gcs_core::solid::FieldMeshing;
    guard((), move || sk(h).field_meshing.set(if on != 0 { FieldMeshing::Deferred } else { FieldMeshing::Now }))
}

/// **What a page meshing swept solids in a worker has to mesh** (`Sketch::field_jobs`), as JSON:
/// `[{ "solid", "name", "key" }]`, `key` the drawing's digest in hex (`FieldJob::key`) — the
/// same for the same drawing on any core, so a page compares keys and decides nothing else.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_field_jobs(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        Json::Arr(sk(h).field_jobs().into_iter().map(|j| Json::Obj(vec![
            ("solid".into(), Json::Int(j.solid as i64)),
            ("name".into(), Json::Str(j.name)),
            ("key".into(), Json::Str(format!("{:016x}", j.key))),
        ])).collect())
    })
}

/// Give swept solid `idx` a surface meshed elsewhere: `nv` vertices as `3·nv` doubles and `nt`
/// triangles as `3·nt` vertex indices. 0, or −1 with the core's reason (`Sketch::supply_field`).
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_supply_field(
    h: *mut Sketch,
    idx: i32,
    vertices: *const f64,
    nv: i32,
    triangles: *const u32,
    nt: i32,
    provisional: i32,
) -> i32 {
    guard(-1, move || {
        let (nv, nt) = (nv.max(0) as usize, nt.max(0) as usize);
        let v = if nv == 0 { &[][..] } else { std::slice::from_raw_parts(vertices, 3 * nv) };
        let t = if nt == 0 { &[][..] } else { std::slice::from_raw_parts(triangles, 3 * nt) };
        let surface = gcs_core::solid::FieldSurface {
            vertices: v.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect(),
            triangles: t.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect(),
            provisional: provisional != 0,
            exact: None,
        };
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        match s.supply_field(i, surface) {
            Ok(()) => 0,
            Err(message) => { set_error(message); -1 }
        }
    })
}

/* -- every object built elsewhere, exactly (#671) -------------------------------- */

/// A page that builds every object in its pool of workers says so (`Sketch::supply_objects`):
/// the objects are then drawn from what is supplied, never built on the thread that draws.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_supply_objects(h: *mut Sketch, on: i32) {
    guard((), move || sk(h).supply_objects(on != 0))
}

/// **Every build the page's pool has to make** (`Sketch::object_jobs`), as JSON:
/// `[{ "solid", "name", "key", "objects": [..], "swept" }]` — one a binding, `key` in hex as a
/// field job's, the same for the same build on any core.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_object_jobs(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        Json::Arr(sk(h).object_jobs().into_iter().map(|j| Json::Obj(vec![
            ("solid".into(), Json::Int(j.solid as i64)),
            ("name".into(), Json::Str(j.name)),
            ("key".into(), Json::Str(format!("{:016x}", j.key))),
            ("objects".into(), Json::Arr(j.objects.into_iter().map(|i| Json::Int(i as i64)).collect())),
            ("swept".into(), Json::Bool(j.swept)),
        ])).collect())
    })
}

/// Give the solid a job builds its surface (`Sketch::supply_object`): the build's buffer as the
/// builder wrote it (`gcs_exact_builder_display`, `Display::bytes`). 1 where it is current — what
/// the solid reads is what the build read — 0 where it is an earlier drawing's, drawn stale, or −1
/// with the reason.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_supply_object(h: *mut Sketch, idx: i32, ptr: *const u8, len: usize) -> i32 {
    guard(-1, move || {
        let bytes = if ptr.is_null() || len == 0 { &[][..] } else { std::slice::from_raw_parts(ptr, len) };
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        let supplied = gcs_core::brep::export::Display::read(bytes).and_then(|d| s.supply_object(i, d));
        match supplied {
            Ok(current) => current as i32,
            Err(message) => { set_error(message); -1 }
        }
    })
}

/// The build of the solid a job builds refused, in the build's words (`Sketch::refuse_object`).
/// 0, or −1 with the reason.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_refuse_object(h: *mut Sketch, idx: i32, ptr: *const u8, len: usize) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        match s.refuse_object(i, as_str(ptr, len)) {
            Ok(()) => 0,
            Err(message) => { set_error(message); -1 }
        }
    })
}

/// Where each object stands with the page's pool (`Sketch::object_state`), as JSON:
/// `[{ "solid", "state": "built" | "building" | "rebuilding" | "refused", "why"? }]`.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_object_states(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        use gcs_core::solid::ObjectState;
        let s = sk(h);
        Json::Arr(model::objects(s).into_iter().map(|i| {
            let (state, why) = match s.object_state(i) {
                ObjectState::Built => ("built", None),
                ObjectState::Building => ("building", None),
                ObjectState::Rebuilding => ("rebuilding", None),
                ObjectState::Refused(why) => ("refused", Some(why)),
            };
            let mut kv = vec![("solid".into(), Json::Int(i as i64)), ("state".into(), Json::Str(state.into()))];
            if let Some(why) = why { kv.push(("why".into(), Json::Str(why))); }
            Json::Obj(kv)
        }).collect())
    })
}

/// An object's exact surface, built a stage at a time (`brep::export::Builder`).
pub struct ExactBuilder(gcs_core::brep::export::Builder);

/// Start building solid `idx`'s exact surface: a handle, or null with the reason.
#[no_mangle]
pub unsafe extern "C" fn gcs_exact_builder_new(h: *mut Sketch, idx: i32) -> *mut ExactBuilder {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        if let Some((_, message)) = gcs_core::solid::bearing_errors(s).into_iter().next() {
            set_error(message);
            return std::ptr::null_mut();
        }
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        match gcs_core::brep::export::Builder::new(s, i) {
            Ok(b) => Box::into_raw(Box::new(ExactBuilder(b))),
            Err(refusal) => { set_error(format!("{} (at {})", refusal.message, refusal.stage.key())); std::ptr::null_mut() }
        }
    })
}

/// Run the next stage: 0 still going, 1 built, −1 refused with the reason (and the stage).
#[no_mangle]
pub unsafe extern "C" fn gcs_exact_builder_step(b: *mut ExactBuilder) -> i32 {
    guard(-1, move || match (*b).0.step() {
        Ok(done) => done as i32,
        Err(refusal) => { set_error(format!("{} (at {})", refusal.message, refusal.stage.key())); -1 }
    })
}

/// Where the build stands: `{ "doing", "done", "total", "said" }` — what the next stage does, the
/// stages run and in all, and the last thing a stage said (or null).
#[no_mangle]
pub unsafe extern "C" fn gcs_exact_builder_progress(b: *mut ExactBuilder) -> *mut u8 {
    json_out(move || {
        let b = &(*b).0;
        let (done, total) = b.stages();
        Json::Obj(vec![
            ("doing".into(), Json::Str(b.doing())),
            ("done".into(), Json::Int(done as i64)),
            ("total".into(), Json::Int(total as i64)),
            ("said".into(), b.said().map_or(Json::Null, Json::Str)),
        ])
    })
}

/// The built surface as one buffer (`Display::bytes`: its mesh, faces, names and edges), what
/// `gcs_sketch_supply_object` takes; or null with the reason while it is not built.
#[no_mangle]
pub unsafe extern "C" fn gcs_exact_builder_display(b: *mut ExactBuilder) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let Some(d) = (*b).0.display() else { set_error("the exact surface is not built yet"); return std::ptr::null_mut() };
        out_bytes(d.bytes())
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_exact_builder_free(b: *mut ExactBuilder) {
    guard((), move || {
        if !b.is_null() {
            drop(Box::from_raw(b));
        }
    })
}

/// A field mesher and the last surface it was asked for.
pub struct Mesher {
    mesher: gcs_core::solid::FieldMesher,
    last: gcs_core::solid::FieldSurface,
}

/// Start meshing swept solid `idx` of this sketch, `fineness` times finer than the preview's
/// (`FieldMesher::with_fineness`): a handle, or null with the reason.
#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_new(h: *mut Sketch, idx: i32, fineness: f64) -> *mut Mesher {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        match gcs_core::solid::FieldMesher::with_fineness(s, i, fineness) {
            Ok(mesher) => Box::into_raw(Box::new(Mesher { mesher, last: Default::default() })),
            Err(message) => { set_error(message); std::ptr::null_mut() }
        }
    })
}

/// Refine at most about `budget` facets: 0 still going, 1 finished, −1 failed with the reason.
#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_step(m: *mut Mesher, budget: i32) -> i32 {
    guard(-1, move || match (*m).mesher.step(budget.max(0) as usize) {
        Ok(done) => done as i32,
        Err(message) => { set_error(message); -1 }
    })
}

/// Take the surface as it stands, writing `[vertices, triangles, provisional]` to `out`: 0, or
/// −1 with the reason. `gcs_field_mesher_vertices` and `_triangles` then read it.
#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_snapshot(m: *mut Mesher, out: *mut i32) -> i32 {
    guard(-1, move || {
        (*m).last = (*m).mesher.snapshot();
        let last = &(*m).last;
        write(out, &[last.vertices.len() as i32, last.triangles.len() as i32, last.provisional as i32]);
        0
    })
}

/// The snapshot's vertices, three doubles each: the number written, or −1 past `cap`.
#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_vertices(m: *mut Mesher, out: *mut f64, cap: i32) -> i32 {
    guard(-1, move || {
        let flat: Vec<f64> = (*m).last.vertices.iter().flatten().copied().collect();
        if flat.len() > cap.max(0) as usize { return -1; }
        write(out, &flat);
        flat.len() as i32
    })
}

/// The snapshot's triangles, three vertex indices each: the number written, or −1 past `cap`.
#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_triangles(m: *mut Mesher, out: *mut u32, cap: i32) -> i32 {
    guard(-1, move || {
        let flat: Vec<u32> = (*m).last.triangles.iter().flatten().copied().collect();
        if flat.len() > cap.max(0) as usize { return -1; }
        write(out, &flat);
        flat.len() as i32
    })
}

/// Where the meshing stands, as JSON: `{ "doing", "phase", "stage", "rebuild", "queued", "worst",
/// "inserted", "queries", "readings", "curves", "fraction", "within", "failed" }`
/// (`FieldProgress`; `doing` is the words a host shows, `FieldProgress::doing`).
#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_progress(m: *mut Mesher) -> *mut u8 {
    json_out(move || {
        let p = (*m).mesher.progress();
        Json::Obj(vec![
            ("doing".into(), Json::Str(p.doing())),
            ("phase".into(), Json::Str(p.phase.into())),
            ("stage".into(), Json::Str(p.refine.stage.word().into())),
            ("rebuild".into(), Json::Int(p.refine.rebuild as i64)),
            ("queued".into(), Json::Int(p.refine.queued as i64)),
            ("worst".into(), Json::Num(p.refine.worst)),
            ("inserted".into(), Json::Int(p.refine.inserted as i64)),
            ("queries".into(), Json::Int(p.refine.queries as i64)),
            ("readings".into(), Json::Int(p.refine.readings as i64)),
            ("curves".into(), p.curves.map_or(Json::Null, |c| Json::Int(c as i64))),
            ("fraction".into(), Json::Num(p.fraction)),
            ("within".into(), Json::Num(p.within)),
            ("failed".into(), Json::Bool(p.failed)),
        ])
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_field_mesher_free(m: *mut Mesher) {
    guard((), move || {
        if !m.is_null() {
            drop(Box::from_raw(m));
        }
    })
}

/// **What a Boolean would make of a body, before it is written** (#162 F4): `{target, word:
/// "union" | "cut" | "common", tools: [index…]}` in, `Sketch::preview_body`'s mesh out as `{faces,
/// positions, normals}` — the shape `gcs_solid_mesh`'s three calls give apart, in world
/// coordinates, in one JSON answer: it is asked once a pick, never per frame.  Null with the
/// cause where it is refused.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_preview_json(h: *mut Sketch, ptr: *const u8, len: usize) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let v = as_json(ptr, len);
        let index = |x: &Json| usize::try_from(x.as_i64()).unwrap_or(usize::MAX);
        let Some(word) = gcs_core::syntax::BodyWord::parse(v.get("word").map_or("", |w| w.as_str())) else {
            set_error("a preview's word is the body rule's: union, cut or bound".to_string());
            return std::ptr::null_mut();
        };
        let target = v.get("target").map_or(usize::MAX, index);
        let tools: Vec<usize> = v.get("tools").map(|t| t.arr().iter().map(index).collect()).unwrap_or_default();
        let solid = match sk(h).preview_body(target, word, &tools) {
            Ok(s) => s,
            Err(message) => { set_error(message); return std::ptr::null_mut(); }
        };
        let m = solid.world_mesh();
        out_json(Json::Obj(vec![
            ("faces".into(), face_groups(&m)),
            ("positions".into(), Json::from(m.positions)),
            ("normals".into(), Json::from(m.normals)),
        ]))
    })
}

/// **The faces of that mesh**, as the document names them: `[{ "path", "start", "count",
/// "smooth" }]`, where `start` and `count` are triangles.  A viewer selects and shades by these
/// — `body.bore.wall` and not a triangle index.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_faces_json(h: *mut Sketch, idx: i32, unit: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        let solid = match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)) {
            Ok(s) => s,
            Err(message) => { set_error(message); return std::ptr::null_mut(); }
        };
        out_json(face_groups(solid.mesh()))
    })
}

/// A mesh's faces as `[{ "path", "start", "count", "smooth" }]` — what `gcs_solid_faces_json`
/// and a preview's `faces` say, one shape for `core/mesh.ts`'s one `Face`.
fn face_groups(m: &gcs_core::mesh::Mesh) -> Json {
    Json::Arr(m.groups.iter().map(|g| {
        let mut o = Json::Obj(vec![
            ("path".into(), Json::Str(g.path.clone())),
            ("start".into(), Json::Int(g.start as i64)),
            ("count".into(), Json::Int(g.count as i64)),
        ]);
        if g.smooth {
            o.set("smooth", Json::Bool(true));
        }
        o
    }).collect())
}

/// **A solid as a `.glb`**: binary glTF, every face of the object a named node, positions in
/// metres where the document names a unit.  Returned as a length-prefixed byte buffer — the same
/// handle a string comes back as, since that convention is bytes and not characters.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_glb(h: *mut Sketch, idx: i32, unit: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        // -1 asks for **every object** — the solids nothing else is made of, which is what a
        // document's "export the solid" means when it holds an assembly
        let s = sk(h);
        let which: Vec<usize> = if idx < 0 {
            gcs_core::overview::objects(s)
        } else {
            let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
            vec![i]
        };
        if let Some((_, message)) = gcs_core::solid::bearing_errors(s).into_iter().next() {
            set_error(message);
            return std::ptr::null_mut();
        }
        match gcs_core::gltf::checked_glb(s, &which, gcs_core::solid::ApproximationPolicy::from_unit(unit)) {
            Ok(bytes) => out_bytes(bytes),
            Err(message) => { set_error(message); std::ptr::null_mut() }
        }
    })
}

/// **A solid as binary STL**: what a printer takes.  Welded, so every edge has its partner —
/// the file a boundary evaluation does not give on its own.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_stl(h: *mut Sketch, idx: i32, unit: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        if let Some((_, message)) = gcs_core::solid::bearing_errors(s).into_iter().next() {
            set_error(message);
            return std::ptr::null_mut();
        }
        match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)).and_then(|s| s.stl()) {
            Ok(bytes) => out_bytes(bytes),
            Err(message) => { set_error(message); std::ptr::null_mut() }
        }
    })
}

/// Solid `idx`'s surface as binary STL as it stands, though still being refined
/// (`EvaluatedSolid::preview_stl`); null with the reason where there is none yet.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_stl_preview(h: *mut Sketch, idx: i32, unit: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::from_unit(unit)).and_then(|s| s.preview_stl()) {
            Ok(bytes) => out_bytes(bytes),
            Err(message) => { set_error(message); std::ptr::null_mut() }
        }
    })
}

/// **A solid's exact export by the core's own kernel** (`gcs_core::brep::export`, the same functions
/// `solventc` calls): `kind` 0 for its STEP file (text), 1 for its STL, held to `tolerance`
/// millimetres (zero or less: the gross bars). A body with swept cuts of the generating class is
/// admitted and built as one sector patterned, its STL held to its material field: seconds, not
/// milliseconds, which is why the app asks for it in its worker. Null with the reason where the
/// export refuses, the stage it refused at named.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_exact(h: *mut Sketch, idx: i32, kind: i32, tolerance: f64) -> *mut u8 {
    guard(std::ptr::null_mut(), move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return std::ptr::null_mut() };
        if let Some((_, message)) = gcs_core::solid::bearing_errors(s).into_iter().next() {
            set_error(message);
            return std::ptr::null_mut();
        }
        let tolerance = if tolerance > 0. {
            match gcs_core::solid::export::Tolerance::new(tolerance) { Ok(t) => Some(t), Err(m) => { set_error(m); return std::ptr::null_mut() } }
        } else { None };
        let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
        let made = gcs_core::brep::export::exact(s, i, None, tolerance, &say).and_then(|exact| match kind {
            0 => gcs_core::brep::export::step(&exact, &s.solids[i].name, tolerance, &say).map(String::into_bytes),
            _ => gcs_core::brep::export::stl(s, i, &exact, tolerance, &say),
        });
        match made {
            Ok(bytes) => out_bytes(bytes),
            Err(refusal) => { set_error(format!("{} (at {})", refusal.message, refusal.stage.key())); std::ptr::null_mut() }
        }
    })
}

/// Whether solid `idx`'s surface is still being refined: 1, 0, or −1 with the reason where it has
/// none to ask about (`Sketch::field_provisional`; a solid with no sweep is 0 unevaluated).
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_provisional(h: *mut Sketch, idx: i32) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        match s.field_provisional(i) {
            Ok(p) => p as i32,
            Err(message) => { set_error(message); -1 }
        }
    })
}

/// Whether solid `idx` is drawn from a build of an earlier drawing while its new one is made
/// (`EvaluatedSolid::stale`, #672): 1 or 0, read off the evaluation the box drew; −1 with the
/// reason where it does not evaluate.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_stale(h: *mut Sketch, idx: i32) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let Some(i) = solid_at(s, idx) else { return -1 };
        match s.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Mesh) {
            Ok(solid) => solid.stale() as i32,
            Err(message) => { set_error(message); -1 }
        }
    })
}

/// **The objects a document has**: the solids nothing else is made of, by name and index.  A
/// bore is a hole in a part and not a part beside it, and this is the rule that says so.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_objects_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || named_solids(sk(h), gcs_core::overview::objects(sk(h))))
}

/// The objects the workspace draws (`overview::shown_objects`): `gcs_solid_objects_json`'s, less
/// what the model outline hid (#435).
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_shown_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || named_solids(sk(h), gcs_core::overview::shown_objects(sk(h))))
}

/// Solids by name and index: `[{name, index}…]`.
fn named_solids(s: &Sketch, solids: Vec<usize>) -> Json {
    Json::Arr(solids.into_iter().map(|i| {
        Json::Obj(vec![
            ("name".into(), Json::Str(s.solid_name(i))),
            ("index".into(), Json::Int(i as i64)),
        ])
    }).collect())
}

/// **The objects that are not drawn, and why** (`overview::unbuilt`):
/// `[{ "solid", "name", "reason" }]`, the reason the core's own sentence.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_unbuilt_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        Json::Arr(
            gcs_core::overview::unbuilt(s)
                .into_iter()
                .map(|(i, why)| {
                    Json::Obj(vec![
                        ("solid".into(), Json::Int(i as i64)),
                        ("name".into(), Json::Str(s.solid_name(i))),
                        ("reason".into(), Json::Str(why)),
                    ])
                })
                .collect(),
        )
    })
}

/// **How finely a mesh of this solid should be cut**: `MESH_SAGITTA` of its own diagonal, as a
/// `unit`.  A viewer that wants its own faceting passes its own; one that has no opinion passes
/// this, and gets a mesh matched to the object rather than to a report's four digits.
#[no_mangle]
pub unsafe extern "C" fn gcs_solid_mesh_unit(h: *mut Sketch, idx: i32) -> f64 {
    guard(0.0, move || {
        let s = sk(h);
        solid_at(s, idx).map_or(0.0, |i| gcs_core::solid::mesh_unit(s, i))
    })
}

/// The materials the drawing states, and the solids made of each (`report::materials_json`,
/// #510 P3).
#[no_mangle]
pub unsafe extern "C" fn gcs_materials_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || report::materials_json(sk(h)))
}
