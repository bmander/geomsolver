//! **The object as a file a viewer opens**: glTF 2.0, in its binary container.
//!
//! Of the formats a solid can leave in, this is the one that fits what the kernel actually knows.
//! STL carries triangles and nothing else — no face is named in it, no unit is recorded, and the
//! grouping `mesh::grouped` works out is thrown away at the door.  STEP carries all of that and
//! more, and is a boundary representation, which this kernel is deliberately not.  glTF sits
//! where the data already is: positions, normals, and a named group per face, which is precisely
//! `mesh::Mesh`.
//!
//! So the mapping is almost an identity, and that is the argument for it.  **Every face of the
//! solid becomes a named node**, so a viewer's outliner shows `body.bore.wall` and clicking the
//! bore's wall selects the bore's wall.  The normals are the ones `grouped` already computed —
//! averaged across a round face and flat across a corner — so a cylinder shades round with no
//! smoothing pass on the far side.
//!
//! **The container is trivial and needs no dependency.**  A `.glb` is a twelve-byte header and
//! two chunks: a JSON manifest, which `json.rs` already writes, and one blob of the numbers.
//! There is no compression to implement and no ZIP, which is what a `.3mf` would have wanted.
//!
//! **Units.**  glTF says a unit is a metre, and it says so normatively.  A document that names
//! its own (`unit mm`) is therefore scaled on the way out, so a forty-millimetre part opens as a
//! forty-millimetre part and not a forty-metre one; a document that names none is written as it
//! stands, there being nothing to convert from.  Either way the document's own unit goes in
//! `asset.extras`, so nothing is lost by the scaling.

use crate::json::Json;
use crate::mesh::Mesh;
use crate::model::Sketch;

/// The glTF component type for a 32-bit float, and the accessor type for a 3-vector.
const FLOAT: i64 = 5126;

/// **The objects of a document as a `.glb`.**  One node per solid, and under each, one node per
/// face — so a viewer's outliner is the document's own tree of names.
///
/// Which solids is the caller's: `overview::objects` is the rule the box uses (a solid is an
/// object exactly when nothing else is made of it), and a caller that wants one names one.
pub fn glb(sk: &Sketch, solids: &[usize], unit: f64) -> Vec<u8> {
    let parts: Vec<(String, Mesh)> = solids
        .iter()
        .map(|&i| (sk.solid_name(i), sk.solid_mesh(i, unit)))
        .collect();
    // glTF is in metres; a document that names millimetres is scaled by that, and one that names
    // no unit is left alone because there is nothing to convert from
    let scale = sk.units.length.map(|(_, mm_per)| mm_per / 1000.0).unwrap_or(1.0);
    let (json, bin) = build(&parts, scale, sk.units.name());
    container(&json.dump(None), &bin)
}

/// The manifest and the blob.  Split out so a test can read either without unpacking a file.
pub fn build(parts: &[(String, Mesh)], scale: f64, unit_name: Option<&str>) -> (Json, Vec<u8>) {
    // One blob, laid out as glTF wants it: every part's positions, then every part's normals,
    // each a float32 triple per vertex — and a face is a **window** into them rather than a copy.
    // The two halves are the same length part for part, so one offset serves both accessors.
    let mut bin: Vec<u8> = Vec::new();
    let mut base: Vec<usize> = Vec::with_capacity(parts.len());
    for (_, m) in parts {
        base.push(bin.len());
        for v in &m.positions {
            bin.extend(((*v * scale) as f32).to_le_bytes());
        }
    }
    let normals_at = bin.len();
    for (_, m) in parts {
        for v in &m.normals {
            bin.extend((*v as f32).to_le_bytes());
        }
    }
    while bin.len() % 4 != 0 {
        bin.push(0);
    }

    let views = Json::Arr(vec![
        object(&[
            ("buffer", Json::Int(0)),
            ("byteOffset", Json::Int(0)),
            ("byteLength", Json::Int(normals_at as i64)),
        ]),
        object(&[
            ("buffer", Json::Int(0)),
            ("byteOffset", Json::Int(normals_at as i64)),
            ("byteLength", Json::Int(normals_at as i64)),
        ]),
    ]);

    let mut accessors: Vec<Json> = Vec::new();
    let mut meshes: Vec<Json> = Vec::new();
    // the objects first and their faces after, so a parent's `children` can name indices that do
    // not exist yet: `n_parents + children.len()` is where the next face will land
    let mut parents: Vec<Json> = Vec::new();
    let mut children: Vec<Json> = Vec::new();
    let n_parents = parts.len();
    for (pi, (name, m)) in parts.iter().enumerate() {
        let mut kids: Vec<Json> = Vec::new();
        for g in &m.groups {
            let verts = g.count * 3;
            // within each view, and the same in both — a vertex's position and its normal sit at
            // the same offset in their own halves of the blob
            let off = base[pi] + g.start * 3 * 3 * 4;
            // **POSITION must carry its own bounds** — the one accessor field the spec insists
            // on, because a loader sizes its scene from them before reading a byte of the blob
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for t in g.start..g.start + g.count {
                for v in 0..3 {
                    for k in 0..3 {
                        let x = m.positions[(t * 3 + v) * 3 + k] * scale;
                        lo[k] = lo[k].min(x);
                        hi[k] = hi[k].max(x);
                    }
                }
            }
            let pos = accessors.len() as i64;
            accessors.push(object(&[
                ("bufferView", Json::Int(0)),
                ("byteOffset", Json::Int(off as i64)),
                ("componentType", Json::Int(FLOAT)),
                ("count", Json::Int(verts as i64)),
                ("type", Json::Str("VEC3".into())),
                ("min", floats(&lo)),
                ("max", floats(&hi)),
            ]));
            accessors.push(object(&[
                ("bufferView", Json::Int(1)),
                ("byteOffset", Json::Int(off as i64)),
                ("componentType", Json::Int(FLOAT)),
                ("count", Json::Int(verts as i64)),
                ("type", Json::Str("VEC3".into())),
            ]));
            let prim = object(&[
                (
                    "attributes",
                    object(&[("POSITION", Json::Int(pos)), ("NORMAL", Json::Int(pos + 1))]),
                ),
                ("material", Json::Int(0)),
            ]);
            meshes.push(object(&[
                ("name", Json::Str(g.path.clone())),
                ("primitives", Json::Arr(vec![prim])),
            ]));
            kids.push(Json::Int((n_parents + children.len()) as i64));
            // **the name twice, and not by accident.**  `name` is for a person reading an
            // outliner; `extras` is for a program, because a loader may sanitise a name — three
            // .js strips the dots out of `body.bore.wall`, its own animation paths being written
            // with them — and `extras` is passed through verbatim as `userData`.  So the path a
            // document wrote survives whatever the viewer does to the label.
            children.push(object(&[
                ("name", Json::Str(g.path.clone())),
                ("mesh", Json::Int(meshes.len() as i64 - 1)),
                (
                    "extras",
                    object(&[
                        ("face", Json::Str(g.path.clone())),
                        ("smooth", Json::Bool(g.smooth)),
                    ]),
                ),
            ]));
        }
        parents.push(object(&[
            ("name", Json::Str(name.clone())),
            ("children", Json::Arr(kids)),
            ("extras", object(&[("solid", Json::Str(name.clone()))])),
        ]));
    }
    let roots: Vec<Json> = (0..n_parents as i64).map(Json::Int).collect();
    let mut all = parents;
    all.extend(children);

    let mut asset = object(&[
        ("version", Json::Str("2.0".into())),
        ("generator", Json::Str("solvent".into())),
    ]);
    if let Some(u) = unit_name {
        // scaled to metres as the spec says, and the document's own unit recorded beside it so
        // the scaling loses nothing
        asset.set("extras", object(&[("unit", Json::Str(u.to_string()))]));
    }
    let doc = object(&[
        ("asset", asset),
        ("scene", Json::Int(0)),
        ("scenes", Json::Arr(vec![object(&[("nodes", Json::Arr(roots))])])),
        ("nodes", Json::Arr(all)),
        ("meshes", Json::Arr(meshes)),
        (
            "materials",
            Json::Arr(vec![object(&[
                ("name", Json::Str("solvent".into())),
                (
                    "pbrMetallicRoughness",
                    object(&[
                        ("baseColorFactor", floats(&[0.72, 0.73, 0.75, 1.0])),
                        ("metallicFactor", Json::Num(0.1)),
                        ("roughnessFactor", Json::Num(0.65)),
                    ]),
                ),
            ])]),
        ),
        ("accessors", Json::Arr(accessors)),
        ("bufferViews", views),
        ("buffers", Json::Arr(vec![object(&[("byteLength", Json::Int(bin.len() as i64))])])),
    ]);
    (doc, bin)
}

/// The `.glb` wrapper: a header and two chunks, each padded to four bytes — JSON with spaces and
/// the blob with zeros, which is what the spec asks for and what a strict loader checks.
fn container(json: &str, bin: &[u8]) -> Vec<u8> {
    let mut j = json.as_bytes().to_vec();
    while j.len() % 4 != 0 {
        j.push(b' ');
    }
    let mut b = bin.to_vec();
    while b.len() % 4 != 0 {
        b.push(0);
    }
    let total = 12 + 8 + j.len() + 8 + b.len();
    let mut out = Vec::with_capacity(total);
    out.extend(0x4654_6C67u32.to_le_bytes()); // "glTF"
    out.extend(2u32.to_le_bytes());
    out.extend((total as u32).to_le_bytes());
    out.extend((j.len() as u32).to_le_bytes());
    out.extend(0x4E4F_534Au32.to_le_bytes()); // "JSON"
    out.extend(j);
    out.extend((b.len() as u32).to_le_bytes());
    out.extend(0x004E_4942u32.to_le_bytes()); // "BIN\0"
    out.extend(b);
    out
}

fn object(fields: &[(&str, Json)]) -> Json {
    Json::Obj(fields.iter().map(|(k, v)| (k.to_string(), v.clone())).collect())
}

fn floats(v: &[f64]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Num(x)).collect())
}
