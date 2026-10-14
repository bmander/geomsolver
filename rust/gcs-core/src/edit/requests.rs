//! Gestures as a front end asks for them: each request's JSON read into what its edit takes.
//!
//! A binding hands a gesture over as one JSON object (`{outer: [..], holes: [[..]], word?,
//! body?}`); what it says is read here, once, into a typed request — so the ABI only marshals,
//! and a request that is not one is refused in the core's words.  Anything a request leaves out
//! reads as nothing, and anything not a string where a name goes reads as the empty name, as the
//! edits themselves then refuse.

use super::{EdgeWord, SolidSweep};
use crate::json::Json;
use crate::model::{EntKind, EntRef, Way};
use crate::syntax::{self, BodyWord};

/// A JSON array of strings, as names; anything not a string reads as the empty name.
fn strings(a: &Json) -> Vec<String> {
    a.arr().iter().map(|x| x.as_str().to_string()).collect()
}

/// The names at `key`, none where it is left out.
fn names(v: &Json, key: &str) -> Vec<String> {
    v.get(key).map(strings).unwrap_or_default()
}

/// The loops of names at `key` (a face's holes), none where it is left out.
fn loops(v: &Json, key: &str) -> Vec<Vec<String>> {
    v.get(key).map(|a| a.arr().iter().map(strings).collect()).unwrap_or_default()
}

/// The text at `key`, or `None` where it is left out.
fn text(v: &Json, key: &str) -> Option<String> {
    v.get(key).map(|x| x.as_str().to_string())
}

/// An element drawn by a gesture, `{kind, args: [names], seed: [numbers], name?}`: a plane over
/// two axes or lines (`args` their names, `name` the one asked for), or any other kind over its
/// children (`edit::add_entity`).
pub struct EntityRequest {
    pub kind: EntKind,
    pub args: Vec<String>,
    pub seed: Vec<f64>,
    pub name: Option<String>,
}

impl EntityRequest {
    pub fn from_json(v: &Json) -> Result<EntityRequest, String> {
        let kind = EntKind::parse(v.get("kind").map(|k| k.as_str()).unwrap_or(""))
            .ok_or("unknown entity kind")?;
        let seed =
            v.get("seed").map(|a| a.arr().iter().map(|x| x.as_f64()).collect()).unwrap_or_default();
        Ok(EntityRequest { kind, args: names(v, "args"), seed, name: text(v, "name") })
    }
}

/// A face over drawn edges by name, `{edges: [..], holes: [[..]], name?}`: `edit::add_face`.
pub struct FaceRequest {
    pub edges: Vec<String>,
    pub holes: Vec<Vec<String>>,
    pub name: Option<String>,
}

impl FaceRequest {
    pub fn from_json(v: &Json) -> FaceRequest {
        FaceRequest { edges: names(v, "edges"), holes: loops(v, "holes"), name: text(v, "name") }
    }
}

/// A solid swept from a face by name, `{face, …sweep, name?}`: `edit::add_solid`.
pub struct SolidRequest {
    pub face: String,
    pub sweep: SolidSweep,
    pub name: Option<String>,
}

impl SolidRequest {
    pub fn from_json(v: &Json) -> SolidRequest {
        SolidRequest {
            face: text(v, "face").unwrap_or_default(),
            sweep: SolidSweep::from_json(v),
            name: text(v, "name"),
        }
    }
}

impl SolidSweep {
    /// A sweep as a JSON object says it: `{depth?, from?, to?, through?, about?, sweep?,
    /// sense?}`, every extent the text written.
    pub fn from_json(v: &Json) -> SolidSweep {
        let sense = match v.get("sense").map(|x| x.as_str()) {
            Some("cw") => Some(syntax::Sense::Cw),
            Some("ccw") => Some(syntax::Sense::Ccw),
            _ => None,
        };
        SolidSweep {
            depth: text(v, "depth"), from: text(v, "from"), to: text(v, "to"),
            through: text(v, "through"), about: text(v, "about"), sweep: text(v, "sweep"), sense,
        }
    }

    /// How a solid is to be swept again (`edit::set_sweep`): `{way: "behind" | "front" |
    /// "both", thick}`, an extrusion's thickness the way it runs, or a sweep outright.
    pub fn from_request(v: &Json) -> SolidSweep {
        let way = v.get("way").and_then(|w| Way::parse(w.as_str()));
        match (way, v.get("thick")) {
            (Some(way), Some(t)) => SolidSweep::thick(way, t.as_str()),
            _ => SolidSweep::from_json(v),
        }
    }
}

/// What a region's solid is asked: its outer loop and holes by name, and the body rule it joins
/// with — `word` and `body`, or neither.
pub struct RegionRequest {
    pub outer: Vec<String>,
    pub holes: Vec<Vec<String>>,
    pub body: Option<(BodyWord, String)>,
}

impl RegionRequest {
    /// `{outer: [..], holes: [[..]], word?: "union" | "cut", body?}`, refused for a word that is
    /// not one, or one with no body.
    pub fn from_json(v: &Json) -> Result<RegionRequest, String> {
        let body = match (v.get("word").map(|w| w.as_str()), v.get("body").map(|b| b.as_str())) {
            (None, _) => None,
            (Some(w), body) => match (BodyWord::parse(w), body) {
                (Some(word), Some(body)) => Some((word, body.to_string())),
                _ => return Err(format!("`{w}` with a body is the body rule: union, cut or bound")),
            },
        };
        Ok(RegionRequest { outer: names(v, "outer"), holes: loops(v, "holes"), body })
    }

    /// The body rule as an edit takes it.
    pub fn with(&self) -> Option<(BodyWord, &str)> {
        self.body.as_ref().map(|(w, b)| (*w, b.as_str()))
    }
}

/// An extrusion of the region a click found, `{…region, through?}`: `edit::extrude`.
pub struct ExtrudeRequest {
    pub region: RegionRequest,
    pub through: bool,
}

impl ExtrudeRequest {
    pub fn from_json(v: &Json) -> Result<ExtrudeRequest, String> {
        let region = RegionRequest::from_json(v)?;
        Ok(ExtrudeRequest { region, through: v.get("through").is_some_and(|x| x.as_bool()) })
    }
}

/// A revolution of the region a click found about a line or an axis, `{…region, about}`:
/// `edit::revolve`.
pub struct RevolveRequest {
    pub region: RegionRequest,
    pub about: String,
}

impl RevolveRequest {
    pub fn from_json(v: &Json) -> Result<RevolveRequest, String> {
        let region = RegionRequest::from_json(v)?;
        Ok(RevolveRequest { region, about: text(v, "about").unwrap_or_default() })
    }
}

/// A datum dropped by a button, `{kind: "plane" | "axis", from: [names], current?, name?}`:
/// `edit::add_datum`.
pub struct DatumRequest {
    pub kind: EntKind,
    pub from: Vec<String>,
    pub current: Option<String>,
    pub name: Option<String>,
}

impl DatumRequest {
    pub fn from_json(v: &Json) -> Result<DatumRequest, String> {
        let kind = EntKind::parse(text(v, "kind").as_deref().unwrap_or_default())
            .filter(|k| matches!(k, EntKind::Plane | EntKind::Axis))
            .ok_or("a datum is a plane or an axis")?;
        Ok(DatumRequest {
            kind,
            from: names(v, "from"),
            current: text(v, "current"),
            name: text(v, "name"),
        })
    }
}

/// Sketching on a face, `{face, at?: [x, y, z], name?}`: `edit::sketch_on_face`.
pub struct FaceSketchRequest {
    pub face: String,
    pub at: Option<[f64; 3]>,
    pub name: Option<String>,
}

impl FaceSketchRequest {
    pub fn from_json(v: &Json) -> FaceSketchRequest {
        let at = v.get("at").and_then(|a| match a.arr() {
            [x, y, z] => Some([x.as_f64(), y.as_f64(), z.as_f64()]),
            _ => None,
        });
        FaceSketchRequest { face: text(v, "face").unwrap_or_default(), at, name: text(v, "name") }
    }
}

/// The body rule, `{word: "union" | "cut" | "common", what: [..], body}`:
/// `edit::add_body_word`.
pub struct BodyWordRequest {
    pub word: BodyWord,
    pub what: Vec<String>,
    pub body: String,
}

impl BodyWordRequest {
    pub fn from_json(v: &Json) -> Result<BodyWordRequest, String> {
        let word = text(v, "word").unwrap_or_default();
        let Some(parsed) = BodyWord::parse(&word) else {
            return Err(format!("`{word}` is not a body word: union, cut or bound"));
        };
        Ok(BodyWordRequest {
            word: parsed,
            what: names(v, "what"),
            body: text(v, "body").unwrap_or_default(),
        })
    }
}

/// Rounds or chamfers, `{word: "round" | "chamfer", faces: [..], edges: [[a, b]..], size}`, faces
/// and edges by the paths an object's mesh names them by: `edit::add_rounds`.
pub struct RoundsRequest {
    pub word: EdgeWord,
    pub faces: Vec<String>,
    pub edges: Vec<(String, String)>,
    pub size: String,
}

impl RoundsRequest {
    pub fn from_json(v: &Json) -> Result<RoundsRequest, String> {
        let word = text(v, "word").unwrap_or_default();
        let Some(parsed) = EdgeWord::parse(&word) else {
            return Err(format!("`{word}` is not an edge word: round or chamfer"));
        };
        // an edge is a pair of faces; anything else names no edge
        let edges = v.get("edges").map(|a| a.arr().iter().filter_map(|p| match &strings(p)[..] {
            [a, b] => Some((a.clone(), b.clone())),
            _ => None,
        }).collect()).unwrap_or_default();
        Ok(RoundsRequest {
            word: parsed,
            faces: names(v, "faces"),
            edges,
            size: text(v, "size").unwrap_or_default(),
        })
    }
}

/// A selection as an edit is handed it, `{entities: [["point", 3], ...], constraints: [id, ...],
/// name?}`, each entity by kind and index (one of a kind the model does not know left out), and
/// the name a component made of it is asked to take (`None` where empty).
pub struct SelectionRequest {
    pub ents: Vec<EntRef>,
    pub cons: Vec<u32>,
    pub name: Option<String>,
}

impl SelectionRequest {
    pub fn from_json(v: &Json) -> SelectionRequest {
        let ents = v.get("entities").map(|a| a.arr().iter().filter_map(|r| {
            let p = r.arr();
            (p.len() == 2)
                .then(|| EntKind::parse(p[0].as_str()))
                .flatten()
                .map(|k| EntRef::new(k, p[1].as_i64() as usize))
        }).collect()).unwrap_or_default();
        let cons = v.get("constraints")
            .map(|a| a.arr().iter().map(|x| x.as_i64() as u32).collect())
            .unwrap_or_default();
        let name = text(v, "name").filter(|n| !n.is_empty());
        SelectionRequest { ents, cons, name }
    }
}

/// One constraint, in `report::constraint_from_json`'s shape but with entities by *name* — the
/// document's own way of saying which, rather than an index into a sketch it is about to
/// replace: `{type, args: [..]}`, as the relation `edit::add_relation` writes.  Either spelling
/// of the type: `Horizontal`, the record shape both bindings build, or `horizontal`, the way a
/// statement says it.  An argument left out (`null`) is one the core reads off the geometry.
pub fn relation_from_json(v: &Json) -> Result<syntax::Relation, String> {
    use crate::constraints::CKind;
    let name = text(v, "type").unwrap_or_default();
    let kind = CKind::from_name(&name).or_else(|| CKind::from_name(&syntax::camel(&name)));
    let Some(kind) = kind else {
        return Err(format!("unknown constraint type: {name}"));
    };
    let spec = kind.spec();
    let raw = v.get("args").cloned().unwrap_or(Json::Arr(Vec::new()));
    let raw = raw.arr();
    let mut args: Vec<Option<syntax::Arg>> = vec![None; spec.len()];
    for (i, (_, k)) in spec.iter().enumerate() {
        let Some(a) = raw.get(i) else { continue };
        if matches!(a, Json::Null) {
            continue; // left out: the core reads it off the geometry
        }
        args[i] = Some(if k.is_entity() {
            syntax::Arg::Ref(syntax::Ref::new(a.as_str().to_string()))
        } else if k.is_dimension() {
            syntax::Arg::Dim { text: a.as_str().to_string(), span: Default::default() }
        } else {
            match a {
                Json::Bool(b) => syntax::Arg::Bool(*b),
                Json::Str(s) => syntax::Arg::Word(s.clone()),
                Json::Int(n) => syntax::Arg::Int(*n),
                other => syntax::Arg::Num(other.as_f64()),
            }
        });
    }
    // the app states constraints; a claim is written in the program panel
    Ok(syntax::Relation::of(syntax::RelationForm::Canonical { kind, args, bound: None }))
}
