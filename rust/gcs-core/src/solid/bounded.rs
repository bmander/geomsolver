//! **A region in general position, built exactly** (#238): a region solid whose terms share no
//! axis (`region::in_general`), each a ball, a cylinder, a cone narrower than square or a
//! half-space, is the common of the primitives its terms bound — each one the region keeps within
//! taken in common, each one it keeps outside taken away — every primitive bounded past the box
//! the region lies in (`RegionField::support`), so nothing but its terms' own surfaces reaches the
//! material: Steinmetz's bicylinder, two cylinders crossing.  The recipe's `region` node; held to
//! the region read point by point before it stands (`gate`), as the lowering table's pieces are.
//! Any other term (a sum of measurements, a face's, an offset) leaves the region the field's.
use crate::brep::build::{Profile, ProfileEdge};
use crate::brep::geom::Frame;
use crate::json::{object, Json};
use crate::model::{RegionShape, Sketch, SolidDef};
use crate::space::{add, dot, norm, scale, sub};
use crate::syntax::Cmp;
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::rc::Rc;

#[allow(unused_imports)]
use crate::fmath::Det;

type V = [f64; 3];

/// A region in general position as its terms' primitives, in the order they are combined: first
/// those it keeps within, then those it keeps outside.
#[derive(Clone, Debug)]
pub struct Bounded {
    pieces: Vec<Piece>,
}

/// One term's primitive: what it is, whether the region lies within it (taken in common) or
/// outside it (taken away), and the set the term came from — what the faces it makes are called.
#[derive(Clone, Debug)]
struct Piece {
    keep: bool,
    shape: Shape,
    name: String,
}

#[derive(Clone, Debug)]
enum Shape {
    /// The ball of radius `r` about `c`.
    Ball { c: V, r: f64 },
    /// The cylinder of radius `r` about the line through `a` along `d` (unit), `reach` along it
    /// either way from `a`.
    Cylinder { a: V, d: V, r: f64, reach: f64 },
    /// The cone with its apex at `a`, opening along `d` (unit) `half` off it, `reach` long.
    Cone { a: V, d: V, half: f64, reach: f64 },
    /// The half-space `n · (x − o) ≤ 0` (`n` unit) as a block standing on its plane, `reach` from
    /// `o` across it either way and `reach` deep.
    Half { o: V, n: V, reach: f64 },
}

impl Bounded {
    /// The recipe's node (lengths times `scale`, to millimetres): `region`, its `pieces` each a
    /// `revolve` or a `prism` with `keep` beside it.
    pub fn node(&self, scale_by: f64) -> Result<Json, String> {
        let v = |p: V| Json::Arr(p.iter().map(|&x| Json::from(x * scale_by)).collect());
        let dir = |p: V| Json::Arr(p.iter().map(|&x| Json::from(x)).collect());
        let pieces = self.pieces.iter().map(|piece| {
            let profile = |origin: V, normal: V, edges: Vec<ProfileEdge>| {
                let names = vec![vec![piece.name.clone(); edges.len()]];
                Profile { origin, normal, loops: vec![edges], names }.to_json(scale_by)
            };
            let mut node = match piece.shape {
                Shape::Ball { c, r } => {
                    // a half disc turned once about a line through the centre
                    let f = Frame::about(c, [0.0, 0.0, 1.0]);
                    let arc = ProfileEdge::Arc { frame: Frame::new(c, f.z, f.y), r, span: Some([0.0, PI]) };
                    let shut = ProfileEdge::Line { a: f.at([0.0, -r, 0.0]), b: f.at([0.0, r, 0.0]) };
                    object([("kind", "revolve".into()), ("profile", profile(c, f.z, vec![arc, shut])?),
                        ("origin", v(c)), ("axis", dir(f.y)), ("angle", TAU.into())])
                }
                Shape::Cylinder { a, d, r, reach } => {
                    let circle = ProfileEdge::Arc { frame: Frame::about(a, d), r, span: None };
                    object([("kind", "prism".into()), ("profile", profile(a, d, vec![circle])?),
                        ("from", (-reach * scale_by).into()), ("to", (reach * scale_by).into())])
                }
                Shape::Cone { a, d, half, reach } => {
                    // the triangle from the apex to the rim and back along the axis, turned once
                    let f = Frame::about(a, d);
                    let tip = add(a, scale(d, reach));
                    let rim = add(tip, scale(f.x, reach * half.dtan()));
                    let edges = vec![ProfileEdge::Line { a, b: rim }, ProfileEdge::Line { a: rim, b: tip },
                                     ProfileEdge::Line { a: tip, b: a }];
                    object([("kind", "revolve".into()), ("profile", profile(a, f.y, edges)?),
                        ("origin", v(a)), ("axis", dir(d)), ("angle", TAU.into())])
                }
                Shape::Half { o, n, reach } => {
                    // a square on the plane, swept away from `n` into the half-space
                    let keep = scale(n, -1.0);
                    let f = Frame::about(o, keep);
                    let at = |x: f64, y: f64| f.at([x, y, 0.0]);
                    let corners = [at(-reach, -reach), at(reach, -reach), at(reach, reach), at(-reach, reach)];
                    let edges = (0..4).map(|k| ProfileEdge::Line { a: corners[k], b: corners[(k + 1) % 4] })
                        .collect();
                    object([("kind", "prism".into()), ("profile", profile(o, keep, edges)?),
                        ("from", 0.0.into()), ("to", (reach * scale_by).into())])
                }
            };
            node.set("keep", Json::Bool(piece.keep));
            Ok(node)
        }).collect::<Result<Vec<_>, String>>()?;
        Ok(object([("kind", "region".into()), ("pieces", Json::Arr(pieces))]))
    }
}

/// Region solid `solid` built as the common of its terms, where it is read in general position
/// and every term is one this construction makes (`None` otherwise): the construction, or why it
/// stands not — its gate refused it, or it holds nowhere.  Remembered against `solid::reads`
/// (`Sketch::bounded_cache`).
pub fn built(sk: &Sketch, solid: usize) -> Option<Result<Rc<Bounded>, String>> {
    let SolidDef::Region { terms, .. } = &sk.solids.get(solid)?.def else { return None };
    // a region over a body's faces is the lowering table's
    if terms.iter().any(|t| super::faces::reads_faces(&t.shape)) {
        return None;
    }
    let key = super::reads(sk, solid, 0.0);
    if let Some((old, b)) = sk.bounded_cache.borrow().get(&solid) {
        if *old == key {
            return b.clone();
        }
    }
    let b = construction(sk, solid).map(|b| b.and_then(|b| {
        gate(sk, solid, &b)?;
        Ok(Rc::new(b))
    }));
    sk.bounded_cache.borrow_mut().insert(solid, (key, b.clone()));
    b
}

/// The pieces of region solid `solid`, or `None` where it is not read in general position or a
/// term is none this construction makes.
fn construction(sk: &Sketch, solid: usize) -> Option<Result<Bounded, String>> {
    let SolidDef::Region { terms, .. } = &sk.solids[solid].def else { return None };
    // only a term of these shapes, at a stated number with no offset
    let made = |shape: &RegionShape| matches!(shape, RegionShape::Ball { .. } | RegionShape::Cylinder { .. }
        | RegionShape::Cone { .. } | RegionShape::Plane { .. });
    if !terms.iter().all(|t| made(&t.shape) && t.grow.is_none()) {
        return None;
    }
    let field = match super::region::in_general_position(sk, solid)? {
        Ok(f) => f,
        Err(e) => return Some(Err(e)),
    };
    let name = &sk.solids[solid].name;
    let support = field.support()?;
    // the box's middle and how far its corners stand from it
    let middle: V = std::array::from_fn(|k| (support[k][0] + support[k][1]) / 2.0);
    let h = 0.5 * norm(std::array::from_fn(|k| support[k][1] - support[k][0])).max(f64::MIN_POSITIVE);
    let mut keeps = Vec::new();
    let mut cuts = Vec::new();
    for t in terms {
        let Some(lo) = t.lo.read(sk) else {
            return Some(Err(format!("`{name}`: a term reads an unknown nothing determines")));
        };
        let Ok(Some(measure)) = super::region::measure_of(sk, &t.shape) else { return None };
        use crate::solid::field::region::Measure;
        let piece = |keep: bool, shape: Shape| Piece { keep, shape, name: t.name.clone() };
        // the bounds a term sets, each `(within, number)`: `within` the region keeps what lies
        // within the shape at the number, else what lies outside it
        let sides: Vec<(bool, f64)> = match t.cmp {
            Cmp::Le => vec![(true, lo)],
            Cmp::Ge => vec![(false, lo)],
            Cmp::In => vec![(true, t.hi.unwrap_or(lo)), (false, lo)],
        };
        for (within, v) in sides {
            let shape = match measure {
                Measure::Point(_) | Measure::Line { .. } if v <= 0.0 => {
                    // within nothing holds nowhere; outside nothing, everywhere
                    if within { return Some(Err(format!("`{name}` holds nowhere"))) }
                    continue;
                }
                Measure::Point(c) => Shape::Ball { c, r: v },
                Measure::Line { a, d } => {
                    let a = add(a, scale(d, dot(sub(middle, a), d)));
                    Shape::Cylinder { a, d, r: v, reach: 2.0 * h }
                }
                Measure::Cone { a, d } => {
                    if !(v > 0.0 && v < FRAC_PI_2 - 1e-9) {
                        // a cone no narrower than square: the field's
                        return None;
                    }
                    let far = dot(sub(middle, a), d) + h;
                    if far <= 0.0 {
                        if within { return Some(Err(format!("`{name}` holds nowhere"))) }
                        continue;
                    }
                    Shape::Cone { a, d, half: v, reach: far + h }
                }
                Measure::Ordinate { o, n } => {
                    // `n · (x − o) ≤ v`, or `≥ v` turned round: one half-space either way
                    let len = norm(n);
                    if !(len > 0.0) { return None }
                    let (n, e) = (scale(n, 1.0 / len), (dot(n, o) + v) / len);
                    let (n, e) = if within { (n, e) } else { (scale(n, -1.0), -e) };
                    let foot = sub(middle, scale(n, dot(n, middle) - e));
                    let reach = 2.0 * (h + norm(sub(middle, foot)));
                    keeps.push(piece(true, Shape::Half { o: foot, n, reach }));
                    continue;
                }
                Measure::Signed(_) => return None,
            };
            if within { keeps.push(piece(true, shape)) } else { cuts.push(piece(false, shape)) }
        }
    }
    if keeps.is_empty() {
        return Some(Err(format!("`{name}` keeps within no term, which this construction needs")));
    }
    keeps.extend(cuts);
    Some(Ok(Bounded { pieces: keeps }))
}

/// **The gate**, as the lowering table's (`lowering::gate`): the construction stands only where
/// the region's own field agrees with it — every probe a little inside its boundary material by
/// the field, every one a little outside not (`agreement`, sampled by area).
fn gate(sk: &Sketch, solid: usize, b: &Bounded) -> Result<(), String> {
    let brep = crate::brep::recipe::node(&b.node(1.0)?, &Default::default())?;
    let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
    let size = brep.size().max(f64::MIN_POSITIVE);
    let offset = 2e-3 * size;
    let m = crate::brep::mesh::mesh(&brep, offset / 20.0, crate::brep::export::ANGULAR)?;
    let options = super::agreement::Options {
        offset, confirm: offset / 4.0, value_tolerance: offset / 5.0, triangles: 400,
        ..Default::default()
    };
    let report = super::agreement::of_triangles_parallel(&m.pts, &m.tris, &field, 1, &options)?;
    match report.disagreements.first() {
        None => Ok(()),
        Some(d) => Err(format!("its exact construction disagrees with the field at [{:.6}, {:.6}, {:.6}] ({} of {} probes)",
            d.point[0], d.point[1], d.point[2], report.disagreements.len(), report.probes)),
    }
}
