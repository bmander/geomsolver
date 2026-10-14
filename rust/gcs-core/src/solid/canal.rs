//! **A ball swept, built exactly** (#238): a continuous sweep whose tool is a ball and whose motion
//! is one turn about one axis — a rotation, or a screw — sweeps the points within the ball's
//! radius of the path its centre runs: the ball's disc square to the path carried along it, with
//! the ball where the path starts and where it ends (`brep::build::canal`). So the tool need not end
//! clear of anything: its end poses are the construction's own, and a sweep standing alone (a
//! spring) is a solid like any other. The recipe's `canal` node, built by this kernel; held to the
//! sweep's own field before it stands (`gate`), as the lowering table's pieces are. Any other tool
//! or motion stays the generating class's, or the field's.
use crate::json::{object, Json};
use crate::model::{Sketch, SolidDef};
use crate::space::{add, cross, dot, norm, scale, sub};
use std::f64::consts::TAU;
use std::rc::Rc;

type V = [f64; 3];

/// A ball's sweep (model units): the ball of `radius` about `centre` where the roll starts, carried
/// by the screw about the line through `origin` along the unit `axis` — a turn by `angle` and a step
/// of `advance` — and the name of the ball's face, which names every face the tube has.
#[derive(Clone, Debug)]
pub struct Canal {
    centre: V,
    radius: f64,
    origin: V,
    axis: V,
    angle: f64,
    advance: f64,
    face: String,
}

impl Canal {
    /// The recipe's node (lengths times `scale`, to millimetres).
    pub fn node(&self, scale_by: f64) -> Json {
        let v = |p: V| Json::Arr(p.iter().map(|&x| Json::from(x * scale_by)).collect());
        object([("kind", "canal".into()), ("centre", v(self.centre)), ("radius", (self.radius * scale_by).into()),
            ("origin", v(self.origin)), ("axis", Json::Arr(self.axis.iter().map(|&x| x.into()).collect())),
            ("angle", self.angle.into()), ("advance", (self.advance * scale_by).into()),
            ("face", self.face.clone().into())])
    }
}

/// Sweep `swept` built as a ball's tube, where its tool is a ball and its motion one turn about one
/// axis and the tube meets itself nowhere (`None` otherwise, a sweep of another kind): the
/// construction, or why it stands not — its gate refused it.  Remembered against `solid::reads`
/// (`Caches::canal`).
pub fn built(sk: &Sketch, swept: usize) -> Option<Result<Rc<Canal>, String>> {
    if !matches!(sk.solids.get(swept)?.def, SolidDef::Swept { .. }) {
        return None;
    }
    sk.caches.canal.get_or(swept, super::MemoKey::of(sk, swept), || read(sk, swept).map(|c| {
        let c = c?;
        gate(sk, swept, &c)?;
        Ok(Rc::new(c))
    }))
}

/// Whether sweep `i` is built as a ball's tube: not a continuous sweep the generating class or the
/// field answers.
pub fn is_canal(sk: &Sketch, i: usize) -> bool {
    matches!(built(sk, i), Some(Ok(_)))
}

/// The ball's sweep `swept` is, read off its tool's recipe and its motion where both are what the
/// tube needs: `None` where they are not, an error where they cannot be read.
fn read(sk: &Sketch, swept: usize) -> Option<Result<Canal, String>> {
    let SolidDef::Swept { source, motion, .. } = &sk.solids[swept].def else { return None };
    let family = crate::motion::Family::read(sk, *motion as usize).ok()?;
    let turn = family.turning()?;
    let scale_mm = super::cad::millimetres(sk).ok()?;
    // the tool: one node, a ball (millimetres back to the model's units)
    let recipe = super::cad::recipe(sk, *source as usize).ok()?;
    let [n] = recipe.get("nodes")?.arr() else { return None };
    let (c, r) = crate::brep::recipe::ball_of(n, 1e-9 * scale_mm.max(1.0)).ok()??;
    let (c, r) = (c.map(|x| x / scale_mm), r / scale_mm);
    let profile = crate::brep::build::Profile::from_json(n.get("profile")?).ok()?;
    let face = (0..profile.loops.len()).flat_map(|l| (0..profile.loops[l].len()).map(move |k| (l, k)))
        .find(|&(l, k)| matches!(profile.loops[l][k], crate::brep::build::ProfileEdge::Arc { .. }))
        .map_or(String::new(), |(l, k)| profile.name(l, k));
    // where the roll starts, and the turn and the step it takes to its end
    let [from, to] = match super::roll::interval(sk, swept) {
        Ok(roll) => roll,
        Err(e) => return Some(Err(e)),
    };
    let at = match family.at(from) {
        Ok(pose) => pose,
        Err(e) => return Some(Err(e)),
    };
    let centre = at.point(c);
    let tau = to - from;
    let (angle, advance) = (turn.ratio * tau, turn.advance * tau / TAU);
    // the tube meets itself nowhere: its centre's path off the axis by more than the radius, and a
    // helix of more than a turn climbing more than its diameter a turn
    let a = turn.axis;
    let radial = sub(centre, add(turn.origin, scale(a, dot(sub(centre, turn.origin), a))));
    let moves = norm(add(scale(cross(a, radial), angle), scale(a, advance))) > 1e-12 * r.max(1.0);
    if moves && norm(radial) <= r * (1.0 + 1e-9) {
        return None;
    }
    if advance != 0.0 && angle.abs() > TAU && turn.advance.abs() / turn.ratio.abs() <= 2.0 * r {
        return None;
    }
    Some(Ok(Canal { centre, radius: r, origin: turn.origin, axis: a, angle, advance, face }))
}

/// **The gate**, as the lowering table's (`lowering::gate`): the tube stands only where the sweep's
/// own field agrees with it — every probe a little inside its boundary material by the field, every
/// one a little outside not (`agreement`, sampled by area).
fn gate(sk: &Sketch, swept: usize, c: &Canal) -> Result<(), String> {
    let brep = crate::brep::recipe::node(&c.node(1.0), &Default::default())?;
    let field = super::MaterialField::read(sk, swept, super::cad::AXIS_TOLERANCE)?;
    let what = format!("`{}`: the ball's tube disagrees with the sweep's field", sk.solids[swept].name);
    super::agreement::gate(&field, &brep, None, &what)
}
