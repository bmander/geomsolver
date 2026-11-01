//! **A ball swept, built exactly** (#238, #684 D): a continuous sweep whose tool is a ball sweeps
//! the points within the ball's radius of the path its centre runs, whatever the motion turns the
//! ball by. The tube is the kernel's (`brep::swept::canal`, #721 F2c): along a constant twist in
//! closed form, along any other trajectory a face family fitted to its disc, refused by rows B1 and
//! B2 where it would meet itself. What is here is the drawing's reading: the tool's recipe read as
//! a ball, the motion lowered and the roll; the kernel's refusals said in the drawing's words; the
//! recipe's `canal` node; and the tube held to the sweep's own field before it stands (`gate`), as
//! the lowering table's pieces are. Any other tool stays the generating class's, the constant
//! twist's, or the field's.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::Condition;
use crate::brep::recipe::{Spine, Writer};
use crate::brep::swept::canal::{self as kernel, Row, Seen};
use crate::json::{object, Json};
use crate::model::{Sketch, SolidDef};
use crate::motion::Family;
use crate::vocab::num;
use std::rc::Rc;

type V = [f64; 3];

/// A ball's sweep (model units): the ball of `radius` about `centre` where the roll starts, carried
/// along its centre's path (`spine`: a screw's, or the face family its disc makes, `along`), and the
/// name of the ball's face, which names every face the tube has.
#[derive(Clone, Debug)]
pub struct Canal {
    centre: V,
    radius: f64,
    spine: Spine,
    face: String,
}

impl Canal {
    /// The recipe's node, written by `w`.
    pub fn node(&self, w: &Writer) -> Json {
        let mut o = object([("kind", "canal".into()), ("centre", w.point(self.centre)),
            ("radius", w.length(self.radius)), ("face", self.face.clone().into())]);
        match &self.spine {
            &Spine::Screw { origin, axis, angle, advance } => {
                let mm = |p: V| p.map(|x| x * w.scale);
                o.set("origin", w.line_origin(mm(origin), axis));
                o.set("axis", Writer::triple(axis));
                o.set("angle", angle.into());
                o.set("advance", w.length(advance));
            }
            Spine::Tube(walls) => o.set("tube", super::family::json(walls, w)),
        }
        o
    }

    /// Whether the tube is fitted: its centre's path one no closed form carries (#684 D).
    pub fn fitted(&self) -> bool {
        matches!(self.spine, Spine::Tube(_))
    }
}

/// A ball's sweep refused by a row of its own (#684 D): the row, what was seen, and a point of the
/// centre's path where it was (the sweep's coordinates).
#[derive(Clone, Debug)]
pub struct Refusal {
    pub condition: Condition,
    pub message: String,
    pub witness: Option<V>,
}

/// Why a ball's sweep is no tube: a row refuses it, or it could not be read, built or gated.
#[derive(Clone, Debug)]
pub(crate) enum Why {
    Refused(Refusal),
    Failed(String),
}

impl From<String> for Why {
    fn from(m: String) -> Why { Why::Failed(m) }
}

/// Sweep `swept` built as a ball's tube, where its tool is a ball and the tube meets itself nowhere
/// (`None` otherwise: a sweep of another kind, or a tube a row refuses, `refusal`): the
/// construction, or why it stands not — its gate refused it.  Remembered against `solid::reads`
/// (`Caches::canal`).
pub fn built(sk: &Sketch, swept: usize) -> Option<Result<Rc<Canal>, String>> {
    match outcome(sk, swept)? {
        Ok(c) => Some(Ok(c)),
        Err(Why::Failed(m)) => Some(Err(m)),
        Err(Why::Refused(_)) => None,
    }
}

/// Why sweep `swept`, a ball's along a path no closed form carries, is no tube: the row its
/// centre's path fails (`None` for a tube, a sweep whose tool is no ball, and one the constant
/// twist's rows are asked of).
pub fn refusal(sk: &Sketch, swept: usize) -> Option<Refusal> {
    match outcome(sk, swept)? {
        Err(Why::Refused(r)) => Some(r),
        _ => None,
    }
}

fn outcome(sk: &Sketch, swept: usize) -> Option<Result<Rc<Canal>, Why>> {
    if !matches!(sk.solids.get(swept)?.def, SolidDef::Swept { .. }) {
        return None;
    }
    sk.caches.canal.get_or(swept, super::MemoKey::of(sk, swept), || read(sk, swept).map(|c| {
        let c = c?;
        gate(sk, swept, &c)?;
        Ok(Rc::new(c))
    }))
}

/// Whether sweep `i` is built as a ball's tube: not a continuous sweep another class or the field
/// answers.
pub fn is_canal(sk: &Sketch, i: usize) -> bool {
    matches!(built(sk, i), Some(Ok(_)))
}

/// The ball's sweep `swept` is, read off its tool's recipe and its motion where the tool is a ball:
/// `None` where it is not, or where it is a screw's whose tube would meet itself (the constant
/// twist's rows are asked of it instead); why not where its path is refused or cannot be read.
fn read(sk: &Sketch, swept: usize) -> Option<Result<Canal, Why>> {
    let SolidDef::Swept { source, motion, .. } = &sk.solids[swept].def else { return None };
    let family = Family::read(sk, *motion as usize).ok()?;
    let scale_mm = super::cad::millimetres(sk);
    // the tool: one node, a ball (millimetres back to the model's units)
    let recipe = crate::brep::recipe::Recipe::read(&super::cad::recipe(sk, *source as usize).ok()?).ok()?;
    let [n] = &recipe.nodes[..] else { return None };
    let (c, r) = n.node.ball(1e-9 * scale_mm.max(1.0)).ok()??;
    let (c, r) = (c.map(|x| x / scale_mm), r / scale_mm);
    let crate::brep::recipe::Node::Revolve { profile, .. } = &n.node else { return None };
    let profile = crate::brep::build::Profile::of(profile).ok()?;
    let face = (0..profile.loops.len()).flat_map(|l| (0..profile.loops[l].len()).map(move |k| (l, k)))
        .find(|&(l, k)| matches!(profile.loops[l][k], crate::brep::build::ProfileEdge::Arc { .. }))
        .map_or(String::new(), |(l, k)| profile.name(l, k));
    tube(sk, swept, &family, c, r, face).transpose()
}

/// The tube the ball of radius `r` about `c` sweeps under `family` over sweep `swept`'s roll, its
/// faces named `face` (`brep::swept::canal::tube`): `None` where it is a screw's that would meet
/// itself.
fn tube(sk: &Sketch, swept: usize, family: &Family, c: V, r: f64, face: String) -> Result<Option<Canal>, Why> {
    let roll = super::roll::interval(sk, swept)?;
    let tube = kernel::tube(family.path()?, c, r, &face, roll).map_err(|e| said(e, r))?;
    Ok(tube.map(|t| Canal { centre: t.centre, radius: t.radius, spine: t.spine, face }))
}

/// The kernel's refusal of a ball's tube of radius `r`, said in the drawing's words.
fn said(e: kernel::Why, r: f64) -> Why {
    let stops = |s: f64| format!("the ball's centre stops at {} of the roll, where its path has no tangent", num(s));
    match e {
        kernel::Why::Refused(kernel::Refusal { row, seen, samples, witness }) => {
            let said = format!("read at {samples} points of its path");
            let message = match seen {
                Seen::Stops { at } => format!("{} ({said})", stops(at)),
                Seen::Turns { radius, at } => format!("its centre's path turns on a radius of {} at {} of the roll, \
                    within its radius {} ({said})", num(radius), num(at), num(r)),
                Seen::Returns { distance, from, to, spacing } => format!("its centre's path comes back {} from itself \
                    between {} and {} of the roll, within its diameter {} and the {} its points stand apart ({said})",
                    num(distance), num(from), num(to), num(2.0 * r), num(spacing)),
            };
            let condition = match row { Row::Curvature => Condition::Curvature, Row::Return => Condition::Return };
            Why::Refused(Refusal { condition, message, witness: Some(witness) })
        }
        kernel::Why::Stops(s) => Why::Failed(stops(s)),
        kernel::Why::Pieces { most, tolerance } => Why::Failed(format!("the ball's tube needs more than {most} pieces \
            to follow its centre's path within {}", num(tolerance))),
        kernel::Why::Rough { from, to, miss } => Why::Failed(format!("the ball's centre's path does not run smoothly \
            between {} and {} of the roll: the tube's polynomials miss its disc between them by {}", num(from), num(to),
            num(miss))),
        kernel::Why::Path(e) => Why::Failed(e.msg),
    }
}

/// **The gate**, as the lowering table's (`lowering::gate`): the tube stands only where the sweep's
/// own field agrees with it — every probe a little inside its boundary material by the field, every
/// one a little outside not (`agreement`, sampled by area).
fn gate(sk: &Sketch, swept: usize, c: &Canal) -> Result<(), String> {
    let brep = crate::brep::recipe::node(&c.node(&Writer::world(1.0)), &Default::default())?;
    let field = super::MaterialField::read(sk, swept, super::cad::AXIS_TOLERANCE)?;
    let what = format!("`{}`: the ball's tube disagrees with the sweep's field", sk.solids[swept].name);
    super::agreement::gate(&field, &brep, None, &what)
}
