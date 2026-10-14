//! What the core reads off the geometry for a constraint: seeds, inferred arguments,
//! validation and scales.

use super::*;

/// Where a hidden unknown starts when the caller leaves it out — the `Param` counterpart of
/// `infers_arg`: the core reads it off the geometry, so no binding ever has to name a curve
/// parameter.  A document that saved one passes the saved number instead and never comes here.
pub fn seed_param(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> f64 {
    // a place on an envelope starts where the envelope passes nearest the point: the contact's,
    // or the point a tangent is at (#283)
    if let Some((env, first)) = kind.contact_on(SpecKind::Envelope) {
        let at = if kind == CKind::PointOnEnvelope { 0 } else { 2 };
        let (Some(Arg::Ent(e)), Some(Arg::Ent(p))) = (args.get(env), args.get(at)) else { return 0.0 };
        return envelope_place(sk, e.i(), p.i()).map_or(0.0, |x| x[i - first]);
    }
    match (kind, i) {
        // where the curve already comes nearest the thing it is being tied to: a curve can meet
        // a point or a line in several places, and the nearest is the branch the user drew
        (CKind::PointOnSpline, 2) => {
            let (x, y) = sk.point_xy(args[0].ent().i());
            crate::curve::closest(sk, args[1].ent().i(), x, y).0
        }
        (CKind::SplineTangentLine, 2) => {
            let [ax, ay, bx, by] = sk.line_params(args[1].ent().i());
            let g = |p: u32| sk.params[p as usize].value;
            crate::curve::nearest_to_line(sk, args[0].ent().i(), g(ax), g(ay), g(bx), g(by))
        }
        // the language curves' contacts start where their polyline comes nearest — the same
        // readings as a spline's, over the drawn curve rather than a basis
        (CKind::CurveTangentLine, 2) => {
            // nearest the infinite line, since a tangency is about direction
            let [ax, ay, bx, by] = sk.line_params(args[1].ent().i());
            let g = |p: u32| sk.params[p as usize].value;
            let (ax, ay, dx, dy) = (g(ax), g(ay), g(bx) - g(ax), g(by) - g(ay));
            let len = dx.dhypot(dy).max(kernels::MIN_LINE_LEN);
            sk.curve_nearest_by(args[0].ent().i(), |px, py| {
                ((px - ax) * dy - (py - ay) * dx).abs() / len
            })
        }
        // a point in space starts where the curve comes nearest its place in the curve's view
        (CKind::PointOnExtrusion, 2) => {
            let ci = args[1].ent().i();
            let (x, y) = sk.on_view_sheet(sk.world_point(args[0].ent().i()), sk.curve_view(ci));
            sk.curve_nearest_by(ci, |px, py| (px - x).dhypot(py - y))
        }
        (CKind::CurveCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            sk.curve_nearest_by(args[0].ent().i(), |px, py| (px - cx).dhypot(py - cy))
        }
        // the tangent at a point starts where the point is: the contact holding it owns the
        // place, and a statement without one (`Sketch::add` found none yet) starts beside it
        (CKind::SplineTangentAt, 3) => {
            let (x, y) = sk.point_xy(args[2].ent().i());
            crate::curve::closest(sk, args[0].ent().i(), x, y).0
        }
        (CKind::CurveTangentAt, 3) => {
            let (x, y) = sk.point_xy(args[2].ent().i());
            sk.curve_nearest_by(args[0].ent().i(), |px, py| (px - x).dhypot(py - y))
        }
        // a point on a curve starts where the curve passes nearest it
        (CKind::PointOnCurve, 2) => {
            let (x, y) = sk.point_xy(args[0].ent().i());
            sk.curve_nearest_by(args[1].ent().i(), |px, py| (px - x).dhypot(py - y))
        }
        // an osculating circle sits centred a radius off the curve, so the curve point nearest
        // the centre it already has is the place it is asking about
        (CKind::SplineCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            crate::curve::closest(sk, args[0].ent().i(), cx, cy).0
        }
        // a fraction starts where the point stands: its foot on the line, its bearing round the
        // arc — each read where the point stands in space, so a point in another view is read
        // where it is
        (CKind::PointAlongLine | CKind::PointAlongLine3, 2) => {
            let l = &sk.lines[args[1].ent().i()];
            let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
            let (d, w) = (crate::space::sub(b, a), crate::space::sub(sk.world_point(args[0].ent().i()), a));
            let dd = crate::space::dot(d, d);
            if dd > 0.0 { crate::space::dot(w, d) / dd } else { 0.0 }
        }
        (CKind::PointAlongArc | CKind::PointAlongArc3, 2) => {
            let i = args[1].ent().i();
            let (a0, a1) = sk.arc_angles(i);
            let c = sk.round_center(args[1].ent());
            let view = sk.plane_of(c);
            let (cx, cy) = sk.on_view_sheet(sk.world_point(c), view);
            let (px, py) = sk.on_view_sheet(sk.world_point(args[0].ent().i()), view);
            arc_fraction((py - cy).datan2(px - cx), a0, a1 - a0)
        }
        _ => 0.0,
    }
}

/// **An arc's fraction is on the arc** (#248): its turn from `start` to `end` is all there is of
/// it, so a place pinned past either end is a point the arc does not reach.  A line's runs on, as
/// `coincident` on a line always has.  Asked by `validate`, and by the elaborator before it, which
/// says it as a value refused (E040) rather than a relation the model cannot hold.
pub fn on_the_arc(kind: CKind, args: &[Arg], name: &dyn Fn(EntRef) -> String) -> Result<(), String> {
    if !matches!(kind, CKind::PointAlongArc | CKind::PointAlongArc3) {
        return Ok(());
    }
    match args.get(2) {
        Some(Arg::Seed { value, pinned: true })
            if !(-ARC_FRACTION_SLACK..=1.0 + ARC_FRACTION_SLACK).contains(value) =>
        {
            Err(format!(
                "`t == {}` is off {}: a fraction of an arc's turn runs from 0 at its start to 1 \
                 at its end",
                crate::syntax::num(*value),
                name(args[1].ent())
            ))
        }
        _ => Ok(()),
    }
}

/// How far past an end of an arc a pinned fraction may be written and still be read as on it: a
/// number a formula came to (`1 - s` at `s == 1`) is one rounding off, never a design.
const ARC_FRACTION_SLACK: f64 = 1e-12;

/// The fraction of an arc's turn a bearing `phi` stands at, the arc starting at the bearing `a0`
/// and sweeping `sweep` counter-clockwise: on `[0, 1]`, a bearing off the arc read at the end it
/// is nearer by angle.
fn arc_fraction(phi: f64, a0: f64, sweep: f64) -> f64 {
    const TURN: f64 = 2.0 * std::f64::consts::PI;
    let past = (phi - a0).rem_euclid(TURN);
    if past <= sweep {
        return past / sweep;
    }
    if past - sweep <= TURN - past { 1.0 } else { 0.0 }
}

/// An entity slot the core fills when the caller leaves it out — the entity counterpart of
/// `seed_param`: a projection's planes are its points' memberships, and nobody writes them.
/// `Err` is the reason it cannot, in the words the caller reports, naming entities by `name`.
pub fn infer_entity(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    i: usize,
    name: &dyn Fn(EntRef) -> String,
) -> Result<EntRef, String> {
    match (kind, i) {
        (CKind::Project | CKind::ProjectSolved, 2 | 3) => {
            let p = args[i - 2].ent();
            sk.plane_of(p.i()).map(EntRef::plane).ok_or_else(|| {
                format!(
                    "{} is on no plane, so `project` cannot say which view it is in",
                    name(p)
                )
            })
        }
        // an ordinate's direction said in a word: an axis of the view both points are drawn in.
        // Against a plane the elaborator names the plane's axis, so a word left to infer is the
        // view's — `u` and `v` as much as `right` and `up` — and `n` is a plane's, with no plane
        // to read it off
        (CKind::Ordinate | CKind::Level, 2) => {
            let w = ordinate_word(kind, args);
            let toward = Toward::of(w).ok_or_else(|| {
                format!("an ordinate names its direction: an axis, a line, or one of {}",
                    crate::syntax::one_of(&ALONG_WORDS))
            })?;
            let (p, q) = (args[0].ent(), args[1].ent());
            match (toward.in_view(), shared_view(sk, p.i(), q.i())) {
                (Some(Toward::PageV), Some(v)) => Ok(EntRef::axis(sk.planes[v].v as usize)),
                (Some(_), Some(v)) => Ok(EntRef::axis(sk.planes[v].u as usize)),
                (Some(_), None) => Err(format!(
                    "`{w}` is a direction of the view both points are drawn in, and {} and {} \
                     are not drawn in one: name the axis",
                    name(p),
                    name(q)
                )),
                (None, _) => Err(format!("`{w}` is a direction of a plane, which the ordinate is \
                    measured from: `{} distance(d, along: {w}) P`", name(q))),
            }
        }
        _ => Err(format!("{} leaves nothing for the core to infer in slot {i}", kind.name())),
    }
}

/// What an ordinate (`Ordinate`, `Level`) must be to mean something — `validate`'s arm for the
/// two.  A plane's own word is measured from that plane's origin along its own direction; a
/// level of a point with itself, and an ordinate of two points of one view along that view's
/// normal, are identically nothing; a reading in space needs its points' places, and a drawn
/// line a length.
fn validate_ordinate(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    name: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    let (p, q, t) = (args[0].ent(), args[1].ent(), args[2].ent());
    let word = ordinate_word(kind, args);
    if let Some(toward) = Toward::of(word).filter(|t| t.of_plane()) {
        let datum = sk.plane_of_origin(p.i());
        let on = datum.is_some_and(|pl| match toward {
            Toward::PlaneU => t == EntRef::axis(sk.planes[pl].u as usize),
            Toward::PlaneV => t == EntRef::axis(sk.planes[pl].v as usize),
            _ => t == EntRef::plane(pl),
        });
        // or, `u` and `v` with no plane operand, the axis of the view both points are drawn in
        let in_view = shared_view(sk, p.i(), q.i()).is_some_and(|v| match toward {
            Toward::PlaneU => t == EntRef::axis(sk.planes[v].u as usize),
            Toward::PlaneV => t == EntRef::axis(sk.planes[v].v as usize),
            _ => false,
        });
        if !on && !in_view {
            let view = match toward {
                Toward::PlaneN => "",
                _ => ", or the view's both points are drawn in",
            };
            return Err(format!(
                "`along: {word}` is a plane's own direction, measured from its origin: \
                 `{} distance(d, along: {word}) P`{view}",
                name(q)
            ));
        }
    }
    if p == q {
        return Err(format!("{} is level with itself along every direction", name(p)));
    }
    let form = ordinate_form(sk, p.i(), q.i(), t, word);
    let mut a = args.to_vec();
    if let Some(i) = kind.form_slot() {
        a[i] = Arg::Int(form as i64);
    }
    let c = Constraint::new(kind, a);
    for e in c.lifted_points(sk) {
        if !sk.has_place(e) {
            return Err(format!(
                "{} is a point of a 2D sketch, with no place in space to relate",
                name(EntRef::point(e))
            ));
        }
    }
    if t.kind == EntKind::Line {
        let ln = &sk.lines[t.i()];
        let [a, b] = [ln.p1, ln.p2].map(|e| sk.world_point(e as usize));
        if crate::space::norm(crate::space::sub(b, a)) <= kernels::MIN_LINE_LEN {
            return Err(format!("{} has no length to measure along", name(t)));
        }
    }
    // two points of one view, along its normal: the row is identically zero.  An axis held
    // square to a view held still is the same statement by value, which the count of rows
    // cannot see
    if let Some(v) = shared_view(sk, p.i(), q.i()) {
        let square = match t.kind {
            EntKind::Plane => t.i() == v,
            EntKind::Axis => {
                let d = &sk.axes[t.i()].d;
                let held = d.iter().all(|&k| sk.params[k as usize].fixed) && sk.plane_fixed(v);
                let dir = d.map(|k| sk.params[k as usize].value);
                let n = sk.basis(v).normal();
                held && crate::space::norm(crate::space::cross(dir, n))
                    <= 1e-9 * crate::space::norm(dir).max(1e-300)
            }
            _ => false,
        };
        if square {
            return Err(format!(
                "{} and {} are drawn in {}, so how far apart they stand along its normal is not \
                 a question: every point of a view is on it",
                name(p),
                name(q),
                sk.plane_name(v)
            ));
        }
    }
    Ok(())
}

/// What a contact's parameter runs along — a spline, a language curve, a line's or an arc's run,
/// whichever the contact names — and `None` for a kind whose own unknown runs along nothing.
pub(crate) fn contact_carrier(kind: CKind, args: &[Arg]) -> Option<EntRef> {
    let (e, _) = kind.place_on()?;
    Some(args[e].ent())
}

/// A shared parameter (`t == s`) is **one place along one curve**: every contact owning it
/// stands on the same curve, so the unknown has one interval, one seam and one speed.  Two
/// curves could not agree on any of the three, and a contact's own unknown that runs along no
/// curve has nothing to share.
fn shared_on_one_curve(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    named: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    let Some(name) = args.iter().find_map(|a| match a {
        Arg::Shared { name, .. } => Some(name),
        _ => None,
    }) else {
        return Ok(());
    };
    let Some(here) = contact_carrier(kind, args) else {
        return Err(format!("`{name}` names a contact's place along a curve, and this \
            relation's unknown runs along none"));
    };
    if sk.free_vars.get(name).is_some_and(|&p| !sk.params[p as usize].fixed) {
        return Err(format!("`{name}` is a free variable a dimension reads, and a place along a \
            curve is another unknown"));
    }
    // **a fraction is one place along any run** (#248): `[0, 1]` from one end to the other on
    // every line and arc alike, so two of them sharing it is similar figures — but not a place
    // along a curve's own parameter, whose interval is the curve's
    let fraction = |e: EntRef| matches!(e.kind, EntKind::Line | EntKind::Arc);
    match sk.shared.get(name) {
        Some(place) if fraction(place.along) && fraction(here) => Ok(()),
        Some(place) if place.along != here => Err(format!(
            "`{name}` is a place along {}, and a contact on {} cannot share it",
            named(place.along),
            named(here)
        )),
        _ => Ok(()),
    }
}

/// What a kind refuses once its arguments are all in — the checks that need the sketch, which
/// the type check on the spec cannot make.  One rule for the elaborator, the document readers,
/// the FFI and the Rust constructors alike; `name` is what the caller calls an entity.
pub fn validate(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    name: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    shared_on_one_curve(sk, kind, args, name)?;
    on_the_arc(kind, args, name)?;
    // an envelope a contact holds to is one whose source and motion the solve can read (#283),
    // and a circle osculating it is drawn in a plane
    if kind.envelope_contact().is_some() {
        let c = Constraint::new(kind, args.to_vec());
        let e = c.envelope_of().ok_or("an envelope contact names an envelope")?;
        crate::envelope::held::Parts::of(sk, e.i())
            .map_err(|why| format!("the solve cannot hold anything to {}: {why}", name(e)))?;
        if kind == CKind::EnvelopeCurvatureAt && sk.plane_of(sk.round_center(args[1].ent())).is_none() {
            return Err(format!("{} is drawn in no plane, so it has no section of {} to osculate",
                               name(args[1].ent()), name(e)));
        }
    }
    match kind {
        // a curvature reads the curve's second derivative, which a trace gives exactly only
        // where every row of its block has a Taylor form (`taylor.rs`): a residual by difference
        // would solve to a slightly wrong circle and call it right
        CKind::CurveCurvature => {
            let cv = &sk.curves[args[0].ent().i()];
            if let crate::model::CurveBody::Trace(l) = &sk.curve_defs[cv.def as usize].body {
                if let Some(kernel) = l.without_form() {
                    return Err(format!(
                        "{} is traced through a `{kernel}`, whose second derivative is not \
                         written, so the curve has no curvature to state a circle against",
                        name(args[0].ent())
                    ));
                }
            }
            Ok(())
        }
        CKind::Project | CKind::ProjectSolved => {
            let (pa, pb) = (args[2].ent(), args[3].ent());
            if pa == pb {
                return Err(format!(
                    "both points are on {}, and one view relates nothing to itself",
                    name(pa)
                ));
            }
            // two fixed planes are parallel or not now and for ever; where either is solved,
            // whether they came out parallel is a question for after the solve (E065)
            let solved = |e: EntRef| !sk.plane_fixed(e.i());
            let basis = |e: EntRef| sk.basis(e.i());
            if !solved(pa) && !solved(pb)
                && crate::plane::fold_line(&basis(pa), &basis(pb)).is_none()
            {
                return Err(format!(
                    "{} and {} are parallel, so no fold line relates their views",
                    name(pa),
                    name(pb)
                ));
            }
            Ok(())
        }
        CKind::Ordinate | CKind::Level => validate_ordinate(sk, kind, args, name),
        k if k.spatial() => {
            let c = Constraint::new(kind, args.to_vec());
            for p in c.lifted_points(sk) {
                if !sk.has_place(p) {
                    return Err(format!(
                        "{} is a point of a 2D sketch, with no place in space to relate",
                        name(EntRef::point(p))
                    ));
                }
            }
            if let Some((i, _, _)) = c.dimensions().first() {
                if k.magnitude() && matches!(args[*i], Arg::Num(v) if v < 0.0) {
                    return Err("a distance in space is a magnitude, and is never negative".into());
                }
            }
            // a line in space needs a direction, and two of them a common perpendicular
            let lines: Vec<EntRef> =
                c.entities().into_iter().filter(|e| e.kind == EntKind::Line).collect();
            for &l in &lines {
                let ln = &sk.lines[l.i()];
                let [a, b] = [ln.p1, ln.p2].map(|p| sk.world_point(p as usize));
                if crate::space::norm(crate::space::sub(b, a)) <= kernels::MIN_LINE_LEN {
                    return Err(format!("{} has no length in space", name(l)));
                }
            }
            // a view's own points are on it by construction: the row would be identically zero
            let own = |e: EntRef| {
                c.lifted_points(sk).iter().all(|&p| sk.plane_of(p) == Some(e.i()))
            };
            if matches!(k, CKind::PointOnPlane | CKind::LineOnPlane)
                && own(args[1].ent())
            {
                return Err(format!(
                    "{} is drawn in {}, so where it stands along that view's normal is not a \
                     question: every point of a view is on it",
                    name(args[0].ent()),
                    name(args[1].ent())
                ));
            }
            if k == CKind::LineLine3
                && skew_parallel(sk, lines[0], lines[1])
            {
                return Err(format!(
                    "{} and {} are parallel in space, and parallel lines have no common \
                     perpendicular to measure",
                    name(lines[0]),
                    name(lines[1])
                ));
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

/// Whether two lines stand parallel as drawn, with no common perpendicular to measure.  Read off
/// the drawn pose (`Sketch::world_point`), which is where the hidden points are seeded, so it is
/// the same reading before and after the add.
fn skew_parallel(sk: &Sketch, l1: EntRef, l2: EntRef) -> bool {
    use crate::space::{cross, norm, sub};
    let ends = |l: EntRef| {
        let l = &sk.lines[l.i()];
        (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
    };
    let ((a, b), (c, d)) = (ends(l1), ends(l2));
    let (e1, e2) = (sub(b, a), sub(d, c));
    norm(cross(e1, e2)) <= crate::plane::PARALLEL_TOL * norm(e1) * norm(e2)
}

/// A non-entity argument the core reads off the geometry, for the kinds whose omitted value is
/// the geometry's and not a constant: an ordinate's form, and whether a skew distance has a
/// side to leave unpinned.  `None` for every other slot, which keeps what the caller or the
/// defaults gave it.
pub fn infer_value(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> Option<Arg> {
    if kind.form_slot() == Some(i) {
        let ent = |j: usize| match args.get(j) {
            Some(Arg::Ent(e)) => Some(*e),
            _ => None,
        };
        let (p, q, t) = (ent(0)?, ent(1)?, ent(2)?);
        let word = ordinate_word(kind, args);
        return Some(Arg::Int(ordinate_form(sk, p.i(), q.i(), t, word) as i64));
    }
    match (kind, i) {
        // no side read off the seed: the distance is a magnitude (#217); two parallel lines,
        // which have no common perpendicular, are refused by `validate`
        (CKind::LineLine3, 3) => {
            (!skew_parallel(sk, args[0].ent(), args[1].ent())).then_some(Arg::Int(0))
        }
        _ => None,
    }
}

/// The world length one unit of a hidden unknown is worth — see `Param::scale`.  Read off the
/// geometry at the moment the constraint is added; it preconditions the step, so an estimate
/// that drifts as the sketch moves costs convergence rate, never correctness.
pub fn param_scale(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> f64 {
    // Whichever family the contact runs along, the question is the same one, so it is asked once
    // and answered by the entity the slot actually names.  A hidden unknown that runs along
    // nothing is a length already.
    match kind.place_on() {
        Some((e, t)) if t == i => contact_speed(sk, args[e].ent()),
        _ => 1.0,
    }
}

/// The world length one unit of a contact's parameter is worth, whichever family it runs along
/// — the one answer, so the seed `Sketch::add` records and the scale `System::new` compiles
/// against cannot come from two different rules.
pub fn contact_speed(sk: &Sketch, e: EntRef) -> f64 {
    match e.kind {
        // a free curve runs over [0, 1] at its length's speed; another curve's parameter is its own
        EntKind::Curve => sk.curves[e.i()].length.map_or(1.0, |l| sk.params[l as usize].value.abs().max(1e-9)),
        // a fraction is worth the whole run: a line's length, an arc's along itself
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            crate::space::distance(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize)).max(1e-9)
        }
        EntKind::Arc => sk.arc_length(e.i()).max(1e-9),
        _ => crate::curve::speed(sk, e.i()),
    }
}

/// Where on envelope `e` a solve holding point `p` to it starts (#283): `envelope::held::seed`
/// over the envelope's parts as the drawing stands, remembered for the last question, since a
/// contact's three unknowns ask it three times.
fn envelope_place(sk: &Sketch, e: usize, p: usize) -> Option<[f64; 3]> {
    use crate::envelope::held::{seed, Contact, Parts};
    use std::cell::RefCell;
    thread_local! {
        static LAST: RefCell<Option<(Vec<f64>, [f64; 3])>> = const { RefCell::new(None) };
    }
    let parts = Parts::of(sk, e).ok()?;
    let k = parts.consts(Contact::Point);
    let values = parts.values(sk);
    let at = sk.lifted(p);
    let roll = sk.envelopes[e].roll;
    let key: Vec<f64> = k.iter().chain(&values).chain(&at).chain(&roll).copied().collect();
    if let Some(x) = LAST.with(|l| l.borrow().as_ref().filter(|(k, _)| *k == key).map(|(_, x)| *x)) {
        return Some(x);
    }
    let x = seed(&k, &values, at, roll)?;
    LAST.with(|l| *l.borrow_mut() = Some((key, x)));
    Some(x)
}
