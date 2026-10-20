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
    // a point on a meshing starts at its phase (pinned, seeded, or 0 within the interval), no lag,
    // and each flank's place nearest the point seen from its member there, the contact solved
    // from those (#283): the contact's point, or the point a line along its normal or its path
    // is at (#345), until a contact holding that point hands the line its own
    if let Some((mesh, first)) = kind.contact_on(SpecKind::Meshing) {
        let at = if kind == CKind::PointOnMeshing { 0 } else { 2 };
        let (Some(Arg::Ent(p)), Some(Arg::Ent(m))) = (args.get(at), args.get(mesh)) else {
            return 0.0;
        };
        let Some(e) = sk.meshings.get(m.i()) else { return 0.0 };
        let [a, b] = e.roll.radians();
        let phase = match args.get(first) {
            Some(Arg::Seed { value, .. }) => *value,
            _ if a <= 0.0 && 0.0 <= b => 0.0,
            _ => ((a + b) / 2.0).to_degrees(),
        };
        return meshing_place(sk, m.i(), sk.lifted(p.i()), phase).map_or(0.0, |x| x[i - first]);
    }
    // a place on a traced body starts where the body passes nearest the point (#326): the
    // contact's, or the point a tangent is at
    if let Some((tr, first)) = kind.contact_on(SpecKind::Traced) {
        let at = if kind == CKind::PointOnTraced { 0 } else { 2 };
        let (Some(Arg::Ent(e)), Some(Arg::Ent(p))) = (args.get(tr), args.get(at)) else { return 0.0 };
        return crate::traced::place_of(sk, e.i(), sk.lifted(p.i())).map_or(0.0, |x| x[i - first]);
    }
    // a rate's slot is the contact's, which hands it its own (`Sketch::add`): until then it
    // starts where that contact's stands (#365)
    if matches!(kind, CKind::Rate | CKind::RateDistance) {
        let (Some(Arg::Ent(on)), Some(Arg::Ent(at))) = (args.first(), args.get(2)) else { return 0.0 };
        return sk.holding(at.i(), *on).and_then(|p| p.first().copied())
            .map_or(0.0, |p| sk.params[p as usize].value);
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
        // a point a motion carries starts at the roll that carries its point nearest where it
        // stands (#365), as a point on a curve starts where the curve passes nearest it: at roll
        // 0 it starts on the point carried, which can be as far from the roll drawn as from its
        // mirror — a rolling circle's point seen 50 out starts halfway between the epicycloid's
        // two places there, and a whole solve holding it to the path too settles between them
        // as readily as on the one drawn
        (CKind::CarriedPoint, 3) => carried_roll(sk, args).unwrap_or(0.0),
        _ => 0.0,
    }
}

/// How many rolls over a whole turn a carried point's seed is sampled at (`carried_roll`).
const ROLL_SAMPLES: usize = 720;

/// The roll, in degrees within half a turn of 0, at which motion `args[2]` carries point
/// `args[1]` nearest where `args[0]` stands, sampled `ROLL_SAMPLES` times over the turn — 0
/// where no roll sampled is nearer.  `None` where the solve reads no such motion.
fn carried_roll(sk: &Sketch, args: &[Arg]) -> Option<f64> {
    use crate::envelope::held::{carried, Moving};
    let (Some(Arg::Ent(p)), Some(Arg::Ent(q)), Some(Arg::Ent(m))) =
        (args.first(), args.get(1), args.get(2)) else { return None };
    let moving = Moving::of_any(sk, m.i()).ok()?;
    let values = moving.values(sk);
    let (at, x) = (sk.lifted(p.i()), sk.lifted(q.i()));
    let gap = |deg: f64| {
        carried(&moving.steps, &values, deg.to_radians(), x)
            .map_or(f64::INFINITY, |(w, _)| crate::space::norm(crate::space::sub(w, at)))
    };
    let step = 360.0 / ROLL_SAMPLES as f64;
    let (mut best, mut near) = (0.0, gap(0.0));
    for k in 0..=ROLL_SAMPLES {
        let t = -180.0 + k as f64 * step;
        let g = gap(t);
        if g < near {
            (best, near) = (t, g);
        }
    }
    near.is_finite().then_some(best)
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
                crate::vocab::num(*value),
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
        // what the contact holding a rate's point holds it on, whose slot the rate names (#365)
        (CKind::Rate | CKind::RateDistance, 0) => {
            let at = args[2].ent();
            let over = rate_over(kind, args);
            match rate_carriers(sk, at.i(), over).as_slice() {
                [one] => Ok(*one),
                [] => Err(format!(
                    "`{over}` is no formal here, and no contact holding {} owns a slot `{over}`: a \
                     rate runs over a formal its body is given its running number in, or over the \
                     slot of the contact holding its point — `rate(p over t)` of `p coincident(t == \
                     …) g`, or of `p coincident(under: m, t == …) q`",
                    name(at)
                )),
                many => Err(format!(
                    "{} is held by {} contacts with a slot `{over}`, on {}: say which with its \
                     tangent, `g tangent(at: {}) l`",
                    name(at),
                    many.len(),
                    many.iter().map(|&e| name(e)).collect::<Vec<_>>().join(" and "),
                    name(at)
                )),
            }
        }
        (CKind::Project | CKind::ProjectSolved, 2 | 3) => {
            let p = args[i - 2].ent();
            sk.plane_of(p.i()).map(EntRef::frame).ok_or_else(|| {
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
                    crate::vocab::one_of(&ALONG_WORDS))
            })?;
            let (p, q) = (args[0].ent(), args[1].ent());
            match (toward.in_view(), shared_view(sk, p.i(), q.i())) {
                (Some(Toward::PageV), Some(v)) => Ok(EntRef::axis(sk.frames[v].v as usize)),
                (Some(_), Some(v)) => Ok(EntRef::axis(sk.frames[v].u as usize)),
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
            Toward::PlaneU => t == EntRef::axis(sk.frames[pl].u as usize),
            Toward::PlaneV => t == EntRef::axis(sk.frames[pl].v as usize),
            _ => t == EntRef::frame(pl),
        });
        // or, `u` and `v` with no plane operand, the axis of the view both points are drawn in
        let in_view = shared_view(sk, p.i(), q.i()).is_some_and(|v| match toward {
            Toward::PlaneU => t == EntRef::axis(sk.frames[v].u as usize),
            Toward::PlaneV => t == EntRef::axis(sk.frames[v].v as usize),
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
            EntKind::Frame => t.i() == v,
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
/// or a curve traced in space (#363 S1), whichever the contact names — and `None` for a kind
/// whose own unknown runs along nothing.
pub(crate) fn contact_carrier(kind: CKind, args: &[Arg]) -> Option<EntRef> {
    let (e, _) = match kind {
        CKind::PointOnTraced => kind.contact_on(SpecKind::Traced)?,
        _ => kind.place_on()?,
    };
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

/// The name a rate's slot is written by (#365): its `over`.
fn rate_over(kind: CKind, args: &[Arg]) -> &str {
    let i = if kind == CKind::RateDistance { 5 } else { 4 };
    match args.get(i) {
        Some(Arg::Str(s)) => s.as_str(),
        _ => "",
    }
}

/// What the contacts holding point `at` that own a slot named `over` hold it on (#365): a spline's,
/// a curve's, a meshing's or a carried point's `t`, a traced curve's `u`, a traced surface's `u`
/// or `s`.  What a rate written
/// `rate(at over …)` is of, where it is one.
pub fn rate_carriers(sk: &Sketch, at: usize, over: &str) -> Vec<EntRef> {
    let mut found: Vec<EntRef> = Vec::new();
    for c in &sk.constraints {
        let Some((a, on, _)) = c.holds_on() else { continue };
        let Some(src) = RateSource::of(on.kind) else { continue };
        let surface = src == RateSource::Traced && sk.traced.get(on.i()).is_some_and(|t| t.formals() == 2);
        let slot = src.slot() == over || (surface && over == "s");
        if a == at && slot && src != RateSource::Envelope && !found.contains(&on) {
            found.push(on);
        }
    }
    found
}

/// What a rate of a contact refuses (#365): a word its source has nothing for — a curve's normal,
/// an envelope's rate — a slot by another name than its contact's, and what it is held against
/// of the wrong kind: a line or an axis for a rate, a normal or a line touching, a plane for a
/// plane, a circle drawn in a plane for a curvature, and in a view a line drawn there.
fn validate_rate(sk: &Sketch, kind: CKind, args: &[Arg], name: &dyn Fn(EntRef) -> String)
    -> Result<(), String> {
    use crate::contact_rate::Word;
    let c = Constraint::new(kind, args.to_vec());
    let Some(src) = c.rate_source() else {
        return Err("a rate is of a point a contact holds: on a spline, a curve, a body traced in \
                    space, an envelope or a meshing, or carried by a motion".to_string());
    };
    let (on, at, to) = (args[0].ent(), args[2].ent(), args[1].ent());
    let word = c.rate_word();
    let rated = matches!(word, Word::Parallel | Word::Square | Word::Along);
    let surface = match src {
        RateSource::Traced => sk.traced.get(on.i()).is_some_and(|t| t.formals() == 2),
        RateSource::Envelope => true,
        _ => false,
    };
    // what the source has for the word
    let has = match src {
        RateSource::Spline | RateSource::Curve | RateSource::Carried => rated,
        RateSource::Traced => rated || (surface != (word == Word::Centre)),
        RateSource::Meshing => rated || word == Word::Normal,
        RateSource::Envelope => !rated,
    };
    if !has {
        return Err(match src {
            RateSource::Traced if surface => format!("{} is a surface, which has no one centre of \
                curvature at a point: the circle osculating its section by a plane is `curvature` \
                of a circle", name(on)),
            RateSource::Traced => format!("{} is a curve in space, which has a tangent at a point \
                and no normal or section this relation reads: a surface has them; its curvature is \
                a centre, `curvature(at:)` of a point", name(on)),
            RateSource::Envelope => format!("{} is an envelope, whose place no rate runs over: its \
                normal, a plane or a line touching it, and a section's circle are read there",
                name(on)),
            RateSource::Meshing => format!("{} is a meshing: its path of contact and its common \
                normal are read at a point it holds", name(on)),
            _ => format!("{} has a rate at a point it holds, and no normal or curvature this \
                relation reads", name(on)),
        });
    }
    if rated {
        let over = rate_over(kind, args);
        let slots: &[&str] = if surface { &["u", "s"] } else { &[src.slot()] };
        if !slots.contains(&over) {
            return Err(format!("the contact holding {} on {} runs over {}, not `{over}`",
                               name(at), name(on), crate::vocab::one_of(slots)));
        }
    }
    let against = match word {
        Word::Plane => to.kind == EntKind::Frame,
        Word::Curvature => matches!(to.kind, EntKind::Circle | EntKind::Arc),
        Word::Centre => to.kind == EntKind::Point,
        _ => matches!(to.kind, EntKind::Line | EntKind::Axis),
    };
    if !against {
        return Err(format!("{} is not what this reads at {}: {}", name(to), name(at), match word {
            Word::Plane => "a plane",
            Word::Curvature => "a circle",
            Word::Centre => "a point",
            _ => "a line or an axis",
        }));
    }
    if word == Word::Curvature && sk.plane_of(sk.round_center(to)).is_none() {
        return Err(format!("{} is drawn in no plane, so it has no section of {} to osculate",
                           name(to), name(on)));
    }
    match src {
        RateSource::Spline | RateSource::Curve => {
            let drawn_there = to.kind == EntKind::Line && sk.lines.get(to.i())
                .is_some_and(|l| sk.plane_of(l.p1 as usize) == sk.plane_of(at.i()));
            if !drawn_there {
                return Err(format!("{} is drawn in a view, and its rate is held against a line \
                                    drawn there, not {}", name(on), name(to)));
            }
        }
        RateSource::Traced => {
            if sk.traced.get(on.i()).is_none_or(|t| t.body.n_formals() == 0) {
                return Err(format!("{} is a traced body the solve cannot read", name(on)));
            }
        }
        RateSource::Meshing => {
            crate::envelope::meshing::Parts::of(sk, on.i())
                .map_err(|why| format!("the solve cannot read {}: {why}", name(on)))?;
        }
        RateSource::Carried => {
            crate::envelope::held::Moving::of_any(sk, on.i())
                .map_err(|why| format!("the solve cannot read {}: {why}", name(on)))?;
        }
        RateSource::Envelope => {
            crate::envelope::held::Parts::of(sk, on.i())
                .map_err(|why| format!("the solve cannot read {}: {why}", name(on)))?;
        }
    }
    Ok(())
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
    // a point of the drawing followed along a parameter is held to through the locus its
    // drawing's moving part is lowered to (#398 A, `followed::lower`): one not lowered is drawn,
    // and held to by nothing
    let unlowered = |e: &EntRef| e.kind == EntKind::Curve && e.i() < sk.curves.len()
        && matches!(sk.curve_defs[sk.curves[e.i()].def as usize].body,
                    crate::model::CurveBody::Followed(None));
    // a profile cut under a frame the drawing moves is read off the march after the solve (#398 F)
    let tabled = |e: &EntRef| e.kind == EntKind::Curve && e.i() < sk.curves.len()
        && matches!(&sk.curve_defs[sk.curves[e.i()].def as usize].body,
                    crate::model::CurveBody::Envelope(g) if !crate::generate::tabled(&g.flat).is_empty());
    if let Some(e) = args.iter().filter_map(|a| match a { Arg::Ent(e) => Some(e), _ => None })
        .find(|e| tabled(e)) {
        return Err(format!("{} is cut under a frame the drawing moves, read off its march after \
                            the solve: a relation the solve holds reads no march (#398)", name(*e)));
    }
    if let Some(e) = args.iter().filter_map(|a| match a { Arg::Ent(e) => Some(e), _ => None })
        .find(|e| unlowered(e)) {
        let over = &sk.curve_defs[sk.curves[e.i()].def as usize].param;
        return Err(format!("{} is traced as `{over}` runs, and is not lowered to a curve the \
                            solve can hold to (#398)", name(*e)));
    }
    shared_on_one_curve(sk, kind, args, name)?;
    on_the_arc(kind, args, name)?;
    // an envelope a contact holds to is one whose source and motion the solve can read (#283),
    // and a circle osculating it is drawn in a plane
    if kind.envelope_contact().is_some() {
        let c = Constraint::new(kind, args.to_vec());
        let e = c.envelope_of().ok_or("an envelope contact names an envelope")?;
        crate::envelope::held::Parts::of(sk, e.i())
            .map_err(|why| format!("the solve cannot hold anything to {}: {why}", name(e)))?;
    }
    // a meshing a point is held to, or a line at a point held there, is one whose flanks and
    // motion the solve can read (#283, #345)
    if kind.meshing_contact().is_some() {
        let c = Constraint::new(kind, args.to_vec());
        let m = c.meshing_of().ok_or("a contact with a meshing names a meshing")?;
        crate::envelope::meshing::Parts::of(sk, m.i())
            .map_err(|why| format!("the solve cannot hold anything to {}: {why}", name(m)))?;
    }
    // a body traced in space a contact holds to is one the solve can read (#326)
    if kind.traced_contact().is_some() {
        let c = Constraint::new(kind, args.to_vec());
        let t = c.traced_of().ok_or("a contact with a traced body names one")?;
        if t.i() >= sk.traced.len() || sk.traced[t.i()].body.n_formals() == 0 {
            return Err(format!("{} is a traced body the solve cannot read", name(t)));
        }
    }
    // a flank's regularity is read where the solve can read the envelope, the tool and the blank
    // (#345)
    if kind == CKind::RegularWithin {
        Constraint::new(kind, args.to_vec()).regular_reading(sk)
            .map_err(|why| format!("the solve cannot read the flank's regularity: {why}"))?;
    }
    // a point is carried by a motion the solve can read (#326)
    if kind.reads_motion() {
        let c = Constraint::new(kind, args.to_vec());
        let m = c.motion_of().ok_or("a point is carried by a motion")?;
        crate::envelope::held::Moving::of_any(sk, m.i())
            .map_err(|why| format!("the solve cannot read {}: {why}", name(m)))?;
    }
    if matches!(kind, CKind::Rate | CKind::RateDistance) {
        validate_rate(sk, kind, args, name)?;
    }
    match kind {
        // a curvature reads the curve's second derivative, which a trace gives exactly only
        // where every row of its block has a Taylor form (`taylor.rs`): a residual by difference
        // would solve to a slightly wrong circle and call it right
        CKind::CurveCurvature => {
            let cv = &sk.curves[args[0].ent().i()];
            if let Some(l) = sk.curve_defs[cv.def as usize].body.locus() {
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
        // a map's source frame has a place for every point of its plane only where it spans it
        // (#357): three points standing in a line, or two that are one, have none, and its rows
        // would divide by zero.  Asked of a frame held where it stands; one the solve moves is
        // the solve's
        CKind::Mapped | CKind::MappedSimilar | CKind::MappedCircle => {
            let frame = kind.map_sides().expect("a map has sides").frame();
            let src: Vec<usize> = frame.iter().map(|&i| args[i].ent().i()).collect();
            if !src.iter().all(|&p| sk.point_held(p)) {
                return Ok(());
            }
            let (f, t) = (sk.point_xy(src[0]), sk.point_xy(src[1]));
            let d1 = [t.0 - f.0, t.1 - f.1];
            let l1 = d1[0].hypot(d1[1]);
            let pt = |i: usize| name(EntRef::point(src[i]));
            if l1 <= kernels::MIN_LINE_LEN {
                return Err(format!(
                    "{} and {} are one point, so the map has no frame",
                    pt(0),
                    pt(1)
                ));
            }
            if let Some(&h) = src.get(2) {
                let h = sk.point_xy(h);
                let d2 = [h.0 - f.0, h.1 - f.1];
                let det = d1[0] * d2[1] - d1[1] * d2[0];
                if det.abs() <= 1e-9 * l1 * d2[0].hypot(d2[1]).max(l1) {
                    return Err(format!(
                        "{}, {} and {} stand in a line, so the map has no place for a point off \
                         it: the source frame is three points not in a line",
                        pt(0), pt(1), pt(2)
                    ));
                }
            }
            Ok(())
        }
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

/// How many seeds of each kind are remembered (`Recent`).
const SEEDS_KEPT: usize = 32;

/// **The last few seeds, remembered** (#283): a seed of a place on an envelope or a meshing is a
/// search over a grid of thousands of its points, and the same question comes again — a
/// contact's unknowns each ask it, the elaborator's passes ask its contacts again, a solve steered
/// by a bound passes its blocks again from where the first began, and a block pass seeds each
/// contact its block holds as the blocks come up.  An answer is a function of its question
/// alone, so remembered it is the same to the bit; the newest `SEEDS_KEPT`, each thread its own.
struct Recent<T>(Vec<(Vec<f64>, T)>);

impl<T: Copy> Recent<T> {
    fn get(&self, key: &[f64]) -> Option<T> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, x)| *x)
    }

    fn put(&mut self, key: Vec<f64>, x: T) {
        if self.0.len() == SEEDS_KEPT {
            self.0.remove(0);
        }
        self.0.push((key, x));
    }
}

/// Where on envelope `e` a solve holding point `p` to it starts (#283): `envelope::held::seed`
/// over the envelope's parts as the drawing stands, remembered (`Recent`).
fn envelope_place(sk: &Sketch, e: usize, p: usize) -> Option<[f64; 3]> {
    use crate::envelope::held::{seed, Contact, Parts};
    use std::cell::RefCell;
    thread_local! {
        static SEEDS: RefCell<Recent<[f64; 3]>> = const { RefCell::new(Recent(Vec::new())) };
    }
    let parts = Parts::of(sk, e).ok()?;
    let k = parts.consts(Contact::Point);
    let values = parts.values(sk);
    let at = sk.lifted(p);
    let roll = sk.envelopes[e].roll.radians();
    let key: Vec<f64> = k.iter().chain(&values).chain(&at).chain(&roll).copied().collect();
    if let Some(x) = SEEDS.with(|s| s.borrow().get(&key)) {
        return Some(x);
    }
    let x = seed(&k, &values, at, roll)?;
    SEEDS.with(|s| s.borrow_mut().put(key, x));
    Some(x)
}

/// Where a point on a meshing would start were it stated now, the point standing at `at` and its
/// phase where it stands (#283): what its block seeds it again with when it comes up
/// (`blocks::seed_contacts`, #304).
pub fn meshing_seed(sk: &Sketch, c: &Constraint, at: [f64; 3]) -> Option<[f64; crate::envelope::meshing::UNKNOWNS]> {
    let m = c.meshing_of()?;
    meshing_place(sk, m.i(), at, c.args.get(2)?.value(sk))
}

/// Where on meshing `m` a solve holding a point at `at` to it at `phase` starts (#283):
/// `envelope::meshing::seed` over its parts as the drawing stands — the phase, the lag and the two
/// places — remembered (`Recent`).
fn meshing_place(sk: &Sketch, m: usize, at: [f64; 3], phase: f64) -> Option<[f64; crate::envelope::meshing::UNKNOWNS]> {
    use crate::envelope::meshing::{seed, Meshing, Parts};
    use crate::model::Flank;
    use std::cell::RefCell;
    thread_local! {
        static SEEDS: RefCell<Recent<[f64; crate::envelope::meshing::UNKNOWNS]>> =
            const { RefCell::new(Recent(Vec::new())) };
    }
    let parts = Parts::of(sk, m).ok()?;
    let e = &sk.meshings[m];
    let k = parts.consts(sk, 0);
    let values = parts.values(sk);
    let rolls = e.flanks.clone().map(|f| match f {
        Flank::Envelope(v) => sk.envelopes[v as usize].roll.radians(),
        Flank::Formed(_) | Flank::Traced(_) => [0.0; 2],
    });
    let key: Vec<f64> = k.iter().chain(&values).chain(&at).chain(rolls.iter().flatten())
        .chain(&[phase]).copied().collect();
    if let Some(x) = SEEDS.with(|s| s.borrow().get(&key)) {
        return Some(x);
    }
    let mesh = Meshing::new(k, values, e.roll.radians())?;
    let x = seed(&mesh, rolls, at, phase)?.values();
    SEEDS.with(|s| s.borrow_mut().put(key, x));
    Some(x)
}

/// Fill in what the caller left out, from the geometry — the hidden unknowns, and the entity
/// slots the core infers (a projection's planes) — then whatever the kind refuses once its
/// arguments are all in.  The one place the rule lives, shared by the elaborator, the document
/// reader, the bindings' constraint records and the Rust constructors, so a constraint is
/// refused by one rule wherever it comes from.  `Err` is the reason, in the caller's words.
pub fn seed_omitted(
    sk: &Sketch,
    kind: CKind,
    args: &mut [Arg],
    left_out: impl Fn(usize) -> bool,
) -> Result<(), String> {
    seed_omitted_named(sk, kind, args, left_out, &crate::model::entity_name)
}

/// The same, a refusal naming entities by `name`: the elaborator's source names, where a reader
/// with no source has only the minted label (`entity_name`).
pub fn seed_omitted_named(
    sk: &Sketch,
    kind: CKind,
    args: &mut [Arg],
    left_out: impl Fn(usize) -> bool,
    name: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    for (i, _) in kind.param_slots() {
        if left_out(i) {
            args[i] = Arg::Num(crate::constraints::seed_param(sk, kind, args, i));
        }
    }
    for (i, (_, k)) in kind.spec().iter().enumerate() {
        if k.is_entity() && kind.infers_arg(i) && left_out(i) {
            args[i] = Arg::Ent(crate::constraints::infer_entity(sk, kind, args, i, name)?);
        }
    }
    // and a number the geometry decides — a skew distance's side — once the entities are in
    for i in 0..args.len() {
        if left_out(i) {
            if let Some(a) = crate::constraints::infer_value(sk, kind, args, i) {
                args[i] = a;
            }
        }
    }
    crate::constraints::validate(sk, kind, args, name)
}
