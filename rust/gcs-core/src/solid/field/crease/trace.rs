//! Following one crease: pinning a point onto two operands' zero sets, the corner three meet at,
//! where a third takes the boundary over, and the predictor-corrector walk between.
use super::{Crease,CreaseOptions,CreaseSource,End,LEAST_DIHEDRAL,OperandId,Reading,P};
use super::graph::meets;
use crate::space::{dot,cross,norm,distance,lerp};
use crate::roots::{bisect,newton_onto};

/// Pull `p` onto both operands' zero sets (`roots::newton_onto`: each step the least move that
/// satisfies both linearised, no step longer than `limit`). The operands' contact times follow the
/// point. The point and both readings, or `None` where the two surfaces are tangent or Newton does
/// not settle.
pub(super) fn pin<S: CreaseSource + ?Sized>(field: &S,p: P,operands: &mut [OperandId;2],tolerance: f64,limit: f64)
    -> Option<(P,Reading,Reading)> {
    let mut last = None;
    let p = newton_onto(p,limit,|p| {
        let o = super::accuracy(tolerance);
        let ra = field.read_operand(p,operands[0],o)?;
        let rb = field.read_operand(p,operands[1],o)?;
        operands[0] = operands[0].at_time(ra.time());
        operands[1] = operands[1].at_time(rb.time());
        let at = [(ra.gradient,ra.value),(rb.gradient,rb.value)];
        last = Some((ra,rb));
        Some(at)
    },|at| at.iter().all(|r| r.1.abs() <= tolerance))?;
    last.map(|(ra,rb)| (p,ra,rb))
}

/// Pull `p` onto three operands' zero sets at once (`roots::newton_onto`, a 3 × 3 solve), no step
/// longer than `limit`: a corner of the material. `None` where they do not meet in a point.
fn corner<S: CreaseSource + ?Sized>(field: &S,p: P,operands: &mut [OperandId;3],tolerance: f64,limit: f64) -> Option<P> {
    if !operands[2].differs(operands[0],0.) || !operands[2].differs(operands[1],0.) { return None; }
    newton_onto(p,limit,|p| {
        let o = super::accuracy(tolerance);
        let mut at = [([0.;3],0.);3];
        for k in 0..3 {
            let r = field.read_operand(p,operands[k],o)?;
            operands[k] = operands[k].at_time(r.time());
            at[k] = (r.gradient,r.value);
        }
        Some(at)
    },|at| at.iter().all(|r| r.1.abs() <= tolerance))
}

/// Whether `p` is on the material's boundary: the whole field reads zero there, to the tolerance.
pub(super) fn on_boundary<S: CreaseSource + ?Sized>(field: &S,p: P,tolerance: f64) -> bool {
    // read to the tolerance, not the hundredth of it a pin needs: the test allows four
    field.read(p,tolerance).value.abs() <= 4.*tolerance
}

/// Whether `p`, pinned onto two operands' carriers, is a point of their crease: on the material's
/// boundary, and each operand's own leaf — trimmed, not its piece's whole carrier — reading zero
/// there too. A carrier runs on past its piece, and the boundary goes on elsewhere (a tip face
/// the cutter is nowhere near reads zero everywhere): neither alone says the two meet at `p`.
fn on_crease<S: CreaseSource + ?Sized>(field: &S,p: P,ops: &[OperandId;2],tolerance: f64) -> bool {
    on_boundary(field,p,tolerance) && ops.iter().all(|op| active(field,p,*op,tolerance).is_ok())
}

/// Whether an operand's leaf is active at `p` — reads zero there as the field sees it (a leaf of a
/// sweep's tool through the whole tool), and decides that reading. Its reading either way.
fn active<S: CreaseSource + ?Sized>(field: &S,p: P,op: OperandId,tolerance: f64) -> Result<Reading,Option<Reading>> {
    let r = field.read_operand(p,op.whole(),super::accuracy(tolerance)).ok_or(None)?;
    if r.value.abs() <= 4.*tolerance && r.operand.is_some_and(|o| o.same_leaf(&op)) { Ok(r) } else { Err(Some(r)) }
}

/// The point between `p`, on the crease, and `q`, pinned onto both operands' carriers past it,
/// where the crease leaves them: bisected along the chord, each trial pinned onto both, to
/// `tolerance`. That point with its operands, the point just past it, and the reading of the
/// operand that took over there: where one of the two leaves has gone inactive (its piece's
/// carrier runs on past the piece), that leaf's own deciding piece and contact time; otherwise
/// what decides the whole field.
fn takeover<S: CreaseSource + ?Sized>(field: &S,p: P,ops: [OperandId;2],q: P,options: &CreaseOptions)
    -> (P,[OperandId;2],P,Reading) {
    let (mut on,mut on_ops,mut off) = (p,ops,q);
    let length = distance(p,q);
    // to a ten-thousandth of the step: a corner there is then found by Newton on all three
    // (`corner`), and a hand-off or an unpinned end is as good to that as to the tolerance
    bisect(0.,1.,|lo,hi| 0.5*(lo+hi),|lo,hi| (hi-lo)*length > options.tolerance.max(1e-4*options.step),|mid| {
        let mut trial = on_ops;
        match pin(field,lerp(p,q,mid),&mut trial,options.tolerance,length) {
            Some((x,..)) if on_crease(field,x,&trial,options.tolerance) => { on = x; on_ops = trial; true }
            Some((x,..)) => { off = x; false }
            None => false,
        }
    });
    let inactive = on_ops.iter().find_map(|&op| active(field,off,op,options.tolerance).err().flatten());
    let reading = inactive.unwrap_or_else(|| field.read(off,super::accuracy(options.tolerance)));
    (on,on_ops,off,reading)
}

/// Follow the crease of two operands through `start`, both ways, while it stays on the material's
/// boundary. `None` when `start` cannot be pinned onto it.
/// A trace stops where it walks onto a crease in `existing` (`End::Met`), ending on that crease: at
/// its end where it is near one, which joins the two into one chain of curves, and otherwise at the
/// nearest point of it, which `creases` then splits the other at. Without it a crease one trace left
/// early and another seed started again is traced twice over the stretch they share.
pub(super) fn trace<S: CreaseSource + ?Sized>(field: &S,start: P,operands: [OperandId;2],options: &CreaseOptions,existing: &[Crease])
    -> Option<Crease> {
    let mut first = operands;
    let (p0,ra,rb) = pin(field,start,&mut first,options.tolerance,options.step)?;
    if !on_crease(field,p0,&first,options.tolerance) { return None; }
    let tangent0 = cross(ra.gradient,rb.gradient);
    // Two operands meeting tangentially — a sweep's envelope running on into the tool at a roll
    // limit — leave the surface smooth there: no crease to keep.
    if !(norm(tangent0) > LEAST_DIHEDRAL.sin()*norm(ra.gradient)*norm(rb.gradient)) { return None; }
    let mut halves: [Vec<P>;2] = [Vec::new(),Vec::new()];
    let mut ends = [End::Budget;2];
    'direction: for (side,sign) in [(0usize,1f64),(1,-1.)] {
        let (mut p,mut ops,mut tangent) = (p0,first,tangent0.map(|x| x*sign/norm(tangent0)));
        let mut step = options.step;
        let mut walked = 0.;
        // Hand-offs since the last step: a crease is followed across pieces that meet
        // tangentially, but not round and round between them.
        let mut handed = 0;
        ends[side] = loop {
            if halves[0].len()+halves[1].len() >= options.max_points { break End::Budget; }
            // Predictor along the tangent, corrector back onto both; a turn sharper than a
            // fifth of a radian, or a corrector that fails, halves the step.
            let mut next = None;
            for _ in 0..6 {
                let guess: P = std::array::from_fn(|k| p[k]+step*tangent[k]);
                let mut trial = ops;
                if let Some((q,qa,qb)) = pin(field,guess,&mut trial,options.tolerance,step) {
                    let t = cross(qa.gradient,qb.gradient);
                    let n = norm(t);
                    if n > 0. {
                        let t = t.map(|x| x/n);
                        let t = if dot(t,tangent) < 0. { t.map(|x| -x) } else { t };
                        if dot(t,tangent) > (0.2f64).cos() && distance(q,p) > 0.25*step { next = Some((q,trial,t)); break; }
                    }
                }
                step *= 0.5;
            }
            let Some((q,trial,t)) = next else { break End::Lost };
            if distance(q,options.centre) > options.radius { break End::Ball; }
            // Both ends on the crease do not make the step one: a crease can run on across a gap
            // the material leaves between them (a blank's edge across a narrow tooth space), so the
            // crease is pinned at the step's middle too, and a middle off it is where it left.
            let middle = {
                let mut ops = trial;
                pin(field,lerp(p,q,0.5),&mut ops,options.tolerance,distance(p,q)).map(|(m,..)| (m,ops))
            };
            let (q,trial) = match middle {
                Some((m,ops)) if !on_crease(field,m,&ops,options.tolerance) => (m,ops),
                _ => (q,trial),
            };
            if !on_crease(field,q,&trial,options.tolerance) {
                // A third operand has taken the boundary over, at `at`.
                let (at,at_ops,_,reading) = takeover(field,p,ops,q,options);
                // a reading naming no operand (its sweep's pose could not be read) says nothing
                // of what took over
                let Some(third) = reading.operand else { break End::Lost };
                let o = super::accuracy(options.tolerance);
                let here = field.read_operand(at,third,o);
                // It continues one being followed without a corner — the next piece of a tool's
                // profile, tangent to the last, or another contact time on one smooth sheet — so
                // the crease goes on along it from there.
                let tangent_to = |k: usize| match (&here,field.read_operand(at,at_ops[k],o)) {
                    (Some(h),Some(r)) => third.differs(at_ops[k],0.) && dot(r.gradient,h.gradient) > 0.
                        && norm(cross(r.gradient,h.gradient)) <= LEAST_DIHEDRAL.sin()*norm(r.gradient)*norm(h.gradient),
                    _ => false,
                };
                if let Some(k) = (0..2).find(|&k| tangent_to(k)) {
                    if handed >= 2 { break End::Handoffs; }
                    handed += 1;
                    if distance(at,p) > options.tolerance { halves[side].push(at); walked += distance(at,p); p = at; }
                    ops = at_ops;
                    ops[k] = third;
                    if let Some(h) = here { ops[k] = ops[k].at_time(h.time()); }
                    continue;
                }
                // Otherwise the three meet at a corner of the material, which ends the crease.
                let mut three = [at_ops[0],at_ops[1],third];
                match corner(field,at,&mut three,options.tolerance,step) {
                    Some(c) if distance(c,at) <= step && on_boundary(field,c,options.tolerance) => {
                        if distance(c,p) > options.tolerance { halves[side].push(c); }
                        break End::Corner(third);
                    }
                    _ => {
                        if distance(at,p) > options.tolerance { halves[side].push(at); }
                        break End::Unpinned(third);
                    }
                }
            }
            walked += distance(q,p);
            // Back at the start, having gone somewhere: a closed crease, the other way not needed.
            if side == 0 && walked > 3.*options.step && distance(q,p0) < 1.5*options.step {
                ends = [End::Closed;2];
                break 'direction;
            }
            if let Some((k,x)) = meets(existing,q,0.25*options.step) {
                halves[side].push(x);
                break End::Met(k);
            }
            halves[side].push(q);
            (p,ops,tangent) = (q,trial,t);
            step = (step*1.5).min(options.step);
            handed = 0;
        };
    }
    let closed = ends[0] == End::Closed;
    let [ahead,behind] = halves;
    let mut points: Vec<P> = behind.into_iter().rev().collect();
    points.push(p0);
    points.extend(ahead);
    if closed { points.push(p0); }
    Some(Crease {points,closed,operands:first,ends:[ends[1],ends[0]]})
}

