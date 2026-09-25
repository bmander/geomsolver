//! The creases of a material field, found from the field itself: where two operands meet on the
//! material's boundary. The protected Delaunay refinement (`delaunay::refine`) keeps a sharp edge
//! only where it is told the edge's curve, and an implicit domain has no curves to be told; these
//! are them.
//!
//! A crease is where the operand deciding the field changes — the block's face on one side, the
//! groove's sweep on the other — or where one sweep's contact time jumps, two contacts of one
//! tool. `MaterialField::operand` reads either operand alone, with its gradient, so a point is
//! pinned onto both zero sets by minimum-norm Newton on the two values and traced by
//! predictor-corrector along the cross product of their gradients (the marching of surface
//! intersection in CAD). A crease ends where a third operand takes the boundary over (a corner) or
//! where it leaves the bounding ball; it closes where it comes back to its start.
//!
//! Starting points come from a mesh already made without features: an edge whose ends are decided
//! by different operands crosses a crease. A crease loop smaller than that mesh's facets can be
//! missed, which is marching's known limit (its remedy, loop detection, is not attempted).
use super::{MaterialField,Reading,ReadingOptions,WHOLE};

type P = [f64;3];

/// A crease starts only where its two surfaces meet at least this far from tangent, in radians:
/// two degrees. It may run on to where they are nearly tangent (`pin` refuses only exact tangency).
const LEAST_DIHEDRAL: f64 = 2.*std::f64::consts::PI/180.;

fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: P,b: P) -> P { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: P) -> f64 { dot(a,a).sqrt() }
fn dist(a: P,b: P) -> f64 { norm(sub(a,b)) }

/// One operand of a field: a piece of a leaf's boundary (a face of a prism, a turned edge of a
/// revolution), and for a leaf of a sweep the roll time of its contact.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Operand { pub leaf: usize,pub piece: usize,pub time: Option<f64> }

impl Operand {
    pub fn of(r: &Reading) -> Self { Self {leaf:r.leaf,piece:r.piece,time:r.time} }
    /// Two readings' operands are different: other pieces, or one sweep's contacts `gap` apart.
    fn differs(self,other: Self,gap: f64) -> bool {
        self.leaf != other.leaf || self.piece != other.piece || match (self.time,other.time) {
            (Some(s),Some(t)) => (s-t).abs() > gap,
            _ => false,
        }
    }
}

/// How finely creases are found and followed.
#[derive(Clone,Copy,Debug)]
pub struct CreaseOptions {
    /// The step along a crease.
    pub step: f64,
    /// A point is on both zero sets, and on the material's boundary, to this.
    pub tolerance: f64,
    /// Contact times of one sweep this far apart are two operands.
    pub time_gap: f64,
    /// The bounding ball a crease is followed within.
    pub centre: P,
    pub radius: f64,
    /// Points a crease may have before it is given up (a crease that never closes nor ends).
    pub max_points: usize,
}

/// A traced crease: its points in order, whether it closes on itself, the two operands it was
/// started between (a hand-off may change them along it), and why it stops at each end.
#[derive(Clone,Debug)]
pub struct Crease { pub points: Vec<P>,pub closed: bool,pub operands: [Operand;2],pub ends: [End;2] }

fn options_at(p: P,tolerance: f64) -> ReadingOptions {
    ReadingOptions {accuracy:0.01*tolerance,relative:0.,..ReadingOptions::at(p)}
}

/// Pull `p` onto both operands' zero sets: Newton on their two values, each step the least move
/// that satisfies both linearised (`Jᵀ(JJᵀ)⁻¹F`), no step longer than `limit`. The operands'
/// contact times follow the point. The point and both readings, or `None` where the two surfaces
/// are tangent or Newton does not settle.
pub fn pin(field: &MaterialField,mut p: P,operands: &mut [Operand;2],tolerance: f64,limit: f64)
    -> Option<(P,Reading,Reading)> {
    for _ in 0..40 {
        let o = options_at(p,tolerance);
        let ra = field.operand(p,operands[0].leaf,operands[0].piece,operands[0].time,&o)?;
        let rb = field.operand(p,operands[1].leaf,operands[1].piece,operands[1].time,&o)?;
        operands[0].time = ra.time;
        operands[1].time = rb.time;
        if ra.value.abs() <= tolerance && rb.value.abs() <= tolerance { return Some((p,ra,rb)); }
        let (ga,gb) = (ra.gradient,rb.gradient);
        let (aa,ab,bb) = (dot(ga,ga),dot(ga,gb),dot(gb,gb));
        let det = aa*bb-ab*ab;
        // Tangent (or a gradient lost): the two equations do not fix a curve here.
        if !(det > 1e-8*aa*bb) { return None; }
        let (l0,l1) = ((bb*ra.value-ab*rb.value)/det,(aa*rb.value-ab*ra.value)/det);
        let mut step: P = std::array::from_fn(|k| l0*ga[k]+l1*gb[k]);
        let length = norm(step);
        if !length.is_finite() { return None; }
        if length > limit { step = step.map(|x| x*limit/length); }
        p = sub(p,step);
    }
    None
}

/// Pull `p` onto three operands' zero sets at once, Newton on the three values (a 3 × 3 solve),
/// no step longer than `limit`: a corner of the material. `None` where they do not meet in a point.
pub fn corner(field: &MaterialField,mut p: P,operands: &mut [Operand;3],tolerance: f64,limit: f64) -> Option<P> {
    if operands[2].differs(operands[0],0.) == false || operands[2].differs(operands[1],0.) == false { return None; }
    for _ in 0..40 {
        let o = options_at(p,tolerance);
        let mut rows = [[0.;3];3];
        let mut values = [0.;3];
        for k in 0..3 {
            let r = field.operand(p,operands[k].leaf,operands[k].piece,operands[k].time,&o)?;
            operands[k].time = r.time;
            rows[k] = r.gradient;
            values[k] = r.value;
        }
        if values.iter().all(|v| v.abs() <= tolerance) { return Some(p); }
        let det = dot(rows[0],cross(rows[1],rows[2]));
        let scale = norm(rows[0])*norm(rows[1])*norm(rows[2]);
        if !(det.abs() > 1e-6*scale) { return None; }
        // Cramer's rule through the adjugate: J⁻¹ = [b×c, c×a, a×b]ᵀ / det, applied to the values.
        let columns = [cross(rows[1],rows[2]),cross(rows[2],rows[0]),cross(rows[0],rows[1])];
        let mut step: P = std::array::from_fn(|i| (0..3).map(|k| columns[k][i]*values[k]).sum::<f64>()/det);
        let length = norm(step);
        if !length.is_finite() { return None; }
        if length > limit { step = step.map(|x| x*limit/length); }
        p = sub(p,step);
    }
    None
}

/// Whether `p` is on the material's boundary: the whole field reads zero there, to the tolerance.
fn on_boundary(field: &MaterialField,p: P,tolerance: f64) -> bool {
    field.reading_with(p,&options_at(p,tolerance),&mut 0).value.abs() <= 4.*tolerance
}

/// Whether `p`, pinned onto two operands' carriers, is a point of their crease: on the material's
/// boundary, and each operand's own leaf — trimmed, not its piece's whole carrier — reading zero
/// there too. A carrier runs on past its piece, and the boundary goes on elsewhere (a tip face
/// the cutter is nowhere near reads zero everywhere): neither alone says the two meet at `p`.
fn on_crease(field: &MaterialField,p: P,ops: &[Operand;2],tolerance: f64) -> bool {
    on_boundary(field,p,tolerance) && ops.iter().all(|op| active(field,p,*op,tolerance).is_ok())
}

/// Whether an operand's leaf is active at `p` — reads zero there as the field sees it (a leaf of a
/// sweep's tool through the whole tool), and decides that reading. Its reading either way.
fn active(field: &MaterialField,p: P,op: Operand,tolerance: f64) -> Result<Reading,Option<Reading>> {
    let r = field.operand(p,op.leaf,WHOLE,op.time,&options_at(p,tolerance)).ok_or(None)?;
    if r.value.abs() <= 4.*tolerance && r.leaf == op.leaf { Ok(r) } else { Err(Some(r)) }
}

/// Why a traced crease stops where it does.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum End {
    /// It came back to its start.
    Closed,
    /// A third operand meets both at a point: a corner of the material.
    Corner(Operand),
    /// A third operand takes the boundary over, and the three pin no point there.
    Unpinned(Operand),
    /// It left the bounding ball.
    Ball,
    /// The corrector failed at every step size: the two surfaces turn tangent, or Newton fails.
    Lost,
    /// Hand-offs across tangent operands went round without progress.
    Handoffs,
    /// The point budget ran out.
    Budget,
    /// It walked onto crease `n` of those already traced, at a point of it.
    Met(usize),
}

/// The point between `p`, on the crease, and `q`, pinned onto both operands' carriers past it,
/// where the crease leaves them: bisected along the chord, each trial pinned onto both, to
/// `tolerance`. That point with its operands, the point just past it, and the reading of the
/// operand that took over there: where one of the two leaves has gone inactive (its piece's
/// carrier runs on past the piece), that leaf's own deciding piece and contact time; otherwise
/// what decides the whole field.
fn takeover(field: &MaterialField,p: P,ops: [Operand;2],q: P,options: &CreaseOptions)
    -> (P,[Operand;2],P,Reading) {
    let (mut lo,mut hi) = (0.,1.);
    let (mut on,mut on_ops,mut off) = (p,ops,q);
    let length = dist(p,q);
    while (hi-lo)*length > options.tolerance {
        let mid = 0.5*(lo+hi);
        let mut trial = on_ops;
        match pin(field,lerp(p,q,mid),&mut trial,options.tolerance,length) {
            Some((x,..)) if on_crease(field,x,&trial,options.tolerance) => { lo = mid; on = x; on_ops = trial; }
            Some((x,..)) => { hi = mid; off = x; }
            None => hi = mid,
        }
    }
    let inactive = on_ops.iter().find_map(|&op| active(field,off,op,options.tolerance).err().flatten());
    let reading = inactive.unwrap_or_else(|| field.reading_with(off,&options_at(off,options.tolerance),&mut 0));
    (on,on_ops,off,reading)
}

fn lerp(a: P,b: P,t: f64) -> P { std::array::from_fn(|k| a[k]+t*(b[k]-a[k])) }

/// Follow the crease of two operands through `start`, both ways, while it stays on the material's
/// boundary. `None` when `start` cannot be pinned onto it.
/// A trace stops where it walks onto a crease in `existing` (`End::Met`), ending on that crease: at
/// its end where it is near one, which joins the two into one chain of curves, and otherwise at the
/// nearest point of it, which `creases` then splits the other at. Without it a crease one trace left
/// early and another seed started again is traced twice over the stretch they share.
pub fn trace(field: &MaterialField,start: P,operands: [Operand;2],options: &CreaseOptions,existing: &[Crease])
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
                        if dot(t,tangent) > (0.2f64).cos() && dist(q,p) > 0.25*step { next = Some((q,trial,t)); break; }
                    }
                }
                step *= 0.5;
            }
            let Some((q,trial,t)) = next else { break End::Lost };
            if dist(q,options.centre) > options.radius { break End::Ball; }
            // Both ends on the crease do not make the step one: a crease can run on across a gap
            // the material leaves between them (a blank's edge across a narrow tooth space), so the
            // crease is pinned at the step's middle too, and a middle off it is where it left.
            let middle = {
                let mut ops = trial;
                pin(field,lerp(p,q,0.5),&mut ops,options.tolerance,dist(p,q)).map(|(m,..)| (m,ops))
            };
            let (q,trial) = match middle {
                Some((m,ops)) if !on_crease(field,m,&ops,options.tolerance) => (m,ops),
                _ => (q,trial),
            };
            if !on_crease(field,q,&trial,options.tolerance) {
                // A third operand has taken the boundary over, at `at`.
                let (at,at_ops,_,reading) = takeover(field,p,ops,q,options);
                let third = Operand::of(&reading);
                let o = options_at(at,options.tolerance);
                let here = field.operand(at,third.leaf,third.piece,third.time,&o);
                // It continues one being followed without a corner — the next piece of a tool's
                // profile, tangent to the last, or another contact time on one smooth sheet — so
                // the crease goes on along it from there.
                let tangent_to = |k: usize| match (&here,field.operand(at,at_ops[k].leaf,at_ops[k].piece,at_ops[k].time,&o)) {
                    (Some(h),Some(r)) => third.differs(at_ops[k],0.) && dot(r.gradient,h.gradient) > 0.
                        && norm(cross(r.gradient,h.gradient)) <= LEAST_DIHEDRAL.sin()*norm(r.gradient)*norm(h.gradient),
                    _ => false,
                };
                if let Some(k) = (0..2).find(|&k| tangent_to(k)) {
                    if handed >= 2 { break End::Handoffs; }
                    handed += 1;
                    if dist(at,p) > options.tolerance { halves[side].push(at); walked += dist(at,p); p = at; }
                    ops = at_ops;
                    ops[k] = third;
                    if let Some(h) = here { ops[k].time = h.time; }
                    continue;
                }
                // Otherwise the three meet at a corner of the material, which ends the crease.
                let mut three = [at_ops[0],at_ops[1],third];
                match corner(field,at,&mut three,options.tolerance,step) {
                    Some(c) if dist(c,at) <= step && on_boundary(field,c,options.tolerance) => {
                        if dist(c,p) > options.tolerance { halves[side].push(c); }
                        break End::Corner(third);
                    }
                    _ => {
                        if dist(at,p) > options.tolerance { halves[side].push(at); }
                        break End::Unpinned(third);
                    }
                }
            }
            walked += dist(q,p);
            // Back at the start, having gone somewhere: a closed crease, the other way not needed.
            if side == 0 && walked > 3.*options.step && dist(q,p0) < 1.5*options.step {
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

/// Starting points for creases on a mesh made without them: the middle of every edge whose ends
/// are decided by different operands, with those two operands.
pub fn seeds(field: &MaterialField,vertices: &[P],triangles: &[[u32;3]],options: &CreaseOptions)
    -> Vec<(P,[Operand;2])> {
    let operands: Vec<Operand> = vertices.iter().map(|&v| Operand::of(&field.reading_with(v,&options_at(v,options.tolerance),&mut 0))).collect();
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for t in triangles {
        for j in 0..3 {
            let (u,v) = (t[j].min(t[(j+1)%3]),t[j].max(t[(j+1)%3]));
            if !seen.insert((u,v)) { continue; }
            let (a,b) = (operands[u as usize],operands[v as usize]);
            if a.differs(b,options.time_gap) {
                let (pu,pv) = (vertices[u as usize],vertices[v as usize]);
                out.push((std::array::from_fn(|k| 0.5*(pu[k]+pv[k])),[a,b]));
            }
        }
    }
    out
}

/// The creases through a set of seeds, each traced once: a seed that pins onto a crease already
/// traced starts nothing. A seed may lie some way off the crease it finds — a mesh made without
/// features cuts a sharp edge on a chamfer — so it is the pinned point that is compared.
pub fn creases(field: &MaterialField,seeds: &[(P,[Operand;2])],options: &CreaseOptions) -> Vec<Crease> {
    let mut out: Vec<Crease> = Vec::new();
    let near = |out: &[Crease],p: P,within: f64| out.iter().any(|c| c.points.len() == 1 && dist(p,c.points[0]) < within
        || c.points.windows(2).any(|w| segment_distance(p,w[0],w[1]) < within));
    for &(p,ops) in seeds {
        // a seed on a crease already traced needs no pinning
        if near(&out,p,0.5*options.step) { continue; }
        let Some((q,..)) = pin(field,p,&mut ops.clone(),options.tolerance,options.step) else { continue };
        if near(&out,q,0.25*options.step) { continue; }
        let Some(c) = trace(field,q,ops,options,&out) else { continue };
        if c.points.len() < 2 { continue; }
        // A trace that ended on another crease's middle splits that crease there, so the two share
        // a corner rather than one ending beside the other.
        for (e,at) in [(c.ends[0],c.points[0]),(c.ends[1],*c.points.last().unwrap())] {
            if let End::Met(k) = e { split(&mut out,k,at); }
        }
        out.push(c);
    }
    out
}

/// Run after the ends are welded (`features`), so what is split in is the corner as it will stand.
/// Every crease end lying on another crease's middle — a T, where a crease meets one that runs on
/// through the point (the tip rim, handed on tangentially between two pieces of a cutter's profile,
/// where the crease between those pieces reaches it) — splits that other crease at the end's exact
/// point, which protection then takes as the corner the two share. Ends near another's end are the
/// weld's; one merely near another's end, past the weld, still splits it, beside that end.
fn junctions(creases: &mut Vec<Crease>,within: f64) {
    let mut i = 0;
    while i < creases.len() {
        if !creases[i].closed {
            for which in [0,1] {
                let at = if which == 0 { creases[i].points[0] } else { *creases[i].points.last().unwrap() };
                enum Act { Snap(P),Split(usize) }
                let mut act = None;
                for (k,c) in creases.iter().enumerate() {
                    if k == i { continue; }
                    let ends = [c.points[0],*c.points.last().unwrap()];
                    // already one corner, as the weld or an earlier split left it
                    if ends.contains(&at) { act = None; break; }
                    // near another's end, past the weld: the same corner found two ways, so this
                    // end is moved onto that one — split instead, each would split the other again
                    if let Some(&e) = ends.iter().find(|&&e| !c.closed && dist(e,at) <= 2.*within) { act = Some(Act::Snap(e)); break; }
                    if c.points.windows(2).any(|w| segment_distance(at,w[0],w[1]) < within) { act = Some(Act::Split(k)); break; }
                }
                match act {
                    Some(Act::Snap(e)) => {
                        let n = creases[i].points.len();
                        creases[i].points[if which == 0 { 0 } else { n-1 }] = e;
                    }
                    Some(Act::Split(k)) => split(creases,k,at),
                    None => {}
                }
            }
        }
        i += 1;
    }
}

/// The crease of `existing` that `q` lies within `within` of, and the point of it `q` is to end at:
/// the crease's own end where that is within two of `within`, and otherwise its nearest point.
fn meets(existing: &[Crease],q: P,within: f64) -> Option<(usize,P)> {
    for (k,c) in existing.iter().enumerate() {
        let near = c.points.windows(2).map(|w| (w[0],w[1],segment_distance(q,w[0],w[1])))
            .fold(None,|best: Option<(P,P,f64)>,x| if best.is_none_or(|b| x.2 < b.2) { Some(x) } else { best });
        let Some((a,b,d)) = near else { continue };
        if d >= within { continue; }
        let ends = if c.closed { vec![] } else { vec![c.points[0],*c.points.last().unwrap()] };
        if let Some(&e) = ends.iter().find(|&&e| dist(e,q) <= 2.*within) { return Some((k,e)); }
        return Some((k,closest(q,a,b)));
    }
    None
}

/// Crease `k` split at `at`, a point on it: `at` put in at its nearest segment, and the crease cut
/// in two there — or, closed, begun and ended there. Nothing is done where `at` is already an end.
fn split(creases: &mut Vec<Crease>,k: usize,at: P) {
    let c = &creases[k];
    let n = c.points.len();
    if c.points[0] == at || c.points[n-1] == at { return; }
    let Some(j) = (0..n-1).min_by(|&x,&y| segment_distance(at,c.points[x],c.points[x+1])
        .total_cmp(&segment_distance(at,c.points[y],c.points[y+1]))) else { return };
    let mut points = c.points.clone();
    points.insert(j+1,at);
    if c.closed {
        // a loop begun and ended at the corner: still closed, the corner its one end
        let mut ring: Vec<P> = points[j+1..points.len()-1].to_vec();
        ring.extend_from_slice(&points[..=j]);
        ring.push(at);
        creases[k].points = ring;
    } else {
        let (head,tail) = (points[..=j+1].to_vec(),points[j+1..].to_vec());
        let second = Crease {points:tail,closed:false,operands:c.operands,ends:[End::Met(k),c.ends[1]]};
        creases[k].points = head;
        creases[k].ends[1] = End::Met(creases.len());
        creases.push(second);
    }
}

fn closest(p: P,a: P,b: P) -> P {
    let (d,w) = (sub(b,a),sub(p,a));
    let l = dot(d,d);
    let s = if l > 0. { (dot(w,d)/l).clamp(0.,1.) } else { 0. };
    std::array::from_fn(|k| a[k]+s*d[k])
}

fn segment_distance(p: P,a: P,b: P) -> f64 {
    let (d,w) = (sub(b,a),sub(p,a));
    let l = dot(d,d);
    let s = if l > 0. { (dot(w,d)/l).clamp(0.,1.) } else { 0. };
    dist(p,std::array::from_fn(|k| a[k]+s*d[k]))
}

/// The feature curves of a field, for `delaunay::refine`'s protection, from a mesh made without
/// them: every crease its edges cross, traced, with the ends that meet at one corner made one
/// point, since a corner is a ball the curves share only where their ends are equal. Ends within
/// a fiftieth of a step are one corner: an end found where a crease stops being followed lands
/// within the bisection of the corner another found exactly, and two corners that close could
/// not be protected apart anyway.
pub fn features(field: &MaterialField,vertices: &[P],triangles: &[[u32;3]],options: &CreaseOptions) -> Vec<Vec<P>> {
    let mut found = creases(field,&seeds(field,vertices,triangles,options),options);
    let mut corners: Vec<P> = Vec::new();
    let mut weld = |p: P| -> P {
        match corners.iter().find(|&&c| dist(c,p) <= 0.02*options.step) {
            Some(&c) => c,
            None => { corners.push(p); p }
        }
    };
    for c in found.iter_mut().filter(|c| !c.closed) {
        let last = c.points.len()-1;
        c.points[0] = weld(c.points[0]);
        c.points[last] = weld(c.points[last]);
    }
    // the T-junctions last, at the corners as welded: split any earlier, the weld could move the
    // end away from the point split in, and the two would meet a few microns apart
    junctions(&mut found,0.25*options.step);
    found.into_iter().map(|c| c.points).collect()
}
