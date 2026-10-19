//! **A section against a smooth chain** (#422): where one face of an edge goes on tangent across a
//! seam (`F.smooth`) and the section beside the edge reaches across it, the section in the plane
//! square to the edge is the corner of the other face's trace and a chain of two — the face's own
//! trace to the seam, the next face's from there.  A round's ball touches whichever piece of the
//! chain its offset meets the other face's in, its touch within that piece; a flat is the sum of
//! distances from the chain's nearest piece and the other face, so it changes piece where it
//! crosses the chain's normal at the seam, and is there two arcs each by the rule of its own pair
//! (`corners::sum_between`, the meridian's: a line between lines, else a conic's arc).  With no
//! seam the chain is the face's own trace, and the section the corner of two traces: a ball where
//! their offsets meet, a flat by their sum of distances — every section the table builds is this
//! one's (#427).  Swept along a straight
//! edge, or turned about the axis of a circle every surface is about, the section is a prism's or
//! a revolution's like any other: the strips on the chain's side two faces, split at the body's
//! seam, and the flat two, split where it changes piece — the one over the face's own surface what
//! the table built before.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::piece::{unbuilt, Corner2, Trace};
use super::{Run, Section, Stroke, V};
use crate::brep::corners::{self, Signed};
use crate::brep::plane2::{self, add, cross, dist, dot, left, norm, scale, sub, unit, Carrier, Touch, P};
use crate::brep::planar::Seg;

/// The face going on from one of an edge's faces across a seam its section reaches: the seam's
/// point in the section's plane, and the next face's trace there.
#[derive(Clone, Copy, Debug)]
pub(super) struct Next {
    pub seam: V,
    pub trace: Trace,
}

/// A step of a side of the section (outward from the edge) and its carrier's signed distance,
/// positive on the region's side.
#[derive(Clone, Copy, Debug)]
struct Piece2 {
    seg: Seg,
    toward: Signed,
}

impl Piece2 {
    /// Step `seg`, the region on the left of its travel times `sigma`.
    fn new(seg: Seg, sigma: f64) -> Piece2 { Piece2 { seg, toward: Signed::of(&seg, sigma) } }

    fn carrier(&self) -> Carrier { Carrier::of(&self.seg) }

    /// Its carrier moved `by` toward the region (none where a circle shrinks away).
    fn offset(&self, by: f64) -> Option<Carrier> {
        match (self.toward, self.carrier()) {
            (Signed::Line { n, .. }, Carrier::Line { p, d }) => Some(Carrier::Line { p: add(p, scale(n, by)), d }),
            (Signed::Circle { k, .. }, Carrier::Circle { c, r }) => (r + k * by > 0.0).then_some(Carrier::Circle { c, r: r + k * by }),
            _ => unreachable!("a step of a line or a circle"),
        }
    }

    /// Where `q`'s foot on its carrier stands along the step (0 its start, 1 its end).
    fn at(&self, q: P) -> f64 { self.seg.param(self.carrier().foot(q)) }
}

/// Whether `t` is along a step, within a billionth.
fn within(t: f64) -> bool { (-1e-9..=1.0 + 1e-9).contains(&t) }

/// Whether `t` is along step `i` of a chain of `n` as far as the seams bound it: past its start
/// only at the seam it goes on from (the first's start is the edge, a face's carrier going on
/// behind it), past its end only at the seam it goes on across.
fn between(i: usize, n: usize, t: f64) -> bool { (i == 0 || t >= -1e-9) && (i + 1 == n || t <= 1.0 + 1e-9) }

/// The section beside an edge at `p0` (in the plane square to the unit `along`), between faces seen
/// there as `traces`, face `c`'s going on across a seam into the first of `next`, and each of those
/// across another into the one after it (none: the corner of the two traces alone), on the
/// region's side of both (`within_`: the material's) — a round's ball, or a flat with its legs each
/// from the other face's surface, against the chain's nearest piece; and the section's footprint on
/// the chain (its leg, its ball's touch).
pub(super) fn cut_chain(p0: V, along: V, traces: [Trace; 2], (c, next): (usize, &[Next]), within_: bool, section: Section)
    -> Result<(Run, V), String> {
    let (ua, ub) = (super::piece::toward(traces[0].normal(), within_), super::piece::toward(traces[1].normal(), within_));
    if 1.0 + crate::space::dot(ua, ub) <= 1e-9 {
        return Err(unbuilt("faces folded back on one another"));
    }
    let size = section.size();
    let reach = 8.0 * size;
    let corner = Corner2::new(p0, along, traces, [ua, ub], reach)?;
    let o = 1 - c;
    // each side's step outward from the edge, the region left of travel times `sigma`
    let outward = [corner.steps[0].reversed(), corner.steps[1]];
    let sigma = |k: usize| if k == 0 { -corner.side } else { corner.side };
    // the chain's steps: each face's from the last seam (the edge) to the next, the last's on
    // `reach` past its seam
    let mut segs = vec![outward[c]];
    let mut seams: Vec<P> = Vec::new();
    for n in next {
        let s = corner.at2(n.seam);
        let last = *segs.last().expect("a step");
        let ts = last.param(s);
        if !(ts > 0.0 && ts < 1.0) || dist(last.point(ts), s) > 1e-9 * (size + norm(s)) {
            return Err(unbuilt("a seam off the face it goes on from"));
        }
        let own = last.part(0.0, ts);
        *segs.last_mut().expect("a step") = own;
        let way = unit(own.tangent(1.0));
        segs.push(match n.trace {
            Trace::Line(_) => Seg::Line { a: s, b: add(s, scale(way, reach)) },
            Trace::Circle { centre, radius, .. } => {
                let k = corner.at2(centre);
                let from = sub(s, k);
                if (norm(from) - radius).abs() > 1e-9 * (size + radius) || dot(unit(from), way).abs() > 1e-6 {
                    return Err(unbuilt("a seam where the faces do not go on tangent"));
                }
                let turn = cross(from, way).signum();
                Seg::Arc { c: k, r: radius, a0: from[1].datan2(from[0]),
                    sweep: turn * (reach / radius).min(0.5 * std::f64::consts::PI) }
            }
        });
        seams.push(s);
    }
    let chain: Vec<Piece2> = segs.iter().map(|&g| Piece2::new(g, sigma(c))).collect();
    let other = Piece2::new(outward[o], sigma(o));
    // the chain's steps from the edge to piece `i`'s point at `t`, and the other side's from its
    // point at `t` back to the edge
    let up_to = |i: usize, t: f64| -> Vec<Seg> {
        let mut out: Vec<Seg> = chain[..i].iter().map(|p| p.seg).collect();
        out.push(chain[i].seg.part(0.0, t));
        out
    };
    let (walk_c, mut middle, walk_o, foot): (Vec<Seg>, Vec<(Seg, &'static str)>, Seg, P) = match section {
        Section::Round(r) => {
            let (i, centre, tc, to) = ball(&chain, &other, r)?;
            let (fc, fo) = (chain[i].seg.point(tc), other.seg.point(to));
            // from the chain's foot to the other's: the corner's way from `a` to `b`, or back
            let turn = if c == 0 { corner.side } else { -corner.side };
            (up_to(i, tc), vec![(corners::arc_between(centre, r, fc, fo, turn), "round")], other.seg.part(0.0, to), fc)
        }
        Section::Flat { la, lb } => {
            let legs = [la, lb];
            let (lc, lo) = (legs[c], legs[o]);
            // the leg on the chain: its point `lc` from the other face, nearest the edge
            let (i, tc) = (0..chain.len()).find_map(|i| {
                let crossings = other.offset(lc)
                    .and_then(|off| plane2::meet(chain[i].carrier(), off, Touch::Exact)).unwrap_or_default();
                crossings.into_iter().map(|q| chain[i].seg.param(q)).filter(|&t| within(t))
                    .min_by(|a, b| a.total_cmp(b)).map(|t| (i, t))
            }).ok_or_else(|| unbuilt("a leg that reaches neither face of the chain"))?;
            // the leg on the other face: its point `lo` from the chain's nearest piece
            let (j, to) = (0..chain.len()).filter_map(|j| {
                let crossings = chain[j].offset(lo)
                    .and_then(|off| plane2::meet(other.carrier(), off, Touch::Exact)).unwrap_or_default();
                crossings.into_iter().filter(|&q| between(j, chain.len(), chain[j].at(q))).map(|q| other.seg.param(q))
                    .filter(|&t| within(t)).min_by(|a, b| a.total_cmp(b)).map(|t| (j, t))
            }).min_by(|a, b| a.1.total_cmp(&b.1)).ok_or_else(|| unbuilt("a leg that reaches no piece of the chain"))?;
            let (pc, po) = (chain[i].seg.point(tc), other.seg.point(to));
            // the flat's arc over each piece, by that piece's sum of distances (a Cartesian oval is
            // no construction left to make: no rational curve holds it)
            let over = |k: usize, q0: P, q1: P| -> Result<Seg, String> {
                corners::sum_between(q0, q1, chain[k].toward, other.toward, [lo, lc])
                    .map_err(|e| if e.contains("no rational curve") { e } else { unbuilt(&e) })
            };
            // where it changes piece: on the chain's normal at each seam between the legs' pieces,
            // where both pieces' distances are the distance from the seam
            let crossing = |m: usize| -> Result<P, String> {
                let (s, n) = (seams[m - 1], scale(left(unit(chain[m - 1].seg.tangent(1.0))), sigma(c)));
                let f = |u: f64| { let q = add(s, scale(n, u)); u / lo + other.toward.distance(q) / lc - 1.0 };
                let (f0, f1) = (f(0.0), f(lo));
                if !(f0 < 0.0 && f1 >= 0.0) {
                    return Err(unbuilt("a flat that does not cross the chain's normal at its seam"));
                }
                Ok(add(s, scale(n, crate::roots::bracketed_root(|u| Some(f(u)), 0.0, f0, lo, f1, 1e-15))))
            };
            // from the chain's leg to the other's, piece by piece
            let order: Vec<usize> = if i >= j { (j..=i).rev().collect() } else { (i..=j).collect() };
            let mut points = vec![pc];
            for w in order.windows(2) {
                points.push(crossing(w[0].max(w[1]))?);
            }
            points.push(po);
            let flats = order.iter().zip(points.windows(2))
                .map(|(&k, q)| Ok((over(k, q[0], q[1])?, "flat"))).collect::<Result<Vec<_>, String>>()?;
            (up_to(i, tc), flats, other.seg.part(0.0, to), pc)
        }
    };
    // the loop: from the edge out along `a`, across, and back along `b`
    let lift = |g: &Seg| corner.stroke(g);
    let mut strokes: Vec<(Stroke, &'static str)> = Vec::new();
    if c == 0 {
        strokes.extend(walk_c.iter().map(|g| (lift(g), "a")));
        strokes.extend(middle.iter().map(|(g, n)| (lift(g), *n)));
        strokes.push((lift(&walk_o.reversed()), "b"));
    } else {
        strokes.push((lift(&walk_o), "a"));
        middle.reverse();
        strokes.extend(middle.iter().map(|(g, n)| (lift(&g.reversed()), *n)));
        strokes.extend(walk_c.iter().rev().map(|g| (lift(&g.reversed()), "b")));
    }
    Ok((Run { origin: p0, along, span: [0.0; 2], strokes, within: Vec::new(), kept: Vec::new() }, corner.lift(foot)))
}

/// The ball of radius `r` touching the chain and the other face: which piece of the chain it
/// touches, its centre, and where it touches that piece and the other face (along each); the one
/// nearest the edge whose touches are on their pieces.
fn ball(chain: &[Piece2], other: &Piece2, r: f64) -> Result<(usize, P, f64, f64), String> {
    let mut best: Option<(f64, (usize, P, f64, f64))> = None;
    for (i, piece) in chain.iter().enumerate() {
        let (Some(a), Some(b)) = (piece.offset(r), other.offset(r)) else { continue };
        for centre in plane2::meet(a, b, Touch::Exact).unwrap_or_default() {
            let (tc, to) = (piece.at(centre), other.at(centre));
            if !(within(tc) && within(to)) {
                continue;
            }
            let off = norm(centre);
            if best.is_none_or(|(d, _)| off < d) {
                best = Some((off, (i, centre, tc.clamp(0.0, 1.0), to.clamp(0.0, 1.0))));
            }
        }
    }
    best.map(|(_, b)| b).ok_or_else(|| unbuilt("a ball that touches no piece of the chain"))
}
