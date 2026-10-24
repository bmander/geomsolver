//! **Corner rounds** (#576): the arcs a chain's `A -> round(r: R) B` joint mints, recorded with
//! what they round — the two lines and the virtual corner they were written meeting at — so the
//! construction a round is (its setbacks along its lines) can be worked out where the drawing
//! stands, and a round that does not fit said in its own terms.

use super::Sketch;

/// One corner round: its arc, the line the walk arrives along (`from`, whose `p2` is the arc's
/// point there) and the one it leaves along (`to`, whose `p1` is), the corner they were written
/// meeting at where one was written, and the way the corner turns — `1` left, the arc
/// counter-clockwise from `from` to `to`; `-1` right, the arc drawn from its end (start and end
/// swapped, `Sketch::walked_back` keeping the names).  What each is called, for a message.  Set
/// by the elaborator (`program::rounds`).
#[derive(Clone, Debug)]
pub struct Round {
    pub arc: u32,
    pub from: u32,
    pub to: u32,
    pub corner: Option<u32>,
    pub turn: i8,
    pub names: [String; 3],
}

/// Where a round of radius `r` touches the two lines leaving corner `p` toward `a` and toward
/// `b`, and where its centre is — the round inside the corner, so on the side the walk from `a`
/// through `p` to `b` turns toward.  Each setback is `r / tan(θ/2)` from `p`, the centre `r /
/// sin(θ/2)` along the bisector, `θ` the angle at `p`.  `None` where the corner is no corner: a
/// line running straight on, or folded back on itself, or a side of no length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rounded {
    pub turn: i8,
    pub setback: f64,
    pub start: (f64, f64),
    pub end: (f64, f64),
    pub center: (f64, f64),
    /// `tan(θ/2)`: a setback `d` takes a radius `d · tan(θ/2)`.
    pub half_tan: f64,
}

/// How near a turn may come to none — `|sin θ|` — before a corner is no corner.
const STRAIGHT: f64 = 1e-9;

/// The round of radius `r` at corner `p` between the directions to `a` and to `b` (`Rounded`).
pub fn rounded(p: (f64, f64), a: (f64, f64), b: (f64, f64), r: f64) -> Option<Rounded> {
    let unit = |q: (f64, f64)| {
        let (dx, dy) = (q.0 - p.0, q.1 - p.1);
        let l = dx.hypot(dy);
        (l > 0.0).then(|| (dx / l, dy / l))
    };
    let (u, v) = (unit(a)?, unit(b)?);
    // the turn from `a → p` onto `p → b`: (p − a) × (b − p) is −u × v, its length's
    let sin = u.1 * v.0 - u.0 * v.1;
    let cos = u.0 * v.0 + u.1 * v.1;
    if sin.abs() <= STRAIGHT {
        return None;
    }
    // half the angle at `p` between the two sides
    let theta = sin.abs().atan2(cos);
    let half_tan = (theta / 2.0).tan();
    let setback = r / half_tan;
    let reach = r / (theta / 2.0).sin();
    let (bx, by) = (u.0 + v.0, u.1 + v.1);
    let bl = bx.hypot(by);
    Some(Rounded {
        turn: if sin > 0.0 { 1 } else { -1 },
        setback,
        start: (p.0 + setback * u.0, p.1 + setback * u.1),
        end: (p.0 + setback * v.0, p.1 + setback * v.1),
        center: (p.0 + reach * bx / bl, p.1 + reach * by / bl),
        half_tan,
    })
}

/// Where two lines through `(a, b)` and `(c, d)` cross, where they do.
fn crossing(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> Option<(f64, f64)> {
    let (r, s) = ((b.0 - a.0, b.1 - a.1), (d.0 - c.0, d.1 - c.1));
    let den = r.0 * s.1 - r.1 * s.0;
    if den.abs() <= STRAIGHT * r.0.hypot(r.1) * s.0.hypot(s.1) {
        return None;
    }
    let t = ((c.0 - a.0) * s.1 - (c.1 - a.1) * s.0) / den;
    Some((a.0 + t * r.0, a.1 + t * r.1))
}

impl Sketch {
    /// The corner round `k` rounds, where the drawing stands: its point where one was written,
    /// else where its two lines cross.
    pub fn round_corner(&self, k: &Round) -> Option<(f64, f64)> {
        if let Some(c) = k.corner {
            return Some(self.point_xy(c as usize));
        }
        let (f, t) = (&self.lines[k.from as usize], &self.lines[k.to as usize]);
        let at = |p: u32| self.point_xy(p as usize);
        crossing(at(f.p1), at(f.p2), at(t.p1), at(t.p2))
    }

    /// Where line `l`'s end at `p` stands for a round further along it: the corner of the round
    /// that point is the end of, where one is, else the point itself — so a line between two
    /// rounds runs corner to corner.
    pub fn round_far(&self, p: u32) -> (f64, f64) {
        let mine = self.rounds.iter().find(|k| {
            let (f, t) = (&self.lines[k.from as usize], &self.lines[k.to as usize]);
            f.p2 == p || t.p1 == p
        });
        mine.and_then(|k| self.round_corner(k)).unwrap_or_else(|| self.point_xy(p as usize))
    }

    /// Round `k` where the drawing stands (`rounded`), its radius the arc's.
    pub fn round_at(&self, k: &Round) -> Option<Rounded> {
        let (f, t) = (&self.lines[k.from as usize], &self.lines[k.to as usize]);
        let r = self.params[self.arcs[k.arc as usize].radius as usize].value;
        rounded(self.round_corner(k)?, self.round_far(f.p1), self.round_far(t.p2), r)
    }
}

/// A length as a message says it, to two places — rounded, or where it is a bound, down.
fn said(sk: &Sketch, v: f64, down: bool) -> String {
    let v = match down {
        true => (v * 100.0 + 1e-6).floor() / 100.0,
        false => (v * 100.0).round() / 100.0,
    };
    crate::units::Dim::LENGTH.number_text(v, sk.units)
}

/// **The rounds that do not fit where the drawing stands** (#576), each said by name: a round
/// whose setback runs past its line's far end, with the largest radius that fits the line as
/// drawn; two rounds whose setbacks on one line add to more than it is long; a round at a
/// corner its lines run straight through.  Read after a solve, beside the solids' own
/// complaints, so a profile refused as self-intersecting is told which round made it so.
pub fn overruns(sk: &Sketch) -> Vec<String> {
    let mut out = Vec::new();
    let at: Vec<Option<Rounded>> = sk.rounds.iter().map(|k| sk.round_at(k)).collect();
    for (k, r) in sk.rounds.iter().zip(&at) {
        if r.is_none() && sk.round_corner(k).is_some() {
            out.push(format!("`{}` rounds no corner: `{}` and `{}` run straight on there",
                k.names[0], k.names[1], k.names[2]));
        }
    }
    let radius = |k: &Round| sk.params[sk.arcs[k.arc as usize].radius as usize].value;
    // each line some round sets back along, once: the round at its start and at its end
    let mut lines: Vec<u32> = sk.rounds.iter().flat_map(|k| [k.from, k.to]).collect();
    lines.sort_unstable();
    lines.dedup();
    for l in lines {
        let line = &sk.lines[l as usize];
        let (a, b) = (sk.round_far(line.p1), sk.round_far(line.p2));
        let long = (b.0 - a.0).hypot(b.1 - a.1);
        // the round the line is entered from, and the one it leaves into, each with its setback
        let set = |i: Option<usize>| {
            i.and_then(|i| Some((&sk.rounds[i], at[i]?))).filter(|(_, r)| r.setback > 0.0)
        };
        let entering = set(sk.rounds.iter().position(|k| k.to == l));
        let leaving = set(sk.rounds.iter().position(|k| k.from == l));
        let back = |r: Option<(&Round, Rounded)>| r.map_or(0.0, |(_, r)| r.setback);
        let (da, db) = (back(entering), back(leaving));
        if da + db <= long * (1.0 + 1e-9) + 1e-9 {
            continue;
        }
        match (entering, leaving) {
            (Some((i, _)), Some((j, _))) => out.push(format!(
                "`{}` (r {}) and `{}` (r {}) overlap on `{}`: they set back {} and {} from \
                 corners {} apart",
                i.names[0], said(sk, radius(i), false), j.names[0], said(sk, radius(j), false),
                j.names[1], said(sk, da, false), said(sk, db, false), said(sk, long, false))),
            (Some((k, r)), None) | (None, Some((k, r))) => {
                let name = if k.from == l { &k.names[1] } else { &k.names[2] };
                out.push(format!(
                    "`{}` (r {}) runs past `{name}`'s far end; it fits up to r {}",
                    k.names[0], said(sk, radius(k), false), said(sk, long * r.half_tan, true)));
            }
            (None, None) => {}
        }
    }
    out
}
