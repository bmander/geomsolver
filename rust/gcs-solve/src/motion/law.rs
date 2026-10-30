//! **A joint's drive** (#391 S5): its value written over a declared parameter, `fix(value ==
//! 360deg * h(t)) f`, read as a function of the shared roll — the roll *is* that parameter while
//! a reading runs over it (one clock), so the frame stands at its joint turned `ratio · value(θ)`
//! and risen `advance · value(θ) / 360°` at the roll `θ`: a cam's law, or any other.  A drive is
//! read as no constant twist even where its value is linear (`Family::screw` and its kin refuse
//! it); a sweep under one alone is its joint's over the law's reach (`range`, `solid::paced`, #684
//! L), and one seen from another motion is the field's.
//!
//! The text is compiled once over the drawing's held numbers (`tape::Tape`), the parameter among
//! them, and read three ways: its value at a roll (`eval_num` over `f64`), its value and its rate
//! there (`eval_num` over `taylor::Jet` — nothing is differenced), and an enclosure of both over
//! a stretch of the roll (`tape::eval_interval`), which is what keeps `MotionBounds`
//! and the speed bound, and so the roll search and a swept field, conservative under a drive.
//! Every number but the parameter is read where the drawing stands when the law is read, as a
//! `motion::Family` reads its axes.
use crate::interval::{Error, Interval as I};
use crate::model::Sketch;

/// A joint's value as a function of the roll, in radians a radian.
#[derive(Clone, Debug)]
pub struct Law {
    /// The value as the `fix` wrote it.
    text: String,
    /// The declared parameter it runs over, which the roll stands for.
    over: String,
    flat: Vec<f64>,
    /// The held numbers the text reads where they stand, in the tape's order; the parameter's slot
    /// is the roll's.
    x: Vec<f64>,
    at: usize,
}

/// Why a relation the solve holds cannot read the frame `f` driven by `over`: a meshing's or a
/// traced body's row reads no law (a point carried to a roll and an envelope in space do, #398
/// G), so what is drawn in the frame is read at its value worked out, and the readings over the
/// roll read the law after the solve.  The one wording, wherever the row is refused.
pub fn unread_drive(f: &str, over: &str) -> String {
    format!("`{f}` is driven by `{over}`, and a relation held to a meshing or a traced body under \
             it reads no drive (#391): draw in it, `n := x in {f}`, which stands where its value \
             puts it, or read it over the roll — a sweep, a placement")
}

/// How near the least and the greatest value `Law::range` finds are to the law's own: radians, or
/// that fraction of a value larger than a radian.
pub const RANGE: f64 = 1e-12;
/// How many stretches of the roll `Law::range` may ask before it stands unsettled.
const STRETCHES: usize = 1 << 16;

/// Degrees a radian, enclosed.
fn degrees() -> Result<I, Error> {
    let d = 180.0 / std::f64::consts::PI;
    I::new(d.next_down(), d.next_up())
}

impl Law {
    /// The value `text` of a frame driven by `over`, read over `sk`'s held numbers as they stand:
    /// refused where `over` is no declared angle (the roll is one) or the text reads a name no
    /// held number is (an unknown, whose value is the solve's).
    pub fn read(sk: &Sketch, text: &str, over: &str) -> Result<Law, String> {
        let p = sk.parameters.get(over)
            .ok_or_else(|| format!("`{over}` is no declared parameter"))?;
        if p.dim != crate::units::Dim::ANGLE {
            return Err(format!("`{over}` is a {}, and a drive read over the roll runs over an \
                                angle", p.dim.name()));
        }
        let parsed = crate::expr::parse_in(text, sk.units)?;
        let deps = parsed.body.deps();
        if let Some(d) = deps.iter().find(|d| !sk.held.contains_key(*d)) {
            return Err(format!("`{text}` reads `{d}`, which the drawing does not state (an \
                                unknown is the solve's), and a law is read over stated numbers"));
        }
        let names: Vec<String> = sk.held.keys().filter(|n| *n == over || deps.contains(*n))
            .cloned().collect();
        let at = names.iter().position(|n| n == over)
            .ok_or_else(|| format!("`{over}` is not held"))?;
        let tape = crate::tape::Tape::compile_wide(&parsed.body, &names)
            .map_err(|m| format!("`{text}`: {m}"))?;
        let x = names.iter().map(|n| sk.params[sk.held[n].param as usize].value).collect();
        Ok(Law { text: text.to_string(), over: over.to_string(), flat: tape.flat, x, at })
    }

    /// **The law as a profile in the plane carries it** (#398 G, `generate::Drive`): refused
    /// where the text reads a number of the geometry (`HeldNumber::reads`), which the solve moves
    /// while a profile's constants stand still.
    pub fn carried(sk: &Sketch, text: &str, over: &str) -> Result<Law, String> {
        Law::stated(sk, text, over, "a profile in the plane is generated over stated numbers")
    }

    /// **The law as a relation the solve holds carries it** (#398 G, `envelope::held::Moving::
    /// driven`): a point carried to a roll under a drive, its rate.  Refused, as a profile's, where
    /// the text reads a number of the geometry.
    pub fn held(sk: &Sketch, text: &str, over: &str) -> Result<Law, String> {
        Law::stated(sk, text, over, "a relation the solve holds reads a law over stated numbers")
    }

    fn stated(sk: &Sketch, text: &str, over: &str, why: &str) -> Result<Law, String> {
        let law = Law::read(sk, text, over)?;
        let deps = crate::expr::parse_in(text, sk.units)?.body.deps();
        if let Some(d) = deps.iter().find(|d| sk.held.get(*d).is_some_and(|h| h.reads.is_some())) {
            return Err(format!("`{text}` reads `{d}`, a number of the geometry the solve moves, \
                                and {why}"));
        }
        Ok(law)
    }

    /// Its tape, its numbers and the roll's slot, borrowed: what a profile's encoding carries.
    pub fn drive(&self) -> crate::generate::Drive<'_> {
        crate::generate::Drive { flat: &self.flat, x: &self.x, at: self.at }
    }

    fn at_roll(&self, theta: f64) -> Vec<f64> {
        let mut x = self.x.clone();
        x[self.at] = theta.to_degrees();
        x
    }

    /// The value at the roll `theta`, both in radians.
    pub fn value(&self, theta: f64) -> Result<f64, String> {
        crate::tape::eval_num::<f64>(&self.flat, &self.at_roll(theta)).map(f64::to_radians)
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("`{}` has no value at {} = {}°", self.text, self.over,
                                   theta.to_degrees()))
    }

    /// The value at `theta` and its rate a radian of the roll, read off a Taylor jet.
    pub fn jet(&self, theta: f64) -> Result<[f64; 2], String> {
        let j = self.drive().series(crate::taylor::Jet::var(theta.to_degrees()))
            .ok_or_else(|| format!("`{}` has no rate at {} = {}°", self.text, self.over,
                                   theta.to_degrees()))?;
        // degrees a degree is radians a radian
        Ok([j.nth(0).to_radians(), j.nth(1)])
    }

    /// An enclosure of the value (radians) and of its rate (radians a radian) for every roll in
    /// `theta` (radians), or the reason none is carried.
    pub fn enclose(&self, theta: I) -> Result<[I; 2], Error> {
        let d = degrees()?;
        let mut x = self.x.iter().map(|&v| I::point(v)).collect::<Result<Vec<_>, _>>()?;
        x[self.at] = theta.mul(d)?;
        let [v, r] = crate::tape::eval_interval(&self.flat, &x, self.at)?;
        Ok([v.div(d)?, r])
    }

    /// **The values the law takes over the roll `[a, b]`** (#684 L): its least and its greatest
    /// (radians), each a value it takes at a roll read, and within `RANGE` of the least and the
    /// greatest it takes anywhere in the roll — proved by branch and bound, a stretch dropped
    /// where its enclosure (`enclose`), or its value at the stretch's middle and its rate's reach
    /// across half of it, keeps it within `RANGE` of what is found, and where its rate keeps one
    /// sign, its ends.  The enclosure carries only functions continuous where it carries them, so
    /// the law takes every value between the two.  Refused where it carries none (`enclose`'s
    /// failures), and where `STRETCHES` stretches leave it unsettled.
    pub fn range(&self, [a, b]: [f64; 2]) -> Result<[f64; 2], String> {
        if !(a < b) {
            return Err(format!("the roll ({}°, {}°) runs nowhere", a.to_degrees(),
                               b.to_degrees()));
        }
        Ok([self.extreme([a, b], false)?, self.extreme([a, b], true)?])
    }

    /// The greatest value over `[a, b]` (`up`), or the least: `range`'s search, the value signed
    /// so the greatest is sought.
    fn extreme(&self, [a, b]: [f64; 2], up: bool) -> Result<f64, String> {
        let sign = if up { 1.0 } else { -1.0 };
        let at = |t: f64| self.value(t).map(|v| sign * v);
        let enclosed = |lo: f64, hi: f64| -> Result<[I; 2], String> {
            I::new(lo, hi).and_then(|t| self.enclose(t)).map_err(|e| format!(
                "`{}` carries no enclosure over {} in ({}°, {}°) ({e:?}), so what it reaches is \
                 not proved", self.text, self.over, lo.to_degrees(), hi.to_degrees()))
        };
        let mut best = at(a)?.max(at(b)?);
        let pieces = 64;
        let cut = |k: usize| if k == pieces { b } else { a + (b - a) * k as f64 / pieces as f64 };
        let mut stack: Vec<[f64; 2]> = (0..pieces).map(|k| [cut(k), cut(k + 1)]).collect();
        let mut asked = 0;
        while let Some([lo, hi]) = stack.pop() {
            asked += 1;
            if asked > STRETCHES {
                return Err(format!("`{}` over {} is not settled within {STRETCHES} stretches of \
                                    the roll", self.text, self.over));
            }
            let mid = 0.5 * (lo + hi);
            let middle = at(mid)?;
            best = best.max(middle);
            let [v, r] = enclosed(lo, hi)?;
            let [vl, vh] = v.bounds();
            let [rl, rh] = r.bounds();
            // the enclosure, and the mean value: the middle's and the rate's reach across half
            let mut most = (if up { vh } else { -vl })
                .min(middle + rl.abs().max(rh.abs()) * (hi - mid));
            // a rate of one sign: the greatest is at an end
            if rl > 0.0 || rh < 0.0 {
                let ends = at(lo)?.max(at(hi)?);
                best = best.max(ends);
                most = most.min(ends);
            }
            if most <= best + RANGE * best.abs().max(1.0) || !(lo < mid && mid < hi) {
                continue;
            }
            stack.push([lo, mid]);
            stack.push([mid, hi]);
        }
        Ok(sign * best)
    }
}
