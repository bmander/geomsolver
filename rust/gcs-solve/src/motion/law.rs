//! **A joint's drive** (#391 S5): its value written over a declared parameter, `fix(value ==
//! 360deg * h(t)) f`, read as a function of the shared roll — the roll *is* that parameter while
//! a reading runs over it (one clock), so the frame stands at its joint turned `ratio · value(θ)`
//! and risen `advance · value(θ) / 360°` at the roll `θ`: a cam's law, or any other.  A drive is
//! read as no constant twist even where its value is linear (`Family::screw` and its kin refuse
//! it), so what is built under one is the field's.
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

/// Why a relation the solve holds cannot read the frame `f` driven by `over`: a row reads no law,
/// so what is drawn in the frame is read at its value worked out, and the readings over the roll
/// read the law after the solve.  The one wording, wherever the row is refused.
pub fn unread_drive(f: &str, over: &str) -> String {
    format!("`{f}` is driven by `{over}`, and a relation the solve holds reads no drive (#391): \
             draw in it, `n := x in {f}`, which stands where its value puts it, or read it over \
             the roll — a sweep, a placement")
}

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
        let law = Law::read(sk, text, over)?;
        let deps = crate::expr::parse_in(text, sk.units)?.body.deps();
        if let Some(d) = deps.iter().find(|d| sk.held.get(*d).is_some_and(|h| h.reads.is_some())) {
            return Err(format!("`{text}` reads `{d}`, a number of the geometry the solve moves, \
                                and a profile in the plane is generated over stated numbers"));
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
}
