//! **A joint's drive** (#391 S5): its value written over a declared parameter, `fix(value ==
//! 360deg * h(t)) f`, read as a function of the shared roll — the roll *is* that parameter while a
//! reading runs over it (one clock), so the frame stands at its joint turned `ratio · value(θ)`
//! and risen `advance · value(θ) / 360°` at the roll `θ`.  A value linear in its parameter is a
//! constant twist; any other is a cam's law.
//!
//! The text is compiled once over the drawing's held numbers (`tape::Tape`), the parameter among
//! them, and read three ways: its value at a roll (`eval_num` over `f64`), its value and its rate
//! there (`taylor::Jet`, `tape::derivatives_along` — nothing is differenced), and an enclosure of
//! both over a stretch of the roll (`tape::eval_interval`), which is what keeps `MotionBounds`
//! and the speed bound, and so the roll search and a swept field, conservative under a drive.
//! Every number but the parameter is read where the drawing stands when the law is read, as a
//! `motion::Family` reads its axes.
use crate::interval::{Error, Interval as I};
use crate::model::Sketch;

/// A joint's value as a function of the roll, in radians a radian.
#[derive(Clone, Debug, PartialEq)]
pub struct Law {
    /// The value as the `fix` wrote it.
    pub text: String,
    /// The declared parameter it runs over, which the roll stands for.
    pub over: String,
    flat: Vec<f64>,
    /// Every held number where it stands, in the tape's order; the parameter's slot is the roll's.
    x: Vec<f64>,
    at: usize,
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
        let p = sk.parameters.get(over).ok_or_else(|| format!("`{over}` is no declared parameter"))?;
        if p.dim != crate::units::Dim::ANGLE {
            return Err(format!("`{over}` is a {}, and a drive read over the roll runs over an \
                                angle", p.dim.name()));
        }
        let names: Vec<String> = sk.held.keys().cloned().collect();
        let at = names.iter().position(|n| n == over)
            .ok_or_else(|| format!("`{over}` is not held"))?;
        let parsed = crate::expr::parse_in(text, sk.units)?;
        let tape = crate::tape::Tape::compile_wide(&parsed.body, &names)
            .map_err(|m| format!("`{text}`: {m}"))?;
        let x = names.iter().map(|n| sk.params[sk.held[n].param as usize].value).collect();
        Ok(Law { text: text.to_string(), over: over.to_string(), flat: tape.flat, x, at })
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
        let mut dir = vec![0.0; self.x.len()];
        dir[self.at] = 1.0;
        let d = crate::tape::derivatives_along(&self.flat, &self.at_roll(theta), &dir)
            .filter(|d| d[0].is_finite() && d[1].is_finite())
            .ok_or_else(|| format!("`{}` has no rate at {} = {}°", self.text, self.over,
                                   theta.to_degrees()))?;
        // degrees a degree is radians a radian
        Ok([d[0].to_radians(), d[1]])
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
