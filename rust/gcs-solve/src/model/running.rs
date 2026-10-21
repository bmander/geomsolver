//! **A parameter axis** (#391 stage 1): the one model type for a number the drawing runs over.

use crate::units::Dim;

/// **A number that runs, and the interval it runs over** (#391 S1): a declared parameter
/// (`param t: Angle over (0deg, 360deg)`, `Sketch::parameters`), a face family's running number
/// (`SolidDef::Family::over`), a traced body's formals (`TracedE::over`, `CurveE::over`) and an
/// interval of the shared roll a reading runs over (an envelope's, a meshing's, a sweep's, a
/// claim's) — one type, so each is a use of it rather than its own mechanism.
///
/// The ends are in the units a person reads, **degrees for an angle**, as every named number of
/// the core is (`HeldNumber`, `Declared`); a reader that works in radians asks `radians`, which
/// is the one conversion the readers made before this type stood between them and the text, so
/// no value moves.  `from < to` is the elaborator's to check, where the interval is written; a
/// reader takes the ends as they stand.
#[derive(Clone, Debug, PartialEq)]
pub struct Running {
    /// What the number is called: a declared parameter's name, a face family's running number
    /// by its absolute name (`#f12.s`, its key in `Sketch::held`), a traced body's formal by its
    /// name in the component, or for an interval of the roll the parameter its reading names
    /// (`over t in (a, b)`, #391 S2) — `Running::ROLL` where nothing names one.
    pub name: String,
    /// What it is: `Length`, `Angle` or a `Scalar`.
    pub dim: Dim,
    pub from: f64,
    pub to: f64,
}

impl Running {
    /// The name of the one shared roll every joint reads (#383): a frame turns `ratio` radians a
    /// radian of it.  No document need declare it — it is there once anything moves — and what a
    /// reading over an interval of it names nothing by (one built in Rust) is called this.
    pub const ROLL: &'static str = "roll";

    pub fn new(name: impl Into<String>, dim: Dim, from: f64, to: f64) -> Running {
        Running { name: name.into(), dim, from, to }
    }

    /// An interval of the shared roll, its ends in degrees.
    pub fn roll(from: f64, to: f64) -> Running {
        Running::new(Running::ROLL, Dim::ANGLE, from, to)
    }

    /// An interval of the shared roll from its ends in radians, for a caller building a
    /// reading in Rust.
    pub fn roll_radians([from, to]: [f64; 2]) -> Running {
        Running::roll(from.to_degrees(), to.to_degrees())
    }

    /// Whether the number is an angle, so a whole turn of it may close what it traces.
    pub fn turns(&self) -> bool {
        self.dim == Dim::ANGLE
    }

    /// The ends, as written.
    pub fn span(&self) -> [f64; 2] {
        [self.from, self.to]
    }

    /// The ends, as written, as a pair.
    pub fn pair(&self) -> (f64, f64) {
        (self.from, self.to)
    }

    /// The middle of the interval: where a face family's station is drawn.
    pub fn mid(&self) -> f64 {
        0.5 * (self.from + self.to)
    }

    /// The ends in the units the kernels read: radians for an angle, as written otherwise.
    pub fn radians(&self) -> [f64; 2] {
        if self.turns() {
            [self.from.to_radians(), self.to.to_radians()]
        } else {
            self.span()
        }
    }
}
