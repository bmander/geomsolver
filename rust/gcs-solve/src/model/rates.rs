//! **Rate rows** (#339 D3): `rate(x over s) perpendicular y`, a relation that reads the rate of
//! `x` along a face family's running number.
//!
//! A rate row is no row of the drawing.  The drawn station is the **home** of its family, its
//! initial value: solved with the drawing, with what holds there alone (`initially { … }`,
//! `Constraint::initially`) and without its rate rows.  Everywhere else the station is the flow
//! integrated from the home (`solid::rated`), and the rate rows are what make that flow one:
//! together with the station's relations differentiated along the running number they are a
//! linear system in the station's rates.  So the sketch keeps them beside its constraints, read by
//! the family alone, and never compiles one.

use super::*;

/// What a rate row states of the rate it reads (decision 1 of D0: a rate stands where a direction
/// is read, and the rows it states are linear in it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RateWord {
    /// Square to a direction (one row), or along a plane's normal (two).
    Perpendicular,
    /// Along a direction (two rows), or within a plane (one).
    Parallel,
}

/// `rate(of over s) word to`, `s` a face family's running number (`over`, its held name).
#[derive(Clone, Debug, PartialEq)]
pub struct RateRow {
    /// Whose rate: an axis (the rate of its direction) or a point (its velocity).
    pub of: EntRef,
    pub word: RateWord,
    /// What the rate is held against: an axis or a line (a direction), or a plane.
    pub to: EntRef,
    /// The running number it is a rate along (`Sketch::held`).
    pub over: String,
    /// The statement as it was written, for what a refusal says.
    pub text: String,
}

impl RateRow {
    /// A rate row comes along with both the entities it names (#339 D3).
    pub fn remapped(&self, map: &Remap) -> Option<RateRow> {
        let RateRow { of, word, to, over, text } = self;
        Some(RateRow { of: map(*of)?, word: *word, to: map(*to)?, over: over.clone(),
                       text: text.clone() })
    }
}
