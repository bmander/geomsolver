//! Evidence values have checked constructors; a verdict has no independently writable boolean.
use crate::model::SolidRequirement;

/// A closed, finite interval. Overflow is an error, never a missing-data sentinel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interval {
    lower: f64,
    upper: f64,
}
impl Interval {
    pub fn new(lower: f64, upper: f64) -> Result<Self, String> {
        if !lower.is_finite() || !upper.is_finite() || lower > upper {
            return Err("measurement requires finite ordered interval bounds".into());
        }
        Ok(Self { lower, upper })
    }
    pub fn around(value: f64, error: f64) -> Result<Self, String> {
        if !error.is_finite() || error < 0.0 {
            return Err("measurement uncertainty must be finite and nonnegative".into());
        }
        // Rounding must not collapse nonzero uncertainty into an exact measurement.
        let (lower, upper) = if error == 0.0 {
            (value, value)
        } else {
            ((value - error).next_down(), (value + error).next_up())
        };
        Self::new(lower, upper)
    }
    pub fn lower(self) -> f64 {
        self.lower
    }
    pub fn upper(self) -> f64 {
        self.upper
    }
    pub fn midpoint(self) -> f64 {
        self.lower.midpoint(self.upper)
    }
    /// Radius around the reported (rounded) midpoint, enclosing both interval endpoints.
    pub fn uncertainty(self) -> f64 {
        let midpoint = self.midpoint();
        (midpoint - self.lower).max(self.upper - midpoint)
    }
    fn at_least(self, gap: f64) -> Option<bool> {
        if self.upper < gap {
            Some(false)
        } else if self.lower > gap || (self.lower == self.upper && self.lower == gap) {
            Some(true)
        } else {
            None
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Measurement {
    Bounded(Interval),
    /// No finite boundary distance exists (for example, an empty evaluated boundary).
    Unbounded,
}
impl Measurement {
    pub fn interval(&self) -> Option<Interval> {
        match self {
            Self::Bounded(i) => Some(*i),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Predicate {
    Satisfied,
    Refuted,
    Unresolved(String),
}
impl Predicate {
    pub fn holds(&self) -> Option<bool> {
        match self {
            Self::Satisfied => Some(true),
            Self::Refuted => Some(false),
            Self::Unresolved(_) => None,
        }
    }
}

/// **Where a measurement was read**: the face of each solid, by its path through the operands as
/// a reader writes it (`uno.low.tails.body.near`, `near_screw[1].post.rim`), and a point of each
/// in the world, between which the two come nearest — one point where their boundaries touch or
/// cross.  Two solids whose material is common are read at the `deepest` of it instead: the centre
/// of the largest ball both hold, both points, and the faces of each nearest it.
#[derive(Clone, Debug, PartialEq)]
pub struct Nearest {
    pub a: String,
    pub b: String,
    pub at_a: [f64; 3],
    pub at_b: [f64; 3],
    pub deepest: bool,
}

#[derive(Clone, Debug)]
pub struct GeometricEvidence {
    pub(super) requirement: SolidRequirement,
    pub(super) predicate: Predicate,
    pub(super) measurement: Measurement,
    pub(super) nearest: Option<Nearest>,
}
impl GeometricEvidence {
    pub fn predicate_name(&self) -> &'static str {
        match self.requirement {
            SolidRequirement::Clear { .. } => "disjointness",
            _ => "containment",
        }
    }
    pub fn predicate(&self) -> &Predicate {
        &self.predicate
    }
    pub fn measurement(&self) -> &Measurement {
        &self.measurement
    }
    pub fn measured(&self) -> Option<f64> {
        self.measurement.interval().map(Interval::midpoint)
    }
    /// Where it was measured, and between which faces.
    pub fn nearest(&self) -> Option<&Nearest> {
        self.nearest.as_ref()
    }
    /// The evidence read with the world moved by `pose`: a claim that held the moving solid still
    /// and moved the other back says its points where the drawing's motion stands them.
    pub(crate) fn moved(mut self, pose: crate::envelope::Motion) -> Self {
        if let Some(n) = &mut self.nearest {
            n.at_a = pose.point(n.at_a);
            n.at_b = pose.point(n.at_b);
        }
        self
    }
    /// The sole geometric verdict rule: conjunction with counterexamples dominating uncertainty.
    pub fn holds(&self) -> Option<bool> {
        let required = self.predicate.holds();
        let spacing = match self.requirement.gap() {
            None => Some(true),
            Some(gap) => self.measurement.interval().and_then(|i| i.at_least(gap.value())),
        };
        match (required, spacing) {
            (Some(false), _) | (_, Some(false)) => Some(false),
            (Some(true), Some(true)) => Some(true),
            _ => None,
        }
    }
}

/// A single geometric evaluation. Only the evaluator can construct it.
///
/// ```compile_fail
/// fn override_verdict(v: &mut gcs_solve::clear::Verdict) {
///     v.evidence = Err(String::new());
/// }
/// ```
/// ```compile_fail
/// let interval = gcs_solve::clear::Interval { lower: 2.0, upper: 1.0 };
/// ```
#[derive(Clone, Debug)]
pub struct Verdict {
    pub(super) evidence: Result<GeometricEvidence, String>,
}
impl Verdict {
    pub(super) fn failed(reason: String) -> Self {
        Self { evidence: Err(reason) }
    }
    pub fn evidence(&self) -> Result<&GeometricEvidence, &str> {
        self.evidence.as_ref().map_err(String::as_str)
    }
    pub fn measurement(&self) -> Option<&Measurement> {
        self.evidence.as_ref().ok().map(|e| &e.measurement)
    }
    pub fn measured(&self) -> Option<f64> {
        self.measurement()?.interval().map(Interval::midpoint)
    }
    pub fn tolerance(&self) -> Option<f64> {
        self.measurement()?.interval().map(Interval::uncertainty)
    }
    pub(crate) fn into_evidence(self) -> Result<GeometricEvidence, String> {
        self.evidence
    }
    pub fn holds(&self) -> Option<bool> {
        self.evidence.as_ref().ok()?.holds()
    }
}
