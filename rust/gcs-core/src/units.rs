//! Dimensions and units: what a number *is*, beside what it is worth.
//!
//! Two base dimensions, because two is what the language has — a **length** and an **angle**.
//! `*` and `/` derive them, `+` and `-` demand agreement, and the dimension an expression comes
//! to is checked against the slot it is written in (spec §3.3).  It catches
//! `distance(a, b) == 45deg`, `param x = w + phi`, and the involute formula's unstated radians.
//!
//! **A bare number is dimensionless, and a *context* may take one.**  That is the whole of what
//! "a document with no `unit` line is in drawing units" means: `distance(a, b) == 80` is a length
//! because the slot says so, and `sin(30)` reads 30 as degrees because the function does.  What a
//! context may **not** do is speak for a second operand — `90 / N + ivp` is a plain number added
//! to an angle, and the language asks rather than answers (`90deg / N + ivp` is the answer).  See
//! `Dim::fits` against `Dim::agree`; the asymmetry between them is the design.
//!
//! A name is worth a *number*, and where that number is used decides what it is: `w = 80` in a
//! `Length` slot does not make `w` a length, because the same 80 may be a run, a rise or an
//! angle.  `w = 80mm` is how a person says otherwise, and *that* travels — as does a component
//! formal's declared type, which is what catches `param x = w + phi`.
//!
//! **Lengths cost the core nothing.**  Every kernel is already homogeneous in length: a residual
//! is judged over `extent^degree`, "solved" is `max_relative_residual`, and rank is judged on
//! `conditioned` against a dimensionless tolerance.  Scale every length in a sketch by a constant
//! and not one of those moves.  The core has been unit-agnostic all along; the language is what
//! never said so, and `unit mm` is the language saying it.
//!
//! **Angles are the exception, and it is not a choice.**  `cos θ` is not homogeneous — there is
//! no consistent unit it works in other than radians.  So an angle's *stored* value is radians
//! and its *expression* value is degrees, converted at the text seam exactly where it converts
//! today.  What dies is not the conversion but the guess: `to_arg_units(Angle, v)` stops meaning
//! "the text was degrees, presumably" and starts meaning "convert from the unit the literal
//! named, or the document's".

/// An exponent: a rational, because `sqrt` halves one and `sqrt(area)` is a length.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rat {
    n: i32,
    /// Always positive, and always in lowest terms with `n`.
    d: i32,
}

fn gcd(a: i32, b: i32) -> i32 {
    let (mut a, mut b) = (a.abs(), b.abs());
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a.max(1)
}

impl Rat {
    pub const ZERO: Rat = Rat { n: 0, d: 1 };
    pub const ONE: Rat = Rat { n: 1, d: 1 };

    fn new(n: i32, d: i32) -> Rat {
        let s = if d < 0 { -1 } else { 1 };
        let g = gcd(n, d);
        Rat { n: s * n / g, d: s.abs() * d.abs() / g }
    }

    fn add(self, o: Rat) -> Rat {
        Rat::new(self.n * o.d + o.n * self.d, self.d * o.d)
    }

    fn neg(self) -> Rat {
        Rat { n: -self.n, d: self.d }
    }

    fn mul(self, o: Rat) -> Rat {
        Rat::new(self.n * o.n, self.d * o.d)
    }

    pub fn is_zero(self) -> bool {
        self.n == 0
    }

    fn text(self) -> String {
        if self.d == 1 {
            format!("{}", self.n)
        } else {
            format!("{}/{}", self.n, self.d)
        }
    }
}

/// What a quantity *is*: a power of each base.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dim {
    pub len: Rat,
    pub ang: Rat,
}

impl Default for Dim {
    fn default() -> Dim {
        Dim::SCALAR
    }
}

impl Dim {
    pub const SCALAR: Dim = Dim { len: Rat::ZERO, ang: Rat::ZERO };
    pub const LENGTH: Dim = Dim { len: Rat::ONE, ang: Rat::ZERO };
    pub const ANGLE: Dim = Dim { len: Rat::ZERO, ang: Rat::ONE };

    pub fn is_scalar(self) -> bool {
        self.len.is_zero() && self.ang.is_zero()
    }

    pub fn mul(self, o: Dim) -> Dim {
        Dim { len: self.len.add(o.len), ang: self.ang.add(o.ang) }
    }

    pub fn div(self, o: Dim) -> Dim {
        Dim { len: self.len.add(o.len.neg()), ang: self.ang.add(o.ang.neg()) }
    }

    /// `self ^ k`.  A plain number takes any power; a **dimensioned base takes a whole one** —
    /// `x ^ 2.5` where `x` is a length is not a dimension anybody meant, and `sqrt` is how a
    /// half is written.
    pub fn powf(self, k: f64) -> Option<Dim> {
        if self.is_scalar() {
            return Some(Dim::SCALAR);
        }
        if k.fract() != 0.0 || k.abs() >= 1e6 {
            return None;
        }
        let r = Rat::new(k as i32, 1);
        Some(Dim { len: self.len.mul(r), ang: self.ang.mul(r) })
    }

    /// `sqrt(self)` — the exponents halved, which is why they are rational at all.
    pub fn sqrt(self) -> Dim {
        let h = Rat::new(1, 2);
        Dim { len: self.len.mul(h), ang: self.ang.mul(h) }
    }

    /// What two *operands* have in common.  **Strict**: `+` and `-` demand agreement, and a
    /// bare number does not quietly become an angle because it was added to one.  That is what
    /// catches `param x = w + phi`, and what makes the involute formula's `tan(phi) * 180 / pi -
    /// phi` say the radians it was silently working in — `tan(phi) * 1rad - phi`.
    ///
    /// The asymmetry with `fits` is deliberate and is the whole of the design.  A **context**
    /// that says what it wants may take a bare number, because a document with no `unit` line is
    /// in drawing units and `distance(a, b) == 80` is a length by virtue of the slot.  Two
    /// operands are not a context: neither is authoritative, so mixing one that said what it was
    /// with one that did not is a question the language should ask rather than answer.
    pub fn agree(self, o: Dim) -> Option<Dim> {
        (self == o).then_some(self)
    }

    /// Whether a value of this dimension may stand where `want` is expected.  A bare number may
    /// stand anywhere — that is what "drawing units" means — and anything else must agree.
    pub fn fits(self, want: Dim) -> bool {
        self.is_scalar() || self == want
    }

    /// `fits`, and the complaint when it does not.  **The language's central dimension
    /// complaint, written once**: a constraint's slot (`expr::check_dim`) and a component's
    /// formal (`flatten::bind_value`) are different places to make the same mistake, and two
    /// copies of the sentence would describe one error two ways the moment either was edited.
    /// `what` is whatever the reader would name it by — an attribute, a formal.
    pub fn require(self, want: Dim, what: &str) -> Result<(), String> {
        if self.fits(want) {
            Ok(())
        } else {
            Err(format!("`{what}` is {}, and this is {}", want.name(), self.name()))
        }
    }

    /// How it reads in a diagnostic: `Length`, `Angle`, `Length²`, `Length/Angle`, `Scalar`.
    pub fn name(self) -> String {
        let part = |what: &str, r: Rat| -> Option<String> {
            (!r.is_zero()).then(|| {
                if r == Rat::ONE {
                    what.to_string()
                } else {
                    format!("{what}^{}", r.text())
                }
            })
        };
        let parts: Vec<String> =
            [part("Length", self.len), part("Angle", self.ang)].into_iter().flatten().collect();
        if parts.is_empty() {
            "Scalar".to_string()
        } else {
            parts.join("·")
        }
    }
}

/// A unit a literal may name.
///
/// A length unit is a *ratio*, and what it is a ratio to is the document's own unit — so the
/// table is in millimetres and `Units::length` divides by whatever the document said.  An angle
/// unit is a ratio to the degree, which is the unit every angle in this language is written and
/// read in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Unit {
    pub dim: Dim,
    /// How many of the base (millimetres, or degrees) one of these is worth.
    pub per: f64,
}

/// Every unit name the language knows.  `'` and `"` are in `expr`'s tokenizer rather than here:
/// they are punctuation, not words.
pub const UNITS: &[(&str, Unit)] = &[
    ("mm", Unit { dim: Dim::LENGTH, per: 1.0 }),
    ("cm", Unit { dim: Dim::LENGTH, per: 10.0 }),
    ("m", Unit { dim: Dim::LENGTH, per: 1000.0 }),
    ("km", Unit { dim: Dim::LENGTH, per: 1_000_000.0 }),
    ("in", Unit { dim: Dim::LENGTH, per: 25.4 }),
    ("ft", Unit { dim: Dim::LENGTH, per: 304.8 }),
    ("thou", Unit { dim: Dim::LENGTH, per: 0.0254 }),
    ("deg", Unit { dim: Dim::ANGLE, per: 1.0 }),
    ("rad", Unit { dim: Dim::ANGLE, per: 180.0 / std::f64::consts::PI }),
    ("grad", Unit { dim: Dim::ANGLE, per: 0.9 }),
];

pub fn unit(name: &str) -> Option<Unit> {
    UNITS.iter().find(|&&(n, _)| n == name).map(|&(_, u)| u)
}

/// What the document's numbers are in.
///
/// `length` is `None` for a document with no `unit` line: it is in **drawing units**, a length
/// dimension with no name.  Everything still checks — `distance(a, b) == 45deg` is still an
/// error and `Length + Angle` is still an error — you simply cannot write `mm` or `"`, because
/// there is nothing to convert to.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Units {
    /// Millimetres per drawing unit, and the name it was written under.
    pub length: Option<(&'static str, f64)>,
}

impl Units {
    /// The document's length unit by name, or `Err` naming what the language knows.
    pub fn with_length(name: &str) -> Result<Units, String> {
        // one lookup, which carries both halves: the `'static` name is the row's own, so the
        // name a document is stored under and the ratio it converts by cannot disagree
        match UNITS.iter().find(|&&(n, _)| n == name) {
            Some(&(n, u)) if u.dim == Dim::LENGTH => Ok(Units { length: Some((n, u.per)) }),
            Some(_) => Err(format!("`{name}` is an angle, and a document's unit is its length")),
            None => Err(format!("`{name}` is not a unit: {}", length_names())),
        }
    }

    pub fn name(&self) -> Option<&'static str> {
        self.length.map(|(n, _)| n)
    }

    /// How many of the document's own length units a literal in `u` is worth.
    ///
    /// `Err` where the document named no unit: a number with a unit on it in a document that has
    /// none is a number with nothing to convert to, and guessing would be the language deciding
    /// what a drawing is drawn in.
    pub fn convert(&self, v: f64, u: Unit) -> Result<f64, String> {
        if u.dim != Dim::LENGTH {
            return Ok(v * u.per);
        }
        match self.length {
            Some((_, per)) => Ok(v * u.per / per),
            None => Err("this document names no unit, so a length here is a plain number — \
                         write `unit mm` (or `in`, `cm`, …) at the top to use one"
                .to_string()),
        }
    }
}

fn length_names() -> String {
    let ns: Vec<&str> =
        UNITS.iter().filter(|(_, u)| u.dim == Dim::LENGTH).map(|&(n, _)| n).collect();
    ns.join(", ")
}
