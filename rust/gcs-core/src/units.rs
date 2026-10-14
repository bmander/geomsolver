//! Dimensions and units: what a number *is*, beside what it is worth.
//!
//! Two base dimensions a drawing is made of — a **length** and an **angle** — and three a design
//! table's figures are (issue #311, [0.98]): a **mass**, a **time** and a **current**, which are
//! enough to derive a watt, a volt, an ampere, a newton-metre, a hertz, a tesla, a weber and a
//! pascal.  `*` and `/` derive them, `+` and `-` demand agreement, and the dimension an expression
//! comes to is checked against the slot it is written in (spec §3.3).  It catches
//! `distance(a, b) == 45deg`, `x := w + phi`, the involute formula's unstated radians, and a
//! torque that came out a power.  No slot is of the three: they are for values, so a figure that
//! carries one never reaches a constraint.
//!
//! **A bare number is dimensionless, and a *context* may take one.**  `distance(a, b) == 80` is
//! a length in the document's unit because the slot says so — millimetres where no `unit` line
//! names another — and `sin(30)` reads 30 as degrees because the function does.  What a
//! context may **not** do is speak for a second operand — `90 / N + ivp` is a plain number added
//! to an angle, and the language asks rather than answers (`90deg / N + ivp` is the answer).  See
//! `Dim::fits` against `Dim::agree`; the asymmetry between them is the design.
//!
//! A name is worth a *number*, and where that number is used decides what it is: `w = 80` in a
//! `Length` slot does not make `w` a length, because the same 80 may be a run, a rise or an
//! angle.  `w = 80mm` is how a person says otherwise, and *that* travels — as does a component
//! formal's declared type, which is what catches `x := w + phi`.
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
//!
//! **The three are SI, and lengths stay the document's.**  A number of mass, time or current is
//! in kilograms, seconds and amperes, and a length in it is in the document's unit, so a watt in
//! a millimetre document is 10⁶ kg·mm²/s³: what a watt is, written in the units the arithmetic
//! is done in.  Dividing by the unit reads a figure back (`torque / 1Nm` is 27.9 in any
//! document), and nothing else ever has to know.

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

    /// Its value.
    pub fn value(self) -> f64 {
        self.n as f64 / self.d as f64
    }

    /// The whole number this is, if it is one.
    pub fn whole(self) -> Option<i32> {
        (self.d == 1).then_some(self.n)
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
    pub mass: Rat,
    pub time: Rat,
    pub current: Rat,
}

impl Default for Dim {
    fn default() -> Dim {
        Dim::SCALAR
    }
}

impl Dim {
    pub const SCALAR: Dim = Dim::of(0, 0, 0, 0, 0);
    pub const LENGTH: Dim = Dim::of(1, 0, 0, 0, 0);
    pub const ANGLE: Dim = Dim::of(0, 1, 0, 0, 0);
    pub const MASS: Dim = Dim::of(0, 0, 1, 0, 0);
    pub const TIME: Dim = Dim::of(0, 0, 0, 1, 0);
    pub const CURRENT: Dim = Dim::of(0, 0, 0, 0, 1);

    /// Whole powers of length, angle, mass, time and current, in that order: what the unit
    /// table is written in.
    const fn of(len: i32, ang: i32, mass: i32, time: i32, current: i32) -> Dim {
        const fn r(n: i32) -> Rat {
            Rat { n, d: 1 }
        }
        Dim { len: r(len), ang: r(ang), mass: r(mass), time: r(time), current: r(current) }
    }

    /// Each base's power, beside the name the language spells that base by.
    fn powers(self) -> [(Rat, &'static str); 5] {
        [
            (self.len, "Length"),
            (self.ang, "Angle"),
            (self.mass, "Mass"),
            (self.time, "Time"),
            (self.current, "Current"),
        ]
    }

    /// The same operation on every base's power.
    fn each(self, o: Dim, f: impl Fn(Rat, Rat) -> Rat) -> Dim {
        Dim {
            len: f(self.len, o.len),
            ang: f(self.ang, o.ang),
            mass: f(self.mass, o.mass),
            time: f(self.time, o.time),
            current: f(self.current, o.current),
        }
    }

    /// Spell an evaluated value without erasing its dimension during substitution.  The bases
    /// beyond length and angle are spelt in their SI units, which is what a value of theirs is in.
    pub(crate) fn number_text(self, value: f64, units: Units) -> String {
        let number = crate::syntax::num(value);
        if self.is_scalar() { return number; }
        if self == Self::LENGTH { return format!("{number}{}", units.name()); }
        if self == Self::ANGLE { return format!("{number}deg"); }
        let mut text = number;
        let bases = [units.name(), "deg", "kg", "s", "A"];
        for ((power, _), unit) in self.powers().into_iter().zip(bases) {
            if power.is_zero() { continue; }
            let mut factor = format!("1{unit}");
            // Fractional dimensions arise only through sqrt, so denominators are powers of 2.
            let mut denominator = power.d;
            while denominator > 1 {
                factor = format!("sqrt({factor})");
                denominator /= 2;
            }
            if power.n != 1 { factor = format!("({factor})^({})", power.n); }
            text.push_str(&format!(" * {factor}"));
        }
        text
    }

    pub fn is_scalar(self) -> bool {
        self == Dim::SCALAR
    }

    /// Whether this is a power of length alone (a length, an area, a plain number): what a
    /// variational integrand may be.
    pub fn is_length_power(self) -> bool {
        Dim { len: Rat::ZERO, ..self }.is_scalar()
    }

    pub fn mul(self, o: Dim) -> Dim {
        self.each(o, Rat::add)
    }

    pub fn div(self, o: Dim) -> Dim {
        self.each(o, |a, b| a.add(b.neg()))
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
        Some(self.each(Dim::SCALAR, |a, _| a.mul(r)))
    }

    /// `sqrt(self)` — the exponents halved, which is why they are rational at all.
    pub fn sqrt(self) -> Dim {
        let h = Rat::new(1, 2);
        self.each(Dim::SCALAR, |a, _| a.mul(h))
    }

    /// What two *operands* have in common.  **Strict**: `+` and `-` demand agreement, and a
    /// bare number does not quietly become an angle because it was added to one.  That is what
    /// catches `x := w + phi`, and what makes the involute formula's `tan(phi) * 180 / pi -
    /// phi` say the radians it was silently working in — `tan(phi) * 1rad - phi`.
    ///
    /// The asymmetry with `fits` is deliberate and is the whole of the design.  A **context**
    /// that says what it wants may take a bare number, because `distance(a, b) == 80` is a
    /// length in the document's unit by virtue of the slot.  Two
    /// operands are not a context: neither is authoritative, so mixing one that said what it was
    /// with one that did not is a question the language should ask rather than answer.
    pub fn agree(self, o: Dim) -> Option<Dim> {
        (self == o).then_some(self)
    }

    /// Whether a value of this dimension may stand where `want` is expected.  A bare number may
    /// stand anywhere — it is in the document's unit — and anything else must agree.
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

    /// How it reads in a diagnostic: `Length`, `Angle`, `Length^2`, `Length·Angle^-1`,
    /// `Scalar` — and where an SI unit is this dimension, that unit beside it, since
    /// `Length^2·Mass·Time^-3` is a watt and a reader should not have to work it out:
    /// `Length^2·Mass·Time^-3 (W)`.
    pub fn name(self) -> String {
        let part = |(r, what): (Rat, &str)| -> Option<String> {
            (!r.is_zero()).then(|| {
                if r == Rat::ONE {
                    what.to_string()
                } else {
                    format!("{what}^{}", r.text())
                }
            })
        };
        let parts: Vec<String> = self.powers().into_iter().filter_map(part).collect();
        if parts.is_empty() {
            return "Scalar".to_string();
        }
        let name = parts.join("·");
        match SI.iter().find(|&&(_, d)| d == self) {
            Some((unit, _)) => format!("{name} ({unit})"),
            None => name,
        }
    }
}

/// The SI units a dimension is named by in a diagnostic, where it is one of theirs.  A newton-
/// metre and a joule are one dimension, and a torque is what a machine's table means by it.
const SI: &[(&str, Dim)] = &[
    ("kg", Dim::MASS),
    ("s", Dim::TIME),
    ("A", Dim::CURRENT),
    ("Hz", Dim::of(0, 0, 0, -1, 0)),
    ("deg/s", Dim::of(0, 1, 0, -1, 0)),
    ("N", Dim::of(1, 0, 1, -2, 0)),
    ("N·m", Dim::of(2, 0, 1, -2, 0)),
    ("W", Dim::of(2, 0, 1, -3, 0)),
    ("V", Dim::of(2, 0, 1, -3, -1)),
    ("ohm", Dim::of(2, 0, 1, -3, -2)),
    ("Pa", Dim::of(-1, 0, 1, -2, 0)),
    ("Wb", Dim::of(2, 0, 1, -2, -1)),
    ("T", Dim::of(0, 0, 1, -2, -1)),
];

/// A unit a literal may name.
///
/// A length unit is a *ratio*, and what it is a ratio to is the document's own unit — so the
/// table is in millimetres and `Units::convert` divides by whatever the document said, once for
/// each power of length the unit carries.  An angle unit is a ratio to the degree, which is the
/// unit every angle in this language is written and read in; a unit of mass, time or current is
/// a ratio to the kilogram, the second and the ampere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Unit {
    pub dim: Dim,
    /// How many of the bases (millimetres, degrees, kilograms, seconds, amperes, each to its
    /// power) one of these is worth.
    pub per: f64,
}

/// A metre in the table's millimetres, which every SI unit with a length in it is written over.
const M: f64 = 1000.0;

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
    // issue #311: a design table's figures.  Only a suffix: `48V` is a voltage, and `V` alone
    // is still a name.  No `h` or `min`, which are a height and a function long before they are
    // an hour and a minute; `rpm` is the speed a table states, an angle over a time.
    ("kg", Unit { dim: Dim::MASS, per: 1.0 }),
    ("g", Unit { dim: Dim::MASS, per: 1e-3 }),
    ("s", Unit { dim: Dim::TIME, per: 1.0 }),
    ("ms", Unit { dim: Dim::TIME, per: 1e-3 }),
    ("A", Unit { dim: Dim::CURRENT, per: 1.0 }),
    ("mA", Unit { dim: Dim::CURRENT, per: 1e-3 }),
    ("Hz", Unit { dim: Dim::of(0, 0, 0, -1, 0), per: 1.0 }),
    ("kHz", Unit { dim: Dim::of(0, 0, 0, -1, 0), per: 1e3 }),
    ("rpm", Unit { dim: Dim::of(0, 1, 0, -1, 0), per: 360.0 / 60.0 }),
    ("N", Unit { dim: Dim::of(1, 0, 1, -2, 0), per: M }),
    ("kN", Unit { dim: Dim::of(1, 0, 1, -2, 0), per: 1e3 * M }),
    ("Nm", Unit { dim: Dim::of(2, 0, 1, -2, 0), per: M * M }),
    ("J", Unit { dim: Dim::of(2, 0, 1, -2, 0), per: M * M }),
    ("W", Unit { dim: Dim::of(2, 0, 1, -3, 0), per: M * M }),
    ("kW", Unit { dim: Dim::of(2, 0, 1, -3, 0), per: 1e3 * M * M }),
    ("V", Unit { dim: Dim::of(2, 0, 1, -3, -1), per: M * M }),
    ("mV", Unit { dim: Dim::of(2, 0, 1, -3, -1), per: 1e-3 * M * M }),
    ("kV", Unit { dim: Dim::of(2, 0, 1, -3, -1), per: 1e3 * M * M }),
    ("ohm", Unit { dim: Dim::of(2, 0, 1, -3, -2), per: M * M }),
    ("Pa", Unit { dim: Dim::of(-1, 0, 1, -2, 0), per: 1.0 / M }),
    ("kPa", Unit { dim: Dim::of(-1, 0, 1, -2, 0), per: 1e3 / M }),
    ("MPa", Unit { dim: Dim::of(-1, 0, 1, -2, 0), per: 1e6 / M }),
    ("GPa", Unit { dim: Dim::of(-1, 0, 1, -2, 0), per: 1e9 / M }),
    ("Wb", Unit { dim: Dim::of(2, 0, 1, -2, -1), per: M * M }),
    ("mWb", Unit { dim: Dim::of(2, 0, 1, -2, -1), per: 1e-3 * M * M }),
    ("T", Unit { dim: Dim::of(0, 0, 1, -2, -1), per: 1.0 }),
    ("mT", Unit { dim: Dim::of(0, 0, 1, -2, -1), per: 1e-3 }),
];

pub fn unit(name: &str) -> Option<Unit> {
    UNITS.iter().find(|&&(n, _)| n == name).map(|&(_, u)| u)
}

/// What the document's numbers are in.
///
/// **Millimetres unless the document says otherwise** ([0.93]): a document with no `unit` line
/// is a document in millimetres, so a suffix converts in it as in any other and a length read
/// off its geometry is a `Length`.  `stated` is whether a `unit` line said so, which is what a
/// document saves and prints back: reading one and writing it out adds no line it did not have.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Units {
    /// The document's length unit by name, and how many millimetres one is worth.
    pub length: (&'static str, f64),
    /// Whether a `unit` line named it.
    pub stated: bool,
}

impl Default for Units {
    fn default() -> Units {
        Units { length: ("mm", 1.0), stated: false }
    }
}

impl Units {
    /// The document's length unit by name, as a `unit` line states it, or `Err` naming what the
    /// language knows.
    pub fn with_length(name: &str) -> Result<Units, String> {
        // one lookup, which carries both halves: the `'static` name is the row's own, so the
        // name a document is stored under and the ratio it converts by cannot disagree
        match UNITS.iter().find(|&&(n, _)| n == name) {
            Some(&(n, u)) if u.dim == Dim::LENGTH => Ok(Units { length: (n, u.per), stated: true }),
            Some(&(_, u)) => Err(format!(
                "`{name}` is {}, and a document's unit is its length",
                if u.dim == Dim::ANGLE { "an angle".to_string() } else { u.dim.name() }
            )),
            None => Err(format!("`{name}` is not a unit: {}", length_names())),
        }
    }

    /// The length unit the document's numbers are in, stated or not.
    pub fn name(&self) -> &'static str {
        self.length.0
    }

    /// The unit a `unit` line named, where one did: what is saved and printed back.
    pub fn stated(&self) -> Option<&'static str> {
        self.stated.then_some(self.length.0)
    }

    /// Millimetres per one of the document's lengths.
    pub fn mm_per(&self) -> f64 {
        self.length.1
    }

    /// What a literal in `u` is worth in the document's own units: its length in the
    /// document's length unit, to whatever power `u` carries one (an angle's, how many degrees;
    /// a watt's, how many kg·mm²/s³ in a millimetre document).
    pub fn convert(&self, v: f64, u: Unit) -> f64 {
        let len = u.dim.len;
        if len.is_zero() {
            return v * u.per;
        }
        v * u.per / self.length.1.powf(len.value())
    }
}

fn length_names() -> String {
    let ns: Vec<&str> =
        UNITS.iter().filter(|(_, u)| u.dim == Dim::LENGTH).map(|&(n, _)| n).collect();
    ns.join(", ")
}
