//! **A material** (#510 P3): what a solid is made of, as the document states it.
//!
//! `steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)` and `arm :=
//! solid(sec, depth: 4mm) of steel`.  A model value the analysis of #510 will read (a frame's
//! stiffness, a mode's mass, a factor of safety); nothing reads one yet.  Its properties are
//! kept in **SI** — pascals, kilograms a cubic metre — whatever the document's unit, since what
//! a material is does not change with the unit a drawing is dimensioned in (`Units::to_si`).

use crate::units::{Dim, Units};
use std::collections::BTreeMap;

/// A property a material may state.  Each is optional: a material states what its analyses need,
/// and an analysis that needs one it does not state says so where it is asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Property {
    /// Young's modulus, a pressure.
    E,
    /// Poisson's ratio, a plain number.
    Nu,
    /// Density, a mass over a volume.
    Rho,
    /// Yield strength, a pressure.
    Yield,
}

impl Property {
    pub const ALL: [Property; 4] = [Property::E, Property::Nu, Property::Rho, Property::Yield];

    /// How the language spells it: the label a `material(…)` gives it, and its JSON key.
    pub fn name(self) -> &'static str {
        match self {
            Property::E => "E",
            Property::Nu => "nu",
            Property::Rho => "rho",
            Property::Yield => "yield",
        }
    }

    /// The property `s` names, or the complaint that it names none.
    pub fn named(s: &str) -> Result<Property, String> {
        Property::ALL.into_iter().find(|p| p.name() == s)
            .ok_or_else(|| format!("`{s}` is no property of a material: {}", Property::names()))
    }

    /// What a reader calls it.
    pub fn what(self) -> &'static str {
        match self {
            Property::E => "Young's modulus",
            Property::Nu => "Poisson's ratio",
            Property::Rho => "a density",
            Property::Yield => "a yield strength",
        }
    }

    pub fn dim(self) -> Dim {
        match self {
            Property::E | Property::Yield => Dim::PRESSURE,
            Property::Nu => Dim::SCALAR,
            Property::Rho => Dim::DENSITY,
        }
    }

    /// The property's names, for a complaint about one that is none of them.
    pub fn names() -> String {
        let ns: Vec<String> = Property::ALL.iter().map(|p| format!("`{}`", p.name())).collect();
        ns.join(", ")
    }

    /// A value worked out in the document's units, `dim` what it came to, as this property in SI
    /// — or why it is none.  **A bare number is refused where the property has a dimension**: a
    /// length slot reads one in the document's unit, but the document's unit of pressure is a
    /// kilogram a millimetre-second², which no one means by `200000`, so the text says its unit.
    pub fn read(self, value: f64, dim: Dim, units: Units) -> Result<f64, String> {
        let want = self.dim();
        if dim.is_scalar() && !want.is_scalar() {
            return Err(format!("`{}` is {}, {}: a bare number says no unit, so write one ({})",
                               self.name(), self.what(), want.name(), self.example()));
        }
        dim.require(want, self.name())?;
        let si = units.to_si(value, want);
        self.admits(si)?;
        Ok(si)
    }

    /// A value written as a person writes it, for a complaint.
    fn example(self) -> &'static str {
        match self {
            Property::E => "`200GPa`",
            Property::Nu => "`0.3`",
            Property::Rho => "`7850kg / 1m^3`",
            Property::Yield => "`250MPa`",
        }
    }

    /// Whether an SI value is one this property may take: a stiffness, a density and a strength
    /// are positive, and Poisson's ratio of an isotropic solid lies in (-1, 0.5) — at 0.5 it is
    /// incompressible and its bulk modulus infinite.
    pub fn admits(self, v: f64) -> Result<(), String> {
        let ok = v.is_finite() && match self {
            Property::Nu => v > -1.0 && v < 0.5,
            _ => v > 0.0,
        };
        if ok {
            return Ok(());
        }
        Err(match self {
            Property::Nu => format!("`nu` is Poisson's ratio, which lies between -1 and 0.5, and \
                                     this is {}", self.text(v)),
            _ => format!("`{}` is {}, which is positive, and this is {}", self.name(),
                         self.what(), self.text(v)),
        })
    }

    /// An SI value as Solvent writes it, `200GPa`, `0.3`, `7850kg / 1m^3`: what a program lifted
    /// from a sketch states (`program::to_program`), each figure shortest round-trip.
    pub fn source(self, v: f64) -> String {
        self.spell(v, crate::vocab::num, "kg / 1m^3")
    }

    /// An SI value as a reader reads it: `200GPa`, `0.3`, `7850kg/m^3` — to twelve figures, which
    /// is a report's to round, since `7850kg / 1m^3` worked out in millimetres comes back a hair
    /// under 7850 (the value kept is the value worked out).
    pub fn text(self, v: f64) -> String {
        self.spell(v, |v| crate::json::fmt_g(v, 12), "kg/m^3")
    }

    /// An SI value with its unit, a pressure under the largest prefix it reaches: `source` and
    /// `text`, which differ in how a figure is written and how a density's unit is.
    fn spell(self, v: f64, num: impl Fn(f64) -> String, density: &str) -> String {
        match self {
            Property::E | Property::Yield => {
                let prefixed = [(1e9, "GPa"), (1e6, "MPa"), (1e3, "kPa")];
                match prefixed.iter().find(|(per, _)| v.abs() >= *per) {
                    Some((per, unit)) => format!("{}{unit}", num(v / per)),
                    None => format!("{}Pa", num(v)),
                }
            }
            Property::Nu => num(v),
            Property::Rho => format!("{}{density}", num(v)),
        }
    }
}

/// **What a solid is made of** (#510 P3): a name and the properties stated of it, in SI.  Held by
/// `Sketch::materials` under its name, which a solid made of it names (`SolidE::material`).
#[derive(Clone, Debug, PartialEq)]
pub struct Material {
    /// What the document calls it, absolute: `steel`, or `P0.steel` inside an instance.
    pub name: String,
    /// What is stated, each in SI, every one admitted (`Property::admits`).
    pub props: BTreeMap<Property, f64>,
}

impl Material {
    /// A material stating `props` (SI), or why it is none: one that states nothing, or a value no
    /// property takes.
    pub fn new(name: impl Into<String>, props: BTreeMap<Property, f64>)
        -> Result<Material, String> {
        let name = name.into();
        if props.is_empty() {
            return Err(format!("`{name}` states no property: a material says what it is made \
                                of, {}", Property::names()));
        }
        for (&p, &v) in &props {
            p.admits(v)?;
        }
        Ok(Material { name, props })
    }

    /// A property, in SI, where the material states it.
    pub fn get(&self, p: Property) -> Option<f64> {
        self.props.get(&p).copied()
    }

    /// It, as a reader reads it: `steel: E 200GPa, nu 0.3, rho 7850kg/m^3, yield 250MPa`.
    pub fn text(&self) -> String {
        let parts: Vec<String> =
            self.props.iter().map(|(p, &v)| format!("{} {}", p.name(), p.text(v))).collect();
        format!("{}: {}", self.name, parts.join(", "))
    }
}
