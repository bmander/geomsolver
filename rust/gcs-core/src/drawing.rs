//! Solvent Drawing (.svd): presentation of immutable solved models.
//!
//! The host loads model/import texts and solves models. Drawing compilation reads those
//! snapshots by name; it cannot add equations, choose a solution, or move model geometry.
mod parser;
mod render;
mod compile;
mod highlight;

use crate::style::Style;
use crate::syntax::Span;
pub use parser::parse;
pub use render::{fills, named_view, plane_across, plane_named, render, Model};
pub use compile::{colours, compile, compile_with, Compiled, Loaded};
pub use highlight::highlight;

#[derive(Clone, Debug)]
pub struct Error {
    pub span: Span,
    pub message: String,
}

#[derive(Clone, Debug, Default)]
pub struct Document {
    pub models: Vec<ModelImport>,
    pub imports: Vec<String>,
    pub styles: Vec<Rule>,
    pub sheets: Vec<Sheet>,
}

#[derive(Clone, Debug)]
pub struct ModelImport {
    pub name: String,
    pub path: String,
    /// The instant the model is shown at (`model m from "…" at t = 90deg`, #398 L): one of its
    /// declared parameters and the value, as written.  Presentation: the model is drawn as it is
    /// and stood there, on the branch its march carries.
    pub at: Option<Instant>,
    /// The values the drawing gives the model's own (`model m from "…" with g_wf = 2in`, #940):
    /// each a name the model binds at its top level and the value's text, read where the model
    /// binds it as `solventc --set` reads one.  Presentation of a variant: the file is unchanged.
    pub with: Vec<Given>,
    pub span: Span,
}

/// A value a drawing gives one of its model's (`with g_wf = 2in`, #940).
#[derive(Clone, Debug)]
pub struct Given {
    pub name: String,
    pub text: String,
    pub span: Span,
}

/// An instant of a model's declared parameter, as a drawing writes it.
#[derive(Clone, Debug)]
pub struct Instant {
    pub name: String,
    pub text: String,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Sheet {
    pub name: String,
    /// Physical paper dimensions, in millimetres.
    pub size: (f64, f64),
    pub scale: f64,
    pub views: Vec<View>,
    pub styles: Vec<Rule>,
    pub labels: Vec<Label>,
    /// How a figure the drawing works out (an extent, a `measure`) is written (`format fraction
    /// 1/8`, `format decimal 2`, #940).
    pub figures: Figures,
    pub span: Span,
}

/// **How a drawing writes a length it worked out** (#940): to six significant figures (a reading,
/// the default), to so many decimal places (`format decimal 2`, `87.99`), or to the nearest
/// fraction of the unit (`format fraction 1/8`, `87 7/8"` in an inch model, `7' 3 7/8"` in a foot
/// one — the fraction always an inch's there).  A dimension the model states is drawn as written.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Figures {
    #[default]
    Reading,
    Decimal(usize),
    /// The denominator: 8 for `1/8`.
    Fraction(u32),
}

impl Figures {
    /// `v`, a length in a model of `units`, written this way.
    pub fn text(self, v: f64, units: crate::units::Units) -> String {
        match self {
            Figures::Reading => crate::expr::reading(crate::constraints::SpecKind::Length, v),
            Figures::Decimal(places) => {
                let t = format!("{v:.places$}");
                // a figure that rounds to nothing has no sign
                if t.trim_start_matches('-').chars().all(|c| c == '0' || c == '.') {
                    t.trim_start_matches('-').to_string()
                } else { t }
            }
            Figures::Fraction(d) => {
                let sign = if v < 0.0 { "-" } else { "" };
                let (feet, inches) = match units.name() {
                    "ft" => {
                        // whole feet, and the rest in inches to the nearest fraction
                        let n = (v.abs() * 12.0 * d as f64).round() as u64;
                        let per_foot = 12 * d as u64;
                        (Some(n / per_foot), n % per_foot)
                    }
                    _ => (None, (v.abs() * d as f64).round() as u64),
                };
                let (whole, rest) = (inches / d as u64, inches % d as u64);
                let fraction = if rest == 0 { String::new() } else {
                    let g = gcd(rest, d as u64);
                    format!("{}/{}", rest / g, d as u64 / g)
                };
                let number = match (whole, fraction.is_empty()) {
                    (_, true) => whole.to_string(),
                    (0, false) => fraction,
                    (_, false) => format!("{whole} {fraction}"),
                };
                let mark = if matches!(units.name(), "in" | "ft") { "\"" } else { "" };
                match feet {
                    Some(f) if f > 0 => format!("{sign}{f}' {number}{mark}"),
                    _ => format!("{sign}{number}{mark}"),
                }
            }
        }
    }
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a.max(1) } else { gcd(b, a % b) }
}

#[derive(Clone, Debug)]
pub struct View {
    pub name: String,
    pub target: String,
    /// `front`, `back`, `right`, `left`, `top`, `bottom`, or a model plane path.
    pub direction: String,
    pub cut: Option<String>,
    /// A sketch preview reads 2D entities; a solid view projects evaluated material.
    pub sketch: bool,
    /// Page coordinates in mm, x right, y down: where the projected origin stands, or the point
    /// of the view's own box `anchor` names.
    pub at: (f64, f64),
    /// The point of the box round what the view draws that `at` places (`at topleft (…)`, #940),
    /// as fractions across it from its left and down it from its top; `None` places the origin.
    pub anchor: Option<(f64, f64)>,
    pub scale: Option<f64>,
    /// `scale fit (W, H)` (#940): the largest scale at which what the view draws fits a box W by H
    /// on the paper, in millimetres.
    pub fit: Option<(f64, f64)>,
    pub dimensions: bool,
    /// What `dimensions SEL, … in VIEW` narrows a sketch's dimensions to (#940): each a model
    /// path (the dimensions whose geometry is all under it) or a class (`.structural`); empty,
    /// every dimension of the view's geometry.
    pub dimensions_of: Vec<String>,
    pub annotations: Vec<Annotation>,
    pub measurements: Vec<Measurement>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Annotation {
    /// A named model dimension, never a constraint index or source line number.
    pub target: String,
    /// Placement in the dimension's own frame, as in the callout engine.
    pub at: Option<(f64, f64)>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Measurement {
    pub points: (String, String),
    /// Paper millimetres away from the measured segment.
    pub offset: f64,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Rule {
    /// An implicit class (`.hidden`), or a model entity/component path.
    pub selector: String,
    /// The one view of its sheet it styles (`style m.rod2 in end { … }`, #461), or every view.
    pub view: Option<String>,
    pub style: Style,
    /// `fill`: the colour of the parts it names, in the workspace and in the STEP export (#677)
    /// — not a sheet's ink, so only a rule outside every sheet states it.
    pub fill: Option<[u8; 3]>,
    pub span: Span,
}

#[derive(Clone, Debug)]
pub struct Label {
    pub text: String,
    pub at: (f64, f64),
}

impl Document {
    /// Imports are styles only. Resolve them in the host, relative to the importing file.
    /// Keeping this explicit makes browser, CLI, and in-memory callers share the same language.
    pub fn prepend_styles(&mut self, imported: &Document) -> Result<(), String> {
        if !imported.models.is_empty() || !imported.sheets.is_empty() {
            return Err("a drawing `use` imports styles; model imports and sheets belong in the drawing".into());
        }
        self.styles.splice(0..0, imported.styles.iter().cloned());
        Ok(())
    }
}

/// Resolve a host bundle path without filesystem access. The host owns access policy;
/// this normalization only gives imports and cycle detection a consistent identity.
pub fn relative_path(path: &str, from: &str) -> String {
    let joined = if path.starts_with('/') { path.to_string() }
        else { format!("{}{}", from.rsplit_once('/').map_or("", |(p, _)| &from[..p.len() + 1]), path) };
    let mut parts = Vec::new();
    for p in joined.split('/') {
        match p {
            "" | "." => {},
            ".." if parts.last().is_some_and(|p| *p != "..") => { parts.pop(); },
            p => parts.push(p),
        }
    }
    format!("{}{}", if joined.starts_with('/') { "/" } else { "" }, parts.join("/"))
}

/// An editor may request an automatic preview of a bare model. This is host presentation
/// state, installed after elaboration, and never serialized as .sv source.
pub fn preview(sk: &mut crate::model::Sketch, plane: Option<usize>) {
    sk.derived = crate::overview::objects(sk).into_iter().map(|i| crate::model::DerivedE {
        solid: i as u32, plane: plane.filter(|&p| p < sk.frames.len()).map(|p| p as u32),
        at: None, dims: false, name: sk.solids[i].name.clone(), class: Default::default(),
    }).collect();
}
