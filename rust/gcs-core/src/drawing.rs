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
pub use render::{render, Model};
pub use compile::compile;
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
    pub span: Span,
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
    /// Page coordinates in mm, x right, y down; this is the projected origin.
    pub at: (f64, f64),
    pub scale: Option<f64>,
    pub dimensions: bool,
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
    pub style: Style,
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
        solid: i as u32, plane: plane.filter(|&p| p < sk.planes.len()).map(|p| p as u32),
        at: None, dims: false, name: sk.solids[i].name.clone(), class: Default::default(),
    }).collect();
}
