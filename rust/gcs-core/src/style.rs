//! Presentation: what a drawing *looks* like, which is a separate statement from what it **is**.
//!
//! A Solvent document says what the drawing is.  How it is presented is a class on a declaration
//! and a `style` block that says what that class looks like — and **no algorithm in the core
//! consults either**.  Nothing here reaches the model, the kernels, diagnosis or decomposition;
//! it is read by `report` and stroked by a front end, and that is the whole of its reach.
//!
//! That is the point of it.  `construction` was a `bool` on seven entity structs, serialized,
//! grafted, exported, published by the binding and given a toggle — all to reach one arm in
//! `paint.ts`.  Each new look cost the same again.  A class is one string in the same places,
//! once, and the count goes into the sheet instead.
//!
//! **Lengths in a sheet are screen pixels**, never world units, which is the rule the codebase
//! already follows for everything drawn at a constant size (callout figures go through `unit`
//! for the same reason).  A dashed line does not change its dash pattern when you zoom.

use std::collections::BTreeMap;

/// The classes a declaration carries, in the order they were written: `class centerline heavy`.
///
/// Several, because one costs nothing now and forecloses nothing: on a conflicting property the
/// later class wins, so `class centerline heavy` is a centreline drawn thick.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Classes(pub Vec<String>);

impl Classes {
    pub fn one(name: &str) -> Classes {
        Classes(vec![name.to_string()])
    }

    pub fn has(&self, name: &str) -> bool {
        self.0.iter().any(|c| c == name)
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Add or remove one class, keeping the order of the rest — which is what decides a
    /// conflicting property, so a toggle must not reshuffle them.
    pub fn set(&mut self, name: &str, on: bool) {
        match (on, self.has(name)) {
            (true, false) => self.0.push(name.to_string()),
            (false, true) => self.0.retain(|c| c != name),
            _ => {}
        }
    }
}

/// What a class looks like.
///
/// `None` is *says nothing*, which is what makes the cascade work: a later class overrides only
/// the properties it states, so `class centerline heavy` keeps the centreline's dash and colour
/// and takes the heavy weight.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Style {
    /// Screen pixels, as `setLineDash` / `stroke-dasharray` take them.  Empty or absent is solid.
    pub dash: Option<Vec<f64>>,
    /// Stroke weight, in screen pixels.
    pub width: Option<f64>,
    /// `#rrggbb`.
    pub color: Option<String>,
}

impl Style {
    /// Lay `other` over this one: what it states wins, what it leaves out does not.
    pub fn over(&mut self, other: &Style) {
        if let Some(d) = &other.dash {
            self.dash = Some(d.clone());
        }
        if let Some(w) = other.width {
            self.width = Some(w);
        }
        if let Some(c) = &other.color {
            self.color = Some(c.clone());
        }
    }

    /// One property, by the name a `style` block writes.  `None` is a property the sheet does
    /// not know — which is not an error, exactly as an unmatched class is not.
    pub fn set(&mut self, prop: &str, values: &[f64], text: &str) -> bool {
        match prop {
            "dash" => self.dash = Some(values.to_vec()),
            "width" => self.width = values.first().copied(),
            "color" => self.color = Some(text.to_string()),
            _ => return false,
        }
        true
    }
}

/// A document's sheet: what each class looks like.
pub type Sheet = BTreeMap<String, Style>;

/// The rules the implementation ships, which a document may override.
///
/// `construction` is here and nowhere else.  It stopped being a word in the language when a
/// class could say the same thing; what it *did* is one rule, and a document that wants
/// reference geometry drawn some other way says so and changes nothing else about the drawing.
pub fn base() -> Sheet {
    let mut s = Sheet::new();
    s.insert(
        "construction".to_string(),
        Style { dash: Some(vec![7.0, 4.0]), width: None, color: None },
    );
    s
}

/// What a class list comes to: the base sheet under the document's, cascaded in written order.
///
/// An unmatched class simply has no rule, exactly as in CSS — which is also what makes paste
/// work, since a figure copied out of a document with a sheet keeps its class names and picks up
/// whatever the destination says about them, or nothing.
pub fn resolve(sheet: &Sheet, classes: &Classes) -> Style {
    let base = base();
    let mut out = Style::default();
    for c in &classes.0 {
        if let Some(r) = base.get(c) {
            out.over(r);
        }
        if let Some(r) = sheet.get(c) {
            out.over(r);
        }
    }
    out
}
