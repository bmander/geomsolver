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
    /// `display: none | inline | geometry` — see `Display`.
    pub display: Option<Display>,
}

/// What `display` says is drawn (§13.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Display {
    /// Not drawn at all; a dimension is neither laid out nor picked.
    None,
    /// Drawn — and, from a later class, drawn again after an earlier class hid it.
    Inline,
    /// Drawn, but never dimensioned: a phantom position is geometry only.  An entity under it
    /// is shown; a dimension whose statement carries it is not.
    Geometry,
}

impl Display {
    pub fn parse(s: &str) -> Option<Display> {
        Some(match s {
            "none" => Display::None,
            "inline" => Display::Inline,
            "geometry" => Display::Geometry,
            _ => return None,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Display::None => "none",
            Display::Inline => "inline",
            Display::Geometry => "geometry",
        }
    }
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
        if let Some(d) = other.display {
            self.display = Some(d);
        }
    }

    /// Whether what carries this style is drawn at all.
    pub fn shown(&self) -> bool {
        self.display != Some(Display::None)
    }

    /// Whether a dimension whose statement carries this style is laid out: not under `none`,
    /// and not under `geometry` either.
    pub fn dimensioned(&self) -> bool {
        !matches!(self.display, Some(Display::None) | Some(Display::Geometry))
    }

    /// One property, by the name a `style` block writes.  `false` is a property the sheet cannot
    /// use — a name it does not know, *or* a value it cannot read — which is not an error,
    /// exactly as an unmatched class is not.
    ///
    /// A value it cannot read is dropped rather than stored, because a half-stated property is
    /// worse than an absent one: `color:` with nothing after it stored `Some("")`, and an empty
    /// string is not nullish, so it travelled all the way to `ctx.fillStyle`, which ignores an
    /// assignment it cannot parse and leaves the previous colour standing.  Every dimension's
    /// label came out drawn in the background colour.  `None` already means *says nothing*, and
    /// that is what a value with no reading says.
    ///
    /// `dash` is the one that keeps an empty list: "empty or absent is solid" is what the field
    /// documents, so `dash:` states solid and has a reading of its own.
    /// The property names this style actually states, in the order `set` reads them.
    ///
    /// One table, for `Ty::parse`'s reason: a `StyleRule` lifted out of a `Sketch` has no source
    /// to have said what it said, so what it prints is what the `Style` holds — and a fourth
    /// property would otherwise parse, cascade and paint while silently never printing back.
    pub fn stated(&self) -> Vec<&'static str> {
        let mut out = Vec::with_capacity(3);
        if self.dash.is_some() {
            out.push("dash");
        }
        if self.width.is_some() {
            out.push("width");
        }
        if self.color.is_some() {
            out.push("color");
        }
        if self.display.is_some() {
            out.push("display");
        }
        out
    }

    /// Whether `prop` is a property a sheet can state at all — as against one it can state
    /// but was given no reading for, which `set` alone cannot tell apart.
    pub fn knows(prop: &str) -> bool {
        matches!(prop, "dash" | "width" | "color" | "display")
    }

    pub fn set(&mut self, prop: &str, values: &[f64], text: &str) -> bool {
        match prop {
            "dash" => self.dash = Some(values.to_vec()),
            "width" if !values.is_empty() => self.width = Some(values[0]),
            "color" if !text.is_empty() => self.color = Some(text.to_string()),
            // CSS's words, and one of the draughtsman's: `none` hides, `inline` shows again —
            // which a later class needs a way to say, since `None` is *says nothing* — and
            // `geometry` draws without dimensioning
            "display" => match Display::parse(text) {
                Some(d) => self.display = Some(d),
                None => return false,
            },
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
    let rule = |dash: Option<Vec<f64>>, width: Option<f64>, color: Option<&str>| Style {
        dash,
        width,
        color: color.map(str::to_string),
        display: None,
    };
    // reference geometry.  What the retired `construction` keyword did, and the whole of it.
    s.insert("construction".into(), rule(Some(vec![7.0, 4.0]), None, None));
    // A dimension callout's ink, and nothing about its *figure*: the extension lines, the heads,
    // the label's box and the hit test are geometry, laid out by `callout.rs` in world units so
    // that every front end agrees where the figure is (issue #14).  What is presentational there
    // is the ink it is stroked in, which is here.
    //
    // These are what makes a *placement* fit to stay on its statement (§13.1): a class is a rule
    // many statements share, and everything a callout shares is in these three rules.  What is
    // left on the statement is the one pair of numbers that is about that statement alone.
    s.insert("dimension".into(), rule(None, Some(1.0), Some("#0f6f7a")));
    // a claimed dimension — the draughtsman's reference dimension, which `callout.rs` draws
    // parenthesised.  Lighter than a controlling one, because it does not control.
    //
    // **Only the difference.**  A reference dimension *is* a dimension, so it is drawn with both
    // classes and this rule says the one thing that is not shared: the lighter ink.  Restating
    // the weight here would make `.reference` a complete rule rather than a modifier, and a
    // document that said `style .dimension { width: 2 }` would get thick controlling dimensions
    // and thin claimed ones — a split drawing, from a sheet that named neither.
    s.insert("reference".into(), rule(None, None, Some("#7aa7ad")));
    // a dimension's extension and witness lines: finer dashes than reference geometry, being
    // annotation rather than something the sketch is made of
    s.insert("extension".into(), rule(Some(vec![4.0, 3.0]), None, None));
    // a plane's datum glyph — the chord from its origin to the point it is turned toward, which
    // is where a view is taken hold of.  Fine and light: it is a label on the sheet, not a
    // line of the object.  The kind carries the class itself (`EntKind::implicit_class`), so a
    // document's own `style .plane` rule overrides this one without the declaration saying so.
    s.insert("plane".into(), rule(Some(vec![2.0, 3.0]), Some(0.75), Some("#8a8a8a")));
    s
}

/// What a class list comes to: the base sheet under the document's, each cascaded in written
/// order.
///
/// **Two layers, not one interleaved pass.**  What a document says beats what the implementation
/// ships, whichever class it happens to be written on — the rule CSS states between an author
/// sheet and the user agent's, and the reason the base sheet is describable as being *under* the
/// document's at all.  Resolved a class at a time (base, sheet, base, sheet), a later class's
/// shipped rule would override an earlier class's *stated* one: `class dimension reference` with
/// `style .dimension { color: #b00020 }` came back the base `.reference` teal, so the one
/// document rule anybody would write to recolour their callouts recoloured half of them.
///
/// An unmatched class simply has no rule, exactly as in CSS — which is also what makes paste
/// work, since a figure copied out of a document with a sheet keeps its class names and picks up
/// whatever the destination says about them, or nothing.
pub fn resolve(sheet: &Sheet, classes: &Classes) -> Style {
    // the shipped rules never change, and this runs once per entity per repaint and twice per
    // callout in the SVG export — rebuilding four rules and their names each time is work no
    // caller asked for
    static BASE: std::sync::OnceLock<Sheet> = std::sync::OnceLock::new();
    let mut out = Style::default();
    for layer in [BASE.get_or_init(base), sheet] {
        for c in &classes.0 {
            if let Some(r) = layer.get(c) {
                out.over(r);
            }
        }
    }
    out
}
