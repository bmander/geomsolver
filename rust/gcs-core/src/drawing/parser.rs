//! A small, bounded drawing grammar. No model statements are accepted here.
use super::*;
use std::collections::BTreeSet;

#[derive(Clone)]
pub(super) struct Token { pub(super) text: String, pub(super) quoted: bool, pub(super) span: Span }
struct P<'t> { tokens: Vec<Token>, i: usize, text: &'t str }
type Result<T> = std::result::Result<T, Error>;

/// The points of a view's box `at` may place (#940), as fractions across it from its left and
/// down it from its top.
const ANCHORS: [(&str, (f64, f64)); 9] = [
    ("topleft", (0.0, 0.0)), ("top", (0.5, 0.0)), ("topright", (1.0, 0.0)),
    ("left", (0.0, 0.5)), ("center", (0.5, 0.5)), ("right", (1.0, 0.5)),
    ("bottomleft", (0.0, 1.0)), ("bottom", (0.5, 1.0)), ("bottomright", (1.0, 1.0)),
];

pub fn parse(text: &str) -> Result<Document> {
    if text.len() > crate::syntax::MAX_TEXT {
        return Err(Error { span: Span::default(), message: "drawing is too large".into() });
    }
    let mut p = P { tokens: lex(text)?, i: 0, text };
    let mut doc = Document::default();
    let mut names = BTreeSet::new();
    while !p.done() {
        let span = p.span();
        match p.word()?.as_str() {
            "model" => {
                let name = p.name()?;
                if !names.insert(name.clone()) { return p.fail("model alias declared twice"); }
                p.want("from")?;
                let path = p.string()?;
                let at = if p.eat("at") { Some(p.instant()?) } else { None };
                let mut with = Vec::new();
                if p.eat("with") {
                    loop {
                        let given = p.given()?;
                        if with.iter().any(|g: &Given| g.name == given.name) {
                            return Err(Error { span: given.span,
                                message: format!("`{}` is given a value twice", given.name) });
                        }
                        with.push(given);
                        if !p.eat(",") { break }
                    }
                }
                doc.models.push(ModelImport { name, path, at, with, span });
            }
            "use" => doc.imports.push(p.string()?),
            "style" => {
                let rule = p.rule(span)?;
                if rule.view.is_some() {
                    return Err(Error { span, message: "a style for one view stands in that \
                        view's sheet: `sheet NAME { … style SELECTOR in VIEW { … } }`".into() });
                }
                doc.styles.push(rule)
            }
            "sheet" => {
                let sheet = p.sheet(span)?;
                if doc.sheets.iter().any(|s| s.name == sheet.name) {
                    return p.fail("sheet name declared twice");
                }
                doc.sheets.push(sheet);
            }
            _ => return p.fail("expected `model`, `use`, `style`, or `sheet` in a .svd drawing"),
        }
        p.eat(";");
    }
    Ok(doc)
}

impl P<'_> {
    fn done(&self) -> bool { self.i == self.tokens.len() }
    fn span(&self) -> Span { self.tokens.get(self.i).map(|t| t.span).unwrap_or_default() }
    fn peek(&self, s: &str) -> bool {
        self.tokens.get(self.i).is_some_and(|t| !t.quoted && t.text == s)
    }
    fn eat(&mut self, s: &str) -> bool {
        if self.peek(s) { self.i += 1; true } else { false }
    }
    fn fail<T>(&self, s: &str) -> Result<T> {
        Err(Error { span: self.span(), message: s.into() })
    }
    fn want(&mut self, s: &str) -> Result<()> {
        if self.eat(s) { Ok(()) } else { self.fail(&format!("expected `{s}`")) }
    }
    fn word(&mut self) -> Result<String> {
        let Some(t) = self.tokens.get(self.i) else { return self.fail("unexpected end of drawing") };
        if t.quoted { return self.fail("expected a word, found a string") }
        let s = t.text.clone(); self.i += 1; Ok(s)
    }
    fn name(&mut self) -> Result<String> {
        let span = self.span();
        let s = self.word()?;
        if !crate::syntax::is_name(&s) {
            return Err(Error { span, message: format!("`{s}` is not a name") });
        }
        Ok(s)
    }
    fn string(&mut self) -> Result<String> {
        let Some(t) = self.tokens.get(self.i) else { return self.fail("expected a quoted string") };
        if !t.quoted { return self.fail("expected a quoted string") }
        let s = t.text.clone(); self.i += 1; Ok(s)
    }
    /// `t = 90deg`: a parameter's name and its value's text, which the model's own units read
    /// (`follow::instant`) once it is loaded.  Written `t=90deg` it is one word.
    fn instant(&mut self) -> Result<Instant> {
        let span = self.span();
        let word = self.word()?;
        let (name, text) = match word.split_once('=') {
            Some((n, v)) if !v.is_empty() => (n.to_string(), v.to_string()),
            Some((n, _)) => (n.to_string(), self.word()?),
            None if self.eat("=") => (word, self.word()?),
            None => return self.fail("expected `=` and the instant: `at t = 90deg`"),
        };
        if !crate::syntax::is_name(&name) {
            return Err(Error { span, message: format!("`{name}` is not a name") });
        }
        let hi = self.tokens[self.i - 1].span.hi as usize;
        Ok(Instant { name, text, span: Span::new(span.lo as usize, hi) })
    }
    /// `g_wf = 2in` of a model's `with` (#940): a name the model binds and the value's text, as
    /// far as a `,` or `;` outside parentheses, or the next statement; written `g_wf=2in` it is one
    /// word, and a quoted value (`"1' 6\""`) is the text inside the quotes.
    fn given(&mut self) -> Result<Given> {
        let span = self.span();
        let word = self.word()?;
        // where the value starts in the source when it is glued to its `=` (`g=2in`)
        let mut glued = None;
        let (name, first) = match word.split_once('=') {
            Some((n, v)) => {
                if !v.is_empty() { glued = Some(span.lo as usize + n.len() + 1); }
                (n.to_string(), (!v.is_empty()).then(|| v.to_string()))
            }
            None if self.eat("=") => (word, None),
            None => return self.fail("expected `=` and a value: `with g_wf = 2in`"),
        };
        if !crate::syntax::is_name(&name) {
            return Err(Error { span, message: format!("`{name}` is not a name") });
        }
        let mut parts: Vec<String> = first.into_iter().collect();
        let from = self.i;
        if parts.is_empty() && self.tokens.get(self.i).is_some_and(|t| t.quoted) {
            parts.push(self.string()?);
        } else {
            let mut depth = 0usize;
            while let Some(t) = self.tokens.get(self.i) {
                let stop = !t.quoted && match t.text.as_str() {
                    "," | ";" => depth == 0,
                    "model" | "use" | "style" | "sheet" => depth == 0,
                    "{" | "}" => true,
                    _ => false,
                };
                if stop || t.quoted { break }
                match t.text.as_str() {
                    "(" => depth += 1,
                    ")" if depth == 0 => break,
                    ")" => depth -= 1,
                    _ => {}
                }
                parts.push(t.text.clone());
                self.i += 1;
            }
        }
        // the value as written, from after its `=` to its last token
        let quoted = self.tokens.get(from).is_some_and(|t| t.quoted) && self.i == from + 1;
        let text = if quoted { parts.join(" ") } else {
            let lo = match glued {
                Some(lo) => lo,
                None => self.tokens.get(from).map_or(0, |t| t.span.lo as usize),
            };
            let hi = self.tokens[self.i - 1].span.hi as usize;
            if hi > lo { self.text[lo..hi].to_string() } else { String::new() }
        };
        if text.trim().is_empty() {
            return self.fail(&format!("`{name}` is given no value"));
        }
        let hi = self.tokens[self.i - 1].span.hi as usize;
        Ok(Given { name, text, span: Span::new(span.lo as usize, hi) })
    }
    fn number(&mut self, length: bool) -> Result<f64> {
        let span = self.span();
        let s = self.word()?;
        let (raw, factor) = if length {
            if let Some(v) = s.strip_suffix("mm") { (v, 1.0) }
            else if let Some(v) = s.strip_suffix("cm") { (v, 10.0) }
            else if let Some(v) = s.strip_suffix("in") { (v, 25.4) }
            else { (s.as_str(), 1.0) }
        } else { (s.as_str(), 1.0) };
        match raw.parse::<f64>() {
            Ok(n) if n.is_finite() && (n * factor).is_finite() => Ok(n * factor),
            _ => Err(Error { span, message: format!("`{s}` is not a finite {}", if length { "page length (mm, cm, or in)" } else { "number" }) }),
        }
    }
    fn positive(&mut self, length: bool) -> Result<f64> {
        let n = self.number(length)?;
        if n <= 0.0 { return self.fail("size and scale must be positive") }
        Ok(n)
    }
    fn pair(&mut self, length: bool) -> Result<(f64, f64)> {
        self.want("(")?; let x = self.number(length)?;
        self.want(",")?; let y = self.number(length)?; self.want(")")?;
        Ok((x, y))
    }
    fn sheet(&mut self, span: Span) -> Result<Sheet> {
        let name = self.name()?;
        self.want("{")?;
        let mut s = Sheet { name, size: (210.0, 297.0), scale: 1.0, views: Vec::new(),
            styles: Vec::new(), labels: Vec::new(), figures: Figures::Reading, span };
        let mut annotations = Vec::new();
        let mut measurements = Vec::new();
        let mut filters: Vec<(String, Vec<String>)> = Vec::new();
        while !self.eat("}") {
            let span = self.span();
            match self.word()?.as_str() {
                "size" => {
                    self.eat(":");
                    s.size = if self.eat("A4") { (210.0, 297.0) }
                        else if self.eat("A3") { (297.0, 420.0) }
                        else if self.eat("Letter") { (215.9, 279.4) }
                        else { self.pair(true)? };
                    if s.size.0 <= 0.0 || s.size.1 <= 0.0 { return self.fail("paper size must be positive") }
                }
                "scale" => { self.eat(":"); s.scale = self.positive(false)?; }
                // how a figure the drawing works out is written (#940)
                "format" => {
                    self.eat(":");
                    let span = self.span();
                    s.figures = match self.word()?.as_str() {
                        "reading" => Figures::Reading,
                        "decimal" => {
                            let n = self.number(false)?;
                            if n.fract() != 0.0 || !(0.0..=10.0).contains(&n) {
                                return Err(Error { span, message: "`format decimal N` takes a \
                                    whole number of places, 0 to 10".into() });
                            }
                            Figures::Decimal(n as usize)
                        }
                        "fraction" => {
                            let word = self.word()?;
                            let d = word.strip_prefix("1/").and_then(|d| d.parse::<u32>().ok())
                                .filter(|d| (1..=1024).contains(d));
                            match d {
                                Some(d) => Figures::Fraction(d),
                                None => return Err(Error { span, message: "`format fraction` \
                                    takes the finest fraction, `1/8`, `1/16` (to 1/1024)".into() }),
                            }
                        }
                        _ => return Err(Error { span, message: "expected `format reading`, \
                            `format decimal N` or `format fraction 1/N`".into() }),
                    };
                }
                kind @ ("view" | "section" | "sketch") => {
                    let name = self.name()?;
                    if s.views.iter().any(|v| v.name == name) { return self.fail("view name declared twice") }
                    self.want("(")?; let target = self.word()?; self.want(")")?;
                    let mut v = View { name, target, direction: "front".into(), cut: None,
                        sketch: kind == "sketch", at: (0.0, 0.0), anchor: None, scale: None,
                        fit: None, dimensions: false, dimensions_of: Vec::new(),
                        annotations: Vec::new(),
                        measurements: Vec::new(), span };
                    if self.eat("from") { v.direction = self.word()?; }
                    if kind == "section" { self.want("cut")?; v.cut = Some(self.word()?); }
                    self.want("at")?;
                    // `at topleft (x, y)`: a point of the view's own box, not the origin (#940)
                    v.anchor = ANCHORS.iter().find(|(w, _)| self.peek(w)).map(|&(_, a)| a);
                    if v.anchor.is_some() { self.i += 1; }
                    v.at = self.pair(true)?;
                    if self.eat("scale") {
                        if self.eat("fit") {
                            let (w, h) = self.pair(true)?;
                            if w <= 0.0 || h <= 0.0 {
                                return self.fail("a fit's box must be positive");
                            }
                            v.fit = Some((w, h));
                        } else {
                            v.scale = Some(self.positive(false)?);
                        }
                    }
                    s.views.push(v);
                }
                // `dimensions m.frame, .structural in VIEW`: those alone (#940)
                "dimensions" => {
                    let mut of = Vec::new();
                    while !self.peek("in") {
                        let span = self.span();
                        let w = self.word()?;
                        if w == "," || w == ";" || w == "}" {
                            return Err(Error { span, message: "expected a model path or a \
                                class, then `in VIEW`".into() });
                        }
                        of.push(w);
                        self.eat(",");
                    }
                    self.want("in")?;
                    let view = self.name()?;
                    filters.push((view.clone(), of));
                    annotations.push((view, None));
                }
                "dimension" => {
                    let target = self.word()?;
                    self.want("in")?; let view = self.name()?;
                    let at = if self.eat("at") { Some(self.pair(false)?) } else { None };
                    annotations.push((view, Some(Annotation { target, at, span })));
                }
                "measure" => {
                    self.want("distance")?; self.want("(")?;
                    let a = self.word()?; self.want(",")?; let b = self.word()?;
                    self.want(")")?; self.want("in")?; let view = self.name()?;
                    let offset = if self.eat("offset") { self.number(true)? } else { 6.0 };
                    measurements.push((view, Measurement { points: (a, b), offset, span }));
                }
                "label" => {
                    let text = self.string()?; self.want("at")?;
                    s.labels.push(Label { text, at: self.pair(true)? });
                }
                "style" => {
                    let rule = self.rule(span)?;
                    if rule.fill.is_some() {
                        return Err(Error { span, message: "`fill` colours a part in the workspace \
                            and the STEP export, not a sheet's ink: state it in a style outside \
                            every sheet".into() });
                    }
                    s.styles.push(rule)
                }
                _ => return self.fail("expected size, scale, format, view, section, sketch, dimensions, dimension, measure, label, or style"),
            }
            self.eat(";");
        }
        for (name, a) in annotations {
            let Some(v) = s.views.iter_mut().find(|v| v.name == name) else {
                return self.fail(&format!("no view named `{name}`"));
            };
            if let Some(a) = a { v.annotations.push(a); } else { v.dimensions = true; }
        }
        // a bare `dimensions in VIEW` asks for all of them, whatever else narrows it
        for (name, of) in filters {
            if let Some(v) = s.views.iter_mut().find(|v| v.name == name) {
                if of.is_empty() { v.dimensions_of.clear(); v.dimensions_of.push(String::new()); }
                else if v.dimensions_of.first().is_none_or(|f| !f.is_empty()) {
                    v.dimensions_of.extend(of);
                }
            }
        }
        for v in &mut s.views {
            if v.dimensions_of.first().is_some_and(String::is_empty) { v.dimensions_of.clear(); }
        }
        for (name, m) in measurements {
            let Some(v) = s.views.iter_mut().find(|v| v.name == name) else {
                return self.fail(&format!("no view named `{name}`"));
            };
            v.measurements.push(m);
        }
        // a style for one view names one of this sheet's (#461)
        for r in &s.styles {
            if let Some(view) = r.view.as_ref().filter(|w| !s.views.iter().any(|v| &v.name == *w)) {
                return Err(Error { span: r.span, message: format!("no view named `{view}`") });
            }
        }
        Ok(s)
    }
    fn rule(&mut self, span: Span) -> Result<Rule> {
        let selector = self.word()?;
        // `in VIEW`: the rule styles that view of its sheet alone, as `dimensions in VIEW` asks
        // for one view's dimensions
        let view = if self.eat("in") { Some(self.name()?) } else { None };
        self.want("{")?;
        let mut style = Style::default();
        let mut fill = None;
        while !self.eat("}") {
            let prop = self.word()?; self.want(":")?;
            let mut vals = Vec::new();
            let text;
            match prop.as_str() {
                // a part's colour (#677): `fill: #e8a33d`, opaque
                "fill" => {
                    fill = Some(crate::colours::parse(&self.word()?)
                        .ok_or_else(|| Error { span: self.tokens[self.i - 1].span,
                            message: "fill must be #RGB or #RRGGBB".into() })?);
                    if !self.peek("}") { self.want(";")?; }
                    continue;
                }
                "color" | "display" => text = self.word()?,
                "width" => { vals.push(self.positive(false)?); text = String::new(); }
                "dash" => {
                    while !self.done() && !self.peek(";") && !self.peek("}") {
                        vals.push(self.positive(false)?);
                    }
                    text = String::new();
                }
                _ => return self.fail(&format!("unknown style property `{prop}`")),
            }
            if prop == "color" && !(text.starts_with('#') && matches!(text.len(), 4 | 7 | 9)
                && text[1..].bytes().all(|b| b.is_ascii_hexdigit())) {
                return self.fail("color must be #RGB, #RRGGBB, or #RRGGBBAA");
            }
            if !style.set(&prop, &vals, &text) { return self.fail(&format!("invalid `{prop}`")) }
            if !self.peek("}") { self.want(";")?; }
        }
        if fill.is_some() && (selector.starts_with('.') || view.is_some()) {
            return Err(Error { span, message: "`fill` colours the parts a model path names: \
                `style m.handle { fill: #e8a33d }`, for every view".into() });
        }
        Ok(Rule { selector, view, style, fill, span })
    }
}

fn lex(s: &str) -> Result<Vec<Token>> {
    let (tokens, _, error) = scan(s);
    match error { Some(e) => Err(e), None => Ok(tokens) }
}

/// The tokens as far as they read, the comments, and the error that stopped the scan if one did:
/// `parse` refuses at the error, and `highlight` colours what came before it.
pub(super) fn scan(s: &str) -> (Vec<Token>, Vec<Span>, Option<Error>) {
    let mut out = Vec::new(); let mut comments = Vec::new(); let mut i = 0;
    while i < s.len() {
        let c = s[i..].chars().next().unwrap();
        if c.is_whitespace() { i += c.len_utf8(); continue }
        if s[i..].starts_with("//") {
            let lo = i;
            i += s[i..].find('\n').unwrap_or(s.len() - i);
            comments.push(Span::new(lo, i));
            continue;
        }
        let lo = i;
        let quoted = c == '"';
        let text = if quoted {
            i += 1; let mut value = String::new(); let mut closed = false;
            while i < s.len() {
                let c = s[i..].chars().next().unwrap(); i += c.len_utf8();
                if c == '"' { closed = true; break }
                if c == '\\' {
                    let Some(c) = s[i..].chars().next() else { break };
                    i += c.len_utf8();
                    match c { '"' | '\\' => value.push(c), 'n' => value.push('\n'),
                        _ => {
                            out.push(Token { text: value, quoted, span: Span::new(lo, i) });
                            let e = Error { span: Span::new(lo, i), message: "unsupported string escape".into() };
                            return (out, comments, Some(e));
                        } }
                } else { value.push(c); }
            }
            if !closed {
                out.push(Token { text: value, quoted, span: Span::new(lo, i) });
                let e = Error { span: Span::new(lo, i), message: "unterminated string".into() };
                return (out, comments, Some(e));
            }
            value
        } else if "{}(),:;".contains(c) { i += 1; c.to_string() }
        else {
            while i < s.len() {
                let c = s[i..].chars().next().unwrap();
                if c.is_whitespace() || "{}(),:;\"".contains(c) || s[i..].starts_with("//") { break }
                i += c.len_utf8();
            }
            s[lo..i].to_string()
        };
        out.push(Token { text, quoted, span: Span::new(lo, i) });
        if out.len() > crate::syntax::MAX_STMTS * 32 {
            let e = Error { span: Span::new(lo, i), message: "too many drawing tokens".into() };
            return (out, comments, Some(e));
        }
    }
    (out, comments, None)
}
