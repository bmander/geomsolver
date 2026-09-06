//! A small, bounded drawing grammar. No model statements are accepted here.
use super::*;
use std::collections::BTreeSet;

#[derive(Clone)]
struct Token { text: String, quoted: bool, span: Span }
struct P { tokens: Vec<Token>, i: usize }
type Result<T> = std::result::Result<T, Error>;

pub fn parse(text: &str) -> Result<Document> {
    if text.len() > crate::syntax::MAX_TEXT {
        return Err(Error { span: Span::default(), message: "drawing is too large".into() });
    }
    let mut p = P { tokens: lex(text)?, i: 0 };
    let mut doc = Document::default();
    let mut names = BTreeSet::new();
    while !p.done() {
        let span = p.span();
        match p.word()?.as_str() {
            "model" => {
                let name = p.name()?;
                if !names.insert(name.clone()) { return p.fail("model alias declared twice"); }
                p.want("from")?;
                doc.models.push(ModelImport { name, path: p.string()?, span });
            }
            "use" => doc.imports.push(p.string()?),
            "style" => doc.styles.push(p.rule(span)?),
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

impl P {
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
            styles: Vec::new(), labels: Vec::new(), span };
        let mut annotations = Vec::new();
        let mut measurements = Vec::new();
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
                kind @ ("view" | "section" | "sketch") => {
                    let name = self.name()?;
                    if s.views.iter().any(|v| v.name == name) { return self.fail("view name declared twice") }
                    self.want("(")?; let target = self.word()?; self.want(")")?;
                    let mut v = View { name, target, direction: "front".into(), cut: None,
                        sketch: kind == "sketch", at: (0.0, 0.0), scale: None,
                        dimensions: false, annotations: Vec::new(), measurements: Vec::new(), span };
                    if self.eat("from") { v.direction = self.word()?; }
                    if kind == "section" { self.want("cut")?; v.cut = Some(self.word()?); }
                    self.want("at")?; v.at = self.pair(true)?;
                    if self.eat("scale") { v.scale = Some(self.positive(false)?); }
                    if v.sketch && v.direction != "front" { return self.fail("a sketch shows its own 2D coordinates; use a solid view for projection") }
                    s.views.push(v);
                }
                "dimensions" => {
                    self.want("in")?;
                    annotations.push((self.name()?, None));
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
                "style" => s.styles.push(self.rule(span)?),
                _ => return self.fail("expected size, scale, view, section, sketch, dimensions, dimension, measure, label, or style"),
            }
            self.eat(";");
        }
        for (name, a) in annotations {
            let Some(v) = s.views.iter_mut().find(|v| v.name == name) else {
                return self.fail(&format!("no view named `{name}`"));
            };
            if let Some(a) = a { v.annotations.push(a); } else { v.dimensions = true; }
        }
        for (name, m) in measurements {
            let Some(v) = s.views.iter_mut().find(|v| v.name == name) else {
                return self.fail(&format!("no view named `{name}`"));
            };
            v.measurements.push(m);
        }
        Ok(s)
    }
    fn rule(&mut self, span: Span) -> Result<Rule> {
        let selector = self.word()?;
        self.want("{")?;
        let mut style = Style::default();
        while !self.eat("}") {
            let prop = self.word()?; self.want(":")?;
            let mut vals = Vec::new();
            let text;
            match prop.as_str() {
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
        Ok(Rule { selector, style, span })
    }
}

fn lex(s: &str) -> Result<Vec<Token>> {
    let mut out = Vec::new(); let mut i = 0;
    while i < s.len() {
        let c = s[i..].chars().next().unwrap();
        if c.is_whitespace() { i += c.len_utf8(); continue }
        if s[i..].starts_with("//") {
            i += s[i..].find('\n').unwrap_or(s.len() - i); continue;
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
                        _ => return Err(Error { span: Span::new(lo, i), message: "unsupported string escape".into() }) }
                } else { value.push(c); }
            }
            if !closed { return Err(Error { span: Span::new(lo, i), message: "unterminated string".into() }) }
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
            return Err(Error { span: Span::new(lo, i), message: "too many drawing tokens".into() });
        }
    }
    Ok(out)
}
