//! A small JSON value, parser and writer.
//!
//! The core owns document I/O, so it owns JSON too — and the bindings speak JSON for everything
//! that is not on the hot path, which keeps them thin.  Objects preserve insertion order so a
//! document round-trips byte-for-byte, and integers stay integers so a saved file reads the same
//! in Python and in the browser.  No external crates: the WebAssembly build has no build step
//! beyond `cargo build`.

use std::fmt::Write as _;

#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Int(i64),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    pub fn obj() -> Json {
        Json::Obj(Vec::new())
    }

    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(kv) => kv.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    pub fn set(&mut self, key: &str, v: Json) {
        if let Json::Obj(kv) = self {
            match kv.iter_mut().find(|(k, _)| k == key) {
                Some(slot) => slot.1 = v,
                None => kv.push((key.to_string(), v)),
            }
        }
    }

    pub fn arr(&self) -> &[Json] {
        match self {
            Json::Arr(a) => a,
            _ => &[],
        }
    }

    pub fn as_f64(&self) -> f64 {
        match self {
            Json::Int(i) => *i as f64,
            Json::Num(n) => *n,
            Json::Bool(true) => 1.0,
            _ => 0.0,
        }
    }

    pub fn as_i64(&self) -> i64 {
        match self {
            Json::Int(i) => *i,
            Json::Num(n) => *n as i64,
            Json::Bool(b) => *b as i64,
            _ => 0,
        }
    }

    pub fn as_bool(&self) -> bool {
        match self {
            Json::Bool(b) => *b,
            Json::Int(i) => *i != 0,
            Json::Num(n) => *n != 0.0,
            _ => false,
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            Json::Str(s) => s,
            _ => "",
        }
    }

    pub fn num(v: f64) -> Json {
        Json::Num(v)
    }

    pub fn dump(&self, indent: Option<usize>) -> String {
        let mut out = String::new();
        write_json(&mut out, self, indent, 0);
        out
    }
}

impl From<f64> for Json {
    fn from(v: f64) -> Json {
        Json::Num(v)
    }
}
impl From<i64> for Json {
    fn from(v: i64) -> Json {
        Json::Int(v)
    }
}
impl From<usize> for Json {
    fn from(v: usize) -> Json {
        Json::Int(v as i64)
    }
}
impl From<u32> for Json {
    fn from(v: u32) -> Json {
        Json::Int(v as i64)
    }
}
impl From<i32> for Json {
    fn from(v: i32) -> Json {
        Json::Int(v as i64)
    }
}
impl From<bool> for Json {
    fn from(v: bool) -> Json {
        Json::Bool(v)
    }
}
impl From<&str> for Json {
    fn from(v: &str) -> Json {
        Json::Str(v.to_string())
    }
}
impl From<String> for Json {
    fn from(v: String) -> Json {
        Json::Str(v)
    }
}
impl<T: Into<Json>> From<Vec<T>> for Json {
    fn from(v: Vec<T>) -> Json {
        Json::Arr(v.into_iter().map(|x| x.into()).collect())
    }
}
impl<T: Into<Json>> From<Option<T>> for Json {
    fn from(v: Option<T>) -> Json {
        match v {
            Some(x) => x.into(),
            None => Json::Null,
        }
    }
}

/// Build an object from `(key, value)` pairs, in order.
pub fn object<const N: usize>(pairs: [(&str, Json); N]) -> Json {
    Json::Obj(pairs.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

/// A float exactly as Python's `json` writes it: shortest round-trip, but always with a decimal
/// point so the value stays a float across a save/load.
fn write_f64(out: &mut String, v: f64) {
    if !v.is_finite() {
        out.push_str(if v.is_nan() {
            "NaN"
        } else if v > 0.0 {
            "Infinity"
        } else {
            "-Infinity"
        });
        return;
    }
    let s = format!("{v}");
    out.push_str(&s);
    if !s.contains(['.', 'e', 'E']) {
        out.push_str(".0");
    }
}

fn write_str(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn write_json(out: &mut String, v: &Json, indent: Option<usize>, depth: usize) {
    let pad = |out: &mut String, d: usize| {
        if let Some(n) = indent {
            out.push('\n');
            for _ in 0..n * d {
                out.push(' ');
            }
        }
    };
    match v {
        Json::Null => out.push_str("null"),
        Json::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Json::Int(i) => {
            let _ = write!(out, "{i}");
        }
        Json::Num(n) => write_f64(out, *n),
        Json::Str(s) => write_str(out, s),
        Json::Arr(a) => {
            if a.is_empty() {
                out.push_str("[]");
                return;
            }
            out.push('[');
            for (i, x) in a.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                pad(out, depth + 1);
                write_json(out, x, indent, depth + 1);
            }
            pad(out, depth);
            out.push(']');
        }
        Json::Obj(kv) => {
            if kv.is_empty() {
                out.push_str("{}");
                return;
            }
            out.push('{');
            for (i, (k, x)) in kv.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                pad(out, depth + 1);
                write_str(out, k);
                out.push(':');
                if indent.is_some() {
                    out.push(' ');
                }
                write_json(out, x, indent, depth + 1);
            }
            pad(out, depth);
            out.push('}');
        }
    }
}

/* -- parser ---------------------------------------------------------------- */

pub fn parse(s: &str) -> Result<Json, String> {
    let b: Vec<char> = s.chars().collect();
    let mut p = Parser { b, i: 0 };
    p.ws();
    let v = p.value()?;
    p.ws();
    if p.i != p.b.len() {
        return Err(format!("trailing data at {}", p.i));
    }
    Ok(v)
}

struct Parser {
    b: Vec<char>,
    i: usize,
}

impl Parser {
    fn ws(&mut self) {
        while self.i < self.b.len() && self.b[self.i].is_ascii_whitespace() {
            self.i += 1;
        }
    }

    fn peek(&self) -> Option<char> {
        self.b.get(self.i).copied()
    }

    fn expect(&mut self, c: char) -> Result<(), String> {
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            Err(format!("expected {c:?} at {}", self.i))
        }
    }

    fn lit(&mut self, word: &str) -> bool {
        if self.b[self.i..].starts_with(&word.chars().collect::<Vec<_>>()[..]) {
            self.i += word.len();
            true
        } else {
            false
        }
    }

    fn value(&mut self) -> Result<Json, String> {
        self.ws();
        match self.peek() {
            None => Err("unexpected end of input".into()),
            Some('{') => self.object(),
            Some('[') => self.array(),
            Some('"') => Ok(Json::Str(self.string()?)),
            Some('t') if self.lit("true") => Ok(Json::Bool(true)),
            Some('f') if self.lit("false") => Ok(Json::Bool(false)),
            Some('n') if self.lit("null") => Ok(Json::Null),
            Some('N') if self.lit("NaN") => Ok(Json::Num(f64::NAN)),
            Some('I') if self.lit("Infinity") => Ok(Json::Num(f64::INFINITY)),
            Some(_) => self.number(),
        }
    }

    fn object(&mut self) -> Result<Json, String> {
        self.expect('{')?;
        let mut kv = Vec::new();
        self.ws();
        if self.peek() == Some('}') {
            self.i += 1;
            return Ok(Json::Obj(kv));
        }
        loop {
            self.ws();
            let k = self.string()?;
            self.ws();
            self.expect(':')?;
            let v = self.value()?;
            kv.push((k, v));
            self.ws();
            match self.peek() {
                Some(',') => self.i += 1,
                Some('}') => {
                    self.i += 1;
                    return Ok(Json::Obj(kv));
                }
                _ => return Err(format!("expected ',' or '}}' at {}", self.i)),
            }
        }
    }

    fn array(&mut self) -> Result<Json, String> {
        self.expect('[')?;
        let mut a = Vec::new();
        self.ws();
        if self.peek() == Some(']') {
            self.i += 1;
            return Ok(Json::Arr(a));
        }
        loop {
            a.push(self.value()?);
            self.ws();
            match self.peek() {
                Some(',') => self.i += 1,
                Some(']') => {
                    self.i += 1;
                    return Ok(Json::Arr(a));
                }
                _ => return Err(format!("expected ',' or ']' at {}", self.i)),
            }
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.expect('"')?;
        let mut out = String::new();
        loop {
            let Some(c) = self.peek() else { return Err("unterminated string".into()) };
            self.i += 1;
            match c {
                '"' => return Ok(out),
                '\\' => {
                    let Some(e) = self.peek() else { return Err("bad escape".into()) };
                    self.i += 1;
                    out.push(match e {
                        'n' => '\n',
                        't' => '\t',
                        'r' => '\r',
                        'b' => '\u{8}',
                        'f' => '\u{c}',
                        'u' => {
                            let hex: String = self.b[self.i..(self.i + 4).min(self.b.len())]
                                .iter()
                                .collect();
                            self.i += 4;
                            let cp = u32::from_str_radix(&hex, 16).map_err(|e| e.to_string())?;
                            char::from_u32(cp).unwrap_or('\u{fffd}')
                        }
                        other => other,
                    });
                }
                c => out.push(c),
            }
        }
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.i;
        if self.peek() == Some('-') {
            self.i += 1;
            if self.lit("Infinity") {
                return Ok(Json::Num(f64::NEG_INFINITY));
            }
        }
        let mut is_float = false;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() {
                self.i += 1;
            } else if c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-' {
                is_float = is_float || c == '.' || c == 'e' || c == 'E';
                self.i += 1;
            } else {
                break;
            }
        }
        let s: String = self.b[start..self.i].iter().collect();
        if s.is_empty() {
            return Err(format!("expected a number at {start}"));
        }
        if !is_float {
            if let Ok(i) = s.parse::<i64>() {
                return Ok(Json::Int(i));
            }
        }
        s.parse::<f64>().map(Json::Num).map_err(|e| e.to_string())
    }
}

/// Python's `%g`-style formatting: `sig` significant digits, trailing zeros dropped.
pub fn fmt_g(v: f64, sig: usize) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    if v == 0.0 {
        return "0".to_string();
    }
    let exp = v.abs().log10().floor() as i32;
    if exp < -4 || exp >= sig as i32 {
        let mantissa = format!("{:.*}", sig.saturating_sub(1), v / 10f64.powi(exp));
        let mantissa = trim_zeros(&mantissa);
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{mantissa}e{sign}{:02}", exp.abs())
    } else {
        let decimals = (sig as i32 - 1 - exp).max(0) as usize;
        trim_zeros(&format!("{v:.decimals$}"))
    }
}

fn trim_zeros(s: &str) -> String {
    if !s.contains('.') {
        return s.to_string();
    }
    let t = s.trim_end_matches('0');
    t.trim_end_matches('.').to_string()
}
