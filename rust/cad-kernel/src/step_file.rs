//! The text of a STEP file (ISO 10303-21, the "Part 21" exchange structure) read into its
//! entities: the syntax alone, shared by what verifies a file the kernel wrote (`step_check`)
//! and what reads a file into a solid (`step_read`). Nothing here knows what an entity means.
//!
//! **The syntax as writers write it.** A byte-order mark, white space and comments (`/* … */`,
//! anywhere a token may stand) before and between tokens; lines ended `\n`, `\r\n` or `\r`;
//! entities several to a line or one over many; strings holding anything but a lone `'`
//! (newlines, `;`, `#`, `/*`, `ENDSEC`), their escapes left in the text (`Value::Str` borrows it)
//! and decoded where a string is used as a name (`decode`); reals as the grammar writes them
//! (`1.`, `1.E5`, `-0.5E-3`) and, each having one meaning, without a digit before the point
//! (`-.5E-3`), without a point (`1E5`) or with a lowercase `e`; `$`, `*`, enumerations, typed
//! parameters (`LENGTH_MEASURE(1.E-07)`), user-defined keywords (`!NAME`) and binaries. The
//! HEADER section is parsed (`Header`). What the reader does not read is an error saying so
//! (`Error::unsupported`): a second DATA section, an ANCHOR, REFERENCE or SIGNATURE section
//! (edition 3), a SCOPE, a value instance (`@n`). Every error says where: the instance it is in
//! where there is one, and the line and byte. No input panics (the text is untrusted, and
//! WebAssembly aborts on a panic).
//!
//! **The data section is parsed on every core** and its answer is the one parse from the start
//! gives, however many cores there are: the section is cut into runs, each where a `;` is
//! followed (after white space) by a `#`, and each run parsed alone. A cut inside a string or a
//! comment cannot pass unseen: a run that begins where an entity begins parses to its own end
//! only if its end is where an entity ends, so the first run (which begins at the section's
//! start) vouches for the second cut, and so on. Where any run fails, the section is parsed again
//! from its start on one core, and that parse's answer (an error, or the entities where a cut
//! was wrong) is the answer. Every instance number must be defined once and every `#` reference
//! resolve.
use std::collections::BTreeMap;

/// A parameter of an entity, borrowing the text it was read from.
#[derive(Debug,Clone,PartialEq)]
pub enum Value<'a> { Unset,Derived,Int(i64),Real(f64),Str(&'a str),Enum(&'a str),Ref(u32),List(Vec<Value<'a>>),Typed(&'a str,Vec<Value<'a>>) }

impl<'a> Value<'a> {
    pub fn list(&self) -> Option<&[Value<'a>]> { if let Value::List(l) = self { Some(l) } else { None } }
    pub fn reference(&self) -> Option<u32> { if let Value::Ref(r) = self { Some(*r) } else { None } }
    pub fn number(&self) -> Option<f64> { match self { Value::Real(x) => Some(*x),Value::Int(i) => Some(*i as f64),_ => None } }
    pub fn int(&self) -> Option<i64> { if let Value::Int(i) = self { Some(*i) } else { None } }
    pub fn word(&self) -> Option<&'a str> { if let Value::Enum(e) = self { Some(e) } else { None } }
    pub fn text(&self) -> Option<&'a str> { if let Value::Str(s) = self { Some(s) } else { None } }
    pub fn logical(&self) -> Option<bool> { match self.word()? { "T" => Some(true),"F" => Some(false),_ => None } }
    /// A string's text decoded (`decode`): the string used as a name.
    pub fn name(&self) -> Option<String> { self.text().map(decode) }
    /// A measure's number: a number, or a typed parameter holding one (`LENGTH_MEASURE(25.4)`).
    pub fn measure(&self) -> Option<f64> {
        match self { Value::Typed(_,v) if v.len() == 1 => v[0].measure(),_ => self.number() }
    }
    /// Every reference in it, depth first.
    pub fn references(&self,out: &mut impl FnMut(u32)) {
        match self { Value::Ref(r) => out(*r),Value::List(l) | Value::Typed(_,l) => for v in l { v.references(out) },_ => {} }
    }
}

/// A string's text as it is written in the file, decoded: `''` is `'` and `\\` is `\`; `\S\c`
/// the character of `c` in the upper half of ISO 8859-1 (another part, chosen by `\PA\` …
/// `\PI\`, is no table the kernel keeps: such a character is U+FFFD); `\X\hh` the character
/// U+00hh, `\X2\…\X0\` the UTF-16 and `\X4\…\X0\` the UCS-4 code units between; `\N\` and `\T\`
/// (edition 1's print control) a newline and a tab. A line break in the text is no part of the
/// string (Part 21 ignores print control), and a backslash that starts none of these stands for
/// itself. What cannot be decoded is U+FFFD; the decoding never fails.
pub fn decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut page = b'A';
    let hex = |h: &[u8]| -> Option<u32> { std::str::from_utf8(h).ok().filter(|t| t.bytes().all(|c| c.is_ascii_hexdigit())).and_then(|t| u32::from_str_radix(t,16).ok()) };
    let mut i = 0;
    while i < b.len() {
        let rest = &b[i..];
        if rest.starts_with(b"''") { out.push('\''); i += 2; continue }
        if rest[0] == b'\n' || rest[0] == b'\r' { i += 1; continue }
        if rest[0] == b'\\' {
            if rest.starts_with(b"\\\\") { out.push('\\'); i += 2; continue }
            if rest.starts_with(b"\\N\\") { out.push('\n'); i += 3; continue }
            if rest.starts_with(b"\\T\\") { out.push('\t'); i += 3; continue }
            if rest.len() >= 4 && rest.starts_with(b"\\S\\") {
                let c = rest[3];
                if c.is_ascii() && page == b'A' { out.push(char::from_u32(c as u32+128).unwrap_or('\u{fffd}')); }
                else { out.push('\u{fffd}'); }
                i += 3+s[i+3..].chars().next().map_or(1,char::len_utf8);
                continue
            }
            if rest.len() >= 4 && rest[1] == b'P' && (b'A'..=b'I').contains(&rest[2]) && rest[3] == b'\\' { page = rest[2]; i += 4; continue }
            if rest.starts_with(b"\\X2\\") || rest.starts_with(b"\\X4\\") {
                let width = if rest[2] == b'2' { 4 } else { 8 };
                let body = &rest[4..];
                let end = body.windows(4).position(|w| w == b"\\X0\\").unwrap_or(body.len());
                let units: Vec<u32> = body[..end].chunks(width).map(|h| if h.len() == width { hex(h).unwrap_or(0xfffd) } else { 0xfffd }).collect();
                if width == 4 {
                    out.extend(char::decode_utf16(units.iter().map(|&u| u as u16)).map(|c| c.unwrap_or('\u{fffd}')));
                } else {
                    out.extend(units.iter().map(|&u| char::from_u32(u).unwrap_or('\u{fffd}')));
                }
                i += 4+(end+4).min(body.len());
                continue
            }
            if rest.len() >= 5 && rest.starts_with(b"\\X\\") {
                if let Some(c) = hex(&rest[3..5]).and_then(char::from_u32) { out.push(c); i += 5; continue }
            }
            out.push('\\');
            i += 1;
            continue
        }
        let c = s[i..].chars().next().unwrap_or('\u{fffd}');
        out.push(c);
        i += c.len_utf8();
    }
    out
}

/// An entity: its instance number and its record, one `(name, parameters)` part for a simple
/// entity and several for a complex one.
#[derive(Debug,PartialEq)]
pub struct Entity<'a> { pub id: u32,pub parts: Vec<(&'a str,Vec<Value<'a>>)> }

impl<'a> Entity<'a> {
    pub fn simple(&self) -> Option<(&'a str,&[Value<'a>])> { (self.parts.len() == 1).then(|| (self.parts[0].0,&self.parts[0].1[..])) }
    pub fn part(&self,name: &str) -> Option<&[Value<'a>]> { self.parts.iter().find(|(n,_)| *n == name).map(|(_,v)| &v[..]) }
    pub fn is(&self,name: &str) -> bool { self.parts.iter().any(|(n,_)| *n == name) }
    /// Its kinds joined `+`, as a message names it (`LENGTH_UNIT+NAMED_UNIT+SI_UNIT`).
    pub fn kinds(&self) -> String { self.parts.iter().map(|p| p.0).collect::<Vec<_>>().join("+") }
}

/// Why a text is not read, and where: the instance it is in where there is one, and the line
/// (from 1) and the byte offset where the parse stopped where there is a place.
#[derive(Clone,Debug,PartialEq)]
pub struct Error {
    pub entity: Option<u32>,
    pub line: Option<usize>,
    pub offset: Option<usize>,
    /// The text uses what the reader does not read (a SCOPE, a second DATA section): it may be a
    /// good file. Otherwise the text is no exchange structure.
    pub unsupported: bool,
    pub msg: String,
}

impl Error {
    fn at(offset: usize,msg: String) -> Error { Error {entity:None,line:None,offset:Some(offset),unsupported:false,msg} }
    fn of(entity: u32,msg: String) -> Error { Error {entity:Some(entity),line:None,offset:None,unsupported:false,msg} }
    fn unsupported(mut self) -> Error { self.unsupported = true; self }
    /// The line the offset is on, counted in `text` (a `\r\n`, a `\n` and a lone `\r` each end one).
    fn located(mut self,text: &str) -> Error {
        if let Some(at) = self.offset {
            let b = &text.as_bytes()[..at.min(text.len())];
            let n = b.iter().enumerate().filter(|&(k,&c)| c == b'\n' || (c == b'\r' && b.get(k+1) != Some(&b'\n'))).count();
            self.line = Some(n+1);
        }
        self
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let mut at = Vec::new();
        if let Some(e) = self.entity { at.push(format!("#{e}")); }
        if let Some(l) = self.line { at.push(format!("line {l}")); }
        if let Some(o) = self.offset { at.push(format!("byte {o}")); }
        if at.is_empty() { f.write_str(&self.msg) } else { write!(f,"{}: {}",at.join(", "),self.msg) }
    }
}

impl std::error::Error for Error {}

type Parsed<T> = Result<T,Error>;

/// A byte reader over the text up to `end`; `depth` the lists open where it stands.
struct Lexer<'a> { text: &'a str,at: usize,end: usize,depth: usize }

/// How deep lists may nest: a B-spline surface's weights are a list of lists in a typed parameter
/// in a complex entity, four deep. Deeper than this is no exchange structure a writer means, and a
/// parser that followed it would run off its stack (which no `catch_unwind` catches) and so would
/// the value's drop.
const DEPTH: usize = 64;

fn keyword_byte(c: u8) -> bool { c.is_ascii_uppercase() || c.is_ascii_digit() || c == b'_' }

impl<'a> Lexer<'a> {
    fn bytes(&self) -> &'a [u8] { &self.text.as_bytes()[..self.end] }
    fn fail<T>(&self,what: &str) -> Parsed<T> { Err(self.error(what)) }
    fn error(&self,what: &str) -> Error {
        let end = (self.at+40).min(self.end);
        let mut from = self.at.min(end);
        while !self.text.is_char_boundary(from) { from -= 1; }
        let mut to = end;
        while !self.text.is_char_boundary(to) { to -= 1; }
        let shown: String = self.text[from..to].chars().map(|c| if c.is_control() { ' ' } else { c }).collect();
        if from == to { Error::at(self.at,format!("{what} at the end")) } else { Error::at(self.at,format!("{what} at `{shown}`")) }
    }
    fn skip(&mut self) -> Parsed<()> {
        let b = self.bytes();
        loop {
            while self.at < b.len() && b[self.at].is_ascii_whitespace() { self.at += 1; }
            if b[self.at..].starts_with(b"/*") {
                match b[self.at+2..].windows(2).position(|w| w == b"*/") { Some(k) => self.at += k+4,None => return self.fail("an unclosed comment") }
            } else { return Ok(()) }
        }
    }
    fn peek(&mut self) -> Parsed<Option<u8>> { self.skip()?; Ok(self.bytes().get(self.at).copied()) }
    fn expect(&mut self,c: u8) -> Parsed<()> {
        if self.peek()? == Some(c) { self.at += 1; Ok(()) } else { self.fail(&format!("`{}` expected",c as char)) }
    }
    /// Whether the next token is `word` (a keyword or the file's own `ISO-10303-21`), taken if so.
    fn literal(&mut self,word: &str) -> Parsed<bool> {
        self.skip()?;
        let b = self.bytes();
        let w = word.as_bytes();
        if b[self.at..].starts_with(w) && !b.get(self.at+w.len()).is_some_and(|&c| keyword_byte(c) || c == b'-') { self.at += w.len(); return Ok(true) }
        Ok(false)
    }
    /// A standard keyword (`UPPER {UPPER | DIGIT}`, `_` an upper) or a user-defined one (`!` before).
    fn keyword(&mut self) -> Parsed<&'a str> {
        self.skip()?;
        let b = self.bytes();
        let start = self.at;
        let mut k = start+(b.get(start) == Some(&b'!')) as usize;
        if !(k < b.len() && (b[k].is_ascii_uppercase() || b[k] == b'_')) { return self.fail("a keyword expected") }
        while k < b.len() && keyword_byte(b[k]) { k += 1; }
        self.at = k;
        Ok(&self.text[start..k])
    }
    fn integer(&mut self) -> Parsed<u32> {
        let b = self.bytes();
        let start = self.at;
        while self.at < b.len() && b[self.at].is_ascii_digit() { self.at += 1; }
        if self.at == start { return self.fail("an instance number expected") }
        self.text[start..self.at].parse().or_else(|_| { self.at = start; self.fail("an instance number beyond 4294967295") })
    }
    /// `( value, … )`, the parenthesis already met; no deeper than `DEPTH`.
    fn values(&mut self) -> Parsed<Vec<Value<'a>>> {
        if self.depth >= DEPTH { return self.fail(&format!("lists nested deeper than {DEPTH}")) }
        self.depth += 1;
        let got = self.items();
        self.depth -= 1;
        got
    }
    fn items(&mut self) -> Parsed<Vec<Value<'a>>> {
        let mut out = Vec::new();
        if self.peek()? == Some(b')') { self.at += 1; return Ok(out) }
        loop {
            out.push(self.value()?);
            match self.peek()? {
                Some(b',') => self.at += 1,
                Some(b')') => { self.at += 1; return Ok(out) }
                _ => return self.fail("`,` or `)` expected"),
            }
        }
    }
    /// A number: `[sign] digits [. digits] [E [sign] digits]`, a real where it has a point or
    /// an exponent (a digit before the point and an uppercase `E`, as the grammar has them, not
    /// asked).
    fn number(&mut self) -> Parsed<Value<'a>> {
        let b = self.bytes();
        let start = self.at;
        let mut k = start;
        let digits = |k: &mut usize| { let s = *k; while *k < b.len() && b[*k].is_ascii_digit() { *k += 1; } *k-s };
        if matches!(b.get(k),Some(b'+' | b'-')) { k += 1; }
        let mut n = digits(&mut k);
        let mut real = false;
        if b.get(k) == Some(&b'.') { k += 1; real = true; n += digits(&mut k); }
        if n == 0 { return self.fail("a number expected") }
        if matches!(b.get(k),Some(b'E' | b'e')) {
            k += 1;
            if matches!(b.get(k),Some(b'+' | b'-')) { k += 1; }
            if digits(&mut k) == 0 { self.at = start; return self.fail("an exponent's digits expected") }
            real = true;
        }
        let text = &self.text[start..k];
        if real { self.at = k; text.parse().map(Value::Real).or_else(|_| self.fail("a real expected")) }
        else { match text.parse() { Ok(i) => { self.at = k; Ok(Value::Int(i)) } Err(_) => self.fail("an integer beyond 64 bits") } }
    }
    fn value(&mut self) -> Parsed<Value<'a>> {
        let b = self.bytes();
        match self.peek()? {
            Some(b'$') => { self.at += 1; Ok(Value::Unset) }
            Some(b'*') => { self.at += 1; Ok(Value::Derived) }
            Some(b'#') => { self.at += 1; Ok(Value::Ref(self.integer()?)) }
            Some(b'(') => { self.at += 1; Ok(Value::List(self.values()?)) }
            Some(b'\'') => {
                let start = self.at+1;
                let mut k = start;
                loop {
                    match b[k..].iter().position(|&c| c == b'\'') {
                        None => return self.fail("an unclosed string"),
                        Some(q) if b.get(k+q+1) == Some(&b'\'') => k += q+2,
                        Some(q) => { k += q; break }
                    }
                }
                self.at = k+1;
                Ok(Value::Str(&self.text[start..k]))
            }
            Some(b'"') => {
                let start = self.at+1;
                let Some(k) = b[start..].iter().position(|&c| c == b'"') else { return self.fail("an unclosed binary") };
                if k == 0 || !b[start..start+k].iter().all(u8::is_ascii_hexdigit) || b[start] > b'3' { return self.fail("a binary expected") }
                self.at = start+k+1;
                Ok(Value::Str(&self.text[start..start+k]))
            }
            Some(b'.') if b.get(self.at+1).is_some_and(|c| c.is_ascii_uppercase() || *c == b'_') => {
                let start = self.at+1;
                let mut k = start;
                while k < b.len() && keyword_byte(b[k]) { k += 1; }
                if b.get(k) != Some(&b'.') { return self.fail("an enumeration expected") }
                self.at = k+1;
                Ok(Value::Enum(&self.text[start..k]))
            }
            Some(c) if c.is_ascii_digit() || matches!(c,b'-' | b'+' | b'.') => self.number(),
            Some(c) if c.is_ascii_uppercase() || c == b'_' || c == b'!' => {
                let name = self.keyword()?;
                self.expect(b'(')?;
                Ok(Value::Typed(name,self.values()?))
            }
            Some(b'@') => Err(self.error("a value instance, which the reader does not read").unsupported()),
            _ => self.fail("a parameter expected"),
        }
    }
    /// `KEYWORD ( … )`.
    fn record(&mut self) -> Parsed<(&'a str,Vec<Value<'a>>)> {
        let name = self.keyword()?;
        self.expect(b'(')?;
        Ok((name,self.values()?))
    }
    /// `#n = RECORD ;` or `#n = ( RECORD RECORD … ) ;`, the `#` next.
    fn entity(&mut self) -> Parsed<Entity<'a>> {
        self.at += 1;
        let id = self.integer()?;
        self.instance(id).map_err(|mut e| { e.entity = Some(id); e })
    }
    fn instance(&mut self,id: u32) -> Parsed<Entity<'a>> {
        self.expect(b'=')?;
        let mut parts = Vec::new();
        match self.peek()? {
            Some(b'(') => {
                self.at += 1;
                while self.peek()? != Some(b')') { parts.push(self.record()?); }
                self.at += 1;
                if parts.is_empty() { return self.fail("an empty complex entity") }
            }
            Some(b'&') => return Err(self.error("a SCOPE, which the reader does not read").unsupported()),
            _ => parts.push(self.record()?),
        }
        self.expect(b';')?;
        Ok(Entity {id,parts})
    }
    /// Entities to the end of the lexer's text.
    fn run(&mut self) -> Parsed<Vec<Entity<'a>>> {
        let mut out: Vec<Entity> = Vec::new();
        loop {
            match self.peek()? {
                None => return Ok(out),
                Some(b'#') => out.push(self.entity()?),
                _ => return Err(after(self.error("an entity expected"),&out)),
            }
        }
    }
}

/// An error between entities, said to be after the last one read.
fn after(mut e: Error,read: &[Entity]) -> Error {
    if let Some(last) = read.last() { e.msg = format!("{} (after #{})",e.msg,last.id); }
    e
}

/// The HEADER section: its entities as written, and what the three the standard asks for say,
/// decoded. A field the file does not give is empty.
#[derive(Clone,Debug,Default,PartialEq)]
pub struct Header<'a> {
    /// Every header entity, in the file's order.
    pub records: Vec<(&'a str,Vec<Value<'a>>)>,
    /// `FILE_DESCRIPTION`'s description and implementation level (`2;1`).
    pub description: Vec<String>,
    pub level: String,
    /// `FILE_NAME`'s fields.
    pub name: String,
    pub time_stamp: String,
    pub author: Vec<String>,
    pub organization: Vec<String>,
    pub preprocessor: String,
    pub originating_system: String,
    pub authorization: String,
    /// `FILE_SCHEMA`'s schemas, as written (`AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }`).
    pub schemas: Vec<String>,
}

impl<'a> Header<'a> {
    fn of(records: Vec<(&'a str,Vec<Value<'a>>)>) -> Header<'a> {
        let mut h = Header::default();
        let text = |v: &[Value],k: usize| v.get(k).and_then(Value::name).unwrap_or_default();
        let texts = |v: &[Value],k: usize| v.get(k).and_then(Value::list).map(|l| l.iter().filter_map(Value::name).collect()).unwrap_or_default();
        for (name,v) in &records {
            match *name {
                "FILE_DESCRIPTION" => { h.description = texts(v,0); h.level = text(v,1); }
                "FILE_NAME" => {
                    (h.name,h.time_stamp,h.author,h.organization) = (text(v,0),text(v,1),texts(v,2),texts(v,3));
                    (h.preprocessor,h.originating_system,h.authorization) = (text(v,4),text(v,5),text(v,6));
                }
                "FILE_SCHEMA" => h.schemas = texts(v,0),
                _ => {}
            }
        }
        h.records = records;
        h
    }
    /// The schemas' names, without their object identifiers and upper-cased
    /// (`AUTOMOTIVE_DESIGN`).
    pub fn schema_names(&self) -> Vec<String> {
        self.schemas.iter().map(|s| s.split(|c: char| c == '{' || c.is_whitespace()).find(|w| !w.is_empty()).unwrap_or("").to_ascii_uppercase()).collect()
    }
}

/// The file up to its data section's first entity: `ISO-10303-21;`, the header, and `DATA;` (or
/// `DATA(…);`); the lexer left after it.
fn prologue<'a>(lx: &mut Lexer<'a>) -> Parsed<Header<'a>> {
    if !lx.literal("ISO-10303-21")? { return lx.fail("the file does not start `ISO-10303-21;`") }
    lx.expect(b';')?;
    if !lx.literal("HEADER")? { return lx.fail("`HEADER;` expected") }
    lx.expect(b';')?;
    let mut records = Vec::new();
    while !lx.literal("ENDSEC")? {
        let r = lx.record()?;
        lx.expect(b';')?;
        records.push(r);
    }
    lx.expect(b';')?;
    let at = lx.at;
    if !lx.literal("DATA")? {
        if lx.literal("END-ISO-10303-21")? { lx.at = at; return lx.fail("the file has no DATA section") }
        for s in ["ANCHOR","REFERENCE","SIGNATURE"] {
            if lx.literal(s)? { lx.at = at; return Err(lx.error(&format!("an {s} section, which the reader does not read")).unsupported()) }
        }
        return lx.fail("a section expected")
    }
    if lx.peek()? == Some(b'(') { lx.at += 1; lx.values()?; }
    lx.expect(b';')?;
    Ok(Header::of(records))
}

/// The data section from where the lexer stands, then the rest of the file, on one core.
fn sequential<'a>(lx: &mut Lexer<'a>) -> Parsed<Vec<Entity<'a>>> {
    let mut out: Vec<Entity> = Vec::new();
    loop {
        match lx.peek()? {
            Some(b'#') => out.push(lx.entity()?),
            Some(b'E') if lx.literal("ENDSEC")? => break,
            Some(b'E') if lx.bytes()[lx.at..].starts_with(b"END-ISO-10303-21") => return Err(after(lx.error("the DATA section has no ENDSEC"),&out)),
            None => return Err(after(lx.error("the DATA section has no ENDSEC"),&out)),
            _ => return Err(after(lx.error("an entity expected"),&out)),
        }
    }
    lx.expect(b';')?;
    let at = lx.at;
    if lx.literal("DATA")? { lx.at = at; return Err(lx.error("a second DATA section, which the reader does not read").unsupported()) }
    for s in ["ANCHOR","REFERENCE","SIGNATURE"] {
        if lx.literal(s)? { lx.at = at; return Err(lx.error(&format!("an {s} section, which the reader does not read")).unsupported()) }
    }
    if !lx.literal("END-ISO-10303-21")? { return lx.fail("the file does not end `END-ISO-10303-21;`") }
    lx.expect(b';')?;
    if lx.peek()?.is_some() { return lx.fail("text after `END-ISO-10303-21;`") }
    Ok(out)
}

/// Where the data section ends if the file ends as most do — `ENDSEC;`, `END-ISO-10303-21;`
/// and white space — the offset of that `ENDSEC`.
fn plain_end(text: &str) -> Option<usize> {
    let t = text.trim_end_matches(|c: char| c.is_ascii_whitespace()).strip_suffix("END-ISO-10303-21;")?;
    let t = t.trim_end_matches(|c: char| c.is_ascii_whitespace()).strip_suffix("ENDSEC;")?;
    Some(t.len())
}

/// The header and the data section's entities, the section parsed in `runs` runs (one core
/// each, as many at once as there are cores); the answer does not depend on `runs`.
fn sections(text: &str,runs: usize) -> Parsed<(Header<'_>,Vec<Entity<'_>>)> {
    let bom = if text.starts_with('\u{feff}') { 3 } else { 0 };
    let mut lx = Lexer {text,at:bom,end:text.len(),depth:0};
    let header = prologue(&mut lx)?;
    let from = lx.at;
    if let Some(to) = plain_end(text).filter(|&to| to >= from) {
        // the cuts: after a `;` that white space and a `#` follow
        let b = text.as_bytes();
        let mut cuts = vec![from];
        for r in 1..runs.max(1) {
            let mut at = (from+(to-from)*r/runs).max(*cuts.last().unwrap());
            while let Some(k) = b[at..to].iter().position(|&c| c == b';') {
                let mut n = at+k+1;
                while n < to && b[n].is_ascii_whitespace() { n += 1; }
                if n < to && b[n] == b'#' { cuts.push(at+k+1); break }
                at = n;
            }
        }
        cuts.push(to);
        cuts.dedup();
        let parsed = crate::util::par::indices(cuts.len()-1,|r| Lexer {text,at:cuts[r],end:cuts[r+1],depth:0}.run());
        if parsed.iter().all(Result::is_ok) {
            return Ok((header,parsed.into_iter().flat_map(Result::unwrap).collect()))
        }
    }
    let entities = sequential(&mut lx)?;
    Ok((header,entities))
}

/// Where an instance number's entity is in the file's list.
enum Index { Dense(Vec<u32>),Sparse(Vec<(u32,u32)>) }

/// A file's entities, every instance number defined once and every reference resolved, and its
/// header.
pub struct File<'a> { pub entities: Vec<Entity<'a>>,pub header: Header<'a>,index: Index }

impl<'a> File<'a> {
    /// Parse `text`; `lap` is told each phase as it ends (for a caller timing them).
    pub fn parse(text: &'a str,lap: &dyn Fn(&str)) -> Result<File<'a>,String> {
        File::read(text,crate::util::par::threads()*4,lap).map_err(|e| e.to_string())
    }
    /// Parse `text`, its data section cut into `runs` runs (the answer is the same for any), the
    /// error saying where.
    pub fn read(text: &'a str,runs: usize,lap: &dyn Fn(&str)) -> Result<File<'a>,Error> {
        let (header,entities) = sections(text,runs).map_err(|e| e.located(text))?;
        lap("parsed");
        // every instance number once, and every reference to one
        let most = entities.iter().map(|e| e.id).max().unwrap_or(0) as usize;
        let index = if most <= 4*entities.len()+16 {
            let mut index = vec![u32::MAX;most+1];
            for (k,e) in entities.iter().enumerate() {
                if index[e.id as usize] != u32::MAX { return Err(Error::of(e.id,format!("#{} is defined twice",e.id))) }
                index[e.id as usize] = k as u32;
            }
            Index::Dense(index)
        } else {
            let mut index: Vec<(u32,u32)> = entities.iter().enumerate().map(|(k,e)| (e.id,k as u32)).collect();
            index.sort_unstable();
            if let Some(w) = index.windows(2).find(|w| w[0].0 == w[1].0) { return Err(Error::of(w[0].0,format!("#{} is defined twice",w[0].0))) }
            Index::Sparse(index)
        };
        let file = File {entities,header,index};
        let dangling = crate::util::par::indices(file.entities.len().div_ceil(4096),|chunk| {
            let mut bad = None;
            for e in &file.entities[chunk*4096..((chunk+1)*4096).min(file.entities.len())] {
                for (_,values) in &e.parts { for v in values { v.references(&mut |r| {
                    if bad.is_none() && file.slot(r).is_none() { bad = Some((e.id,r)); }
                }) } }
            }
            bad
        });
        if let Some((from,to)) = dangling.into_iter().flatten().next() {
            return Err(Error::of(from,format!("#{from} refers to #{to}, which the file does not define")))
        }
        lap("resolved");
        Ok(file)
    }
    fn slot(&self,r: u32) -> Option<usize> {
        match &self.index {
            Index::Dense(v) => v.get(r as usize).filter(|&&k| k != u32::MAX).map(|&k| k as usize),
            Index::Sparse(v) => v.binary_search_by_key(&r,|p| p.0).ok().map(|k| v[k].1 as usize),
        }
    }
    /// The entity `#r` (which the file defines: every reference in it was resolved).
    pub fn get(&self,r: u32) -> &Entity<'a> { &self.entities[self.slot(r).expect("an instance the file defines")] }
    /// The entity `#r`, where the file defines it.
    pub fn find(&self,r: u32) -> Option<&Entity<'a>> { self.slot(r).map(|k| &self.entities[k]) }
    /// The simple entity `r` refers to, of one of `kinds`: its kind and parameters.
    pub fn of(&self,r: Option<u32>,kinds: &[&str],what: &str) -> Result<(&'a str,&[Value<'a>]),String> {
        let r = r.ok_or_else(|| format!("{what}: a reference expected"))?;
        let e = self.get(r);
        match e.simple() { Some((name,values)) if kinds.contains(&name) => Ok((name,values)),
            _ => Err(format!("{what}: #{r} is {}, not {}",e.kinds(),kinds.join(" or "))) }
    }
    /// How many entities are, or have a part that is, `name`.
    pub fn count(&self,name: &str) -> usize { self.entities.iter().filter(|e| e.is(name)).count() }
    /// The simple entities of `kind`, in the file's order.
    pub fn all(&self,kind: &str) -> Vec<&Entity<'a>> { self.entities.iter().filter(|e| e.simple().is_some_and(|(n,_)| n == kind)).collect() }
    /// The entities of `kind` by what their parameter `at` refers to.
    pub fn referring(&self,kind: &str,at: usize) -> BTreeMap<u32,Vec<u32>> {
        let mut by: BTreeMap<u32,Vec<u32>> = BTreeMap::new();
        for e in self.all(kind) {
            if let Some(r) = e.simple().and_then(|(_,v)| v.get(at)).and_then(Value::reference) { by.entry(r).or_default().push(e.id); }
        }
        by
    }
}
