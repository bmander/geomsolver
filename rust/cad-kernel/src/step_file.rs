//! The text of a STEP file (ISO 10303-21, the "Part 21" exchange structure) read into its
//! entities: the syntax alone, shared by what verifies a file the kernel wrote (`step_check`)
//! and what reads a file into a solid (`step_read`). A run of the data section is parsed on each
//! core; every entity must parse, every instance number be defined once and every `#` reference
//! resolve. Nothing here knows what an entity means.
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
    /// Every reference in it, depth first.
    pub fn references(&self,out: &mut impl FnMut(u32)) {
        match self { Value::Ref(r) => out(*r),Value::List(l) | Value::Typed(_,l) => for v in l { v.references(out) },_ => {} }
    }
}

/// An entity: its instance number and its record, one `(name, parameters)` part for a simple
/// entity and several for a complex one.
#[derive(Debug)]
pub struct Entity<'a> { pub id: u32,pub parts: Vec<(&'a str,Vec<Value<'a>>)> }

impl<'a> Entity<'a> {
    pub fn simple(&self) -> Option<(&'a str,&[Value<'a>])> { (self.parts.len() == 1).then(|| (self.parts[0].0,&self.parts[0].1[..])) }
    pub fn part(&self,name: &str) -> Option<&[Value<'a>]> { self.parts.iter().find(|(n,_)| *n == name).map(|(_,v)| &v[..]) }
    pub fn is(&self,name: &str) -> bool { self.parts.iter().any(|(n,_)| *n == name) }
}

/// A byte reader over one run of the data section.
struct Lexer<'a> { text: &'a str,at: usize }

impl<'a> Lexer<'a> {
    fn bytes(&self) -> &'a [u8] { self.text.as_bytes() }
    fn fail<T>(&self,what: &str) -> Result<T,String> {
        let end = (self.at+40).min(self.text.len());
        let mut from = self.at.min(end);
        while !self.text.is_char_boundary(from) { from -= 1; }
        let mut to = end;
        while !self.text.is_char_boundary(to) { to -= 1; }
        Err(format!("{what} at `{}`",&self.text[from..to]))
    }
    fn skip(&mut self) -> Result<(),String> {
        let b = self.bytes();
        loop {
            while self.at < b.len() && b[self.at].is_ascii_whitespace() { self.at += 1; }
            if b[self.at..].starts_with(b"/*") {
                match self.text[self.at+2..].find("*/") { Some(k) => self.at += k+4,None => return self.fail("an unclosed comment") }
            } else { return Ok(()) }
        }
    }
    fn peek(&mut self) -> Result<Option<u8>,String> { self.skip()?; Ok(self.bytes().get(self.at).copied()) }
    fn expect(&mut self,c: u8) -> Result<(),String> {
        if self.peek()? == Some(c) { self.at += 1; Ok(()) } else { self.fail(&format!("`{}` expected",c as char)) }
    }
    fn keyword(&mut self) -> Result<&'a str,String> {
        self.skip()?;
        let b = self.bytes();
        let start = self.at;
        if !(self.at < b.len() && (b[self.at].is_ascii_uppercase() || b[self.at] == b'_')) { return self.fail("a keyword expected") }
        while self.at < b.len() && (b[self.at].is_ascii_uppercase() || b[self.at].is_ascii_digit() || b[self.at] == b'_' || b[self.at] == b'-') { self.at += 1; }
        Ok(&self.text[start..self.at])
    }
    fn integer(&mut self) -> Result<u32,String> {
        let b = self.bytes();
        let start = self.at;
        while self.at < b.len() && b[self.at].is_ascii_digit() { self.at += 1; }
        self.text[start..self.at].parse().or_else(|_| { self.at = start; self.fail("an instance number expected") })
    }
    /// `( value, … )`, the parenthesis already met.
    fn values(&mut self) -> Result<Vec<Value<'a>>,String> {
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
    fn value(&mut self) -> Result<Value<'a>,String> {
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
                    match b.get(k) {
                        None => return self.fail("an unclosed string"),
                        Some(b'\'') if b.get(k+1) == Some(&b'\'') => k += 2,
                        Some(b'\'') => break,
                        _ => k += 1,
                    }
                }
                self.at = k+1;
                Ok(Value::Str(&self.text[start..k]))
            }
            Some(b'"') => {
                let start = self.at+1;
                let Some(k) = self.text[start..].find('"') else { return self.fail("an unclosed binary") };
                self.at = start+k+1;
                Ok(Value::Str(&self.text[start..start+k]))
            }
            Some(b'.') => {
                let start = self.at+1;
                let mut k = start;
                while k < b.len() && (b[k].is_ascii_uppercase() || b[k].is_ascii_digit() || b[k] == b'_') { k += 1; }
                if k == start || b.get(k) != Some(&b'.') { return self.fail("an enumeration expected") }
                self.at = k+1;
                Ok(Value::Enum(&self.text[start..k]))
            }
            Some(c) if c.is_ascii_digit() || c == b'-' || c == b'+' => {
                let start = self.at;
                let mut k = start+1;
                let mut real = false;
                while k < b.len() {
                    match b[k] {
                        b'0'..=b'9' => {}
                        b'.' | b'E' | b'e' => real = true,
                        b'+' | b'-' if matches!(b[k-1],b'E' | b'e') => {}
                        _ => break,
                    }
                    k += 1;
                }
                let text = &self.text[start..k];
                self.at = k;
                if real { text.parse().map(Value::Real).or_else(|_| self.fail("a real expected")) }
                else { text.parse().map(Value::Int).or_else(|_| self.fail("an integer expected")) }
            }
            Some(c) if c.is_ascii_uppercase() || c == b'_' => {
                let name = self.keyword()?;
                self.expect(b'(')?;
                Ok(Value::Typed(name,self.values()?))
            }
            _ => self.fail("a parameter expected"),
        }
    }
    /// `#n = RECORD ;` or `#n = ( PART(…) PART(…) … ) ;`, or None at the end of the run.
    fn entity(&mut self) -> Result<Option<Entity<'a>>,String> {
        match self.peek()? { None => return Ok(None),Some(b'#') => self.at += 1,_ => return self.fail("an entity expected") }
        let id = self.integer()?;
        self.expect(b'=')?;
        let mut parts = Vec::new();
        if self.peek()? == Some(b'(') {
            self.at += 1;
            while self.peek()? != Some(b')') {
                let name = self.keyword()?;
                self.expect(b'(')?;
                parts.push((name,self.values()?));
            }
            self.at += 1;
            if parts.is_empty() { return self.fail("an empty complex entity") }
        } else {
            let name = self.keyword()?;
            self.expect(b'(')?;
            parts.push((name,self.values()?));
        }
        self.expect(b';')?;
        Ok(Some(Entity {id,parts}))
    }
}

/// The data section's entities, a run of them parsed on each core, in the file's order.
pub fn entities(text: &str) -> Result<Vec<Entity<'_>>,String> {
    if !text.starts_with("ISO-10303-21;") { return Err("the file does not start `ISO-10303-21;`".into()) }
    let Some(data) = text.find("\nDATA;") else { return Err("the file has no DATA section".into()) };
    let from = data+"\nDATA;".len();
    let Some(end) = text[from..].find("\nENDSEC;") else { return Err("the DATA section has no ENDSEC".into()) };
    let (section,to) = (&text[from..from+end],from+end);
    if !text[to+"\nENDSEC;".len()..].trim_start().starts_with("END-ISO-10303-21;") {
        return Err("the file does not end `END-ISO-10303-21;`".into());
    }
    // runs split where an entity starts a line after one that ended
    let runs = (crate::util::par::threads()*4).max(1);
    let mut cuts = vec![0];
    for r in 1..runs {
        let mut at = section.len()*r/runs;
        while !section.is_char_boundary(at) { at += 1; }
        if at <= *cuts.last().unwrap() { continue }
        match section[at..].find(";\n#") { Some(k) => { let cut = at+k+2; if cut > *cuts.last().unwrap() { cuts.push(cut) } } None => break }
    }
    cuts.push(section.len());
    cuts.dedup();
    let parsed = crate::util::par::indices(cuts.len()-1,|r| -> Result<Vec<Entity>,String> {
        let mut lexer = Lexer {text:&section[cuts[r]..cuts[r+1]],at:0};
        let mut out: Vec<Entity> = Vec::new();
        loop {
            match lexer.entity() {
                Ok(Some(e)) => out.push(e),
                Ok(None) => return Ok(out),
                Err(e) => return Err(match out.last() { Some(last) => format!("the entity after #{} does not parse: {e}",last.id),
                    None => format!("an entity does not parse: {e}") }),
            }
        }
    });
    let mut all = Vec::new();
    for run in parsed { all.extend(run?); }
    Ok(all)
}

/// A file's entities, every instance number defined once and every reference resolved.
pub struct File<'a> { pub entities: Vec<Entity<'a>>,index: Vec<u32> }

impl<'a> File<'a> {
    /// Parse `text`; `lap` is told each phase as it ends (for a caller timing them).
    pub fn parse(text: &'a str,lap: &dyn Fn(&str)) -> Result<File<'a>,String> {
        let entities = entities(text)?;
        lap("parsed");
        // every instance number once, and every reference to one
        let most = entities.iter().map(|e| e.id).max().unwrap_or(0) as usize;
        if most > 4*entities.len()+16 { return Err(format!("instance numbers up to #{most} for {} entities",entities.len())) }
        let mut index = vec![u32::MAX;most+1];
        for (k,e) in entities.iter().enumerate() {
            if index[e.id as usize] != u32::MAX { return Err(format!("#{} is defined twice",e.id)) }
            index[e.id as usize] = k as u32;
        }
        let dangling = crate::util::par::indices(entities.len().div_ceil(4096),|chunk| {
            let mut bad = None;
            for e in &entities[chunk*4096..((chunk+1)*4096).min(entities.len())] {
                for (_,values) in &e.parts { for v in values { v.references(&mut |r| {
                    if bad.is_none() && index.get(r as usize).is_none_or(|&k| k == u32::MAX) { bad = Some((e.id,r)); }
                }) } }
            }
            bad
        });
        if let Some((from,to)) = dangling.into_iter().flatten().next() { return Err(format!("#{from} refers to #{to}, which the file does not define")) }
        lap("resolved");
        Ok(File {entities,index})
    }
    /// The entity `#r` (which the file defines: every reference in it was resolved).
    pub fn get(&self,r: u32) -> &Entity<'a> { &self.entities[self.index[r as usize] as usize] }
    /// The simple entity `r` refers to, of one of `kinds`: its kind and parameters.
    pub fn of(&self,r: Option<u32>,kinds: &[&str],what: &str) -> Result<(&'a str,&[Value<'a>]),String> {
        let r = r.ok_or_else(|| format!("{what}: a reference expected"))?;
        let e = self.get(r);
        match e.simple() { Some((name,values)) if kinds.contains(&name) => Ok((name,values)),
            _ => Err(format!("{what}: #{r} is {}, not {}",e.parts.iter().map(|p| p.0).collect::<Vec<_>>().join("+"),kinds.join(" or "))) }
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
