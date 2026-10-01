//! The STEP file an export writes, verified against the solid it was written from without the
//! kernel's reader (docs/native-speed-plan.md, phase 6). The solid is a valid closed solid already
//! (the construction checked it); what is left to know is that the file says it. So the file is
//! parsed here, a run of entities on each core: every entity must parse and every `#` reference
//! resolve; the solid's topology is walked from the shape representation down (solids, shells,
//! faces, loops, oriented edges, edges, vertices), every edge used exactly twice by the closed
//! shell's faces, each count the solid's (`solvent_cad_brep_summary`) and no entity of those kinds
//! left out of the walk; the units are millimetres and radians; and each face, in the closed
//! shell's order, is on a surface of the solid's face's kind — with an elementary surface's
//! placement, radii and angle, and a B-spline's degrees, knots, multiplicities, poles and weights,
//! within `NEAR` of the solid's. It is not a check that a reader would rebuild the same solid (that
//! is `--verify-step full`, the kernel's own read-back); it is a check that the file carries this
//! solid's topology and surfaces, entity for entity, as the writer was given them.
use std::collections::{BTreeMap,BTreeSet};

/// How a written STEP file is verified: its text against the solid (`verify`), or that and the
/// kernel's own read-back as well (`solvent_cad_step`'s `full`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub(crate) enum Verification { Light,Full }

static FULL: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Ask for the full verification (`--verify-step full`) of every STEP file this process writes.
#[allow(dead_code)]
pub(crate) fn ask_full() { FULL.store(true,std::sync::atomic::Ordering::Relaxed); }

/// How files are verified: in full where `ask_full` or `SOLVENT_STEP_VERIFY=full` asks, else light.
pub(crate) fn verification() -> Verification {
    if FULL.load(std::sync::atomic::Ordering::Relaxed) || std::env::var("SOLVENT_STEP_VERIFY").is_ok_and(|v| v == "full") {
        Verification::Full
    } else { Verification::Light }
}

/// How near a written number must be to the solid's: the writer prints thirteen or fourteen
/// significant digits, so a relative billionth (and a billionth of a millimetre about zero).
const NEAR: f64 = 1e-9;

/// A parameter of an entity, borrowing the text it was read from.
#[derive(Debug,Clone,PartialEq)]
enum Value<'a> { Unset,Derived,Int(i64),Real(f64),Str(&'a str),Enum(&'a str),Ref(u32),List(Vec<Value<'a>>),Typed(&'a str,Vec<Value<'a>>) }

impl<'a> Value<'a> {
    fn list(&self) -> Option<&[Value<'a>]> { if let Value::List(l) = self { Some(l) } else { None } }
    fn reference(&self) -> Option<u32> { if let Value::Ref(r) = self { Some(*r) } else { None } }
    fn number(&self) -> Option<f64> { match self { Value::Real(x) => Some(*x),Value::Int(i) => Some(*i as f64),_ => None } }
    fn int(&self) -> Option<i64> { if let Value::Int(i) = self { Some(*i) } else { None } }
    fn word(&self) -> Option<&'a str> { if let Value::Enum(e) = self { Some(e) } else { None } }
    fn logical(&self) -> Option<bool> { match self.word()? { "T" => Some(true),"F" => Some(false),_ => None } }
    /// Every reference in it, depth first.
    fn references(&self,out: &mut impl FnMut(u32)) {
        match self { Value::Ref(r) => out(*r),Value::List(l) | Value::Typed(_,l) => for v in l { v.references(out) },_ => {} }
    }
}

/// An entity: its instance number and its record, one `(name, parameters)` part for a simple
/// entity and several for a complex one.
#[derive(Debug)]
struct Entity<'a> { id: u32,parts: Vec<(&'a str,Vec<Value<'a>>)> }

impl<'a> Entity<'a> {
    fn simple(&self) -> Option<(&'a str,&[Value<'a>])> { (self.parts.len() == 1).then(|| (self.parts[0].0,&self.parts[0].1[..])) }
    fn part(&self,name: &str) -> Option<&[Value<'a>]> { self.parts.iter().find(|(n,_)| *n == name).map(|(_,v)| &v[..]) }
    fn is(&self,name: &str) -> bool { self.parts.iter().any(|(n,_)| *n == name) }
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
fn entities(text: &str) -> Result<Vec<Entity<'_>>,String> {
    if !text.starts_with("ISO-10303-21;") { return Err("the file does not start `ISO-10303-21;`".into()) }
    let Some(data) = text.find("\nDATA;") else { return Err("the file has no DATA section".into()) };
    let from = data+"\nDATA;".len();
    let Some(end) = text[from..].find("\nENDSEC;") else { return Err("the DATA section has no ENDSEC".into()) };
    let (section,to) = (&text[from..from+end],from+end);
    if !text[to+"\nENDSEC;".len()..].trim_start().starts_with("END-ISO-10303-21;") {
        return Err("the file does not end `END-ISO-10303-21;`".into());
    }
    // runs split where an entity starts a line after one that ended
    let runs = (gcs_core::par::threads()*4).max(1);
    let mut cuts = vec![0];
    for r in 1..runs {
        let mut at = section.len()*r/runs;
        while !section.is_char_boundary(at) { at += 1; }
        if at <= *cuts.last().unwrap() { continue }
        match section[at..].find(";\n#") { Some(k) => { let cut = at+k+2; if cut > *cuts.last().unwrap() { cuts.push(cut) } } None => break }
    }
    cuts.push(section.len());
    cuts.dedup();
    let parsed = gcs_core::par::indices(cuts.len()-1,|r| -> Result<Vec<Entity>,String> {
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

/// The solid a STEP file must describe (`solvent_cad_brep_summary`'s numbers read).
#[derive(Debug,Clone,PartialEq)]
pub(crate) struct Solid { pub solids: usize,pub shells: usize,pub edges: usize,pub vertices: usize,pub faces: Vec<Face> }

/// A face of it: its surface's kind (`GeomAbs_SurfaceType`'s number), whether the solid reverses
/// it, whether its surface's placement is left-handed (the writer writes it right-handed, the
/// face's sense turned), and the surface's numbers (`solvent_cad_brep_summary`).
#[derive(Debug,Clone,PartialEq)]
pub(crate) struct Face { pub kind: u8,pub reversed: bool,pub indirect: bool,pub numbers: Vec<f64> }

/// `GeomAbs_SurfaceType`'s numbers for the kinds compared.
const PLANE: u8 = 0;
const CYLINDER: u8 = 1;
const CONE: u8 = 2;
const SPHERE: u8 = 3;
const TORUS: u8 = 4;
const BSPLINE: u8 = 6;
const REVOLUTION: u8 = 7;
const EXTRUSION: u8 = 8;

impl Solid {
    /// The solid a STEP file of the core's own B-rep must describe (`gcs_core::brep::step`): its
    /// counts, and each face's surface as that writer writes it.
    pub(crate) fn of(b: &gcs_core::brep::topo::Brep) -> Solid {
        use gcs_core::brep::{geom::Surface,topo::EdgeCurve};
        let faces = b.faces.iter().map(|f| {
            // a loft's face is written as the B-spline fitted to it
            // (one the writer could not fit was refused there, and has no numbers here)
            if let Surface::Blend(_,bl) = &f.surface {
                let numbers = gcs_core::brep::step::blend_net(bl).map(|n| gcs_core::brep::step::net_numbers(&n)).unwrap_or_default();
                return Face {kind:BSPLINE,reversed:f.reversed,indirect:false,numbers}
            }
            // a sheet is written as its own net
            if let Surface::BSpline(_,net) = &f.surface {
                return Face {kind:BSPLINE,reversed:f.reversed,indirect:false,numbers:gcs_core::brep::step::net_numbers(net)}
            }
            let (frame,scalars) = gcs_core::brep::step::written(&f.surface);
            let kind = match f.surface { Surface::Plane(_) => PLANE,Surface::Cylinder(..) => CYLINDER,
                Surface::Cone(..) => CONE,Surface::Sphere(..) => SPHERE,Surface::Torus(..) => TORUS,
                Surface::Revolution(..) => REVOLUTION,Surface::Extrusion(..) => EXTRUSION,
                Surface::Blend(..) | Surface::BSpline(..) => BSPLINE };
            let mut numbers: Vec<f64> = frame.o.iter().chain(&frame.z).chain(&frame.x).copied().collect();
            numbers.extend(scalars);
            Face {kind,reversed:f.reversed,indirect:false,numbers}
        }).collect();
        // a pole's degenerate edge is not written, nor a vertex only it ends at
        let written: Vec<&gcs_core::brep::topo::Edge> = b.edges.iter().filter(|e| matches!(e.curve,EdgeCurve::Curve(_))).collect();
        let mut used = vec![false;b.vertices.len()];
        for e in &written { for v in e.v { used[v as usize] = true; } }
        Solid {solids:1,shells:1,edges:written.len(),vertices:used.iter().filter(|&&u| u).count(),faces}
    }
    /// Read the kernel's summary.
    pub(crate) fn read(summary: &[f64]) -> Result<Solid,String> {
        let bad = || "a malformed solid summary".to_string();
        if summary.len() < 6 || summary[0] != 1. { return Err(bad()) }
        let n = |x: f64| x as usize;
        let mut faces = Vec::new();
        let mut at = 6;
        while at < summary.len() {
            let [kind,sense,count] = summary.get(at..at+3).ok_or_else(bad)?.try_into().unwrap();
            let numbers = summary.get(at+3..at+3+n(count)).ok_or_else(bad)?.to_vec();
            let sense = sense as u8;
            faces.push(Face {kind:kind as u8,reversed:sense & 1 != 0,indirect:sense & 2 != 0,numbers});
            at += 3+n(count);
        }
        if faces.len() != n(summary[3]) { return Err(bad()) }
        Ok(Solid {solids:n(summary[1]),shells:n(summary[2]),edges:n(summary[4]),vertices:n(summary[5]),faces})
    }
}

/// What the verification found: the file's entity count and its solid's counts.
#[derive(Debug)]
pub(crate) struct Verified { pub entities: usize,pub faces: usize,pub edges: usize,pub vertices: usize,pub splines: usize }

fn near(a: f64,b: f64) -> bool { (a-b).abs() <= NEAR*(1.+a.abs().max(b.abs())) }

/// Verify a STEP file's text against the solid it was written from.
pub(crate) fn verify(text: &str,solid: &Solid) -> Result<Verified,String> {
    let debug = std::env::var_os("SOLVENT_STEP_DEBUG").is_some();
    let clock = std::time::Instant::now();
    let lap = |what: &str| if debug { eprintln!("step check: {what} {:?}",clock.elapsed()) };
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
    let dangling = gcs_core::par::indices(entities.len().div_ceil(4096),|chunk| {
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
    let get = |r: u32| &entities[index[r as usize] as usize];
    let of = |r: Option<u32>,kinds: &[&str],what: &str| -> Result<(&str,&[Value]),String> {
        let r = r.ok_or_else(|| format!("{what}: a reference expected"))?;
        let e = get(r);
        match e.simple() { Some((name,values)) if kinds.contains(&name) => Ok((name,values)),
            _ => Err(format!("{what}: #{r} is {}, not {}",e.parts.iter().map(|p| p.0).collect::<Vec<_>>().join("+"),kinds.join(" or "))) }
    };
    let count = |name: &str| entities.iter().filter(|e| e.is(name)).count();
    // the units
    let unit = |kind: &str,prefix: Option<&str>,name: &str| entities.iter().any(|e| e.is(kind) && e.part("SI_UNIT").is_some_and(|v| v.len() == 2
        && v[0].word() == prefix && v[1].word() == Some(name)));
    if !unit("LENGTH_UNIT",Some("MILLI"),"METRE") { return Err("the file's length unit is not the millimetre".into()) }
    if !unit("PLANE_ANGLE_UNIT",None,"RADIAN") { return Err("the file's plane angle unit is not the radian".into()) }
    // the topology, from the shape representation down
    let breps: Vec<&Entity> = entities.iter().filter(|e| e.simple().is_some_and(|(n,_)| n == "MANIFOLD_SOLID_BREP")).collect();
    let represented: BTreeSet<u32> = entities.iter().filter(|e| e.is("ADVANCED_BREP_SHAPE_REPRESENTATION"))
        .flat_map(|e| e.part("ADVANCED_BREP_SHAPE_REPRESENTATION").and_then(|v| v.get(1)).and_then(Value::list).unwrap_or(&[])
            .iter().filter_map(Value::reference).collect::<Vec<_>>()).collect();
    let shells_in_file = count("CLOSED_SHELL")+count("OPEN_SHELL");
    if breps.len() != solid.solids || shells_in_file != solid.shells || count("MANIFOLD_SOLID_BREP") != breps.len() {
        return Err(format!("the file has {} solids and {shells_in_file} shells, the solid {} and {}",breps.len(),solid.solids,solid.shells));
    }
    let mut faces: Vec<u32> = Vec::new();
    for b in &breps {
        if !represented.contains(&b.id) { return Err(format!("the solid #{} is not an item of the shape representation",b.id)) }
        let (_,shell) = of(b.simple().unwrap().1.get(1).and_then(Value::reference),&["CLOSED_SHELL"],&format!("the solid #{}",b.id))?;
        faces.extend(shell.get(1).and_then(Value::list).ok_or("a closed shell without faces")?.iter().map(|f| f.reference().ok_or("a shell's face is not a reference"))
            .collect::<Result<Vec<_>,_>>()?);
    }
    if faces.len() != solid.faces.len() || count("ADVANCED_FACE") != faces.len() {
        return Err(format!("the file's shells list {} faces of its {} faces, the solid has {}",faces.len(),count("ADVANCED_FACE"),solid.faces.len()));
    }
    lap("counted");
    let mut uses: BTreeMap<u32,u32> = BTreeMap::new();
    let mut vertices: BTreeSet<u32> = Default::default();
    let mut splines = 0;
    for (k,(&f,expected)) in faces.iter().zip(&solid.faces).enumerate() {
        let what = format!("face {k} (#{f})");
        let (_,face) = of(Some(f),&["ADVANCED_FACE"],&what)?;
        for bound in face.get(1).and_then(Value::list).ok_or_else(|| format!("{what}: no bounds"))? {
            let (_,bound) = of(bound.reference(),&["FACE_BOUND","FACE_OUTER_BOUND"],&what)?;
            bound.get(2).and_then(Value::logical).ok_or_else(|| format!("{what}: a bound without an orientation"))?;
            // a loop of edges, or the one vertex a pole leaves (the writer writes no degenerate edge)
            let (kind,edge_loop) = of(bound.get(1).and_then(Value::reference),&["EDGE_LOOP","VERTEX_LOOP"],&what)?;
            if kind == "VERTEX_LOOP" {
                let vertex = edge_loop.get(1).and_then(Value::reference);
                let (_,point) = of(vertex,&["VERTEX_POINT"],&what)?;
                of(point.get(1).and_then(Value::reference),&["CARTESIAN_POINT"],&what)?;
                vertices.insert(vertex.unwrap());
                continue;
            }
            for oriented in edge_loop.get(1).and_then(Value::list).ok_or_else(|| format!("{what}: an empty loop"))? {
                let (_,oriented) = of(oriented.reference(),&["ORIENTED_EDGE"],&what)?;
                oriented.get(4).and_then(Value::logical).ok_or_else(|| format!("{what}: an oriented edge without an orientation"))?;
                let edge = oriented.get(3).and_then(Value::reference);
                let (_,curve) = of(edge,&["EDGE_CURVE"],&what)?;
                *uses.entry(edge.unwrap()).or_insert(0) += 1;
                for v in [1,2] {
                    let vertex = curve.get(v).and_then(Value::reference);
                    let (_,point) = of(vertex,&["VERTEX_POINT"],&what)?;
                    of(point.get(1).and_then(Value::reference),&["CARTESIAN_POINT"],&what)?;
                    vertices.insert(vertex.unwrap());
                }
            }
        }
        let same_sense = face.get(3).and_then(Value::logical).ok_or_else(|| format!("{what}: no sense"))?;
        if same_sense != (expected.reversed == expected.indirect) { return Err(format!("{what}: its sense is {same_sense}, the solid's face \
            {}{}",if expected.reversed { "reversed" } else { "forward" },if expected.indirect { " on a left-handed placement" } else { "" })) }
        let surface = face.get(2).and_then(Value::reference).ok_or_else(|| format!("{what}: no surface"))?;
        if surface_matches(&get,surface,expected).map_err(|e| format!("{what}: {e}"))? { splines += 1; }
    }
    if let Some((e,n)) = uses.iter().find(|(_,&n)| n != 2) { return Err(format!("the edge #{e} is used {n} times by the shell's faces, not twice")) }
    if uses.len() != solid.edges || count("EDGE_CURVE") != uses.len() {
        return Err(format!("the file's faces use {} of its {} edges, the solid has {}",uses.len(),count("EDGE_CURVE"),solid.edges));
    }
    if vertices.len() != solid.vertices || count("VERTEX_POINT") != vertices.len() {
        return Err(format!("the file's edges end at {} of its {} vertices, the solid has {}",vertices.len(),count("VERTEX_POINT"),solid.vertices));
    }
    lap("walked");
    Ok(Verified {entities:entities.len(),faces:faces.len(),edges:uses.len(),vertices:vertices.len(),splines})
}

/// Whether the surface `r` is the solid's face's `expected`: an error where it is not, and
/// `Ok(true)` for a B-spline.
fn surface_matches<'a>(get: &dyn Fn(u32) -> &'a Entity<'a>,r: u32,expected: &Face) -> Result<bool,String> {
    let e = get(r);
    let reals = |values: &[Value],what: &str| -> Result<Vec<f64>,String> {
        values.iter().map(|v| v.number().ok_or_else(|| format!("{what}: a number expected"))).collect()
    };
    let triple = |r: Option<u32>,kind: &str| -> Result<[f64;3],String> {
        let r = r.ok_or_else(|| format!("a {kind} reference expected"))?;
        match get(r).simple() {
            Some((name,v)) if name == kind => {
                let x = reals(v.get(1).and_then(Value::list).ok_or_else(|| format!("#{r}: no coordinates"))?,kind)?;
                x.try_into().map_err(|_| format!("#{r}: not three coordinates"))
            }
            _ => Err(format!("#{r} is not a {kind}")),
        }
    };
    let compare = |what: &str,written: &[f64],solid: &[f64]| -> Result<(),String> {
        if written.len() != solid.len() { return Err(format!("{what}: {} numbers written, the solid has {}",written.len(),solid.len())) }
        match written.iter().zip(solid).position(|(a,b)| !near(*a,*b)) {
            Some(k) => Err(format!("{what}: {} written as its number {k}, the solid's {}{}",written[k],solid[k],
                if std::env::var_os("SOLVENT_STEP_DEBUG").is_some() { format!(" (written {written:?}, the solid's {solid:?})") } else { String::new() })),
            None => Ok(()),
        }
    };
    let elementary = |name: &str,kind: u8,scalars: usize| -> Result<bool,String> {
        let Some((n,v)) = e.simple().filter(|(n,_)| *n == name) else { unreachable!() };
        if expected.kind != kind { return Err(format!("#{r} is a {n}, the solid's surface kind {}",expected.kind)) }
        let (_,place) = match get(v.get(1).and_then(Value::reference).ok_or("no placement")?).simple() {
            Some(p @ ("AXIS2_PLACEMENT_3D",_)) => p,
            _ => return Err(format!("#{r}'s placement is not an AXIS2_PLACEMENT_3D")),
        };
        let mut written = Vec::new();
        written.extend(triple(place.get(1).and_then(Value::reference),"CARTESIAN_POINT")?);
        written.extend(triple(place.get(2).and_then(Value::reference),"DIRECTION")?);
        written.extend(triple(place.get(3).and_then(Value::reference),"DIRECTION")?);
        if v.len() != 2+scalars { return Err(format!("#{r}: {} parameters",v.len())) }
        written.extend(reals(&v[2..],n)?);
        compare(n,&written,&expected.numbers)?;
        Ok(false)
    };
    match e.simple().map(|(n,_)| n) {
        Some("PLANE") => return elementary("PLANE",PLANE,0),
        Some("CYLINDRICAL_SURFACE") => return elementary("CYLINDRICAL_SURFACE",CYLINDER,1),
        Some("CONICAL_SURFACE") => return elementary("CONICAL_SURFACE",CONE,2),
        Some("SPHERICAL_SURFACE") => return elementary("SPHERICAL_SURFACE",SPHERE,1),
        Some("TOROIDAL_SURFACE") => return elementary("TOROIDAL_SURFACE",TORUS,2),
        _ => {}
    }
    // a B-spline: simple, or complex with its rational weights
    let (surface,knots,weights) = match e.simple() {
        Some(("B_SPLINE_SURFACE_WITH_KNOTS",v)) if v.len() == 13 => (&v[1..8],&v[8..12],None),
        Some(_) => {
            let name = e.parts[0].0;
            let kind = match name { "SURFACE_OF_REVOLUTION" => REVOLUTION,"SURFACE_OF_LINEAR_EXTRUSION" => EXTRUSION,"OFFSET_SURFACE" => 9,"BEZIER_SURFACE" => 5,_ => 10 };
            if kind != expected.kind { return Err(format!("#{r} is a {name}, the solid's surface kind {}",expected.kind)) }
            return Ok(false);
        }
        None => match (e.part("B_SPLINE_SURFACE"),e.part("B_SPLINE_SURFACE_WITH_KNOTS")) {
            (Some(s),Some(k)) if s.len() == 7 && k.len() == 5 => (s,&k[..4],e.part("RATIONAL_B_SPLINE_SURFACE")),
            _ => return Err(format!("#{r} is a complex entity that is not a B-spline surface")),
        },
    };
    if expected.kind != BSPLINE { return Err(format!("#{r} is a B-spline surface, the solid's surface kind {}",expected.kind)) }
    let int = |v: &Value,what: &str| v.int().ok_or_else(|| format!("#{r}: {what} is not an integer"));
    let (ud,vd) = (int(&surface[0],"a degree")?,int(&surface[1],"a degree")?);
    let rows = surface[2].list().ok_or_else(|| format!("#{r}: no control points"))?;
    let columns = rows.first().and_then(Value::list).map_or(0,|c| c.len());
    let lists = |v: &Value| -> Result<Vec<f64>,String> { reals(v.list().ok_or_else(|| format!("#{r}: a list expected"))?,"a knot vector") };
    let (umults,vmults,uknots,vknots) = (lists(&knots[0])?,lists(&knots[1])?,lists(&knots[2])?,lists(&knots[3])?);
    let rational = weights.is_some();
    let mut written = vec![ud as f64,vd as f64,rows.len() as f64,columns as f64,uknots.len() as f64,vknots.len() as f64,if rational { 1. } else { 0. }];
    written.extend(&uknots); written.extend(&umults); written.extend(&vknots); written.extend(&vmults);
    let head = written.len();
    compare("the B-spline's degrees and knots",&written,expected.numbers.get(..head).unwrap_or(&expected.numbers))?;
    let mut poles = Vec::with_capacity(3*rows.len()*columns);
    for row in rows {
        let row = row.list().filter(|c| c.len() == columns).ok_or_else(|| format!("#{r}: the control points are not a grid"))?;
        for p in row { poles.extend(triple(p.reference(),"CARTESIAN_POINT")?); }
    }
    if let Some(w) = weights {
        let w = w.first().and_then(Value::list).ok_or_else(|| format!("#{r}: no weights"))?;
        for row in w { poles.extend(reals(row.list().ok_or_else(|| format!("#{r}: weights not a grid"))?,"a weight")?); }
    }
    compare("the B-spline's poles and weights",&poles,&expected.numbers[head..])?;
    Ok(true)
}

