//! **A reader never wrong, held to files broken on purpose** (#993 stage 1; docs/contract/brep.md):
//! the corpus (`step_corpus`) holds the reader to files written right; here it is held to files
//! each broken by one edit, which it must refuse or read to the solid the edit leaves — never a
//! solid that is not the file's, never a panic, never past a time bound.
//!
//! Three tests:
//!
//! * `no_mutant_is_read_wrong` (`#[ignore]`d: the mutants are made from the corpus outside the
//!   repository, `$STEP_CORPUS/mutants/`, by scripts/step-corpus/mutants.py): every mutant of the
//!   table `step_mutants/mutants.tsv` refused, or read to OCCT's reading of that mutant within its
//!   row's bar (`step_corpus`'s own judgement, `wrong`); a mutant OCCT reads to no solid (an open
//!   or inverted shell, a crash, a hang) or to an invalid one whose measures moved is refused. No
//!   read panics or takes more than `BOUND`.
//! * `kernel_written_mutants`: the same property over files the kernel writes (`step::write`)
//!   and this test breaks, each edit's outcome known without an oracle — refused, or (where the
//!   edit leaves the solid, as a stated uncertainty, or scales it, as the length unit) read to
//!   the solid's own measures. Fast, in the ordinary suite.
//! * `syntax_never_panics`: a kernel-written file truncated at every 97th byte, with seeded
//!   single-byte substitutions and with its lines shuffled, read without a panic, each within
//!   `QUICK`. What it returns is not judged.
use crate::step_corpus::{measured,read_table,report,table_at,unread,within,wrong,Outcome,Read,Row};
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{cone,disc,prism,sphere,square,torus};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::step::{write,Fit};
use gcs_core::brep::topo::Brep;
use gcs_core::rng::Rng;
use std::collections::BTreeMap;
use std::sync::OnceLock;
use std::time::{Duration,Instant};

const TABLE: &str = concat!(env!("CARGO_MANIFEST_DIR"),"/tests/step_mutants/mutants.tsv");

/// The time a corpus mutant's read may take: the slowest of the mutants' sources reads in some
/// 3 s on a busy machine (the progress report's `s` column), and a mutant is refused sooner.
const BOUND: Duration = Duration::from_secs(60);

/// The time a kernel-written file's read (some fifty entities, read in milliseconds) may take.
const QUICK: Duration = Duration::from_secs(5);

/// Every mutant of the table read once (the table `STEP_MUTANTS_TABLE` names, else the
/// committed one); `None` where there is no corpus.
fn mutants() -> Option<&'static (Vec<Row>,Vec<Read>)> {
    static ALL: OnceLock<Option<(Vec<Row>,Vec<Read>)>> = OnceLock::new();
    ALL.get_or_init(|| {
        let path = std::env::var("STEP_MUTANTS_TABLE").unwrap_or_else(|_| TABLE.to_string());
        read_table(table_at(&path),Some(BOUND))
    }).as_ref()
}

/// The mutation a row's path names (`mutants/<kind>/…`).
fn kind(row: &Row) -> &str { row.path.split('/').nth(1).unwrap_or("?") }

#[test]
#[ignore = "a tool: STEP_CORPUS names the corpus outside the repository (scripts/step-corpus/mutants.py)"]
fn no_mutant_is_read_wrong() {
    let Some((rows,reads)) = mutants() else { return };
    report(rows,reads);
    // the tally by mutation: refused, read right, read wrong, panicked, hung
    let mut tally = BTreeMap::<&str,[usize; 6]>::new();
    let mut bad = unread(rows,reads);
    for (row,r) in rows.iter().zip(reads) {
        let t = tally.entry(kind(row)).or_default();
        match &r.outcome {
            Outcome::Refused(_) => t[0] += 1,
            Outcome::Read {..} => match wrong(row,&r.outcome) {
                Some(w) => { t[2] += 1; bad.push(format!("{}: {w} ({})",row.path,row.from)) }
                None => t[1] += 1,
            },
            Outcome::Panicked(m) => { t[3] += 1; bad.push(format!("{}: panicked: {m}",row.path)) }
            Outcome::Hung(s) => { t[4] += 1; bad.push(format!("{}: not done in {s} s",row.path)) }
            Outcome::Missing | Outcome::Stale(_) => t[5] += 1,
        }
    }
    println!("{:<20} {:>8} {:>8} {:>6} {:>6} {:>5} {:>8}","mutation","refused","read","WRONG","PANIC","HUNG","missing");
    for (k,t) in &tally { println!("{k:<20} {:>8} {:>8} {:>6} {:>6} {:>5} {:>8}",t[0],t[1],t[2],t[3],t[4],t[5]) }
    assert!(bad.is_empty(),"{} mutants read wrong, panicked on or hung:\n{}",bad.len(),bad.join("\n"));
}

// ---- mutants of files the kernel writes --------------------------------------------------------

/// The solids written: a face of each surface kind the writer writes, a hole through one.
fn solids() -> Vec<(&'static str,Brep)> {
    let z = [0.,0.,1.];
    let block = prism(&square([0.,0.,0.],z,5.),0.,10.).unwrap();
    let pin = prism(&disc([0.,0.,-1.],z,3.),0.,12.).unwrap();
    vec![
        ("block",block.clone()),
        ("block drilled",combined(&block,&pin,Op::Cut,0.).unwrap()),
        ("cylinder",cone([0.,0.,0.],[0.2,-0.3,1.],1.2,1.2,3.).unwrap()),
        ("frustum",cone([0.,0.,0.],z,1.5,0.6,2.).unwrap()),
        ("sphere",sphere([0.5,0.,0.],z,1.3).unwrap()),
        ("torus",torus([0.,0.,0.],z,3.,1.).unwrap()),
    ]
}

/// A line `#n = KIND(args);` cut at its arguments: (`#n = KIND(`, the arguments split at their
/// top level, `);`). None for another line, or a complex entity's.
fn split(line: &str) -> Option<(String,Vec<String>,String)> {
    let open = line.find('(')?;
    if line[..open].trim_end().ends_with('=') { return None }
    let close = line.rfind(')')?;
    let (mut args,mut depth,mut quoted,mut start) = (Vec::new(),0i32,false,open+1);
    for (i,c) in line.char_indices().take(close).skip(open+1) {
        match c {
            '\'' => quoted = !quoted,
            '(' if !quoted => depth += 1,
            ')' if !quoted => depth -= 1,
            ',' if !quoted && depth == 0 => { args.push(line[start..i].to_string()); start = i+1 }
            _ => {}
        }
    }
    args.push(line[start..close].to_string());
    Some((line[..=open].to_string(),args,line[close..].to_string()))
}

/// The entity a line defines and its kind.
fn head(line: &str) -> Option<(u32,&str)> {
    let rest = line.strip_prefix('#')?;
    let eq = rest.find('=')?;
    let id = rest[..eq].trim().parse().ok()?;
    let kind = rest[eq+1..].trim_start();
    Some((id,&kind[..kind.find('(')?].trim_end()))
}

/// Whether the loop `id` names holds each of its edges twice: a loop of seams alone.
fn seams_only(loops: &BTreeMap<String,Vec<u32>>,id: &str) -> bool {
    loops.get(id).is_some_and(|es| !es.is_empty() && es.iter().all(|e| es.iter().filter(|f| *f == e).count() == 2))
}

/// What a mutant must be read to: refused, or (refused or) the solid scaled by the factor.
#[derive(Clone,Copy,Debug,PartialEq)]
enum Expect { Refused,Same(f64) }

/// A file's text as its lines, and the mutants of it each edit makes: (what, expectation, text).
fn mutate(text: &str,size: f64) -> Vec<(String,Expect,String)> {
    let lines: Vec<&str> = text.lines().collect();
    let ids: Vec<Option<(u32,&str)>> = lines.iter().map(|l| head(l)).collect();
    let of = |kinds: &[&str]| -> Vec<u32> { ids.iter().flatten().filter(|(_,k)| kinds.contains(k)).map(|(i,_)| *i).collect() };
    let next = ids.iter().flatten().map(|(i,_)| *i).max().unwrap_or(0)+1;
    let arg = |kind: &str,k: usize| -> BTreeMap<String,String> { lines.iter().filter_map(|l| {
        let (id,kd) = head(l)?;
        (kd == kind).then(|| (format!("#{id}"),split(l).unwrap().1[k].trim().to_string()))
    }).collect() };
    let oriented = arg("ORIENTED_EDGE",3);
    let loops: BTreeMap<String,Vec<u32>> = arg("EDGE_LOOP",1).into_iter().map(|(id,list)| {
        let edges = list.trim_matches(|c| c == '(' || c == ')').split(',')
            .filter_map(|o| oriented.get(o.trim())?.trim_start_matches('#').parse().ok()).collect();
        (id,edges)
    }).collect();
    let mut out = Vec::new();
    // an argument of each line of `kinds` replaced: `edit(args) -> new args`, None for no edit
    let mut each = |what: &str,expect: &dyn Fn(u32) -> Expect,kinds: &[&str],edit: &dyn Fn(u32,&[String]) -> Option<(Vec<String>,Option<String>)>| {
        for (k,line) in lines.iter().enumerate() {
            let Some((id,kind)) = ids[k] else { continue };
            if !kinds.contains(&kind) { continue }
            let Some((pre,args,post)) = split(line) else { continue };
            let Some((new,extra)) = edit(id,&args) else { continue };
            let mut ls: Vec<String> = lines.iter().map(|l| l.to_string()).collect();
            ls[k] = format!("{pre}{}{post}",new.join(","));
            if let Some(extra) = extra {
                let end = ls.iter().rposition(|l| l.starts_with("ENDSEC")).unwrap();
                ls.insert(end,extra);
            }
            out.push((format!("{what} at #{id}"),expect(id),ls.join("\n")+"\n"));
        }
    };
    let flip = |a: &str| match a.trim() { ".T." => Some(".F.".to_string()),".F." => Some(".T.".to_string()),_ => None };
    let set = |args: &[String],i: usize,v: String| -> Vec<String> { let mut a = args.to_vec(); a[i] = v; a };
    each("face sense flipped",&|_| Expect::Refused,&["ADVANCED_FACE"],&|_,a| Some((set(a,3,flip(&a[3])?),None)));
    each("oriented edge flipped",&|_| Expect::Refused,&["ORIENTED_EDGE"],&|_,a| Some((set(a,4,flip(&a[4])?),None)));
    // a loop of seams alone (a whole sphere's or torus's face: each edge in it twice, once each
    // way) is the same loop traversed either way, and its edges' arcs the same between their ends
    // whichever way they run: flipped or swapped there, the face is the one written (or refused)
    let bounds_on_seams: Vec<u32> = lines.iter().filter_map(|l| {
        let (id,kind) = head(l)?;
        if !matches!(kind,"FACE_BOUND" | "FACE_OUTER_BOUND") { return None }
        seams_only(&loops,split(l)?.1[1].trim()).then_some(id)
    }).collect();
    let edges_on_seams: Vec<u32> = loops.values().filter(|es| es.iter().all(|e| es.iter().filter(|f| *f == e).count() == 2))
        .flatten().copied().collect();
    let same_if = |ids: Vec<u32>| move |id: u32| if ids.contains(&id) { Expect::Same(1.) } else { Expect::Refused };
    each("bound flipped",&same_if(bounds_on_seams),&["FACE_BOUND","FACE_OUTER_BOUND"],&|_,a| Some((set(a,2,flip(&a[2])?),None)));
    each("vertices swapped",&same_if(edges_on_seams),&["EDGE_CURVE"],&|_,a| (a[1] != a[2]).then(|| {
        let mut b = a.to_vec(); b.swap(1,2); (b,None) }));
    let points: BTreeMap<String,String> = lines.iter().filter_map(|l| {
        let (id,kind) = head(l)?;
        (kind == "CARTESIAN_POINT").then(|| (format!("#{id}"),split(l).unwrap().1[1].clone()))
    }).collect();
    each("vertex moved half the part",&|_| Expect::Refused,&["VERTEX_POINT"],&|_,a| {
        let xyz: Vec<f64> = points.get(a[1].trim())?.trim_matches(|c| c == '(' || c == ')').split(',')
            .map(|x| x.trim().parse().ok()).collect::<Option<_>>()?;
        let moved = format!("#{next} = CARTESIAN_POINT('',({:?},{:?},{:?}));",xyz[0]+size/2.,xyz[1],xyz[2]);
        Some((set(a,1,format!("#{next}")),Some(moved)))
    });
    each("radius scaled",&|_| Expect::Refused,&["CYLINDRICAL_SURFACE","CONICAL_SURFACE","SPHERICAL_SURFACE","TOROIDAL_SURFACE"],
        &|_,a| { let r: f64 = a[2].trim().parse().ok()?; Some((set(a,2,format!("{:?}",r*1.01)),None)) });
    let dirs = of(&["DIRECTION"]);
    let verts = of(&["VERTEX_POINT"]);
    each("surface a DIRECTION",&|_| Expect::Refused,&["ADVANCED_FACE"],&|_,a| Some((set(a,2,format!("#{}",dirs.first()?)),None)));
    each("curve a VERTEX_POINT",&|_| Expect::Refused,&["EDGE_CURVE"],&|_,a| Some((set(a,3,format!("#{}",verts.first()?)),None)));
    each("surface missing",&|_| Expect::Refused,&["ADVANCED_FACE"],&|_,a| Some((set(a,2,format!("#{}",next+1000)),None)));
    each("surface itself",&|_| Expect::Refused,&["ADVANCED_FACE"],&|id,a| Some((set(a,2,format!("#{id}")),None)));
    each("uncertainty huge",&|_| Expect::Same(1.),&["UNCERTAINTY_MEASURE_WITH_UNIT"],
        &|_,a| Some((set(a,0,format!("LENGTH_MEASURE({:?})",size/2.)),None)));
    each("uncertainty zero",&|_| Expect::Same(1.),&["UNCERTAINTY_MEASURE_WITH_UNIT"],
        &|_,a| Some((set(a,0,"LENGTH_MEASURE(0.)".into()),None)));
    // the shell's faces: each dropped, each listed twice
    for (k,line) in lines.iter().enumerate() {
        if ids[k].map(|(_,kind)| kind) != Some("CLOSED_SHELL") { continue }
        let (pre,args,post) = split(line).unwrap();
        let faces: Vec<&str> = args[1].trim().trim_matches(|c| c == '(' || c == ')').split(',').collect();
        for i in 0..faces.len() {
            let mut dropped = faces.clone();
            let f = dropped.remove(i);
            let mut twice = faces.clone();
            twice.push(f);
            for (what,list) in [("dropped",dropped),("listed twice",twice)] {
                // a shell left with no face is `an_empty_shell_is_refused`'s
                if list.is_empty() { continue }
                let mut ls: Vec<String> = lines.iter().map(|l| l.to_string()).collect();
                ls[k] = format!("{pre}{},({}){post}",args[0],list.join(","));
                out.push((format!("face {f} {what}"),Expect::Refused,ls.join("\n")+"\n"));
            }
        }
    }
    // millimetres to centimetres: the solid ten times as large
    let unit = "SI_UNIT(.MILLI.,.METRE.)";
    if text.matches(unit).count() == 1 {
        out.push(("unit to centimetres".into(),Expect::Same(10.),text.replace(unit,"SI_UNIT(.CENTI.,.METRE.)")));
    }
    out
}

#[test]
fn kernel_written_mutants() {
    let clock = Instant::now();
    let (mut count,mut refused,mut bad) = (0,0,Vec::new());
    for (name,b) in solids() {
        let (v,a) = (volume(&b),b.faces.iter().map(|f| area(&b,f)).sum::<f64>());
        let (lo,hi) = b.bounds();
        let size = ((hi[0]-lo[0]).powi(2)+(hi[1]-lo[1]).powi(2)+(hi[2]-lo[2]).powi(2)).sqrt();
        let text = write(&b,name,1e-7,Fit::Exact).unwrap();
        // the file as written reads back (step_read holds it to that)
        assert!(matches!(measured(&text),Outcome::Read {..}),"{name}: not read as written");
        for (what,expect,mutant) in mutate(&text,size) {
            count += 1;
            let start = Instant::now();
            let got = within(mutant,QUICK);
            let s = start.elapsed().as_secs_f64();
            let fault = match (&got,expect) {
                (Outcome::Refused(_),_) => { refused += 1; None }
                (Outcome::Read {volume,area,..},Expect::Same(k)) => {
                    let (dv,da) = ((volume-v*k*k*k).abs()/(v*k*k*k),(area-a*k*k).abs()/(a*k*k));
                    (dv > 1e-9 || da > 1e-9).then(|| format!("read volume {volume} area {area}, the solid's {} {}",v*k*k*k,a*k*k))
                }
                (Outcome::Read {volume,area,..},Expect::Refused) =>
                    Some(format!("read a solid, volume {volume} area {area} (the unbroken solid's {v} {a})")),
                (Outcome::Panicked(m),_) => Some(format!("panicked: {m}")),
                (Outcome::Hung(t),_) => Some(format!("not done in {t} s")),
                _ => Some("no outcome".into()),
            };
            if let Some(f) = fault { bad.push(format!("{name}, {what}: {f} ({s:.2} s)")) }
        }
    }
    eprintln!("{count} mutants of kernel-written files, {refused} refused, in {:.1} s",clock.elapsed().as_secs_f64());
    assert!(count > 300,"{count} mutants");
    assert!(bad.is_empty(),"{} mutants read wrong:\n{}",bad.len(),bad.join("\n"));
}

/// A solid whose shell lists no face is refused: the whole sphere's and the whole torus's one
/// face dropped from its shell.
#[test]
fn an_empty_shell_is_refused() {
    for (name,b) in solids().into_iter().filter(|(_,b)| b.faces.len() == 1) {
        let text = write(&b,name,1e-7,Fit::Exact).unwrap();
        let line = text.lines().find(|l| head(l).is_some_and(|(_,k)| k == "CLOSED_SHELL")).unwrap();
        let (pre,args,post) = split(line).unwrap();
        let empty = text.replace(line,&format!("{pre}{},(){post}",args[0]));
        let got = within(empty,QUICK);
        assert!(matches!(got,Outcome::Refused(_)),"{name}: a shell of no faces not refused");
    }
}

// ---- syntax --------------------------------------------------------------------------------------

/// What a byte is replaced by: the characters Part 21 gives meaning, a digit, a letter, a space.
const BYTES: &[u8] = b"#=();,'.$*-+0123456789EeTFABCDZ_ \n/";

#[test]
fn syntax_never_panics() {
    let z = [0.,0.,1.];
    let block = prism(&square([0.,0.,0.],z,5.),0.,10.).unwrap();
    let pin = prism(&disc([0.,0.,-1.],z,3.),0.,12.).unwrap();
    let text = write(&combined(&block,&pin,Op::Cut,0.).unwrap(),"drilled",1e-7,Fit::Exact).unwrap();
    assert!(text.is_ascii());
    let mut texts: Vec<(String,String)> = (0..text.len()).step_by(97).map(|n| (format!("cut at byte {n}"),text[..n].to_string())).collect();
    let mut rng = Rng::new(993);
    for _ in 0..300 {
        let (at,b) = (rng.int(text.len()),BYTES[rng.int(BYTES.len())]);
        let mut bytes = text.clone().into_bytes();
        let was = bytes[at];
        bytes[at] = b;
        texts.push((format!("byte {at} {:?} to {:?}",was as char,b as char),String::from_utf8(bytes).unwrap()));
    }
    for k in 0..20 {
        let mut lines: Vec<&str> = text.lines().collect();
        // Fisher–Yates over the seeded draws
        for i in (1..lines.len()).rev() { lines.swap(i,rng.int(i+1)) }
        texts.push((format!("lines shuffled ({k})"),lines.join("\n")));
    }
    // lists nested a hundred thousand deep: refused, where following them would run the parser
    // off its stack (an abort no `catch_unwind` catches)
    let n = 100_000;
    let deep = text.replacen("CARTESIAN_POINT('',(",&format!("CARTESIAN_POINT('',{}{},(","(".repeat(n),")".repeat(n)),1);
    assert!(matches!(within(deep.clone(),QUICK),Outcome::Refused(_)),"lists nested {n} deep not refused");
    texts.push(("lists nested deep".into(),deep));
    let clock = Instant::now();
    let mut bad = Vec::new();
    let mut tally = BTreeMap::<&str,usize>::new();
    for (what,t) in texts.iter() {
        let start = Instant::now();
        let got = within(t.clone(),QUICK);
        let s = start.elapsed().as_secs_f64();
        *tally.entry(match &got { Outcome::Read {..} => "read",Outcome::Refused(_) => "refused",
            Outcome::Panicked(_) => "panicked",Outcome::Hung(_) => "hung",_ => "?" }).or_default() += 1;
        match got {
            Outcome::Panicked(m) => bad.push(format!("{what}: panicked: {m}")),
            Outcome::Hung(t) => bad.push(format!("{what}: not done in {t} s")),
            _ if s > QUICK.as_secs_f64() => bad.push(format!("{what}: {s:.2} s")),
            _ => {}
        }
    }
    eprintln!("{} broken texts read in {:.1} s: {tally:?}",texts.len(),clock.elapsed().as_secs_f64());
    assert!(bad.is_empty(),"{} broken texts panicked or hung:\n{}",bad.len(),bad.join("\n"));
}
