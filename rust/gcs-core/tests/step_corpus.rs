//! **STEP files other writers wrote, read** (#993 stage 1; docs/contract/brep.md): the corpus
//! outside the repository, each file read to a closed solid whose measures agree with its
//! reference's.
//!
//! The corpus is `$STEP_CORPUS` (scripts/step-corpus/README.md makes it from nothing): the 32
//! CADGenBench editing inputs, the STEP files other kernels write for brep-bench's settled cases,
//! and a sample of both written again by OCCT under other writers' habits. Its reference is the
//! committed table `step_corpus/reference.tsv` (`STEP_CORPUS_TABLE` names another, a table
//! regenerated beside a corpus regenerated): a row a file, its bytes known by the SHA-256 of its
//! DATA section, the topological entities its solids hold (each ADVANCED_FACE, EDGE_CURVE and
//! VERTEX_POINT once), and its volume and area with the bar a reading must come within — the
//! case's closed form at the bench's own grade where the file holds it, else OCCT's reading
//! (with its healing off) at a bar its own error and the file's stated slack set
//! (scripts/step-corpus/table.py says how).
//!
//! Three tests, each reading every file once between them (`outcomes`, in parallel), each
//! `#[ignore]`d since the corpus is outside the repository and the ordinary suite must not need
//! it:
//!
//! * `nothing_is_read_wrong`: each file is refused, or read to its solids, each file entity
//!   read and none twice-counted, and their volume and area within the bar, and each face's area
//!   within its own bar of OCCT's and brep-bench's measurer's readings of that face (`Refs`,
//!   `face_bar`, `faces_judged`: a total within its bar does not show a solid read right); a
//!   file that represents no solid (`ref_from` `refuse`: a shell-based surface model, an empty product) is
//!   refused. Holds at every commit, the reader refusing everything included.
//! * `nothing_panics`: no read panics (`catch_unwind`); a panic names its file.
//! * `the_corpus_reads`: every file read, none refused but those that represent no solid, that
//!   OCCT and brep-bench's measurer read differently (`disputed`: mostly loops against ISO
//!   10303-42, which OCCT repairs), or whose account (`step_corpus/account.tsv`, a verdict, a
//!   witness and the evidence a file) says the file is no solid's boundary (`defective`, which
//!   must be refused) or an assembly. The stage's finish line: fails until the reader is done,
//!   and its table is the progress report, with the account by source and verdict (`account`).
//!
//! A read's totals are held to its row's bar or to what the file determines, whichever is wider
//! (`determined`, `bars`): its area to the sum of its faces' slacks, its volume to that swept
//! over its reach. Where the readers agree closer than that and the reading does not, it is said
//! (`read~`), not failed.
//!
//! ```sh
//! STEP_CORPUS=/path/to/corpus cargo test --manifest-path rust/Cargo.toml -p gcs-core -- \
//!     step_corpus:: --ignored --nocapture
//! ```
//! `STEP_CORPUS_ONLY=cadgenbench/2` reads only the files whose path starts so (several prefixes
//! separated by commas);
//! `STEP_CORPUS_THREADS` how many at once (the machine's parallelism by default).
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::step_read::{read,Given,Refusal};
use gcs_core::brep::topo::{Brep,EdgeCurve};
use std::collections::{BTreeMap,BTreeSet};
use std::path::{Path,PathBuf};
use std::sync::OnceLock;
use std::time::{Duration,Instant};

/// The committed table, read when the tests run (a table regenerated needs no rebuild).
const TABLE: &str = concat!(env!("CARGO_MANIFEST_DIR"),"/tests/step_corpus/reference.tsv");

/// A file's row of the reference table (and of `step_mutants`'s, which has its columns).
pub(crate) struct Row {
    pub(crate) path: String,
    data_sha256: String,
    solids: usize,
    /// The entities its solids hold: ADVANCED_FACEs, EDGE_CURVEs and VERTEX_POINTs, each once.
    entities: [usize; 3],
    volume: f64,
    area: f64,
    bar_volume: f64,
    bar_area: f64,
    /// Where the reference comes from: `closed` (the case's closed form), `mass+occt`, `mass`,
    /// `occt`, `source:…`; `refuse` for a file that represents no solid; `disputed` for one the
    /// two readers read differently, where `alt` is the second reading.
    pub(crate) from: String,
    alt: Option<(f64,f64)>,
    /// The account of a file not read right (`step_corpus/account.tsv`, written into the table by
    /// `table.py`): `gap` (the reader's: a sound file another reader reads to the reference),
    /// `defective` (not a solid's boundary by ISO 10303-42, or its edges past `sound::LOOSE` off
    /// its faces: must be refused), `assembly` (product structure, deferred by the stage),
    /// `nosolid`, `disputed`, `reference` or `bar` (the row corrected), `unchecked`, `open` (not
    /// settled by the evidence in hand); empty for a file the reader must simply read right.
    pub(crate) verdict: String,
    /// Faces whose one reference no second reader has checked (OCCT's integration in question,
    /// the measurer unable): a face past its bar of that one alone is said, not failed.
    pub(crate) unchecked: BTreeSet<u32>,
    /// Why, with the witness entity and the evidence (`scripts/step-corpus/check.py`).
    pub(crate) account: String,
}

impl Row {
    /// Whether the file represents no solid, so that the reader must refuse it.
    pub(crate) fn refuse(&self) -> bool { self.from == "refuse" }
}

fn table() -> Vec<Row> {
    table_at(&std::env::var("STEP_CORPUS_TABLE").unwrap_or_else(|_| TABLE.to_string()))
}

/// The rows of the table at `path`: a `#` line is a comment, the first other its header.
pub(crate) fn table_at(path: &str) -> Vec<Row> {
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let head: Vec<&str> = lines.next().expect("the table's header").split('\t').collect();
    let col = |name: &str| head.iter().position(|h| *h == name).unwrap_or_else(|| panic!("no column {name}"));
    let (path,sha,solids,faces,edges,vertices) = (col("path"),col("data_sha256"),col("solids"),
        col("step_faces"),col("step_edges"),col("step_vertices"));
    let (v,a,bv,ba,from) = (col("ref_volume"),col("ref_area"),col("bar_volume"),col("bar_area"),col("ref_from"));
    let (av,aa) = (col("alt_volume"),col("alt_area"));
    // the account's columns, which a mutant's table has not
    let opt = |name: &str| head.iter().position(|h| *h == name);
    let (verdict,unchecked,account) = (opt("verdict"),opt("unchecked"),opt("account"));
    lines.filter(|l| !l.is_empty()).map(|l| {
        let f: Vec<&str> = l.split('\t').collect();
        let n = |i: usize| f[i].parse::<usize>().unwrap_or_else(|_| panic!("{}: {:?} is no count",f[path],f[i]));
        let x = |i: usize| f[i].parse::<f64>().unwrap_or(f64::NAN);
        let s = |i: Option<usize>| i.and_then(|i| f.get(i)).map_or(String::new(),|s| s.to_string());
        Row { path: f[path].to_string(),data_sha256: f[sha].to_string(),solids: n(solids),
            entities: [n(faces),n(edges),n(vertices)],volume: x(v),area: x(a),bar_volume: x(bv),bar_area: x(ba),
            from: f[from].to_string(),alt: (!f[av].is_empty()).then(|| (x(av),x(aa))),verdict: s(verdict),
            unchecked: s(unchecked).split(',').filter_map(|e| e.trim().trim_start_matches('#').parse().ok()).collect(),
            account: s(account) }
    }).collect()
}

/// What reading a file came to.
pub(crate) enum Outcome {
    /// Not in the corpus.
    Missing,
    /// In the corpus, but not the bytes the table describes.
    Stale(String),
    Refused(Refusal),
    Panicked(String),
    /// Not done within the time bound (seconds); its thread is left to run.
    Hung(f64),
    /// `reach`: the farthest the solid's vertices and edges stand from the point its volume's flux
    /// is taken about (`props::volume`: the origin, or the solid's centre where it lies more than
    /// ten sizes away), what an unclosed boundary's slack is swept over (`determined`).
    Read { solids: usize,entities: [usize; 3],volume: f64,area: f64,faces: Vec<Face>,reach: f64 },
}

/// A face read, as the per-face check needs it.
pub(crate) struct Face {
    /// The ADVANCED_FACE it was read from (0 for one the reader laid).
    pub(crate) entity: u32,
    pub(crate) kind: &'static str,
    pub(crate) area: f64,
    /// `Σ max(tol, u) × length` over its loops' uses (mm²), `tol` the edge's measured tolerance
    /// and `u` the uncertainty its file states: the area its boundary's slack can sweep, so the
    /// most two readings of a tolerant boundary can differ by.
    pub(crate) slack: f64,
    /// `Σ tol × length` alone, the uncertainty left out (for the census's measure of the bar).
    pub(crate) measured: f64,
}

pub(crate) struct Read { pub(crate) outcome: Outcome,pub(crate) seconds: f64,pub(crate) refs: Refs }

/// The entities `given` read, each once (0, an entity the reader laid itself, not counted).
fn entities(given: &[Given]) -> [usize; 3] {
    let distinct = |ids: &mut dyn Iterator<Item=u32>| ids.filter(|&e| e != 0).collect::<BTreeSet<_>>().len();
    [distinct(&mut given.iter().flat_map(|g| g.faces.iter().copied())),
        distinct(&mut given.iter().flat_map(|g| g.edges.iter().copied())),
        distinct(&mut given.iter().flat_map(|g| g.vertices.iter().copied()))]
}

/// An edge's length, by the chord of `CHORDS` pieces of its parameter (a degenerate edge's 0).
fn length(b: &Brep,e: usize) -> f64 {
    const CHORDS: usize = 32;
    let edge = &b.edges[e];
    if matches!(edge.curve,EdgeCurve::Degenerate) { return 0. }
    let at = |k: usize| edge.point(edge.t[0]+(edge.t[1]-edge.t[0])*k as f64/CHORDS as f64,&b.vertices);
    (0..CHORDS).map(|k| { let (p,q) = (at(k),at(k+1)); ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt() }).sum()
}

/// How far `b`'s vertices and edges (each at `CHORDS` points) reach from the point
/// `props::volume` takes its flux about: the origin, or the centre of the vertices' box where that
/// lies more than ten of the box's diagonals away.
fn reach(b: &Brep) -> f64 {
    const CHORDS: usize = 8;
    let (mut lo,mut hi) = ([f64::INFINITY; 3],[f64::NEG_INFINITY; 3]);
    for v in &b.vertices { for k in 0..3 { lo[k] = lo[k].min(v.p[k]); hi[k] = hi[k].max(v.p[k]) } }
    if b.vertices.is_empty() { return 0. }
    let c: [f64; 3] = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
    let size = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
    let o = if (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt() > 10.*size { c } else { [0.; 3] };
    let far = |p: [f64; 3]| ((p[0]-o[0]).powi(2)+(p[1]-o[1]).powi(2)+(p[2]-o[2]).powi(2)).sqrt();
    let on_edges = b.edges.iter().filter(|e| !matches!(e.curve,EdgeCurve::Degenerate)).flat_map(|e|
        (0..=CHORDS).map(move |k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/CHORDS as f64,&b.vertices)));
    b.vertices.iter().map(|v| far(v.p)).chain(on_edges.map(far)).fold(0.,f64::max)
}

/// Each face of `given`, its area measured once (the file's area is their sum).
fn faces_of(given: &[Given]) -> Vec<Face> {
    given.iter().flat_map(|g| {
        let b = &g.brep;
        let lengths: Vec<f64> = (0..b.edges.len()).map(|e| length(b,e)).collect();
        let u = g.context.uncertainty.unwrap_or(0.);
        b.faces.iter().enumerate().map(move |(k,f)| Face {entity: g.faces.get(k).copied().unwrap_or(0),
            kind: f.surface.kind(),area: area(b,f),
            slack: f.loops.iter().flatten().map(|c| b.edges[c.edge as usize].tol.max(u)*lengths[c.edge as usize]).sum(),
            measured: f.loops.iter().flatten().map(|c| b.edges[c.edge as usize].tol*lengths[c.edge as usize]).sum()})
            .collect::<Vec<_>>()
    }).collect()
}

fn read_one(root: &Path,row: &Row,bound: Option<Duration>) -> Outcome {
    let Ok(bytes) = std::fs::read(root.join(&row.path)) else { return Outcome::Missing };
    let data = bytes.windows(6).position(|w| w == b"\nDATA;").unwrap_or(0);
    let sha = hex(&sha256(&bytes[data..]));
    if sha != row.data_sha256 { return Outcome::Stale(sha) }
    let text = String::from_utf8_lossy(&bytes).into_owned();
    match bound { Some(bound) => within(text,bound),None => measured(&text) }
}

/// `measured` on a thread of its own, waited on no longer than `bound`: `Hung` past it (the
/// thread left to run).
pub(crate) fn within(text: String,bound: Duration) -> Outcome {
    let (tx,rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || { let _ = tx.send(measured(&text)); });
    rx.recv_timeout(bound).unwrap_or(Outcome::Hung(bound.as_secs_f64()))
}

/// The solids `text` reads to, measured; a refusal, or the panic's message.
pub(crate) fn measured(text: &str) -> Outcome {
    let got = std::panic::catch_unwind(|| read(&text).map(|given| {
        let v: f64 = given.iter().map(|g| volume(&g.brep)).sum();
        let faces = faces_of(&given);
        let r = given.iter().map(|g| reach(&g.brep)).fold(0.,f64::max);
        (given.len(),entities(&given),v,faces.iter().map(|f| f.area).sum::<f64>(),faces,r)
    }));
    match got {
        Ok(Ok((solids,entities,volume,area,faces,reach))) => Outcome::Read {solids,entities,volume,area,faces,reach},
        Ok(Err(r)) => Outcome::Refused(r),
        Err(p) => Outcome::Panicked(p.downcast_ref::<&str>().map(|s| s.to_string())
            .or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_else(|| "a panic".into())),
    }
}

/// Every file of the table read once, in parallel, the largest first; `None` where there is no
/// corpus.
fn outcomes() -> Option<&'static (Vec<Row>,Vec<Read>)> {
    static ALL: OnceLock<Option<(Vec<Row>,Vec<Read>)>> = OnceLock::new();
    ALL.get_or_init(|| read_table(table(),None)).as_ref()
}

/// `rows`'s files under `$STEP_CORPUS` (narrowed by `STEP_CORPUS_ONLY`) each read once, in
/// parallel, the largest first, each within `bound` where one is given; `None` where there is no
/// corpus.
pub(crate) fn read_table(rows: Vec<Row>,bound: Option<Duration>) -> Option<(Vec<Row>,Vec<Read>)> {
    let Ok(root) = std::env::var("STEP_CORPUS") else {
        eprintln!("STEP_CORPUS is not set: no corpus to read (scripts/step-corpus/README.md)");
        return None
    };
    let root = PathBuf::from(root);
    // several prefixes, each a path's start, separated by commas (no corpus path holds one)
    let only = std::env::var("STEP_CORPUS_ONLY").unwrap_or_default();
    let only: Vec<&str> = only.split(',').collect();
    let rows: Vec<Row> = rows.into_iter().filter(|r| only.iter().any(|o| r.path.starts_with(o))).collect();
    let mut order: Vec<usize> = (0..rows.len()).collect();
    let size = |i: usize| std::fs::metadata(root.join(&rows[i].path)).map(|m| m.len()).unwrap_or(0);
    order.sort_by_key(|&i| std::cmp::Reverse(size(i)));
    let threads = std::env::var("STEP_CORPUS_THREADS").ok().and_then(|t| t.parse().ok())
        .unwrap_or_else(|| std::thread::available_parallelism().map(|n| n.get()).unwrap_or(2));
    let next = std::sync::atomic::AtomicUsize::new(0);
    let done = std::sync::Mutex::new((0..rows.len()).map(|_| None).collect::<Vec<Option<Read>>>());
    let clock = Instant::now();
    std::thread::scope(|s| for _ in 0..threads.max(1) {
        s.spawn(|| loop {
            let k = next.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
            let Some(&i) = order.get(k) else { break };
            let start = Instant::now();
            let outcome = read_one(&root,&rows[i],bound);
            let seconds = start.elapsed().as_secs_f64();
            let refs = match &outcome { Outcome::Read {..} => Refs::of(&root,&rows[i]),_ => Refs::none() };
            done.lock().unwrap()[i] = Some(Read {outcome,seconds,refs});
        });
    });
    eprintln!("{} files of {} read in {:.1} s, {threads} at once",rows.len(),root.display(),
        clock.elapsed().as_secs_f64());
    let reads = done.into_inner().unwrap().into_iter().map(|r| r.unwrap()).collect();
    Some((rows,reads))
}

pub(crate) fn rel(got: f64,want: f64) -> f64 { (got-want).abs()/want.abs() }

// ---- face by face ------------------------------------------------------------------------------

/// What the two readers say of each face of a file, by its ADVANCED_FACE, read from
/// `$STEP_CORPUS/oracle/` (scripts/step-corpus/faces.py makes it): OCCT's area and centroid
/// (`<path>.faces.tsv`, a face OCCT split summed, named where `faces_mapped` or `faces_named`
/// says every face is), and brep-bench's measurer's area (`<path>.mass.faces.tsv`, the faces it
/// can measure). Each is the reason it is missing where it is: a table absent or of other bytes
/// is said, never taken for agreement.
///
/// A third reader's areas (`<path>.quad.faces.tsv`, the measurer's columns), where there is one:
/// an independent quadrature of some faces over the file's own boundary, made where the two
/// readers' integrations are in question (scripts/step-corpus/teapot.py for brep-bench's Utah
/// teapot, whose knob all three of OCCT, the measurer and the case's kernels read low). A face it
/// measures is held to it, and to each reader within its bar of it; a reader past its bar of it
/// is set aside for that face.
pub(crate) struct Refs {
    pub(crate) occt: Result<BTreeMap<u32,(f64,[f64; 3])>,String>,
    pub(crate) mass: Result<BTreeMap<u32,f64>,String>,
    pub(crate) quad: Result<BTreeMap<u32,f64>,String>,
}

impl Refs {
    pub(crate) fn none() -> Refs {
        Refs {occt: Err("not read".into()),mass: Err("not read".into()),quad: Err("not read".into())}
    }

    /// The faces of `row`'s file. A reader whose faces sum to an area that is not the file's —
    /// outside the row's bar of its reference, or of its second reading — has misread a face
    /// somewhere, and is no reference for any (OCCT reads `balls-meeting-through-a-pole`'s two
    /// spheres each a quarter of themselves); the measurer is held to that only where it measured
    /// every face.
    pub(crate) fn of(root: &Path,row: &Row) -> Refs {
        let (path,sha) = (&row.path,&row.data_sha256);
        let at = |x: &str| root.join("oracle").join(format!("{path}{x}"));
        let file = |a: f64| rel(a,row.area) <= row.bar_area || row.alt.map_or(false,|(_,b)| rel(a,b) <= row.bar_area);
        let occt = occt_faces(&at(".json"),&at(".faces.tsv"),sha).and_then(|o| {
            let a: f64 = o.values().map(|x| x.0).sum();
            if file(a) { Ok(o) } else { Err(format!("OCCT's faces sum to {a:.9e}, {:.1e} off the file's area",rel(a,row.area))) }
        });
        let mass = mass_faces(&at(".mass.faces.tsv"),sha).and_then(|m| {
            let k = measurer_scale(&at(".json"),&occt,&m,row)?;
            let m: BTreeMap<u32,f64> = m.into_iter().map(|(e,a)| (e,k*a)).collect();
            let a: f64 = m.values().sum();
            if m.len() < row.entities[0] || file(a) { Ok(m) }
            else { Err(format!("the measurer's faces sum to {a:.9e}, {:.1e} off the file's area",rel(a,row.area))) }
        });
        let quad = mass_faces(&at(".quad.faces.tsv"),sha).map_err(|_| "no quadrature".to_string());
        Refs {occt,mass,quad}
    }
}

fn occt_faces(json: &Path,tsv: &Path,sha: &str) -> Result<BTreeMap<u32,(f64,[f64; 3])>,String> {
    let text = std::fs::read_to_string(json).map_err(|_| "no OCCT reading".to_string())?;
    let j = gcs_core::json::parse(&text).map_err(|e| format!("OCCT's reading unreadable: {e}"))?;
    if j.get("error").is_some() { return Err("OCCT failed".into()) }
    if j.get("data_sha256").map(|s| s.as_str()) != Some(sha) { return Err("OCCT's reading is of other bytes".into()) }
    let named = j.get("faces_named").map(|n| n.arr().len() == 2 && n.arr()[0].as_i64() == n.arr()[1].as_i64())
        .unwrap_or(false);
    if !(named || j.get("faces_mapped").map(|m| m.as_bool()).unwrap_or(false)) {
        return Err("OCCT's faces not named".into())
    }
    let text = std::fs::read_to_string(tsv).map_err(|_| "no OCCT face table".to_string())?;
    let mut out = BTreeMap::<u32,(f64,[f64; 3])>::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split('\t').collect();
        let x = |i: usize| f.get(i).and_then(|s| s.parse::<f64>().ok()).unwrap_or(f64::NAN);
        let e: u32 = f.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
        if e == 0 { return Err("a face OCCT names no entity".into()) }
        let (a,c) = (x(5),[x(6),x(7),x(8)]);
        // a face OCCT split: the parts' areas summed, their centroids weighed by them
        let s = out.entry(e).or_insert((0.,[0.; 3]));
        let w = s.0+a;
        s.1 = std::array::from_fn(|k| if w != 0. { (s.1[k]*s.0+c[k]*a)/w } else { c[k] });
        s.0 = w;
    }
    Ok(out)
}

/// What the measurer's areas are multiplied by to be in mm². brep-bench's measurer (at 7d811e9)
/// reads a conversion-based unit as millimetres, so an inch file's areas come out in square inches,
/// and a file of several contexts in another one's unit: the same faces, scaled. The scale is
/// read off the faces both readers measured, 1 or 25.4² where the median of their ratios is so to
/// a part in a hundred (a unit, not a fit: the readers agree to 1e-4 on the worst file); where
/// OCCT's faces are no reference, off the measurer's whole against the row's reference area where
/// it measured every face, and else 1 for a file whose only unit of length is the millimetre.
/// Otherwise none, and the measurer is not used.
fn measurer_scale(json: &Path,occt: &Result<BTreeMap<u32,(f64,[f64; 3])>,String>,mass: &BTreeMap<u32,f64>,row: &Row)
    -> Result<f64,String> {
    let unit = |r: f64| [1.,25.4*25.4].into_iter().find(|k| (k*r-1.).abs() < 1e-2);
    if let Ok(o) = occt {
        // the median of the faces' ratios, so that a face one reader misreads does not move it
        let mut r: Vec<f64> = mass.iter().filter_map(|(e,m)| o.get(e).filter(|x| x.0 > 0. && *m > 0.).map(|x| m/x.0)).collect();
        r.sort_by(f64::total_cmp);
        if let Some(&r) = r.get(r.len()/2) {
            return unit(r).ok_or_else(|| format!("the measurer reads the file at {r:.3e} of OCCT's scale"))
        }
    }
    if mass.len() == row.entities[0] && row.area > 0. {
        let r = mass.values().sum::<f64>()/row.area;
        if let Some(k) = unit(r) { return Ok(k) }
    }
    let text = std::fs::read_to_string(json).map_err(|_| "no OCCT reading".to_string())?;
    let j = gcs_core::json::parse(&text).map_err(|e| format!("OCCT's reading unreadable: {e}"))?;
    match j.get("units").map(|u| u.as_str()).unwrap_or("") {
        "mm" => Ok(1.),
        u => Err(format!("the measurer's scale in {u:?} unknown")),
    }
}

fn mass_faces(tsv: &Path,sha: &str) -> Result<BTreeMap<u32,f64>,String> {
    let text = std::fs::read_to_string(tsv).map_err(|_| "no measurer face table".to_string())?;
    let mut lines = text.lines();
    if lines.next() != Some(&format!("#\t{sha}")) { return Err("the measurer's faces are of other bytes".into()) }
    let mut out = BTreeMap::new();
    for line in lines.skip(1) {
        let (e,a) = line.split_once('\t').unwrap_or((line,""));
        let e: u32 = e.parse().unwrap_or(0);
        if e == 0 { return Err(format!("the measurer: {a}")) }
        // a face the measurer cannot measure is said by its reason, and not compared
        if let Ok(a) = a.parse::<f64>() { *out.entry(e).or_insert(0.) += a }
    }
    Ok(out)
}

/// **The bar a face's area is held to** (mm²): `TOL × Σ tol·length + REL × area + ABS`. A
/// tolerant face's boundary is known to within its edges' measured tolerances, so two readings
/// of it differ by at most the area those tolerances sweep along its edges, `Σ tol·length`
/// (`Face::slack`: one reader follows the file's pcurves, the other the edges' curves in space);
/// `REL` is the integration error of the readers' quadratures, OCCT's adaptive rule asked for
/// 1e-10 and ours; `ABS` the floor under a face of no area. The constants are measured on the
/// faces of files read right (scripts/step-corpus/README.md, "Face by face").
pub(crate) const TOL: f64 = 1.;
pub(crate) const REL: f64 = 1e-6;
pub(crate) const ABS: f64 = 1e-9;
/// How many times the two readers' difference a reading may stand from them and still be said to
/// be within their agreement (the totals' `AGREE` of scripts/step-corpus/table.py).
pub(crate) const AGREE: f64 = 3.;

pub(crate) fn face_bar(slack: f64,area: f64) -> f64 { TOL*slack+REL*area.abs()+ABS }

/// What the face-by-face check found of a file read.
#[derive(Default)]
pub(crate) struct Faces {
    /// Faces held to a reference, to two that disagree past the bar (`disputed`, read either
    /// way), and not held to any (no reader measured them).
    pub(crate) compared: usize,
    pub(crate) disputed: usize,
    pub(crate) uncompared: usize,
    /// Faces held to the third reader's quadrature (`Refs::quad`), and how many readers' areas of
    /// them it set aside (past their bars of it).
    pub(crate) checked: usize,
    pub(crate) set_aside: usize,
    /// Faces whose one reference is unchecked (`Row::unchecked`), and those of them past their
    /// bars of it: said, not failed, until a third reader measures them.
    pub(crate) unchecked: usize,
    pub(crate) unchecked_off: Vec<String>,
    /// Faces within their bars, but where the two readers agree far closer than the slack (to a
    /// hundredth of the bar) and the reading stands farther from them than `AGREE` times their
    /// difference and the quadrature's `REL`: the second, tighter line, said and not failed.
    pub(crate) outside_agreement: Vec<String>,
    /// Why the file's faces are not compared, where none are.
    pub(crate) why_not: Option<String>,
    /// The face farthest from its reference, by its error over its bar, and what it says.
    pub(crate) worst: Option<(f64,String)>,
    /// The faces past their bars.
    pub(crate) wrong: Vec<String>,
}

/// Each face of `faces` against `refs`: within its bar of the reference the two readers agree
/// on, or of either where they do not (disputed); where the third reader measured it, within its
/// bar of that and of each reader within its bar of that, the others set aside; faces of one
/// ADVANCED_FACE taken together. A face of `unchecked` held to one reader alone and past its bar
/// of it is said (`Faces::unchecked_off`), not counted wrong.
pub(crate) fn faces_judged(faces: &[Face],refs: &Refs,unchecked: &BTreeSet<u32>) -> Faces {
    let mut ours = BTreeMap::<u32,(&str,f64,f64)>::new();
    for f in faces.iter().filter(|f| f.entity != 0) {
        let s = ours.entry(f.entity).or_insert((f.kind,0.,0.));
        s.1 += f.area;
        s.2 += f.slack;
    }
    let mut out = Faces::default();
    let ids: BTreeSet<u32> = ours.keys().copied().collect();
    let occt = match &refs.occt {
        Ok(m) if m.keys().copied().collect::<BTreeSet<_>>() == ids => Ok(m),
        Ok(_) => Err("OCCT's faces are not the ones read".to_string()),
        Err(e) => Err(e.clone()),
    };
    let mass = refs.mass.as_ref().map_err(|e| e.clone());
    let quad = refs.quad.as_ref().ok();
    if let (Err(a),Err(b),None) = (&occt,&mass,quad) { out.why_not = Some(format!("{a}; {b}")) }
    for (e,(kind,area,slack)) in &ours {
        let o = occt.as_ref().ok().map(|m| m[e].0);
        let m = mass.as_ref().ok().and_then(|m| m.get(e).copied());
        let q = quad.and_then(|m| m.get(e).copied());
        let bar = face_bar(*slack,*area);
        let off = |r: f64| (area-r).abs()/bar;
        let (err,disputed,alone) = match (q,o,m) {
            (Some(q),o,m) => {
                // the quadrature checks the readers: one past its bar of it is set aside
                let agree: Vec<f64> = [o,m].into_iter().flatten().filter(|r| (r-q).abs() <= bar).collect();
                out.checked += 1;
                out.set_aside += o.into_iter().chain(m).count()-agree.len();
                (agree.into_iter().map(off).fold(off(q),f64::max),false,false)
            }
            (None,None,None) => { out.uncompared += 1; continue }
            (None,Some(r),None) | (None,None,Some(r)) => (off(r),false,true),
            (None,Some(a),Some(b)) if (a-b).abs() <= bar => (off(a).max(off(b)),false,false),
            (None,Some(a),Some(b)) => (off(a).min(off(b)),true,false),
        };
        out.compared += 1;
        out.disputed += disputed as usize;
        let open = alone && unchecked.contains(e);
        out.unchecked += unchecked.contains(e) as usize;
        let say = || format!("face #{e} ({kind}): area {area:.9e}, OCCT {}, measurer {}{}, bar {bar:.1e} (slack {slack:.1e}){}",
            o.map_or("-".into(),|x| format!("{x:.9e}")),m.map_or("-".into(),|x| format!("{x:.9e}")),
            q.map_or(String::new(),|x| format!(", quadrature {x:.9e}")),
            if disputed { ", disputed" } else if open { ", its one reference unchecked" } else { "" });
        if !(err <= 1.) { if open { out.unchecked_off.push(say()) } else { out.wrong.push(say()) } }
        else if let (None,Some(a),Some(b)) = (q,o,m) {
            let (mid,apart) = (0.5*(a+b),(a-b).abs());
            if apart <= 0.01*bar && (area-mid).abs() > (AGREE*apart).max(REL*area.abs()) {
                out.outside_agreement.push(format!("{}, {:.1e} of its bar off the readers, who agree to {:.1e} of it",say(),
                    (area-mid).abs()/bar,apart/bar))
            }
        }
        if out.worst.as_ref().map_or(true,|w| err > w.0) { out.worst = Some((err,say())) }
    }
    out
}

/// Why a read is wrong against its row, if it is: its totals, then each of its faces.
pub(crate) fn wrong(row: &Row,read: &Read) -> Option<String> {
    match totals(row,&read.outcome) {
        Totals::Wrong(w) => Some(w),
        Totals::Alt => None,
        Totals::Right => {
            let Outcome::Read {faces,..} = &read.outcome else { return None };
            let f = faces_judged(faces,&read.refs,&row.unchecked);
            (!f.wrong.is_empty()).then(|| format!("{} of {} faces past their bars: {}",f.wrong.len(),f.compared,
                f.wrong.iter().take(4).cloned().collect::<Vec<_>>().join("; ")))
        }
    }
}

/// A read's totals against its row.
pub(crate) enum Totals {
    Right,
    /// Right only as the row's second reading (`alt`: a disputed file's measurer's, a mutant's
    /// unedited source's), whose faces the oracle's tables do not give: its faces are not held.
    Alt,
    Wrong(String),
}

/// **What a file determines of its totals** (relative), from its reading's own slack: its area no
/// better than the sum of its faces' slacks, `S = Σ_faces Σ max(tol, u)·length` (each face's
/// boundary is known to within it, `face_bar`), and its volume no better than that slack swept
/// over the solid's reach `R` from the point the flux is taken about, `R·S/3`: the flux `∮ X·n dA/3`
/// moves by `|X|/3` for each unit of area the slack moves, and a boundary whose edges stand off
/// their faces does not close, so its volume moves with that point by `|d|·|∮ n dA|/3 ≤ |d|·S/3`
/// for a move `d` (CADGenBench 212 moved 10 mm: 1.0e-6). `(S/A, R·S/(3V))`.
pub(crate) fn determined(read: &Outcome) -> (f64,f64) {
    let Outcome::Read {volume,area,faces,reach,..} = read else { return (0.,0.) };
    let s: f64 = faces.iter().map(|f| f.slack).sum();
    (reach*s/(3.*volume.abs()),s/area.abs())
}

/// The bars a read's totals are held to: the row's (its reference's own error — the case's grade,
/// three times the two readers' difference, one reader's — floored as `table.py` says), or what
/// the file determines (`determined`) where that is wider: a reading cannot be held closer to a
/// reference than the file holds its own solid. `(volume, area)`.
pub(crate) fn bars(row: &Row,read: &Outcome) -> (f64,f64) {
    let (dv,da) = determined(read);
    (row.bar_volume.max(dv),row.bar_area.max(da))
}

pub(crate) fn totals(row: &Row,read: &Outcome) -> Totals {
    let Outcome::Read {solids,entities,volume,area,..} = read else { return Totals::Right };
    if row.refuse() { return Totals::Wrong(format!("read {solids} solids, volume {volume:.12e} area {area:.12e}, from a file that represents none")) }
    if *solids != row.solids { return Totals::Wrong(format!("{solids} solids, the file has {}",row.solids)) }
    if *entities != row.entities {
        return Totals::Wrong(format!("faces, edges, vertices read {entities:?}, the file's solids hold {:?}",row.entities))
    }
    let (bv,ba) = bars(row,read);
    let (dv,da) = (rel(*volume,row.volume),rel(*area,row.area));
    if dv <= bv && da <= ba { return Totals::Right }
    // a disputed file may be read to either reading
    if let Some((v,a)) = row.alt {
        if rel(*volume,v) <= bv && rel(*area,a) <= ba { return Totals::Alt }
    }
    if !(dv <= bv) { return Totals::Wrong(format!("volume {volume:.12e} is {dv:.1e} off {:.12e} ({}), the bar {bv:.1e}",row.volume,row.from)) }
    Totals::Wrong(format!("area {area:.12e} is {da:.1e} off {:.12e} ({}), the bar {ba:.1e}",row.area,row.from))
}

/// Whether a read right is also within its row's own bar (the readers' agreement, or the case's
/// grade), not only within what the file determines: the second, tighter line, said and not
/// failed (two readers agreeing far inside the file's slack may share a convention its writer
/// meant, pcurves over curves).
pub(crate) fn agreed(row: &Row,read: &Outcome) -> bool {
    let Outcome::Read {volume,area,..} = read else { return true };
    let within = |v: f64,a: f64| rel(*volume,v) <= row.bar_volume && rel(*area,a) <= row.bar_area;
    within(row.volume,row.area) || row.alt.map_or(false,|(v,a)| within(v,a))
}

/// Where a file stands against its account: its class in the progress report, and whether it
/// keeps the stage from its finish (`the_corpus_reads`).
pub(crate) fn standing(row: &Row,r: &Read) -> (&'static str,bool) {
    // the account first: a dispute it settled (a reader shown to misread) is no excuse
    let gap = row.verdict == "gap";
    match &r.outcome {
        Outcome::Refused(_) if row.refuse() => ("nosolid",false),
        Outcome::Refused(_) if row.verdict == "defective" => ("defective",false),
        Outcome::Refused(_) if row.verdict == "assembly" => ("assembly",false),
        Outcome::Refused(_) if gap => ("gap",true),
        Outcome::Refused(_) if row.verdict == "open" => ("open",true),
        Outcome::Refused(_) if row.from == "disputed" => ("disputed",false),
        Outcome::Refused(_) => ("refused?",true),
        Outcome::Read {..} if wrong(row,r).is_some() => (if gap { "gap" } else if row.verdict == "open" { "open" } else { "WRONG?" },true),
        // a file the account says is no solid's boundary is finished by its refusal: a reading
        // of it is no pass until the account says what the file's own geometry holds
        Outcome::Read {..} if row.verdict == "defective" => ("read defective",true),
        Outcome::Read {..} => ("read",false),
        Outcome::Panicked(_) => ("PANIC",true),
        Outcome::Hung(_) => ("HUNG",true),
        Outcome::Missing | Outcome::Stale(_) => ("missing",true),
    }
}

/// The source a corpus path is of: `cadgenbench`, `brep-bench/<kernel>`, `commercial/<source>`,
/// `variants/<habit>`.
fn source(path: &str) -> String {
    let p: Vec<&str> = path.split('/').collect();
    if p[0] == "cadgenbench" || p.len() < 2 { p[0].to_string() } else { format!("{}/{}",p[0],p[1]) }
}

/// The progress report: a line a file.
pub(crate) fn report(rows: &[Row],reads: &[Read]) {
    let mut tally = std::collections::BTreeMap::<&str,usize>::new();
    println!("{:<64} {:<8} {:>9} {:>9} {:>11} {:>8} {:>7}  why / witness; the worst face","file","outcome","dV","dA",
        "faces","worst","s");
    for (row,r) in rows.iter().zip(reads) {
        let judged = match &r.outcome { Outcome::Read {faces,..} => Some((faces.len(),faces_judged(faces,&r.refs,&row.unchecked))),_ => None };
        let (word,dv,da,why) = match &r.outcome {
            Outcome::Missing => ("missing","".into(),"".into(),String::new()),
            Outcome::Stale(sha) => ("stale","".into(),"".into(),format!("DATA's sha256 {sha}")),
            Outcome::Refused(f) => (standing(row,r).0,"".into(),"".into(),format!("{:?} {f}",f.why)),
            Outcome::Panicked(m) => ("PANIC","".into(),"".into(),m.clone()),
            Outcome::Hung(s) => ("HUNG","".into(),"".into(),format!("not done in {s} s")),
            Outcome::Read {volume,area,..} => match wrong(row,r) {
                Some(w) => ("WRONG",format!("{:.1e}",rel(*volume,row.volume)),format!("{:.1e}",rel(*area,row.area)),w),
                // `~` marks a read within what the file determines but outside its row's own bar
                None => (if row.verdict == "defective" { "read def" } else if agreed(row,&r.outcome) { "read" } else { "read~" },
                    format!("{:.1e}",rel(*volume,row.volume)),format!("{:.1e}",rel(*area,row.area)),String::new()),
            },
        };
        *tally.entry(word).or_default() += 1;
        let (n,worst,face) = match &judged {
            Some((n,f)) => (format!("{}/{n}",f.compared),f.worst.as_ref().map_or("".into(),|w| format!("{:.1e}",w.0)),
                f.worst.as_ref().map_or(f.why_not.clone().unwrap_or_default(),|w| w.1.clone())),
            None => (String::new(),String::new(),String::new()),
        };
        let why: String = why.chars().take(110).collect();
        let face: String = face.chars().take(160).collect();
        let account = if row.verdict.is_empty() { String::new() }
            else { format!("  [{}: {}]",row.verdict,row.account.chars().take(140).collect::<String>()) };
        println!("{:<64} {word:<8} {dv:>9} {da:>9} {n:>11} {worst:>8} {:>7.2}  {why}{}{face}{account}",row.path,r.seconds,
            if why.is_empty() || face.is_empty() { "" } else { "; " });
    }
    println!("{tally:?}");
}

/// The account, by source and by where each file stands (`standing`): what reads, what is refused
/// and why that finishes it or not, the files read within what they determine but outside their
/// rows' own bars (`read~`: the readers agree closer than the file's slack, and the reading does
/// not), and every file the account leaves unexplained.
pub(crate) fn account(rows: &[Row],reads: &[Read]) {
    let mut by = BTreeMap::<String,BTreeMap<&str,usize>>::new();
    let mut classes = BTreeSet::<&str>::new();
    let mut loose = Vec::new();
    for (row,r) in rows.iter().zip(reads) {
        let (mut class,_) = standing(row,r);
        if class == "read" && !agreed(row,&r.outcome) {
            class = "read~";
            let (dv,da) = determined(&r.outcome);
            if let Outcome::Read {volume,area,..} = &r.outcome {
                loose.push(format!("{}: dV {:.1e} dA {:.1e}, its row's bars {:.0e} {:.0e} ({}), the file's slack {dv:.1e} {da:.1e}",
                    row.path,rel(*volume,row.volume),rel(*area,row.area),row.bar_volume,row.bar_area,row.from))
            }
        }
        classes.insert(class);
        *by.entry(source(&row.path)).or_default().entry(class).or_default() += 1;
    }
    let classes: Vec<&str> = classes.into_iter().collect();
    print!("{:<26}","source");
    for c in &classes { print!(" {c:>14}") }
    println!();
    let mut total = BTreeMap::<&str,usize>::new();
    for (s,t) in &by {
        print!("{s:<26}");
        for c in &classes {
            let n = t.get(c).copied().unwrap_or(0);
            *total.entry(c).or_default() += n;
            print!(" {:>14}",if n > 0 { n.to_string() } else { String::new() });
        }
        println!();
    }
    print!("{:<26}","all");
    for c in &classes { print!(" {:>14}",total[c]) }
    println!();
    // every row whose bars what the file determines widens, before and after
    let wide: Vec<String> = rows.iter().zip(reads).filter_map(|(row,r)| {
        let (bv,ba) = bars(row,&r.outcome);
        (bv > row.bar_volume || ba > row.bar_area).then(|| format!("{}: volume {:.1e} -> {bv:.1e}, area {:.1e} -> {ba:.1e}",
            row.path,row.bar_volume,row.bar_area))
    }).collect();
    println!("bars widened to what the file determines ({}):",wide.len());
    for w in &wide { println!("  {w}") }
    println!("read within what the file determines, outside its row's own bar ({}):",loose.len());
    for l in &loose { println!("  {l}") }
    let unexplained: Vec<&str> = rows.iter().zip(reads).filter(|(row,r)| matches!(standing(row,r).0,"refused?" | "WRONG?"))
        .map(|(row,_)| row.path.as_str()).collect();
    println!("refused or wrong with no account ({}): {}",unexplained.len(),unexplained.join(" "));
}

/// How many files read were compared face by face, and why the others were not (a tally); every
/// face of every file read, with its references, to `STEP_CORPUS_FACES_OUT` where it names a file
/// (the measure the bar's constants are set by).
pub(crate) fn face_census(rows: &[Row],reads: &[Read]) {
    let mut why = BTreeMap::<String,usize>::new();
    let (mut files,mut faces,mut uncompared,mut disputed) = (0,0,0,0);
    let (mut checked,mut set_aside,mut unchecked,mut open,mut apart) = (0,0,0,Vec::new(),Vec::new());
    let mut dump = String::from("path\tfrom\ttotals\tentity\tkind\tarea\tslack\tocct\tmass\tmeasured\n");
    for (row,r) in rows.iter().zip(reads) {
        let Outcome::Read {faces: ours,..} = &r.outcome else { continue };
        let f = faces_judged(ours,&r.refs,&row.unchecked);
        checked += f.checked;
        set_aside += f.set_aside;
        unchecked += f.unchecked;
        for o in &f.unchecked_off { open.push(format!("{}: {o}",row.path)) }
        for o in &f.outside_agreement { apart.push(format!("{}: {o}",row.path)) }
        if f.compared > 0 { files += 1 } else { *why.entry(f.why_not.clone().unwrap_or_else(|| "no face measured".into())).or_default() += 1 }
        faces += f.compared;
        uncompared += f.uncompared;
        disputed += f.disputed;
        let t = match totals(row,&r.outcome) { Totals::Right => "right",Totals::Alt => "alt",Totals::Wrong(_) => "wrong" };
        for x in ours {
            let o = r.refs.occt.as_ref().ok().and_then(|m| m.get(&x.entity)).map_or(String::new(),|o| format!("{:e}",o.0));
            let m = r.refs.mass.as_ref().ok().and_then(|m| m.get(&x.entity)).map_or(String::new(),|m| format!("{m:e}"));
            dump += &format!("{}\t{}\t{t}\t{}\t{}\t{:e}\t{:e}\t{o}\t{m}\t{:e}\n",row.path,row.from,x.entity,x.kind,x.area,x.slack,
                x.measured);
        }
    }
    let read = reads.iter().filter(|r| matches!(r.outcome,Outcome::Read {..})).count();
    eprintln!("face by face: {files} of {read} files read compared, {faces} faces ({disputed} disputed), {uncompared} \
        faces no reader measured; files not compared: {why:?}");
    eprintln!("{checked} faces held to a third reader's quadrature ({set_aside} readers' areas of them set aside past \
        their bars); {unchecked} faces with one unchecked reference, {} of them past their bars of it, not failed:",open.len());
    for o in &open { eprintln!("  {o}") }
    eprintln!("{} faces within their bars but outside the readers' agreement (they agree to a hundredth of the bar, the \
        reading is farther than {AGREE} times their difference and the quadrature's):",apart.len());
    for o in &apart { eprintln!("  {o}") }
    if let Ok(out) = std::env::var("STEP_CORPUS_FACES_OUT") { std::fs::write(&out,dump).unwrap_or_else(|e| panic!("{out}: {e}")) }
}

/// A file missing or stale is said, and is a failure only where the table should describe it.
pub(crate) fn unread(rows: &[Row],reads: &[Read]) -> Vec<String> {
    rows.iter().zip(reads).filter_map(|(row,r)| match &r.outcome {
        Outcome::Stale(sha) => Some(format!("{}: its DATA's sha256 is {sha}, the table's {} \
            (regenerate the table: scripts/step-corpus/table.py)",row.path,row.data_sha256)),
        _ => None,
    }).collect()
}

#[test]
#[ignore = "a tool: STEP_CORPUS names the corpus outside the repository (scripts/step-corpus/README.md)"]
fn nothing_is_read_wrong() {
    let Some((rows,reads)) = outcomes() else { return };
    let missing = reads.iter().filter(|r| matches!(r.outcome,Outcome::Missing)).count();
    if missing > 0 { eprintln!("{missing} files of the table are not in the corpus, and are not read") }
    let mut bad = unread(rows,reads);
    bad.extend(rows.iter().zip(reads).filter_map(|(row,r)| wrong(row,r).map(|w| format!("{}: {w}",row.path))));
    let refused = reads.iter().filter(|r| matches!(r.outcome,Outcome::Refused(_))).count();
    let read = reads.iter().filter(|r| matches!(r.outcome,Outcome::Read {..})).count();
    eprintln!("{read} read, {refused} refused, {} wrong or stale",bad.len());
    face_census(rows,reads);
    assert!(bad.is_empty(),"read wrong:\n{}",bad.join("\n"));
}

#[test]
#[ignore = "a tool: STEP_CORPUS names the corpus outside the repository (scripts/step-corpus/README.md)"]
fn nothing_panics() {
    let Some((rows,reads)) = outcomes() else { return };
    let panics: Vec<String> = rows.iter().zip(reads).filter_map(|(row,r)| match &r.outcome {
        Outcome::Panicked(m) => Some(format!("{}: {m}",row.path)),
        _ => None,
    }).collect();
    assert!(panics.is_empty(),"the reader panicked:\n{}",panics.join("\n"));
}

#[test]
#[ignore = "a tool: STEP_CORPUS names the corpus outside the repository; the stage's finish line, \
    failing until the reader reads every file"]
fn the_corpus_reads() {
    let Some((rows,reads)) = outcomes() else { return };
    report(rows,reads);
    account(rows,reads);
    // a file is finished when it is read right, or refused and its account says a refusal is
    // right: it represents no solid, the readers dispute it, its boundary is no solid's
    // (`defective`), or it is an assembly; the reader's gaps are not finished
    let short = rows.iter().zip(reads).filter(|(row,r)| standing(row,r).1).count();
    assert!(short == 0,"{short} of {} files not read to their reference",rows.len());
}

/// SHA-256 (FIPS 180-4), for knowing the corpus's bytes are the ones the table describes.
fn sha256(data: &[u8]) -> [u8; 32] {
    const K: [u32; 64] = [
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
    let mut h: [u32; 8] = [0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
    let mut msg = data.to_vec();
    msg.push(0x80);
    while msg.len()%64 != 56 { msg.push(0) }
    msg.extend_from_slice(&((data.len() as u64)*8).to_be_bytes());
    for block in msg.chunks(64) {
        let mut w = [0u32; 64];
        for i in 0..16 { w[i] = u32::from_be_bytes([block[4*i],block[4*i+1],block[4*i+2],block[4*i+3]]) }
        for i in 16..64 {
            let s0 = w[i-15].rotate_right(7)^w[i-15].rotate_right(18)^(w[i-15] >> 3);
            let s1 = w[i-2].rotate_right(17)^w[i-2].rotate_right(19)^(w[i-2] >> 10);
            w[i] = w[i-16].wrapping_add(s0).wrapping_add(w[i-7]).wrapping_add(s1);
        }
        let mut s = h;
        for i in 0..64 {
            let s1 = s[4].rotate_right(6)^s[4].rotate_right(11)^s[4].rotate_right(25);
            let ch = (s[4] & s[5])^(!s[4] & s[6]);
            let t1 = s[7].wrapping_add(s1).wrapping_add(ch).wrapping_add(K[i]).wrapping_add(w[i]);
            let s0 = s[0].rotate_right(2)^s[0].rotate_right(13)^s[0].rotate_right(22);
            let maj = (s[0] & s[1])^(s[0] & s[2])^(s[1] & s[2]);
            let t2 = s0.wrapping_add(maj);
            s = [t1.wrapping_add(t2),s[0],s[1],s[2],s[3].wrapping_add(t1),s[4],s[5],s[6]];
        }
        for i in 0..8 { h[i] = h[i].wrapping_add(s[i]) }
    }
    let mut out = [0u8; 32];
    for i in 0..8 { out[4*i..4*i+4].copy_from_slice(&h[i].to_be_bytes()) }
    out
}

fn hex(b: &[u8]) -> String { b.iter().map(|x| format!("{x:02x}")).collect() }

#[test]
fn the_tables_hash_is_sha256() {
    assert_eq!(hex(&sha256(b"abc")),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    assert_eq!(hex(&sha256(b"")),"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    let long = vec![b'a'; 1000];
    assert_eq!(hex(&sha256(&long)),"41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
}

#[test]
fn the_table_is_whole() {
    let rows = table();
    assert!(rows.len() > 32,"the reference table has {} rows",rows.len());
    for r in &rows {
        assert_eq!(r.data_sha256.len(),64,"{}",r.path);
        if r.refuse() { continue }
        assert!(r.volume.is_finite() && r.area.is_finite() && r.volume > 0. && r.area > 0.,"{}: no reference",r.path);
        assert!(r.bar_volume > 0. && r.bar_area > 0. && r.bar_volume < 0.05 && r.bar_area < 0.05,"{}: bars {} {}",
            r.path,r.bar_volume,r.bar_area);
        assert!(r.solids >= 1 && r.entities[0] > 0,"{}: {:?}",r.path,r.entities);
    }
    // every account a verdict of the vocabulary, and one that excuses a file a witness and a why
    for r in &rows {
        assert!(["","gap","open","defective","assembly","nosolid","disputed","reference","bar","unchecked"].contains(&r.verdict.as_str()),
            "{}: verdict {:?}",r.path,r.verdict);
        if !r.verdict.is_empty() { assert!(r.account.contains('#') && r.account.len() > 20,"{}: an account with no witness",r.path) }
        assert!(r.verdict != "unchecked" || !r.unchecked.is_empty(),"{}: unchecked, but no face named",r.path);
    }
}
