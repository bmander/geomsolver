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
//!   read and none twice-counted, and their volume and area within the bar. Holds at every
//!   commit, the reader refusing everything included.
//! * `nothing_panics`: no read panics (`catch_unwind`); a panic names its file.
//! * `the_corpus_reads`: every file read, none refused. The stage's finish line: fails until the
//!   reader is done, and its table is the progress report.
//!
//! ```sh
//! STEP_CORPUS=/path/to/corpus cargo test --manifest-path rust/Cargo.toml -p gcs-core -- \
//!     step_corpus:: --ignored --nocapture
//! ```
//! `STEP_CORPUS_ONLY=cadgenbench/2` reads only the files whose path starts so;
//! `STEP_CORPUS_THREADS` how many at once (the machine's parallelism by default).
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::step_read::{read,Given,Refusal};
use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::OnceLock;
use std::time::Instant;

/// The committed table, read when the tests run (a table regenerated needs no rebuild).
const TABLE: &str = concat!(env!("CARGO_MANIFEST_DIR"),"/tests/step_corpus/reference.tsv");

/// A file's row of the reference table.
struct Row {
    path: String,
    data_sha256: String,
    solids: usize,
    /// The entities its solids hold: ADVANCED_FACEs, EDGE_CURVEs and VERTEX_POINTs, each once.
    entities: [usize; 3],
    volume: f64,
    area: f64,
    bar_volume: f64,
    bar_area: f64,
    /// Where the reference comes from: `closed` (the case's closed form), `occt`, or `source:…`.
    from: String,
}

fn table() -> Vec<Row> {
    let path = std::env::var("STEP_CORPUS_TABLE").unwrap_or_else(|_| TABLE.to_string());
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let head: Vec<&str> = lines.next().expect("the table's header").split('\t').collect();
    let col = |name: &str| head.iter().position(|h| *h == name).unwrap_or_else(|| panic!("no column {name}"));
    let (path,sha,solids,faces,edges,vertices) = (col("path"),col("data_sha256"),col("solids"),
        col("step_faces"),col("step_edges"),col("step_vertices"));
    let (v,a,bv,ba,from) = (col("ref_volume"),col("ref_area"),col("bar_volume"),col("bar_area"),col("ref_from"));
    lines.filter(|l| !l.is_empty()).map(|l| {
        let f: Vec<&str> = l.split('\t').collect();
        let n = |i: usize| f[i].parse::<usize>().unwrap_or_else(|_| panic!("{}: {:?} is no count",f[path],f[i]));
        let x = |i: usize| f[i].parse::<f64>().unwrap_or(f64::NAN);
        Row { path: f[path].to_string(),data_sha256: f[sha].to_string(),solids: n(solids),
            entities: [n(faces),n(edges),n(vertices)],volume: x(v),area: x(a),bar_volume: x(bv),bar_area: x(ba),
            from: f[from].to_string() }
    }).collect()
}

/// What reading a file came to.
enum Outcome {
    /// Not in the corpus.
    Missing,
    /// In the corpus, but not the bytes the table describes.
    Stale(String),
    Refused(Refusal),
    Panicked(String),
    Read { solids: usize,entities: [usize; 3],volume: f64,area: f64 },
}

struct Read { outcome: Outcome,seconds: f64 }

/// The entities `given` read, each once (0, an entity the reader laid itself, not counted).
fn entities(given: &[Given]) -> [usize; 3] {
    let distinct = |ids: &mut dyn Iterator<Item=u32>| ids.filter(|&e| e != 0).collect::<BTreeSet<_>>().len();
    [distinct(&mut given.iter().flat_map(|g| g.faces.iter().copied())),
        distinct(&mut given.iter().flat_map(|g| g.edges.iter().copied())),
        distinct(&mut given.iter().flat_map(|g| g.vertices.iter().copied()))]
}

fn read_one(root: &PathBuf,row: &Row) -> Outcome {
    let Ok(bytes) = std::fs::read(root.join(&row.path)) else { return Outcome::Missing };
    let data = bytes.windows(6).position(|w| w == b"\nDATA;").unwrap_or(0);
    let sha = hex(&sha256(&bytes[data..]));
    if sha != row.data_sha256 { return Outcome::Stale(sha) }
    let text = String::from_utf8_lossy(&bytes).into_owned();
    let got = std::panic::catch_unwind(|| read(&text).map(|given| {
        let v: f64 = given.iter().map(|g| volume(&g.brep)).sum();
        let a: f64 = given.iter().map(|g| g.brep.faces.iter().map(|f| area(&g.brep,f)).sum::<f64>()).sum();
        (given.len(),entities(&given),v,a)
    }));
    match got {
        Ok(Ok((solids,entities,volume,area))) => Outcome::Read {solids,entities,volume,area},
        Ok(Err(r)) => Outcome::Refused(r),
        Err(p) => Outcome::Panicked(p.downcast_ref::<&str>().map(|s| s.to_string())
            .or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_else(|| "a panic".into())),
    }
}

/// Every file of the table read once, in parallel, the largest first; `None` where there is no
/// corpus.
fn outcomes() -> Option<&'static (Vec<Row>,Vec<Read>)> {
    static ALL: OnceLock<Option<(Vec<Row>,Vec<Read>)>> = OnceLock::new();
    ALL.get_or_init(|| {
        let Ok(root) = std::env::var("STEP_CORPUS") else {
            eprintln!("STEP_CORPUS is not set: no corpus to read (scripts/step-corpus/README.md)");
            return None
        };
        let root = PathBuf::from(root);
        let only = std::env::var("STEP_CORPUS_ONLY").unwrap_or_default();
        let rows: Vec<Row> = table().into_iter().filter(|r| r.path.starts_with(&only)).collect();
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
                let outcome = read_one(&root,&rows[i]);
                done.lock().unwrap()[i] = Some(Read {outcome,seconds: start.elapsed().as_secs_f64()});
            });
        });
        eprintln!("{} files of {} read in {:.1} s, {threads} at once",rows.len(),root.display(),
            clock.elapsed().as_secs_f64());
        let reads = done.into_inner().unwrap().into_iter().map(|r| r.unwrap()).collect();
        Some((rows,reads))
    }).as_ref()
}

fn rel(got: f64,want: f64) -> f64 { (got-want).abs()/want.abs() }

/// Why a read is wrong against its row, if it is.
fn wrong(row: &Row,read: &Outcome) -> Option<String> {
    let Outcome::Read {solids,entities,volume,area} = read else { return None };
    if *solids != row.solids { return Some(format!("{solids} solids, the file has {}",row.solids)) }
    if *entities != row.entities {
        return Some(format!("faces, edges, vertices read {entities:?}, the file's solids hold {:?}",row.entities))
    }
    let (dv,da) = (rel(*volume,row.volume),rel(*area,row.area));
    if !(dv <= row.bar_volume) { return Some(format!("volume {volume:.12e} is {dv:.1e} off {:.12e} ({}), the bar {:.0e}",row.volume,row.from,row.bar_volume)) }
    if !(da <= row.bar_area) { return Some(format!("area {area:.12e} is {da:.1e} off {:.12e} ({}), the bar {:.0e}",row.area,row.from,row.bar_area)) }
    None
}

/// The progress report: a line a file.
fn report(rows: &[Row],reads: &[Read]) {
    let mut tally = std::collections::BTreeMap::<&str,usize>::new();
    println!("{:<64} {:<8} {:>9} {:>9} {:>7}  why / witness","file","outcome","dV","dA","s");
    for (row,r) in rows.iter().zip(reads) {
        let (word,dv,da,why) = match &r.outcome {
            Outcome::Missing => ("missing","".into(),"".into(),String::new()),
            Outcome::Stale(sha) => ("stale","".into(),"".into(),format!("DATA's sha256 {sha}")),
            Outcome::Refused(f) => ("refused","".into(),"".into(),format!("{:?} {f}",f.why)),
            Outcome::Panicked(m) => ("PANIC","".into(),"".into(),m.clone()),
            Outcome::Read {volume,area,..} => match wrong(row,&r.outcome) {
                Some(w) => ("WRONG",format!("{:.1e}",rel(*volume,row.volume)),format!("{:.1e}",rel(*area,row.area)),w),
                None => ("read",format!("{:.1e}",rel(*volume,row.volume)),format!("{:.1e}",rel(*area,row.area)),String::new()),
            },
        };
        *tally.entry(word).or_default() += 1;
        let why: String = why.chars().take(110).collect();
        println!("{:<64} {word:<8} {dv:>9} {da:>9} {:>7.2}  {why}",row.path,r.seconds);
    }
    println!("{tally:?}");
}

/// A file missing or stale is said, and is a failure only where the table should describe it.
fn unread(rows: &[Row],reads: &[Read]) -> Vec<String> {
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
    bad.extend(rows.iter().zip(reads).filter_map(|(row,r)| wrong(row,&r.outcome).map(|w| format!("{}: {w}",row.path))));
    let refused = reads.iter().filter(|r| matches!(r.outcome,Outcome::Refused(_))).count();
    let read = reads.iter().filter(|r| matches!(r.outcome,Outcome::Read {..})).count();
    eprintln!("{read} read, {refused} refused, {} wrong or stale",bad.len());
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
    let short = rows.iter().zip(reads).filter(|(row,r)| !matches!(r.outcome,Outcome::Read {..})
        || wrong(row,&r.outcome).is_some()).count();
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
    assert_eq!(hex(&sha256(b"abc")),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9c3cdf3ab12a1a5b5a");
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
        assert!(r.volume.is_finite() && r.area.is_finite() && r.volume > 0. && r.area > 0.,"{}: no reference",r.path);
        assert!(r.bar_volume > 0. && r.bar_area > 0. && r.bar_volume < 0.01 && r.bar_area < 0.01,"{}: bars {} {}",
            r.path,r.bar_volume,r.bar_area);
        assert!(r.solids >= 1 && r.entities.iter().all(|&n| n > 0),"{}: {:?}",r.path,r.entities);
    }
}
