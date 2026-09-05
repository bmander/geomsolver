//! `solventc` — check a Solvent document without a browser.
//!
//! Parse, elaborate, solve, diagnose, report.  It is the first way to check a drawing in CI, and
//! the natural home for module resolution when that arrives.
//!
//! **The CLI invents no wording of its own.**  A per-document line is `diagnose::summary`, a
//! culprit is `io::describe`, and `--json` is `report::solve_result_json` /
//! `report::diagnosis_json`.  So this and the app cannot come to describe the same drawing
//! differently — which is the same bargain the app already strikes with the core, said again for
//! a second front end.
//!
//! **The core takes text and has no filesystem.**  It has no dependencies and it runs in wasm, so
//! it cannot open a file and must not learn how.  Whatever `use "part.sv"` turns out to mean, the
//! thing that resolves it is the one with a working directory: this binary, handing the core
//! either fully-resolved text or a list of named sources.  So the seam is here from the start — a
//! `Source { name, text }` list in and a report per source out — rather than a loop over `argv`
//! that would have to be turned inside out later.

use std::process::ExitCode;

use gcs_core::constraints::SpecKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::json::Json;
use gcs_core::model::Sketch;
use gcs_core::program::{elaborate, Elaborated, Severity};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::{line_col, parse};
use gcs_core::{io, json, report};

const USAGE: &str = "\
solventc — check a Solvent document

    solventc [OPTIONS] FILE...

    --json              structured output instead of the text report
    --where NAME        where a name landed: its own numbers and everything under it
                        (repeatable; the whole table in --json when none is given)
    --no-diagnose       solve only; skip the diagnosis
    --allow-unsolved    a document that does not solve is not a failure
    -o, --output PATH   write an SVG (one file, so one document)
    --stl PATH          write a solid as binary STL (one file, so one document)
    --gltf PATH         write a solid as binary glTF: every face a named node
    --solid NAME        which solid --stl writes; the only one, when there is only one
    --width PX          the SVG's page width in pixels (default 800)
    -h, --help          this

Exit codes: 0 every document elaborated and solved; 1 a document failed to parse or
elaborate; 2 a document elaborated but did not solve.
";

/// One document, as the core sees it: a name to report against and the text itself.
///
/// The unit an importer would hand over, which is why it exists before there is one.
struct Source {
    name: String,
    text: String,
}

struct Opts {
    json: bool,
    /// `--where NAME` — the names a reader asked about.  Empty asks about nothing in the text
    /// report and about everything in `--json`, which is the difference between a terminal and
    /// a file something else reads.
    wanted: Vec<String>,
    no_diagnose: bool,
    allow_unsolved: bool,
    output: Option<String>,
    /// `--stl PATH` — the one output of a drawing that is not a picture, and the reason a
    /// printer can be given a part at all.
    stl: Option<String>,
    /// `--gltf PATH` — the object as a viewer opens it, every face named.
    gltf: Option<String>,
    solid: Option<String>,
    /// An SVG has no screen, so the export must choose a `unit` — the world length of one screen
    /// pixel, which every constant size goes through.  A page width fixes it.
    width: f64,
}

impl Default for Opts {
    fn default() -> Opts {
        Opts {
            json: false,
            wanted: Vec::new(),
            no_diagnose: false,
            allow_unsolved: false,
            output: None,
            stl: None,
            gltf: None,
            solid: None,
            width: 800.0,
        }
    }
}

fn main() -> ExitCode {
    let mut opts = Opts::default();
    let mut paths: Vec<String> = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--output" | "-o" => match args.next() {
                Some(p) => opts.output = Some(p),
                None => {
                    eprintln!("solventc: --output needs a path");
                    return ExitCode::from(2);
                }
            },
            "--width" => match args.next().and_then(|v| v.parse::<f64>().ok()) {
                Some(w) if w.is_finite() && w > 0.0 => opts.width = w,
                _ => {
                    eprintln!("solventc: --width needs a page width in pixels");
                    return ExitCode::from(2);
                }
            },
            "--gltf" | "--glb" => match args.next() {
                Some(p) => opts.gltf = Some(p),
                None => {
                    eprintln!("solventc: --gltf needs a path");
                    return ExitCode::from(2);
                }
            },
            "--stl" => match args.next() {
                Some(p) => opts.stl = Some(p),
                None => {
                    eprintln!("solventc: --stl needs a path");
                    return ExitCode::from(2);
                }
            },
            "--solid" => match args.next() {
                Some(n) => opts.solid = Some(n),
                None => {
                    eprintln!("solventc: --solid needs a name");
                    return ExitCode::from(2);
                }
            },
            "--where" => match args.next() {
                Some(n) => opts.wanted.push(n),
                None => {
                    eprintln!("solventc: --where needs a name");
                    return ExitCode::from(2);
                }
            },
            "--json" => opts.json = true,
            "--no-diagnose" => opts.no_diagnose = true,
            "--allow-unsolved" => opts.allow_unsolved = true,
            "-h" | "--help" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            s if s.starts_with('-') => {
                eprintln!("solventc: unknown option `{s}`\n\n{USAGE}");
                return ExitCode::from(2);
            }
            s => paths.push(s.to_string()),
        }
    }
    if opts.gltf.is_some() && paths.len() != 1 {
        eprintln!("solventc: --gltf writes one file, so it takes one document");
        return ExitCode::from(2);
    }
    if opts.stl.is_some() && paths.len() != 1 {
        eprintln!("solventc: --stl writes one file, so it takes one document");
        return ExitCode::from(2);
    }
    if opts.output.is_some() && paths.len() != 1 {
        eprintln!("solventc: --output writes one file, so it takes one document");
        return ExitCode::from(2);
    }
    if paths.is_empty() {
        eprint!("{USAGE}");
        return ExitCode::from(2);
    }

    let mut sources: Vec<Source> = Vec::new();
    for p in &paths {
        match std::fs::read_to_string(p) {
            Ok(text) => sources.push(Source { name: p.clone(), text }),
            Err(e) => {
                eprintln!("solventc: {p}: {e}");
                return ExitCode::from(1);
            }
        }
    }

    // the exit code is the worst of them, and every document is reported on whatever the others
    // did: a run over a directory should say everything it found, not stop at the first fault
    let mut worst = 0u8;
    let mut docs: Vec<Json> = Vec::new();
    for s in &sources {
        let (code, doc) = check(s, &opts);
        worst = worst.max(code);
        docs.extend(doc);
    }
    if opts.json {
        println!("{}", json::object([("documents", Json::Arr(docs))]).dump(Some(2)));
    }
    ExitCode::from(worst)
}

/// One document: its exit code, and its JSON report when one was asked for.
fn check(s: &Source, opts: &Opts) -> (u8, Option<Json>) {
    let (mut prog, errs) = parse(&s.text);
    // `use engine.crank` is `engine/crank.sv` beside the document, and failing that the module
    // library compiled into the core — the one place a working directory enters the core's work
    let dir = std::path::Path::new(&s.name).parent().map(|d| d.to_path_buf()).unwrap_or_default();
    let mut resolve = |name: &str| -> Option<String> {
        let rel = format!("{}.sv", name.replace('.', "/"));
        std::fs::read_to_string(dir.join(rel)).ok().or_else(|| gcs_core::library::resolve(name))
    };
    let linked = gcs_core::modules::link(&mut prog, &mut resolve);
    let mut e = elaborate(&prog);
    // a module's diagnostics are the document's to hear, before the elaboration's
    let mut diags = linked;
    diags.append(&mut e.diags);
    e.diags = diags;
    // Nothing is said in `--json` mode, so nothing is worked out either: the collapse below is
    // part of deciding what to print, and running it to feed a function that discards every line
    // is work done for no reader.
    if !opts.json {
        // a syntax error is a diagnostic like any other; it just has no `Code` of its own yet
        for x in &errs {
            say(s, x.span.lo, "error", "E001", &x.message);
        }
        // the same reference can be reported by expansion and again by the build that follows
        // it; saying it twice is a repetition and not a second finding, so identical lines
        // collapse.  The *wording* is still the core's — this is paging, like `SHOW` below.
        let mut said: std::collections::BTreeSet<(u32, &str, &str)> = Default::default();
        for d in &e.diags {
            let (sev, code) = (severity(d.severity()), d.code.as_str());
            if said.insert((d.span.lo, code, &d.message)) {
                say(s, d.span.lo, sev, code, &d.message);
            }
        }
    }
    if !errs.is_empty() || !e.ok() {
        if !opts.json {
            println!("{}: did not elaborate", s.name);
        }
        return (1, opts.json.then(|| doc_json(s, None, None, &e, Json::Null)));
    }

    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    let invalid = gcs_core::program::solid_diagnostics(&sk, &e.map);
    if !invalid.is_empty() {
        if !opts.json {
            for d in &invalid { say(s, d.span.lo, "error", d.code.as_str(), &d.message); }
        }
        e.diags.extend(invalid);
        return (1, opts.json.then(|| doc_json(s, Some(&r), None, &e, Json::Null)));
    }
    let d = (!opts.no_diagnose).then(|| diagnose(&mut sk, DiagnoseOptions::default()));

    if !opts.json {
        println!("{}: {}", s.name, if r.success { "solved" } else { "did not solve" });
        if !r.success {
            println!("  {} (max residual {:.3e})", r.message, r.max_residual);
        }
        if let Some(d) = &d {
            println!("  {}", gcs_core::diagnose::summary(d));
            report_set(&sk, &e.map, "conflict", d.conflicts.as_deref().unwrap_or(&[]));
            report_set(&sk, &e.map, "over", &d.over);
            report_set(&sk, &e.map, "implied", &d.implied);
            report_set(&sk, &e.map, "claim refuted", &d.claims_violated);
            report_set(&sk, &e.map, "claim independent", &d.claims_consuming);
            // the claims about solids, in the core's own words: what was asked, what was
            // measured, and — for one the faceting cannot decide — that it could not
            for v in &d.solid_claims {
                let sampling = if v.samples > 0 {
                    format!(", sampling {} poses ({} failed)", v.samples, v.failed_samples.len())
                } else {
                    String::new()
                };
                if v.samples > 0 && v.failed_samples.len() == v.samples {
                    println!("  {} — undecided: no solved valid poses{}", v.text, sampling);
                    continue;
                }
                let at = match v.worst {
                    Some(w) => format!(", worst at {}", gcs_core::syntax::num(w)),
                    None => String::new(),
                } + &sampling;
                let m = gcs_core::io::reading(SpecKind::Length, v.measured);
                match v.holds {
                    Some(true) => println!("  {} — holds, measured {m}{at}", v.text),
                    Some(false) => println!("  {} — refuted, measured {m}{at}", v.text),
                    None => println!(
                        "  {} — undecided: measured {m}, and the faceting is good to {}{at}",
                        v.text,
                        gcs_core::io::reading(SpecKind::Length, v.tolerance)
                    ),
                }
            }
            for w in &d.warnings {
                println!("  note: {w}");
            }
        }
        for q in &opts.wanted {
            let rows = wanted(&sk, &e.map, std::slice::from_ref(q));
            if rows.is_empty() {
                println!("  where {q}: the source names nothing there");
            }
            for (n, v) in rows {
                // a position is one of an entity's own numbers, which is `SpecKind::Scalar`
                // exactly — so it is read to `READING_SIG` like every other number a person is
                // shown, and converted by nothing: it is already in the units of the drawing
                println!("  {n} = {}", io::reading(SpecKind::Scalar, v));
            }
        }
    }
    let mut code = if r.success || opts.allow_unsolved { 0 } else { 2 };
    if let Some(path) = &opts.output {
        // the writer is `gcs_core::svg`, not this crate's: an "export SVG" button in the web app
        // must not be a second implementation, the same reason callout layout is in the core
        if let Err(err) = std::fs::write(path, gcs_core::svg::render(&sk, opts.width)) {
            eprintln!("solventc: {path}: {err}");
            code = 1;
        }
    }
    if let Some(path) = &opts.stl {
        // the mesh is `gcs_core::mesh`'s for `svg`'s reason: a printer's file and a drawing are
        // two readings of one boundary, and a second walk would be a second object
        match pick_solid(&sk, opts.solid.as_deref()) {
            Ok(i) => {
                // cut to the *object*, not to the report: a printer resolves a tenth of a
                // millimetre and a volume is quoted to four digits, and those are not one number
                match sk.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Mesh).and_then(|s| s.stl()) {
                    Ok(bytes) => if let Err(err) = std::fs::write(path, bytes) {
                        eprintln!("solventc: {path}: {err}");
                        code = 1;
                    },
                    Err(message) => {
                        let site = e.map.site_of(gcs_core::model::EntRef::solid(i));
                        e.diags.push(gcs_core::program::Diag {
                            code: gcs_core::program::Code::E080,
                            span: site.map(|s| s.span).unwrap_or_default(),
                            stmt: site.map(|s| s.stmt), message: message.clone(),
                        });
                        if !opts.json { eprintln!("solventc: {message}"); }
                        code = 1;
                    }
                }
            }
            Err(m) => {
                eprintln!("solventc: {m}");
                code = 1;
            }
        }
    }
    if let Some(path) = &opts.gltf {
        // a named solid, or **every object the document has** — glTF holds a scene, so unlike an
        // STL it need not be told which one part of an assembly to be
        let which: Vec<usize> = match &opts.solid {
            Some(_) => match pick_solid(&sk, opts.solid.as_deref()) {
                Ok(i) => vec![i],
                Err(m) => {
                    eprintln!("solventc: {m}");
                    code = 1;
                    Vec::new()
                }
            },
            None => gcs_core::overview::objects(&sk),
        };
        if which.is_empty() && opts.solid.is_none() {
            eprintln!("solventc: this document has no solid to write");
            code = 1;
        } else if !which.is_empty() {
            match gcs_core::gltf::checked_glb(&sk, &which, gcs_core::solid::ApproximationPolicy::Mesh) {
                Ok(bytes) => if let Err(err) = std::fs::write(path, bytes) {
                    eprintln!("solventc: {path}: {err}"); code = 1;
                },
                Err(message) => { eprintln!("solventc: {message}"); code = 1; }
            }
        }
    }
    let positions = opts.json.then(|| {
        Json::Obj(
            wanted(&sk, &e.map, &opts.wanted)
                .into_iter()
                .map(|(n, v)| (n, Json::Num(v)))
                .collect(),
        )
    });
    (
        code,
        opts.json.then(|| {
            doc_json(s, Some(&r), d.as_ref().map(|d| (&sk, d)), &e, positions.unwrap_or(Json::Null))
        }),
    )
}

/// How many culprits a set prints before it says how many more there are.  A conflict on a truss
/// can name every member of it; the wording is the core's, the paging is the terminal's.
const SHOW: usize = 8;

/// The constraints in one of the diagnosis's sets, worded by the core and named as the source
/// names them — `over: corner distance(60) along`, so the reader can find the statement.
fn report_set(sk: &Sketch, map: &gcs_core::program::SourceMap, what: &str, ids: &[u32]) {
    for id in ids.iter().take(SHOW) {
        if let Some(c) = sk.constraint(*id) {
            println!("  {what}: {}", io::describe_with(c, &|e| map.name_of(e).cloned()));
        }
    }
    if ids.len() > SHOW {
        println!("  {what}: … and {} more", ids.len() - SHOW);
    }
}

/// The rows of `report::positions` a reader asked for: a name matches its own numbers
/// (`o` gives `o.x`, `o.y`) and everything written under it (`views` gives the whole view,
/// `views.right_origin.x` gives the one number).  Three questions, one rule, since a scalar, an
/// entity and an instance are all just names with dots in them.  Empty asks for everything.
fn wanted(
    sk: &Sketch,
    map: &gcs_core::program::SourceMap,
    names: &[String],
) -> Vec<(String, f64)> {
    let all = report::positions(sk, map);
    if names.is_empty() {
        return all;
    }
    all.into_iter()
        .filter(|(n, _)| {
            names.iter().any(|q| n == q || n.strip_prefix(q.as_str()).is_some_and(|r| r.starts_with('.')))
        })
        .collect()
}

/// `file:line:col: error[Exxx]: message`.
///
/// Offsets cross from the core in **UTF-8 bytes** and a column counts characters, which
/// `syntax::line_col` already knows — `gear.sv` has an em dash in its second line, so this is the
/// ordinary case and not a corner one.
fn say(s: &Source, off: u32, sev: &str, code: &str, msg: &str) {
    let (line, col) = line_col(&s.text, off);
    eprintln!("{}:{line}:{col}: {sev}[{code}]: {msg}", s.name);
}

fn severity(s: Severity) -> &'static str {
    match s {
        Severity::Error => "error",
        Severity::Warning => "warning",
        Severity::Note => "note",
    }
}

/// Which solid `--stl` writes.  Named, or the only one there is — a document with one part in it
/// should not have to say which part.
fn pick_solid(sk: &gcs_core::model::Sketch, name: Option<&str>) -> Result<usize, String> {
    match name {
        Some(n) => sk
            .solids
            .iter()
            .position(|s| s.name == n)
            .ok_or_else(|| format!("no solid called `{n}`")),
        None if sk.solids.len() == 1 => Ok(0),
        None if sk.solids.is_empty() => Err("this document has no solid to write".into()),
        None => {
            let names: Vec<&str> = sk.solids.iter().map(|s| s.name.as_str()).collect();
            Err(format!("say which solid with --solid: {}", names.join(", ")))
        }
    }
}

/// One document's report.  `positions` is where the names landed (`--where`, or all of them),
/// and is `Null` for a document that never got as far as a drawing.
fn doc_json(
    s: &Source,
    r: Option<&gcs_core::solve::SolveResult>,
    d: Option<(&Sketch, &gcs_core::diagnose::Diagnosis)>,
    e: &Elaborated,
    positions: Json,
) -> Json {
    let diags: Vec<Json> = e
        .diags
        .iter()
        .map(|x| {
            let (line, col) = line_col(&s.text, x.span.lo);
            json::object([
                ("code", Json::Str(x.code.as_str().to_string())),
                ("severity", Json::Str(severity(x.severity()).to_string())),
                ("line", Json::Int(line as i64)),
                ("column", Json::Int(col as i64)),
                ("message", Json::Str(x.message.clone())),
            ])
        })
        .collect();
    json::object([
        ("name", Json::Str(s.name.clone())),
        ("diagnostics", Json::Arr(diags)),
        ("solve", r.map(report::solve_result_json).unwrap_or(Json::Null)),
        ("diagnosis", d.map(|(sk, d)| report::diagnosis_json(sk, d)).unwrap_or(Json::Null)),
        ("positions", positions),
    ])
}
