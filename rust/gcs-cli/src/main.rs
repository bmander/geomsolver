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
    --no-diagnose       solve only; skip the diagnosis
    --allow-unsolved    a document that does not solve is not a failure
    --output PATH       write an SVG (one file, so one document)
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
    no_diagnose: bool,
    allow_unsolved: bool,
    output: Option<String>,
    /// An SVG has no screen, so the export must choose a `unit` — the world length of one screen
    /// pixel, which every constant size goes through.  A page width fixes it.
    width: f64,
}

impl Default for Opts {
    fn default() -> Opts {
        Opts { json: false, no_diagnose: false, allow_unsolved: false, output: None, width: 800.0 }
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
        if opts.json {
            docs.push(doc);
        }
    }
    if opts.json {
        println!("{}", json::object([("documents", Json::Arr(docs))]).dump(Some(2)));
    }
    ExitCode::from(worst)
}

/// One document: its exit code, and its JSON report when one was asked for.
fn check(s: &Source, opts: &Opts) -> (u8, Json) {
    let (prog, errs) = parse(&s.text);
    // a syntax error is a diagnostic like any other; it just has no `Code` of its own yet
    for e in &errs {
        say(s, e.span.lo, "error", "E001", &e.message, opts.json);
    }
    let e = elaborate(&prog);
    // the same reference can be reported by expansion and again by the build that follows it;
    // saying it twice is a repetition and not a second finding, so identical lines collapse.
    // The *wording* is still the core's — this is paging, like `SHOW` below.
    let mut said: std::collections::BTreeSet<(u32, &str, &str)> = std::collections::BTreeSet::new();
    for d in &e.diags {
        let (sev, code) = (severity(d.severity()), d.code.as_str());
        if said.insert((d.span.lo, code, &d.message)) {
            say(s, d.span.lo, sev, code, &d.message, opts.json);
        }
    }
    if !errs.is_empty() || !e.ok() {
        if !opts.json {
            println!("{}: did not elaborate", s.name);
        }
        return (1, doc_json(s, None, None, &e));
    }

    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    let d = (!opts.no_diagnose).then(|| diagnose(&mut sk, DiagnoseOptions::default()));

    if !opts.json {
        println!("{}: {}", s.name, if r.success { "solved" } else { "did not solve" });
        if !r.success {
            println!("  {} (max residual {:.3e})", r.message, r.max_residual);
        }
        if let Some(d) = &d {
            println!("  {}", gcs_core::diagnose::summary(d));
            report_set(&sk, "conflict", d.conflicts.as_deref().unwrap_or(&[]));
            report_set(&sk, "over", &d.over);
            report_set(&sk, "implied", &d.implied);
            report_set(&sk, "claim refuted", &d.claims_violated);
            report_set(&sk, "claim independent", &d.claims_consuming);
            for w in &d.warnings {
                println!("  note: {w}");
            }
        }
    }
    if let Some(path) = &opts.output {
        // the writer is `gcs_core::svg`, not this crate's: an "export SVG" button in the web app
        // must not be a second implementation, the same reason callout layout is in the core
        if let Err(err) = std::fs::write(path, gcs_core::svg::render(&sk, opts.width)) {
            eprintln!("solventc: {path}: {err}");
            return (1, doc_json(s, Some(&r), d.as_ref().map(|d| (&sk, d)), &e));
        }
    }
    let code = if r.success || opts.allow_unsolved { 0 } else { 2 };
    (code, doc_json(s, Some(&r), d.as_ref().map(|d| (&sk, d)), &e))
}

/// How many culprits a set prints before it says how many more there are.  A conflict on a truss
/// can name every member of it; the wording is the core's, the paging is the terminal's.
const SHOW: usize = 8;

/// The constraints in one of the diagnosis's sets, named the way the core names them.
fn report_set(sk: &Sketch, what: &str, ids: &[u32]) {
    for id in ids.iter().take(SHOW) {
        if let Some(c) = sk.constraint(*id) {
            println!("  {what}: {}", io::describe(c));
        }
    }
    if ids.len() > SHOW {
        println!("  {what}: … and {} more", ids.len() - SHOW);
    }
}

/// `file:line:col: error[Exxx]: message`.
///
/// Offsets cross from the core in **UTF-8 bytes** and a column counts characters, which
/// `syntax::line_col` already knows — `gear.sv` has an em dash in its second line, so this is the
/// ordinary case and not a corner one.
fn say(s: &Source, off: u32, sev: &str, code: &str, msg: &str, quiet: bool) {
    if quiet {
        return;
    }
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

fn doc_json(
    s: &Source,
    r: Option<&gcs_core::solve::SolveResult>,
    d: Option<(&Sketch, &gcs_core::diagnose::Diagnosis)>,
    e: &Elaborated,
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
    ])
}
