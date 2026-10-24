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
mod cad;
mod picture;
mod png;
mod watch;

use gcs_core::constraints::SpecKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::json::Json;
use gcs_core::model::Sketch;
use gcs_core::program::{elaborate, Elaborated, Severity};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::{line_col, parse, Program, SynErr};
use gcs_core::{expr, io, json, progress, report};

// A released binary is stamped with its tag (`.github/workflows/release.yml` sets the variable);
// a local build says the crate's version.
const VERSION: &str = match option_env!("SOLVENTC_VERSION") {
    Some(v) => v,
    None => env!("CARGO_PKG_VERSION"),
};

const SYNOPSIS: &str = "solventc [OPTIONS] FILE...";

const USAGE: &str = "\
solventc — check a Solvent document

    solventc [OPTIONS] FILE...

Each FILE is a model (.sv: geometry, constraints, hints and claims) or a drawing (.svd: sheets,
views and dimensions over the models it imports).  Each is parsed, elaborated, solved and
diagnosed, and the report says where it stands.

    --json              structured output instead of the text report
    --where NAME        where a name landed: its own numbers and everything under it, and a
                        value the document names, in its unit (repeatable; the whole table in
                        --json when none is given)
    --no-diagnose       solve only; skip the diagnosis
    --lint              also say where the document's own text drifts from the language's style
                        (W201-W206: a pun, a bare component statement, an asserting component, a
                        deep path, a presentation flag, a group threaded unread), and (W207) a
                        part whose root its seeds choose, read at the solved pose; advisory, so
                        never a failure
    --allow-unsolved    a document that does not solve is not a failure
    --progress          say each stage on stderr as it is done: the solve, each solid built and
                        each claim judged, with its time; work still going is said again past
                        5 s.  On a terminal, unasked, the stages of a second or more are said
    --no-progress       say nothing on stderr as the run goes, even on a terminal
    --timings           at the end, the slowest solids and claims, each with its statement
    --components        what each component is against the contract for a reusable solve
                        boundary (#452), read through its first instance: asserts only, leaves
                        freedoms, locally determined, branch chosen, and whether it reads the
                        world.  Advisory: nothing is refused on its word
    -o, --output PATH   write an SVG (one file, so one document)
    --stl PATH          write a solid as binary STL (one document; repeatable, each with its own
                        --solid: every output of a run reads the one build of a solid)
    --step PATH         write an analytic STEP solid (by the kernel --kernel names); beside an
                        --stl with no --solid between them, the same solid's
    --kernel NAME       rust (default) or occt: which kernel builds a solid's STEP and STL (rust,
                        the core's own B-rep, a swept body of the generating class included;
                        occt, where the build has it, kept as the oracle)
    --verify-step HOW   light (default): the written file parsed and checked against the solid,
                        its topology, surfaces and their numbers; full: that, and read back by
                        the kernel, repaired as a reader would, checked and measured
    --stl-backend NAME  occt, mesh or refine; every solid defaults to the kernel's export (the
                        core's, or occt as --kernel says), a body with swept cuts included (mesh is
                        the faceted kernel; refine meshes the material field by the core's Delaunay
                        refinement, checked by the field-agreement probe)
    --gltf PATH         write a solid as binary glTF: every face a named node
    --png PATH          draw what the workspace shows as a PNG (a part file: its preview), each
                        object shaded in its instance's colour, depth-buffered, with its creases
                        and silhouettes, a swept one from its exact build where the kernel
                        builds it; --solid draws one solid or one instance's objects.
                        Repeatable, each with its own --view, --size and --hide
    --view VIEW         the eye --png draws from: home (default, as the workspace opens), a view
                        a sheet names (front, back, right, left, top, bottom, isometric), or
                        az,el in degrees
    --size PX           the PNG's longer side in pixels (default 1200)
    --hide NAMES        objects --png leaves out, each a name and everything under it, comma
                        separated (repeatable): a cutaway
    --solid NAME        which solid to export; defaults to the only Boolean root
    --tolerance [LENGTH] export for fabrication: the native STEP/STL within LENGTH of the exact
                        surface (default 0.01mm; a bare number is in the document's unit, and
                        mm, um, cm, m, in and thou name their own); a swept body's sheets are
                        refined into it, the STL meshed within it, and --measure judges by it.
                        Without it the native export keeps its gross bars (no fabrication claim)
    --measure PATH      measure an exported STEP or STL against the solid's exact surface: the
                        solid this run wrote it from, else the one --solid gives before every
                        output (repeatable; STEP requires a native OCCT build)
    --measure-samples N about how many points to measure a file at (default 20000)
    --width PX          the SVG's page width in pixels (default 800)
    --sheet NAME        select a sheet in a .svd drawing (its page size sets SVG size)
    -h, --help          this
    -V, --version       the version of this build (a release says its tag)

--solid, --view, --size and --hide qualify an output: given after one, the last before them that
they qualify (--solid any output, the rest a --png); given before every output, each output told
none of its own.  An option that takes one value is refused given twice, and so is an output
given twice to one path.

Examples:
    solventc part.sv                          check a document: solved, and how constrained
    solventc --json part.sv                   the same report, structured
    solventc part.sv --where hinge            the numbers under one name
    solventc drive.sv --where torque          a value, in its unit: torque = 0.0864 N·m
    solventc part.sv --png part.png           look at it, as the workspace shows it
    solventc part.sv --stl part.stl --solid body
                                              a solid, for a printer
    solventc bot.sv --stl wheel.stl --solid wheel --stl frame.stl --solid frame
                                              two solids, each built once
    solventc part.sv --png top.png --view top --png iso.png --view isometric
                                              two pictures, the solids built once
    solventc drawing.svd --sheet front -o front.svg
                                              one sheet of a drawing, as SVG

Exit codes: 0 every document elaborated and solved; 1 a document failed to parse or
elaborate, or an output was refused or left a solid out (a picture is still written, each solid
missing from it named); 2 a document elaborated but did not solve; 64 the command line was
wrong (an unknown option, an option missing its value, no document).

The language is in the primer: solvent-primer.md beside an installed solventc, or
docs/solvent-primer.md in the repository (https://github.com/bmander/geomsolver/tree/main/docs);
exports, drawings and the app are in docs/usage.md there.
";

/// The exit code of a command line that could not be read (`EX_USAGE`), kept apart from 1 and 2,
/// which say what a document is.
const MISUSE: u8 = 64;

/// Refuse a command line: what was wrong, the synopsis and where the options are, never the whole
/// of `USAGE`, which would push the one line that matters off the screen.
fn misuse(why: impl std::fmt::Display) -> ExitCode {
    eprintln!("solventc: {why}\nusage: {SYNOPSIS}\n`solventc --help` says every option.");
    ExitCode::from(MISUSE)
}

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
    /// `--lint`: the drift lint's warnings (`program::drift`) beside elaboration's.
    lint: bool,
    allow_unsolved: bool,
    output: Option<String>,
    /// What the run writes of its solids, in the order the command line gives it (#592): every
    /// one reads the sketch the run solved, so a solid two of them read is built once.
    outputs: Vec<Output>,
    /// The qualifiers given before every output: each output's where it is told none of its own.
    run: Qualifiers,
    native_stl: bool,
    /// `--stl-backend refine`: the core's Delaunay refinement of the material field.
    refine: bool,
    /// `--measure PATH` — exported files to measure against the solid's exact surface.
    measure: Vec<String>,
    measure_samples: usize,
    /// `--tolerance` — the export's stated tolerance: a value and the millimetres its unit is
    /// worth, `None` for the document's own unit (known only once the document is read).
    tolerance: Option<(f64,Option<f64>)>,
    /// An SVG has no screen, so the export must choose a `unit` — the world length of one screen
    /// pixel, which every constant size goes through.  A page width fixes it.
    width: f64,
    sheet: Option<String>,
    /// `--progress`/`--no-progress`, or what a terminal is told unasked.
    say: watch::Say,
    /// `--timings`: the slowest solids and claims, said at the end.
    timings: bool,
    /// `--components`: each component classified against the boundary contract (#452).
    components: bool,
}

impl Default for Opts {
    fn default() -> Opts {
        Opts {
            json: false,
            wanted: Vec::new(),
            no_diagnose: false,
            lint: false,
            allow_unsolved: false,
            output: None,
            outputs: Vec::new(),
            run: Qualifiers::default(),
            native_stl: true,
            refine: false,
            measure: Vec::new(),
            measure_samples: 20000,
            tolerance: None,
            width: 800.0,
            sheet: None,
            say: watch::Say::Nothing,
            timings: false,
            components: false,
        }
    }
}

/// What an output may be told beside its path (#592): which solid, and for a picture its eye, its
/// size and what it leaves out.
#[derive(Default)]
struct Qualifiers {
    solid: Option<String>,
    view: Option<picture::Eye>,
    size: Option<usize>,
    hide: Option<Vec<String>>,
}

/// What one output writes.
enum Writes {
    /// `--stl PATH` and `--step PATH`, one solid built once for both: the output of a drawing
    /// that is not a picture, and the reason a printer can be given a part at all.
    Export { stl: Option<String>, step: Option<String> },
    /// `--gltf PATH` — the object as a viewer opens it, every face named.
    Gltf(String),
    /// `--png PATH` — the objects as the workspace shows them, drawn without a page (#541), from
    /// `--view`'s eye at `--size`, less `--hide`'s.
    Png(String),
}

/// One file a run writes from its solids, and what it was told of its own.
struct Output {
    writes: Writes,
    own: Qualifiers,
}

impl Output {
    fn new(writes: Writes) -> Output {
        Output { writes, own: Qualifiers::default() }
    }

    /// The files it writes.
    fn paths(&self) -> Vec<&str> {
        match &self.writes {
            Writes::Export { stl, step } => stl.iter().chain(step).map(String::as_str).collect(),
            Writes::Gltf(p) | Writes::Png(p) => vec![p.as_str()],
        }
    }
}

impl Opts {
    /// The qualifiers a qualifier given now sets, and how a refusal says which: the last output
    /// it qualifies (a picture's alone where `png`), else the run's.
    fn qualified(&mut self, png: bool) -> (&mut Qualifiers, String) {
        match self.outputs.iter_mut().rev().find(|o| !png || matches!(o.writes, Writes::Png(_))) {
            Some(o) => {
                let at = format!("for `{}`", o.paths()[0]);
                (&mut o.own, at)
            }
            None => (&mut self.run, "before every output".into()),
        }
    }

    /// The solid `o` writes: its own, else the run's.
    fn solid<'a>(&'a self, o: &'a Qualifiers) -> Option<&'a str> {
        o.solid.as_deref().or(self.run.solid.as_deref())
    }

    /// The solid `--measure PATH` is measured against: the one this run wrote `path` from, else
    /// the run's.
    fn measured(&self, path: &str) -> Option<&str> {
        self.outputs.iter().find(|o| o.paths().contains(&path))
            .map_or(self.run.solid.as_deref(), |o| self.solid(&o.own))
    }

    /// Whether any output writes a picture.
    fn pictured(&self) -> bool {
        self.outputs.iter().any(|o| matches!(o.writes, Writes::Png(_)))
    }

    /// `--stl PATH` or `--step PATH`: beside the export before it where that writes the other
    /// format and was told no solid of its own since (`--stl a.stl --step a.step --solid body`),
    /// else an export of its own.
    fn add_export(&mut self, stl: Option<String>, step: Option<String>) {
        let last = self.outputs.last_mut();
        if let Some(Output { writes: Writes::Export { stl: s, step: t }, own }) = last {
            let beside = stl.is_some() && s.is_none() || step.is_some() && t.is_none();
            if own.solid.is_none() && beside {
                if stl.is_some() { *s = stl; } else { *t = step; }
                return;
            }
        }
        self.outputs.push(Output::new(Writes::Export { stl, step }));
    }
}

/// `value` for a qualifier's `slot`, refused where `flag` was given it already (`at` says whose).
fn qualify<T>(slot: &mut Option<T>, value: T, flag: &str, at: &str) -> Result<(), String> {
    match slot.replace(value) {
        Some(_) => Err(format!("`{flag}` given twice {at}")),
        None => Ok(()),
    }
}

/// The options that take one value a run, whatever its outputs: each said once (#592), a second
/// refused, never taken over the first.
const ONCE: [&str; 8] = ["--stl-backend", "--kernel", "--verify-step", "--sheet", "--output",
                         "--width", "--tolerance", "--measure-samples"];

fn main() -> ExitCode {
    let mut opts = Opts::default();
    let mut given: std::collections::BTreeSet<&str> = Default::default();
    let mut paths: Vec<String> = Vec::new();
    let mut said: Option<watch::Say> = None;
    let mut args = std::env::args().skip(1).peekable();
    while let Some(a) = args.next() {
        let flag = if a == "-o" { "--output" } else { a.as_str() };
        if let Some(&once) = ONCE.iter().find(|&&f| f == flag) {
            if !given.insert(once) {
                return misuse(format!("`{once}` given twice; it takes one value a run"));
            }
        }
        match a.as_str() {
            "--stl-backend" => match args.next().as_deref() {
                Some("occt") => { opts.native_stl = true; cad::ask_occt(); }
                Some("mesh") => opts.native_stl = false,
                Some("refine") => opts.refine = true,
                _ => return misuse("--stl-backend needs occt, mesh or refine"),
            },
            "--step" => match args.next() {
                Some(p) => opts.add_export(None, Some(p)),
                None => return misuse("--step needs a path"),
            },
            "--kernel" => match args.next().as_deref() {
                Some("occt") => cad::ask_occt(),
                Some("rust") => { cad::use_rust_kernel(); opts.native_stl = true; }
                _ => return misuse("--kernel needs occt or rust"),
            },
            "--verify-step" => match args.next().as_deref() {
                Some("light") => {}
                Some("full") => cad::verify_step_fully(),
                _ => return misuse("--verify-step needs light or full"),
            },
            "--sheet" => match args.next() {
                Some(n) => opts.sheet = Some(n),
                None => return misuse("--sheet needs a name"),
            },
            "--output" | "-o" => match args.next() {
                Some(p) => opts.output = Some(p),
                None => return misuse("--output needs a path"),
            },
            "--width" => match args.next().and_then(|v| v.parse::<f64>().ok()) {
                Some(w) if w.is_finite() && w > 0.0 => opts.width = w,
                _ => return misuse("--width needs a page width in pixels"),
            },
            "--gltf" => match args.next() {
                Some(p) => opts.outputs.push(Output::new(Writes::Gltf(p))),
                None => return misuse("--gltf needs a path"),
            },
            "--png" => match args.next() {
                Some(p) => opts.outputs.push(Output::new(Writes::Png(p))),
                None => return misuse("--png needs a path"),
            },
            "--view" => match args.next().as_deref().map(picture::eye) {
                Some(Ok(eye)) => {
                    let (q, at) = opts.qualified(true);
                    if let Err(m) = qualify(&mut q.view, eye, "--view", &at) { return misuse(m); }
                }
                Some(Err(m)) => return misuse(m),
                None => return misuse("--view needs a view"),
            },
            "--size" => match args.next().and_then(|v| v.parse::<usize>().ok()).filter(|n| (16..=4096).contains(n)) {
                Some(n) => {
                    let (q, at) = opts.qualified(true);
                    if let Err(m) = qualify(&mut q.size, n, "--size", &at) { return misuse(m); }
                }
                None => return misuse("--size needs a side in pixels, 16 to 4096"),
            },
            "--hide" => match args.next() {
                Some(n) => {
                    let names = n.split(',').map(str::trim).filter(|n| !n.is_empty());
                    opts.qualified(true).0.hide.get_or_insert_with(Vec::new)
                        .extend(names.map(String::from));
                }
                None => return misuse("--hide needs a name"),
            },
            "--stl" => match args.next() {
                Some(p) => opts.add_export(Some(p), None),
                None => return misuse("--stl needs a path"),
            },
            "--solid" => match args.next() {
                Some(n) => {
                    let (q, at) = opts.qualified(false);
                    if let Err(m) = qualify(&mut q.solid, n, "--solid", &at) { return misuse(m); }
                }
                None => return misuse("--solid needs a name"),
            },
            "--measure" => match args.next() {
                Some(p) => opts.measure.push(p),
                None => return misuse("--measure needs a path"),
            },
            // the value is optional: what follows is taken only when it reads as a length
            "--tolerance" => match args.peek().map(|v| length(v)) {
                Some(Some(Ok(t))) => { args.next(); opts.tolerance = Some(t); }
                Some(Some(Err(m))) => return misuse(format!("--tolerance: {m}")),
                _ => opts.tolerance = Some((gcs_core::solid::export::Tolerance::FABRICATION.millimetres,Some(1.))),
            },
            "--measure-samples" => match args.next().and_then(|v| v.parse::<usize>().ok()).filter(|&n| n > 0) {
                Some(n) => opts.measure_samples = n,
                None => return misuse("--measure-samples needs a positive count"),
            },
            "--where" => match args.next() {
                Some(n) => opts.wanted.push(n),
                None => return misuse("--where needs a name"),
            },
            "--json" => opts.json = true,
            "--no-diagnose" => opts.no_diagnose = true,
            "--lint" => opts.lint = true,
            "--progress" => said = Some(watch::Say::Everything),
            "--no-progress" => said = Some(watch::Say::Nothing),
            "--timings" => opts.timings = true,
            "--components" => opts.components = true,
            "--allow-unsolved" => opts.allow_unsolved = true,
            "-h" | "--help" => {
                print!("{USAGE}");
                return ExitCode::SUCCESS;
            }
            "-V" | "--version" => {
                println!("solventc {VERSION}");
                return ExitCode::SUCCESS;
            }
            s if s.starts_with('-') => return misuse(format!("unknown option `{s}`")),
            s => paths.push(s.to_string()),
        }
    }
    if paths.is_empty() {
        return misuse("no document given");
    }
    if let Some(o) = opts.outputs.first().filter(|_| paths.len() != 1) {
        let flag = match o.writes {
            Writes::Export { .. } => "--stl or --step",
            Writes::Gltf(_) => "--gltf",
            Writes::Png(_) => "--png",
        };
        return misuse(format!("{flag} writes one file, so it takes one document"));
    }
    let native = opts.outputs.iter().any(|o| match &o.writes {
        Writes::Export { stl, step } =>
            step.is_some() || (stl.is_some() && !opts.refine && opts.native_stl),
        _ => false,
    });
    if opts.tolerance.is_some() && opts.measure.is_empty() && !native {
        return misuse("--tolerance holds a native export (or a measurement) to it; give --step, \
            --stl by the kernel's export, or --measure");
    }
    if opts.output.is_some() && paths.len() != 1 {
        return misuse("--output writes one file, so it takes one document");
    }
    // two outputs to one file would leave only the second (#592)
    let mut written: Vec<&str> = opts.output.iter().map(String::as_str).collect();
    for p in opts.outputs.iter().flat_map(Output::paths) {
        if written.contains(&p) {
            return misuse(format!("two outputs write `{p}`"));
        }
        written.push(p);
    }
    // a qualifier given before every output and qualifying none would be dropped
    let run = &opts.run;
    if run.solid.is_some() && opts.outputs.is_empty() && opts.measure.is_empty() {
        return misuse("`--solid` says which solid an output writes, and none is given");
    }
    let qualified = [("--view", run.view.is_some()), ("--size", run.size.is_some()),
                     ("--hide", run.hide.is_some())];
    for (flag, given) in qualified {
        if given && !opts.pictured() {
            return misuse(format!("`{flag}` qualifies a --png, and none is given"));
        }
    }
    // a person watching a terminal is told of slow work unasked; a pipe or a file only on asking
    use std::io::IsTerminal;
    let terminal = std::io::stderr().is_terminal();
    opts.say = said.unwrap_or(if terminal { watch::Say::Slow } else { watch::Say::Nothing });

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
    if std::path::Path::new(&s.name).extension().is_some_and(|e| e == "svd") {
        return check_drawing(s, opts);
    }
    if opts.say == watch::Say::Nothing && !opts.timings {
        return check_model(s, opts, &mut Default::default());
    }
    let watch = watch::Watch::new(opts.say);
    let mut places = Places::new();
    let out = progress::listen(watch.clone(), || check_model(s, opts, &mut places));
    if opts.timings {
        timings(s, opts, &watch.timings.borrow(), &places);
    }
    out
}

/// How many of the slowest solids and claims `--timings` lists.
const TIMINGS: usize = 12;

/// Where each solid and each claim about solids was written, by `progress::At`: the offset of its
/// statement in the document, for those the document itself wrote.
type Places = std::collections::BTreeMap<progress::At, u32>;

/// `--timings`: the core's lines, each at its statement.  Beside the JSON report rather than in
/// it, as `--measure`'s lines are.
fn timings(s: &Source, opts: &Opts, timings: &progress::Timings, places: &Places) {
    let line = |l: String| if opts.json { eprintln!("{l}") } else { println!("{l}") };
    line(format!("{}: {}", s.name, timings.heading()));
    for (text, work) in timings.lines(TIMINGS) {
        match work.at.and_then(|a| places.get(&a)) {
            Some(&lo) => {
                let (l, c) = line_col(&s.text, lo);
                line(format!("  {text}  {}:{l}:{c}", s.name));
            }
            None => line(format!("  {text}")),
        }
    }
}

/// The `Places` of a document: a solid a module's component made is where the document made that
/// instance of it.
fn where_written(prog: &Program, e: &Elaborated) -> Places {
    use gcs_core::ir::PathStep;
    use gcs_core::progress::At;
    let mut places = Places::new();
    for i in 0..e.sketch.solids.len() {
        let Some(site) = e.map.site_of(gcs_core::model::EntRef::solid(i)) else { continue };
        let lo = if prog.document_stmt(site.stmt).is_some() { Some(site.span.lo) } else {
            site.path.0.iter().rev().find_map(|step| match step {
                PathStep::Instance(id) | PathStep::Copy { block: id, .. } =>
                    prog.document_stmt(*id).map(|st| st.span.lo),
            })
        };
        if let Some(lo) = lo { places.insert(At::Solid(i), lo); }
    }
    for c in &e.sketch.solid_claims {
        if let Some(st) = prog.document_stmt(gcs_core::syntax::StmtId(c.stmt())) {
            places.insert(At::Stmt(c.stmt()), st.span.lo);
        }
    }
    places
}

/// Parse, link and elaborate: the program, its syntax errors, and what it elaborated to.
fn read_model(s: &Source) -> (Program, Vec<SynErr>, Elaborated) {
    let (mut prog, errs) = parse(&s.text);
    // Resolve beside the model, then its ancestors, then the compiled module library.
    let dir = std::path::Path::new(&s.name).parent().map(|d| d.to_path_buf()).unwrap_or_default();
    let mut resolve = |name: &str| -> Option<String> {
        gcs_core::modules::search_paths(name, &s.name).iter()
            .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
            .or_else(|| gcs_core::library::resolve(name))
    };
    let linked = gcs_core::modules::link(&mut prog, &mut resolve);
    let mut e = elaborate(&prog);
    // a module's diagnostics are the document's to hear, before the elaboration's
    let mut diags = linked;
    diags.append(&mut e.diags);
    e.diags = diags;
    (prog, errs, e)
}

fn check_model(s: &Source, opts: &Opts, places: &mut Places) -> (u8, Option<Json>) {
    let elaborating = || progress::Work::elaborate(&s.name);
    let (prog, errs, mut e) = progress::during(elaborating, || read_model(s));
    if opts.lint {
        e.diags.extend(gcs_core::program::drift(&prog));
    }
    if opts.timings {
        *places = where_written(&prog, &e);
    }
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
        return (1, opts.json.then(|| {
            doc_json(s, &errs, None, None, &e, &[], Json::Null, Json::Null, &opts.wanted)
        }));
    }

    let mut sk = e.sketch.clone();
    // named from the elaborated sketch, which `sk` is a copy of, so only when somebody listens
    let solving = || progress::Work::solve(&e.sketch);
    let r = progress::during(solving, || solve(&mut sk, SolveOpts::default()));
    // a solve that stopped on its iteration limit is no answer, whatever its residual (#449)
    let solved = r.settled();
    let mut invalid = gcs_core::program::solid_diagnostics(&sk, &e.map, &e.program);
    // read at a pose the solve did not settle on, they are no finding about the document: said
    // under the solve's report, never in place of it, and never the exit (#571)
    let unbuilt = if solved { Vec::new() } else { std::mem::take(&mut invalid) };
    let failed = invalid.iter().any(|d| d.severity() == Severity::Error);
    // a part that leaves its root to its seeds, read at the solved pose (#461 PT5)
    if opts.lint && !failed && unbuilt.is_empty() {
        invalid.extend(gcs_core::parts::roots(&e, &sk));
    }
    if !opts.json {
        for d in &invalid { say(s, d.span.lo, severity(d.severity()), d.code.as_str(), &d.message); }
    }
    e.diags.extend(invalid);
    if failed {
        return (1, opts.json.then(|| {
            doc_json(s, &[], Some(&r), None, &e, &[], Json::Null, Json::Null, &opts.wanted)
        }));
    }
    let diagnosing = || progress::Work::diagnose(&s.name);
    let d = (!opts.no_diagnose)
        .then(|| progress::during(diagnosing, || {
            diagnose(&mut sk, DiagnoseOptions { settled: solved, ..DiagnoseOptions::default() })
        }));
    // read at the solved pose, before anything below builds or exports
    let components = opts.components.then(|| gcs_core::boundary::classify(&e, &sk));

    if !opts.json {
        println!("{}: {}", s.name, r.outcome());
        if !solved {
            println!("  {} (max residual {:.3e})", r.message, r.max_residual);
        }
        // each material the document states, and what is made of it (#510 P3)
        for name in sk.materials.keys() {
            println!("  material {}", gcs_core::report::material_text(&sk, name));
        }
        // each claim over the document's values, judged before the solve (#596)
        for c in &e.value_claims {
            println!("  {}", gcs_core::report::value_claim_text(sk.units, c));
        }
        // what each scenario's supports leave free, and where a stress it asks for is
        // unbounded (#510 P7), then what its frames answer of each requirement (#510 S0): read
        // at the solved pose, a line only where there is something
        if solved {
            for st in &e.scenarios {
                let (words, read) = gcs_core::diagnose::analysis::scenario_read(&sk, &st.scenario);
                for w in words {
                    println!("  {w}");
                }
                for a in gcs_core::report::scenario_answers(&sk, st, &read) {
                    println!("  {}", a.text);
                }
            }
            // and what the drawing states a beam or a solid is made of (#510)
            for a in gcs_core::report::measure_answers(&sk, &e.measures) {
                println!("  {}", a.text);
            }
        }
        if let Some(d) = &d {
            println!("  {}", gcs_core::diagnose::summary(d));
            // a plane's freedoms by the ledger's names; a line only where one is left
            let views = gcs_core::diagnose::view_freedoms(&sk, d);
            if !views.is_empty() {
                println!("  free views: {}", views.join(", "));
            }
            // each declared parameter: what reads it, and how far the drawing runs over it (#391)
            for p in d.parameters.iter().filter(|p| !p.readers.is_empty()) {
                report_set(&sk, &e.map, &format!("driven by {}", p.name), &p.readers);
                println!("  {}", gcs_core::report::parameter_text(&sk, &p.name));
            }
            // a requirement out of reach is the whole conflict, said with why (#510 P7), not as
            // one to remove
            if d.out_of_reach.is_empty() {
                report_set(&sk, &e.map, "conflict", d.conflicts.as_deref().unwrap_or(&[]));
            }
            for (id, why) in &d.out_of_reach {
                if let Some(c) = sk.constraint(*id) {
                    println!("  {}: {why}", io::describe_with(c, &|r| e.map.name_of(r).cloned()));
                }
            }
            report_set(&sk, &e.map, "over", &d.over);
            report_set(&sk, &e.map, "implied", &d.implied);
            report_set(&sk, &e.map, "claim refuted", &d.claims_violated);
            report_set(&sk, &e.map, "claim independent", &d.claims_consuming);
            report_set(&sk, &e.map, "bound broken", &d.bounds_violated);
            // each orientation the document states that the solution breaks (#157)
            for k in &d.orientations_violated {
                let name = |r| e.map.name_of(r).cloned();
                if let Some(text) = gcs_core::report::orientation_text(&sk, k, &name) {
                    println!("  orientation broken: {text}");
                }
            }
            report_set(&sk, &e.map, "degenerate", &d.collapsed);
            // what it says of the document's parts, in their terms (#461 D1–D3)
            for line in gcs_core::parts::text(&gcs_core::parts::read(&e, &sk, d, solved)) {
                println!("  {line}");
            }
            // what each energy came to, in its statement's words (#121)
            for &(id, v) in &d.extrema {
                if let Some(c) = sk.constraint(id) {
                    println!("  {}: {}", v.name(), io::describe_with(c, &|r| e.map.name_of(r).cloned()));
                }
            }
            // the claims about solids, in the core's own words: what was asked, what was
            // measured, and — for one the faceting cannot decide — that it could not
            for v in &d.solid_claims {
                println!("  {}", gcs_core::report::solid_claim_text(v));
            }
            // each meshing's contact over its phase, in the core's words (#283)
            for i in 0..sk.meshings.len() {
                println!("  {}", gcs_core::report::meshing_text(&sk, i));
            }
            // each body traced in space, and whether it is regular over its domain (#326)
            for i in 0..sk.traced.len() {
                println!("  {}", gcs_core::report::traced_text(&sk, i));
            }
            // each curve whose body states a rate: how far it is integrated (#339 D2)
            for i in 0..sk.curves.len() {
                if let Some(text) = gcs_core::report::rates_text(&sk, i) {
                    let r = gcs_core::model::EntRef::new(gcs_core::model::EntKind::Curve, i);
                    let name = e.map.name_of(r).cloned().unwrap_or_else(|| format!("curve {i}"));
                    println!("  {name}: {text}");
                }
            }
            // a solid's failure at an unsolved pose is said once, under `NOT_BUILT` below
            for w in d.warnings.iter().filter(|w| !unbuilt.iter().any(|u| &u.message == *w)) {
                println!("  note: {w}");
            }
        }
        if !unbuilt.is_empty() {
            println!("  {}", gcs_core::program::NOT_BUILT);
            for d in &unbuilt {
                let (line, col) = line_col(&s.text, d.span.lo);
                println!("    {}:{line}:{col}: note[{}]: {}", s.name, d.code.as_str(), d.message);
            }
        }
        for q in &opts.wanted {
            let rows = wanted(&sk, &e.map, std::slice::from_ref(q));
            let values = values_wanted(&e, std::slice::from_ref(q));
            if rows.is_empty() && values.is_empty() {
                println!("  where {q}: the source names nothing there");
            }
            for (n, v) in rows {
                // a position is one of an entity's own numbers, which is `SpecKind::Scalar`
                // exactly — so it is read to `READING_SIG` like every other number a person is
                // shown, and converted by nothing: it is already in the units of the drawing
                println!("  {n} = {}", expr::reading(SpecKind::Scalar, v));
            }
            // and a number the document names, in the unit it reads in (#596)
            for (n, v) in values {
                println!("  {}", report::value_text(sk.units, n, v));
            }
        }
    }
    if !opts.json {
        for line in components.iter().flatten().flat_map(gcs_core::boundary::text) {
            println!("  {line}");
        }
    }
    let mut code = if solved || opts.allow_unsolved { 0 } else { 2 };
    // each export in its turn, then the pictures, of one solved sketch and its memo, so a solid
    // two of them read is built once (#592).  The pictures read a copy of it given what each
    // export built: an export's own build is the same solid as the memo's, its faces named by its
    // recipe, which a picture never reads and every other output does
    let exports: Vec<&Output> =
        opts.outputs.iter().filter(|o| matches!(o.writes, Writes::Export { .. })).collect();
    let seen = (opts.pictured() && !exports.is_empty()).then(|| sk.clone());
    for o in exports {
        if !export(&sk, &mut e, opts, o, seen.as_ref(), solved) {
            code = 1;
        }
    }
    if let Some(path) = &opts.output {
        // the writer is `gcs_core::svg`, not this crate's: an "export SVG" button in the web app
        // must not be a second implementation, the same reason callout layout is in the core
        if let Err(err) = std::fs::write(path, gcs_core::svg::render(&sk, opts.width)) {
            eprintln!("solventc: {path}: {err}");
            code = 1;
        }
    }
    for o in &opts.outputs {
        let solid = opts.solid(&o.own);
        match &o.writes {
            Writes::Export { .. } => {}
            Writes::Gltf(path) => {
                // a named solid, or **every object the document has** — glTF holds a scene, so
                // unlike an STL it need not be told which one part of an assembly to be
                let which: Vec<usize> = match solid {
                    Some(_) => match pick_solid(&sk, solid) {
                        Ok(i) => vec![i],
                        Err(m) => {
                            eprintln!("solventc: {m}");
                            code = 1;
                            Vec::new()
                        }
                    },
                    None => gcs_core::overview::objects(&sk),
                };
                if which.is_empty() && solid.is_none() {
                    eprintln!("solventc: this document has no solid to write");
                    code = 1;
                } else if !which.is_empty() {
                    match gcs_core::gltf::checked_glb(&sk, &which, gcs_core::solid::ApproximationPolicy::Mesh) {
                        Ok(bytes) => if let Err(err) = cad::output::write(path, &bytes) {
                            eprintln!("solventc: {err}"); code = 1;
                        },
                        Err(message) => { eprintln!("solventc: {message}"); code = 1; }
                    }
                }
            }
            Writes::Png(path) => {
                // what the workspace shows, at the pose the run reached: a drawing that did not
                // solve is still looked at, as its SVG is still written
                let (own, run) = (&o.own, &opts.run);
                let hide = own.hide.as_ref().or(run.hide.as_ref()).map_or(&[][..], Vec::as_slice);
                let ask = picture::Ask {
                    eye: own.view.or(run.view).unwrap_or(picture::HOME),
                    size: own.size.or(run.size).unwrap_or(1200),
                    solid,
                    hide,
                };
                match picture::render(seen.as_ref().unwrap_or(&sk), &ask) {
                    Ok((bytes, unbuilt)) => {
                        // an object that does not evaluate is left out, as the box leaves it, and
                        // said against the picture it is missing from: the picture is written, and
                        // the run fails, since it is not the whole of what was asked (#614)
                        for why in &unbuilt { eprintln!("solventc: {path}: {why}"); }
                        if !unbuilt.is_empty() { code = 1; }
                        if let Err(err) = cad::output::write(path, &bytes) {
                            eprintln!("solventc: {err}");
                            code = 1;
                        }
                    }
                    Err(m) => { eprintln!("solventc: {m}"); code = 1; }
                }
            }
        }
    }
    // measured after anything written, so one run may export a file and measure it
    for path in &opts.measure {
        let lines = if solved { pick_solid(&sk,opts.measured(path)).and_then(|i| {
            let t = tolerance(&sk,opts)?.map(|t| t.millimetres);
            cad::measure::report(&sk,i,path,opts.measure_samples,t)
        }) }
            else { Err("measuring an export requires a solved model".into()) };
        match lines {
            Ok(lines) => for l in lines { if opts.json { eprintln!("{l}"); } else { println!("{l}"); } },
            Err(m) => { eprintln!("solventc: {m}"); code = 1; }
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
            let components = components.as_ref()
                .map(|cs| Json::Arr(cs.iter().map(gcs_core::boundary::json).collect()));
            doc_json(s, &[], Some(&r), d.as_ref().map(|d| (&sk, d)), &e, &unbuilt,
                     positions.unwrap_or(Json::Null), components.unwrap_or(Json::Null),
                     &opts.wanted)
        }),
    )
}

/// One export of a solid (`--stl` and `--step`, #592): admitted, built once for both formats and
/// written, each refusal said in the core's words and recorded at the solid's statement; whether
/// all of it was.  `seen`, where the run draws pictures, is the sketch they read, given what it
/// built.
fn export(sk: &Sketch, e: &mut Elaborated, opts: &Opts, o: &Output, seen: Option<&Sketch>,
          solved: bool) -> bool {
    let Writes::Export { stl, step } = &o.writes else { return true };
    let (mut stl, mut step, solid) = (stl.clone(), step.clone(), opts.solid(&o.own));
    let mut ok = true;
    // A body with swept cuts is built only when every sweep is in the generating class
    // (docs/generating-sweeps.md): a refusal names the row it fails and a point where, and
    // nothing is written, so an earlier output stays as it was. The class is what the native
    // boundary construction needs; the core's field refinement (`--stl-backend refine`) needs
    // only the field, and is gated by the field-agreement probe instead.
    // A native export admits the body itself, beside the first of its construction.
    let native = cfg!(feature="occt") && !cad::rust_kernel() && (step.is_some() || (stl.is_some() && opts.native_stl && !opts.refine));
    let mut body = None;
    if (step.is_some() || (stl.is_some() && !opts.refine)) && solved {
        if let Ok(i) = pick_solid(sk,solid) {
            let mut b = cad::Body::read(sk,i);
            if b.swept() && !native {
                if let Err(refusal) = b.admit(sk) {
                    refused(e,i,refusal);
                    ok = false;
                    stl = None;
                    step = None;
                }
            }
            body = Some(b);
        }
    }
    if let Some(path) = stl.clone().filter(|_| opts.refine && solved) {
        #[cfg(feature="occt")]
        if let Ok(i) = pick_solid(sk,solid) {
            if let Err(refusal) = cad::field_mesh::export_refine(sk,i,&path) {
                refused(e,i,refusal);
                ok = false;
            }
        }
        #[cfg(not(feature="occt"))]
        { let _ = path; eprintln!("solventc: --stl-backend refine needs a build with the `occt` feature"); ok = false; }
        stl = None;
    }
    if step.is_some() || (stl.is_some() && opts.native_stl) {
        match (solved,pick_solid(sk,solid)) {
            (false,_) => {
                let message = "native STEP/STL export requires a solved model".to_string();
                eprintln!("solventc: {message}");
                e.diags.push(gcs_core::program::Diag::new(gcs_core::program::Code::E080,
                    Default::default(),None,message));
                ok = false;
            }
            (true,Err(message)) => {
                eprintln!("solventc: {message}");
                e.diags.push(gcs_core::program::Diag::new(gcs_core::program::Code::E080,
                    Default::default(),None,message));
                ok = false;
            }
            (true,Ok(i)) => {
                let body = body.take().filter(|b| b.index == i).unwrap_or_else(|| cad::Body::read(sk,i));
                match tolerance(sk,opts) {
                    Err(message) => { eprintln!("solventc: {message}"); ok = false; }
                    Ok(t) => if let Err(refusal) = cad::export(sk,&body,step.as_deref(),stl.as_deref().filter(|_| opts.native_stl),t,seen) {
                        // refused admission, nothing is written (the mesh backend's STL included)
                        if refusal.stage == cad::Stage::Admission { stl = None; }
                        refused(e,i,refusal);
                        ok = false;
                    },
                }
            }
        }
    }
    if let Some(path) = stl.as_ref().filter(|_| !opts.native_stl) {
        // the mesh is `gcs_core::mesh`'s for `svg`'s reason: a printer's file and a drawing are
        // two readings of one boundary, and a second walk would be a second object
        match pick_solid(sk, solid) {
            Ok(i) => {
                // cut to the *object*, not to the report: a printer resolves a tenth of a
                // millimetre and a volume is quoted to four digits, and those are not one number
                match sk.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Mesh).and_then(|s| s.stl()) {
                    Ok(bytes) => if let Err(err) = cad::output::write(path, &bytes) {
                        eprintln!("solventc: {err}");
                        ok = false;
                    },
                    Err(message) => {
                        let site = e.map.site_of(gcs_core::model::EntRef::solid(i));
                        e.diags.push(gcs_core::program::Diag::new(
                            gcs_core::program::Code::E080,
                            site.map(|s| s.span).unwrap_or_default(),
                            site.map(|s| s.stmt), message.clone(),
                        ));
                        if !opts.json { eprintln!("solventc: {message}"); }
                        ok = false;
                    }
                }
            }
            Err(m) => {
                eprintln!("solventc: {m}");
                ok = false;
            }
        }
    }
    ok
}

fn check_drawing(s: &Source, opts: &Opts) -> (u8, Option<Json>) {
    let result = if !opts.outputs.is_empty() || opts.run.solid.is_some() {
        Err(gcs_core::drawing::Error { span: Default::default(),
            message: "STL/STEP/glTF/PNG export selects geometry from a .sv model, not a .svd drawing".into() })
    } else {
        gcs_core::drawing::compile(&s.text, &s.name, opts.sheet.as_deref(), &mut |path, from| {
            let base = std::path::Path::new(from).parent().unwrap_or(std::path::Path::new("."));
            let p = base.join(path);
            let text = std::fs::read_to_string(&p).ok()?;
            Some((p.canonicalize().ok()?.to_string_lossy().into_owned(), text))
        })
    };
    match result {
        Ok(svg) => {
            if let Some(path) = &opts.output {
                if let Err(e) = std::fs::write(path, svg) {
                    eprintln!("solventc: {path}: {e}"); return (1, None);
                }
            }
            if !opts.json { println!("{}: drawing compiled", s.name); }
            (0, opts.json.then(|| json::object([("file", Json::Str(s.name.clone())),
                ("drawing", Json::Bool(true)), ("ok", Json::Bool(true))])))
        }
        Err(e) => {
            if !opts.json { say(s, e.span.lo, "error", "D001", &e.message); }
            (1, opts.json.then(|| json::object([("file", Json::Str(s.name.clone())),
                ("drawing", Json::Bool(true)), ("ok", Json::Bool(false)),
                ("error", Json::Str(e.message))])))
        }
    }
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
/// (`o` gives `o.x`, `o.y`) and everything written under it (`cyl` gives the whole instance,
/// `cyl.pivot.x` gives the one number).  Three questions, one rule, since a scalar, an
/// entity and an instance are all just names with dots in them.  Empty asks for everything.
fn wanted(
    sk: &Sketch,
    map: &gcs_core::program::SourceMap,
    names: &[String],
) -> Vec<(String, f64)> {
    // a solid is read only where a name asked reaches it, or it reaches a name asked — a swept
    // one from its exact surface where one builds, as a drawing reads it
    let asked = |n: &str| names.is_empty() || names.iter().any(|q| under(n, q) || under(q, n));
    gcs_core::brep::export::supply_exact_where(sk, &|i| asked(&sk.solids[i].name));
    let all = report::positions_where(sk, map, &asked);
    if names.is_empty() {
        return all;
    }
    all.into_iter().filter(|(n, _)| names.iter().any(|q| under(n, q))).collect()
}

/// The numbers the document names (#596) a reader asked for, by the rule `wanted` reads
/// positions by: a name matches itself and everything under it.  Empty asks for every one.
fn values_wanted<'e>(e: &'e Elaborated, names: &[String])
    -> Vec<(&'e String, &'e gcs_core::flatten::Valued)>
{
    e.values.iter().filter(|(n, _)| names.is_empty() || names.iter().any(|q| under(n, q)))
        .collect()
}

/// Whether `name` is `asked` or something under it: `cyl` reaches `cyl.pivot.x`, and `head`
/// reaches `head.casting`.
fn under(name: &str, asked: &str) -> bool {
    name == asked || name.strip_prefix(asked).is_some_and(|r| r.starts_with('.'))
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

/// An export refused: said on stderr in the core's words, recorded at the stage it was met at
/// for a harness, and reported as a diagnostic at the solid's own statement.
fn refused(e: &mut gcs_core::program::Elaborated,solid: usize,refusal: gcs_core::solid::export::ExportRefusal) {
    eprintln!("solventc: {refusal}");
    cad::progress::refused(refusal.stage);
    let site = e.map.site_of(gcs_core::model::EntRef::solid(solid));
    e.diags.push(gcs_core::program::Diag::new(gcs_core::program::Code::E080,
        site.map(|s| s.span).unwrap_or_default(),site.map(|s| s.stmt),refusal.message));
}

/// A length as `--tolerance` takes it: a number, and the millimetres its unit is worth (`None`
/// for none written, the document's own). `None` when the text is no length at all.
fn length(text: &str) -> Option<Result<(f64,Option<f64>),String>> {
    let split = text.find(|c: char| !(c.is_ascii_digit() || c == '.')).unwrap_or(text.len());
    let (number,unit) = text.split_at(split);
    let value: f64 = number.parse().ok()?;
    let per = match unit {
        "" => None,
        "um" | "µm" => Some(1e-3),
        u => match gcs_core::units::unit(u) {
            Some(x) if x.dim == gcs_core::units::Dim::LENGTH => Some(x.per),
            _ => return (!unit.contains('.') && !unit.contains('/')).then(|| Err(format!("`{unit}` is not a length unit"))),
        },
    };
    Some(if value > 0. && value.is_finite() { Ok((value,per)) } else { Err("a tolerance is a positive length".into()) })
}

/// The export tolerance `--tolerance` asked for, in native millimetres: a bare number is in the
/// document's unit.
fn tolerance(sk: &Sketch,opts: &Opts) -> Result<Option<gcs_core::solid::export::Tolerance>,String> {
    let Some((value,per)) = opts.tolerance else { return Ok(None) };
    let per = match per { Some(p) => p, None => gcs_core::solid::cad::millimetres(sk) };
    gcs_core::solid::export::Tolerance::new(value*per).map(Some)
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
        None if sk.solids.is_empty() => Err("this document has no solid to write".into()),
        None => {
            let roots = gcs_core::overview::objects(sk);
            if roots.len() == 1 { return Ok(roots[0]); }
            let names: Vec<&str> = roots.iter().map(|&i| sk.solids[i].name.as_str()).collect();
            Err(format!("say which solid with --solid: {}", names.join(", ")))
        }
    }
}

/// One document's report.  `positions` is where the names landed (`--where`, or all of them),
/// and is `Null` for a document that never got as far as a drawing; `wanted` is the names
/// `--where` asked, which the document's values are read by too (#596).  `unbuilt` is what the
/// solids said of a pose the solve did not settle on (#571), apart from the document's
/// `diagnostics`.
fn doc_json(
    s: &Source,
    syntax: &[SynErr],
    r: Option<&gcs_core::solve::SolveResult>,
    d: Option<(&Sketch, &gcs_core::diagnose::Diagnosis)>,
    e: &Elaborated,
    unbuilt: &[gcs_core::program::Diag],
    positions: Json,
    components: Json,
    wanted: &[String],
) -> Json {
    let place = |lo: u32| {
        let (line, col) = line_col(&s.text, lo);
        [("line", Json::Int(line as i64)), ("column", Json::Int(col as i64))]
    };
    let said = |lo: u32, code: &str, sev: &str, message: &str, cause: Option<u32>| {
        let [line, column] = place(lo);
        let mut d = json::object([
            ("code", Json::Str(code.to_string())),
            ("severity", Json::Str(sev.to_string())),
            line,
            column,
            ("message", Json::Str(message.to_string())),
        ]);
        // where the fault a note follows from is said (#572): the diagnostic it points back at
        if let (Some(at), Json::Obj(fields)) = (cause, &mut d) {
            fields.push(("cause".into(), json::object(place(at))));
        }
        d
    };
    // a syntax error first, as the text report says it (`E001`, no `Code` of its own yet)
    let diags: Vec<Json> = syntax.iter().map(|x| said(x.span.lo, "E001", "error", &x.message, None))
        .chain(e.diags.iter().map(|x| {
            said(x.span.lo, x.code.as_str(), severity(x.severity()), &x.message,
                x.cause.map(|c| c.lo))
        }))
        .collect();
    let mut doc = json::object([
        ("name", Json::Str(s.name.clone())),
        ("diagnostics", Json::Arr(diags)),
        ("solve", r.map(report::solve_result_json).unwrap_or(Json::Null)),
        ("diagnosis", d.map(|(sk, d)| report::diagnosis_json(sk, d)).unwrap_or(Json::Null)),
        ("positions", positions),
        ("meshings", d.map(|(sk, _)| report::meshings_json(sk)).unwrap_or(Json::Null)),
        ("components", components),
    ]);
    if !unbuilt.is_empty() {
        let notes = unbuilt.iter().map(|x| said(x.span.lo, x.code.as_str(), "note", &x.message,
            x.cause.map(|c| c.lo)));
        doc.set("notBuilt", json::object([
            ("why", Json::Str(gcs_core::program::NOT_BUILT.to_string())),
            ("diagnostics", Json::Arr(notes.collect())),
        ]));
    }
    // the numbers the document names, as `--where` asks (or every one), and each claim over
    // them, judged before the solve (#596)
    if !e.values.is_empty() {
        doc.set("values", report::values_json(e.sketch.units, values_wanted(e, wanted)));
    }
    if !e.value_claims.is_empty() {
        doc.set("valueClaims", report::value_claims_json(e.sketch.units, &e.value_claims));
    }
    // each scenario's words (#510 P7) at the solved pose, where the document states any
    let settled = r.is_some_and(|r| r.settled());
    if let Some((sk, _)) = d.filter(|_| settled && !e.scenarios.is_empty()) {
        doc.set("scenarios", report::scenarios_json(sk, &e.scenarios));
    }
    // and the masses and volumes the drawing states (#510), at the solved pose
    if let Some((sk, _)) = d.filter(|_| settled && !e.measures.is_empty()) {
        doc.set("measures", report::measures_json(sk, &e.measures));
    }
    // the document's parts, read through the diagnosis (#461), where it has any
    if let Some((sk, d)) = d {
        let parts = gcs_core::parts::read(e, sk, d, settled);
        if parts.instances > 0 {
            doc.set("parts", gcs_core::parts::json(&parts));
        }
    }
    doc
}

