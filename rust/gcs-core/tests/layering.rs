//! The layering gate (#409, step 0): the solver's modules do not import the language's.  The
//! solver is usable without Solvent at run time; this holds the build to the same line, one
//! `crate::` path at a time, until the crate is split and the compiler holds it instead.
//!
//! The edges left are listed in `ALLOWED`, and the list is a ratchet: a new edge fails the gate,
//! and so does an edge removed but still listed, so the list only shrinks.  It is empty (#409,
//! step 6), and the reach is gated too: no route from the solver through the modules between
//! (the curves, the solids, the B-rep) arrives at the language.

use std::collections::{BTreeMap, VecDeque};
use std::path::Path;

/// The constraint solve: the model, its constraints and kernels, the solvers, diagnosis and
/// decomposition, and the leaves they stand on.
const SOLVER: &[&str] = &[
    "model", "constraints", "kernels", "system", "newton", "linalg", "sparse", "graph", "cgraph",
    "diagnose", "decompose", "witness", "solve", "homotopy", "rng", "fmath", "graft", "vocab",
];

/// The language and what is built on it: the parser, elaboration, modules, presentation, the
/// case library and the reports.
const LANGUAGE: &[&str] = &[
    "syntax", "flatten", "program", "modules", "edit", "style", "library", "examples",
    "overview", "callout", "svg", "drawing", "report", "io",
];

/// The edges still standing, as (file under `src/`, language module, how many paths name it).
/// None since step 6: `objects` is the model's and `public_path` the vocabulary's.
const ALLOWED: &[(&str, &str, usize)] = &[];

#[test]
fn the_solver_does_not_import_the_language() {
    let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    for m in SOLVER.iter().chain(LANGUAGE) {
        assert!(is_module(&src, m), "no module `{m}` in src/: rename it here");
    }
    let mut found: BTreeMap<(String, &str), Vec<usize>> = BTreeMap::new();
    for file in rust_files(&src) {
        let rel = file.strip_prefix(&src).unwrap().to_str().unwrap().replace('\\', "/");
        let top = rel.split('/').next().unwrap().trim_end_matches(".rs");
        if !SOLVER.contains(&top) {
            continue;
        }
        let text = std::fs::read_to_string(&file).unwrap();
        for (name, line) in crate_paths(&text) {
            if let Some(m) = LANGUAGE.iter().find(|&&m| m == name) {
                found.entry((rel.clone(), m)).or_default().push(line);
            }
        }
    }
    let mut wrong = Vec::new();
    for ((file, m), lines) in &found {
        let allowed = ALLOWED.iter().find(|a| a.0 == file && a.1 == *m).map_or(0, |a| a.2);
        if lines.len() > allowed {
            wrong.push(format!("{file} imports `{m}` at lines {lines:?}: {} new", lines.len()
                - allowed));
        }
    }
    for &(file, m, n) in ALLOWED {
        let have = found.get(&(file.to_string(), m)).map_or(0, |l| l.len());
        if have < n {
            let fix = if have == 0 { "delete it from ALLOWED" } else { "lower ALLOWED" };
            wrong.push(format!("{file} names `{m}` {have} times, not {n}: {fix}"));
        }
    }
    assert!(wrong.is_empty(), "the layering gate (#409) and the solver's imports disagree:\n{}",
        wrong.join("\n"));
}

/// What the solver reaches through every module in between is not the language either: a solver
/// module importing one that imports the parser links the parser as surely as importing it
/// itself.  A failure names the shortest route, a `file:line` for each hop.  The B-rep is on
/// the way (`brep/mod.rs`): the reach reads a module written either way.
#[test]
fn the_solver_does_not_reach_the_language() {
    let names = imports(&Path::new(env!("CARGO_MANIFEST_DIR")).join("src"));
    let from = reached(&names, SOLVER);
    assert!(from.contains_key("brep"), "the reach no longer passes the B-rep: read `is_module`");
    let wrong: Vec<String> = LANGUAGE.iter().filter(|m| from.contains_key(**m))
        .map(|m| route(&names, &from, m)).collect();
    assert!(wrong.is_empty(), "the solver reaches the language (#409):\n{}", wrong.join("\n"));
}

/// The reach follows a route of any length, and names each hop where it is written.
#[test]
fn reached_follows_a_route_through_the_modules_between() {
    let mut names: Imports = BTreeMap::new();
    let mut import = |a: &str, b: &str, at: &str| {
        names.entry(a.to_string()).or_default().insert(b.to_string(), at.to_string());
    };
    import("model", "solid", "model.rs:3");
    import("solid", "program", "solid/family.rs:7");
    import("program", "syntax", "program.rs:1");
    import("syntax", "model", "syntax.rs:2");
    let from = reached(&names, &["model"]);
    assert_eq!(from.keys().collect::<Vec<_>>(), ["model", "program", "solid", "syntax"]);
    assert_eq!(route(&names, &from, "program"),
        "model → solid (model.rs:3), solid → program (solid/family.rs:7)");
}

/// The scanner reads the forms the gate depends on: a path, a group, a nested group, a rename,
/// and nothing in a comment.
#[test]
fn crate_paths_reads_groups_and_skips_comments() {
    let text = "use crate::style::Sheet;\n\
                use crate::{vocab::Cmp, overview::{self, eye}, io as out};\n\
                // crate::syntax in a comment\n\
                /* crate::program\n in a block */\n\
                fn f() { crate::report::json(); }\n";
    let expected = [("style", 1), ("vocab", 2), ("overview", 2), ("io", 2), ("report", 6)];
    assert_eq!(crate_paths(text), expected.map(|(n, l)| (n.to_string(), l)));
}

/// Each top-level module's imports of the others, with where the first is written.
type Imports = BTreeMap<String, BTreeMap<String, String>>;

fn imports(src: &Path) -> Imports {
    let mut names: Imports = BTreeMap::new();
    for file in rust_files(src) {
        let rel = file.strip_prefix(src).unwrap().to_str().unwrap().replace('\\', "/");
        let top = rel.split('/').next().unwrap().trim_end_matches(".rs").to_string();
        for (name, line) in crate_paths(&std::fs::read_to_string(&file).unwrap()) {
            if name != top && is_module(src, &name) {
                names.entry(top.clone()).or_default().entry(name)
                    .or_insert_with(|| format!("{rel}:{line}"));
            }
        }
    }
    names
}

/// Whether `name` is a top-level module of the crate, written as `name.rs` or as `name/mod.rs`
/// (`brep`, `renderer`): a route through either is a route.
fn is_module(src: &Path, name: &str) -> bool {
    src.join(format!("{name}.rs")).is_file() || src.join(name).join("mod.rs").is_file()
}

/// Every module reached from `start`, breadth first, with the one it was reached from (`None`
/// for a start): so the route back is a shortest one.
fn reached(names: &Imports, start: &[&str]) -> BTreeMap<String, Option<String>> {
    let mut from: BTreeMap<String, Option<String>> =
        start.iter().map(|m| (m.to_string(), None)).collect();
    let mut queue: VecDeque<String> = start.iter().map(|m| m.to_string()).collect();
    while let Some(m) = queue.pop_front() {
        for next in names.get(&m).into_iter().flat_map(|n| n.keys()) {
            if !from.contains_key(next) {
                from.insert(next.clone(), Some(m.clone()));
                queue.push_back(next.clone());
            }
        }
    }
    from
}

/// The route `reached` found to `m`, hop by hop.
fn route(names: &Imports, from: &BTreeMap<String, Option<String>>, m: &str) -> String {
    let mut path = vec![m.to_string()];
    while let Some(Some(prev)) = from.get(path.last().unwrap()) {
        path.push(prev.clone());
    }
    path.reverse();
    path.windows(2).map(|w| format!("{} → {} ({})", w[0], w[1], names[&w[0]][&w[1]]))
        .collect::<Vec<_>>().join(", ")
}

fn rust_files(dir: &Path) -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            out.extend(rust_files(&path));
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
    out.sort();
    out
}

/// The top-level module each `crate::` path names, with its line: `crate::a::b` names `a`, and
/// `crate::{a::b, c}` names `a` and `c`.  Comments are blanked first, keeping their newlines.
fn crate_paths(text: &str) -> Vec<(String, usize)> {
    let code = without_comments(text);
    let mut out = Vec::new();
    for (at, _) in code.match_indices("crate::") {
        if code[..at].ends_with(|c: char| c.is_alphanumeric() || c == '_') {
            continue;
        }
        let line = 1 + code[..at].matches('\n').count();
        let rest = &code[at + "crate::".len()..];
        let Some(group) = rest.strip_prefix('{') else {
            out.push((ident(rest).to_string(), line));
            continue;
        };
        let (mut depth, mut items) = (0, vec![0]);
        for (i, c) in group.char_indices() {
            match c {
                '{' => depth += 1,
                '}' if depth == 0 => break,
                '}' => depth -= 1,
                ',' if depth == 0 => items.push(i + 1),
                _ => {}
            }
        }
        out.extend(items.into_iter().map(|s| ident(group[s..].trim_start()))
            .filter(|name| !name.is_empty()).map(|name| (name.to_string(), line)));
    }
    out
}

fn ident(s: &str) -> &str {
    &s[..s.find(|c: char| !(c.is_alphanumeric() || c == '_')).unwrap_or(s.len())]
}

/// `text` with every `//` and `/* */` comment replaced by spaces, newlines kept, so offsets
/// and lines still match.  A `//` inside a string is taken for a comment too, which can only
/// hide a path written after it on the same line.
fn without_comments(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while !rest.is_empty() {
        let line = rest.find("//");
        let block = rest.find("/*");
        let (at, end) = match (line, block) {
            (Some(l), b) if b.is_none_or(|b| l < b) => (l, l + rest[l..].find('\n')
                .unwrap_or(rest.len() - l)),
            (_, Some(b)) => (b, b + 2 + rest[b + 2..].find("*/").map_or(rest.len() - b - 2,
                |e| e + 2)),
            _ => (rest.len(), rest.len()),
        };
        out.push_str(&rest[..at]);
        out.extend(rest[at..end].chars().map(|c| if c == '\n' { '\n' } else { ' ' }));
        rest = &rest[end..];
    }
    out
}
