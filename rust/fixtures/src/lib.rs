//! Test fixtures shared by the core's and the CLI's suites, written once: `read` elaborates a
//! document (or the example corpus), `tools` and `motions` hold the Solvent snippets a small
//! swept case is written from, `gear` rewrites and reads the spiral-bevel project at a
//! design.  A dev-dependency only, so none of it reaches a released artefact.
pub mod read;
pub mod tools;
pub mod motions;
pub mod gear;
pub mod drill;
pub mod wankel;

pub use read::{accurate,elaborate,unsolved,read,module,beside,read_beside,solid,examples,example,
    example_names};

/// A generated document's zero ordinates said as what they are: `p level(v) P`, `a horizontal b`.
/// A generator writes each coordinate as `p distance({v}mm, along: v) P`, and a number written as
/// zero is refused (`docs/ordinate-plan.md`): the two are level, which states no number.
pub fn levelled(src: String) -> String {
    let mut out: String = src.lines().map(|l| level_line(l).unwrap_or_else(|| l.to_string()))
        .collect::<Vec<_>>().join("\n");
    if src.ends_with('\n') {
        out.push('\n');
    }
    out
}

/// A composed document's body words imported (#783 P3): `cut`, `union` and `common` are words of
/// `std.solids`, named where a file uses them, and a fixture is written from snippets each of
/// which says what it needs — so the document they make says it once, `use std.solids (…)` with
/// every name its statements use and any its snippets named, where the first `use std.solids`
/// stood (else after `use std`).  A document using none is left as it is.
pub fn imported(src: &str) -> String {
    const WORDS: [&str; 3] = ["cut","union","common"];
    let reference = |t: &str| !t.is_empty()
        && t.chars().all(|c| c.is_alphanumeric() || "_.[]+-".contains(c));
    let said = |l: &str| -> Option<&'static str> {
        let l = l.split("//").next()?.trim().trim_start_matches("private ").trim();
        match l.split_whitespace().collect::<Vec<_>>().as_slice() {
            [a,w,b] if reference(a) && reference(b) => WORDS.iter().copied().find(|x| x == w),
            _ => None,
        }
    };
    let mut names: Vec<String> = src.lines().filter_map(said).map(str::to_string).collect();
    let solids = |l: &str| {
        let l = l.trim_start();
        l.starts_with("use std.solids") && !l.starts_with("use std.solids.")
    };
    let uses: Vec<&str> = src.lines().filter(|l| solids(l)).collect();
    for u in &uses {
        if let Some((_,list)) = u.split_once('(') {
            let list = list.trim_end().trim_end_matches(')');
            names.extend(list.split(',').map(|n| n.trim().to_string()));
        }
    }
    names.retain(|n| !n.is_empty());
    let mut listed: Vec<String> = Vec::new();
    for w in WORDS.iter().map(|w| w.to_string()).chain(names.iter().cloned()) {
        if names.contains(&w) && !listed.contains(&w) {
            listed.push(w);
        }
    }
    if listed.is_empty() {
        return src.to_string();
    }
    let line = format!("use std.solids ({})",listed.join(", "));
    let mut out: Vec<String> = Vec::new();
    let mut placed = false;
    for l in src.lines() {
        if solids(l) {
            if !placed {
                out.push(line.clone());
                placed = true;
            }
            continue;
        }
        out.push(l.to_string());
        if !placed && uses.is_empty() && l.trim() == "use std" {
            out.push(line.clone());
            placed = true;
        }
    }
    let mut text = out.join("\n");
    if src.ends_with('\n') {
        text.push('\n');
    }
    text
}

fn level_line(l: &str) -> Option<String> {
    let (a,rest) = l.split_once(" distance(")?;
    let (n,rest) = rest.split_once(", along: ")?;
    let (w,b) = rest.split_once(") ")?;
    if n.trim().trim_end_matches("mm").parse::<f64>().ok()? != 0.0 {
        return None;
    }
    Some(match w {
        "u" | "v" => format!("{a} level({w}) {b}"),
        "y" | "up" | "down" => format!("{a} horizontal {b}"),
        "x" | "right" | "left" => format!("{a} vertical {b}"),
        _ => return None,
    })
}
