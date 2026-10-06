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

pub use read::{accurate,elaborate,unsolved,read,module,beside,read_beside,solid,examples};

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
