//! **A union that adds its body nothing, said** (#961, W119): `solid::unkept` reads which unions
//! of a body leave it as it would be without them while its cuts take some of them, or its
//! commons leave some out; each is said at the statement that wrote the union, naming the union,
//! the body, the terms whose material is all the body keeps of it, and the statements that take
//! the rest, as the document calls them.

use super::refused::Placing;
use super::{Code, Diag, SourceMap};
use crate::model::{SolidDef, Sketch};

/// `a`, `a` and `b`, `a`, `b` and `c`: solids as the document calls them.
fn listed(sk: &Sketch, of: &[usize]) -> String {
    let named: Vec<String> = of.iter().map(|&i| format!("`{}`", sk.solid_name(i))).collect();
    match named.split_last() {
        Some((last, [])) => last.clone(),
        Some((last, rest)) => format!("{} and {last}", rest.join(", ")),
        None => String::new(),
    }
}

/// Each union that adds a body nothing while its cuts or commons take some of it, at the
/// statement that wrote it.  A body whose terms are not
/// built exactly says nothing here: what refuses it is said where it is built.
pub(super) fn unions(sk: &Sketch, map: &SourceMap, place: &Placing) -> Vec<Diag> {
    let mut out = Vec::new();
    for body in 0..sk.solids.len() {
        if !matches!(sk.solids[body].def, SolidDef::Body { .. }) {
            continue;
        }
        let Ok(unkept) = crate::solid::unkept(sk, body) else { continue };
        for u in unkept {
            let Some(site) = map.joined.get(&(body, u.union)) else { continue };
            let (span, stmt) = place.site(Some(site));
            let (b, x) = (sk.solid_name(body), sk.solid_name(u.union));
            let mut whence = Vec::new();
            if !u.cuts.is_empty() {
                let verb = if u.cuts.len() == 1 { "cuts" } else { "cut" };
                whence.push(format!("in what {} {verb} from `{b}`", listed(sk, &u.cuts)));
            }
            if !u.commons.is_empty() {
                whence.push(format!("outside {}, which `{b}` is in `common` with",
                                    listed(sk, &u.commons)));
            }
            // the word whose order misleads: a cut where one takes it, else a common
            let after = if u.cuts.is_empty() {
                "a `union` written after a `common` is kept within it too"
            } else {
                "a `union` written after a cut puts nothing back"
            };
            let tail = format!("a body is its stock and what is in `union` with it, less what \
                cuts it and within what it is in `common` with, whatever order its statements \
                stand in — {after}");
            let whence = whence.join(" or ");
            // clear of every other term, it is wholly taken; else what it keeps is another's
            let message = if u.held.is_empty() {
                format!("`{x}` lies wholly {whence}, so `{b}` keeps none of it: {tail}")
            } else {
                let give = if u.held.len() == 1 { "gives" } else { "give" };
                format!("`{b}` keeps of `{x}` only what {} {give} it already, and the rest of \
                         `{x}` lies {whence}, so `{b}` is as it would be without `{x}`: {tail}",
                        listed(sk, &u.held))
            };
            out.push(Diag::new(Code::W119, span, stmt, message));
        }
    }
    out
}
