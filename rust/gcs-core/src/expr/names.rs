//! What a text's names are: the one scanner the lexer reads a name by, and the flattener's
//! rewriting of a text (`flatten::values::substitute_with`, `map_measured`) reads names by too,
//! so a rewrite finds the names the lexer will read.  Where a reader differs from the lexer it
//! says how, in a `NameScan`, rather than in a lexer of its own.

use super::MEASURES;

/// How a reader takes a text's names apart (`name_at`).  `LEXER` is the expression lexer's own
/// reading; the flattener's rewriting differs from it only where a field says so.
#[derive(Clone, Copy, Debug)]
pub struct NameScan {
    /// A letter past ASCII begins and continues a name.  The lexer refuses one (`unexpected`),
    /// so a rewrite reading it as a name rewrites nothing the lexer would read.
    pub unicode: bool,
    /// A name may begin with a block copy's key (`#3.0.w`), whose digits after a dot are then
    /// the name's (`3.0` alone is a number).
    pub keys: bool,
    /// A copy's key may stand inside a path (`side.#282.0.small`, the absolute name a
    /// measurement's argument resolves to).
    pub inner_keys: bool,
    /// A name may begin right after a name character or a dot: where the lexer has read a number
    /// first (`80mm`, its unit) or a path broke off (`a.5b`).  A reader that asks only after a
    /// word standing on its own (`map_measured`) says not.
    pub glued: bool,
}

/// The expression lexer's reading (`tokenize`): ASCII names, a copy's key at the head of a
/// name and inside a path.  The lexer reads a number before it asks for a name, so a name never
/// begins inside one.
pub const LEXER: NameScan = NameScan { unicode: false, keys: true, inner_keys: true, glued: true };

/// Where the name starting at `i` ends, or `None` where none starts there: a word (`w`), a
/// dotted path, one name however many dots (`c.center.x`; a dot only between name characters,
/// so `a.5` and a trailing `a.` are left alone), or a copy's key (`#3.0.w`) — read as `scan`
/// says.
pub fn name_at(b: &[char], i: usize, scan: NameScan) -> Option<usize> {
    let ident = |c: char| {
        c == '_' || if scan.unicode { c.is_alphanumeric() } else { c.is_ascii_alphanumeric() }
    };
    let key_at = |j: usize| b.get(j) == Some(&'#') && b.get(j + 1).is_some_and(|&c| ident(c));
    let c = *b.get(i)?;
    if !scan.glued && i > 0 && (ident(b[i - 1]) || b[i - 1] == '.') {
        return None;
    }
    let mut key = scan.keys && key_at(i);
    let letter = if scan.unicode { c.is_alphabetic() } else { c.is_ascii_alphabetic() };
    if !(letter || c == '_' || key) {
        return None;
    }
    let mut i = i + key as usize;
    loop {
        while b.get(i).is_some_and(|&c| ident(c)) {
            i += 1;
        }
        // a dot followed by a letter carries the path on; by a digit only within a copy's key
        // (`#3.0.w`), and by a key where a reader takes one inside a path
        let inner = scan.inner_keys && key_at(i + 1);
        let on = b.get(i + 1).is_some_and(|&c| ident(c) && (key || !c.is_ascii_digit()));
        if b.get(i) != Some(&'.') || !(on || inner) {
            return Some(i);
        }
        i += 1;
        if inner {
            key = true;
            i += 1;
        }
    }
}

/// Where a measurement call opens and closes when `word`, ending at `i`, names one: its `(`
/// after any space, and its `)` (the text's end where it is unclosed).
pub fn measure_call(b: &[char], word: &str, i: usize) -> Option<(usize, usize)> {
    if !MEASURES.iter().any(|m| m.0 == word) {
        return None;
    }
    let j = i + b[i..].iter().take_while(|c| c.is_whitespace()).count();
    if b.get(j) != Some(&'(') {
        return None;
    }
    Some((j, b[j..].iter().position(|&c| c == ')').map_or(b.len(), |k| j + k)))
}
