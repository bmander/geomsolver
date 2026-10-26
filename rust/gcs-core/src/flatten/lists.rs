//! **A block over a list's items** (§12.10): `repeat h in holes { … }`, or the list written in
//! place, `repeat h in [(0mm, 0mm), (4mm, 2mm)] { … }` — a copy per item, in order, `h` that
//! copy's item: a number, or a group of numbers read by member (`h.x`, `h.d`).
//!
//! A list is a group whose members are its items by place (`holes.0.x`, `syntax::is_list`), so
//! a named one is worked out in dependency order, imported from a module and read inside a body
//! as any group of numbers is, and the numbers in force are where its items are found.  What is
//! here is reading them off that table, and working out a list written in place.

use super::table::{Table, Vals};
use super::Walk;
use crate::expr::Aff;
use crate::syntax::{ref_text, Block, InstArg, InstVal, Name, Ref, Seg, Span};
use crate::units::Units;
use std::collections::BTreeMap;

/// One copy's item: its numbers, each under what it is read as past the copy's name — `""` for
/// an item that is a number, `.x` for a vector's first component, `.at.x` for a group's vector.
pub(super) type Item = Vec<(String, Aff)>;

impl Walk<'_> {
    /// The items a block runs over, where it runs over a list, and what each copy calls its item:
    /// `None` where it runs over none — a count, or a chain, a face or a solid, which are set
    /// aside to be found once every name is known — and the first item that cannot be worked
    /// out, at the item.
    pub(super) fn list_of<'b>(&self, b: &'b Block, vals: &Vals)
        -> Option<Result<(&'b Name, Vec<Item>), (Span, String)>> {
        if let Some(items) = &b.items {
            return Some(written(&items.list, vals, self.units).map(|l| (&items.var, l)));
        }
        let over = b.over.as_ref().filter(|o| o.zip.is_empty() && b.image.is_none())?;
        named(&plain(&over.chain)?, vals).map(|l| Ok((&over.var, l)))
    }
}

/// A reference that is a name and its fields, with no index: what a list may be named by.
fn plain(r: &Ref) -> Option<String> {
    r.path.iter().all(|s| matches!(s, Seg::Field(_))).then(|| ref_text(r))
}

/// The items of the list `name` names among the numbers in force, by place — `None` where what
/// is under the name is not a list's (a group's members, or nothing).  An item whose definition
/// failed was said where it is written; the block then makes no copy.
fn named(name: &str, vals: &Vals) -> Option<Vec<Item>> {
    let prefix = format!("{name}.");
    let mut by_place: BTreeMap<usize, Item> = BTreeMap::new();
    for (key, v) in vals.under(&prefix) {
        let rest = &key[prefix.len()..];
        let (place, member) = match rest.find('.') {
            Some(at) => rest.split_at(at),
            None => (rest, ""),
        };
        if place.is_empty() || !place.bytes().all(|c| c.is_ascii_digit()) {
            return None;
        }
        by_place.entry(place.parse().ok()?).or_default().push((member.to_string(), v));
    }
    if by_place.is_empty() {
        return None;
    }
    let whole = by_place.keys().enumerate().all(|(k, &p)| k == p);
    Some(if whole { by_place.into_values().collect() } else { Vec::new() })
}

/// The items of a list written in place, worked out over the numbers in force.
fn written(list: &[InstArg], vals: &Vals, units: Units) -> Result<Vec<Item>, (Span, String)> {
    list.iter().map(|f| {
        let mut item = Vec::new();
        value(&f.value, "", f.span, vals, units, &mut item)?;
        Ok(item)
    }).collect()
}

/// One item, or one member of it, read as `member` past the copy's name.
fn value(
    v: &InstVal,
    member: &str,
    span: Span,
    vals: &Vals,
    units: Units,
    out: &mut Item,
) -> Result<(), (Span, String)> {
    let number = |t: &str| super::values::value_aff(t, vals, units)
        .map_err(|e| (span, format!("`{t}`: {e}")));
    match v {
        InstVal::Expr(t) => out.push((member.to_string(), number(t)?)),
        // a name: a number, or a group of numbers whose members the item is
        InstVal::Ref(r) => {
            let t = ref_text(r);
            let under = vals.under(&format!("{t}."));
            if vals.has(&t) || under.is_empty() {
                out.push((member.to_string(), number(&t)?));
            } else {
                out.extend(under.into_iter().map(|(k, a)| (format!("{member}{}", &k[t.len()..]), a)));
            }
        }
        InstVal::Group(fields) => {
            for f in fields {
                let label = f.label.as_ref().map_or("", |l| l.text.as_str());
                value(&f.value, &format!("{member}.{label}"), f.span, vals, units, out)?;
            }
        }
        InstVal::Hint(..) => {
            return Err((span, "a list's items are values, and a `hint` seeds an unknown".into()))
        }
    }
    Ok(())
}
