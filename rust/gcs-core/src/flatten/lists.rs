//! **A block over a list's items** (§12.10): `repeat h in holes { … }`, or the list written in
//! place, `repeat h in [(0mm, 0mm), (4mm, 2mm)] { … }` — a copy per item, in order, `h` that
//! copy's item: a number, or a group of numbers read by member (`h.x`, `h.d`).
//!
//! A list is a group whose members are its items by place (`holes.0.x`, `syntax::is_list`), so
//! a named one is worked out in dependency order, imported from a module and read inside a body
//! as any group of numbers is, and the numbers in force are where its items are found.  What is
//! here is reading them off that table, and working out a list written in place.
//!
//! **A list written in place may name what is drawn** (#882): `repeat e in [box.near, box.far]`
//! makes a copy per name, `e` an alias to it in that copy, as a walk over a solid's faces or a
//! chain's edges binds its name — a face, an edge, a point, whatever the name stands for.  Its
//! items are all values or all names: one of each is E103 at the first that is not the first's
//! kind.  A list of names defined has no numbers here: its members are aliases, and the block
//! over it is set aside and run over them as a walk is (`resolve::listed_links`).

use super::table::{Table, Vals};
use super::Walk;
use crate::expr::Aff;
use crate::syntax::{ref_text, Block, InstArg, InstVal, Name, Ref, Seg, Span};
use crate::units::Units;
use std::collections::BTreeMap;

/// One copy's item: its numbers, each under what it is read as past the copy's name — `""` for
/// an item that is a number, `.x` for a vector's first component, `.at.x` for a group's vector.
pub(super) type Item = Vec<(String, Aff)>;

/// What a list's copies are each given: an item of numbers, or a name of what is drawn, which
/// the copy's name is an alias to.
pub(super) enum Items<'b> {
    Values(Vec<Item>),
    Names(Vec<&'b Ref>),
}

impl Items<'_> {
    pub(super) fn len(&self) -> usize {
        match self {
            Items::Values(v) => v.len(),
            Items::Names(n) => n.len(),
        }
    }
}

impl Walk<'_> {
    /// The items a block runs over, where it runs over a list, and what each copy calls its item:
    /// `None` where it runs over none — a count, or a chain, a face or a solid, which are set
    /// aside to be found once every name is known — and the first item that cannot be worked
    /// out, at the item.
    pub(super) fn list_of<'b>(&self, b: &'b Block, vals: &Vals)
        -> Option<Result<(&'b Name, Items<'b>), (Span, String)>> {
        if let Some(items) = &b.items {
            return Some(written(&items.list, vals, self.units).map(|l| (&items.var, l)));
        }
        let over = b.over.as_ref().filter(|o| o.zip.is_empty() && b.image.is_none())?;
        named(&plain(&over.chain)?, vals).map(|l| Ok((&over.var, Items::Values(l))))
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
        let (place, member) = rest.find('.').map_or((rest, ""), |at| rest.split_at(at));
        let place: usize = place.parse().ok()?;
        by_place.entry(place).or_default().push((member.to_string(), v));
    }
    if by_place.is_empty() {
        return None;
    }
    let whole = by_place.keys().enumerate().all(|(k, &p)| k == p);
    Some(if whole { by_place.into_values().collect() } else { Vec::new() })
}

/// The items of a list written in place: worked out over the numbers in force, or — where the
/// first is a name that is no value — each a name of what is drawn (#882).
fn written<'b>(list: &'b [InstArg], vals: &Vals, units: Units)
    -> Result<Items<'b>, (Span, String)> {
    let named = |f: &'b InstArg| match &f.value {
        InstVal::Ref(r) => (!is_value(r, vals, units)).then_some(r),
        _ => None,
    };
    if let Some(first) = list.first().and_then(named) {
        let mut names = vec![first];
        for f in &list[1..] {
            match named(f) {
                Some(r) => names.push(r),
                None => {
                    let what = match &f.value {
                        InstVal::Ref(r) => format!("`{}`", ref_text(r)),
                        InstVal::Expr(t) => format!("`{t}`"),
                        _ => "this item".to_string(),
                    };
                    return Err((f.span, format!(
                        "{what} is a value, and `{}` before it names what is drawn: a list's items \
                         are all values (`[2mm, 3mm]`) or all names (`[box.near, box.far]`)",
                        ref_text(first))));
                }
            }
        }
        return Ok(Items::Names(names));
    }
    list.iter().map(|f| {
        let mut item = Vec::new();
        value(&f.value, "", f.span, vals, units, &mut item).map_err(|(at, m)| match &f.value {
            // a name after a value: perhaps meant as one of a list of names
            InstVal::Ref(_) => (at, format!("{m}; a list's items are all values (`[2mm, 3mm]`) \
                or all names of what is drawn (`[box.near, box.far]`), and the first is a value")),
            _ => (at, m),
        })?;
        Ok(item)
    }).collect::<Result<_, _>>().map(Items::Values)
}

/// Whether a name read as an item is a value: a number, or a group of numbers, in force.
fn is_value(r: &Ref, vals: &Vals, units: Units) -> bool {
    let t = ref_text(r);
    vals.has(&t) || !vals.under(&format!("{t}.")).is_empty()
        || super::values::value_aff(&t, vals, units).is_ok()
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
                let rename = |(k, a): (String, Aff)| (format!("{member}{}", &k[t.len()..]), a);
                out.extend(under.into_iter().map(rename));
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
