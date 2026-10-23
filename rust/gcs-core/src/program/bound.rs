//! Each solid a closed instance makes, stamped with its binding (#566): the key its instance's
//! binding gives it (`flatten::Bound`) with its own path in the body, the frame the instance
//! stands in, and the turn of the `ring` copy it is in.  `gcs-solve` reads the stamp and nothing
//! else of the language (`model::Binding`): solids of one key are one evaluation placed.

use super::resolve::Resolver;
use crate::flatten::{InstanceInfo, RingInfo};
use crate::model::{Binding, EntKind, Sketch};

/// Stamp every solid made under a closed instance — the innermost one it is made in — once every
/// solid is built and the body rule has joined its tools.
pub(super) fn stamp(sk: &mut Sketch, res: &Resolver, instances: &[InstanceInfo], rings: &[RingInfo]) {
    let mut closed: Vec<&InstanceInfo> = instances.iter().filter(|i| i.binding.is_some()).collect();
    if closed.is_empty() {
        return;
    }
    // innermost first: the longest prefix a name starts with is the instance that made it
    closed.sort_by_key(|i| std::cmp::Reverse(i.prefix.len()));
    let mut seen: std::collections::BTreeMap<(String, String), u32> = Default::default();
    for si in 0..sk.solids.len() {
        let name = &sk.solids[si].name;
        let Some(inst) = closed.iter().find(|i| name.starts_with(&i.prefix)) else { continue };
        let Some(bound) = inst.binding.as_ref() else { continue };
        let frame = match &bound.frame {
            Some(f) => match res.of.get(f) {
                Some(e) if e.kind == EntKind::Frame => Some(e.i() as u32),
                _ => continue,
            },
            None => None,
        };
        // a copy of a ring's body past the first stands in its frame turned; one in two rings'
        // copies (a ring in an instance in a ring) is its own
        let turns: Vec<_> = rings.iter().filter_map(|r| {
            let (k, _) = inst.prefix.strip_prefix(&r.prefix)?.split_once('.')?;
            Some((r, k.parse::<u32>().ok()?))
        }).filter(|&(_, k)| k > 0).collect();
        let turn = match turns.as_slice() {
            [] => None,
            [(r, k)] => match res.entity(sk, &r.about) {
                Some(about) => Some((about, *k, r.n)),
                None => continue,
            },
            _ => continue,
        };
        // its path in the body, and which of the solids so named it is (a body and the stock
        // the body rule moved out from under its name share one)
        let path = name[inst.prefix.len()..].to_string();
        let nth = seen.entry((inst.prefix.clone(), path.clone())).or_default();
        let key = format!("{}\u{1e}{path}\u{1e}{nth}", bound.key);
        *nth += 1;
        sk.solids[si].binding = Some(Binding { key, at: inst.prefix.clone(), frame, turn });
    }
}
